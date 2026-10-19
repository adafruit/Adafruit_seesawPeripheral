/*!
 * @file Adafruit_seesawPeripheral_stm32.h
 * @brief STM32C011 GPIO/ADC/PWM backend with deferred command execution.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_H

#if CONFIG_NEOPIXEL || CONFIG_UART || CONFIG_ENCODER || CONFIG_EEPROM ||       \
    CONFIG_FHT
#error "The STM32C011 backend currently supports GPIO, ADC, and PWM only"
#endif
#if USE_PINCHANGE_INTERRUPT
#error "The STM32C011 backend polls GPIO changes from run()"
#endif
#if CONFIG_INTERRUPT
static_assert(CONFIG_INTERRUPT_PIN < 32 &&
                  (ALL_GPIO & (1UL << CONFIG_INTERRUPT_PIN)),
              "The interrupt output must use an exposed GPIO pin");
#endif
#if CONFIG_ADDR_0
static_assert(CONFIG_ADDR_0_PIN < 32 && (ALL_GPIO & (1UL << CONFIG_ADDR_0_PIN)),
              "Address strap 0 must use an exposed GPIO pin");
#endif
#if CONFIG_ADDR_1
static_assert(CONFIG_ADDR_1_PIN < 32 && (ALL_GPIO & (1UL << CONFIG_ADDR_1_PIN)),
              "Address strap 1 must use an exposed GPIO pin");
#endif
#if CONFIG_ADDR_2
static_assert(CONFIG_ADDR_2_PIN < 32 && (ALL_GPIO & (1UL << CONFIG_ADDR_2_PIN)),
              "Address strap 2 must use an exposed GPIO pin");
#endif
#if CONFIG_ADDR_3
static_assert(CONFIG_ADDR_3_PIN < 32 && (ALL_GPIO & (1UL << CONFIG_ADDR_3_PIN)),
              "Address strap 3 must use an exposed GPIO pin");
#endif

#if CONFIG_ADC
#include "stm32c0xx_ll_adc.h"
#endif

/**
 * @brief Apply address straps to the base I2C address.
 * @param address Base I2C address.
 * @return Address after applying configured straps.
 */
uint8_t Adafruit_seesawPeripheral_applyAddressStraps(uint8_t address) {
#if CONFIG_ADDR_0
  pinMode(CONFIG_ADDR_0_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_0_PIN) == CONFIG_ADDR_INVERTED)
    address += 1;
#endif
#if CONFIG_ADDR_1
  pinMode(CONFIG_ADDR_1_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_1_PIN) == CONFIG_ADDR_INVERTED)
    address += 2;
#endif
#if CONFIG_ADDR_2
  pinMode(CONFIG_ADDR_2_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_2_PIN) == CONFIG_ADDR_INVERTED)
    address += 4;
#endif
#if CONFIG_ADDR_3
  pinMode(CONFIG_ADDR_3_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_3_PIN) == CONFIG_ADDR_INVERTED)
    address += 8;
#endif
  return address;
}

namespace SeesawSTM32 {
const uint8_t queueSize = 4;    ///< Bounded queue; one entry stays empty.
const uint8_t commandSize = 32; ///< Classic seesaw packet capacity.
struct Command {
  uint8_t length;            ///< Packet length, including module and register.
  uint8_t data[commandSize]; ///< Received bytes.
};
volatile Command commands[queueSize]; ///< Single producer/consumer queue.
volatile uint8_t head = 0, tail = 0;  ///< Queue cursors.
volatile uint8_t selectedBase = 0, selectedRegister = 0; ///< Read pointer.
uint32_t direction = 0, output = 0, pull = 0; ///< GPIO register state.
uint32_t lastGPIO = 0;                        ///< Previous GPIO sample.
uint8_t address = CONFIG_I2C_PERIPH_ADDR;     ///< Effective I2C address.
#if CONFIG_PWM || CONFIG_PWM_16BIT
uint32_t pwmActive = 0; ///< Protocol pins currently using timers.
uint16_t pwmValues[16] =
    {}; ///< Duty settings retained across frequency changes.
uint16_t pwmFrequency = 1000; ///< Shared PWM frequency in hertz.
// Select non-complementary outputs. D3/D9 share TIM1_CH4, and D7/D15
// share TIM3_CH2; use only one pin from each pair at a time.
const uint16_t pwmPins[16] = {PA0_ALT1, PA1_ALT1, 2, PA3_ALT1, PA4_ALT1, 5,  6,
                              PA7_ALT1, PA8_ALT3, 9, 10,       11,       12, 13,
                              14,       15};

/** @brief Restore active PWM outputs after changing their shared frequency. */
void applyPWM() {
  analogWriteFrequency(pwmFrequency);
  for (uint8_t pin = 0; pin < 16; pin++) {
    if (pwmActive & (1UL << pin)) {
      analogWrite(pwmPins[pin], pwmValues[pin]);
    }
  }
}

/** @brief Release one PWM pin without stopping other active timer outputs. */
void stopPWM(uint8_t pin) {
  uint32_t bit = 1UL << pin;
  if (!(pwmActive & bit)) {
    return;
  }
  pwmActive &= ~bit;
  // STM32duino 3.0.0 pinMode() stops the whole HardwareTimer. Use the exact
  // alternate-function pin, then restore the other active channels.
  pinMode(pwmPins[pin], INPUT);
  applyPWM();
}
#endif
#if CONFIG_ADC
bool adcReady = false;                  ///< Calibration and enable completed.
const uint32_t adcTimeoutMicros = 5000; ///< Calibration/conversion timeout.
#endif

#if CONFIG_INTERRUPT
GPIO_TypeDef *irqPort;           ///< Cached open-drain interrupt port.
uint32_t irqPinMask;             ///< Interrupt pin bit.
volatile bool irqActive = false; ///< Last published IRQ state.

/** @brief Publish latched GPIO changes on the active-low interrupt pin. */
void updateIRQ() {
  bool active = g_irqFlags != 0;
  if (active == irqActive) {
    return;
  }
  uint32_t savedMask = __get_PRIMASK();
  __disable_irq();
  // A read-to-clear callback may have run before interrupts were masked.
  active = g_irqFlags != 0;
  if (active) {
    irqPort->BSRR = irqPinMask << 16;
  } else {
    irqPort->BSRR = irqPinMask;
  }
  irqActive = active;
  __set_PRIMASK(savedMask);
}
#endif

/** @brief Snapshot exposed GPIOs without touching SWD, I2C, or the LED. */
inline uint32_t readGPIO() {
  // This backend is limited to C011F6Ux: D0..8 = PA0..8, D9/10 = PA11/12,
  // D15 = PC14. Snapshot each input register once instead of calling
  // digitalRead() for every pin while the I2C interrupt is waiting.
  uint32_t portA = GPIOA->IDR;
  uint32_t portC = GPIOC->IDR;
  uint32_t value = (portA & 0x01FFUL) | ((portA >> 2) & 0x0600UL) |
                   ((portC << 1) & 0x8000UL);
  return value & VALID_GPIO;
}

/** @brief Apply cached direction, output, and pull state to selected pins. */
void applyGPIO(uint32_t pins) {
  for (uint8_t pin = 0; pin < 16; pin++) {
    uint32_t bit = 1UL << pin;
    if (!(pins & VALID_GPIO & bit)) {
      continue;
    }
#if CONFIG_PWM || CONFIG_PWM_16BIT
    stopPWM(pin);
#endif
    if (direction & bit) {
      digitalWrite(pin, (output & bit) ? HIGH : LOW);
      pinMode(pin, OUTPUT);
    } else if (pull & bit) {
      pinMode(pin, (output & bit) ? INPUT_PULLUP : INPUT_PULLDOWN);
    } else {
      pinMode(pin, INPUT);
    }
  }
}

/** @brief Select a live first-byte source before a repeated START. */
void prepareFirstByte() {
  static const uint8_t zero = 0, hardwareID = SEESAW_HW_ID;
  static const uint8_t versionHigh = (uint32_t)PRODUCT_CODE >> 8;
  seesawWire.setFirstByte(&zero);
  if (selectedBase == SEESAW_STATUS_BASE) {
    if (selectedRegister == SEESAW_STATUS_HW_ID) {
      seesawWire.setFirstByte(&hardwareID);
    } else if (selectedRegister == SEESAW_STATUS_VERSION) {
      seesawWire.setFirstByte(&versionHigh);
    }
    // All supported OPTIONS and GPIO bits are below bit 24.
  }
#if CONFIG_ADC
  else if (selectedBase == SEESAW_ADC_BASE) {
    if (selectedRegister >= SEESAW_ADC_CHANNEL_OFFSET) {
      seesawWire.setFirstByte((const volatile uint8_t *)&g_bufferedADCRead + 1);
    } else if (selectedRegister == SEESAW_ADC_STATUS) {
      seesawWire.setFirstByte(&g_adcStatus);
    }
  }
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  else if (selectedBase == SEESAW_TIMER_BASE &&
           selectedRegister == SEESAW_TIMER_STATUS) {
    seesawWire.setFirstByte(&g_pwmStatus);
  }
#endif
}

/*! Decode the register pointer before a possible immediate repeated START. */
bool receiveHeader(uint8_t base, uint8_t reg) {
  selectedBase = base;
  selectedRegister = reg;
  prepareFirstByte();
  // A full queue can invalidate the pointer in receive(). In that case finish
  // reception before loading the first reply byte, even on a repeated START.
  // Between this header and START, the main loop can only free queue entries.
  return (head + 1) % queueSize != tail;
}

void receive(int length) {
  uint8_t next = (head + 1) % queueSize;
  if (length < 2 || length > commandSize || next == tail) {
    selectedBase = selectedRegister = 0xFF;
    prepareFirstByte();
    while (seesawWire.available()) {
      seesawWire.read();
    }
    return;
  }
  commands[head].length = length;
  for (uint8_t i = 0; i < length; i++) {
    commands[head].data[i] = seesawWire.read();
  }
  selectedBase = commands[head].data[0];
  selectedRegister = commands[head].data[1];
  prepareFirstByte();
  head = next;
}

#if CONFIG_ADC
/** @brief Calibrate and enable the ADC with bounded waits. */
bool beginADC() {
  // STM32C0 LL sequence: regulator, calibration, two ADC clocks, then enable.
  // HCLK/4 keeps the ADC at 12 MHz with the generic board's 48 MHz clock.
  __HAL_RCC_ADC_CLK_ENABLE();
  __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
  LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV4);
  LL_ADC_REG_SetSequencerConfigurable(ADC1, LL_ADC_REG_SEQ_FIXED);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_COMMON_1,
                                       LL_ADC_SAMPLINGTIME_160CYCLES_5);
  LL_ADC_EnableInternalRegulator(ADC1);
  delayMicroseconds(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);
  LL_ADC_StartCalibration(ADC1);
  uint32_t started = micros();
  while (LL_ADC_IsCalibrationOnGoing(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return false;
  }
  delayMicroseconds(1); // More than two ADC clocks at 12 MHz.
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  started = micros();
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return false;
  }
  return true;
}
#endif

/** @brief Reset GPIO and ADC state without writing nonvolatile memory. */
void reset() {
  direction = output = pull = 0;
#if CONFIG_INTERRUPT
  g_irqGPIO = g_irqFlags = 0;
#endif
  applyGPIO(VALID_GPIO);
#if CONFIG_PWM || CONFIG_PWM_16BIT
  g_pwmStatus = 0;
  pwmFrequency = 1000;
  analogWriteResolution(16);
  analogWriteFrequency(pwmFrequency);
#endif
#if CONFIG_ADC
  g_bufferedADCRead = g_adcStatus = 0;
  adcReady = beginADC();
  if (!adcReady) {
    g_adcStatus = 1;
  }
#endif
#if CONFIG_INTERRUPT
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(CONFIG_INTERRUPT_PIN, HIGH);
  // Keep a real open-drain output with a weak pull-up in the 3.3 V domain.
  PinName irqPin = digitalPinToPinName(CONFIG_INTERRUPT_PIN);
  pin_function(irqPin, STM_PIN_DATA(STM_MODE_OUTPUT_OD, LL_GPIO_PULL_UP, 0));
  irqPort = get_GPIO_Port(STM_PORT(irqPin));
  irqPinMask = 1UL << STM_PIN(irqPin);
  irqActive = false;
#endif
  lastGPIO = readGPIO();
}

/** @brief Apply address straps, falling back to the base on reserved addresses. */
uint8_t configuredAddress() {
  uint8_t result =
      Adafruit_seesawPeripheral_applyAddressStraps(CONFIG_I2C_PERIPH_ADDR);
  if (result < 8 || result > 0x77) {
    result = CONFIG_I2C_PERIPH_ADDR;
  }
  return result;
}
} // namespace SeesawSTM32

/** @brief Read a snapshot of the selected valid GPIO pins. */
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins = VALID_GPIO) {
  return SeesawSTM32::readGPIO() & validpins;
}

/** @brief Set or clear output direction while preserving pull configuration. */
void Adafruit_seesawPeripheral_gpioDirection(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  mask &= VALID_GPIO;
  if (enabled) {
    direction |= mask;
  } else {
    direction &= ~mask;
  }
  applyGPIO(mask);
}

/** @brief Update output levels, or pull polarity while a pin is an input. */
void Adafruit_seesawPeripheral_gpioWrite(uint32_t mask, uint8_t value) {
  using namespace SeesawSTM32;
  mask &= VALID_GPIO;
  if (value == HIGH) {
    output |= mask;
  } else {
    output &= ~mask;
  }
  applyGPIO(mask);
}

/** @brief Apply a complete GPIO value with one hardware pass. */
void Adafruit_seesawPeripheral_gpioBulk(uint32_t value) {
  using namespace SeesawSTM32;
  output = (output & ~VALID_GPIO) | (value & VALID_GPIO);
  applyGPIO(VALID_GPIO);
}

/** @brief Toggle selected valid GPIO output levels. */
void Adafruit_seesawPeripheral_gpioToggle(uint32_t mask) {
  using namespace SeesawSTM32;
  mask &= VALID_GPIO;
  output ^= mask;
  applyGPIO(mask);
}

/** @brief Enable or disable pulls without changing the direction register. */
void Adafruit_seesawPeripheral_gpioPull(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  mask &= VALID_GPIO;
  if (enabled) {
    pull |= mask;
  } else {
    pull &= ~mask;
  }
  applyGPIO(mask);
}

#if CONFIG_INTERRUPT
/** @brief Release the active-low open-drain interrupt output. */
void Adafruit_seesawPeripheral_clearIRQ(void) {
  SeesawSTM32::irqPort->BSRR = SeesawSTM32::irqPinMask;
  SeesawSTM32::irqActive = false;
}

/** @brief Enable or disable change reporting for one valid GPIO pin. */
void Adafruit_seesawPeripheral_setGPIOInterrupt(uint8_t pin, bool enabled) {
  using namespace SeesawSTM32;
  uint32_t mask = 1UL << pin;
  if (enabled) {
    lastGPIO = readGPIO();
    g_irqGPIO |= mask;
  } else {
    // Do not resurrect flags cleared by an intervening I2C read callback.
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();
    g_irqGPIO &= ~mask;
    g_irqFlags &= ~mask;
    __set_PRIMASK(savedMask);
    updateIRQ();
  }
}
#endif

#if CONFIG_ADC
/**
 * @brief Convert one validated ADC pin.
 * @param pin Protocol GPIO index, from 0 through 10.
 * @return 12-bit ADC result, or -1 if initialization or conversion failed.
 */
int32_t Adafruit_seesawPeripheral_readADC(uint8_t pin) {
  using namespace SeesawSTM32;
  if (!adcReady)
    return -1;
#if CONFIG_PWM || CONFIG_PWM_16BIT
  stopPWM(pin);
#endif
  pinMode(pin, INPUT_ANALOG);
  uint8_t channel = pin;
  if (pin == 9)
    channel = 11;
  if (pin == 10)
    channel = 12;
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_REG_SetSequencerChannels(ADC1,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
  uint32_t started = micros();
  while (!LL_ADC_IsActiveFlag_CCRDY(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return -1;
  }
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_ClearFlag_EOS(ADC1);
  LL_ADC_ClearFlag_OVR(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  started = micros();
  while (!LL_ADC_IsActiveFlag_EOC(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return -1;
  }
  return LL_ADC_REG_ReadConversionData12(ADC1);
}
#endif

#if CONFIG_PWM || CONFIG_PWM_16BIT
/**
 * @brief Set PWM duty on a validated timer pin using the Arduino core.
 * @param pin Protocol GPIO index with PWM support.
 * @param value Duty setting from 0 through 65535.
 */
void Adafruit_seesawPeripheral_setPWM(uint8_t pin, uint16_t value) {
  using namespace SeesawSTM32;
  pwmValues[pin] = value;
  analogWriteFrequency(pwmFrequency);
  analogWrite(pwmPins[pin], value);
  pwmActive |= 1UL << pin;
}

/**
 * @brief Change the shared PWM frequency and retain active duty settings.
 * @param pin Validated PWM pin; frequency applies to all active outputs.
 * @param frequency Nonzero frequency in hertz.
 */
void Adafruit_seesawPeripheral_setPWMFrequency(uint8_t pin,
                                               uint16_t frequency) {
  using namespace SeesawSTM32;
  (void)pin;
  pwmFrequency = frequency;
  applyPWM();
}
#endif

/** @brief Queue writes so ADC conversion never runs inside an I2C interrupt. */
void receiveEvent(int howMany) { SeesawSTM32::receive(howMany); }

/** @brief Serve a live GPIO snapshot or the last completed ADC conversion. */
void requestEvent(void) {
  using namespace SeesawSTM32;
  if (selectedBase == SEESAW_GPIO_BASE &&
      selectedRegister == SEESAW_GPIO_BULK) {
    g_bufferedBulkGPIORead = readGPIO();
  }
  Adafruit_seesawPeripheral_processRequest(selectedBase, selectedRegister);
}

/** @brief Reset peripheral state and apply a changed strap address. */
void Adafruit_seesawPeripheral_reset(void) {
  using namespace SeesawSTM32;
  reset();
  uint8_t newAddress = configuredAddress();
  if (newAddress != address) {
    seesawWire.end();
    address = newAddress;
    seesawWire.begin(address);
  }
}

/** @brief Initialize the GPIO/ADC/PWM backend and dedicated I2C target. */
bool Adafruit_seesawPeripheral_begin(void) {
  using namespace SeesawSTM32;
  Adafruit_seesawPeripheral_setDatecode();
  reset();
#if CONFIG_ADC
  if (!adcReady) {
    return false;
  }
#endif
  seesawWire.setSDA(CONFIG_I2C_SDA_PIN);
  seesawWire.setSCL(CONFIG_I2C_SCL_PIN);
  address = configuredAddress();
  // This dedicated transport retains callbacks across begin(), unlike Wire.
  seesawWire.onHeader(receiveHeader);
  seesawWire.onReceive(receiveEvent);
  seesawWire.onRequest(requestEvent);
  return seesawWire.begin(address);
}

/** @brief Process queued commands and poll GPIO change interrupts. */
void Adafruit_seesawPeripheral_run(void) {
  using namespace SeesawSTM32;
  if (tail != head) {
    Command command;
    command.length = commands[tail].length;
    for (uint8_t i = 0; i < command.length; i++) {
      command.data[i] = commands[tail].data[i];
    }
    tail = (tail + 1) % queueSize;
    Adafruit_seesawPeripheral_processCommand(command.data, command.length);
  }
#if CONFIG_INTERRUPT
  uint32_t current = readGPIO();
  uint32_t changed = (current ^ lastGPIO) & g_irqGPIO;
  lastGPIO = current;
  if (changed) {
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();
    g_irqFlags |= changed;
    __set_PRIMASK(savedMask);
  }
  updateIRQ();
#endif
}
#endif
