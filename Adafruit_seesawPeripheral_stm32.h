/*!
 * @file Adafruit_seesawPeripheral_stm32.h
 * STM32duino hardware and deferred-I2C execution backend.
 * Register decoding and responses live in the shared receive/request headers.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_H

#if !defined(ARDUINO_GENERIC_C011F6UX)
#error "The STM32 backend currently supports the generic STM32C011F6Ux only"
#endif
#if CONFIG_ADC
#include "stm32c0xx_ll_adc.h"
#endif
#if CONFIG_SPI
#include <SPI.h>
#endif
#if CONFIG_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif
#if CONFIG_FHT
#error "The AVR assembly FHT backend is not available on STM32"
#endif
#if CONFIG_UART || CONFIG_UART_DEBUG
#ifndef CONFIG_UART_SERCOM
#define CONFIG_UART_SERCOM Serial
#endif
#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 1 // PA1, USART1 RX.
#endif
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 0 // PA0, USART1 TX.
#endif
#endif
#if CONFIG_EEPROM
extern "C" uint8_t _sidata, _sdata, _edata;
#endif

namespace SeesawSTM32 {
const uint32_t gpioMask = 0x000087FFUL; ///< PA0-PA8, PA11, PA12, PC14.
const uint32_t adcMask = 0x000007FFUL;  ///< Eleven exposed analog inputs.
const uint32_t pwmMask = 0x000083FFUL;  ///< All exposed pins except PA12.
const uint8_t queueSize = 4; ///< Bounded command queue; one entry stays empty.
const uint8_t commandSize = 32; ///< Matches the classic seesaw Wire packet.
struct Command {
  uint8_t length;            ///< Packet length, including module and register.
  uint8_t data[commandSize]; ///< Received bytes.
};
volatile Command commands[queueSize]; ///< Single producer/consumer queue.
volatile uint8_t head = 0, tail = 0;  ///< Queue cursors.
volatile uint8_t selectedBase = 0, selectedRegister = 0; ///< Read pointer.
uint32_t direction = 0, output = 0, pull = 0; ///< GPIO register semantics.
uint32_t lastGPIO = 0;                        ///< Last sampled GPIO state.
uint32_t pwmActive = 0;                       ///< Pins currently using timers.
#if CONFIG_SPI
const uint8_t spiBase = 0x13; ///< Experimental SPI controller register module.
const uint32_t spiPinMask = 0xF0; ///< PA4 CS, PA5 SCK, PA6 MISO, PA7 MOSI.
SPIClass spiPort(PA7, PA6, PA5);  ///< Dedicated hardware SPI controller.
volatile bool spiConfigured = false,
              spiSelected = false; ///< Pin ownership and CS state.
volatile uint8_t spiPending =
    0; ///< Accepted commands, including the active one.
volatile uint8_t spiError = 0,
                 spiSequence = 0;      ///< Sticky error and completion ID.
volatile bool spiAbortPending = false; ///< ISR asks main loop to release CS.
volatile uint8_t spiRxLength = 0;  ///< Completed bytes available to the host.
uint8_t spiRx[29];                 ///< One full I2C transfer's receive bytes.
uint32_t spiFrequency = 1000000;   ///< Requested SPI frequency in hertz.
uint8_t spiMode = 0, spiOrder = 0; ///< SPI mode 0-3, MSB-first 0 / LSB-first 1.
#endif
uint16_t pwmValues[16] = {};  ///< Duty values needed when frequency changes.
uint16_t pwmFrequency = 1000; ///< Shared PWM frequency in hertz.
#if CONFIG_PWM
const uint16_t pwmPins[16] = {PA0_ALT1, PA1_ALT1, 2, PA3_ALT1, PA4_ALT1, 5,  6,
                              PA7_ALT1, PA8_ALT3, 9, 10,       11,       12, 13,
                              14,       15}; ///< Prefer non-complementary
                                             ///< channels.
#endif
#if CONFIG_ADC
bool adcReady = false; ///< Calibration and enable completed.
const uint32_t adcTimeoutMicros =
    5000; ///< Bounded calibration/conversion waits.
#endif
uint8_t address = CONFIG_I2C_PERIPH_ADDR; ///< Current effective I2C address.
#if CONFIG_EEPROM
const uint32_t eepromAddress = FLASH_BASE + 30 * 1024; ///< Last 2 KB page.
const uint16_t eepromSize = 256;    ///< Wire-addressable EEPROM bytes.
const uint8_t addressOffset = 0xFF; ///< Persistent I2C address byte.
uint8_t eepromBuffer[eepromSize]
    __attribute__((aligned(8))); ///< Page workspace.
bool eepromSafe = false; ///< True only if program data ends before this page.
#endif
#if CONFIG_ENCODER
static_assert(CONFIG_NUM_ENCODERS >= 1 && CONFIG_NUM_ENCODERS <= 4,
              "Configure between one and four encoders");
const uint8_t encoderPins[CONFIG_NUM_ENCODERS][2] = {
    {CONFIG_ENCODER0_A_PIN, CONFIG_ENCODER0_B_PIN},
#if CONFIG_NUM_ENCODERS > 1
    {CONFIG_ENCODER1_A_PIN, CONFIG_ENCODER1_B_PIN},
#endif
#if CONFIG_NUM_ENCODERS > 2
    {CONFIG_ENCODER2_A_PIN, CONFIG_ENCODER2_B_PIN},
#endif
#if CONFIG_NUM_ENCODERS > 3
    {CONFIG_ENCODER3_A_PIN, CONFIG_ENCODER3_B_PIN},
#endif
}; ///< Encoder A/B pin indices.
volatile uint8_t encoderInterrupts = 0; ///< Interrupt enable bitmap.
#endif
#if CONFIG_NEOPIXEL
class RawPixels : public Adafruit_NeoPixel {
public:
  RawPixels()
      : Adafruit_NeoPixel((CONFIG_NEOPIXEL_BUF_MAX + 2) / 3, -1,
                          NEO_RGB + NEO_KHZ800) {}
  void byteLength(uint16_t length) { numBytes = length; }
}; ///< Uses NeoPixel's STM32 timing with unmodified raw RGB/RGBW byte order.
RawPixels pixels; ///< Fixed-capacity LED buffer, allocated once.
#endif

uint32_t validGPIO() {
  uint32_t pins = gpioMask;
#if CONFIG_SPI
  if (spiConfigured)
    pins &= ~spiPinMask;
#endif
#ifdef CONFIG_INTERRUPT_PIN
  pins &= ~(1UL << CONFIG_INTERRUPT_PIN);
#endif
#if CONFIG_UART || CONFIG_UART_DEBUG
  pins &= ~((1UL << CONFIG_UART_RX_PIN) | (1UL << CONFIG_UART_TX_PIN));
#endif
#if CONFIG_ADDR_0
  pins &= ~(1UL << CONFIG_ADDR_0_PIN);
#endif
#if CONFIG_ADDR_1
  pins &= ~(1UL << CONFIG_ADDR_1_PIN);
#endif
#if CONFIG_ADDR_2
  pins &= ~(1UL << CONFIG_ADDR_2_PIN);
#endif
#if CONFIG_ADDR_3
  pins &= ~(1UL << CONFIG_ADDR_3_PIN);
#endif
  return pins;
}

void updateIRQ() {
#ifdef CONFIG_INTERRUPT_PIN
  // A real open-drain output; the local pull-up is in the 3.3 V domain.
  bool active = g_irqFlags != 0;
#if CONFIG_UART
  active |= g_uart_inten && CONFIG_UART_SERCOM.available();
#endif
#if CONFIG_ENCODER
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    active |= (encoderInterrupts & (1 << i)) && g_enc_delta[i] != 0;
  }
#endif
  digitalWrite(CONFIG_INTERRUPT_PIN, active ? LOW : HIGH);
#endif
}

uint32_t readGPIO() {
  uint32_t value = 0;
  for (uint8_t pin = 0; pin < 16; pin++) {
    if ((validGPIO() & (1UL << pin)) && digitalRead(pin)) {
      value |= 1UL << pin;
    }
  }
  return value;
}

void applyGPIO(uint32_t pins) {
  for (uint8_t pin = 0; pin < 16; pin++) {
    uint32_t bit = 1UL << pin;
    if (!(pins & validGPIO() & bit)) {
      continue;
    }
    // Stop the exact alternate timer function before switching back to GPIO.
#if CONFIG_PWM
    if (pwmActive & bit)
      pinMode(pwmPins[pin], INPUT);
#endif
    if (direction & bit) {
      digitalWrite(pin, (output & bit) ? HIGH : LOW);
      pinMode(pin, OUTPUT);
    } else if (pull & bit) {
      pinMode(pin, (output & bit) ? INPUT_PULLUP : INPUT_PULLDOWN);
    } else {
      pinMode(pin, INPUT);
    }
    pwmActive &= ~bit;
  }
}

#if CONFIG_SPI
#include "Adafruit_seesawPeripheral_stm32_spi.h"
#endif

void receive(int length) {
#if CONFIG_SPI
  // Status/RX reads must remain available when the work queue is full.
  if (length >= 2) {
    uint8_t base = Wire.peek();
    if (base == spiBase && length == 2) {
      Wire.read();
      uint8_t reg = Wire.read();
      selectedBase = base;
      selectedRegister = reg;
      if (reg == spiStatusRegister || reg == spiReadRegister ||
          reg == spiConfigRegister || reg == spiClockRegister)
        return;
      // Other two-byte SPI commands are invalid; do not silently enqueue them.
      spiError = spiInvalidCommand;
      spiAbortPending = true;
      return;
    }
  }
#endif
  uint8_t next = (head + 1) % queueSize;
  if (length < 2 || length > commandSize || next == tail) {
#if CONFIG_SPI
    if (spiConfigured || spiPending ||
        (length >= 2 && Wire.peek() == spiBase)) {
      spiError = spiQueueOverflow;
      spiAbortPending = true;
    }
#endif
    selectedBase = selectedRegister = 0xFF;
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }
  commands[head].length = length;
  for (uint8_t i = 0; i < length; i++) {
    commands[head].data[i] = Wire.read();
  }
  selectedBase = commands[head].data[0];
  selectedRegister = commands[head].data[1];
#if CONFIG_SPI
  if (selectedBase == spiBase)
    spiPending++;
#endif
  head = next;
}

#if CONFIG_ADC
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

bool readADC(uint8_t pin, uint16_t &value) {
  if (!adcReady)
    return false;
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
      return false;
  }
  LL_ADC_ClearFlag_EOC(ADC1);
  LL_ADC_ClearFlag_EOS(ADC1);
  LL_ADC_ClearFlag_OVR(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
  started = micros();
  while (!LL_ADC_IsActiveFlag_EOC(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return false;
  }
  value = LL_ADC_REG_ReadConversionData12(ADC1);
  return true;
}
#endif

void reset() {
#if CONFIG_SPI
  spiStop();
  spiError = spiSequence = spiRxLength = 0;
  spiAbortPending = false;
#endif
  direction = output = pull = 0;
  g_irqGPIO = g_irqFlags = 0;
#if CONFIG_ADC
  g_bufferedADCRead = g_adcStatus = 0;
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  g_pwmStatus = 0;
#endif
  pwmFrequency = 1000;
  applyGPIO(validGPIO());
#if CONFIG_ADC
  adcReady = beginADC();
  if (!adcReady)
    g_adcStatus = 1;
#endif
#if CONFIG_NEOPIXEL
  g_neopixel_bufsize = 0;
  g_neopixel_pin = 0xFF;
  pixels.setPin(-1);
  pixels.byteLength(CONFIG_NEOPIXEL_BUF_MAX);
  pixels.clear();
  pixels.byteLength(0);
  g_neopixel_status = pixels.getPixels() ? 0 : 1;
#endif
#if CONFIG_UART
  g_uart_inten = 0;
  g_uart_baud = 9600;
  CONFIG_UART_SERCOM.end();
  CONFIG_UART_SERCOM.setRx(CONFIG_UART_RX_PIN);
  CONFIG_UART_SERCOM.setTx(CONFIG_UART_TX_PIN);
  CONFIG_UART_SERCOM.begin(g_uart_baud);
#endif
#if CONFIG_ENCODER
  encoderInterrupts = 0;
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    pinMode(encoderPins[i][0], INPUT_PULLUP);
    pinMode(encoderPins[i][1], INPUT_PULLUP);
    uint32_t bits = (1UL << encoderPins[i][0]) | (1UL << encoderPins[i][1]);
    pull |= bits;
    output |= bits;
    g_enc_prev_pos[i] = 3 ^ (digitalRead(encoderPins[i][0]) |
                             (digitalRead(encoderPins[i][1]) << 1));
    g_enc_value[i] = g_enc_delta[i] = 0;
    g_enc_flags[i] = 0;
  }
#endif
#ifdef CONFIG_INTERRUPT_PIN
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(CONFIG_INTERRUPT_PIN, HIGH);
  // Enable a weak pull-up without changing the open-drain output mode.
  PinName irqPin = digitalPinToPinName(CONFIG_INTERRUPT_PIN);
  pin_function(irqPin, STM_PIN_DATA(STM_MODE_OUTPUT_OD, LL_GPIO_PULL_UP, 0));
#endif
  lastGPIO = readGPIO();
}

uint8_t configuredAddress() {
  uint8_t result = CONFIG_I2C_PERIPH_ADDR;
#if CONFIG_EEPROM
  if (eepromSafe) {
    uint8_t saved = *((const uint8_t *)eepromAddress + addressOffset);
    if (saved >= 8 && saved <= 0x77)
      result = saved;
  }
#endif
  result = Adafruit_seesawPeripheral_applyAddressStraps(result);
  if (result < 8 || result > 0x77)
    result = CONFIG_I2C_PERIPH_ADDR;
  return result;
}

#if CONFIG_EEPROM
void writeEEPROM(uint8_t offset, const uint8_t *data, uint8_t length) {
  if (!eepromSafe || (uint16_t)offset + length > eepromSize)
    return;
  memcpy(eepromBuffer, (const void *)eepromAddress, eepromSize);
  if (memcmp(eepromBuffer + offset, data, length) == 0)
    return;
  memcpy(eepromBuffer + offset, data, length);
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = 15;
  erase.NbPages = 1;
  uint32_t pageError;
  if (HAL_FLASH_Unlock() != HAL_OK)
    return;
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  if (HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK) {
    for (uint16_t pos = 0; pos < eepromSize; pos += 8) {
      uint64_t word;
      memcpy(&word, eepromBuffer + pos, sizeof(word));
      if (word != UINT64_MAX &&
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, eepromAddress + pos,
                            word) != HAL_OK)
        break;
    }
  }
  HAL_FLASH_Lock();
}
#endif

} // namespace SeesawSTM32

// GPIO hardware hooks; register decoding is shared with AVR.
void Adafruit_seesawPeripheral_gpioDirection(uint32_t mask, bool output) {
  using namespace SeesawSTM32;
  if (output)
    direction |= mask;
  else
    direction &= ~mask;
  applyGPIO(mask);
}
void Adafruit_seesawPeripheral_gpioWrite(uint32_t mask, bool high) {
  using namespace SeesawSTM32;
  if (high)
    output |= mask;
  else
    output &= ~mask;
  applyGPIO(mask);
}
void Adafruit_seesawPeripheral_gpioToggle(uint32_t mask) {
  using namespace SeesawSTM32;
  output ^= mask;
  applyGPIO(mask);
}
void Adafruit_seesawPeripheral_gpioPull(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  if (enabled)
    pull |= mask;
  else
    pull &= ~mask;
  applyGPIO(mask);
}
void Adafruit_seesawPeripheral_gpioInterrupt(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  if (enabled) {
    lastGPIO = readGPIO();
    g_irqGPIO |= mask;
  } else {
    g_irqGPIO &= ~mask;
    g_irqFlags &= ~mask;
    updateIRQ();
  }
}
#if CONFIG_PWM
void Adafruit_seesawPeripheral_setPWM(uint8_t pin, uint16_t value) {
  using namespace SeesawSTM32;
  pwmValues[pin] = value;
  analogWriteFrequency(pwmFrequency);
  analogWrite(pwmPins[pin], value);
  pwmActive |= 1UL << pin;
}
void Adafruit_seesawPeripheral_setPWMFrequency(uint8_t pin, uint16_t value) {
  using namespace SeesawSTM32;
  pwmFrequency = value;
  analogWriteFrequency(value);
  for (uint8_t activePin = 0; activePin < 16; activePin++)
    if (pwmActive & (1UL << activePin))
      analogWrite(pwmPins[activePin], pwmValues[activePin]);
}
#endif
#if CONFIG_NEOPIXEL
volatile uint8_t *Adafruit_seesawPeripheral_pixelBuffer() {
  return SeesawSTM32::pixels.getPixels();
}
void Adafruit_seesawPeripheral_setPixelPin(uint8_t pin) {
  SeesawSTM32::pixels.setPin(pin);
}
void Adafruit_seesawPeripheral_setPixelLength(uint16_t length) {
  SeesawSTM32::pixels.byteLength(length);
}
void Adafruit_seesawPeripheral_showPixels() {
  using namespace SeesawSTM32;
#if CONFIG_PWM
  if (pwmActive & (1UL << g_neopixel_pin))
    pinMode(pwmPins[g_neopixel_pin], INPUT);
#endif
  pinMode(g_neopixel_pin, OUTPUT);
  pwmActive &= ~(1UL << g_neopixel_pin);
  pixels.show();
}
#endif

/*! Reset peripheral state, then apply a changed persistent I2C address. */
void Adafruit_seesawPeripheral_reset(void) {
  using namespace SeesawSTM32;
  reset();
  uint8_t newAddress = configuredAddress();
  if (newAddress != address) {
    Wire.end();
    address = newAddress;
    Wire.begin((int)address);
    // STM32duino clears callbacks in begin(); always reattach afterward.
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
  }
}

bool Adafruit_seesawPeripheral_begin() {
  using namespace SeesawSTM32;
  Adafruit_seesawPeripheral_setDatecode();
#if CONFIG_EEPROM
  // Refuse flash writes if a build forgot to reserve the final page.
  eepromSafe =
      (uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata) <=
      eepromAddress;
  if (!eepromSafe)
    return false;
#endif
  reset();
#if CONFIG_PWM
  analogWriteResolution(16);
#endif
  Wire.setSDA(CONFIG_I2C_SDA_PIN);
  Wire.setSCL(CONFIG_I2C_SCL_PIN);
  address = configuredAddress();
  Wire.begin((int)address);
  // STM32duino begin() clears the request callback: attach after
  // initialization.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  return true;
}

void Adafruit_seesawPeripheral_run() {
  using namespace SeesawSTM32;
#if CONFIG_SPI
  if (spiAbortPending) {
    spiRelease();
    spiAbortPending = false;
  }
#endif
  if (tail != head) {
    Command command;
    command.length = commands[tail].length;
    for (uint8_t i = 0; i < command.length; i++) {
      command.data[i] = commands[tail].data[i];
    }
    tail = (tail + 1) % queueSize;
#if CONFIG_SPI && defined(CONFIG_SPI_TEST_HOLD_US)
    // Engineering fault injection only: leave IRQs enabled while the host
    // deterministically fills the queue. Production builds omit this entirely.
    if (command.data[0] == spiBase)
      delayMicroseconds(CONFIG_SPI_TEST_HOLD_US);
#endif
    Adafruit_seesawPeripheral_processCommand(command.data, command.length);
  }
#if defined(CONFIG_INTERRUPT_PIN) || CONFIG_ENCODER
  // Snapshot and flag updates must not race the read-to-clear Wire callback.
  noInterrupts();
  uint32_t current = readGPIO();
#if CONFIG_ENCODER
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    uint8_t phase = 3 ^ (digitalRead(encoderPins[i][0]) |
                         (digitalRead(encoderPins[i][1]) << 1));
    Adafruit_seesawPeripheral_updateEncoder(i, phase);
  }
#endif
  g_irqFlags |= (current ^ lastGPIO) & g_irqGPIO;
  lastGPIO = current;
  updateIRQ();
  interrupts();
#endif
}
#endif
