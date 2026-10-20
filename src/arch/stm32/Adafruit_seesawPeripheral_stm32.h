/*!
 * @file Adafruit_seesawPeripheral_stm32.h
 * @brief STM32 backend with deferred command execution.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_H

#if CONFIG_EEPROM
extern "C" uint8_t _sidata, _sdata, _edata;
#endif
#if CONFIG_UART && CONFIG_UART_DEBUG
#error "Keep Serial debug output disabled when using the STM32 UART bridge"
#endif
#if CONFIG_UART
static_assert(CONFIG_UART_RX_PIN < 32 && CONFIG_UART_TX_PIN < 32 &&
                  CONFIG_UART_RX_PIN != CONFIG_UART_TX_PIN &&
                  (ALL_GPIO & SEESAW_UART_PIN_MASK) == SEESAW_UART_PIN_MASK &&
                  !(INVALID_GPIO & SEESAW_UART_PIN_MASK),
              "UART pins must be distinct exposed pins without other uses");
#endif
#if USE_PINCHANGE_INTERRUPT
#error "The STM32 backend polls GPIO changes from run()"
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

#if CONFIG_ADC || CONFIG_FHT
#include "stm32yyxx_ll_adc.h"
#endif
#if (CONFIG_PWM || CONFIG_PWM_16BIT) && defined(ARDUINO_GENERIC_G071GBUX)
#include "analog.h"
#endif
#if CONFIG_FHT
#include "Adafruit_seesawPeripheral_stm32_spectrum.h"
#endif
#if CONFIG_SPI
#include <SPI.h>
#endif
#if CONFIG_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif

#if defined(ARDUINO_GENERIC_C031G6UX) || defined(ARDUINO_GENERIC_C051G8UX)
/**
 * @brief Supply the 48 MHz clock missing from this generic core variant.
 *
 * Arduino calls this startup hook before setup(). Use the internal oscillator;
 * these breakouts do not fit a crystal. The core's weak hook is empty.
 */
extern "C" void SystemClock_Config(void) {
  RCC_ClkInitTypeDef clocks = {};
  clocks.ClockType =
      RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV1;
  // Raise flash latency before increasing the reset-time HSI frequency.
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  RCC_OscInitTypeDef oscillator = {};
  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscillator.HSIState = RCC_HSI_ON;
  oscillator.HSIDiv = RCC_HSI_DIV1;
  oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // HAL updates SystemCoreClock and the SysTick time base after the divider.
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
    Error_Handler();
  }
}
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
#if CONFIG_BOOTLOADER
const uint32_t systemMemory =
    0x1FFF0000;                        ///< Supported C0/G0 ROM vectors, AN2606.
const uint32_t bootMagic = 0xB007C011; ///< One-shot reset request signature.
volatile uint32_t bootRequest[2]
    __attribute__((section(".noinit"))); ///< Retained across a software reset.

/**
 * @brief Switch stacks without compiler stack access afterward.
 * @param vectors Factory-ROM vector table address.
 */
__attribute__((naked, noreturn)) void enterSystemMemory(uint32_t vectors) {
  __asm volatile("ldr r1, [r0]\n"
                 "ldr r2, [r0, #4]\n"
                 "msr msp, r1\n"
                 "cpsie i\n"
                 "bx r2\n");
}

/** @brief Reset first so active peripherals cannot interfere with the ROM. */
void requestBootloader() {
  bootRequest[0] = bootMagic;
  bootRequest[1] = ~bootMagic;
  __DSB();
  NVIC_SystemReset();
}

/** @brief Honor a one-shot request before constructors or Arduino setup. */
void checkBootloader() {
  bool requested = __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) &&
                   bootRequest[0] == bootMagic && bootRequest[1] == ~bootMagic;
  bootRequest[0] = bootRequest[1] = 0;
  if (!requested) {
    return;
  }

  // Run immediately after reset, before Arduino configures any peripherals.
  __disable_irq();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  SCB->VTOR = systemMemory;
  __set_CONTROL(0);
  __DSB();
  __ISB();
  enterSystemMemory(systemMemory);
}
void (*const bootloaderPreinit)() __attribute__((section(".preinit_array"),
                                                 used)) =
    checkBootloader; ///< Preserve Arduino startup hooks.
#endif
#if CONFIG_EEPROM
const uint32_t eepromAddress =
    FLASH_BASE + SEESAW_STM32_FLASH_BYTES - FLASH_PAGE_SIZE; ///< Final page.
const uint16_t eepromSize = 256; ///< Bytes addressable by the seesaw protocol.
uint8_t *eepromBuffer = nullptr; ///< Full-page workspace, allocated once.
bool eepromSafe = false;         ///< Application fits below the reserved page.
bool eepromDirty = false;        ///< Current command changed at least one byte.
#endif
#if CONFIG_ENCODER
static_assert(CONFIG_NUM_ENCODERS >= 1 && CONFIG_NUM_ENCODERS <= 4,
              "Configure between one and four encoders");
constexpr uint8_t encoderPins[CONFIG_NUM_ENCODERS][2] = {
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
}; ///< Protocol pin indices for each encoder's A/B inputs.

/** @brief Require distinct encoder inputs on exposed, unreserved GPIOs. */
constexpr bool validEncoderPins() {
  uint32_t used = 0;
  for (uint8_t encoder = 0; encoder < CONFIG_NUM_ENCODERS; encoder++) {
    for (uint8_t channel = 0; channel < 2; channel++) {
      uint8_t pin = encoderPins[encoder][channel];
      if (pin >= 32 || !(VALID_GPIO & (1UL << pin)) || (used & (1UL << pin))) {
        return false;
      }
#if CONFIG_FHT
      if (pin == CONFIG_FHT_PIN) {
        return false;
      }
#endif
      used |= 1UL << pin;
    }
  }
  return true;
}
static_assert(validEncoderPins(), "Encoder pins must be distinct valid GPIOs");
#if CONFIG_INTERRUPT
volatile uint8_t encoderInterrupts =
    0; ///< Per-encoder interrupt-enable bitmap.
#endif
#endif
#if CONFIG_NEOPIXEL
static_assert(CONFIG_NEOPIXEL_BUF_MAX > 0 && CONFIG_NEOPIXEL_BUF_MAX <= 65532,
              "NeoPixel capacity must fit the library's 16-bit byte count");
/** @brief Allocate once, then send the host's raw RGB or RGBW byte sequence. */
class RawPixels : public Adafruit_NeoPixel {
public:
  RawPixels()
      : Adafruit_NeoPixel((CONFIG_NEOPIXEL_BUF_MAX + 2) / 3, -1,
                          NEO_RGB + NEO_KHZ800) {}
  /** @brief Select a byte count without reallocating or converting colors. */
  void byteLength(uint16_t length) { numBytes = length; }
};
RawPixels pixels; ///< Fixed-capacity NeoPixel storage and Arduino transmitter.
#endif
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
#if CONFIG_SPI
const uint8_t spiBase = 0x13; ///< Experimental SPI controller module.
constexpr uint16_t spiPins[] = {
    CONFIG_SPI_CS_PIN, CONFIG_SPI_SCK_PIN, CONFIG_SPI_MISO_PIN,
    CONFIG_SPI_MOSI_PIN}; ///< Protocol pin indices for CS, SCK, MISO, MOSI.

/** @brief Require distinct SPI pins on exposed, unreserved GPIOs. */
constexpr bool validSPIPins() {
  uint32_t used = 0;
  for (uint16_t pin : spiPins) {
    if (pin >= 32 || !(VALID_GPIO & (1UL << pin)) || (used & (1UL << pin))) {
      return false;
    }
    used |= 1UL << pin;
  }
  return true;
}
static_assert(validSPIPins(), "SPI pins must be distinct valid GPIOs");
const uint32_t spiPinMask =
    (1UL << CONFIG_SPI_CS_PIN) | (1UL << CONFIG_SPI_SCK_PIN) |
    (1UL << CONFIG_SPI_MISO_PIN) |
    (1UL << CONFIG_SPI_MOSI_PIN); ///< Pins reserved while SPI is configured.
SPIClass spiPort(CONFIG_SPI_MOSI_PIN, CONFIG_SPI_MISO_PIN,
                 CONFIG_SPI_SCK_PIN); ///< Dedicated Arduino SPI controller.
volatile bool spiConfigured = false, spiSelected = false; ///< SPI ownership/CS.
volatile uint8_t spiPending = 0; ///< Queued commands, including the active one.
volatile uint8_t spiError = 0, spiSequence = 0; ///< Error and completion count.
volatile bool spiAbortPending = false; ///< ISR requests main-loop CS release.
volatile uint8_t spiRxLength = 0;  ///< Completed bytes available to the host.
uint8_t spiRx[29];                 ///< One I2C command's received bytes.
uint32_t spiFrequency = 1000000;   ///< Requested SPI frequency in hertz.
uint8_t spiMode = 0, spiOrder = 0; ///< Mode 0..3; MSB-first 0, LSB-first 1.

#endif
#if CONFIG_SPI || CONFIG_FHT
/** @brief Exclude pins owned by active SPI and spectrum capture. */
uint32_t validGPIO() {
  uint32_t pins = VALID_GPIO;
#if CONFIG_SPI
  if (spiConfigured) {
    pins &= ~spiPinMask;
  }
#endif
#if CONFIG_FHT
  pins &= ~(1UL << spectrumChannel);
#endif
  return pins;
}
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
uint32_t pwmActive = 0; ///< Protocol pins currently using timers.
uint16_t pwmValues[SEESAW_STM32_PIN_COUNT] =
    {}; ///< Duty settings retained across frequency changes.
uint16_t pwmFrequency = 1000; ///< Shared PWM frequency in hertz.
#if defined(ARDUINO_GENERIC_C011F6UX)
// Select non-complementary outputs. D3/D9 share TIM1_CH4, and D7/D15
// share TIM3_CH2; use only one pin from each pair at a time.
const uint16_t pwmPins[SEESAW_STM32_PIN_COUNT] = {
    PA0_ALT1, PA1_ALT1, 2,  PA3_ALT1, PA4_ALT1, 5,  6,  PA7_ALT1,
    PA8_ALT3, 9,        10, 11,       12,       13, 14, 15};
#elif defined(ARDUINO_GENERIC_C051G8UX)
// Keep the C031 timer assignments. C051's extra TIM2 changes alternate indices.
const uint16_t pwmPins[SEESAW_STM32_PIN_COUNT] = {
    PA0_ALT2, PA1_ALT2, PA2,  PA3_ALT1, PA4_ALT1, PA5,  PA6,      PA7_ALT1,
    PA8_ALT3, PA11,     PA12, PA13,     PA14,     PA15, PB0_ALT1, PB1_ALT2,
    PB3,      PB4,      PB5,  PB6,      PB7,      PB8,  PC6_ALT1, PC14};
#elif defined(ARDUINO_GENERIC_G071GBUX)
// G071: PA2/PA3 use TIM15, PB1 uses ALT1, and PC14 has no PWM output.
const uint16_t pwmPins[SEESAW_STM32_PIN_COUNT] = {
    PA0, PA1,  PA2_ALT1, PA3_ALT1, PA4,  PA5,  PA6,      PA7_ALT1,
    PA8, PA11, PA12,     PA13,     PA14, PA15, PB0_ALT1, PB1_ALT1,
    PB3, PB4,  PB5,      PB6,      PB7,  PB8,  PC6,      PC14};
#else
// C031: select normal rather than complementary outputs. Several pins share
// timer channels; see the README before enabling those outputs together.
const uint16_t pwmPins[SEESAW_STM32_PIN_COUNT] = {
    PA0_ALT1, PA1_ALT1, PA2,  PA3_ALT1, PA4_ALT1, PA5,  PA6,      PA7_ALT1,
    PA8_ALT3, PA11,     PA12, PA13,     PA14,     PA15, PB0_ALT1, PB1_ALT2,
    PB3,      PB4,      PB5,  PB6,      PB7,      PB8,  PC6,      PC14};
#endif

#if CONFIG_FHT
/** @brief Keep Arduino PWM away from the timer driving ADC conversions. */
bool spectrumAllowsPWM(uint8_t pin) {
  return pinmap_peripheral(digitalPinToPinName(pwmPins[pin]), PinMap_PWM) !=
         TIM3;
}
#endif

/**
 * @brief Write one timer output without selecting a G071 DAC by accident.
 * @param pin Validated protocol pin.
 * @param value Duty setting from 0 through 65535.
 */
void writePWM(uint8_t pin, uint16_t value) {
#if defined(ARDUINO_GENERIC_G071GBUX)
  // analogWrite() chooses DAC before PWM on PA4/PA5. Use STM32duino's own
  // timer helper explicitly; it still owns HardwareTimer setup and pin muxing.
  pwm_start(digitalPinToPinName(pwmPins[pin]), pwmFrequency, value,
            RESOLUTION_16B_COMPARE_FORMAT);
#else
  analogWrite(pwmPins[pin], value);
#endif
}

/** @brief Restore active PWM outputs after changing their shared frequency. */
void applyPWM() {
  analogWriteFrequency(pwmFrequency);
  for (uint8_t pin = 0; pin < SEESAW_STM32_PIN_COUNT; pin++) {
    if (pwmActive & (1UL << pin)) {
      writePWM(pin, pwmValues[pin]);
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
#if defined(ARDUINO_GENERIC_G071GBUX)
  // Direct timer output bypasses analogWrite's DAC/PWM ownership flag.
  pwm_stop(digitalPinToPinName(pwmPins[pin]));
#endif
  pinMode(pwmPins[pin], INPUT);
  applyPWM();
}
#endif
#if CONFIG_FHT
/** @brief Reject ADC inputs already owned by another enabled feature. */
bool spectrumPinAvailable(uint8_t pin) {
  if (pin >= 32 || !(VALID_ADC & (1UL << pin))) {
    return false;
  }
  uint32_t bit = 1UL << pin;
#if CONFIG_SPI
  if (spiConfigured && (spiPinMask & bit)) {
    return false;
  }
#endif
#if CONFIG_ENCODER
  for (uint8_t encoder = 0; encoder < CONFIG_NUM_ENCODERS; encoder++) {
    if (pin == encoderPins[encoder][0] || pin == encoderPins[encoder][1]) {
      return false;
    }
  }
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  if (pwmActive & bit) {
    return false;
  }
#endif
#if CONFIG_NEOPIXEL
  if (pin == g_neopixel_pin) {
    return false;
  }
#endif
  (void)bit;
  return true;
}
#endif
#if CONFIG_ADC || CONFIG_FHT
bool adcReady = false;                  ///< Calibration and enable completed.
const uint32_t adcTimeoutMicros = 5000; ///< Calibration/conversion timeout.
#endif
#if CONFIG_ADC
volatile int32_t temperature = 0; ///< Last die temperature in signed 16.16 C.
#endif

#if CONFIG_INTERRUPT
volatile bool irqActive = false; ///< Last published IRQ state.

/** @brief Check all sources sharing the active-low interrupt output. */
bool pendingIRQ() {
  bool active = g_irqFlags != 0;
#if CONFIG_UART
  active |= g_uart_inten && CONFIG_UART_SERCOM.available();
#endif
#if CONFIG_ENCODER
  for (uint8_t encoder = 0; encoder < CONFIG_NUM_ENCODERS; encoder++) {
    active |=
        (encoderInterrupts & (1U << encoder)) && g_enc_delta[encoder] != 0;
  }
#endif
  return active;
}

/** @brief Publish pending GPIO or encoder changes on the interrupt pin. */
void updateIRQ() {
  bool active = pendingIRQ();
  if (active == irqActive) {
    return;
  }
  uint32_t savedMask = __get_PRIMASK();
  __disable_irq();
  // A read-to-clear callback may have run before interrupts were masked.
  active = pendingIRQ();
  if (active) {
    digitalWrite(CONFIG_INTERRUPT_PIN, LOW);
  } else {
    digitalWrite(CONFIG_INTERRUPT_PIN, HIGH);
  }
  irqActive = active;
  __set_PRIMASK(savedMask);
}
#endif

/** @brief Read exposed GPIOs without touching SWD, I2C, or the LED. */
inline uint32_t readGPIO() {
  uint32_t value = 0;
  for (uint8_t pin = 0; pin < SEESAW_STM32_PIN_COUNT; pin++) {
    uint32_t bit = 1UL << pin;
    if ((AVAILABLE_GPIO & bit) && digitalRead(pin) == HIGH) {
      value |= bit;
    }
  }
  return value;
}

/** @brief Apply cached direction, output, and pull state to selected pins. */
void applyGPIO(uint32_t pins) {
  for (uint8_t pin = 0; pin < SEESAW_STM32_PIN_COUNT; pin++) {
    uint32_t bit = 1UL << pin;
    if (!(pins & AVAILABLE_GPIO & bit)) {
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

#if CONFIG_SPI
#include "Adafruit_seesawPeripheral_stm32_spi.h"
#endif

#if CONFIG_UART
/** @brief Read live UART readiness before the first I2C reply byte. */
uint8_t firstUARTStatus() {
  Adafruit_seesawPeripheral_updateUARTStatus();
  return g_uart_status;
}
/** @brief Peek without consuming the byte returned by the request callback. */
uint8_t firstUARTData() { return CONFIG_UART_SERCOM.peek(); }
#endif

/** @brief Select a live first-byte source before a repeated START. */
void prepareFirstByte() {
  static const uint8_t zero = 0, hardwareID = SEESAW_HW_ID;
#if CONFIG_EEPROM
  static const uint8_t unavailable = 0xFF;
#endif
  static const uint8_t versionHigh = (uint32_t)PRODUCT_CODE >> 8;
  seesawWire.setFirstByte(&zero);
  if (selectedBase == SEESAW_STATUS_BASE) {
    if (selectedRegister == SEESAW_STATUS_HW_ID) {
      seesawWire.setFirstByte(&hardwareID);
    } else if (selectedRegister == SEESAW_STATUS_VERSION) {
      seesawWire.setFirstByte(&versionHigh);
    } else if (selectedRegister == SEESAW_STATUS_UID) {
      // STM32 is little-endian; the wire reply starts with UID word 0's MSB.
      seesawWire.setFirstByte((const uint8_t *)UID_BASE + 3);
    }
#if CONFIG_ADC
    else if (selectedRegister == SEESAW_STATUS_TEMP) {
      seesawWire.setFirstByte((const volatile uint8_t *)&temperature + 3);
    }
#endif
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
#if CONFIG_FHT
  else if (selectedBase == SEESAW_SPECTRUM_BASE) {
    if (selectedRegister == SEESAW_SPECTRUM_RESULTS_LOWER) {
      seesawWire.setFirstByte(firstSpectrumLower);
    } else if (selectedRegister == SEESAW_SPECTRUM_RESULTS_UPPER) {
      seesawWire.setFirstByte(&spectrumSnapshot[32]);
    } else if (selectedRegister == SEESAW_SPECTRUM_CHANNEL) {
      seesawWire.setFirstByte(&spectrumChannel);
    } else if (selectedRegister == SEESAW_SPECTRUM_RATE) {
      seesawWire.setFirstByte(&spectrumRate);
    }
  }
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  else if (selectedBase == SEESAW_TIMER_BASE &&
           selectedRegister == SEESAW_TIMER_STATUS) {
    seesawWire.setFirstByte(&g_pwmStatus);
  }
#endif
#if CONFIG_ENCODER
  else if (selectedBase == SEESAW_ENCODER_BASE) {
    uint8_t encoder = selectedRegister & 0x0F;
    uint8_t command = selectedRegister & 0xF0;
    if (encoder < CONFIG_NUM_ENCODERS) {
      if (command == SEESAW_ENCODER_POSITION) {
        seesawWire.setFirstByte(
            (const volatile uint8_t *)&g_enc_value[encoder] + 3);
      } else if (command == SEESAW_ENCODER_DELTA) {
        seesawWire.setFirstByte(
            (const volatile uint8_t *)&g_enc_delta[encoder] + 3);
      }
    }
  }
#endif
#if CONFIG_UART
  else if (selectedBase == SEESAW_SERCOM0_BASE) {
    if (selectedRegister == SEESAW_SERCOM_STATUS) {
      seesawWire.setFirstByte(firstUARTStatus);
    } else if (selectedRegister == SEESAW_SERCOM_INTEN) {
      seesawWire.setFirstByte(&g_uart_inten);
    } else if (selectedRegister == SEESAW_SERCOM_BAUD) {
      seesawWire.setFirstByte((const volatile uint8_t *)&g_uart_baud + 3);
    } else if (selectedRegister == SEESAW_SERCOM_DATA) {
      seesawWire.setFirstByte(firstUARTData);
    }
  }
#endif
#if CONFIG_EEPROM
  else if (selectedBase == SEESAW_EEPROM_BASE) {
    if (eepromSafe) {
      seesawWire.setFirstByte((const uint8_t *)eepromAddress +
                              selectedRegister);
    } else {
      seesawWire.setFirstByte(&unavailable);
    }
  }
#endif
#if CONFIG_SPI
  else if (selectedBase == spiBase) {
    if (selectedRegister == spiStatusRegister) {
      seesawWire.setFirstByte(spiStatusFlags);
    } else if (selectedRegister == spiReadRegister) {
      seesawWire.setFirstByte(spiFirstReceiveByte);
    } else if (selectedRegister == spiConfigRegister) {
      seesawWire.setFirstByte(&spiMode);
    } else if (selectedRegister == spiClockRegister) {
      seesawWire.setFirstByte(spiFirstClockByte);
    }
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
#if CONFIG_SPI
  if (base == spiBase) {
    return true; // Two-byte SPI queries bypass the command queue.
  }
#endif
  return (head + 1) % queueSize != tail;
}

void receive(int length) {
#if CONFIG_SPI
  // Status/RX reads must remain available when the work queue is full.
  if (length == 2 && seesawWire.peek() == spiBase) {
    seesawWire.read();
    uint8_t reg = seesawWire.read();
    selectedBase = spiBase;
    selectedRegister = reg;
    prepareFirstByte();
    if (reg == spiStatusRegister || reg == spiReadRegister ||
        reg == spiConfigRegister || reg == spiClockRegister) {
      return;
    }
    spiError = spiInvalidCommand;
    spiAbortPending = true;
    return;
  }
#endif
  uint8_t next = (head + 1) % queueSize;
  if (length < 2 || length > commandSize || next == tail) {
#if CONFIG_SPI
    if (spiConfigured || spiPending ||
        (length >= 2 && seesawWire.peek() == spiBase)) {
      spiError = spiQueueOverflow;
      spiAbortPending = true;
    }
#endif
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
#if CONFIG_SPI
  if (selectedBase == spiBase) {
    spiPending++;
  }
#endif
  head = next;
}

#if CONFIG_ADC || CONFIG_FHT
/** @brief Calibrate and enable the ADC with bounded waits. */
bool beginADC() {
  // C0/G0 LL sequence: regulator, calibration, two ADC clocks, then enable.
  // HCLK/4 gives 12 MHz on C0 and 16 MHz on G071.
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
  delayMicroseconds(1); // More than two ADC clocks at either supported clock.
  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  started = micros();
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {
    if (micros() - started > adcTimeoutMicros)
      return false;
  }
#if CONFIG_ADC
  // Temperature and VREFINT use internal channels, not GPIO pins. Keep these
  // paths off in spectrum mode, which owns the ADC continuously.
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
                                 LL_ADC_PATH_INTERNAL_TEMPSENSOR |
                                     LL_ADC_PATH_INTERNAL_VREFINT);
  delayMicroseconds(LL_ADC_DELAY_TEMPSENSOR_STAB_US);
#endif
  return true;
}
#endif

#if CONFIG_ADC
/**
 * @brief Convert one external or internal ADC channel with bounded waits.

 * * @param channel LL ADC channel constant.
 * @return 12-bit result, or -1 if
 * initialization or conversion failed.
 */
int32_t readADCChannel(uint32_t channel) {
  if (!adcReady)
    return -1;
  LL_ADC_ClearFlag_CCRDY(ADC1);
  LL_ADC_REG_SetSequencerChannels(ADC1, channel);
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

/** @brief Refresh die temperature outside the I2C interrupt; retain on error.
 */
void readTemperature() {
  int32_t reference = readADCChannel(LL_ADC_CHANNEL_VREFINT);
  int32_t raw = readADCChannel(LL_ADC_CHANNEL_TEMPSENSOR);
  if (reference <= 0 || raw < 0) {
    g_adcStatus = 1;
    return;
  }
  // Normalize to the factory calibration supply, retaining 16 fractional bits.
  // VREFINT compensates for the actual board supply instead of assuming 3.3 V.
  int64_t normalized = (int64_t)raw * *VREFINT_CAL_ADDR * VREFINT_CAL_VREF *
                       65536 / ((int64_t)reference * TEMPSENSOR_CAL_VREFANALOG);
  int64_t delta = normalized - (int64_t)*TEMPSENSOR_CAL1_ADDR * 65536;
#if defined(STM32G0xx)
  // G071 has two factory temperature calibration points.
  int32_t span = (int32_t)*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR;
  if (!span) {
    g_adcStatus = 1;
    return;
  }
  delta = delta * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / span;
#else
  // C0 has only the 30 C calibration point. DS13866/DS13867/DS14721 specify
  // a typical positive slope of 2.53 mV/C (not a precision ambient sensor).
  delta = delta * TEMPSENSOR_CAL_VREFANALOG * 1000 / (4095L * 2530);
#endif
  temperature = TEMPSENSOR_CAL1_TEMP * 65536 + delta;
  g_adcStatus = 0;
}
#endif

/** @brief Reset GPIO and ADC state without writing nonvolatile memory. */
void reset() {
#if CONFIG_FHT
  stopSpectrum();
  pinMode(spectrumChannel, INPUT);
#endif
#if CONFIG_SPI
  spiStop();
  spiError = spiSequence = spiRxLength = 0;
  spiAbortPending = false;
#endif
  direction = output = pull = 0;
#if CONFIG_INTERRUPT
  g_irqGPIO = g_irqFlags = 0;
#endif
  applyGPIO(VALID_GPIO);
#if CONFIG_UART
  g_uart_status = g_uart_inten = g_uart_tx_len = 0;
  g_uart_baud = 9600;
  CONFIG_UART_SERCOM.end();
  CONFIG_UART_SERCOM.setRx(CONFIG_UART_RX_PIN);
  CONFIG_UART_SERCOM.setTx(CONFIG_UART_TX_PIN);
  CONFIG_UART_SERCOM.begin(g_uart_baud);
#endif
#if CONFIG_ENCODER
#if CONFIG_INTERRUPT
  encoderInterrupts = 0;
#endif
  for (uint8_t encoder = 0; encoder < CONFIG_NUM_ENCODERS; encoder++) {
    pinMode(encoderPins[encoder][0], INPUT_PULLUP);
    pinMode(encoderPins[encoder][1], INPUT_PULLUP);
    uint32_t bits =
        (1UL << encoderPins[encoder][0]) | (1UL << encoderPins[encoder][1]);
    pull |= bits;
    output |= bits;
    // The shared decoder uses active-low phases; sample the actual idle state.
    g_enc_prev_pos[encoder] = 3 ^ (digitalRead(encoderPins[encoder][0]) |
                                   (digitalRead(encoderPins[encoder][1]) << 1));
    g_enc_value[encoder] = g_enc_delta[encoder] = 0;
    g_enc_flags[encoder] = 0;
  }
#endif
#if CONFIG_NEOPIXEL
  g_neopixel_pin = 0xFF;
  g_neopixel_bufsize = 0;
  if (pixels.getPixels()) {
    pixels.byteLength(CONFIG_NEOPIXEL_BUF_MAX);
    pixels.clear();
  }
  pixels.byteLength(0);
#endif
#if CONFIG_PWM || CONFIG_PWM_16BIT
  g_pwmStatus = 0;
  pwmFrequency = 1000;
  analogWriteResolution(16);
  analogWriteFrequency(pwmFrequency);
#endif
#if CONFIG_ADC
  g_bufferedADCRead = g_adcStatus = 0;
  temperature = 0;
  adcReady = beginADC();
  if (!adcReady) {
    g_adcStatus = 1;
  } else {
    readTemperature();
  }
#endif
#if CONFIG_FHT
  adcReady = beginSpectrum();
#endif
#if CONFIG_INTERRUPT
  pinMode(CONFIG_INTERRUPT_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(CONFIG_INTERRUPT_PIN, HIGH);
  // Keep a real open-drain output with a weak pull-up in the 3.3 V domain.
  PinName irqPin = digitalPinToPinName(CONFIG_INTERRUPT_PIN);
  pin_function(irqPin, STM_PIN_DATA(STM_MODE_OUTPUT_OD, LL_GPIO_PULL_UP, 0));
  irqActive = false;
#endif
  lastGPIO = readGPIO();
}

/** @brief Apply address straps, falling back to the base on reserved addresses.
 */
uint8_t configuredAddress() {
  uint8_t result = CONFIG_I2C_PERIPH_ADDR;
#if CONFIG_EEPROM
  if (eepromSafe) {
    uint8_t saved = *((const uint8_t *)eepromAddress + eepromSize - 1);
    if (saved >= 8 && saved <= 0x77) {
      result = saved;
    }
  }
#endif
  result = Adafruit_seesawPeripheral_applyAddressStraps(result);
  if (result < 8 || result > 0x77) {
    result = CONFIG_I2C_PERIPH_ADDR;
  }
  return result;
}
} // namespace SeesawSTM32

#if CONFIG_EEPROM
/** @brief Return the emulated EEPROM's protocol-visible byte count. */
uint16_t Adafruit_seesawPeripheral_eepromLength(void) {
  return SeesawSTM32::eepromSize;
}

/**
 * @brief Read one byte from the reserved flash page.
 * @param address EEPROM byte offset.
 * @return Stored byte, or 0xFF if unavailable or outside the EEPROM.
 */
uint8_t Adafruit_seesawPeripheral_readEEPROM(uint16_t address) {
  if (!SeesawSTM32::eepromSafe || address >= SeesawSTM32::eepromSize) {
    return 0xFF;
  }
  return *((const uint8_t *)SeesawSTM32::eepromAddress + address);
}

/** @brief Preserve the entire flash page before staging one command. */
void Adafruit_seesawPeripheral_beginEEPROMWrite(void) {
  using namespace SeesawSTM32;
  eepromDirty = false;
  if (eepromSafe) {
    memcpy(eepromBuffer, (const void *)eepromAddress, FLASH_PAGE_SIZE);
  }
}

/**
 * @brief Stage a byte in the current EEPROM command's page buffer.
 * @param address EEPROM byte offset.
 * @param value Byte to store at commit time.
 */
void Adafruit_seesawPeripheral_writeEEPROM(uint16_t address, uint8_t value) {
  using namespace SeesawSTM32;
  if (eepromSafe && address < eepromSize && eepromBuffer[address] != value) {
    eepromBuffer[address] = value;
    eepromDirty = true;
  }
}

/**
 * @brief Commit changed bytes in one erase, preserving the rest of the page.
 * @return True on success or an unchanged command; false on flash failure.
 */
bool Adafruit_seesawPeripheral_commitEEPROMWrite(void) {
  using namespace SeesawSTM32;
  if (!eepromSafe) {
    return false;
  }
  if (!eepromDirty) {
    return true;
  }
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(ARDUINO_GENERIC_G071GBUX)
  erase.Banks = FLASH_BANK_1;
#endif
  erase.Page = (eepromAddress - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = 1;
  uint32_t pageError;
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
#if defined(ARDUINO_GENERIC_G071GBUX)
  // G0's clear-flag macro accepts one encoded flag, not a mask. This is the
  // same write-one-to-clear error mask used by the G0 HAL flash driver.
  WRITE_REG(FLASH->SR, FLASH_SR_ERRORS);
#else
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
#endif
  bool success = HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
  for (uint16_t pos = 0; success && pos < FLASH_PAGE_SIZE; pos += 8) {
    uint64_t word;
    memcpy(&word, eepromBuffer + pos, sizeof(word));
    if (word != UINT64_MAX) {
      success = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  eepromAddress + pos, word) == HAL_OK;
    }
  }
  HAL_FLASH_Lock();
  return success && memcmp((const void *)eepromAddress, eepromBuffer,
                           FLASH_PAGE_SIZE) == 0;
}
#endif

/** @brief Read the current levels of the selected valid GPIO pins. */
uint32_t Adafruit_seesawPeripheral_readBulk(uint32_t validpins = VALID_GPIO) {
  return SeesawSTM32::readGPIO() & validpins;
}

/** @brief Set or clear output direction while preserving pull configuration. */
void Adafruit_seesawPeripheral_gpioDirection(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  mask &= AVAILABLE_GPIO;
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
  mask &= AVAILABLE_GPIO;
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
  uint32_t mask = AVAILABLE_GPIO;
  output = (output & ~mask) | (value & mask);
  applyGPIO(mask);
}

/** @brief Toggle selected valid GPIO output levels. */
void Adafruit_seesawPeripheral_gpioToggle(uint32_t mask) {
  using namespace SeesawSTM32;
  mask &= AVAILABLE_GPIO;
  output ^= mask;
  applyGPIO(mask);
}

/** @brief Enable or disable pulls without changing the direction register. */
void Adafruit_seesawPeripheral_gpioPull(uint32_t mask, bool enabled) {
  using namespace SeesawSTM32;
  mask &= AVAILABLE_GPIO;
  if (enabled) {
    pull |= mask;
  } else {
    pull &= ~mask;
  }
  applyGPIO(mask);
}

#if CONFIG_INTERRUPT
/** @brief Release the interrupt only after all enabled sources are cleared. */
void Adafruit_seesawPeripheral_clearIRQ(void) { SeesawSTM32::updateIRQ(); }

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

#if CONFIG_ENCODER
/** @brief Enable or disable one encoder's contribution to the shared IRQ. */
void Adafruit_seesawPeripheral_setEncoderInterrupt(uint8_t encoder,
                                                   bool enabled) {
#if CONFIG_INTERRUPT
  if (enabled) {
    SeesawSTM32::encoderInterrupts |= 1U << encoder;
  } else {
    SeesawSTM32::encoderInterrupts &= ~(1U << encoder);
  }
  SeesawSTM32::updateIRQ();
#else
  (void)encoder;
  (void)enabled;
#endif
}
#endif

#if CONFIG_ADC
/**
 * @brief Convert one validated ADC pin.
 * @param pin Protocol GPIO index with ADC support.
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
  // The core's package-specific map also covers PB0/PB1 on the 28-pin board.
  uint32_t function = pinmap_function(digitalPinToPinName(pin), PinMap_ADC);
  uint32_t channel = STM_PIN_CHANNEL(function);
  return readADCChannel(__LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
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
  writePWM(pin, value);
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

#if CONFIG_NEOPIXEL
/** @brief Return the allocated raw-byte buffer used by protocol writes. */
volatile uint8_t *Adafruit_seesawPeripheral_pixelBuffer(void) {
  return SeesawSTM32::pixels.getPixels();
}

/** @brief Send the selected raw bytes at 800 kHz on a valid GPIO pin. */
void Adafruit_seesawPeripheral_showPixels(void) {
  using namespace SeesawSTM32;
  if (!pixels.getPixels() || !g_neopixel_bufsize || g_neopixel_pin >= 32 ||
      !(AVAILABLE_GPIO & (1UL << g_neopixel_pin))) {
    return;
  }
#if CONFIG_PWM || CONFIG_PWM_16BIT
  stopPWM(g_neopixel_pin);
#endif
  pixels.setPin(g_neopixel_pin);
  pixels.byteLength(g_neopixel_bufsize);
  // GPIO/ADC commands may have changed the mode since the previous SHOW.
  // STM32 needs no extra NeoPixel resources; configure this output directly.
  pinMode(g_neopixel_pin, OUTPUT);
  digitalWrite(g_neopixel_pin, LOW);
  pixels.show();
}
#endif

/** @brief Queue writes so ADC and NeoPixel work run outside I2C interrupts. */
void receiveEvent(int howMany) { SeesawSTM32::receive(howMany); }

/** @brief Serve live GPIO levels or the last completed ADC conversion. */
void requestEvent(void) {
  using namespace SeesawSTM32;
  if (selectedBase == SEESAW_GPIO_BASE &&
      selectedRegister == SEESAW_GPIO_BULK) {
    g_bufferedBulkGPIORead = readGPIO();
  }
#if CONFIG_UART
  Adafruit_seesawPeripheral_updateUARTStatus();
#endif
  Adafruit_seesawPeripheral_processRequest(selectedBase, selectedRegister);
#if CONFIG_UART && CONFIG_INTERRUPT
  updateIRQ();
#endif
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

/** @brief Initialize the STM32 backend and dedicated I2C target. */
bool Adafruit_seesawPeripheral_begin(void) {
  using namespace SeesawSTM32;
  Adafruit_seesawPeripheral_setDatecode();
#if CONFIG_EEPROM
  // Include initialized RAM data's flash load image, not just executable code.
  eepromSafe =
      (uintptr_t)&_sidata + ((uintptr_t)&_edata - (uintptr_t)&_sdata) <=
      eepromAddress;
  if (!eepromSafe) {
    return false;
  }
  if (!eepromBuffer) {
    eepromBuffer = (uint8_t *)malloc(FLASH_PAGE_SIZE);
  }
  if (!eepromBuffer) {
    eepromSafe = false;
    return false;
  }
#endif
  reset();
#if CONFIG_NEOPIXEL
  if (!pixels.getPixels()) {
    return false;
  }
#endif
#if CONFIG_ADC || CONFIG_FHT
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
#if CONFIG_FHT
  runSpectrum();
#endif
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
    // Test-only delay for deterministic queue-overflow checks; never ship it.
    if (command.data[0] == spiBase) {
      delayMicroseconds(CONFIG_SPI_TEST_HOLD_US);
    }
#endif
    Adafruit_seesawPeripheral_processCommand(command.data, command.length);
  }
#if CONFIG_UART
  if (g_uart_tx_len > 0) {
    CONFIG_UART_SERCOM.write((const uint8_t *)g_uart_buf, g_uart_tx_len);
    g_uart_tx_len = 0;
  }
#endif
#if CONFIG_INTERRUPT || CONFIG_ENCODER
  uint32_t current = readGPIO();
#if CONFIG_ENCODER
  for (uint8_t encoder = 0; encoder < CONFIG_NUM_ENCODERS; encoder++) {
    uint8_t phase = 3 ^ (((current >> encoderPins[encoder][0]) & 1U) |
                         (((current >> encoderPins[encoder][1]) & 1U) << 1));
    Adafruit_seesawPeripheral_updateEncoder(encoder, phase);
  }
#endif
#if CONFIG_INTERRUPT
  uint32_t changed = (current ^ lastGPIO) & g_irqGPIO & AVAILABLE_GPIO;
  lastGPIO = current;
  if (changed) {
    uint32_t savedMask = __get_PRIMASK();
    __disable_irq();
    g_irqFlags |= changed;
    __set_PRIMASK(savedMask);
  }
  updateIRQ();
#endif
#endif
}
#endif
