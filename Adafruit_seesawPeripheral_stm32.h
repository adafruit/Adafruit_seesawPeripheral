/*!
 * @file Adafruit_seesawPeripheral_stm32.h
 * STM32duino backend. Slow peripheral work runs outside Wire interrupts.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_H

#if !defined(ARDUINO_GENERIC_C011F6UX)
#error "The STM32 backend currently supports the generic STM32C011F6Ux only"
#endif

#ifndef CONFIG_ADC
#define CONFIG_ADC 0 ///< Enable ADC channels.
#endif
#ifndef CONFIG_PWM
#if CONFIG_PWM_16BIT
#define CONFIG_PWM 1 ///< STM32 PWM always accepts the 16-bit seesaw duty value.
#else
#define CONFIG_PWM 0 ///< Enable PWM channels.
#endif
#endif
#if CONFIG_ADC
#include "stm32c0xx_ll_adc.h"
#endif
#ifndef CONFIG_EEPROM
#define CONFIG_EEPROM 0 ///< Enable the reserved-flash EEPROM window.
#endif
#ifndef CONFIG_NEOPIXEL
#define CONFIG_NEOPIXEL 0 ///< Enable raw NeoPixel buffer output.
#endif
#ifndef CONFIG_UART
#define CONFIG_UART 0 ///< Enable the UART bridge.
#endif
#ifndef CONFIG_SPI
#define CONFIG_SPI 0 ///< Enable the experimental I2C-to-SPI controller bridge.
#endif
#if CONFIG_SPI
#include <SPI.h>
#endif
#ifndef CONFIG_UART_DEBUG
#define CONFIG_UART_DEBUG                                                      \
  0 ///< Reserve the default UART pins for sketch logging.
#endif
#ifndef CONFIG_ENCODER
#define CONFIG_ENCODER 0 ///< Enable quadrature encoders.
#endif
#if CONFIG_FHT
#error "The AVR assembly FHT backend is not available on STM32"
#endif
#if CONFIG_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#ifndef CONFIG_NEOPIXEL_BUF_MAX
#define CONFIG_NEOPIXEL_BUF_MAX 192 ///< Default raw LED buffer capacity.
#endif
#endif
#if CONFIG_UART || CONFIG_UART_DEBUG
#ifndef CONFIG_UART_SERCOM
#define CONFIG_UART_SERCOM Serial ///< UART implementation.
#endif
#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 1 ///< PA1, USART1 RX.
#endif
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 0 ///< PA0, USART1 TX.
#endif
#endif

#if CONFIG_EEPROM
extern "C" uint8_t _sidata, _sdata, _edata;
#endif

namespace SeesawSTM32 {
const uint8_t hardwareID = 0x90;        ///< Provisional STM32C011 hardware ID.
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
uint32_t irqEnabled = 0, lastGPIO = 0;        ///< Change interrupt state.
volatile uint32_t irqFlags = 0; ///< Latched changes, cleared by Wire reads.
uint32_t pwmActive = 0;         ///< Pins currently using timers.
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
volatile uint16_t adcValue = 0;                  ///< Completed ADC conversion.
volatile uint8_t adcStatus = 0, timerStatus = 0; ///< Peripheral error flags.
#if CONFIG_ADC
bool adcReady = false; ///< Calibration and enable completed.
const uint32_t adcTimeoutMicros =
    5000; ///< Bounded calibration/conversion waits.
#endif
uint32_t version = 0; ///< Product/date code, compatible with existing seesaw.
constexpr uint16_t buildDate(const char *date) {
  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  uint8_t month = 1;
  for (; month <= 12; month++) {
    uint8_t pos = (month - 1) * 3;
    if (date[0] == months[pos] && date[1] == months[pos + 1] &&
        date[2] == months[pos + 2])
      break;
  }
  uint8_t day = (date[4] == ' ' ? 0 : date[4] - '0') * 10 + date[5] - '0';
  uint8_t year = (date[9] - '0') * 10 + date[10] - '0';
  return (day << 11) | (month << 7) | year;
}
uint8_t address = CONFIG_I2C_PERIPH_ADDR; ///< Current effective I2C address.
#if CONFIG_EEPROM
const uint32_t eepromAddress = FLASH_BASE + 30 * 1024; ///< Last 2 KB page.
const uint16_t eepromSize = 256;    ///< Wire-addressable EEPROM bytes.
const uint8_t addressOffset = 0xFF; ///< Persistent I2C address byte.
uint8_t eepromBuffer[eepromSize]
    __attribute__((aligned(8))); ///< Page workspace.
bool eepromSafe = false; ///< True only if program data ends before this page.
#endif
#if CONFIG_UART
volatile uint32_t uartBaud = 9600;  ///< UART bit rate, exposed to Wire reads.
volatile uint8_t uartInterrupt = 0; ///< RX-ready interrupt enable.
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
volatile int32_t encoderPosition[CONFIG_NUM_ENCODERS] =
    {}; ///< Absolute counts.
volatile int32_t encoderDelta[CONFIG_NUM_ENCODERS] =
    {}; ///< Unread count changes.
uint8_t encoderPrevious[CONFIG_NUM_ENCODERS] =
    {};                                        ///< Previous quadrature phase.
int8_t encoderSteps[CONFIG_NUM_ENCODERS] = {}; ///< Partial-detent counts.
volatile uint8_t encoderInterrupts = 0;        ///< Interrupt enable bitmap.
#endif
#if CONFIG_NEOPIXEL
class RawPixels : public Adafruit_NeoPixel {
public:
  RawPixels()
      : Adafruit_NeoPixel((CONFIG_NEOPIXEL_BUF_MAX + 2) / 3, -1,
                          NEO_RGB + NEO_KHZ800) {}
  void byteLength(uint16_t length) { numBytes = length; }
}; ///< Uses NeoPixel's STM32 timing with unmodified raw RGB/RGBW byte order.
RawPixels pixels;         ///< Fixed-capacity LED buffer, allocated once.
uint16_t pixelLength = 0; ///< Requested byte count.
uint8_t pixelPin = 0xFF,
        pixelStatus = 0; ///< Selected pin and validation status.
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
#ifdef CONFIG_ADDR_0_PIN
  pins &= ~(1UL << CONFIG_ADDR_0_PIN);
#endif
#ifdef CONFIG_ADDR_1_PIN
  pins &= ~(1UL << CONFIG_ADDR_1_PIN);
#endif
#ifdef CONFIG_ADDR_2_PIN
  pins &= ~(1UL << CONFIG_ADDR_2_PIN);
#endif
#ifdef CONFIG_ADDR_3_PIN
  pins &= ~(1UL << CONFIG_ADDR_3_PIN);
#endif
  return pins;
}

void updateIRQ() {
#ifdef CONFIG_INTERRUPT_PIN
  // A real open-drain output; the local pull-up is in the 3.3 V domain.
  bool active = irqFlags != 0;
#if CONFIG_UART
  active |= uartInterrupt && CONFIG_UART_SERCOM.available();
#endif
#if CONFIG_ENCODER
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    active |= (encoderInterrupts & (1 << i)) && encoderDelta[i] != 0;
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

uint32_t read32(const uint8_t *data) {
  return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
         ((uint32_t)data[2] << 8) | data[3];
}

void write32(uint32_t value) {
  uint8_t data[4] = {(uint8_t)(value >> 24), (uint8_t)(value >> 16),
                     (uint8_t)(value >> 8), (uint8_t)value};
  Wire.write(data, sizeof(data));
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

void request() {
#if CONFIG_SPI
  if (selectedBase == spiBase) {
    spiRequest(selectedRegister);
    return;
  }
#endif
  if (selectedBase == SEESAW_STATUS_BASE) {
    switch (selectedRegister) {
    case SEESAW_STATUS_HW_ID:
      Wire.write(hardwareID);
      break;
    case SEESAW_STATUS_VERSION:
      write32(version);
      break;
    case SEESAW_STATUS_OPTIONS:
      write32((1UL << SEESAW_STATUS_BASE) | (1UL << SEESAW_GPIO_BASE) |
              ((uint32_t)CONFIG_ADC << SEESAW_ADC_BASE) |
              ((uint32_t)CONFIG_PWM << SEESAW_TIMER_BASE) |
              ((uint32_t)CONFIG_EEPROM << SEESAW_EEPROM_BASE) |
              ((uint32_t)CONFIG_NEOPIXEL << SEESAW_NEOPIXEL_BASE) |
              ((uint32_t)CONFIG_UART << SEESAW_SERCOM0_BASE) |
              ((uint32_t)CONFIG_ENCODER << SEESAW_ENCODER_BASE)
#if CONFIG_SPI
              | (1UL << spiBase)
#endif
      );
      break;
    default:
      Wire.write((uint8_t)0);
    }
  } else if (selectedBase == SEESAW_GPIO_BASE) {
    if (selectedRegister == SEESAW_GPIO_BULK) {
      write32(readGPIO());
      irqFlags = 0;
      updateIRQ();
    } else if (selectedRegister == SEESAW_GPIO_INTFLAG) {
      write32(irqFlags);
      irqFlags = 0;
      updateIRQ();
    }
#if CONFIG_ADC
  } else if (selectedBase == SEESAW_ADC_BASE) {
    if (selectedRegister == SEESAW_ADC_STATUS) {
      Wire.write(adcStatus);
    } else if (selectedRegister >= SEESAW_ADC_CHANNEL_OFFSET) {
      Wire.write((uint8_t)(adcValue >> 8));
      Wire.write((uint8_t)adcValue);
    }
#endif
#if CONFIG_PWM
  } else if (selectedBase == SEESAW_TIMER_BASE) {
    Wire.write(timerStatus);
#endif
#if CONFIG_EEPROM
  } else if (selectedBase == SEESAW_EEPROM_BASE) {
    // Reads are live flash contents, not an optimistic write cache.
    uint8_t value = 0xFF;
    if (eepromSafe)
      value = *((const uint8_t *)eepromAddress + selectedRegister);
    Wire.write(value);
#endif
#if CONFIG_NEOPIXEL
  } else if (selectedBase == SEESAW_NEOPIXEL_BASE) {
    Wire.write(pixelStatus);
#endif
#if CONFIG_UART
  } else if (selectedBase == SEESAW_SERCOM0_BASE) {
    switch (selectedRegister) {
    case SEESAW_SERCOM_STATUS:
      Wire.write((uint8_t)(CONFIG_UART_SERCOM.available() ? 2 : 0));
      break;
    case SEESAW_SERCOM_INTEN:
      Wire.write(uartInterrupt);
      break;
    case SEESAW_SERCOM_BAUD:
      write32(uartBaud);
      break;
    case SEESAW_SERCOM_DATA:
      Wire.write((uint8_t)CONFIG_UART_SERCOM.read());
      break;
    }
#endif
#if CONFIG_ENCODER
  } else if (selectedBase == SEESAW_ENCODER_BASE) {
    uint8_t index = selectedRegister & 0x0F;
    uint8_t reg = selectedRegister & 0xF0;
    if (index < CONFIG_NUM_ENCODERS &&
        (reg == SEESAW_ENCODER_POSITION || reg == SEESAW_ENCODER_DELTA)) {
      write32(reg == SEESAW_ENCODER_POSITION ? encoderPosition[index]
                                             : encoderDelta[index]);
      encoderDelta[index] = 0;
      updateIRQ();
    }
#endif
  } else {
    Wire.write((uint8_t)0);
  }
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
  irqEnabled = irqFlags = 0;
  adcValue = adcStatus = timerStatus = 0;
  pwmFrequency = 1000;
  applyGPIO(validGPIO());
#if CONFIG_ADC
  adcReady = beginADC();
  if (!adcReady)
    adcStatus = 1;
#endif
#if CONFIG_NEOPIXEL
  pixelLength = 0;
  pixelPin = 0xFF;
  pixels.setPin(-1);
  pixels.byteLength(CONFIG_NEOPIXEL_BUF_MAX);
  pixels.clear();
  pixels.byteLength(0);
  pixelStatus = pixels.getPixels() ? 0 : 1;
#endif
#if CONFIG_UART
  uartInterrupt = 0;
  uartBaud = 9600;
  CONFIG_UART_SERCOM.end();
  CONFIG_UART_SERCOM.setRx(CONFIG_UART_RX_PIN);
  CONFIG_UART_SERCOM.setTx(CONFIG_UART_TX_PIN);
  CONFIG_UART_SERCOM.begin(uartBaud);
#endif
#if CONFIG_ENCODER
  encoderInterrupts = 0;
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    pinMode(encoderPins[i][0], INPUT_PULLUP);
    pinMode(encoderPins[i][1], INPUT_PULLUP);
    uint32_t bits = (1UL << encoderPins[i][0]) | (1UL << encoderPins[i][1]);
    pull |= bits;
    output |= bits;
    encoderPrevious[i] =
        digitalRead(encoderPins[i][0]) | (digitalRead(encoderPins[i][1]) << 1);
    encoderPosition[i] = encoderDelta[i] = 0;
    encoderSteps[i] = 0;
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
#ifdef CONFIG_ADDR_INVERTED
  const uint8_t activeLevel = HIGH;
#else
  const uint8_t activeLevel = LOW;
#endif
#ifdef CONFIG_ADDR_0_PIN
  pinMode(CONFIG_ADDR_0_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_0_PIN) == activeLevel)
    result += 1;
#endif
#ifdef CONFIG_ADDR_1_PIN
  pinMode(CONFIG_ADDR_1_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_1_PIN) == activeLevel)
    result += 2;
#endif
#ifdef CONFIG_ADDR_2_PIN
  pinMode(CONFIG_ADDR_2_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_2_PIN) == activeLevel)
    result += 4;
#endif
#ifdef CONFIG_ADDR_3_PIN
  pinMode(CONFIG_ADDR_3_PIN, INPUT_PULLUP);
  if (digitalRead(CONFIG_ADDR_3_PIN) == activeLevel)
    result += 8;
#endif
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

void process(const Command &command) {
  uint8_t base = command.data[0], reg = command.data[1];
  const uint8_t *data = command.data + 2;
  uint8_t length = command.length - 2;
#if CONFIG_SPI
  if (base == spiBase) {
    spiProcess(reg, data, length);
    noInterrupts();
    if (spiPending)
      spiPending--;
    interrupts();
    return;
  }
#endif
  if (base == SEESAW_STATUS_BASE && reg == SEESAW_STATUS_SWRST && length == 1 &&
      data[0] == 0xFF) {
    reset();
    uint8_t newAddress = configuredAddress();
    if (newAddress != address) {
      Wire.end();
      address = newAddress;
      Wire.begin((int)address);
      Wire.onReceive(receive);
      Wire.onRequest(request);
    }
  } else if (base == SEESAW_GPIO_BASE && length == 4) {
    uint32_t bits = read32(data) & validGPIO();
    switch (reg) {
    case SEESAW_GPIO_DIRSET_BULK:
      direction |= bits;
      break;
    case SEESAW_GPIO_DIRCLR_BULK:
      direction &= ~bits;
      break;
    case SEESAW_GPIO_BULK:
      output = bits;
      bits = validGPIO();
      break;
    case SEESAW_GPIO_BULK_SET:
      output |= bits;
      break;
    case SEESAW_GPIO_BULK_CLR:
      output &= ~bits;
      break;
    case SEESAW_GPIO_BULK_TOGGLE:
      output ^= bits;
      break;
    case SEESAW_GPIO_PULLENSET:
      pull |= bits;
      break;
    case SEESAW_GPIO_PULLENCLR:
      pull &= ~bits;
      break;
    case SEESAW_GPIO_INTENSET:
      lastGPIO = readGPIO();
      irqEnabled |= bits;
      return;
    case SEESAW_GPIO_INTENCLR:
      irqEnabled &= ~bits;
      irqFlags &= ~bits;
      updateIRQ();
      return;
    default:
      return;
    }
    applyGPIO(bits);
#if CONFIG_ADC
  } else if (base == SEESAW_ADC_BASE && length == 0 &&
             reg >= SEESAW_ADC_CHANNEL_OFFSET) {
    uint8_t pin = reg - SEESAW_ADC_CHANNEL_OFFSET;
    adcStatus = 1;
    adcValue = 0;
    if (pin < 16 && (adcMask & validGPIO() & (1UL << pin))) {
      uint16_t value;
      if (readADC(pin, value)) {
        adcValue = value;
        adcStatus = 0;
      }
    }
#endif
#if CONFIG_EEPROM
  } else if (base == SEESAW_EEPROM_BASE && length > 0) {
    writeEEPROM(reg, data, length);
#endif
#if CONFIG_NEOPIXEL
  } else if (base == SEESAW_NEOPIXEL_BASE) {
    pixelStatus = 0;
    if (reg == SEESAW_NEOPIXEL_PIN && length == 1) {
      if (data[0] >= 16 || !(validGPIO() & (1UL << data[0]))) {
        pixelStatus = 1;
        return;
      }
      pixelPin = data[0];
      pixels.setPin(pixelPin);
    } else if (reg == SEESAW_NEOPIXEL_SPEED && length == 1) {
      // Like the AVR peripheral, this backend supports 800 kHz only.
      if (data[0] != 1)
        pixelStatus = 1;
    } else if (reg == SEESAW_NEOPIXEL_BUF_LENGTH && length == 2) {
      pixelLength = ((uint16_t)data[0] << 8) | data[1];
      if (pixelLength > CONFIG_NEOPIXEL_BUF_MAX)
        pixelLength = CONFIG_NEOPIXEL_BUF_MAX;
      pixels.byteLength(pixelLength);
    } else if (reg == SEESAW_NEOPIXEL_BUF && length >= 2) {
      uint16_t offset = ((uint16_t)data[0] << 8) | data[1];
      if (!pixels.getPixels() ||
          (uint32_t)offset + length - 2 > CONFIG_NEOPIXEL_BUF_MAX) {
        pixelStatus = 1;
        return;
      }
      memcpy(pixels.getPixels() + offset, data + 2, length - 2);
    } else if (reg == SEESAW_NEOPIXEL_SHOW && length == 0) {
      if (pixelPin == 0xFF || !pixels.getPixels() || pixelLength == 0) {
        pixelStatus = 1;
        return;
      }
#if CONFIG_PWM
      if (pwmActive & (1UL << pixelPin))
        pinMode(pwmPins[pixelPin], INPUT);
#endif
      pinMode(pixelPin, OUTPUT);
      pwmActive &= ~(1UL << pixelPin);
      pixels.show();
    }
#endif
#if CONFIG_UART
  } else if (base == SEESAW_SERCOM0_BASE) {
    if (reg == SEESAW_SERCOM_INTEN && length == 1)
      uartInterrupt |= data[0] & 1;
    else if (reg == SEESAW_SERCOM_INTENCLR && length == 1)
      uartInterrupt &= ~(data[0] & 1);
    else if (reg == SEESAW_SERCOM_BAUD && length == 4) {
      uint32_t baud = read32(data);
      if (baud >= 300 && baud <= 1000000) {
        uartBaud = baud;
        CONFIG_UART_SERCOM.end();
        CONFIG_UART_SERCOM.begin(baud);
      }
    } else if (reg == SEESAW_SERCOM_DATA && length > 0) {
      CONFIG_UART_SERCOM.write(data, length);
    }
#endif
#if CONFIG_ENCODER
  } else if (base == SEESAW_ENCODER_BASE) {
    uint8_t index = reg & 0x0F;
    if (index >= CONFIG_NUM_ENCODERS)
      return;
    switch (reg & 0xF0) {
    case SEESAW_ENCODER_INTENSET:
      encoderInterrupts |= 1 << index;
      break;
    case SEESAW_ENCODER_INTENCLR:
      encoderInterrupts &= ~(1 << index);
      break;
    case SEESAW_ENCODER_POSITION:
      if (length == 4) {
        noInterrupts();
        encoderPosition[index] = (int32_t)read32(data);
        encoderDelta[index] = 0;
        interrupts();
      }
      break;
    }
#endif
#if CONFIG_PWM
  } else if (base == SEESAW_TIMER_BASE && length == 3) {
    uint8_t pin = data[0];
    uint16_t value = ((uint16_t)data[1] << 8) | data[2];
    timerStatus = 1;
    if (pin >= 16 || !(pwmMask & validGPIO() & (1UL << pin))) {
      return;
    }
    if (reg == SEESAW_TIMER_PWM) {
      pwmValues[pin] = value;
      analogWriteFrequency(pwmFrequency);
      analogWrite(pwmPins[pin], value);
      pwmActive |= 1UL << pin;
      timerStatus = 0;
    } else if (reg == SEESAW_TIMER_FREQ && value != 0) {
      // Channels share hardware timers: keep one documented global frequency.
      pwmFrequency = value;
      analogWriteFrequency(value);
      for (uint8_t activePin = 0; activePin < 16; activePin++) {
        if (pwmActive & (1UL << activePin)) {
          analogWrite(pwmPins[activePin], pwmValues[activePin]);
        }
      }
      timerStatus = 0;
    }
#endif
  }
}
} // namespace SeesawSTM32

bool Adafruit_seesawPeripheral_begin() {
  using namespace SeesawSTM32;
  constexpr uint16_t dateCode = buildDate(__DATE__);
  version = ((uint32_t)PRODUCT_CODE << 16) | dateCode;
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
  Wire.onReceive(receive);
  Wire.onRequest(request);
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
    process(command);
  }
#if defined(CONFIG_INTERRUPT_PIN) || CONFIG_ENCODER
  // Snapshot and flag updates must not race the read-to-clear Wire callback.
  noInterrupts();
  uint32_t current = readGPIO();
#if CONFIG_ENCODER
  // Reject impossible two-bit transitions; count only complete detents.
  const int8_t transitions[16] = {0, 1, -1, 0,  -1, 0,  0, 1,
                                  1, 0, 0,  -1, 0,  -1, 1, 0};
  for (uint8_t i = 0; i < CONFIG_NUM_ENCODERS; i++) {
    uint8_t phase =
        digitalRead(encoderPins[i][0]) | (digitalRead(encoderPins[i][1]) << 1);
    uint8_t previous = encoderPrevious[i];
    if ((phase ^ previous) == 3)
      encoderSteps[i] = 0;
    else
      encoderSteps[i] += transitions[(previous << 2) | phase];
    encoderPrevious[i] = phase;
#if CONFIG_ENCODER_2TICKS
    const int8_t ticks = 2;
    bool detent = phase == 0 || phase == 3;
#else
    const int8_t ticks = 4;
    bool detent = phase == 3;
#endif
    if (detent && (encoderSteps[i] >= ticks || encoderSteps[i] <= -ticks)) {
      int8_t step = encoderSteps[i] > 0 ? 1 : -1;
      encoderPosition[i] = (int32_t)((uint32_t)encoderPosition[i] + step);
      encoderDelta[i] = (int32_t)((uint32_t)encoderDelta[i] + step);
      encoderSteps[i] = 0;
    }
  }
#endif
  irqFlags |= (current ^ lastGPIO) & irqEnabled;
  lastGPIO = current;
  updateIRQ();
  interrupts();
#endif
}
#endif
