/*!
 * @file Adafruit_seesawPeripheral_stm32_config.h
 * @brief Pin capabilities for supported STM32 seesaw breakouts.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H

#if defined(ARDUINO_GENERIC_C011F6UX)
#define SEESAW_HW_ID 0x90     ///< Provisional STM32C011 hardware ID.
#define ALL_GPIO 0x0000D9FFUL ///< PA0..8, PA11/12, PC14/15; reserve SWD/I2C.
#define ALL_ADC 0x000019FFUL  ///< D0..8, D11/12 match PA0..8, PA11/12.
#define ALL_PWM 0x0000C9FFUL  ///< All exposed pins except D12 / PA12 have PWM.
#define SEESAW_STM32_PIN_COUNT 16 ///< Last usable GPIO is D15 / PC15 LED.
#define SEESAW_STM32_FLASH_BYTES 32768UL ///< Physical flash capacity.
#elif defined(ARDUINO_GENERIC_C031G6UX)
#define SEESAW_HW_ID 0x91                ///< Provisional STM32C031 hardware ID.
#define SEESAW_STM32_FLASH_BYTES 32768UL ///< Physical flash capacity.
#define ALL_GPIO 0xC17B99FFUL ///< PA0..8/11/12/15, PB0/1/3/4/5/8, PC6/14/15.
#define ALL_ADC 0x000319FFUL  ///< D0..8, D11/12, D16/17.
#define ALL_PWM 0xC17B89FFUL ///< All exposed GPIOs except D12/PA12 have timers.
#define SEESAW_STM32_PIN_COUNT 32 ///< Last usable GPIO is D31 / PC15 LED.
#elif defined(ARDUINO_GENERIC_C051G8UX)
#define SEESAW_HW_ID 0x92                ///< Provisional STM32C051 hardware ID.
#define SEESAW_STM32_FLASH_BYTES 65536UL ///< Physical flash capacity.
// D0..10: PA0..8, PA11/12; D13..18: PA15, PB0/1/3/4/5;
// D21..23: PB8, PC6, PC14. Reserve SWD, I2C, reset, aliases and PC15 LED.
#define ALL_GPIO 0x00E7E7FFUL
#define ALL_PWM 0x00E7E3FFUL
#define SEESAW_STM32_PIN_COUNT 24
#define ALL_ADC 0x0000C7FFUL ///< PA0..8, PA11/12, PB0/1.
#elif defined(ARDUINO_GENERIC_G071GBUX)
#define SEESAW_HW_ID 0x93     ///< Provisional STM32G071 hardware ID.
#define ALL_GPIO 0x00E7E7FFUL ///< Same exposed 28-pin GPIO map as C051.
#define ALL_ADC 0x0000C0FFUL  ///< PA0..7 and PB0/1; PA8/11/12 are digital only.
#define ALL_PWM 0x0067E3FFUL  ///< PA12 and PC14 have no timer output.
#define SEESAW_STM32_PIN_COUNT 24         ///< Last usable GPIO is D23 / PC14.
#define SEESAW_STM32_FLASH_BYTES 131072UL ///< Physical flash capacity.
#else
#error                                                                         \
    "Select a supported generic C011F6Ux, C031G6Ux, C051G8Ux, or G071GBUx target"
#endif
#if defined(ARDUINO_GENERIC_G071GBUX)
#define SEESAW_STM32_PCLK 64000000UL ///< Generic G071's peripheral clock.
#else
#define SEESAW_STM32_PCLK 48000000UL ///< Required peripheral clock in hertz.
#endif

namespace SeesawSTM32 {
/**
 * @brief Translate a protocol GPIO number to the Arduino core pin index.
 * @param pin Protocol GPIO number, validated by the caller's capability mask.
 * @return Core pin index, or PNUM_NOT_DEFINED for a reserved protocol number.
 */
constexpr uint32_t corePin(uint8_t pin) {
#if defined(ARDUINO_GENERIC_C011F6UX)
  switch (pin) {
  case 11:
    return PA11;
  case 12:
    return PA12;
  case 14:
    return PC14;
  case 15:
    return PC15;
  default:
    if (pin <= 8) {
      return pin;
    }
    return PNUM_NOT_DEFINED;
  }
#elif defined(ARDUINO_GENERIC_C031G6UX)
  switch (pin) {
  case 11:
    return PA11;
  case 12:
    return PA12;
  case 15:
    return PA15;
  case 16:
    return PB0;
  case 17:
    return PB1;
  case 19:
    return PB3;
  case 20:
    return PB4;
  case 21:
    return PB5;
  case 22:
    return PC6;
  case 24:
    return PB8;
  case 30:
    return PC14;
  case 31:
    return PC15;
  default:
    if (pin <= 8) {
      return pin;
    }
    return PNUM_NOT_DEFINED;
  }
#else
  return pin;
#endif
}

/**
 * @brief Find the protocol bit for a physical core pin used by I2C.
 * @param pin Arduino core pin index, not a protocol GPIO number.
 * @return GPIO protocol mask, or zero when the core pin is not exposed.
 */
constexpr uint32_t protocolPinMask(uint32_t pin) {
  // GPIO masks are protocol bitfields; the core's dense indices may differ.
  for (uint8_t index = 0; index < SEESAW_STM32_PIN_COUNT; index++) {
    if ((ALL_GPIO & (1UL << index)) && corePin(index) == pin) {
      return 1UL << index;
    }
  }
  return 0;
}
} // namespace SeesawSTM32

#define UART_DEBUG_RXD 1
#define UART_DEBUG_TXD 0

#if CONFIG_UART
#ifndef CONFIG_UART_SERCOM
#define CONFIG_UART_SERCOM Serial
#endif
#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 1 // PA1, default UART RX (USART4 on G071).
#endif
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 0 // PA0, default UART TX (USART4 on G071).
#endif
#define SEESAW_UART_PIN_MASK                                                   \
  ((1UL << CONFIG_UART_RX_PIN) | (1UL << CONFIG_UART_TX_PIN))
#else
#define SEESAW_UART_PIN_MASK 0UL
#endif

#ifndef CONFIG_I2C_SDA_PIN
#define CONFIG_I2C_SDA_PIN PB7
#endif
#ifndef CONFIG_I2C_SCL_PIN
#define CONFIG_I2C_SCL_PIN PB6
#endif
#define SEESAW_I2C_PIN_MASK                                                    \
  (SeesawSTM32::protocolPinMask(CONFIG_I2C_SDA_PIN) |                          \
   SeesawSTM32::protocolPinMask(CONFIG_I2C_SCL_PIN))
static_assert(CONFIG_I2C_PERIPH_ADDR >= 8 && CONFIG_I2C_PERIPH_ADDR <= 0x77,
              "Use a non-reserved 7-bit I2C peripheral address");
#endif
