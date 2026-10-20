/*!
 * @file Adafruit_seesawPeripheral_stm32_config.h
 * @brief Pin capabilities for supported STM32 seesaw breakouts.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H

#if defined(ARDUINO_GENERIC_C011F6UX)
#define SEESAW_HW_ID 0x90     ///< Provisional STM32C011 hardware ID.
#define ALL_GPIO 0x000087FFUL ///< D0..10 and D15; SWD/I2C/LED stay reserved.
#define ALL_ADC 0x000007FFUL  ///< D0..8 = PA0..8, D9/10 = PA11/12.
#define ALL_PWM 0x000083FFUL  ///< Exposed timer outputs; PA12 has no PWM.
#define SEESAW_STM32_PIN_COUNT 16        ///< Last usable GPIO is D15 / PC14.
#define SEESAW_STM32_FLASH_BYTES 32768UL ///< Physical flash capacity.
#elif defined(ARDUINO_GENERIC_C031G6UX) || defined(ARDUINO_GENERIC_C051G8UX)
#if defined(ARDUINO_GENERIC_C031G6UX)
#define SEESAW_HW_ID 0x91                ///< Provisional STM32C031 hardware ID.
#define SEESAW_STM32_FLASH_BYTES 32768UL ///< Physical flash capacity.
#else
#define SEESAW_HW_ID 0x92                ///< Provisional STM32C051 hardware ID.
#define SEESAW_STM32_FLASH_BYTES 65536UL ///< Physical flash capacity.
#endif
// D0..10: PA0..8, PA11/12; D13..18: PA15, PB0/1/3/4/5;
// D21..23: PB8, PC6, PC14. Reserve SWD, I2C, PC15 LED, reset and remap aliases.
#define ALL_GPIO 0x00E7E7FFUL
#define ALL_ADC 0x0000C7FFUL ///< PA0..8, PA11/12, PB0/1.
#define ALL_PWM 0x00E7E3FFUL ///< All exposed GPIOs except PA12 have timers.
#define SEESAW_STM32_PIN_COUNT 24 ///< Last usable GPIO is D23 / PC14.
#else
#error "Select a supported generic STM32C011F6Ux, C031G6Ux, or C051G8Ux target"
#endif
#define SEESAW_STM32_PCLK 48000000UL ///< Required peripheral clock in hertz.

#define UART_DEBUG_RXD 1
#define UART_DEBUG_TXD 0

#if CONFIG_UART
#ifndef CONFIG_UART_SERCOM
#define CONFIG_UART_SERCOM Serial
#endif
#ifndef CONFIG_UART_RX_PIN
#define CONFIG_UART_RX_PIN 1 // PA1, USART1 RX.
#endif
#ifndef CONFIG_UART_TX_PIN
#define CONFIG_UART_TX_PIN 0 // PA0, USART1 TX.
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
static_assert(CONFIG_I2C_PERIPH_ADDR >= 8 && CONFIG_I2C_PERIPH_ADDR <= 0x77,
              "Use a non-reserved 7-bit I2C peripheral address");
#endif
