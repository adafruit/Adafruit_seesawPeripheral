/*!
 * @file Adafruit_seesawPeripheral_stm32_config.h
 * @brief Pin capabilities for the generic STM32C011F6Ux target.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_CONFIG_H

#if !defined(ARDUINO_GENERIC_C011F6UX)
#error "The STM32 backend currently supports the generic STM32C011F6Ux only"
#endif
#define SEESAW_HW_ID 0x90     ///< Provisional STM32C011 hardware ID.
#define ALL_GPIO 0x000087FFUL ///< D0..10 and D15; SWD/I2C/LED stay reserved.
#define ALL_ADC 0x000007FFUL  ///< D0..8 = PA0..8, D9/10 = PA11/12.
#define ALL_PWM 0x000083FFUL  ///< Exposed timer outputs; PA12 has no PWM.
#define UART_DEBUG_RXD 1
#define UART_DEBUG_TXD 0

#ifndef CONFIG_I2C_SDA_PIN
#define CONFIG_I2C_SDA_PIN PB7
#endif
#ifndef CONFIG_I2C_SCL_PIN
#define CONFIG_I2C_SCL_PIN PB6
#endif
static_assert(CONFIG_I2C_PERIPH_ADDR >= 8 && CONFIG_I2C_PERIPH_ADDR <= 0x77,
              "Use a non-reserved 7-bit I2C peripheral address");
#endif
