/*!
 * @file Adafruit_seesawPeripheral_stm32_registers.h
 * @brief Wire protocol constants used by the STM32 peripheral backend.
 *
 * These values match Adafruit_seesaw.h. Keeping the peripheral's constants
 * separate avoids pulling the host-side BusIO/Serial implementation into a
 * small device that only needs to respond to I2C requests.
 */
#ifndef ADAFRUIT_SEESAWPERIPHERAL_STM32_REGISTERS_H
#define ADAFRUIT_SEESAWPERIPHERAL_STM32_REGISTERS_H

namespace SeesawSTM32 {
/** Standard seesaw module addresses. */
enum {
  SEESAW_STATUS_BASE = 0x00,
  SEESAW_GPIO_BASE = 0x01,
  SEESAW_SERCOM0_BASE = 0x02,
  SEESAW_TIMER_BASE = 0x08,
  SEESAW_ADC_BASE = 0x09,
  SEESAW_EEPROM_BASE = 0x0D,
  SEESAW_NEOPIXEL_BASE = 0x0E,
  SEESAW_ENCODER_BASE = 0x11,
};
/** GPIO register addresses. */
enum {
  SEESAW_GPIO_DIRSET_BULK = 0x02,
  SEESAW_GPIO_DIRCLR_BULK = 0x03,
  SEESAW_GPIO_BULK = 0x04,
  SEESAW_GPIO_BULK_SET = 0x05,
  SEESAW_GPIO_BULK_CLR = 0x06,
  SEESAW_GPIO_BULK_TOGGLE = 0x07,
  SEESAW_GPIO_INTENSET = 0x08,
  SEESAW_GPIO_INTENCLR = 0x09,
  SEESAW_GPIO_INTFLAG = 0x0A,
  SEESAW_GPIO_PULLENSET = 0x0B,
  SEESAW_GPIO_PULLENCLR = 0x0C,
};
/** Device status register addresses. */
enum {
  SEESAW_STATUS_HW_ID = 0x01,
  SEESAW_STATUS_VERSION = 0x02,
  SEESAW_STATUS_OPTIONS = 0x03,
  SEESAW_STATUS_SWRST = 0x7F,
};
/** PWM register addresses. */
enum { SEESAW_TIMER_PWM = 0x01, SEESAW_TIMER_FREQ = 0x02 };
/** ADC register addresses. */
enum { SEESAW_ADC_STATUS = 0x00, SEESAW_ADC_CHANNEL_OFFSET = 0x07 };
/** UART register addresses. */
enum {
  SEESAW_SERCOM_STATUS = 0x00,
  SEESAW_SERCOM_INTEN = 0x02,
  SEESAW_SERCOM_INTENCLR = 0x03,
  SEESAW_SERCOM_BAUD = 0x04,
  SEESAW_SERCOM_DATA = 0x05,
};
/** NeoPixel register addresses. */
enum {
  SEESAW_NEOPIXEL_PIN = 0x01,
  SEESAW_NEOPIXEL_SPEED = 0x02,
  SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
  SEESAW_NEOPIXEL_BUF = 0x04,
  SEESAW_NEOPIXEL_SHOW = 0x05,
};
/** Encoder register addresses. */
enum {
  SEESAW_ENCODER_INTENSET = 0x10,
  SEESAW_ENCODER_INTENCLR = 0x20,
  SEESAW_ENCODER_POSITION = 0x30,
  SEESAW_ENCODER_DELTA = 0x40,
};
} // namespace SeesawSTM32
#endif
