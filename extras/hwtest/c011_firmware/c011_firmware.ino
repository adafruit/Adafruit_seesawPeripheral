// Adafruit STM32C011 seesaw engineering firmware.
// Protocol GPIO indices 0..8 = PA0..PA8, 9 = PA11, 10 = PA12, 15 = PC14.
// SWD (indices 11/12), I2C (13/14), and the PC15 LED (16) stay reserved.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_I2C_SDA_PIN PB7
#define CONFIG_I2C_SCL_PIN PB6
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_EEPROM 1
#define CONFIG_NEOPIXEL 1
#define CONFIG_NEOPIXEL_BUF_MAX 192
#ifndef HWTEST_SPI
#define HWTEST_SPI                                                             \
  0 // Experimental controller bridge, enabled in the spi target.
#endif
#define CONFIG_SPI HWTEST_SPI
#ifdef HWTEST_SPI_QUEUE_HOLD
#define CONFIG_SPI_TEST_HOLD_US 10000
#endif
#ifndef HWTEST_UART
#define HWTEST_UART 0 // Specialty bridge; enable explicitly when testing UART.
#endif
#ifndef HWTEST_ENCODER
#define HWTEST_ENCODER 1
#endif
#ifndef HWTEST_IRQ
#define HWTEST_IRQ 1
#endif
#define CONFIG_UART HWTEST_UART
#define CONFIG_ENCODER HWTEST_ENCODER
#ifndef HWTEST_ENCODERS
#define HWTEST_ENCODERS 1
#endif
#define CONFIG_NUM_ENCODERS HWTEST_ENCODERS
#define CONFIG_ENCODER0_A_PIN 2
#define CONFIG_ENCODER0_B_PIN 3
#define CONFIG_ENCODER1_A_PIN 4
#define CONFIG_ENCODER1_B_PIN 5
#define CONFIG_ENCODER2_A_PIN 6
#define CONFIG_ENCODER2_B_PIN 7
#define CONFIG_ENCODER3_A_PIN 9
#define CONFIG_ENCODER3_B_PIN 10
// PC14 is dedicated to active-low interrupt output in this test build.
#if HWTEST_IRQ
#define CONFIG_INTERRUPT_PIN 15
#endif

#include "Adafruit_seesawPeripheral.h"

void setup() { Adafruit_seesawPeripheral_begin(); }

void loop() { Adafruit_seesawPeripheral_run(); }
