// Adafruit STM32C051G8U6 seesaw breakout with all supported bridge features.
// Select STM32duino's generic STM32C051G8Ux target; see README for menu setup.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_NEOPIXEL 1
#define CONFIG_ENCODER 1
#define CONFIG_NUM_ENCODERS 1
#define CONFIG_ENCODER0_A_PIN 2 // PA2
#define CONFIG_ENCODER0_B_PIN 3 // PA3
#define CONFIG_INTERRUPT_PIN 23 // PC14, active-low open-drain output.
#define CONFIG_UART 1 // PA0/TX and PA1/RX, 3.3 V logic.
#define CONFIG_UART_BUF_MAX 32
#define CONFIG_SPI 1
#define CONFIG_SPI_CS_PIN 4 // PA4, active-low CS, 3.3 V logic.
#define CONFIG_SPI_SCK_PIN 5 // PA5
#define CONFIG_SPI_MISO_PIN 6 // PA6
#define CONFIG_SPI_MOSI_PIN 7 // PA7
#define CONFIG_EEPROM 1

// Reserve the final 2 KiB flash page: application maximum 63488 bytes.
// EEPROM byte 255 holds the persistent I2C address; erased bytes are 0xFF.
// I2C is PB7/SDA and PB6/SCL. SWD, PC15 LED, and reset stay reserved.
// This target keeps the dense GPIO/ADC map documented in the README.
// UART pins are reserved at startup; SPI pins are reserved when configured.
// Choose another free GPIO for NeoPixels, display D/C, reset, or BUSY.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
