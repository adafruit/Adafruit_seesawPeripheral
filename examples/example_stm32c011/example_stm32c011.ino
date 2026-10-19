// Adafruit STM32C011 seesaw core: I2C, GPIO, and 12-bit ADC.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_EEPROM 0

// Optional active-low GPIO change interrupt on D15 / PC14:
// #define CONFIG_INTERRUPT_PIN 15
// I2C uses PB7/SDA and PB6/SCL. SWD and the PC15 LED are reserved.
// Protocol D0..8 = PA0..8, D9/10 = PA11/12, and D15 = PC14.
// Only D0..10 support ADC; configured address/interrupt pins are excluded.

#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    // No serial monitor is required. Do not run after failed initialization.
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
