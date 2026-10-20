// Adafruit seesaw with EEPROM enabled on AVR and STM32C011.
// AVR uses its hardware EEPROM. On STM32, reserve the final 2 KiB flash page:
// the application must fit in 30 KiB, and emulation uses a 2 KiB RAM buffer.
// Do not use flash-backed EEPROM for frequently updated data.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_EEPROM 1

#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    // On STM32, initialization checks firmware size and the emulation buffer.
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
