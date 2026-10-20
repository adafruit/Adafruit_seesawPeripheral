// Adafruit STM32C011 seesaw with opt-in flash-backed EEPROM.
// Reserve the final 2 KiB flash page: the application must fit in 30 KiB.
// Emulation uses a 2 KiB RAM buffer. Do not use it for frequently updated data.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_EEPROM 1

#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    // Initialization refuses overlapping firmware or an unavailable RAM buffer.
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
