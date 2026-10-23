// Adafruit STM32C011 I2C-to-SPI controller for displays and other SPI devices.
// The SPI protocol is documented in the library README.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_SPI 1
#ifndef CONFIG_SPI_CS_PIN
#define CONFIG_SPI_CS_PIN 4 // PA4; any available GPIO may provide CS.
#endif
#ifndef CONFIG_SPI_SCK_PIN
#define CONFIG_SPI_SCK_PIN 5 // PA5
#endif
#ifndef CONFIG_SPI_MISO_PIN
#define CONFIG_SPI_MISO_PIN 6 // PA6
#endif
#ifndef CONFIG_SPI_MOSI_PIN
#define CONFIG_SPI_MOSI_PIN 7 // PA7
#endif
#define CONFIG_ADC 1
#define CONFIG_PWM 1

// Configure all four pins above using seesaw protocol GPIO numbers.
// SCK, MISO, and MOSI must map to the same hardware SPI peripheral.
// All SPI signals use 3.3 V logic.
// Other GPIOs may provide a display's D/C, reset, or BUSY signals.
// The selected pins return to GPIO/ADC/PWM when the host disables SPI.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
