// Adafruit STM32C011 I2C-to-SPI controller for displays and other SPI devices.
// The SPI protocol is documented in the library README.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_SPI 1
#define CONFIG_ADC 1
#define CONFIG_PWM 1

// SPI uses PA4/CS, PA5/SCK, PA6/MISO, and PA7/MOSI at 3.3 V.
// Other GPIOs may provide a display's D/C, reset, or BUSY signals.
// Pins 4..7 return to GPIO/ADC/PWM when the host disables SPI.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
