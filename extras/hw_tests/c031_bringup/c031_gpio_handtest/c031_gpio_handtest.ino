// C031G6 PID 6540: dedicated GPIO/ADC/PWM test image, no encoders or IRQ.
// Uses the C031 label-based pin map; D31 is the PC15 LED. Flash the default image
// after basic pin tests to enable NeoPixel, EEPROM and encoder support.
// Adafruit STM32C031G6 GPIO hand-test peripheral, I2C address 0x49.
// All twenty header GPIOs and the LED remain available. No straps or IRQ pin.
// I2C bootloader entry stays enabled for later firmware updates.
#define PRODUCT_CODE 6540
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_BOOTLOADER 1
#define CONFIG_EEPROM 0
#define CONFIG_ENCODER 0
#define CONFIG_NEOPIXEL 0
#define CONFIG_UART 0

#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() {
  Adafruit_seesawPeripheral_run();
}
