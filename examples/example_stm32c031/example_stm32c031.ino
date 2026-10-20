// Adafruit STM32C031G6U6 seesaw breakout: GPIO, ADC, PWM, NeoPixel, encoder.
// Select STM32duino's generic STM32C031G6Ux target.
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

// I2C is PB7/SDA and PB6/SCL. SWD, PC15 LED, and reset stay reserved.
// D0..8 = PA0..8; D9/10 = PA11/12; D13 = PA15; D14/15 = PB0/1;
// D16..18 = PB3..5; D21 = PB8; D22 = PC6; D23 = PC14.
// ADC: D0..10, D14/15. PWM: all usable GPIOs except D10.
// Some PWM pins share timer channels; see the README before combining them.
// CONFIG_EEPROM and CONFIG_SPI are optional; check flash usage when combining
// features on this 32 KiB target. The final 2 KiB page is reserved for EEPROM.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
