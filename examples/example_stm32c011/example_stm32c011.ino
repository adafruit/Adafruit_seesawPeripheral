// Adafruit STM32C011 seesaw: GPIO, ADC, PWM, NeoPixel, and encoder.
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_NEOPIXEL 1
#define CONFIG_EEPROM 0
#define CONFIG_ENCODER 1
#ifndef CONFIG_NUM_ENCODERS
#define CONFIG_NUM_ENCODERS 1
#endif
#define CONFIG_ENCODER0_A_PIN 2
#define CONFIG_ENCODER0_B_PIN 3

// Optional active-low GPIO change interrupt on D15 / PC14:
// #define CONFIG_INTERRUPT_PIN 15
// I2C uses PB7/SDA and PB6/SCL. SWD and the PC15 LED are reserved.
// Protocol D0..8 = PA0..8, D9/10 = PA11/12, and D15 = PC14.
// Only D0..10 support ADC; configured address/interrupt pins are excluded.
// PWM: D0..9 and D15, with 16-bit duty commands and a shared frequency.
// D3/D9 and D7/D15 share timer channels: use one pin from each pair at a time.
// NeoPixel: 800 kHz raw bytes, with a default 192-byte buffer.
// Encoder 0 uses D2/D3 (PA2/PA3), with internal pull-ups.
// For more encoders, set CONFIG_NUM_ENCODERS and each additional A/B pair.

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
