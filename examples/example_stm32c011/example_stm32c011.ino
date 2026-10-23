// Adafruit STM32C011 seesaw: GPIO, ADC, PWM, NeoPixel, and encoder.
// This feature configuration also runs on the supported tinyAVR targets.
// Pin descriptions below are for STM32; use your AVR board's Arduino pin map.
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

// The AVR encoder backend requires an interrupt output; STM32 can omit it.
#if !defined(ARDUINO_GENERIC_C011F6UX)
#define CONFIG_INTERRUPT_PIN 15
#else
// Optional active-low GPIO/encoder interrupt on D14 / PC14:
// #define CONFIG_INTERRUPT_PIN 14
#endif
// I2C uses PB7/SDA and PB6/SCL. SWD remains reserved.
// Protocol D0..8 = PA0..8, D11/12 = PA11/12, D14 = PC14, D15 = PC15 LED.
// All other numbers are reserved, including D13 / SWDIO.
// Only D0..8 and D11/12 support ADC; address/interrupt pins are excluded.
// PWM: D0..8, D11, D14, D15, with 16-bit duty commands and shared frequency.
// D3/D11, D7/D14, D8/D15 share channels: use one pin from each pair at a time.
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
