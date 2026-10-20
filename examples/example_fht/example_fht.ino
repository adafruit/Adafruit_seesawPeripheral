#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x49

// Can have up to 3 addresses
#define CONFIG_ADDR_0_PIN          6
#define CONFIG_ADDR_1_PIN          7

#define CONFIG_FHT                 1

// STM32C011/C031/C051: connect a 0..3.3 V analog microphone output to PA0.
// For the Adafruit SPW2430 breakout, use DC, not AC; power it from 3.3 V.
// The measured DC offset is removed before each 128-point transform.
// CONFIG_FHT_PIN is a protocol GPIO index; the default is 0 (PA0).
// #define CONFIG_FHT_PIN 0
// Default rate code 12 = 12,500 samples/s, giving 97.65625 Hz per bin.
// STM32 owns ADC1, DMA1 channel 1, and TIM3. Do not enable ADC, PWM,
// NeoPixel, EEPROM, SPI, or encoders in the same sketch.

#include "Adafruit_seesawPeripheral.h"

void setup() {
#if CONFIG_UART_DEBUG
  Serial.begin(115200);
  delay(500);
#endif

  if (!Adafruit_seesawPeripheral_begin()) {
    // Do not run after a failed ADC or I2C initialization.
    while (true) {
      delay(10);
    }
  }
}


void loop() {
  Adafruit_seesawPeripheral_run();
}
