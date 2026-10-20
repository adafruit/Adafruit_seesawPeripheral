// Adafruit STM32C011/C031/C051/G071 analog microphone spectrum peripheral.
// Same spectrum configuration as example_fht, without address straps.
#define PRODUCT_CODE 1234
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_FHT 1
#define CONFIG_FHT_PIN 0 // PA0. Supply the microphone from 3.3 V.

// SPW2430: VIN to 3.3 V, GND to ground, DC to PA0; leave AC unconnected.
// The ADC input must stay between ground and 3.3 V.
// Default: 128 samples at 12,500 Hz; 64 logarithmic magnitude bins.
// Read SPECTRUM_RESULTS_LOWER, then UPPER to get one coherent frame.
// This mode owns ADC1, DMA1 channel 1, and TIM3.
// Other features can coexist on unreserved pins/timers if memory permits.
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
