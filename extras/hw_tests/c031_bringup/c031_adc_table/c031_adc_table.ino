// Adafruit STM32C031 seesaw ADC table, running on a Metro 328 / Uno.
// Upload this sketch to the Metro, not to the C031.
// Requires Adafruit_seesaw with STM32 support and C031 bring-up firmware.
// Connect the Metro through the breakout's level-shifted I2C bus, with GND.
// ADC inputs must stay between GND and the C031 board's 3.3V supply. Never 5V!
// Unconnected analog inputs float and will show changing values.
// Open Serial Monitor at 115200 baud. Readings are 10-bit values (0..1023).

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t adcPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 16, 17};
const uint8_t adcCount = sizeof(adcPins) / sizeof(adcPins[0]);

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw ADC table\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("seesaw not found! Check power, wiring, firmware, and library.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("seesaw found at 0x49. ADC range: 0-1023.\n\r"));
  Serial.print(F("Use only 0-3.3V on the ADC inputs.\n\r\n\r"));
  Serial.print(F("PA0\tPA1\tPA2\tPA3\tPA4\tPA5\tPA6\tPA7\tPA8\tPA11\tPA12\tPB0\tPB1\n\r"));
}

void loop() {
  for (uint8_t index = 0; index < adcCount; index++) {
    uint16_t reading = ss.analogRead(adcPins[index]);
    if (reading > 1023) {
      Serial.print(F("ERR"));
    } else {
      Serial.print(reading);
    }
    if (index + 1 < adcCount) {
      Serial.print('\t');
    }
  }
  Serial.print(F("\n\r"));
  delay(500);
}
