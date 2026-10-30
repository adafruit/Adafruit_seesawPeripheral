// Adafruit STM32C031 seesaw input hand test on a Metro 328 / Uno.
// Upload to the Metro, not the C031. Open Serial Monitor at 115200 baud.
// Requires Adafruit_seesaw with STM32 support and the unrestricted C031
// GPIO/ADC/PWM hand-test firmware at address 0x49, not the default image.
// Connect through the level-shifted STEMMA QT bus, with common GND and power.
// Never put Metro 5V on a C031 GPIO. Use a 4.7K resistor to GND or the C031
// board's 3V for header input tests; an unbiased input may float.
// PC15 is the onboard LED, not a header input: do not jumper it.
// n / p selects the next / previous pin. This step never enables outputs.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t testPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 15, 16, 17, 19, 20, 21, 24, 22, 30, 31};
const char *pinLabels[] = {"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA11", "PA12", "PA15", "PB0", "PB1", "PB3", "PB4", "PB5", "PB8", "PC6", "PC14", "PC15 LED"};
const uint8_t pinCount = sizeof(testPins) / sizeof(testPins[0]);
uint8_t selected = 0;
uint32_t lastReading = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw input hand test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("Begin failed. Check power, wiring, firmware, and library version.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("Begin succeeded at 0x49. Firmware version: 0x"));
  Serial.print(ss.getVersion(), HEX);
  Serial.print(F("\n\r"));

  for (uint8_t index = 0; index < pinCount; index++) {
    ss.pinMode(testPins[index], INPUT);
  }
  // The current host library's INPUT mode does not clear an existing pull.
  Serial.print(F("All outputs released; previous pull settings retained.\n\r"));
  Serial.print(F("n / p: next / previous pin. Digital readings every 500 ms.\n\r"));
  printSelectedPin();
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'n' || command == 'p') {
      ss.pinMode(testPins[selected], INPUT);
      if (command == 'n') {
        selected++;
        if (selected == pinCount) {
          selected = 0;
        }
      } else if (selected == 0) {
        selected = pinCount - 1;
      } else {
        selected--;
      }
      printSelectedPin();
    }
  }

  if (millis() - lastReading >= 500) {
    lastReading = millis();
    Serial.print(pinLabels[selected]);
    Serial.print(F(": "));
    if (ss.digitalRead(testPins[selected])) {
      Serial.print(F("HIGH\n\r"));
    } else {
      Serial.print(F("LOW\n\r"));
    }
  }
}

void printSelectedPin() {
  Serial.print(F("\n\rSelected "));
  Serial.print(pinLabels[selected]);
  Serial.print(F(" (seesaw pin "));
  Serial.print(testPins[selected]);
  Serial.print(F("). GPIO\n\r"));
}
