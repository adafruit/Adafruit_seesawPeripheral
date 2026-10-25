// Adafruit STM32C031 seesaw input/pull-up test on a Metro 328 / Uno.
// Upload this sketch to the Metro, not to the C031.
// Requires Adafruit_seesaw with STM32 support and C031 bring-up firmware.
// Connect through the breakout's level-shifted I2C bus, with a shared GND.
// Open Serial Monitor at 115200 baud and wait for READY before grounding pins.
// Touch each listed C031 header pin to GND to simulate a button press.
// A 1K series resistor in the ground jumper adds protection against mistakes.
// Never apply 5V. Remove external LEDs or other loads from the tested pins.
// Run the unrestricted hand-test firmware: INT/ADDR pins are tested as GPIO.
// PC15 (pin 31) is the onboard LED, not a header input, so it is omitted.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t inputPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 15, 16, 17, 19, 20, 21, 24, 22, 30};
const uint8_t inputCount = sizeof(inputPins) / sizeof(inputPins[0]);
uint8_t tableRow = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw input/pull-up test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("seesaw not found! Check power, wiring, firmware, and library.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("seesaw found at 0x49.\n\r"));

  for (uint8_t index = 0; index < inputCount; index++) {
    ss.pinMode(inputPins[index], INPUT_PULLUP);
  }
  delay(20);

  Serial.print(F("READY: all 20 header pins are inputs with pull-ups.\n\r"));
  Serial.print(F("Ground any listed pin: PRESSED = LOW, - = released/HIGH.\n\r"));
  Serial.print(F("Hold each contact briefly; the table updates every 250 ms.\n\r\n\r"));
}

void loop() {
  // Repeat the column labels so they remain visible as the table scrolls.
  if (tableRow == 0) {
    Serial.print(F("PA0\tPA1\tPA2\tPA3\tPA4\tPA5\tPA6\tPA7\tPA8\tPA11\tPA12\tPA15\tPB0\tPB1\tPB3\tPB4\tPB5\tPB8\tPC6\tPC14\n\r"));
  }

  for (uint8_t index = 0; index < inputCount; index++) {
    if (ss.digitalRead(inputPins[index]) == LOW) {
      Serial.print(F("PRESSED"));
    } else {
      Serial.print('-');
    }
    if (index + 1 < inputCount) {
      Serial.print('\t');
    }
  }
  Serial.print(F("\n\r"));

  tableRow++;
  if (tableRow == 20) {
    tableRow = 0;
  }
  delay(250);
}
