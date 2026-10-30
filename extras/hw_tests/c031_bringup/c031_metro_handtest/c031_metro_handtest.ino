// Adafruit STM32C031 seesaw GPIO hand test on a Metro 328 / Uno.
// Upload to the Metro, not the C031. Open Serial Monitor at 115200 baud.
// Requires Adafruit_seesaw with STM32 support and the unrestricted C031
// GPIO/ADC/PWM hand-test firmware at address 0x49, not the default image.
// Connect through the level-shifted STEMMA QT bus, with common GND and power.
// Never put Metro 5V on a C031 GPIO. Use a 4.7K resistor to GND or the C031
// board's 3V for header input tests; an unbiased input may float.
// PC15 is the onboard LED: use 0/1, not an input jumper, to test it.
// Remove external jumpers before enabling an output or selecting another pin.
// Commands 0/1 should produce 0/3.3V; check the selected pad with a meter.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t testPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 12, 15, 16, 17, 19, 20, 21, 24, 22, 30, 31};
const char *pinLabels[] = {"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA11", "PA12", "PA15", "PB0", "PB1", "PB3", "PB4", "PB5", "PB8", "PC6", "PC14", "PC15 LED"};
const uint8_t pinCount = sizeof(testPins) / sizeof(testPins[0]);
uint8_t selected = 0;
char readingMode = 0;
uint32_t lastReading = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw GPIO hand test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("Begin failed. Check power, wiring, firmware, and library version.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("Begin succeeded at 0x49. Firmware version: 0x"));
  Serial.print(ss.getVersion(), HEX);
  Serial.print(F("\n\r"));

  releaseOutputs();
  printHelp();
  printSelectedPin();
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    uint8_t pin = testPins[selected];
    if (command == 'n' || command == 'p') {
      readingMode = 0;
      ss.pinMode(pin, INPUT); // Release the previous output before selecting.
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
    } else if (command == '0' || command == '1') {
      readingMode = 0;
      // Set the output latch before enabling the output driver.
      ss.digitalWrite(pin, command == '1');
      ss.pinMode(pin, OUTPUT);
      Serial.print(F("Output set to "));
      Serial.print(command);
      Serial.print(F(". Check the voltage on the selected pad.\n\r"));
    } else if (command == 'u') {
      ss.pinMode(pin, INPUT_PULLUP);
      readingMode = 'r';
      Serial.print(F("Pull-up input: expect HIGH; resistor to GND -> LOW.\n\r"));
    } else if (command == 'd') {
      ss.pinMode(pin, INPUT_PULLDOWN);
      readingMode = 'r';
      Serial.print(F("Pull-down input: expect LOW; resistor to 3V -> HIGH.\n\r"));
    } else if (command == 'r') {
      ss.pinMode(pin, INPUT);
      readingMode = 'r';
      Serial.print(F("Reading input; the previous pull setting is retained.\n\r"));
    } else if (command == 'x') {
      releaseOutputs();
    } else if (command == '?') {
      printHelp();
      printSelectedPin();
    }
  }

  if (readingMode && millis() - lastReading >= 500) {
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

void releaseOutputs() {
  readingMode = 0;
  for (uint8_t index = 0; index < pinCount; index++) {
    ss.pinMode(testPins[index], INPUT);
  }
  // The current host library's INPUT mode does not clear an existing pull.
  Serial.print(F("All outputs released; previous pull settings retained.\n\r"));
}

void printSelectedPin() {
  Serial.print(F("\n\rSelected "));
  Serial.print(pinLabels[selected]);
  Serial.print(F(" (seesaw pin "));
  Serial.print(testPins[selected]);
  Serial.print(F("). GPIO\n\r"));
}

void printHelp() {
  Serial.print(F("\n\rn / p : next / previous pin (releases the old output)\n\r"));
  Serial.print(F("0 / 1 : output LOW / HIGH\n\r"));
  Serial.print(F("u / d : input pull-up / pull-down, with continuous readings\n\r"));
  Serial.print(F("r     : read input continuously (retains previous pull)\n\r"));
  Serial.print(F("x     : stop readings and release all outputs\n\r"));
  Serial.print(F("?     : show this help\n\r"));
  Serial.print(F("Use only GND or the C031 board's 3V for input tests.\n\r"));
}
