// Adafruit STM32C011 seesaw hand test, running on a Metro 328 / Uno.
// Use the Adafruit seesaw Library with STM32 support:
// https://github.com/adafruit/Adafruit_Seesaw
// Upload this to the METRO, not to the C011. Open Serial Monitor at 115200.
// The C011 must run the GPIO/ADC/PWM hand-test firmware at address 0x49.
// Connect Metro SDA/SCL through the breakout's level-shifted STEMMA QT bus,
// with common ground and board power. Never put Metro 5V on a C011 GPIO.
//
// Test each pin with a meter or scope. Commands 0/1 should produce 0/3.3V.
// For inputs, use a 4.7K resistor to connect the selected pin to GND or 3V:
// u = pull-up (normally HIGH); ground through the resistor -> LOW.
// d = pull-down (normally LOW); connect to 3V through the resistor -> HIGH.
// a = ADC: GND -> about 0, 3V -> about 1023; a pot tests intermediate values.
// w = PWM: check approximately 1kHz, 50% duty with a scope.
// A meter showing about 1.65V does not by itself prove the PWM waveform.
// Remove external jumpers before enabling an output or selecting another pin.
//
// PA11 is seesaw pin 9; PA12 is 10; PC14 is 15 (not the port pin number).
// PA12 has ADC but no PWM; PC14 has PWM but no ADC.
// PA3/PA11 and PA7/PC14 share timer channels, so only one output runs at once.
// This is a manual electrical test, not an automatic PASS/FAIL fixture.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C011
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;

const uint8_t testPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15};
const char *pinLabels[] = {"PA0", "PA1", "PA2", "PA3", "PA4", "PA5",
                           "PA6", "PA7", "PA8", "PA11", "PA12", "PC14"};
const uint8_t pinCount = sizeof(testPins) / sizeof(testPins[0]);
uint8_t selected = 0;
char readingMode = 0;
uint32_t lastReading = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("\n\rAdafruit STM32C011 seesaw GPIO / ADC / PWM hand test\n\r"));

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
    switch (command) {
      case 'n':
      case 'p':
        readingMode = 0;
        ss.pinMode(pin, INPUT); // Stop PWM and release the previous output.
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
        break;

      case '0':
      case '1':
        readingMode = 0;
        // Set the output latch before enabling the output driver.
        ss.digitalWrite(pin, command == '1');
        ss.pinMode(pin, OUTPUT);
        Serial.print(F("Output set to "));
        Serial.print(command);
        Serial.print(F(". Check the voltage on the selected pad.\n\r"));
        break;

      case 'u':
        ss.pinMode(pin, INPUT_PULLUP);
        readingMode = 'r';
        Serial.print(F("Pull-up input: expect HIGH; resistor to GND -> LOW.\n\r"));
        break;

      case 'd':
        ss.pinMode(pin, INPUT_PULLDOWN);
        readingMode = 'r';
        Serial.print(F("Pull-down input: expect LOW; resistor to 3V -> HIGH.\n\r"));
        break;

      case 'r':
        ss.pinMode(pin, INPUT);
        readingMode = 'r';
        Serial.print(F("Reading input; the previous pull setting is retained.\n\r"));
        break;

      case 'a':
        if (pin == 15) {
          Serial.print(F("PC14 has no ADC. Select another pin.\n\r"));
          break;
        }
        ss.pinMode(pin, INPUT);
        readingMode = 'a';
        Serial.print(F("Reading ADC (0-1023); voltage assumes a 3.3V supply.\n\r"));
        break;

      case 'w':
        if (pin == 10) {
          Serial.print(F("PA12 has no PWM. Select another pin.\n\r"));
          break;
        }
        readingMode = 0;
        ss.pinMode(pin, OUTPUT);
        ss.setPWMFreq(pin, 1000);
        ss.analogWrite(pin, 128); // 8-bit duty: 0..255, approximately 50%.
        Serial.print(F("PWM set to 1kHz, about 50% duty. Check with a scope.\n\r"));
        break;

      case 'x':
        releaseOutputs();
        break;

      case '?':
        printHelp();
        printSelectedPin();
        break;

      case '\r':
      case '\n':
      case ' ':
        break;

      default:
        Serial.print(F("Unknown command. Send ? for help.\n\r"));
        break;
    }
  }

  if (readingMode && millis() - lastReading >= 500) {
    lastReading = millis();
    Serial.print(pinLabels[selected]);
    Serial.print(F(": "));
    if (readingMode == 'a') {
      uint16_t raw = ss.analogRead(testPins[selected]);
      if (raw > 1023) {
        Serial.print(F("ADC read failed\n\r"));
      } else {
        float volts = map(raw, 0, 1023, 0, 3300) / 1000.0;
        Serial.print(raw);
        Serial.print(F(" / 1023, about "));
        Serial.print(volts, 3);
        Serial.print(F(" V\n\r"));
      }
    } else {
      if (ss.digitalRead(testPins[selected])) {
        Serial.print(F("HIGH\n\r"));
      } else {
        Serial.print(F("LOW\n\r"));
      }
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
  Serial.print(F("). GPIO"));
  if (testPins[selected] != 15) {
    Serial.print(F(" + ADC"));
  }
  if (testPins[selected] != 10) {
    Serial.print(F(" + PWM"));
  }
  Serial.print(F("\n\r"));
}

void printHelp() {
  Serial.print(F("\n\rn / p : next / previous pin (releases the old output)\n\r"));
  Serial.print(F("0 / 1 : output LOW / HIGH\n\r"));
  Serial.print(F("u / d : input pull-up / pull-down, with continuous readings\n\r"));
  Serial.print(F("r     : read input continuously (retains previous pull)\n\r"));
  Serial.print(F("a     : read ADC continuously\n\r"));
  Serial.print(F("w     : output 1kHz PWM at about 50% duty\n\r"));
  Serial.print(F("x     : stop readings and release all outputs\n\r"));
  Serial.print(F("?     : show this help\n\r"));
  Serial.print(F("Use only GND or the C011 board's 3V for input tests.\n\r"));
}
