// Adafruit STM32C031 seesaw PWM sweep, running on a Metro 328 / Uno.
// Upload this sketch to the Metro, not to the C031.
// Requires Adafruit_seesaw with STM32 support and C031 bring-up firmware.
// Connect through the breakout's level-shifted I2C bus, with a shared GND.
// Disconnect the ADC test jumpers before running: these pins become outputs!
// Observe the 3.3V PWM with a scope, or an LED with a series resistor.
// Open Serial Monitor at 115200 baud to see which pin is being swept.
// Only one pin runs at a time because these pairs share timer channels:
// PA3/PA11; PA5/PA15; PA6/PB4/PB8/PC6; PA7/PB5/PC14;
// PA8/PB0/PC15. PC15 is the onboard LED.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t pwmPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24};
const char *pinLabels[] = {"PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA11", "PA15", "PB0", "PB1", "PB3", "PB4", "PB5", "PB8", "PC6", "PC14", "PC15 LED"};
const uint8_t pwmCount = sizeof(pwmPins) / sizeof(pwmPins[0]);
const uint16_t pwmFrequency = 1000; // Hz
const uint16_t stepDelay = 10;      // ms; about 5 seconds per up/down sweep

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw PWM sweep\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("seesaw not found! Check power, wiring, firmware, and library.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("seesaw found at 0x49. PWM frequency: 1000 Hz.\n\r"));
  Serial.print(F("Remove ADC jumpers to GND or 3.3V before testing outputs!\n\r"));
  Serial.print(F("Each pin sweeps 0-100-0%, then the next pin starts.\n\r\n\r"));

  for (uint8_t index = 0; index < pwmCount; index++) {
    ss.pinMode(pwmPins[index], INPUT);
  }
  delay(2000);
}

void loop() {
  for (uint8_t index = 0; index < pwmCount; index++) {
    uint8_t pin = pwmPins[index];
    Serial.print(pinLabels[index]);
    Serial.print(F(" (seesaw pin "));
    Serial.print(pin);
    Serial.print(F("): sweeping up\n\r"));

    ss.pinMode(pin, OUTPUT);
    ss.setPWMFreq(pin, pwmFrequency);
    // analogWrite uses an 8-bit duty cycle: 0 is off, 255 is fully on.
    for (int duty = 0; duty <= 255; duty++) {
      ss.analogWrite(pin, duty);
      delay(stepDelay);
    }

    Serial.print(F("  Sweeping down\n\r"));
    for (int duty = 254; duty >= 0; duty--) {
      ss.analogWrite(pin, duty);
      delay(stepDelay);
    }

    // Disconnect this pin from PWM before enabling a shared timer channel.
    ss.pinMode(pin, INPUT);
    Serial.print(F("  Released\n\r\n\r"));
    delay(500);
  }
}
