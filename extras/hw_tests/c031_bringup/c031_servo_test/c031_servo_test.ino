// Adafruit STM32C031 seesaw 16-bit servo PWM test on a Metro 328 / Uno.
// Upload this sketch to the Metro; keep the current C031 firmware.
// Requires Adafruit_seesaw with STM32 support and C031 bring-up firmware.
// Connect through the breakout's level-shifted I2C bus, with a shared GND.
// Probe C031 PA0 and GND with a scope before connecting a servo.
// Expected: a 20 ms period (50 Hz), with 1000-2000 us high pulses.
// Remove the input-test jumper from PA0: it will become an output!
// For a servo, connect its signal wire to PA0, use a suitable external servo
// supply, and join the grounds. Do not power the servo from C031 3.3V.
// Start with the servo unloaded; stop if it hits a mechanical limit or stalls.
// These pulse widths are test values, not guaranteed servo travel endpoints.
// Open Serial Monitor at 115200 baud. Printed widths are requested, not measured.

#include <Adafruit_seesaw.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif

Adafruit_seesaw ss;
const uint8_t servoPin = 0; // C031 PA0, NOT the Metro's pin 0
const uint16_t servoFrequency = 50; // Hz; shared by all C031 PWM outputs
const uint32_t periodUs = 1000000UL / servoFrequency;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw 16-bit servo PWM test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("seesaw not found! Check power, wiring, firmware, and library.\n\r"));
    while (true) {
      delay(10);
    }
  }
  Serial.print(F("seesaw found at 0x49.\n\r"));
  Serial.print(F("Remove the PA0 input-test jumper before running this test.\n\r"));
  Serial.print(F("Scope PA0: expect 50 Hz / 20 ms period and 3.3V pulses.\n\r\n\r"));
  delay(2000);

  ss.pinMode(servoPin, OUTPUT);
  ss.setPWMFreq(servoPin, servoFrequency);
  writePulse(1500);
  Serial.print(F("Starting at 1500 us (approximately center).\n\r"));
  delay(3000);
}

void loop() {
  Serial.print(F("Hold 1000 us for 3 seconds\n\r"));
  writePulse(1000);
  delay(3000);

  Serial.print(F("Hold 1500 us for 3 seconds\n\r"));
  writePulse(1500);
  delay(3000);

  Serial.print(F("Hold 2000 us for 3 seconds\n\r"));
  writePulse(2000);
  delay(3000);

  // Five-microsecond steps are much finer than 8-bit PWM at 50 Hz (~78 us).
  Serial.print(F("Sweep 2000 -> 1000 us in 5 us steps\n\r"));
  for (int pulseUs = 2000; pulseUs >= 1000; pulseUs -= 5) {
    writePulse(pulseUs);
    delay(20);
  }

  Serial.print(F("Sweep 1000 -> 2000 us in 5 us steps\n\r"));
  for (int pulseUs = 1000; pulseUs <= 2000; pulseUs += 5) {
    writePulse(pulseUs);
    delay(20);
  }

  Serial.print(F("Hold 1500 us, then repeat\n\r\n\r"));
  writePulse(1500);
  delay(3000);
}

void writePulse(uint16_t pulseUs) {
  // Convert pulse width to a 16-bit fraction of the 20,000 us period.
  // Use 32-bit arithmetic on the Metro, and round to the nearest duty value.
  uint16_t duty = ((uint32_t)pulseUs * 65535UL + periodUs / 2) / periodUs;
  ss.analogWrite(servoPin, duty, 16);
}
