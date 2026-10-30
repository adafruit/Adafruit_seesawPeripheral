// Adafruit C031 PID 6540 encoder test. Upload to the Metro/Uno.
// Run the C031 default firmware. Encoder common -> GND, A -> PA2, B -> PA3.
// Remove other PA2/PA3 loads. Internal pull-ups are enabled by the peripheral.
#include <Adafruit_seesaw.h>

Adafruit_seesaw ss;
int32_t previousPosition = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 encoder test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("Begin failed; check wiring and default firmware.\n\r"));
    while (true) delay(10);
  }
  ss.setEncoderPosition(0);
  Serial.print(F("Begin succeeded. Rotate both ways; position should reverse direction.\n\r"));
}

void loop() {
  int32_t position = ss.getEncoderPosition();
  if (position != previousPosition) {
    Serial.print(F("Position: "));
    Serial.print(position);
    Serial.print(F("\n\r"));
    previousPosition = position;
  }
  delay(20);
}
