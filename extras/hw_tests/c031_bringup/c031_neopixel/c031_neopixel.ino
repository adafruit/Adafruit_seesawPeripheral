// Adafruit C031 PID 6540 NeoPixel test. Upload to the Metro/Uno.
// Run the C031 default firmware. One pixel DIN -> PA4, with common GND.
// Use suitable pixel power and level shifting if its DIN needs a 5 V signal.
// Never connect 5 V to a C031 GPIO. Remove other PA4 test leads.
#include <seesaw_neopixel.h>

seesaw_NeoPixel pixel(1, 4, NEO_GRB + NEO_KHZ800);
uint8_t color = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 NeoPixel test\n\r"));

  if (!pixel.begin(0x49)) {
    Serial.print(F("Begin failed; check wiring and default firmware.\n\r"));
    while (true) delay(10);
  }
  pixel.setBrightness(20);
  pixel.setPixelColor(0, 0);
  pixel.show();
  Serial.print(F("Begin succeeded. Expect red, green, blue, off in order.\n\r"));
}

void loop() {
  switch (color) {
    case 0:
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
      Serial.print(F("Red\n\r"));
      break;
    case 1:
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
      Serial.print(F("Green\n\r"));
      break;
    case 2:
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));
      Serial.print(F("Blue\n\r"));
      break;
    default:
      pixel.setPixelColor(0, 0);
      Serial.print(F("Off\n\r"));
      break;
  }
  pixel.show();
  color++;
  if (color == 4) color = 0;
  delay(1000);
}
