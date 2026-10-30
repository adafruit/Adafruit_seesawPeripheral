// Adafruit C031 PID 6540 EEPROM persistence test. Upload to Metro/Uno.
// Run the C031 default firmware. Only byte 0x02 is changed, then restored.
// Keep the Metro powered until restoration; its RAM holds the original byte.
// This test never changes the I2C address byte (0xFF).
#include <Adafruit_seesaw.h>

// Use checked reads so an I2C failure cannot masquerade as EEPROM value 0xFF.
class EEPROMSeesaw : public Adafruit_seesaw {
public:
  bool readChecked(uint8_t address, uint8_t *value);
};

EEPROMSeesaw ss;
const uint8_t testAddress = 0x02;
uint8_t originalValue;
uint8_t testValue;
bool wroteTest = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 EEPROM persistence test\n\r"));

  if (!ss.begin(0x49)) {
    Serial.print(F("Begin failed; check wiring and default firmware.\n\r"));
    while (true) delay(10);
  }
  if (!ss.readChecked(testAddress, &originalValue)) {
    Serial.print(F("Original byte could not be read; nothing written.\n\r"));
    while (true) delay(10);
  }
  testValue = 0x5A;
  if (originalValue == testValue) testValue = 0xA5;
  Serial.print(F("Begin succeeded. Original byte 0x02 = 0x"));
  Serial.print(originalValue, HEX);
  Serial.print(F("\n\rWrite down this value before testing.\n\r"));
  Serial.print(F("w: write once. Power-cycle ONLY C031, then v: verify, r: restore.\n\r"));
  Serial.print(F("Keep Metro powered; do not write repeatedly (flash wear).\n\r"));
}

void loop() {
  if (!Serial.available()) return;
  char command = Serial.read();
  uint8_t readback;
  if (command == 'w' && !wroteTest) {
    wroteTest = true; // Even an unconfirmed write must allow restoration.
    ss.EEPROMWrite8(testAddress, testValue);
    delay(100);
    if (!ss.readChecked(testAddress, &readback) || readback != testValue) {
      Serial.print(F("Write readback failed. Check communication; r restores the original.\n\r"));
      return;
    }
    Serial.print(F("Write readback passed. Power-cycle only C031, then send v.\n\r"));
  } else if (command == 'v' && wroteTest) {
    if (ss.readChecked(testAddress, &readback) && readback == testValue) {
      Serial.print(F("Read matches. If C031 was power-cycled, persistence passed. Send r.\n\r"));
    } else {
      Serial.print(F("Read mismatch; persistence not verified. Send r to restore.\n\r"));
    }
  } else if (command == 'r' && wroteTest) {
    ss.EEPROMWrite8(testAddress, originalValue);
    delay(100);
    if (ss.readChecked(testAddress, &readback) && readback == originalValue) {
      Serial.print(F("Original byte restored. Test finished; no further writes.\n\r"));
      while (true) delay(10);
    } else {
      Serial.print(F("Restore readback failed; retain original value and check connection.\n\r"));
    }
  }
}

bool EEPROMSeesaw::readChecked(uint8_t address, uint8_t *value) {
  return read(SEESAW_EEPROM_BASE, address, value, 1);
}
