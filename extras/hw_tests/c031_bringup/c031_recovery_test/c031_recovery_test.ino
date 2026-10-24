// Adafruit STM32C031 seesaw reset/power-cycle test on a Metro 328 / Uno.
// IMPORTANT: select "Arduino Uno" in the IDE for the Metro 328. Its Wire
// library has I2C timeouts; the older Adafruit Metro package does not.
// Upload only to the Metro. Keep the current PID 6540 C031 firmware.
// Uses Adafruit_seesaw for all device communication, with no automatic SWReset.
// Connect through the breakout's level-shifted I2C bus, with shared GND.
// Disconnect the servo and all GPIO test jumpers. Only PC15 (LED) is driven.
// Serial Monitor: 115200 baud. No Serial Monitor is required for startup.
// Try C031 reset/power cycling, Metro reset, and cold-starting both boards.
// For a true C031 power-off, avoid back-powering it through other connections.
// OFFLINE -> ONLINE and the visible LED blinking again demonstrate recovery.
// A very short reset can fall between polls; LED mode is restored every blink.

#include <Adafruit_seesaw.h>
#include <Wire.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif
#ifndef ARDUINO_AVR_UNO
#error "For this Metro 328 recovery test, select Arduino Uno to use Wire timeouts."
#endif

// The ordinary product getter does not report failed I2C reads. This small
// subclass uses the library's checked read so a disconnect cannot look valid.
class RecoverySeesaw : public Adafruit_seesaw {
public:
  int32_t getCheckedProductID();
};

RecoverySeesaw ss;
const uint8_t seesawAddress = 0x49;
const uint16_t expectedProduct = 6540;
const uint8_t ledPin = 24; // C031 PC15 onboard LED, NOT a Metro pin
bool online = false;
bool ledState = false;
uint32_t connectionCount = 0;
uint32_t lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw recovery / LED test\n\r"));

  // Bound each I2C wait to 25 ms and reset the Metro's I2C controller on timeout.
  // Adafruit_seesaw.begin() initializes Wire; no raw Wire transactions are used.
  Wire.setWireTimeout(25000, true);
  Serial.print(F("Waiting for PID 6540 at 0x49; PC15 LED will blink.\n\r"));
  Serial.print(F("Reset/power-cycle the C031 without resetting the Metro.\n\r"));
  Serial.print(F("Also try Metro-only reset and cold-starting both.\n\r\n\r"));
}

void loop() {
  Wire.clearWireTimeoutFlag();

  if (!online) {
    // Do not send a software reset: we want to observe the real startup state.
    if (!ss.begin(seesawAddress, -1, false)) {
      reportOffline();
      delay(500);
      return;
    }
  }

  int32_t product = ss.getCheckedProductID();
  if (product != expectedProduct || Wire.getWireTimeoutFlag()) {
    if (product >= 0 && product != expectedProduct) {
      Serial.print(F("Wrong product ID: "));
      Serial.print(product);
      Serial.print(F("; leaving LED untouched.\n\r"));
    }
    reportOffline();
    delay(500);
    return;
  }

  // Reapply OUTPUT every time so even a reset between polls restores blinking.
  // This is host-side recovery, not evidence that GPIO state survives a reset.
  ss.pinMode(ledPin, OUTPUT);
  ledState = !ledState;
  ss.digitalWrite(ledPin, ledState);

  // Confirm communication still works after the LED commands.
  if (ss.getCheckedProductID() != expectedProduct || Wire.getWireTimeoutFlag()) {
    reportOffline();
    delay(500);
    return;
  }

  // PC15 is GPIO bit 24, in the first reply byte. Check both HIGH and LOW
  // readback; visual confirmation is still needed to prove the LED lights.
  if (ss.digitalRead(ledPin) != ledState || Wire.getWireTimeoutFlag()) {
    Serial.print(F("LED GPIO readback mismatch; stopping this blink.\n\r"));
    ss.digitalWrite(ledPin, LOW);
    reportOffline();
    delay(500);
    return;
  }

  if (!online) {
    online = true;
    connectionCount++;
    Serial.print(F("ONLINE: PID 6540 at 0x49; LED blink commands resumed. Connection #"));
    Serial.print(connectionCount);
    Serial.print(F("\n\r"));
    lastHeartbeat = millis();
  }

  if (millis() - lastHeartbeat >= 5000) {
    Serial.print(F("Alive: PID 6540 responding. Check that the LED is blinking.\n\r"));
    lastHeartbeat = millis();
  }
  delay(250);
}

int32_t RecoverySeesaw::getCheckedProductID() {
  uint8_t version[4] = {0};
  if (!read(SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, version, sizeof(version))) {
    return -1;
  }
  // The first two version bytes contain the product ID, most significant first.
  return word(version[0], version[1]);
}

void reportOffline() {
  if (Wire.getWireTimeoutFlag()) {
    Serial.print(F("I2C TIMEOUT: Metro bus controller reset; retrying.\n\r"));
    Wire.clearWireTimeoutFlag();
  }
  if (online) {
    Serial.print(F("OFFLINE: lost communication; waiting for the C031.\n\r"));
  }
  online = false;
}
