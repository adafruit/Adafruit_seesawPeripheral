// Adafruit STM32C031 seesaw reconnect test on a Metro 328 / Uno.
// Select Arduino Uno in the IDE for Wire timeouts; upload only to the Metro.
// Requires Adafruit_seesaw with STM32 support and C031 PID 6540 firmware.
// Connect through the breakout's level-shifted I2C bus, with shared GND.
// This read-only test does not drive GPIOs or send a software reset.
// Serial Monitor: 115200 baud. No Serial Monitor is required for startup.
// Try C031 reset/power cycling, Metro reset, and cold-starting both boards.
// For a true C031 power-off, avoid back-powering through other connections.
// OFFLINE -> ONLINE shows reconnection; very short resets may fall between polls.

#include <Adafruit_seesaw.h>
#include <Wire.h>

#ifndef SEESAW_HW_ID_CODE_STM32C031
#error "Install the current Adafruit_Seesaw library from GitHub for STM32 support."
#endif
#ifndef ARDUINO_AVR_UNO
#error "For this Metro 328 recovery test, select Arduino Uno to use Wire timeouts."
#endif

// Use checked reads so a disconnect cannot look like a valid product ID.
class RecoverySeesaw : public Adafruit_seesaw {
public:
  int32_t getCheckedProductID();
};

RecoverySeesaw ss;
const uint8_t seesawAddress = 0x49;
const uint16_t expectedProduct = 6540;
bool online = false;
uint32_t connectionCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.print(F("Adafruit STM32C031 seesaw reconnect test\n\r"));

  // Bound each I2C wait to 25 ms and reset the Metro's I2C controller on timeout.
  // Adafruit_seesaw.begin() initializes Wire; no raw Wire transactions are used.
  Wire.setWireTimeout(25000, true);
  Serial.print(F("Waiting for PID 6540 at 0x49.\n\r"));
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
      Serial.print(F("; leaving target untouched.\n\r"));
    }
    reportOffline();
    delay(500);
    return;
  }

  if (!online) {
    online = true;
    connectionCount++;
    Serial.print(F("ONLINE: PID 6540 at 0x49. Connection #"));
    Serial.print(connectionCount);
    Serial.print(F("\n\r"));
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
