// Adafruit STM32C011 bootloader USB-to-I2C bridge.
// Run this on a classic Nano, Uno, or Metro Mini, NOT on the STM32 target.
// Use the breakout's level-shifted QT bus with a 5 V host. See ../README.md.
// ASCII commands use hexadecimal bytes:
// W address data... ; U address count ; R address module register count.
#include <Wire.h>

char command[110];
uint8_t commandLength = 0;
bool discardLine = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Use the breakout's external pull-ups, not AVR pull-ups to 5 V.
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  delay(250);
  Serial.print(F("Adafruit STM32C011 I2C bootloader bridge READY\n\r"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (discardLine) {
        Serial.print(F("ERR line too long\n\r"));
      } else {
        command[commandLength] = 0;
        executeCommand();
      }
      commandLength = 0;
      discardLine = false;
    } else if (c != '\r' && !discardLine) {
      if (commandLength < sizeof(command) - 1) {
        command[commandLength++] = c;
      } else {
        discardLine = true;
      }
    }
  }
}

void executeCommand() {
  char *token = strtok(command, " ");
  if (!token)
    return;
  if (strlen(token) != 1) {
    Serial.print(F("ERR operation\n\r"));
    return;
  }
  char operation = token[0];
  uint8_t bytes[33];
  uint8_t count = 0;
  while ((token = strtok(NULL, " "))) {
    char *end;
    unsigned long value = strtoul(token, &end, 16);
    if (*end || value > 255 || count == sizeof(bytes)) {
      Serial.print(F("ERR invalid byte count or value\n\r"));
      return;
    }
    bytes[count++] = value;
  }
  if (count < 2 || bytes[0] < 8 || bytes[0] > 0x77 ||
      (operation != 'W' && operation != 'U' && operation != 'R') ||
      (operation == 'U' && (count != 2 || bytes[1] == 0 || bytes[1] > 32)) ||
      (operation == 'R' && (count != 4 || bytes[3] == 0 || bytes[3] > 32))) {
    Serial.print(F("ERR command\n\r"));
    return;
  }
  if (operation != 'U') {
    Wire.beginTransmission(bytes[0]);
    Wire.write(bytes + 1, operation == 'R' ? 2 : count - 1);
    uint8_t error = Wire.endTransmission();
    if (error) {
      Serial.print(F("ERR I2C "));
      Serial.print(error);
      Serial.print(F("\n\r"));
      return;
    }
    // Seesaw runs deferred commands after its I2C interrupt returns.
    delay(2);
  }
  if (operation == 'W') {
    Serial.print(F("OK\n\r"));
    return;
  }
  uint8_t length = bytes[1];
  if (operation == 'R')
    length = bytes[3];
  if (Wire.requestFrom(bytes[0], length) != length) {
    Serial.print(F("ERR short I2C read\n\r"));
    return;
  }
  Serial.print(F("DATA"));
  while (Wire.available()) {
    uint8_t value = Wire.read();
    Serial.print(' ');
    if (value < 16)
      Serial.print('0');
    Serial.print(value, HEX);
  }
  Serial.print(F("\n\r"));
}
