// Adafruit STM32C011 bootloader USB-to-I2C bridge.
// Run this on a classic Nano, Uno, Metro 328, or Metro Mini, NOT on the STM32.
// Use the breakout's level-shifted QT bus with a 5 V host.
// Wiring: extras/stm32_i2c_bootloader/README.md in this library.
// Web uploader: https://adafruit.github.io/Adafruit_seesawPeripheral/bootloader/
// A factory-blank C011 enters ROM at power-on; the bridge needs no BOOT0 wire.
// ASCII commands use hexadecimal bytes:
// S scans 7-bit I2C addresses (address-only probes, no register writes).
// W address data... ; U address count ; R address module register count.
// O address queries C011 option bytes (64 = already in ROM, else seesaw address).
// O address nBOOT_SEL nBOOT1 nBOOT0 sets only those three bits (each 00 or 01).
// Reset/power-cycle the C011 before EACH option write: one write per ROM session.
#include <Wire.h>
#include <util/twi.h>

#if !defined(__AVR_ATmega328P__)
#error This bridge requires an ATmega328P Nano, Uno, or Metro 328.
#endif

const uint8_t ROM_ADDRESS = 0x64;
const uint8_t ROM_ACK = 0x79;
const uint8_t ROM_BUSY = 0x76;
const uint8_t ROM_GET = 0x00;
const uint8_t ROM_GET_ID = 0x02;
const uint8_t ROM_READ_MEMORY = 0x11;
const uint8_t ROM_WRITE_NO_STRETCH = 0x32;
const uint32_t OPTION_ADDRESS = 0x1FFF7800;
const uint8_t OPTION_SIZE = 128;

// RM0490 table 18: byte 3 of FLASH_OPTR, in AVR GCC bit-field order.
union BootOptions {
  uint8_t raw;
  struct {
    uint8_t nBootSel : 1;
    uint8_t nBoot1 : 1;
    uint8_t nBoot0 : 1;
    uint8_t unchanged : 5;
  } fields;
};
static_assert(sizeof(BootOptions) == 1, "Boot option byte layout");
// RM0490: SECR bit 16 is in byte 2 of the word at option offset 0x70.
union SecurityOptions {
  uint8_t raw;
  struct {
    uint8_t bootLock : 1;
    uint8_t unchanged : 7;
  } fields;
};
static_assert(sizeof(SecurityOptions) == 1, "Security option byte layout");

uint8_t expectedOptions[OPTION_SIZE];
bool optionsPendingVerification = false;
uint8_t lastROMReply = 0;
uint8_t lastTWIStatus = 0;
const uint8_t optionWordOffsets[] = {0, 8, 16, 24, 32, 40, 48, 112};
// C011-specific register masks from ST's stm32c011xx.h / RM0490. The ROM
// normalizes reserved bits through these registers during option programming.
// Compare ALL of OPTR (including the requested bits) and every implemented
// protection field, but do not mistake reserved-bit normalization for a change.
const uint32_t optionWordMasks[] = {0xFFFFFFFF, 0x0000003F, 0x8000003F,
                                    0x000F000F, 0x000F000F, 0x0000003F,
                                    0x0000003F, 0x0001001F};

char command[110];
uint8_t commandLength = 0;
bool discardLine = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
#ifndef ARDUINO_AVR_METRO
  // The legacy Adafruit Metro Wire library does not provide this timeout API.
  Wire.setWireTimeout(25000, true);
#endif
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
  if (operation == 'S' && count == 0) {
    scanBus();
    return;
  }
  if (operation == 'O') {
    if ((count != 1 && count != 4) || bytes[0] < 8 || bytes[0] > 0x77 ||
        (count == 4 && (bytes[1] > 1 || bytes[2] > 1 || bytes[3] > 1))) {
      Serial.print(F("ERR use O address [nBOOT_SEL nBOOT1 nBOOT0], bits 00/01\n\r"));
      return;
    }
    optionCommand(bytes, count);
    return;
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

void scanBus() {
  uint8_t found[112];
  uint8_t count = 0;
  for (uint8_t address = 0x08; address <= 0x77; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      found[count++] = address;
    } else if (error != 2) { // Address NACK means no device; other errors stop.
      Serial.print(F("ERR I2C scan failed at 0x"));
      Serial.print(address, HEX);
      Serial.print(F(" status="));
      Serial.print(error);
      Serial.print(F("\n\r"));
      return;
    }
  }
  Serial.print(F("SCAN"));
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(' ');
    if (found[i] < 16)
      Serial.print('0');
    Serial.print(found[i], HEX);
  }
  Serial.print(F("\n\r"));
}

bool romWrite(const uint8_t *data, uint8_t length) {
  Wire.beginTransmission(ROM_ADDRESS);
  if (Wire.write(data, length) != length) {
    Wire.endTransmission();
    return false;
  }
  return Wire.endTransmission() == 0;
}

bool romRead(uint8_t *data, uint8_t length) {
  if (Wire.requestFrom(ROM_ADDRESS, length) != length)
    return false;
  for (uint8_t i = 0; i < length; i++)
    data[i] = Wire.read();
  return true;
}

bool romAck() {
  lastROMReply = 0;
  uint32_t started = millis();
  do {
    uint8_t answer;
    if (!romRead(&answer, 1))
      return false;
    lastROMReply = answer;
    if (answer == ROM_ACK)
      return true;
    if (answer != ROM_BUSY)
      return false;
    delay(5);
  } while (millis() - started < 5000);
  return false;
}

bool romCommand(uint8_t code) {
  // AN4221 wire encoding: command followed by its complement.
  uint8_t packet[] = {code, (uint8_t)(code ^ 0xFF)};
  return romWrite(packet, sizeof(packet)) && romAck();
}

bool romAddress(uint32_t address) {
  // AN4221 uses big-endian addresses and an XOR checksum.
  uint8_t packet[] = {(uint8_t)(address >> 24), (uint8_t)(address >> 16),
                      (uint8_t)(address >> 8), (uint8_t)address, 0};
  for (uint8_t i = 0; i < 4; i++)
    packet[4] ^= packet[i];
  return romWrite(packet, sizeof(packet)) && romAck();
}

bool identifyROM(uint8_t address) {
  if (address != ROM_ADDRESS) {
    // Validate the running seesaw before requesting its ROM-entry reset.
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x01);
    if (Wire.endTransmission() != 0)
      return false;
    delay(2);
    if (Wire.requestFrom(address, (uint8_t)1) != 1 || Wire.read() != 0x90)
      return false;
    const uint8_t enterROM[] = {0x00, 0x7E, 0xB0, 0x07, 0x10, 0xAD};
    Wire.beginTransmission(address);
    Wire.write(enterROM, sizeof(enterROM));
    if (Wire.endTransmission() != 0)
      return false;
    delay(500);
  }
  uint8_t response[20];
  if (!romCommand(ROM_GET_ID) || !romRead(response, 3) || !romAck())
    return false;
  if (response[0] != 1 || response[1] != 0x04 || response[2] != 0x43)
    return false;
  if (!romCommand(ROM_GET) || !romRead(response, 20) || !romAck())
    return false;
  if (response[0] != 18 || response[1] != 0x11)
    return false;
  bool hasRead = false;
  bool hasWrite = false;
  for (uint8_t i = 2; i < sizeof(response); i++) {
    if (response[i] == ROM_READ_MEMORY)
      hasRead = true;
    if (response[i] == ROM_WRITE_NO_STRETCH)
      hasWrite = true;
  }
  return hasRead && hasWrite;
}

bool readOptions(uint8_t *options) {
  for (uint8_t offset = 0; offset < OPTION_SIZE; offset += 32) {
    if (!romCommand(ROM_READ_MEMORY) || !romAddress(OPTION_ADDRESS + offset))
      return false;
    const uint8_t length[] = {31, 0xE0}; // 32 bytes and complement of 31.
    if (!romWrite(length, sizeof(length)) || !romAck() ||
        !romRead(options + offset, 32))
      return false;
  }
  return true;
}

bool waitTWI(uint8_t expectedStatus) {
  // Poll the AVR TWI hardware only for the long option packet. Wire's ISR is
  // disabled for this transaction; the normal timer/serial interrupts remain on.
  uint32_t started = micros();
  while (!(TWCR & _BV(TWINT))) {
    if (micros() - started > 25000) {
      lastTWIStatus = TW_STATUS;
      return false;
    }
  }
  lastTWIStatus = TW_STATUS;
  return lastTWIStatus == expectedStatus;
}

bool writeOptionPacket(const uint8_t *options) {
  // AN4221 erases the option area for a write. Send all 128 preserved bytes in
  // ONE transaction, not independent 24-byte application-flash writes. This
  // 130-byte frame exceeds Wire's fixed 32-byte buffer, so use the AVR hardware
  // directly without changing the installed core or its buffers.
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
  bool success = waitTWI(TW_START);
  if (success) {
    TWDR = ROM_ADDRESS * 2;
    TWCR = _BV(TWINT) | _BV(TWEN);
    success = waitTWI(TW_MT_SLA_ACK);
  }
  uint8_t checksum = OPTION_SIZE - 1;
  for (uint16_t i = 0; success && i < OPTION_SIZE + 2; i++) {
    uint8_t value;
    if (i == 0) {
      value = OPTION_SIZE - 1;
    } else if (i <= OPTION_SIZE) {
      value = options[i - 1];
      checksum ^= value; // AN4221 packet checksum, not a register field.
    } else {
      value = checksum;
    }
    TWDR = value;
    TWCR = _BV(TWINT) | _BV(TWEN);
    success = waitTWI(TW_MT_DATA_ACK);
  }
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
  uint32_t started = micros();
  while (TWCR & _BV(TWSTO)) {
    if (micros() - started > 25000) {
      success = false;
      break;
    }
  }
  // Restore Wire's interrupt-driven controller, including after a failed write.
  TWCR = 0;
  Wire.begin();
  Wire.setClock(100000);
  return success;
}

void printOptions(const uint8_t *options) {
  Serial.print(F("OPTIONS"));
  for (uint8_t i = 0; i < OPTION_SIZE; i++) {
    Serial.print(' ');
    if (options[i] < 16)
      Serial.print('0');
    Serial.print(options[i], HEX);
  }
  BootOptions boot;
  boot.raw = options[3];
  Serial.print(F("\n\rnBOOT_SEL="));
  Serial.print(boot.fields.nBootSel);
  Serial.print(F(" nBOOT1="));
  Serial.print(boot.fields.nBoot1);
  Serial.print(F(" nBOOT0="));
  Serial.print(boot.fields.nBoot0);
  Serial.print(F(" BOOT_LOCK="));
  SecurityOptions security;
  security.raw = options[0x72];
  Serial.print(security.fields.bootLock);
  Serial.print(F(" RDP=0x"));
  Serial.print(options[0], HEX);
  Serial.print(F("\n\r"));
}

void optionCommand(const uint8_t *arguments, uint8_t count) {
  if (!identifyROM(arguments[0])) {
    Serial.print(F("ERR C011 ROM entry/identification failed\n\r"));
    return;
  }
  uint8_t options[OPTION_SIZE];
  if (!readOptions(options)) {
    Serial.print(F("ERR option read failed\n\r"));
    return;
  }
  printOptions(options);
  if (!validOptionComplements(options)) {
    Serial.print(F("ERR option/complement mismatch; refusing write\n\r"));
    return;
  }
  if (optionsPendingVerification) {
    if (!optionsMatch(options, expectedOptions)) {
      Serial.print(F("ERR pending option write does not match; no further writes\n\r"));
      return;
    }
    optionsPendingVerification = false;
    Serial.print(F("VERIFIED boot options and all other defined settings; complements valid\n\r"));
  }
  if (count == 1)
    return;

  SecurityOptions security;
  security.raw = options[0x72];
  if (options[0] != 0xAA || security.fields.bootLock) {
    Serial.print(F("ERR requires RDP level 0 and BOOT_LOCK=0; refusing write\n\r"));
    return;
  }
  BootOptions boot;
  boot.raw = options[3];
  boot.fields.nBootSel = arguments[1];
  boot.fields.nBoot1 = arguments[2];
  boot.fields.nBoot0 = arguments[3];
  if (boot.raw == options[3]) {
    Serial.print(F("OK boot options already match; no write\n\r"));
    return;
  }
  options[3] = boot.raw;
  options[7] = boot.raw ^ 0xFF; // Required complement of the modified byte.
  if (!romCommand(ROM_WRITE_NO_STRETCH) || !romAddress(OPTION_ADDRESS)) {
    Serial.print(F("ERR option write setup failed; no data sent\n\r"));
    return;
  }
  memcpy(expectedOptions, options, OPTION_SIZE);
  optionsPendingVerification = true;
  if (!writeOptionPacket(options)) {
    Serial.print(F("ERR option packet failed; TWI status=0x"));
    Serial.print(lastTWIStatus, HEX);
    Serial.print(F("; write unconfirmed, do not retry\n\r"));
  } else if (!romAck()) {
    Serial.print(F("ERR option write unconfirmed; ROM reply=0x"));
    Serial.print(lastROMReply, HEX);
    Serial.print(F(" (00=unavailable); do not retry\n\r"));
  } else {
    Serial.print(F("ACK option write; readback still required\n\r"));
  }
  Serial.print(F("Keep Metro powered. Reset/power-cycle C011 before another option write\n\r"));
  Serial.print(F("Re-enter ROM, then query O 64 (or O 49 from running seesaw) to verify\n\r"));
}

bool validOptionComplements(const uint8_t *options) {
  // RM0490 tables 17/18: each implemented word has a complemented copy.
  for (uint8_t word = 0; word < sizeof(optionWordOffsets); word++) {
    for (uint8_t byte = 0; byte < 4; byte++) {
      uint8_t index = optionWordOffsets[word] + byte;
      if ((uint8_t)(options[index] ^ options[index + 4]) != 0xFF)
        return false;
    }
  }
  return true;
}

bool optionsMatch(const uint8_t *actual, const uint8_t *expected) {
  for (uint8_t word = 0; word < sizeof(optionWordOffsets); word++) {
    uint32_t actualWord, expectedWord;
    memcpy(&actualWord, actual + optionWordOffsets[word], 4);
    memcpy(&expectedWord, expected + optionWordOffsets[word], 4);
    // Hardware-defined masks are needed here: ROM rewrites reserved bits even
    // when given their original values. Never mask any defined protection bit.
    if ((actualWord & optionWordMasks[word]) !=
        (expectedWord & optionWordMasks[word]))
      return false;
  }
  // The unimplemented doublewords are not register fields and must stay intact.
  return memcmp(actual + 56, expected + 56, 56) == 0 &&
         memcmp(actual + 120, expected + 120, 8) == 0;
}
