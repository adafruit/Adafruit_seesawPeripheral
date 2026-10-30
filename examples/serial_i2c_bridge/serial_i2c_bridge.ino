// Adafruit STM32C011/C031 bootloader USB-to-I2C bridge.
// Run this on the USB host bridge, NOT on the STM32 being programmed.
// Supports classic AVR, Arduino-Pico RP2040/RP2350, ESP32, and ESP8266 hosts.
// Use the breakout's level-shifted QT bus with a 5 V host.
// Wiring: extras/stm32_i2c_bootloader/README.md in this library.
// Web uploader: https://adafruit.github.io/Adafruit_seesawPeripheral/bootloader/
// A factory-blank C011/C031 enters ROM at power-on; no BOOT0 wire is needed.
// ASCII commands use hexadecimal bytes:
// S scans I2C addresses; ROM addresses use a complete read-only GET_ID command.
// W address data... ; U address count ; R address module register count.
// O address queries C011/C031 options (64/63 = ROM, else seesaw address).
// O address nBOOT_SEL nBOOT1 nBOOT0 sets only those three bits (each 00 or 01).
// Reset/power-cycle the STM32 before EACH option write: one write per ROM session.
#include <Wire.h>

// Optional pin overrides for RP2040/RP2350 and ESP32/ESP8266. Otherwise use
// the selected board's default Wire SDA/SCL pins (A4/A5 on Uno/Metro 328).
// #define BRIDGE_SDA_PIN 4
// #define BRIDGE_SCL_PIN 5
#if defined(BRIDGE_SDA_PIN) != defined(BRIDGE_SCL_PIN)
#error Define both BRIDGE_SDA_PIN and BRIDGE_SCL_PIN, or neither.
#endif

#if defined(ARDUINO_ARCH_AVR) && defined(TWCR) && defined(TWDR)
#define BRIDGE_AVR_TWI 1
#include <util/twi.h>
#else
#define BRIDGE_AVR_TWI 0
#endif
#if defined(ESP8266)
#include <twi.h>
#endif

// Other cores still support scans, firmware updates, and option reads. Option
// writes need a known 130-byte transmit capacity. This override DECLARES the
// core's actual capacity; it does not resize a separately compiled Wire library.
#ifndef BRIDGE_WIRE_CAPACITY
#if defined(I2C_BUFFER_LENGTH)
#define BRIDGE_WIRE_CAPACITY I2C_BUFFER_LENGTH
#elif defined(WIRE_BUFFER_SIZE)
#define BRIDGE_WIRE_CAPACITY WIRE_BUFFER_SIZE
#elif defined(BUFFER_LENGTH)
#define BRIDGE_WIRE_CAPACITY BUFFER_LENGTH
#else
#define BRIDGE_WIRE_CAPACITY 32
#endif
#endif

#define C011_ROM_ADDRESS 0x64
#define C031_ROM_ADDRESS 0x63
#define C011_SEESAW_ID 0x90
#define C031_SEESAW_ID 0x91
#define C011_ROM_PRODUCT_LOW 0x43
#define C031_ROM_PRODUCT_LOW 0x53

uint8_t romAddressI2C = C011_ROM_ADDRESS;
uint8_t pendingROMAddress = 0;
uint8_t romProductLow = C011_ROM_PRODUCT_LOW;
uint8_t romGetLength = 20;
const uint8_t ROM_ACK = 0x79;
const uint8_t ROM_BUSY = 0x76;
const uint8_t ROM_GET = 0x00;
const uint8_t ROM_GET_ID = 0x02;
const uint8_t ROM_READ_MEMORY = 0x11;
const uint8_t ROM_WRITE_NO_STRETCH = 0x32;
const uint32_t OPTION_ADDRESS = 0x1FFF7800;
const uint8_t OPTION_SIZE = 128;

// RM0490 table 18: masks in byte 3 of the serialized FLASH_OPTR word.
// Explicit hardware masks avoid compiler-specific C++ bit-field ordering.
const uint8_t NBOOT_SEL_MASK = 0x01;
const uint8_t NBOOT1_MASK = 0x02;
const uint8_t NBOOT0_MASK = 0x04;
// RM0490: SECR bit 16 is in byte 2 of the word at option offset 0x70.
const uint8_t BOOT_LOCK_MASK = 0x01;

uint8_t expectedOptions[OPTION_SIZE];
bool optionsPendingVerification = false;
bool busReady = true;
bool optionWriteSupported = BRIDGE_WIRE_CAPACITY >= OPTION_SIZE + 2;
uint8_t lastROMReply = 0;
uint8_t lastBusStatus = 0;
const uint8_t optionWordOffsets[] = {0, 8, 16, 24, 32, 40, 48, 112};
// C011/C031 register masks from ST's stm32c011xx.h / stm32c031xx.h. The ROM
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
#if BRIDGE_AVR_TWI || defined(ESP8266)
  optionWriteSupported = true; // These cores use their direct long-write path.
#elif defined(WIRE_HAS_BUFFER_SIZE)
  optionWriteSupported = Wire.setBufferSize(OPTION_SIZE + 2) >= OPTION_SIZE + 2;
#endif

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
#ifdef BRIDGE_SDA_PIN
  busReady = Wire.setSDA(BRIDGE_SDA_PIN) && Wire.setSCL(BRIDGE_SCL_PIN);
#endif
  Wire.begin();
  Wire.setTimeout(25, true); // Arduino-Pico uses milliseconds.
#elif defined(ESP32)
#ifdef BRIDGE_SDA_PIN
  busReady = Wire.begin(BRIDGE_SDA_PIN, BRIDGE_SCL_PIN);
#else
  busReady = Wire.begin();
#endif
  Wire.setTimeOut(25); // ESP32 uses milliseconds and a capital O.
#elif defined(ESP8266)
#ifdef BRIDGE_SDA_PIN
  Wire.begin(BRIDGE_SDA_PIN, BRIDGE_SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClockStretchLimit(25000); // ESP8266 uses microseconds.
#else
#ifdef BRIDGE_SDA_PIN
#error "Select SDA/SCL using the core Wire API; pin overrides support RP/ESP."
#endif
  Wire.begin();
#if defined(__AVR_ATmega328P__) && !defined(ARDUINO_AVR_METRO)
  // The legacy Adafruit Metro Wire library does not provide this timeout API.
  Wire.setWireTimeout(25000, true);
#endif
#endif
  Wire.setClock(100000);
  delay(250);
  Serial.print(F("Adafruit STM32C011/C031 I2C bootloader bridge READY\n\r"));
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
  if (!busReady) {
    Serial.print(F("ERR bridge I2C initialization failed; check SDA/SCL pins\n\r"));
    return;
  }
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
    bool romProbe = address == C011_ROM_ADDRESS || address == C031_ROM_ADDRESS;
    Wire.beginTransmission(address);
    if (romProbe) {
      // Empty write probes caused C031 ROM timeouts on the Metro Mini.
      // Complete a read-only GET_ID transaction instead.
      Wire.write(ROM_GET_ID);
      Wire.write((uint8_t)(ROM_GET_ID ^ 0xFF)); // AN4221 command complement.
    }
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      if (romProbe) {
        uint8_t previousROMAddress = romAddressI2C;
        romAddressI2C = address;
        uint8_t id[3]; // GET_ID: byte count followed by the two-byte chip ID.
        bool complete = romAck() && romRead(id, sizeof(id)) && romAck();
        romAddressI2C = previousROMAddress;
        if (!complete) {
          Serial.print(F("ERR ROM scan failed at 0x"));
          Serial.print(address, HEX);
          Serial.print(F("\n\r"));
          return;
        }
      }
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
  Wire.beginTransmission(romAddressI2C);
  if (Wire.write(data, length) != length) {
    Wire.endTransmission();
    return false;
  }
  return Wire.endTransmission() == 0;
}

bool romRead(uint8_t *data, uint8_t length) {
  if (Wire.requestFrom(romAddressI2C, length) != length)
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
  // O accepts either family's ROM address, or a running seesaw address.
  uint8_t family = 0;
  if (address == C011_ROM_ADDRESS) family = C011_SEESAW_ID;
  else if (address == C031_ROM_ADDRESS) family = C031_SEESAW_ID;
  else {
    // Validate the running seesaw before requesting its ROM-entry reset.
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x01);
    if (Wire.endTransmission() != 0)
      return false;
    delay(2);
    if (Wire.requestFrom(address, (uint8_t)1) != 1)
      return false;
    family = Wire.read();
    if (family != C011_SEESAW_ID && family != C031_SEESAW_ID)
      return false;
    // Do not enter a different family while a saved write needs verification.
    uint8_t destination = family == C031_SEESAW_ID ? C031_ROM_ADDRESS : C011_ROM_ADDRESS;
    if (optionsPendingVerification && pendingROMAddress != destination)
      return false;
    const uint8_t enterROM[] = {0x00, 0x7E, 0xB0, 0x07, 0x10, 0xAD};
    Wire.beginTransmission(address);
    Wire.write(enterROM, sizeof(enterROM));
    if (Wire.endTransmission() != 0)
      return false;
    delay(500);
  }
  romAddressI2C = family == C031_SEESAW_ID ? C031_ROM_ADDRESS : C011_ROM_ADDRESS;
  romProductLow = family == C031_SEESAW_ID ? C031_ROM_PRODUCT_LOW : C011_ROM_PRODUCT_LOW;
  // Both observed C011/C031 ROMs return 18 commands (20-byte frame).
  romGetLength = 20;
  if (optionsPendingVerification && pendingROMAddress != romAddressI2C)
    return false;
  uint8_t response[20];
  if (!romCommand(ROM_GET_ID) || !romRead(response, 3) || !romAck())
    return false;
  if (response[0] != 1 || response[1] != 0x04 || response[2] != romProductLow)
    return false;
  if (!romCommand(ROM_GET) || !romRead(response, romGetLength))
    return false;
  Serial.print(F("ROM GET count="));
  Serial.print(response[0]);
  Serial.print(F(" protocol=0x"));
  Serial.print(response[1], HEX);
  Serial.print(F("\n\r"));
  if (!romAck())
    return false;
  if (response[0] != romGetLength - 2 || response[1] != 0x11)
    return false;
  bool hasRead = false;
  bool hasWrite = false;
  for (uint8_t i = 2; i < romGetLength; i++) {
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

#if BRIDGE_AVR_TWI
bool waitTWI(uint8_t expectedStatus) {
  // Poll the AVR TWI hardware only for the long option packet. Wire's ISR is
  // disabled for this transaction; the normal timer/serial interrupts remain on.
  uint32_t started = micros();
  while (!(TWCR & _BV(TWINT))) {
    if (micros() - started > 25000) {
      lastBusStatus = TW_STATUS;
      return false;
    }
  }
  lastBusStatus = TW_STATUS;
  return lastBusStatus == expectedStatus;
}
#endif

bool writeOptionPacket(const uint8_t *options) {
  // AN4221 erases the option area for a write. Send all 128 preserved bytes in
  // ONE transaction, not independent 24-byte application-flash writes.
  // Never split this 130-byte frame or transmit a partially buffered packet.
#if BRIDGE_AVR_TWI
  // Classic AVR Wire buffers only 32 bytes; bypass it for this transaction.
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
  bool success = waitTWI(TW_START);
  if (success) {
    TWDR = romAddressI2C * 2;
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
#else
  uint8_t packet[OPTION_SIZE + 2];
  packet[0] = OPTION_SIZE - 1;
  uint8_t checksum = packet[0];
  for (uint8_t i = 0; i < OPTION_SIZE; i++) {
    packet[i + 1] = options[i];
    checksum ^= options[i]; // AN4221 packet checksum.
  }
  packet[OPTION_SIZE + 1] = checksum;
#if defined(ESP8266)
  // ESP8266 Wire has a fixed 128-byte buffer, but its underlying software-I2C
  // writer accepts the complete frame on the same pins initialized by Wire.
  lastBusStatus = twi_writeTo(romAddressI2C, packet, sizeof(packet), true);
#else
  Wire.beginTransmission(romAddressI2C);
  if (Wire.write(packet, sizeof(packet)) != sizeof(packet)) {
    lastBusStatus = 1; // Buffer overflow: do NOT send a truncated option frame.
    return false;
  }
  lastBusStatus = Wire.endTransmission();
#endif
  return lastBusStatus == 0;
#endif
}

void printOptions(const uint8_t *options) {
  Serial.print(F("OPTIONS"));
  for (uint8_t i = 0; i < OPTION_SIZE; i++) {
    Serial.print(' ');
    if (options[i] < 16)
      Serial.print('0');
    Serial.print(options[i], HEX);
  }
  Serial.print(F("\n\rnBOOT_SEL="));
  Serial.print((options[3] & NBOOT_SEL_MASK) != 0);
  Serial.print(F(" nBOOT1="));
  Serial.print((options[3] & NBOOT1_MASK) != 0);
  Serial.print(F(" nBOOT0="));
  Serial.print((options[3] & NBOOT0_MASK) != 0);
  Serial.print(F(" BOOT_LOCK="));
  Serial.print((options[0x72] & BOOT_LOCK_MASK) != 0);
  Serial.print(F(" RDP=0x"));
  Serial.print(options[0], HEX);
  Serial.print(F("\n\r"));
}

void optionCommand(const uint8_t *arguments, uint8_t count) {
  if (count == 4 && !optionWriteSupported) {
    Serial.print(F("ERR bridge needs a 130-byte I2C transmit buffer; no option write\n\r"));
    return;
  }
  if (!identifyROM(arguments[0])) {
    Serial.print(F("ERR C0 ROM entry/identification failed\n\r"));
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

  if (options[0] != 0xAA || (options[0x72] & BOOT_LOCK_MASK) != 0) {
    Serial.print(F("ERR requires RDP level 0 and BOOT_LOCK=0; refusing write\n\r"));
    return;
  }
  // Change only the three serialized hardware fields, preserving every other bit.
  uint8_t boot = options[3] & ~(NBOOT_SEL_MASK | NBOOT1_MASK | NBOOT0_MASK);
  if (arguments[1])
    boot |= NBOOT_SEL_MASK;
  if (arguments[2])
    boot |= NBOOT1_MASK;
  if (arguments[3])
    boot |= NBOOT0_MASK;
  if (boot == options[3]) {
    Serial.print(F("OK boot options already match; no write\n\r"));
    return;
  }
  options[3] = boot;
  options[7] = boot ^ 0xFF; // Required complement of the modified byte.
  if (!romCommand(ROM_WRITE_NO_STRETCH) || !romAddress(OPTION_ADDRESS)) {
    Serial.print(F("ERR option write setup failed; no data sent\n\r"));
    return;
  }
  memcpy(expectedOptions, options, OPTION_SIZE);
  pendingROMAddress = romAddressI2C;
  optionsPendingVerification = true;
  if (!writeOptionPacket(options)) {
    Serial.print(F("ERR option packet failed; bus status=0x"));
    Serial.print(lastBusStatus, HEX);
    Serial.print(F("; write unconfirmed, do not retry\n\r"));
  } else if (!romAck()) {
    Serial.print(F("ERR option write unconfirmed; ROM reply=0x"));
    Serial.print(lastROMReply, HEX);
    Serial.print(F(" (00=unavailable); do not retry\n\r"));
  } else {
    Serial.print(F("ACK option write; readback still required\n\r"));
  }
  Serial.print(F("Keep bridge powered. Reset/power-cycle STM32 before another option write\n\r"));
  Serial.print(F("Re-enter ROM, then query O 64 (C011) / O 63 (C031), or O 49 from seesaw to verify\n\r"));
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
    uint32_t actualWord = 0, expectedWord = 0;
    // The ROM sends little-endian words regardless of the bridge CPU's layout.
    for (uint8_t byte = 0; byte < 4; byte++) {
      uint8_t offset = optionWordOffsets[word] + byte;
      actualWord |= (uint32_t)actual[offset] << (8 * byte);
      expectedWord |= (uint32_t)expected[offset] << (8 * byte);
    }
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
