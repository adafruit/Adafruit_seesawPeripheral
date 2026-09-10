// Adafruit STM32C011 seesaw bench host, Arduino Uno / Metro Mini.
// ASCII commands (hexadecimal bytes):
// W address module register [data...] -> OK or ERR
// R address module register count -> DATA xx xx ... or ERR
// P encoder-mask phase -> hold quadrature phase (bits A/B), 3 releases both.
// E encoder-mask direction detents step-ms -> generate quadrature, then release.
// C clock -> 1 selects 100 kHz, 4 selects 400 kHz I2C.
// B address -> BEGIN ACCEPTED/REJECTED using the installed seesaw host library.
// H subcommand [arguments] -> host-library calls; see executeCommand().
// S mode order -> start INPUT-ONLY hardware SPI observer (D10 CS, D11 MOSI, D13 SCK).
// T -> CAP count first-us last-us [up to 64 bytes], all hexadecimal.
// X -> stop observer. D12/MISO is NEVER an output.
// Q count -> burst write-only SPI chunks without delays, for queue-overflow testing.
// D4..D11 only pull LOW or release INPUT: NEVER output 5 V or enable pull-ups.
// The C011 supplies the 3.3 V encoder pull-ups. D0..D3 are not touched.
#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <seesaw_spi.h>

char command[110];
uint8_t commandLength = 0;
const uint8_t eepromModule = 0x0D;
const uint8_t encoderPins[4][2] = {{4, 5}, {6, 7}, {8, 9}, {10, 11}};
class HostSeesaw : public Adafruit_seesaw {
public:
  // Expose only the mapping query for this engineering test.
  using Adafruit_seesaw::getI2CaddrEEPROMloc;
};
HostSeesaw hostSeesaw;
seesaw_SPI spiHost;
volatile uint8_t spiCapture[64];
volatile uint16_t spiCaptureCount = 0;
volatile uint32_t spiFirstMicros = 0, spiLastMicros = 0;
void executeCommand();
void encoderPhase(uint8_t mask, uint8_t phase);

void setup() {
  Serial.begin(115200);
  encoderPhase(0x0F, 3);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  delay(250);
  Serial.println(F("Adafruit C011 seesaw bench host READY"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      command[commandLength] = 0;
      executeCommand();
      commandLength = 0;
    } else if (c != '\r') {
      if (commandLength < sizeof(command) - 1) {
        command[commandLength++] = c;
      } else {
        commandLength = 0;
        Serial.println(F("ERR line too long"));
      }
    }
  }
}

ISR(SPI_STC_vect) {
  uint8_t value = SPDR;
  uint32_t timestamp = micros();
  if (spiCaptureCount == 0) spiFirstMicros = timestamp;
  spiLastMicros = timestamp;
  if (spiCaptureCount < sizeof(spiCapture)) spiCapture[spiCaptureCount] = value;
  if (spiCaptureCount != 65535) spiCaptureCount++;
}

void executeCommand() {
  char *token = strtok(command, " ");
  if (!token) return;
  char operation = token[0];
  uint8_t bytes[34];
  uint8_t count = 0;
  while ((token = strtok(NULL, " ")) && count < sizeof(bytes)) {
    char *end;
    unsigned long value = strtoul(token, &end, 16);
    if (*end || value > 255) {
      Serial.println(F("ERR invalid byte"));
      return;
    }
    bytes[count++] = value;
  }
  if (operation == 'J' && count >= 1) {
    bool okay = false;
    uint8_t tx[96], rx[96];
    if (bytes[0] == 0 && count == 7) {
      uint32_t frequency = ((uint32_t)bytes[3] << 24) | ((uint32_t)bytes[4] << 16) |
                           ((uint32_t)bytes[5] << 8) | bytes[6];
      okay = spiHost.begin(0x49, -1, false) && spiHost.beginSPI(frequency, bytes[1], bytes[2]);
      if (okay) {
        Serial.print(F("CLOCK "));
        Serial.println(spiHost.getSPIClockFrequency(), HEX);
        return;
      }
    } else if ((bytes[0] == 1 || bytes[0] == 2 || bytes[0] == 5) && count == 3 &&
               bytes[1] <= 3 && bytes[2] <= sizeof(tx)) {
      for (uint8_t i = 0; i < bytes[2]; i++) tx[i] = i * 37 + 13;
      okay = spiHost.transfer(bytes[0] == 5 ? NULL : tx, bytes[0] == 2 ? NULL : rx,
                             bytes[2], bytes[1] & 1, bytes[1] & 2);
      if (okay && bytes[0] != 2) {
        Serial.print(F("DATA"));
        for (uint8_t i = 0; i < bytes[2]; i++) {
          Serial.print(' ');
          if (rx[i] < 16) Serial.print('0');
          Serial.print(rx[i], HEX);
        }
        Serial.println();
        return;
      }
    } else if (bytes[0] == 3 && count == 1) {
      okay = spiHost.abortSPI(true);
    } else if (bytes[0] == 4 && count == 1) {
      Serial.print(F("LAST "));
      Serial.println(spiHost.getLastSPIError(), HEX);
      return;
    }
    if (okay) Serial.println(F("OK"));
    else {
      Serial.print(F("FAIL "));
      Serial.println(spiHost.getLastSPIError(), HEX);
    }
    return;
  }
  if (operation == 'Z' && count == 1) { // Empty hardware I2C probe for BusDevice.
    Wire.beginTransmission(bytes[0]);
    Serial.println(Wire.endTransmission() ? F("ERR probe") : F("OK"));
    return;
  }
  if (operation == 'U' && count == 2 && bytes[1] > 0 && bytes[1] <= 32) {
    // Raw read with no prefix write: preserve the CircuitPython transaction.
    if (Wire.requestFrom(bytes[0], bytes[1]) != bytes[1]) {
      Serial.println(F("ERR raw read"));
      return;
    }
    Serial.print(F("DATA"));
    while (Wire.available()) {
      uint8_t value = Wire.read();
      Serial.print(' ');
      if (value < 16) Serial.print('0');
      Serial.print(value, HEX);
    }
    Serial.println();
    return;
  }
  if (operation == 'S' && count == 2 && bytes[0] <= 3 && bytes[1] <= 1) {
    SPCR = 0;
    for (uint8_t pin = 10; pin <= 13; pin++) {
      digitalWrite(pin, LOW);
      pinMode(pin, INPUT); // Includes MISO: observe only, never drive 5 V.
    }
    noInterrupts();
    spiCaptureCount = 0;
    spiFirstMicros = spiLastMicros = 0;
    uint8_t discard = SPSR;
    discard = SPDR;
    (void)discard;
    SPCR = _BV(SPE) | _BV(SPIE) | ((bytes[0] & 2) ? _BV(CPOL) : 0) |
           ((bytes[0] & 1) ? _BV(CPHA) : 0) | (bytes[1] ? _BV(DORD) : 0);
    interrupts();
    Serial.println(F("OK"));
    return;
  }
  if (operation == 'T' && count == 0) {
    noInterrupts();
    uint16_t captured = spiCaptureCount;
    uint32_t first = spiFirstMicros, last = spiLastMicros;
    interrupts();
    Serial.print(F("CAP "));
    Serial.print(captured, HEX);
    Serial.print(' ');
    Serial.print(first, HEX);
    Serial.print(' ');
    Serial.print(last, HEX);
    for (uint8_t i = 0; i < min(captured, (uint16_t)sizeof(spiCapture)); i++) {
      Serial.print(' ');
      if (spiCapture[i] < 16) Serial.print('0');
      Serial.print(spiCapture[i], HEX);
    }
    Serial.println();
    return;
  }
  if (operation == 'X' && count == 0) {
    SPCR = 0;
    Serial.println(F("OK"));
    return;
  }
  if (operation == 'Q' && count == 1 && bytes[0] <= 32) {
    uint8_t packet[32] = {0x13, 0x02, 0x04}; // SPI TRANSFER, discard RX, CS already held.
    for (uint8_t i = 3; i < sizeof(packet); i++) packet[i] = i ^ 0xA5;
    for (uint8_t i = 0; i < bytes[0]; i++) {
      Wire.beginTransmission(0x49);
      Wire.write(packet, sizeof(packet));
      if (Wire.endTransmission()) {
        Serial.println(F("ERR SPI burst I2C"));
        return;
      }
    }
    Serial.println(F("OK"));
    return;
  }
  if (operation == 'C' && count == 1 && (bytes[0] == 1 || bytes[0] == 4)) {
    Wire.setClock((uint32_t)bytes[0] * 100000);
    Serial.println(F("OK"));
    return;
  }
  if (operation == 'B' && count == 1 && bytes[0] >= 8 && bytes[0] <= 0x77) {
    Adafruit_seesaw seesaw;
    // No software reset: this is a host-driver acceptance check only.
    bool accepted = seesaw.begin(bytes[0], -1, false);
    Serial.println(accepted ? F("BEGIN ACCEPTED") : F("BEGIN REJECTED"));
    return;
  }
  if (operation == 'H' && count >= 1) {
    uint16_t value = 0;
    uint16_t rawADC = 0;
    if (bytes[0] == 0 && count == 2) { // Begin, including software reset.
      Serial.println(hostSeesaw.begin(bytes[1]) ? F("OK") : F("ERR host begin"));
      return;
    } else if (bytes[0] == 1 && count == 2) { // Ten-bit analogRead.
      value = hostSeesaw.analogRead(bytes[1]);
      // Read the same cached conversion, WITHOUT selecting a new ADC request.
      if (Wire.requestFrom((uint8_t)0x49, (uint8_t)2) != 2) {
        Serial.println(F("ERR cached ADC read"));
        return;
      }
      rawADC = (uint16_t)Wire.read() << 8;
      rawADC |= Wire.read();
    } else if (bytes[0] == 2 && count == 3) { // GPIO pin mode.
      hostSeesaw.pinMode(bytes[1], bytes[2]);
    } else if (bytes[0] == 3 && count == 3) { // GPIO write.
      hostSeesaw.digitalWrite(bytes[1], bytes[2]);
    } else if (bytes[0] == 4 && count == 2) { // GPIO read.
      value = hostSeesaw.digitalRead(bytes[1]);
    } else if (bytes[0] == 5 && count == 5) { // PWM pin, value high/low, width.
      hostSeesaw.analogWrite(bytes[1], ((uint16_t)bytes[2] << 8) | bytes[3], bytes[4]);
    } else if (bytes[0] == 6 && count == 4) { // PWM frequency in hertz.
      hostSeesaw.setPWMFreq(bytes[1], ((uint16_t)bytes[2] << 8) | bytes[3]);
    } else if (bytes[0] == 7 && count == 1) { // Persistent I2C-address location.
      value = hostSeesaw.getI2CaddrEEPROMloc();
    } else {
      Serial.println(F("ERR host command"));
      return;
    }
    delay(2); // C011 completes deferred pin changes outside its I2C interrupt.
    Serial.print(F("VALUE "));
    Serial.print(value, HEX);
    if (bytes[0] == 1) {
      Serial.print(F(" RAW "));
      Serial.print(rawADC, HEX);
    }
    Serial.println();
    return;
  }
  if (operation == 'P' && count == 2 && bytes[0] > 0 && bytes[0] <= 15 && bytes[1] <= 3) {
    encoderPhase(bytes[0], bytes[1]);
    Serial.println(F("OK"));
    return;
  }
  if (operation == 'E' && count == 4 && bytes[0] > 0 && bytes[0] <= 15 &&
      bytes[1] <= 1 && bytes[2] <= 64 && bytes[3] >= 1 && bytes[3] <= 20) {
    // Direction 0: 3->1->0->2->3, the AVR seesaw negative direction.
    const uint8_t phases[2][4] = {{1, 0, 2, 3}, {2, 0, 1, 3}};
    encoderPhase(bytes[0], 3);
    delay(bytes[3]);
    for (uint8_t step = 0; step < bytes[2]; step++) {
      for (uint8_t edge = 0; edge < 4; edge++) {
        encoderPhase(bytes[0], phases[bytes[1]][edge]);
        delay(bytes[3]);
      }
    }
    encoderPhase(bytes[0], 3);
    Serial.println(F("OK"));
    return;
  }
  if (count < 3 || bytes[0] < 8 || bytes[0] > 0x77 ||
      (operation != 'R' && operation != 'W') ||
      (operation == 'W' && count > 33) ||
      (operation == 'R' && (count != 4 || bytes[3] == 0 || bytes[3] > 32))) {
    Serial.println(F("ERR command"));
    return;
  }
  Wire.beginTransmission(bytes[0]);
  Wire.write(bytes + 1, operation == 'R' ? 2 : count - 1);
  uint8_t error = Wire.endTransmission();
  if (error) {
    Serial.print(F("ERR I2C "));
    Serial.println(error);
    return;
  }
  if (operation == 'W') {
    // Allows the peripheral to finish deferred work before the next command.
    if (bytes[1] == eepromModule) delay(100); // One reserved flash page is rewritten.
    else delay(2);
    Serial.println(F("OK"));
    return;
  }
  delayMicroseconds(2000);
  if (Wire.requestFrom(bytes[0], bytes[3]) != bytes[3]) {
    Serial.println(F("ERR short read"));
    return;
  }
  Serial.print(F("DATA"));
  while (Wire.available()) {
    uint8_t value = Wire.read();
    Serial.print(' ');
    if (value < 16) Serial.print('0');
    Serial.print(value, HEX);
  }
  Serial.println();
}

void encoderPhase(uint8_t mask, uint8_t phase) {
  for (uint8_t encoder = 0; encoder < 4; encoder++) {
    if (!(mask & (1 << encoder))) continue;
    for (uint8_t channel = 0; channel < 2; channel++) {
      uint8_t pin = encoderPins[encoder][channel];
      // LOW clears the AVR PORT latch before DDR can enable its output.
      digitalWrite(pin, LOW);
      if (phase & (1 << channel)) pinMode(pin, INPUT);
      else pinMode(pin, OUTPUT);
    }
  }
}
