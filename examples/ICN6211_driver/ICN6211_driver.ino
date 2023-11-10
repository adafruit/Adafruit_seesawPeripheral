#define PRODUCT_CODE            6211
#define CONFIG_I2C_PERIPH_ADDR  0x49
#define CONFIG_UART_DEBUG          1

#define LED_BUILTIN    2
#define TFT_BACKLIGHT  7
#define TFT_RESET      0
#define TFT_CLOCK      15
#define TFT_DATA       14
#define TFT_CS         16

#define EEPROM_ADDRESS 0x50
#define EEPROM_SIZE 4096

#define CONFIG_PWM                 1


void sendSPIinit(const uint8_t PROGMEM *initcmd);
#include "SPI_initcodes.h"


#include "Adafruit_seesawPeripheral.h"

void setup() {
#if CONFIG_UART_DEBUG
  Serial.begin(115200);
  delay(500);
  Serial.println("ICN test");
#endif

  printI2Cscan();
  prettyPrintEEPROM();

  delay(10);
  if (configICN6211()) {
    Serial.println("Found and configured ICN6211");
  }
  
  //Adafruit_seesawPeripheral_begin();
  pinMode(TFT_RESET, OUTPUT);
  digitalWrite(TFT_RESET, LOW);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_CLOCK, OUTPUT);
  pinMode(TFT_DATA, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_CLOCK, HIGH);
  delay(10);
  digitalWrite(TFT_RESET, HIGH);
  delay(100);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  //sendSPIinit(HD40015C40_initcode);  // 4" round
  //sendSPIinit(TL021WVC02_initcode);   // 2.1" round
  sendSPIinit(TL040WVS01_initcode); // 3.4" 480x480
  while (1) {
   sendSPIcommand(0x22, NULL, 0);
   delay(1000);
   sendSPIcommand(0x23, NULL, 0);
   delay(1000);     
  }

/*
  digitalWrite(TFT_CS, LOW);
  SPItransfer(0xDA, false);
  Serial.println(SPIread(), HEX);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_CS, LOW);
  SPItransfer(0xDB, false);
  Serial.println(SPIread(), HEX);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_CS, LOW);
  SPItransfer(0xDC, false);
  Serial.println(SPIread(), HEX);
  digitalWrite(TFT_CS, HIGH);
*/
}


void loop() {
  //Adafruit_seesawPeripheral_run();
}


bool configICN6211() {
  // send ICN6211 configuration
  Wire.begin();
  // softreset
  write_i2c_register(0x2C, 0x09, 0x01);
  delay(10);

  // check vendorID & deviceID
  if ((read_i2c_register(0x2C, 0x00) != 0xC1) ||
      (read_i2c_register(0x2C, 0x01) != 0x62) ||
      (read_i2c_register(0x2C, 0x02) != 0x11)) {
        Wire.end();
        return false;
  }
  
  Serial.println("\nFound ICN6211");

  uint8_t reg, val;
  for (size_t i = 0; i < 0xFF; i += 2) {
    readEEPROM(0x400 + i, &reg, 1);
    readEEPROM(0x400 + i + 1, &val, 1);
    
    if (reg == 0xFF) break; // we're done!
    Serial.print("0x"); Serial.print(reg, HEX); 
    Serial.print(" = ");
    Serial.print("0x"); Serial.println(val, HEX);
    if (!write_i2c_register(0x2C, reg, val)) {
      Wire.end();
      return false;
    }
  }
  
  Wire.end();
  Serial.println();
  return true;
}

uint8_t write_i2c_register(uint8_t i2c_address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    return 0; // Transmission failed
  }
  return 1; // Success
}


uint8_t read_i2c_register(uint8_t i2c_address, uint8_t reg) {
  Wire.beginTransmission(i2c_address);
  if (Wire.write(reg) != 1) {
    Serial.println("Failed to write");
    return 0; // Write operation failed
  }

  if (Wire.endTransmission(false) != 0) {
    Serial.println("Failed to end");
    return 0; // Transmission failed to end properly
  }

  Wire.requestFrom(i2c_address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.println("Failed to read");
    return 0;
  }
  return Wire.read();
}


void printI2Cscan() {
  Wire.begin();
  Serial.print("\nI2C Scan:\t");
  for (uint8_t a=0x01; a<0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("0x");
      Serial.print(a, HEX);
      Serial.print(", ");
    }
  }
  Serial.println();
  Wire.end();
}

/************************** EEPROM Configuration */

void prettyPrintEEPROM() {
  const int BYTES_PER_LINE = 16;
  Wire.begin();
  
  bool previous_all_ff = false;
  bool current_all_ff = true;
  byte line_data[BYTES_PER_LINE];

  Serial.println("\nI2C EEPROM contents:");
  for (int i = 0; i < EEPROM_SIZE; i += BYTES_PER_LINE) {
    current_all_ff = true;
    readEEPROM(i, line_data, BYTES_PER_LINE);
    for (byte b : line_data) {
      if (b != 0xFF) {
        current_all_ff = false;
        break;
      }
    }

    if (!current_all_ff || !previous_all_ff || i + BYTES_PER_LINE >= EEPROM_SIZE) {
      if (previous_all_ff || (i + BYTES_PER_LINE >= EEPROM_SIZE))
        Serial.println("...");
      Serial.print(i, HEX);
      Serial.print(": ");
      for (byte b : line_data) {
        if (b < 0x10) Serial.print('0');
        Serial.print(b, HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    previous_all_ff = current_all_ff;
  }
  Serial.println();
  Wire.end();
}


void readEEPROM(int address, byte *data, int length) {
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((address >> 8) & 0xFF); // MSB
  Wire.write(address & 0xFF);        // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDRESS, length);
  for (int i = 0; i < length && Wire.available(); i++) {
    data[i] = Wire.read();
  }
}


/************************** SPI Configuration */


void sendSPIinit(const uint8_t PROGMEM *initcmd) {
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TFT_CLOCK, OUTPUT);
  digitalWrite(TFT_CLOCK, HIGH);
  pinMode(TFT_DATA, OUTPUT);
  
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;

    uint8_t args[numArgs];
    for (uint8_t i=0; i<numArgs; i++) {
      args[i] = pgm_read_byte(addr++);
    }
    sendSPIcommand(cmd, args, numArgs);
    if (x & 0x80) {
      uint16_t delaytime = pgm_read_byte(addr++);
      //Serial.print("Delaying for "); Serial.print(delaytime); Serial.println(" ms");
      delay(delaytime);
    }
  }
}

void sendSPIcommand(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes) {
  Serial.print("\tSPI command 0x");
  Serial.print(commandByte, HEX);
  Serial.print(":");
  
  digitalWrite(TFT_CS, LOW);
  SPItransfer(commandByte, false);

  while (numDataBytes--) {
    uint8_t d = dataBytes[0];
    dataBytes++;
    SPItransfer(d, true);
    Serial.print(" 0x");
    Serial.print(d, HEX);
  }
  Serial.println();
  digitalWrite(TFT_CS, HIGH);
}

// software SPI
void SPItransfer(uint8_t data, bool dc) {
  uint16_t send = data;
  
  if (dc) {
    send |= 0x100;
  }

  //Serial.print("(0x"); Serial.print(send, HEX); Serial.print(") ");
  digitalWrite(TFT_CLOCK, HIGH);

  for (int mask = 0x100; mask != 0x00; mask >>= 1) {
    if (send & mask)
      digitalWrite(TFT_DATA, HIGH);
    else 
      digitalWrite(TFT_DATA, LOW);
    digitalWrite(TFT_CLOCK, LOW);
    digitalWrite(TFT_CLOCK, HIGH);
  }
  return;
}

uint8_t SPIread(void) {
  uint8_t reply = 0;
  pinMode(TFT_DATA, INPUT);
  digitalWrite(TFT_CLOCK, HIGH);

  for (int b = 8; b >= 0; b--) {
      digitalWrite(TFT_CLOCK, LOW);
      digitalWrite(TFT_CLOCK, HIGH);
      if (digitalRead(TFT_DATA)) reply |= 1;
      reply <<= 1;
  }
  pinMode(TFT_DATA, OUTPUT);
  return;
}