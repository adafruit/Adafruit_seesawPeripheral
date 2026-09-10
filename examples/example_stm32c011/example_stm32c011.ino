#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_EEPROM 0

#define CONFIG_I2C_SDA_PIN PB7
#define CONFIG_I2C_SCL_PIN PB6
#define CONFIG_STATUS_LED_PIN PC15
// The GPIO mask uses digital indices; PA13/PA14 are analog aliases in this
// core.
#define CONFIG_SWDIO_PIN 11 // PA13
#define CONFIG_SWCLK_PIN 12 // PA14

#include "Adafruit_seesawPeripheral.h"

void setup() {
  pinMode(CONFIG_STATUS_LED_PIN, OUTPUT);
  digitalWrite(CONFIG_STATUS_LED_PIN, HIGH);

  Adafruit_seesawPeripheral_begin();
}

void loop() { Adafruit_seesawPeripheral_run(); }
