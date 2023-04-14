#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#define CONFIG_INTERRUPT_PIN       5

// Can have up to 3 addresses
#define CONFIG_ADDR_0_PIN          6
#define CONFIG_ADDR_1_PIN          7

#define CONFIG_UART                1
#define CONFIG_UART_BUF_MAX       32
#define CONFIG_UART_SERCOM    Serial

#include "Adafruit_seesawPeripheral.h"

void setup() {
  Adafruit_seesawPeripheral_begin();
}


void loop() {
  Adafruit_seesawPeripheral_run();
}
