#define PRODUCT_CODE            5743
#define CONFIG_I2C_PERIPH_ADDR  0x50
//#define CONFIG_UART_DEBUG          1

// Can have up to 3 addresses
#define CONFIG_ADDR_INVERTED
#define CONFIG_ADDR_0_PIN          3
#define CONFIG_ADDR_1_PIN          4

#define CONFIG_INTERRUPT_PIN       7

#define CONFIG_ADC                 1

#include "Adafruit_seesawPeripheral.h"

void setup() {
#if CONFIG_UART_DEBUG
  Serial.begin(115200);
  delay(500);
#endif

  Adafruit_seesawPeripheral_begin();
}


void loop() {
  Adafruit_seesawPeripheral_run();
}
