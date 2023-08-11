#define PRODUCT_CODE            5753
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1
#define CONFIG_INTERRUPT_PIN      10
//#define USE_PINCHANGE_INTERRUPT    1


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
