#define PRODUCT_CODE            5753
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1
#if defined(ARDUINO_GENERIC_C011F6UX) || defined(ARDUINO_GENERIC_C031G6UX)
#define CONFIG_INTERRUPT_PIN      12 // PA12
#else
#define CONFIG_INTERRUPT_PIN      10
#endif
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
