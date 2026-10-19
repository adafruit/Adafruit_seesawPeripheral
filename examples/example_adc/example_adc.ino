#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#if defined(ARDUINO_GENERIC_C011F6UX)
#define CONFIG_INTERRUPT_PIN      15 // PC14; leave the SWD pins available
#else
#define CONFIG_INTERRUPT_PIN      12
#endif

// Can have up to 3 addresses
#define CONFIG_ADDR_0_PIN          6
#define CONFIG_ADDR_1_PIN          7

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
