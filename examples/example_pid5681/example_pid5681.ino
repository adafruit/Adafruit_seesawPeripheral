#define PRODUCT_CODE            5681
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG         1

#define CONFIG_INTERRUPT_PIN        6
//#define USE_PINCHANGE_INTERRUPT   1

// Can have up to 3 addresses
#define CONFIG_ADDR_0_PIN          12
#define CONFIG_ADDR_1_PIN          13

#define CONFIG_ADC                 1
#define CONFIG_PWM                 1
#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX   (60*3) // 60 pixels == 180 bytes

#include "Adafruit_seesawPeripheral.h"

void setup() {
#if CONFIG_UART_DEBUG
  Serial.begin(115200);
  delay(500);
#endif
  delay(1);
  Adafruit_seesawPeripheral_begin();
}


void loop() {
  Adafruit_seesawPeripheral_run();
}