#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x42
//#define CONFIG_UART_DEBUG         1

#define CONFIG_INTERRUPT_PIN       13
//#define USE_PINCHANGE_INTERRUPT   1

// Can have up to 8 addresses
#define CONFIG_ADDR_0_PIN          10
#define CONFIG_ADDR_1_PIN          11
#define CONFIG_ADDR_2_PIN          12

//#define CONFIG_ADC                 1
//#define CONFIG_PWM                 1
#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX   (1*3) // 1 pixel == 3 bytes

//* ============== SOIL =================== *//
#define CONFIG_SOIL                  1
#define CONFIG_NUM_SOIL              1
#define CONFIG_SOIL0_EXC_PIN         0
#define CONFIG_SOIL0_SEN_PIN         1
// #define CONFIG_SOIL1_EXC_PIN         2
// #define CONFIG_SOIL1_SEN_PIN         3
// #define CONFIG_SOIL2_EXC_PIN         4
// #define CONFIG_SOIL2_SEN_PIN         5
// #define CONFIG_SOIL3_EXC_PIN         6
// #define CONFIG_SOIL3_SEN_PIN         7

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
