#define PRODUCT_CODE            5742 // the original PID
#define CONFIG_I2C_PERIPH_ADDR  0x60
//#define CONFIG_UART_DEBUG          1

// Can have up to 3 addresses
#define CONFIG_ADDR_INVERTED
#define CONFIG_ADDR_0_PIN          1
#define CONFIG_ADDR_1_PIN          2
#define CONFIG_ADDR_2_PIN          3

#define CONFIG_ACTIVITY_PIN        6

#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX    (512*3) // 512 pixels == 1.5K bytes
#define CONFIG_NEOPIXEL_ACT_LED    14 

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
