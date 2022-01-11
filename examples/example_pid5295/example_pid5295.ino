// Firmware that is programmed into PID 5295 ATtiny817 Slide Potentiometer

#define PRODUCT_CODE            5295
#define CONFIG_I2C_PERIPH_ADDR  0x30
//#define CONFIG_UART_DEBUG          1

#define CONFIG_INTERRUPT_PIN      9  // PB02

#define CONFIG_ADDR_0_PIN         1  // PA05
#define CONFIG_ADDR_1_PIN         3  // PA07
#define CONFIG_ADDR_2_PIN         5  // PB06
#define CONFIG_ADDR_3_PIN         7  // PB04
#define CONFIG_ADDR_INVERTED

#define CONFIG_ADC                 1
#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX   (4*3) // 4 built in pixels!

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
