// Firmware that is programmed into PID 5296 ATtiny817 Arcade QT

#define PRODUCT_CODE            5296
#define CONFIG_I2C_PERIPH_ADDR  0x3A
//#define CONFIG_UART_DEBUG          1

#define CONFIG_INTERRUPT_PIN      9  // PB02

#define CONFIG_ADDR_INVERTED
#define CONFIG_ADDR_0_PIN         3  // PA07
#define CONFIG_ADDR_1_PIN         4  // PB07
#define CONFIG_ADDR_2_PIN         5  // PB06
#define CONFIG_ADDR_3_PIN         6  // PB05

#define CONFIG_PWM                 1

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
