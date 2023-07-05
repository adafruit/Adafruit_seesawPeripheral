#define PRODUCT_CODE            5752
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1
#define CONFIG_INTERRUPT_PIN      3
//#define USE_PINCHANGE_INTERRUPT    1

// Can have up to 3 addresses
#define CONFIG_ADDR_INVERTED
#define CONFIG_ADDR_0_PIN           2
#define CONFIG_ADDR_1_PIN           1
#define CONFIG_ADDR_2_PIN           20

#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX   (30*3) // 30 pixels == 180 bytes

//* ============== ENCODER =================== *//
#define CONFIG_ENCODER 1
#define CONFIG_NUM_ENCODERS 4
#define CONFIG_ENCODER0_A_PIN 13
#define CONFIG_ENCODER0_B_PIN 16
#define CONFIG_ENCODER1_A_PIN 15
#define CONFIG_ENCODER1_B_PIN 19
#define CONFIG_ENCODER2_A_PIN 7
#define CONFIG_ENCODER2_B_PIN 6
#define CONFIG_ENCODER3_A_PIN 4
#define CONFIG_ENCODER3_B_PIN 5

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