#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#if defined(ARDUINO_GENERIC_C011F6UX)
#define CONFIG_INTERRUPT_PIN      14 // PC14
#else
#define CONFIG_INTERRUPT_PIN      15
#endif
//#define USE_PINCHANGE_INTERRUPT    1

// Can have up to 3 addresses
#if defined(ARDUINO_GENERIC_C011F6UX) || defined(ARDUINO_AVR_ATtiny806) || defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny1616)
// STM32 PA6/PA7, or the 20-pin tinyAVR board's D6/D7.
#define CONFIG_ADDR_0_PIN          6
#define CONFIG_ADDR_1_PIN          7
#else
#define CONFIG_ADDR_0_PIN          16
#define CONFIG_ADDR_1_PIN          17
#endif

#define CONFIG_ADC                 1
#define CONFIG_PWM                 1
#define CONFIG_NEOPIXEL            1
#define CONFIG_NEOPIXEL_BUF_MAX   (60*3) // 60 pixels == 180 bytes

//* ============== ENCODER =================== *//
#define CONFIG_ENCODER 1
#define CONFIG_NUM_ENCODERS 1
#define CONFIG_ENCODER0_A_PIN 0
#define CONFIG_ENCODER0_B_PIN 1
//#define CONFIG_ENCODER1_A_PIN 2
//#define CONFIG_ENCODER1_B_PIN 3
//#define CONFIG_ENCODER2_A_PIN 5
//#define CONFIG_ENCODER2_B_PIN 6
//#define CONFIG_ENCODER3_A_PIN 7
//#define CONFIG_ENCODER3_B_PIN 8

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
