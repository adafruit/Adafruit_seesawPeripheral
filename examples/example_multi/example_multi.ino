#define PRODUCT_CODE            1234
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#if defined(ARDUINO_GENERIC_C011F6UX)
#define CONFIG_INTERRUPT_PIN      14 // PC14; leave the SWD pins available
#elif defined(ARDUINO_GENERIC_C031G6UX)
#define CONFIG_INTERRUPT_PIN      30 // PC14
#elif defined(ARDUINO_ARCH_STM32)
#define CONFIG_INTERRUPT_PIN      23 // PC14 on C051 and G071
#else
#define CONFIG_INTERRUPT_PIN      12
#endif

// Can have up to 3 addresses
#if defined(ARDUINO_GENERIC_C031G6UX)
#define CONFIG_ADDR_0_PIN          24 // PB8
#define CONFIG_ADDR_1_PIN          22 // PC6
#else
#define CONFIG_ADDR_0_PIN          6
#define CONFIG_ADDR_1_PIN          7
#endif

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

  Adafruit_seesawPeripheral_begin();
}


void loop() {
  Adafruit_seesawPeripheral_run();
}
