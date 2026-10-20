// Firmware that is programmed into PID 5233 ATtiny817 breakout

#define PRODUCT_CODE            5233
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#define CONFIG_INTERRUPT_PIN      15
//#define USE_PINCHANGE_INTERRUPT    1

// Can have up to 3 addresses
#if defined(ARDUINO_GENERIC_C011F6UX) || defined(ARDUINO_AVR_ATtiny806) || defined(ARDUINO_AVR_ATtiny816) || defined(ARDUINO_AVR_ATtiny1616)
// Alternate wiring for STM32C011 and 20-pin tinyAVR targets.
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
