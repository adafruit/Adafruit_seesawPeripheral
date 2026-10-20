#define PRODUCT_CODE            5740
#define CONFIG_I2C_PERIPH_ADDR  0x49
//#define CONFIG_UART_DEBUG          1

#define CONFIG_INTERRUPT_PIN       6

// Can have up to 4 addresses
#define CONFIG_ADDR_INVERTED
#if defined(ARDUINO_ARCH_STM32)
// Alternate STM32 wiring: PA2..PA5; keep SWD and I2C available.
#define CONFIG_ADDR_0_PIN          2
#define CONFIG_ADDR_1_PIN          3
#define CONFIG_ADDR_2_PIN          4
#define CONFIG_ADDR_3_PIN          5
#else
#define CONFIG_ADDR_0_PIN          10
#define CONFIG_ADDR_1_PIN          11
#define CONFIG_ADDR_2_PIN          12
#define CONFIG_ADDR_3_PIN          13
#endif

//* ============== ENCODER =================== *//
#define CONFIG_ENCODER 1
#define CONFIG_ENCODER_2TICKS 1
#define CONFIG_NUM_ENCODERS 1
#if defined(ARDUINO_ARCH_STM32)
#define CONFIG_ENCODER0_A_PIN 0 // PA0
#define CONFIG_ENCODER0_B_PIN 1 // PA1
#else
#define CONFIG_ENCODER0_A_PIN 16
#define CONFIG_ENCODER0_B_PIN 15
#endif

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
