// Adafruit STM32G071GBU6 seesaw breakout with all supported bridge features.
// Select STM32duino's generic STM32G071GBUx target (not the UxN pinout).
#define PRODUCT_CODE 0
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_NEOPIXEL 1
#define CONFIG_ENCODER 1
#define CONFIG_NUM_ENCODERS 1
#define CONFIG_ENCODER0_A_PIN 2 // PA2
#define CONFIG_ENCODER0_B_PIN 3 // PA3
#define CONFIG_INTERRUPT_PIN 23 // PC14, active-low open-drain output.
#define CONFIG_UART 1 // PA0/TX and PA1/RX, USART4, 3.3 V logic.
#define CONFIG_UART_BUF_MAX 32
#define CONFIG_SPI 1 // PA4/CS, PA5/SCK, PA6/MISO, PA7/MOSI, 3.3 V logic.
#define CONFIG_EEPROM 1

// Reserve the final 2 KiB flash page: application maximum 129024 bytes.
// EEPROM byte 255 holds the persistent I2C address; erased bytes are 0xFF.
// I2C is PB7/SDA and PB6/SCL. SWD, PC15 LED, and reset stay reserved.
// GPIO numbering matches C031/C051, but ADC is D0..7 and D14/15 only.
// PWM is available on exposed GPIOs except D10/PA12 and D23/PC14.
// Shared PWM groups: D0/5/13, D6/17, and D7/18; use one pin per group.
// PA4/PA5 provide PWM here, not DAC. The DAC has no seesaw protocol yet.
// UART pins are reserved at startup; SPI pins are reserved when configured.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
