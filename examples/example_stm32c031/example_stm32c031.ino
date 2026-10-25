// Adafruit STM32C031G6U6 seesaw breakout: GPIO, ADC, PWM, NeoPixel, encoder.
// Select STM32duino's generic STM32C031G6Ux target.
#define PRODUCT_CODE 6540
#define CONFIG_I2C_PERIPH_ADDR 0x49
#define CONFIG_ADC 1
#define CONFIG_PWM 1
#define CONFIG_NEOPIXEL 1
#define CONFIG_EEPROM 1
#define CONFIG_BOOTLOADER 1
#define CONFIG_ENCODER 1
#define CONFIG_NUM_ENCODERS 1
#define CONFIG_ENCODER0_A_PIN 2 // PA2
#define CONFIG_ENCODER0_B_PIN 3 // PA3
#define CONFIG_INTERRUPT_PIN 30 // PC14, active-low open-drain output.
#define CONFIG_ADDR_0_PIN 24    // PB8: ground for address +1.
#define CONFIG_ADDR_1_PIN 22    // PC6: ground for address +2.

// I2C is PB7/SDA and PB6/SCL. SWD and reset stay reserved.
// D0..8 = PA0..8; D11/12/15 = PA11/12/15; D16/17 = PB0/1;
// D19..21 = PB3..5; D22 = PC6; D24 = PB8; D30 = PC14; D31 = PC15 LED.
// ADC: D0..8, D11/12, D16/17. PWM: all usable GPIOs except D12.
// INT and address pins are reserved. Floating straps select 0x49;
// ground ADDR0, ADDR1, or both for 0x4A, 0x4B, or 0x4C after reset.
// Some PWM pins share timer channels; see the README before combining them.
// GPIO, ADC, PWM, NeoPixel, EEPROM and one encoder are enabled by default.
// Encoder 0 uses PA2/PA3 with pull-ups. The final 2 KiB page is reserved for
// EEPROM. SPI/UART/spectrum remain separate configurations on this 32 KiB MCU.
#include "Adafruit_seesawPeripheral.h"

void setup() {
  if (!Adafruit_seesawPeripheral_begin()) {
    while (true) {
      delay(10);
    }
  }
}

void loop() { Adafruit_seesawPeripheral_run(); }
