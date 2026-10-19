# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## STM32C011 core support

Select STM32duino's generic **STM32C011F6Ux** target and start with
`examples/example_stm32c011`. This backend supports GPIO direction, bulk
read/write/toggle, pull-up/pull-down selection, optional active-low GPIO change
interrupts, and 12-bit ADC reads. PWM, NeoPixel, UART, encoder, EEPROM, FHT, SPI,
and ROM-bootloader features are not included in this core port.

I2C uses PB7 (SDA) and PB6 (SCL), with address `0x49` in the example. The target
requires a 48 MHz peripheral clock and retains hardware clock stretching as a
safety fallback. Call `Adafruit_seesawPeripheral_run()` regularly to process
queued commands and sample GPIO changes. ADC replies contain the last completed
conversion; allow conversion time after selecting a channel. ADC status is zero
on success and one for an invalid channel or a conversion failure.

Protocol GPIO indices 0–8 map to PA0–PA8, 9/10 to PA11/PA12, and 15 to PC14.
Only indices 0–10 support ADC. SWD, I2C, the PC15 LED, and configured address or
interrupt pins are excluded from ordinary GPIO/ADC access. Enable
`CONFIG_INTERRUPT_PIN` only on an exposed GPIO pin; its output is open-drain.

