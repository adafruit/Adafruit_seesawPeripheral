# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## STM32C011 core support

Select STM32duino's generic **STM32C011F6Ux** target and start with
`examples/example_stm32c011`. This backend supports GPIO direction, bulk
read/write/toggle, pull-up/pull-down selection, optional active-low GPIO change
interrupts, 12-bit ADC reads, PWM, NeoPixel, encoders, and UART. EEPROM, FHT, SPI,
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

Enable `CONFIG_PWM` (or `CONFIG_PWM_16BIT`) for PWM on indices 0–9 and 15.
Both settings accept 16-bit duty values, from 0 through 65535. Frequency is
shared across active PWM outputs, defaults to 1000 Hz, and accepts 1–65535 Hz;
zero is rejected. Timer resolution limits the available duty steps at high
frequencies. D3/D9 share TIM1 channel 4, and D7/D15 share TIM3 channel 2: use
only one pin from each pair at a time. GPIO configuration or an ADC read
releases that pin from PWM; a software reset releases all PWM outputs.

Enable `CONFIG_NEOPIXEL` for 800 kHz output through Adafruit_NeoPixel on any
valid GPIO. Set the output pin and byte length before sending SHOW. The default
`CONFIG_NEOPIXEL_BUF_MAX` is 192 bytes (64 RGB or 48 RGBW pixels); hosts provide
the bytes in the strip's color order. Lengths are clamped to that capacity, and
buffer writes preserve only the portion that fits. Storage is allocated once;
`begin()` fails if allocation fails. Software reset clears the buffer and pin
selection. SHOW releases its selected pin from PWM and temporarily masks
interrupts while sending data, delaying I2C service and GPIO-change polling.

Enable `CONFIG_ENCODER` and set `CONFIG_NUM_ENCODERS` from 1 through 4, with
`CONFIG_ENCODER0_A_PIN` / `CONFIG_ENCODER0_B_PIN` and corresponding pairs for
additional encoders. Each input must be a distinct valid GPIO. Inputs start with
pull-ups; connect a mechanical encoder's common terminal to ground. Keep these
pins configured as inputs while using the encoder.

Position and delta are signed 32-bit values. Reading either clears that
encoder's delta. With `CONFIG_INTERRUPT_PIN` configured, enable interrupts per
encoder to assert the shared active-low output while its delta is nonzero;
reading one encoder does not clear another encoder's pending interrupt. Counting
also works without an interrupt pin. `CONFIG_ENCODER_2TICKS` selects the shared
decoder's two-tick mode. Encoders are polled by `run()`, so long NeoPixel shows
or slow application code can miss transitions. Reset clears counts and interrupt
enables and samples each input's current phase.

`examples/example_uart` enables the UART bridge on PA0 (TX) and PA1 (RX),
reserved from GPIO/ADC/PWM while enabled. Use 3.3 V logic on these raw pins.
`CONFIG_UART_RX_PIN`, `CONFIG_UART_TX_PIN`, and `CONFIG_UART_SERCOM` may select
another valid hardware-serial mapping. Keep Serial debug output disabled when
using the bridge. Reset selects 9600 baud and clears buffered data and interrupt
enables; a zero baud command is ignored. RX-ready status is live, and an enabled
RX interrupt stays asserted until the receive buffer is empty. Each I2C DATA
read consumes one byte (0xFF when empty); writes can carry up to 30 bytes with
the default 32-byte command buffer. Long interrupt-masked NeoPixel shows can
lose incoming serial bytes. Check flash usage when combining features.
