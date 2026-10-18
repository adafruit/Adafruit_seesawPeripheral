# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## Experimental STM32C011 backend

STM32duino and megaTinyCore share protocol configuration, state, command and
response decoding, byte encoding, address-strap logic, and the encoder decoder.
Target-specific code handles GPIO, ADC, timers, LED timing, EEPROM storage, and
I2C execution (deferred on STM32, callback-based on AVR). Both use the register
definitions from `Adafruit_seesaw.h`; no host seesaw or BusIO object is created.
For STM32 builds with core Serial disabled, pass `-DNO_GLOBAL_SERIAL` globally;
BusIO already supports this guard.
STM32duino's global `SPI` object is still linked through that dependency, so
non-SPI builds have a measurable size cost despite not using host BusIO objects.
Start with `examples/example_stm32c011`.
Hardware ID `0x90` is provisional and needs the companion Arduino or
CircuitPython host-library changes. This is not yet a drop-in ATtiny replacement.

The C011 uses a dedicated I2C1 interrupt transport for 100-400 kHz operation.
It keeps hardware clock stretching available as a safety fallback, especially
during flash-backed EEPROM writes and interrupt-masked NeoPixel output.
`CONFIG_I2C_NO_STRETCH=1` is an experimental bench override, not a safe general
default: delayed interrupt service can corrupt transfers in that mode.

### STM32 factory-ROM bootloader

The C011 backend enables `CONFIG_BOOTLOADER` by default. Write the three bytes
`00 7E FF` to the current seesaw I2C address to reset into ST's factory ROM.
This is separate from the normal seesaw software-reset command, `00 7F FF`.
Define `CONFIG_BOOTLOADER 0` before including the library to omit ROM entry.

The ROM speaks ST's [AN4221 I2C protocol](https://www.st.com/resource/en/application_note/an4221-i2c-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)
at 7-bit address `0x64`, using PB6/SCL and PB7/SDA. It does not speak seesaw
commands. Use pull-ups to the C011's 3.3 V supply: ROM enables internal pulls,
so the application-mode exception for a 5 V bus does not apply. No BOOT0 wire
or option-byte change is needed for software entry.
With a 5 V host, use a bidirectional level shifter. On the Rev A breakout,
the QT connectors are shifted; the `SDA_3V`/`SCL_3V` headers are not.

When EEPROM emulation is enabled, reserve the last 2 KiB flash page at
`0x08007800`, limit the application to 30720 bytes, and exclude that page from
bootloader erases. Use ROM's Go command with address `0x08000000` to restart
the application after verifying an update.

