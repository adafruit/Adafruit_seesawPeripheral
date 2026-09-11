# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## Experimental STM32C011 backend

STM32duino and megaTinyCore share protocol configuration, state, command and
response decoding, byte encoding, address-strap logic, and the encoder decoder.
Target-specific code handles GPIO, ADC, timers, LED timing, EEPROM storage, and
I2C execution (deferred on STM32, callback-based on AVR). Both use the register
definitions from `Adafruit_seesaw.h`; no host seesaw or BusIO object is created.
For STM32 builds with core Serial disabled, pass `-DNO_GLOBAL_SERIAL` globally,
as shown in `extras/hwtest/build.ps1`; BusIO already supports this guard.
STM32duino's global `SPI` object is still linked through that dependency, so
non-SPI builds have a measurable size cost despite not using host BusIO objects.
Start with `examples/example_stm32c011`. The engineering firmware and measured
feature/limitation matrix are in [extras/hwtest](extras/hwtest/README.md), including
the optional [I2C-to-SPI controller bridge](extras/hwtest/SPI_PROTOCOL.md).
Hardware ID `0x90` is provisional and needs the companion Arduino or
CircuitPython host-library changes. This is not yet a drop-in ATtiny replacement.

