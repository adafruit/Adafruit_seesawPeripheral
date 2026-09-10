# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## Experimental STM32C011 backend

STM32duino support is isolated from the existing megaTinyCore implementation.
Start with `examples/example_stm32c011`. The engineering firmware and measured
feature/limitation matrix are in [extras/hwtest](extras/hwtest/README.md), including
the optional [I2C-to-SPI controller bridge](extras/hwtest/SPI_PROTOCOL.md).
Hardware ID `0x90` is provisional and needs the companion Arduino or
CircuitPython host-library changes. This is not yet a drop-in ATtiny replacement.

