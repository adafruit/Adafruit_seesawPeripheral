# STM32C011 I2C bootloading demo

This demo updates a **STM32C011F6 seesaw** over its existing I2C connection.
A classic Arduino Nano, Uno, or Adafruit Metro Mini bridges USB serial to I2C;
Python handles ST's factory-ROM protocol. No BOOT0 wire is needed.

This is **C011-specific**, including its Rev Z ROM command-frame layout.
Do not use it for C031, C051, G071, or other STM32 parts: their ROM addresses,
flash sizes, and command behavior differ. The target must already run seesaw
firmware with `CONFIG_BOOTLOADER` enabled and the `B0 07 10 AD` entry key.

## Connections and prerequisites

- Connect host SDA, SCL, and GND to the breakout's **level-shifted QT connector**.
  Classic Nano/Uno/Metro Mini use A4 for SDA and A5 for SCL.
- A 5 V host must not be connected directly to the MCU-side 3.3 V bus without
  a suitable bidirectional level shifter. Use the breakout's normal VIN power
  arrangement; never connect 5 V to its 3.3 V output.
- Keep power stable, close other programs using the host's serial port, and
  keep an ST-Link/SWD recovery connection available.
- Connect only one update target; its factory-ROM I2C address is fixed at `0x64`.
- Install Python 3, Arduino CLI, the Arduino AVR core, and STM32duino 3.0.0.
  Install this library and its dependencies as described in the main README.

Run the following commands from the repository root. Replace `COM65` with
your bridge's USB serial port; Linux/macOS device paths work too.

```sh
pip3 install -r tools/stm32_i2c_bootloader/requirements.txt
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 tools/stm32_i2c_bootloader/serial_i2c_bridge
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328 --port COM65 tools/stm32_i2c_bootloader/serial_i2c_bridge
```

Use `arduino:avr:uno` for an Uno or Metro Mini. Uploading replaces the bridge
board's application sketch; save any sketch you need to restore first.
The bridge uses only Wire and Serial, not the seesaw host library.

## 1. Identify the ROM without writing flash

```sh
python tools/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --go
```

This checks seesaw hardware ID `0x90`, sends `00 7E B0 07 10 AD` to its current
address (default `0x49`), and identifies the C011 ROM at address `0x64`.
It reads the application vectors, returns to seesaw, and checks a software reset.
No flash erase or programming command is sent.

Use `--address 0xNN` when the running seesaw has another address. The new
firmware must also return at that address for the final restart check to pass.

## 2. Build an application that fits below EEPROM

The existing ADC example is a useful payload: it changes a spectrum-only
seesaw into a normal GPIO/ADC seesaw while keeping bootloader entry available.
Its address straps must remain unconnected for the default `0x49` address.

```sh
arduino-cli compile --fqbn STMicroelectronics:stm32:GenC0:pnum=GENERIC_C011F6UX,xserial=generic,usb=none,opt=oslto,dbg=none,rtlib=nano --library . --build-property upload.maximum_size=30720 --output-dir tools/stm32_i2c_bootloader/build examples/example_adc
```

Use the generated `example_adc.ino.bin`, not an ELF, HEX, or full-flash dump.
The script checks both image length and C011 application vectors before opening
the serial port. The maximum application size is **30720 bytes**.

## 3. Back up, program, verify, and restart

Choose a backup filename that does not already exist:

```sh
python tools/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --image tools/stm32_i2c_bootloader/build/example_adc.ino.bin --backup tools/stm32_i2c_bootloader/before-update.bin --go
```

The updater:

1. Reads and saves all 32768 flash bytes before any erase.
2. Erases only the application pages needed by the image, never EEPROM page 15.
3. Programs aligned doublewords using bounded I2C packets.
4. Reads back and compares the complete programmed image.
5. Verifies that all 2048 EEPROM bytes and all 128 option-area bytes are unchanged.
6. Uses ROM Go, checks the seesaw ID, and checks seesaw software reset.

The full-flash backup includes EEPROM and is deliberately rejected as an
application input. Keep it for recovery; do not commit device backups to Git.
The tool never writes option bytes, performs a mass erase, or automatically
retries failed programming. It is an updater, not a general ROM recovery tool.

If programming or verification fails, **stop and recover with SWD**. The C011
ROM can time out and reset into a partially written application, so omitting
`--go` does not guarantee it will remain in ROM. The boot-entry key prevents
accidental entry; it is not authentication or signed-firmware protection.

## Protocol references

- [ST AN4221: I2C bootloader protocol](https://www.st.com/resource/en/application_note/an4221-i2c-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)
- [ST AN2606: system-memory boot mode](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
- [STM32C0 reference manual RM0490](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

The C011 Rev Z ROM reports protocol `0x11` but returns 18 commands in its Get
response. This demo reads that complete 20-byte response in one transaction.
