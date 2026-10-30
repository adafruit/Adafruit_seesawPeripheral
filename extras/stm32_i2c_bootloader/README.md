# STM32C011 / STM32C031 I2C bootloading demo

This demo updates a **STM32C011F6 or STM32C031G6 seesaw** over its existing I2C connection.
An Arduino-compatible host bridges USB serial to I2C (classic AVR,
Arduino-Pico RP2040/RP2350, ESP32, or ESP8266);
Python or the standalone Web Serial page handles ST's factory-ROM protocol.
No BOOT0 wire is needed.

The browser detects C011/C031 automatically; the CLI uses `--target`. C011 Rev Z and C031 Rev Z
(device 0x10016453) have passed ROM application programming/readback/Go on hardware.
C031 option writes and physical BOOT0 recovery remain unverified:

| Target | Seesaw ID | ROM address / device ID | SRAM | GET data frame |
| --- | --- | --- | --- | --- |
| C011F6 | 0x90 | 0x64 / 0x0443 | 6 KiB | 20 bytes, 18 commands (observed Rev Z) |
| C031G6 | 0x91 | 0x63 / 0x0453 | 12 KiB | 20 bytes, 18 commands (observed device 0x10016453) |

Both use 32 KiB flash, 2 KiB pages, a 30 KiB application limit and the
128-byte option area at 0x1FFF7800. Device ID and command-layout checks fail
closed before erase/write if the connected ROM differs. C031's exact GET
frame was confirmed on device 0x10016453; option-write behavior still needs verification.
[AN2606 section 6](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
lists C031 ROM V5.2 (distinct from its I2C protocol version), address 0x63,
and external pull-ups on PB6/PB7.
[AN4221](https://www.st.com/resource/en/application_note/an4221-i2c-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)
defines the V1.1 command frame. ST's stm32c031xx.h confirms device ID 0x0453
and the same implemented option-register masks used for C011.
Before C031 firmware operations, its factory flash-size halfword at 0x1FFF75A0
must report 32 KiB; a 16 KiB C031x4 is rejected.

C051, G071 and other STM32 parts are not supported by this uploader.
Running seesaw firmware needs CONFIG_BOOTLOADER and the B0 07 10 AD entry key.
Both frontends also accept targets already in ROM (CLI: --rom).

## Arduino bridge + Web Serial (HEX or BIN)

Open **File > Examples > Adafruit seesaw Peripheral > serial_i2c_bridge** in
the Arduino IDE, or open `examples/serial_i2c_bridge/serial_i2c_bridge.ino` in
this repository. Select the host board in Arduino and upload the sketch; use
Arduino Uno (`arduino:avr:uno`) for Metro 328/Metro Mini. For native-USB ESP32
boards, enable **USB CDC On Boot** when using that USB connector, then close
Serial Monitor. No `while (!Serial)` gate is needed: the bridge processes
commands whenever its USB serial connection is opened.
Use the wiring below. Open [the uploader](https://adafruit.github.io/Adafruit_seesawPeripheral/bootloader/)
in desktop Chrome or Edge. Alternatively, download `webserial.html` and open
that standalone file locally. Ordinary HTTP LAN hosting cannot use Web Serial.

1. Upload the latest bridge sketch and connect its USB serial port on the page.
   The page scans the I2C bus and detects C011 or C031 from its chip ID.
2. In **Enter the bootloader**, select the detected STM32 seesaw and click
   **Enter bootloader**. If a factory-blank or manually booted STM32 is
   already in ROM at its target address, the page identifies it directly and reports ready.
   Keep one target connected during programming or option changes.
3. After detection, drop the matching application HEX or BIN file onto the page. BIN starts at
   `0x08000000`; HEX uses absolute flash addresses and checked record checksums.
   Click **Write** once step 2 reports that ROM is ready.
   **Verify after flash** is checked by default. No backup-file dialog is shown.
4. The page identifies ROM, reads EEPROM/options for comparison, programs only
   required application pages, compares every programmed byte when verification
   is enabled, checks EEPROM/options unchanged, then sends Go. Unchecking
   **Verify after flash** skips application readback, not EEPROM/option checks.
5. Power-cycle the STM32 to confirm the application starts normally.

Files stay on your computer. No Python installation is needed. The page rejects
images over 30720 bytes, bad vectors, overlapping HEX records, and data outside
the application area, including the reserved EEPROM page. Firmware Write never
changes boot options or protection; boot-option changes use the separate controls
below. Keep the tab visible and power stable during programming.
If programming fails, stop and recover with SWD; there is no automatic retry.
**Read** downloads a full 32768-byte BIN backup, including EEPROM, without
requiring a selected firmware file. If the browser blocks the automatic download,
use **Save readout**. This full-flash backup is not an application input for Write.
**Verify** compares the selected firmware and its doubleword padding with flash
without erasing or programming it, regardless of the Verify after flash checkbox.
Read and Verify do not send Go; power-cycle afterward to return to an existing
application. Reconnect the bridge before each operation. Write never creates a
backup automatically; use Read first if you want one. The Python updater below
retains its separate backup workflow.

For subsequent updates, scan and select the running seesaw, then use **Enter
bootloader**. The installed firmware must retain the
seesaw boot-entry command, or another existing ROM-entry method is needed.
Factory-blank auto-entry is not a persistent bootloader-entry mechanism after
the chip is programmed. Write reports whether application readback was verified
and whether Go was acknowledged, not a functional test of the application.
The Python updater's seesaw restart checks below remain separate.

The bridge's `S` command scans addresses `0x08`–`0x77` using address-only probes
and returns `SCAN` followed by responding hexadecimal addresses. The page queries
the seesaw hardware ID at responding addresses except 0x63/0x64.
Both ROM addresses are checked using their device ID and supported command
layout. Never send seesaw register bytes to either ROM parser. Unknown devices
are listed but disabled. Multiple supported devices require a selection.
Changing families clears the firmware file; a pending boot-option check locks
the family until verification or recovery completes.
Scanning does not send a boot-entry key, erase flash, or write options. Boot
entry rechecks the seesaw ID and destination-address occupancy before the key.
If an older bridge sketch rejects `S`, the page asks you to upload the new sketch.
As with any shared I2C bus, duplicate addresses cannot be distinguished; keep
only one device in ROM. A seesaw configured at `0x63` or `0x64` must be moved to another
address before this guided workflow can use it.

### Browser boot-option controls

Use the latest bridge sketch (with the `S` and `O` commands). No firmware
file is needed. Complete device discovery/ROM entry in step 2, then **Read boot
options / Verify pending write** displays nBOOT_SEL, nBOOT1, nBOOT0, BOOT_LOCK,
and RDP. A query leaves the STM32 in ROM.

Choose **Enable BOOT0 recovery** for `0 1 1`, or **Use normal flash boot** for
`1 1 1`. Both open a guided dialog; you do not need to return to the discovery
section. The guide keeps the reset instructions, device selection, option write,
and readback in one place:

1. Briefly touch **STM32 RST to GND**, then release it. Click **I've reset it —
   continue**. The guide scans and, if needed, lets you select a STM32 and boot it
   into ROM. Nothing has been written yet.
2. Click **Enable BOOT0** (or **Set normal flash boot**) to make the change.
3. Follow the dialog's wiring/reset instructions and continue to verify.
4. After the check passes, remove any B0 jumper and reset the STM32 to return to
   the application. Click **Done**.

Before **every** option write, the guide requires this target-reset
acknowledgement. An ordinary **Scan**, or resetting only the bridge, does not
count. The bridge requires RDP level 0 and BOOT_LOCK=0 and preserves all other
defined settings. Firmware Write and the Python updater still never write options.

An acknowledged write is **not verified**. Keep the bridge powered and the page
connected so the bridge retains its expected snapshot. Other programming controls
are disabled while verification is pending. To verify BOOT0 recovery, disconnect
ST-Link from PA14/SWCLK, tie B0 to **3.3 V** (never 5 V), then briefly touch
**STM32 RST to GND** and release it. The dialog shows these instructions at the
right moment. Continuing scans for ROM and reads the options automatically;
require the **Verified** result. This BOOT0 check does not substitute software
boot entry if the physical strap fails. For normal flash boot, the dialog can
boot the detected seesaw into ROM before readback if that firmware is installed.
The bridge stays connected throughout so its pending snapshot is preserved.
The page cannot drive the STM32 reset pin or B0 for you. Closing a pending guide
warns without disconnecting; **Continue boot-option guide** reopens it.

Errors stop further writes until rediscovery and successful option readback (and
pending-write verification, if needed); no write is retried automatically.
A disconnect warning protects pending verification, but resetting the bridge
loses its saved comparison. Reconnecting may reset the host (classic AVR does;
native USB does not always). A plain read
afterward is not verification of the earlier write. See the Serial Monitor
section below for recovery precautions and the exact preservation checks.

The GitHub Pages workflow publishes this page at `/bootloader/` after merging
to `main`; repository Pages settings must use **GitHub Actions** as the source.

For a C031, use `O 63` where the C011 examples below use `O 64`.
The bridge detects a running C031 by ID 0x91 at the supplied seesaw address,
and then uses ROM 0x63. It refuses to switch families while verification of a
saved option write is pending. Connect only one target: identical devices
sharing an address cannot be distinguished.

CLI examples for C031 (use a new backup filename):
```sh
python extras/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --target c031 --rom
python extras/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --target c031 --rom --image seesaw-c031-6540-default.bin --backup c031-before.bin --go
```
Omit `--rom` when starting from bootloader-enabled seesaw at 0x49. Without
`--target`, the CLI remains C011-compatible. Neither frontend infers the family
from a BIN/HEX file; vectors alone cannot prove which MCU an application targets.

## Connections and prerequisites

- Connect host SDA, SCL, and GND to the breakout's **level-shifted QT connector**.
  Classic Nano/Uno/Metro Mini use A4 for SDA and A5 for SCL. Other boards use
  their default `Wire` pins. RP/ESP hosts can override them by defining both
  `BRIDGE_SDA_PIN` and `BRIDGE_SCL_PIN` near the top of the sketch. Choose a valid
  pair for that board's I2C controller; these definitions do not apply to AVR.
- A 5 V host must not be connected directly to the MCU-side 3.3 V bus without
  a suitable bidirectional level shifter. Use the breakout's normal VIN power
  arrangement; never connect 5 V to its 3.3 V output.
- With RP2040/RP2350 or ESP hosts, use **3.3 V I2C pull-ups**. A QT connector
  powered at 5 V can also pull its bus up to 5 V: use the 3.3 V side or a suitable
  level shifter instead. Never connect 5 V pull-ups to a 3.3 V host.
- Keep power stable, close other programs using the host's serial port, and
  keep an ST-Link/SWD recovery connection available.
- Connect only one update target; its ROM address is fixed at 0x64 (C011) or 0x63 (C031).
- For the CLI workflow, install Python 3, Arduino CLI, the host's board core,
  and STM32duino 3.0.0 for compiling these STM32 applications.
  Install this library and its dependencies as described in the main README.

Run the following commands from the repository root. Replace `COM65` with
your bridge's USB serial port; Linux/macOS device paths work too.

```sh
pip3 install -r extras/stm32_i2c_bootloader/requirements.txt
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 examples/serial_i2c_bridge
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328 --port COM65 examples/serial_i2c_bridge
```

Use `arduino:avr:uno` for an Uno or Metro Mini. Uploading replaces the bridge
board's application sketch; save any sketch you need to restore first.
The bridge uses only Wire and Serial, not the seesaw host library.
The legacy Adafruit Metro board package also works, but its Wire library lacks
the I2C timeout API. If its bus stalls, reset the bridge before reconnecting.
Its `.none.test.only` marker excludes it from the peripheral-target matrix.
CI compiles this bridge separately as `arduino:avr:uno` for Metro 328.

### Host portability

The sketch selects its I2C path with preprocessor definitions:

- Classic AVR with `TWCR`/`TWDR`: direct TWI for the 130-byte option packet,
  retaining standard Wire for other commands.
- Arduino-Pico RP2040/RP2350 and ESP32: `WIRE_HAS_BUFFER_SIZE` enables
  `Wire.setBufferSize(130)` before `Wire.begin()`. The entire option packet is
  buffered and sent with one `endTransmission()`. Buffer allocation and ESP32
  bus initialization are checked.
- ESP8266: its core's `twi_writeTo()` sends the complete frame through the same
  software-I2C bus initialized by Wire, bypassing Wire's fixed 128-byte buffer.
- Other Wire cores: scan, firmware, and option reads use the standard API.
  Option writes are refused before ROM entry unless the core advertises at least
  130 transmit bytes. `BRIDGE_WIRE_CAPACITY` may declare a verified buffer size
  when the core does not publish one; it **does not resize the core's buffer**.
  Never set it larger than the actual compiled capacity. A short buffered write
  is never transmitted as a partial option packet.

Timeouts use each supported core's API and units. Generic cores without a known
timeout API, including the legacy Metro package, may require a bridge reset if
the bus stalls. The serial protocol and 32-byte ordinary-packet limit stay the
same on every host. Boot/protection bits are decoded explicitly from serialized
bytes, without depending on the host compiler's bit-field layout or endianness.

## Serial Monitor: query and set boot options

The bridge's `O` command reads the complete 128-byte C011 option area. Its
optional three arguments change **only** `nBOOT_SEL`, `nBOOT1`, and `nBOOT0`.
Protection, watchdog, reset configuration, BOOT_LOCK, and all other defined
settings are preserved. The C011 ROM normalizes reserved bits in its option
registers; their raw bytes can change even when sent unchanged. This is not an
unrestricted option-byte programmer. The browser's separate boot-option controls
use this command; its firmware Write and the Python updater never change options.

Use Serial Monitor at **115200 baud**, with **Newline** or **Both NL & CR**.
Close the browser's serial connection first. All arguments are hexadecimal:

```text
O 49             Query options, entering ROM from seesaw at address 0x49
O 64             Query options when the C011 is already in ROM
O 49 00 01 01    Enable physical BOOT0 selection, starting from running seesaw
O 64 00 01 01    Enable physical BOOT0 selection, starting from ROM
O 64 01 01 01    Select normal flash boot independently of the BOOT0 pin
```

The setter syntax is `O address nBOOT_SEL nBOOT1 nBOOT0`, with each bit `00` or
`01`. For jumper-based recovery use `00 01 01`: BOOT0 low boots the application;
BOOT0 high boots ROM. Other combinations can select ROM unconditionally or SRAM;
do not choose them unless that is intentional. BOOT_LOCK is query-only and must
already be zero. Readout protection must be level 0 (`RDP=AA`). The command checks
the C011 ID, ROM protocol, and each implemented option word's complement before
writing. If the requested values already match, it does not write.

Option programming requires **stable power and a clean reset line**. Keep SWD
recovery available. It is separate from application programming and can leave
the chip needing SWD recovery if interrupted. The bridge reads the existing
options, changes the three requested bits and their complements, and sends the
entire 128-byte snapshot in one ROM no-stretch write. The host-specific paths
above send the complete 130-byte frame in one transaction; normal W/U/R commands
are unchanged. There is no automatic write retry.

**Reset or power-cycle the C011 before every option write**, even if it is still
responding in ROM. Use one option write per ROM session. Readback queries can
use the current session, but do not issue a second write without resetting the
target first. Resetting only the bridge is not a target reset. The bridge cannot
control the C011 reset pin through the I2C wiring.

An ACK means the ROM accepted the write, **not** that readback passed. Keep the
bridge powered and the Serial Monitor open so it retains the expected snapshot.
To verify enabling BOOT0:

1. Disconnect ST-Link from PA14/SWCLK before strapping that shared pin.
2. Tie C011 B0 to **3.3 V**, then power-cycle **only the C011**.
3. Send `O 64`. Require `VERIFIED boot options and all other defined settings; complements valid`
   and `nBOOT_SEL=0 nBOOT1=1 nBOOT0=1 BOOT_LOCK=0 RDP=0xAA`.
4. Remove B0's jumper and reset the C011 to return to the application.

For a setting that boots the application, re-enter ROM with `O 49` instead, if
the installed application supports boot entry. Resetting the bridge loses its
pending comparison; subsequent queries still show the actual bytes but cannot
claim verification against that earlier snapshot. A mismatch blocks further
option writes for that session. Verification checks all of OPTR, every defined
C011 protection field, every implemented word's complement, and the remaining
unimplemented bytes; only reserved register bits may differ. Stop and investigate
any error; do not blindly repeat a write. Query commands leave the C011 in ROM
until it resets or times out.

These commands cannot enter ROM on an already-corrupted application when the
BOOT0 pin is disabled. Configure hardware recovery while the application still
works, while factory-blank ROM entry is available, or using ST-Link.

## 1. Identify the ROM without writing flash

```sh
python extras/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --go
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
arduino-cli compile --fqbn STMicroelectronics:stm32:GenC0:pnum=GENERIC_C011F6UX,xserial=generic,usb=none,opt=oslto,dbg=none,rtlib=nano --library . --build-property upload.maximum_size=30720 --output-dir extras/stm32_i2c_bootloader/build examples/example_adc
```

Use the generated `example_adc.ino.bin`, not an ELF, HEX, or full-flash dump.
The script checks both image length and C011 application vectors before opening
the serial port. The maximum application size is **30720 bytes**.

## 3. Back up, program, verify, and restart

Choose a backup filename that does not already exist:

```sh
python extras/stm32_i2c_bootloader/i2c_bootloader.py --port COM65 --image extras/stm32_i2c_bootloader/build/example_adc.ino.bin --backup extras/stm32_i2c_bootloader/before-update.bin --go
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
