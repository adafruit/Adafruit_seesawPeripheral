# STM32C011 I2C bootloading demo

This demo updates a **STM32C011F6 seesaw** over its existing I2C connection.
A classic Arduino Nano, Uno, or Adafruit Metro Mini bridges USB serial to I2C;
Python or the standalone Web Serial page handles ST's factory-ROM protocol.
No BOOT0 wire is needed.

This is **C011-specific**, including its Rev Z ROM command-frame layout.
Do not use it for C031, C051, G071, or other STM32 parts: their ROM addresses,
flash sizes, and command behavior differ. The Python updater requires seesaw
firmware with `CONFIG_BOOTLOADER` enabled and the `B0 07 10 AD` entry key.
The browser uploader also supports factory-blank chips already in ROM.

## Metro 328 + Web Serial (HEX or BIN)

Open **File > Examples > Adafruit seesaw Peripheral > serial_i2c_bridge** in
the Arduino IDE, or open `examples/serial_i2c_bridge/serial_i2c_bridge.ino` in
this repository. Upload it to the Metro 328 using the Arduino Uno board
selection (`arduino:avr:uno`), then close Serial Monitor.
Use the wiring below. Open [the uploader](https://adafruit.github.io/Adafruit_seesawPeripheral/bootloader/)
in desktop Chrome or Edge. Alternatively, download `webserial.html` and open
that standalone file locally. Ordinary HTTP LAN hosting cannot use Web Serial.

1. Upload the latest Metro bridge sketch, then connect its USB serial port on
   the page. Connecting automatically scans the I2C bus.
2. In **Find your C011 and enter ROM**, select the detected C011 seesaw and click
   **Boot selected C011 into ROM**. If a factory-blank or manually booted C011 is
   already in ROM at `0x64`, the page identifies it directly and reports ready.
   With multiple C011 seesaws, choose which one to boot; only one may be in ROM.
3. Drop a C011 application HEX or BIN file onto the page. BIN starts at
   `0x08000000`; HEX uses absolute flash addresses and checked record checksums.
   Click **Write** once step 2 reports that ROM is ready.
   **Verify after flash** is checked by default. No backup-file dialog is shown.
4. The page identifies ROM, reads EEPROM/options for comparison, programs only
   required application pages, compares every programmed byte when verification
   is enabled, checks EEPROM/options unchanged, then sends Go. Unchecking
   **Verify after flash** skips application readback, not EEPROM/option checks.
5. Power-cycle the C011 to confirm the application starts normally.

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
application. Reconnect the Metro before each operation. Write never creates a
backup automatically; use Read first if you want one. The Python updater below
retains its separate backup workflow.

For subsequent updates, scan and select the running seesaw, then use **Boot
selected C011 into ROM**. The installed firmware must retain the
seesaw boot-entry command, or another existing ROM-entry method is needed.
Factory-blank auto-entry is not a persistent bootloader-entry mechanism after
the chip is programmed. Write reports whether application readback was verified
and whether Go was acknowledged, not a functional test of the application.
The Python updater's seesaw restart checks below remain separate.

The bridge's `S` command scans addresses `0x08`–`0x77` using address-only probes
and returns `SCAN` followed by responding hexadecimal addresses. The page queries
the seesaw hardware ID at each responding address except `0x64`; only ID `0x90`
is selectable as C011 seesaw. At `0x64` it instead checks the C011 ROM product ID
and supported command layout, never sending seesaw register bytes to ROM.
Unknown devices are listed but disabled. Scanning does not send a boot-entry key,
erase flash, or write options. Boot entry is a separate explicit action and
rechecks the selected seesaw and occupancy of `0x64` before sending the key.
If an older Metro sketch rejects `S`, the page asks you to upload the new sketch.
As with any shared I2C bus, duplicate addresses cannot be distinguished; keep
only one device in ROM. A seesaw configured at `0x64` must be moved to another
address before this guided workflow can use it.

### Browser boot-option controls

Use the latest Metro bridge sketch (with the `S` and `O` commands). No firmware
file is needed. Complete device discovery/ROM entry in step 2, then **Read boot
options / Verify pending write** displays nBOOT_SEL, nBOOT1, nBOOT0, BOOT_LOCK,
and RDP. A query leaves the C011 in ROM.

Before **every** option write, reset or power-cycle only the C011, then click
**I reset the C011 — scan again** in step 2. If seesaw is found, select it and
boot it into ROM. This reset acknowledgement unlocks one option write; an ordinary
scan does not. Resetting the Metro alone does not count. Choose **Enable BOOT0
recovery** for `0 1 1`, or **Use normal flash
boot** for `1 1 1`. The bridge requires RDP level 0 and BOOT_LOCK=0 and preserves
all other defined settings. Firmware Write and the Python updater still never
write options.

An acknowledged write is **not verified**. Keep the Metro powered and the page
connected so the bridge retains its expected snapshot. Other programming controls
are disabled while verification is pending. To verify BOOT0 recovery, disconnect
ST-Link from PA14/SWCLK, tie B0 to **3.3 V**, reset/power-cycle only the C011, and
use **I reset the C011 — scan again**. Once ROM is found, click **Read boot options
/ Verify pending write**. Require the **Verified** result. Remove the B0 jumper
and reset afterward to run the application. For normal flash boot, reset and scan,
then boot the detected seesaw into ROM if that firmware is installed. The Metro
stays connected throughout discovery/entry so its pending snapshot is preserved.
The page cannot drive the C011 reset pin or B0 for you.

Errors stop further writes until rediscovery and successful option readback (and
pending-write verification, if needed); no write is retried automatically.
A disconnect warning protects pending verification, but
resetting the Metro or reconnecting loses its saved comparison. A plain read
afterward is not verification of the earlier write. See the Serial Monitor
section below for recovery precautions and the exact preservation checks.

The GitHub Pages workflow publishes this page at `/bootloader/` after merging
to `main`; repository Pages settings must use **GitHub Actions** as the source.

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
pip3 install -r extras/stm32_i2c_bootloader/requirements.txt
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 examples/serial_i2c_bridge
arduino-cli upload --fqbn arduino:avr:nano:cpu=atmega328 --port COM65 examples/serial_i2c_bridge
```

Use `arduino:avr:uno` for an Uno or Metro Mini. Uploading replaces the bridge
board's application sketch; save any sketch you need to restore first.
The bridge uses only Wire and Serial, not the seesaw host library.
The legacy Adafruit Metro board package also works, but its Wire library lacks
the I2C timeout API. If its bus stalls, reset the Metro before reconnecting.
Its `.uno.test.only` marker selects the AVR host rather than the peripheral
targets. CI compiles this bridge separately as `arduino:avr:uno` for Metro 328.

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
entire 128-byte snapshot in one ROM no-stretch write. The ATmega328P's TWI
hardware sends this one long packet because the standard Wire buffer is only
32 bytes; normal W/U/R commands are unchanged. There is no automatic write retry.

**Reset or power-cycle the C011 before every option write**, even if it is still
responding in ROM. Use one option write per ROM session. Readback queries can
use the current session, but do not issue a second write without resetting the
target first. Resetting only the Metro is not a target reset. The bridge cannot
control the C011 reset pin through the I2C wiring.

An ACK means the ROM accepted the write, **not** that readback passed. Keep the
Metro powered and the Serial Monitor open so it retains the expected snapshot.
To verify enabling BOOT0:

1. Disconnect ST-Link from PA14/SWCLK before strapping that shared pin.
2. Tie C011 B0 to **3.3 V**, then power-cycle **only the C011**.
3. Send `O 64`. Require `VERIFIED boot options and all other defined settings; complements valid`
   and `nBOOT_SEL=0 nBOOT1=1 nBOOT0=1 BOOT_LOCK=0 RDP=0xAA`.
4. Remove B0's jumper and reset the C011 to return to the application.

For a setting that boots the application, re-enter ROM with `O 49` instead, if
the installed application supports boot entry. Resetting the Metro loses its
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
