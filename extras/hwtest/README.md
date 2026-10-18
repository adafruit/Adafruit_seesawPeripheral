# STM32C011 engineering bring-up

These are engineering tests, not production fixture firmware.

- `c011_firmware/`: STM32C011F6Ux peripheral firmware. Full configuration enables
  GPIO, 12-bit ADC, 16-bit PWM commands, open-drain GPIO interrupt,
  one quadrature encoder, 192-byte NeoPixel buffer, and flash-backed EEPROM.
  The UART bridge is off by default; use the `uart` build target to enable it.
  Core Serial support is disabled except in the UART build. Both targets use
  the register definitions in `Adafruit_seesaw.h`, without instantiating a host
  seesaw or BusIO object. Serial-disabled builds use BusIO's existing global
  `NO_GLOBAL_SERIAL` guard. STM32duino's global `SPI` object is still linked by
  that dependency, including in builds where the seesaw SPI module is disabled.
- `metro_host/`: Metro Mini / Uno serial-to-seesaw test host. Its GPIO remain
  inputs except D4-D11, which only pull low or release to emulate encoders.
  It never enables 5 V pull-ups or drives those pins high.
- `bench.py`: independent Jumperless voltage/digital measurements and Metro
  encoder stimuli. Each test reports its actual assertions.
- `circuitpython_host.py`: the real CircuitPython seesaw library on CPython,
  using a Metro adapter for physical I2C rather than mocked register responses.
- `build.ps1`: reproducible full, GPIO-only, four-encoder, address-strap,
  opt-in UART/SPI, SPI queue-fault injection, Metro, and AVR-example compilation targets. Supply an output
  directory outside this repository.
- `cold_cycle.py`: human-gated power-off measurement, EEPROM retention check,
  and restoration from the original full-flash backup. Never switches a rail.
- `openocd_command.py`: explicit commands to an already-running local OpenOCD
  server. It does not launch a debugger or change option bytes automatically.
## Fast C011 I2C transport

The dedicated I2C1 handler bypasses Wire/HAL callback setup, selects the first
byte's source when the command header arrives, and reads that source live at
the read address. The remaining response and read-to-clear effects still use
the shared request handler. A full command queue must be resolved before the
first byte; the queue-fault test includes that immediate repeated-START case.
The address/byte interrupt body runs from `.RamFunc`; callbacks remain in flash.

The final safe SPI build passed 100 ID/version/options cycles at each of
100/200/400 kHz and 1000 further cycles with immediate repeated START at
400 kHz (3900 register reads total). Its overrun, bus-error, and first-byte
mismatch counters all remained zero. At 10 MS/s, the captured address-ACK
low periods were:

| Bus clock | Ordinary clock low | Write-address ACK low | Read-address ACK low |
|---|---:|---:|---:|
| 100 kHz | 5.2-5.3 us | 5.2-5.3 us | 5.2-5.3 us |
| 200 kHz | 2.7-2.8 us | 2.7-2.8 us | 2.8 us |
| 400 kHz | 1.5 us | 1.8-1.9 us | 2.4-2.5 us |

Repeated START at 400 kHz had 2.6-2.7 us read-address ACK lows. These are
measured normal STATUS paths, not worst-case guarantees for every operation.
The original Wire/HAL read-address ACK lows at 400 kHz were 16.8-42.3 us on
this same physical Nano fixture. Final 400 kHz electrical checks passed GPIO,
ADC, PWM, EEPROM software-reset retention/restoration, address changes, GPIO
IRQ, encoder, and the SPI mode/order/chunk matrix. The queue-fault build also
passed rejection of an immediate repeated-START header while its queue was full.

Default `CONFIG_I2C_NO_STRETCH=0` retains the hardware safety fallback. A forced
no-stretch build passed normal reads at 100/200/400 kHz during development, but
that does not establish safe concurrent EEPROM writes or NeoPixel output.
Those operations can block interrupt service, so forced no-stretch is a bench
option only. NeoPixel ring testing remains paused.

`capture_i2c.py` accepts `--clock 100000`, `200000`, or `400000`; the Nano
`--read-mode` choices are `delayed` (STOP plus 2 ms), `immediate` (STOP without
the delay), and `restart` (immediate repeated START). Nano inter-byte pauses
are controller-generated and must not be counted as target stretching.
`bench.py --clock` selects the same speeds for the electrical checks.

The all-feature UART image now exceeds the C011's reserved 30 KiB application
region. It is not in the C011 engineering CI matrix; the standalone public
UART example still compiles. The `uart` build target remains available for
development, but must not be flashed across the reserved EEPROM page.

## Nano fixture (2026-10-18)

Use `--fixture nano --physical-i2c` with `bench.py`, `circuitpython_host.py`, or
`cold_cycle.py` for the physical-jumper setup below.
The classic ATmega328P Nano is in the Nano socket, on its own USB connection;
build it with `build.ps1 -Target nano`. Its internal 5 V I2C pull-ups are used
only on the QT side of the onboard level shifter.
Preserve the existing D2/D3 OLED connections. Leave the ST-Link power lead
disconnected.

The rotated breakout has VIN/3V/GND at rows 1/2/3, PC14 at row 4,
raw SDA_3V/SCL_3V at rows 5/6, SWCLK/SWDIO at rows 7/8, PA12/PA11 at rows 9/10,
RST at row 31, and PA0 through PA8 at rows 32 through 40. Use physical ground
and VIN jumpers to the previously measured rails; crossbar resistance caused
a measured 0.264 V ground offset without the ground jumper. The Nano fixture
checks 3.0-3.6 V VDD and 4.0-5.3 V VIN without changing rail setpoints.

For the 5 V Nano, connect QT SDA to row 21 and QT SCL to row 22, then add physical
Nano A4-to-row-21 and A5-to-row-22 jumpers. Leave the corresponding switched
routes disconnected; `--physical-i2c` checks this and does not create them.
QT power/ground may use the same TOP-positive/GND rails as the
physical VIN/GND jumpers; remove both QT and VIN when performing a cold cycle.
The QT nets use the onboard shifter with high-side pull-ups to VIN. Do not use
the raw 3.3 V headers for this fixture. The direct-header run passed 100 kHz
status reads but failed at 400 kHz. The first shifted-connection run with Nano pull-ups disabled also
failed at 100 kHz. Enabling the Nano's normal pull-ups on the shifted side
initially passed 25 ID/version/options cycles and software reset at both 100
and 400 kHz, but longer tests still failed. RP2040 captures at 10/20 MS/s found
extra short SCL pulses associated with NACKs and corrupted reads. The failure
persisted with the debugger stopped. Direct 3.3 V STEMMA Friend control passed
3000 register reads at each of 100/400 kHz. Keeping the original Nano and level
shifter but bypassing the switched path with physical jumpers also passed
3000 reads at each speed. This isolates the observed problem to the switched
signal path or its loading/timing interaction, not a proven defective switch IC.

`rp2040_capture/` is Arduino firmware for the STEMMA Friend, retaining USB reset
and upload. On the verified GPIO28/29 connector revision, white SDA connects to
raw row 5, green SCL to raw row 6, black to ground, and red stays unconnected.
GPIO25 is also left input-only because it shares SDA on that revision.
`capture_i2c.py --physical-nano` captures the Nano's reads without switched I2C
routes; supply `--helper` and a new `--output` directory. The finite PIO buffer
preserves the beginning of a failed transaction, not an unlimited recording.
Only `--direct` enables the Friend's temporary open-drain controller: remove
the Nano I2C jumpers and any other controller connections before using it.
The Friend returns its pins to passive inputs after each direct transaction.

The fixture cross-checks ADC0/1 and DAC1 against the independent INA0 converter
for each run. These corrections are temporary; persistent calibration is not
rewritten. Before a UART test, disable Jumperless `serial_1` automatic Nano
routing (`autoconnect_flashing=0`, `connect_on_boot=0`), then restore the prior
configuration when the passthrough port is closed.
Opening the passthrough still resets the Nano through DTR, even with automatic
routing disabled. The UART test waits four seconds, discards the Nano startup
message, and reinitializes seesaw before transmitting.

Pre-fast-transport host checks passed: Arduino GPIO/ADC on nine pins and PWM on eight,
Arduino SPI's complete mode/order/chunk matrix and recovery, and the real
CircuitPython driver on CPython through the physical Nano I2C bus. Native
CircuitPython runtime remains untested. The independent raw SPI run also
passed all 16 mode/order/clock cases and its 4736-byte held-CS frame.
The GPIO-only build passed input/output/toggle/pull-up/pull-down on all 12
exposed pins, all 11 ADC inputs at five voltages, all 11 PWM-capable pins at
three frequency/duty settings, EEPROM software-reset retention/restoration,
and address 0x49->0x4A->0x49. The four-encoder build passed direction,
position/delta clearing, interrupt disable, channel isolation, and simultaneous
-8 detents per channel at 2 ms phase spacing.
Both address-strap builds passed all 16 combinations, including physical level
checks and preservation of the saved address byte.

UART passed 16-byte binary TX and 15-byte RX at 9600 and 115200 baud on two
complete runs. RX interrupt assert/clear/disable also passed. The first attempt failed because
opening the Jumperless passthrough reset the Nano; a separate later I2C NACK
was part of the switched-path investigation above. Its configuration precheck
also needed a 4096-byte read rather than exhausting the MicroPython heap.
The SPI queue-fault build passed its 400 kHz burst, error 3, measured CS release,
abort, and successful new transaction with the physical I2C jumpers. Earlier
switched-path attempts included both passes and I2C failures; they are not
counted as reliable transport evidence.

Cold-power EEPROM retention passed for eight nontrivial bytes, with measured
power-off and restoration of the original bytes. An unpowered ST-Link clamped
RST near 1.28 V; reconnecting its USB released the target without a debugger
command or reset, allowing the retention check to complete. Keep that adapter
powered while its reset lead is attached. All 2048 reserved EEPROM-page bytes
matched the original pre-verification backup after the firmware tests.
NeoPixel testing remains paused; no ring commands or power changes were made.

## Historical post-refactor validation — 2026-09-10, incomplete

The shared AVR/STM32 handlers were compiled and tested on 2026-09-10:

- All 18 AVR examples, the minimal C011 example, and all eight C011 engineering
  configurations compiled, including UART, four encoders, both strap polarities,
  and the SPI queue-fault build. No AVR hardware run was performed.
- Four-encoder firmware: all four physical channels passed -5/+3 motion,
  position/delta clearing, interrupt-disable, isolation, and simultaneous -8
  detents at 2 ms phase spacing.
- Both programmed images passed byte verification. After the tests, all 2048
  reserved EEPROM-page bytes matched the new pre-refactor full-flash backup;
  the option register remained `0xfffffeaa`. The 24396-byte SPI image remains
  loaded. No option-byte writes or mass erase were performed.
- Final SPI firmware: identity/options/software reset passed 25 cycles each at
  100/400 kHz; nine ADC pins passed five voltage points each; eight PWM pins
  passed all three frequency/duty combinations. GPIO IRQ, eight EEPROM bytes
  across software reset with restoration, and address 0x49->0x4A->0x49 passed.
- **FAILED:** the general GPIO run passed PA2-PA4, then stopped during PA5's
  high-output readback with `R 49 01 04 04: ERR I2C 3`. No GPIO retry was run.
- **FAILED:** Arduino SPI mode/order pairs (0,0), (0,1), and (1,0) passed all
  lengths 0/1/29/30/58/96; the next pair stopped with host result `FAIL FF`.
  No SPI retry was run. The source review did not establish either failure's
  cause; do not label them fixture faults or proven refactor defects.
- Further bus testing stopped: raw SPI, CircuitPython, Arduino GPIO/ADC/PWM,
  queue-overflow hardware, and physical strap combinations were not rerun on
  this refactor. Prior passes below remain historical. UART is compile-only in
  this sequence; NeoPixel ring commands and power changes remain stopped.

## Pre-refactor hardware evidence

2026-09-10 replacement-readiness checks before the shared-handler refactor,
using the same C011F6Ux breakout. These are historical passes, not substitutes
for the post-refactor checks below:

| Area | Actual evidence | Result |
|---|---|---|
| Programming | Application byte verification after each firmware change; option register remains `0xfffffeaa` | PASS |
| Preserved data | All 2048 reserved-page bytes compared with the original full-flash backup after EEPROM/address tests and firmware changes | PASS |
| I2C identity/reset | HWID `0x90`, version `0x549a`, normal options `0x26303`; 25 repeated ID/version/options cycles at both 100 and 400 kHz, then software reset | PASS |
| GPIO | Input/output, toggle, pull-up/down on all 12 exposed pins, including PA0/PA1/PC14 in the GPIO build | PASS |
| ADC | All 11 analog inputs, five points each from nominal 0.2 to 3.0 V; physical reference comparison within 0.12 V | PASS |
| PWM | All 11 PWM-capable pins; 200 Hz at 25%/75% and 500 Hz at 50%, measured independently | PASS within 8% frequency / 10 percentage-point duty limits |
| GPIO interrupt | PC14 asserted on PA4 changes, cleared on flags read, stayed released when disabled | PASS |
| Encoders | Four Metro-generated A/B pairs: physical voltage/readback, -5/+3 direction, delta and position clearing, IRQ disable, channel isolation, all four moving together | PASS at tested 2/5 ms phase spacing |
| EEPROM software reset | Eight nontrivial bytes written/read, retained over software reset, original bytes restored | PASS |
| EEPROM cold power | VDD 0.116 V / VIN 0.174 V measured off twice, five seconds apart; all eight pattern bytes retained after repowering through Jumperless, original bytes restored | PASS |
| EEPROM I2C address | Move 0x49->0x4A and restore; saved byte read back | PASS |
| Address straps | All 16 combinations each in active-low and inverted builds, with physical level checks; saved address byte unchanged | PASS |
| UART | Earlier TX/RX binary-data and RX-IRQ functional passes at 9600/115200; disabled in current normal build | Prior PASS; not rerun in this sequence |
| Arduino development host | Earlier full nine-pin GPIO/ADC and eight-pin PWM pass; final-image rerun passed pins PA0/PA1/PA4/PA5/PA6 then stopped at PA7 low-point ADC comparison | Final rerun incomplete: 54/1023 at VDD 3.418 V vs reference 0.303 V exceeds 0.12 V limit by about 0.003 V |
| Arduino SPI host | All four modes, both orders, lengths 0/1/29/30/58/96, receive-only, write-only, held CS, invalid continuation and recovery | PASS on final SPI firmware |
| CircuitPython development host | Real driver on CPython via physical Metro I2C: begin/map, GPIO/ADC/PWM, same SPI mode/chunk matrix and error recovery | PASS on final SPI firmware; native CircuitPython untested |
| AVR regression | All 18 examples recompiled after final STM32 dependency separation; AVR implementation unchanged | PASS |
| SPI wire protocol | Final image: all 16 mode/order/clock combinations decoded independently; 4736-byte held-CS loopback frame, D/C changes, write-only and invalid-command recovery | Passing rerun; one unresolved intermittent long-frame mismatch on preceding run |
| SPI queue overflow | Final-code fault-injection build: 10 ms servicing hold, 400 kHz I2C burst, error 3, physical CS release, abort and successful new transaction | PASS; artificial overload, not an observed production traffic rate |
| FHT/audio spectrum | AVR assembly implementation has no STM32 port; enabling it explicitly fails compilation | Unsupported |
| NeoPixel | No ring commands or ring power changes during these checks | STOP: supervised testing only |

After the shared-handler refactor, the normal UART-off build occupies
**22252 flash bytes**, leaving **8468 bytes** before the reserved EEPROM page,
and uses 1796 bytes of static RAM. The SPI build occupies **24396 flash bytes**
and uses 1972 bytes of static RAM. The opt-in UART build occupies **30468 flash
bytes** and uses 2364 bytes of static RAM: only **252 flash bytes** remain before
EEPROM, so additional features require a new size check. The minimal example
occupies 17004 flash bytes and 1400 static RAM bytes.
The Arduino size summary omits 212 bytes of vectors/initialization sections;
these flash figures use the binary size and programmer's byte verification.

SPI is experimental: the first post-size-reduction long-frame run reported a
loopback mismatch after all 16 short-frame combinations passed. One bounded
diagnostic rerun (with explicit sent/received diagnostics added) passed the
entire sequence, including 4736 bytes in 15.22 seconds. The original mismatch
has not been explained; do not count this as a clean reliability qualification.

The final Arduino host ADC rerun also stopped at PA7's low stimulus: the
10-bit host result was 54 with VDD 3.418 V, versus a 0.303 V independent fixture
reading. The difference is about 0.123 V against the unchanged 0.12 V limit.
The raw ADC sweep and CircuitPython driver checks passed on the final image;
the marginal Arduino comparison remains unresolved, and that rerun did not
reach PA7 PWM or PA8/PA11/PA12. Earlier complete passes are not substitutes for
this failed final rerun.

One additional reset-release probe was inconclusive: the Jumperless weak-pull
voltage did not establish a low level after software reset. Direct debugger
reads did prove PA0 MODER changes from output (`01`) to input (`00`); the
electrical high-impedance reset probe is **not** counted as passed. The initial
probe also used an incorrect pull argument (`LOW` is no pull; down is `-1`),
but correcting that did not resolve this fixture check. Further probing stopped.

The first Metro encoder attempt used Nano-socket nodes and stopped during its
electrical preflight, before quadrature generation. Ladyada then supplied the
actual breadboard anchors (USB row 30, D13 row 60); the corrected row mapping
below passed all four channels. No push-pull high is used on the Metro.

Earlier UART testing encountered a USB cleanup stall and automatic Nano D0/D1
passthrough routes. Ladyada reset the Jumperless; both ports subsequently
responded and the extra routes were removed. Its root cause remains unknown.
This sequence did not open the UART passthrough port. Serial drain waits in the
test harness are now bounded; all completed phases exited normally.

If the C011 flash controller is stuck with CFGBSY set despite working memory
reads, the verified recovery is OpenOCD `cortex_m reset_config sysresetreq`,
`reset_config none`, then `reset halt`. The adapter's physical reset pulse did
not clear this session's stuck state. Preserve a full flash backup and option
readout first, inspect each programming response, and verify before running.

All 18 existing AVR examples compiled again after the shared-handler refactor
with megaTinyCore 2.6.11, using their chip-specific test selections where present.
The neodriver example retains its existing low-RAM warning. AVR hardware has not
been rerun in this refactor; its protocol implementation is now shared.

## Shared protocol architecture

The common header owns module defaults/state, register byte encoding, build-date
packing, address-strap polarity/weights, and the existing AVR detent decoder.
`Adafruit_seesawPeripheral_receive.h` and `_request.h` now decode commands and
responses for both targets. GPIO/PWM/NeoPixel hardware hooks, ADC acquisition,
EEPROM storage, and transport scheduling remain target-specific. STM32 queues
blocking work for `loop()`; AVR retains its callback dispatch and deferred UART TX.
FHT remains AVR-only and the experimental SPI controller remains STM32-only.

Valid ordinary host commands keep their encoding. The common decoder also
requires complete payloads, bounds-checks pin shifts and NeoPixel writes, and
rejects zero PWM frequency. AVR gains GPIO toggle/interrupt-clear dispatch and
module-status responses previously missing from its handlers. IRQ timing and
encoder position-write delta behavior retain their target-specific semantics.

## Physical fixture

The breakout straddles the Jumperless gap:

| Jumperless row | Breakout label | Seesaw pin index |
|---|---|---|
| 1-9 | PA8, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0 | 8-0 |
| 10 | RST | Reserved |
| 31 | PA11 | 9 |
| 32 | PA12 | 10 |
| 33 | PA13 / SWDIO | Reserved |
| 34 | PA14 / BOOT0 / SWCLK | Reserved |
| 35 | SCL_3V | Reserved |
| 36 | SDA_3V | Reserved |
| 37 | PC14 | 15; IRQ in the full build |
| 38 | GND | Ground |
| 39 | 3V | Sense only |
| 40 | VIN | Sense only |

The QT-connected setup was used for the main test sequence. Do not assume the
assembled bench board has the level shifter shown in the local breakout design;
Ladyada reports no shifter on this fixture. Do not connect arbitrary Metro 5 V
GPIO directly to the C011 without checking its exact pin configuration. Connect ST-Link SWDIO,
SWCLK, and GND; leave the ST-Link power-output lead disconnected. The firmware
reserves SWD, I2C, reset, and the onboard PC15 LED.

The Metro Mini is plugged into the **breadboard**, not the Nano socket. Ladyada
confirmed USB/VBUS at row 30 and D13 at row 60. The
[official Metro Mini pinout](https://learn.adafruit.com/adafruit-metro-mini/pinouts)
and local Rev C board header pad order map D4-D11 to rows 51-58:

| Encoder | C011 A/B rows | Metro pins | Metro rows |
|---|---|---|---|
| 0 | 7, 6 (PA2, PA3) | D4, D5 | 51, 52 |
| 1 | 5, 4 (PA4, PA5) | D6, D7 | 53, 54 |
| 2 | 3, 2 (PA6, PA7) | D8, D9 | 55, 56 |
| 3 | 31, 32 (PA11, PA12) | D10, D11 | 57, 58 |

The Nano socket's D2/D3-to-GPIO7/GPIO8 OLED connections are unrelated and must
remain untouched. Encoder tests verify each released/low voltage and C011
input readback before generating pulses. Use `--encoders 4` only with the
four-encoder firmware. Metro commands `P` and `E` never drive a high output.

The default full build leaves PA0/PA1 available as GPIO and uses PA2/PA3 for
encoder A/B. The opt-in `uart` build reserves PA0/PA1 for UART. The `gpio` build disables UART, encoders,
and IRQ so every exposed GPIO, including PC14, can be tested separately.

Without `--jumperless-link`, `bench.py` never supplies VIN or 3V. It supplies only bounded 0.2-3.0 V ADC test
stimuli and 3.3 V logic stimuli after configuring the target pins as inputs. It
preserves the pre-existing OLED/probe routes and ground, removes only its own
temporary routes, and restores the DAC1 setpoint after a run. Unexpected routes
abort the run before signal routing.

The separately verified `--jumperless-link` route in `cold_cycle.py` and `bench.py` uses
Metro VBUS row 30 -> C011 VIN row 40, Metro A5/SCL row 17 -> C011 PB6 row 35,
and Metro A4/SDA row 18 -> C011 PB7 row 36, with the existing row 38 ground.
Power is connected and measured before I2C; I2C is disconnected before power.
Measured VBUS/VIN/VDD were 5.214/5.158/3.476 V, and idle SCL/SDA were
3.697/3.717 V. PB6/PB7 are FT_f pins; [DS13866 Rev 5](https://www.st.com/resource/en/datasheet/stm32c011f6.pdf),
Table 23 note 2, requires internal pulls disabled above VDD+0.3 V. The loaded
STM32duino mapping uses alternate-function open-drain with `LL_GPIO_PULL_NO`.
This verified exception is not permission to drive arbitrary pins at 5 V or
apply I2C signals while the C011 is unpowered. Jumperless rails are not used.

## Build and run

Use STM32duino 3.0.0, Arduino AVR 1.8.8, pyserial, and the official
[Jumperless helper](https://github.com/Architeuthis-Flux/Large-Breadboard-Model).
The helper path and serial ports are explicit arguments, since USB interface
numbers differ by host. The UART port must be the Jumperless **passthrough**
interface, not its main terminal or MicroPython interface.

```powershell
./extras/hwtest/build.ps1 -Target full -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target metro -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target gpio -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target four_encoders -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target address_straps -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target address_straps_inverted -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target uart -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target spi -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target spi_queue -OutputRoot <build-directory>
./extras/hwtest/build.ps1 -Target avr -OutputRoot <build-directory>
```

The Metro sketch now needs the companion development Arduino seesaw library;
pass `-HostLibrary <Adafruit_Seesaw-checkout>` when building `metro`. Other
peripheral targets do not need that development host checkout.
Use `bench.py --phase host` for Arduino GPIO/ADC/PWM, `--phase spi_host` for
Arduino SPI, `--phase spi` for independent raw-protocol SPI checks, and
`--phase spi_queue` only with the fault-injection firmware.

`circuitpython_host.py` takes `--library <Adafruit_CircuitPython_seesaw-checkout>`,
`--metro`, `--jumperless`, and `--helper`; it uses the verified powered link.
Neither host-library test issues NeoPixel commands.

The C011 build deliberately sets `upload.maximum_size=30720`: the final 2 KB
page at `0x08007800` belongs to EEPROM emulation. The firmware also checks its
linked data end before permitting flash writes. Do not remove that reservation.

After uploading both firmwares with the verified programmer/wiring, run one
phase at a time first:

```text
python extras/hwtest/bench.py --metro <port> --jumperless <repl-port> --uart <passthrough-port> --helper <helper-checkout>/scripts/jumperless.py --phase preflight
```

Then select `status`, `gpio`, `adc`, `pwm`, `irq`, `encoder`, `uart`, `eeprom`,
`address`, or `straps`. Use `--encoders 4` with the four-encoder build, and
`--straps-inverted --address 0x58` with the inverted-strap build at idle.
`all` runs the automatic phases, not straps or the optical ring test; UART runs
only when advertised by the loaded firmware. Add `--gpio-build`
only with the GPIO-only firmware; that run checks all twelve exposed GPIO pins.
The saved-data tests preserve and restore the bytes they touch. The address test
temporarily moves the device from 0x49 to 0x4A and restores its original address.

## NeoPixel ring

Reserve PA8 / Jumperless row 1 for DIN. Verify the ring type, pixel count, supply,
and input-high requirement before connecting it. A 5 V ring may require a proper
3.3-to-5 V logic buffer; the Jumperless crossbar is not a level shifter. Do not
power a ring from the C011 regulator without checking the current budget.

Once an RGB/GRB ring is safely connected, `--phase neopixel --pixels <count>`
emits dim red, green, blue, and white, then turns the ring off. It reports
`OBSERVATION_REQUIRED`, never an optical PASS based only on register writes.
RGBW rings use raw bytes too, but need an appropriate RGBW test pattern rather
than this RGB test. The firmware buffer is 192 bytes: 64 RGB or 48 RGBW pixels.

## Known limits / remaining work

- The experimental SPI controller bridge is implemented. Its
  [protocol and limits](SPI_PROTOCOL.md) describe chunking, held CS, errors,
  clock range, and the remaining real-display integration work.
- NeoPixel testing is a supervised stop point: do not connect, power, or run
  the ring until Ladyada and the operator review the wiring together.
- Hardware ID 0x90 is provisional. Released Arduino/CircuitPython seesaw drivers
  need the companion development ID/mapping updates before ordinary `begin()`
  accepts this chip. `metro_host` supports both raw and actual host-library tests.
- ADC results are 12-bit (0-4095), not the original ATtiny's 10-bit counts.
- PWM has one shared requested frequency. PA3/PA11 share TIM1 CH4;
  PA7/PC14 share TIM3 CH2. Aliased channels cannot have independent duty values.
- GPIO changes and encoders are polled. NeoPixel transfers and flash writes can
  delay polling, so this is not a guaranteed high-speed edge counter.
- EEPROM provides 256 wire-addressable bytes; byte 255 selects the I2C address.
  A changed write rewrites one flash page. It is **not wear-levelled or
  power-failure atomic**. Allow 100 ms after writes; unchanged writes do not erase.
  Eight-byte cold-power-cycle retention passed; interrupted-write behavior
  remains untested.
- The existing AVR-assembly FHT audio-spectrum module is not ported to STM32.
  `CONFIG_FHT` fails compilation explicitly instead of advertising a fake module.
