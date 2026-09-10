# STM32C011 engineering bring-up

These are engineering tests, not production fixture firmware.

- `c011_firmware/`: STM32C011F6Ux peripheral firmware. Full configuration enables
  GPIO, 12-bit ADC, 16-bit PWM commands, open-drain GPIO interrupt,
  one quadrature encoder, 192-byte NeoPixel buffer, and flash-backed EEPROM.
  The UART bridge is off by default; use the `uart` build target to enable it.
  Core Serial support is disabled except in the UART build. The STM32 backend
  uses private protocol constants without the host-side BusIO dependency.
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

## Current validation status

2026-09-10 replacement-readiness checks, using the same C011F6Ux breakout:

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

The normal UART-off build occupies **20592 flash bytes**, leaving **10128 bytes**
before the reserved EEPROM page, and uses 1660 bytes of static RAM. The SPI build
occupies **24140 flash bytes** and uses 1964 bytes of static RAM. The opt-in UART
build occupies 28804 flash bytes and uses 2228 bytes of static RAM.
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

All 18 existing AVR examples compiled with megaTinyCore 2.6.11, using their
chip-specific test selections where present. The existing AVR implementation is
unchanged. The neodriver example retains its existing low-RAM warning.

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
- ROM I2C bootloader programming remains unimplemented. No option bytes have
  been changed. Host-library code is implemented but not in released libraries.
