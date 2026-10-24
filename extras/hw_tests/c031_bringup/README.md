# C031G6 breakout bring-up — PID 6540

These are manual bench tests, not a production PASS/FAIL fixture.
Software compilation and protocol simulation do not qualify a board for booking.
Use the current Adafruit_Seesaw host library with STM32C031 ID 0x91 support.

In the downloadable bundle, first upload `serial_i2c_bridge/serial_i2c_bridge.ino`
to the Metro/Uno, then open the bundled `webserial.html` locally in Chrome/Edge.
Select **STM32C031G6**, connect, and use the supplied BIN or HEX files.
Use the bundled page/bridge until the PR is merged; the public page updates
only after that merge. After flashing the STM32, replace the Metro bridge
sketch with the desired host test from `c031_bringup`.
The default firmware source is also included under `examples/example_stm32c031`;
rebuilding it requires this PR's peripheral-library changes for the D24 LED.

## Firmware and wiring

- Build/upload `c031_gpio_handtest` to the **STM32C031G6** for unrestricted
  GPIO/ADC/PWM checks. It reports PID 6540, address 0x49, and supports ROM entry.
- After pin checks, upload `examples/example_stm32c031` for the default
  GPIO, ADC, PWM, NeoPixel, EEPROM and PA2/PA3 encoder configuration.
- Both images must fit below 0x08007800 (30 KiB); the last 2 KiB is EEPROM.
  Use the generic C031G6Ux STM32duino target and the supplied build_opt.h.
- Upload all other sketches here to the **Metro 328 / Uno**, not the STM32.
  Select **Arduino Uno**, open Serial Monitor at **115200 baud**, and connect
  A4/SDA, A5/SCL and GND through the breakout's level-shifted QT bus.
- Never put 5 V on a C031 GPIO. Disconnect input jumpers before output/PWM tests.
  Only one target board should be attached during programming/option changes.

The C031 keeps its existing dense numbering; it does **not** use the C011's
label-matching numbers:

| Seesaw number | Physical pin |
| --- | --- |
| 0–8 | PA0–PA8 |
| 9, 10 | PA11, PA12 |
| 13 | PA15 |
| 14, 15 | PB0, PB1 |
| 16–18 | PB3–PB5 |
| 21, 22, 23 | PB8, PC6, PC14 |
| 24 | PC15 onboard LED |

SWD (11/12), I2C (19/20), reset (25), and remap aliases (26/27) remain reserved.
ADC: 0–10 and 14/15. PWM: every listed pin except 10 (PA12).
Shared PWM channels: 3/9, 5/13, 6/17/21/22, 7/18/23, and 8/14/24.
Test one pin from each group at a time.

## Bench sequence

1. `c031_metro_handtest`: n/p selects a labeled pin; 0/1 drives LOW/HIGH;
   u/d selects pull-up/pull-down; r reads; a reads ADC; w drives 1 kHz PWM;
   x releases outputs. Check real 0/3.3 V with a meter, and PWM with a scope.
   Use a 4.7 kΩ resistor to GND/3.3 V for input tests.
2. `c031_adc_table`: all 13 analog inputs, 10-bit host readings (0–1023).
3. `c031_input_pullup`: all 20 header inputs. Ground each through 1 kΩ;
   check that only the matching column says PRESSED. Use the hand tester's
   d command for the corresponding pull-down test.
4. `c031_pwm_sweep`: each PWM output in sequence, including the LED.
5. `c031_servo_test`: PA0, 50 Hz, 16-bit duty; inspect with a scope first.
   If connecting a servo, use suitable external power and common GND.
6. Flash the **default** firmware, then `c031_neopixel`: one pixel DIN on PA4.
   Check red/green/blue/off with appropriate pixel power and level shifting.
7. `c031_encoder`: A/B on PA2/PA3, common to GND; rotate both ways.
8. `c031_eeprom`: record the printed original byte, send w once, power-cycle
   only the C031, send v, then r to restore. Keep Metro powered throughout.
   Byte 0x02 is used; the I2C address byte is untouched.
9. `c031_recovery_test`: LED blink resumes after target reset/power-cycle,
   Metro reset, and a cold start of both. Avoid back-powering during power-off.
10. With the USB bridge sketch, select **STM32C031G6** in the uploader.
    Confirm ROM ID 0x0453 at 0x63, then test application Write/Verify/Go and
    seesaw software reset. The guided BOOT0 option write/readback is a separate
    test: disconnect SWCLK, use B0 = 3.3 V during reset, and require VERIFIED.

## Hardware results — 2026-10-24

Tested STM32C031G6U6, 32 KiB flash, DBGMCU ID 0x10016453 (Rev Z), using
STM32duino 3.0.0, a classic Nano on the level-shifted QT bus, and Jumperless.
The C031 ROM at 0x63 reports ID 0x0453 and protocol 0x11. Its observed GET
data is `12110001021121314463738292324564748393a1`: 20 bytes, not the
provisional 19-byte frame. The uploader now requires that observed framing.

- All 20 exposed GPIO inputs/outputs and internal pulls passed electrical checks.
- All 13 ADC inputs passed five voltage points from 0.2 to 3.0 V; the largest
  observed error against the bench reference was under 0.015 V. This is a bench
  comparison, not a calibrated accuracy specification.
- All 19 header PWM pins passed 200 Hz at 25%/75% and 500 Hz at 50%, measured
  with the existing coarse edge sampler (8% frequency, 10 percentage-point duty limits).
- 3,600 register reads passed at 100/400 kHz with delayed, immediate, and
  repeated-start reads; independent 10 MS/s captures decoded correctly.
  The LED test exposed a first-reply-byte bug that hid GPIO bit 24. After the
  fix, 4,800 status/GPIO reads passed those same modes while toggling D24;
  the captures confirmed the nonzero first byte on the wire.
  D24 interrupt flags also passed all six bus/read modes, with the interrupt
  pin asserting and releasing correctly on flag reads.
- Clock stretching is **not eliminated**. At 400 kHz, the original read-address
  ACK lows were 2.3–2.7 us against ordinary bit lows of 1.4–1.6 us. Releasing
  ADDR before loading TXDR reduced the observed read-address ACK lows to
  2.0–2.3 us; write-address ACK lows remained about 1.8–2.0 us. Another 4,800
  reads passed all six bus/read modes at the unchanged 48 MHz CPU clock.
  These are 10 MS/s Nano-bus captures, not a guaranteed worst-case bound or a
  throughput measurement: controller byte-boundary pauses remain. Stretching
  remains enabled for late TXDR service, EEPROM writes and NeoPixel output;
  busy-case timing was not measured in this follow-up.
- GPIO interrupts and four encoders passed independent and simultaneous
  direction/count/interrupt tests. The older bench needed its interrupt-enable
  payload byte restored and stale deltas drained before each interrupt test.
- EEPROM software-reset and hardware-NRST persistence, address change/restore,
  and full reserved-page restoration passed. PWM also passed with EEPROM and
  four encoders enabled.
- SPI passed all four modes and both bit orders at two requested clock rates,
  independent byte decoding, a 4,736-byte held-CS transfer, and error recovery.
  The Arduino SPI host passed 0/1/29/30/58/96-byte chunk-boundary tests.
- UART binary TX/RX and receive interrupts passed at 9,600 and 115,200 baud.
  The UART example's two address straps passed all four combinations.
- Spectrum sampling placed 500/1,000/2,000 Hz test tones in the expected bins
  at two sampling rates, including upper-half bins and rate clamping.
- The default PID 6540 image passed Arduino host GPIO, 10-bit ADC normalization,
  and 8-/16-bit PWM checks. Its UID matched SWD readout; temperature was about
  35 degrees C. The 50 Hz servo test measured 1/1.5/2 ms pulses within 0.15 ms.
  LED pin readback and NeoPixel command handling passed, without optical proof.
- ROM application erase/write/readback/Go and seesaw software reset passed,
  including entry from running seesaw and preservation of a nonblank EEPROM
  marker. After restoring the marker, the full EEPROM and option areas matched
  their initial backups. The bridge's C031 option query also passed.

Peripheral tests used separate feature-specific firmware images. Only the
default and GPIO hand-test images are included in the download; both fit below
the EEPROM boundary. Generic examples compiled with the core's full 32 KiB
limit are compile checks, not EEPROM-safe updater images. Some exceed 30 KiB
and must be reduced and size-checked before programming.

Still requires physical verification: cold power cycling without back-powering,
NeoPixel waveform/color, onboard LED light output, and physical BOOT0 entry plus
option-write/readback. Disconnect ST-Link SWCLK before strapping B0 to 3.3 V.
No boot options or protection settings were changed during these tests.
Browser protocol tests passed in simulation; a live browser Web Serial session
was not exercised on this board.
