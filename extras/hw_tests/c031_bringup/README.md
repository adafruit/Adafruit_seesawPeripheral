# C031G6 breakout bring-up — PID 6540

These are manual bench tests, not a production PASS/FAIL fixture.
Compilation and protocol simulation alone do not establish hardware correctness.
Use an Adafruit_Seesaw host library with STM32C031 ID 0x91 support.

The dated results below preserve bench evidence originally recorded in
[#124](https://github.com/adafruit/Adafruit_seesawPeripheral/pull/124).
They are historical measurements, not fresh validation of every host sketch.
See the outstanding physical checks at the end before treating a board as verified.

To update a C031 over I2C, upload the
[serial bridge](../../../examples/serial_i2c_bridge/serial_i2c_bridge.ino)
to the Metro/Uno and follow the
[Python/Web Serial updater instructions](../../stm32_i2c_bootloader/README.md).
Select **STM32C031G6** and use a matching application BIN or HEX built from the
sources below. Check its size against the EEPROM boundary before programming.
After flashing the STM32, replace the Metro bridge sketch with the desired host test.

## Firmware and wiring

- Build/upload [c031_gpio_handtest](c031_gpio_handtest/c031_gpio_handtest.ino)
  to the **STM32C031G6** for unrestricted GPIO/ADC/PWM checks. It reports PID 6540,
  address 0x49, and supports ROM entry.
- After pin checks, upload the
  [default C031 firmware](../../../examples/example_stm32c031/example_stm32c031.ino)
  for GPIO, ADC, PWM, NeoPixel, EEPROM and PA2/PA3 encoder support, with
  PC14/D30 INT, PB8/D24 ADDR0 and PC6/D22 ADDR1 reserved.
- Both images must fit below 0x08007800 (30 KiB); the last 2 KiB is EEPROM.
  Use the generic C031G6Ux STM32duino target and the supplied build_opt.h.
- Upload all other sketches here to the **Metro 328 / Uno**, not the STM32.
  Select **Arduino Uno**, open Serial Monitor at **115200 baud**, and connect
  A4/SDA, A5/SCL and GND through the breakout's level-shifted QT bus.
- Never put 5 V on a C031 GPIO. Disconnect input jumpers before output/PWM tests.
  Only one target board should be attached during programming/option changes.

The C031 uses this label-based map. Earlier bring-up binaries used dense
numbering and are not pin-compatible; update the peripheral and host sketches together:

| Seesaw number | Physical pin |
| --- | --- |
| 0–8 | PA0–PA8 |
| 11, 12, 15 | PA11, PA12, PA15 |
| 16, 17 | PB0, PB1 |
| 19–21 | PB3–PB5 |
| 22 | PC6 — ADDR1 by default |
| 24 | PB8 — ADDR0 by default |
| 30 | PC14 — INT by default |
| 31 | PC15 onboard LED |

SWD, I2C, reset, and unlisted numbers remain reserved. In the default image,
INT is active-low/open-drain; floating address straps select 0x49. Ground
ADDR0, ADDR1, or both for 0x4A, 0x4B, or 0x4C, then reset. The unrestricted
hand-test image leaves all three role pins available as GPIO.
ADC: 0–8, 11/12 and 16/17. PWM: every available pin except 12 (PA12).
Shared PWM channels: 3/11, 5/15, 6/20/24/22, 7/21/30, and 8/16/31.
Test one pin from each group at a time.

## Bench sequence

1. [c031_metro_handtest](c031_metro_handtest/c031_metro_handtest.ino):
   n/p selects a labeled pin; 0/1 drives LOW/HIGH; u/d selects pull-up/pull-down;
   r reads; a reads ADC; w drives 1 kHz PWM; x releases outputs.
   Check real 0/3.3 V with a meter, and PWM with a scope.
   Use a 4.7 kΩ resistor to GND/3.3 V for input tests.
2. [c031_adc_table](c031_adc_table/c031_adc_table.ino):
   all 13 analog inputs, 10-bit host readings (0–1023).
3. [c031_input_pullup](c031_input_pullup/c031_input_pullup.ino):
   all 20 header inputs. Ground each through 1 kΩ; check that only the matching
   column says PRESSED. Use the hand tester's d command for the pull-down test.
4. [c031_pwm_sweep](c031_pwm_sweep/c031_pwm_sweep.ino):
   each PWM output in sequence, including the LED.
5. [c031_servo_test](c031_servo_test/c031_servo_test.ino):
   PA0, 50 Hz, 16-bit duty; inspect with a scope first.
   If connecting a servo, use suitable external power and common GND.
6. Flash the **default** firmware, then run
   [c031_neopixel](c031_neopixel/c031_neopixel.ino): one pixel DIN on PA4.
   Check red/green/blue/off with appropriate pixel power and level shifting.
7. [c031_encoder](c031_encoder/c031_encoder.ino):
   A/B on PA2/PA3, common to GND; rotate both ways.
8. [c031_eeprom](c031_eeprom/c031_eeprom.ino): record the printed original byte,
   send w once, power-cycle only the C031, send v, then r to restore.
   Keep Metro powered throughout. Byte 0x02 is used; the I2C address byte is untouched.
9. [c031_recovery_test](c031_recovery_test/c031_recovery_test.ino):
   check that LED blinking resumes after target reset/power-cycle, Metro reset,
   and a cold start of both. Avoid back-powering during power-off.
10. With the USB bridge sketch, select **STM32C031G6** in the uploader.
    Confirm ROM ID 0x0453 at 0x63, then test application Write/Verify/Go and
    seesaw software reset. The guided BOOT0 option write/readback is a separate,
    physically unverified test: disconnect SWCLK, use B0 = 3.3 V during reset,
    and require VERIFIED. See the updater instructions before changing options.

## Label-based map verification — 2026-10-25 UTC

The remapped hand-test image passed all 20 header GPIO input/output/pull checks,
all 13 ADC inputs at five voltages, and all 19 header PWM outputs at three
settings. One initial PB4 duty capture failed; nine focused measurements passed
without a firmware change, so that failure was not reproduced. After compacting
the PWM lookup table to 16-bit core indices, all 57 PWM measurements passed again.

All eight combinations of high-byte GPIO bits 24/30/31 passed at 100/400 kHz
with delayed, immediate and repeated-start reads. On the default image, PC14
interrupt assertion/flag-read/release and disable passed, including the D31
interrupt flag `0x80000000`. PB8/PC6 selected all four addresses, 0x49–0x4C;
reserved INT/ADDR pins rejected GPIO control and were masked from GPIO reads.
LED D31 level readback and software reset also passed.

The tested default BIN was 29,736 bytes (2,248 bytes static RAM); the unrestricted
hand-test BIN was 26,984 bytes (2,240 bytes static RAM). Both were programmed
and readback-verified. The final EEPROM and option areas matched the pre-test
backups byte-for-byte. At the end of that run, the DUT was left on the new default
image, and the Nano bridge and original Jumperless connections were restored.
Earlier bring-up binaries use the old map and must not be mixed with these host sketches.
The SPI example uses the same unused-debug-UART and EXTI build exclusions as
the default example. Its tested BIN was 29,472 bytes, down from 32,768, leaving
1,248 bytes below the 30 KiB EEPROM boundary. The example also compiled for C011,
C051, and G071. On C031, the smaller image passed all four SPI modes, both bit
orders, two requested clock rates, independent byte decoding, a 4,736-byte
held-CS transfer, host chunk boundaries, and error recovery.

## Hardware results — 2026-10-24

The results below used the earlier dense pin map; D24 there means PC15.

Tested STM32C031G6U6, 32 KiB flash, DBGMCU ID 0x10016453 (Rev Z), using
STM32duino 3.0.0, a classic Nano on the level-shifted QT bus, and Jumperless.
The C031 ROM at 0x63 reports ID 0x0453 and protocol 0x11. Its observed GET
data is `12110001021121314463738292324564748393a1`: 20 bytes, not the
provisional 19-byte frame. The uploader requires that observed framing.

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

Peripheral tests used separate feature-specific firmware images. The default
and GPIO hand-test images were verified to fit below the EEPROM boundary in
that bench run. Generic examples compiled with the core's full 32 KiB limit
are compile checks, not EEPROM-safe updater images. Some exceed 30 KiB and
must be reduced and size-checked before programming.

Still requires physical verification: cold power cycling without back-powering,
NeoPixel waveform/color, onboard LED light output, and physical BOOT0 entry plus
option-write/readback. Disconnect ST-Link SWCLK before strapping B0 to 3.3 V.
No boot options or protection settings were changed during these tests.
Browser protocol tests passed in simulation; a live browser Web Serial session
was not exercised on this board.
