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

Record the MCU/ROM revision and actual measurements. The C031 ROM profile
currently expects AN4221 V1.1's 19-byte GET data frame; a different response
stops before any erase/write. C031 flashing, option writes, BOOT0 entry, GPIO
voltages/waveforms, and all peripheral functions still need physical verification.
