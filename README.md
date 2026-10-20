# Adafruit seesawPeripheral Library[![Build Status](https://github.com/adafruit/Adafruit_seesawPeripheral/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_seesawPeripheral/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_seesawPeripheral/html/index.html)

Library for making seesaw i2c peripherals in Arduino core

To install, use the Arduino Library Manager and search for 'Adafruit seesaw Peripheral' and install the library.

## Source layout

The public include remains `Adafruit_seesawPeripheral.h`. Shared protocol code
lives in `src/`, with hardware-specific code in `src/arch/avr/` and
`src/arch/stm32/`. Transform lookup tables shared by the spectrum backends live
in `src/dsp/`. The AVR FHT implementation and its original documentation remain
together in `src/arch/avr/`.

## Example portability

General feature examples, including `example_eeprom` and the combined-feature
`example_stm32c011`, are enabled in CI for the larger tinyAVR and supported STM32
targets. Spectrum capture uses AVR FHT or the STM32 FFT backend. Product examples
keep their original board wiring and use explicitly
marked alternate pin mappings where supported on other targets.

The large NeoPixel buffers in `example_neodriver` and `example_pid5690` need
more RAM than the 8 KiB tinyAVR parts provide. NeoDriver also uses an AVR-only
activity-LED hook. `example_pid5752` needs the 24-pin tinyAVR pinout: its four
encoders, address straps, interrupt and NeoPixel need 13 GPIOs, while the
STM32C011 exposes 12 after reserving I2C, SWD and the board LED.

## ATtiny416 support

Select megaTinyCore's **ATtiny416** target and start with `example_gpioonly`
or `example_adc`. It uses the same 20-pin GPIO, ADC, PWM, and debug-UART pin
mapping as ATtiny816, and reports hardware ID `0x46`. The host-side seesaw
library must recognize this ID.

The ATtiny416 has only 4 KiB flash and 256 bytes RAM. Keep serial debugging
disabled and check both memory totals before adding features. The existing
PWM example exceeds flash capacity; even GPIO plus PWM with EEPROM and
debugging disabled is too large with megaTinyCore 2.6.11. FHT and large
NeoPixel buffers require a larger chip. ATtiny416 is not in the CI matrix.

## STM32C011 core support

Select STM32duino's generic **STM32C011F6Ux** target and start with
`examples/example_stm32c011`. This backend supports GPIO direction, bulk
read/write/toggle, pull-up/pull-down selection, optional active-low GPIO change
interrupts, 12-bit ADC reads, PWM, NeoPixel, encoders, UART, opt-in EEPROM,
and an opt-in SPI controller bridge. Audio spectrum capture is described below.

I2C uses PB7 (SDA) and PB6 (SCL), with address `0x49` in the example. The target
requires a 48 MHz peripheral clock and retains hardware clock stretching as a
safety fallback. Call `Adafruit_seesawPeripheral_run()` regularly to process
queued commands and sample GPIO changes. ADC replies contain the last completed
conversion; allow conversion time after selecting a channel. ADC status is zero
on success and one for an invalid channel or a conversion failure.

On C011, C031, C051, and G071, `CONFIG_ADC` also enables the internal die
temperature sensor through the standard seesaw `getTemp()` / `get_temp()` call
(`STATUS_TEMP`, signed 16.16 degrees Celsius). It uses factory calibration and
VREFINT supply compensation; C0 uses a typical slope, while G071 has two factory
calibration points. This measures chip temperature, not precision ambient
temperature. Allow 1 ms between selecting the register and reading it; an
immediate repeated START returns the previous reading. A failed conversion
retains the previous reading (zero until the first success) and sets ADC status
to one. Temperature sensing is disabled with ADC, including in FHT builds.
`examples/example_stm32_temperature` prints a reading each second on the debug
UART (PA0 TX, 115200 baud) while continuing to serve I2C.

All supported STM32 targets expose their factory-programmed 96-bit unique ID
at status base `0x00`, register `0x05` (`SEESAW_STATUS_UID`). Read 12 bytes:
UID words 0, 1, and 2 in address order, with each word sent most significant
byte first. The ID is read-only, survives resets and reflashing, and is available
without ADC or EEPROM, including in spectrum builds. No conversion delay is
needed. Host libraries without a UID helper can use a raw 12-byte register read;
the result can be displayed as a 24-digit hexadecimal serial number.

Protocol GPIO indices 0–8 map to PA0–PA8, 9/10 to PA11/PA12, and 15 to PC14.
Only indices 0–10 support ADC. SWD, I2C, the PC15 LED, and configured address or
interrupt pins are excluded from ordinary GPIO/ADC access. Enable
`CONFIG_INTERRUPT_PIN` only on an exposed GPIO pin; its output is open-drain.

Enable `CONFIG_PWM` (or `CONFIG_PWM_16BIT`) for PWM on indices 0–9 and 15.
Both settings accept 16-bit duty values, from 0 through 65535. Frequency is
shared across active PWM outputs, defaults to 1000 Hz, and accepts 1–65535 Hz;
zero is rejected. Timer resolution limits the available duty steps at high
frequencies. D3/D9 share TIM1 channel 4, and D7/D15 share TIM3 channel 2: use
only one pin from each pair at a time. GPIO configuration or an ADC read
releases that pin from PWM; a software reset releases all PWM outputs.

Enable `CONFIG_NEOPIXEL` for 800 kHz output through Adafruit_NeoPixel on any
valid GPIO. Set the output pin and byte length before sending SHOW. The default
`CONFIG_NEOPIXEL_BUF_MAX` is 192 bytes (64 RGB or 48 RGBW pixels); hosts provide
the bytes in the strip's color order. Lengths are clamped to that capacity, and
buffer writes preserve only the portion that fits. Storage is allocated once;
`begin()` fails if allocation fails. Software reset clears the buffer and pin
selection. SHOW releases its selected pin from PWM and temporarily masks
interrupts while sending data, delaying I2C service and GPIO-change polling.

Enable `CONFIG_ENCODER` and set `CONFIG_NUM_ENCODERS` from 1 through 4, with
`CONFIG_ENCODER0_A_PIN` / `CONFIG_ENCODER0_B_PIN` and corresponding pairs for
additional encoders. Each input must be a distinct valid GPIO. Inputs start with
pull-ups; connect a mechanical encoder's common terminal to ground. Keep these
pins configured as inputs while using the encoder.

Position and delta are signed 32-bit values. Reading either clears that
encoder's delta. With `CONFIG_INTERRUPT_PIN` configured, enable interrupts per
encoder to assert the shared active-low output while its delta is nonzero;
reading one encoder does not clear another encoder's pending interrupt. Counting
also works without an interrupt pin. `CONFIG_ENCODER_2TICKS` selects the shared
decoder's two-tick mode. Encoders are polled by `run()`, so long NeoPixel shows
or slow application code can miss transitions. Reset clears counts and interrupt
enables and samples each input's current phase.

`examples/example_uart` enables the UART bridge on PA0 (TX) and PA1 (RX),
reserved from GPIO/ADC/PWM while enabled. Use 3.3 V logic on these raw pins.
`CONFIG_UART_RX_PIN`, `CONFIG_UART_TX_PIN`, and `CONFIG_UART_SERCOM` may select
another valid hardware-serial mapping. Keep Serial debug output disabled when
using the bridge. Reset selects 9600 baud and clears buffered data and interrupt
enables; a zero baud command is ignored. RX-ready status is live, and an enabled
RX interrupt stays asserted until the receive buffer is empty. Each I2C DATA
read consumes one byte (0xFF when empty); writes can carry up to 30 bytes with
the default 32-byte command buffer. Long interrupt-masked NeoPixel shows can
lose incoming serial bytes. Check flash usage when combining features.

`examples/example_eeprom` enables `CONFIG_EEPROM` (disabled by default on STM32).
It emulates 256 bytes in the final 2 KiB flash page at `0x08007800`. Reserve that
page when linking/programming: the application, including initialized data,
must fit below it (30 KiB maximum). `begin()` fails if the application overlaps
the page or its 2 KiB RAM workspace cannot be allocated. Writes preserve the
whole page, including bytes outside the 256-byte protocol window.

Each changed write command erases/programs the page once; unchanged writes skip
the erase. Writes are deferred to `run()` and can delay I2C replies while flash
is busy. Allow completion and read back the bytes to check success. Flash has
limited endurance; avoid frequent updates, and do not interrupt power during a
write. Updates are not power-loss atomic. Software reset does not erase data.
Byte `0xFF` holds the persistent base I2C address: values `0x08`–`0x77` take
effect on the next reset before address straps are applied; other values use
`CONFIG_I2C_PERIPH_ADDR`.

### STM32 factory-ROM bootloader

The [C011 I2C bootloading demo](extras/stm32_i2c_bootloader/README.md) includes
a Nano/Uno/Metro USB-to-I2C bridge and a Python updater with flash backup,
readback verification, and EEPROM preservation.

The C011 backend enables `CONFIG_BOOTLOADER` by default. Write exactly these six
bytes in one transaction to the current seesaw I2C address:
`00 7E B0 07 10 AD`. The four-byte key spells "BOOTLOAD" in leetspeak. Shorter
commands, incorrect keys, and trailing bytes are ignored; `00 7E FF` is not
accepted. Normal seesaw software reset remains `00 7F FF`.
Define `CONFIG_BOOTLOADER 0` before including the library to omit ROM entry.

The command requests a clean reset, then enters ST's factory ROM before Arduino
initializes peripherals. No BOOT0 wire or option-byte change is needed. ROM uses
PB6/SCL and PB7/SDA at 7-bit I2C address `0x64` and speaks ST's
[AN4221 protocol](https://www.st.com/resource/en/application_note/an4221-i2c-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf),
not seesaw commands. Use 3.3 V pull-ups on the raw bus; a 5 V host needs a
bidirectional level shifter. The breakout's QT connectors are level shifted.

Preserve the final 2 KiB EEPROM page at `0x08007800`: limit application images
to 30720 bytes, erase only the required application pages, and verify written
bytes before using ROM Go at `0x08000000`. Do not change option bytes or
mass-erase. Keep SWD recovery available if power or an update is interrupted;
this key prevents accidental entry, not unauthorized firmware updates.

## STM32C031 breakout support

The STM32C031G6U6 Rev A breakout uses the 28-pin generic **STM32C031G6Ux**
variant: 32 KiB flash, 12 KiB SRAM, and a 48 MHz peripheral clock.
`examples/example_stm32c031` enables GPIO, ADC, PWM, NeoPixel, one encoder on
PA2/PA3, and an interrupt output on PC14. The provisional hardware ID is `0x91`;
host libraries must recognize that ID before using their normal `begin()`.
The backend supplies the missing 48 MHz `SystemClock_Config()` startup hook
for this generic variant, using the internal oscillator and HAL clock APIs.

Protocol indices follow STM32duino's package-specific digital pin map:

| Protocol indices | Board pins |
| --- | --- |
| 0–8 | PA0–PA8 |
| 9, 10 | PA11, PA12 |
| 13 | PA15 |
| 14, 15 | PB0, PB1 |
| 16–18 | PB3–PB5 |
| 21, 22, 23 | PB8, PC6, PC14 |

These are 20 usable GPIOs before optional UART, interrupt, address, or SPI
reservations. SWD (11/12), I2C (19/20), the PC15 LED (24), reset (25),
and duplicate remap aliases (26/27) are not ordinary GPIOs. PC14 is exposed on
the saved Rev A design. ADC supports 0–10 and 14/15; PWM supports every usable
GPIO except 10. These PWM groups share channels: **3/9**, **5/13**,
**6/17/21/22**, **7/18/23**, and **8/14**. Use only one pin from each group
at a time.

The C011 feature rules above also apply, including the optional SPI controller
with header-configured pins. EEPROM reserves `0x08007800` and limits the
application to 30 KiB.
The same BOOTLOAD key enters the C031 ROM, whose PB6/PB7 I2C address is
**0x63**, not the C011's 0x64; see
[ST AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf).

STM32duino 3.0.0 includes this variant's sources but not its board-menu entry
or linker script. Add the following to `boards.local.txt` beside that core's
`boards.txt`, preserving any existing entries. This uses the core's supplied
G6Ux variant and its existing C031 linker script; no core source changes are
needed. Restart the IDE after adding the entry.

```ini
GenC0.menu.pnum.GENERIC_C031G6UX=Generic C031G6Ux
GenC0.menu.pnum.GENERIC_C031G6UX.upload.maximum_size=32768
GenC0.menu.pnum.GENERIC_C031G6UX.upload.maximum_data_size=12288
GenC0.menu.pnum.GENERIC_C031G6UX.build.board=GENERIC_C031G6UX
GenC0.menu.pnum.GENERIC_C031G6UX.build.product_line=STM32C031xx
GenC0.menu.pnum.GENERIC_C031G6UX.build.variant=STM32C0xx/C031G(4-6)U
GenC0.menu.pnum.GENERIC_C031G6UX.build.ldscript=../C031C(4-6)(T-U)/ldscript.ld
```

Compile with the EEPROM page reserved:

```sh
arduino-cli compile --fqbn "STMicroelectronics:stm32:GenC0:pnum=GENERIC_C031G6UX,xserial=generic,usb=none,opt=oslto,dbg=none,rtlib=nano,upload_method=OpenOCDSTLink" --build-property upload.maximum_size=30720 --library . examples/example_stm32c031
```

The linker uses the 30 KiB application limit and 12 KiB RAM size. This
target-specific example is included in the C031 CI platform matrix.

## STM32C051 breakout support

The STM32C051G8U6 Rev A breakout uses the 28-pin generic **STM32C051G8Ux**
variant with 64 KiB flash, 12 KiB SRAM, and a 48 MHz internal clock. Its GPIO,
ADC, and selected PWM channel map matches the C031 table above. The backend
uses C051-specific alternate-function indices to preserve those PWM channels,
and supplies its missing startup clock hook through HAL. Its provisional
hardware ID is **0x92**, which host libraries must recognize.

`examples/example_stm32c051` enables GPIO, ADC, PWM, NeoPixel, encoder, UART,
SPI, and EEPROM together. UART reserves PA0/PA1; the encoder uses PA2/PA3;
PC14 is the interrupt output. The example selects PA4–PA7 for SPI using the
four `CONFIG_SPI_*_PIN` defines; change them for other valid pin assignments. EEPROM
uses the final 2 KiB page at **0x0800F800**, so reserve **63488 bytes** for the
application. Do not use the C011/C031 EEPROM address or image-size limit.
The bridge exposes one configured SPI bus, not multiple simultaneous buses.
Additional I2C instances on the C051 are not exposed by this backend.

The BOOTLOAD key enters ROM I2C at **0x76** on PB6/PB7. ST's C051 ROM also
initializes its SPI interfaces, so isolate attached SPI peripherals when
using the ROM loader. The ROM's I2C limit is 400 kHz; see
[ST AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf).

As with C031, STM32duino 3.0.0 supplies the G8Ux source files without a menu
entry or linker script. Add this entry to the same `boards.local.txt`:

```ini
GenC0.menu.pnum.GENERIC_C051G8UX=Generic C051G8Ux
GenC0.menu.pnum.GENERIC_C051G8UX.upload.maximum_size=65536
GenC0.menu.pnum.GENERIC_C051G8UX.upload.maximum_data_size=12288
GenC0.menu.pnum.GENERIC_C051G8UX.build.board=GENERIC_C051G8UX
GenC0.menu.pnum.GENERIC_C051G8UX.build.product_line=STM32C051xx
GenC0.menu.pnum.GENERIC_C051G8UX.build.variant=STM32C0xx/C051G(6-8)U
GenC0.menu.pnum.GENERIC_C051G8UX.build.ldscript=../C051C(6-8)(T-U)/ldscript.ld
```

```sh
arduino-cli compile --fqbn "STMicroelectronics:stm32:GenC0:pnum=GENERIC_C051G8UX,xserial=generic,usb=none,opt=oslto,dbg=none,rtlib=nano,upload_method=OpenOCDSTLink" --build-property upload.maximum_size=63488 --library . examples/example_stm32c051
```

This example is included in the C051 CI platform matrix.

## STM32G071 breakout support

The STM32G071GBU6 Rev A breakout uses the standard 28-pin generic
**STM32G071GBUx** target, not the alternate **UxN** pinout. It has 128 KiB
flash and 36 KiB SRAM. STM32duino supplies its 64 MHz clock initialization;
the backend selects matching I2C timing and SPI clock limits. The provisional
hardware ID is **0x93**, which host libraries must recognize.

GPIO numbering and reserved pins match the C031/C051 table. G071 ADC is
limited to indices **0–7 and 14/15**. PWM is available on the exposed GPIOs
except **10/PA12 and 23/PC14**; shared channel groups are **0/5/13**,
**6/17**, and **7/18**. Use one pin from each group at a time. PA4/PA5 use
STM32duino's timer helpers explicitly because ordinary `analogWrite()` would
select their DAC instead. This firmware does not expose a DAC command.

`examples/example_stm32g071` enables GPIO, ADC, PWM, NeoPixel, encoder, UART,
SPI, and EEPROM. Its UART is USART4 on PA0/PA1, encoder is PA2/PA3, and
interrupt output is PC14. The example selects PA4–PA7 through the four
`CONFIG_SPI_*_PIN` defines; SPI clock requests range from 250 kHz through
32 MHz. EEPROM occupies **0x0801F800**, the final 2 KiB page;
the application must fit within **129024 bytes**.

The BOOTLOAD key enters ROM I2C at **0x51** on PB6/PB7. This 28-pin package
requires ROM V11.1 or later. Check the actual ROM version before updating:
ST documents extra stretching and single-page erase precautions for V11.3.
The ROM also initializes SPI, so isolate attached SPI peripherals while
using it. See [ST AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf).

This target is already present in STM32duino 3.0.0's board menu:

```sh
arduino-cli compile --fqbn "STMicroelectronics:stm32:GenG0:pnum=GENERIC_G071GBUX,xserial=generic,usb=none,opt=oslto,dbg=none,rtlib=nano,upload_method=OpenOCDSTLink" --build-property upload.maximum_size=129024 --library . examples/example_stm32g071
```

This example is included in the G071 CI platform matrix.

## I2C-to-SPI controller bridge

`examples/example_spi` enables `CONFIG_SPI` alongside GPIO, ADC, and PWM.
The experimental module at `0x13` supports all four SPI modes, either bit
order, and held chip select across bounded chunks. Use it for e-paper displays
or other SPI devices, with ordinary seesaw GPIOs for D/C, reset, or BUSY.
All raw SPI/control signals use 3.3 V logic.

Define `CONFIG_SPI_CS_PIN`, `CONFIG_SPI_SCK_PIN`, `CONFIG_SPI_MISO_PIN`, and
`CONFIG_SPI_MOSI_PIN` before including the library, as in the example. CS may
use any available GPIO; SCK/MISO/MOSI must map to one hardware SPI peripheral.
The selected pins are reserved while SPI is configured; disabling SPI releases
them. Transfers run from `run()`, not the I2C callback.
Read status and consume received data before sending another chunk. See the
[SPI protocol](SPI_PROTOCOL.md) for registers, framing, clock limits, and errors.

## STM32 audio spectrum

`examples/example_stm32_spectrum` enables the existing `CONFIG_FHT` spectrum
interface on C011, C031, C051, and G071. The STM32 implementation uses the
STM32duino core's bundled CMSIS-DSP Q15 FFT, not the AVR assembly FHT.
ADC1 is triggered by TIM3 and writes 128 evenly spaced samples through DMA1
channel 1. Call `run()` frequently to process and publish frames. There can
be gaps between frames, but I2C interrupts do not change sample spacing
within a frame.

Connect an analog microphone's DC-biased output to PA0, with signal voltage
always between ground and 3.3 V. For the Adafruit SPW2430 breakout, supply
VIN from 3.3 V and use **DC**, not AC. Set `CONFIG_FHT_PIN` before including
the library to choose another initial ADC-capable protocol GPIO index.
The selected input is reserved from ordinary GPIO commands. This mode owns
ADC1, DMA1 channel 1, and TIM3; do not also enable `CONFIG_ADC`. PWM requests
using TIM3 are rejected, while other timers remain available. SPI, encoders,
NeoPixels, and EEPROM may coexist when pins and memory permit. Microphone
channel changes cannot take pins assigned to those features or active PWM.
The C051 and G071 have more flash for feature combinations than C011/C031.
EEPROM also needs a 2 KiB RAM buffer and reserved flash page.
Spectrum plus PWM uses nearly all of the C011's 32 KiB flash, so that
combination cannot retain the final 2 KiB EEPROM page.

NeoPixel output, flash writes, and SPI transfers can delay spectrum processing
and I2C servicing; completed DMA frames remain available. FFT processing also
delays the polling-based encoder decoder, so fast encoder edges can be missed.

The existing spectrum registers are unchanged:

- `CHANNEL` selects a protocol GPIO index, not a raw STM32 ADC channel.
  Invalid, address-strap, interrupt, and UART-reserved pins are ignored.
- `RATE` accepts codes 0..31 (larger values clamp to 31). The sample rate
  is approximately `312500 / (13 + code)` Hz, rounded to a timer
  tick. The default code 12 gives exactly 12,500 Hz.
- Read `RESULTS_LOWER` (32 bytes) followed by `RESULTS_UPPER` (32 bytes).
  LOWER latches all 64 bins; UPPER returns the second half of that frame.
  Bin `k` represents `k * sample_rate / 128` Hz. At the default rate the
  bin spacing is 97.65625 Hz; bin 0 is DC and Nyquist is not returned.

Signals above half the sample rate alias into lower bins. Use an analog
low-pass filter when needed; a piezo driven by a square wave can produce
strong harmonics as well as its fundamental.

Each frame has its measured mean removed, is multiplied by 8 and a Hann
window, and is transformed with CMSIS's 1/128 scaling. Output is a
zero-floored approximation to `16 * log2(magnitude)`, in 8-bit counts,
not calibrated sound pressure or dB SPL. The AVR microphone-specific
noise and peak tables are deliberately not applied to STM32. Digital
scaling does not improve the ADC's analog resolution; a very quiet
microphone may need an external analog amplifier.
