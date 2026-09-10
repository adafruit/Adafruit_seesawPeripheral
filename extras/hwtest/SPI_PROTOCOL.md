# Experimental I2C-to-SPI controller bridge

This is a new, provisional seesaw module at **0x13**, advertised by option bit
19. It is not the older SPI-slave transport. Enable `CONFIG_SPI=1` before
including the peripheral header, or build the engineering `spi` target.
UART remains disabled in that target.

C011 uses PA4 (index 4) for active-low CS, PA5 (5) for SCK, PA6 (6) for MISO,
and PA7 (7) for MOSI. Successful configuration reserves all four pins until
disabled. Conflicting encoder/NeoPixel assignments reject configuration.
Other GPIOs can provide display D/C, reset, and BUSY. The software has been
tested with electrical loopback and an independent AVR receiver, not an
actual e-paper panel or a drop-in display-library integration.

All multi-byte integers are big-endian. A register transaction starts with the
module and register bytes. The maximum complete I2C write is 32 bytes.

| Register | Write | Read |
|---|---|---|
| 0x00 STATUS | None | Four bytes: flags, error, RX length, completion sequence |
| 0x01 CONFIG | Mode byte (0..3), order byte (0 MSB / 1 LSB), requested Hz (4 bytes) | The six requested configuration bytes |
| 0x02 TRANSFER | Flags byte followed by 0..29 transmit bytes | Not used |
| 0x03 READ | None | Consume the completed RX buffer; read exactly STATUS RX length |
| 0x04 ABORT | One byte: 0 releases CS but retains SPI; 1 also disables SPI and releases pins | Not used |
| 0x05 CLOCK | None | Actual configured clock in Hz (4 bytes), zero if disabled |

STATUS flags: busy=0x01, configured=0x02, CS selected=0x04, RX ready=0x08,
error=0x80. Completion sequence increments modulo 256 after a successful
TRANSFER, including a zero-byte CS-control command.

TRANSFER flags: BEGIN=0x01 asserts CS, END=0x02 releases CS after the bytes,
DISCARD_RX=0x04 avoids storing received bytes. A continuation without BEGIN
requires CS already selected. Zero payload bytes allow explicit CS changes.
For a long read/write, wait until not busy, read all RX bytes, then submit the
next chunk with CS held. Write-only chunks can discard RX. There is one
29-byte receive buffer: overwriting unread data is an error, not silent loss.

Clock requests are 187500..24000000 Hz for the tested 48 MHz C011 configuration.
The hardware selects a divisor of 2..256, rounding down (a 1 MHz request yields
750 kHz). CLOCK reports the peripheral setting, not effective frame throughput.
Only 187.5 and 750 kHz have been electrically exercised through this fixture;
the upper clock range is not validated through the Jumperless crossbar.

Errors: 0=none, 1=configuration, 2=transaction state, 3=queue overflow,
4=unread RX, 5=HAL transfer failure, 6=invalid command, 7=pin conflict.
Errors remain visible until ABORT/reset. Overflow aborts CS in the main loop;
status, RX, and clock remain readable even when the command queue is full.
Transfers run outside the I2C callbacks with a 50 ms HAL timeout. Hosts must
poll completion with a bounded timeout and must not assume I2C ACK means that
the SPI command completed successfully.

The Arduino `seesaw_SPI` and CircuitPython `SPIBridge` implementations handle
chunking, completion checks, and best-effort abort on transfer failure. They
are companion development changes, not features of released host libraries.
The `spi_queue` test build inserts a 10 ms servicing delay solely to make
overflow reproducible; do not ship that test configuration.
