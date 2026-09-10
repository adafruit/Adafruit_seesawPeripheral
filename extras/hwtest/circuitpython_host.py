"""Run the real CircuitPython host library via a byte-preserving Metro I2C adapter.

This exercises library code on CPython, not a native CircuitPython firmware build.
No NeoPixel commands are issued. Only the verified Metro-powered fixture is used.
"""
import argparse
import sys
import time

from bench import Jumperless, Metro, ROWS, check, passed, pwm_capture


class MetroI2C:
    """BusDevice-compatible locking and hardware I2C byte transactions."""

    def __init__(self, metro):
        self.metro = metro
        self.locked = False

    def try_lock(self):
        if self.locked:
            return False
        self.locked = True
        return True

    def unlock(self):
        self.locked = False

    def writeto(self, address, buffer, *, start=0, end=None, stop=True):
        del stop
        payload = bytes(buffer[start:end])
        try:
            if not payload:
                check(self.metro.command("Z", [address]) == "OK", "I2C probe failed")
            else:
                check(2 <= len(payload) <= 32, "Unsupported Metro I2C packet length")
                check(self.metro.command("W", [address, *payload]) == "OK", "I2C write failed")
        except RuntimeError as error:
            raise OSError(str(error)) from error

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        if end is None:
            end = len(buffer)
        count = end - start
        if not count:
            return
        try:
            response = self.metro.command("U", [address, count])
        except RuntimeError as error:
            raise OSError(str(error)) from error
        check(response.startswith("DATA "), "Raw I2C read returned no data")
        payload = bytes.fromhex(response[5:])
        check(len(payload) == count, "Raw I2C read length mismatch")
        buffer[start:end] = payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True)
    parser.add_argument("--metro", required=True)
    parser.add_argument("--jumperless", required=True)
    parser.add_argument("--helper", required=True)
    args = parser.parse_args()
    sys.path.insert(0, args.library)
    from adafruit_seesaw.seesaw import Seesaw
    from adafruit_seesaw.analoginput import AnalogInput
    from adafruit_seesaw.pwmout import PWMOut
    from adafruit_seesaw.spi import SPIBridge

    metro = Metro(args.metro)
    jl = None
    bridge = None
    try:
        jl = Jumperless(args.jumperless, args.helper, metro_link=True)
        jl.connect_metro_link()
        supply = jl.sense(39)
        seesaw = Seesaw(MetroI2C(metro))
        check(seesaw.chip_id == 0x90, "CircuitPython did not recognize C011")
        check(seesaw.pin_mapping.adc_width == 12, "CircuitPython selected the wrong pin map")
        check(seesaw._get_eeprom_i2c_addr() == 255, "CircuitPython EEPROM address-byte mapping is wrong")
        passed("circuitpython_begin_reset_and_mapping")
        for pin in (0, 1, 4, 5, 6, 7, 8, 9, 10):
            row = ROWS[pin]
            try:
                seesaw.pin_mode(pin, seesaw.OUTPUT)
                for level in (False, True, False):
                    seesaw.digital_write(pin, level)
                    time.sleep(0.01)
                    voltage = jl.sense(row)
                    check(seesaw.digital_read(pin) == level, "CircuitPython GPIO readback mismatch")
                    check(voltage > 2.8 if level else abs(voltage) < 0.2, "CircuitPython GPIO voltage mismatch")
                seesaw.pin_mode(pin, seesaw.INPUT)
                jl.exec("dac_set(1, 1.65); time.sleep_ms(50)")
                jl.connect(row, "DAC1")
                reference = jl.sense(row)
                raw = seesaw.analog_read(pin)
                scaled = AnalogInput(seesaw, pin).value
                check(0 <= raw <= 4095 and 0 <= scaled <= 65535 and scaled % 16 == 0,
                      "CircuitPython ADC range/alignment mismatch")
                check(abs(raw * supply / 4095 - reference) < 0.12, "CircuitPython raw ADC voltage mismatch")
                check(abs(scaled * supply / 65520 - reference) < 0.12, "CircuitPython scaled ADC voltage mismatch")
                jl.disconnect(row, "DAC1")
                if pin != 10:
                    jl.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set_pull(GPIO_1, FLOATING); gpio_set_read_floating(GPIO_1, False)")
                    jl.connect(row, "GPIO_1")
                    pwm = PWMOut(seesaw, pin)
                    pwm.frequency = 200
                    pwm.duty_cycle = 16384
                    time.sleep(0.01)
                    frequency, duty = pwm_capture(jl)
                    check(abs(frequency - 200) / 200 < 0.08 and abs(duty - 0.25) < 0.1,
                          "CircuitPython PWM measurement mismatch")
                    jl.disconnect(row, "GPIO_1")
                passed("circuitpython_gpio_adc_pwm", pin=pin, raw12=raw, analoginput16=scaled)
            finally:
                for node in ("DAC1", "GPIO_1"):
                    if (row, node) in jl.routes:
                        jl.disconnect(row, node)
                seesaw.pin_mode(pin, seesaw.INPUT)

        bridge = SPIBridge(seesaw)
        jl.connect(2, 3)
        for mode in range(4):
            for order in (False, True):
                bridge.configure(1000000, mode, order)
                check(0 < bridge.clock_frequency <= 1000000, "CircuitPython SPI clock mismatch")
                for length in (0, 1, 29, 30, 58, 96):
                    tx = bytes((i * 37 + 13) & 255 for i in range(length))
                    rx = bytearray(length)
                    bridge.transfer(tx, rx)
                    check(rx == tx, "CircuitPython SPI physical loopback mismatch")
                passed("circuitpython_spi_modes_and_chunks", mode=mode, lsb_first=order)
        rx = bytearray(96)
        bridge.transfer(rx=rx)
        check(rx == bytes([255]) * len(rx), "CircuitPython SPI receive-only mismatch")
        bridge.transfer(bytes(range(96)), end=False)
        check(abs(jl.sense(5)) < 0.2, "CircuitPython SPI did not hold CS")
        bridge.transfer(bytes(range(96)), begin=False)
        check(jl.sense(5) > 2.8, "CircuitPython SPI did not release CS")
        try:
            bridge.transfer(b"x", begin=False)
        except RuntimeError as error:
            check("error 2" in str(error), "CircuitPython SPI reported an unexpected failure")
        else:
            raise AssertionError("CircuitPython SPI accepted an invalid continuation")
        bridge.configure()
        bridge.transfer(b"recovered")
        passed("circuitpython_spi_receive_write_cs_error_recovery")
        passed("circuitpython_host_complete", runtime="CPython via physical Metro I2C")
    finally:
        try:
            if bridge is not None:
                bridge.abort()
        finally:
            try:
                if jl is not None:
                    jl.close()
            finally:
                metro.serial.close()


if __name__ == "__main__":
    main()
