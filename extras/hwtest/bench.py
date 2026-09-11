"""Electrical tests for c011_firmware through metro_host and Jumperless V5.

Requires pyserial and the official Large-Breadboard-Model helper checkout.
The C011 uses Metro power, via QT or the explicitly selected Jumperless link.
Jumperless rail setpoints are never used to power the C011.
"""
import argparse
import ast
import importlib.util
import json
import time
from pathlib import Path

import serial

ROWS = {0: 9, 1: 8, 2: 7, 3: 6, 4: 5, 5: 4, 6: 3, 7: 2, 8: 1,
        9: 31, 10: 32, 15: 37}
GPIO_NAMES = {**{i: f"PA{i}" for i in range(9)}, 9: "PA11", 10: "PA12", 15: "PC14"}
# Metro Mini is in the breadboard, NOT the Nano socket. User anchors:
# USB/VBUS row 30 and D13 row 60. Adafruit Metro Mini Rev C header order gives
# D4..D11 on rows 51..58. These rows carry only pull-low/release test signals.
METRO_ENCODER_ROWS = [(51, 52), (53, 54), (55, 56), (57, 58)]
BASELINE_BRIDGES = {frozenset(pair) for pair in
                    [(72, 137), (73, 138), (139, 106), (38, 100)]}
METRO_LINK_ROUTES = ((30, 40), (17, 35), (18, 36))


class BoundedSerial(serial.Serial):
    """Keep a stalled USB interface from hanging a safety cleanup forever."""

    def flush(self):
        deadline = time.monotonic() + 2
        while self.out_waiting:
            if time.monotonic() >= deadline:
                raise serial.SerialTimeoutException("USB transmit drain timed out")
            time.sleep(0.01)


class Metro:
    def __init__(self, port):
        self.serial = BoundedSerial(port, 115200, timeout=3, write_timeout=2)
        self.address = 0x49
        time.sleep(2)
        self.serial.reset_input_buffer()

    def command(self, operation, values):
        line = operation + " " + " ".join(f"{value:02X}" for value in values) + "\n"
        self.serial.write(line.encode("ascii"))
        answer = self.serial.readline().decode("ascii", errors="replace").strip()
        if answer.startswith("ERR") or not answer:
            raise RuntimeError(f"{line.strip()}: {answer or 'timeout'}")
        return answer

    def write(self, base, reg, data=()):
        answer = self.command("W", [self.address, base, reg, *data])
        if answer != "OK":
            raise RuntimeError(answer)

    def read(self, base, reg, length):
        answer = self.command("R", [self.address, base, reg, length])
        if not answer.startswith("DATA "):
            raise RuntimeError(answer)
        data = bytes.fromhex(answer[5:])
        if len(data) != length:
            raise RuntimeError("Short serial response")
        return data

    def value(self, base, reg, length=4, signed=False):
        return int.from_bytes(self.read(base, reg, length), "big", signed=signed)

    def gpio(self, reg, mask):
        self.write(1, reg, mask.to_bytes(4, "big"))

    def input(self, pin):
        self.gpio(3, 1 << pin)
        self.gpio(12, 1 << pin)

    def reset(self):
        self.write(0, 0x7F, [0xFF])
        time.sleep(0.1)


class Jumperless:
    def __init__(self, port, helper, metro_link=False):
        spec = importlib.util.spec_from_file_location("jumperless_helper", helper)
        self.helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.helper)
        self.serial = BoundedSerial(port, 115200, timeout=0.2, write_timeout=2)
        self.routes = set()
        self.helper.enter_raw_repl(self.serial)
        bridges = ast.literal_eval(self.exec("print([get_bridge(i) for i in range(get_num_bridges())])"))
        self.existing_routes = {frozenset((a, b)) for a, b, *_ in bridges}
        permitted = BASELINE_BRIDGES.copy()
        if metro_link:
            permitted.update(frozenset(pair) for pair in METRO_LINK_ROUTES)
        for a, b, *_ in bridges:
            if frozenset((a, b)) not in permitted:
                raise RuntimeError(f"Unexpected existing route {a}-{b}; inspect before testing")
        self.saved_dac = float(self.exec("print(dac_get(1))"))
        self.exec("bench_saved_state = get_state()")

    def exec(self, code):
        out, error = self.helper.raw_exec(self.serial, code, timeout=8)
        if error.strip():
            raise RuntimeError(error.strip())
        return out.strip()

    def connect(self, a, b):
        self.exec(f"connect({a}, {b}); time.sleep_ms(50)")
        if frozenset((a, b)) not in self.existing_routes:
            self.routes.add((a, b))

    def disconnect(self, a, b):
        self.exec(f"disconnect({a}, {b}); time.sleep_ms(10)")
        self.routes.discard((a, b))

    def voltage(self, channel=0):
        return float(self.exec(f"print(sum(adc_get({channel}) for _ in range(16)) / 16)"))

    def sense(self, row, channel=0):
        self.connect(row, f"ADC{channel}")
        try:
            return self.voltage(channel)
        finally:
            self.disconnect(row, f"ADC{channel}")

    def connect_metro_link(self):
        # PB6/PB7 are FT_f, configured AF open-drain without internal pulls.
        # DS13866 Rev 5 Table 23 note 2 permits this powered 5 V I2C bus.
        source = self.sense(30)
        check(4.7 < source < 5.3, f"Unexpected Metro VBUS {source:.3f} V")
        self.connect(30, 40)
        supply, vin = self.sense(39), self.sense(40)
        check(3.0 < supply < 3.6 and 4.3 < vin < 5.3,
              f"Routed supply outside limits: VDD={supply:.3f}, VIN={vin:.3f}")
        passed("jumperless_link_power", source_V=source, supply_V=supply, vin_V=vin)
        for host, target in ((17, 35), (18, 36)):
            high = self.sense(host)
            check(3.0 < high < 5.3, f"Metro I2C row {host}: {high:.3f} V")
            self.connect(host, target)
            voltage = self.sense(target)
            check(3.0 < voltage < 5.3, f"Connected I2C row {target}: {voltage:.3f} V")
            passed("jumperless_link_i2c_idle", host_row=host, target_row=target, volts=voltage)

    def close(self):
        try:
            # Release our drivers first, then remove only our temporary routes.
            self.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set_dir(GPIO_2, INPUT); gpio_set_dir(GPIO_3, INPUT)")
            self.exec("gpio_set_pull(GPIO_1, LOW); gpio_set_pull(GPIO_2, LOW); gpio_set_pull(GPIO_3, LOW)")
            # Disconnect all signal routes before removing a temporary supply.
            for a, b in sorted(self.routes, key=lambda pair: pair == (30, 40)):
                self.disconnect(a, b)
            self.exec(f"dac_set(1, {self.saved_dac}); time.sleep_ms(50)")
            self.exec("gpio_set_read_floating(GPIO_1, True)")
        finally:
            try:
                self.helper.exit_raw_repl(self.serial)
            finally:
                self.serial.close()


def check(condition, detail):
    if not condition:
        raise AssertionError(detail)


def passed(test, **values):
    print(json.dumps({"result": "PASS", "test": test, **values}), flush=True)


def preflight(metro, jl, args):
    supply = jl.sense(39)
    vin = jl.sense(40)
    check(3.1 < supply < 3.5, f"C011 supply is {supply:.3f} V")
    check(4.4 < vin < 5.5, f"C011 VIN is {vin:.3f} V")
    check(metro.value(0, 1, 1) == 0x90, "Wrong hardware ID")
    passed("preflight", supply_V=supply, vin_V=vin,
           version=hex(metro.value(0, 2)), options=hex(metro.value(0, 3)))
    return supply


def status_test(metro, jl):
    version = metro.value(0, 2)
    options = metro.value(0, 3)
    check(not (options & (1 << 2)), "Default build unexpectedly advertises UART")
    try:
        for speed in (1, 4):
            check(metro.command("C", [speed]) == "OK", "Could not select I2C clock")
            for _ in range(25):
                check(metro.value(0, 1, 1) == 0x90, "Hardware ID changed")
                check(metro.value(0, 2) == version, "Version changed")
                check(metro.value(0, 3) == options, "Options changed")
            # PA0 is available now that the UART bridge is disabled.
            metro.gpio(5, 1)
            metro.gpio(2, 1)
            check(jl.sense(ROWS[0]) > 2.8, "GPIO setup before reset failed")
            metro.reset()
            check(metro.value(0, 1, 1) == 0x90, "Software reset lost I2C response")
            check(metro.value(0, 3) == options, "Software reset changed capabilities")
            # A separate debugger read verified PA0 MODER changed 01->00.
            # The Jumperless weak-pull voltage probe was inconclusive; do not
            # count it as an electrical high-impedance reset test here.
            passed("status_reset", i2c_hz=speed * 100000, read_cycles=25,
                   version=hex(version), options=hex(options), uart_disabled=True)
    finally:
        metro.command("C", [1])
        metro.input(0)


def spi_test(metro, jl, queue_only=False):
    """Hardware MOSI/MISO loopback plus an independent input-only AVR decoder."""
    def status(expected_error=0):
        deadline = time.monotonic() + 1
        while True:
            result = metro.read(0x13, 0, 4)
            if not result[0] & 1:
                check(result[1] == expected_error,
                      f"SPI error {result[1]}, expected {expected_error}: {result.hex()}")
                return result
            check(time.monotonic() < deadline, "SPI completion timed out")

    def configure(mode=0, order=0, frequency=1000000):
        payload = bytes([mode, order]) + frequency.to_bytes(4, "big")
        metro.write(0x13, 1, payload)
        check(status()[0] & 2, "SPI did not become ready")
        check(metro.read(0x13, 1, 6) == payload, "SPI configuration readback mismatch")

    def transfer(payload, flags, receive=True):
        before = status()[3]
        metro.write(0x13, 2, [flags, *payload])
        result = status()
        check(result[3] == (before + 1) & 255, "SPI completion sequence did not advance once")
        if receive and not flags & 4 and payload:
            check(result[2] == len(payload), "SPI receive length mismatch")
            received = metro.read(0x13, 3, len(payload))
            check(received == bytes(payload),
                  f"Physical SPI loopback mismatch: sent={bytes(payload).hex()} received={received.hex()}")
            check(status()[2] == 0, "SPI RX read did not consume the buffer")
        return result

    def abort(disable=False):
        metro.write(0x13, 4, [int(disable)])
        return status()

    check(metro.value(0, 3) & (1 << 19), "SPI build is not loaded")
    if queue_only:
        try:
            configure(0, 0, 187500)
            transfer(b"", 5)
            check(abs(jl.sense(5)) < 0.2, "CS did not start asserted")
            metro.command("C", [4])
            check(metro.command("Q", [16]) == "OK", "SPI burst transport failed")
            status(3)
            check(jl.sense(5) > 2.8, "Queue overflow did not release CS")
            abort(True)
            metro.command("C", [1])
            configure()
            transfer(b"", 7)
            passed("spi_queue_overflow_and_recovery", fault_injection="10 ms main-loop hold")
        finally:
            metro.write(0x13, 4, [1])
            metro.command("C", [1])
        return
    routes = ((2, 3), (4, 60), (2, 58), (5, 57))
    try:
        check(metro.command("S", [0, 0]) == "OK", "Observer setup failed")
        for pin in (4, 5, 6, 7):
            metro.input(pin)
        for route in routes:
            jl.connect(*route)
        pattern = bytes([0, 255, 0x55, 0xAA, 0x96, 0x69] + [(i * 37 + 13) & 255 for i in range(23)])
        for frequency in (187500, 1000000):
            for mode in range(4):
                for order in range(2):
                    configure(mode, order, frequency)
                    actual_clock = metro.value(0x13, 5, 4)
                    check(0 < actual_clock <= frequency, "SPI clock exceeds the requested maximum")
                    idle = jl.sense(4)
                    check(idle > 2.8 if mode & 2 else abs(idle) < 0.2, "SPI clock idle polarity mismatch")
                    check(jl.sense(5) > 2.8, "SPI CS is not released before transfer")
                    check(metro.command("S", [mode, order]) == "OK", "Observer mode setup failed")
                    result = transfer(pattern, 3)
                    check(not result[0] & 4 and jl.sense(5) > 2.8, "SPI CS remained asserted")
                    capture = metro.command("T", []).split()
                    check(capture[0] == "CAP" and int(capture[1], 16) == len(pattern),
                          f"Independent SPI decoder count: {capture[:2]}")
                    check(bytes.fromhex(" ".join(capture[4:])) == pattern,
                          f"Independent SPI decoder mismatch: mode={mode}, order={order}, frequency={frequency}")
                    elapsed_us = (int(capture[3], 16) - int(capture[2], 16)) & 0xFFFFFFFF
                    # Byte timestamps include inter-byte pauses; this is wire
                    # throughput, not an individual SCK-period measurement.
                    effective_bit_rate = (len(pattern) - 1) * 8e6 / elapsed_us
                    check(0 < effective_bit_rate <= actual_clock * 1.08,
                          "Observed SPI throughput exceeds the clock limit")
                    passed("spi_mode_order_loopback_and_decoder", mode=mode, order=order,
                           requested_hz=frequency, actual_hz=actual_clock,
                           effective_bits_per_second=effective_bit_rate, bytes=len(pattern))

        # Identical reconfiguration must work after peripheral deinitialization.
        abort(True)
        configure(0, 0, 1000000)
        transfer(pattern, 3)
        abort(True)
        configure(0, 0, 1000000)
        transfer(pattern, 3)
        passed("spi_reinitialize_same_settings")

        # Hold CS through an e-paper-sized frame, changing D/C on independent PA0.
        metro.gpio(2, 1)
        frame_length = 4736
        started = time.monotonic()
        for offset in range(0, frame_length, 29):
            payload = bytes(((offset + i) * 73 + 19) & 255 for i in range(min(29, frame_length - offset)))
            first, last = offset == 0, offset + len(payload) == frame_length
            result = transfer(payload, int(first) | (int(last) << 1))
            check(bool(result[0] & 4) == (not last), "SPI CS continuity status mismatch")
            if first:
                check(abs(jl.sense(5)) < 0.2, "CS did not physically stay low between chunks")
                for level in (0, 1, 0):
                    metro.gpio(5 if level else 6, 1)
                    voltage = jl.sense(9)
                    check(voltage > 2.8 if level else abs(voltage) < 0.2,
                          "Independent e-paper D/C GPIO did not change")
                    check(abs(jl.sense(5)) < 0.2, "D/C change interrupted CS")
        check(jl.sense(5) > 2.8, "Frame end did not release CS")
        passed("spi_chunked_epaper_frame", bytes=frame_length, elapsed_s=time.monotonic() - started)
        metro.input(0)

        # Write-only chunks do not require draining receive data.
        transfer(pattern, 5)
        transfer(pattern, 6)
        check(status()[2] == 0, "Write-only SPI unexpectedly retained RX data")
        passed("spi_write_only_chunks")

        # Unread RX must stop, rather than overwrite a previous result.
        transfer(pattern, 1, receive=False)
        metro.write(0x13, 2, [2, 0xAA])
        status(4)
        check(jl.sense(5) > 2.8, "Unread-data error did not release CS")
        abort()
        metro.write(0x13, 2, [8])
        status(6)
        abort()
        metro.write(0x13, 1, [4, 0, 0, 0, 0, 0])
        status(1)
        abort(True)
        metro.write(0x13, 2, [7, 0x55])
        status(2)
        abort(True)
        passed("spi_invalid_commands_and_unread_rx_recovery")

        abort(True)
        for pin in (4, 5, 6, 7):
            metro.input(pin)
        passed("spi_complete")
    finally:
        try:
            metro.write(0x13, 4, [1])
            metro.command("X", [])
            metro.command("C", [1])
            metro.input(0)
        finally:
            for route in routes:
                if route in jl.routes:
                    jl.disconnect(*route)


def spi_host_test(metro, jl):
    """Use the actual Arduino seesaw_SPI class and its automatic chunking."""
    try:
        metro.command("X", [])
        jl.connect(2, 3)
        for mode in range(4):
            for order in range(2):
                response = metro.command("J", [0, mode, order, *int(1000000).to_bytes(4, "big")])
                check(response.startswith("CLOCK "), f"SPI host begin failed: {response}")
                actual = int(response.split()[1], 16)
                check(0 < actual <= 1000000, "Host actual SPI clock is invalid")
                for length in (0, 1, 29, 30, 58, 96):
                    data = metro.command("J", [1, 3, length]).split()
                    check(data[0] == "DATA", f"Host SPI transfer failed: {data}")
                    check(bytes.fromhex(" ".join(data[1:])) == bytes((i * 37 + 13) & 255 for i in range(length)),
                          "Host auto-chunked SPI loopback mismatch")
                check(jl.sense(5) > 2.8, "Host SPI left CS asserted")
                passed("arduino_spi_host_chunk_boundaries", mode=mode, order=order, lengths=[0, 1, 29, 30, 58, 96])
        response = metro.command("J", [5, 3, 96]).split()
        check(response[0] == "DATA" and bytes.fromhex(" ".join(response[1:])) == bytes([255]) * 96,
              "Host receive-only fill bytes did not loop back")
        check(metro.command("J", [2, 1, 96]) == "OK", "Host write-only begin failed")
        check(abs(jl.sense(5)) < 0.2, "Host did not keep CS low between API calls")
        check(metro.command("J", [2, 2, 96]) == "OK", "Host write-only end failed")
        check(jl.sense(5) > 2.8, "Host did not release CS at the end")
        # A continuation without a prior begin must fail and retain its cause.
        check(metro.command("J", [1, 2, 1]) == "FAIL 2", "Host did not report bad transaction state")
        check(metro.command("J", [4]) == "LAST 2", "Host cleanup lost the original error")
        response = metro.command("J", [0, 0, 0, *int(1000000).to_bytes(4, "big")])
        check(response.startswith("CLOCK "), "Host did not recover after the failure")
        check(metro.command("J", [2, 3, 96]) == "OK", "Host recovery transfer failed")
        passed("arduino_spi_host_receive_write_only_cs_and_error_recovery")
    finally:
        try:
            metro.command("J", [3])
        finally:
            if (2, 3) in jl.routes:
                jl.disconnect(2, 3)


def host_test(metro, jl, supply):
    """Exercise the real Arduino host APIs, with independent pin measurements."""
    def host(values):
        response = metro.command("H", values)
        check(response.startswith("VALUE "), f"Unexpected host response: {response}")
        return int(response.split()[1], 16)

    check(metro.command("H", [0, metro.address]) == "OK", "Host begin/reset failed")
    check(host([7]) == 0xFF, "Host chose the wrong EEPROM I2C-address location")
    passed("arduino_host_begin_reset", address=hex(metro.address), address_byte=255)
    # Normal one-encoder build reserves PA2/PA3 and PC14.
    for pin in (0, 1, 4, 5, 6, 7, 8, 9, 10):
        row = ROWS[pin]
        try:
            host([2, pin, 1])  # OUTPUT
            for level in (0, 1, 0):
                host([3, pin, level])
                voltage = jl.sense(row)
                check(host([4, pin]) == level, "Host digitalRead mismatch")
                check(voltage > 2.8 if level else abs(voltage) < 0.2,
                      f"Host GPIO {pin} physical level {voltage:.3f} V")
            passed("arduino_host_gpio", pin=GPIO_NAMES[pin])
            host([2, pin, 0])  # INPUT
            jl.exec("dac_set(1, 0.2); time.sleep_ms(50)")
            jl.connect(row, "DAC1")
            for target in (0.2, 1.65, 3.0):
                jl.exec(f"dac_set(1, {target}); time.sleep_ms(50)")
                reference = jl.sense(row)
                adc_response = metro.command("H", [1, pin]).split()
                check(len(adc_response) == 4 and adc_response[0] == "VALUE" and adc_response[2] == "RAW",
                      "Host did not return its same-conversion ADC comparison")
                value, raw = int(adc_response[1], 16), int(adc_response[3], 16)
                check(0 <= value <= 1023, "Host analogRead is not 10-bit")
                check(abs(value * supply / 1023 - reference) < 0.12,
                      f"Host ADC {pin}: {value} vs {reference:.3f} V")
                check(value == raw >> 2,
                      f"Host ADC normalization mismatch: pin={pin}, host10={value}, raw12={raw}, reference={reference:.3f} V")
                passed("arduino_host_adc", pin=GPIO_NAMES[pin], value=value, raw12=raw, volts=reference)
            jl.disconnect(row, "DAC1")
            if pin != 10:
                jl.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set_pull(GPIO_1, FLOATING); gpio_set_read_floating(GPIO_1, False)")
                jl.connect(row, "GPIO_1")
                for frequency, value, width, expected in ((200, 64, 8, 64 / 255), (500, 32768, 16, 0.5)):
                    host([6, pin, frequency >> 8, frequency & 255])
                    host([5, pin, value >> 8, value & 255, width])
                    measured, duty = pwm_capture(jl)
                    check(abs(measured - frequency) / frequency < 0.08, "Host PWM frequency mismatch")
                    check(abs(duty - expected) < 0.1, "Host PWM duty mismatch")
                    passed("arduino_host_pwm", pin=GPIO_NAMES[pin], hz=measured, duty=duty, width=width)
                jl.disconnect(row, "GPIO_1")
        finally:
            for node in ("DAC1", "GPIO_1"):
                if (row, node) in jl.routes:
                    jl.disconnect(row, node)
            host([2, pin, 0])


def gpio_test(metro, jl, pins):
    for pin in pins:
        row, bit = ROWS[pin], 1 << pin
        metro.input(pin)
        jl.connect(row, "ADC0")
        try:
            metro.gpio(2, bit)
            for level in (0, 1, 0):
                metro.gpio(5 if level else 6, bit)
                volts = jl.voltage()
                check(volts > 2.8 if level else abs(volts) < 0.2,
                      f"{GPIO_NAMES[pin]} output {level}: {volts:.3f} V")
                check(bool(metro.value(1, 4) & bit) == bool(level), "Output readback mismatch")
                passed("gpio_output", pin=GPIO_NAMES[pin], level=level, volts=volts)
            metro.gpio(7, bit)
            check(jl.voltage() > 2.8, "GPIO toggle did not go high")
            metro.input(pin)
            for high in (True, False):
                metro.gpio(5 if high else 6, bit)
                metro.gpio(11, bit)
                volts = jl.voltage()
                check(volts > 2.4 if high else abs(volts) < 0.3,
                      f"{GPIO_NAMES[pin]} pull {high}: {volts:.3f} V")
                passed("gpio_pull", pin=GPIO_NAMES[pin], pull="up" if high else "down", volts=volts)
                metro.gpio(12, bit)
            # Only 3.3 V Jumperless GPIO drives the C011 input.
            jl.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set_pull(GPIO_1, FLOATING)")
            jl.connect(row, "GPIO_1")
            for level in (0, 1, 0):
                jl.exec(f"gpio_set(GPIO_1, {level}); gpio_set_dir(GPIO_1, OUTPUT); time.sleep_ms(10)")
                volts = jl.voltage()
                check(bool(metro.value(1, 4) & bit) == bool(level), "Input readback mismatch")
                check(volts > 2.8 if level else abs(volts) < 0.2, "Input stimulus voltage wrong")
                passed("gpio_input", pin=GPIO_NAMES[pin], level=level, volts=volts)
            jl.exec("gpio_set_dir(GPIO_1, INPUT)")
            jl.disconnect(row, "GPIO_1")
        finally:
            jl.exec("gpio_set_dir(GPIO_1, INPUT)")
            if (row, "GPIO_1") in jl.routes:
                jl.disconnect(row, "GPIO_1")
            jl.disconnect(row, "ADC0")
            metro.input(pin)


def adc_test(metro, jl, pins, supply):
    for pin in (p for p in pins if p != 15):
        row = ROWS[pin]
        metro.input(pin)
        jl.exec("dac_set(1, 0.2); time.sleep_ms(50)")
        jl.connect(row, "ADC0")
        jl.connect(row, "DAC1")
        try:
            last = -1
            for target in (0.2, 0.8, 1.65, 2.5, 3.0):
                jl.exec(f"dac_set(1, {target}); time.sleep_ms(50)")
                measured = jl.voltage()
                check(abs(measured - target) < 0.12, "DAC stimulus does not match setpoint")
                readings = [metro.value(9, 7 + pin, 2) for _ in range(4)]
                check(metro.value(9, 0, 1) == 0, "ADC returned error status")
                count = sum(readings) / len(readings)
                volts = count * supply / 4095
                check(abs(volts - measured) < 0.12, f"{GPIO_NAMES[pin]} ADC {volts:.3f} vs {measured:.3f} V")
                check(count > last, "ADC sweep is not increasing")
                last = count
                passed("adc", pin=GPIO_NAMES[pin], count=count, adc_V=volts, reference_V=measured)
        finally:
            jl.disconnect(row, "DAC1")
            jl.disconnect(row, "ADC0")
            metro.input(pin)


def pwm_capture(jl):
    out = jl.exec("""
edges = []
previous = bool(gpio_get(GPIO_1))
start = time.ticks_us()
while len(edges) < 24 and time.ticks_diff(time.ticks_us(), start) < 300000:
    current = bool(gpio_get(GPIO_1))
    if current != previous:
        edges.append((time.ticks_us(), current))
        previous = current
print(edges)
""")
    edges = ast.literal_eval(out)
    highs, periods = [], []
    last_rise = None
    for stamp, high in edges:
        if high:
            if last_rise is not None:
                periods.append((stamp - last_rise) % (1 << 30))
            last_rise = stamp
        elif last_rise is not None:
            highs.append((stamp - last_rise) % (1 << 30))
    check(len(periods) >= 5 and len(highs) >= 5, "Insufficient PWM edges")
    period = sum(periods) / len(periods)
    return 1e6 / period, (sum(highs) / len(highs)) / period


def pwm_test(metro, jl, pins):
    for pin in (p for p in pins if p != 10):
        row = ROWS[pin]
        metro.input(pin)
        jl.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set_pull(GPIO_1, FLOATING); gpio_set_read_floating(GPIO_1, False)")
        jl.connect(row, "GPIO_1")
        try:
            for frequency, duty in ((200, 0.25), (200, 0.75), (500, 0.5)):
                metro.write(8, 2, [pin, frequency >> 8, frequency & 255])
                value = round(duty * 65535)
                metro.write(8, 1, [pin, value >> 8, value & 255])
                check(metro.value(8, 0, 1) == 0, "PWM returned error status")
                measured_hz, measured_duty = pwm_capture(jl)
                check(abs(measured_hz - frequency) < frequency * 0.08, "PWM frequency mismatch")
                check(abs(measured_duty - duty) < 0.10, "PWM duty mismatch")
                passed("pwm", pin=GPIO_NAMES[pin], expected_Hz=frequency,
                       measured_Hz=measured_hz, expected_duty=duty, measured_duty=measured_duty)
        finally:
            jl.disconnect(row, "GPIO_1")
            metro.input(pin)


def irq_test(metro, jl):
    pin, row, bit = 4, ROWS[4], 1 << 4
    metro.input(pin)
    jl.exec("gpio_set_dir(GPIO_1, INPUT); gpio_set(GPIO_1, 0); gpio_set_pull(GPIO_1, FLOATING)")
    jl.connect(row, "GPIO_1")
    jl.connect(37, "ADC0")
    try:
        jl.exec("gpio_set_dir(GPIO_1, OUTPUT); time.sleep_ms(10)")
        metro.value(1, 10)
        check(jl.voltage() > 2.4, "IRQ not released initially")
        metro.gpio(8, bit)
        for level in (1, 0):
            jl.exec(f"gpio_set(GPIO_1, {level}); time.sleep_ms(20)")
            check(abs(jl.voltage()) < 0.2, "IRQ did not assert")
            flags = metro.value(1, 10)
            check(flags == bit, f"Wrong IRQ flags: {flags:#x}")
            check(jl.voltage() > 2.4, "IRQ did not release after flags read")
            passed("gpio_irq", input_level=level, flags=hex(flags))
        metro.gpio(9, bit)
        jl.exec("gpio_set(GPIO_1, 1); time.sleep_ms(20)")
        check(jl.voltage() > 2.4 and metro.value(1, 10) == 0, "Disabled interrupt still asserted")
        passed("gpio_irq_disable")
    finally:
        jl.exec("gpio_set_dir(GPIO_1, INPUT)")
        jl.disconnect(row, "GPIO_1")
        jl.disconnect(37, "ADC0")
        metro.gpio(9, bit)


def encoder_test(metro, jl, count):
    # Metro D4..D11 are pull-low/release only, on breadboard rows 51..58.
    # The separate Nano socket's existing D2/D3 OLED routes stay untouched.
    encoder_pins = [(2, 3), (4, 5), (6, 7), (9, 10)][:count]
    connections = []
    check(metro.command("P", [15, 3]) == "OK", "Metro did not release encoder pins")
    try:
        for index, pair in enumerate(encoder_pins):
            for channel, pin in enumerate(pair):
                row, node = ROWS[pin], METRO_ENCODER_ROWS[index][channel]
                volts = jl.sense(row, 1)
                check(2.4 < volts < 3.6, "Encoder pull-up missing before connecting Metro")
                jl.connect(row, node)
                connections.append((row, node))
            # Prove both physical signal paths and their voltage domains first.
            for phase in (1, 2, 3):
                check(metro.command("P", [1 << index, phase]) == "OK", "Metro phase failed")
                time.sleep(0.02)
                for channel, pin in enumerate(pair):
                    volts = jl.sense(ROWS[pin], 1)
                    released = bool(phase & (1 << channel))
                    check(2.4 < volts < 3.6 if released else abs(volts) < 0.2,
                          f"Encoder {index} channel {channel}: {volts:.3f} V")
                    check(bool(metro.value(1, 4) & (1 << pin)) == released,
                          "Encoder input does not see the Metro stimulus")
            passed("encoder_electrical", encoder=index, metro_pins=[4 + index * 2, 5 + index * 2])

        jl.connect(37, "ADC0")
        connections.append((37, "ADC0"))
        for index in range(count):
            metro.write(0x11, 0x30 + index, [0, 0, 0, 0])
        for index in range(count):
            metro.write(0x11, 0x10 + index)
            check(jl.voltage() > 2.4, "Encoder IRQ not released initially")
            check(metro.command("E", [1 << index, 0, 5, 5]) == "OK", "Metro encoder generation failed")
            check(abs(jl.voltage()) < 0.2, "Encoder interrupt did not assert")
            forward = metro.value(0x11, 0x40 + index, signed=True)
            check(forward == -5, f"Encoder {index} direction/count mismatch: {forward}")
            check(metro.value(0x11, 0x40 + index, signed=True) == 0, "Delta did not clear on read")
            check(metro.value(0x11, 0x30 + index, signed=True) == -5, "Position mismatch")
            check(jl.voltage() > 2.4, "Encoder delta read did not clear IRQ")
            check(metro.command("E", [1 << index, 1, 3, 5]) == "OK", "Reverse generation failed")
            check(metro.value(0x11, 0x30 + index, signed=True) == -2, "Reverse count mismatch")
            check(metro.value(0x11, 0x40 + index, signed=True) == 0, "Position read did not clear delta")
            check(jl.voltage() > 2.4, "Position read did not release IRQ")
            metro.write(0x11, 0x20 + index)
            check(metro.command("E", [1 << index, 1, 1, 5]) == "OK", "Disabled-IRQ stimulus failed")
            check(jl.voltage() > 2.4, "Disabled encoder IRQ asserted")
            check(metro.value(0x11, 0x30 + index, signed=True) == -1, "Counting stopped with IRQ disabled")
            for other in range(count):
                expected = -1 if other <= index else 0
                check(metro.value(0x11, 0x30 + other, signed=True) == expected,
                      f"Encoder {index} changed unrelated encoder {other}")
            passed("encoder", encoder=index, forward=forward, reverse_steps=3, irq_disable=True)

        # All four move together; reading one must not clear another's IRQ.
        for index in range(count):
            metro.write(0x11, 0x30 + index, [0, 0, 0, 0])
            metro.write(0x11, 0x10 + index)
        check(metro.command("E", [(1 << count) - 1, 0, 8, 2]) == "OK", "Combined encoder generation failed")
        for index in range(count):
            check(abs(jl.voltage()) < 0.2, "Unread encoder delta failed to hold IRQ")
            check(metro.value(0x11, 0x40 + index, signed=True) == -8, "Combined encoder count mismatch")
        check(jl.voltage() > 2.4, "Combined encoder IRQ did not clear")
        passed("encoder_combined", encoders=count, detents_each=-8, step_ms=2)
    finally:
        try:
            metro.command("P", [15, 3])
            for index in range(count):
                metro.write(0x11, 0x20 + index)
        finally:
            for a, b in reversed(connections):
                jl.disconnect(a, b)


def uart_test(metro, jl, port):
    # Jumperless UART is 3.3 V; do not use the Metro's 5 V UART.
    jl.connect(9, "UART_RX")
    jl.connect(8, "UART_TX")
    jl.connect(37, "ADC0")
    try:
        with BoundedSerial(port, 9600, timeout=2, write_timeout=2) as uart:
            for baud in (9600, 115200):
                metro.write(2, 4, baud.to_bytes(4, "big"))
                uart.baudrate = baud
                time.sleep(0.1)
                check(metro.value(2, 4) == baud, "UART baud readback mismatch")
                uart.reset_input_buffer()
                message = b"C011 seesaw TX\x00\xA5"
                metro.write(2, 5, message)
                check(uart.read(len(message)) == message, "UART TX bytes mismatch")
                message = b"Jumperless RX\x00\x5A"
                uart.write(message)
                uart.flush()
                time.sleep(0.05)
                check(metro.value(2, 0, 1) & 2, "UART RX-ready missing")
                received = bytes(metro.value(2, 5, 1) for _ in message)
                check(received == message, "UART RX bytes mismatch")
                check(not (metro.value(2, 0, 1) & 2), "UART RX-ready did not clear")
                passed("uart", baud=baud, transmit_bytes=len(message), receive_bytes=len(received))
            metro.write(2, 2, [1])
            check(jl.voltage() > 2.4, "UART IRQ not idle")
            uart.write(b"X")
            uart.flush()
            time.sleep(0.05)
            check(abs(jl.voltage()) < 0.2, "UART RX interrupt did not assert")
            check(metro.value(2, 5, 1) == ord("X"), "UART interrupt byte mismatch")
            check(jl.voltage() > 2.4, "UART interrupt did not release after read")
            metro.write(2, 3, [1])
            uart.write(b"Y")
            uart.flush()
            time.sleep(0.05)
            check(jl.voltage() > 2.4, "Disabled UART interrupt asserted")
            check(metro.value(2, 5, 1) == ord("Y"), "Disabled-IRQ receive failed")
            passed("uart_rx_irq")
    finally:
        jl.disconnect(9, "UART_RX")
        jl.disconnect(8, "UART_TX")
        jl.disconnect(37, "ADC0")


def eeprom_test(metro):
    # Preserve all pre-existing bytes. One batch is one page erase; avoid wear loops.
    offset = 0x20
    saved = bytes(metro.value(13, offset + i, 1) for i in range(8))
    pattern = bytes((value ^ 0xA5) for value in saved)
    try:
        metro.write(13, offset, pattern)
        time.sleep(0.1)
        check(bytes(metro.value(13, offset + i, 1) for i in range(8)) == pattern, "EEPROM write/read mismatch")
        metro.reset()
        check(bytes(metro.value(13, offset + i, 1) for i in range(8)) == pattern, "EEPROM lost data on software reset")
        passed("eeprom_software_reset", bytes_tested=8, power_cycle_test="pending")
    finally:
        metro.write(13, offset, saved)
        time.sleep(0.1)
        check(bytes(metro.value(13, offset + i, 1) for i in range(8)) == saved, "EEPROM restore failed")


def address_test(metro):
    original = metro.address
    saved = metro.value(13, 0xFF, 1)
    temporary = 0x4A if original != 0x4A else 0x4B
    try:
        metro.write(13, 0xFF, [temporary])
        time.sleep(0.1)
        metro.reset()
        metro.address = temporary
        check(metro.value(0, 1, 1) == 0x90, "New I2C address did not respond")
        check(metro.value(13, 0xFF, 1) == temporary, "Address byte not persistent")
        passed("i2c_address_change", temporary=hex(temporary))
    finally:
        metro.write(13, 0xFF, [saved])
        time.sleep(0.1)
        metro.reset()
        metro.address = original
        check(metro.value(0, 1, 1) == 0x90, "Original I2C address was not restored")
        passed("i2c_address_restored", address=hex(original))


def address_strap_test(metro, jl, inverted):
    # Dedicated strap build: PA0..PA3 are address inputs, not encoder channels.
    # Metro D4..D7 only pull low/release; never drive high against the C011.
    connections = []
    saved_address_byte = metro.value(13, 255, 1)
    base = saved_address_byte if 8 <= saved_address_byte <= 0x77 else 0x49
    check(base + 15 <= 0x77, "Base address leaves no room for four strap bits")
    check(metro.command("P", [3, 3]) == "OK", "Metro did not release strap pins")
    try:
        for pin in range(4):
            volts = jl.sense(ROWS[pin], 1)
            check(2.4 < volts < 3.6, "Strap pin pull-up missing before connection")
            jl.connect(ROWS[pin], 51 + pin)
            connections.append((ROWS[pin], 51 + pin))
        for offset in range(16):
            released = offset if inverted else offset ^ 15
            check(metro.command("P", [1, released & 3]) == "OK", "Strap 0/1 stimulus failed")
            check(metro.command("P", [2, released >> 2]) == "OK", "Strap 2/3 stimulus failed")
            for pin in range(4):
                volts = jl.sense(ROWS[pin], 1)
                check(2.4 < volts < 3.6 if released & (1 << pin) else abs(volts) < 0.2,
                      f"Address strap {pin} stimulus is {volts:.3f} V")
            metro.reset()
            metro.address = base + offset
            check(metro.value(0, 1, 1) == 0x90, "Wrong address after strap reset")
            check(metro.value(13, 255, 1) == saved_address_byte, "Straps changed persistent address byte")
            passed("address_straps", offset=offset, address=hex(metro.address), inverted=inverted)
    finally:
        try:
            metro.command("P", [3, 3])
            metro.reset()
            metro.address = base + (15 if inverted else 0)
            check(metro.value(0, 1, 1) == 0x90, "Strap idle address not restored")
        finally:
            for a, b in reversed(connections):
                jl.disconnect(a, b)


def neopixel_test(metro, count):
    check(1 <= count <= 64, "The configured 192-byte buffer holds 1-64 RGB pixels")
    length = count * 3
    metro.write(14, 1, [8]) # PA8, Jumperless row 1.
    metro.write(14, 2, [1]) # 800 kHz.
    metro.write(14, 3, [length >> 8, length & 255])
    try:
        for name, grb in (("red", (0, 12, 0)), ("green", (12, 0, 0)),
                          ("blue", (0, 0, 12)), ("white", (8, 8, 8))):
            data = bytes(grb) * count
            for offset in range(0, length, 24):
                metro.write(14, 4, [offset >> 8, offset & 255, *data[offset:offset + 24]])
            metro.write(14, 5)
            check(metro.value(14, 0, 1) == 0, "NeoPixel command failed")
            print(json.dumps({"result": "OBSERVATION_REQUIRED", "test": "neopixel",
                              "expected_color": name, "pixels": count}), flush=True)
            time.sleep(3)
    finally:
        for offset in range(0, length, 24):
            metro.write(14, 4, [offset >> 8, offset & 255, *([0] * min(24, length - offset))])
        metro.write(14, 5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metro", required=True)
    parser.add_argument("--jumperless", required=True)
    parser.add_argument("--uart", help="Jumperless UART-passthrough serial port")
    parser.add_argument("--helper", required=True)
    parser.add_argument("--phase", choices=["preflight", "host", "spi", "spi_queue", "spi_host", "status", "gpio", "adc", "pwm", "irq", "encoder", "uart", "eeprom", "address", "straps", "neopixel", "all"], default="preflight")
    parser.add_argument("--jumperless-link", action="store_true", help="Use verified Metro VBUS/I2C breadboard routes")
    parser.add_argument("--keep-link", action="store_true", help="Keep the powered Metro link after successful preflight for SWD programming")
    parser.add_argument("--pixels", type=int, help="RGB ring pixel count; only after wiring is verified")
    parser.add_argument("--gpio-build", action="store_true", help="Use the UART/encoder/IRQ-disabled build to test every GPIO")
    parser.add_argument("--encoders", type=int, choices=[1, 2, 3, 4], default=1,
                        help="Match the flashed encoder count; uses Metro D4..D11")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x49,
                        help="Initial DUT I2C address (for example 0x58 for inverted straps)")
    parser.add_argument("--straps-inverted", action="store_true",
                        help="Match the dedicated active-high address-strap build")
    args = parser.parse_args()
    check(not args.keep_link or (args.jumperless_link and args.phase == "preflight"),
          "--keep-link is only allowed with --jumperless-link --phase preflight")
    metro = Metro(args.metro)
    metro.address = args.address
    jl = None
    try:
        jl = Jumperless(args.jumperless, args.helper, args.jumperless_link)
        if args.jumperless_link:
            jl.connect_metro_link()
        supply = preflight(metro, jl, args)
        if args.phase == "preflight":
            if args.keep_link:
                jl.routes.difference_update(METRO_LINK_ROUTES)
                passed("metro_link_retained_for_swd")
            return
        options = metro.value(0, 3)
        check(options & 0x6303 == 0x6303, "Full GPIO/ADC/PWM/EEPROM/NeoPixel firmware is not loaded")
        pins = list(ROWS) if args.gpio_build else list(range(2, 11))
        metro.reset()
        phases = [args.phase]
        if args.phase == "all":
            phases = ["gpio", "adc", "pwm", "eeprom", "address"]
            if not args.gpio_build:
                phases += ["irq", "encoder"]
                if options & (1 << 2):
                    phases += ["uart"]
        for phase in phases:
            jl.exec(f"oled_clear(); oled_print('C011 bench test'); oled_print('{phase}')")
            if phase == "status": status_test(metro, jl)
            elif phase == "host": host_test(metro, jl, supply)
            elif phase == "spi": spi_test(metro, jl)
            elif phase == "spi_queue": spi_test(metro, jl, queue_only=True)
            elif phase == "spi_host": spi_host_test(metro, jl)
            elif phase == "gpio": gpio_test(metro, jl, pins)
            elif phase == "adc": adc_test(metro, jl, pins, supply)
            elif phase == "pwm": pwm_test(metro, jl, pins)
            elif phase == "irq": irq_test(metro, jl)
            elif phase == "encoder":
                check(options & (1 << 17), "Encoder module not enabled")
                metro.reset()
                encoder_test(metro, jl, args.encoders)
            elif phase == "uart":
                check(options & (1 << 2), "UART module not enabled")
                check(args.uart is not None, "Specify the Jumperless UART-passthrough port")
                uart_test(metro, jl, args.uart)
            elif phase == "eeprom": eeprom_test(metro)
            elif phase == "address": address_test(metro)
            elif phase == "straps": address_strap_test(metro, jl, args.straps_inverted)
            elif phase == "neopixel":
                check(args.pixels is not None, "Specify the verified RGB ring pixel count")
                neopixel_test(metro, args.pixels)
        if "neopixel" in phases:
            print(json.dumps({"result": "OBSERVATION_REQUIRED", "phases": phases,
                              "detail": "Commands sent; confirm the ring colors visually"}), flush=True)
        else:
            passed("selected_phases_complete", phases=phases)
    finally:
        if jl is not None: jl.close()
        metro.serial.close()


if __name__ == "__main__":
    main()
