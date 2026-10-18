"""Capture Nano I2C reads, or explicitly use the STEMMA Friend as controller.

The finite capture starts before the host command. It retains the beginning of
that transaction even if a bus timeout delays the host's response. No automatic
retry hides an I2C failure. Raw captures use one nibble per simultaneous sample:
bit 0 = GPIO26, bit 1 = GPIO27, bit 2 = GPIO28, bit 3 = GPIO29; low nibble first.
"""
import argparse
import json
import time
from pathlib import Path

import serial

from bench import Jumperless, Metro


def report(message):
    print(message, end="\n\r", flush=True)


class Capture:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=5, write_timeout=3)
        time.sleep(0.3)
        self.serial.reset_input_buffer()
        report(self.command("I"))

    def line(self):
        data = self.serial.read_until(b"\n\r")
        if not data.endswith(b"\n\r"):
            raise RuntimeError(f"Capture response timed out: {data!r}")
        return data[:-2].decode("ascii")

    def command(self, text):
        self.serial.write(text.encode("ascii") + b"\n")
        result = self.line()
        if result.startswith("ERR"):
            raise RuntimeError(result)
        return result

    def arm(self, rate):
        if self.command(f"A {rate}") != "ARMED":
            raise RuntimeError("Capture did not arm")

    def stop(self):
        result = self.command("S")
        if not result.startswith("STOPPED "):
            raise RuntimeError(result)
        return result

    def download(self):
        header = self.command("D").split()
        if len(header) != 4 or header[0] != "DATA":
            raise RuntimeError(f"Bad capture header: {header}")
        length, rate, flags = map(int, header[1:])
        if length < 4 or length > 196608 or length % 4:
            raise RuntimeError(f"Bad capture size: {length}")
        raw = self.serial.read(length)
        if len(raw) != length or self.line() != "END":
            raise RuntimeError("Incomplete capture download")
        return raw, rate, flags


def decode(raw, base_pin, rate):
    """Return unfiltered I2C edges/bytes from a selected adjacent input pair."""
    shift = base_pin - 26
    previous = None
    active = False
    bits = []
    frames = []
    frame = None
    index = 0
    for packed in raw:
        for sample in (packed & 15, packed >> 4):
            state = (sample >> shift) & 3
            sda, scl = state & 1, state >> 1
            if previous is not None:
                old_sda, old_scl = previous & 1, previous >> 1
                if old_scl and scl and sda != old_sda:
                    if not sda:
                        if active:
                            frame["end"] = "restart"
                            frame["partial_bits"] = bits
                        frame = {"start_s": index / rate, "bytes": [], "acks": []}
                        frames.append(frame)
                        active = True
                        bits = []
                    elif active:
                        frame["end"] = "stop"
                        frame["end_s"] = index / rate
                        frame["partial_bits"] = bits
                        active = False
                        bits = []
                elif active and not old_scl and scl:
                    bits.append(sda)
                    if len(bits) == 9:
                        value = 0
                        for bit in bits[:8]:
                            value = value * 2 + bit
                        frame["bytes"].append(value)
                        frame["acks"].append(bits[8] == 0)
                        bits = []
            previous = state
            index += 1
    if active:
        frame["end"] = "capture_end"
        frame["partial_bits"] = bits
    return frames


def save_capture(capture, destination, details):
    raw, rate, flags = capture.download()
    details.update(sample_rate=rate, byte_count=len(raw), pio_debug_flags=flags)
    details["decoded"] = {str(pin): decode(raw, pin, rate) for pin in (26, 28)}
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.with_suffix(".bin").open("xb") as output:
        output.write(raw)
    with destination.with_suffix(".json").open("x", encoding="utf-8") as output:
        json.dump(details, output, indent=2)
    report(json.dumps(details))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-port", default="COM69")
    parser.add_argument("--nano-port", default="COM65")
    parser.add_argument("--jumperless-port", default="COM11")
    parser.add_argument("--helper", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=500)
    parser.add_argument("--clock", type=int, choices=(100000, 400000), default=100000)
    parser.add_argument("--rate", type=int, default=5000000)
    controller = parser.add_mutually_exclusive_group()
    controller.add_argument("--direct", action="store_true",
                            help="Use Friend GPIO28/29 as the temporary controller")
    controller.add_argument("--physical-nano", action="store_true",
                            help="Use physical Nano A4/A5 jumpers; do not route I2C")
    args = parser.parse_args()
    capture = Capture(args.capture_port)
    nano = None
    fixture = None
    try:
        fixture = Jumperless(args.jumperless_port, args.helper, fixture="nano")
        if args.direct or args.physical_nano:
            # Physical wires bypass the matrix. Refuse a parallel switched path.
            bridges = fixture.existing_routes
            if any(21 in pair or 22 in pair or 5 in pair or 6 in pair for pair in bridges):
                raise RuntimeError("Disconnect routed I2C controllers before direct test")
        if args.direct:
            report("Direct Friend GPIO28/29 controller; Nano disconnected")
        else:
            nano = Metro(args.nano_port)
            fixture.connect_metro_link(args.physical_nano)
            # Existing host C command selects 1 or 4 multiples of 100 kHz.
            report(nano.command("C", [args.clock // 100000]))
        expected_reads = ((1, b"\x90"), (2, b"\x00\x00\x95\x1a"),
                          (3, b"\x00\x0a\x63\x03"))
        for cycle in range(args.cycles):
            for register, expected in expected_reads:
                capture.arm(args.rate)
                details = {"cycle": cycle, "register": register,
                           "bus_clock": args.clock, "expected": expected.hex(),
                           "direct": args.direct, "physical_nano": args.physical_nano}
                try:
                    if args.direct:
                        reply = capture.command(f"T {register} {len(expected)} {args.clock}")
                        if not reply.startswith("DATA "):
                            raise RuntimeError(reply)
                        actual = bytes.fromhex(reply[5:])
                    else:
                        actual = nano.read(0, register, len(expected))
                    details["actual"] = actual.hex()
                    if actual != expected:
                        raise RuntimeError(f"Wrong bytes: {actual.hex()}")
                except Exception as error:
                    details["error"] = str(error)
                    capture.stop()
                    save_capture(capture, args.output / "failure", details)
                    raise
                capture.stop()
                if cycle == 0:
                    save_capture(capture, args.output / f"known-register-{register}", details)
            if cycle % 100 == 0:
                report(f"PASS cycle {cycle} at {args.clock} Hz")
        report(f"PASS {args.cycles} cycles at {args.clock} Hz")
    finally:
        if fixture is not None:
            fixture.close()
        if nano is not None:
            nano.serial.close()
        capture.serial.close()


if __name__ == "__main__":
    main()
