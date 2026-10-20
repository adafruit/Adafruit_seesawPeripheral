"""Update an STM32C011 seesaw through the bundled USB-to-I2C bridge.

Without --image, identify the factory ROM and read the application vectors.
With --image, back up flash, update application pages, and verify every byte.
Only C011 Rev Z's ROM command layout is supported. See the adjacent README.
"""

import argparse
import hashlib
import time
from pathlib import Path

import serial

ROM_ADDRESS = 0x64
FLASH_BASE = 0x08000000
FLASH_SIZE = 32768
PAGE_SIZE = 2048
APP_SIZE = FLASH_SIZE - PAGE_SIZE
OPTION_BYTES = 0x1FFF7800
OPTION_SIZE = 128
BOOTLOAD_KEY = bytes.fromhex("B0 07 10 AD")
ACK = 0x79
BUSY = 0x76
# AN4221 command numbers and packet checksums are wire-protocol encodings.
GET = 0x00
GET_ID = 0x02
READ_MEMORY = 0x11
GO = 0x21
WRITE_NO_STRETCH = 0x32
ERASE_NO_STRETCH = 0x45


def report(message):
    print(message, end="\n\r", flush=True)


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


def checksum(data):
    value = 0
    for byte in data:
        value ^= byte  # AN4221 appends the XOR of the packet bytes.
    return bytes(data) + bytes([value])


class SerialBridge:
    """The W/R/U ASCII protocol used by serial_i2c_bridge.ino."""

    def __init__(self, port, address=0x49):
        self.serial = serial.Serial(port, 115200, timeout=3, write_timeout=2)
        self.address = address
        time.sleep(2)  # Opening a classic Nano's USB port resets the bridge.
        self.serial.reset_input_buffer()

    def command(self, operation, values):
        # Commands are LF-delimited; replies use human-readable LF then CR.
        line = operation + " " + " ".join(f"{value:02X}" for value in values) + "\n"
        self.serial.write(line.encode("ascii"))
        answer = self.serial.readline().decode("ascii", errors="replace").strip()
        check(answer and not answer.startswith("ERR"),
              f"{line.strip()}: {answer or 'serial timeout'}")
        return answer

    def write(self, base, register, data):
        check(self.command("W", [self.address, base, register, *data]) == "OK",
              "Seesaw write failed")

    def read(self, base, register, length):
        answer = self.command("R", [self.address, base, register, length])
        check(answer.startswith("DATA "), answer)
        data = bytes.fromhex(answer[5:])
        check(len(data) == length, "Short seesaw response")
        return data

    def reset(self):
        self.write(0, 0x7F, [0xFF])
        time.sleep(0.1)


class Bootloader:
    def __init__(self, bridge):
        self.bridge = bridge

    def write(self, data):
        check(2 <= len(data) <= 32, "Packet exceeds the AVR Wire buffer")
        check(self.bridge.command("W", [ROM_ADDRESS, *data]) == "OK",
              "ROM I2C write failed")

    def read(self, length):
        check(1 <= length <= 32, "Read exceeds the AVR Wire buffer")
        answer = self.bridge.command("U", [ROM_ADDRESS, length])
        check(answer.startswith("DATA "), answer)
        data = bytes.fromhex(answer[5:])
        check(len(data) == length, "Short ROM response")
        return data

    def ack(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            result = self.read(1)[0]
            if result == ACK:
                return
            check(result == BUSY, f"ROM returned 0x{result:02X}, expected ACK")
            time.sleep(0.005)
        raise TimeoutError("ROM stayed busy for more than five seconds")

    def command(self, code):
        self.write(bytes([code, code ^ 0xFF]))  # Command and its complement.
        self.ack()

    def identify(self):
        self.command(GET_ID)
        product = self.read(3)
        self.ack()
        check(product == bytes([1, 0x04, 0x43]),
              f"Not an STM32C011: {product.hex()}")
        self.command(GET)
        # C011 Rev Z returns 18 commands even with protocol version 0x11.
        # Read this complete frame together, not as separate one-byte reads.
        data = self.read(20)
        self.ack()
        check(data[0] == 18, "Unsupported C011 ROM command-frame length")
        required = (READ_MEMORY, GO, WRITE_NO_STRETCH, ERASE_NO_STRETCH)
        check(all(code in data[2:] for code in required),
              "ROM is missing required read/go/no-stretch write/erase commands")
        report(f"ROM ID 0x0443, protocol 0x{data[1]:02X}")

    def address(self, address):
        self.write(checksum(address.to_bytes(4, "big")))
        self.ack()

    def read_memory(self, address, length):
        result = bytearray()
        while len(result) < length:
            count = min(32, length - len(result))
            self.command(READ_MEMORY)
            self.address(address + len(result))
            self.write(bytes([count - 1, (count - 1) ^ 0xFF]))
            self.ack()
            result.extend(self.read(count))
        return bytes(result)

    def erase(self, length):
        pages = (length + PAGE_SIZE - 1) // PAGE_SIZE
        check(1 <= pages <= 15, "Refusing to erase the EEPROM page")
        self.command(ERASE_NO_STRETCH)
        self.write(checksum((pages - 1).to_bytes(2, "big")))
        self.ack()
        self.write(checksum(b"".join(page.to_bytes(2, "big") for page in range(pages))))
        self.ack()
        report(f"Erased application pages 0 through {pages - 1}; EEPROM excluded")

    def program(self, image):
        check(8 <= len(image) <= APP_SIZE, "Image overlaps reserved EEPROM")
        # C011 programs aligned 64-bit doublewords. A 24-byte data block plus
        # length/checksum fits the Nano's 32-byte Wire buffer.
        padded = image + bytes([0xFF]) * (-len(image) % 8)
        for offset in range(0, len(padded), 24):
            block = padded[offset:offset + 24]
            try:
                self.command(WRITE_NO_STRETCH)
                self.address(FLASH_BASE + offset)
                self.write(checksum(bytes([len(block) - 1]) + block))
                self.ack()
            except Exception as error:
                raise RuntimeError(
                    f"Programming failed at offset 0x{offset:04X}; "
                    "no automatic retry or Go attempted. Use SWD recovery."
                ) from error
            if offset % 3072 == 0:
                report(f"Programmed {offset + len(block)} / {len(padded)} bytes")
        return padded

    def go(self):
        self.command(GO)
        self.address(FLASH_BASE)


def validate_image(image):
    check(8 <= len(image) <= APP_SIZE, "Image must fit below the EEPROM page")
    stack = int.from_bytes(image[:4], "little")
    entry = int.from_bytes(image[4:8], "little")
    check(0x20000000 < stack <= 0x20001800 and stack % 8 == 0,
          "Image has an invalid C011 stack pointer")
    check(entry & 1 and FLASH_BASE <= entry < FLASH_BASE + len(image),
          "Image has an invalid C011 reset vector")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Nano/Uno/Metro USB port")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x49)
    parser.add_argument("--image", type=Path, help="Application-only .bin, at most 30 KiB")
    parser.add_argument("--backup", type=Path, help="New full-flash backup; required with --image")
    parser.add_argument("--go", action="store_true", help="Restart seesaw after successful checks")
    args = parser.parse_args()
    check(8 <= args.address <= 0x77, "Use a non-reserved 7-bit seesaw address")
    image = None
    if args.image:
        image = args.image.read_bytes()
        validate_image(image)
        check(args.backup is not None and not args.backup.exists(),
              "Provide a new --backup filename before any erase")
    report("Adafruit STM32C011 I2C bootloader demo")
    bridge = SerialBridge(args.port, args.address)
    try:
        check(bridge.read(0, 1, 1) == b"\x90", "Expected seesaw C011 hardware ID")
        bridge.write(0, 0x7E, BOOTLOAD_KEY)
        time.sleep(0.5)
        rom = Bootloader(bridge)
        rom.identify()
        report(f"Application vectors: {rom.read_memory(FLASH_BASE, 8).hex()}")
        if image is not None:
            options = rom.read_memory(OPTION_BYTES, OPTION_SIZE)
            report("Reading the full flash backup over I2C")
            original = rom.read_memory(FLASH_BASE, FLASH_SIZE)
            with args.backup.open("xb") as backup:
                backup.write(original)
            check(args.backup.read_bytes() == original, "Backup did not verify")
            report("Backed up all 32768 flash bytes")
            rom.erase(len(image))
            padded = rom.program(image)
            check(rom.read_memory(FLASH_BASE, len(padded)) == padded,
                  "Application readback mismatch; refusing Go")
            check(rom.read_memory(FLASH_BASE + APP_SIZE, PAGE_SIZE) == original[APP_SIZE:],
                  "Reserved EEPROM page changed; refusing Go")
            check(rom.read_memory(OPTION_BYTES, OPTION_SIZE) == options,
                  "Option bytes changed; refusing Go")
            report(f"Verified image SHA256 {hashlib.sha256(image).hexdigest()}")
            report("EEPROM page and option bytes are unchanged")
        if args.go:
            rom.go()
            time.sleep(0.5)
            check(bridge.read(0, 1, 1) == b"\x90", "Seesaw did not restart after ROM Go")
            bridge.reset()
            check(bridge.read(0, 1, 1) == b"\x90", "Seesaw software reset failed after Go")
            report("Seesaw restarted and passed hardware-ID/software-reset checks")
    finally:
        bridge.serial.close()


if __name__ == "__main__":
    main()
