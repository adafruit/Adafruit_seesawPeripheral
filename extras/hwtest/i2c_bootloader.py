"""STM32C011 ROM updater through the existing Nano/Metro serial I2C bridge.

ST AN2606 (C011) and AN4221 define the address, frames and checksums. This
chip-specific updater never writes option bytes or the final EEPROM page.
Without --image it only identifies the ROM and reads the application vectors.
"""
import argparse
import hashlib
import time
from pathlib import Path

from bench import Metro, check


ROM_ADDRESS = 0x64
FLASH_BASE = 0x08000000
FLASH_SIZE = 32768
PAGE_SIZE = 2048
APP_SIZE = FLASH_SIZE - PAGE_SIZE
OPTION_BYTES = 0x1FFF7800  # RM0490 option-byte flash mirror, not peripheral registers.
OPTION_SIZE = 128  # RM0490 Table 2: C011 option area ends at 0x1FFF787F.
ACK = 0x79
BUSY = 0x76


def report(message):
    print(message, end="\n\r", flush=True)


def checksum(data):
    value = 0
    for byte in data:
        value ^= byte
    return bytes(data) + bytes([value])


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
        self.write(bytes([code, code ^ 0xFF]))
        self.ack()

    def identify(self):
        self.command(0x02)  # Get ID: count, then the two-byte product ID.
        product = self.read(3)
        self.ack()
        check(product == bytes([1, 0x04, 0x43]),
              f"Not an STM32C011: {product.hex()}")
        self.command(0x00)
        # C011 Rev Z returns 18 commands even with protocol version 0x11.
        # Read the complete data frame in one transaction, not byte by byte.
        # ST clarification: https://community.st.com/stm32-mcus-products-25/
        # stm32c011-rom-bootloader-has-a-version-bug-168164
        data = self.read(20)
        self.ack()
        check(data[0] == 18, "Unexpected C011 command-frame length")
        check(all(code in data[2:] for code in (0x11, 0x21, 0x32, 0x45)),
              "ROM is missing required read/go/no-stretch write/erase commands")
        report(f"ROM ID 0x0443, protocol 0x{data[1]:02X}, commands {data[2:].hex()}")

    def address(self, address):
        self.write(checksum(address.to_bytes(4, "big")))
        self.ack()

    def read_memory(self, address, length):
        result = bytearray()
        while len(result) < length:
            count = min(32, length - len(result))
            self.command(0x11)
            self.address(address + len(result))
            self.write(bytes([count - 1, (count - 1) ^ 0xFF]))
            self.ack()
            result.extend(self.read(count))
        return bytes(result)

    def erase(self, length):
        pages = (length + PAGE_SIZE - 1) // PAGE_SIZE
        check(1 <= pages <= 15, "Refusing to erase the EEPROM page")
        self.command(0x45)
        self.write(checksum((pages - 1).to_bytes(2, "big")))
        self.ack()
        self.write(checksum(b"".join(page.to_bytes(2, "big") for page in range(pages))))
        self.ack()
        report(f"Erased application pages 0 through {pages - 1}; EEPROM page excluded")

    def program(self, image):
        check(8 <= len(image) <= APP_SIZE, "Image overlaps reserved EEPROM")
        # C011 programs 64-bit doublewords; 24 data bytes plus length/checksum
        # fit the Nano's 32-byte Wire buffer. Never rewrite a doubleword.
        padded = image + bytes([0xFF]) * (-len(image) % 8)
        for offset in range(0, len(padded), 24):
            block = padded[offset:offset + 24]
            try:
                self.command(0x32)
                self.address(FLASH_BASE + offset)
                self.write(checksum(bytes([len(block) - 1]) + block))
                self.ack()
            except Exception as error:
                raise RuntimeError(f"Programming failed at flash offset 0x{offset:04X}; "
                                   "no automatic retry or Go attempted") from error
            if offset % 3072 == 0:
                report(f"Programmed {offset + len(block)} / {len(padded)} bytes")
        return padded

    def go(self):
        self.command(0x21)
        self.address(FLASH_BASE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Nano/Metro USB port")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x49)
    parser.add_argument("--already-in-rom", action="store_true")
    parser.add_argument("--image", type=Path, help="Application-only .bin, at most 30 KiB")
    parser.add_argument("--backup", type=Path, help="New full-flash backup filename; required with --image")
    parser.add_argument("--go", action="store_true", help="Return to seesaw after successful checks")
    args = parser.parse_args()
    image = None
    if args.image:
        image = args.image.read_bytes()
        check(8 <= len(image) <= APP_SIZE, "Image must fit below the EEPROM page")
        stack = int.from_bytes(image[:4], "little")
        entry = int.from_bytes(image[4:8], "little")
        check(0x20000000 < stack <= 0x20001800 and stack % 8 == 0,
              "Image has an invalid C011 stack pointer")
        check(entry & 1 and FLASH_BASE <= entry < FLASH_BASE + len(image),
              "Image has an invalid reset vector")
        check(args.backup is not None and not args.backup.exists(),
              "Provide a new --backup filename before any erase")
    bridge = Metro(args.port)
    bridge.address = args.address
    try:
        if not args.already_in_rom:
            check(bridge.value(0, 1, 1) == 0x90, "Expected seesaw C011 hardware ID")
            bridge.write(0, 0x7E, [0xFF])
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
            report(f"Backed up all 32768 flash bytes; option bytes {options.hex()}")
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
            check(bridge.value(0, 1, 1) == 0x90, "Seesaw did not restart after ROM Go")
            bridge.reset()
            check(bridge.value(0, 1, 1) == 0x90, "Seesaw software reset failed after ROM Go")
            report("Seesaw restarted and passed hardware-ID/software-reset checks")
    finally:
        bridge.serial.close()


if __name__ == "__main__":
    main()
