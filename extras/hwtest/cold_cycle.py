"""Human-gated cold-power EEPROM retention test. Never changes a rail setpoint.

prepare -> user disconnects DUT QT and ST-Link USB -> off -> user reconnects
QT -> verify (or verify --jumperless-link temporarily routes Metro power/I2C).
The original full-flash backup supplies the restoration bytes.
"""
import argparse
import time
from pathlib import Path

from bench import Jumperless, Metro, check, passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", choices=["prepare", "off", "verify", "restore"])
    parser.add_argument("--metro", required=True)
    parser.add_argument("--jumperless", required=True)
    parser.add_argument("--helper", required=True)
    parser.add_argument("--backup", required=True)
    parser.add_argument("--jumperless-link", action="store_true",
                        help="Temporarily route Metro VBUS/I2C through verified breadboard rows")
    args = parser.parse_args()
    check(not args.jumperless_link or args.step in ("verify", "restore"),
          "The temporary powered link is only for verify/restore after measured power-off")
    backup = Path(args.backup).read_bytes()
    check(len(backup) == 32768, "Expected the original complete C011 flash backup")
    offset = 0x20
    saved = backup[30720 + offset:30720 + offset + 8]
    pattern = bytes([0xA5, 0x5A, 0x00, 0xFF, 0x12, 0x34, 0x7E, 0x81])
    check(saved != pattern, "Choose a different pattern for this device")
    metro = None
    jl = Jumperless(args.jumperless, args.helper)
    try:
        if args.step == "off":
            # No I2C transactions while the DUT is unpowered.
            for sample in range(2):
                supply, vin = jl.sense(39), jl.sense(40)
                check(abs(supply) < 0.3 and abs(vin) < 0.3,
                      f"DUT still powered/back-powered: 3V={supply:.3f}, VIN={vin:.3f}")
                passed("cold_power_off", sample=sample, supply_V=supply, vin_V=vin)
                if sample == 0:
                    time.sleep(5)
            jl.exec("oled_clear(); oled_print('Cold off verified'); oled_print('Reconnect C011 QT')")
            return

        if args.jumperless_link:
            # Metro anchors: VBUS row 30, D13 row 60. A4/A5 are rows 18/17.
            # C011 PB6/PB7 are FT_f; DS13866 Rev 5 Table 23 note 2 requires
            # internal pulls OFF above VDD+0.3 V. STM32duino's AF_OD mapping
            # uses LL_GPIO_PULL_NO. Power must precede these 5 V bus signals.
            source = jl.sense(30)
            check(4.7 < source < 5.3, f"Unexpected Metro VBUS {source:.3f} V")
            jl.connect(30, 40)
            supply, vin = jl.sense(39), jl.sense(40)
            check(3.0 < supply < 3.6 and 4.3 < vin < 5.3,
                  f"Routed supply outside limits: VDD={supply:.3f}, VIN={vin:.3f}")
            passed("jumperless_link_power", source_V=source, supply_V=supply, vin_V=vin)
            for host, target in ((17, 35), (18, 36)):
                high = jl.sense(host)
                check(3.0 < high < 5.3, f"Metro I2C row {host} not idle high: {high:.3f} V")
                jl.connect(host, target)
                voltage = jl.sense(target)
                check(3.0 < voltage < 5.3, f"Connected I2C row {target}: {voltage:.3f} V")
                passed("jumperless_link_i2c_idle", host_row=host, target_row=target, volts=voltage)

        metro = Metro(args.metro)
        check(metro.value(0, 1, 1) == 0x90, "C011 not responding at the normal address")
        before = bytes(metro.value(13, offset + i, 1) for i in range(8))
        if args.step == "prepare":
            check(before == saved or before == pattern,
                  "EEPROM differs from backup/test pattern; do not overwrite changed user data")
            if before == saved:
                metro.write(13, offset, pattern)
                time.sleep(0.1)
            check(bytes(metro.value(13, offset + i, 1) for i in range(8)) == pattern,
                  "Cold-cycle preparation write did not verify")
            # Dim guide pixels: DUT supply and SWD connections.
            jl.exec("overlay_set_pixel(8, 9, 0x403000); overlay_set_pixel(8, 10, 0x403000); "
                    "overlay_set_pixel(8, 3, 0x301040); overlay_set_pixel(8, 4, 0x301040); "
                    "oled_clear(); oled_print('Unplug C011 QT'); oled_print('Unplug STLink USB')")
            passed("cold_pattern_prepared", offset=offset, pattern=pattern.hex(), original=saved.hex())
            return

        retained = before == pattern
        check(before == pattern or before == saved,
              "Unexpected EEPROM contents; preserve evidence before restoring")
        metro.write(13, offset, saved)
        time.sleep(0.1)
        check(bytes(metro.value(13, offset + i, 1) for i in range(8)) == saved,
              "Original EEPROM bytes did not restore")
        jl.exec("overlay_set_pixel(8, 9, 0); overlay_set_pixel(8, 10, 0); "
                "overlay_set_pixel(8, 3, 0); overlay_set_pixel(8, 4, 0); oled_clear()")
        passed("cold_original_restored", bytes=8)
        if args.step == "verify":
            check(retained, "EEPROM pattern did not survive the cold power cycle")
            passed("eeprom_cold_retention", bytes=8, pattern=pattern.hex())
    finally:
        # Remove I2C before power, including when any measurement fails.
        if args.jumperless_link:
            for route in ((17, 35), (18, 36), (30, 40)):
                if route in jl.routes:
                    jl.disconnect(*route)
        jl.close()
        if metro is not None:
            metro.serial.close()


if __name__ == "__main__":
    main()
