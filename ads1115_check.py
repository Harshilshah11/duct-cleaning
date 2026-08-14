#!/usr/bin/env python3
"""Check whether an ADS1115 is on the I2C bus and whether a potentiometer
feeding it actually produces changing data. Uses smbus2 only."""
import sys
import time

from smbus2 import SMBus, i2c_msg

BUS = 1
ADS_ADDRESSES = (0x48, 0x49, 0x4A, 0x4B)  # ADDR->GND, VDD, SDA, SCL
REG_CONV, REG_CONFIG = 0x00, 0x01
PGA_4096 = 4.096  # full-scale volts for PGA setting 001
MUX = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}  # AINx vs GND


def scan(bus):
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.i2c_rdwr(i2c_msg.write(addr, []))
            found.append(addr)
        except OSError:
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError:
                pass
    return found


def read_channel(bus, addr, ch):
    cfg = 0x8000 | MUX[ch] | 0x0200 | 0x0100 | 0x0080 | 0x0003
    bus.write_i2c_block_data(addr, REG_CONFIG, [(cfg >> 8) & 0xFF, cfg & 0xFF])
    time.sleep(0.012)
    hi, lo = bus.read_i2c_block_data(addr, REG_CONV, 2)
    raw = (hi << 8) | lo
    if raw > 0x7FFF:
        raw -= 0x10000
    return raw, raw * PGA_4096 / 32768.0


def main():
    with SMBus(BUS) as bus:
        devices = scan(bus)
        print("I2C scan on bus %d: %s" % (
            BUS, ", ".join("0x%02X" % d for d in devices) if devices else "NOTHING FOUND"))

        ads = [d for d in devices if d in ADS_ADDRESSES]
        if not ads:
            print("\nRESULT: no ADS1115 detected at 0x48-0x4B.")
            print("The ADC is not reachable — check VDD/GND, SDA->GPIO2, SCL->GPIO3,")
            print("and that ADDR is tied to a rail (floating ADDR gives no address).")
            return 2

        addr = ads[0]
        print("\nADS1115 found at 0x%02X. Sampling all 4 channels for 10s." % addr)
        print("TURN THE POTENTIOMETER NOW so movement shows up.\n")
        print("  %-6s %10s %10s %10s %10s" % ("t", "A0 (V)", "A1 (V)", "A2 (V)", "A3 (V)"))

        stats = {ch: [] for ch in range(4)}
        t0 = time.time()
        while time.time() - t0 < 10:
            volts = []
            for ch in range(4):
                try:
                    _, v = read_channel(bus, addr, ch)
                except OSError as exc:
                    print("  read error on A%d: %s" % (ch, exc))
                    return 3
                stats[ch].append(v)
                volts.append(v)
            print("  %-6.1f %10.4f %10.4f %10.4f %10.4f"
                  % (time.time() - t0, volts[0], volts[1], volts[2], volts[3]))
            time.sleep(0.4)

        print("\n  %-4s %9s %9s %9s  %s" % ("ch", "min", "max", "swing", "verdict"))
        live = []
        for ch in range(4):
            s = stats[ch]
            swing = max(s) - min(s)
            if swing > 0.05:
                verdict = "CHANGING <- potentiometer here"
                live.append(ch)
            elif max(s) > 0.05:
                verdict = "steady voltage (connected, not moving)"
            else:
                verdict = "~0V (floating or grounded)"
            print("  A%-3d %9.4f %9.4f %9.4f  %s" % (ch, min(s), max(s), swing, verdict))

        print()
        if live:
            print("RESULT: data IS coming through — channel(s) %s responded to movement."
                  % ", ".join("A%d" % c for c in live))
        else:
            print("RESULT: ADS1115 responds, but no channel changed by more than 50mV.")
            print("Either the pot was not turned during the test, or its wiper is not")
            print("wired to an analog input. Check wiper->AINx, and the outer legs to 3V3/GND.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
