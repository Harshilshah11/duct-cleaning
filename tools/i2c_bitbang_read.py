#!/usr/bin/env python3
"""
Read the ADS1x15 joystick over BIT-BANGED I2C with SDA/SCL deliberately
reversed, i.e. matching how the wires are physically in the header right now.

This exists only to prove the stick works before the jumpers get swapped back;
once SDA is on pin 3 and SCL on pin 5, the real hardware bus works and
joystick_link.py is the thing to run instead.

Config word is taken from joystick_link.py so the readings are directly
comparable: PGA +/-6.144V, single-shot, 860 SPS, comparator off.
"""

import os
import sys
import time

# Import a sibling in tools/ regardless of where this is run from. This used
# to read "/home/arnobot", which stopped being right when the tree moved into
# DuctCleaning/ and was simply broken until 2026-08-29.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from i2c_bitbang_probe import IN, CHIP, Bus  # noqa: E402

import gpiod  # noqa: E402

ADDR = 0x48
REG_CONV, REG_CONFIG = 0x00, 0x01
CFG_BASE = 0x8000 | (0 << 9) | (1 << 8) | (7 << 5) | 0x0003

# Physically swapped: SDA is on gpio3, SCL is on gpio2.
SDA, SCL = 3, 2

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0


def read_byte(bus, ack):
    val = 0
    for _ in range(8):
        val = (val << 1) | (1 if bus.read_bit() else 0)
    bus.write_bit(not ack)  # pulling SDA low is the ACK
    return val


def write_reg(bus, reg, val):
    bus.start()
    ok = bus.write_byte(ADDR << 1)
    ok &= bus.write_byte(reg)
    ok &= bus.write_byte((val >> 8) & 0xFF)
    ok &= bus.write_byte(val & 0xFF)
    bus.stop()
    return ok


def read_reg(bus, reg):
    bus.start()
    if not bus.write_byte(ADDR << 1):
        bus.stop()
        return None
    if not bus.write_byte(reg):
        bus.stop()
        return None
    bus.start()  # repeated start
    if not bus.write_byte((ADDR << 1) | 1):
        bus.stop()
        return None
    msb = read_byte(bus, ack=True)
    lsb = read_byte(bus, ack=False)
    bus.stop()
    return (msb << 8) | lsb


def sample(bus, ch):
    if not write_reg(bus, REG_CONFIG, CFG_BASE | ((4 + ch) << 12)):
        return None
    time.sleep(0.002)  # 860 SPS -> ~1.2 ms conversion
    raw = read_reg(bus, REG_CONV)
    if raw is None:
        return None
    return raw - 65536 if raw > 32767 else raw


def main():
    req = gpiod.request_lines(CHIP, consumer="ads-read", config={SDA: IN, SCL: IN})
    bus = Bus(req, SDA, SCL)
    bus.reset_state()
    try:
        stats = {}
        print("reading A0 (VRx) and A1 (VRy) for %.0f s - MOVE THE STICK NOW" % SECONDS)
        print("%-8s %8s %8s" % ("t", "A0", "A1"))
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < SECONDS:
            vals = []
            for ch in (0, 1):
                v = sample(bus, ch)
                vals.append(v)
                if v is not None:
                    lo, hi = stats.get(ch, (v, v))
                    stats[ch] = (min(lo, v), max(hi, v))
            n += 1
            if n % 5 == 0:
                print("%-8.1f %8s %8s" % (time.monotonic() - t0,
                                          vals[0] if vals[0] is not None else "ERR",
                                          vals[1] if vals[1] is not None else "ERR"))
            time.sleep(0.05)

        print()
        print("=== swing over the run (%d samples) ===" % n)
        for ch in (0, 1):
            if ch in stats:
                lo, hi = stats[ch]
                swing = hi - lo
                verdict = "MOVING" if swing > 500 else "flat - not wired / not moved"
                print("  A%d: min=%-8d max=%-8d swing=%-8d %s" % (ch, lo, hi, swing, verdict))
            else:
                print("  A%d: no successful reads" % ch)
    finally:
        bus.reset_state()
        req.release()
        import subprocess
        for pin in (2, 3):
            subprocess.run(["/usr/bin/pinctrl", "set", str(pin), "a0", "pu"],
                           capture_output=True)


if __name__ == "__main__":
    sys.exit(main())
