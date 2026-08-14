#!/usr/bin/env python3
"""
Raw ADS1x15 sampler over the real hardware bus, all four channels.

Reads A0-A3 rather than just the two joystick axes, because a wire that has come
off the wiper usually shows up as one channel pinned to the rail while a
neighbouring channel suddenly carries the signal.

Prints a running view and a per-channel min/max so an axis can be judged as
moving, pinned to VDD, pinned to GND, or floating.
"""

import sys
import time

sys.path.insert(0, "/home/arnobot/ground_station")
from joystick_link import ADS1x15  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
COUNTS_PER_VOLT = 32768 / 6.144  # PGA is +/-6.144 V


def classify(lo, hi):
    swing = hi - lo
    if swing > 500:
        return "MOVING"
    mid = (lo + hi) / 2
    volts = mid / COUNTS_PER_VOLT
    if volts > 3.15:
        return "PINNED HIGH (~VDD) - wiper wire off, input tied to 3V3?"
    if volts < 0.15:
        return "PINNED LOW (~GND) - wiper wire off, input tied to GND?"
    return "flat at %.2f V - steady, just not moved" % volts


def main():
    ads = ADS1x15(bus=1, addr=0x48)
    stats = {}
    print("sampling A0-A3 for %.0f s" % SECONDS)
    print("%-6s %8s %8s %8s %8s" % ("t", "A0", "A1", "A2", "A3"))
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < SECONDS:
        row = []
        for ch in range(4):
            try:
                v = ads.read(ch)
            except OSError:
                v = None
            row.append(v)
            if v is not None:
                lo, hi = stats.get(ch, (v, v))
                stats[ch] = (min(lo, v), max(hi, v))
        n += 1
        if n % 4 == 0:
            print("%-6.1f %8s %8s %8s %8s"
                  % ((time.monotonic() - t0,)
                     + tuple(("ERR" if v is None else v) for v in row)))
        time.sleep(0.05)

    print()
    print("=== per-channel over %d samples ===" % n)
    for ch in range(4):
        if ch not in stats:
            print("  A%d: no successful reads" % ch)
            continue
        lo, hi = stats[ch]
        print("  A%d: min=%-7d max=%-7d swing=%-7d %.2f-%.2f V  %s"
              % (ch, lo, hi, hi - lo,
                 lo / COUNTS_PER_VOLT, hi / COUNTS_PER_VOLT, classify(lo, hi)))


if __name__ == "__main__":
    sys.exit(main())
