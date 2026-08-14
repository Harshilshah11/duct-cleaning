#!/usr/bin/env python3
"""
Wait for BOTH joystick axes to swing, over the bit-banged reversed wiring.

motion_watch.py does this on the hardware i2c bus, which is dead while SDA/SCL
are swapped. Same idea here: a fixed-duration sampler keeps expiring before a
human can get to the stick, so this waits up to TIMEOUT and **exits the moment
both axes have moved** instead of burning a fixed window.

Only A0/A1 count toward success. A2/A3 are unconnected and float across most of
the range on noise alone, so they are reported for context but never gate the
result -- treating their swing as motion would pass instantly and prove nothing.

Usage:  python3 bitbang_motion_watch.py [timeout_seconds]
Exit 0 = both axes swung. Exit 1 = timed out.
"""

import subprocess
import sys
import time

sys.path.insert(0, "/home/arnobot")
from i2c_bitbang_probe import IN, CHIP, Bus  # noqa: E402
from i2c_bitbang_read import SDA, SCL, sample  # noqa: E402

import gpiod  # noqa: E402

TIMEOUT = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0

# A real deflection moves thousands of counts (the axes rest near 8800 and the
# rail is ~17600), so this is far above sampling noise on a connected channel.
SWING_OK = 3000
AXES = (0, 1)
COUNTS_PER_VOLT = 32768 / 6.144


def main():
    req = gpiod.request_lines(CHIP, consumer="ads-motion", config={SDA: IN, SCL: IN})
    bus = Bus(req, SDA, SCL)
    bus.reset_state()
    try:
        stats = {}
        print("waiting up to %.0f s - MOVE BOTH AXES FULLY (left+right, up+down)"
              % TIMEOUT)
        print("exits as soon as both axes have swung; no need to rush")
        sys.stdout.flush()

        t0 = time.monotonic()
        done = {}
        n = 0
        while time.monotonic() - t0 < TIMEOUT:
            for ch in AXES + (2, 3):
                v = sample(bus, ch)
                if v is None:
                    continue
                lo, hi = stats.get(ch, (v, v))
                stats[ch] = (min(lo, v), max(hi, v))
            n += 1

            for ch in AXES:
                if ch in done or ch not in stats:
                    continue
                lo, hi = stats[ch]
                if hi - lo >= SWING_OK:
                    done[ch] = time.monotonic() - t0
                    print("  A%d SWUNG at t=%.1fs  (min=%d max=%d swing=%d)"
                          % (ch, done[ch], lo, hi, hi - lo))
                    sys.stdout.flush()

            if len(done) == len(AXES):
                print()
                print("BOTH AXES MOVING - joystick confirmed good after %.1f s"
                      % (time.monotonic() - t0))
                break
            time.sleep(0.03)

        print()
        print("=== final, %d sample rounds ===" % n)
        for ch in (0, 1, 2, 3):
            if ch not in stats:
                print("  A%d: no successful reads" % ch)
                continue
            lo, hi = stats[ch]
            tag = "axis" if ch in AXES else "unconnected/floating"
            mark = "OK" if ch in done else ("NOT MOVED" if ch in AXES else "")
            print("  A%d: min=%-8d max=%-8d swing=%-8d %.2f-%.2f V  %s %s"
                  % (ch, lo, hi, hi - lo,
                     lo / COUNTS_PER_VOLT, hi / COUNTS_PER_VOLT, tag, mark))

        missing = [ch for ch in AXES if ch not in done]
        if missing:
            print()
            print("TIMED OUT - never saw a full swing on: %s"
                  % ", ".join("A%d" % c for c in missing))
            return 1
        return 0
    finally:
        req.release()
        # Give GPIO2/3 back to the hardware i2c controller, or the real bus
        # stays broken for every later run.
        subprocess.run(["pinctrl", "set", "2", "a0", "pu"], check=False)
        subprocess.run(["pinctrl", "set", "3", "a0", "pu"], check=False)


if __name__ == "__main__":
    sys.exit(main())
