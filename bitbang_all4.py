#!/usr/bin/env python3
"""
Bit-bang read of ALL FOUR ADS1x15 channels through the reversed wiring.

i2c_bitbang_read.py only samples A0/A1, which is blind to the most common
wiring slip: a signal wire that has been pushed into the wrong hole on the
module. If VRx has landed in A2 or A3, that reader shows A0 flat and never
reveals where the signal actually went. This one shows all four so a stray
axis has somewhere to turn up.

Same config word as joystick_link.py so counts are directly comparable.
Restores the pins to ALT0 on the way out, exactly like the other two scripts.
"""

import subprocess
import sys
import time

sys.path.insert(0, "/home/arnobot")
from i2c_bitbang_probe import IN, CHIP, Bus  # noqa: E402
from i2c_bitbang_read import SDA, SCL, sample  # noqa: E402

import gpiod  # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
CHANNELS = (0, 1, 2, 3)
COUNTS_PER_VOLT = 32768 / 6.144


def classify(lo, hi, jitter):
    """Swing alone cannot tell a moved axis from a floating pin.

    An unconnected ADS input drifts across most of the range on noise, so a big
    swing is NOT proof of signal. What separates them is sample-to-sample
    jitter: a hand-moved pot is a low-impedance source and steps smoothly, while
    a floating pin jumps hundreds of counts between adjacent samples.
    """
    swing = hi - lo
    mid = (lo + hi) / 2.0
    volts = mid / COUNTS_PER_VOLT
    if swing > 500:
        if jitter > 300:
            return "FLOATING - big swing but jitter=%d/sample, nothing attached" % jitter
        return "MOVING (smooth, jitter=%d) <-- real signal" % jitter
    if volts > 3.15:
        return "pinned HIGH (~VDD)"
    if volts < 0.15:
        return ("at ~0 V, jitter=%d - %s" % (
            jitter,
            "LOW-IMPEDANCE tie to GND" if jitter < 100 else "floating near 0"))
    return "flat at %.2f V - connected but not moved" % volts


def main():
    req = gpiod.request_lines(CHIP, consumer="ads-all4", config={SDA: IN, SCL: IN})
    bus = Bus(req, SDA, SCL)
    bus.reset_state()
    try:
        stats = {}
        jitter = {}   # ch -> [sum of |delta| between adjacent samples, count]
        last = {}
        print("reading A0-A3 for %.0f s - MOVE BOTH AXES FULLY NOW" % SECONDS)
        print("%-8s %8s %8s %8s %8s" % ("t", "A0", "A1", "A2", "A3"))
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < SECONDS:
            vals = []
            for ch in CHANNELS:
                v = sample(bus, ch)
                vals.append(v)
                if v is not None:
                    lo, hi = stats.get(ch, (v, v))
                    stats[ch] = (min(lo, v), max(hi, v))
                    if ch in last:
                        s, c = jitter.get(ch, (0, 0))
                        jitter[ch] = (s + abs(v - last[ch]), c + 1)
                    last[ch] = v
            n += 1
            if n % 5 == 0:
                print("%-8.1f %8s %8s %8s %8s"
                      % ((time.monotonic() - t0,)
                         + tuple("ERR" if v is None else v for v in vals)))
            time.sleep(0.03)

        print()
        print("=== swing over the run (%d samples) ===" % n)
        for ch in CHANNELS:
            if ch not in stats:
                print("  A%d: no successful reads" % ch)
                continue
            lo, hi = stats[ch]
            s, c = jitter.get(ch, (0, 0))
            j = int(s / c) if c else 0
            print("  A%d: min=%-8d max=%-8d swing=%-8d %s"
                  % (ch, lo, hi, hi - lo, classify(lo, hi, j)))
    finally:
        req.release()
        # Hand GPIO2/3 back to the hardware i2c controller, or the real bus
        # stays broken for every later run.
        subprocess.run(["pinctrl", "set", "2", "a0", "pu"], check=False)
        subprocess.run(["pinctrl", "set", "3", "a0", "pu"], check=False)


if __name__ == "__main__":
    sys.exit(main())
