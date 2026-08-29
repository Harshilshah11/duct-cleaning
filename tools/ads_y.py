#!/usr/bin/env python3
"""
Decide whether the Y wiper is actually connected to A1.

A connected potentiometer wiper swept by hand produces a SMOOTH RAMP: successive
samples differ by small amounts and the value spends most of its time away from
the supply rail. A disconnected (floating) input cannot do that - it parks at a
rail and leaves only in large, isolated jumps as charge smears through the mux.

So instead of eyeballing the trace, this measures the two things that separate
those cases: how much of the run is spent pinned at the rail, and how large the
sample-to-sample steps are.
"""

import sys
import time

sys.path.insert(0, "/home/arnobot/ground_station")
from joystick_link import ADS1x15  # noqa: E402

CH = int(sys.argv[1]) if len(sys.argv) > 1 else 1
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
COUNTS_PER_VOLT = 32768 / 6.144
RAIL = 3.30 * COUNTS_PER_VOLT  # ~17600
NEAR_RAIL = 200                # counts either side still counts as "at the rail"


def main():
    ads = ADS1x15(bus=1, addr=0x48)
    print("sampling A%d for %.0f s - sweep that axis SLOWLY, extreme to extreme" % (CH, SECONDS))
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < SECONDS:
        try:
            samples.append(ads.read(CH))
        except OSError:
            pass
        time.sleep(0.02)

    if len(samples) < 10:
        print("too few samples")
        return 1

    at_rail = sum(1 for v in samples if abs(v - RAIL) < NEAR_RAIL)
    steps = [abs(b - a) for a, b in zip(samples, samples[1:])]
    big = sum(1 for s in steps if s > 2000)
    small = sum(1 for s in steps if s <= 200)

    print()
    print("=== A%d over %d samples ===" % (CH, len(samples)))
    print("  range        : %d .. %d" % (min(samples), max(samples)))
    print("  at rail(3.3V): %d/%d = %.0f%%" % (at_rail, len(samples), 100.0 * at_rail / len(samples)))
    print("  steps <=200  : %d/%d = %.0f%%  (smooth motion)" % (small, len(steps), 100.0 * small / len(steps)))
    print("  steps >2000  : %d/%d = %.0f%%  (impossible for a real pot sweep)"
          % (big, len(steps), 100.0 * big / len(steps)))

    # A hand sweep is overwhelmingly small steps; a floating pin is rail + jumps.
    print()
    if at_rail > 0.5 * len(samples) and big > 0.05 * len(steps):
        print("  VERDICT: A%d is FLOATING - parked at VDD with isolated jumps." % CH)
        print("  The wiper is not connected. Re-seat the Y signal wire.")
    elif small > 0.8 * len(steps) and at_rail < 0.5 * len(samples):
        print("  VERDICT: A%d tracks a real pot - smooth sweep, wiper connected." % CH)
    else:
        print("  VERDICT: inconclusive - sweep the axis slowly through its whole")
        print("  travel during the sampling window and run again.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
