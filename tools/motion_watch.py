#!/usr/bin/env python3
"""
Watch both joystick axes until real motion is seen, then stop.

The fixed-duration samplers kept expiring before the operator saw the prompt, so
this one is event-driven instead of timed: it runs for up to WAIT seconds and
exits the moment BOTH axes have swung past THRESHOLD counts from their resting
value. There is no window to hit - move the stick whenever.

Progress is flushed to the log continuously so the run can be inspected while it
is still going.
"""

import sys
import time

sys.path.insert(0, "/home/arnobot/ground_station")
from joystick_link import ADS1x15  # noqa: E402

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
THRESHOLD = 3000  # counts from centre; well past the 800 deadband
COUNTS_PER_VOLT = 32768 / 6.144


def main():
    ads = ADS1x15(bus=1, addr=0x48)

    rest = {}
    for ch in (0, 1):
        vals = [ads.read(ch) for _ in range(32)]
        rest[ch] = sum(vals) / len(vals)
    print("resting  X=%.0f  Y=%.0f" % (rest[0], rest[1]))
    print("waiting up to %.0f s - move the stick to all four extremes whenever "
          "you are ready" % WAIT)
    sys.stdout.flush()

    lo = {0: rest[0], 1: rest[1]}
    hi = {0: rest[0], 1: rest[1]}
    done = {0: False, 1: False}
    t0 = time.monotonic()
    n = 0
    last_report = 0.0

    while time.monotonic() - t0 < WAIT:
        for ch in (0, 1):
            try:
                v = ads.read(ch)
            except OSError:
                continue
            lo[ch] = min(lo[ch], v)
            hi[ch] = max(hi[ch], v)
            if not done[ch] and (hi[ch] - lo[ch]) > THRESHOLD:
                done[ch] = True
                print("  [%5.1fs] axis %s MOVED: %d..%d (swing %d)"
                      % (time.monotonic() - t0, "XY"[ch], lo[ch], hi[ch], hi[ch] - lo[ch]))
                sys.stdout.flush()
        n += 1

        now = time.monotonic() - t0
        if now - last_report >= 5.0:
            last_report = now
            print("  [%5.1fs] X swing %d, Y swing %d"
                  % (now, hi[0] - lo[0], hi[1] - lo[1]))
            sys.stdout.flush()

        if all(done.values()):
            print("  both axes moved - stopping early")
            break
        time.sleep(0.01)

    print()
    print("=== result over %d samples ===" % n)
    for ch in (0, 1):
        swing = hi[ch] - lo[ch]
        print("  %s: rest=%-7.0f min=%-7d max=%-7d swing=%-7d  %.2f-%.2f V  %s"
              % ("XY"[ch], rest[ch], lo[ch], hi[ch], swing,
                 lo[ch] / COUNTS_PER_VOLT, hi[ch] / COUNTS_PER_VOLT,
                 "MOVING" if swing > THRESHOLD else "no motion seen"))
    return 0 if all(done.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
