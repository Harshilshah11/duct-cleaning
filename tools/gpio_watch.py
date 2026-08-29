#!/usr/bin/env python3
"""Find and prove out switch GPIOs without being told which pins they are.

Idle level cannot tell an open switch from a floating pin - both just follow
whatever bias is applied. So each cycle applies BOTH biases and compares:

    pull-up=1, pull-down=0  -> follows bias, nothing driving it (open/floating)
    pull-up=0, pull-down=0  -> held LOW  regardless of bias -> tied to GND
    pull-up=1, pull-down=1  -> held HIGH regardless of bias -> tied to 3V3

That distinguishes a real connection from a floating pin AND recovers the
switch polarity, so it works whether the switches are active-high or
active-low. GPIO2/3 are excluded - the ADS bit-bang owns them.
"""
import subprocess
import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
# every header GPIO except 2/3 (I2C bit-bang)
PINS = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18,
        19, 20, 21, 22, 23, 24, 25, 26, 27]
# Harshil named these; he said "6 GPIOs" but listed 5, so the full sweep above
# stays in to catch whichever pin the sixth actually is.
EXPECTED = {25, 12, 16, 19, 13}
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0

PU = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
PD = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_DOWN)


def read_all(req, settings):
    req.reconfigure_lines({p: settings for p in PINS})
    time.sleep(0.002)          # let the bias settle through the pull resistor
    return {p: (1 if req.get_value(p) == Value.ACTIVE else 0) for p in PINS}


def classify(u, d):
    if u == 1 and d == 0:
        return "open"          # follows bias - open switch or nothing wired
    if u == 0 and d == 0:
        return "LOW"           # driven to GND
    if u == 1 and d == 1:
        return "HIGH"          # driven to 3V3
    return "odd"


def main():
    req = gpiod.request_lines(CHIP, consumer="gpio-watch",
                              config={p: PU for p in PINS})
    state, events = {p: "open" for p in PINS}, {p: set() for p in PINS}
    t0 = time.monotonic()
    print("probing %d GPIOs for %.0f s - press/hold each switch ~1 s"
          % (len(PINS), SECONDS), flush=True)
    try:
        while time.monotonic() - t0 < SECONDS:
            up = read_all(req, PU)
            dn = read_all(req, PD)
            for p in PINS:
                c = classify(up[p], dn[p])
                events[p].add(c)
                if c != state[p]:
                    print("[%7.1fs] GPIO%-3d %-5s -> %-5s"
                          % (time.monotonic() - t0, p, state[p], c), flush=True)
                    state[p] = c
            time.sleep(0.01)
    finally:
        print("=== SUMMARY ===", flush=True)
        for p in PINS:
            seen, tag = events[p], "  <- named" if p in EXPECTED else ""
            drives = sorted(s for s in seen if s in ("LOW", "HIGH"))
            if seen == {"open"}:
                # silent unless Harshil named it - then its silence is the finding
                if p in EXPECTED:
                    print("  GPIO%-3d NO RESPONSE - stayed open all run%s"
                          % (p, tag), flush=True)
                continue
            if "open" in seen and drives:
                print("  GPIO%-3d WORKING - toggled open <-> %s (active-%s)%s"
                      % (p, "/".join(drives),
                         "low" if drives[0] == "LOW" else "high", tag), flush=True)
            else:
                print("  GPIO%-3d STUCK at %s - never toggled%s"
                      % (p, "/".join(sorted(seen)), tag), flush=True)
        print("  (unnamed pins not listed stayed 'open' - nothing wired there)",
              flush=True)
        req.release()


if __name__ == "__main__":
    sys.exit(main())

