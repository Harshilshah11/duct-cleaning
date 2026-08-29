#!/usr/bin/env python3
"""
Short sampled read with the internal PULL-DOWN engaged, which is the correct
bias for these active-high switches: released should read 0, pressed 1.

Prints the value twice a second and, at the end, says plainly whether each pin
ever changed. Not a background watcher - it runs for a fixed 20 s and exits.
"""

import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
PINS = [22, 27]
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

req = gpiod.request_lines(
    CHIP, consumer="toggle-read",
    config={p: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_DOWN)
            for p in PINS},
)
try:
    seen = {p: set() for p in PINS}
    print("pull-down engaged: released should be 0, pressed 1")
    print("%-7s %8s %8s" % ("t", "GPIO22", "GPIO27"))
    t0 = time.monotonic()
    while time.monotonic() - t0 < SECONDS:
        vals = {}
        # sample fast between prints so a quick tap is not missed
        for _ in range(200):
            for p in PINS:
                v = 1 if req.get_value(p) == Value.ACTIVE else 0
                seen[p].add(v)
                vals[p] = v
            time.sleep(0.0025)
        print("%-7.1f %8d %8d" % (time.monotonic() - t0, vals[22], vals[27]))
        sys.stdout.flush()

    print()
    print("=== did the value change? ===")
    for p in PINS:
        vs = sorted(seen[p])
        if len(vs) > 1:
            print("  GPIO%-3d CHANGES - saw both %s  => switch is working" % (p, vs))
        else:
            print("  GPIO%-3d STUCK at %d - never changed" % (p, vs[0]))
finally:
    req.release()
