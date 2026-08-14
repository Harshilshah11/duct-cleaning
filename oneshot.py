#!/usr/bin/env python3
"""
One-shot switch state read - no watcher, no waiting.

Reads each pin under both internal biases and reports what is actually driving
it right now. Run it with the switches held pressed, then again with them
released; comparing the two answers says whether the switch does anything.
"""

import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
PINS = [int(a) for a in sys.argv[1:]] or [22, 27]


def read(bias):
    req = gpiod.request_lines(
        CHIP, consumer="oneshot",
        config={p: gpiod.LineSettings(direction=Direction.INPUT, bias=bias)
                for p in PINS},
    )
    try:
        time.sleep(0.08)
        return {p: (req.get_value(p) == Value.ACTIVE) for p in PINS}
    finally:
        req.release()


def main():
    pu = read(Bias.PULL_UP)
    pd = read(Bias.PULL_DOWN)
    for p in PINS:
        a, b = int(pu[p]), int(pd[p])
        if a and not b:
            v = "FLOATING - nothing driving it (switch not making contact)"
        elif not a and not b:
            v = "driven LOW - shorted to GND (a pressed switch looks like this)"
        elif a and b:
            v = "driven HIGH - held at 3V3 by something external"
        else:
            v = "inverted/noise"
        print("  GPIO%-3d pull-up=%d pull-down=%d  ->  %s" % (p, a, b, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
