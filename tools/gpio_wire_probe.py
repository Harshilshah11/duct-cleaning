#!/usr/bin/env python3
"""
Decide whether a wire is physically attached to a GPIO, without anyone pressing
anything.

An open switch and an empty header pin both read "floating", so bias tests
cannot separate them. Capacitance can: a pin with a jumper and a switch body
hanging off it holds noticeably more charge than a bare pin, and it also picks
up more mains hum because the wire acts as an antenna.

Two independent measurements, each comparing the pins under test against control
pins believed to be bare:

  1. DECAY - drive the pin high, release it to a pull-down input, and count how
     many reads it stays high. More attached capacitance -> slower decay.
  2. HUM   - float the pin with no bias at all and count transitions. A longer
     wire couples more 50 Hz noise -> more transitions.

Neither is conclusive alone, so both are reported and the controls set the
baseline. This is safe with an open switch; if a switch were closed to GND the
brief high drive would be a momentary short, so the caller checks that the pins
read floating first.
"""

import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
TARGETS = [22, 27]
CONTROLS = [5, 6, 26]  # unused on this board; the bare-pin baseline
DECAY_TRIALS = 40
HUM_SAMPLES = 40000


def decay_count(pin):
    """Drive high, release to pull-down, count reads before it falls."""
    req = gpiod.request_lines(
        CHIP, consumer="wire-probe",
        config={pin: gpiod.LineSettings(direction=Direction.OUTPUT,
                                        output_value=Value.ACTIVE)},
    )
    try:
        time.sleep(0.0005)  # let the pin charge
        req.reconfigure_lines(
            {pin: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_DOWN)})
        n = 0
        while n < 5000:
            if req.get_value(pin) != Value.ACTIVE:
                break
            n += 1
        return n
    finally:
        req.release()


def hum_transitions(pin, samples=HUM_SAMPLES):
    """Float with no bias and count edges - a wire picks up mains hum."""
    req = gpiod.request_lines(
        CHIP, consumer="wire-probe",
        config={pin: gpiod.LineSettings(direction=Direction.INPUT,
                                        bias=Bias.DISABLED)},
    )
    try:
        last = req.get_value(pin)
        edges = 0
        for _ in range(samples):
            v = req.get_value(pin)
            if v != last:
                edges += 1
                last = v
        return edges
    finally:
        req.release()


def main():
    pins = TARGETS + CONTROLS

    # Safety: a pin held low by a closed switch must not be driven high.
    req = gpiod.request_lines(
        CHIP, consumer="wire-probe",
        config={p: gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
                for p in pins},
    )
    try:
        time.sleep(0.05)
        held_low = [p for p in pins if req.get_value(p) != Value.ACTIVE]
    finally:
        req.release()
    if held_low:
        print("pins held LOW with pull-up (switch closed?): %s - skipping decay test"
              % held_low)
        return 1

    print("=== decay test (higher = more capacitance = wire attached) ===")
    decay = {}
    for p in pins:
        trials = sorted(decay_count(p) for _ in range(DECAY_TRIALS))
        decay[p] = trials[len(trials) // 2]  # median resists outliers
        tag = "TARGET " if p in TARGETS else "control"
        print("  %s GPIO%-3d median decay count = %d" % (tag, p, decay[p]))

    print()
    print("=== hum test (higher = longer antenna = wire attached) ===")
    hum = {}
    for p in pins:
        hum[p] = hum_transitions(p)
        tag = "TARGET " if p in TARGETS else "control"
        print("  %s GPIO%-3d floating transitions = %d" % (tag, p, hum[p]))

    ctl_decay = sorted(decay[p] for p in CONTROLS)[len(CONTROLS) // 2]
    ctl_hum = sorted(hum[p] for p in CONTROLS)[len(CONTROLS) // 2]

    print()
    print("=== verdict (control baseline: decay=%d, hum=%d) ===" % (ctl_decay, ctl_hum))
    for p in TARGETS:
        d_hi = decay[p] > ctl_decay * 2 + 2
        h_hi = hum[p] > ctl_hum * 2 + 20
        if d_hi and h_hi:
            v = "WIRE ATTACHED - both tests well above the bare-pin baseline"
        elif d_hi or h_hi:
            v = "LIKELY attached - one test above baseline, the other not"
        else:
            v = "looks BARE - indistinguishable from an unconnected pin"
        print("  GPIO%-3d decay=%-5d hum=%-6d  %s" % (p, decay[p], hum[p], v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
