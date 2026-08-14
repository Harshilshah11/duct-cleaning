#!/usr/bin/env python3
"""
Characterise switches on GPIO pins, then watch them for presses.

Reading a pin once tells you almost nothing: a floating input, a switch wired to
GND, and a pin hard-tied to 3V3 can all read the same. So this reads each pin
TWICE, once with the internal pull-up and once with the pull-down, which
separates the cases:

    pull-up=1, pull-down=0  -> FLOATING: the pin follows whatever bias is
                               applied, so nothing external is driving it.
                               Normal for an open switch with no external
                               resistor - press it and it should go to 0.
    pull-up=0, pull-down=0  -> driven LOW  (switch closed to GND, or a
                               strong external pull-down)
    pull-up=1, pull-down=1  -> driven HIGH (external pull-up, or tied to 3V3)

Then it watches for transitions. Like motion_watch.py this is event-driven
rather than timed - there is no window to hit, it exits once every pin has been
seen to change at least once.

Usage:  gpio_switch_check.py [seconds] [pin ...]     default: 120s, pins 22 27
"""

import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
DEBOUNCE = 0.02  # 20 ms - contact bounce is well under this


def settings(bias):
    return gpiod.LineSettings(direction=Direction.INPUT, bias=bias)


def read_with_bias(pins, bias, settle=0.05):
    req = gpiod.request_lines(
        CHIP, consumer="switch-check",
        config={p: settings(bias) for p in pins},
    )
    try:
        time.sleep(settle)
        return {p: (req.get_value(p) == Value.ACTIVE) for p in pins}
    finally:
        req.release()


def classify(pu, pd):
    if pu and not pd:
        return "FLOATING - no external resistor; open switch looks like this"
    if not pu and not pd:
        return "driven LOW - switch closed to GND, or external pull-down"
    if pu and pd:
        return "driven HIGH - external pull-up, or tied to 3V3"
    return "inverted?? - reads low with pull-up and high with pull-down (noise)"


def main():
    argv = sys.argv[1:]
    seconds = float(argv[0]) if argv else 120.0
    pins = [int(a) for a in argv[1:]] or [22, 27]

    print("=== static characterisation ===")
    pu = read_with_bias(pins, Bias.PULL_UP)
    pd = read_with_bias(pins, Bias.PULL_DOWN)
    state = {}
    for p in pins:
        print("  GPIO%-3d pull-up=%d pull-down=%d  ->  %s"
              % (p, pu[p], pd[p], classify(pu[p], pd[p])))
        state[p] = classify(pu[p], pd[p])

    # A floating pin needs a bias to be usable; pull-up is the convention for a
    # switch that shorts to GND, which is how these are almost always wired.
    print()
    print("=== watching for presses (pull-up enabled, so PRESSED = 0) ===")
    print("press each switch a few times - up to %.0f s, stops early once all "
          "have toggled" % seconds)
    sys.stdout.flush()

    req = gpiod.request_lines(
        CHIP, consumer="switch-watch",
        config={p: settings(Bias.PULL_UP) for p in pins},
    )
    try:
        last = {p: req.get_value(p) == Value.ACTIVE for p in pins}
        stable = dict(last)
        changed_at = {p: 0.0 for p in pins}
        toggles = {p: 0 for p in pins}
        print("  initial: " + "  ".join("GPIO%d=%d" % (p, last[p]) for p in pins))
        sys.stdout.flush()

        t0 = time.monotonic()
        last_report = 0.0
        while time.monotonic() - t0 < seconds:
            now = time.monotonic()
            for p in pins:
                v = req.get_value(p) == Value.ACTIVE
                if v != last[p]:
                    last[p] = v
                    changed_at[p] = now
                elif v != stable[p] and (now - changed_at[p]) > DEBOUNCE:
                    stable[p] = v
                    toggles[p] += 1
                    print("  [%6.1fs] GPIO%-3d -> %d  (%s)"
                          % (now - t0, p, v, "RELEASED" if v else "PRESSED"))
                    sys.stdout.flush()

            if now - t0 - last_report >= 10.0:
                last_report = now - t0
                print("  [%6.1fs] still watching - toggles so far: %s"
                      % (last_report, ", ".join("GPIO%d=%d" % (p, toggles[p]) for p in pins)))
                sys.stdout.flush()

            if all(toggles[p] > 0 for p in pins):
                print("  every pin has toggled - stopping early")
                break
            time.sleep(0.002)

        print()
        print("=== result ===")
        for p in pins:
            verdict = ("WORKING - %d transitions seen" % toggles[p]) if toggles[p] else \
                      "no transitions - not pressed, or not wired"
            print("  GPIO%-3d %-52s %s" % (p, state[p], verdict))
        return 0 if all(toggles[p] > 0 for p in pins) else 1
    finally:
        req.release()


if __name__ == "__main__":
    sys.exit(main())
