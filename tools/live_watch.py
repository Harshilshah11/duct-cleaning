#!/usr/bin/env python3
"""Continuous ADS1115 watcher - logs movement events as they happen.

Runs detached so there is no need to sync a capture window with whoever is
at the hardware. Move a control whenever; the event log records it.
A3 is unconnected and deliberately not sampled.
"""
import subprocess
import os
import sys
import time

# Import a sibling in tools/ regardless of where this is run from. This used
# to read "/home/arnobot", which stopped being right when the tree moved into
# DuctCleaning/ and was simply broken until 2026-08-29.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from i2c_bitbang_probe import IN, CHIP, Bus            # noqa: E402
from i2c_bitbang_read import SDA, SCL, sample          # noqa: E402
import gpiod                                            # noqa: E402

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
CHANNELS = (0, 1, 2)
NAMES = {0: "A0 joyX", 1: "A1 joyY", 2: "A2 pot"}
CPV = 32768 / 6.144
MOVE = 400          # counts from baseline that count as a deliberate move


def main():
    req = gpiod.request_lines(CHIP, consumer="live-watch",
                              config={SDA: IN, SCL: IN})
    bus = Bus(req, SDA, SCL)
    bus.reset_state()
    base, lo_all, hi_all, moved = {}, {}, {}, {}
    t0 = time.monotonic()
    print("watching A0/A1/A2 for %.0f s - move anything, any time" % SECONDS,
          flush=True)
    try:
        while time.monotonic() - t0 < SECONDS:
            now = time.monotonic() - t0
            for ch in CHANNELS:
                v = sample(bus, ch)
                if v is None:
                    continue
                base.setdefault(ch, v)
                lo_all[ch] = min(lo_all.get(ch, v), v)
                hi_all[ch] = max(hi_all.get(ch, v), v)
                if abs(v - base[ch]) > MOVE and not moved.get(ch):
                    moved[ch] = True
                    print("[%7.1fs] %-9s MOVED  %d -> %d  (%.2f V)"
                          % (now, NAMES[ch], base[ch], v, v / CPV), flush=True)
                elif abs(v - base[ch]) <= MOVE and moved.get(ch):
                    moved[ch] = False
                    base[ch] = v
            time.sleep(0.02)
    finally:
        print("=== TOTAL RANGE SEEN ===", flush=True)
        for ch in CHANNELS:
            if ch not in lo_all:
                print("  %-9s no reads" % NAMES[ch], flush=True)
                continue
            lo, hi = lo_all[ch], hi_all[ch]
            verdict = "DATA FLOWING" if hi - lo > MOVE else "never moved"
            print("  %-9s %.2f-%.2f V  swing=%-6d %s"
                  % (NAMES[ch], lo / CPV, hi / CPV, hi - lo, verdict), flush=True)
        req.release()
        for pin in (2, 3):
            subprocess.run(["/usr/bin/pinctrl", "set", str(pin), "a0", "pu"],
                           check=False, capture_output=True)


if __name__ == "__main__":
    sys.exit(main())

