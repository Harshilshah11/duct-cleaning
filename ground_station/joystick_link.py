#!/usr/bin/env python3
"""
Read the ADS1115 joystick on the Pi's I2C and send it to the Uno over Ethernet.

The stick used to hang off the Uno's own I2C pins and drive the motor directly
(ADS1115_Joystick_Test.ino). It now lives here instead, so the ground station
owns the operator input and the Uno is only an actuator. Calibration therefore
lives on this side too: this script finds the centre, removes the deadband and
scales to +/-1000, and UnoEthLink.ino just turns that into DIR/PWM. Re-tuning a
constant here costs a restart, not a reflash.

Talks to 192.168.50.20:5005 on the ISOLATED robot segment (the Pi reaches it
directly from eth0's 192.168.50.15/24 — no host route needed, unlike the old
192.168.1.20 which collided with the Pi's own wlan0 DHCP lease).

No third-party packages. Debian trixie is an externally-managed environment, so
`pip install adafruit-...` would need a venv or --break-system-packages; the
ADS1x15 is simple enough that talking to /dev/i2c-1 through ioctl is less
trouble than the dependency. Works for both ADS1115 and ADS1015 — the raw
16-bit read is the same, the 1015 just left-aligns its 12 bits (which is why
its readings come back as exact multiples of 16).

Usage:
    python3 joystick_link.py                 # 50 Hz to the Uno
    python3 joystick_link.py --print-only    # read the stick, send nothing
    python3 joystick_link.py --scan          # list I2C addresses and exit
    python3 joystick_link.py --raw           # show raw counts, for re-tuning
    python3 joystick_link.py --host 192.168.50.20 --hz 20
"""

import argparse
import fcntl
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from uno_link import UnoLink  # noqa: E402  (needs the path fix above)

I2C_SLAVE = 0x0703  # from linux/i2c-dev.h

UNO_HOST = os.environ.get("UNO_HOST", "192.168.50.20")
UNO_PORT = int(os.environ.get("UNO_PORT", "5005"))

# ---- calibration -------------------------------------------------------------
# Carried over from ADS1115_Joystick_Test.ino, which measured them on this exact
# stick: X swept -9627..+10933 around a resting centre of ~12100.
CH_X = 0
CH_Y = 1

# The PGA fixes counts-per-volt (+/-6.144 V across 16 bits), so the deadband is
# an absolute voltage and does NOT move with the joystick's supply rail:
# 800 counts is ~150 mV either side of centre on 3V3 or 5V alike.
DEADBAND = 800

# Full deflection does scale with the supply, though, because the stick is just
# a divider across it. So rather than hardcode the 8664 the Uno sketch measured
# on 5V, take it as a fraction of the *measured* centre (8664/12100) — that way
# the same behaviour comes out whether the ADS ends up on the Pi's 5V or 3V3.
FULL_TILT_RATIO = 0.716

# Only used as a fallback when the stick is being moved during centring.
CENTRE_TYPICAL = 12100


class ADS1x15:
    """Minimal single-ended reader for ADS1115/ADS1015 over /dev/i2c-N."""

    REG_CONV = 0x00
    REG_CONFIG = 0x01

    # OS=1 (start), PGA=000 (+/-6.144V, safe for a 5V stick), MODE=1 (single
    # shot), DR=111 (fastest: 860 SPS on the 1115), COMP_QUE=11 (comparator off).
    # Single-shot rather than continuous because we read two channels alternately
    # and continuous mode would hand back whichever conversion finished last.
    CFG_BASE = 0x8000 | (0 << 9) | (1 << 8) | (7 << 5) | 0x0003

    def __init__(self, bus=1, addr=0x48):
        self.bus, self.addr = bus, addr
        path = "/dev/i2c-%d" % bus
        if not os.path.exists(path):
            raise SystemExit(
                "%s does not exist - I2C is not enabled on this Pi.\n"
                "Fix:  sudo raspi-config nonint do_i2c 0  && sudo reboot" % path
            )
        self.fd = os.open(path, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def _write_reg(self, reg, val):
        os.write(self.fd, bytes([reg, (val >> 8) & 0xFF, val & 0xFF]))

    def _read_reg(self, reg):
        os.write(self.fd, bytes([reg]))
        d = os.read(self.fd, 2)
        return (d[0] << 8) | d[1]

    def read(self, ch):
        """One single-ended conversion on AIN<ch>, as a signed 16-bit count."""
        # MUX 100..111 select AIN0..AIN3 single-ended against GND.
        self._write_reg(self.REG_CONFIG, self.CFG_BASE | ((4 + ch) << 12))

        # Poll the OS bit rather than sleeping a fixed time: it is ~1.2 ms on a
        # 1115 at 860 SPS but ~0.3 ms on a 1015, and guessing wrong either wastes
        # half our budget at 50 Hz or reads the previous conversion.
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            if self._read_reg(self.REG_CONFIG) & 0x8000:
                break
            time.sleep(0.0002)

        raw = self._read_reg(self.REG_CONV)
        return raw - 65536 if raw > 32767 else raw

    def close(self):
        os.close(self.fd)


def scan(bus=1):
    """Probe every address on the bus. Standalone so bring-up needs no ADS."""
    path = "/dev/i2c-%d" % bus
    if not os.path.exists(path):
        raise SystemExit(
            "%s does not exist - I2C is not enabled.\n"
            "Fix:  sudo raspi-config nonint do_i2c 0  && sudo reboot" % path
        )
    found = []
    fd = os.open(path, os.O_RDWR)
    try:
        for addr in range(0x03, 0x78):
            try:
                fcntl.ioctl(fd, I2C_SLAVE, addr)
                os.read(fd, 1)
                found.append(addr)
            except OSError:
                pass
    finally:
        os.close(fd)
    return found


def sample_centre(ads, ch, n=32):
    """Mean resting value, plus peak-to-peak spread so we can tell if it moved."""
    vals = [ads.read(ch) for _ in range(n)]
    return int(round(sum(vals) / float(len(vals)))), max(vals) - min(vals)


def validate_centre(c, spread, label):
    """Trust a steady reading whatever its absolute value.

    A stick held off-centre at boot would otherwise become "zero", making one
    direction unreachable. The Uno sketch caught that by comparing against a
    hardcoded 12100, but that silently assumes a 5V supply — on 3V3 the true
    centre sits near 8000 and would be "corrected" to a value the stick can
    never reach. Stillness identifies a resting stick without assuming a rail.
    """
    if spread > DEADBAND:
        print("  centre %s=%d spread=%d  REJECTED (stick being moved?) -> %d"
              % (label, c, spread, CENTRE_TYPICAL))
        return CENTRE_TYPICAL
    print("  centre %s=%d (spread %d)" % (label, c, spread))
    return c


def normalise(raw, centre, full_tilt):
    """Raw counts -> -1000..+1000, deadband already removed. 0 means centred."""
    d = raw - centre
    mag = abs(d)
    if mag <= DEADBAND:
        return 0
    span = max(1, full_tilt - DEADBAND)
    val = int(min(mag - DEADBAND, span) * 1000 / span)
    return val if d > 0 else -val


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--host", default=UNO_HOST, help="default %s" % UNO_HOST)
    ap.add_argument("--port", type=int, default=UNO_PORT)
    ap.add_argument("--bus", type=int, default=1, help="I2C bus, default 1")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x48)
    ap.add_argument("--hz", type=float, default=50.0, help="send rate, default 50")
    ap.add_argument("--scan", action="store_true", help="list I2C devices and exit")
    ap.add_argument("--print-only", action="store_true", help="read but do not send")
    ap.add_argument("--raw", action="store_true", help="also show raw counts")
    ap.add_argument("--full-tilt", type=int, default=None, metavar="COUNTS",
                    help="raw counts from centre for 100%%; "
                         "default is derived from the measured centre")
    ap.add_argument("--no-ack", action="store_true", help="fire and forget")
    args = ap.parse_args()

    if args.scan:
        found = scan(args.bus)
        if not found:
            print("no I2C devices on bus %d - check SDA->pin 3, SCL->pin 5, "
                  "3V3->pin 1, GND->pin 6" % args.bus)
            return 1
        for a in found:
            note = "  <- ADS1x15" if 0x48 <= a <= 0x4B else ""
            print("  0x%02X%s" % (a, note))
        return 0

    ads = ADS1x15(args.bus, args.addr)
    print("ADS1x15 on bus %d at 0x%02X" % (args.bus, args.addr))

    print("centring - leave the stick alone...")
    cx, sx = sample_centre(ads, CH_X)
    cy, sy = sample_centre(ads, CH_Y)
    centre_x = validate_centre(cx, sx, "X")
    centre_y = validate_centre(cy, sy, "Y")

    full_tilt = args.full_tilt or max(1000, int(centre_x * FULL_TILT_RATIO))
    print("  full tilt = %d counts%s"
          % (full_tilt, "" if args.full_tilt
             else " (%.3f x centre - scales with the supply rail)"
                  % FULL_TILT_RATIO))

    link = None
    if not args.print_only:
        link = UnoLink(args.host, args.port)
        print("sending to %s at %g Hz - Ctrl-C to stop" % (link.target, args.hz))
    else:
        print("print-only: nothing will be sent")
    print("x\ty\t%srtt" % ("rawX\trawY\t" if args.raw else ""))

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    next_report = time.monotonic()
    try:
        while True:
            cycle = time.monotonic()
            rx = ads.read(CH_X)
            ry = ads.read(CH_Y)
            x = normalise(rx, centre_x, full_tilt)
            y = normalise(ry, centre_y, full_tilt)

            rtt = None
            if link is not None:
                rtt = link.send("J %d %d" % (x, y), wait_ack=not args.no_ack)

            # One line per 100 ms. At 50 Hz a line per sample scrolls unreadably
            # and the print costs more than the I2C read.
            if cycle >= next_report:
                next_report = cycle + 0.1
                cols = [str(x), str(y)]
                if args.raw:
                    cols += [str(rx), str(ry)]
                if link is not None:
                    cols.append("%.2f ms" % rtt if rtt is not None else "no ack")
                    cols.append("loss=%.1f%%" % link.loss_pct)
                print("\t".join(cols))

            slack = period - (time.monotonic() - cycle)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        print()
    finally:
        # Don't leave the motor coasting on the last command until the sketch's
        # 300 ms failsafe notices. Say zero explicitly, several times in case one
        # datagram is lost on the way out.
        if link is not None:
            for _ in range(5):
                link.send("J 0 0", wait_ack=False)
                time.sleep(0.01)
            print("sent=%d acked=%d loss=%.1f%%"
                  % (link.sent, link.acked, link.loss_pct))
            link.close()
        ads.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
