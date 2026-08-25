#!/usr/bin/env python3
"""ADS1115 reads over the KERNEL's bit-banged I2C bus (/dev/i2c-3).

WHY THIS EXISTS. The panel's ADS1115 is wired with SDA/SCL CROSSED into the
header, so the Pi's hardware i2c-1 (fixed pin roles) can never see it, and for a
year the only path was a userspace gpiod bit-bang (i2c_bitbang_probe.Bus). That
worked but was murdered by load: every clock edge is a separate ioctl round trip
from a normal-priority thread, and with two software H.264 decoders saturating
the cores the 3-channel poll measured 31.7 ms idle but ~190 ms under the real
viewer (5.2 Hz joystick, 2026-08-17). A control input running at 5 Hz is the
"data is lagging" complaint, verbatim.

The kernel's i2c-gpio driver does the same bit-banging but in KERNEL space,
with microsecond timing and no per-edge context switch -- and it takes ANY two
GPIOs, so the crossed wiring stops mattering: /boot/firmware/config.txt maps
SDA=GPIO3, SCL=GPIO2, i.e. the wires exactly as they are:

    dtparam=i2c_arm=off
    dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=3,i2c_gpio_scl=2,i2c_gpio_delay_us=2

A full transfer is then one ioctl, so a 3-channel poll costs ~12 syscalls
instead of ~1500 and the poll rate holds under any decode load.

NO THIRD-PARTY DEPENDENCIES, deliberately: smbus2 is not in the base image and
Debian 13's system Python refuses pip installs. Plain os.read/os.write on the
device node is all the ADS1115 needs -- its register pointer persists across
transactions, so "write pointer, then read two bytes" works without repeated
starts.

inputs.py tries this first and falls back to the userspace bit-bang if the
device node is missing (overlay not configured / older SD card), so a Pi
without the config.txt change keeps working exactly as before.
"""

from __future__ import annotations

import fcntl
import os
import time

# From <linux/i2c-dev.h>. Stable ABI, same values on every architecture.
I2C_SLAVE = 0x0703

ADDR = 0x48
REG_CONV, REG_CONFIG = 0x00, 0x01

# Same config word as the bit-bang path (i2c_bitbang_read.py), so readings are
# directly comparable: PGA +/-6.144V, single-shot, 860 SPS, comparator off.
CFG_BASE = 0x8000 | (0 << 9) | (1 << 8) | (7 << 5) | 0x0003

# 860 SPS -> ~1.2 ms per conversion. Waited out with ONE sleep and then verified
# via the OS bit rather than assumed: under load a sleep can stretch, which is
# harmless, but it can never SHRINK below the requested time, so 2 ms guarantees
# the conversion is done and the OS check is a guard against a wedged chip
# rather than a timing crutch.
CONV_WAIT_S = 0.002
CONV_TIMEOUT_S = 0.05


class ADS1115:
    """One ADS1115 on a kernel i2c device node. Same sample() shape as the
    bit-bang path: counts on success, None on a failed transfer."""

    def __init__(self, dev="/dev/i2c-3", addr=ADDR):
        self.dev = dev
        self._fd = os.open(dev, os.O_RDWR)
        try:
            fcntl.ioctl(self._fd, I2C_SLAVE, addr)
        except OSError:
            os.close(self._fd)
            raise

    def close(self):
        try:
            os.close(self._fd)
        except OSError:
            pass

    # -- register access ------------------------------------------------------

    def _write_reg(self, reg, val):
        os.write(self._fd, bytes([reg, (val >> 8) & 0xFF, val & 0xFF]))

    def _read_reg(self, reg):
        os.write(self._fd, bytes([reg]))
        hi, lo = os.read(self._fd, 2)
        return (hi << 8) | lo

    # -- the one public operation ---------------------------------------------

    def probe(self):
        """True if the chip answers at all. Used once at open to pick a path."""
        try:
            self._read_reg(REG_CONFIG)
            return True
        except OSError:
            return False

    def sample(self, ch):
        """Single-shot read of channel `ch` (0-3), signed counts or None.

        Every OSError is caught and becomes None: a NACK mid-transfer (EREMOTEIO
        / ENXIO) is the same "this reading never happened" that the bit-bang
        path reports, and inputs.py's dead-read counter and validators handle
        the rest. Nothing here retries -- the poll loop IS the retry.
        """
        try:
            self._write_reg(REG_CONFIG, CFG_BASE | ((4 + ch) << 12))
            time.sleep(CONV_WAIT_S)
            # OS bit high = conversion complete. Normally true on the first
            # look; the loop only exists for a chip that is wedged or a clock
            # that is being stretched into next week.
            deadline = time.monotonic() + CONV_TIMEOUT_S
            while not (self._read_reg(REG_CONFIG) & 0x8000):
                if time.monotonic() > deadline:
                    return None
                time.sleep(0.001)
            raw = self._read_reg(REG_CONV)
        except OSError:
            return None
        return raw - 65536 if raw > 32767 else raw


if __name__ == "__main__":
    import sys
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/i2c-3"
    adc = ADS1115(dev)
    if not adc.probe():
        print(f"no ACK from 0x{ADDR:02x} on {dev}")
        sys.exit(1)
    print(f"ADS1115 answering on {dev}. 5 s of A0/A1/A2 (x / y / pot):")
    t0 = time.monotonic()
    n = 0
    while time.monotonic() - t0 < 5.0:
        vals = [adc.sample(ch) for ch in (0, 1, 2)]
        n += 1
        if n % 10 == 0:
            print("  " + "  ".join(
                f"A{ch}={v if v is not None else 'ERR':>6}"
                for ch, v in enumerate(vals)))
    print(f"{n} polls in 5 s = {n / 5.0:.1f} Hz")
    adc.close()
