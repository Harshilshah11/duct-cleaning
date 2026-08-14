#!/usr/bin/env python3
"""
Bit-bang I2C on GPIO2/GPIO3 to tell three failure modes apart without touching
the hardware:

  1. wires swapped   -> a scan succeeds in the REVERSED orientation
  2. SDA/SCL shorted -> driving one line low pulls the other low too
  3. genuinely open  -> nothing responds either way

The kernel i2c-1 driver holds these pins in ALT0. Requesting them as GPIO
re-muxes them; `pinctrl set 2 a0 / 3 a0` at the end puts them back, and the
script always restores even on failure.

Open-drain is done by flipping direction: input = released (pull-ups take it
high), output-low = driven. A line is never driven high.
"""

import sys
import time

import gpiod
from gpiod.line import Bias, Direction, Value

CHIP = "/dev/gpiochip0"
SDA_DEFAULT, SCL_DEFAULT = 2, 3
DELAY = 20e-6          # ~25 kHz. I2C has no minimum clock, so slow is safe.
STRETCH_TIMEOUT = 0.01

IN = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
OUT_LOW = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)


class Bus:
    """Software I2C master over two gpiod lines."""

    def __init__(self, req, sda, scl):
        self.req, self.sda, self.scl = req, sda, scl
        self._state = {sda: IN, scl: IN}

    # release = high-Z, the external pull-up decides the level
    def release(self, line):
        self._state[line] = IN
        self.req.reconfigure_lines({self.sda: self._state[self.sda],
                                    self.scl: self._state[self.scl]})

    def drive_low(self, line):
        self._state[line] = OUT_LOW
        self.req.reconfigure_lines({self.sda: self._state[self.sda],
                                    self.scl: self._state[self.scl]})

    def read(self, line):
        return self.req.get_value(line) == Value.ACTIVE

    def reset_state(self):
        self._state = {self.sda: IN, self.scl: IN}
        self.req.reconfigure_lines({self.sda: IN, self.scl: IN})

    # --- primitives ---------------------------------------------------------
    def _scl_release(self):
        """Release SCL and honour clock stretching."""
        self.release(self.scl)
        t0 = time.monotonic()
        while not self.read(self.scl):
            if time.monotonic() - t0 > STRETCH_TIMEOUT:
                return False
        time.sleep(DELAY)
        return True

    def _scl_low(self):
        self.drive_low(self.scl)
        time.sleep(DELAY)

    def start(self):
        self.release(self.sda)
        self._scl_release()
        self.drive_low(self.sda)
        time.sleep(DELAY)
        self._scl_low()

    def stop(self):
        self.drive_low(self.sda)
        self._scl_release()
        self.release(self.sda)
        time.sleep(DELAY)

    def write_bit(self, bit):
        if bit:
            self.release(self.sda)
        else:
            self.drive_low(self.sda)
        time.sleep(DELAY)
        self._scl_release()
        self._scl_low()

    def read_bit(self):
        self.release(self.sda)
        time.sleep(DELAY)
        self._scl_release()
        bit = self.read(self.sda)
        self._scl_low()
        return bit

    def write_byte(self, val):
        for i in range(8):
            self.write_bit(bool(val & (0x80 >> i)))
        return not self.read_bit()  # ACK = device pulled SDA low

    def probe(self, addr):
        self.start()
        ack = self.write_byte((addr << 1) | 0)
        self.stop()
        return ack


def scan(req, sda, scl, label):
    bus = Bus(req, sda, scl)
    bus.reset_state()
    idle_sda, idle_scl = bus.read(sda), bus.read(scl)
    print("  idle levels: SDA(gpio%d)=%s  SCL(gpio%d)=%s"
          % (sda, "hi" if idle_sda else "LOW", scl, "hi" if idle_scl else "LOW"))
    if not (idle_sda and idle_scl):
        print("  !! a line is stuck LOW - shorted to GND or a device is holding it")

    found = []
    for addr in range(0x03, 0x78):
        try:
            if bus.probe(addr):
                found.append(hex(addr))
        except Exception:
            pass
    print("  %s -> %s" % (label, ", ".join(found) if found else "no ACK from any address"))
    bus.reset_state()
    return found


def short_test(req, a, b):
    """Drive line a low, see whether line b follows (i.e. they are shorted)."""
    req.reconfigure_lines({a: IN, b: IN})
    time.sleep(0.001)
    before = req.get_value(b) == Value.ACTIVE
    req.reconfigure_lines({a: OUT_LOW, b: IN})
    time.sleep(0.002)
    during = req.get_value(b) == Value.ACTIVE
    req.reconfigure_lines({a: IN, b: IN})
    return before, during


def main():
    req = gpiod.request_lines(
        CHIP, consumer="i2c-bitbang-probe",
        config={SDA_DEFAULT: IN, SCL_DEFAULT: IN},
    )
    try:
        print("=== normal orientation (SDA=gpio2, SCL=gpio3) ===")
        normal = scan(req, SDA_DEFAULT, SCL_DEFAULT, "scan")

        print()
        print("=== REVERSED orientation (SDA=gpio3, SCL=gpio2) ===")
        print("  an ACK here means the two signal wires are swapped")
        swapped = scan(req, SCL_DEFAULT, SDA_DEFAULT, "scan")

        print()
        print("=== short test between the two lines ===")
        b_before, b_during = short_test(req, SDA_DEFAULT, SCL_DEFAULT)
        print("  drive gpio2 low -> gpio3 reads %s (was %s)"
              % ("LOW" if not b_during else "hi", "hi" if b_before else "LOW"))
        a_before, a_during = short_test(req, SCL_DEFAULT, SDA_DEFAULT)
        print("  drive gpio3 low -> gpio2 reads %s (was %s)"
              % ("LOW" if not a_during else "hi", "hi" if a_before else "LOW"))
        if not b_during or not a_during:
            print("  !! SDA and SCL appear SHORTED TOGETHER")
        else:
            print("  lines are independent - no short between them")

        print()
        print("=== verdict ===")
        if normal:
            print("  device(s) answer in the NORMAL orientation:", ", ".join(normal))
        elif swapped:
            print("  device(s) answer REVERSED:", ", ".join(swapped),
                  "-> SDA and SCL are swapped, move pin 3 <-> pin 5")
        else:
            print("  no ACK in either orientation, no short, both lines idle high.")
            print("  The Pi is driving the bus correctly and nothing is listening,")
            print("  so the break is between the header and the chip's own pins:")
            print("  an open jumper on SDA or SCL, or the module's GND not returning.")
    finally:
        req.release()
        # Hand the pins back to the hardware i2c controller.
        import subprocess
        for pin in (SDA_DEFAULT, SCL_DEFAULT):
            subprocess.run(["/usr/bin/pinctrl", "set", str(pin), "a0", "pu"],
                           capture_output=True)
        out = subprocess.run(["/usr/bin/pinctrl", "get", "2,3"],
                             capture_output=True, text=True).stdout
        print()
        print("=== pins restored ===")
        print(out.rstrip())


if __name__ == "__main__":
    sys.exit(main())
