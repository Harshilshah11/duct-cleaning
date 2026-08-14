#!/usr/bin/env python3
"""
Analog joystick -> normalized axes, read through an ADS1115 (guide Step 5).

A thumb joystick is two potentiometers. The Pi has no analog inputs at all, so
the ADS1115 on I2C does the conversion and this module turns its raw counts into
the -1.0 .. +1.0 pair that a motor command wants.

Three decisions here that are not obvious:

**Raw I2C, not Blinka.** The Adafruit CircuitPython stack (blinka + ads1x15) is
the usual answer and pulls a dependency tree that has to match the interpreter -
this Pi is Debian 13 with Python 3.13, where that stack is a gamble. Talking to
the chip directly is four registers and no dependencies beyond python3-smbus2,
which is packaged. Nothing is lost: the datasheet's config word is spelled out
below, so this is *more* readable than the library call it replaces.

**Its own thread**, like link.py. A single-shot conversion at 250 SPS takes 4 ms
and two axes take 8 ms. Doing that inside the 50 Hz command timer would spend
40% of every period parked in an I2C read, and any bus hiccup would stall the Qt
event loop outright. The thread samples continuously; readers take the newest
value and never wait.

**Failure reads as centre, not as the last value.** If the bus stops answering,
`x`/`y` go to 0.0 and `ok` goes False. Holding the last good sample would leave a
robot driving at whatever it was doing when the ADC died - the failure mode this
whole link is supposed to prevent. The sender should check `ok` and stop sending
so the Arduino's 300 ms failsafe takes over.

Wiring (ADDR to GND = address 0x48):

    ADS1115         Pi 4                 Joystick
    VDD ----------- 3V3 (pin 1)          +5V/VCC -- 3V3  (see note)
    GND ----------- GND (pin 6)          GND ------ GND
    SCL ----------- GPIO3 / SCL (pin 5)  VRx ------ A0
    SDA ----------- GPIO2 / SDA (pin 3)  VRy ------ A1
    ADDR ---------- GND                  SW ------- GPIO17 (pin 11), optional

Power the joystick from the same 3V3 the ADS1115 uses. On 5V its wiper would
swing past the ADS1115's VDD+0.3 V absolute maximum at full deflection, and the
readings above 3.3 V would clip anyway.

I2C has to be enabled once (`sudo raspi-config nonint do_i2c 0`, then reboot);
`--probe` below says so plainly if it is not.

Check it from a terminal, no GUI needed:

    python3 joystick.py --probe          # what is on the bus, is the wiring live
    python3 joystick.py                  # live axes at 20 Hz until Ctrl-C
    python3 joystick.py --raw            # raw counts, for checking travel limits
    python3 joystick.py --calibrate      # capture centre + travel, save to JSON

Exits non-zero if the ADC never answered, so it works in a shell test:

    python3 joystick.py --probe && echo "joystick is alive"
"""

import argparse
import json
import os
import struct
import sys
import threading
import time

# --- Bus -----------------------------------------------------------------------
# 0x48 is ADDR->GND. The other three straps are 0x49 (VDD), 0x4A (SDA), 0x4B (SCL).
I2C_BUS = int(os.environ.get("ADS_I2C_BUS", "1"))
ADS_ADDR = int(os.environ.get("ADS_ADDR", "0x48"), 0)

# Which ADS1115 input each axis is on.
CH_X = int(os.environ.get("JOY_CH_X", "0"))
CH_Y = int(os.environ.get("JOY_CH_Y", "1"))

# Optional push-button (the click under the stick). Set JOY_SW_PIN=-1 to skip it.
SW_PIN = int(os.environ.get("JOY_SW_PIN", "17"))

# --- ADS1115 registers (datasheet section 9.6) ---------------------------------
REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# Config word, bit by bit. Assembled in _start_conversion().
CFG_OS_SINGLE = 0x8000      # bit 15 w: begin a single conversion / r: 1 = idle
_MUX_SINGLE_ENDED = 0x4000  # bits 14:12, 100b + channel = AINx vs GND
CFG_MODE_SINGLE = 0x0100    # bit 8, one conversion then power down
CFG_COMP_OFF = 0x0003       # bits 1:0, comparator disabled

# Bits 11:9. A joystick on a 3V3 rail swings 0 .. 3.3 V, so the range must be at
# least that: +/-4.096 V is the smallest one that fits without clipping the ends
# of the stick's travel. +/-6.144 V also fits and throws away a third of the
# resolution for nothing.
PGA_4_096V = 0x0200
FULL_SCALE_V = 4.096

# Bits 7:5. 250 SPS = 4 ms per conversion, so both axes cost 8 ms and a 50 Hz
# command loop still has 12 ms of slack. 128 SPS (the chip default) would take
# 15.6 ms for the pair and leave almost nothing; 860 SPS is available but its
# extra noise has to be filtered back out again.
DATA_RATE_BITS = {8: 0x0000, 16: 0x0020, 32: 0x0040, 64: 0x0060,
                  128: 0x0080, 250: 0x00A0, 475: 0x00C0, 860: 0x00E0}
SAMPLE_RATE = int(os.environ.get("ADS_SPS", "250"))

# --- Feel ----------------------------------------------------------------------
# Fraction of travel around centre that reports exactly 0.0. A centred stick
# still jitters a few hundred counts, and without this the robot creeps.
DEADZONE = float(os.environ.get("JOY_DEADZONE", "0.08"))

# Exponential smoothing on the normalized axes. 1.0 disables it. This is a
# light touch on purpose - every bit of smoothing is added control latency.
SMOOTHING = float(os.environ.get("JOY_SMOOTHING", "0.4"))

# Screen/stick convention: pushing the stick away from you should be +1.0, and
# on most modules that direction reads *lower* counts, hence the flip.
INVERT_Y = os.environ.get("JOY_INVERT_Y", "1") == "1"
INVERT_X = os.environ.get("JOY_INVERT_X", "0") == "1"

POLL_HZ = float(os.environ.get("JOY_POLL_HZ", "100"))

# Where --calibrate writes. Loaded automatically when present.
CAL_PATH = os.path.expanduser(
    os.environ.get("JOY_CAL_PATH", "~/.config/arnobot/joystick_cal.json")
)

# Counts are 16-bit signed; a single-ended reading uses the positive half only.
COUNTS_FULL_SCALE = 32767


class ADS1115:
    """Minimal single-shot driver. One conversion per read_channel() call."""

    def __init__(self, bus=I2C_BUS, address=ADS_ADDR, sample_rate=SAMPLE_RATE):
        if sample_rate not in DATA_RATE_BITS:
            raise ValueError(
                f"sample rate {sample_rate} is not one of {sorted(DATA_RATE_BITS)}"
            )
        self.address = address
        self.sample_rate = sample_rate

        # Worst case wait for a conversion, with headroom for the chip's
        # internal oscillator being up to ~10% slow.
        self._conversion_s = (1.0 / sample_rate) * 1.3 + 0.001

        try:
            from smbus2 import SMBus
        except ImportError:
            try:
                from smbus import SMBus  # older Raspberry Pi OS name
            except ImportError:
                raise RuntimeError(
                    "no I2C library - install it with:\n"
                    "    sudo apt install python3-smbus2"
                ) from None

        try:
            self._bus = SMBus(bus)
        except (FileNotFoundError, PermissionError) as exc:
            raise RuntimeError(
                f"cannot open /dev/i2c-{bus} ({exc}). Enable I2C with:\n"
                "    sudo raspi-config nonint do_i2c 0 && sudo reboot"
            ) from None

    # -- register access -------------------------------------------------------

    def _write_config(self, value):
        # The ADS1115 is big-endian and SMBus word transfers are little-endian,
        # so write_word_data() would silently swap the bytes and configure the
        # wrong everything. Block transfers move the bytes in the order given.
        self._bus.write_i2c_block_data(
            self.address, REG_CONFIG, [(value >> 8) & 0xFF, value & 0xFF]
        )

    def _read_register(self, reg):
        hi, lo = self._bus.read_i2c_block_data(self.address, reg, 2)
        return (hi << 8) | lo

    # -- conversions -----------------------------------------------------------

    def read_channel(self, channel):
        """One single-ended conversion on AIN<channel>. Returns signed counts."""
        if not 0 <= channel <= 3:
            raise ValueError(f"ADS1115 has channels 0-3, got {channel}")

        config = (
            CFG_OS_SINGLE
            | (_MUX_SINGLE_ENDED | (channel << 12))
            | PGA_4_096V
            | CFG_MODE_SINGLE
            | DATA_RATE_BITS[self.sample_rate]
            | CFG_COMP_OFF
        )
        self._write_config(config)

        # Poll the OS bit rather than sleeping a fixed time: it reads back 1 the
        # moment the conversion lands, which is usually sooner than the worst
        # case above, and it catches a chip that has stopped converting instead
        # of returning the previous sample as if it were fresh.
        deadline = time.monotonic() + self._conversion_s
        while time.monotonic() < deadline:
            if self._read_register(REG_CONFIG) & CFG_OS_SINGLE:
                break
        else:
            raise OSError(
                f"ADS1115 at 0x{self.address:02x} did not finish a conversion "
                f"within {self._conversion_s * 1000:.1f} ms"
            )

        raw = self._read_register(REG_CONVERSION)
        # Two's complement. Single-ended inputs below ground read slightly
        # negative rather than clamping, so this really can be < 0.
        return struct.unpack(">h", struct.pack(">H", raw))[0]

    def read_voltage(self, channel):
        return self.read_channel(channel) * FULL_SCALE_V / COUNTS_FULL_SCALE

    def read_config(self):
        """The config register as the chip currently holds it. Any successful
        read proves the ADC is acknowledging its address."""
        return self._read_register(REG_CONFIG)

    def close(self):
        self._bus.close()


class Button:
    """The stick's push switch, if one is wired. Absent is not an error."""

    def __init__(self, pin=SW_PIN):
        self.pin = pin
        self.available = False
        self._device = None

        if pin is None or pin < 0:
            return
        try:
            # gpiozero over lgpio is the supported path on Debian 13; RPi.GPIO
            # no longer works on current kernels.
            from gpiozero import Button as _GpioButton

            # The module has no pull-up of its own - SW shorts to GND when
            # pressed and floats otherwise, so the Pi has to hold it high.
            self._device = _GpioButton(pin, pull_up=True, bounce_time=0.02)
            self.available = True
        except Exception:
            # Missing library, pin already claimed, no /dev/gpiochip - none of
            # it should stop the axes from working.
            self._device = None

    @property
    def pressed(self):
        if self._device is None:
            return False
        try:
            return bool(self._device.is_pressed)
        except Exception:
            return False

    def close(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass


class Calibration:
    """Centre and travel limits per axis, in raw counts."""

    def __init__(self, x_centre=None, y_centre=None):
        # Defaults assume a 3V3 stick sitting mid-travel until measured.
        nominal = int(1.65 / FULL_SCALE_V * COUNTS_FULL_SCALE)
        self.x_centre = x_centre if x_centre is not None else nominal
        self.y_centre = y_centre if y_centre is not None else nominal

        # Travel is learned as the stick is moved. Seeding min and max at the
        # centre means the envelope only ever grows outward, so a stick that has
        # not been pushed to a corner yet simply reports less than full scale
        # instead of reporting nonsense.
        self.x_min = self.x_max = self.x_centre
        self.y_min = self.y_max = self.y_centre

    def observe(self, x_raw, y_raw):
        self.x_min = min(self.x_min, x_raw)
        self.x_max = max(self.x_max, x_raw)
        self.y_min = min(self.y_min, y_raw)
        self.y_max = max(self.y_max, y_raw)

    def to_dict(self):
        return {
            "x_centre": self.x_centre, "x_min": self.x_min, "x_max": self.x_max,
            "y_centre": self.y_centre, "y_min": self.y_min, "y_max": self.y_max,
        }

    @classmethod
    def load(cls, path=CAL_PATH):
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError):
            return None
        cal = cls(data.get("x_centre"), data.get("y_centre"))
        cal.x_min = data.get("x_min", cal.x_centre)
        cal.x_max = data.get("x_max", cal.x_centre)
        cal.y_min = data.get("y_min", cal.y_centre)
        cal.y_max = data.get("y_max", cal.y_centre)
        return cal

    def save(self, path=CAL_PATH):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.to_dict(), fh, indent=2)
        return path


def _normalize(raw, centre, low, high, deadzone, invert):
    """Raw counts -> -1.0 .. +1.0, with the deadzone taken out of the travel.

    Each side of centre is scaled by its own span. A stick is never mechanically
    symmetric and its centre is never exactly mid-scale, so using one span for
    both directions makes full deflection read ~0.8 one way and clip at 1.0 the
    other.
    """
    span = (high - centre) if raw >= centre else (centre - low)
    if span <= 0:
        return 0.0

    value = (raw - centre) / span
    value = max(-1.0, min(1.0, value))

    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0

    # Rescale what is left of the travel back to a full 0..1, so the output
    # starts moving from zero at the edge of the deadzone rather than jumping
    # straight to `deadzone` the instant the stick crosses it.
    scaled = (magnitude - deadzone) / (1.0 - deadzone)
    scaled = min(1.0, scaled)
    if value < 0:
        scaled = -scaled
    return -scaled if invert else scaled


class Joystick:
    """Background joystick reader. Start it, then read the attributes.

    Mirrors LinkMonitor in link.py: a daemon thread owns the hardware and the
    GUI only ever touches plain attributes.
    """

    def __init__(self, bus=I2C_BUS, address=ADS_ADDR, ch_x=CH_X, ch_y=CH_Y,
                 sw_pin=SW_PIN, poll_hz=POLL_HZ, deadzone=DEADZONE,
                 smoothing=SMOOTHING, calibration=None):
        self.ch_x = ch_x
        self.ch_y = ch_y
        self.poll_hz = poll_hz
        self.deadzone = deadzone
        self.smoothing = max(0.01, min(1.0, smoothing))

        self.x = 0.0
        self.y = 0.0
        self.x_raw = 0
        self.y_raw = 0
        # None until the first sample, so a caller can tell "not read yet" from
        # "read and the stick is centred" - the same distinction LinkMonitor
        # draws with connected=None.
        self.ok = None
        self.samples = 0
        self.errors = 0
        self.last_error = None
        self.last_ok = None

        self._adc = ADS1115(bus, address)
        self.button = Button(sw_pin)

        # A saved calibration counts as already centred, so start() does not
        # spend a second re-measuring something that was measured properly.
        saved = calibration or Calibration.load()
        self.cal = saved or Calibration()
        self._calibrated = saved is not None

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    # -- lifecycle -------------------------------------------------------------

    def start(self, settle=True):
        if self._thread and self._thread.is_alive():
            return
        if settle and not self._calibrated:
            self.capture_centre()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="joystick", daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self.button.close()
        self._adc.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_exc):
        self.stop()

    # -- calibration -----------------------------------------------------------

    def capture_centre(self, samples=24):
        """Average the resting position. The stick must not be touched.

        Called once at startup because the centre drifts with temperature and
        with however the module was soldered - assuming mid-scale leaves a
        permanent lean in one direction.
        """
        xs, ys = [], []
        for _ in range(samples):
            try:
                xs.append(self._adc.read_channel(self.ch_x))
                ys.append(self._adc.read_channel(self.ch_y))
            except OSError:
                continue
        if xs and ys:
            self.cal.x_centre = int(sum(xs) / len(xs))
            self.cal.y_centre = int(sum(ys) / len(ys))
            self.cal.x_min = self.cal.x_max = self.cal.x_centre
            self.cal.y_min = self.cal.y_max = self.cal.y_centre
        return self.cal

    # -- worker ----------------------------------------------------------------

    def _run(self):
        period = 1.0 / self.poll_hz if self.poll_hz > 0 else 0.0
        while not self._stop.is_set():
            cycle = time.monotonic()
            self._sample_once()
            slack = period - (time.monotonic() - cycle)
            if slack > 0:
                self._stop.wait(slack)

    def _sample_once(self):
        try:
            x_raw = self._adc.read_channel(self.ch_x)
            y_raw = self._adc.read_channel(self.ch_y)
        except OSError as exc:
            with self._lock:
                self.errors += 1
                self.last_error = str(exc)
                # Centre on failure. See the module docstring - a held-over
                # value is a robot that keeps driving.
                self.x = 0.0
                self.y = 0.0
                self.ok = False
            return

        self.cal.observe(x_raw, y_raw)
        x = _normalize(x_raw, self.cal.x_centre, self.cal.x_min, self.cal.x_max,
                       self.deadzone, INVERT_X)
        y = _normalize(y_raw, self.cal.y_centre, self.cal.y_min, self.cal.y_max,
                       self.deadzone, INVERT_Y)

        with self._lock:
            a = self.smoothing
            self.x += a * (x - self.x)
            self.y += a * (y - self.y)
            # Smoothing leaves an exponential tail that never quite reaches
            # zero; snap it so a released stick reports a real, exact stop.
            if x == 0.0 and abs(self.x) < 0.005:
                self.x = 0.0
            if y == 0.0 and abs(self.y) < 0.005:
                self.y = 0.0
            self.x_raw = x_raw
            self.y_raw = y_raw
            self.samples += 1
            self.last_ok = time.monotonic()
            self.ok = True

    # -- reading ---------------------------------------------------------------

    def sample(self):
        """Consistent (x, y, pressed, ok) snapshot.

        Use this rather than reading .x and .y one after the other: the worker
        can replace y between the two reads, and a mismatched pair is a robot
        that turns while it was told to go straight.
        """
        with self._lock:
            return self.x, self.y, self.button.pressed, bool(self.ok)

    def command(self, fmt="X={x:+.3f} Y={y:+.3f} B={b:d}"):
        """The string uno_link.send() expects. Empty when the ADC is unhealthy,
        so the caller can simply not send and let the Arduino failsafe stop."""
        x, y, pressed, ok = self.sample()
        if not ok:
            return ""
        return fmt.format(x=x, y=y, b=int(pressed))


def probe(bus=I2C_BUS, address=ADS_ADDR):
    """Report what is actually on the bus. Returns True if the ADC answered."""
    print(f"i2c bus     : /dev/i2c-{bus}")
    if not os.path.exists(f"/dev/i2c-{bus}"):
        print("              MISSING - enable I2C:")
        print("              sudo raspi-config nonint do_i2c 0 && sudo reboot")
        return False

    try:
        adc = ADS1115(bus, address)
    except RuntimeError as exc:
        print(f"              {exc}")
        return False

    print(f"ads1115     : 0x{address:02x}", end=" ")
    try:
        config = adc.read_config()
    except OSError:
        print("NO RESPONSE")
        print("              check SDA/SCL/VDD/GND and the ADDR strap")
        print("              (0x48=GND  0x49=VDD  0x4A=SDA  0x4B=SCL)")
        adc.close()
        return False
    print(f"OK (config=0x{config:04x})")

    for ch in (0, 1, 2, 3):
        try:
            counts = adc.read_channel(ch)
        except OSError as exc:
            print(f"  AIN{ch}      : read failed - {exc}")
            continue
        volts = counts * FULL_SCALE_V / COUNTS_FULL_SCALE
        note = ""
        if ch in (CH_X, CH_Y):
            note = "  <- joystick " + ("X" if ch == CH_X else "Y")
            if volts < 0.05:
                note += "  (reads 0 V - is VCC connected?)"
        print(f"  AIN{ch}      : {counts:6d} counts  {volts:5.3f} V{note}")

    button = Button()
    print(f"button      : GPIO{SW_PIN} " +
          ("available" if button.available else "not available (axes unaffected)"))
    button.close()

    adc.close()
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--probe", action="store_true",
                    help="check the bus and wiring, then exit")
    ap.add_argument("--raw", action="store_true",
                    help="show raw counts alongside the normalized axes")
    ap.add_argument("--calibrate", action="store_true",
                    help="capture centre and travel, then save")
    ap.add_argument("--hz", type=float, default=20.0,
                    help="display rate, default 20 (the reader itself runs at "
                         f"{POLL_HZ:g} Hz regardless)")
    ap.add_argument("--deadzone", type=float, default=DEADZONE)
    args = ap.parse_args()

    if args.probe:
        return 0 if probe() else 1

    try:
        joy = Joystick(deadzone=args.deadzone)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1

    if args.calibrate:
        print("centring - do not touch the stick ...")
        joy.capture_centre()
        print(f"  centre  X={joy.cal.x_centre}  Y={joy.cal.y_centre}")
        print("now sweep the stick to all four corners a few times, "
              "then release and press Ctrl-C")
        joy.start(settle=False)
        try:
            while True:
                time.sleep(0.1)
                print(f"\r  X {joy.cal.x_min:5d}..{joy.cal.x_max:5d}   "
                      f"Y {joy.cal.y_min:5d}..{joy.cal.y_max:5d}", end="", flush=True)
        except KeyboardInterrupt:
            print()
        # Travel under about a third of full scale means the stick never really
        # moved - saving that would permanently scale every later reading up.
        if (joy.cal.x_max - joy.cal.x_min) < COUNTS_FULL_SCALE // 3:
            print("travel looks too small to be a real sweep - not saving",
                  file=sys.stderr)
            joy.stop()
            return 1
        path = joy.cal.save()
        print(f"saved {path}")
        joy.stop()
        return 0

    print("centring - do not touch the stick ...")
    joy.start()
    print(f"  centre  X={joy.cal.x_centre}  Y={joy.cal.y_centre}")
    print("move the stick — Ctrl-C to stop")

    period = 1.0 / args.hz if args.hz > 0 else 0.05
    try:
        while True:
            time.sleep(period)
            x, y, pressed, ok = joy.sample()
            if not ok:
                print(f"\r  ADC ERROR  {joy.last_error or ''}"[:110],
                      end="", flush=True)
                continue
            bar_x = _bar(x)
            bar_y = _bar(y)
            extra = f"  raw {joy.x_raw:5d},{joy.y_raw:5d}" if args.raw else ""
            print(f"\r  X {x:+.3f} {bar_x}   Y {y:+.3f} {bar_y}   "
                  f"{'BTN' if pressed else '   '}{extra}", end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        joy.stop()

    print(f"samples={joy.samples} errors={joy.errors}")
    return 0 if joy.samples else 1


def _bar(value, width=11):
    """Little centre-anchored meter, so the terminal shows the stick's position."""
    mid = width // 2
    cells = ["-"] * width
    cells[mid] = "|"
    pos = int(round(mid + value * mid))
    cells[max(0, min(width - 1, pos))] = "#"
    return "".join(cells)


if __name__ == "__main__":
    sys.exit(main())
