#!/usr/bin/env python3
"""
Pi -> Arduino Uno motor link over the USB tether.

Replaces the Ethernet path in uno_link.py for the tethered Uno. Same command
framing so the two stay interchangeable:

    Pi  -> Uno   "CMD <seq> M <left> <right>\\n"    left/right = -255..255
    Pi  -> Uno   "CMD <seq> STOP\\n"
    Uno -> Pi    "ACK <seq>\\n"

Arcade mixing happens HERE, not on the Uno: steering can then be retuned
without a reflash, and the code sitting next to the motors stays small enough
to audit. The sketch only applies what it is told.

Same split as stream.py / inputs.py -- this module owns the port and a thread,
and callers push joystick samples in. main.py already owns the one InputReader,
so it feeds this rather than letting it poll:

    from uno_serial import MotorLink
    self.motors = MotorLink()
    self.motors.start()
    ...
    joy = self.inputs.latest()["joy"]
    self.motors.set_joystick(joy["x"], joy["y"])

Bench test it standalone (STOP main.py first -- a second InputReader cannot
claim the switch pins while the viewer holds them):

    python3 uno_serial.py --status        # find the port, report the link
    python3 uno_serial.py --test          # ramp both motors, wheels OFF ground
    python3 uno_serial.py --drive         # live joystick -> motors

THINGS THAT COST TIME HERE:

  * Opening the port RESETS the Uno. USB-serial DTR toggling reboots the board,
    so the first ~1.5 s after open() goes to the bootloader and every command
    sent in that window is silently lost. OPEN_SETTLE_S covers it.
  * pyserial is NOT in the Pi's base image: `sudo apt install python3-serial`.
    Do not pip-install it into the system Python on Debian 13; it is externally
    managed and will refuse.
  * The Uno's failsafe trips after 300 ms of silence, so this must keep sending
    even when nothing changes. SEND_HZ is a floor, not a rate limit on input.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import threading
import time

# --- tuning ------------------------------------------------------------------
# 50 Hz: six frames inside the sketch's 300 ms failsafe, so five can be lost in
# a row without the motors cutting out.
SEND_HZ = float(os.environ.get("UNO_SEND_HZ", "50"))
# 115200 on the operator's order 2026-08-27, back down from 250000 (which had
# been raised from 115200 the day before, for throughput).
#
# THIS NUMBER IS ONE HALF OF A PAIR. The other half is SERIAL_BAUD in
# uno_usb_link.ino and the two must be identical - a mismatch is not a slow
# link, it is a dead one, and it looks exactly like a board flashed with the
# wrong sketch (sent N / acked 0). Change one, reflash the other.
#
# NOTE THE ACCURACY COST OF THIS RATE. On a 16 MHz AVR, UBRR=7 with U2X gives
# 250000.0 exactly; 115200 needs UBRR=16 and actually runs at 117647.1, +2.12%.
# That is inside the ~4% a UART tolerates, so it works - with less margin than
# the rate it replaced. Framing garbage here is the symptom to expect if that
# margin ever runs out. See the note in the sketch.
BAUD = int(os.environ.get("UNO_BAUD", "250000"))

# Opening the port resets the board; nothing sent before this elapses arrives.
OPEN_SETTLE_S = float(os.environ.get("UNO_OPEN_SETTLE_S", "1.8"))

# Residual deadzone applied HERE, on top of whatever the source already did.
#
# ZERO BY DEFAULT, and that is a fix rather than a disabled safety net. Both
# real callers of mix() are fed from inputs.py, whose _norm_axis() already
# applies INPUTS_AXIS_DEADBAND (0.08) AND rescales the travel past it so the
# output leaves centre continuously from 0.0. Applying a second 0.08 band to
# that rescaled value cost twice over: the dead patch became
# 0.08 + 0.08 * 0.92 = ~15.4% of stick travel instead of 8%, and because the
# second band clips WITHOUT rescaling, the wheels then broke away at
# 0.08 * 255 = 20 PWM instead of easing up from zero. A wide dead patch
# followed by a step is precisely what "the acceleration off centre is not
# smooth" feels like in the hand.
#
# The deadband belongs to inputs.py because that is where the centre is learned
# and where the rescale lives; one owner, one number. Set UNO_DEADZONE back to
# 0.08 if mix() is ever fed raw, un-centred axes from somewhere else.
DEADZONE = float(os.environ.get("UNO_DEADZONE", "0.0"))

# Ceiling on demand sent to the driver. Drop it to tame the robot indoors --
# it scales both channels, so steering stays proportional.
MAX_PWM = int(os.environ.get("UNO_MAX_PWM", "255"))

# A joystick sample older than this is treated as no sample at all, so a stalled
# reader coasts the robot to a stop instead of pinning the last demand forever.
SAMPLE_STALE_S = 0.25

# How long to wait before looking for the board again. The Uno is routinely
# plugged in after the viewer is already up (and re-enumerates on every replug),
# so the link has to keep looking -- one that only ever connects at startup
# would need a viewer restart to notice the cable, which on a tethered robot is
# worse than useless.
RETRY_S = 3.0


def find_port():
    """Locate the Uno's serial device, most stable identifier first.

    /dev/serial/by-id survives replug and enumeration order; ttyACM0 does not,
    and picking it blindly can address a different board entirely once a second
    device is attached.
    """
    for pattern in ("/dev/serial/by-id/*Arduino*",
                    "/dev/serial/by-id/*arduino*",
                    "/dev/serial/by-id/*",
                    "/dev/ttyACM*",
                    "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def mix(x, y):
    """Arcade mix: stick to (left, right) in -MAX_PWM..MAX_PWM.

    y drives both wheels together (forward/back), x drives them in opposition
    (turn). Full deflection on both axes would otherwise demand 2.0 from one
    wheel, so the pair is scaled down together -- clipping each wheel
    independently instead would bend the turn as the robot speeds up.
    """
    if x is None or y is None:
        return 0, 0
    if abs(x) < DEADZONE:
        x = 0.0
    if abs(y) < DEADZONE:
        y = 0.0

    left = y + x
    right = y - x

    peak = max(1.0, abs(left), abs(right))
    left /= peak
    right /= peak

    return int(round(left * MAX_PWM)), int(round(right * MAX_PWM))


# Ceiling on the linear actuator's PWM, separate from MAX_PWM: the rod and the
# wheels are different mechanisms and have different tolerance for being run
# flat out, so slowing one must not slow the other.
ACT_MAX_PWM = int(os.environ.get("UNO_ACT_MAX_PWM", "255"))

# THE ROD'S LEADS ARE LANDED THE OTHER WAY ROUND ON THIS RIG, so EXTEND has to
# go out NEGATIVE. Measured on the rod 2026-08-20: the lever's EXTEND throw
# retracted it while the panel correctly read ACT=EXTEND.
#
# CORRECTED HERE RATHER THAN IN THE PIN MAP, and the difference is not cosmetic.
# Swapping ACT_EXTEND_PIN/ACT_RETRACT_PIN also moves the rod the right way - it
# was tried first - but it does so by mislabelling the lever, so the panel then
# reads RETRACT while the rod extends. Only one of the two fixes leaves the
# operator's display telling the truth, and on a control this is the whole
# point: the strip has to say what the rod is about to do.
#
# THE HONEST FIX IS A SOLDERING IRON - swap the two leads at the actuator, or
# invert ACT_DIR in uno_eth_link.ino, and set this to 0. This constant is the
# software standing in for a wiring fault, which is worth knowing if the rod is
# ever rewired: whoever does that has to clear this at the same time, or they
# will re-invert it. Same shape as INVERT_X / INVERT_Y in inputs.py, which
# stands in for the stick's wiring for the same reason.
ACT_INVERT = os.environ.get("UNO_ACT_INVERT", "1") == "1"


def act_demand(state, pot_pct):
    """3-position actuator switch -> ONE signed demand.

    ONLY THE SIGN REACHES THE HARDWARE. The actuator lost its speed demand when
    its PWM pin went to the light on 2026-08-14 (D5), so uno_eth_link.ino reads
    nothing from this value but its sign -- positive extends, negative retracts,
    zero cuts the enable. The magnitude stays at full scale so the field still
    means "-255..255" on the wire.

    THE POT NO LONGER SETS ROD SPEED. It dims the light instead -- see
    light_demand() below. pot_pct is still accepted so callers need not change,
    and is deliberately ignored: making rod DIRECTION depend on whether the ADC
    answered would mean a failed pot read could reverse the rod.

    inputs.py has already decoded the GPIO16/19 pair, which is active-LOW:

        16=0 19=1 -> EXTEND      16=1 19=1 -> STOP      16=1 19=0 -> RETRACT

    Anything that is not clearly EXTEND or RETRACT returns 0, and that
    deliberately includes FAULT. Both legs closed means the mechanical interlock
    has failed, and the safe answer to a switch you cannot trust is not to guess
    which way it was travelling.

    0 IS A GENUINE STOP AGAIN. The rod has a real enable line on the Uno (D4 as
    of 2026-08-15, A2 before that), so 0 cuts the channel's power and the rod
    holds position -- it no longer means "drive the other way". That is what
    makes returning 0 for FAULT and for a stale sample the safe answer rather
    than merely the least-bad one, and it is what lets the sketch's failsafe stop
    the rod instead of running it into an end stop.
    """
    if state not in ("EXTEND", "RETRACT"):
        return 0
    out = ACT_MAX_PWM if state == "EXTEND" else -ACT_MAX_PWM
    return -out if ACT_INVERT else out


def brush_demand(toggle_closed):
    """Panel TOGGLE (Pi GPIO27) -> brush motor demand, 255 or 0.

    TOGGLE ONLY, AT FULL SCALE. The brush briefly took its speed from the
    light's pot on 2026-08-17 and it lasted one evening: one knob feeding two
    mechanisms meant dimming the lamp also slowed the brush, and a knob parked
    at zero made an armed brush look broken (it cost a live debugging round on
    the rig). The operator's verdict was "Pwm->A1, keep 255" - the brush is an
    on/off tool.

    255 rather than 1 because the wire field is a DUTY now: uno_eth_link.ino
    soft-PWMs A1 at whatever magnitude arrives, so full speed must be SAID
    (255), not implied. A board still running the pre-duty build treats any
    non-zero as ON and behaves identically, so this value is right on every
    sketch this rig has ever flashed.

    inputs.py has already done the active-LOW inversion, so what arrives here is
    a plain "is the switch closed" boolean and True means ON. Doing that
    inversion twice is the single easiest mistake to make on this panel, which
    is why this function takes the DECODED value and not a raw pin level.

    None means the reader has not produced a sample yet (or the GPIO is
    unavailable), and that is treated as OFF -- an unknown switch must never
    start a brush.
    """
    return 255 if toggle_closed else 0


def light_demand(pot_pct):
    """Panel potentiometer -> light brightness, 0..255.

    The pot (ADS1115 A2, read in inputs.py) could only take this job because the
    actuator gave up its PWM pin in the same change. Before that, one knob could
    not serve both without the light and the rod fighting over it.

    None means the ADC has produced no sample yet, or the bit-banged read failed.
    That returns 0 -- dark. An unknown pot must not read as full brightness on a
    rig that already browns out under load.

    No deadband, unlike the motors: a lamp at the bottom of the pot's travel is
    simply dim, and folding small values to zero would put a dead patch at the
    start of the knob that looks like a fault.
    """
    if pot_pct is None:
        return 0
    return int(round(max(0.0, min(100.0, pot_pct)) * MAX_PWM / 100.0))


class MotorLink(threading.Thread):
    """Owns the serial port and the send loop. Never raises at callers."""

    def __init__(self, port=None, baud=BAUD):
        super().__init__(daemon=True)
        self._port_hint = port
        self._baud = baud
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ser = None
        self._seq = 0
        self._joy = (None, None)
        self._joy_at = 0.0
        self._state = {
            "ok": False, "error": "starting", "port": None,
            "left": 0, "right": 0, "acks": 0, "sent": 0, "last_ack": None,
        }

    # -- public ---------------------------------------------------------------

    def set_joystick(self, x, y):
        """Push the newest stick sample. Cheap enough for the UI timer."""
        with self._lock:
            self._joy = (x, y)
            self._joy_at = time.monotonic()

    def latest(self):
        with self._lock:
            return dict(self._state)

    def stop(self):
        self._stop.set()

    # -- internals ------------------------------------------------------------

    def _fail(self, msg):
        with self._lock:
            self._state["ok"] = False
            self._state["error"] = msg
        return False

    def _open(self):
        try:
            import serial                                    # noqa: F401
        except ImportError:
            return self._fail("pyserial missing (sudo apt install python3-serial)")

        port = self._port_hint or find_port()
        if port is None:
            return self._fail("no serial port found (is the Uno plugged in?)")

        try:
            import serial
            self._ser = serial.Serial(port, self._baud, timeout=0, write_timeout=0.2)
        except Exception as exc:
            return self._fail(f"open {port} failed: {exc}")

        # The board is rebooting into its bootloader right now. Anything written
        # before this settles is dropped on the floor.
        time.sleep(OPEN_SETTLE_S)
        self._ser.reset_input_buffer()

        with self._lock:
            self._state["ok"] = True
            self._state["error"] = None
            self._state["port"] = port
        return True

    def _close(self, send_stop=True):
        if self._ser is not None:
            if send_stop:
                try:
                    self._send("STOP")        # last word is always neutral
                    time.sleep(0.05)
                except Exception:
                    pass
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        with self._lock:
            self._state["ok"] = False
            self._state["port"] = None
            self._state["left"] = 0
            self._state["right"] = 0

    def _send(self, payload):
        self._seq = (self._seq + 1) & 0xFFFF
        self._ser.write(f"CMD {self._seq} {payload}\n".encode("ascii"))
        with self._lock:
            self._state["sent"] += 1

    def _drain_acks(self):
        try:
            waiting = self._ser.in_waiting
        except Exception:
            return
        if not waiting:
            return
        try:
            chunk = self._ser.read(waiting).decode("ascii", "replace")
        except Exception:
            return
        for line in chunk.splitlines():
            line = line.strip()
            if line.startswith("ACK "):
                with self._lock:
                    self._state["acks"] += 1
                    self._state["last_ack"] = line[4:].strip()

    def run(self):
        period = 1.0 / max(1.0, SEND_HZ)
        try:
            while not self._stop.is_set():
                if self._ser is None:
                    if not self._open():
                        self._stop.wait(RETRY_S)
                        continue

                started = time.monotonic()
                try:
                    with self._lock:
                        x, y = self._joy
                        fresh = (time.monotonic() - self._joy_at) < SAMPLE_STALE_S
                    left, right = mix(x, y) if fresh else (0, 0)

                    self._send(f"M {left} {right}")
                    self._drain_acks()
                    with self._lock:
                        self._state["left"] = left
                        self._state["right"] = right
                except Exception as exc:
                    # Unplugged mid-drive, most likely. Drop the handle without
                    # trying to send a STOP down a dead port and let the loop
                    # reopen it; the Uno's own 300 ms failsafe stops the motors.
                    self._fail(f"write failed: {exc}")
                    self._close(send_stop=False)
                    self._stop.wait(RETRY_S)
                    continue
                self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            self._close()


def _status():
    port = find_port()
    print("serial port :", port or "NONE FOUND")
    try:
        import serial
        print("pyserial    :", serial.__version__)
    except ImportError:
        print("pyserial    : MISSING (sudo apt install python3-serial)")
        return 1
    if port is None:
        print("\nNo Uno visible. Check the USB cable is a data cable, not "
              "charge-only, and that `lsusb` lists the board.")
        return 1

    link = MotorLink()
    link.start()
    time.sleep(OPEN_SETTLE_S + 1.0)
    s = link.latest()
    link.stop()
    time.sleep(0.3)
    print("link ok     :", s["ok"], "" if s["ok"] else f"({s['error']})")
    print("sent/acked  :", s["sent"], "/", s["acks"])
    return 0 if s["ok"] and s["acks"] else 1


def _test():
    print("WHEELS OFF THE GROUND. Ramping each channel, Ctrl-C to abort.")
    link = MotorLink()
    link.start()
    time.sleep(OPEN_SETTLE_S + 0.5)
    if not link.latest()["ok"]:
        print("link failed:", link.latest()["error"])
        return 1
    try:
        for label, (x, y) in (("forward", (0.0, 0.6)), ("back", (0.0, -0.6)),
                              ("left", (-0.6, 0.0)), ("right", (0.6, 0.0))):
            print(f"  {label:8} ->", mix(x, y))
            link.set_joystick(x, y)
            time.sleep(1.5)
            link.set_joystick(0.0, 0.0)
            time.sleep(0.7)
    except KeyboardInterrupt:
        pass
    finally:
        link.set_joystick(0.0, 0.0)
        time.sleep(0.2)
        link.stop()
        time.sleep(0.3)
    print("done:", link.latest())
    return 0


def _drive():
    import inputs
    reader = inputs.InputReader()
    reader.start()
    link = MotorLink()
    link.start()
    time.sleep(OPEN_SETTLE_S + 0.5)
    print("driving from the joystick, Ctrl-C to stop")
    try:
        while True:
            joy = reader.latest().get("joy") or {}
            link.set_joystick(joy.get("x"), joy.get("y"))
            s = link.latest()
            print(f"\r  x={joy.get('x')} y={joy.get('y')}  ->  "
                  f"L={s['left']:>4} R={s['right']:>4}  acks={s['acks']}   ",
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        link.set_joystick(0.0, 0.0)
        time.sleep(0.2)
        link.stop()
        reader.stop()
        time.sleep(0.3)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--status", action="store_true", help="find port, prove link")
    ap.add_argument("--test", action="store_true", help="ramp motors, no joystick")
    ap.add_argument("--drive", action="store_true", help="live joystick -> motors")
    args = ap.parse_args()

    if args.test:
        sys.exit(_test())
    if args.drive:
        sys.exit(_drive())
    sys.exit(_status())
