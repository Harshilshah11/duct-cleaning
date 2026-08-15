#!/usr/bin/env python3
"""
Operator controls: the 7 panel switches, the joystick and the potentiometer.

Same split as stream.py / link.py â€” this module owns the hardware and a thread,
main.py owns the single UI timer and pushes snapshots into inputs_panel.py.
Nothing here touches Qt, so it can be exercised headless:

    python3 inputs.py

WIRING FACTS THIS DEPENDS ON (all measured on the rig, 2026-08-13):

  * Every switch is ACTIVE-LOW â€” closing shorts the pin to GND, open floats.
    So the internal PULL-UP is mandatory and **closed reads 0**. Do not copy
    toggle_read.py, which assumes active-high with a pull-down and therefore
    reads the same value pressed and released.

  * SDA/SCL are DELIBERATELY CROSSED into the ADS1115 (SDA->SCL, SCL->SDA).
    The kernel i2c peripheral has fixed pin roles and cannot adapt, so
    /dev/i2c-1 always scans empty and smbus2 CANNOT work â€” which is why
    joystick_link.py has never been able to read this chip. Bit-banging is the
    only path, hence the imports from the parent directory below.

  * A0 = joystick X, A1 = joystick Y, A2 = potentiometer, A3 = NOT CONNECTED
    (A3 floats and drifts across most of the range; it is never sampled here).

Requesting GPIO2/3 as GPIO lines re-muxes them away from the i2c controller, so
stop() hands them back with `pinctrl set N a0 pu`. Skipping that leaves the
hardware bus broken for anything that runs later.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)          # the bit-bang modules live in ~

# --- tuning ------------------------------------------------------------------
# 20 Hz is well past what an operator can see and leaves the bit-bang plenty of
# headroom (it manages ~50 full 3-channel cycles/sec). The UI redraws at
# config.UI_FPS regardless; this only sets how fresh the numbers are.
POLL_HZ = float(os.environ.get("INPUTS_POLL_HZ", "20"))
ENABLED = os.environ.get("INPUTS_ENABLED", "1") == "1"

# How the two session switches behave, because the panel hardware decides this
# and not us:
#   1 (default) LATCHING levers - the switch position IS the state. Thrown ==
#               recording; centre/off == stopped. Fail-safe: knock the panel and
#               the recording stops rather than silently keeping the old state.
#   0           MOMENTARY buttons - each press flips the state and it stays
#               flipped after the button springs back.
# Flip this to 0 if the pills blink for a fifth of a second per press instead of
# staying lit while you hold them (that is what GPIO25 measured doing).
SESSION_LATCHING = os.environ.get("SESSION_LATCHING", "1") == "1"

# Consecutive agreeing samples before a switch change is believed.
#
# NOT optional on this rig. Logged on 2026-08-15 with pinctrl at ~23 Hz, over a
# 6.5 minute window in which nobody touched the panel: GPIO23 blipped low 42
# times, every blip exactly ONE sample wide (~43 ms), while 24 and 25 never
# moved at all. That is noise on the 23 line, not an operator - a real throw is
# hundreds of ms and shows a spread of durations.
#
# Undebounced, each of those blips is a PAUSE and an immediate RESUME in the
# middle of a recording. 2 samples at the 20 Hz poll = 100 ms of agreement,
# which no 43 ms glitch can survive, and which still catches the save button's
# measured ~180 ms press with 2 samples to spare. Raise it only if the glitches
# ever come in pairs; every extra sample is 50 ms of lag on a real throw.
DEBOUNCE_SAMPLES = int(os.environ.get("INPUTS_DEBOUNCE", "2"))

# Â±6.144 V PGA over a 16-bit signed range. 3.3 V rail => ~17600 counts full scale.
COUNTS_PER_VOLT = 32768 / 6.144
FULL_SCALE = 3.3 * COUNTS_PER_VOLT

# label -> BCM pin. Names come from Harshil; 16/19 are the actuator pair and are
# decoded together rather than shown as two anonymous pins.
# Roles confirmed with Harshil 2026-08-14. GPIO12 is gone on purpose: it was
# the other candidate for the "updated toggle" and measured completely idle
# (never moved) across a 30 s window in which GPIO25 moved 56 times, so the
# updated toggle is on 25 and nothing is wired to 12.
#
# 23/24 are the recording controls, confirmed with Harshil 2026-08-15. They are
# two INDEPENDENT switches, not the interlocked pair the earlier comment here
# guessed at - measured both reading `hi` (open) at rest on 2026-08-15, whereas
# an interlocked ON-OFF-ON pair always has one leg down. So they are decoded
# together into a session state but each keeps its own meaning:
#   switch 1, RED   leg, GPIO24 ... START / STOP
#   switch 2, GREEN leg, GPIO23 ... PAUSE / RESUME
SWITCHES = [
    ("BRUSH", 13),
    ("SAVE", 25),
    ("START / STOP", 24),
    ("PAUSE / RESUME", 23),
]
ACT_EXTEND_PIN = 16
ACT_RETRACT_PIN = 19

# Named separately from SWITCHES so the panel and the recorder can reference the
# roles without matching on a display string.
BRUSH_PIN = 13          # brush motor on / off
REC_PIN = 24            # switch 1 - START / STOP
PAUSE_PIN = 23          # switch 2 - PAUSE / RESUME
SESSION_PINS = (REC_PIN, PAUSE_PIN)

# The three session states, in the order the operator moves through them.
RECORDING, PAUSED, STOPPED = "RECORDING", "PAUSED", "STOPPED"

ALL_PINS = [p for _, p in SWITCHES] + [ACT_EXTEND_PIN, ACT_RETRACT_PIN]


# The save button. GPIO25 was measured pulsing low for ~0.18 s per press, i.e.
# it is momentary, not latching -- so its LEVEL is useless to a 30 fps UI that
# can sample between two frames of the pulse. The reader counts edges instead
# and publishes a running total; a consumer remembers the last count it acted on
# and fires once per increment. That makes a missed press impossible as long as
# the 20 Hz poll catches the pulse, which it does with ~3 samples to spare.
SAVE_PIN = 25


def _blank():
    """Snapshot shape, used before the first read and whenever hardware is absent."""
    return {
        "ok": False,
        "error": "startingâ€¦",
        "switches": {name: None for name, _ in SWITCHES},
        "actuator": None,          # "EXTEND" / "STOP" / "RETRACT" / "FAULT"
        # Raw pin levels behind that decode, so the panel can show the operator
        # WHY it is calling a stage rather than just asserting it.
        "act_pins": {ACT_EXTEND_PIN: None, ACT_RETRACT_PIN: None},
        # Recording session decoded from the two panel switches, plus the raw
        # levels behind it for the same reason act_pins exists.
        "session": None,           # "RECORDING" / "PAUSED" / "STOPPED"
        "session_pins": {REC_PIN: None, PAUSE_PIN: None},
        # Total open->closed edges seen on the save button since start-up.
        "save_presses": 0,
        "joy": {"x": None, "y": None, "x_raw": None, "y_raw": None},
        "pot": {"pct": None, "raw": None, "volts": None},
        "updated": 0.0,
    }


class SessionDecode:
    """The two panel switches -> STOPPED / RECORDING / PAUSED.

    Kept as a class because the momentary mode has to remember what it was told
    last; the latching mode is stateless and ignores all of it. Both paths run
    off the same `closed` booleans the rest of inputs.py deals in, so neither has
    to know that closed means the pin reads 0.
    """

    def __init__(self, latching=None):
        self.latching = SESSION_LATCHING if latching is None else latching
        self._running = False
        self._paused = False
        self._was_rec = None        # None until the first sample, so the very
        self._was_pause = None      # first reading cannot score a phantom press

    def update(self, rec_closed, pause_closed):
        if rec_closed is None or pause_closed is None:
            return None

        if self.latching:
            self._running, self._paused = rec_closed, pause_closed
        else:
            if rec_closed and self._was_rec is False:
                self._running = not self._running
                # Starting a run always starts it rolling. Inheriting a stale
                # pause from the previous run would mean pressing START and
                # recording nothing, with the UI insisting it was paused.
                self._paused = False
            if pause_closed and self._was_pause is False and self._running:
                self._paused = not self._paused

        self._was_rec, self._was_pause = rec_closed, pause_closed

        if not self._running:
            # PAUSED only exists inside a run. Holding the pause lever with the
            # recording lever off is STOPPED, not some fourth state.
            self._paused = False
            return STOPPED
        return PAUSED if self._paused else RECORDING


class InputReader(threading.Thread):
    """Polls switches + ADS channels in the background. Never raises at callers."""

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._state = _blank()
        self._stop = threading.Event()
        self._req = None            # switch lines
        self._ads_req = None        # GPIO2/3 for the bit-bang
        self._bus = None
        # Auto-centred at startup, exactly like joystick_link.py: the stick's
        # rest point is VDD/2 on THIS rail, and joystick_link.py's hardcoded
        # CENTRE_TYPICAL=12100 is a stale 5 V constant that does not apply.
        self._centre = {}
        # Session + save-button state live on the reader, not in the snapshot:
        # _read_once() rebuilds the snapshot from _blank() every pass, so
        # anything that has to survive between reads has to be held here.
        self._session = SessionDecode()
        self._save_presses = 0
        self._save_was = None
        self._stable = {}           # pin -> last believed `closed`
        self._candidate = {}        # pin -> (value, consecutive samples seen)

    # -- public ---------------------------------------------------------------

    def latest(self):
        with self._lock:
            return dict(self._state)

    def stop(self):
        self._stop.set()

    # -- internals ------------------------------------------------------------

    def _fail(self, msg):
        with self._lock:
            self._state = _blank()
            self._state["error"] = msg
        return False

    def _open(self):
        if not ENABLED:
            return self._fail("disabled (INPUTS_ENABLED=0)")
        if _PARENT not in sys.path:
            sys.path.insert(0, _PARENT)
        try:
            global gpiod, Bias, Direction, Value, IN, Bus, SDA, SCL, sample
            import gpiod
            from gpiod.line import Bias, Direction, Value
            from i2c_bitbang_probe import IN, Bus
            from i2c_bitbang_read import SDA, SCL, sample
        except Exception as exc:            # missing lib / missing helper module
            return self._fail(f"import failed: {exc}")

        try:
            pu = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
            self._req = gpiod.request_lines(
                "/dev/gpiochip0", consumer="ground-station-switches",
                config={p: pu for p in ALL_PINS})
        except Exception as exc:            # EBUSY if a probe script is running
            return self._fail(f"switch pins busy: {exc}")

        try:
            self._ads_req = gpiod.request_lines(
                "/dev/gpiochip0", consumer="ground-station-ads",
                config={SDA: IN, SCL: IN})
            self._bus = Bus(self._ads_req, SDA, SCL)
            self._bus.reset_state()
        except Exception as exc:
            # Switches still work without the ADC, so this is not fatal â€” the
            # panel shows analog as unavailable and the pills keep updating.
            self._ads_req = self._bus = None
            with self._lock:
                self._state["error"] = f"ADC unavailable: {exc}"
        return True

    def _close(self):
        for req in (self._req, self._ads_req):
            try:
                if req is not None:
                    req.release()
            except Exception:
                pass
        self._req = self._ads_req = self._bus = None
        # Hand GPIO2/3 back to the i2c controller, or the hardware bus stays
        # broken for every later run.
        for pin in (2, 3):
            try:
                subprocess.run(["/usr/bin/pinctrl", "set", str(pin), "a0", "pu"],
                               check=False, capture_output=True, timeout=2)
            except Exception:
                pass

    def _norm_axis(self, ch, raw):
        """Raw counts -> -1.0..+1.0 about the auto-captured centre."""
        if raw is None:
            return None
        self._centre.setdefault(ch, raw)
        span = FULL_SCALE / 2.0
        return max(-1.0, min(1.0, (raw - self._centre[ch]) / span))

    def _debounce(self, raw):
        """Accept a change only after DEBOUNCE_SAMPLES agreeing reads.

        The first sample seeds every pin, so start-up is immediate rather than
        showing unknown switches for the first 100 ms.
        """
        if not self._stable:
            self._stable = dict(raw)
            return dict(raw)
        for pin, value in raw.items():
            if value == self._stable[pin]:
                # Back to the believed level before it was ever accepted - the
                # blip is over, so forget it rather than counting it toward the
                # next change in the same direction.
                self._candidate.pop(pin, None)
                continue
            prev, runs = self._candidate.get(pin, (value, 0))
            runs = runs + 1 if prev == value else 1
            if runs >= DEBOUNCE_SAMPLES:
                self._stable[pin] = value
                self._candidate.pop(pin, None)
            else:
                self._candidate[pin] = (value, runs)
        return dict(self._stable)

    def _read_once(self):
        state = _blank()
        state["error"] = None

        raw = {}
        for pin in ALL_PINS:
            # PULL_UP means INACTIVE (0) == shorted to GND == closed. This
            # inversion is the single easiest thing to get backwards here.
            raw[pin] = self._req.get_value(pin) == Value.INACTIVE
        closed = self._debounce(raw)
        for name, pin in SWITCHES:
            state["switches"][name] = closed[pin]

        ext, ret = closed[ACT_EXTEND_PIN], closed[ACT_RETRACT_PIN]
        # The actuator switch is mechanically interlocked (ON-OFF-ON), so both
        # closed should be impossible â€” surfaced as FAULT rather than assumed
        # away, since a broken interlock is exactly what you want to see.
        state["actuator"] = ("FAULT" if ext and ret else
                             "EXTEND" if ext else
                             "RETRACT" if ret else "STOP")
        # Back to raw levels for display: closed == shorted to GND == reads 0.
        # Measured on the rig 2026-08-14, the three stages are
        # 16=0/19=1 EXTEND, 16=1/19=1 STOP, 16=1/19=0 RETRACT.
        state["act_pins"] = {ACT_EXTEND_PIN: 0 if ext else 1,
                             ACT_RETRACT_PIN: 0 if ret else 1}

        state["session"] = self._session.update(closed[REC_PIN], closed[PAUSE_PIN])
        state["session_pins"] = {REC_PIN: 0 if closed[REC_PIN] else 1,
                                 PAUSE_PIN: 0 if closed[PAUSE_PIN] else 1}

        # Edge count, not level - see SAVE_PIN. `is False` rather than `not`, so
        # the first sample after start-up cannot be mistaken for a release.
        if closed[SAVE_PIN] and self._save_was is False:
            self._save_presses += 1
        self._save_was = closed[SAVE_PIN]
        state["save_presses"] = self._save_presses

        if self._bus is not None:
            x_raw, y_raw, pot_raw = (sample(self._bus, 0), sample(self._bus, 1),
                                     sample(self._bus, 2))
            state["joy"] = {
                "x": self._norm_axis(0, x_raw), "y": self._norm_axis(1, y_raw),
                "x_raw": x_raw, "y_raw": y_raw,
            }
            if pot_raw is not None:
                state["pot"] = {
                    "pct": max(0.0, min(100.0, 100.0 * pot_raw / FULL_SCALE)),
                    "raw": pot_raw, "volts": pot_raw / COUNTS_PER_VOLT,
                }
        else:
            state["error"] = "ADC unavailable"

        state["ok"] = True
        state["updated"] = time.time()
        with self._lock:
            self._state = state

    def run(self):
        if not self._open():
            return
        period = 1.0 / max(1.0, POLL_HZ)
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                try:
                    self._read_once()
                except Exception as exc:
                    with self._lock:
                        self._state["ok"] = False
                        self._state["error"] = f"read failed: {exc}"
                self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            self._close()


if __name__ == "__main__":
    reader = InputReader()
    reader.start()
    try:
        while True:
            time.sleep(0.5)
            s = reader.latest()
            if not s["ok"]:
                print("waiting:", s["error"])
                continue
            pills = " ".join(
                f"{n}={'ON ' if v else 'off'}" for n, v in s["switches"].items())
            joy, pot = s["joy"], s["pot"]
            head = (f"{pills} | ACT={s['actuator']:<7} "
                    f"| {s['session']:<9} saves={s['save_presses']}")
            print(f"{head} | joy x={joy['x']:+.2f} y={joy['y']:+.2f} "
                  f"| pot {pot['pct']:.0f}%"
                  if joy["x"] is not None and pot["pct"] is not None
                  else f"{head} | analog: {s['error']}")
    except KeyboardInterrupt:
        reader.stop()
        time.sleep(0.3)

