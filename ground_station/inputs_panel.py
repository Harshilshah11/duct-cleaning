#!/usr/bin/env python3
"""
The operator-controls strip: switch pills, actuator state, joystick, pot.

Like topbar.py, nothing in here polls anything â€” main.py owns the single UI
timer and pushes a snapshot in through set_state(). That keeps this cheap
enough to redraw at the full UI frame rate.

Preview it standalone against the real hardware:

    python3 inputs_panel.py
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QRectF, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# Pin numbers only, so the caption and the raw readout cannot drift out of sync
# with the reader. Importing inputs touches no hardware at module scope.
from inputs import ACT_EXTEND_PIN, ACT_RETRACT_PIN, SWITCHES

# Pins carrying a plain on/off switch. These get a trail of their own rather
# than just a pill, because they are what actually gets watched during bring-up.
# GPIO12 is deliberately NOT here: measured idle on 2026-08-14 (never moved)
# while GPIO25 -- its supposed alternative -- moved 56 times, so of that pair
# only 25 is wired.
TOGGLE_PINS = (13, 25)
PIN_TO_NAME = {pin: name for name, pin in SWITCHES}

MONO = "DejaVu Sans Mono"

# 156 not 132: the joystick box plus its caption and readout needs ~140px of
# content, and at 132 the box was visibly clipped against the bottom of a
# 1080p screen. This costs the video panels 14% of the height, which is the
# trade the strip is worth.
STRIP_HEIGHT = 156

# Dark-field palette, picked to sit under main.py's #0b0f14 window rather than
# the light top bar â€” the strip lives beside the video, not above it.
BG = "#05080b"
LINE = "#1e2a38"
TEXT = "#d7e0ea"
MUTED = "#6d7c92"
LIVE = "#35d07f"      # closed / active
IDLE = "#2a3646"      # open / inactive
WARN = "#e0a83a"
BAD = "#e0564a"
ACCENT = "#4a90d9"

# Per-switch lit colour, keyed on the names in inputs.SWITCHES. Anything not
# listed falls back to LIVE, so adding a switch needs no change here.
PILL_COLOUR = {"GREEN": LIVE, "RED": BAD}


class Pill(QLabel):
    """One switch. Lit in its own colour when closed, flat grey when open,
    outlined when unknown.

    The colour is per-pill rather than always green because GPIO23/24 are the
    two legs of a red/green switch: a RED leg that lights up green when thrown
    is actively misleading at a glance, which is the only moment this strip is
    ever read.
    """

    def __init__(self, name, colour=LIVE):
        super().__init__(name)
        self._name = name
        self._colour = colour
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont(MONO, 9, QFont.Bold))
        self.setFixedHeight(26)
        self.setMinimumWidth(94)
        self.set_value(None)

    def set_value(self, closed):
        if closed is None:
            fg, bg, border = MUTED, "#131a24", LINE
        elif closed:
            fg, bg, border = BG, self._colour, self._colour
        else:
            fg, bg, border = MUTED, "#0d141d", IDLE
        self.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {border};"
            f"border-radius:13px; padding:0 10px;")


class JoystickView(QWidget):
    """Crosshair box with a dot at the stick position.

    A 2D dot is the whole point: two separate bars make you reconstruct the
    diagonal in your head, which is the one thing an operator reads at a glance.
    """

    def __init__(self):
        super().__init__()
        self.setFixedSize(92, 92)
        self._x = self._y = None

    def set_pos(self, x, y):
        if (x, y) != (self._x, self._y):
            self._x, self._y = x, y
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        box = QRectF(1, 1, self.width() - 2, self.height() - 2)

        p.setPen(QPen(QColor(LINE), 1))
        p.setBrush(QColor("#0a1119"))
        p.drawRoundedRect(box, 4, 4)

        cx, cy = box.center().x(), box.center().y()
        p.setPen(QPen(QColor(IDLE), 1, Qt.DashLine))
        p.drawLine(int(box.left()), int(cy), int(box.right()), int(cy))
        p.drawLine(int(cx), int(box.top()), int(cx), int(box.bottom()))

        if self._x is None or self._y is None:
            p.setPen(QColor(MUTED))
            p.setFont(QFont(MONO, 8))
            p.drawText(box, Qt.AlignCenter, "no ADC")
            return

        half = (box.width() - 18) / 2.0
        # Screen Y grows downward, so the axis is negated: pushing the stick
        # "up" has to move the dot up, not down.
        dx, dy = cx + self._x * half, cy - self._y * half
        p.setPen(QPen(QColor(ACCENT), 1))
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QRectF(dx - 5, dy - 5, 10, 10))


class PotBar(QWidget):
    """Horizontal fill bar, 0-100%."""

    def __init__(self):
        super().__init__()
        self.setFixedHeight(18)
        self.setMinimumWidth(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._pct = None

    def set_pct(self, pct):
        if pct != self._pct:
            self._pct = pct
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        box = QRectF(0.5, 0.5, self.width() - 1, self.height() - 1)
        p.setPen(QPen(QColor(LINE), 1))
        p.setBrush(QColor("#0a1119"))
        p.drawRoundedRect(box, 3, 3)
        if self._pct is None:
            return
        inner = box.adjusted(2, 2, -2, -2)
        inner.setWidth(max(0.0, inner.width() * self._pct / 100.0))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(ACCENT))
        p.drawRoundedRect(inner, 2, 2)


class ActuatorStages(QWidget):
    """The 3-position actuator lever, drawn as a lever.

    The old readout was one word that swapped between EXTEND/STOP/RETRACT. That
    tells you the state but not the travel: you cannot see at a glance that
    there are three stages, nor which way the next throw goes. Showing all three
    rows with a knob on a track reads the same way the physical ON-OFF-ON switch
    does, so the operator maps screen to hand without thinking.

    Rows are ordered to match the lever: up extends, down retracts.
    """

    # (state string from inputs.py, label shown, colour when active)
    STAGES = (
        ("EXTEND", "EXTEND", LIVE),
        ("STOP", "NEUTRAL", TEXT),
        ("RETRACT", "RETRACT", ACCENT),
    )

    ROW_H = 22
    TRACK_W = 16
    PAD = 4

    def __init__(self):
        super().__init__()
        self._stage = None
        self.setFixedSize(150, self.ROW_H * len(self.STAGES) + self.PAD * 2)

    def set_stage(self, stage):
        if stage != self._stage:
            self._stage = stage
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # FAULT means both pins low, i.e. the interlock is broken and no stage
        # is truthfully "current" - so nothing lights and the track goes red.
        fault = self._stage == "FAULT"

        track = QRectF(2.0, float(self.PAD), float(self.TRACK_W),
                       float(self.ROW_H * len(self.STAGES)))
        p.setPen(QPen(QColor(BAD if fault else LINE), 1))
        p.setBrush(QColor("#0a1119"))
        p.drawRoundedRect(track, self.TRACK_W / 2.0, self.TRACK_W / 2.0)

        cx = track.center().x()
        text_x = track.right() + 9
        text_w = self.width() - text_x - 2

        for i, (key, label, colour) in enumerate(self.STAGES):
            cy = self.PAD + self.ROW_H * i + self.ROW_H / 2.0
            active = (not fault) and key == self._stage

            # detent dot: big and coloured at the current stage, small and flat
            # everywhere else, so position is legible from across the room
            radius = 5.0 if active else 2.5
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour if active else IDLE))
            p.drawEllipse(QRectF(cx - radius, cy - radius, radius * 2, radius * 2))

            p.setPen(QColor(colour if active else MUTED))
            p.setFont(QFont(MONO, 9, QFont.Bold if active else QFont.Normal))
            p.drawText(QRectF(text_x, cy - self.ROW_H / 2.0, text_w, self.ROW_H),
                       Qt.AlignVCenter | Qt.AlignLeft, label)


# Trail vocabularies: state -> (letter, colour). The actuator reuses the
# lever's own colours so the two readouts are obviously the same language.
ACT_TRAIL_LETTERS = {"EXTEND": ("E", LIVE), "STOP": ("N", TEXT),
                     "RETRACT": ("R", ACCENT), "FAULT": ("!", BAD)}
# Keyed on the `closed` bool straight out of inputs.py, not on a pin level:
# closed == shorted to GND == the operator has it switched ON. Marked I/O like
# a power rocker rather than 1/0, which would read as the pin level and so mean
# exactly the opposite of the truth on this active-low wiring.
TOGGLE_TRAIL_LETTERS = {True: ("I", LIVE), False: ("O", MUTED)}


class StateTrail(QWidget):
    """The last few state changes, oldest left, newest right and filled solid.

    A live readout only ever shows you *now*. The question during bring-up is
    almost always about the recent past -- did that throw register, did it pass
    through neutral, how long has it been sitting there -- which previously
    meant watching a terminal over SSH. Nine chips is about four full
    EXTEND-NEUTRAL-RETRACT cycles, which is as far back as anyone reads.
    """

    CHIP_W = 13
    CHIP_H = 16
    GAP = 3

    def __init__(self, letters, slots=9):
        super().__init__()
        self._letters = letters
        self._slots = slots
        self._trail = []          # oldest -> newest
        self._last = None
        self._since = None
        self.setFixedSize(slots * (self.CHIP_W + self.GAP) + 2, self.CHIP_H + 2)

    def push(self, state):
        """Record a state. Only actual changes land, so repeats are free."""
        # `is None` rather than a falsy test: False is a real toggle state.
        if state is None or state == self._last:
            return
        self._last = state
        self._since = time.monotonic()
        self._trail.append(state)
        del self._trail[:-self._slots]
        self.update()

    def held(self):
        """Seconds in the current state, or None before the first reading."""
        return None if self._since is None else time.monotonic() - self._since

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setFont(QFont(MONO, 8, QFont.Bold))

        # Right-align: the newest chip always sits in the last slot, so the eye
        # can park in one place instead of tracking a growing list.
        lead = self._slots - len(self._trail)
        for slot in range(self._slots):
            box = QRectF(slot * (self.CHIP_W + self.GAP) + 1.0, 1.0,
                         float(self.CHIP_W), float(self.CHIP_H))
            idx = slot - lead
            if idx < 0:
                p.setPen(QPen(QColor(IDLE), 1))
                p.setBrush(Qt.NoBrush)
                p.drawRoundedRect(box, 2, 2)
                continue

            state = self._trail[idx]
            letter, colour = self._letters.get(state, ("?", MUTED))
            newest = idx == len(self._trail) - 1
            p.setPen(QPen(QColor(colour), 1))
            p.setBrush(QColor(colour) if newest else QColor("#0a1119"))
            p.drawRoundedRect(box, 2, 2)
            p.setPen(QColor(BG) if newest else QColor(colour))
            p.drawText(box, Qt.AlignCenter, letter)


def _held_text(seconds):
    """Seconds only up to a minute, then m:ss -- a bare "184.2s" is a number
    you have to stop and divide, which defeats a glance readout."""
    if seconds is None:
        return ""
    if seconds < 60:
        return f"{seconds:4.1f}s"
    return f"{int(seconds) // 60}m{int(seconds) % 60:02d}"


class ToggleRow(QWidget):
    """Caption + trail + readout for one on/off pin.

    The press counter is not decoration. GPIO25 was measured pulsing low for
    ~0.18 s at a time -- three or four UI frames -- which the 20 Hz reader
    catches easily but an operator watching a pill will miss entirely. A count
    that ticks up is still there after the fact; a pill that blinked is not.
    It also settles the open question about that pin: if the count climbs while
    the state sits at OFF, it is a momentary button, not a latching toggle.
    """

    def __init__(self, pin, name):
        super().__init__()
        self.pin = pin
        self.name = name
        self.trail = StateTrail(TOGGLE_TRAIL_LETTERS)
        self.presses = 0
        self._was = None

        self.text = QLabel("")
        self.text.setFont(QFont(MONO, 9, QFont.Bold))
        self.text.setStyleSheet(f"color:{MUTED};")

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        row.addWidget(_caption(f"{name} ({pin})"))
        row.addWidget(self.trail)
        row.addWidget(self.text)
        row.addStretch(1)

    def set_closed(self, closed):
        # Count only genuine open->closed edges: `self._was is False` rather
        # than `not self._was`, or the very first reading scores a phantom press.
        if closed and self._was is False:
            self.presses += 1
        self._was = closed
        self.trail.push(closed)

        if closed is None:
            self.text.setText(f"{self.pin}:-")
            self.text.setStyleSheet(f"color:{MUTED};")
            return
        # Raw level and the word together: on active-low wiring "25:0" and "ON"
        # look contradictory until you have seen them side by side once.
        self.text.setText(
            f"{self.pin}:{0 if closed else 1}  {'ON ' if closed else 'OFF'}  "
            f"{self.presses:>3}x  {_held_text(self.trail.held())}")
        self.text.setStyleSheet(f"color:{LIVE if closed else MUTED};")


def _caption(text):
    label = QLabel(text)
    label.setFont(QFont(MONO, 8, QFont.Bold))
    label.setStyleSheet(f"color:{MUTED}; letter-spacing:1px;")
    return label


class InputsPanel(QFrame):
    """The whole strip. Push state in with set_state()."""

    def __init__(self):
        super().__init__()
        self.setObjectName("inputsPanel")
        self.setFixedHeight(STRIP_HEIGHT)
        self.setStyleSheet(
            f"#inputsPanel {{ background:{BG}; border:1px solid {LINE};"
            f"border-radius:4px; }}")

        # --- switches -------------------------------------------------------
        # Driven off inputs.SWITCHES rather than a second hardcoded list, so a
        # rewiring only has to be recorded in one place.
        self.pills = {}
        pill_row = QHBoxLayout()
        pill_row.setSpacing(6)
        for name, _pin in SWITCHES:
            pill = Pill(name, PILL_COLOUR.get(name, LIVE))
            self.pills[name] = pill
            pill_row.addWidget(pill)
        pill_row.addStretch(1)

        self.actuator = ActuatorStages()

        # Raw levels under the lever. During bring-up the decode is exactly what
        # you are trying to confirm, so showing the two bits it was derived from
        # turns "the UI says NEUTRAL" into something checkable against a meter.
        self.act_pins = QLabel("")
        self.act_pins.setFont(QFont(MONO, 8))
        self.act_pins.setStyleSheet(f"color:{MUTED};")

        self.act_trail = StateTrail(ACT_TRAIL_LETTERS)

        act_col = QVBoxLayout()
        act_col.setSpacing(3)
        act_col.addWidget(_caption(f"ACTUATOR ({ACT_EXTEND_PIN}/{ACT_RETRACT_PIN})"))
        act_col.addWidget(self.actuator)
        act_col.addWidget(self.act_trail)
        act_col.addWidget(self.act_pins)
        act_col.addStretch(1)      # pin to the top, level with the other captions

        # The toggles again, below their own pills: a pill answers "is it on
        # now", these answer "did my last flip take, when, and how many times".
        # They live in the dead space under the pill row rather than in columns
        # of their own, which the 1024px strip has no horizontal room for.
        self.toggles = {}

        sw_col = QVBoxLayout()
        sw_col.setSpacing(3)
        sw_col.addWidget(_caption("SWITCHES"))
        sw_col.addLayout(pill_row)
        sw_col.addSpacing(6)
        for pin in TOGGLE_PINS:
            row = ToggleRow(pin, PIN_TO_NAME.get(pin, f"GPIO{pin}"))
            self.toggles[pin] = row
            sw_col.addWidget(row)
        sw_col.addStretch(1)

        # --- analog ---------------------------------------------------------
        self.joy = JoystickView()
        self.joy_text = QLabel("â€”")
        self.joy_text.setFont(QFont(MONO, 8))
        self.joy_text.setStyleSheet(f"color:{MUTED};")
        self.joy_text.setAlignment(Qt.AlignCenter)

        joy_col = QVBoxLayout()
        joy_col.setSpacing(3)
        joy_col.addWidget(_caption("JOYSTICK"), 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy, 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy_text, 0, Qt.AlignHCenter)
        joy_col.addStretch(1)

        self.pot = PotBar()
        self.pot_text = QLabel("â€”")
        self.pot_text.setFont(QFont(MONO, 9, QFont.Bold))
        self.pot_text.setStyleSheet(f"color:{TEXT};")

        pot_col = QVBoxLayout()
        pot_col.setSpacing(3)
        pot_col.addWidget(_caption("POTENTIOMETER"))
        pot_col.addWidget(self.pot)
        pot_col.addWidget(self.pot_text)
        pot_col.addStretch(1)

        self.status = QLabel("")
        self.status.setFont(QFont(MONO, 8))
        self.status.setStyleSheet(f"color:{WARN};")

        left = QVBoxLayout()
        left.setSpacing(4)
        left.addLayout(sw_col)
        left.addWidget(self.status)

        root = QHBoxLayout(self)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(18)
        root.addLayout(left, 1)
        root.addLayout(act_col)
        root.addLayout(pot_col)
        root.addLayout(joy_col)

    # -- state ---------------------------------------------------------------

    def set_state(self, state):
        for name, pill in self.pills.items():
            pill.set_value(state["switches"].get(name))

        for row in self.toggles.values():
            row.set_closed(state["switches"].get(row.name))

        act = state.get("actuator")
        self.actuator.set_stage(act)
        self.act_trail.push(act)

        pins = state.get("act_pins") or {}
        ext_lvl, ret_lvl = pins.get(ACT_EXTEND_PIN), pins.get(ACT_RETRACT_PIN)
        if ext_lvl is None or ret_lvl is None:
            self.act_pins.setText(f"{ACT_EXTEND_PIN}:-  {ACT_RETRACT_PIN}:-")
            self.act_pins.setStyleSheet(f"color:{MUTED};")
        else:
            age = _held_text(self.act_trail.held())
            suffix = ("  BOTH LOW" if act == "FAULT"
                      else (f"   {age}" if age else ""))
            self.act_pins.setText(
                f"{ACT_EXTEND_PIN}:{ext_lvl}  {ACT_RETRACT_PIN}:{ret_lvl}{suffix}")
            self.act_pins.setStyleSheet(
                f"color:{BAD if act == 'FAULT' else MUTED};")

        joy = state.get("joy") or {}
        self.joy.set_pos(joy.get("x"), joy.get("y"))
        if joy.get("x") is not None:
            self.joy_text.setText(f"x{joy['x']:+.2f} y{joy['y']:+.2f}")
        else:
            self.joy_text.setText("â€”")

        pot = state.get("pot") or {}
        self.pot.set_pct(pot.get("pct"))
        self.pot_text.setText(
            f"{pot['pct']:5.1f}%   {pot['volts']:.2f} V"
            if pot.get("pct") is not None else "â€”")

        # Only surfaced when something is actually wrong; a healthy rig shows a
        # blank line rather than a reassuring message nobody reads.
        self.status.setText(state.get("error") or "")


if __name__ == "__main__":
    import sys

    from PySide6.QtWidgets import QApplication

    import inputs

    app = QApplication(sys.argv)
    window = QWidget()
    window.setStyleSheet("background:#0b0f14;")
    panel = InputsPanel()
    box = QVBoxLayout(window)
    box.addWidget(panel)
    window.resize(1000, STRIP_HEIGHT + 40)

    reader = inputs.InputReader()
    reader.start()
    timer = QTimer()
    timer.timeout.connect(lambda: panel.set_state(reader.latest()))
    timer.start(50)

    window.show()
    code = app.exec()
    reader.stop()
    sys.exit(code)

