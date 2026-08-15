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
from PySide6.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# Pin numbers only, so the caption and the raw readout cannot drift out of sync
# with the reader. Importing inputs touches no hardware at module scope.
from inputs import (
    ACT_EXTEND_PIN, ACT_RETRACT_PIN, BRUSH_PIN, PAUSE_PIN, REC_PIN, SAVE_PIN,
    SWITCHES,
)
from recorder import hms

PIN_TO_NAME = {pin: name for name, pin in SWITCHES}

# GPIO12 is deliberately absent everywhere here: measured idle on 2026-08-14
# (never moved) while GPIO25 -- its supposed alternative -- moved 56 times, so
# of that pair only 25 is wired.

MONO = "DejaVu Sans Mono"

# 174 not 156: the strip is now three titled blocks rather than four bare
# columns, and each block spends 13px on its title plus 12px of padding. The
# floor is still set by the same thing it always was - the joystick box, its
# caption and its readout need ~127px of content height, and anything less
# clips the box against the bottom of the screen. This is the height at the
# compact scale; InputsPanel multiplies it by LARGE_K on a wide screen.
STRIP_HEIGHT = 174

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

REC = "#e0463f"       # the recording red, saturated - it has to shout

# Per-switch lit colour, keyed on the names in inputs.SWITCHES. Anything not
# listed falls back to LIVE, so adding a switch needs no change here.
PILL_COLOUR = {"START / STOP": REC, "PAUSE / RESUME": WARN, "SAVE": ACCENT,
               "BRUSH": LIVE}

# Session state -> (dot+text colour, whether the dot blinks). Blinking is
# reserved for RECORDING: it is the one state where being wrong about it costs
# you the footage, and a blinking dot is the universal shorthand for it.
SESSION_LOOK = {
    "RECORDING": (REC, True),
    "PAUSED": (WARN, False),
    "STOPPED": (MUTED, False),
}


class Pill(QLabel):
    """One switch. Lit in its own colour when closed, flat grey when open,
    outlined when unknown.

    The colour is per-pill rather than always green because GPIO23/24 are the
    two legs of a red/green switch: a RED leg that lights up green when thrown
    is actively misleading at a glance, which is the only moment this strip is
    ever read.
    """

    def __init__(self, name, colour=LIVE, compact=False):
        super().__init__(name)
        self._name = name
        self._colour = colour
        self._compact = compact
        self._closed = None
        self.setAlignment(Qt.AlignCenter)
        self.set_scale(1.0)

    def set_scale(self, k):
        # Compact pills carry a ROLE ("PAUSE / RESUME"), not a short label, and
        # three of them share one block. At the full size the row needs 285px it
        # does not have on a 1024 screen and Qt squeezes them into "USE / RESU".
        self._pad = round((8 if self._compact else 10) * k)
        self._radius = round((11 if self._compact else 13) * k)
        self.setFont(QFont(MONO, round((8 if self._compact else 9) * k), QFont.Bold))
        self.setFixedHeight(round((22 if self._compact else 26) * k))
        # A QLabel's minimumSizeHint is smaller than its sizeHint - it will
        # happily be squeezed and clip its own text, which is how
        # "PAUSE / RESUME" became "USE / RESU". Pin the floor to the text the
        # pill actually holds, from metrics rather than from sizeHint(), which
        # is not reliable before the widget is polished.
        floor = (QFontMetrics(self.font()).horizontalAdvance(self._name)
                 + 2 * self._pad + 4)
        self.setMinimumWidth(floor if self._compact else max(round(94 * k), floor))
        self.set_value(self._closed)

    def set_value(self, closed):
        self._closed = closed
        if closed is None:
            fg, bg, border = MUTED, "#131a24", LINE
        elif closed:
            fg, bg, border = BG, self._colour, self._colour
        else:
            fg, bg, border = MUTED, "#0d141d", IDLE
        self.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {border};"
            f"border-radius:{self._radius}px; padding:0 {self._pad}px;")


class JoystickView(QWidget):
    """Crosshair box with a dot at the stick position.

    A 2D dot is the whole point: two separate bars make you reconstruct the
    diagonal in your head, which is the one thing an operator reads at a glance.
    """

    def __init__(self):
        super().__init__()
        self._x = self._y = None
        self._k = 1.0
        self.set_scale(1.0)

    def set_scale(self, k):
        self._k = k
        self.setFixedSize(round(92 * k), round(92 * k))

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
            p.setFont(QFont(MONO, round(8 * self._k)))
            p.drawText(box, Qt.AlignCenter, "no ADC")
            return

        r = 5.0 * self._k
        half = (box.width() - 18 * self._k) / 2.0
        # Screen Y grows downward, so the axis is negated: pushing the stick
        # "up" has to move the dot up, not down.
        dx, dy = cx + self._x * half, cy - self._y * half
        p.setPen(QPen(QColor(ACCENT), 1))
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QRectF(dx - r, dy - r, r * 2, r * 2))


class PotBar(QWidget):
    """Horizontal fill bar, 0-100%."""

    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._pct = None
        self.set_scale(1.0)

    def set_scale(self, k):
        self.setFixedHeight(round(18 * k))
        self.setMinimumWidth(round(150 * k))

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
    WIDTH = 150

    def __init__(self):
        super().__init__()
        self._stage = None
        self._font_pt = 9
        self.set_scale(1.0)

    def set_scale(self, k):
        # Instance copies of the class defaults: the paint code reads these, so
        # scaling is a matter of rewriting them rather than threading k through
        # every drawing call.
        self.ROW_H = round(22 * k)
        self.TRACK_W = round(16 * k)
        self.PAD = round(4 * k)
        self._font_pt = round(9 * k)
        self.setFixedSize(round(self.WIDTH * k),
                          self.ROW_H * len(self.STAGES) + self.PAD * 2)

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
            radius = (5.0 if active else 2.5) * self.TRACK_W / 16.0
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour if active else IDLE))
            p.drawEllipse(QRectF(cx - radius, cy - radius, radius * 2, radius * 2))

            p.setPen(QColor(colour if active else MUTED))
            p.setFont(QFont(MONO, self._font_pt,
                            QFont.Bold if active else QFont.Normal))
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
        self._font_pt = 8
        self.set_scale(1.0)

    def set_scale(self, k):
        self.CHIP_W = round(13 * k)
        self.CHIP_H = round(16 * k)
        self.GAP = round(3 * k)
        self._font_pt = round(8 * k)
        self.setFixedSize(self._slots * (self.CHIP_W + self.GAP) + 2,
                          self.CHIP_H + 2)

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
        p.setFont(QFont(MONO, self._font_pt, QFont.Bold))

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

    def __init__(self, pin, name, caption=True, slots=9):
        super().__init__()
        self.pin = pin
        self.name = name
        self.trail = StateTrail(TOGGLE_TRAIL_LETTERS, slots=slots)
        self.presses = 0
        self._was = None
        self._captioned = caption

        self.text = _sized(QLabel(""), 8 if not caption else 9, bold=True)
        self.text.setStyleSheet(f"color:{MUTED};")

        # Inside a block the name is already on the block's caption, and the
        # readout is too wide to sit beside the trail in a third of a 1024
        # screen - so it stacks under it instead of being clipped beside it.
        box = QVBoxLayout(self) if not caption else QHBoxLayout(self)
        box.setContentsMargins(0, 0, 0, 0)
        box.setSpacing(4 if not caption else 8)
        if caption:
            box.addWidget(_caption(f"{name} ({pin})"))
        box.addWidget(self.trail)
        box.addWidget(self.text)
        if caption:
            box.addStretch(1)

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
        # The compact form drops the column padding rather than any field - it
        # sits under a block caption that is only a third of a 1024 screen wide.
        held = _held_text(self.trail.held())
        self.text.setText(
            f"{self.pin}:{0 if closed else 1} {'ON' if closed else 'OFF'} "
            f"{self.presses}x {held.strip()}"
            if not self._captioned else
            f"{self.pin}:{0 if closed else 1}  {'ON ' if closed else 'OFF'}  "
            f"{self.presses:>3}x  {held}")
        self.text.setStyleSheet(f"color:{LIVE if closed else MUTED};")


def _sized(label, points, bold=False):
    """Set a label's font AND record the size it was designed at.

    InputsPanel._apply_scale() reads that back off the widget to rescale the
    whole strip in one pass. Storing it as a Qt property rather than in a list
    means a label cannot be added to the strip and forgotten by the scaler -
    the two cannot drift apart, because they are the same object.
    """
    label.setProperty("basePt", points)
    label.setFont(QFont(MONO, points, QFont.Bold if bold else QFont.Normal))
    return label


def _caption(text):
    label = _sized(QLabel(text), 8, bold=True)
    label.setStyleSheet(f"color:{MUTED}; letter-spacing:1px;")
    return label


def _vline():
    line = QFrame()
    line.setFixedWidth(1)
    line.setStyleSheet(f"background:{LINE};")
    return line


class Block(QFrame):
    """One titled group of controls.

    The strip used to be four bare columns in a row, which left the operator to
    work out from the labels alone that the pot drives the light and the two
    unnamed pills drive the recording. Grouping them by what they DO - drive,
    tools, recording - means a glance at one box answers one question, and the
    title says which question it answers.

    Add content with `.add()`; each call is a sub-column inside the block.
    """

    def __init__(self, title):
        super().__init__()
        self.setObjectName("block")
        self.setStyleSheet(
            f"#block {{ background:#0a1119; border:1px solid {LINE};"
            f"border-radius:5px; }}")

        self._body = QHBoxLayout()
        self._body.setContentsMargins(0, 0, 0, 0)
        self._body.setSpacing(16)

        title_label = _sized(QLabel(title), 8, bold=True)
        title_label.setStyleSheet(
            f"color:{ACCENT}; letter-spacing:2px; background:transparent;")

        self._outer = QVBoxLayout(self)
        self._outer.addWidget(title_label)
        self._outer.addLayout(self._body, 1)
        self.set_scale(1.0)

    def set_scale(self, k):
        # The padding scales with the type, or the large size would put 1.4x
        # content inside 1x margins and overflow the block it sits in.
        self._outer.setContentsMargins(round(10 * k), round(6 * k),
                                       round(10 * k), round(6 * k))
        self._outer.setSpacing(round(4 * k))
        self._body.setSpacing(round(16 * k))

    def add(self, item, stretch=0):
        if isinstance(item, QWidget):
            self._body.addWidget(item, stretch)
        else:
            self._body.addLayout(item, stretch)
        return item

    def add_divider(self):
        self._body.addWidget(_vline())

    def add_stretch(self):
        self._body.addStretch(1)


def _column(caption, *widgets):
    """Caption over a stack of widgets, top-aligned - the strip's unit of layout."""
    col = QVBoxLayout()
    col.setSpacing(3)
    col.addWidget(_caption(caption))
    for widget in widgets:
        col.addWidget(widget)
    col.addStretch(1)
    return col


class SessionView(QWidget):
    """What the two recording switches and the save button are doing.

    The operator's question is never "what level is GPIO24" - it is "am I
    getting this on tape, and how much of it". So the state word and the
    recorded time are the big elements, the clip/size line is the detail
    underneath, and the raw levels stay at the bottom in the same place the
    actuator keeps its own, for the one day the decode is what is in doubt.

    Recorded time is *recorded* time: it does not advance while paused, because
    it is also the playback length of the file being written.
    """

    BLINK_HZ = 1.4

    # The save button is momentary - measured ~0.18s per press, which is 5 UI
    # frames and easy to miss. Hold its pill lit for long enough to register as
    # feedback that the press was seen, independently of the toast.
    SAVE_FLASH_S = 0.9

    def __init__(self):
        super().__init__()
        self.setMinimumWidth(256)

        self._state = None
        self._blink_on = True
        self._presses = None
        self._flash_until = 0.0

        self.head = _sized(QLabel("●  STOPPED"), 13, bold=True)
        self.head.setStyleSheet("background:transparent;")

        self.elapsed = _sized(QLabel("--:--"), 11, bold=True)
        self.elapsed.setStyleSheet(f"color:{TEXT}; background:transparent;")

        self.detail = _sized(QLabel("no session"), 8)
        self.detail.setStyleSheet(f"color:{MUTED}; background:transparent;")

        # The SAVE confirmation. Blanked rather than hidden: hide() collapses
        # the row and everything below it jumps 22px up, so every toast would
        # shove the pills the operator is reading, twice - once appearing and
        # once expiring. Keeping the row costs 22px of a block that has it.
        self.toast = _sized(QLabel(""), 9, bold=True)
        self.toast.setAlignment(Qt.AlignCenter)
        self.toast.setFixedHeight(19)

        # One pill per control, labelled with the ROLE rather than the pin, and
        # in the order they sit on the panel. The pin number stays underneath
        # for the day the decode itself is in doubt.
        self.rec_pill = Pill("START / STOP", REC, compact=True)
        self.pause_pill = Pill("PAUSE / RESUME", WARN, compact=True)
        self.save_pill = Pill("SAVE", ACCENT, compact=True)

        self.pins = _sized(QLabel(""), 8)
        self.pins.setStyleSheet(f"color:{MUTED}; background:transparent;")

        head_row = QHBoxLayout()
        head_row.setContentsMargins(0, 0, 0, 0)
        head_row.setSpacing(10)
        head_row.addWidget(self.head)
        head_row.addWidget(self.elapsed)
        head_row.addStretch(1)

        pill_row = QHBoxLayout()
        pill_row.setContentsMargins(0, 0, 0, 0)
        pill_row.setSpacing(5)
        pill_row.addWidget(self.rec_pill)
        pill_row.addWidget(self.pause_pill)
        pill_row.addWidget(self.save_pill)
        pill_row.addStretch(1)

        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(3)
        col.addLayout(head_row)
        col.addWidget(self.detail)
        col.addWidget(self.toast)
        col.addLayout(pill_row)
        col.addWidget(self.pins)
        col.addStretch(1)

    def set_scale(self, k):
        self.setMinimumWidth(round(256 * k))
        self.toast.setFixedHeight(round(19 * k))
        for pill in (self.rec_pill, self.pause_pill, self.save_pill):
            pill.set_scale(k)

    def set_status(self, status, snapshot):
        state = status.get("state") or "STOPPED"
        colour, blinks = SESSION_LOOK.get(state, SESSION_LOOK["STOPPED"])

        # Phase off the wall clock rather than a counter, so the blink rate does
        # not follow the UI frame rate when the Pi is busy.
        lit = (not blinks) or (int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        if (state, lit) != (self._state, self._blink_on):
            self._state, self._blink_on = state, lit
            self.head.setText(f"{'●' if lit else '○'}  {state}")
            self.head.setStyleSheet(f"color:{colour}; background:transparent;")

        self.elapsed.setText(hms(status.get("elapsed")) if state != "STOPPED"
                             else "--:--")

        error = status.get("error")
        if error:
            self.detail.setText(error)
            self.detail.setStyleSheet(f"color:{BAD}; background:transparent;")
        elif state == "STOPPED":
            free = status.get("free_mb")
            self.detail.setText(f"idle  ·  {free / 1024:.1f} GB free"
                                if free is not None else "idle")
            self.detail.setStyleSheet(f"color:{MUTED}; background:transparent;")
        else:
            mb = (status.get("bytes") or 0) / 1e6
            self.detail.setText(
                f"clip {status.get('clip', 0):03d}  ·  {hms(status.get('clip_elapsed'))}"
                f"  ·  {mb:,.0f} MB")
            self.detail.setStyleSheet(f"color:{MUTED}; background:transparent;")

        toast = status.get("toast")
        if toast is None:
            self.toast.setText("")
            self.toast.setStyleSheet("background:transparent; border:none;")
        else:
            text, detail = toast
            good = text.startswith("SAVED") and "EMPTY" not in text
            self.toast.setText(f"{text}   {detail}")
            self.toast.setStyleSheet(
                f"color:{BG}; background:{LIVE if good else WARN};"
                f"border-radius:4px;")

        switches = snapshot.get("switches") or {}
        self.rec_pill.set_value(switches.get("START / STOP"))
        self.pause_pill.set_value(switches.get("PAUSE / RESUME"))

        # Stretch the save button's 0.18s pulse into something visible. Driven
        # off the press COUNT, not the level, for the same reason recorder.py
        # is - the level can fall between two UI frames, the count cannot.
        presses = snapshot.get("save_presses")
        if presses is not None:
            if self._presses is not None and presses > self._presses:
                self._flash_until = time.monotonic() + self.SAVE_FLASH_S
            self._presses = presses
        self.save_pill.set_value(
            True if time.monotonic() < self._flash_until
            else (None if presses is None else False))

        pins = snapshot.get("session_pins") or {}
        rec_lvl, pause_lvl = pins.get(REC_PIN), pins.get(PAUSE_PIN)
        count = "" if not presses else f"   {presses}x"
        self.pins.setText(
            f"{REC_PIN}:-  {PAUSE_PIN}:-  {SAVE_PIN}:-"
            if rec_lvl is None or pause_lvl is None
            else f"{REC_PIN}:{rec_lvl}  {PAUSE_PIN}:{pause_lvl}  {SAVE_PIN}{count}")


class InputsPanel(QFrame):
    """The whole strip. Push state in with set_state()."""

    # The strip is drawn at one of two sizes, chosen from its own width. On the
    # 1080p operator panel the compact size left an eighth of the screen holding
    # 8pt text and reading empty; at the large size the same content fills it and
    # is legible standing up. On the 1024 headless display the large size does
    # not fit at all - the blocks alone need ~1390px of it - so this is a real
    # fork, not a preference. 1500 is the measured crossover, with margin.
    LARGE_AT = 1500
    LARGE_K = 1.4

    def __init__(self):
        super().__init__()
        self.setObjectName("inputsPanel")
        self.setStyleSheet(
            f"#inputsPanel {{ background:{BG}; border:1px solid {LINE};"
            f"border-radius:4px; }}")
        self._scale = None

        # Three blocks, grouped by what the operator is doing rather than by
        # what kind of electrical part it is: DRIVE is the two analog inputs
        # that move the robot and light the duct, TOOLS is the two switches that
        # work it, RECORDING is the three controls that keep the footage.
        #
        # Left to right in the order they are used: you drive to the spot, you
        # work it, you record it.

        # --- block 1: DRIVE --------------------------------------------------
        self.joy = JoystickView()
        self.joy_text = QLabel("—")
        self.joy_text.setFont(QFont(MONO, 8))
        self.joy_text.setStyleSheet(f"color:{MUTED};")
        self.joy_text.setAlignment(Qt.AlignCenter)

        joy_col = QVBoxLayout()
        joy_col.setSpacing(3)
        joy_col.addWidget(_caption("JOYSTICK · POSITION"), 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy, 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy_text, 0, Qt.AlignHCenter)
        joy_col.addStretch(1)

        self.pot = PotBar()
        self.pot_text = QLabel("—")
        self.pot_text.setFont(QFont(MONO, 10, QFont.Bold))
        self.pot_text.setStyleSheet(f"color:{TEXT};")

        # The ADC error belongs here and nowhere else: the joystick and the pot
        # are the only two readings it can break, and a warning parked under the
        # switches read as if the switches were the thing that had failed.
        self.status = QLabel("")
        self.status.setFont(QFont(MONO, 8))
        self.status.setStyleSheet(f"color:{WARN};")

        pot_col = QVBoxLayout()
        pot_col.setSpacing(3)
        pot_col.addWidget(_caption("LIGHT INTENSITY"))
        pot_col.addWidget(self.pot)
        pot_col.addWidget(self.pot_text)
        pot_col.addWidget(self.status)
        pot_col.addStretch(1)

        drive = Block("DRIVE")
        drive.add(joy_col)
        drive.add(pot_col, 1)

        # --- block 2: TOOLS --------------------------------------------------
        # Driven off inputs.SWITCHES rather than a second hardcoded list, so a
        # rewiring only has to be recorded in one place.
        self.pills = {}
        self.toggles = {}

        brush_col = QVBoxLayout()
        brush_col.setSpacing(3)
        brush_col.addWidget(_caption(f"BRUSH ON / OFF ({BRUSH_PIN})"))
        for name, pin in SWITCHES:
            if pin != BRUSH_PIN:
                continue
            pill = Pill(name, PILL_COLOUR.get(name, LIVE))
            self.pills[name] = pill
            brush_col.addWidget(pill, 0, Qt.AlignLeft)
        # The trail answers a different question from the pill - "did my last
        # flip take, when, and how many times" rather than "is it on now".
        brush_row = ToggleRow(BRUSH_PIN, PIN_TO_NAME.get(BRUSH_PIN, "BRUSH"),
                              caption=False, slots=7)
        self.toggles[BRUSH_PIN] = brush_row
        brush_col.addWidget(brush_row)
        brush_col.addStretch(1)

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

        # Both columns here are fixed-width content (a pill, a trail, a lever),
        # so stretching one just parks a hole beside it. Push them to the two
        # ends instead and let the hole fall between them, where it reads as
        # spacing rather than as a column that failed to fill.
        tools = Block("TOOLS")
        tools.add(brush_col)
        tools.add_stretch()
        tools.add(act_col)

        # --- block 3: RECORDING ----------------------------------------------
        self.session = SessionView()
        recording = Block("RECORDING")
        recording.add(self.session, 1)

        # Stretch factors, not natural widths: the strip spans whatever the
        # screen is, and three blocks huddled at the left with dead space to the
        # right reads as a layout that failed rather than one that fits. The
        # 3:4:3 split is each block's content need (~280 / ~370 / ~280 px), so
        # they run out of room at the same moment rather than one clipping while
        # its neighbour still has slack.
        root = self._root = QHBoxLayout(self)
        root.addWidget(drive, 3)
        root.addWidget(tools, 4)
        root.addWidget(recording, 3)

        self._apply_scale(1.0)

    # -- scale ----------------------------------------------------------------

    def _apply_scale(self, k):
        if k == self._scale:
            return
        self._scale = k
        self.setFixedHeight(round(STRIP_HEIGHT * k))
        self._root.setContentsMargins(round(10 * k), round(7 * k),
                                      round(10 * k), round(7 * k))
        self._root.setSpacing(round(10 * k))

        # Plain labels carry the size they were designed at as a property, so
        # this reaches every one of them - including any added later - without a
        # registry to keep in step. Custom widgets do their own arithmetic.
        for label in self.findChildren(QLabel):
            base = label.property("basePt")
            if base:
                font = label.font()
                font.setPointSize(round(base * k))
                label.setFont(font)
        for widget in self.findChildren(QWidget):
            scaler = getattr(widget, "set_scale", None)
            # Bound method on a child, not this panel's own _apply_scale.
            if callable(scaler) and widget is not self:
                scaler(k)

    def resizeEvent(self, event):
        self._apply_scale(self.LARGE_K if self.width() >= self.LARGE_AT else 1.0)
        super().resizeEvent(event)

    # -- state ---------------------------------------------------------------

    def set_state(self, state, session=None):
        """`state` is an inputs.py snapshot; `session` a recorder status dict.

        The two arrive together so the strip can never show a switch position
        that the recorder did not act on - the same reason main.py takes one
        input snapshot and shares it between the panel and the motors.
        """
        for name, pill in self.pills.items():
            pill.set_value(state["switches"].get(name))

        if session is not None:
            self.session.set_status(session, state)

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
    window.resize(1100, STRIP_HEIGHT + 40)

    # No streams, so no recorder - but the session column still has to render.
    # A SessionManager over an empty camera list is the real object doing real
    # state transitions, which is a better preview than a hand-made dict.
    from recorder import SessionManager

    reader = inputs.InputReader()
    reader.start()
    session = SessionManager([])

    def refresh():
        snapshot = reader.latest()
        session.on_inputs(snapshot)
        panel.set_state(snapshot, session.status())

    timer = QTimer()
    timer.timeout.connect(refresh)
    timer.start(50)

    window.show()
    code = app.exec()
    session.stop()
    reader.stop()
    sys.exit(code)

