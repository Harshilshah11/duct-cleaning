#!/usr/bin/env python3
"""
The operator-controls strip: drive inputs, tools, recording.

Like topbar.py, nothing in here polls anything - main.py owns the single UI
timer and pushes a snapshot in through set_state(). That keeps this cheap
enough to redraw at the full UI frame rate.

Preview it standalone against the real hardware:

    python3 inputs_panel.py

DESIGN NOTES

Three blocks, grouped by what the operator is doing rather than by what kind of
electrical part each control is - you drive to the spot, you work it, you record
it, left to right in that order.

It wears the same light field as the top bar. The strip used to be dark, which
made the window three themes tall: a white bar, black video, a near-black strip.
Light top and bottom with the video between them is one frame around one picture,
and the video is the only thing here that wants a dark surround.

Pin numbers, level readouts and state trails are all gone. They were bring-up
instruments - they answered "is my wiring right" - and they were being read by an
operator asking "is the brush on". `python3 inputs.py` still prints all of it if
a wiring question ever comes back.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QLinearGradient, QPainter, QPen,
)
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# Pin numbers only, so the caption and the reader cannot drift out of sync.
# Importing inputs touches no hardware at module scope.
from inputs import BRUSH_PIN, PAUSE_PIN, REC_PIN, SAVE_PIN, SWITCHES
from recorder import hms

PIN_TO_NAME = {pin: name for name, pin in SWITCHES}

# A normal UI typeface, not the terminal mono the strip used to be set in. The
# numbers that change every frame are given a fixed width in code instead, which
# is what mono was really being used for.
#
# A LIST, not a name. Naming one family that turns out not to be installed does
# not fall back to something sensible - Qt renders every glyph as an empty box,
# and the whole strip becomes tofu with the layout still perfectly correct. That
# is what happens on a machine without DejaVu (this was caught rendering on
# Windows), and it would happen on the Pi too if the fonts package ever slims
# down. Qt walks this list and takes the first family actually present.
SANS_FAMILIES = ["DejaVu Sans", "Noto Sans", "Liberation Sans", "Segoe UI",
                 "Helvetica Neue", "Arial"]
# For style sheets, which take a CSS-style comma list.
SANS_CSS = ", ".join(f'"{f}"' for f in SANS_FAMILIES) + ", sans-serif"


def font(points, bold=False):
    f = QFont()
    f.setFamilies(SANS_FAMILIES)
    f.setPointSize(points)
    f.setBold(bold)
    return f

# Height at the compact scale; InputsPanel multiplies it by LARGE_K on a wide
# screen. The floor is the joystick box plus its caption and readout - the one
# element here that cannot be made shorter without making it useless.
#
# 179 is measured, not chosen: it is what the DRIVE block's sizeHint asks for.
# Guessing gave 162, which cost the joystick box its bottom edge and dropped the
# x/y readout onto the block border - and a fixed height clips silently rather
# than reporting anything. Re-measure it whenever anything in DRIVE grows, or
# any block gains chrome: the 3px accent border alone moved it by 2.
STRIP_HEIGHT = 179

# --- palette -----------------------------------------------------------------
# The top bar's colours, so the two light surfaces are the same light. Keep them
# in step with topbar.py: BAR_LINE, INK, ACCENT and MUTED are shared by eye.
FIELD = "#eef2f9"     # the strip itself
CARD = "#ffffff"      # a block sitting on it
LINE = "#c3cee2"
INK = "#241f7a"       # headings
TEXT = "#2c3a52"      # body
MUTED = "#8590ab"
ACCENT = "#3f6fb5"    # the brand blue - captions, bars, the joystick dot
TRACK = "#dde5f2"     # empty bar / inactive detent

LIVE = "#0f7a4a"      # on / running
WARN = "#8a5a06"      # paused
BAD = "#c02b26"       # fault / error
REC = "#c0322b"       # recording

# One accent per block, so the three groups are told apart by colour before the
# titles are read - which is how you find the one you want without scanning.
# TOOLS is teal rather than the obvious green: its brush pill goes green when
# on, and a block whose chrome is the same green as its state makes the state
# harder to spot, not easier.
DRIVE_TONE = "#3f6fb5"      # brand blue
TOOLS_TONE = "#0e7490"      # teal
REC_TONE = "#c0322b"        # the recording red

# state -> (text+border colour, fill). Tinted fills rather than saturated
# blocks: on a light field the colour has to read against white, which is the
# same rule topbar.py's chips follow.
TONES = {
    "on":    (LIVE, "#e7f6ee"),
    "rec":   (REC, "#fdecea"),
    "pause": (WARN, "#fdf3de"),
    "info":  (ACCENT, "#e9f0fa"),
    "off":   (MUTED, "#f5f8fc"),
    "none":  (MUTED, "#f0f3f9"),
}

# Session state -> (tone key, does the dot blink). Blinking is reserved for the
# two states where being wrong about it costs the footage: RECORDING, and the
# window in which the recording is about to be deleted for want of a button.
SESSION_LOOK = {
    "RECORDING": ("rec", True),
    "PAUSED": ("pause", False),
    "SAVE?": ("pause", True),
    "STOPPED": ("off", False),
}

# Per-control lit tone, keyed on the names in inputs.SWITCHES.
PILL_TONE = {"START / STOP": "rec", "PAUSE / RESUME": "pause",
             "SAVE": "info", "BRUSH": "on"}


def wash(colour, amount):
    """Blend `colour` toward white. amount 0 = untouched, 1 = white.

    QColor.lighter() raises the HSV value, which on a dark saturated hue does
    not fade it - it turns it neon. The teal used by TOOLS came back as a bright
    cyan that pulled the eye harder than the state it was framing. Interpolating
    to white instead gives the same soft tint from any base hue, which is what a
    tint is actually for.
    """
    c, w = QColor(colour), QColor("#ffffff")
    mix = lambda a, b: round(a + (b - a) * amount)
    return QColor(mix(c.red(), w.red()), mix(c.green(), w.green()),
                  mix(c.blue(), w.blue()))


def _sized(label, points, bold=False):
    """Set a label's font AND record the size it was designed at.

    InputsPanel._apply_scale() reads that back off the widget to rescale the
    whole strip in one pass. Storing it as a Qt property rather than in a list
    means a label cannot be added to the strip and forgotten by the scaler -
    the two cannot drift apart, because they are the same object.
    """
    label.setProperty("basePt", points)
    label.setFont(font(points, bold))
    return label


def _tint(label, colour, extra=""):
    """Colour a label AND kill its inherited background.

    `background:transparent` is not decoration. main.py paints EVERY QWidget in
    the app's near-black, and a QLabel is a QWidget as far as that rule is
    concerned - so on these white cards each label would otherwise sit in its
    own black box. topbar.py carries the same rule for the same reason.
    """
    label.setStyleSheet(f"color:{colour}; background:transparent; {extra}")
    return label


def _label(text, points, colour, bold=False):
    return _tint(_sized(QLabel(text), points, bold), colour)


def _caption(text):
    return _tint(_sized(QLabel(text), 8, bold=True), MUTED, "letter-spacing:1px;")


class Pill(QLabel):
    """One control's state. Tinted in its own colour when on, flat when off,
    outlined when the reader has nothing to say yet.

    The colour is per-pill rather than always green because these are not
    interchangeable: START / STOP lighting up in the same green as the brush
    would make the one control you must never misread the one that blends in.
    """

    def __init__(self, name, tone="on", compact=False, alternates=()):
        super().__init__(name)
        self._name = name
        self._tone = tone
        self._compact = compact
        # Every other word this pill can show, so the width floor covers them
        # all and the label never resizes as its state changes.
        self._alternates = tuple(alternates)
        self._value = None
        self.setAlignment(Qt.AlignCenter)
        self.set_scale(1.0)

    def set_text(self, text):
        """Change the word without losing the width floor it was sized for.

        A pill whose text changes (ON/OFF) must not resize as it does, or the
        column beside it steps sideways every time the brush is switched. The
        floor is held at the WIDEST word this pill will ever show.
        """
        if text != self.text():
            self.setText(text)

    def _width_floor(self, k):
        widest = max([self._name, self.text()] + list(self._alternates),
                     key=len)
        return (QFontMetrics(self.font()).horizontalAdvance(widest)
                + 2 * self._pad + 4)

    def set_scale(self, k):
        self._pad = round((9 if self._compact else 12) * k)
        self._radius = round((10 if self._compact else 12) * k)
        self.setFont(font(round((8 if self._compact else 9) * k), bold=True))
        self.setFixedHeight(round((21 if self._compact else 25) * k))
        # A QLabel's minimumSizeHint is smaller than its sizeHint - it will
        # happily be squeezed and clip its own text, which is how
        # "PAUSE / RESUME" once became "USE / RESU". Pin the floor to the text
        # the pill actually holds, from metrics rather than from sizeHint(),
        # which is not reliable before the widget is polished.
        self.setMinimumWidth(self._width_floor(k))
        self.set_value(self._value)

    def set_value(self, on):
        self._value = on
        if on is None:
            fg, bg = TONES["none"]
            border = LINE
        elif on:
            fg, bg = TONES[self._tone]
            border = fg
        else:
            fg, bg = TONES["off"]
            border = LINE
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
        self.setFixedSize(round(86 * k), round(86 * k))

    def set_pos(self, x, y):
        if (x, y) != (self._x, self._y):
            self._x, self._y = x, y
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        box = QRectF(1, 1, self.width() - 2, self.height() - 2)

        p.setPen(QPen(wash(ACCENT, 0.60), 1))
        p.setBrush(wash(ACCENT, 0.93))
        p.drawRoundedRect(box, 5, 5)

        cx, cy = box.center().x(), box.center().y()
        p.setPen(QPen(wash(ACCENT, 0.72), 1, Qt.DashLine))
        p.drawLine(int(box.left()), int(cy), int(box.right()), int(cy))
        p.drawLine(int(cx), int(box.top()), int(cx), int(box.bottom()))

        if self._x is None or self._y is None:
            p.setPen(QColor(MUTED))
            p.setFont(font(round(8 * self._k)))
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
        self._radius = round(4 * k)
        self.setFixedHeight(round(16 * k))
        self.setMinimumWidth(round(140 * k))

    def set_pct(self, pct):
        if pct != self._pct:
            self._pct = pct
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        box = QRectF(0.5, 0.5, self.width() - 1, self.height() - 1)
        p.setPen(QPen(QColor(LINE), 1))
        p.setBrush(QColor(TRACK))
        p.drawRoundedRect(box, self._radius, self._radius)
        if not self._pct:
            return
        inner = box.adjusted(1.5, 1.5, -1.5, -1.5)
        inner.setWidth(max(0.0, inner.width() * self._pct / 100.0))
        # Lighter at the top, so the fill has a lit edge rather than reading as
        # a flat rectangle of colour. This is the one element on the strip big
        # enough for the shading to be worth anything.
        fill = QLinearGradient(inner.topLeft(), inner.bottomLeft())
        fill.setColorAt(0.0, wash(ACCENT, 0.22))
        fill.setColorAt(1.0, QColor(ACCENT))
        p.setPen(Qt.NoPen)
        p.setBrush(fill)
        p.drawRoundedRect(inner, self._radius - 1, self._radius - 1)


class ActuatorStages(QWidget):
    """The 3-position actuator lever, drawn as a lever.

    A single word swapping between EXTEND/STOP/RETRACT tells you the state but
    not the travel: you cannot see at a glance that there are three stages, nor
    which way the next throw goes. Showing all three rows with a knob on a track
    reads the way the physical ON-OFF-ON switch does, so the operator maps
    screen to hand without thinking.

    Rows are ordered to match the lever: up extends, down retracts.
    """

    # (state string from inputs.py, label shown, colour when active)
    STAGES = (
        ("EXTEND", "EXTEND", LIVE),
        ("STOP", "NEUTRAL", TEXT),
        ("RETRACT", "RETRACT", ACCENT),
    )

    WIDTH = 132

    def __init__(self, tone=ACCENT):
        super().__init__()
        self._stage = None
        self._tone = tone
        self.set_scale(1.0)

    def set_scale(self, k):
        # Instance copies of the defaults: the paint code reads these, so
        # scaling is a matter of rewriting them rather than threading k through
        # every drawing call.
        self.ROW_H = round(22 * k)
        self.TRACK_W = round(14 * k)
        self.PAD = round(3 * k)
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

        track = QRectF(1.0, float(self.PAD), float(self.TRACK_W),
                       float(self.ROW_H * len(self.STAGES)))
        p.setPen(QPen(QColor(BAD) if fault else wash(self._tone, 0.62), 1))
        p.setBrush(QColor("#fdecea") if fault else wash(self._tone, 0.88))
        p.drawRoundedRect(track, self.TRACK_W / 2.0, self.TRACK_W / 2.0)

        cx = track.center().x()
        text_x = track.right() + round(9 * self.TRACK_W / 14.0)
        text_w = self.width() - text_x - 2

        for i, (key, label, colour) in enumerate(self.STAGES):
            cy = self.PAD + self.ROW_H * i + self.ROW_H / 2.0
            active = (not fault) and key == self._stage

            # detent dot: big and coloured at the current stage, small and flat
            # everywhere else, so position is legible from across the room
            radius = (5.0 if active else 2.5) * self.TRACK_W / 14.0
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour) if active else wash(self._tone, 0.55))
            p.drawEllipse(QRectF(cx - radius, cy - radius, radius * 2, radius * 2))

            p.setPen(QColor(colour if active else MUTED))
            p.setFont(font(self._font_pt, bold=active))
            p.drawText(QRectF(text_x, cy - self.ROW_H / 2.0, text_w, self.ROW_H),
                       Qt.AlignVCenter | Qt.AlignLeft, label)


class Block(QFrame):
    """One titled group of controls, drawn as a card on the strip's field.

    Add content with `.add()`; each call is a sub-column inside the block.
    """

    def __init__(self, title, tone=ACCENT):
        super().__init__()
        self.setObjectName("block")
        self._tone = tone
        # A coloured top edge rather than a coloured card: the card has to stay
        # near-white for the tinted state fills inside it to read, so the colour
        # goes on the one edge that can carry it without tinting the contents.
        self.setStyleSheet(
            f"#block {{ background:{CARD}; border:1px solid {LINE};"
            f"border-top:3px solid {tone}; border-radius:6px; }}")

        self._body = QHBoxLayout()
        self._body.setContentsMargins(0, 0, 0, 0)

        title_label = _tint(_sized(QLabel(title), 8, bold=True), tone,
                            "letter-spacing:2px;")

        self._outer = QVBoxLayout(self)
        self._outer.addWidget(title_label)
        self._outer.addLayout(self._body, 1)
        self.set_scale(1.0)

    def set_scale(self, k):
        # The padding scales with the type, or the large size would put 1.4x
        # content inside 1x margins and overflow the block it sits in.
        self._outer.setContentsMargins(round(12 * k), round(7 * k),
                                       round(12 * k), round(7 * k))
        self._outer.setSpacing(round(5 * k))
        self._body.setSpacing(round(18 * k))

    def add(self, item, stretch=0):
        if isinstance(item, QWidget):
            self._body.addWidget(item, stretch)
        else:
            self._body.addLayout(item, stretch)
        return item

    def add_stretch(self):
        self._body.addStretch(1)


def _column(caption, *widgets):
    """Caption over a stack of widgets, top-aligned - the strip's unit of layout."""
    col = QVBoxLayout()
    col.setSpacing(4)
    col.addWidget(_caption(caption))
    for widget in widgets:
        col.addWidget(widget)
    col.addStretch(1)
    return col


class SessionView(QWidget):
    """What the two recording switches and the save button are doing.

    The operator's question is never "what level is GPIO24" - it is "am I
    getting this on tape, and how much of it". So the state word and the
    recorded time are the big elements and everything else is detail under them.

    Recorded time is *recorded* time: it does not advance while paused, because
    it is also the playback length of the file being written.
    """

    BLINK_HZ = 1.4

    # The save button is momentary - measured ~0.18s per press, which is 5 UI
    # frames and easy to miss. Hold its pill lit long enough to register as
    # feedback that the press was seen, independently of the toast.
    SAVE_FLASH_S = 0.9

    def __init__(self):
        super().__init__()
        self._state = None
        self._blink_on = True
        self._presses = None
        self._flash_until = 0.0

        self.head = _label("● STOPPED", 14, MUTED, bold=True)

        self.elapsed = _label("--:--", 13, INK, bold=True)

        self.detail = _label("idle", 8, MUTED)

        # The SAVE confirmation. Blanked rather than hidden: hide() collapses
        # the row and everything below it jumps up, so every toast would shove
        # the pills the operator is reading - once appearing, once expiring.
        self.toast = _sized(QLabel(""), 9, bold=True)
        self.toast.setAlignment(Qt.AlignCenter)

        # One pill per control, labelled with the ROLE, in the order they sit
        # on the panel.
        self.rec_pill = Pill("START / STOP", PILL_TONE["START / STOP"], compact=True)
        self.pause_pill = Pill("PAUSE / RESUME", PILL_TONE["PAUSE / RESUME"],
                               compact=True)
        self.save_pill = Pill("SAVE", PILL_TONE["SAVE"], compact=True)

        head_row = QHBoxLayout()
        head_row.setContentsMargins(0, 0, 0, 0)
        head_row.setSpacing(10)
        head_row.addWidget(self.head)
        head_row.addWidget(self.elapsed)
        head_row.addStretch(1)

        pill_row = QHBoxLayout()
        pill_row.setContentsMargins(0, 0, 0, 0)
        pill_row.setSpacing(6)
        pill_row.addWidget(self.rec_pill)
        pill_row.addWidget(self.pause_pill)
        pill_row.addWidget(self.save_pill)
        pill_row.addStretch(1)

        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(4)
        col.addLayout(head_row)
        col.addWidget(self.detail)
        # Left-aligned and content-width: stretched across the block it reads as
        # a status bar that is always there, rather than as a confirmation that
        # just fired. The height stays fixed so the pills below never move.
        col.addWidget(self.toast, 0, Qt.AlignLeft)
        col.addLayout(pill_row)
        col.addStretch(1)

        self.set_scale(1.0)

    def set_scale(self, k):
        self.setMinimumWidth(round(250 * k))
        self.toast.setFixedHeight(round(19 * k))
        # The clock is the only text here that changes every frame. Give it the
        # width of its widest reading so the layout beside it never twitches -
        # which is the whole reason the strip used to be set in mono.
        self.elapsed.setFixedWidth(
            QFontMetrics(self.elapsed.font()).horizontalAdvance("0:00:00") + 4)
        for pill in (self.rec_pill, self.pause_pill, self.save_pill):
            pill.set_scale(k)

    def set_status(self, status, snapshot):
        state = status.get("state") or "STOPPED"
        tone, blinks = SESSION_LOOK.get(state, SESSION_LOOK["STOPPED"])
        colour = TONES[tone][0]

        # Phase off the wall clock rather than a counter, so the blink rate does
        # not follow the UI frame rate when the Pi is busy.
        lit = (not blinks) or (int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        if (state, lit) != (self._state, self._blink_on):
            self._state, self._blink_on = state, lit
            self.head.setText(f"{'●' if lit else '○'} {state}")
            _tint(self.head, colour)

        left = status.get("pending_left")

        if left is not None:
            # The clock becomes a countdown, because during this window the
            # number that matters is how long is left to act, not how long the
            # run was - that is in the line underneath.
            self.elapsed.setText(f"{left:.0f}s")
        else:
            self.elapsed.setText(hms(status.get("elapsed"))
                                 if state != "STOPPED" else "--:--")

        error = status.get("error")
        if left is not None:
            clips = status.get("pending_clips") or 0
            self.detail.setText(
                f"press SAVE to keep  ·  {clips} clip{'s' if clips != 1 else ''}"
                f"  {hms(status.get('pending_held'))}")
            _tint(self.detail, WARN)
        elif error:
            self.detail.setText(error)
            _tint(self.detail, BAD)
        elif state == "STOPPED":
            free = status.get("free_mb")
            self.detail.setText(f"idle  ·  {free / 1024:.1f} GB free"
                                if free is not None else "idle")
            _tint(self.detail, MUTED)
        else:
            mb = (status.get("bytes") or 0) / 1e6
            self.detail.setText(
                f"clip {status.get('clip', 0):03d}  ·  "
                f"{hms(status.get('clip_elapsed'))}  ·  {mb:,.0f} MB")
            self.detail.setStyleSheet(f"color:{MUTED}; background:transparent;")

        toast = status.get("toast")
        if toast is None:
            self.toast.setText("")
            self.toast.setStyleSheet("background:transparent; border:none; color:%s;" % MUTED)
        else:
            text, detail = toast
            # DISCARDED gets the recording red, not the amber "something is a
            # bit off" - footage was deleted, and that has to look different
            # from a save that merely found nothing to write.
            fg, bg = TONES["rec" if text.startswith("DISCARDED")
                           else "on" if text == "SAVED" else "pause"]
            self.toast.setText(f"{text}   {detail}")
            self.toast.setStyleSheet(
                f"color:{fg}; background:{bg}; border:1px solid {fg};"
                f"border-radius:4px; padding:0 10px;")

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
        # Through the confirm window the SAVE pill pulses on its own: it is the
        # only control that can keep the recording, and the operator has a
        # counted number of seconds to find it.
        if left is not None:
            self.save_pill.set_value(
                int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        else:
            self.save_pill.set_value(
                True if time.monotonic() < self._flash_until
                else (None if presses is None else False))


class InputsPanel(QFrame):
    """The whole strip. Push state in with set_state()."""

    # The strip is drawn at one of two sizes, chosen from its own width. On the
    # 1080p operator panel the compact size left an eighth of the screen holding
    # 8pt text and reading empty; at the large size the same content fills it
    # and is legible standing up. On the 1024 headless display the large size
    # does not fit at all, so this is a real fork, not a preference. 1500 is the
    # measured crossover, with margin.
    LARGE_AT = 1500
    # 1.75, up from 1.4. The ceiling is not taste, it is the video: the strip is
    # a fixed STRIP_HEIGHT * k tall and every pixel it takes comes off the
    # camera panels. At 1.75 the strip is ~313px of a 1080p screen, leaving the
    # video ~713 - still the large majority of the window, and the text is
    # readable standing back from the panel rather than leaning into it.
    LARGE_K = 1.75

    def __init__(self):
        super().__init__()
        self.setObjectName("inputsPanel")
        # Mirrors the top bar's gradient - that one runs white at the top to
        # #e8edf6 at the bottom, so running this one the other way puts the two
        # deepest edges against the video and frames it, instead of two flat
        # slabs bolted either side of it.
        #
        # The gradient has to stay on ONE line: Qt's style sheet parser ends the
        # value at the newline, so a wrapped qlineargradient(...) silently drops
        # the whole rule and the strip keeps the app's dark background.
        self.setStyleSheet(
            f"#inputsPanel {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {FIELD}, stop:1 #ffffff);"
            f"border-top:1px solid {LINE}; }}")
        self._scale = None

        # --- block 1: DRIVE --------------------------------------------------
        self.joy = JoystickView()
        self.joy_text = _label("—", 8, MUTED)
        self.joy_text.setAlignment(Qt.AlignCenter)

        joy_col = QVBoxLayout()
        joy_col.setSpacing(4)
        joy_col.addWidget(_caption("JOYSTICK"), 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy, 0, Qt.AlignHCenter)
        joy_col.addWidget(self.joy_text, 0, Qt.AlignHCenter)
        joy_col.addStretch(1)

        self.pot = PotBar()
        self.pot_text = _label("—", 12, INK, bold=True)

        # The ADC error belongs here and nowhere else: the joystick and the pot
        # are the only two readings it can break, and a warning parked under the
        # switches read as if the switches were the thing that had failed.
        self.status = _label("", 8, WARN)

        pot_col = QVBoxLayout()
        pot_col.setSpacing(4)
        pot_col.addWidget(_caption("LIGHT INTENSITY"))
        pot_col.addWidget(self.pot)
        pot_col.addWidget(self.pot_text)
        pot_col.addWidget(self.status)
        pot_col.addStretch(1)

        drive = Block("DRIVE", DRIVE_TONE)
        drive.add(joy_col)
        drive.add(pot_col, 1)

        # --- block 2: TOOLS --------------------------------------------------
        # The caption says which control it is, the pill says what it is doing.
        # A pill reading BRUSH with the word ON under it said both twice and
        # neither clearly - and "BRUSH" lighting up green does not actually tell
        # you the brush is on, it tells you something about the brush is true.
        self.brush = Pill("OFF", PILL_TONE.get("BRUSH", "on"),
                          alternates=("ON", "—"))

        brush_col = QVBoxLayout()
        brush_col.setSpacing(4)
        brush_col.addWidget(_caption("BRUSH"))
        brush_col.addWidget(self.brush, 0, Qt.AlignLeft)
        brush_col.addStretch(1)

        self.actuator = ActuatorStages(TOOLS_TONE)

        act_col = QVBoxLayout()
        act_col.setSpacing(4)
        act_col.addWidget(_caption("BRUSH HEIGHT"))
        act_col.addWidget(self.actuator)
        act_col.addStretch(1)

        # Both columns are fixed-width content (a pill, a lever), so stretching
        # one just parks a hole beside it. Push them to the two ends and let the
        # hole fall between them, where it reads as spacing rather than as a
        # column that failed to fill.
        tools = Block("TOOLS", TOOLS_TONE)
        tools.add(brush_col)
        tools.add_stretch()
        tools.add(act_col)

        # --- block 3: RECORDING ----------------------------------------------
        self.session = SessionView()
        recording = Block("RECORDING", REC_TONE)
        recording.add(self.session, 1)

        # Stretch factors, not natural widths: the strip spans whatever the
        # screen is, and three blocks huddled at the left with dead space to the
        # right reads as a layout that failed rather than one that fits. The
        # 4:3:5 split is each block's content need - RECORDING carries three
        # role pills on one line and needs the most, TOOLS is now a single word
        # and a lever and needs the least - so they run out of room at the same
        # moment rather than one clipping while its neighbour still has slack.
        root = self._root = QHBoxLayout(self)
        root.addWidget(drive, 4)
        root.addWidget(tools, 3)
        root.addWidget(recording, 5)

        self._apply_scale(1.0)

    # -- scale ----------------------------------------------------------------

    def _apply_scale(self, k):
        if k == self._scale:
            return
        self._scale = k
        self.setFixedHeight(round(STRIP_HEIGHT * k))
        self._root.setContentsMargins(round(10 * k), round(8 * k),
                                      round(10 * k), round(8 * k))
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

    # -- state ----------------------------------------------------------------

    def set_state(self, state, session=None):
        """`state` is an inputs.py snapshot; `session` a recorder status dict.

        The two arrive together so the strip can never show a switch position
        that the recorder did not act on - the same reason main.py takes one
        input snapshot and shares it between the panel and the motors.
        """
        switches = state.get("switches") or {}

        brush = switches.get(PIN_TO_NAME.get(BRUSH_PIN, "BRUSH"))
        self.brush.set_text("—" if brush is None else ("ON" if brush else "OFF"))
        self.brush.set_value(brush)

        if session is not None:
            self.session.set_status(session, state)

        self.actuator.set_stage(state.get("actuator"))

        joy = state.get("joy") or {}
        self.joy.set_pos(joy.get("x"), joy.get("y"))
        self.joy_text.setText(
            f"x {joy['x']:+.2f}   y {joy['y']:+.2f}"
            if joy.get("x") is not None else "—")

        pot = state.get("pot") or {}
        self.pot.set_pct(pot.get("pct"))
        self.pot_text.setText(
            f"{pot['pct']:.0f}%" if pot.get("pct") is not None else "—")

        # Only surfaced when something is actually wrong; a healthy rig shows a
        # blank line rather than a reassuring message nobody reads.
        self.status.setText(state.get("error") or "")


if __name__ == "__main__":
    import sys

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    import inputs
    from recorder import SessionManager

    app = QApplication(sys.argv)
    window = QWidget()
    window.setStyleSheet("background:#0b0f14;")
    panel = InputsPanel()
    box = QVBoxLayout(window)
    box.setContentsMargins(0, 0, 0, 0)
    box.addStretch(1)
    box.addWidget(panel)
    window.resize(1280, STRIP_HEIGHT + 120)

    # No streams, so nothing is encoded - but the session column still has to
    # render. A SessionManager over an empty camera list is the real object
    # doing real state transitions, which beats a hand-made dict.
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
