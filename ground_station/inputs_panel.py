#!/usr/bin/env python3
"""
The operator-controls strip: drive inputs, tools, recording.

Like topbar.py, nothing in here polls anything - main.py owns the single UI
timer and pushes a snapshot in through set_state(). That keeps this cheap
enough to redraw at the full UI frame rate.

Preview it standalone against the real hardware:

    python3 inputs_panel.py

DESIGN NOTES

Three cards, grouped by what the operator is doing rather than by what kind of
electrical part each control is - you drive to the spot, you work it, you record
it, left to right in that order. Each carries its own accent on its top edge
(blue / teal / red) so the group you want is found by colour before any label is
read.

It wears the same light field as the top bar. The strip used to be dark, which
made the window three themes tall: a white bar, black video, a near-black strip.

HEIGHT IS THE BINDING CONSTRAINT, and it is a budget, not an outcome. The bar
above is 54px; this one is capped at STRIP_HEIGHT and every pixel it takes comes
straight off the camera panels, which are the only reason the machine exists. So
the layout spends WIDTH - which is free, the strip spans the screen - and hoards
height: readouts sit beside their captions rather than under them, the actuator
lever is horizontal instead of a three-row column, and the save confirmation
replaces the status line rather than adding a row.

That budget is also why there is no font scaling any more. An earlier version
grew everything 1.75x on a wide screen, which read well and cost 313px - most of
a sixth of a 1080p display, nearly six times the bar above it. One size that
fits 1024 and fills 1920 is worth more than two that fight the height cap.

Pin numbers, level readouts and state trails are all gone. They were bring-up
instruments - they answered "is my wiring right" - and they were being read by an
operator asking "is the brush on". `python3 inputs.py` still prints all of it if
a wiring question ever comes back.
"""

from __future__ import annotations

import time

from PySide6.QtCore import Qt, QRectF, QSize
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QLinearGradient, QPainter, QPen,
)
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# Pin numbers only, so the caption and the reader cannot drift out of sync.
# Importing inputs touches no hardware at module scope.
from inputs import BRUSH_PIN, SWITCHES
from recorder import hms

PIN_TO_NAME = {pin: name for name, pin in SWITCHES}

# A normal UI typeface, not the terminal mono the strip used to be set in. The
# numbers that change every frame are given a fixed width in code instead, which
# is what mono was really being used for.
#
# A LIST, not a name. Naming one family that turns out not to be installed does
# not fall back to something sensible - Qt renders every glyph as an empty box,
# and the whole strip becomes tofu with the layout still perfectly correct. That
# is what happens on a machine without DejaVu (caught rendering on Windows), and
# it would happen on the Pi too if the fonts package ever slims down. Qt walks
# this list and takes the first family actually present.
SANS_FAMILIES = ["DejaVu Sans", "Noto Sans", "Liberation Sans", "Segoe UI",
                 "Helvetica Neue", "Arial"]


def font(points, bold=False):
    f = QFont()
    f.setFamilies(SANS_FAMILIES)
    f.setPointSize(points)
    f.setBold(bold)
    return f


# 124px against the top bar's 54 - 2.3x, inside the 2.5x this is allowed to
# spend. Everything below is sized to fill exactly that and no more; if any of
# it grows, something else has to give rather than the bar.
STRIP_HEIGHT = 124

# --- palette -----------------------------------------------------------------
# The top bar's colours, so the two light surfaces are the same light. Keep them
# in step with topbar.py: BAR_LINE, INK, ACCENT and MUTED are shared by eye.
FIELD = "#eef2f9"     # the strip itself
CARD = "#ffffff"      # a card sitting on it
LINE = "#c3cee2"
INK = "#241f7a"       # headings and the numbers that get read
TEXT = "#2c3a52"      # body
MUTED = "#8590ab"
ACCENT = "#3f6fb5"    # the brand blue - bars, the joystick dot
TRACK = "#dde5f2"     # empty bar / inactive detent

LIVE = "#0f7a4a"      # on / running
WARN = "#8a5a06"      # paused
BAD = "#c02b26"       # fault / error
REC = "#c0322b"       # recording

# One accent per card. TOOLS is teal rather than the obvious green: its brush
# pill goes green when on, and a card whose chrome is the same green as its
# state makes the state harder to spot, not easier.
DRIVE_TONE = "#3f6fb5"
TOOLS_TONE = "#0e7490"
REC_TONE = "#c0322b"

# state -> (text+border colour, fill). Tinted fills rather than saturated
# blocks: on a light field the colour has to read against white, which is the
# same rule topbar.py's chips follow. Off is WHITE - a tinted "off" fill on a
# white card reads as a dark box rather than as an unlit control, which is the
# opposite of what off should look like.
TONES = {
    "on":    (LIVE, "#e7f6ee"),
    "rec":   (REC, "#fdecea"),
    "pause": (WARN, "#fdf3de"),
    "info":  (ACCENT, "#e9f0fa"),
    "off":   (MUTED, CARD),
    "none":  (MUTED, CARD),
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

PILL_TONE = {"START / STOP": "rec", "PAUSE / RESUME": "pause",
             "SAVE": "info", "BRUSH": "on"}

# --- type scale --------------------------------------------------------------
# Four sizes, no more. CAP labels a thing, VALUE is the thing, BIG is the one
# number per card worth reading across the room, PILL is a control.
PT_CAP = 8
PT_VALUE = 11
PT_BIG = 16
PT_PILL = 10


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


def label(text, points, colour, bold=False, extra=""):
    """A strip label: font, colour, and no inherited background.

    Both halves matter. The SIZE has to travel in the style sheet, not through
    setFont(): main.py sets `QWidget { font-size: 12px }` for the whole app and
    a style sheet rule beats setFont(), so labels sized the other way rendered
    at a flat 12px on the Pi while the QPainter-drawn text beside them scaled
    correctly - painters do not consult style sheets. topbar.py already carries
    that warning.

    And `background:transparent`, because main.py paints EVERY QWidget in the
    app's near-black; without it each label sits in its own black box on these
    white cards.
    """
    lb = QLabel(text)
    lb.setFont(font(points, bold))
    lb.setStyleSheet(
        f"color:{colour}; background:transparent; font-size:{points}pt;"
        f"font-weight:{'bold' if bold else 'normal'}; {extra}")
    return lb


def recolour(lb, colour, points, bold=False, extra=""):
    """Restyle a label at runtime without losing its size.

    Assigning a style sheet replaces the whole rule, so a call that set only the
    colour would silently drop the font size back to the app-wide 12px.
    """
    lb.setStyleSheet(
        f"color:{colour}; background:transparent; font-size:{points}pt;"
        f"font-weight:{'bold' if bold else 'normal'}; {extra}")
    return lb


def caption(text):
    return label(text, PT_CAP, MUTED, bold=True, extra="letter-spacing:1px;")


class Pill(QLabel):
    """One control's state. Tinted in its own colour when on, outlined when off.

    The colour is per-pill rather than always green because these are not
    interchangeable: START / STOP lighting up in the same green as the brush
    would make the one control you must never misread the one that blends in.
    """

    def __init__(self, name, tone="on", alternates=(), points=PT_PILL):
        super().__init__(name)
        self._name = name
        self._tone = tone
        self._pt = points
        # Every other word this pill can show, so the width floor covers them
        # all and the label never resizes as its state changes.
        self._alternates = tuple(alternates)
        self._value = None
        self._pad = 12
        self.setAlignment(Qt.AlignCenter)
        self.setFont(font(points, bold=True))
        self.setFixedHeight(points * 2 + 8)
        # A QLabel's minimumSizeHint is smaller than its sizeHint - it will
        # happily be squeezed and clip its own text, which is how
        # "PAUSE / RESUME" once became "USE / RESU". Pin the floor to the widest
        # word this pill can hold, from metrics rather than sizeHint(), which is
        # not reliable before the widget is polished. The generous slack covers
        # the border and Qt's internal margin, which differ per platform - the
        # same floor was 1px clear on Windows and clipped on the Pi.
        widest = max([name, *self._alternates], key=len)
        self.setMinimumWidth(
            QFontMetrics(self.font()).horizontalAdvance(widest)
            + 2 * self._pad + 12)
        self.set_value(None)

    def set_text(self, text):
        """Change the word without resizing - the floor already covers it."""
        if text != self.text():
            self.setText(text)

    def set_value(self, on):
        self._value = on
        if on is None:
            fg, bg, border = TONES["none"][0], TONES["none"][1], LINE
        elif on:
            fg, bg = TONES[self._tone]
            border = fg
        else:
            fg, bg, border = TONES["off"][0], TONES["off"][1], LINE
        self.setStyleSheet(
            f"color:{fg}; background:{bg}; border:1px solid {border};"
            f"border-radius:{self.height() // 2}px; padding:0 {self._pad}px;"
            f"font-size:{self._pt}pt; font-weight:bold;")


class JoystickView(QWidget):
    """Crosshair box with a dot at the stick position.

    A 2D dot is the whole point: two separate bars make you reconstruct the
    diagonal in your head, which is the one thing an operator reads at a glance.
    Sized to the strip's height budget rather than to comfort - at 62px it is
    still an unambiguous 2D position, which is all it has to be.
    """

    SIDE = 62

    def __init__(self):
        super().__init__()
        self._x = self._y = None
        self.setFixedSize(self.SIDE, self.SIDE)

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
            p.setFont(font(7))
            p.drawText(box, Qt.AlignCenter, "no ADC")
            return

        r = 5.0
        half = (box.width() - 14) / 2.0
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
        self.setFixedHeight(20)
        self.setMinimumWidth(110)
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
        p.setBrush(QColor(TRACK))
        p.drawRoundedRect(box, 4, 4)
        if not self._pct:
            return
        inner = box.adjusted(1.5, 1.5, -1.5, -1.5)
        inner.setWidth(max(0.0, inner.width() * self._pct / 100.0))
        # Lighter at the top, so the fill has a lit edge rather than reading as
        # a flat rectangle of colour.
        fill = QLinearGradient(inner.topLeft(), inner.bottomLeft())
        fill.setColorAt(0.0, wash(ACCENT, 0.22))
        fill.setColorAt(1.0, QColor(ACCENT))
        p.setPen(Qt.NoPen)
        p.setBrush(fill)
        p.drawRoundedRect(inner, 3, 3)


class ActuatorTrack(QWidget):
    """The 3-position brush-height lever, drawn as a horizontal track.

    It used to be three stacked rows, which reads beautifully and costs 3x the
    height - and height is the one thing this strip has none of. Laid on its
    side it carries the same three facts (there are three stages, which one is
    current, which way the next throw goes) in a third of the space, spending
    width the strip has to spare.

    Left to right is retract -> extend, so the dot travels the way the rod does.
    """

    # (state string from inputs.py, colour when active)
    STAGES = (("RETRACT", ACCENT), ("STOP", TEXT), ("EXTEND", LIVE))
    LABELS = {"RETRACT": "RETRACT", "STOP": "NEUTRAL", "EXTEND": "EXTEND"}

    TRACK_H = 16
    HEIGHT = 40

    def __init__(self, tone=ACCENT):
        super().__init__()
        self._stage = None
        self._tone = tone
        self.setFixedHeight(self.HEIGHT)
        self.setMinimumWidth(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

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
        w = self.width()

        track = QRectF(1.0, 0.0, float(w - 2), float(self.TRACK_H))
        p.setPen(QPen(QColor(BAD) if fault else wash(self._tone, 0.62), 1))
        p.setBrush(QColor("#fdecea") if fault else wash(self._tone, 0.88))
        p.drawRoundedRect(track, self.TRACK_H / 2.0, self.TRACK_H / 2.0)

        inset = self.TRACK_H
        span = track.width() - inset * 2
        cy = track.center().y()
        for i, (key, colour) in enumerate(self.STAGES):
            active = (not fault) and key == self._stage
            cx = track.left() + inset + span * i / (len(self.STAGES) - 1)
            r = 6.0 if active else 3.0
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour) if active else wash(self._tone, 0.55))
            p.drawEllipse(QRectF(cx - r, cy - r, r * 2, r * 2))

        # One word under the track rather than three: the inactive stages are
        # already drawn as their detents, and three labels at a readable size
        # would not fit the width the track needs.
        name = "FAULT" if fault else self.LABELS.get(self._stage, "—")
        colour = BAD if fault else dict(self.STAGES).get(self._stage, MUTED)
        p.setPen(QColor(colour))
        p.setFont(font(PT_VALUE, bold=True))
        p.drawText(QRectF(0, self.TRACK_H + 1, w, self.HEIGHT - self.TRACK_H - 1),
                   Qt.AlignHCenter | Qt.AlignVCenter, name)


class Card(QFrame):
    """One group of controls. Add columns with .add()."""

    def __init__(self, tone=ACCENT):
        super().__init__()
        self.setObjectName("card")
        # A coloured top edge rather than a coloured card: the card has to stay
        # near-white for the tinted state fills inside it to read, so the colour
        # goes on the one edge that can carry it without tinting the contents.
        self.setStyleSheet(
            f"#card {{ background:{CARD}; border:1px solid {LINE};"
            f"border-top:3px solid {tone}; border-radius:6px; }}")
        self._body = QHBoxLayout(self)
        self._body.setContentsMargins(11, 5, 11, 5)
        self._body.setSpacing(14)

    def add(self, item, stretch=0):
        if isinstance(item, QWidget):
            self._body.addWidget(item, stretch)
        else:
            self._body.addLayout(item, stretch)
        return item


def column(cap, *widgets, spacing=3):
    """Caption over content - the strip's unit of layout."""
    col = QVBoxLayout()
    col.setSpacing(spacing)
    col.setContentsMargins(0, 0, 0, 0)
    col.addWidget(caption(cap))
    for w in widgets:
        if isinstance(w, QWidget):
            col.addWidget(w)
        else:
            col.addLayout(w)
    col.addStretch(1)
    return col


class SessionView(QWidget):
    """What the two recording switches and the save button are doing.

    The operator's question is never "what level is GPIO24" - it is "am I
    getting this on tape, and how much of it". So the state word and the clock
    are the big elements and everything else is one line under them.

    Recorded time is *recorded* time: it does not advance while paused, because
    it is also the playback length of the file being written.
    """

    BLINK_HZ = 1.4

    # The save button is momentary - measured ~0.18s per press, which is a
    # handful of UI frames and easy to miss. Hold its pill lit long enough to
    # register as feedback that the press was seen.
    SAVE_FLASH_S = 0.9

    def __init__(self):
        super().__init__()
        self._state = None
        self._blink_on = True
        self._presses = None
        self._flash_until = 0.0

        self.head = label("● STOPPED", PT_BIG, MUTED, bold=True)
        self.elapsed = label("--:--", PT_BIG, INK, bold=True)
        # The only text here that changes every frame. Fixed to its widest
        # reading so the layout beside it never twitches - which is the whole
        # thing mono was ever buying.
        self.elapsed.setFixedWidth(
            QFontMetrics(self.elapsed.font()).horizontalAdvance("0:00:00") + 6)

        # One status line, not two. The SAVE confirmation REPLACES it rather
        # than adding a row, because a row costs ~16px of a 124px budget and the
        # two never need to be read at the same moment.
        self.detail = label("idle", PT_CAP, MUTED)

        self.rec_pill = Pill("START / STOP", PILL_TONE["START / STOP"])
        self.pause_pill = Pill("PAUSE / RESUME", PILL_TONE["PAUSE / RESUME"])
        self.save_pill = Pill("SAVE", PILL_TONE["SAVE"])

        head_row = QHBoxLayout()
        head_row.setContentsMargins(0, 0, 0, 0)
        head_row.setSpacing(9)
        head_row.addWidget(self.head)
        head_row.addWidget(self.elapsed)
        head_row.addStretch(1)

        pill_row = QHBoxLayout()
        pill_row.setContentsMargins(0, 0, 0, 0)
        pill_row.setSpacing(7)
        # Weighted by label length so all three end up the same visual density
        # rather than the same width - "SAVE" in a pill as wide as
        # "PAUSE / RESUME" is mostly empty pill.
        pill_row.addWidget(self.rec_pill, 6)
        pill_row.addWidget(self.pause_pill, 7)
        pill_row.addWidget(self.save_pill, 3)

        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(2)
        col.addLayout(head_row)
        col.addWidget(self.detail)
        col.addStretch(1)
        col.addLayout(pill_row)

    def set_status(self, status, snapshot):
        state = status.get("state") or "STOPPED"
        tone, blinks = SESSION_LOOK.get(state, SESSION_LOOK["STOPPED"])
        colour = TONES[tone][0]
        left = status.get("pending_left")

        # Phase off the wall clock rather than a counter, so the blink rate does
        # not follow the UI frame rate when the Pi is busy.
        lit = (not blinks) or (int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        if (state, lit) != (self._state, self._blink_on):
            self._state, self._blink_on = state, lit
            self.head.setText(f"{'●' if lit else '○'} {state}")
            recolour(self.head, colour, PT_BIG, bold=True)

        if left is not None:
            # The clock becomes a countdown: during this window the number that
            # matters is how long is left to act, not how long the run was.
            self.elapsed.setText(f"{left:.0f}s")
        else:
            self.elapsed.setText(hms(status.get("elapsed"))
                                 if state != "STOPPED" else "--:--")

        toast = status.get("toast")
        error = status.get("error")
        if toast is not None:
            text, extra = toast
            # DISCARDED gets the recording red, not amber - footage was deleted,
            # and that has to look different from a save that found nothing.
            tone_key = ("rec" if text.startswith("DISCARDED")
                        else "on" if text == "SAVED" else "pause")
            self.detail.setText(f"{text}   {extra}")
            recolour(self.detail, TONES[tone_key][0], PT_CAP, bold=True)
        elif left is not None:
            clips = status.get("pending_clips") or 0
            self.detail.setText(
                f"press SAVE to keep  ·  {clips} clip{'s' if clips != 1 else ''}"
                f"  {hms(status.get('pending_held'))}")
            recolour(self.detail, WARN, PT_CAP, bold=True)
        elif error:
            self.detail.setText(error)
            recolour(self.detail, BAD, PT_CAP, bold=True)
        elif state == "STOPPED":
            free = status.get("free_mb")
            self.detail.setText(f"idle  ·  {free / 1024:.1f} GB free"
                                if free is not None else "idle")
            recolour(self.detail, MUTED, PT_CAP)
        else:
            mb = (status.get("bytes") or 0) / 1e6
            self.detail.setText(
                f"clip {status.get('clip', 0):03d}  ·  "
                f"{hms(status.get('clip_elapsed'))}  ·  {mb:,.0f} MB")
            recolour(self.detail, MUTED, PT_CAP)

        switches = snapshot.get("switches") or {}
        self.rec_pill.set_value(switches.get("START / STOP"))
        self.pause_pill.set_value(switches.get("PAUSE / RESUME"))

        presses = snapshot.get("save_presses")
        if presses is not None:
            if self._presses is not None and presses > self._presses:
                self._flash_until = time.monotonic() + self.SAVE_FLASH_S
            self._presses = presses
        if left is not None:
            # Through the confirm window the SAVE pill pulses on its own: it is
            # the only control that can keep the recording, and the operator has
            # a counted number of seconds to find it.
            self.save_pill.set_value(
                int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        else:
            self.save_pill.set_value(
                True if time.monotonic() < self._flash_until
                else (None if presses is None else False))


class InputsPanel(QFrame):
    """The whole strip. Push state in with set_state()."""

    def __init__(self):
        super().__init__()
        self.setObjectName("inputsPanel")
        self.setFixedHeight(STRIP_HEIGHT)
        # Mirrors the top bar's gradient - that one runs white at the top to
        # #e8edf6 at the bottom, so running this one the other way puts the two
        # deepest edges against the video and frames it.
        #
        # The gradient has to stay on ONE line: Qt's style sheet parser ends the
        # value at the newline, so a wrapped qlineargradient(...) silently drops
        # the whole rule and the strip keeps the app's dark background.
        self.setStyleSheet(
            f"#inputsPanel {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {FIELD}, stop:1 #ffffff);"
            f"border-top:1px solid {LINE}; }}")

        # --- DRIVE -----------------------------------------------------------
        self.joy = JoystickView()
        self.joy_text = label("—", PT_CAP, MUTED)
        self.joy_text.setAlignment(Qt.AlignHCenter)

        self.pot = PotBar()
        self.pot_text = label("—", PT_BIG, INK, bold=True)
        # The ADC warning belongs here and nowhere else: the joystick and the
        # pot are the only two readings it can break.
        self.status = label("", PT_CAP, WARN)

        # Bar and number side by side, not stacked: the number is the reading
        # and the bar is its shape, and putting them on one line costs a row
        # this strip cannot spare.
        pot_row = QHBoxLayout()
        pot_row.setContentsMargins(0, 0, 0, 0)
        pot_row.setSpacing(10)
        pot_row.addWidget(self.pot, 1)
        pot_row.addWidget(self.pot_text)

        drive = Card(DRIVE_TONE)
        drive.add(column("JOYSTICK", self.joy, self.joy_text))
        drive.add(column("LIGHT INTENSITY", pot_row, self.status), 1)

        # --- TOOLS -----------------------------------------------------------
        # The caption says which control it is, the pill says what it is doing.
        # A pill reading BRUSH with the word ON under it said both twice and
        # neither clearly.
        self.brush = Pill("OFF", PILL_TONE["BRUSH"], alternates=("ON", "—"),
                          points=PT_VALUE)
        self.actuator = ActuatorTrack(TOOLS_TONE)

        tools = Card(TOOLS_TONE)
        tools.add(column("BRUSH", self.brush))
        tools.add(column("BRUSH HEIGHT", self.actuator), 1)

        # --- RECORDING -------------------------------------------------------
        self.session = SessionView()
        recording = Card(REC_TONE)
        recording.add(self.session, 1)

        # Stretch factors, not natural widths: the strip spans whatever the
        # screen is, and three cards huddled at the left with dead space to the
        # right reads as a layout that failed rather than one that fits. The
        # split is each card's content need, so they run out of room at the same
        # moment rather than one clipping while its neighbour still has slack.
        root = QHBoxLayout(self)
        root.setContentsMargins(9, 5, 9, 6)
        root.setSpacing(9)
        root.addWidget(drive, 7)
        root.addWidget(tools, 6)
        root.addWidget(recording, 8)

    def minimumSizeHint(self):
        # Without this the strip is a floor on the whole WINDOW's width: a
        # layout propagates a child's minimum all the way up, so the window
        # could not be made narrower than the strip's content and resize() was
        # silently clamped. topbar.py carries the same override for the same
        # reason.
        return QSize(320, STRIP_HEIGHT)

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
            f"x {joy['x']:+.2f}  y {joy['y']:+.2f}"
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
    import main as _main
    from recorder import SessionManager

    app = QApplication(sys.argv)
    app.setStyleSheet(_main.STYLESHEET)      # the size rule the strip fights
    window = QWidget()
    panel = InputsPanel()
    box = QVBoxLayout(window)
    box.setContentsMargins(0, 0, 0, 0)
    box.addStretch(1)
    box.addWidget(panel)
    window.resize(1280, STRIP_HEIGHT + 120)

    # No streams, so nothing is encoded - but the session card still has to
    # render. A SessionManager over an empty camera list is the real object
    # doing real state transitions, which beats a hand-made dict.
    reader = inputs.InputReader()
    reader.start()
    session = SessionManager([])

    def refresh():
        snapshot = reader.latest()
        session.on_inputs(snapshot)
        session.poll()
        panel.set_state(snapshot, session.status())

    timer = QTimer()
    timer.timeout.connect(refresh)
    timer.start(50)

    window.show()
    code = app.exec()
    session.stop()
    reader.stop()
    sys.exit(code)
