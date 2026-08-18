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
height: every block is a CENTRED title with its control directly under it, the
actuator lever is a horizontal track rather than a three-row column, and the
save confirmation replaces the status line rather than adding a row.

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

# How long the stick display may coast on its last known position when a sample
# is rejected - see the display-hold note in set_state(). Half a second spans
# any single-sample validation blank (~40 ms at the 25 Hz analog poll) and the
# centre relearn after an ADC reopen (~360 ms), while a genuinely dead ADC
# still turns to a dash before the operator has finished noticing.
JOY_DISPLAY_HOLD_S = 0.5

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


# Everything below is sized to fill exactly this and no more; if any of it
# grows, something else has to give rather than the bar above.
# 150 is set by the tallest column: the joystick's caption + 84px box + x/y
# readout. The strip briefly ran 130 with captions beside their controls, but
# the operator wants titles on top and centred everywhere, so the three rows
# are back and so is the height that carries them.
STRIP_HEIGHT = 150

# --- palette -----------------------------------------------------------------
# The top bar's colours, so the two light surfaces are the same light. Keep them
# in step with topbar.py: BAR_LINE, INK, ACCENT and MUTED are shared by eye.
FIELD = "#eef2f9"     # the strip itself
CARD = "#ffffff"      # a card sitting on it
LINE = "#c3cee2"
INK = "#241f7a"       # headings and the numbers that get read
TEXT = "#2c3a52"      # body
MUTED = "#8590ab"
CAP_INK = "#4a5573"   # card titles - darker than MUTED so they actually read
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
# Raised from 8/11/16/10: at the old scale the captions and the small readouts
# were not legible on the real screen - the strip is read at arm's length on a
# 1024x768 panel, not on a desktop monitor two feet away. PILL now equals VALUE
# so every control in the strip is set at one size; a brush pill a point larger
# than the recording pills read as an accident rather than a hierarchy.
PT_CAP = 10
PT_VALUE = 12
PT_BIG = 17
PT_PILL = 12


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
    """Every card's title, and they must all be identical.

    CAP_INK rather than MUTED: at #8590ab on a white card these washed out at
    arm's length, and a title you cannot read is not labelling anything.
    """
    return label(text, PT_CAP, CAP_INK, bold=True, extra="letter-spacing:1px;")


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

    84px is what the 150px strip can actually give: the card spends ~17px on its
    caption and ~17px on the x/y readout under the box, leaving ~85. Asking for
    more does not make it bigger, it makes Qt squeeze the readout out.
    """

    SIDE = 84

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
        # The one rule here: pushing the stick "up" must move the dot up.
        # Which SIGN that takes depends on the stick wiring and INVERT_Y, so
        # it was set empirically on the rig (2026-08-18: the dot mirrored the
        # hand vertically after the harness rework settled on INVERT_Y=1).
        # If a future rewire flips the dot again, flip this + back to -.
        dx, dy = cx + self._x * half, cy + self._y * half
        p.setPen(QPen(QColor(ACCENT), 1))
        p.setBrush(QColor(ACCENT))
        p.drawEllipse(QRectF(dx - r, dy - r, r * 2, r * 2))


class PotBar(QWidget):
    """Circular gauge, 0-100%, with the reading in the middle.

    A dial rather than a bar because the control it mirrors IS a knob - the
    operator turns a round thing, so a round thing should move. It also carries
    its own number in the hole, which buys back the width the separate readout
    used to take beside the bar.

    The sweep starts at the bottom-left and runs 270 degrees clockwise, the way
    a physical knob is marked, so "off" and "full" sit where the hand expects.
    """

    # 92/10 rather than 84/11 so the number clears the ring. The hole is
    # SIDE - 2*THICK, and it has to fit "100%" at PT_BIG, not just "68%" -
    # at 84/11 the widest real value would have run into the arc. This column
    # has no readout under it, unlike the joystick, so it can afford the height.
    SIDE = 92
    THICK = 10.0
    START_ANGLE = 225      # bottom-left, in Qt's degrees-from-3-o'clock
    SWEEP = -270           # negative = clockwise

    def __init__(self):
        super().__init__()
        self.setFixedSize(self.SIDE, self.SIDE)
        self._pct = None

    def set_pct(self, pct):
        if pct != self._pct:
            self._pct = pct
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        m = self.THICK / 2.0 + 1.0
        ring = QRectF(m, m, self.width() - 2 * m, self.height() - 2 * m)

        # Qt's arc angles are in sixteenths of a degree.
        p.setPen(QPen(QColor(TRACK), self.THICK, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(ring, self.START_ANGLE * 16, self.SWEEP * 16)

        pct = self._pct
        if pct:
            p.setPen(QPen(QColor(ACCENT), self.THICK, Qt.SolidLine, Qt.RoundCap))
            p.drawArc(ring, self.START_ANGLE * 16,
                      int(self.SWEEP * 16 * max(0.0, min(100.0, pct)) / 100.0))

        p.setPen(QColor(INK) if pct is not None else QColor(MUTED))
        p.setFont(font(PT_BIG, bold=True))
        p.drawText(self.rect(), Qt.AlignCenter,
                   "—" if pct is None else f"{pct:.0f}%")


class ActuatorTrack(QWidget):
    """The 3-position brush-height lever, drawn as a HORIZONTAL track.

    RETRACT sits at the left and EXTEND at the right, with the current stage
    named under the track. Horizontal on the operator's ask: sideways the whole
    control is one track's thickness plus a line of text, and it sits level
    with the brush pill beside it instead of towering over it.
    """

    # (state string from inputs.py, colour when active) - left to right.
    STAGES = (("RETRACT", ACCENT), ("STOP", TEXT), ("EXTEND", LIVE))
    LABELS = {"RETRACT": "RETRACT", "STOP": "NEUTRAL", "EXTEND": "EXTEND"}

    TRACK_LEN = 96
    TRACK_W = 16
    WIDTH = 120                # room for the widest state word under the track
    HEIGHT = TRACK_W + 24      # track plus the one-word state under it

    def __init__(self, tone=ACCENT):
        super().__init__()
        self._stage = None
        self._tone = tone
        # Fixed, not Expanding: the track is drawn centred in its own width, so
        # an expanding widget would stretch the hit area without moving the
        # drawing, and the control would drift off-centre from its caption.
        self.setFixedSize(self.WIDTH, self.HEIGHT)

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
        cx = w / 2.0

        track = QRectF(cx - self.TRACK_LEN / 2.0, 1.0,
                       float(self.TRACK_LEN), float(self.TRACK_W))
        p.setPen(QPen(QColor(BAD) if fault else wash(self._tone, 0.62), 1))
        p.setBrush(QColor("#fdecea") if fault else wash(self._tone, 0.88))
        p.drawRoundedRect(track, self.TRACK_W / 2.0, self.TRACK_W / 2.0)

        # Detents inset by the track thickness at each end, so the outer two
        # sit inside the rounded caps rather than on them.
        inset = self.TRACK_W
        span = track.width() - inset * 2
        cy = track.center().y()
        for i, (key, colour) in enumerate(self.STAGES):
            active = (not fault) and key == self._stage
            dx = track.left() + inset + span * i / (len(self.STAGES) - 1)
            r = 6.0 if active else 3.0
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(colour) if active else wash(self._tone, 0.55))
            p.drawEllipse(QRectF(dx - r, cy - r, r * 2, r * 2))

        # One word under the track rather than three: the inactive stages are
        # already drawn as their detents, and three labels at a readable size
        # would not fit along a track this short.
        name = "FAULT" if fault else self.LABELS.get(self._stage, "—")
        colour = BAD if fault else dict(self.STAGES).get(self._stage, MUTED)
        p.setPen(QColor(colour))
        p.setFont(font(PT_VALUE, bold=True))
        p.drawText(QRectF(0, self.TRACK_W + 4, w, self.HEIGHT - self.TRACK_W - 4),
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

    def spread(self, *items):
        """Add columns with equal air before, between and after them.

        This is what centres a card's contents as a group: fixed-size controls
        plus stretches either side, instead of everything hugging the left edge
        with all the slack piling up on the right.
        """
        self._body.addStretch(1)
        for item in items:
            self.add(item)
            self._body.addStretch(1)


def column(cap, *widgets, spacing=3, fill=False):
    """Centred caption over centred content - the strip's layout unit.

    Everything centres on the block's own axis: the title sits over the middle
    of its control, so a block reads as one labelled instrument. Blocks are
    top-aligned by the trailing stretch, which keeps every caption in the strip
    on the same line no matter how tall the control under it is.
    """
    col = QVBoxLayout()
    col.setSpacing(spacing)
    col.setContentsMargins(0, 0, 0, 0)
    col.addWidget(caption(cap), 0, Qt.AlignHCenter)
    for w in widgets:
        if isinstance(w, QWidget):
            # fill=True for content that genuinely wants the whole card width -
            # the recording card's session view. Centring that would pin it to
            # its sizeHint and leave the card half empty.
            col.addWidget(w) if fill else col.addWidget(w, 0, Qt.AlignHCenter)
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

        # Both rows centred, a stretch on each side, to match the centred
        # captions everywhere else in the strip.
        head_row = QHBoxLayout()
        head_row.setContentsMargins(0, 0, 0, 0)
        head_row.setSpacing(9)
        head_row.addStretch(1)
        head_row.addWidget(self.head)
        head_row.addWidget(self.elapsed)
        head_row.addStretch(1)

        pill_row = QHBoxLayout()
        pill_row.setContentsMargins(0, 0, 0, 0)
        pill_row.setSpacing(9)
        # NATURAL widths - each pill is its own text plus the same padding, and
        # the leftover card width goes into the stretches either side. The row
        # used to stretch the pills to fill the card, which blew START / STOP
        # and PAUSE / RESUME up to several times the width of every other
        # control in the strip; same words + same padding is the same density.
        pill_row.addStretch(1)
        pill_row.addWidget(self.rec_pill)
        pill_row.addWidget(self.pause_pill)
        pill_row.addWidget(self.save_pill)
        pill_row.addStretch(1)

        # Order: heading, then the three buttons centred in the block, then the
        # detail line pinned to the bottom. A stretch either side of the pill row
        # is what centres it - without the second one the buttons ride up against
        # the heading and the detail line floats in the middle of the block.
        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(2)
        col.addLayout(head_row)
        col.addStretch(1)
        col.addLayout(pill_row)
        col.addStretch(1)
        col.addWidget(self.detail, 0, Qt.AlignHCenter)

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
            hold = status.get("save_hold") or 0.0
            need = status.get("save_hold_need") or 3.0
            if hold > 0.0:
                # Mid-hold the only number that matters is how much longer -
                # counted down live so the operator never guesses at 3 seconds.
                self.detail.setText(
                    f"keep holding SAVE  ·  {max(0.0, need - hold):.1f}s")
            else:
                self.detail.setText(
                    f"hold SAVE {need:.0f}s to keep  ·  "
                    f"{clips} clip{'s' if clips != 1 else ''}"
                    f"  {hms(status.get('pending_held'))}")
            recolour(self.detail, WARN, PT_CAP, bold=True)
        elif (status.get("usb") or {}).get("state") == "copying":
            # The USB daemon is mirroring /recordings onto a stick right now.
            # Shown above everything routine (but below toasts and the confirm
            # window, which are operator decisions in flight) because the one
            # mistake this line prevents is pulling the stick mid-copy.
            usb = status["usb"]
            total = usb.get("bytes_total") or 0
            pct = 100.0 * (usb.get("bytes_done") or 0) / total if total else 0.0
            self.detail.setText(
                f"USB TRANSFERRING  {pct:.0f}%  ·  {usb.get('file') or ''}"
                f"  ·  {usb.get('file_i') or 0}/{usb.get('files_total') or 0}"
                f" files")
            recolour(self.detail, WARN, PT_CAP, bold=True)
        elif (status.get("usb") or {}).get("state") == "done":
            usb = status["usb"]
            self.detail.setText(
                f"USB BACKUP DONE  ·  safe to remove  ·  "
                f"{usb.get('copied') or 0} copied, "
                f"{usb.get('deleted') or 0} cleared off Pi")
            recolour(self.detail, TONES["on"][0], PT_CAP, bold=True)
        elif (status.get("usb") or {}).get("state") == "error":
            self.detail.setText(
                f"USB BACKUP FAILED  ·  {(status['usb'].get('detail') or '')}")
            recolour(self.detail, BAD, PT_CAP, bold=True)
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
        # Last drawable stick position, for DISPLAY continuity only - see
        # set_state(). (value_x, value_y, monotonic stamp).
        self._joy_disp = None

        self.pot = PotBar()
        # Kept as a widget but no longer laid out: the dial draws its own number
        # in the middle now, and a second copy beside it read as two readings.
        # set_state() still updates it so nothing downstream has to care.
        self.pot_text = label("—", PT_BIG, INK, bold=True)
        self.pot_text.hide()
        # The ADC warning belongs here and nowhere else: the joystick and the
        # pot are the only two readings it can break.
        self.status = label("", PT_CAP, WARN)

        drive = Card(DRIVE_TONE)
        drive.spread(column("JOYSTICK", self.joy, self.joy_text),
                     column("LIGHT INTENSITY", self.pot, self.status))

        # --- TOOLS -----------------------------------------------------------
        # The caption says which control it is, the pill says what it is doing.
        # A pill reading BRUSH with the word ON under it said both twice and
        # neither clearly.
        # No points= override: every pill in the strip takes the default so the
        # brush control and the three recording controls are set identically.
        self.brush = Pill("OFF", PILL_TONE["BRUSH"], alternates=("ON", "—"))
        self.actuator = ActuatorTrack(TOOLS_TONE)

        tools = Card(TOOLS_TONE)
        tools.spread(column("BRUSH", self.brush),
                     column("BRUSH HEIGHT", self.actuator))

        # --- RECORDING -------------------------------------------------------
        # Wrapped in column() like the other two cards, so all three carry a
        # title in the same style and at the same height. Without it the
        # recording card started straight in on its state word and its contents
        # sat a caption's-worth higher than everything beside it.
        self.session = SessionView()
        recording = Card(REC_TONE)
        recording.add(column("RECORDING", self.session, fill=True), 1)

        # Stretch factors, not natural widths: the strip spans whatever the
        # screen is, and three cards huddled at the left with dead space to the
        # right reads as a layout that failed rather than one that fits. The
        # split is each card's content need, so they run out of room at the same
        # moment rather than one clipping while its neighbour still has slack.
        root = QHBoxLayout(self)
        root.setContentsMargins(9, 5, 9, 6)
        root.setSpacing(9)
        # Rebalanced when the pills stopped stretching to fill their card: the
        # old 4/3/11 existed to feed the recording card's full-width buttons,
        # and with those at natural size it just left that card mostly empty
        # while the two control cards clipped first. This split tracks each
        # card's actual content width, so slack spreads evenly.
        root.addWidget(drive, 4)
        root.addWidget(tools, 3)
        root.addWidget(recording, 5)

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
        # BOTH axes have to be checked, not just x: sample() reads each ADC
        # channel independently, so a partial read leaves one of them None. The
        # guard used to test x alone and the y format then raised every frame -
        # which aborted main.py's tick() before it reached the motor demand, and
        # MotorLink went on transmitting the last demand at 50Hz. A display bug
        # that keeps the wheels turning is worth this extra condition.
        x, y = joy.get("x"), joy.get("y")
        # DISPLAY HOLD, display only. inputs.py's validators blank a sample the
        # instant it looks wrong, and the motors must see that None immediately
        # - but painted at 30fps those single-sample blanks read as the whole
        # strip blinking, which the operator reports as "the data is not
        # continuous". Bridging gaps shorter than JOY_DISPLAY_HOLD_S keeps the
        # picture steady without touching what the motors are told; a real
        # outage (dead ADC, unzeroed stick) outlives the hold and still shows
        # as the dash it deserves.
        now = time.monotonic()
        if x is not None and y is not None:
            self._joy_disp = (x, y, now)
        elif (self._joy_disp is not None
                and now - self._joy_disp[2] < JOY_DISPLAY_HOLD_S):
            x, y = self._joy_disp[0], self._joy_disp[1]
        self.joy.set_pos(x, y)
        self.joy_text.setText(
            f"x {x:+.2f}  y {y:+.2f}"
            if x is not None and y is not None else "—")

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
