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

import math
import re
import time

from PySide6.QtCore import Qt, QPointF, QRectF, QSize, QTimer
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QLinearGradient, QPainter, QPainterPath, QPen,
    QPolygonF,
)
from PySide6.QtWidgets import (
    QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# Pin numbers only, so the caption and the reader cannot drift out of sync.
# Importing inputs touches no hardware at module scope.
import theme
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
# Inter leads now (theme.FAMILY_TEXT — Apple's SF Pro stand-in), and the old
# list stays behind it UNCHANGED as the tofu guard the note above describes.
# The strip is the one place in the app that renders on Windows as well as the
# Pi, so the fallbacks earn their keep: a machine without fonts-inter walks the
# list exactly as before instead of boxing every glyph.
SANS_FAMILIES = [theme.FAMILY_TEXT, "Noto Sans", "DejaVu Sans",
                 "Liberation Sans", "Segoe UI", "Helvetica Neue", "Arial"]


# Figures that hold still, for readouts that change in place. Inter's default
# digits are PROPORTIONAL — a "1" is 3px narrower than a "0" — and this PySide6
# build cannot switch on the tabular set (see theme.FAMILY_NUMERIC for the
# measurements and the two routes that were tried). Anything whose digits tick
# has to be built with numeric=True or it will slide as it counts; FIGURE_SPACE
# below depends on this outright.
NUMERIC_FAMILIES = [theme.FAMILY_NUMERIC] + theme.NUMERIC_FALLBACKS


def font(px, weight=theme.W_SEMIBOLD, numeric=False):
    """A strip font, in PIXELS, from the shared ramp.

    WAS POINTS UNTIL 2026-08-24 and that was the whole reason the strip did not
    change when theme.py landed. topbar.py, main.py and splash.py had all moved
    to theme.font_for(), which sets a PIXEL size; this module alone was still
    calling setPointSize(). At the rig's 96 DPI a point is 1.333px, so the two
    scales silently disagreed by a third - PT_VALUE "12" was 16px while
    theme.BODY "17" meant 17px - and the strip kept its old sizes while
    inheriting the new typeface. That is why the retheme looked like it had
    done nothing down here.

    Pixels, not points, for the reason theme.font_for() gives: the rig has one
    fixed display at one fixed DPI, so a pixel is a known quantity and a point
    is an indirection through a DPI that never varies.
    """
    return (theme.font_numeric(px, weight) if numeric
            else theme.font_for(px, weight))


# Everything below is sized to fill exactly this and no more; if any of it
# grows, something else has to give rather than the bar above.
# 150 is set by the tallest column: the joystick's caption + 84px box + x/y
# readout. The strip briefly ran 130 with captions beside their controls, but
# the operator wants titles on top and centred everywhere, so the three rows
# are back and so is the height that carries them.
# 170: two rows and nothing else. See CONTENT_H - the strip is now exactly a
# heading row plus one content band plus padding, so this number is derived
# rather than chosen. Briefly 200 while the brush ring was a literal 2cm; the
# ring came down to 84px when that turned out to dominate the strip, and 30px
# went back to the camera panels with it.
# ON THE 8pt GRID, and derived rather than chosen:
#
#     11 root margins + 10 card padding + 38 caption chip
#        + SPACE_1 gap + CONTENT_H 128 = 191  ->  192 (24 x SPACE_2)
#
# It grew from 177 when the type ramp went from points to theme's pixels: a
# SUBHEAD caption is 38px tall against the old 11pt's 30, and a BODY readout 42
# against 32. The strip is two rows and its own padding and nothing else, so
# when the rows get taller this number has to follow or the rows get squeezed.
#
# Every pixel still comes off the camera panels - see the module docstring. 15px
# is the price of the whole strip being on one ramp with the rest of the app.
STRIP_HEIGHT = 24 * theme.SPACE_2

# EVERY BLOCK'S CONTENT IS THIS TALL, whatever is in it.
#
# This is the whole of what makes the strip read as one instrument panel rather
# than five unrelated widgets: column() puts each control inside a box of
# exactly this height and centres it there, so every heading sits on one line
# and every control sits on one line under it, regardless of whether the block
# holds a 76px joystick, a 96px dial or an 88px recording view.
#
# Sized to the TALLEST block's real need, measured rather than guessed. That
# used to be the joystick (76px box + 3px + a 36px readout = 115); it is now
# the light dial, because its drawn circle was matched to the brush ring's and
# it carries far more margin than the ring does:
#
#   brush ring   83px circle + 2*8   margin  =  99  (particles only)
#   light dial   83px circle + 2*18.5 margin = 120  (halo AND sparks)
#   joystick     76 + 3 + 36                 = 115
#
# The two circles are the SAME 83px on screen even though their boxes differ by
# 21px - the dial simply needs more room around its circle to bloom into. That
# asymmetry is the reason this number is 122 and not 115.
# 128 = 16 x SPACE_2, the first grid step that clears the tallest block.
#
# MEASURED at the new ramp, not carried over: the joystick column is 76px box
# + 3px + a 42px BODY readout = 121, the light dial is 120 (83px circle plus
# halo and spark clearance), the brush ring 99, the session view 88. 121 is the
# binding one and 128 is the next multiple of 8 above it.
#
# Raising any control past 128 means raising this AND STRIP_HEIGHT with it,
# which costs camera height - so measure before changing either.
CONTENT_H = 16 * theme.SPACE_2

# --- palette -----------------------------------------------------------------
# The top bar's colours, so the two light surfaces are the same light. Keep them
# in step with topbar.py: BAR_LINE, INK, ACCENT and MUTED are shared by eye.
# NOW SHARED IN CODE, not "by eye". The note above asked for these to be kept
# in step with topbar.py by hand, and they had already drifted — LINE, MUTED and
# the four state colours were all a shade off their opposite numbers on the bar.
# Both files read theme.py now, so the two light surfaces cannot disagree again.
FIELD = theme.LIGHT["gray6"]   # the strip itself — Apple's grouped-list ground
CARD = theme.LIGHT["bg3"]      # a card sitting on it
LINE = theme.LIGHT["separator"]
INK = theme.BRAND_INK          # headings and the numbers that get read
TEXT = theme.LIGHT["label"]    # body
MUTED = theme.LIGHT["label2"]
CAP_INK = theme.LIGHT["label2"]  # card titles - see caption(), they sit in a chip
ACCENT = theme.BRAND_ACCENT    # the brand blue - bars, the joystick dot
TRACK = theme.LIGHT["gray5"]   # empty bar / inactive detent

# Apple's light-mode system semantics, same four the top bar's chips use.
LIVE = theme.LIGHT["green"]    # on / running
WARN = theme.LIGHT["orange"]   # paused
BAD = theme.LIGHT["red"]       # fault / error
REC = theme.LIGHT["red"]       # recording

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
# Fills are theme.STATUS_LIGHT's, i.e. the same tint at the same alpha the top
# bar's chips use, so a "recording" pill down here and the REC chip up there are
# finally the same red rather than two reds a few points apart.
TONES = {
    "on":    (LIVE, theme.STATUS_LIGHT["ok"][2]),
    "rec":   (REC, theme.STATUS_LIGHT["bad"][2]),
    "pause": (WARN, theme.STATUS_LIGHT["warn"][2]),
    "info":  (ACCENT, "rgba(63, 111, 181, 0.12)"),
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
# REBUILT AS A REAL SCALE ON 2026-08-24. The four sizes had collapsed onto
# each other - PT_HEAD, PT_VALUE and PT_PILL were ALL 12pt - so a heading, the
# number under it and the control beside it were typographically identical and
# the strip read as one flat sheet of text. Every step below is now a clear
# jump, and the jumps go in the direction the eye expects:
#
#   PT_CAP  10  fine print. Status lines, "idle - 21.6 GB free". Recedes.
#   PT_HEAD 11  the card titles. SMALL ON PURPOSE - they sit in a tinted accent
#               chip (see caption()), and the chip is what marks them as
#               titles. A label does not have to be big to be found; it has to
#               be found FIRST, which shape does better than size. Making these
#               the biggest text was the earlier mistake - it put the label
#               above the reading in the visual order, which is backwards.
#   PT_VALUE 12 the readings. Still bigger than the 11pt title above them, so
#               the hierarchy holds, but 14 made "THROTTLE 0%" a wide slab
#               under a 76px box and the dial's number crowd its own ring.
#               Dropping the whole step keeps every reading in the strip at one
#               size rather than special-casing the two that were too big.
#   PT_BIG   18 the one number per card worth reading across the room: the
#               light dial's percentage, the recording clock. Was 20 - at that
#               size the dial's number crowded its own ring and pulled harder
#               than the heading above it, which inverted the hierarchy the
#               rest of this scale exists to establish.
#
# PT_PILL folds into PT_VALUE - a control and a reading are peers, and the
# earlier note about every pill sharing one size still holds.
# FROM theme's RAMP, IN PIXELS. Mapped to land within a pixel of what the
# strip already rendered, so this is a change of SYSTEM, not of size - the
# hierarchy that was tuned against the real screen is preserved exactly:
#
#   PT_CAP    10pt = 13.3px -> FOOTNOTE 13   fine print, status lines
#   PT_HEAD   11pt = 14.7px -> SUBHEAD  15   card titles, in their chip
#   PT_VALUE  12pt = 16.0px -> BODY     17   the readings
#   PT_BIG    18pt = 24.0px -> TITLE2   22   the one number per card
#
# The names stay PT_* on purpose even though they are pixels now: every call
# site in this 1700-line module refers to them, and renaming them would bury a
# unit change inside a diff that touched every line. The values are the ramp;
# the names are just where the ramp is spelled locally.
PT_CAP = theme.FOOTNOTE
PT_HEAD = theme.SUBHEAD
PT_VALUE = theme.BODY
PT_BIG = theme.TITLE2
PT_PILL = PT_VALUE

# --- the recording card's own scale ------------------------------------------
# One rung down the ramp from the rest of the strip, on the operator's ask to
# shrink this card's contents.
#
# ITS OWN CONSTANTS RATHER THAN THE SHARED PT_*, because only this card moves -
# the joystick, light and brush blocks stay where they are, and reusing the
# shared names would have dragged them along. Named so the three stay in step:
# every size in SessionView comes from here, INCLUDING the recolour() calls,
# which restyle a label at runtime and silently resize it if they disagree with
# the size it was built at.
REC_HEAD_PX = theme.BODY          # state word + clock   (was TITLE2 22)
REC_PILL_PX = theme.SUBHEAD       # the three buttons    (was BODY 17)
REC_DETAIL_PX = theme.CAPTION1    # the footnote line    (was FOOTNOTE 13)

# U+2007 FIGURE SPACE: a space defined to be exactly as wide as a digit, which
# is what makes it the right pad for a number that has to hold still. MEASURED
# in this font at PT_VALUE: every digit 0-9 advances 11px and so does this,
# while an ordinary space is 6px and would not line anything up.
#
# Used to pad the throttle reading to a constant three digits. Pinning the
# LABEL's width (see joy_text) stopped the block from shifting, but the text
# inside it is centred, so "THROTTLE 0%" and "THROTTLE 100%" still re-centred
# against each other and the word slid every time the number gained a digit.
# Padding makes every reading the same width, so the word never moves and only
# the digits change - which is the whole ask.
FIGURE_SPACE = "\u2007"


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


def label(text, px, colour, bold=False, extra="", numeric=False):
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
    weight = theme.W_SEMIBOLD if bold else theme.W_REGULAR
    # numeric=True picks the uniform-figure face; see font(). The family travels
    # through setFont() rather than the style sheet below, which is why this
    # works despite the size having to go the other way.
    lb.setFont(font(px, weight, numeric))
    # px, not pt - see font(). The unit here has to match the one setFont() used
    # above or the style sheet silently wins and resizes the text.
    lb.setStyleSheet(
        f"color:{colour}; background:transparent; font-size:{px}px;"
        f"font-weight:{weight}; {extra}")
    return lb


def recolour(lb, colour, px, bold=False, extra=""):
    """Restyle a label at runtime without losing its size.

    Assigning a style sheet replaces the whole rule, so a call that set only the
    colour would silently drop the font size back to the app-wide 12px.

    PIXELS, matching label() - see font(). This one is easy to miss because it
    only fires on a state change, so a unit mismatch here would look like "the
    text resizes when the recorder starts" rather than like a wrong constant.
    """
    weight = theme.W_SEMIBOLD if bold else theme.W_REGULAR
    lb.setStyleSheet(
        f"color:{colour}; background:transparent; font-size:{px}px;"
        f"font-weight:{weight}; {extra}")
    return lb


def caption(text):
    """Every card's title, and they must all be identical.

    CAP_INK rather than MUTED: at #8590ab on a white card these washed out at
    arm's length, and a title you cannot read is not labelling anything.

    SET IN A TINTED CHIP, 2026-08-24. Size alone was carrying the whole
    distinction between a title and the value under it, and at 12pt over 10pt
    that is a two-point difference an operator reads at arm's length in a lit
    room - not enough. The chip gives the title a shape, so the eye finds the
    label before it reads any of them.
    
    A PILL RATHER THAN AN INVENTED TREATMENT: the top bar's status chips, the
    brush OFF control and the brush-height track are all rounded fills already,
    so this is the vocabulary the panel speaks. wash(ACCENT, 0.88) is the same
    tint-toward-white the state fills use, and the text goes to ACCENT so the
    chip reads as one brand-coloured object instead of grey text in a blue box.

    The radius is derived from label()'s fixed height rather than typed, so a
    change to PT_HEAD keeps the ends round instead of quietly going square.
    """
    # THE BORDER IS NOT DECORATION, it is what makes the corners round: Qt only
    # applies the rounded box model to a QLabel once a border property is set.
    # background + border-radius alone renders a SQUARE fill, which is how the
    # first version of this chip shipped and immediately looked like a bug.
    #
    # The height is label()'s pinned one, halved, so the ends stay semicircular
    # if PT_HEAD ever moves instead of quietly squaring off.
    # EXPLICIT setFixedHeight, not sizeHint() - the same pattern Pill already
    # uses below, and for the same reason: Pill computes its radius from
    # self.height() AFTER pinning it, so the radius is always exactly half the
    # widget's REAL, already-final height with nothing left for layout timing
    # to get out of step with. The first version of this chip computed the
    # radius from a hand-typed height guess while leaving the actual widget on
    # sizeHint() - the two matched in every isolated test but rendered as a
    # plain square in the live app, which a fixed height removes as a variable
    # entirely rather than explains.
    height = PT_HEAD * 2 + 8
    lb = label(text, PT_HEAD, ACCENT, bold=True,
               extra=("letter-spacing:1px;"
                      f" background:{wash(ACCENT, 0.88).name()};"
                      f" border:1px solid {wash(ACCENT, 0.72).name()};"
                      f" border-radius:{theme.capsule(height)}px; padding:0 14px;"))
    lb.setFixedHeight(height)
    return lb


# USB daemon states that mean "a stick is in and being worked on - do not
# pull it". Kept as a set so adding a phase to usb_backup.py does not need
# a matching elif here to stay visible.
_USB_BUSY = ("detected", "mounting", "scanning", "copying", "finishing",
             "clearing")


def _queued(fv):
    """'  +2 sessions' when saves have stacked up behind the running build."""
    n = fv.get("queued") or 0
    return f"  +{n} session{'s' if n != 1 else ''}" if n else ""


class SparkleRing(QWidget):
    """The brush control: a 2cm ring of particles that turns while the brush is.

    REPLACES the small ON/OFF pill, on the operator's call 2026-08-24. The pill
    said what was COMMANDED; this says what is HAPPENING, and on a control whose
    failure mode is mechanical - a stalled motor, a stuck relay - those are not
    the same fact. Motion is the readout: the ring turns and its particles
    twinkle while the brush runs, and the instant it stops the whole thing
    freezes mid-rotation.

    2cm IS LITERAL. This monitor publishes its physical size over EDID
    (344x195mm across 1920x1080 = 5.56 px/mm), so 20mm is 111px and SIZE says
    111 rather than a number that merely looked about right. On a panel that
    does not report a size, or a different one that does, 2cm is a different
    pixel count and this constant is what to recompute.

    NOT THE STOCK CLIP IT WAS MODELLED ON. The reference was a licensed
    Shutterstock particle-ring video; this is an original QPainter drawing of
    the same idea. A video would also have meant decoding a loop forever on a Pi
    that is already running two H.264 camera streams, to animate one indicator.

    FROZEN, NOT HIDDEN, WHEN OFF. stop() leaves the ring exactly where it
    stopped and repaints it in the muted palette - it does not reset the angle
    or blank the widget. A ring that vanishes reads as a broken display; a ring
    visibly stopped mid-turn reads as a brush that is off, which is the whole
    point of using motion as the signal.
    """

    # 1.5cm EXACTLY. This panel publishes its physical size over EDID
    # (344x195mm across 1920x1080 = 5.56 px/mm), so 15mm is 83.4px -> 83.
    #
    # Was a literal 2cm (111px) briefly, which towered over the dial and the
    # recording view and made the tools card the loudest thing on a panel whose
    # point is the cameras. Came down to 84 by eye, then to this when 1.5cm was
    # asked for - 84 had already landed within a pixel of it.
    #
    # The ring is the CIRCLE, not the widget: particles swell outside the path,
    # so the box is the circle plus that margin. Sizing the BOX instead draws a
    # circle 2*MARGIN smaller than intended, which is a mistake already made
    # once here.
    RING_PX = 83
    MARGIN = 8                  # room for a particle at full size
    SIZE = RING_PX + 2 * MARGIN
    N_PARTICLES = 28
    STEP_DEG = 2.4              # a full turn in ~4s at 40ms - see INTERVAL_MS
    INTERVAL_MS = 40            # 25fps: smooth, and cheap next to the decoders

    def __init__(self, tone="on"):
        super().__init__()
        self.setFixedSize(self.SIZE, self.SIZE)
        self._tone = tone
        self._on = None
        self._text = "—"
        self._angle = 0.0
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    # -- state in ------------------------------------------------------------

    def set_text(self, text):
        """Same name and shape as Pill.set_text, so set_state() is unchanged."""
        if text != self._text:
            self._text = text
            self.update()

    def set_value(self, on):
        """None = no reading, True = running, False = stopped."""
        if on == self._on:
            return
        self._on = on
        if on:
            if not self._timer.isActive():
                self._timer.start(self.INTERVAL_MS)
        else:
            self._timer.stop()
        self.update()

    def _tick(self):
        self._angle = (self._angle + self.STEP_DEG) % 360.0
        # Twinkle runs on its OWN counter at a rate that does not divide into
        # the rotation, so particles do not flash in step with their position
        # and turn the ring into a strobing wheel of fixed bright spots.
        self._phase += 0.37
        self.update()

    # -- painting ------------------------------------------------------------

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx = cy = self.SIZE / 2.0
        running = bool(self._on)

        if self._on is None:
            base = QColor(MUTED)
        elif running:
            base = QColor(TONES[self._tone][0])
        else:
            base = QColor(MUTED)

        # The faint track the particles ride on. Always drawn, running or not,
        # so the control keeps its shape and its footprint when it is off -
        # nothing on this strip should change size with its state.
        radius = self.RING_PX / 2.0
        track = QPen(QColor(wash(base.name(), 0.82 if running else 0.88)))
        track.setWidthF(2.0)
        p.setPen(track)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QRectF(cx - radius, cy - radius, radius * 2, radius * 2))

        # The particles. Size and opacity both ride a sine so each one swells as
        # it brightens - a dot that only changed alpha read as flicker rather
        # than a spark.
        p.setPen(Qt.NoPen)
        for i in range(self.N_PARTICLES):
            frac = i / float(self.N_PARTICLES)
            ang = math.radians(self._angle + frac * 360.0)
            tw = 0.5 + 0.5 * math.sin(self._phase + frac * math.tau * 3.0)
            if not running:
                # Held still AND flattened: a frozen ring that still had bright
                # and dim spots looked like a paused animation waiting to
                # resume. Uniform reads as genuinely stopped.
                #
                # 0.45 rather than 0.25 as of 2026-08-24: at a quarter the dots
                # were so small and faint that the resting ring read as a
                # loading placeholder rather than a control at rest. Settled,
                # not switched off - it still sits well below the lit ring.
                tw = 0.45
            x = cx + radius * math.cos(ang)
            y = cy + radius * math.sin(ang)
            r = 1.4 + 2.6 * tw
            col = QColor(base)
            col.setAlphaF(0.20 + 0.80 * tw)
            p.setBrush(col)
            p.drawEllipse(QRectF(x - r, y - r, r * 2, r * 2))

        # The word sits INSIDE the ring rather than under it: the ring is the
        # control, and a caption below would have made the pair read as two
        # separate things again - which is what replacing the pill was undoing.
        p.setPen(QColor(base if running else MUTED))
        p.setFont(font(PT_VALUE))
        p.drawText(self.rect(), Qt.AlignCenter, self._text)


class Pill(QLabel):
    """One control's state. Tinted in its own colour when on, outlined when off.

    The colour is per-pill rather than always green because these are not
    interchangeable: START / STOP lighting up in the same green as the brush
    would make the one control you must never misread the one that blends in.
    """

    def __init__(self, name, tone="on", alternates=(), px=PT_PILL,
                 primary=False):
        super().__init__(name)
        self._name = name
        self._tone = tone
        self._pt = px
        self._primary = primary
        # Every other word this pill can show, so the width floor covers them
        # all and the label never resizes as its state changes.
        self._alternates = tuple(alternates)
        self._value = None
        # A breathing halo behind this pill while it is the live control, drawn
        # by the parent - see SessionView.paintEvent. Only the phase lives here,
        # because the pill cannot paint outside its own rectangle and the halo
        # has to bloom past its edge.
        self._halo = 0.0
        self._pad = 12
        self.setAlignment(Qt.AlignCenter)
        self.setFont(font(px))
        # Height from the size, as before - the ramp is px now so this is a
        # real pixel height rather than a points-to-pixels accident.
        self.setFixedHeight(px * 2 + 8)
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
        # PRIMARY EMPHASIS APPLIES TO EVERY RESTING STATE, not just the unknown
        # one. First version only tinted `on is None`, which is the no-recorder
        # case - so it showed in an isolated preview and NOT in the running app,
        # where a live SessionManager hands these pills a real False. The two
        # resting states look identical to an operator, so they have to look
        # identical here too.
        #
        # `on` True is left alone deliberately: an active control takes its own
        # tone from TONES, and overriding that would hide which one is running.
        if self._primary and not on:
            fg, bg, border = TONES["info"][0], TONES["info"][1], TONES["info"][0]
        # RADIUS 6, NOT A FULL PILL. The card captions are fully-rounded
        # tinted chips (see caption()), and while these were too the operator
        # could not tell a label from a control - "heading and content all are
        # same". A button is the thing you press, so it gets the squarer,
        # heavier shape and the labels keep the soft pill. Shape carries the
        # distinction; both stay the same size and colour family.
        #
        # The 2px border does the rest: a caption chip is a 1px hairline, so at
        # a glance the row of buttons reads as raised and the captions as
        # printed on the card.
        self.setStyleSheet(
            f"color:{fg}; background:{bg}; border:2px solid {border};"
            f"border-radius:{theme.RADIUS_SM}px; padding:0 {self._pad}px;"
            f"font-size:{self._pt}px; font-weight:{theme.W_SEMIBOLD};")


class JoystickView(QWidget):
    """Crosshair box with a dot at the stick position.

    A 2D dot is the whole point: two separate bars make you reconstruct the
    diagonal in your head, which is the one thing an operator reads at a glance.

    84px is what the 150px strip can actually give: the card spends ~17px on its
    caption and ~17px on the PT_CAP readout under the box, leaving ~85. Asking
    for more does not make it bigger, it makes Qt squeeze the readout out.

    NOW DERIVED, NOT CHOSEN. This block is the tallest in the strip - it is the
    only one that is a control AND a readout - so it is what CONTENT_H was
    sized around: 76 + 3px spacing + a 36px PT_VALUE readout = 115 exactly.
    Change this and CONTENT_H has to move with it or the readout is squeezed.
    """

    SIDE = 76

    def __init__(self):
        super().__init__()
        self._x = self._y = None
        self.setFixedSize(self.SIDE, self.SIDE)
        # A halo behind the dot that breathes while there is demand, 2026-08-24.
        # The other two controls in this strip already say "I am doing
        # something" by moving - the light dial's glow and the brush ring's
        # rotation - and the joystick was the one live control that sat
        # perfectly still while commanding the wheels.
        #
        # SCALED BY DEMAND, like the dial's glow, so it is a readout and not a
        # decoration: nothing at centre, a faint pulse just past the deadband,
        # full swing at the stop. It is the same number THROTTLE prints, in a
        # form that can be read without looking straight at it.
        #
        # 90ms matches PotBar's glow exactly. Two breathing things on one strip
        # at different rates read as a fault, not as two indicators.
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def _demand(self):
        """|x|+|y| clamped - the same peak-wheel figure THROTTLE shows."""
        if self._x is None or self._y is None:
            return 0.0
        return max(0.0, min(1.0, abs(self._x) + abs(self._y)))

    def _tick(self):
        self._phase += 0.13
        self.update()

    def _sync_timer(self):
        want = self._demand() > 0.0
        if want and not self._timer.isActive():
            self._timer.start(90)
        elif not want and self._timer.isActive():
            self._timer.stop()

    def set_pos(self, x, y):
        if (x, y) != (self._x, self._y):
            self._x, self._y = x, y
            self._sync_timer()
            self.update()

    # Direction arrows, one per edge. Small on purpose: the box is 84px and the
    # dot is the primary readout - these are a confirmation, not a second thing
    # to read. INACTIVE they are drawn in the same wash as the dashed guides so
    # they read as frame furniture; ACTIVE they go solid ACCENT like the dot.
    # Sized on the operator's call 2026-08-19 to double the lit area of
    # the first version (3.5/5.0). MEASURED by sweeping both constants and
    # counting solid-accent pixels along the lit edge: the count is coarse
    # because antialiased edges do not match the exact colour, so it steps
    # 8 -> 10 -> 12 -> 14 rather than varying smoothly. 4.0/5.2 is the
    # LARGEST pair that lands on 10, so the arrow is as visible as that
    # count allows. Raising LEN past ~5.5 makes a fully deflected dot
    # collide with the arrow it points at - see the dot inset below.
    ARROW_HALF = 4.0     # half-width of the triangle base
    ARROW_LEN = 5.2      # tip to base
    ARROW_INSET = 2.5    # gap between the tip and the box edge

    def _arrow(self, p, tipx, tipy, ddx, ddy, active):
        """One filled triangle with its tip at (tipx, tipy), pointing (ddx, ddy).

        (ddx, ddy) is a unit vector; (-ddy, ddx) is its perpendicular, which is
        what spreads the base. Doing it vectorially rather than with four
        hard-coded triangles means the four calls below differ only by the
        direction they are handed, so none of them can drift out of shape.
        """
        bx, by = tipx - ddx * self.ARROW_LEN, tipy - ddy * self.ARROW_LEN
        px, py = -ddy, ddx
        poly = QPolygonF([
            QPointF(tipx, tipy),
            QPointF(bx + px * self.ARROW_HALF, by + py * self.ARROW_HALF),
            QPointF(bx - px * self.ARROW_HALF, by - py * self.ARROW_HALF),
        ])
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(ACCENT) if active else wash(ACCENT, 0.72))
        p.drawPolygon(poly)

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

        # PER AXIS, not one flag: the 2026-08-22 Y-wiper fault (A1 pinned at the
        # 3.3 V rail) killed the y channel for a whole session while x kept
        # reading perfectly, and the old both-or-nothing test blanked the entire
        # widget - the operator read "no ADC" over a bus that was 60 Hz healthy.
        # A dead axis draws as centred and is NAMED below; the live one still
        # moves its dot and lights its arrows.
        live_x = self._x is not None
        live_y = self._y is not None
        live = live_x and live_y

        # ox / oy ARE THE SINGLE SOURCE OF TRUTH for this widget: the dot is
        # placed with them AND the arrows are lit from them, so an arrow can
        # never point somewhere the dot is not. If a rewire mirrors the display
        # again, flip a sign HERE and both follow together.
        #
        # The one rule they encode: THE DOT MUST FOLLOW THE HAND. Push the stick
        # up and the dot goes up; push it left and the dot goes left. Which sign
        # that takes depends on the stick wiring AND on INVERT_X / INVERT_Y in
        # inputs.py, because what arrives here is the ALREADY-inverted value, so
        # both are empirical and both have now been flipped once:
        #
        #   Y flipped 2026-08-18, after the harness rework settled INVERT_Y=1
        #   X flipped 2026-08-19, when INVERT_X went to 1 to correct the steering
        #   X flipped BACK 2026-08-24, operator: "left right interchange in
        #     frontend" - the dot and arrows were mirrored against the hand
        #
        # DISPLAY ONLY. Nothing in this widget reaches the motors - the drive
        # path reads inputs.py, not the panel - so a wrong sign here is a lying
        # dot, never a robot that drives the wrong way.
        # NEGATED AGAIN 2026-08-24, and ONLY to cancel INVERT_X going back to
        # 0 in inputs.py on the same change. The dot was already following the
        # hand correctly; flipping the motors would have flipped it too, so this
        # sign moves with it and the display is left exactly as it was.
        #
        # That is the whole rule for this line: it is not an opinion about
        # orientation, it is whatever keeps THE DOT FOLLOWING THE HAND given
        # INVERT_X upstream. Change one, check the other.
        ox = -self._x if live_x else 0.0
        oy = self._y if live_y else 0.0

        # Lit on ANY departure from centre rather than at some display
        # threshold: inputs.py has already applied AXIS_DEADBAND and rescaled,
        # so a non-zero value here is exactly a value the motors are acting on.
        # An arrow therefore lights at the same instant the wheels are commanded,
        # which is what makes it worth looking at. eps only guards float dust.
        eps = 1e-6
        top, bot = box.top() + self.ARROW_INSET, box.bottom() - self.ARROW_INSET
        lft, rgt = box.left() + self.ARROW_INSET, box.right() - self.ARROW_INSET
        self._arrow(p, cx, top, 0.0, -1.0, live_y and oy < -eps)  # forward
        self._arrow(p, cx, bot, 0.0, 1.0, live_y and oy > eps)    # reverse
        self._arrow(p, lft, cy, -1.0, 0.0, live_x and ox < -eps)  # left
        self._arrow(p, rgt, cy, 1.0, 0.0, live_x and ox > eps)    # right

        if not (live_x or live_y):
            p.setPen(QColor(MUTED))
            p.setFont(font(theme.CAPTION2, theme.W_REGULAR))
            p.drawText(box, Qt.AlignCenter, "no ADC")
            return

        if not live:
            # One axis dead: name WHICH one, tucked at the bottom edge so the
            # live axis keeps the field for its dot.
            p.setPen(QColor(MUTED))
            p.setFont(font(theme.CAPTION2, theme.W_REGULAR))
            dead = "y" if live_x else "x"
            p.drawText(QRectF(box.left(), box.bottom() - 13, box.width(), 12),
                       Qt.AlignCenter, f"no ADC · {dead}")

        r = 5.0
        # 26, not the old 14: the dot's travel is pulled in so a fully deflected
        # dot stops just short of the arrow it is pointing at instead of sitting
        # on top of it. Widen this and they collide at full stick.
        half = (box.width() - 28) / 2.0
        dx, dy = cx + ox * half, cy + oy * half

        # The breathing halo, UNDER the dot so the dot itself stays a hard,
        # precisely-placed mark - the position is the primary reading here and
        # must not go soft. Radius and alpha both ride the demand, so at centre
        # this draws nothing at all and the box looks exactly as it always did.
        # See __init__ for why this exists and why 90ms.
        demand = self._demand()
        if demand > 0.0:
            swell = 0.5 + 0.5 * math.sin(self._phase)
            glow = QColor(ACCENT)
            glow.setAlphaF(0.10 + 0.28 * swell * demand)
            gr = r + 3.0 + (5.0 + 4.0 * swell) * demand
            p.setPen(Qt.NoPen)
            p.setBrush(glow)
            p.drawEllipse(QRectF(dx - gr, dy - gr, gr * 2, gr * 2))

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
    # 96 with a 9px ring. Was 104/10 while the strip was 200 tall; both came
    # down a step when the strip did, so the dial reads as a peer of the other
    # controls rather than the biggest object in its card.
    #
    # HALO_ROOM is not padding for looks: the breathing glow is drawn as a
    # FATTER pen on the same arc path, up to THICK+9 wide, so without clearance
    # outside the path Qt clips the swell flat against the widget edge and the
    # glow renders as a rectangle.
    # 120 so the DRAWN ARC is 83px across - exactly the brush ring's circle, on
    # the operator's ask that the two read as the same size. The boxes are not
    # the same (99 vs 120) and must not be: this one reserves HALO_ROOM and
    # SPARK_ROOM outside its circle where the ring only reserves particle room.
    # Matching the BOXES would leave the visible circles 21px apart, which is
    # the opposite of what was asked for.
    SIDE = 120
    THICK = 9.0
    HALO_ROOM = 6.0        # clearance for the glow; see paintEvent
    # Clearance for the sparks OUTSIDE the glow. They fly further out than the
    # halo swells, so they need their own allowance on top of it - without this
    # the outermost ones clip flat against the widget edge, which looks like a
    # rendering fault rather than a spark. 112 = the same ~37px arc radius as
    # before plus this room; it still fits CONTENT_H (115) with air to spare.
    SPARK_ROOM = 7.0
    N_SPARKS_MAX = 26      # at 100%; scaled down by the reading - see paintEvent
    START_ANGLE = 225      # bottom-left, in Qt's degrees-from-3-o'clock
    SWEEP = -270           # negative = clockwise

    def __init__(self):
        super().__init__()
        self.setFixedSize(self.SIDE, self.SIDE)
        self._pct = None
        # A slow breathing glow behind the lit arc, 2026-08-24. The dial mirrors
        # a LAMP, and a lamp that is on is not a still object - this is the one
        # control on the strip where an animation is describing the thing rather
        # than decorating it.
        #
        # AMPLITUDE RIDES THE READING, so the animation carries information
        # instead of just moving: at 0% there is nothing to glow and the timer
        # does not even run, at 100% the halo is at full swing. An operator can
        # see roughly how bright the lamp is from across the cab without reading
        # the number.
        #
        # 90ms, not the ring's 40ms: this is a slow swell, and a breathing
        # effect wants FEWER frames than a rotation - running it at 25fps would
        # cost the Pi the same as the ring for an effect that changes far more
        # slowly. ~1.7s a cycle at 0.13 rad a tick.
        self._phase = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def _tick(self):
        self._phase += 0.13
        self.update()

    def _sync_timer(self):
        want = bool(self._pct)          # None and 0.0 both mean "nothing lit"
        if want and not self._timer.isActive():
            self._timer.start(90)
        elif not want and self._timer.isActive():
            self._timer.stop()

    def set_pct(self, pct):
        if pct != self._pct:
            self._pct = pct
            self._sync_timer()
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        m = self.THICK / 2.0 + 1.0 + self.HALO_ROOM + self.SPARK_ROOM
        ring = QRectF(m, m, self.width() - 2 * m, self.height() - 2 * m)

        # Qt's arc angles are in sixteenths of a degree.
        p.setPen(QPen(QColor(TRACK), self.THICK, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(ring, self.START_ANGLE * 16, self.SWEEP * 16)

        pct = self._pct
        if pct:
            frac = max(0.0, min(100.0, pct)) / 100.0
            span = int(self.SWEEP * 16 * frac)

            # The halo: the same arc drawn underneath, fatter and translucent.
            # Breathing between roughly half and full strength so it never
            # disappears entirely (a glow that blinks out reads as a fault), and
            # scaled by frac so a dim lamp gets a faint halo and a bright one a
            # strong one.
            swell = 0.5 + 0.5 * math.sin(self._phase)
            halo = QColor(ACCENT)
            halo.setAlphaF(0.10 + 0.30 * swell * frac)
            p.setPen(QPen(halo, self.THICK + 4.0 + 5.0 * swell * frac,
                          Qt.SolidLine, Qt.RoundCap))
            p.drawArc(ring, self.START_ANGLE * 16, span)

            p.setPen(QPen(QColor(ACCENT), self.THICK, Qt.SolidLine, Qt.RoundCap))
            p.drawArc(ring, self.START_ANGLE * 16, span)

            # SPARKS THROWN OFF THE LIT ARC. The dial mirrors a lamp, and this
            # is the part that says how hard the lamp is working: both the
            # NUMBER of sparks and how far they fly scale with the reading, so
            # a dial at 15% throws a few short ones and a dial at 100% throws a
            # full crown of them. That makes the animation carry the same
            # information as the number in the middle, readable from further
            # away than the digits are.
            #
            # They are placed only along the arc that is actually LIT - a spark
            # sitting out past the unlit remainder of the track would say the
            # lamp is brighter than it is, which is the one thing this must not
            # do.
            #
            # Qt's arc angles run counter-clockwise from 3 o'clock in degrees,
            # while screen Y grows downward, hence cos(+a) but -sin(+a) below.
            cx, cy = self.width() / 2.0, self.height() / 2.0
            arc_r = ring.width() / 2.0
            n = max(2, int(self.N_SPARKS_MAX * frac))
            p.setPen(Qt.NoPen)
            for i in range(n):
                t = (i + 0.5) / n
                a = math.radians(self.START_ANGLE + (self.SWEEP * frac) * t)
                # Each spark twinkles on its own offset so the crown shimmers
                # instead of pulsing as one block.
                tw = 0.5 + 0.5 * math.sin(self._phase * 1.7 + i * 2.3)
                rr = arc_r + self.THICK / 2.0 + 2.0 + (4.0 + 5.0 * frac) * tw
                r = 0.9 + 1.7 * tw * frac
                col = QColor(ACCENT)
                col.setAlphaF(0.12 + 0.68 * tw * frac)
                p.setBrush(col)
                p.drawEllipse(QRectF(cx + rr * math.cos(a) - r,
                                     cy - rr * math.sin(a) - r,
                                     r * 2, r * 2))

        # PT_VALUE, not PT_BIG: at 18pt the number filled the hole and turned
        # the dial into a number with a ring around it rather than a gauge with
        # a reading in it. Smaller lets the arc and its sparks be the thing the
        # eye lands on, which is the part that is actually animated.
        p.setPen(QColor(INK) if pct is not None else QColor(MUTED))
        p.setFont(font(PT_VALUE))
        p.drawText(self.rect(), Qt.AlignCenter,
                   "—" if pct is None else f"{pct:.0f}%")


class LinearActuator(QWidget):
    """The brush-height actuator, drawn as the linear actuator it actually is.

    REPLACES the 3-detent track, 2026-08-24, on the operator's note that the
    mechanism is a linear actuator. A barrel with a rod out of it says what the
    hardware is at a glance in a way three dots on a line never did, and it
    gives the control somewhere to put motion - which is what the rest of this
    strip now uses to mean "this is doing something right now".

    THE ROD TRAVELS WITH THE COMMAND: NEUTRAL parks it mid-stroke, EXTEND drives
    it right, RETRACT drives it left, easing between the three. Operator's ask,
    2026-08-24, replacing a fixed-length rod that only flowed chevrons.

    WHAT THIS DRAWS IS THE COMMAND, NOT A MEASUREMENT, and that has to be said
    plainly because the drawing now looks like a measurement. Nothing in this
    system knows where the rod physically is: inputs.py reports which of
    RETRACT/STOP/EXTEND the interlock pins are asserting, and there is no
    encoder, no limit switch and no current sense anywhere on the path. So a
    rod shown fully extended means "EXTEND has been held long enough for this
    animation to finish", NOT "the rod is at its stop" - if the actuator were
    jammed or unpowered this would look exactly the same.

    Read the rod for direction and the chevrons for "being driven right now".
    If a position sensor ever appears, drive _ext from it and this becomes the
    honest readout it currently only resembles.

    Chevrons still freeze on STOP, for the reason the brush ring freezes - a
    stopped animation is a stronger "not moving" than any colour change.
    """

    # (state string from inputs.py, colour when active)
    STAGES = (("RETRACT", ACCENT), ("STOP", TEXT), ("EXTEND", LIVE))
    LABELS = {"RETRACT": "RETRACT", "STOP": "NEUTRAL", "EXTEND": "EXTEND"}

    WIDTH = 120
    BAND_H = 30                # barrel + rod
    HEIGHT = BAND_H + 30       # plus the one-word state under it
    BARREL_W = 46
    BARREL_H = 22
    ROD_H = 8
    STEP = 0.055               # chevron travel per tick, as a fraction of pitch
    INTERVAL_MS = 60

    # Where each command parks the rod, as a fraction of the stroke. STOP is
    # 0.5 rather than 0.0 because NEUTRAL on this rig is a mid-stroke hold, not
    # a retracted one - the brush sits at working height there.
    STAGE_POS = {"RETRACT": 0.0, "STOP": 0.5, "EXTEND": 1.0}
    ROD_MIN = 16.0             # visible rod at fully retracted
    ROD_MAX = 52.0             # ... and at fully extended
    # Fraction of the remaining distance covered per tick. 0.16 at 60ms crosses
    # a full stroke in ~1.2s: slow enough to read as travel rather than a jump,
    # fast enough that a short tap on the switch still visibly moves it.
    EASE = 0.16

    def __init__(self, tone=ACCENT):
        super().__init__()
        self._stage = None
        self._tone = tone
        self._flow = 0.0
        self._ext = 0.5        # current rod position; starts parked at NEUTRAL
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        # Fixed, not Expanding: the drawing is centred in its own width, so an
        # expanding widget would stretch the hit area without moving the
        # drawing and the control would drift off-centre from its caption.
        self.setFixedSize(self.WIDTH, self.HEIGHT)

    def _target(self):
        # An unknown stage (None at startup, or FAULT) parks mid-stroke rather
        # than driving anywhere - with no command asserted there is no direction
        # to claim, and NEUTRAL is the honest place to sit.
        return self.STAGE_POS.get(self._stage, 0.5)

    def _tick(self):
        self._flow = (self._flow + self.STEP) % 1.0
        target = self._target()
        if abs(self._ext - target) > 0.002:
            self._ext += (target - self._ext) * self.EASE
        else:
            self._ext = target
            # Arrived AND nothing is being driven: nothing left to animate.
            if self._stage not in ("RETRACT", "EXTEND"):
                self._timer.stop()
        self.update()

    def _sync_timer(self):
        # Run while a direction is commanded (for the chevrons) OR while the rod
        # still has travel left to do - releasing to NEUTRAL has to animate back
        # to mid-stroke, and that is not a commanded-motion state.
        need = (self._stage in ("RETRACT", "EXTEND")
                or abs(self._ext - self._target()) > 0.002)
        if need and not self._timer.isActive():
            self._timer.start(self.INTERVAL_MS)

    def set_stage(self, stage):
        if stage != self._stage:
            self._stage = stage
            self._sync_timer()
            self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        # FAULT means both interlock pins low, i.e. no stage is truthfully
        # current - nothing lights, and the whole assembly goes red.
        fault = self._stage == "FAULT"
        stage = self._stage
        colour = (BAD if fault
                  else dict(self.STAGES).get(stage, MUTED))
        moving = stage in ("RETRACT", "EXTEND") and not fault
        out = 1.0 if stage == "EXTEND" else -1.0

        cy = self.BAND_H / 2.0
        # Laid out for the LONGEST rod, so the barrel stays put and only the rod
        # moves. Sizing to the current length would slide the whole assembly
        # sideways as it extended, which reads as the actuator sliding rather
        # than its rod coming out.
        total = self.BARREL_W + self.ROD_MAX + 12.0
        x0 = (self.width() - total) / 2.0

        # Barrel.
        barrel = QRectF(x0, cy - self.BARREL_H / 2.0,
                        float(self.BARREL_W), float(self.BARREL_H))
        p.setPen(QPen(QColor(BAD) if fault else wash(self._tone, 0.55), 1))
        p.setBrush(QColor("#fdecea") if fault else wash(self._tone, 0.86))
        p.drawRoundedRect(barrel, float(theme.RADIUS_SM), float(theme.RADIUS_SM))

        # Rod, its length driven by the eased position - see _tick.
        rod_x0 = barrel.right() - 2.0
        rod_x1 = rod_x0 + self.ROD_MIN + (self.ROD_MAX - self.ROD_MIN) * self._ext
        rod = QRectF(rod_x0, cy - self.ROD_H / 2.0, rod_x1 - rod_x0, self.ROD_H)
        p.setPen(Qt.NoPen)
        p.setBrush(wash(colour, 0.72) if not fault else wash(BAD, 0.72))
        p.drawRoundedRect(rod, self.ROD_H / 2.0, self.ROD_H / 2.0)

        # Rod end.
        p.setBrush(QColor(colour))
        p.drawEllipse(QRectF(rod_x1 - 1.0, cy - 6.0, 12.0, 12.0))

        # Chevrons flowing along the rod. Clipped to the rod so one leaving the
        # end does not spill over the barrel or the end cap.
        if moving:
            p.save()
            path_clip = QRectF(rod.left() + 2, rod.top() - 3,
                               rod.width() - 4, rod.height() + 6)
            p.setClipRect(path_clip)
            pitch = 13.0
            pen = QPen(QColor(colour))
            pen.setWidthF(2.0)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            n = int(rod.width() / pitch) + 2
            for i in range(n):
                # out<0 reverses travel so RETRACT flows back toward the barrel.
                cxx = (rod.left() + (i + self._flow * out) * pitch)
                p.drawLine(QPointF(cxx - 3.0 * out, cy - 4.0),
                           QPointF(cxx + 1.0 * out, cy))
                p.drawLine(QPointF(cxx + 1.0 * out, cy),
                           QPointF(cxx - 3.0 * out, cy + 4.0))
            p.restore()

        name = "FAULT" if fault else self.LABELS.get(stage, "—")
        p.setPen(QColor(colour))
        p.setFont(font(PT_VALUE))
        p.drawText(QRectF(0, self.BAND_H, self.width(), self.HEIGHT - self.BAND_H),
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
            f"border-top:3px solid {tone};"
            f"border-radius:{theme.concentric(theme.RADIUS_SM, theme.SPACE_1)}px; }}")
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

    TWO ROWS, AND EVERY BLOCK USES THE SAME TWO. The caption is row one. Row
    two is a box of exactly CONTENT_H with the control centred inside it. That
    fixed box is the mechanism: it means a 76px joystick, a 96px dial and an
    88px recording view all occupy an identical band, so the headings line up
    AND the controls line up, instead of each block hanging from its caption at
    whatever height its contents happened to need.

    Two earlier attempts at this failed and are worth not repeating: top-
    aligning everything lined the tops up but left the brush card half empty,
    and centring each control in its own full-height card lined nothing up at
    all. Fixing the BAND rather than the alignment is what actually works.
    """
    col = QVBoxLayout()
    col.setSpacing(4)
    col.setContentsMargins(0, 0, 0, 0)
    col.addWidget(caption(cap), 0, Qt.AlignHCenter)

    # main.py paints EVERY QWidget in the app's near-black, and this container
    # is a QWidget - without the transparent background each block would sit in
    # its own dark box on the white card. label() carries the same warning.
    body = QWidget()
    body.setStyleSheet("background:transparent;")
    body.setFixedHeight(CONTENT_H)
    inner = QVBoxLayout(body)
    inner.setContentsMargins(0, 0, 0, 0)
    inner.setSpacing(spacing)
    inner.addStretch(1)
    for w in widgets:
        if isinstance(w, QWidget):
            # fill=True for content that genuinely wants the whole card width -
            # the recording card's session view. Centring that would pin it to
            # its sizeHint and leave the card half empty.
            inner.addWidget(w) if fill else inner.addWidget(w, 0, Qt.AlignHCenter)
        else:
            inner.addLayout(w)
    inner.addStretch(1)

    col.addWidget(body) if fill else col.addWidget(body, 0, Qt.AlignHCenter)
    col.addStretch(1)
    return col


class TapeBar(QWidget):
    """A slim strip under the recording state that MOVES while tape is running.

    Added 2026-08-24. The recording card was the last block on the strip with
    no motion in it: the joystick breathes, the light glows, the brush spins and
    the actuator flows, and the one control that is actually capturing something
    sat perfectly still whether it was recording or not.

    It is the same idea as the brush ring, in the shape this card had room for -
    stripes travelling right while RECORDING, FROZEN while PAUSED, and an empty
    track when STOPPED. Paused freezing rather than blanking matters here more
    than anywhere else on the panel: paused is the state most easily mistaken
    for recording, and a bar of stripes sitting still is a much louder "you are
    NOT capturing this" than a word that changed.

    Deliberately not a progress bar: a recording has no known length to be a
    fraction of, and drawing something that looked like one would invite the
    question "progress towards what".
    """

    HEIGHT = 7
    MIN_W = 120            # never let the layout collapse it - see __init__
    MAX_W = 250            # see the tape_row comment in SessionView
    PITCH = 14.0
    STEP = 0.05
    INTERVAL_MS = 55

    def __init__(self):
        super().__init__()
        self.setFixedHeight(self.HEIGHT)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        # BOTH BOUNDS, and the minimum is not optional: a bare QWidget has no
        # sizeHint, so between two stretches the layout handed this a width of
        # ZERO and the bar silently vanished - the timer ran, set_mode() was
        # called, and nothing was ever drawn. Expanding only says "grow if you
        # are given room"; it does not ask for any.
        self.setMinimumWidth(self.MIN_W)
        self.setMaximumWidth(self.MAX_W)
        self._mode = "STOPPED"
        self._colour = MUTED
        self._flow = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def _tick(self):
        self._flow = (self._flow + self.STEP) % 1.0
        self.update()

    def set_mode(self, mode, colour):
        """mode: RECORDING (moving), PAUSED (frozen stripes), else empty."""
        if (mode, colour) == (self._mode, self._colour):
            return
        self._mode, self._colour = mode, colour
        if mode == "RECORDING":
            if not self._timer.isActive():
                self._timer.start(self.INTERVAL_MS)
        else:
            self._timer.stop()
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        r = self.HEIGHT / 2.0
        track = QRectF(0, 0, float(self.width()), float(self.HEIGHT))

        # NOTHING AT ALL WHEN STOPPED. An empty grey track under the word
        # STOPPED was a bar that could not fill, sitting where a progress bar
        # would be - it read as a stalled something rather than as the absence
        # of a recording. The row keeps its height either way, so the buttons
        # below do not move when a session starts.
        live = self._mode in ("RECORDING", "PAUSED")
        if not live or self.width() <= 0:
            return
        p.setPen(Qt.NoPen)
        p.setBrush(wash(self._colour, 0.86))
        p.drawRoundedRect(track, r, r)

        # Stripes are clipped to the rounded track so they cannot square off
        # its ends.
        p.save()
        clip = QPainterPath()
        clip.addRoundedRect(track, r, r)
        p.setClipPath(clip)
        pen = QPen(QColor(self._colour))
        pen.setWidthF(4.0)
        pen.setCapStyle(Qt.FlatCap)
        p.setPen(pen)
        n = int(self.width() / self.PITCH) + 3
        for i in range(n):
            x = (i + self._flow) * self.PITCH - self.PITCH
            # Leaning stripes, so direction of travel is unambiguous even at
            # this height - vertical bars would look identical either way.
            p.drawLine(QPointF(x, self.HEIGHT + 1.0),
                       QPointF(x + self.HEIGHT + 1.0, -1.0))
        p.restore()


def qcol(spec, alpha=None):
    """QColor from a theme token, INCLUDING the rgba() strings.

    QColor does not parse CSS rgba(): it returns an invalid colour, which
    paints solid black. Half the theme's tokens - every label tier and the
    separator - are written in that form because they were authored for
    style sheets, so anything that reaches QPainter has to come through here.
    """
    m = re.match(r"rgba?\(([^)]*)\)", str(spec).strip())
    if m:
        parts = [x.strip() for x in m.group(1).split(",")]
        c = QColor(int(float(parts[0])), int(float(parts[1])),
                   int(float(parts[2])))
        c.setAlphaF(float(parts[3]) if len(parts) > 3 else 1.0)
    else:
        c = QColor(spec)
    if alpha is not None:
        c.setAlphaF(alpha)
    return c


class RoundSaveButton(QWidget):
    """SAVE, as a round control that doubles as the transfer dial.

    REPLACES the SAVE pill AND the whole footnote line under the recording
    card, on the operator's call 2026-08-26. Three separate facts used to be
    spread across three places - whether SAVE had been pressed (a pill), how
    far the merge had got (a percentage buried mid-sentence), and whether the
    footage was safe on the card (a line of standing instructions). To an
    operator standing at the strip waiting for a job to finish those are ONE
    question, so they are now one control.

    THE HUD LOOK is the operator's call 2026-08-26, with a reference clip of a
    futuristic transfer ring. It is not decoration for its own sake - each
    layer is carrying something:

        tick gauge   sixty marks, lit up to the current percentage. Reads at a
                     glance from across the room, where a thin arc does not.
        comet arc    the progress itself, brightening toward its head, so the
                     eye is pulled to the leading edge rather than to the
                     middle of a uniform band.
        dashed ring  turns continuously, INDEPENDENT of progress. This is the
                     liveness signal: the percentage can sit on 61% for half a
                     minute on a long clip, and a still dial at 61% looks
                     exactly like a wedged ffmpeg. This one never stops.
        orbit dot    a second, faster, counter-turning mark. Same job, and it
                     keeps moving even when the dashed ring is edge-on to the
                     eye at a glance.

    THE PERCENTAGE MOVED INSIDE THE RING. It was under the button, which is
    where the operator asked for it on 2026-08-26 - but the +0.5cm they asked
    for on the same day does not fit in this card WITH a caption line under it.
    SessionView gets 128px, the heading and tape bar take 34, and a 89px dial
    plus a 12px caption does not come in under the rest. Inside the ring is
    where the reference puts it anyway. Say the word and it goes back under, at
    a smaller ring.
    """

    # SIZES ARE MILLIMETRES, converted once. This panel publishes its physical
    # size over EDID - 344x195mm across 1920x1080 = 5.56 px/mm - which is the
    # same constant SparkleRing sizes itself with. The operator asks for
    # centimetres because that is what a control on a panel actually is.
    OUTER = 89                 # was 61: +0.5cm, as asked
    FACE = 46                  # the solid centre disc
    PAD = 2                    # bloom room. There is no more than this - see
    #                            the class docstring on what the card gives.
    WIDTH = OUTER + 2 * PAD

    RING_W = 5.0               # progress arc thickness
    TICKS = 60                 # marks around the gauge
    FPS_MS = 40                # 25fps. This is a Pi with two encoders running.
    BREATH_S = 2.8             # idle breath period
    SPIN_S = 3.6               # dashed ring revolution
    ORBIT_S = 2.1              # the counter-turning dot
    EASE = 0.18                # per-frame approach to the target percentage

    def __init__(self):
        super().__init__()
        self._mode = "idle"        # idle | busy | done | error
        self._target = 0.0
        self._frac = 0.0
        self._press = 0.0
        self._check = 0.0
        self._tick = 0.0
        self._armed = False
        self.setFixedSize(self.WIDTH, self.WIDTH)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._step)
        self._timer.start(self.FPS_MS)

    # ---- state in ------------------------------------------------------
    def set_status(self, mode, frac=0.0):
        if mode == "done" and self._mode != "done":
            self._check = 0.0
        self._mode = mode
        self._target = max(0.0, min(1.0, float(frac or 0.0)))
        # A fresh build starts the dial at zero rather than easing down from
        # wherever the last one finished, which otherwise reads as progress
        # running backwards.
        if mode == "busy" and self._frac > self._target + 0.25:
            self._frac = self._target

    def flash(self):
        """The panel took a SAVE press."""
        self._press = 1.0

    def set_armed(self, on):
        self._armed = bool(on)

    # ---- animation -----------------------------------------------------
    def _step(self):
        self._tick += self.FPS_MS / 1000.0
        self._frac += (self._target - self._frac) * self.EASE
        self._press *= 0.86
        if self._press < 0.01:
            self._press = 0.0
        if self._mode == "done":
            self._check += (1.0 - self._check) * 0.16
        else:
            self._check = 0.0
        self.update()

    # ---- paint ---------------------------------------------------------
    def _tone(self):
        if self._mode == "error":
            return QColor(BAD)
        if self._mode == "done":
            return QColor(TONES["on"][0])
        if self._mode == "busy":
            return QColor(WARN)
        return QColor(ACCENT)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        c = QPointF(self.width() / 2.0, self.height() / 2.0)
        r_out = self.OUTER / 2.0
        r_face = self.FACE / 2.0
        r_dash = r_out - 17.0

        tone = self._tone()
        breath = 0.5 + 0.5 * math.sin(self._tick * math.tau / self.BREATH_S)
        busy = self._mode == "busy"
        lit = self._mode in ("busy", "done", "error")
        frac = (1.0 if self._mode in ("done", "error")
                else (self._frac if busy else 0.0))

        self._gauge(p, c, r_out, tone, frac, lit, breath)
        if busy:
            self._dashes(p, c, r_dash, tone)
            self._orbit(p, c, r_out - 3.5, tone, breath)
        self._face(p, c, r_face, tone, breath)
        p.end()

    # -- layers ----------------------------------------------------------
    def _gauge(self, p, c, r, tone, frac, lit, breath):
        """Sixty radial marks: the progress, the trail and the head, in one band.

        The coarse readout. An arc tells you roughly how far round something
        is; a count of lit marks is a NUMBER you can read without reading, and
        it is the layer that still works at the far end of a duct. The last
        twelve marks behind the leading edge brighten and grow, which is what
        turns a gauge into a comet without adding a second ring to do it.
        """
        r_in = r - 6.0
        tail = 12.0                      # marks the trail spans
        for i in range(self.TICKS):
            t = (i + 0.5) / float(self.TICKS)
            on = lit and t <= frac
            if on:
                behind = (frac - t) * self.TICKS
                k = max(0.0, 1.0 - behind / tail)
                col = QColor(tone).lighter(int(100 + 22 * k))
                col.setAlphaF(0.60 + 0.40 * k)
                w, ext = 2.2 + 1.3 * k, 1.8 * k
            else:
                col = qcol(MUTED, 0.16)
                w, ext = 1.6, 0.0
            a = math.radians(90.0 - t * 360.0)
            ca, sa = math.cos(a), math.sin(a)
            p.setPen(QPen(col, w, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(c.x() + ca * r_in, c.y() - sa * r_in),
                       QPointF(c.x() + ca * (r + ext), c.y() - sa * (r + ext)))

        # A hairline at the inner edge of the band, so the gauge has a rim to
        # sit on rather than floating as loose strokes.
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(qcol(MUTED, 0.14), 1.0))
        p.drawEllipse(c, r_in - 2.0, r_in - 2.0)

        # The leading edge, with a little bloom. This is the mark the eye
        # tracks, so it is the one thing here allowed to be pure white.
        if not lit or frac <= 0.002 or frac >= 0.999:
            return
        a = math.radians(90.0 - frac * 360.0)
        head = QPointF(c.x() + math.cos(a) * (r - 2.0),
                       c.y() - math.sin(a) * (r - 2.0))
        p.setPen(Qt.NoPen)
        for n in range(3, 0, -1):
            col = QColor(tone)
            col.setAlphaF((0.26 + 0.10 * breath) * (n / 3.0))
            p.setBrush(col)
            p.drawEllipse(head, 2.6 + n * 1.5, 2.6 + n * 1.5)
        p.setBrush(QColor(255, 255, 255, 240))
        p.drawEllipse(head, 2.2, 2.2)

    def _dashes(self, p, c, r, tone):
        """Three arcs turning at a constant rate. Liveness, not progress."""
        rect = QRectF(c.x() - r, c.y() - r, r * 2, r * 2)
        spin = (self._tick / self.SPIN_S) * 360.0
        col = QColor(tone)
        col = QColor(tone).lighter(126)
        col.setAlphaF(0.80)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(col, 2.2, Qt.SolidLine, Qt.RoundCap))
        for k in range(3):
            start = spin + k * 120.0
            p.drawArc(rect, int(round(start * 16)), int(round(46 * 16)))

    def _orbit(self, p, c, r, tone, breath):
        """One mark going the other way, faster. A second heartbeat."""
        a = math.radians(-(self._tick / self.ORBIT_S) * 360.0 + 90.0)
        x, y = c.x() + math.cos(a) * r, c.y() - math.sin(a) * r
        p.setPen(Qt.NoPen)
        col = QColor(tone)
        col.setAlphaF(0.35 + 0.25 * breath)
        p.setBrush(col)
        p.drawEllipse(QPointF(x, y), 3.2, 3.2)
        p.setBrush(QColor(255, 255, 255, 210))
        p.drawEllipse(QPointF(x, y), 1.5, 1.5)

    def _face(self, p, c, r_face, tone, breath):
        # Presses dip it; the arm-blink lifts it. Both are scale, not colour,
        # because the colour is already carrying the mode.
        lift = 1.0 + (0.03 * breath if self._armed else 0.0)
        rf = r_face * (1.0 - 0.055 * self._press) * lift

        if self._mode == "idle":
            top, bot = QColor("#ffffff"), QColor(theme.LIGHT["gray6"])
            edge, ink = qcol(LINE), QColor(theme.LIGHT["label"])
        else:
            top, bot = QColor(tone), QColor(tone).darker(114)
            edge, ink = QColor(tone).darker(112), QColor("#ffffff")

        for i in range(3, 0, -1):
            col = QColor(0, 0, 0)
            col.setAlphaF(0.045 * (i / 3.0))
            p.setPen(Qt.NoPen)
            p.setBrush(col)
            g = rf + i * 0.9
            p.drawEllipse(QPointF(c.x(), c.y() + 1.5), g, g)

        grad = QLinearGradient(c.x(), c.y() - rf, c.x(), c.y() + rf)
        grad.setColorAt(0.0, top)
        grad.setColorAt(1.0, bot)
        p.setBrush(grad)
        p.setPen(QPen(edge, 1.0))
        p.drawEllipse(c, rf, rf)

        spec = QColor("#ffffff")
        spec.setAlphaF(0.55 if self._mode == "idle" else 0.38)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(spec, 1.4))
        p.drawArc(QRectF(c.x() - rf + 1.2, c.y() - rf + 1.2,
                         (rf - 1.2) * 2, (rf - 1.2) * 2), 30 * 16, 120 * 16)

        box = QRectF(c.x() - rf, c.y() - rf, rf * 2, rf * 2)
        if self._mode == "done" and self._check > 0.02:
            self._draw_check(p, c.x(), c.y(), rf, ink)
            return
        if self._mode == "error":
            f = QFont(self.font())
            f.setPixelSize(int(rf * 1.15))
            f.setWeight(QFont.Bold)
            p.setFont(f)
            p.setPen(ink)
            p.drawText(box, Qt.AlignCenter, "!")
            return

        f = QFont(self.font())
        p.setPen(ink)
        if self._mode == "busy":
            # THE READOUT, in the middle of its own dial. Tabular digits so the
            # number does not jitter sideways as it counts.
            f.setPixelSize(17)
            f.setWeight(QFont.Bold)
            p.setFont(f)
            p.drawText(box, Qt.AlignCenter, "%d%%" % round(self._frac * 100))
        else:
            f.setPixelSize(12)
            f.setWeight(QFont.Bold)
            f.setLetterSpacing(QFont.AbsoluteSpacing, 0.9)
            p.setFont(f)
            p.drawText(box, Qt.AlignCenter, "SAVE")

    def _draw_check(self, p, cx, cy, rf, ink):
        """A tick that DRAWS ITSELF ON rather than appearing.

        Two strokes, revealed in order along their combined length, so the eye
        follows the stroke and lands on the finished mark. An instant tick on a
        control that has been turning for a minute is easy to miss.
        """
        sz = rf * 0.62
        a = QPointF(cx - sz * 0.92, cy + sz * 0.05)
        b = QPointF(cx - sz * 0.22, cy + sz * 0.72)
        d = QPointF(cx + sz * 0.95, cy - sz * 0.62)
        l1 = math.hypot(b.x() - a.x(), b.y() - a.y())
        l2 = math.hypot(d.x() - b.x(), d.y() - b.y())
        t = min(1.0, self._check) * (l1 + l2)
        path = QPainterPath(a)
        if t <= l1:
            k = t / l1
            path.lineTo(a.x() + (b.x() - a.x()) * k, a.y() + (b.y() - a.y()) * k)
        else:
            path.lineTo(b)
            k = min(1.0, (t - l1) / l2)
            path.lineTo(b.x() + (d.x() - b.x()) * k, b.y() + (d.y() - b.y()) * k)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(ink, max(3.0, rf * 0.17), Qt.SolidLine,
                      Qt.RoundCap, Qt.RoundJoin))
        p.drawPath(path)


class SessionView(QWidget):
    """Recording state, its controls, and a footnote.

    HALO NOTE: the pulse behind the live button is painted HERE, not by the
    Pill, because a widget cannot draw outside its own rectangle and a glow
    that stopped at the button's edge would read as a border rather than a
    bloom. paintEvent() below draws it under the children; Qt paints a parent
    before its children, so nothing has to be re-ordered for the pill to stay
    crisp on top.
    """
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
    SAVE_INSET = 7              # trims the move to a round 3cm
    SAVE_FLASH_S = 0.9

    def __init__(self):
        super().__init__()
        self._state = None
        self._blink_on = True
        # Drives the halo in paintEvent(). Runs ONLY while a session is live -
        # main.py's tick already repaints this view every frame when something
        # is happening, but it does not when the rig is idle, and an idle card
        # must not cost a repaint every 60ms for a glow that is not drawn.
        self._halo_timer = QTimer(self)
        self._halo_timer.timeout.connect(self.update)
        self._presses = None
        self._flash_until = 0.0

        self.head = label("● STOPPED", REC_HEAD_PX, MUTED, bold=True)
        # Counts up once a second — numeric face, or the clock shuffles.
        self.elapsed = label("--:--", REC_HEAD_PX, INK, bold=True, numeric=True)
        # The only text here that changes every frame. Fixed to its widest
        # reading so the layout beside it never twitches - which is the whole
        # thing mono was ever buying.
        self.elapsed.setFixedWidth(
            QFontMetrics(self.elapsed.font()).horizontalAdvance("0:00:00") + 6)

        # One status line, not two. The SAVE confirmation REPLACES it rather
        # than adding a row, because a row costs ~16px of a 124px budget and the
        # two never need to be read at the same moment.
        self.tape = TapeBar()
        # TERTIARY, and it has to LOOK it. At PT_CAP in MUTED it was the same
        # weight and nearly the same colour as the buttons directly above, so
        # the card read as four rows of equal grey rather than a state, its
        # controls, and a footnote. label3 is the HIG's tertiary label - the
        # same hue, dropped in alpha - which quiets it without shifting it.
        self.detail = label("idle", REC_DETAIL_PX, theme.LIGHT["label3"])
        # GONE, operator 2026-08-26. This line carried three things at
        # once - a percentage, a running commentary, and a block of
        # standing instructions ('PLUG IN USB', 'nothing to save') that
        # said the same words on every run and so stopped being read.
        # The percentage moved onto the SAVE button, where the thing it
        # measures already is. The widget stays built and still takes
        # its text so none of the branches below have to be unpicked;
        # it simply never shows, and a hidden widget costs the layout
        # nothing. To bring it back, delete this one line.
        self.detail.setVisible(False)

        # PRIMARY. All three buttons were identical grey at rest, so nothing
        # said which one starts the job - and START / STOP is the only one an
        # operator reaches for without already being mid-session. It carries a
        # tinted resting fill; the other two stay quiet until their state makes
        # them light. This is emphasis at REST only: once recording, each pill
        # still takes its own tone from TONES exactly as before.
        self.rec_pill = Pill("START / STOP", PILL_TONE["START / STOP"],
                             px=REC_PILL_PX, primary=True)
        self.pause_pill = Pill("PAUSE / RESUME", PILL_TONE["PAUSE / RESUME"],
                               px=REC_PILL_PX)
        # NOT a pill. SAVE is the only control here whose result takes
        # time, so it is the only one that needs somewhere to show how
        # much time is left - see RoundSaveButton.
        self.save_btn = RoundSaveButton()

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
        # SAVE MOVED 3cm RIGHT, operator 2026-08-26. Not by wedging a
        # spacer in front of it - that widens the centred group, so the
        # two pills would slide LEFT by half of whatever SAVE moved
        # right. SAVE goes to the right edge and an empty slot of its own
        # width goes on the left, which leaves START / STOP and PAUSE /
        # RESUME centred in the card exactly where they were. 5.56 px/mm:
        # the dial's centre travels 761 - 7 - 47 = 707 from 539, or 30.2mm.
        slot = RoundSaveButton.WIDTH + self.SAVE_INSET
        save_slot = QWidget()
        save_slot.setFixedWidth(slot)
        slot_l = QHBoxLayout(save_slot)
        slot_l.setContentsMargins(0, 0, self.SAVE_INSET, 0)
        slot_l.setSpacing(0)
        slot_l.addWidget(self.save_btn, 0, Qt.AlignVCenter)
        pill_row.addSpacing(slot)
        pill_row.addStretch(1)
        pill_row.addWidget(self.rec_pill)
        pill_row.addWidget(self.pause_pill)
        pill_row.addStretch(1)
        pill_row.addWidget(save_slot)

        # Order: heading, then the three buttons centred in the block, then the
        # detail line pinned to the bottom. A stretch either side of the pill row
        # is what centres it - without the second one the buttons ride up against
        # the heading and the detail line floats in the middle of the block.
        # ON THE GRID, and unevenly on purpose - the gaps encode the grouping.
        # The state word, its tape bar and its buttons are ONE thing and sit
        # close together; the footnote is a separate thing and gets a bigger gap
        # above it. Uniform spacing made all four rows read as a list of equals,
        # which is what "heading and content all mixed" was describing.
        col = QVBoxLayout(self)
        col.setContentsMargins(0, 0, 0, 0)
        # TIGHTER THAN THE GRID, and only here. SessionView gets 128px and
        # the 89px dial plus the heading and tape bar do not fit at
        # SPACE_1. The two gaps this affects are inside one group that the
        # original comment already calls ONE thing, so closing them does
        # not blur a boundary that meant anything.
        col.setSpacing(2)
        col.addLayout(head_row)
        # Directly under the state word and clock it belongs to, and inset so it
        # does not run the full card width like a progress bar would.
        # Stretches and a width cap, not margins: the recording card is the
        # widest of the three and grows with the screen, so fixed side margins
        # let the bar sprawl to ~700px on a 1920 panel - which read as a heavy
        # band across the card rather than an indicator belonging to the state
        # word above it. Capped, it stays roughly the width of that word.
        tape_row = QHBoxLayout()
        tape_row.setContentsMargins(0, 1, 0, 0)
        tape_row.addStretch(1)
        # Stretch factor 3, not 0: with 0 the bar only ever gets its sizeHint,
        # which for a plain QWidget is nothing. 3-against-1 either side lets it
        # claim space and then MAX_W caps it, so it centres at its intended
        # width on a wide card and shrinks gracefully on a narrow one.
        tape_row.addWidget(self.tape, 3)
        tape_row.addStretch(1)
        col.addLayout(tape_row)
        col.addLayout(pill_row)
        # Twice the gap above the footnote that sits inside the group, so the
        # eye separates "what I can do" from "what is going on".
        # No addSpacing() ahead of this any more: the gap it opened was
        # for the footnote, and the footnote is hidden. Restore the
        # SPACE_3 here if self.detail is ever shown again.
        col.addWidget(self.detail, 0, Qt.AlignHCenter)
        col.addStretch(1)

    def paintEvent(self, _event):
        """The breathing halo behind whichever control is live."""
        target = None
        if self._state == "RECORDING":
            target, tone = self.rec_pill, REC
        elif self._state == "PAUSED":
            target, tone = self.pause_pill, WARN
        if target is None or not target.isVisible():
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        # Off the wall clock, like the blink and the tape bar, so all three
        # pulse together and a busy Pi cannot drift them apart.
        swell = 0.5 + 0.5 * math.sin(time.monotonic() * 2.2)
        r = target.geometry()
        grow = 3.0 + 5.0 * swell
        col = QColor(tone)
        col.setAlphaF(0.10 + 0.16 * swell)
        p.setPen(Qt.NoPen)
        p.setBrush(col)
        p.drawRoundedRect(
            QRectF(r.left() - grow, r.top() - grow,
                   r.width() + 2 * grow, r.height() + 2 * grow),
            theme.RADIUS_MD + grow, theme.RADIUS_MD + grow)

    def set_status(self, status, snapshot):
        state = status.get("state") or "STOPPED"
        tone, blinks = SESSION_LOOK.get(state, SESSION_LOOK["STOPPED"])
        colour = TONES[tone][0]
        left = status.get("pending_left")

        # Phase off the wall clock rather than a counter, so the blink rate does
        # not follow the UI frame rate when the Pi is busy.
        # Start/stop the halo repaint with the session it belongs to.
        live = state in ("RECORDING", "PAUSED")
        if live and not self._halo_timer.isActive():
            self._halo_timer.start(60)
        elif not live and self._halo_timer.isActive():
            self._halo_timer.stop()
            self.update()               # clear the last frame of the halo

        lit = (not blinks) or (int(time.monotonic() * self.BLINK_HZ * 2) % 2 == 0)
        if (state, lit) != (self._state, self._blink_on):
            self._state, self._blink_on = state, lit
            self.head.setText(f"{'●' if lit else '○'} {state}")
            recolour(self.head, colour, REC_HEAD_PX, bold=True)
        # Outside the blink guard on purpose: that guard fires only when the
        # state or the blink phase changes, and the bar has to follow the state
        # even on a frame where neither did.
        self.tape.set_mode(state, colour)

        if left is not None:
            # The clock becomes a countdown: during this window the number that
            # matters is how long is left to act, not how long the run was.
            self.elapsed.setText(f"{left:.0f}s")
        else:
            self.elapsed.setText(hms(status.get("elapsed"))
                                 if state != "STOPPED" else "--:--")

        toast = status.get("toast")
        error = status.get("error")

        # A RUNNING BUILD OUTRANKS THE TOAST, and that is the whole fix for
        # "the processing only shows sometimes" (operator 2026-08-26).
        #
        # The chain below puts `toast` first, so the SAVED toast covered the
        # strip for TOAST_S while the merge was already running underneath. On a
        # long recording the build outlasted the toast and the operator saw
        # PROCESSING; on a short one it finished first and they saw nothing at
        # all - the same code, two different experiences, depending only on clip
        # length. That is exactly what "only some time" describes.
        #
        # Progress is a STATE, a toast is a MESSAGE. When the two collide the
        # state wins: the operator needs to know ffmpeg is still working far
        # more than they need to be told again that the save they just made
        # succeeded - and pulling the USB stick during that window is the
        # failure this whole line exists to prevent.
        #
        # Only the IN-PROGRESS states mask it. "ready", "done" and "error" do
        # not, so the SAVED / DISCARDED / MERGE FAILED toasts still show
        # normally once there is nothing running to report.
        fv_busy = ((status.get("full_view") or {}).get("state")
                   in ("queued", "normalising", "building", "joining"))
        if toast is not None and not fv_busy:
            text, extra = toast
            # DISCARDED gets the recording red, not amber - footage was deleted,
            # and that has to look different from a save that found nothing.
            tone_key = ("rec" if text.startswith("DISCARDED")
                        else "on" if text == "SAVED" else "pause")
            self.detail.setText(f"{text}   {extra}")
            recolour(self.detail, TONES[tone_key][0], REC_DETAIL_PX, bold=True)
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
            recolour(self.detail, WARN, REC_DETAIL_PX, bold=True)
        elif (status.get("usb") or {}).get("state") in _USB_BUSY:
            # The USB daemon is working on a stick right now. Shown above
            # everything routine (but below toasts and the confirm window, which
            # are operator decisions in flight) because the one mistake this line
            # prevents is pulling the stick mid-copy.
            #
            # Every phase is named, not just the copy: detect, mount and the
            # copy plan used to publish nothing, so on a full stick the strip sat
            # blank for seconds through exactly the part where the operator is
            # standing there wondering whether the stick took at all.
            usb = status["usb"]
            state = usb.get("state")
            if state == "copying":
                total = usb.get("bytes_total") or 0
                pct = (100.0 * (usb.get("bytes_done") or 0) / total
                       if total else 0.0)
                self.detail.setText(
                    f"COPYING TO USB  {pct:.0f}%  ·  {usb.get('file') or ''}"
                    f"  ·  {usb.get('file_i') or 0}"
                    f"/{usb.get('files_total') or 0} files")
            elif state == "detected":
                self.detail.setText("USB FOUND  ·  do not remove")
            elif state == "mounting":
                self.detail.setText("OPENING USB  ·  do not remove")
            elif state == "scanning":
                self.detail.setText(
                    "CHECKING WHAT TO COPY  ·  do not remove")
            elif state == "finishing":
                # The stick went in before the merge finished. Nothing is being
                # copied this second, which is exactly when the old strip fell
                # silent and the operator pulled it - see usb_backup's settle
                # loop for the recording that lost its full view that way.
                self.detail.setText(
                    "FINISHING VIDEO FILES  ·  DO NOT REMOVE USB")
            else:                                   # clearing
                self.detail.setText(
                    f"FREEING SPACE ON PI  ·  copy verified  ·  "
                    f"{usb.get('files_total') or 0} files")
            recolour(self.detail, WARN, REC_DETAIL_PX, bold=True)
        elif (status.get("usb") or {}).get("state") == "done":
            usb = status["usb"]
            self.detail.setText(
                f"COPY COMPLETE  ·  SAFE TO REMOVE USB  ·  "
                f"{usb.get('copied') or 0} files copied, "
                f"{usb.get('deleted') or 0} freed off Pi")
            recolour(self.detail, TONES["on"][0], REC_DETAIL_PX, bold=True)
        elif (status.get("usb") or {}).get("state") == "error":
            self.detail.setText(
                f"COPY FAILED  ·  {(status['usb'].get('detail') or '')}"
                f"  ·  nothing deleted from Pi")
            recolour(self.detail, BAD, REC_DETAIL_PX, bold=True)
        elif ((status.get("full_view") or {}).get("state")
              in ("queued", "normalising", "joining")):
            # Same reason the BUILDING line exists: this runs after the save,
            # rewrites the per-camera masters in place, and on a long clip it
            # outlasts the SAVED toast. An idle strip here is what makes an
            # operator pull the stick out mid re-encode.
            fv = status["full_view"]
            self.detail.setText(
                f"PROCESSING VIDEO  {fv.get('frac', 0.0) * 100:.0f}%"
                f"  ·  clip {fv.get('clip') or 0}"
                f"/{fv.get('clips_total') or 0}{_queued(fv)}"
                f"  ·  wait before plugging in USB")
            recolour(self.detail, WARN, REC_DETAIL_PX, bold=True)
        elif (status.get("full_view") or {}).get("state") == "building":
            # The side-by-side is built after the save, not while recording, so
            # it outlives the SAVED toast on any run of length. Without a line
            # for it the operator sees SAVED, then an idle strip, while ffmpeg
            # is still working - and pulls the stick before the file exists.
            fv = status["full_view"]
            self.detail.setText(
                f"MERGING CAMERAS INTO ONE VIDEO"
                f"  {fv.get('frac', 0.0) * 100:.0f}%"
                f"  ·  clip {fv.get('clip') or 0}"
                f"/{fv.get('clips_total') or 0}{_queued(fv)}"
                f"  ·  wait before plugging in USB")
            recolour(self.detail, WARN, REC_DETAIL_PX, bold=True)
        elif (status.get("full_view") or {}).get("ready"):
            # The whole point of the two lines above: this is the instant the
            # merged file exists and the footage is complete on the card. Plug
            # the stick in NOW and the transfer is a straight copy with nothing
            # left running behind it.
            fv = status["full_view"]
            if fv.get("state") == "error":
                self.detail.setText(
                    f"VIDEO SAVED (merge failed)  ·  camera files are safe"
                    f"  ·  PLUG IN USB  ·  {(fv.get('error') or '')[:34]}")
                recolour(self.detail, BAD, REC_DETAIL_PX, bold=True)
            else:
                # LEADS WITH "VIDEO SAVED", operator 2026-08-26: "if already
                # saved to show video is already saved". The old wording only
                # said what to do NEXT (plug in the stick) and never said the
                # thing the operator was waiting to hear - so a strip that had
                # finished looked much like one still working, and the only way
                # to be sure was to remember whether the percentage had gone.
                n = fv.get("built") or 0
                self.detail.setText(
                    f"VIDEO SAVED  ·  {n} file{'s' if n != 1 else ''} ready"
                    f"  ·  PLUG IN USB TO TRANSFER")
                recolour(self.detail, TONES["on"][0], REC_DETAIL_PX, bold=True)
        elif (status.get("full_view") or {}).get("state") == "error":
            self.detail.setText(
                f"MERGE FAILED  ·  {(status['full_view'].get('error') or '')}"
                f"  ·  camera files are safe")
            recolour(self.detail, BAD, REC_DETAIL_PX, bold=True)
        elif error:
            self.detail.setText(error)
            recolour(self.detail, BAD, REC_DETAIL_PX, bold=True)
        elif state == "STOPPED":
            free = status.get("free_mb")
            self.detail.setText(f"idle  ·  {free / 1024:.1f} GB free"
                                if free is not None else "idle")
            recolour(self.detail, MUTED, REC_DETAIL_PX)
        else:
            mb = (status.get("bytes") or 0) / 1e6
            self.detail.setText(
                f"clip {status.get('clip', 0):03d}  ·  "
                f"{hms(status.get('clip_elapsed'))}  ·  {mb:,.0f} MB")
            recolour(self.detail, MUTED, REC_DETAIL_PX)

        # The ring IS the old footnote. Every state that line used to
        # describe in words now lands on the button: turning while the merge
        # runs, a tick when the footage is on the card, a cross when it is not.
        fv = status.get("full_view") or {}
        fv_state = fv.get("state")
        if fv_state in ("queued", "normalising", "joining", "building"):
            self.save_btn.set_status("busy", fv.get("frac") or 0.0)
        elif fv_state == "error":
            self.save_btn.set_status("error", 1.0)
        elif fv.get("ready"):
            self.save_btn.set_status("done", 1.0)
        else:
            self.save_btn.set_status("idle", 0.0)

        switches = snapshot.get("switches") or {}
        self.rec_pill.set_value(switches.get("START / STOP"))
        self.pause_pill.set_value(switches.get("PAUSE / RESUME"))

        presses = snapshot.get("save_presses")
        if presses is not None:
            if self._presses is not None and presses > self._presses:
                self._flash_until = time.monotonic() + self.SAVE_FLASH_S
                self.save_btn.flash()
            self._presses = presses
        # Through the confirm window the button breathes on its own: it is
        # the only control that can keep the recording, and the operator has a
        # counted number of seconds to find it.
        self.save_btn.set_armed(left is not None)


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
        # THE READOUT FIGURE_SPACE EXISTS FOR. Padding to three digits only
        # holds the word still if every digit is the same width, which is true
        # of the numeric face and NOT of Inter — see font().
        self.joy_text = label("—", PT_VALUE, INK, bold=True, numeric=True)
        # LEFT, not centred: the pad is trailing now, and centring a string
        # whose right end is blank would shove the visible text off-centre by
        # half the pad - and by a DIFFERENT amount at each reading, which is
        # exactly the drift the padding exists to remove. The label's own
        # pinned width keeps the block centred in the column.
        # CENTRED, so the reading sits on the joystick box's axis. Left-aligned
        # with a trailing pad it hung ~11px to the left of the box, because the
        # pad is blank and only the ink is visible - which is the "not in
        # between" the operator was looking at.
        self.joy_text.setAlignment(Qt.AlignCenter)
        # PINNED TO ITS WIDEST READING. This label is centred in its column, so
        # its width is what centres everything above it - and the text grows a
        # digit at a time ("THROTTLE 0%" 127px -> "THROTTLE 100%" 149px). Left
        # free, the joystick box shifted sideways as the number crossed 10% and
        # again at 100%, so the whole block twitched while the stick was moved.
        #
        # Same guard the clock in topbar.py and `elapsed` below already carry,
        # for the same reason: any label that both changes every frame AND
        # participates in centring has to be measured at its widest once.
        self.joy_text.setFixedWidth(
            QFontMetrics(self.joy_text.font()).horizontalAdvance("THROTTLE 100%") + 6)

        # Last drawable stick position, for DISPLAY continuity only - see
        # set_state(). PER AXIS since 2026-08-22: axis -> (value, stamp), so a
        # channel that dies alone dashes alone instead of taking both down.
        self._joy_disp = {"x": None, "y": None}

        self.pot = PotBar()
        # Kept as a widget but no longer laid out: the dial draws its own number
        # in the middle now, and a second copy beside it read as two readings.
        # set_state() still updates it so nothing downstream has to care.
        # The light dial's percentage, redrawn as the pot turns — numeric face.
        self.pot_text = label("—", PT_BIG, INK, bold=True, numeric=True)
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
        # A SparkleRing, not a Pill - see that class. Same set_text/set_value
        # API, so set_state() below did not change.
        self.brush = SparkleRing(PILL_TONE["BRUSH"])
        self.actuator = LinearActuator(TOOLS_TONE)

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
        for name, val in (("x", x), ("y", y)):
            if val is not None:
                self._joy_disp[name] = (val, now)
        held = self._joy_disp["x"]
        if x is None and held is not None and now - held[1] < JOY_DISPLAY_HOLD_S:
            x = held[0]
        held = self._joy_disp["y"]
        if y is None and held is not None and now - held[1] < JOY_DISPLAY_HOLD_S:
            y = held[0]
        # DISPLAY-ONLY AXIS SWAP, operator 2026-08-26: "in my backend my x and y
        # is swaped ... only frontend, backend side no any change because its
        # correct".
        #
        # The wheels are right and must not be touched, so this cannot go in
        # inputs.py - SWAP_XY there feeds the mixer as well and would undo a
        # calibration that is already correct on the rig. This is the last point
        # before the picture, and nothing downstream of it reaches a motor.
        #
        # KEEP IT HERE IF THE PANEL EVER LOOKS WRONG AGAIN. The rule for this
        # file is the same one the dot's ox/oy signs already follow: it is not an
        # opinion about orientation, it is whatever makes the PICTURE match the
        # hand, given whatever inputs.py is sending. Change one, check the other.
        self.joy.set_pos(y, x)
        # ONE number for the whole stick, 2026-08-24 on the operator's ask:
        # "remove turn and add all in throttle". Not |y|, which would read 0%
        # while spinning on the spot at full power - THE PEAK WHEEL DEMAND, so
        # any deflection in any direction shows up in the one figure.
        #
        # Computed the way uno_serial.mix() computes it, deliberately duplicated
        # rather than approximated, so the percentage on the panel IS the
        # fraction of MAX_PWM the faster wheel is being commanded to:
        #
        #     left = y + x, right = y - x, then both divided by
        #     peak = max(1, |left|, |right|) so a corner cannot demand 2.0.
        #
        # Full forward reads 100%. Full turn on the spot reads 100%. Full
        # forward AND full turn also reads 100%, because that is exactly what
        # the wheels get once mix() has scaled the pair back down - the outer
        # wheel is saturated either way. Algebraically this is min(1, |x|+|y|).
        #
        # Both axes required: mix() returns a dead stop if either is None, and
        # "-" is the honest reading for "no data" rather than a confident 0%.
        # The caption above stays JOYSTICK, so the word rides on the value line
        # - a bare "0%" under "JOYSTICK" does not say what is 0%. Operator's
        # call, 2026-08-24, after trying it the other way round.
        # UNPADDED, and centred by the label - see its setAlignment above.
        #
        # THE PADDING WAS REMOVED DELIBERATELY, and the trade it was making is
        # worth stating because it is not free either way. Three things want to
        # be true of this readout and only two can be at once:
        #
        #   1. the joystick box above must not move
        #   2. the text must sit centred on that box
        #   3. the word must not shift as the number gains a digit
        #
        # (1) is held by the label's FIXED width, which keeps the column - and
        # so the box - the same size at every reading. That one is not
        # negotiable; a twitching box is the worst of the three.
        #
        # Padding to three digits bought (3) and gave up (2): the pad is blank,
        # so the visible ink sat left of centre by half of it. Dropping the pad
        # buys (2) and gives up (3) - the word now shifts by up to half a digit
        # as the reading crosses 10% and 100%. That is ~5px, against the ~11px
        # of off-centre it replaces, and the operator asked for centred.
        #
        # numeric=True still earns its keep: it makes every DIGIT the same
        # width, so 0->9 and 10->99 do not move at all. Only a change in the
        # NUMBER OF digits can shift anything now.
        if x is None or y is None:
            self.joy_text.setText("THROTTLE —")
        else:
            left, right = y + x, y - x
            peak = max(1.0, abs(left), abs(right))
            demand = max(abs(left), abs(right)) / peak
            self.joy_text.setText(f"THROTTLE {demand * 100:.0f}%")

        pot = state.get("pot") or {}
        # The SETTLED value, not the one the lamp gets - see POT_VIEW_S in
        # inputs.py. Falls back to "pct" so an older reader still renders.
        _v = pot.get("pct_view")
        if _v is None:
            _v = pot.get("pct")
        self.pot.set_pct(_v)
        self.pot_text.setText(
            f"{_v:.0f}%" if _v is not None else "—")

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
