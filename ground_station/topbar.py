#!/usr/bin/env python3
"""
The window's top bar: logo, title, camera chips, robot link chip, SoC
temperature, clock.

Everything the operator has to be able to check without looking away from the
video. The camera chips and the robot chip answer *different* questions and are
allowed to disagree — MediaMTX can be up and reachable with a dead USB camera
(robot green, camera red), and video keeps arriving for a moment after the
tether is pulled (camera green, robot red). Showing one merged "ok" light would
throw that away, which is exactly the information you need when something breaks.

Nothing in here polls anything. main.py owns the single UI timer and pushes
state in through set_camera() / set_robot() / set_clock(); link.py owns the
robot probe. That keeps the bar cheap enough to update at the full UI frame rate.

Preview it on its own, with no cameras and no robot:

    python3 topbar.py
"""

import os
import time

from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QColor, QFont, QFontMetrics, QPainter, QPixmap
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QWidget

import theme
from recorder import hms

# The bar used to set its chips and clock in the terminal mono. Mono was really
# only buying a clock that does not twitch as the digits change, and a fixed
# width on that one label buys the same thing without making the whole bar look
# like a console. Kept as a name so the clock rule below stays readable.
#
# Now Inter, via theme.py — Apple's SF Pro stand-in. Inter carries TABULAR
# FIGURES, which is the other half of what mono was doing here: the clock rule
# below asks for them by feature tag, so the digits hold their columns without
# the whole bar having to be set in a typewriter face.
SANS = theme.FAMILY_TEXT
MONO = SANS

# The bar is sized around the logo rather than the other way round: 42 + 6px of
# air top and bottom. Those 16px over the original 38 come straight out of the
# video panels, which is the trade — the logo is the one thing on the bar that
# is brand rather than data, and at 18px it read as a smudge from any distance.
# Scaled up 15% from 54/42/20 with the type and the pill widths moved to match -
# raising the bar alone would just have added white space around the same small
# text. The pill min-widths went up with the font size for the same reason
# resizeEvent() cares: a wider glyph in an unchanged floor clips the label.
BAR_HEIGHT = 62
# The source is 4.47:1, so every pixel of height costs ~4.5px of width on the
# left group; check the narrow widths in resizeEvent() if you go further.
LOGO_HEIGHT = 48
# 28, up from 23, and on the 8pt grid at last. Two reasons beyond tidiness: a
# 13px label in a 23px chip leaves 5px of air above and below, which is under
# half the leading the type wants and is why the chips read as cramped strips
# rather than as objects; and capsule(28) = 14, a radius big enough to be a
# deliberate shape rather than a softened rectangle. Still 34px clear of
# BAR_HEIGHT, so the logo's 48 is untouched.
PILL_HEIGHT = 28
# Horizontal padding inside a chip. Dropped 12 -> 10 when the chip floors were
# re-measured: with the +15% type every chip needed more width than the row had
# to give, and 2px a side across six chips buys back 24 of the 128px needed to
# keep the title AND render every chip whole at 1920. See resizeEvent().
PILL_PAD = 10

# Chip width floors - the widest text each chip can ever hold, MEASURED with
# that text actually in the chip (QT_QPA_PLATFORM=offscreen, then
# StatusPill(text).sizeHint().width()) rather than estimated from characters.
#
# A floor BELOW the widest text does not give a narrower chip, it gives a WRONG
# one: a QLabel does not elide, it clips, so the chip renders "CAM 1  NO SIGNAI"
# and the operator reads a status that does not exist. That is precisely what
# these were doing - BAR_HEIGHT and the type went up 15%, the floors stayed at
# the old numbers, and every chip on the bar lost its last glyph.
#
# Re-measure whenever a chip gains a longer string. Current worst cases:
#   REC   "PROCESSING  100%"     USB    "USB  COPYING 100%"
#   CAM   "CAM 1  CONNECTING"    ROBOT  "ROBOT  DISCONNECTED"
#   TEMP  "TEMP  100°C"
# RE-MEASURED 2026-08-24 for Inter Medium 13px, exactly as the note above
# demands — QT_QPA_PLATFORM=offscreen, StatusPill(text).sizeHint().width(), then
# rounded UP to the 4px grid so the row keeps its rhythm and no floor can land
# under its measurement. Every one came in NARROWER than the DejaVu Bold it
# replaced (980px of chips -> 900px), because medium weight in a UI face is
# tighter than bold in a general-purpose one. That 80px goes to the title, which
# is what used to be squeezed out first at 1920.
W_REC, W_USB, W_CAM, W_ROBOT, W_TEMP = 180, 188, 188, 212, 132

# --- palette -----------------------------------------------------------------
# The bar wears the boot theme, so the handover from Plymouth -> splash.py ->
# viewer never changes colour: the same white->#e8edf6 field and the same navy
# logo carry all the way from power-on into the running app. Both of those are
# defined elsewhere (setup_splash.sh paints the Plymouth background, splash.py
# the loading screen) - keep the three in step if you retheme any one of them.
BAR_TOP = "#ffffff"
BAR_BOTTOM = "#e8edf6"
# HAIRLINE, not a drawn border. Apple's separator is a low-alpha grey that
# disappears into whatever it sits on; the old #c3cee2 was solid and dark enough
# to read as a rule the operator was meant to notice. Deference — the bar should
# end, not announce that it ends.
BAR_LINE = theme.LIGHT["separator"]

# Brand, from theme.py so the boot chain (Plymouth -> splash -> bar) and this
# file cannot drift apart the way they had.
INK = theme.BRAND_INK      # the dark navy of "ARNOBOT"
ACCENT = theme.BRAND_ACCENT   # the mid blue of the "R" — splash.py's subtitle colour
# Secondary label rather than its own grey. The same navy at 60% alpha, so the
# clock recedes without shifting hue — the HIG's emphasis levels, of which the
# bar now uses three: wordmark (label), title (accent), clock (label2).
MUTED = theme.LIGHT["label2"]

# On the light field the logo is drawn in its own brand colours, exactly as
# Plymouth and the splash draw it. Set TOPBAR_LOGO_TINT="#e8f0fb" (or any
# colour) to restamp every non-transparent pixel instead — needed only if the
# bar is ever darkened again.
LOGO_TINT = os.environ.get("TOPBAR_LOGO_TINT", "")

# state -> (text/dot colour, border, fill). The dot and the text share a colour,
# which is what lets a whole chip be a single QLabel instead of a dot widget
# plus a text widget kept in sync. Tinted-light fills, not the saturated blocks
# a dark bar needs: on this field the colour has to read against white.
# Now Apple's own light-mode system semantics instead of six hand-mixed tints —
# see theme.STATUS_LIGHT, which keeps this exact (fg, border, fill) shape
# because the shape was right. The structure and every comment below it still
# apply; only the six values moved.
PILL_STATES = {
    "ok":   theme.STATUS_LIGHT["ok"],
    "warn": theme.STATUS_LIGHT["warn"],
    "bad":  theme.STATUS_LIGHT["bad"],
    "idle": theme.STATUS_LIGHT["idle"],
    # The one saturated chip on the bar, and deliberately so: every other state
    # here is something you check, whereas "am I recording" is something that
    # has to reach you when you are looking at the video instead of the bar.
    "rec":  theme.STATUS_LIGHT["rec"],
}


def load_logo(path, height=LOGO_HEIGHT, tint=LOGO_TINT):
    """Bar-height logo pixmap, optionally restamped in one colour. None if missing."""
    if not path or not os.path.exists(path):
        return None
    pixmap = QPixmap(path)
    if pixmap.isNull():
        return None

    # Scale first, tint second: the tint is per-pixel work and the source is
    # 1345x301. SmoothTransformation leaves soft alpha at the edges, and the
    # SourceIn pass below preserves it, so the result has no jagged fringe.
    pixmap = pixmap.scaledToHeight(height, Qt.SmoothTransformation)
    if not tint:
        return pixmap

    stamped = QPixmap(pixmap.size())
    stamped.fill(Qt.transparent)
    painter = QPainter(stamped)
    painter.drawPixmap(0, 0, pixmap)
    # SourceIn keeps the destination alpha and replaces the colour everywhere.
    painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
    painter.fillRect(stamped.rect(), QColor(tint))
    painter.end()
    return stamped


def _tracked(family, spacing):
    """A font with letter spacing — the one thing Qt style sheets cannot set.

    Sizes stay in the style sheet: a QWidget rule there (main.py sets one) beats
    anything setFont() asks for, so mixing the two only produces surprises.
    Letter spacing has no style sheet property at all, so it survives.
    """
    font = QFont(family)
    font.setLetterSpacing(QFont.AbsoluteSpacing, spacing)
    return font


def _separator():
    line = QFrame()
    line.setObjectName("sep")
    line.setFixedSize(1, 16)
    return line


def _after_session(fv):
    """The REC chip once recording has STOPPED: (text, state, dot).

    STANDBY is only true when nothing is happening. The post-save pipeline -
    re-encoding the per-camera masters, then hstacking them into the full view -
    outlives the SAVED toast on any run of length, and a chip reading STANDBY
    all the way through it is exactly what sends an operator to the USB socket
    while ffmpeg is still writing. inputs_panel's strip has narrated this since
    the pipeline landed; the chip is the half of the bar the operator is already
    looking at, because it is the half next to the picture.
    """
    fv = fv or {}
    state = fv.get("state")
    if state in ("queued", "normalising"):
        # "PROCESSING", not "NORMALISING": the operator is being told to wait,
        # not what ffmpeg is doing. Same word the strip uses.
        #
        # "queued" is the instant between Thread.start() and the builder's first
        # statement. Left out, it put STANDBY back on the chip for those frames.
        return f"PROCESSING  {fv.get('frac', 0.0) * 100:.0f}%", "warn", "◐"
    if state == "building":
        return f"MERGING  {fv.get('frac', 0.0) * 100:.0f}%", "warn", "◐"
    if fv.get("ready"):
        # Processing has finished and no backup has run since - the one moment
        # when plugging a stick in is a straight copy of a complete session.
        if state == "error":
            return "MERGE FAILED", "bad", "●"
        return "READY  PLUG USB", "ok", "●"
    if state == "error":
        return "MERGE FAILED", "bad", "●"
    return "STANDBY", "idle", "●"


def _usb_chip(usb):
    """The USB chip: (text, state, dot).

    The strip's vocabulary cut to chip length. Every phase the daemon publishes
    gets a word, not just the copy: detect, mount and the copy plan are seconds
    each on a full stick, and a chip that goes blank through them reads as "the
    stick did not take" at precisely the moment it must not.
    """
    usb = usb or {}
    state = usb.get("state")
    if not state:
        # No status file, or one nobody has stamped for 10s - main._usb_status()
        # already treats a dead daemon as absent. There is no backup here.
        return "USB  —", "idle", "●"
    if state == "idle":
        # The daemon is up and watching the bus. This is the state that had no
        # chip at all: the socket works, there is simply nothing in it, and the
        # operator could not tell that apart from the daemon being dead.
        return "USB  READY", "idle", "○"
    if state in ("detected", "mounting"):
        return "USB  OPENING", "warn", "●"
    if state == "scanning":
        return "USB  CHECKING", "warn", "●"
    if state == "copying":
        total = usb.get("bytes_total") or 0
        pct = 100.0 * (usb.get("bytes_done") or 0) / total if total else 0.0
        # The one chip state where pulling the stick destroys something, so it
        # blinks - off the wall clock, like the REC chip, so a busy Pi drops
        # frames without slowing the blink.
        lit = int(time.monotonic() * 2.8) % 2 == 0
        return (f"USB  COPYING {pct:.0f}%", "warn", "●" if lit else "○")
    if state == "finishing":
        # The stick went in before the merge finished. Nothing is copying this
        # second, which is the gap the operator used to read as "done".
        return "USB  FINISHING", "warn", "●"
    if state == "clearing":
        return "USB  CLEARING", "warn", "●"
    if state == "done":
        return "USB  COMPLETE", "ok", "●"
    if state == "error":
        return "USB  FAILED", "bad", "●"
    # A phase added to usb_backup.py and not here still shows as busy rather
    # than vanishing, which is the failure mode that matters.
    return (("USB  " + state.upper())[:20], "warn", "●")


class StatusPill(QLabel):
    """A rounded chip whose colour carries the state: ok / warn / bad / idle."""

    def __init__(self, text="—", state="idle", min_width=0):
        super().__init__()
        self.setObjectName("pill")
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(PILL_HEIGHT)
        self.setFont(_tracked(MONO, 0.6))
        if min_width:
            # Chips sit in a right-aligned row, so a chip that changes width
            # shoves its neighbours sideways. A floor on the width keeps the row
            # still while the text underneath changes every second.
            self.setMinimumWidth(min_width)

        self._text = None
        self._state = None
        self._dot = None
        self.set(text, state)

    def set(self, text, state, dot="●"):
        """Idempotent — safe to call at the UI frame rate.

        `dot` is swapped rather than the colour when a chip blinks: changing the
        text is one repaint, while assigning a style sheet forces a full
        repolish of the widget, and a chip that blinks twice a second would be
        doing that 4x/second forever.
        """
        if (text, dot) != (self._text, self._dot):
            self._text, self._dot = text, dot
            self.setText(f"{dot}  {text}")
        if state != self._state:
            self._state = state
            fg, border, bg = PILL_STATES.get(state, PILL_STATES["idle"])
            # Assigning a style sheet forces a full repolish of the widget, so
            # it happens on state *changes* only, not on every text update.
            # MEDIUM, NOT BOLD. Bold at 13px on a light field is heavier than
            # anything else on the bar including the wordmark, so six chips of
            # it out-shouted the brand and each other — and weight was being
            # asked to carry state that the colour already carries. Medium keeps
            # the chip legible at distance, lets the colour do the signalling,
            # and is narrower, which buys the row back a few px per chip.
            self.setStyleSheet(
                f"QLabel#pill {{ color: {fg}; background: {bg};"
                f" border: 1px solid {border};"
                f" border-radius: {theme.capsule(PILL_HEIGHT)}px;"
                f" padding: 0 {PILL_PAD}px; font-size: {theme.FOOTNOTE}px;"
                f" font-weight: {theme.W_MEDIUM}; }}"
            )


class TopBar(QWidget):
    """Logo | title | REC | USB | one chip per camera | robot | temp | clock."""

    def __init__(self, cameras, logo_path=None, title="GROUND CONTROL STATION"):
        super().__init__()
        self.setObjectName("topbar")
        self.setFixedHeight(BAR_HEIGHT)
        # A plain QWidget subclass does not paint a style sheet background of
        # its own accord — QFrame and QLabel do, which is why the pills work
        # without this and the bar itself would not.
        self.setAttribute(Qt.WA_StyledBackground, True)

        # CHIP NAMES ARE THE NUMBER ONLY - "CAM 1", never "CAM 1 · FRONT".
        #
        # config.camera_name() still carries the FRONT/BACK label and it still
        # rides the PANEL HEADER and the RECORDING FILE NAMES, which is where it
        # earns its keep: footage pulled off a USB stick months later has to be
        # tellable apart, and cam1_front_003.mp4 does that on its own. Up here it
        # only made the chip wider and repeated what the panel directly below it
        # already says, so the bar reads CAM 1 / CAM 2 and nothing else.
        # Operator's call, 2026-08-19. Changing this does NOT rename any file.
        self._names = [f"CAM {i + 1}" for i in range(len(cameras))]

        # --- brand ----------------------------------------------------------
        logo_label = QLabel()
        logo_label.setObjectName("logo")
        self._logo_label = logo_label
        logo = load_logo(logo_path)
        if logo is not None:
            logo_label.setPixmap(logo)
        else:
            # Never leave the corner empty just because the asset moved.
            logo_label.setText("ARNOBOT")
            logo_label.setObjectName("wordmark")
            logo_label.setFont(_tracked(SANS, 1.5))

        self._title_label = QLabel(title.upper())
        self._title_label.setObjectName("title")
        self._title_label.setFont(_tracked(SANS, 2.5))

        title_group = QWidget()
        title_group.setObjectName("titlegroup")
        title_row = QHBoxLayout(title_group)
        title_row.setContentsMargins(0, 0, 0, 0)
        title_row.setSpacing(8)
        title_row.addWidget(_separator())
        title_row.addWidget(self._title_label)
        self._title_group = title_group

        # --- status ---------------------------------------------------------
        self._camera_pills = []
        for name, url in cameras:
            pill = StatusPill(f"{name}  —", "idle", min_width=W_CAM)
            pill.setToolTip(url)
            self._camera_pills.append(pill)

        # Recording sits left of the camera chips, not with the robot/temp
        # group: it answers "is this being kept", which belongs beside the
        # cameras it is keeping rather than beside the housekeeping.
        self._rec_pill = StatusPill("STANDBY", "idle", min_width=W_REC)

        # Where the footage GOES, beside where it is being kept. The backup
        # daemon has published a full phase list all along and nothing on this
        # bar read it, so the bar could not answer "is the stick in and working"
        # - the question an operator asks while standing at the socket looking
        # at the screen, not down at the strip.
        self._usb_pill = StatusPill("USB  —", "idle", min_width=W_USB)
        # ON DEMAND, 2026-08-24: the chip appears when there is a stick to talk
        # about and is absent otherwise. Two things gate it and BOTH have to
        # agree - this flag (is anything happening?) and the width threshold in
        # resizeEvent (is there room?) - so they are applied together in
        # _apply_usb_visibility() rather than each calling setVisible() and
        # fighting the other. Starts False: nothing is plugged in at boot.
        self._usb_present = False
        # Same gate as the USB chip - see _apply_rec_visibility(). Nothing is
        # recording at boot.
        self._rec_present = False

        self._robot_pill = StatusPill("ROBOT  LINKING", "warn", min_width=W_ROBOT)

        # This machine's own temperature, so it belongs with the clock rather
        # than with the robot. See thermal.py for why it cannot be the robot's.
        self._temp_pill = StatusPill("TEMP  —", "idle", min_width=W_TEMP)

        self._clock_label = QLabel("--:--:--")
        self._clock_label.setObjectName("clock")
        # Ticks once a second in a proportional face, so pin the width to its
        # widest reading or the separator beside it steps sideways all day.
        # MEASURED AT THE SIZE THE STYLE SHEET ACTUALLY USES. This asked for
        # QFont(SANS, 10) while the rule above set 14px, so the pin was computed
        # from the wrong metrics and only held because +20 of slack covered the
        # gap. theme.font_for() returns the same face, size and tracking the
        # rule applies, so the number below is now the real one — and it has to
        # be, because the clock is 15px now and tabular figures changed the
        # advance again.
        self._clock_label.setFixedWidth(
            QFontMetrics(theme.font_numeric(theme.SUBHEAD, theme.W_MEDIUM))
            .horizontalAdvance("00:00:00") + theme.SPACE_4)
        self._clock_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # --- layout ---------------------------------------------------------
        row = QHBoxLayout(self)
        row.setContentsMargins(12, 0, 12, 0)
        row.setSpacing(8)
        # A layout normally forces its own minimum onto the widget, and that
        # minimum propagates all the way up to the window - which would undo
        # minimumSizeHint() below and keep the bar too wide to ever shed anything.
        row.setSizeConstraint(QHBoxLayout.SetNoConstraint)
        row.addWidget(logo_label)
        row.addWidget(title_group)
        row.addStretch(1)
        row.addWidget(self._usb_pill)
        row.addWidget(_separator())
        for pill in self._camera_pills:
            row.addWidget(pill)
        # REC sits to the RIGHT of the camera chips as of 2026-08-24, operator's
        # call. It was on their left, which put it beside the cameras it is
        # keeping either way - the reason for that placement is unchanged, only
        # the side. On this side it also lands next to the group that appears
        # and disappears with it, rather than in the middle of the always-on
        # chips, so the row does not reflow through the cameras when a session
        # starts.
        # REC IS NOT ON THIS BAR any more, 2026-08-24: it moved onto the two
        # camera panel headers, where "is this being kept" sits on the picture
        # it is about (see CameraPanel.set_recording in main.py). The widget and
        # set_recording() below are kept intact - the phase vocabulary, the
        # blink and the width floor are all still correct - so restoring it is
        # this one addWidget.
        self._rec_pill.hide()
        row.addWidget(_separator())
        row.addWidget(self._robot_pill)
        row.addWidget(self._temp_pill)
        row.addWidget(_separator())
        row.addWidget(self._clock_label)

        self.setStyleSheet(STYLESHEET)

    # -- state in --------------------------------------------------------------

    def set_camera(self, index, connected, fps=0.0, status=""):
        """One camera chip. `status` is RTSPStream.status_text.

        `fps` is accepted and ignored: the chip answers "is this camera up?",
        and a number that ticks between 24 and 26 all day makes that harder to
        read at a glance, not easier. check_streams.py reports the rate.
        """
        if not 0 <= index < len(self._camera_pills):
            return
        name = self._names[index]
        if connected:
            self._camera_pills[index].set(f"{name}  CONNECTED", "ok")
        elif "connecting" in status.lower():
            # status_text carries a ticking "connecting 4s"; the panel header
            # shows those seconds, the chip only needs the state, and dropping
            # them stops the chip resizing once a second.
            self._camera_pills[index].set(f"{name}  CONNECTING", "warn")
        else:
            self._camera_pills[index].set(f"{name}  NO SIGNAL", "bad")

    def set_recording(self, status):
        """One recorder.SessionManager.status() dict, or None for no recorder.

        The elapsed time is on the chip rather than only down in the strip
        because the operator is looking AT the video: this bar is the nearest
        place to the picture that can carry it, and "how long have I been
        recording" is the question that follows "am I recording" every time.
        """
        # HIDDEN WHEN IT WOULD SAY STANDBY, 2026-08-24, the same rule the USB
        # chip follows: the bar carries what is HAPPENING, and "nothing is being
        # recorded" is not an event worth a chip on a display the operator reads
        # while driving. Everything else still shows - REC with its clock,
        # PAUSED, the SAVE? countdown, and the post-session PROCESSING /
        # MERGING / READY PLUG USB / MERGE FAILED run, because each of those is
        # work in flight that an operator can act on or ruin.
        #
        # No recorder at all is also nothing to report.
        if not status:
            self._rec_present = False
            self._apply_rec_visibility()
            return

        state = status.get("state") or "STOPPED"
        held = hms(status.get("elapsed"))
        left = status.get("pending_left")
        if left is not None:
            # The recording is about to be deleted. This is the one chip state
            # that is a question rather than a report, so it blinks and counts -
            # a still chip would read as just another thing that is fine.
            lit = int(time.monotonic() * 2.8) % 2 == 0
            self._rec_pill.set(f"SAVE?  {left:.0f}s", "warn", "●" if lit else "○")
        elif state == "RECORDING":
            # Blink at ~1.4Hz off the wall clock, so a busy Pi slows the frame
            # rate without slowing the blink. Matches inputs_panel.SessionView.
            lit = int(time.monotonic() * 2.8) % 2 == 0
            self._rec_pill.set(f"REC  {held}", "rec", "●" if lit else "○")
        elif state == "PAUSED":
            # Two bars, not a dot: paused is the state most easily mistaken for
            # recording at a glance, and the glyph carries that further than the
            # colour does.
            self._rec_pill.set(f"PAUSED  {held}", "warn", "❚❚")
        else:
            # STOPPED does not mean FINISHED - see _after_session.
            text, tone, dot = _after_session(status.get("full_view"))
            self._rec_pill.set(text, tone, dot)
            # _after_session returns literal "STANDBY" only when nothing is
            # happening at all: no session, no re-encode, no merge, nothing
            # waiting for a stick. Comparing its word rather than re-deriving
            # the condition here keeps the two from drifting apart.
            self._rec_present = text != "STANDBY"
            self._apply_rec_visibility()
            return

        # Reached only from the branches above, all of which are live states.
        self._rec_present = True
        self._apply_rec_visibility()

    def set_usb(self, usb):
        """One main._usb_status() dict, or None/{} when there is no daemon.

        Its own chip rather than another line in the REC chip: the transfer and
        the recording are two different questions, and both get asked at once
        every time a stick goes in.
        """
        # "idle" is the daemon saying "I am alive and the socket is empty", and
        # no state at all means the daemon is not running. Neither is worth a
        # chip on a bar the operator reads while driving - what earns space is
        # a stick actually being worked on, and every one of those phases still
        # shows: OPENING, CHECKING, COPYING nn%, FINISHING, CLEARING, COMPLETE,
        # FAILED.
        #
        # THE PULL-EARLY WARNING IS NOT WEAKENED BY THIS. The states that mean
        # "do not pull the stick" are exactly the ones that make the chip
        # appear, and the strip's own status line narrates them too (see
        # _USB_BUSY in inputs_panel.py). What is dropped is the idle
        # reassurance, which said nothing was happening.
        state = (usb or {}).get("state")
        self._usb_present = bool(state) and state != "idle"
        if self._usb_present:
            self._usb_pill.set(*_usb_chip(usb))
        self._apply_usb_visibility()

    def set_robot(self, connected):
        """connected: True / False / None (no probe result yet)."""
        if connected is None:
            self._robot_pill.set("ROBOT  LINKING", "warn")
        elif connected:
            self._robot_pill.set("ROBOT  CONNECTED", "ok")
        else:
            self._robot_pill.set("ROBOT  DISCONNECTED", "bad")

    def set_temp(self, celsius):
        """This machine's SoC temperature in °C, or None if it has no sensor.

        The thresholds are the Pi's own throttle points, not round numbers: it
        soft-throttles at 80°C and hard-throttles at 85°C, so amber at 70 is a
        warning with time left to do something about it (and red means the
        stutter you are about to see is thermal, not the network).
        """
        if celsius is None:
            self._temp_pill.set("TEMP  —", "idle")
            return
        state = "ok" if celsius < 70 else "warn" if celsius < 80 else "bad"
        self._temp_pill.set(f"TEMP  {celsius:.0f}°C", state)

    def set_clock(self, text):
        if text != self._clock_label.text():
            self._clock_label.setText(text)

    # -- layout ----------------------------------------------------------------

    def _apply_rec_visibility(self):
        """The REC chip shows only when there IS a session AND there is room.

        Same two-part test as _apply_usb_visibility(), and split out for the
        same reason: set_recording() knows about the session and resizeEvent()
        knows about the width, and a setVisible() in each would have them
        overwrite one another.
        """
        # Off the bar entirely - see the layout. Without this, a resize would
        # put it back the moment the window got wide enough.
        self._rec_pill.hide()

    def _apply_usb_visibility(self):
        """The USB chip shows only when there IS a stick AND there is room.

        One place for both tests. Called from set_usb() (presence changed) and
        from resizeEvent() (room changed), because either can flip the answer
        and a setVisible() in each would have them overwrite one another.

        The width thresholds elsewhere in resizeEvent() are deliberately still
        derived with this chip PRESENT: they are a worst-case fit, and sizing
        them to the narrower no-stick row would make the bar shed the title the
        moment a stick went in.
        """
        self._usb_pill.setVisible(self._usb_present and self.width() >= 1166)

    def minimumSizeHint(self):
        # Without this the bar is a floor on the whole window's width: the chips
        # alone add up to ~800px, so on a small panel the window would be forced
        # wider than the screen and the right-hand end would simply hang off it.
        # A small hint lets the window get narrow, resizeEvent() then sheds the
        # optional parts, and the layout fits again.
        return QSize(320, BAR_HEIGHT)

    def resizeEvent(self, event):
        # Ground stations get built on whatever panel is in the drawer, and an
        # 800x480 one cannot fit all of this. Shed the title and keep the logo,
        # the chips and the clock, which are the load-bearing part. Below ~760px
        # the chips themselves stop fitting; nothing sensible is left to do there.
        #
        # The threshold is the width at which the title fits WHOLE. A QLabel does
        # not elide, it clips, so a title that is 40px short does not look tight —
        # it looks like "GROUND CONTROL STATI".
        # Every threshold here moved when the REC chip joined the row, and
        # again when USB did. They are
        # the widths at which the REMAINING content fits WHOLE, so they have to
        # be re-derived whenever a chip is added or its text grows - the old
        # numbers left CAM 1 reading "CONNECTEI" at 1024.
        #
        # These are measured, not estimated, and there is now an exact way to
        # get them: put WORST-CASE text in every chip, hide the tier below, then
        # read layout().minimumSize().width(). With every chip carrying a floor
        # equal to its own worst-case sizeHint (W_* above) that number IS the
        # width at which the remaining content renders whole - no sweeping, no
        # slack to guess at. Re-run it whenever a chip or a floor changes:
        #
        # RE-MEASURED 2026-08-24 after REC moved onto the camera panels. A
        # 197px chip leaving the row moves every threshold under it, and leaving
        # the old numbers would have shed the title at 1893 when the row now
        # fits whole at 1589 - throwing the title away with 300px to spare.
        #
        #   everything visible ... 1589   <- title threshold
        #   title hidden ......... 1306   <- temperature threshold
        #   + temp hidden ........ 1166   <- USB threshold
        #   + USB hidden .........  970   <- logo threshold
        #   + logo hidden .........  748   <- nothing left to shed
        #
        # Measured 2026-08-24, after the floors were corrected. The previous set
        # (1915/1599/1371/1153) predated the +15% type: USB and the logo were
        # both held VISIBLE ~60px below the width their own chips needed, so the
        # bar shed nothing and clipped everything instead.
        width = self.width()
        # Measured with WORST-CASE text in every chip: hide the tier below,
        # then read layout().minimumSize().width(). The +2 on each is not
        # padding for taste - minimumSize() is the EXACT fitting width, and at
        # exactly that width Qt hands one QLabel floor(share) while its
        # sizeHint carries a rounding margin, so the last chip lands 1px short
        # and clips. Verified at every boundary, not just mid-band.
        #
        #   everything visible ... 1893   <- title threshold
        #   title hidden ......... 1585   <- temperature threshold
        #   + temp hidden ........ 1434   <- USB threshold
        #   + USB hidden ......... 1222   <- logo threshold
        #   + logo hidden ........ 1000   <- nothing left to shed
        self._title_group.setVisible(width >= 1589)
        # The temperature chip goes next. Below this the ROBOT chip is the one
        # that clips, and a chip reading "ROBOT  DISCONNEC" is worse than no
        # temperature at all. The cameras, the robot and REC all outrank it.
        self._temp_pill.setVisible(width >= 1306)
        # USB outranks the temperature and loses to everything else. A
        # stick in the socket is something the operator is doing right
        # now and can ruin by pulling early; the SoC temperature is
        # housekeeping they will read off the strip. It is also the only
        # chip here that is dark most of the time, so shedding it costs
        # nothing on the runs where no stick ever appears.
        # USB outranks the temperature and loses to everything else. A
        # stick in the socket is something the operator is doing right
        # now and can ruin by pulling early; the SoC temperature is
        # housekeeping they will read off the strip.
        #
        # Width is only half the test for these two - see the helpers.
        self._apply_usb_visibility()
        self._apply_rec_visibility()
        # The logo goes last, and only because it got big: at 42px tall it is
        # ~188px wide, and on a 1024px panel that is the difference between the
        # camera chips fitting and CAM 1 reading "CONNECTEE". Brand loses to
        # data — on a screen that small the chips ARE the interface.
        self._logo_label.setVisible(width >= 970)
        # Below 748 the cameras, the robot and REC stop fitting between them
        # and there is nothing sensible left to shed.
        super().resizeEvent(event)


STYLESHEET = f"""
/* The gradient has to stay on ONE line: Qt's style sheet parser ends the value
   at the newline, so a wrapped qlineargradient(...) silently drops the whole
   rule and the bar keeps the app's dark background. */
#topbar    {{ background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 {BAR_TOP}, stop:1 {BAR_BOTTOM});
              border-bottom: 1px solid {BAR_LINE}; }}
#sep       {{ background: {BAR_LINE}; }}
/* main.py paints every QWidget in the app's near-black, and a QLabel is a
   QWidget as far as that rule is concerned — without this, each piece of the
   bar would sit in its own dark box on the light field. */
#logo, #titlegroup, #title, #wordmark, #clock {{ background: transparent; }}
/* On the type ramp: subhead for the title, headline for the wordmark. Those are
   the HIG's own sizes, and the pair is exactly the case the guidelines describe
   — headline and body share a size and are told apart by WEIGHT, so the
   wordmark can outrank the title by two steps without being physically much
   bigger. Semibold, not bold: 700 on a 62px bar reads as shouting. */
#title     {{ color: {ACCENT}; font-size: {theme.SUBHEAD}px;
              font-weight: {theme.W_MEDIUM};
              font-family: {theme.stack_for(theme.SUBHEAD)}; }}
#wordmark  {{ color: {INK}; font-size: {theme.HEADLINE}px;
              font-weight: {theme.W_SEMIBOLD};
              font-family: {theme.stack_for(theme.HEADLINE)}; }}
/* THE ONE THING ON THIS BAR NOT SET IN INTER, and deliberately: the clock is
   the only label here whose digits change in place, and Inter's default figures
   are proportional — a "1" is 3px narrower than a "0", so the reading would
   twitch every second. Inter has tabular figures under `tnum` and this PySide6
   build cannot switch them on (QFont.Tag is not constructible; the QSS
   font-feature-settings route only reaches QSS-styled widgets and was tried
   here first). theme.FAMILY_NUMERIC carries uniform figures natively, which is
   what the bar used before and why the clock looks unchanged. The full
   reasoning, with the measurements, is in theme.py. */
#clock     {{ color: {MUTED}; font-size: {theme.SUBHEAD}px;
              font-weight: {theme.W_MEDIUM};
              font-family: {theme.stack_numeric()}; }}
"""


def main():
    """Preview the bar on its own:  python3 topbar.py"""
    import sys
    from datetime import datetime

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication, QVBoxLayout

    import config
    from thermal import read_c

    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("Top bar preview")
    window.resize(1400, 240)
    window.setStyleSheet("background: #0b0f14;")

    bar = TopBar(config.CAMERAS, config.LOGO_PATH, config.TITLE)
    hint = QLabel("cycling through every state — 2s each")
    hint.setAlignment(Qt.AlignCenter)
    hint.setStyleSheet("color: #4c5c70; font-family: monospace;")

    layout = QVBoxLayout(window)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.addWidget(bar)
    layout.addStretch(1)
    layout.addWidget(hint)
    layout.addStretch(1)

    # (cam1, cam2, robot) — the four combinations worth eyeballing.
    scenes = [
        ((False, 0, "connecting"), (False, 0, "connecting"), None),
        ((True, 25.4, "live"), (False, 0, "no signal - retrying"), True),
        ((True, 25.1, "live"), (True, 24.8, "live"), True),
        ((False, 0, "no signal"), (False, 0, "no signal"), False),
    ]
    step = {"i": 0}

    def advance():
        index = step["i"]
        step["i"] += 1
        cam1, cam2, robot = scenes[index % len(scenes)]
        bar.set_camera(0, *cam1)
        bar.set_camera(1, *cam2)
        bar.set_robot(robot)
        # Real reading on a Pi; on anything else cycle the thresholds, so the
        # preview shows all four temperature states without hot silicon.
        live = read_c()
        bar.set_temp(live if live is not None else [None, 47.2, 72.4, 84.1][index % 4])

    def tick_clock():
        bar.set_clock(datetime.now().strftime("%H:%M:%S"))

    advance()
    tick_clock()
    scene_timer = QTimer(window)
    scene_timer.timeout.connect(advance)
    scene_timer.start(2000)
    clock_timer = QTimer(window)
    clock_timer.timeout.connect(tick_clock)
    clock_timer.start(250)

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
