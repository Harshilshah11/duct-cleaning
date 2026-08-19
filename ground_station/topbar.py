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

from recorder import hms

SANS = "DejaVu Sans"
# The bar used to set its chips and clock in the terminal mono. Mono was really
# only buying a clock that does not twitch as the digits change, and a fixed
# width on that one label buys the same thing without making the whole bar look
# like a console. Kept as a name so the clock rule below stays readable.
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
PILL_HEIGHT = 23

# --- palette -----------------------------------------------------------------
# The bar wears the boot theme, so the handover from Plymouth -> splash.py ->
# viewer never changes colour: the same white->#e8edf6 field and the same navy
# logo carry all the way from power-on into the running app. Both of those are
# defined elsewhere (setup_splash.sh paints the Plymouth background, splash.py
# the loading screen) - keep the three in step if you retheme any one of them.
BAR_TOP = "#ffffff"
BAR_BOTTOM = "#e8edf6"
BAR_LINE = "#c3cee2"

INK = "#241f7a"      # the dark navy of "ARNOBOT"
ACCENT = "#3f6fb5"   # the mid blue of the "R" — splash.py's subtitle colour
MUTED = "#8590ab"

# On the light field the logo is drawn in its own brand colours, exactly as
# Plymouth and the splash draw it. Set TOPBAR_LOGO_TINT="#e8f0fb" (or any
# colour) to restamp every non-transparent pixel instead — needed only if the
# bar is ever darkened again.
LOGO_TINT = os.environ.get("TOPBAR_LOGO_TINT", "")

# state -> (text/dot colour, border, fill). The dot and the text share a colour,
# which is what lets a whole chip be a single QLabel instead of a dot widget
# plus a text widget kept in sync. Tinted-light fills, not the saturated blocks
# a dark bar needs: on this field the colour has to read against white.
PILL_STATES = {
    "ok":   ("#0f7a4a", "#a6dcc0", "#e7f6ee"),
    "warn": ("#8a5a06", "#eccf8d", "#fdf3de"),
    "bad":  ("#c02b26", "#f1b3ae", "#fdecea"),
    "idle": ("#6f7c99", "#ccd6e8", "#eef2f9"),
    # The one saturated chip on the bar, and deliberately so: every other state
    # here is something you check, whereas "am I recording" is something that
    # has to reach you when you are looking at the video instead of the bar.
    "rec":  ("#ffffff", "#a3211d", "#d93a33"),
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
            self.setStyleSheet(
                f"QLabel#pill {{ color: {fg}; background: {bg};"
                f" border: 1px solid {border}; border-radius: {PILL_HEIGHT // 2}px;"
                f" padding: 0 12px; font-size: 13px; font-weight: bold; }}"
            )


class TopBar(QWidget):
    """Logo | title | one chip per camera | robot link | SoC temperature | clock."""

    def __init__(self, cameras, logo_path=None, title="GROUND CONTROL STATION"):
        super().__init__()
        self.setObjectName("topbar")
        self.setFixedHeight(BAR_HEIGHT)
        # A plain QWidget subclass does not paint a style sheet background of
        # its own accord — QFrame and QLabel do, which is why the pills work
        # without this and the bar itself would not.
        self.setAttribute(Qt.WA_StyledBackground, True)

        # CHIP NAMES ARE THE NUMBER ONLY - "CAM 1", never "CAM 1 � FRONT".
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
        title_row.setSpacing(12)
        title_row.addWidget(_separator())
        title_row.addWidget(self._title_label)
        self._title_group = title_group

        # --- status ---------------------------------------------------------
        self._camera_pills = []
        for name, url in cameras:
            pill = StatusPill(f"{name}  —", "idle", min_width=173)
            pill.setToolTip(url)
            self._camera_pills.append(pill)

        # Recording sits left of the camera chips, not with the robot/temp
        # group: it answers "is this being kept", which belongs beside the
        # cameras it is keeping rather than beside the housekeeping.
        self._rec_pill = StatusPill("STANDBY", "idle", min_width=173)

        self._robot_pill = StatusPill("ROBOT  LINKING", "warn", min_width=225)

        # This machine's own temperature, so it belongs with the clock rather
        # than with the robot. See thermal.py for why it cannot be the robot's.
        self._temp_pill = StatusPill("TEMPERATURE  —", "idle", min_width=173)

        self._clock_label = QLabel("--:--:--")
        self._clock_label.setObjectName("clock")
        # Ticks once a second in a proportional face, so pin the width to its
        # widest reading or the separator beside it steps sideways all day.
        self._clock_label.setFixedWidth(
            QFontMetrics(QFont(SANS, 10)).horizontalAdvance("00:00:00") + 20)
        self._clock_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        # --- layout ---------------------------------------------------------
        row = QHBoxLayout(self)
        row.setContentsMargins(12, 0, 12, 0)
        row.setSpacing(12)
        # A layout normally forces its own minimum onto the widget, and that
        # minimum propagates all the way up to the window - which would undo
        # minimumSizeHint() below and keep the bar too wide to ever shed anything.
        row.setSizeConstraint(QHBoxLayout.SetNoConstraint)
        row.addWidget(logo_label)
        row.addWidget(title_group)
        row.addStretch(1)
        row.addWidget(self._rec_pill)
        row.addWidget(_separator())
        for pill in self._camera_pills:
            row.addWidget(pill)
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
        if not status:
            self._rec_pill.set("STANDBY", "idle")
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
            self._rec_pill.set("STANDBY", "idle")

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
            self._temp_pill.set("TEMPERATURE  —", "idle")
            return
        state = "ok" if celsius < 70 else "warn" if celsius < 80 else "bad"
        self._temp_pill.set(f"TEMPERATURE  {celsius:.0f}°C", state)

    def set_clock(self, text):
        if text != self._clock_label.text():
            self._clock_label.setText(text)

    # -- layout ----------------------------------------------------------------

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
        # Every threshold here moved when the REC chip joined the row. They are
        # the widths at which the REMAINING content fits WHOLE, so they have to
        # be re-derived whenever a chip is added or its text grows - the old
        # numbers left CAM 1 reading "CONNECTEI" at 1024.
        #
        # These are measured, not estimated: sweep the width down and compare
        # each chip's width() against its sizeHint().width(), which is what a
        # QLabel needs to render without clipping. Guessing from character
        # counts was off by ~100px, because the tracked font and the pill
        # padding both land outside the obvious arithmetic.
        width = self.width()
        self._title_group.setVisible(width >= 1510)
        # The temperature chip goes next. Below this the ROBOT chip is the one
        # that clips, and a chip reading "ROBOT  DISCONNEC" is worse than no
        # temperature at all. The cameras, the robot and REC all outrank it.
        self._temp_pill.setVisible(width >= 1225)
        # The logo goes last, and only because it got big: at 42px tall it is
        # ~188px wide, and on a 1024px panel that is the difference between the
        # camera chips fitting and CAM 1 reading "CONNECTEE". Brand loses to
        # data — on a screen that small the chips ARE the interface.
        self._logo_label.setVisible(width >= 1065)
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
#title     {{ color: {ACCENT}; font-size: 15px; font-weight: bold;
              font-family: "{SANS}", sans-serif; }}
#wordmark  {{ color: {INK}; font-size: 17px; font-weight: bold;
              font-family: "{SANS}", sans-serif; }}
#clock     {{ color: {MUTED}; font-size: 14px; font-family: "{MONO}", monospace; }}
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
