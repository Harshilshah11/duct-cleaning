#!/usr/bin/env python3
"""
Branded loading screen, shown while the viewer wakes up.

Covers the dead time between X coming up and the first video frame — importing
cv2 + PySide6 on a Pi 4 takes a couple of seconds on its own, and the RTSP
connect takes a couple more. Without this the screen sits on the bare X root
(black, with an X cursor) for the whole time and looks like a failed boot.

Nothing here talks to the streams; main.py drives it via set_status() and
decides when to hand over to the real window.
"""

import os

from PySide6.QtCore import Qt, QRect, QTimer
from PySide6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPixmap

import theme
from PySide6.QtWidgets import QWidget

# Sampled from the logo, so the chrome around it never clashes. The field is
# light because the logo is dark navy — on the app's near-black background it
# would be all but invisible.
BG_TOP = QColor("#ffffff")
BG_BOTTOM = QColor("#ffffff")  # flat, was #e8edf6: Plymouth dithers a
# gradient this shallow into visible horizontal banding on the panel in
# the case, and the loading screen has to match the boot theme exactly or
# the handover changes colour. Both ends flattened together, 2026-08-24.
INK = QColor(theme.BRAND_INK)        # the dark navy of "ARNOBOT"
ACCENT = QColor(theme.BRAND_ACCENT)  # the mid blue of the "R"
# Secondary label, not its own grey — same navy at 60%, so the status line
# recedes from the subtitle without shifting hue. See theme.LIGHT.
MUTED = QColor(36, 31, 122, 153)

# Inter, standing in for SF Pro — see theme.py. The splash is the FIRST thing
# the operator sees after Plymouth, so it is also where a font change is most
# visible; keeping it on the same family as the bar is what makes the handover
# read as one app starting rather than two screens in a row.
FONT = theme.FAMILY_TEXT


class SplashScreen(QWidget):
    """Full-screen logo + status line, with a moving dot so it never looks hung."""

    def __init__(self, logo_path=None, subtitle="GROUND STATION", backdrop=False):
        # backdrop=True is the persistent logo layer behind everything (see
        # backdrop.py). Two deliberate differences from the loading screen:
        #
        #   - No WindowStaysOnTopHint. There is no window manager on the ground
        #     station, so stacking is purely the order windows are mapped in:
        #     the backdrop maps first and every later window sits above it. With
        #     the hint it would cover the viewer instead.
        #   - Logo only, no subtitle/dots/status. Plymouth's boot theme draws
        #     exactly the logo on exactly this gradient, so a bare backdrop makes
        #     the handover from boot splash to X invisible — the picture simply
        #     does not change.
        flags = Qt.FramelessWindowHint
        if not backdrop:
            flags |= Qt.SplashScreen | Qt.WindowStaysOnTopHint
        super().__init__(None, flags)
        self.setWindowTitle("Arnobot")
        self.setAttribute(Qt.WA_OpaquePaintEvent, True)

        self._backdrop = backdrop
        self._subtitle = "" if backdrop else subtitle
        self._status = "" if backdrop else "starting"

        # Set by main.py to "stop waiting, show the viewer now". Closing the
        # splash on its own would be worse than useless: it is the only visible
        # window at that point, so Qt would quit the whole app.
        self.on_skip = None

        self._logo = None
        if logo_path and os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            if not pixmap.isNull():
                self._logo = pixmap

        # Rects are filled in by resizeEvent; the animation repaints only the
        # dots strip, so the scaled logo is not re-blitted 16 times a second.
        self._logo_rect = QRect()
        self._subtitle_rect = QRect()
        self._status_rect = QRect()
        self._scaled = None

        # The travelling dots were removed at the operator's request, so
        # nothing on this screen animates any more and there is no 90ms repaint
        # to pay for. Liveness is carried by the status line instead, which
        # main.py restamps as the cameras come up. The timer object stays only
        # because finish() stops it; it is never started.
        self._timer = QTimer(self)

    # -- lifecycle -----------------------------------------------------------

    def show_on_primary(self, app):
        """Size to the whole screen and map it.

        There is no window manager on the ground station, so a plain show() would
        leave this at its requested size in the top-left corner — the same trap
        showFullScreen() falls into in main.py. Set the geometry explicitly.
        """
        screen = app.primaryScreen()
        if screen is not None:
            self.setGeometry(screen.geometry())
        self.show()
        self.raise_()
        app.processEvents()

    def set_status(self, text):
        if text == self._status:
            return
        self._status = text
        self.update(self._status_rect)

    def finish(self, window=None):
        """Hand over to the real window and go away."""
        self._timer.stop()
        self.close()

    # -- painting ------------------------------------------------------------

    def resizeEvent(self, event):
        w, h = self.width(), self.height()

        # Logo sits a little above centre; the status block hangs below it.
        logo_w = min(int(w * 0.46), 900)
        logo_h = int(h * 0.30)
        if self._logo is not None:
            self._scaled = self._logo.scaled(
                logo_w, logo_h, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
            logo_w, logo_h = self._scaled.width(), self._scaled.height()

        # Logo above centre, status block hanging under it, so the group as a
        # whole reads as optically centred rather than stuck to the top.
        top = int(h * 0.38) - logo_h // 2
        self._logo_rect = QRect((w - logo_w) // 2, top, logo_w, logo_h)
        self._subtitle_rect = QRect(0, self._logo_rect.bottom() + int(h * 0.045), w, 28)
        self._status_rect = QRect(0, self._subtitle_rect.bottom() + int(h * 0.06), w, 26)
        super().resizeEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0.0, BG_TOP)
        gradient.setColorAt(1.0, BG_BOTTOM)
        painter.fillRect(self.rect(), QBrush(gradient))

        if self._scaled is not None:
            painter.drawPixmap(self._logo_rect.topLeft(), self._scaled)
        else:
            painter.setPen(INK)
            # Large title, one step up the ramp from Apple's 34 because this
            # is a wordmark on an otherwise empty screen rather than a heading
            # in a list. Semibold rather than bold — the logo pixmap it stands
            # in for is not a bold weight either, so the fallback should not
            # jump when the file is missing.
            painter.setFont(theme.font_for(theme.LARGE_TITLE + 10,
                                           theme.W_SEMIBOLD))
            painter.drawText(self._logo_rect, Qt.AlignCenter, "ARNOBOT")

        if self._backdrop:
            return          # logo only — matches the Plymouth theme exactly

        if self._subtitle:
            # Tracked-out small caps is the one place wide letter-spacing
            # earns its keep, and 6px stays. Medium weight, footnote size: at
            # this tracking bold turns the subtitle into a second wordmark
            # competing with the real one directly above it.
            font = theme.font_for(theme.FOOTNOTE, theme.W_MEDIUM, tracking=6)
            painter.setFont(font)
            painter.setPen(ACCENT)
            painter.drawText(self._subtitle_rect, Qt.AlignCenter, self._subtitle)

        # Caption. The quietest thing on the screen, and the only one
        # that changes while it is up.
        painter.setFont(theme.font_for(theme.CAPTION1))
        painter.setPen(MUTED)
        painter.drawText(self._status_rect, Qt.AlignCenter, self._status)

    def keyPressEvent(self, event):
        # The backdrop must survive every keystroke: closing it is closing the
        # only window in that process, which would take the logo layer away for
        # the rest of the session.
        if self._backdrop:
            super().keyPressEvent(event)
            return
        # Never trap the operator behind the splash if a camera hangs.
        if event.key() in (Qt.Key_Escape, Qt.Key_Q, Qt.Key_Space, Qt.Key_Return):
            if self.on_skip is not None:
                self.on_skip()
            else:
                self.finish()
        else:
            super().keyPressEvent(event)


def main():
    """Preview the splash on its own:  python3 splash.py"""
    import sys
    from PySide6.QtWidgets import QApplication

    import config

    app = QApplication(sys.argv)
    splash = SplashScreen(config.LOGO_PATH)
    splash.show_on_primary(app)
    splash.set_status("connecting to cameras…")
    QTimer.singleShot(6000, app.quit)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
