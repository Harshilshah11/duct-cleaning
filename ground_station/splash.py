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
from PySide6.QtWidgets import QWidget

# Sampled from the logo, so the chrome around it never clashes. The field is
# light because the logo is dark navy — on the app's near-black background it
# would be all but invisible.
BG_TOP = QColor("#ffffff")
BG_BOTTOM = QColor("#e8edf6")
INK = QColor("#241f7a")        # the dark navy of "ARNOBOT"
ACCENT = QColor("#3f6fb5")     # the mid blue of the "R"
MUTED = QColor("#8590ab")

FONT = "DejaVu Sans"


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
        self._status = "" if backdrop else "starting…"
        self._phase = 0

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
        self._dots_rect = QRect()
        self._status_rect = QRect()
        self._scaled = None

        # No animation on the backdrop: it lives for the whole session, and a
        # repaint every 90ms forever is a wakeup the Pi does not need to pay for
        # something nobody is watching. The loading screen keeps its dots.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._advance)
        if not backdrop:
            self._timer.start(90)

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

    def _advance(self):
        self._phase += 1
        self.update(self._dots_rect)

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
        self._dots_rect = QRect(0, self._subtitle_rect.bottom() + int(h * 0.06), w, 24)
        self._status_rect = QRect(0, self._dots_rect.bottom() + int(h * 0.035), w, 26)
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
            painter.setFont(QFont(FONT, 44, QFont.Bold))
            painter.drawText(self._logo_rect, Qt.AlignCenter, "ARNOBOT")

        if self._backdrop:
            return          # logo only — matches the Plymouth theme exactly

        if self._subtitle:
            font = QFont(FONT, 13, QFont.Bold)
            font.setLetterSpacing(QFont.AbsoluteSpacing, 6)
            painter.setFont(font)
            painter.setPen(ACCENT)
            painter.drawText(self._subtitle_rect, Qt.AlignCenter, self._subtitle)

        self._paint_dots(painter)

        painter.setFont(QFont(FONT, 11))
        painter.setPen(MUTED)
        painter.drawText(self._status_rect, Qt.AlignCenter, self._status)

    def _paint_dots(self, painter):
        count = 3
        radius = 5
        gap = 26
        cx = self.width() // 2 - (count - 1) * gap // 2
        cy = self._dots_rect.center().y()

        painter.setPen(Qt.NoPen)
        for i in range(count):
            # Each dot lights in turn — a plain travelling highlight, cheap to
            # repaint and obviously alive even if a stream never connects.
            lit = (self._phase // 3) % count == i
            colour = QColor(INK if lit else MUTED)
            colour.setAlpha(255 if lit else 90)
            painter.setBrush(colour)
            grow = 2 if lit else 0
            painter.drawEllipse(
                cx + i * gap - radius - grow,
                cy - radius - grow,
                2 * (radius + grow),
                2 * (radius + grow),
            )

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
