#!/usr/bin/env python3
"""
The logo layer that sits behind everything, for the whole session.

Without it the screen falls back to the bare X root in every gap, and there are
three of them:

    1. X starts, the viewer has not imported cv2 yet          (~3s at boot)
    2. the viewer is restarted by .xinitrc's supervise loop   (~5s, any time)
    3. the viewer crashed and is between retries              (however long)

A solid-colour root makes those read as a dead machine; this makes them read as
"still starting". It draws the logo on the same gradient as the Plymouth boot
theme and the app's own loading screen, so all three are the same picture and
the handovers are invisible.

Started once from .xinitrc, before the viewer, and never exits. Mapping order is
what puts it at the bottom: there is no window manager on the ground station, so
the first window mapped stays below every window mapped after it.

Preview it (Ctrl-C to stop):

    python3 backdrop.py
"""

import sys

from PySide6.QtWidgets import QApplication

import config
from splash import SplashScreen


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Arnobot Backdrop")

    backdrop = SplashScreen(config.LOGO_PATH, backdrop=True)
    backdrop.show_on_primary(app)

    # Plymouth is still holding the framebuffer at this point on a real boot
    # (setup_splash.sh masks plymouth-quit so the logo survives until X exists).
    # .xinitrc quits it once this window is up — see the comment there.
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
