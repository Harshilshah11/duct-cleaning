#!/usr/bin/env python3
"""
Windows entry point — double-click this instead of main.py.

Windows binds the .pyw extension to pythonw.exe, which is the same interpreter
built without a console. Launching main.py with plain python.exe opens a black
console window first and leaves it there for the life of the app; with this
launcher the Arnobot splash is the first and only thing that appears, and the
viewer window replaces it. Nothing else changes — main() is the same function,
and any RTSP URLs given here are passed straight through to it.

    run_ground_station.pyw
    run_ground_station.pyw rtsp://127.0.0.1:8554/cam1 rtsp://127.0.0.1:8554/cam2

Fullscreen is off by default here, unlike the Pi: this is a desktop with a
window manager and a taskbar, and the Pi's reason for forcing the geometry (no
WM to act on showFullScreen) does not apply. Press F for fullscreen, or set
START_FULLSCREEN=1 in the environment to have it start that way.

On the Pi nothing here is used — .xinitrc runs main.py directly, and Plymouth
covers the boot before X even exists.
"""

import os
import sys

os.environ.setdefault("START_FULLSCREEN", "0")

HERE = os.path.dirname(os.path.abspath(__file__))
CRASH_LOG = os.path.join(HERE, "crash.log")


def _log_crash(exc_type, exc, tb):
    """Write the traceback somewhere it can actually be read.

    The whole point of pythonw.exe is that there is no console — which also
    means sys.stderr is None, so the interpreter's own "print the traceback and
    exit" has nowhere to go. Without this, a crash looks exactly like a clean
    quit: the window vanishes and nothing is left behind to explain it. Qt's own
    fatal errors still bypass this, so run `python main.py` in a terminal if the
    log stays empty.

    Not installed for SystemExit — sys.excepthook is never called for it, which
    is what makes main()'s sys.exit(app.exec()) a normal quit rather than a
    logged crash.
    """
    import traceback
    from datetime import datetime

    try:
        with open(CRASH_LOG, "a", encoding="utf-8") as fh:
            fh.write(f"\n{'=' * 70}\n{datetime.now():%Y-%m-%d %H:%M:%S}\n")
            traceback.print_exception(exc_type, exc, tb, file=fh)
    except OSError:
        pass


sys.excepthook = _log_crash

# Python already puts this file's directory on sys.path, so `import main` finds
# the app whatever directory the shortcut was launched from. config.py resolves
# cameras.txt and the logo relative to its own file for the same reason.
from main import main

if __name__ == "__main__":
    sys.exit(main() or 0)
