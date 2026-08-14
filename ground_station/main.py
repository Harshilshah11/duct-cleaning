#!/usr/bin/env python3
"""
Ground Station — dual RTSP test viewer (PySide6).

A deliberately simple bring-up UI: a status top bar (logo, per-camera chips,
robot link, clock — see topbar.py) over two side-by-side camera panels that
reconnect on their own. No joystick / no command sending yet — this exists
purely to prove both RTSP streams decode on the ground station Pi before you
wire in Steps 5-9 of the guide.

Run:
    python3 main.py
    python3 main.py rtsp://192.168.1.30:8554/cam1 rtsp://192.168.1.30:8554/cam2

On Windows, launch run_ground_station.pyw instead — same app, no console window.

Keys:
    F        fullscreen toggle        S   snapshot both panels
    R        reconnect both           1/2 solo a camera, 0 = both
    Q / Esc  quit
"""

from __future__ import annotations

import os
import sys
import time
from datetime import datetime

from PySide6.QtCore import Qt, QRectF, QTimer
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPixmap, QShortcut, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import config
import inputs
import thermal
from inputs_panel import InputsPanel
from splash import SplashScreen
from uno_motors import MotorLink

MONO = "DejaVu Sans Mono"

# cv2, link, stream and topbar are NOT imported here — _load_video_stack() does
# it once main() has the logo on the screen. cv2 alone is ~2s of import on a Pi 4
# (numpy, then the whole FFMPEG/GStreamer stack), and every one of those seconds
# is spent before Qt can paint anything: at module scope the operator watches a
# bare console or a black X root instead of the splash. Importing them from a
# function binds the same module-level names, so the classes below are unchanged.
#
# `from __future__ import annotations` above is what keeps that honest — without
# it, CameraPanel's `stream: RTSPStream` annotation would be evaluated at class
# definition time, i.e. before the import has happened.
cv2 = None
LinkMonitor = RTSPStream = describe_backends = TopBar = None


def _load_video_stack():
    """Import the heavy half of the app. Idempotent."""
    global cv2, LinkMonitor, RTSPStream, describe_backends, TopBar
    import cv2
    from link import LinkMonitor
    from stream import RTSPStream, describe_backends
    from topbar import TopBar


class VideoCanvas(QWidget):
    """Draws the newest frame, letterboxed, or a NO SIGNAL placeholder."""

    def __init__(self):
        super().__init__()
        self._pixmap = None
        self._message = "CONNECTING..."
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_pixmap(self, pixmap):
        self._pixmap = pixmap
        self.update()

    def clear(self, message="NO SIGNAL"):
        self._pixmap = None
        self._message = message
        self.update()

    def _blit_rects(self, pixmap):
        """(source, target) rectangles for config.VIDEO_ZOOM. None if unpaintable.

        Scaling straight to self.size() with KeepAspectRatio is the zoom==0 case
        and leaves a third of the panel black; KeepAspectRatioByExpanding is
        zoom==1 and throws away the sides of the frame. This walks between the
        two, and hands Qt only the part of the frame that will be visible — one
        scaled blit of the cropped region instead of scaling the whole frame up
        and then clipping most of it away, which matters on the Pi's CPU.
        """
        pw, ph = pixmap.width(), pixmap.height()
        vw, vh = self.width(), self.height()
        if not (pw and ph and vw and vh):
            return None

        fit = min(vw / pw, vh / ph)          # whole frame inside the panel
        cover = max(vw / pw, vh / ph)        # panel completely covered
        scale = min(fit / max(1e-3, 1.0 - config.VIDEO_ZOOM), cover)

        src_w, src_h = min(pw, vw / scale), min(ph, vh / scale)
        source = QRectF((pw - src_w) / 2, (ph - src_h) / 2, src_w, src_h)
        out_w, out_h = src_w * scale, src_h * scale
        target = QRectF((vw - out_w) / 2, (vh - out_h) / 2, out_w, out_h)
        return source, target

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#05080b"))

        rects = (
            self._blit_rects(self._pixmap)
            if self._pixmap is not None and not self._pixmap.isNull()
            else None
        )
        if rects is not None:
            # SmoothPixmapTransform stays off — the Pi 4 CPU is doing the
            # decode too, and this runs once per panel per frame.
            source, target = rects
            painter.drawPixmap(target, self._pixmap, source)
        else:
            painter.setPen(QColor("#e0564a"))
            painter.setFont(QFont(MONO, 13, QFont.Bold))
            painter.drawText(self.rect(), Qt.AlignCenter, self._message)


class CameraPanel(QWidget):
    """Title strip (status dot + name) over the video canvas, for one camera."""

    def __init__(self, stream: RTSPStream):
        super().__init__()
        self.stream = stream
        self._last_seq = -1

        self.name_label = QLabel(stream.name)
        self.name_label.setObjectName("panelName")

        self.dot = QLabel("●")
        self.dot.setObjectName("dot")

        # Nothing else goes in here. Resolution, frame rate and backend are
        # bring-up numbers, not operating ones: they never change while the
        # stream is up, and the top bar's chip already carries the two things
        # that do (live / not, and how fast). Read them from the console with
        # `python3 check_streams.py` when you actually need them.
        header = QHBoxLayout()
        header.setContentsMargins(10, 4, 10, 4)
        header.setSpacing(8)
        header.addWidget(self.dot)
        header.addWidget(self.name_label)
        header.addStretch(1)

        header_widget = QWidget()
        header_widget.setObjectName("panelHeader")
        header_widget.setLayout(header)

        self.canvas = VideoCanvas()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(header_widget)
        layout.addWidget(self.canvas, 1)

        self.setObjectName("panel")

    def refresh(self):
        """Pull the newest frame (if any) and update the status line."""
        frame, seq = self.stream.latest()

        if frame is not None and seq != self._last_seq:
            self._last_seq = seq
            h, w = frame.shape[:2]
            # Format_BGR888 consumes OpenCV's native byte order — no cvtColor needed.
            # .copy() detaches the QImage from the numpy buffer, which is about to
            # be replaced by the capture thread.
            image = QImage(frame.data, w, h, frame.strides[0], QImage.Format_BGR888).copy()
            self.canvas.set_pixmap(QPixmap.fromImage(image))
        elif not self.stream.connected:
            self.canvas.clear(self.stream.status_text.upper())

        connected = self.stream.connected
        self.dot.setStyleSheet(f"color: {'#38c172' if connected else '#e0564a'};")


class GroundStationWindow(QWidget):
    def __init__(self, cameras):
        super().__init__()
        self.setWindowTitle(f"Arnobot — {config.TITLE}")
        self.resize(1280, 620)

        self.streams = [
            RTSPStream(
                name=name,
                url=url,
                latency_ms=config.RTSP_LATENCY_MS,
                protocols=config.RTSP_PROTOCOL,
                hw_decode=config.USE_HW_DECODE,
                reconnect_delay=config.RECONNECT_DELAY_S,
                read_fail_limit=config.READ_FAIL_LIMIT,
            )
            for name, url in cameras
        ]
        self.panels = [CameraPanel(s) for s in self.streams]

        # --- top bar --------------------------------------------------------
        # Hand it the camera list rather than letting it read config: these URLs
        # may have come from cameras.txt or argv, and each chip's tooltip has to
        # show the stream it is ACTUALLY pulling from.
        self.topbar = TopBar(cameras, config.LOGO_PATH, config.TITLE)

        # Separate question, separate answer: the cameras are standalone IP
        # cameras, so live video proves nothing about the robot and this has to
        # be probed on its own. See link.py.
        self.link = LinkMonitor(
            config.ROBOT_LINK_HOST,
            config.ROBOT_LINK_PORT,
            interval=config.LINK_POLL_S,
            timeout=config.LINK_TIMEOUT_S,
        )
        self.link.start()

        # --- operator controls ----------------------------------------------
        # Local hardware, not the robot: the switches, joystick and pot are
        # wired to THIS Pi. The reader owns the GPIO/ADS lines on its own thread
        # and never raises at us, so a missing chip or a busy pin degrades to a
        # message in the strip instead of taking the video viewer down with it.
        self.inputs = inputs.InputReader()
        self.inputs.start()
        self.inputs_panel = InputsPanel()

        # --- motor tether ----------------------------------------------------
        # The Uno is on the eth0 wire at 192.168.50.20, reached over UDP. Same
        # contract as InputReader: its thread owns the socket and never raises
        # at us, so an absent Uno degrades to a dead link instead of taking the
        # viewer down. UDP sends never fail loudly, so the link's health comes
        # from whether ACKs come back, not from whether send() threw.
        self.motors = MotorLink()
        self.motors.start()

        # --- video row ------------------------------------------------------
        video_row = QHBoxLayout()
        video_row.setContentsMargins(10, 10, 10, 10)
        video_row.setSpacing(10)
        for panel in self.panels:
            video_row.addWidget(panel, 1)

        # No footer: every action it held has a key, the keys are listed in the
        # module docstring, and on the ground station there is no mouse in the
        # first place. Removing it also gives the panels the full window below
        # the bar, which is the only thing an operator is actually looking at.
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.topbar)
        root.addLayout(video_row, 1)
        # Below the video: the operator looks up at the picture and down at the
        # controls, and the strip is fixed-height so it never steals from the
        # panels as the window grows.
        root.addWidget(self.inputs_panel)

        self.setStyleSheet(STYLESHEET)
        self._install_shortcuts()

        for stream in self.streams:
            stream.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(max(1, int(1000 / config.UI_FPS)))

        # The SoC temperature gets its own slow timer rather than riding tick():
        # silicon takes seconds to change, and there is no reason to touch sysfs
        # 30 times a second. Read once up front so the chip is populated before
        # the first interval elapses instead of showing a dash for two seconds.
        self.temp_timer = QTimer(self)
        self.temp_timer.timeout.connect(self.refresh_temp)
        self.temp_timer.start(max(200, int(config.TEMP_POLL_S * 1000)))
        self.refresh_temp()

    # -- actions -------------------------------------------------------------

    def _install_shortcuts(self):
        def bind(seq, slot):
            QShortcut(QKeySequence(seq), self, activated=slot)

        bind("F", self.toggle_fullscreen)
        bind("F11", self.toggle_fullscreen)
        bind("S", self.snapshot)
        bind("R", self.reconnect_all)
        bind("Q", self.close)
        bind("Esc", self.close)
        bind("1", lambda: self.solo(0))
        bind("2", lambda: self.solo(1))
        bind("0", lambda: self.solo(None))

    def solo(self, index):
        for i, panel in enumerate(self.panels):
            panel.setVisible(index is None or i == index)

    def toggle_fullscreen(self):
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def reconnect_all(self):
        for stream in self.streams:
            stream.reconnect()

    def snapshot(self):
        # There is no status line left to report into, so this goes to stdout —
        # visible when the app was started from a terminal, silently dropped
        # under pythonw. The camera chips going green is the feedback that
        # matters; whether a PNG landed is something you check afterwards.
        os.makedirs(config.SNAPSHOT_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        saved = 0
        for stream in self.streams:
            frame, _ = stream.latest()
            if frame is None:
                continue
            slug = stream.name.lower().replace(" ", "")
            path = os.path.join(config.SNAPSHOT_DIR, f"{stamp}_{slug}.png")
            if cv2.imwrite(path, frame):
                saved += 1
        print(f"snapshot: saved {saved} file(s) to {config.SNAPSHOT_DIR}")

    # -- main loop -----------------------------------------------------------

    def tick(self):
        for index, panel in enumerate(self.panels):
            panel.refresh()
            stream = panel.stream
            self.topbar.set_camera(
                index, stream.connected, stream.fps, stream.status_text
            )

        # The chip reports what the probe found and nothing else - no inferring
        # a healthy robot from healthy video, because they are now different
        # machines on different addresses.
        self.topbar.set_robot(self.link.connected)
        self.topbar.set_clock(datetime.now().strftime("%H:%M:%S"))

        # Same push model as the top bar - latest() is a cheap dict copy off the
        # reader thread, so this costs nothing at the UI frame rate. Taken once
        # and shared: the panel and the motors must act on the SAME sample, or
        # the strip can show a stick position the wheels never got.
        snapshot = self.inputs.latest()
        self.inputs_panel.set_state(snapshot)

        # Stick -> wheels. This only refreshes the demand; MotorLink keeps
        # transmitting at its own 50 Hz regardless, so a slow UI frame can never
        # starve the Uno into tripping its 300 ms failsafe. With no ADC the axes
        # are None, which mixes to a dead stop rather than a stale demand.
        joy = snapshot.get("joy") or {}
        self.motors.set_joystick(joy.get("x"), joy.get("y"))

        # Actuator switch -> rod direction. The pot argument is vestigial: the
        # rod has no speed input since D5 became the light, and act_demand()
        # ignores it. Deliberately the SAME snapshot as the stick above, so the
        # panel can never show a switch position the actuator did not get.
        self.motors.set_actuator(snapshot.get("actuator"),
                                 (snapshot.get("pot") or {}).get("pct"))
        self.motors.set_brush((snapshot.get("switches") or {}).get("TOGGLE"))
        # Pot -> light brightness, which is what that knob actually does now.
        self.motors.set_light((snapshot.get("pot") or {}).get("pct"))

    def refresh_temp(self):
        self.topbar.set_temp(thermal.read_c())

    def closeEvent(self, event):
        self.timer.stop()
        self.temp_timer.stop()
        self.link.stop()
        # Motors before inputs: stopping the link sends a final STOP, and it
        # should go out while the app is still otherwise alive rather than
        # racing the rest of the teardown.
        self.motors.stop()
        self.inputs.stop()
        for stream in self.streams:
            stream.stop()
        event.accept()


STYLESHEET = f"""
QWidget {{
    background: #0b0f14;
    color: #d7e0ea;
    font-family: "{MONO}", monospace;
    font-size: 12px;
}}
#panel  {{ background: #05080b; border: 1px solid #1e2a38; border-radius: 4px; }}
#panelHeader {{ background: #111823; border-bottom: 1px solid #1e2a38; }}
#panelName  {{ font-weight: bold; letter-spacing: 1px; }}
#dot    {{ font-size: 12px; color: #e0564a; }}
"""


def show_window(window, app):
    if not config.START_FULLSCREEN:
        window.show()
        return

    window.showFullScreen()
    # showFullScreen() only sets the _NET_WM_STATE_FULLSCREEN hint, and on the
    # ground station there is NO window manager to act on it - the window just
    # stays at its requested size in the corner. Force the geometry to the
    # screen rectangle so it fills the display either way.
    screen = app.primaryScreen()
    if screen is not None:
        window.setGeometry(screen.geometry())


def main():
    # Optional: pass one or two RTSP URLs on the command line.
    urls = sys.argv[1:]

    app = QApplication(sys.argv[:1])
    app.setApplicationName("Arnobot Ground Station")

    # Logo up FIRST — ahead of the cv2 import, the capture threads and the
    # window, which is everything slow. show_on_primary() ends in
    # processEvents(), so the splash is actually painted before the next line
    # blocks; from here to hand_over() there is always something branded on the
    # screen and never a console or a bare root window.
    splash = SplashScreen(config.LOGO_PATH) if config.SPLASH_ENABLED else None
    if splash is not None:
        splash.show_on_primary(app)
        splash.set_status("loading video pipeline…")

    _load_video_stack()

    cameras = (
        [(f"CAM {i + 1}", url) for i, url in enumerate(urls)]
        if urls else config.CAMERAS
    )

    # Goes nowhere under pythonw.exe (no console, sys.stdout is None, and print()
    # is a documented no-op in that case) — which is the point of the .pyw
    # launcher. Run main.py from a terminal when you want to read this.
    print(describe_backends())
    for name, url in cameras:
        print(f"  {name}: {url}")

    if splash is None:
        window = GroundStationWindow(cameras)
        show_window(window, app)
        sys.exit(app.exec())

    splash.set_status("starting video pipeline…")
    window = GroundStationWindow(cameras)
    total = len(window.streams)
    started = time.monotonic()

    def hand_over():
        if not poll.isActive():
            return          # already handed over (Esc raced the timer)
        poll.stop()
        show_window(window, app)
        # Mapped after the splash and with no window manager to restack them,
        # so the viewer is already on top by the time the splash goes.
        splash.finish(window)

    def poll_ready():
        waited = time.monotonic() - started
        live = sum(1 for s in window.streams if s.connected)
        splash.set_status(
            f"{live}/{total} camera{'s' if total != 1 else ''} live"
            if live else "connecting to cameras…"
        )
        if waited < config.SPLASH_MIN_S:
            return
        if (live == total
                or (live and waited >= config.SPLASH_PARTIAL_S)
                or waited >= config.SPLASH_MAX_S):
            hand_over()

    poll = QTimer()
    poll.timeout.connect(poll_ready)
    poll.start(150)
    splash.on_skip = hand_over      # Esc / Space skips the wait

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
