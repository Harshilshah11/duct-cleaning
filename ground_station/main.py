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

Recording is on the panel switches, not the keyboard (see recorder.py):

    switch 1, GPIO24 ... START / STOP
    switch 2, GPIO23 ... PAUSE / RESUME, inside the same file
    button,   GPIO25 ... SAVE - bank the clip so far and keep rolling

Keys:
    F        fullscreen toggle        S       snapshot both panels
    R        reconnect both           1/2     solo a camera, 0 = both
    Q / Esc  quit
    Space    start/stop  P  pause     Ctrl+S  save clip   (bench only - these
             do nothing while the panel switches are readable)
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
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

# How old the ANALOG sample may be before the stick is treated as unknown.
#
# Sized off the measured analog poll rate, not guessed: inputs.py bit-bangs three
# ADS channels in ~32 ms and is capped at 25 Hz, so a healthy sample is never more
# than ~40 ms old and 0.4 s is ten poll periods of slack. Comfortably above any
# real jitter on a Pi that is also decoding two camera streams, and far below the
# point where a driving robot has gone anywhere - the wheels stop within half a
# second of the reader going quiet. See the block in tick() for why this cannot be
# left to MotorLink's own SAMPLE_STALE_S.
ANALOG_STALE_S = float(os.environ.get("ANALOG_STALE_S", "0.4"))

SANS = "DejaVu Sans"
# One typeface across the app. The panel headers and the NO SIGNAL placeholder
# used the terminal mono; nothing here is tabular data, so it only ever read as
# a console that had been given a logo.
MONO = SANS

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
LinkMonitor = RTSPStream = SessionManager = describe_backends = TopBar = None


def _load_video_stack():
    """Import the heavy half of the app. Idempotent."""
    global cv2, LinkMonitor, RTSPStream, SessionManager, describe_backends, TopBar
    import cv2
    from link import LinkMonitor
    from recorder import SessionManager
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
                stall_timeout=config.READ_STALL_TIMEOUT_S,
                reconnect_max_delay=config.RECONNECT_MAX_DELAY_S,
                connect_stagger=config.CONNECT_STAGGER_S,
                session_wait=config.SESSION_WAIT_S,
                unreachable_max_delay=config.UNREACHABLE_MAX_DELAY_S,
                rotate=config.rotate_for(index),
            )
            for index, (name, url) in enumerate(cameras)
        ]
        # Filename identity, distinct from the display name: 'CAM 1 · FRONT'
        # on screen is cam1_front on disk. SessionManager reads .slug.
        for index, stream in enumerate(self.streams):
            stream.slug = config.camera_slug(index)
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

        # --- recording -------------------------------------------------------
        # Driven entirely by the panel switches via inputs.py: switch 1 (GPIO24)
        # starts and stops, switch 2 (GPIO23) pauses and resumes inside the same
        # file, and the GPIO25 button banks the clip so far and keeps rolling.
        # The encoders read the SAME decoded frames the panels are drawing, so
        # recording opens no second RTSP session and what lands on disk is what
        # the operator saw. See recorder.py.
        self.session = SessionManager(self.streams)

        # Keyboard fallback, used only when the reader has no switch state to
        # give (no gpiod, pins busy, INPUTS_ENABLED=0). Letting the keys fight a
        # live switch would mean the panel showing STOPPED while a file grows.
        self._kbd_session = None

        # The USB backup daemon's status file, re-read at most once a second.
        # A missing or stale file is simply "no transfer" - the daemon is a
        # separate root process and the viewer must not care whether it is up.
        self._usb = {}
        self._usb_read_at = 0.0

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

        # Always-on correlation log: what the motors were asked to do, next to
        # what each camera was doing, once a second.
        #
        # This exists because the interesting failure - "the cameras stall while
        # I drive" - only happens with a human on the stick, and a diagnostic
        # that needs the operator and the investigator online simultaneously
        # never gets captured. Logging it continuously means the next drive
        # records its own evidence. Reads MotorLink's published state rather
        # than sniffing the wire, so it needs no root and no tcpdump.
        self.corr_timer = QTimer(self)
        self.corr_timer.timeout.connect(self.log_correlation)
        self.corr_timer.start(1000)

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
        # Recording, for a bench with no panel wired to it. Ignored the moment
        # the switches are readable — see _session_state().
        bind("Space", self.kbd_start_stop)
        bind("P", self.kbd_pause)
        bind("Ctrl+S", self.session_save)

    # -- recording -------------------------------------------------------------

    def kbd_start_stop(self):
        self._kbd_session = (None if self._kbd_session in ("RECORDING", "PAUSED")
                             else "RECORDING")

    def kbd_pause(self):
        if self._kbd_session == "RECORDING":
            self._kbd_session = "PAUSED"
        elif self._kbd_session == "PAUSED":
            self._kbd_session = "RECORDING"

    def session_save(self):
        # Bench-only key, so it stands in for the whole gesture: a held key
        # does not deliver a measurable 3s level the way GPIO25 does, so in the
        # post-stop window Ctrl+S claims the recording directly.
        if not self.session.finalize():
            self.session.save_clip()

    def _usb_status(self):
        """The backup daemon's published state, or {} when there is none.

        Re-read at most once a second - it is a tmpfs file, but 30 reads a
        second for a value that changes every two is still noise. A file whose
        daemon has stopped stamping it is treated as absent rather than shown:
        a strip stuck on "transferring" after the daemon died would have the
        operator waiting on a copy that is not happening.
        """
        now = time.monotonic()
        if now - self._usb_read_at >= 1.0:
            self._usb_read_at = now
            try:
                with open(config.USB_STATUS_PATH, encoding="utf-8") as fh:
                    usb = json.load(fh) or {}
            except (OSError, ValueError):
                usb = {}
            if time.time() - (usb.get("updated") or 0) > 10.0:
                usb = {}
            self._usb = usb
        return self._usb

    def _session_state(self, snapshot):
        """Switch state if the panel is readable, otherwise the keyboard latch.

        One source at a time and the hardware always wins: two of them would let
        a key press and a lever disagree, and the whole point of the strip is
        that it shows what the recorder is actually doing.
        """
        state = snapshot.get("session")
        return state if state is not None else (self._kbd_session or "STOPPED")

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

        # The chip reports a real measurement and nothing else - no inferring a
        # healthy robot from healthy video, because they are different machines
        # on different addresses.
        #
        # That measurement is now the Uno command link, not LinkMonitor's TCP
        # probe. LinkMonitor still points at ROBOT_LINK_HOST:PORT, which defaults
        # to the guide's robot-Pi RTSP port (192.168.1.30:8554) - a host that
        # does not exist on this rig, so the chip read DISCONNECTED forever even
        # with the Arduino answering every packet. The Arduino link is UDP, so it
        # cannot be probed by connecting a TCP socket to it either; the honest
        # test is whether it is ACKing the commands we send, which MotorLink
        # already tracks over a rolling window (see uno_motors.ACK_MIN_PCT).
        # If ROBOT_LINK_HOST is ever pointed at something real that speaks TCP,
        # that probe counts too - either being up means the robot is reachable.
        motor_state = self.motors.latest() or {}
        self.topbar.set_robot(bool(motor_state.get("ok")) or self.link.connected)
        self.topbar.set_clock(datetime.now().strftime("%H:%M:%S"))

        # Same push model as the top bar - latest() is a cheap dict copy off the
        # reader thread, so this costs nothing at the UI frame rate. Taken once
        # and shared: the panel and the motors must act on the SAME sample, or
        # the strip can show a stick position the wheels never got.
        snapshot = self.inputs.latest()

        # Recording before the panel is told anything: on_inputs() applies the
        # switch state and the save-button edge count, so the status the strip
        # renders this frame is the one the recorder is already acting on rather
        # than one frame stale.
        self.session.set_state(self._session_state(snapshot))
        self.session.on_save_button(snapshot.get("save_presses"))
        # The hold level as well as the press edges: holding SAVE for 3s after
        # a stop is what finalizes the recording into /recordings.
        self.session.on_save_hold(snapshot.get("save_held_s"))
        # Expires the post-stop confirm window, which can delete the recording.
        # After the save button and the hold, so a press or a completed hold
        # landing on the last frame of the window is honoured rather than raced.
        self.session.poll()
        status = self.session.status()
        # The USB transfer rides in the same status dict the strip renders, so
        # "data is transferring" appears exactly where the operator already
        # looks for recording state.
        status["usb"] = self._usb_status()
        # Drawing the strip must never be able to strand the motors. Everything
        # below this block is the demand the Uno keeps transmitting at 50Hz, so
        # an exception raised in here used to skip the rest of tick() and leave
        # MotorLink repeating the LAST demand forever - the wheels kept driving
        # on a stale stick position. That is not hypothetical: a partial ADC read
        # left joy['y'] None and inputs_panel's f-string raised on every frame.
        # The panel is display; the motor demand below is safety. Never let the
        # first take out the second.
        try:
            self.inputs_panel.set_state(snapshot, status)
            self.topbar.set_recording(status)
        except Exception:
            traceback.print_exc()

        # Stick -> wheels. This only refreshes the demand; MotorLink keeps
        # transmitting at its own 50 Hz regardless, so a slow UI frame can never
        # starve the Uno into tripping its 300 ms failsafe. With no ADC the axes
        # are None, which mixes to a dead stop rather than a stale demand.
        #
        # THE ANALOG SAMPLE'S OWN AGE IS CHECKED HERE, and it has to be. This push
        # is what resets MotorLink's staleness timer, so pushing a value that the
        # ADC thread stopped updating would re-assert the last stick position as
        # "fresh" 30 times a second - the wheels would keep driving on a reading
        # nobody had taken since the reader died. Past the limit the axes are
        # forced to None, which mixes to a dead stop. inputs.py stamps
        # adc_updated on every pass including all-rejected ones, so this fires
        # only for a reader that has genuinely stopped, not for a noisy one.
        joy = snapshot.get("joy") or {}
        pot_pct = (snapshot.get("pot") or {}).get("pct")
        adc_age = time.time() - (snapshot.get("adc_updated") or 0.0)
        if adc_age > ANALOG_STALE_S:
            joy, pot_pct = {}, None
        self.motors.set_joystick(joy.get("x"), joy.get("y"))

        # Actuator switch -> rod direction. The pot argument is vestigial: the
        # rod has no speed input since D5 became the light, and act_demand()
        # ignores it. Deliberately the SAME snapshot as the stick above, so the
        # panel can never show a switch position the actuator did not get.
        #
        # NOT gated on adc_age: the rod is driven by the SWITCH pins, which are on
        # their own thread and keep working with a dead ADC. Stopping the rod
        # because an unrelated analog bus wedged would be a failsafe firing on the
        # wrong evidence.
        self.motors.set_actuator(snapshot.get("actuator"), pot_pct)
        # Toggle -> brush, on/off at full scale. The brush briefly took its
        # speed from the pot (2026-08-17, one evening): one knob feeding two
        # mechanisms meant dimming the lamp also slowed the brush, and a knob
        # at zero made an armed brush look broken. See brush_demand().
        self.motors.set_brush((snapshot.get("switches") or {}).get("BRUSH"))
        # Pot -> light brightness, which is the knob's ONLY job again.
        self.motors.set_light(pot_pct)

    def log_correlation(self):
        """One line a second pairing motor demand with camera state.

        Never allowed to raise: this is instrumentation, and instrumentation
        that can take down the viewer is worse than no instrumentation.
        """
        try:
            m = self.motors.latest() or {}
            snap = self.inputs.latest()
            joy = snap.get("joy") or {}
            pot = snap.get("pot") or {}
            rej = snap.get("adc_rejects") or {}
            parts = [
                time.strftime("%H:%M:%S"),
                f"L={m.get('left') or 0:+4d}",
                f"R={m.get('right') or 0:+4d}",
                f"act={m.get('act') or 0:+2d}",
                f"brush={m.get('brush') or 0}",
                f"uno={'ok' if m.get('ok') else 'DOWN'}",
            ]
            # THE RAW ADC COUNTS, and they are the point of this addition. A line
            # reading `L=+237 R=+237` is indistinguishable from an operator pushing
            # the stick forward unless the counts behind it are recorded too: three
            # channels sitting on the same value is a dead bus, whereas three
            # unrelated values is a hand. Diagnosing the phantom demand of
            # 2026-08-17 needed exactly this and had to infer it from the wire.
            def _c(v):
                return "----" if v is None else f"{v:5d}"
            parts.append(f"raw=x{_c(joy.get('x_raw'))},y{_c(joy.get('y_raw'))}"
                         f",p{_c(pot.get('raw'))}")
            parts.append(f"adc={snap.get('adc_hz') or 0:.1f}Hz")
            # Only when non-zero, so a healthy run does not carry four counters
            # that never move and the ones that DO move are conspicuous.
            fired = " ".join(f"{k}{v}" for k, v in sorted(rej.items()) if v)
            if fired:
                parts.append(f"rej={fired}")
            for s in self.streams:
                tag = s.name.replace(" ", "")
                parts.append(
                    f"{tag}={'up' if s.connected else 'DOWN'}"
                    f"/{s.fps:.0f}fps/r{s.reconnects}"
                )
            path = os.path.expanduser(
                os.environ.get("MOTOR_CAM_LOG", "~/motor_cam.log"))
            # Cap it rather than filling the SD card - a full root filesystem
            # takes X, the viewer and any ssh session with it.
            try:
                if os.path.getsize(path) > 5_000_000:
                    os.replace(path, path + ".1")
            except OSError:
                pass
            # buffering=1 (line buffered) and a flush, because this log was found
            # holding 1580 NUL bytes mid-file on 2026-08-17. That is the ext4
            # delayed-allocation signature of an unclean exit: the inode's size had
            # been committed but the data blocks had not, so the gap reads back as
            # zeros. It matters beyond tidiness - a single NUL makes grep treat the
            # whole file as binary and silently refuse to print matches, which is
            # how a correlator nobody can grep stops being a correlator.
            with open(path, "a", encoding="utf-8", buffering=1) as fh:
                fh.write(" ".join(parts) + "\n")
                fh.flush()
        except Exception:
            pass

    def refresh_temp(self):
        self.topbar.set_temp(thermal.read_c())

    def closeEvent(self, event):
        self.timer.stop()
        self.temp_timer.stop()
        self.link.stop()
        # First of the workers to go: it is the only one holding a file that is
        # invalid until it is closed properly, and it needs the streams it is
        # reading from to still exist while it finishes.
        self.session.stop()
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
        [(config.camera_name(i), url) for i, url in enumerate(urls)]
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
