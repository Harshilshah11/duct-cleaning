#!/usr/bin/env python3
"""
Recording: panel switches -> video files on disk.

Same split as stream.py / inputs.py - this module owns the encoders and their
threads, main.py owns the single UI timer and pushes state in. Nothing here
touches Qt.

WHAT DRIVES IT (inputs.py decodes the pins, this acts on the decode):

    switch 1, red leg, GPIO24 .... START / STOP
    switch 2, green leg, GPIO23 .. PAUSE / RESUME
    button,   GPIO25 ............. SAVE - cut the current clip, keep rolling

WHY IT RE-ENCODES THE DECODED FRAMES rather than copying the RTSP stream with
ffmpeg, which would be nearly free:

  * These cameras are cheap IP units with a small cap on concurrent RTSP
    sessions. A second connection per camera, opened exactly when the operator
    starts recording, is the one moment you cannot afford the viewer to lose its
    picture - and that is the failure it would produce.
  * What lands on disk is then provably what the operator saw, including the
    reconnects. A `-c copy` file and the screen can silently disagree.

The cost is real and budgeted: ~0.4 of a core per 720p camera at RECORD_FPS=15,
on top of the ~0.6 the decode already takes. Two cameras recording is ~2 of the
Pi 4's 4 cores. Drop RECORD_FPS or set RECORD_MAX_WIDTH if that is too tight.

Frames are written on a wall clock, not on stream arrival: one frame per tick
whether or not the camera delivered a new one. An hour of duct run is therefore
an hour of video, and a camera that stalls for 20 s leaves 20 s of held frame
instead of a jump cut that hides the outage. Time in PAUSE is not written at
all, which is what makes pause a cut rather than a freeze.

Exercise it headless, with no switches and no cameras:

    python3 recorder.py rtsp://192.168.1.103:554/stream
"""

from __future__ import annotations

import os
import shutil
import threading
import time
from datetime import datetime

import config

# cv2 is loaded lazily, exactly as main.py defers it and for the same reason:
# inputs_panel.py and topbar.py import hms() from this module, main.py imports
# inputs_panel at module scope, and cv2 is ~2s of import on a Pi 4 - every
# second of it spent before the splash can paint. Only the encoder thread needs
# it, and by the time that runs main.py has loaded the video stack anyway.
cv2 = None


def _load_cv2():
    global cv2
    if cv2 is None:
        import cv2 as _cv2
        cv2 = _cv2

# inputs.py owns the vocabulary; importing it here keeps one definition of the
# three state strings rather than a second set that can drift.
from inputs import PAUSED, RECORDING, STOPPED


def _stamp(when=None):
    return (when or datetime.now()).strftime("%Y%m%d-%H%M%S")


def hms(seconds):
    """Elapsed seconds as H:MM:SS / M:SS. Used by the panel and the top bar."""
    if seconds is None:
        return "--:--"
    seconds = int(seconds)
    h, m, s = seconds // 3600, (seconds // 60) % 60, seconds % 60
    return f"{h}:{m:02d}:{s:02d}" if h else f"{m}:{s:02d}"


def free_mb(path):
    """Free megabytes on the filesystem holding `path`, or None if unknown."""
    probe = path
    while probe and not os.path.isdir(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            return None
        probe = parent
    try:
        return shutil.disk_usage(probe).free / (1024 * 1024)
    except OSError:
        return None


class CameraRecorder(threading.Thread):
    """Encodes one camera into one file per clip. Never raises at callers.

    The contract is the same as InputReader's: every failure becomes `error`
    text that the panel can show, because a recorder that takes the viewer down
    with it is worse than one that admits it is not recording.
    """

    def __init__(self, stream, slug):
        super().__init__(daemon=True)
        self.stream = stream
        self.slug = slug

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._writer = None
        self._size = None            # (w, h) the open writer was built for
        self._path = None
        self._pending = None         # next clip path, picked up by the thread
        self._close = False          # asked to finish the open clip
        self._next_path = None       # thread-local: where the open clip goes

        # Published, read under the lock by status().
        self._frames = 0
        self._clip_frames = 0
        self._error = None
        self._rolling = False

    # -- public ---------------------------------------------------------------

    def begin_clip(self, path):
        """Finish whatever is open and start writing `path`."""
        with self._lock:
            self._pending = path
            self._close = True

    def end_clip(self):
        """Finish the open clip and write nothing until begin_clip() again."""
        with self._lock:
            self._pending = None
            self._close = True

    def set_rolling(self, rolling):
        """True only while the session is RECORDING - PAUSE writes nothing."""
        with self._lock:
            self._rolling = bool(rolling)

    def status(self):
        with self._lock:
            return {
                "slug": self.slug,
                "path": self._path,
                "frames": self._frames,
                "clip_frames": self._clip_frames,
                "error": self._error,
                "bytes": self._bytes(self._path),
            }

    def stop(self):
        self._stop.set()

    # -- internals ------------------------------------------------------------

    @staticmethod
    def _bytes(path):
        try:
            return os.path.getsize(path) if path else 0
        except OSError:
            return 0

    def _target_size(self, frame):
        h, w = frame.shape[:2]
        limit = config.RECORD_MAX_WIDTH
        if limit and w > limit:
            scale = limit / float(w)
            # Both dimensions forced even: most encoders reject odd sizes, and
            # cv2.VideoWriter reports that by quietly not opening.
            w, h = int(w * scale) // 2 * 2, int(h * scale) // 2 * 2
        return w, h

    def _open_writer(self, path, frame):
        size = self._target_size(frame)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*config.RECORD_FOURCC)
        writer = cv2.VideoWriter(path, fourcc, config.RECORD_FPS, size)
        if not writer.isOpened():
            writer.release()
            # The usual cause is a fourcc this OpenCV build cannot encode
            # (avc1 without libx264), and OpenCV only warns on stderr - which
            # goes nowhere under the .pyw launcher. Say it where it will be read.
            with self._lock:
                self._error = f"cannot encode {config.RECORD_FOURCC} -> {path}"
            return None, None
        return writer, size

    def _release(self):
        if self._writer is not None:
            self._writer.release()
        self._writer, self._size = None, None

    def _write(self, frame):
        w, h = self._size
        if (frame.shape[1], frame.shape[0]) != (w, h):
            # The camera can come back at a different resolution after a
            # reconnect. Resizing keeps one continuous clip; a new writer
            # mid-clip would truncate the file the operator is watching grow.
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
        self._writer.write(frame)

    def run(self):
        _load_cv2()
        period = 1.0 / max(1.0, config.RECORD_FPS)
        next_tick = time.monotonic()
        last_frame = None
        checked_disk = 0.0

        try:
            while not self._stop.is_set():
                next_tick += period
                now = time.monotonic()
                # A long stall (a reconnect storm, a busy Pi) must not turn into
                # a burst of catch-up writes that all carry the same frame.
                if next_tick < now:
                    next_tick = now
                self._stop.wait(max(0.0, next_tick - now))
                if self._stop.is_set():
                    break

                with self._lock:
                    rolling, close_now, pending = (
                        self._rolling, self._close, self._pending)
                    self._close = False
                    if pending is not None:
                        self._pending = None

                if close_now:
                    # end_clip() sends pending=None, so this clears the target
                    # as well as closing the file. Leaving the old path behind
                    # would have the next roll reopen - and truncate - the clip
                    # that was just saved.
                    self._release()
                    self._next_path = pending
                    with self._lock:
                        self._clip_frames = 0
                        self._path = pending

                if not rolling or self._next_path is None:
                    continue
                path = self._next_path

                frame, _seq = self.stream.latest()
                if frame is None:
                    # Hold the last good frame so the timeline stays real-time
                    # across a camera dropout. Before the FIRST frame there is
                    # nothing to hold, so the file simply starts when video does.
                    frame = last_frame
                if frame is None:
                    with self._lock:
                        self._error = "waiting for video"
                    continue
                last_frame = frame

                if self._writer is None:
                    writer, size = self._open_writer(path, frame)
                    if writer is None:
                        # Do not spin retrying a broken encoder every tick.
                        self._stop.wait(1.0)
                        continue
                    self._writer, self._size = writer, size
                    with self._lock:
                        self._error = None

                # Checked here rather than by the session, because this is the
                # thread that is about to make the file bigger.
                if now - checked_disk > 5.0:
                    checked_disk = now
                    free = free_mb(path)
                    if free is not None and free < config.RECORD_MIN_FREE_MB:
                        self._release()
                        with self._lock:
                            self._error = f"disk full - {free:.0f} MB free"
                        self._stop.wait(5.0)
                        continue

                try:
                    self._write(frame)
                except Exception as exc:            # encoder blew up mid-clip
                    with self._lock:
                        self._error = f"write failed: {exc}"
                    self._release()
                    continue

                with self._lock:
                    self._frames += 1
                    self._clip_frames += 1
        finally:
            self._release()


class SessionManager:
    """The recording session: state machine, output paths, one recorder each.

    main.py pushes the decoded switch state in every UI frame and reads status()
    back out. Everything below the API is idempotent, so calling set_state()
    thirty times a second with the same value costs nothing.

    Layout on disk - one directory per session, one file per camera per clip:

        ~/recordings/20260815-134500/cam1_001.mp4
                                     cam2_001.mp4
                                     cam1_002.mp4   <- after a SAVE press
    """

    # How long the SAVED confirmation stays on screen. Long enough to read
    # after looking back up from the panel, short enough not to be furniture.
    TOAST_S = float(os.environ.get("SAVE_TOAST_S", "6.0"))

    def __init__(self, streams, root=None):
        self.root = root or config.RECORD_DIR
        self.recorders = [
            CameraRecorder(s, s.name.lower().replace(" ", "")) for s in streams
        ]
        for rec in self.recorders:
            rec.start()

        self.state = STOPPED
        self.session_dir = None
        self.session_started = None
        self.clip = 0

        # Recorded seconds, i.e. wall clock spent RECORDING with the pauses
        # taken out - which is both what the operator means by "how long" and
        # the playback length of the files.
        self._rolled = 0.0          # closed-out time, before the current roll
        self._roll_since = None     # monotonic when the current roll began
        self._clip_rolled = 0.0
        self._clip_since = None

        self._toast = None          # (text, detail, monotonic_expiry)
        self._last_saves = None     # last save_presses count acted on

    # -- clock ----------------------------------------------------------------

    def _elapsed(self):
        extra = 0.0 if self._roll_since is None else time.monotonic() - self._roll_since
        return self._rolled + extra

    def _clip_elapsed(self):
        extra = 0.0 if self._clip_since is None else time.monotonic() - self._clip_since
        return self._clip_rolled + extra

    def _roll(self, rolling):
        """Start/stop both clocks. Idempotent."""
        now = time.monotonic()
        if rolling and self._roll_since is None:
            self._roll_since, self._clip_since = now, now
        elif not rolling and self._roll_since is not None:
            self._rolled += now - self._roll_since
            self._clip_rolled += now - self._clip_since
            self._roll_since = self._clip_since = None

    # -- clips ----------------------------------------------------------------

    def _clip_paths(self):
        return {
            rec.slug: os.path.join(
                self.session_dir, f"{rec.slug}_{self.clip:03d}{config.RECORD_EXT}")
            for rec in self.recorders
        }

    def _begin_clip(self):
        self.clip += 1
        paths = self._clip_paths()
        for rec in self.recorders:
            rec.begin_clip(paths[rec.slug])
        self._clip_rolled = 0.0
        self._clip_since = time.monotonic() if self.state == RECORDING else None

    def _end_clip(self):
        for rec in self.recorders:
            rec.end_clip()

    # -- session --------------------------------------------------------------

    def _start(self):
        self.session_started = datetime.now()
        self.session_dir = os.path.join(self.root, _stamp(self.session_started))
        os.makedirs(self.session_dir, exist_ok=True)
        self.clip = 0
        self._rolled = self._clip_rolled = 0.0
        self._roll_since = self._clip_since = None
        self._begin_clip()

    def _stop_session(self):
        self._roll(False)
        held = self._elapsed()
        self._end_clip()
        for rec in self.recorders:
            rec.set_rolling(False)
        if held >= 1.0:
            self._toast_now("SAVED", f"{self.clip} clip{'s' if self.clip != 1 else ''}"
                                     f"  {hms(held)}")
        self._roll_since = self._clip_since = None
        self._discard_if_empty()

    def _discard_if_empty(self):
        """Remove the session directory if nothing was ever written into it.

        With the cameras off the wire a start/stop leaves a dated, empty
        directory - and three of those appeared in ten seconds during bring-up,
        from switch bounce alone. A directory per non-recording is worse than
        useless: it makes the recording list a place you have to sort real runs
        out of. rmdir(), never rmtree() - it refuses on a directory holding
        anything at all, so this cannot delete footage even if the frame counts
        are wrong.
        """
        if not self.session_dir:
            return
        try:
            if not os.listdir(self.session_dir):
                os.rmdir(self.session_dir)
                self.session_dir = None
        except OSError:
            pass

    def set_state(self, state):
        """Push the decoded switch state. Safe to call at the UI frame rate."""
        if state not in (RECORDING, PAUSED, STOPPED):
            return
        if state == self.state:
            # Still has to keep the clock honest across the frames *between*
            # transitions - _roll() is the idempotent part.
            self._roll(state == RECORDING)
            return

        was, self.state = self.state, state
        if was == STOPPED:
            self._start()
        if state == STOPPED:
            self._stop_session()
            return

        self._roll(state == RECORDING)
        for rec in self.recorders:
            rec.set_rolling(state == RECORDING)

    def save_clip(self):
        """The GPIO25 press: close this clip, open the next, confirm on screen.

        Deliberately does not stop the recording. The button exists so an
        operator who just drove past something worth keeping can bank it without
        interrupting the run - stopping to save would mean the next thing down
        the duct goes unrecorded.
        """
        if self.state == STOPPED or self.session_dir is None:
            self._toast_now("NOTHING TO SAVE", "not recording")
            return False

        held = self._clip_elapsed()
        frames = max((r.status()["frames"] for r in self.recorders), default=0)
        label = f"clip {self.clip:03d}"
        self._begin_clip()

        if frames == 0:
            self._toast_now("SAVED (EMPTY)", f"{label}  no video")
        else:
            self._toast_now("SAVED", f"{label}  {hms(held)}")
        return True

    def on_save_button(self, presses):
        """Fire save_clip() once per new press of the GPIO25 button.

        Counted edges, not levels - see inputs.SAVE_PIN. The first call only
        primes the counter, or a viewer restarted mid-session would fire a save
        it was never asked for. `>` not `!=`, so the counter resetting to 0
        (which _blank() does whenever the reader loses the pins) is a no-op
        rather than a phantom press.
        """
        presses = presses or 0
        if self._last_saves is None or presses > self._last_saves:
            fire = self._last_saves is not None
            self._last_saves = presses
            if fire:
                self.save_clip()

    def on_inputs(self, snapshot):
        """Apply one whole inputs.py snapshot. Convenience for callers with no
        keyboard fallback to merge in - main.py drives the two halves itself."""
        state = snapshot.get("session")
        if state is not None:
            self.set_state(state)
        self.on_save_button(snapshot.get("save_presses"))

    # -- readout --------------------------------------------------------------

    def _toast_now(self, text, detail):
        self._toast = (text, detail, time.monotonic() + self.TOAST_S)

    def toast(self):
        """(text, detail) for the confirmation banner, or None once it expires."""
        if self._toast is None:
            return None
        text, detail, expiry = self._toast
        if time.monotonic() > expiry:
            self._toast = None
            return None
        return text, detail

    def status(self):
        cams = [rec.status() for rec in self.recorders]
        return {
            "state": self.state,
            "elapsed": self._elapsed(),
            "clip": self.clip,
            "clip_elapsed": self._clip_elapsed(),
            "dir": self.session_dir,
            "cameras": cams,
            "bytes": sum(c["bytes"] for c in cams),
            # One line for the strip: the first real error, or None.
            "error": next((c["error"] for c in cams if c["error"]), None),
            "free_mb": free_mb(self.session_dir or self.root),
            "toast": self.toast(),
        }

    def stop(self):
        if self.state != STOPPED:
            self.set_state(STOPPED)
        for rec in self.recorders:
            rec.stop()
        for rec in self.recorders:
            rec.join(timeout=3.0)


def main():
    """Record from the URLs on the command line for 10s, pause 3s, save, stop."""
    import sys

    from stream import RTSPStream

    urls = sys.argv[1:] or [url for _, url in config.CAMERAS]
    streams = [
        RTSPStream(name=f"CAM {i + 1}", url=url,
                   latency_ms=config.RTSP_LATENCY_MS,
                   protocols=config.RTSP_PROTOCOL)
        for i, url in enumerate(urls)
    ]
    for s in streams:
        s.start()

    session = SessionManager(streams)
    script = [(RECORDING, 10), (PAUSED, 3), ("SAVE", 0), (RECORDING, 6),
              (STOPPED, 1)]
    try:
        for step, hold in script:
            if step == "SAVE":
                session.save_clip()
            else:
                session.set_state(step)
            print(f"-> {step}")
            for _ in range(int(hold * 2)):
                time.sleep(0.5)
                st = session.status()
                print(f"   {st['state']:<9} {hms(st['elapsed'])} "
                      f"clip {st['clip']} "
                      f"{st['bytes'] / 1e6:.1f} MB "
                      f"{[c['frames'] for c in st['cameras']]} "
                      f"{st['error'] or ''}")
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        for s in streams:
            s.stop()
        print(f"files in {session.session_dir}")


if __name__ == "__main__":
    main()
