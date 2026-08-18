#!/usr/bin/env python3
"""
Recording: panel switches -> video files on disk.

Same split as stream.py / inputs.py - this module owns the encoders and their
threads, main.py owns the single UI timer and pushes state in. Nothing here
touches Qt.

WHAT DRIVES IT (inputs.py decodes the pins, this acts on the decode):

    switch 1, red leg, GPIO24 .... START / STOP
    switch 2, green leg, GPIO23 .. PAUSE / RESUME
    button,   GPIO25 ............. SAVE - while rolling, a tap cuts the clip
                                   and keeps rolling; after STOP, HOLDING it
                                   for RECORD_SAVE_HOLD_S (3s) finalizes the
                                   session into RECORD_DIR (/recordings)

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
np = None


def _load_cv2():
    global cv2, np
    if cv2 is None:
        import cv2 as _cv2
        import numpy as _np
        cv2 = _cv2
        np = _np

# inputs.py owns the vocabulary; importing it here keeps one definition of the
# three state strings rather than a second set that can drift.
from inputs import PAUSED, RECORDING, STOPPED

# A fourth state that no switch produces: the window after STOP in which the
# recording exists on disk but has not been claimed. It lives here rather than
# in inputs.py precisely because it is not a switch position - it is what the
# recorder is doing while it waits to be told whether the run was worth keeping.
PENDING = "SAVE?"


def _stamp(when=None):
    # YYYYMMDD_HHMMSS - the operator-specified convention for session names.
    return (when or datetime.now()).strftime("%Y%m%d_%H%M%S")


def _next_session_no(root):
    """1 + the highest SESSIONnnn already in `root`, so numbers never repeat.

    Scanned from disk on every start rather than counted in memory: the counter
    has to survive viewer restarts, and the directory listing IS the durable
    record of how many sessions exist. An unreadable root (first boot, no
    /recordings yet) starts at 1.
    """
    import re
    highest = 0
    try:
        for name in os.listdir(root):
            m = re.search(r"_SESSION(\d+)$", name)
            if m:
                highest = max(highest, int(m.group(1)))
    except OSError:
        pass
    return highest + 1


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


class CombinedView:
    """A pseudo-stream: every real camera side by side in ONE frame.

    Quacks exactly like RTSPStream where CameraRecorder is concerned - .name,
    .slug, .latest() -> (frame, seq) - so the recorder that encodes it is the
    same code that encodes a camera, and the full view inherits the wall-clock
    timeline, the disk-space guard and the error reporting for free.

    Composition happens INSIDE latest(), i.e. on the encoder thread at
    RECORD_FPS, not per decoded frame: two resizes to COMBINED_HEIGHT plus an
    hstack ~ a millisecond, and doing it 15x/s instead of 25x/s per camera is
    the cheap direction. The GUI never sees this object.

    A camera that has not delivered yet gets its LAST frame, then a black tile,
    so one dead camera cannot black out the other's half of the record - the
    same hold-the-last-frame honesty the per-camera files have.
    """

    def __init__(self, streams, height=None):
        self.streams = streams
        self.name = "FULL VIEW"
        self.slug = "full"
        self.height = height or config.COMBINED_HEIGHT
        self._last = [None] * len(streams)
        self._seq = 0

    def _tile(self, index, frame):
        h = self.height
        if frame is None:
            frame = self._last[index]
        if frame is None:
            # Nothing ever decoded: a black 16:9 placeholder keeps the canvas
            # geometry stable until the camera shows up.
            return np.zeros((h, h * 16 // 9, 3), dtype=np.uint8)
        self._last[index] = frame
        fh, fw = frame.shape[:2]
        tile = cv2.resize(frame, (max(2, int(round(fw * h / fh))) // 2 * 2, h),
                          interpolation=cv2.INTER_AREA)
        label = config.camera_label(index) or f"CAM {index + 1}"
        # Label burned into the pixels, not into metadata: the file gets copied
        # to USB and watched anywhere, so FRONT/BACK has to travel inside it.
        cv2.putText(tile, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(tile, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return tile

    def latest(self):
        frames, seq_sum = [], 0
        for stream in self.streams:
            frame, seq = stream.latest()
            frames.append(frame)
            seq_sum += seq or 0
        if all(f is None for f in frames) and all(f is None for f in self._last):
            return None, 0
        canvas = np.hstack([self._tile(i, f) for i, f in enumerate(frames)])
        # Monotonic even if a camera reconnects and its seq resets.
        self._seq = max(self._seq + 1, seq_sum)
        return canvas, self._seq


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
        # NOT self._stop: threading.Thread already has a private _stop() that
        # Thread.join() calls to reap a finished thread, so an Event of that
        # name shadows it and join() dies with "'Event' object is not callable".
        # It only shows up at shutdown, which is the worst place to find it.
        self._stopping = threading.Event()
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

    def idle(self):
        """True when no file is open, so its clips are safe to move or delete."""
        with self._lock:
            return self._writer is None and not self._close

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
        self._stopping.set()

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
            while not self._stopping.is_set():
                next_tick += period
                now = time.monotonic()
                # A long stall (a reconnect storm, a busy Pi) must not turn into
                # a burst of catch-up writes that all carry the same frame.
                if next_tick < now:
                    next_tick = now
                self._stopping.wait(max(0.0, next_tick - now))
                if self._stopping.is_set():
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
                        self._stopping.wait(1.0)
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
                        self._stopping.wait(5.0)
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

    Layout on disk - one directory per session, one file per camera per clip,
    plus the side-by-side FULL VIEW (see CombinedView):

        /recordings/20260815_134500_SESSION001/cam1_front_001.mp4
                                               cam2_back_001.mp4
                                               full_001.mp4
                                               cam1_front_002.mp4  <- after SAVE
    """

    # How long the SAVED confirmation stays on screen. Long enough to read
    # after looking back up from the panel, short enough not to be furniture.
    TOAST_S = float(os.environ.get("SAVE_TOAST_S", "6.0"))

    def __init__(self, streams, root=None):
        self.root = root or config.RECORD_DIR
        sources = list(streams)
        # The FULL VIEW is a session output, so it is created here rather than
        # by main.py: anything that records through a SessionManager gets the
        # combined file with no extra wiring, the headless test included.
        if config.RECORD_COMBINED and len(sources) > 1:
            sources.append(CombinedView(sources[:]))
        self.recorders = [
            CameraRecorder(
                s, getattr(s, "slug", None) or s.name.lower().replace(" ", ""))
            for s in sources
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
        self._hold_s = 0.0          # how long SAVE is currently held (pushed in)

        # Every file this session has handed to a recorder, so a discard can
        # remove exactly what was written and nothing else - see _discard().
        self._written = []
        # The unclaimed recording, if any: see PENDING.
        self._pending = None

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
        self._written.extend(paths.values())
        self._clip_rolled = 0.0
        self._clip_since = time.monotonic() if self.state == RECORDING else None

    def _end_clip(self):
        for rec in self.recorders:
            rec.end_clip()

    # -- session --------------------------------------------------------------

    def _start(self):
        # Anything still unclaimed loses its window here. Silence is a discard
        # everywhere else in this flow, and a second run starting is a stronger
        # signal than silence that the operator has moved on from the first.
        if self._pending:
            self._resolve_pending(keep=False, reason="superseded")

        self.session_started = datetime.now()
        self.session_dir = os.path.join(
            self.root,
            f"{_stamp(self.session_started)}"
            f"_SESSION{_next_session_no(self.root):03d}")
        os.makedirs(self.session_dir, exist_ok=True)
        self.clip = 0
        self._written = []
        self._rolled = self._clip_rolled = 0.0
        self._roll_since = self._clip_since = None
        self._begin_clip()

    def _stop_session(self):
        self._roll(False)
        held = self._elapsed()
        self._end_clip()
        for rec in self.recorders:
            rec.set_rolling(False)
        self._roll_since = self._clip_since = None

        # Nothing worth asking about: no time on the clock, or the cameras never
        # delivered a frame so there is no file to keep either way.
        wrote = any(rec.status()["frames"] for rec in self.recorders)
        if held < 1.0 or not wrote or config.RECORD_CONFIRM_S <= 0:
            if held >= 1.0 and wrote:
                self._toast_now("SAVED", f"{self._clip_word()}  {hms(held)}")
            self._discard_if_empty()
            return

        # The recorder threads close their writers on their next tick, up to
        # 1/RECORD_FPS later. The window is orders of magnitude longer than
        # that, so the files are always complete before it can be resolved.
        self._pending = {
            "until": time.monotonic() + config.RECORD_CONFIRM_S,
            "held": held,
            "clips": self.clip,
            "dir": self.session_dir,
            "files": list(self._written),
        }

    def _clip_word(self, clips=None):
        clips = self.clip if clips is None else clips
        return f"{clips} clip{'s' if clips != 1 else ''}"

    # -- the confirm window ----------------------------------------------------

    def pending_left(self):
        """Seconds remaining to claim the last recording, or None if not waiting."""
        if not self._pending:
            return None
        return max(0.0, self._pending["until"] - time.monotonic())

    def _resolve_pending(self, keep, reason=""):
        pending, self._pending = self._pending, None
        if not pending:
            return
        held, clips = pending["held"], pending["clips"]
        if keep:
            self._toast_now("SAVED", f"{self._clip_word(clips)}  {hms(held)}")
            return

        # The encoder threads close their writers on their own tick, so a
        # discard fired the instant after STOP - which "superseded" is - can
        # land while the file is still open. On Windows os.remove then raises
        # and the delete silently does nothing; on Linux it unlinks a file the
        # writer is still filling. Wait for them, briefly and boundedly.
        self._await_closed()

        # Remove exactly the files this session handed out, then the directory -
        # os.remove per known path and a bare rmdir, never rmtree. A recursive
        # delete pointed at a path built from a timestamp is one bad join away
        # from taking something else with it, and this runs unattended.
        failed = 0
        for path in pending["files"]:
            try:
                os.remove(path)
            except FileNotFoundError:
                pass                    # never opened - the camera was dead
            except OSError:
                failed += 1
        try:
            os.rmdir(pending["dir"])
        except OSError:
            pass
        if pending["dir"] == self.session_dir:
            self.session_dir = None

        detail = f"{self._clip_word(clips)}  {hms(held)}"
        if failed:
            # Do not claim a discard that did not happen - the operator would
            # walk away believing the card is clear when it is not.
            self._toast_now("DISCARD FAILED",
                            f"{failed} file{'s' if failed != 1 else ''} still on disk")
        else:
            self._toast_now("DISCARDED",
                            f"{detail}  ({reason})" if reason else detail)

    def _await_closed(self, timeout=1.5):
        """Block until every encoder has released its file, or `timeout`.

        Only ever called on a state transition, never per frame, and normally
        returns at once - by the time a confirm window expires the writers have
        been shut for fifteen seconds. The bound exists so a wedged encoder
        thread cannot freeze the UI, not because waiting is expected.
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if all(rec.idle() for rec in self.recorders):
                return True
            time.sleep(0.02)
        return False

    def poll(self):
        """Expire the confirm window. main.py calls this once per UI frame.

        Deliberately not folded into status(): a getter that deletes files is a
        getter nobody can call safely, including from a debugger.
        """
        if self._pending and time.monotonic() >= self._pending["until"]:
            self._resolve_pending(keep=False, reason="not saved")

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
        """The GPIO25 press. It means two different things by design:

        While rolling, it closes this clip and opens the next without stopping -
        so an operator who just drove past something worth keeping can bank it
        and let the run continue, rather than having to stop and lose whatever
        comes next down the duct.

        In the window after STOP, a tap only HINTS: claiming the whole session
        is a press-and-hold (RECORD_SAVE_HOLD_S, see on_save_hold), so a stray
        tap of the same button that banks clips mid-run cannot silently commit
        a whole recording. The hint teaches the gesture at the exact moment it
        is needed.
        """
        if self._pending:
            # No toast: the strip's pending line already reads "hold SAVE ...",
            # and a toast would sit on top of the live keep-holding countdown
            # for TOAST_S - exactly the seconds the operator needs to see it.
            return False

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

    def finalize(self):
        """Claim the unclaimed recording NOW. True if there was one to claim.

        The programmatic form of the 3s hold - the keyboard fallback and
        shutdown use it, the GPIO path arrives via on_save_hold().
        """
        if not self._pending:
            return False
        self._resolve_pending(keep=True)
        return True

    def on_save_hold(self, held_s):
        """Push how long GPIO25 has been held. main.py calls this every frame.

        The press-and-hold that finalizes a stopped recording (operator spec
        2026-08-18): after STOP, holding SAVE for RECORD_SAVE_HOLD_S claims the
        session into RECORD_DIR. Level-driven rather than edge-driven because a
        hold IS a level - and while the button is down the confirm window is
        pushed back, so a hold that starts on the window's last second is
        honoured instead of the countdown deleting the files mid-hold.
        """
        self._hold_s = held_s or 0.0
        if not self._pending or self._hold_s <= 0.0:
            return
        if self._hold_s >= config.RECORD_SAVE_HOLD_S:
            self._resolve_pending(keep=True)
            return
        self._pending["until"] = max(
            self._pending["until"],
            time.monotonic()
            + (config.RECORD_SAVE_HOLD_S - self._hold_s) + 1.0)

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
        left = self.pending_left()
        return {
            # PENDING outranks the switch state: the switch says STOPPED, but
            # what the operator has to act on is the unanswered question.
            "state": PENDING if left is not None else self.state,
            "pending_left": left,
            "pending_held": self._pending["held"] if self._pending else None,
            "pending_clips": self._pending["clips"] if self._pending else None,
            # Progress of the press-and-hold, so the strip can count it down
            # live instead of the operator guessing when 3 seconds is up.
            "save_hold": self._hold_s if self._pending else None,
            "save_hold_need": config.RECORD_SAVE_HOLD_S,
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
        # Shutting the viewer down is not the operator declining to save - it is
        # the window being cut short before they could answer. Keep it. The rule
        # is "unclaimed footage is discarded", and footage nobody was given the
        # chance to claim is not unclaimed.
        if self._pending:
            self._resolve_pending(keep=True)
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
