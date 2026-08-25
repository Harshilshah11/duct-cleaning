"""
Threaded RTSP capture.

The naive approach - calling cap.read() on a QTimer in the GUI thread - blocks
the event loop for as long as the network takes to deliver a frame, so one slow
camera freezes the whole window (including the joystick/command timer you add in
Step 9). Instead each camera gets its own daemon thread that reads as fast as the
stream delivers and keeps only the newest frame. The GUI samples that slot at its
own rate, which drops stale frames automatically and keeps latency low.
"""

import os
import socket
import threading
import time
from urllib.parse import urlparse

import cv2

import config

# Must be set BEFORE the first cv2.VideoCapture(..., CAP_FFMPEG) call.
#   timeout      = socket timeout in microseconds (ffmpeg >= 5.0; 'stimeout' on older)
#   max_delay    = demuxer reorder delay
_TIMEOUT_US = int(config.OPEN_TIMEOUT_S * 1_000_000)
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    f"rtsp_transport;{config.RTSP_PROTOCOL}|timeout;{_TIMEOUT_US}"
    f"|max_delay;500000|reorder_queue_size;0",
)


# --- connection-churn control -------------------------------------------------
#
# Everything in this block exists to keep the number of TCP connections we open
# to the cameras low. See config.RECONNECT_MAX_DELAY_S for the measurements: the
# cameras' embedded TCP stacks fall over under repeated RTSP session churn and
# then stop answering ARP entirely, which reads as "the camera died" when it is
# really "we knocked it over". This only ever bit with both cameras configured,
# because that doubles the rate.

_connect_gate = threading.Lock()
_last_connect_at = 0.0

# Set STREAM_STATS_LOG=/path/to/file to get one line per camera per second with
# the numbers that actually matter when someone reports "it stalls": decoded
# fps, total frames, reconnect count and abandoned-reader count. Without this
# the only evidence is the picture on the screen, and the viewer's stdout goes
# to tty1 where nothing keeps it. Off unless the variable is set.
_STATS_PATH = os.environ.get("STREAM_STATS_LOG", "")
_STATS_INTERVAL = float(os.environ.get("STREAM_STATS_INTERVAL", "1.0"))
_stats_lock = threading.Lock()

# probe_codec() costs a whole extra TCP connection + DESCRIBE per attempt, and a
# camera does not change codec between reconnects - so ask once per URL and
# remember the answer. Only successful probes are cached; a failed one must stay
# retryable or a camera that was down at startup would be stuck on the default.
_codec_cache = {}
_codec_cache_lock = threading.Lock()


def _stagger_connects(min_gap: float):
    """Block until at least `min_gap` has passed since any camera last connected.

    The cameras share one tether. Letting both open RTSP sessions in the same
    instant is the worst case for them, so attempts are serialised rig-wide
    rather than per-stream. Deliberately sleeps holding the lock: the point is
    that only one connection attempt is in flight at a time.
    """
    global _last_connect_at
    if min_gap <= 0:
        return
    with _connect_gate:
        wait = min_gap - (time.monotonic() - _last_connect_at)
        if wait > 0:
            time.sleep(wait)
        _last_connect_at = time.monotonic()


def cached_codec(url: str, timeout: float):
    """probe_codec() with a per-URL cache - see _codec_cache above."""
    with _codec_cache_lock:
        hit = _codec_cache.get(url)
    if hit:
        return hit
    codec = probe_codec(url, timeout)
    if codec:
        with _codec_cache_lock:
            _codec_cache[url] = codec
    return codec


def has_gstreamer() -> bool:
    """True if this OpenCV build was compiled with GStreamer support.

    The Raspberry Pi OS apt package (python3-opencv) has it.
    The PyPI wheel (pip install opencv-python) does NOT.
    """
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer" in line:
            return "YES" in line.upper()
    return False


def probe_codec(url: str, timeout: float = 4.0):
    """Ask the RTSP server which video codec it serves; returns 'h264'/'h265'/None.

    Necessary because the GStreamer pipeline needs a matching depayloader +
    decoder pair and we cannot use decodebin to autoplug it (see gst_pipeline).
    Cameras routinely serve H.264 on the main stream and H.265 on the sub-stream,
    so hard-coding one codec silently breaks half the panels.

    Reads the codec from the SDP 'a=rtpmap:<pt> H264/90000' line of a DESCRIBE.
    """
    try:
        parsed = urlparse(url)
        host, port = parsed.hostname, parsed.port or 554
    except ValueError:
        return None
    if not host:
        return None

    path = parsed.path or "/"
    if parsed.query:
        path = f"{path}?{parsed.query}"
    request = (
        f"DESCRIBE rtsp://{host}:{port}{path} RTSP/1.0\r\n"
        f"CSeq: 1\r\n"
        f"Accept: application/sdp\r\n\r\n"
    ).encode()

    data = b""
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            sock.sendall(request)
            while len(data) < 8192:
                chunk = sock.recv(2048)
                if not chunk:
                    break
                data += chunk
    except OSError:
        if not data:
            return None

    text = data.decode("utf-8", "replace").lower()
    # Check h265/hevc first - "h264" never appears inside those tokens, but a
    # camera may list both if it offers multiple payload types.
    if "h265" in text or "hevc" in text:
        return "h265"
    if "h264" in text:
        return "h264"
    return None


def gst_pipeline(url: str, latency_ms: int = 50, protocols: str = "tcp",
                 hw_decode: bool = False, codec: str = "h264",
                 rotate: int = 0) -> str:
    """Low-latency GStreamer pipeline feeding an OpenCV appsink.

    max-buffers=1 drop=true sync=false is the important part: it throws away
    stale frames instead of queueing them, so you always see 'now'.
    """
    # Two hard-won details here, both verified on a Pi 4 / GStreamer 1.26:
    #
    # 1. Use an EXPLICIT rtph264depay ! h264parse ! <decoder> chain, not
    #    decodebin. decodebin exposes its output on a dynamic pad, and OpenCV's
    #    manual-pipeline path cannot wait for that - the pipeline dies with
    #    "Internal data stream error" and isOpened() returns False. decodebin,
    #    decodebin+queue and uridecodebin were all measured failing; the explicit
    #    chain delivered a full 15/15 fps. (The same decodebin pipeline runs fine
    #    under gst-launch, which is what makes this so easy to misdiagnose.)
    #
    # 2. Do NOT set name= on the appsink. OpenCV looks the sink up by its
    #    auto-assigned default name "appsink0"; any other name makes it invisible
    #    and the pipeline is rejected with "cannot find appsink in manual
    #    pipeline". Both failures fall back to the FFMPEG backend, which still
    #    "works" but delivers buffered bursts at a fraction of the frame rate -
    #    so this fails quietly rather than loudly.
    #
    # Because the chain is explicit it must match the actual codec - hence
    # probe_codec(). H.265 sub-streams are common on IP cameras.
    if codec == "h265":
        depay, parser, decoder = "rtph265depay", "h265parse", "avdec_h265"
    else:
        depay, parser = "rtph264depay", "h264parse"
        decoder = "v4l2h264dec" if hw_decode else "avdec_h264"

    # The 'queue leaky=downstream' after the decoder is what makes this keep up
    # with a real camera. It decouples the decoder from the appsink so decoding
    # never stalls waiting for the GUI to collect a frame; stale frames are
    # dropped instead of backing up. Measured against a 25 fps 720p camera:
    #
    #   latency=50  drop-on-latency=true  (no queue) .... 19.2 fps
    #   latency=200 drop-on-latency=true  (no queue) .... 10.3 fps
    #   latency=200 (no queue) .......................... 14.8 fps
    #   latency=500 (no queue) .......................... 10.6 fps
    #   latency=200 + leaky queue ....................... 25.6 fps  <-- source rate
    #
    # drop-on-latency / do-retransmission were dropped: they measured worse than
    # leaving rtspsrc at its defaults once the queue is present.
    # tcp-timeout bounds how long rtspsrc waits on a dead TCP session before it
    # errors out. Without it, when a camera closes its RTSP connection (FIN ->
    # the socket goes CLOSE-WAIT) rtspsrc can sit on the half-dead socket and
    # cap.read() blocks indefinitely, so the panel freezes on the last frame and
    # never reconnects. 5s (in microseconds) makes a dropped camera surface as a
    # read error quickly. This is load-bearing for the abandon path in
    # RTSPStream._reader: an abandoned reader is left sitting in its doomed
    # read(), and tcp-timeout is what eventually returns it so it can release
    # its capture and exit instead of leaking for the life of the process.
    # Rotate INSIDE the pipeline, and specifically BEFORE videoconvert.
    #
    # Doing it here rather than with cv2.rotate() on the decoded BGR frame is
    # worth about a core on a Pi 4 running two 720p streams. At this point in
    # the pipeline the frame is still the decoder's planar I420 - 1.5 bytes per
    # pixel - so a 1280x720 rotation moves ~1.4 MB. After videoconvert it is
    # BGR at 3 bytes per pixel, ~2.7 MB, so rotating there costs nearly double
    # the memory traffic for exactly the same picture. Measured: rotating in
    # OpenCV took idle CPU from ~45% to ~18%.
    flip = {90: " videoflip method=clockwise !",
            180: " videoflip method=rotate-180 !",
            270: " videoflip method=counterclockwise !"}.get(int(rotate) % 360, "")

    return (
        f"rtspsrc location={url} latency={latency_ms} protocols={protocols} "
        f"tcp-timeout=5000000 ! "
        f"{depay} ! {parser} ! {decoder} !{flip} "
        f"queue leaky=downstream max-size-buffers=2 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink max-buffers=1 drop=true sync=false"
    )


def probe_reachable(url: str, timeout: float) -> bool:
    """Fast TCP connect to the RTSP port before handing the URL to OpenCV.

    Why this exists: cv2.VideoCapture() blocks until the OS gives up on the TCP
    connect, and neither the FFMPEG 'timeout' option nor the GStreamer pipeline
    reliably bounds that. If the 30m tether is unplugged there is no ARP reply,
    so the connect silently hangs for ~2 minutes on Linux and the panel sits on
    "connecting" the whole time. A plain socket with an explicit timeout gives us
    a bounded, predictable failure instead.

    RTSP signalling is TCP even when the media flows over UDP, so this is a valid
    check for both transports.
    """
    try:
        parsed = urlparse(url)
        host, port = parsed.hostname, parsed.port or 554
    except ValueError:
        return True  # unparseable - let VideoCapture make the call
    if not host:
        return True
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class RTSPStream:
    """One RTSP camera. Call start(), then latest() from anywhere."""

    def __init__(self, name, url, latency_ms=50, protocols="tcp",
                 hw_decode=False, reconnect_delay=2.0, read_fail_limit=50,
                 stall_timeout=5.0, abandon_timeout=2.0,
                 reconnect_max_delay=20.0, connect_stagger=1.0,
                 session_wait=8.0, unreachable_max_delay=4.0, rotate=0):
        self.name = name
        self.url = url
        self.latency_ms = latency_ms
        self.protocols = protocols
        self.hw_decode = hw_decode
        self.reconnect_delay = reconnect_delay
        # Failed attempts back off from reconnect_delay up to this, so a camera
        # that is genuinely down is retried occasionally instead of hammered.
        # This is what keeps two cameras alive at once - see config.
        self.reconnect_max_delay = max(reconnect_delay, reconnect_max_delay)
        # Ceiling used instead of the above while the camera is off the wire.
        self.unreachable_max_delay = max(reconnect_delay, unreachable_max_delay)
        self._absent = False        # last open failed because nothing answered
        # Resolved once here rather than per frame. None means "leave it alone",
        # which keeps the hot path free of a dict lookup at 25 fps per camera.
        self.rotate = int(rotate) % 360
        self._rotate_code = {
            90: cv2.ROTATE_90_CLOCKWISE,
            180: cv2.ROTATE_180,
            270: cv2.ROTATE_90_COUNTERCLOCKWISE,
        }.get(self.rotate)
        # Set per connection by _open(): False when the GStreamer pipeline is
        # doing the rotation, True on the FFMPEG fallback which cannot.
        self._rotate_in_reader = True
        self.connect_stagger = connect_stagger
        # Never hold two RTSP sessions on one camera - see config.SESSION_WAIT_S.
        self.session_wait = session_wait
        self.read_fail_limit = read_fail_limit
        # Wall-clock seconds with no decoded frame before we tear the capture
        # down and reconnect. This is the backstop for a cap.read() that blocks
        # forever on a wedged RTSP socket, where read_fail_limit (which only
        # counts reads that RETURN) never trips. 0 disables it.
        self.stall_timeout = stall_timeout
        # How long to wait for a retired reader thread to notice and exit before
        # we stop waiting on it and open a fresh connection anyway. See _reader.
        self.abandon_timeout = abandon_timeout

        # Published state (plain attributes; reads are atomic enough for status text)
        self.connected = False
        self.backend = "-"
        self.codec = "-"
        self.fps = 0.0
        self.frames_total = 0
        self.reconnects = 0
        # Reader threads we walked away from because their read() was wedged.
        # Non-zero is normal on a flaky camera; steadily climbing means the
        # stream is dropping often.
        self.abandoned = 0

        self._status_base = "idle"
        self._connecting_since = None

        self._frame = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._force_reconnect = threading.Event()
        self._thread = None

        # _last_rx is the monotonic time of the most recent decoded frame, used
        # by _run to spot a stall. _generation identifies the current connection:
        # a reader thread whose generation is stale has been abandoned and must
        # not publish frames or status over the connection that replaced it.
        self._last_rx = 0.0
        self._generation = 0

    # -- public API -----------------------------------------------------------

    @property
    def status_text(self):
        """Human-readable state. While connecting, includes elapsed seconds.

        cv2.VideoCapture() blocks while it opens an RTSP URL - if the robot Pi is
        powered down that can be many seconds. Showing the counter tick makes it
        obvious the app is waiting on the network, not frozen.
        """
        if self._connecting_since is not None and not self.connected:
            return f"{self._status_base} {time.monotonic() - self._connecting_since:.0f}s"
        return self._status_base

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"rtsp-{self.name}", daemon=True
        )
        self._thread.start()
        if _STATS_PATH:
            threading.Thread(target=self._log_stats, name=f"rtsp-st-{self.name}",
                             daemon=True).start()

    def _log_stats(self):
        """Append one stats line per second - see _STATS_PATH."""
        last_frames = 0
        while not self._stop.wait(_STATS_INTERVAL):
            frames = self.frames_total
            delta = frames - last_frames
            last_frames = frames
            line = (f"{time.strftime('%H:%M:%S')} {self.name:6s} "
                    f"conn={int(self.connected)} fps={self.fps:5.1f} "
                    f"new={delta:3d} total={frames:7d} "
                    f"reconn={self.reconnects:3d} aband={self.abandoned:3d} "
                    f"status={self.status_text}\n")
            try:
                with _stats_lock, open(_STATS_PATH, "a", encoding="utf-8") as fh:
                    fh.write(line)
            except OSError:
                return

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def reconnect(self):
        """Force the worker to tear down and reopen the stream."""
        self._force_reconnect.set()

    def latest(self):
        """Return (frame, seq). frame is None until the first decode.

        seq increments per decoded frame - compare it against the last value you
        saw to skip redundant work when the camera is slower than the GUI.
        """
        with self._lock:
            return self._frame, self._seq

    # -- worker ---------------------------------------------------------------

    def _set_status(self, connected, text, timing=False):
        self.connected = connected
        self._status_base = text
        self._connecting_since = time.monotonic() if timing else None

    def _open(self):
        """Try GStreamer first, fall back to FFMPEG. Returns (cap, backend_name)."""
        # Never let both cameras open sessions at the same instant.
        _stagger_connects(self.connect_stagger)

        # Bail out fast if nothing is listening - see probe_reachable(). The
        # reason is recorded because _run backs off very differently for "the
        # camera is not there" than for "the camera is there and said no".
        if not probe_reachable(self.url, config.OPEN_TIMEOUT_S):
            self._absent = True
            return None, "-"
        self._absent = False

        if has_gstreamer():
            # Cached per URL: cameras can serve H.264 on one path and H.265 on
            # another, but never change codec between reconnects, so this costs
            # one DESCRIBE for the life of the process instead of one per retry.
            codec = cached_codec(self.url, config.OPEN_TIMEOUT_S) or "h264"
            self.codec = codec
            pipeline = gst_pipeline(
                self.url, self.latency_ms, self.protocols, self.hw_decode, codec,
                self.rotate,
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                # The pipeline already rotated - do not do it again in _reader.
                self._rotate_in_reader = False
                return cap, f"gstreamer/{codec}"
            cap.release()

        # FFMPEG fallback has no videoflip, so the reader has to do it.
        self._rotate_in_reader = True

        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except cv2.error:
                pass
            return cap, "ffmpeg"
        cap.release()
        return None, "-"

    def _run(self):
        # Grows on every failed attempt and resets as soon as a connection
        # actually delivers frames, so a working camera still reconnects fast
        # while a dead one is not hammered. See config.RECONNECT_MAX_DELAY_S.
        delay = self.reconnect_delay
        prev_reader = None

        while not self._stop.is_set():
            # An abandoned reader still owns an RTSP session on this camera, and
            # these cameras refuse a second one - so wait for the corpse before
            # asking for a new session. See config.SESSION_WAIT_S.
            if prev_reader is not None and prev_reader.is_alive():
                self._set_status(False, "releasing old session", timing=True)
                prev_reader.join(timeout=self.session_wait)
                prev_reader = None

            self._set_status(False, "connecting", timing=True)
            cap, backend = self._open()

            if cap is None:
                # Count failed opens too, not just dropped-mid-stream sessions -
                # otherwise a camera that never comes up reports 0 retries.
                self.reconnects += 1
                self._set_status(False, "no signal - retrying", timing=True)
                if self._stop.wait(delay):
                    break
                # An absent camera is polled briskly (one unanswered SYN costs
                # nothing and it may be rebooting); a camera that is present but
                # refused us gets the long backoff. See UNREACHABLE_MAX_DELAY_S.
                cap_delay = (self.unreachable_max_delay if self._absent
                             else self.reconnect_max_delay)
                delay = min(delay * 2, cap_delay)
                continue

            frames_at_open = self.frames_total
            self.backend = backend
            self._force_reconnect.clear()
            self._set_status(True, f"live ({backend})")

            # Hand the capture to a reader thread that owns it outright for this
            # connection's whole life - see _reader() for why nothing else may
            # touch it. This thread only supervises and never reads or releases.
            self._generation += 1
            gen = self._generation
            conn_stop = threading.Event()
            self._last_rx = time.monotonic()
            reader = threading.Thread(
                target=self._reader, args=(cap, gen, conn_stop),
                name=f"rtsp-rd-{self.name}-{gen}", daemon=True,
            )
            reader.start()

            while True:
                if self._stop.wait(0.2):
                    break
                if not reader.is_alive():
                    break               # reader hit read_fail_limit and gave up
                if self._force_reconnect.is_set():
                    break
                if (self.stall_timeout and self.stall_timeout > 0
                        and time.monotonic() - self._last_rx > self.stall_timeout):
                    self._set_status(False, "stream stalled")
                    break

            # Retire this connection. conn_stop asks the reader to finish and
            # release its own capture; if its read() is wedged it cannot notice
            # yet, so we wait briefly and otherwise walk away (see _reader).
            conn_stop.set()
            reader.join(timeout=self.abandon_timeout)
            if reader.is_alive():
                self.abandoned += 1
                # Carry it to the top of the loop, which waits for it to finish
                # releasing before opening a replacement session.
                prev_reader = reader

            self.fps = 0.0
            with self._lock:
                self._frame = None

            # A session that delivered frames was a real connection, not a
            # failing one - start the next backoff from scratch. A session that
            # opened but never decoded anything keeps escalating, because that
            # is the camera-is-drowning case the backoff exists for.
            delay = self.reconnect_delay if self.frames_total > frames_at_open \
                else min(delay * 2, self.reconnect_max_delay)

            if not self._stop.is_set():
                self.reconnects += 1
                self._stop.wait(delay)

        self._set_status(False, "stopped")

    def _reader(self, cap, gen, conn_stop):
        """Read frames until retired, then release the capture. Owns `cap`.

        This thread is the ONLY one that ever touches `cap`, and that is the
        whole point of the design. The previous version had a watchdog thread
        call cap.release() while this loop sat blocked inside cap.read(), to
        force the wedged read to return. It does force it to return - but
        releasing a VideoCapture concurrently with a read on it tears the
        GStreamer pipeline down underneath the reader, and the process dies with

            double free or corruption (out)
            Aborted

        which killed the entire viewer on every camera blip; .xinitrc then
        respawned it, giving a ~90s restart loop that looked like the cameras
        flapping. So a stalled connection is now ABANDONED, never killed: _run
        stops waiting on us and opens a fresh capture while we stay parked in
        the doomed read(). rtspsrc's tcp-timeout (see gst_pipeline) makes that
        read return within a few seconds, and we then release our own capture
        and exit. `gen` keeps an abandoned reader from publishing frames or
        status over the newer connection that replaced it.
        """
        fails = 0
        t_prev = time.monotonic()
        try:
            while not conn_stop.is_set() and not self._stop.is_set():
                ok, frame = cap.read()

                if not ok or frame is None:
                    fails += 1
                    if fails >= self.read_fail_limit:
                        if gen == self._generation:
                            self._set_status(False, "stream stalled")
                        return
                    time.sleep(0.01)
                    continue

                fails = 0
                # Only when GStreamer's videoflip was not available to do it -
                # see gst_pipeline(). Either way every consumer, panels AND
                # recorder, sees one orientation. See config.VIDEO_ROTATE.
                if self._rotate_in_reader and self._rotate_code is not None:
                    frame = cv2.rotate(frame, self._rotate_code)

                now = time.monotonic()
                dt = now - t_prev
                t_prev = now

                if gen != self._generation:
                    return          # abandoned - a newer reader owns this panel
                self._last_rx = now
                if dt > 0:
                    inst = 1.0 / dt
                    # exponential moving average so the readout doesn't jitter
                    self.fps = inst if self.fps == 0 else self.fps * 0.9 + inst * 0.1

                with self._lock:
                    self._frame = frame
                    self._seq += 1
                    self.frames_total += 1
        finally:
            try:
                cap.release()
            except Exception:
                pass


def describe_backends() -> str:
    """One-line summary of what this OpenCV build can do - printed at startup."""
    gst = "YES" if has_gstreamer() else "NO (falling back to FFMPEG)"
    return f"OpenCV {cv2.__version__} | GStreamer support: {gst}"
