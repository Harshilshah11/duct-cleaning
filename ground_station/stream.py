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
                 hw_decode: bool = False, codec: str = "h264") -> str:
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
    return (
        f"rtspsrc location={url} latency={latency_ms} protocols={protocols} ! "
        f"{depay} ! {parser} ! {decoder} ! "
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
                 hw_decode=False, reconnect_delay=2.0, read_fail_limit=50):
        self.name = name
        self.url = url
        self.latency_ms = latency_ms
        self.protocols = protocols
        self.hw_decode = hw_decode
        self.reconnect_delay = reconnect_delay
        self.read_fail_limit = read_fail_limit

        # Published state (plain attributes; reads are atomic enough for status text)
        self.connected = False
        self.backend = "-"
        self.codec = "-"
        self.fps = 0.0
        self.frames_total = 0
        self.reconnects = 0

        self._status_base = "idle"
        self._connecting_since = None

        self._frame = None
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._force_reconnect = threading.Event()
        self._thread = None

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
        # Bail out fast if nothing is listening - see probe_reachable().
        if not probe_reachable(self.url, config.OPEN_TIMEOUT_S):
            return None, "-"

        if has_gstreamer():
            # Detect the codec once per connection attempt; cameras can serve
            # H.264 on one path and H.265 on another.
            codec = probe_codec(self.url, config.OPEN_TIMEOUT_S) or "h264"
            self.codec = codec
            pipeline = gst_pipeline(
                self.url, self.latency_ms, self.protocols, self.hw_decode, codec
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                return cap, f"gstreamer/{codec}"
            cap.release()

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
        while not self._stop.is_set():
            self._set_status(False, "connecting", timing=True)
            cap, backend = self._open()

            if cap is None:
                # Count failed opens too, not just dropped-mid-stream sessions -
                # otherwise a camera that never comes up reports 0 retries.
                self.reconnects += 1
                self._set_status(False, "no signal - retrying", timing=True)
                if self._stop.wait(self.reconnect_delay):
                    break
                continue

            self.backend = backend
            self._force_reconnect.clear()
            self._set_status(True, f"live ({backend})")
            fails = 0
            t_prev = time.monotonic()

            while not self._stop.is_set() and not self._force_reconnect.is_set():
                ok, frame = cap.read()

                if not ok or frame is None:
                    fails += 1
                    if fails >= self.read_fail_limit:
                        self._set_status(False, "stream stalled")
                        break
                    time.sleep(0.01)
                    continue

                fails = 0
                now = time.monotonic()
                dt = now - t_prev
                t_prev = now
                if dt > 0:
                    inst = 1.0 / dt
                    # exponential moving average so the readout doesn't jitter
                    self.fps = inst if self.fps == 0 else self.fps * 0.9 + inst * 0.1

                with self._lock:
                    self._frame = frame
                    self._seq += 1
                    self.frames_total += 1

            cap.release()
            self.fps = 0.0
            with self._lock:
                self._frame = None

            if not self._stop.is_set():
                self.reconnects += 1
                self._stop.wait(self.reconnect_delay)

        self._set_status(False, "stopped")


def describe_backends() -> str:
    """One-line summary of what this OpenCV build can do - printed at startup."""
    gst = "YES" if has_gstreamer() else "NO (falling back to FFMPEG)"
    return f"OpenCV {cv2.__version__} | GStreamer support: {gst}"
