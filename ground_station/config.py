"""
Central configuration for the ground station.

Every value can be overridden with an environment variable, so you can point the
GUI at a different Pi without editing code:

    ROBOT_PI_IP=192.168.1.31 python3 main.py
"""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# --- Network -----------------------------------------------------------------
# Static IPs from the guide (Step 2):
#   Ground Station Pi 4 ......... 192.168.1.10
#   Arduino + Ethernet Shield ... 192.168.1.20
#   Robot-side Pi (cameras) ..... 192.168.1.30
ROBOT_PI_IP = os.environ.get("ROBOT_PI_IP", "192.168.1.30")
RTSP_PORT = int(os.environ.get("RTSP_PORT", "8554"))

# Fallback URLs for the guide's two-Pi layout (cameras served by the robot Pi).
# The rig in use does NOT work that way any more - it runs two IP cameras on
# their own addresses, listed in cameras.txt, which overrides everything here.
CAM1_URL = os.environ.get("CAM1_URL", f"rtsp://{ROBOT_PI_IP}:{RTSP_PORT}/cam1")
CAM2_URL = os.environ.get("CAM2_URL", f"rtsp://{ROBOT_PI_IP}:{RTSP_PORT}/cam2")

# (label, url) pairs rendered left-to-right in the GUI.
CAMERAS = [
    ("CAM 1", CAM1_URL),
    ("CAM 2", CAM2_URL),
]

# Simplest way to set your two streams: edit cameras.txt next to this file,
# one RTSP URL per line. If that file exists it wins over everything above.
_CAM_FILE = os.path.join(_HERE, "cameras.txt")
if os.path.exists(_CAM_FILE):
    with open(_CAM_FILE, encoding="utf-8") as fh:
        _urls = [
            line.strip()
            for line in fh
            if line.strip() and not line.lstrip().startswith("#")
        ]
    if _urls:
        CAMERAS = [(f"CAM {i + 1}", url) for i, url in enumerate(_urls)]
        CAM1_URL = CAMERAS[0][1]
        CAM2_URL = CAMERAS[1][1] if len(CAMERAS) > 1 else CAM1_URL

# --- Robot link ----------------------------------------------------------------
# Drives the CONNECTED / DISCONNECTED chip in the top bar. This is deliberately
# NOT derived from the camera URLs: the cameras are standalone IP cameras on
# 192.168.1.10x, so their reachability says nothing about the robot. A chip
# labelled ROBOT that is really reporting on a camera is worse than no chip.
#
# The only thing listening on the robot Pi today is MediaMTX, hence the RTSP
# port. Point this at the Arduino once Step 8's sketch answers on 192.168.1.20.
ROBOT_LINK_HOST = os.environ.get("ROBOT_LINK_HOST", ROBOT_PI_IP)
ROBOT_LINK_PORT = int(os.environ.get("ROBOT_LINK_PORT", str(RTSP_PORT)))

# Each poll opens and closes a TCP connection, which MediaMTX writes a line to
# its log for. Raise this if that noise bothers you - the chip just goes stale
# by that much longer.
LINK_POLL_S = float(os.environ.get("LINK_POLL_S", "2.0"))
LINK_TIMEOUT_S = float(os.environ.get("LINK_TIMEOUT_S", "1.5"))

# --- Ground station health ---------------------------------------------------
# How often the top bar re-reads this machine's SoC temperature. Unlike the robot
# probe above this is a sysfs file read, not a network round trip, so it is
# essentially free - the interval exists only because doing it at the 30fps UI
# rate would be pointless. See thermal.py.
TEMP_POLL_S = float(os.environ.get("TEMP_POLL_S", "2.0"))

# --- RTSP tuning -------------------------------------------------------------
# rtspsrc jitter buffer. 200ms measured best against a real 25fps IP camera;
# 50ms starves the decoder and 500ms is no better, see gst_pipeline() for the
# full measurements. Every ms here is added glass-to-glass delay, so do not
# raise it without measuring.
RTSP_LATENCY_MS = int(os.environ.get("RTSP_LATENCY_MS", "200"))

# "tcp" is far more reliable on a long tether (no silent packet loss).
# Switch to "udp" only if you are chasing the absolute lowest latency.
RTSP_PROTOCOL = os.environ.get("RTSP_PROTOCOL", "tcp")

# Hardware H.264 decode (v4l2h264dec). LEAVE THIS OFF unless you re-measure.
#
# It sounds like an obvious win and is not. Measured on this Pi 4 against the
# 720p25 camera, with the viewer stopped so the baseline was 0.1% CPU:
#
#   software avdec_h264 .... 25.1 fps, 14.7% system CPU   <-- better on both
#   hardware v4l2h264dec ... 13.7 fps, 20.4% system CPU
#
# The V4L2 M2M decoder hands back NV12 in DMA buffers, and converting those to
# the BGR that OpenCV requires costs more CPU than the software decode ever
# saved - while also halving the frame rate. Software decode of one 720p stream
# is only ~0.6 of a core, so there is nothing to rescue here.
USE_HW_DECODE = os.environ.get("USE_HW_DECODE", "0") == "1"

# --- Behaviour ---------------------------------------------------------------
# Start fullscreen. On the ground station there is NO window manager (X runs the
# app directly from .xinitrc), so a normal window just sits at its requested size
# in the top-left corner with dead space around it - nothing exists to maximise
# it. Set START_FULLSCREEN=0 when developing on a desktop.
START_FULLSCREEN = os.environ.get("START_FULLSCREEN", "1") == "1"

UI_FPS = int(os.environ.get("UI_FPS", "30"))  # GUI repaint rate

# How much of the frame you allow to be cropped so the picture covers more of
# the panel. The cameras are 16:9 and two of them side by side on a 1920x1080
# screen can only be ~945 wide, hence ~531 tall - which leaves a third of the
# screen black above and below the video.
#
#   0.0 ... whole frame, letterboxed (nothing hidden, biggest black bands)
#   0.2 ... 20% of the frame's width is cropped away, picture is 25% taller
#   1.0 ... picture covers the panel completely, ~45% of the width is gone
#
# It is a *crop*, not a zoom out of thin air: whatever this hides, the operator
# never sees. Raise it for a fuller screen, drop it to 0 to see everything the
# camera does.
VIDEO_ZOOM = max(0.0, min(1.0, float(os.environ.get("VIDEO_ZOOM", "0.2"))))

# Wordmark in the top bar, next to the logo.
TITLE = os.environ.get("GS_TITLE", "GROUND CONTROL STATION")

# --- Loading screen ----------------------------------------------------------
# The Arnobot logo covers the whole startup: X comes up on a bare black root,
# then cv2 + PySide6 import (~2s on a Pi 4), then the RTSP connect. Set SPLASH=0
# when developing, so the window appears immediately.
SPLASH_ENABLED = os.environ.get("SPLASH", "1") == "1"
LOGO_PATH = os.environ.get("LOGO_PATH", os.path.join(_HERE, "assets", "arnobot_logo.png"))

SPLASH_MIN_S = float(os.environ.get("SPLASH_MIN_S", "1.5"))   # don't flash past
SPLASH_MAX_S = float(os.environ.get("SPLASH_MAX_S", "8.0"))   # hand over regardless
# If one camera is dead, don't sit on the logo for the full timeout waiting for
# it — show the UI (with NO SIGNAL on that panel) once the others are up.
SPLASH_PARTIAL_S = float(os.environ.get("SPLASH_PARTIAL_S", "3.5"))
RECONNECT_DELAY_S = 2.0                        # wait before retrying a dead stream
READ_FAIL_LIMIT = 50                           # consecutive bad reads -> reconnect

# How long to wait for the RTSP port to accept a TCP connection before declaring
# NO SIGNAL. On a direct LAN the round trip is sub-millisecond, so 4s is generous.
# This bounds the "tether unplugged" case - without it the OS TCP connect can
# hang for ~2 minutes because an unplugged cable produces no ARP reply at all.
OPEN_TIMEOUT_S = float(os.environ.get("OPEN_TIMEOUT_S", "4.0"))

SNAPSHOT_DIR = os.path.expanduser(os.environ.get("SNAPSHOT_DIR", "~/snapshots"))

# --- Recording ---------------------------------------------------------------
# Driven by the two panel switches (see inputs.py): switch 1 on GPIO24 is
# START/STOP, switch 2 on GPIO23 is PAUSE/RESUME. Each run gets its own
# directory under RECORD_DIR, with one file per camera inside it.
RECORD_DIR = os.path.expanduser(os.environ.get("RECORD_DIR", "~/recordings"))

# Frames are re-encoded from the ones already decoded for the screen, so this is
# a straight CPU cost on top of the decode: ~0.4 of a core per 720p camera at 15.
# It is also the *wall-clock* sample rate, not just the header value - recorder.py
# writes one frame per tick whether or not the camera delivered a new one, so an
# hour of duct run is an hour of video and the timeline stays honest.
RECORD_FPS = float(os.environ.get("RECORD_FPS", "15"))

# "mp4v" (MPEG-4 Part 2) is the one fourcc the apt OpenCV can always write into
# a .mp4 without an external encoder. "avc1" gives smaller files where the build
# has libx264, and dies with a warning where it does not - so it is not default.
RECORD_FOURCC = os.environ.get("RECORD_FOURCC", "mp4v")
RECORD_EXT = os.environ.get("RECORD_EXT", ".mp4")

# 0 keeps the camera's native size. Set e.g. 960 to shrink 720p before encoding
# if the Pi runs out of CPU with both cameras recording.
RECORD_MAX_WIDTH = int(os.environ.get("RECORD_MAX_WIDTH", "0"))

# Stop writing rather than fill the SD card. A full root filesystem does not
# just lose the recording - it takes X, the viewer and this SSH session with it,
# which is a far worse failure than a truncated video.
RECORD_MIN_FREE_MB = int(os.environ.get("RECORD_MIN_FREE_MB", "512"))
