#!/usr/bin/env bash
# Ground Station Pi 4 — install dependencies for the dual RTSP test viewer.
#
#   chmod +x install.sh && ./install.sh
#
# Use apt for OpenCV, NOT `pip install opencv-python`: the PyPI wheel is built
# WITHOUT GStreamer, so the low-latency RTSP pipeline silently fails and you fall
# back to the higher-latency FFMPEG path.
set -euo pipefail

echo "==> Updating package lists"
sudo apt update

echo "==> Installing OpenCV + GStreamer (RTSP decode)"
sudo apt install -y \
    python3-opencv python3-numpy \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav \
    ffmpeg

echo "==> Installing PySide6 (GUI)"
if apt-cache show python3-pyside6.qtwidgets >/dev/null 2>&1; then
    sudo apt install -y \
        python3-pyside6.qtcore python3-pyside6.qtgui python3-pyside6.qtwidgets
else
    echo "    apt package unavailable, using pip"
    pip3 install PySide6 --break-system-packages
fi

# If you flashed Raspberry Pi OS LITE on the GROUND STATION, there is no display
# server, so no Qt window can open. Uncomment ONE of the following:
#
#   Minimal X session (~200MB) — then launch with:  startx /full/path/to/run.sh
# sudo apt install -y --no-install-recommends xserver-xorg xinit x11-xserver-utils openbox
#
#   Full Raspberry Pi desktop (~1GB, simplest):
# sudo apt install -y raspberrypi-ui-mods
#
# The ROBOT-side Pi should stay Lite — it never renders anything.

echo
echo "==> Verifying OpenCV GStreamer support"
python3 - <<'PY'
import cv2
gst = [l for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
print("OpenCV", cv2.__version__)
print((gst[0].strip() if gst else "GStreamer: not listed"))
PY

echo
echo "Done. Next:"
echo "  python3 check_streams.py     # headless, works over SSH"
echo "  python3 main.py              # the GUI (needs a display)"
