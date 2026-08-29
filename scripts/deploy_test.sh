#!/bin/bash
# Waits for the apt install to finish, then brings up the test-pattern RTSP
# server and verifies both streams decode. Runs on the Pi.

echo "=== waiting for apt install to finish ==="
for i in $(seq 1 300); do
    if grep -q SETUP_EXIT_CODE "$HOME/setup.log" 2>/dev/null; then break; fi
    sleep 10
done
grep SETUP_EXIT_CODE "$HOME/setup.log" 2>/dev/null || echo "TIMED OUT waiting for setup"

echo
echo "=== what landed ==="
python3 -c 'import cv2; print("cv2", cv2.__version__)' 2>/dev/null || echo "cv2: MISSING"
python3 -c 'import PySide6; print("PySide6", PySide6.__version__)' 2>/dev/null || echo "PySide6: MISSING"
ffmpeg -version 2>/dev/null | head -1 || echo "ffmpeg: MISSING"
command -v startx >/dev/null && echo "startx: present" || echo "startx: MISSING"

echo
echo "=== OpenCV GStreamer support (critical for low-latency RTSP) ==="
python3 -c "import cv2; print([l.strip() for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])" 2>/dev/null

echo
echo "=== starting MediaMTX test patterns ==="
echo 'arnobot' | sudo -S -v 2>/dev/null
sudo systemctl enable --now mediamtx 2>&1 | tail -2
sleep 20
sudo systemctl is-active mediamtx
echo "--- ffmpeg publishers ---"
pgrep -af "ffmpeg.*rtsp" | head -4 || echo "  none running"
echo "--- listening on 8554? ---"
ss -tlnp 2>/dev/null | grep 8554 || echo "  nothing on 8554"

echo
echo "=== decoding both streams ==="
python3 "$HOME/ground_station/check_streams.py"
echo "CHECK_EXIT=$?"
