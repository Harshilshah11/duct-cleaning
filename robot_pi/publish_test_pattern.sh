#!/usr/bin/env bash
# Publish two SYNTHETIC RTSP streams so you can test the ground station GUI
# before the robot Pi / cameras are ready.
#
# Run this on ANY machine with MediaMTX + ffmpeg — including the ground station
# Pi itself (then point the GUI at rtsp://127.0.0.1:8554/cam1).
#
#   1. Start MediaMTX with a default config in another terminal:  mediamtx
#   2. ./publish_test_pattern.sh
#   3. python3 ../ground_station/main.py rtsp://127.0.0.1:8554/cam1 \
#                                        rtsp://127.0.0.1:8554/cam2
#
# Ctrl-C stops both publishers.
set -euo pipefail

HOST="${1:-127.0.0.1}"
PORT="${2:-8554}"

publish() {
    local path="$1" pattern="$2"
    ffmpeg -nostdin -hide_banner -loglevel warning \
        -re -f lavfi -i "${pattern}=size=640x480:rate=30" \
        -f lavfi -i "sine=frequency=1000:sample_rate=48000" \
        -c:v libx264 -preset ultrafast -tune zerolatency \
        -profile:v baseline -pix_fmt yuv420p -g 30 -b:v 1500k \
        -an -f rtsp -rtsp_transport tcp "rtsp://${HOST}:${PORT}/${path}" &
}

echo "Publishing test patterns to rtsp://${HOST}:${PORT}/cam1 and /cam2"
publish cam1 testsrc
publish cam2 smptebars

trap 'echo; echo "stopping…"; kill $(jobs -p) 2>/dev/null; wait' INT TERM
wait
