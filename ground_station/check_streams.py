#!/usr/bin/env python3
"""
Headless RTSP check — run this BEFORE the GUI.

Needs only OpenCV (no PySide6, no display), so it works over SSH on Pi OS Lite.
It opens each camera, pulls frames for a few seconds and reports what it got.

    python3 check_streams.py
    python3 check_streams.py rtsp://192.168.1.30:8554/cam1

Exit code 0 = every stream delivered frames, 1 = at least one failed.
"""

import sys
import time

import cv2

import config
from stream import RTSPStream, describe_backends

# Real IP cameras need a couple of seconds before the first frame: the RTSP
# handshake, then waiting for the next keyframe (measured ~2.1s on the test
# camera). A 5s window left almost no time to actually count frames and made
# working streams look broken.
TEST_SECONDS = 12.0


def check(name, url):
    print(f"\n-- {name}  {url}")
    stream = RTSPStream(
        name=name,
        url=url,
        latency_ms=config.RTSP_LATENCY_MS,
        protocols=config.RTSP_PROTOCOL,
        hw_decode=config.USE_HW_DECODE,
    )
    stream.start()

    deadline = time.monotonic() + TEST_SECONDS
    resolution = None
    while time.monotonic() < deadline:
        frame, _ = stream.latest()
        if frame is not None and resolution is None:
            h, w = frame.shape[:2]
            resolution = f"{w}x{h}"
        time.sleep(0.1)

    # Snapshot the state BEFORE stopping, or the reason is always "stopped".
    frames = stream.frames_total
    fps = stream.fps
    backend = stream.backend
    reason = stream.status_text
    retries = stream.reconnects
    stream.stop()

    if frames == 0:
        print(f"   FAIL  no frames decoded  ({reason}, {retries} retries)")
        return False

    print(f"   OK    {frames} frames in {TEST_SECONDS:.0f}s  "
          f"| {resolution} | ~{fps:.1f} fps | backend={backend}")
    return True


def main():
    urls = sys.argv[1:]
    cameras = (
        [(f"CAM {i + 1}", u) for i, u in enumerate(urls)] if urls else config.CAMERAS
    )

    print(describe_backends())
    results = [check(name, url) for name, url in cameras]

    ok = sum(results)
    print(f"\n{ok}/{len(results)} stream(s) working.")
    if ok < len(results):
        print("\nTroubleshooting:")
        print(f"  1. ping {config.ROBOT_PI_IP}")
        print(f"  2. on the robot Pi:  systemctl status mediamtx")
        print(f"  3. from here:        ffplay -rtsp_transport tcp {config.CAM1_URL}")
        print("  4. check the camera actually enumerated:  ls /dev/video*")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
