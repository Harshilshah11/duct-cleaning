#!/usr/bin/env bash
# Robot-side Raspberry Pi OS Lite (192.168.1.30)
# Installs MediaMTX + ffmpeg and serves both USB cameras as RTSP on boot.
#
#   chmod +x install_robot_streams.sh && ./install_robot_streams.sh
#
# Result:
#   rtsp://192.168.1.30:8554/cam1
#   rtsp://192.168.1.30:8554/cam2
set -euo pipefail

ARCH_TAG="linux_arm64"   # use linux_armv7 if you flashed the 32-bit OS
[ "$(uname -m)" = "armv7l" ] && ARCH_TAG="linux_armv7"

echo "==> Installing ffmpeg and v4l tools"
sudo apt update
sudo apt install -y ffmpeg v4l-utils curl

echo
echo "==> Cameras currently detected"
v4l2-ctl --list-devices || true
echo
echo "    Persistent device paths (prefer these in mediamtx.yml):"
ls -l /dev/v4l/by-id/ 2>/dev/null || echo "    (none — are both cameras plugged in?)"

echo
echo "==> Downloading MediaMTX ($ARCH_TAG)"
TMP="$(mktemp -d)"
cd "$TMP"

# MediaMTX assets embed the version (mediamtx_v1.19.3_linux_arm64.tar.gz), so the
# usual /releases/latest/download/<name> shortcut 404s. Ask the API for the real
# asset URL instead of guessing it.
ASSET_URL="$(curl -fsSL -m 30 https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
    | grep -o "https://[^\"]*${ARCH_TAG}\.tar\.gz" | head -1)"
if [ -z "$ASSET_URL" ]; then
    echo "ERROR: could not resolve the MediaMTX download URL for $ARCH_TAG." >&2
    echo "       Check https://github.com/bluenviron/mediamtx/releases and download manually." >&2
    exit 1
fi
echo "    $ASSET_URL"

curl -fsSL -m 300 -o mediamtx.tar.gz "$ASSET_URL"
# A truncated download extracts a binary that segfaults on first run, so verify.
tar -tzf mediamtx.tar.gz >/dev/null 2>&1 || {
    echo "ERROR: download is incomplete or corrupt - re-run this script." >&2
    exit 1
}
tar -xzf mediamtx.tar.gz

sudo install -m 755 mediamtx /usr/local/bin/mediamtx
sudo mkdir -p /usr/local/etc

CONF_SRC="$(dirname "$(readlink -f "$0")")/mediamtx.yml"
if [ -f "$CONF_SRC" ]; then
    sudo install -m 644 "$CONF_SRC" /usr/local/etc/mediamtx.yml
    echo "==> Installed config from $CONF_SRC"
else
    sudo install -m 644 mediamtx.yml /usr/local/etc/mediamtx.yml
    echo "==> WARNING: using MediaMTX's default config — edit /usr/local/etc/mediamtx.yml"
fi
cd - >/dev/null
rm -rf "$TMP"

echo "==> Installing systemd service"
SERVICE_SRC="$(dirname "$(readlink -f "$0")")/mediamtx.service"
sudo install -m 644 "$SERVICE_SRC" /etc/systemd/system/mediamtx.service
sudo systemctl daemon-reload
sudo systemctl enable --now mediamtx

sleep 3
echo
echo "==> Service status"
sudo systemctl --no-pager --lines=15 status mediamtx || true

echo
echo "Done. Verify locally on this Pi:"
echo "  ffprobe -rtsp_transport tcp rtsp://127.0.0.1:8554/cam1"
echo "Then from the ground station:"
echo "  ffplay -rtsp_transport tcp rtsp://192.168.1.30:8554/cam1"
echo
echo "Logs:  journalctl -u mediamtx -f"
