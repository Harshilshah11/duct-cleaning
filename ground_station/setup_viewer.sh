#!/usr/bin/env bash
# ONE-COMMAND SETUP - Raspberry Pi OS Lite, Raspberry Pi 4.
#
# Turns a bare Lite install into a two-panel RTSP viewer that comes up on the
# screen by itself at boot.
#
#   chmod +x setup_viewer.sh
#   ./setup_viewer.sh
#
# Edit cameras.txt first if your stream URLs aren't the defaults.
set -euo pipefail

APP_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

echo "==> App directory : $APP_DIR"
echo "==> Autostart user: $USER_NAME"
echo

# ---------------------------------------------------------------------------
# 1. Video decode stack
# ---------------------------------------------------------------------------
# apt's python3-opencv is built WITH GStreamer. The PyPI wheel is NOT - if you
# pip install opencv-python instead, the low-latency RTSP path silently fails.
echo "==> Installing OpenCV + GStreamer"
sudo apt update
sudo apt install -y \
    python3-opencv python3-numpy \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav \
    ffmpeg

# ---------------------------------------------------------------------------
# 2. GUI toolkit
# ---------------------------------------------------------------------------
echo "==> Installing PySide6"
if apt-cache show python3-pyside6.qtwidgets >/dev/null 2>&1; then
    sudo apt install -y \
        python3-pyside6.qtcore python3-pyside6.qtgui python3-pyside6.qtwidgets
else
    pip3 install PySide6 --break-system-packages
fi

# ---------------------------------------------------------------------------
# 3. Minimal display server
# ---------------------------------------------------------------------------
# Lite has no display server at all, so no Qt window can open. This is the
# smallest thing that will host one - no desktop, no file manager, no panel,
# which leaves the CPU free for decoding two H.264 streams.
echo "==> Installing minimal X server"
sudo apt install -y --no-install-recommends \
    xserver-xorg xinit x11-xserver-utils xserver-xorg-legacy unclutter

# Allow starting X from a normal console login (Debian defaults to console-only).
sudo sed -i 's/^allowed_users=.*/allowed_users=anybody/' /etc/X11/Xwrapper.config 2>/dev/null || true
if ! grep -q '^allowed_users=' /etc/X11/Xwrapper.config 2>/dev/null; then
    echo 'allowed_users=anybody' | sudo tee -a /etc/X11/Xwrapper.config >/dev/null
fi
if ! grep -q '^needs_root_rights=' /etc/X11/Xwrapper.config 2>/dev/null; then
    echo 'needs_root_rights=yes' | sudo tee -a /etc/X11/Xwrapper.config >/dev/null
fi

# ---------------------------------------------------------------------------
# 4. Launch the viewer whenever X starts
# ---------------------------------------------------------------------------
echo "==> Writing $USER_HOME/.xinitrc"
cat > "$USER_HOME/.xinitrc" <<XINIT
#!/bin/sh
xset s off          # no screen blanking
xset -dpms          # no power management
xset s noblank
unclutter -idle 1 & # hide the mouse pointer

# The X root defaults to a black stipple, which is what you would stare at for
# the couple of seconds cv2 and PySide6 take to import. White matches the boot
# splash and the app's own loading screen, so the handover is invisible.
xsetroot -solid "#ffffff"

# The logo layer, mapped FIRST so everything else stacks above it (no window
# manager here, so stacking is just mapping order). This is what shows in every
# gap: X up but the viewer still importing, and every pass of the restart loop
# below. Same logo on the same gradient as Plymouth, so nothing visibly changes.
python3 "$APP_DIR/backdrop.py" &
sleep 1     # let it map before the viewer, or the viewer ends up underneath

# Plymouth has been holding the screen since power-on - setup_splash.sh masks
# plymouth-quit so it does NOT let go at the end of boot and flash a black
# console. X owns the display now and the backdrop is showing the same image,
# so this is the moment it is safe to retire plymouthd.
plymouth quit --retain-splash 2>/dev/null || true

# Supervise the viewer rather than 'exec'-ing it. With exec, the app IS the X
# session: if it ever crashes, X tears down and the screen goes dead with no
# recovery until someone power-cycles the Pi. This loop restarts it instead.
# Pressing Q in the app exits cleanly and leaves the loop (marker file), so you
# can still quit to a console for debugging.
while true; do
    python3 "$APP_DIR/main.py"
    [ -f "\$HOME/.viewer-stop" ] && { rm -f "\$HOME/.viewer-stop"; break; }
    sleep 3
done
XINIT
chmod +x "$USER_HOME/.xinitrc"
chown "$USER_NAME":"$USER_NAME" "$USER_HOME/.xinitrc"

# ---------------------------------------------------------------------------
# 5. Boot straight into it
# ---------------------------------------------------------------------------
# Console autologin + startx from .bash_profile is the standard Pi kiosk recipe.
# It's more reliable here than a systemd unit, which would race the X server.
echo "==> Enabling console autologin"
sudo raspi-config nonint do_boot_behaviour B2

PROFILE="$USER_HOME/.bash_profile"
touch "$PROFILE"
if ! grep -q "RTSP_VIEWER_AUTOSTART" "$PROFILE"; then
    cat >> "$PROFILE" <<'PROF'

# RTSP_VIEWER_AUTOSTART - start the viewer on the physical console only.
# The tty1 test keeps SSH sessions clean; without it every ssh login would
# try to launch X.
if [ -z "${DISPLAY:-}" ] && [ "$(tty)" = "/dev/tty1" ]; then
    startx >/dev/null 2>&1
fi
PROF
fi
chown "$USER_NAME":"$USER_NAME" "$PROFILE"

# ---------------------------------------------------------------------------
# 6. Branded boot screen
# ---------------------------------------------------------------------------
# Without this the screen shows the rainbow pattern, then the kernel log, then a
# login prompt before the viewer appears. SKIP_SPLASH=1 leaves the text boot
# alone (useful while debugging - you can see what the Pi is complaining about).
if [ "${SKIP_SPLASH:-0}" = "1" ]; then
    echo "==> Skipping the boot splash (SKIP_SPLASH=1)"
else
    echo "==> Installing the boot splash"
    chmod +x "$APP_DIR/setup_splash.sh"
    "$APP_DIR/setup_splash.sh"
fi

# ---------------------------------------------------------------------------
echo
echo "==> Streams configured in $APP_DIR/cameras.txt:"
grep -vE '^\s*(#|$)' "$APP_DIR/cameras.txt" | sed 's/^/    /'
echo
echo "Done."
echo
echo "  Test the streams now (works over SSH, no display needed):"
echo "      python3 $APP_DIR/check_streams.py"
echo
echo "  Start the viewer on the Pi's screen right now:"
echo "      startx"
echo
echo "  Or reboot - it will come up on its own:"
echo "      sudo reboot"
