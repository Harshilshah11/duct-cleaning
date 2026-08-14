#!/usr/bin/env bash
# BRANDED BOOT SCREEN — Raspberry Pi OS Lite, ground station Pi.
#
# Everything the screen shows between power-on and the viewer window becomes the
# Arnobot logo. Four separate things print to that screen and each is silenced
# by a different knob:
#
#   rainbow test pattern .... disable_splash=1        (config.txt)
#   raspberry logos ......... logo.nologo             (cmdline.txt)
#   kernel log .............. quiet + console=tty3    (cmdline.txt)
#   login prompt / MOTD ..... blank /etc/issue + ~/.hushlogin
#   IP banner + autologin ... agetty --skip-login     (getty@tty1 drop-in)
#
# and Plymouth draws the logo over the top of the lot.
#
#   chmod +x setup_splash.sh && ./setup_splash.sh
#
# setup_viewer.sh calls this for you; run it directly to re-apply or after
# changing the logo. Safe to run twice. Undo notes are at the bottom.
set -euo pipefail

APP_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

LOGO="${LOGO_PATH:-$APP_DIR/assets/arnobot_logo.png}"
THEME_NAME="arnobot"
THEME_DIR="/usr/share/plymouth/themes/$THEME_NAME"

# Bookworm moved the boot partition; older releases still use /boot.
BOOT_DIR=/boot/firmware
[ -d "$BOOT_DIR" ] || BOOT_DIR=/boot
CMDLINE="$BOOT_DIR/cmdline.txt"
CONFIG="$BOOT_DIR/config.txt"

if [ ! -f "$LOGO" ]; then
    echo "!! logo not found: $LOGO" >&2
    echo "   Put a PNG there, or run with LOGO_PATH=/path/to/logo.png" >&2
    exit 1
fi

echo "==> Logo      : $LOGO"
echo "==> Boot files: $BOOT_DIR"
echo

# ---------------------------------------------------------------------------
# 1. Plymouth
# ---------------------------------------------------------------------------
echo "==> Installing Plymouth"
sudo apt update
sudo apt install -y plymouth plymouth-themes

# ---------------------------------------------------------------------------
# 2. The theme
# ---------------------------------------------------------------------------
echo "==> Building the '$THEME_NAME' theme"
sudo mkdir -p "$THEME_DIR"
sudo cp "$LOGO" "$THEME_DIR/logo.png"

sudo tee "$THEME_DIR/$THEME_NAME.plymouth" >/dev/null <<PLY
[Plymouth Theme]
Name=Arnobot
Description=Arnobot ground station boot splash
ModuleName=script

[script]
ImageDir=$THEME_DIR
ScriptFile=$THEME_DIR/$THEME_NAME.script
PLY

# The logo is dark navy, so the field behind it is light — on a black boot
# screen it would be all but invisible.
sudo tee "$THEME_DIR/$THEME_NAME.script" >/dev/null <<'SCRIPT'
Window.SetBackgroundTopColor(1.0, 1.0, 1.0);
Window.SetBackgroundBottomColor(0.91, 0.93, 0.96);

screen_width = Window.GetWidth(0);
screen_height = Window.GetHeight(0);

original = Image("logo.png");
scale = (screen_width * 0.46) / original.GetWidth();
logo = original.Scale(original.GetWidth() * scale, original.GetHeight() * scale);

sprite = Sprite(logo);
sprite.SetX((screen_width - logo.GetWidth()) / 2);
sprite.SetY((screen_height - logo.GetHeight()) / 2);
sprite.SetZ(10);

# A slow pulse, so a long fsck or a stalled service never reads as a freeze.
# Deliberately plain arithmetic - a counted triangle wave needs no Math.* and
# no modulo, so it behaves the same on every plymouth build.
tick = 0;
fun refresh_callback() {
    tick++;
    if (tick >= 120) tick = 0;
    step = tick;
    if (step > 60) step = 120 - step;
    sprite.SetOpacity(0.55 + step / 133);
}
Plymouth.SetRefreshFunction(refresh_callback);

# Swallow the boot chatter - nothing should print across the logo.
fun message_callback(text) { }
Plymouth.SetMessageFunction(message_callback);

fun quit_callback() { sprite.SetOpacity(1.0); }
Plymouth.SetQuitFunction(quit_callback);
SCRIPT

echo "==> Selecting the theme"
sudo plymouth-set-default-theme -R "$THEME_NAME" \
    || sudo plymouth-set-default-theme "$THEME_NAME"

# ---------------------------------------------------------------------------
# 3. Hold the logo through the console phase
# ---------------------------------------------------------------------------
# Plymouth normally exits at the end of boot and hands a bare console back,
# which flashes black for the seconds between then and startx.
#
# --retain-splash leaves the last frame on the framebuffer instead of handing
# back a bare console, so the logo stays put until X paints over it.
#
# KNOWN GAP, and read this before trying to close it, because both obvious fixes
# were measured on this Pi and both are worse:
#
#   - On a KMS Pi --retain-splash does not actually hold the picture. When
#     plymouthd releases DRM master the CRTC is left with no mode set, so
#     nothing is being scanned out at all until X sets one. The screen is black
#     for the ~2-4s between plymouth-quit and startx no matter what is in the
#     framebuffer: writing the logo straight into /dev/fb0 at that moment (both
#     with cat and through an mmap, service reporting success, X confirmed not
#     yet running) still showed black.
#   - Not letting plymouth quit at all DOES hold the logo, and breaks the boot:
#     plymouthd owns tty1 while it lives, so the autologin login(1) on that tty
#     never completes. Measured: `login -- arnobot` still sitting there two
#     minutes in, no shell, no startx, no viewer.
#
# Closing the gap properly means keeping a DRM client alive across it, which
# means X must start from a systemd unit rather than from a console login. That
# is a real change to how the ground station boots - deliberately not done here.
echo "==> Retaining the splash until X starts"
sudo mkdir -p /etc/systemd/system/plymouth-quit.service.d
sudo tee /etc/systemd/system/plymouth-quit.service.d/retain-splash.conf >/dev/null <<'UNIT'
[Service]
ExecStart=
ExecStart=-/usr/bin/plymouth quit --retain-splash
UNIT
sudo systemctl daemon-reload

# ---------------------------------------------------------------------------
# 4. cmdline.txt — quiet kernel, no logos, kernel log off the visible tty
# ---------------------------------------------------------------------------
echo "==> Patching $CMDLINE"
sudo cp -n "$CMDLINE" "$CMDLINE.arnobot-backup"   # -n: keep the pristine original
LINE="$(tr -d '\n' < "$CMDLINE")"

# Kernel messages go to tty3; tty1 is left clean for the splash and then X.
LINE="${LINE//console=tty1/console=tty3}"

has_word() { case " $LINE " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }
has_key()  { local w; for w in $LINE; do [ "${w%%=*}" = "$1" ] && return 0; done; return 1; }

add_word() { has_word "$1" || LINE="$LINE $1"; }
add_key()  { has_key "${1%%=*}" || LINE="$LINE $1"; }

add_word quiet                             # no kernel log
add_word splash                            # tell plymouth to show itself
add_word plymouth.ignore-serial-consoles
add_word logo.nologo                       # no raspberry logos in the corner
add_key  vt.global_cursor_default=0        # no blinking underscore
add_key  loglevel=3
add_key  systemd.show_status=false         # no green [ OK ] list

# cmdline.txt must stay a single line - a stray newline is silently ignored by
# the bootloader and every parameter after it is lost.
printf '%s\n' "$LINE" | sudo tee "$CMDLINE" >/dev/null

# ---------------------------------------------------------------------------
# 5. config.txt — kill the rainbow test pattern
# ---------------------------------------------------------------------------
echo "==> Patching $CONFIG"
sudo cp -n "$CONFIG" "$CONFIG.arnobot-backup"
if ! grep -q '^[[:space:]]*disable_splash=' "$CONFIG"; then
    printf '\n# Arnobot: no rainbow test pattern at power-on\ndisable_splash=1\n' \
        | sudo tee -a "$CONFIG" >/dev/null
fi

# ---------------------------------------------------------------------------
# 6. Silence the autologin console
# ---------------------------------------------------------------------------
# /etc/issue is the local console banner only (ssh uses issue.net), so blanking
# it costs nothing and stops "Raspberry Pi OS ... tty1" printing over the logo.
echo "==> Quieting the login banner"
sudo cp -n /etc/issue /etc/issue.arnobot-backup 2>/dev/null || true
printf '' | sudo tee /etc/issue >/dev/null
touch "$USER_HOME/.hushlogin"                     # no MOTD, no last-login line
chown "$USER_NAME":"$USER_NAME" "$USER_HOME/.hushlogin"

# ---------------------------------------------------------------------------
# 7. Silence agetty on tty1
# ---------------------------------------------------------------------------
# Blanking /etc/issue is NOT enough, which is easy to miss because the screen
# looks quiet right up until you actually watch a boot. Two more lines land on
# tty1 in the gap between plymouth quitting and X starting, painting text over
# the retained logo:
#
#   My IP address is 192.168.50.15 fe80::...   <- /etc/issue.d/IP.issue, which
#                                                 Raspberry Pi OS ships and
#                                                 agetty concatenates after
#                                                 /etc/issue (the addresses are
#                                                 agetty's \4 and \6 escapes)
#   raspberrypi login: arnobot (automatic login)  <- agetty's own prompt
#
# --skip-login drops the whole prompt, issue files included; --noissue is belt
# and braces. --noclear matters just as much: without it agetty clears the
# screen and takes the retained splash with it.
#
# This is a drop-in named to sort AFTER raspi-config's autologin.conf (systemd
# applies them alphabetically and the last ExecStart wins), so raspi-config's
# own file is left untouched and re-running it will not fight this.
echo "==> Silencing the autologin prompt on tty1"
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d
sudo tee /etc/systemd/system/getty@tty1.service.d/zz-arnobot-quiet.conf >/dev/null <<UNIT
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $USER_NAME --noclear --skip-login --noissue %I \$TERM
UNIT
sudo systemctl daemon-reload

# ---------------------------------------------------------------------------
echo
echo "Done. The logo now covers power-on -> viewer."
echo
echo "  See it without rebooting (it will sit for 8s):"
echo "      sudo plymouthd --mode=boot --tty=tty1 && sudo plymouth --show-splash"
echo "      sleep 8; sudo plymouth quit"
echo
echo "  For real:"
echo "      sudo reboot"
echo
echo "  To go back to a normal text boot:"
echo "      sudo cp $CMDLINE.arnobot-backup $CMDLINE"
echo "      sudo cp $CONFIG.arnobot-backup $CONFIG"
echo "      sudo rm -rf /etc/systemd/system/plymouth-quit.service.d"
echo "      sudo rm -f /etc/systemd/system/getty@tty1.service.d/zz-arnobot-quiet.conf"
echo "      sudo cp /etc/issue.arnobot-backup /etc/issue"
