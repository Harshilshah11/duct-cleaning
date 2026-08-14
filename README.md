# Ground Station — Dual RTSP Camera Test

Bring-up code for Steps 3–4 of the robot ground station guide: the robot-side Pi
(Raspberry Pi OS Lite) serves two USB cameras as RTSP, and the ground station Pi 4
displays both side by side.

This is the **camera test stage only** — no joystick, no UDP command packets, no
Arduino. Prove video works first, then layer Steps 5–9 on top.

```
robot Pi (Lite, .30)                     30m Cat6                ground station Pi 4 (.10)
  USB cam0 ─┐                               │                       ┌─ CAM 1 panel
            ├─ ffmpeg → MediaMTX :8554 ─────┼── switch ──────────── ┤
  USB cam2 ─┘                               │                       └─ CAM 2 panel
                                            └── Arduino (.20)  ← not used yet
```

The rig on the bench is wired differently: `cameras.txt` points at two **IP
cameras** on eth0 — CAM 1 on `192.168.1.103`, CAM 2 on `192.168.1.101` — not at
the robot Pi's USB cameras. See [Connecting the IP cameras to
eth0](#connecting-the-ip-cameras-to-eth0). The robot Pi is then only what the
top bar's ROBOT chip probes.

## Quick start — Pi 4 with Raspberry Pi OS Lite as the viewer

Two RTSP streams come in, both show on the Pi's screen. Three steps:

```bash
# 1. copy the folder to the Pi
scp -r ground_station pi@<pi-ip>:~/

# 2. set your two stream URLs (one per line)
nano ~/ground_station/cameras.txt

# 3. one command installs everything and enables autostart
cd ~/ground_station && chmod +x setup_viewer.sh && ./setup_viewer.sh
```

`setup_viewer.sh` installs OpenCV+GStreamer, PySide6 and a **minimal X server**
(Lite has no display server, so a Qt window cannot open without one), then wires
up console autologin so the viewer appears on the screen by itself at boot. It
also replaces the text boot with the Arnobot logo — see [Boot screen](#boot-screen).

Check the streams without a display, over SSH:

```bash
python3 ~/ground_station/check_streams.py
```

Show the window now, without rebooting:

```bash
startx
```

## Boot screen

The Pi comes up on the Arnobot logo and stays there until the video appears — no
rainbow square, no kernel log, no login prompt, no bare X root. `setup_viewer.sh`
does this for you; run it alone to (re-)apply just the boot screen:

```bash
cd ~/ground_station && chmod +x setup_splash.sh && ./setup_splash.sh
```

**Four different things print to that screen**, and each needs its own knob —
miss one and it shows through the logo:

| What you'd otherwise see | Silenced by | Where |
|---|---|---|
| Rainbow test pattern | `disable_splash=1` | `config.txt` |
| Raspberry logos, top-left | `logo.nologo` | `cmdline.txt` |
| Kernel log | `quiet` + `console=tty3` | `cmdline.txt` |
| Login banner and MOTD | empty `/etc/issue` + `~/.hushlogin` | — |
| `My IP address is …` | `agetty --skip-login --noissue` | `getty@tty1` drop-in |
| `login: arnobot (automatic login)` | same drop-in | `getty@tty1` drop-in |

The last two are the ones that bite. Blanking `/etc/issue` looks sufficient
until you watch a real boot: Raspberry Pi OS also ships `/etc/issue.d/IP.issue`,
which `agetty` concatenates after `/etc/issue`, and `agetty` prints its own
autologin line as well. Both land on tty1 in the gap between Plymouth quitting
and X starting, straight over the retained logo. `--noclear` is equally
load-bearing — without it `agetty` clears the screen and the retained splash
goes with it.

Plymouth draws the logo from early boot. The handover to X is the awkward part,
and it takes three separate pieces to make it seamless.

**1. Plymouth draws the logo** from ~5s until `plymouth-quit` at ~11s, which
`setup_splash.sh` overrides to `plymouth quit --retain-splash`.

**2. `backdrop.py` is mapped first under X**, drawing the same logo on the same
gradient. With no window manager, stacking is just mapping order, so everything
later — the viewer and its loading screen — sits above it.

**3. `main.py` puts the logo up again as a Qt loading screen** with a live
status line while it imports cv2 and connects.

The backdrop is what makes every *restart* seamless too: the supervise loop in
`.xinitrc` reveals the logo rather than a bare root, for as long as the viewer
takes to come back.

### The one gap that is still there

There is a **~2–4 second black window between `plymouth-quit` (~11s) and X
starting**, and it is not for want of trying. Both obvious fixes were measured
on this Pi and both are worse than the gap:

- **`--retain-splash` does not actually hold the picture on a KMS Pi.** When
  `plymouthd` releases DRM master the CRTC is left with no mode set, so nothing
  is being scanned out at all until X sets one. The screen is black regardless
  of what is in the framebuffer — writing the logo straight into `/dev/fb0` at
  that moment, both with `cat` and through an `mmap` (unit reporting success, X
  confirmed not yet running), still dumped back solid black.
- **Never letting Plymouth quit does hold the logo, and breaks the boot.**
  `plymouthd` owns tty1 while it lives, so the autologin `login(1)` on that tty
  never completes: measured `login -- arnobot` still sitting there two minutes
  in, with no shell, no `startx` and no viewer behind the logo.

Closing it properly means keeping a DRM client alive across the gap, which means
starting X from a systemd unit instead of a console login. That is a real change
to how the ground station boots and is deliberately not done here.

That last stage is not cosmetic padding: `import cv2` plus PySide6 takes a couple
of seconds on a Pi 4, and the RTSP connect takes a couple more.

**Changing the logo** — replace `assets/arnobot_logo.png` and re-run
`./setup_splash.sh`. Use a tightly cropped PNG with a transparent background; it
is drawn at 46% of the screen width in both the boot and app splashes.

**Tuning the app's loading screen** (all environment variables, as usual):

| Variable | Default | Meaning |
|---|---|---|
| `SPLASH` | `1` | `0` = straight to the window, no loading screen |
| `SPLASH_MIN_S` | `1.5` | never flash past quicker than this |
| `SPLASH_PARTIAL_S` | `3.5` | hand over once *some* cameras are up |
| `SPLASH_MAX_S` | `8.0` | hand over regardless, dead cameras and all |
| `LOGO_PATH` | `assets/arnobot_logo.png` | image for both splashes |

`Esc` or `Space` skips the wait. Preview the loading screen on its own, without
the cameras or a reboot:

```bash
python3 splash.py                       # the app's loading screen
sudo plymouthd --mode=boot --tty=tty1 && sudo plymouth --show-splash
sleep 8; sudo plymouth quit             # the boot screen
```

**Getting the text boot back.** `SKIP_SPLASH=1 ./setup_viewer.sh` installs the
viewer without touching the boot screen — useful while debugging, when you want
to see what the Pi is complaining about. If it is already applied, every file the
script edits was copied to `*.arnobot-backup` first, and `setup_splash.sh` prints
the exact restore commands when it finishes.

Everything below is the fuller two-Pi setup from the guide.

## Top bar

```
[ARNOBOT] │ GROUND CONTROL STATION   ● CAM 1 CONNECTED  ● CAM 2 NO SIGNAL │ ● ROBOT CONNECTED  ● TEMPERATURE 52°C │ 11:50:51
```

- **One chip per camera** — green on CONNECTED, amber while connecting, red on
  NO SIGNAL. Hover a chip for that camera's RTSP URL. No frame rate: a number
  ticking between 24 and 26 all day is harder to read at a glance than a state,
  and `check_streams.py` reports the rate when you actually want it.
- **The ROBOT chip answers a different question** and is probed separately
  (`link.py`), because the cameras are standalone IP cameras on their own
  addresses: live video proves nothing about the robot, and a reachable robot
  proves nothing about the cameras. Green = it answered a TCP connect, amber =
  no result yet, red = it did not answer. The address being probed is
  deliberately **not** on the bar — set it with `ROBOT_LINK_HOST` /
  `ROBOT_LINK_PORT` below, or watch it live with `python3 link.py`.
- **The TEMPERATURE chip is this machine's SoC temperature**, not the robot's — the
  ground station Pi is the one whose thermals you can do anything about, and
  nothing on the robot Pi serves a reading (`thermal.py`). Green below 70°C,
  amber to 80°C, red above it: the Pi soft-throttles at 80°C and hard-throttles
  at 85°C, and the resulting stutter looks exactly like a network fault, so it
  is worth being able to rule out at a glance. A dash means no sensor — normal
  when running the viewer on a Windows or macOS desktop.
- **The bar wears the boot theme** — the same white → `#e8edf6` field the
  Plymouth splash and `splash.py` paint, so power-on to running viewer never
  changes colour, and the logo is drawn in its own brand navy rather than
  restamped in a tint. Retheme one of the three and the other two need the same
  edit (`setup_splash.sh`, `splash.py`, `topbar.py`). The bar is 54px tall,
  sized around a 42px logo with 6px of air. Three things shed as the window
  narrows, in order of what you can most afford to lose: the title below
  **1150px**, the TEMPERATURE chip below **950px**, and the logo itself below
  **900px** — at 42px tall it is ~188px wide, and on an 800x480 panel that is
  the difference between the camera chips fitting and CAM 1 reading
  `CONNECTEE`. Brand loses to data: on a screen that small the chips *are* the
  interface.

| Variable | Default | Meaning |
|---|---|---|
| `ROBOT_LINK_HOST` | `$ROBOT_PI_IP` (`192.168.1.30`) | what the ROBOT chip probes |
| `ROBOT_LINK_PORT` | `$RTSP_PORT` (`8554`) | port on that host |
| `LINK_POLL_S` | `2.0` | seconds between probes |
| `LINK_TIMEOUT_S` | `1.5` | per-probe TCP timeout |
| `TEMP_POLL_S` | `2.0` | seconds between SoC temperature reads |
| `GS_TITLE` | `GROUND CONTROL STATION` | wordmark next to the logo |
| `TOPBAR_LOGO_TINT` | *(empty)* | brand colours; set a colour to restamp the logo |

The probe target is MediaMTX's RTSP port only because that is the one thing
listening on the robot Pi today — point it at the Arduino once Step 8's sketch
answers on `192.168.1.20`. Each poll opens and closes a TCP connection, which
MediaMTX writes a log line for; raise `LINK_POLL_S` if that noise bothers you.

Try both without cameras, a robot, or a reboot:

```bash
python3 topbar.py     # the bar alone, cycling through every state
python3 link.py       # watch the robot probe from a terminal, over SSH
python3 thermal.py    # one SoC temperature reading
```

## Connecting the IP cameras to eth0

Both cameras hang off eth0: **CAM 1 on `192.168.1.103`, CAM 2 on
`192.168.1.101`**. They sit on `192.168.1.x` while the Pi's WiFi is *also*
`192.168.1.0/24`, so do **not** give eth0 a normal `/24` address. That creates a
second connected route for the same subnet and the kernel starts sending replies
out the wrong interface — SSH and internet break in ways that are hard to
diagnose.

```bash
./setup_camera_link.sh 192.168.1.103 192.168.1.101
ETH_IP=192.168.1.2 PROFILE="Wired connection 1" ./setup_camera_link.sh 192.168.1.103
```

That script gives eth0 a **`/32`** address (a `/32` creates no connected route at
all) plus a `/32` host route **per camera** — more specific than wlan0's `/24`,
so only camera traffic uses eth0. It persists to the NetworkManager profile and
sets `arp_ignore`, so the Pi won't answer ARP for that address over WiFi.

List **every** camera in one invocation. The persistent step has to clear
`ipv4.routes` before writing it (otherwise re-running stacks duplicate routes),
so naming one camera removes the other.

**Finding a camera whose IP you don't know.** It's on a different subnet, so
scanning won't reach it. Sniff instead — cameras announce themselves via ARP:

```bash
sudo tcpdump -i eth0 -n -e "arp or (udp and (port 67 or port 68))"
# 00:c2:33:71:13:6d > ff:ff:ff:ff:ff:ff, Request who-has 192.168.1.103 tell 192.168.1.103
```

**Finding the RTSP path.** Don't trust `DESCRIBE` status codes — the test camera
returned `200 OK` for *every* path, including nonsense ones. Only pulling video
proves anything:

```bash
for P in /stream /stream1 /live /video /11 /Streaming/Channels/101; do
  echo -n "$P: "
  ffprobe -v error -rtsp_transport tcp -select_streams v:0 \
    -show_entries stream=codec_name,width,height,avg_frame_rate \
    -of default=noprint_wrappers=1:nokey=1 "rtsp://192.168.1.103:554$P" 2>&1 | tr '\n' ' '
  echo
done
```

**Codecs can differ per stream.** The test camera served H.264 on `/stream`
(1280x720) and **H.265** on `/stream1` (720x576). `stream.py` detects this per
connection by reading the SDP, and picks the matching depayloader/decoder — a
hard-coded H.264 pipeline silently fails on the H.265 path.

`cameras.txt` assumes CAM 2 (`192.168.1.101`) is the same model as CAM 1 and
serves `/stream` too. Confirm that with the loop above before trusting it — the
paths are the one thing that varies between otherwise identical cameras.

## Layout

| Path | Runs on | What it is |
|---|---|---|
| `robot_pi/install_robot_streams.sh` | robot Pi (Lite) | Installs MediaMTX + ffmpeg, enables on boot |
| `robot_pi/mediamtx.yml` | robot Pi | Publishes `/dev/video0` → `cam1`, `/dev/video2` → `cam2` |
| `robot_pi/mediamtx.service` | robot Pi | systemd unit |
| `robot_pi/publish_test_pattern.sh` | anywhere | Fake streams, so you can test the GUI today |
| `ground_station/install.sh` | ground station | Installs OpenCV+GStreamer and PySide6 |
| `ground_station/setup_viewer.sh` | ground station | One-command install + autostart + boot screen |
| `ground_station/setup_splash.sh` | ground station | Boot screen only (Plymouth theme, quiet boot) |
| `ground_station/check_streams.py` | ground station | **Headless** check — works over SSH |
| `ground_station/main.py` | ground station | The two-panel GUI |
| `ground_station/run_ground_station.pyw` | Windows desktop | Console-less launcher — logo first, no terminal |
| `ground_station/topbar.py` | ground station | Top bar: logo, camera chips, robot link, temperature, clock |
| `ground_station/link.py` | ground station | Robot reachability probe behind the link chip |
| `ground_station/thermal.py` | ground station | SoC temperature behind the TEMPERATURE chip |
| `ground_station/splash.py` | ground station | The logo loading screen |
| `ground_station/backdrop.py` | ground station | Persistent logo layer behind everything |
| `ground_station/config.py` | ground station | IPs, URLs, latency |
| `ground_station/stream.py` | ground station | Threaded RTSP capture |
| `ground_station/assets/arnobot_logo.png` | ground station | Logo used by both splashes |

Copy each folder to the matching Pi, e.g.:

```bash
scp -r robot_pi        pi@192.168.1.30:~/
scp -r ground_station  pi@192.168.1.10:~/
```

---

## 1. Robot-side Pi (the Lite install)

**Disable WiFi/Bluetooth** — the whole system is wired, and this frees CPU:

```bash
sudo nano /boot/firmware/config.txt     # Bookworm path; older OS uses /boot/config.txt
# add at the end:
#   dtoverlay=disable-wifi
#   dtoverlay=disable-bt
sudo reboot
```

**Static IP** (Bookworm Lite uses NetworkManager, not dhcpcd):

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.1.30/24
sudo nmcli con up "Wired connection 1"
ip -4 addr show eth0
```

**Confirm both cameras enumerate** — you should see two devices:

```bash
ls /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext   # check MJPEG @ 640x480 exists
```

> Most UVC webcams claim **two** `/dev/video*` nodes each (capture + metadata),
> which is why the config uses `video0` and `video2`, not `video0`/`video1`.
> Confirm with `--list-devices` and adjust if yours differ.

**Install and start the streams:**

```bash
cd ~/robot_pi
chmod +x install_robot_streams.sh
./install_robot_streams.sh
```

Verify on the robot Pi itself before touching the ground station:

```bash
ffprobe -rtsp_transport tcp rtsp://127.0.0.1:8554/cam1
journalctl -u mediamtx -f
```

---

## 2. Ground station Pi 4

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.1.10/24
sudo nmcli con up "Wired connection 1"
ping -c3 192.168.1.30

cd ~/ground_station
chmod +x install.sh
./install.sh
```

**Test headless first** (works over SSH, no display needed):

```bash
python3 check_streams.py
```

Expect:

```
OpenCV 4.6.0 | GStreamer support: YES
── CAM 1  rtsp://192.168.1.30:8554/cam1
   OK    148 frames in 5s  | 640x480 | ~29.8 fps | backend=gstreamer
── CAM 2  rtsp://192.168.1.30:8554/cam2
   OK    147 frames in 5s  | 640x480 | ~29.6 fps | backend=gstreamer
2/2 stream(s) working.
```

**Then the GUI:**

```bash
python3 main.py
```

| Key | Action |
|---|---|
| `F` | Fullscreen |
| `S` | Snapshot both panels → `~/snapshots/` |
| `R` | Reconnect both streams |
| `1` / `2` / `0` | Solo CAM 1 / CAM 2 / show both |
| `Q` or `Esc` | Quit |

There are no on-screen buttons: the window is the top bar and the two panels and
nothing else. Each panel is a name strip over the picture — no resolution, frame
rate or backend readout, since those are bring-up numbers that never change
while a stream is up. The keys above are the whole interface, which is what the
ground station wants anyway, since there is no mouse on it.

**Filling the screen — `VIDEO_ZOOM`.** The cameras are 16:9 and two panels side
by side on a 1920x1080 screen are only ~945px wide, so the picture is ~531px
tall and a third of the screen is black. `VIDEO_ZOOM` trades frame for coverage:

| `VIDEO_ZOOM` | Picture on a 1920x1080 screen | Cost |
|---|---|---|
| `0` | 945x531, whole frame | ~480px of black, split top and bottom |
| `0.2` *(default)* | 1181x664 | 20% of the frame's width cropped away |
| `0.35` | 1454x818 | 35% cropped |
| `0.48` and up | 1796x1010, panel filled | ~47% of the width gone |

It is a crop, not a magnifying glass — whatever it hides, the operator never
sees, which is why it does not default to filling the screen. `VIDEO_ZOOM=0
python3 main.py` to check what is outside the frame.

---

## Testing the GUI before the cameras are ready

On the ground station itself:

```bash
# terminal 1
mediamtx
# terminal 2
./publish_test_pattern.sh
# terminal 3
python3 main.py rtsp://127.0.0.1:8554/cam1 rtsp://127.0.0.1:8554/cam2
```

---

## If the ground station is also Pi OS Lite

Lite has **no display server**, so `main.py` cannot open a window — you'll get
`qt.qpa.plugin: could not connect to display`. Either:

- Use `check_streams.py` over SSH (no GUI needed), **or**
- Install a minimal X session:
  ```bash
  sudo apt install -y --no-install-recommends xserver-xorg xinit x11-xserver-utils openbox
  startx /usr/bin/python3 ~/ground_station/main.py
  ```
- Or install the full desktop: `sudo apt install -y raspberrypi-ui-mods`

The **robot-side** Pi should stay Lite — it never renders anything.

---

## Running the viewer on a Windows desktop

Handy for working on the UI without occupying the Pi. The GUI half runs
unchanged; only the video half depends on what the machine can reach.

```powershell
pip install PySide6 opencv-python
python main.py                     # from a terminal, with the startup log
```

Or double-click **`run_ground_station.pyw`** to launch it the way the Pi does:
`.pyw` is bound to `pythonw.exe`, the same interpreter without a console, so the
Arnobot logo is the first thing that appears and the viewer replaces it — no
black terminal window at any point. It starts windowed rather than fullscreen,
since a desktop has a window manager and the Pi's reason for forcing the
geometry does not apply; `START_FULLSCREEN=1` overrides that and `F` toggles.

Having no console cuts both ways: an unhandled exception has nowhere to print
and the window would just vanish, so the launcher writes the traceback to
`ground_station/crash.log` instead.

Three differences from the Pi:

- **`GStreamer support: NO`** — the PyPI wheel has no GStreamer, so every stream
  uses the FFMPEG backend. Fine for looking at the UI, higher latency than the
  Pi's apt build, and not worth chasing on a desktop.
- **The TEMPERATURE chip shows a dash** — there is no `/sys/class/thermal` on
  Windows, and no elevation-free substitute (`MSAcpi_ThermalZoneTemperature`
  returns *Access denied*, `Win32_TemperatureProbe` is empty). A dash beats a
  wrong number; the real reading appears when the viewer runs on the Pi.
- **The cameras are probably unreachable.** They hang off the ground station
  Pi's eth0 segment, which is the entire point of the `/32` host routes: a
  desktop on the WiFi side cannot see them, however similar the addresses look.
  Confirm with `python3 check_streams.py`, then tunnel one through the Pi if you
  want a picture:

  ```powershell
  ssh -L 8554:192.168.1.103:554 arnobot@<ground-station-pi>
  python main.py rtsp://127.0.0.1:8554/stream
  ```

  Keep `RTSP_PROTOCOL=tcp` (the default) — the tunnel carries TCP only.

---

## Troubleshooting

**`GStreamer support: NO`** — you have the PyPI OpenCV wheel, which is built
without GStreamer. The code falls back to FFMPEG (works, but higher latency). Fix:

```bash
pip3 uninstall opencv-python opencv-python-headless
sudo apt install --reinstall python3-opencv
```

**`backend=ffmpeg` when GStreamer support says YES** — the GStreamer pipeline
failed to build and the code fell back. This fails *quietly*: FFMPEG still shows
picture, but it buffers, so you get bursts at a fraction of the real frame rate.
Measured on this system: FFMPEG delivered 30 frames per 5s from a 15 fps source,
GStreamer delivered 58. Two causes, both verified on a Pi 4 / GStreamer 1.26:

- **`decodebin` does not work under OpenCV.** Its output is a dynamic pad and
  OpenCV's manual-pipeline path can't wait for it — the pipeline dies with
  `Internal data stream error` and `isOpened()` returns False. `decodebin`,
  `decodebin ! queue` and `uridecodebin` were all measured failing. Use the
  explicit `rtph264depay ! h264parse ! avdec_h264` chain instead. Confusingly the
  same `decodebin` pipeline runs perfectly under `gst-launch-1.0`, so testing
  there proves nothing about OpenCV.
- **Never set `name=` on the appsink.** OpenCV finds the sink by its
  auto-assigned default name `appsink0`; any other name yields
  `cannot find appsink in manual pipeline`.

Check which backend you actually got — `check_streams.py` prints it per camera.

**No frames / NO SIGNAL, but ping works** — the stream isn't being published.
Check `systemctl status mediamtx` and `journalctl -u mediamtx -f` on the robot Pi.
A common cause is ffmpeg failing to open the camera because `-input_format mjpeg`
isn't supported; remove that flag in `mediamtx.yml` and restart.

**Both panels show the same camera** — `/dev/video*` numbering swapped on reboot.
Use the persistent paths from `ls -l /dev/v4l/by-id/` in `mediamtx.yml`.

**Video is laggy or drifts behind** — keep `RTSP_PROTOCOL=tcp`, lower resolution
in `mediamtx.yml`, and confirm `-g 30` is present so keyframes arrive every second.
On a Pi 4 ground station you can try hardware decode:

```bash
USE_HW_DECODE=1 python3 main.py
```

(Pi 5 has no hardware H.264 decoder — leave this off there.)

**One camera works, both together stutter** — two USB cameras on one Pi frequently
exceed a single USB 2.0 bus's bandwidth in uncompressed mode. Make sure both are
using MJPEG (`-input_format mjpeg`), and drop to 640x480 if needed.

**Different IPs?** No need to edit code:

```bash
ROBOT_PI_IP=192.168.1.31 python3 main.py
```

---

## Next steps (guide Steps 5–9)

Once both panels are live, add to `main.py`:
- a second `QTimer` at 50 Hz reading the joystick via `pygame.joystick`
- `send_command()` over UDP to `192.168.1.20:5005`
- the Arduino watchdog sketch (Step 8) — **test the 300 ms failsafe stop before
  putting wheels on the ground**

Keep the joystick timer separate from the video timer, as the guide specifies, so
a slow frame never delays a motor command. The threaded capture in `stream.py`
already guarantees decode never blocks the Qt event loop.
