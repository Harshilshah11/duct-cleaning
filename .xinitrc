#!/bin/sh
xset s off          # no screen blanking
xset -dpms          # no power management
xset s noblank
unclutter -idle 1 & # hide the mouse pointer

# White beats the default black stipple for the instant before the backdrop maps.
xsetroot -solid "#ffffff"

# The logo layer, mapped FIRST so everything else stacks above it (no window
# manager here, so stacking is just mapping order). This is what shows in every
# gap: X up but the viewer still importing, and every pass of the restart loop
# below. Same logo on the same gradient as Plymouth, so nothing visibly changes.
python3 "/home/arnobot/ground_station/backdrop.py" &
sleep 1     # let it map before the viewer, or the viewer ends up underneath

# Plymouth has been holding the screen since power-on - setup_splash.sh masks
# plymouth-quit so it does NOT let go at the end of boot and flash a black
# console. X owns the display now and the backdrop is showing the same image,
# so this is the moment it is safe to retire plymouthd.
plymouth quit --retain-splash 2>/dev/null || true

# Supervise the viewer rather than 'exec'-ing it: a crash restarts it instead of
# tearing down X. The backdrop stays up underneath the whole time.
#
# Output is redirected because the viewer's stdout/stderr otherwise go to tty1,
# where X is drawing - so a Python traceback or a GStreamer error scrolls into a
# console nobody can read and is gone forever. Diagnosing the "double free or
# corruption" abort that used to restart this loop every ~90s needed the console
# text buffer scraped out of /dev/vcs1, which only survives while the screen
# holds it. Keep the logs. STREAM_STATS_LOG adds one line per camera per second
# (fps / reconnects / abandoned readers) - see stream.py.
while true; do
    # NO USE_HW_DECODE=1 HERE ANY MORE. It was forcing the Pi's v4l2 hardware
    # H.264 decoder on, and that decoder drops more than half the frames on this
    # machine: the ground station's own stats logged CAM 1 at 12.0 fps against a
    # 25 fps source, with "v4l2h264dec: initial frames were not dequeued: bug in
    # decoder" from GStreamer. Benchmarked 2026-08-31 at 7.8 fps hardware against
    # 26.3 software, at every jitter-buffer setting.
    #
    # IT FAILS AS STUTTER, NOT AS AN ERROR, which is what let it survive: the
    # picture simply looks laggy and nothing in the log says why unless the
    # per-camera fps is being written out.
    #
    # AND AN ENV VAR HERE BEATS config.py, which is the part worth remembering.
    # USE_HW_DECODE was set back to 0 in config.py on 2026-08-31 and nothing
    # changed, because this line overrode it on every launch. Config defaults are
    # not the last word - this file is.
    STREAM_STATS_LOG="$HOME/stream_stats.log" \
        python3 "/home/arnobot/ground_station/main.py" >> "$HOME/viewer.log" 2>&1
    [ -f "$HOME/.viewer-stop" ] && { rm -f "$HOME/.viewer-stop"; break; }
    sleep 3
done
