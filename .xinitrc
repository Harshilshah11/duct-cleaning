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
    STREAM_STATS_LOG="$HOME/stream_stats.log" \
        python3 "/home/arnobot/ground_station/main.py" >> "$HOME/viewer.log" 2>&1
    [ -f "$HOME/.viewer-stop" ] && { rm -f "$HOME/.viewer-stop"; break; }
    sleep 3
done
