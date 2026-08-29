#!/bin/bash
# Correlate motor demand (read off the wire) against camera reachability.
# Proves or disproves "joystick movement kills the cameras" without anyone
# having to command the motors from this side.
OUT=/home/arnobot/motor_cam_correlation.log
: > "$OUT"

# Capture the UDP command payloads. The sketch's format is
#   CMD <seq> M <L> <R> <ACT> <BRUSH> <LIGHT>
# so a non-zero L or R means the wheels are actually being driven.
echo arnobot | sudo -S timeout 200 tcpdump -i eth0 -n -l -A "udp port 5005 and greater 30" 2>/dev/null \
  | stdbuf -oL grep -oE "M -?[0-9]+ -?[0-9]+" \
  | stdbuf -oL awk '{ l=$2; r=$3; if (l<0) l=-l; if (r<0) r=-r; m=(l>r?l:r);
        if (m>0) print strftime("%H:%M:%S"), "MOTOR", m; fflush() }' >> "$OUT" &
TCPD=$!

# Camera reachability once a second.
for i in $(seq 1 190); do
  a=$(ping -c1 -W1 192.168.1.103 >/dev/null 2>&1 && echo UP || echo DOWN)
  b=$(ping -c1 -W1 192.168.1.102 >/dev/null 2>&1 && echo UP || echo DOWN)
  u=$(ping -c1 -W1 192.168.50.20 >/dev/null 2>&1 && echo UP || echo DOWN)
  echo "$(date +%H:%M:%S) CAMS cam1=$a cam2=$b uno=$u" >> "$OUT"
  sleep 1
done
wait $TCPD 2>/dev/null
echo "DONE" >> "$OUT"
