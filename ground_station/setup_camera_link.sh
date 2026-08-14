#!/usr/bin/env bash
# Make the eth0 -> IP-camera link survive a reboot.
#
# THE PROBLEM
# The cameras ship on 192.168.1.10x, but the Pi's WiFi is also 192.168.1.0/24.
# Giving eth0 a normal 192.168.1.x/24 address would create a SECOND connected
# route for that subnet, and the kernel would start sending replies out the
# wrong interface - which breaks SSH and internet in ways that are painful to
# diagnose (it happened during bring-up).
#
# THE FIX
# Give eth0 a /32 address - a /32 creates no connected route at all - plus one
# /32 host route per camera. A /32 route is more specific than wlan0's /24, so
# ONLY camera traffic uses eth0 and everything else still goes over WiFi.
#
# Pass every camera on the command line. They are applied as a set, because the
# persistent step has to clear ipv4.routes before writing (otherwise re-running
# stacks duplicates) - so naming one camera here REMOVES the others.
#
#   ./setup_camera_link.sh                                # both defaults below
#   ./setup_camera_link.sh 192.168.1.103 192.168.1.101
#   ETH_IP=192.168.1.2 PROFILE="Wired connection 1" ./setup_camera_link.sh 192.168.1.103
set -euo pipefail

CAM_IPS=("$@")
if [ ${#CAM_IPS[@]} -eq 0 ]; then
    CAM_IPS=(192.168.1.103 192.168.1.101)
fi
ETH_IP="${ETH_IP:-192.168.1.2}"
PROFILE="${PROFILE:-Wired connection 1}"

echo "==> cameras  : ${CAM_IPS[*]}"
echo "==> eth0 /32 : $ETH_IP"
echo "==> profile  : $PROFILE"
echo

# --- runtime (immediate) ----------------------------------------------------
echo "==> applying now"
sudo ip addr add "${ETH_IP}/32" dev eth0 2>/dev/null || echo "    (address already present)"
for CAM_IP in "${CAM_IPS[@]}"; do
    sudo ip route replace "${CAM_IP}/32" dev eth0 src "$ETH_IP"
done

# --- persistent (survives reboot) -------------------------------------------
echo "==> writing to the NetworkManager profile"
# +ipv4.addresses appends, keeping the existing 192.168.50.15/24 tether address.
if ! nmcli -g ipv4.addresses connection show "$PROFILE" | grep -q "$ETH_IP"; then
    sudo nmcli connection modify "$PROFILE" +ipv4.addresses "${ETH_IP}/32"
fi
# The src= attribute is ESSENTIAL and easy to miss. Without it the kernel picks
# eth0's primary address (192.168.50.15) as the source, the camera receives
# packets from a subnet it cannot reply to, and the link silently fails on every
# reboot even though the route and address both look correct in `ip addr`.
# Clear any previously-added routes first so re-running cannot stack duplicates.
sudo nmcli connection modify "$PROFILE" ipv4.routes ""
for CAM_IP in "${CAM_IPS[@]}"; do
    sudo nmcli connection modify "$PROFILE" +ipv4.routes "${CAM_IP}/32 0.0.0.0 100 src=${ETH_IP}"
done

# eth0 must never provide the default route - that segment has no internet.
sudo nmcli connection modify "$PROFILE" ipv4.never-default yes

# Do not answer ARP for the /32 on the wrong interface. Without this the Pi
# would reply to ARP requests for $ETH_IP arriving over WiFi, which looks like
# an IP conflict if the router ever hands that address to another device.
echo "==> tightening ARP behaviour"
echo "net.ipv4.conf.all.arp_ignore = 1"  | sudo tee /etc/sysctl.d/99-camera-link.conf >/dev/null
echo "net.ipv4.conf.all.arp_announce = 2" | sudo tee -a /etc/sysctl.d/99-camera-link.conf >/dev/null
sudo sysctl -q -p /etc/sysctl.d/99-camera-link.conf

echo
echo "==> verifying"
ip -brief -4 addr show eth0
FAILED=0
for CAM_IP in "${CAM_IPS[@]}"; do
    ip route get "$CAM_IP" | head -1
    if ping -c2 -W3 "$CAM_IP" >/dev/null 2>&1; then
        echo "    $CAM_IP reachable: OK"
    else
        # Keep going: one dead camera should still leave the other one routed.
        echo "    $CAM_IP reachable: FAIL"
        FAILED=1
    fi
done
[ "$FAILED" -eq 0 ] || exit 1

echo
echo "Done. Reboot-safe. Verify the streams with:"
echo "    python3 ~/ground_station/check_streams.py"
