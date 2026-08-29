#!/bin/bash
# Pin wlan0 at a static 192.168.1.20, with auto-rollback to DHCP if the Pi
# ends up unable to reach the gateway. Run detached (systemd-run) so that the
# Wi-Fi bounce dropping your SSH session cannot kill it half-applied.
set -u
CON="ARS"
IP="192.168.1.20/24"
GW="192.168.1.1"
DNS="192.168.1.1"

exec >>/var/log/pin-wlan0.log 2>&1
echo "=== $(date -Is) applying static $IP to profile '$CON' ==="

nmcli con mod "$CON" \
    ipv4.method manual \
    ipv4.addresses "$IP" \
    ipv4.gateway "$GW" \
    ipv4.dns "$DNS" || { echo "FATAL: con mod failed"; exit 1; }

nmcli con up "$CON"
sleep 10

if ping -c 4 -W 2 "$GW" >/dev/null 2>&1; then
    echo "OK: gateway $GW reachable — static config kept"
    ip -4 -br addr show wlan0
else
    echo "FAIL: gateway $GW unreachable — rolling back to DHCP"
    nmcli con mod "$CON" \
        ipv4.method auto \
        ipv4.addresses "" \
        ipv4.gateway "" \
        ipv4.dns ""
    nmcli con up "$CON"
    sleep 8
    ip -4 -br addr show wlan0
fi
echo "=== $(date -Is) done ==="
