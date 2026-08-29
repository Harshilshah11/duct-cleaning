#!/bin/bash
cd "$HOME/ground_station" || exit 1
# Prime the sudo credential, then refresh it every 45s so a long apt run
# never stalls waiting for a password on a terminal that does not exist.
echo 'arnobot' | sudo -S -v 2>/dev/null
( while true; do sudo -n -v 2>/dev/null || exit 0; sleep 45; done ) &
KA=$!
./setup_viewer.sh
RC=$?
kill $KA 2>/dev/null
echo "===== SETUP_EXIT_CODE=$RC ====="