# pi_services — the parts of the ground station that are not the ground station

Three things the robot depends on that systemd runs, not `main.py`. They lived
only on the Pi until 2026-08-30, which meant a reimage would have taken them
silently — and one of them is load-bearing enough that the robot does not work
without it.

## Installing

```sh
sudo cp uno-arp.service uno-watch.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now uno-arp.service uno-watch.service
```

`uno-watch.service` expects the logger at `/home/arnobot/diag/uno_logger.py`;
the copy of record is `tools/uno_logger.py` in this repo.

`camproxy.py` is run from `/home/arnobot/camproxy.py` by its own unit.

---

## uno-arp.service — READ THIS ONE

**Without it the robot can be unreachable even though both ends are healthy.**

A wedged W5100 does not answer ARP, so the Pi cannot resolve 192.168.50.20, so
every control packet sits on an unresolved-neighbour queue and *nothing leaves
the Pi*. The Uno then hears silence — which is exactly the condition its own
recovery is waiting to break — and both ends sit waiting for the other.

Measured 2026-08-27: `tcpdump` showed the Pi ARPing once a second forever and not
one UDP frame going out.

The Uno's address is static and its MAC is compiled into the firmware, so ARP
buys nothing here anyway. Pinning it makes the Pi transmit unconditionally, which
is what the firmware's recovery needs in order to hear anything at all.

## uno-watch.service — the non-resetting serial recorder

Opens the Uno's USB serial with **DTR held low**, because opening it normally
pulses DTR and *warm-resets the board* — and a warm reset cures the very
deaf-shield fault the log exists to capture. Ordinary serial probes were healing
the patient mid-diagnosis for most of a day before that was understood.

It runs as a service rather than by hand because the interesting failures have
repeatedly landed either side of a Pi reboot, which killed the `nohup` version.

Its baud must match `SERIAL_BAUD` in the firmware's `Config.h`. A mismatch is not
a slow log, it is an unreadable one.

## camproxy.py — cameras reachable from a browser

Forwards `8103 -> 192.168.1.103:80` and `8102 -> 192.168.1.102:80`, so the camera
web UIs can be opened from a machine that is not on the robot segment. Only
needed for a human at a browser; nothing in the drive path uses it.

Note that `tools/camera_config.py` configures the cameras directly over their
HTTP API and does **not** need this proxy.
