#!/usr/bin/env python3
"""
Ground station SoC temperature, for the top bar's TEMPERATURE chip.

A Pi 4 starts soft-throttling at 80°C and hard-throttles at 85°C. When that
happens frames arrive late and stutter — which looks exactly like a network or
camera problem, and is the one cause you cannot diagnose by staring at the
video. Putting the number on the bar turns that into one glance.

This reads the temperature of the machine the viewer is running on, i.e. the
GROUND STATION Pi. It is deliberately not the robot's: nothing on the robot Pi
serves a temperature, and a chip that silently reports a different machine than
its label suggests is the exact mistake the separate ROBOT chip exists to avoid
(see link.py).

Unlike link.py this needs no background thread. A TCP connect to a dead host can
block for ~2 minutes; this is a few microseconds of sysfs and cannot block, so
main.py's timer calls it inline.

Check it from a terminal:

    python3 thermal.py
"""

# thermal_zone0 is the bcm2835 SoC sensor on a Pi, in millidegrees. vcgencmd
# reports the same number but costs a subprocess per reading.
SYSFS_PATH = "/sys/class/thermal/thermal_zone0/temp"


def read_c(path=SYSFS_PATH):
    """SoC temperature in °C, or None if this machine has no such sensor.

    None is the normal answer on a Windows or macOS desktop — the file is simply
    not there — and the bar shows a dash instead of a wrong number.
    """
    try:
        with open(path, "r", encoding="ascii") as fh:
            millidegrees = int(fh.read().strip())
    except (OSError, ValueError):
        return None
    return millidegrees / 1000.0


def main():
    """Read it once:  python3 thermal.py"""
    celsius = read_c()
    if celsius is None:
        print(f"no sensor at {SYSFS_PATH} — not a Pi?")
    else:
        print(f"{celsius:.1f} °C")


if __name__ == "__main__":
    main()
