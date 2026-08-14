#!/usr/bin/env python3
"""
Joystick -> motor demands -> Arduino Uno, over the Ethernet tether.

This is the drive layer main.py actually uses. It owns a thread and a UDP
socket (via uno_link.UnoLink) and turns stick positions into the per-wheel
commands uno_eth_link.ino applies:

    Pi  -> Uno   "CMD <seq> M <left> <right>\\n"   left/right = -255..255
    Uno -> Pi    "ACK <seq>\\n"

Arcade mixing happens HERE rather than on the Uno so steering can be retuned
without a reflash. mix() itself lives in uno_serial.py and is imported rather
than copied — the two transports must agree exactly on what a stick position
means, and two copies of that arithmetic would eventually disagree.

Same split as stream.py / inputs.py: this owns the hardware and a thread,
main.py owns the one UI timer and pushes samples in.

    from uno_motors import MotorLink
    self.motors = MotorLink()
    self.motors.start()
    ...
    joy = self.inputs.latest()["joy"]
    self.motors.set_joystick(joy["x"], joy["y"])

Bench test it standalone (works with main.py running — this only needs the
network, not the GPIO/ADC lines the viewer holds):

    python3 uno_motors.py --status     # prove the Uno answers
    python3 uno_motors.py --test       # drive each direction, WHEELS OFF GROUND

THINGS THAT COST TIME HERE:

  * The Uno must be 192.168.50.20, NOT 192.168.1.20. The Pi owns .1.20 on
    wlan0, and a host delivers traffic for its own address locally, so packets
    to an Uno at .1.20 never reach the wire at all.
  * UDP send() to a dead host does NOT raise. Silence is indistinguishable from
    success on the sending side, which is why acked/loss_pct is the only honest
    health signal here.
  * The sketch fails safe after 300 ms of silence, so this must keep sending
    even when the stick has not moved. SEND_HZ is a floor, not a rate limit.
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

from uno_link import UNO_HOST, UNO_PORT, UnoLink
# Shared with the USB serial transport so both agree on what a stick means.
from uno_serial import (MAX_PWM, SAMPLE_STALE_S, SEND_HZ, act_demand,
                        brush_demand, light_demand, mix)

# Health is judged over a window, like link.py debounces its status chip: a
# single lost datagram is normal and must not flip the link to "down".
ACK_WINDOW = 50
ACK_MIN_PCT = 20.0


class MotorLink(threading.Thread):
    """Owns the UDP socket and the send loop. Never raises at callers."""

    def __init__(self, host=UNO_HOST, port=UNO_PORT):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._link = None
        self._joy = (None, None)
        self._joy_at = 0.0
        self._act = (None, None)          # (switch state, pot %)
        self._act_at = 0.0
        self._brush = None                # panel TOGGLE, already decoded
        self._brush_at = 0.0
        self._light = None                # panel pot %, drives brightness
        self._light_at = 0.0
        self._recent = []                 # rolling window of per-frame ack counts
        self._state = {
            "ok": False, "error": "starting", "target": f"{host}:{port}",
            "left": 0, "right": 0, "act": 0, "brush": 0, "light": 0,
            "sent": 0, "acked": 0, "loss_pct": 0.0,
        }

    # -- public ---------------------------------------------------------------

    def set_joystick(self, x, y):
        """Push the newest stick sample. Cheap enough for the UI timer."""
        with self._lock:
            self._joy = (x, y)
            self._joy_at = time.monotonic()

    def set_actuator(self, state, pot_pct):
        """Push the newest actuator switch position and pot speed.

        Kept separate from set_joystick so a caller that only drives the wheels
        does not have to invent an actuator state -- but note the run loop ages
        this on its OWN clock, so a caller that stops pushing stops the rod.
        """
        with self._lock:
            self._act = (state, pot_pct)
            self._act_at = time.monotonic()

    def set_brush(self, toggle_closed):
        """Push the newest brush toggle state (inputs.py's decoded boolean)."""
        with self._lock:
            self._brush = toggle_closed
            self._brush_at = time.monotonic()

    def set_light(self, pot_pct):
        """Push the newest pot reading, as a percentage, for the light.

        Separate from set_actuator even though both once read the same knob: the
        rod no longer has a speed input, and a light that went dark because the
        ACTUATOR sample went stale would be a genuinely confusing failure.
        """
        with self._lock:
            self._light = pot_pct
            self._light_at = time.monotonic()

    def latest(self):
        with self._lock:
            return dict(self._state)

    def stop(self):
        self._stop.set()

    # -- internals ------------------------------------------------------------

    def run(self):
        try:
            self._link = UnoLink(self._host, self._port)
        except Exception as exc:
            with self._lock:
                self._state["error"] = f"socket failed: {exc}"
            return

        period = 1.0 / max(1.0, SEND_HZ)
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                try:
                    with self._lock:
                        x, y = self._joy
                        fresh = (time.monotonic() - self._joy_at) < SAMPLE_STALE_S
                        act_state, act_pot = self._act
                        act_fresh = (
                            time.monotonic() - self._act_at) < SAMPLE_STALE_S
                        brush_on = self._brush
                        brush_fresh = (
                            time.monotonic() - self._brush_at) < SAMPLE_STALE_S
                        light_pct = self._light
                        light_fresh = (
                            time.monotonic() - self._light_at) < SAMPLE_STALE_S
                    left, right = mix(x, y) if fresh else (0, 0)
                    # Aged independently of the stick: a reader that dies must
                    # stop the rod even if the last joystick sample was fine.
                    act = act_demand(act_state, act_pot) if act_fresh else 0
                    brush = brush_demand(brush_on) if brush_fresh else 0
                    # Stale pot -> dark, for the same reason a stale stick means
                    # stop: the last known value is not evidence of anything.
                    light = light_demand(light_pct) if light_fresh else 0

                    # wait_ack=False: blocking for the reply would cap the loop
                    # at 5 Hz and trip the sketch's own 300 ms failsafe.
                    self._link.send(
                        f"M {left} {right} {act} {brush} {light}",
                        wait_ack=False)
                    got = self._link.drain_acks()

                    self._recent.append(1 if got else 0)
                    del self._recent[:-ACK_WINDOW]
                    # Only judge once the window has filled, or the first few
                    # frames (sent, no reply yet) would read as a dead link.
                    if len(self._recent) >= ACK_WINDOW:
                        rate = 100.0 * sum(self._recent) / len(self._recent)
                        alive = rate >= ACK_MIN_PCT
                    else:
                        alive = any(self._recent)

                    with self._lock:
                        self._state["left"] = left
                        self._state["right"] = right
                        self._state["act"] = act
                        self._state["brush"] = brush
                        self._state["light"] = light
                        self._state["sent"] = self._link.sent
                        self._state["acked"] = self._link.acked
                        self._state["loss_pct"] = self._link.loss_pct
                        self._state["ok"] = alive
                        self._state["error"] = (
                            None if alive else "no ACK from the Uno")
                except Exception as exc:
                    with self._lock:
                        self._state["ok"] = False
                        self._state["error"] = f"send failed: {exc}"
                self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            # Last word is always neutral, and it is worth a blocking ACK wait
            # because nothing follows it to cover a loss.
            try:
                self._link.send("STOP")
            except Exception:
                pass
            try:
                self._link.close()
            except Exception:
                pass


def _status():
    print("target:", f"{UNO_HOST}:{UNO_PORT}")
    link = MotorLink()
    link.start()
    time.sleep(2.0)
    link.set_joystick(0.0, 0.0)
    time.sleep(1.5)
    s = link.latest()
    link.stop()
    time.sleep(0.3)
    print(f"sent={s['sent']}  acked={s['acked']}  loss={s['loss_pct']:.1f}%")
    if s["acked"]:
        print("LINK UP - the Uno is answering")
        return 0
    print("LINK DOWN -", s["error"])
    print("\nNothing answered. Check, in order:")
    print("  1. the sketch is flashed with ip(192,168,50,20) - NOT 192.168.1.20")
    print("  2. the Uno is powered and its shield's link LED is lit")
    print("  3. the LAN cable runs to the Pi's eth0")
    return 1


def _test():
    print("WHEELS OFF THE GROUND. Driving each direction, Ctrl-C to abort.")
    link = MotorLink()
    link.start()
    time.sleep(1.0)
    try:
        for label, (x, y) in (("FRONT", (0.0, 0.6)), ("BACK", (0.0, -0.6)),
                              ("LEFT", (-0.6, 0.0)), ("RIGHT", (0.6, 0.0))):
            print(f"  {label:6} -> {mix(x, y)}")
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                link.set_joystick(x, y)     # keep it fresh, or it goes stale
                time.sleep(0.05)
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                link.set_joystick(0.0, 0.0)
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        time.sleep(0.3)
    s = link.latest()
    print(f"\nsent={s['sent']} acked={s['acked']} loss={s['loss_pct']:.1f}%")
    return 0 if s["acked"] else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="joystick -> Uno motor link")
    ap.add_argument("--status", action="store_true", help="prove the Uno answers")
    ap.add_argument("--test", action="store_true", help="drive each direction")
    args = ap.parse_args()
    sys.exit(_test() if args.test else _status())
