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
import os
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

# Maximum change in wheel PWM per second - a slew-rate limit on the demand.
#
# Without this, pushing the stick steps the wheels from 0 to full PWM inside one
# 20 ms frame, which is the worst case for inrush current on a shared supply.
# The rig's reported symptom is that the cameras drop while the motors are being
# driven, and when they drop they stop answering ARP entirely (192.168.1.102
# INCOMPLETE) - which is a device losing power, not a network fault. The Uno on
# the same wire never misses a packet, and the Pi's own rail reads
# throttled=0x0, so the sag is on whatever feeds the cameras.
#
# Ramping cannot fix an undersized supply, but it removes the current STEP that
# triggers the sag, which is the part software controls.
#
# RAISED 0.4 -> 0.15 ON 2026-08-19, on the operator's report that the stick does
# not feel instant. This number IS the responsiveness, and it dominates
# everything else in the chain. Measured latency budget from stick to wheels:
#
#     ADS1115 bit-bang read (3 channels)     ~32 ms
#     this send loop at SEND_HZ              <=20 ms
#     Uno loop pause + W5x00 wait             ~2 ms  (was ~10, lowered same day)
#     THIS RAMP, 0 -> full scale             400 ms  <-- 85% of the total
#
# So 400 ms was the whole complaint. 0.15 makes it 150 ms, about 2.7x more
# responsive, while still spreading the inrush over ~7 send frames rather than
# stepping it in one.
#
# BE CAREFUL RAISING IT FURTHER. This limiter is the only thing blunting the
# current step into a driver rail that is ALREADY browning out - on
# 2026-08-19 the wheels and the light were measured blacking out together while
# the Uno, running on separate USB power, saw zero packet loss and never once
# failed safe. That is a supply fault, and a faster ramp feeds it. Fix the power
# before going below 0.15.
#
# Set MOTOR_SLEW_PER_S=0 to disable the limiter entirely (instant response, full
# inrush - bench use only).
MOTOR_SLEW_PER_S = float(os.environ.get("MOTOR_SLEW_PER_S", str(MAX_PWM / 0.15)))

# How old a snapshot may be before the demand behind it is treated as unknown.
#
# TWO clocks, because the snapshot is filled by TWO threads in inputs.py and
# either can die alone: the ADC can wedge while the GPIO loop is healthy, and
# stopping the rod because an unrelated analog bus went quiet would be a
# failsafe firing on the wrong evidence. ANALOG_STALE_S ages the stick and the
# pot; SWITCH_STALE_S ages the actuator and the brush.
#
# These live HERE rather than in main.py because the demand path no longer runs
# on the UI thread - see set_source(). ANALOG_STALE_S keeps its old environment
# variable name so an existing override still applies, and 0.4 s is still ten
# analog poll periods of slack. SWITCH_STALE_S is looser (20 polls at the
# switch loop's 20 Hz) because a switch position is a latched state rather than
# a continuously sampled one, so a couple of missed polls mean nothing.
ANALOG_STALE_S = float(os.environ.get("ANALOG_STALE_S", "0.4"))
SWITCH_STALE_S = float(os.environ.get("SWITCH_STALE_S", "1.0"))



def slew(prev, target, max_step):
    """Rate-limit a RISE in demand. Falling and stopping are never delayed.

    Safety property, and the reason this is not a plain clamp: a limiter that
    can slow down a STOP is a limiter that can run the robot into something.
    Any move toward zero is applied in full, immediately. A reversal is passed
    through zero first rather than slammed across, which is both the biggest
    current spike available and the hardest thing on the gearbox.

    THE REVERSAL CASE USED TO RETURN A HARD 0, AND THAT WAS A BUG (fixed
    2026-08-19). Returning 0 pinned the wheel at a dead stop for a whole frame,
    and because the caller stores what was actually sent, it also restarted the
    ramp from zero afterwards. Fast stick work flips a sign several times a
    second, so the robot was commanded to a DEAD STOP on 24% of frames -
    measured, on a forward/reverse waggle - which is exactly what "the bot
    suddenly switches off" feels like in the hand.

    The fix keeps the safety intent unchanged. Decelerating to zero is still
    free, so the crossing itself stays unlimited; what changes is that the frame
    which reaches zero now also carries the first rate-limited step into the NEW
    direction, instead of being pinned at 0 and only starting to ramp on the
    next frame.

    Measured on the same waggle: dead-stop frames 24% -> 0%, peak PWM UNCHANGED
    at 48 - so this adds no current draw at all, which matters because this rig
    has a supply that already sags under motor load. A full +255 -> -255
    reversal takes 440 ms, against 460 ms before.

    Every safety case is bit-identical to the old function: full stop from full
    speed, stop from reverse, slowing in either direction, rise from rest, and
    holding steady all return exactly what they returned before.

    DO NOT "fix" the remaining sluggishness here. Under a fast waggle the peak
    demand is capped at 48/255 by MOTOR_SLEW_PER_S itself (a 400 ms full-scale
    ramp), not by this function. Raising that rate is the knob, and it directly
    increases the current step that is currently browning out the driver rail -
    so fix the supply first.
    """
    if max_step <= 0:
        return target                       # limiter disabled
    # Checked BEFORE the magnitude test on purpose: an equal-and-opposite
    # reversal (+200 -> -200) has the same magnitude, so a magnitude-first test
    # waves it straight through - the exact case this guard exists to stop.
    reversing = (prev > 0 and target < 0) or (prev < 0 and target > 0)
    if not reversing and abs(target) <= abs(prev):
        return target                       # slowing or stopping - immediate
    # A reversal decelerates to zero for free and only then accelerates under
    # the limit, so ramp from zero rather than from the old signed demand.
    base = 0 if reversing else prev
    if target > base:
        return int(min(target, base + max_step))
    return int(max(target, base - max_step))


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
        self._source = None           # pull-model demand source, see set_source
        self._recent = []                 # rolling window of per-frame ack counts
        self._prev_left = 0               # last demand actually sent, for slew()
        self._prev_right = 0
        self._slew_at = 0.0               # monotonic stamp of the last ramp step
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

    def set_source(self, fn):
        """Pull demands from `fn()` on THIS thread instead of waiting for pushes.

        WHY THIS EXISTS, and it is the whole point of the class owning a thread.
        Every demand used to be pushed in from main.py's Qt timer, so the wheels,
        the rod and the brush all depended on the UI thread getting a turn inside
        SAMPLE_STALE_S (250 ms). That thread also refreshes the camera panels.
        Measured on this rig on 2026-08-19: 433 gaps of 2 s or more in the 1 Hz
        correlation log, which is driven by a timer on that same thread - so the
        demand went stale, the mixer produced (0, 0), and every one of those gaps
        stopped the robot dead and then made it crawl back up the 400 ms slew
        ramp when the UI caught up. The operator sees that as the stick going
        numb and the acceleration coming and going, which is what was reported.

        `fn` is inputs.InputReader.latest - a cheap locked dict copy off the
        reader threads, safe to call at SEND_HZ. Nothing in the drive path now
        touches Qt, so a stalled repaint costs a stale PANEL and nothing else.

        Pushing still works and still applies when no source is set, so the
        bench entry points below and the USB transport are unaffected.
        """
        with self._lock:
            self._source = fn

    def _pull(self, source):
        """One snapshot -> the same set_*() calls main.py used to make.

        Deliberately routed through the public setters rather than writing the
        fields directly: they carry the locking and the timestamps, and having
        one way to set a demand is what keeps the pull and push models honest.

        Never raises. A reader that throws must not be able to stop the send
        loop, because the send loop is what keeps the Uno's 300 ms failsafe from
        tripping - losing the demand is survivable, losing the carrier is not.
        """
        try:
            snap = source() or {}
        except Exception as exc:
            with self._lock:
                self._state["error"] = f"input source failed: {exc}"
            return

        now = time.time()
        joy = snap.get("joy") or {}
        pot_pct = (snap.get("pot") or {}).get("pct")
        # Past the limit the axes are forced to None, which mixes to a dead stop.
        # inputs.py stamps adc_updated on every pass INCLUDING one where every
        # channel was rejected, so this fires only for a reader that has
        # genuinely stopped, not for a noisy one.
        if now - (snap.get("adc_updated") or 0.0) > ANALOG_STALE_S:
            joy, pot_pct = {}, None
        self.set_joystick(joy.get("x"), joy.get("y"))
        # Stale pot -> dark, for the same reason a stale stick means stop: the
        # last known value is not evidence of anything.
        self.set_light(pot_pct)

        # The switch half, aged on its own clock. act_demand(None, ...) and
        # brush_demand(None) both return 0, so a dead switch loop cuts the rod's
        # enable and the brush rather than latching their last position - the
        # hole the old UI-thread push left open, because it re-asserted whatever
        # the snapshot still held regardless of whether anyone had refreshed it.
        if now - (snap.get("updated") or 0.0) > SWITCH_STALE_S:
            self.set_actuator(None, None)
            self.set_brush(None)
        else:
            self.set_actuator(snap.get("actuator"), pot_pct)
            self.set_brush((snap.get("switches") or {}).get("BRUSH"))


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
                    # Demands are PULLED here, on this thread, whenever a
                    # source is set - so a stalled UI repaint can no longer age
                    # the stick out from under the mixer. See set_source().
                    with self._lock:
                        source = self._source
                    if source is not None:
                        self._pull(source)
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
                    # Ramp the wheels rather than stepping them - see
                    # MOTOR_SLEW_PER_S. Uses the real frame time, so a jittery
                    # loop still ramps at the configured rate rather than a rate
                    # that depends on how busy the Pi happened to be.
                    now = time.monotonic()
                    dt = now - self._slew_at if self._slew_at else period
                    self._slew_at = now
                    max_step = MOTOR_SLEW_PER_S * max(0.0, min(dt, 0.5))
                    left = slew(self._prev_left, left, max_step)
                    right = slew(self._prev_right, right, max_step)
                    self._prev_left, self._prev_right = left, right
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


def _test(hold_s=5.0, gap_s=1.0, deflect=0.6, loop=False, only=None):
    """Drive each direction in turn with the joystick OUT of the loop.

    NOTHING HAS TO BE COMMENTED OUT TO GET HERE. set_joystick() below IS the
    bypass: no source is registered (see set_source), so the run loop consumes
    these synthetic samples and inputs.py, the ADC and the panel are never
    consulted. A direction that fails here is the mixer, the link, the Uno or
    the driver, and cannot be the stick -- which is the point of the split.

    Each sample is re-pushed every 50 ms because the run loop ages it at
    SAMPLE_STALE_S (250 ms): stop feeding it and the wheels stop, which is the
    same failsafe the panel relies on. A gap stage pushes a CENTRED stick for
    the same reason, so it is an active zero rather than an absence of demand.

    gap_s is not padding. The interesting failure is a wheel that drives one
    way and not the other, and a back-to-back reversal hides it behind slew()'s
    single pass-through-zero frame. --gap 0 gives a continuous square.
    """
    legs = f"{hold_s:g}s per leg"
    if gap_s > 0:
        legs += f", {gap_s:g}s stopped between"
    print(f"WHEELS OFF THE GROUND. {legs}. Ctrl-C to stop.")
    print(f"stick deflection {deflect:g} of full, MAX_PWM={MAX_PWM}\n")

    stages = (("FORWARD", (0.0, deflect)), ("BACK", (0.0, -deflect)),
              ("LEFT", (-deflect, 0.0)), ("RIGHT", (deflect, 0.0)))

    # One leg at a time, for splitting a fault that only one direction
    # shows. Filtered AFTER the table is built so the names and the
    # deflection stay defined in exactly one place.
    if only:
        want = only.strip().upper()
        names = [s[0] for s in stages]
        if want not in names:
            print('unknown direction %r -- pick one of %s'
                  % (only, ', '.join(names)))
            return 2
        stages = tuple(s for s in stages if s[0] == want)

    link = MotorLink()
    link.start()
    time.sleep(1.0)

    def hold(x, y, secs):
        deadline = time.monotonic() + secs
        while time.monotonic() < deadline:
            link.set_joystick(x, y)
            time.sleep(0.05)

    cycle = 0
    try:
        while True:
            cycle += 1
            if loop:
                print(f"-- cycle {cycle} --", flush=True)
            for label, (x, y) in stages:
                left, right = mix(x, y)
                print(f"  {label:8} stick=({x:+.2f},{y:+.2f})"
                      f" -> left={left:+4d} right={right:+4d}", flush=True)
                hold(x, y, hold_s)
                if gap_s > 0:
                    hold(0.0, 0.0, gap_s)
            if not loop:
                break
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        # Centre the stick before dropping the link, so the last demand on the
        # wire is a stop rather than whichever leg was interrupted.
        try:
            hold(0.0, 0.0, 0.3)
        except KeyboardInterrupt:
            pass
        link.stop()
        time.sleep(0.3)
    s = link.latest()
    print(f"\ncycles={cycle} sent={s['sent']} acked={s['acked']}"
          f" loss={s['loss_pct']:.1f}%")
    if not s["acked"]:
        print("NOTHING ACKED - the wheels were never the problem; fix the link"
              " first (python3 uno_motors.py --status)")
        return 1
    return 0


def _act_test(hold_s):
    """Drive the ROD ONLY, with the panel switch out of the loop.

    THIS EXISTS TO SPLIT ONE QUESTION INTO TWO. "The switch reads EXTEND but the
    rod does not move" has two halves -- does the demand reach the Uno's pins,
    and does the driver act on them -- and watching the panel cannot separate
    them. This drives act_demand() directly, so GPIO16/19, the ADC and the UI are
    all bypassed: whatever happens here is the Uno and everything downstream of
    it, nothing else.

    Probe D7 and D4 against the Uno's GND with a multimeter while each stage
    holds. The table printed below is what a correctly flashed board produces.
    D4 is the gate -- it is what makes the rod move at all -- and D7 only picks
    which way. So:

      * D4 never goes high  -> the board is not running this build. Check the
        reset banner says ACT_DIR=D7 ACT_PWM=D4.
      * D4 HIGH the whole time, including on STOP -> the board is running an OLD
        build that parks pin 4 high to deselect the shield's SD card. That park
        is what drove the rod continuously; it is gone from the current sketch.
      * D7 never changes -> an old build is still driving D7 as BRUSH_DIR, which
        it held HIGH forever. HIGH is retract, which is why the rod only ever
        went one way.
      * both pins correct, rod still dead -> the Uno has done its job. The fault
        is the driver, its motor supply, or the rod's own wiring.
      * link drops when the rod stops -> a microSD card is in the shield's slot.
        D4 is its chip select and STOP (D4 low) selects it. Pull the card.
    """
    print("ROD FREE TO MOVE, both end stops clear. Ctrl-C to abort.\n")
    print("  probe D7 (Dir) and D4 (Pwm) against GND:")
    print("    EXTEND   D7 = LOW    D4 = HIGH")
    print("    STOP     D7 = held   D4 = LOW    <- rod holds position")
    print("    RETRACT  D7 = HIGH   D4 = HIGH")
    print("  NOTE D7 LOW extends -- this channel is the opposite of the wheels.\n")

    link = MotorLink()
    link.start()
    time.sleep(1.0)
    try:
        # STOP between the throws, not just at the end: the interesting failure
        # is a rod that extends and will not retract, and running the two throws
        # back to back would hide it behind the reversal.
        for state in ("EXTEND", "STOP", "RETRACT", "STOP"):
            print(f"  {state:8} -> act={act_demand(state, None):+d}")
            deadline = time.monotonic() + hold_s
            while time.monotonic() < deadline:
                # Pushed continuously because the run loop ages this sample on
                # its own clock -- stop feeding it and the rod stops, which is
                # the same failsafe the panel relies on.
                link.set_actuator(state, None)
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\naborted")
    finally:
        link.stop()
        time.sleep(0.3)
    s = link.latest()
    print(f"\nsent={s['sent']} acked={s['acked']} loss={s['loss_pct']:.1f}%")
    if not s["acked"]:
        print("NOTHING ACKED - the rod was never the problem; fix the link first"
              " (python3 uno_motors.py --status)")
        return 1
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="joystick -> Uno motor link")
    ap.add_argument("--status", action="store_true", help="prove the Uno answers")
    ap.add_argument("--test", action="store_true", help="drive each direction")
    ap.add_argument("--act", action="store_true",
                    help="drive the linear actuator only, panel switch bypassed")
    ap.add_argument("--hold", type=float, default=None, metavar="SEC",
                    help="seconds per stage: default 5 for --test, 3 for --act"
                         " (long enough to get a meter on the pin)")
    ap.add_argument("--loop", action="store_true",
                    help="with --test, repeat the square until Ctrl-C")
    ap.add_argument("--gap", type=float, default=1.0, metavar="SEC",
                    help="stopped seconds between --test legs, default 1;"
                         " 0 runs them back to back")
    ap.add_argument("--deflect", type=float, default=0.6, metavar="FRAC",
                    help="stick deflection for --test, 0..1, default 0.6")
    ap.add_argument("--only", metavar="DIR",
                    help="with --test, drive ONE leg only: FORWARD, BACK,"
                         " LEFT or RIGHT")
    args = ap.parse_args()
    if args.act:
        sys.exit(_act_test(3.0 if args.hold is None else args.hold))
    if args.test:
        sys.exit(_test(hold_s=5.0 if args.hold is None else args.hold,
                       gap_s=args.gap, deflect=args.deflect,
                       loop=args.loop, only=args.only))
    sys.exit(_status())
