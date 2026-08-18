#!/usr/bin/env python3
"""
Operator controls: the 7 panel switches, the joystick and the potentiometer.

Same split as stream.py / link.py â€” this module owns the hardware and a thread,
main.py owns the single UI timer and pushes snapshots into inputs_panel.py.
Nothing here touches Qt, so it can be exercised headless:

    python3 inputs.py

WIRING FACTS THIS DEPENDS ON (all measured on the rig, 2026-08-13):

  * Every switch is ACTIVE-LOW â€” closing shorts the pin to GND, open floats.
    So the internal PULL-UP is mandatory and **closed reads 0**. Do not copy
    toggle_read.py, which assumes active-high with a pull-down and therefore
    reads the same value pressed and released.

  * SDA/SCL are DELIBERATELY CROSSED into the ADS1115 (SDA->SCL, SCL->SDA).
    The kernel i2c peripheral has fixed pin roles and cannot adapt, so
    /dev/i2c-1 always scans empty and smbus2 CANNOT work â€” which is why
    joystick_link.py has never been able to read this chip. Bit-banging is the
    only path, hence the imports from the parent directory below.

  * A0 = joystick X, A1 = joystick Y, A2 = potentiometer, A3 = NOT CONNECTED
    (A3 floats and drifts across most of the range; it is never sampled here).

TWO THREADS, NOT ONE, and the split is load-bearing rather than tidiness:

  * the SWITCH loop reads six GPIO pads (~70 us) at a real POLL_HZ
  * the ADC loop bit-bangs three ADS channels (~32 ms) at its own ADC_POLL_HZ

They shared a thread until 2026-08-17, which cost both correctness and feel. The
switch poll inherited the ADC's ~94 ms (measured), so it ran at ~10.6 Hz instead
of 20 -- and DEBOUNCE_SAMPLES is counted in SAMPLES, so a debounce tuned for
100 ms silently became ~190 ms, on the actuator among everything else. A
safety-relevant debounce clock must not be set by how slow an unrelated analog
bus is. Meanwhile the joystick reached the wheels a quarter-second behind the
operator's hand. See POLL_HZ and ADC_POLL_HZ for the numbers.

ANALOG READS ARE VALIDATED BEFORE THEY CAN DRIVE ANYTHING. A bit-banged transfer
can return a number that is entirely in range and still never happened, and here
that number becomes wheel demand. See ADC_AGREE_REJECT for the failure that was
observed on the wire and the two physical tests that catch it.

Requesting GPIO2/3 as GPIO lines re-muxes them away from the i2c controller, so
stop() hands them back with `pinctrl set N a0 pu`. Skipping that leaves the
hardware bus broken for anything that runs later.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)          # the bit-bang modules live in ~

# --- tuning ------------------------------------------------------------------
# Switch poll rate. This is now a REAL 20 Hz because the switches no longer wait
# on the ADC: reading six GPIO pads costs ~70 us, so the loop sleeps out the rest
# of its period instead of overrunning it.
#
# THE TWO POLLS USED TO SHARE ONE THREAD AND THAT WAS A BUG, not just a slowdown.
# _read_once() read the switches and then spent ~94 ms bit-banging three ADS
# channels (measured on the rig, see ADC_POLL_HZ), so the switch loop actually
# ran at ~10.6 Hz. DEBOUNCE_SAMPLES is counted in SAMPLES, so its meaning silently
# doubled from the 100 ms it was tuned for to ~190 ms, and every switch throw --
# including the actuator's -- carried that lag. A safety-relevant debounce clock
# must not be set by how slow an unrelated analog bus happens to be, so the ADC
# now runs on its own thread and this one keeps its own time.
POLL_HZ = float(os.environ.get("INPUTS_POLL_HZ", "20"))
ENABLED = os.environ.get("INPUTS_ENABLED", "1") == "1"

# Kill-switch for the ANALOG half only (joystick + pot), leaving the switches
# fully alive. Exists because the A1/A2 wiring is intermittent and its garbage
# occasionally survives every validation gate as phantom wheel demand - set
# INPUTS_ADC_ENABLED=0 (currently done in the Pi's .xinitrc) to take the whole
# analog path out of the loop while testing the rest of the rig. The axes and
# pot then read None everywhere, which mixes to a dead stop and a dark lamp,
# and the strip says why instead of showing a dead stick.
ADC_ENABLED = os.environ.get("INPUTS_ADC_ENABLED", "1") == "1"

# Ceiling on the ADC thread's rate. It is a CEILING, not a promise: a full
# 3-channel read is the slowest thing in this module and the thread simply runs
# flat out when it cannot keep up.
#
# 40 Hz is sized for the KERNEL bus (ads_i2c.py), where a 3-channel poll costs
# ~12 ms - mostly the chip's own conversion time - so a 25 ms period still
# leaves half the budget idle. Raised from 25 on 2026-08-17 because every
# sample-counted latency in this file (the median, the jump confirm, the pot's
# stability window) is priced in poll periods, and the reported pot lag was
# exactly those windows at 40 ms a tick. On the userspace-bit-bang FALLBACK
# this cap is unreachable (~32 ms/poll idle, far worse under load) and the
# thread just runs as fast as that path allows, same as before.
ADC_POLL_HZ = float(os.environ.get("INPUTS_ADC_POLL_HZ", "40"))

# The kernel's own bit-banged I2C bus, tried BEFORE the userspace bit-bang.
# /boot/firmware/config.txt maps it onto the crossed wiring (SDA=GPIO3,
# SCL=GPIO2) with dtoverlay=i2c-gpio - see ads_i2c.py for why this exists: the
# userspace path pays an ioctl per clock edge and collapsed to 5 Hz under the
# viewer's decode load, while the kernel path is one ioctl per whole transfer.
# Set INPUTS_I2C_DEV="" to force the old userspace path.
KERNEL_I2C_DEV = os.environ.get("INPUTS_I2C_DEV", "/dev/i2c-3")

# Seconds between attempts to bring the ADC back up. The bit-banged ADS bus is
# opened at start-up, but it was previously opened ONCE and never again: if it
# failed at that moment - or dropped out later - the joystick and the pot stayed
# dead for the whole life of the process and only a restart of main.py brought
# them back. The switches kept working the entire time, so the panel looked
# healthy apart from a "no ADC" label. Retrying costs one GPIO claim every few
# seconds and turns a permanent outage into a self-healing one.
ADC_RETRY_S = float(os.environ.get("INPUTS_ADC_RETRY_S", "5.0"))

# Consecutive polls where EVERY channel reads None before the bus is treated as
# wedged and torn down for the retry above to reopen. Individual channels return
# None on a single bad transfer, which is normal and must not trip this.
ADC_DEAD_READS = int(os.environ.get("INPUTS_ADC_DEAD_READS", "10"))

# --- joystick zeroing --------------------------------------------------------
# How the stick's rest point is established, and how much it is trusted.
#
# This is safety code. The centre used to be whatever the FIRST sample happened
# to be (`self._centre.setdefault(ch, raw)`), captured once and never revisited.
# On this rig the ADC is bit-banged over GPIO and its first read after coming up
# is exactly the one most likely to be garbage - and a wrong centre does not
# read as "no signal", it reads as a stick held hard over. The robot then drives
# itself at a steady demand with nobody touching anything. Observed on the wire
# 2026-08-17: `M 237 237 0 0 237` while the joystick sat at rest.
#
# So: take several samples and use their MEDIAN, and refuse a centre that is not
# somewhere a resting stick could actually be. A stick at rest sits at VDD/2, so
# a "centre" far from mid-scale is a bad read, an unplugged wiper or a stick
# being held - none of which may be adopted as zero. Until a centre passes, the
# axes report None, which main.py mixes to a dead stop.
CENTRE_SAMPLES = int(os.environ.get("INPUTS_CENTRE_SAMPLES", "9"))
CENTRE_TOLERANCE = float(os.environ.get("INPUTS_CENTRE_TOLERANCE", "0.25"))

# Fraction of full deflection ignored around centre, then rescaled so there is
# no step at the edge of the band. Even a good centre drifts by a few counts,
# and without this the wheels creep whenever the rig is powered up.
AXIS_DEADBAND = float(os.environ.get("INPUTS_AXIS_DEADBAND", "0.08"))

# Which way the stick's electrical travel maps onto the robot's motion. Flip
# these, not the mixer or the sketch, if a re-mount ever reverses an axis
# again - this is the one place that knows about stick orientation.
#
# History, because this has now flipped TWICE in one day (2026-08-18): both
# axes were inverted in the evening after the robot drove against the hand -
# then the ADC harness was rewired later that night and both axes came out
# reversed AGAIN, so the defaults went back to 0. The lesson: the stick's
# polarity is a property of the WIRING, so re-verify all four directions on
# the real robot after any analog rework, and expect to touch only these two
# values when it changes.
INVERT_X = os.environ.get("INPUTS_INVERT_X", "0") == "1"
INVERT_Y = os.environ.get("INPUTS_INVERT_Y", "0") == "1"

# Raw reads kept per channel for median smoothing.
#
# A single bad transfer on a bit-banged bus produces a wild count, and a wild
# count here is not cosmetic - on an axis it is wheel demand, on the pot it is
# lamp brightness. Observed at rest: the light flicking between 0 and 7 while
# nobody touched the knob. A median discards the outlier outright; an average
# would blend it in and still move the output. 3 samples at POLL_HZ is ~100ms of
# lag, which is imperceptible on a knob and well inside the stick's feel.
#
# Deliberately NOT a deadband on the pot - light_demand() explains why folding
# small values to zero would put a dead patch at the bottom of the knob's travel.
SMOOTH_SAMPLES = int(os.environ.get("INPUTS_SMOOTH_SAMPLES", "3"))

# --- rejecting reads that no operator produced -------------------------------
# THE WHEELS MUST ONLY TURN WHEN THE STICK IS ACTUALLY MOVED. A bit-banged read
# can hand back a number that is perfectly in range and still never happened, and
# on this rig that number becomes wheel demand.
#
# OBSERVED ON THE WIRE 2026-08-17, stick at rest, nobody at the panel:
#     M 237 237 0 0 237
# Both axes AND the pot all reporting ~93% of full scale at the same instant.
# That is raw 16383 on every channel -- 0x3FFF, fourteen bits set -- which is what
# a transfer returns when the mux never moved or SDA was never driven and the
# remaining bits floated high. `L=+237 R=+237` and `L=+255 R=+255` both appear in
# ~/motor_cam.log, with L and R IDENTICAL, i.e. a pure forward command with the
# steering axis exactly centred. A hand on an analog stick does not repeatedly
# produce the same handful of quantised levels.
#
# A RANGE CHECK CANNOT CATCH THIS and that is the whole difficulty: 16383 sits
# inside the stick's real 0..17600 travel, so nothing about the value alone is
# wrong. Two physical facts catch it instead, and both are free.
#
# 1. X, Y and the knob are three INDEPENDENT devices on three separate wipers.
#    Reading the same value to within this fraction of full scale means the bus
#    returned one register three times, not that three inputs coincided. The whole
#    poll is discarded, because if the mux never moved then no channel is
#    trustworthy - not just the ones that look extreme.
ADC_AGREE_REJECT = float(os.environ.get("INPUTS_ADC_AGREE_REJECT", "0.01"))

# 2. The stick is SPRING-CENTRED and a hand takes 150 ms+ to cross it, which is
#    several polls. A single sample that leaps more than this fraction of full
#    scale is a glitch until a second sample agrees with it. On a real push the
#    confirmation is already there in the next sample, so this costs one poll
#    (~30 ms) on a fast flick and nothing at all on a steady deflection -- and it
#    makes it arithmetically impossible for ONE bad transfer to reach the wheels.
#    Raised 0.35 -> 0.50 on 2026-08-18 night: during fast continuous shaking
#    every direction change re-tripped the 35% limit (a hard flick covers
#    ~5000-7000 counts per 35 Hz sample, right at the old threshold) and the
#    constant one-sample rejections compounded with the noise gate into "the
#    stick is stuck". At 50% only a genuinely impossible move trips it - a
#    single wild transfer like 0 -> 16383 still cannot pass - and a hand no
#    longer can.
ADC_JUMP_LIMIT = float(os.environ.get("INPUTS_ADC_JUMP_LIMIT", "0.50"))
ADC_JUMP_CONFIRM = int(os.environ.get("INPUTS_ADC_JUMP_CONFIRM", "2"))

# Outside the rail is not a reading at all. The ADS1115 is signed and a shifted or
# truncated transfer readily produces a large negative or over-scale count; a
# wiper on a 3.3 V rail cannot. Generous either side so a stick genuinely at an
# end stop is never mistaken for a fault.
ADC_RANGE_MARGIN = float(os.environ.get("INPUTS_ADC_RANGE_MARGIN", "0.06"))

# 3. A FLOATING INPUT NEVER STOPS CHANGING DIRECTION, and a hand cannot change
#    direction that fast. Observed 2026-08-18 with the chip answering but its
#    analog harness loose: every channel wandering the full scale at rest
#    (x 2752 -> 14192 -> 5984 within seconds), ~27% caught by the jump test and
#    the remainder walking through as wheel demand, L=+135 R=+201 with nobody
#    at the panel. The samples that get through drift SMOOTHLY, so no
#    per-sample test can catch them - but over any short window they reverse
#    direction over and over, which a spring-centred stick under a hand does
#    exactly once per push-and-return.
#
#    So: over the last ADC_NOISE_WINDOW samples, if the spread exceeds
#    ADC_NOISE_BAND of full scale AND the deltas flipped sign at least
#    ADC_NOISE_REVERSALS times, the channel is floating, not driven - blank it.
#    A resting stick fails the spread test (tens of counts). A sweep fails the
#    reversal test (deltas all one sign; small flips are LSB noise and not
#    counted). Costs nothing on a healthy channel, and turns a loose harness
#    into "no ADC" on the strip instead of a twitching robot.
#
#    REVERSALS retuned twice on 2026-08-18, both times against a real hand on
#    the real rig: 3 -> 4 after ~5 Hz up-down shaking blanked (2-3 flips per
#    window plus turnaround tremor over the 32-count floor), then 4 -> 5 with
#    the floor at 96 after fast four-direction stick work STILL stuck
#    (rej=noise reached 5340 in one session; at genuinely fast shakes the
#    turnaround legs are large, so the floor cannot separate them). At 5 of 7
#    deltas only near-every-sample jitter trips - which is exactly what a
#    float's random walk does and a hand at any speed measured so far does
#    not. This is the LAST notch: at 6 the gate cannot fire at all inside an
#    8-sample window with the floor applied, so if a hand ever hits 5, shrink
#    the demand another way (lower ADC_NOISE_BAND) rather than raising this.
ADC_NOISE_WINDOW = int(os.environ.get("INPUTS_ADC_NOISE_WINDOW", "8"))
ADC_NOISE_BAND = float(os.environ.get("INPUTS_ADC_NOISE_BAND", "0.04"))
ADC_NOISE_REVERSALS = int(os.environ.get("INPUTS_ADC_NOISE_REVERSALS", "5"))
ADC_NOISE_FLIP_MIN = int(os.environ.get("INPUTS_ADC_NOISE_FLIP_MIN", "96"))

# 4. A WIPER THAT LOSES CONTACT MID-TRAVEL READS EXACTLY 0x3FFF - all data
#    bits floating high - which is 16383, INSIDE the stick's real 0..17600
#    travel, ~93% deflection. Seen twice on this rig, and it is the mechanism
#    behind "left and right swap" (reported 2026-08-18): push LEFT, the
#    marginal wiper blips open, the channel reads 16383 = hard RIGHT, and the
#    robot turns against the hand. The value is STEADY while floating, so the
#    reversal gate cannot catch it, and it confirms itself past the jump test
#    in two samples.
#
#    Its fingerprint: a real wiper voltage jitters by LSBs and this rig's true
#    right endstop measures 17568, a clear 1185 counts above - so a read
#    within this band of exactly 16383 is rejected outright. Cost: a real
#    stick held PRECISELY at 93.1% travel (a +-0.3% sliver) reads as no
#    demand - a dead stop, never a wrong direction. A sweep passes through
#    the band in one sample, which smoothing already absorbs.
ADC_FLOAT_RAW = 0x3FFF
ADC_FLOAT_BAND = int(os.environ.get("INPUTS_ADC_FLOAT_BAND", "48"))

# --- the pot must EARN being believed ----------------------------------------
# The light follows this channel directly, so a wandering reading IS a blinking
# lamp - which is exactly what the rig does when the pot's wiper connection goes
# intermittent. Watched on 2026-08-17 with nobody at the panel: A2 read 0, then
# 1840, then 15136, then -544, while A0/A1 sat rock steady on the SAME bus in the
# SAME transfers - so it is the pot's own wiring, not the ADC, and jump/range
# rejection alone cannot catch the slow drifts in between the leaps.
#
# A real knob is either being held somewhere or resting somewhere; either way
# consecutive reads agree to within a few counts. A floating input never stops
# moving. So a new pot value is only ACCEPTED after POT_STABLE_SAMPLES
# consecutive good reads inside a POT_STABLE_BAND fraction of full scale.
#
# WHAT HAPPENS BETWEEN ACCEPTANCES IS A LATCH, NOT A BLANK. The pot is a
# SETTING, like a thermostat: the operator turns it, lets go, and expects the
# lamp to HOLD that brightness - including through every wander the flaky wiper
# produces afterwards. The first version of this gate published None whenever
# the window was unstable, which was safe but wrong twice over: the lamp fell
# dark the moment the operator released a knob whose wiper then floated
# (reported 2026-08-17: "when potentiometer is stopped it is not holding that
# value"), and the panel blinked between a percentage and a dash several times
# a second. Holding the last stable reading fixes both, and costs nothing in
# safety - this channel drives a LAMP, and the held value is always one the
# operator genuinely set.
#
# The lamp still STARTS dark: the latch is empty at power-up, so nothing is
# shown or lit until the knob produces its first stable reading. Cost on a
# healthy pot: ~4 polls (~160 ms) of lag after a turn settles, invisible on a
# lamp. And the latch deliberately survives an ADC reopen - a bus hiccup is not
# evidence the operator moved the knob.
# SNAP ON LARGE, CONFIRM ON SMALL (operator's algorithm, 2026-08-17 evening -
# the third iteration of this gate, each one trading a little safety margin for
# response after the lag was felt on the rig):
#
#   * a reading further than POT_SNAP_BAND from the latch is a DELIBERATE MOVE
#     and is adopted immediately - one poll, ~25 ms. Garbage cannot ride this
#     path because _validate() has already killed the wire's signature (leaps
#     over 35% of scale between adjacent samples need one confirming sample,
#     so a real hard spin costs 50 ms and a lone spike costs nothing at all).
#   * a reading INSIDE the band is either a gentle trim or wiper noise, and
#     only POT_STABLE_SAMPLES agreeing polls (~75 ms) move the latch - this is
#     what keeps the lamp from dithering while the knob rests.
#   * None, or an unstable window, holds the latch exactly as before.
#
# Numbers at the 40 Hz poll: any movement a hand can make lands in 25-50 ms,
# a trim in ~75 ms, and the hold-when-released behaviour is unchanged.
POT_STABLE_SAMPLES = int(os.environ.get("INPUTS_POT_STABLE_SAMPLES", "3"))
POT_STABLE_BAND = float(os.environ.get("INPUTS_POT_STABLE_BAND", "0.06"))
POT_SNAP_BAND = float(os.environ.get("INPUTS_POT_SNAP_BAND", "0.06"))

# THE SUPPLY-DROPOUT HOLD (2026-08-18 evening). The pot's 3.3V/ground feed is
# intermittent: the reading alternates between the knob's true value and a
# clean 0 as the supply makes and breaks, dwelling up to a second at each -
# which the snap path faithfully turned into a lamp strobing at 1 Hz while
# the operator held the knob still (log: light=255 / light=0 / light=255 on
# consecutive seconds, p17600 / p0 / p17648 behind it).
#
# The discriminator: a HAND turning the knob to zero passes through the
# middle of the travel on the way down, so the latch has already followed it
# below POT_ZERO_CLIFF by the time a genuine 0 arrives - whereas a supply
# drop is a cliff, full straight to 0 in one poll with no intermediates. So a
# near-zero read arriving while the latch is still high is treated as a
# dropout and held through, and only a zero SUSTAINED for POT_ZERO_HOLD_S
# adopts - which keeps "knob at minimum = lamp off" working even on the
# broken supply, just 2 s late. A real spin-to-off stays instant: the latch
# tracks the sweep down and is below the cliff before zero lands.
POT_ZERO_RAW = float(os.environ.get("INPUTS_POT_ZERO_RAW", "0.03"))
POT_ZERO_CLIFF = float(os.environ.get("INPUTS_POT_ZERO_CLIFF", "0.20"))
POT_ZERO_HOLD_S = float(os.environ.get("INPUTS_POT_ZERO_HOLD_S", "2.0"))

# How the two session switches behave, because the panel hardware decides this
# and not us:
#   1 (default) LATCHING levers - the switch position IS the state. Thrown ==
#               recording; centre/off == stopped. Fail-safe: knock the panel and
#               the recording stops rather than silently keeping the old state.
#   0           MOMENTARY buttons - each press flips the state and it stays
#               flipped after the button springs back.
# Flip this to 0 if the pills blink for a fifth of a second per press instead of
# staying lit while you hold them (that is what GPIO25 measured doing).
SESSION_LATCHING = os.environ.get("SESSION_LATCHING", "1") == "1"

# Consecutive agreeing samples before a switch change is believed.
#
# NOT optional on this rig. Logged on 2026-08-15 with pinctrl at ~23 Hz, over a
# 6.5 minute window in which nobody touched the panel: GPIO23 blipped low 42
# times, every blip exactly ONE sample wide (~43 ms), while 24 and 25 never
# moved at all. That is noise on the 23 line, not an operator - a real throw is
# hundreds of ms and shows a spread of durations.
#
# Undebounced, each of those blips is a PAUSE and an immediate RESUME in the
# middle of a recording. 2 samples at the 20 Hz poll = 100 ms of agreement,
# which no 43 ms glitch can survive, and which still catches the save button's
# measured ~180 ms press with 2 samples to spare. Raise it only if the glitches
# ever come in pairs; every extra sample is 50 ms of lag on a real throw.
DEBOUNCE_SAMPLES = int(os.environ.get("INPUTS_DEBOUNCE", "2"))

# Â±6.144 V PGA over a 16-bit signed range. 3.3 V rail => ~17600 counts full scale.
COUNTS_PER_VOLT = 32768 / 6.144
FULL_SCALE = 3.3 * COUNTS_PER_VOLT

# label -> BCM pin. Names come from Harshil; 16/19 are the actuator pair and are
# decoded together rather than shown as two anonymous pins.
# Roles confirmed with Harshil 2026-08-14. GPIO12 is gone on purpose: it was
# the other candidate for the "updated toggle" and measured completely idle
# (never moved) across a 30 s window in which GPIO25 moved 56 times, so the
# updated toggle is on 25 and nothing is wired to 12.
#
# The recording controls are two INDEPENDENT switches, not an interlocked
# pair - measured both reading `hi` (open) at rest on 2026-08-15 (then on
# GPIO24/23), whereas an interlocked ON-OFF-ON pair always has one leg down.
# They are decoded together into a session state but each keeps its own
# meaning. Rewired to 22/11 on 2026-08-18 - see REC_PIN below:
#   switch 1, RED   leg, GPIO22 ... START / STOP
#   switch 2, GREEN leg, GPIO11 ... PAUSE / RESUME
SWITCHES = [
    ("BRUSH", 27),
    ("SAVE", 9),
    ("START / STOP", 22),
    ("PAUSE / RESUME", 11),
]
ACT_EXTEND_PIN = 16
ACT_RETRACT_PIN = 19

# Named separately from SWITCHES so the panel and the recorder can reference the
# roles without matching on a display string.
# Moved 13 -> 27 on the operator's order 2026-08-18; nothing is on 13 now.
BRUSH_PIN = 27          # brush motor on / off

# Switches whose OPEN state means ON - the reverse of every other switch here.
#
# THE BRUSH TOGGLE IS INVERTED ON THE OPERATOR'S EXPLICIT ORDER (2026-08-17,
# "high -> brush on, low -> brush off"): the lever is mounted so that the throw
# Harshil uses as ON leaves the pin open, and the throw used as OFF closes it to
# ground. Decoding it active-low therefore ran the brush in the OFF position -
# diagnosed live, with the pad register and the wire log side by side. That was
# measured on the OLD pin (GPIO13); the orientation carried over to GPIO27
# unverified, so if the brush now runs with the lever OFF, empty this set.
#
# KNOW WHAT THIS COSTS. With the pull-up, an UNPLUGGED OR BROKEN WIRE floats
# HIGH, and high now reads ON: a snapped toggle wire is a running brush that
# the panel cannot stop (the joystick, actuator and light keep their fail-dead
# active-low sense; only this switch trades that away). If the brush ever runs
# with the toggle unplugged, this constant is why - and moving the wire to the
# switch's other outer terminal, then emptying this set, restores the fail-safe
# orientation without any code change.
OPEN_IS_ON = {BRUSH_PIN}
# Rewired 2026-08-18 on the operator's order: the pair moved 24/23 -> 22/11
# (red kept START/STOP, green kept PAUSE/RESUME - if the strip shows the two
# levers swapped, exchange these two numbers, nothing else references them).
# GPIO11 is SPI SCLK by default; SPI is unused on this rig, plain input here.
REC_PIN = 22            # switch 1, RED   leg - START / STOP
PAUSE_PIN = 11          # switch 2, GREEN leg - PAUSE / RESUME
SESSION_PINS = (REC_PIN, PAUSE_PIN)

# The three session states, in the order the operator moves through them.
RECORDING, PAUSED, STOPPED = "RECORDING", "PAUSED", "STOPPED"

ALL_PINS = [p for _, p in SWITCHES] + [ACT_EXTEND_PIN, ACT_RETRACT_PIN]


# The save button. Measured (on its old pin) pulsing low for ~0.18 s per press,
# i.e. it is momentary, not latching -- so its LEVEL is useless to a 30 fps UI
# that can sample between two frames of the pulse. The reader counts edges
# instead and publishes a running total; a consumer remembers the last count it
# acted on and fires once per increment. That makes a missed press impossible as
# long as the 20 Hz poll catches the pulse, which it does with ~3 samples to
# spare. Moved 25 -> 9 on the operator's order 2026-08-18 (GPIO9 is SPI MISO by
# default, but nothing on this rig uses SPI, so it is a plain input here).
SAVE_PIN = 9


def _blank():
    """Snapshot shape, used before the first read and whenever hardware is absent."""
    return {
        "ok": False,
        # TWO error slots because there are now two threads and either can fail on
        # its own: the switches keep working with a dead ADC, and the ADC keeps
        # working while a switch pin is busy. latest() joins them into the single
        # "error" the strip renders, so no caller has to know that.
        "error": "startingâ€¦",
        "switch_error": "startingâ€¦",
        "adc_error": None,
        # How fast the analog path is really running, and which validation test is
        # rejecting reads. Both published so main.py can log them - a collapsed
        # poll rate and a phantom stick demand are the two failures here, and
        # neither is visible from the outside without these.
        "adc_hz": 0.0,
        "adc_rejects": {"range": 0, "agree": 0, "jump": 0, "read": 0,
                        "noise": 0, "float": 0},
        "switches": {name: None for name, _ in SWITCHES},
        "actuator": None,          # "EXTEND" / "STOP" / "RETRACT" / "FAULT"
        # Raw pin levels behind that decode, so the panel can show the operator
        # WHY it is calling a stage rather than just asserting it.
        "act_pins": {ACT_EXTEND_PIN: None, ACT_RETRACT_PIN: None},
        # Recording session decoded from the two panel switches, plus the raw
        # levels behind it for the same reason act_pins exists.
        "session": None,           # "RECORDING" / "PAUSED" / "STOPPED"
        "session_pins": {REC_PIN: None, PAUSE_PIN: None},
        # Total open->closed edges seen on the save button since start-up.
        "save_presses": 0,
        # Seconds the save button has been continuously held closed, 0.0 while
        # open. The recorder's post-stop claim is a press-AND-HOLD, so the
        # consumer needs the level's duration as well as the edge count above.
        "save_held_s": 0.0,
        "joy": {"x": None, "y": None, "x_raw": None, "y_raw": None},
        "pot": {"pct": None, "raw": None, "volts": None},
        "updated": 0.0,
        # The ANALOG half's own clock, separate from "updated" above.
        #
        # main.py pushes the stick into MotorLink on every UI frame, and that push
        # is what resets MotorLink's staleness timer -- so if this thread stopped
        # producing samples while the switch thread kept running, the last stick
        # position would be re-asserted as "fresh" 30 times a second and the wheels
        # would keep driving on it. A reader that has died must stop the robot, so
        # the age of the ANALOG sample has to be checkable on its own.
        "adc_updated": 0.0,
    }


class SessionDecode:
    """The two panel switches -> STOPPED / RECORDING / PAUSED.

    Kept as a class because the momentary mode has to remember what it was told
    last; the latching mode is stateless and ignores all of it. Both paths run
    off the same `closed` booleans the rest of inputs.py deals in, so neither has
    to know that closed means the pin reads 0.
    """

    def __init__(self, latching=None):
        self.latching = SESSION_LATCHING if latching is None else latching
        self._running = False
        self._paused = False
        self._was_rec = None        # None until the first sample, so the very
        self._was_pause = None      # first reading cannot score a phantom press

    def update(self, rec_closed, pause_closed):
        if rec_closed is None or pause_closed is None:
            return None

        if self.latching:
            self._running, self._paused = rec_closed, pause_closed
        else:
            if rec_closed and self._was_rec is False:
                self._running = not self._running
                # Starting a run always starts it rolling. Inheriting a stale
                # pause from the previous run would mean pressing START and
                # recording nothing, with the UI insisting it was paused.
                self._paused = False
            if pause_closed and self._was_pause is False and self._running:
                self._paused = not self._paused

        self._was_rec, self._was_pause = rec_closed, pause_closed

        if not self._running:
            # PAUSED only exists inside a run. Holding the pause lever with the
            # recording lever off is STOPPED, not some fourth state.
            self._paused = False
            return STOPPED
        return PAUSED if self._paused else RECORDING


class InputReader(threading.Thread):
    """Polls switches + ADS channels in the background. Never raises at callers."""

    def __init__(self):
        super().__init__(daemon=True)
        self._lock = threading.Lock()
        self._state = _blank()
        self._stop = threading.Event()
        self._req = None            # switch lines
        self._ads_req = None        # GPIO2/3 for the bit-bang
        self._bus = None
        # Auto-centred at startup, exactly like joystick_link.py: the stick's
        # rest point is VDD/2 on THIS rail, and joystick_link.py's hardcoded
        # CENTRE_TYPICAL=12100 is a stale 5 V constant that does not apply.
        self._centre = {}
        self._centre_samples = {}   # ch -> readings being median-filtered
        self._smooth_buf = {}       # ch -> recent raw reads, median-filtered
        # Session + save-button state live on the reader, not in the snapshot:
        # _read_once() rebuilds the snapshot from _blank() every pass, so
        # anything that has to survive between reads has to be held here.
        self._session = SessionDecode()
        self._save_presses = 0
        self._save_was = None
        self._save_down_since = None   # monotonic when the hold began
        self._stable = {}           # pin -> last believed `closed`
        self._candidate = {}        # pin -> (value, consecutive samples seen)
        self._dead_reads = 0        # consecutive all-None ADC polls
        # Validation state for the analog path - see ADC_AGREE_REJECT.
        self._last_good = {}        # ch -> last raw read that passed every test
        self._jump_runs = {}        # ch -> (pending raw, consecutive agreements)
        self._pot_window = []       # recent good pot reads - see POT_STABLE_BAND
        self._pot_latched = None    # last ACCEPTED pot reading - the setting
        self._pot_zero_since = None # when the current run of cliff-zeros began
        self._kadc = None           # kernel-bus ADS1115, preferred over _bus
        self._adc_thread = None
        # Published so main.py can log it: a poll rate that has collapsed is the
        # difference between "the stick feels laggy" and "the stick is laggy", and
        # the reject counters name which test is firing when a phantom shows up.
        self._adc_hz = 0.0
        self._rejects = {"range": 0, "agree": 0, "jump": 0, "read": 0,
                         "noise": 0, "float": 0}
        self._noise_buf = {}        # ch -> recent accepted raws, for the float test

    # -- public ---------------------------------------------------------------

    def latest(self):
        with self._lock:
            snap = dict(self._state)
        # The switches are the operator's controls; a switch fault is the more
        # urgent of the two, so it wins the single line the strip has room for.
        snap["error"] = snap.get("switch_error") or snap.get("adc_error")
        return snap

    def stop(self):
        self._stop.set()

    # -- internals ------------------------------------------------------------

    def _fail(self, msg):
        with self._lock:
            self._state = _blank()
            self._state["switch_error"] = msg
        return False

    def _open(self):
        if not ENABLED:
            return self._fail("disabled (INPUTS_ENABLED=0)")
        if _PARENT not in sys.path:
            sys.path.insert(0, _PARENT)
        try:
            global gpiod, Bias, Direction, Value, IN, Bus, SDA, SCL, sample
            import gpiod
            from gpiod.line import Bias, Direction, Value
            from i2c_bitbang_probe import IN, Bus
            from i2c_bitbang_read import SDA, SCL, sample
        except Exception as exc:            # missing lib / missing helper module
            return self._fail(f"import failed: {exc}")

        try:
            pu = gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
            self._req = gpiod.request_lines(
                "/dev/gpiochip0", consumer="ground-station-switches",
                config={p: pu for p in ALL_PINS})
        except Exception as exc:            # EBUSY if a probe script is running
            return self._fail(f"switch pins busy: {exc}")

        # Switches still work without the ADC, so a failure here is not fatal -
        # the panel shows analog as unavailable and the pills keep updating. It
        # is also not permanent any more: run() retries this every ADC_RETRY_S.
        if ADC_ENABLED:
            self._open_adc()
        return True

    def _open_adc(self):
        """Bring up the ADS1115, kernel bus first. True on success.

        Split out of _open() so run() can call it again later. Re-openable at
        runtime is the point: whatever knocks the ADC out (it lost its claim, the
        rail dipped, the bus wedged mid-transfer) used to cost the joystick and
        the pot until someone restarted the viewer.

        TWO PATHS, tried in order. The kernel i2c-gpio bus (/dev/i2c-3, mapped
        onto the crossed wiring in config.txt) does the bit-banging in kernel
        space and survives the viewer's decode load; the userspace gpiod
        bit-bang is the fallback for an SD card whose config.txt predates the
        overlay. Same chip, same config word, same counts either way.
        """
        if KERNEL_I2C_DEV and os.path.exists(KERNEL_I2C_DEV):
            try:
                import ads_i2c
                adc = ads_i2c.ADS1115(KERNEL_I2C_DEV)
                if adc.probe():
                    self._kadc = adc
                    self._dead_reads = 0
                    self._centre.clear()
                    self._centre_samples.clear()
                    self._smooth_buf.clear()
                    self._last_good.clear()
                    self._jump_runs.clear()
                    self._noise_buf.clear()
                    # The WINDOW resets; _pot_latched does NOT. A reopened bus
                    # invalidates in-flight evidence, but the last accepted knob
                    # setting is a fact about the operator, not the bus - going
                    # dark on a reconnect would be the "not holding that value"
                    # bug in one more costume.
                    self._pot_window.clear()
                    return True
                # Node exists but the chip does not answer - a wiring fault the
                # userspace path cannot fix either, but fall through and let it
                # try: it claims the same pins the overlay owns and will report
                # a clean EBUSY into adc_error rather than sitting silent.
                adc.close()
            except Exception:
                pass
        try:
            self._ads_req = gpiod.request_lines(
                "/dev/gpiochip0", consumer="ground-station-ads",
                config={SDA: IN, SCL: IN})
            self._bus = Bus(self._ads_req, SDA, SCL)
            self._bus.reset_state()
            self._dead_reads = 0
            # Re-zero on every reopen. Carrying a centre across a bus that just
            # failed is how a garbage calibration outlives the fault that caused
            # it - and a stale centre drives the wheels, it does not just look
            # wrong. Cheap to retake: CENTRE_SAMPLES polls is well under a second.
            self._centre.clear()
            self._centre_samples.clear()
            self._smooth_buf.clear()
            # Same reasoning as the centre: _validate compares each read against
            # the last one it trusted, and a value from before the bus failed is
            # not a baseline any more. Keeping it would have the jump test measure
            # the first good read against a stale reference and reject it.
            self._last_good.clear()
            self._jump_runs.clear()
            self._noise_buf.clear()
            # Window only - the latch survives, same as the kernel branch above.
            self._pot_window.clear()
            return True
        except Exception as exc:
            self._release_adc()
            with self._lock:
                self._state["adc_error"] = f"ADC unavailable: {exc}"
            return False

    def _release_adc(self):
        """Drop the ADC claim so _open_adc() can take it cleanly next time."""
        try:
            if self._kadc is not None:
                self._kadc.close()
        except Exception:
            pass
        self._kadc = None
        try:
            if self._ads_req is not None:
                self._ads_req.release()
        except Exception:
            pass
        self._ads_req = self._bus = None

    def _sample(self, ch):
        """One channel via whichever path is up. Counts, or None."""
        if self._kadc is not None:
            return self._kadc.sample(ch)
        return sample(self._bus, ch)

    def _adc_present(self):
        return self._kadc is not None or self._bus is not None

    def _close(self):
        # Remember which path was live BEFORE dropping it: the pinctrl restore
        # below must only run for the userspace path.
        used_bitbang = self._ads_req is not None
        self._release_adc()
        try:
            if self._req is not None:
                self._req.release()
        except Exception:
            pass
        self._req = None
        # Hand GPIO2/3 back to the i2c controller - ONLY if the userspace
        # bit-bang re-muxed them in the first place. On the kernel path the
        # i2c-gpio driver owns those pins for the life of the boot, and forcing
        # them to ALT0 underneath it would wedge the bus for every later run.
        if used_bitbang:
            for pin in (2, 3):
                try:
                    subprocess.run(
                        ["/usr/bin/pinctrl", "set", str(pin), "a0", "pu"],
                        check=False, capture_output=True, timeout=2)
                except Exception:
                    pass

    def _norm_axis(self, ch, raw):
        """Raw counts -> -1.0..+1.0 about the centre, or None while unzeroed.

        None rather than 0.0 while calibrating on purpose: None is "no reading",
        which ages out through SAMPLE_STALE_S and mixes to a dead stop, whereas
        0.0 would assert that the stick is definitely centred - a claim this has
        not earned yet. See CENTRE_SAMPLES.
        """
        if raw is None:
            return None
        centre = self._centre.get(ch)
        if centre is None:
            centre = self._learn_centre(ch, raw)
            if centre is None:
                return None                     # still zeroing - no demand
        value = max(-1.0, min(1.0, (raw - centre) / (FULL_SCALE / 2.0)))
        if abs(value) < AXIS_DEADBAND:
            return 0.0
        # Rescale past the band so there is no jump from 0 to AXIS_DEADBAND the
        # instant the stick leaves centre.
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - AXIS_DEADBAND) / (1.0 - AXIS_DEADBAND)

    def _smooth(self, ch, raw):
        """Median of the last SMOOTH_SAMPLES good reads on `ch`.

        A failed read returns None straight through rather than the previous
        median: masking it with a stale value would hide a dead bus from the
        _dead_reads detector that reopens it.
        """
        if raw is None:
            return None
        buf = self._smooth_buf.setdefault(ch, [])
        buf.append(raw)
        del buf[:-SMOOTH_SAMPLES]
        return sorted(buf)[len(buf) // 2]

    def _pot_gate(self, raw):
        """The pot as a SETTING: snap on large moves, confirm small ones.

        Returns the latched value on every call - None only before the first
        reading is ever accepted (lamp starts dark). See POT_SNAP_BAND for the
        algorithm and its latency numbers; the split in one line: a big change
        is a hand and is believed at once, a small change might be the wiper
        and must repeat itself first, and silence or churn holds the latch so
        a released knob keeps its setting.

        A None read clears the window rather than being skipped over: "two
        stable reads, a failure, then a third" is not three consecutive stable
        reads, and on this rig a failed transfer and a flapping wiper travel
        together.
        """
        if raw is None:
            self._pot_window.clear()
            return self._pot_latched
        # A cliff to zero while the latch is high is the supply dropping, not
        # the hand - hold the setting unless the zero persists. See POT_ZERO_*.
        if (raw <= POT_ZERO_RAW * FULL_SCALE
                and self._pot_latched is not None
                and self._pot_latched > POT_ZERO_CLIFF * FULL_SCALE):
            if self._pot_zero_since is None:
                self._pot_zero_since = time.monotonic()
            if time.monotonic() - self._pot_zero_since < POT_ZERO_HOLD_S:
                return self._pot_latched
            # Sustained: the knob really is at the bottom (or the supply is
            # dead for good, in which case dark is the honest reading).
        else:
            self._pot_zero_since = None
        if (self._pot_latched is not None
                and abs(raw - self._pot_latched) > POT_SNAP_BAND * FULL_SCALE):
            # A deliberate move - adopt it NOW. _validate() upstream has
            # already made a lone wire spike impossible on this path, and the
            # window is reseeded so a follow-up trim measures against the new
            # position rather than a stale run of pre-move samples.
            self._pot_latched = raw
            self._pot_window.clear()
            self._pot_window.append(raw)
            return self._pot_latched
        # Small change (or the first-ever reading, which earns trust the slow
        # way - that is what keeps the lamp starting dark): require agreement.
        self._pot_window.append(raw)
        del self._pot_window[:-POT_STABLE_SAMPLES]
        if (len(self._pot_window) >= POT_STABLE_SAMPLES
                and (max(self._pot_window) - min(self._pot_window)
                     <= POT_STABLE_BAND * FULL_SCALE)):
            self._pot_latched = raw         # a genuine setting - adopt it
        return self._pot_latched

    def _agreement_would_move(self, reads):
        """Does an all-channels-agree reading actually command motion?

        THE REST STATE LEGITIMATELY LOOKS LIKE THE FAULT, which is why the
        agreement test is not allowed to fire on its own. A stick at rest sits at
        mid-scale on BOTH axes, so if the knob happens to be near half travel then
        all three channels really do read the same value -- an entirely normal
        panel, and rejecting it would blink the light off and report a dead stick
        several times a second for as long as the operator left the knob there.

        The phantom is separable because it agreed at 93% of full scale: an
        agreement that puts an axis well off its centre would DRIVE THE WHEELS, and
        that is the only case worth refusing. An agreement at the centre commands
        nothing, so letting it through costs at most a wrong lamp level for one
        poll and never a wrong movement.

        Unlearned centre counts as "would not move": the axes report None until a
        centre is accepted, so there is no demand to protect against yet.
        """
        for ch in (0, 1):                       # the two axes; the pot cannot move
            centre = self._centre.get(ch)
            raw = reads.get(ch)
            if centre is None or raw is None:
                continue
            if abs(raw - centre) > AXIS_DEADBAND * (FULL_SCALE / 2.0):
                return True
        return False

    def _validate(self, reads):
        """Drop readings that no physical input could have produced.

        `reads` is {ch: raw or None} straight off the bus for ONE poll; the return
        is the same shape with anything untrustworthy replaced by None. None then
        flows on as "no reading", which _norm_axis turns into a dead stop and
        light_demand turns into darkness -- never into a stale value that keeps
        driving. See ADC_AGREE_REJECT for what this is defending against and why a
        range check on its own cannot do it.

        Tests run whole-poll first, then per channel, because they answer
        different questions: agreement condemns the BUS (so every channel goes,
        including ones that look reasonable), while range and jump condemn one
        reading.
        """
        good = dict(reads)

        # -- whole poll: did the mux actually move? ---------------------------
        live = [v for v in good.values() if v is not None]
        if (len(live) == len(good) and len(live) > 1
                and max(live) - min(live) <= ADC_AGREE_REJECT * FULL_SCALE
                and self._agreement_would_move(good)):
            # Three independent wipers cannot agree this closely by accident.
            self._rejects["agree"] += 1
            return {ch: None for ch in good}

        for ch, raw in good.items():
            if raw is None:
                self._rejects["read"] += 1
                continue

            # -- per channel: is it even on the rail? -------------------------
            if not (-ADC_RANGE_MARGIN * FULL_SCALE
                    <= raw <= (1.0 + ADC_RANGE_MARGIN) * FULL_SCALE):
                self._rejects["range"] += 1
                good[ch] = None
                continue

            # -- per channel: the floating-open signature - see ADC_FLOAT_RAW.
            if abs(raw - ADC_FLOAT_RAW) <= ADC_FLOAT_BAND:
                self._rejects["float"] += 1
                good[ch] = None
                continue

            # -- per channel: could a hand have moved it that far? ------------
            prev = self._last_good.get(ch)
            if prev is not None and abs(raw - prev) > ADC_JUMP_LIMIT * FULL_SCALE:
                pending, runs = self._jump_runs.get(ch, (raw, 0))
                # "Agrees" means the confirming sample landed near the first one,
                # not that it repeated it exactly - the stick is still moving
                # while it is being confirmed.
                near = abs(raw - pending) <= ADC_JUMP_LIMIT * FULL_SCALE / 2.0
                runs = runs + 1 if near else 1
                if runs < ADC_JUMP_CONFIRM:
                    self._jump_runs[ch] = (raw, runs)
                    self._rejects["jump"] += 1
                    good[ch] = None
                    continue
                # Confirmed: a real, fast deflection. Fall through and accept it.
                self._jump_runs.pop(ch, None)
            else:
                self._jump_runs.pop(ch, None)

            self._last_good[ch] = raw

        return good

    def _noise_gate(self, ch, raw):
        """Blank a channel that is floating rather than driven. See ADC_NOISE_*.

        Runs AFTER _validate() (so the window holds only reads that passed the
        range/agree/jump tests) and BEFORE smoothing, the centre learner and
        the pot latch - a floating channel must not be allowed to teach the
        stick a garbage zero or snap the lamp around, it must read as absent.

        A None read leaves the window alone rather than clearing it: on a
        loose harness Nones and wander arrive interleaved, and forgetting the
        history on each None would let the channel sneak through on a short,
        briefly-coherent window.
        """
        if raw is None:
            return None
        buf = self._noise_buf.setdefault(ch, [])
        buf.append(raw)
        del buf[:-ADC_NOISE_WINDOW]
        if len(buf) >= 3:
            spread = max(buf) - min(buf)
            if spread > ADC_NOISE_BAND * FULL_SCALE:
                deltas = [b - a for a, b in zip(buf, buf[1:])]
                # A flip counts only when BOTH legs are substantial: a hand
                # reversing direction dwells at the turnaround (tiny deltas),
                # a float's random walk reverses at full stride.
                flips = sum(1 for a, b in zip(deltas, deltas[1:])
                            if a * b < 0 and abs(a) > ADC_NOISE_FLIP_MIN
                            and abs(b) > ADC_NOISE_FLIP_MIN)
                if flips >= ADC_NOISE_REVERSALS:
                    self._rejects["noise"] += 1
                    return None
        return raw

    def _learn_centre(self, ch, raw):
        """Median of CENTRE_SAMPLES readings, or None if it is not believable.

        Returns the accepted centre, or None while still collecting / after a
        rejected batch. Rejecting starts a fresh batch rather than giving up, so
        a stick released after start-up zeroes itself on the next pass.
        """
        buf = self._centre_samples.setdefault(ch, [])
        buf.append(raw)
        if len(buf) < CENTRE_SAMPLES:
            return None
        median = sorted(buf)[len(buf) // 2]
        buf.clear()
        if abs(median - FULL_SCALE / 2.0) > CENTRE_TOLERANCE * FULL_SCALE:
            return None                         # not a resting stick - refuse it
        self._centre[ch] = median
        return median

    def _debounce(self, raw):
        """Accept a change only after DEBOUNCE_SAMPLES agreeing reads.

        The first sample seeds every pin, so start-up is immediate rather than
        showing unknown switches for the first 100 ms.
        """
        if not self._stable:
            self._stable = dict(raw)
            return dict(raw)
        for pin, value in raw.items():
            if value == self._stable[pin]:
                # Back to the believed level before it was ever accepted - the
                # blip is over, so forget it rather than counting it toward the
                # next change in the same direction.
                self._candidate.pop(pin, None)
                continue
            prev, runs = self._candidate.get(pin, (value, 0))
            runs = runs + 1 if prev == value else 1
            if runs >= DEBOUNCE_SAMPLES:
                self._stable[pin] = value
                self._candidate.pop(pin, None)
            else:
                self._candidate[pin] = (value, runs)
        return dict(self._stable)

    def _read_switches(self):
        """Poll the switch pads and decode them. Touches no analog state.

        Fast and constant-time - six GPIO pad reads, ~70 us - which is the point
        of it being on its own thread: DEBOUNCE_SAMPLES and the save button's edge
        count are both measured in POLLS, so they only mean what they were tuned to
        mean while this loop keeps a steady period.
        """
        # Only the keys this thread owns. _read_adc() owns joy/pot and the two
        # are merged into the published snapshot by update(), so neither thread can
        # blank the other's half by rebuilding the whole dict from _blank().
        state = {}

        raw = {}
        for pin in ALL_PINS:
            # PULL_UP means INACTIVE (0) == shorted to GND == closed. This
            # inversion is the single easiest thing to get backwards here.
            raw[pin] = self._req.get_value(pin) == Value.INACTIVE
        closed = self._debounce(raw)
        # Published as "is this switch ON", not "is this pin closed" - the two
        # differ exactly for OPEN_IS_ON, where the operator's ON throw leaves
        # the pin open. Inverted here, after the debounce, so the debouncer
        # keeps reasoning about raw electrical levels.
        state["switches"] = {
            name: (not closed[pin]) if pin in OPEN_IS_ON else closed[pin]
            for name, pin in SWITCHES}

        ext, ret = closed[ACT_EXTEND_PIN], closed[ACT_RETRACT_PIN]
        # The actuator switch is mechanically interlocked (ON-OFF-ON), so both
        # closed should be impossible â€” surfaced as FAULT rather than assumed
        # away, since a broken interlock is exactly what you want to see.
        state["actuator"] = ("FAULT" if ext and ret else
                             "EXTEND" if ext else
                             "RETRACT" if ret else "STOP")
        # Back to raw levels for display: closed == shorted to GND == reads 0.
        # Measured on the rig 2026-08-14, the three stages are
        # 16=0/19=1 EXTEND, 16=1/19=1 STOP, 16=1/19=0 RETRACT.
        state["act_pins"] = {ACT_EXTEND_PIN: 0 if ext else 1,
                             ACT_RETRACT_PIN: 0 if ret else 1}

        state["session"] = self._session.update(closed[REC_PIN], closed[PAUSE_PIN])
        state["session_pins"] = {REC_PIN: 0 if closed[REC_PIN] else 1,
                                 PAUSE_PIN: 0 if closed[PAUSE_PIN] else 1}

        # Edge count, not level - see SAVE_PIN. `is False` rather than `not`, so
        # the first sample after start-up cannot be mistaken for a release.
        if closed[SAVE_PIN] and self._save_was is False:
            self._save_presses += 1
        self._save_was = closed[SAVE_PIN]
        state["save_presses"] = self._save_presses

        # Hold duration, measured off the DEBOUNCED level so the 43ms glitches
        # that forced DEBOUNCE_SAMPLES cannot restart the clock mid-hold.
        if closed[SAVE_PIN]:
            if self._save_down_since is None:
                self._save_down_since = time.monotonic()
            state["save_held_s"] = time.monotonic() - self._save_down_since
        else:
            self._save_down_since = None
            state["save_held_s"] = 0.0

        state["ok"] = True
        state["switch_error"] = None
        state["updated"] = time.time()
        # Replaced wholesale, never mutated in place: latest() hands out a SHALLOW
        # copy, so a caller iterating state["switches"] while this thread edited it
        # would see a half-updated dict.
        with self._lock:
            self._state.update(state)

    def _read_adc(self):
        """One 3-channel ADS poll, validated. Owns joy/pot and nothing else.

        THE SLOW HALF, and the reason it is not on the switch thread: ~32 ms of
        bit-banged I2C per pass even after the busy-wait fix, against the ~70 us
        the switches cost.
        """
        analog = {}
        if not self._adc_present():
            analog["adc_error"] = "ADC unavailable"
            # Axes and pot back to None rather than left at their last values: an
            # absent bus is not evidence the stick is where it was.
            analog["joy"] = {"x": None, "y": None, "x_raw": None, "y_raw": None}
            analog["pot"] = {"pct": None, "raw": None, "volts": None}
            analog["adc_updated"] = time.time()
            with self._lock:
                self._state.update(analog)
            return

        # Validated BEFORE smoothing, so a rejected read never enters the median
        # buffer. Letting one in would let it go on influencing the output for
        # SMOOTH_SAMPLES more polls after it had already been identified as junk.
        fresh = self._validate({ch: self._sample(ch) for ch in (0, 1, 2)})
        # Then the float test, on what survived - see _noise_gate().
        gated = {ch: self._noise_gate(ch, fresh[ch]) for ch in (0, 1, 2)}
        x_raw = self._smooth(0, gated[0])
        y_raw = self._smooth(1, gated[1])
        # The pot skips the median the axes get: _pot_gate() carries its own
        # noise handling (small changes need agreeing samples), so the median's
        # one-sample delay bought nothing on this channel - and the pot's lag
        # is the one the operator actually feels, on the lamp. Raw and straight
        # into the gate, exactly one poll between hand and latch on a big move.
        pot_raw = self._pot_gate(gated[2])

        analog["adc_error"] = None
        # Every channel None repeatedly means the bus is wedged rather than one
        # transfer being unlucky. Drop it and let the retry loop reopen, instead
        # of reporting a dead stick forever.
        if fresh[0] is None and fresh[1] is None and fresh[2] is None:
            self._dead_reads += 1
            if self._dead_reads >= ADC_DEAD_READS:
                self._release_adc()
                analog["adc_error"] = "ADC stopped responding - reopening"
        else:
            self._dead_reads = 0

        x_norm = self._norm_axis(0, x_raw)
        y_norm = self._norm_axis(1, y_raw)
        # Orientation applied AFTER normalisation, so the centre learner and
        # every validation gate reason about raw electrical travel and only
        # the demand the motors see is flipped. See INVERT_X / INVERT_Y.
        if x_norm is not None and INVERT_X:
            x_norm = -x_norm
        if y_norm is not None and INVERT_Y:
            y_norm = -y_norm
        analog["joy"] = {
            "x": x_norm, "y": y_norm,
            "x_raw": x_raw, "y_raw": y_raw,
        }
        analog["pot"] = (
            {"pct": max(0.0, min(100.0, 100.0 * pot_raw / FULL_SCALE)),
             "raw": pot_raw, "volts": pot_raw / COUNTS_PER_VOLT}
            if pot_raw is not None else
            {"pct": None, "raw": None, "volts": None})
        analog["adc_hz"] = self._adc_hz
        analog["adc_rejects"] = dict(self._rejects)
        # Stamped on every pass, INCLUDING one where every channel was rejected:
        # this timestamp answers "is the reader alive", not "is the reading good".
        # Conflating the two would have a burst of rejected reads look like a dead
        # thread, and the reader is doing exactly its job in that case.
        analog["adc_updated"] = time.time()
        with self._lock:
            self._state.update(analog)

    def run(self):
        """The switch loop. Starts the ADC thread and then keeps its own time."""
        if not self._open():
            return
        if ADC_ENABLED:
            self._adc_thread = threading.Thread(target=self._adc_run, daemon=True)
            self._adc_thread.start()
        else:
            # Said in the strip, not just implied by dashes: a deliberately
            # disabled stick and a broken one must not look the same.
            with self._lock:
                self._state["adc_error"] = "ADC off for testing (INPUTS_ADC_ENABLED=0)"

        period = 1.0 / max(1.0, POLL_HZ)
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                try:
                    self._read_switches()
                except Exception as exc:
                    with self._lock:
                        self._state["ok"] = False
                        self._state["switch_error"] = f"read failed: {exc}"
                self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            # Let the ADC thread notice the stop flag and drop the bus before the
            # switch lines go, so _close() is not racing it for the pinctrl reset.
            if self._adc_thread is not None:
                self._adc_thread.join(timeout=1.0)
            self._close()

    def _adc_run(self):
        """The analog loop. Runs at whatever rate the bit-bang allows."""
        next_retry = 0.0
        smoothed_hz = 0.0
        period = 1.0 / max(1.0, ADC_POLL_HZ)
        while not self._stop.is_set():
            started = time.monotonic()
            # Bring the ADC back if it is missing - at start-up or after it
            # dropped out. See ADC_RETRY_S.
            if not self._adc_present() and started >= next_retry:
                next_retry = started + ADC_RETRY_S
                self._open_adc()
            try:
                self._read_adc()
            except Exception as exc:
                with self._lock:
                    self._state["adc_error"] = f"read failed: {exc}"
            # Measured, not assumed. Includes the sleep below, so this is the rate
            # samples actually reach the motors at - the number to look at when the
            # stick feels behind the operator's hand.
            elapsed = time.monotonic() - started
            wait = max(0.0, period - elapsed)
            hz = 1.0 / max(1e-6, elapsed + wait)
            smoothed_hz = hz if not smoothed_hz else 0.9 * smoothed_hz + 0.1 * hz
            self._adc_hz = smoothed_hz
            if wait:
                self._stop.wait(wait)
        self._release_adc()


if __name__ == "__main__":
    # Prints the RAW counts and the validation counters alongside the decoded
    # values, because the two questions this module gets asked are "why is the
    # stick laggy" (adc= Hz) and "why did it move on its own" (rej= counters, and
    # whether raw x/y/pot are suspiciously equal). Neither is answerable from the
    # normalised numbers alone.
    reader = InputReader()
    reader.start()
    try:
        while True:
            time.sleep(0.5)
            s = reader.latest()
            if not s["ok"]:
                print("waiting:", s["error"])
                continue
            pills = " ".join(
                f"{n}={'ON ' if v else 'off'}" for n, v in s["switches"].items())
            joy, pot = s["joy"], s["pot"]
            rej = s.get("adc_rejects") or {}
            head = (f"{pills} | ACT={s['actuator']:<7} "
                    f"| {str(s['session']):<9} saves={s['save_presses']}")
            tail = (f"adc={s.get('adc_hz') or 0:.1f}Hz "
                    f"rej=range{rej.get('range', 0)}/agree{rej.get('agree', 0)}"
                    f"/jump{rej.get('jump', 0)}/read{rej.get('read', 0)}")
            def _n(v, fmt="{:+.2f}"):
                return "----" if v is None else fmt.format(v)
            print(f"{head} | joy x={_n(joy['x'])} y={_n(joy['y'])} "
                  f"raw {_n(joy['x_raw'], '{:>6}')},{_n(joy['y_raw'], '{:>6}')}"
                  f" | pot {_n(pot['pct'], '{:.0f}')}% raw {_n(pot['raw'], '{:>6}')}"
                  f" | {tail}"
                  + (f" | {s['error']}" if s.get("error") else ""))
    except KeyboardInterrupt:
        reader.stop()
        time.sleep(0.3)

