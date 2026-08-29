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

import math
import os
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
# The bit-bang helpers live in tools/, one level up and across. They are not
# diagnostics-only: the ADC fallback below imports them at runtime, which is why
# this path is computed rather than assumed. Moved there 2026-08-29 when the
# repo root was tidied; before that they sat loose beside it.
_PARENT = os.path.join(os.path.dirname(_HERE), "tools")

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
# 40 Hz was sized for the KERNEL bus (ads_i2c.py), where a 3-channel poll costs
# ~12 ms - mostly the chip's own conversion time - so a 25 ms period still left
# half the budget idle. Raised from 25 on 2026-08-17 because every
# sample-counted latency in this file (the median, the jump confirm, the pot's
# stability window) is priced in poll periods, and the reported pot lag was
# exactly those windows at 40 ms a tick.
#
# RAISED 40 -> 60 ON 2026-08-19, on the operator's report that the stick still
# lags. 60 Hz is a 16.7 ms period against that same ~12 ms poll, so the loop
# still finishes with ~28% of its budget spare rather than running flat out.
# MEASURED before making the change, so this is headroom rather than optimism:
#
#     load average 0.98 on 4 cores, main.py at 130% CPU, throttled=0x0
#     adc=40.0Hz sustained (the thread was pinned at its OLD ceiling, i.e.
#         it had capacity to spare and the CEILING was the limit, not the bus)
#     67 rejected samples in 2h31m of log = 0.018% of ~362,000 samples
#
# That last number is why tightening the windows is safe: every sample-counted
# gate here shortens IN TIME as the rate rises, which trades a little noise
# immunity for latency - and the noise gates were rejecting essentially nothing.
#
# BE HONEST ABOUT THE SIZE OF THIS WIN. It removes ~17 ms from a stick-to-wheel
# budget still dominated by MOTOR_SLEW_PER_S in uno_motors.py (150 ms). The ADC
# was measured NOT to be the bottleneck; this is the last cheap millisecond on
# this side, not the fix for a sluggish stick.
#
# On the userspace-bit-bang FALLBACK this cap is unreachable (~32 ms/poll idle,
# far worse under load) and the thread just runs as fast as that path allows,
# same as before.
ADC_POLL_HZ = float(os.environ.get("INPUTS_ADC_POLL_HZ", "60"))

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
#
# LOWERED 0.08 -> 0.04 ON 2026-08-19. THE BAND THE OPERATOR ACTUALLY FELT WAS
# 12.3%, NOT 8%, BECAUSE A SECOND DEADBAND STACKS ON TOP OF THIS ONE.
#
# uno_eth_link's own DEADBAND (12 of 255) is applied to the value this function
# has ALREADY rescaled, and it does not rescale again - the exact double-band
# failure that UNO_DEADZONE in uno_serial.py was set to 0.0 to cure, just with
# the second copy living on the Uno instead of in Python. The arithmetic: a
# wheel needs PWM >= 12, i.e. normalised >= 12/255 = 0.047, which needs raw
# deflection >= 0.08 + 0.047 * 0.92 = 0.1233 of half travel.
#
# MEASURED, not derived: over 7433 logged frames in ~/motor_cam.log the 5th
# percentile deflection on frames where the wheels were actually commanded was
# 1056 counts, against 1085 predicted by that formula and 704 predicted if this
# band were the only one. The operator reported motion starting below ~1300 and
# above ~1700 on a ~1500 centre, which is the same number in millivolts.
#
# 0.04 here plus DEADBAND 4 on the Uno gives 0.04 + 4/255 * 0.96 = 0.0551, or
# ~485 counts. THE SAFETY MARGIN IS ALSO MEASURED: on the same log the stick at
# rest stayed within 240 counts at the 95th percentile (median 80), so 485 is
# about 2x the observed resting spread and the wheels still cannot creep.
#
# LOWER THIS FURTHER ONLY AGAINST THAT NUMBER, not by feel - re-run the resting
# spread from the log first. And if you change it, remember the Uno's DEADBAND
# is the other half of the total.
# RAISED 0.04 -> 0.289 ON 2026-08-19, on the operator's instruction: nothing at
# all must happen in the first 30% of stick travel, in every direction.
#
# RAISED 0.289 -> 0.3396 ON 2026-08-24, same instruction, 30% -> 35%.
#
# WHY NOT SIMPLY 0.35. The Uno's own DEADBAND (4 of 255) still stacks on top of
# this one, and it is applied AFTER the rescale below, so it adds
# 4/255 * (1 - D) of stick travel. Solving for a 35.0% total:
#
#     0.35 = D + (4/255) * (1 - D)   ->   D = (0.35 - 4/255) / (1 - 4/255)
#                                          = 0.33964
#
# So 0.3396 here puts the FIRST MOVING SAMPLE at 35.0% of travel, which is what
# was asked for. Setting this to 0.35 would put it at 36.0%.
#
# AND THIS IS NOW 35% OF REAL STICK TRAVEL, which it was not before today. Until
# AXIS_TILT_FRAC landed the deflection was scaled against the 3.3 V rail rather
# than the stick's measured ±6150 counts, so the nominal 30% band was really
# eating 41.6% of the travel a hand can actually produce. Both numbers below are
# fractions of what the stick really does.
#
# This is per-AXIS, applied to x and y separately in _norm_axis, which is what
# makes the band 30% in forward, backward, left and right alike rather than a
# 30% circle around centre.
#
# The travel past the band is still RESCALED to the full 0..1 range, so there is
# no step at the edge - the remaining 70% of stick now carries the whole demand
# range, which makes the stick more sensitive per millimetre past 30%. That is
# inherent to spending travel on a deadband, not a side effect to fix.
# 0.3179 FOR A TRUE 35%, corrected 2026-08-24 after reading the firmware.
#
# THE OLD 0.3396 WAS SOLVED AGAINST A STALE NUMBER. The note above says the
# Uno's own band is "4 of 255" - it is not, and has not been for some time:
# uno_eth_link.ino has `const int DEADBAND = 12`. Since that band is applied
# AFTER this rescale, the arithmetic is:
#
#     total = D + (DEADBAND/255) * (1 - D)
#
# and 0.3396 with the real 12/255 gives 37.1%, not the 35.0% it was solved for.
# The operator had been driving a dead zone 2 percentage points wider than the
# one written down, in every direction.
#
# Re-solved with the measured firmware value:
#
#     D = (0.35 - 12/255) / (1 - 12/255) = 0.31790
#     check: 0.3179 + 0.04706 * (1 - 0.3179) = 0.35000
#
# IF THE FIRMWARE'S DEADBAND EVER CHANGES, this has to be re-solved with it.
# The two numbers only add up to 35% together, and nothing checks that at
# runtime - which is exactly how they drifted apart the first time.
AXIS_DEADBAND = float(os.environ.get("INPUTS_AXIS_DEADBAND", "0.3179"))

# HOW FAR THE STICK ACTUALLY MOVES, as a fraction of the centre count.
#
# _norm_axis used to divide the deflection by FULL_SCALE / 2, i.e. it assumed
# the stick swings all the way to the 3.3 V rail and to 0 V. No potentiometer
# does: it stops on a mechanical end stop with resistance still on both sides of
# the wiper. MEASURED on this rig (~/joy_native3.log, the centring pass):
#
#     centre X=8590 (spread 16)      centre Y=8655 (spread 16)
#     full tilt = 6150 counts (0.716 x centre - scales with the supply rail)
#
# 6150, not the 8800 that FULL_SCALE / 2 assumes. So a fully deflected stick
# reached only 6150/8800 = 0.699, and after the deadband rescale below that came
# out as (0.699 - 0.289) / (1 - 0.289) = 0.577 - the operator pushing the stick
# to its stop got 58% of the demand range and could not reach full PWM at all.
# That is the "full throttle does not reach 100" report, 2026-08-24.
#
# EXPRESSED AGAINST THE CENTRE, NOT AS AN ABSOLUTE COUNT, because the log line
# above says why: the tilt "scales with the supply rail" - both the centre and
# the end stops are the same divider fed from the same 3.3 V, so their RATIO
# holds when the rail sags under motor load while an absolute 6150 would not.
# The centre is already learned per axis at startup, so this rides along for
# free and stays correct on a rail this file cannot measure.
#
# 0.67, MEASURED ON THE RIG rather than derived from joy_native3.log.
#
# 0.70 was a 2% margin under that log's 0.716, and on the real stick it left
# full deflection reading 96%, not 100% - so the true tilt is smaller than the
# log's centring pass suggested. Solving back from the observed 96%, with the
# deadband D = 0.3396 applied after this scale:
#
#     displayed = (v - D) / (1 - D)      v = 0.96 * (1 - D) + D = 0.9736
#
# i.e. full stick only reached 0.9736 of the span 0.70 assumed, so the real
# tilt is 0.70 * 0.9736 = 0.6815 x centre, about 5854 counts.
#
# 0.67 rather than 0.6815 for the same reason 0.70 was not 0.716: it puts 100%
# at ~98% of travel, so the last of the PWM does not depend on how hard the end
# stop is pushed. The excess clamps. If full stick ever reads short again,
# re-solve with the formula above rather than nudging this by feel.
# PER DIRECTION, NOT PER AXIS, AND MEASURED - 2026-08-24.
#
# THE STICK DOES NOT TRAVEL THE SAME DISTANCE EACH WAY. Swept to all four stops
# and logged (4502 samples), as a fraction of each axis's learned centre:
#
#     centre X = 8032        centre Y = 10656
#     X +: 0.8685            Y +: 0.6261
#     X -: 0.9442            Y -: 0.6577      (8.0% / 4.8% asymmetric)
#
# WHY THIS MATTERS FOR THE DEAD ZONE, which is what sent me looking. The band is
# a fraction of the NORMALISED value, so it is a fixed number of COUNTS - and a
# fixed count is a different fraction of travel in each direction whenever the
# resting centre is not the mechanical middle. With the old single-per-axis
# spans (0.67 / 0.44, both guesses) the "35%" dead zone was really:
#
#     X + 26.2%    X - 24.1%    Y + 23.9%    Y - 22.7%
#
# - never 35, and never the same twice. Operator report: "not set in forward
# 0-35%". Scaling each direction by ITS OWN travel makes the normalised value
# reach 1.0 at that direction's own stop, so the band is the same fraction of
# travel everywhere and the number finally means what it says.
#
# x0.98 ON EACH, the same stop margin as before: full output lands ~2% of travel
# before the mechanical stop, so 100% is reachable by hand instead of depending
# on how hard the stop is pushed. It costs 0.7% of dead zone (34.3% of travel
# rather than 35.0%), which is well inside what a hand can feel.
#
# RE-MEASURE, DO NOT NUDGE, if the stick is ever re-wired or replaced: log
# x_raw/y_raw with the learned centres through a four-corner sweep and divide.
AXIS_TRAVEL_X_POS = float(os.environ.get("INPUTS_AXIS_TRAVEL_X_POS", "0.851"))
AXIS_TRAVEL_X_NEG = float(os.environ.get("INPUTS_AXIS_TRAVEL_X_NEG", "0.925"))
AXIS_TRAVEL_Y_POS = float(os.environ.get("INPUTS_AXIS_TRAVEL_Y_POS", "0.614"))
AXIS_TRAVEL_Y_NEG = float(os.environ.get("INPUTS_AXIS_TRAVEL_Y_NEG", "0.645"))

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
# Orientation as confirmed on the robot 2026-08-18 night: left/right true,
# forward/back reversed - so Y alone was flipped then.
#
# X IS NOW FLIPPED TOO (2026-08-19). The operator reported the turns swapped:
# stick left steered right and stick right steered left, while forward and back
# were correct. An X-only reversal is exactly that signature.
#
# NOTE THE AMBIGUITY, because it matters if this ever has to be re-derived.
# "Turns reversed, forward/back correct" is produced IDENTICALLY by a reversed
# X axis and by the two MOTOR CHANNELS being swapped in the wiring - in arcade
# mixing both flip the sign of x relative to y and neither touches straight
# running. Flipping it here fixes the feel either way, and this file is the
# designated home for stick orientation, so it is the right first move.
#
# But if the real fault is swapped channels, then telemetry "L=" is driving the
# physical RIGHT wheel, and that will mislead whoever debugs this next. To settle
# it, drive one wheel on its own with the wheels off the ground:
#
#     python3 uno_motors.py --test --only LEFT
#
# If the RIGHT wheel turns, the channels are swapped and the honest fix is
# INVERT_1/INVERT_2 or the pin map in the sketch, not this line.
# BACK TO 0 (2026-08-24). Operator: "my backend left right is interchange" -
# the robot steers against the hand again, forward/back correct, which is the
# X-only signature described above. This is the third flip of this value; the
# comment above says why that is expected of a wiring-dependent constant.
#
# PAIRED WITH A DISPLAY FLIP IN inputs_panel.JoystickView. What arrives at the
# panel is the ALREADY-inverted value, so changing this moves the dot as well as
# the wheels - and the dot was correct. The two are flipped together in this one
# change so the display ends up exactly where it was; see the note at `ox` in
# that file. If you ever flip one of them alone, check the other.
# --- which stick axis is which -----------------------------------------------
# SWAP_XY exchanges the two ADC channels' ROLES: A0 becomes forward/back and A1
# becomes left/right, instead of the other way round.
#
# TRUE 2026-08-26. Operator: "my forward backward is left right and left right
# is forward backward". That is a wiring fact, not a preference - the stick's
# two pots are landed on the opposite ADC inputs to what this file assumed - and
# it is exactly the kind of thing this layer exists to absorb. The alternative
# is unplugging two wires at the panel, which is worse for the same result.
#
# APPLIED BEFORE INVERT_X / INVERT_Y ON PURPOSE, so those two keep meaning what
# their names say from the OPERATOR's seat: INVERT_X flips left/right as the
# hand experiences it, INVERT_Y flips forward/back. Swapping after them would
# leave INVERT_X secretly controlling forward/back, which is exactly the sort of
# trap that has cost this rig a day at a time.
#
# NOT APPLIED TO THE CALIBRATION. AXIS_TRAVEL_* and the centre learner still key
# off the PHYSICAL channel (ch 0 / ch 1) inside _norm_axis, because those were
# measured against real electrical travel and the wiring did not change - only
# the label on it did.
# FLIPPED TO 0 on 2026-08-29, and this one was MEASURED rather than reasoned.
#
# Operator: "the issue in the physical mounting is not align with the joystick in
# the ui. i am holding front physically but showing right in ui". With the stick
# held physically FORWARD the logger recorded fourteen consecutive samples of
#
#     x = -1.0    y = 0.0
#
# Full deflection on the ELECTRICAL X channel and nothing at all on Y. The stick
# is mounted a quarter turn round, so its forward throw lands on the channel this
# code was calling turn.
#
# WHY THIS BELONGS HERE AND NOT IN THE PANEL. A display-only swap lived in
# inputs_panel.py for three days doing half of this job, and it could only ever
# have been half: the panel and the mixer read the SAME joy["x"] / joy["y"], so
# crossed axes bend the wheels exactly as much as the arrows. Correcting it at
# the source fixes both at once, and it is the honest description of the fault -
# the wiring is rotated, not the picture.
#
# WHAT THIS ALSO EXPLAINS, which is how a display bug turned out to be a drive
# bug. The FRONT/BACK camera highlight lights on `left + right`, the pair's
# common component, which a pure turn cancels to zero. With the axes crossed, a
# physical LEFT/RIGHT throw arrived as y and drove both wheels together - a
# non-zero sum - so turning lit a camera while pushing forward did not. That was
# reported as a highlight bug and chased as one; it was this.
SWAP_XY = os.environ.get("INPUTS_SWAP_XY", "0") == "1"

INVERT_X = os.environ.get("INPUTS_INVERT_X", "0") == "1"
# Y FLIPPED TO 0 (2026-08-25). Operator: "my forward backward is interchange in
# my frontend". The panel's forward/reverse arrows, the FRONT/BACK camera
# highlight and the wheels ALL read from the same already-inverted y - arrows
# via `oy = self._y` in inputs_panel (oy < 0 is forward), the highlight via
# drive = left + right in main.py (drive < 0 is forward), the wheels via mix().
# There is no path by which the frontend can be backwards while the motors are
# right, so the sign itself is backwards and this is where it gets corrected.
#
# NOT PAIRED WITH A DISPLAY FLIP, unlike INVERT_X above - and that is the whole
# difference between the two cases. There the dot was already following the hand
# and only the motors were wrong, so `ox` had to cancel the change. Here the
# frontend is the thing being corrected, so `oy` stays exactly as it is and the
# arrows, the highlight and the wheels all turn over together.
#
# WHY THE SIGN MOVED AT ALL. The learned centres shifted hard between the
# 2026-08-24 calibration and today - X 8032 -> ~9400, Y 10656 -> ~8900 - which is
# the signature of a re-wired or replaced stick, and absorbing that is exactly
# what this constant is for. The AXIS_TRAVEL_* fractions above were measured
# against those OLD centres and are stale in the same way; they have NOT been
# touched here and still want re-measuring.
#
# THE WHEELS WERE NOT VERIFIED WHEN THIS WAS CHANGED. The Uno at 192.168.50.20
# was not answering (eth0 up, both cameras replying, Uno silent), so only the
# frontend half could be confirmed on the rig. If the wheels ever drive against
# the hand while the arrows read right, the fault is DOWNSTREAM of this line -
# the driver's DIR lines or INVERT_1/INVERT_2 in the sketch - and flipping this
# again will only move the error onto the display. Settle it with
# `python3 uno_motors.py --test --only LEFT`, wheels off the ground.
INVERT_Y = os.environ.get("INPUTS_INVERT_Y", "0") == "1"

# How long a rejected sample may be papered over with the last GOOD reading.
#
# The validation gates reject single samples by design - and MotorLink's slew
# limiter applies FALLS instantly (a stop must never be delayed) while ramping
# rises over 400 ms. Put together, one rejected 30 ms sample became a full
# stop plus a 400 ms crawl back up: measured 2026-08-18 night holding the
# stick at the full-right rail, the log read L=+255, +16, +59, +255 on
# consecutive seconds ("left right not working properly" - the x wire is the
# marginal one, so turns stuttered while forward/back felt fine).
#
# Bridging a gap this short with the last VALIDATED value cannot drive the
# robot on garbage - garbage is exactly what was rejected - and a genuine
# outage still reads None the moment the gap outlives the hold. The stick
# telling the robot to stop is a VALUE (centre), not a gap, so stopping is
# never delayed by this either.
AXIS_HOLD_S = float(os.environ.get("INPUTS_AXIS_HOLD_S", "0.15"))

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
POT_STABLE_SAMPLES = int(os.environ.get("INPUTS_POT_STABLE_SAMPLES", "1"))
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

# HYSTERESIS ON THE REPORTED PERCENTAGE, in percent. 2026-08-24.
#
# _pot_gate() latches RAW COUNTS and re-adopts raw every time the window agrees
# - which, with a quiet wiper, is constantly. The latch therefore tracks noise
# of a few counts, and 100 * raw / FULL_SCALE turns a few counts into a fraction
# of a percent that rounds up and down forever: the panel sat there reading
# 74, 75, 74, 75 with nobody touching the knob.
#
# This is NOT a second stability gate - the gate is doing its job, holding a
# steady SETTING. It is a display/command quantiser: the reported number only
# moves once the true reading is at least this far from the number already
# being shown, so a still knob reports one fixed integer and a turned knob
# still tracks smoothly.
#
# 0.9 not 0.5: at 0.5 a reading sitting exactly between two integers still
# flips, which is the failure being fixed. Just under 1.0 means one full
# percent of real movement is always enough to move the display, so nothing is
# lost - the knob has ~100 usable steps either way.
#
# IT FIXES THE LAMP TOO, not just the readout: uno_serial.light_demand() takes
# this same percentage and turns it into PWM, so the dither was being sent to
# the light as a 1/255 flicker on every frame.
# ONE STEP PER SAMPLE. Operator 2026-08-25: "its fast increase or decrease
# unstable i want to very reliable in 1 value increase pwm".
#
# The published percent may move by at most POT_MAX_STEP per ADC sample, so the
# lamp walks to a new setting instead of jumping to it. At the measured ~58 Hz
# poll a step of 1 crosses the whole range in about 1.7 s, which is faster than
# a hand turns the knob deliberately and far slower than any glitch.
#
# This is the single most effective thing on a failing pot, and better than any
# of the filters above it: a wiper dropout that used to teleport the reading to
# 100% can now drag it 1 point per sample, so a 260 ms excursion moves the lamp
# ~15 points and walks straight back when contact returns. It converts a flash
# to full brightness into a barely visible waver, WITHOUT the window lag a
# median needs, because it never has to wait to decide - it just refuses to move
# far in one go.
#
# The cost is honest and bounded: a deliberate fast sweep is followed at 58
# points/second rather than instantly. Raise POT_MAX_STEP for a snappier knob,
# set it to 0 to remove the limit entirely.
# DISPLAY-ONLY smoothing. Operator 2026-08-25: the lamp is right but the number
# on screen is not - "light value is done by uno and send but not show properly
# in frontend".
#
# Both used to be the same field, so making the lamp instant necessarily made
# the readout twitchy: with every filter stripped, each wiper glitch lands on
# screen at full size. But the two want opposite things. The LAMP wants the
# newest reading, because latency there is latency the hand feels. The NUMBER
# wants a settled reading, because a digit that flickers is unreadable and
# nobody is steering by it.
#
# So the pot now publishes both: "pct" stays instant and drives the lamp, and
# "pct_view" is medianed for the panel. The window must be more than TWICE the
# worst excursion, not merely longer than it: a median only rejects a wrong
# value while wrong values are a MINORITY of the window. 0.5 s was tried first
# and failed exactly there - a 260 ms glitch is 15 samples against a 29-sample
# window, a majority, so the readout followed it to 100%. 1.2 s clears the worst
# measured 462 ms with margin. Display lag costs nothing here - it is a
# label, not a control path.
#
# Set INPUTS_POT_VIEW_S=0 to make the readout instant again, and identical to
# what the lamp gets.
# DEFAULT 0 - THE READOUT IS INSTANT TOO. Operator 2026-08-25: "in frontend now
# its show delay change it to instant chage value". At 1.2 s the number trailed
# the knob by ~600 ms, which reads as lag even though it is only a label.
#
# So both paths are now unfiltered and identical. The glitches reach the screen
# as well as the lamp; that is the accepted trade, and the sizing below is kept
# because it was measured, not guessed. INPUTS_POT_VIEW_S=1.2 restores it.
POT_VIEW_S = float(os.environ.get("INPUTS_POT_VIEW_S", "0"))

POT_MAX_STEP = float(os.environ.get("INPUTS_POT_MAX_STEP", "0"))
POT_PCT_HYST = float(os.environ.get("INPUTS_POT_PCT_HYST", "0.9"))

# WIPER DROPOUT REJECTION. Measured on this rig 2026-08-25.
#
# The pot's wiper intermittently lifts off its track. While it is open the ADC
# input floats to the top of the range, so the reading teleports to full scale
# and back: 3% -> 100.0% inside ONE 17 ms sample, held ~290 ms, then back to the
# same 3% it left, thirteen times in thirty seconds. The operator sees the value
# crawl with their hand and then leap, which reads as "slow up to 15 then fast".
#
# The generic jump test cannot catch it. ADC_JUMP_CONFIRM accepts any jump that
# two consecutive samples agree on, which is correct for the joystick - a real
# flick IS fast - but a 290 ms dropout clears that bar in 33 ms and the rest of
# the burst is not even a jump any more.
#
# What separates them is not speed but WHERE it lands. A dropout always ends up
# pinned at the rail, because that is where an undriven input goes. A hand can
# also reach 100%, but only by walking there - the sample before it is at 96%,
# not at 3%. So: a big step that ENDS at the rail is the wiper, and a small step
# that ends at the rail is the operator. That rule needs no timing at all, which
# matters because a duration-based rule has to guess how long a dropout lasts.
#
# Only the TOP rail is tested. This ADC input floats high, measured; nothing has
# ever been seen to float low here, and adding an untested bottom-rail rule
# would risk rejecting a genuine fast turn down to zero.
# MEDIAN WINDOW over the pot, sized from the failing wiper - see _pot_median.
#
# The rail-dropout test above only catches excursions that END at the rail. This
# pot also glitches to arbitrary mid-range values, measured 2026-08-25 over 400 s
# of real use: 18 round-trip excursions, median 15 samples / 259 ms, worst 27
# samples / 462 ms, landing on 2, 4, 9, 71 and 100. Only 6% lasted 4 samples or
# fewer, so no confirmation count separates them - a 260 ms excursion is exactly
# as long as a real hand movement.
#
# A median does separate them, because it does not care how long a wrong value
# lasts, only whether wrong values are the MAJORITY of the window. These
# excursions are isolated, so any window comfortably longer than twice the worst
# one holds the true reading throughout.
#
# 0.9 s was chosen by replaying the 9640 logged samples through candidate
# windows rather than by reasoning about it:
#
#     window   >=20pt jumps   worst jump
#     none          20            52
#     0.3 s         13            48
#     0.6 s          5            35
#     0.9 s          0            18     <- first window that removes them all
#     1.5 s          0            18        (wider buys nothing)
#
# The cost is half a window of lag on a genuine turn, ~450 ms. That is real and
# it is the trade being made: a dimmer knob that follows half a second late
# beats one that leaps to full brightness while the operator nudges it.
#
# THIS IS A MITIGATION, NOT A REPAIR. Set INPUTS_POT_MEDIAN_S=0 once the pot has
# been replaced - a healthy wiper needs none of this and gets its response back.
# DEFAULT 0 - THE MEDIAN IS OFF. Operator, 2026-08-25: "increase decrease light
# is delayed and also delayed data show in frontend". It was, and this was why:
# measured at 58 Hz the window cost ~450 ms of the ~570 ms total in this path,
# every other stage together costing ~120 ms. A dimmer that answers half a
# second late is worse to use than one that occasionally twitches, and that is
# the operator's call to make.
#
# The window sizing below is kept because it was measured, not guessed, and
# because the wiper it was built for has not been repaired. Set
# INPUTS_POT_MEDIAN_S=0.9 to switch it back on without touching code.
# STEP LIMIT. The discriminator that costs no latency at all.
#
# Turning the knob and a wiper glitch look nothing alike PER SAMPLE. Measured
# 2026-08-25 over 56,971 samples of real use: a real turn steps 1 point at the
# median and 10 at the 90th percentile, while every glitch onset moved 40-97
# points between two consecutive 17 ms reads. So an implausible step is rejected
# on the spot and the previous value held - normal turning never trips it, which
# is why this adds nothing to the response the operator feels.
#
# A rejected level that PERSISTS for POT_STEP_HOLD_S is then accepted anyway, so
# a violent flick of the knob is delayed rather than refused. The hold is longer
# than the worst measured glitch (462 ms) so a glitch cannot outlast it.
#
# Replayed over that log the limiter alone takes >=20-point jumps from 47 to 12
# while holding 0.7% of samples. It is not a cure - a glitch that happens to
# land within the limit still gets through - and nothing here repairs the pot.
POT_STEP_MAX = float(os.environ.get("INPUTS_POT_STEP_MAX", "0.20"))
POT_STEP_HOLD_S = float(os.environ.get("INPUTS_POT_STEP_HOLD_S", "0.6"))

# Small median BEHIND the step limiter, to catch what lands inside the limit.
# 0.10 s, not the 0.9 s that was measured for a median working alone: the
# limiter has already removed the large excursions, so this only has to smooth
# the residue. ~50 ms of lag against the 450 ms the big window cost.
POT_MEDIAN_S = float(os.environ.get("INPUTS_POT_MEDIAN_S", "0"))

# TAPER CORRECTION - a smooth curve, deliberately NOT the measured table.
#
# The knob is an extreme log taper. Measured 2026-08-25 from a continuous sweep
# logged in RAW ADC counts:
#
#   rotation   0    20    40    50    60    70    80    90   100
#   raw     %  0   2.7   6.2   8.4   8.8   9.1  11.8  28.5   100
#
# The first 80% of travel carries 11.8% of the signal, which is why the knob
# reads "slow, then all at once".
#
# TWO EARLIER ATTEMPTS FAILED, and both failures are the reason this is a smooth
# function now rather than a lookup:
#
#  1. A table inverted from the integer PERCENT. That had four distinct values
#     across the first 70% of travel, so its first segment mapped a reading of 1
#     onto an output of 20 - a 20x gain that the operator felt immediately:
#     "in the potentiometer value 0 to 25 full fast speed".
#
#  2. A table inverted from the RAW counts above. Better resolution, but rotations
#     50/60/70 land on 1454/1520/1574 counts - a plateau no potentiometer has.
#     That is hand speed and a glitching wiper recorded as if it were geometry,
#     and inverting it puts a 28x gain in the MIDDLE. Fitting a table to one hand
#     sweep fits the hand, not the pot.
#
# So: one exponent, applied smoothly. out = 100*(pct/100)**(1/POT_GAMMA). Its
# gain is bounded everywhere and it cannot encode a measurement artefact, at the
# cost of not tracking every wiggle in the real taper - which is the right trade
# when the wiggles are measurement noise.
#
#   POT_GAMMA = 1.0  no correction, the raw taper
#   POT_GAMMA = 2.0  the default: the bottom of the travel lands on the ideal
#                    ramp exactly, where 2.5 over-boosts it back to 16 at 10%
#   POT_GAMMA = 4.0  near-even numbers, noticeably twitchy at the bottom
#
# Tune it live without a code edit: INPUTS_POT_GAMMA=3 python3 main.py
#
# NO SETTING MAKES THIS KNOB GOOD. Stretching a region holding 11.8% of the
# signal cannot invent resolution the pot never encoded, and it amplifies the
# failing wiper in exact proportion. A LINEAR pot needs none of this - set
# INPUTS_POT_GAMMA=1 the day one is fitted.
POT_GAMMA = float(os.environ.get("INPUTS_POT_GAMMA", "2.0"))
POT_LINEARISE = os.environ.get("INPUTS_POT_LINEARISE", "1") == "1"

# --- the measured taper, and why the power law above was never going to work --
# MEASURED 2026-08-26, 16 separate bottom-to-top sweeps of the knob, median at
# each 5% of travel. Time is the stand-in for shaft angle, which is why it took
# SIXTEEN sweeps: any one hand speeds up and slows down, but the errors are
# random and the median cancels them. The agreement is tight (inter-quartile
# spread under 3.3 points) everywhere except the 70-80% knee, where the pot's
# own steepness magnifies any speed wobble - that region is the least certain
# part of this table and the first place to re-measure if the feel is off.
#
# WHAT IT SHOWS, and it is worse than "a log taper":
#
#     the first 70% of the knob covers 19% of the electrical range
#     the last  30% of the knob covers 72%
#
# The operator's report was "after 50 its fast increase", and that is exactly
# this: nothing happens for two thirds of the turn, then it arrives all at once.
#
# WHY NO EXPONENT FIXES IT. Fitted against the measured profile:
#
#     gamma 2.0 (what was here)   rms error 14.6 points
#     gamma 2.55 (best possible)  rms error 11.9
#     log curve, C=33 (best)      rms error  8.2
#     this table                  rms error  0.0
#
# A power law is the wrong SHAPE - it cannot bend the way this pot bends - and
# the best gamma available is only 2.7 points better than the one already set.
# That is why POT_GAMMA was retuned twice and still felt wrong: the knob was
# never going to be fixed by choosing a better exponent.
#
# A TABLE HAS NO SHAPE OF ITS OWN, which is the whole point: it follows whatever
# the pot actually does, including the knee, and it is re-measurable in one
# sweep if the pot is ever replaced.
#
# STILL A MITIGATION, NOT A REPAIR. This straightens the READING; it cannot
# invent resolution the pot never encoded. Below the knee the whole bottom two
# thirds of the turn is squeezed into ~3300 counts, so it stays coarse - it just
# stops pretending otherwise. A LINEAR pot needs none of this: fit one, set
# INPUTS_POT_CURVE=none, and delete the table.
POT_LUT_DEFAULT = ("158,348,570,802,980,1171,1372,1624,1874,2135,2372,"
                   "2595,2806,3066,3322,3917,5287,8181,11010,13168,16110")
try:
    POT_LUT = [float(x) for x in
               os.environ.get("INPUTS_POT_LUT", POT_LUT_DEFAULT).split(",")
               if x.strip()]
except ValueError:
    POT_LUT = []

# Which correction to apply:
#   lut    the measured table above                            [default]
#   log    100*ln(1+C*r)/ln(1+C), C = INPUTS_POT_LOG_C - a smooth
#          approximation, worse than the table but immune to a bad measurement
#   gamma  the old power law, POT_GAMMA
#   none   raw proportion, for a LINEAR pot
POT_CURVE = os.environ.get("INPUTS_POT_CURVE", "lut").strip().lower()
POT_LOG_C = float(os.environ.get("INPUTS_POT_LOG_C", "33"))


def _lut_pct(raw):
    """Counts -> knob position 0..100, interpolating the measured table.

    The table is raw counts at evenly spaced knob positions, so this is its
    INVERSE: find which pair of breakpoints the reading falls between and
    report where it sits across that step. Linear inside a step, because 5% of
    travel is finer than anything the operator can feel or the lamp can show.
    """
    n = len(POT_LUT)
    if n < 2:
        return None
    step = 100.0 / (n - 1)
    if raw <= POT_LUT[0]:
        # Below the first breakpoint the table says nothing, so fall back to a
        # straight line into zero rather than clamping - the sub-LSB floor in
        # _pot_pct has already decided what counts as "off".
        return 0.0 if POT_LUT[0] <= 0 else max(0.0, step * raw / POT_LUT[0])
    if raw >= POT_LUT[-1]:
        return 100.0
    for i in range(n - 1):
        a, b = POT_LUT[i], POT_LUT[i + 1]
        if a <= raw <= b:
            return step * (i + (0.0 if b == a else (raw - a) / (b - a)))
    return 100.0


def _linearise_pot(pct):
    """Straighten the taper. See POT_LUT_DEFAULT for what is being straightened.

    Takes and returns a PERCENTAGE, so `pct` arrives as the raw proportion of
    full scale and leaves as the knob's position. The conversion back to counts
    for the table is exact - both are the same number in different units.
    """
    if pct is None:
        return pct
    if POT_CURVE == "none":
        return pct
    if pct <= 0.0:
        return 0.0
    if pct >= 100.0:
        return 100.0
    if POT_CURVE == "lut":
        out = _lut_pct(pct / 100.0 * FULL_SCALE)
        if out is not None:
            return out
    if POT_CURVE == "log" and POT_LOG_C > 0:
        return 100.0 * math.log1p(POT_LOG_C * pct / 100.0) / math.log1p(POT_LOG_C)
    if not POT_LINEARISE or POT_GAMMA <= 1.0:
        return pct
    if pct <= 0.0:
        return 0.0
    if pct >= 100.0:
        return 100.0
    return 100.0 * (pct / 100.0) ** (1.0 / POT_GAMMA)
# --- the sub-LSB floor: why ONLY the bottom of the travel flickers ----------
# The ADS1115 at this PGA and data rate quantises in steps of 16 counts. Every
# pot value this rig has ever logged is a multiple of 16 - 0, 16, 2560, 2576,
# 3472, 3488 - so 16 counts is ONE ADC CODE and there is nothing in between.
#
# One code is 0.09% of full scale, which is invisible in the middle of the
# travel: at 3472 vs 3488 counts the readout moves 44.4% -> 44.5%, a tenth of a
# percent. But POT_GAMMA=2.0 is a square root, and a square root has INFINITE
# slope at zero. The same single code at the bottom lands 0 counts -> 0.0% and
# 16 counts -> 3.0%. The knob has not moved; the ADC has flipped one bit.
#
# MEASURED on the running rig 2026-08-25, a 5258-sample run with the knob held
# at the bottom (raw only ever 0, 16 or 32):
#
#   no filtering      0 <-> 3, 18.8 changes/SECOND   <- the reported flicker
#   kalman only       parks on 2, 1.2 changes/s      <- better, still visible
#   kalman + floor    0, 0.02 changes/s              <- chosen
#
# So the Kalman alone does not finish this. It averages the dither honestly and
# settles on its MEAN - about 7 counts, which the gamma curve prints as 2% - so
# a knob turned fully off reads 2 and still wobbles onto 3. The filter is not
# wrong; the reading genuinely is somewhere under one code, and no amount of
# averaging invents the resolution to say where.
#
# Below one code the ADC is not measuring, it is rounding. Treat it as zero:
# that is the only value in that band the hardware can actually support, and it
# is what "the knob is turned off" is supposed to mean. Costs one comparison
# and no latency, and it stops at the FIRST real code - 16 counts still reads
# 3%, 32 still reads 4.3%, so nothing above the noise floor is deadbanded away.
#
# This is a mitigation for a LOG-TAPER pot on a 16-bit ADC, not a repair. On a
# linear pot (INPUTS_POT_GAMMA=1) the bottom code is worth 0.09% like every
# other code and none of this is needed. INPUTS_POT_SUBLSB=0 disables it.
POT_LSB = float(os.environ.get("INPUTS_POT_LSB", "16"))
POT_SUBLSB = os.environ.get("INPUTS_POT_SUBLSB", "1") == "1"
POT_RAIL_FRAC = float(os.environ.get("INPUTS_POT_RAIL_FRAC", "0.97"))
POT_DROPOUT_STEP = float(os.environ.get("INPUTS_POT_DROPOUT_STEP", "0.40"))

# --- the pot's Kalman filter -------------------------------------------------
# A scalar (1-D) Kalman filter on the pot's counts, ahead of _pot_gate(). The
# knob is a slowly-varying quantity read by a noisy ADC, which is the textbook
# case for one, and it replaces "average the last N" with an average that
# weights each new reading by how much it is worth trusting.
#
# R IS MEASURED, NOT PICKED. 2711 samples logged off the running rig
# 2026-08-24 with the knob at rest: the readings quantise in steps of 16 counts
# and span +/-16, i.e. a standard deviation near 10 counts, so R = 10^2 = 100.
# That is only 0.09% of FULL_SCALE - tiny, but a value sitting near a rounding
# boundary flips on far less, which is exactly the flicker that was reported.
#
# CAVEAT ON R: those samples were taken with the knob at ZERO, because that is
# where it happened to be. If the wiper is noisier mid-travel - which is normal
# for a worn pot - R is understated and the filter will be trusting the ADC
# more than it should. Re-log at mid-travel and re-derive if the reading ever
# looks jumpy in the middle of the range but steady at the ends.
#
# Q FOLLOWS FROM R AND A CHOSEN GAIN rather than being tuned by feel. For a
# random-walk model the steady-state gain K solves Q = K^2*R/(1-K), so:
#
#     K = 0.20 -> Q = 5.0    (~5 samples averaged, ~200 ms of lag at 25 Hz)
#     K = 0.25 -> Q = 8.3    (~4 samples, ~160 ms)     <- chosen
#     K = 0.40 -> Q = 26.7   (~2 samples, ~100 ms)
#
# 0.25 buys a 4-sample average for 160 ms, which is under the ~200 ms the pot
# already spends in _pot_gate's stability window - so this costs no latency an
# operator can notice.
POT_KALMAN_R = float(os.environ.get("INPUTS_POT_KALMAN_R", "100.0"))
POT_KALMAN_Q = float(os.environ.get("INPUTS_POT_KALMAN_Q", "8.3"))
# Innovation beyond this is a HAND, not noise, and the filter reinitialises on
# it instead of dragging toward it over several samples. Without this the
# smoothing fights _pot_gate's own snap-on-large-move rule and a deliberate
# turn arrives late. 4% of full scale is well outside anything the measured
# noise can produce (0.09%) and inside POT_SNAP_BAND's 6%, so the filter is
# always already caught up by the time the gate decides to snap.
POT_KALMAN_SNAP = float(os.environ.get("INPUTS_POT_KALMAN_SNAP", "0.04"))
# The stage's own on/off, so it matches every other pot filter and the next
# round of "take the smoothing out" is an env var rather than a code edit -
# which is what the three previous round trips on this knob each cost.
POT_KALMAN = os.environ.get("INPUTS_POT_KALMAN", "1") == "1"

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

# A LONGER DEBOUNCE FOR THE SAVE PIN ALONE, and it is measured rather than
# picked. GPIO25 shares a wire with the dead GPIO9 and the line will not settle:
# 427 level changes in 15 seconds with nobody near the panel. But chatter cannot
# HOLD a level - measured over that window:
#
#     shortest run   4 ms
#     median        16 ms
#     p90           89 ms
#     longest      264 ms   <-- the number this has to beat
#
#     debounce 100 ms (the old floor) ->  30 of 427 runs survive  = phantom saves
#     debounce 300 ms                 ->   0 of 427 survive
#
# 500 ms is 300 with the margin doubled, because 264 ms was the worst run seen
# in one sample window and the next one may be worse.
#
# THIS COSTS NOTHING ON A LATCHING SWITCH. A latch that has been thrown holds
# its level indefinitely, so it clears any floor; only a glitch has to be short,
# and every one of them is. The pin it protects is the one that ticks rows in
# the USB chooser, where a phantom press selects footage nobody chose.
#
# It is a MITIGATION, not a repair: the shared GPIO9 wire is still the fault and
# pulling it is still the right fix. Set INPUTS_SAVE_DEBOUNCE_MS=0 to fall back
# to the common floor once it has been removed.
# DEFAULT 0 - THE LONG DEBOUNCE IS OFF, because it did not work and the reason
# is worth keeping.
#
# The idea was sound on paper: chatter cannot hold a level (worst run measured
# 264 ms) while a thrown latch holds for ever, so a 500 ms floor should pass one
# and reject the other. It did reject the chatter - the app saw zero phantom
# presses. It also rejected every REAL press, because a floor of 500 ms needs
# ten CONSECUTIVE agreeing samples and a line changing every ~36 ms never gives
# ten of anything. The button went from firing by itself to not firing at all,
# which is not an improvement.
#
# The fault is not filterable, and the measurement says exactly why. Toggling
# GPIO9 s bias alone, with nothing else changed:
#
#     GPIO9 pull-up   -> GPIO25 chatters   189 and 205 edges per 6 s
#     GPIO9 pull-down -> GPIO25 dead still   0 edges
#
# GPIO9 is shorted to ground and shares a wire with GPIO25, so GPIO25 sits
# between GPIO9 s pull-up and that short and oscillates. Pulling GPIO9 down
# stops the noise but parks the line at "pressed" for ever, so neither bias
# leaves a working button. THE WIRE TO GPIO9 (header pin 21) HAS TO COME OFF -
# no debounce can help, and this one made it worse.
SAVE_DEBOUNCE_MS = float(os.environ.get("INPUTS_SAVE_DEBOUNCE_MS", "0"))

# STUCK_CLOSED_S IS GONE, and deliberately not replaced. It timed how long a
# line could read closed before being called a short. That test was removed the
# same day it was added - see _fault_filter - because a latching switch left
# thrown is indistinguishable from a short by level alone, and it silently
# killed a working save button. Only the chatter thresholds below survive.

# CHATTER: GPIO9 was measured at ~11 edges/second indefinitely, with lows near
# the save button's real ~170 ms press - so duration cannot separate them, but
# rate can. 30 edges in 5 s is 3 full presses a second sustained for five
# seconds, which no hand produces and which the measured noise clears sixfold.
CHATTER_WINDOW_S = float(os.environ.get("INPUTS_CHATTER_WINDOW_S", "10.0"))
# RE-SIZED 2026-08-27 after the SAVE switch on GPIO25 started chattering too.
#
# The first threshold was set from GPIO9, which ran ~11 debounced edges/second;
# 30-in-5s caught that easily. GPIO25 chatters SLOWER - measured 551 raw edges
# in 20 s with nobody near the panel, arriving at the app as roughly 4 debounced
# edges/second, or about 20 in a 5 s window. It slipped under the old bar, and
# every phantom edge became a SAVE press: in the USB chooser each one calls
# _activate(), which ticks or unticks whatever row the cursor is on. The
# operator saw rows selecting and deselecting themselves and read it, sensibly,
# as the joystick doing it.
#
# A LONGER window rather than just a lower count, because that is what separates
# the two cases. Chatter is SUSTAINED; a hand presses in bursts. 24 edges in 10 s
# is 1.2 presses/second held for ten seconds straight, which no operator does,
# while the measured chatter turns in 40.
CHATTER_EDGES = int(os.environ.get("INPUTS_CHATTER_EDGES", "24"))

# Â±6.144 V PGA over a 16-bit signed range. 3.3 V rail => ~17600 counts full scale.
COUNTS_PER_VOLT = 32768 / 6.144
FULL_SCALE = 3.3 * COUNTS_PER_VOLT

# label -> BCM pin. Names come from Harshil; 16/19 are the actuator pair and are
# decoded together rather than shown as two anonymous pins.
# Roles confirmed with Harshil 2026-08-14. GPIO12 WAS unused then - measured
# completely idle across a 30 s window in which GPIO25 moved 56 times - and that
# is why it was left out. IT IS IN USE AGAIN AS OF 2026-08-25: the panel was
# rewired and 12 now carries PAUSE / RESUME. The old note is kept because it
# explains why the pin was ever absent, not because it still holds.
#
# The recording controls are two INDEPENDENT switches, not an interlocked
# pair - measured both reading `hi` (open) at rest on 2026-08-15 (then on
# GPIO24/23), whereas an interlocked ON-OFF-ON pair always has one leg down.
# They are decoded together into a session state but each keeps its own
# meaning. Rewired to 22/11 on 2026-08-18 - see REC_PIN below:
#   switch 1, RED   leg, GPIO22 ... START / STOP
#   switch 2, GREEN leg, GPIO11 ... PAUSE / RESUME
#
# REWIRED AGAIN 2026-08-25, operator: START / STOP moved to GPIO11 and
# PAUSE / RESUME to GPIO12. GPIO22 is no longer wired to anything and is dropped
# from this list rather than left claimed - a pin held with a pull-up but read
# by nothing is a trap for whoever probes this panel next.
#
# CONFIRMED ON THE WIRE before the change: 11 was toggling as the panel was
# worked, while 12 sat unclaimed with no consumer at all (gpioinfo), so nothing
# else is fighting for it.
# CORRECTED 2026-08-25 FROM MEASUREMENT, not from the wiring notes. A 20 s
# sampler watched every line while one switch was worked: GPIO22 was the only
# switch pin that moved (2 and 3 also move, but those are the bit-banged I2C
# bus polling the ADC). The operator identified that switch as PAUSE / RESUME,
# so pause is on 22.
#
# GPIO12 is dropped again - it was assigned on 2026-08-25 and never once moved,
# so nothing is wired to it after all.
#
# START / STOP IS STILL UNIDENTIFIED. It is left on GPIO11 below, but 11 has not
# moved in any sample either, so that is a placeholder rather than a measurement
# - recording cannot start until the real pin is found the same way pause was.
#   switch    GPIO22 ... PAUSE / RESUME   (measured)
#   switch    GPIO11 ... START / STOP     (UNCONFIRMED - never seen to move)
SWITCHES = [
    ("BRUSH", 27),
    # SAVE MOVED 25 -> 5, 2026-08-27, operator rewired it. Measured before and
    # after, hands off the panel:
    #
    #     GPIO25   341 edges in 12 s   chattering, unusable
    #     GPIO5      0 edges in 12 s   steady high
    #
    # And with the button worked: 13 presses of 190-460 ms each, clean release
    # between every one, not a single bounce. It reads as a MOMENTARY button,
    # which is what the edge-counting in _read_switches was written for in the
    # first place - see SAVE_PIN.
    #
    # GPIO25 was never faulty itself. It was tied to GPIO9, which is shorted to
    # ground, so it sat between its own pull-up and that short and oscillated.
    # Neither pin is read any more, so the short no longer reaches anything.
    ("SAVE", 5),
    ("START / STOP", 11),
    ("PAUSE / RESUME", 17),
]
# SWAPPED AND PUT BACK, 2026-08-20. Worth keeping the round trip on record,
# because the symptom does not distinguish the two causes on its own.
#
# The complaint was that the lever's EXTEND throw retracted the rod. That has
# two possible causes and they need opposite fixes: either this pair is mapped
# the wrong way round, or the rod's leads are landed backwards. The pair was
# swapped here first, which DID correct the rod - and broke the panel with it,
# because the label then disagreed with the lever. That is the tell: the label
# had been right all along, so the fault was never here. The 2026-08-14 bench
# measurement of this pair stands.
#
# The real inversion is in the rod's wiring and it is corrected at the point it
# actually occurs - see ACT_INVERT in uno_serial.py.
ACT_EXTEND_PIN = 16
ACT_RETRACT_PIN = 19

# Named separately from SWITCHES so the panel and the recorder can reference the
# roles without matching on a display string.
# Moved 13 -> 27 on the operator's order 2026-08-18; nothing is on 13 now.
BRUSH_PIN = 27          # brush motor on / off

# Switches whose OPEN state means ON - the reverse of every other switch here.
# EMPTY, and it belongs empty: every control on this rig is fail-dead active-low
# again, so an unplugged or snapped wire reads OFF.
#
# Kept as history because this was flipped once and it cost us. The brush toggle
# was inverted on 2026-08-17 on the operator's explicit order ("high -> brush on,
# low -> brush off"): as wired then, the throw Harshil used as ON left the pin
# open and the throw used as OFF shorted it to ground, so an active-low decode
# ran the brush in the OFF position. That was measured on the OLD pin (GPIO13)
# and carried over to GPIO27 unverified.
#
# The bill arrived on 2026-08-19. With the toggle unplugged the pull-up floats
# GPIO27 HIGH, and HIGH meant ON, so the panel read BRUSH ON with no switch on
# the rig at all - and a snapped wire would have read as a running brush the
# panel could not stop. Harshil rewired the toggle so its OFF throw reads off,
# which is the fail-safe orientation, so the inversion is gone.
#
# DO NOT RE-ADD IT. If the lever ever reads backwards, move the wire between the
# switch's two outer terminals; that fixes the sense without giving up fail-dead.
OPEN_IS_ON = set()
# Rewired 2026-08-18 on the operator's order: the pair moved 24/23 -> 22/11
# (red kept START/STOP, green kept PAUSE/RESUME - if the strip shows the two
# levers swapped, exchange these two numbers, nothing else references them).
# GPIO11 is SPI SCLK by default; SPI is unused on this rig, plain input here.
REC_PIN = 11            # switch 1 - START / STOP   (was 22 until 2026-08-25)
PAUSE_PIN = 17          # PAUSE / RESUME - moved off shorted GPIO22 2026-08-25
SESSION_PINS = (REC_PIN, PAUSE_PIN)

# The three session states, in the order the operator moves through them.
RECORDING, PAUSED, STOPPED = "RECORDING", "PAUSED", "STOPPED"

ALL_PINS = [p for _, p in SWITCHES] + [ACT_EXTEND_PIN, ACT_RETRACT_PIN]

# PARKED PINS: claimed and pulled UP, never read. Measured 2026-08-25.
#
# GPIO9 is wired in parallel with GPIO25 - the operator landed the save switch on
# both pins while working out which one was usable. GPIO9 is shorted and was
# dropped from SWITCHES, which released it. AN UNCLAIMED PI PIN DEFAULTS TO
# PULL-DOWN, and through that shared wire GPIO9's pull-down beat GPIO25's pull-up
# and clamped the save line low. The save switch then read as permanently
# pressed: no edges in nine minutes of sampling, every confirm window claimed the
# instant it opened, and the switch looked mechanically jammed when nothing was
# wrong with it at all.
#
# Proven rather than guessed - toggling GPIO9's bias moved GPIO25 four times out
# of four:
#     GPIO9 pd -> GPIO25 lo      GPIO9 pu -> GPIO25 hi
#
# So GPIO9 is held here purely so its bias cannot fight GPIO25. It is requested
# with the same pull-up and never read.
#
# THIS REVERSES THE ADVICE IN THE REWIRING NOTE ABOVE, which said an unread pin
# should be released rather than "left claimed - a trap for whoever probes this
# panel next". That is right for an isolated pin and wrong for a paralleled one:
# releasing this pin does not neutralise it, it hands it back to a pull-down that
# is wired straight onto a pin that IS read. Releasing is only safe once the
# GPIO9 wire is physically off the panel.
#
# GPIO22 was checked the same way and does NOT affect GPIO17, so it stays out.
# EMPTY AGAIN. GPIO9 was held here with a pull-up for one reason: it is wired to
# GPIO25 and, released, its default pull-DOWN dragged that line low. SAVE has
# since moved to GPIO5, so GPIO25 is no longer read by anything and GPIO9 has
# nothing left to poison. Both are now unclaimed and harmless where they sit.
#
# Put a pin back in here if a dead line is ever found sharing a wire with a live
# one again - the note above is the whole argument for why that is not the same
# as "release what you do not read".
PARKED_PINS = []


# SHORTED PINS: GPIO9 AND GPIO22 ARE DEAD, MEASURED 2026-08-25.
#
# Both were held LOW with the operator's switches physically REMOVED from the
# panel and the internal pull-up enabled. An unconnected pull-up pin must read
# HIGH, and on the same sweep GPIO11/16/19/27 all did - as did every one of the
# twelve free pins (5,6,12,13,17,18,20,21,23,24,25,26) when pull-ups were forced
# onto them. Only 9 and 22 stayed low, so the short is on those two lines and
# nothing else. Pi power was ruled out at the same time: get_throttled read 0x0,
# i.e. no under-voltage or throttling, not even a historical flag.
#
# PAUSE / RESUME HAS MOVED OFF 22 TO GPIO17, on the operator's instruction. 17
# was measured clean in the same sweep that condemned 22 - high with a pull-up
# and nothing landed on it - and it is claimed by nothing else on this Pi.
#
# SAVE HAS MOVED OFF 9 TO GPIO25 as well, 2026-08-25. The operator landed the
# save switch on BOTH pins at once and a 15 s watch settled it outright: GPIO25
# logged 96 transitions while GPIO9 sat flat at 100% low and logged none. A pin
# already shorted to ground cannot produce an edge, so a switch wired to it is
# invisible however hard it is pressed - which is why counting presses on 9 kept
# returning zero. 25 is also where SAVE lived before 2026-08-18, so this is a
# return rather than a new home.
#
# NEITHER 9 NOR 22 IS READ BY ANYTHING NOW. Both are still shorted and both are
# harmless left that way; they are simply no longer claimed.
#
# _fault_filter STAYS regardless of these moves. It costs nothing on a healthy
# pin, and it is the only reason the shorted pins did not keep painting a
# permanent PAUSE and minting phantom saves before they were tracked down.
#
# What the short costs while it is un-recognised: GPIO22 low means "pause
# closed" for ever, so the strip shows PAUSE lit permanently and, because the
# latching decode reads both levers literally, every run starts in PAUSED and
# records nothing. GPIO9 low with intermittent ~170 ms excursions mints phantom
# SAVE presses, which is where the "SAVED (EMPTY)" clips came from.
#
# A level alone could never have proved this - a latching lever left thrown
# reads identically to a short, which is the trap this panel set twice before.
# What settled it was measuring with the switches OFF the rig.

SAVE_PIN = 5            # moved off the GPIO9-poisoned GPIO25, 2026-08-27
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
        "switch_faults": {},
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
        self._pin_was = {}          # pin -> last debounced level, for edge counting
        self._pin_edges = {}        # pin -> recent edge times (chatter test)
        self._pin_faults = {}       # pin -> reason, for the one-shot log line
        self._stable = {}           # pin -> last believed `closed`
        self._candidate = {}        # pin -> (value, consecutive samples seen)
        self._dead_reads = 0        # consecutive all-None ADC polls
        # Validation state for the analog path - see ADC_AGREE_REJECT.
        self._last_good = {}        # ch -> last raw read that passed every test
        self._jump_runs = {}        # ch -> (pending raw, consecutive agreements)
        self._pot_window = []       # recent good pot reads - see POT_STABLE_BAND
        self._pot_latched = None    # last ACCEPTED pot reading - the setting
        self._pot_pct_shown = None  # last REPORTED percentage - see POT_PCT_HYST
        self._pot_kf_x = None       # Kalman state estimate, in counts
        self._pot_kf_p = 0.0        # ... and its error covariance
        self._pot_zero_since = None # when the current run of cliff-zeros began
        self._axis_hold = {}        # ch -> (last good norm, monotonic) - AXIS_HOLD_S
        self._kadc = None           # kernel-bus ADS1115, preferred over _bus
        self._adc_thread = None
        # Published so main.py can log it: a poll rate that has collapsed is the
        # difference between "the stick feels laggy" and "the stick is laggy", and
        # the reject counters name which test is firing when a phantom shows up.
        self._adc_hz = 0.0
        self._pot_last_good = None  # counts, for the dropout test
        self._pot_med = []          # (t, counts) window for _pot_median
        self._pot_view = []         # (t, pct) window for the DISPLAY value
        self._pot_step_last = None  # last believed counts, for _pot_step
        self._pot_step_pend = None  # (counts, t) a rejected level proving itself
        self._rejects = {"range": 0, "agree": 0, "jump": 0, "read": 0,
                         "noise": 0, "float": 0, "pot_drop": 0,
                         "pot_step": 0}
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
            # PARKED_PINS are requested but never read - see the note there.
            self._req = gpiod.request_lines(
                "/dev/gpiochip0", consumer="ground-station-switches",
                config={p: pu for p in list(ALL_PINS) + list(PARKED_PINS)})
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
                    self._axis_hold.clear()
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
            self._axis_hold.clear()
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
        # Against the stick's MEASURED travel, not the supply rail - see
        # AXIS_TRAVEL_*. PER DIRECTION because the four do not
        # share a real range - see the comment on those constants. ch==0 is X,
        # ch==1 is Y, matching ads_i2c's channel wiring (A0=x, A1=y) and every
        # other per-channel branch in this file.
        #
        # The FULL_SCALE / 2.0 fallback only guards a nonsense centre (a zero
        # or negative learned centre would divide by ~0); it is the old,
        # under-scaled behaviour and should never be the live path.
        # PER DIRECTION - see AXIS_TRAVEL_*. The sign is taken in RAW space,
        # before INVERT_X/INVERT_Y, because these fractions were measured there:
        # "raw above centre" is one physical end of the stick's throw whichever
        # way the display and the motors are later flipped.
        delta = raw - centre
        if ch == 0:
            tilt_frac = AXIS_TRAVEL_X_POS if delta >= 0 else AXIS_TRAVEL_X_NEG
        else:
            tilt_frac = AXIS_TRAVEL_Y_POS if delta >= 0 else AXIS_TRAVEL_Y_NEG
        span = centre * tilt_frac
        if span <= 0:
            span = FULL_SCALE / 2.0
        value = max(-1.0, min(1.0, (raw - centre) / span))
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

    def _pot_step(self, raw):
        """Reject an implausible single-sample step. See POT_STEP_MAX.

        Holds the previous value rather than believing a jump no hand made.
        Normal turning steps far below the limit and passes untouched, so this
        costs nothing in response. A rejected level that keeps repeating for
        POT_STEP_HOLD_S is accepted, so a genuine hard flick is delayed, never
        refused - and that hold outlasts the worst glitch measured here.
        """
        if POT_STEP_MAX <= 0 or raw is None:
            return raw
        limit = POT_STEP_MAX * FULL_SCALE
        prev = self._pot_step_last
        if prev is None:
            self._pot_step_last = raw
            return raw
        if abs(raw - prev) <= limit:
            self._pot_step_last = raw
            self._pot_step_pend = None
            return raw
        now = time.monotonic()
        pend = self._pot_step_pend
        if pend is None or abs(raw - pend[0]) > limit:
            self._pot_step_pend = (raw, now)
            self._rejects["pot_step"] += 1
            return prev
        if now - pend[1] >= POT_STEP_HOLD_S:
            self._pot_step_last = raw
            self._pot_step_pend = None
            return raw
        self._rejects["pot_step"] += 1
        return prev

    def _pot_median(self, raw):
        """Median of the last POT_MEDIAN_S of readings. See POT_MEDIAN_S.

        Rejects the failing wiper's mid-range excursions, which the rail test
        cannot see and which last far too long for any confirmation count to
        reject. Costs ~half a window of lag on a real turn.

        Returns None only while nothing has been read yet; a None reading is not
        added to the window but does not clear it either - a dropped transfer is
        not evidence about where the knob is, and clearing would let the very
        next sample through unfiltered, which is exactly the spike being caught.
        """
        if POT_MEDIAN_S <= 0:
            return raw
        if raw is None:
            return None
        now = time.monotonic()
        self._pot_med.append((now, raw))
        cutoff = now - POT_MEDIAN_S
        while self._pot_med and self._pot_med[0][0] < cutoff:
            self._pot_med.pop(0)
        vals = sorted(v for _, v in self._pot_med)
        n = len(vals)
        if n == 0:
            return raw
        return vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])

    def _pot_view_pct(self, pct):
        """A settled version of `pct` for the panel only. See POT_VIEW_S.

        Never reaches the lamp: uno_motors reads "pct", this fills "pct_view".
        """
        if POT_VIEW_S <= 0 or pct is None:
            return pct
        now = time.monotonic()
        self._pot_view.append((now, pct))
        cutoff = now - POT_VIEW_S
        while self._pot_view and self._pot_view[0][0] < cutoff:
            self._pot_view.pop(0)
        vals = sorted(v for _, v in self._pot_view)
        n = len(vals)
        if not n:
            return pct
        mid = vals[n // 2] if n % 2 else 0.5 * (vals[n // 2 - 1] + vals[n // 2])
        return float(round(mid))

    def _pot_dropout(self, raw):
        """Reject a wiper dropout - a step to the rail no hand could have made.

        See POT_RAIL_FRAC. The test is deliberately about the DESTINATION, not
        the speed: a reading that lands within POT_RAIL_FRAC of full scale
        having moved more than POT_DROPOUT_STEP to get there is the wiper open,
        not the operator, because a hand arriving at 100% is always coming from
        somewhere near it. Turning the knob normally is untouched - the last
        sample before max is in the nineties, so the step is tiny.

        Returns None for a rejected sample. None is already "no reading" all the
        way down this path, and _pot_gate latches the previous setting through
        it, so the lamp holds its real brightness instead of flashing to full
        every time the wiper stutters.

        THIS IS A MITIGATION, NOT A REPAIR. The pot is failing and wants
        replacing; this only stops a failing pot from driving the lamp to full.
        """
        if raw is None:
            return None
        prev = self._pot_last_good
        if (prev is not None
                and raw >= POT_RAIL_FRAC * FULL_SCALE
                and (raw - prev) > POT_DROPOUT_STEP * FULL_SCALE):
            self._rejects["pot_drop"] += 1
            return None
        self._pot_last_good = raw
        return raw

    def _pot_kalman(self, z):
        """Scalar Kalman filter over the pot's counts. See POT_KALMAN_R.

        Returns the filtered estimate, or None for a failed read - a None is
        passed straight through rather than predicted over, because everything
        downstream already treats None as "this reading never happened", and
        inventing a value for a dead bus is exactly the failure _pot_gate and
        the dead-read counter exist to catch.
        """
        if z is None or not POT_KALMAN:
            return z
        z = float(z)
        if self._pot_kf_x is None:
            self._pot_kf_x, self._pot_kf_p = z, POT_KALMAN_R
            return self._pot_kf_x
        innovation = z - self._pot_kf_x
        if abs(innovation) > POT_KALMAN_SNAP * FULL_SCALE:
            # A hand on the knob. Reinitialise rather than converge - see the
            # POT_KALMAN_SNAP note.
            self._pot_kf_x, self._pot_kf_p = z, POT_KALMAN_R
            return self._pot_kf_x
        # Predict (random walk: the estimate itself does not move), then update.
        pred_p = self._pot_kf_p + POT_KALMAN_Q
        gain = pred_p / (pred_p + POT_KALMAN_R)
        self._pot_kf_x += gain * innovation
        self._pot_kf_p = (1.0 - gain) * pred_p
        return self._pot_kf_x

    def _pot_pct(self, raw):
        """Latched counts -> a STEADY integer percent, or None.

        See POT_PCT_HYST for why this exists. Returns the SAME number until the
        true reading has moved at least POT_PCT_HYST away from it, so a knob
        nobody is touching reports one value instead of oscillating between two
        neighbouring integers forever.

        Rounded to a whole percent on purpose: the knob has ~100 usable
        positions, the panel prints the number with no decimals, and the lamp
        PWM it feeds has 256 steps - a fractional percent was precision that
        nothing downstream could use and that only ever showed up as jitter.
        """
        if raw is None:
            return None
        # Under one ADC code is not a brightness, it is a rounding error the
        # gamma curve magnifies into 0-3%. See POT_SUBLSB. Applied HERE and not
        # in the chain above so "raw" and "volts" keep reporting what the ADC
        # actually returned - the diagnostics stay honest, only the lamp and the
        # readout are floored.
        if POT_SUBLSB and raw < POT_LSB:
            raw = 0.0
        target = max(0.0, min(100.0, 100.0 * raw / FULL_SCALE))
        # Straightens the log taper before the hysteresis below, so the whole
        # numbers the panel latches onto are evenly spaced across the travel.
        target = _linearise_pot(target)
        if self._pot_pct_shown is None:
            # First ever reading is adopted whole - there is nothing to step from,
            # and walking up from 0 would light the lamp slowly on every start.
            self._pot_pct_shown = float(round(target))
        elif abs(target - self._pot_pct_shown) >= POT_PCT_HYST:
            want = float(round(target))
            if POT_MAX_STEP > 0:
                delta = want - self._pot_pct_shown
                if abs(delta) > POT_MAX_STEP:
                    # Walk, do not jump - see POT_MAX_STEP.
                    want = self._pot_pct_shown + (
                        POT_MAX_STEP if delta > 0 else -POT_MAX_STEP)
            self._pot_pct_shown = want
        return self._pot_pct_shown

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

    def _hold_axis(self, ch, value, now_m):
        """The AXIS_HOLD_S bridge: last good value through a brief gap."""
        if value is not None:
            self._axis_hold[ch] = (value, now_m)
            return value
        prev = self._axis_hold.get(ch)
        if prev is not None and now_m - prev[1] < AXIS_HOLD_S:
            return prev[0]
        return None

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

    def _fault_filter(self, closed):
        """Force a FAULTED switch line to OFF, and record why.

        ONE fault is tested for. A stuck-closed test used to sit beside it and
        was removed the same day - the note in the body says why.

          CHATTERING - more than CHATTER_EDGES transitions inside
            CHATTER_WINDOW_S. The measured noise ran ~11 edges/second forever;
            a hand cannot.

        A faulted line reports OFF, NOT None. None means "no reading" and stops
        SessionDecode returning any state at all, which would take recording
        down along with the broken pin - the exact opposite of what a fault
        filter is for. OFF is also the fail-dead direction every switch on this
        rig already uses, so a dead line looks like an untouched one.

        NEITHER FAULT LATCHES. Both are recomputed every poll, so repairing the
        wiring brings the switch back with no restart: the stuck test clears the
        instant the pin reads open once, and the chatter test clears as soon as
        the edges stop arriving.
        """
        now = time.monotonic()
        faults = {}
        out = {}
        for pin, is_closed in closed.items():
            if is_closed is None:
                out[pin] = None
                continue

            was = self._pin_was.get(pin)
            if was is not None and is_closed != was:
                self._pin_edges.setdefault(pin, []).append(now)
            self._pin_was[pin] = is_closed

            edges = [t for t in self._pin_edges.get(pin, ())
                     if now - t <= CHATTER_WINDOW_S]
            self._pin_edges[pin] = edges

            # STUCK-CLOSED TEST REMOVED 2026-08-25, same day it was added.
            #
            # It existed to catch GPIO9 and GPIO22, which were shorted to ground
            # and so read "pressed" for ever. Both roles have since moved to
            # healthy pins (SAVE 9->25, PAUSE 22->17), and neither dead pin is
            # claimed any more, so the fault it was built for cannot occur.
            #
            # What it did instead was misfire. It was scoped to SAVE and PAUSE
            # on the reasoning that a permanently closed contact is pathological
            # there, with BRUSH and START/STOP exempt as latching levers someone
            # may leave thrown. Then a LATCHING save switch was fitted, and a
            # thrown latch is indistinguishable from the short - GPIO25 sat low
            # and legitimately closed, was called stuck after 20 s, and the save
            # button stopped working with nothing wrong with it.
            #
            # That is the same trap this panel has set twice before: A LEVEL
            # ALONE CANNOT TELL A SHORT FROM A CLOSED SWITCH. The earlier
            # diagnosis only worked because the switches were physically off the
            # rig at the time. Nothing at runtime has that luxury, so runtime
            # must not try to guess.
            #
            # The chatter test stays: no operator produces CHATTER_EDGES inside
            # CHATTER_WINDOW_S, so it has no legitimate case to misread.
            if len(edges) >= CHATTER_EDGES:
                faults[pin] = "chattering"

            out[pin] = False if pin in faults else is_closed

        for pin, why in faults.items():
            if self._pin_faults.get(pin) != why:
                print("inputs: GPIO%d FAULTED (%s) - reported OFF" % (pin, why))
        for pin in self._pin_faults:
            if pin not in faults:
                print("inputs: GPIO%d recovered" % pin)
        self._pin_faults = faults
        return out

    def _debounce(self, raw):
        """Accept a change only after DEBOUNCE_SAMPLES agreeing reads.

        The first sample seeds every pin, so start-up is immediate rather than
        showing unknown switches for the first 100 ms.
        """
        if not self._stable:
            self._stable = dict(raw)
            return dict(raw)
        # Samples needed per pin. Everything keeps the common floor; SAVE gets
        # its own because its line does not settle - see SAVE_DEBOUNCE_MS.
        save_runs = DEBOUNCE_SAMPLES
        if SAVE_DEBOUNCE_MS > 0:
            save_runs = max(DEBOUNCE_SAMPLES,
                            int(round(SAVE_DEBOUNCE_MS / 1000.0 * POLL_HZ)))
        for pin, value in raw.items():
            need = save_runs if pin == SAVE_PIN else DEBOUNCE_SAMPLES
            if value == self._stable[pin]:
                # Back to the believed level before it was ever accepted - the
                # blip is over, so forget it rather than counting it toward the
                # next change in the same direction.
                self._candidate.pop(pin, None)
                continue
            prev, runs = self._candidate.get(pin, (value, 0))
            runs = runs + 1 if prev == value else 1
            if runs >= need:
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
        # A shorted or chattering line is not a switch - see _fault_filter.
        closed = self._fault_filter(closed)
        # Published as "is this switch ON", not "is this pin closed" - the two
        # differ exactly for OPEN_IS_ON, where the operator's ON throw leaves
        # the pin open. Inverted here, after the debounce, so the debouncer
        # keeps reasoning about raw electrical levels.
        state["switches"] = {
            name: (not closed[pin]) if pin in OPEN_IS_ON else closed[pin]
            for name, pin in SWITCHES}
        state["switch_faults"] = dict(self._pin_faults)

        ext, ret = closed[ACT_EXTEND_PIN], closed[ACT_RETRACT_PIN]
        # The actuator switch is mechanically interlocked (ON-OFF-ON), so both
        # closed should be impossible â€” surfaced as FAULT rather than assumed
        # away, since a broken interlock is exactly what you want to see.
        state["actuator"] = ("FAULT" if ext and ret else
                             "EXTEND" if ext else
                             "RETRACT" if ret else "STOP")
        # Back to raw levels for display: closed == shorted to GND == reads 0.
        # Measured on the rig 2026-08-14 and re-confirmed 2026-08-20, the three
        # stages are 16=0/19=1 EXTEND, 16=1/19=1 STOP, 16=1/19=0 RETRACT.
        state["act_pins"] = {ACT_EXTEND_PIN: 0 if ext else 1,
                             ACT_RETRACT_PIN: 0 if ret else 1}

        # TEMPORARY session-chain capture, 2026-08-25 - remove with this comment.
        _st = self._session.update(closed[REC_PIN], closed[PAUSE_PIN])
        try:
            _prev = getattr(self, "_dbg_prev", None)
            _now = (closed[REC_PIN], closed[PAUSE_PIN], closed[SAVE_PIN], _st)
            if _now != _prev:
                import time as _t
                with open("/tmp/session_debug.log", "a") as _f:
                    _f.write("%.3f rec=%s pause=%s save=%s -> %s\n" % (
                        _t.time(), closed[REC_PIN], closed[PAUSE_PIN],
                        closed[SAVE_PIN], _st))
                self._dbg_prev = _now
        except Exception:
            pass
        state["session"] = _st
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
            analog["pot"] = {"pct": None, "pct_view": None, "raw": None, "volts": None}
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
        # ALL THREE POT FILTERS BACK IN, operator's instruction 2026-08-25.
        #
        # They were taken out earlier the same day ("remove kalman filter on
        # light and also remove delay", then the dropout test too) and put back
        # once the unfiltered knob was seen in use. The round trip is kept on
        # record because it is the argument for the latency, made from the rig
        # rather than from theory: without these the wiper's faults reach the
        # lamp directly.
        #
        # Order matters and is not arbitrary:
        #   _pot_dropout - first, so an open-circuit teleport to the rail never
        #                  enters the median window at all. Costs nothing: one
        #                  comparison, no window.
        #   _pot_median  - 0.9 s, sized by replaying 9640 logged samples. The
        #                  first window that removes every >=20-point jump.
        #                  Costs ~450 ms of lag on a real turn, measured 438 ms.
        #   _pot_kalman  - last, smoothing what survives down to the +/-1 count
        #                  flicker the lamp would otherwise show.
        #
        # Each is independently disabled by its own env var, so the next round
        # of this does not need a code edit: INPUTS_POT_MEDIAN_S=0 drops the
        # median and its lag, and the other two are one wrap each in this line.
        #
        # ALL OF IT IS A MITIGATION. The wiper is failing - 18 mid-range
        # excursions in 400 s, median 259 ms, plus rail teleports - and a new
        # potentiometer removes the need for every line of it.
        # INSTANT. Operator 2026-08-25: "light send potentiometer data instant not
        # delay". Every stage that costs time is out of this chain, and their
        # defaults are set so nothing re-enables them quietly:
        #
        #   POT_MAX_STEP=0        was 1/sample = 1.72 s to cross the range
        #   POT_MEDIAN_S=0        was 0.10 s window
        #   POT_STABLE_SAMPLES=1  was 3 agreeing samples on small trims
        #   _pot_kalman removed   ~69 ms   <- BACK ON, see below
        #   _pot_step removed     held an implausible jump for up to 0.6 s
        #
        # What is left is one ADC sample, ~17 ms, which is the floor the hardware
        # sets. _pot_dropout STAYS because it is a single comparison with no
        # window - it cannot add delay, and it is the only thing keeping the
        # wiper's open-circuit teleports from slamming the lamp to full.
        #
        # KALMAN BACK ON, operator 2026-08-25, after the INSTANT note above.
        # It is the ONLY stage re-enabled - the median, the step test and the
        # stability count all stay off - so the chain is dropout -> kalman ->
        # gate and the cost is the ~69 ms in the table, not the ~450 ms the
        # median wanted. What it buys is the +/-1 count flicker off the lamp on
        # a settled knob: the measured noise is 10 counts and a percentage
        # sitting on a rounding boundary flips on far less than that.
        #
        # A deliberate turn does NOT pay the 69 ms. Anything past
        # POT_KALMAN_SNAP (4% of full scale) reinitialises the filter on the
        # spot instead of converging toward it, so a hand on the knob is still
        # one poll from lamp - only a knob left alone is averaged.
        #
        # INPUTS_POT_KALMAN=0 takes it back out with no code edit.
        #
        # THE GLITCHES COME BACK. All of them, at full size. Every removed stage
        # was suppressing a real measured fault, not a hypothetical one, and the
        # pot is still failing. Each is one env var away if any of it becomes the
        # worse annoyance - the notes above each constant say what it cost and
        # what it bought.
        pot_raw = self._pot_gate(self._pot_kalman(self._pot_dropout(gated[2])))

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
        # The stick's pots are landed on the opposite ADC inputs to what the
        # channel numbering assumes - see SWAP_XY. Swapped HERE, after
        # _norm_axis has applied each channel's own measured travel and before
        # the inversions below, so the calibration stays with the wire and the
        # INVERT_* flags stay meaningful to the operator.
        #
        # The RAW pair is swapped with it. Nothing downstream computes from
        # these, but the panel dot and motor_cam.log both print them, and a log
        # whose x_raw belongs to the y axis is worse than no log at all.
        if SWAP_XY:
            x_norm, y_norm = y_norm, x_norm
            x_raw, y_raw = y_raw, x_raw
        # Orientation applied AFTER normalisation, so the centre learner and
        # every validation gate reason about raw electrical travel and only
        # the demand the motors see is flipped. See INVERT_X / INVERT_Y.
        if x_norm is not None and INVERT_X:
            x_norm = -x_norm
        if y_norm is not None and INVERT_Y:
            y_norm = -y_norm
        # Bridge single-sample rejections with the last validated value, so
        # the slew limiter is not slammed to zero by a 30 ms gap - see
        # AXIS_HOLD_S for the measured failure this removes.
        now_m = time.monotonic()
        x_norm = self._hold_axis(0, x_norm, now_m)
        y_norm = self._hold_axis(1, y_norm, now_m)
        analog["joy"] = {
            "x": x_norm, "y": y_norm,
            "x_raw": x_raw, "y_raw": y_raw,
        }
        _pct = self._pot_pct(pot_raw) if pot_raw is not None else None
        analog["pot"] = (
            {"pct": _pct, "pct_view": self._pot_view_pct(_pct),
             # INTEGER COUNTS, and this is not cosmetic. _pot_kalman publishes a
             # FRACTIONAL estimate - that is the whole point of it - and _pct
             # above keeps using that full precision. But "raw" is an ADC count
             # and every consumer formats it as one: main.py's correlation log
             # uses "{v:5d}", which raises ValueError on a float. That
             # exception was swallowed by log_correlation's own "never allowed
             # to raise" try/except, so motor_cam.log simply STOPPED on
             # 2026-08-25 20:40 - the same minute the Kalman went live - and
             # nothing said why. Round here, at the producer, so the published
             # contract matches what it has always been.
             "raw": int(round(pot_raw)), "volts": pot_raw / COUNTS_PER_VOLT}
            if pot_raw is not None else
            {"pct": None, "pct_view": None, "raw": None, "volts": None})
        # TEMPORARY joystick capture, 2026-08-25 - remove with this comment.
        try:
            import time as _t
            with open("/tmp/joy_health.log", "a") as _f:
                _f.write("%.3f x=%s y=%s xr=%s yr=%s pot=%s praw=%s hz=%.1f rej=%s\n" % (
                    _t.time(), x_norm, y_norm, x_raw, y_raw,
                    (analog.get("pot") or {}).get("pct"),
                    (analog.get("pot") or {}).get("raw"),
                    self._adc_hz, dict(self._rejects)))
        except Exception:
            pass
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

