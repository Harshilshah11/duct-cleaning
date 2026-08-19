#pragma once
#include <Arduino.h>

/*
 * Config.h — every pin and every tuning number for uno_eth_link, in one place.
 *
 * THIS FILE IS THE HARD-WON PART. The classes around it are just structure; the
 * numbers here were each paid for with a debugging session, so the reasoning
 * stays attached to the value it justifies. Read the comment before changing
 * the constant.
 */

// ---------------------------------------------------------------------------
// SERIAL — telemetry only on this build
// ---------------------------------------------------------------------------
// Commands arrive over UDP, NOT over this port. Raising the baud does not make
// the control link faster; it makes the LOG faster, and that matters for one
// specific reason: loop() hand-rolls the soft-PWM for the rod and the brush on
// a 4 ms period, and a Serial.print that outruns the 64-byte TX buffer BLOCKS
// until the UART drains. At 9600 a ~55-byte telemetry line is ~57 ms of wire
// time; at 250000 it is ~2.2 ms. The print stops being able to stall a PWM edge.
//
// 250000 IS CHOSEN FOR ACCURACY, NOT JUST SPEED. On a 16 MHz AVR with U2X the
// divisor is UBRR = F_CPU/(8*baud) - 1, and only some rates land on an integer:
//
//     9600    -> UBRR 207     +0.2% error
//     115200  -> UBRR  16     +2.1% error
//     230400  -> UBRR   7.68  -3.5% error   <- NOT integer, out of spec, avoid
//     250000  -> UBRR   7      0.0% error   <- exact
//     500000  -> UBRR   3      0.0% error   <- exact
//     1000000 -> UBRR   1      0.0% error   <- exact, but no margin left
//
// Match the serial monitor to this or the log reads as garbage.
const unsigned long SERIAL_BAUD = 250000;

// ---------------------------------------------------------------------------
// NETWORK — why .50.20 and not .1.20
// ---------------------------------------------------------------------------
// This board must NOT be 192.168.1.20. That is the ground station Pi's own
// wlan0 address, and a host always delivers traffic for its own address
// locally, so every packet the Pi sent to the Uno would be answered by the Pi
// itself and never reach the wire. The eth0 segment is 192.168.50.0/24, so the
// Uno lives at 192.168.50.20 and ground_station/uno_link.py targets that.
//
// The MAC is locally administered (the 0x02 bit in the first byte) and must be
// unique on your LAN. Newer shields ship with a real MAC on a sticker — use
// that if yours has one.
const uint16_t NET_LISTEN_PORT = 5005;

// ---------------------------------------------------------------------------
// MOTOR PIN MAP — chosen around the shield, NOT freely
// ---------------------------------------------------------------------------
//
//   channel 1 / LEFT    DIR1 = D9        PWM1 = D3    (Timer2)
//   channel 2 / RIGHT   DIR2 = D8        PWM2 = D6    (Timer0)
//   linear actuator     ACT_DIR = D7     ACT_PWM = D4  (LOW extends! see below)
//   brush motor         BRUSH_DIR = D2   BRUSH_PWM = A1
//   light               LIGHT_DIR = A0   LIGHT_PWM = D5 (Timer0)
//
// EVERY PIN IS NOW SPOKEN FOR. D0/D1 are the USB serial this telemetry goes out
// on, D10-D13 belong to the shield, and all four of the Uno's usable PWM pins
// (D3, D5, D6, D9) are allocated — D9 to a direction line, which is the one
// place a PWM pin is still spendable if something else ever needs dimming.
// A1 is the brush's speed line; A2-A5 are spare plain digital I/O since the
// rod's enable moved off A2 to D4.
//
// REWIRED 2026-08-14: D7 is freed and the old DIR1=D7 / PWM1=D9 / PWM2=D3 map
// is gone. DIR1 took over D9 (a direction line only needs digitalWrite, so
// spending a PWM-capable pin on it is fine), PWM1 moved to D3, PWM2 to D6.
//
// The obvious map (PWM on D10/D11) is IMPOSSIBLE with this shield fitted: the
// W5100/W5500 owns D10 (chip select), D11 (MOSI), D12 (MISO) and D13 (SCK) for
// SPI, plus D4 for the microSD slot. Driving motors from D10/D11 breaks the
// Ethernet link and the motors together. D3/D6/D8/D9 clear all of those.
//
// D6 IS ON TIMER0, AND THAT HAS ONE REAL CONSEQUENCE. Timer0 also generates
// millis(), which the failsafe is timed off. analogWrite() on D6 does NOT
// disturb millis() — only changing Timer0's prescaler would. What it DOES do is
// make a 0 duty cycle unreliable: on Timer0 pins analogWrite(pin, 0) can still
// emit a narrow pulse every period, enough to leave a motor creeping. So every
// full stop on a PWM pin goes through digitalWrite(pin, LOW), never
// analogWrite(pin, 0). MotorChannel and PanelLight both encode that rule; it is
// deliberate, and removing it reintroduces a robot that will not quite stop.
//
// The two channels run on different timers and therefore different default PWM
// frequencies: D3 ~490 Hz (Timer2), D6 ~980 Hz (Timer0). Both drive the bridge
// fine, but the channels are NOT interchangeable — if you ever raise one
// timer's frequency, raise BOTH. Mismatched channels respond differently to the
// same demand, which reads as a mechanical fault.
const uint8_t PIN_DIR1 = 9;    // channel 1 direction (LEFT)
const uint8_t PIN_PWM1 = 3;    // channel 1 speed, Timer2  (~490 Hz)
const uint8_t PIN_DIR2 = 8;    // channel 2 direction (RIGHT)
const uint8_t PIN_PWM2 = 6;    // channel 2 speed, Timer0  (~980 Hz, see above)

// Flip either of these if a wheel spins the wrong way. Doing it here is much
// cheaper than re-soldering, and far safer than negating on the Pi — the Pi
// feeds BOTH this link and uno_serial.py, so a sign flip there would silently
// desync the two transports.
const bool INVERT_1 = false;
const bool INVERT_2 = false;

// ---------------------------------------------------------------------------
// LINEAR ACTUATOR — D4 IS THE SHIELD'S microSD CHIP SELECT
// ---------------------------------------------------------------------------
// Direction comes from the ground station's 3-position actuator switch
// (GPIO16/19 on the Pi), decoded there and arriving as one signed number:
// positive extends, negative retracts, zero STOPS.
//
// MEASURED ON THE RIG, and this is the authority — these are the levels the
// driver actually wants, not an inference from how the wheels work:
//
//     level 1 / EXTEND    D7 = LOW    D4 = HIGH
//     middle  / STOP      D7 = held   D4 = LOW    -> rod holds position
//     level 3 / RETRACT   D7 = HIGH   D4 = HIGH
//
// EXTEND IS DIR **LOW**. Every other direction line on this board (the wheels,
// the brush) treats HIGH as forward, so the natural assumption is wrong here and
// MotorChannel's convention must NOT be copied onto this channel. That is why
// LinearActuator is its own class rather than another MotorChannel.
//
// D4 IS THE ONLY THING THAT GATES THE ROD. D7 selects a direction but does not
// start or stop anything, so STOP is D4 LOW with D7 left wherever it was.
//
// RUN THIS BOARD WITH THE microSD SLOT EMPTY:
//
//   1. WITH A CARD IN THE SLOT THIS LINK WILL DIE. CS is active-LOW, so the
//      rod's STOP level (D4 LOW) SELECTS the card — and stopped is what the rod
//      is most of the time. A selected card drives MISO during the SPI reads
//      this sketch makes to the W5100 on every pass of loop(), corrupting them.
//      The symptom is an Ethernet link that works while the rod is moving and
//      dies when it stops, which reads as a power or wiring fault and is
//      neither. There is no software fix: one pin cannot be both a chip select
//      and a motor gate. Empty slot, or move this line to A2-A5.
//
//   2. setup() USED TO PARK D4 HIGH to deselect that card, AND THAT WAS THE BUG
//      THAT KEPT THE ROD RUNNING. Together with D7 being held HIGH as the
//      brush's direction line, the rod saw "retract, at full scale" from the
//      moment the Uno left reset, forever, whatever the panel switch said. Two
//      pins that nothing thought of as the actuator's added up to a permanent
//      retract command. There is no SD-deselect anywhere in this build.
//
// If a card ever has to go in that slot, A2 is free and is a drop-in
// replacement — change the one constant below and nothing else, because this
// channel is driven by digitalWrite/soft-PWM and needs no timer.
const uint8_t PIN_ACT_DIR = 7;   // "Dir" on the rig — LOW extends, HIGH retracts
const uint8_t PIN_ACT_PWM = 4;   // "Pwm" on the rig — the only line that gates it

// Spelled out because this channel is the ODD ONE OUT: LOW is forward here and
// HIGH is forward everywhere else on the board. Swap these two if the rod
// travels the wrong way — a one-line change that needs no other edit.
const uint8_t ACT_LEVEL_EXTEND  = LOW;
const uint8_t ACT_LEVEL_RETRACT = HIGH;

// The panel's 3-position switch selects these stages.
// RETRACT RAISED 128 -> 255 on 2026-08-14. At 128 the rod moved on the extend
// throw and did nothing at all on the retract throw — the signature of a stage
// that cannot break away rather than one wired wrong: 50% duty is about where a
// loaded actuator stalls, and lifting against gravity runs out of torque first.
// Put it back to 128 for a slow stage, but only once the rod is known to travel
// BOTH ways at full scale.
const uint8_t ACT_DUTY_STOP    = 0;
const uint8_t ACT_DUTY_RETRACT = 255;
const uint8_t ACT_DUTY_EXTEND  = 255;

// Flip if EXTEND on the panel drives the rod the wrong way. Equivalent to
// swapping ACT_LEVEL_EXTEND/ACT_LEVEL_RETRACT; use whichever reads better, but
// do not do both — they cancel.
const bool INVERT_ACT = false;

// ---------------------------------------------------------------------------
// SOFT-PWM — shared by the rod (D4) and the brush (A1)
// ---------------------------------------------------------------------------
// Neither pin has a timer behind it, so any duty between the endpoints is
// synthesised in software by SoftPwmPin::service(), called every pass of loop().
// 0 and 255 resolve to a plain static level and cost nothing; only the middle
// range is chopped, where a stalled loop costs a slower actuator, never a
// runaway one.
//
// 250 Hz. Slow enough that the main loop — which spends most of its time in an
// SPI read of the W5100 — can hit each edge closely enough, fast enough that the
// mechanism sees a smooth average. Both a linear actuator and a brush motor are
// mechanically far too slow to care about ripple at this rate, which is why one
// period serves both: two independent soft-PWM periods would be two numbers to
// keep in sync for no benefit.
const unsigned long SOFT_PWM_PERIOD_US = 4000;

// ---------------------------------------------------------------------------
// BRUSH MOTOR
// ---------------------------------------------------------------------------
// Driven from the panel's TOGGLE switch (Pi GPIO13).
//
// THIS WAS ONE PIN AND THAT IS WHY IT DID NOT WORK. The brush hangs off a
// dual-channel driver channel, exactly like the wheels, so it needs BOTH inputs:
// a direction line AND a speed line. Driving D7 alone set the direction of a
// channel whose PWM input was never asserted, so the bridge stayed off and the
// motor never turned — while telemetry cheerfully reported BRUSH=ON, because the
// sketch really was driving the one pin it knew about.
//
// MOVED D7 -> D2 ON 2026-08-15, AND THIS WAS NOT A TIDY-UP. D7 is the linear
// actuator's direction line on the rig, and this sketch was holding it HIGH
// permanently as the brush's direction. HIGH on that pin means RETRACT — see the
// actuator block above for the other half of that bug.
//
// D2 IS CONFIRMED AGAINST THE RIG (Harshil, 2026-08-15): brush is Dir -> D2,
// Pwm -> A1.
const uint8_t PIN_BRUSH_DIR = 2;
const uint8_t PIN_BRUSH_PWM = A1;

// The brush spins one way only, so its direction is a constant rather than a
// demand. Flip this if the brush runs backwards.
const uint8_t BRUSH_DIR_LEVEL = HIGH;

// Many driver and relay inputs are ACTIVE-LOW — the channel enables when the pin
// goes low, and such a board will run the brush the whole time the Uno is in
// reset if this is wrong. Set false for those. Applies to PIN_BRUSH_PWM, the
// line that actually gates the motor, including the soft-PWM's on-phase.
const bool BRUSH_ACTIVE_HIGH = true;

// The smallest duty that actually TURNS the brush rather than buzzing it, the
// same physics as MIN_DUTY on the wheels. Any non-zero demand is stretched onto
// BRUSH_MIN_DUTY..255. Set to 0 for a raw linear map.
const uint8_t BRUSH_MIN_DUTY = 90;

// ---------------------------------------------------------------------------
// PANEL LIGHT
// ---------------------------------------------------------------------------
// Brightness follows the panel potentiometer (ADS1115 A2 on the Pi), scaled to
// 0..255 there and applied here. The pot was free to take this job precisely
// because the actuator lost its PWM pin, and with it any use for a speed demand
// — the two changes are one change.
//
// A0 is an ANALOG pin driven as a plain digital output, which is legal on the
// Uno (A0 == D14) and is what makes this fit at all: every real digital pin is
// spoken for. It carries the driver channel's direction line, which a lamp does
// not actually need.
const uint8_t PIN_LIGHT_DIR = A0;
const uint8_t PIN_LIGHT_PWM = 5;   // Timer0 (~980 Hz) — same 0-duty caveat as D6

// ---------------------------------------------------------------------------
// STATUS LED
// ---------------------------------------------------------------------------
// LED_BUILTIN = D13, which is also the SPI clock the shield uses. With the
// shield fitted this lamp tracks Ethernet traffic rather than link state, and is
// NOT a reliable indicator. Harmless, but do not read anything into it — and
// never put a real signal on D13.
const uint8_t PIN_STATUS_LED = LED_BUILTIN;

// ---------------------------------------------------------------------------
// FAILSAFE
// ---------------------------------------------------------------------------
// 300 ms: long enough to ride out a handful of dropped datagrams at the 50 Hz
// command rate (15 in a row), short enough that the robot stops within a third
// of a second of a real tether failure. TEST IT WITH THE WHEELS OFF THE GROUND.
const unsigned long FAILSAFE_MS = 300;

// ---------------------------------------------------------------------------
// DRIVE TUNING
// ---------------------------------------------------------------------------
// Below DEADBAND the motor buzzes and heats without turning, so treat it as zero.
//
// LOWERED 12 -> 4 ON 2026-08-19, AND THE REASON IS THAT THIS BAND STACKS.
//
// inputs.py already applies INPUTS_AXIS_DEADBAND around the stick centre AND
// RESCALES the travel past it, so its output leaves zero continuously. This
// band is then applied to that already-rescaled number and does NOT rescale, so
// it re-clips the bottom of the range - the exact double-deadband failure that
// UNO_DEADZONE in uno_serial.py was zeroed to cure, with the second copy simply
// living over here instead.
//
// The arithmetic at the old values: a wheel needs PWM >= 12, i.e. normalised
// >= 12/255 = 0.047, which needs stick deflection >= 0.08 + 0.047 * 0.92 =
// 0.1233 of half travel. The operator felt a dead patch of ~12%, not the 8%
// inputs.py advertises.
//
// MEASURED on 7433 logged frames: the 5th percentile deflection on frames where
// the wheels were actually commanded was 1056 counts, against 1085 predicted by
// that formula (704 if this band did not exist). With 0.04 on the Pi and 4 here
// the threshold falls to ~485 counts, while the stick at rest was measured
// staying inside 240 counts at the 95th percentile - so roughly 2x margin
// against creep, which is what this constant is actually for.
//
// KEEP IT SMALL. This band exists to reject NOISE from any sender, not to be
// the stick's deadband - that one belongs to inputs.py, where the centre is
// learned and the rescale lives. One owner, one number.
//
// Do NOT set it to 0: MIN_DUTY below stretches every surviving non-zero demand
// up to 90, so a stray demand of 1 would spin a wheel at 35% duty.
const uint8_t DEADBAND = 4;
const uint8_t MAX_PWM  = 255;

// MIN_DUTY is the smallest duty that actually TURNS a loaded wheel. Between
// DEADBAND and roughly a third of full duty these gearmotors only buzz: the
// bridge is switching, but the average voltage never breaks static friction, so
// a half-deflected stick makes heat instead of motion.
//
// Any surviving non-zero demand is therefore stretched onto MIN_DUTY..MAX_PWM
// instead of 1..MAX_PWM, so the first millimetre of stick travel already drives.
// Done on the Uno rather than the Pi so it covers EVERY sender — the UDP link,
// uno_serial.py and any bench tool alike — and so two of them cannot apply it
// twice.
//
// Zero stays exactly zero. This raises the smallest MOVING demand; it must never
// turn a stop into a crawl. DEADBAND decides what counts as a stop and must stay
// BELOW this value.
//
// LOWERED 90 -> 0 ON 2026-08-19, on the operator's instruction: the throttle
// must build from slow to fast, not jump straight to 90 the instant the stick
// leaves the deadband.
//
// WHAT THIS CHANGES. The map is now plain linear: PWM == demand. With the Pi's
// 30% axis deadband, stick 30% gives PWM ~0 and stick 100% gives PWM 255,
// rising smoothly in between. That is the acceleration feel that was asked for,
// and it removes a 0 -> 91 current STEP that was being fed into a driver rail
// already known to brown out.
//
// WHAT IT COSTS, AND READ THIS BEFORE DECIDING IT IS BROKEN. The floor was not
// arbitrary: below roughly a third of full duty these gearmotors only BUZZ -
// the bridge switches, but the average voltage never breaks static friction, so
// the wheels make heat instead of motion. With the floor gone, PWM 90 is now
// only reached at about 55% of stick travel:
//
//     stick 30%  -> PWM   0      stick 55%  -> PWM  90   <- likely break-away
//     stick 40%  -> PWM  36      stick 70%  -> PWM 145
//     stick 50%  -> PWM  73      stick 100% -> PWM 255
//
// So if 90 really is this drivetrain's break-away duty, nothing will turn until
// ~55% stick and the 30-55% band will buzz. THAT IS A MEASUREMENT, NOT A
// SETTING: put the robot on the ground, loaded, push the stick up slowly and
// note the PWM in the telemetry line (L=/R=) at the instant the wheels start to
// turn. Set MIN_DUTY to a little under that number.
//
// Anything above 0 reintroduces a step of exactly that size at the deadband
// edge - that is unavoidable, it is what a floor IS. The trade is: a lower
// floor gives a gentler start and a wider buzzing band; a higher floor gives a
// harder start and no buzz. 0 is the extreme end of that trade and was chosen
// deliberately here.
//
// DEADBAND decides what counts as a stop and is independent of this; a stop
// stays exactly zero either way.
const uint8_t MIN_DUTY = 0;

// ---------------------------------------------------------------------------
// RECEIVE BUFFER
// ---------------------------------------------------------------------------
// NOT UDP_TX_PACKET_MAX_SIZE. That constant is 24 bytes in the stock library and
// silently truncates anything longer, which looks exactly like a garbled link.
// 96 bytes is affordable out of the Uno's 2 KB of SRAM; keep the Pi side's
// MAX_PAYLOAD below it.
const uint16_t RX_BUFFER = 96;

// Telemetry pacing: report on change no faster than this, and on a heartbeat
// regardless, so a resting link still proves itself.
const unsigned long TELEMETRY_MIN_INTERVAL_MS = 200;
const unsigned long TELEMETRY_HEARTBEAT_MS    = 3000;

// Loop pause. Was 5 ms on the operator's order 2026-08-18; LOWERED TO 1 ms on
// 2026-08-19 when the operator asked for the stick to feel instant.
//
// TWO THINGS THIS COSTS AT 5 ms, both fixed by dropping it:
//
//   1. LATENCY. The loop tops out near 200 Hz, so a command can wait up to 5 ms
//      in the W5x00 before it is even read, plus up to 5 ms more before the
//      next pass acts on it. At 1 ms that is ~2 ms total. Free responsiveness:
//      it changes no demand and draws no extra current, unlike the ramp rate on
//      the Pi (MOTOR_SLEW_PER_S), which is the other 85% of the latency.
//
//   2. SOFT-PWM RESOLUTION, which was actually BROKEN. SOFT_PWM_PERIOD_US is
//      4000 us = 4 ms, so a 5 ms loop called service() LESS THAN ONCE PER
//      PERIOD - the mid-range duty was aliased to nonsense. It never bit only
//      because the rod and the brush are both driven at 0 or 255 today, where
//      SoftPwmPin short-circuits to a static level. At 1 ms there are four
//      service calls per period, so a mid-range stage would now actually work.
//
// Raising this back to 5 restores both problems. Set to 0 for a free-running
// loop; there is no longer much reason not to.
const unsigned long LOOP_PAUSE_MS = 1;
