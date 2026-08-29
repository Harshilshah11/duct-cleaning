/*
 * uno_usb_link — Arduino Uno over the USB tether, driving a dual
 * channel motor driver from ground station joystick data (guide Steps 8-9).
 *
 *   Pi -> Uno  "CMD <seq> M <l> <r>\n"                        wheels only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act>\n"                  + actuator, sign only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush>\n"          + brush, 0..255
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush> <light>\n"  + light, 0..255
 *   Pi -> Uno  "CMD <seq> J <x> <y>\n"                        raw stick, -1000..1000
 *   Pi -> Uno  "CMD <seq> STOP\n"                             explicit neutral
 *   Uno -> Pi  "ACK <seq>\n"                                  back to sender addr/port
 *
 * EVERY OUTPUT RIDES IN THE SAME PACKET, as trailing fields on M. They are
 * deliberately not separate datagrams: one packet per frame keeps a SINGLE
 * failsafe clock covering wheels, actuator and brush together, and keeps the
 * traffic at one datagram per 20 ms. A drive command that stopped refreshing
 * some other output's private timer would let that output keep running after
 * the tether died — which for the rod means driving into its end stop, and for
 * the brush means a spinning brush nobody can stop.
 *
 * Trailing fields are OPTIONAL and default to 0 (see the parse in loop()), so
 * an older sender degrades to "wheels only, everything else stopped" rather
 * than to "everything else stuck at its last value".
 *
 * <act> is signed by the Pi: positive extends, negative retracts, and 0 STOPS.
 * Only the sign is used — this channel has no speed demand, having lost the pot
 * to the light on 2026-08-14 — but it does have an enable again on D4, so zero
 * cuts power rather than picking the other direction. The 3-position switch
 * truth table (GPIO16/19, active-LOW) is decoded in ground_station/inputs.py,
 * NOT here:
 *     16=0 19=1 -> EXTEND     16=1 19=1 -> STOP     16=1 19=0 -> RETRACT
 * Both legs closed is a broken interlock; inputs.py calls that FAULT and
 * uno_motors.py sends 0, which is now a genuine stop — the safe answer to a
 * switch you cannot trust.
 *
 * <brush> is a DUTY, 0..255, since 2026-08-17 — it was a plain 0/1 before. The
 * Pi currently sends only 0 or 255: the panel's TOGGLE switch (Pi GPIO13,
 * active-LOW like every switch on that panel) is the whole control, full speed
 * or stopped. The duty path stays because the WIRE should stay expressive —
 * for one evening the pot set the speed here, and it was reverted on the
 * operator's call ("keep 255": one knob feeding both the lamp and the brush
 * meant dimming one slowed the other) without needing a reflash, precisely
 * because only the Pi's brush_demand() had to change. A mid-range duty is
 * synthesised in software — see serviceBrushPwm(), the
 * same mechanism the rod uses on D4; at 0 and 255 it resolves to a static
 * level and costs nothing.
 *
 * <light> is the panel potentiometer (ADS1115 A2), scaled to 0..255 on the Pi
 * in uno_motors.py. It is unsigned — a lamp has no reverse. The pot could take
 * this job only because the actuator stopped needing a speed demand in the same
 * change; before that, one pot could not serve both.
 *
 * The ACK goes back up the same USB port the command arrived on - there is
 * only one peer on a tether, so there is nothing to address:
 * the Pi's sending socket is on an ephemeral port, so a fixed reply port would
 * land nowhere. This also means any machine on the LAN can test the link.
 *
 * For the M form, arcade mixing happens on the Pi (ground_station/uno_motors.py)
 * and this sketch only applies per-wheel demands. Steering can then be retuned
 * without a reflash, and the code next to the motors stays small enough to
 * audit. The J form is the exception: joystick_link.py sends the calibrated but
 * UNMIXED stick, so mixJoystick() below does that one job — see its comment for
 * why the two must stay numerically identical.
 *
 * ---------------------------------------------------------------------------
 * MOTOR PIN MAP — chosen around the shield, NOT freely
 * ---------------------------------------------------------------------------
 *
 *   channel 1 / LEFT    DIR1 = D7        PWM1 = D6    (Timer0 OC0A, 62.5 kHz)
 *   channel 2 / RIGHT   DIR2 = D4        PWM2 = D5    (Timer0 OC0B, 62.5 kHz)
 *   linear actuator     ACT_DIR = A3     ACT_PWM = A2  (LOW on A3 extends!)
 *   brush motor         BRUSH_DIR = D2   BRUSH_PWM = D3 (Timer2 OC2B, 62.5 kHz)
 *   light               LIGHT_DIR = D8   LIGHT_PWM = D9 (Timer1 OC1A, 62.5 kHz)
 *
 * ONE TIMER PER JOB:
 *   Timer0 (D5+D6) - BOTH wheels, so they share a frequency by construction
 *   Timer1 (D9)    - the lamp
 *   Timer2 (D3)    - the brush
 * The rod needs no timer, being soft-PWM, and lives on A2/A3 - which is what
 * frees the timer pins for the four things that actually need dimming.
 *
 * D4 IS THE SHIELD'S microSD CHIP SELECT AND THE RIGHT WHEEL'S DIRECTION LINE
 * OWNS IT — see the DIR2 block below. RUN THIS BOARD WITH THE SLOT EMPTY.
 *
 * THIS MAP MUST MATCH uno_eth_link. Both sketches drive one rig; a pin map that
 * differs between them is a trap, not a variant.
 *
 * The obvious map (PWM on D10/D11) is IMPOSSIBLE with this shield fitted: the
 * W5100/W5500 owns D10 (chip select), D11 (MOSI), D12 (MISO) and D13 (SCK) for
 * SPI, plus D4 for the microSD slot. Driving motors from D10/D11 breaks the
 * Ethernet link and the motors together. D3/D6/D8/D9 clear all of those.
 *
 * D6 IS ON TIMER0, AND THAT HAS ONE REAL CONSEQUENCE. Timer0 also generates
 * millis(), which the failsafe below is timed off. Calling analogWrite() on D6
 * does NOT disturb millis() — only changing Timer0's prescaler would, and
 * nothing here does. What it DOES do is make a 0 duty cycle unreliable: on
 * Timer0 pins, analogWrite(pin, 0) can still emit a narrow pulse every period,
 * which is enough to leave a motor creeping. So every full stop on a PWM pin
 * goes through digitalWrite(pin, LOW), never analogWrite(pin, 0). That is why
 * applyMotor() and safeState() below look asymmetric — it is deliberate, and
 * removing it reintroduces a robot that will not quite stop.
 *
 * The two channels run on different timers and therefore different default PWM
 * frequencies: D3 ~490 Hz (Timer2), D6 ~980 Hz (Timer0). Both drive the bridge
 * fine, but the channels are NOT interchangeable — if you ever raise one
 * timer's frequency, raise BOTH. Mismatched channels respond differently to the
 * same demand, which reads as a mechanical fault.
 *
 * ---------------------------------------------------------------------------
 * ADDRESSING — why .50.20 and not .1.20
 * ---------------------------------------------------------------------------
 *
 * This board must NOT be 192.168.1.20. That is the ground station Pi's own
 * wlan0 address, and a host always delivers traffic for its own address
 * locally, so every packet the Pi sent to the Uno would be answered by the Pi
 * itself and never reach the wire. The eth0 segment is 192.168.50.0/24, so the
 * Uno lives at 192.168.50.20 and ground_station/uno_link.py targets that.
 *
 * FAILSAFE — the reason this sketch is more than a print loop. If no valid
 * packet arrives for FAILSAFE_MS, safeState() stops both motors and the link is
 * marked down. A robot that keeps driving on its last command after the tether
 * dies is the failure mode this exists to prevent, so TEST IT WITH THE WHEELS
 * OFF THE GROUND: unplug the Ethernet cable mid-drive and confirm both motors
 * stop within a third of a second.
 *
 * OTHER WIRING NOTES:
 *   - The shield's microSD slot shares SPI on pin 4, which the rod's RETRACT
 *     line now owns. A card left in that slot fights the Ethernet chip for MISO
 *     whenever the rod is stopped, which is most of the time; the slot must be
 *     EMPTY. See the ACT_DIR/ACT_PWM block.
 *
 * The MAC below is locally administered (the 0x02 bit in the first byte) and
 * must be unique on your LAN. Newer shields ship with a real MAC on a sticker —
 * use that if yours has one.
 *
 * Build: Arduino IDE, board "Arduino Uno", stock Ethernet library.
 */

// NO NETWORK INCLUDES. This is the USB-tether twin of uno_eth_link: same pins,
// same driver logic, same failsafe, same command grammar - only the wire is
// different. Ported 2026-08-26 because the Ethernet link was flapping (17 carrier
// transitions in 9 minutes) and the shield was unreachable, while USB enumerated
// clean. Everything below the transport is BYTE-IDENTICAL to uno_eth_link.ino by
// construction; if you fix a driver bug there, port it here and vice versa.
//
// BAUD IS 115200, AND IT MUST EQUAL uno_serial.py's UNO_BAUD ON THE PI. This
// port carries commands AND ACKs, so the two ends are one link: mismatch them
// and every byte arrives as framing garbage, which presents as "sent 50 /
// acked 0" - indistinguishable from a board running the wrong sketch.
//
// 115200 ON THE OPERATOR'S ORDER 2026-08-27. It had been 250000 since
// 2026-08-26, and the accuracy note that motivated that rate is kept here
// because it is still true and still the thing to check if this link misbehaves.
// At 16 MHz with U2X:
//
//     250000 -> UBRR=7   actual 250000.0   error  0.00%
//     115200 -> UBRR=16  actual 117647.1   error +2.12%
//
// 2.12% is inside the ~4% a UART tolerates, so 115200 works - but it works with
// less margin than 250000 did, and margin is what absorbs a long cable and a
// warm clock. Framing garbage on this port ("sent 50 / acked 0") is the symptom
// that spent margin produces, so suspect the rate before suspecting the sketch.
//
// Going the other way is not free either: 500000 is also exact (UBRR=3), but the
// Uno's 64-byte RX buffer then fills in 1.3 ms against this sketch's 5 ms loop
// pause. 250000 filled it in 2.6 ms; 115200 takes 5.6 ms and so has the most
// headroom of the three. Do not raise it without shortening that pause to match.
const unsigned long SERIAL_BAUD = 115200;

// --- Motor driver pins (see the pin-map note above before changing) ----------
const uint8_t DIR1 = 7;    // channel 1 direction (LEFT)
const uint8_t PWM1 = 6;    // channel 1 speed, Timer0 OC0A - 62.5 kHz
// DIR2 IS THE SHIELD'S microSD CHIP SELECT (D4). It goes LOW whenever the right
// wheel is driven in the negative direction, and LOW is what SELECTS a card in
// that slot - which then drives MISO through any SPI the board is doing and
// corrupts it. With the slot EMPTY, as it must be, D4 is an ordinary output.
// setup() parks it HIGH, which is the deselected state.
const uint8_t DIR2 = 4;    // channel 2 direction (RIGHT) - SD CS, see above
const uint8_t PWM2 = 5;    // channel 2 speed, Timer0 OC0B - 62.5 kHz, SAME timer as PWM1
//
// BOTH WHEEL PWMs ARE ON TIMER2, and that is the point of the D11 choice, not a
// coincidence to be tidied away. D3 is OC2B and D11 is OC2A - two compare
// outputs of the SAME timer - so they share one prescaler and one waveform
// mode and cannot drift to different frequencies. The old map had D3 on Timer2
// and D6 on Timer0, which meant two prescalers to keep in step; the pin note
// below warns that channels on different frequencies answer the same demand
// differently and read as a mechanical fault - a pull to one side on a straight
// run. That is now structurally impossible rather than merely configured away.
//
// D11 IS NOT A TIMER0 PIN. If a comment, a note or a wiring diagram says it is,
// that is stale: Timer0's PWM pins are D5 and D6 only, and nothing here uses
// Timer0 for PWM any more.
//
// THE CLAUSE THAT USED TO END THAT SENTENCE - "which is exactly why MILLIS_SCALE
// is back to 1" - WAS ITSELF STALE, and corrected 2026-08-27. setup() still
// writes TCCR0B to prescaler 1 (search for it), so Timer0 still runs 64x fast
// and MILLIS_SCALE is still 64. Not using Timer0 for PWM did not undo its
// prescaler. This matters more than a tidy comment: the note at MILLIS_SCALE
// records that believing 1 makes the 300 ms failsafe fire at 4.7 ms, and
// believing 64 when it is 1 leaves a robot driving for 19 seconds after the
// link dies. A wrong comment here is a wrong failsafe.
//
// D6 -> D11, on the operator's instruction, after D6 was eliminated as a
// software fault. What was ruled out first, so nobody re-checks it:
//
//   the Pi sends both wheels the same demand   logged "L=+77 R=+77"
//   the pin map matched the working Ethernet build  DIR2=8, PWM2=6
//   applyMotor() drives both channels identically
//   PWM frequency - dead at 62.5 kHz AND at the core's stock rate
//
// Everything upstream of the pin was proven good and the channel was still
// dead, which leaves the pin, its wiring or its driver. D6 is also one of the
// two Timer0 PWM pins, and the light on the OTHER Timer0 pin (D5) was dead at
// the same time - two adjacent pins on one timer failing together.
//
// WHY D11 AND NOT D10, the only other free PWM pin: D11 is on TIMER2, the same
// timer as D3, which is the left wheel and the one channel that never stopped
// working. The two wheels now share a timer, so they run at the same frequency
// BY CONSTRUCTION rather than by keeping two prescaler settings in step. The
// pin note warns that mismatched channel frequencies read as a mechanical
// fault - a pull to one side on a straight run - and that class of bug is now
// impossible here. D10 is Timer1: a third timer at a third rate, keeping the
// problem alive for no gain.
//
// REQUIRES ONE WIRE MOVED: the right driver's PWM input from D6 to D11. DIR2
// stays on D8. If the channel is STILL dead after that, the pin was never the
// fault - it is the driver or its wiring, and the Uno's own 5V rail is the next
// suspect given it dropped off USB five times the same afternoon.

// --- Linear actuator: DIR + PWM, DIR on D7 (pinout corrected 2026-08-15) -----
// Direction comes from the ground station's 3-position actuator switch
// (GPIO16/19 on the Pi), decoded there and arriving as one signed number:
// > 0 extends, < 0 retracts, 0 STOPS.
//
// MEASURED ON THE RIG, and this is the authority — the levels below are the
// ones the driver actually wants, not an inference from how the wheels work:
//
//     level 1 / EXTEND    D7 = LOW    D4 = HIGH
//     middle  / STOP      D7 = held   D4 = LOW    -> rod holds position
//     level 3 / RETRACT   D7 = HIGH   D4 = HIGH
//
// EXTEND IS DIR **LOW**. Every other direction line on this board (the wheels,
// the brush) treats HIGH as forward, so the natural assumption is wrong here and
// applyMotor()'s convention must not be copied onto this channel. That is why
// the two levels are named constants below instead of a bare ternary.
//
// D4 IS THE ONLY THING THAT GATES THE ROD. D7 selects a direction but does not
// start or stop anything, so STOP is D4 LOW with D7 left wherever it was — which
// is what "hold that position" means, and why applyActuator() deliberately does
// not touch the direction line on a stop.
//
// THE DIRECTION LINE IS D7, NOT D2. It was on D2 until this was measured, and D2
// is now free. D7's previous owner was BRUSH_DIR — see the brush block below,
// because that pin cannot serve both and the brush had to move.
//
// Neither pin has a timer behind it, which costs nothing: this channel lost its
// speed demand when the pot moved to the light, so the only levels it ever needs
// are full-scale and off, and a static HIGH is 255/255 duty. The middle stage,
// if it is ever wanted again, is synthesised on D4 — see serviceActuatorPwm().
//
// ---------------------------------------------------------------------------
// D4 IS THE SHIELD'S microSD CHIP SELECT. RUN WITH THE SLOT EMPTY.
// ---------------------------------------------------------------------------
// The rod's second line moved A2 -> D4 on 2026-08-15 to match the wiring on the
// rig. D4 is electrically fine as an output, but it is shared with the microSD
// socket on every W5100/W5500 shield, and that has two consequences worth
// knowing before you chase a ghost:
//
//   1. WITH A CARD IN THE SLOT THIS LINK WILL DIE. CS is active-LOW, so the
//      rod's STOP level (D4 LOW) SELECTS the card — and stopped is what the rod
//      is most of the time. A selected card drives MISO during the SPI reads
//      this sketch makes to the W5100 on every pass of loop(), which corrupts
//      them. The symptom is an Ethernet link that works while the rod is moving
//      and dies when it stops, which reads as a power or wiring fault and is
//      neither. There is no software fix from here: one pin cannot be both a
//      chip select and a motor gate. Empty slot, or move this line to A2-A5.
//
//   2. setup() USED TO PARK D4 HIGH to deselect that card, AND THAT WAS THE BUG
//      THAT KEPT THE ROD RUNNING. That park is now gone — see setup(). Together
//      with D7 being held HIGH as the brush's direction line, it meant the rod
//      saw "retract, at full scale" from the moment the Uno left reset, forever,
//      no matter what the panel switch said. Two pins that nothing thought of as
//      the actuator's added up to a permanent retract command.
//
// If a card ever has to go in that slot, A2 is free and is a drop-in replacement
// — change the one constant below and nothing else, because this channel is
// driven by digitalWrite/soft-PWM and needs no timer.
const uint8_t ACT_DIR = A3;  // "Dir" on the rig — LOW extends, HIGH retracts
const uint8_t ACT_PWM = A2;  // "Pwm" on the rig — no timer, soft-PWM; off the shield now

// Spelled out because this channel is the ODD ONE OUT: LOW is forward here and
// HIGH is forward everywhere else on the board. Swap these two if the rod
// travels the wrong way — it is a one-line change and needs no other edit.
const int ACT_LEVEL_EXTEND = LOW;
const int ACT_LEVEL_RETRACT = HIGH;

// THREE STAGES: the panel's 3-position switch selects 0% / 50% / 100% duty.
//
//     STOP    -> ACT_DUTY_STOP     (0)    rod holds position
//     RETRACT -> ACT_DUTY_RETRACT  (128)  50%
//     EXTEND  -> ACT_DUTY_EXTEND   (255)  100%
//
// Swap the two non-zero values if you want the fast stage on the other throw;
// they are named so that is a one-line change and not an arithmetic puzzle.
// RETRACT RAISED 128 -> 255 on 2026-08-14. At 128 the rod moved on the extend
// throw and did nothing at all on the retract throw, which is the signature of a
// stage that cannot break away rather than one that is wired wrong: 50% duty is
// about where a loaded actuator stalls, and lifting against gravity is the
// direction that runs out of torque first. Put it back to 128 if you want the
// slow stage, but only once the rod is known to travel BOTH ways at full scale.
const int ACT_DUTY_STOP = 0;
const int ACT_DUTY_RETRACT = 255;
const int ACT_DUTY_EXTEND = 255;

// NEITHER LINE IS A TIMER PIN, so the 50% stage is generated in software by
// serviceActuatorPwm() below, on whichever line is the active one. 0% and 100%
// still resolve to a plain static level and cost nothing; only the middle stage
// is synthesised.
//
// 250 Hz. Slow enough that the main loop — which spends most of its time in an
// SPI read of the W5100 — can hit each edge closely enough, fast enough that the
// rod sees a smooth average rather than steps. A linear actuator is mechanically
// far too slow to care about ripple at this rate.
// SCALED, because micros() comes off Timer0 exactly as millis() does - see
// MILLIS_SCALE. With Timer0 at prescaler 1 this counter runs 64x fast, so a
// raw 4000 here would be 62.5 us of real time: the soft-PWM period for the rod
// and the brush would have jumped from 250 Hz to 16 kHz, far faster than
// loop() can service, and the duty would have collapsed into noise.
//
// x64 IS BACK, because Timer0 is prescaled again and micros() counts 64x fast
// with it. 4000 us * 64 = 250 Hz real, which is what the rod, the brush and the
// lamp were all tuned for.
//
// THIS BIT US ONCE ALREADY: when D6 stopped being a wheel the prescaler went
// away and this factor did not, which left every soft-PWM channel running at
// ~4 Hz. The rod and the brush hid it behind their inertia; the LAMP flickered
// visibly. See MILLIS_SCALE - the two are the same setting.
const unsigned long ACT_PWM_PERIOD_US = 4000UL * 64UL;

/* HOW LONG loop() PAUSES EACH PASS. Raised to 10 ms real on the operator's
 * instruction 2026-08-27 ("delay(10) in the void loop so the uno wont stuck").
 *
 * IT IS STILL NOT delay(10), AND THAT REMAINS THE WHOLE POINT - see the pause
 * itself at the bottom of loop(). delay() stops the world, and the world it
 * stops includes serviceActuatorPwm() and serviceBrushPwm(). Their period is
 * ACT_PWM_PERIOD_US, 4 ms. Servicing a 4 ms waveform once per 10 ms pause is
 * worse than the 5 ms case this file already documents: 10000 % 4000 leaves the
 * sampled phase alternating between just 0 and 2000 us, so the chopper can only
 * ever express 0% or 50% duty at about 100 Hz. That is visible flicker and
 * audible stutter, and it is exactly the "brush powered on off on off" symptom
 * already recorded here. The busy-wait keeps both stages serviced at loop speed
 * throughout the pause, so the pacing costs nothing.
 *
 * THE OLD VALUE WAS NOT 5 ms EITHER, which is worth knowing before trusting any
 * timing comment in this file. It read `micros() - pauseStart < 5000UL`,
 * unscaled - and micros() runs MILLIS_SCALE times fast because Timer0 is at
 * prescaler 1 for the brush PWM. 5000 of those is 78 REAL microseconds, so the
 * loop was never paced at 200 Hz; it ran flat out and polled the W5100 as fast
 * as SPI allowed. Note ACT_PWM_PERIOD_US directly above IS scaled, which is why
 * the actuator's 4 ms was right while the pause beside it was 64x short.
 *
 * WHAT THE PAUSE BUYS, now that it is real: the AVR stops hammering the shield
 * with back-to-back SPI transactions it has no reason to make. The Pi sends at
 * 50 Hz, so polling at tens of kHz finds nothing almost every time. At 10 ms a
 * command waits at most 10 ms longer inside the W5100 - well inside the 20 ms
 * command period and nowhere near the 300 ms failsafe - and the board draws
 * less doing it, which on a rail this marginal is the point. */
const unsigned long LOOP_PAUSE_US = 10000UL * 64UL;   // 10 ms REAL


// The duty currently demanded on D4, 0..255. Written by applyActuator(), acted
// on by serviceActuatorPwm() every pass of loop().
int actDuty = 0;

// Flip if EXTEND on the panel drives the rod the wrong way. Equivalent to
// swapping ACT_LEVEL_EXTEND/ACT_LEVEL_RETRACT; use whichever reads better to
// you, but do not do both — they cancel.
const bool INVERT_ACT = false;

// --- Panel light: dimmable, added 2026-08-14 ---------------------------------
// Brightness follows the panel potentiometer (ADS1115 A2 on the Pi), scaled to
// 0..255 there and applied here. The pot was free to take this job precisely
// because the actuator above lost its PWM pin, and with it any use for a speed
// demand — the two changes are one change.
//
// LIGHT_DIR carries the driver channel's direction line, which a lamp does not
// actually need — see applyLight(). It is a static level, so it costs no timer.
// ---------------------------------------------------------------------------
// PIN MAP vs THE ETHERNET SHIELD - READ BEFORE GOING BACK TO uno_eth_link.ino
//
// This USB build now uses D11 (right wheel PWM) and D12 (lamp return leg).
// BOTH BELONG TO THE SPI BUS the W5100/W5500 shield runs on:
//
//     D10  shield chip select        D12  MISO
//     D11  MOSI                      D13  SCK   (also STATUS_LED here)
//     D4   SD-card chip select on shields with a card slot
//
// So this pin map and the Ethernet shield CANNOT COEXIST. On the USB tether
// that costs nothing - there is no shield in the stack and those pins are just
// ordinary I/O. But if the Ethernet build is ever flashed again, the wheel and
// the lamp have to move back off D11/D12 first, or the shield and the motors
// will fight over the same three wires and neither will work.
//
// That is not hypothetical: D4 already carries this warning in the actuator's
// note above, and it cost this rig a day when the SD chip-select and the rod's
// gate turned out to be the same pin.
// ---------------------------------------------------------------------------
const uint8_t LIGHT_DIR = 8;
// THE LAMP'S PWM, set to D5 on the operator's pin map 2026-08-26.
//
// D5 is Timer0 OC0B, and Timer0 is ALREADY at prescaler 1 for D6's wheel PWM -
// so this pin gets 62.5 kHz with no extra setup, the same way D6 does. The two
// share a timer: do not un-prescale Timer0 while either lives here, or both the
// wheel and the lamp change frequency together. See MILLIS_SCALE.
//
// HISTORY, because this pin has been contested all day and the record is worth
// more than the current value:
//
//   A0 / D12   software-chopped from loop(). Smooth at 255, fluctuating at every
//              other level - a polled chopper mistimes both edges of a partial
//              duty, and 255 is the one level that has no edges.
//   D5         tried, reported dead.
//   D10 / D11  both driven at once to test in one flash; D10 reported working.
//   D5         set again here, per the operator's map.
//
// IF IT IS DARK AGAIN, D5 is the pin to suspect first - it and D6 are Timer0's
// only two PWM outputs and both have been reported dead at least once today.
// D10 and D11 are free and both are on healthy timers.
const uint8_t LIGHT_PWM = 9;   // Timer1 OC1A - hardware PWM 62.5 kHz

// --- Brush motor: DIR + PWM on a driver channel (rewired 2026-08-14) ---------
// Driven from the panel's TOGGLE switch (Pi GPIO13).
//
// THIS WAS ONE PIN AND THAT IS WHY IT DID NOT WORK. The brush hangs off a
// dual-channel driver channel, exactly like the wheels, so it needs BOTH inputs:
// a direction line AND a speed line. Driving D7 alone set the direction of a
// channel whose PWM input was never asserted, so the bridge stayed off and the
// motor never turned — while the telemetry cheerfully reported BRUSH=ON, because
// the sketch really was driving the one pin it knew about.
//
// SPEED CONTROL ARRIVED 2026-08-17: the pot (freed from the rod, then shared
// with the light) now sets the brush duty, 0..255 in the <brush> field. The
// duty is SOFT-PWM — serviceBrushPwm() below chops
// the pin from loop() exactly the way serviceActuatorPwm() runs the rod's D4.
// The 0 and 255 endpoints still resolve to static levels, so OFF cannot be
// caught mid-cycle and full speed cannot be chopped by a stalled loop; only
// the middle range is synthesised, where a stall costs a slower brush, never a
// runaway one. (Hardware PWM was never an option: 3 and 6 are the wheels, 5
// the light, 9 is DIR1, 10-13 belong to the shield's SPI.)
// MOVED D7 -> D2 ON 2026-08-15, AND THIS WAS NOT A TIDY-UP. D7 is the linear
// actuator's direction line on the rig, and this sketch was holding it HIGH
// permanently as the brush's direction — a constant, written on every frame.
// HIGH on that pin means RETRACT, so between this and the SD-deselect park on
// D4 the rod was handed a full-scale retract command from reset onwards, by two
// lines neither of which anyone thought of as the actuator's. That is the whole
// explanation for a rod that only ever drove one way and never stopped.
//
// D2 IS CONFIRMED AGAINST THE RIG (Harshil, 2026-08-15): brush is Dir -> D2,
// Pwm -> D3, driven from the panel TOGGLE on Pi GPIO13. This is not an inference
// from D7 having been taken — the wire really is on D2, and the sketch was the
// thing that was wrong.
const uint8_t BRUSH_DIR = 2;
const uint8_t BRUSH_PWM = 3;    // Timer2 OC2B - hardware-capable pin

// The brush spins one way only, so its direction is a constant rather than a
// demand. Flip this if the brush runs backwards.
const bool BRUSH_DIR_LEVEL = HIGH;

// Many driver and relay inputs are ACTIVE-LOW — the channel enables when the pin
// goes low, and such a board will run the brush the whole time the Uno is in
// reset if this is wrong. Set false for those. Applies to BRUSH_PWM, the line
// that actually gates the motor — including the soft-PWM's on-phase, which
// serviceBrushPwm() inverts through this same constant.
const bool BRUSH_ACTIVE_HIGH = true;

// The duty currently demanded on BRUSH_PWM, 0..255. Written by applyBrush(), acted on
// by serviceBrushPwm() every pass of loop() — the same pairing as
// actDuty / serviceActuatorPwm(), and it shares ACT_PWM_PERIOD_US (250 Hz)
// because two independent soft-PWM periods would just be two numbers to keep
// in sync for no benefit. A brush motor's inertia makes 250 Hz ripple
// invisible, exactly as it does for the rod.
int brushDuty = 0;


// The smallest duty that actually TURNS the brush rather than buzzing it, the
// same physics as MIN_DUTY on the wheels. Any non-zero demand is stretched
// onto BRUSH_MIN_DUTY..255 so the bottom of the knob's travel is already a
// moving brush instead of a heater. Set to 0 for a raw linear map.
const int BRUSH_MIN_DUTY = 90;

// Flip either of these if a wheel spins the wrong way. Doing it here is much
// cheaper than re-soldering, and far safer than negating on the Pi — the Pi
// feeds BOTH this link and uno_serial.py, so a sign flip there would silently
// desync the two transports.
const bool INVERT_1 = false;
// TRUE 2026-08-26: the right wheel came back on D11 but ran BACKWARDS - it
// drove reverse while the left drove forward on the same demand. That is a
// motor-lead polarity, not a code fault: the channel was dead until this
// afternoon, so nothing had ever established which way round its leads were.
//
// Flipped HERE rather than by swapping the two motor leads, because the leads
// are the harder thing to get at and this is the constant that exists for it.
// If the leads are ever re-terminated, set this back to false rather than
// stacking a second inversion on top - the two cancel and the wheel silently
// goes backwards again.
const bool INVERT_2 = true;

// --- Failsafe ----------------------------------------------------------------
// 300 ms: long enough to ride out a handful of dropped datagrams at the 50 Hz
// command rate (15 in a row), short enough that the robot stops within a third
// of a second of a real tether failure.
// TIMER0'S PRESCALER IS CHANGED IN setup() TO RAISE THE D6 PWM FREQUENCY, and
// Timer0 is what drives millis(). At prescaler 1 instead of the stock 64,
// millis() counts 64x too fast - so every duration measured in "ms" below is
// really that many 64ths of a millisecond.
//
// EVERY millis()-DERIVED CONSTANT IS THEREFORE SCALED BY THIS. Without it the
// failsafe fires after 300/64 = 4.7 ms of real time, and since command packets
// arrive every 20 ms it would trip between every single one - the wheels would
// never turn at all. That is not a subtle regression; it is a robot that does
// not drive.
//
// If Timer0 is ever put back to its stock prescaler, set this to 1.
// PWM_FAST — do the wheel pins run at 62.5 kHz, or at the core's stock rates?
//
// TRUE was the 2026-08-24 setting: both wheel timers to prescaler 1, giving
// 62.5 kHz on D3 and D6. FALSE leaves the Arduino core alone: D3 at ~490 Hz
// (Timer2, phase-correct) and D6 at ~980 Hz (Timer0, fast).
//
// SET FALSE 2026-08-26 TO TEST A DEAD RIGHT CHANNEL. The right wheels stopped
// responding entirely while the left ran normally, on identical demands - the
// Pi logs "L=+77 R=+77" and only one side turns. 62.5 kHz is above what many
// driver modules can switch: an L298N and most of its clones give up somewhere
// between 20 and 40 kHz, and a driver that cannot follow its gate signal reads
// as a dead channel, not as a slow one. If the two sides use different driver
// modules - which nothing in this sketch knows - the faster setting can kill
// one and not the other.
//
// If the right side comes back with this false, the frequency was the fault and
// the fix is to pick a rate BOTH drivers can switch, not to go back to 62.5.
// If it stays dead, the fault is downstream of the Uno - wiring, driver or
// motor - and this should go back to true.
// TESTED FALSE 2026-08-26 AND IT WAS NOT THE FAULT: the right channel stayed
// dead at the core's stock 490/980 Hz too, so the driver was never failing to
// switch a 62.5 kHz gate. Back to true - the fast rate is inaudible and buys a
// smoother drive, and there is no reason to keep a whinier one that fixed
// nothing. Whatever is wrong with the right channel is DOWNSTREAM of this chip.
const bool PWM_FAST = true;

// SCALED BY THE PRESCALER, and it MUST track PWM_FAST. With Timer0 at prescaler
// 1 the core's millis() runs 64x fast, so every duration this sketch measures
// is multiplied to compensate. Leave this at 64 with PWM_FAST false and the
// 300 ms failsafe becomes 19 SECONDS - a robot that keeps driving for nineteen
// seconds after the link dies. That is why it is derived here rather than
// written as a number.
// 64 AGAIN, because Timer0 is back at prescaler 1 for D6's PWM and that makes
// the core's millis() tick 64x fast. Every duration in this sketch is written
// as `X * MILLIS_SCALE` so they all follow this one number.
//
// GET THIS WRONG AND THE FAILSAFE STOPS MEANING 300 ms. At 1 with a prescaled
// Timer0 it becomes 4.7 ms and the wheels stutter; at 64 with a stock Timer0 it
// becomes 19 SECONDS of a robot still driving after the link has died. It has
// been both today.
//
// ONE SETTING IN THREE PLACES: this, ACT_PWM_PERIOD_US, and the TCCR0B line in
// setup(). Change one, change all three.
const unsigned long MILLIS_SCALE = 64UL;

// 300 ms REAL: long enough to ride out a handful of dropped datagrams at the
// 50 Hz command rate, short enough that the robot stops within a third of a
// second of a real tether failure. The multiply is what keeps it 300 ms of
// wall clock rather than 300 of Timer0's accelerated ticks.
const unsigned long FAILSAFE_MS = 300UL * MILLIS_SCALE;

// --- Tuning ------------------------------------------------------------------
// Below this the motor buzzes and heats without turning, so treat it as zero.
const int DEADBAND = 12;
const int MAX_PWM = 255;

// The smallest duty that actually TURNS a loaded wheel. Between DEADBAND and
// roughly a third of full duty these gearmotors only buzz: the bridge is
// switching, but the average voltage never breaks static friction, so a
// half-deflected stick makes heat instead of motion.
//
// Any surviving non-zero demand is therefore stretched onto MIN_DUTY..MAX_PWM
// instead of 1..MAX_PWM, so the first millimetre of stick travel already drives.
//
// Zero stays exactly zero. This raises the smallest MOVING demand; it must never
// turn a stop into a crawl. DEADBAND decides what counts as a stop and must stay
// BELOW this value — DEADBAND zeroes the small demands first, and only what
// survives is lifted to MIN_DUTY.
//
// Set to 0 to disable the floor entirely and go back to a linear 0..255 map.
// Raise it if the robot still stalls on carpet, lower it if it lurches the
// moment the stick leaves centre. 90/255 is about 35%.
const int MIN_DUTY = 90;

// --- Buffers -----------------------------------------------------------------
// NOT UDP_TX_PACKET_MAX_SIZE. That constant is 24 bytes in the stock library and
// silently truncates anything longer, which looks exactly like a garbled link.
// 96 bytes is affordable out of the Uno's 2 KB of SRAM; keep the Pi side's
// MAX_PAYLOAD below it.
const uint16_t RX_BUFFER = 96;
char packet[RX_BUFFER];

// The UDP socket's replacement: a line assembler over the USB CDC port. The
// W5x00 handed us whole datagrams with their own boundaries; a serial stream has
// none, so the newline in "CMD <seq> ...\n" becomes the frame marker and this
// buffer holds the partial line between passes.
uint16_t rxLen = 0;
bool rxOverflow = false;

unsigned long lastPacketMs = 0;
bool linkUp = false;
uint16_t lastSeq = 0;
unsigned long packetsReceived = 0;

// Bench telemetry over USB serial. LINK UP / LINK DOWN alone cannot tell
// "commands are flowing" from "nothing ever arrived" — both are silent in the
// second case, because LINK DOWN only fires on a transition out of LINK UP.
// These print the demand actually applied, which is also what you read when
// deciding whether INVERT_1 / INVERT_2 need flipping.
int curL = 0;
int curR = 0;
int curA = 0;
int curB = 0;
int curLight = 0;
int printedL = 0;
int printedR = 0;
int printedA = 0;
int printedB = 0;
int printedLight = 0;
unsigned long lastPrintMs = 0;

// Bench telemetry on/off. OFF by default since 2026-08-26.
//
// WHY IT HAD TO GO: the telemetry line is about 65 characters, and the Uno's
// serial TRANSMIT buffer is 64 bytes. One byte over, and Serial.print() stops
// being a queue and becomes a BLOCKING wait - roughly 2 ms at 250000 baud while
// the line drains. loop() is stopped dead for that whole time, and with it the
// software choppers that drive the lamp, the rod and the brush.
//
// The lamp is the one that shows it. Its on-window at low brightness is ~150 us,
// so a 2 ms freeze is more than ten whole cycles held at whatever level the pin
// happened to be on - a visible flash or dropout, repeating every 200 ms while
// the knob is moving. At FULL brightness the pin is held high anyway and the
// freeze is invisible, which is exactly the reported symptom: "very fluctuate in
// low pwm, complete work on full pwm".
//
// The rod and the brush have inertia and hide it. A lamp does not.
//
// NOTHING NEEDS THIS TO RUN. The Pi already logs L/R/act/brush/light every
// second in motor_cam.log, from the demand it SENT, and reads the link's health
// from ACKs - which are only ~10 bytes and never fill the buffer. This print was
// a bench aid from before that logging existed.
//
// Set true to get it back when debugging on a serial monitor, and expect the
// lamp to flicker while it is on.
const bool TELEMETRY = false;

const uint8_t STATUS_LED = LED_BUILTIN;

/* Apply one motor channel. Sign picks direction, magnitude becomes PWM. */
void applyMotor(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert) {
  if (invert) demand = -demand;
  if (demand > MAX_PWM) demand = MAX_PWM;
  if (demand < -MAX_PWM) demand = -MAX_PWM;
  if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

  // Direction is set BEFORE the new PWM value. The other order spends a few
  // microseconds driving the old direction at the new speed, which is a current
  // spike through the bridge on every reversal.
  digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);

  int duty = demand >= 0 ? demand : -demand;
  if (duty == 0) {
    // NOT analogWrite(pwmPin, 0) — on a Timer0 pin (PWM2 = D6) that can still
    // emit a narrow pulse each period and leave the motor creeping. See the
    // pin-map note at the top.
    digitalWrite(pwmPin, LOW);
  } else {
    if (MIN_DUTY > 0) {
      // Stretch 1..MAX_PWM onto MIN_DUTY..MAX_PWM so the slowest demand the
      // stick can express is still one the motor can act on. Done here rather
      // than on the Pi so it covers EVERY sender — the UDP link, uno_serial.py
      // and any bench tool alike — and so two of them cannot apply it twice.
      //
      // The multiply is promoted to long deliberately: 254 * 165 is 41910, which
      // overflows the Uno's 16-bit int and would wrap to a negative duty, i.e. a
      // wheel that runs backwards near full stick.
      duty = MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - MIN_DUTY)) / (MAX_PWM - 1));
    }
    analogWrite(pwmPin, duty);
  }
}

/* Brush motor: duty 0..255 off the panel pot, gated by the panel toggle (the
 * Pi folds the two into one number — see brush_demand()). A pre-2026-08-17
 * sender's 0/1 still behaves: 1 stretches to BRUSH_MIN_DUTY, a slow brush
 * rather than a dead one.
 *
 * Both lines are written every call, not just the one that changed. The
 * direction is a constant, but writing it here means a channel that browns out
 * and comes back gets its direction restored by the next frame rather than
 * running whichever way its input floated to. */
void applyBrush(int duty) {
  if (duty < 0) duty = 0;
  if (duty > MAX_PWM) duty = MAX_PWM;

  // A STOP DRIVES BOTH LINES LOW, and that is the whole fix for "brush is always
  // on" (operator, 2026-08-29).
  //
  // This function used to assert BRUSH_DIR unconditionally, on the reasoning
  // that direction is a constant for a brush that spins one way. So a stopped
  // brush sat at DIR = HIGH, PWM = LOW - and setup() left it there from reset.
  //
  // THAT IS ONLY "OFF" IF THE DRIVER TAKES DIR + ENABLE. Plenty of dual-channel
  // boards instead take two logic inputs, IN1 and IN2, where HIGH/LOW is not
  // "stopped pointing forwards" but FORWARD AT FULL SCALE. On such a channel the
  // old code commanded the brush to run from the moment the Uno left reset, and
  // no demand from the Pi could countermand it, because every applyBrush(0) wrote
  // exactly the same pair of levels.
  //
  // Both lines LOW is a real stop under EITHER reading: IN1=IN2=LOW is coast on
  // a two-input driver, and enable LOW is off on a DIR+enable one, where the
  // direction line is then don't-care. That is why this is written as a stop of
  // both pins rather than as a polarity constant - it does not require knowing
  // which kind of board is on the other end of the wire.
  if (duty <= 0) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(BRUSH_DIR, LOW);
    brushDuty = 0;
    return;
  }

  // Direction before speed, the same ordering rule applyMotor() follows: setting
  // the gate first would spend a moment driving the old direction at full duty.
  digitalWrite(BRUSH_DIR, BRUSH_DIR_LEVEL);
  if (BRUSH_MIN_DUTY > 0) {
    // Same stretch as applyMotor(): the smallest demand the knob can express
    // must be one the motor can act on. Long arithmetic for the same overflow
    // reason — 254 * 165 wraps a 16-bit int.
    duty = BRUSH_MIN_DUTY
           + (int)(((long)(duty - 1) * (MAX_PWM - BRUSH_MIN_DUTY))
                   / (MAX_PWM - 1));
  }
  // APPLIED ON THIS LINE, RISING OR FALLING - see the note at serviceBrushPwm().
  // The two ENDPOINTS land here rather than waiting for the chopper's next pass,
  // so a demand of 255 is at 255 before this function returns and a stop is a
  // stop. Mid-range duty is still synthesised by serviceBrushPwm().
  brushDuty = duty;
  if (duty <= 0) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  } else if (duty >= MAX_PWM) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
  }
}

/* Software PWM for the brush. D9 DOES have a timer (Timer1, already configured
 * for 8-bit fast PWM below), so this could be handed to hardware the way
 * uno_eth_link does it - deliberately not done here, to keep this sketch's
 * change to the pin map alone. Chopping D9 from loop() works because
 * digitalWrite() clears the timer's compare-output bits on the way past. Same
 * contract as
 * serviceActuatorPwm(): called every pass of loop(), static levels for the 0
 * and 255 endpoints so neither OFF nor full speed can be caught mid-cycle, and
 * only the middle range is chopped — where a stalled loop costs a slower
 * brush, never a runaway one. Shares ACT_PWM_PERIOD_US; both mechanisms are
 * far too slow mechanically to care about 250 Hz ripple. */
/* THE BRUSH GOES STRAIGHT TO THE DEMANDED DUTY. No ramp, no soft-start.
 *
 * A 1.5 s soft-start lived here for part of 2026-08-27 and was REMOVED THE SAME
 * DAY on the operator's instruction: "make brush not slow to fast, jake direct
 * fast 255". The brush is a toggle in practice - the Pi sends 0 or 255 - and
 * waiting a second and a half for it to wind up is not what the job wants.
 *
 * THE REASON IT WAS TRIED IS STILL TRUE, and is written down so the next person
 * weighs it rather than rediscovers it. Going 0-to-100% in one step is a slam
 * into a motor that is not yet turning, and a stalled DC motor draws its
 * locked-rotor current - several times its running current - until it spins up.
 * On a shared rail that is a real sag.
 *
 * What changed is the diagnosis, not the physics. The disconnects that motivated
 * the ramp were later measured to be a power fault in their own right: the
 * board's .noinit boot counter caught "ram=LOST (true power loss)", meaning the
 * 5 V rail reached zero, and the Pi hard-reset eight times the same day with a
 * 60 s watchdog armed. A ramp cannot fix a rail that is being cut, so it was
 * paying a real cost in responsiveness for a benefit that was never the cure.
 * If the supply is ever fixed and the brush still disturbs the link, this is the
 * first thing to try again - the code is in git history at 7c52a31. */
void serviceBrushPwm() {
  if (brushDuty <= 0) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    return;
  }
  if (brushDuty >= MAX_PWM) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
    return;
  }
  unsigned long phase = micros() % ACT_PWM_PERIOD_US;
  unsigned long onFor = (ACT_PWM_PERIOD_US * (unsigned long)brushDuty) / MAX_PWM;
  bool on = phase < onFor;
  digitalWrite(BRUSH_PWM, (on == BRUSH_ACTIVE_HIGH) ? HIGH : LOW);
}

/* Linear actuator: D7 picks the direction, D4 gates it, zero holds position.
 *
 * ZERO IS A REAL STOP. D4 is what powers the channel, so dropping it low leaves
 * the rod exactly where it is — which is what the panel's middle throw means and
 * what the failsafe needs. It is not a brake and not a reversal; the rod simply
 * stops being driven.
 *
 * THE DIRECTION LINE IS DELIBERATELY NOT TOUCHED ON A STOP. Re-pointing it at a
 * rod that is no longer powered buys nothing, and holding the last direction
 * means a resumed command carries on the way it was already going. It also
 * keeps STOP to a single write on the one pin that matters.
 */
void applyActuator(int demand, bool invert) {
  if (invert) demand = -demand;
  if (demand == 0) {
    actDuty = ACT_DUTY_STOP;
    digitalWrite(ACT_PWM, LOW);
    return;
  }
  // Direction BEFORE the gate, the same ordering rule applyMotor() follows: the
  // other order spends a few microseconds driving the old direction at full
  // scale, which is a current spike through the bridge on every reversal.
  //
  // ACT_LEVEL_* rather than a bare HIGH/LOW because this channel extends on LOW
  // while every other direction line here goes forward on HIGH.
  digitalWrite(ACT_DIR, demand > 0 ? ACT_LEVEL_EXTEND : ACT_LEVEL_RETRACT);
  // Only the SIGN chooses the stage. The Pi sends full scale either way, so
  // reading a magnitude here would just be reading a constant — and the whole
  // point of the three stages is that the switch position picks the speed.
  actDuty = demand > 0 ? ACT_DUTY_EXTEND : ACT_DUTY_RETRACT;
}

/* Software PWM for the rod, because D4 has no timer. Called every pass of
 * loop(), which is what makes it work at all: the loop is short and the only
 * thing that can stall it for long is a Serial.print, so the edges land close
 * enough for a mechanism this slow.
 *
 * 0 and 255 short-circuit to a static level. That matters for more than speed:
 * a stopped rod must be held LOW by something that cannot be caught mid-cycle,
 * and a full-speed rod should not be chopped by a software timer that a blocked
 * loop could freeze in the low half. Only the middle stage is synthesised, and
 * a stall there costs a slower rod, never a runaway one. */
void serviceActuatorPwm() {
  if (actDuty <= ACT_DUTY_STOP) {
    digitalWrite(ACT_PWM, LOW);
    return;
  }
  if (actDuty >= MAX_PWM) {
    digitalWrite(ACT_PWM, HIGH);
    return;
  }
  // micros() wraps about every 71 minutes; the modulo makes that a single
  // short cycle, not a stuck output, so it is left unhandled deliberately.
  unsigned long phase = micros() % ACT_PWM_PERIOD_US;
  unsigned long onFor = (ACT_PWM_PERIOD_US * (unsigned long)actDuty) / MAX_PWM;
  digitalWrite(ACT_PWM, phase < onFor ? HIGH : LOW);
}

/* Panel light: brightness 0..255, straight off the potentiometer.
 *
 * LIGHT_DIR is held HIGH rather than steered. A lamp has no reverse, but a motor
 * driver channel used as a dimmer still wants a defined polarity on its
 * direction input; on a driver that ignores the pin this costs nothing.
 *
 * NO DEADBAND here, deliberately unlike applyMotor(). A motor below ~12 buzzes
 * and heats without turning, so folding that to zero is right. A lamp at 12/255
 * is simply dim, and applying the same rule would give the pot a dead patch at
 * the bottom of its travel that reads as a broken knob.
 */
void applyLight(int level) {
  if (level < 0) level = 0;
  if (level > MAX_PWM) level = MAX_PWM;

  // HARDWARE PWM ON D5. Moved here 2026-08-26 with the operator's approval, and
  // it is the fix rather than another tuning of one.
  //
  // WHAT WAS WRONG WITH THE SOFTWARE CHOPPER, because the symptom named it
  // exactly: "255 does not fluctuate, 0-250 does". A polled chopper only moves
  // the pin when loop() gets round to calling it, so every partial duty has two
  // edges per cycle and each lands late by however long loop() was busy
  // elsewhere. Varying on-time IS varying brightness. At 255 the pin is simply
  // HELD HIGH - zero edges, nothing to mistime - which is why that one value was
  // always rock steady. No period could fix that; changing it only made the
  // fixed timing error a smaller fraction of a longer window.
  //
  // A TIMER HAS NO SUCH PROBLEM. OC0B toggles D5 in silicon on exact clock
  // counts, and nothing loop() does can disturb it.
  //
  // 62.5 kHz FOR FREE: D5 is Timer0, which is ALREADY at prescaler 1 for D6's
  // wheel PWM, so this pin inherits the same 62.5 kHz with no extra setup. The
  // two are one setting - see MILLIS_SCALE, and do not un-prescale Timer0 while
  // the lamp lives here.
  //
  // A0 IS THE RETURN LEG and is simply held at ground. D12 is no longer used by
  // the lamp at all.
  digitalWrite(LIGHT_DIR, LOW);

  if (level <= 0) {
    // NOT analogWrite(pin, 0): a zero duty can still emit a narrow pulse every
    // period, which on a lamp is a faint glow that will not go out.
    // digitalWrite is the only certain dark.
    digitalWrite(LIGHT_PWM, LOW);
  } else if (level >= MAX_PWM) {
    digitalWrite(LIGHT_PWM, HIGH);
  } else {
    analogWrite(LIGHT_PWM, level);
  }
}

/* The lamp no longer needs software chopping - D5 is a timer pin and OC0B does
 * it in hardware. Kept as an empty function so the two call sites in loop() stay
 * valid; the compiler removes it. Delete both calls if you ever tidy loop(). */
void serviceLightPwm() {
}

/* Both motors to neutral. Runs on every failsafe trip, so it is unconditional
 * and must not depend on any prior state. The LED doubles as a link lamp: lit
 * means commands are arriving, dark means failsafe. */
void safeState() {
  // digitalWrite, not analogWrite(pin, 0): PWM2 is on Timer0, where a 0 duty is
  // not a guaranteed dead level. This is the one place that must be certain.
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  // The rod is genuinely STOPPED, not merely pointed somewhere: applyActuator(0)
  // drops BOTH drive lines, which is this driver's off state. Before the channel
  // was rewired as a pair a failsafe could only pick a direction, and the rod ran
  // to its end stop.
  applyActuator(0, INVERT_ACT);
  // Brush off too — via applyBrush so an active-LOW module gets the right
  // level. A spinning brush is the loudest thing on the robot; it must not be
  // what survives a failsafe.
  applyBrush(0);
  // Light out. It is the one output here that poses no motion hazard, so
  // leaving it lit was tempting — but the cameras stream over the SAME tether,
  // so once the link is down there is nobody left to see by it, and this rig
  // already browns out under load (the Pi logs under-voltage). Dark is cheaper.
  applyLight(0);
  digitalWrite(STATUS_LED, LOW);
}

/* Arcade mix for the "J" form: stick (-1000..1000) -> wheels (-255..255).
 *
 * Mirrors mix() in ground_station/uno_serial.py exactly, because BOTH feed this
 * board: uno_motors.py mixes on the Pi and sends M, while joystick_link.py
 * sends the raw calibrated stick as J and expects the mixing here. If you
 * change one, change the other, or the robot steers differently depending on
 * which program is driving.
 *
 * y drives both wheels together, x drives them in opposition. Full deflection
 * on both axes would demand 2.0 from one wheel, so the PAIR is scaled down
 * together — clipping each wheel on its own instead bends the turn as the robot
 * speeds up. No deadband here: the Pi has already removed it, and applying a
 * second one would silently eat part of that calibration.
 */
void mixJoystick(int x, int y, int *left, int *right) {
  long l = (long)y + (long)x;
  long r = (long)y - (long)x;

  long peak = 1000;                    // == max(1.0, ...) in the Python
  if (labs(l) > peak) peak = labs(l);
  if (labs(r) > peak) peak = labs(r);

  *left = (int)(l * MAX_PWM / peak);
  *right = (int)(r * MAX_PWM / peak);
}

/* WHY DID THE BOARD RESTART? Ported from uno_eth_link 2026-08-27.
 *
 * A disconnect can be the AVR resetting or the link wedging, and the two want
 * completely different fixes, so the board says which.
 *
 * MCUSR holds the reset source but must be read before the C runtime clobbers
 * it - hence .init3, which runs ahead of the .bss clear in .init4. BORF is a
 * BROWN-OUT: the 5V rail sagged below the detector threshold. That is the
 * signature a motor inrush leaves. Optiboot may clear MCUSR before we ever see
 * it - measured on this rig, it does - which is what the .noinit counter is for.
 *
 * .noinit survives a RESET but not a POWER LOSS: RAM holds its contents down to
 * roughly 1.5V while the brown-out detector trips at about 2.7V. Intact magic
 * therefore means the board reset with power broadly maintained; garbage magic
 * means the rail actually collapsed. That distinction does not depend on the
 * bootloader leaving MCUSR alone. */
uint8_t  resetFlags  __attribute__((section(".noinit")));
uint16_t bootCount   __attribute__((section(".noinit")));
uint16_t bootMagic   __attribute__((section(".noinit")));
bool     ramSurvived = false;
static const uint16_t BOOT_MAGIC = 0xB07F;

void captureResetCause(void) __attribute__((naked, used, section(".init3")));
void captureResetCause(void) {
  resetFlags = MCUSR;
  MCUSR = 0;
}

/* Print a pin number the way the schematic names it. Ported from uno_eth_link
 * 2026-08-27 together with the pin swap, because the two banner lines below used
 * to be hand-typed literals - and a banner exists precisely to catch a stale
 * board. A hand-typed one restates the bug it was meant to reveal. */
void printPin(uint8_t pin) {
  if (pin >= A0) {
    Serial.print('A');
    Serial.print(pin - A0);
  } else {
    Serial.print('D');
    Serial.print(pin);
  }
}

void reportResetCause() {
  Serial.print(F("RESET: flags=0x"));
  Serial.print(resetFlags, HEX);
  if (resetFlags & _BV(PORF))  Serial.print(F(" POWER-ON"));
  if (resetFlags & _BV(EXTRF)) Serial.print(F(" EXTERNAL"));
  if (resetFlags & _BV(BORF))  Serial.print(F(" BROWN-OUT"));
  if (resetFlags & _BV(WDRF))  Serial.print(F(" WATCHDOG"));
  Serial.print(ramSurvived ? F("  ram=KEPT (reset, rail held)")
                           : F("  ram=LOST (true power loss)"));
  Serial.print(F("  boot#"));
  Serial.println(bootCount);
}

void setup() {

  // Boot bookkeeping before anything can use RAM for other purposes. See the
  // .noinit note above: intact magic means we reset without losing the rail.
  if (bootMagic != BOOT_MAGIC) {
    bootMagic = BOOT_MAGIC;
    bootCount = 0;
    ramSurvived = false;
  } else {
    bootCount++;
    ramSurvived = true;
  }

  // 250000, NOT the Ethernet sketch's console rate: this port is the command
  // and ACK channel and must match uno_serial.py's UNO_BAUD on the Pi exactly.
  // See SERIAL_BAUD above for why 250000 and not 115200.
  Serial.begin(SERIAL_BAUD);
  reportResetCause();

  // Outputs are driven to a stopped state BEFORE they become outputs, so the
  // pins cannot glitch high in the gap between pinMode and the first write.
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  // BOTH rod lines must be at their stopped level before they become outputs —
  // the same reasoning as the brush's gate below. This is also what replaced the
  // old "park pin 4 HIGH to deselect the SD card" line: HIGH on D4 is the rod's
  // RETRACT drive, and that park is why the rod ran from reset forever.
  digitalWrite(ACT_DIR, ACT_LEVEL_EXTEND);
  digitalWrite(ACT_PWM, LOW);
  // BOTH LOW = dark at this polarity (no differential), and this runs before
  // pinMode so the lamp cannot flash during the gap. Do NOT "tidy" these to
  // match applyLight's A0-HIGH/D5-HIGH off state: A0 HIGH with D5 still an
  // input would light the lamp through the pull-up for that instant.
  digitalWrite(LIGHT_DIR, LOW);
  digitalWrite(LIGHT_PWM, LOW);
  // The brush's OFF level is written BEFORE pinMode, and on an active-LOW
  // module that level is HIGH. Writing it first switches on the input pull-up,
  // which holds the module off across the gap; leaving the pin floating there
  // is exactly how a relay board ends up running the brush during reset.
  // Only BRUSH_PWM gates the motor, so it is the one that must be safe here —
  // the direction line can settle whenever.
  digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(BRUSH_DIR, BRUSH_DIR_LEVEL);
  pinMode(BRUSH_PWM, OUTPUT);
  pinMode(BRUSH_DIR, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ACT_DIR, OUTPUT);
  pinMode(ACT_PWM, OUTPUT);
  // Nothing special is needed for these two; pinMode(OUTPUT) is the whole
  // ceremony.
  pinMode(LIGHT_DIR, OUTPUT);
  pinMode(LIGHT_PWM, OUTPUT);
  // --- PWM frequency ---------------------------------------------------
  // Operator's change 2026-08-24. Both wheel PWM pins go to prescaler 1.
  //
  // Timer2: D3
  // Prescaler = 1, AND fast PWM -> 62.5 kHz, matching D6.
  //
  // The prescaler alone gave 31.37 kHz, not 62.5: the Arduino core leaves
  // Timer2 in PHASE-CORRECT PWM (WGM=001), which counts up and back down and
  // so takes 510 ticks per period instead of 256. Timer0 is left in fast PWM
  // by the core, hence the 2:1 split. Setting WGM21 alongside the core's WGM20
  // makes WGM=011 (fast PWM, TOP=255) and the two land on the same number:
  //
  //     16e6 / (256 * 1) = 62500 Hz   both channels
  //
  // Matching matters - the pin note warns that channels on different
  // frequencies respond differently to the same demand and read as a
  // mechanical fault, which is a pull to one side on a straight run.
  if (PWM_FAST) {
    TCCR2A |= _BV(WGM21);
    TCCR2B = (TCCR2B & 0b11111000) | 0b001;

    // TIMER1 left configured for 8-bit fast PWM at prescaler 1. The lamp is on
    // D5 (Timer0) now, so nothing uses this - it is harmless, and it means D10
    // or D9 are ready to carry the lamp at 62.5 kHz if D5 fails again.
    // The core leaves Timer1 in 8-bit PHASE-CORRECT at prescaler 64, which is
    // 490 Hz - fine for a servo, useless as a fair comparison against D11.
    //   16 MHz / 256 counts / 1 = 62500 Hz
    // Since 2026-08-27 D9 IS the brush's gate - but serviceBrushPwm() drives it
    // with digitalWrite(), which clears Timer1's compare-output bits, so the
    // timer's configuration here never reaches the pin and the brush does not
    // notice this block. Hand the brush to analogWrite() and that stops being
    // true: the two would then share Timer1, which is fine, but say so here.
    TCCR1A = _BV(WGM10);                    // 8-bit fast PWM, low bits
    TCCR1B = _BV(WGM12) | _BV(CS10);        // ... high bit, prescaler 1
  }

  // Timer0: D6
  // Prescaler = 1 -> about 62.5 kHz
  // WARNING: this breaks normal millis()/micros()/delay() timing - millis()
  // now runs 64x fast. MILLIS_SCALE above compensates every duration this
  // sketch measures; anything ELSE that calls millis() (the Ethernet library's
  // internal timeouts, any future delay()) is NOT compensated and will be 64x
  // short. There are no delay() calls in this sketch today, and the link runs
  // on a static IP so nothing here waits on a DHCP timeout.
  if (PWM_FAST) {
    // TIMER0 BACK TO PRESCALER 1, because D6 is a wheel again and has to match
    // D3's 62.5 kHz. Two wheels on different PWM frequencies answer the same
    // demand differently and read as a mechanical fault - a pull to one side on
    // a straight run - which is why this is not optional.
    //
    // THE PRICE IS A 64x FAST millis(). MILLIS_SCALE below compensates every
    // duration this sketch measures; ACT_PWM_PERIOD_US carries the same factor.
    // All three move together or none of them do.
    TCCR0B = (TCCR0B & 0b11111000) | 0b001;
  }

  // NOTE: STATUS_LED is LED_BUILTIN = D13, which is also the SPI clock the
  // shield uses. With the shield fitted this lamp tracks Ethernet traffic
  // rather than link state, and is not a reliable indicator. Harmless, but do
  // not read anything into it — and never put a real signal on D13.
  pinMode(STATUS_LED, OUTPUT);
  safeState();

  // NO SD DESELECT HERE ANY MORE, AND THIS WAS HALF THE BUG. Pin 4 is the rod's
  // gate (ACT_PWM), and the old pinMode(4, OUTPUT)/digitalWrite(4, HIGH) pair
  // asserted it at every reset and never lowered it again — nothing else in the
  // sketch touched pin 4. The other half was D7, held HIGH as the brush's
  // direction, which on the rod's driver means RETRACT. Full-scale retract,
  // latched from reset, from two pins neither of which was thought of as the
  // actuator's. Neither line looked wrong on its own.
  //
  // With the slot empty there is no card to deselect, so the line is simply
  // gone; safeState() above has already left D4 LOW. See the ACT_DIR/ACT_PWM
  // block.

  // NO ADDRESSING AT ALL on this build - the tether has exactly one peer. The
  // Ethernet twin needed a static IP here because DHCP would stall ~60 s, unlike
  // the DHCP form which stalls ~60 s when no server answers. On a point-to-point
  // tether there is no DHCP server at all, so static is the only sane choice.
  // Nothing to bring up: the CDC port is already open by the time setup() runs.
  // This banner is printed anyway because the Pi's _open() sleeps OPEN_SETTLE_S
  // and then reset_input_buffer()s, so it is flushed before the first command -
  // it exists for whoever opens a serial monitor, exactly like the old one.
  Serial.print(F("uno_usb_link on USB serial @"));
  Serial.println(SERIAL_BAUD);
  Serial.print(F("DIR1="));           printPin(DIR1);
  Serial.print(F(" PWM1="));          printPin(PWM1);
  Serial.print(F(" (left)  DIR2="));  printPin(DIR2);
  Serial.print(F(" PWM2="));          printPin(PWM2);
  Serial.println(F(" (right)"));
  // Printed because a silently stale board is the expensive failure here: the
  // link ACKs and the pins look right whatever build is loaded, so every value
  // that changes behaviour belongs in the banner where a reset reveals it.
  Serial.print(F("deadband<"));
  Serial.print(DEADBAND);
  Serial.print(F(", non-zero demand scaled to "));
  Serial.print(MIN_DUTY);
  Serial.print(F(".."));
  Serial.println(MAX_PWM);
  // Printed as the truth table rather than as two pin numbers, because the bug
  // this channel spent a day on was a WIRING SCHEME misread, not a wrong pin:
  // both pin numbers were right the whole time. A banner that says only
  // "ACT=D2/D4" would have looked correct on the broken build too.
  Serial.print(F("ACT_DIR="));   printPin(ACT_DIR);
  Serial.print(F(" ACT_PWM="));  printPin(ACT_PWM);
  Serial.print(F(" - LOW on ")); printPin(ACT_DIR);
  Serial.println(F(" EXTENDS (opposite the wheels)"));
  Serial.print(F("  soft-PWM stages 0/"));
  Serial.print(ACT_DUTY_RETRACT);
  Serial.print(F("/"));
  Serial.print(ACT_DUTY_EXTEND);
  Serial.println(F(" (stop/retract/extend)"));
  // Loud, and in the banner rather than a comment, because the failure it warns
  // about looks like a flaky cable: with a card in the slot the link dies except
  // while retracting. A reset is the one moment someone is watching.
  // Printed from DIR2, not typed: the pin this warns about moved on 2026-08-29
  // from the rod's gate to the right wheel's direction line.
  Serial.print(F("  ^ "));
  printPin(DIR2);
  Serial.println(F(" is the shield's SD chip select - RUN WITH THE SLOT EMPTY"));
  // The build with pot speed control announces itself: an ACK-identical old
  // build is otherwise indistinguishable over the LAN (see the banner note
  // above), and "held 255" vs "soft-PWM" is exactly the difference that
  // decides whether the knob does anything.
  Serial.print(F("BRUSH_DIR="));   printPin(BRUSH_DIR);
  Serial.print(F(" BRUSH_PWM="));  printPin(BRUSH_PWM);
  Serial.print(F(" soft-PWM duty 0-"));
  Serial.print(MAX_PWM);
  Serial.print(F(" floor "));
  Serial.print(BRUSH_MIN_DUTY);
  Serial.println(F(" (TOGGLE on/off, Pi sends 0 or 255)"));
  Serial.print(F("LIGHT: HARDWARE PWM "));
  printPin(LIGHT_PWM);
  Serial.print(F(" (Timer1, 62.5kHz), return "));
  printPin(LIGHT_DIR);
  Serial.println();
  // DIVIDED BY MILLIS_SCALE, because FAILSAFE_MS is counted in the 64x-fast
  // milliseconds a prescaled Timer0 produces. Printing the raw constant said
  // "19200 ms" on a board whose failsafe is really 300 ms, which reads like a
  // robot that keeps driving for nineteen seconds after the link dies - alarming
  // and wrong. The banner exists to be trusted at a glance, so it prints real
  // time.
  Serial.print(F("failsafe after "));
  Serial.print(FAILSAFE_MS / MILLIS_SCALE);
  Serial.print(F(" ms of silence  (raw "));
  Serial.print(FAILSAFE_MS);
  Serial.print(F(" @ MILLIS_SCALE "));
  Serial.print(MILLIS_SCALE);
  Serial.println(F(")"));
}

/* Assemble one newline-terminated command out of the serial stream.
 *
 * Returns true exactly once per complete line, with `packet` NUL-terminated and
 * ready for the same sscanf ladder the Ethernet build used. This is the only
 * function in this sketch with no counterpart in uno_eth_link.ino - the W5x00
 * did this framing in hardware.
 *
 * An over-long line is swallowed to its newline rather than parsed as a
 * truncated command: half a command is not a safe thing to hand to sscanf,
 * because "CMD 7 M 200 2" is a VALID parse of a truncated "CMD 7 M 200 250".
 *
 * Returning mid-drain is deliberate. Any bytes still in the UART are picked up
 * on the next pass; at 50 Hz commands and 115200 baud the 64-byte hardware
 * buffer holds about 5.5 ms of traffic and the loop pauses 5 ms, so it cannot
 * back up in practice.
 */
bool pumpSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;                       // tolerate CRLF senders
    }
    if (c == '\n') {
      if (rxOverflow) {               // drop the whole over-long line
        rxLen = 0;
        rxOverflow = false;
        Serial.println(F("WARN: over-long line dropped"));
        continue;
      }
      packet[rxLen] = '\0';
      rxLen = 0;
      if (packet[0] != '\0') {
        return true;
      }
      continue;                       // bare newline is not a command
    }
    if (rxLen >= RX_BUFFER - 1) {
      rxOverflow = true;
      continue;
    }
    packet[rxLen++] = c;
  }
  return false;
}

void loop() {
  // First thing every pass: the rod's 50% stage and the brush's pot-set speed
  // are both synthesised in software, so the more often these run the cleaner
  // their duty. Everything else in this loop is either instant or rate-limited.
  serviceActuatorPwm();
  serviceBrushPwm();
  serviceLightPwm();

  if (pumpSerial()) {
    unsigned int seq = 0;
    int left = 0;
    int right = 0;
    bool understood = false;

    int jx = 0;
    int jy = 0;
    int act = 0;
    int brush = 0;
    int light = 0;

    // ONE pattern for every length of M, judged by how many fields sscanf
    // actually filled. Testing the short patterns as separate branches does not
    // work: "CMD %u M %d %d" happily matches the LONGER string too — sscanf
    // simply stops early and returns 3 — so a shorter branch placed first would
    // silently swallow <act>/<brush> and freeze them at their last value.
    //
    // Anything the sender omitted stays 0, which is also the safe default: a
    // sender that cannot talk about the actuator, the brush or the light must
    // never be able to leave any of them running.
    int nf = sscanf(packet, "CMD %u M %d %d %d %d %d",
                    &seq, &left, &right, &act, &brush, &light);
    if (nf >= 3) {
      if (nf < 4) act = 0;
      if (nf < 5) brush = 0;
      if (nf < 6) light = 0;
      applyMotor(DIR1, PWM1, left, INVERT_1);
      applyMotor(DIR2, PWM2, right, INVERT_2);
      applyActuator(act, INVERT_ACT);
      applyBrush(brush);
      applyLight(light);
      curL = left;
      curR = right;
      curA = act;
      curB = brush;
      curLight = light;
      understood = true;
    } else if (sscanf(packet, "CMD %u J %d %d", &seq, &jx, &jy) == 3) {
      // Raw stick from joystick_link.py, already centred and deadbanded on the
      // Pi but NOT mixed. Mixing happens here for this form only.
      mixJoystick(jx, jy, &left, &right);
      applyMotor(DIR1, PWM1, left, INVERT_1);
      applyMotor(DIR2, PWM2, right, INVERT_2);
      // J carries no actuator, brush or light, so none of them run.
      applyActuator(0, INVERT_ACT);
      applyBrush(0);
      applyLight(0);
      curL = left;
      curR = right;
      curA = 0;
      curB = 0;
      curLight = 0;
      understood = true;
    } else if (sscanf(packet, "CMD %u STOP", &seq) == 1) {
      curL = 0;
      curR = 0;
      curA = 0;
      curB = 0;
      curLight = 0;
      safeState();
      understood = true;
    } else if (sscanf(packet, "CMD %u", &seq) == 1) {
      // A bare keepalive with no payload. Valid: it proves the link is alive
      // and refreshes the failsafe without changing the motor demand.
      understood = true;
    }

    if (understood) {
      lastSeq = (uint16_t)seq;
      packetsReceived++;
      lastPacketMs = millis();

      if (!linkUp) {
        linkUp = true;
        Serial.println(F("LINK UP"));
      }
      digitalWrite(STATUS_LED, HIGH);

      // Same three bytes plus the number the Pi's _drain_acks() greps for. It
      // matches on the "ACK " prefix and ignores every other line, which is why
      // the telemetry block below can keep printing to this same port.
      Serial.print(F("ACK "));
      Serial.println(lastSeq);
    } else {
      Serial.print(F("WARN: unparsable line: "));
      Serial.println(packet);
    }
  }

  // millis() subtraction, never `millis() > last + FAILSAFE_MS`. Unsigned
  // wraparound at ~49 days makes the additive form compare wrong exactly once,
  // and this form stays correct across the rollover.
  if (linkUp && (millis() - lastPacketMs) >= FAILSAFE_MS) {
    linkUp = false;
    curL = 0;
    curR = 0;
    curA = 0;
    curB = 0;
    safeState();
    Serial.print(F("LINK DOWN - failsafe after "));
    Serial.print(packetsReceived);
    Serial.println(F(" packets"));
  }

  // Report on change (rate-limited, or a moving stick floods the port at 50 Hz)
  // and on a 3 s heartbeat, so a resting link still proves itself.
  if (TELEMETRY) {
    unsigned long now = millis();
  bool changed = (curL != printedL || curR != printedR || curA != printedA
                  || curB != printedB || curLight != printedLight)
                 && (now - lastPrintMs) > 200UL * MILLIS_SCALE;
  // Same scaling as FAILSAFE_MS - without it the telemetry would print 64x as
  // often and drown the serial line at 9600 baud.
  if (changed || (now - lastPrintMs) > 3000UL * MILLIS_SCALE) {
    printedL = curL;
    printedR = curR;
    printedA = curA;
    printedB = curB;
    printedLight = curLight;
    lastPrintMs = now;
    Serial.print(F("L="));
    Serial.print(curL);
    Serial.print(F(" R="));
    Serial.print(curR);
    Serial.print(F(" ACT="));
    Serial.print(curA);
    Serial.print(F(" BRUSH="));
    Serial.print(curB);          // duty 0..255 now, not ON/off
    Serial.print(F(" LIGHT="));
    Serial.print(curLight);
    Serial.print(F("  pkts="));
    Serial.println(packetsReceived);
  }
  }

  // 5 ms pause per pass, on the operator's order 2026-08-18: it holds the loop
  // near 200 Hz so a command waits at most 5 ms extra in the W5x00 - well
  // inside the 50 Hz command period and the 300 ms failsafe.
  //
  // IT IS NOT delay(5), AND THAT IS THE WHOLE POINT. delay() stops the world,
  // and the world it was stopping included serviceBrushPwm() and
  // serviceActuatorPwm() - whose period, ACT_PWM_PERIOD_US, is 4 ms. Servicing
  // a 4 ms waveform once every 5 ms cannot resolve it: the sampled phase
  // (micros() % 4000) advances by exactly 5000 % 4000 = 1000 us per pass, so it
  // only ever lands on 0, 1000, 2000, 3000 and takes four passes - 20 ms - to
  // walk one cycle. What reached the brush was therefore a 50 Hz square wave
  // with its duty quantised to 25% steps, not a 250 Hz chop at the demanded
  // duty. 50 Hz is visible flicker and audible stutter, which is exactly how it
  // presented: the brush "powered on off on off" at any mid duty, while 0 and
  // 255 looked perfect because both endpoints bypass the chopper entirely
  // (see serviceBrushPwm) and never alias.
  //
  // So: pause for the same 5 ms, but keep the two synthesised stages running
  // through it. The pacing the operator asked for is unchanged; the PWM now
  // gets serviced at loop speed - tens of kHz - instead of once per pause.
  unsigned long pauseStart = micros();
  while (micros() - pauseStart < LOOP_PAUSE_US) {
    serviceActuatorPwm();
    serviceBrushPwm();
    serviceLightPwm();
  }
}
