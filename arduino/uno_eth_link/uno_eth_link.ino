/*
 * uno_eth_link — Arduino Uno + W5100/W5500 Ethernet shield, driving a dual
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
 * to the light — but it does have an enable again on A2, so zero
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
 * because only the Pi's brush_demand() had to change. D3 is Timer2 OC2B, so any
 * duty is real hardware PWM at 62.5 kHz and costs the loop nothing.
 *
 * <light> is the panel potentiometer (ADS1115 A2), scaled to 0..255 on the Pi
 * in uno_motors.py. It is unsigned — a lamp has no reverse. The pot could take
 * this job only because the actuator stopped needing a speed demand in the same
 * change; before that, one pot could not serve both.
 *
 * The ACK goes to udp.remoteIP()/remotePort(), NOT to a hardcoded Pi address:
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
 * EVERY PIN IS NOW SPOKEN FOR. D0/D1 are the USB serial this telemetry goes out
 * on, D10-D13 belong to the shield, and all four of the Uno's usable PWM pins
 * (D3, D5, D6, D9) now carry the four things that actually need dimming: both
 * wheels, the brush and the lamp. Nothing spends a timer pin on a direction
 * line any more. The rod, which needs no timer because it is soft-PWM, moved out
 * to A2/A3 - and that is what freed the timer pins up.
 *
 * ONE TIMER PER JOB, which is the real gain of this map:
 *   Timer0 (D5+D6) - BOTH wheels, so they share a frequency by construction
 *   Timer1 (D9)    - the lamp
 *   Timer2 (D3)    - the brush
 * The two wheels being on ONE timer matters more than it looks. This file has
 * carried a warning for weeks that channels on different frequencies answer the
 * same demand differently and read as a pull to one side on a straight run.
 * That is now structurally impossible rather than merely configured away.
 *
 * D4 IS THE SHIELD'S microSD CHIP SELECT AND THE RIGHT WHEEL'S DIRECTION LINE
 * NOW OWNS IT - see the DIR2 block below. RUN THIS BOARD WITH THE SLOT EMPTY.
 *
 * TIMER NAMES, easy to misremember and worth having written down: on an
 * ATmega328P D6 is Timer0 OC0A, D5 is Timer0 OC0B, D3 is Timer2 OC2B and D9 is
 * Timer1 OC1A. LEFT is DIR1 + PWM1; RIGHT is DIR2 + PWM2.
 *
 * The obvious map (PWM on D10/D11) is IMPOSSIBLE with this shield fitted: the
 * W5100/W5500 owns D10 (chip select), D11 (MOSI), D12 (MISO) and D13 (SCK) for
 * SPI, plus D4 for the microSD slot. Driving motors from D10/D11 breaks the
 * Ethernet link and the motors together. D2-D9 and A2/A3 clear all of those.
 *
 * D6 IS ON TIMER0, AND THAT HAS ONE REAL CONSEQUENCE. Timer0 also generates
 * millis(), which the failsafe below is timed off. Calling analogWrite() on D5/D6
 * does NOT disturb millis() — only changing Timer0's prescaler would, and
 * nothing here does. What it DOES do is make a 0 duty cycle unreliable: on
 * Timer0 pins, analogWrite(pin, 0) can still emit a narrow pulse every period,
 * which is enough to leave a motor creeping. So every full stop on a PWM pin
 * goes through digitalWrite(pin, LOW), never analogWrite(pin, 0). That is why
 * applyMotor() and safeState() below look asymmetric — it is deliberate, and
 * removing it reintroduces a robot that will not quite stop.
 *
 * Both channels are OC0A and OC0B of Timer0, so one prescaler governs the pair
 * and they run at the same 62.5 kHz by construction. Keep it that way: if a
 * wheel is ever moved to another timer, match the frequency deliberately,
 * because mismatched channels respond differently to the
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

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
// Reaches the W5100 registers directly. Needed because the library gives no
// public way to RESET the chip after init() has run once - see the re-init
// block in loop().
#include <utility/w5100.h>

// --- Network (address plan from ground_station/config.py) --------------------
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
IPAddress ip(192, 168, 50, 20);
const uint16_t LISTEN_PORT = 5005;

// --- Motor driver pins (see the pin-map note above before changing) ----------
const uint8_t DIR1 = 7;    // channel 1 direction (LEFT)
const uint8_t PWM1 = 6;    // channel 1 speed, Timer0 OC0A - 62.5 kHz
// DIR2 IS THE SHIELD'S microSD CHIP SELECT (D4). It goes LOW whenever the right
// wheel is driven in the negative direction, and LOW is what SELECTS a card in
// that slot - which then drives MISO through the SPI reads this sketch makes to
// the W5100, corrupting them. With the slot EMPTY, as it must be, D4 is an
// ordinary output and none of this applies. setup() parks it HIGH (deselected).
const uint8_t DIR2 = 4;    // channel 2 direction (RIGHT) - SD CS, see above
const uint8_t PWM2 = 5;    // channel 2 speed, Timer0 OC0B - 62.5 kHz, SAME timer as PWM1

// --- Linear actuator: DIR on A3, PWM on A2 ----------------------------------
// Direction comes from the ground station's 3-position actuator switch
// (GPIO16/19 on the Pi), decoded there and arriving as one signed number:
// > 0 extends, < 0 retracts, 0 STOPS.
//
// MEASURED ON THE RIG, and this is the authority — the levels below are the
// ones the driver actually wants, not an inference from how the wheels work:
//
//     level 1 / EXTEND    A3 = LOW    A2 = HIGH
//     middle  / STOP      A3 = held   A2 = LOW    -> rod holds position
//     level 3 / RETRACT   A3 = HIGH   A2 = HIGH
//
// EXTEND IS DIR **LOW**. Every other direction line on this board (the wheels,
// the brush) treats HIGH as forward, so the natural assumption is wrong here and
// applyMotor()'s convention must not be copied onto this channel. That is why
// the two levels are named constants below instead of a bare ternary.
//
// A2 IS THE ONLY THING THAT GATES THE ROD. A3 selects a direction but does not
// start or stop anything, so STOP is A2 LOW with A3 left wherever it was — which
// is what "hold that position" means, and why applyActuator() deliberately does
// not touch the direction line on a stop.
//
// Neither pin has a timer behind it, which costs nothing: this channel lost its
// speed demand when the pot moved to the light, so the only levels it ever needs
// are full-scale and off, and a static HIGH is 255/255 duty. The middle stage,
// if it is ever wanted again, is synthesised on A2 — see serviceActuatorPwm().
//
// ---------------------------------------------------------------------------
// THE ROD IS CLEAR OF THE SHIELD.
// ---------------------------------------------------------------------------
// A2/A3 carry no timer, no SPI and nothing the W5100 wants. This channel is
// driven by digitalWrite and soft-PWM and needs neither, so nothing the rod does
// can disturb the Ethernet link.
//
// THE SHIELD'S microSD CHIP SELECT IS STILL ON THE BOARD THOUGH - it is DIR2
// (D4). See that constant. RUN WITH THE microSD SLOT EMPTY.
const uint8_t ACT_DIR = A3;  // "Dir" on the rig — LOW extends, HIGH retracts
const uint8_t ACT_PWM = A2;  // "Pwm" on the rig — the only line that gates it

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
// MILLIS_SCALE is declared further down with the failsafe, so the factor is
// spelled out here rather than referenced - keep the two in step.
//
// 4000 us * 64 = 250 Hz real, which is what both mechanisms were tuned for.
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

// The duty currently demanded on ACT_PWM, 0..255. Written by applyActuator(),
// acted on by serviceActuatorPwm() every pass of loop().
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
const uint8_t LIGHT_DIR = 8;
const uint8_t LIGHT_PWM = 9;   // Timer1 OC1A - hardware PWM 62.5 kHz

// --- Brush motor: DIR + PWM on a driver channel (rewired 2026-08-14) ---------
// Driven from the panel's TOGGLE switch (Pi GPIO13).
//
// THE BRUSH NEEDS BOTH INPUTS. It hangs off a dual-channel driver channel,
// exactly like the wheels, so a direction line alone does nothing: drive DIR
// without asserting PWM and the bridge stays off while the telemetry happily
// reports BRUSH=ON. Both pins, every time.
//
// The pot (shared with the light) sets the duty, 0..255 in the <brush> field.
// BRUSH_PWM is Timer2 OC2B, so that duty is real hardware PWM at 62.5 kHz - the
// same figure the wheels and the lamp run at - and the loop does no work for it.
const uint8_t BRUSH_DIR = 2;
const uint8_t BRUSH_PWM = 3;    // Timer2 OC2B - hardware PWM 62.5 kHz

// The brush spins one way only, so its direction is a constant rather than a
// demand. Flip this if the brush runs backwards.
const bool BRUSH_DIR_LEVEL = HIGH;

// Many driver and relay inputs are ACTIVE-LOW — the channel enables when the pin
// goes low, and such a board will run the brush the whole time the Uno is in
// reset if this is wrong. Set false for those. Applies to BRUSH_PWM, the line
// that actually gates the motor — including the soft-PWM's on-phase, which
// serviceBrushPwm() inverts through this same constant.
const bool BRUSH_ACTIVE_HIGH = true;

// The duty currently demanded on BRUSH_PWM, 0..255. Written by applyBrush() —
// the same pairing as
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
// TRUE, synced from uno_usb_link 2026-08-26. The right wheel ran BACKWARDS on
// the rig once its channel started working again - a motor-lead polarity, not
// a code fault, and the same on this build because it is the same motor and
// the same driver. The USB twin has carried this since it was found; this one
// had not been flashed since, so it still had the old value.
//
// KEEP THE TWO SKETCHES IN STEP. They share every pin and every driver; only
// the transport differs. A fix found on one is a fix owed to the other, and a
// wheel that reverses when you change transport is exactly the kind of bug
// that costs an afternoon.
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
const unsigned long MILLIS_SCALE = 64;

// Serial is DIAGNOSTIC ONLY in this sketch - commands and telemetry ride
// Ethernet - so this rate has to match only whatever reads the port: the Arduino
// IDE monitor, `screen /dev/ttyACM0 115200`, and diag/uno_logger.py on the Pi.
// A mismatch costs the log rather than the robot, but it costs all of it.
//
// 115200 on the operator's order 2026-08-27, back down from 250000 (itself
// raised from 115200 on 2026-08-26). See the note at Serial.begin() for the
// accuracy tradeoff that swap carries.
const unsigned long SERIAL_BAUD = 115200;

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

EthernetUDP udp;

unsigned long lastPacketMs = 0;

// --- the shield re-init, and the fault it exists for -------------------------
// SYMPTOM 2026-08-26: on barrel-jack power alone the board never answers on
// Ethernet; plug USB in and it works; REMOVE USB again and it KEEPS working.
// That last part is the tell - a cable, a subnet or a firmware fault would
// break again the moment USB came out.
//
// CAUSE: the W5100's reset is not reliably asserted on a slow power ramp. From
// cold, the regulator brings 5 V up over milliseconds, Ethernet.begin() in
// setup() runs against a chip still in reset, and the shield is left
// uninitialised - deaf, with a perfectly good cable. Plugging USB pulls DTR,
// which RESETS THE AVR; setup() runs again against a shield that is now fully
// powered, and it comes up. Nothing about USB matters except that it happens to
// reset the processor.
//
// FIX: provide that second reset ourselves instead of needing a human with a
// cable. If nothing has arrived over UDP for this long, re-run Ethernet.begin()
// and reopen the socket. On a healthy link this NEVER fires - the Pi sends at
// SEND_HZ and the counter is refreshed constantly - so it is free when not
// needed.
//
// 5 s: long enough that a brief carrier drop does not thrash the chip (re-init
// leaves it deaf for ~60 ms), short enough that a cold start is driving within
// a few seconds of power-up.
const unsigned long ETH_REINIT_MS = 5000UL * MILLIS_SCALE;

// How long to tolerate silence BEFORE the first packet ever arrives - see
// everHeard. Sized so a Pi booting from cold alongside this board is never
// mistaken for a broken shield: Linux plus the ground station is comfortably
// under a minute, this board is listening about five seconds in, and 45 s of
// patience covers the gap with room to spare. Nothing waits on it - the link
// comes up the instant the Pi speaks, whatever this value is. It only decides
// how long the board sits quiet instead of resetting a chip that is fine.
const unsigned long ETH_COLD_WAIT_MS = 45000UL * MILLIS_SCALE;


// --- is the USB port allowed to DRIVE the robot? -----------------------------
// FALSE on the operator's instruction 2026-08-26: "data transfer only via
// ethernet". USB is a FLASHING CABLE on this rig, nothing more.
//
// The Pi already agrees - UNO_TRANSPORT is pinned to "udp" in uno_motors.py -
// so this is the board saying the same thing. Both ends stating one data path
// is worth more than either end assuming it: with the serial parser live, a
// bench script or a stray terminal could drive the wheels over a cable that was
// only plugged in to reflash, and nothing would report it as unusual.
//
// WHAT STAYS: Serial itself is untouched. The boot banner still prints, the
// board is still flashed over USB, and a serial monitor still shows what it
// always did. Only the COMMAND parser on that port is switched off.
//
// WHAT YOU LOSE: the second wire. Until 2026-08-26 this build accepted commands
// from either transport and a dead Ethernet link simply failed over to USB. That
// redundancy is now off by choice - set this true to get it back, and the Pi's
// UNO_TRANSPORT=auto with it.
const bool SERIAL_COMMANDS = false;
unsigned long lastUdpMs = 0;
// HAS ANY PACKET EVER ARRIVED SINCE BOOT? Added 2026-08-27.
//
// It separates two silences that the recovery path used to treat identically.
// Before first contact the ground station may simply still be booting - the Pi
// needs the better part of a minute to start transmitting, while this board is
// listening within about five seconds of power-on. Resetting the shield through
// that window resets a chip that was never broken, which is what turned a normal
// cold start into "it does not connect until I cycle the Uno a few times".
//
// After first contact the meaning inverts: a link that WAS working has gone
// quiet, and that is the wedge worth resetting for.
bool everHeard = false;
unsigned long lastEthTryMs = 0;
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
    // NOT analogWrite(pwmPin, 0) — on a Timer0 pin that can still emit a
    // narrow pulse each period and leave the motor creeping. BOTH wheels are
    // on Timer0 (D6 and D5), so this matters for both. See the pin map.
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
/* The only place the brush pin is actually written.
 *
 * The two ENDPOINTS still use digitalWrite rather than analogWrite(0)/(255): on
 * a timer pin a zero duty can still emit a narrow pulse every period, which is a
 * creeping motor - the same caveat this file already records for D6 and D5. A
 * hard level is the only certain stop, and the only certain full-on. */
void writeBrushHardware(int duty) {
  if (duty <= 0) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  } else if (duty >= MAX_PWM) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
  } else {
    analogWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? duty : (MAX_PWM - duty));
  }
}

void applyBrush(int duty) {
  if (duty < 0) duty = 0;
  if (duty > MAX_PWM) duty = MAX_PWM;
  // Direction before speed, the same ordering rule applyMotor() follows: setting
  // the gate first would spend a moment driving the old direction at full duty.
  digitalWrite(BRUSH_DIR, BRUSH_DIR_LEVEL);
  if (duty > 0 && BRUSH_MIN_DUTY > 0) {
    // Same stretch as applyMotor(): the smallest demand the knob can express
    // must be one the motor can act on. Long arithmetic for the same overflow
    // reason — 254 * 165 wraps a 16-bit int.
    duty = BRUSH_MIN_DUTY
           + (int)(((long)(duty - 1) * (MAX_PWM - BRUSH_MIN_DUTY))
                   / (MAX_PWM - 1));
  }
  // APPLIED ON THIS LINE, RISING OR FALLING - see the note at serviceBrushPwm().
  // Both directions are immediate: a demand of 255 is at 255 before this
  // function returns, and a stop is a stop. Nothing waits for a later pass.
  brushDuty = duty;
  writeBrushHardware(brushDuty);
}

/* Software PWM for the brush, because A1 has no timer. Same contract as
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
  // NOTHING TO DO. The brush is on Timer2 and its duty is applied in hardware by
  // applyBrush(), so there is no waveform to synthesise. Kept as an empty call
  // because loop() and the failsafe path both invoke it, and because this is
  // where a chopper would go if the brush ever moved to a timer-less pin.
}

/* Linear actuator: A3 picks the direction, A2 gates it, zero holds position.
 *
 * ZERO IS A REAL STOP. A2 is what powers the channel, so dropping it low leaves
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

  // LIGHT_DIR IS THE RETURN LEG. IT STAYS LOW. ALWAYS.
  //
  // This channel is a TWO-LEG BRIDGE, not a direction line plus a gate, and the
  // lamp sees the DIFFERENCE between its two legs. That was settled on the rig
  // 2026-08-26 by three observations, not by reading the driver's part number:
  //
  //     DIR=HIGH PWM=0      lamp ON     <- "light is on in bot", knob at zero
  //     DIR=LOW  PWM=0      lamp OFF    <- correct
  //     DIR=HIGH PWM=level  lamp OFF    <- "when i turn pot light not on"
  //
  // The third line is the one that names the fault. With DIR pinned high,
  // brightness went as (MAX_PWM - level): the knob ran BACKWARDS, full off at
  // full demand, and the lamp was brightest at zero. Holding DIR low instead
  // makes brightness simply follow the PWM leg, which is what everything
  // upstream - the pot table, light_demand(), the panel readout - already
  // assumes.
  //
  // THE LEGS CANNOT BE SWAPPED, so do not try it as a fix: LIGHT_DIR is D8,
  // which has no timer behind it and cannot carry a PWM at all. The static leg
  // has to be that one, which means it has to be the LOW one.
  digitalWrite(LIGHT_DIR, LOW);

  if (level == 0) {
    // LIGHT_PWM is a timer pin, so analogWrite(pin, 0) can still emit a narrow
    // pulse every period. On a motor that is a creep; on a lamp it is a faint
    // glow that will not go out. digitalWrite is the only certain dark.
    digitalWrite(LIGHT_PWM, LOW);
  } else {
    analogWrite(LIGHT_PWM, level);
  }
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

// Print a pin the way a human reads it: "D9", or "A1" for the analog block.
// The banner used to spell the map out in literal strings, which is exactly the
// failure the banner exists to catch - on 2026-08-27 the constants moved and the
// text did not, so a correctly-flashed board reported the OLD pins and looked
// stale when it was not. Derived from the constants now, so it cannot lie.
// TRUE CHIP RESET FOR A WEDGED SHIELD - added 2026-08-27.
//
// The note further down says the only cures are hardware. That was wrong, and
// this is the missing option it did not find.
//
// The problem it describes is real: W5100::init() opens with
// `if (initialized) return 1`, so the chip gets ONE hardware reset per boot and
// every later Ethernet.begin() is a no-op. A shield that came up wedged stayed
// wedged for the whole boot - board powered, sketch running, USB fine, no
// packets - which is exactly "always power on but only not send data".
//
// But the W5100 has its own RESET BIT in the Mode Register, and this library
// exposes writeMR/readMR PUBLICLY (w5100.h, the public block at line 192).
// Writing 0x80 there resets the chip itself over SPI, which the `initialized`
// flag knows nothing about. The library's own private softReset() does exactly
// this; it is simply not reachable from a sketch.
//
// After the reset the chip is blank, so the settings init() would have written
// are re-applied by hand: MAC, IP, subnet, gateway. Socket memory (RMSR/TMSR)
// comes back at the 2KB-per-socket default, which is what this library uses.
//
// NOTE THE delay() SCALING. Timer0 runs at prescaler 1 for the 62.5 kHz PWM, so
// delay() is 64x short - MILLIS_SCALE compensates every duration this sketch
// measures, and it has to be applied here too or the poll gives up in ~300us.
static const uint8_t SHIELD_SUBNET[4] = {255, 255, 255, 0};
static const uint8_t SHIELD_GATEWAY[4] = {192, 168, 50, 1};
static const uint8_t SHIELD_IP[4] = {192, 168, 50, 20};

// DID THE CHIP ACTUALLY TAKE ITS CONFIGURATION? Added 2026-08-27.
//
// hardwareStatus() only reads a VERSION register. A W5100 that is powered but
// came up wrong still answers it, so the boot retry loop below - which was
// conditioned on `hardwareStatus() == EthernetNoHardware` - never ran even
// once. The board reported "W5100 ok" and sat deaf on the network: chip answers
// is not the same as chip works, and that gap is what "every time I power on it
// does not connect" was.
//
// Reading SIPR back is a real check. It is the address Ethernet.begin() should
// have written into the chip; if it does not read back, the configuration never
// landed and the shield needs a genuine reset rather than another no-op.
bool shieldConfigured() {
  // NOTHING BELOW IS MEANINGFUL IF THE CHIP WAS NEVER DETECTED - see
  // chipDetected(). Reading SIPR with chip==0 does not read SIPR; it sends
  // W5500-format frames at a W5100 and returns whatever falls out.
  if (!chipDetected()) return false;
  uint8_t got[4];
  W5100.readSIPR(got);
  return got[0] == SHIELD_IP[0] && got[1] == SHIELD_IP[1]
      && got[2] == SHIELD_IP[2] && got[3] == SHIELD_IP[3];
}

/* HAS THE LIBRARY ACTUALLY IDENTIFIED THE CHIP? Added 2026-08-27, and it is the
 * precondition for every other register access in this file.
 *
 * W5100Class::init() ends with `chip = 51` for a W5100, 52 for a W5200, 55 for a
 * W5500 - or `chip = 0` when detection failed. And every register access in the
 * library dispatches on that value like this:
 *
 *     if (chip == 51) { ...W5100 framing... }
 *     else if (chip == 52) { ...W5200... }
 *     else { ...W5500 framing... }        <-- chip == 0 LANDS HERE
 *
 * There is no case for 0. A failed detection therefore does not disable register
 * access, it silently switches it to the WRONG PROTOCOL: every read and write
 * afterwards is W5500 framing aimed at a W5100. Writes vanish and reads return
 * rubbish that is indistinguishable from data.
 *
 * That is what made this fault so hard to see. resetShield() would write MR.RST
 * into nothing, read a stray 0 back, and report success; shieldConfigured()
 * would compare rubbish against the expected IP. The board announced a healthy
 * shield it had never once spoken to.
 *
 * init() is the ONLY thing that can set chip back to 51, and Ethernet.begin()
 * re-runs it because its `initialized` guard is still false after a failure.
 * So: chip==0 is cured by begin(), never by a reset. */
bool chipDetected() {
  return W5100.getChip() == 51;
}

bool resetShield() {
  // A reset cannot reach a chip the library cannot address - see chipDetected().
  // Returning false here rather than going through the motions is what stops
  // this function reporting a success it never had.
  if (!chipDetected()) return false;

  W5100.writeMR(0x80);                       // RST - the chip resets itself
  for (uint8_t i = 0; i < 50; i++) {         // datasheet clears in well under 10ms
    if (W5100.readMR() == 0) break;
    delay(1UL * MILLIS_SCALE);
  }
  if (W5100.readMR() != 0) return false;     // never came out of reset
  W5100.setMACAddress(mac);
  W5100.writeSIPR(SHIELD_IP);
  W5100.writeSUBR(SHIELD_SUBNET);
  W5100.writeGAR(SHIELD_GATEWAY);
  return true;
}

void printPin(uint8_t pin) {
  if (pin >= A0) {
    Serial.print('A');
    Serial.print(pin - A0);
  } else {
    Serial.print('D');
    Serial.print(pin);
  }
}

/* WHY DID THE BOARD RESTART? Added 2026-08-27.
 *
 * Operator: "when i turn on brush to robo disconnect connect disconnect connect
 * continuosly". The ping log agrees - seventeen down/up flaps in four minutes,
 * only while the brush was running. That is either the AVR resetting or the
 * shield wedging, and the two want completely different fixes, so the board now
 * says which.
 *
 * MCUSR holds the reset source, but it must be read before the C runtime and
 * before anything else clobbers it - hence .init3, which runs ahead of the .bss
 * clear in .init4. BORF there is a BROWN-OUT: the 5V rail sagged below the
 * detector threshold and the chip reset itself. That is the signature a motor
 * inrush leaves. Optiboot may clear MCUSR before we ever see it, which is what
 * the .noinit counter below is for.
 *
 * .noinit survives a RESET but not a POWER LOSS - RAM holds its contents down to
 * roughly 1.5V while the brown-out detector trips at about 2.7V. So a magic word
 * that is still intact means the board reset with power broadly maintained (a
 * brown-out or an external reset); a garbage magic means the rail actually
 * collapsed. That distinction is the whole question here, and it does not depend
 * on the bootloader leaving MCUSR alone. */
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


  // THE BRUSH IS SILENCED FIRST, BEFORE ANYTHING ELSE IN THIS FUNCTION.
  //
  // Operator, 2026-08-27: "when bot on and off to brush motor is rotate without
  // on switch". It did, and the old ordering could not have stopped it.
  //
  // The block below writes every output to its stopped level BEFORE calling
  // pinMode, on the reasoning that a pin cannot glitch in the gap. That is true
  // for an ACTIVE-LOW load: digitalWrite(pin, HIGH) on a pin that is still an
  // input switches the internal pull-up on, which does weakly hold the line
  // high. It is NOT true for this one. BRUSH_ACTIVE_HIGH is true, so the off
  // level is LOW - and digitalWrite(pin, LOW) on an input merely turns the
  // pull-up OFF and leaves the pin floating at high impedance. Nothing holds it
  // anywhere. If the driver's enable input drifts high, the brush runs.
  //
  // Driving it as a real output is the only thing that holds it, so that
  // happens here, ahead of Serial.begin() and everything after it. pinMode
  // first is safe in this direction precisely BECAUSE the load is active-high:
  // PORTA1 powers up as 0, so the pin drives LOW the instant it becomes an
  // output. The order that is wrong for an active-low load is the right one
  // here, which is why it is stated rather than copied.
  //
  // WHAT THIS CANNOT FIX, and it matters: from the moment power arrives until
  // this line executes, the AVR is in its bootloader and EVERY pin is an input.
  // That is one to two seconds on a Uno and no sketch can shorten it. If the
  // brush still twitches at power-on, the fix is a PULL-DOWN RESISTOR (10k is
  // ample) from the driver's enable input to ground - a part that holds the
  // line while no chip is driving it. Firmware can only close the window after
  // the bootloader, not the bootloader itself.
  pinMode(BRUSH_PWM, OUTPUT);
  digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(BRUSH_DIR, OUTPUT);
  digitalWrite(BRUSH_DIR, BRUSH_DIR_LEVEL);

  // 250000 on the operator's order 2026-08-26, up from 9600 - match the serial
  // monitor to this or the log reads as garbage. ONE RATE ACROSS THE PROJECT
  // was the instruction, and this line is what makes that true: the Pi, the USB
  // sketch and this console now all read 250000, so there is no second number
  // to remember or to get wrong at 2am.
  //
  // BE CLEAR WHY THIS ONE WAS FREE TO CHANGE, because the rule is not the same
  // on both builds. Here the port is a CONSOLE: commands arrive over UDP and
  // ACKs leave the same way, so nothing on the other end has to agree with it
  // and picking a number is a preference. On the USB twin the SAME port carries
  // the commands, so its rate and the Pi's UNO_BAUD are one setting in two
  // files - change one, reflash the other, or the link is dead rather than
  // slow. Uniformity here is convenience; there it is a hard constraint.
  //
  // 250000 is also the EXACT rate for a 16 MHz AVR - UBRR=7 with U2X divides
  // evenly, where 115200 lands on 117647 (+2.12%) and spends half a UART's
  // error budget standing still. Nothing on a console depends on that, but
  // there is no reason to take the worse number when they cost the same.
  //
  // It buys headroom rather than speed: the telemetry block at the bottom of
  // loop() prints on every change, rate-limited to 200 ms, and at 9600 a long
  // line took ~90 ms of that budget. At 115200 it takes under 9 ms, so the print
  // still cannot sit in the way of a command being serviced.
  //
  // 115200 ON THE OPERATOR'S ORDER 2026-08-27, back down from 250000, and the
  // tradeoff is recorded here so nobody has to rediscover it. On a 16 MHz AVR:
  //
  //     250000 -> UBRR=7   actual 250000.0   error  0.00%
  //     115200 -> UBRR=16  actual 117647.1   error +2.12%
  //
  // 2.12% is inside the ~4% a UART tolerates, so it works - but with less margin
  // for a long cable or a warm clock than the rate it replaces.
  //
  // If the banner ever comes back as garbage, that missing margin is the first
  // thing to suspect. Every ordinary terminal handles 115200, which 250000 could
  // not always claim.
  Serial.begin(SERIAL_BAUD);
  reportResetCause();

  // Outputs are driven to a stopped state BEFORE they become outputs, so the
  // pins cannot glitch high in the gap between pinMode and the first write.
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  // BOTH rod lines must be at their stopped level before they become outputs,
  // for the same reason the brush's gate is driven first: a pin that is still an
  // input is held nowhere, and a driver whose enable drifts high will run.
  digitalWrite(ACT_DIR, ACT_LEVEL_EXTEND);
  digitalWrite(ACT_PWM, LOW);
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
  // Timer2 carries the BRUSH (D3). The matching argument above is about the two
  // wheels, which share Timer0 and therefore cannot disagree; this timer drives
  // one load and only needs to be fast. See the pin map at the top.
  TCCR2A |= _BV(WGM21);
  TCCR2B = (TCCR2B & 0b11111000) | 0b001;

  // Timer0: D6 AND D5 - BOTH WHEELS.
  // Prescaler = 1 -> about 62.5 kHz on both, which is the point: one prescaler
  // write now sets both channels, so left and right cannot drift apart.
  // WARNING: this breaks normal millis()/micros()/delay() timing - millis()
  // now runs 64x fast. MILLIS_SCALE above compensates every duration this
  // sketch measures; anything ELSE that calls millis() (the Ethernet library's
  // internal timeouts, any future delay()) is NOT compensated and will be 64x
  // short. There are no delay() calls in this sketch today, and the link runs
  // on a static IP so nothing here waits on a DHCP timeout.
  TCCR0B = (TCCR0B & 0b11111000) | 0b001;

  // Timer1: D9, the lamp. A hardware waveform far above anything the eye can
  // follow, which is what keeps a dimmed lamp free of visible flicker.
  //
  // The core leaves Timer1 in 8-bit PHASE-CORRECT with prescaler 64, which is
  // 490 Hz. Fast PWM 8-bit (WGM13:0 = 0101) with prescaler 1 gives
  //
  //     16e6 / (256 * 1) = 62500 Hz
  //
  // the same number every other driven pin on this board runs at. For the lamp
  // the frequency buys freedom from visible flicker rather than motor matching.
  // Timer1 drives no timing in this sketch - millis() and micros() are Timer0 -
  // so nothing else moves with it.
  TCCR1A = (TCCR1A & 0b11111100) | _BV(WGM10);
  TCCR1B = (TCCR1B & 0b11100000) | _BV(WGM12) | 0b001;

  // NOTE: STATUS_LED is LED_BUILTIN = D13, which is also the SPI clock the
  // shield uses. With the shield fitted this lamp tracks Ethernet traffic
  // rather than link state, and is not a reliable indicator. Harmless, but do
  // not read anything into it — and never put a real signal on D13.
  pinMode(STATUS_LED, OUTPUT);
  safeState();

  // NO SD DESELECT HERE. D4 is DIR2 and safeState() has already parked it HIGH,
  // which is the deselected level in any case. Run with the slot empty.

  // Static IP — no DHCP. Ethernet.begin(mac, ip) cannot fail or block, unlike
  // the DHCP form which stalls ~60 s when no server answers. On a point-to-point
  // tether there is no DHCP server at all, so static is the only sane choice.
  // --- LET THE 5V RAIL SETTLE BEFORE TOUCHING THE W5100 --------------------
  // Added 2026-08-26 for the cold-start failure: on barrel-jack power the board
  // comes up but the shield never answers, while on USB it always works.
  //
  // THE ONE-SHOT PROBLEM. W5100::init() begins with `if (initialized) return 1`,
  // so the chip gets exactly ONE hardware reset per boot, taken here. If the 5 V
  // rail has not settled when that happens, the shield is dead until the next
  // RESET - and no amount of calling Ethernet.begin() again will redo it. So the
  // single shot has to be aimed well rather than repeated.
  //
  // On this rig the rail is slow and weak: 12 V into the Uno's LINEAR regulator
  // means (12-5) x 0.23 A of heat it cannot shed, so it sags and recovers rather
  // than snapping up. USB feeds 5 V straight in and skips all of that, which is
  // exactly why USB "fixes" a problem that has nothing to do with data.
  //
  // A second of grace costs nothing at boot and gives the regulator time to
  // reach a steady state before the W5100 is asked to come out of reset.
  //
  // x MILLIS_SCALE because Timer0 is prescaled for D6's PWM - delay() counts in
  // the same 64x-fast milliseconds as everything else here. Without it this
  // would be a 16 ms pause, not a second.
  // 3 s, RAISED FROM 1 s 2026-08-26 on a very specific report: the board fails
  // to come up ONLY when both it and the Pi have been powered off for more than
  // five minutes. A shorter outage always works, and so does unplugging and
  // replugging power once after a failure.
  //
  // THAT IS A CAPACITOR-DISCHARGE SIGNATURE, not a logic fault. After a brief
  // power-off the bulk capacitors are still part-charged and the 5 V rail snaps
  // up; after five minutes they are flat and the rail crawls up through a
  // regulator that is already burning (12-5) x 0.23 A of heat it cannot shed.
  // The replug works for the same reason - the caps are charged from the failed
  // attempt, so the second power-up is the fast one.
  //
  // A slow rail is exactly what the W5100's one-shot init cannot survive, so
  // the single reset has to be aimed later still. 3 s is chosen to be longer
  // than any plausible RC rise on this rail rather than tuned to it; it is paid
  // once at boot and nothing is waiting on it.
  //
  // IT MAY NOT BE ENOUGH, AND THE REASON IS NOT SOFTWARE. If the AVR itself
  // starts before Vcc is valid it can come up in an undefined state and never
  // run this sketch at all - in which case no delay, retry or reset here can
  // help, because none of them execute. The way to tell the two apart is to
  // reproduce the failure and then read the serial banner: if it prints, the
  // sketch is running and the shield is the problem; if the port is silent, the
  // processor never started. Fix the rail either way.
  delay(3000UL * MILLIS_SCALE);

  Ethernet.begin(mac, ip);

  // WHAT THE SHIELD ACTUALLY REPORTS, printed every boot. This is the line that
  // separates "no power / dead chip" from "chip fine, cable out" - and it had
  // never been looked at, which is why the cold-start fault was guessed at for
  // an hour instead of read off the board.
  Serial.print(F("W5100: "));
  EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  if (hw == EthernetNoHardware) {
    Serial.print(F("NOT DETECTED (no power to the shield, or not seated)"));
  } else if (hw == EthernetW5100) {
    Serial.print(F("W5100 ok"));
  } else {
    Serial.print(F("detected, type "));
    Serial.print((int)hw);
  }
  Serial.print(F("   link: "));
  EthernetLinkStatus ls = Ethernet.linkStatus();
  Serial.println(ls == LinkON ? F("UP") : (ls == LinkOFF ? F("DOWN") : F("unknown")));

  // BRING THE SHIELD UP. Rewritten 2026-08-27 to treat the two failures that
  // look identical from outside as the different faults they are.
  //
  //   A) THE CHIP WAS NEVER DETECTED (chip==0). init() failed, so the library is
  //      addressing it with the wrong protocol - see chipDetected(). No reset can
  //      reach it. The one and only cure is another init(), which Ethernet.begin()
  //      performs because its `initialized` guard is still false after a failure.
  //
  //   B) THE CHIP IS DETECTED BUT ITS CONFIG DID NOT LAND. Registers work, so a
  //      real MR.RST reset is both possible and the right move.
  //
  // Getting these backwards is why this took so long: a reset aimed at case A
  // does nothing and reports success, which reads as "the shield is fine" while
  // the board sits deaf. Each pass re-tests rather than assuming, and the delay
  // between passes is more rail-settling time on top of the 3 s already spent.
  for (uint8_t tries = 1; tries <= 8; tries++) {
    if (!chipDetected()) {
      Ethernet.begin(mac, ip);            // case A - the only thing that helps
    } else if (!shieldConfigured()) {
      resetShield();                      // case B - reachable, so reset it
      Ethernet.begin(mac, ip);
    } else {
      break;                              // detected AND configured
    }
    Serial.print(F("W5100 try "));
    Serial.print(tries);
    Serial.print(F(": chip="));
    Serial.print(W5100.getChip());
    Serial.println(shieldConfigured() ? F(" configured") : F(" not yet"));
    delay(400UL * MILLIS_SCALE);
  }

  // chip= IS THE MOST DIAGNOSTIC NUMBER THIS BOARD PRINTS. 51 means the library
  // is talking W5100 framing to a W5100 and everything downstream can be
  // believed. 0 means detection failed, every later register value is fiction,
  // and the fault is the rail or the shield - not this sketch.
  Serial.print(F("W5100: chip="));
  Serial.print(W5100.getChip());
  Serial.println(shieldConfigured() ? F(" config VERIFIED (IP reads back)")
                                    : F(" NOT VERIFIED - link will not work"));

  udp.begin(LISTEN_PORT);

  // NO BOOT-TIME PACKET PROBE HERE ANY MORE, and it must not come back.
  //
  // Added earlier on 2026-08-27, removed the same day. It waited up to 3 s for a
  // packet and reset the chip if none came, six times over - reasoning that the
  // Pi transmits at 50 Hz so silence must mean a deaf shield.
  //
  // THAT REASONING FAILS AT EXACTLY THE MOMENT THAT MATTERS. When the operator
  // powers the whole rig on, this board is listening about five seconds later
  // while the Pi is still most of a minute from booting Linux and starting the
  // ground station. There is nothing to hear yet and nothing wrong. The probe
  // read that normal silence as a fault and fired six chip resets into a
  // perfectly healthy shield, then handed over to a loop() path that kept
  // resetting every five seconds until the Pi finally appeared.
  //
  // The operator's report - "turn off pi and uno 10-15 min, after start its not
  // connected, 3-4 times on off uno and it works" - is that behaviour exactly: a
  // cold start where both come up together fails, while power-cycling the Uno
  // ALONE succeeds, because by then the Pi is already transmitting.
  //
  // A board must be able to come up on its own. The chip is verified against its
  // own registers above, which needs nobody else to be awake; the wedge the
  // probe was aimed at is handled in loop(), where waiting costs nothing.
  // Seeded so the watchdog measures from BOOT, not from zero - otherwise it
  // fires on the first pass, before the Pi has had a chance to send anything.
  lastUdpMs = millis();
  lastEthTryMs = millis();

  Serial.print(F("uno_eth_link: ETHERNET ONLY (serial commands off). UDP on "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.println(LISTEN_PORT);
  Serial.print(F("DIR1="));  printPin(DIR1);
  Serial.print(F(" PWM1=")); printPin(PWM1);
  Serial.print(F(" (left)  DIR2=")); printPin(DIR2);
  Serial.print(F(" PWM2=")); printPin(PWM2);
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
  // Printed as the truth table rather than as two bare pin numbers: the failure
  // this channel is prone to is a WIRING SCHEME misread, which two correct pin
  // numbers would not reveal.
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
  // about looks like a flaky cable: with a card in the slot the link dies while
  // the right wheel is driven in the LOW direction. Printed from DIR2 rather
  // than typed out, so it cannot go stale if that pin ever moves.
  Serial.print(F("  ^ "));
  printPin(DIR2);
  Serial.println(F(" is the shield's SD chip select - RUN WITH THE SLOT EMPTY"));
  // The build with pot speed control announces itself: an ACK-identical old
  // build is otherwise indistinguishable over the LAN (see the banner note
  // above), and "held 255" vs "soft-PWM" is exactly the difference that
  // decides whether the knob does anything.
  Serial.print(F("BRUSH_DIR=")); printPin(BRUSH_DIR);
  Serial.print(F(" BRUSH_PWM=")); printPin(BRUSH_PWM);
  Serial.print(F(" hw-PWM 62.5kHz duty 0-"));
  Serial.print(MAX_PWM);
  Serial.print(F(" floor "));
  Serial.print(BRUSH_MIN_DUTY);
  Serial.println(F(" (TOGGLE on/off, Pi sends 0 or 255)"));
  Serial.print(F("LIGHT_DIR="));  printPin(LIGHT_DIR);
  Serial.print(F(" LIGHT_PWM=")); printPin(LIGHT_PWM);
  Serial.println(F(" hw-PWM 62.5kHz (pot-dimmed, 0-255)"));
  Serial.print(F("failsafe after "));
  Serial.print(FAILSAFE_MS);
  Serial.println(F(" ms of silence"));
}

/* ---------------------------------------------------------------------------
 * DUAL TRANSPORT: this build listens on the Ethernet shield AND the USB serial
 * port at the same time, on the operator's instruction 2026-08-26.
 *
 * WHY BOTH IS WORTH THE CODE ON THIS RIG. Neither link has been reliable: the
 * Ethernet carrier flapped 17 times in nine minutes on 2026-08-25, and the USB
 * device dropped off the Pi six times on 2026-08-26. They fail for completely
 * unrelated reasons - one is a cable and a shield, the other is a bus and a
 * power rail - so the chance of both being down at the same instant is far
 * smaller than either alone. Whichever is alive drives the robot.
 *
 * THE FAILSAFE IS SHARED, AND THAT IS THE POINT. lastPacketMs is refreshed by a
 * command from EITHER source, so the 300 ms timeout only trips when BOTH have
 * gone quiet. A link dying while the other is talking is now a non-event.
 *
 * THE ACK GOES BACK THE WAY THE COMMAND CAME. The Pi judges link health from
 * ACKs, and it is only ever using one transport at a time - answering on the
 * wrong one would read as total loss on the transport it is actually watching.
 * --------------------------------------------------------------------------- */

// Serial framing state. UDP delivers whole datagrams with their own boundaries;
// a serial stream has none, so the newline in "CMD <seq> ...\n" is the frame
// marker and this holds the partial line between passes.
uint16_t rxLen = 0;
bool rxOverflow = false;

bool pumpSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      if (rxOverflow) {
        rxLen = 0;
        rxOverflow = false;
        continue;
      }
      packet[rxLen] = '\0';
      rxLen = 0;
      if (packet[0] != '\0') {
        return true;
      }
      continue;
    }
    if (rxLen >= RX_BUFFER - 1) {
      // Swallow an over-long line to its newline rather than parse a truncated
      // one: "CMD 7 M 200 2" is a VALID parse of a truncated "CMD 7 M 200 250".
      rxOverflow = true;
      continue;
    }
    packet[rxLen++] = c;
  }
  return false;
}

/* Parse and act on whatever is in `packet`. Returns true if it was understood.
 *
 * Extracted from loop() so the UDP and serial paths run the SAME code - two
 * copies of this ladder would drift, and the one that drifted would be the one
 * nobody was testing that week. */
bool handleCommand(unsigned int *seqOut) {
  unsigned int seq = 0;
  int left = 0, right = 0, jx = 0, jy = 0, act = 0, brush = 0, light = 0;
  bool understood = false;

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
    curL = left; curR = right; curA = act; curB = brush; curLight = light;
    understood = true;
  } else if (sscanf(packet, "CMD %u J %d %d", &seq, &jx, &jy) == 3) {
    mixJoystick(jx, jy, &left, &right);
    applyMotor(DIR1, PWM1, left, INVERT_1);
    applyMotor(DIR2, PWM2, right, INVERT_2);
    applyActuator(0, INVERT_ACT);
    applyBrush(0);
    applyLight(0);
    curL = left; curR = right; curA = 0; curB = 0; curLight = 0;
    understood = true;
  } else if (sscanf(packet, "CMD %u STOP", &seq) == 1) {
    curL = 0; curR = 0; curA = 0; curB = 0; curLight = 0;
    safeState();
    understood = true;
  } else if (sscanf(packet, "CMD %u", &seq) == 1) {
    understood = true;                 // bare keepalive - refreshes the failsafe
  }

  if (understood) {
    *seqOut = seq;
    lastSeq = (uint16_t)seq;
    packetsReceived++;
    lastPacketMs = millis();           // shared by both transports - see above
    if (!linkUp) {
      linkUp = true;
    }
    digitalWrite(STATUS_LED, HIGH);
  }
  return understood;
}

void loop() {
  // First thing every pass: the rod's 50% stage and the brush's pot-set speed
  // are both synthesised in software, so the more often these run the cleaner
  // their duty. Everything else in this loop is either instant or rate-limited.
  serviceActuatorPwm();
  serviceBrushPwm();

  // --- ETHERNET ------------------------------------------------------------
  int size = udp.parsePacket();
  if (size > 0) {
    int n = udp.read(packet, RX_BUFFER - 1);
    if (n < 0) {
      n = 0;
    }
    packet[n] = '\0';
    // Anything longer than the buffer is still queued in the W5x00; drop the
    // remainder so the next parsePacket() starts on a clean packet boundary.
    if (size > n) {
      udp.flush();
    }
    lastUdpMs = millis();          // proof the shield is alive - see ETH_REINIT_MS
    everHeard = true;              // first contact: silence now means a wedge
    unsigned int seq = 0;
    if (handleCommand(&seq)) {
      // Straight back to whoever sent it - see the DUAL TRANSPORT note.
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(F("ACK "));
      udp.print(lastSeq);
      udp.print(F("\n"));
      udp.endPacket();
    }
  }

  // --- USB SERIAL ----------------------------------------------------------
  // Off by default - see SERIAL_COMMANDS. The port is drained either way so a
  // terminal typing at it cannot silently fill the RX buffer and stall the
  // Ethernet side; the bytes are simply discarded rather than obeyed.
  if (pumpSerial() && SERIAL_COMMANDS) {
    unsigned int seq = 0;
    if (handleCommand(&seq)) {
      Serial.print(F("ACK "));
      Serial.println(lastSeq);
    }
  }

  // --- shield watchdog -----------------------------------------------------
  // See ETH_REINIT_MS. Deliberately independent of linkUp and of the serial
  // side: the state it recovers is a shield that NEVER came up, which nothing
  // else in this sketch would ever notice.
  {
    unsigned long nowE = millis();
    // PATIENT BEFORE FIRST CONTACT, PROMPT AFTER IT - see everHeard. A rig
    // powered on all at once leaves this board waiting on a Pi that is still
    // booting; that silence is normal and must not be treated as a fault.
    const unsigned long quietFor = everHeard ? ETH_REINIT_MS : ETH_COLD_WAIT_MS;
    if (nowE - lastUdpMs > quietFor && nowE - lastEthTryMs > quietFor) {
      lastEthTryMs = nowE;

      // RESET THE CHIP ITSELF, not just its configuration.
      //
      // THE FAULT THIS FIXES, reported 2026-08-26: after everything is
      // restarted the Uno does not come back, but unplugging its power ONCE and
      // plugging it in again fixes it every time. That difference is the whole
      // clue - a power CYCLE takes the 5 V rail to zero and gives the W5100a
      // true power-on reset, while a dip or a warm restart leaves the AVR
      // running and the shield in an undefined state with nothing able to clear
      // it. The settle delay in setup() only helps a CLEAN cold start.
      //
      // The library cannot do this: W5100::init() returns early once
      // `initialized` is set, so Ethernet.begin() reprograms the MAC and IP into
      // a chip that was never reset. Writing MR.RST (bit 7 of the Mode Register)
      // over SPI is the same reset the chip performs at power-on, and it is
      // reachable directly.
      //
      // ORDER MATTERS: reset the silicon, let it come up, THEN reprogram it, and
      // only then take a socket. Doing begin() first would push the config into
      // a chip that is about to be wiped.
      // RELEASE THE SOCKET BEFORE REOPENING IT. This one line is the whole fix
      // for "the Uno answers ping but the ground station never reconnects".
      //
      // MEASURED 2026-08-26: after the Pi's eth0 went down and came back, the
      // W5100 kept answering ICMP - its IP stack is in hardware and never
      // stopped - while its UDP SOCKET stayed deaf. tcpdump showed the Pi
      // sending at 50 Hz and not one reply coming back, indefinitely.
      //
      // WHY THE OLD RE-INIT DID NOT CLEAR IT. EthernetUDP::begin() looks for a
      // FREE socket. The wedged one was still allocated, so begin() found
      // nothing, returned 0, and did nothing at all - every 5 seconds, forever.
      // It looked like a working retry and was a no-op.
      //
      // stop() closes the socket and hands it back, so the begin() below gets a
      // clean one. Harmless when the link is healthy: this whole block only runs
      // after ETH_REINIT_MS of total silence, which a working link never reaches.
      udp.stop();

        // SAME TWO FAULTS AS AT BOOT, same two cures - see chipDetected().
        // The MR.RST write that used to sit above this block is gone: it was a
        // second reset on top of the one resetShield() performs, and on the
        // chip==0 path it was writing W5500 frames at a W5100 for no reason.
        if (!chipDetected()) {
          Ethernet.begin(mac, ip);        // only init() can recover chip==0
          udp.begin(LISTEN_PORT);
          Serial.print(F("LINK SILENT - chip undetected, init retried: chip="));
          Serial.println(W5100.getChip());
        } else {
          bool wasReset = resetShield();
          Ethernet.begin(mac, ip);
          udp.begin(LISTEN_PORT);
          Serial.print(F("LINK SILENT - shield reset "));
          Serial.print(wasReset ? F("OK, socket reopened") : F("FAILED"));
          Serial.println(shieldConfigured() ? F(", configured")
                                            : F(", NOT configured"));
        }

      // NO WATCHDOG RESET HERE, AND IT MUST NOT COME BACK IN THIS FORM.
      //
      // Added 2026-08-26 to force the hardware reset that Ethernet.begin() does
      // not perform, and REMOVED THE SAME HOUR: it put the board in a RESET
      // LOOP - 6 boots in 35 seconds, banners truncated mid-print. On this AVR a
      // watchdog reset leaves the timer ARMED with its short timeout, and the
      // bootloader takes longer than that to hand over, so the board resets
      // again before it ever reaches setup(). Clearing MCUSR and calling
      // wdt_disable() first does not help - execution never gets that far.
      //
      // THE CURE WAS WORSE THAN THE DISEASE. A wedged shield still leaves a
      // working board on USB; a reset loop kills every transport at once, which
      // is exactly what "now it is not connected via usb either" was.
      //
      // IF THE SHIELD WEDGE NEEDS SOLVING, the honest options are hardware:
      // wire a spare pin to the shield's RESET line and pulse it, or fix the 5V
      // rail so the W5100 comes up cleanly at all - which on this rig is the
      // real root cause (12V through the Uno's linear regulator browns the
      // shield out). Do not reach for the watchdog again.
      // Not announced on Serial: on this build the serial port is a COMMAND
      // transport too, and a warning would sit in the TX buffer competing with
      // ACKs - the exact stall that made the lamp flicker earlier today.
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

  // Pause per pass - LOOP_PAUSE_US, 10 ms real since 2026-08-27, raised from a
  // value that was meant to be 5 ms and was really 78 us. See the constant for
  // both the scaling bug and why this is a busy-wait rather than delay().
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
  }
}
