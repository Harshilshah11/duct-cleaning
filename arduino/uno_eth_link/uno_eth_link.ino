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
 * because only the Pi's brush_demand() had to change. A1 has no timer, so a
 * mid-range duty would be synthesised in software — see serviceBrushPwm(), the
 * same mechanism the rod uses on D4; at 0 and 255 it resolves to a static
 * level and costs nothing.
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
 *   channel 1 / LEFT    DIR1 = D9        PWM1 = D3    (Timer2)
 *   channel 2 / RIGHT   DIR2 = D8        PWM2 = D6    (Timer0)
 *   linear actuator     ACT_DIR = D7     ACT_PWM = D4  (LOW extends! see below)
 *   brush motor         BRUSH_DIR = D2   BRUSH_PWM = A1 (held 255, on/off)
 *   light               LIGHT_DIR = A0   LIGHT_PWM = D5 (Timer0)
 *
 * EVERY PIN IS NOW SPOKEN FOR. D0/D1 are the USB serial this telemetry goes out
 * on, D10-D13 belong to the shield, and all four of the Uno's usable PWM pins
 * (D3, D5, D6, D9) are allocated — D9 to a direction line, which is the one
 * place a PWM pin is still spendable if something else ever needs dimming.
 * A1 is the brush's speed line, a static HIGH that needs no timer; A2-A5 are
 * spare plain digital I/O since the rod's enable moved off A2 to D4.
 *
 * D4 IS THE SHIELD'S microSD CHIP SELECT AND THE ROD NOW OWNS IT — see the
 * ACT_DIR/ACT_PWM block below. RUN THIS BOARD WITH THE microSD SLOT EMPTY.
 *
 * REWIRED 2026-08-14: D7 is freed and the old DIR1=D7 / PWM1=D9 / PWM2=D3 map
 * is gone. DIR1 took over D9 (a direction line only needs digitalWrite, so
 * spending a PWM-capable pin on it is fine), PWM1 moved to D3, and PWM2 moved
 * to the newly used D6.
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
const uint8_t DIR1 = 9;    // channel 1 direction (LEFT)
const uint8_t PWM1 = 3;    // channel 1 speed, Timer2  (~490 Hz)
const uint8_t DIR2 = 8;    // channel 2 direction (RIGHT)
const uint8_t PWM2 = 6;    // channel 2 speed, Timer0  (~980 Hz, see pin note)

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
const uint8_t ACT_DIR = 7;   // "Dir" on the rig — LOW extends, HIGH retracts
const uint8_t ACT_PWM = 4;   // "Pwm" on the rig — the only line that gates it

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
// A0 is an ANALOG pin driven as a plain digital output, which is legal on the
// Uno (A0 == D14) and is what makes this fit at all: every real digital pin is
// spoken for. It carries the driver channel's direction line, which a lamp does
// not actually need — see applyLight().
const uint8_t LIGHT_DIR = A0;
const uint8_t LIGHT_PWM = 5;   // Timer0 (~980 Hz) — same 0-duty caveat as D6

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
// with the light) now sets the brush duty, 0..255 in the <brush> field. A1 has
// no timer behind it, so the duty is SOFT-PWM — serviceBrushPwm() below chops
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
// Pwm -> A1, driven from the panel TOGGLE on Pi GPIO13. This is not an inference
// from D7 having been taken — the wire really is on D2, and the sketch was the
// thing that was wrong.
const uint8_t BRUSH_DIR = 2;
const uint8_t BRUSH_PWM = A1;

// The brush spins one way only, so its direction is a constant rather than a
// demand. Flip this if the brush runs backwards.
const bool BRUSH_DIR_LEVEL = HIGH;

// Many driver and relay inputs are ACTIVE-LOW — the channel enables when the pin
// goes low, and such a board will run the brush the whole time the Uno is in
// reset if this is wrong. Set false for those. Applies to BRUSH_PWM, the line
// that actually gates the motor — including the soft-PWM's on-phase, which
// serviceBrushPwm() inverts through this same constant.
const bool BRUSH_ACTIVE_HIGH = true;

// The duty currently demanded on A1, 0..255. Written by applyBrush(), acted on
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
  brushDuty = duty;
  // The static endpoints are written HERE as well as in serviceBrushPwm(), so
  // a stop takes effect on this very line rather than on the next loop() pass.
  if (duty <= 0) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
  } else if (duty >= MAX_PWM) {
    digitalWrite(BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
  }
}

/* Software PWM for the brush, because A1 has no timer. Same contract as
 * serviceActuatorPwm(): called every pass of loop(), static levels for the 0
 * and 255 endpoints so neither OFF nor full speed can be caught mid-cycle, and
 * only the middle range is chopped — where a stalled loop costs a slower
 * brush, never a runaway one. Shares ACT_PWM_PERIOD_US; both mechanisms are
 * far too slow mechanically to care about 250 Hz ripple. */
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
  // THE LEGS CANNOT BE SWAPPED, so do not try it as a fix: LIGHT_DIR is A0,
  // which has no timer behind it and cannot carry a PWM at all. The static leg
  // has to be that one, which means it has to be the LOW one.
  digitalWrite(LIGHT_DIR, LOW);

  if (level == 0) {
    // D5 is a Timer0 pin, so analogWrite(pin, 0) can still emit a narrow pulse
    // every period. On a motor that is a creep; on a lamp it is a faint glow
    // that will not go out. digitalWrite is the only certain dark.
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

void setup() {
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
  // line took ~90 ms of that budget. At 250000 it takes under 4 ms, so the
  // print can no longer sit in the way of a command being serviced.
  //
  // YOUR SERIAL MONITOR MUST SUPPORT 250000 or the banner reads as garbage -
  // the Arduino IDE offers it, `screen /dev/ttyACM0 250000` takes it, and some
  // older terminal programs stop at 115200.
  Serial.begin(250000);

  // Outputs are driven to a stopped state BEFORE they become outputs, so the
  // pins cannot glitch high in the gap between pinMode and the first write.
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  // BOTH rod lines must be at their stopped level before they become outputs —
  // the same reasoning as the brush's A1 below. This is also what replaced the
  // old "park pin 4 HIGH to deselect the SD card" line: HIGH on D4 is the rod's
  // RETRACT drive, and that park is why the rod ran from reset forever.
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
  // A0 as a digital output. pinMode(A0, OUTPUT) is the whole ceremony — nothing
  // else is needed to stop it being an ADC input.
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
  TCCR2A |= _BV(WGM21);
  TCCR2B = (TCCR2B & 0b11111000) | 0b001;

  // Timer0: D6
  // Prescaler = 1 -> about 62.5 kHz
  // WARNING: this breaks normal millis()/micros()/delay() timing - millis()
  // now runs 64x fast. MILLIS_SCALE above compensates every duration this
  // sketch measures; anything ELSE that calls millis() (the Ethernet library's
  // internal timeouts, any future delay()) is NOT compensated and will be 64x
  // short. There are no delay() calls in this sketch today, and the link runs
  // on a static IP so nothing here waits on a DHCP timeout.
  TCCR0B = (TCCR0B & 0b11111000) | 0b001;

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

  // RETRY THE INIT while the chip reports absent. Each pass is a fresh
  // Ethernet.begin(), which is a no-op on an already-initialised chip but the
  // only thing worth trying if init() never got a working chip in the first
  // place - and the delay between passes is more rail-settling time.
  for (uint8_t tries = 0; tries < 5 &&
       Ethernet.hardwareStatus() == EthernetNoHardware; tries++) {
    delay(300UL * MILLIS_SCALE);
    Ethernet.begin(mac, ip);
    Serial.print(F("W5100 retry "));
    Serial.print(tries + 1);
    Serial.println(Ethernet.hardwareStatus() == EthernetNoHardware
                   ? F(": still absent") : F(": DETECTED"));
  }

  udp.begin(LISTEN_PORT);
  // Seeded so the watchdog measures from BOOT, not from zero - otherwise it
  // fires on the first pass, before the Pi has had a chance to send anything.
  lastUdpMs = millis();
  lastEthTryMs = millis();

  Serial.print(F("uno_eth_link: ETHERNET ONLY (serial commands off). UDP on "));
  Serial.print(Ethernet.localIP());
  Serial.print(F(":"));
  Serial.println(LISTEN_PORT);
  Serial.println(F("DIR1=D9 PWM1=D3 (left)  DIR2=D8 PWM2=D6 (right)"));
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
  Serial.println(F("ACT_DIR=D7 ACT_PWM=D4 - LOW on D7 EXTENDS (opposite the wheels)"));
  Serial.print(F("  soft-PWM stages 0/"));
  Serial.print(ACT_DUTY_RETRACT);
  Serial.print(F("/"));
  Serial.print(ACT_DUTY_EXTEND);
  Serial.println(F(" (stop/retract/extend)"));
  // Loud, and in the banner rather than a comment, because the failure it warns
  // about looks like a flaky cable: with a card in the slot the link dies except
  // while retracting. A reset is the one moment someone is watching.
  Serial.println(F("  ^ D4 is the shield's SD chip select - RUN WITH THE SLOT EMPTY"));
  // The build with pot speed control announces itself: an ACK-identical old
  // build is otherwise indistinguishable over the LAN (see the banner note
  // above), and "held 255" vs "soft-PWM" is exactly the difference that
  // decides whether the knob does anything.
  Serial.print(F("BRUSH_DIR=D2 BRUSH_PWM=A1 soft-PWM duty 0-"));
  Serial.print(MAX_PWM);
  Serial.print(F(" floor "));
  Serial.print(BRUSH_MIN_DUTY);
  Serial.println(F(" (TOGGLE on/off, Pi sends 0 or 255)"));
  Serial.println(F("LIGHT_DIR=A0 LIGHT_PWM=D5 (pot-dimmed, 0-255)"));
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
    if (nowE - lastUdpMs > ETH_REINIT_MS && nowE - lastEthTryMs > ETH_REINIT_MS) {
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
      W5100.writeMR(0x80);              // MR.RST - W5100 software reset
      delay(50UL * MILLIS_SCALE);       // datasheet needs only microseconds

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

      // Then the config, in case that was what was lost.
      Ethernet.begin(mac, ip);
      udp.begin(LISTEN_PORT);

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
  while (micros() - pauseStart < 5000UL) {
    serviceActuatorPwm();
    serviceBrushPwm();
  }
}
