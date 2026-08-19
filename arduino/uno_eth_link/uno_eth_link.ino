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
const unsigned long ACT_PWM_PERIOD_US = 4000;

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
const bool INVERT_2 = false;

// --- Failsafe ----------------------------------------------------------------
// 300 ms: long enough to ride out a handful of dropped datagrams at the 50 Hz
// command rate (15 in a row), short enough that the robot stops within a third
// of a second of a real tether failure.
const unsigned long FAILSAFE_MS = 300;

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

  digitalWrite(LIGHT_DIR, HIGH);
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
  Serial.begin(115200); // 115200 on the operator's order 2026-08-19 - match the
                        // serial monitor to this or the log reads as garbage.

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
  Ethernet.begin(mac, ip);

  // Report hardware trouble rather than sitting mute.
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("ERROR: no Ethernet shield detected - check it is seated"));
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("WARN: Ethernet cable is not connected"));
  }

  udp.begin(LISTEN_PORT);

  Serial.print(F("uno_eth_link listening on "));
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

void loop() {
  // First thing every pass: the rod's 50% stage and the brush's pot-set speed
  // are both synthesised in software, so the more often these run the cleaner
  // their duty. Everything else in this loop is either instant or rate-limited.
  serviceActuatorPwm();
  serviceBrushPwm();

  int size = udp.parsePacket();
  if (size > 0) {
    int n = udp.read(packet, RX_BUFFER - 1);
    if (n < 0) {
      n = 0;
    }
    packet[n] = '\0';

    // Anything longer than the buffer is still queued in the W5x00; drop the
    // remainder so the next parsePacket() starts on a clean packet boundary
    // instead of returning the tail as a bogus command.
    if (size > n) {
      udp.flush();
      Serial.print(F("WARN: packet truncated, "));
      Serial.print(size);
      Serial.println(F(" bytes"));
    }

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

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(F("ACK "));
      udp.print(lastSeq);
      udp.print(F("\n"));
      udp.endPacket();
    } else {
      Serial.print(F("WARN: unparsable packet: "));
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
  unsigned long now = millis();
  bool changed = (curL != printedL || curR != printedR || curA != printedA
                  || curB != printedB || curLight != printedLight)
                 && (now - lastPrintMs) > 200;
  if (changed || (now - lastPrintMs) > 3000) {
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

  // 5 ms pause per pass, on the operator's order 2026-08-18. Costs: the loop
  // tops out near 200 Hz, so the rod's and brush's synthesised PWM stages get
  // 5 ms duty granularity, and a command can wait up to 5 ms extra in the
  // W5x00 before it is read - both well inside the 50 Hz command period and
  // the 300 ms failsafe. Remove this line to restore the free-running loop.
  delay(5);
}
