#pragma once
#include <Arduino.h>

/*
 * Config.h — every pin, every tuning number, and the transport switch.
 *
 * THESE VALUES ARE READ OFF THE RUNNING BOARD. Each one was paid for with a
 * debugging session, so the reasoning stays attached to the value it justifies.
 * Read the comment before changing the constant.
 *
 * A NOTE ON THE OLD src/ CLASS LIBRARY, deleted 2026-08-29 when this folder was
 * consolidated. It carried a confident, fully-commented Config.h whose every
 * tuned value was WRONG against the board: MIN_DUTY 0 (really 90), DEADBAND 4
 * (really 12), INVERT_2 false (really true), baud 250000 (really 115200), and
 * every duration unscaled — see MILLIS_SCALE below, which it did not have at
 * all. Adopting it would have given a robot whose failsafe fired every 5 ms and
 * whose right wheel ran backwards. It is in git history at 3db5676 if the class
 * decomposition is ever wanted; the numbers in it are not.
 */

// ===========================================================================
// TRANSPORT — pick one, reflash
// ===========================================================================
// The only difference between what used to be three sketches. Everything below
// this block is shared, which is the point of consolidating them: the pin map,
// the failsafe, the brush fix and the soft-PWM were drifting between copies,
// and the copy that drifted was always the one nobody was testing that week.
//
//   LINK_ETHERNET — the rig. Commands arrive as UDP on 192.168.50.20:5005 and
//                   the USB port is a console only (see SERIAL_COMMANDS).
//   LINK_SERIAL   — the bench. Commands arrive on the USB tether at
//                   SERIAL_BAUD, ACKs go back the same way, no shield needed.
//
// The Pi is pinned to UDP (UNO_TRANSPORT in uno_motors.py), so LINK_ETHERNET is
// the build that matches the ground station today.
#define LINK_ETHERNET 1
#define LINK_SERIAL   2

#define LINK_TRANSPORT LINK_ETHERNET

// ===========================================================================
// MILLIS_SCALE — READ THIS BEFORE WRITING ANY DURATION IN THIS PROJECT
// ===========================================================================
// setup() puts Timer0 on prescaler 1 to get 62.5 kHz PWM on the wheels. Timer0
// is also what drives millis(), micros() and delay(), so ALL THREE RUN 64x
// FAST. A bare `300` is not 300 ms, it is 4.7 ms.
//
// THIS HAS ALREADY BITTEN TWICE, which is why it is a macro now rather than a
// convention. The loop pause once read `micros() - start < 5000UL`, unscaled —
// 78 REAL microseconds, so the loop was never paced at all and hammered the
// W5100 with back-to-back SPI. Meanwhile the actuator period directly beside it
// WAS scaled, so one was right and one was wrong in the same screenful.
//
// Every duration below goes through REAL_MS() or REAL_US(). Anything that does
// not is a bug. Library code that calls millis() internally (the Ethernet
// stack's timeouts) is NOT compensated and runs 64x short — which is survivable
// only because this build uses a static IP and never waits on DHCP.
//
// If Timer0 is ever put back to its stock prescaler, set MILLIS_SCALE to 1 and
// nothing else changes.
const unsigned long MILLIS_SCALE = 64;
#define REAL_MS(x) ((unsigned long)(x) * MILLIS_SCALE)
#define REAL_US(x) ((unsigned long)(x) * MILLIS_SCALE)

// ===========================================================================
// SERIAL
// ===========================================================================
// 115200 on the operator's order 2026-08-27, down from 250000. On a 16 MHz AVR:
//
//     250000 -> UBRR=7   actual 250000.0   error  0.00%
//     115200 -> UBRR=16  actual 117647.1   error +2.12%
//
// 2.12% is inside the ~4% a UART tolerates, so it works — with less margin for
// a long cable or a warm clock. If the banner ever comes back as garbage, that
// missing margin is the first thing to suspect.
//
// ON THE ETHERNET BUILD this port is a CONSOLE and the number is a preference.
// ON THE SERIAL BUILD it carries the commands, so it and the Pi's UNO_BAUD are
// one setting living in two files — change one, reflash the other, or the link
// is dead rather than slow.
const unsigned long SERIAL_BAUD = 115200;

// ===========================================================================
// NETWORK — why .50.20 and not .1.20
// ===========================================================================
// This board must NOT be 192.168.1.20. That is the ground station Pi's own
// wlan0 address, and a host always delivers traffic for its own address
// locally, so every packet the Pi sent would be answered by the Pi itself and
// never reach the wire. The eth0 segment is 192.168.50.0/24.
//
// The MAC is locally administered (the 0x02 bit) and must be unique on the LAN.
const uint16_t NET_LISTEN_PORT = 5005;
extern byte      NET_MAC[6];
extern const uint8_t NET_IP[4];
extern const uint8_t NET_SUBNET[4];
extern const uint8_t NET_GATEWAY[4];

// ===========================================================================
// MOTOR PIN MAP — chosen around the shield, NOT freely
// ===========================================================================
//
//   channel 1 / LEFT    DIR1 = D7        PWM1 = D6    (Timer0 OC0A, 62.5 kHz)
//   channel 2 / RIGHT   DIR2 = D4        PWM2 = D5    (Timer0 OC0B, 62.5 kHz)
//   linear actuator     ACT_DIR = A3     ACT_PWM = A2  (LOW on A3 extends!)
//   brush motor         BRUSH_DIR = D2   BRUSH_PWM = D3 (Timer2 OC2B, 62.5 kHz)
//   light               LIGHT_DIR = D8   LIGHT_PWM = D9 (Timer1 OC1A, 62.5 kHz)
//
// ONE TIMER PER JOB:
//   Timer0 (D5+D6) — BOTH wheels, so they share a frequency by construction
//   Timer1 (D9)    — the lamp
//   Timer2 (D3)    — the brush
// The rod is soft-PWM on A2/A3 and needs no timer, which is what frees them.
//
// D4 IS THE SHIELD'S microSD CHIP SELECT AND THE RIGHT WHEEL'S DIRECTION LINE
// OWNS IT. It goes LOW whenever that wheel is driven negative, and LOW selects
// a card. RUN WITH THE SLOT EMPTY — a selected card drives MISO during the SPI
// reads this sketch makes to the W5100 on every pass of loop(), corrupting
// them. The symptom is an Ethernet link that dies when the rod stops, which
// reads as a power fault and is not one. There is no software fix.
//
// The obvious map (PWM on D10/D11) is IMPOSSIBLE with this shield fitted: the
// W5100/W5500 owns D10 (CS), D11 (MOSI), D12 (MISO) and D13 (SCK), plus D4 for
// the microSD slot.
//
// D6 IS ON TIMER0, which also generates millis(). analogWrite() on D6 does not
// disturb millis() — only changing the prescaler does, and setup() does exactly
// that (see MILLIS_SCALE). What it DOES mean is that analogWrite(pin, 0) can
// still emit a narrow pulse and leave a motor creeping, so every full stop on a
// PWM pin goes through digitalWrite(pin, LOW). Removing that reintroduces a
// robot that will not quite stop.
const uint8_t PIN_DIR1 = 7;    // channel 1 direction (LEFT)
const uint8_t PIN_PWM1 = 6;    // channel 1 speed, Timer0 OC0A
const uint8_t PIN_DIR2 = 4;    // channel 2 direction (RIGHT) — SD CS, slot empty
const uint8_t PIN_PWM2 = 5;    // channel 2 speed, Timer0 OC0B

// Flip either of these if a wheel spins the wrong way. Doing it here is far
// safer than negating on the Pi — the Pi feeds BOTH transports, so a sign flip
// there would silently desync them.
//
// INVERT_2 IS true ON THIS RIG. That is measured, not a default.
const bool INVERT_1 = false;
const bool INVERT_2 = true;

// ===========================================================================
// LINEAR ACTUATOR — A3 picks direction, A2 gates it
// ===========================================================================
// Direction arrives from the ground station's 3-position switch as one signed
// number: positive extends, negative retracts, zero STOPS.
//
// MEASURED ON THE RIG — these are the levels the driver actually wants:
//
//     EXTEND    A3 = LOW     A2 = driven
//     STOP      A3 = held    A2 = LOW      -> rod holds position
//     RETRACT   A3 = HIGH    A2 = driven
//
// EXTEND IS DIR **LOW**. Every other direction line on this board treats HIGH
// as forward, so the natural assumption is wrong here.
//
// A2 IS THE ONLY THING THAT GATES THE ROD. A3 selects a direction but does not
// start or stop anything, so STOP is A2 LOW with A3 left wherever it was.
//
// A2/A3 carry no timer, no SPI and nothing the W5100 wants, so nothing the rod
// does can disturb the Ethernet link.
const uint8_t PIN_ACT_DIR = A3;
const uint8_t PIN_ACT_PWM = A2;

const uint8_t ACT_LEVEL_EXTEND  = LOW;
const uint8_t ACT_LEVEL_RETRACT = HIGH;

// RETRACT RAISED 128 -> 255 on 2026-08-14. At 128 the rod moved on extend and
// did nothing on retract — the signature of a stage that cannot break away: 50%
// is about where a loaded actuator stalls, and lifting against gravity runs out
// of torque first. Put it back to 128 for a slow stage, but only once the rod
// is known to travel BOTH ways at full scale.
const int ACT_DUTY_STOP    = 0;
const int ACT_DUTY_RETRACT = 255;
const int ACT_DUTY_EXTEND  = 255;

// Flip if EXTEND drives the rod the wrong way. Equivalent to swapping
// ACT_LEVEL_EXTEND/RETRACT — do one or the other, not both, they cancel.
const bool INVERT_ACT = false;

// SCALED, because micros() comes off Timer0 exactly as millis() does. A raw
// 4000 here would be 62.5 us of real time: 16 kHz, far faster than loop() can
// service, and the duty would collapse into noise.
//
// 250 Hz real. Slow enough that a loop which spends most of its time in an SPI
// read can hit each edge, fast enough that the mechanism sees a smooth average.
const unsigned long ACT_PWM_PERIOD_US = REAL_US(4000);

// ===========================================================================
// BRUSH MOTOR
// ===========================================================================
// Driven from the panel's TOGGLE switch (Pi GPIO13), folded with the pot into
// one 0..255 number on the Pi.
//
// THIS WAS ONE PIN AND THAT IS WHY IT DID NOT WORK. The brush hangs off a
// dual-channel driver exactly like a wheel, so it needs BOTH inputs. Driving
// the direction line alone set the polarity of a bridge whose gate was never
// asserted — motor dead, telemetry cheerfully reporting BRUSH=ON.
//
// MOVED D7 -> D2 ON 2026-08-15: D7 is the actuator's direction line, and this
// sketch was holding it HIGH permanently as the brush's direction, which means
// RETRACT. Two pins nothing thought of as the actuator's added up to a
// permanent retract command.
//
// D3 is Timer2 OC2B, so the duty is real hardware PWM at 62.5 kHz.
const uint8_t PIN_BRUSH_DIR = 2;
const uint8_t PIN_BRUSH_PWM = 3;

// The running direction level. NOT used for a stop — see brushApply(), which
// drops BOTH lines. Flip this if the brush runs backwards.
const uint8_t BRUSH_DIR_LEVEL = HIGH;

// Many driver and relay inputs are ACTIVE-LOW and will run the load for the
// whole time the Uno is in reset if this is wrong.
const bool BRUSH_ACTIVE_HIGH = true;

// The smallest duty that TURNS the brush rather than buzzing it. Any non-zero
// demand is stretched onto BRUSH_MIN_DUTY..255.
const int BRUSH_MIN_DUTY = 90;

// ===========================================================================
// PANEL LIGHT
// ===========================================================================
// Brightness follows the panel potentiometer (ADS1115 A2 on the Pi), scaled to
// 0..255 there and applied here.
//
// LIGHT_DIR IS THE RETURN LEG AND STAYS LOW. ALWAYS. This channel is a TWO-LEG
// BRIDGE, not a direction line plus a gate, and the lamp sees the DIFFERENCE
// between its legs. Settled on the rig 2026-08-26 by observation:
//
//     DIR=HIGH PWM=0      lamp ON     <- "light is on in bot", knob at zero
//     DIR=LOW  PWM=0      lamp OFF    <- correct
//     DIR=HIGH PWM=level  lamp OFF    <- "when i turn pot light not on"
//
// With DIR pinned high, brightness went as (255 - level): the knob ran
// BACKWARDS. THE LEGS CANNOT BE SWAPPED — D8 has no timer and cannot carry PWM,
// so the static leg has to be that one, which means it has to be the LOW one.
const uint8_t PIN_LIGHT_DIR = 8;
const uint8_t PIN_LIGHT_PWM = 9;   // Timer1 OC1A — hardware PWM 62.5 kHz

// ===========================================================================
// STATUS LED
// ===========================================================================
// LED_BUILTIN = D13, which is also the SPI clock the shield uses. With the
// shield fitted this tracks Ethernet traffic rather than link state and is NOT
// a reliable indicator. Never put a real signal on D13.
const uint8_t PIN_STATUS_LED = LED_BUILTIN;

// ===========================================================================
// FAILSAFE
// ===========================================================================
// 300 ms real: long enough to ride out ~15 dropped datagrams at the 50 Hz
// command rate, short enough that the robot stops within a third of a second of
// a real tether failure. TEST IT WITH THE WHEELS OFF THE GROUND.
const unsigned long FAILSAFE_MS = REAL_MS(300);

// ===========================================================================
// DRIVE TUNING
// ===========================================================================
// Below DEADBAND the motor buzzes and heats without turning, so treat it as
// zero. This band rejects NOISE from any sender; the stick's own deadband
// belongs to inputs.py, where the centre is learned and the rescale lives.
//
// Do NOT set it to 0: MIN_DUTY stretches every surviving non-zero demand up to
// 90, so a stray demand of 1 would spin a wheel at 35% duty.
const int DEADBAND = 12;
const int MAX_PWM  = 255;

// MIN_DUTY is the smallest duty that actually TURNS a loaded wheel. Between
// DEADBAND and roughly a third of full duty these gearmotors only buzz: the
// bridge switches, but the average voltage never breaks static friction, so a
// half-deflected stick makes heat instead of motion.
//
// Any surviving non-zero demand is stretched onto MIN_DUTY..MAX_PWM. Done on
// the Uno rather than the Pi so it covers EVERY sender and so two of them
// cannot apply it twice. Zero stays exactly zero — this raises the smallest
// MOVING demand and must never turn a stop into a crawl.
//
// A floor reintroduces a step of exactly its own size at the deadband edge —
// that is unavoidable, it is what a floor IS. Lower gives a gentler start and a
// wider buzzing band; higher gives a harder start and no buzz.
const int MIN_DUTY = 90;

// ===========================================================================
// RECEIVE BUFFER
// ===========================================================================
// NOT UDP_TX_PACKET_MAX_SIZE — that constant is 24 bytes in the stock library
// and silently truncates anything longer, which looks exactly like a garbled
// link. Keep the Pi side's MAX_PAYLOAD below this.
const uint16_t RX_BUFFER = 96;

// ===========================================================================
// TELEMETRY PACING
// ===========================================================================
// Report on change no faster than this, and on a heartbeat regardless, so a
// resting link still proves itself.
const unsigned long TELEMETRY_MIN_INTERVAL_MS = REAL_MS(200);
const unsigned long TELEMETRY_HEARTBEAT_MS    = REAL_MS(3000);

// ===========================================================================
// LOOP PACING
// ===========================================================================
// 10 ms real, on the operator's instruction 2026-08-27 ("delay(10) in the void
// loop so the uno wont stuck").
//
// IT IS NOT delay(10), AND THAT IS THE WHOLE POINT. delay() stops the world,
// and the world it stops includes the soft-PWM service whose period is 4 ms.
// Servicing a 4 ms waveform once per 10 ms pause leaves the sampled phase
// alternating between 0 and 2000 us, so the chopper can only express 0% or 50%
// at about 100 Hz — visible flicker and audible stutter, which is exactly the
// "brush powered on off on off" symptom. The busy-wait in loop() keeps both
// synthesised stages serviced at loop speed throughout the pause, so the pacing
// costs nothing.
//
// What the pause buys: the AVR stops hammering the shield with back-to-back SPI
// transactions it has no reason to make. The Pi sends at 50 Hz, so polling at
// tens of kHz finds nothing almost every time, and the board draws less — which
// on a rail this marginal is the point.
const unsigned long LOOP_PAUSE_US = REAL_US(10000);

// ===========================================================================
// ETHERNET RECOVERY — see Link.cpp for the faults these exist for
// ===========================================================================
// 5 s: long enough that a brief carrier drop does not thrash the chip (re-init
// leaves it deaf for ~60 ms), short enough that a cold start is driving within
// a few seconds of power-up.
const unsigned long ETH_REINIT_MS = REAL_MS(5000);

// How long to tolerate silence BEFORE the first packet ever arrives. Sized so a
// Pi booting from cold alongside this board is never mistaken for a broken
// shield: Linux plus the ground station is comfortably under a minute, this
// board is listening about five seconds in.
const unsigned long ETH_COLD_WAIT_MS = REAL_MS(45000);

// Let the 5 V rail settle before the W5100's init. 3 s, raised from 1 s
// 2026-08-26: the board failed only when it and the Pi had been off more than
// five minutes, which is a capacitor-discharge signature. After a brief outage
// the bulk caps are part-charged and the rail snaps up; after five minutes they
// are flat and it crawls up through a regulator already burning (12-5) x 0.23 A.
//
// NOT SCALED, AND THAT IS DELIBERATE. linkBegin() runs BEFORE timersBegin(), so
// Timer0 is still on its stock prescaler and delay() means what it says. Every
// other duration in this file is used after the switch and goes through
// REAL_MS(); these two are the exception, and putting REAL_MS() on them would
// make this a 192-SECOND pause.
const unsigned long ETH_SETTLE_MS = 3000;

// Gap between shield bring-up attempts, also pre-prescaler real milliseconds.
const unsigned long ETH_RETRY_GAP_MS = 400;

// ===========================================================================
// HOLD IN setup() UNTIL THE GROUND STATION IS ACTUALLY TALKING
// ===========================================================================
// Requested 2026-08-29: don't enter loop() until the link is established, and
// keep retrying until it is.
//
// READ THIS BEFORE CHANGING IT — A BOOT-TIME WAIT HAS ALREADY FAILED ONCE.
// A probe like this existed on 2026-08-27 and was removed the same day. It
// waited for a packet and RESET THE W5100 when none came, reasoning that the Pi
// transmits at 50 Hz so silence must mean a deaf shield. That reasoning fails at
// exactly the moment that matters: when the operator powers the whole rig on,
// this board is listening about five seconds later while the Pi is still most of
// a minute from booting Linux and starting the ground station. There is nothing
// to hear yet and nothing wrong. The probe read that normal silence as a fault
// and fired six chip resets into a perfectly healthy shield — which is what
// "turn off pi and uno 10-15 min, after start its not connected, 3-4 times on
// off uno and it works" actually was.
//
// THE WAIT WAS NEVER THE PROBLEM. THE RESET WAS. So this one waits as long as it
// takes and repairs the shield ONLY on positive evidence that it is broken —
// chip undetected, or its IP not reading back. A healthy shield with nobody
// talking to it is left alone, however long the silence lasts.
//
// SAFE TO BLOCK IN: outputsBegin(), timersBegin() and safeState() have all run
// before this, so every output is already at a stopped level and nothing can
// command them while we are here. The rod's soft-PWM is serviced throughout.
const bool LINK_WAIT_FOR_GROUND_STATION = true;

// 0 = wait forever, which is the default and what was asked for. Set a real
// number of milliseconds to give up and run anyway — loop() handles a missing
// link perfectly well on its own (that is what the failsafe and linkService()
// are for), so a timeout here costs nothing but the determinism.
//
// A board that waits forever is safe but SILENT-LOOKING, so the wait prints a
// progress line every few seconds. If the console is quiet too, the fault is the
// board or the rail, not the link.
const unsigned long LINK_WAIT_TIMEOUT_MS = 0;

// How often to print a progress line while waiting, and how often to re-test the
// shield for a genuine fault.
const unsigned long LINK_WAIT_NOTE_MS   = REAL_MS(3000);
const unsigned long LINK_WAIT_RECHECK_MS = REAL_MS(5000);

// ===========================================================================
// IS THE USB PORT ALLOWED TO DRIVE THE ROBOT?
// ===========================================================================
// On the ETHERNET build: false, on the operator's instruction 2026-08-26 —
// "data transfer only via ethernet". USB is a FLASHING CABLE on this rig.
// The port is still drained so a terminal cannot stall the RX buffer, and the
// banner and telemetry still print; only the COMMAND parser is switched off.
//
// On the SERIAL build this is necessarily true — it is the only transport.
// SET THIS true ON THE ETHERNET BUILD TO GET THE SECOND WIRE BACK. Until
// 2026-08-26 that build accepted commands from either transport and a dead
// Ethernet link simply failed over to USB. That redundancy is off by choice,
// not by structure — flip this and set the Pi's UNO_TRANSPORT=auto with it.
//
// On the SERIAL build it is necessarily true: that port is the only transport.
#if LINK_TRANSPORT == LINK_SERIAL
const bool SERIAL_COMMANDS = true;
#else
const bool SERIAL_COMMANDS = false;
#endif
