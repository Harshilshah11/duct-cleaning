#pragma once
#include <Arduino.h>

// Transport: LINK_ETHERNET (rig, UDP) or LINK_SERIAL (bench, USB).
#define LINK_ETHERNET 1
#define LINK_SERIAL   2
#define LINK_TRANSPORT LINK_ETHERNET

// Timer0 runs at prescaler 1 for 62.5 kHz PWM, so millis/micros/delay are 64x
// fast. EVERY duration below must go through REAL_MS/REAL_US. Library delays
// are NOT compensated - see StockTimer0 in Timers.h.
const unsigned long MILLIS_SCALE = 64;
#define REAL_MS(x) ((unsigned long)(x) * MILLIS_SCALE)
#define REAL_US(x) ((unsigned long)(x) * MILLIS_SCALE)

// 500000: UBRR=3 with U2X, exactly 500000.0, 0.00% error on a 16 MHz AVR.
// NOTE: macOS cannot open this rate (tcsetattr: Invalid argument) and falls
// back to 9600, so the console is unreadable from a Mac - read it from the Pi,
// which takes arbitrary rates. Use 115200 (+2.12%) if you need Mac-side serial.
// SERIAL BUILD: the 64-byte RX buffer fills in 1.3 ms at this rate, so drop to
// 250000 or 115200 if LINK_TRANSPORT ever becomes LINK_SERIAL.
const unsigned long SERIAL_BAUD = 500000;

// Not 192.168.1.20 - that is the Pi's own wlan0, so packets never reach the wire.
const uint16_t NET_LISTEN_PORT = 5005;
extern byte          NET_MAC[6];
extern const uint8_t NET_IP[4];
extern const uint8_t NET_SUBNET[4];
extern const uint8_t NET_GATEWAY[4];

// D4 is the shield's microSD chip select AND the right wheel's direction line.
// RUN WITH THE SLOT EMPTY. D10-D13 are SPI and unusable.
const uint8_t PIN_DIR1 = 7;
const uint8_t PIN_PWM1 = 6;    // Timer0 OC0A
const uint8_t PIN_DIR2 = 4;    // SD CS - slot empty
const uint8_t PIN_PWM2 = 5;    // Timer0 OC0B
const bool INVERT_1 = false;
// TRUE on this rig - measured, not a default. The right motor is landed the
// other way round, so without this "forward" spins the robot and "turn right"
// drives it straight. It was briefly false (429a21b) and that is exactly what
// happened. Flip only against the hardware, never to taste.
// FALSE. Both channels uninverted, which is what the wiring wants.
//
// Set true on the way to this and it is what CAUSED the fault it was meant to
// fix: the operator reported the whole mapping rotated a quarter turn - "right
// is forward and left is backward and forward is right and backward is left" -
// with the panel reading correctly, so the stick and the picture agreed and only
// the wheels did not.
//
// A ONE-WHEEL INVERSION IS EXACTLY THAT ROTATION. mix() carries the forward
// demand in the COMMON component of the pair and the turn demand in the
// DIFFERENCE. Invert one wheel and the two exchange roles: a common command
// becomes a spin and a differential command drives straight. Forward becomes
// turn and turn becomes forward - a rotation, not a mirror. So a rotated mapping
// means the inverts DISAGREE with each other, and the cure is to make them
// match, never to add another.
//
// WHICH WAY THE MATCHED PAIR THEN DRIVES is the second question, and separate:
// both true and both false each stop the spin, and they drive opposite ways.
// Both false is the pairing that drives forward on this rig, confirmed on the
// bench 2026-08-29.
//
// NOT TO BE FIXED IN inputs.py. SWAP_XY and INVERT_X/INVERT_Y there feed the
// panel as well as the mixer, and the panel is right; touching them would move
// the picture to fix the wheels. This is the last point that reaches the motors
// alone.
const bool INVERT_2 = false;

// Rod EXTENDS on DIR **LOW** - opposite to every other channel here.
// PIN_ACT_PWM is the only line that gates it; a stop drops it and holds.
const uint8_t PIN_ACT_DIR = A3;
const uint8_t PIN_ACT_PWM = A2;
const uint8_t ACT_LEVEL_EXTEND  = LOW;
const uint8_t ACT_LEVEL_RETRACT = HIGH;
const int ACT_DUTY_STOP    = 0;
const int ACT_DUTY_RETRACT = 255;
const int ACT_DUTY_EXTEND  = 255;
const bool INVERT_ACT = false;
const unsigned long ACT_PWM_PERIOD_US = REAL_US(4000);   // 250 Hz real

// Brush needs BOTH lines. A stop drives both LOW - on a two-input driver,
// DIR high with the gate low is FORWARD AT FULL SCALE, not "stopped".
const uint8_t PIN_BRUSH_DIR = 2;
const uint8_t PIN_BRUSH_PWM = 3;    // Timer2 OC2B
const uint8_t BRUSH_DIR_LEVEL = HIGH;
const bool BRUSH_ACTIVE_HIGH = true;
const int BRUSH_MIN_DUTY = 90;

// Ceiling on the brush, separate from MAX_PWM: full demand maps here, not to
// 255. Full scale is what the driver and the motor spend the most current on,
// and this rail is already the reason the W5100's PHY struggles.
const int BRUSH_MAX_DUTY = 220;

// Soft-start. A stalled DC motor draws locked-rotor current - several times its
// running current - until it spins up, and on this rail that surge is what the
// W5100's PHY cannot survive. Ramps UP only: a stop is always immediate, so the
// failsafe still kills the brush in one call.
// HOW LONG the climb takes, end to end. Time-proportional, not step-per-tick:
// the duty is computed from elapsed time, so the total is this value whether
// the service runs every pass or misses most of them. The step-based version
// this replaces drifted to ~6 s against a 2 s design, because a missed tick
// stretched the ramp instead of being caught up.
const unsigned long BRUSH_RAMP_MS = REAL_MS(2000);

// Two-leg bridge: PIN_LIGHT_DIR stays LOW always, or the knob runs backwards.
const uint8_t PIN_LIGHT_DIR = 8;
const uint8_t PIN_LIGHT_PWM = 9;    // Timer1 OC1A

const uint8_t PIN_STATUS_LED = LED_BUILTIN;   // D13 = SPI clock, not a real indicator

const unsigned long FAILSAFE_MS = REAL_MS(300);

// While the link is DOWN, re-assert neutral this often. The trip itself writes
// safeState() once; this keeps writing it. A driver channel that browns out and
// recovers - which happens on this rig, the brush shares the Uno's buck - would
// otherwise come back to whatever its inputs floated to, with nothing driving
// them again until a command arrived. Cheap: a handful of digitalWrites.
const unsigned long FAILSAFE_HOLD_MS = REAL_MS(250);

// DEADBAND rejects sender noise; MIN_DUTY is the smallest duty that turns a
// loaded wheel. Zero stays zero. Keep DEADBAND below MIN_DUTY.
const int DEADBAND = 12;
const int MAX_PWM  = 255;
const int MIN_DUTY = 90;

const uint16_t RX_BUFFER = 96;      // not UDP_TX_PACKET_MAX_SIZE (24, truncates)

const bool TELEMETRY_ENABLED = false;
const unsigned long TELEMETRY_MIN_INTERVAL_MS = REAL_MS(200);
const unsigned long TELEMETRY_HEARTBEAT_MS    = REAL_MS(3000);

const unsigned long LOOP_PAUSE_US = REAL_US(10000);   // busy-wait, not delay()

const unsigned long ETH_REINIT_MS    = REAL_MS(5000);
const unsigned long ETH_COLD_WAIT_MS = REAL_MS(45000);  // patient before first contact
const unsigned long ETH_RETRY_GAP_MS = 400;             // pre-prescaler: real ms

#if LINK_TRANSPORT == LINK_SERIAL
const bool SERIAL_COMMANDS = true;
#else
const bool SERIAL_COMMANDS = false;   // USB is a flashing cable on this rig
#endif
