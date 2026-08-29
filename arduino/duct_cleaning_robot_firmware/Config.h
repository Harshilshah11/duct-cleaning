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

// 115200: UBRR=16 gives 117647, +2.12% - inside the ~4% a UART tolerates.
// NOT the exact rates (250000/500000, 0.00%): macOS refuses both outright
// (tcsetattr: Invalid argument) and silently falls back to 9600, which makes
// the console unreadable from the dev Mac. 230400 is worse on both counts
// (-3.55%, UBRR rounds to 8). Telemetry is off, so the print-cost argument for
// a faster rate no longer applies.
// 500000, and exact on a 16 MHz AVR: UBRR=3 with U2X gives 500000.0, error
// 0.00% - better than 115200's +2.12%. Halves the telemetry print cost, which is
// LOOP time because that line prints from loop().
//
// LAST SAFE STEP UP, and the limit is the RECEIVE side. The 64-byte serial RX
// buffer fills in 1.3 ms at this rate. Harmless on the Ethernet build, where
// nothing sends to this port and it is only drained - but the SERIAL build
// carries commands there and would overrun inside one loop pass. If
// LINK_TRANSPORT ever goes back to LINK_SERIAL, bring this down with it.
//
// ONE SETTING IN TWO FILES: this and UNO_BAUD in uno_serial.py (and the
// out-of-repo diag/uno_logger.py). Neither compiles against the other, so a
// split is silent until something reads the port - and a mismatch is a DEAD
// link, not a slow one. Change one, change all three.
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

// Two-leg bridge: PIN_LIGHT_DIR stays LOW always, or the knob runs backwards.
const uint8_t PIN_LIGHT_DIR = 8;
const uint8_t PIN_LIGHT_PWM = 9;    // Timer1 OC1A

const uint8_t PIN_STATUS_LED = LED_BUILTIN;   // D13 = SPI clock, not a real indicator

const unsigned long FAILSAFE_MS = REAL_MS(300);

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
