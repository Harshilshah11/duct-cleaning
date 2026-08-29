#pragma once
#include <Arduino.h>

/*
 * Outputs — everything this board drives.
 *
 * Ordering rule shared by every channel here: DIRECTION IS WRITTEN BEFORE
 * SPEED. The other order spends a few microseconds driving the old direction at
 * the new duty, which is a current spike through the bridge on every reversal.
 *
 * Stop rule shared by every PWM pin: a full stop goes through digitalWrite(pin,
 * LOW), never analogWrite(pin, 0). On a timer pin a zero duty can still emit a
 * narrow pulse every period — a creeping motor, or a lamp with a faint glow
 * that will not go out.
 */

/* Drive every pin to a stopped level, then make it an output. Call from setup()
 * before anything else can be commanded. */
void outputsBegin();

/* Drive the software PWM for the rod. MUST be called every pass of loop() and
 * throughout the loop pause — the more often it runs, the cleaner the duty. */
void outputsService();

/* One wheel. Sign picks direction, magnitude becomes PWM (-255..255). */
void motorApply(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert);

/* The rod. SIGN ONLY: positive extends, negative retracts, zero stops and
 * holds position. */
void actuatorApply(int demand, bool invert);

/* The brush, duty 0..255. A stop drops BOTH lines — see the .cpp. */
void brushApply(int duty);

/* The lamp, brightness 0..255 straight off the pot. No deadband. */
void lightApply(int level);

/* Everything to neutral. Runs on every failsafe trip, so it is unconditional
 * and must not depend on any prior state. */
void safeState();

/* The link lamp. Lit means commands are arriving, dark means failsafe. */
void setLinkLed(bool on);
