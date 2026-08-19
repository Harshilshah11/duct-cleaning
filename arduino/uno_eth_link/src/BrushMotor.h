#pragma once
#include <Arduino.h>
#include "SoftPwmPin.h"

/*
 * BrushMotor — the cleaning brush: a driver channel with a CONSTANT direction.
 *
 * IT NEEDS BOTH LINES, AND FORGETTING THAT IS WHY IT ONCE DID NOTHING. The
 * brush hangs off a dual-channel driver exactly like a wheel, so a direction
 * line alone sets the polarity of a bridge whose gate is never asserted: the
 * motor stays dead while the telemetry cheerfully reports it running, because
 * the sketch really is driving the one pin it knows about.
 *
 * The brush spins one way only, so direction is a fixed level rather than a
 * demand — which is what separates this from MotorChannel and why the duty is
 * unsigned.
 *
 * Both lines are written on every call, not just the one that changed. The
 * direction is a constant, but writing it every frame means a channel that
 * browns out and comes back gets its direction restored by the next frame
 * rather than running whichever way its input floated to. On this rig, which
 * has a supply that sags under motor load, that is not hypothetical.
 */
class BrushMotor {
public:
    BrushMotor(uint8_t dirPin, uint8_t pwmPin, uint8_t dirLevel,
               bool activeHigh, uint8_t minDuty, uint8_t maxDuty,
               unsigned long periodUs);

    void begin();

    /* Duty 0..maxDuty. Any non-zero demand is stretched onto minDuty..maxDuty,
     * so the bottom of the knob's travel is already a moving brush instead of a
     * heater. A pre-duty sender's 0/1 still behaves: 1 stretches to minDuty, a
     * slow brush rather than a dead one. */
    void setDuty(int duty);

    void service() { _pwm.service(); }

    int duty() const { return _pwm.duty(); }

private:
    const uint8_t _dirPin;
    const uint8_t _dirLevel;
    const uint8_t _minDuty;
    const uint8_t _maxDuty;
    SoftPwmPin    _pwm;
};
