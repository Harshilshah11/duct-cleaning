#include "BrushMotor.h"

BrushMotor::BrushMotor(uint8_t dirPin, uint8_t pwmPin, uint8_t dirLevel,
                       bool activeHigh, uint8_t minDuty, uint8_t maxDuty,
                       unsigned long periodUs)
    : _dirPin(dirPin), _dirLevel(dirLevel),
      _minDuty(minDuty), _maxDuty(maxDuty),
      _pwm(pwmPin, activeHigh, periodUs, maxDuty) {}

void BrushMotor::begin() {
    // The GATE is the line that must be safe before pinMode — it is what
    // actually powers the motor, and on an active-LOW module its off level is
    // HIGH. SoftPwmPin::begin() writes that level first and only then makes the
    // pin an output, which switches on the input pull-up across the gap and
    // holds the module off. The direction line can settle whenever.
    _pwm.begin();
    digitalWrite(_dirPin, _dirLevel);
    pinMode(_dirPin, OUTPUT);
}

void BrushMotor::setDuty(int duty) {
    if (duty < 0) duty = 0;
    if (duty > (int)_maxDuty) duty = (int)_maxDuty;

    // Direction before speed, the same ordering rule MotorChannel follows:
    // asserting the gate first would spend a moment driving the old direction
    // at the new duty.
    digitalWrite(_dirPin, _dirLevel);

    if (duty > 0 && _minDuty > 0) {
        // Same stretch as MotorChannel, and long arithmetic for the same
        // overflow reason: 254 * 165 wraps a 16-bit int.
        duty = (int)_minDuty
             + (int)(((long)(duty - 1) * (long)((int)_maxDuty - (int)_minDuty))
                     / (long)((int)_maxDuty - 1));
    }
    _pwm.setDuty(duty);
}
