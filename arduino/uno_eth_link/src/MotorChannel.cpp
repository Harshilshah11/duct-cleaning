#include "MotorChannel.h"

MotorChannel::MotorChannel(uint8_t dirPin, uint8_t pwmPin, bool invert,
                           uint8_t deadband, uint8_t minDuty, uint8_t maxPwm)
    : _dirPin(dirPin), _pwmPin(pwmPin), _invert(invert),
      _deadband(deadband), _minDuty(minDuty), _maxPwm(maxPwm) {}

void MotorChannel::begin() {
    // Levels BEFORE pinMode, so the pin cannot glitch high for the instant
    // between becoming an output and receiving its first write.
    digitalWrite(_dirPin, LOW);
    digitalWrite(_pwmPin, LOW);
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
}

void MotorChannel::setDemand(int demand) {
    if (_invert) demand = -demand;

    const int maxPwm = (int)_maxPwm;
    if (demand > maxPwm)  demand = maxPwm;
    if (demand < -maxPwm) demand = -maxPwm;

    // Below the deadband the motor buzzes and heats without turning, so fold
    // it to a real zero.
    if (demand > -(int)_deadband && demand < (int)_deadband) demand = 0;

    // Direction is set BEFORE the new PWM value. The other order spends a few
    // microseconds driving the old direction at the new speed, which is a
    // current spike through the bridge on every reversal.
    digitalWrite(_dirPin, demand >= 0 ? HIGH : LOW);

    int duty = demand >= 0 ? demand : -demand;

    if (duty == 0) {
        // NOT analogWrite(pin, 0). On a Timer0 pin (PWM2 = D6 on this rig) a
        // zero duty can still emit a narrow pulse every period, which leaves
        // the motor creeping. digitalWrite is the only guaranteed dead level,
        // and this is the line that makes the robot actually stop.
        digitalWrite(_pwmPin, LOW);
        return;
    }

    if (_minDuty > 0) {
        // Stretch 1..maxPwm onto minDuty..maxPwm so the slowest demand the
        // stick can express is still one the motor can act on.
        //
        // The multiply is promoted to long DELIBERATELY: 254 * 165 is 41910,
        // which overflows the Uno's 16-bit int and would wrap to a negative
        // duty — a wheel that runs backwards near full stick. Do not "simplify"
        // this cast away.
        duty = (int)_minDuty
             + (int)(((long)(duty - 1) * (long)(maxPwm - (int)_minDuty))
                     / (long)(maxPwm - 1));
    }
    analogWrite(_pwmPin, duty);
}

void MotorChannel::stop() {
    // PWM first, then direction — the same order safeState() used when it wrote
    // these pins directly. Killing the drive before touching the direction line
    // means the bridge is already idle when the direction changes.
    digitalWrite(_pwmPin, LOW);
    digitalWrite(_dirPin, LOW);
}
