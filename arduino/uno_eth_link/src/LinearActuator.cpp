#include "LinearActuator.h"

LinearActuator::LinearActuator(uint8_t dirPin, uint8_t pwmPin, bool invert,
                               uint8_t extendLevel, uint8_t retractLevel,
                               uint8_t extendDuty, uint8_t retractDuty,
                               unsigned long periodUs, uint8_t maxDuty)
    : _dirPin(dirPin), _invert(invert),
      _extendLevel(extendLevel), _retractLevel(retractLevel),
      _extendDuty(extendDuty), _retractDuty(retractDuty),
      // The gate is active-HIGH: it powers the channel when driven high.
      _pwm(pwmPin, true, periodUs, maxDuty) {}

void LinearActuator::begin() {
    // BOTH lines reach their stopped level before either becomes an output.
    //
    // This is also what replaced the old "park pin 4 HIGH to deselect the SD
    // card" line. HIGH on the gate is a full-scale drive command, and that park
    // is why the rod ran from reset forever. There is no SD deselect here, and
    // there must not be one: run the board with the microSD slot EMPTY.
    digitalWrite(_dirPin, _extendLevel);
    _pwm.begin();                 // writes the gate's off level, then pinMode
    pinMode(_dirPin, OUTPUT);
}

void LinearActuator::setDemand(int demand) {
    if (_invert) demand = -demand;

    if (demand == 0) {
        // Gate down, direction left exactly where it was. See the class comment
        // — this is a genuine stop-and-hold, and the direction line is not part
        // of it.
        _pwm.setDuty(0);
        return;
    }

    // Direction BEFORE the gate, the same ordering rule MotorChannel follows:
    // the other order spends a few microseconds driving the old direction at
    // full scale, a current spike through the bridge on every reversal.
    //
    // Named levels rather than a bare ternary because this channel extends on
    // LOW while every other direction line on the board goes forward on HIGH.
    digitalWrite(_dirPin, demand > 0 ? _extendLevel : _retractLevel);
    _pwm.setDuty(demand > 0 ? (int)_extendDuty : (int)_retractDuty);
}
