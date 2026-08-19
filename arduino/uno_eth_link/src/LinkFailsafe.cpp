#include "LinkFailsafe.h"

LinkFailsafe::LinkFailsafe(unsigned long timeoutMs)
    : _timeoutMs(timeoutMs), _lastMs(0), _count(0), _up(false) {}

bool LinkFailsafe::feed() {
    _count++;
    _lastMs = millis();
    if (!_up) {
        _up = true;
        return true;            // down -> up edge
    }
    return false;
}

bool LinkFailsafe::expired() {
    // millis() SUBTRACTION, never `millis() > last + timeout`. Unsigned
    // wraparound at ~49 days makes the additive form compare wrong exactly
    // once; this form stays correct across the rollover. Do not "tidy" it.
    if (_up && (millis() - _lastMs) >= _timeoutMs) {
        _up = false;
        return true;            // up -> down edge, fires once
    }
    return false;
}
