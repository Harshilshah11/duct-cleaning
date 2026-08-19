#include "SoftPwmPin.h"

SoftPwmPin::SoftPwmPin(uint8_t pin, bool activeHigh, unsigned long periodUs,
                       uint8_t maxDuty)
    : _pin(pin), _activeHigh(activeHigh), _periodUs(periodUs),
      _maxDuty(maxDuty), _duty(0) {}

void SoftPwmPin::write(bool on) {
    // One place where the active-high/low sense is resolved. Everything else in
    // this class speaks in terms of "on" and "off".
    digitalWrite(_pin, (on == _activeHigh) ? HIGH : LOW);
}

void SoftPwmPin::begin() {
    write(false);            // safe level BEFORE the pin becomes an output
    pinMode(_pin, OUTPUT);
}

void SoftPwmPin::setDuty(int duty) {
    if (duty < 0) duty = 0;
    if (duty > (int)_maxDuty) duty = (int)_maxDuty;
    _duty = duty;

    // Endpoints are written here as well as in service(), so a stop lands on
    // this line rather than waiting for the next loop pass.
    if (duty <= 0) {
        write(false);
    } else if (duty >= (int)_maxDuty) {
        write(true);
    }
}

void SoftPwmPin::service() {
    if (_duty <= 0)             { write(false); return; }
    if (_duty >= (int)_maxDuty) { write(true);  return; }

    // micros() wraps about every 71 minutes. The modulo makes that a single
    // short cycle rather than a stuck output, so it is left unhandled
    // deliberately — a mechanism this slow cannot notice one ragged period.
    const unsigned long phase = micros() % _periodUs;
    const unsigned long onFor = (_periodUs * (unsigned long)_duty)
                              / (unsigned long)_maxDuty;
    write(phase < onFor);
}
