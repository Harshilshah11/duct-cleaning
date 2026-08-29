#include "PanelLight.h"

PanelLight::PanelLight(uint8_t dirPin, uint8_t pwmPin, uint8_t maxLevel)
    : _dirPin(dirPin), _pwmPin(pwmPin), _maxLevel(maxLevel) {}

void PanelLight::begin() {
    digitalWrite(_dirPin, LOW);
    digitalWrite(_pwmPin, LOW);
    // An ANALOG pin driven as a plain digital output is legal on the Uno
    // (A0 == D14); pinMode(A0, OUTPUT) is the whole ceremony, and nothing else
    // is needed to stop it being an ADC input.
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
}

void PanelLight::setLevel(int level) {
    if (level < 0) level = 0;
    if (level > (int)_maxLevel) level = (int)_maxLevel;

    digitalWrite(_dirPin, HIGH);

    if (level == 0) {
        // The PWM pin is on Timer0, where analogWrite(pin, 0) can still emit a
        // narrow pulse every period. On a motor that is a creep; on a lamp it
        // is a faint glow that will not go out. digitalWrite is the only
        // certain dark.
        digitalWrite(_pwmPin, LOW);
    } else {
        analogWrite(_pwmPin, level);
    }
}
