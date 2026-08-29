#pragma once
#include <Arduino.h>

/*
 * PanelLight — a lamp on a motor-driver channel, dimmed by the panel pot.
 *
 * NO DEADBAND HERE, deliberately unlike MotorChannel. A motor below ~12 buzzes
 * and heats without turning, so folding that to zero is right. A lamp at 12/255
 * is simply dim, and applying the same rule would give the pot a dead patch at
 * the bottom of its travel that reads as a broken knob.
 *
 * The direction line is held HIGH rather than steered. A lamp has no reverse,
 * but a driver channel used as a dimmer still wants a defined polarity on its
 * direction input; on a driver that ignores the pin this costs nothing.
 *
 * The PWM pin may be a Timer0 pin, so a full-off goes through digitalWrite —
 * see setLevel().
 */
class PanelLight {
public:
    PanelLight(uint8_t dirPin, uint8_t pwmPin, uint8_t maxLevel);

    void begin();

    /* Brightness 0..maxLevel, straight off the potentiometer. */
    void setLevel(int level);

private:
    const uint8_t _dirPin;
    const uint8_t _pwmPin;
    const uint8_t _maxLevel;
};
