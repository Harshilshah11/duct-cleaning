#pragma once
#include <Arduino.h>

/*
 * MotorChannel — one DIR + PWM channel of a dual-channel motor driver.
 *
 * Sign picks direction, magnitude becomes PWM. Everything that made the old
 * applyMotor() correct is preserved here and documented at the point it
 * applies; none of it is incidental.
 *
 * Tuning arrives through the constructor rather than from Config.h so this
 * class has no dependency on any particular rig, and so two channels can be
 * given different limits if that is ever wanted.
 *
 * NOT used for the linear actuator. That channel extends on DIR **LOW** while
 * every channel here goes forward on HIGH — see LinearActuator, which exists
 * precisely so that convention cannot be copied by accident.
 */
class MotorChannel {
public:
    MotorChannel(uint8_t dirPin, uint8_t pwmPin, bool invert,
                 uint8_t deadband, uint8_t minDuty, uint8_t maxPwm);

    /* Drive the pins to a stopped state and only then make them outputs, so
     * neither can glitch high in the gap between pinMode and the first write. */
    void begin();

    /* Apply a signed demand, -maxPwm..maxPwm. */
    void setDemand(int demand);

    /* Hard stop: BOTH lines low.
     *
     * Deliberately NOT setDemand(0). A zero demand leaves the direction line
     * HIGH (it is the "forward" level for a non-negative demand), whereas the
     * failsafe wants every line it owns driven low. safeState() relies on this
     * distinction, and the original code made the same one by writing the pins
     * directly rather than calling applyMotor(). */
    void stop();

private:
    const uint8_t _dirPin;
    const uint8_t _pwmPin;
    const bool    _invert;
    const uint8_t _deadband;
    const uint8_t _minDuty;
    const uint8_t _maxPwm;
};
