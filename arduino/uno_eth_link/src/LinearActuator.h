#pragma once
#include <Arduino.h>
#include "SoftPwmPin.h"

/*
 * LinearActuator — the rod: a direction line plus a gate line.
 *
 * THIS IS NOT A MotorChannel, AND THE SEPARATION IS THE POINT. On this rig the
 * rod EXTENDS on direction **LOW**, while the wheels and the brush all go
 * forward on HIGH. Reusing MotorChannel here would silently invert the rod, so
 * the levels are constructor parameters and this class carries its own rules.
 *
 * ONLY THE SIGN OF THE DEMAND IS USED. The Pi sends full scale either way, so
 * reading a magnitude would just be reading a constant — the three-position
 * panel switch picks the stage, and the stage table lives in the constructor.
 *
 * ZERO IS A REAL STOP, not a direction. The gate line is what powers the
 * channel, so dropping it leaves the rod exactly where it is — which is what the
 * panel's middle throw means and what the failsafe needs. It is not a brake and
 * not a reversal; the rod simply stops being driven.
 *
 * THE DIRECTION LINE IS DELIBERATELY NOT TOUCHED ON A STOP. Re-pointing a rod
 * that is no longer powered buys nothing, and holding the last direction means a
 * resumed command carries on the way it was already going. It also keeps a stop
 * to a single write on the one pin that matters.
 */
class LinearActuator {
public:
    LinearActuator(uint8_t dirPin, uint8_t pwmPin, bool invert,
                   uint8_t extendLevel, uint8_t retractLevel,
                   uint8_t extendDuty, uint8_t retractDuty,
                   unsigned long periodUs, uint8_t maxDuty);

    void begin();

    /* Sign only: positive extends, negative retracts, zero stops and holds. */
    void setDemand(int demand);

    /* Drive the soft-PWM. Call every pass of loop(). */
    void service() { _pwm.service(); }

    int duty() const { return _pwm.duty(); }

private:
    const uint8_t _dirPin;
    const bool    _invert;
    const uint8_t _extendLevel;
    const uint8_t _retractLevel;
    const uint8_t _extendDuty;
    const uint8_t _retractDuty;
    SoftPwmPin    _pwm;
};
