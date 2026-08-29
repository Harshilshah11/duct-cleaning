#pragma once
#include <Arduino.h>

/*
 * SoftPwmPin — software PWM on a pin that has no timer behind it.
 *
 * Both the linear actuator's gate (D4) and the brush's speed line (A1) are
 * plain digital pins on this rig: every hardware PWM pin the Uno has is already
 * spent (D3 and D6 on the wheels, D5 on the light, D9 on a direction line,
 * D10-D13 on the shield's SPI). This class is the one mechanism that serves
 * both, which is why the period is shared — two independent soft-PWM periods
 * would be two numbers to keep in sync for no benefit.
 *
 * THE ENDPOINTS ARE STATIC, AND THAT IS A SAFETY PROPERTY, not an optimisation.
 * A duty of 0 or full resolves to a plain level that cannot be caught mid-cycle:
 *
 *   - a stopped actuator must be held off by something a stalled loop cannot
 *     freeze in the "on" half;
 *   - a full-speed channel should not be chopped by a software timer at all.
 *
 * Only the middle range is synthesised, and a stalled loop there costs a slower
 * actuator, never a runaway one.
 *
 * service() must be called every pass of loop(). The more often it runs, the
 * cleaner the duty; the mechanisms driven this way are far too slow mechanically
 * to care about ripple at a few hundred Hz.
 */
class SoftPwmPin {
public:
    /* activeHigh=false for driver or relay inputs that enable on a LOW — the
     * off level, the static endpoints and the chopped on-phase all invert
     * together through this one flag. Getting it wrong runs the load for the
     * whole time the board is in reset. */
    SoftPwmPin(uint8_t pin, bool activeHigh, unsigned long periodUs,
               uint8_t maxDuty);

    /* Write the OFF level, then make the pin an output. On an active-LOW module
     * the off level is HIGH, and writing it first switches on the input pull-up,
     * which holds the load off across the gap before pinMode. Leaving the pin
     * floating there is exactly how a relay board ends up running its load
     * during reset. */
    void begin();

    /* Store the duty and apply the static endpoints IMMEDIATELY, so a stop takes
     * effect on this very call rather than on the next service(). */
    void setDuty(int duty);

    /* Chop the mid-range. Cheap and non-blocking; call it every loop pass. */
    void service();

    int duty() const { return _duty; }

private:
    void write(bool on);

    const uint8_t       _pin;
    const bool          _activeHigh;
    const unsigned long _periodUs;
    const uint8_t       _maxDuty;
    int                 _duty;
};
