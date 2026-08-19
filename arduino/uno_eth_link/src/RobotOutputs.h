#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotorChannel.h"
#include "LinearActuator.h"
#include "BrushMotor.h"
#include "PanelLight.h"

/*
 * RobotOutputs — everything this board drives, wired to THIS rig.
 *
 * The channel classes above it (MotorChannel, SoftPwmPin, LinearActuator,
 * BrushMotor, PanelLight) know nothing about any particular robot: all their
 * pins and limits arrive through their constructors. This class is where those
 * generic parts meet Config.h and become one specific machine. That split is
 * the point — port the sketch to another rig by rewriting this file and
 * Config.h, and leave everything else alone.
 */
class RobotOutputs {
public:
    RobotOutputs();

    /* Every pin reaches a stopped level before it becomes an output, then
     * safeState() runs once so the board leaves setup() genuinely idle. */
    void begin();

    /* Drive the software PWM for the rod and the brush. MUST be called every
     * pass of loop() — the more often it runs, the cleaner their duty. */
    void service();

    /* The M form: wheels plus the three trailing channels. */
    void applyAll(int left, int right, int act, int brush, int light);

    /* Everything to neutral. Runs on every failsafe trip, so it is
     * unconditional and must not depend on any prior state. */
    void safeState();

    /* The link lamp. Lit means commands are arriving, dark means failsafe.
     *
     * NOTE: this is LED_BUILTIN = D13, which is also the SPI clock the shield
     * uses. With the shield fitted it tracks Ethernet traffic rather than link
     * state and is NOT a reliable indicator. Harmless, but do not read anything
     * into it — and never put a real signal on D13. */
    void setLinkLed(bool on);

private:
    MotorChannel   _left;
    MotorChannel   _right;
    LinearActuator _actuator;
    BrushMotor     _brush;
    PanelLight     _light;
};
