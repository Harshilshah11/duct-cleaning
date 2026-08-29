#include "RobotOutputs.h"

RobotOutputs::RobotOutputs()
    : _left (PIN_DIR1, PIN_PWM1, INVERT_1, DEADBAND, MIN_DUTY, MAX_PWM),
      _right(PIN_DIR2, PIN_PWM2, INVERT_2, DEADBAND, MIN_DUTY, MAX_PWM),
      _actuator(PIN_ACT_DIR, PIN_ACT_PWM, INVERT_ACT,
                ACT_LEVEL_EXTEND, ACT_LEVEL_RETRACT,
                ACT_DUTY_EXTEND, ACT_DUTY_RETRACT,
                SOFT_PWM_PERIOD_US, MAX_PWM),
      _brush(PIN_BRUSH_DIR, PIN_BRUSH_PWM, BRUSH_DIR_LEVEL, BRUSH_ACTIVE_HIGH,
             BRUSH_MIN_DUTY, MAX_PWM, SOFT_PWM_PERIOD_US),
      _light(PIN_LIGHT_DIR, PIN_LIGHT_PWM, MAX_PWM) {}

void RobotOutputs::begin() {
    // Each channel writes its own safe level before its own pinMode, so the
    // order between channels does not matter — no pin here depends on another
    // pin's mode.
    _left.begin();
    _right.begin();
    _actuator.begin();
    _brush.begin();
    _light.begin();
    pinMode(PIN_STATUS_LED, OUTPUT);

    safeState();
}

void RobotOutputs::service() {
    _actuator.service();
    _brush.service();
}

void RobotOutputs::applyAll(int left, int right, int act, int brush, int light) {
    _left.setDemand(left);
    _right.setDemand(right);
    _actuator.setDemand(act);
    _brush.setDuty(brush);
    _light.setLevel(light);
}

void RobotOutputs::safeState() {
    // stop(), NOT setDemand(0): a zero demand leaves the direction line HIGH,
    // and the failsafe wants every line it owns driven low.
    _left.stop();
    _right.stop();

    // The rod is genuinely STOPPED, not merely pointed somewhere — a zero
    // demand drops its gate, which is this driver's off state. Before the
    // channel was rewired as a pair, a failsafe could only pick a direction and
    // the rod ran to its end stop.
    _actuator.setDemand(0);

    // Brush off too, and through BrushMotor so an active-LOW module gets the
    // right level. A spinning brush is the loudest thing on the robot; it must
    // not be what survives a failsafe.
    _brush.setDuty(0);

    // Light out. It is the one output here that poses no motion hazard, so
    // leaving it lit was tempting — but the cameras stream over the SAME
    // tether, so once the link is down there is nobody left to see by it, and
    // this rig already browns out under load. Dark is cheaper.
    _light.setLevel(0);

    setLinkLed(false);
}

void RobotOutputs::setLinkLed(bool on) {
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}
