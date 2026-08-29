#include "Outputs.h"
#include "Config.h"

static int actDuty = 0;

void motorApply(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert) {
    if (invert) demand = -demand;
    if (demand >  MAX_PWM) demand =  MAX_PWM;
    if (demand < -MAX_PWM) demand = -MAX_PWM;
    if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

    digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);

    int duty = demand >= 0 ? demand : -demand;
    if (duty == 0) { digitalWrite(pwmPin, LOW); return; }
    if (MIN_DUTY > 0) {
        // long: 254 * 165 overflows a 16-bit int and wraps negative.
        duty = MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - MIN_DUTY)) / (MAX_PWM - 1));
    }
    analogWrite(pwmPin, duty);
}

void actuatorApply(int demand, bool invert) {
    if (invert) demand = -demand;
    if (demand == 0) {
        actDuty = ACT_DUTY_STOP;
        digitalWrite(PIN_ACT_PWM, LOW);     // direction deliberately untouched
        return;
    }
    digitalWrite(PIN_ACT_DIR, demand > 0 ? ACT_LEVEL_EXTEND : ACT_LEVEL_RETRACT);
    actDuty = demand > 0 ? ACT_DUTY_EXTEND : ACT_DUTY_RETRACT;
}

/* Endpoints resolve to a static level - a stalled loop then costs a slower rod,
 * never a runaway one. Only the mid range is chopped. */
void outputsService() {
    if (actDuty <= ACT_DUTY_STOP) { digitalWrite(PIN_ACT_PWM, LOW);  return; }
    if (actDuty >= MAX_PWM)       { digitalWrite(PIN_ACT_PWM, HIGH); return; }
    unsigned long phase = micros() % ACT_PWM_PERIOD_US;
    unsigned long onFor = (ACT_PWM_PERIOD_US * (unsigned long)actDuty) / MAX_PWM;
    digitalWrite(PIN_ACT_PWM, phase < onFor ? HIGH : LOW);
}

static void writeBrushHardware(int duty) {
    if (duty <= 0)            digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    else if (duty >= MAX_PWM) digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
    else analogWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? duty : (MAX_PWM - duty));
}

void brushApply(int duty) {
    if (duty < 0) duty = 0;
    if (duty > MAX_PWM) duty = MAX_PWM;
    // A stop drops BOTH lines: on a two-input driver, DIR high with the gate
    // low is FORWARD AT FULL SCALE, which is what made the brush always-on.
    if (duty <= 0) {
        writeBrushHardware(0);
        digitalWrite(PIN_BRUSH_DIR, LOW);
        return;
    }
    digitalWrite(PIN_BRUSH_DIR, BRUSH_DIR_LEVEL);
    if (BRUSH_MIN_DUTY > 0) {
        duty = BRUSH_MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - BRUSH_MIN_DUTY)) / (MAX_PWM - 1));
    }
    writeBrushHardware(duty);
}

void lightApply(int level) {
    if (level < 0) level = 0;
    if (level > MAX_PWM) level = MAX_PWM;
    digitalWrite(PIN_LIGHT_DIR, LOW);       // return leg - stays LOW, always
    if (level == 0) digitalWrite(PIN_LIGHT_PWM, LOW);
    else            analogWrite(PIN_LIGHT_PWM, level);
}

void safeState() {
    digitalWrite(PIN_PWM1, LOW);
    digitalWrite(PIN_PWM2, LOW);
    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, LOW);
    actuatorApply(0, INVERT_ACT);
    brushApply(0);
    lightApply(0);
    setLinkLed(false);
}

void setLinkLed(bool on) { digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW); }

void outputsBegin() {
    // Brush first: its off level is LOW, and digitalWrite(LOW) on a still-input
    // pin only disables the pull-up and leaves it floating. pinMode first is
    // safe here precisely because the load is active-high.
    pinMode(PIN_BRUSH_PWM, OUTPUT);
    digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(PIN_BRUSH_DIR, OUTPUT);
    digitalWrite(PIN_BRUSH_DIR, LOW);

    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, LOW);
    digitalWrite(PIN_PWM1, LOW);
    digitalWrite(PIN_PWM2, LOW);
    digitalWrite(PIN_ACT_DIR, ACT_LEVEL_EXTEND);
    digitalWrite(PIN_ACT_PWM, LOW);
    digitalWrite(PIN_LIGHT_DIR, LOW);
    digitalWrite(PIN_LIGHT_PWM, LOW);

    pinMode(PIN_DIR1, OUTPUT);      pinMode(PIN_DIR2, OUTPUT);
    pinMode(PIN_PWM1, OUTPUT);      pinMode(PIN_PWM2, OUTPUT);
    pinMode(PIN_ACT_DIR, OUTPUT);   pinMode(PIN_ACT_PWM, OUTPUT);
    pinMode(PIN_LIGHT_DIR, OUTPUT); pinMode(PIN_LIGHT_PWM, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
}
