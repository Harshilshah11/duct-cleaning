#include "Outputs.h"
#include "Config.h"

static int actDuty = 0;

// Brush soft-start state. brushTarget/brushCurrent are POST-stretch duties, so
// the ramp starts at 0 rather than jumping straight to BRUSH_MIN_DUTY.
static int           brushTarget = 0;
static int           brushCurrent = 0;
static unsigned long brushRampStart = 0;
static void brushService();

void motorApply(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert) {
    if (invert) demand = -demand;
    if (demand >  MAX_PWM) demand =  MAX_PWM;
    if (demand < -MAX_PWM) demand = -MAX_PWM;
    if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

    int duty = demand >= 0 ? demand : -demand;

    /* A STOPPED WHEEL DROPS BOTH LINES. This used to write DIR from the sign
     * before testing for zero, and `demand >= 0` takes the HIGH branch at zero,
     * so a wheel commanded to a standstill was left DIR HIGH with the gate LOW.
     * On this two-input driver that is FORWARD AT FULL SCALE - the same state
     * brushApply() drops both lines to avoid, twenty lines below, and a state
     * safeState() never uses: it writes both LOW.
     *
     * SO THE STOPPED WHEEL WAS THE ONE THAT RAN, at full speed, in whichever
     * direction DIR HIGH happens to mean. Only steering that commands a wheel to
     * exactly zero could hit it, which is why straight ahead and both spins were
     * always correct and only the DIAGONALS misbehaved - and why no amount of
     * sign-changing in uno_serial.mix() could fix them. The moving wheel was not
     * the one being commanded, so flipping the command's sign changed nothing.
     * That null result, 2026-09-01, is what pinned the fault to this line.
     *
     * IT ALSO MADE A CRAWL IMPOSSIBLE: DEADBAND below zeroes any small demand,
     * so a wheel asked to inch forward got full scale instead. */
    if (duty == 0) {
        digitalWrite(pwmPin, LOW);
        digitalWrite(dirPin, LOW);
        return;
    }

    if (MIN_DUTY > 0) {
        // long: 254 * 165 overflows a 16-bit int and wraps negative.
        duty = MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - MIN_DUTY)) / (MAX_PWM - 1));
    }

    /* THESE ARE THE TWO INPUTS OF AN H-BRIDGE, NOT A DIRECTION PIN AND A GATE.
     * The names say dir/pwm and that is what misled every fix before this one.
     * What the driver actually does with the pair:
     *
     *     LOW  + LOW   coast          HIGH + HIGH  brake
     *     LOW  + duty  drives, speed rises with duty
     *     HIGH + duty  drives THE OTHER WAY, speed rises as duty FALLS
     *
     * brushApply() below has known this since it was written - "DIR high with
     * the gate low is FORWARD AT FULL SCALE" - but motorApply drove the wheels
     * as if dirPin picked a direction and pwmPin set a speed. That gives one
     * working direction and one that lands on HIGH+HIGH, a brake:
     *
     *     demand -255  ->  LOW + 255   full speed          fine
     *     demand +255  ->  HIGH + 255  both high = BRAKE   no motion
     *
     * SO REVERSE NEVER WORKED. Flipping a sign in uno_serial.mix() swapped one
     * live direction for a standstill, which reads as "nothing changed" from the
     * driving seat - the operator reported exactly that twice, 2026-09-01, and
     * the second report is what sent me here. Hours of sign-flipping in Python
     * were chasing a wheel that had only ever had one direction to give.
     *
     * The cure is the complement, exactly as writeBrushHardware() does it: hold
     * one input LOW and PWM the other for one direction; hold it HIGH and PWM
     * the INVERSE for the other. Both ends stay correct - duty 0 gives HIGH+HIGH
     * (a brake, which is the stop the zero branch above has already taken) and
     * duty 255 gives HIGH+LOW, full scale the other way. */
    /* PLAIN DIR + PWM. The wheels really are on a direction pin and a speed
     * gate, and this line is what the driver wants.
     *
     * IT WAS BRIEFLY WRITTEN AS A TWO-INPUT H-BRIDGE on 2026-09-01 - hold one
     * input low and PWM the other, hold it high and PWM the complement, the way
     * writeBrushHardware() does. The theory was that reverse had never worked
     * because HIGH + full duty is a brake on such a bridge, which would have
     * explained why flipping signs in uno_serial.mix() kept changing nothing.
     *
     * IT IS DISPROVED, and simply: with the complement in place, full reverse
     * sends DIR HIGH with the gate at 0. On a two-input bridge that is full
     * speed; on a direction-and-speed driver it is a standstill. The operator
     * drove it and reported "now backward not work" - so it is the latter, and
     * the brush's two-input note does not generalise to the wheels. Do not
     * re-derive this: the drive motors and the brush are different drivers. */
    digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);
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
    brushService();
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

    // A stop drops BOTH lines, immediately and without ramping: on a two-input
    // driver, DIR high with the gate low is FORWARD AT FULL SCALE, and the
    // failsafe must be able to kill the brush in one call.
    if (duty <= 0) {
        brushTarget = brushCurrent = 0;
        writeBrushHardware(0);
        digitalWrite(PIN_BRUSH_DIR, LOW);
        return;
    }

    // Stretch 1..MAX_PWM onto BRUSH_MIN_DUTY..BRUSH_MAX_DUTY, so the smallest
    // demand already turns the brush and full demand stops at the ceiling.
    // long: (254 * 130) is 33020 and overflows a 16-bit int.
    int want = BRUSH_MIN_DUTY
             + (int)(((long)(duty - 1) * (BRUSH_MAX_DUTY - BRUSH_MIN_DUTY))
                     / (MAX_PWM - 1));
    if (want > BRUSH_MAX_DUTY) want = BRUSH_MAX_DUTY;
    // Clock starts when the brush leaves a standstill; a target that changes
    // mid-climb just re-aims the same ramp.
    if (brushCurrent <= 0) brushRampStart = millis();
    brushTarget = want;

    // Direction is written every call: a channel that browns out and recovers
    // gets it restored on the next frame instead of running whichever way its
    // input floated to.
    digitalWrite(PIN_BRUSH_DIR, BRUSH_DIR_LEVEL);

    // Only the climb is ramped. Any reduction lands at once.
    if (brushCurrent >= brushTarget) {
        brushCurrent = brushTarget;
        writeBrushHardware(brushCurrent);
    }
}

/* Walk the brush up to its target. Time-gated, so calling it every loop pass
 * and all through the busy-wait costs nothing. */
static void brushService() {
    if (brushCurrent >= brushTarget) return;
    // Duty from ELAPSED TIME, not one step per tick: the climb then takes
    // BRUSH_RAMP_MS whether this runs every pass or misses most of them. The
    // step-per-tick version drifted to ~6 s against a 2 s design.
    unsigned long elapsed = millis() - brushRampStart;
    int want = (elapsed >= BRUSH_RAMP_MS)
             ? brushTarget
             : (int)(((long)brushTarget * elapsed) / BRUSH_RAMP_MS);
    if (want <= brushCurrent) return;
    brushCurrent = want;
    writeBrushHardware(brushCurrent);
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
