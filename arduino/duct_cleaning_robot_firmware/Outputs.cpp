#include "Outputs.h"
#include "Config.h"

// The duty currently demanded on PIN_ACT_PWM, 0..255. Written by
// actuatorApply(), acted on by outputsService() every pass of loop().
static int actDuty = 0;

// ---------------------------------------------------------------------------
// WHEELS
// ---------------------------------------------------------------------------
void motorApply(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert) {
    if (invert) demand = -demand;
    if (demand >  MAX_PWM) demand =  MAX_PWM;
    if (demand < -MAX_PWM) demand = -MAX_PWM;
    if (demand > -DEADBAND && demand < DEADBAND) demand = 0;

    digitalWrite(dirPin, demand >= 0 ? HIGH : LOW);

    int duty = demand >= 0 ? demand : -demand;
    if (duty == 0) {
        // NOT analogWrite(pwmPin, 0) — both wheels are on Timer0, where a zero
        // duty can still emit a narrow pulse and leave the motor creeping.
        digitalWrite(pwmPin, LOW);
        return;
    }
    if (MIN_DUTY > 0) {
        // Stretch 1..MAX_PWM onto MIN_DUTY..MAX_PWM so the slowest demand the
        // stick can express is still one the motor can act on.
        //
        // The multiply is promoted to long DELIBERATELY: 254 * 165 is 41910,
        // which overflows the Uno's 16-bit int and would wrap to a negative
        // duty — a wheel that runs backwards near full stick.
        duty = MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - MIN_DUTY)) / (MAX_PWM - 1));
    }
    analogWrite(pwmPin, duty);
}

// ---------------------------------------------------------------------------
// LINEAR ACTUATOR
// ---------------------------------------------------------------------------
void actuatorApply(int demand, bool invert) {
    if (invert) demand = -demand;

    if (demand == 0) {
        // ZERO IS A REAL STOP. PIN_ACT_PWM powers the channel, so dropping it
        // leaves the rod exactly where it is — what the panel's middle throw
        // means and what the failsafe needs. Not a brake, not a reversal.
        //
        // THE DIRECTION LINE IS DELIBERATELY NOT TOUCHED. Re-pointing a rod
        // that is no longer powered buys nothing, and holding the last
        // direction means a resumed command carries on the way it was going.
        actDuty = ACT_DUTY_STOP;
        digitalWrite(PIN_ACT_PWM, LOW);
        return;
    }

    // ACT_LEVEL_* rather than a bare HIGH/LOW because this channel extends on
    // LOW while every other direction line here goes forward on HIGH.
    digitalWrite(PIN_ACT_DIR, demand > 0 ? ACT_LEVEL_EXTEND : ACT_LEVEL_RETRACT);

    // Only the SIGN chooses the stage. The Pi sends full scale either way, so
    // reading a magnitude here would just be reading a constant.
    actDuty = demand > 0 ? ACT_DUTY_EXTEND : ACT_DUTY_RETRACT;
}

/* Software PWM for the rod, because A2 has no timer.
 *
 * 0 and 255 short-circuit to a static level, and that is a SAFETY property, not
 * an optimisation: a stopped rod must be held LOW by something a stalled loop
 * cannot freeze in the high half, and a full-speed rod should not be chopped by
 * a software timer at all. Only the middle stage is synthesised, where a stall
 * costs a slower rod, never a runaway one. */
void outputsService() {
    if (actDuty <= ACT_DUTY_STOP) {
        digitalWrite(PIN_ACT_PWM, LOW);
        return;
    }
    if (actDuty >= MAX_PWM) {
        digitalWrite(PIN_ACT_PWM, HIGH);
        return;
    }
    // micros() wraps about every 71 minutes of its own fast time; the modulo
    // makes that a single short cycle, not a stuck output, so it is left
    // unhandled deliberately.
    unsigned long phase = micros() % ACT_PWM_PERIOD_US;
    unsigned long onFor = (ACT_PWM_PERIOD_US * (unsigned long)actDuty) / MAX_PWM;
    digitalWrite(PIN_ACT_PWM, phase < onFor ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// BRUSH
// ---------------------------------------------------------------------------
/* The only place PIN_BRUSH_PWM is written. Both ENDPOINTS use digitalWrite
 * rather than analogWrite(0)/(255): on a timer pin a zero duty can still emit a
 * narrow pulse, and a hard level is the only certain stop and the only certain
 * full-on. */
static void writeBrushHardware(int duty) {
    if (duty <= 0) {
        digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    } else if (duty >= MAX_PWM) {
        digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? HIGH : LOW);
    } else {
        analogWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? duty : (MAX_PWM - duty));
    }
}

void brushApply(int duty) {
    if (duty < 0) duty = 0;
    if (duty > MAX_PWM) duty = MAX_PWM;

    // A STOP DRIVES BOTH LINES LOW, and that is the whole fix for "brush is
    // always on" (operator, 2026-08-29).
    //
    // This used to assert BRUSH_DIR unconditionally, on the reasoning that
    // direction is a constant for a brush that spins one way. So a stopped
    // brush sat at DIR = HIGH, PWM = LOW — and setup() left it there from reset.
    //
    // THAT IS ONLY "OFF" IF THE DRIVER TAKES DIR + ENABLE. Plenty of
    // dual-channel boards instead take two logic inputs, IN1 and IN2, where
    // HIGH/LOW is not "stopped pointing forwards" but FORWARD AT FULL SCALE. On
    // such a channel the old code commanded the brush to run from the moment
    // the Uno left reset, and no demand from the Pi could countermand it,
    // because every brushApply(0) wrote exactly the same pair of levels.
    //
    // Both lines LOW is a real stop under EITHER reading: IN1=IN2=LOW is coast
    // on a two-input driver, and enable LOW is off on a DIR+enable one, where
    // the direction line is then don't-care. Written as a stop of both pins
    // rather than as a polarity constant deliberately — it does not require
    // knowing which board is on the other end of the wire, and the wire is the
    // thing that keeps changing.
    if (duty <= 0) {
        writeBrushHardware(0);
        digitalWrite(PIN_BRUSH_DIR, LOW);
        return;
    }

    // Direction before speed. Both lines are written on every call, not just
    // the one that changed: a channel that browns out and comes back gets its
    // direction restored by the next frame rather than running whichever way
    // its input floated to. On a rig whose supply sags under motor load that is
    // not hypothetical.
    digitalWrite(PIN_BRUSH_DIR, BRUSH_DIR_LEVEL);
    if (BRUSH_MIN_DUTY > 0) {
        // Same stretch as motorApply(), same long-arithmetic overflow reason.
        duty = BRUSH_MIN_DUTY
             + (int)(((long)(duty - 1) * (MAX_PWM - BRUSH_MIN_DUTY))
                     / (MAX_PWM - 1));
    }
    writeBrushHardware(duty);
}

// ---------------------------------------------------------------------------
// PANEL LIGHT
// ---------------------------------------------------------------------------
void lightApply(int level) {
    if (level < 0) level = 0;
    if (level > MAX_PWM) level = MAX_PWM;

    // LIGHT_DIR IS THE RETURN LEG. IT STAYS LOW. ALWAYS. See Config.h for the
    // three rig observations that settled this — with DIR pinned high the knob
    // ran backwards, brightest at zero.
    digitalWrite(PIN_LIGHT_DIR, LOW);

    // NO DEADBAND, deliberately unlike motorApply(). A motor below ~12 buzzes
    // and heats without turning, so folding that to zero is right. A lamp at
    // 12/255 is simply dim, and the same rule would give the pot a dead patch
    // at the bottom of its travel that reads as a broken knob.
    if (level == 0) {
        digitalWrite(PIN_LIGHT_PWM, LOW);   // the only certain dark
    } else {
        analogWrite(PIN_LIGHT_PWM, level);
    }
}

// ---------------------------------------------------------------------------
// SAFE STATE
// ---------------------------------------------------------------------------
void safeState() {
    // digitalWrite, not analogWrite(pin, 0): both wheel PWM pins are on Timer0,
    // where a zero duty is not a guaranteed dead level. This is the one place
    // that must be certain.
    digitalWrite(PIN_PWM1, LOW);
    digitalWrite(PIN_PWM2, LOW);
    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, LOW);

    // The rod is genuinely STOPPED, not merely pointed somewhere: this drops
    // the gate line, which is this driver's off state. Before the channel was
    // rewired as a pair, a failsafe could only pick a direction and the rod ran
    // to its end stop.
    actuatorApply(0, INVERT_ACT);

    // A spinning brush is the loudest thing on the robot; it must not be what
    // survives a failsafe.
    brushApply(0);

    // Light out. It is the one output here that poses no motion hazard, so
    // leaving it lit was tempting — but the cameras stream over the SAME
    // tether, so once the link is down there is nobody left to see by it, and
    // this rig already browns out under load. Dark is cheaper.
    lightApply(0);

    setLinkLed(false);
}

void setLinkLed(bool on) {
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// BOOT
// ---------------------------------------------------------------------------
void outputsBegin() {
    // THE BRUSH IS SILENCED FIRST, BEFORE ANYTHING ELSE.
    //
    // Operator, 2026-08-27: "when bot on and off to brush motor is rotate
    // without on switch". The general rule below — write the stopped level
    // BEFORE pinMode, so the pin cannot glitch in the gap — is true for an
    // ACTIVE-LOW load, where digitalWrite(pin, HIGH) on a still-input pin
    // switches the pull-up on and weakly holds the line. It is NOT true here.
    // BRUSH_ACTIVE_HIGH is true, so the off level is LOW — and
    // digitalWrite(pin, LOW) on an input merely turns the pull-up OFF and
    // leaves the pin floating. Nothing holds it anywhere, and if the driver's
    // enable input drifts high, the brush runs.
    //
    // Driving it as a real output is the only thing that holds it. pinMode
    // first is safe in this direction precisely BECAUSE the load is
    // active-high: the port register powers up as 0, so the pin drives LOW the
    // instant it becomes an output.
    //
    // WHAT THIS CANNOT FIX: from the moment power arrives until this line
    // executes, the AVR is in its bootloader and EVERY pin is an input. That is
    // one to two seconds on a Uno and no sketch can shorten it. If the brush
    // still twitches at power-on, the fix is a PULL-DOWN RESISTOR (10k is
    // ample) from the driver's enable input to ground.
    pinMode(PIN_BRUSH_PWM, OUTPUT);
    digitalWrite(PIN_BRUSH_PWM, BRUSH_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(PIN_BRUSH_DIR, OUTPUT);
    // LOW, NOT BRUSH_DIR_LEVEL — see brushApply(). Parking the direction line
    // at its running level while the gate is low is a RUN command on a
    // two-input driver, and it is what made the brush spin from reset.
    digitalWrite(PIN_BRUSH_DIR, LOW);

    // Everything else: stopped level first, then pinMode.
    digitalWrite(PIN_DIR1, LOW);
    digitalWrite(PIN_DIR2, LOW);
    digitalWrite(PIN_PWM1, LOW);
    digitalWrite(PIN_PWM2, LOW);
    // BOTH rod lines must be at their stopped level before they become outputs,
    // for the same reason the brush's gate is driven first.
    digitalWrite(PIN_ACT_DIR, ACT_LEVEL_EXTEND);
    digitalWrite(PIN_ACT_PWM, LOW);
    digitalWrite(PIN_LIGHT_DIR, LOW);
    digitalWrite(PIN_LIGHT_PWM, LOW);

    pinMode(PIN_DIR1, OUTPUT);
    pinMode(PIN_DIR2, OUTPUT);
    pinMode(PIN_PWM1, OUTPUT);
    pinMode(PIN_PWM2, OUTPUT);
    pinMode(PIN_ACT_DIR, OUTPUT);
    pinMode(PIN_ACT_PWM, OUTPUT);
    pinMode(PIN_LIGHT_DIR, OUTPUT);
    pinMode(PIN_LIGHT_PWM, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
}
