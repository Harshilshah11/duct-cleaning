/*
 * light_blink_test - light the panel lamp, then blink it. Nothing else runs.
 *
 * THE LAMP IS A TWO-LEG BRIDGE, not a direction line plus a gate. D8 is the
 * return leg and must stay LOW; D9 carries the drive. Hold D8 HIGH and the lamp
 * is brightest at zero demand - that is a real fault this rig has already had.
 *
 * EVERY OTHER OUTPUT IS PARKED FIRST. A test sketch that leaves the brush pins
 * floating will run the brush: its driver enables on a floating input, and the
 * pins are inputs from reset until something drives them.
 */

const uint8_t LIGHT_DIR = 8;    // return leg - LOW always
const uint8_t LIGHT_PWM = 9;    // drive leg

const uint8_t BRUSH_DIR = 2, BRUSH_PWM = 3;
const uint8_t DIR1 = 7, PWM1 = 6;
const uint8_t DIR2 = 4, PWM2 = 5;      // D4 is also the shield's SD CS
const uint8_t ACT_DIR = A3, ACT_PWM = A2;

const unsigned long ON_MS  = 1000;
const unsigned long OFF_MS = 1000;

void setup() {
    // Brush first and as outputs immediately - see the note above.
    pinMode(BRUSH_PWM, OUTPUT); digitalWrite(BRUSH_PWM, LOW);
    pinMode(BRUSH_DIR, OUTPUT); digitalWrite(BRUSH_DIR, LOW);

    // Wheels stopped: both lines low on each channel.
    pinMode(PWM1, OUTPUT); digitalWrite(PWM1, LOW);
    pinMode(DIR1, OUTPUT); digitalWrite(DIR1, LOW);
    pinMode(PWM2, OUTPUT); digitalWrite(PWM2, LOW);
    pinMode(DIR2, OUTPUT); digitalWrite(DIR2, LOW);

    // Rod: the gate is what powers it, so dropping A2 is a real stop.
    pinMode(ACT_PWM, OUTPUT); digitalWrite(ACT_PWM, LOW);
    pinMode(ACT_DIR, OUTPUT); digitalWrite(ACT_DIR, LOW);

    pinMode(LIGHT_DIR, OUTPUT); digitalWrite(LIGHT_DIR, LOW);
    pinMode(LIGHT_PWM, OUTPUT);

    Serial.begin(115200);
    Serial.println(F("light_blink_test: D8 LOW, blinking D9"));

    // On at start, so a working lamp is obvious before the blink begins.
    digitalWrite(LIGHT_PWM, HIGH);
    delay(2000);
}

void loop() {
    digitalWrite(LIGHT_PWM, HIGH);
    Serial.println(F("ON"));
    delay(ON_MS);
    digitalWrite(LIGHT_PWM, LOW);
    Serial.println(F("off"));
    delay(OFF_MS);
}
