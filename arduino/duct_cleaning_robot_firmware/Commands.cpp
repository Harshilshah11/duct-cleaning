#include "Commands.h"
#include "Config.h"
#include "Outputs.h"

AppliedState  applied = {0, 0, 0, 0, 0};
unsigned long packetsReceived = 0;
uint16_t      lastSeq = 0;
bool          linkUp = false;
unsigned long lastPacketMs = 0;

void arcadeMix(int x, int y, int *left, int *right) {
    long l = (long)y + (long)x;
    long r = (long)y - (long)x;

    // Scale the PAIR, not each wheel, in ONE division. peak starts at 1000
    // (== max(1.0, ...) in the Python) so a stick inside full scale is passed
    // through untouched; only an over-range pair is scaled down.
    //
    // Keep this as a single divide. Normalising to 1000 first and converting to
    // PWM second truncates twice and drifts from uno_serial.py by a count or so
    // across the range, which is exactly the silent desync this function's
    // must-match rule exists to prevent.
    long peak = 1000;
    if (labs(l) > peak) peak = labs(l);
    if (labs(r) > peak) peak = labs(r);

    // NOTE: C truncates where the Python mix() rounds, so the two can differ by
    // one count. That divergence predates this refactor and is preserved rather
    // than fixed, because changing it here alone would widen the gap it sits in.
    *left  = (int)(l * MAX_PWM / peak);
    *right = (int)(r * MAX_PWM / peak);
}

bool handleCommand(const char *text, unsigned int *seqOut) {
    unsigned int seq = 0;
    int left = 0, right = 0, jx = 0, jy = 0, act = 0, brush = 0, light = 0;
    bool understood = false;

    int nf = sscanf(text, "CMD %u M %d %d %d %d %d",
                    &seq, &left, &right, &act, &brush, &light);
    if (nf >= 3) {
        // Trailing fields are optional — an older sender that only knows about
        // wheels still drives, and the channels it never heard of stay off.
        if (nf < 4) act = 0;
        if (nf < 5) brush = 0;
        if (nf < 6) light = 0;
        motorApply(PIN_DIR1, PIN_PWM1, left,  INVERT_1);
        motorApply(PIN_DIR2, PIN_PWM2, right, INVERT_2);
        actuatorApply(act, INVERT_ACT);
        brushApply(brush);
        lightApply(light);
        applied.left = left; applied.right = right; applied.act = act;
        applied.brush = brush; applied.light = light;
        understood = true;
    } else if (sscanf(text, "CMD %u J %d %d", &seq, &jx, &jy) == 3) {
        arcadeMix(jx, jy, &left, &right);
        motorApply(PIN_DIR1, PIN_PWM1, left,  INVERT_1);
        motorApply(PIN_DIR2, PIN_PWM2, right, INVERT_2);
        // The J form carries no auxiliary channels, so they are commanded off
        // rather than left at whatever the last M frame set.
        actuatorApply(0, INVERT_ACT);
        brushApply(0);
        lightApply(0);
        applied.left = left; applied.right = right;
        applied.act = 0; applied.brush = 0; applied.light = 0;
        understood = true;
    } else if (sscanf(text, "CMD %u STOP", &seq) == 1) {
        applied.left = 0; applied.right = 0; applied.act = 0;
        applied.brush = 0; applied.light = 0;
        safeState();
        understood = true;
    } else if (sscanf(text, "CMD %u", &seq) == 1) {
        understood = true;          // bare keepalive — refreshes the failsafe
    }

    if (understood) {
        *seqOut = seq;
        lastSeq = (uint16_t)seq;
        packetsReceived++;
        lastPacketMs = millis();
        linkUp = true;
        setLinkLed(true);
    }
    return understood;
}
