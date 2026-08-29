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
    // Scale the PAIR in ONE divide; peak starts at 1000 (== max(1.0,..)).
    long peak = 1000;
    if (labs(l) > peak) peak = labs(l);
    if (labs(r) > peak) peak = labs(r);
    *left  = (int)(l * MAX_PWM / peak);
    *right = (int)(r * MAX_PWM / peak);
}

// Hand-rolled to keep sscanf out of the binary - it costs ~1.5 KB of flash.
static void skipSpace(const char *&p) { while (*p == ' ' || *p == '\t') p++; }

static bool parseInt(const char *&p, int &out) {
    skipSpace(p);
    bool neg = false;
    if (*p == '-') { neg = true; p++; } else if (*p == '+') p++;
    if (*p < '0' || *p > '9') return false;
    long v = 0;
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    out = (int)(neg ? -v : v);
    return true;
}

bool handleCommand(const char *text, unsigned int *seqOut) {
    const char *p = text;
    // No leading skipSpace: sscanf would not skip before a literal either.
    if (p[0] != 'C' || p[1] != 'M' || p[2] != 'D') return false;
    p += 3;

    int seq;
    if (!parseInt(p, seq) || seq < 0) return false;

    int v[5] = {0, 0, 0, 0, 0};
    skipSpace(p);
    char kind = *p;
    int left = 0, right = 0;

    if (kind == 'M') {
        p++;
        uint8_t n = 0;
        while (n < 5 && parseInt(p, v[n])) n++;
        if (n < 2) kind = 0;                    // truncated -> stop, as sscanf did
        else { left = v[0]; right = v[1]; }
    } else if (kind == 'J') {
        p++;
        int jx, jy;
        if (parseInt(p, jx) && parseInt(p, jy)) arcadeMix(jx, jy, &left, &right);
        else kind = 0;
    }

    if (kind == 'M' || kind == 'J') {
        motorApply(PIN_DIR1, PIN_PWM1, left,  INVERT_1);
        motorApply(PIN_DIR2, PIN_PWM2, right, INVERT_2);
        actuatorApply(kind == 'M' ? v[2] : 0, INVERT_ACT);
        brushApply(kind == 'M' ? v[3] : 0);
        lightApply(kind == 'M' ? v[4] : 0);
        applied.left  = left;
        applied.right = right;
        applied.act   = kind == 'M' ? v[2] : 0;
        applied.brush = kind == 'M' ? v[3] : 0;
        applied.light = kind == 'M' ? v[4] : 0;
    } else {
        // STOP, and anything not understood after a valid seq. Matches the old
        // sscanf ladder, where the keepalive branch was unreachable.
        applied.left = applied.right = applied.act = 0;
        applied.brush = applied.light = 0;
        safeState();
    }

    *seqOut = (unsigned int)seq;
    lastSeq = (uint16_t)seq;
    packetsReceived++;
    lastPacketMs = millis();
    linkUp = true;
    setLinkLed(true);
    return true;
}
