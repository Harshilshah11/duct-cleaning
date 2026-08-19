#include "CommandParser.h"

bool parseCommand(const char *text, Command &out) {
    out.kind  = CommandKind::None;
    out.seq   = 0;
    out.left  = out.right = 0;
    out.act   = out.brush = out.light = 0;
    out.jx    = out.jy = 0;

    unsigned int seq = 0;
    int l = 0, r = 0, a = 0, b = 0, li = 0;

    // ONE pattern for every length of M, judged by how many fields sscanf
    // actually filled.
    //
    // TESTING THE SHORT PATTERNS AS SEPARATE BRANCHES DOES NOT WORK, and this
    // is the subtle part: "CMD %u M %d %d" happily matches the LONGER string
    // too — sscanf simply stops early and returns 3 — so a shorter branch
    // placed first would silently swallow the trailing fields and freeze the
    // actuator, brush and light at their last values.
    //
    // Anything the sender omitted stays 0, which is also the safe default: a
    // sender that cannot talk about the actuator, the brush or the light must
    // never be able to leave any of them running.
    const int nf = sscanf(text, "CMD %u M %d %d %d %d %d",
                          &seq, &l, &r, &a, &b, &li);
    if (nf >= 3) {
        if (nf < 4) a  = 0;
        if (nf < 5) b  = 0;
        if (nf < 6) li = 0;
        out.kind  = CommandKind::Motors;
        out.seq   = seq;
        out.left  = l;
        out.right = r;
        out.act   = a;
        out.brush = b;
        out.light = li;
        return true;
    }

    int jx = 0, jy = 0;
    if (sscanf(text, "CMD %u J %d %d", &seq, &jx, &jy) == 3) {
        out.kind = CommandKind::Joystick;
        out.seq  = seq;
        out.jx   = jx;
        out.jy   = jy;
        return true;
    }

    if (sscanf(text, "CMD %u STOP", &seq) == 1) {
        out.kind = CommandKind::Stop;
        out.seq  = seq;
        return true;
    }

    // A bare keepalive with no payload. Valid: it proves the link is alive and
    // refreshes the failsafe without changing the motor demand.
    if (sscanf(text, "CMD %u", &seq) == 1) {
        out.kind = CommandKind::Keepalive;
        out.seq  = seq;
        return true;
    }

    return false;
}
