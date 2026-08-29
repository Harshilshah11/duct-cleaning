#pragma once
#include <Arduino.h>

/*
 * Telemetry — the bench log over USB serial.
 *
 * WHY IT REPORTS THE APPLIED DEMAND rather than just link state: "LINK UP" and
 * "LINK DOWN" alone cannot tell "commands are flowing" from "nothing ever
 * arrived", because LINK DOWN only fires on a transition OUT of LINK UP — both
 * cases are silent in the second one. These lines print what was actually
 * applied, which is also what you read when deciding whether a channel needs
 * inverting.
 *
 * Rate-limited on change (a moving stick would otherwise flood the port at
 * 50 Hz) and emitted on a heartbeat regardless, so a resting link still proves
 * itself.
 *
 * PRINTING COSTS LOOP TIME. Serial.print blocks once it outruns the 64-byte TX
 * buffer, and loop() also hand-rolls the soft-PWM for the rod and the brush, so
 * a slow port stalls a PWM edge. At 9600 one of these lines is ~57 ms of wire
 * time; at 250000 it is ~2.2 ms. That is the real reason the baud matters on
 * this build — the commands themselves arrive over UDP, not over this port.
 */
class Telemetry {
public:
    Telemetry(unsigned long minIntervalMs, unsigned long heartbeatMs);

    /* Print if something changed (and the rate limit allows) or if the
     * heartbeat is due. Cheap and non-blocking otherwise. */
    void report(int left, int right, int act, int brush, int light,
                unsigned long packets);

private:
    const unsigned long _minIntervalMs;
    const unsigned long _heartbeatMs;
    unsigned long       _lastMs;
    int _pLeft, _pRight, _pAct, _pBrush, _pLight;
};
