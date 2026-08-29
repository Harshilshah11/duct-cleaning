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
 * PRINTING COSTS LOOP TIME. Serial.print blocks once it outruns the 64-byte TX
 * buffer, and loop() also hand-rolls the soft-PWM for the rod, so a slow port
 * stalls a PWM edge. Rate-limited on change (a moving stick would otherwise
 * flood the port at 50 Hz) and emitted on a heartbeat regardless, so a resting
 * link still proves itself.
 */
void telemetryReport();
