#include "Telemetry.h"
#include "Config.h"
#include "Commands.h"

static int printedL = 0, printedR = 0, printedA = 0;
static int printedB = 0, printedLight = 0;
static unsigned long lastPrintMs = 0;

void telemetryReport() {
    // Off by default — see TELEMETRY_ENABLED. Returning here rather than at the
    // call site keeps loop() reading the same either way, and costs nothing: the
    // compiler folds a compile-time false into a dropped call.
    if (!TELEMETRY_ENABLED) return;

    unsigned long now = millis();

    bool changed = (applied.left  != printedL || applied.right != printedR
                 || applied.act   != printedA || applied.brush != printedB
                 || applied.light != printedLight)
                 && (now - lastPrintMs) > TELEMETRY_MIN_INTERVAL_MS;

    if (!changed && (now - lastPrintMs) <= TELEMETRY_HEARTBEAT_MS) return;

    printedL = applied.left;
    printedR = applied.right;
    printedA = applied.act;
    printedB = applied.brush;
    printedLight = applied.light;
    lastPrintMs = now;

    Serial.print(F("L="));      Serial.print(applied.left);
    Serial.print(F(" R="));     Serial.print(applied.right);
    Serial.print(F(" ACT="));   Serial.print(applied.act);
    Serial.print(F(" BRUSH=")); Serial.print(applied.brush);   // duty, not ON/off
    Serial.print(F(" LIGHT=")); Serial.print(applied.light);
    Serial.print(F("  pkts=")); Serial.println(packetsReceived);
}
