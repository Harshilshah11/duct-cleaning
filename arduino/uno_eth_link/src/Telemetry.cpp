#include "Telemetry.h"

Telemetry::Telemetry(unsigned long minIntervalMs, unsigned long heartbeatMs)
    : _minIntervalMs(minIntervalMs), _heartbeatMs(heartbeatMs), _lastMs(0),
      _pLeft(0), _pRight(0), _pAct(0), _pBrush(0), _pLight(0) {}

void Telemetry::report(int left, int right, int act, int brush, int light,
                       unsigned long packets) {
    const unsigned long now = millis();

    const bool changed = (left  != _pLeft  || right != _pRight ||
                          act   != _pAct   || brush != _pBrush ||
                          light != _pLight)
                         && (now - _lastMs) > _minIntervalMs;

    if (!changed && (now - _lastMs) <= _heartbeatMs) return;

    _pLeft  = left;
    _pRight = right;
    _pAct   = act;
    _pBrush = brush;
    _pLight = light;
    _lastMs = now;

    // F() keeps every one of these literals in flash. On a 2 KB part the string
    // table is the difference between fitting and not.
    Serial.print(F("L="));
    Serial.print(left);
    Serial.print(F(" R="));
    Serial.print(right);
    Serial.print(F(" ACT="));
    Serial.print(act);
    Serial.print(F(" BRUSH="));
    Serial.print(brush);          // duty 0..255, not ON/off
    Serial.print(F(" LIGHT="));
    Serial.print(light);
    Serial.print(F("  pkts="));
    Serial.println(packets);
}
