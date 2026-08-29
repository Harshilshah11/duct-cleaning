#pragma once
#include <Arduino.h>

/* Rules shared by every channel: direction is written BEFORE speed, and a full
 * stop uses digitalWrite(pin, LOW) never analogWrite(pin, 0) - on a timer pin a
 * zero duty can still emit a narrow pulse. */
void outputsBegin();
void outputsService();          // soft-PWM for the rod; call every loop pass
void motorApply(uint8_t dirPin, uint8_t pwmPin, int demand, bool invert);
void actuatorApply(int demand, bool invert);   // sign only; 0 stops and holds
void brushApply(int duty);
void lightApply(int level);
void safeState();
void setLinkLed(bool on);
