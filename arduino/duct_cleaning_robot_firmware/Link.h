#pragma once
#include <Arduino.h>
#include "Config.h"

extern uint8_t  linkVerdict;
extern uint16_t linkServiceCount;   // how many times the shield was re-inited
uint16_t linkChipId();              // live W5100 chip id, 0 = undetected   // 0 deaf, 1 detected-not-configured, 2 good
void linkBegin();
bool linkReceive(char *buf, uint16_t bufSize);
void linkAck(uint16_t seq);
void linkService();                                  // shield recovery; no-op on serial
bool linkAuxSerialLine(char *buf, uint16_t bufSize); // USB as second wire; false on serial
void linkAuxAck(uint16_t seq);
