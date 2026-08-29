#pragma once
#include <Arduino.h>

/* Wire forms:
 *   CMD <seq> M <l> <r> [act] [brush] [light]
 *   CMD <seq> J <x> <y>
 *   anything else with a valid seq  -> STOP
 */
struct AppliedState { int left, right, act, brush, light; };
extern AppliedState  applied;
extern unsigned long packetsReceived;
extern uint16_t      lastSeq;
extern bool          linkUp;
extern unsigned long lastPacketMs;

bool handleCommand(const char *text, unsigned int *seqOut);

// Must stay numerically identical to mix() in ground_station/uno_serial.py.
void arcadeMix(int x, int y, int *left, int *right);
