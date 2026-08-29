#pragma once
#include <Arduino.h>

/* Rate-limited on change, plus a heartbeat. Printing costs loop time - it is
 * what starved the ACK rate before the RX drain was added. */
void telemetryReport();
