#pragma once
#include <Arduino.h>
#include "Config.h"

/*
 * Link — the transport, and the ONLY thing that differs between what used to be
 * three separate sketches.
 *
 * Four calls make up the whole interface. loop() is written against them and
 * does not know which build it is in.
 */

/* Bring the transport up and print what it found. */
void linkBegin();

/* Poll for one command. Returns true and fills `buf` (NUL-terminated,
 * bufSize bytes) when a whole command is available. Non-blocking. */
bool linkReceive(char *buf, uint16_t bufSize);

/* ACK the sequence number back to whoever sent it. */
void linkAck(uint16_t seq);

/* Recovery housekeeping. Call every pass of loop(). No-op on serial. */
void linkService();

/* Block in setup() until the ground station actually sends something, retrying
 * to establish the link for as long as it takes.
 *
 * Returns true when a command was received and acted on, false if
 * LINK_WAIT_TIMEOUT_MS elapsed first (or the wait is disabled). Either way it is
 * safe to fall through into loop() — the failsafe and linkService() handle a
 * missing link on their own.
 *
 * REPAIRS THE SHIELD ONLY ON POSITIVE EVIDENCE OF A FAULT, never on silence
 * alone. See LINK_WAIT_FOR_GROUND_STATION in Config.h for why that distinction
 * is the whole design. */
bool linkWaitForGroundStation(char *buf, uint16_t bufSize);

/* Assemble one line from the USB port on the Ethernet build, where that port is
 * the SECOND transport rather than the first. Returns true when a whole line is
 * ready; loop() then decides whether to obey it (SERIAL_COMMANDS) or discard
 * it. Draining it either way is what stops a terminal typing at the board from
 * filling the RX buffer and stalling the Ethernet side.
 *
 * Always false on the serial build — there this port IS linkReceive and the
 * bytes have already been taken. */
bool linkAuxSerialLine(char *buf, uint16_t bufSize);

/* ACK back over USB, for a command that arrived that way. */
void linkAuxAck(uint16_t seq);
