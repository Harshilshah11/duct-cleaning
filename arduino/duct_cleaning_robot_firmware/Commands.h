#pragma once
#include <Arduino.h>

/*
 * Commands — one received datagram or line, parsed and applied.
 *
 * WIRE FORMS (newline-terminated on the serial transport; one per datagram on
 * the UDP one):
 *
 *   CMD <seq> M <l> <r>                        wheels only
 *   CMD <seq> M <l> <r> <act>                  + actuator, sign only
 *   CMD <seq> M <l> <r> <act> <brush>          + brush, 0..255
 *   CMD <seq> M <l> <r> <act> <brush> <light>  + light, 0..255
 *   CMD <seq> J <x> <y>                        raw stick, -1000..1000
 *   CMD <seq> STOP                             explicit neutral
 *   CMD <seq>                                  bare keepalive
 *
 * Both transports run THIS code. Two copies of the ladder would drift, and the
 * one that drifted would be the one nobody was testing that week.
 */

/* What was last applied, for the telemetry line. */
struct AppliedState {
    int left, right, act, brush, light;
};
extern AppliedState applied;

extern unsigned long packetsReceived;
extern uint16_t      lastSeq;
extern bool          linkUp;
extern unsigned long lastPacketMs;

/* Parse `text` and drive the outputs. Returns true if it was understood, and
 * writes the sequence number to *seqOut. Refreshes the failsafe. */
bool handleCommand(const char *text, unsigned int *seqOut);

/* Arcade mix for the "J" form: stick (-1000..1000) -> wheels (-255..255).
 *
 * MUST STAY NUMERICALLY IDENTICAL TO mix() IN ground_station/uno_serial.py,
 * because BOTH feed this board: uno_motors.py mixes on the Pi and sends the "M"
 * form, while joystick_link.py sends the raw calibrated stick as "J" and
 * expects the mixing to happen here. Change one and you must change the other,
 * or the robot steers differently depending on which program is driving.
 *
 * y drives both wheels together, x drives them in opposition. Full deflection
 * on both axes would demand 2.0 from one wheel, so the PAIR is scaled down
 * together — clipping each wheel on its own instead bends the turn as the robot
 * speeds up. No deadband here: the Pi has already removed it, and applying a
 * second one would silently eat part of that calibration.
 */
void arcadeMix(int x, int y, int *left, int *right);
