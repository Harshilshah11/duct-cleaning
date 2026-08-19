#pragma once
#include <Arduino.h>

/*
 * CommandParser — turns one received datagram into a typed Command.
 *
 * WIRE FORMS (all newline-terminated on the wire; the terminator is not needed
 * here because each datagram is parsed whole):
 *
 *   CMD <seq> M <l> <r>                        wheels only
 *   CMD <seq> M <l> <r> <act>                  + actuator, sign only
 *   CMD <seq> M <l> <r> <act> <brush>          + brush, 0..255
 *   CMD <seq> M <l> <r> <act> <brush> <light>  + light, 0..255
 *   CMD <seq> J <x> <y>                        raw stick, -1000..1000
 *   CMD <seq> STOP                             explicit neutral
 *   CMD <seq>                                  bare keepalive
 *
 * Parsing is pure: it touches no hardware and no globals, which is what makes
 * it testable off the board.
 */

enum class CommandKind : uint8_t {
    None = 0,   // not understood
    Motors,     // M form
    Joystick,   // J form
    Stop,       // STOP
    Keepalive   // bare CMD <seq>, refreshes the failsafe without changing demand
};

struct Command {
    CommandKind  kind;
    unsigned int seq;
    int left, right;          // Motors
    int act, brush, light;    // Motors, trailing and optional
    int jx, jy;               // Joystick
};

/* Returns true if the text was understood, filling `out`. Returns false and
 * leaves `out.kind == CommandKind::None` otherwise. */
bool parseCommand(const char *text, Command &out);
