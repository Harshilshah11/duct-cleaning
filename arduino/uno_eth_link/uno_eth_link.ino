/*
 * uno_eth_link — Arduino Uno + W5100/W5500 Ethernet shield, driving a dual
 * channel motor driver from ground station joystick data (guide Steps 8-9).
 *
 *   Pi -> Uno  "CMD <seq> M <l> <r>"                        wheels only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act>"                  + actuator, sign only
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush>"          + brush, 0..255
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush> <light>"  + light, 0..255
 *   Pi -> Uno  "CMD <seq> J <x> <y>"                        raw stick, -1000..1000
 *   Pi -> Uno  "CMD <seq> STOP"                             explicit neutral
 *   Uno -> Pi  "ACK <seq>"                                  back to sender addr/port
 *
 * EVERY OUTPUT RIDES IN THE SAME PACKET, as trailing fields on M. They are
 * deliberately not separate datagrams: one packet per frame keeps a SINGLE
 * failsafe clock covering wheels, actuator and brush together, and keeps the
 * traffic at one datagram per 20 ms. A drive command that stopped refreshing
 * some other output's private timer would let that output keep running after
 * the tether died — which for the rod means driving into its end stop, and for
 * the brush means a spinning brush nobody can stop.
 *
 * ---------------------------------------------------------------------------
 * STRUCTURE — where to look for what
 * ---------------------------------------------------------------------------
 *
 *   src/Config.h         every pin and tuning number, with the reasoning that
 *                        justifies it. START HERE. This is the file that was
 *                        paid for in debugging sessions.
 *
 *   src/MotorChannel     one DIR+PWM wheel channel: deadband, minimum-duty
 *                        stretch, Timer0-safe stop.
 *   src/SoftPwmPin       software PWM for pins with no timer behind them,
 *                        shared by the rod's gate and the brush's speed line.
 *   src/LinearActuator   the rod. Extends on DIR **LOW** — its own class so
 *                        MotorChannel's opposite convention cannot leak in.
 *   src/BrushMotor       brush: constant direction, unsigned duty.
 *   src/PanelLight       lamp: no deadband, certain-dark off.
 *   src/RobotOutputs     where those generic parts meet Config.h and become
 *                        THIS rig. Port the sketch by rewriting this + Config.h.
 *
 *   src/CommandParser    wire text -> typed Command. Pure; no hardware.
 *   src/ArcadeMixer      stick -> wheels, for the J form only.
 *   src/LinkFailsafe     "have I heard from the ground station recently?"
 *   src/UdpCommandLink   the W5x00 transport: receive, ACK to the sender.
 *   src/Telemetry        the rate-limited bench log.
 *
 * For the M form, arcade mixing happens on the Pi
 * (ground_station/uno_motors.py) and this sketch only applies per-wheel
 * demands. Steering can then be retuned without a reflash, and the code next to
 * the motors stays small enough to audit. The J form is the exception:
 * joystick_link.py sends the calibrated but UNMIXED stick, so ArcadeMixer does
 * that one job — see its header for why the two must stay identical.
 *
 * TWO WARNINGS THAT COST DAYS, BOTH DETAILED IN src/Config.h:
 *   - RUN WITH THE microSD SLOT EMPTY. D4 is both the rod's gate and the
 *     shield's SD chip select.
 *   - The rod EXTENDS on DIR LOW, opposite every other channel here.
 *
 * Build: Arduino IDE, board "Arduino Uno", stock Ethernet library. The src/
 * subfolder is compiled automatically — no library needs installing.
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "src/Config.h"
#include "src/RobotOutputs.h"
#include "src/CommandParser.h"
#include "src/ArcadeMixer.h"
#include "src/LinkFailsafe.h"
#include "src/Telemetry.h"
#include "src/UdpCommandLink.h"

// The MAC is locally administered (the 0x02 bit in the first byte) and must be
// unique on your LAN. Newer shields ship with a real MAC on a sticker — use
// that if yours has one.
byte      mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
IPAddress ip(192, 168, 50, 20);

static RobotOutputs   outputs;
static UdpCommandLink link(NET_LISTEN_PORT);
static LinkFailsafe   failsafe(FAILSAFE_MS);
static Telemetry      telemetry(TELEMETRY_MIN_INTERVAL_MS,
                                TELEMETRY_HEARTBEAT_MS);

static char packet[RX_BUFFER];

// The demand actually applied, kept for the telemetry line.
static int curL = 0, curR = 0, curA = 0, curB = 0, curLight = 0;

void setup() {
    Serial.begin(SERIAL_BAUD);

    outputs.begin();          // safe levels, then pinMode, then safeState()
    link.begin(mac, ip);

    // Everything that changes behaviour goes in the banner, because a silently
    // stale board is the expensive failure here: the link ACKs and the pins
    // look right whatever build is loaded, so a reset is the one moment
    // someone is watching.
    Serial.print(F("uno_eth_link listening on "));
    Serial.print(link.localIP());
    Serial.print(F(":"));
    Serial.println(NET_LISTEN_PORT);
    Serial.print(F("serial telemetry at "));
    Serial.print(SERIAL_BAUD);
    Serial.println(F(" baud - match the monitor or this reads as garbage"));
    Serial.println(F("DIR1=D9 PWM1=D3 (left)  DIR2=D8 PWM2=D6 (right)"));
    Serial.print(F("deadband<"));
    Serial.print(DEADBAND);
    Serial.print(F(", non-zero demand scaled to "));
    Serial.print(MIN_DUTY);
    Serial.print(F(".."));
    Serial.println(MAX_PWM);
    // Printed as the truth table rather than as two pin numbers, because the
    // bug this channel spent a day on was a WIRING SCHEME misread, not a wrong
    // pin: both pin numbers were right the whole time. A banner that said only
    // "ACT=D7/D4" would have looked correct on the broken build too.
    Serial.println(F("ACT_DIR=D7 ACT_PWM=D4 - LOW on D7 EXTENDS (opposite the wheels)"));
    Serial.print(F("  soft-PWM stages 0/"));
    Serial.print(ACT_DUTY_RETRACT);
    Serial.print(F("/"));
    Serial.print(ACT_DUTY_EXTEND);
    Serial.println(F(" (stop/retract/extend)"));
    // Loud, and in the banner rather than a comment, because the failure it
    // warns about looks like a flaky cable.
    Serial.println(F("  ^ D4 is the shield's SD chip select - RUN WITH THE SLOT EMPTY"));
    Serial.print(F("BRUSH_DIR=D2 BRUSH_PWM=A1 soft-PWM duty 0-"));
    Serial.print(MAX_PWM);
    Serial.print(F(" floor "));
    Serial.print(BRUSH_MIN_DUTY);
    Serial.println(F(" (TOGGLE on/off, Pi sends 0 or 255)"));
    Serial.println(F("LIGHT_DIR=A0 LIGHT_PWM=D5 (pot-dimmed, 0-255)"));
    Serial.print(F("failsafe after "));
    Serial.print(FAILSAFE_MS);
    Serial.println(F(" ms of silence"));
}

void loop() {
    // FIRST THING EVERY PASS: the rod's and the brush's duty are synthesised in
    // software, so the more often this runs the cleaner they are. Everything
    // else in this loop is either instant or rate-limited.
    outputs.service();

    const int size = link.receive(packet, RX_BUFFER);
    if (size > 0) {
        Command cmd;
        if (parseCommand(packet, cmd)) {
            switch (cmd.kind) {
            case CommandKind::Motors:
                outputs.applyAll(cmd.left, cmd.right, cmd.act, cmd.brush,
                                 cmd.light);
                curL = cmd.left;
                curR = cmd.right;
                curA = cmd.act;
                curB = cmd.brush;
                curLight = cmd.light;
                break;

            case CommandKind::Joystick: {
                // Raw stick from joystick_link.py: already centred and
                // deadbanded on the Pi, but NOT mixed. Mixing happens here for
                // this form only.
                int l = 0, r = 0;
                arcadeMix(cmd.jx, cmd.jy, MAX_PWM, &l, &r);
                // J carries no actuator, brush or light, so none of them run.
                outputs.applyAll(l, r, 0, 0, 0);
                curL = l;
                curR = r;
                curA = curB = curLight = 0;
                break;
            }

            case CommandKind::Stop:
                curL = curR = curA = curB = curLight = 0;
                outputs.safeState();
                break;

            case CommandKind::Keepalive:
                // Proves the link is alive and refreshes the failsafe without
                // changing any demand.
                break;

            case CommandKind::None:
                break;      // unreachable: parseCommand returned true
            }

            if (failsafe.feed()) {
                Serial.println(F("LINK UP"));
            }
            outputs.setLinkLed(true);
            link.ack((uint16_t)cmd.seq);
        } else {
            Serial.print(F("WARN: unparsable packet: "));
            Serial.println(packet);
        }
    }

    if (failsafe.expired()) {
        curL = curR = curA = curB = 0;
        // NOTE: curLight is deliberately NOT cleared here, preserving the
        // original behaviour exactly. safeState() below DOES switch the lamp
        // off, so for one telemetry line the log reports a light level that is
        // no longer being driven. Harmless, but know it before you read the log
        // during a brownout hunt — the lamp really is off.
        outputs.safeState();
        Serial.print(F("LINK DOWN - failsafe after "));
        Serial.print(failsafe.count());
        Serial.println(F(" packets"));
    }

    telemetry.report(curL, curR, curA, curB, curLight, failsafe.count());

    // Loop pause, on the operator's order 2026-08-18. See LOOP_PAUSE_MS in
    // src/Config.h for what it costs. Set that to 0 for a free-running loop.
    if (LOOP_PAUSE_MS > 0) delay(LOOP_PAUSE_MS);
}
