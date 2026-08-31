/*
 * duct_cleaning_robot_firmware - Arduino Uno, duct-cleaning robot.
 *
 *   Pi -> Uno  "CMD <seq> M <l> <r> [act] [brush] [light]\n"
 *   Pi -> Uno  "CMD <seq> J <x> <y>\n"      raw stick, mixed here
 *   Uno -> Pi  "ACK <seq>\n"                every packet, not every batch
 *
 * Transport is one #define in Config.h. Two things that bite: millis() is 64x
 * fast (see MILLIS_SCALE), and D4 is the shield's SD chip select - RUN WITH THE
 * SLOT EMPTY. Test the failsafe with the wheels off the ground.
 */
#include "Config.h"
#include "Timers.h"
#include "ResetCause.h"
#include "Outputs.h"
#include "Commands.h"
#include "Link.h"
#include "Telemetry.h"


static char packet[RX_BUFFER];

void setup() {

    resetCauseBeginBoot();
    outputsBegin();
    Serial.begin(SERIAL_BAUD);
    resetCauseReport();
    safeState();

    // Ethernet BEFORE the timers: the W5100 driver measures its reset waits with
    // delay(), and timersBegin() makes delay() 64x fast. Bring it up after and
    // detection fails on a slow rail - a deaf shield with a good cable.
    linkBegin();
    timersBegin();
}

void loop() {
    outputsService();

    // DRAIN, do not take one packet per pass. The Pi declares the link dead
    // below a 20% ACK rate over 50 frames, so one-per-pass tied ACKs to the loop
    // rate and the board fell permanently behind. Bounded so a flood cannot
    // starve the failsafe; every packet is ACKed because the Pi counts per-frame.
    for (uint8_t drained = 0; drained < 8; drained++) {
        if (!linkReceive(packet, RX_BUFFER)) break;
        unsigned int seq = 0;
        if (handleCommand(packet, &seq)) linkAck(lastSeq);
    }

    // Always drained so a terminal cannot stall the link; obeyed only if enabled.
    if (linkAuxSerialLine(packet, RX_BUFFER) && SERIAL_COMMANDS) {
        unsigned int seq = 0;
        if (handleCommand(packet, &seq)) linkAuxAck(lastSeq);
    }

    linkService();

    // Subtraction, never millis() > last + N: that compares wrong across the wrap.
    if (linkUp && (millis() - lastPacketMs) >= FAILSAFE_MS) {
        linkUp = false;
        applied.left = applied.right = applied.act = 0;
        applied.brush = applied.light = 0;
        safeState();
        Serial.print(F("LINK DOWN after "));
        Serial.println(packetsReceived);
    }

    // HOLD the stop, do not just write it once. The trip above fires on an edge;
    // this re-drives every output to neutral for as long as the link stays down,
    // so a channel that browns out and recovers cannot come back running.
    {
        static unsigned long lastHoldMs = 0;
        if (!linkUp && (millis() - lastHoldMs) >= FAILSAFE_HOLD_MS) {
            lastHoldMs = millis();
            safeState();
        }
    }

    telemetryReport();

    resetCauseStash(linkVerdict, (uint16_t)packetsReceived, linkServiceCount,
                    linkChipId(), linkUp ? 1 : 0);


    // Busy-wait, not delay(): delay() would stop outputsService() too, and
    // servicing a 4 ms waveform once per pause aliases it into visible flicker.
    unsigned long pauseStart = micros();
    while (micros() - pauseStart < LOOP_PAUSE_US) outputsService();
}
