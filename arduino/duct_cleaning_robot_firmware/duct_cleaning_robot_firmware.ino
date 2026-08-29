/*
 * duct_cleaning_robot_firmware — Arduino Uno, the duct-cleaning robot.
 *
 * Drives two wheels, a linear actuator, a cleaning brush and a panel lamp from
 * ground station joystick data, and stops everything if the tether goes quiet.
 *
 *   Pi -> Uno  "CMD <seq> M <l> <r> <act> <brush> <light>\n"   trailing optional
 *   Pi -> Uno  "CMD <seq> J <x> <y>\n"                         raw stick
 *   Pi -> Uno  "CMD <seq> STOP\n"                              explicit neutral
 *   Uno -> Pi  "ACK <seq>\n"
 *
 * ONE SKETCH, TWO TRANSPORTS. Set LINK_TRANSPORT in Config.h to LINK_ETHERNET
 * (the rig — UDP on 192.168.50.20:5005) or LINK_SERIAL (the bench — the USB
 * tether). This folder replaced three near-identical sketches on 2026-08-29;
 * they had drifted apart, and the copy that drifted was always the one nobody
 * was testing that week.
 *
 * WHERE TO LOOK
 *   Config.h      every pin and every tuning number, with the reasoning
 *   Timers.cpp    the three PWM timers — and the line that makes millis() 64x fast
 *   Outputs.cpp   wheels, rod, brush, lamp, safeState()
 *   Commands.cpp  the wire protocol and the arcade mixer
 *   Link.cpp      the transport, and the W5100 recovery the rig needs
 *   Telemetry.cpp the L=/R=/ACT= line
 *   ResetCause.cpp why the board restarted — brown-out vs true power loss
 *
 * BEFORE YOU FLASH, TWO THINGS THAT WILL BITE:
 *   1. RUN WITH THE microSD SLOT EMPTY. D4 is the right wheel's direction line
 *      AND the shield's card select. See the pin map in Config.h.
 *   2. TEST THE FAILSAFE WITH THE WHEELS OFF THE GROUND. Unplug the tether
 *      mid-drive and confirm everything stops within a third of a second.
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
    // Boot bookkeeping FIRST, before anything can use RAM for other purposes.
    resetCauseBeginBoot();

    // Outputs before Serial: the brush must be held off as early as possible,
    // and outputsBegin() opens by driving it as a real output. See the note
    // there — until this runs, every pin is a floating input.
    outputsBegin();

    Serial.begin(SERIAL_BAUD);
    resetCauseReport();

    safeState();

    // NO SD DESELECT HERE. D4 is the right wheel's direction line and
    // safeState() has already parked it LOW. Run with the slot empty.

    // ETHERNET COMES UP BEFORE THE TIMERS ARE TOUCHED, AND THAT ORDER IS THE
    // FIX FOR "it does not start until I press reset".
    //
    // timersBegin() puts Timer0 on prescaler 1, which makes delay() run 64x
    // fast — and the W5100 driver measures two things with delay() that it
    // genuinely needs: init()'s 560 ms wait for the shield's CAT811 reset
    // supervisor to let go, and softReset()'s ~20 ms poll for the chip to
    // finish resetting. Bring the shield up after the switch and those become
    // 8.75 ms and 0.31 ms. On a cold, slowly-rising rail the chip cannot answer
    // that fast, so detection fails, chip is set to 0, and the library spends
    // the rest of the boot talking W5500 framing at a W5100 — a deaf shield
    // with a perfectly good cable. Press the reset button once the rail is up
    // and the chip answers immediately, which is exactly why that worked.
    //
    // Nothing here needs fast PWM: every output is stopped and stays stopped.
    linkBegin();

    // NOW the timers. After this line every bare millis()/micros()/delay() in
    // this sketch is 64x fast — see MILLIS_SCALE in Config.h. Anything that
    // re-enters the Ethernet library from here on wraps itself in a StockTimer0
    // guard (see Timers.h) to get real milliseconds back.
    timersBegin();

    // HOLD HERE UNTIL THE GROUND STATION IS ACTUALLY TALKING, retrying for as
    // long as it takes. Safe to block: every output reached a stopped level in
    // outputsBegin() and safeState(), and nothing can command them from here.
    //
    // It repairs the shield only when the shield PROVES faulty — see
    // LINK_WAIT_FOR_GROUND_STATION in Config.h, which carries the story of the
    // 2026-08-27 probe that reset a healthy W5100 six times because the Pi was
    // still booting. Waiting was never the problem; resetting on silence was.
    //
    // Falling through without a link is not a failure either: loop()'s failsafe
    // and linkService() are built for exactly that. Set LINK_WAIT_TIMEOUT_MS if
    // you would rather it gave up.
    linkWaitForGroundStation(packet, RX_BUFFER);
}

void loop() {
    // First thing every pass: the rod's stage is synthesised in software, so
    // the more often this runs the cleaner its duty. Everything else in this
    // loop is either instant or rate-limited.
    outputsService();

    // --- commands --------------------------------------------------------
    // DRAIN THE QUEUE, DO NOT TAKE ONE PACKET PER PASS. Operator, 2026-08-29,
    // on the robot dropping out while being driven: "the issue was of an
    // acknowledgement from the rpi" — which is exactly where it is.
    //
    // The Pi judges the robot connected on ACK RATE: fewer than ACK_MIN_PCT
    // (20%) of the last ACK_WINDOW (50) frames answered inside ACK_TIMEOUT_S
    // (200 ms) and it declares a dead link. So this loop does not merely need to
    // keep up on average — every frame it fails to answer in time counts against
    // that window.
    //
    // One packet per pass ties the ACK rate to the LOOP rate, and this loop is
    // not fast: LOOP_PAUSE_US is a real 10 ms, and a telemetry line at 115200
    // adds ~8 ms, so a printing pass is ~18 ms — about 55 a second against a Pi
    // sending 50. That is not a margin, it is a coincidence. Once anything
    // jitters, the board falls behind and STAYS behind, because consumption
    // never exceeds production: the W5100's queue only grows, ACK latency climbs
    // past 200 ms, and the rate collapses through the threshold. The symptom is
    // a link that works until it is used.
    //
    // Draining decouples the two. However far behind the board is, one pass
    // catches it up, so ACK latency stays bounded by the loop period rather than
    // growing without limit.
    //
    // BOUNDED, because an unbounded drain is just a different stall — a flood
    // would hold the loop here and starve outputsService() and the failsafe.
    // Eight is far above the ~1 per pass that 50 Hz delivers, so the bound is
    // only reached while catching up from a backlog.
    //
    // EVERY packet is ACKed, not just the last of a batch: the Pi counts ACKs
    // per frame SENT, so swallowing a backlog silently would read as loss and
    // trip the very threshold this exists to stay clear of.
    for (uint8_t drained = 0; drained < 8; drained++) {
        if (!linkReceive(packet, RX_BUFFER)) break;
        unsigned int seq = 0;
        if (handleCommand(packet, &seq)) {
            linkAck(lastSeq);
        }
    }

    // The USB port on the Ethernet build. It is ALWAYS drained, so a terminal
    // typing at the board cannot fill the RX buffer and stall the link; whether
    // the line is then obeyed is SERIAL_COMMANDS, which is false by default on
    // that build ("data transfer only via ethernet"). Dead code on the serial
    // build, where linkReceive() above already took these bytes.
    if (linkAuxSerialLine(packet, RX_BUFFER) && SERIAL_COMMANDS) {
        unsigned int seq = 0;
        if (handleCommand(packet, &seq)) {
            linkAuxAck(lastSeq);
        }
    }

    // --- transport recovery ----------------------------------------------
    linkService();

    // --- failsafe ---------------------------------------------------------
    // millis() SUBTRACTION, never `millis() > last + FAILSAFE_MS`. Unsigned
    // wraparound makes the additive form compare wrong exactly once; this form
    // stays correct across the rollover.
    if (linkUp && (millis() - lastPacketMs) >= FAILSAFE_MS) {
        linkUp = false;
        applied.left = 0;
        applied.right = 0;
        applied.act = 0;
        applied.brush = 0;
        // The lamp too. safeState() genuinely turns it off, but the original
        // sketch zeroed only the other four here, so the telemetry line kept
        // reporting the last brightness after a failsafe had already darkened
        // it. Every other channel was zeroed; this one was an omission, and it
        // made the log disagree with the robot at exactly the moment the log
        // matters most. Reporting only — no output changes.
        applied.light = 0;
        safeState();
        Serial.print(F("LINK DOWN - failsafe after "));
        Serial.print(packetsReceived);
        Serial.println(F(" packets"));
    }

    telemetryReport();

    // --- pacing -----------------------------------------------------------
    // IT IS NOT delay(), AND THAT IS THE WHOLE POINT — see LOOP_PAUSE_US in
    // Config.h. delay() stops the world, and the world it stops includes
    // outputsService(), whose waveform is 4 ms. Servicing that once per pause
    // aliases the duty into a ~100 Hz square wave, which is the "brush powered
    // on off on off" symptom. Busy-waiting keeps the rod serviced at loop speed
    // throughout, so the pacing costs nothing.
    unsigned long pauseStart = micros();
    while (micros() - pauseStart < LOOP_PAUSE_US) {
        outputsService();
    }
    delay(1);
}
