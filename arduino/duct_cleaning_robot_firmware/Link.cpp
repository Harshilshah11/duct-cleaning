#include "Link.h"
#include "Commands.h"
#include "Outputs.h"
#include "Timers.h"

byte          NET_MAC[6]     = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
const uint8_t NET_IP[4]      = {192, 168, 50, 20};
const uint8_t NET_SUBNET[4]  = {255, 255, 255, 0};
const uint8_t NET_GATEWAY[4] = {192, 168, 50, 1};

// ===========================================================================
// Shared line assembly for the USB port.
// ===========================================================================
// Used as the command transport on the serial build and as a drain on the
// Ethernet one.
// THE ASSEMBLY BUFFER IS PRIVATE, and that is a fix rather than a style choice.
//
// A serial line arrives one character at a time across many passes of loop(),
// so it needs somewhere to accumulate that survives between calls. The original
// sketches accumulated straight into the SHARED `packet` buffer that the UDP
// reader also writes — so on the Ethernet build a datagram landing mid-line
// overwrote the half-typed command underneath it. That was harmless only
// because SERIAL_COMMANDS was false and the result was discarded anyway; it
// would have corrupted real commands the moment anyone flipped it true to get
// the second wire back.
//
// Costs RX_BUFFER bytes of SRAM. There is comfortably room, and it makes the
// dual-transport option in Config.h actually usable rather than a trap.
static char     rxLine[RX_BUFFER];
static uint16_t rxLen = 0;
static bool     rxOverflow = false;

static bool pumpSerialLine(char *buf, uint16_t bufSize) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (rxOverflow) {           // discard the tail of an over-long line
                rxLen = 0;
                rxOverflow = false;
                continue;
            }
            rxLine[rxLen] = '\0';
            uint16_t n = rxLen;
            rxLen = 0;
            if (n == 0) continue;       // bare newline, nothing to hand back
            // Hand the completed line to the caller's buffer.
            uint16_t lim = (n < bufSize - 1) ? n : (uint16_t)(bufSize - 1);
            memcpy(buf, rxLine, lim);
            buf[lim] = '\0';
            return true;
        }
        if (rxLen >= sizeof(rxLine) - 1) {
            // Swallow an over-long line to its newline rather than parse a
            // truncated one: "CMD 7 M 200 2" is a VALID parse of a truncated
            // "CMD 7 M 200 250".
            rxOverflow = true;
            continue;
        }
        rxLine[rxLen++] = c;
    }
    return false;
}

#if LINK_TRANSPORT == LINK_ETHERNET
// ===========================================================================
// ETHERNET (W5100/W5500) — the rig
// ===========================================================================
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <utility/w5100.h>

static EthernetUDP   udp;
static unsigned long lastUdpMs = 0;
static unsigned long lastEthTryMs = 0;

// HAS ANY PACKET EVER ARRIVED SINCE BOOT?
//
// It separates two silences the recovery path used to treat identically. Before
// first contact the ground station may simply still be booting — the Pi needs
// the better part of a minute, while this board is listening within about five
// seconds of power-on. Resetting the shield through that window resets a chip
// that was never broken, which is what turned a normal cold start into "it does
// not connect until I cycle the Uno a few times".
//
// After first contact the meaning inverts: a link that WAS working has gone
// quiet, and that is the wedge worth resetting for.
static bool everHeard = false;

/* HAS THE LIBRARY ACTUALLY IDENTIFIED THE CHIP? This is the precondition for
 * every other register access in this file.
 *
 * W5100Class::init() ends with `chip = 51` for a W5100 — or `chip = 0` when
 * detection failed. Every register access dispatches on that value:
 *
 *     if (chip == 51) { ...W5100 framing... }
 *     else if (chip == 52) { ...W5200... }
 *     else { ...W5500 framing... }        <-- chip == 0 LANDS HERE
 *
 * There is no case for 0. A failed detection therefore does not disable
 * register access, it silently switches it to the WRONG PROTOCOL: every read
 * and write afterwards is W5500 framing aimed at a W5100. Writes vanish and
 * reads return rubbish indistinguishable from data — so a reset would write
 * MR.RST into nothing, read a stray 0 back, and report success. The board
 * announced a healthy shield it had never once spoken to.
 *
 * init() is the ONLY thing that can set chip back to 51, and Ethernet.begin()
 * re-runs it because its `initialized` guard is still false after a failure.
 * So: chip==0 is cured by begin(), NEVER by a reset. */
static bool chipDetected() {
    return W5100.getChip() == 51;
}

/* Reading SIPR back is a real check: it is the address Ethernet.begin() should
 * have written into the chip. If it does not read back, the configuration never
 * landed and the shield needs a genuine reset rather than another no-op. */
static bool shieldConfigured() {
    if (!chipDetected()) return false;      // nothing below is meaningful
    uint8_t got[4];
    W5100.readSIPR(got);
    return got[0] == NET_IP[0] && got[1] == NET_IP[1]
        && got[2] == NET_IP[2] && got[3] == NET_IP[3];
}

static bool resetShield() {
    if (!chipDetected()) return false;      // a reset cannot reach chip==0

    // Plain delay(1), not REAL_MS(1). EVERY caller of this function runs under
    // a StockTimer0 guard or before timersBegin(), so a millisecond is a
    // millisecond here. Scaling it would ask for 50 x 64 ms and stall the loop.
    W5100.writeMR(0x80);                    // RST — the chip resets itself
    for (uint8_t i = 0; i < 50; i++) {      // datasheet clears in well under 10ms
        if (W5100.readMR() == 0) break;
        delay(1);
    }
    if (W5100.readMR() != 0) return false;  // never came out of reset
    W5100.setMACAddress(NET_MAC);
    W5100.writeSIPR(NET_IP);
    W5100.writeSUBR(NET_SUBNET);
    W5100.writeGAR(NET_GATEWAY);
    return true;
}

void linkBegin() {
    // CALLED BEFORE timersBegin(), AND THAT ORDERING IS LOad-BEARING. Timer0 is
    // still on its stock prescaler here, so delay() inside the Ethernet library
    // — the 560 ms CAT811 reset-pulse wait in init(), and softReset()'s ~20 ms
    // completion poll — gets the real time it was written for. Bring the shield
    // up with Timer0 already at prescaler 1 and those become 8.75 ms and
    // 0.31 ms, which is what made the board deaf until someone pressed reset.
    // See StockTimer0 in Timers.h.
    delay(ETH_SETTLE_MS);

    IPAddress ip(NET_IP[0], NET_IP[1], NET_IP[2], NET_IP[3]);
    // Static IP, no DHCP: Ethernet.begin(mac, ip) cannot fail or block, unlike
    // the DHCP form which stalls ~60 s when no server answers. On a
    // point-to-point tether there is no DHCP server at all.
    Ethernet.begin(NET_MAC, ip);

    // WHAT THE SHIELD ACTUALLY REPORTS. This is the line that separates "no
    // power / dead chip" from "chip fine, cable out", and it had never been
    // looked at — which is why the cold-start fault was guessed at for an hour
    // instead of read off the board.
    Serial.print(F("W5100: "));
    EthernetHardwareStatus hw = Ethernet.hardwareStatus();
    if (hw == EthernetNoHardware) {
        Serial.print(F("NOT DETECTED (no power to the shield, or not seated)"));
    } else if (hw == EthernetW5100) {
        Serial.print(F("W5100 ok"));
    } else {
        Serial.print(F("detected, type "));
        Serial.print((int)hw);
    }
    Serial.print(F("   link: "));
    EthernetLinkStatus ls = Ethernet.linkStatus();
    Serial.println(ls == LinkON ? F("UP")
                                : (ls == LinkOFF ? F("DOWN") : F("unknown")));

    // BRING THE SHIELD UP, treating the two failures that look identical from
    // outside as the different faults they are:
    //
    //   A) THE CHIP WAS NEVER DETECTED (chip==0). No reset can reach it. The
    //      one and only cure is another init(), which Ethernet.begin() performs.
    //   B) DETECTED BUT ITS CONFIG DID NOT LAND. Registers work, so a real
    //      MR.RST reset is both possible and the right move.
    //
    // Getting these backwards is why this took so long: a reset aimed at case A
    // does nothing and reports success, which reads as "the shield is fine"
    // while the board sits deaf. Each pass re-tests rather than assuming.
    for (uint8_t tries = 1; tries <= 8; tries++) {
        if (!chipDetected()) {
            Ethernet.begin(NET_MAC, ip);        // case A
        } else if (!shieldConfigured()) {
            resetShield();                      // case B
            Ethernet.begin(NET_MAC, ip);
        } else {
            break;                              // detected AND configured
        }
        Serial.print(F("W5100 try "));
        Serial.print(tries);
        Serial.print(F(": chip="));
        Serial.print(W5100.getChip());
        Serial.println(shieldConfigured() ? F(" configured") : F(" not yet"));
        delay(ETH_RETRY_GAP_MS);
    }

    // chip= IS THE MOST DIAGNOSTIC NUMBER THIS BOARD PRINTS. 51 means the
    // library is talking W5100 framing to a W5100 and everything downstream can
    // be believed. 0 means detection failed, every later register value is
    // fiction, and the fault is the rail or the shield — not this sketch.
    Serial.print(F("W5100: chip="));
    Serial.print(W5100.getChip());
    Serial.println(shieldConfigured() ? F(" config VERIFIED (IP reads back)")
                                      : F(" NOT VERIFIED - link will not work"));

    udp.begin(NET_LISTEN_PORT);
    lastUdpMs = millis();

    // NO BOOT-TIME PACKET PROBE HERE, AND IT MUST NOT COME BACK. It waited for a
    // packet and reset the chip if none came, reasoning that the Pi transmits at
    // 50 Hz so silence must mean a deaf shield. THAT FAILS AT EXACTLY THE MOMENT
    // THAT MATTERS: when the rig is powered on, this board is listening about
    // five seconds later while the Pi is still most of a minute from starting
    // the ground station. The probe read that normal silence as a fault and
    // fired six chip resets into a healthy shield. A board must be able to come
    // up on its own; the chip is verified against its own registers above, which
    // needs nobody else to be awake.
}

bool linkReceive(char *buf, uint16_t bufSize) {
    int size = udp.parsePacket();
    if (size <= 0) return false;

    int n = udp.read(buf, bufSize - 1);
    if (n < 0) n = 0;
    buf[n] = '\0';
    // Anything longer than the buffer is still queued in the W5x00; drop the
    // remainder so the next parsePacket() starts on a clean packet boundary.
    if (size > n) udp.flush();

    lastUdpMs = millis();   // proof the shield is alive — see ETH_REINIT_MS
    everHeard = true;       // first contact: silence now means a wedge
    return true;
}

void linkAck(uint16_t seq) {
    // STRAIGHT BACK TO WHOEVER SENT IT, never to a hardcoded Pi address. The
    // Pi's sending socket is on an ephemeral port, so a fixed reply port would
    // land nowhere. This also means any machine on the LAN can bench-test the
    // link without the ground station.
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(F("ACK "));
    udp.print(seq);
    udp.print(F("\n"));
    udp.endPacket();
}

bool linkAuxSerialLine(char *buf, uint16_t bufSize) {
    return pumpSerialLine(buf, bufSize);
}

void linkAuxAck(uint16_t seq) {
    Serial.print(F("ACK "));
    Serial.println(seq);
}

void linkService() {
    // See ETH_REINIT_MS. Deliberately independent of linkUp: the state it
    // recovers is a shield that NEVER came up, which nothing else would notice.
    unsigned long now = millis();

    // PATIENT BEFORE FIRST CONTACT, PROMPT AFTER IT. A rig powered on all at
    // once leaves this board waiting on a Pi that is still booting; that
    // silence is normal and must not be treated as a fault.
    const unsigned long quietFor = everHeard ? ETH_REINIT_MS : ETH_COLD_WAIT_MS;
    if (now - lastUdpMs <= quietFor || now - lastEthTryMs <= quietFor) return;
    lastEthTryMs = now;

    // RELEASE THE SOCKET BEFORE REOPENING IT. This one line is the whole fix
    // for "the Uno answers ping but the ground station never reconnects".
    //
    // MEASURED 2026-08-26: after the Pi's eth0 went down and came back, the
    // W5100 kept answering ICMP — its IP stack is in hardware and never stopped
    // — while its UDP SOCKET stayed deaf. tcpdump showed the Pi sending at
    // 50 Hz and not one reply coming back, indefinitely.
    //
    // WHY THE OLD RE-INIT DID NOT CLEAR IT: EthernetUDP::begin() looks for a
    // FREE socket. The wedged one was still allocated, so begin() found
    // nothing, returned 0, and did nothing at all — every 5 seconds, forever.
    // It looked like a working retry and was a no-op.
    udp.stop();

    // SAME TWO FAULTS AS AT BOOT, same two cures — see chipDetected().
    IPAddress ip(NET_IP[0], NET_IP[1], NET_IP[2], NET_IP[3]);
    // Timer0 back to stock for the repair so the library's delays are real.
    // Safe here: this block cannot run until the link has been silent for
    // ETH_REINIT_MS, by which time the failsafe stopped every output long ago.
    StockTimer0 realTime;
    if (!chipDetected()) {
        Ethernet.begin(NET_MAC, ip);        // only init() can recover chip==0
        udp.begin(NET_LISTEN_PORT);
        Serial.print(F("LINK SILENT - chip undetected, init retried: chip="));
        Serial.println(W5100.getChip());
    } else {
        bool wasReset = resetShield();
        Ethernet.begin(NET_MAC, ip);
        udp.begin(NET_LISTEN_PORT);
        Serial.print(F("LINK SILENT - shield reset "));
        Serial.print(wasReset ? F("OK, socket reopened") : F("FAILED"));
        Serial.println(shieldConfigured() ? F(", configured")
                                          : F(", NOT configured"));
    }

    // NO WATCHDOG RESET HERE, AND IT MUST NOT COME BACK IN THIS FORM. Tried
    // 2026-08-26 to force the hardware reset Ethernet.begin() does not perform,
    // and REMOVED THE SAME HOUR: it put the board in a RESET LOOP — 6 boots in
    // 35 seconds, banners truncated mid-print. On this AVR a watchdog reset
    // leaves the timer ARMED with its short timeout, and the bootloader takes
    // longer than that to hand over, so the board resets again before it ever
    // reaches setup(). Clearing MCUSR and calling wdt_disable() first does not
    // help — execution never gets that far.
    //
    // THE CURE WAS WORSE THAN THE DISEASE: a wedged shield still leaves a
    // working board on USB; a reset loop kills every transport at once.
    //
    // If the shield wedge needs solving, the honest options are hardware: wire a
    // spare pin to the shield's RESET line and pulse it, or fix the 5V rail so
    // the W5100 comes up cleanly at all — which on this rig is the real root
    // cause. Do not reach for the watchdog again.
}

bool linkWaitForGroundStation(char *buf, uint16_t bufSize) {
    if (!LINK_WAIT_FOR_GROUND_STATION) return true;

    Serial.println(F("WAIT: holding in setup() until the ground station speaks"));
    Serial.println(F("WAIT: the shield is only reset if it PROVES faulty, not because it is quiet"));

    const unsigned long start = millis();
    unsigned long lastNote    = start;
    unsigned long lastCheck   = start;
    uint16_t      repairs     = 0;

    for (;;) {
        // The rod is the one output synthesised in software. It is at zero
        // here, so this just holds the gate low — but it costs nothing and
        // means it is never left unattended while we block.
        outputsService();

        // Drain the USB port so a terminal cannot fill the RX buffer while we
        // sit here. Discarded: SERIAL_COMMANDS decides whether it may drive,
        // and that is loop()'s business, not the wait's.
        char scratch[RX_BUFFER];
        while (pumpSerialLine(scratch, RX_BUFFER)) { /* discard */ }

        // --- the thing we are actually waiting for -------------------------
        if (linkReceive(buf, bufSize)) {
            unsigned int seq = 0;
            if (handleCommand(buf, &seq)) {
                // ACK it, so the link is proven in BOTH directions before we
                // hand over. handleCommand() has also set lastPacketMs and
                // linkUp, so the failsafe starts armed and correct rather than
                // counting from a boot that happened minutes ago.
                linkAck(lastSeq);
                Serial.print(F("WAIT: ground station is up, seq="));
                Serial.print(seq);
                Serial.print(F(", waited "));
                Serial.print((millis() - start) / MILLIS_SCALE);
                Serial.print(F(" ms, repairs="));
                Serial.println(repairs);
                return true;
            }
            // A datagram that did not parse proves the shield hears, which is
            // most of what we came for — but not that the sender is the ground
            // station. Keep waiting for one we understand.
            //
            // RATE-LIMITED, sharing the progress line's budget. A sender
            // pushing malformed frames at 50 Hz would otherwise print 50 lines
            // a second, and Serial.print BLOCKS once it outruns the 64-byte TX
            // buffer — so the noise would slow the very loop that is trying to
            // catch a good frame.
            if (millis() - lastNote >= LINK_WAIT_NOTE_MS) {
                lastNote = millis();
                Serial.println(F("WAIT: datagram received but not understood"));
            }
        }

        unsigned long now = millis();

        // --- is the shield genuinely broken? -------------------------------
        // ONLY POSITIVE EVIDENCE COUNTS. Silence is not evidence: on a cold
        // start the Pi is a minute behind this board, and resetting a healthy
        // W5100 through that window is the documented fault this wait was
        // nearly a repeat of.
        if (now - lastCheck >= LINK_WAIT_RECHECK_MS) {
            lastCheck = now;
            IPAddress ip(NET_IP[0], NET_IP[1], NET_IP[2], NET_IP[3]);
            // Timer0 back to stock for the whole repair, so the library's
            // internal delays are real milliseconds. Nothing is moving here.
            StockTimer0 realTime;
            if (!chipDetected()) {
                // chip==0: the library is addressing it with the wrong
                // protocol. Only init() can recover this, never a reset.
                Ethernet.begin(NET_MAC, ip);
                udp.begin(NET_LISTEN_PORT);
                repairs++;
                Serial.print(F("WAIT: chip undetected, init retried -> chip="));
                Serial.println(W5100.getChip());
            } else if (!shieldConfigured()) {
                // Registers work but the config never landed, so a real MR.RST
                // reset is both possible and the right move.
                bool ok = resetShield();
                Ethernet.begin(NET_MAC, ip);
                udp.begin(NET_LISTEN_PORT);
                repairs++;
                Serial.print(F("WAIT: config not in the chip, reset "));
                Serial.println(ok ? F("OK") : F("FAILED"));
            } else {
                // THE CHIP IS FINE AND THE SOCKET MAY NOT BE, and that third
                // case is why this branch is not empty.
                //
                // The W5x00 wedges its UDP socket while its IP stack keeps
                // running: measured 2026-08-26, the board still answered ICMP
                // while tcpdump showed the Pi sending at 50 Hz with not one
                // reply, indefinitely. chipDetected() and shieldConfigured()
                // are BOTH true throughout, so neither branch above fires.
                //
                // linkService() recovers this in loop() — but this wait blocks
                // BEFORE loop(), so leaving the branch empty meant a wedged
                // socket hung the board forever where the old firmware
                // recovered in five seconds. That is a regression this wait
                // introduced, and this is the fix for it.
                //
                // stop() FIRST, and that ordering is the whole trick:
                // EthernetUDP::begin() looks for a FREE socket, so with the
                // wedged one still allocated it finds none, returns 0, and does
                // nothing at all — a retry that looks like a retry and is a
                // no-op.
                //
                // SAFE ON A HEALTHY-BUT-QUIET LINK, which is what makes it
                // usable here. Reopening a socket touches no chip state and
                // costs no reset; at worst it drops one in-flight datagram out
                // of the 50 the Pi sends every second. It does NOT weaken the
                // "never reset a healthy shield" rule this function exists to
                // keep — no reset happens on this path.
                udp.stop();
                udp.begin(NET_LISTEN_PORT);
                Serial.println(F("WAIT: shield healthy, socket reopened in case it wedged"));
            }
        }

        // --- keep the console alive ----------------------------------------
        if (now - lastNote >= LINK_WAIT_NOTE_MS) {
            lastNote = now;
            Serial.print(F("WAIT: "));
            Serial.print((now - start) / MILLIS_SCALE / 1000);
            Serial.print(F("s, chip="));
            Serial.print(W5100.getChip());
            Serial.print(shieldConfigured() ? F(" configured") : F(" NOT configured"));
            Serial.print(F(", link="));
            EthernetLinkStatus ls = Ethernet.linkStatus();
            Serial.print(ls == LinkON ? F("UP") : (ls == LinkOFF ? F("DOWN") : F("unknown")));
            Serial.println(F(" - no command yet"));
        }

        // --- give up? -------------------------------------------------------
        if (LINK_WAIT_TIMEOUT_MS != 0 && (now - start) >= LINK_WAIT_TIMEOUT_MS) {
            Serial.println(F("WAIT: timed out, running anyway - loop() handles a dead link"));
            return false;
        }
    }
}

#else
// ===========================================================================
// USB SERIAL — the bench
// ===========================================================================
// No shield, so none of the W5100 recovery applies. The port that is a console
// on the Ethernet build carries the commands here, which is the one hard
// constraint this build adds: SERIAL_BAUD and the Pi's UNO_BAUD are a single
// setting living in two files. Change one, reflash the other, or the link is
// dead rather than slow.

void linkBegin() {
    Serial.println(F("LINK: USB serial (no Ethernet shield in this build)"));
}

bool linkReceive(char *buf, uint16_t bufSize) {
    return pumpSerialLine(buf, bufSize);
}

void linkAck(uint16_t seq) {
    Serial.print(F("ACK "));
    Serial.println(seq);
}

/* The port IS the transport on this build, and linkReceive() has already taken
 * its bytes. Returning false keeps loop()'s second branch dead. */
bool linkAuxSerialLine(char *, uint16_t) { return false; }

void linkAuxAck(uint16_t seq) {
    Serial.print(F("ACK "));
    Serial.println(seq);
}

void linkService() { /* nothing to recover */ }

bool linkWaitForGroundStation(char *buf, uint16_t bufSize) {
    if (!LINK_WAIT_FOR_GROUND_STATION) return true;

    // No shield to repair on this build, so there is nothing to retry — the
    // port is either being written to or it is not. The wait is still worth
    // having: it makes "the bench script is running" visible at boot.
    Serial.println(F("WAIT: holding in setup() until the Pi sends a command"));

    const unsigned long start = millis();
    unsigned long lastNote = start;

    for (;;) {
        outputsService();

        if (linkReceive(buf, bufSize)) {
            unsigned int seq = 0;
            if (handleCommand(buf, &seq)) {
                linkAuxAck(lastSeq);
                Serial.print(F("WAIT: link is up, seq="));
                Serial.println(seq);
                return true;
            }
        }

        unsigned long now = millis();
        if (now - lastNote >= LINK_WAIT_NOTE_MS) {
            lastNote = now;
            Serial.print(F("WAIT: "));
            Serial.print((now - start) / MILLIS_SCALE / 1000);
            Serial.println(F("s - no command yet"));
        }
        if (LINK_WAIT_TIMEOUT_MS != 0 && (now - start) >= LINK_WAIT_TIMEOUT_MS) {
            Serial.println(F("WAIT: timed out, running anyway"));
            return false;
        }
    }
}

#endif
