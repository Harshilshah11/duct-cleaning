#include "Link.h"
#include "Commands.h"
#include "Outputs.h"
#include "Timers.h"

byte          NET_MAC[6]     = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x20};
const uint8_t NET_IP[4]      = {192, 168, 50, 20};
const uint8_t NET_SUBNET[4]  = {255, 255, 255, 0};
const uint8_t NET_GATEWAY[4] = {192, 168, 50, 1};

// Private assembly buffer: a line arrives across many loop passes, and sharing
// the caller's buffer let a datagram overwrite a half-typed command.
static char     rxLine[RX_BUFFER];
static uint16_t rxLen = 0;
static bool     rxOverflow = false;

static bool pumpSerialLine(char *buf, uint16_t bufSize) {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (rxOverflow) { rxLen = 0; rxOverflow = false; continue; }
            rxLine[rxLen] = '\0';
            uint16_t n = rxLen;
            rxLen = 0;
            if (n == 0) continue;
            uint16_t lim = (n < bufSize - 1) ? n : (uint16_t)(bufSize - 1);
            memcpy(buf, rxLine, lim);
            buf[lim] = '\0';
            return true;
        }
        // Swallow an over-long line: a truncated one still parses, wrongly.
        if (rxLen >= sizeof(rxLine) - 1) { rxOverflow = true; continue; }
        rxLine[rxLen++] = c;
    }
    return false;
}

#if LINK_TRANSPORT == LINK_ETHERNET
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <utility/w5100.h>

uint8_t  linkVerdict = 0;
uint16_t linkServiceCount = 0;
static EthernetUDP   udp;
static unsigned long lastUdpMs = 0;
static unsigned long lastEthTryMs = 0;
static bool          everHeard = false;   // before first contact, silence is normal

/* chip==0 means detection failed, and the library then talks W5500 framing to a
 * W5100 - reads return rubbish rather than failing. Only init() clears it. */
static bool chipDetected() { return W5100.getChip() == 51; }

static bool shieldConfigured() {
    if (!chipDetected()) return false;
    uint8_t got[4];
    W5100.readSIPR(got);
    return got[0] == NET_IP[0] && got[1] == NET_IP[1]
        && got[2] == NET_IP[2] && got[3] == NET_IP[3];
}

static bool resetShield() {
    if (!chipDetected()) return false;      // a reset cannot reach chip==0
    W5100.writeMR(0x80);
    for (uint8_t i = 0; i < 50; i++) {      // plain delay: callers hold stock timing
        if (W5100.readMR() == 0) break;
        delay(1);
    }
    if (W5100.readMR() != 0) return false;
    W5100.setMACAddress(NET_MAC);
    W5100.writeSIPR(NET_IP);
    W5100.writeSUBR(NET_SUBNET);
    W5100.writeGAR(NET_GATEWAY);
    return true;
}

/* Runs BEFORE timersBegin(), so the library's own delays are real. No blocking
 * settle wait: the retry loop below buys the same time adaptively. */
void linkBegin() {
    // DESELECT THE SHIELD'S microSD FIRST. D4 is the card's chip select AND the
    // right wheel's direction line, and outputsBegin()/safeState() park it LOW -
    // which ASSERTS chip select. A card in the slot then drives MISO through
    // every W5100 transaction below, so detection reads rubbish and chip lands
    // on 0. Safe to force high here: PWM2 is low, so the wheel cannot turn.
    digitalWrite(PIN_DIR2, HIGH);

    IPAddress ip(NET_IP[0], NET_IP[1], NET_IP[2], NET_IP[3]);
    Ethernet.begin(NET_MAC, ip);            // static: cannot block like DHCP

    for (uint8_t tries = 1; tries <= 8; tries++) {
        if (!chipDetected())            Ethernet.begin(NET_MAC, ip);  // only init() helps
        else if (!shieldConfigured()) { resetShield(); Ethernet.begin(NET_MAC, ip); }
        else break;
        delay(ETH_RETRY_GAP_MS);
    }

    // chip= is the most diagnostic number this board prints: 51 good, 0 fiction.
    Serial.print(F("W5100 chip="));
    Serial.print(W5100.getChip());
    Serial.println(shieldConfigured() ? F(" VERIFIED") : F(" NOT VERIFIED"));

    linkVerdict = shieldConfigured() ? 2 : (chipDetected() ? 1 : 0);
    udp.begin(NET_LISTEN_PORT);
    lastUdpMs = millis();
}

bool linkReceive(char *buf, uint16_t bufSize) {
    int size = udp.parsePacket();
    if (size <= 0) return false;
    int n = udp.read(buf, bufSize - 1);
    if (n < 0) n = 0;
    buf[n] = '\0';
    if (size > n) udp.flush();              // keep the next parsePacket aligned
    lastUdpMs = millis();
    everHeard = true;
    return true;
}

// Back to the sender's port - the Pi's socket is ephemeral.
void linkAck(uint16_t seq) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(F("ACK "));
    udp.print(seq);
    udp.print('\n');
    udp.endPacket();
}

uint16_t linkChipId() { return W5100.getChip(); }

bool linkAuxSerialLine(char *buf, uint16_t bufSize) { return pumpSerialLine(buf, bufSize); }

void linkAuxAck(uint16_t seq) { Serial.print(F("ACK ")); Serial.println(seq); }

void linkService() {
    unsigned long now = millis();
    const unsigned long quietFor = everHeard ? ETH_REINIT_MS : ETH_COLD_WAIT_MS;
    if (now - lastUdpMs <= quietFor || now - lastEthTryMs <= quietFor) return;
    lastEthTryMs = now;
    linkServiceCount++;

    StockTimer0 realTime;   // library delays must be real; outputs already stopped
    IPAddress ip(NET_IP[0], NET_IP[1], NET_IP[2], NET_IP[3]);

    // stop() FIRST: begin() looks for a FREE socket, so a wedged one makes it a
    // silent no-op. This is the fix for "answers ping, never reconnects".
    udp.stop();
    if (!chipDetected()) Ethernet.begin(NET_MAC, ip);
    else { resetShield(); Ethernet.begin(NET_MAC, ip); }
    udp.begin(NET_LISTEN_PORT);

    Serial.print(F("LINK SILENT - reinit chip="));
    Serial.println(W5100.getChip());
    // No watchdog reset here: it looped the board 6 boots in 35 s.
}

#else

uint8_t  linkVerdict = 2;
uint16_t linkServiceCount = 0;
uint16_t linkChipId() { return 51; }
void linkBegin() { Serial.println(F("LINK: USB serial")); }
bool linkReceive(char *buf, uint16_t bufSize) { return pumpSerialLine(buf, bufSize); }
void linkAck(uint16_t seq) { Serial.print(F("ACK ")); Serial.println(seq); }
bool linkAuxSerialLine(char *, uint16_t) { return false; }
void linkAuxAck(uint16_t seq) { Serial.print(F("ACK ")); Serial.println(seq); }
void linkService() {}

#endif
