#include "UdpCommandLink.h"

UdpCommandLink::UdpCommandLink(uint16_t listenPort)
    : _listenPort(listenPort) {}

bool UdpCommandLink::begin(byte *mac, IPAddress ip) {
    Ethernet.begin(mac, ip);

    bool ok = true;
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println(F("ERROR: no Ethernet shield detected - check it is seated"));
        ok = false;
    } else if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println(F("WARN: Ethernet cable is not connected"));
    }

    _udp.begin(_listenPort);
    return ok;
}

int UdpCommandLink::receive(char *buf, uint16_t bufSize) {
    const int size = _udp.parsePacket();
    if (size <= 0) return 0;

    int n = _udp.read(buf, bufSize - 1);
    if (n < 0) n = 0;
    buf[n] = '\0';

    if (size > n) {
        // Anything longer than the buffer is still queued in the W5x00. Drop
        // the remainder so the next parsePacket() starts on a clean packet
        // boundary instead of returning the tail as a bogus command.
        _udp.flush();
        Serial.print(F("WARN: packet truncated, "));
        Serial.print(size);
        Serial.println(F(" bytes"));
    }
    return size;
}

void UdpCommandLink::ack(uint16_t seq) {
    _udp.beginPacket(_udp.remoteIP(), _udp.remotePort());
    _udp.print(F("ACK "));
    _udp.print(seq);
    _udp.print(F("\n"));
    _udp.endPacket();
}
