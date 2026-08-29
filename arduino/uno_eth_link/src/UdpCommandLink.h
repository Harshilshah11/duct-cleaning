#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

/*
 * UdpCommandLink — the W5100/W5500 transport: receive a datagram, ACK it.
 *
 * THE ACK GOES BACK TO THE SENDER'S ADDRESS AND PORT, never to a hardcoded Pi
 * address. The Pi's sending socket is on an ephemeral port, so a fixed reply
 * port would land nowhere. This also means any machine on the LAN can test the
 * link — which is how the board gets bench-tested without the ground station.
 *
 * Static IP, no DHCP: Ethernet.begin(mac, ip) cannot fail or block, unlike the
 * DHCP form which stalls ~60 s when no server answers. On a point-to-point
 * tether there is no DHCP server at all.
 */
class UdpCommandLink {
public:
    explicit UdpCommandLink(uint16_t listenPort);

    /* Brings up the interface and reports hardware trouble rather than sitting
     * mute. Returns false if no shield was detected. */
    bool begin(byte *mac, IPAddress ip);

    /* Poll for one datagram.
     *
     * Returns the DATAGRAM's size (0 if nothing arrived), and fills `buf` with
     * a NUL-terminated copy of as much as fits. The return value is the size on
     * the wire, NOT the number of bytes copied, so a caller can tell a
     * truncated packet from a short one. */
    int receive(char *buf, uint16_t bufSize);

    void ack(uint16_t seq);

    IPAddress localIP() { return Ethernet.localIP(); }

private:
    EthernetUDP    _udp;
    const uint16_t _listenPort;
};
