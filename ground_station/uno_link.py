#!/usr/bin/env python3
"""
Pi -> Arduino Uno command link over Ethernet (guide Step 8).

The Uno has no networking of its own; the W5100/W5500 shield on top of it does.
That shield answers on 192.168.50.20:5005 on the isolated robot segment, so
"send data to the Arduino" means send a UDP datagram to that host and port.

UDP, not TCP, on purpose. A joystick sample is only interesting until the next
one arrives 20 ms later, so a lost packet must never be retransmitted ahead of
fresh data — which is exactly what TCP would do. Head-of-line blocking on a
motor command link shows up as the robot lurching to catch up after a hiccup.
Loss is handled by sending often and letting the sketch's failsafe cover a gap.

The sketch echoes `ACK <seq>` back to whichever address the packet came from, so
this side can prove delivery without anyone watching the Arduino's serial
monitor. That echo is the whole point of the ACK — it is not a retransmit
trigger and nothing here waits on it before sending the next packet.

Check the link from a terminal, no GUI needed:

    python3 uno_link.py                    # 10 Hz counter until Ctrl-C
    python3 uno_link.py --hz 50            # the rate Step 5-9 actually uses
    python3 uno_link.py --once "HELLO"     # single packet, print the ACK
    UNO_HOST=192.168.50.21 python3 uno_link.py

Exits non-zero if nothing ever answered, so it works in a shell test:

    python3 uno_link.py --once PING && echo "Arduino is alive"
"""

import argparse
import os
import socket
import sys
import time

# Static IPs from the guide (Step 2) — same values as config.py, repeated here
# so this file still runs when copied to a Pi on its own.
UNO_HOST = os.environ.get("UNO_HOST", "192.168.50.20")
UNO_PORT = int(os.environ.get("UNO_PORT", "5005"))

# How long to wait for the sketch's ACK. On a direct LAN the round trip is well
# under a millisecond; 0.2 s is generous and still far below the 300 ms failsafe,
# so a blocking --once call cannot itself trip the watchdog it is testing.
ACK_TIMEOUT_S = float(os.environ.get("UNO_ACK_TIMEOUT_S", "0.2"))

# Keep this under the sketch's RX_BUFFER. The Uno has 2 KB of SRAM total, so the
# receive buffer over there is 96 bytes and not negotiable.
MAX_PAYLOAD = 80


class UnoLink:
    """UDP command link to the Arduino. One socket, reused for every packet."""

    def __init__(self, host=UNO_HOST, port=UNO_PORT, timeout=ACK_TIMEOUT_S):
        self.host = host
        self.port = int(port)
        self.timeout = timeout

        self.seq = 0
        self.sent = 0
        self.acked = 0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to an ephemeral port explicitly: the sketch replies to the source
        # address of the packet it received, so we must be listening on the same
        # socket we sent from. connect() would also work but would make every
        # stray ICMP port-unreachable raise on a later send.
        self._sock.settimeout(self.timeout)

    @property
    def target(self):
        return f"{self.host}:{self.port}"

    @property
    def loss_pct(self):
        if not self.sent:
            return 0.0
        return 100.0 * (self.sent - self.acked) / self.sent

    # -- sending ---------------------------------------------------------------

    def send(self, payload="", wait_ack=True):
        """Send one command. Returns round-trip ms, or None if no ACK came back.

        None means "no answer this time", not "link is down" — a single dropped
        datagram is normal and the caller should keep sending. Judge the link on
        loss_pct over many packets, the way link.py debounces its status chip.
        """
        if len(payload) > MAX_PAYLOAD:
            raise ValueError(
                f"payload is {len(payload)} chars, the sketch truncates above "
                f"{MAX_PAYLOAD} — split it or raise RX_BUFFER in the sketch too"
            )

        self.seq = (self.seq + 1) % 65536  # wraps like the sketch's uint16_t
        wire = f"CMD {self.seq} {payload}\n".encode("ascii", errors="replace")

        started = time.monotonic()
        self._sock.sendto(wire, (self.host, self.port))
        self.sent += 1

        if not wait_ack:
            return None

        # Drain until we see the ACK for *this* seq. A late ACK from an earlier
        # packet is still sitting in the buffer after any loss, and accepting it
        # here would report a round trip that never happened.
        deadline = started + self.timeout
        while time.monotonic() < deadline:
            self._sock.settimeout(max(0.0, deadline - time.monotonic()))
            try:
                data, _ = self._sock.recvfrom(64)
            except (socket.timeout, OSError):
                return None
            parts = data.decode("ascii", errors="replace").strip().split()
            if len(parts) >= 2 and parts[0] == "ACK" and parts[1] == str(self.seq):
                self.acked += 1
                return (time.monotonic() - started) * 1000.0
        return None

    def drain_acks(self):
        """Count ACKs already waiting, without blocking. Returns how many.

        The 50 Hz drive loop in uno_motors.py cannot afford send()'s blocking
        ACK wait — 0.2 s per packet would cap it at 5 Hz and trip the sketch's
        own 300 ms failsafe. It sends with wait_ack=False and calls this
        instead, judging the link on loss_pct over many packets rather than
        per packet, exactly as the docstring on send() describes.

        Sequence numbers are deliberately not matched here: at 50 Hz an ACK
        routinely arrives after the next command has gone out, so demanding
        the current seq would discard almost every valid reply.
        """
        seen = 0
        try:
            self._sock.settimeout(0)          # non-blocking for the drain
            while True:
                try:
                    data, _ = self._sock.recvfrom(64)
                except (BlockingIOError, socket.timeout, OSError):
                    break
                parts = data.decode("ascii", errors="replace").strip().split()
                if len(parts) >= 2 and parts[0] == "ACK":
                    self.acked += 1
                    seen += 1
        finally:
            self._sock.settimeout(self.timeout)
        return seen

    def close(self):
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--host", default=UNO_HOST, help=f"default {UNO_HOST}")
    ap.add_argument("--port", type=int, default=UNO_PORT, help=f"default {UNO_PORT}")
    ap.add_argument("--hz", type=float, default=10.0, help="send rate, default 10")
    ap.add_argument("--once", metavar="TEXT", help="send one packet and exit")
    ap.add_argument(
        "--no-ack", action="store_true", help="fire and forget, do not wait"
    )
    args = ap.parse_args()

    with UnoLink(args.host, args.port) as link:
        if args.once is not None:
            rtt = link.send(args.once, wait_ack=not args.no_ack)
            if args.no_ack:
                print(f"sent seq={link.seq} to {link.target} (no ACK requested)")
                return 0
            if rtt is None:
                print(f"NO ACK from {link.target}", file=sys.stderr)
                return 1
            print(f"ACK seq={link.seq} from {link.target} in {rtt:.2f} ms")
            return 0

        period = 1.0 / args.hz if args.hz > 0 else 0.0
        print(f"sending to {link.target} at {args.hz:g} Hz — Ctrl-C to stop")
        next_report = time.monotonic() + 1.0
        try:
            while True:
                cycle = time.monotonic()
                rtt = link.send(f"N={link.seq}", wait_ack=not args.no_ack)

                # One status line per second, not per packet: at 50 Hz a line per
                # packet scrolls too fast to read and the print itself starts to
                # cost more than the send.
                if cycle >= next_report:
                    next_report = cycle + 1.0
                    shown = f"{rtt:.2f} ms" if rtt is not None else "no ack"
                    print(
                        f"  seq={link.seq:<6} {shown:<10} "
                        f"sent={link.sent} acked={link.acked} "
                        f"loss={link.loss_pct:.1f}%"
                    )

                # Sleep the remainder of the period, not the whole period, so the
                # ACK wait above does not silently halve the send rate.
                slack = period - (time.monotonic() - cycle)
                if slack > 0:
                    time.sleep(slack)
        except KeyboardInterrupt:
            print()

        print(f"sent={link.sent} acked={link.acked} loss={link.loss_pct:.1f}%")
        return 0 if link.acked else 1


if __name__ == "__main__":
    sys.exit(main())
