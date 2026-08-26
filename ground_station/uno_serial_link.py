#!/usr/bin/env python3
"""
Serial twin of uno_link.UnoLink, for the USB-tethered Uno.

Exposes EXACTLY the surface uno_motors.MotorLink uses - send(), drain_acks(),
sent, acked, loss_pct, target, close() - so swapping the two is a one-line
change in MotorLink.run() and nothing else in the drive path knows the wire
changed. The command grammar is identical on both transports ("CMD <seq> ...\\n"
up, "ACK <seq>\\n" back), which is what makes the swap honest rather than a
rewrite; see the header of uno_serial.py.

WHY THIS EXISTS. The Ethernet link on this rig flapped 17 times in 9 minutes on
2026-08-25 and the shield stopped answering entirely, while USB enumerated
clean. uno_serial.py already had a MotorLink of its own, but it is WHEELS ONLY -
no actuator, no brush, no light, and no set_source() - so pointing main.py at it
would have silently dropped three subsystems. Keeping uno_motors.MotorLink and
replacing only its transport keeps the rod, the brush, the light, the slew
limiter and the ACK-window health test exactly as they were.

THE FRAMING DIFFERENCE THAT MATTERS. UDP delivered whole datagrams with their
own boundaries. A serial stream has none, so ACKs arrive as a byte stream that
can split mid-line between reads. drain_acks() therefore keeps a partial-line
buffer; dropping it would lose roughly one ACK in every few hundred and read as
a lossy link that is in fact perfect.
"""

from __future__ import annotations

import os
import time

from uno_serial import BAUD, OPEN_SETTLE_S, find_port

# Same default as uno_link.ACK_TIMEOUT_S. Only send(wait_ack=True) uses it - the
# 50 Hz drive loop sends with wait_ack=False and judges health over a window.
ACK_TIMEOUT_S = float(os.environ.get("UNO_ACK_TIMEOUT_S", "0.2"))

# The Uno reboots when the port opens (DTR), and anything written while the
# bootloader is running is dropped. Same settle the uno_serial.py MotorLink uses.
MAX_PAYLOAD = int(os.environ.get("UNO_MAX_PAYLOAD", "80"))


class SerialLink:
    """USB command link to the Arduino. One port, reused for every command."""

    def __init__(self, port=None, baud=BAUD, timeout=ACK_TIMEOUT_S):
        import serial                                  # pyserial

        self.port = port or find_port()
        if self.port is None:
            raise RuntimeError("no serial port found - is the Uno plugged in?")
        self.baud = int(baud)
        self.timeout = timeout

        self.seq = 0
        self.sent = 0
        self.acked = 0

        self._ser = serial.Serial(self.port, self.baud, timeout=0,
                                  write_timeout=0.2)
        # The board is in its bootloader right now. Writing before this settles
        # puts the first commands on the floor and, worse, makes the link look
        # dead exactly when the operator is watching it start.
        time.sleep(OPEN_SETTLE_S)
        self._ser.reset_input_buffer()
        # Partial ACK line carried between drains - see the header note.
        self._buf = ""

    @property
    def target(self):
        return self.port

    @property
    def loss_pct(self):
        if not self.sent:
            return 0.0
        return 100.0 * (self.sent - self.acked) / self.sent

    # -- sending ---------------------------------------------------------------

    def send(self, payload="", wait_ack=True):
        """Send one command. Returns round-trip ms, or None if no ACK came back.

        None means "no answer this time", not "the link is down" - identical
        semantics to UnoLink.send(), because MotorLink judges health on
        loss_pct over many commands rather than on any single one.
        """
        if len(payload) > MAX_PAYLOAD:
            raise ValueError(
                f"payload is {len(payload)} chars, the sketch's RX_BUFFER "
                f"truncates above {MAX_PAYLOAD} - split it or raise both")

        self.seq = (self.seq + 1) % 65536      # wraps like the sketch's uint16_t
        wire = f"CMD {self.seq} {payload}\n".encode("ascii", errors="replace")

        started = time.monotonic()
        self._ser.write(wire)
        self.sent += 1

        if not wait_ack:
            return None

        deadline = started + self.timeout
        while time.monotonic() < deadline:
            for tok in self._read_acks():
                if tok == str(self.seq):
                    self.acked += 1
                    return (time.monotonic() - started) * 1000.0
            time.sleep(0.002)
        return None

    def drain_acks(self):
        """Count ACKs already waiting, without blocking. Returns how many.

        Sequence numbers are deliberately NOT matched, exactly as in
        UnoLink.drain_acks(): at 50 Hz an ACK routinely lands after the next
        command has gone out, so demanding the current seq would throw away
        almost every valid reply and report a dead link.
        """
        seen = 0
        for _ in self._read_acks():
            self.acked += 1
            seen += 1
        return seen

    def _read_acks(self):
        """Yield the seq token of every complete ACK line now readable.

        Non-ACK lines are skipped rather than being an error: the sketch prints
        its banner and its L=/R=/ACT= telemetry down this same port, and that is
        allowed precisely because this filter ignores it.
        """
        try:
            waiting = self._ser.in_waiting
        except Exception:
            return
        if waiting:
            try:
                self._buf += self._ser.read(waiting).decode("ascii", "replace")
            except Exception:
                return
        # Keep the tail after the last newline: it is a line still arriving.
        if "\n" not in self._buf:
            return
        chunk, self._buf = self._buf.rsplit("\n", 1)
        for line in chunk.splitlines():
            line = line.strip()
            if line.startswith("ACK "):
                yield line[4:].strip()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
