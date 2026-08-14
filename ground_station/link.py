#!/usr/bin/env python3
"""
Is the robot actually there?

The camera panels answer "is video arriving", which is a different question —
and since the cameras are now IP cameras on their own addresses, video says
nothing whatsoever about the robot. This asks the robot itself: one TCP connect
per interval, in its own daemon thread.

Its own thread because a connect to an address with nothing at the other end
can hang for ~2 minutes on Linux (an unplugged cable produces no ARP reply at
all, so there is nothing to refuse the connection) — the same trap
stream.probe_reachable() exists to avoid. Every socket here gets an explicit
timeout, and the GUI never waits on any of it.

Today the only thing listening on the robot Pi is MediaMTX's RTSP port, so that
is the default target. When Step 5-9 adds the Arduino command link, point
ROBOT_LINK_HOST / ROBOT_LINK_PORT at whatever answers there instead.

Check it from a terminal, no GUI needed:

    python3 link.py                       # uses config
    python3 link.py 192.168.1.30 8554
"""

import socket
import threading
import time


class LinkMonitor:
    """Background reachability probe for one host:port. Read the attributes."""

    def __init__(self, host, port, interval=2.0, timeout=1.5, fail_limit=2):
        self.host = host
        self.port = int(port)
        self.interval = interval
        self.timeout = timeout
        self.fail_limit = fail_limit

        # None until the first probe finishes, so the UI can say "linking"
        # instead of calling the robot dead before we have even looked.
        self.connected = None
        self.rtt_ms = None
        self.last_ok = None
        self.probes = 0

        self._fails = 0
        self._stop = threading.Event()
        self._thread = None

    @property
    def target(self):
        return f"{self.host}:{self.port}"

    # -- lifecycle -------------------------------------------------------------

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"link-{self.host}", daemon=True
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=self.timeout + 1.0)

    # -- worker ----------------------------------------------------------------

    def _run(self):
        while not self._stop.is_set():
            self._probe_once()
            self._stop.wait(self.interval)

    def _probe_once(self):
        started = time.monotonic()
        try:
            with socket.create_connection((self.host, self.port), self.timeout):
                pass
        except OSError:
            self._fails += 1
            # One dropped probe is not an outage. Requiring fail_limit in a row
            # stops the chip flickering red on a healthy tether every time a
            # packet goes missing — at the cost of noticing a real drop one
            # interval later, which is the right trade for a status light.
            #
            # The debounce starts only once we have an answer to debounce: the
            # very first result is reported as it comes, so a robot that is not
            # there says DISCONNECTED straight away instead of leaving the bar
            # on LINKING for two intervals at every startup.
            if self.connected is None or self._fails >= self.fail_limit:
                self.connected = False
                self.rtt_ms = None
            self.probes += 1
            return

        self._fails = 0
        self.rtt_ms = (time.monotonic() - started) * 1000.0
        self.last_ok = time.monotonic()
        self.connected = True
        self.probes += 1


def main():
    """Watch the link from a terminal:  python3 link.py [host] [port]"""
    import sys

    import config

    host = sys.argv[1] if len(sys.argv) > 1 else config.ROBOT_LINK_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else config.ROBOT_LINK_PORT

    monitor = LinkMonitor(host, port, config.LINK_POLL_S, config.LINK_TIMEOUT_S)
    monitor.start()
    print(f"probing {monitor.target} every {monitor.interval}s — Ctrl-C to stop")
    try:
        while True:
            time.sleep(monitor.interval)
            state = {None: "LINKING", True: "CONNECTED", False: "DISCONNECTED"}[
                monitor.connected
            ]
            rtt = f"{monitor.rtt_ms:.1f} ms" if monitor.rtt_ms is not None else "—"
            print(f"  {state:<13} {rtt}")
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop()


if __name__ == "__main__":
    main()
