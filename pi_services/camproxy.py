#!/usr/bin/env python3
"""Plain TCP forwarder so the cameras can be opened from the LAN.

The cameras sit on the Pi's eth0 as /32 host routes, so nothing else on the
network can reach them. This listens on the Pi's wlan0 address and pipes bytes
straight through - no HTTP parsing, so the app's WebSocket upgrade survives it
unchanged, which a naive HTTP proxy would break.
"""
import socket, sys, threading

MAP = {8103: ("192.168.1.103", 80), 8102: ("192.168.1.102", 80)}


def pipe(a, b):
    try:
        while True:
            data = a.recv(65536)
            if not data:
                break
            b.sendall(data)
    except OSError:
        pass
    finally:
        for s in (a, b):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def serve(listen_port, dest):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", listen_port))
    srv.listen(16)
    while True:
        try:
            cli, _ = srv.accept()
            up = socket.create_connection(dest, timeout=10)
        except OSError:
            continue
        for a, b in ((cli, up), (up, cli)):
            threading.Thread(target=pipe, args=(a, b), daemon=True).start()


for port, dest in MAP.items():
    threading.Thread(target=serve, args=(port, dest), daemon=True).start()
    print("listening 0.0.0.0:%d -> %s:%d" % (port, dest[0], dest[1]), flush=True)

threading.Event().wait()
