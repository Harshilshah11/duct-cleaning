#!/usr/bin/env python3
"""Serial + ping logger for the Uno that must NOT reset it.

Opening /dev/ttyACM0 normally pulses DTR, which warm-resets the board - and a
warm reset is the very thing that masks the cold-boot fault. So the port is
opened with dtr forced low first, and hupcl cleared so a close cannot pulse it
either. When the board power-cycles the device node vanishes; we reopen the
same careful way. Ping state is logged on the same clock so the two can be
correlated even for boots the serial side missed.
"""
import os, subprocess, time

LOG = "/home/arnobot/diag/uno_watch.log"
os.makedirs("/home/arnobot/diag", exist_ok=True)

def log(line):
    with open(LOG, "a") as f:
        f.write("%s %s\n" % (time.strftime("%H:%M:%S"), line))

def open_port():
    """Open the port while asserting DTR for as short a time as possible.

    MEASURED 2026-08-27: every "SERIAL opened" line in the log was followed
    about two seconds later by a board RESET line. This watcher was resetting
    the board it exists to observe, and inflating the boot counter it exists to
    read - which matters, because that counter is the only thing that tells a
    brown-out apart from a true power loss.

    Setting pyserial's .dtr before .open() is not enough. The kernel raises DTR
    as part of the open() itself, and pyserial only applies the requested state
    afterwards in _reconfigure_port(). The Arduino's auto-reset is a 100 nF cap
    on that line, so even a brief assertion is a reset.

    So: clear HUPCL FIRST with the port closed (that stops the close from
    pulsing it), then open non-blocking, then drop DTR/RTS with TIOCMBIC as the
    very next syscall. The assertion window shrinks from milliseconds to
    microseconds, which the reset cap does not integrate. This cannot be made
    exactly zero from userspace - the first open after a re-enumeration may
    still nudge it - but it stops the watcher being a reset source of its own.
    """
    import serial, fcntl, termios, struct
    subprocess.run(["stty", "-F", "/dev/ttyACM0", "-hupcl"], capture_output=True)
    fd = os.open("/dev/ttyACM0", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        # TIOCMBIC = clear the named modem bits. Straight after open, before
        # anything has had time to charge the reset cap.
        bits = struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS)
        fcntl.ioctl(fd, termios.TIOCMBIC, bits)
    except Exception:
        pass
    s = serial.Serial()
    s.port = "/dev/ttyACM0"
    s.baudrate = 115200   # must equal SERIAL_BAUD in Config.h
    s.timeout = 1
    s.dtr = False
    s.rts = False
    s.open()
    os.close(fd)
    subprocess.run(["stty", "-F", "/dev/ttyACM0", "-hupcl"], capture_output=True)
    return s

log("=== watcher started ===")
ser = None
last_ping = None
last_ping_t = 0.0
while True:
    now = time.time()
    if now - last_ping_t >= 1.0:
        up = subprocess.run(["ping", "-c1", "-W1", "192.168.50.20"],
                            capture_output=True).returncode == 0
        if up != last_ping:
            log("PING %s" % ("UP" if up else "DOWN"))
            last_ping = up
        last_ping_t = now
    if ser is None:
        if os.path.exists("/dev/ttyACM0"):
            try:
                ser = open_port()
                log("SERIAL opened (dtr held low)")
            except Exception as e:
                log("SERIAL open failed: %s" % e)
                time.sleep(2)
        else:
            time.sleep(1)
        continue
    try:
        ln = ser.readline().decode(errors="replace").strip()
        if ln:
            log("UNO| " + ln)
    except Exception as e:
        log("SERIAL lost: %s" % e)
        try: ser.close()
        except Exception: pass
        ser = None
