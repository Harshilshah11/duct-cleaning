#!/usr/bin/env python3
"""Put both IP cameras into the state the ground station wants, and prove it.

Idempotent: it reads before it writes, skips anything already correct, and
reports what it changed. Safe to re-run after a camera reboots and forgets.

WHY THIS EXISTS RATHER THAN A NOTE SAYING "do it in the browser". The cameras'
web UI is a Vue app talking WebSocket, and an earlier attempt stopped there
concluding there was no scriptable path. There is one underneath: a plain HTTP
JSON API, `goform/frm*`, with HTTP digest auth. The endpoints and the enum
meanings were read out of the camera's own app.js:

    SwitchToH264(){ this.SetVideoEncode(0, ...) }             VideoEncType 0 = H.264
    {method:"POST", data:{Type:.., Dev:1, Ch:.., Data:..}}    Type 0 reads, 1 writes

NOTHING HERE IS A GUESSED ENUM. VideoBitrate 15 = 512Kbps came from the camera's
own frmVideoCompressAbility table, and Resolution 3 was read off CAM 1 while it
was already serving what was wanted. A working device is a better specification
than an inferred lookup.

READ-MODIFY-WRITE THROUGHOUT. Every POST carries the endpoint's CURRENT Data with
only the named keys changed. These cameras accept a partial Data and backfill the
rest from defaults, silently resetting settings nobody touched.

THE ENCODER CHANGE REBOOTS THE CAMERA, and does not survive that first reboot -
observed 2026-08-29: the write returned Result 0, the readback confirmed it, and
it came back as H.265 anyway. Run this, wait for the camera to answer again, run
it a second time. The second run is a no-op if the first one stuck.
"""
import json
import subprocess
import sys

USER, PASS = "admin", "123456"
CAMS = [("192.168.1.103", "CAM 1"), ("192.168.1.102", "CAM 2")]

# --- what the ground station needs -------------------------------------------
# H.264 because gst_pipeline() hardwires H.265 to the SOFTWARE decoder
# avdec_h265 (v4l2slh265dec exists on the Pi but is not wired up). Measured
# 2026-08-29: CAM 2 on H.265 2560x1440 managed 13 fps against a 20 fps source,
# while CAM 1 on H.264 720p managed 26 fps on the same machine.
ENCODER = {
    "VideoEncType": 0,       # 0 = H.264
    "Resolution": 3,         # 1280x720 as served by these cameras
    "VideoH264Profile": 1,
    "VideoBitrate": 15,      # 15 = 512Kbps, from frmVideoCompressAbility
}

# The caption and the timestamp burnt into the picture. Two separate flags.
OSD_OFF = {"IsShowChanName": 0, "IsShowOSD": 0}


def call(ip, endpoint, typ, data):
    body = json.dumps({"Type": typ, "Dev": 1, "Ch": 1, "Data": data})
    out = subprocess.run(
        ["curl", "-s", "-m", "12", "--digest", "-u", "%s:%s" % (USER, PASS),
         "-H", "Content-Type: application/json", "--data", body,
         "http://%s/goform/%s" % (ip, endpoint)],
        capture_output=True, text=True).stdout
    try:
        return json.loads(out)
    except Exception:
        return {"Result": "unparsed", "raw": out[:160]}


def read(ip, endpoint):
    r = call(ip, endpoint, 0, {})
    return r.get("Data") if r.get("Result") == 0 else None


def apply(ip, endpoint, wanted, container=None):
    """Read, compare, write only if something differs. Returns a status string."""
    data = read(ip, endpoint)
    if data is None:
        return "unreadable"
    target = data[container] if container else data
    if all(target.get(k) == v for k, v in wanted.items()):
        return "already correct"
    before = {k: target.get(k) for k in wanted}
    target.update(wanted)
    r = call(ip, endpoint, 1, data)
    return "%s -> %s  Result=%s" % (before, wanted, r.get("Result"))


def main():
    print("person detection + snapshot capture OFF")
    for ip, name in CAMS:
        # SnapEnable is the capture riding on the detection; leaving it on keeps
        # the camera doing the work you asked it to stop.
        print("  %-6s %-15s %s" % (name, ip,
              apply(ip, "frmVideoPersonPara", {"Enable": 0, "SnapEnable": 0})))

    print("encoder: H.264 1280x720 @ 512Kbps")
    for ip, name in CAMS:
        print("  %-6s %-15s %s" % (name, ip, apply(ip, "frmVideoIPCSetPara", ENCODER)))

    print("burnt-in text overlay OFF")
    for ip, name in CAMS:
        print("  %-6s %-15s %s" % (name, ip,
              apply(ip, "frmSingleLineOSD", OSD_OFF, container="OSD")))
        print("  %-6s %-15s multi-line %s" % (name, ip,
              apply(ip, "frmMultiLineOSD", {"IsShowMultiOSD": 0}, container="MultiOSD")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
