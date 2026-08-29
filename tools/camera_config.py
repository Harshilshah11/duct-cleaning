#!/usr/bin/env python3
"""Configure the two IP cameras over their private HTTP API.

WHY THIS EXISTS RATHER THAN A NOTE SAYING "do it in the browser". The cameras'
web UI is a Vue app that talks WebSocket, and an earlier attempt stopped there
concluding there was no scriptable path. There is: underneath the UI sits a
plain HTTP JSON API, `goform/frm*`, with HTTP digest auth. The endpoint names
and the enum meanings were read out of the camera's own app.js:

    SwitchToH264(){ this.SetVideoEncode(0, ...) }      ->  VideoEncType 0 = H.264
    {method:"POST", data:{Type:.., Dev:1, Ch:.., Data:..}}  ->  Type 0 reads, 1 writes

Resolution 3 = 1280x720 was NOT guessed from a table - it was read off CAM 1,
which was already serving exactly what was wanted. A working device is a better
specification than an inferred enum.

WHAT IT SETS
  * person/human detection OFF on both, and SnapEnable with it - that is the
    snapshot capture riding on the detection, and leaving it on keeps the camera
    doing the work you asked it to stop.
  * CAM 2 to H.264 1280x720. It had been reconfigured to H.265 at 2560x1440,
    which the ground station could not keep up with: measured 13 fps against a
    20 fps source, because gst_pipeline() hardwires H.265 to the SOFTWARE
    decoder avdec_h265 (v4l2slh265dec exists on the Pi but is not wired up).
    CAM 1 on H.264 720p managed 26 fps on the same machine.

READ-MODIFY-WRITE THROUGHOUT. Every POST carries the endpoint's CURRENT Data
with only the named keys changed. These cameras accept a partial Data and
backfill the rest from defaults, silently resetting settings nobody touched.

THE ENCODER CHANGE REBOOTS THE CAMERA, and the setting does not survive that
first reboot - observed 2026-08-29: the write returned Result 0, the readback
confirmed it, and it came back as H.265 anyway. Re-running this once the camera
is up again makes it stick. So: run it, wait for the camera to return, run it
again, and check.
"""
import json, subprocess, sys

USER, PASS = "admin", "123456"
CAMS = {"192.168.1.103": "CAM 1", "192.168.1.102": "CAM 2"}
ENCODER_TARGET = {"VideoEncType": 0, "Resolution": 3, "VideoH264Profile": 1}


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
        return {"Result": "unparsed", "raw": out[:200]}


def read(ip, endpoint):
    r = call(ip, endpoint, 0, {})
    return r.get("Data") if r.get("Result") == 0 else None


def main():
    for ip, name in CAMS.items():
        d = read(ip, "frmVideoPersonPara")
        if d is None:
            print("%s %s: unreachable" % (name, ip))
            continue
        was = d.get("Enable")
        d["Enable"] = 0
        d["SnapEnable"] = 0
        r = call(ip, "frmVideoPersonPara", 1, d)
        print("%s %s: person-detect %s -> 0   Result=%s" % (name, ip, was, r.get("Result")))

    for ip, name in CAMS.items():
        d = read(ip, "frmVideoIPCSetPara")
        if d is None:
            print("%s %s: encoder unreadable" % (name, ip))
            continue
        if all(d.get(k) == v for k, v in ENCODER_TARGET.items()):
            print("%s %s: already H.264 1280x720" % (name, ip))
            continue
        before = {k: d.get(k) for k in ENCODER_TARGET}
        d.update(ENCODER_TARGET)
        # The app uses 2 x framerate for the H.264 I-frame interval.
        if d.get("VideoFrameRate"):
            d["IFrameInterval"] = 2 * int(d["VideoFrameRate"])
        r = call(ip, "frmVideoIPCSetPara", 1, d)
        print("%s %s: encoder %s -> H.264 720p   Result=%s"
              % (name, ip, before, r.get("Result")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
