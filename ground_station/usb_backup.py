#!/usr/bin/env python3
"""
USB auto-backup: plug a stick in, get a copy of /recordings, pull it out.

Runs as its own root daemon (usb-backup.service), NOT inside the viewer,
because mounting block devices needs root and the viewer must never have it.
The two only meet at the filesystem: the viewer writes sessions into
config.RECORD_DIR (/recordings) and this mirrors that tree verbatim -
same folder structure, same file names - onto any USB drive that appears.

The cycle, per insertion:

    detect  - lsblk poll every POLL_S; a partition whose disk arrived over
              USB transport counts, the SD card and loop devices never do
    mount   - at /media/usb_backup-<dev>, unless an automounter got there first
    copy    - /recordings -> <stick>/recordings, incremental: a file that
              already exists on the stick with the same size is skipped, so
              re-plugging a 30GB stick after one new session copies one session
    finish  - BACKUP_INFO.txt written to the stick, sync, unmount

The unmount at the end is the point of the design: when the log says done the
stick is ALREADY safe to pull, because an operator at a duct will not be asking
politely with `umount` first. A handled device is remembered until its node
disappears, so leaving the stick in does not re-trigger the cycle; unplugging
and re-plugging does, and costs only the incremental copy.

Everything lands in ~arnobot/usb_backup.log (capped like motor_cam.log).
Exercise without hardware:  python3 usb_backup.py --once  (one scan, verbose).
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

RECORD_DIR = os.environ.get("RECORD_DIR", "/recordings")
MOUNT_BASE = "/media"
LOG_PATH = os.environ.get("USB_BACKUP_LOG", "/home/arnobot/usb_backup.log")
POLL_S = float(os.environ.get("USB_BACKUP_POLL_S", "2.0"))
LOG_CAP_BYTES = 5_000_000

# Filesystems worth trying to mount. A stick with none of these (or no
# filesystem at all) is logged once and left alone.
SUPPORTED_FS = {"vfat", "exfat", "ntfs", "ext4", "ext3", "ext2"}

# Device paths that can never be the backup target, whatever lsblk says about
# them: the SD card the Pi runs from, and every flavour of virtual device.
NEVER = ("/dev/mmcblk", "/dev/loop", "/dev/ram", "/dev/zram", "/dev/dm-")


def log(msg):
    line = f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} {msg}"
    print(line, flush=True)
    try:
        # Same cap-and-rotate as motor_cam.log - a log that fills the SD card
        # takes the viewer down with it.
        try:
            if os.path.getsize(LOG_PATH) > LOG_CAP_BYTES:
                os.replace(LOG_PATH, LOG_PATH + ".1")
        except OSError:
            pass
        with open(LOG_PATH, "a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except OSError:
        pass


def _truthy(value):
    # lsblk's JSON booleans changed across util-linux versions: new emits
    # true/false, old emits "1"/"0". Accept both.
    return value in (True, 1, "1", "true", "yes")


def usb_partitions():
    """[(dev_path, fstype, mountpoint)] for every USB-attached filesystem."""
    try:
        out = subprocess.run(
            ["lsblk", "-J", "-o", "NAME,PATH,TYPE,TRAN,RM,FSTYPE,MOUNTPOINT"],
            capture_output=True, text=True, timeout=10)
        data = json.loads(out.stdout)
    except Exception as exc:
        log(f"lsblk failed: {exc}")
        return []

    found = []
    for disk in data.get("blockdevices") or []:
        path = disk.get("path") or f"/dev/{disk.get('name')}"
        if any(path.startswith(p) for p in NEVER):
            continue
        # Transport is the reliable signal; the removable flag backs it up
        # because some SD-to-USB bridges report tran null but rm true.
        if disk.get("tran") != "usb" and not _truthy(disk.get("rm")):
            continue
        children = disk.get("children") or []
        for part in children:
            if part.get("type") == "part" and part.get("fstype"):
                found.append((part.get("path") or f"/dev/{part.get('name')}",
                              part["fstype"], part.get("mountpoint")))
        # A stick formatted with no partition table has its filesystem on the
        # whole device.
        if not children and disk.get("fstype"):
            found.append((path, disk["fstype"], disk.get("mountpoint")))
    return found


def copy_tree(src, dst):
    """Mirror src into dst. Returns (copied, skipped, errors).

    Size-compare, not hash: the recordings are append-once video files, so
    same-path-same-size means already backed up, and hashing gigabytes over
    USB2 would turn a 10s top-up into minutes. A session actively being
    written when the stick goes in copies as a snapshot and self-repairs on
    the next insertion, when its size no longer matches.
    """
    copied = skipped = errors = 0
    for root, _dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        target_root = dst if rel == "." else os.path.join(dst, rel)
        try:
            os.makedirs(target_root, exist_ok=True)
        except OSError as exc:
            log(f"  mkdir failed: {target_root}: {exc}")
            errors += 1
            continue
        for name in files:
            s = os.path.join(root, name)
            t = os.path.join(target_root, name)
            try:
                if os.path.exists(t) and os.path.getsize(t) == os.path.getsize(s):
                    skipped += 1
                    continue
                shutil.copy2(s, t)
                copied += 1
            except OSError as exc:
                log(f"  copy failed: {s}: {exc}")
                errors += 1
    return copied, skipped, errors


def backup_to(dev, fstype, premounted):
    """Mount (if needed), mirror /recordings, sync, unmount. One insertion."""
    if fstype not in SUPPORTED_FS:
        log(f"{dev}: filesystem '{fstype}' not supported - ignored")
        return

    mounted_here = False
    if premounted:
        mnt = premounted
        log(f"{dev}: already mounted at {mnt} (automounter) - using it")
    else:
        mnt = os.path.join(MOUNT_BASE,
                           "usb_backup-" + os.path.basename(dev))
        os.makedirs(mnt, exist_ok=True)
        # uid/gid on the FAT-family mounts so the files are readable as
        # arnobot while mounted; the native-Linux filesystems keep their own
        # ownership and refuse those options.
        opts = ["-o", "uid=1000,gid=1000"] if fstype in ("vfat", "exfat") else []
        cmd = ["mount", *opts, dev, mnt]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if res.returncode != 0:
            log(f"{dev}: mount failed: {res.stderr.strip()}")
            return
        mounted_here = True
        log(f"{dev}: mounted at {mnt} ({fstype})")

    try:
        if not os.path.isdir(RECORD_DIR):
            log(f"{RECORD_DIR} does not exist - nothing to back up")
            return
        dst = os.path.join(mnt, os.path.basename(RECORD_DIR.rstrip("/")))
        started = time.monotonic()
        copied, skipped, errors = copy_tree(RECORD_DIR, dst)
        took = time.monotonic() - started

        # The receipt on the stick itself, so whoever plugs it into a laptop
        # can see when and from where the contents came without this log.
        try:
            with open(os.path.join(mnt, "BACKUP_INFO.txt"), "a",
                      encoding="utf-8") as fh:
                fh.write(
                    f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                    f"ground station backup: {copied} file(s) copied, "
                    f"{skipped} already present, {errors} error(s), "
                    f"{took:.0f}s\n")
        except OSError:
            pass

        os.sync()
        log(f"{dev}: backup done - {copied} copied, {skipped} already there, "
            f"{errors} errors, {took:.0f}s")
    finally:
        if mounted_here:
            res = subprocess.run(["umount", mnt], capture_output=True,
                                 text=True, timeout=60)
            if res.returncode == 0:
                log(f"{dev}: unmounted - SAFE TO REMOVE")
                try:
                    os.rmdir(mnt)
                except OSError:
                    pass
            else:
                # Better a mounted stick than a corrupted one: say so and
                # leave it. os.sync() above means the data itself is on flash.
                log(f"{dev}: umount failed ({res.stderr.strip()}) - "
                    f"data is synced but unmount by hand before pulling")


def main():
    once = "--once" in sys.argv
    log(f"usb_backup up - watching for USB drives, mirroring {RECORD_DIR}")
    handled = set()
    while True:
        parts = usb_partitions()
        present = {dev for dev, _fs, _mnt in parts}
        # Forget devices that were unplugged, so re-plugging re-triggers.
        handled &= present
        for dev, fstype, mountpoint in parts:
            if dev in handled:
                continue
            # Marked handled even when the attempt fails: retrying a broken
            # stick every POLL_S would fill the log and hammer the port. The
            # operator's retry gesture is unplug/replug, which clears this.
            handled.add(dev)
            log(f"{dev}: new USB drive detected")
            try:
                backup_to(dev, fstype, mountpoint)
            except Exception as exc:
                log(f"{dev}: backup crashed: {exc}")
        if once:
            log(f"--once: {len(parts)} USB filesystem(s) seen")
            return
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
