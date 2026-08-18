#!/usr/bin/env python3
"""
USB auto-backup: plug a stick in, get /recordings moved onto it, pull it out.

Runs as its own root daemon (usb-backup.service), NOT inside the viewer,
because mounting block devices needs root and the viewer must never have it.
The two meet in exactly two places: the viewer writes sessions into
config.RECORD_DIR (/recordings), and this daemon publishes what it is doing
to STATUS_PATH (/run/usb_backup_status.json, tmpfs) - which the viewer's
strip renders as "USB TRANSFERRING nn%" so the operator knows not to pull
the stick mid-copy.

The cycle, per insertion:

    detect  - lsblk poll every POLL_S; a partition whose disk arrived over
              USB transport counts, the SD card and loop devices never do
    mount   - at /media/usb_backup-<dev>, unless an automounter got there first
    copy    - /recordings -> <stick>/recordings, same folder structure and
              file names, incremental (same-path-same-size files skipped),
              4MB chunks with the byte count published as it goes
    clear   - THE PI'S COPY IS THEN DELETED (operator spec 2026-08-18: the
              stick is the recording's destination, the Pi is only a buffer).
              Guarded three ways, per file: it is only removed if the stick's
              copy exists AND matches the source's size right now, AND the
              source has not been written for ACTIVE_GRACE_S - so a session
              still being recorded, or one sitting in the post-stop confirm
              window, survives untouched. No rmtree anywhere: files by name,
              then bare rmdir on whatever emptied. Errors during the copy
              skip the clear entirely.
    finish  - BACKUP_INFO.txt appended on the stick, sync, unmount

The unmount at the end is the point: when the strip says done, the stick is
ALREADY safe to pull. A handled device is remembered until its node
disappears, so leaving the stick in does not loop; replug re-triggers.

Everything lands in ~arnobot/usb_backup.log (capped like motor_cam.log).
Exercise without hardware:  python3 usb_backup.py --once  (one scan, verbose).
Set USB_BACKUP_DELETE=0 in the unit to keep the Pi's copies instead.
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
STATUS_PATH = os.environ.get("USB_STATUS_PATH", "/run/usb_backup_status.json")
POLL_S = float(os.environ.get("USB_BACKUP_POLL_S", "2.0"))
LOG_CAP_BYTES = 5_000_000
CHUNK = 4 * 1024 * 1024

# Delete the Pi's copy after a verified transfer. On by default per the spec;
# the off switch exists for bring-up, when losing a test recording to a flaky
# stick would cost a rig visit.
DELETE_AFTER = os.environ.get("USB_BACKUP_DELETE", "1") == "1"

# A source file written to within this window is never deleted: it is either
# an active recording or a stop so recent the operator may still be deciding
# whether to keep it. It gets cleared on the NEXT insertion instead.
ACTIVE_GRACE_S = float(os.environ.get("USB_BACKUP_GRACE_S", "30"))

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


_status = {"state": "idle"}


def publish(**fields):
    """Replace the published status. The viewer re-reads this once a second.

    Written atomically (tmp + rename) so the viewer can never parse half a
    file, and stamped `updated` so a dead daemon's last words age out instead
    of pinning "transferring" on the strip forever - the viewer discards
    anything older than 10s, which is also why refresh() re-stamps every poll.
    """
    global _status
    if fields:
        _status = dict(fields)
    _status["updated"] = time.time()
    try:
        tmp = STATUS_PATH + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(_status, fh)
        os.replace(tmp, STATUS_PATH)
    except OSError:
        pass


def refresh():
    """Re-stamp the current status without changing it."""
    publish(**{k: v for k, v in _status.items() if k != "updated"})


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


def plan_copy(src, dst):
    """Walk src once: (all_files, to_copy, bytes_to_copy).

    all_files is [(source, target)] for EVERY file - the clear pass needs the
    complete list. to_copy is the subset whose target is missing or a
    different size; its byte total is what the progress percentage is out of,
    so the strip counts down real work, not files that were already there.
    """
    all_files, to_copy, bytes_to_copy = [], [], 0
    for root, _dirs, files in os.walk(src):
        rel = os.path.relpath(root, src)
        target_root = dst if rel == "." else os.path.join(dst, rel)
        for name in sorted(files):
            s = os.path.join(root, name)
            t = os.path.join(target_root, name)
            all_files.append((s, t))
            try:
                size = os.path.getsize(s)
                if os.path.exists(t) and os.path.getsize(t) == size:
                    continue
                to_copy.append((s, t, size))
                bytes_to_copy += size
            except OSError:
                pass
    return all_files, to_copy, bytes_to_copy


def copy_file(src, dst, on_progress):
    """One file in CHUNK pieces, reporting bytes as they land."""
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with open(src, "rb") as fs, open(dst, "wb") as fd:
        while True:
            chunk = fs.read(CHUNK)
            if not chunk:
                break
            fd.write(chunk)
            on_progress(len(chunk))
    try:
        shutil.copystat(src, dst)
    except OSError:
        pass                # FAT has no unix modes; timestamps are best-effort


def clear_source(all_files):
    """Delete Pi-side files whose stick copy is verified. (deleted, kept).

    Verification is against the source's size NOW, not at plan time: a file
    that grew during the copy (an active recording) fails the compare and is
    kept. Only ever removes paths inside RECORD_DIR, one os.remove per known
    file, then bare rmdir on emptied directories - the same no-rmtree rule as
    recorder.py, for the same reason: this runs unattended, as root.
    """
    real_root = os.path.realpath(RECORD_DIR)
    deleted = kept = 0
    for s, t in all_files:
        try:
            if not os.path.realpath(s).startswith(real_root + os.sep):
                kept += 1
                continue
            if not os.path.exists(t) or not os.path.exists(s):
                kept += os.path.exists(s)
                continue
            if os.path.getsize(t) != os.path.getsize(s):
                kept += 1
                continue
            if time.time() - os.path.getmtime(s) < ACTIVE_GRACE_S:
                kept += 1
                continue
            os.remove(s)
            deleted += 1
        except OSError as exc:
            log(f"  clear failed: {s}: {exc}")
            kept += 1
    # Prune session directories that just emptied. rmdir refuses on anything
    # non-empty, so a directory holding a kept file survives on its own.
    for root, _dirs, _files in os.walk(RECORD_DIR, topdown=False):
        if os.path.realpath(root) == real_root:
            continue
        try:
            os.rmdir(root)
        except OSError:
            pass
    return deleted, kept


def backup_to(dev, fstype, premounted):
    """Mount (if needed), mirror /recordings, clear, sync, unmount."""
    if fstype not in SUPPORTED_FS:
        log(f"{dev}: filesystem '{fstype}' not supported - ignored")
        publish(state="error", device=dev,
                detail=f"unsupported filesystem {fstype}")
        return

    mounted_here = False
    if premounted:
        mnt = premounted
        log(f"{dev}: already mounted at {mnt} (automounter) - using it")
    else:
        mnt = os.path.join(MOUNT_BASE, "usb_backup-" + os.path.basename(dev))
        os.makedirs(mnt, exist_ok=True)
        # uid/gid on the FAT-family mounts so the files are readable as
        # arnobot while mounted; the native-Linux filesystems keep their own
        # ownership and refuse those options.
        opts = ["-o", "uid=1000,gid=1000"] if fstype in ("vfat", "exfat") else []
        res = subprocess.run(["mount", *opts, dev, mnt],
                             capture_output=True, text=True, timeout=30)
        if res.returncode != 0:
            log(f"{dev}: mount failed: {res.stderr.strip()}")
            publish(state="error", device=dev, detail="mount failed")
            return
        mounted_here = True
        log(f"{dev}: mounted at {mnt} ({fstype})")

    try:
        if not os.path.isdir(RECORD_DIR):
            log(f"{RECORD_DIR} does not exist - nothing to back up")
            publish(state="done", device=dev, copied=0, skipped=0, deleted=0)
            return
        dst = os.path.join(mnt, os.path.basename(RECORD_DIR.rstrip("/")))
        all_files, to_copy, bytes_total = plan_copy(RECORD_DIR, dst)
        log(f"{dev}: {len(to_copy)} file(s) to copy "
            f"({bytes_total / 1e6:.0f} MB), "
            f"{len(all_files) - len(to_copy)} already on the stick")

        started = time.monotonic()
        done_bytes = 0
        copied = errors = 0
        last_pub = 0.0
        for i, (s, t, _size) in enumerate(to_copy, start=1):
            name = os.path.basename(s)

            def on_progress(n, name=name, i=i):
                nonlocal done_bytes, last_pub
                done_bytes += n
                # Once a second is plenty for a strip redrawn from a file the
                # viewer itself polls at 1Hz; every chunk would be pure churn.
                if time.monotonic() - last_pub >= 1.0:
                    last_pub = time.monotonic()
                    publish(state="copying", device=dev, file=name,
                            file_i=i, files_total=len(to_copy),
                            bytes_done=done_bytes, bytes_total=bytes_total)

            publish(state="copying", device=dev, file=name,
                    file_i=i, files_total=len(to_copy),
                    bytes_done=done_bytes, bytes_total=bytes_total)
            try:
                copy_file(s, t, on_progress)
                copied += 1
            except OSError as exc:
                log(f"  copy failed: {s}: {exc}")
                errors += 1
        took = time.monotonic() - started

        deleted = kept = 0
        if errors:
            # Do not clear a Pi whose backup is not known-complete.
            log(f"{dev}: {errors} copy error(s) - keeping Pi copies")
        elif DELETE_AFTER:
            deleted, kept = clear_source(all_files)
            if kept:
                log(f"{dev}: kept {kept} file(s) on Pi "
                    f"(active/recent or unverified)")

        # The receipt on the stick itself, so whoever plugs it into a laptop
        # can see when and from where the contents came without this log.
        try:
            with open(os.path.join(mnt, "BACKUP_INFO.txt"), "a",
                      encoding="utf-8") as fh:
                fh.write(
                    f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} "
                    f"ground station backup: {copied} copied, "
                    f"{len(all_files) - len(to_copy)} already present, "
                    f"{errors} error(s), {deleted} cleared off the Pi, "
                    f"{took:.0f}s\n")
        except OSError:
            pass

        os.sync()
        skipped = len(all_files) - len(to_copy)
        if errors:
            publish(state="error", device=dev,
                    detail=f"{errors} file(s) failed - Pi copies kept")
        else:
            publish(state="done", device=dev, copied=copied, skipped=skipped,
                    deleted=deleted, finished=time.time())
        log(f"{dev}: backup done - {copied} copied, {skipped} already there, "
            f"{errors} errors, {deleted} cleared off the Pi, {took:.0f}s")
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
    log(f"usb_backup up - watching for USB drives, mirroring {RECORD_DIR}"
        f"{' (delete-after-transfer ON)' if DELETE_AFTER else ''}")
    publish(state="idle")
    handled = set()
    while True:
        parts = usb_partitions()
        present = {dev for dev, _fs, _mnt in parts}
        # Forget devices that were unplugged, so re-plugging re-triggers -
        # and drop the done/error banner once its stick is gone.
        if handled - present and not present:
            publish(state="idle")
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
                publish(state="error", device=dev, detail=str(exc)[:80])
        if once:
            log(f"--once: {len(parts)} USB filesystem(s) seen")
            return
        refresh()
        time.sleep(POLL_S)


if __name__ == "__main__":
    main()
