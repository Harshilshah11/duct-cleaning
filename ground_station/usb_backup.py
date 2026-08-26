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
              -> publishes "detected" the instant the node is seen
    mount   - at /media/usb_backup-<dev>, unless an automounter got there first
              -> publishes "mounting", then "scanning" while the copy is planned
    copy    - /recordings -> <stick>/recordings, same folder structure and
              file names, incremental (same-path-same-size files skipped),
              4MB chunks with the byte count published as it goes
    settle  - publishes "finishing". Waits for the recorder to stop producing:
              the per-camera re-encode and the MERGED full view both land after
              a save, and both used to land after this daemon had already said
              "done". Nothing counts as transferred until the tree holds no
              .norm/.part temporaries and has been untouched for
              SETTLE_QUIET_S. See SETTLE_QUIET_S for the measurement.
    clear   - publishes "clearing".
              THE PI'S COPY IS THEN DELETED (operator spec 2026-08-18: the
              stick is the recording's destination, the Pi is only a buffer).
              Guarded per file: it is only removed if the stick's copy exists
              AND matches the source's size right now. A transfer that did NOT
              settle adds a third guard - the source must not have been written
              for ACTIVE_GRACE_S - so a session still being recorded survives
              untouched. A settled transfer drops that guard, because settling
              already proved the same thing and more; keeping it meant the
              merged file was always too new to delete and /recordings never
              emptied. No rmtree anywhere: files by name, then bare rmdir on
              whatever emptied. Errors during the copy skip the clear entirely.
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
# OFF, operator 2026-08-26: "i not want to auto delete after save".
#
# Saving to a stick and clearing the Pi are now two separate decisions, made
# with two separate buttons in the chooser. Nothing this daemon does removes
# footage any more - only DELETE and DELETE ALL in the viewer do, and both ask
# first.
#
# Only reachable at all with USB_AUTO_COPY=1; the manual path never calls the
# code this guards. It is left wired so the two settings still describe the old
# behaviour honestly if anyone turns mirroring back on.
DELETE_AFTER = os.environ.get("USB_BACKUP_DELETE", "0") == "1"

# AUTOMATIC COPYING IS OFF, operator's instruction 2026-08-26: a stick now opens
# a chooser in the viewer and nothing moves until SAVE or DELETE is pressed.
#
# WHY THE DAEMON STAYS RUNNING ANYWAY. It is what finds the partition, mounts
# it, and publishes where it landed - the viewer has no business poking at
# block devices, and it would need root for the mount. So the daemon keeps
# doing the part that needs privilege and stops doing the part that is now the
# operator's decision. It publishes state="mounted" with the mount path and
# waits.
#
# THIS TURNS OFF AN AUTOMATIC BACKUP. With it on, plugging a stick in copied
# everything and - with USB_BACKUP_DELETE=1 - deleted it from the Pi. Footage
# now stays on the Pi until somebody chooses. That is what was asked for, and
# it means an operator who forgets to press SAVE keeps the footage rather than
# losing it, which is the safer half of the trade.
#
# USB_AUTO_COPY=1 restores the old behaviour exactly.
AUTO_COPY = os.environ.get("USB_AUTO_COPY", "0") == "1"

# How many times the transfer will re-COPY files that appeared while it was
# running before it gives up on reaching a clean sweep. See the settle loop in
# backup_to. Three is enough for the two writers that legitimately produce work
# after a scan (the full view build and the master re-encode) without letting an
# active recording hold the stick for ever.
SETTLE_PASSES = int(os.environ.get("USB_BACKUP_SETTLE_PASSES", "3"))

# Re-scanning is not enough on its own. Measured on 2026-08-20: a stick plugged
# in the moment SAVE was confirmed was declared "done, safe to remove" at
# 11:28:29.32 - and the recorder then rewrote cam1 at 11:28:29.83, cam2 at
# 11:28:36.39 and wrote the MERGED full_001.mp4 at 11:28:45.72. All three
# re-scans had run inside the same second, found nothing new, and called it
# settled: the stick went home with two stale per-camera files and no merged
# video at all, while the reset marker below claimed everything had transferred.
#
# So the settle loop WAITS, and it has two different clocks because there are
# two different things it can be waiting for:
#
#   a .norm/.part temporary exists  -> the recorder is provably mid re-encode or
#       mid merge. That work always finishes, and its output is footage the
#       operator asked for, so wait a long time (SETTLE_BUILD_MAX_S).
#   nothing explains the writes     -> could be an active recording, which never
#       converges. Wait only long enough to cover the gaps BETWEEN the recorder's
#       ffmpeg stages (SETTLE_QUIET_MAX_S), then give up on the reset and leave
#       the stragglers for the next insertion.
#
# Either way the directory has to go untouched for SETTLE_QUIET_S before the
# transfer counts as complete.
SETTLE_QUIET_S = float(os.environ.get("USB_BACKUP_QUIET_S", "10"))
SETTLE_BUILD_MAX_S = float(os.environ.get("USB_BACKUP_BUILD_MAX_S", "900"))
SETTLE_QUIET_MAX_S = float(os.environ.get("USB_BACKUP_QUIET_MAX_S", "25"))
SETTLE_TEMP_STALE_S = float(os.environ.get("USB_BACKUP_TEMP_STALE_S", "120"))
SETTLE_POLL_S = 1.0

# Dropped in RECORD_DIR at the end of a verified transfer, holding the unix time
# that transfer finished. recorder.py reads it and restarts its session numbering
# at 001 for anything recorded after that instant - see recorder._next_session_no.
# Left in place rather than consumed, so the reset survives a viewer restart and
# so sessions kept back from this backup can still be told apart from new ones.
SESSION_RESET_MARKER = ".session_reset"

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


def fat_time_offset():
    """Minutes east of UTC, for the vfat/exfat time_offset= mount option.

    FAT stores a wall-clock local time with no timezone recorded beside it, and
    Windows reads it straight back as local time. Linux has to convert, and by
    default it converts using the KERNEL's sys_tz - which is not the same thing
    as /etc/localtime. sys_tz is set by userspace at boot, normally off the back
    of the hardware clock, and this Pi has no RTC at all (timedatectl reports
    "RTC time: n/a"). When it never gets set the stamps go onto the stick as
    UTC, and every file then reads 5.5 hours early on the operator's laptop -
    the clocks on both machines being correct and in agreement the whole time.

    Passing the offset explicitly takes sys_tz out of the question. It is also
    safe if sys_tz was right all along: "subtract 330" is what the default was
    already doing, so this is identical then and corrective otherwise.

    Read from the Pi's own timezone at mount time rather than hard-coded, so a
    rig that travels keeps writing stamps its operator recognises.
    """
    if time.daylight and time.localtime().tm_isdst > 0:
        return -time.altzone // 60
    return -time.timezone // 60


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
            # Dotfiles and .part/.norm temporaries are scratch belonging to this
            # daemon and to the recorder - the session reset marker, a full view
            # still being written, a master midway through its re-encode - never
            # footage. Copying one puts a truncated file on the stick, and the
            # clear pass afterwards would delete state the recorder still needs.
            if name.startswith(".") or name.endswith((".part", ".norm")):
                continue
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


def pending_work(root):
    """(temporaries, seconds since the last write) for the whole tree.

    The two signals the settle loop runs on. A temporary is one of the recorder's
    two in-flight names - `.<master>.norm` while a per-camera file is being
    re-encoded, `full_nnn.mp4.part` while the merged view is being built - and
    its presence is proof that more footage is still coming. The reset marker is
    excluded from the mtime: this daemon writes it, so counting it would mean
    every transfer restarted its own quiet window.
    """
    temps, newest, now = 0, 0.0, time.time()
    for dirpath, _dirs, files in os.walk(root):
        for name in files:
            if name == SESSION_RESET_MARKER:
                continue
            try:
                mtime = os.path.getmtime(os.path.join(dirpath, name))
            except OSError:
                continue
            newest = max(newest, mtime)
            # A temporary only counts while it is still GROWING. ffmpeg touches
            # its output every few seconds; one that has not moved in two
            # minutes is debris from a viewer that was killed mid-build, and
            # treating that as live work would hold the stick for the full
            # SETTLE_BUILD_MAX_S on every insertion from now on.
            if (name.endswith((".part", ".norm"))
                    and now - mtime < SETTLE_TEMP_STALE_S):
                temps += 1
    # No files at all is as quiet as it gets - do not report it as "just written".
    return temps, (time.time() - newest) if newest else 1e9


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


def clear_source(all_files, grace=None):
    """Delete Pi-side files whose stick copy is verified. (deleted, kept).

    Verification is against the source's size NOW, not at plan time: a file
    that grew during the copy (an active recording) fails the compare and is
    kept. Only ever removes paths inside RECORD_DIR, one os.remove per known
    file, then bare rmdir on emptied directories - the same no-rmtree rule as
    recorder.py, for the same reason: this runs unattended, as root.

    `grace` is how recently a file may have been written and still be deleted.
    The caller passes 0 on a SETTLED transfer, and that is the whole point of
    settling: the loop in backup_to has already proved there are no temporaries
    and that nothing in the tree has been touched for SETTLE_QUIET_S, so the
    "it might still be being written" worry the default grace exists for cannot
    apply. Without this the merged view - written seconds before the transfer
    finishes, by construction - failed the age test every single time and was
    left on the card, so /recordings never actually emptied (operator, 2026-08-20).
    Unsettled transfers still get the full ACTIVE_GRACE_S.
    """
    if grace is None:
        grace = ACTIVE_GRACE_S
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
            if grace and time.time() - os.path.getmtime(s) < grace:
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


def mount_stick(dev, fstype, premounted):
    """Mount the stick if it is not already. Returns (mountpoint, mounted_here).

    Lifted out of backup_to() 2026-08-26 so the manual chooser can reuse it:
    both the automatic mirror and the operator-driven one need a mounted stick,
    and mounting needs root, which the viewer does not have. Returns
    (None, False) on failure, having already published the reason.
    """
    mounted_here = False
    if premounted:
        mnt = premounted
        log(f"{dev}: already mounted at {mnt} (automounter) - using it")
        return mnt, mounted_here

    mnt = os.path.join(MOUNT_BASE, "usb_backup-" + os.path.basename(dev))
    os.makedirs(mnt, exist_ok=True)
    # uid/gid on the FAT-family mounts so the files are readable as arnobot
    # while mounted; the native-Linux filesystems keep their own ownership and
    # refuse those options. time_offset is what makes the copied files carry the
    # right Date Modified on a Windows laptop - see fat_time_offset().
    if fstype in ("vfat", "exfat"):
        opts = ["-o", "uid=1000,gid=1000,time_offset=%d" % fat_time_offset()]
    else:
        opts = []
    res = subprocess.run(["mount", *opts, dev, mnt],
                         capture_output=True, text=True, timeout=30)
    if res.returncode != 0:
        log(f"{dev}: mount failed: {res.stderr.strip()}")
        publish(state="error", device=dev, detail="mount failed")
        return None, False
    log(f"{dev}: mounted at {mnt} ({fstype})"
        + (f" tz offset {fat_time_offset():+d} min"
           if fstype in ("vfat", "exfat") else ""))
    return mnt, True


def backup_to(dev, fstype, premounted):
    """Mount (if needed), mirror /recordings, clear, sync, unmount."""
    if fstype not in SUPPORTED_FS:
        log(f"{dev}: filesystem '{fstype}' not supported - ignored")
        publish(state="error", device=dev,
                detail=f"unsupported filesystem {fstype}")
        return

    publish(state="mounting", device=dev)
    mnt, mounted_here = mount_stick(dev, fstype, premounted)
    if mnt is None:
        return

    try:
        if not os.path.isdir(RECORD_DIR):
            log(f"{RECORD_DIR} does not exist - nothing to back up")
            publish(state="done", device=dev, copied=0, skipped=0, deleted=0)
            return
        dst = os.path.join(mnt, os.path.basename(RECORD_DIR.rstrip("/")))
        # Walks both trees and stats every file; on a stick with a few full
        # sessions already on it that is not instant.
        publish(state="scanning", device=dev)
        all_files, to_copy, bytes_total = plan_copy(RECORD_DIR, dst)
        # Held from the FIRST scan: all_files is re-listed by the settle loop
        # below, so counting it afterwards would report the wrong thing.
        already_present = len(all_files) - len(to_copy)
        log(f"{dev}: {len(to_copy)} file(s) to copy "
            f"({bytes_total / 1e6:.0f} MB), "
            f"{already_present} already on the stick")

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

        # SETTLE. plan_copy's listing is a snapshot, and two things get written
        # into RECORD_DIR after a save with nobody touching the rig: the full
        # view, built once the save is confirmed, and the per-camera masters,
        # re-encoded in place to match each other. Either can land after the
        # scan, and would then sit on the Pi having never been offered to the
        # stick - while the transfer still truthfully reported zero errors.
        #
        # So re-scan, WAIT for the writers to finish, and copy whatever appeared,
        # until the directory is both empty of new work and quiet. `settled` is
        # what earns the numbering reset further down: only a sweep that ends
        # with nothing left to copy AND nothing still being written means ALL the
        # data is on the stick. See SETTLE_QUIET_S for the measurement that made
        # the wait necessary - re-scanning alone raced the merge and lost.
        #
        # Bounded every way in, because a recording still in progress never
        # converges: the file grows faster than it can be copied. Hitting a bound
        # just means the reset is not earned this time and the stragglers go on
        # the next insertion, which is exactly the conservative outcome.
        settled = False
        copy_rounds = 0
        settle_started = time.monotonic()
        while not errors:
            all_files, late, late_bytes = plan_copy(RECORD_DIR, dst)
            temps, quiet_for = pending_work(RECORD_DIR)
            if not late and not temps and quiet_for >= SETTLE_QUIET_S:
                settled = True
                break

            waited = time.monotonic() - settle_started
            # A temporary present is proof of work that WILL finish; anything
            # else might be a recording that never will.
            cap = SETTLE_BUILD_MAX_S if temps else SETTLE_QUIET_MAX_S
            if waited >= cap:
                log(f"{dev}: {RECORD_DIR} still changing after {waited:.0f}s "
                    f"({len(late)} file(s) to copy, {temps} still being "
                    f"written) - not resetting the session count")
                break

            if late:
                if copy_rounds >= SETTLE_PASSES:
                    log(f"{dev}: still producing files after {SETTLE_PASSES} "
                        f"extra copy pass(es) - not resetting the session count")
                    break
                copy_rounds += 1
                log(f"{dev}: {len(late)} file(s) appeared during the transfer "
                    f"({late_bytes / 1e6:.0f} MB) - copying")
                for i, (s, t, _size) in enumerate(late, start=1):
                    publish(state="copying", device=dev,
                            file=os.path.basename(s), file_i=i,
                            files_total=len(late), bytes_done=0,
                            bytes_total=late_bytes)
                    try:
                        copy_file(s, t, lambda _n: None)
                        copied += 1
                    except OSError as exc:
                        log(f"  copy failed: {s}: {exc}")
                        errors += 1
                continue                # a copy is progress - re-scan at once

            # Nothing to copy, but the recorder has not finished. Say so: this
            # is the window in which the operator, told COPY COMPLETE, used to
            # pull the stick and take home a session with no merged video.
            publish(state="finishing", device=dev, temps=temps,
                    waited=int(waited), cap=int(cap))
            time.sleep(SETTLE_POLL_S)

        took = time.monotonic() - started

        deleted = kept = 0
        if errors:
            # Do not clear a Pi whose backup is not known-complete.
            log(f"{dev}: {errors} copy error(s) - keeping Pi copies")
        elif DELETE_AFTER:
            publish(state="clearing", device=dev, files_total=len(all_files))
            deleted, kept = clear_source(all_files, 0.0 if settled else None)
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
                    f"{already_present} already present, "
                    f"{errors} error(s), {deleted} cleared off the Pi, "
                    f"{took:.0f}s\n")
        except OSError:
            pass

        # The numbering reset. Once a verified copy of EVERYTHING is on the
        # stick, the next session recorded on the Pi starts again at SESSION001
        # (operator spec 2026-08-19: "after all data transferred to it then
        # start storing again from session1, not prev").
        #
        # Both conditions are load-bearing. `not errors` means every file that
        # was offered to the stick landed on it; `settled` means nothing was
        # left un-offered - without it a full view built moments after the scan
        # would strand a file on the Pi AND still reset the count. Neither one
        # alone is "all data transferred".
        if not errors and settled:
            try:
                with open(os.path.join(RECORD_DIR, SESSION_RESET_MARKER), "w",
                          encoding="utf-8") as fh:
                    fh.write("%.3f\n" % time.time())
            except OSError as exc:
                log(f"{dev}: could not write session reset marker: {exc}")

        os.sync()
        skipped = already_present
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
    # SAYS WHICH MODE IT IS IN. The old line announced "delete-after-transfer
    # ON" unconditionally, which since AUTO_COPY went to 0 describes something
    # that never happens - and a log that claims it is about to delete footage
    # when it is not is worse than no log at all.
    if AUTO_COPY:
        log(f"usb_backup up - AUTO-COPY: mirroring {RECORD_DIR} on insert"
            f"{' (delete-after-transfer ON)' if DELETE_AFTER else ''}")
    else:
        log(f"usb_backup up - MANUAL: a stick is mounted and left alone; "
            f"the viewer's chooser saves or deletes. Nothing is copied or "
            f"removed automatically. USB_AUTO_COPY=1 restores mirroring.")
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
            # Said before anything slow happens. Mounting an unclean FAT stick
            # can take seconds and scanning a full one longer; without this the
            # strip showed nothing at all until the first byte moved, which is
            # the exact moment the operator most wants to know it was seen.
            publish(state="detected", device=dev)
            try:
                if AUTO_COPY:
                    backup_to(dev, fstype, mountpoint)
                else:
                    # Mount it and stop. The viewer takes it from here - see
                    # AUTO_COPY. mount_ro=False because the whole point is that
                    # the operator may choose to write to it.
                    if fstype not in SUPPORTED_FS:
                        log(f"{dev}: filesystem '{fstype}' not supported")
                        publish(state="error", device=dev,
                                detail=f"unsupported filesystem {fstype}")
                        continue
                    mnt, _here = mount_stick(dev, fstype, mountpoint)
                    if not mnt:
                        publish(state="error", device=dev,
                                detail="mount failed")
                    else:
                        log(f"{dev}: mounted at {mnt} - waiting for the operator")
                        publish(state="mounted", device=dev, mount=mnt)
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
