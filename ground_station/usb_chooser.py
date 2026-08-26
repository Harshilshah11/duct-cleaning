"""
USB chooser - the popup that opens when a stick is plugged in.

WHY THIS EXISTS. Until 2026-08-26 a stick triggered usb_backup.py to mirror
/recordings and then DELETE it from the Pi, with no way to intervene. The
operator asked for the choice back: a stick now mounts and waits, and nothing
moves until SAVE or DELETE is pressed here.

WHAT IT IS NOT. It does not mount anything. Mounting needs root and the viewer
does not have it, so usb_backup.py still owns finding, mounting and publishing
the stick - it simply stops copying. This reads `mount` out of that published
status and does the file work as arnobot.

THREADED, BECAUSE COPYING IS SLOW. A 60 MB session onto a FAT stick is seconds,
and doing it on the UI thread would freeze the video panels for all of them -
which on this rig is indistinguishable from the crash we spent an evening on.
The worker owns the filesystem; the dialog only ever reads its progress.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time

from PySide6.QtCore import QRectF, QSize, Qt, QThread, Signal, QTimer
from PySide6.QtGui import (QColor, QFont, QIcon, QPainter, QPainterPath, QPen,
                           QPixmap, QRegion)
from PySide6.QtWidgets import (QAbstractItemView, QDialog, QHBoxLayout, QLabel,
                               QListWidget, QListWidgetItem, QMessageBox,
                               QProgressBar, QPushButton, QStyle,
                               QStyledItemDelegate, QVBoxLayout, QWidget)

import config
import theme

# ---------------------------------------------------------------------------
# THE DARK PALETTE, and why this window is the one dark thing in the app.
#
# Operator 2026-08-26, with a Finder screenshot: "make this type layout and
# color code". It is the right instinct for THIS window specifically. Every
# other surface in the rig is a live readout you glance at while driving, so
# they are light, flat and quiet. This one is a file manager - it appears only
# when a stick is plugged in, it takes the operator's full attention, and it
# performs irreversible deletions. Dropping the whole panel to near-black is
# what says "you have stopped driving and you are now moving files".
#
# The values are theme.DARK, which is already Apple's dark system palette -
# same blue, same red, same label tiers. The only literals here are the three
# window-chrome tones and the traffic lights, which have no equivalent in a
# palette built for a light UI.
WIN = theme.DARK["gray6"]            # #1C1C1E - the content ground
CHROME = "#2A2A2C"                   # the toolbar
SIDEBAR = "#212123"                  # the source list, a shade under the rest
CARD = theme.DARK["bg3"]
LINE = "rgba(255, 255, 255, 0.09)"   # hairlines INSIDE the window
EDGE = "rgba(255, 255, 255, 0.18)"   # the window's own 1px rim
TEXT = theme.DARK["label"]
MUTED = theme.DARK["label2"]
FAINT = theme.DARK["label3"]
ACCENT = theme.DARK["blue"]          # #0A84FF - selection, progress, folders
WARN = theme.DARK["orange"]          # a session still being merged
BAD = theme.DARK["red"]
GOOD = theme.DARK["green"]
TRACK = theme.DARK["gray4"]
INK = TEXT



def qcol(spec, alpha=None):
    """QColor from a token, INCLUDING the rgba() strings.

    QColor does not parse CSS rgba(): it returns an invalid colour that paints
    solid black. Every label tier in theme.DARK is written that way because
    those tokens were authored for style sheets, so anything reaching QPainter
    has to come through here.
    """
    m = re.match(r"rgba?\(([^)]*)\)", str(spec).strip())
    if m:
        parts = [x.strip() for x in m.group(1).split(",")]
        c = QColor(int(float(parts[0])), int(float(parts[1])),
                   int(float(parts[2])))
        c.setAlphaF(float(parts[3]) if len(parts) > 3 else 1.0)
    else:
        c = QColor(spec)
    if alpha is not None:
        c.setAlphaF(alpha)
    return c


def _round_mask(w, h, radius):
    """A rounded-rectangle region, for masking a frameless window.

    setMask() rather than WA_TranslucentBackground because there is no
    compositing manager on this rig - .xinitrc runs the viewer under a bare X
    server with no WM at all. An ARGB window without a compositor does not show
    what is behind it; it shows whatever happened to be in the framebuffer. A
    mask is 1-bit and so the corners are a little crisp, but they are actually
    transparent, which is the part that matters.
    """
    path = QPainterPath()
    path.addRoundedRect(QRectF(0, 0, w, h), radius, radius)
    return QRegion(path.toFillPolygon().toPolygon())




def _dir_size(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def _human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return "%.0f %s" % (n, unit) if unit != "GB" else "%.1f %s" % (n, unit)
        n /= 1024.0
    return "%.1f GB" % n


# A per-clip source file: cam1_front_007.mp4, cam2_back_012.mp4, full_003.mp4.
# recorder.py writes one of these per camera per clip while recording and then
# JOINS them into front.mp4 / back.mp4 and deletes them - see
# FullViewBuilder._join_all. Their presence therefore means one thing: the
# merge for this session has not finished yet.
_PART_RE = re.compile(r"_\d{3}\.mp4$", re.I)


def session_files(path):
    """(finished_files, still_working) for one session folder.

    WHY THIS EXISTS. On 2026-08-26 a stick came back holding 68 per-camera
    clips and a leftover .norm temp file alongside the two videos that were
    supposed to be the whole result. The Pi's own copy of that session was
    exactly two files, so nothing was wrong with the recorder or the merge -
    the COPY had run while the merge was still in flight, snapshotted the
    working files, and then (copytree with dirs_exist_ok, on a second press)
    topped the same folder up with the finished ones. The stick ended up
    holding the union of two moments.

    So the copy no longer takes a directory. It takes the finished files by
    name, and a session that still has parts or temps in it is not offered.
    """
    working = False
    finals = []
    try:
        names = os.listdir(path)
    except OSError:
        return [], False
    for n in names:
        if n.startswith(".") or not n.lower().endswith(".mp4"):
            working = True            # .norm / .tmp / the concat list file
        elif _PART_RE.search(n):
            working = True            # a clip the join has not consumed yet
        else:
            finals.append(n)
    return sorted(finals), working


def list_sessions(root):
    """Session folders, NEWEST FIRST, as (name, path, bytes, mtime).

    Sorted on mtime rather than on the name. The names lead with a session
    number that RESETS after a transfer, so sorting them as text puts session01
    of today above session12 of an hour ago - see recorder._next_session_no.
    """
    out = []
    try:
        for name in os.listdir(root):
            path = os.path.join(root, name)
            if not os.path.isdir(path):
                continue
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                mtime = 0.0
            finals, working = session_files(path)
            # Sized on what will actually be COPIED, not on what is in the
            # folder - mid-merge those are wildly different numbers, and
            # the one the operator needs is the one that has to fit.
            size = 0
            for fn in finals:
                try:
                    size += os.path.getsize(os.path.join(path, fn))
                except OSError:
                    pass
            state = "working" if working or not finals else "ready"
            out.append((name, path, size, mtime, state))
    except OSError:
        return []
    out.sort(key=lambda row: row[3], reverse=True)
    return out


class _Worker(QThread):
    """Copies or deletes in the background. One job per instance."""

    progressed = Signal(str, int, int)      # label, done, total
    finished_ok = Signal(str)               # summary
    failed = Signal(str)

    def __init__(self, jobs, dest=None, delete=False, parent=None):
        super().__init__(parent)
        self._jobs = list(jobs)             # [(name, path, size, mtime), ...]
        self._dest = dest
        self._delete = delete
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        done = 0
        total = max(1, len(self._jobs))
        try:
            for name, path, _size, _mtime, _state in self._jobs:
                if self._stop:
                    break
                self.progressed.emit(name, done, total)
                if self._delete:
                    shutil.rmtree(path, ignore_errors=True)
                else:
                    # BY NAME, NOT copytree. The finished files are re-read
                    # here rather than trusted from the list, so even if a
                    # session finished merging between the click and this
                    # line, what lands on the stick is still only the joined
                    # videos. copytree took the folder, which is how 68 clips
                    # and a .norm temp got onto a stick - see session_files.
                    finals, working = session_files(path)
                    if working or not finals:
                        raise RuntimeError(
                            "%s is still being processed" % name)
                    dst = os.path.join(self._dest, name)
                    os.makedirs(dst, exist_ok=True)
                    for fn in finals:
                        shutil.copy2(os.path.join(path, fn),
                                     os.path.join(dst, fn))
                done += 1
            # One sync for the whole batch: per-file sync on a FAT stick costs
            # more than the copy. Without it a stick pulled straight after
            # "done" can still be missing the tail of the last file.
            if not self._delete:
                os.sync()
        except Exception as exc:                       # never take the UI down
            self.failed.emit(str(exc)[:160])
            return
        verb = "deleted" if self._delete else "saved"
        self.finished_ok.emit("%d session%s %s"
                              % (done, "" if done == 1 else "s", verb))


class StorageBar(QWidget):
    """Used / free on the Pi's recording volume, along the foot of the window."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(52)
        self._used = self._total = 0

    def refresh(self, path):
        try:
            st = os.statvfs(path)
            self._total = st.f_blocks * st.f_frsize
            self._used = self._total - (st.f_bavail * st.f_frsize)
        except OSError:
            self._used = self._total = 0
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        frac = (self._used / self._total) if self._total else 0.0
        full = frac > 0.9

        bar_h = 8
        y = h - bar_h - 10
        p.setPen(Qt.NoPen)
        p.setBrush(qcol("rgba(255, 255, 255, 0.10)"))
        p.drawRoundedRect(QRectF(0, y, w, bar_h), bar_h / 2, bar_h / 2)
        if frac > 0:
            # Red once the card is nearly full: this is the number that decides
            # whether the next run has anywhere to go.
            p.setBrush(qcol(BAD if full else ACCENT))
            p.drawRoundedRect(QRectF(0, y, max(bar_h, w * frac), bar_h),
                              bar_h / 2, bar_h / 2)

        p.setFont(theme.font_for(theme.FOOTNOTE, theme.W_SEMIBOLD))
        p.setPen(qcol(BAD if full else MUTED))
        p.drawText(0, 0, w, y - 4, Qt.AlignLeft | Qt.AlignVCenter,
                   "%s of %s used" % (_human(self._used), _human(self._total)))
        p.setPen(qcol(BAD if full else MUTED))
        p.drawText(0, 0, w, y - 4, Qt.AlignRight | Qt.AlignVCenter,
                   "%s available" % _human(self._total - self._used))
        p.end()


class ProgressPopup(QDialog):
    """The little copy/delete sheet, in the same dark chrome as its parent.

    Operator 2026-08-26: "when i delete all and save to other small popup open
    like when i copy and paste to open popup in windows ... when its complete to
    close their small popup".

    WHY A SEPARATE WINDOW rather than the inline bar it replaces. The inline bar
    sat inside a dialog whose list was still there to be clicked, so a copy in
    flight looked like a chooser that had merely grown a bar - the operator
    could still move the cursor and tick rows while the files under them were
    being deleted. This is MODAL: while it is up the chooser cannot be touched,
    which is the honest description of what is happening.

    NO CANCEL BUTTON, deliberately. Stopping a copytree halfway leaves a session
    on the stick that LOOKS complete and is not, and there is no way to tell
    afterwards. The jobs here are seconds; waiting is cheaper than a folder that
    lies about what it holds.
    """

    RADIUS = 12

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setFixedSize(500, 152)
        # Frameless for the same reason the chooser is: there is no window
        # manager to draw a frame, so the window draws its own or has none.
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setStyleSheet(
            "QDialog { background: %s; }"
            "QProgressBar { background: rgba(255,255,255,0.10); border: none;"
            "   border-radius: 4px; height: 8px; text-align: center;"
            "   color: transparent; }"
            "QProgressBar::chunk { background: %s; border-radius: 4px; }"
            % (WIN, ACCENT))

        self.head = QLabel(title)
        self.head.setFont(theme.font_for(theme.HEADLINE, theme.W_SEMIBOLD))
        self.head.setStyleSheet("color: %s;" % TEXT)

        self.count = QLabel("")
        self.count.setFont(theme.font_for(theme.FOOTNOTE, theme.W_MEDIUM))
        self.count.setStyleSheet("color: %s;" % MUTED)
        self.count.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.detail = QLabel("Starting\u2026")
        self.detail.setFont(theme.font_for(theme.FOOTNOTE, theme.W_REGULAR))
        self.detail.setStyleSheet("color: %s;" % FAINT)
        self.detail.setWordWrap(False)

        self.bar = QProgressBar()
        self.bar.setTextVisible(False)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.addWidget(self.head)
        top.addStretch(1)
        top.addWidget(self.count)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(26, 24, 26, 24)
        lay.setSpacing(14)
        lay.addLayout(top)
        lay.addWidget(self.bar)
        lay.addWidget(self.detail)

    def resizeEvent(self, event):
        self.setMask(_round_mask(self.width(), self.height(), self.RADIUS))
        super().resizeEvent(event)

    def showEvent(self, event):
        _centre_on_parent(self)
        super().showEvent(event)

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(qcol(EDGE), 1.0))
        p.drawRoundedRect(QRectF(0.5, 0.5, self.width() - 1, self.height() - 1),
                          self.RADIUS, self.RADIUS)
        p.end()

    def update_progress(self, name, done, total):
        self.bar.setRange(0, total)
        self.bar.setValue(done)
        self.count.setText("%d of %d" % (min(done + 1, total), total))
        # Elide from the LEFT: session names differ at the end (the times), so
        # trimming the tail would make every row read the same.
        self.detail.setText(name if len(name) <= 52 else "\u2026" + name[-51:])


def _centre_on_parent(dlg):
    """Put a frameless dialog in the middle of what it belongs to.

    With no window manager nothing places windows, so an unplaced dialog maps
    at whatever geometry Qt last guessed - which on this rig is the top-left
    corner, half off the panel.
    """
    par = dlg.parentWidget()
    ref = par.window().geometry() if par is not None else None
    if ref is None:
        scr = dlg.screen()
        ref = scr.availableGeometry() if scr is not None else None
    if ref is not None:
        dlg.move(ref.center().x() - dlg.width() // 2,
                 ref.center().y() - dlg.height() // 2)


def glyph(kind, colour, px=20):
    """One stroked SF-ish icon, drawn to a pixmap in the colour asked for.

    Drawn rather than shipped: this rig has no icon theme installed and no
    network to fetch one, and four hand-stroked paths weigh less than a font
    dependency. The colour is a parameter because a sidebar row inverts to
    white when it takes focus, so every icon exists in two tones and is
    re-rendered on the focus change - see UsbChooser._paint_focus.
    """
    pm = QPixmap(px, px)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(Qt.NoBrush)
    p.setPen(QPen(qcol(colour), 1.8, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
    s = px / 20.0

    def L(x1, y1, x2, y2):
        p.drawLine(QRectF(x1 * s, y1 * s, 0, 0).topLeft(),
                   QRectF(x2 * s, y2 * s, 0, 0).topLeft())

    if kind == "save":                      # arrow down into a tray
        L(10, 2.5, 10, 12.5)
        path = QPainterPath()
        path.moveTo(6.0 * s, 9.0 * s)
        path.lineTo(10.0 * s, 13.0 * s)
        path.lineTo(14.0 * s, 9.0 * s)
        p.drawPath(path)
        path = QPainterPath()
        path.moveTo(3.5 * s, 13.5 * s)
        path.lineTo(3.5 * s, 17.0 * s)
        path.lineTo(16.5 * s, 17.0 * s)
        path.lineTo(16.5 * s, 13.5 * s)
        p.drawPath(path)
    elif kind == "trash":                   # lid, then a tapered body
        L(3.0, 5.0, 17.0, 5.0)
        L(8.0, 5.0, 8.0, 3.0)
        L(8.0, 3.0, 12.0, 3.0)
        L(12.0, 3.0, 12.0, 5.0)
        path = QPainterPath()
        path.moveTo(5.0 * s, 5.0 * s)
        path.lineTo(6.0 * s, 17.0 * s)
        path.lineTo(14.0 * s, 17.0 * s)
        path.lineTo(15.0 * s, 5.0 * s)
        p.drawPath(path)
        L(8.5, 8.0, 8.8, 14.0)
        L(11.5, 8.0, 11.2, 14.0)
    elif kind == "check":                   # tick in a circle
        p.drawEllipse(QRectF(2.2 * s, 2.2 * s, 15.6 * s, 15.6 * s))
        path = QPainterPath()
        path.moveTo(6.2 * s, 10.2 * s)
        path.lineTo(9.0 * s, 13.2 * s)
        path.lineTo(14.0 * s, 7.2 * s)
        p.drawPath(path)
    elif kind == "exit":                    # out through a door
        path = QPainterPath()
        path.moveTo(11.5 * s, 3.0 * s)
        path.lineTo(4.0 * s, 3.0 * s)
        path.lineTo(4.0 * s, 17.0 * s)
        path.lineTo(11.5 * s, 17.0 * s)
        p.drawPath(path)
        L(8.5, 10.0, 17.0, 10.0)
        path = QPainterPath()
        path.moveTo(13.8 * s, 6.6 * s)
        path.lineTo(17.2 * s, 10.0 * s)
        path.lineTo(13.8 * s, 13.4 * s)
        p.drawPath(path)
    p.end()
    return pm


def usb_label(mount):
    """The stick's volume name, for the toolbar.

    /proc/mounts gives the device behind the mount point and lsblk gives its
    LABEL. The basename of the mount is the fallback and is usually the same
    thing - udisks mounts at /media/<user>/<LABEL> - but it is only the label
    by CONVENTION, and a stick mounted by hand at /media/usb would otherwise
    put the word "usb" on screen as though that were its name.
    """
    if not mount:
        return "NO DRIVE"
    dev = ""
    try:
        with open("/proc/mounts") as fh:
            for line in fh:
                bits = line.split()
                if len(bits) > 1 and bits[1] == str(mount).rstrip("/"):
                    dev = bits[0]
                    break
    except OSError:
        pass
    if dev:
        try:
            out = subprocess.run(["lsblk", "-no", "LABEL", dev],
                                 capture_output=True, text=True, timeout=3)
            name = (out.stdout or "").strip().splitlines()
            if name and name[0].strip():
                return name[0].strip().upper()
        except (OSError, subprocess.SubprocessError):
            pass
    base = os.path.basename(str(mount).rstrip("/"))
    return (base or "USB DRIVE").upper()


def usb_space(mount):
    """(total, available) on the stick, or (0, 0) if it cannot be read."""
    try:
        st = os.statvfs(mount)
        return st.f_blocks * st.f_frsize, st.f_bavail * st.f_frsize
    except (OSError, TypeError):
        return 0, 0


class TitleBar(QWidget):
    """The window's own toolbar: traffic lights over the sidebar, title centred.

    Split down the same seam as the body - the sidebar tone runs up behind the
    lights, the toolbar tone starts where the file list starts. That single
    detail is most of what makes a Finder window look like a Finder window.
    """

    HEIGHT = 54

    def __init__(self, title, split, parent=None):
        super().__init__(parent)
        self._title = title
        self._sub = ""
        self._split = split
        self.setFixedHeight(self.HEIGHT)

    def set_subtitle(self, text):
        self._sub = text or ""
        self.update()

    def set_title(self, text):
        self._title = text or ""
        self.update()

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setPen(Qt.NoPen)
        p.setBrush(qcol(SIDEBAR))
        p.drawRect(0, 0, self._split, h)
        p.setBrush(qcol(CHROME))
        p.drawRect(self._split, 0, w - self._split, h)
        p.setPen(QPen(qcol(LINE), 1.0))
        p.drawLine(0, h - 1, w, h - 1)

        # NO TRAFFIC LIGHTS, operator 2026-08-26. They were the strongest
        # cue that this is a window rather than part of the panel, but
        # they are also three controls that cannot be operated - there is
        # no pointer on this rig - sitting where the eye goes first. The
        # rounded rim and the drawn toolbar carry the same message
        # without offering something that does not work.

        p.setFont(theme.font_for(theme.SUBHEAD, theme.W_SEMIBOLD))
        p.setPen(qcol(TEXT))
        p.drawText(self._split, 0, w - self._split, h, Qt.AlignCenter,
                   self._title)
        if self._sub:
            p.setFont(theme.font_for(theme.FOOTNOTE, theme.W_MEDIUM))
            p.setPen(qcol(MUTED))
            p.drawText(self._split, 0, w - self._split - 22, h,
                       Qt.AlignRight | Qt.AlignVCenter, self._sub)
        p.end()


class SessionDelegate(QStyledItemDelegate):
    """One recording, drawn as a Finder row: folder, name, size and date.

    A delegate rather than item text because the row carries four facts at two
    weights and two alignments, and a QListWidgetItem carries one string. The
    old row was 'name<tab><tab>size' in one colour, which put the date nowhere
    and made every row the same shape - so a list of eight sessions was eight
    identical grey lines and the newest one did not look like anything.
    """

    ROW_H = 56

    def __init__(self, parent=None):
        super().__init__(parent)
        self.focused = True         # does the LIST hold panel focus just now

    def sizeHint(self, _option, _index):
        return QSize(0, self.ROW_H)

    def paint(self, p, option, index):
        data = index.data(Qt.UserRole)
        if not data:
            return
        name, _path, size, mtime, state = data
        working = state != "ready"
        # CheckStateRole comes back as a plain int here, not the enum, so
        # comparing it to Qt.Checked is always False - which drew every
        # box empty while the sidebar correctly said '(2)'.
        checked = int(index.data(Qt.CheckStateRole) or 0) == 2
        cursor = bool(option.state & QStyle.State_Selected)

        p.save()
        p.setRenderHint(QPainter.Antialiasing)
        r = QRectF(option.rect).adjusted(8, 3, -8, -3)

        # THE CURSOR IS NOT THE CHOICE. The highlight says where the joystick
        # is; the tick says what will be copied or deleted. They are drawn
        # differently on purpose - a filled blue row and a ticked box are not
        # the same statement, and conflating them is how the wrong session gets
        # erased. Blue only while the list holds focus; when focus is in the
        # sidebar the cursor drops to grey, exactly as Finder does.
        if cursor:
            p.setPen(Qt.NoPen)
            p.setBrush(qcol(ACCENT) if self.focused
                       else qcol("rgba(255, 255, 255, 0.10)"))
            p.drawRoundedRect(r, 7, 7)
        on_blue = cursor and self.focused

        # -- the tick box --------------------------------------------------
        # A session still being merged has no box at all. Greying a tick
        # box invites a press to see what happens; leaving the slot empty
        # says there is nothing to press.
        box = QRectF(r.left() + 14, r.center().y() - 10, 20, 20)
        if working:
            pass
        elif checked:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#ffffff") if on_blue else qcol(ACCENT))
            p.drawRoundedRect(box, 6, 6)
            tick = QPainterPath()
            tick.moveTo(box.left() + 5.0, box.center().y() + 0.2)
            tick.lineTo(box.center().x() - 0.8, box.bottom() - 5.6)
            tick.lineTo(box.right() - 4.6, box.top() + 5.8)
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(qcol(ACCENT) if on_blue else QColor("#ffffff"), 2.4,
                          Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.drawPath(tick)
        else:
            p.setBrush(Qt.NoBrush)
            p.setPen(QPen(QColor(255, 255, 255, 110 if on_blue else 70), 1.6))
            p.drawRoundedRect(box.adjusted(0.8, 0.8, -0.8, -0.8), 5.4, 5.4)

        # -- the folder ----------------------------------------------------
        self._folder(p, QRectF(r.left() + 50, r.center().y() - 12, 30, 24),
                     on_blue)

        # -- name, then size and date under it ------------------------------
        tx = r.left() + 94
        p.setFont(theme.font_for(theme.SUBHEAD, theme.W_SEMIBOLD))
        p.setPen(QColor("#ffffff") if (on_blue or True) else qcol(TEXT))
        p.drawText(QRectF(tx, r.top() + 8, r.width() - 110, 20),
                   Qt.AlignLeft | Qt.AlignVCenter, name)

        p.setFont(theme.font_for(theme.FOOTNOTE, theme.W_SEMIBOLD if working
                                 else theme.W_REGULAR))
        when = time.strftime("%d %b %Y  %H:%M", time.localtime(mtime))
        if working:
            p.setPen(QColor(255, 255, 255, 220) if on_blue else qcol(WARN))
            line = "PROCESSING \u2014 not ready to copy   \u00b7   %s" % when
        else:
            p.setPen(QColor(255, 255, 255, 200) if on_blue else qcol(MUTED))
            line = "%s   \u00b7   %s" % (_human(size), when)
        p.drawText(QRectF(tx, r.bottom() - 26, r.width() - 110, 18),
                   Qt.AlignLeft | Qt.AlignVCenter, line)
        p.restore()

    @staticmethod
    def _folder(p, r, on_blue):
        """A folder glyph. Two tones, back flap darker, like every file manager."""
        back = QColor("#ffffff") if on_blue else qcol("#4E9BF0")
        front = QColor(255, 255, 255, 205) if on_blue else qcol("#6EB6F5")
        tab = QPainterPath()
        tab.addRoundedRect(QRectF(r.left(), r.top(), r.width() * 0.52,
                                  r.height() * 0.34), 3, 3)
        p.setPen(Qt.NoPen)
        p.setBrush(back)
        p.drawPath(tab)
        p.drawRoundedRect(QRectF(r.left(), r.top() + r.height() * 0.16,
                                 r.width(), r.height() * 0.84), 4, 4)
        p.setBrush(front)
        p.drawRoundedRect(QRectF(r.left(), r.top() + r.height() * 0.28,
                                 r.width(), r.height() * 0.72), 4, 4)


class UsbChooser(QDialog):
    """The popup itself. Opened by main.py when a stick reports `mounted`."""

    def __init__(self, root, mount, parent=None):
        super().__init__(parent)
        self.root = root
        self.mount = mount
        self._worker = None
        self._popup = None
        # Panel-navigation state - see on_inputs().
        self._zone = "list"          # "list" or "buttons"
        self._btn_index = 0
        self._v_dir = 0
        self._v_since = 0.0
        self._v_next = None
        self._h_dir = 0
        self._saves_seen = None
        self.setWindowTitle("USB DRIVE")
        self.setModal(True)
        # BIGGER, operator 2026-08-26: "big some size in main popup". 760x460
        # gave five rows before it scrolled, on a 1920x1080 panel with nothing
        # else competing for the space while this is up. 1140x700 shows eleven
        # and still leaves the video visible around every edge, which is the
        # reason not to simply run it full-screen.
        self.setFixedSize(1140, 700)
        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)

        self.setStyleSheet(
            "QDialog { background: %(win)s; }"
            "QWidget#sidebar { background: %(side)s; }"
            "QWidget#content { background: %(win)s; }"
            "QLabel#section { color: %(faint)s; padding-left: 15px; }"
            # ONE rule set for every sidebar row, switched by two dynamic
            # properties: `tone` (danger or not) and `navfocus` (does the
            # joystick sit here). Doing it with properties rather than by
            # writing a stylesheet per button - which is what the light
            # version did - means the focus ring cannot fight the base style
            # and blank the row, which is exactly what happened the first time.
            "QPushButton#nav { background: transparent; color: %(text)s;"
            "   border: none; border-radius: 7px; text-align: left;"
            "   padding-left: 13px; padding-right: 12px; }"
            "QPushButton#nav:disabled { color: %(faint)s; }"
            "QPushButton#nav[tone=\"danger\"] { color: %(bad)s; }"
            "QPushButton#nav[tone=\"danger\"]:disabled {"
            "   color: rgba(255, 69, 58, 0.35); }"
            "QPushButton#nav[navfocus=\"true\"] { background: %(accent)s;"
            "   color: #ffffff; }"
            "QPushButton#nav[tone=\"danger\"][navfocus=\"true\"] {"
            "   background: %(bad)s; color: #ffffff; }"
            # A focused row that is DISABLED still has to show where the
            # joystick is, or the operator pushes down and nothing appears to
            # move. Grey slab, grey text - present, plainly not pressable.
            "QPushButton#nav[navfocus=\"true\"]:disabled {"
            "   background: rgba(255, 255, 255, 0.10); color: %(faint)s; }"
            "QListWidget { background: transparent; border: none;"
            "   outline: none; }"
            "QScrollBar:vertical { background: transparent; width: 11px;"
            "   margin: 4px 2px 4px 0; }"
            "QScrollBar::handle:vertical { background: rgba(255,255,255,0.22);"
            "   border-radius: 4px; min-height: 40px; }"
            "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
            "   height: 0; }"
            "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {"
            "   background: transparent; }"
            % {"win": WIN, "side": SIDEBAR, "faint": FAINT, "text": TEXT,
               "bad": BAD, "accent": ACCENT})

        self.title_bar = TitleBar(self._drive_title(), self.SIDEBAR_W, self)

        # -- the source list ------------------------------------------------
        self.btn_save = QPushButton("  Save to USB")
        self.btn_delete = QPushButton("  Delete")
        self.btn_all = QPushButton("  Select All")
        self.btn_delete_all = QPushButton("  Delete All")
        self.btn_exit = QPushButton("  Eject and Close")
        self._icons = {self.btn_save: "save", self.btn_delete: "trash",
                       self.btn_all: "check", self.btn_delete_all: "trash",
                       self.btn_exit: "exit"}
        for b in (self.btn_delete, self.btn_delete_all):
            b.setProperty("tone", "danger")
        for b in self._buttons():
            b.setObjectName("nav")
            b.setFixedHeight(38)
            b.setIconSize(QSize(19, 19))
            b.setFont(theme.font_for(theme.SUBHEAD, theme.W_MEDIUM))
        self.btn_save.clicked.connect(self._save)
        self.btn_delete.clicked.connect(self._delete)
        self.btn_all.clicked.connect(self._select_all)
        self.btn_delete_all.clicked.connect(self._delete_all)
        self.btn_exit.clicked.connect(self.reject)

        side = QVBoxLayout()
        side.setContentsMargins(9, 12, 9, 14)
        side.setSpacing(2)
        side.addWidget(self._section("SELECTED"))
        side.addWidget(self.btn_save)
        side.addWidget(self.btn_delete)
        side.addSpacing(16)
        side.addWidget(self._section("EVERYTHING"))
        side.addWidget(self.btn_all)
        side.addWidget(self.btn_delete_all)
        # EXIT sits at the BOTTOM, pushed there by the stretch, so it is the
        # furthest thing from DELETE ALL. The two are adjacent in the
        # navigation ring but not under the hand, which is the point - one
        # closes a window, the other erases every recording on the Pi.
        side.addStretch(1)
        side.addWidget(self.btn_exit)
        sidebar = QWidget()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(self.SIDEBAR_W)
        sidebar.setLayout(side)

        # -- the file list ---------------------------------------------------
        self.hint = QLabel("")
        self.hint.setFont(theme.font_for(theme.FOOTNOTE, theme.W_MEDIUM))
        self.hint.setStyleSheet("color: %s; padding: 0 4px;" % MUTED)

        # SINGLE selection, because the highlight is now a CURSOR, not the
        # choice. What is chosen is what is TICKED - the operator drives this
        # with a joystick and one button, and a multi-select highlight cannot
        # express "these four" with one button and no modifier key.
        self.list = QListWidget()
        self.list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.list.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.list.setFrameShape(QListWidget.NoFrame)
        self._delegate = SessionDelegate(self.list)
        self.list.setItemDelegate(self._delegate)
        self.list.currentRowChanged.connect(lambda _r: self._sync_buttons())
        self.list.itemChanged.connect(lambda _i: self._sync_buttons())

        self.storage = StorageBar()

        content_l = QVBoxLayout()
        content_l.setContentsMargins(14, 10, 14, 12)
        content_l.setSpacing(8)
        content_l.addWidget(self.hint)
        content_l.addWidget(self.list, 1)
        content_l.addWidget(self.storage)
        content = QWidget()
        content.setObjectName("content")
        content.setLayout(content_l)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(sidebar)
        body.addWidget(content, 1)

        root_l = QVBoxLayout(self)
        root_l.setContentsMargins(0, 0, 0, 0)
        root_l.setSpacing(0)
        root_l.addWidget(self.title_bar)
        root_l.addLayout(body, 1)

        self._paint_focus()
        self.reload()
        # A session that is still merging when this opens becomes copyable
        # a minute later, and the operator should not have to unplug the
        # stick and plug it back in to find that out.
        self._watch = QTimer(self)
        self._watch.timeout.connect(self._refresh_states)
        self._watch.start(2000)

    SIDEBAR_W = 250
    RADIUS = 14

    def _drive_title(self):
        """NAME (total/free), operator 2026-08-26 - the stick, not the Pi.

        The foot of the window already carries the Pi's card; this is the
        other volume in the transfer and until now it was named nowhere. The
        pair is total first, then what is still free, which is the order the
        operator asked for.
        """
        total, avail = usb_space(self.mount)
        if not total:
            return usb_label(self.mount)
        return "%s   (%s / %s)" % (usb_label(self.mount),
                                   _human(total), _human(avail))


    @staticmethod
    def _section(text):
        lab = QLabel(text)
        lab.setObjectName("section")
        f = theme.font_for(theme.CAPTION2, theme.W_BOLD)
        f.setLetterSpacing(QFont.AbsoluteSpacing, 0.7)
        lab.setFont(f)
        lab.setFixedHeight(26)
        return lab

    def resizeEvent(self, event):
        self.setMask(_round_mask(self.width(), self.height(), self.RADIUS))
        super().resizeEvent(event)

    def showEvent(self, event):
        _centre_on_parent(self)
        super().showEvent(event)

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        # The seam between the source list and the files, and the window's own
        # rim. Without the rim a dark window on a dark video has no edge at all.
        p.setPen(QPen(qcol(LINE), 1.0))
        p.drawLine(self.SIDEBAR_W, TitleBar.HEIGHT, self.SIDEBAR_W,
                   self.height())
        p.setBrush(Qt.NoBrush)
        p.setPen(QPen(qcol(EDGE), 1.0))
        p.drawRoundedRect(QRectF(0.5, 0.5, self.width() - 1, self.height() - 1),
                          self.RADIUS, self.RADIUS)
        p.end()

    # -- data -----------------------------------------------------------------

    def reload(self):
        self.list.clear()
        self._rows = list_sessions(self.root)
        for row in self._rows:
            item = QListWidgetItem()
            item.setData(Qt.UserRole, row)
            item.setSizeHint(QSize(0, SessionDelegate.ROW_H))
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.list.addItem(item)
        if self.list.count():
            self.list.setCurrentRow(0)
        total = sum(r[2] for r in self._rows)
        self.title_bar.set_subtitle(
            "%d recording%s   \u00b7   %s"
            % (len(self._rows), "" if len(self._rows) == 1 else "s",
               _human(total)) if self._rows else "empty")
        self.hint.setText(
            "Joystick to move   \u00b7   SAVE to tick a recording   \u00b7   "
            "right for the menu, left back to the list"
            if self._rows else "No recordings on the Pi.")
        self.title_bar.set_title(self._drive_title())
        self.storage.refresh(self.root)
        self._sync_buttons()

    def _refresh_states(self):
        """Re-read the folders and update the rows IN PLACE.

        Not reload(): that rebuilds the list, which would drop the ticks and
        send the cursor back to the top every two seconds. Only the row data
        changes, so a session flipping from PROCESSING to ready keeps whatever
        the operator had already chosen.
        """
        if self._worker is not None and self._worker.isRunning():
            return
        rows = list_sessions(self.root)
        old = [self.list.item(i).data(Qt.UserRole)[0]
               for i in range(self.list.count())]
        if [r[0] for r in rows] != old:
            self.reload()               # one appeared or vanished
            return
        self._rows = rows
        changed = False
        for i, row in enumerate(rows):
            item = self.list.item(i)
            if item.data(Qt.UserRole) != row:
                item.setData(Qt.UserRole, row)
                changed = True
        if changed:
            self.title_bar.set_title(self._drive_title())
            self.storage.refresh(self.root)
            self._sync_buttons()
            self.list.viewport().update()

    def _selected(self):
        """The TICKED rows. Never the highlighted one - that is just the cursor."""
        out = []
        for row in range(self.list.count()):
            item = self.list.item(row)
            if item.checkState() == Qt.Checked:
                out.append(item.data(Qt.UserRole))
        return out

    def _toggle_current(self):
        """The panel SAVE button's job in here: tick or untick the cursor row."""
        item = self.list.currentItem()
        if item is None:
            return
        row = item.data(Qt.UserRole)
        if row and row[4] != "ready":
            # Nothing to copy yet, so there is nothing to choose. Silent
            # rather than a toast: the row itself already says PROCESSING
            # on the line the operator is looking at.
            return
        item.setCheckState(Qt.Unchecked
                           if item.checkState() == Qt.Checked else Qt.Checked)

    def _sync_buttons(self):
        busy = self._worker is not None and self._worker.isRunning()
        sel = self._selected()
        n = len(sel)
        ready = all(row[4] == "ready" for row in sel)
        self.btn_save.setEnabled(bool(n) and ready and not busy
                                 and bool(self.mount))
        self.btn_delete.setEnabled(bool(n) and not busy)
        self.btn_all.setEnabled(bool(self._rows) and not busy)
        self.btn_delete_all.setEnabled(bool(self._rows) and not busy)
        self.btn_save.setText("  Save to USB" if not n
                              else "  Save to USB  (%d)" % n)
        self.btn_delete.setText("  Delete" if not n
                                else "  Delete  (%d)" % n)

    def _select_all(self):
        for row in range(self.list.count()):
            item = self.list.item(row)
            data = item.data(Qt.UserRole)
            if data and data[4] == "ready":
                item.setCheckState(Qt.Checked)

    # -- panel navigation -----------------------------------------------------
    #
    # THE POPUP IS DRIVEN FROM THE PANEL, not from a keyboard the rig does not
    # have. main.py pushes an inputs snapshot in every UI frame and this turns
    # it into movement:
    #
    #     joystick DOWN / UP   move the cursor in whichever column has focus
    #     joystick RIGHT       focus the button column
    #     joystick LEFT        focus the video list
    #     SAVE button          tick / untick the row under the cursor, or press
    #                          the focused button
    #
    # EDGE-TRIGGERED, THEN REPEATING. A stick held down is one intent, not sixty
    # a second: the first push moves one row, and only after NAV_HOLD_S does it
    # start repeating at NAV_REPEAT_S. Without that, a single nudge scrolled to
    # the end of the list before the operator could let go.

    NAV_DEADBAND = 0.45         # of full stick travel
    NAV_HOLD_S = 2.0            # hold this long before auto-repeat starts
    NAV_REPEAT_S = 1.0          # then one step per this many seconds

    def _buttons(self):
        return [self.btn_save, self.btn_delete, self.btn_all,
                self.btn_delete_all, self.btn_exit]

    def _paint_focus(self):
        """Move the highlight, and re-render the icons that go white under it.

        Dynamic properties + a repolish, NOT a per-widget stylesheet. Setting
        one on a button REPLACES the sheet it inherits, so the old version's
        focus ring also erased the row's background and text colour - the
        focused row was the one that looked broken.
        """
        for i, b in enumerate(self._buttons()):
            focused = (self._zone == "buttons" and i == self._btn_index)
            b.setProperty("navfocus", "true" if focused else "false")
            if focused and b.isEnabled():
                tone = "#ffffff"
            elif not b.isEnabled():
                tone = FAINT
            else:
                tone = BAD if b.property("tone") == "danger" else TEXT
            b.setIcon(QIcon(glyph(self._icons[b], tone)))
            b.style().unpolish(b)
            b.style().polish(b)
        self._delegate.focused = (self._zone == "list")
        self.list.viewport().update()

    def on_inputs(self, snap):
        """One inputs.py snapshot per UI frame. Safe to call at frame rate."""
        joy = (snap or {}).get("joy") or {}
        # AXES SWAPPED, IN THIS WINDOW ONLY, operator 2026-08-26: "in
        # popup view x is y and y is x ... only change in popup view".
        # inputs.py applies INPUTS_SWAP_XY once for DRIVING, and
        # inputs_panel then swaps them back again for the joystick
        # display - so the snapshot carries the driving axes, which are
        # not the ones the operator sees under their hand. Everywhere
        # else that is right, because everywhere else the stick is
        # steering a robot. In here it is steering a cursor and the only
        # reference is the physical stick, so this window takes them the
        # display way round. Nothing outside these two lines changes; if
        # a direction ends up mirrored rather than transposed, it is the
        # sign that is wrong here, not the swap.
        x, y = joy.get("y"), joy.get("x")
        now = time.monotonic()

        # Vertical: step the cursor.
        step = 0
        if y is not None and abs(y) >= self.NAV_DEADBAND:
            # y is already oriented so that pushing the stick UP is negative -
            # see inputs.INVERT_Y. Up must move UP the list.
            want = -1 if y < 0 else 1
            if self._v_dir != want:
                self._v_dir, self._v_since, self._v_next = want, now, None
                step = want                      # the first push always moves
            elif self._v_next is None:
                if now - self._v_since >= self.NAV_HOLD_S:
                    self._v_next = now + self.NAV_REPEAT_S
                    step = want
            elif now >= self._v_next:
                self._v_next = now + self.NAV_REPEAT_S
                step = want
        else:
            self._v_dir, self._v_next = 0, None

        if step:
            self._move(step)

        # Horizontal: swap column. Edge only - holding right must not walk off.
        if x is not None and abs(x) >= self.NAV_DEADBAND:
            want = "buttons" if x > 0 else "list"
            if self._h_dir == 0:
                self._h_dir = 1 if x > 0 else -1
                if want != self._zone:
                    self._zone = want
                    if self._zone == "buttons":
                        self._btn_index = 0
                    self._paint_focus()
        else:
            self._h_dir = 0

        # The SAVE button: select here, not save. Edge-triggered on the press
        # COUNT, which is how inputs.py reports it - a level would fire every
        # frame the switch is held.
        presses = (snap or {}).get("save_presses")
        if presses is not None and presses != self._saves_seen:
            if self._saves_seen is not None and presses > self._saves_seen:
                self._activate()
            self._saves_seen = presses

    def _move(self, step):
        if self._zone == "list":
            n = self.list.count()
            if n:
                self.list.setCurrentRow(max(0, min(n - 1,
                                                   self.list.currentRow() + step)))
        else:
            # WRAPS, operator 2026-08-26: "run all button continuous". Off the
            # bottom returns to the top and vice versa, so the column is a ring
            # and no amount of pushing can leave the stick apparently dead
            # against an end stop.
            #
            # The LIST deliberately does NOT wrap: a file list can be long, and
            # a cursor that leaps from the newest recording to the oldest
            # because the stick was held a moment too long is a way to delete
            # the wrong thing. Say the word if you want it ringed too.
            btns = self._buttons()
            self._btn_index = (self._btn_index + step) % len(btns)
            self._paint_focus()

    def _activate(self):
        """SAVE pressed: tick the row, or press the focused button."""
        if self._zone == "list":
            self._toggle_current()
        else:
            btn = self._buttons()[self._btn_index]
            if btn.isEnabled():
                btn.click()

    # -- actions --------------------------------------------------------------

    def _confirm(self, title, text):
        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(title)
        box.setInformativeText(text)
        box.setIcon(QMessageBox.Warning)
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        box.setDefaultButton(QMessageBox.No)     # never delete on a stray Enter
        box.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        # Qt draws QMessageBox from the app palette, which is light everywhere
        # else in this process - inside a near-black window that lands as a
        # white slab. Dressed by hand rather than by switching the whole app to
        # a dark palette, which would repaint every panel behind it.
        box.setStyleSheet(
            "QMessageBox { background: %s; }"
            "QLabel { color: %s; }"
            "QPushButton { background: rgba(255,255,255,0.12); color: %s;"
            "   border: none; border-radius: 7px; padding: 8px 20px;"
            "   min-width: 84px; font-weight: 600; }"
            "QPushButton:default { background: %s; color: #ffffff; }"
            % (WIN, TEXT, TEXT, BAD))
        return box.exec() == QMessageBox.Yes

    def _start(self, jobs, delete):
        if not jobs:
            return
        self._popup = ProgressPopup(
            "Deleting from the Pi" if delete else "Saving to USB drive", self)
        self._popup.update_progress("", 0, len(jobs))
        self._popup.show()
        self._worker = _Worker(jobs, dest=self.mount, delete=delete, parent=self)
        self._worker.progressed.connect(self._on_progress)
        self._worker.finished_ok.connect(self._on_done)
        self._worker.failed.connect(self._on_failed)
        self._worker.start()
        self._sync_buttons()

    def _save(self):
        jobs = self._selected()
        total = sum(j[2] for j in jobs)
        self.hint.setText("Saving %d session%s (%s) to the USB drive…"
                          % (len(jobs), "" if len(jobs) == 1 else "s", _human(total)))
        self._start(jobs, delete=False)

    def _delete(self):
        jobs = self._selected()
        if not self._confirm("Delete from the Pi",
                             "Delete %d session%s from the Pi?\n\n"
                             "This cannot be undone."
                             % (len(jobs), "" if len(jobs) == 1 else "s")):
            return
        self._start(jobs, delete=True)

    def _delete_all(self):
        jobs = [r for r in self._rows if r[4] == "ready"]
        held = len(self._rows) - len(jobs)
        if not jobs:
            self.hint.setText(
                "Nothing to delete - every recording is still processing.")
            return
        extra = ("\n\n%d still being processed and will be left alone."
                 % held) if held else ""
        if not self._confirm("Delete everything",
                             "Delete ALL %d recordings from the Pi?\n\n"
                             "This cannot be undone.%s"
                             % (len(jobs), extra)):
            return
        self._start(jobs, delete=True)

    # -- worker callbacks -----------------------------------------------------

    def _on_progress(self, name, done, total):
        if self._popup is not None:
            self._popup.update_progress(name, done, total)

    def _close_popup(self):
        """Shut the little box. Called on BOTH endings - a popup left up after a
        failure would have the operator waiting on a job that already stopped."""
        if self._popup is not None:
            self._popup.accept()
            self._popup = None

    def _on_done(self, summary):
        self._close_popup()
        self.hint.setText(summary)
        self._worker = None
        self.reload()

    def _on_failed(self, why):
        self._close_popup()
        self.hint.setText("Failed: %s" % why)
        self._worker = None
        self.reload()

    # -- lifecycle ------------------------------------------------------------

    def closeEvent(self, event):
        # A copy in flight owns files on both sides. Let it finish rather than
        # leaving half a session on the stick with nothing to say so.
        if self._worker is not None and self._worker.isRunning():
            self._worker.wait(15000)
        super().closeEvent(event)
