"""UTPP red4ext log reading, per-step extraction and metrics (read-only, stdlib only).

Trace semantics (MotionHost.cpp): an `MMTRACE tick` summary line is written about once per second and an
`MMTRACE TRANS` line on every transition, both only while UTPPSetMMTrace(true) is active. All "time" metrics are
therefore sample-and-hold estimates on a ~1 Hz grid plus transition events; short spikes between samples are not seen.
The log is written from several threads, so lines are sorted by timestamp before use.

MM v2 (default driver since 2026-10-20, cet/utpp_motion/init.lua mm.motionDriver=true): the real selection is in the
'UTPP MOTION t= db=<i>:<name> clip=<i>:<name> ct= cost= ... ov=<overlay>/<aim>/<crouch><moving> ...' line
(MotionDriver.cpp TraceLocked: written on every db / clip / mode / overlay key change, else every 0.5 s of feed time; kind
'MOTION'). Under v2 the v1 matcher does not drive, so the v1 fields of the MMTRACE tick (clip, f, cost, rate, off,
searches, trans) and the MMTRACE TRANS lines are frozen placeholders: the v1-only metrics are reported as not applicable
(None + 'not_applicable'), and clip / class / cost / transition metrics come from the MOTION lines. The tick fields that do
not come from the v1 matcher keep working under v2 and are still used: loco (game PSM), L= / R= (foot lock), |v|, cam, pl,
bt / btCmds / err.

Gait quality (G6, 2026-10-31, report-only): speed_by_mv (|v| median / p90 per |mv| bucket), root_off_* (|off| of the tick; `off`
was observed live under v2 in the logs of 2026-10-23 and 2026-10-31 - it moves and saturates at 30 cm - so it is reported
under its own names while the v1-only offset_* metrics stay as they are) and the motion_db_walk_* /
motion_clip_transition_fraction / motion_walk_* time shares and motion_steady_* re-selection rates (see _gait_motion).

Step windows never carry the previous step's last sample: a window starts without a sample until its first in-window
line (2026-10-23 fix; the old prior-sample hold produced false fails such as sprint_stop offset_saturated_s 0.672 with
an in-window offset of 0.5 cm).
"""
from __future__ import annotations

import glob
import math
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timedelta

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utpp_paths import game_dir  # noqa: E402
DEFAULT_LOG_DIR = os.path.join(game_dir(required=False) or '', 'red4ext', 'logs')

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'recon'))
try:  # keep the compact format in sync with Source\tools\recon\mmtrace_summary.py
    from mmtrace_summary import KEYS as SUMMARY_KEYS  # type: ignore
except Exception:  # pragma: no cover
    SUMMARY_KEYS = ['clip', 'f', 'cost', '|v|', 'rate', 'loco', 'mv', 'cam', 'pl', 'off', 'L', 'R', 'bt', 'tgt']
COMPACT_KEYS = list(SUMMARY_KEYS) + [k for k in ('btCmds', 'err', 'gate') if k not in SUMMARY_KEYS]

TS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')
FIELD_RE = re.compile(r'(?:^|\s)([A-Za-z|]+)=(\S+)')
PAIR_RE = re.compile(r'\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)')
FOOT_RE = re.compile(r'^(-?\d+)/(-?[\d.]+)/(-?[\d.]+)cm$')

SATURATION_CM = 24.0
LOCO_STAND, LOCO_CROUCH = 1, 2   # MMTRACE loco= values (0 other, 1 stand, 2 crouch)

# MMTRACE lines that are NOT trace samples get their own kind (they must not take over the sample-and-hold window of the
# TRANS / tick line they follow): loco C0 'UTPP MMTRACE feet ...' (after every TRANS / tick line) and 'UTPP MMTRACE footstep
# ...' (per foot event), 0.5.8 'UTPP MMTRACE costcap ...' (cost cap events). Fields: the key=value pairs after the token.
SIDE_TRACE_KINDS = (('MMTRACE feet ', 'FEET'), ('MMTRACE footstep ', 'STEP'), ('MMTRACE costcap ', 'COSTCAP'))

# MM v2 trace line token ('UTPP MOTIONPOSE ...' has no space after MOTION and stays kind 'UTPP')
MOTION_TOKEN = 'UTPP MOTION '
# a step counts as driven by v2 when an MOTION line lies in the window or at most this long before it (heartbeat 0.5 s)
MOTION_LOOKBACK_S = 1.0
# the same clip under another database label is no clip change when its clip time advanced by the elapsed time within this
MOTION_RELABEL_CT_TOL_S = 0.05
# a tick with the left stick at least this far counts as pushed (pushed_speed_max, run.step_blocked)
PUSHED_STICK = 0.2   # aim walk scales mv to ~0.27 at stick 0.5-0.8 (Female V 17:37:56)
# the last tick before a window serves as the baseline of counter / yaw deltas (bodyturn_cmds, cam_yaw_*) when it is at
# most this old (tick period ~1 s); it is never a sample of the window
BASELINE_LOOKBACK_S = 1.5
# selection metrics per driver: a step with no effective threshold on any of them is reported 'selection_unscored'
SELECTION_METRICS = {'v2': ('motion_transitions_per_s', 'motion_cost_mean', 'motion_cost_max'),
                     'v1': ('transitions_per_s', 'cost_mean', 'cost_max')}
# metrics that only exist for the v1 matcher (MMTRACE TRANS lines and the tick fields cost / rate / off); under v2 they
# are placeholders and are reported as not applicable instead of being scored
BASE_ONLY_METRICS = ('trans_lines', 'transitions_per_s', 'cost_mean', 'cost_max', 'offset_saturated_s', 'offset_max_cm',
                   'rate_min', 'rate_mean', 'rate_max')


@dataclass
class LogLine:
    ts: datetime
    kind: str            # 'TRANS', 'tick', 'FEET', 'STEP', 'COSTCAP', 'GATE', 'AIR', 'MOTION', 'CET', 'UTPP'
    raw: str
    fields: dict = field(default_factory=dict)

    def num(self, key: str, default=None):
        v = self.fields.get(key)
        if v is None:
            return default
        try:
            return float(v)
        except ValueError:
            return default

    def pair(self, key: str):
        v = self.fields.get(key)
        m = PAIR_RE.search(v) if v else None
        return (float(m.group(1)), float(m.group(2))) if m else None

    def foot(self, key: str):
        v = self.fields.get(key)
        m = FOOT_RE.match(v) if v else None
        return (int(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None

    @property
    def is_trace(self) -> bool:
        return self.kind in ('TRANS', 'tick')

    def motion_name(self, key: str) -> str:
        """MOTION 'db=' / 'clip=' value without the '<index>:' prefix ('' when absent)."""
        v = self.fields.get(key, '')
        return v.split(':', 1)[1] if ':' in v else v

    def motion_ov(self):
        """MOTION 'ov=<overlay>/<aim>/<crouch><moving>' -> (overlay, aim, crouch, moving) ints, or None."""
        m = re.match(r'^(-?\d+)/(-?\d+)/([01])([01])$', self.fields.get('ov', ''))
        return tuple(int(g) for g in m.groups()) if m else None

    def compact(self) -> str:
        t = self.ts.strftime('%H:%M:%S.%f')[:12]
        if self.kind == 'MOTION':
            parts = [f'{k}={self.fields[k]}' for k in ('db', 'clip', 'ct', 'cost', 'blend', 'rot', 'ov', 'w')
                     if k in self.fields]
            return f'{t} MOTION  ' + ' '.join(parts).replace('M_Neutral_', '')
        if self.kind == 'GATE':
            return f'{t} GATE  ' + self.raw.split('MMGATE', 1)[1].strip()[:110]
        if self.kind == 'AIR':
            return f'{t} AIR   ' + self.raw.split('MMAIR', 1)[1].strip()[:140]
        if self.kind == 'CET':
            return f'{t} CET   ' + self.raw.split('UTPP CET', 1)[1].strip()[:140]
        for token, kind in SIDE_TRACE_KINDS:
            if self.kind == kind:
                return f'{t} {kind:<5} ' + self.raw.split(token, 1)[1].strip()[:140]
        if not self.is_trace:
            return f'{t} UTPP  ' + self.raw.split('[UTPP]', 1)[-1].strip()[:140]
        parts = []
        for k in COMPACT_KEYS:
            if k in self.fields:
                v = self.fields[k]
                if k == 'clip':
                    v = v.replace('M_Neutral_', '')
                parts.append(f'{k}={v}')
        return f'{t} {"TRANS" if self.kind == "TRANS" else "tick "} ' + ' '.join(parts)


def parse_line(line: str) -> LogLine | None:
    m = TS_RE.match(line)
    if not m or 'UTPP' not in line:
        return None
    ts = datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
    line = line.rstrip('\r\n')
    if 'MMTRACE' in line:
        for token, kind in SIDE_TRACE_KINDS:
            if token in line:
                return LogLine(ts, kind, line, dict(FIELD_RE.findall(line.split(token, 1)[1])))
        kind = 'TRANS' if 'MMTRACE TRANS' in line else 'tick'
        body = line.split('MMTRACE', 1)[1]
        return LogLine(ts, kind, line, dict(FIELD_RE.findall(body)))
    if 'MMGATE' in line:
        return LogLine(ts, 'GATE', line, dict(FIELD_RE.findall(line.split('MMGATE', 1)[1])))
    # AIR HOLD stage (2026-10-13): 'UTPP MMAIR phase=<new> was=<old> action= mode= detail= onG= ...' on air phase changes (own
    # kind, never a trace sample)
    if 'UTPP MMAIR ' in line:
        return LogLine(ts, 'AIR', line, dict(FIELD_RE.findall(line.split('UTPP MMAIR ', 1)[1])))
    if MOTION_TOKEN in line:
        return LogLine(ts, 'MOTION', line, dict(FIELD_RE.findall(line.split(MOTION_TOKEN, 1)[1])))
    if 'UTPP CET' in line:
        return LogLine(ts, 'CET', line)
    return LogLine(ts, 'UTPP', line)


def parse_text(text: str) -> list[LogLine]:
    out = [p for p in (parse_line(l) for l in text.splitlines()) if p is not None]
    out.sort(key=lambda l: l.ts)  # stable: same-ms lines keep file order
    return out


# ---- rotation-safe reading --------------------------------------------------------------------------------------------
# The red4ext logger rotates the UTPP log at 10 MB (seen 2026-11-01): the full file utpp-<start>.log is renamed to
# utpp-<start>.1.log (an older .1 becomes .2, ...) and a new utpp-<start>.log starts. A log "session" is the current file
# plus its rotated siblings. The functions below read that session as ONE stream, oldest content first, so a marker or a
# position taken before a rotation stays valid after it. A position is a LogMark (or the int returned by log_size).
_LOG_NAME_RE = re.compile(r'^(?P<base>utpp-.+?)(?:\.(?P<idx>\d+))?\.log$')
_FIRST_TS_RE = re.compile(rb'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]', re.M)
_HEAD_BYTES = 262144
_MARKS: dict = {}          # (normcase path, virtual offset) -> LogMark taken by this process (see log_size)
_MARKS_MAX = 4096


def _split_log_name(path: str):
    m = _LOG_NAME_RE.match(os.path.basename(path))
    return (os.path.dirname(path), m.group('base'), m.group('idx')) if m else None


def is_session_log(path: str | None) -> bool:
    """True for the CURRENT utpp-<start>.log (the file name that stays the same across rotations)."""
    if not path:
        return False
    sp = _split_log_name(path)
    return sp is not None and sp[2] is None


def _open_shared(path: str):
    """Open a log for reading WITHOUT blocking the game's rotation. Python's open() shares READ|WRITE but not DELETE on
    Windows, so a handle held during the logger's rename/delete makes spdlog's rotation fail and it truncates the log it
    is writing. This handle shares READ|WRITE|DELETE (a rename/delete of the file goes through, our handle follows the
    file). Other platforms: plain open."""
    if os.name != 'nt':
        return open(path, 'rb')
    import ctypes
    import msvcrt
    from ctypes import wintypes
    k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    k32.CreateFileW.restype = ctypes.c_void_p
    k32.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
                                wintypes.DWORD, ctypes.c_void_p]
    h = k32.CreateFileW(path, 0x80000000, 0x7, None, 3, 0x80, None)   # GENERIC_READ, share R|W|D, OPEN_EXISTING, NORMAL
    if h is None or h == ctypes.c_void_p(-1).value:
        raise ctypes.WinError(ctypes.get_last_error())
    return os.fdopen(msvcrt.open_osfhandle(h, os.O_RDONLY), 'rb')


def _ident_size(fh) -> tuple:
    """(identity, size) of an open log from the SAME handle: identity = timestamp of the first line, size = byte length."""
    fh.seek(0)
    m = _FIRST_TS_RE.search(fh.read(_HEAD_BYTES))
    fh.seek(0, os.SEEK_END)
    return (m.group(1).decode('ascii') if m else None), fh.tell()


def _first_ts(path: str) -> str | None:
    """Identity of a log file: the timestamp of its first line (a rename keeps it, a rotation cannot repeat it)."""
    try:
        with _open_shared(path) as fh:
            return _ident_size(fh)[0]
    except OSError:
        return None


def _list_session(path: str) -> list[str]:
    d, base, _ = _split_log_name(path)
    found = []
    try:
        names = os.listdir(d or '.')
    except OSError:
        return []
    for n in names:
        m = _LOG_NAME_RE.match(n)
        if m and m.group('base') == base:
            found.append((os.path.join(d, n), int(m.group('idx') or 0)))
    # content time first (robust while a rotation renames .1 -> .2 -> ... in several steps); a file without a timestamp
    # (empty) sorts by its rotation index (the unsuffixed current file is the newest)
    found.sort(key=lambda t: (_first_ts(t[0]) or '9999', -t[1]))
    return [t[0] for t in found]


def session_files(path: str) -> list[str]:
    """Files of the log session of `path`, oldest content first, current file last. A rotated name (.N.log) or any other
    name is just that file. The listing is repeated when a rotation changes it while it is read."""
    if not is_session_log(path):
        return [path] if os.path.isfile(path) else []
    files = _list_session(path)
    for _ in range(3):
        again = _list_session(path)
        if again == files:
            break
        files = again
    return files


class LogMark:
    """A position in a log session. `virtual` = bytes of all session files before it (sum of sizes); `ident` / `local` =
    the file (identified by its first-line timestamp) and the byte offset inside it. The identity form survives a rotation
    (rename) and the deletion of the oldest file; the virtual form is the fallback for a bare integer or an empty session."""

    def __init__(self, virtual: int = 0, ident: str | None = None, local: int = 0):
        self.virtual, self.ident, self.local = virtual, ident, local

    def token(self) -> str:
        """Opaque string form for another process (PowerShell): '<ident>@<local>|<virtual>'."""
        return f'{self.ident or ""}@{self.local}|{self.virtual}'

    @staticmethod
    def parse(v) -> 'LogMark':
        if isinstance(v, LogMark):
            return v
        if isinstance(v, int):
            return LogMark(max(0, v))
        s = str(v).strip()
        if '|' in s and '@' in s:
            head, virt = s.rsplit('|', 1)
            ident, local = head.rsplit('@', 1)
            return LogMark(int(virt), ident or None, int(local))
        return LogMark(max(0, int(s)))


def _open_session(path: str) -> list:
    """Snapshot of the session: [(handle, identity, size)] oldest content first. The handles stay valid while the logger
    renames the files, so identity, size and data of one file always come from the same handle."""
    out = []
    for name in session_files(path):
        try:
            fh = _open_shared(name)
        except OSError:
            continue
        ident, size = _ident_size(fh)
        out.append((fh, ident, size))
    return out


def _close_all(snap: list) -> None:
    for fh, _, _ in snap:
        fh.close()


def log_mark(path: str | None) -> LogMark | None:
    """Mark at the current end of the session of `path` (None when there is no log)."""
    if not path:
        return None
    snap = _open_session(path)
    try:
        if not snap:
            return None
        ident, local = None, 0
        for _, i, sz in reversed(snap):
            if sz > 0:
                ident, local = i, sz
                break
        return LogMark(sum(sz for _, _, sz in snap), ident, local)
    finally:
        _close_all(snap)


def log_size(path: str | None) -> int:
    """Rotation-safe stand-in for os.path.getsize of a session log: the virtual end offset (-1 = no log). The mark with
    the file identity is remembered in this process, so read_bytes_since / read_text_since / parse_file resolve it exactly."""
    m = log_mark(path)
    if m is None:
        return -1
    if len(_MARKS) >= _MARKS_MAX:
        _MARKS.clear()
    _MARKS[(os.path.normcase(os.path.abspath(path)), m.virtual)] = m
    return m.virtual


def read_bytes_since(path: str | None, pos=0) -> bytes:
    """Session bytes from `pos` (int from log_size, LogMark or token string) to the end, across rotated siblings.
    A mark whose file is gone reads every surviving file that started after it, from its start. A position behind the end reads
    nothing; there is no 'restart at 0' (over a joined session that would re-read old lines)."""
    if not path:
        return b''
    if isinstance(pos, int) and not isinstance(pos, bool):
        pos = _MARKS.get((os.path.normcase(os.path.abspath(path)), pos), pos)
    mark = LogMark.parse(pos)
    snap = _open_session(path)
    try:
        if not snap:
            return b''
        first, local = None, 0
        if mark.ident is not None:
            for i, (_, ident, _) in enumerate(snap):
                if ident == mark.ident:
                    first, local = i, mark.local
                    break
            if first is None:
                # marked file gone: deleted oldest, or truncated in place after a failed rename (then the older siblings survive
                # and predate the mark). Only files that started after the mark; the identity is a sortable timestamp string.
                out = []
                for fh, ident, _ in snap:
                    if ident is None or ident > mark.ident:
                        fh.seek(0)
                        out.append(fh.read())
                return b''.join(out)
        else:
            left = mark.virtual
            for i, (_, _, sz) in enumerate(snap):
                if left < sz or (left == sz and i == len(snap) - 1):
                    first, local = i, left
                    break
                left -= sz
            if first is None:
                return b''
        out = []
        for i in range(first, len(snap)):
            fh = snap[i][0]
            fh.seek(local if i == first else 0)
            out.append(fh.read())
        return b''.join(out)
    finally:
        _close_all(snap)


def read_text_since(path: str | None, pos=0) -> str:
    return read_bytes_since(path, pos).decode('utf-8', errors='replace')


def read_session_text(path: str | None, since=None, tail_bytes: int | None = None) -> str:
    """Session text from the mark `since` (default: the start), optionally only the last `tail_bytes` bytes."""
    data = read_bytes_since(path, since if since is not None else 0)
    if tail_bytes is not None and len(data) > tail_bytes:
        data = data[-tail_bytes:]
    return data.decode('utf-8', errors='replace')


def parse_file(path: str, start_offset=0) -> list[LogLine]:
    """Lines of the log from `start_offset` (int / LogMark / token). A session log (utpp-<start>.log) includes the
    rotated siblings of the same session."""
    if is_session_log(path):
        data = read_bytes_since(path, start_offset)
    else:
        with open(path, 'rb') as f:
            f.seek(LogMark.parse(start_offset).virtual)
            data = f.read()
    return parse_text(data.decode('utf-8', errors='replace'))


# Pose owner of a session (which stage of the frame wrote the player's pose). The plugin selects it once at load: the load line below
# appears only when the switch file selected the early mapping owner; the first "FK counters (tpp)" lines carry earlyOwner (0 = FK,
# 1 = Mapping). Two runs on different owners are not comparable.
_OWNER_LINE_RE = re.compile(r'UTPP early pose experiment: owner=(\w+) catalogValid=(\d+) helpers=(\d+)')
_FK_OWNER_RE = re.compile(r'UTPP FK counters \(tpp\):.*?\searlyOwner=(\d+)')


def pose_owner(text: str) -> dict:
    """{'owner': 'Mapping' | 'FK' | 'unknown' | 'conflict', 'owner_line', 'early_owner', 'catalog_valid', 'helpers', 'detail'}.
    Mapping = the load line (owner=Mapping) and no FK counters line that says 0; FK = an FK counters line with earlyOwner=0 and no
    load line; unknown = neither line (log cut or the body never reached third person); conflict = the two sources disagree."""
    m = _OWNER_LINE_RE.search(text)
    early = sorted({int(x) for x in _FK_OWNER_RE.findall(text)})
    out = {'owner': 'unknown', 'owner_line': bool(m), 'early_owner': early,
           'catalog_valid': int(m.group(2)) if m else None, 'helpers': int(m.group(3)) if m else None, 'detail': ''}
    if m and m.group(1) != 'Mapping':
        out.update(owner='conflict', detail=f'load line names owner={m.group(1)}')
    elif m and early and early != [1]:
        out.update(owner='conflict', detail='load line says Mapping, FK counters say earlyOwner=%s' % '/'.join(map(str, early)))
    elif m:
        out['owner'] = 'Mapping'
        if not out['catalog_valid']:
            out['detail'] = 'catalogValid=0: the early owner writes no pose at all'
    elif early == [0]:
        out['owner'] = 'FK'
    elif early:
        out.update(owner='conflict', detail='FK counters say earlyOwner=%s without the load line' % '/'.join(map(str, early)))
    else:
        out['detail'] = 'neither the load line nor an FK counters line in the log'
    return out


def pose_owner_of_log(path: str | None) -> dict:
    """pose_owner of a session log (a utpp-<start>.log includes its rotated siblings); a missing file = unknown."""
    if not path or not os.path.isfile(path):
        return dict(pose_owner(''), detail='no mod log')
    return pose_owner(read_session_text(path))


def pose_owner_label(po: dict | None) -> str:
    """Short text for the one-line results: 'FK', 'Mapping', 'Mapping (catalogValid=0 ...)', 'unknown', 'CONFLICT (...)'."""
    if not po:
        return 'unknown'
    owner = po.get('owner') or 'unknown'
    if owner == 'conflict':
        return f'CONFLICT ({po.get("detail")})'
    return owner + (f' ({po["detail"]})' if owner == 'Mapping' and po.get('detail') else '')


def newest_log(log_dir: str = DEFAULT_LOG_DIR, pattern: str = 'utpp-*.log') -> str | None:
    """The CURRENT file (utpp-<start>.log, unsuffixed) of the session written last. A rotated sibling (.1.log, .2.log)
    never wins on its own: the session is ranked by its newest member and answered by its current file."""
    sessions: dict = {}
    for p in glob.glob(os.path.join(log_dir, pattern)):
        sp = _split_log_name(p)
        key = (sp[0], sp[1]) if sp else (None, p)
        try:
            t = os.path.getmtime(p)
        except OSError:
            continue
        cur = sessions.get(key, (-1.0, None, p))
        sessions[key] = (max(cur[0], t), p if (sp is None or sp[2] is None) else cur[1], p)
    if not sessions:
        return None
    t, current, anyfile = max(sessions.values(), key=lambda v: v[0])
    return current or anyfile


# ---------------------------------------------------------------------------------------------- metrics

CLIP_CLASSES = ('Transition', 'Reface', 'Pivot', 'Start', 'Stop', 'Turn', 'Idle', 'Loop', 'Other')


def clip_class(clip: str) -> str:
    c = clip
    if 'Transition_' in c:
        return 'Transition'
    if 'Reface' in c:
        return 'Reface'
    if '_Pivot' in c or 'Pivot_' in c:
        return 'Pivot'
    if '_Start' in c:
        return 'Start'
    if '_Stop' in c:
        return 'Stop'
    if '_Turn_' in c or 'Turn_' in c:
        return 'Turn'
    if 'Idle' in c:
        return 'Idle'
    if '_Loop' in c:
        return 'Loop'
    return 'Other'


def _wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _held(samples: list[LogLine], t0: datetime, t1: datetime):
    """Sample-and-hold segments (line, seconds) inside [t0, t1]. The window has no sample before its first in-window
    line (no carry-over of the previous step's last sample), so the segments may cover less than the window."""
    segs = []
    cur, cur_t = None, t0
    for s in samples:
        if cur is not None:
            segs.append((cur, (s.ts - cur_t).total_seconds()))
        cur, cur_t = s, s.ts
    if cur is not None:
        segs.append((cur, (t1 - cur_t).total_seconds()))
    return [(l, d) for l, d in segs if d > 0]


def _stats(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None, None
    return min(vals), sum(vals) / len(vals), max(vals)


def _r(v, nd=3):
    return None if v is None else round(v, nd)


def step_lines(lines: list[LogLine], t0: datetime, t1: datetime) -> list[LogLine]:
    return [l for l in lines if t0 <= l.ts <= t1]


def base_trace_frozen(trace: list[LogLine]):
    """Placeholder detection for the v1 MMTRACE fields: True when every tick in the list reports searches=0 and trans=0
    with one unchanged (clip, f) (the v1 matcher is not driving, e.g. under MM v2), False when they move, None when the
    lines carry no searches= field or there is no tick."""
    ticks = [l for l in trace if l.kind == 'tick' and l.num('searches') is not None]
    if not ticks:
        return None
    return (all(l.num('searches') == 0 and (l.num('trans') or 0) == 0 for l in ticks)
            and len({(l.fields.get('clip'), l.fields.get('f')) for l in ticks}) == 1)


def _base_metrics(m: dict, trace: list[LogLine], segs, dur: float, saturation_cm: float) -> None:
    trans = [l for l in trace if l.kind == 'TRANS']
    m['trans_lines'] = len(trans)
    m['transitions_per_s'] = round(len(trans) / dur, 3)
    costs = [l.num('cost') for l in trans]
    if not costs:
        costs = [c for c in (l.num('cost') for l in trace) if c]
    _, cmean, cmax = _stats(costs)
    m['cost_mean'], m['cost_max'] = _r(cmean), _r(cmax)

    classes: dict = {}
    clips: list = []
    for l in trace:   # clips seen in the window (first in-window sample first, no clip carried from before t0)
        c = l.fields.get('clip', '')
        if c:
            if l.kind == 'TRANS':
                classes[clip_class(c)] = classes.get(clip_class(c), 0) + 1
            if c not in clips:
                clips.append(c)
    m['trans_classes'] = classes
    m['clips'] = [c.replace('M_Neutral_', '') for c in clips]
    m['clip_classes_played'] = sorted({clip_class(c) for c in clips})

    sat = 0.0
    offs = []
    for l, d in segs:
        o = l.pair('off')
        if o is not None and math.hypot(*o) >= saturation_cm:
            sat += d
    for l in trace:
        o = l.pair('off')
        if o is not None:
            offs.append(math.hypot(*o))
    m['offset_saturated_s'] = round(sat, 3)
    m['offset_max_cm'] = _r(max(offs) if offs else None, 2)
    rmin, rmean, rmax = _stats([l.num('rate') for l in trace])
    m['rate_min'], m['rate_mean'], m['rate_max'] = _r(rmin), _r(rmean), _r(rmax)


def _motion_metrics(m: dict, v2: list[LogLine], ref: LogLine | None, t0: datetime, t1: datetime, dur: float) -> None:
    """Selection metrics from the MOTION lines in the window. `ref` (the last MOTION line at most MOTION_LOOKBACK_S before t0)
    is only the reference for detecting a clip change on the first in-window line; it adds no time and no clip."""
    m['motion_samples'] = len(v2)
    # A change is a new clip or a jump inside the same clip. The same clip continuing with its clip time advancing by the
    # elapsed time under another database label (Run_Starts -> Run_SpinTransition, 2026-11-11 16:06:55: ct 0.167 -> 0.339
    # in 0.184 s) shows no animation change and is not counted.
    prev = ref
    changes = []
    for l in v2:
        if prev is not None and (l.fields.get('db'), l.fields.get('clip')) != (prev.fields.get('db'), prev.fields.get('clip')):
            relabel = False
            if l.fields.get('clip') == prev.fields.get('clip'):
                ct0, ct1 = prev.num('ct'), l.num('ct')
                ta, tb = prev.num('t'), l.num('t')
                dt = (tb - ta) if ta is not None and tb is not None else (l.ts - prev.ts).total_seconds()
                relabel = ct0 is not None and ct1 is not None and abs((ct1 - ct0) - dt) <= MOTION_RELABEL_CT_TOL_S
            if not relabel:
                changes.append(l)
        prev = l
    m['motion_changes'] = len(changes)
    m['motion_transitions_per_s'] = round(len(changes) / dur, 3)
    # cost=-1.000 is the exact 'no match' sentinel (MotionDriver.cpp TraceLocked: r.cost < 1e30f ? r.cost : -1.0f, '%.3f');
    # other negative costs (e.g. -0.200 continuing-pose bias) are real and kept
    costs = [c for c in (l.num('cost') for l in v2) if c is not None and abs(c + 1.0) > 1e-6]
    _, cmean, cmax = _stats(costs)
    m['motion_cost_mean'], m['motion_cost_max'] = _r(cmean), _r(cmax)
    classes: dict = {}
    for l in changes:
        c = l.motion_name('clip')
        classes[clip_class(c)] = classes.get(clip_class(c), 0) + 1
    m['trans_classes'] = classes
    clips: list = []
    for l in v2:
        c = l.motion_name('clip')
        if c and c not in clips:
            clips.append(c)
    m['clips'] = [c.replace('M_Neutral_', '') for c in clips]
    m['clip_classes_played'] = sorted({clip_class(c) for c in clips})
    m['motion_dbs'] = list(dict.fromkeys(l.motion_name('db') for l in v2 if l.motion_name('db')))
    oseg = [(l.motion_ov(), d) for l, d in _held(v2, t0, t1) if l.motion_ov() is not None]
    otot = sum(d for _, d in oseg)
    m['motion_crouch_fraction'] = _r(sum(d for o, d in oseg if o[2] == 1) / otot if otot > 0 else None)


# ---- gait quality metrics (G6, 2026-10-31); report-only, no default thresholds ----
GAIT_MV_MIN = 0.05            # |mv| above this counts as stick input (bucketed in GAIT_MV_STEP steps)
GAIT_MV_STEP = 0.05
# steady locomotion = an MOTION line with ov moving=1 whose db is a '...Loops' db (Starts / Stops / Idles / Pivots excluded)
STEADY_DB_TOKEN = 'Loop'
CT_DROP_TOL_S = 0.05          # a clip-time decrease larger than this inside one clip is a re-selection candidate
CT_WRAP_RATE_MAX = 1.8        # ... a 'wrap' (natural loop restart) when the new ct <= feed dt * this + CT_WRAP_SLACK_S
CT_WRAP_SLACK_S = 0.02
WALK_LOOPS_DB, WALK_STARTS_DB = 'Walk_Loops', 'Walk_Starts'


def _median(vals: list):
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2


def _p90(vals: list):
    """Nearest-rank 90th percentile (the ceil(0.9 n)-th smallest value)."""
    if not vals:
        return None
    s = sorted(vals)
    return s[math.ceil(0.9 * len(s)) - 1]


def _motion_steady(l: LogLine) -> bool:
    o = l.motion_ov()
    return o is not None and o[3] == 1 and STEADY_DB_TOKEN in l.motion_name('db')


def _gait_common(m: dict, trace: list[LogLine]) -> None:
    """Metrics from the MMTRACE tick fields that stay live under v2 (|v|, mv, off); independent of the driver.
    `off` is the root offset of the MMTRACE tick. It was observed live under v2 (logs of 2026-10-23 and 2026-10-31: it
    moves and saturates at 30 cm); it is reported here under its own names, the v1-only offset_* metrics stay as they are.
      speed_by_mv: {'<|mv| bucket>': {'n', 'median', 'p90'}} of |v| (m/s) over the ticks with |mv| > GAIT_MV_MIN, bucket =
                   |mv| rounded to GAIT_MV_STEP (string key '1.00'), median / nearest-rank p90.
      root_off_median_cm / root_off_max_cm: |off| over every tick of the window; root_off_moving_median_cm: ticks with
                   |mv| > GAIT_MV_MIN only."""
    buckets: dict = {}
    offs_all, offs_mv = [], []
    for l in trace:
        if l.kind != 'tick':
            continue
        mv = l.pair('mv')
        mvm = math.hypot(*mv) if mv is not None else None
        v = l.num('|v|')
        if mvm is not None and mvm > GAIT_MV_MIN and v is not None:
            key = f'{round(mvm / GAIT_MV_STEP) * GAIT_MV_STEP:.2f}'
            buckets.setdefault(key, []).append(v)
        o = l.pair('off')
        if o is not None:
            offs_all.append(math.hypot(*o))
            if mvm is not None and mvm > GAIT_MV_MIN:
                offs_mv.append(math.hypot(*o))
    m['speed_by_mv'] = {k: {'n': len(v), 'median': _r(_median(v)), 'p90': _r(_p90(v))}
                        for k, v in sorted(buckets.items())}
    m['root_off_median_cm'] = _r(_median(offs_all), 2)
    m['root_off_max_cm'] = _r(max(offs_all) if offs_all else None, 2)
    m['root_off_moving_median_cm'] = _r(_median(offs_mv), 2)


GAIT_MOTION_KEYS = ('motion_db_walk_loops_fraction', 'motion_db_walk_starts_fraction', 'motion_clip_transition_fraction',
                'motion_walk_loop_clip_fraction', 'motion_walk_nonloop_fraction', 'motion_walk_loop_share', 'motion_steady_s',
                'motion_steady_switches', 'motion_steady_ct_wraps', 'motion_steady_ct_jumps', 'motion_steady_reselect_per_s',
                'motion_steady_reselect_nowrap_per_s')


def _gait_motion(m: dict, v2: list[LogLine], t0: datetime, t1: datetime) -> None:
    """Selection quality of walking from the MOTION lines in the window (sample-and-hold time, like motion_crouch_fraction).
    Time shares are fractions of the held MOTION time of the window:
      motion_db_walk_loops_fraction / motion_db_walk_starts_fraction: db name contains Walk_Loops / Walk_Starts (any clip);
      motion_clip_transition_fraction: clip class Transition in any db (the db axis and the clip axis are different: the
          2026-10-31 log plays Transition_Run_to_Walk_Rfoot inside PSD_Dense_Stand_Walk_Loops);
      motion_walk_loop_clip_fraction: Walk_Loops db AND a non-Transition clip (a real walk loop);
      motion_walk_nonloop_fraction: Walk_Starts db OR a Transition clip; motion_walk_loop_share: loop / (loop + nonloop).
    Re-selection in steady locomotion (motion_steady_*): steady = _motion_steady line (moving, '...Loops' db). Between two
    consecutive in-window steady MOTION lines it counts a switch (db / clip changed), a ct 'wrap' (ct dropped by more
    than CT_DROP_TOL_S inside one clip to a value reachable by a natural loop restart, <= feed dt * CT_WRAP_RATE_MAX +
    slack) or a ct 'jump' (any other drop). motion_steady_reselect_per_s = (switches + wraps + jumps) / steady seconds,
    the _nowrap_ variant leaves the wraps out. All None when there is no steady time.
    Caveats: 'steady' is any moving '...Loops' db (walk, run, sprint alike), not walk only. The wrap/jump split is a
    heuristic on the log only: a ct drop to a value above feed dt * rate can still be a designed loop-back (the 2026-10-31
    walk log returns to ct ~1.75 of clip 508 on every cycle), and a drop from mid-clip to a small ct (a re-selection near the clip start) is
    counted as a wrap, so neither variant is an exact count of matcher jumps; the DB has per-clip kClipLoop / per-frame kFrameLoopDuplicate flags that the log does not carry."""
    segs = _held(v2, t0, t1)
    tot = sum(d for _, d in segs)
    if tot <= 0:
        for k in GAIT_MOTION_KEYS:
            m[k] = None
        return

    def frac(pred):
        return _r(sum(d for l, d in segs if pred(l)) / tot)

    def is_trans(l):
        return clip_class(l.motion_name('clip')) == 'Transition'

    def is_loopclip(l):
        return WALK_LOOPS_DB in l.motion_name('db') and not is_trans(l)

    def is_nonloop(l):
        return WALK_STARTS_DB in l.motion_name('db') or is_trans(l)

    m['motion_db_walk_loops_fraction'] = frac(lambda l: WALK_LOOPS_DB in l.motion_name('db'))
    m['motion_db_walk_starts_fraction'] = frac(lambda l: WALK_STARTS_DB in l.motion_name('db'))
    m['motion_clip_transition_fraction'] = frac(is_trans)
    m['motion_walk_loop_clip_fraction'] = frac(is_loopclip)
    m['motion_walk_nonloop_fraction'] = frac(is_nonloop)
    loop_t = sum(d for l, d in segs if is_loopclip(l))
    non_t = sum(d for l, d in segs if is_nonloop(l))
    m['motion_walk_loop_share'] = _r(loop_t / (loop_t + non_t) if loop_t + non_t > 0 else None)

    steady_s = sum(d for l, d in segs if _motion_steady(l))
    sw = wraps = jumps = 0
    for a, b in zip(v2, v2[1:]):
        if not (_motion_steady(a) and _motion_steady(b)):
            continue
        if (a.fields.get('db'), a.fields.get('clip')) != (b.fields.get('db'), b.fields.get('clip')):
            sw += 1
            continue
        ca, cb = a.num('ct'), b.num('ct')
        if ca is None or cb is None or cb >= ca - CT_DROP_TOL_S:
            continue
        ta, tb = a.num('t'), b.num('t')
        dt = (tb - ta) if ta is not None and tb is not None else (b.ts - a.ts).total_seconds()
        if cb <= dt * CT_WRAP_RATE_MAX + CT_WRAP_SLACK_S:
            wraps += 1
        else:
            jumps += 1
    m['motion_steady_s'] = round(steady_s, 3)
    m['motion_steady_switches'], m['motion_steady_ct_wraps'], m['motion_steady_ct_jumps'] = sw, wraps, jumps
    m['motion_steady_reselect_per_s'] = _r((sw + wraps + jumps) / steady_s) if steady_s > 0 else None
    m['motion_steady_reselect_nowrap_per_s'] = _r((sw + jumps) / steady_s) if steady_s > 0 else None


def compute_metrics(lines: list[LogLine], t0: datetime, t1: datetime, saturation_cm: float = SATURATION_CM) -> dict:
    """Metrics for the window [t0, t1] (lines must be sorted). 'mm_driver' says which matcher the step was scored
    against: 'v2' (MOTION lines present; v1-only metrics are None and listed in 'not_applicable'), 'v1' (legacy log
    without MOTION lines) or None (no trace at all)."""
    dur = max(1e-6, (t1 - t0).total_seconds())
    window = step_lines(lines, t0, t1)
    trace = [l for l in window if l.is_trace]
    v2 = [l for l in window if l.kind == 'MOTION']
    lo = t0 - timedelta(seconds=MOTION_LOOKBACK_S)
    ref = None
    prior = None    # last trace sample at most BASELINE_LOOKBACK_S before t0: counter / yaw delta baseline only
    log_has_motion = False
    for l in lines:
        if l.kind == 'MOTION':
            log_has_motion = True
        if l.ts >= t0:
            if log_has_motion:
                break
            continue
        if l.kind == 'MOTION' and l.ts >= lo:
            ref = l
        if l.is_trace:
            prior = l
    if prior is not None and prior.ts < t0 - timedelta(seconds=BASELINE_LOOKBACK_S):
        prior = None
    driver = 'v2' if (v2 or ref is not None) else ('v1' if trace else None)
    driver_note = None
    if driver == 'v1' and log_has_motion and base_trace_frozen(trace):
        # the log is a v2 log and the v1 tick is a frozen placeholder: never score this step as v1
        driver = 'v2'
        driver_note = 'no MOTION lines in window (log has MOTION lines, base tick frozen)'
    m: dict = {'duration_s': round(dur, 3), 'mm_driver': driver, 'mm_driver_note': driver_note, 'samples': len(trace),
               'gate_lines': sum(1 for l in window if l.kind == 'GATE'),
               'cet_lines': sum(1 for l in window if l.kind == 'CET')}

    segs = _held(trace, t0, t1)
    covered = sum(d for _, d in segs)
    m['trace_coverage'] = round(covered / dur, 3)

    if driver == 'v2':
        for k in BASE_ONLY_METRICS:
            m[k] = None
        m['not_applicable'] = list(BASE_ONLY_METRICS)
        m['base_trace_frozen'] = base_trace_frozen(trace)
        _motion_metrics(m, v2, ref, t0, t1, dur)
        _gait_motion(m, v2, t0, t1)
    else:
        m['not_applicable'] = []
        for k in ('motion_samples', 'motion_changes', 'motion_transitions_per_s', 'motion_cost_mean', 'motion_cost_max',
                  'motion_crouch_fraction', 'base_trace_frozen'):
            m[k] = None
        m['motion_dbs'] = []
        for k in GAIT_MOTION_KEYS:
            m[k] = None
        _base_metrics(m, trace, segs, dur, saturation_cm)
    _gait_common(m, trace)

    corr = []
    for l in trace:
        for k in ('L', 'R'):
            f = l.foot(k)
            if f is not None:
                corr.append(abs(f[2]))
    m['footlock_corr_max_cm'] = _r(max(corr) if corr else None, 2)

    _, vmean, vmax = _stats([l.num('|v|') for l in trace])
    m['speed_mean'], m['speed_max'] = _r(vmean, 2), _r(vmax, 2)
    # capsule speed on the ticks where the left stick was pushed (|mv| >= PUSHED_STICK): a walk against an obstacle
    # keeps it near 0 (run.step_blocked); ticks after a release (stop steps) are left out
    pushed = []
    for l in trace:
        mv, v = l.pair('mv'), l.num('|v|')
        if mv is not None and v is not None and math.hypot(*mv) >= PUSHED_STICK:
            pushed.append(v)
    m['pushed_ticks'] = len(pushed)
    m['pushed_speed_max'] = _r(max(pushed), 2) if pushed else None

    bt_time = sum(d for l, d in segs if (l.num('bt') or 0) > 0)
    m['bodyturn_active_s'] = round(bt_time, 3)
    bt_vals = [l.num('btCmds') for l in trace if l.num('btCmds') is not None]
    # counter delta up to the last in-window sample; the last sample before the window (<= BASELINE_LOOKBACK_S) is only
    # the baseline of the delta (it adds no time and is not a sample), else the first in-window sample
    base = prior.num('btCmds') if prior is not None and prior.num('btCmds') is not None else (bt_vals[0] if bt_vals else None)
    m['bodyturn_cmds'] = int(bt_vals[-1] - base) if bt_vals and base is not None else 0
    errs = [abs(l.num('err')) for l in trace if l.num('err') is not None and (l.num('bt') or 0) > 0]
    m['bodyturn_err_max_deg'] = _r(max(errs) if errs else None, 1)

    cams = [l.num('cam') for l in trace if l.num('cam') is not None]
    if cams and prior is not None and prior.num('cam') is not None:
        cams.insert(0, prior.num('cam'))   # yaw delta baseline only
    net = absum = 0.0
    for a, b in zip(cams, cams[1:]):
        d = _wrap180(b - a)
        net += d
        absum += abs(d)
    m['cam_yaw_net_deg'] = round(net, 1)
    m['cam_yaw_abs_deg'] = round(absum, 1)
    gaps = [abs(_wrap180(l.num('cam') - l.num('pl'))) for l in trace
            if l.num('cam') is not None and l.num('pl') is not None]
    m['cam_pl_gap_max_deg'] = _r(max(gaps) if gaps else None, 1)
    loco = [l.num('loco') for l in trace if l.num('loco') is not None]
    m['loco_fraction'] = _r(sum(1 for v in loco if v > 0) / len(loco) if loco else None)
    # share of the window (sample-and-hold) spent in a given loco state: 2 = crouch, 1 = stand (see MMDriver kLoco*)
    lseg = [(l.num('loco'), d) for l, d in segs if l.num('loco') is not None]
    ltot = sum(d for _, d in lseg)
    m['crouch_fraction'] = _r(sum(d for v, d in lseg if v == LOCO_CROUCH) / ltot if ltot > 0 else None)
    m['stand_fraction'] = _r(sum(d for v, d in lseg if v == LOCO_STAND) / ltot if ltot > 0 else None)
    return m


def motion_precondition(lines: list[LogLine], t0: datetime, t1: datetime) -> dict:
    """Scoring precondition: MM v2 is the active driver. Needs 'UTPP MOTION' lines inside the scored span [t0, t1]
    (they exist only while v2 drives and the trace is on; UTPPSetMMTrace(1) switches the v1 and v2 traces together).
    The time of the last 'UTPPSetMMTrace(1)' echo before t1 is reported for reference."""
    echo = None
    n = 0
    for l in lines:
        if l.ts > t1:
            break
        if 'UTPPSetMMTrace(1)' in l.raw:
            echo = l.ts
        if l.kind == 'MOTION' and l.ts >= t0:
            n += 1
    ok = n > 0
    reason = None if ok else ('mm.motionDriver not active: no "UTPP MOTION" lines in the scored span'
                              + (' after the CET UTPPSetMMTrace(1) echo' if echo else ' (no UTPPSetMMTrace(1) echo found)')
                              + ' - enable MM motion (CET mm.motionDriver) and the trace, or pass --allow-base for a legacy base log')
    return {'ok': ok, 'motion_lines': n, 'span': [t0, t1], 'trace_echo': echo, 'reason': reason}


# ---------------------------------------------------------------------------------------------- pass/fail

DEFAULT_THRESHOLDS = {
    'samples': {'min': 1},
    # MM v2 steps need at least one MOTION line (heartbeat 2 Hz); None (= skipped) for v1 logs
    'motion_samples': {'min': 1, 'msg': 'no MOTION lines in window'},
    # v1 matcher only (None under MM v2, see BASE_ONLY_METRICS):
    'transitions_per_s': {'max': 3.0},
    'cost_mean': {'max': 30.0},   # 0.3.8-0.4.1 TRANS cost: median 5.3, p90 33, p97 53, max 774
    'cost_max': {'max': 100.0},
    'offset_saturated_s': {'max': 0.5},
    'footlock_corr_max_cm': {'max': 15.0},
    'rate_min': None,                # report only since 2026-10-23 (contradicts the intended slow-walk rate 0.25)
    'rate_min@slow': {'min': 0.2},   # partial stick / ramps / stops: slow-walk rate floor (~0.25) is intended
    'rate_max': {'max': 1.8},
    # MM v2 selection: provisional loose limit (the v1 value) until a reviewed v2 baseline exists; costs report only
    'motion_transitions_per_s': {'max': 3.0},
    'motion_cost_mean': None,
    'motion_cost_max': None,
    'motion_crouch_fraction': None,
}
# keep in sync with thresholds.json (test_gametest checks that the file and these defaults agree)
SLOW_STICK = 0.6   # a step is 'slow' when its left-stick magnitude goes below this at any time


def merge_thresholds(base: dict, *overrides: dict) -> dict:
    """Later overrides win per key; None is kept as an explicit 'disabled' marker (see evaluate)."""
    out = {k: (None if v is None else dict(v)) for k, v in base.items()}
    for ov in overrides:
        for k, v in (ov or {}).items():
            out[k] = None if v is None else {**(out.get(k) or {}), **v}
    return out


def effective_thresholds(thresholds: dict, step: str = '', slow: bool = False) -> dict:
    """Per-metric limits that apply to one step (see evaluate for the precedence); None = disabled."""
    def layer(eff, metric, v):
        if v is None:
            eff[metric] = None
        else:
            eff[metric] = {**(eff.get(metric) or {}), **v}

    eff: dict = {}
    for want_step, want_slow in ((False, False), (False, True), (True, False), (True, True)):
        if want_slow and not slow:
            continue
        for k, v in thresholds.items():
            key, at = (k.split('@', 1) + [''])[:2]
            if (at == 'slow') != want_slow or (at and at != 'slow'):
                continue
            if ('.' in key) != want_step:
                continue
            if want_step:
                s, metric = key.split('.', 1)
                if s != step:
                    continue
            else:
                metric = key
            layer(eff, metric, v)
    return eff


def selection_unscored(metrics: dict, thresholds: dict, step: str = '', slow: bool = False) -> bool:
    """True when no threshold on a selection metric of the step's driver (SELECTION_METRICS) applied, i.e. a PASS says
    nothing about clip selection / jerks."""
    eff = effective_thresholds(thresholds, step, slow)
    keys = SELECTION_METRICS.get(metrics.get('mm_driver') or '', ())
    return not any(eff.get(k) and metrics.get(k) is not None for k in keys)


TRACE_PERIOD_S = 1.0   # the trace line is written about once a second


def trace_gap(metrics: dict) -> bool:
    """True when a window without a trace sample is shorter than the trace period while the motion lines show the driver ran in it:
    the window fell between two trace lines, which says nothing about the mod. Its sample-and-hold metrics are not measured then."""
    dur = metrics.get('duration_s')
    return (dur is not None and dur < TRACE_PERIOD_S and metrics.get('samples') == 0
            and (metrics.get('motion_samples') or 0) >= 1)


def evaluate(metrics: dict, thresholds: dict, step: str = '', slow: bool = False) -> list[str]:
    """Return failure messages.

    Key precedence (later wins, per metric): '<metric>', '<metric>@slow' (only when slow), '<step>.<metric>',
    '<step>.<metric>@slow' (only when slow). A null value disables the metric.
    """
    eff = effective_thresholds(thresholds, step, slow)
    fails = []
    for metric, lim in eff.items():
        if lim is None:
            continue
        val = metrics.get(metric)
        if val is None:
            continue
        if metric == 'samples' and trace_gap(metrics):
            continue
        # optional 'msg' in a limit gives the failure a clear prefix (e.g. "crouch not engaged")
        pre = f'{lim["msg"]}: ' if lim.get('msg') else ''
        if 'max' in lim and val > lim['max']:
            fails.append(f'{pre}{metric}={val} > {lim["max"]}')
        if 'min' in lim and val < lim['min']:
            fails.append(f'{pre}{metric}={val} < {lim["min"]}')
    return fails
