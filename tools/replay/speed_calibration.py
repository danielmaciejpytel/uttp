#!/usr/bin/env python3
"""Steady capsule speeds, leg / capsule speed ratios and a few locomotion event measurements from recorded game sessions.

Read only. Inputs are the game-test session folders (Research/logs/night/<run>/): the feed dump (logs/utpp-feed-*.txt, one row
per motion feed), the plugin log (logs/utpp-<stamp>.log with its rotated parts .N.log: MOTION trace lines and the harness POS
lines) and, when present,
summary.json (body) and gametest/*.txt (step windows). The motion databases (Data/utpp_motion_<body>.utppmm) give the clip root
paths.

    python -I speed_calibration.py --baseline <list.json> --json out.json
    python -I speed_calibration.py --runs <run folder> [<run folder> ...] --json out.json

The tool writes the JSON only; a report is made from it separately.

What is measured
  speeds      median steady capsule speed (|vx, vy| of the feed, m/s) per body x mode x gait x stick direction x stick
              magnitude. Mode: aim (PSM UpperBody 6), walk toggle (feed column walk), free. Gait: sprint (loco 3), crouch
              (loco 2), stand (loco 1, or any loco while aiming); stand is split by the stick magnitude (walk below 0.7, run
              from 0.7). Direction: sector of the raw stick angle atan2(mvx, mvy). A row is steady when its classification key
              and the stick have not changed for SETTLE_S, V is on the ground, the PSM detail is not a slide / air / traversal
              state, the speed changes by less than ACCEL_MAX and the velocity heading by less than YAW_RATE_MAX, and the
              position derivative agrees with the velocity columns.
  ratios      play rate of the motion clip = clip time advance / trace time between two consecutive MOTION lines of the same
              clip (the second written on the trace period, so nothing changed in between), clip root path travelled in that
              interval / interval = leg speed; ratio = leg speed / mean capsule speed of the feed rows of the interval.
  stops       capsule travel from the stick release to rest vs the remaining root path of the first stop clip picked.
  landings    capsule travel 0-0.3 s after the ground flag returns.
  exit idles  picks of the vehicle-exit idle clips and whether a vehicle exit happened shortly before.
  overlay     overlay key / legs layer times around the holster and the following unarmed walk.
"""
import argparse
import bisect
import csv
import datetime as _dt
import glob
import json
import math
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gametest'))
from utpp_paths import project_root  # noqa: E402
import gt_log  # noqa: E402

# ---------------------------------------------------------------------------------------------------------- constants
SETTLE_S = 0.6           # key and stick unchanged for this long before a row counts as steady
ACCEL_MAX = 0.6          # m/s^2, speed derivative over +-DERIV_S
YAW_RATE_MAX = 30.0      # deg/s, velocity heading derivative over +-DERIV_S
DERIV_S = 0.15
POS_TOL = 0.12           # m/s + 10 %: position derivative vs velocity columns
MIN_SEGMENT_S = 0.3
STICK_RELEASED = 0.05
WALK_RUN_STICK = 0.7
FULL_STICK = 0.95
SAMPLE_HZ = 10.0         # decimation rate for the sample count (rows are ~60 Hz and correlated)
MIN_SAMPLES = 50         # decimated samples (= 5 s) per cell
MIN_SEGMENTS = 2
BLOCKED_FRACTION = 0.6  # a steady segment below this fraction of its cell's typical speed is V held by an obstacle
UNSTEADY_DETAIL = {5, 7, 8, 14, 18}   # PSM LocomotionDetailed: slide, air / jump / fall, traversal states seen in the feeds
UPPER_AIM = 6
FEED_COLS = 31
EXIT_WINDOW_S = 6.0      # a vehicle-exit idle picked within this time after an exit counts as "in exit"
LAND_WINDOW_S = 0.3
MAX_AIR_S = 3.0          # longer ground gaps are vehicles / scenes, not jumps
MAX_LAND_SPEED = 12.0    # m/s over the landing window: faster = a teleport
LAND_MOVING = 0.3        # m/s at touchdown: slower landings are on the spot
MIN_AIR_S = 0.15

FEED_NAME = re.compile(r'utpp-feed-(\d{4})-(\d\d)-(\d\d)-(\d\d)-(\d\d)-(\d\d)(?:-\d+)?\.txt$')
WALL = re.compile(r'^\[(\d{4})-(\d\d)-(\d\d) (\d\d):(\d\d):(\d\d)\.(\d{3})\]')
MOTION = re.compile(r'UTPP MOTION t=([-\d.]+) db=(-?\d+):(\S*) clip=(-?\d+):(\S*) ct=([-\d.]+) .*? rot=(\w+) start=(\d) '
                    r'.*? ov=(\d+)/(\d+)/(\d)(\d)\S*(?: w=\S+)? lay=spine([-\d.]+),armL([-\d.]+),armR([-\d.]+),legs([-\d.]+)')
BODY = re.compile(r' bundle=(\w+)')
POS = re.compile(r'\[night\] POS p=([-\d.]+),([-\d.]+),([-\d.]+) .*? m=(\d)')
EXIT_START = re.compile(r'vehicle exit start')
OTHER = re.compile(r'UTPP (MMWEAPON|MMMASK) (.*)$')
STEP = re.compile(r'^=== step (\S+)\s+(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3}) -> (?:\d{4}-\d\d-\d\d )?(\d\d:\d\d:\d\d\.\d{3})\s+(\w+)')


def _epoch(y, mo, d, h, mi, s, ms=0):
    return _dt.datetime(int(y), int(mo), int(d), int(h), int(mi), int(s), int(ms) * 1000).timestamp()


def wall_of(text):
    return _dt.datetime.strptime(text, '%Y-%m-%d %H:%M:%S.%f').timestamp()


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else None


def rnd(x, n=3):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(float(x), n)


# ---------------------------------------------------------------------------------------------------------- classification
def sector(mvx, mvy):
    """Stick direction sector from the raw stick (x right, y forward): F, FD (forward diagonal), S (strafe), BD, B."""
    a = abs(math.degrees(math.atan2(mvx, mvy)))
    if a <= 22.5:
        return 'F'
    if a < 67.5:
        return 'FD'
    if a <= 112.5:
        return 'S'
    if a < 157.5:
        return 'BD'
    return 'B'


def stick_bin(stick):
    return round(round(stick * 20.0) / 20.0, 2)


def classify(loco, upper, walk, stick):
    """(mode, gait) of one feed row, or None for rows outside locomotion (vehicle, scene, unknown state)."""
    if upper == UPPER_AIM:
        mode = 'aim'
        if loco == 2:
            return mode, 'crouch'
        if loco in (0, 1):
            return mode, ('stand_walk' if stick < WALK_RUN_STICK else 'stand_run')
        return None
    mode = 'walk_toggle' if walk == 1 else 'free'
    if loco == 3:
        return mode, 'sprint'
    if loco == 2:
        return mode, 'crouch'
    if loco == 1:
        return mode, ('stand_walk' if stick < WALK_RUN_STICK else 'stand_run')
    return None


# ---------------------------------------------------------------------------------------------------------- feed
def _num(s):
    if s == '-':
        return math.nan
    try:
        return float(s)
    except ValueError:
        return math.nan


def read_feed(path):
    cols = {k: [] for k in ('t', 'dt', 'vx', 'vy', 'mvx', 'mvy', 'yaw', 'loco', 'upper', 'det', 'gnd', 'walk', 'x', 'y', 'z')}
    idx = {'t': 0, 'dt': 1, 'vx': 2, 'vy': 3, 'mvx': 5, 'mvy': 6, 'yaw': 7, 'loco': 9, 'x': 19, 'y': 20, 'z': 21, 'upper': 25,
           'det': 26, 'gnd': 27, 'walk': 30}
    with open(path, encoding='utf-8', errors='replace') as f:
        for ln in f:
            if not ln.strip() or ln[0] == '#':
                continue
            p = ln.split()
            if len(p) < FEED_COLS:
                continue
            for k, i in idx.items():
                cols[k].append(_num(p[i]))
    return {k: np.asarray(v, float) for k, v in cols.items()}


def feed_start(path):
    m = FEED_NAME.search(os.path.basename(path))
    return _epoch(*m.groups()) if m else None


def derive(feed):
    """Speed, stick, heading-rate, acceleration and position-derivative arrays of a feed (adds keys in place)."""
    t = feed['t']
    n = len(t)
    sp = np.hypot(feed['vx'], feed['vy'])
    feed['speed'] = sp
    feed['stick'] = np.hypot(feed['mvx'], feed['mvy'])
    if n < 3:
        feed['accel'] = np.zeros(n)
        feed['yawrate'] = np.zeros(n)
        feed['posspeed'] = sp.copy()
        return feed
    lo = np.searchsorted(t, t - DERIV_S)
    hi = np.clip(np.searchsorted(t, t + DERIV_S, side='right') - 1, 0, n - 1)
    span = np.maximum(t[hi] - t[lo], 1e-6)
    feed['accel'] = (sp[hi] - sp[lo]) / span
    head = np.unwrap(np.arctan2(feed['vy'], feed['vx']))
    feed['yawrate'] = np.degrees(np.abs(head[hi] - head[lo])) / span
    lo2 = np.searchsorted(t, t - 0.1)
    hi2 = np.clip(np.searchsorted(t, t + 0.1, side='right') - 1, 0, n - 1)
    span2 = np.maximum(t[hi2] - t[lo2], 1e-6)
    feed['posspeed'] = np.hypot(feed['x'][hi2] - feed['x'][lo2], feed['y'][hi2] - feed['y'][lo2]) / span2
    return feed


def steady_rows(feed):
    """Per row: cell key (mode, gait, sector, stick bin) or None, and the steady flag."""
    t, n = feed['t'], len(feed['t'])
    keys = [None] * n
    for i in range(n):
        st = feed['stick'][i]
        loco = feed['loco'][i]
        if not math.isfinite(loco):
            continue
        up = feed['upper'][i]
        c = classify(int(loco), int(up) if math.isfinite(up) else -1, int(feed['walk'][i]) if math.isfinite(feed['walk'][i]) else 0, st)
        if c is None or st < STICK_RELEASED:
            continue
        keys[i] = (c[0], c[1], sector(feed['mvx'][i], feed['mvy'][i]), stick_bin(st))
    steady = np.zeros(n, bool)
    since = 0
    for i in range(n):
        if i == 0 or keys[i] != keys[i - 1] or abs(feed['stick'][i] - feed['stick'][i - 1]) > 0.02:
            since = i
        if keys[i] is None:
            continue
        if t[i] - t[since] < SETTLE_S:
            continue
        det = feed['det'][i]
        if feed['gnd'][i] != 1 or (math.isfinite(det) and int(det) in UNSTEADY_DETAIL):
            continue
        s = feed['speed'][i]
        if s < 0.05 or abs(feed['accel'][i]) > ACCEL_MAX or feed['yawrate'][i] > YAW_RATE_MAX:
            continue
        if abs(feed['posspeed'][i] - s) > POS_TOL + 0.1 * s:
            continue
        steady[i] = True
    return keys, steady


def segments(keys, steady, t):
    """Contiguous steady runs of one key: list of (key, first row, last row) at least MIN_SEGMENT_S long."""
    out = []
    i, n = 0, len(t)
    while i < n:
        if not steady[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and steady[j + 1] and keys[j + 1] == keys[i]:
            j += 1
        if t[j] - t[i] >= MIN_SEGMENT_S:
            out.append((keys[i], i, j))
        i = j + 1
    return out


# ---------------------------------------------------------------------------------------------------------- log
def read_log(path):
    """MOTION, harness POS, vehicle exit and MMWEAPON / MMMASK lines of the session of `path` (the rotated parts included)."""
    motion, pos, exits, other = [], [], [], []
    body = None
    with gt_log.open_session_lines(path) as f:
        for ln in f:
            w = WALL.match(ln)
            if not w:
                continue
            if 'UTPP MOTION ' in ln:
                m = MOTION.search(ln)
                if m:
                    g = m.groups()
                    b = BODY.search(ln)
                    if b:
                        body = body or b.group(1)
                    motion.append({'wall': _epoch(*w.groups()), 't': float(g[0]), 'db': int(g[1]), 'dbname': g[2], 'clip': int(g[3]),
                                   'name': g[4], 'ct': float(g[5]), 'rot': g[6], 'start': int(g[7]), 'overlay': int(g[8]),
                                   'aim': int(g[9]), 'crouch': int(g[10]), 'moving': int(g[11]), 'legs': float(g[15]),
                                   'chg': ' chg' in ln, 'bundle': b.group(1) if b else None})
                continue
            if '[night] POS' in ln:
                m = POS.search(ln)
                if m:
                    pos.append((_epoch(*w.groups()), float(m.group(1)), float(m.group(2)), float(m.group(3)), int(m.group(4))))
                continue
            if EXIT_START.search(ln):
                exits.append(_epoch(*w.groups()))
                continue
            m = OTHER.search(ln)
            if m:
                other.append((_epoch(*w.groups()), m.group(1), m.group(2)))
    for a, b in zip(pos, pos[1:]):   # mounted flag 1 -> 0 on the harness position line = an exit
        if a[4] == 1 and b[4] == 0:
            exits.append(b[0])
    exits.sort()
    return {'motion': motion, 'pos': pos, 'exits': exits, 'other': other, 'body': body}


def align(feed, start, pos):
    """Offset (s) such that wall = start + t + offset, from the harness position lines matched to the feed capsule path.
    Returns (offset, residual MAD in s, matches)."""
    t, x, y, sp = feed['t'], feed['x'], feed['y'], feed['speed']
    offs = []
    for w, px, py, _pz, _m in pos:
        tn = w - start
        lo, hi = np.searchsorted(t, tn - 3.0), np.searchsorted(t, tn + 3.0)
        if hi - lo < 10:
            continue
        d = np.hypot(x[lo:hi] - px, y[lo:hi] - py)
        k = int(np.nanargmin(d)) if np.isfinite(d).any() else -1
        if k < 0 or d[k] > 0.03 or sp[lo + k] < 0.8:
            continue
        offs.append(w - (start + t[lo + k]))
    if len(offs) < 5:
        return 0.0, None, len(offs)
    o = float(np.median(offs))
    return o, float(np.median(np.abs(np.asarray(offs) - o))), len(offs)


# ---------------------------------------------------------------------------------------------------------- database
class ClipPaths:
    """Planar root path of every clip of a motion database (ROOT section): distance travelled at a clip time."""

    HEADER_FMT = '<8sIIQQI' + 'I' * 11 + 'f' + 'IIIIIII' + 'QQ'

    def __init__(self, path):
        with open(path, 'rb') as f:
            h = struct.unpack_from(self.HEADER_FMT, f.read(128), 0)
            if h[0] != b'UTPPMMDB' or h[1] != 2:
                raise ValueError('not a version 2 motion database: ' + path)
            nframes, nclips, fps, nsec, tab = h[6], h[8], h[17], h[18], h[19]
            f.seek(tab)
            secs = {}
            for _ in range(nsec):
                sid, cnt, esz, _r, off, size = struct.unpack('<IIIIQQ', f.read(32))
                secs[struct.pack('<I', sid).decode('ascii')] = (off, size)

            def blob(name):
                off, size = secs[name]
                f.seek(off)
                return f.read(size)

            strs, cb = blob('STRS'), blob('CLIP')
            root = np.frombuffer(blob('ROOT'), dtype='<f4').reshape(nframes, 4)
        self.fps = float(fps)
        self.clips = {}
        step = np.hypot(root[:, 0], root[:, 1]).astype(float)
        for i in range(nclips):
            noff, first, n, flags, period, _nsel, _mask = struct.unpack_from('<IIIIIIQ', cb, 32 * i)
            name = strs[noff:strs.index(b'\0', noff)].decode('utf-8')
            loop = bool(flags & 1) and period > 0
            steps = step[first:first + (period if loop else max(n - 1, 0))]
            cum = np.concatenate([[0.0], np.cumsum(steps)])
            self.clips[name] = {'index': i, 'loop': loop, 'frames': n, 'cum': cum, 'length': len(steps) / self.fps}

    def get(self, name):
        return self.clips.get(name)

    def distance(self, clip, ct):
        """Root path length from clip time 0 to ct (loops continue past their length)."""
        cum, L = clip['cum'], clip['length']
        if L <= 0:
            return 0.0
        cycles = 0
        if clip['loop']:
            cycles = math.floor(ct / L)
            ct -= cycles * L
        f = min(max(ct * self.fps, 0.0), len(cum) - 1.0)
        k = int(f)
        frac = f - k
        d = cum[k] + (frac * (cum[k + 1] - cum[k]) if k + 1 < len(cum) else 0.0)
        return cycles * cum[-1] + d

    def mean_speed(self, clip):
        return clip['cum'][-1] / clip['length'] if clip['length'] > 0 else 0.0


def unwrap_ct(clip, ct1, ct2):
    """Clip time ct2 continued from ct1 for a loop that wrapped once; None when the advance is not determinable."""
    if ct2 >= ct1 - 1e-4:
        return ct2
    if clip is None or not clip['loop']:
        return None
    return ct2 + clip['length']


# ---------------------------------------------------------------------------------------------------------- run processing
class Run:
    def __init__(self, folder):
        self.folder = folder
        self.name = os.path.basename(folder.rstrip('\\/'))
        logs = os.path.join(folder, 'logs')
        feeds = sorted(glob.glob(os.path.join(logs, 'utpp-feed-*.txt')))
        plog = gt_log.session_log_in(logs)
        self.body = None
        sj = os.path.join(folder, 'summary.json')
        if os.path.isfile(sj):
            with open(sj, encoding='utf-8') as f:
                self.body = json.load(f).get('body')
        self.feed = derive(read_feed(feeds[0])) if feeds else None
        self.start = feed_start(feeds[0]) if feeds else None
        self.log = read_log(plog) if plog else {'motion': [], 'pos': [], 'exits': [], 'other': [], 'body': None}
        if self.body is None and self.log['body']:
            self.body = {'woman': 'female', 'man': 'male'}.get(self.log['body'], self.log['body'])
        self.offset, self.align_mad, self.align_n = (align(self.feed, self.start, self.log['pos']) if self.feed is not None
                                                     else (0.0, None, 0))
        self.aligned = self.align_mad is not None and self.align_mad < 0.05
        if self.feed is not None:
            self.keys, self.steady = steady_rows(self.feed)
            self.segs = segments(self.keys, self.steady, self.feed['t'])
        else:
            self.keys, self.steady, self.segs = [], np.zeros(0, bool), []
        self.steps = []
        for p in glob.glob(os.path.join(folder, 'gametest', '*_r*.txt')):
            scen = re.sub(r'_r\d+\.txt$', '', os.path.basename(p))
            with open(p, encoding='utf-8', errors='replace') as f:
                for ln in f:
                    m = STEP.match(ln)
                    if m:
                        s0 = wall_of(m.group(2))
                        s1 = wall_of(m.group(2)[:11] + m.group(3))
                        self.steps.append((scen, m.group(1), s0, s1 if s1 >= s0 else s1 + 86400.0, m.group(4)))
        self.steps.sort(key=lambda s: s[2])

    def feed_t(self, wall):
        return wall - self.start - self.offset

    def row_at(self, wall):
        t = self.feed['t']
        k = int(np.searchsorted(t, self.feed_t(wall)))
        return min(max(k, 0), len(t) - 1)


def cell_stats(values_by_cell, seconds, segs_by_cell):
    out = {}
    for key, vals in values_by_cell.items():
        v = np.asarray(vals, float)
        secs = seconds[key]
        nseg = len(segs_by_cell[key])
        seg_meds = [s for s in segs_by_cell[key]]
        samples = int(round(secs * SAMPLE_HZ))
        out[key] = {'median': rnd(np.median(v)), 'p10': rnd(pct(v, 10)), 'p90': rnd(pct(v, 90)), 'mean': rnd(v.mean()),
                    'std': rnd(v.std()), 'rows': int(len(v)), 'seconds': rnd(secs, 2), 'samples': samples, 'segments': nseg,
                    'segment_median_min': rnd(min(seg_meds)), 'segment_median_max': rnd(max(seg_meds)),
                    'enough': bool(samples >= MIN_SAMPLES and nseg >= MIN_SEGMENTS)}
    return out


def blocked_segments(runs, frac=BLOCKED_FRACTION):
    """(run name, first row) of steady segments whose median speed is below frac x the median of the segment medians of their
    cell (per body): V pressed against an obstacle at a constant low speed, not the free speed of the cell."""
    meds = {}
    for r in runs:
        if r.feed is None:
            continue
        for key, i, j in r.segs:
            meds.setdefault((r.body,) + key, []).append((r.name, i, float(np.median(r.feed['speed'][i:j + 1]))))
    out = set()
    for k, lst in meds.items():
        ref = float(np.median([m for _n, _i, m in lst]))
        out.update((n, i) for n, i, m in lst if m < frac * ref)
    return out


def measure_speeds(runs, blocked):
    vals, secs, segm, stick, nblk = {}, {}, {}, {}, {}
    for r in runs:
        if r.feed is None:
            continue
        f = r.feed
        for key, i, j in r.segs:
            for body in (r.body, 'both'):
                k = (body,) + key
                if (r.name, i) in blocked:
                    nblk[k] = nblk.get(k, 0) + 1
                    continue
                sl = slice(i, j + 1)
                vals.setdefault(k, []).extend(f['speed'][sl].tolist())
                secs[k] = secs.get(k, 0.0) + float(np.nansum(f['dt'][sl]))
                segm.setdefault(k, []).append(float(np.median(f['speed'][sl])))
                stick.setdefault(k, []).extend(f['stick'][sl].tolist())
    st = cell_stats(vals, secs, segm)
    for k in st:
        st[k]['stick'] = rnd(np.median(stick[k]))
        st[k]['blocked_segments_excluded'] = nblk.get(k, 0)
    return st


def coverage(runs):
    """Rows per (mode, gait, sector, stick bin) with no steady filter: what the recordings contain at all."""
    cov = {}
    for r in runs:
        if r.feed is None:
            continue
        for k, dt in zip(r.keys, r.feed['dt']):
            if k is None:
                continue
            kk = (r.body,) + k
            c = cov.setdefault(kk, [0, 0.0])
            c[0] += 1
            c[1] += float(dt) if math.isfinite(dt) else 0.0
    return cov


def seg_index(run):
    """Row -> segment key lookup (None outside steady segments)."""
    seg_of = [None] * len(run.feed['t'])
    for key, i, j in run.segs:
        for k in range(i, j + 1):
            seg_of[k] = (key, i)
    return seg_of


def measure_ratios(runs, dbs, blocked=frozenset()):
    pairs = []
    for r in runs:
        if r.feed is None or not r.aligned:
            continue
        db = dbs.get(r.body)
        if db is None:
            continue
        seg_of = seg_index(r)
        mo = r.log['motion']
        f = r.feed
        for a, b in zip(mo, mo[1:]):
            if b['chg'] or a['clip'] != b['clip'] or a['db'] != b['db'] or a['name'] != b['name']:
                continue
            dt = b['t'] - a['t']
            if not 0.2 <= dt <= 0.8:
                continue
            clip = db.get(a['name'])
            if clip is None or clip['length'] <= 0:
                continue
            if clip['loop'] and dt * 1.6 >= clip['length']:
                continue
            ct2 = unwrap_ct(clip, a['ct'], b['ct'])
            if ct2 is None:
                continue
            if not clip['loop'] and ct2 >= clip['length'] - 1.0 / db.fps:
                continue
            i, j = r.row_at(a['wall']), r.row_at(b['wall'])
            if j <= i or seg_of[i] is None or seg_of[i] != seg_of[j] or (r.name, seg_of[i][1]) in blocked:
                continue
            cap = float(np.mean(f['speed'][i:j + 1]))
            if cap < 0.05:
                continue
            d = db.distance(clip, ct2) - db.distance(clip, a['ct'])
            dct = ct2 - a['ct']
            rate = dct / dt
            if not 0.2 <= rate <= 2.5:
                continue
            leg = d / dt
            pairs.append({'run': r.name, 'body': r.body, 'key': seg_of[i][0], 'clip': a['name'], 'loop': clip['loop'],
                          'rate': rate, 'clip_speed': d / dct if dct > 0 else 0.0, 'clip_mean_speed': db.mean_speed(clip),
                          'leg': leg, 'capsule': cap, 'ratio': leg / cap})
    cells = {}
    for p in pairs:
        mode, gait, sec, sb = p['key']
        for body in (p['body'], 'both'):
            cells.setdefault((body, mode, gait, sec, sb), []).append(p)
    out = {}
    for k, ps in cells.items():
        clips = {}
        for p in ps:
            clips[p['clip']] = clips.get(p['clip'], 0) + 1
        arr = lambda f: np.asarray([p[f] for p in ps], float)  # noqa: E731
        out[k] = {'pairs': len(ps), 'rate': rnd(np.median(arr('rate'))), 'clip_speed': rnd(np.median(arr('clip_speed'))),
                  'leg_speed': rnd(np.median(arr('leg'))), 'capsule_speed': rnd(np.median(arr('capsule'))),
                  'ratio': rnd(np.median(arr('ratio'))), 'ratio_p10': rnd(pct(arr('ratio'), 10)),
                  'ratio_p90': rnd(pct(arr('ratio'), 90)), 'loop_share': rnd(np.mean(arr('loop'))),
                  'clips': sorted(clips.items(), key=lambda kv: -kv[1])[:4]}
    return out, pairs


def gait_label(feed, i):
    loco = feed['loco'][i]
    up = feed['upper'][i]
    c = classify(int(loco) if math.isfinite(loco) else -1, int(up) if math.isfinite(up) else -1,
                 int(feed['walk'][i]) if math.isfinite(feed['walk'][i]) else 0, feed['stick'][i])
    return None if c is None else c[0] + '/' + c[1]


def path_length(feed, i, j):
    if j <= i:
        return 0.0
    return float(np.nansum(np.hypot(np.diff(feed['x'][i:j + 1]), np.diff(feed['y'][i:j + 1]))))


def measure_stops(runs, dbs):
    events = []
    for r in runs:
        if r.feed is None or not r.aligned:
            continue
        f, t = r.feed, r.feed['t']
        db = dbs.get(r.body)
        mo = r.log['motion']
        mwall = [m['wall'] for m in mo]
        n = len(t)
        for i in range(1, n):
            if not (f['stick'][i - 1] >= 0.2 and f['stick'][i] < STICK_RELEASED):
                continue
            lo = int(np.searchsorted(t, t[i] - 0.5))
            if np.ptp(f['stick'][lo:i]) > 0.05 or f['speed'][i - 1] < 0.3 or f['gnd'][i - 1] != 1:
                continue
            k = i
            while k < n and f['stick'][k] < STICK_RELEASED and f['speed'][k] >= 0.05 and t[k] - t[i] < 3.0:
                k += 1
            if k >= n or f['stick'][k] >= STICK_RELEASED or f['speed'][k] >= 0.05:
                continue
            gait = gait_label(f, i - 1)
            if gait is None:
                continue
            rel_wall = r.start + r.offset + t[i]
            ev = {'run': r.name, 'body': r.body, 'gait': gait, 'stick': rnd(f['stick'][i - 1], 2), 'speed': rnd(f['speed'][i - 1]),
                  'capsule_stop_m': rnd(path_length(f, i - 1, k)), 'stop_s': rnd(t[k] - t[i])}
            q = bisect.bisect_left(mwall, rel_wall - 0.1)
            pick = None
            while q < len(mo) and mo[q]['wall'] <= rel_wall + 0.8:
                if 'Stop' in mo[q]['name']:
                    pick = mo[q]
                    break
                q += 1
            if pick is not None and db is not None and db.get(pick['name']) is not None:
                clip = db.get(pick['name'])
                remaining = db.distance(clip, clip['length']) - db.distance(clip, pick['ct'])
                pi = r.row_at(pick['wall'])
                cap_rem = path_length(f, pi, k) if pi < k else 0.0
                later = 0
                q2 = q + 1
                while q2 < len(mo) and mo[q2]['wall'] <= r.start + r.offset + t[k] + 0.5:
                    if mo[q2]['chg'] and mo[q2]['name'] != mo[q2 - 1]['name']:
                        later += 1
                    q2 += 1
                ev.update({'stop_clip': pick['name'], 'pick_delay_s': rnd(pick['wall'] - rel_wall), 'pick_ct': rnd(pick['ct']),
                           'clip_remaining_m': rnd(remaining), 'capsule_remaining_m': rnd(cap_rem),
                           'overshoot_cm': rnd(100.0 * (remaining - cap_rem), 1), 'clip_changes_until_rest': later})
            else:
                ev['stop_clip'] = None
            events.append(ev)
    groups = {}
    for e in events:
        g = (e['gait'], stick_bin(e['stick']))
        groups.setdefault((e['body'],) + g, []).append(e)
        groups.setdefault(('both',) + g, []).append(e)
    summary = {}
    for k, es in groups.items():
        ov = np.asarray([e['overshoot_cm'] for e in es if e.get('overshoot_cm') is not None], float)
        summary[k] = {'events': len(es), 'with_stop_clip': int(len(ov)),
                      'speed': rnd(np.median([e['speed'] for e in es])),
                      'capsule_stop_m': rnd(np.median([e['capsule_stop_m'] for e in es])),
                      'stop_s': rnd(np.median([e['stop_s'] for e in es])),
                      'overshoot_cm_median': rnd(np.median(ov), 1) if len(ov) else None,
                      'overshoot_cm_p90': rnd(pct(ov, 90), 1) if len(ov) else None,
                      'overshoot_cm_max': rnd(ov.max(), 1) if len(ov) else None,
                      'share_over_10cm': rnd(float(np.mean(ov > 10.0)), 2) if len(ov) else None}
    return summary, events


def measure_landings(runs, dbs=None):
    events = []
    skipped = {'standing': 0, 'long_air_or_teleport': 0}
    for r in runs:
        if r.feed is None:
            continue
        f, t = r.feed, r.feed['t']
        g = f['gnd']
        n = len(t)
        air_since = None
        for i in range(1, n):
            if g[i - 1] == 0 and air_since is None:
                air_since = i - 1
            if g[i] == 1 and g[i - 1] == 0 and air_since is not None:
                a0 = air_since
                air = t[i] - t[a0]
                air_since = None
                if air < MIN_AIR_S:
                    continue
                k = int(np.searchsorted(t, t[i] + LAND_WINDOW_S))
                if k >= n:
                    continue
                travel = path_length(f, i, k)
                held = bool(np.nanmax(f['stick'][i:k + 1]) >= 0.2)
                dets = sorted({int(d) for d in f['det'][a0:i] if math.isfinite(d)})
                if air > MAX_AIR_S or travel > MAX_LAND_SPEED * LAND_WINDOW_S:
                    skipped['long_air_or_teleport'] += 1
                    continue
                if f['speed'][i] < LAND_MOVING:
                    skipped['standing'] += 1
                    continue
                ev = {'run': r.name, 'body': r.body, 'air_s': rnd(air, 2), 'speed_touchdown': rnd(f['speed'][i]),
                      'speed_end': rnd(f['speed'][k]), 'travel_m': rnd(travel), 'stick_held': held, 'air_details': dets,
                      'no_decel_travel_m': rnd(f['speed'][i] * LAND_WINDOW_S)}
                land = land_pick(r, dbs, r.start + r.offset + t[i])
                if land:
                    ev.update(land)
                events.append(ev)
            if g[i] == 1:
                air_since = None
    summary = {}
    for held in (True, False):
        for body in ('female', 'male', 'both'):
            es = [e for e in events if e['stick_held'] == held and (body == 'both' or e['body'] == body)]
            if not es:
                continue
            tr = np.asarray([e['travel_m'] for e in es], float)
            summary[(body, 'stick_held' if held else 'stick_released')] = {
                'events': len(es), 'travel_m_median': rnd(np.median(tr)), 'travel_m_p90': rnd(pct(tr, 90)),
                'travel_m_max': rnd(tr.max()), 'speed_touchdown_median': rnd(np.median([e['speed_touchdown'] for e in es])),
                'speed_end_median': rnd(np.median([e['speed_end'] for e in es])),
                'no_decel_travel_m_median': rnd(np.median([e['no_decel_travel_m'] for e in es])),
                'clip_travel_m_median': rnd(np.median([e['clip_travel_m'] for e in es if e.get('clip_travel_m') is not None]))
                if any(e.get('clip_travel_m') is not None for e in es) else None}
    summary[('skipped',)] = skipped
    return summary, events


def land_pick(run, dbs, wall):
    """First MOTION line within the landing window after the touchdown: clip, its clip time, and the root path of that clip over
    the next LAND_WINDOW_S of game time (at the play rate seen between this and the next line of the same clip, else 1)."""
    if not run.aligned:
        return None
    mo = run.log['motion']
    walls = [m['wall'] for m in mo]
    q = bisect.bisect_left(walls, wall - 0.02)
    if q >= len(mo) or mo[q]['wall'] > wall + LAND_WINDOW_S:
        return None
    m = mo[q]
    out = {'land_clip': m['name'], 'land_clip_ct': rnd(m['ct']), 'land_pick_delay_s': rnd(m['wall'] - wall)}
    db = (dbs or {}).get(run.body)
    clip = db.get(m['name']) if db else None
    if clip is not None:
        rate = 1.0
        if q + 1 < len(mo) and mo[q + 1]['name'] == m['name'] and mo[q + 1]['t'] > m['t']:
            ct2 = unwrap_ct(clip, m['ct'], mo[q + 1]['ct'])
            if ct2 is not None:
                rate = min(max((ct2 - m['ct']) / (mo[q + 1]['t'] - m['t']), 0.2), 2.5)
        out['clip_rate'] = rnd(rate)
        out['clip_travel_m'] = rnd(db.distance(clip, m['ct'] + rate * LAND_WINDOW_S) - db.distance(clip, m['ct']))
    return out


def measure_exit_idles(runs, prefix='VX_Exit_Idle'):
    episodes = []
    totals = {'motion_lines': 0, 'idle_db_lines': 0}
    for r in runs:
        mo = r.log['motion']
        totals['motion_lines'] += len(mo)
        totals['idle_db_lines'] += sum(1 for m in mo if 'Idle' in m['dbname'])
        exits = r.log['exits']
        k = 0
        while k < len(mo):
            if not mo[k]['name'].startswith(prefix):
                k += 1
                continue
            j = k
            while j + 1 < len(mo) and mo[j + 1]['name'] == mo[k]['name']:
                j += 1
            w = mo[k]['wall']
            q = bisect.bisect_right(exits, w)
            since = (w - exits[q - 1]) if q > 0 else None
            episodes.append({'run': r.name, 'body': r.body, 'clip': mo[k]['name'], 'db': mo[k]['dbname'], 'lines': j - k + 1,
                             'seconds_since_exit': rnd(since, 2), 'in_exit': bool(since is not None and since <= EXIT_WINDOW_S)})
            k = j + 1
    return {'episodes': len(episodes), 'in_exit': sum(e['in_exit'] for e in episodes),
            'outside_exit': sum(not e['in_exit'] for e in episodes), **totals}, episodes


def measure_overlay_switch(runs):
    """Holster -> unarmed walk: when the overlay key and the legs layer change relative to the walk start."""
    out = []
    for r in runs:
        walk = [s for s in r.steps if s[0] == 'weapon_walk' and s[1] == 'walk_unarmed']
        if not walk or r.feed is None:
            continue
        _sc, _st, w0, w1, verdict = walk[0]
        hol = [s for s in r.steps if s[1] == 'holster' and s[2] < w0]
        h0 = hol[-1][2] if hol else w0 - 5.0
        mo = [m for m in r.log['motion'] if h0 - 1.0 <= m['wall'] <= w1]
        f, t = r.feed, r.feed['t']
        i0 = r.row_at(w0)
        i1 = r.row_at(w1)
        move = next((i for i in range(i0, i1) if f['stick'][i] >= 0.1), None)
        walk_start = (r.start + r.offset + t[move]) if move is not None else None
        changes = []
        prev = None
        for m in mo:
            key = (m['overlay'], m['aim'], m['crouch'])
            if key != prev:
                changes.append({'dt_to_walk_s': rnd(m['wall'] - walk_start, 3) if walk_start else None, 'overlay': m['overlay'],
                                'aim': m['aim'], 'crouch': m['crouch'], 'legs': m['legs'], 'clip': m['name']})
                prev = key
        last_armed = max((m['wall'] for m in mo if m['overlay'] != 0), default=None)
        legs_zero = next((m['wall'] for m in mo if last_armed and m['wall'] > last_armed and m['legs'] <= 0.0), None)
        legs_series = [(rnd(m['wall'] - walk_start, 3) if walk_start else None, m['overlay'], m['legs']) for m in mo]
        legs_max = max((m['legs'] for m in mo if walk_start and m['wall'] >= walk_start), default=None)
        weapon = [(rnd(w - walk_start, 3) if walk_start else None, kind, txt[:80]) for w, kind, txt in r.log['other']
                  if h0 - 1.0 <= w <= w1]
        out.append({'run': r.name, 'body': r.body, 'verdict': verdict, 'holster_to_walk_s': rnd(w0 - h0, 2),
                    'walk_start_after_step_s': rnd(walk_start - w0, 3) if walk_start else None,
                    'last_armed_overlay_to_walk_s': rnd(last_armed - walk_start, 3) if (last_armed and walk_start) else None,
                    'legs_zero_to_walk_s': rnd(legs_zero - walk_start, 3) if (legs_zero and walk_start) else None,
                    'legs_weight_max_during_walk': legs_max, 'overlay_changes': changes, 'legs_series': legs_series,
                    'weapon_lines': weapon,
                    'walk_unarmed_corr_cm': next((c for sc, st, _s, c in step_corrections(r) if st == 'walk_unarmed'), None)})
    return out


def step_corrections(run):
    """(scenario, step, stick_max, footlock_corr_max_cm) of the stand walk steps with the 0.8 stick of gametest/summary.csv."""
    p = os.path.join(run.folder, 'gametest', 'summary.csv')
    out = []
    if not os.path.isfile(p):
        return out
    with open(p, encoding='utf-8', errors='replace', newline='') as f:
        for row in csv.DictReader(f):
            try:
                corr = float(row.get('footlock_corr_max_cm') or 'nan')
                stick = float(row.get('stick_max') or 'nan')
                stand = float(row.get('stand_fraction') or 'nan')
            except ValueError:
                continue
            if math.isfinite(corr) and abs(stick - 0.8) < 0.01 and stand >= 0.99 and row.get('step') in ('walk_unarmed', 'walk_armed'):
                out.append((row.get('scenario') or '', row.get('step'), stick, corr))
    return out


def overlay_corrections(runs):
    """Per body: foot-lock correction maxima of the unarmed walk after the holster vs the armed walks with the same stick."""
    out = {}
    for r in runs:
        for scen, step, _stick, corr in step_corrections(r):
            d = out.setdefault(r.body, {'walk_unarmed_after_holster_cm': [], 'walk_armed_cm': []})
            d['walk_unarmed_after_holster_cm' if step == 'walk_unarmed' else 'walk_armed_cm'].append(corr)
    for d in out.values():
        for k in list(d):
            v = d[k]
            d[k.replace('_cm', '_median_cm')] = rnd(float(np.median(v)), 1) if v else None
            d[k.replace('_cm', '_max_cm')] = rnd(max(v), 1) if v else None
            d[k.replace('_cm', '_n')] = len(v)
            del d[k]
    return out


# ---------------------------------------------------------------------------------------------------------- output
def keystr(k):
    return '|'.join(str(x) for x in k)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--baseline', help='JSON list of {"run": <night folder name>, ...}')
    ap.add_argument('--runs', nargs='*', default=[])
    ap.add_argument('--night', default=os.path.join(project_root(), 'Research', 'logs', 'night'))
    ap.add_argument('--db-female', default=os.path.join(project_root(), 'Data', 'utpp_motion_woman.utppmm'))
    ap.add_argument('--db-male', default=os.path.join(project_root(), 'Data', 'utpp_motion_man.utppmm'))
    ap.add_argument('--json', required=True)
    a = ap.parse_args(argv)
    folders = list(a.runs)
    meta = {}
    if a.baseline:
        with open(a.baseline, encoding='utf-8') as f:
            for e in json.load(f):
                folders.append(os.path.join(a.night, e['run']))
                meta[e['run']] = {'set': e.get('set'), 'body': e.get('body')}
    runs = [Run(p) for p in folders]
    dbs = {}
    for body, p in (('female', a.db_female), ('male', a.db_male)):
        if os.path.isfile(p):
            dbs[body] = ClipPaths(p)
    blocked = blocked_segments(runs)
    speeds = measure_speeds(runs, blocked)
    cov = coverage(runs)
    ratios, pairs = measure_ratios(runs, dbs, blocked)
    stops, stop_events = measure_stops(runs, dbs)
    lands, land_events = measure_landings(runs, dbs)
    exits, exit_eps = measure_exit_idles(runs)
    overlay = measure_overlay_switch(runs)
    clip_ref = {}
    for body, db in dbs.items():
        for name in ('M_Neutral_Walk_Loop_F', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F', 'M_Neutral_Crouch_Loop_F',
                     'M_Neutral_Walk_Loop_B', 'M_Neutral_Run_Loop_B', 'M_Neutral_Crouch_Loop_B'):
            c = db.get(name)
            if c is not None:
                clip_ref[body + '|' + name] = rnd(db.mean_speed(c))
    out = {
        'generator': 'Source/tools/replay/speed_calibration.py',
        'rules': {'settle_s': SETTLE_S, 'accel_max': ACCEL_MAX, 'yaw_rate_max': YAW_RATE_MAX, 'min_segment_s': MIN_SEGMENT_S,
                  'sample_hz': SAMPLE_HZ, 'min_samples': MIN_SAMPLES, 'min_segments': MIN_SEGMENTS,
                  'walk_run_stick': WALK_RUN_STICK, 'unsteady_detail': sorted(UNSTEADY_DETAIL)},
        'runs': [{'run': r.name, 'body': r.body, **meta.get(r.name, {}), 'feed_rows': 0 if r.feed is None else len(r.feed['t']),
                  'motion_lines': len(r.log['motion']), 'align_offset_s': rnd(r.offset), 'align_mad_s': rnd(r.align_mad, 4),
                  'align_matches': r.align_n, 'aligned': r.aligned, 'steady_segments': len(r.segs)} for r in runs],
        'speeds': {keystr(k): v for k, v in sorted(speeds.items())},
        'coverage': {keystr(k): {'rows': v[0], 'seconds': rnd(v[1], 2)} for k, v in sorted(cov.items())},
        'clip_mean_speeds': clip_ref,
        'ratios': {keystr(k): v for k, v in sorted(ratios.items())},
        'ratio_pairs': len(pairs),
        'stops': {keystr(k): v for k, v in sorted(stops.items())},
        'stop_events': stop_events,
        'landings': {keystr(k): v for k, v in sorted(lands.items())},
        'landing_events': land_events,
        'exit_idles': exits,
        'exit_idle_episodes': exit_eps,
        'overlay_switch': overlay,
        'overlay_corrections': overlay_corrections(runs),
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
    with open(a.json, 'w', encoding='utf-8') as f:
        json.dump(out, f, indent=1)
    print('runs %d, speed cells %d, ratio pairs %d, stops %d, landings %d, exit idle episodes %d -> %s'
          % (len(runs), len(speeds), len(pairs), len(stop_events), len(land_events), len(exit_eps), a.json))
    return 0


if __name__ == '__main__':
    sys.exit(main())
