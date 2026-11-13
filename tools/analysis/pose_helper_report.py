"""Reports the upper-body core bones and every shoulder-girdle / upper-back helper (deformation) bone of a compact pose recording
(utpp-pose-*.utpppose, written by the mod's 'Diagnostic logs'), per recorded phase, for a wall-clock window or the whole file.

  python -I pose_helper_report.py INPUT [INPUT ...] --out-dir DIR [--from HH:MM:SS --to HH:MM:SS] [--compare OTHER [OTHER ...]]
                                  [--compare-from HH:MM:SS --compare-to HH:MM:SS] [--prefix NAME] [--top N] [--utc | --tz-hours H]

INPUT / OTHER are .utpppose files, folders or wildcards (segments of one capture are joined in time order, like pose_capture_to_csv.py,
whose parsing functions are imported; the format is described at the top of src/mm/PoseCaptureFormat.hpp). --out-dir is required: nothing
is ever written next to the input (the recordings normally sit in the game folder). Read-only on the inputs.

Phases are named exactly as the file header names them: stream 0 = before_mm, after_mm_and_aim (the pose before and after the mod writes
the upper body); stream 1 (only in captures that enabled it) = post_helper_apply_after_commit (what the engine holds after helper /
deformer evaluation). The frames of stream 1 are matched to the frames of stream 0 by their QPC ticks (tolerance 5 ms).

Per bone value layout (BodyDeformerTrace.hpp:205, GameIntegration.hpp:37-38): 12 floats = local translation xyz in metres (x100 -> cm),
one unused float, local quaternion xyzw (renormalised), scale. Angles are the rotation angle of conj(rest) * current in degrees.

Outputs (in --out-dir, prefix default pose_helper_report):
  <prefix>_bones.csv        per bone and phase: rotation angle vs the reference min / median / p95 / max (deg), translation delta
                            median / p95 / max (cm)
  <prefix>_consistency.csv  per helper and phase: frames with a frame-to-frame jump (rotation >= --jump-deg or translation >= --jump-cm),
                            jumps while the drivers hardly move, frames at an extreme (>= --extreme-deg or >= --extreme-cm from the
                            reference) while all drivers are in the normal range, worst frames, FLAGS
  <prefix>_phasediff.csv    per bone: difference between two phases of the same frame (median / p95 / max deg, max cm)
  <prefix>_compare.csv      with --compare: per bone and phase both recordings side by side, sorted by the size of the difference
  <prefix>_summary.txt      short text summary (also printed)

LIMITS (read before trusting a flag):
  * The format holds NO rest / bind pose. The reference of every phase is the per-bone median of the recording itself (whole file with a
    stride of --rest-stride frames, or --rest-from/--rest-to when given). 'deg / cm from rest' therefore means 'from the typical pose of this
    recording'; a helper that sits at an extreme for most of the recording is not caught by the extreme check, a pose held for most of the
    recording becomes the reference. Use --rest-from/--rest-to on a calm idle stretch for a meaningful reference.
  * Percentiles come from fixed-width histograms (0.02 deg / 0.02 cm); min and max are exact. Values carry the quantization error of the
    format (< 1/65535 of the block range).
  * Drivers of a helper: the nearest recorded ancestor in the rig topology, the same-side clavicle (<Side>Shoulder), the same-side upper arm
    (<Side>Arm) and Spine3. 'Normal range' = clavicle / spine / parent below --driver-deg, upper arm (and a parent that is an arm) below
    --arm-deg. A frame-to-frame jump counts as 'undriven' when no driver moved more than a quarter of the jump threshold in that frame.
  * Only the 185 catalogue bones are recorded; bones of the 645-bone rig that are not in the catalogue are not recorded.
  * read_capture() of the converter reads a whole file into memory; the analysis itself keeps only histograms and counters.
"""
import argparse
import csv
import datetime
import os
import pathlib
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import pose_capture_to_csv as pc  # noqa: E402

CORE_BONES = ['Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm',
              'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand']
ANCHORS = {'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm'}
STOPS = {'Hips', 'Spine', 'Head', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand'}
ARM_BONES = {'LeftArm', 'RightArm'}
HELPER_REGEX = re.compile(r'butterfly|latissimus|lats|scapula|SHL|deltoid|trapez|pector|armpit|twist|clavicle|sternocleido|chest_|biceps|triceps',
                          re.IGNORECASE)
KEYWORDS = ['butterfly', 'latissimus', 'lats', 'scapula', 'SHL', 'deltoid', 'trapez', 'pector', 'armpit', 'twist', 'clavicle', 'sternocleido',
            'chest_', 'biceps', 'triceps', 'JNT']
TICK_TOLERANCE_S = 0.005
ROT_BW, ROT_BINS = 0.02, 9001
TR_BW, TR_BINS = 0.02, 10001


class Config:
    def __init__(self, **kw):
        self.window = None            # (from_s, to_s): seconds since local midnight of the first file's day
        self.tz_seconds = None        # None = local offset detected from the file name stamp
        self.rest_window = None
        self.rest_stride = 10
        self.jump_deg, self.jump_cm = 25.0, 3.0
        self.extreme_deg, self.extreme_cm = 60.0, 5.0
        self.driver_deg, self.arm_deg = 35.0, 100.0
        self.min_jump_frames, self.min_extreme_frames = 1, 3
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError('unknown option ' + k)
            setattr(self, k, v)


# --------------------------------------------------------------------------------------------------------------------- time
def parse_clock(text):
    """HH:MM:SS[.fff] -> seconds since midnight."""
    m = re.fullmatch(r'(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)', text.strip())
    if not m:
        raise SystemExit('bad time %r (expected HH:MM:SS)' % text)
    return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))


def make_window(a, b):
    """(from, to) text -> seconds window; to <= from means the window crosses midnight."""
    if a is None and b is None:
        return None
    lo = parse_clock(a) if a is not None else 0.0
    hi = parse_clock(b) if b is not None else 1e12
    if hi <= lo:
        hi += 86400.0
    return (lo, hi)


def clock(sec):
    s = sec % 86400.0
    return '%02d:%02d:%06.3f' % (int(s // 3600), int(s % 3600 // 60), s % 60)


def local_offset_seconds(path, header, tz_seconds=None):
    """Local-minus-UTC offset in seconds. The header holds UTC; the file name stamp (utpp-pose-YYYY-MM-DD-HH-MM-SS) is local time."""
    if tz_seconds is not None:
        return float(tz_seconds)
    m = re.search(r'(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})', os.path.basename(path))
    if not m:
        return 0.0
    name_dt = datetime.datetime(*(int(x) for x in m.groups()))
    u = header.utc
    utc_dt = datetime.datetime(u[0], u[1], u[2], u[3], u[4], u[5], u[6] * 1000)
    diff = (name_dt - utc_dt).total_seconds()
    if abs(diff) > 15 * 3600:
        return 0.0
    return round(diff / 900.0) * 900.0


def read_header(path):
    with open(path, 'rb') as f:
        head = f.read(16)
        if len(head) == 16 and head[:8] == pc.MAGIC:
            head += f.read(struct.unpack_from('<I', head, 12)[0] - 16)
    try:
        return pc.parse_header(head)[0]
    except ValueError as e:
        raise SystemExit('%s: %s' % (path, e))


def load_groups(inputs, cfg):
    """-> list of groups; a group = dict(files=[(path, header, start_sec)], header, offset) of one rig topology in time order.
    start_sec = local seconds since local midnight of the first file's day."""
    files = pc._expand(inputs)
    loaded = []
    for p in files:
        h = read_header(p)
        loaded.append((h.start_key(), p, h))
    loaded.sort(key=lambda t: (t[0], t[1]))
    groups, key = [], None
    for _, p, h in loaded:
        if h.topology_key() != key:
            key = h.topology_key()
            groups.append({'files': [], 'header': h})
        groups[-1]['files'].append((p, h))
    for g in groups:
        out, day0 = [], None
        for p, h in g['files']:
            off = local_offset_seconds(p, h, cfg.tz_seconds)
            u = h.utc
            start = datetime.datetime(u[0], u[1], u[2], u[3], u[4], u[5], u[6] * 1000) + datetime.timedelta(seconds=off)
            if day0 is None:
                day0 = start.replace(hour=0, minute=0, second=0, microsecond=0)
                g['offset'] = off
                g['day'] = day0.date().isoformat()
            out.append((p, h, (start - day0).total_seconds()))
        g['files'] = out
    return groups


def iter_blocks(group):
    """Yields (stream, ticks, sec, gidx, values[n, bones, stages, 12]) over the files of a group; collects notes about cut-short files."""
    counters = {0: 0, 1: 0}
    notes = group.setdefault('notes', [])
    for path, h, start_sec in group['files']:
        nm = len(h.mapped())
        for item in pc.read_capture(path):
            if item[0] == 'block':
                _, stream, meta, values = item
                n = meta.shape[0]
                ticks = meta[:, 0]
                sec = start_sec + (ticks - h.arm_ticks) / float(h.qpc_hz)
                gidx = counters[stream] + np.arange(n)
                counters[stream] += n
                yield stream, ticks, sec, gidx, values.reshape(n, nm, pc.STAGES[stream], 12)
            elif item[0] == 'trailer':
                if item[1] is None or item[2]:
                    n = '%s: no trailer / truncated block (cut short); read up to the last complete block' % os.path.basename(path)
                    if n not in notes:
                        notes.append(n)


# --------------------------------------------------------------------------------------------------------------------- math
def normq(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(n > 1e-9, n, np.nan)


def qangle(a, b):
    """Rotation angle in degrees of conj(a) * b for xyzw quaternions (broadcasting, float64, accurate for small angles)."""
    av, aw = a[..., :3], a[..., 3]
    bv, bw = b[..., :3], b[..., 3]
    w = aw * bw + np.sum(av * bv, axis=-1)
    v = aw[..., None] * bv - bw[..., None] * av - np.cross(av, bv)
    return np.degrees(2.0 * np.arctan2(np.linalg.norm(v, axis=-1), np.abs(w)))


class Hist:
    """Per column: exact count / min / max and a fixed-width histogram for percentiles."""

    def __init__(self, columns, width, bins):
        self.width, self.bins, self.cols = width, bins, columns
        self.n = np.zeros(columns, dtype=np.int64)
        self.min = np.full(columns, np.inf)
        self.max = np.full(columns, -np.inf)
        self.h = np.zeros((columns, bins), dtype=np.int64)

    def add(self, x):
        if x.shape[0] == 0:
            return
        valid = np.isfinite(x)
        self.n += valid.sum(axis=0)
        self.min = np.minimum(self.min, np.where(valid, x, np.inf).min(axis=0))
        self.max = np.maximum(self.max, np.where(valid, x, -np.inf).max(axis=0))
        idx = np.minimum((np.where(valid, x, 0.0) / self.width).astype(np.int64), self.bins - 1)
        flat = (idx + np.arange(self.cols)[None, :] * self.bins)[valid]
        self.h += np.bincount(flat, minlength=self.cols * self.bins).reshape(self.cols, self.bins)

    def quantile(self, q):
        cum = np.cumsum(self.h, axis=1)
        target = np.maximum(np.ceil(q * self.n), 1)
        idx = (cum >= target[:, None]).argmax(axis=1)
        val = np.minimum(np.maximum((idx + 0.5) * self.width, self.min), self.max)
        return np.where(self.n > 0, val, np.nan)

    def minimum(self):
        return np.where(self.n > 0, self.min, np.nan)

    def maximum(self):
        return np.where(self.n > 0, self.max, np.nan)


# --------------------------------------------------------------------------------------------------------------------- bones
def side_of(name):
    if name.startswith('l_') or name.startswith('Left'):
        return 'Left'
    if name.startswith('r_') or name.startswith('Right'):
        return 'Right'
    return None


class Layout:
    """Bone classification and driver map of one rig topology (from the file header)."""

    def __init__(self, header, cfg):
        self.names = [header.names[i] for i in header.mapped()]
        self.rigidx = [header.game_index[i] for i in header.mapped()]
        self.nm = len(self.names)
        self.cat = {n: k for k, n in enumerate(self.names)}
        rig_to_cat = {g: k for k, g in enumerate(self.rigidx)}
        parents = header.parents

        def ancestors(k):
            p, seen, out = parents[self.rigidx[k]] if self.rigidx[k] < len(parents) else -1, 0, []
            while p >= 0 and seen < 64:
                if p in rig_to_cat:
                    out.append(rig_to_cat[p])
                if p >= len(parents):
                    break
                p = parents[p]
                seen += 1
            return out

        self.core = [self.cat[n] for n in CORE_BONES if n in self.cat]
        self.missing_core = [n for n in CORE_BONES if n not in self.cat]
        self.helpers, self.why, self.driver_list = [], {}, {}
        core_set = set(self.core)
        for k, n in enumerate(self.names):
            if k in core_set or n in ('Hips', 'Spine', 'Root', 'Trajectory', 'reference_joint') or n.startswith(('LeftHand', 'RightHand', 'LeftInHand', 'RightInHand')):
                continue
            reason = None
            for a in ancestors(k):
                an = self.names[a]
                if an in STOPS:
                    break
                if an in ANCHORS:
                    reason = 'under ' + an
                    break
            if reason is None and HELPER_REGEX.search(n):
                reason = 'name'
            if reason is not None and not (n.startswith(('LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot')) or '_leg_' in n):
                self.helpers.append(k)
                self.why[k] = reason
        for k in self.helpers:
            n = self.names[k]
            side = side_of(n)
            drv = []
            anc = ancestors(k)
            if anc:
                drv.append(anc[0])
            if side:
                for dn in (side + 'Shoulder', side + 'Arm'):
                    if dn in self.cat:
                        drv.append(self.cat[dn])
            if 'Spine3' in self.cat:
                drv.append(self.cat['Spine3'])
            drv = [d for i, d in enumerate(drv) if d != k and d not in drv[:i]]
            lim = [cfg.arm_deg if self.names[d] in ARM_BONES else cfg.driver_deg for d in drv]
            self.driver_list[k] = (drv, np.array(lim))
        self.sel = self.core + self.helpers
        self.sel_pos = {k: i for i, k in enumerate(self.sel)}
        self.klass = ['core'] * len(self.core) + ['helper'] * len(self.helpers)
        self.keyword_counts = {w: sum(1 for n in self.names if w.lower() in n.lower()) for w in KEYWORDS}


# --------------------------------------------------------------------------------------------------------------------- analysis
class Result:
    pass


def _norm_rows(v):
    return np.linalg.norm(v, axis=-1)


def compute_rest(group, layout, cfg):
    """Per phase (stream, stage): per-bone median translation (cm) and quaternion of the whole recording (stride) or of cfg.rest_window."""
    samples = {}
    for stream, ticks, sec, gidx, v in iter_blocks(group):
        if cfg.rest_window:
            ix = np.nonzero((sec >= cfg.rest_window[0]) & (sec <= cfg.rest_window[1]))[0]
        else:
            ix = np.nonzero(gidx % cfg.rest_stride == 0)[0]
        if not len(ix):
            continue
        for st in range(v.shape[2]):
            d = samples.setdefault((stream, st), ([], []))
            d[0].append(v[ix][:, :, st, 0:3].astype(np.float64) * 100.0)
            d[1].append(normq(v[ix][:, :, st, 4:8].astype(np.float64)))
    if not samples:
        raise SystemExit('no frames to build the reference pose from (check --rest-from/--rest-to)')
    rest = {}
    for key, (ps, qs) in samples.items():
        p = np.concatenate(ps)
        q = np.concatenate(qs)
        sign = np.sign(np.sum(q * q[0][None], axis=-1, keepdims=True))
        q = q * np.where(sign == 0, 1.0, sign)
        rq = np.nanmedian(q, axis=0)
        rest[key] = (np.nanmedian(p, axis=0), normq(rq), p.shape[0])
    return rest


def analyze_group(group, cfg):
    h = group['header']
    lay = Layout(h, cfg)
    rest = compute_rest(group, lay, cfg)
    phases = []          # [(stream, stage, name)]
    for sid in sorted(h.streams):
        for st, name in enumerate(h.streams[sid]):
            phases.append((sid, st, name))
    phase_of = {(s, st): i for i, (s, st, _) in enumerate(phases)}
    ns = len(lay.sel)
    sel = np.array(lay.sel, dtype=int)
    R = Result()
    R.group, R.layout, R.phases, R.cfg, R.rest_samples = group, lay, [p[2] for p in phases], cfg, {phase_of[k]: v[2] for k, v in rest.items()}
    R.rot = [Hist(ns, ROT_BW, ROT_BINS) for _ in phases]
    R.tr = [Hist(ns, TR_BW, TR_BINS) for _ in phases]
    R.frames = [0] * len(phases)
    pairs = []
    if 0 in h.streams and len(h.streams[0]) == 2:
        pairs.append((phase_of[(0, 0)], phase_of[(0, 1)]))
    if 1 in h.streams and 0 in h.streams:
        pairs.append((phase_of[(0, 1)], phase_of[(1, 0)]))
    R.pairs = pairs
    R.pair_rot = {pr: Hist(ns, ROT_BW, ROT_BINS) for pr in pairs}
    R.pair_tr = {pr: Hist(ns, TR_BW, TR_BINS) for pr in pairs}
    nh = len(lay.helpers)
    R.hl = [{k: np.zeros(nh, dtype=np.int64) for k in ('jump', 'undriven', 'extreme', 'extreme_normal')} for _ in phases]
    R.worst_ext = [[None] * nh for _ in phases]
    R.worst_jump = [[None] * nh for _ in phases]
    R.t_first = None
    R.t_last = None
    buffers = {0: [], 1: []}        # for the stream-1 match: (ticks, pos cm [m, ns, 3], quat [m, ns, 4])
    prev = {}
    drv_idx = [np.array(lay.driver_list[k][0], dtype=int) for k in lay.helpers]
    drv_lim = [lay.driver_list[k][1] for k in lay.helpers]
    hel = np.array(lay.helpers, dtype=int)
    with np.errstate(invalid='ignore'):
        for stream, ticks, sec, gidx, v in iter_blocks(group):
            if cfg.window:
                ix = np.nonzero((sec >= cfg.window[0]) & (sec <= cfg.window[1]))[0]
            else:
                ix = np.arange(len(sec))
            if not len(ix):
                continue
            R.t_first = sec[ix[0]] if R.t_first is None else min(R.t_first, sec[ix[0]])
            R.t_last = sec[ix[-1]] if R.t_last is None else max(R.t_last, sec[ix[-1]])
            g = gidx[ix]
            vv = v[ix]
            m = len(ix)
            P = vv[..., 0:3].astype(np.float64) * 100.0
            Q = normq(vv[..., 4:8].astype(np.float64))
            for st in range(vv.shape[2]):
                ph = phase_of[(stream, st)]
                rp, rq, _ = rest[(stream, st)]
                p, q = P[:, :, st], Q[:, :, st]
                ang = qangle(q, rq[None])
                tr = _norm_rows(p - rp[None])
                pp, pq = np.empty_like(p), np.empty_like(q)
                pp[1:], pq[1:] = p[:-1], q[:-1]
                ok = np.zeros(m, dtype=bool)
                ok[1:] = (g[1:] - g[:-1]) == 1
                pr = prev.get(ph)
                if pr is not None and pr[0] == g[0] - 1:
                    pp[0], pq[0], ok[0] = pr[1], pr[2], True
                prev[ph] = (g[-1], p[-1].copy(), q[-1].copy())
                dang = qangle(q, pq)
                dtr = _norm_rows(p - pp)
                dang[~ok] = np.nan
                dtr[~ok] = np.nan
                R.frames[ph] += m
                R.rot[ph].add(ang[:, sel])
                R.tr[ph].add(tr[:, sel])
                for j in range(nh):
                    hi = hel[j]
                    di, dl = drv_idx[j], drv_lim[j]
                    jump = (dang[:, hi] >= cfg.jump_deg) | (dtr[:, hi] >= cfg.jump_cm)
                    dd = np.nan_to_num(dang[:, di], nan=0.0).max(axis=1) if len(di) else np.zeros(m)
                    undriven = jump & (dd < cfg.jump_deg / 4.0)
                    ext = (ang[:, hi] >= cfg.extreme_deg) | (tr[:, hi] >= cfg.extreme_cm)
                    normal = np.all(ang[:, di] < dl[None], axis=1) if len(di) else np.ones(m, dtype=bool)
                    extn = ext & normal
                    c = R.hl[ph]
                    c['jump'][j] += int(jump.sum())
                    c['undriven'][j] += int(undriven.sum())
                    c['extreme'][j] += int(ext.sum())
                    c['extreme_normal'][j] += int(extn.sum())
                    if extn.any():
                        score = np.where(extn, np.maximum(ang[:, hi] / cfg.extreme_deg, tr[:, hi] / cfg.extreme_cm), -1.0)
                        k = int(score.argmax())
                        if R.worst_ext[ph][j] is None or score[k] > R.worst_ext[ph][j][0]:
                            drv_txt = ' '.join('%s=%.1f' % (lay.names[d], ang[k, d]) for d in di)
                            R.worst_ext[ph][j] = (float(score[k]), 'at %s rot %.1f deg trans %.2f cm; drivers (deg from reference) %s' %
                                                  (clock(sec[ix[k]]), ang[k, hi], tr[k, hi], drv_txt))
                    if undriven.any():
                        score = np.where(undriven, np.maximum(dang[:, hi] / cfg.jump_deg, dtr[:, hi] / cfg.jump_cm), -1.0)
                        k = int(score.argmax())
                        if R.worst_jump[ph][j] is None or score[k] > R.worst_jump[ph][j][0]:
                            R.worst_jump[ph][j] = (float(score[k]), 'at %s jump %.1f deg %.2f cm; max driver step %.2f deg' %
                                                   (clock(sec[ix[k]]), dang[k, hi], dtr[k, hi], dd[k]))
            if vv.shape[2] == 2 and (0, 1) in R.pair_rot:
                R.pair_rot[(0, 1)].add(qangle(Q[:, sel, 0], Q[:, sel, 1]))
                R.pair_tr[(0, 1)].add(_norm_rows(P[:, sel, 0] - P[:, sel, 1]))
            if 1 in h.streams:
                if stream == 0:
                    buffers[0].append((ticks[ix], P[:, sel, 1].astype(np.float32), Q[:, sel, 1].astype(np.float32)))
                else:
                    buffers[1].append((ticks[ix], P[:, sel, 0].astype(np.float32), Q[:, sel, 0].astype(np.float32)))
    if buffers[0] and buffers[1]:
        ta = np.concatenate([b[0] for b in buffers[0]])
        pa = np.concatenate([b[1] for b in buffers[0]]).astype(np.float64)
        qa = np.concatenate([b[2] for b in buffers[0]]).astype(np.float64)
        tb = np.concatenate([b[0] for b in buffers[1]])
        pb = np.concatenate([b[1] for b in buffers[1]]).astype(np.float64)
        qb = np.concatenate([b[2] for b in buffers[1]]).astype(np.float64)
        order = np.argsort(tb, kind='stable')
        tb, pb, qb = tb[order], pb[order], qb[order]
        pos = np.clip(np.searchsorted(tb, ta), 1, len(tb) - 1)
        near = np.where(np.abs(tb[pos] - ta) < np.abs(tb[pos - 1] - ta), pos, pos - 1)
        good = np.abs(tb[near] - ta) <= TICK_TOLERANCE_S * h.qpc_hz
        R.matched = int(good.sum())
        if good.any():
            R.pair_rot[pairs[-1]].add(qangle(qa[good], qb[near[good]]))
            R.pair_tr[pairs[-1]].add(_norm_rows(pa[good] - pb[near[good]]))
    else:
        R.matched = 0
    return R


# --------------------------------------------------------------------------------------------------------------------- tables
def r3(x):
    return '' if x is None or (isinstance(x, float) and not np.isfinite(x)) else '%.3f' % x


def bones_rows(R):
    lay = R.layout
    rows = []
    for ph, name in enumerate(R.phases):
        rmin, rmed, r95, rmax = R.rot[ph].minimum(), R.rot[ph].quantile(0.5), R.rot[ph].quantile(0.95), R.rot[ph].maximum()
        tmed, t95, tmax = R.tr[ph].quantile(0.5), R.tr[ph].quantile(0.95), R.tr[ph].maximum()
        for i, k in enumerate(lay.sel):
            rows.append({'bone': lay.names[k], 'class': lay.klass[i], 'phase': name, 'frames': int(R.rot[ph].n[i]),
                         'rot_min_deg': r3(rmin[i]), 'rot_median_deg': r3(rmed[i]), 'rot_p95_deg': r3(r95[i]), 'rot_max_deg': r3(rmax[i]),
                         'trans_median_cm': r3(tmed[i]), 'trans_p95_cm': r3(t95[i]), 'trans_max_cm': r3(tmax[i])})
    return rows


def consistency_rows(R):
    lay, cfg = R.layout, R.cfg
    rows = []
    for ph, name in enumerate(R.phases):
        for j, k in enumerate(lay.helpers):
            c = R.hl[ph]
            flags = []
            if c['undriven'][j] >= cfg.min_jump_frames:
                flags.append('UNDRIVEN_JUMP')
            elif c['jump'][j] >= cfg.min_jump_frames:
                flags.append('JUMP')
            if c['extreme_normal'][j] >= cfg.min_extreme_frames:
                flags.append('EXTREME_WITH_NORMAL_DRIVERS')
            wj, we = R.worst_jump[ph][j], R.worst_ext[ph][j]
            rows.append({'bone': lay.names[k], 'phase': name, 'why_helper': lay.why[k],
                         'drivers': ' '.join(lay.names[d] for d in lay.driver_list[k][0]),
                         'jump_frames': int(c['jump'][j]), 'undriven_jump_frames': int(c['undriven'][j]), 'extreme_frames': int(c['extreme'][j]),
                         'extreme_with_normal_drivers_frames': int(c['extreme_normal'][j]),
                         'worst_undriven_jump': wj[1] if wj else '', 'worst_extreme': we[1] if we else '', 'flags': ' '.join(flags)})
    return rows


def phasediff_rows(R):
    lay = R.layout
    rows = []
    for pr in R.pairs:
        rr, tt = R.pair_rot[pr], R.pair_tr[pr]
        med, p95, mx, tm = rr.quantile(0.5), rr.quantile(0.95), rr.maximum(), tt.maximum()
        for i, k in enumerate(lay.sel):
            rows.append({'bone': lay.names[k], 'class': lay.klass[i], 'pair': '%s vs %s' % (R.phases[pr[0]], R.phases[pr[1]]), 'frames': int(rr.n[i]),
                         'rot_diff_median_deg': r3(med[i]), 'rot_diff_p95_deg': r3(p95[i]), 'rot_diff_max_deg': r3(mx[i]), 'trans_diff_max_cm': r3(tm[i])})
    return rows


def compare_rows(A, B):
    """Side by side per bone and phase name, sorted by |difference of rot p95| (then rot max, trans max)."""
    ra = {(r['bone'], r['phase']): r for r in bones_rows(A)}
    rb = {(r['bone'], r['phase']): r for r in bones_rows(B)}
    rows = []

    def f(x):
        return float(x) if x != '' else np.nan

    for key in ra:
        if key not in rb:
            continue
        a, b = ra[key], rb[key]
        d95 = f(a['rot_p95_deg']) - f(b['rot_p95_deg'])
        dmax = f(a['rot_max_deg']) - f(b['rot_max_deg'])
        dt = f(a['trans_max_cm']) - f(b['trans_max_cm'])
        rows.append({'bone': key[0], 'class': a['class'], 'phase': key[1],
                     'rot_p95_A': a['rot_p95_deg'], 'rot_p95_B': b['rot_p95_deg'], 'rot_p95_diff': r3(d95),
                     'rot_max_A': a['rot_max_deg'], 'rot_max_B': b['rot_max_deg'], 'rot_max_diff': r3(dmax),
                     'trans_max_A_cm': a['trans_max_cm'], 'trans_max_B_cm': b['trans_max_cm'], 'trans_max_diff_cm': r3(dt),
                     '_k': (abs(d95) if np.isfinite(d95) else -1, abs(dmax) if np.isfinite(dmax) else -1, abs(dt) if np.isfinite(dt) else -1)})
    rows.sort(key=lambda r: r['_k'], reverse=True)
    only_a = sorted({k[0] for k in ra} - {k[0] for k in rb})
    only_b = sorted({k[0] for k in rb} - {k[0] for k in ra})
    for r in rows:
        del r['_k']
    return rows, only_a, only_b


def write_csv(path, rows, header_lines=()):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        for line in header_lines:
            f.write('# ' + line + '\n')
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)


def describe(R, label):
    g, lay, cfg = R.group, R.layout, R.cfg
    h = g['header']
    first = os.path.basename(g['files'][0][0])
    lines = ['%s: %d file(s) from %s, rig %d bones, %d recorded (catalogue), local UTC offset %+.0f min (day %s)' %
             (label, len(g['files']), first, h.rig_count, lay.nm, g['offset'] / 60.0, g['day'])]
    win = 'whole file' if not cfg.window else '%s - %s local' % (clock(cfg.window[0]), clock(cfg.window[1]))
    span = '' if R.t_first is None else ', frames %s - %s local' % (clock(R.t_first), clock(R.t_last))
    lines.append('  window: %s%s' % (win, span))
    lines.append('  phases (frames in window): ' + ', '.join('%s=%d' % (n, R.frames[i]) for i, n in enumerate(R.phases)))
    if R.pairs and 1 in h.streams:
        lines.append('  stream-1 frames matched to stream 0 by ticks (<= 5 ms): %d' % R.matched)
    elif 1 not in h.streams:
        lines.append('  NOTE: this recording has no post_helper_apply_after_commit stream (only the pose before / after the mod writes it)')
    rs = ('window %s - %s' % (clock(cfg.rest_window[0]), clock(cfg.rest_window[1]))) if cfg.rest_window else \
        ('whole recording, every %d. frame' % cfg.rest_stride)
    lines.append('  reference pose: per-bone median of this recording (%s); the format holds no bind pose' % rs)
    lines.append('  core bones present: %s%s' % (' '.join(lay.names[k] for k in lay.core),
                                                ('; NOT recorded: ' + ' '.join(lay.missing_core)) if lay.missing_core else ''))
    lines.append('  helper bones (%d): ' % len(lay.helpers) + ' '.join('%s[%s]' % (lay.names[k], lay.why[k]) for k in lay.helpers))
    lines.append('  name keywords found (bones): ' + ', '.join('%s=%d' % (w, c) for w, c in lay.keyword_counts.items()) +
                 '; keywords with 0 are not recorded (only %d of the %d rig bones are in the catalogue)' % (lay.nm, h.rig_count))
    for n in g.get('notes', []):
        lines.append('  NOTE: ' + n)
    return lines


def top_lines(R, top):
    lay, cfg = R.layout, R.cfg
    out = []
    flagged = sorted((r for r in consistency_rows(R) if r['flags']),
                     key=lambda r: (-r['undriven_jump_frames'], -r['jump_frames'], -r['extreme_with_normal_drivers_frames']))
    out.append('  flagged helpers (%d rows; jump >= %.0f deg / %.1f cm per frame, extreme >= %.0f deg / %.1f cm with drivers < %.0f deg (arm < %.0f deg)):' %
               (len(flagged), cfg.jump_deg, cfg.jump_cm, cfg.extreme_deg, cfg.extreme_cm, cfg.driver_deg, cfg.arm_deg))
    for r in flagged[:max(top, 0) * 3]:
        out.append('    %s | %s | %s | jump %d (undriven %d), extreme %d (drivers normal %d) %s %s' %
                   (r['bone'], r['phase'], r['flags'], r['jump_frames'], r['undriven_jump_frames'], r['extreme_frames'],
                    r['extreme_with_normal_drivers_frames'], r['worst_extreme'], r['worst_undriven_jump']))
    if not flagged:
        out.append('    none')
    rows = [r for r in bones_rows(R) if r['class'] == 'helper' and r['rot_p95_deg'] != '']
    if rows:
        last = R.phases[1] if len(R.phases) > 1 else R.phases[0]
        sel = sorted((r for r in rows if r['phase'] == last), key=lambda r: -float(r['rot_p95_deg']))[:top]
        out.append('  helpers with the largest rotation p95 vs reference in %s:' % last)
        for r in sel:
            out.append('    %-40s rot min/med/p95/max %s/%s/%s/%s deg, trans med/p95/max %s/%s/%s cm' %
                       (r['bone'], r['rot_min_deg'], r['rot_median_deg'], r['rot_p95_deg'], r['rot_max_deg'],
                        r['trans_median_cm'], r['trans_p95_cm'], r['trans_max_cm']))
    pd = phasediff_rows(R)
    if pd:
        for pair in dict.fromkeys(r['pair'] for r in pd):
            sel = sorted((r for r in pd if r['pair'] == pair and r['rot_diff_max_deg'] != ''), key=lambda r: -float(r['rot_diff_max_deg']))[:top]
            out.append('  largest phase differences, %s:' % pair)
            for r in sel:
                out.append('    %-40s rot diff median/p95/max %s/%s/%s deg, trans diff max %s cm' %
                           (r['bone'], r['rot_diff_median_deg'], r['rot_diff_p95_deg'], r['rot_diff_max_deg'], r['trans_diff_max_cm']))
    return out


def run(inputs, out_dir, cfg, compare=None, compare_cfg=None, prefix='pose_helper_report', top=15, log=print):
    """Analyses the groups of `inputs` (and the first group of `compare`), writes the CSV / summary files; returns (results, compare_result)."""
    os.makedirs(out_dir, exist_ok=True)
    groups = load_groups(inputs, cfg)
    results = [analyze_group(g, cfg) for g in groups]
    cmp_result = None
    if compare:
        cgroups = load_groups(compare, compare_cfg or Config(tz_seconds=cfg.tz_seconds))
        cmp_result = analyze_group(cgroups[0], compare_cfg or Config(tz_seconds=cfg.tz_seconds))
    summary = []
    written = []
    for gi, R in enumerate(results):
        tag = '' if len(results) == 1 else '_g%d' % (gi + 1)
        head = ['reference pose = per-bone median of the recording itself (no bind pose in the format); window = %s' %
                ('whole file' if not cfg.window else '%s-%s local' % (clock(cfg.window[0]), clock(cfg.window[1]))),
                'files: ' + ' '.join(os.path.basename(p) for p, _, _ in R.group['files'])]
        for name, rows in (('bones', bones_rows(R)), ('consistency', consistency_rows(R)), ('phasediff', phasediff_rows(R))):
            p = os.path.join(out_dir, '%s%s_%s.csv' % (prefix, tag, name))
            write_csv(p, rows, head)
            written.append(p)
        summary += describe(R, 'recording' + (' (rig group %d)' % (gi + 1) if len(results) > 1 else ''))
        summary += top_lines(R, top)
    if cmp_result is not None:
        rows, only_a, only_b = compare_rows(results[0], cmp_result)
        p = os.path.join(out_dir, '%s_compare.csv' % prefix)
        write_csv(p, rows, ['A = ' + os.path.basename(results[0].group['files'][0][0]), 'B = ' + os.path.basename(cmp_result.group['files'][0][0])])
        written.append(p)
        summary += describe(cmp_result, 'compare recording B')
        summary.append('  compare: A = first recording, B = --compare; largest differences (A - B) by rotation p95:')
        for r in rows[:top]:
            summary.append('    %-40s %-22s rot p95 %s vs %s (%s), max %s vs %s (%s), trans max %s vs %s cm (%s)' %
                           (r['bone'], r['phase'], r['rot_p95_A'], r['rot_p95_B'], r['rot_p95_diff'], r['rot_max_A'], r['rot_max_B'], r['rot_max_diff'],
                            r['trans_max_A_cm'], r['trans_max_B_cm'], r['trans_max_diff_cm']))
        if only_a or only_b:
            summary.append('  bones only in A: %s; only in B: %s' % (' '.join(only_a) or '-', ' '.join(only_b) or '-'))
    p = os.path.join(out_dir, '%s_summary.txt' % prefix)
    pathlib.Path(p).write_text('\n'.join(summary) + '\n', encoding='utf-8')
    written.append(p)
    for line in summary:
        log(line)
    for w in written:
        log('wrote ' + w)
    return results, cmp_result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('inputs', nargs='+', help='.utpppose file(s), folder(s) or wildcard(s)')
    ap.add_argument('--out-dir', required=True, help='output folder (required; never next to the inputs)')
    ap.add_argument('--prefix', default='pose_helper_report')
    ap.add_argument('--from', dest='t_from', help='window start HH:MM:SS (local wall clock, as in the game logs)')
    ap.add_argument('--to', dest='t_to', help='window end HH:MM:SS')
    ap.add_argument('--compare', nargs='+', help='second recording (e.g. Female V) for the side-by-side table')
    ap.add_argument('--compare-from')
    ap.add_argument('--compare-to')
    ap.add_argument('--rest-from', help='reference pose window start HH:MM:SS (default: the whole recording)')
    ap.add_argument('--rest-to')
    ap.add_argument('--rest-stride', type=int, default=10)
    ap.add_argument('--utc', action='store_true', help='interpret --from/--to as UTC (default: local time from the file name stamp)')
    ap.add_argument('--tz-hours', type=float, help='explicit local-minus-UTC offset in hours')
    ap.add_argument('--jump-deg', type=float, default=25.0)
    ap.add_argument('--jump-cm', type=float, default=3.0)
    ap.add_argument('--extreme-deg', type=float, default=60.0)
    ap.add_argument('--extreme-cm', type=float, default=5.0)
    ap.add_argument('--driver-deg', type=float, default=35.0)
    ap.add_argument('--arm-deg', type=float, default=100.0)
    ap.add_argument('--min-jump-frames', type=int, default=1)
    ap.add_argument('--min-extreme-frames', type=int, default=3)
    ap.add_argument('--top', type=int, default=15)
    a = ap.parse_args(argv)
    tz = 0.0 if a.utc else (a.tz_hours * 3600.0 if a.tz_hours is not None else None)
    common = dict(tz_seconds=tz, rest_stride=a.rest_stride, jump_deg=a.jump_deg, jump_cm=a.jump_cm, extreme_deg=a.extreme_deg,
                  extreme_cm=a.extreme_cm, driver_deg=a.driver_deg, arm_deg=a.arm_deg, min_jump_frames=a.min_jump_frames,
                  min_extreme_frames=a.min_extreme_frames, rest_window=make_window(a.rest_from, a.rest_to) if (a.rest_from or a.rest_to) else None)
    cfg = Config(window=make_window(a.t_from, a.t_to), **common)
    ccfg = Config(window=make_window(a.compare_from, a.compare_to), **common) if a.compare else None
    run(a.inputs, a.out_dir, cfg, compare=a.compare, compare_cfg=ccfg, prefix=a.prefix, top=a.top)
    return 0


if __name__ == '__main__':
    sys.exit(main())
