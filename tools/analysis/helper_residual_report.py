"""Checks that no skin helper bone is stale: residual of the recorded helper bones against the deformer graph, per time window.

  python -I helper_residual_report.py INPUT [INPUT ...] --out-dir DIR [--bundle-dir DIR] [--plan plan.json | --window-seconds N]
                                      [--limit-pos-cm 0.05] [--limit-rot-deg 0.5] [--jobs N] [--detail summary|full]

INPUT is a .utpppose file, a folder or a wildcard (the segments of one capture are joined in time order); every INPUT argument is one
recording, analysed on its own. The recordings are read-only. Output goes to --out-dir only: <prefix>.json (all numbers), <prefix>_bones.csv
(per window and bone), <prefix>_summary.txt (also printed).

What is compared. The graph of the player's body deformer (the offline reference evaluator, here evaluated for all frames at once, see
helper_graph_batch.py) is evaluated on the recorded core pose of each frame, and every helper bone of the bundle (the bones the runtime
recompute owns; owned.txt of --bundle-dir, default Build\\test-data\\body_helper_<body>) is compared with the recorded value. Both are model
space transforms aligned at Spine3, so only the helper bone's own error remains. Per bone: position difference in cm and rotation difference
in degrees (angle between the two rotations, sign safe), per window p50 / p99 / max.

Modes (the helper values compared with the graph):
  post   stream 1 of the recording (post_helper_apply_after_commit): what the engine holds after the runtime recompute. This is the proof
         that the mod leaves no stale helper. It exists only when the recording was made with the post-helper stream (see the end of
         this text). The verdict is taken on frames where the mod changed the core pose.
  stale  stream 0, stage after_mm_and_aim: the helper values the engine computed from ITS pose, before the commit. On frames where the mod
         changed the core pose this is what the skin would show without the recompute (staleness); on frames without a mod write it is
         the graph-against-engine control. Informational, no verdict.

Frame classes. A frame is 'written' when the mod changed the core pose (the largest change of any core bone between stage before_mm and
stage after_mm_and_aim exceeds --write-threshold, default 0.002; sign safe for quaternions). Only written frames are gated: on frames the
mod did not touch, the post stream equals the engine's own helpers and any difference is the engine against the graph (reported
separately as 'not_written', never a verdict).

Verdict per window (post mode, written frames): PASS when every bundle bone found in the recording has p99 position <= --limit-pos-cm and
p99 rotation <= --limit-rot-deg; FAIL otherwise; NO_WRITES when the window has fewer than --min-written written frames. Overall status:
  PASS / FAIL            at least one gated window; FAIL if any window fails
  NO_MOD_WRITES          the mod changed no core pose (nothing to prove)
  NOT_VERIFIABLE         the recording has no post stream (or no usable frame): nothing is proved; stale numbers are still reported
Exit code: 0 PASS or NO_MOD_WRITES, 1 FAIL, 2 NOT_VERIFIABLE or unreadable input.
The bundle bones the recording lacks (not in its bone catalogue) and the ones whose ancestor chain is not fully recorded are listed per
recording (`unverified`); they are not part of any verdict.

Windows: --plan reads the plan.json of an in-game harness run (runs[].steps[] with start / end wall-clock times; the window label is
<scenario>#<run>:<step>; --clock-offset = log time minus plan time in seconds, as in the harness) or --window-seconds N makes fixed windows;
the whole recording is always a window ('all').

How the recorder gets stream 1: the DLL writes the post stream only when (1) skin_helpers\\mode.txt in the plugin folder says APPLY and
(2) the empty marker file post-helper-trace.txt exists in red4ext\\logs when the recording starts. The Diagnostic logs list of the mod does
not create that file.
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import re
import sys
import time
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import pose_helper_report as ph  # noqa: E402
import helper_graph_batch as hgb  # noqa: E402
from export_player_helper_bundle import fnv64  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[3]
BODY_OF_RIG = {619: 'man', 645: 'woman'}
DEFAULT_POS_CM = 0.05
DEFAULT_ROT_DEG = 0.5
WRITE_THRESHOLD = 0.002
CHUNK = 2048
TICK_TOLERANCE_S = 0.005
CORE_GAP_LIMIT = 1e-3                      # a post-stream core pose further than this from the evaluated pose: the core changed after the mod's write
GROUP_ORDER = ['upper', 'lower', 'elbow', 'wrist_muscle', 'wrist_roll', 'lower_leg_back', 'hand', 'sterno', 'other']


# --------------------------------------------------------------------------------------------------------------------- bundle
def helper_group(name):
    """Anatomical group of a helper bone by name (case sensitive: wrist_A_mscl is a muscle, Wrist_0_JNT is the wrist roll bone)."""
    if 'sternocleidomastoid' in name:
        return 'sterno'
    if 'Wrist_' in name:
        return 'wrist_roll'
    if 'lowLeg_back' in name:
        return 'lower_leg_back'
    if any(k in name for k in ('knuckle', 'thumb', 'pinkyBase')):
        return 'hand'
    if 'elbow' in name:
        return 'elbow'
    if 'wrist_' in name:
        return 'wrist_muscle'
    if any(k in name for k in ('calf', 'knee', 'leg_back', 'leg_groin', 'leg_buttock', 'thigh')):
        return 'lower'
    return 'upper'


def default_bundle_dir(body):
    return str(ROOT / 'Build' / 'test-data' / ('body_helper_' + body))


def read_owned(bundle_dir, rig_names=None):
    """Names of the helper bones the bundle owns: owned.txt of the folder, else (an installed bundle folder) the OWNED hashes of its
    binding.dat matched against the rig bone names `rig_names` (a hash that matches no rig bone is an error)."""
    path = os.path.join(bundle_dir, 'owned.txt')
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            return [x.strip() for x in f if x.strip() and not x.startswith('#')]
    binding = os.path.join(bundle_dir, 'binding.dat')
    if not os.path.isfile(binding):
        raise FileNotFoundError('%s has neither owned.txt nor binding.dat' % bundle_dir)
    if rig_names is None:
        raise ValueError('the rig bone names are needed to read the owned bones of binding.dat')
    by_hash = {fnv64(n): n for n in rig_names}
    out, unknown = [], []
    with open(binding, encoding='ascii') as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 2 and parts[0] == 'OWNED':
                h = int(parts[1], 16)
                if h in by_hash:
                    out.append(by_hash[h])
                else:
                    unknown.append(parts[1])
    if unknown:
        raise ValueError('binding.dat owns %d bone(s) that are not in the deformation rig: %s' % (len(unknown), ' '.join(unknown[:4])))
    return out


def bundle_info(bundle_dir, body, owned):
    """Identity of the bundle against the graph files this tool evaluates (the exporter's identity hash), so a report says whether it
    judges the same graph the bundle was built from."""
    import evaluate_body_deformer_graph as E
    info = {'dir': os.path.relpath(bundle_dir, ROOT) if str(bundle_dir).startswith(str(ROOT)) else os.path.basename(str(bundle_dir)),
            'owned': len(owned), 'owned_from': 'owned.txt' if os.path.isfile(os.path.join(bundle_dir, 'owned.txt')) else 'binding.dat'}
    for key, fname in (('binding_file_sha256', 'binding.dat'), ('program_file_sha256', 'program.ir')):
        fp = os.path.join(bundle_dir, fname)
        if os.path.isfile(fp):
            info[key] = hashlib.sha256(pathlib.Path(fp).read_bytes()).hexdigest()
    manifest = os.path.join(bundle_dir, 'bundle_manifest.json')
    try:
        graph, rig, core = E.BODY_FILES[body]
        h = hashlib.sha256()
        for p in (graph, rig, core):
            h.update(pathlib.Path(p).read_bytes())
        h.update('\n'.join(sorted(owned)).encode())
        mine = h.hexdigest()
        info['graph_identity'] = mine
        if os.path.isfile(manifest):
            with open(manifest, encoding='utf-8') as f:
                m = json.load(f)
            info['bundle_identity'] = m.get('identity')
            info['program_sha256'] = m.get('program_sha256')
            info['graph_matches_bundle'] = (m.get('identity') == mine)
    except OSError as e:
        info['identity_error'] = str(e)
    return info


# --------------------------------------------------------------------------------------------------------------------- recording
class Recording:
    """One recording (one rig topology) in memory: stream 0 [n, bones, 2, 12], stream 1 matched to the frames of stream 0."""

    def __init__(self, group):
        h = group['header']
        self.header, self.group = h, group
        self.path = group['files'][0][0]
        self.files = [os.path.basename(p) for p, _, _ in group['files']]
        self.body = BODY_OF_RIG.get(h.rig_count)
        self.has_post = 1 in h.streams
        self.day, self.offset = group.get('day', ''), group.get('offset', 0.0)
        self.names = [h.names[i] for i in h.mapped()]
        s0, s1 = [], []
        for stream, ticks, sec, gidx, v in ph.iter_blocks(group):
            (s0 if stream == 0 else s1).append((ticks, sec, v))
        self.notes = list(group.get('notes', []))
        self.post_frames = 0
        if not s0:
            self.n = 0
            self.ticks = self.sec = np.zeros(0)
            self.V0 = np.zeros((0, len(self.names), 2, 12), np.float32)
            self.V1 = None
            self.post_of = np.zeros(0, dtype=np.int64)
            return
        self.ticks = np.concatenate([a for a, _, _ in s0]).astype(np.float64)
        self.sec = np.concatenate([b for _, b, _ in s0])
        self.V0 = np.concatenate([c for _, _, c in s0]).astype(np.float32, copy=False)
        self.n = len(self.ticks)
        self.V1 = None
        self.post_of = np.full(self.n, -1, dtype=np.int64)
        if s1:
            t1 = np.concatenate([a for a, _, _ in s1]).astype(np.float64)
            self.V1 = np.concatenate([c for _, _, c in s1]).astype(np.float32, copy=False)[:, :, 0, :]
            self.post_frames = len(t1)
            order = np.argsort(t1, kind='stable')
            t1s = t1[order]
            pos = np.clip(np.searchsorted(t1s, self.ticks), 1, max(len(t1s) - 1, 1))
            lo = np.clip(pos - 1, 0, len(t1s) - 1)
            near = np.where(np.abs(t1s[np.clip(pos, 0, len(t1s) - 1)] - self.ticks) < np.abs(t1s[lo] - self.ticks), np.clip(pos, 0, len(t1s) - 1), lo)
            good = np.abs(t1s[near] - self.ticks) <= TICK_TOLERANCE_S * h.qpc_hz
            self.post_of = np.where(good, order[near], -1)

    @property
    def matched(self):
        return int((self.post_of >= 0).sum())


# --------------------------------------------------------------------------------------------------------------------- per recording engine
_EVALUATORS = {}


def evaluator_for(body):
    if body not in _EVALUATORS:
        _EVALUATORS[body] = hgb.BatchEvaluator(body)
    return _EVALUATORS[body]


class Engine:
    """Residuals of the helper bones of one recording, frame chunk by frame chunk. The bundle bones (`owned`) are gated; with extra=True
    every other recorded helper bone of the rig is evaluated too (reported, never gated: they show what the recompute does not cover)."""

    def __init__(self, rec, owned, extra=True):
        h = rec.header
        self.rec, self.owned = rec, list(owned)
        self.ev = evaluator_for(rec.body)
        self.names = rec.names
        self.nidx = {n: i for i, n in enumerate(self.names)}
        self.parents = [int(p) for p in h.parents]
        merged = {int(x): i for i, x in enumerate(h.hashes)}
        self.rig_of = [int(h.game_index[i]) for i in h.mapped()]               # catalogue bone -> rig index
        self.rec_rig = {r: b for b, r in enumerate(self.rig_of)}               # rig index -> catalogue bone
        self.order = sorted(self.rec_rig)                                        # rig order: parents come first
        self.core_rec = [n for n in self.names if n in self.ev.core_names]
        self.missing_core = [n for n in self.ev.rig_core if n not in self.nidx]
        self.core_idx = np.array([self.nidx[n] for n in self.core_rec], dtype=np.int64)
        self.sp3 = merged.get(fnv64('Spine3'))
        # resolve every owned bone: rig index in the recording, then whether the chain of ancestors is recorded
        self.lacking, self.unresolved = [], []
        self.bones = []                                                          # evaluated helper bones: owned first (gated), then the others
        self.extra_unresolved = []
        owned_set = set(self.owned)

        def resolve(n):
            """(rig index, '') when bone n is recorded with all its ancestors, else (None, 'lacking' | 'chain')"""
            i = merged.get(fnv64(n))
            if i is None or i not in self.rec_rig or n not in self.ev.names:
                return None, 'lacking'
            p = self.parents[i]
            while p >= 0:
                if p not in self.rec_rig:
                    return None, 'chain'
                p = self.parents[p]
            return i, ''

        for n in self.owned:
            i, why = resolve(n)
            if i is not None:
                self.bones.append((n, i))
            else:
                (self.lacking if why == 'lacking' else self.unresolved).append(n)
        self.n_gated = len(self.bones)
        if extra:
            for n in self.ev.helper_names:
                if n in owned_set:
                    continue
                i, why = resolve(n)
                if i is not None:
                    self.bones.append((n, i))
                elif why == 'chain':
                    self.extra_unresolved.append(n)
        self.bone_names = [b[0] for b in self.bones]
        self.bone_rig = [b[1] for b in self.bones]
        self.gated = np.arange(len(self.bones)) < self.n_gated
        self.sp3_ok = self.sp3 is not None and self.sp3 in self.rec_rig

    def world(self, vals):
        """vals [m, bones, 12] -> {rig index: [m,4,4] model matrix} for every recorded bone whose ancestors are all recorded."""
        Ml = hgb.quat_matrices(vals)
        W = {}
        for i in self.order:
            p = self.parents[i]
            if p >= 0 and p not in W:
                continue
            W[i] = Ml[:, self.rec_rig[i]] if p < 0 else np.matmul(W[p], Ml[:, self.rec_rig[i]])
        return W

    def run(self):
        """-> dict of per-frame arrays: written [n], bad [n], pos_stale / rot_stale [n,k], pos_post / rot_post [n,k] (nan where absent),
        core_gap_post [n] (largest core difference between the stage after_mm_and_aim and the post stream)."""
        rec = self.rec
        n, k = rec.n, len(self.bones)
        out = dict(written=np.zeros(n, bool), bad=np.zeros(n, bool), nonfinite=np.zeros(n, bool),
                   pos_stale=np.full((n, k), np.nan, np.float32), rot_stale=np.full((n, k), np.nan, np.float32),
                   pos_post=np.full((n, k), np.nan, np.float32), rot_post=np.full((n, k), np.nan, np.float32),
                   core_gap_post=np.full(n, np.nan, np.float32), write_size=np.zeros(n, np.float32))
        if not k or self.missing_core or not self.sp3_ok:
            return out
        want = self.bone_names + ['Spine3']
        rig_idx = np.array(self.bone_rig)
        ci = self.core_idx
        used = [0, 1, 2, 4, 5, 6, 7, 8, 9, 10]                                   # translation, quaternion, scale (columns 3 and 11 are padding)
        with np.errstate(all='ignore'):
            for a in range(0, n, CHUNK):
                b = min(n, a + CHUNK)
                V = rec.V0[a:b].astype(np.float64)
                core0, core1 = V[:, ci, 0, :], V[:, ci, 1, :]
                dq = np.minimum(np.abs(core1[..., 4:8] - core0[..., 4:8]).max(-1), np.abs(core1[..., 4:8] + core0[..., 4:8]).max(-1))
                dt = np.abs(core1[..., 0:3] - core0[..., 0:3]).max(-1)
                out['write_size'][a:b] = np.nanmax(np.maximum(dq, dt), axis=1)
                vals1 = V[:, :, 1, :]
                finite = np.isfinite(core1[:, :, used]).all(axis=(1, 2))
                core_locals = {nm: hgb.quat_matrices(vals1[:, self.nidx[nm], :]) for nm in self.core_rec}
                models, bad = self.ev.run(core_locals, want=want)
                out['bad'][a:b] = bad
                out['nonfinite'][a:b] = ~finite
                W1 = self.world(vals1)
                if self.sp3 not in W1:
                    continue
                ok = finite & ~bad
                M_sp3_inv = np.linalg.inv(np.where(ok[:, None, None], models['Spine3'], np.eye(4)))
                M_owned = np.stack([models[nm] for nm in self.bone_names], axis=1)               # [m,k,4,4]

                def residual(W):
                    xfer = np.matmul(W[self.sp3], M_sp3_inv)
                    want_m = np.matmul(xfer[:, None], M_owned)
                    got = np.stack([W[i] for i in self.bone_rig], axis=1)
                    return np.linalg.norm(want_m[..., :3, 3] - got[..., :3, 3], axis=-1) * 100.0, hgb.rotation_error_deg(want_m, got)

                pos, rot = residual(W1)
                pos[~ok] = np.nan
                rot[~ok] = np.nan
                out['pos_stale'][a:b], out['rot_stale'][a:b] = pos, rot
                if rec.V1 is not None:
                    pi = rec.post_of[a:b]
                    has = pi >= 0
                    if has.any():
                        Vp = np.zeros((b - a, len(self.names), 12))
                        Vp[has] = rec.V1[pi[has]].astype(np.float64)
                        Wp = self.world(Vp)
                        if self.sp3 in Wp:
                            pos, rot = residual(Wp)
                            sel = has & ok
                            pos[~sel] = np.nan
                            rot[~sel] = np.nan
                            out['pos_post'][a:b], out['rot_post'][a:b] = pos, rot
                            # the core pose of the post stream must equal the stage the graph was evaluated on (quaternions compared sign safe)
                            qd = np.minimum(np.abs(Vp[:, ci, 4:8] - core1[..., 4:8]).max(-1), np.abs(Vp[:, ci, 4:8] + core1[..., 4:8]).max(-1))
                            td = np.abs(Vp[:, ci, 0:3] - core1[..., 0:3]).max(-1)
                            g = np.nanmax(np.maximum(qd, td), axis=1)
                            g[~has] = np.nan
                            out['core_gap_post'][a:b] = g
        out['written'] = out['write_size'] > WRITE_THRESHOLD
        return out


# --------------------------------------------------------------------------------------------------------------------- windows
def parse_clock(text, day, clock_offset=0.0):
    """'YYYY-MM-DD HH:MM:SS[.ffffff]' -> seconds since local midnight of `day` (+ clock offset)."""
    t = datetime.datetime.strptime(text, '%Y-%m-%d %H:%M:%S.%f' if '.' in text else '%Y-%m-%d %H:%M:%S')
    d0 = datetime.datetime.strptime(day, '%Y-%m-%d')
    return (t - d0).total_seconds() + clock_offset


def windows_from_plan(plan_path, day, clock_offset=0.0):
    with open(plan_path, encoding='utf-8') as f:
        plan = json.load(f)
    out = []
    for run in plan.get('runs', []):
        for st in run.get('steps', []):
            if not st.get('start') or not st.get('end'):
                continue
            out.append(('%s#%s:%s' % (run.get('scenario'), run.get('run'), st.get('name')),
                        parse_clock(st['start'], day, clock_offset), parse_clock(st['end'], day, clock_offset)))
    return out


def fixed_windows(sec, seconds):
    if not len(sec) or seconds <= 0:
        return []
    t0, t1 = float(sec[0]), float(sec[-1])
    out, a, k = [], t0, 0
    while a <= t1:
        out.append(('t%04d-%s' % (int(a - t0), clock(a)), a, min(a + seconds, t1 + 1e-6)))
        a += seconds
        k += 1
    return out


def clock(sec):
    s = float(sec) % 86400.0
    return '%02d:%02d:%06.3f' % (int(s // 3600), int(s % 3600 // 60), s % 60)


# --------------------------------------------------------------------------------------------------------------------- statistics
def _quant(x, mask):
    """p50 / p99 / max per column over the masked rows; nan columns stay nan."""
    sub = x[mask]
    k = x.shape[1]
    if sub.shape[0] == 0:
        return np.full((3, k), np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        q = np.nanpercentile(sub.astype(np.float64), [50, 99], axis=0)
        return np.vstack([q, np.nanmax(sub, axis=0)])


def _bone_stats(pos, rot, mask, names, gated):
    qp, qr = _quant(pos, mask), _quant(rot, mask)
    cnt = np.isfinite(pos[mask]).sum(axis=0) if mask.any() else np.zeros(len(names), int)
    return {nm: dict(n=int(cnt[i]), gated=bool(nm in gated), pos_cm=[_r(qp[0, i]), _r(qp[1, i]), _r(qp[2, i])],
                     rot_deg=[_r(qr[0, i]), _r(qr[1, i]), _r(qr[2, i])]) for i, nm in enumerate(names)}


def _r(x):
    return None if x is None or not np.isfinite(x) else round(float(x), 5)


def _group_summary(bstats, limits):
    """Per group: statistics over its measured bones; `covered` = the group has bundle (gated) bones; `failing` lists gated bones over
    a limit; `ok` is None for a group without bundle bones (reported, never judged)."""
    out = {}
    for nm, s in bstats.items():
        g = helper_group(nm)
        o = out.setdefault(g, dict(bones=0, gated_bones=0, measured=0, p99_pos_cm=0.0, p99_rot_deg=0.0, max_pos_cm=0.0, max_rot_deg=0.0,
                                   p50_pos_cm=0.0, worst_bone=None, failing=[]))
        o['bones'] += 1
        o['gated_bones'] += int(s['gated'])
        if s['n'] == 0:
            continue
        o['measured'] += 1
        o['p50_pos_cm'] = max(o['p50_pos_cm'], s['pos_cm'][0])
        if s['pos_cm'][1] >= o['p99_pos_cm']:
            o['worst_bone'] = nm
        o['p99_pos_cm'] = max(o['p99_pos_cm'], s['pos_cm'][1])
        o['p99_rot_deg'] = max(o['p99_rot_deg'], s['rot_deg'][1])
        o['max_pos_cm'] = max(o['max_pos_cm'], s['pos_cm'][2])
        o['max_rot_deg'] = max(o['max_rot_deg'], s['rot_deg'][2])
        if s['gated'] and (s['pos_cm'][1] > limits['pos_cm'] or s['rot_deg'][1] > limits['rot_deg']):
            o['failing'].append(nm)
    for o in out.values():
        for key in ('p50_pos_cm', 'p99_pos_cm', 'p99_rot_deg', 'max_pos_cm', 'max_rot_deg'):
            o[key] = round(o[key], 5)
        o['covered'] = o['gated_bones'] > 0
        o['ok'] = ((not o['failing']) and o['measured'] > 0) if o['covered'] else None
    return {g: out[g] for g in GROUP_ORDER if g in out}


def _worst_frames(pos, rot, idx, sec, names, limits, top):
    """Frames of `idx` (frame indices) with the largest excess over the limits: [{frame, clock, bone, pos_cm, rot_deg}]."""
    if not len(idx) or top <= 0:
        return []
    ps, rs = pos[idx].astype(np.float64), rot[idx].astype(np.float64)
    score = np.fmax(ps / limits['pos_cm'], rs / limits['rot_deg'])
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        per_frame = np.where(np.isfinite(score).any(axis=1), np.nanmax(np.where(np.isfinite(score), score, -1.0), axis=1), -1.0)
    order = np.argsort(-per_frame)[:top]
    out = []
    for j in order:
        if per_frame[j] < 0:
            continue
        b = int(np.nanargmax(np.where(np.isfinite(score[j]), score[j], -1.0)))
        out.append(dict(frame=int(idx[j]), clock=clock(sec[idx[j]]), bone=names[b], pos_cm=_r(ps[j, b]), rot_deg=_r(rs[j, b]), excess=_r(per_frame[j])))
    return out


def window_report(arr, rec, names, gated, lo, hi, limits, min_written, top, detail):
    sec = rec.sec
    in_win = (sec >= lo) & (sec < hi) if hi is not None else np.ones(len(sec), bool)
    usable = in_win & ~arr['bad'] & ~arr['nonfinite']
    written = usable & arr['written']
    unwritten = usable & ~arr['written']
    rep = dict(start=_r(lo) if lo is not None else None, end=_r(hi) if hi is not None else None,
               start_clock=clock(lo) if lo is not None else None, end_clock=clock(hi) if hi is not None else None,
               frames=int(in_win.sum()), usable=int(usable.sum()), written=int(written.sum()), not_written=int(unwritten.sum()),
               evaluator_refused=int((in_win & arr['bad']).sum()), nonfinite=int((in_win & arr['nonfinite']).sum()))
    for mode, pk, rk in (('stale', 'pos_stale', 'rot_stale'), ('post', 'pos_post', 'rot_post')):
        if mode == 'post' and not np.isfinite(arr[pk]).any():
            continue
        block = {}
        for cls, mask in (('written', written), ('not_written', unwritten)):
            if mode == 'post':
                mask = mask & np.isfinite(arr[pk]).any(axis=1)
            bs = _bone_stats(arr[pk], arr[rk], mask, names, gated)
            entry = dict(frames=int(mask.sum()), groups=_group_summary(bs, limits))
            if detail == 'full' or lo is None:
                entry['bones'] = bs
            idx = np.nonzero(mask)[0]
            entry['worst_frames'] = _worst_frames(arr[pk], arr[rk], idx, sec, names, limits, top if cls == 'written' else min(top, 3))
            block[cls] = entry
        rep[mode] = block
    if 'post' in rep:
        w = rep['post']['written']
        if w['frames'] < min_written:
            rep['verdict'] = 'NO_WRITES'
        else:
            bad_groups = [g for g, o in w['groups'].items() if o['covered'] and not o['ok'] and o['measured'] > 0]
            rep['verdict'] = 'FAIL' if bad_groups else 'PASS'
            rep['failing_groups'] = bad_groups
    else:
        rep['verdict'] = 'NOT_VERIFIABLE'
    return rep


# --------------------------------------------------------------------------------------------------------------------- one recording
def analyze_recording(args_dict, group):
    """-> dict (JSON ready) for one recording group."""
    t_start = time.time()
    a = args_dict
    limits = dict(pos_cm=a['limit_pos_cm'], rot_deg=a['limit_rot_deg'])
    rec = Recording(group)
    res = dict(file=os.path.basename(rec.path), files=rec.files, body=rec.body, rig_count=rec.header.rig_count, day=rec.day,
               frames=rec.n, post_frames=getattr(rec, 'post_frames', 0), post_matched=rec.matched if rec.n else 0,
               duration_s=round(float(rec.sec[-1] - rec.sec[0]), 2) if rec.n else 0.0, limits=limits, write_threshold=WRITE_THRESHOLD, notes=rec.notes)
    if rec.body is None or rec.n == 0:
        res.update(status='NOT_VERIFIABLE', reason='unsupported rig (%d bones)' % rec.header.rig_count if rec.body is None else 'no frames')
        return res
    bundle_dir = a['bundle_dir'] or default_bundle_dir(rec.body)
    owned = read_owned(bundle_dir, evaluator_for(rec.body).names)
    res['bundle'] = bundle_info(bundle_dir, rec.body, owned)
    eng = Engine(rec, owned, extra=a['extra_helpers'])
    res['bundle'].update(recorded=eng.n_gated, extra_helpers_evaluated=len(eng.bones) - eng.n_gated,
                         unverified=dict(lacking=eng.lacking, unresolved_chain=eng.unresolved))
    if eng.missing_core or not eng.sp3_ok:
        res.update(status='NOT_VERIFIABLE', reason='the recording lacks core bones needed by the graph: %s' % (', '.join(eng.missing_core) or 'Spine3'))
        return res
    arr = eng.run()
    res['evaluation'] = dict(seconds=round(time.time() - t_start, 2), evaluator_refused_frames=int(arr['bad'].sum()),
                             nonfinite_frames=int(arr['nonfinite'].sum()), written_frames=int(arr['written'].sum()),
                             max_core_gap_post=_r(np.nanmax(arr['core_gap_post'])) if np.isfinite(arr['core_gap_post']).any() else None,
                             core_gap_frames=int((arr['core_gap_post'] > CORE_GAP_LIMIT).sum()))
    names = eng.bone_names
    gated = set(eng.bone_names[:eng.n_gated])
    wins = [('all', None, None)]
    if a['plan']:
        wins += windows_from_plan(a['plan'], rec.day, a['clock_offset'])
    elif a['window_seconds']:
        wins += fixed_windows(rec.sec, a['window_seconds'])
    res['windows'] = []
    for label, lo, hi in wins:
        rep = window_report(arr, rec, names, gated, lo, hi, limits, a['min_written'], a['worst'], a['detail'])
        rep['label'] = label
        if label != 'all' and rep['frames'] == 0:
            rep['verdict'] = 'OUTSIDE_RECORDING'
        res['windows'].append(rep)
    allw = res['windows'][0]
    verdicts = [w['verdict'] for w in res['windows']]
    if not rec.has_post or rec.matched == 0:
        res['status'] = 'NOT_VERIFIABLE'
        res['reason'] = 'the recording has no post_helper_apply_after_commit stream' if not rec.has_post else 'no post frame matches a pose frame'
    elif allw['written'] == 0:
        res['status'] = 'NO_MOD_WRITES'
    elif 'FAIL' in verdicts:
        res['status'] = 'FAIL'
    elif 'PASS' in verdicts:
        res['status'] = 'PASS'
    else:
        res['status'] = 'NO_MOD_WRITES'
    res['evaluation']['total_seconds'] = round(time.time() - t_start, 2)
    return res


def _worker(task):
    args_dict, path = task
    out = []
    groups = ph.load_groups([path], ph.Config())
    for g in groups:
        try:
            out.append(analyze_recording(args_dict, g))
        except Exception as e:                                       # noqa: BLE001 - one unreadable recording must not stop the others
            out.append(dict(file=os.path.basename(g['files'][0][0]), status='NOT_VERIFIABLE', reason='%s: %s' % (type(e).__name__, e)))
    return out


# --------------------------------------------------------------------------------------------------------------------- output
def summary_lines(results):
    lines = []
    for r in results:
        lines.append('%s  %s  status %s%s' % (r.get('file'), r.get('body') or '?', r.get('status'), ('  (%s)' % r['reason']) if r.get('reason') else ''))
        if 'frames' not in r:
            continue
        ev = r.get('evaluation', {})
        lines.append('  frames %d, post frames %d (matched %d), %.0f s, written frames %s, evaluator refused %s, non-finite %s, evaluated in %s s' % (
            r['frames'], r['post_frames'], r['post_matched'], r['duration_s'], ev.get('written_frames'), ev.get('evaluator_refused_frames'),
            ev.get('nonfinite_frames'), ev.get('total_seconds')))
        if ev.get('core_gap_frames'):
            lines.append('  NOTE: in %d post frames the core pose differs from the evaluated pose by more than %g (largest %s): the core changed after the mod\'s '
                         'write, so a residual there is not only a stale commit' % (ev['core_gap_frames'], CORE_GAP_LIMIT, ev.get('max_core_gap_post')))
        b = r.get('bundle')
        if b:
            lines.append('  bundle %s (%s): %d owned bones, %d gated, %s other helper bones reported only; graph matches bundle: %s; lacking from the recording: %s; '
                         'ancestor chain not recorded: %s' % (
                             b.get('dir'), b.get('owned_from'), b.get('owned', 0), b.get('recorded', 0), b.get('extra_helpers_evaluated', 0),
                             b.get('graph_matches_bundle'), ', '.join(b['unverified']['lacking']) or '-',
                             ', '.join(b['unverified']['unresolved_chain']) or '-'))
        for w in r.get('windows', []):
            if w['verdict'] == 'OUTSIDE_RECORDING':
                lines.append('  [%s] outside the recording' % w['label'])
                continue
            line = '  [%s] %s frames %d (written %d, not written %d)' % (w['label'], w['verdict'], w['frames'], w['written'], w['not_written'])
            lines.append(line)
            for mode in ('post', 'stale'):
                if mode not in w:
                    continue
                wr = w[mode]['written']
                show = lambda o: o['measured'] and (o['covered'] or w['label'] == 'all')                  # noqa: E731
                tag = lambda o: '' if o['covered'] else ' (not in bundle)'                               # noqa: E731
                if wr['frames']:
                    lines.append('      %-5s written    : ' % mode + '; '.join('%s p99 %.3f cm / %.2f deg, max %.3f cm%s%s' % (
                        g, o['p99_pos_cm'], o['p99_rot_deg'], o['max_pos_cm'], tag(o), ' ***' if o['covered'] and not o['ok'] else '')
                        for g, o in wr['groups'].items() if show(o)))
                nw = w[mode]['not_written']
                if nw['frames'] and w['label'] == 'all':
                    lines.append('      %-5s not written: ' % mode + '; '.join('%s p99 %.3f cm / %.2f deg%s' % (g, o['p99_pos_cm'], o['p99_rot_deg'], tag(o))
                                                                              for g, o in nw['groups'].items() if show(o)))
            if w['verdict'] == 'FAIL':
                wf = w['post']['written']['worst_frames'][:3]
                lines.append('      worst: ' + '; '.join('frame %d %s %s %.3f cm %.2f deg' % (f['frame'], f['clock'], f['bone'], f['pos_cm'], f['rot_deg']) for f in wf))
    return lines


def write_outputs(results, out_dir, prefix):
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    p = os.path.join(out_dir, prefix + '.json')
    with open(p, 'w', encoding='utf-8') as f:
        json.dump(dict(tool='helper_residual_report.py', recordings=results), f, indent=1)
    paths.append(p)
    p = os.path.join(out_dir, prefix + '_bones.csv')
    with open(p, 'w', encoding='utf-8', newline='') as f:
        f.write('recording,window,mode,class,bone,group,in_bundle,frames,pos_p50_cm,pos_p99_cm,pos_max_cm,rot_p50_deg,rot_p99_deg,rot_max_deg\n')
        for r in results:
            for w in r.get('windows', []):
                for mode in ('post', 'stale'):
                    for cls in ('written', 'not_written'):
                        for nm, s in w.get(mode, {}).get(cls, {}).get('bones', {}).items():
                            if s['n']:
                                f.write('%s,%s,%s,%s,%s,%s,%d,%d,%s,%s,%s,%s,%s,%s\n' % (r['file'], w['label'], mode, cls, nm, helper_group(nm), int(s['gated']),
                                                                                      s['n'], *s['pos_cm'], *s['rot_deg']))
    paths.append(p)
    p = os.path.join(out_dir, prefix + '_summary.txt')
    text = '\n'.join(summary_lines(results)) + '\n'
    pathlib.Path(p).write_text(text, encoding='utf-8')
    paths.append(p)
    return paths, text


def exit_code(results):
    st = [r.get('status') for r in results]
    if any(s == 'FAIL' for s in st):
        return 1
    if any(s == 'NOT_VERIFIABLE' for s in st) or not st:
        return 2
    return 0


def run(inputs, out_dir, a, log=print, prefix='helper_residual'):
    import concurrent.futures as cf
    tasks = [(a, x) for x in inputs]
    results = []
    jobs = max(1, min(a['jobs'], len(tasks)))
    if jobs == 1:
        for t in tasks:
            results += _worker(t)
    else:
        for var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
            os.environ.setdefault(var, '1')                           # children are started after this: one BLAS thread each
        with cf.ProcessPoolExecutor(jobs) as ex:
            for r in ex.map(_worker, tasks):
                results += r
    paths, text = write_outputs(results, out_dir, prefix)
    for line in text.splitlines():
        log(line)
    for p in paths:
        log('wrote ' + p)
    return results


def build_args(ns):
    return dict(bundle_dir=ns.bundle_dir, plan=ns.plan, clock_offset=ns.clock_offset, window_seconds=ns.window_seconds,
                limit_pos_cm=ns.limit_pos_cm, limit_rot_deg=ns.limit_rot_deg, min_written=ns.min_written, worst=ns.worst,
                detail=ns.detail, jobs=ns.jobs, extra_helpers=not ns.bundle_bones_only)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('inputs', nargs='+', help='.utpppose file, folder or wildcard; each argument is one recording')
    ap.add_argument('--out-dir', required=True, help='output folder (required; never next to the inputs)')
    ap.add_argument('--prefix', default='helper_residual')
    ap.add_argument('--bundle-dir', help='folder with owned.txt (default Build\\test-data\\body_helper_<body>)')
    ap.add_argument('--plan', help='plan.json of a harness run: one window per scenario step')
    ap.add_argument('--clock-offset', type=float, default=0.0, help='log time minus plan time [s]')
    ap.add_argument('--window-seconds', type=float, default=0.0, help='fixed windows of this length when no --plan is given')
    ap.add_argument('--limit-pos-cm', type=float, default=DEFAULT_POS_CM)
    ap.add_argument('--limit-rot-deg', type=float, default=DEFAULT_ROT_DEG)
    ap.add_argument('--min-written', type=int, default=5, help='written frames a window needs for a verdict')
    ap.add_argument('--worst', type=int, default=5, help='worst frames listed per window')
    ap.add_argument('--detail', choices=['summary', 'full'], default='summary', help='full = per-bone statistics for every window (default: whole recording only)')
    ap.add_argument('--bundle-bones-only', action='store_true', help='skip the helper bones outside the bundle (default: they are evaluated and reported, never gated)')
    ap.add_argument('--jobs', type=int, default=max(1, min(12, (os.cpu_count() or 2) // 2)), help='recordings analysed in parallel (processes)')
    ns = ap.parse_args(argv)
    results = run(ns.inputs, ns.out_dir, build_args(ns), prefix=ns.prefix)
    return exit_code(results)


if __name__ == '__main__':
    sys.exit(main())
