"""Skin deformation of the offline runtime replay poses (utpp_replay_pose_stages), per body and per motion class.

  python -I replay_deformation.py --after-dir DIR [--before-dir DIR] --out OUT.json [--md OUT.md]
         [--runs body:run ...] [--root ROOT] [--mesh-root ROOT] [--stages-args ".."] [--before-stages-args ".."]
         [--after-stages-args ".."] [--stage final|raw] [--step 4] [--worst 6] [--window 4] [--jobs 8]
         [--work DIR] [--keep-bin] [--dump DIR --dump-rows body:run:row,...]
         [--before-stages-dir DIR] [--after-stages-dir DIR]   (existing replay outputs <DIR>/<run>.txt / .bin instead of a replay)

Pipeline: every session's recorded feed goes through utpp_replay_pose_stages of each exe dir (tools/replay/runtime_world_slide.py
run_stages: the same feed / settings log discovery, --ahead recorded, plus the stage arguments). The pose of a feed row (final stage by
default, --stage raw = the first stage; model space locals, Root and Trajectory included) gives the core locals of the body's
deformation rig; helpers come from the game's deformer graph and the body, arms and head meshes are skinned with the code of
measure_body_deformation.py (Rig.skin / Rig.metrics). Region metrics are REF-relative, as in measure_body_deformation.py:
  sev_p999   max(edge stretch p99.9, 1 / edge compression p0.1) of the region (the ranking key; robust to single bad edges)
  sev        the same on the extreme edges;  area_min   smallest triangle area ratio;  stretch_cm  largest edge lengthening, cm
Regions: shoulder, elbow, neck, knee.

Rows: only rows the replay published (published and tailed). Every --step-th row is evaluated (the uniform sample: the percentiles
come from it), then for each session and region the --worst rows of the sample (the larger of both sides) are refined: every row
within +-window of them is evaluated too (the maxima and the worst-frame lists use all evaluated rows). With --before-dir both sides
are evaluated on the same rows (feed row r is record r on both sides) and identical poses are skinned once.
Classes: world_slide.clip_class of the row's clip on that side (start, stop, pivot, loop, landing, idle, other) plus 'all'.

Output JSON: sides.{before,after}.{body}.{class}.{region} = n, p50 / p99 / p999 / max of sev_p999, sev_max, area_min,
stretch_cm_max; worst.{side}.{body}.{region} = the worst frames (run, row, t, clip, clip_time, value, other side value);
delta.{body}.{class}.{region} = after - before of each stat plus the paired row deltas (max, min, share of rows worse by more than
1e-4); worst_increase.{body}.{region} = the rows with the largest after - before increase; flexion_matched.{body}.{class} = the knee
region binned by each side's knee flexion (knee_flexion: the more flexed leg, 0 = straight), so a change of the flexion amount is
separated from a change of the skinning at equal flexion (see flexion_matched). --dump writes one npz per listed row and side (vertices in Z-up metres, faces, per-vertex stretch, region masks).
Read-only on every input; the replay output goes to --work and is deleted afterwards unless --keep-bin.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (TOOLS, HERE, os.path.join(TOOLS, 'replay'), os.path.join(TOOLS, 'footslide'), os.path.join(TOOLS, 'gametest')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from utpp_paths import project_root  # noqa: E402

REGIONS = ('shoulder', 'elbow', 'neck', 'knee')
STATS = ('sev_p999', 'sev', 'area_min', 'stretch_cm_max')
CLASSES = ('start', 'stop', 'pivot', 'loop', 'landing', 'idle', 'other')
BODY_NAME = {'woman': 'female', 'man': 'male'}
WORSE_EPS = 1e-4


# ------------------------------------------------------------------------------------------------------------------------ stage files
class StageFile:
    """A utpp_replay_pose_stages output (<prefix>.txt header + <prefix>.bin records), the records memory-mapped."""

    def __init__(self, prefix):
        self.names, self.parents, self.clips, self.meta_names, nb = [], [], {}, [], 0
        with open(prefix + '.txt', encoding='utf-8') as f:
            for ln in f:
                p = ln.split()
                if not p:
                    continue
                if p[0] == 'bones':
                    nb = int(p[1])
                elif p[0] == 'meta':
                    self.meta_names = p[2:]
                elif p[0] == 'bone':
                    self.names.append(p[2])
                    self.parents.append(int(p[3]))
                elif p[0] == 'clip':
                    self.clips[int(p[1])] = p[2]
        self.nb = nb
        self.width = len(self.meta_names) + nb * 14
        raw = np.memmap(prefix + '.bin', dtype=np.float32, mode='r')
        self.raw = raw[:len(raw) // self.width * self.width].reshape(-1, self.width)
        self.n = len(self.raw)

    def meta(self, key):
        return np.asarray(self.raw[:, self.meta_names.index(key)], dtype=np.float64)

    def pose(self, row, stage='final'):
        """[nb, 7] locals (t xyz, q xyzw) of a row; stage final = the published pose, raw = the first stage."""
        k = len(self.meta_names) + (self.nb * 7 if stage == 'final' else 0)
        return np.asarray(self.raw[row, k:k + self.nb * 7], dtype=np.float64).reshape(self.nb, 7)

    def clip_name(self, cid):
        return self.clips.get(int(cid), '-') if np.isfinite(cid) and cid >= 0 else '-'


def valid_rows(published, tailed):
    """Rows whose pose the replay published (the offline_session mask)."""
    return (np.nan_to_num(published) > 0.5) & (np.nan_to_num(tailed) > 0.5)


def sample_rows(valid, step):
    """Every step-th row (row % step == 0) that is valid."""
    r = np.arange(len(valid))
    return r[(r % max(1, step) == 0) & valid]


def window_rows(rows, scores, valid, worst, window):
    """Rows within +-window of the `worst` highest-scoring rows (scores aligned with rows; NaN ignored), valid only, sorted."""
    rows = np.asarray(rows)
    s = np.nan_to_num(np.asarray(scores, dtype=np.float64), nan=-np.inf)
    if not len(rows) or worst <= 0:
        return np.zeros(0, dtype=np.int64)
    top = rows[np.argsort(-s, kind='stable')[:worst]]
    out = set()
    for r in top:
        for k in range(int(r) - window, int(r) + window + 1):
            if 0 <= k < len(valid) and valid[k]:
                out.add(k)
    return np.array(sorted(out), dtype=np.int64)


def stage_core(names, pose, wanted, mat):
    """{bone: 4x4 local} of the stage bones in `wanted` (pose [nb, 7] t xyz q xyzw; no scale in the stage records)."""
    return {n: mat(pose[i, 0:3], pose[i, 3:7]) for i, n in enumerate(names) if n in wanted}


def _pct(v, q):
    v = np.asarray(v, dtype=np.float64)
    v = v[np.isfinite(v)]
    return round(float(np.percentile(v, q)), 4) if len(v) else None


def pool_stats(sample, every, cls_sample, cls_every):
    """{class: {region: stats}} from per-row region values.

    sample / every: [rows, regions, STATS] arrays of the uniform sample and of all evaluated rows (sample + windows); cls_*: class of
    each row. Percentiles from the sample, maxima / minima from every evaluated row."""
    out = {}
    for c in CLASSES + ('all',):
        ms = np.ones(len(sample), bool) if c == 'all' else (np.asarray(cls_sample) == c)
        me = np.ones(len(every), bool) if c == 'all' else (np.asarray(cls_every) == c)
        if not ms.any() and not me.any():
            continue
        reg = {}
        for k, name in enumerate(REGIONS):
            s = sample[ms, k, 0] if ms.any() else np.zeros(0)
            e = every[me, k] if me.any() else np.zeros((0, len(STATS)))
            reg[name] = dict(n=int(np.isfinite(s).sum()), p50=_pct(s, 50), p99=_pct(s, 99), p999=_pct(s, 99.9),
                             max=_nanmax(e[:, 0]), sev_max=_nanmax(e[:, 1]), area_min=_nanmin(e[:, 2]), stretch_cm_max=_nanmax(e[:, 3]))
        out[c] = reg
    return out


def _nanmax(v):
    v = np.asarray(v, dtype=np.float64)
    v = v[np.isfinite(v)]
    return round(float(v.max()), 4) if len(v) else None


def _nanmin(v):
    v = np.asarray(v, dtype=np.float64)
    v = v[np.isfinite(v)]
    return round(float(v.min()), 4) if len(v) else None


def paired_delta(before, after, cls):
    """{class: {region: max / min / mean row delta of sev_p999 and share of rows worse by > WORSE_EPS}} on rows valid on both sides."""
    out = {}
    for c in CLASSES + ('all',):
        m = np.ones(len(before), bool) if c == 'all' else (np.asarray(cls) == c)
        if not m.any():
            continue
        reg = {}
        for k, name in enumerate(REGIONS):
            d = after[m, k, 0] - before[m, k, 0]
            d = d[np.isfinite(d)]
            reg[name] = dict(rows=int(len(d)), max=round(float(d.max()), 4) if len(d) else None,
                             min=round(float(d.min()), 4) if len(d) else None, mean=round(float(d.mean()), 5) if len(d) else None,
                             worse_share=round(float(np.mean(d > WORSE_EPS)), 4) if len(d) else None,
                             better_share=round(float(np.mean(d < -WORSE_EPS)), 4) if len(d) else None)
        out[c] = reg
    return out


def stat_delta(before, after):
    """after - before of every pooled stat ({class: {region: stats}}), plus the relative change of the excess (sev - 1)."""
    out = {}
    for c, regs in after.items():
        if c not in before:
            continue
        out[c] = {}
        for r, s in regs.items():
            b = before[c].get(r, {})
            d = {}
            for key, v in s.items():
                if key == 'n' or v is None or b.get(key) is None:
                    continue
                d[key] = round(v - b[key], 4)
                if key in ('p99', 'p999', 'max') and b[key] > 1.0:
                    d[key + '_excess_pct'] = round(100.0 * (v - b[key]) / (b[key] - 1.0), 2)
            out[c][r] = d
    return out


LEG_CHAINS = (('LeftUpLeg', 'LeftLeg', 'LeftFoot'), ('RightUpLeg', 'RightLeg', 'RightFoot'))


def _qrot(q, v):
    """Rotates v [n, 3] by unit quaternions q [n, 4] (x y z w)."""
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def _qmul(a, b):
    ax, ay, az, aw = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    bx, by, bz, bw = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    q = np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                  aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], 1)
    return q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)


def knee_flexion(sf, rows, stage='final'):
    """[len(rows)] knee flexion in degrees (angle between thigh and shin, 0 = straight) of the more flexed leg of each row; NaN when the
    stage file has no leg chain."""
    rows = np.asarray(rows, dtype=np.int64)
    out = np.full(len(rows), np.nan)
    if not len(rows):
        return out
    k = len(sf.meta_names) + (sf.nb * 7 if stage == 'final' else 0)
    poses = np.asarray(sf.raw[rows, k:k + sf.nb * 7], dtype=np.float64).reshape(len(rows), sf.nb, 7)
    for chain in LEG_CHAINS:
        if any(b not in sf.names for b in chain):
            continue
        idx = [sf.names.index(b) for b in chain]
        path, b = [], idx[2]
        while b >= 0:
            path.insert(0, b)
            b = sf.parents[b]
        if idx[0] not in path or idx[1] not in path:
            continue
        q = np.tile([0.0, 0.0, 0.0, 1.0], (len(rows), 1))
        p = np.zeros((len(rows), 3))
        pos = {}
        for b in path:
            p = p + _qrot(q, poses[:, b, 0:3])
            pos[b] = p
            q = _qmul(q, poses[:, b, 3:7])
        u, v = pos[idx[1]] - pos[idx[0]], pos[idx[2]] - pos[idx[1]]
        c = np.sum(u * v, 1) / np.maximum(np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1), 1e-12)
        out = np.fmax(out, np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))
    return out


def flexion_matched(before, after, flex_before, flex_after, cls, width=15.0, min_rows=20):
    """Knee sev_p999 at matched knee flexion: {class: bins + summary}. Rows of each side are binned by their own knee flexion (width
    degrees); per bin p50 / p99 of each side. matched_delta_p50 / _p99 = the after - before change averaged over the bins with at least
    min_rows rows on both sides (weight = the smaller count): the skinning change at equal flexion. raw_delta_p50 = the change of the
    pooled medians; flexion_part_p50 = raw - matched, the part that comes from the flexion distribution itself."""
    k = REGIONS.index('knee')
    edges = np.arange(0.0, 180.0 + width, width)
    out = {}
    for c in CLASSES + ('all',):
        m = np.ones(len(before), bool) if c == 'all' else (np.asarray(cls) == c)
        vb, va = before[m, k, 0], after[m, k, 0]
        fb, fa = np.asarray(flex_before)[m], np.asarray(flex_after)[m]
        okb, oka = np.isfinite(vb) & np.isfinite(fb), np.isfinite(va) & np.isfinite(fa)
        if not okb.any() or not oka.any():
            continue
        bins, acc = [], {50: [0.0, 0.0], 99: [0.0, 0.0]}
        for lo, hi in zip(edges[:-1], edges[1:]):
            sb = vb[okb & (fb >= lo) & (fb < hi)]
            sa = va[oka & (fa >= lo) & (fa < hi)]
            if not len(sb) and not len(sa):
                continue
            row = dict(lo=float(lo), hi=float(hi), n_before=int(len(sb)), n_after=int(len(sa)))
            for q in (50, 99):
                row['p%d_before' % q], row['p%d_after' % q] = _pct(sb, q), _pct(sa, q)
            w = min(len(sb), len(sa))
            if w >= min_rows:
                for q in (50, 99):
                    d = row['p%d_after' % q] - row['p%d_before' % q]
                    row['delta_p%d' % q] = round(d, 4)
                    acc[q][0] += w * d
                    acc[q][1] += w
            bins.append(row)
        res = dict(bins=bins, rows_matched=int(acc[50][1]), flex_p50_before=_pct(fb[okb], 50), flex_p50_after=_pct(fa[oka], 50),
                   flex_p99_before=_pct(fb[okb], 99), flex_p99_after=_pct(fa[oka], 99),
                   raw_delta_p50=round(_pct(va[oka], 50) - _pct(vb[okb], 50), 4))
        for q in (50, 99):
            res['matched_delta_p%d' % q] = round(acc[q][0] / acc[q][1], 4) if acc[q][1] else None
        res['flexion_part_p50'] = round(res['raw_delta_p50'] - res['matched_delta_p50'], 4) if res['matched_delta_p50'] is not None else None
        out[c] = res
    return out


# ------------------------------------------------------------------------------------------------------------------------ skinning
_RIGS = {}


def _rig(body, mesh_root):
    if body not in _RIGS:
        os.environ['UTPP_PROJECT_ROOT'] = mesh_root  # the mesh loader resolves the project root at import time
        import measure_body_deformation as MB
        rig = MB.Rig(body)
        refv, refW = rig.skin(rig.ref_core())
        _, _, refstate = rig.metrics(refv, refW, refW)
        wanted = set(rig.idx) | set(rig.ev.coreNames)
        _RIGS[body] = (MB, rig, refW, refstate, wanted)
    return _RIGS[body]


def region_values(m):
    """[regions, STATS] of one Rig.metrics result."""
    return np.array([[m[r]['sev_p999'], m[r]['sev'], m[r]['area_min'] if m[r]['area_min'] is not None else np.nan,
                      m[r]['stretch_cm_max']] for r in REGIONS], dtype=np.float64)


def skin_rows(job):
    """Worker: region values of rows of one session on each side; a row whose pose equals the previous side's is not skinned again."""
    body, prefixes, rows, stage, mesh_root = job
    MB, rig, refW, refstate, wanted = _rig(body, mesh_root)
    files = [StageFile(p) for p in prefixes]
    out = np.full((len(files), len(rows), len(REGIONS), len(STATS)), np.nan)
    for j, r in enumerate(rows):
        prev = None
        for s, sf in enumerate(files):
            pose = sf.pose(int(r), stage)
            if prev is not None and np.array_equal(pose, prev[0]):
                out[s, j] = prev[1]
                continue
            v, W = rig.skin(stage_core(sf.names, pose, wanted, MB.SK.mat))
            m, _, _ = rig.metrics(v, W, refW, refstate)
            val = region_values(m)
            out[s, j] = val
            prev = (pose, val)
    return out


def dump_row(body, prefix, row, stage, mesh_root, path):
    MB, rig, refW, refstate, wanted = _rig(body, mesh_root)
    sf = StageFile(prefix)
    v, W = rig.skin(stage_core(sf.names, sf.pose(row, stage), wanted, MB.SK.mat))
    m, r, _ = rig.metrics(v, W, refW, refstate)
    st = rig.vertex_stretch(r)
    joints = ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot', 'Hips', 'Head']
    np.savez(path, verts=np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1).astype(np.float32), faces=rig.faces.astype(np.int32),
             stretch=st.astype(np.float32), **{'mask_' + k: x for k, x in rig.region.items()},
             joints=np.array([W[n][:3, 3] for n in joints]), joint_names=np.array(joints), metrics=json.dumps(m))


# ------------------------------------------------------------------------------------------------------------------------ driver
def session_inputs(root, rid):
    logs = os.path.join(root, 'Research', 'logs', 'night', rid, 'logs')
    feed = sorted(f for f in os.listdir(logs) if f.startswith('utpp-feed-') and f.endswith('.txt'))[-1]
    log = sorted(f for f in os.listdir(logs) if f.startswith('utpp-2') and f.endswith('.log'))[-1]
    return os.path.join(logs, feed), os.path.join(logs, log)


def replay(side, exe_dir, extra, spec, a):
    import runtime_world_slide as rws
    body, rid = spec.split(':', 1)
    feed, log = session_inputs(a.root, rid)
    prefix = os.path.join(a.work, side, '%s_%s' % (body, rid))
    os.makedirs(os.path.dirname(prefix), exist_ok=True)
    rws.run_stages(exe_dir, a.root, feed, log, body, prefix, extra)
    try:
        os.remove(prefix + '.motion.log')
    except OSError:
        pass
    return prefix


def chunks(rows, size):
    return [rows[i:i + size] for i in range(0, len(rows), size)]


def evaluate(a, sides, prefixes):
    """{spec: dict(rows, values [sides, rows, regions, STATS], sample mask)} over the sample and the worst windows."""
    pool = ProcessPoolExecutor(max_workers=a.jobs)
    info = {}
    for spec in a.runs:
        files = [StageFile(prefixes[s][spec]) for s in sides]
        n = min(f.n for f in files)
        valid = np.ones(n, bool)
        for f in files:
            valid &= valid_rows(f.meta('published')[:n], f.meta('tailed')[:n])
        info[spec] = dict(valid=valid, sample=sample_rows(valid, a.step))
        del files

    def run(spec_rows):
        futs = {spec: [pool.submit(skin_rows, (spec.split(':')[0], [prefixes[s][spec] for s in sides], c, a.stage, a.mesh_root))
                       for c in chunks(rows, a.chunk)] for spec, rows in spec_rows.items() if len(rows)}
        res = {}
        for spec, fl in futs.items():
            res[spec] = np.concatenate([f.result() for f in fl], axis=1)
        return res
    first = run({s: v['sample'] for s, v in info.items()})
    extra = {}
    for spec, v in info.items():
        vals = first.get(spec, np.zeros((len(sides), 0, len(REGIONS), len(STATS))))
        v['values'] = vals
        want = set()
        for k in range(len(REGIONS)):
            score = np.nanmax(vals[:, :, k, 0], axis=0) if vals.shape[1] else np.zeros(0)
            want.update(window_rows(v['sample'], score, v['valid'], a.worst, a.window).tolist())
        extra[spec] = np.array(sorted(want - set(v['sample'].tolist())), dtype=np.int64)
    second = run(extra) if a.window > 0 else {}
    pool.shutdown()
    for spec, v in info.items():
        add = second.get(spec)
        rows = v['sample']
        is_sample = np.ones(len(rows), bool)
        vals = v['values']
        if add is not None and len(extra[spec]):
            rows = np.concatenate([rows, extra[spec]])
            is_sample = np.concatenate([is_sample, np.zeros(len(extra[spec]), bool)])
            vals = np.concatenate([vals, add], axis=1)
        v.update(rows=rows, is_sample=is_sample, values=vals)
    return info


def build_report(a, sides, prefixes, info):
    doc = dict(args=dict(step=a.step, worst=a.worst, window=a.window, stage=a.stage, runs=a.runs,
                         stages_args={s: getattr(a, s + '_stages_args') for s in sides},
                         stages_dir={s: bool(getattr(a, s + '_stages_dir', None)) for s in sides}),
               sides={}, worst={}, delta={}, paired={}, sessions={})
    per_body = {}
    for spec, v in info.items():
        body = spec.split(':')[0]
        files = [StageFile(prefixes[s][spec]) for s in sides]
        rows = v['rows']
        cls, meta = [], []
        for f in files:
            clip = f.meta('clip')[rows]
            names = [f.clip_name(c) for c in clip]
            cls.append(np.array([clip_class(nm) for nm in names]))
            meta.append(dict(names=names, t=f.meta('t')[rows], ct=f.meta('clipTime')[rows]))
        flex = [knee_flexion(f, rows, a.stage) for f in files]
        per_body.setdefault(body, []).append(dict(spec=spec, rows=rows, is_sample=v['is_sample'], values=v['values'], cls=cls, meta=meta,
                                                  flex=flex))
        doc['sessions'][spec] = dict(rows=int(len(v['valid'])), published=int(v['valid'].sum()), sampled=int(v['is_sample'].sum()),
                                     window_rows=int((~v['is_sample']).sum()))
    for si, s in enumerate(sides):
        doc['sides'][s], doc['worst'][s] = {}, {}
        for body, parts in per_body.items():
            bn = BODY_NAME.get(body, body)
            samp = np.concatenate([p['values'][si][p['is_sample']] for p in parts])
            every = np.concatenate([p['values'][si] for p in parts])
            cs = np.concatenate([p['cls'][si][p['is_sample']] for p in parts])
            ce = np.concatenate([p['cls'][si] for p in parts])
            doc['sides'][s][bn] = pool_stats(samp, every, cs, ce)
            wl = {}
            for k, reg in enumerate(REGIONS):
                cand = []
                for p in parts:
                    for j in range(len(p['rows'])):
                        val = p['values'][si][j, k, 0]
                        if np.isfinite(val):
                            other = [round(float(p['values'][o][j, k, 0]), 4) for o in range(len(sides)) if o != si]
                            cand.append((float(val), p, j, other))
                cand.sort(key=lambda x: -x[0])
                wl[reg] = [dict(run=p['spec'].split(':')[1], row=int(p['rows'][j]), t=round(float(p['meta'][si]['t'][j]), 3),
                                clip=p['meta'][si]['names'][j], clip_time=round(float(p['meta'][si]['ct'][j]), 3),
                                cls=str(p['cls'][si][j]), value=round(val, 4), other=other) for val, p, j, other in cand[:a.list]]
            doc['worst'][s][bn] = wl
    if len(sides) == 2:
        for body, parts in per_body.items():
            bn = BODY_NAME.get(body, body)
            doc['delta'][bn] = stat_delta(doc['sides']['before'][bn], doc['sides']['after'][bn])
            b = np.concatenate([p['values'][0][p['is_sample']] for p in parts])
            af = np.concatenate([p['values'][1][p['is_sample']] for p in parts])
            cls = np.concatenate([p['cls'][0][p['is_sample']] for p in parts])
            doc['paired'][bn] = paired_delta(b, af, cls)
            doc.setdefault('flexion_matched', {})[bn] = flexion_matched(
                b, af, np.concatenate([p['flex'][0][p['is_sample']] for p in parts]),
                np.concatenate([p['flex'][1][p['is_sample']] for p in parts]), cls)
            inc = {}
            for k, reg in enumerate(REGIONS):
                cand = []
                for p in parts:
                    d = p['values'][1][:, k, 0] - p['values'][0][:, k, 0]
                    for j in np.argsort(-np.nan_to_num(d, nan=-np.inf))[:a.list]:
                        if np.isfinite(d[j]) and d[j] > WORSE_EPS:
                            cand.append((float(d[j]), p, int(j)))
                cand.sort(key=lambda x: -x[0])
                inc[reg] = [dict(run=p['spec'].split(':')[1], row=int(p['rows'][j]), t=round(float(p['meta'][1]['t'][j]), 3),
                                 clip_before=p['meta'][0]['names'][j], clip=p['meta'][1]['names'][j], cls=str(p['cls'][1][j]),
                                 before=round(float(p['values'][0][j, k, 0]), 4), after=round(float(p['values'][1][j, k, 0]), 4),
                                 delta=round(dv, 4)) for dv, p, j in cand[:a.list]]
            doc.setdefault('worst_increase', {})[bn] = inc
    return doc


def clip_class(name):
    from world_slide import clip_class as cc
    return cc(name)


def write_md(doc, path):
    lines = ['# Replay skin deformation', '', 'sev_p999 per region (REF-relative): p99 / p999 / max; class = the clip class of the row.', '']
    sides = list(doc['sides'])
    for body in doc['sides'][sides[0]]:
        lines += ['## %s' % body, '', '| class | region | ' + ' | '.join(sides) + (' | delta p99 / p999 / max | rows worse / better |'
                                                                               if len(sides) == 2 else '') + ' |',
                  '|---|---|' + '---|' * (len(sides) + (2 if len(sides) == 2 else 0))]
        for c in CLASSES + ('all',):
            for r in REGIONS:
                cells = []
                for s in sides:
                    st = doc['sides'][s][body].get(c, {}).get(r)
                    cells.append('-' if not st else '%s / %s / %s' % (st['p99'], st['p999'], st['max']))
                if all(x == '-' for x in cells):
                    continue
                if len(sides) == 2:
                    d = doc['delta'][body].get(c, {}).get(r, {})
                    pd = doc['paired'][body].get(c, {}).get(r, {})
                    cells.append('%s / %s / %s' % (d.get('p99'), d.get('p999'), d.get('max')))
                    cells.append('%s / %s' % (pd.get('worse_share'), pd.get('better_share')))
                lines.append('| %s | %s | %s |' % (c, r, ' | '.join(cells)))
        lines.append('')
        fm = doc.get('flexion_matched', {}).get(body)
        if fm:
            lines += ['Knee at matched flexion (sev_p999 after - before; flexion in degrees, p50 / p99 before -> after):', '',
                      '| class | flexion | raw delta p50 | matched delta p50 / p99 | flexion part p50 | rows matched |',
                      '|---|---|---|---|---|---|']
            for c in CLASSES + ('all',):
                x = fm.get(c)
                if x:
                    lines.append('| %s | %s / %s -> %s / %s | %s | %s / %s | %s | %s |' % (
                        c, x['flex_p50_before'], x['flex_p99_before'], x['flex_p50_after'], x['flex_p99_after'], x['raw_delta_p50'],
                        x['matched_delta_p50'], x['matched_delta_p99'], x['flexion_part_p50'], x['rows_matched']))
            lines.append('')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def main(argv=None):
    import runtime_world_slide as rws
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--after-dir')
    ap.add_argument('--before-dir')
    ap.add_argument('--after-stages-dir', help='existing replay outputs (<dir>/<run>.txt / .bin) used instead of running --after-dir')
    ap.add_argument('--before-stages-dir', help='the same for the before side')
    ap.add_argument('--out', required=True)
    ap.add_argument('--md')
    ap.add_argument('--runs', nargs='+', default=rws.DEFAULT_RUNS)
    ap.add_argument('--root', default=project_root(), help='project root of the replay inputs (Data, Research/logs/night)')
    ap.add_argument('--mesh-root', help='root with Build/experiments/BodyDeformationOffline (default: --root or the nearest parent with it)')
    ap.add_argument('--stages-args', default='', help='replay arguments of both sides')
    ap.add_argument('--before-stages-args')
    ap.add_argument('--after-stages-args')
    ap.add_argument('--stage', default='final', choices=['final', 'raw'])
    ap.add_argument('--step', type=int, default=4)
    ap.add_argument('--worst', type=int, default=6, help='sample rows refined per session and region')
    ap.add_argument('--window', type=int, default=4)
    ap.add_argument('--list', type=int, default=8, help='worst frames listed per body and region')
    ap.add_argument('--chunk', type=int, default=300)
    ap.add_argument('--jobs', type=int, default=8)
    ap.add_argument('--work')
    ap.add_argument('--keep-bin', action='store_true')
    ap.add_argument('--dump', help='folder for npz dumps of --dump-rows')
    ap.add_argument('--dump-rows', default='', help='body:run:row,... (one npz per side)')
    a = ap.parse_args(argv)
    if not (a.after_dir or a.after_stages_dir):
        ap.error('--after-dir or --after-stages-dir is required')
    a.before_stages_args = a.stages_args if a.before_stages_args is None else a.before_stages_args
    a.after_stages_args = a.stages_args if a.after_stages_args is None else a.after_stages_args
    a.work = a.work or os.path.join(a.root, 'Build', 'work', '_tmp', 'replay_deformation')
    a.mesh_root = a.mesh_root or _mesh_root(a.root)
    sides = ['before', 'after'] if (a.before_dir or a.before_stages_dir) else ['after']
    dirs = dict(before=a.before_dir, after=a.after_dir)
    given = {s: getattr(a, s + '_stages_dir') for s in sides if getattr(a, s + '_stages_dir')}  # read-only, never deleted
    jobs = [(s, spec) for s in sides if s not in given for spec in a.runs]
    out = []
    if jobs:
        with ThreadPoolExecutor(max_workers=min(6, len(jobs))) as ex:
            out = list(ex.map(lambda j: replay(j[0], dirs[j[0]], getattr(a, j[0] + '_stages_args'), j[1], a), jobs))
    prefixes = {s: {} for s in sides}
    for (s, spec), p in zip(jobs, out):
        prefixes[s][spec] = p
    for s, d in given.items():
        for spec in a.runs:
            prefixes[s][spec] = os.path.join(d, spec.split(':', 1)[1])
    try:
        if a.dump:
            os.makedirs(a.dump, exist_ok=True)
            for item in filter(None, a.dump_rows.split(',')):
                body, rid, row = item.split(':')
                spec = '%s:%s' % (body, rid)
                for s in sides:
                    path = os.path.join(a.dump, '%s_%s_%s_%s.npz' % (s, body, rid, row))
                    dump_row(body, prefixes[s][spec], int(row), a.stage, a.mesh_root, path)
                    print('dumped', path)
            return 0
        info = evaluate(a, sides, prefixes)
        doc = build_report(a, sides, prefixes, info)
    finally:
        if not a.keep_bin:
            for s in sides:
                if s in given:
                    continue
                for p in prefixes[s].values():
                    for ext in ('.bin', '.txt', '.motion.log'):
                        try:
                            os.remove(p + ext)
                        except OSError:
                            pass
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1)
    if a.md:
        write_md(doc, a.md)
    for s in sides:
        for body, cl in doc['sides'][s].items():
            al = cl['all']
            print('%-6s %-6s ' % (s, body) + '  '.join('%s %s/%s/%s' % (r, al[r]['p99'], al[r]['p999'], al[r]['max']) for r in REGIONS))
    for body, cl in doc['paired'].items():
        al = cl['all']
        print('paired %-6s ' % body + '  '.join('%s max %s worse %s' % (r, al[r]['max'], al[r]['worse_share']) for r in REGIONS))
    return 0


def _mesh_root(root):
    d = os.path.abspath(root)
    while True:
        if os.path.isdir(os.path.join(d, 'Build', 'experiments', 'BodyDeformationOffline')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return root
        d = parent


if __name__ == '__main__':
    sys.exit(main())
