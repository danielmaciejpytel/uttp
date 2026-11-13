"""Replays recorded fists punches through the PRODUCTION glue of the melee chest base correction and measures the striking arm.

Subcommands (python -I fist_base_offline.py <cmd> ...; needs numpy, Pillow-free; Source/tools/analysis/pose_helper_report.py):
  eval     --pose NAME --night DIR --utpp-log LOG --body F|M --tag TAG [--exe EXE] [--out DIR]
           builds a replay file from EVERY full-body / mask window of the 'punches' and 'punch_walk' steps (frames in time order, one
           arm per window), runs utpp_melee_base_offline (tests/MeleeBaseOffline.cpp: MeleeBase.cpp OnArmed / SetArmedBody /
           LowerBodyKeepWeight / BlendIntoRecords / Apply, the calls and order of FkHook.cpp, real table of the armed body), and
           writes <out>/offline_rows_<TAG>.pkl for the report scripts.
  fixture  same inputs + --fixture PATH [--stand-frames N] [--walk-frames N]: a small excerpt (one standing combo window + one walking
           window) with the clip reference elevation per frame, for the ctest (tests/data/melee_base_<body>.bin).
The recording: stage 0 = the engine's records, stage 1 = after the MM blend (before this layer). The snapshot's MM pose of the lower bones is the
recorded MM pose while walking (stage 1) and the MM idle pose of the 'settle' window while standing (the recording shows the engine there).
"""
import argparse
import datetime
import glob
import json
import math
import os
import pickle
import re
import struct
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'analysis'))
import pose_helper_report as ph  # noqa: E402
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import game_dir  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
CLIPS_IN = os.path.join(ROOT, 'Build', 'work', 'melee-fists-combined-20261109', 'clips-in')
EXE = os.path.join(ROOT, 'Build', 'UTPP', 'utpp_melee_base_offline.exe')
OUT2 = struct.Struct('<5i 4f 3f 3f 3f I f')
LOWER_PREFIX = ('LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftHeel', 'LeftToe', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightHeel', 'RightToe')


def is_lower(n):
    return n in ('Hips', 'Spine') or n.startswith(LOWER_PREFIX)


def sec_of(txt):
    t = datetime.datetime.strptime(txt, '%H:%M:%S.%f')
    return t.hour * 3600 + t.minute * 60 + t.second + t.microsecond / 1e6


def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def qrot(q, v):
    qv, w = q[..., :3], q[..., 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def qmean(q):
    ref = q[len(q) // 2]
    s = np.sign(np.sum(q * ref, axis=-1, keepdims=True))
    s[s == 0] = 1
    m = np.nanmean(q * s, axis=0)
    return m / np.linalg.norm(m)


class Recording:
    def __init__(self, pose_name, night_dir):
        cfg = ph.Config()
        g = ph.load_groups([os.path.join(game_dir(), 'red4ext', 'logs', pose_name)], cfg)[0]
        h = g['header']
        self.names = [h.names[i] for i in h.mapped()]
        self.cat = {n: k for k, n in enumerate(self.names)}
        rig = [h.game_index[i] for i in h.mapped()]
        r2c = {r: k for k, r in enumerate(rig)}
        self.parent = [r2c.get(h.parents[r], -1) if r < len(h.parents) else -1 for r in rig]
        secs, vals = [], []
        for stream, ticks, sec, gidx, v in ph.iter_blocks(g):
            secs.append(sec)
            vals.append(v.astype(np.float32))
        self.t = np.concatenate(secs)
        self.V = np.concatenate(vals)
        s = json.load(open(os.path.join(night_dir, 'gametest', 'summary.json')))
        self.steps = {}
        for st in s['steps']:
            if st['scenario'].startswith('melee'):
                self.steps[st['step']] = (sec_of(st['t0'].split(' ')[1]), sec_of(st['t1'].split(' ')[1]))
        order, done = [], set()

        def visit(i):
            if i in done:
                return
            if self.parent[i] >= 0:
                visit(self.parent[i])
            done.add(i)
            order.append(i)
        for i in range(len(self.names)):
            visit(i)
        self.order = order

    def idx(self, a, b):
        return np.nonzero((self.t >= a) & (self.t < b))[0]

    def P(self, ix, stage):
        return self.V[ix][:, :, stage, 0:3].astype(np.float64)

    def Q(self, ix, stage):
        q = self.V[ix][:, :, stage, 4:8].astype(np.float64)
        n = np.linalg.norm(q, axis=-1, keepdims=True)
        return q / np.where(n > 1e-9, n, np.nan)

    def fk(self, P, Q):
        nb = len(self.names)
        if not getattr(self, 'order', None):
            order, done = [], set()

            def visit(i):
                if i in done:
                    return
                if self.parent[i] >= 0:
                    visit(self.parent[i])
                done.add(i)
                order.append(i)
            for i in range(nb):
                visit(i)
            self.order = order
        MP, MQ = np.zeros((nb, 3)), np.zeros((nb, 4))
        for i in self.order:
            p = self.parent[i]
            if p < 0:
                MP[i], MQ[i] = P[i], Q[i]
            else:
                MP[i] = MP[p] + qrot(MQ[p], P[i])
                MQ[i] = qmul(MQ[p], Q[i])
        return MP, MQ


def windows(log, pat):
    out, cur = [], None
    for ln in open(log, encoding='utf-8', errors='replace'):
        m = re.match(r'\[2026-11-\d\d (\d\d:\d\d:\d\d\.\d+)\].*UTPP ' + pat + r' (on|off)\b', ln)
        if not m:
            continue
        t = sec_of(m.group(1))
        if m.group(2) == 'on' and cur is None:
            cur = t
        elif m.group(2) == 'off' and cur is not None:
            out.append((cur, t))
            cur = None
    return out


def load_clip_table(body):
    prefix = {'F': 'pwa_fists', 'M': 'pma_fists'}[body]
    clips = {}
    for f in sorted(glob.glob(os.path.join(CLIPS_IN, prefix + '__*.npz'))):
        nm = os.path.basename(f).split('__')[1].split('.')[0]
        z = np.load(f, allow_pickle=True)
        clips[nm] = dict(W=z['W'].astype(np.float64), n73=list(z['names']))
    return clips


def clip_pose(clips, name, f, t=0.0, f2=None):
    c = clips[name]
    W = c['W'][f] if f2 is None or t <= 0 else (1 - t) * c['W'][f] + t * c['W'][f2]
    return {n: (W[i, :3, 3] * 100.0) for i, n in enumerate(c['n73'])}


def elev_yaw(pos, side):
    v = pos[side + 'Hand'] - pos[side + 'Arm']
    return (float(np.degrees(np.arcsin(np.clip(v[2] / np.linalg.norm(v), -1, 1)))), float(np.degrees(np.arctan2(v[0], v[1]))), float(np.linalg.norm(v)))


def build_frames(r, log, wins, stand_frames=None, walk_frames=None):
    """list of dicts (one per replay frame), in time order; each window starts with a hard reset (dt < 0)."""
    mask_w = windows(log, 'MMMASK')
    full_w = windows(log, 'MMMELEEFULL')
    ixs = r.idx(r.steps['settle'][0] + 0.3, r.steps['settle'][1] - 0.1)
    lower = [n for n in r.names if is_lower(n)]
    mmP = {n: np.median(r.P(ixs, 1)[:, r.cat[n]], 0) for n in lower}
    mmQ = {n: qmean(r.Q(ixs, 1)[:, r.cat[n]]) for n in lower}
    out = []
    for wname in wins:
        t0, t1 = r.steps[wname]
        limit = stand_frames if wname == 'punches' else walk_frames
        got_one = False
        for wa, wb in mask_w:
            if not (t0 - 1 < wa < t1 + 1):
                continue
            if limit is not None and got_one:
                break
            got_one = True
            fa = [w for w in full_w if abs(w[0] - wa) < 0.5]
            fon = fa[0][0] if fa else None
            ix = r.idx(max(wa - 0.1, t0 - 1), min(wb, t1 + 1))
            if limit is not None:
                ix = ix[:limit]
            for n, i in enumerate(ix):
                tt = float(r.t[i])
                mw = float(np.clip((tt - wa) / 0.15, 0.0, 1.0)) if tt >= wa else 0.0
                ls = 1.0
                if fon is not None and tt >= fon:
                    ls = float(1.0 - np.clip((tt - fon) / 0.2, 0.0, 1.0))
                P0, Q0 = r.P([i], 0)[0], r.Q([i], 0)[0]
                P1, Q1 = r.P([i], 1)[0], r.Q([i], 1)[0]
                ok = np.isfinite(P0).all() and np.isfinite(Q0).all()
                if not ok:
                    continue
                walking = wname == 'punch_walk'
                mp = [P1[r.cat[n_]] if walking else mmP[n_] for n_ in lower]
                mq = [Q1[r.cat[n_]] if walking else mmQ[n_] for n_ in lower]
                if walking and not (np.isfinite(np.array(mp)).all() and np.isfinite(np.array(mq)).all()):
                    mp = [mmP[n_] for n_ in lower]
                    mq = [mmQ[n_] for n_ in lower]
                out.append(dict(i=int(i), w=wname, t=tt, since=tt - wa, P=P0, Q=Q0, P1=P1, Q1=Q1, mmP=np.array(mp), mmQ=np.array(mq), meleeW=mw, lowerScale=ls,
                                upperScale=1.0 - mw, base=1, dt=-1.0 if n == 0 else tt - out[-1]['t'] if out else -1.0))
    return out, lower


NEEDED = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Head', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftShoulder',
          'LeftArm', 'LeftForeArm', 'LeftHand']


def write_mbo2(path, r, body, frames, lower, subset=False):
    """subset: only the bones the glue and the checks need (key bones, arms, lower bones) and their ancestors: a small fixture"""
    names = list(r.names)
    keep = list(range(len(names)))
    if subset:
        want = set(r.cat[n] for n in NEEDED + list(lower))
        closure = set()
        for b in want:
            while b >= 0 and b not in closure:
                closure.add(b)
                b = r.parent[b]
        keep = sorted(closure)
    newi = {b: k for k, b in enumerate(keep)}
    with open(path, 'wb') as fh:
        fh.write(b'MBO2' + struct.pack('<i', len(keep)))
        for b in keep:
            fh.write(struct.pack('<h', newi[r.parent[b]] if r.parent[b] >= 0 else -1) + names[b].encode()[:31].ljust(32, b'\0'))
        fh.write(struct.pack('<ii', 0 if body == 'F' else 1, len(lower)))
        fh.write(b''.join(struct.pack('<i', newi[r.cat[n]]) for n in lower))
        fh.write(struct.pack('<i', len(frames)))
        for f in frames:
            fh.write(f['P'][keep].astype('<f4').tobytes() + f['Q'][keep].astype('<f4').tobytes() + f['mmP'].astype('<f4').tobytes() + f['mmQ'].astype('<f4').tobytes())
            fh.write(struct.pack('<fffI', f['meleeW'], f['lowerScale'], f['upperScale'], f['base']))
            fh.write(struct.pack('<ffi', f['dt'], f.get('refElev', float('nan')), f.get('side', 0)))


def run_exe(exe, inp, out):
    res = subprocess.run([exe, inp, out], capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError('replay failed: ' + res.stderr)
    sys.stderr.write(res.stderr)
    data = open(out, 'rb').read()
    assert data[:4] == b'MBOO'
    nb, nf = struct.unpack_from('<ii', data, 4)
    off = 12
    res_ = []
    for k in range(nf):
        Q = np.frombuffer(data, '<f4', nb * 4, off).reshape(nb, 4).astype(np.float64)
        off += nb * 16
        so = OUT2.unpack_from(data, off)
        off += OUT2.size
        res_.append((Q, so))
    clipnames = {'F': [], 'M': []}
    for ln in open(out + '.clips.txt', encoding='utf-8'):
        if ln.strip():
            b_, nm_, first_, cnt_ = ln.strip().split(':')
            clipnames[b_].append((nm_, int(first_), int(cnt_)))
    return res_, clipnames


def analyse(r, body, frames, res_, clipnames, clips, ext_min=45.0):
    rows = []
    for fr, (Qo, so) in zip(frames, res_):
        matched, bd, clip, frame, nbr, t_, rms, gate, wgt = so[:9]
        # 'before' = the pose the game showed without this layer: stage 1 (engine upper body on the MM lower body) while walking, the engine pose standing
        pb, qb = (fr['P1'], fr['Q1']) if fr['w'] == 'punch_walk' and np.isfinite(fr['Q1']).all() else (fr['P'], fr['Q'])
        MP0, _ = r.fk(pb, qb)
        MP1, _ = r.fk(fr['P'], Qo)
        e = {s_: np.linalg.norm(MP0[r.cat[s_ + 'Hand']] - MP0[r.cat[s_ + 'Arm']]) * 100 for s_ in ('Right', 'Left')}
        side = 'Right' if e['Right'] >= e['Left'] else 'Left'
        pos0 = {n: MP0[r.cat[n]] * 100 for n in ('RightArm', 'RightHand', 'LeftArm', 'LeftHand')}
        pos1 = {n: MP1[r.cat[n]] * 100 for n in ('RightArm', 'RightHand', 'LeftArm', 'LeftHand')}
        e0, y0, ext = elev_yaw(pos0, side)
        e1, y1, _ = elev_yaw(pos1, side)
        ec = yc = None
        cname = '-'
        if matched and bd >= 0:
            cname, first, cnt = clipnames['F' if bd == 0 else 'M'][clip]
            fl = frame - first
            f2 = (nbr - first) if nbr >= 0 else None
            ec, yc, _ = elev_yaw(clip_pose(clips, cname, fl, t_, f2), side)
        rows.append(dict(w=fr['w'], t=fr['t'], since=fr['since'], side=side, e0=e0, y0=y0, e1=e1, y1=y1, ec=ec, yc=yc, ext=ext, so=so, cname=cname, P=fr['P'], Q0=qb, Q1=Qo))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['eval', 'fixture'])
    ap.add_argument('--pose', required=True)
    ap.add_argument('--night', required=True)
    ap.add_argument('--utpp-log', required=True)
    ap.add_argument('--body', required=True, choices=['F', 'M'])
    ap.add_argument('--tag', default='')
    ap.add_argument('--exe', default=EXE)
    ap.add_argument('--out', default=os.path.join(ROOT, 'Build', 'work', 'release-0.1.0', '_tmp', 'fists_dirfix'))
    ap.add_argument('--fixture', default='')
    ap.add_argument('--stand-frames', type=int, default=110)
    ap.add_argument('--walk-frames', type=int, default=60)
    a = ap.parse_args()
    r = Recording(a.pose, a.night)
    tag = a.tag or a.body
    clips = load_clip_table(a.body)
    if a.cmd == 'fixture':
        frames, lower = build_frames(r, a.utpp_log, ('punches', 'punch_walk'), a.stand_frames, a.walk_frames)
    else:
        frames, lower = build_frames(r, a.utpp_log, ('punches', 'punch_walk'))
    inp = os.path.join(a.out, 'offline_in_%s.bin' % tag)
    out = os.path.join(a.out, 'offline_out_%s.bin' % tag)
    write_mbo2(inp, r, a.body, frames, lower)
    res_, clipnames = run_exe(a.exe, inp, out)
    rows = analyse(r, a.body, frames, res_, clipnames, clips)
    if a.cmd == 'fixture':
        for fr, row in zip(frames, rows):
            fr['side'] = 0 if row['side'] == 'Right' else 1
            fr['refElev'] = row['ec'] if (row['ec'] is not None and row['ext'] >= 45.0) else float('nan')
        write_mbo2(a.fixture, r, a.body, frames, lower, subset=True)
        print('fixture', a.fixture, os.path.getsize(a.fixture), 'bytes,', len(frames), 'frames,', sum(1 for f in frames if not math.isnan(f['refElev'])), 'with a reference')
        return
    pickle.dump(dict(rows=rows, names=r.names, parent=r.parent, cat=r.cat), open(os.path.join(a.out, 'offline_rows_%s.pkl' % tag), 'wb'))
    print('%s: frames %d, punch-extension frames %d' % (tag, len(frames), sum(1 for x in rows if x['ext'] >= 45.0)))


if __name__ == '__main__':
    main()
