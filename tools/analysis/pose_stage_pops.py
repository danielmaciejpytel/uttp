"""Single-frame pose discontinuities of an offline stage replay (utpp_replay_pose_stages) and of the matching in-game recording.

  python -I pose_stage_pops.py STAGES_PREFIX [--game POSE_DIR] [--skip S] [--json OUT]

STAGES_PREFIX is the output prefix of utpp_replay_pose_stages (<prefix>.txt and <prefix>.bin): per feed the local pose of every database
bone before the leg tail ('driver') and as published ('final'). The same discontinuity test as the pose meter (pose_metrics.spike_mask:
excess over the median of the +-6 neighbouring frames) runs on both stages for the tracked bones; a pop of the published pose that the
driver stage does not contain was added by the leg tail (root offset, stance guard, foot locker, head lead).

Classes (worst bone of a frame): hips_pos (Hips translation, model space, cm), foot (LeftFoot / RightFoot rotation relative to the parent,
deg), hand (LeftHand / RightHand, model space), axial (spine, neck, head). 'big' = hips step >= 3 cm, other bones >= 6 deg.

--game POSE_DIR: the in-game recording of the same session (compact rings, log, feed dump). Its feed rows are matched to the recorded
frames (pose_metrics.align_feed); the replay record of a feed row is the row itself, so for every in-game frame the replayed pose is known.
Reported: the per-frame difference of the published local rotations (replay against game: how well the replay reproduces the session),
and for every in-game pop whether the replay has it at the same feed row and in which stage.

Library: load_stages(prefix), stage_pops(stages, stage), classify(bone). Read-only on every input; writes only --json.
"""
import argparse
import collections
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GAMETEST = os.path.join(os.path.dirname(HERE), 'gametest')
for _p in (HERE, GAMETEST):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import pose_metrics as pm  # noqa: E402

TRACK = pm.TRACK
BIG_CM = 3.0
BIG_DEG = 6.0


def classify(bone):
    if bone == 'Hips_pos':
        return 'hips_pos'
    if bone in ('LeftFoot', 'RightFoot'):
        return 'foot'
    if bone in ('LeftHand', 'RightHand'):
        return 'hand'
    return 'axial'


def is_big(bone, size):
    return size >= (BIG_CM if bone == 'Hips_pos' else BIG_DEG)


class Stages:
    """meta: dict name -> [N]; names, parents; pose[stage] = [N, nb, 7] (px py pz qx qy qz qw), stage 0 driver, 1 final; clips id -> name."""

    def __init__(self, meta, names, parents, pose, clips):
        self.meta, self.names, self.parents, self.pose, self.clips = meta, names, parents, pose, clips
        self.cat = {n: i for i, n in enumerate(names)}

    def fk(self, stage, bones):
        """Model-space (q [N,4], p [N,3]) of the named bones (from Root, Trajectory excluded: it is not an ancestor of Hips)."""
        local = self.pose[stage].astype(np.float64)
        out, memo = {}, {}

        def build(k):
            if k in memo:
                return memo[k]
            q = pm.qnorm(local[:, k, 3:7])
            p = local[:, k, 0:3]
            par = self.parents[k]
            if par >= 0:
                pq, pp = build(par)
                q, p = pm.qmul(pq, q), pp + pm.qrotate(pq, p)
            memo[k] = (q, p)
            return memo[k]

        for b in bones:
            if b in self.cat:
                out[b] = build(self.cat[b])
        return out

    def local_q(self, stage, bone):
        return pm.qnorm(self.pose[stage][:, self.cat[bone], 3:7].astype(np.float64))


def load_stages(prefix):
    names, parents, clips, meta_names, nb = [], [], {}, [], 0
    with open(prefix + '.txt', 'r', encoding='utf-8') as f:
        for ln in f:
            p = ln.split()
            if not p:
                continue
            if p[0] == 'bones':
                nb = int(p[1])
            elif p[0] == 'meta':
                meta_names = p[2:]
            elif p[0] == 'bone':
                names.append(p[2])
                parents.append(int(p[3]))
            elif p[0] == 'clip':
                clips[int(p[1])] = p[2]
    width = len(meta_names) + nb * 14
    raw = np.fromfile(prefix + '.bin', dtype=np.float32)
    raw = raw[:len(raw) // width * width].reshape(-1, width)
    meta = {n: raw[:, i].astype(np.float64) for i, n in enumerate(meta_names)}
    k = len(meta_names)
    pose = [raw[:, k:k + nb * 7].reshape(-1, nb, 7), raw[:, k + nb * 7:].reshape(-1, nb, 7)]
    return Stages(meta, names, parents, pose, clips)


def series_spikes(st, stage, dt):
    """{(bone, space): (step [N], excess [N], mask [N])} for the tracked bones; steps in deg (cm for Hips_pos)."""
    fk = st.fk(stage, TRACK)
    res = {}
    for name in TRACK:
        if name not in fk:
            continue
        mq, mt = fk[name]
        _, sp, _ = pm.angular_series(mq, dt, local=False)
        m, ex = pm.spike_mask(sp, dt, pm.SPIKE_MIN_DEG)
        res[(name, 'model')] = (sp * dt, ex, m)
        if name != 'Hips':
            _, sp, _ = pm.angular_series(st.local_q(stage, name), dt, local=True)
            m, ex = pm.spike_mask(sp, dt, pm.SPIKE_MIN_DEG)
            res[(name, 'parent')] = (sp * dt, ex, m)
    _, sp, _ = pm.linear_series(fk['Hips'][1] * 100.0, dt)
    m, ex = pm.spike_mask(sp, dt, pm.SPIKE_MIN_CM)
    res[('Hips_pos', 'model')] = (sp * dt, ex, m)
    return res


def stage_pops(st, stage, valid=None):
    """Pop frames of one stage: [(frame, bone, space, excess)], worst bone per frame. valid: [N] bool mask of usable frames."""
    dt = np.maximum(np.nan_to_num(st.meta['dt'], nan=1 / 60.0), 1e-3)
    ser = series_spikes(st, stage, dt)
    best = {}
    for (bone, space), (_, ex, m) in ser.items():
        for f in np.nonzero(m)[0]:
            if valid is not None and not (valid[f] and (f == 0 or valid[f - 1])):
                continue
            if f not in best or ex[f] > best[f][3]:
                best[f] = (int(f), bone, space, float(ex[f]))
    return [best[f] for f in sorted(best)], ser


def summarize(pops, seconds):
    c, b = collections.Counter(), collections.Counter()
    for _, bone, _, size in pops:
        c[classify(bone)] += 1
        if is_big(bone, size):
            b[classify(bone)] += 1
    return dict(per_class=dict(c), per_class_big=dict(b), big_per_s={k: round(v / seconds, 4) for k, v in b.items()} if seconds > 0 else None)


def moving_mask(st):
    """Feeds with a ground locomotion stick input or a moving capsule (the meter's movement window)."""
    mv = np.hypot(np.nan_to_num(st.meta['mvx']), np.nan_to_num(st.meta['mvy'])) > 0.1
    return mv & (st.meta['owned'] > 0) & (st.meta['tailed'] > 0)


class ClipContent:
    """Stored clip frames (utpp_replay_pose_stages --clips): the step a bone makes in the clip itself between two clip times."""

    def __init__(self, clips_prefix):
        self.st = load_stages(clips_prefix)
        cid = self.st.meta['clip'].astype(int)
        self.first = {}
        self.count = collections.Counter(cid.tolist())
        for i, c in enumerate(cid):
            self.first.setdefault(int(c), i)
        self.fps = 1.0 / float(self.st.meta['dt'][0])
        self._fk = self.st.fk(0, TRACK)

    def value(self, clip, t, bone, space):
        """Interpolated bone value at clip time t (quaternion, or Hips position in cm), None outside the clip."""
        if clip not in self.first or not np.isfinite(t) or t < 0:
            return None
        x = t * self.fps
        k = int(np.floor(x))
        if k + 1 >= self.count[clip]:
            return None
        i, a = self.first[clip] + k, x - k
        if bone == 'Hips_pos':
            p = self._fk['Hips'][1]
            return (p[i] * (1 - a) + p[i + 1] * a) * 100.0
        q = self._fk[bone][0] if space == 'model' else self.st.local_q(0, bone)
        q0, q1 = q[i], q[i + 1] * (1.0 if np.dot(q[i], q[i + 1]) >= 0 else -1.0)
        v = q0 * (1 - a) + q1 * a
        return v / np.linalg.norm(v)

    def step(self, clip, t0, t1, bone, space):
        a, b = self.value(clip, t0, bone, space), self.value(clip, t1, bone, space)
        if a is None or b is None:
            return None
        if bone == 'Hips_pos':
            return float(np.linalg.norm(b - a))
        return float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))))


def attribute(st, ser, row, bone, space, content=None):
    """Where the step of one bone at one replay row comes from: final / driver step, the clip's own step over the played interval, and
    the layer that adds it: 'leg_tail' (final - driver >= threshold), 'driver_blend' (driver - clip >= threshold), else 'clip_content'
    (or 'driver' when the clip step is unknown: clip change, loop wrap, no clip dump)."""
    key = (bone, space)
    fin = float(ser[1][key][0][row])
    drv = float(ser[0][key][0][row])
    thr = BIG_CM if bone == 'Hips_pos' else BIG_DEG
    clip_step = None
    c0, c1 = int(st.meta['clip'][row - 1]), int(st.meta['clip'][row])
    if content is not None and c0 == c1 and c1 >= 0:
        # the clip time of the status has 0.01 s resolution: the played interval is rate x dt ending at it
        rate = float(st.meta['playRate'][row])
        rate = rate if np.isfinite(rate) and rate > 0 else 1.0
        t1 = float(st.meta['clipTime'][row])
        clip_step = content.step(c1, t1 - rate * float(st.meta['dt'][row]), t1, bone, space)
    if fin - drv >= thr:
        where = 'leg_tail'
    elif clip_step is None:
        where = 'driver'
    elif drv - clip_step >= thr:
        where = 'driver_blend'
    else:
        where = 'clip_content'
    return dict(final=round(fin, 2), driver=round(drv, 2), clip=None if clip_step is None else round(clip_step, 2), where=where,
                clip_change=c0 != c1)


def compare_game(st, pose_dir, skip_s, content=None):
    """In-game pops of the recording (final stage) against the replay at the matching feed rows."""
    segs = sorted(glob.glob(os.path.join(pose_dir, 'utpp-pose-ring-*.utpppose')) or glob.glob(os.path.join(pose_dir, 'utpp-pose-*.utpppose')))
    feed_path = sorted(glob.glob(os.path.join(pose_dir, 'utpp-feed-*.txt')))[-1]
    rec, data = pm.load_recording(segs, set(TRACK))
    sec = data['sec']
    feed = pm.read_feed(feed_path)
    r2f, info = pm.align_feed(sec, feed)
    if r2f is None:
        return dict(alignment=info)
    n_rows = min(len(r2f), len(st.meta['t']))
    f2r = np.full(len(sec), -1)
    ok = (r2f[:n_rows] >= 0)
    f2r[r2f[:n_rows][ok]] = np.nonzero(ok)[0]
    # reproduction: published local rotation, replay vs game, on matched frames after the skip
    frames = np.nonzero((f2r >= 0) & (sec - sec[0] >= skip_s))[0]
    rows = f2r[frames]
    diff = {}
    for bone in ('Hips', 'LeftFoot', 'RightFoot', 'Spine3', 'Head', 'RightHand'):
        if bone not in rec.cat or bone not in st.cat:
            continue
        gi = data['keep'].index(rec.cat[bone])
        gq = pm.qnorm(data['q'][frames, gi, 1].astype(np.float64))
        rq = st.local_q(1, bone)[rows]
        d = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(gq * rq, axis=1)), 0, 1)))
        d = d[np.isfinite(d)]
        diff[bone] = dict(median_deg=round(float(np.median(d)), 2), p90_deg=round(float(np.percentile(d, 90)), 2),
                          share_below_2deg=round(float(np.mean(d < 2.0)), 3)) if len(d) else None
    gi = data['keep'].index(rec.cat['Hips'])
    gp = data['t'][frames, gi, 1].astype(np.float64) * 100
    rp = st.pose[1][rows, st.cat['Hips'], 0:3].astype(np.float64) * 100
    dp = np.linalg.norm(gp - rp, axis=1)
    dp = dp[np.isfinite(dp)]
    diff['Hips_pos_cm'] = dict(median=round(float(np.median(dp)), 2), p90=round(float(np.percentile(dp, 90)), 2)) if len(dp) else None
    # in-game pops (final stage), the replay's stage pops at the same row (+-1)
    res = pm.analyze(segs, None, None, feed_file=feed_path, helpers=False, top=100000)
    gpops = [e for e in res['windows'][0]['worst_discontinuities'] if e['t_s'] >= skip_s]
    pops_d, ser_d = stage_pops(st, 0)
    pops_f, ser_f = stage_pops(st, 1)
    ser = {0: ser_d, 1: ser_f}
    rp_driver = {f: x for f, *x in pops_d}
    rp_final = {f: x for f, *x in pops_f}
    table = []
    for e in gpops:
        f = int(np.argmin(np.abs(sec - (sec[0] + e['t_s']))))
        r = int(f2r[f])
        hit_d = next((rp_driver[r + k] for k in (0, -1, 1) if r >= 0 and r + k in rp_driver), None)
        hit_f = next((rp_final[r + k] for k in (0, -1, 1) if r >= 0 and r + k in rp_final), None)
        row = dict(time=e['time'], bone=e['bone'], space=e['space'], size=e['size'], row=r, replay_final=hit_f, replay_driver=hit_d)
        if hit_f and r > 0 and (e['bone'], e['space']) in ser_f:
            rr =next(r + k for k in (0, -1, 1) if r + k in rp_final)
            row['attribution'] = attribute(st, ser, rr, e['bone'], e['space'], content)
        table.append(row)
    return dict(alignment=info, reproduction=diff, game_pops=table)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('stages')
    ap.add_argument('--game')
    ap.add_argument('--skip', type=float, default=20.0)
    ap.add_argument('--clips', help='clip dump prefix (utpp_replay_pose_stages --clips) for the clip content check')
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    st = load_stages(a.stages)
    content = ClipContent(a.clips) if a.clips else None
    t = st.meta['t']
    valid = moving_mask(st) & (t - t[0] >= a.skip)
    seconds = float(np.sum(np.nan_to_num(st.meta['dt'])[valid]))
    out = dict(stages=os.path.basename(a.stages), feeds=len(t), moving_s=round(seconds, 1))
    for stage, name in ((0, 'driver'), (1, 'final')):
        pops, _ = stage_pops(st, stage, valid)
        out[name] = summarize(pops, seconds)
        out[name + '_pops'] = [dict(row=f, t=round(float(t[f]), 3), bone=b, space=s, size=round(x, 2),
                                    clip=st.clips.get(int(st.meta['clip'][f]), str(int(st.meta['clip'][f]))))
                               for f, b, s, x in pops]
    if a.game:
        out['game'] = compare_game(st, a.game, a.skip, content)
    txt = {k: v for k, v in out.items() if not k.endswith('_pops') and k != 'game'}
    print(json.dumps(txt, indent=1))
    if a.game:
        g = out['game']
        print('game alignment:', g.get('alignment'))
        print('reproduction (published local rotation, replay vs game):', json.dumps(g.get('reproduction')))
        tab = g.get('game_pops') or []
        cnt = collections.Counter()
        for e in tab:
            where = 'driver' if e['replay_driver'] else ('leg_tail' if e['replay_final'] else 'not_reproduced')
            cnt[(classify(e['bone']), where)] += 1
        print('in-game pops by class and where the replay has them:', dict(sorted(cnt.items())))
        att = collections.Counter()
        for e in tab:
            if e.get('attribution') and is_big(e['bone'], e['size']):
                att[(classify(e['bone']), e['attribution']['where'])] += 1
        print('big in-game pops reproduced, by class and adding layer:', dict(sorted(att.items())))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(out, f, indent=0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
