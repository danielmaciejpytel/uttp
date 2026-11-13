"""Strike-intent metric of the melee redo (round F, review M5): deviation of the hand path relative to the chest from the game clip's, scaled to V's arms.

  python -I redo_intent.py <clips dir> [--kind v3|v4] [--out FILE] [--rig man]     batch: every <set>__<clip>.<kind>.npz of the dir vs its game clip
Metric (the reviewer's a4_intent measure, scaled): per frame and hand  | (hand - Spine3)_clip - ratio x (hand - Spine3)_game |  in the game axes of the clip
(both clips live in the same world: the redo output is in the frame-0 character frame of the game clip), ratio = arm length (Arm -> ForeArm -> Hand) of V's base
body / of the game rig at frame 0 (redo_solve.arm_ratio).  'chest' variant: the same offsets rotated into each clip's own Spine3 frame (information only: it
moves with every chest turn / lean).  Per clip: mean / max per hand, frame of the max, worst hand.
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_solve as RS  # noqa: E402

MEAN_MAX_CM, MAX_MAX_CM = 5.0, 9.0       # acceptance of round F (task): mean <= 5 cm, max <= 9 cm where the gate allows


def dev(B, Wg, Wc, ratio=None):
    """-> {S: dict(world [T] cm, chest [T] cm, vec [T,3] cm (clip - scaled game, game axes))} for S in Left / Right."""
    ix = B.ix
    ratio = ratio or RS.arm_ratio(B, Wg)
    out = {}
    for S in ('Left', 'Right'):
        og = Wg[:, ix[S + 'Hand'], :3, 3] - Wg[:, ix['Spine3'], :3, 3]
        oc = Wc[:, ix[S + 'Hand'], :3, 3] - Wc[:, ix['Spine3'], :3, 3]
        v = (oc - ratio[S] * og) * 100.0
        cg = np.einsum('tji,tj->ti', Wg[:, ix['Spine3'], :3, :3], og)
        cc = np.einsum('tji,tj->ti', Wc[:, ix['Spine3'], :3, :3], oc)
        out[S] = dict(world=np.linalg.norm(v, axis=1), chest=np.linalg.norm(cc - ratio[S] * cg, axis=1) * 100.0, vec=v)
    return out


def summary(dv):
    res = {}
    for S, x in dv.items():
        w = x['world']
        t = int(w.argmax())
        res[S[0]] = dict(mean=round(float(w.mean()), 2), max=round(float(w.max()), 2), f_max=t, vec_at_max=[round(float(a), 1) for a in x['vec'][t]],
                         chest_mean=round(float(x['chest'].mean()), 2), chest_max=round(float(x['chest'].max()), 2))
    wh = max(res, key=lambda k: res[k]['max'])
    res['worst'] = dict(hand=wh, mean=max(res['L']['mean'], res['R']['mean']), max=res[wh]['max'])
    res['ok'] = bool(res['worst']['mean'] <= MEAN_MAX_CM and res['worst']['max'] <= MAX_MAX_CM)
    return res


def clip_summary(B, s, c, Wc):
    Wg = MC.load_dump(MC.clip_path(s, c))['W']
    return summary(dev(B, Wg, Wc))


def batch(d, kind='v3'):
    B = RS.Body()
    rows = {}
    for p in sorted(glob.glob(os.path.join(d, '*.%s.npz' % kind))):
        s, c = os.path.basename(p)[:-len('.%s.npz' % kind)].split('__')
        if MC.RIG_OF_SET.get(s) != BB.RIG_NAME:
            continue
        rows['%s/%s' % (s, c)] = clip_summary(B, s, c, np.load(p)['W'])
    m = np.array([r['worst']['mean'] for r in rows.values()])
    x = np.array([r['worst']['max'] for r in rows.values()])
    sm = dict(clips=len(rows), mean_of_means=round(float(m.mean()), 2) if len(m) else None, p90_mean=round(float(np.percentile(m, 90)), 2) if len(m) else None,
              mean_of_max=round(float(x.mean()), 2) if len(x) else None, p90_max=round(float(np.percentile(x, 90)), 2) if len(x) else None,
              clips_mean_le_5=int((m <= MEAN_MAX_CM).sum()), clips_max_le_9=int((x <= MAX_MAX_CM).sum()), clips_ok=sum(1 for r in rows.values() if r['ok']))
    return dict(summary=sm, rows=rows)


if __name__ == '__main__':
    a = sys.argv[1:]
    res = batch(os.path.abspath(a[0]), a[a.index('--kind') + 1] if '--kind' in a else 'v3')
    if '--out' in a:
        MC.jdump(res, os.path.abspath(a[a.index('--out') + 1]))
    print(json.dumps(res['summary']))
