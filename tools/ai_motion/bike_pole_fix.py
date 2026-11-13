"""Knee / elbow pole helpers of a MOTORCYCLE get-off clip that was re-stepped offline: keep the vanilla pole where the limb did not move (offline, no game).

  python -I bike_pole_fix.py --glb <builder glb> --vanilla <vanilla p?a_sportbike glb> --clip exit_r --out <out glb> [--lo-cm 1.0] [--hi-cm 3.0] [--report out.json]

Why: exit_steps.reanchor_helpers (build_exit_duck.py --steps) re-places the pole helpers (Torso_[World_]{Left,Right}{Leg,Arm}_PoleVector_JNT) of EVERY limb on frames c+1..g-1 in the limb plane frame of the new
pose. A limb that is (almost) straight or unchanged has no well defined bend plane there: the re-placement moved a pole by up to 25 mm in ONE frame (pma_sportbike3 exit_r frame 5, left leg, while the
limb moved < 1 cm) and turned the pole direction of the nearly straight legs by 30..107 deg in the last frame (the end pose is the vanilla one). The game's limb IK bends the knee toward the pole,
so such a frame is a knee pop. Same idea as build_exit_duck.place_poles(min_move_cm): the pole of a limb keeps the vanilla value (the IK solution did not change) on the frames where none of the three
limb joints moved more than --lo-cm against the vanilla pose, the re-placed one is used from --hi-cm, in between a smoothstep blend (no switch, no new jump). Everything else is untouched; the
output goes through the same import route as the builder glb (build_bike_exit.ps1).
"""
import argparse
import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def fix(tgt, lt, lr, van, vlt, vlr, lo_cm, hi_cm):
    Wt, _ = tgt.world(lt, lr)
    vWt, _ = van.world(vlt, vlr)
    rep = {}
    for s_ in ('Left', 'Right'):
        for limb in ('Leg', 'Arm'):
            chain = ((s_ + 'UpLeg', s_ + 'Leg', s_ + 'Foot') if limb == 'Leg' else (s_ + 'Arm', s_ + 'ForeArm', s_ + 'Hand'))
            moved = np.max([np.linalg.norm(Wt[:, tgt.index[n]] - vWt[:, van.index[n]], axis=-1) for n in chain], axis=0) * 100.0
            w = smooth01((moved - lo_cm) / max(hi_cm - lo_cm, 1e-6))
            for pn in ('Torso_%s%s_PoleVector_JNT' % (s_, limb), 'Torso_World_%s%s_PoleVector_JNT' % (s_, limb)):
                if pn not in tgt.index:
                    continue
                j, k = tgt.index[pn], van.index[pn]
                old = float(np.linalg.norm(lt[:, j] - vlt[:, k], axis=-1).max() * 1000)
                lt[:, j] = (1 - w)[:, None] * vlt[:, k] + w[:, None] * lt[:, j]
                q1 = lr[:, j].copy()
                flip = np.sum(q1 * vlr[:, k], -1) < 0
                q1[flip] *= -1
                q = (1 - w)[:, None] * vlr[:, k] + w[:, None] * q1
                lr[:, j] = q / np.linalg.norm(q, axis=-1, keepdims=True)
                rep[pn] = dict(dev_before_mm=round(old, 2), dev_after_mm=round(float(np.linalg.norm(lt[:, j] - vlt[:, k], axis=-1).max() * 1000), 2),
                               frames_vanilla=int((w == 0).sum()), frames_blend=int(((w > 0) & (w < 1)).sum()))
    lr[:] = S.qcontinuous(lr)
    return rep


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb', required=True)
    p.add_argument('--vanilla', required=True)
    p.add_argument('--clip', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--lo-cm', type=float, default=1.0)
    p.add_argument('--hi-cm', type=float, default=3.0)
    p.add_argument('--report')
    a = p.parse_args()
    tgt = S.Skeleton.from_glb(a.glb)
    c = tgt.clip(a.clip)
    dur, times = B.frames_of(tgt, c)
    lt, lr = tgt.sample_local(c, times)
    lr = S.qcontinuous(lr)
    van = S.Skeleton.from_glb(a.vanilla)
    vc = van.clip(a.clip)
    vdur, vtimes = B.frames_of(van, vc)
    assert len(vtimes) == len(times), 'frame count %d != vanilla %d' % (len(times), len(vtimes))
    vlt, vlr = van.sample_local(vc, vtimes)
    vlr = S.qcontinuous(vlr)
    lt0, lr0 = lt.copy(), lr.copy()
    rep = fix(tgt, lt, lr, van, vlt, vlr, a.lo_cm, a.hi_cm)
    # only the pole bones may have changed
    changed = sorted({tgt.names[j] for j in range(lt.shape[1]) if (np.abs(lt[:, j] - lt0[:, j]).max() > 1e-7 or np.abs(lr[:, j] - lr0[:, j]).max() > 1e-6)})
    assert all('PoleVector' in n for n in changed), changed
    extras = copy.deepcopy(tgt.gj['animations'][c].get('extras', {}))
    B.write_out(tgt, a.out, a.clip, times, lt, lr, extras)
    for n, r in rep.items():
        print('%-38s dev vs vanilla %7.2f -> %7.2f mm  vanilla frames %2d blend %2d' % (n, r['dev_before_mm'], r['dev_after_mm'], r['frames_vanilla'], r['frames_blend']))
    print('changed bones', changed)
    if a.report:
        import json
        json.dump(dict(poles=rep, changed=changed, lo_cm=a.lo_cm, hi_cm=a.hi_cm), open(a.report, 'w'), indent=1)


if __name__ == '__main__':
    main()
