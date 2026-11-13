"""Arm-through-torso penetration count on the skinned V mesh (offline, the same cache as the demo).

  python -I redo_pen_check.py <clip_demo_dir> [name=redo]        |   python -I redo_pen_check.py ref
Bind vertices are labelled by the nearest joint (rest pose): TORSO = Hips / Spine / Spine1 / Spine2 / Spine3 / Neck, ARM = ForeArm / Hand (without fingers' tips? all hand
joints incl. fingers).  Per frame: number of ARM vertices closer than PEN_M to a TORSO vertex (cKDTree).  Upper-arm vertices are skipped (armpit contact is normal).
`ref` = the same count for the base pose, the idle and a few source pack clips (what a healthy V shows; the skin is a thin shell, so 0-30 touching vertices is normal).
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import base_body as BB  # noqa: E402

PEN_M = 0.01
TORSO = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck']


def labels(rest_mesh):
    names, par, ref, rest = BB.rig()
    ix = {n: i for i, n in enumerate(names)}
    cand = [n for n in names if n not in ('Root', 'Trajectory', 'reference_joint')]
    J = np.stack([rest[ix[n], :3, 3] for n in cand])
    lab = np.array([cand[i] for i in np.argmin(np.linalg.norm(rest_mesh[:, None, :] - J[None], axis=2), axis=1)])
    tor = np.isin(lab, TORSO)
    arm = np.array([('ForeArm' in n or 'Hand' in n) for n in lab])
    return tor, arm


def count(skin_base):
    z = np.load(skin_base + '.npz')
    rest = z['rest'].astype(float)
    pc = AG._read_pc2(skin_base + '.pc2', len(rest))
    tor, arm = labels(rest)
    out = []
    for t in range(len(pc)):
        tr = cKDTree(pc[t][tor])
        d, _ = tr.query(pc[t][arm])
        out.append(int((d < PEN_M).sum()))
    return np.array(out)


def main(a):
    if a[0] == 'ref':
        res = {}
        for k in ('base_ready', 'base_idle', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F', 'AO_Pistol2H_Stand_Sweep', 'overlay_poses'):
            c = count(os.path.join(BB.REDO, 'skin_ref', k + '.skin'))
            res[k] = dict(max=int(c.max()), p95=int(np.percentile(c, 95)))
            print(k, res[k])
        json.dump(res, open(os.path.join(BB.REDO, 'pen_ref.json'), 'w'))
        return
    c = count(os.path.join(a[0], (a[1] if len(a) > 1 else 'redo') + '.skin'))
    print(json.dumps(dict(max=int(c.max()), frames_over_ref=None, per_frame=c.tolist())))


if __name__ == '__main__':
    main(sys.argv[1:])
