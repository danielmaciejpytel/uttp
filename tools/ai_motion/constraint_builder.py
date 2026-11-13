"""Kimodo fullbody constraints from a bridged source pack clip (soma npz built with body_tpose_constraint.npz).

Same held-out setup as clip_inbetween.py: frames c-3..c become generated frames 0..3, frames g..g+3 become
frames (g-c)..(g-c+3). Canonical space: Hips XZ at frame c moved to the origin, heading at frame c turned to +Z
(Kimodo first_heading_angle 0). Writes constraints.json (77-joint local axis-angle; Kimodo converts to 30) and
a gt soma npz of the span in the same canonical space, then validates with Kimodo's own loader + FK.

python constraint_builder.py <soma_k.npz> <c> <g> <out_dir> [--targets 4] [--context 4]
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


def main(a):
    src, c, g, out = a[0], int(a[1]), int(a[2]), a[3]
    nt = int(a[a.index('--targets') + 1]) if '--targets' in a else 4
    nc = int(a[a.index('--context') + 1]) if '--context' in a else 4
    s = np.load(src)
    G, hips = s['G'].astype(np.float64), s['hips'].astype(np.float64)
    names = [str(x) for x in s['names']]
    fwd = G[c, names.index('Hips')] @ np.array([0, 0, 1.0])  # T-pose faces +Z
    yaw = np.arctan2(fwd[0], fwd[2])
    Ry = R.from_euler('y', -yaw).as_matrix()
    o = hips[c].copy(); o[1] = 0
    Gc = Ry[None, None] @ G
    hc = (Ry @ (hips - o).T).T
    from kimodo.skeleton.definitions import SOMASkeleton30, SOMASkeleton77
    import torch
    sk77 = SOMASkeleton77()
    assert names == list(sk77.bone_order_names)
    par = sk77.joint_parents.numpy()
    L = Gc.copy()
    for j, p in enumerate(par):
        if p >= 0:
            L[:, j] = np.swapaxes(Gc[:, p], -1, -2) @ Gc[:, j]
    src_frames = list(range(c - nc + 1, c + 1)) + list(range(g, g + nt))
    idx = [f - c + (nc - 1) for f in src_frames]
    aa = np.array([R.from_matrix(L[f]).as_rotvec() for f in src_frames])
    con = [{'type': 'fullbody', 'frame_indices': idx, 'local_joints_rot': aa.tolist(),
            'root_positions': hc[src_frames].tolist(), 'smooth_root_2d': hc[src_frames][:, [0, 2]].tolist()}]
    os.makedirs(out, exist_ok=True)
    json.dump(con, open(os.path.join(out, 'constraints.json'), 'w'))
    span = slice(c - nc + 1, g + nt)
    np.savez(os.path.join(out, 'gt_canonical.npz'), G=Gc[span], hips=hc[span], names=s['names'], k=s['k'], fps=30,
             c=nc - 1, g=g - c + nc - 1)
    # validate with Kimodo: load as the model would (30-joint skeleton) and compare FK positions with our FK
    from kimodo.constraints import load_constraints_lst
    from skeleton_bridge import Bridge  # noqa
    cl = load_constraints_lst(os.path.join(out, 'constraints.json'), SOMASkeleton30())
    pk = cl[0].global_joints_positions.numpy()
    t = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'body_tpose_constraint.npz'))
    br = Bridge.__new__(Bridge)
    br.snames = names; br.sidx = {n: i for i, n in enumerate(names)}
    br.spar = np.asarray(t['parents']); br.spos = np.asarray(t['pos'], dtype=np.float64)
    s30 = list(SOMASkeleton30().bone_order_names)
    ours = np.array([br.soma_fk(Gc[f], hc[f]) for f in src_frames])[:, [names.index(n) for n in s30]]
    err = np.linalg.norm(pk - ours, axis=-1) * 100
    dur = (idx[-1] + 1) / 30.0
    print('KIMODO_CONSTRAINTS %s frames %s | loader FK vs bridge max %.2f cm | duration >= %.2f s | heading %.1f deg' % (
        out, idx, err.max(), dur, np.degrees(yaw)))


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main(sys.argv[1:])
