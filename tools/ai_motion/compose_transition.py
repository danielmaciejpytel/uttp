"""Compose a two-clip transition scene (world dumps: mannequin_dump.py, or game_skeleton_bridge.py dump-clip / dump-glb) for
MotionBricks in-betweening.

python -I compose_transition.py <dumpA> <endA> <dumpB> <startB> <lenB> <N> <out_dump.npz> [--meta-dir DIR --name NAME]
                                [--rig source pack|cp] [--align travel|facing]
Frames: A[0..endA] | gap of 4N-8 placeholder frames (copies of A[endA]) | B[startB .. startB+lenB-1] moved rigidly.
--align travel (default): B is rotated about the vertical so its travel direction matches A's at endA, and placed so
  its pelvis lands D = gap time x mean(speed A, speed B) ahead of A's pelvis along A's direction (heights stay B's own).
--align facing: for stops / standing poses: B's pelvis heading matches A's at endA and B's pelvis lands on A's
  ground point plus D along that heading (D from the speeds as above, ~0 between two standing poses).
--rig: source pack = UEFN dump (pelvis, cm, faces -Y); cp = game dump (Hips, metres, faces +Y). Both Z up.
  cp: the whole scene is shifted vertically so A's lowest ankle at endA is at the rig's bind ankle height (feet on
  z = 0; game clips such as vehicle exits are relative to the seat, the generators assume the floor at 0).
For clip_inbetween.py on the bridged dump: c = endA, N (g = c + 4N - 7 is B's first frame).
--meta-dir/--name also write <DIR>/<NAME>.json (num_frames, empty curves/notifies) for retarget_batch.py.
"""
import json
import os
import sys

import numpy as np

RIGS = {'sourcepack': ('pelvis', 100.0, np.array([0.0, -1.0, 0.0])), 'cp': ('Hips', 1.0, np.array([0.0, 1.0, 0.0]))}


def travel(W, pel, f0, f1, unit=100.0):
    p = W[f0:f1 + 1, pel, :2, 3]
    d = p[-1] - p[0]
    n = np.linalg.norm(d)
    return (d / n if n > 1e-9 else d), np.linalg.norm(np.diff(p, axis=0), axis=1).mean() * 30.0 / unit  # dir, m/s


def heading(W, rest, pel, f, fwd):
    v = W[f, pel, :3, :3] @ rest[pel, :3, :3].T @ fwd
    v = v[:2]
    return v / np.linalg.norm(v)


def main(a):
    A, B = np.load(a[0]), np.load(a[2])
    endA, startB, lenB, N, out = int(a[1]), int(a[3]), int(a[4]), int(a[5]), a[6]
    rig = a[a.index('--rig') + 1] if '--rig' in a else 'sourcepack'
    align = a[a.index('--align') + 1] if '--align' in a else 'travel'
    pname, unit, fwd = RIGS[rig]
    names = [str(x) for x in A['names']]
    assert names == [str(x) for x in B['names']], 'different skeletons'
    pel = names.index(pname)
    WA, WB = A['W'], B['W']
    lenB = min(lenB, len(WB) - startB)
    dA, vA = travel(WA, pel, max(0, endA - 5), endA, unit)
    dB, vB = travel(WB, pel, startB, min(len(WB) - 1, startB + 5), unit)
    if align == 'facing':
        dA, dB = heading(WA, A['rest'], pel, endA, fwd), heading(WB, B['rest'], pel, startB, fwd)
    ang = np.arctan2(dA[1], dA[0]) - np.arctan2(dB[1], dB[0])
    Rz = np.eye(4)
    Rz[:2, :2] = [[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]]
    gap = 4 * N - 8
    D = (gap + 1) / 30.0 * 0.5 * (vA + vB) * unit  # rig units, A[endA] -> B[startB]
    pB = WB[startB, pel, :3, 3]
    target = WA[endA, pel, :3, 3].copy()
    target[:2] += dA * D
    M = np.eye(4)
    M[:3, 3] = -pB
    M = Rz @ M
    M[:2, 3] += target[:2]
    M[2, 3] += pB[2]
    if rig == 'cp':  # B's feet on A's ground: shift B by the difference of the lowest ankle heights
        fa = [names.index(x) for x in ('LeftFoot', 'RightFoot')]
        M[2, 3] += WA[endA, fa, 2, 3].min() - WB[startB, fa, 2, 3].min()
    Bpart = M[None, None] @ WB[startB:startB + lenB]
    W = np.concatenate([WA[:endA + 1], np.repeat(WA[endA:endA + 1], gap, 0), Bpart])
    if rig == 'cp':  # put A's end feet on the floor (z = 0): game clips can have another origin (vehicle seat ~0.47 m
        # above the ground) and MotionBricks / Kimodo assume the floor at height 0 (otherwise they "jump" back up to it)
        rest = A['rest']
        dz = min(rest[i, 2, 3] for i in fa) - WA[endA, fa, 2, 3].min()
        W[:, :, 2, 3] += dz
    keep = {k: A[k] for k in A.files if k not in ('W', 'frames', 'src', 't0', 't1')}
    np.savez(out, W=W, frames=len(W), c=endA, N=N, g=endA + 4 * N - 7, **keep)
    if '--meta-dir' in a:
        md, nm = a[a.index('--meta-dir') + 1], a[a.index('--name') + 1]
        os.makedirs(md, exist_ok=True)
        json.dump({'name': nm, 'num_frames': len(W) - 1, 'length_s': (len(W) - 1) / 30.0, 'enable_root_motion': True,
                   'curves': {}, 'notifies': [], 'note': 'composed transition (compose_transition.py)'},
                  open(os.path.join(md, nm + '.json'), 'w', encoding='utf-8'), indent=1)
    print('COMPOSE %s frames %d | c=%d N=%d g=%d | speed A %.2f B %.2f m/s | turn %.1f deg | gap distance %.2f m'
          % (out, len(W), endA, N, endA + 4 * N - 7, vA, vB, np.degrees(ang), D / unit))


if __name__ == '__main__':
    main(sys.argv[1:])
