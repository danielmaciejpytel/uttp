"""Anchor the exit part of a VX_* transition clip on the exit clip's own feet.

python -I vx_anchor.py <vx.utppclip.json> <exit.dump.npz> <scene.dump.npz> <tpose.npz> <out.utppclip.json> [--start 50] [--fade 14] [--name NAME]

game_skeleton_bridge.py inv --leg-ik puts the ankles where the SOMA legs say, which differs from the game clip's ankles by a few cm (other leg proportions): the first frames of the transition
clip (the exit's last frames) then do not coincide with the exit end pose the game hands over (measured 5-6 cm at the ankles). Here the ankles of the clip frames 0..(exit length - start - 1)
are put back on the exit clip's ankles by two-bone leg IK (knee plane and foot orientation kept) and the offset of the last exit frame fades out over --fade frames of the generated part.
Everything else (hips, spine, arms, the generated gap, the idle half) is the input clip; Trajectory is recomputed from the corrected poses by the same rule as the input.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import game_skeleton_bridge as CB  # noqa: E402
import exit_steps_gen as EG  # noqa: E402


def main(a):
    vx, exit_dump, scene, tpose, out = a[:5]
    start = int(a[a.index('--start') + 1]) if '--start' in a else 50
    fade = int(a[a.index('--fade') + 1]) if '--fade' in a else 14
    names, par, ref = CB.load_rig()
    W, clip = CB.world_from_utppclip(vx, names, par)
    A = np.load(exit_dump, allow_pickle=True)['W']
    br, d = CB.load_cp_bridge(tpose, scene)
    n_exit = len(A) - start   # clip frames 0..n_exit-1 are exit frames start..len(A)-1
    hi = names.index('Hips')
    dz = float(W[n_exit - 1, hi, 2, 3] - A[-1, hi, 2, 3])
    rep = {}
    for sd in ('Left', 'Right'):
        fo = br.uidx[br.legs[sd][2]]
        off = np.zeros((len(W), 3))
        for k in range(n_exit):
            off[k] = (A[start + k, fo, :3, 3] + np.array([0, 0, dz])) - W[k, fo, :3, 3]
        for k in range(n_exit, min(len(W), n_exit + fade)):
            off[k] = off[n_exit - 1] * (1 - EG.smooth01((k - (n_exit - 1)) / float(fade)))
        rep[sd] = dict(before_cm=[round(float(np.linalg.norm(off[k]) * 100), 1) for k in (0, n_exit // 2, n_exit - 1)])
        for k in range(len(W)):
            if np.linalg.norm(off[k]) > 1e-5:
                EG.ik_to(br, W[k], sd, W[k, fo, :3, 3] + off[k])
        rep[sd]['after_cm'] = [round(float(np.linalg.norm(W[k, fo, :3, 3] - (A[start + k, fo, :3, 3] + np.array([0, 0, dz]))) * 100), 3) for k in range(n_exit)][::5]
    name = a[a.index('--name') + 1] if '--name' in a else clip.get('source_clip', 'VX')
    note = 'anchored on the exit ankles by vx_anchor.py (start %d, fade %d); ' % (start, fade) + clip['retarget'].get('note', '')
    new = CB.to_utppclip(W, names, par, ref, d['rest'], name, 15, note=note)
    json.dump(new, open(out, 'w', encoding='utf-8'), separators=(',', ':'))
    print('VX_ANCHOR %s frames %d dz %.4f %s' % (out, len(W), dz, json.dumps(rep)))


if __name__ == '__main__':
    main(sys.argv[1:])
