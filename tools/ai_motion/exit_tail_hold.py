"""Tail hold for an existing car-exit steps npz (2026-11-09, vehexit6): a foot whose LAST landing still creeps to the vanilla final spot (MotionBricks labels it not planted at g, so
exit_steps_gen.py assemble --tail keeps the scene's gliding leg after g) is re-aimed so it touches down ON its final spot (the shift blended over its last swing) and stays there; the foot keeps
the last frame's world orientation once planted (slerped in over --orient frames). Two-bone leg IK with the exact knee hinge of exit_land_steps.ik_hinge. Everything else of the npz is kept.

  python -I exit_tail_hold.py <scene.dump.npz (prep, the dump the npz was assembled from)> <tpose.npz> <in.steps.npz> <out.steps.npz> [--sides Left,Right] [--h 0.03] [--orient 3] [--min-creep-cm 1.0]
Contact = the checker's rule (lower of ankle / toe within --h m of its floor = lowest height from frame 40 of the scene). A side is changed only if its final contact run moves more than --min-creep-cm (horizontal path).
UTPP_RIG=man for male rows.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import exit_land_steps as LS  # noqa: E402


def main(a):
    import game_skeleton_bridge as CB
    scene, tpose, src, out = a[:4]
    opt = lambda k, d: a[a.index(k) + 1] if k in a else d
    sides = opt('--sides', 'Left,Right').split(',')
    hcon, korient, minc = float(opt('--h', 0.03)), int(opt('--orient', 3)), float(opt('--min-creep-cm', 1.0))
    br, d = CB.load_cp_bridge(tpose, scene)
    st = np.load(src, allow_pickle=True)
    keep = {k: st[k] for k in st.files}
    W = st['W'].copy()
    c = int(st['c'])
    n = len(W)
    rep = []
    for sd in sides:
        th, ca, fo = (br.uidx[x] for x in br.legs[sd])
        ax = LS.hinge_axis(d['W'], th, ca)
        A = W[:, fo, :3, 3].copy()
        F = A[-1].copy()
        Rf = W[-1, fo, :3, :3].copy()
        # contact as in check_car_exit.step_metrics: the lower of ankle / toe within --h of that joint's floor (lowest height from frame 40 of the scene = head-duck / vanilla legs)
        tb = br.uidx[sd + 'ToeBase']
        dzs = float(d['dz']) if 'dz' in d.files else 0.0
        fa, ft = float(d['W'][40:, fo, 2, 3].min()) - dzs, float(d['W'][40:, tb, 2, 3].min()) - dzs
        con = np.minimum(A[:, 2] - fa, W[:, tb, 2, 3] - ft) < hcon
        if not con[-1]:
            rep.append('%s: not in contact on the last frame, unchanged' % sd)
            continue
        td = n - 1
        while td - 1 >= 0 and con[td - 1]:
            td -= 1
        path = float(np.linalg.norm(np.diff(A[td:, :2], axis=0), axis=1).sum() * 100)
        if path <= minc or td == 0:
            rep.append('%s: final contact from frame %d moves %.2f cm, unchanged' % (sd, c + td, path))
            continue
        ts = td - 1
        while ts - 1 >= 0 and not con[ts - 1]:
            ts -= 1
        ts = max(ts - 1, 0)   # last planted frame before the swing (weight 0 there)
        delta = F - A[td]
        delta[2] = 0.0
        for t in range(ts, n):
            if t < td:
                goal = A[t] + delta * float(LS.smooth01((t - ts) / float(td - ts)))
            else:
                goal = F.copy()
            if np.linalg.norm(goal - W[t, fo, :3, 3]) > 1e-6:
                LS.ik_hinge(br, W[t], sd, goal, ax)
            w = 1.0 if t >= td else (float(LS.smooth01(1.0 - (td - t) / float(korient + 1))) if td - t <= korient else 0.0)
            if w > 0.0:
                cur = W[t, fo, :3, :3].copy()
                br._rotate_subtree(W[t], fo, LS.slerp_m(cur, Rf, w) @ cur.T, W[t, fo, :3, 3].copy())
        A2 = W[:, fo, :3, 3]
        rep.append('%s: final contact from frame %d crept %.2f cm -> %.2f cm; landing re-aimed by (%.1f %.1f) cm over frames %d..%d' % (
            sd, c + td, path, float(np.linalg.norm(np.diff(A2[td:, :2], axis=0), axis=1).sum() * 100), delta[0] * 100, delta[1] * 100, c + ts, c + td))
    keep['W'] = W
    keep['src'] = np.array(str(st['src']) + '+tail_hold' if 'src' in st.files else 'tail_hold')
    np.savez(out, **keep)
    print('TAIL_HOLD %s | %s' % (out, ' | '.join(rep)))


if __name__ == '__main__':
    main(sys.argv[1:])
