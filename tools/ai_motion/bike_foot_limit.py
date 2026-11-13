"""Gentle foot-slide limiter for a MOTORCYCLE get-off clip (offline, no game): keep the rhythm of the clip, remove the visible skating of the planted feet.

  python -I bike_foot_limit.py <scene.npz> <out steps npz> [--vmax 0.8] [--decay 6] [--omax 14] [--merge <MotionBricks steps npz>] [--sides Left,Right]

scene.npz = exit_steps_gen.py prep of the game_skeleton_bridge dump of the VANILLA clip (the 'scene' of bike_steps_run.sh). The output is a steps npz over the WHOLE clip (c = 0, g = last frame,
same schema as exit_steps_gen.py assemble) that bike_steps_gen.py pick / build_exit_duck.py --identity --steps turn into a clip. Hips, spine, arms, Root: untouched (vanilla). With --merge the frames c..g of a
MotionBricks candidate replace the scene frames first (vanilla + MotionBricks stepping), then the limiter runs over the whole clip.

What it does, per foot (the checker's contact rule of bike_steps_gen.slide_list: pivot = the lower of ankle / toe, within 3 cm of its lowest level of the clip, not moving vertically faster than
5 mm / frame): while a foot is planted, its horizontal pivot speed is cut to --vmax cm / frame (the checker counts only > 1 cm / frame as glide; the cut keeps the direction of the vanilla motion and
removes the excess, so the foot "creeps" at a speed that does not read as skating instead of sliding at the vanilla speed). The foot then lags the vanilla path by the removed excess (clamped to --omax cm);
once the foot leaves the ground (swing) the lag is handed back to the vanilla path with a smoothstep over --decay frames (the foot is in the air: no contact, no skate). The ankle is moved with the leg IK of
exit_steps_gen.ik_to (knee keeps its bending plane, foot keeps its world orientation); the vertical foot path is unchanged. The first and the last frame stay the vanilla ones (the lag is 0 there:
a run that is still moving at the end is not limited in its last frames, the report lists the remaining glide).
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import exit_steps_gen as ESG  # noqa: E402
import game_skeleton_bridge as CB  # noqa: E402

TPOSE = os.path.join(HERE, 'data', 'body_tpose_generator.npz')


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def contact_rule(W, names, side, h=0.03, vz_max=0.005):
    """(con, piv, pivot positions) of one side by the checker's rule; con[f] = planted, slide[f] = planted over f-1 -> f on the same pivot without vertical speed"""
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
    ha, ht = za - za.min(), zt - zt.min()
    piv = np.where(ha <= ht, ja, jt)
    hp = np.minimum(ha, ht)
    P = np.array([W[f, piv[f], :3, 3] for f in range(len(W))])
    con = hp < h
    slide = np.zeros(len(W), bool)
    for f in range(1, len(W)):
        slide[f] = bool(con[f] and con[f - 1] and piv[f] == piv[f - 1] and abs(P[f, 2] - P[f - 1, 2]) < vz_max)
    return con, slide, P


def lag_offsets(con, slide, P, vmax, decay, omax):
    """per-frame horizontal offset (m, shape (n, 2)) of the foot: constant speed cut while sliding, smoothstep hand-back after the contact ends"""
    n = len(P)
    off = np.zeros((n, 2))
    last, k = np.zeros(2), 10 ** 6
    for f in range(1, n):
        if slide[f]:
            dP = (P[f] - P[f - 1])[:2]
            v = float(np.linalg.norm(dP))
            cur = off[f - 1].copy()
            if v > vmax:
                cand = cur - dP / v * (v - vmax)
                if np.linalg.norm(cand) <= omax:
                    cur = cand
            off[f] = cur
            last, k = cur.copy(), 0
        elif con[f]:
            off[f] = off[f - 1]          # planted without the slide rule (toe roll / pivot change): keep the lag, no new motion
            last, k = off[f].copy(), 0
        else:
            k += 1
            off[f] = last * (1.0 - smooth01(k / float(decay)))
    return off


def main():
    p = argparse.ArgumentParser()
    p.add_argument('scene')
    p.add_argument('out')
    p.add_argument('--vmax', type=float, default=0.8, help='cm / frame, planted pivot speed limit (the checker counts > 1.0 as glide)')
    p.add_argument('--decay', type=int, default=6, help='frames: hand-back of the lag to the vanilla path once the foot is in the air')
    p.add_argument('--omax', type=float, default=14.0, help='cm: largest lag behind the vanilla path')
    p.add_argument('--merge', help='MotionBricks steps npz (bike_steps_run.sh assemble) laid over the scene frames c..g before the limiter')
    p.add_argument('--sides', default='Left,Right')
    a = p.parse_args()
    br, d = CB.load_cp_bridge(TPOSE, a.scene)
    W = d['W'].copy()
    names = [str(x) for x in d['names']]
    dz = float(d['dz'])
    n = len(W)
    c_gen = g_gen = 0
    if a.merge:
        m = np.load(a.merge, allow_pickle=True)
        c, g = int(m['c']), int(m['g'])
        W[c:g + 1] = m['W']
        W[c:g + 1, :, 2, 3] += dz
        c_gen, g_gen = (int(m['c_gen']), int(m['g_gen'])) if 'c_gen' in m.files else (c, g)
    rep = {}
    for sd in a.sides.split(','):
        con, slide, P = contact_rule(W, names, sd)
        off = lag_offsets(con, slide, P, a.vmax / 100.0, a.decay, a.omax / 100.0)
        # the first / last frame stay vanilla
        off[0] = 0.0
        off[-1] = 0.0
        fo = br.uidx[br.legs[sd][2]]
        goal0 = W[:, fo, :3, 3].copy()
        moved = 0
        for t in range(n):
            if np.linalg.norm(off[t]) > 1e-5:
                g_ = goal0[t].copy()
                g_[:2] += off[t]
                ESG.ik_to(br, W[t], sd, g_)
                moved += 1
        con2, slide2, P2 = contact_rule(W, names, sd)
        sp = [np.linalg.norm((P2[f] - P2[f - 1])[:2]) * 100 for f in range(1, n) if slide2[f]]
        sp0 = [np.linalg.norm((P[f] - P[f - 1])[:2]) * 100 for f in range(1, n) if slide[f]]
        rep[sd] = dict(frames_moved=moved, lag_max_cm=round(float(np.linalg.norm(off, axis=1).max()) * 100, 1),
                       glide_before_cm=round(float(sum(x for x in sp0 if x > 1.0)), 1), glide_after_cm=round(float(sum(x for x in sp if x > 1.0)), 1))
        print('FOOTLIMIT %-5s moved %2d frames, lag max %.1f cm, glide (>1 cm/f) %.1f -> %.1f cm' % (sd, moved, rep[sd]['lag_max_cm'], rep[sd]['glide_before_cm'], rep[sd]['glide_after_cm']))
    Wo = W.copy()
    Wo[:, :, 2, 3] -= dz
    np.savez(a.out, W=Wo, c=0, g=n - 1, dz=dz, foot_contacts=np.zeros((n, 4)), ramp_end=1, c_gen=c_gen, g_gen=g_gen, names=np.array(names), parents=d['parents'], leg_ik=True,
             lift='', end_fix=False, floor=False, knee_smooth='', src='footlimit v%.2f decay %d omax %.0f merge %s' % (a.vmax, a.decay, a.omax, os.path.basename(a.merge) if a.merge else '-'))
    print('FOOTLIMIT %s frames 0..%d' % (a.out, n - 1))


if __name__ == '__main__':
    main()
