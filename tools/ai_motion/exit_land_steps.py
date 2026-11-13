"""Landing steps for a HIGH car exit (utility / Kaukaz Bratsk cab, 81-frame driver exit; 2026-11-09, vehexit6). Legs only: Hips, spine, arms, head and Root stay the
scene's (the head-duck exit of build_exit.ps1), so the pelvis / torso path, the roof / door / shell gates and the upper body are the vanilla ones.

The vanilla exit drops off the cab step: the left foot lands (~f64) 8 cm short of its final spot and then creeps forward 0.53 cm / frame to the end, the trailing right foot
hovers 40 cm behind the pelvis and sinks to the floor over ~15 frames while sliding forward (no touchdown until ~f75). Here:
  left   the drop is re-aimed so the foot touches down ON its final spot (the shift is blended into the fast part of the drop, --lfrom..--lland), then it stays planted;
  right  the trailing foot finishes its fall and plants behind the pelvis at the spot it was heading for (--rspot-frame; it catches the backward drift of the vanilla pelvis,
         which moves back until ~f68 and forward again after it), lifts at --roff and steps forward into the final stance (--rland2, lift --rlift cm);
  both   planted feet keep the last frame's world orientation (slerped in over --orient frames before touchdown), two-bone leg IK on the scene legs (knee plane of the scene).
The last frame equals the scene (vanilla end pose for the motion matcher): the steps npz carries ramp_end = 1 (exit_steps.splice hands over in one frame).

  python -I exit_land_steps.py <scene.dump.npz (exit_steps_gen.py prep)> <tpose.npz> <out.steps.npz> [--c 56] [--tail 80] [--lfrom 58] [--lland 65] [--rfrom 62] [--rland 66]
        [--rspot-frame 64] [--roff 69] [--rland2 78] [--rlift 7] [--orient 3] [--ease 0.35] [--plain-ik]
Output = the npz format of exit_steps_gen.py assemble --tail / plan (W of frames c..tail in game axes with dz removed, c / g / ramp_end / c_gen / g_gen / foot_contacts / names / parents),
for build_exit.ps1 -Steps. UTPP_RIG=man for the male rig (game_skeleton_bridge). Nothing here edits other files.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
SIDES = ('Left', 'Right')


def smooth01(x):
    x = np.clip(np.asarray(x, dtype=float), 0.0, 1.0)
    return x * x * (3 - 2 * x)


def slerp_m(Ra, Rb, w):
    from scipy.spatial.transform import Rotation as Rot, Slerp
    if w <= 0.0:
        return Ra
    if w >= 1.0:
        return Rb
    return Slerp([0, 1], Rot.from_matrix(np.stack([Ra, Rb])))([w]).as_matrix()[0]


def hinge_axis(W, th, ca):
    """dominant axis of the local shin (Leg) rotation vectors over the scene clip: the game's knee is a pure hinge (vanilla off-hinge 0.00 deg)"""
    from scipy.spatial.transform import Rotation as Rot
    rv = Rot.from_matrix(np.einsum('tji,tjk->tik', W[:, th, :3, :3], W[:, ca, :3, :3])).as_rotvec()
    ax = np.linalg.svd(rv, full_matrices=False)[2][0]
    return ax if float((rv @ ax).mean()) >= 0 else -ax


def ik_hinge(br, Wt, side, goal, ax):
    """two-bone leg IK with an EXACT knee hinge (only the hinge angle of the local Leg rotation changes, then the thigh turns minimally so the ankle reaches the goal;
    the foot keeps its world orientation). exit_steps_gen.ik_to turns the shin by a minimal world rotation instead, which adds 1-3 deg of knee twist off the hinge."""
    import skeleton_bridge as SB
    from scipy.spatial.transform import Rotation as Rot
    th, ca, fo = (br.uidx[x] for x in br.legs[side])
    hip = Wt[th, :3, 3].copy()
    Rth, Rca = Wt[th, :3, :3].copy(), Wt[ca, :3, :3].copy()
    rf = Wt[fo, :3, :3].copy()
    kl = Rth.T @ (Wt[ca, :3, 3] - hip)
    al = Rca.T @ (Wt[fo, :3, 3] - Wt[ca, :3, 3])
    t0 = float(Rot.from_matrix(Rth.T @ Rca).as_rotvec() @ ax)
    grid = t0 + np.radians(np.linspace(-150.0, 150.0, 6001))
    Rg = Rot.from_rotvec(grid[:, None] * ax[None, :]).as_matrix()
    dist = np.linalg.norm(kl[None, :] + np.einsum('nij,j->ni', Rg, al), axis=1)
    d = float(np.linalg.norm(goal - hip))
    s = np.sign(dist - d)
    cross = np.flatnonzero(s[1:] != s[:-1])
    if len(cross):
        k = int(cross[np.argmin(np.abs(grid[cross] - t0))])
        u = (d - dist[k]) / (dist[k + 1] - dist[k])
        tn = float(grid[k] + u * (grid[k + 1] - grid[k]))
    else:   # out of reach: the straightest hinge angle of the branch around the current one
        tn = float(grid[int(np.argmax(dist))])
    Lr = Rot.from_rotvec(tn * ax).as_matrix()
    v = Rth @ (kl + Lr @ al)
    Rm = SB.rot_between(v, goal - hip)
    br._rotate_subtree(Wt, th, Rm, hip)
    Q = (Rm @ Rth @ Lr) @ Wt[ca, :3, :3].T
    br._rotate_subtree(Wt, ca, Q, Wt[ca, :3, 3].copy())
    br._rotate_subtree(Wt, fo, rf @ Wt[fo, :3, :3].T, Wt[fo, :3, 3].copy())


def main(a):
    import game_skeleton_bridge as CB
    import exit_steps_gen as EG
    scene, tpose, out = a[0], a[1], a[2]
    opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
    c0, T = int(opt('--c', 56)), int(opt('--tail', 80))
    lfrom, lland = int(opt('--lfrom', 58)), int(opt('--lland', 65))
    rfrom, rland, rspot_f = int(opt('--rfrom', 62)), int(opt('--rland', 66)), int(opt('--rspot-frame', 64))
    roff, rland2, rlift = int(opt('--roff', 69)), int(opt('--rland2', 78)), opt('--rlift', 7.0) / 100.0
    korient, ease = int(opt('--orient', 3)), opt('--ease', 0.35)
    assert c0 < lfrom < lland < T and c0 < rfrom < rland < roff < rland2 < T, 'schedule out of order'
    br, d = CB.load_cp_bridge(tpose, scene)
    W0 = d['W'].copy()
    assert T == len(W0) - 1, 'tail %d but the scene has %d frames' % (T, len(W0))
    names = [str(x) for x in d['names']]
    fo = {sd: br.uidx[br.legs[sd][2]] for sd in SIDES}
    th = {sd: br.uidx[br.legs[sd][0]] for sd in SIDES}
    ca = {sd: br.uidx[br.legs[sd][1]] for sd in SIDES}
    chain = {sd: float(np.linalg.norm(W0[0, th[sd], :3, 3] - W0[0, ca[sd], :3, 3]) + np.linalg.norm(W0[0, ca[sd], :3, 3] - W0[0, fo[sd], :3, 3])) for sd in SIDES}
    A = {sd: W0[:, fo[sd], :3, 3].copy() for sd in SIDES}
    Rfin = {sd: W0[T, fo[sd], :3, :3].copy() for sd in SIDES}
    Pfin = {sd: A[sd][T].copy() for sd in SIDES}
    goal = {sd: {} for sd in SIDES}
    orient = {sd: {} for sd in SIDES}     # planted orientation weight 0..1 (1 = last frame's foot orientation)
    planted = {sd: np.zeros(T + 1, bool) for sd in SIDES}
    # ---- left: re-aimed drop, planted on the final spot from lland
    dL = Pfin['Left'] - A['Left'][lland]
    for t in range(lfrom, T + 1):
        if t < lland:
            goal['Left'][t] = A['Left'][t] + dL * float(smooth01((t - lfrom) / float(lland - lfrom)))
        else:
            goal['Left'][t] = Pfin['Left'].copy()
            planted['Left'][t] = True
    # ---- right: finish the fall onto the spot it was heading for (scene ankle xy of rspot_f, final ankle height), planted rland..roff, step to the final spot
    X = A['Right'][rspot_f].copy()
    X[2] = Pfin['Right'][2]
    dR = X - A['Right'][rland]
    for t in range(rfrom, rland):
        goal['Right'][t] = A['Right'][t] + dR * float(smooth01((t - rfrom) / float(rland - rfrom)))
    for t in range(rland, roff + 1):
        goal['Right'][t] = X.copy()
        planted['Right'][t] = True
    sw = lambda u: (1.0 - ease) * float(smooth01(u)) + ease * float(np.clip(u, 0.0, 1.0))
    for t in range(roff + 1, rland2):
        u = (t - roff) / float(rland2 - roff)
        p = X + (Pfin['Right'] - X) * sw(u)
        p[2] = X[2] + (Pfin['Right'][2] - X[2]) * sw(u) + rlift * np.sin(np.pi * u)
        goal['Right'][t] = p
    for t in range(rland2, T + 1):
        goal['Right'][t] = Pfin['Right'].copy()
        planted['Right'][t] = True
    # orientation: the last frame's foot orientation while planted, slerped in over korient frames before each touchdown (and kept through the short right swing)
    for sd in SIDES:
        for t in range(c0, T + 1):
            if planted[sd][t] or (sd == 'Right' and roff < t < rland2):
                orient[sd][t] = 1.0
        first = int(np.flatnonzero(planted[sd])[0])
        for k in range(1, korient + 1):
            t = first - k
            if t > c0:
                orient[sd][t] = float(smooth01(1.0 - k / float(korient + 1)))
    W = W0[c0:T + 1].copy()
    reach = {sd: [] for sd in SIDES}
    plain = '--plain-ik' in a   # the old minimal-rotation IK (exit_steps_gen.ik_to), for comparison only
    hax = {sd: hinge_axis(W0, th[sd], ca[sd]) for sd in SIDES}
    for sd in SIDES:
        for t in range(c0, T + 1):
            Wt = W[t - c0]
            if t in goal[sd]:
                g_ = goal[sd][t]
                reach[sd].append((t, float(np.linalg.norm(Wt[th[sd], :3, 3] - g_)) / chain[sd]))
                if np.linalg.norm(g_ - Wt[fo[sd], :3, 3]) > 1e-6:
                    if plain:
                        EG.ik_to(br, Wt, sd, g_)
                    else:
                        ik_hinge(br, Wt, sd, g_, hax[sd])
            w = orient[sd].get(t, 0.0)
            if w > 0.0:
                cur = Wt[fo[sd], :3, :3].copy()
                tgt = slerp_m(cur, Rfin[sd], w)
                br._rotate_subtree(Wt, fo[sd], tgt @ cur.T, Wt[fo[sd], :3, 3].copy())
    err = max(float(np.linalg.norm(W[t - c0, fo[sd], :3, 3] - goal[sd][t])) for sd in SIDES for t in goal[sd] if t >= c0) * 100
    fcs = np.zeros((len(W), 4))
    for t in range(c0, T + 1):
        fcs[t - c0] = [planted['Left'][t], planted['Left'][t], planted['Right'][t], planted['Right'][t]]
    dz = float(d['dz']) if 'dz' in d.files else 0.0
    W[:, :, 2, 3] -= dz
    np.savez(out, W=W, c=c0, g=T, dz=dz, foot_contacts=fcs, ramp_end=1, c_gen=c0, g_gen=T, names=np.array(names), parents=d['parents'], leg_ik=True, lift='land_steps', end_fix=False,
             floor=False, knee_smooth='', src='exit_land_steps')
    worst = {sd: max(reach[sd], key=lambda x: x[1]) for sd in SIDES}
    print('LAND_STEPS %s frames %d..%d | left drop re-aimed %d..%d by (%.1f %.1f %.1f) cm, planted from %d | right falls %d..%d onto (%.1f %.1f) [scene f%d], planted %d..%d, step %d..%d lift %.0f cm '
          'to (%.1f %.1f) travel %.1f cm | max reach/chain L %.3f @%d R %.3f @%d (chain L %.3f R %.3f m) | max ankle-goal error %.2f cm | ik %s' % (
              out, c0, T, lfrom, lland, *(dL * 100), lland, rfrom, rland, *(X[:2] * 100), rspot_f, rland, roff, roff, rland2, rlift * 100, *(Pfin['Right'][:2] * 100),
              float(np.linalg.norm((Pfin['Right'] - X)[:2]) * 100), worst['Left'][1], worst['Left'][0], worst['Right'][1], worst['Right'][0], chain['Left'], chain['Right'], err,
              'plain' if plain else 'hinge'))


if __name__ == '__main__':
    main(sys.argv[1:])
