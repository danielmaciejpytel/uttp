"""Motorcycle ENTRY clips (get-on): turn the low, gliding foot movement of the approach into real steps (offline, no game, no generator).

  python -I bike_entry_gait.py <dump.npz of the shipped clip (game_skeleton_bridge.py dump-glb)> <out steps npz> [--h-frac 0.25] [--h-min 4] [--h-max 9] [--clear 4.5] [--min-travel 7] [--vthr 1.0]
        [--arch-pow 2] [--pad 2] [--h-scale 1] [--step-lim 13.5] [--decay 6] [--omax 14] [--knee-rate 10] [--toe-clear 0 (off; 4 = pitch the toe up: tried 2026-11-07, breaks the step gate on 10 of 12 clips)] [--toe-max 40] [--hop-other 6] [--h-skip 5] [--limit-vmax 0.8]

Why not MotionBricks: the shortest span it accepts (N = 6, 18 frames) runs past the approach into the leg-over-saddle swing, where it knows no bike (sweep 2026-11-07: all 56 candidates failed the clearance /
acceleration gates). Here hips, spine, arms, Root and every frame outside the edited span stay the shipped clip's; only the legs change (two-bone leg IK, knee keeps its bending plane, the foot its
world orientation), i.e. rhythm and hip path are kept.

Per foot (ankle / toe speed in cm / frame, 30 fps):
  * a STEP EPISODE is a run of frames (gaps of one frame bridged) where the ankle moves faster than --vthr; its stance frames before / after are the neighbouring frames. Travel = horizontal ankle distance
    between those two stance frames. Episodes with travel >= --min-travel cm are steps;
  * the step gets a vertical arch h * sin(pi u)^--arch-pow (2: zero slope at lift-off / touch-down - the knee of an almost straight leg bends ~ sqrt(lift), a plain sine would need > 15 deg / frame there) above the local ground of the ankle (lowest ankle height inside +-8 frames) (h = clamp(--h-frac * travel, --h-min, --h-max) cm), applied as a soft maximum with the shipped height
    (a foot that already clears keeps its own path), only when the shipped foot clears the local ground (lowest height of the joint inside +-8 frames) by less than --clear cm (ankle AND toe) somewhere
    in the episode while it moves faster than --vthr: a toe that skims the floor becomes a lifted foot, touch-down and lift-off stay on the ground;
  * while the OTHER foot is more than --hop-other cm above the local ground (V stands on this foot only: the over-saddle leg is up) the arch is capped at --h-skip cm (a low skip; a high step on the only
    support foot would read as a hop);
  * the planted remainder (contact-rule slide) is then cut to --limit-vmax cm / frame by bike_foot_limit.lag_offsets on the stepped clip (the lag is handed back while the foot is in the air).
The steps npz has the format of entry_steps_gen.py pinonly / exit_steps_gen.py assemble (c..g, W of the 73 rig bones, Root untouched) and goes through build_entry_steps.py.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import game_skeleton_bridge as CB  # noqa: E402
import exit_steps_gen as ESG  # noqa: E402
import bike_foot_limit as BFL  # noqa: E402

TPOSE = os.path.join(HERE, 'data', 'body_tpose_generator.npz')
SIDES = ('Left', 'Right')


def ik_keep_plane(br, Wt, side, goal):
    """ESG.ik_to with the knee bend direction taken from the frame BEFORE the edit, perpendicular to the OLD hip-ankle chord: ik_to's default (knee - hip projected on the new chord) is dominated by the chord's
    tilt when the leg is nearly straight (a 5 cm lift moved the knee 15-21 cm sideways); legs with no measurable bend (< 2 mm) bend along the foot's forward direction instead"""
    th, ca, fo = (br.uidx[x] for x in br.legs[side])
    hip, knee, ank = Wt[th, :3, 3].copy(), Wt[ca, :3, 3].copy(), Wt[fo, :3, 3].copy()
    u0 = (ank - hip) / np.linalg.norm(ank - hip)
    v = knee - hip
    kp = v - u0 * v.dot(u0)
    if np.linalg.norm(kp) < 0.002:
        tb = br.uidx[side + 'ToeBase']
        fwd = Wt[tb, :3, 3] - Wt[fo, :3, 3]
        kp = fwd - u0 * fwd.dot(u0)
    ESG.ik_to(br, Wt, side, goal, pole_dir=kp)


def local_steps(W, par, joints):
    """angle (deg) between consecutive LOCAL rotations (parent-relative, from the world matrices) of the joints: shape (T-1, len(joints)) - the gate (d) metric of check_bike_entry for these bones"""
    out = []
    for j in joints:
        Rl = np.einsum('tji,tjk->tik', W[:, par[j], :3, :3], W[:, j, :3, :3])
        rel = np.einsum('tji,tjk->tik', Rl[:-1], Rl[1:])
        out.append(np.degrees(np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1))))
    return np.stack(out, 1)


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def softmax_cm(a, b, k=0.01):
    """smooth maximum of a, b (m) that equals a exactly for b <= a - k and b for b >= a + k (quadratic blend between): no residual lift where the shipped foot is already higher (a softplus adds up to
    k * ln 2 = 7 mm everywhere, which is enough to bend an almost straight leg by 6 cm at the knee)"""
    d = b - a
    if d <= -k:
        return a
    if d >= k:
        return b
    return a + (d + k) ** 2 / (4.0 * k)


def local_floor(z, win=8):
    return np.array([z[max(0, f - win):f + win + 1].min() for f in range(len(z))])


def episodes(v, vthr):
    """runs of f with v[f] > vthr, gaps of one frame bridged; returns [(e0, e1)] = stance frame before ... last moving frame (positions e0..e1)"""
    mv = v > vthr
    n = len(v)
    for f in range(1, n - 1):
        if not mv[f] and mv[f - 1] and mv[f + 1]:
            mv[f] = True
    out, f = [], 1
    while f < n:
        if mv[f]:
            e = f
            while e + 1 < n and mv[e + 1]:
                e += 1
            out.append((f - 1, e))
            f = e + 1
        else:
            f += 1
    return out


def clearance_cm(W, names, side):
    """per frame: cm above the local ground (lowest height of the joint inside +-8 frames) of the lower of ankle / toe"""
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
    return np.minimum(za - local_floor(za), zt - local_floor(zt)) * 100


def plan_side(W, names, side, a, scales=None):
    """goal ankle per frame (n, 3) m, or None where unchanged; returns (goals, report)"""
    other = 'Right' if side == 'Left' else 'Left'
    air_o = clearance_cm(W, names, other) > a['hop_other']      # the other foot is off the ground: this foot carries V, its steps stay LOW (a skip, not a hop)
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    A, Tt = W[:, ja, :3, 3], W[:, jt, :3, 3]
    n = len(W)
    va = np.r_[0.0, np.linalg.norm(np.diff(A[:, :2], axis=0), axis=1)] * 100
    vt = np.r_[0.0, np.linalg.norm(np.diff(Tt[:, :2], axis=0), axis=1)] * 100
    v = va                                                         # the swing is the ANKLE's motion; a toe that creeps under a planted ankle is the limiter's job
    ga, gt = local_floor(A[:, 2]), local_floor(Tt[:, 2])
    clr = np.minimum(A[:, 2] - ga, Tt[:, 2] - gt) * 100          # cm above the local ground, lower of ankle / toe
    goal = A.copy()
    rep = []
    for ei, (e0, e1) in enumerate(episodes(v, a['vthr'])):
        L = float(np.linalg.norm((A[e1] - A[e0])[:2])) * 100
        mv = [f for f in range(max(e0 + 1, 1), e1 + 1) if v[f] > a['vthr']]
        if L < a['min_travel'] or not mv:
            rep.append(dict(frames=[e0, e1], travel_cm=round(L, 1), action='none (travel below step size)'))
            continue
        low = min(clr[f] for f in mv)
        if low >= a['clear']:
            rep.append(dict(frames=[e0, e1], travel_cm=round(L, 1), action='none (clears %.1f cm)' % low))
            continue
        h = float(np.clip(a['h_frac'] * L, a['h_min'], a['h_max'])) * a['h_scale'] * (scales or {}).get(ei, 1.0) / 100.0
        hs = np.where(air_o, min(h, a['h_skip'] / 100.0), h)
        hs = np.convolve(np.pad(hs, 2, mode='edge'), np.ones(5) / 5.0, mode='valid')   # smooth the cap in / out (frames)
        pad = int(a['pad'])
        # baseline of the arch: a step that ends ON THE GROUND (ankle within 8 cm of the lowest ankle level of the clip) rises above the straight line between its two stance heights (the start ground of the
        # shipped clip lies ~9 cm above the bike-side ground, so a ground step has a large height difference); a step that ends elsewhere (the over-saddle leg ending on the peg) above the local ground
        ground_end = (A[e1, 2] - A[:, 2].min()) < 0.08
        base_line = np.array([(A[e0, 2] + (A[e1, 2] - A[e0, 2]) * float(np.clip((f - e0) / float(max(e1 - e0, 1)), 0.0, 1.0))) if ground_end else ga[f] for f in range(n)])
        p0, p1 = max(0, e0 - pad), min(n - 1, e1 + pad)       # the arch starts / ends `pad` frames outside the moving frames: the heel rises before the foot moves (smaller knee rate at lift-off)
        for f in range(p0 + 1, p1):
            u = (f - p0) / float(p1 - p0)
            base = base_line[f]
            want = base + float(hs[f]) * np.sin(np.pi * u) ** a['arch_pow']
            goal[f, 2] = softmax_cm(A[f, 2], want)
        rep.append(dict(ep=ei, frames=[e0, e1], p=[p0, p1], travel_cm=round(L, 1), lift_cm=round(h * 100, 1), lift_while_other_foot_up_cm=round(float(hs[e0 + 1:e1].min()) * 100, 1) if e1 > e0 + 1 else None, pad=pad, min_clear_cm=round(float(low), 1), action='arch'))
    return goal, rep


def apply_limiter(br, W, names, a, changed, report):
    """planted remainder: the foot limiter (contact rule), lag handed back in the swing over --decay frames"""
    n = len(W)
    for sd in SIDES:
        con, slide, P = BFL.contact_rule(W, names, sd)
        off = BFL.lag_offsets(con, slide, P, a['limit_vmax'] / 100.0, int(a['decay']), a['omax'] / 100.0)
        off[0] = 0.0
        off[-1] = 0.0
        fo = br.uidx[br.legs[sd][2]]
        goal0 = W[:, fo, :3, 3].copy()
        for t in range(1, n - 1):
            if np.linalg.norm(off[t]) > 1e-5:
                g_ = goal0[t].copy()
                g_[:2] += off[t]
                ik_keep_plane(br, W[t], sd, g_)
                changed[t] = True
        report[sd + '_limiter_lag_max_cm'] = round(float(np.linalg.norm(off, axis=1).max()) * 100, 1)


def limit_knee_rate(br, W0, side, goal, a):
    """bound the knee angle rate of the planned ankle path: the shipped legs are fully extended (hip-ankle chord = 0.999 of the leg) at toe-off, where the knee angle theta = 2 acos(chord / leg) changes by
    > 15 deg / frame for a 1 cm lift. theta of the goal path may not rise (forward pass) or fall (backward pass) faster than max(--knee-rate deg, the shipped path's own rate) per frame; the ankle height is
    re-solved for the limited chord (horizontal ankle position kept)"""
    th, ca, fo = (br.uidx[x] for x in br.legs[side])
    n = len(W0)
    hip = W0[:, th, :3, 3]
    Lg = np.linalg.norm(W0[:, ca, :3, 3] - hip, axis=1) + np.linalg.norm(W0[:, fo, :3, 3] - W0[:, ca, :3, 3], axis=1)
    def theta(P):
        r = np.clip(np.linalg.norm(P - hip, axis=1) / Lg, 0.0, 1.0)
        return np.degrees(2 * np.arccos(r))
    t0, t1 = theta(W0[:, fo, :3, 3]), theta(goal)
    lim = np.maximum(a['knee_rate'], np.r_[0.0, np.abs(np.diff(t0))])
    moved = np.flatnonzero(np.linalg.norm(goal - W0[:, fo, :3, 3], axis=1) > 1e-5)
    if not len(moved):
        return goal
    t = t1.copy()
    for f in range(1, n):
        t[f] = min(t[f], t[f - 1] + lim[f])            # may not bend faster than the limit
    for f in range(n - 2, -1, -1):
        t[f] = min(t[f], t[f + 1] + lim[f + 1])        # may not straighten faster than the limit
    out = goal.copy()
    for f in moved:
        if t[f] < t1[f] - 1e-6:                        # only ever reduce the bend (= lower the foot)
            rL = np.cos(np.radians(t[f]) / 2) * Lg[f]
            dxy = goal[f, :2] - hip[f, :2]
            z2 = rL * rL - float(dxy.dot(dxy))
            if z2 > 0:
                out[f, 2] = max(W0[f, fo, 2, 3], hip[f, 2] - np.sqrt(z2))
    return out


def toe_pitch(br, W, W0, side, a, changed):
    """a toe that drags over the floor while the ankle is already up (the over-saddle foot's toe-off: ankle 12-20 cm up, toe pointing down at 0-1 cm) cannot be lifted by the ankle arch (the foot keeps its world
    orientation). Pitch the foot (foot + toe subtree) about the ankle's lateral axis, toe up, by the angle that brings the toe --toe-clear cm above its local ground (lowest toe height of the SHIPPED clip inside
    +-8 frames) in the frames where the ankle or toe moves faster than --vthr (+-2 frames); the angle is smoothed in time and capped at --toe-max degrees"""
    th, ca, fo = (br.uidx[x] for x in br.legs[side])
    tb = br.uidx[side + 'ToeBase']
    n = len(W)
    gt = local_floor(W0[:, tb, 2, 3])
    va = np.r_[0.0, np.linalg.norm(np.diff(W0[:, fo, :2, 3], axis=0), axis=1)] * 100
    vt = np.r_[0.0, np.linalg.norm(np.diff(W0[:, tb, :2, 3], axis=0), axis=1)] * 100
    mv = np.maximum(va, vt) > a['vthr']
    mv = np.convolve(mv.astype(float), np.ones(5), mode='same') > 0
    ang = np.zeros(n)
    for f in range(1, n - 1):
        if not mv[f]:
            continue
        need = a['toe_clear'] / 100.0 - (W[f, tb, 2, 3] - gt[f])
        if need <= 0:
            continue
        ln = float(np.linalg.norm(W[f, tb, :3, 3] - W[f, fo, :3, 3]))
        ang[f] = np.arcsin(np.clip(need / max(ln, 1e-3), 0.0, 0.7))
    for _ in range(2):
        ang = np.convolve(np.pad(ang, 1, mode='edge'), [0.25, 0.5, 0.25], mode='valid')
    ang = np.minimum(ang, np.radians(a['toe_max']))
    ang[0] = ang[-1] = 0.0
    moved = 0
    for f in range(1, n - 1):
        if ang[f] < np.radians(0.5):
            continue
        fwd = W[f, tb, :3, 3] - W[f, fo, :3, 3]
        fwd[2] = 0.0
        if np.linalg.norm(fwd) < 1e-4:
            continue
        fwd /= np.linalg.norm(fwd)
        ax = np.cross(fwd, [0.0, 0.0, 1.0])             # lateral axis; a positive angle about it (right-hand rule) turns the forward direction toward +z (toe up)
        ax /= np.linalg.norm(ax)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + np.sin(ang[f]) * K + (1 - np.cos(ang[f])) * (K @ K)
        br._rotate_subtree(W[f], fo, R, W[f, fo, :3, 3].copy())
        changed[f] = True
        moved += 1
    return moved, round(float(np.degrees(ang.max())), 1)


def main():
    argv = sys.argv[1:]
    a = dict(h_frac=0.25, h_min=4.0, h_max=9.0, clear=4.5, min_travel=7.0, vthr=1.0, limit_vmax=0.8, hop_other=6.0, h_skip=5.0, arch_pow=2.0, pad=2, h_scale=1.0, step_lim=13.5, iters=8, decay=6, omax=14.0, knee_rate=10.0, toe_clear=0.0, toe_max=40.0)
    pos = []
    k = 0
    while k < len(argv):
        s = argv[k]
        if s.startswith('--'):
            key = s[2:].replace('-', '_')
            a[key] = float(argv[k + 1])
            k += 2
        else:
            pos.append(s)
            k += 1
    dump, out = pos[0], pos[1]
    br, d = CB.load_cp_bridge(TPOSE, dump)
    W0 = d['W'].copy()
    W = W0.copy()
    names = [str(x) for x in d['names']]
    n = len(W)
    report = {}
    changed = np.zeros(n, bool)
    par = [int(x) for x in d['parents']]
    scales = {sd: {} for sd in SIDES}
    for it in range(int(a['iters'])):
        W = W0.copy()
        changed[:] = False
        viol = False
        reps = {}
        for sd in SIDES:
            goal, rep = plan_side(W0, names, sd, a, scales[sd])
            if a['knee_rate'] > 0:
                goal = limit_knee_rate(br, W0, sd, goal, a)
            fo = br.uidx[br.legs[sd][2]]
            for t in range(1, n - 1):
                if np.linalg.norm(goal[t] - W0[t, fo, :3, 3]) > 1e-5:
                    ik_keep_plane(br, W[t], sd, goal[t])
                    changed[t] = True
            report[sd] = reps[sd] = rep
        if a['limit_vmax'] > 0:
            apply_limiter(br, W, names, a, changed, report)
        if a['toe_clear'] > 0:
            for sd in SIDES:
                report[sd + '_toe_pitch'] = toe_pitch(br, W, W0, sd, a, changed)
        for sd in SIDES:
            js = [br.uidx[x] for x in br.legs[sd]] + [br.uidx[sd + 'ToeBase']]
            st0, st1 = local_steps(W0, par, js), local_steps(W, par, js)
            bad = np.flatnonzero((st1 > np.maximum(a['step_lim'], st0 * 1.05)).any(axis=1))      # step f is between frames f and f + 1
            for r in reps[sd]:
                if r.get('action') == 'arch' and any(r['p'][0] - 1 <= f <= r['p'][1] for f in bad):
                    scales[sd][r['ep']] = scales[sd].get(r['ep'], 1.0) * 0.8
                    viol = True
            report[sd + '_step_violations'] = [int(f) for f in bad]
        if not viol:
            break
    report['adapt_iterations'] = it + 1
    report['episode_scales'] = {sd: {str(k): round(v, 3) for k, v in scales[sd].items()} for sd in SIDES}
    fr = np.flatnonzero(changed)
    if not len(fr):
        print('GAIT nothing to change')
        return 1
    c, g = max(1, int(fr.min()) - 1), min(n - 2, int(fr.max()) + 1)
    np.savez(out, W=W[c:g + 1], c=c, g=g, dz=0.0, foot_contacts=np.zeros((g - c + 1, 4)), names=np.array(names), parents=d['parents'], leg_ik=True, src='bike_entry_gait')
    json.dump(report, open(out + '.report.json', 'w'), indent=1)
    for sd in SIDES:
        for r in report[sd]:
            print('GAIT %-5s %s' % (sd, r))
    print('GAIT %s span %d..%d changed frames %d' % (out, c, g, int(changed.sum())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
