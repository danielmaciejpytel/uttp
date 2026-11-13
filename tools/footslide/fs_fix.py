"""Offline foot pin ("correction A") for utppclip locomotion clips (game rig, 30 fps): cancels the horizontal drift of the planted foot's
ground contact point with a two-bone leg IK, leaving Root / Trajectory / Hips / spine / arms bit-identical.

python -I fs_fix.py <woman|man> <src clip dir> <out dir> <report.json> [--names regex] [--min-worst 6] [--ramp 8] [--all]

Per foot: plant = contact curve >= 0.5 AND heel or toe within 1.5 cm of its ground level (fs_lib.planted), gaps <= 2 frames merged. Contact point of a
frame pair = the lower of Heel/ToeBase (flat foot: the smaller displacement). Inside a plant its horizontal motion is cancelled frame by frame
(a roll heel->toe stays natural; the foot keeps its world rotation, so a pivot turns about the contact point). The accumulated offset moves the
ankle target (height untouched) and is faded out after the plant with a smoothstep over --ramp frames (before the plant when the plant touches
the clip end). A plant touching both clip ends keeps frames 0 / N-1 exact and only the straight-line net drift remains.
Gates (all must hold, else the clip is not written): worst single-plant drift >= --min-worst cm, drift path reduced >= 50 % and worst single plant <= 60 % of the source, no new per-frame jump in a plant (max pair <= max(source, 1 cm) + 0.5), Root/Trajectory/Hips/
non-leg bones unchanged, leg bone step / second difference not above source + 4 deg, heel/toe never lower than source - 0.5 cm, <= 1 reach-clamped
frame, knee straightness <= 0.998, |offset| <= 20 cm, knee off-hinge <= source + 1 deg (the source Leg rotation is a pure hinge), local Foot angle change <= 25 deg.
"""
import json
import os
import re
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L  # noqa: E402

v1 = L.v1
qmul, qrot = v1.qmul, v1.qrot
REACH = 0.985
MAX_UNWIND = 0.02   # m per frame (offset fade-out / fade-in speed cap)
DEL = np.linspace(-0.9, 0.9, 1801)


def rotvec(q):
    q = q * np.where(q[..., 3] < 0, -1.0, 1.0)[..., None]
    ang = 2 * np.arccos(np.clip(q[..., 3], -1, 1))
    n = np.linalg.norm(q[..., :3], axis=-1, keepdims=True)
    return np.where(n > 1e-9, q[..., :3] / np.maximum(n, 1e-9), 0.0) * ang[..., None]


def hinge_axis(qleg):
    """dominant rotation axis of the source Leg local quaternions (SVD of the rotation vectors), sign so the mean flex is positive."""
    rv = rotvec(qleg)
    ax = np.linalg.svd(rv, full_matrices=False)[2][0]
    return ax if (rv @ ax).sum() >= 0 else -ax


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def q_between(u, v):
    u = u / np.linalg.norm(u)
    v = v / np.linalg.norm(v)
    c = float(u @ v)
    if c < -0.999999:
        a = np.cross(u, [1.0, 0.0, 0.0])
        if np.linalg.norm(a) < 1e-3:
            a = np.cross(u, [0.0, 1.0, 0.0])
        a /= np.linalg.norm(a)
        return np.array([a[0], a[1], a[2], 0.0])
    q = np.array([*np.cross(u, v), 1.0 + c])
    return q / np.linalg.norm(q)


def smooth(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * x * (x * (6 * x - 15) + 10)


def plant_segments(mask, h=None, t=None, gap=2, max_move=0.015):
    """plant runs of `mask`; runs separated by <= gap frames are merged only when the contact point did not travel more than max_move there
    (a flicker of the height test, not a step)."""
    m = mask.copy()
    segs = L.segments(m)
    for (b0, e0), (b1, e1) in zip(segs[:-1], segs[1:]):
        if b1 - e0 - 1 <= gap:
            if h is not None:
                pa = h if contact_choice(h, t, e0 if e0 + 1 < len(h) else e0 - 1) == 0 else t
                if np.linalg.norm(pa[b1, :2] - pa[e0, :2]) > max_move:
                    continue
            m[e0 + 1:b1] = True
    return [(b, e) for (b, e) in L.segments(m) if e - b >= 2]


def contact_choice(h, t, k, flat_h=L.FLAT_H):
    """0 = heel, 1 = toe for the pair (k, k+1) (same rule as fs_lib.contact_point_slide)."""
    dh = np.linalg.norm(h[k + 1, :2] - h[k, :2])
    dt = np.linalg.norm(t[k + 1, :2] - t[k, :2])
    hz, tz = 0.5 * (h[k, 2] + h[k + 1, 2]), 0.5 * (t[k, 2] + t[k + 1, 2])
    if min(hz, tz) < flat_h and abs(hz - tz) < flat_h:
        return 0 if dh <= dt else 1
    return 0 if hz < tz else 1


def offsets_for_side(c, side, ramp, gh, gt):
    """(N,2) world horizontal ankle offsets (m) for one foot, plus segment list."""
    a, h, t = c.foot_points(side)
    N = c.N
    pl = L.planted(c, side, gh=gh, gt=gt)
    segs = plant_segments(pl, h, t)
    o = np.zeros((N, 2))
    for si, (b, e) in enumerate(segs):
        inc = np.zeros((N, 2))   # inc[k] = cp(k+1)-cp(k) of the chosen contact joint
        for k in range(b, e):
            p = h if contact_choice(h, t, k) == 0 else t
            inc[k] = p[k + 1, :2] - p[k, :2]
        touch0, touch1 = b == 0, e == N - 1
        seg = np.zeros((N, 2))
        if touch1 and not touch0:
            for k in range(e - 1, b - 1, -1):
                seg[k] = seg[k + 1] + inc[k]
        else:
            for k in range(b, e):
                seg[k + 1] = seg[k] - inc[k]
            if touch0 and touch1:
                f = (np.arange(b, e + 1) - b) / float(e - b)
                seg[b:e + 1] -= f[:, None] * seg[e]
        # the offset has to unwind in the swing before the next plant: smoothstep slope <= 1.9 / R per frame -> cap the end value so the unwind
        # moves the foot by <= MAX_UNWIND per frame (short swings get a partial pin instead of a pop)
        if not touch1:
            nxt0 = segs[si + 1][0] if si + 1 < len(segs) else N
            R0 = int(max(1, min(ramp, nxt0 - e)))
            end = np.linalg.norm(seg[e])
            if end * 1.9 / R0 > MAX_UNWIND:
                seg *= MAX_UNWIND * R0 / (1.9 * end)
        if touch1 and not touch0:
            prv0 = segs[si - 1][1] if si > 0 else -1
            R0 = int(max(1, min(ramp, b - prv0)))
            start = np.linalg.norm(seg[b])
            if start * 1.9 / R0 > MAX_UNWIND:
                seg *= MAX_UNWIND * R0 / (1.9 * start)
        o[b:e + 1] = seg[b:e + 1]
        if not touch1:
            nxt = segs[si + 1][0] if si + 1 < len(segs) else N
            R = int(max(1, min(ramp, nxt - e)))
            for j in range(1, R + 1):
                if e + j < N:
                    o[e + j] = seg[e] * (1.0 - smooth(j / R))
        if touch1 and not touch0:
            prv = segs[si - 1][1] if si > 0 else -1
            R = int(max(1, min(ramp, b - prv)))
            for j in range(1, R + 1):
                if b - j >= 0:
                    o[b - j] = seg[b] * (1.0 - smooth(j / R))
    return o, segs


def leg_ik_all(c, offs, gp, gq):
    """Apply per-side world offsets via two-bone IK. Returns (Q', clamped_frames, straight_ratio_after, degenerate)."""
    bi = c.bi
    Q = c.Q.copy()
    tq = gq[:, bi['Trajectory']]
    hips_q = gq[:, bi['Hips']]
    clamped = 0
    maxratio = 0.0
    degen = 0
    for side in ('Left', 'Right'):
        iu, il, jf = bi[side + 'UpLeg'], bi[side + 'Leg'], bi[side + 'Foot']
        o = offs[side].copy()
        # soft reach limit: scale the offset so the ankle stays within REACH of the full leg length (smoothed, never above the raw limit)
        sc = np.ones(c.N)
        for k in range(c.N):
            if np.abs(o[k]).max() < 1e-7:
                continue
            A, B, C = gp[k, iu], gp[k, il], gp[k, jf]
            R = (np.linalg.norm(B - A) + np.linalg.norm(C - B)) * REACH
            m = qrot(qconj(tq[k]), np.array([o[k, 0], o[k, 1], 0.0]))
            d0 = C - A
            f0 = d0 @ d0
            cap = max(R * R, f0)
            best = 0.0
            for sv in np.linspace(0.0, 1.0, 41):
                dd = d0 + sv * m
                if dd @ dd <= cap + 1e-12:
                    best = sv
            sc[k] = best
        if sc.min() < 1.0:
            w = 5
            mn = np.array([sc[max(0, i - w // 2 - 1):i + w // 2 + 2].min() for i in range(c.N)])
            sc = np.convolve(np.pad(mn, (w, w), mode='edge'), np.ones(2 * w + 1) / (2 * w + 1), mode='same')[w:-w]
            sc = np.minimum(sc, 1.0)
        o = o * sc[:, None]
        # hinge-exact knee: the source Leg rotation is a pure hinge about one fixed axis (retarget knee_fix v2 / TwistFix); the new ankle
        # distance is reached by changing only the hinge angle, then the thigh swings (minimal rotation) so the ankle hits the goal.
        axis = hinge_axis(c.Q[:, il])
        rv0 = rotvec(c.Q[:, il])
        for k in range(c.N):
            A, B, C = gp[k, iu], gp[k, il], gp[k, jf]
            la, lb = np.linalg.norm(B - A), np.linalg.norm(C - B)
            if np.abs(o[k]).max() < 1e-7:
                maxratio = max(maxratio, np.linalg.norm(C - A) / (la + lb))
                continue
            wshift = np.array([o[k, 0], o[k, 1], 0.0])
            mshift = qrot(qconj(tq[k]), wshift)
            goal = C + mshift
            d = goal - A
            dist = np.linalg.norm(d)
            lim = (la + lb) * 0.999
            if dist > lim:
                clamped += 1
            dist_c = float(np.clip(dist, abs(la - lb) + 1e-4, lim))
            n = d / dist
            th0 = float(rv0[k] @ axis)
            ths = th0 + DEL
            qh = np.concatenate([axis[None, :] * np.sin(ths / 2)[:, None], np.cos(ths / 2)[:, None]], 1)
            u = c.P[k, il][None, :] + qrot(qh, c.P[k, jf][None, :])
            err = np.abs(np.linalg.norm(u, axis=1) - dist_c) + 1e-6 * np.abs(DEL)
            err[ths < min(th0, 0.0)] = 1e9   # no knee hyperextension branch
            m = int(np.argmin(err))
            qleg = qh[m]
            C1 = A + qrot(gq[k, iu], u[m])
            r = q_between(C1 - A, n * dist_c)
            gq_up = qmul(r, gq[k, iu])
            gq_leg = qmul(gq_up, qleg)
            Q[k, iu] = qmul(qconj(hips_q[k]), gq_up)
            Q[k, il] = qleg
            Q[k, jf] = qmul(qconj(gq_leg), gq[k, jf])
            maxratio = max(maxratio, dist_c / (la + lb))
    # hemisphere continuity of the touched quaternions
    for side in ('Left', 'Right'):
        for nm in ('UpLeg', 'Leg', 'Foot'):
            j = bi[side + nm]
            for k in range(c.N):
                if float(Q[k, j] @ c.Q[k, j]) < 0:
                    Q[k, j] = -Q[k, j]
    return Q, clamped, maxratio, degen


def ang_step(Q):
    d = np.abs(np.sum(Q[1:] * Q[:-1], -1))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1)))


def leg_stats(c, Q):
    out = {}
    for side in ('Left', 'Right'):
        for nm in ('UpLeg', 'Leg', 'Foot'):
            j = c.bi[side + nm]
            s = ang_step(Q[:, j])
            out[side + nm] = (float(s.max()), float(np.abs(np.diff(s)).max()) if len(s) > 1 else 0.0)
    return out


def process(c, ramp=10, min_worst=6.0):
    """returns (new B array or None, report dict)"""
    gp, gq, wp = c.world()
    gh, gt = L.ground_levels(c)
    offs, segs = {}, {}
    for side in ('Left', 'Right'):
        offs[side], segs[side] = offsets_for_side(c, side, ramp, gh, gt)
    before = L.measure(c)
    rep = dict(name=os.path.basename(c.path)[:-14], worst_before=max(before[s]['worst_seg'] for s in ('Left', 'Right')),
               path_before=sum(before[s]['path'] for s in ('Left', 'Right')),
               max_offset_cm=float(max(np.abs(offs[s]).max() for s in offs) * 100))
    if not any(np.abs(offs[s]).max() > 1e-7 for s in offs):
        rep['verdict'] = 'no plant'
        return None, rep
    ratio0 = 0.0
    for side in ('Left', 'Right'):
        iu, il, jf = c.bi[side + 'UpLeg'], c.bi[side + 'Leg'], c.bi[side + 'Foot']
        for k in range(c.N):
            A, B, C = gp[k, iu], gp[k, il], gp[k, jf]
            ratio0 = max(ratio0, np.linalg.norm(C - A) / (np.linalg.norm(B - A) + np.linalg.norm(C - B)))
    Q, clamped, maxratio, degen = leg_ik_all(c, offs, gp, gq)
    # (maxratio includes the untouched frames; the gate compares with the source value)
    # build the new clip data and re-measure with the same measure code
    c2 = L.Clip.__new__(L.Clip)
    c2.__dict__.update(c.__dict__)
    c2.Q = Q
    c2.B = c.B.copy()
    c2.B[..., 3:7] = Q
    after = L.measure(c2)
    rep.update(worst_after=max(after[s]['worst_seg'] for s in ('Left', 'Right')), path_after=sum(after[s]['path'] for s in ('Left', 'Right')),
               clamped=clamped, degenerate=degen, straight=float(maxratio), straight_src=float(ratio0))
    changed = np.abs(Q - c.Q).max(axis=(0, 2)) > 0
    allowed = {c.bi[s + n] for s in ('Left', 'Right') for n in ('UpLeg', 'Leg', 'Foot')}
    rep['bones_changed_ok'] = bool(all(j in allowed for j in np.nonzero(changed)[0]))
    s0, s1 = leg_stats(c, c.Q), leg_stats(c2, Q)
    rep['step_worse'] = max(s1[k][0] - s0[k][0] for k in s0)
    rep['jerk_worse'] = max(s1[k][1] - s0[k][1] for k in s0)
    zmin0 = min(c.foot_points(s)[1][:, 2].min() for s in ('Left', 'Right')), min(c.foot_points(s)[2][:, 2].min() for s in ('Left', 'Right'))
    zmin1 = min(c2.foot_points(s)[1][:, 2].min() for s in ('Left', 'Right')), min(c2.foot_points(s)[2][:, 2].min() for s in ('Left', 'Right'))
    rep['z_drop_cm'] = float(max(zmin0[0] - zmin1[0], zmin0[1] - zmin1[1]) * 100)
    off0 = off1 = foot = 0.0
    for side in ('Left', 'Right'):
        il, jf = c.bi[side + 'Leg'], c.bi[side + 'Foot']
        ax = hinge_axis(c.Q[:, il])
        for QQ, key in ((c.Q, 0), (Q, 1)):
            rv = rotvec(QQ[:, il])
            off = np.degrees(np.linalg.norm(rv - (rv @ ax)[:, None] * ax[None, :], axis=1)).max()
            if key == 0:
                off0 = max(off0, off)
            else:
                off1 = max(off1, off)
        dd = np.abs(np.sum(c.Q[:, jf] * Q[:, jf], -1))
        foot = max(foot, float(np.degrees(2 * np.arccos(np.clip(dd, -1, 1))).max()))
    rep.update(off_hinge_src=float(off0), off_hinge_fixed=float(off1), foot_change_deg=float(foot))
    fails = []
    if off1 > off0 + 1.0:
        fails.append('knee_twist')
    if foot > 25.0:
        fails.append('foot_angle')
    if rep['worst_before'] < min_worst:
        fails.append('worst<%.0f' % min_worst)
    if rep['path_after'] > 0.5 * rep['path_before'] or rep['worst_after'] > 0.6 * rep['worst_before']:
        fails.append('path_gain')
    if not rep['bones_changed_ok']:
        fails.append('bones')
    if rep['step_worse'] > 4.0:
        fails.append('step')
    if rep['jerk_worse'] > 4.0:
        fails.append('jerk')
    if rep['z_drop_cm'] > 0.5:
        fails.append('z')
    mx0 = max(before[sd]['max'] for sd in ('Left', 'Right'))
    mx1 = max(after[sd]['max'] for sd in ('Left', 'Right'))
    rep.update(max_pair_before=float(mx0), max_pair_after=float(mx1))
    if mx1 > max(mx0, 1.0) + 0.5:
        fails.append('pop')
    if clamped > 1:
        fails.append('clamp')
    if maxratio > max(0.998, ratio0 + 0.0005):
        fails.append('straight')
    if rep['max_offset_cm'] > 20.0:
        fails.append('offset')
    rep['verdict'] = 'ok' if not fails else 'fail:' + ','.join(fails)
    return c2.B if not fails else None, rep


def write_clip(c, B, outpath, note):
    d = json.loads(json.dumps(c.d))   # deep copy
    for k, fr in enumerate(d['frames']):
        fr['bones'] = [[float(x) for x in row] for row in B[k]]
    d['retarget']['foot_pin'] = note
    tmp = outpath + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(d, f, separators=(',', ':'))
    os.replace(tmp, outpath)


if __name__ == '__main__':
    g, src, out, repp = sys.argv[1:5]
    args = sys.argv[5:]
    rx = re.compile(args[args.index('--names') + 1] if '--names' in args else r'(_Turn_|_Pivot_|_Reface_|Idle_turn|Idle_Turn)')
    min_worst = float(args[args.index('--min-worst') + 1]) if '--min-worst' in args else 6.0
    ramp = int(args[args.index('--ramp') + 1]) if '--ramp' in args else 10
    os.makedirs(out, exist_ok=True)
    note = dict(version=1, tool='Source/tools/footslide/fs_fix.py', method='planted-foot contact-point pin, two-bone leg IK, smoothstep fade, Root/Trajectory/Hips unchanged',
                ramp=ramp, plant_h_m=L.PLANT_H)
    reps = []
    for f in sorted(os.listdir(src)):
        if not f.endswith('.utppclip.json'):
            continue
        n = f[:-14]
        if not rx.search(n) or n.startswith('AO_'):
            continue
        c = L.Clip(os.path.join(src, f), g)
        if 'contact_l' not in c.c:
            continue
        B, rep = process(c, ramp, min_worst)
        reps.append(rep)
        if B is not None:
            write_clip(c, B, os.path.join(out, f), note)
        print('%-44s %-22s worst %5.1f->%5.1f path %5.1f->%5.1f off %4.1f cm' % (n, rep['verdict'], rep['worst_before'], rep.get('worst_after', -1), rep['path_before'], rep.get('path_after', -1), rep['max_offset_cm']))
    json.dump(reps, open(repp, 'w'), indent=1)
    print('written', sum(r['verdict'] == 'ok' for r in reps), 'of', len(reps))
