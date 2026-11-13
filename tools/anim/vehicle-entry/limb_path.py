"""Limb path layer for the car entry clips (round 7, utility rows): edits of the FINISHED pose that remove (1) contact of a hand / forearm / shin with the B-pillar jamb strip
('jamb' phase of check_door_clearance.py) and (2) the rearward slide of the hips while they sit down ('sink slide' of start_gate.py), without touching the head, the chest
chain, the door-side limbs or the last frames (the seated pose is never changed).

Applied after --head-duck, before --limb-ik-final (build_entry_clip.py --limb-path), or stand-alone on a builder GLB:

  python -I limb_path.py --in <builder.glb> --out <edited.glb> --target <vanilla player glb> --class utility --seat driver [options]

(1) JAMB ESCAPE (--lp-limbs RightHand,RightFoot ...): per iteration the penetration of the limb capsules (hand / forearm / upper arm, shin / thigh; hand_r ... naming of
    check_door_clearance.BODY) into the jamb strip cloud is measured at the clip frames in the vehicle frame; the gate per capsule is the one of the check
    (max(1 cm, seated baseline + 0.5, the game's own clip worst + 0.5)) minus --lp-slack-cm. A capsule over its gate asks for a displacement along the escape direction
    (capsule sample point - nearest strip point) of its end effector and / or its middle joint (elbow / knee). The requests accumulate (damped), are smoothed in time and
    zero at the first / last frames; each iteration re-solves the limb by two-bone IK from the untouched pose (end goal, middle joint pole). Frames without an edit stay bit-identical.
(2) HIPS CAP (--lp-hips-cap L): from the highest hips frame on the vehicle-frame y of the Hips may not fall back below its LATER values (reverse running minimum from the last
    frame, blended by L); the shortfall is a rigid translation of the Hips (the whole upper body follows), feet held in world by two-bone IK, arms follow. 0 at the last frame.
Report: rep['limb_path'].
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import ik as IK  # noqa: E402
import head_duck as HD  # noqa: E402

LIMBS = {
    'RightHand': dict(chain=('RightArm', 'RightForeArm', 'RightHand'), end=('hand_r', 'farm_r'), mid=('farm_r', 'uarm_r')),
    'LeftHand': dict(chain=('LeftArm', 'LeftForeArm', 'LeftHand'), end=('hand_l', 'farm_l'), mid=('farm_l', 'uarm_l')),
    'RightFoot': dict(chain=('RightUpLeg', 'RightLeg', 'RightFoot'), end=('shin_r',), mid=('shin_r', 'thigh_r')),
    'LeftFoot': dict(chain=('LeftUpLeg', 'LeftLeg', 'LeftFoot'), end=('shin_l',), mid=('shin_l', 'thigh_l')),
}


def add_args(p):
    p.add_argument('--limb-path', action='store_true', help='limb path layer (limb_path.py) on the finished pose, after --head-duck')
    p.add_argument('--lp-json', help='body-data/<class>.json of the class (strip cloud of the jamb phase); required for --lp-limbs')
    p.add_argument('--lp-seat', choices=['left', 'right'], default='left')
    p.add_argument('--lp-limbs', default='', help='comma list of RightHand,LeftHand,RightFoot,LeftFoot that escape the jamb strip')
    p.add_argument('--lp-slack-cm', type=float, default=0.6, help='cm: the escape aims this far under the gate of the jamb phase')
    p.add_argument('--lp-iters', type=int, default=14)
    p.add_argument('--lp-damp', type=float, default=0.8)
    p.add_argument('--lp-sigma', type=float, default=1.5, help='frames: Gaussian smoothing of the accumulated displacements')
    p.add_argument('--lp-zero-first', type=int, default=4, help='frames at the start without edits')
    p.add_argument('--lp-zero-last', type=int, default=14, help='frames at the end without edits (the seated pose and its end blend)')
    p.add_argument('--lp-max-cm', type=float, default=14.0, help='cm: largest accumulated displacement of a goal')
    p.add_argument('--lp-dir', default='', help='x,y,z vehicle-frame escape direction (e.g. 0,1,0 = forward into the door opening) instead of the nearest-point normal: the strip is the SOLID pillar behind the aperture, the normals flip inside it')
    p.add_argument('--lp-hips-cap', type=float, default=0.0, help='0..1: share of the hips overshoot (rearward travel while sitting down) removed; 0 = off')
    p.add_argument('--lp-hips-mode', choices=['cap', 'iso'], default='cap', help='cap = reverse running minimum (only removes the forward overshoot); iso = isotonic regression of the hips y (nearest non-decreasing path: also lifts the rearward dips of the climb)')
    p.add_argument('--lp-hips-start', type=int, default=0, help='iso mode: first frame of the regression')
    p.add_argument('--lp-hips-pin', type=int, default=6, help='iso mode: the last N frames keep their y (monotone in the clips; larger values pin the overshoot itself)')
    p.add_argument('--lp-hips-tail', type=int, default=10, help='iso mode: frames of the smoothstep fade of the hips correction to 0 before the last frame')
    p.add_argument('--lp-hips-feet', choices=['follow', 'hold', 'replant'], default='follow', help='feet during the hips correction: swinging feet follow the shift (planted hold) | every foot holds its world position (IK) | replant: every planted run is moved by the hips shift, swings blend')
    p.add_argument('--lp-feet-sigma', type=float, default=0.0, help='frames: extra Gaussian smoothing of the follow weight of the swinging feet (slower hand-over between hold and follow)')
    p.add_argument('--lp-leg-step-limit', type=float, default=0.0, help='deg/frame: after the hips correction the leg rotations inside the edited frames are smoothed until no step exceeds this (check (d) allows 15)')
    p.add_argument('--lp-sink-profile', default='', help="'frame:cm,...' Hips drop (feet held) per clip frame, cosine easing, 0 outside the knots; applied first")
    p.add_argument('--lp-fwd-bump', default='', help="'frame:cm,...' extra FORWARD hips shift (vehicle y) per clip frame, cosine easing, 0 outside the knots; needs --lp-hips-cap > 0")
    p.add_argument('--lp-hips-sigma', type=float, default=2.0, help='frames: smoothing of the hips correction')
    p.add_argument('--lp-hips-from', type=int, default=-1, help='first frame of the cap (default: the frame of the highest hips)')
    p.add_argument('--lp-hips-hands', choices=['follow', 'lock'], default='follow', help='arms during the hips cap: follow the body or hold their world position (IK)')


class Strip:
    """Jamb strip cloud of a class in the vehicle frame (same selection as check_door_clearance.Model)."""

    def __init__(self, cls, side):
        import check_door_clearance as C
        import start_gate as SG
        self.C = C
        tj = C.load_timing()
        bdd = C.load_body(C.DATA, cls)
        ar = tj['arcs'][cls][side]
        import math as _m
        y_edge = ar['hingeY'] + ar['length'] * _m.sin(_m.radians(ar['closedDirDeg']))
        zs_, ys_, _ = SG.aperture_rear(bdd, side, y_edge)
        cl = bdd['cloud']
        lim_ = np.interp(cl[:, 2], zs_, ys_)
        jm = (cl[:, 1] > y_edge + C.BODY_BEHIND_EDGE_M) & (cl[:, 1] <= lim_) & (cl[:, 2] < bdd['seat_z'] + C.BODY_UPPER_ABOVE_SEAT_M)
        c = cl[jm]
        o = np.argsort(c[:, 0], kind='stable')
        self.cloud = np.ascontiguousarray(c[o]).astype(np.float32)
        self.xs = np.ascontiguousarray(self.cloud[:, 0])


def _starts(cid):
    return np.flatnonzero(np.r_[True, np.diff(cid) != 0])


def capsule_pen(strip, Vv, rad, starts, names, want, with_dir=True):
    """Vv (P,3) vehicle-frame sample points of ONE frame -> {capsule: (pen m, escape dir (3,), worst sample point index range start)} for the wanted capsules."""
    out = {}
    for ci, nm in enumerate(names):
        if nm not in want:
            continue
        s = starts[ci]
        e = starts[ci + 1] if ci + 1 < len(starts) else len(Vv)
        q = Vv[s:e].astype(np.float32)
        m = float(rad[s]) + 0.1
        lo, hi = q.min(0) - m, q.max(0) + m
        i0, i1 = np.searchsorted(strip.xs, lo[0]), np.searchsorted(strip.xs, hi[0])
        if i1 <= i0:
            out[nm] = (-0.15, np.zeros(3), 0.0)
            continue
        sub = strip.cloud[i0:i1]
        k = np.flatnonzero((sub[:, 1] >= lo[1]) & (sub[:, 1] <= hi[1]) & (sub[:, 2] >= lo[2]) & (sub[:, 2] <= hi[2]))
        if len(k) == 0:
            out[nm] = (-0.15, np.zeros(3), 0.0)
            continue
        pts = sub[k]
        d2 = ((q[:, None, :] - pts[None, :, :]) ** 2).sum(-1)
        j = np.unravel_index(int(d2.argmin()), d2.shape)
        d = math.sqrt(float(d2[j]))
        pen = float(rad[s]) - d
        v = q[j[0]] - pts[j[1]]
        n = v / max(float(np.linalg.norm(v)), 1e-6)
        frac = j[0] / max(1.0, len(q) - 1.0)
        out[nm] = (max(pen, -0.15), n, frac)
    return out


def push_need(strip, q0, rad_c, gate, d, smax=0.22, step=0.01):
    """Smallest displacement s (m) of the capsule sample points q0 (n,3) along the unit vector d with penetration <= gate (m); smax if none."""
    s = 0.0
    while s <= smax + 1e-9:
        q = (q0 + s * d).astype(np.float32)
        m = rad_c + 0.1
        lo, hi = q.min(0) - m, q.max(0) + m
        i0, i1 = np.searchsorted(strip.xs, lo[0]), np.searchsorted(strip.xs, hi[0])
        pen = -0.15
        if i1 > i0:
            sub = strip.cloud[i0:i1]
            k = np.flatnonzero((sub[:, 1] >= lo[1]) & (sub[:, 1] <= hi[1]) & (sub[:, 2] >= lo[2]) & (sub[:, 2] <= hi[2]))
            if len(k):
                d2 = ((q[:, None, :] - sub[k][None, :, :]) ** 2).sum(-1)
                pen = rad_c - math.sqrt(float(d2.min()))
        if pen <= gate:
            return s
        s += step
    return smax


class Eval:
    """Pose -> jamb penetration of chosen capsules, in the vehicle frame of the pose's own last root (seat frame)."""

    def __init__(self, tgt, ctx, strip):
        self.tgt, self.ctx, self.strip = tgt, ctx, strip

    def pens(self, nlt, nlr, want, mid=True):
        import check_door_clearance as C
        Wt, Wr = self.tgt.world(nlt, nlr)
        P, rad, cid, names = C.body_points(self.tgt, Wt)
        Pv = self.ctx.to_veh(P)                          # (T,P,3)
        st = _starts(cid)
        T = len(Pv)
        pen = {n: np.full(T, -0.15) for n in want}
        nrm = {n: np.zeros((T, 3)) for n in want}
        frac = {n: np.zeros(T) for n in want}
        for t in range(T):
            r = capsule_pen(self.strip, Pv[t], rad, st, names, want)
            for n, (p, nv, fr) in r.items():
                pen[n][t], nrm[n][t], frac[n][t] = p, nv, fr
        self.last = (Pv, rad, st, names)
        return pen, nrm, frac, Wt, Wr

    def need(self, cname, t, gate, d):
        Pv, rad, st, names = self.last
        ci = names.index(cname)
        s0 = st[ci]
        e0 = st[ci + 1] if ci + 1 < len(st) else Pv.shape[1]
        return push_need(self.strip, Pv[t, s0:e0], float(rad[s0]), gate, d)


def _gate(ev_ours, ev_van_pen, want, slack_cm):
    """Per capsule gate in m under the check's gate: max(1 cm, seated + 0.5 cm, vanilla worst + 0.5 cm) - slack."""
    g = {}
    for n in want:
        seated = max(float(ev_ours[n][-1]), 0.0)
        ref = max(float(np.max(ev_van_pen[n])), 0.0)
        g[n] = max(0.01, seated + 0.005, ref + 0.005) - slack_cm / 100.0
    return g


def _smooth_vec(x, sigma):
    return IK.smooth_rows(x, sigma) if sigma > 0 else x


def _ramp_mask(T, zf, zl):
    m = np.ones(T)
    m[:zf] = 0.0
    if zl > 0:
        m[T - zl:] = 0.0
    # soft edges (3 frames)
    k = np.array([0.25, 0.5, 0.25])
    return np.clip(np.convolve(np.pad(m, 1, mode='edge'), k, mode='valid'), 0, 1)


def pav(y, w=None):
    """Weighted isotonic (non-decreasing) least-squares regression, pool-adjacent-violators."""
    vals, wts, cnt = [], [], []
    for i, v in enumerate(y):
        vals.append(float(v)); wts.append(1.0 if w is None else float(w[i])); cnt.append(1)
        while len(vals) > 1 and vals[-2] > vals[-1]:
            ww = wts[-2] + wts[-1]
            vals[-2] = (vals[-2] * wts[-2] + vals[-1] * wts[-1]) / ww
            wts[-2], cnt[-2] = ww, cnt[-2] + cnt[-1]
            vals.pop(); wts.pop(); cnt.pop()
    out = []
    for v, c in zip(vals, cnt):
        out += [v] * c
    return np.array(out)


def replant_shift(fw, dw, thr=0.4, min_run=3):
    """Per-frame shift (T,3) of a foot goal for a RE-PLANTED foot: every planted run (follow weight < thr) is moved by the hips shift at the middle of the run (constant: no slide on the
    floor), swings blend smoothly (smoothstep) from the shift of the previous run to the shift of the next; 0 at the first and the last frame."""
    T = len(fw)
    plant = fw < thr
    runs, t = [], 0
    while t < T:
        if plant[t]:
            u = t
            while u + 1 < T and plant[u + 1]:
                u += 1
            if u - t + 1 >= min_run:
                runs.append((t, u))
            t = u + 1
        else:
            t += 1
    out = np.zeros((T, 3))
    if not runs:
        return dw * fw[:, None]
    sv = [dw[a:b + 1].mean(axis=0) for a, b in runs]

    def ss(x):
        x = np.clip(x, 0.0, 1.0)
        return x * x * (3.0 - 2.0 * x)
    for (a, b), v in zip(runs, sv):
        out[a:b + 1] = v
    for (a0, b0), v0, (a1, b1), v1 in zip(runs[:-1], sv[:-1], runs[1:], sv[1:]):
        for t in range(b0 + 1, a1):
            out[t] = v0 + (v1 - v0) * ss((t - b0) / float(a1 - b0))
    a0 = runs[0][0]
    for t in range(0, a0):
        out[t] = sv[0] * ss(t / float(max(1, a0)))
    bl = runs[-1][1]
    for t in range(bl + 1, T):
        out[t] = sv[-1] * (1.0 - ss((t - bl) / float(max(1, T - 1 - bl))))
    return out


def limit_steps(tgt, nlr, bone_names, lim_deg, i0, i1, iters=60):
    """Smooth the local rotations of the named bones inside frames i0..i1 until no frame step exceeds lim_deg (binomial smoothing of the two frames around an offending step; frames outside
    the window are not touched). Returns the number of smoothing passes used and the largest remaining step."""
    idx = [tgt.index[b] for b in bone_names if b in tgt.index]
    T = len(nlr)
    worst = 0.0
    n_pass = 0
    for it in range(iters):
        worst = 0.0
        changed = False
        for j in idx:
            q = nlr[:, j]
            d = np.abs(np.sum(q[:-1] * q[1:], axis=1))
            ang = np.degrees(2.0 * np.arccos(np.clip(d, 0.0, 1.0)))
            for t in range(max(i0, 1), min(i1, T - 3)):
                if ang[t] > lim_deg:
                    for u in (t, t + 1):
                        a = q[u - 1] * np.sign(np.dot(q[u - 1], q[u]) or 1.0)
                        b = q[u + 1] * np.sign(np.dot(q[u + 1], q[u]) or 1.0)
                        v = 0.25 * a + 0.5 * q[u] + 0.25 * b
                        q[u] = v / np.linalg.norm(v)
                    changed = True
            worst = max(worst, float(ang[max(i0, 0):min(i1 + 1, T - 1)].max()))
        if not changed:
            break
        n_pass += 1
    return n_pass, worst


def hips_cap(args, tgt, nlt, nlr, ri, ctx, rep):
    ix = tgt.index
    Wt, Wr = tgt.world(nlt, nlr)
    hv = ctx.to_veh(Wt[:, ix['Hips']])
    y, z = hv[:, 1], hv[:, 2]
    T = len(y)
    i0 = int(args.lp_hips_from) if args.lp_hips_from >= 0 else int(np.argmax(z))
    dy = np.zeros(T)
    if args.lp_hips_mode == 'iso':
        i1 = int(args.lp_hips_start)
        wt = np.ones(T - i1)
        wt[-max(1, int(args.lp_hips_pin)):] = 1000.0   # the last frames (seated pose) keep their y: everything before is pooled below their running minimum
        yn = y.copy()
        yn[i1:] = y[i1:] + (pav(y[i1:], wt) - y[i1:]) * float(args.lp_hips_cap)
        yn = IK.smooth(yn, float(args.lp_hips_sigma))   # a smoothed monotone path stays monotone
        dy = yn - y
        dy[:i1] = 0.0
        ramp = np.clip(np.arange(T) / max(1.0, float(i1) + 3.0), 0.0, 1.0)
        dy = dy * np.where(np.arange(T) < i1 + 3, ramp, 1.0)
        tu = np.clip((T - 1 - np.arange(T)) / float(max(1, int(args.lp_hips_tail))), 0.0, 1.0)   # smoothstep fade to exactly 0 at the last frame (check (f): last steps of the helper bones < 1 deg)
        dy = dy * (tu * tu * (3.0 - 2.0 * tu))
    else:
        cap = y.copy()
        for t in range(T - 2, i0 - 1, -1):
            cap[t] = min(y[t], cap[t + 1])
        dy[i0:] = np.minimum(cap[i0:] - y[i0:], 0.0) * float(args.lp_hips_cap)
        dy = IK.smooth(dy, float(args.lp_hips_sigma))
        dy = np.minimum(dy, 0.0)
    if args.lp_fwd_bump:   # extra forward shift of the Hips (vehicle y, cm): a short bump that clears a capsule which passes the aperture edge at one moment
        dy = dy + HD.sink_profile(args.lp_fwd_bump, T)
    dy[-1] = 0.0
    if float(np.abs(dy).max()) < 1e-5:
        rep['hips_cap'] = dict(max_shift_m=0.0)
        return
    dv = np.zeros((T, 3))
    dv[:, 1] = dy
    dw = ctx.dir_to_world(dv)                         # clip world displacement
    hp = tgt.parent[ix['Hips']]
    Rp = Wr[:, hp] if hp >= 0 else np.broadcast_to(np.eye(3), (T, 3, 3))
    nlt[:, ix['Hips']] = nlt[:, ix['Hips']] + np.einsum('tji,tj->ti', Rp, dw)
    act = np.abs(dy) > 1e-5
    wik = np.where(act, np.clip(np.abs(dy) / 0.004, 0.0, 1.0), 0.0)   # IK weight fades with the correction: the last frames keep the original leg bend (check (f))
    errs = {}
    for k in ('LeftFoot', 'RightFoot') + (('LeftHand', 'RightHand') if args.lp_hips_hands == 'lock' else ()):
        a_, b_ = IK.LIMBS[k]
        goal = Wt[:, ix[k]].copy()
        if k.endswith('Foot') and args.lp_hips_feet == 'replant':   # every planted run moves with the body (no slide on the floor), the swings blend between the runs
            goal = goal + replant_shift(IK.follow_weight(goal), dw)
        elif k.endswith('Foot') and args.lp_hips_feet == 'follow':   # a swinging foot follows the shifted body, a planted foot holds its world position
            fw = IK.follow_weight(goal)
            if float(args.lp_feet_sigma) > 0:
                fw = np.clip(IK.smooth(fw, float(args.lp_feet_sigma)), 0.0, 1.0)
            goal = goal + dw * fw[:, None]
        errs[k] = IK.two_bone(tgt, nlt, nlr, ix[a_], ix[b_], ix[k], goal, weight=np.maximum(wik, 1e-9), max_extend=0.999, bend_sigma=2)
    nlr[:] = S.qcontinuous(nlr)
    lim_rep = None
    if float(args.lp_leg_step_limit) > 0:
        i_a = int(np.argmax(act))
        i_b = len(act) - 1 - int(np.argmax(act[::-1]))
        lim_rep = limit_steps(tgt, nlr, [n + b for n in ('Left', 'Right') for b in ('UpLeg', 'Leg')], float(args.lp_leg_step_limit), max(0, i_a - 3), min(len(act) - 1, i_b + 3))
    rep['hips_cap'] = dict(leg_step_limit=lim_rep, from_frame=i0, max_back_m=float(-dy.min()), max_fwd_m=float(dy.max()), frames=int(act.sum()), ik_err_m=errs)


def apply(args, tgt, nlt, nlr, ri, s_times, rep):
    ix = tgt.index
    T = len(s_times)
    lp = {}
    rep['limb_path'] = lp
    ctx = HD.Ctx(args.lp_json, args.lp_seat, nlt[-1, ri].copy(), S.q2m(nlr[-1, ri])) if args.lp_json else None
    if ctx is None:
        raise SystemExit('--limb-path needs --lp-json')
    if args.lp_sink_profile:
        # Hips drop (vehicle-vertical = clip world Y, feet held by two-bone IK) per clip frame, cosine easing between the knots (head_duck.sink_profile): the visible head stays
        # lower while the opening door sweeps past the start spot (approach gate, 'hold' class)
        sp = HD.sink_profile(args.lp_sink_profile, T)
        Wt0_, _ = tgt.world(nlt, nlr)
        nlt[:, ix['Hips'], 1] -= sp
        actp = sp > 1e-4
        errp = {}
        for k in ('LeftFoot', 'RightFoot'):
            a_, b_ = IK.LIMBS[k]
            errp[k] = IK.two_bone(tgt, nlt, nlr, ix[a_], ix[b_], ix[k], Wt0_[:, ix[k]], weight=np.where(actp, 1.0, 1e-9), max_extend=0.999, bend_sigma=2)
        nlr[:] = S.qcontinuous(nlr)
        lp['sink_profile'] = dict(max_cm=float(sp.max() * 100), frames=int(actp.sum()), foot_err_m=errp)
    if float(args.lp_hips_cap) > 0:
        hips_cap(args, tgt, nlt, nlr, ri, ctx, lp)
    limbs = [x for x in str(args.lp_limbs).split(',') if x]
    if not limbs:
        return
    import json as _j
    cls = _j.load(open(args.lp_json, encoding='utf-8'))['class']
    strip = Strip(cls, 'left' if args.lp_seat == 'left' else 'right')
    ev = Eval(tgt, ctx, strip)
    want = sorted({c for l in limbs for c in LIMBS[l]['end'] + LIMBS[l]['mid']})
    # gate: our seated pose + the game's own clip (the vanilla player clip of the row = --target)
    import check_door_clearance as C
    vsk, vt, vWt, vWr = C.load_clip(args.target, 'enter')
    vctx = HD.Ctx(args.lp_json, args.lp_seat, vWt[-1, vsk.index['Root']].copy(), vWr[-1, vsk.index['Root']])
    vP, vrad, vcid, vnames = C.body_points(vsk, vWt)
    vPv = vctx.to_veh(vP)
    vst = _starts(vcid)
    vpen = {n: np.full(len(vPv), -0.15) for n in want}
    for t in range(len(vPv)):
        for n, (p, _nv, _f) in capsule_pen(strip, vPv[t], vrad, vst, vnames, want).items():
            vpen[n][t] = p
    pen0, nrm0, frac0, _, _ = ev.pens(nlt, nlr, want)
    gate = _gate(pen0, vpen, want, args.lp_slack_cm)
    lp['gate_cm'] = {n: round(gate[n] * 100, 2) for n in want}
    lp['pen_before_cm'] = {n: round(float(pen0[n].max()) * 100, 2) for n in want}
    base_t, base_r = nlt.copy(), nlr.copy()
    Wt0, Wr0 = tgt.world(base_t, base_r)
    msk = _ramp_mask(T, int(args.lp_zero_first), int(args.lp_zero_last))
    dE = {l: np.zeros((T, 3)) for l in limbs}   # accumulated displacement of the end goal (vehicle frame)
    dM = {l: np.zeros((T, 3)) for l in limbs}   # accumulated displacement of the middle joint
    maxd = float(args.lp_max_cm) / 100.0
    dirv = None
    if args.lp_dir:
        dirv = np.array([float(x) for x in args.lp_dir.split(',')])
        dirv = dirv / np.linalg.norm(dirv)
    hist = []
    cur_t, cur_r = base_t, base_r
    for it in range(int(args.lp_iters)):
        pen, nrm, frac, _, _ = ev.pens(cur_t, cur_r, want)
        worst = {n: round(float((pen[n] - gate[n]).max()) * 100, 2) for n in want}
        hist.append(worst)
        if max(worst.values()) <= 0.0:
            break
        for l in limbs:
            for cname, kind in [(c, 'E') for c in LIMBS[l]['end']] + [(c, 'M') for c in LIMBS[l]['mid']]:
                ex = np.maximum(pen[cname] - gate[cname], 0.0) * float(args.lp_damp)
                if dirv is not None:   # fixed escape direction: the displacement along it that clears the gate (scan), per frame
                    need_ = np.array([ev.need(cname, t, gate[cname], dirv) if pen[cname][t] > gate[cname] else 0.0 for t in range(T)]) * float(args.lp_damp)
                    nrm = dict(nrm)
                    nrm[cname] = np.repeat(dirv[None], T, 0)
                    ex = need_
                # displacement of the capsule needed = ex along the escape direction; the end goal moves the capsule fully (end capsules), the middle joint 2x for the first segment
                if kind == 'E':
                    dE[l] += nrm[cname] * ex[:, None]
                    if cname.startswith('farm') or cname.startswith('shin'):
                        dM[l] += nrm[cname] * ex[:, None]
                else:
                    if cname.startswith('uarm') or cname.startswith('thigh'):
                        dM[l] += nrm[cname] * (2.0 * ex[:, None])
        # smooth + clip + mask, then re-solve from the untouched pose
        cur_t, cur_r = base_t.copy(), base_r.copy()
        for l in limbs:
            dE[l] = _smooth_vec(dE[l], float(args.lp_sigma)) * msk[:, None]
            dM[l] = _smooth_vec(dM[l], float(args.lp_sigma)) * msk[:, None]
            for d_ in (dE[l], dM[l]):
                nn = np.linalg.norm(d_, axis=1)
                f = np.minimum(1.0, maxd / np.maximum(nn, 1e-9))
                d_ *= f[:, None]
            a_, b_, c_ = [ix[n] for n in LIMBS[l]['chain']]
            actE = np.linalg.norm(dE[l], axis=1) > 1e-4
            actM = np.linalg.norm(dM[l], axis=1) > 1e-4
            act = actE | actM
            if not act.any():
                continue
            goal = Wt0[:, c_] + ctx.dir_to_world(dE[l])
            pole = Wt0[:, b_] + ctx.dir_to_world(dM[l])
            wgt = np.where(act, 1.0, 1e-9)
            IK.two_bone(tgt, cur_t, cur_r, a_, b_, c_, goal, weight=wgt, max_extend=0.999, bend_sigma=2, pole=pole, pole_weight=np.where(actM, 1.0, 0.0))
        cur_r = S.qcontinuous(cur_r)
    pen, nrm, frac, _, _ = ev.pens(cur_t, cur_r, want)
    lp['iters'] = len(hist)
    lp['over_gate_cm_by_iter'] = hist
    lp['pen_after_cm'] = {n: round(float(pen[n].max()) * 100, 2) for n in want}
    lp['max_goal_shift_cm'] = {l: round(float(np.linalg.norm(dE[l], axis=1).max()) * 100, 2) for l in limbs}
    lp['max_mid_shift_cm'] = {l: round(float(np.linalg.norm(dM[l], axis=1).max()) * 100, 2) for l in limbs}
    nlt[:], nlr[:] = cur_t, cur_r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--target', required=True, help='the vanilla player clip glb of the row (the game clip: jamb gate reference)')
    ap.add_argument('--report')
    add_args(ap)
    a = ap.parse_args()
    import build_entry_clip as B
    tgt = S.Skeleton.from_glb(a.inp)
    clip = tgt.clip('enter')
    dur, times = B.frames_of(tgt, clip)
    nlt, nlr = tgt.sample_local(clip, times)
    nlr = S.qcontinuous(nlr)
    ri = tgt.index['Root']
    rep = {}
    apply(a, tgt, nlt, nlr, ri, times, rep)
    extras = tgt.gj['animations'][clip].get('extras', {})
    B.write_out(tgt, a.out, 'enter', times, nlt, nlr, extras)
    print(json.dumps(rep, indent=1, default=float))
    if a.report:
        json.dump(rep, open(a.report, 'w'), indent=1, default=float)


if __name__ == '__main__':
    main()
