"""Neck / upper-spine DUCK layer for the car entry clips (roofpack round 2): a person getting into a low car tips the head under the roof line (neck + upper spine flexion, sideways tilt) instead of
sinking the whole body. The Hips-drop cap of build_entry_clip.py (--roof-json) moves the pelvis / chest / thighs and fails the body, door and approach gates; this layer moves only the joints
ABOVE the chest (Neck, Neck1, optionally Spine3 / Spine2 ...) and leaves the pelvis, legs and (for neck-only chains) the arms untouched.

Applied to the FINISHED pose (after --roof-final, before --limb-ik-final), so nothing downstream moves the head again:
  * the head sphere (centre 0.09 m above the Head joint, r 0.15) is tested against the roof cloud of the class (check_door_clearance: shell + glass + lining higher than the seat + 0.45 m) in the
    vehicle frame; gate = max(1 cm, seated penetration + 0.5 cm) minus --duck-slack-cm (seated = the last frame, which this layer never changes);
  * the bend is a horizontal vector b(t) (vehicle frame, |b| = total bend angle, direction = where the head moves; candidate azimuths --duck-az: 0 = forward, positive = toward the car
    centre line, negative = toward the door, 180 = back); the rotation about up x b is split over the chain --duck-chain (bone:weight, parent -> child) as world-space increments;
  * a dynamic programme over (bend, azimuth) per frame keeps the bend inside the set that clears the roof, within the anatomical limits of check (n) (neck flexion, trunk lean) and
    --duck-max-deg, at most --duck-vmax deg/frame, 0 at the first frame and at the last --duck-fade s, minimising the bend and its changes; the result is smoothed and verified (repair loop);
  * hands are re-locked to their pre-bend world positions by two-bone IK when the chain contains a clavicle-carrying spine bone (Spine3 and below).
Report: duck_rep['head_duck'].
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import ik as IK  # noqa: E402

GLTF_TO_VEHICLE = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
HEAD_UP_M = 0.09
HEAD_R_M = 0.15
TRACK = ['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head']
LIM_NECK, LIM_LEAN = 55.0, 75.0   # anatomy.LIMITS (check n)


def _minjerk(x):
    x = np.clip(x, 0, 1)
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def parse_chain(s):
    out = []
    for part in s.split(','):
        n, w = part.split(':')
        out.append((n.strip(), float(w)))
    for n, _ in out:
        assert n in TRACK[:-1], 'duck chain bone %s not in %s' % (n, TRACK[:-1])
    order = [TRACK.index(n) for n, _ in out]
    assert order == sorted(order), 'duck chain must be parent -> child'
    return out


class Ctx:
    """Roof model in the clip world: head Joint position (n,3) -> penetration (m)."""

    def __init__(self, roof_json, roof_seat, Wt_last_root, Rr_last_root):
        j = json.load(open(roof_json, encoding='utf-8'))
        dd = json.load(open(str(Path(roof_json).parent.parent / 'door-data' / (j['class'] + '.json')), encoding='utf-8'))
        sR = np.array(dd['seat'][roof_seat]['R']).reshape(3, 3)
        self.st = np.array(dd['seat'][roof_seat]['t'])
        seat_z = min(v[2] for v in j['seat'].values())
        P = np.concatenate([np.asarray(j['parts'][k]['points'], float).reshape(-1, 3) for k in ('shell', 'glass', 'lining')])
        self.P = P[P[:, 2] >= seat_z + 0.45].astype(np.float32)
        self.pe = Wt_last_root
        self.A = sR @ Rr_last_root.T          # clip world -> rig space
        self.G = GLTF_TO_VEHICLE
        self.xin = 1.0 if roof_seat == 'left' else -1.0   # vehicle x toward the car centre line
        self.up_off = (self.A @ np.array([0.0, HEAD_UP_M, 0.0])) @ self.G.T   # head sphere centre above the Head joint: clip world UP in the vehicle frame (the seat may be tilted)
        self.UP = np.array([0.0, 1.0, 0.0])

    def bend_axis_world(self, dv):
        """vehicle-frame horizontal direction(s) (n,2: x, y) the head moves toward -> rotation axis (n,3) in the clip world (up x horizontal direction)."""
        d = self.dir_to_world(np.concatenate([dv, np.zeros((len(dv), 1))], axis=1))
        d = d - self.UP * (d @ self.UP)[:, None]
        d = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
        return np.cross(self.UP[None], d)

    def to_veh(self, Pw):
        return (np.einsum('ij,...j->...i', self.A, Pw - self.pe) + self.st) @ self.G.T

    def dir_to_world(self, dv):
        return np.einsum('ij,...j->...i', self.A.T, dv @ self.G)   # vehicle direction -> clip world (A^T G^T dv)

    def pen(self, hv_joint, sub):
        """hv_joint (n,3) Head joint in the vehicle frame, sub = roof points near; -> penetration (m)."""
        c = (hv_joint + self.up_off).astype(np.float32)
        d2 = ((c[:, None, :] - sub[None]) ** 2).sum(-1)
        return HEAD_R_M - np.sqrt(d2.min(1))


def _rot(axis, ang):
    """axis (n,3) unit, ang (n,) -> (n,3,3)."""
    c, s = np.cos(ang), np.sin(ang)
    x, y, z = axis[..., 0], axis[..., 1], axis[..., 2]
    C = 1 - c
    R = np.empty(axis.shape[:-1] + (3, 3))
    R[..., 0, 0] = c + x * x * C
    R[..., 0, 1] = x * y * C - z * s
    R[..., 0, 2] = x * z * C + y * s
    R[..., 1, 0] = y * x * C + z * s
    R[..., 1, 1] = c + y * y * C
    R[..., 1, 2] = y * z * C - x * s
    R[..., 2, 0] = z * x * C - y * s
    R[..., 2, 1] = z * y * C + x * s
    R[..., 2, 2] = c + z * z * C
    return R


def fk_points(base, wts, axis_w, ang, sink=None):
    """base (7,3) tracked joint positions of one frame (TRACK order), wts (6,) per bone fraction, axis_w (n,3) world axes, ang (n,) total bend (rad) -> (n,7,3)."""
    n = len(ang)
    pts = np.repeat(base[None], n, 0)
    for k in range(len(TRACK) - 1):
        if wts[k] == 0.0:
            continue
        R = _rot(axis_w, ang * wts[k])
        rel = pts[:, k + 1:] - pts[:, k:k + 1]
        pts[:, k + 1:] = pts[:, k:k + 1] + np.einsum('nij,nmj->nmi', R, rel)
    if sink is not None:
        pts[:, :, 1] -= sink[:, None]   # the whole upper body sinks (Hips drop, feet held): clip world Y is up
    return pts


def sink_profile(spec, T):
    """'frame:cm,frame:cm,...' -> (T,) m of Hips drop per frame (cosine easing between the knots, 0 outside the first / last knot); empty -> zeros."""
    out = np.zeros(T)
    if not spec:
        return out
    kn = sorted((float(a), float(b) / 100.0) for a, b in (p.split(':') for p in str(spec).split(',')))
    fr = np.arange(T, dtype=float)
    xs = np.array([k[0] for k in kn])
    ys = np.array([k[1] for k in kn])
    for t in range(T):
        if t < xs[0] or t > xs[-1]:
            continue
        if t == xs[-1]:
            out[t] = ys[-1]
            continue
        j = int(np.searchsorted(xs, t, side='right')) - 1
        u = (t - xs[j]) / max(xs[j + 1] - xs[j], 1e-9)
        out[t] = ys[j] + (ys[j + 1] - ys[j]) * (0.5 - 0.5 * np.cos(np.pi * u))
    return out


def _angle(a, b):
    a = a / np.maximum(np.linalg.norm(a, axis=-1, keepdims=True), 1e-9)
    b = b / np.maximum(np.linalg.norm(b, axis=-1, keepdims=True), 1e-9)
    return np.degrees(np.arccos(np.clip((a * b).sum(-1), -1, 1)))


def apply(args, tgt, nlt, nlr, ri, s_times, duck_rep):
    ix = tgt.index
    chain = parse_chain(args.duck_chain)
    wts = np.zeros(len(TRACK) - 1)
    for n, w in chain:
        wts[TRACK.index(n)] = w
    T = len(s_times)
    Wt, Wr = tgt.world(nlt, nlr)
    first = getattr(args, 'duck_anchor', 'last') == 'first'   # exit clips: the seated pose is the FIRST frame (vehicle frame + gate anchored there); the last frame is still never changed
    ai = 0 if first else T - 1
    ctx = Ctx(args.duck_roof_json, args.duck_roof_seat, nlt[ai, ri].copy(), S.q2m(nlr[ai, ri]))
    tix = [ix[n] for n in TRACK]
    base = Wt[:, tix]                                   # (T,7,3)
    hips = Wt[:, ix['Hips']]
    hv_all = ctx.to_veh(base[:, -1])                   # head joint, vehicle frame
    # gate: seated penetration (last frame, never changed; --duck-anchor first: the first frame) + 0.5 cm, minus slack
    sub_s = ctx.P[np.linalg.norm(ctx.P - (hv_all[ai] + ctx.up_off).astype(np.float32), axis=1) < 0.5]
    seated = max(0.0, float(ctx.pen(hv_all[ai:ai + 1], sub_s)[0]))
    gate = max(0.01, seated + 0.005)
    target = gate - args.duck_slack_cm / 100.0
    azs = [float(a) for a in str(args.duck_az).split(',')]
    th_max = float(args.duck_max_deg)
    th_grid = np.radians(np.arange(0.0, th_max + 1e-9, args.duck_step_deg))
    K = len(th_grid)
    kap = float(args.duck_sink_cm_per_deg) / 100.0

    def sink_of(th):   # m of Hips drop per total bend (rad)
        return kap * np.degrees(th)
    sink_grid = sink_of(th_grid)
    # roofpack3: Hips sink profile in time (--duck-sink-profile 'frame:cm,frame:cm,...', cosine easing between the knots, 0 before the first and after the last knot): the pelvis
    # goes down EARLIER / DEEPER at the frames where the door and the body leave room for it, independent of the bend (added to the bend-proportional sink)
    sp = sink_profile(args.duck_sink_profile, T)
    assert abs(sp[-1]) < 1e-9, 'the sink profile must be 0 at the last frame'
    dirs_v = np.array([[ctx.xin * np.sin(np.radians(a)), np.cos(np.radians(a)), 0.0] for a in azs])
    na = len(azs)
    axes_w = ctx.bend_axis_world(dirs_v[:, :2])         # (na,3) rotation axis for a bend toward dirs_v, clip world
    lim_neck, lim_lean = LIM_NECK - args.duck_limit_margin_deg, LIM_LEAN - args.duck_limit_margin_deg
    fade_n = max(1, int(round(args.duck_fade * 30.0)))
    PN = np.full((T, K, na), -0.15)
    NF = np.zeros((T, K, na))
    LN = np.zeros((T, K, na))
    for t in range(T):
        hv = hv_all[t]
        cen = (hv + ctx.up_off).astype(np.float32)
        sub = ctx.P[np.linalg.norm(ctx.P - cen, axis=1) < 0.9]
        for a in range(na):
            ax = np.repeat(axes_w[a][None], K, 0)
            pts = fk_points(base[t], wts, ax, th_grid, sink_grid + sp[t])
            pv = ctx.to_veh(pts[:, -1])
            if len(sub):
                PN[t, :, a] = ctx.pen(pv, sub)
            NF[t, :, a] = _angle(pts[:, 4] - pts[:, 3], pts[:, 6] - pts[:, 4])
            LN[t, :, a] = _angle(pts[:, 4] - (hips[t][None] - np.array([[0.0, 1.0, 0.0]]) * (sink_grid + sp[t])[:, None]), np.array([[0.0, 1.0, 0.0]]))
    pin = np.zeros(T, bool)
    pin[0] = True
    pin[max(0, T - fade_n):] = True
    # the last frames before the (unchanged) seated pose: the total bend may only be cap(k) k frames before the last frame (check (f): the last step of every bone < 1 deg)
    capdeg = np.full(T, th_max)
    for k in range(1, T):
        capdeg[T - 1 - k] = min(th_max, args.duck_end_ramp_deg * k + 0.25 * k * (k - 1))
        if capdeg[T - 1 - k] >= th_max:
            break
    capdeg[T - 1] = 0.0
    if first:   # exit: the bend also leaves the unchanged first frame gently (check (f) mirror: first step of every bone < 1 deg)
        for k in range(1, T):
            capdeg[k] = min(capdeg[k], min(th_max, args.duck_end_ramp_deg * k + 0.25 * k * (k - 1)))
            if args.duck_end_ramp_deg * k + 0.25 * k * (k - 1) >= th_max:
                break
        capdeg[0] = 0.0
    vmax = int(round(np.radians(args.duck_vmax_deg) / max(th_grid[1] - th_grid[0], 1e-9)))
    dth = th_grid[1] - th_grid[0]

    def solve(tgt_t):
        viol = np.maximum(0, PN - tgt_t[:, None, None]) * 100 + np.maximum(0, NF - lim_neck) * 0.5 + np.maximum(0, LN - lim_lean) * 0.5
        for t in np.flatnonzero(pin):
            viol[t] = 1e4
            viol[t, 0, :] = 0.0
        viol += 1e3 * (np.degrees(th_grid)[None, :, None] > capdeg[:, None, None] + 1e-6)
        cost = np.zeros((T, K, na))
        back = np.zeros((T, K, na, 2), np.int32)
        for t in range(T):
            unit = (th_grid[:, None] / np.radians(10.0)) ** 2 * np.ones((1, na)) + np.where(viol[t] > 0, 30.0 + 40.0 * viol[t], 0.0)
            if t == 0:
                cost[0] = unit
                continue
            prev = cost[t - 1]
            best = np.full((K, na), np.inf)
            arg = np.zeros((K, na, 2), np.int32)
            for k in range(K):
                lo, hi = max(0, k - vmax), min(K, k + vmax + 1)
                dk = (np.arange(lo, hi) - k)[:, None] * np.ones((1, na))
                trk = (dk * dth / np.radians(2.0)) ** 2 * args.duck_move_cost
                for a in range(na):
                    cand = prev[lo:hi, :] + trk + (np.arange(na)[None, :] != a) * args.duck_az_cost
                    m = int(np.argmin(cand))
                    ii, jj = np.unravel_index(m, cand.shape)
                    best[k, a] = cand[ii, jj]
                    arg[k, a] = (lo + ii, jj)
            cost[t] = best + unit
            back[t] = arg
        k, a = np.unravel_index(int(np.argmin(cost[-1])), cost[-1].shape)
        path = [(int(k), int(a))]
        for t in range(T - 1, 0, -1):
            kk, aa = back[t][path[-1][0], path[-1][1]]
            path.append((int(kk), int(aa)))
        path = path[::-1]
        unreach = [t for t in range(T) if viol[t, path[t][0], path[t][1]] > 0 and not pin[t]]
        th = np.array([th_grid[q[0]] for q in path])
        az = np.array([q[1] for q in path])
        b = np.array([th[t] * np.array([dirs_v[az[t], 0], dirs_v[az[t], 1]]) for t in range(T)])
        return b, az, unreach

    UPV = np.array([0.0, 1.0, 0.0])

    def eval_frame(t, bv):
        m_ = float(np.linalg.norm(bv))
        d_ = bv / m_ if m_ > 1e-9 else np.array([0.0, 1.0])
        ax = ctx.bend_axis_world(d_[None])
        th_ = np.array([m_])
        sk_ = sink_of(th_) + sp[t]
        pts = fk_points(base[t], wts, ax, th_, sk_)
        pv = ctx.to_veh(pts[:, -1])[0]
        cen = (pv + ctx.up_off).astype(np.float32)
        sub = ctx.P[np.linalg.norm(ctx.P - cen, axis=1) < 0.9]
        pen = float(ctx.pen(pv[None], sub)[0]) if len(sub) else -0.15
        nf = float(_angle(pts[0, 4] - pts[0, 3], pts[0, 6] - pts[0, 4]))
        ln = float(_angle(pts[0, 4] - (hips[t] - UPV * sk_[0]), UPV))
        return pen, pv, nf, ln

    def eval_interval(h0, h1):
        # between two frames the roof phase sees the head joint moving linearly (120 Hz)
        out = -0.15
        for u in (0.25, 0.5, 0.75):
            hq = h0 * (1 - u) + h1 * u
            cen = (hq + ctx.up_off).astype(np.float32)
            sub = ctx.P[np.linalg.norm(ctx.P - cen, axis=1) < 0.9]
            if len(sub):
                out = max(out, float(ctx.pen(hq[None], sub)[0]))
        return out

    def evaluate(bs):
        fr = [eval_frame(t, bs[t]) for t in range(T)]
        iv = [eval_interval(fr[t][1], fr[t + 1][1]) for t in range(T - 1)] + [-0.15]
        return fr, iv

    def excess_of(fr, iv, t):
        """worst excess (m) over the gate and the anatomical limits that frame t is charged with (its own pose and both adjacent intervals)."""
        e = fr[t][0] - verify
        e = max(e, iv[t] - verify, (iv[t - 1] - verify) if t else -1.0)
        lim = max(0.0, fr[t][2] - lim_neck, fr[t][3] - lim_lean) * 0.01   # 1 deg over a limit = 1 cm of excess
        return max(e, lim)

    def polish(bs, fr, iv, iters):
        """Local search on the smoothed solution: the roof cloud is sparse (the penetration jumps by 0.3-0.5 cm between neighbouring head positions), so a frame / interval that still
        exceeds the gate gets small perturbations of the bend vector at itself and its neighbours (<= --duck-vmax-deg from the neighbours, <= max bend); the best one is kept."""
        log = []
        vm = np.radians(args.duck_vmax_deg)
        lo, hi = 1, T - fade_n - 1
        dl = [np.radians(x) for x in (0.5, 1.0, 2.0, 3.0, 5.0)]
        dirs8 = [np.array([np.cos(a_), np.sin(a_)]) for a_ in np.radians(np.arange(0, 360, 45))]
        dead = set()
        for it in range(iters):
            ex = np.array([excess_of(fr, iv, t) for t in range(T)])
            ex[:lo] = -1
            order = [t for t in np.argsort(-ex) if ex[t] > 0 and t not in dead]
            if not order:
                break
            w = int(order[0])
            best = None
            for u in range(max(lo, w - 1), min(hi, w + 1) + 1):
                cur_local = max(excess_of(fr, iv, k) for k in range(max(0, u - 1), min(T, u + 2)))
                for s_ in dl:
                    for d_ in dirs8:
                        nb = bs[u] + s_ * d_
                        nm = np.linalg.norm(nb)
                        if nm > np.radians(capdeg[u]) or np.linalg.norm(nb - bs[u - 1]) > vm or np.linalg.norm(nb - bs[u + 1]) > vm:
                            continue
                        f2 = eval_frame(u, nb)
                        h_prev = fr[u - 1][1]
                        h_next = fr[u + 1][1]
                        i_prev = eval_interval(h_prev, f2[1])
                        i_next = eval_interval(f2[1], h_next)
                        fr_s, iv_s = list(fr), list(iv)
                        fr_s[u] = f2
                        iv_s[u - 1] = i_prev
                        iv_s[u] = i_next
                        loc = max(excess_of(fr_s, iv_s, k) for k in range(max(0, u - 1), min(T, u + 2)))
                        key = (max(loc, 0.0), s_)
                        if best is None or key < best[0]:
                            best = (key, u, nb, f2, i_prev, i_next, loc, cur_local)
            if best is None or best[6] >= best[7] - 1e-6:
                dead.add(w)
                continue
            _, u, nb, f2, i_prev, i_next, loc, cur = best
            bs[u] = nb
            fr[u] = f2
            iv[u - 1] = i_prev
            iv[u] = i_next
            log.append((it, int(u), round(float(cur) * 100, 2), round(float(loc) * 100, 2)))
        return log

    sig = float(args.duck_smooth)
    # a frame whose UNbent pose already clears the gate is not asked for the slack (the roof model is a point cloud: near the seat the penetration is noisy to +-0.5 cm)
    tgt_t = np.where(PN[:, 0, 0] <= gate - 0.0002, np.maximum(target, PN[:, 0, 0]), target)
    if args.duck_slack_until > 0:
        # r13f: frames before this index are asked for the slack even when their unbent pose already clears the gate (the transition keeps a margin under the
        # roof, the seated tail after it keeps the rule above); default off
        tgt_t = np.where(np.arange(T) < args.duck_slack_until, target, tgt_t)
    verify = gate - float(args.duck_verify_cm) / 100.0   # roofpack3: --duck-verify-cm (default 0.03 cm = the round-2 value)
    rep_rounds = []
    b, az, unreachable = solve(tgt_t)
    bs = IK.smooth_rows(b, sig) if sig > 0 else b.copy()
    bs[0] = 0.0
    bs[max(0, T - fade_n):] = 0.0
    for t_ in range(T):   # the end ramp also holds for the smoothed bend
        nm_ = float(np.linalg.norm(bs[t_]))
        if nm_ > np.radians(capdeg[t_]) and nm_ > 0:
            bs[t_] *= np.radians(capdeg[t_]) / nm_
    fr, iv = evaluate(bs)
    ex0 = max(excess_of(fr, iv, t) for t in range(T))
    plog = polish(bs, fr, iv, args.duck_repair)
    ex1 = max(excess_of(fr, iv, t) for t in range(T))
    rep_rounds.append(dict(excess_before_polish_cm=float(ex0 * 100), excess_after_polish_cm=float(ex1 * 100), polish_steps=len(plog), unreachable=unreachable))
    mag = np.linalg.norm(bs, axis=1)
    pen_after = np.array([f[0] for f in fr])
    nf_after = np.array([f[2] for f in fr])
    ln_after = np.array([f[3] for f in fr])
    pen_int = np.array(iv)
    # apply to the pose: increments in world space, parent -> child
    mag = np.linalg.norm(bs, axis=1)
    dirn = bs / np.maximum(mag[:, None], 1e-9)
    axw = ctx.bend_axis_world(dirn)
    hand_goals = {k: Wt[:, ix[k]].copy() for k in ('LeftHand', 'RightHand')}
    elbow_goals = {k: Wt[:, ix[IK.LIMBS[k][1]]].copy() for k in ('LeftHand', 'RightHand')}   # pre-bend elbow world positions (--duck-relock-elbow)
    sinkv = sink_of(mag) + sp
    feet = {k: Wt[:, ix[k]].copy() for k in ('LeftFoot', 'RightFoot')}
    if kap > 0 or sp.any():
        nlt[:, ix['Hips'], 1] -= sinkv
        for k in ('LeftFoot', 'RightFoot'):
            a_, b_ = IK.LIMBS[k]
            kpole = {}
            if getattr(args, 'duck_leg_pole', False):   # exit clips: the knee keeps bending toward its pre-sink world position (the shin stays in its vanilla plane)
                kpole = dict(pole=Wt[:, ix[b_]].copy(), pole_weight=np.where(sinkv > 1e-4, 1.0, 1e-9))
            IK.two_bone(tgt, nlt, nlr, ix[a_], ix[b_], ix[k], feet[k], weight=np.where(sinkv > 1e-4, 1.0, 1e-9), max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma, **kpole)
    for n, w in chain:
        j = ix[n]
        Wt2, Wr2 = tgt.world(nlt, nlr)
        Rinc = _rot(axw, mag * w)
        IK.set_world_rot(tgt, nlt, nlr, j, Rinc @ Wr2[:, j])
    nlr[:] = S.qcontinuous(nlr)
    relock = (bool(args.duck_relock_hands) or bool(args.duck_relock_elbow)) and any(TRACK.index(n) <= TRACK.index('Spine3') for n, _ in chain)
    errs = {}
    if relock:
        wgt = np.where((mag > 1e-4) | (sinkv > 1e-4), 1.0, 1e-9)   # roofpack3: also while the Hips sink profile is on (the locked arm must not snap back while the body is still lower)
        for k in [x.strip() for x in str(args.duck_relock_limbs).split(',') if x.strip()]:
            a_, b_ = IK.LIMBS[k]
            errs[k] = IK.two_bone(tgt, nlt, nlr, ix[a_], ix[b_], ix[k], hand_goals[k], weight=wgt, max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma,
                                  pole=elbow_goals[k] if args.duck_relock_elbow else None, pole_weight=wgt if args.duck_relock_elbow else None)
        nlr[:] = S.qcontinuous(nlr)
    duck_rep['head_duck'] = dict(chain=chain, az=azs, target_cm=target * 100, gate_cm=gate * 100, seated_pen_cm=seated * 100, max_bend_deg=float(np.degrees(mag.max())), sink_cm_per_deg=kap * 100, sink_profile=str(args.duck_sink_profile), max_sink_cm=float((sink_of(mag) + sp).max() * 100),
                                 bend_deg=[round(float(np.degrees(x)), 2) for x in mag], az_idx=[int(x) for x in az], unreachable_frames=unreachable,
                                 repair=rep_rounds, pen_after_cm=[round(float(x) * 100, 2) for x in pen_after], pen_interval_max_cm=[round(float(x) * 100, 2) for x in pen_int], neck_flex_after=[round(float(x), 1) for x in nf_after],
                                 trunk_lean_after=[round(float(x), 1) for x in ln_after], hand_relock_err_m=errs)
    return mag


def add_args(p):
    p.add_argument('--head-duck', action='store_true', help='neck / upper-spine duck layer on the finished pose (head_duck.py); needs --duck-roof-json / --duck-roof-seat')
    p.add_argument('--duck-roof-json', help='body-data/<class>.json of the class')
    p.add_argument('--duck-anchor', choices=['last', 'first'], default='last', help="'last' (entry clips): the seated pose = the last frame (vehicle frame, seated penetration); 'first' (exit clips, build_exit_duck.py): the seated pose = the first frame. Both end frames stay unchanged")
    p.add_argument('--duck-leg-pole', action='store_true', help='with a Hips sink: the leg IK bends each knee toward its pre-sink world position (default off = entry rows unchanged)')
    p.add_argument('--duck-roof-seat', choices=['left', 'right'], default='left')
    p.add_argument('--duck-chain', default='Neck:0.6,Neck1:0.4', help='bone:weight list parent -> child; the total bend angle is multiplied by each weight')
    p.add_argument('--duck-az', default='0', help='candidate bend azimuths in deg (comma list): 0 = head moves forward, + toward the car centre line, - toward the door, 180 = back')
    p.add_argument('--duck-max-deg', type=float, default=30.0, help='largest total bend (deg)')
    p.add_argument('--duck-step-deg', type=float, default=0.5)
    p.add_argument('--duck-vmax-deg', type=float, default=2.5, help='deg/frame: largest change of the total bend per frame')
    p.add_argument('--duck-slack-cm', type=float, default=0.4, help='cm: the bend aims this far under the roof gate (max(1 cm, seated + 0.5 cm))')
    p.add_argument('--duck-slack-until', type=int, default=0, help='r13f: frames with an index below this are asked for --duck-slack-cm even when their unbent pose clears the gate; 0 = off')
    p.add_argument('--duck-fade', type=float, default=0.2, help='s: no bend in the last frames before the seated pose (the last frame is never changed)')
    p.add_argument('--duck-smooth', type=float, default=1.0, help='frames: Gaussian smoothing of the bend vector')
    p.add_argument('--duck-repair', type=int, default=150, help='iterations of the local polish of the smoothed bend (frames / intervals still over the gate)')
    p.add_argument('--duck-limit-margin-deg', type=float, default=3.0, help='deg kept under the anatomical limits of check (n)')
    p.add_argument('--duck-move-cost', type=float, default=1.0)
    p.add_argument('--duck-az-cost', type=float, default=3.0)
    p.add_argument('--duck-relock-hands', action='store_true', help='hold the hands at their pre-bend world positions by two-bone IK when the chain moves the shoulders (the hand then keeps its world rotation: check (f) last step)')
    p.add_argument('--duck-relock-elbow', action='store_true', help='roofpack3: with the hand relock, the elbow bend direction points at the pre-bend elbow world position (the elbow stays where it was while the shoulder moves; implies --duck-relock-hands)')
    p.add_argument('--duck-relock-limbs', default='LeftHand,RightHand', help="roofpack3: which hands the relock holds (comma list of LeftHand / RightHand); the door-side arm only for the Turbo-R driver rows")
    p.add_argument('--duck-verify-cm', type=float, default=0.03, help='roofpack3: cm the verified bend (and the polish) keeps under the roof gate; the round-2 rows used 0.03')
    p.add_argument('--duck-end-ramp-deg', type=float, default=1.5, help='deg: the total bend may be at most this x k (+ a quadratic term) k frames before the last frame (check (f): last step < 1 deg per bone)')
    p.add_argument('--duck-sink-cm-per-deg', type=float, default=0.0, help='cm of Hips drop (feet held by IK) per degree of total bend: ducking sinks the body a little')
    p.add_argument('--duck-sink-profile', default='', help="roofpack3: Hips drop (cm, feet held by IK) as a function of the clip frame, independent of the bend: 'frame:cm,frame:cm,...' with cosine easing between the knots, 0 outside; the last frame must stay 0")


def main():
    """Prototype on a finished clip: head_duck.py --clip in.anims.glb --out out.anims.glb --duck-roof-json ... (the same options as the builder)."""
    import argparse
    import copy
    import build_entry_clip as B
    p = argparse.ArgumentParser()
    p.add_argument('--clip', required=True)
    p.add_argument('--clip-name', default='enter')
    p.add_argument('--out', required=True)
    p.add_argument('--ik-max-extend', type=float, default=0.999)
    p.add_argument('--ik-bend-sigma', type=float, default=2.0)
    add_args(p)
    a = p.parse_args()
    tgt = S.Skeleton.from_glb(a.clip)
    c = tgt.clip(a.clip_name)
    dur, times = B.frames_of(tgt, c)
    lt, lr = tgt.sample_local(c, times)
    lr = S.qcontinuous(lr)
    ri = tgt.index['Root']
    rep = {}
    apply(a, tgt, lt, lr, ri, times, rep)
    hd = rep['head_duck']
    print('target %.2f cm gate %.2f seated %.2f; max bend %.1f deg; unreachable %s' % (hd['target_cm'], hd['gate_cm'], hd['seated_pen_cm'], hd['max_bend_deg'], hd['unreachable_frames']))
    print('bend   ', hd['bend_deg'])
    print('pen cm ', hd['pen_after_cm'])
    print('neck   ', hd['neck_flex_after'])
    print('lean   ', hd['trunk_lean_after'])
    print('repair ', hd['repair'])
    extras = copy.deepcopy(tgt.gj['animations'][c]['extras']) if 'extras' in tgt.gj['animations'][c] else {}
    B.write_out(tgt, a.out, a.clip_name, times, lt, lr, extras)
    return 0


if __name__ == '__main__':
    sys.exit(main())
