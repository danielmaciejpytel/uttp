"""Re-planted support foot for a MOTORCYCLE get-off clip (exit_l / exit_r of the player sets): the vanilla clips glide the planted support foot over the ground for ~1 m (the Root moves 1.14 m while the
skeleton keeps the foot at a nearly constant place relative to the hips), measured 2026-11-07 with the robust contact rule of bike_steps_gen (floors_of(robust=True), contact_phases).

  python -I bike_exit_replant.py --glb <vanilla p?a_sportbike glb> --clip exit_l --out <out glb> [--plan-only] [--report out.json] [--hop-pen 3.0] [--hop-frames 6] [--apex-cm 10]

What it does (offline, no game; hips / spine / arms / Root / events untouched = the vanilla values bit for bit):
  1. support foot = the side with the longest planted phase; the leg is almost straight all the time (hips ~0.8-0.9 m above the foot), so a pinned foot can only stay within the leg reach (--reach x thigh+shin)
     of the vanilla hips path: plan() chooses, by dynamic programming, how many plants (pins) + hops the support foot needs (a hop = lift, move, place over --hop-frames; hop penalty --hop-pen against the
     deviation of the pinned spot from the vanilla foot path) so that every plant is reachable and its spot is as close to the vanilla foot path as possible. The LAST plant is the exact vanilla final stance.
  2. every plant: ankle position AND world orientation constant (a foot that is planted does not slide, yaw or roll); hops: ankle position smoothstep, height arc (--apex-cm), orientation slerp;
     two-bone leg IK on the vanilla hips (ik.two_bone, the knee pole stays near the vanilla knee), toes follow the foot.
  3. the other foot: its final planted phase (touch-down .. last frame) is pinned to the vanilla final spot (the vanilla clip shuffles it 4-15 cm); the swing before it is blended there.
  4. limb IK helpers (feet, knee poles) re-anchored on the new legs (exit_steps.reanchor_helpers, same rule as the entry clips), vanilla bit for bit outside the edited frames and on the first / last frame.
The output GLB goes through build_bike_exit.ps1 (as <PickRoot>\\p?a_sb?_<l|r>\\pick.anims.glb): pole fix, WolvenKit import, motion extraction copied verbatim from the vanilla set (Root unchanged), roundtrip, check_bike_exit.py.
"""
import argparse
import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
VE = os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry')
sys.path.insert(0, HERE)
sys.path.insert(0, VE)
import skel as S  # noqa: E402
import ik as IK  # noqa: E402
import build_entry_clip as B  # noqa: E402
import exit_steps as ES  # noqa: E402
import bike_steps_gen as BSG  # noqa: E402


def red(p):
    """glTF (x, y up, z) -> RED (x right, y forward, z up)"""
    return np.stack([p[..., 0], -p[..., 2], p[..., 1]], -1)


def gltf(p):
    return np.stack([p[..., 0], p[..., 2], -p[..., 1]], -1)


def smooth01(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def slerp_mats(R0, R1, u):
    """rotation matrices (3,3) -> geodesic interpolation at u"""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    ks = Rot.from_matrix(np.stack([R0, R1]))
    return Slerp([0.0, 1.0], ks)([u]).as_matrix()[0]


class Rig:
    def __init__(self, glb, clip):
        self.tgt = S.Skeleton.from_glb(glb)
        self.c = self.tgt.clip(clip)
        self.dur, self.times = B.frames_of(self.tgt, self.c)
        self.lt0, self.lr0 = self.tgt.sample_local(self.c, self.times)
        self.lr0 = S.qcontinuous(self.lr0)
        self.n = len(self.times)
        self.Wt0, self.Wr0 = self.tgt.world(self.lt0, self.lr0)
        self.extras = copy.deepcopy(self.tgt.gj['animations'][self.c].get('extras', {}))

    def idx(self, name):
        return self.tgt.index[name]


def plan_support(U, V, Lleg, t0, n, reach, hop_min, hop_max, hop_pen, min_plant, vstep=0.11, last_tol=1.012, hop_len_pen=0.1, first_off=None, strict=True):
    """U (T,3) hip joint, V (T,3) vanilla ankle (RED). Returns (cost, plants [(a, b, ref)], hops [(b+1, a2-1)]) covering t0..n-1.
    A plant a..b is feasible when its pinned ankle spot (the vanilla ankle of frame ref, any ref a..b) is within the reach (reach x thigh + shin, or the vanilla's own hip-ankle distance of that frame) of the
    hip joint on every frame; the LAST plant (ends at n-1) pins the vanilla final stance (ref = n-1) with the reach last_tol x L (the vanilla final pose is a fully straight leg, the IK clamps the rest).
    Plant cost = sum over its frames of the horizontal distance to the vanilla ankle path (m); a hop of h frames (hop_min..hop_max) between two plants costs hop_pen + hop_len_pen * h and needs
    the horizontal step <= vstep * h (m / frame)."""
    inf = 1e9
    allowed = np.full(len(U), reach * Lleg) if strict else np.maximum(reach * Lleg, np.linalg.norm(U - V, axis=1))
    memo = {}

    def plant(a, b):
        if (a, b) in memo:
            return memo[(a, b)]
        last = (b == n - 1)
        best = (inf, -1)
        rr = range(a, b + 1)
        if first_off is not None and a == t0:
            rr = range(a, min(b, t0 + first_off) + 1)
        for r in ([n - 1] if last else rr):
            P = V[r]
            d = np.linalg.norm(U[a:b + 1] - P, axis=1)
            lim = np.full(b - a + 1, last_tol * Lleg) if last else allowed[a:b + 1]
            if (lim - d).min() < 0:
                continue
            cost = float(np.linalg.norm(V[a:b + 1, :2] - P[:2], axis=1).sum())
            if cost < best[0]:
                best = (cost, r)
        memo[(a, b)] = best
        return best

    nodes = {}
    order = []
    for a in range(t0, n - min_plant + 1):
        for b in range(a + min_plant - 1, n):
            c, r = plant(a, b)
            if c < inf:
                nodes[(a, b)] = (c, r)
                order.append((a, b))
    order.sort()
    best = {}
    prev = {}
    for (a, b) in order:
        if a == t0:
            best[(a, b)] = nodes[(a, b)][0]
    for (a, b) in order:
        if (a, b) not in best:
            # a > t0: reachable from any predecessor (a0, b0) with a0 <= b0 and a = b0 + h + 1
            pass
        # relax outgoing edges
        if (a, b) not in best or b >= n - 1:
            continue
        r = nodes[(a, b)][1]
        for h in range(hop_min, hop_max + 1):
            a2 = b + h + 1
            for b2 in range(a2 + min_plant - 1, n):
                if (a2, b2) not in nodes:
                    continue
                c2, r2 = nodes[(a2, b2)]
                if np.linalg.norm(V[r2, :2] - V[r, :2]) > vstep * h:
                    continue
                tot = best[(a, b)] + hop_pen + hop_len_pen * h + c2
                if tot < best.get((a2, b2), inf):
                    best[(a2, b2)] = tot
                    prev[(a2, b2)] = (a, b)
    ends = [(best[k], k) for k in best if k[1] == n - 1]
    if not ends:
        return inf, [], []
    tot, k = min(ends)
    seq = [k]
    while k in prev:
        k = prev[k]
        seq.append(k)
    seq.reverse()
    plants = [(a, b, nodes[(a, b)][1]) for a, b in seq]
    hops = [(seq[i][1] + 1, seq[i + 1][0] - 1) for i in range(len(seq) - 1)]
    return tot, plants, hops


def slack_report(U, V, Lleg, plants):
    out = []
    for a, b, r in plants:
        d = np.linalg.norm(U[a:b + 1] - V[r], axis=1)
        out.append(dict(a=a, b=b, ref=r, slack_cm=round(float((np.maximum(0.985 * Lleg, np.linalg.norm(U - V, axis=1))[a:b + 1] - d).min() * 100), 1), min_dist_pct=round(float(d.min() / Lleg * 100), 1), max_dist_pct=round(float(d.max() / Lleg * 100), 1)))
    return out


def restore_arm_helpers(tgt, lt, lr, rig):
    """the arms are not edited: the arm IK helpers / elbow poles (and their descendants) that exit_steps.reanchor_helpers re-placed go back to the vanilla locals"""
    arm = [i for i, nm in enumerate(tgt.names) if ('Arm_' in nm and 'Torso' in nm) or nm.startswith('arm_')]
    todo = set(arm)
    for i in range(len(tgt.names)):
        if tgt.parent[i] in todo:
            todo.add(i)
    for j in todo:
        lt[:, j], lr[:, j] = rig.lt0[:, j], rig.lr0[:, j]


def place_leg_poles(tgt, lt, lr, rig, sides, f0, f1):
    """knee pole helpers of the re-planted legs: the vanilla offset from the knee, swung with the minimal rotation that takes the vanilla hip->ankle axis to the new one. reanchor_helpers re-places
    the pole in the limb plane frame, which is undefined for a (nearly) straight leg and flipped the pole of the straight final stance by up to 118 deg (a knee pop of the game's leg IK)."""
    if not sides:
        return
    Wt, Wr = tgt.world(lt, lr)
    vWt = rig.Wt0
    for sd in sides:
        a, b, c = (tgt.index[sd + x] for x in ('UpLeg', 'Leg', 'Foot'))
        uv = vWt[:, c] - vWt[:, a]
        un = Wt[:, c] - Wt[:, a]
        Rm = IK.rot_between(uv, un)
        for pn in ('Torso_%sLeg_PoleVector_JNT' % sd, 'Torso_World_%sLeg_PoleVector_JNT' % sd):
            if pn not in tgt.index:
                continue
            pb = tgt.index[pn]
            off = vWt[:, pb] - vWt[:, b]
            newp = Wt[:, b] + np.einsum('tij,tj->ti', Rm, off)
            # local translation of the pole under its (unchanged) parent; the pole keeps its vanilla world rotation
            par = tgt.parent[pb]
            inv = np.transpose(Wr[:, par], (0, 2, 1))
            lt[f0:f1 + 1, pb] = np.einsum('tij,tj->ti', inv, newp - Wt[:, par])[f0:f1 + 1]
            lr[f0:f1 + 1, pb] = S.m2q(inv @ rig.Wr0[:, pb])[f0:f1 + 1]


def pin_phase(Tp, Tr, P, R, f0, f1, ref, n, ramp=3):
    """pin the ankle xy + world rotation of frames f0..f1 to the vanilla spot of frame ref (height stays the vanilla one: the foot is landing / settling); the ramp frames before f0 blend the vanilla xy /
    rotation into the pin, after f1 (a phase that does not reach the last frame) back out. The last frame is never edited."""
    for f in range(f0, min(f1, n - 2) + 1):
        Tp[f, :2] = P[ref, :2]
        Tr[f] = R[ref]
    for f in range(max(1, f0 - ramp), f0):
        w = smooth01((f - (f0 - ramp) + 1) / float(ramp + 1))
        Tp[f, :2] = (1 - w) * P[f, :2] + w * P[ref, :2]
        Tr[f] = slerp_mats(R[f], R[ref], w)
    if f1 < n - 1:
        for f in range(f1 + 1, min(n - 1, f1 + ramp + 1)):
            w = 1.0 - smooth01((f - f1) / float(ramp + 1))
            Tp[f, :2] = (1 - w) * P[f, :2] + w * P[ref, :2]
            Tr[f] = slerp_mats(R[f], R[ref], w)


SUPPORT_MIN_NET_CM = 25.0
PIN_MIN_NET_CM = 2.0


def build_clip(glb, clip, a, first_off):
    """-> (lt, lr, rep, rig): the re-planted clip locals"""
    rig = Rig(glb, clip)
    tgt, n = rig.tgt, rig.n
    WtR = red(rig.Wt0)
    Wd, nm = BSG.load('%s:%s' % (glb, clip))
    fl = BSG.floors_of(Wd, nm, robust=True)
    ph = {s: BSG.contact_phases(Wd, nm, s, fl) for s in ('Left', 'Right')}
    best = max(((s, q) for s in ('Left', 'Right') for q in ph[s]), key=lambda sq: sq[1]['net_cm'])
    side = a.side or (best[0] if best[1]['net_cm'] >= SUPPORT_MIN_NET_CM else None)
    rep = dict(glb=str(glb), clip=clip, support=side, first_off=first_off, phases_before=ph)
    ramp = 3
    ap = {s: (WtR[:, tgt.index[s + 'Foot']].copy(), rig.Wr0[:, tgt.index[s + 'Foot']].copy()) for s in ('Left', 'Right')}
    tg = {s: (ap[s][0].copy(), ap[s][1].copy()) for s in ('Left', 'Right')}
    w0 = n
    pins = []
    plants, hops = [], []
    t0 = None
    if side:
        sup = best[1] if best[0] == side else max(ph[side], key=lambda q: q['net_cm'])
        U = WtR[:, tgt.index[side + 'UpLeg']]
        A = ap[side][0]
        Kn = WtR[0, tgt.index[side + 'Leg']]
        Lleg = float(np.linalg.norm(Kn - U[0]) + np.linalg.norm(A[0] - Kn))
        t0 = sup['f0']
        tot, plants, hops = plan_support(U, A, Lleg, t0, n, a.reach, a.hop_min, a.hop_max, a.hop_pen, a.min_plant, first_off=first_off)
        rep['plan_cost'] = round(float(tot), 2) if plants else None
        rep['plants'] = [list(map(int, q)) for q in plants]
        rep['hops'] = [list(map(int, q)) for q in hops]
        if not plants:
            rep['error'] = 'no feasible plant plan (first_off %s)' % first_off
            return None, None, rep, rig
        Tp, Tr = tg[side]
        P_s, R_s = ap[side]
        apex = a.apex_cm / 100.0
        xy_delay, lift_frac = a.hop_xy_delay, a.hop_lift_frac
        for (pa, pb, pr) in plants:
            Tp[pa:pb + 1] = P_s[pr]
            Tr[pa:pb + 1] = R_s[pr]
        a1, b1, r1 = plants[0]
        w0 = min(w0, max(1, a1 - ramp))
        for f in range(max(1, a1 - ramp), a1):
            wgt = smooth01((f - (a1 - ramp) + 1) / float(ramp + 1))
            Tp[f, :2] = (1 - wgt) * P_s[f, :2] + wgt * P_s[r1, :2]
            Tr[f] = slerp_mats(R_s[f], R_s[r1], wgt)
        for f in range(a1, min(b1, a1 + 3) + 1):   # the pinned height of the first plant is reached over its first frames (the vanilla toe is still 1-4 cm above the ground at the touch-down)
            wgt = smooth01((f - a1 + 1) / 4.0)
            Tp[f, 2] = (1 - wgt) * P_s[f, 2] + wgt * P_s[r1, 2]
        for (ha, hb), (pa, pb, pr), (qa, qb, qr) in zip(hops, plants[:-1], plants[1:]):
            for f in range(ha, hb + 1):
                u = (f - pb) / float(qa - pb)
                sm = smooth01((u - xy_delay) / (1.0 - 2.0 * xy_delay))   # xy starts once the foot is lifted and ends before it touches down (the foot does not skate while it is still below the contact band)
                Tp[f, :2] = (1 - sm) * P_s[pr, :2] + sm * P_s[qr, :2]
                # height: zero vertical speed at lift-off and touch-down (the leg is almost straight: 1 cm of height is many degrees of knee), apex over the middle
                Tp[f, 2] = (1 - smooth01(u)) * P_s[pr, 2] + smooth01(u) * P_s[qr, 2] + apex * smooth01(u / lift_frac) * smooth01((1.0 - u) / lift_frac)
                Tr[f] = slerp_mats(R_s[pr], R_s[qr], smooth01(u))
    # every other planted phase that still glides more than PIN_MIN_NET_CM (touch-down shuffles of the first frames, the final settle of the other foot / of both feet)
    for s_ in ('Left', 'Right'):
        for q in ph[s_]:
            if s_ == side and t0 is not None and q['f1'] >= t0:
                continue   # inside the support plan (covers t0..last frame)
            if q['net_cm'] < PIN_MIN_NET_CM:
                continue
            f0_, f1_ = q['f0'], q['f1']
            if f1_ >= n - 1:
                ref = n - 1
            else:
                mid = ap[s_][0][f0_:f1_ + 1, :2].mean(0)
                ref = f0_ + int(np.argmin(np.linalg.norm(ap[s_][0][f0_:f1_ + 1, :2] - mid, axis=1)))
            pin_phase(tg[s_][0], tg[s_][1], ap[s_][0], ap[s_][1], f0_, f1_, ref, n, a.pin_ramp)
            pins.append(dict(side=s_, f0=f0_, f1=f1_, ref=ref, net_cm=q['net_cm']))
            w0 = min(w0, max(1, f0_ - a.pin_ramp))
    rep['pins'] = pins
    if w0 >= n:
        rep['error'] = 'nothing to do'
        return None, None, rep, rig
    lt, lr = rig.lt0.copy(), rig.lr0.copy()
    rep['window'] = [w0, n - 2]
    for sd in ('Left', 'Right'):
        if np.abs(tg[sd][0] - ap[sd][0]).max() < 1e-9 and np.abs(tg[sd][1] - ap[sd][1]).max() < 1e-9:
            continue
        a_, b_, c_ = (tgt.index[sd + x] for x in ('UpLeg', 'Leg', 'Foot'))
        err = IK.two_bone(tgt, lt, lr, a_, b_, c_, gltf(tg[sd][0]), max_extend=a.max_extend, bend_sigma=2.0, pole=rig.Wt0[:, b_].copy())
        IK.set_world_rot(tgt, lt, lr, c_, tg[sd][1])
        rep['ik_max_err_cm_' + sd] = round(float(err) * 100, 2)
        rep.setdefault('ik_sides', []).append(sd)
    keep = np.ones(n, bool)
    keep[w0:n - 1] = False
    lt[keep], lr[keep] = rig.lt0[keep], rig.lr0[keep]   # vanilla bit for bit outside the window and on the last frame
    ES.reanchor_helpers(tgt, lt, lr, rig.Wt0, rig.Wr0, w0 - 1, n - 1)
    restore_arm_helpers(tgt, lt, lr, rig)
    place_leg_poles(tgt, lt, lr, rig, [sd for sd in ('Left', 'Right') if sd in rep.get('ik_sides', [])], w0, n - 2)
    lt[keep], lr[keep] = rig.lt0[keep], rig.lr0[keep]
    lr[:] = S.qcontinuous(lr)
    dang = np.stack([np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[:, j] * rig.lr0[:, j], -1)), 0, 1))) for j in range(lr.shape[1])], 1)
    dtr = np.linalg.norm(lt - rig.lt0, axis=-1)
    chg = (dang > 1e-3) | (dtr > 1e-5)
    rep['changed_frames'] = [int(f) for f in np.flatnonzero(chg.any(1))]
    rep['changed_bones'] = sorted({tgt.names[j] for j in np.flatnonzero(chg.any(0))})
    rep['first_last_max_diff_deg'] = [round(float(dang[0].max()), 5), round(float(dang[-1].max()), 5)]
    return lt, lr, rep, rig


def evaluate(glb_van, clip, out_glb):
    """offline numbers of a written clip against the vanilla one: planted-foot phases (robust floors), glide, bike clearance proxy"""
    Wv, nm = BSG.load('%s:%s' % (glb_van, clip))
    Wc, _ = BSG.load('%s:%s' % (out_glb, clip))
    fl = BSG.floors_of(Wv, nm, robust=True)
    ph = {s: BSG.contact_phases(Wc, nm, s, fl) for s in ('Left', 'Right')}
    mx = max([q['net_cm'] for v in ph.values() for q in v] or [0.0])
    cl = BSG.clearance(Wv, Wc, nm)
    g = BSG.glide(Wc, nm, None, fl, robust=True)
    return dict(phases=ph, phase_net_max_cm=mx, glide_cm=g['glide_total_cm'], clearance=cl, ok=bool(mx <= 3.0 and cl['ok']))


LEG_BONES = ('LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot')


def leg_steps(rig, lr):
    """per leg bone: largest local rotation step (deg / frame) of the new clip and of the vanilla one; limit = max(15, 1.1 x vanilla) of the SAME bone"""
    st = lambda q: np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(q[1:] * q[:-1], -1)), 0, 1)))
    out = {}
    for b in LEG_BONES:
        j = rig.tgt.index[b]
        sn, sv = float(st(lr[:, j]).max()), float(st(rig.lr0[:, j]).max())
        out[b] = dict(new=round(sn, 1), vanilla=round(sv, 1), limit=round(max(15.0, 1.1 * sv), 1))
    return out


def run_clip(glb, clip, out_glb, a, report=None):
    """try the first-plant windows of --first-offs in order, keep the first candidate that passes (phases <= 3 cm, clearance proxy), else the one with the smallest violation"""
    best = None
    best_rig = None
    shapes = [tuple(float(x) for x in t.split(':')) for t in a.hop_shapes.split(',')]
    for shp in shapes:
        a.apex_cm, a.hop_lift_frac, a.hop_xy_delay = shp
        for off in [int(x) for x in a.first_offs.split(',')]:
            lt, lr, rep, rig = build_clip(glb, clip, a, off)
            if lt is None:
                print('  hop %s first_off %2d: %s' % (shp, off, rep.get('error')))
                cand = (1e9, rep, None, None)
            else:
                B.write_out(rig.tgt, out_glb, clip, rig.times, lt, lr, rig.extras)
                ev = evaluate(glb, clip, out_glb)
                ls = leg_steps(rig, lr)
                rep['eval'] = ev
                rep['leg_steps'] = ls
                rep['hop_shape'] = shp
                over = max([max(0.0, v['new'] - v['limit']) for v in ls.values()])
                viol = (max(0.0, ev['phase_net_max_cm'] - 3.0) + max(0.0, ev['clearance']['inward_cm'] - BSG.INWARD_CM) + max(0.0, ev['clearance']['down_cm'] - BSG.DOWN_CM)
                        + max(0.0, ev['clearance']['hand_cm'] - BSG.HAND_CM) + over)
                cand = (viol, rep, lt, lr)
                print('  hop %s first_off %2d: plants %s hops %s pins %s | phase net %.1f cm glide %.1f clear %s | leg step over limit %.1f deg -> viol %.1f' % (
                    shp, off, rep.get('plants'), rep.get('hops'), [(q['side'][0], q['f0'], q['f1']) for q in rep['pins']], ev['phase_net_max_cm'], ev['glide_cm'],
                    (ev['clearance']['inward_cm'], ev['clearance']['down_cm']), over, viol))
            if best is None or cand[0] < best[0]:
                best, best_rig = cand, (rig if lt is not None else None)
            if cand[0] <= 0:
                break
        if best[0] <= 0:
            break
    viol, rep, lt, lr = best
    if lt is None:
        print('  NO RESULT:', rep.get('error'))
        return rep
    B.write_out(best_rig.tgt, out_glb, clip, best_rig.times, lt, lr, best_rig.extras)
    rep['violation'] = round(float(viol), 2)
    rep['args'] = {k: v for k, v in vars(a).items() if k not in ('all', 'pick_root')}   # full parameter set of the run (reproducibility)
    if report:
        json.dump(rep, open(report, 'w'), indent=1, default=str)
    return rep


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb')
    p.add_argument('--clip')
    p.add_argument('--out')
    p.add_argument('--report')
    p.add_argument('--all', help='vanilla dir with tpp_p?a_sportbike[N].anims.glb: every exit_l / exit_r is re-planted into --pick-root/<g>a_sb[N]_<l|r>/pick.anims.glb')
    p.add_argument('--pick-root')
    p.add_argument('--reach', type=float, default=0.97, help='fraction of thigh + shin the pinned ankle may be away from the hip joint')
    p.add_argument('--hop-min', type=int, default=5)
    p.add_argument('--hop-max', type=int, default=13)
    p.add_argument('--hop-pen', type=float, default=12.0, help='cost (m x frames of deviation from the vanilla foot path) of one extra hop')
    p.add_argument('--min-plant', type=int, default=7)
    p.add_argument('--apex-cm', type=float, default=10.0)
    p.add_argument('--hop-shapes', default='8:0.40:0.06,10:0.30:0.0,8:0.40:0.05,7.5:0.40:0.05,7:0.45:0.08,6:0.50:0.10', help='apex_cm:lift_frac:xy_delay candidates of the hop (tried in order, with every first-off; the first one that passes every offline gate wins)')
    p.add_argument('--pin-ramp', type=int, default=5, help='frames: blend of a pinned touch-down / final spot into the vanilla foot path')
    p.add_argument('--max-extend', type=float, default=0.9998)
    p.add_argument('--first-offs', default='8,5,3,11', help='the first plant pins the vanilla spot of a frame within t0..t0+off (tried in this order; the knee of a spot that is far from the hips lowers too much at the touch-down)')
    p.add_argument('--side', help='support side (default: the side of the longest planted phase, if it glides >= 25 cm)')
    a = p.parse_args()
    if a.all:
        results = {}
        for g in ('pwa', 'pma'):
            for b in ('', '2', '3'):
                glb = os.path.join(a.all, 'tpp_%s_sportbike%s.anims.glb' % (g, b))
                for c in ('l', 'r'):
                    tag = '%s_sb%s_%s' % (g, b, c)
                    od = os.path.join(a.pick_root, tag)
                    os.makedirs(od, exist_ok=True)
                    print('==', tag)
                    rep = run_clip(glb, 'exit_' + c, os.path.join(od, 'pick.anims.glb'), a, os.path.join(od, 'replant.json'))
                    results[tag] = dict(violation=rep.get('violation'), phase_net_max_cm=(rep.get('eval') or {}).get('phase_net_max_cm'), error=rep.get('error'))
        print(json.dumps(results, indent=1))
        return
    assert a.glb and a.clip and a.out
    rep = run_clip(a.glb, a.clip, a.out, a, a.report)
    print('RESULT violation', rep.get('violation'), rep.get('error'))


if __name__ == '__main__':
    main()
