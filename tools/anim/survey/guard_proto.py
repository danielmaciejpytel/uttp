"""Stage 4 skin confirmation + stage 5 GO / NO-GO prototype of the generic runtime anatomy guard (offline, numpy).

  python -I guard_proto.py --rig woman --confirm 40          skin strain of the worst frames of the 40 most severe VISIBLE clips -> skin_<rig>.json
  python -I guard_proto.py --rig woman --proto [auto|<setkey/clip> ...]   guard prototype on ~12 visible worst clips -> proto_<rig>.json
Guard (per frame, what the DLL would see; joints = guard_limits.GUARD, limits = guard_limits_<rig>.json, the rig's own joint frames):
  1. local rotation rest-relative Rel = Lrest^T L, q = swing * twist (twist about local +X), each of twist / swing_y / swing_z soft-clamped:
     identity inside [lo + K, hi - K], C1 tanh knee saturating at the limit (K = 5 deg = the tolerance band: identity up to the corpus
     p0.5 / p99.5, never beyond the limit) - the same knee as MeshArmClavicleLimit.SoftLimitAngleDeg, so entering / leaving a limit never pops;
  2. translation reset: local translation deviation from the rest local soft-limited to trans_dev_cm (knee at half of it) - removes stretch
     (seg_len_dev, clav_shift) that no rotation clamp can fix; Root / Hips never touched (root motion, pelvis placement);
  3. two-bone IK (UpperArm -> ForeArm -> Hand) restores each hand's WORLD position of the shown pose (pole = elbow plane of the clamped
     pose, reach clamped), then the hand's world rotation is restored (grip / weapon / prop orientation); A clips with counted legs: the same
     for the feet (UpLeg -> Leg -> Foot);
  4. scored AFTER IK: the survey deformation gate (survey_metrics), skinned strain (redo_skin_check.stats vs <REDO>\\skin_ref.json, read
     only), hand residual (cm, with and without IK), elbow displacement, continuity (rot_step guard vs vanilla), foot residual.
Verdict per clip / state class: GO when the hand residual stays <= 2 cm on every frame, the skinned strain of the worst frames is inside
the reference band (no non-hand region over skin_ref + margin) and the guard adds no pop (<= 10 deg extra one-frame rotation vs
vanilla); NO-GO otherwise (that state needs an offline rebuild).
Writes only Build\\work\\anim-survey-20261109\\{skin,proto,proto_<rig>.json,skin_<rig>.json}.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_metrics as SM  # noqa: E402   (sets UTPP_RIG from --rig, imports the melee tools)
import guard_limits as GL  # noqa: E402
import survey_common as SC  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_skin_check as RSC  # noqa: E402
import demo_skin  # noqa: E402

RIG = SM.RIG
K_DEG = 5.0
HAND_GO_CM = 2.0
POP_GO_DEG = 10.0          # max extra one-frame rotation of any body bone vs vanilla (continuity)
SKIN_DIR = os.path.join(SC.WORK, 'skin', RIG)
LIMITS = json.load(open(os.path.join(SC.WORK, 'guard_limits_%s.json' % RIG), encoding='utf-8'))['joints']


def soft(v, lo, hi, k=K_DEG):
    """C1 soft clamp into [lo, hi]: identity inside [lo + k, hi - k], tanh knee outside."""
    v = np.asarray(v, float)
    a, b = lo + k, hi - k
    if b < a:
        a = b = (lo + hi) / 2.0
    out = v.copy()
    up = v > b
    out[up] = b + (hi - b) * np.tanh((v[up] - b) / max(hi - b, 1e-6))
    dn = v < a
    out[dn] = a - (a - lo) * np.tanh((a - v[dn]) / max(a - lo, 1e-6))
    return out


def rmin(u, v):
    """minimal rotations [T,3,3] taking unit u [T,3] to unit v [T,3]."""
    u = u / np.linalg.norm(u, axis=1, keepdims=True)
    v = v / np.linalg.norm(v, axis=1, keepdims=True)
    ax = np.cross(u, v)
    s = np.linalg.norm(ax, axis=1)
    c = np.clip(np.sum(u * v, 1), -1, 1)
    ang = np.arctan2(s, c)
    ax = np.where(s[:, None] > 1e-9, ax / np.maximum(s[:, None], 1e-12), np.array([1.0, 0, 0]))
    return R.from_rotvec(ax * ang[:, None]).as_matrix()


def fk(par, Lr, lt):
    T, N = Lr.shape[:2]
    W = np.zeros((T, N, 4, 4))
    W[..., 3, 3] = 1
    for j in range(N):
        p = par[j]
        if p < 0:
            W[:, j, :3, :3], W[:, j, :3, 3] = Lr[:, j], lt[:, j]
        else:
            W[:, j, :3, :3] = W[:, p, :3, :3] @ Lr[:, j]
            W[:, j, :3, 3] = W[:, p, :3, 3] + np.einsum('tij,tj->ti', W[:, p, :3, :3], lt[:, j])
    return W


def _signed(a, b, axis):
    return np.arctan2(np.sum(np.cross(a, b) * axis, 1), np.sum(a * b, 1))


def two_bone(W, Lr, lt, par, ix, a, b, c, target, Rtarget, rg=None):
    """Analytic HINGE IK on chain a -> b -> c: the elbow / knee stays a pure hinge about the joint's own local Z (the rig convention,
    elbow_flex = -rz), so the IK never creates hinge deviation or roll. Elbow plane = plane of the clamped pose (pole = its elbow offset
    from the shoulder-target line); reach clamped to 99.95 %; flexion sign kept anatomical (no reverse bend). Then the end effector's
    world rotation is set to Rtarget (grip / foot orientation)."""
    T = len(W)
    Pa, Pb = W[:, ix[a], :3, 3], W[:, ix[b], :3, 3]
    la = np.linalg.norm(lt[:, ix[b]], axis=1)
    lb = np.linalg.norm(lt[:, ix[c]], axis=1)
    sa = np.sign(rg.Lrest_t[ix[b]][0]) or 1.0           # bone axis sign (+X left side, -X right side)
    sb = np.sign(rg.Lrest_t[ix[c]][0]) or 1.0
    dv = target - Pa
    dn = np.linalg.norm(dv, axis=1)
    d = np.clip(dn, np.abs(la - lb) + 1e-4, (la + lb) * 0.9995)
    di = dv / np.maximum(dn[:, None], 1e-9)
    pv = (Pb - Pa) - di * np.sum((Pb - Pa) * di, 1, keepdims=True)
    pn = np.linalg.norm(pv, axis=1)
    pv = pv / np.maximum(pn[:, None], 1e-9)
    prev = None
    for t in range(T):                                     # pole continuity: the elbow-plane pole is blended with the previous frame's
        raw = pv[t] if pn[t] > 1e-5 else np.cross(di[t], np.array([0, 0, 1.0]))   # pole by the limb's bend (a straight limb keeps the
        if prev is not None:                                                          # previous plane; no threshold -> no switch pop)
            w = min(1.0, pn[t] / 0.05) ** 2
            q = prev - di[t] * (prev @ di[t])
            raw = w * raw + (1 - w) * q / max(np.linalg.norm(q), 1e-9)
            raw = raw - di[t] * (raw @ di[t])
        pv[t] = raw / max(np.linalg.norm(raw), 1e-9)
        prev = pv[t]
    cosA = np.clip((la ** 2 + d ** 2 - lb ** 2) / (2 * la * d), -1, 1)
    sinA = np.sqrt(1 - cosA ** 2)
    Pb2 = Pa + la[:, None] * (cosA[:, None] * di + sinA[:, None] * pv)
    Pc2 = Pa + di * d[:, None]
    ua = (Pb2 - Pa) / la[:, None]
    vb = (Pc2 - Pb2) / np.maximum(np.linalg.norm(Pc2 - Pb2, axis=1, keepdims=True), 1e-9)
    n = np.cross(di, pv)                                   # normal of the (shoulder, target, pole) plane: defined for a straight limb too
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    Lb_rest = rg.Lrest[ix[b]]                             # forearm / shin rest local rotation in the upper-arm / thigh frame
    e1 = np.array([sa, 0.0, 0.0])
    z = Lb_rest @ np.array([0.0, 0.0, 1.0])
    e2 = z - e1 * (z @ e1)
    e2 /= np.linalg.norm(e2)
    E = np.stack([e1, e2, np.cross(e1, e2)], 1)            # columns in the parent-bone frame
    cand = []
    for sg in (1.0, -1.0):
        f2 = sg * n
        F = np.stack([ua, f2, np.cross(ua, f2)], 2)        # [T,3,3] columns in world
        Wa_ = F @ E.T[None]
        axis = Wa_ @ z
        d0 = (Wa_ @ Lb_rest) @ np.array([sb, 0.0, 0.0])
        cand.append((Wa_, _signed(d0, vb, axis)))          # rotation about the hinge (local Z of b)
    rev = np.stack([np.maximum(np.degrees(c[1]), 0.0) for c in cand], 1)     # reverse-bend amount (elbow_flex = -rz >= 0 is anatomical)
    k = int(rev[0, 1] < rev[0, 0])                         # hinge side chosen on the first frame and KEPT: a side switch is a 180 deg
    Wa, th = cand[k]                                       # upper-arm twist pop (measured 100-160 deg); a would-be reverse bend is
    th = np.minimum(th, 0.0)                               # clamped straight instead (the target is then missed -> hand residual)
    Rz = R.from_rotvec(np.stack([np.zeros(T), np.zeros(T), th], 1)).as_matrix()
    Wb = Wa @ Lb_rest[None] @ Rz
    Wp = W[:, par[ix[a]], :3, :3]
    Lr = Lr.copy()
    Lr[:, ix[a]] = np.swapaxes(Wp, -1, -2) @ Wa
    Lr[:, ix[b]] = Lb_rest[None] @ Rz
    Lr[:, ix[c]] = np.swapaxes(Wb, -1, -2) @ Rtarget
    return Lr


def guard(W0, ctx, feet=True, ik=True):
    """W0 shown pose [T,73,4,4] -> (W1 clamped (no IK), W2 guarded, stats of the clamp)."""
    rg, par, ix = ctx.rg, ctx.par, ctx.ix
    Lr, lt = GL.locals_of(W0, rg.par)
    Lr, lt = Lr.copy(), lt.copy()
    nclamp = {}
    for n, lim in LIMITS.items():
        j = ix[n]
        Rel = np.swapaxes(rg.Lrest[j], -1, -2)[None] @ Lr[:, j]
        tw, sy, sz, _ = GL.decompose(Rel)
        tw, sy, sz = (np.degrees(np.unwrap(np.radians(x))) for x in (tw, sy, sz))   # temporal coherence across the +-180 branch cut (DLL: previous-frame state)
        tw2, sy2, sz2 = soft(tw, *lim['twist']), soft(sy, *lim['swing_y']), soft(sz, *lim['swing_z'])
        moved = np.maximum.reduce([np.abs(tw2 - tw), np.abs(sy2 - sy), np.abs(sz2 - sz)])
        if moved.max() > 1e-3:
            Lr[:, j] = rg.Lrest[j][None] @ GL.compose(tw2, sy2, sz2)
            nclamp[n] = round(float(moved.max()), 1)
        dvec = lt[:, j] - rg.Lrest_t[j]
        m = np.linalg.norm(dvec, axis=1) * 100
        M = lim['trans_dev_cm']
        m2 = soft(m, -1e3, M, k=M / 2)
        sc = np.where(m > 1e-6, m2 / np.maximum(m, 1e-9), 1.0)
        if (m - m2).max() > 0.05:
            lt[:, j] = rg.Lrest_t[j] + dvec * sc[:, None]
            nclamp[n + '.t'] = round(float((m - m2).max()), 2)
    Wc = fk(par, Lr, lt)                       # clamped + translation reset, no IK
    if not ik:
        return Wc, Wc, nclamp
    W1 = Wc
    feet = feet and any(k.startswith(('LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot')) and v > 1.0
                        for k, v in nclamp.items())
    for S in ('Left', 'Right'):
        Lr = two_bone(W1, Lr, lt, par, ix, S + 'Arm', S + 'ForeArm', S + 'Hand', W0[:, ix[S + 'Hand'], :3, 3], W0[:, ix[S + 'Hand'], :3, :3], rg)
        W1 = fk(par, Lr, lt)
        if feet:
            Lr = two_bone(W1, Lr, lt, par, ix, S + 'UpLeg', S + 'Leg', S + 'Foot', W0[:, ix[S + 'Foot'], :3, 3], W0[:, ix[S + 'Foot'], :3, :3], rg)
            W1 = fk(par, Lr, lt)
    for S in ('Left', 'Right'):                            # wrist soft clamp after the grip restore (trades hand orientation for wrist strain)
        j, lim = ix[S + 'Hand'], LIMITS[S + 'Hand']
        Rel = np.swapaxes(rg.Lrest[j], -1, -2)[None] @ Lr[:, j]
        tw, sy, sz, _ = GL.decompose(Rel)
        tw, sy, sz = (np.degrees(np.unwrap(np.radians(x))) for x in (tw, sy, sz))
        Lr[:, j] = rg.Lrest[j][None] @ GL.compose(soft(tw, *lim['twist']), soft(sy, *lim['swing_y']), soft(sz, *lim['swing_z']))
    W1 = fk(par, Lr, lt)
    return Wc, W1, nclamp


def skin_strain(Wf, tag):
    """skinned p98 edge strain per region of the frames Wf [n,73,4,4] -> (stat [n,R], excess vs skin_ref + margin [n,R] (hands -1))."""
    os.makedirs(SKIN_DIR, exist_ok=True)
    names, par, ref, rest = SM.BB.rig()
    Wf = np.asarray(Wf, float)
    if len(Wf) < 3:
        Wf = np.concatenate([Wf] * 3)[:3]
    p = os.path.join(SKIN_DIR, tag + '.dump.npz')
    np.savez(p, W=Wf, rest=rest, names=np.array(names), parents=np.array(par), fps=30)
    demo_skin.skin(p, os.path.join(SKIN_DIR, tag + '.skin'), RIG, tag)
    refj = json.load(open(os.path.join(SM.BB.REDO, 'skin_ref.json'), encoding='utf-8'))
    W0 = SM.BB.inplace(np.load(os.path.join(SM.BB.REDO, 'base_pose.npz'))['W'], names)[0]
    st = RSC.stats(os.path.join(SKIN_DIR, tag + '.skin'), SM.BB.rig_for_gate(), W0)
    allow = np.array([refj['regions'][r] for r in SM.AG.SKIN_REGIONS]) + refj['margin']
    ex = st - allow[None]
    ex[:, [r.endswith('Hand') for r in SM.AG.SKIN_REGIONS]] = -1.0
    return st, ex


def frames_around(top, T):
    fr = []
    for f in top:
        for g in (f - 1, f, f + 1):
            if 0 <= g < T and g not in fr:
                fr.append(g)
    return fr


def load_shown(ctx, key, met):
    sk, clip = key.split('/', 1)
    d = MC.load_dump(os.path.join(SC.DUMPS, RIG, sk, clip + '.npz'))
    return ctx.shown(d, met['mode'])


def confirm(n):
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    ctx = SM.Ctx()
    vis = [(k, m) for k, m in met.items() if m['cls'] in ('A', 'B', 'E') and m['top_frames'] and not m.get('degenerate_export')]
    vis.sort(key=lambda km: -km[1]['severity'])
    seen, per_set = set(), []
    for k, m in vis:                       # the most severe clip of each set, sets by severity
        if m['setkey'] not in seen:
            seen.add(m['setkey'])
            per_set.append((k, m))
    vis = per_set
    out_p = os.path.join(SC.WORK, 'skin_%s.json' % RIG)
    out = json.load(open(out_p, encoding='utf-8')) if os.path.exists(out_p) else {}
    for k, m in vis[:n]:
        if k in out:
            continue
        W = load_shown(ctx, k, m)
        fr = frames_around(m['top_frames'], len(W))
        st, ex = skin_strain(W[fr], 'c__' + k.replace('/', '__'))
        r = int(np.unravel_index(ex.argmax(), ex.shape)[1])
        out[k] = dict(frames=fr, skin_excess_max=round(float(ex.max()), 3), skin_region=SM.AG.SKIN_REGIONS[r],
                      skin_frame=fr[int(ex[:, r].argmax())], skin_stat_max=round(float(st.max()), 3),
                      confirmed=bool(ex.max() > 0), regions_over=[SM.AG.SKIN_REGIONS[i] for i in range(ex.shape[1]) if ex[:, i].max() > 0])
        SC.jdump(out, out_p)
        print('SKIN %s excess %.3f %s' % (k, ex.max(), out[k]['regions_over']))
    print('WROTE', out_p)


def proto(keys):
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    ctx = SM.Ctx()
    out_p = os.path.join(SC.WORK, 'proto_%s.json' % RIG)
    out = json.load(open(out_p, encoding='utf-8')) if os.path.exists(out_p) else {}
    for k in keys:
        m = met[k]
        W0 = load_shown(ctx, k, m)
        feet = m['cls'] in ('A', 'E') and m['legs'] and m['mode'] == 'full'
        W1, W2, ncl = guard(W0, ctx, feet=feet)
        f0, e0 = ctx.measure(W0, legs=m['legs'])
        f2, e2 = ctx.measure(W2, legs=m['legs'])
        s0, s2 = SM.summarize(e0, len(W0)), SM.summarize(e2, len(W2))
        ix = ctx.ix
        hand = lambda W: np.stack([W[:, ix[s + 'Hand'], :3, 3] for s in ('Left', 'Right')], 1)  # noqa: E731
        res_ik = np.linalg.norm(hand(W2) - hand(W0), axis=2).max(1) * 100
        res_no = np.linalg.norm(hand(W1) - hand(W0), axis=2).max(1) * 100
        elb = np.linalg.norm(np.stack([W2[:, ix[s + 'ForeArm'], :3, 3] - W0[:, ix[s + 'ForeArm'], :3, 3] for s in ('Left', 'Right')], 1),
                             axis=2).max(1) * 100
        foot = np.linalg.norm(np.stack([W2[:, ix[s + 'Foot'], :3, 3] - W0[:, ix[s + 'Foot'], :3, 3] for s in ('Left', 'Right')], 1),
                              axis=2).max(1) * 100
        rs0, rs2 = np.asarray(f0['rot_step']), np.asarray(f2['rot_step'])
        rot_err = max(float(MC.rot_delta_deg(W0[:, ix[s_ + 'Hand'], :3, :3], W2[:, ix[s_ + 'Hand'], :3, :3]).max()) for s_ in ('Left', 'Right'))
        fr = frames_around(m['top_frames'] or [m['worst_frame']], len(W0))
        tag = k.replace('/', '__')
        sv, xv = skin_strain(W0[fr], 'v__' + tag)
        sg, xg = skin_strain(W2[fr], 'g__' + tag)
        np.savez_compressed(os.path.join(SKIN_DIR, 'proto__' + tag + '.npz'), W0=W0[fr].astype(np.float32), W2=W2[fr].astype(np.float32),
                            frames=np.array(fr))
        go = bool(res_ik.max() <= HAND_GO_CM and xg.max() <= 0 and float((rs2 - rs0).max()) <= POP_GO_DEG)
        out[k] = dict(cls=m['cls'], state=m['state'], mode=m['mode'], frames=len(W0),
                      fail_pct_vanilla=s0['fail_pct'], fail_pct_guard=s2['fail_pct'], worst_vanilla=[s0['worst_feature'], s0['worst_excess']],
                      worst_guard=[s2['worst_feature'], s2['worst_excess']], trans_vanilla=s0['trans_excess'], trans_guard=s2['trans_excess'],
                      hand_residual_cm_max=round(float(res_ik.max()), 2), hand_residual_cm_p95=round(float(np.percentile(res_ik, 95)), 2),
                      hand_disp_noik_cm_max=round(float(res_no.max()), 2), hand_rot_err_deg_max=round(rot_err, 1), elbow_disp_cm_max=round(float(elb.max()), 2),
                      foot_residual_cm_max=round(float(foot.max()), 2) if feet else None,
                      rot_step_vanilla_max=round(float(rs0.max()), 1), rot_step_guard_max=round(float(rs2.max()), 1),
                      pop_deg=round(float((rs2 - rs0).max()), 1), skin_frames=fr,
                      skin_excess_vanilla=round(float(xv.max()), 3), skin_excess_guard=round(float(xg.max()), 3),
                      skin_regions_over_vanilla=[SM.AG.SKIN_REGIONS[i] for i in range(xv.shape[1]) if xv[:, i].max() > 0],
                      skin_regions_over_guard=[SM.AG.SKIN_REGIONS[i] for i in range(xg.shape[1]) if xg[:, i].max() > 0],
                      clamped=ncl, verdict='GO' if go else 'NO-GO')
        SC.jdump(out, out_p)
        o = out[k]
        print('PROTO %-60s %s fail %5.1f->%5.1f%% skin %.3f->%.3f hand %.2f (noIK %.1f) elbow %.1f pop %.1f %s' % (
            k, m['cls'], o['fail_pct_vanilla'], o['fail_pct_guard'], o['skin_excess_vanilla'], o['skin_excess_guard'], o['hand_residual_cm_max'],
            o['hand_disp_noik_cm_max'], o['elbow_disp_cm_max'], o['pop_deg'], o['verdict']))
    print('WROTE', out_p)


def auto_keys(n_per=1):
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    groups = {}
    for k, m in met.items():
        if m['cls'] not in ('A', 'B', 'E') or not m['top_frames'] or m.get('degenerate_export') or m.get('glitch_frames'):
            continue
        clip = m['clip']
        g = m['state']
        if 'exploration' in g:
            g = 'ladder' if clip.startswith('ladder') else 'knockdown' if 'knockdown' in clip else 'death' if clip.startswith('death') else \
                'swim' if 'swim' in clip else 'vault/climb'
        groups.setdefault(g, []).append((m['severity'], k))
    keys = []
    for g, lst in sorted(groups.items()):
        lst.sort(reverse=True)
        keys += [k for _, k in lst[:n_per]]
    return keys


if __name__ == '__main__':
    a = sys.argv[1:]
    if '--confirm' in a:
        confirm(int(a[a.index('--confirm') + 1]))
    if '--proto' in a:
        ks = [x for x in a[a.index('--proto') + 1:] if not x.startswith('--') and x not in ('woman', 'man')]
        proto(auto_keys() if (not ks or ks == ['auto']) else ks)
