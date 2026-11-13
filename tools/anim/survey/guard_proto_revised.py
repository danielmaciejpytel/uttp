"""UTPP generic anti-deformation guard - PROTOTYPE v2 (offline, numpy, 2026-11-09).
Per frame, only the current pose + a small temporal state (what an FK-stage DLL pass can afford): no optimisation over the clip.

  python -I guard_proto_revised.py --rig woman|man --probe [v1|<setkey/clip> ...]
        reach / swivel / contact diagnostics without skinning -> Build\\work\\anim-guard-v2-20261109\\probe_<rig>.json (+ stdout)
  import guard_proto_revised as G2;  W2, diag = G2.run_clip(W0, ctx, mode, legs)          (used by guard_eval.py)

GuardV2.step(frame) - joints = guard_limits.GUARD masked by the mode of the state (who owns the bones in game):
  full (A / E release states): Spine..Head, clavicles, arms, legs;  melee (B melee mask): Spine1..Head, clavicles, arms;
  action (B weapon action): clavicles, arms only.  Root / Hips, fingers, weapon and IK helper bones are never written.
  1. spine / neck / head: v1 soft clamp of (twist, swing_y, swing_z) (C1 tanh knee K = 5 deg, identity inside [lo + K, hi - K])
     + local translation reset; branch continuity from the PREVIOUS FRAME's decomposition (state), not np.unwrap over the clip.
  2. clavicles: v1 rotation clamp + translation reset, then REACH-AWARE: when the hand target is beyond the comfortable reach of
     the rigid arm (REACH_FRAC * (upper + fore) + TOL_USE_CM), the clavicle turns toward it about its root by the minimal angle
     (re-clamped into its own band). beta = the fraction of the clavicle correction given back (slerp / lerp of its local transform
     toward the clip; 1 = the clip's clavicle): the smallest of {beta_reach (bisection, incl. the near limit), BETA_LEVELS} within
     +-BETA_RATE of the previous frame (the reach part is not rate limited) at which the arm after IK stays within
     max(LIM_TOL_DEG, the clip's own arm excess + LIM_TOL_DEG) = IK -> limit -> IK, the contact-priority fallback (deformation kept
     exactly where the contact needs it; reported). The spine is never given back: 'gamma_need' is a diagnostic only (report.txt 3).
  3. arms (right first, then left) and legs (full mode with legs): LIMIT-AWARE two-bone IK. Elbow / knee = pure hinge about the
     joint's local Z (flex = -swing_z >= 0, anatomical side chosen per candidate). The swivel (elbow on its circle) is chosen
     from SWIVEL_N candidates (+ a fine pass) minimising: excess beyond the HARD band (upper-arm |twist| <= 105, upper arm vs
     chest <= 160, hand twist (pronation) +-130, hand swing = table band) x W_HARD, excess beyond the corpus band x W_PREF,
     distance to the clip's elbow plane (intent) and to the previous frame's swivel (continuity). The applied swivel offset is
     rate limited (SWIVEL_RATE_DEG per frame). No twist joints exist in the 73 / 152-joint rig: twist is split between the
     upper-arm twist (swivel) and the hand roll (pronation); the forearm stays a pure hinge (ForearmTwist.hpp convention).
     Upper-arm twist still beyond the hard band is clamped only as far as the hand stays within TOL_USE_CM of its target
     (contact priority, bisection), then the wrist soft clamp (hand world rotation traded for wrist strain, reported).
  4. contacts: target = the clip's hand / foot world transform. LEFT hand in rigid weapon contact (LeftHand in WeaponRight space,
     < CONTACT_MAX_M, steady vs the previous frame) -> target = guarded WeaponRight * that relative transform (right arm first).
  5. weight: 0 -> 1 from state entry, 1 -> 0 after exit, rate aware: dw = clip(STEP_CAP_DEG / correction, dt / T_MAX, dt / T_MIN)
     (correction = max joint rotation guard vs clip, translation weighted 5 deg / cm). 0 < w < 1: per-joint slerp clip -> guard
     + a minimal-change two-bone contact fix (elbow about its hinge, then the upper arm about the shoulder) + hand world rotation.
  Rate limits apply to the PARAMETERS (swivel offset, clavicle reach angle, beta, weight), never to output locals, so the IK
  closes the contact on every frame.  First frame of an episode: parameters initialised without rate limits (w = 0 hides them).
"""
import copy
import json
import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ARGS = list(sys.argv)                    # base_body strips --rig from sys.argv at import
import survey_metrics as SM  # noqa: E402   (reads --rig, imports the melee tools)
import guard_limits as GL  # noqa: E402
import guard_proto as GP  # noqa: E402     v1 (LIMITS of the rig, soft(), guard())
import survey_common as SC  # noqa: E402
import melee_common as MC  # noqa: E402

RIG = SM.RIG
OUT = os.path.join(SC.ROOT, 'Build', 'work', 'anim-guard-v2-20261109')
DT = 1.0 / SC.FPS
LIM = GP.LIMITS

P = dict(K_DEG=5.0,
         ARM_TWIST_HARD=105.0,       # |upper-arm twist| human bound (anatomy_gate uparm_twist; the gate allows max(105, corpus + 6))
         ARM_CHEST_HARD=160.0,       # upper arm vs chest, rest-relative swing (anatomy_gate uparm_swing 165)
         HAND_TWIST_HARD=130.0,      # pronation (hand roll) bound, = v1 table
         REACH_FRAC=0.985, IK_MAX_FRAC=0.9995, TOL_USE_CM=1.0,
         SWIVEL_N=48, SWIVEL_RATE_DEG=8.0, W_HARD=10.0, W_PREF=1.0, W_PREF_SWING=0.25, W_INTENT=0.02, W_TEMP=0.1,
         CLAV_RATE_DEG=6.0, BETA_RATE=0.2,
         STEP_CAP_DEG=4.0, T_MIN=0.15, T_MAX=0.6, TRANS_DEG_PER_CM=5.0,
         CONTACT_MAX_M=0.45, CONTACT_STEADY_CM=1.0, CONTACT_RELEASE_CM=2.5, N_POST=20,
         HAND_SY_HARD=(-80.0, 75.0), HAND_SZ_HARD=(-35.0, 35.0),   # gate-consistent wrist bands (union of human + corpus)
         HAND_ROT_TOL_DEG=15.0, FLEX_MAX=150.0, W_AXIS=3000.0,
         LIM_TOL_DEG=2.0, BETA_LEVELS=(0.25, 0.5, 0.75, 1.0),   # IK -> limit -> IK: levels of the clavicle correction given back
         GAMMA_DIAG=True)            # gamma_need (spine give-back the hands would need) is computed as a diagnostic, never applied

SPINE = ['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head']
CLAV = {'Left': 'LeftShoulder', 'Right': 'RightShoulder'}
BODY_POP = [b for b in ('Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm',
                        'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot',
                        'RightFoot')]   # anatomy_gate BODY without Hips (= rot_step joints)
REGIONS = dict(spine=['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head'],
               arm_L=['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand'], arm_R=['RightShoulder', 'RightArm', 'RightForeArm', 'RightHand'],
               leg_L=['LeftUpLeg', 'LeftLeg', 'LeftFoot'], leg_R=['RightUpLeg', 'RightLeg', 'RightFoot'])


# ------------------------------------------------------------------------------------------------ small math
def nrm(v, eps=1e-12):
    v = np.asarray(v, float)
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), eps)


def wrap(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def rot_axis(ax, ang):
    """ax [...,3] unit, ang [...] rad -> [...,3,3]."""
    ax = np.asarray(ax, float)
    ang = np.asarray(ang, float)
    rv = ax * ang[..., None]
    sh = rv.shape[:-1]
    return R.from_rotvec(rv.reshape(-1, 3)).as_matrix().reshape(sh + (3, 3))


def rmin1(u, v):
    """minimal rotation taking direction u to direction v (3x3)."""
    u, v = nrm(u), nrm(v)
    ax = np.cross(u, v)
    s = np.linalg.norm(ax)
    c = float(np.clip(u @ v, -1, 1))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        p = np.cross(u, [1.0, 0, 0])
        if np.linalg.norm(p) < 1e-6:
            p = np.cross(u, [0, 1.0, 0])
        return rot_axis(nrm(p), np.pi)
    return rot_axis(ax / s, math.atan2(s, c))


def ang_deg(Ra, Rb):
    """angle (deg) of Ra^T Rb, broadcast."""
    M = np.swapaxes(Ra, -1, -2) @ Rb
    return np.degrees(np.arccos(np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))


def slerp_m(Ra, Rb, w):
    """per-joint slerp of rotation matrices [n,3,3] by scalar w."""
    if w <= 0:
        return Ra.copy()
    if w >= 1:
        return Rb.copy()
    d = R.from_matrix(np.swapaxes(Ra, -1, -2) @ Rb).as_rotvec()
    return Ra @ R.from_rotvec(d * w).as_matrix()


def signed(a, b, axis):
    return np.arctan2(np.sum(np.cross(a, b) * axis, -1), np.sum(a * b, -1))


def fk1(par, L, t):
    """one frame: local rot [N,3,3], trans [N,3] -> world rot [N,3,3], pos [N,3] (parents first)."""
    N = len(par)
    Wr, Wp = np.empty((N, 3, 3)), np.empty((N, 3))
    for j in range(N):
        p = par[j]
        if p < 0:
            Wr[j], Wp[j] = L[j], t[j]
        else:
            Wr[j] = Wr[p] @ L[j]
            Wp[j] = Wp[p] + Wr[p] @ t[j]
    return Wr, Wp


def to_W(Wr, Wp):
    W = np.zeros(Wr.shape[:-2] + (4, 4))
    W[..., :3, :3], W[..., :3, 3], W[..., 3, 3] = Wr, Wp, 1.0
    return W


def soft1(v, lo, hi, k):
    return float(GP.soft(np.array([float(v)]), lo, hi, k)[0])


def excess(v, lo, hi):
    return np.maximum(lo - v, 0.0) + np.maximum(v - hi, 0.0)


# ------------------------------------------------------------------------------------------------ guard v2
class GuardV2:
    def __init__(self, ctx, mode, legs, p=None):
        self.p = dict(P, **(p or {}))
        self.ctx, self.rg, self.ix = ctx, ctx.rg, ctx.ix
        self.par = list(ctx.rg.par)
        self.mode, self.legs = mode, bool(legs)
        self.spine = SPINE if mode == 'full' else (SPINE[1:] if mode == 'melee' else [])
        self.limbs = [('Right', 'Arm', 'ForeArm', 'Hand'), ('Left', 'Arm', 'ForeArm', 'Hand')]
        if mode == 'full' and self.legs:
            self.limbs += [('Left', 'UpLeg', 'Leg', 'Foot'), ('Right', 'UpLeg', 'Leg', 'Foot')]
        names = self.spine + list(CLAV.values()) + [s + x for s, a, b, c in self.limbs for x in (a, b, c)]
        self.joints = [self.ix[n] for n in names]
        self.const = {}
        rg = self.rg
        for s, a, b, c in self.limbs:
            ia, ib, ic = self.ix[s + a], self.ix[s + b], self.ix[s + c]
            sa = np.sign(rg.Lrest_t[ib][0]) or 1.0
            sb = np.sign(rg.Lrest_t[ic][0]) or 1.0
            z = rg.Lrest[ib] @ np.array([0.0, 0.0, 1.0])
            e1 = np.array([sa, 0.0, 0.0])
            e2 = nrm(z - e1 * (z @ e1))
            self.const[s + a] = dict(ia=ia, ib=ib, ic=ic, sa=sa, sb=sb, z=z, Ef=np.stack([e1, e2, np.cross(e1, e2)], 1),
                                     la=float(np.linalg.norm(rg.Lrest_t[ib])), lb=float(np.linalg.norm(rg.Lrest_t[ic])),
                                     arm=(a == 'Arm'), ref=self.ix['Spine3'] if a == 'Arm' else self.ix['Hips'])
            k_ = self.const[s + a]               # nearest reachable target distance (flex <= FLEX_MAX)
            k_['dmin'] = math.sqrt(k_['la'] ** 2 + k_['lb'] ** 2 - 2 * k_['la'] * k_['lb'] * math.cos(math.radians(180.0 - self.p['FLEX_MAX'])))
        self.reset()

    def reset(self):
        self.w, self.first, self.dec = 0.0, True, {}
        self.st = {k: dict(phi=0.0, pole=None, delta=0.0, beta=0.0) for k in self.const}
        self.contact, self.steady, self.rel_prev = False, 0, None
        self.gamma = 0.0

    # ---- 1. clamp of the spine / clavicles with the previous-frame branch state
    def _clamp(self, L, t, names):
        rg, k = self.rg, self.p['K_DEG']
        if not names:
            return {}
        js = [self.ix[n] for n in names]
        Rel = np.swapaxes(rg.Lrest[js], -1, -2) @ L[js]
        tw, sy, sz, _ = GL.decompose(Rel)
        moved = {}
        for i, n in enumerate(names):
            cur = np.array([tw[i], sy[i], sz[i]])
            if n in self.dec and not self.first:
                pw, pu = self.dec[n]
                unw = pu + wrap(cur - pw)
            else:
                unw = cur
            self.dec[n] = (cur, unw)
            lim = LIM[n]
            c = [soft1(unw[0], *lim['twist'], k), soft1(unw[1], *lim['swing_y'], k), soft1(unw[2], *lim['swing_z'], k)]
            mv = float(np.max(np.abs(np.array(c) - unw)))
            j = js[i]
            if mv > 1e-6:
                L[j] = rg.Lrest[j] @ GL.compose(np.array(c[0]), np.array(c[1]), np.array(c[2]))
                moved[n] = mv
            dvec = t[j] - rg.Lrest_t[j]
            m = float(np.linalg.norm(dvec)) * 100
            M = lim['trans_dev_cm']
            if m > 1e-6:
                m2 = soft1(m, -1e3, M, M / 2)
                t[j] = rg.Lrest_t[j] + dvec * (m2 / m)
        return moved

    # ---- 3. limb candidates (vectorised over swivel angles)
    def _cands(self, key, S, T, Rt, Wpar, Wref, ref, phis, prev):
        """limb candidates: elbow / knee on its circle at swivel angles phis (deg about the S->T axis, 0 = ref), BOTH hinge signs
        (2 x len(phis)); twists unwrapped against the previous frame (prev = state, None on the first frame of the episode).
        -> rotations, decompositions, limit terms (deg beyond the band), 'lim' cost (hard + pref + reverse bend), 'tjump' cost."""
        c, rg, p = self.const[key], self.rg, self.p
        la, lb = c['la'], c['lb']
        dv = T - S
        dn = float(np.linalg.norm(dv))
        dmin = math.sqrt(la ** 2 + lb ** 2 - 2 * la * lb * math.cos(math.radians(180.0 - p['FLEX_MAX'])))
        d = float(np.clip(dn, dmin, (la + lb) * p['IK_MAX_FRAC']))
        di = dv / max(dn, 1e-9)
        e1 = nrm(ref - di * (ref @ di))
        e2 = np.cross(di, e1)
        phis = np.asarray(phis, float)
        ph = np.radians(np.concatenate([phis, phis]))
        sg = np.concatenate([np.ones(len(phis)), -np.ones(len(phis))])
        poles = np.cos(ph)[:, None] * e1 + np.sin(ph)[:, None] * e2
        cosA = float(np.clip((la ** 2 + d ** 2 - lb ** 2) / (2 * la * d), -1, 1))
        sinA = math.sqrt(max(0.0, 1 - cosA ** 2))
        E = S + la * (cosA * di + sinA * poles)
        u = (E - S) / la
        v = nrm(S + di * d - E)
        f2 = sg[:, None] * nrm(np.cross(di, poles))
        Wa = np.stack([u, f2, np.cross(u, f2)], 2) @ c['Ef'].T[None]
        th = signed((Wa @ rg.Lrest[c['ib']]) @ np.array([c['sb'], 0.0, 0.0]), v, Wa @ c['z'])
        rev = np.degrees(np.maximum(th, 0.0))                 # reverse bend (flex = -th must be >= 0)
        th = np.minimum(th, 0.0)
        Lb = rg.Lrest[c['ib']][None] @ rot_axis(np.tile([0.0, 0.0, 1.0], (len(th), 1)), th)
        Wb = Wa @ Lb
        La = Wpar.T[None] @ Wa
        Lc = np.swapaxes(Wb, -1, -2) @ Rt[None]
        ta, ya, za, _ = GL.decompose(rg.Lrest[c['ia']].T[None] @ La)
        tc, yc, zc, _ = GL.decompose(rg.Lrest[c['ic']].T[None] @ Lc)
        axis = Wa @ c['z']                                    # hinge axis (forearm / shin local Z) in world
        tjump = np.zeros(len(ph))
        if prev is not None:      # continuity of the hinge axis vs the previous frame (a hinge-side flip = 180 deg roll of the upper limb)
            tjump = p['W_AXIS'] * (1.0 - np.clip(axis @ prev, -1.0, 1.0))
        na, nc = self.names_of(key)
        la_, lc_ = LIM[na], LIM[nc]
        if c['arm']:
            Rc0 = rg.rest[self.ix['Spine3'], :3, :3].T @ rg.rest[c['ia'], :3, :3]
            xa = np.array([c['sa'], 0.0, 0.0])
            chest = np.degrees(np.arccos(np.clip(np.sum((Rc0 @ xa)[None] * ((Wref.T[None] @ Wa) @ xa), 1), -1, 1)))
            terms = dict(arm_tw=excess(np.abs(ta), 0, p['ARM_TWIST_HARD']), chest=excess(chest, 0, p['ARM_CHEST_HARD']),
                         hand_tw=excess(np.abs(tc), 0, p['HAND_TWIST_HARD']), hand_sy=excess(yc, *p['HAND_SY_HARD']),
                         hand_sz=excess(zc, *p['HAND_SZ_HARD']), rev=rev)
            pref = p['W_PREF'] * excess(ta, *la_['twist']) ** 2 + \
                p['W_PREF_SWING'] * (excess(ya, *la_['swing_y']) ** 2 + excess(za, *la_['swing_z']) ** 2) + \
                p['W_PREF'] * (excess(yc, *lc_['swing_y_p']) ** 2 + excess(zc, *lc_['swing_z_p']) ** 2)
        else:
            terms = dict(hip_tw=excess(ta, *la_['twist']), hip_sy=excess(ya, *la_['swing_y']), hip_sz=excess(za, *la_['swing_z']),
                         foot_tw=excess(tc, *lc_['twist']), foot_sy=excess(yc, *lc_['swing_y']), foot_sz=excess(zc, *lc_['swing_z']), rev=rev)
            pref = p['W_PREF'] * (excess(ta, *la_['twist_p']) ** 2 + excess(ya, *la_['swing_y_p']) ** 2 + excess(za, *la_['swing_z_p']) ** 2)
        hard = p['W_HARD'] * sum(v_ ** 2 for k_, v_ in terms.items() if k_ != 'rev') + 1e6 * rev ** 2   # reverse bend: forbidden
        return dict(La=La, Lb=Lb, Lc=Lc, ta=ta, ya=ya, za=za, tc=tc, yc=yc, zc=zc, terms=terms, lim=hard + pref, tjump=tjump,
                    phi=wrap(np.degrees(ph)), poles=poles, axis=axis, dn=dn, d=d)

    def names_of(self, key):
        s = 'Left' if key.startswith('Left') else 'Right'
        return (key, s + 'Hand') if key.endswith('Arm') else (key, s + 'Foot')

    def _solve_limb(self, key, L, t, Wr, Wp, T, Rt, Ev, diag):
        """limit-aware swivel (rate limited) + contact-priority upper-arm twist clamp + bounded wrist clamp; writes the chain."""
        c, rg, p, st = self.const[key], self.rg, self.p, self.st[key]
        ia, ib, ic = c['ia'], c['ib'], c['ic']
        S, Wpar, Wref = Wp[ia], Wr[self.par[ia]], Wr[c['ref']]
        di = nrm(T - S)
        pv = Ev - S
        pv = pv - di * (pv @ di)
        pn = float(np.linalg.norm(pv))
        prev_pole = None
        if st['pole'] is not None and not self.first:
            q = Wref @ st['pole']
            q = q - di * (q @ di)
            prev_pole = nrm(q) if np.linalg.norm(q) > 1e-6 else None
        wb = 0.1 + 0.9 * min(1.0, pn / 0.05) ** 2          # intent weight: the clip's elbow plane is ill-defined when its arm is straight
        if pn > 1e-6:
            ref = pv / pn
        elif prev_pole is not None:
            ref = prev_pole
        else:
            x = np.cross(di, Wref[:, 0])
            ref = nrm(x if np.linalg.norm(x) > 1e-6 else np.cross(di, Wref[:, 1]))
        # temporal terms on the ABSOLUTE swivel (previous pole), never relative to the clip plane (it can flip in one frame)
        phi_prev = float(np.degrees(signed(ref, prev_pole, di))) if prev_pole is not None else None
        prev = (Wref @ st['axis']) if (st.get('axis') is not None and not self.first) else None

        def total(cd):
            tt = cd['lim'] + cd['tjump'] + p['W_INTENT'] * wb * cd['phi'] ** 2
            return tt if phi_prev is None else tt + p['W_TEMP'] * wrap(cd['phi'] - phi_prev) ** 2
        N = p['SWIVEL_N']
        cd = self._cands(key, S, T, Rt, Wpar, Wref, ref, -180.0 + np.arange(N) * (360.0 / N), prev)
        k = int(np.argmin(total(cd)))
        cf = self._cands(key, S, T, Rt, Wpar, Wref, ref, cd['phi'][k] + np.linspace(-7.5, 7.5, 7), prev)
        best = float(cf['phi'][int(np.argmin(total(cf)))])
        rate = p['SWIVEL_RATE_DEG']
        phi = best if phi_prev is None else float(wrap(phi_prev + np.clip(wrap(best - phi_prev), -rate, rate)))
        ch = self._cands(key, S, T, Rt, Wpar, Wref, ref, [phi], prev)
        i = int(np.argmin(ch['lim'] + ch['tjump']))            # hinge side at the applied swivel
        st['phi'] = phi
        La, Lb, Lc = ch['La'][i], ch['Lb'][i], ch['Lc'][i]
        ta, ya, za = float(ch['ta'][i]), float(ch['ya'][i]), float(ch['za'][i])
        k5 = p['K_DEG']
        e0 = float(np.linalg.norm(S + (Wpar @ La) @ rg.Lrest_t[ib] + (Wpar @ La @ Lb) @ rg.Lrest_t[ic] - T)) * 100
        twf = 0.0
        if c['arm']:     # upper-arm twist still beyond the hard band: clamped only as far as the hand stays within TOL_USE_CM
            H = p['ARM_TWIST_HARD']
            ta2 = soft1(ta, -H, H, k5)
            if abs(ta2 - ta) > 1e-3:
                tol = max(p['TOL_USE_CM'], e0)

                def trial(f):
                    La_ = rg.Lrest[ia] @ GL.compose(np.array(ta + f * (ta2 - ta)), np.array(ya), np.array(za))
                    Wa_ = Wpar @ La_
                    return La_, float(np.linalg.norm(S + Wa_ @ rg.Lrest_t[ib] + (Wa_ @ Lb) @ rg.Lrest_t[ic] - T)) * 100
                lo_, hi_ = 0.0, 1.0
                if trial(1.0)[1] <= tol:
                    lo_ = 1.0
                else:
                    for _ in range(10):
                        mid = (lo_ + hi_) / 2
                        if trial(mid)[1] <= tol:
                            lo_ = mid
                        else:
                            hi_ = mid
                twf = lo_
                La = trial(lo_)[0]
                ta = ta + lo_ * (ta2 - ta)
                Lc = (Wpar @ La @ Lb).T @ Rt
        Wa = Wpar @ La
        Wb = Wa @ Lb
        e_ = Wa @ rg.Lrest_t[ib]                              # state = the WRITTEN chain (after the twist clamp): pole + hinge axis
        e_ = e_ - di * (e_ @ di)
        if np.linalg.norm(e_) > 1e-6:
            st['pole'] = Wref.T @ nrm(e_)
        st['axis'] = Wref.T @ (Wa @ c['z'])
        tc, yc, zc, _ = GL.decompose((rg.Lrest[ic].T @ Lc)[None])
        tcw = tcu = float(tc[0])
        if c['arm']:
            bt, by, bz = (-p['HAND_TWIST_HARD'], p['HAND_TWIST_HARD']), p['HAND_SY_HARD'], p['HAND_SZ_HARD']
        else:
            nc = self.names_of(key)[1]
            bt, by, bz = LIM[nc]['twist'], LIM[nc]['swing_y'], LIM[nc]['swing_z']
        Lc2 = rg.Lrest[ic] @ GL.compose(np.array(soft1(tcu, *bt, k5)), np.array(soft1(yc[0], *by, k5)), np.array(soft1(zc[0], *bz, k5)))
        err = float(ang_deg(Lc, Lc2))
        if err > p['HAND_ROT_TOL_DEG']:          # contact priority on the hand / foot orientation (weapon, grip, sole)
            Lc2 = slerp_m(Lc[None], Lc2[None], p['HAND_ROT_TOL_DEG'] / err)[0]
        ta_w = float(GL.decompose((rg.Lrest[ia].T @ La)[None])[0][0])
        st['dec'] = dict(ta=(ta_w, ta_w), tc=(tcw, tcw))
        L[ia], L[ib], L[ic] = La, Lb, Lc2
        t[ib], t[ic] = rg.Lrest_t[ib], rg.Lrest_t[ic]
        hp = S + Wa @ rg.Lrest_t[ib] + Wb @ rg.Lrest_t[ic]
        terms = {k_: float(v_[i]) for k_, v_ in ch['terms'].items()}
        wt = max(terms, key=terms.get)
        fa, ya2, za2, _ = GL.decompose((rg.Lrest[ia].T @ La)[None])
        fc, yc2, zc2, _ = GL.decompose((rg.Lrest[ic].T @ Lc2)[None])
        if c['arm']:
            xa = np.array([c['sa'], 0.0, 0.0])
            Rc0 = rg.rest[self.ix['Spine3'], :3, :3].T @ rg.rest[ia, :3, :3]
            chest = float(np.degrees(np.arccos(np.clip(((Wref.T @ Wa) @ xa) @ (Rc0 @ xa), -1, 1))))
            fin = max(float(excess(abs(fa[0]), 0, p['ARM_TWIST_HARD'])), float(excess(chest, 0, p['ARM_CHEST_HARD'])),
                      float(excess(abs(fc[0]), 0, p['HAND_TWIST_HARD'])), float(excess(yc2[0], *p['HAND_SY_HARD'])),
                      float(excess(zc2[0], *p['HAND_SZ_HARD'])))
        else:
            la_, lc_ = LIM[key], LIM[self.names_of(key)[1]]
            fin = max(float(excess(fa[0], *la_['twist'])), float(excess(ya2[0], *la_['swing_y'])), float(excess(za2[0], *la_['swing_z'])),
                      float(excess(fc[0], *lc_['twist'])), float(excess(yc2[0], *lc_['swing_y'])), float(excess(zc2[0], *lc_['swing_z'])))
        diag.update({key + '.phi': phi, key + '.hand_cm': float(np.linalg.norm(hp - T)) * 100, key + '.rot_err': float(ang_deg(Wb @ Lc2, Rt)),
                     key + '.reach': ch['dn'] / (c['la'] + c['lb']), key + '.hard': terms[wt], key + '.twf': twf,
                     key + '.worst_term': wt if terms[wt] > 0 else '', key + '.wrist_clamp': err})
        return fin

    def _reach(self, side, L, t, Wr, Wp, T, diag):
        """2. reach-aware clavicle: minimal turn toward the target inside the clavicle band (rate limited), written into L.
        -> r = the comfortable reach of the rigid arm (m)."""
        p, rg, ix = self.p, self.rg, self.ix
        key = side + 'Arm'
        c, st = self.const[key], self.st[key]
        jc, ja = ix[CLAV[side]], c['ia']
        r = p['REACH_FRAC'] * (c['la'] + c['lb']) + p['TOL_USE_CM'] / 100
        C, Rc = Wp[jc], Wr[jc]
        S = C + Rc @ t[ja]
        d0 = float(np.linalg.norm(T - S))
        diag[key + '.deficit_cm'] = (d0 - r) * 100
        delta_need = 0.0
        if d0 > r:
            o, h = S - C, T - C
            no_, nh = float(np.linalg.norm(o)), float(np.linalg.norm(h))
            th = math.acos(float(np.clip(o @ h / max(no_ * nh, 1e-9), -1, 1)))
            cs = (no_ ** 2 + nh ** 2 - r ** 2) / (2 * no_ * nh)
            cap = math.acos(float(np.clip(cs, -1, 1))) if cs <= 1 else 0.0
            delta_need = max(0.0, th - cap)
        rate = math.radians(p['CLAV_RATE_DEG'])
        delta = delta_need if self.first else float(np.clip(delta_need, st['delta'] - rate, st['delta'] + rate))
        delta = max(0.0, delta)
        st['delta'] = delta
        diag[key + '.delta_deg'] = 0.0
        if delta > 1e-6:
            ax = nrm(np.cross(S - C, T - C))
            Lc_ = Wr[self.par[jc]].T @ (rot_axis(ax, delta) @ Rc)
            tw, sy, sz, _ = GL.decompose((rg.Lrest[jc].T @ Lc_)[None])
            lim, k5 = LIM[CLAV[side]], p['K_DEG']
            prev = self.dec.get(CLAV[side])
            cur = np.array([tw[0], sy[0], sz[0]])
            unw = (prev[1] + wrap(cur - prev[0])) if prev is not None else cur
            L[jc] = rg.Lrest[jc] @ GL.compose(np.array(soft1(unw[0], *lim['twist'], k5)), np.array(soft1(unw[1], *lim['swing_y'], k5)),
                                              np.array(soft1(unw[2], *lim['swing_z'], k5)))
            diag[key + '.delta_deg'] = float(ang_deg(Rc, Wr[self.par[jc]] @ L[jc]))   # turn actually kept after the band re-clamp
        return r

    def _clav_beta(self, jc, Lr, tr, b):
        """clavicle local transform with the fraction b of its correction given back (b = 1: the clip's own clavicle)."""
        if b <= 1e-6:
            return Lr.copy(), tr.copy()
        if b >= 1 - 1e-6:
            return self._Lv[jc].copy(), self._tv[jc].copy()
        return slerp_m(Lr[None], self._Lv[jc][None], b)[0], tr + b * (self._tv[jc] - tr)

    def _arm_excess(self, key, La, Lc, Wa, Wref):
        """hard-band excess (deg) of an arm chain: |upper-arm twist|, upper arm vs chest, hand twist / swing (same bands as the IK)."""
        p, rg, c = self.p, self.rg, self.const[key]
        fa = GL.decompose((rg.Lrest[c['ia']].T @ La)[None])[0]
        fc, yc, zc, _ = GL.decompose((rg.Lrest[c['ic']].T @ Lc)[None])
        xa = np.array([c['sa'], 0.0, 0.0])
        Rc0 = rg.rest[self.ix['Spine3'], :3, :3].T @ rg.rest[c['ia'], :3, :3]
        chest = float(np.degrees(np.arccos(np.clip(((Wref.T @ Wa) @ xa) @ (Rc0 @ xa), -1, 1))))
        return max(float(excess(abs(fa[0]), 0, p['ARM_TWIST_HARD'])), float(excess(chest, 0, p['ARM_CHEST_HARD'])),
                   float(excess(abs(fc[0]), 0, p['HAND_TWIST_HARD'])), float(excess(yc[0], *p['HAND_SY_HARD'])),
                   float(excess(zc[0], *p['HAND_SZ_HARD'])))

    def _arm(self, s, L, t, Wr, Wp, T, Rt, Ev, exv, diag):
        """reach-aware clavicle, then IK -> limit -> IK over beta = the fraction of the clavicle correction given back (contact
        priority): the smallest beta (within the rate) at which the rigid arm reaches the hand target and stays within
        max(LIM_TOL_DEG, the clip's own arm excess exv + LIM_TOL_DEG); else the level with the least excess. beta = 1: the clip's
        clavicle (the guard yields that shoulder). The reach need is never rate limited (contact first); the rest of beta is."""
        p, key = self.p, s + 'Arm'
        c = self.const[key]
        jc, ja = self.ix[CLAV[s]], c['ia']
        jp = self.par[jc]
        r = self._reach(s, L, t, Wr, Wp, T, diag)
        Lr, tr = L[jc].copy(), t[jc].copy()

        def shoulder(b):
            Lb_, tb_ = self._clav_beta(jc, Lr, tr, b)
            return Wp[jp] + Wr[jp] @ (tb_ + Lb_ @ t[ja])
        near = c['dmin'] + p['TOL_USE_CM'] / 100

        def viol(b):                                  # target outside the rigid arm's workspace [dmin + tol, r] (m)
            dd = float(np.linalg.norm(T - shoulder(b)))
            return max(0.0, near - dd) + max(0.0, dd - r)
        breach = 0.0
        if viol(0.0) > 0:
            if viol(1.0) > 0:
                breach = 1.0 if viol(1.0) < viol(0.0) - 0.005 else 0.0
            else:
                lo_, hi_ = 0.0, 1.0
                for _ in range(10):
                    mid = (lo_ + hi_) / 2
                    if viol(mid) > 0:
                        lo_ = mid
                    else:
                        hi_ = mid
                breach = hi_
        st0 = copy.deepcopy(self.st[key])
        bprev, rate = st0['beta'], p['BETA_RATE']
        if self.first:
            bmin, bmax = breach, 1.0
        else:
            bmin = max(breach, bprev - rate)
            bmax = max(bmin, min(1.0, bprev + rate))
        levels = sorted({bmin, bmax} | {b for b in p['BETA_LEVELS'] if bmin < b < bmax})
        tol = max(p['LIM_TOL_DEG'], exv + p['LIM_TOL_DEG'])
        best = None
        for b in levels:
            L2, t2 = L.copy(), t.copy()
            L2[jc], t2[jc] = self._clav_beta(jc, Lr, tr, b)
            Wr2, Wp2 = fk1(self.par, L2, t2)
            self.st[key] = copy.deepcopy(st0)
            d2 = {}
            ex = self._solve_limb(key, L2, t2, Wr2, Wp2, T, Rt, Ev, d2)
            cand = (b, ex, L2, t2, copy.deepcopy(self.st[key]), d2)
            if best is None or ex < best[1] - 0.5:           # least excess; near-ties keep the smaller beta
                best = cand
            if ex <= tol:
                best = cand
                break
        b, ex, L2, t2, stn, d2 = best
        self.st[key] = stn
        self.st[key]['beta'] = b
        L[:], t[:] = L2, t2
        diag.update(d2)
        diag.update({key + '.beta': b, key + '.beta_reach': breach, key + '.kept_cm': float(np.linalg.norm(shoulder(b) - shoulder(0.0))) * 100,
                     key + '.lim_after': ex, key + '.lim_vanilla': exv})

    def full(self, Lv, tv, Wv_r, Wv_p, diag):
        """the full guard (w = 1) of one frame -> L, t."""
        ix, rg = self.ix, self.rg
        self._Lv, self._tv = Lv, tv
        p = self.p
        L0, t0 = Lv.copy(), tv.copy()
        for k_ in self.const.values():           # rigid limb chains: local translations = rest (FPP stretch removed)
            t0[k_['ia']], t0[k_['ib']], t0[k_['ic']] = rg.Lrest_t[k_['ia']], rg.Lrest_t[k_['ib']], rg.Lrest_t[k_['ic']]
        diag['clamped'] = self._clamp(L0, t0, self.spine + list(CLAV.values()))
        # contact of the left hand with the weapon held by the right hand (state: previous relative transform)
        iwr, ilh = ix['WeaponRight'], ix['LeftHand']
        rel_p = Wv_r[iwr].T @ (Wv_p[ilh] - Wv_p[iwr])
        rel_r = Wv_r[iwr].T @ Wv_r[ilh]
        if self.rel_prev is not None and np.linalg.norm(rel_p) < self.p['CONTACT_MAX_M']:
            dm = float(np.linalg.norm(rel_p - self.rel_prev)) * 100
            self.steady = self.steady + 1 if dm < self.p['CONTACT_STEADY_CM'] else (self.steady if dm < self.p['CONTACT_RELEASE_CM'] else 0)
        else:
            self.steady = 0
        self.rel_prev = rel_p
        self.contact = self.steady >= 2
        diag['contact'] = self.contact
        # spine give-back gamma (contact priority over the spine clamp): the smallest gamma at which every hand target (the clip's
        # hand; with the right hand on its target the weapon - and the left-hand contact - is where the clip has it) lies inside the
        # rigid arm's workspace [dmin + tol, reach] from the shoulder with the clip's clavicle (beta = 1). Continuous (bisection on the
        # spine + clavicle FK only). v2 final: NOT applied (diag 'gamma_need' only) - see the comment below.
        js = [ix[n] for n in self.spine]

        def spine_at(gam):
            L_, t_ = L0.copy(), t0.copy()
            if gam > 1e-6 and js:
                L_[js] = slerp_m(L0[js], Lv[js], gam)
                t_[js] = t0[js] + gam * (tv[js] - t0[js])
            return L_, t_

        def shoulders(gam):                       # both shoulders with the clip's clavicles (beta = 1) on the spine at gamma
            L_, t_ = spine_at(gam)
            for s_ in ('Left', 'Right'):
                L_[ix[CLAV[s_]]], t_[ix[CLAV[s_]]] = Lv[ix[CLAV[s_]]], tv[ix[CLAV[s_]]]
            Wp_ = fk1(self.par, L_, t_)[1]
            return np.stack([Wp_[self.const[s_ + 'Arm']['ia']] for s_ in ('Left', 'Right')])

        def viol_g(gam):
            Sh, v = shoulders(gam), 0.0
            for i_, s_ in enumerate(('Left', 'Right')):
                k_ = self.const[s_ + 'Arm']
                dd = float(np.linalg.norm(Wv_p[k_['ic']] - Sh[i_]))
                r_ = p['REACH_FRAC'] * (k_['la'] + k_['lb']) + p['TOL_USE_CM'] / 100
                v += max(0.0, k_['dmin'] + p['TOL_USE_CM'] / 100 - dd) + max(0.0, dd - r_)
            return v
        gam = 0.0
        if js:
            v0 = viol_g(0.0)
            if v0 > 1e-4:
                v1 = viol_g(1.0)
                if v1 <= 1e-4:
                    lo_, hi_ = 0.0, 1.0
                    for _ in range(10):
                        mid = (lo_ + hi_) / 2
                        if viol_g(mid) > 1e-4:
                            lo_ = mid
                        else:
                            hi_ = mid
                    gam = hi_
                elif v1 < v0 - 0.005:
                    gam = 1.0
        diag['gamma_need'] = gam      # DIAGNOSTIC ONLY: tested as an applied give-back and rejected (see the report): a 0.1 step
        gam = 0.0                     # moved the shoulders ~30 cm (the FPP torso twist comes back); rate limited -> the hands miss
        self.gamma = gam
        diag['gamma'] = gam
        L, t = spine_at(gam)
        Wr, Wp = fk1(self.par, L, t)
        for s, a, b, c in self.limbs:
            key = s + a
            k_ = self.const[key]
            if k_['arm']:
                if s == 'Left' and self.contact:
                    T, Rt = Wr[iwr] @ rel_p + Wp[iwr], Wr[iwr] @ rel_r
                else:
                    T, Rt = Wv_p[k_['ic']], Wv_r[k_['ic']]
                exv = self._arm_excess(key, Lv[k_['ia']], Lv[k_['ic']], Wv_r[k_['ia']], Wv_r[ix['Spine3']])
                self._arm(s, L, t, Wr, Wp, T, Rt, Wv_p[k_['ib']], exv, diag)
            else:
                T, Rt = Wv_p[k_['ic']], Wv_r[k_['ic']]
                diag[key + '.lim_after'] = self._solve_limb(key, L, t, Wr, Wp, T, Rt, Wv_p[k_['ib']], diag)
            Wr, Wp = fk1(self.par, L, t)
            diag[key + '.target'] = (T, Rt)
        return L, t

    def _contact_fix(self, key, L, t, T, Rt):
        """minimal-change two-bone contact fix of a blended chain: elbow about its hinge to the target distance, upper arm about the
        shoulder onto the target, end-effector world rotation = Rt."""
        c, rg = self.const[key], self.rg
        ia, ib, ic = c['ia'], c['ib'], c['ic']
        Wr, Wp = fk1(self.par, L, t)
        S, E, H = Wp[ia], Wp[ib], Wp[ic]
        la, lb = float(np.linalg.norm(E - S)), float(np.linalg.norm(H - E))
        dst = float(np.clip(np.linalg.norm(T - S), abs(la - lb) + 1e-4, (la + lb) * self.p['IK_MAX_FRAC']))
        nw = nrm(Wr[ib][:, 2])                    # the forearm's own hinge axis (local Z) in world
        q, s_ = H - E, S - E
        qp = nw * (q @ nw)
        qo = q - qp
        A, B, C_ = s_ @ qp, s_ @ qo, s_ @ np.cross(nw, qo)
        kap = (s_ @ s_ + q @ q - dst ** 2) / 2
        rr = math.hypot(B, C_)
        if rr > 1e-9:
            base = math.atan2(C_, B)
            off = math.acos(float(np.clip((kap - A) / rr, -1, 1)))
            xs = [base + off, base - off]
            x = min(xs, key=lambda v: abs(wrap(math.degrees(v))))
            Q = rot_axis(nw, x)
            L[ib] = Wr[ia].T @ (Q @ Wr[ib])
        Wr, Wp = fk1(self.par, L, t)
        Q2 = rmin1(Wp[ic] - Wp[ia], T - Wp[ia])
        L[ia] = Wr[self.par[ia]].T @ (Q2 @ Wr[ia])
        Wr, Wp = fk1(self.par, L, t)
        L[ic] = Wr[ib].T @ Rt

    def step(self, Lv, tv, Wv, target):
        """one frame: vanilla locals Lv [73,3,3], tv [73,3], vanilla world Wv [73,4,4], target weight 0 / 1 -> (L, t, diag)."""
        diag = {}
        Wv_r, Wv_p = Wv[:, :3, :3], Wv[:, :3, 3]
        G, Gt = self.full(Lv, tv, Wv_r, Wv_p, diag)
        js = self.joints
        corr = float(np.max(ang_deg(Lv[js], G[js]))) if js else 0.0
        corr = max(corr, float(np.max(np.linalg.norm(Gt[js] - tv[js], axis=1))) * 100 * self.p['TRANS_DEG_PER_CM'])
        if getattr(self, 'force_w', None) is not None:
            self.w = self.force_w
        elif self.first:
            self.w = 0.0
        else:
            dw = float(np.clip(self.p['STEP_CAP_DEG'] / max(corr, 1e-6), DT / self.p['T_MAX'], DT / self.p['T_MIN']))
            self.w = min(target, self.w + dw) if target > self.w else max(target, self.w - dw)
        w = self.w
        diag['w'], diag['corr'] = w, corr
        self.first = False
        if w >= 1 - 1e-9:
            return G, Gt, diag
        if w <= 1e-9:
            return Lv.copy(), tv.copy(), diag
        L, t = Lv.copy(), tv.copy()
        L[js] = slerp_m(Lv[js], G[js], w)
        t[js] = tv[js] + w * (Gt[js] - tv[js])
        Wg_r, _ = fk1(self.par, G, Gt)
        for s, a, b, c in self.limbs:
            key = s + a
            k_ = self.const[key]
            T, Rt_g = diag[key + '.target']
            Rt_w = slerp_m(Wv_r[k_['ic']][None], Wg_r[k_['ic']][None], w)[0]
            if k_['arm'] and s == 'Left' and self.contact:
                Wr, Wp = fk1(self.par, L, t)
                iwr = self.ix['WeaponRight']
                rel_p = Wv_r[iwr].T @ (Wv_p[self.ix['LeftHand']] - Wv_p[iwr])
                T = Wr[iwr] @ rel_p + Wp[iwr]
            self._contact_fix(key, L, t, T, Rt_w)
        return L, t, diag


def run_clip(W0, ctx, mode, legs, p=None, ramp=True):
    """the clip as one episode, state entry at frame 0 (parameters initialised there).
    ramp=True  (TRANSITION run): w ramps 0 -> 1 from frame 0 and 1 -> 0 after the end (post-roll: last clip frame held);
    ramp=False (STEADY run): w = 1 on every frame = the state was entered before the clip (deformation / hand / limit metrics).
    -> W2 [T (+ n_post), 73, 4, 4] guarded world, diag list per frame."""
    g = GuardV2(ctx, mode, legs, p)
    g.force_w = None if ramp else 1.0
    Lv, tv = GL.locals_of(W0, ctx.rg.par)
    T = len(W0)
    n_post = g.p['N_POST'] if ramp else 0
    out, dg = [], []
    for i in range(T + n_post):
        f = min(i, T - 1)
        L, t, d = g.step(Lv[f], tv[f], W0[f], 1.0 if i < T else 0.0)
        Wr, Wp = fk1(g.par, L, t)
        out.append(to_W(Wr, Wp))
        d.pop('clamped', None)
        for k in list(d):
            if k.endswith('.target'):
                d.pop(k)
        dg.append(d)
    return np.stack(out), dg


# ------------------------------------------------------------------------------------------------ probe (no skin)
def pops(W0, Wg, ctx, entry_exit_v1=False, gm=None):
    """per-joint pop (deg): max over frames and BODY_POP joints of (guard local step - vanilla local step). Wg may carry post-roll
    frames (vanilla = last clip frame held, the guard ramps out). entry_exit_v1 (no ramp): + the jump to the guard at frame 0 and
    back to the clip after the last frame (the full correction). v2: entry = guard vs clip at frame 0 (0 when w starts at 0)."""
    par = ctx.rg.par
    Lg = GL.locals_of(Wg, par)[0]
    L0 = GL.locals_of(W0, par)[0]
    T = len(W0)
    js = [ctx.ix[n] for n in BODY_POP]
    L0x = np.concatenate([L0, np.repeat(L0[-1:], len(Wg) - T, 0)]) if len(Wg) > T else L0
    d = ang_deg(Lg[:-1, js], Lg[1:, js]) - ang_deg(L0x[:-1, js], L0x[1:, js])
    ok = np.ones(len(d), bool)                 # steps touching a glitch frame of the clip (survey glitch_mask) are not counted
    if gm is not None and np.any(gm):
        g_ = np.zeros(len(Wg), bool)
        g_[:T] = gm
        ok = ~(g_[:-1] | g_[1:])
        d = np.where(ok[:, None], d, -1e9)
    pin = float(max(d[:T - 1].max(), 0.0)) if T > 1 else 0.0
    fi, ji = np.unravel_index(d[:T - 1].argmax(), d[:T - 1].shape) if T > 1 else (0, 0)
    entry = float(ang_deg(L0[0, js], Lg[0, js]).max())
    if entry_exit_v1:
        exit_ = float(ang_deg(Lg[T - 1, js], L0[T - 1, js]).max())
    else:
        exit_ = float(max(d[T - 1:].max(), 0.0)) if len(Wg) > T else 0.0
    # chain pop (secondary): per region, the largest guard step minus the largest clip step of the SAME region and frame -> a step
    # the clip already makes in another joint of the chain (FPP clavicle jump moved into the elbow) is not counted again
    sg, s0 = ang_deg(Lg[:-1, js], Lg[1:, js]), ang_deg(L0x[:-1, js], L0x[1:, js])
    cp, cat = 0.0, ''
    for rn, members in REGIONS.items():
        ri = [BODY_POP.index(n) for n in members]
        dr = np.where(ok, sg[:, ri].max(1) - s0[:, ri].max(1), -1e9)
        if len(dr) and float(dr.max()) > cp:
            cp, cat = float(dr.max()), '%s@%d' % (rn, int(dr.argmax()) + 1)
    return dict(pop=round(max(pin, entry, exit_), 1), pop_in=round(pin, 1), pop_at='%s@%d' % (BODY_POP[ji], fi + 1), entry=round(entry, 1),
                exit=round(exit_, 1), pop_chain=round(cp, 1), pop_chain_at=cat)


def arm_limits(W, ctx):
    """per frame max excess (deg) of the arm chain beyond the v2 HARD bands: |upper-arm twist| 105, upper arm vs chest 160,
    |hand twist| 130, hand swing_y [-80, 75], swing_z [-35, 35]; forearm hinge: twist / swing_y beyond the table band, flex outside
    [-5, 150]. -> (excess [T], worst name [T])."""
    rg, ix = ctx.rg, ctx.ix
    L = GL.locals_of(W, rg.par)[0]
    Wr = W[:, :, :3, :3]
    cols, names = [], []
    for s in ('Left', 'Right'):
        a, b, c = ix[s + 'Arm'], ix[s + 'ForeArm'], ix[s + 'Hand']
        ta, ya, za, _ = GL.decompose(rg.Lrest[a].T[None] @ L[:, a])
        tb, yb, zb, _ = GL.decompose(rg.Lrest[b].T[None] @ L[:, b])
        tc, yc, zc, _ = GL.decompose(rg.Lrest[c].T[None] @ L[:, c])
        xa = np.array([np.sign(rg.Lrest_t[b][0]) or 1.0, 0.0, 0.0])
        Rc0 = rg.rest[ix['Spine3'], :3, :3].T @ rg.rest[a, :3, :3]
        chest = np.degrees(np.arccos(np.clip(((np.swapaxes(Wr[:, ix['Spine3']], -1, -2) @ Wr[:, a]) @ xa) @ (Rc0 @ xa), -1, 1)))
        fl = LIM[s + 'ForeArm']
        for n, e in (('arm_tw', excess(np.abs(ta), 0, P['ARM_TWIST_HARD'])), ('chest', excess(chest, 0, P['ARM_CHEST_HARD'])),
                     ('hand_tw', excess(np.abs(tc), 0, P['HAND_TWIST_HARD'])), ('hand_sy', excess(yc, *P['HAND_SY_HARD'])),
                     ('hand_sz', excess(zc, *P['HAND_SZ_HARD'])), ('fore_tw', excess(tb, *fl['twist'])),
                     ('fore_sy', excess(yb, *fl['swing_y'])), ('flex', excess(-zb, -5.0, P['FLEX_MAX']))):
            cols.append(e)
            names.append(s[0] + '.' + n)
    M = np.stack(cols, 1)
    return M.max(1), [names[i] for i in M.argmax(1)]


def hand_errors(W0, Wg, ctx, contact):
    """hand position (cm) / rotation (deg) error per frame (max of both hands) vs the contact targets: the clip's hand world transform,
    LEFT hand on frames in weapon contact: guarded WeaponRight * the clip's LeftHand-in-WeaponRight (same rule for v1 and v2)."""
    ix = ctx.ix
    T = len(W0)
    Wg = Wg[:T]
    ep, er = [], []
    for s in ('Left', 'Right'):
        h = ix[s + 'Hand']
        Tg = W0[:, h].copy()
        if s == 'Left' and np.any(contact):
            iwr = ix['WeaponRight']
            tgt = Wg[:, iwr] @ (np.linalg.inv(W0[:, iwr]) @ W0[:, h])
            Tg[contact] = tgt[contact]
        ep.append(np.linalg.norm(Wg[:, h, :3, 3] - Tg[:, :3, 3], axis=1) * 100)
        er.append(ang_deg(Tg[:, :3, :3], Wg[:, h, :3, :3]))
    return np.max(ep, 0), np.max(er, 0)


def clip_metrics(W0, Wg, ctx, m, gm, contact, v1=False):
    """the v1 survey metrics + v2 additions for one guarded clip (Wg may carry post-roll frames)."""
    T = len(W0)
    f, e = ctx.measure(Wg[:T], legs=m['legs'])
    s = SM.summarize(e, T, gm)
    keep = ~gm if gm.any() and not gm.all() else np.ones(T, bool)
    lim, lname = arm_limits(Wg[:T], ctx)
    hp, hr = hand_errors(W0, Wg, ctx, contact)
    armg = [k for k in ('uparm_twist_L', 'uparm_twist_R', 'pronation_L', 'pronation_R', 'wrist_flex_L', 'wrist_flex_R', 'wrist_dev_L',
                        'wrist_dev_R', 'elbow_flex_L', 'elbow_flex_R', 'forearm_roll_L', 'forearm_roll_R', 'uparm_swing_L', 'uparm_swing_R') if k in e]
    ga = np.max(np.stack([e[k] for k in armg], 1), 1)
    lim0, _ = arm_limits(W0, ctx)
    newx = (lim > lim0 + P['LIM_TOL_DEG']) & keep
    r = dict(fail=s['fail_pct'], worst=[s['worst_feature'], s['worst_excess']], severity=s['severity'], fail_features=s['fail_features'],
             trans=s['trans_excess'], lim_max=round(float(lim[keep].max()), 1), lim_at=lname[int(np.argmax(np.where(keep, lim, -1)))],
             lim_frames_pct=round(100.0 * float((lim[keep] > 5.0).mean()), 1), gate_arm_max=round(float(ga[keep].max()), 1),
             new_excess=int(newx.sum()), new_excess_max=round(float(np.max(np.where(keep, lim - lim0, 0.0))), 1),
             hand_cm=round(float(hp.max()), 2), hand_cm_p95=round(float(np.percentile(hp, 95)), 2), hand_at=int(np.argmax(hp)),
             hand_rot=round(float(hr.max()), 1), hand_rot_p95=round(float(np.percentile(hr, 95)), 1),
             hand_rot_cap_pct=round(100.0 * float((hr >= P['HAND_ROT_TOL_DEG'] - 0.1).mean()), 1))
    r.update(pops(W0, Wg, ctx, entry_exit_v1=v1, gm=gm))
    return r


def probe(keys):
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    ctx = SM.Ctx()
    os.makedirs(OUT, exist_ok=True)
    out_p = os.path.join(OUT, 'probe_%s.json' % RIG)
    res = json.load(open(out_p, encoding='utf-8')) if os.path.exists(out_p) else {}
    for k in keys:
        m = met[k]
        W0 = GP.load_shown(ctx, k, m)
        T = len(W0)
        gm = SM.glitch_mask(W0, ctx.par)
        W2x, dg = run_clip(W0, ctx, m['mode'], m['mode'] == 'full' and m['legs'])
        contact = np.array([bool(d.get('contact')) for d in dg[:T]])
        _, W1, _ = GP.guard(W0, ctx, feet=m['cls'] in ('A', 'E') and m['legs'] and m['mode'] == 'full')
        f0, e0 = ctx.measure(W0, legs=m['legs'])
        s0 = SM.summarize(e0, T, gm)
        lim0, _ = arm_limits(W0, ctx)
        r = dict(cls=m['cls'], state=m['state'], mode=m['mode'], frames=T, fail_v=s0['fail_pct'], lim_v=round(float(lim0.max()), 1),
                 v1=clip_metrics(W0, W1, ctx, m, gm, contact, v1=True), v2=clip_metrics(W0, W2x, ctx, m, gm, contact),
                 contact_frames=int(contact.sum()))
        for s in ('Left', 'Right'):
            key = s + 'Arm'
            col = lambda n: np.array([float(d.get(key + '.' + n, 0.0)) for d in dg[:T]])  # noqa: E731
            wt = [d.get(key + '.worst_term', '') for d in dg[:T]]
            r[s] = dict(deficit_max_cm=round(float(col('deficit_cm').max()), 1), deficit_pct=round(100 * float((col('deficit_cm') > 0).mean()), 1),
                        delta_max=round(float(col('delta_deg').max()), 1), beta_max=round(float(col('beta').max()), 2),
                        beta_pct=round(100 * float((col('beta') > 0.02).mean()), 1), kept_max_cm=round(float(col('kept_cm').max()), 1),
                        hard_max=round(float(col('hard').max()), 1), wrist_clamp_max=round(float(col('wrist_clamp').max()), 1),
                        twf_frames=int((col('twf') > 0).sum()), phi_absmax=round(float(np.abs(col('phi')).max()), 1),
                        worst_terms={x: wt.count(x) for x in set(wt) if x})
        res[k] = r
        SC.jdump(res, out_p)
        a, b = r['v1'], r['v2']
        print('PROBE %-56s %s %-6s fail %5.1f | v1 %5.1f hand %5.2f rot %5.1f lim %5.1f pop %5.1f (%s e%.0f x%.0f) | v2 %5.1f hand %5.2f rot %5.1f '
              'lim %5.1f (v %5.1f) pop %5.1f (%s x%.0f) | L def %.1f beta %.2f/%.0f%% kept %.1f | R def %.1f beta %.2f/%.0f%% kept %.1f | %s %s' % (
                  k[:56], m['cls'], m['mode'], r['fail_v'], a['fail'], a['hand_cm'], a['hand_rot'], a['lim_max'], a['pop'], a['pop_at'], a['entry'],
                  a['exit'], b['fail'], b['hand_cm'], b['hand_rot'], b['lim_max'], r['lim_v'], b['pop'], b['pop_at'], b['exit'],
                  r['Left']['deficit_max_cm'], r['Left']['beta_max'], r['Left']['beta_pct'], r['Left']['kept_max_cm'], r['Right']['deficit_max_cm'],
                  r['Right']['beta_max'], r['Right']['beta_pct'], r['Right']['kept_max_cm'], b['worst'], b['lim_at']), flush=True)
    print('WROTE', out_p)


if __name__ == '__main__':
    a = ARGS[1:]
    if '--probe' in a:
        ks = [x for x in a[a.index('--probe') + 1:] if not x.startswith('--') and x not in ('woman', 'man')]
        if not ks or ks == ['v1']:
            ks = list(json.load(open(os.path.join(SC.WORK, 'proto_%s.json' % RIG), encoding='utf-8')))
        probe(ks)
