"""Hard anatomical gate for corrected melee clips (Female / Male V rig, 73-joint dump layout).

  python -I anatomy_gate.py <corrected.npz> --set <set> --clip <clip> [--keys <clip>.keys.json] [--verbose]
  python -I anatomy_gate.py --selftest
A corrected clip passes only when EVERY per-frame feature stays inside
      allowed(t) = [ min(human_lo, vanilla_lo(t) - tol) ,  max(human_hi, vanilla_hi(t) + tol) ]
where vanilla(t) is the ORIGINAL game clip at the source frame that output frame t comes from (time map of keys.json, +-WIN frames).
So the gate is never stricter than what the vanilla clip itself already does at that frame, and never looser than human range + tol
where the vanilla is inside it. When in doubt the correction is rejected (apply_keyposes scales it down; melee_keyposes falls back).

Features (degrees unless _cm / _pct; all computed from the 73 world matrices, rest-relative, swing-twist / Euler ZYX about the rig's
own axes - arms: local X = bone axis, Z = hinge axis of elbow / wrist, spine + neck: X = up):
  elbow_flex_L/R      -rz of the forearm (ZYX Euler, the hinge)           [-5, 150]    reverse bend / hyper-extension / over-flex
  elbow_dev_L/R       |ry| of the forearm: elbow is a hinge               [0, 15]
  forearm_roll_L/R    |rx| of the forearm                                 [0, 110]     candy-wrap twist of the forearm
  wrist_flex_L/R      rz of the hand                                      [-80, 75]
  wrist_dev_L/R       |ry| of the hand (radial / ulnar deviation)         [0, 35]
  wrist_twist_L/R     |rx| of the hand relative to the forearm            [0, 35]
  pronation_L/R       |forearm roll + wrist twist|                        [0, 130]
  uparm_twist_L/R     |twist| of the upper arm about its axis (vs clavicle) [0, 105]
  uparm_swing_L/R     swing of the upper arm relative to the chest        [0, 165]
  clav_swing_L/R      clavicle elevation / protraction (vs chest)         [0, 38]
  clav_twist_L/R      clavicle roll                                       [0, 25]
  clav_shift_L/R_cm   clavicle joint translation vs the reference pose    [0, 3]
  spine_twist_b       |twist| of each spine bone (Spine, Spine1, Spine2, Spine3)   [0, 22]
  spine_bend_b        swing of each spine bone                            [0, 25]
  spine_twist_total   |twist| Spine3 vs Hips                              [0, 65]
  spine_bend_total    swing Spine3 vs Hips                                [0, 60]
  neck_twist/_bend    Head vs Spine3                                      [0, 75] / [0, 55]
  knee_flex_L/R       -rz of the leg                                      [-5, 10]     stance cap (anatomical 150): the static FPP legs are straight, a hips drop buckles them
  knee_dev_L/R, shin_roll_L/R   hinge violations                          [0, 12] / [0, 25]
  hip_flex_L/R        swing of the thigh vs hips                          [0, 125]
  hip_twist_L/R       twist of the thigh                                  [0, 50]
  elbow_torso_cm_L/R  distance elbow joint - torso axis (Hips..Neck)      [8, inf)     arm through the torso
  hand_torso_cm_L/R   same for the hand joint                             [6, inf)
  seg_len_dev_pct     max |bone length / reference length - 1| of the limb / spine / neck bones   [0, 2.5]   stretch
  neck_len_dev_pct    Neck1..Head and Spine3..Neck lengths                (part of seg_len_dev_pct)
  foot_dev_cm_L/R     foot position vs the vanilla foot at the mapped frame  [0, 3]   (vanilla lower body is static in these clips)
  rot_step            max one-frame local rotation change of any body bone   [0, 18]
  hips_drop_cm        hips height above the feet vs reference             [-3, 3]
  ankle / torso_lean_deg  Hips -> Neck from vertical                          [0, 25]
  upperarm_torso_cm / forearm_torso_cm   nearest point of the segment to the torso axis  [8, inf)
  hand_cross_cm / elbow_cross_cm / elbow_back_cm   hand / elbow past the midline, elbow behind the shoulder (CHEST frame)  [20 / 12 / 15]
  hand_eye_cm, shoulder_width_cm   [25, inf) / [20, inf)
  delta_* BUDGETS against the vanilla clip at the same (re-timed) frame, lifted x3 where the vanilla pose itself is broken (FAULT_FEATURES):
    delta_arm_deg 28, delta_leg_deg 35, delta_chest_deg 28, delta_pitch_deg 8, delta_lean_deg 6, delta_hips_cm 2, delta_hand_chest_cm 15, delta_grip_cm 5 (two-hand grips)
  skin_check(): separate, mesh based (edge stretch / squash of the skinned V in shoulder / arm / wrist / hip / knee / neck regions vs vanilla, <= 0.35 log2)
Tolerances: angles TOL_DEG, distances TOL_CM, lengths TOL_PCT, steps scale 1.6x + 6.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

TOL_DEG, TOL_CM, TOL_PCT = 6.0, 1.5, 1.0
BUDGET_FAULT_MULT = 3.0
FAULT_FEATURES = ['hand_torso_cm_L', 'hand_torso_cm_R', 'forearm_torso_cm_L', 'forearm_torso_cm_R', 'upperarm_torso_cm_L', 'upperarm_torso_cm_R',
                  'elbow_torso_cm_L', 'elbow_torso_cm_R', 'hand_eye_cm']
WIN = 3                    # vanilla reference window (source frames either side)
BODY = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm',
        'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot']
SEGS = [('LeftArm', 'LeftForeArm'), ('LeftForeArm', 'LeftHand'), ('RightArm', 'RightForeArm'), ('RightForeArm', 'RightHand'),
        ('LeftUpLeg', 'LeftLeg'), ('LeftLeg', 'LeftFoot'), ('RightUpLeg', 'RightLeg'), ('RightLeg', 'RightFoot'),
        ('Hips', 'Spine'), ('Spine', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Spine3'), ('Spine3', 'Neck'), ('Neck', 'Neck1'),
        ('Neck1', 'Head'), ('LeftShoulder', 'LeftArm'), ('RightShoulder', 'RightArm')]

# name -> (human lo, human hi, kind); kind 'ang' | 'cm' | 'pct' | 'step'
LIM = {}


def _lim(names, lo, hi, kind):
    for n in names:
        LIM[n] = (lo, hi, kind)


_lim(['elbow_flex_L', 'elbow_flex_R'], -5.0, 150.0, 'ang')
_lim(['elbow_dev_L', 'elbow_dev_R'], 0.0, 15.0, 'ang')
_lim(['forearm_roll_L', 'forearm_roll_R'], 0.0, 110.0, 'ang')
_lim(['wrist_flex_L', 'wrist_flex_R'], -80.0, 75.0, 'ang')
_lim(['wrist_dev_L', 'wrist_dev_R'], 0.0, 35.0, 'ang')
_lim(['wrist_twist_L', 'wrist_twist_R'], 0.0, 35.0, 'ang')
_lim(['pronation_L', 'pronation_R'], 0.0, 130.0, 'ang')
_lim(['uparm_twist_L', 'uparm_twist_R'], 0.0, 105.0, 'ang')
_lim(['uparm_swing_L', 'uparm_swing_R'], 0.0, 165.0, 'ang')
_lim(['clav_swing_L', 'clav_swing_R'], 0.0, 38.0, 'ang')
_lim(['clav_twist_L', 'clav_twist_R'], 0.0, 25.0, 'ang')
_lim(['clav_shift_L_cm', 'clav_shift_R_cm'], 0.0, 3.0, 'cm')
_lim(['spine_twist_%s' % b for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')], 0.0, 22.0, 'ang')
_lim(['spine_bend_%s' % b for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')], 0.0, 25.0, 'ang')
_lim(['spine_twist_total'], 0.0, 65.0, 'ang')
_lim(['spine_bend_total'], 0.0, 60.0, 'ang')
_lim(['neck_twist'], 0.0, 75.0, 'ang')
_lim(['neck_bend'], 0.0, 55.0, 'ang')
_lim(['knee_flex_L', 'knee_flex_R'], -5.0, 10.0, 'ang')         # stance cap (anatomical max 150): deeper knee bends only when vanilla has them (the static FPP legs are straight: a hips drop with fixed feet buckles the knees)
_lim(['knee_dev_L', 'knee_dev_R'], 0.0, 12.0, 'ang')
_lim(['shin_roll_L', 'shin_roll_R'], 0.0, 25.0, 'ang')
_lim(['hip_flex_L', 'hip_flex_R'], 0.0, 125.0, 'ang')
_lim(['hip_twist_L', 'hip_twist_R'], 0.0, 50.0, 'ang')
_lim(['elbow_torso_cm_L', 'elbow_torso_cm_R'], 8.0, 1e9, 'cm')
_lim(['hand_torso_cm_L', 'hand_torso_cm_R'], 6.0, 1e9, 'cm')
_lim(['seg_len_dev_pct'], 0.0, 2.5, 'pct')
_lim(['foot_dev_cm_L', 'foot_dev_cm_R'], 0.0, 3.0, 'cm')
_lim(['rot_step'], 0.0, 18.0, 'step')
_lim(['hips_drop_cm'], -3.0, 3.0, 'cm')                            # quality cap: hunch = hips dropped on (near) straight reference legs
_lim(['torso_lean_deg'], 0.0, 25.0, 'ang')                         # Hips -> Neck vs vertical
_lim(['upperarm_torso_cm_L', 'upperarm_torso_cm_R', 'forearm_torso_cm_L', 'forearm_torso_cm_R'], 8.0, 1e9, 'cm')   # nearest point of the segment to the torso axis
_lim(['delta_arm_deg_L', 'delta_arm_deg_R'], 0.0, 28.0, 'ang')    # budget: any arm bone's local rotation vs the vanilla clip at the mapped frame
_lim(['delta_leg_deg_L', 'delta_leg_deg_R'], 0.0, 35.0, 'ang')
_lim(['delta_chest_deg'], 0.0, 28.0, 'ang')                        # world rotation of Spine3 vs vanilla
_lim(['delta_hips_cm'], 0.0, 2.0, 'cm')
_lim(['delta_pitch_deg'], 0.0, 8.0, 'ang')                         # chest pitch (rotation about the character's right axis) vs vanilla, forward or back: no hunch / sway-back
_lim(['delta_lean_deg'], 0.0, 6.0, 'ang')                          # torso pitch / lean (Hips -> Neck) vs vanilla: no hunch
_lim(['delta_grip_cm'], 0.0, 5.0, 'cm')                           # two-hand grip (vanilla hands < 40 cm apart): the hand separation may not change
_lim(['delta_hand_chest_cm_L', 'delta_hand_chest_cm_R'], 0.0, 15.0, 'cm')   # hand position (chest frame) vs vanilla
_lim(['hand_cross_cm_L', 'hand_cross_cm_R'], -1e9, 20.0, 'cm')     # hand past the body midline (chest frame)
_lim(['elbow_cross_cm_L', 'elbow_cross_cm_R'], -1e9, 12.0, 'cm')
_lim(['elbow_back_cm_L', 'elbow_back_cm_R'], -1e9, 15.0, 'cm')    # elbow behind its shoulder joint (chest frame)
_lim(['hand_eye_cm'], 25.0, 1e9, 'cm')
_lim(['shoulder_width_cm'], 20.0, 1e9, 'cm')                       # upper-arm joints along the hips' right axis (rest 36)


# ------------------------------------------------------------------ rotation helpers
def _local(W, parents):
    Rw = W[..., :3, :3]
    L = Rw.copy()
    for j, p in enumerate(parents):
        if p >= 0:
            L[:, j] = np.swapaxes(Rw[:, p], -1, -2) @ Rw[:, j]
    return L


def _local_rest(rest, parents):
    Rr = rest[:, :3, :3]
    L = Rr.copy()
    for j, p in enumerate(parents):
        if p >= 0:
            L[j] = Rr[p].T @ Rr[j]
    return L


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def swing_twist(Rm, axis=0):
    """Rm [...,3,3] -> (swing deg [...], twist deg signed [...]) about local axis (0 = X)."""
    sh = Rm.shape[:-2]
    q = R.from_matrix(Rm.reshape(-1, 3, 3)).as_quat()      # xyzw
    tw = np.degrees(2.0 * np.arctan2(q[:, axis], q[:, 3]))
    tw = _wrap(tw)
    qt = np.zeros_like(q)
    qt[:, axis] = q[:, axis]
    qt[:, 3] = q[:, 3]
    n = np.linalg.norm(qt, axis=1, keepdims=True)
    qt = np.where(n > 1e-9, qt / np.maximum(n, 1e-9), np.array([0, 0, 0, 1.0]))
    sw = (R.from_quat(qt).inv() * R.from_quat(q))
    return np.degrees(sw.magnitude()).reshape(sh), tw.reshape(sh)


def euler_zyx(Rm):
    """Rm [...,3,3] = Rz(a) Ry(b) Rx(c) -> (a, b, c) deg arrays."""
    sh = Rm.shape[:-2]
    e = R.from_matrix(Rm.reshape(-1, 3, 3)).as_euler('ZYX', degrees=True)
    return e[:, 0].reshape(sh), e[:, 1].reshape(sh), e[:, 2].reshape(sh)


# ------------------------------------------------------------------ features
class Rig:
    def __init__(self, names, parents, rest):
        self.names = [str(n) for n in names]
        self.ix = {n: i for i, n in enumerate(self.names)}
        self.par = [int(p) for p in parents]
        self.rest = np.asarray(rest, float)
        self.Lrest = _local_rest(self.rest, self.par)
        P = self.rest[:, :3, 3]
        self.seg_rest = [float(np.linalg.norm(P[self.ix[a]] - P[self.ix[b]])) for a, b in SEGS]
        self.Lrest_t = np.stack([self.rest[self.par[j], :3, :3].T @ (P[j] - P[self.par[j]]) if self.par[j] >= 0 else P[j]
                                 for j in range(len(self.names))])


def rig_of(d):
    return Rig(d['names'], d['parents'], d['rest'])


def features(W, rg, step_frames=True):
    """W [T,73,4,4] -> dict feature name -> [T] (rest-relative, degrees / cm / pct)."""
    W = np.asarray(W, float)
    T = len(W)
    ix, f = rg.ix, {}
    L = _local(W, rg.par)
    Rel = np.swapaxes(rg.Lrest[None], -1, -2) @ L                  # rest-relative local rotation (rotation in the bone's own frame)
    P = W[:, :, :3, 3]
    for s, S in (('L', 'Left'), ('R', 'Right')):
        z, y, x = euler_zyx(Rel[:, ix[S + 'ForeArm']])
        f['elbow_flex_' + s], f['elbow_dev_' + s], f['forearm_roll_' + s] = -z, np.abs(y), np.abs(x)
        roll = x
        zh, yh, xh = euler_zyx(Rel[:, ix[S + 'Hand']])
        f['wrist_flex_' + s], f['wrist_dev_' + s], f['wrist_twist_' + s] = zh, np.abs(yh), np.abs(xh)
        f['pronation_' + s] = np.abs(roll + xh)
        sw, tw = swing_twist(Rel[:, ix[S + 'Arm']])
        f['uparm_twist_' + s] = np.abs(tw)
        # swing of the upper arm relative to the chest (clavicle included): rest-relative rotation of the arm in the chest frame
        Rc = np.swapaxes(W[:, ix['Spine3'], :3, :3], -1, -2) @ W[:, ix[S + 'Arm'], :3, :3]
        Rc0 = rg.rest[ix['Spine3'], :3, :3].T @ rg.rest[ix[S + 'Arm'], :3, :3]
        swc, _ = swing_twist(Rc0.T[None] @ Rc)
        f['uparm_swing_' + s] = swc
        sw, tw = swing_twist(Rel[:, ix[S + 'Shoulder']])
        f['clav_swing_' + s], f['clav_twist_' + s] = sw, np.abs(tw)
        i, p = ix[S + 'Shoulder'], rg.par[ix[S + 'Shoulder']]
        lt = np.einsum('tij,tj->ti', np.swapaxes(W[:, p, :3, :3], -1, -2), P[:, i] - P[:, p])
        f['clav_shift_%s_cm' % s] = np.linalg.norm(lt - rg.Lrest_t[i], axis=1) * 100
        z, y, x = euler_zyx(Rel[:, ix[S + 'Leg']])
        f['knee_flex_' + s], f['knee_dev_' + s], f['shin_roll_' + s] = -z, np.abs(y), np.abs(x)
        sw, tw = swing_twist(Rel[:, ix[S + 'UpLeg']])
        f['hip_flex_' + s], f['hip_twist_' + s] = sw, np.abs(tw)
    for b in ('Spine', 'Spine1', 'Spine2', 'Spine3'):
        sw, tw = swing_twist(Rel[:, ix[b]])
        f['spine_bend_' + b], f['spine_twist_' + b] = sw, np.abs(tw)
    Rh = np.swapaxes(W[:, ix['Hips'], :3, :3], -1, -2)
    for tag, bone, ref in (('spine', 'Spine3', 'Hips'), ('neck', 'Head', 'Spine3')):
        Rr = np.swapaxes(W[:, ix[ref], :3, :3], -1, -2) @ W[:, ix[bone], :3, :3]
        Rr0 = rg.rest[ix[ref], :3, :3].T @ rg.rest[ix[bone], :3, :3]
        sw, tw = swing_twist(Rr0.T[None] @ Rr)
        f[tag + '_bend_total' if tag == 'spine' else 'neck_bend'] = sw
        f[tag + '_twist_total' if tag == 'spine' else 'neck_twist'] = np.abs(tw)
    # torso axis distance
    a, b = P[:, ix['Hips']], P[:, ix['Neck']]
    ab = b - a
    for s, S in (('L', 'Left'), ('R', 'Right')):
        for tag, bone in (('elbow', 'ForeArm'), ('hand', 'Hand')):
            q = P[:, ix[S + bone]] - a
            t = np.clip(np.sum(q * ab, 1) / np.maximum(np.sum(ab * ab, 1), 1e-9), 0, 1)
            f['%s_torso_cm_%s' % (tag, s)] = np.linalg.norm(q - t[:, None] * ab, axis=1) * 100
    v = ab / np.maximum(np.linalg.norm(ab, axis=1, keepdims=True), 1e-9)
    f['torso_lean_deg'] = np.degrees(np.arccos(np.clip(v[:, 2], -1, 1)))
    for s_, S in (('L', 'Left'), ('R', 'Right')):
        for tag, (u0, u1) in (('upperarm', (S + 'Arm', S + 'ForeArm')), ('forearm', (S + 'ForeArm', S + 'Hand'))):
            best = np.full(T, 1e9)
            for w_ in np.linspace(0.0, 1.0, 6):
                q = (P[:, ix[u0]] * (1 - w_) + P[:, ix[u1]] * w_) - a
                t = np.clip(np.sum(q * ab, 1) / np.maximum(np.sum(ab * ab, 1), 1e-9), 0, 1)
                best = np.minimum(best, np.linalg.norm(q - t[:, None] * ab, axis=1))
            f['%s_torso_cm_%s' % (tag, s_)] = best * 100
    dev = np.zeros(T)
    for (u, v), r0 in zip(SEGS, rg.seg_rest):
        dev = np.maximum(dev, np.abs(np.linalg.norm(P[:, ix[u]] - P[:, ix[v]], axis=1) / r0 - 1) * 100)
    f['seg_len_dev_pct'] = dev
    H = MC.heading_frame(P[:, ix['LeftArm']], P[:, ix['RightArm']])           # CHEST heading frame [T,3,3]: right = shoulder line, forward, up
    loc = lambda j: np.einsum('tji,tj->ti', H, P[:, ix[j]] - P[:, ix['Spine3']]) * 100   # noqa: E731
    for s, S, sg in (('L', 'Left', -1.0), ('R', 'Right', 1.0)):
        f['hand_cross_cm_' + s] = -sg * loc(S + 'Hand')[:, 0]
        f['elbow_cross_cm_' + s] = -sg * loc(S + 'ForeArm')[:, 0]
        f['elbow_back_cm_' + s] = -(loc(S + 'ForeArm')[:, 1] - loc(S + 'Arm')[:, 1])
    eyes = (P[:, ix['LeftEye']] + P[:, ix['RightEye']]) / 2
    f['hand_eye_cm'] = np.minimum(np.linalg.norm(P[:, ix['LeftHand']] - eyes, axis=1), np.linalg.norm(P[:, ix['RightHand']] - eyes, axis=1)) * 100
    f['shoulder_width_cm'] = np.sum((P[:, ix['RightArm']] - P[:, ix['LeftArm']]) * H[:, :, 0], 1) * 100
    fz = np.minimum(P[:, ix['LeftFoot'], 2], P[:, ix['RightFoot'], 2])
    rf = min(rg.rest[ix['LeftFoot'], 2, 3], rg.rest[ix['RightFoot'], 2, 3])
    f['hips_drop_cm'] = ((rg.rest[ix['Hips'], 2, 3] - rf) - (P[:, ix['Hips'], 2] - fz)) * 100
    if step_frames:
        st = np.zeros(T)
        M = np.swapaxes(L[:-1], -1, -2) @ L[1:]
        ang = np.degrees(np.arccos(np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))
        st[1:] = ang[:, [ix[b] for b in BODY if b != 'Hips']].max(1)
        f['rot_step'] = st
    return f


# ------------------------------------------------------------------ reference mapping + gate
def src_map_from_keys(keys, n_out, n_src):
    """output frame -> source frame (floats), from keys.json 'keys' (src_frame / time_frame)."""
    tf = [k['time_frame'] for k in keys]
    sf = [k['src_frame'] for k in keys]
    o = np.arange(n_out, dtype=float)
    src = np.interp(o, tf, sf)
    src[o < tf[0]] = sf[0] - (tf[0] - o[o < tf[0]])
    src[o > tf[-1]] = sf[-1] + (o[o > tf[-1]] - tf[-1])
    return np.clip(src, 0, n_src - 1)


def vanilla_on_timeline(d, keys):
    """The vanilla clip re-timed with the key time map only (slerp / lerp of the local transforms, no corrections) -> W [T_out,73,4,4].
    This is the reference of every comparison, so a pure retiming never counts as a deviation."""
    T = len(d['W'])
    n_out = keys[-1]['time_frame'] + (T - 1 - keys[-1]['src_frame']) + 1
    src = src_map_from_keys(keys, n_out, T)
    f0 = np.floor(src).astype(int)
    f1 = np.minimum(f0 + 1, T - 1)
    w = (src - f0)[:, None, None]
    lt = d['lt'][f0].astype(float) * (1 - w) + d['lt'][f1].astype(float) * w
    q0, q1 = d['lr'][f0].astype(float).reshape(-1, 4), d['lr'][f1].astype(float).reshape(-1, 4)
    rv = (R.from_quat(q0).inv() * R.from_quat(q1)).as_rotvec().reshape(n_out, -1, 3)
    lr = (R.from_quat(q0) * R.from_rotvec((rv * w).reshape(-1, 3))).as_quat().reshape(n_out, -1, 4)
    Wl = MC.fk_world(d['parents152'], lt, lr)
    ix = MC.idx(d)
    return np.stack([Wl[:, ix[n]] for n in d['names']], 1)


def window_ref(fv, src, win=WIN):
    """per feature: (min, max) of the vanilla series over [src-win, src+win] for each output frame -> two [T] arrays."""
    T = len(src)
    n = len(fv)
    lo, hi = np.empty(T), np.empty(T)
    for t in range(T):
        a, b = int(np.floor(src[t])) - win, int(np.ceil(src[t])) + win
        seg = fv[max(a, 0):min(b, n - 1) + 1]
        lo[t], hi[t] = seg.min(), seg.max()
    return lo, hi


def gate(Wc, Wv, src, rg, tol_scale=1.0, only=None, deltas=True):
    """Wc corrected W [T,73,4,4], Wv vanilla W, src [T] mapped source frames.
    -> dict(passed, margin_min, worst[...], violations[...], margins{feature: (min margin, frame)})."""
    fc = features(Wc, rg)
    fv = features(Wv, rg)
    ix = rg.ix
    # foot deviation vs mapped vanilla
    T = len(Wc)
    for s, S in (('L', 'Left'), ('R', 'Right')):
        j = ix[S + 'Foot']
        fi = np.clip(np.round(src).astype(int), 0, len(Wv) - 1)
        fc['foot_dev_%s_cm' % s] = np.linalg.norm(Wc[:, j, :3, 3] - Wv[fi, j, :3, 3], axis=1) * 100
        fv['foot_dev_%s_cm' % s] = np.zeros(len(Wv))
    fi = np.clip(np.round(src).astype(int), 0, len(Wv) - 1)
    Lc, Lv = _local(Wc, rg.par), _local(Wv, rg.par)[fi]
    dang = np.degrees(np.arccos(np.clip((np.trace(np.swapaxes(Lc, -1, -2) @ Lv, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))   # [T,J]
    for s, S in (('L', 'Left'), ('R', 'Right')):
        fc['delta_arm_deg_' + s] = dang[:, [ix[S + b] for b in ('Shoulder', 'Arm', 'ForeArm', 'Hand')]].max(1)
        fc['delta_leg_deg_' + s] = dang[:, [ix[S + b] for b in ('UpLeg', 'Leg')]].max(1)
        for k in ('delta_arm_deg_', 'delta_leg_deg_'):
            fv[k + s] = np.zeros(len(Wv))
    Rc3, Rv3 = Wc[:, ix['Spine3'], :3, :3], Wv[fi, ix['Spine3'], :3, :3]
    fc['delta_chest_deg'] = np.degrees(np.arccos(np.clip((np.trace(np.swapaxes(Rc3, -1, -2) @ Rv3, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))
    from scipy.spatial.transform import Rotation as _R
    Hc_ = MC.heading_frame(Wc[:, ix['LeftUpLeg'], :3, 3], Wc[:, ix['RightUpLeg'], :3, 3])
    rvd = _R.from_matrix(Rc3 @ np.swapaxes(Rv3, -1, -2)).as_rotvec()
    fc['delta_pitch_deg'] = np.abs(np.degrees(np.einsum('tji,tj->ti', Hc_, rvd)[:, 0]))
    fv['delta_pitch_deg'] = np.zeros(len(Wv))
    fc['delta_hips_cm'] = np.linalg.norm(Wc[:, ix['Hips'], :3, 3] - Wv[fi, ix['Hips'], :3, 3], axis=1) * 100
    Hc = MC.heading_frame(Wc[:, ix['LeftArm'], :3, 3], Wc[:, ix['RightArm'], :3, 3])
    Hv = MC.heading_frame(Wv[fi, ix['LeftArm'], :3, 3], Wv[fi, ix['RightArm'], :3, 3])
    for s, S in (('L', 'Left'), ('R', 'Right')):
        pc = np.einsum('tji,tj->ti', Hc, Wc[:, ix[S + 'Hand'], :3, 3] - Wc[:, ix['Spine3'], :3, 3])
        pv = np.einsum('tji,tj->ti', Hv, Wv[fi, ix[S + 'Hand'], :3, 3] - Wv[fi, ix['Spine3'], :3, 3])
        fc['delta_hand_chest_cm_' + s] = np.linalg.norm(pc - pv, axis=1) * 100
    sep_c = np.linalg.norm(Wc[:, ix['LeftHand'], :3, 3] - Wc[:, ix['RightHand'], :3, 3], axis=1) * 100
    sep_v = np.linalg.norm(Wv[fi, ix['LeftHand'], :3, 3] - Wv[fi, ix['RightHand'], :3, 3], axis=1) * 100
    def lean(Wx, idx):
        v_ = Wx[idx, ix['Neck'], :3, 3] - Wx[idx, ix['Hips'], :3, 3]
        return v_ / np.maximum(np.linalg.norm(v_, axis=1, keepdims=True), 1e-9)
    fc['delta_lean_deg'] = MC.angle_between(lean(Wc, np.arange(T)), lean(Wv, fi))
    fv['delta_lean_deg'] = np.zeros(len(Wv))
    fc['delta_grip_cm'] = np.where(sep_v < 40.0, np.abs(sep_c - sep_v), 0.0)
    for k in ('delta_grip_cm', 'delta_chest_deg', 'delta_hips_cm', 'delta_hand_chest_cm_L', 'delta_hand_chest_cm_R'):
        fv[k] = np.zeros(len(Wv))
    fault = np.zeros(T, bool)
    for name in FAULT_FEATURES:
        lo_, hi_, _ = LIM[name]
        bad = (fv[name] < lo_) | (fv[name] > hi_)
        ext = np.zeros(len(bad), bool)
        for sft in range(-WIN, WIN + 1):
            ext |= np.roll(bad, sft)
        fault |= ext[np.clip(np.round(src).astype(int), 0, len(bad) - 1)]
    viol, margins = [], {}
    for name, (lo, hi, kind) in LIM.items():
        if name not in fc or (only and name not in only) or (not deltas and name.startswith('delta_')):
            continue
        if name.startswith('delta_'):         # budgets are lifted (x3) where the VANILLA pose itself is broken (arm inside the body / hand at the face)
            hi = hi * np.where(fault, BUDGET_FAULT_MULT, 1.0)
        tol = {'ang': TOL_DEG, 'cm': TOL_CM, 'pct': TOL_PCT, 'step': 0.0}[kind] * tol_scale
        v = fc[name]
        rlo, rhi = window_ref(fv[name], src)
        if kind == 'step':
            up = np.maximum(hi, 1.6 * rhi + 6.0)
            dn = np.full(T, lo)
        else:
            up = np.maximum(hi, rhi + tol)
            dn = np.minimum(lo, rlo - tol)
        one_sided_hi = lo == 0.0 or lo <= -1e8          # absolute-value features / unbounded below
        one_sided_lo = hi >= 1e8                         # distances with only a lower bound
        m = up - v if one_sided_hi else v - dn if one_sided_lo else np.minimum(up - v, v - dn)
        t = int(m.argmin())
        margins[name] = (float(m[t]), t)
        for t in np.nonzero(m < 0)[0]:
            viol.append(dict(feature=name, frame=int(t), value=round(float(v[t]), 2), allowed=[round(float(dn[t]), 2), round(float(up[t]), 2)],
                             vanilla=[round(float(rlo[t]), 2), round(float(rhi[t]), 2)], excess=round(float(-m[t]), 2)))
    viol.sort(key=lambda e: -e['excess'])
    worst = sorted(margins.items(), key=lambda kv: kv[1][0])[:6]
    return dict(passed=not viol, n_violations=len(viol), margin_min=round(min(m[0] for m in margins.values()), 2),
                worst_margins=[(k, round(v[0], 2), v[1]) for k, v in worst], violations=viol[:60], margins={k: round(v[0], 2) for k, v in margins.items()},
                frames_bad=sorted({e['frame'] for e in viol}))


def gate_files(corr_path, set_name, clip, keys_path=None, base=None, deltas=True):
    """Gate a corrected dump (timed / offset / mb.dump npz) against the vanilla clip; keys.json gives the time map."""
    corr_path = corr_path if os.path.isabs(corr_path) else os.path.join(MC.ROOT, corr_path)
    dc = np.load(corr_path, allow_pickle=True)
    dv = MC.load_dump(MC.clip_path(set_name, clip, base))
    rg = rig_of(dv)
    Wc = dc['W']
    if keys_path:
        keys = json.load(open(keys_path, encoding='utf-8'))['keys']
        Wv = vanilla_on_timeline(dv, keys)
    else:
        Wv = dv['W']
    n = min(len(Wc), len(Wv))
    return gate(Wc[:n], Wv[:n], np.arange(n, dtype=float), rg, deltas=deltas)


def report(res, n=8):
    s = 'GATE %s  violations %d  margin_min %.2f' % ('PASS' if res['passed'] else 'FAIL', res['n_violations'], res['margin_min'])
    out = [s, '  tightest: ' + ', '.join('%s %.1f@%d' % w for w in res['worst_margins'])]
    seen = set()
    for e in res['violations']:
        if e['feature'] in seen:
            continue
        seen.add(e['feature'])
        out.append('  ! %-18s f%-3d value %7.1f allowed %s vanilla %s' % (e['feature'], e['frame'], e['value'], e['allowed'], e['vanilla']))
        if len(seen) >= n:
            break
    return '\n'.join(out)


def selftest():
    d = MC.load_dump(MC.clip_path('pwa_katana', 'combo_attack_01'))
    rg = rig_of(d)
    T = len(d['W'])
    src = np.arange(T, dtype=float)
    r = gate(d['W'], d['W'], src, rg)
    assert r['passed'] and r['margin_min'] >= 0, report(r)          # vanilla vs itself always passes
    ix = rg.ix
    # twist the right forearm by 150 deg about its own axis in every frame -> forearm_roll / pronation violations
    W = d['W'].copy()
    j = ix['RightForeArm']
    for t in range(T):
        Rt = R.from_euler('x', 150, degrees=True).as_matrix()
        M = W[t, j, :3, :3] @ Rt
        sub = [k for k in range(len(rg.names)) if _is_desc(rg, k, j)]
        for k in sub + [j]:
            Rk = W[t, k, :3, :3]
            if k == j:
                W[t, k, :3, :3] = M
            else:
                W[t, k, :3, :3] = (M @ W[t, j, :3, :3].T) @ Rk if False else M @ (d['W'][t, j, :3, :3].T @ Rk)
                W[t, k, :3, 3] = W[t, j, :3, 3] + M @ (d['W'][t, j, :3, :3].T @ (d['W'][t, k, :3, 3] - d['W'][t, j, :3, 3]))
    r = gate(W, d['W'], src, rg)
    assert not r['passed'] and any(e['feature'] in ('forearm_roll_R', 'pronation_R') for e in r['violations']), report(r)
    # hyper-extended elbow
    W = d['W'].copy()
    j = ix['LeftForeArm']
    Rz = R.from_euler('z', -170, degrees=True).as_matrix()          # flexion = -rz, so +rz of 170 -> -170 flex; use the opposite sign
    for sgn in (170, -170):
        Rz = R.from_euler('z', sgn, degrees=True).as_matrix()
        W2 = d['W'].copy()
        for t in range(T):
            M = W2[t, j, :3, :3] @ Rz
            for k in [j] + [k for k in range(len(rg.names)) if _is_desc(rg, k, j)]:
                Rk0, pk0 = d['W'][t, k, :3, :3], d['W'][t, k, :3, 3]
                W2[t, k, :3, :3] = M @ (d['W'][t, j, :3, :3].T @ Rk0)
                W2[t, k, :3, 3] = d['W'][t, j, :3, 3] + M @ (d['W'][t, j, :3, :3].T @ (pk0 - d['W'][t, j, :3, 3])) if k != j else pk0
        r = gate(W2, d['W'], src, rg)
        if any(e['feature'] == 'elbow_flex_L' for e in r['violations']):
            break
    else:
        raise AssertionError('elbow reverse bend not detected')
    # vanilla reference loosens: the same pose passes when the "vanilla" has it too
    assert gate(W2, W2, src, rg)['passed']
    print('anatomy_gate selftest OK')


def _is_desc(rg, k, anc):
    while k >= 0:
        k = rg.par[k]
        if k == anc:
            return True
    return False


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a or a[0] in ('-h', '--help'):
        print(__doc__)
    elif a[0] == '--selftest':
        selftest()
    else:
        o = lambda k: a[a.index(k) + 1] if k in a else None  # noqa: E731
        res = gate_files(a[0], o('--set'), o('--clip'), o('--keys'), deltas='--no-deltas' not in a)
        print(report(res, 99 if '--verbose' in a else 8))
        sys.exit(0 if res['passed'] else 1)


# ------------------------------------------------------------------ skin check (what the viewer sees)
SKIN_REGIONS = ['LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand',
                'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'Neck', 'Neck1', 'Spine3']
SKIN_MAX_EXCESS = 0.35      # log2 edge-length change (p98 of a region) beyond what the vanilla clip shows at the same frame


def _read_pc2(path, n_verts):
    return np.fromfile(path, dtype='<f4', offset=32).reshape(-1, n_verts, 3)


def skin_check(skin_dir, W0, rg, src, corr_name='mb', vanilla_name='orig'):
    """Edge stretch / squash of the skinned V mesh in the shoulder, arm, wrist, hip, knee and neck regions: p98 of |log2(edge length at t /
    edge length at frame 0)| per region, corrected clip vs vanilla at the mapped frame (frame 0 is identical in both). Region = nearest joint
    of the vertex at frame 0 (W0 = joint world matrices of frame 0). The mesh is the offline body+arms+head export of skin_clip.py.
    -> dict(passed, worst=[(region, frame, excess)], excess_max)."""
    zc = np.load(os.path.join(skin_dir, corr_name + '.skin.npz'))
    rest = zc['rest'].astype(float)
    faces = zc['faces']
    pc = _read_pc2(os.path.join(skin_dir, corr_name + '.skin.pc2'), len(rest))
    pv = _read_pc2(os.path.join(skin_dir, vanilla_name + '.skin.pc2'), len(rest))
    e = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), 1), axis=0)
    l0 = np.linalg.norm(rest[e[:, 0]] - rest[e[:, 1]], axis=1)
    keep = l0 > 0.004
    e, l0 = e[keep], l0[keep]
    J = np.stack([W0[rg.ix[n], :3, 3] for n in SKIN_REGIONS])
    reg = np.argmin(np.linalg.norm(rest[:, None, :] - J[None], axis=2), axis=1)
    er = reg[e[:, 0]]
    same = er == reg[e[:, 1]]

    def stat(p):
        l = np.linalg.norm(p[e[:, 0]] - p[e[:, 1]], axis=1)
        s = np.abs(np.log2(np.maximum(l, 1e-6) / l0))
        return np.array([np.percentile(s[same & (er == r)], 98) if (same & (er == r)).sum() > 30 else 0.0 for r in range(len(SKIN_REGIONS))])
    sc = np.stack([stat(pc[t]) for t in range(len(pc))])
    sv = np.stack([stat(pv[t]) for t in range(len(pv))])
    fi = np.clip(np.round(src[:len(pc)]).astype(int), 0, len(pv) - 1)
    ex = sc - sv[fi]
    worst = []
    for r in np.argsort(-ex.max(0))[:5]:
        t = int(ex[:, r].argmax())
        worst.append((SKIN_REGIONS[r], t, round(float(ex[t, r]), 3)))
    return dict(passed=bool(ex.max() <= SKIN_MAX_EXCESS), excess_max=round(float(ex.max()), 3), worst=worst)
