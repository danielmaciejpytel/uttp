"""Redo of a game melee clip on the BASE BODY of V (authored pose on a healthy body; the FPP pose is used only as INTENT).

  python -I redo_solve.py <set> <clip> [--out DIR]          -> DIR\\<set>__<clip>.redo.npz (dump layout: W [T,73,4,4] world, rest, names, parents)
Intent kept from the game clip: hand (grip) trajectories relative to the chest, hand orientation relative to the chest, timing (frame for frame),
gross torso direction (turn follows the sideways motion of the hands, lean follows their height / reach).  Everything else is rebuilt from the base:
  clavicles / shoulders  base pose, local translations untouched (width = base), only a small swing (elevation / protraction) towards the reach
  spine                  base locals + chest turn / lean spread over Spine..Spine3 (hips take 40 % of the turn)
  head / neck            base, follows 25 % of the chest turn, never more than ~35 deg
  arms                   hinge IK (elbow = pure flexion, bend plane by an anatomical pole: down + out + slightly back), forearm roll + wrist inside limits,
                         hand targets pushed away from face / torso and pulled inside reach; two-hand grips move as ONE rigid pair
  legs                   base stance (feet pinned, no slide), hips shift / sink with the reach, feet pivot with the hips; hinge IK, knee pole = base knee side
Per-frame: frames the base-body gate (anatomy_gate_base) passes stay as in the game (blend_keep); everything else is the redo, with smoothed transitions.
Round F (2026-11-09, review M5, user: the strike in game must stay the game's strike, aimed at runtime by the DLL): INTENT_MODE (env UTPP_REDO_INTENT, recorded by
the callers) selects the hand objective.  'v3b' (default, = redo\\ / redo_man\\, bit-identical): targets = the BASE chest position (static) + the game hand offset,
pushed out along shoulder -> target to the game's arm extension.  'game_path': the game's hand path relative to the chest IS the objective: target = the SOLVED
chest position of the frame (Spine3 of the planned body) + arm-length ratio x (game hand - game Spine3), in the game axes of the clip (chest position + the
character heading, world up: the strike stays level, no chest pitch / roll of the FPP clip is copied); no arm-extension push.  Clearance pushes (eye / torso /
reach / arm-torso), the gate and every base-body rule are unchanged, so where they act the hand still moves (the trace of solve() shows how far and why).
Round F step 3 (review M4): PEN_MODE (env UTPP_REDO_PEN) 'axis' (default, unchanged: the arm-torso push keeps the arm segments ARM_CLEAR from the Hips-Neck
AXIS) | 'surface': additionally the forearm / hand points keep clear of the torso SURFACE (redo_pen.torso_model: cross-section profile of V's torso vertices
+ forearm radius; healthy reference poses keep >= 4.3 cm), pushed out perpendicular to the axis with the same push rule (x1.4, total <= MAX_PUSH, side never
flips); keep_mask() also refuses to keep a game frame whose arms go through the torso (redo_pen proxy).
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate_base as GB  # noqa: E402
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_pen as RPN  # noqa: E402   (round F: torso surface model / penetration proxy, loaded lazily)

CP = BB.CP
Z = np.array([0.0, 0.0, 1.0])
# tunables
TURN_K = 0.55            # chest turn per hand angle change
HIPS_TURN = 0.40         # share of the turn taken by the hips
SPINE_CUM = (0.20, 0.45, 0.75, 1.00)
HEAD_FOLLOW = 0.25
EYE_CLEAR = 0.30
TORSO_CLEAR = 0.14
REACH = 0.965
ARM_CLEAR = 0.10
MAX_PUSH = 0.09            # total clearance push of a hand target (m)
HOLD_DEG = 55.0           # max change of the elbow bend side per frame
MIN_REACH = 0.40         # hand at least 40 % of the arm length from the shoulder (elbow flexion <= ~135 deg)
SMOOTH = 5
INTENT_MODE = os.environ.get('UTPP_REDO_INTENT', 'v3b')
assert INTENT_MODE in ('v3b', 'game_path'), 'UTPP_REDO_INTENT must be v3b or game_path, got %r' % INTENT_MODE
PEN_MODE = os.environ.get('UTPP_REDO_PEN', 'axis')
assert PEN_MODE in ('axis', 'surface'), 'UTPP_REDO_PEN must be axis or surface, got %r' % PEN_MODE
# round F step 4: 'held' (default, v3b) = every temporal smoothing holds the clip ends; 'periodic' = for a CLOSED loop clip (the game clip's first and
# last frames equal) the smoothing and the keep ramp wrap around the cycle (period T-1), so the seam of a loop (first vs last frame) stays closed.  Measured
# before: v3b sprint loops open by up to 9.9 cm / 40 deg at the seam (smoothing with held ends sees different neighbours at frame 0 and at frame T-1).
LOOP_MODE = os.environ.get('UTPP_REDO_LOOP', 'held')
assert LOOP_MODE in ('held', 'periodic'), 'UTPP_REDO_LOOP must be held or periodic, got %r' % LOOP_MODE
_PERIOD_T = 0          # > 0: the length of the closed loop clip being solved (smooth() wraps arrays of exactly this length)


def closed_loop(W):
    W = np.asarray(W)
    return len(W) > 8 and float(np.abs(W[0] - W[-1]).max()) < 1e-4       # equal up to float noise (pwa_knife melee_sprint_loop: 1.5e-6)


def set_loop(d):
    """Called at the start of plan_intent / solve: periodic smoothing for a closed loop clip when LOOP_MODE is 'periodic' (else the v3b behaviour)."""
    global _PERIOD_T
    _PERIOD_T = len(d['W']) if LOOP_MODE == 'periodic' and closed_loop(d['W']) else 0
    return _PERIOD_T > 0


def frame_order(T):
    """Order of the per-frame IK sweep.  v3b: 0..T-1.  Closed loop clip with LOOP_MODE 'periodic': two warm-up cycles (1..T-2, 0, 1..T-2) before 0..T-1, so the
    continuity state carried from frame to frame (elbow bend side memo, push side, wrist twist wrap, pinned feet) at frame 0 is the state of the frame before it
    in the cycle; the warm-up results are overwritten by the final sweep."""
    if _PERIOD_T and _PERIOD_T == T:
        return list(range(1, T - 1)) + [0] + list(range(1, T - 1)) + list(range(T))
    return range(T)


def _pad_time(x, h):
    if _PERIOD_T and len(x) == _PERIOD_T and len(x) > h + 1:
        return np.concatenate([x[len(x) - 1 - h:len(x) - 1], x, x[1:1 + h]])
    return np.concatenate([np.repeat(x[:1], h, 0), x, np.repeat(x[-1:], h, 0)])


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rotvec(v):
    return R.from_rotvec(v).as_matrix()


def rotvec_of(m):
    return R.from_matrix(m).as_rotvec()


def smooth(x, n=SMOOTH):
    """Triangular low-pass over time (axis 0), ends held."""
    if n <= 1 or len(x) < 3:
        return x.copy()
    k = np.concatenate([np.arange(1, n // 2 + 2), np.arange(n // 2, 0, -1)]).astype(float)
    k /= k.sum()
    h = len(k) // 2
    xp = _pad_time(x, h)
    sh = x.shape[1:]
    flat = xp.reshape(len(xp), -1)
    out = np.stack([np.convolve(flat[:, c], k, mode='valid') for c in range(flat.shape[1])], 1)
    return out.reshape((len(x),) + sh)


class Body:
    def __init__(self):
        self.names, self.par, ref, self.rest = BB.rig()
        self.par = [int(p) for p in self.par]
        self.ix = {n: i for i, n in enumerate(self.names)}
        z = np.load(os.path.join(BB.REDO, 'base_pose.npz'))
        self.Wb = BB.inplace(z['W'], self.names)[0]            # base in the model frame (hips heading +Y, ground z = 0)
        self.Wi = BB.inplace(z['Wi'], self.names)[0]            # neutral idle (shown as the reference panel)
        self.Lb = self.locals_of(self.Wb)
        self.Lrest = self.locals_of(self.rest)
        self.rg = BB.rig_for_gate()

    def locals_of(self, W):
        L = np.tile(np.eye(4), (len(self.names), 1, 1))
        for i, p in enumerate(self.par):
            L[i] = W[i] if p < 0 else np.linalg.inv(W[p]) @ W[i]
        return L

    def fk(self, L):
        W = np.empty_like(L)
        for i, p in enumerate(self.par):
            W[i] = L[i] if p < 0 else W[p] @ L[i]
        return W


def rot_between(a, b):
    a = a / max(np.linalg.norm(a), 1e-9)
    b = b / max(np.linalg.norm(b), 1e-9)
    c = float(np.dot(a, b))
    if c > 1 - 1e-9:
        return np.eye(3)
    if c < -1 + 1e-9:
        ax = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(ax) < 1e-6:
            ax = np.cross(a, [0, 1.0, 0])
        return rotvec(ax / np.linalg.norm(ax) * np.pi)
    ax = np.cross(a, b)
    return rotvec(ax / np.linalg.norm(ax) * np.arccos(c))


def set_world_rot(B, L, j, Rw):
    """Set the world rotation of joint j (parents already final in L): local rotation only."""
    p = B.par[j]
    Wp = B.fk(L)[p] if p >= 0 else np.eye(4)
    L[j, :3, :3] = Wp[:3, :3].T @ Rw


def two_bone(B, L, a, b, c, target, pole, end_world=None, memo=None, twist_limit=None):
    """Hinge IK in place on locals L. Elbow / knee = flexion about local Z only (roll about X kept); the parent swings so the end reaches `target`, then
    twists about parent->target so the middle joint lies on the `pole` side.  Returns the residual distance (m)."""
    W0 = B.fk(L)
    A, Bj, C = W0[a, :3, 3], W0[b, :3, 3], W0[c, :3, 3]
    la, lb = np.linalg.norm(Bj - A), np.linalg.norm(C - Bj)
    dv = target - A
    d = float(np.clip(np.linalg.norm(dv), abs(la - lb) + 1e-4, la + lb - 1e-4))
    u = dv / max(np.linalg.norm(dv), 1e-9)
    cos_b = np.clip((la * la + lb * lb - d * d) / (2 * la * lb), -1, 1)
    flex = np.pi - np.arccos(cos_b)
    Lrest_b = B.Lrest[b, :3, :3]
    z, y, x = R.from_matrix(Lrest_b.T @ L[b, :3, :3]).as_euler('ZYX')
    L[b, :3, :3] = Lrest_b @ R.from_euler('ZYX', [-flex, 0.0, x]).as_matrix()
    W1 = B.fk(L)
    Rsw = rot_between(W1[c, :3, 3] - A, u)
    Ra2 = Rsw @ W1[a, :3, :3]
    Bs = Rsw @ (W1[b, :3, 3] - A)
    vcur = Bs - np.dot(Bs, u) * u
    vp = pole - np.dot(pole, u) * u
    if memo is not None:                      # continuity: a pole (almost) parallel to the reach axis falls back to the previous bend side
        wp = float(np.clip(np.linalg.norm(vp) / 0.5, 0.0, 1.0))
        pv_prev = memo.get('v')
        if pv_prev is not None:
            pp = pv_prev - np.dot(pv_prev, u) * u
            if np.linalg.norm(pp) > 1e-6:
                vpn = vp / max(np.linalg.norm(vp), 1e-9)
                ppn = pp / np.linalg.norm(pp)
                if np.dot(vpn, ppn) < np.cos(np.radians(HOLD_DEG)):      # the bend side would flip: swing it gradually (at most HOLD_DEG per frame)
                    ax = np.cross(ppn, vpn)
                    ax = ax / np.linalg.norm(ax) if np.linalg.norm(ax) > 1e-6 else u
                    vpn = rotvec(ax * np.radians(HOLD_DEG)) @ ppn
                vp = vpn * wp + ppn * (1.0 - wp)
    if np.linalg.norm(vcur) > 1e-6 and np.linalg.norm(vp) > 1e-6:
        vcur /= np.linalg.norm(vcur)
        vp /= np.linalg.norm(vp)
        if memo is not None:
            memo['v'] = vp.copy()
        ang = np.arctan2(np.dot(np.cross(vcur, vp), u), np.dot(vcur, vp))
        Ra2 = rotvec(u * ang) @ Ra2
    if twist_limit is not None:                # candy-wrap guard: turn the bend side about the reach axis until the upper-arm twist (rest-relative) is inside the limit
        Wp = W1[B.par[a]] if B.par[a] >= 0 else np.eye(4)
        best, best_tw = Ra2, None
        for dl in (0, 25, -25, 50, -50, 75, -75, 100, -100):
            Rc = rotvec(u * np.radians(dl)) @ Ra2
            q = R.from_matrix(B.Lrest[a, :3, :3].T @ (Wp[:3, :3].T @ Rc)).as_quat()
            tw = abs((np.degrees(2.0 * np.arctan2(q[0], q[3])) + 180.0) % 360.0 - 180.0)
            if best_tw is None or tw < best_tw:
                best, best_tw = Rc, tw
            if tw <= twist_limit:
                best = Rc
                break
        Ra2 = best
    set_world_rot(B, L, a, Ra2)
    if end_world is not None:
        set_world_rot(B, L, c, end_world)
    return float(np.linalg.norm(B.fk(L)[c, :3, 3] - target)), flex


def wrist(B, L, side, Rh_t, lim, memo=None):
    """Forearm roll + wrist (flex / deviation / twist) inside limits for the desired world hand rotation Rh_t."""
    S = side
    fa, hd = B.ix[S + 'ForeArm'], B.ix[S + 'Hand']
    # roll 0 first, then the twist of the needed hand rotation about the bone axis X goes mostly to the forearm
    z, y, x = R.from_matrix(B.Lrest[fa, :3, :3].T @ L[fa, :3, :3]).as_euler('ZYX')
    L[fa, :3, :3] = B.Lrest[fa, :3, :3] @ R.from_euler('ZYX', [z, 0.0, 0.0]).as_matrix()
    W = B.fk(L)
    rel = B.Lrest[hd, :3, :3].T @ (W[fa, :3, :3].T @ Rh_t)
    tw = np.degrees(R.from_matrix(rel).as_euler('ZYX')[2])
    twh = np.degrees(2.0 * np.arctan2(R.from_matrix(rel).as_quat()[0], R.from_matrix(rel).as_quat()[3]))
    twh = (twh + 180.0) % 360.0 - 180.0
    if memo is not None:                      # continuity of the needed twist across the +-180 wrap
        if 'tw' in memo:
            twh = memo['tw'] + ((twh - memo['tw'] + 180.0) % 360.0 - 180.0)
        memo['tw'] = twh
    wr = float(np.clip(0.3 * twh, -lim['twist'], lim['twist']))
    roll = float(np.clip(twh - wr, -lim['roll'], lim['roll']))
    L[fa, :3, :3] = B.Lrest[fa, :3, :3] @ R.from_euler('ZYX', [z, 0.0, np.radians(roll)]).as_matrix()
    W = B.fk(L)
    rel = B.Lrest[hd, :3, :3].T @ (W[fa, :3, :3].T @ Rh_t)
    zz, yy, xx = np.degrees(R.from_matrix(rel).as_euler('ZYX'))
    zz = lim['flex'] * np.tanh(zz / lim['flex'])          # soft limits: the hand follows the intent but never reaches the end stops
    yy = lim['dev'] * np.tanh(yy / lim['dev'])
    xx = lim['twist'] * np.tanh(xx / lim['twist'])
    L[hd, :3, :3] = B.Lrest[hd, :3, :3] @ R.from_euler('ZYX', np.radians([zz, yy, xx])).as_matrix()


def chest_yaw(Wv, ix):
    f = -Wv[:, ix['Spine3'], :3, 1]
    return np.arctan2(-f[:, 0], f[:, 1])


def arm_ratio(B, Wv):
    """{side: arm length (Arm -> ForeArm -> Hand) of V's base body / of the game clip at frame 0} (round F)."""
    ix = B.ix
    out = {}
    for S in ('Left', 'Right'):
        a, f, h = ix[S + 'Arm'], ix[S + 'ForeArm'], ix[S + 'Hand']
        lb = np.linalg.norm(B.Wb[f, :3, 3] - B.Wb[a, :3, 3]) + np.linalg.norm(B.Wb[h, :3, 3] - B.Wb[f, :3, 3])
        lv = np.linalg.norm(Wv[0, f, :3, 3] - Wv[0, a, :3, 3]) + np.linalg.norm(Wv[0, h, :3, 3] - Wv[0, f, :3, 3])
        out[S] = float(lb / max(lv, 1e-6))
    return out


def plan_intent(B, d):
    """Vanilla clip -> chest-relative hand targets (model frame), hand orientation relative to the vanilla chest, torso intent (yaw / pitch / hips shift)."""
    set_loop(d)
    ix = B.ix
    Wv = d['W']
    T = len(Wv)
    traj = CP.trajectory(Wv, B.rest, B.names, win=1)
    Tinv = np.linalg.inv(traj)
    Wm = Tinv[:, None] @ Wv                                         # per frame model frame (character-relative)
    Pm = Wm[:, :, :3, 3]
    S3v = Pm[:, ix['Spine3']]
    S3b = B.Wb[ix['Spine3'], :3, 3]
    hands, rots = {}, {}
    for S in ('Left', 'Right'):
        hands[S] = S3b + (Pm[:, ix[S + 'Hand']] - S3v)
        # hand orientation relative to the vanilla chest, applied on the base chest
        rots[S] = np.swapaxes(Wm[:, ix['Spine3'], :3, :3], -1, -2) @ Wm[:, ix[S + 'Hand'], :3, :3]
    for S in ('Left', 'Right'):                  # hand orientation intent: smooth rotation-vector series (hemisphere consistent), no frame-to-frame twist flips
        q = R.from_matrix(rots[S]).as_quat()
        for t in range(1, len(q)):
            if np.dot(q[t], q[t - 1]) < 0:
                q[t] = -q[t]
        q = smooth(q, 3)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        rots[S] = R.from_quat(q).as_matrix()
    mid = (hands['Left'] + hands['Right']) / 2
    alpha = np.arctan2(mid[:, 0] - S3b[0], mid[:, 1] - S3b[1] + 0.0)
    alpha = smooth(alpha[:, None], 3)[:, 0]
    # extension (forward reach of the mid-hand beyond frame 0) and height
    ext = smooth((mid[:, 1] - mid[0, 1])[:, None], 3)[:, 0]
    hgt = smooth((mid[:, 2] - mid[0, 2])[:, None], 3)[:, 0]
    sep0 = np.linalg.norm(hands['Left'] - hands['Right'], axis=1)
    tk = 1.0 if np.median(sep0) < 0.40 else TURN_K       # a two-hand grip swings with the whole torso
    yaw = -tk * (alpha - alpha[0])
    yaw = np.clip(yaw, np.radians(-55), np.radians(55))
    pitch_fwd = np.clip(np.clip(-hgt, 0, 0.5) * 18 + np.clip(ext, 0, 0.4) * 10, 0, 10)   # deg forward lean: hands going down / forward
    sep = np.linalg.norm(hands['Left'] - hands['Right'], axis=1)
    grip = bool(np.median(sep) < 0.40)
    # round F 'game_path': the game hand offset from the game chest (Spine3) in the game axes of the clip (frame-0 character frame = the output frame of solve),
    # scaled by the arm-length ratio of V's base body to the game rig
    R0 = traj[0][:3, :3]
    ratio = arm_ratio(B, Wv)
    gp = {S: ratio[S] * np.einsum('ji,tj->ti', R0, Wv[:, ix[S + 'Hand'], :3, 3] - Wv[:, ix['Spine3'], :3, 3]) for S in ('Left', 'Right')}
    fing = [i for i, n in enumerate(B.names) if (n.startswith('LeftHand') or n.startswith('RightHand')) and n not in ('LeftHand', 'RightHand')]
    Lfing = np.stack([B.locals_of(Wm[t])[fing] for t in range(T)])
    fext = {}
    for S in ('Left', 'Right'):
        A, Bj, Cj = Pm[:, ix[S + 'Arm']], Pm[:, ix[S + 'ForeArm']], Pm[:, ix[S + 'Hand']]
        fext[S] = np.linalg.norm(Cj - A, axis=1) / (np.linalg.norm(Bj - A, axis=1) + np.linalg.norm(Cj - Bj, axis=1))     # arm extension (1 = straight)
    return dict(fext=fext, fing=fing, Lfing=Lfing, hands=hands, rots=rots, yaw=smooth(yaw[:, None])[:, 0], pitch_deg=smooth(pitch_fwd[:, None])[:, 0],
                ext=np.clip(ext, 0, 0.5), grip=grip, traj0=traj[0], T=T, gp=gp, arm_ratio=ratio, mode=INTENT_MODE)


def build_frames(B, intent):
    """Plan per-frame state (no IK yet): hips / chest values, clavicle angles, pole directions, then the solve itself."""
    T, ix = intent['T'], B.ix
    yaw, pitch = intent['yaw'], np.radians(intent['pitch_deg'])
    sh_b = {S: B.Wb[ix[S + 'Arm'], :3, 3] for S in ('Left', 'Right')}
    hb = {S: B.Wb[ix[S + 'Hand'], :3, 3] for S in ('Left', 'Right')}
    ang_b, fwd_b = {}, {}
    for S in ('Left', 'Right'):
        v = hb[S] - sh_b[S]
        ang_b[S] = np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1])))
        fwd_b[S] = v[1]
    elev, prot, pole = {}, {}, {}
    for S, sg in (('Left', -1.0), ('Right', 1.0)):
        v = intent['hands'][S] - sh_b[S]
        a_up = np.degrees(np.arctan2(v[:, 2], np.hypot(v[:, 0], v[:, 1])))
        elev[S] = smooth(np.clip(0.18 * (a_up - ang_b[S]), -4.0, 9.0)[:, None])[:, 0]
        prot[S] = smooth(np.clip(30.0 * (v[:, 1] - fwd_b[S]), -5.0, 9.0)[:, None])[:, 0]
        w_hi = np.clip((a_up - 15.0) / 60.0, 0, 1)
        pv = np.stack([np.full(T, sg * 0.55) + sg * 0.5 * w_hi, np.full(T, -0.25), -1.0 + 0.85 * w_hi], 1)
        pole[S] = smooth(pv)
        pole[S] /= np.linalg.norm(pole[S], axis=1, keepdims=True)
    return dict(elev=elev, prot=prot, pole=pole)


def solve(B, d, intent=None, log=None, trace=None):
    """-> W [T,73,4,4] in the world of the game clip (frame-0 character frame), info.  trace (dict, optional): per-frame hand targets of every stage (model frame):
    raw, ext (after the arm-extension push), adj (after eye / torso / reach clearance, smoothed), final (+ arm-torso push), hand (IK result), s3 (solved chest)."""
    set_loop(d)
    ix = B.ix
    intent = intent or plan_intent(B, d)
    mode = intent.get('mode', 'v3b')
    T = intent['T']
    pl = build_frames(B, intent)
    base_head = B.Wb[ix['Head'], :3, :3]
    out = np.empty((T, len(B.names), 4, 4))
    feet0 = {S: B.Wb[ix[S + 'Foot']] for S in ('Left', 'Right')}
    eye0 = (B.Wb[ix['LeftEye'], :3, 3] + B.Wb[ix['RightEye'], :3, 3]) / 2
    lim = dict(flex=55.0, dev=24.0, twist=26.0, roll=80.0)
    res_info = dict(reach_resid_max_cm=0.0, flex_min=1e9)
    grip = intent['grip']
    adj_prev = None
    deltas = []
    # pass 1: targets after body placement (needs shoulders), clearance adjust as a smoothed delta
    states = []
    for t in range(T):
        L = B.Lb.copy()
        yawT, pit = float(intent['yaw'][t]), np.radians(float(intent['pitch_deg'][t]))
        ext = float(intent['ext'][t])
        # hips: turn + weight shift / sink
        yh = HIPS_TURN * yawT
        L[ix['Hips'], :3, :3] = rotz(yh) @ L[ix['Hips'], :3, :3]
        Wb_h = B.Wb[ix['Hips'], :3, 3]
        shift = np.array([0.0, 0.03 * ext / 0.4, -0.03 * ext / 0.4])
        L[ix['Hips'], :3, 3] = L[ix['Hips'], :3, 3] + shift
        # spine chain: remaining turn + lean, distributed
        omega = np.array([-pit, 0.0, 0.0]) + Z * (yawT - yh)
        prev = 0.0
        for nm, cum in zip(('Spine', 'Spine1', 'Spine2', 'Spine3'), SPINE_CUM):
            j = ix[nm]
            Wc = B.fk(L)
            Rb = Wc[j, :3, :3]
            Rw = rotvec(omega * (cum - prev)) @ Rb
            set_world_rot(B, L, j, Rw)
            prev = cum
        # head / neck: keep looking along the base heading, follow part of the chest turn
        Wc = B.fk(L)
        Rs3 = Wc[ix['Spine3'], :3, :3]
        Rrel_base = B.Wb[ix['Spine3'], :3, :3].T @ base_head
        Cnow = Rs3.T @ base_head
        dl = rotvec_of(Rrel_base.T @ Cnow) * (1.0 - HEAD_FOLLOW)
        n = np.linalg.norm(dl)
        if n > np.radians(35):
            dl *= np.radians(35) / n
        for nm, w in (('Neck', 0.25), ('Neck1', 0.25), ('Head', 0.5)):
            j = ix[nm]
            L[j, :3, :3] = L[j, :3, :3] @ rotvec(dl * w)
        # clavicles (swing towards the reach, translations untouched)
        Wc = B.fk(L)
        for S, sg in (('Left', -1.0), ('Right', 1.0)):
            sj, aj = ix[S + 'Shoulder'], ix[S + 'Arm']
            d0 = Wc[aj, :3, 3] - Wc[sj, :3, 3]
            e, p = np.radians(pl['elev'][S][t]), np.radians(pl['prot'][S][t])
            d1 = rotvec(np.array([0.0, 1.0, 0.0]) * (-sg * e)) @ (rotz(sg * p) @ d0)
            Rsw = rot_between(d0, d1)
            set_world_rot(B, L, sj, Rsw @ Wc[sj, :3, :3])
        L[intent['fing']] = intent['Lfing'][t]          # fingers: the game's (fist / grip), they are intent of the hand
        states.append(L)
    # pass 2: targets (adjusted), IK arms + legs
    Wst = [B.fk(L) for L in states]
    gpw = np.asarray(intent.get('gp_w', np.ones(T)), float)      # round F: per-frame weight of the game path (0 = the v3b target, gate fallback)
    spw = np.asarray(intent.get('sp_w', np.ones(T)), float)      # round F step 3: per-frame weight of the torso-surface push (0 = axis rule only, gate fallback)
    if mode == 'game_path':     # round F: the solved chest of the frame + the (scaled) game hand offset
        s3 = np.stack([W[ix['Spine3'], :3, 3] for W in Wst])
        raw = {S: gpw[:, None] * (s3 + intent['gp'][S]) + (1.0 - gpw[:, None]) * intent['hands'][S] for S in ('Left', 'Right')}
    else:
        raw = {S: intent['hands'][S].copy() for S in ('Left', 'Right')}
    tr_ext = {S: raw[S].copy() for S in ('Left', 'Right')}
    tr_cl = {k: {S: np.zeros((T, 3)) for S in ('Left', 'Right')} for k in ('eye', 'torso', 'reach', 'min_reach')}     # trace: clearance shift per kind
    adj = {S: raw[S].copy() for S in ('Left', 'Right')}
    shift_pair = np.zeros((T, 3))
    for t in range(T):
        W = Wst[t]
        eye = (W[ix['LeftEye'], :3, 3] + W[ix['RightEye'], :3, 3]) / 2
        ha, hn = W[ix['Hips'], :3, 3], W[ix['Neck'], :3, 3]
        ab = hn - ha
        tg = {S: raw[S][t].copy() for S in ('Left', 'Right')}
        sh = {S: W[ix[S + 'Arm'], :3, 3] for S in ('Left', 'Right')}
        la = {S: np.linalg.norm(W[ix[S + 'ForeArm'], :3, 3] - sh[S]) + np.linalg.norm(W[ix[S + 'Hand'], :3, 3] - W[ix[S + 'ForeArm'], :3, 3]) for S in sh}
        pairs = [('Left', 'Right')] if grip else [('Left',), ('Right',)]
        ex = {}
        for S in ('Left', 'Right'):               # arm extension of the game clip (straight arm = strike): the redo arm is as extended, its shoulders sit further forward than the FPP ones
            v = tg[S] - sh[S]
            ds = np.linalg.norm(v)
            want = min(float(intent['fext'][S][t]), 0.97) * la[S]
            fw = float(np.clip((tg[S][1] - W[ix['Spine3'], 1, 3] + 0.02) / 0.25, 0.0, 1.0))      # only for reaches in FRONT of the chest (hands behind the back stay bent)
            ex[S] = v / max(ds, 1e-6) * max(want - ds, 0.0) * fw
            if mode == 'game_path':
                ex[S] = ex[S] * (1.0 - gpw[t])      # round F: the game hand path is the objective, no push to the game's arm extension (only the v3b share)
        if grip:
            m_ = (ex['Left'] + ex['Right']) / 2
            ex = {'Left': m_, 'Right': m_}
        for S in ('Left', 'Right'):
            tg[S] = tg[S] + ex[S]
            tr_ext[S][t] = tg[S]
        for grp in pairs:
            shift = np.zeros(3)
            acc = {k: np.zeros(3) for k in tr_cl}
            for it in range(4):
                for S in grp:
                    q = tg[S] + shift
                    # eye clearance
                    v = q - eye
                    dist = np.linalg.norm(v)
                    if dist < EYE_CLEAR:
                        acc['eye'] += v / max(dist, 1e-6) * (EYE_CLEAR - dist)
                        shift += v / max(dist, 1e-6) * (EYE_CLEAR - dist)
                        q = tg[S] + shift
                    # torso clearance (distance to the hips-neck line)
                    qa = q - ha
                    tt = np.clip(np.dot(qa, ab) / np.dot(ab, ab), 0, 1)
                    perp = qa - tt * ab
                    pn = np.linalg.norm(perp)
                    if pn < TORSO_CLEAR:
                        dirn = perp / pn if pn > 1e-6 else np.array([1.0 if S == 'Right' else -1.0, 0.3, 0.0])
                        acc['torso'] += dirn * (TORSO_CLEAR - pn)
                        shift += dirn * (TORSO_CLEAR - pn)
                        q = tg[S] + shift
                    # reach
                    vs = q - sh[S]
                    ds = np.linalg.norm(vs)
                    if ds > REACH * la[S]:
                        acc['reach'] += -vs / ds * (ds - REACH * la[S])
                        shift += -vs / ds * (ds - REACH * la[S])
                    dmin = MIN_REACH * la[S]
                    if ds < dmin:
                        acc['min_reach'] += vs / max(ds, 1e-6) * (dmin - ds)
                        shift += vs / max(ds, 1e-6) * (dmin - ds)
            for S in grp:
                adj[S][t] = tg[S] + shift
                for k in acc:
                    tr_cl[k][S][t] = acc[k]
    for S in ('Left', 'Right'):
        dlt = adj[S] - raw[S]
        adj[S] = raw[S] + smooth(dlt, 5)
    # final solve
    adj_use = adj
    pen_stat = dict(pushed=0)
    for _pass in range(2):      # pass 1 finds the clearance pushes, pass 2 re-solves with them smoothed over time (no per-frame jumps)
        extra_t = {S: np.zeros((T, 3)) for S in ('Left', 'Right')}
        last_dir = {}
        resid = 0.0
        leg_resid = 0.0
        memo = {'Left': {}, 'Right': {}}
        keep = {S: Wb for S, Wb in feet0.items()}
        for t in frame_order(T):
            L = states[t].copy()
            W = Wst[t]
            # legs first (pinned feet; foot yaw follows the hips partly)
            yh = HIPS_TURN * float(intent['yaw'][t])
            for S in ('Left', 'Right'):
                ul, lg, ft = ix[S + 'UpLeg'], ix[S + 'Leg'], ix[S + 'Foot']
                A0, B0 = B.Wb[ul, :3, 3], B.Wb[lg, :3, 3]
                C0 = B.Wb[ft, :3, 3]
                ac = C0 - A0
                pv = (B0 - A0) - np.dot(B0 - A0, ac / np.linalg.norm(ac)) * ac / np.linalg.norm(ac)
                if np.linalg.norm(pv) < 1e-4:
                    pv = np.array([0.0, 1.0, 0.0])
                pv = rotz(yh) @ pv
                Rfoot = rotz(0.7 * yh) @ feet0[S][:3, :3]
                toe0 = B.Wb[ix[S + 'ToeBase'], :3, 3]            # the foot pivots about the ball (toe base): the toe base never moves
                ank = toe0 + rotz(0.7 * yh) @ (feet0[S][:3, 3] - toe0)
                rl, _ = two_bone(B, L, ul, lg, ft, ank, pv, end_world=Rfoot)
                leg_resid = max(leg_resid, rl)
            W = B.fk(L)
            # arms
            ha, hn = W[ix['Hips'], :3, 3], W[ix['Neck'], :3, 3]
            ab = hn - ha
            Lsave = L.copy()
            tg = {S: adj_use[S][t].copy() for S in ('Left', 'Right')}
            groups = [('Left', 'Right')] if grip else [('Left',), ('Right',)]
            for grp in groups:                      # a grip pair is pushed by ONE shared vector (the grip stays rigid)
                for it in range(5):
                    mm = {S: dict(memo[S]) for S in grp}
                    worst, wdir, wS = 1e9, None, None
                    for S in grp:
                        sg = -1.0 if S == 'Left' else 1.0
                        a, b, c = ix[S + 'Arm'], ix[S + 'ForeArm'], ix[S + 'Hand']
                        L[[a, b, c]] = Lsave[[a, b, c]]
                        rs, _ = two_bone(B, L, a, b, c, tg[S], pl['pole'][S][t], memo=mm[S], twist_limit=70.0)
                        resid = max(resid, rs)
                        Rs3 = B.fk(L)[ix['Spine3'], :3, :3]
                        wrist(B, L, S, Rs3 @ intent['rots'][S][t], lim, memo=mm[S])
                        Wa = B.fk(L)
                        for j0, j1 in ((a, b), (b, c)):
                            for w_ in np.linspace(0.0, 1.0, 6):
                                q = Wa[j0, :3, 3] * (1 - w_) + Wa[j1, :3, 3] * w_ - ha
                                tt = np.clip(np.dot(q, ab) / np.dot(ab, ab), 0, 1)
                                perp = q - tt * ab
                                dn = np.linalg.norm(perp)
                                if dn < worst:
                                    worst, wdir = dn, (perp / dn if dn > 1e-6 else np.array([sg, 0.2, 0.0]))
                    deficit = ARM_CLEAR - worst
                    if PEN_MODE == 'surface':           # round F step 3: forearm / hand vs the torso SURFACE (not only the axis)
                        Wa = B.fk(L)
                        fr = RPN.torso_frame(Wa, ix)
                        for S in grp:
                            cl, dirs = RPN.surface_clear(RPN.forearm_points(Wa, ix, S), fr)
                            k_ = int(np.argmin(cl))
                            if np.isfinite(cl[k_]) and -cl[k_] * spw[t] > deficit:
                                deficit, wdir = float(-cl[k_] * spw[t]), dirs[k_]
                                pen_stat['pushed'] += 1
                    if deficit <= 0:
                        break
                    if last_dir.get(grp) is not None and np.dot(wdir, last_dir[grp]) < 0:
                        wdir = last_dir[grp]            # the push side never flips from one frame to the next (a flip throws the hands across the body)
                    last_dir[grp] = wdir
                    for S in grp:
                        tg[S] = tg[S] + wdir * deficit * 1.4
                        dd = tg[S] - adj_use[S][t]
                        nn = np.linalg.norm(dd)
                        if nn > MAX_PUSH:
                            tg[S] = adj_use[S][t] + dd / nn * MAX_PUSH
                for S in grp:
                    memo[S].update(mm[S])
            out[t] = B.fk(L)
            for S in ('Left', 'Right'):
                extra_t[S][t] = tg[S] - adj_use[S][t]

        if grip:
            m_ = (extra_t['Left'] + extra_t['Right']) / 2
            extra_t = {'Left': m_, 'Right': m_}
        adj_use = {S: adj[S] + smooth(extra_t[S], 5) for S in ('Left', 'Right')}
        if trace is not None and _pass == 1:
            trace.update(raw={S: raw[S].copy() for S in raw}, ext=tr_ext, adj={S: adj[S].copy() for S in adj}, clear=tr_cl,
                         final={S: adj[S] + extra_t[S] for S in adj}, hand={S: out[:, ix[S + 'Hand'], :3, 3].copy() for S in adj},
                         s3=out[:, ix['Spine3'], :3, 3].copy(), mode=mode)
    Tw = intent['traj0']
    Wout = Tw[None, None] @ out
    Wout[:, 0] = np.eye(4)
    Wout[:, 1] = Tw
    info = dict(arm_reach_resid_max_cm=round(resid * 100, 3), leg_reach_resid_max_cm=round(leg_resid * 100, 3), grip=grip)
    if PEN_MODE == 'surface':
        info['surface_pushes'] = pen_stat['pushed']
    return Wout, info


GP_PASSES = 4            # round F game_path: chest-turn refinement passes (the hands no longer move away from the midline with the extension push)
GP_RAMP = 3              # round F game_path gate fallback: frames of the ramp from the v3b target back to the game path


def _gp_weights(bad, ramp=GP_RAMP):
    """0 on the frames in `bad` and their direct neighbours, then a linear ramp of `ramp` frames up to 1 (the game path)."""
    T = len(bad)
    idx = np.where(bad)[0]
    if not len(idx):
        return np.ones(T)
    dd = np.abs(np.arange(T)[:, None] - idx[None])
    if _PERIOD_T and T == _PERIOD_T:          # round F step 4: closed loop clip (LOOP_MODE 'periodic'): cyclic distance, period T-1
        dd = np.minimum(dd, (T - 1) - dd)
    dist = np.min(dd, axis=1).astype(float)
    return np.clip((dist - 1.0) / ramp, 0.0, 1.0)


def solve_refined(B, d, intent=None, passes=2, trace=None):
    """solve + chest-turn refinement: when a hand ends up past the body midline (anatomy gate hand_cross), the chest turns further towards it.
    Round F, intent mode 'game_path' (intent['mode']): GP_PASSES refinement passes; then a gate fallback: on frames where the game-path solve fails the base gate but
    the v3b solve (mode 'v3b') passes, the hand targets go back to the v3b targets (per-frame weight intent['gp_w'], 0 there, linear ramp of GP_RAMP frames), at
    most twice; info['gp_fallback_frames'] = frames with weight < 1.  (The gate decides what is legitimate; the deviation stays measurable by redo_intent.)"""
    intent = intent or plan_intent(B, d)
    if intent.get('mode') == 'game_path' and 'gp_w' not in intent:
        return _surface(B, d, intent, passes, trace) if PEN_MODE == 'surface' else _game_path(B, d, intent, passes, trace)
    W, info = solve(B, d, intent, trace=trace)
    for _ in range(passes - 1):
        ft = GB.AG.features(W, B.rg, step_frames=False)
        # right hand crossing to the left -> turn left (+yaw); left hand crossing to the right -> turn right (-yaw)
        extra = np.zeros(intent['T'])
        for k, sg in (('hand_cross_cm_L', -1.0), ('hand_cross_cm_R', 1.0)):
            e = np.maximum(ft[k] - 17.0, 0) / 100.0
            extra += sg * e / 0.45
        if np.abs(extra).max() < 1e-6:
            break
        intent = dict(intent)
        intent['yaw'] = np.clip(intent['yaw'] + smooth(extra[:, None], 3)[:, 0], np.radians(-60), np.radians(60))
        W, info = solve(B, d, intent, trace=trace)
    return W, info, intent


def _game_path(B, d, intent, passes, trace):
    """round F game_path pipeline: GP_PASSES chest-turn passes, then the gate fallback to the v3b targets (see solve_refined)."""
    al = GB.allowed()
    W1, info1, i1 = solve_refined(B, d, dict(intent, gp_w=np.ones(intent['T'])), passes=max(passes, GP_PASSES), trace=trace)
    g1 = GB.frame_gate(W1, B.rg, al=al)
    W0, _, i0 = solve_refined(B, d, dict(intent, mode='v3b'), passes=passes)
    g0 = GB.frame_gate(W0, B.rg, al=al)
    bad = g0['ok'] & ~g1['ok']
    info1['gp_fallback_frames'] = 0
    W, info, it_ = W1, info1, i1
    for _round in range(2):
        if not bad.any():
            break
        w = _gp_weights(bad)
        im = dict(intent, gp_w=w, yaw=w * i1['yaw'] + (1.0 - w) * i0['yaw'])
        W, info, it_ = solve_refined(B, d, im, passes=passes, trace=trace)
        info['gp_fallback_frames'] = int((w < 1.0 - 1e-9).sum())
        gm = GB.frame_gate(W, B.rg, al=al)
        bad = bad | (g0['ok'] & ~gm['ok'])
        if not (g0['ok'] & ~gm['ok']).any():
            break
    return W, info, it_


def clean_frames(B, W, al=None):
    """round F step 3: the base gate extended by the penetration screen -> (ok [T] = gate pass and no arm-through-torso frame (redo_pen proxy), gate ok [T], pen [T])."""
    g = GB.frame_gate(W, B.rg, al=al)
    _, pen = RPN.pen_frames(W)
    return g['ok'] & ~pen, g['ok'], pen


def _surface(B, d, intent, passes, trace):
    """round F step 3 (PEN_MODE 'surface'): the game_path pipeline with the torso-surface push, judged against the SAME pipeline with the axis rule only by the gate
    extended with the penetration screen (clean_frames): where the axis result is clean and the surface result is not, the surface push is switched off
    (intent['sp_w'] 0 there, ramp GP_RAMP), at most twice; the surface result is kept only if it has MORE clean frames than the axis result, else the axis
    result is returned (info['surface'] = 'reverted')."""
    global PEN_MODE
    al = GB.allowed()
    PEN_MODE = 'axis'
    try:
        Wa, info_a, ia = _game_path(B, d, intent, passes, None)
    finally:
        PEN_MODE = 'surface'
    ca, ga, pa = clean_frames(B, Wa, al)
    if not pa.any():        # no arm-through-torso frame with the axis rule: nothing for the surface push to fix (it could only lose clean frames)
        if trace is not None:
            PEN_MODE = 'axis'
            try:
                solve(B, d, ia, trace=trace)
            finally:
                PEN_MODE = 'surface'
        info_a['surface'] = dict(clean_axis=int(ca.sum()), gate_axis=int(ga.sum()), pen_axis=0, result='not needed (no penetration with the axis rule)')
        return Wa, info_a, ia
    W, info, it_ = _game_path(B, d, intent, passes, trace)
    cs, gs, ps = clean_frames(B, W, al)
    bad = ca & ~cs
    sw = np.ones(intent['T'])
    for _round in range(2):
        if not bad.any():
            break
        sw = np.minimum(sw, _gp_weights(bad))
        fbf = info.get('gp_fallback_frames', 0)
        W, info = solve(B, d, dict(it_, sp_w=sw), trace=trace)
        info['gp_fallback_frames'] = fbf
        cs, gs, ps = clean_frames(B, W, al)
        bad = ca & ~cs
    stat = dict(clean_axis=int(ca.sum()), clean_surface=int(cs.sum()), gate_axis=int(ga.sum()), gate_surface=int(gs.sum()), pen_axis=int(pa.sum()),
                pen_surface=int(ps.sum()), surface_off_frames=int((sw < 1.0 - 1e-9).sum()))
    if cs.sum() <= ca.sum():
        if trace is not None:
            PEN_MODE = 'axis'
            try:
                solve(B, d, ia, trace=trace)
            finally:
                PEN_MODE = 'surface'
        info_a['surface'] = dict(stat, result='reverted (not more clean frames than the axis rule)')
        return Wa, info_a, ia
    info['surface'] = dict(stat, result='applied')
    return W, info, dict(it_, sp_w=sw)

LOWER = ('Root', 'Trajectory', 'Hips', 'reference_joint')


def keep_mask(B, Wgame, ok):
    """game frames blend_keep may keep: the base gate passes (ok); round F PEN_MODE 'surface': and the game arms do not go through the torso (redo_pen proxy)."""
    if PEN_MODE != 'surface':
        return ok
    _, pen = RPN.pen_frames(Wgame)
    return ok & ~pen


def blend_keep(B, Wgame, Wredo, ok, margin=4):
    """Per-frame mix of the UPPER body: frames where the game pose passes the base gate stay as in the game, transitions (margin frames each side) are a slerp / lerp
    of the local transforms.  Hips, legs and feet are ALWAYS the redo (the game legs are straight and static: blending them would slide the feet).  -> W, weight_game [T]"""
    T = len(Wredo)
    w = ok.astype(float)
    if margin > 0 and 0 < w.sum() < T:
        k = np.ones(2 * margin + 1) / (2 * margin + 1)
        loop = LOOP_MODE == 'periodic' and closed_loop(Wgame)           # round F step 4: the ramp wraps around a closed loop (v3b: held ends)

        def pad(x):
            return np.concatenate([x[T - 1 - margin:T - 1], x, x[1:1 + margin]]) if loop and T > margin + 1 else np.pad(x, margin, mode='edge')
        core = np.convolve(pad(w), k, mode='valid')
        w = np.where(core > 1 - 1e-9, 1.0, 0.0)                       # eroded: whole neighbourhood passes
        w = np.convolve(pad(w), k, mode='valid')   # ramp
    elif w.sum() == 0:
        w = w * 0
    lower = np.array([n in LOWER or 'UpLeg' in n or n.endswith('Leg') or 'Foot' in n or 'Toe' in n or 'Heel' in n for n in B.names])
    out = Wredo.copy()
    for t in range(T):
        if w[t] <= 1e-6:
            continue
        La, Lb = B.locals_of(Wgame[t]), B.locals_of(Wredo[t])
        L = Lb.copy()
        for j in range(len(B.names)):
            if lower[j]:
                continue
            qa, qb = R.from_matrix(La[j, :3, :3]), R.from_matrix(Lb[j, :3, :3])
            rv = (qb.inv() * qa).as_rotvec()
            L[j, :3, :3] = (qb * R.from_rotvec(rv * w[t])).as_matrix()
            L[j, :3, 3] = Lb[j, :3, 3] * (1 - w[t]) + La[j, :3, 3] * w[t]
        out[t] = B.fk(L)
    return out, w


def main(a):
    s, c = a[0], a[1]
    out = a[a.index('--out') + 1] if '--out' in a else os.path.join(BB.REDO, 'clips')
    os.makedirs(out, exist_ok=True)
    B = Body()
    d = MC.load_dump(MC.clip_path(s, c))
    W, info = solve(B, d)
    p = os.path.join(out, '%s__%s.redo.npz' % (s, c))
    np.savez(p, W=W, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
    print('REDO', p, info)


if __name__ == '__main__':
    main(sys.argv[1:])
