"""Walk-away dismount v2 for a MOTORCYCLE get-off (exit_l / exit_r of the player sets): both sides, natural leg-over, two real steps with upper-body motion (offline, no game).

  python -I bike_exit_walk_away.py --glb <vanilla tpp_p?a_sportbike glb> --clip exit_l|exit_r --out <pick.anims.glb> [--report r.json]

Differences to bike_exit_walk.py (bike6, in-game review 2026-11-07: right knee came very high / folded to 165 deg (vanilla 121), V stood very close to the bike for a moment, arms / torso frozen during the steps):
  * the SWING leg (the one that goes over the seat: right for exit_l, left for exit_r) keeps the vanilla locals relative to the hips until it lands (the vanilla landing frame, found from its ankle height), so its knee flexion, knee height and
    path are the vanilla ones by construction (no foot target is dragged under the hips any more). Only the hips path differs: the vanilla Root path until --decel-frames before it would overstretch the pinned SUPPORT foot, then a constant
    deceleration to rest at hips = support plant + --hips-stop-dx (outwards). The swing foot lands where the vanilla hips-relative offset puts it (>= the support plant in the outward direction, so it never lands behind / beside the rear wheel),
    pinned (position + world rotation) from there;
  * SUPPORT foot (left for exit_l, right for exit_r): vanilla spot of frame 9, one re-place while seated (vanilla-like, hop h0..h1 with an arc) to the vanilla spot at the frame the Root starts moving, pinned until step 1;
  * steps: step 1 support foot (smooth, lift --apex-cm, zero speed at both ends) to the outer final spot, step 2 swing foot to the inner one; final hips = mean of the feet at Root = --root-end; Root path is outward-monotonic (hips never move back
    towards the bike: hips = outermost of the stopped hips and the mean of the pinned feet), so the capsule only moves away;
  * upper body in the steps (weights 0 up to frame n0-4, the vanilla last frame is held before): pelvis yaw +-A towards the stepping foot with Spine1 counter-rotation (net chest -0.5 x pelvis), small arm lag (both arms about the forward axis,
    opposite to the lateral hips acceleration), elbows untouched; all inside the vanilla joint ranges (check_bike_exit.py anatomy gate (i));
  * two-bone leg IK on the final hips (knee pole = natural knee until the landing / step 1, then in front of the hip), limb helpers re-anchored on every edited frame including the last; the arm IK chain
    below arm_<side>_ik (offset / weapon) keeps its vanilla locals so the game's hand IK targets stay on the moved hands (2026-11-08: the stale targets on the hips were the in-game "hand on hip").
Output: GLB with N frames (clip name unchanged), Root retimed -> build_bike_exit.ps1 -RootRetimed, check_bike_exit.py --root-retimed.
"""
import argparse
import json
import os
import re
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bike_exit_replant as R  # noqa: E402
import skel as S  # noqa: E402
import ik as IK  # noqa: E402
import build_entry_clip as B  # noqa: E402
import exit_steps as ES  # noqa: E402
import bike_steps_gen as BSG  # noqa: E402
from bike_exit_walk import ease_out_progress, segment  # noqa: E402

sm = R.smooth01
C_RG = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])   # RED -> glTF basis change (a rotation)


UB_NAMES = {'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand'}
ROOTD = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
PYK = os.path.join(ROOTD, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
BRIDGE = os.path.join(HERE, 'game_skeleton_bridge.py')


def load_sourcepack_dump(ref_dir, rig, clip):
    """world-matrix dump (73 joints, game axes) of a retargeted source pack clip on the V rig (game_skeleton_bridge dump-clip), cached in ref_dir"""
    dst = os.path.join(ref_dir, '%s_%s.npz' % (rig, clip))
    if not os.path.exists(dst):
        src = os.path.join(ROOTD, 'Animations', 'retarget', 'out_sourcepack_' + rig, clip + '.utppclip.json')
        subprocess.run([PYK, '-I', BRIDGE, 'dump-clip', src, dst], check=True, capture_output=True)
    return dict(np.load(dst, allow_pickle=True))


def dump_glb73(glb, clip):
    import tempfile
    out = os.path.join(tempfile.gettempdir(), 'w2dump_%d.npz' % os.getpid())
    subprocess.run([PYK, '-I', BRIDGE, 'dump-glb', glb, clip, out], check=True, capture_output=True)
    d = dict(np.load(out, allow_pickle=True))
    os.remove(out)
    return d


def rig_rest_73(tgt):
    """bind world matrices of the first 73 joints in game axes (as in the dumps)"""
    Wt, Wr = tgt.rest_world()
    out = np.zeros((73, 4, 4))
    Ci = C_RG
    for j in range(73):
        out[j, :3, :3] = Ci.T @ Wr[j] @ Ci
        out[j, :3, 3] = Ci.T @ Wt[j]
        out[j, 3, 3] = 1.0
    return out


def sourcepack_locals(d, tgt):
    """local rotations (glTF axes, quaternions as the skeleton stores them) of the 73 joints of a game-axes world dump, T x 73 x 4"""
    W = d['W'][:, :, :3, :3]
    par = [int(p) for p in d['parents']]
    P = d['W'][:, :, :3, 3]
    out = np.zeros((len(W), 73, 4))
    outt = np.zeros((len(W), 73, 3))
    for j in range(73):
        L = W[:, j] if par[j] < 0 else np.swapaxes(W[:, par[j]], -1, -2) @ W[:, j]
        Lg = C_RG @ L @ C_RG.T
        out[:, j] = S.m2q(Lg)
        if par[j] >= 0:
            outt[:, j] = (np.einsum('tji,tj->ti', W[:, par[j]], P[:, j] - P[:, par[j]])) @ C_RG.T
    return out, outt


def slerp_q(q0, q1, w):
    """quaternion arrays (T,4) in the skeleton's convention; w (T,) fraction"""
    r0, r1 = Rot.from_matrix(S.q2m(q0)), Rot.from_matrix(S.q2m(q1))
    rv = (r0.inv() * r1).as_rotvec() * np.asarray(w)[:, None]
    return S.m2q((r0 * Rot.from_rotvec(rv)).as_matrix())



def hinge_leg(tgt, lt, lr, a_, b_, c_):
    """make the knee a pure hinge: keep the knee / ankle positions the two-bone IK found, but rebuild the UpLeg / Leg local rotations so that the Leg rotates only about the rig's hinge axis
    (rest-relative local Z in game axes = y of the glTF rest-relative frame; anatomy_gate knee_dev = 0): the thigh twist is chosen so that the hinge axis is the normal of the hip-knee-ankle plane.
    Returns the largest ankle position error (m) and the largest hinge-axis / thigh-axis non-orthogonality (deg)."""
    Wt, Wr = tgt.world(lt, lr)
    A, K, C = Wt[:, a_], Wt[:, b_], Wt[:, c_]
    T = len(A)
    Lrest = S.q2m(tgt.rest_r[b_][None])[0]
    ey = np.array([0.0, 1.0, 0.0])
    h_u = Lrest @ ey
    v_t = lt[:, b_]
    e_t = v_t / np.linalg.norm(v_t, axis=1, keepdims=True)
    v_s = lt[:, c_]
    e_s = v_s / np.linalg.norm(v_s, axis=1, keepdims=True)
    orth = float(np.degrees(np.arcsin(np.clip(np.abs(e_t @ h_u).max(), 0, 1))))
    tw = K - A
    tw /= np.linalg.norm(tw, axis=1, keepdims=True)
    sh = C - K
    sh /= np.linalg.norm(sh, axis=1, keepdims=True)
    n = np.cross(tw, sh)
    nn = np.linalg.norm(n, axis=1, keepdims=True)
    # a (nearly) straight leg has no plane: keep the previous hinge direction (the IK pole), continuous in time
    bad = nn[:, 0] < 1e-4
    nprev = Wr[:, a_] @ h_u
    n = np.where(bad[:, None], nprev, n / np.maximum(nn, 1e-9))
    h_loc = h_u[None] - (h_u @ e_t.T)[:, None] * e_t
    h_loc /= np.linalg.norm(h_loc, axis=1, keepdims=True)
    Ru = np.zeros((T, 3, 3))
    zs = np.zeros(T)
    perr = 0.0
    P_hip = Wr[:, tgt.parent[a_]]
    for sgn in (1.0, -1.0):
        pass
    for f in range(T):
        best = None
        for sgn in (1.0, -1.0):
            Floc = np.stack([e_t[f], h_loc[f], np.cross(e_t[f], h_loc[f])], 1)
            nw = sgn * n[f]
            Fw = np.stack([tw[f], nw, np.cross(tw[f], nw)], 1)
            Ruf = Fw @ Floc.T
            q = Ruf.T @ sh[f]
            w = Lrest.T @ q
            u = e_s[f]
            z = np.arctan2(w[0], w[2]) - np.arctan2(u[0], u[2])
            z = (z + np.pi) % (2 * np.pi) - np.pi
            flex = -np.degrees(z)    # the sign convention of anatomy_gate.knee_flex (-rz of the game-axes Euler); in glTF axes the hinge axis maps to +y: checked by the gate itself
            cand = (flex, Ruf, z)
            if best is None or (cand[0] > 0 and best[0] <= 0) or (cand[0] > 0 and best[0] > 0 and False):
                best = cand
        Ru[f], zs[f] = best[1], best[2]
    # sign choice by continuity: prefer the branch whose knee flexion is positive (a knee bends one way); done above per frame
    c, s = np.cos(zs), np.sin(zs)
    Ry = np.zeros((T, 3, 3))
    Ry[:, 0, 0], Ry[:, 0, 2], Ry[:, 1, 1], Ry[:, 2, 0], Ry[:, 2, 2] = c, s, 1.0, -s, c
    Rleg = Lrest[None] @ Ry
    lr[:, a_] = S.m2q(np.transpose(P_hip, (0, 2, 1)) @ Ru)
    lr[:, b_] = S.m2q(Rleg)
    lr[:] = S.qcontinuous(lr)
    Wt2, _ = tgt.world(lt, lr)
    return float(np.linalg.norm(Wt2[:, c_] - C, axis=1).max()), orth


def red_rot_to_gltf(D):
    return C_RG @ D @ C_RG.T


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb', required=True)
    p.add_argument('--clip', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--report')
    p.add_argument('--root-end', type=float, default=0.80, help='m: final Root lateral offset (bike half width 0.35 + capsule 0.40 = 0.75 minimum)')
    p.add_argument('--hips-stop-dx', type=float, default=0.40, help='m: the hips stop this far outwards of the support plant (reach of the pinned support leg)')
    p.add_argument('--decel-frames', type=float, default=8.0, help='the Root decelerates to rest over about this many frames')
    p.add_argument('--hop', default='', help='support foot re-place h0:h1 (default: from the vanilla clip)')
    p.add_argument('--step1-frames', type=int, default=11)
    p.add_argument('--step2-frames', type=int, default=11)
    p.add_argument('--step3-frames', type=int, default=9)
    p.add_argument('--step-delay', type=float, default=0.18, help='fraction of a step at both ends in which the foot only lifts / lowers (horizontal motion finishes before touchdown)')
    p.add_argument('--early-pin', type=int, default=1, help='pin the feet where they first touch down in the first frames (support foot) / stamp (swing foot) - no early planted-phase slide')
    p.add_argument('--ub-ramp-frames', type=int, default=12)
    p.add_argument('--replace-all', type=int, default=0)
    p.add_argument('--end-rise', type=float, default=0.0)
    p.add_argument('--lead-outboard', type=int, default=0)
    p.add_argument('--gap1', type=float, default=0.10, help='m: step 1 ends this far inboard of the landed lead foot')
    p.add_argument('--min-width', type=float, default=0.18)
    p.add_argument('--pre-step', type=int, default=1, help='frames between the swing landing and step 1')
    p.add_argument('--ps-ahead', type=int, default=6, help='support plant = vanilla spot of frame f_root + this')
    p.add_argument('--ps-arrive', type=int, default=2, help='support foot hop ends at f_root + this')
    p.add_argument('--pole-ramp', type=int, default=8, help='support leg: frames from the end of its re-place hop over which the knee pole goes from the vanilla knee to straight ahead')
    p.add_argument('--knee-cap', type=float, default=146.0, help='deg: swing knee flexion limit (foot pushed away from the hip above it)')
    p.add_argument('--knee-cap-hard', type=int, default=0, help='1: the smoothed knee-cap push is never below the per-frame push the cap needs (the 5-tap smoothing otherwise dilutes a 2-frame peak: pma sportbike exit_r stays at 151.6 deg with --knee-cap 146); default off = bike8..10 behaviour')
    p.add_argument('--knee-cap-until', type=int, default=3, help='the knee-cap limiter is off from fp - this (the foot has to reach its pin); pma sportbike exit_r (2026-11-08): 1, the vanilla folds the knee to 151.6 deg in the two frames before the landing, '
                   'which the default leaves unlimited when the foot is pinned at the vanilla landing - 1 (--settle -1, the only pin frame that passes the 11 cm clearance proxy)')
    p.add_argument('--air-follow', type=float, default=0.0, help='1 = the airborne swing foot follows the lowered hips (hips-relative offset kept, foot lower than the vanilla world path); 0 = vanilla world height (seat clearance)')
    p.add_argument('--kappa-desc', type=float, default=0.0, help='kappa while the swing ankle descends past the bike side')
    p.add_argument('--land-blend', type=int, default=4, help='frames over which the swing foot target eases from the natural path into the pin')
    p.add_argument('--pole-out', type=float, default=0.0, help='support leg knee pole: sideways component (outwards) next to straight ahead (0.6): the knee follows the abducted thigh')
    p.add_argument('--kappa', type=float, default=0.6, help='1 = the swing foot keeps the vanilla WORLD path while the Root lags (hips-relative offset then differs: knee too folded)')
    p.add_argument('--settle', type=int, default=3, help='frames after the vanilla landing at which the swing foot is pinned')
    p.add_argument('--hold', type=int, default=12, help='frames held after step 2')
    p.add_argument('--apex-cm', type=float, default=7.0)
    p.add_argument('--ref-dir', default=os.path.join(ROOTD, 'Build', 'work', 'vehicle-bike-archive', 'bike7-20261107', 'ref'))
    p.add_argument('--ub-ramp', type=lambda s: tuple(int(x) for x in s.split(':')), default=None, help='f0:f1 vanilla -> natural upper body (default fc-8:fc+6)')
    p.add_argument('--deform-tol-deg', type=float, default=4.0, help='a group is replaced where a feature is more than this outside the source pack idle / walk / run envelope')
    p.add_argument('--deform-tol-cm', type=float, default=1.0)
    p.add_argument('--walk-amp', type=float, default=0.7)
    p.add_argument('--reach', type=float, default=0.985)
    p.add_argument('--max-extend', type=float, default=0.9998)
    p.add_argument('--ub-mode', default='sourcepack', choices=('sourcepack', 'game'), help="sourcepack (bike8/9): deformed upper-body groups replaced by the source pack idle from the Root start; game (bike10, in-game review 2026-11-08: the source pack arms swung out wide "
                   "while the leg went over the seat and hung stiff): the game's own upper body (the hands leave the grips as the vanilla clip animates it, clavicle translation at the rig rest) until the steps, then settled into the source pack idle "
                   "pose over --ub-settle (the pose MM shows next, so the 0.25 s hand-in has nothing to pull)")
    p.add_argument('--ub-settle', type=lambda s: tuple(int(x) for x in s.split(':')), default=None, help='game mode: f0:f1 of the settle blend (default: first step start : end of the steps + 4)')
    p.add_argument('--ub-settle-land', type=lambda s: tuple(int(x) for x in s.split(':')), default=None, help='game mode: settle blend as frames relative to the swing-foot landing fp (dL:dR, e.g. -6:8). The vanilla exits end with the hands on the hips, '
                   'elbows out (frames ~44..58 of every set); bike10 held that pose through both steps (in-game review 2026-11-08: V walks with her hands on her hips), so the arms now drop into the idle hang around the landing, before the steps, and swing with them')
    p.add_argument('--end-stance', default='vanilla', choices=('vanilla', 'idle'), help='idle: the final feet stand where the source pack idle has them relative to the hips (width / fore-aft, turned to the vanilla final facing), so the clip ends in the '
                   'stance MM continues with (no 20 cm foot re-step in the hand-in); vanilla: vanilla width at the vanilla y')
    a = p.parse_args()
    left = a.clip.endswith('_l')
    SU, SW = ('Left', 'Right') if left else ('Right', 'Left')
    out_sign = -1.0 if left else 1.0
    rig = R.Rig(a.glb, a.clip)
    tgt = rig.tgt
    n0 = rig.n
    ri = tgt.index['Root']
    hi = tgt.index['Hips']
    SF, WF = tgt.index[SU + 'Foot'], tgt.index[SW + 'Foot']
    WtR = R.red(rig.Wt0)
    # ---- key frames from the vanilla clip
    rx0 = rig.lt0[:, ri, 0].copy()
    prog = out_sign * rx0
    f_root = int(np.argmax(np.abs(rx0) > 0.02))               # the Root starts moving
    f_ps = f_root + a.ps_ahead                                  # the support foot is planted at its vanilla spot of this frame (a little further out than where the hips start moving: the leg reaches it while the swing foot is still on the peg)
    zgw = float(WtR[n0 - 1, WF, 2])
    zw = WtR[:, WF, 2]
    fc = int(n0 - 1 - np.argmax(zw[::-1] > zgw + 0.012)) + 1   # first frame the swing ankle is within 1.2 cm of its final height for good
    s9 = WtR[9, SF]
    mv = np.linalg.norm(WtR[9:, SF, :2] - s9[:2], axis=1) > 0.02
    h0 = 9 + int(np.argmax(mv)) - 1 if mv.any() else f_ps - 8
    h1 = f_root + a.ps_arrive                                  # ... and arrives there (hop end) while the swing foot is still on the peg
    h0 = max(8, min(h0, h1 - 8))
    if a.hop:
        h0, h1 = (int(x) for x in a.hop.split(':'))
    fp = fc + a.settle
    ls0 = fc + a.settle + a.pre_step                           # step 1: the trailing (support) foot closes up to the landed foot, staying >= --gap1 inboard of it (no crossing)
    ls1 = ls0 + a.step1_frames
    rs0, rs1 = ls1, ls1 + a.step2_frames                        # step 2: the lead (swing) foot walks on outwards
    s3a, s3b = rs1, rs1 + (a.step3_frames if a.lead_outboard else 0)                        # step 3: the trailing foot joins it
    N = s3b + a.hold
    t0, ramp = 6, 3
    TF, WTF = tgt.index[SU + 'ToeBase'], tgt.index[SW + 'ToeBase']

    def early_window(T):
        """touch-down frame and lift-off frame of a foot in the first 16 frames (vanilla: both feet stamp on the ground briefly before the swing foot goes back up on the peg); contact joint = toe"""
        zt = WtR[:16, T, 2]
        zm = float(zt.min())
        td = int(np.argmax(zt <= zm + 0.06))
        lift = td + 1 + int(np.argmax(zt[td + 1:] > zm + 0.09)) if (zt[td + 1:] > zm + 0.09).any() else 16
        return max(td, 1), lift
    td_s, lift_s = early_window(TF)
    td_w, lift_w = early_window(WTF)
    if a.early_pin:
        t0 = max(td_s, 2)
        ramp = min(3, t0 - 1)
    lt = np.concatenate([rig.lt0, np.repeat(rig.lt0[-1:], N - n0, 0)])
    lr = np.concatenate([rig.lr0, np.repeat(rig.lr0[-1:], N - n0, 0)])
    lt_v, lr_v = lt.copy(), lr.copy()
    times = np.arange(N) / 30.0
    P_S = WtR[f_ps, SF].copy()
    Rs_ref = rig.Wr0[f_ps, SF].copy()
    ref1 = t0 if a.early_pin else 9      # the support foot is pinned where it first touches down (vanilla frame 9 spot in the old variant)
    P_S1 = WtR[ref1, SF].copy()
    Rs_1 = rig.Wr0[ref1, SF].copy()
    R_fin_s = rig.Wr0[n0 - 1, SF].copy()
    R_fin_w = rig.Wr0[n0 - 1, WF].copy()
    zgs = float(WtR[n0 - 1, SF, 2])
    # ---- Root: vanilla, then a constant deceleration to rest where the hips are hips_stop_dx outwards of the support plant
    hips_local_x = float(WtR[n0 - 1, hi, 0] - rig.lt0[n0 - 1, ri, 0])
    hips_stop_x = P_S[0] + out_sign * a.hips_stop_dx
    R1 = out_sign * (hips_stop_x - hips_local_x)
    fs = f_root + 1
    for f in range(f_root + 1, n0):
        v = float(prog[f] - prog[f - 1])
        D = R1 - float(prog[f])
        if v > 1e-4 and D > 0 and 2.0 * D / v <= a.decel_frames:
            fs = f
            break
    v0 = float(prog[fs] - prog[fs - 1])
    p_new, f_stop = ease_out_progress(prog[fs], v0, fs, R1, np.arange(N, dtype=float))
    prog_ext = np.concatenate([prog, np.full(N - n0, prog[-1])])
    prog_new = np.where(np.arange(N) <= fs, prog_ext, p_new)
    rxn = out_sign * prog_new
    lt[:, ri, 0] = rxn
    Wt1, Wr1 = tgt.world(lt, lr)
    Wr = R.red(Wt1)
    ap = {s: (Wr[:, tgt.index[s + 'Foot']].copy(), Wr1[:, tgt.index[s + 'Foot']].copy()) for s in ('Left', 'Right')}
    # ---- final stance
    w_fin = max(abs(float(WtR[n0 - 1, WF, 0] - WtR[n0 - 1, SF, 0])), a.min_width)   # final stance width (vanilla value, >= --min-width); the LEAD (swing) foot ends OUTBOARD, the trailing one inboard: the lateral order of the feet never swaps
    root_end = max(a.root_end, R1 + 0.04)
    hips_end = out_sign * root_end + hips_local_x
    lo_ = 1.0 if a.lead_outboard else -1.0     # default: the SUPPORT foot ends outboard (the legs are not crossed in the final stance, as in vanilla); --lead-outboard: the swing foot stays outboard
    x_s3, x_w3 = hips_end - lo_ * out_sign * w_fin / 2.0, hips_end + lo_ * out_sign * w_fin / 2.0
    y_s3, y_w3 = float(WtR[n0 - 1, SF, 1]), float(WtR[n0 - 1, WF, 1])    # final feet at the vanilla final y (relative to the hips: straight legs, standing height)
    rigname = 'man' if 'pma' in os.path.basename(a.glb) else 'woman'
    if a.end_stance == 'idle':
        # the source pack idle stance (frame 0: left foot ~19 cm ahead, right ~10 cm behind the hips, 22 cm apart) relative to its hips; both the idle and the vanilla exit end face +y (lateral = x, the
        # convention of this whole generator; a facing estimate from the toe directions is off by up to 40 deg because of the asymmetric toe-out, so no turn is applied); the outboard idle foot takes the
        # outboard slot (support foot by default), the other one the inboard slot (= the same foot in all six sets); foot rotations stay the vanilla final ones
        gi0 = load_sourcepack_dump(a.ref_dir, rigname, 'M_Neutral_Stand_Idle_Loop')
        nm0 = [str(x) for x in gi0['names']]
        Pi = gi0['W'][0, :, :3, 3]
        ih = nm0.index('Hips')
        offs = {n_: (Pi[nm0.index(n_ + 'Foot')] - Pi[ih])[:2] for n_ in ('Left', 'Right')}
        outer = max(offs, key=lambda n_: out_sign * offs[n_][0])
        inner = 'Left' if outer == 'Right' else 'Right'
        o_sup, o_sw = (offs[outer], offs[inner]) if lo_ < 0 else (offs[inner], offs[outer])
        hy = float(WtR[n0 - 1, hi, 1])
        x_s3, y_s3 = hips_end + float(o_sup[0]), hy + float(o_sup[1])
        x_w3, y_w3 = hips_end + float(o_sw[0]), hy + float(o_sw[1])
        w_fin = abs(x_s3 - x_w3)
    S3 = np.array([x_s3, y_s3, zgs])
    W3x = x_w3
    # ---- support foot targets
    Tp, Tr = ap[SU][0].copy(), ap[SU][1].copy()
    Tp[t0:h0 + 1] = P_S1
    Tr[t0:h0 + 1] = Rs_1
    for f in range(t0 - ramp, t0):
        w = sm((f - (t0 - ramp) + 1) / float(ramp + 1))
        Tp[f, :2] = (1 - w) * ap[SU][0][f, :2] + w * P_S1[:2]
        Tr[f] = R.slerp_mats(ap[SU][1][f], Rs_1, w)
    for f in range(t0, t0 + 4):
        w = sm((f - t0 + 1) / 4.0)
        Tp[f, 2] = (1 - w) * ap[SU][0][f, 2] + w * P_S1[2]
    segment(Tp, Tr, h0, h1, P_S1, Rs_1, P_S, Rs_ref, apex=a.apex_cm / 100.0)
    Tp[h1:ls0 + 1] = P_S
    Tr[h1:ls0 + 1] = Rs_ref
    # ---- swing foot targets: the natural (vanilla locals + new Root) foot until the landing, pinned from there, step 2
    Rp, Rr = ap[SW][0].copy(), ap[SW][1].copy()
    nat = Rp.copy()
    rx_van = lt_v[:, ri, 0]
    lag_x = rx_van - rxn                                         # how far the new Root lags the vanilla one (signed x)
    z_hold = float(WtR[:, WF, 2][np.argmax(WtR[:, WF, 2] < -0.20) if (WtR[:, WF, 2] < -0.20).any() else fc])
    f_k0 = int(next(f for f in range(f_root, n0) if WtR[f, WF, 2] < -0.20 and f > f_root + 8))
    kap = a.kappa * (1.0 - sm((np.arange(N) - f_k0) / float(max(fp - f_k0, 1))))
    kap[:f_root] = 0.0
    # descent bump: while the swing ankle drops past the bike side (vanilla height -0.05 .. -0.35 after its peak) the lag is compensated more (kappa_desc), so the foot goes down outside the bike band
    pk = int(np.argmax(WtR[:n0, WF, 2]))
    zv = np.concatenate([WtR[:, WF, 2], np.full(N - n0, WtR[-1, WF, 2])])
    bump = ((np.arange(N) > pk) & (zv < -0.05) & (zv > -0.35)).astype(float)
    bump = np.convolve(np.pad(bump, 2, mode='edge'), np.array([1, 4, 6, 4, 1], float) / 16.0, mode='valid')
    kap = np.maximum(kap, a.kappa_desc * bump)
    nat[:, 0] = nat[:, 0] + kap * lag_x
    Rp = nat.copy()
    Rr_nat_early = ap[SW][1].copy()
    if a.early_pin and lift_w > td_w:
        # the swing foot stamps briefly early on (vanilla slides it 3-7 cm over the ground): pinned at its touch-down spot, released into the natural path over 4 frames before it lifts
        Pw = nat[td_w].copy()
        for f in range(td_w, lift_w + 7):
            w_ = 1.0 if f <= lift_w + 1 else 1.0 - sm((f - (lift_w + 1)) / 5.0)
            Rp[f, :2] = nat[f, :2] * (1 - w_) + Pw[:2] * w_
            Rr[f] = R.slerp_mats(Rr_nat_early[f], Rr_nat_early[td_w], w_)
    fp = fc + a.settle                                          # pin frame: the vanilla foot still shuffles a few cm when it arrives (a pinned phase must not move)
    P_W = np.array([nat[fp, 0], nat[fp, 1], zgw])
    R_land = ap[SW][1][fp].copy()
    fb0 = min(fc, fp) - a.land_blend
    for f in range(fb0, fp):
        w = sm((f - fb0 + 1) / float(fp - fb0 + 1))
        Rp[f] = (1 - w) * nat[f] + w * P_W
        Rr[f] = R.slerp_mats(ap[SW][1][f], R_land, w)
    Rp[fp:rs0 + 1] = P_W
    Rr[fp:rs0 + 1] = R_land
    W3 = np.array([W3x, y_w3, zgw])
    segment(Rp, Rr, rs0, rs1, P_W, R_land, W3, R_fin_w, apex=a.apex_cm / 100.0, delay=a.step_delay)
    Rp[rs1:] = W3
    Rr[rs1:] = R_fin_w
    # support foot: step 1 (closes up, inboard of the landed foot), hold during step 2, step 3 to the final spot
    o_ = lambda x: out_sign * x
    x_s1 = out_sign * min(o_(P_W[0]) - a.gap1, o_(S3[0])) if a.lead_outboard else S3[0]    # default: ONE passing step of the trailing foot straight to its final spot (the legs are crossed only between the landing and this step)
    S1 = np.array([x_s1, 0.5 * (P_S[1] + S3[1]), zgs])
    if not a.lead_outboard:
        S1 = S3.copy()
    segment(Tp, Tr, ls0, ls1, P_S, Rs_ref, S1, R_fin_s, apex=a.apex_cm / 100.0, delay=a.step_delay)
    Tp[ls1:s3a + 1] = S1
    Tr[ls1:s3a + 1] = R_fin_s
    if s3b > s3a:
        segment(Tp, Tr, s3a, s3b, S1, R_fin_s, S3, R_fin_s, apex=a.apex_cm / 100.0, delay=a.step_delay)
    Tp[s3b:] = S3
    Tr[s3b:] = R_fin_s
    # ---- hips: outermost of the stopped hips and the mean of the pinned feet from the landing on (outward-monotonic Root)
    hips_now = Wr[:, hi]
    mean_x = 0.5 * (Tp[:, 0] + Rp[:, 0])
    off = np.maximum(0.0, out_sign * (mean_x - hips_now[:, 0]) - 0.0)
    # the stopped hips x: after the deceleration hips_now is constant; the offset is what the mean is further out than the hips
    off = np.where(np.arange(N) >= fp, off, 0.0)
    ramp_in = np.clip((np.arange(N) - fp) / 6.0, 0, 1)
    off = off * ramp_in * ramp_in * (3 - 2 * ramp_in)
    k5 = np.array([1, 4, 6, 4, 1], float) / 16.0
    off = np.convolve(np.pad(off, 2, mode='edge'), k5, mode='valid')
    off[:fp] = 0.0
    lt[:, ri, 0] = rxn + out_sign * off
    # ---- upper body: the game poses stay wherever they are anatomically healthy; only the deformed fragments (per group and frame: spine / neck, each clavicle, each arm chain) are replaced by V's natural source pack body (idle),
    # then the source pack strafe-walk motion (a_t x the delta of the walk against the idle) is added on top of whatever the base is, so the arms / torso are not frozen during the steps.
    f_arr = np.arange(N)
    amp = a.walk_amp
    a_t = np.minimum(sm((f_arr - (ls0 - 8)) / 8.0), sm((s3b + 8 - f_arr) / 8.0)) * amp
    wclip = 'M_Neutral_Walk_Loop_LL' if left else 'M_Neutral_Walk_Loop_RR'
    gi, gw = load_sourcepack_dump(a.ref_dir, rigname, 'M_Neutral_Stand_Idle_Loop'), load_sourcepack_dump(a.ref_dir, rigname, wclip)
    names73 = [str(x) for x in gi['names']]
    if np.abs(gi['rest'] - rig_rest_73(tgt)).max() > 1e-3:
        print('WARNING: source pack dump rest differs from the clip rig', float(np.abs(gi['rest'] - rig_rest_73(tgt)).max()))
    sys.path.insert(0, os.path.join(ROOTD, 'Source', 'tools', 'melee'))
    import anatomy_gate as AG
    envj = json.load(open(os.path.join(a.ref_dir, 'gasp_env_%s.json' % rigname)))
    dv_ = dump_glb73(a.glb, a.clip)
    fv_ = AG.features(np.concatenate([dv_['W'], np.repeat(dv_['W'][-1:], N - n0, 0)]), AG.rig_of(dv_))
    groups = {'spine': [], 'clavL': [], 'clavR': [], 'armL': [], 'armR': []}
    for k_ in fv_:
        if k_ not in envj or k_.startswith(('knee_', 'hip_', 'shin_', 'foot_')) or k_ in ('rot_step', 'hips_drop_cm', 'seg_len_dev_pct', 'hand_eye_cm'):
            continue
        if k_.startswith(('spine_', 'neck_', 'torso_lean')):
            groups['spine'].append(k_)
        elif k_ == 'shoulder_width_cm':
            groups['clavL'].append(k_)
            groups['clavR'].append(k_)
        elif k_.startswith('clav_'):
            groups['clav' + ('L' if '_L' in k_ else 'R')].append(k_)
        else:
            groups['arm' + ('L' if k_.endswith(('_L', '_cm_L', '_L_cm')) else 'R')].append(k_)
    bones_of = {'spine': ['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head'], 'clavL': ['LeftShoulder'], 'clavR': ['RightShoulder']}
    for sd in ('Left', 'Right'):
        bones_of['arm' + sd[0]] = [sd + x for x in ('Arm', 'ForeArm', 'Hand')] + [n for n in names73 if re.fullmatch(sd + r'Hand(Thumb|Index|Middle|Ring|Pinky)[123]', n)]
    tolc = lambda k: a.deform_tol_cm if '_cm' in k else a.deform_tol_deg
    k5 = np.array([1, 4, 6, 4, 1], float) / 16.0
    w_g = {}
    rep_ub = {}
    if a.ub_settle:
        set0, set1 = a.ub_settle
    elif a.ub_settle_land is not None:
        set0, set1 = fp + a.ub_settle_land[0], fp + a.ub_settle_land[1]   # bike11: out of the vanilla hands-on-hips pose around the landing, before the steps
    else:
        set0, set1 = ls0, min(s3b + 4, N - 4)                              # bike10: during the steps (held the vanilla hands-on-hips pose while walking)
    set0 = int(max(set0, f_root + 1))
    set1 = int(min(max(set1, set0 + 1), N - 4))
    w_set = sm((f_arr - set0) / float(max(set1 - set0, 1)))              # game mode: vanilla upper body -> source pack idle pose
    w_clav = sm((f_arr - f_root) / float(a.ub_ramp_frames))               # game mode: clavicle translation -> rig rest from the Root start (the vanilla exits narrow the shoulders by ~10 cm)
    for g_, keys in groups.items():
        ind = np.zeros(N)
        for k_ in keys:
            v = np.asarray(fv_[k_])
            ind = np.maximum(ind, ((v < envj[k_][0] - tolc(k_)) | (v > envj[k_][3] + tolc(k_))).astype(float))
        if a.replace_all:
            ind[:] = 1.0       # arms / clavicles follow a replaced chest (a healthy vanilla arm on a replaced torso can end up crossing the midline)
        ind[:f_root] = 0.0   # the hands are on the grips until the Root moves: riding pose, never touched
        dil = np.maximum.reduce([np.roll(ind, i) for i in range(-3, 4)])
        dil[:f_root] = 0.0
        k9 = np.ones(9) / 9.0
        w = np.convolve(np.pad(dil, 4, mode='edge'), k9, mode='valid')
        w = np.convolve(np.pad(w, 4, mode='edge'), k9, mode='valid')
        w = w * sm((f_arr - f_root) / float(a.ub_ramp_frames))
        if a.ub_mode == 'game':
            w = w_set.copy()
        w_g[g_] = np.clip(w, 0, 1)
        rep_ub[g_] = [int(x) for x in np.flatnonzero(w_g[g_] > 0.5)]
    p0 = 7
    gidx = (p0 + np.round((f_arr - ls0) * (28.0 / float(s3b - ls0))).astype(int)) % len(gw['W'])
    (loc_i, lt_i), (loc_w, lt_w) = sourcepack_locals(gi, tgt), sourcepack_locals(gw, tgt)
    for g_, bl in bones_of.items():
        for n_ in bl:
            j = tgt.index[n_]
            k_ = names73.index(n_)
            q_idle = np.repeat(loc_i[:1, k_], N, 0)
            base = slerp_q(lr[:, j], q_idle, w_g[g_])
            # walk delta against the idle: base * (idle^-1 walk)^a
            r_i, r_w = Rot.from_matrix(S.q2m(q_idle)), Rot.from_matrix(S.q2m(loc_w[gidx, k_]))
            rv = (r_i.inv() * r_w).as_rotvec() * a_t[:, None]
            lr[:, j] = S.m2q((Rot.from_matrix(S.q2m(base)) * Rot.from_rotvec(rv)).as_matrix())
            wt_ = w_clav if a.ub_mode == 'game' else w_g[g_]
            lt[:, j] = lt[:, j] * (1 - wt_[:, None]) + tgt.rest_t[j][None] * wt_[:, None]   # the rig's own rest offset (the vanilla exits animate the clavicle translation: shoulders 10 cm too narrow)
    rep_ub_summary = {g_: ([min(v), max(v)] if v else []) for g_, v in rep_ub.items()}
    # pelvis sway of the same walk (world rotation delta of the Hips against the idle), fraction a_t x 0.8
    Wh_i = gi['W'][0, names73.index('Hips'), :3, :3]
    Dh = gw['W'][gidx, names73.index('Hips'), :3, :3] @ Wh_i.T
    rv = Rot.from_matrix(Dh).as_rotvec() * (a_t * 0.8)[:, None]
    Dhips = Rot.from_rotvec(rv).as_matrix()
    _, Wr_ = tgt.world(lt, lr)
    Dg = red_rot_to_gltf(Dhips)
    hb = hi
    loc = np.transpose(Wr_[:, tgt.parent[hb]], (0, 2, 1)) @ (Dg @ Wr_[:, hb])
    m_ = a_t > 1e-4
    lr[m_, hb] = S.m2q(loc)[m_]
    # (the hips delta rotates the children with it: re-apply the natural upper-body locals is not needed, they are local)
    # ---- hips lowering where a leg would overstretch
    Wt2, Wr2 = tgt.world(lt, lr)
    W2 = R.red(Wt2)
    uleg = {s: W2[:, tgt.index[s + 'UpLeg']] for s in ('Left', 'Right')}
    Lleg = float(np.linalg.norm(rig.Wt0[0, tgt.index['LeftLeg']] - rig.Wt0[0, tgt.index['LeftUpLeg']]) + np.linalg.norm(rig.Wt0[0, tgt.index['LeftFoot']] - rig.Wt0[0, tgt.index['LeftLeg']]))
    need = np.zeros(N)
    reach_t = a.reach + (0.9995 - a.reach) * sm((f_arr - (rs1 - 4)) / float(max(s3b - rs1 + 4, 1)))   # standing height again after the steps: the last frames use the (almost) full leg length (vanilla ends on straight legs)
    for s_, T_ in ((SU, Tp), (SW, Rp)):
        d = uleg[s_] - T_
        h = np.hypot(d[:, 0], d[:, 1])
        vmax = np.sqrt(np.maximum((reach_t * Lleg) ** 2 - h ** 2, 0.0))
        need = np.maximum(need, np.maximum(d[:, 2] - vmax, 0.0))
    need[:f_root] = 0.0
    kk = np.ones(7) / 7.0
    nm = np.maximum.reduce([np.roll(need, i) for i in (-3, -2, -1, 0, 1, 2, 3)])
    need_s = np.convolve(np.pad(nm, 3, mode='edge'), kk, mode='valid')
    need_s[0] = 0.0
    lt[:, hi, 1] -= need_s
    if a.end_rise:
        lt[:, hi, 1] += a.end_rise / 100.0 * sm((f_arr - (rs1 - 8)) / 8.0)    # standing a little taller at the end (cm, glTF y = up): the vanilla last frame of some sets (sb2) has bent knees; the rise ends with step 2 (2026-11-08: the last 10 frames gate (j) reads are static)
    rep = dict(clip=a.clip, support=SU, swing=SW, frames=N, vanilla_frames=n0, f_root=f_root, f_ps=f_ps, hop=[h0, h1], fs=fs, f_stop=round(float(f_stop), 1), fc=fc, steps=[ls0, ls1, rs0, rs1, s3a, s3b], fp=fp,
               R1=float(R1), root_end=float(root_end), hips_lowering_max_cm=round(float(need_s.max()) * 100, 1), support_plant=[float(x) for x in P_S], swing_plant=[float(x) for x in P_W],
               final_support=[float(x) for x in S3], final_swing=[float(x) for x in W3], ub_ramp=([int(set0), int(set1)] if a.ub_mode == 'game' else [int(f_root), int(f_root) + int(a.ub_ramp_frames) + 2]),
               replaced_frames=rep_ub_summary, walk_amp=a.walk_amp, sourcepack_walk=wclip, ub_mode=a.ub_mode, end_stance=a.end_stance, final_width_cm=round(float(w_fin) * 100, 1))
    # swing foot target follows the lowered hips in the air (hips-relative offset stays the vanilla one), tapering to 0 at the landing
    air = 1.0 - sm((f_arr - (fc - 8)) / 6.0)
    air[fc:] = 0.0
    Rp[:, 2] -= need_s * air * a.air_follow
    # knee fold limiter of the swing leg: the foot is pushed away from the hip joint where the hip-ankle distance would fold the knee beyond --knee-cap (the vanilla exit_r folds it to 150+ deg, human limit ~150)
    L1 = float(np.linalg.norm(rig.Wt0[0, tgt.index[SW + 'Leg']] - rig.Wt0[0, tgt.index[SW + 'UpLeg']]))
    L2 = float(np.linalg.norm(rig.Wt0[0, tgt.index[SW + 'Foot']] - rig.Wt0[0, tgt.index[SW + 'Leg']]))
    d_min = float(np.sqrt(L1 * L1 + L2 * L2 - 2 * L1 * L2 * np.cos(np.radians(180.0 - a.knee_cap))))
    hip_sw = uleg[SW].copy()
    hip_sw[:, 2] -= need_s
    dv_ = Rp - hip_sw
    dist = np.linalg.norm(dv_, axis=1)
    scale = np.where(dist < d_min, d_min / np.maximum(dist, 1e-6), 1.0)
    scale[fp - a.knee_cap_until:] = 1.0
    raw_scale = scale.copy()
    scale = np.convolve(np.pad(scale, 2, mode='edge'), np.array([1, 4, 6, 4, 1], float) / 16.0, mode='valid')
    if a.knee_cap_hard:
        scale = np.maximum(scale, raw_scale)
    scale = np.maximum(scale, 1.0) * (scale > 1.0005) + 1.0 * (scale <= 1.0005)
    Rp = hip_sw + dv_ * scale[:, None]
    rep['knee_cap_push_max_cm'] = round(float(((scale - 1.0) * dist).max()) * 100, 1)
    # ---- leg IK on the final hips
    Wt3, Wr3 = tgt.world(lt, lr)
    for sd, (T_, R_) in ((SU, (Tp, Tr)), (SW, (Rp, Rr))):
        a_, b_, c_ = (tgt.index[sd + x] for x in ('UpLeg', 'Leg', 'Foot'))
        w0_, w1_ = (fp - 2, fp + 4) if sd == SW else (h1, h1 + a.pole_ramp)
        w_ = sm((np.arange(N) - w0_) / float(w1_ - w0_))[:, None]
        pole_fwd = Wt3[:, a_] + R.gltf(np.array([(out_sign * a.pole_out) if sd == SU else 0.0, 0.6, 0.0]))
        err = IK.two_bone(tgt, lt, lr, a_, b_, c_, R.gltf(T_), max_extend=a.max_extend, bend_sigma=2.0, pole=(1 - w_) * Wt3[:, b_] + w_ * pole_fwd)
        herr, horth = hinge_leg(tgt, lt, lr, a_, b_, c_)
        rep['hinge_ankle_err_cm_' + sd] = round(herr * 100, 2)
        rep['hinge_orth_deg_' + sd] = round(horth, 2)
        IK.set_world_rot(tgt, lt, lr, c_, R_)
        rep['ik_max_err_cm_' + sd] = round(float(err) * 100, 2)
    keep = np.zeros(N, bool)
    keep[:t0 - ramp] = True
    lt[keep, :], lr[keep, :] = lt_v[keep], lr_v[keep]
    lt[keep, ri, 0] = lt[keep, ri, 0]
    vWt = np.concatenate([rig.Wt0, np.repeat(rig.Wt0[-1:], N - n0, 0)])
    vWr = np.concatenate([rig.Wr0, np.repeat(rig.Wr0[-1:], N - n0, 0)])
    # helpers on every edited frame INCLUDING the last one (bike8..11 stopped at N-2: the last frame - the pose the game holds / hands in from - kept the vanilla hand-IK targets, i.e. the hands on the hips)
    ES.reanchor_helpers(tgt, lt, lr, vWt, vWr, t0 - ramp - 1, N)
    R.place_leg_poles(tgt, lt, lr, type('RigV', (), dict(Wt0=vWt, Wr0=vWr))(), ['Left', 'Right'], t0 - ramp, N - 1)
    # the arm IK chain below arm_<side>_ik keeps the vanilla LOCALS (arm_*_ik_offset = identity on arm_*_ik, arm_*_ik_weapon = the animated grip offset). reanchor_helpers keeps the WORLD pose of a
    # helper's descendants, which left arm_left_ik_offset / _weapon at the vanilla hand position (on the hip from frame ~44 on) 15-25 cm away from the moved hand; the game's left-hand IK pulled the hand
    # there (bike8..11 in-game review 2026-11-08: V walks away with the left hand on her hip although the clip's arm hangs)
    vlt = np.concatenate([rig.lt0, np.repeat(rig.lt0[-1:], N - n0, 0)])
    vlr = np.concatenate([rig.lr0, np.repeat(rig.lr0[-1:], N - n0, 0)])
    helper_fix = []
    for sd_ in ('left', 'right'):
        for nm_ in ('arm_%s_ik_offset' % sd_, 'arm_%s_ik_weapon' % sd_):
            if nm_ in tgt.index:
                j_ = tgt.index[nm_]
                lt[:, j_], lr[:, j_] = vlt[:, j_], vlr[:, j_]
                helper_fix.append(nm_)
    rep['arm_ik_chain_vanilla_locals'] = helper_fix
    lr[:] = S.qcontinuous(lr)
    rep['first_frames_identical'] = bool(np.all(lt[:t0 - ramp] == lt_v[:t0 - ramp]))
    B.write_out(tgt, a.out, a.clip, times, lt, lr, rig.extras)
    # ---- offline numbers
    Wc, nm_ = BSG.load('%s:%s' % (a.out, a.clip))
    fl = BSG.floors_of(Wc, nm_, robust=True)
    rep['phases'] = {s: BSG.contact_phases(Wc, nm_, s, fl) for s in ('Left', 'Right')}
    rep['glide_cm'] = BSG.glide(Wc, nm_, None, fl, robust=True)['glide_total_cm']
    rep['phase_net_max_cm'] = max([q['net_cm'] for v in rep['phases'].values() for q in v] or [0.0])
    ix = {k_: i for i, k_ in enumerate(nm_)}
    rep['root_x_end'] = float(Wc[-1, ix['Root'], 0, 3])
    for j in ('LeftFoot', 'RightFoot', 'LeftLeg', 'RightLeg', 'Hips'):
        rep['min_abs_x_%s_cm' % j] = round(float(np.abs(Wc[n0 - 20:, ix[j], 0, 3]).min()) * 100, 1)
    if a.report:
        json.dump(rep, open(a.report, 'w'), indent=1, default=str)
    print(json.dumps({k_: v for k_, v in rep.items() if k_ != 'phases'}, default=str)[:1800])
    print('PHASES', {s: [(q['f0'], q['f1'], q['net_cm']) for q in v] for s, v in rep['phases'].items()})


if __name__ == '__main__':
    main()
