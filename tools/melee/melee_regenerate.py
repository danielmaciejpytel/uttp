"""MotionBricks stage on top of the melee redo (v3b): whole-body motion (hips, spine, neck) between the key poses of the v3b clip, arms stay v3b.

  python -I melee_regenerate.py <set> <clip> [--seeds 0,7,11] [--out DIR] [--no-mb] [--demo-dir DIR] [--max-comp 1.7] [--rig man] [--keep-tmp]
  UTPP_RIG=woman|man (or --rig) selects the rig; every rig-dependent call goes through melee_common.RIG_OF_SET / UTPP_RIG (one process per rig).
  Outputs per rig: MB4 = WORK\\mb4 (woman) / WORK\\mb4_man (man); default --out = MB4\\clips, work dirs MB4\\_tmp.
Pipeline (details and deviations in the docstrings below):
  1 V3      the v3b clip exactly as redo_demo.py makes it (plan_intent, solve_refined, blend_keep) -> <out>\\<set>__<clip>.v3.npz
  2 keys    frame 0 and T-1, phase boundaries of manifest.json (startup / active / recover) or motion start / speed peak / motion end of the strike
            hand; merged so that every segment has D >= MIN_SEG frames, at most MAX_SEGS segments; gaps longer than MAX_LEN (30) frames are split evenly
  3 scenes  per segment (c, g) a SOMA scene (game_skeleton_bridge fwd of V3, context c-3..c, target g..g+3, gap 4N-7, N = clamp(round((D+7)/4), 6, 16));
            a segment goes to MotionBricks only if gap / D <= max-comp (and D / gap <= max-comp), else it stays V3 ('short' / 'too_long')
            ONE segment_generator.py run (one engine start) for all segments x seeds of all clips of the call; seed 0 = exact context (segment_generator seed -1),
            other seeds = jittered context history (jitter 1.5 cm / 1.5 deg)
  4 splice  the generated span (4N-6 frames) is resampled to D+1 frames (slerp / lerp), first / last forced to the V3 key frames; the SOMA result of
            the interior frames goes through game_skeleton_bridge inv (in process) and is spliced IN GAME SPACE: only the interior frames c+1..g-1
            of an accepted segment differ from V3, key frames / fallback segments / --no-mb are V3 bit for bit. Of the MotionBricks output only
            Hips + Spine..Head (rotations, hips translation; a smooth offset makes the round trip exact at both keys) is used.
  5 arms    LOCAL lock: LeftShoulder / RightShoulder subtrees (clavicle, arm, forearm, hand, fingers, weapon helpers) keep the V3 LOCAL transforms,
            so they are attached to the generated chest with the V3 hand-to-chest transforms (tested at 1e-6); eyes, heels, toes, helpers: V3 locals
  6 legs    two-bone hinge IK (redo_solve.two_bone) from the MotionBricks hips to the V3 foot world transforms: feet planted exactly as V3 (slide 0,
            z unchanged), the knee follows the hips; the raw MotionBricks legs (before pinning) are only measured (slide before)
  7 screen  per segment x seed x weight beta on the whole assembled clip.  beta = weight of the MotionBricks body motion (BETAS 1.0 / 0.65 / 0.4 / 0.2: slerp / lerp of the
            Hips + Spine..Head locals towards V3): MotionBricks does not know the strike, between two keys its chest turn differs from V3's by up to 90 deg and the
            arms ride on the chest, so the largest beta that passes wins.  Acceptance (a candidate must pass ALL): anatomy_gate_base.frame_gate pass frames over the
            window >= V3's and no feature violated on MORE frames than V3; hips height within 3 cm of V3, knee flex <= V3 + 10 deg, leg IK residual <= 0.5 cm,
            max joint acceleration <= 1.25 x V3 + 0.5 cm/frame^2, chest (Spine3) orientation <= 25 deg and hand world position <= 12 cm from V3, plus the screens of
            the E round below.  Among the passing candidates of the largest beta the lowest score wins; none passes -> the segment stays V3 ('fallback').
            No gate threshold is weakened; the extra screens only restrict MotionBricks further.
Fix round E (2026-11-09, after the independent review review-20261109\\review.txt; findings H1, M1, L1, L2, L4, L6, L7):
  F1 legs (H1)  build_span: knee pole = the V3 thigh direction (hip -> knee of V3, NOT turned with the pelvis delta) and no two_bone continuity memo
                (memo=None).  With the memo a nearly straight leg weighed the pole |pole_perp| / 0.5 m ~ 0.2, the bend side (and the thigh twist) lagged
                behind V3 and snapped back on the end key, which is V3 itself (up to 24 deg in one frame).  New screens (continuity()): (a) hip-to-ankle
                distance <= REACH_MAX (0.985) x (thigh + shin) on every interior frame (V3 max 0.975 in both rigs); (b) knee flexion >= max(8 deg, V3 - 10 deg)
                per interior frame (V3 min 25.5 deg); (c) knee bend side (knee offset about the hip -> ankle axis) within 15 deg of V3; (d) lower-body
                acceleration (Hips, UpLeg, Leg, Foot) <= max(2.0, 1.25 x V3's lower-body max over the window + 0.5) cm/frame^2 on every frame c..g incl. the keys;
                (e) key continuity: UpLeg local-rotation step out of the start key / into the end key <= max(V3's step there, the largest step inside the span) + 1 deg
                and the knee (Leg joint) position step <= V3's + 1 cm.
  F2 joins (M1) the MotionBricks delta (chain rotations and hips translation) is eased in after the start key and out before the end key: per-frame weight
                beta x smoothstep(min(t, D - t) / (EASE_FRAMES + 1)) (0 on the keys, 1 from the 4th interior frame on), also at beta 1.0.  Velocity-change
                screens at the keys: hips |dv| <= max(V3's at that key, the largest inner |dv|) + 1 cm/frame; Spine1..Head local angular-velocity change <=
                max(V3's, the largest inner) + 2 deg/frame.  The lower-body acceleration and the key screens run in the join check (window_problems) as well,
                and the score adds 2 x lower-body accel max + 2 x the hips kink at the keys (it was dominated by the strike hands).
  F3 magnitude (L1)  per frame and feature the V4 gate violation must not be deeper than V3's at that frame (+1e-6); the frame-count rules stay; the knee screen
                is also applied per frame (knee flex <= V3 at the same frame + 10 deg), not only as a window maximum.
  F4 hips frame (L2) mb_chain expresses the MotionBricks hips relative to the V3 rig parent of Hips (Root, identity in every clip), not to the V3 Trajectory joint
                (a fixed yaw of 48.6 deg in default_melee_weapon fall_attack* / slide_attack*, 12.4 deg in pwa_knife melee_sprint_loop rotated the hips delta).
  F5 paths (L4) MB4 per rig (mb4 / mb4_man); melee_regenerate_batch writes batch_report.json next to its --out dir.
  F6 keys (L6)  gaps longer than MAX_LEN = 30 frames are split evenly (a 65-frame clip was one span); clips without manifest phases: a speed-peak / motion-edge
                candidate closer than MIN_SEG to a chosen key is moved by up to KEY_NUDGE = 3 frames instead of being dropped.
  F7 join check (L7) the demotion loop is bounded by the total candidate count + 1 (each round demotes >= 1 candidate, so it always converges); adjacent flagged
                segments demote only the worse-scored one per round (a shared key flags both); iterations / convergence are written to the info json.
Round F (2026-11-09, recover the F3 fallbacks):
  F3' magnitude tolerance  MAG_MODE 'unit_tol' (default; 'strict' = round E, --mag strict): a (frame, feature) pair that ALREADY violates in V3 may deepen by
                MAG_TOL[unit] (+0.5 cm distances, +1.0 deg angles, +1.0 deg/frame rot_step, 0 for the bone-length pct: lengths never change in this stage);
                a pair clean in V3 stays strict (+MAG_EPS).  New explicit rule in both modes: a frame that passes the gate in V3 must pass in V4 ('new failing
                frames').  Count rules, F1 / F2 / F4 screens and the per-frame knee bound unchanged.  Units: feature_unit() from anatomy_gate.LIM kinds.
  Screens (round F steps 2 / 3; SCREENS, --screens pen,intent | none; default both): 'intent' = per interior frame and hand the strike-intent deviation of the
                assembled clip (redo_intent.dev: hand - Spine3 vs the scaled game path, game axes) <= V3's + INTENT_TOL_CM (the arms ride on the generated chest,
                so a chest rotation turns the strike); 'pen' = per interior frame the arm-through-torso proxy count (redo_pen) <= max(threshold, V3's count).
                Both run in evaluate() and in the join check (window_problems); --screens none = the step-1 stage (mb5 / mb5i).
  Paths: UTPP_MB_ROOT overrides MB4 (tests / demo / default --out); run_clips puts its scratch work dirs next to its out dir (<dirname(out)>\\_tmp), so a run
                into mb5\\clips never touches mb4.
Outputs <out>\\<set>__<clip>.v4.npz (W, rest, names, parents, fps = dump layout), .v4.info.json (with 'settings', 'join_check', 'legs'), a REDO_MB line.
"""
import glob
import json
import os
import shutil
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
if '--rig' in sys.argv:     # must happen before base_body / game_skeleton_bridge are imported (they read UTPP_RIG once)
    _i = sys.argv.index('--rig')
    os.environ['UTPP_RIG'] = sys.argv[_i + 1]
    del sys.argv[_i:_i + 2]
os.environ.setdefault('UTPP_RIG', 'woman')
RIG = os.environ['UTPP_RIG']

import melee_common as MC  # noqa: E402
import base_body as BB  # noqa: E402  (imports game_skeleton_bridge with the same rig)
import anatomy_gate as AG  # noqa: E402
import anatomy_gate_base as GB  # noqa: E402
import game_skeleton_bridge as CP  # noqa: E402
import melee_keyposes as MK  # noqa: E402
import redo_solve as RS  # noqa: E402
import redo_intent as RI  # noqa: E402
import redo_pen as RPN  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402
from scipy.spatial.transform import Slerp  # noqa: E402

assert CP.RIG_NAME == RIG
TPOSE = MK.TPOSE
MB4 = os.environ.get('UTPP_MB_ROOT') or os.path.join(MC.WORK, 'mb4' if RIG == 'woman' else 'mb4_man')     # F5: one output root per rig (round F: env override)
MIN_SEG = 10            # frames: shorter segments stay V3 (MotionBricks gap >= 17 frames would squeeze them > 1.7x)
MAX_SEGS = 6
MAX_LEN = 30            # F6: longer gaps between keys are split evenly
KEY_NUDGE = 3           # F6: clips without manifest phases: a key candidate too close to a chosen key may move by up to this many frames
MAX_COMP = 1.7
HIPS_Z_CM = 3.0
KNEE_PLUS_DEG = 10.0
IK_RESID_CM = 0.5
ACC_RATIO, ACC_ABS = 1.25, 0.5
CHEST_DEV_DEG = 25.0    # Spine3 / head orientation vs V3 inside a generated span (the arms ride on the chest)
HAND_DEV_CM = 12.0      # hand world position vs V3
JITTER_CM = 1.5
BETAS = (1.0, 0.65, 0.4, 0.2)   # weight of the MotionBricks body motion: the largest weight that passes every screen wins
CHAIN = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head']
UPCHAIN = ['Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head']          # the part the in-game melee mask takes from the clip
LOWER = ['Hips', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot']
EASE_FRAMES = 3         # F2: smoothstep ramp of the MotionBricks delta over the first / last 3 interior frames
REACH_MAX = 0.985       # F1a: hip-to-ankle distance / (thigh + shin)
KNEE_MIN_DEG, KNEE_DROP_DEG = 8.0, 10.0     # F1b: knee flexion >= max(8, V3 - 10) deg
KNEE_SIDE_DEG = 15.0    # F1c: knee bend-side deviation from V3
LOW_ACC_ABS = 2.0       # F1d: lower-body acceleration floor (cm / frame^2); bound = max(2.0, ACC_RATIO x V3 + ACC_ABS)
LEG_STEP_DEG, KNEE_STEP_CM = 1.0, 1.0       # F1e: margins of the UpLeg rotation step / knee position step at the keys
HIPS_KINK_CM, CHAIN_KINK_DEG = 1.0, 2.0     # F2: margins of the hips velocity change / Spine1..Head angular-velocity change at the keys
MAG_EPS = 1e-6          # F3: a V4 gate violation may not be deeper than V3's at the same frame
SCORE_LOW_W, SCORE_KINK_W = 2.0, 2.0        # F2: lower-body accel and hips kink in the selection score
MAG_MODE = 'unit_tol'   # F3' (round F): 'unit_tol' = per-unit tolerance on pairs already violated in V3; 'strict' = round E (every pair <= V3 + MAG_EPS)
MAG_TOL = {'cm': 0.5, 'deg': 1.0, 'deg/frame': 1.0, 'pct': 0.0}     # extra depth allowed per unit (pct: bone length, never changed by this stage -> strict)
UNIT_OF_KIND = {'ang': 'deg', 'cm': 'cm', 'pct': 'pct', 'step': 'deg/frame'}
EXTRA_UNITS = {'clav_gap_cm': 'cm'}          # gate features that are not in anatomy_gate.LIM (anatomy_gate_base extras)
SCREENS = ('intent', 'pen')     # round F: extra MotionBricks screens (strike intent, arm-through-torso); --screens none reproduces the step-1 stage
INTENT_TOL_CM = 3.0     # round F: the hand path relative to the chest may deviate from the scaled game path by at most V3's deviation + this (per frame, hand)


def feature_unit(k):
    """unit of a gate feature (anatomy_gate.LIM kind; extras listed in EXTRA_UNITS).  Unknown features raise: nothing picks up a tolerance silently."""
    if k in EXTRA_UNITS:
        return EXTRA_UNITS[k]
    return UNIT_OF_KIND[AG.LIM[k][2]]


def settings():
    return dict(leg_pole='v3_thigh', leg_memo=False, ease_frames=EASE_FRAMES, max_len=MAX_LEN, key_nudge=KEY_NUDGE, reach_max=REACH_MAX,
                knee_min_deg=KNEE_MIN_DEG, knee_drop_deg=KNEE_DROP_DEG, knee_side_deg=KNEE_SIDE_DEG, low_acc_abs=LOW_ACC_ABS, leg_step_deg=LEG_STEP_DEG,
                knee_step_cm=KNEE_STEP_CM, hips_kink_cm=HIPS_KINK_CM, chain_kink_deg=CHAIN_KINK_DEG, mag_eps=MAG_EPS, mag_mode=MAG_MODE,
                mag_tol=dict(MAG_TOL) if MAG_MODE == 'unit_tol' else 'strict (MAG_EPS on every pair)', new_failing_frames='rejected', hips_frame='Root (rig parent)',
                score='accel + 0.1 rot_step + 0.2 hips_z + %.1f lower_accel + %.1f hips_kink' % (SCORE_LOW_W, SCORE_KINK_W),
                solver_intent=getattr(RS, 'INTENT_MODE', 'v3b'), solver_pen=getattr(RS, 'PEN_MODE', 'axis'), solver_loop=getattr(RS, 'LOOP_MODE', 'held'), screens=list(SCREENS),
                intent_tol_cm=INTENT_TOL_CM if 'intent' in SCREENS else None, pen_slack=RPN.PEN_SLACK if 'pen' in SCREENS else None)


SETTINGS = settings()       # round E name kept (module default); finish() writes settings() so a --mag switch is recorded
_MAN = None


def manifest():
    global _MAN
    if _MAN is None:
        m = json.load(open(os.path.join(MC.WORK, 'manifest.json'), encoding='utf-8'))
        _MAN = {(c['set'], c['clip']): c for c in m['clips']}
    return _MAN


# ------------------------------------------------------------------ V3

def make_v3(B, al, s, c):
    """The v3b clip, identical to redo_demo.py -> d (game dump), Wk [T,73,4,4], game gate, redo intent, solver info."""
    d = MC.load_dump(MC.clip_path(s, c))
    gv = GB.frame_gate(d['W'], B.rg, al=al)
    intent = RS.plan_intent(B, d)
    Wr, info, intent = RS.solve_refined(B, d, intent)
    Wk, wg = RS.blend_keep(B, d['W'], Wr, RS.keep_mask(B, d['W'], gv['ok']))
    return d, Wk, gv, intent, info


def locals_all(B, W):
    return np.stack([B.locals_of(W[t]) for t in range(len(W))])


def fk_all(B, L):
    return np.stack([B.fk(L[t]) for t in range(len(L))])


# ------------------------------------------------------------------ keys / segments

def hand_speeds(B, W):
    out = []
    for S in ('Left', 'Right'):
        p = W[:, B.ix[S + 'Hand'], :3, 3]
        v = np.linalg.norm(np.diff(p, axis=0), axis=1)
        v = np.concatenate([v[:1], v]) * 100.0
        out.append(RS.smooth(v[:, None], 5)[:, 0])
    return out


def pick_keys(B, W3, ph, T, min_seg=MIN_SEG, max_segs=MAX_SEGS, max_len=None, nudge=0):
    """-> (keys, source). Candidates in priority order: speed peak of the strike hand (interior frames), the phase boundaries of the manifest (active start / end,
    recover end; only when present), motion end, motion start.  Frames 0 and T-1 always; a candidate is added only if it keeps D >= min_seg to every key chosen
    so far (at most max_segs segments).  F6: without manifest phases a candidate that is too close is moved by up to `nudge` frames to the nearest admissible
    frame instead of being dropped.  Gaps longer than max_len are split evenly."""
    sp = hand_speeds(B, W3)
    s = sp[0] if sp[0].sum() >= sp[1].sum() else sp[1]
    cand, src = [], []
    lo, hi = min(3, T - 1), max(T - 4, 0)
    if s.max() > 1e-6 and hi > lo:
        cand.append(int(lo + s[lo:hi + 1].argmax()))
        src.append('speed_peak')
    pb = []
    for name in ('active', 'startup', 'recover'):
        r = (ph or {}).get(name)
        if isinstance(r, (list, tuple)) and len(r) == 2:
            pb += [int(min(max(x, 0), T - 1)) for x in r]
    if pb:
        src.append('phases')
        cand += [x for x in sorted(set(pb), key=lambda x: (abs(x - T / 2.0)))]
    if s.max() > 1e-6:
        on = np.where(s > 0.15 * s.max())[0]
        cand += [int(on[-1]), int(on[0])]
        src.append('speed_edges')
    keys = [0, T - 1]
    nudged = []
    for k in cand:
        if len(keys) - 1 >= max_segs:
            break
        if all(abs(k - x) >= min_seg for x in keys):
            keys.append(k)
        elif nudge and not pb:
            for dd in range(1, nudge + 1):
                ok = [kk for kk in (k - dd, k + dd) if 0 < kk < T - 1 and all(abs(kk - x) >= min_seg for x in keys)]
                if ok:
                    keys.append(ok[0])
                    nudged.append('%d->%d' % (k, ok[0]))
                    break
    keys = sorted(set(keys))
    if max_len:      # split long gaps evenly
        out = [keys[0]]
        for a, b in zip(keys[:-1], keys[1:]):
            n = int(np.ceil((b - a) / float(max_len)))
            out += [int(round(a + (b - a) * i / n)) for i in range(1, n + 1)]
        keys = out
    if nudged:
        src.append('nudged(%s)' % ','.join(nudged))
    return keys, '+'.join(src) or 'none'


def n_tokens(D):
    return int(min(max(int(round((D + 7) / 4.0)), 6), 16))


def plan_segments(keys, max_comp=MAX_COMP):
    segs = []
    for a, b in zip(keys[:-1], keys[1:]):
        D = b - a
        N = n_tokens(D)
        gap = 4 * N - 7
        comp = gap / float(D)
        mode, why = 'mb', ''
        if comp > max_comp:
            mode, why = 'v3', 'short (gap %d frames for D %d: %.2fx compression)' % (gap, D, comp)
        elif 1.0 / comp > max_comp:
            mode, why = 'v3', 'too_long (D %d, gap %d)' % (D, gap)
        segs.append(dict(c=int(a), g=int(b), D=int(D), N=N, gap=gap, comp=round(comp, 3), mode=mode, why=why))
    return segs


# ------------------------------------------------------------------ metrics

def feature_arrays(W, B, al):
    gk = GB.frame_gate(W, B.rg, al=al)
    return gk


def joint_accel(W):
    """[T-2] max joint acceleration over all joints (cm / frame^2), with the worst joint index per frame."""
    P = W[:, :, :3, 3]
    a = (P[2:] - 2 * P[1:-1] + P[:-2]) * 100.0
    n = np.linalg.norm(a, axis=2)
    return n.max(1), n.argmax(1)


def lower_accel(B, W):
    """[T] max acceleration of the lower-body joints (Hips, UpLeg, Leg, Foot; cm / frame^2) per frame, 0 on the first / last frame of W."""
    P = W[:, [B.ix[n] for n in LOWER]][:, :, :3, 3]
    a = np.zeros(len(W))
    if len(W) > 2:
        a[1:-1] = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=2).max(1) * 100.0
    return a


def leg_reach(B, W):
    """[T, 2] hip-to-ankle distance / (thigh + shin), Left / Right."""
    out = np.zeros((len(W), 2))
    for k, S in enumerate(('Left', 'Right')):
        h, kn, a = (W[:, B.ix[S + n], :3, 3] for n in ('UpLeg', 'Leg', 'Foot'))
        out[:, k] = np.linalg.norm(a - h, axis=1) / (np.linalg.norm(kn - h, axis=1) + np.linalg.norm(a - kn, axis=1))
    return out


def knee_side_dev(B, W, W3):
    """[T, 2] signed angle (deg) of the knee bend side (knee offset about the hip -> ankle axis) of W vs W3, Left / Right (the round-D measure)."""
    out = np.zeros((len(W), 2))
    for k, S in enumerate(('Left', 'Right')):
        h, kn, a = (W[:, B.ix[S + n], :3, 3] for n in ('UpLeg', 'Leg', 'Foot'))
        h3, kn3, a3 = (W3[:, B.ix[S + n], :3, 3] for n in ('UpLeg', 'Leg', 'Foot'))
        u3 = (a3 - h3) / np.linalg.norm(a3 - h3, axis=1)[:, None]
        u = (a - h) / np.linalg.norm(a - h, axis=1)[:, None]
        v = (kn - h) - np.einsum('ti,ti->t', kn - h, u)[:, None] * u
        v3 = (kn3 - h3) - np.einsum('ti,ti->t', kn3 - h3, u3)[:, None] * u3
        out[:, k] = np.degrees(np.arctan2(np.einsum('ti,ti->t', np.cross(v3, v), u3), np.einsum('ti,ti->t', v3, v)))
    return out


def local_rot(B, W, j):
    """[T,3,3] local rotation of joint j (relative to its rig parent)."""
    p = B.par[j]
    return W[:, j, :3, :3] if p < 0 else np.einsum('tji,tjk->tik', W[:, p, :3, :3], W[:, j, :3, :3])


def foot_stats(B, W, ground):
    """slide = horizontal path length (cm) of the ball of the foot (ToeBase, the planted point of the v3 feet) inside ground contacts (within 4 cm of its clip-minimum
    height in V3, slower than 1.2 m/s); foot_min_z_rel_cm = lowest Foot / ToeBase / Heel joint relative to the lowest one of V3 (< -1 cm = sunk into the ground)."""
    res, mn = {}, 1e9
    for S in ('Left', 'Right'):
        j = B.ix[S + 'ToeBase']
        pos = W[:, j, :3, 3]
        h = pos[:, 2] - ground[j]
        v = np.zeros(len(pos))
        v[1:] = np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1) * MC.FPS
        v[0] = v[1] if len(pos) > 1 else 0.0
        contact = (h < 0.04) & (v < 1.2)
        worst, f = 0.0, 0
        while f < len(pos):
            if contact[f]:
                e = f
                while e + 1 < len(pos) and contact[e + 1]:
                    e += 1
                worst = max(worst, float(np.linalg.norm(np.diff(pos[f:e + 1, :2], axis=0), axis=1).sum() * 100.0))
                f = e + 1
            else:
                f += 1
        res[S] = worst
        for n in ('Foot', 'ToeBase', 'Heel'):
            jj = B.ix[S + n]
            mn = min(mn, float(W[:, jj, 2, 3].min() - ground[jj]))
    return dict(slide_cm=round(max(res.values()), 3), slide_L_cm=round(res['Left'], 3), slide_R_cm=round(res['Right'], 3), foot_min_z_rel_cm=round(mn * 100, 3))


def rot_angle_deg(Ra, Rb):
    M = np.einsum('...ji,...jk->...ik', Ra, Rb)
    return np.degrees(np.arccos(np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))


def added_motion(B, W3, W4):
    ix = B.ix
    h3, h4 = W3[:, ix['Hips'], :3, 3], W4[:, ix['Hips'], :3, 3]
    d = np.linalg.norm(h4 - h3, axis=1) * 100
    out = dict(hips_dev_max_cm=round(float(d.max()), 2), hips_dev_z_max_cm=round(float(np.abs(h4[:, 2] - h3[:, 2]).max() * 100), 2),
               hips_path_cm_v3=round(float(np.linalg.norm(np.diff(h3, axis=0), axis=1).sum() * 100), 2),
               hips_path_cm_v4=round(float(np.linalg.norm(np.diff(h4, axis=0), axis=1).sum() * 100), 2))
    for j in ('Hips', 'Spine3', 'Neck1', 'Head'):
        out['rot_dev_%s_max_deg' % j] = round(float(rot_angle_deg(W3[:, ix[j], :3, :3], W4[:, ix[j], :3, :3]).max()), 2)
    return out


def leg_summary(B, W, W3, gk):
    """per-clip lower-body numbers (info json 'legs')."""
    return dict(lower_accel_max=round(float(lower_accel(B, W).max()), 3), reach_max=round(float(leg_reach(B, W).max()), 4),
                knee_flex_min=round(float(min(gk['feat']['knee_flex_L'].min(), gk['feat']['knee_flex_R'].min())), 2),
                knee_side_dev_max=round(float(np.abs(knee_side_dev(B, W, W3)).max()), 2))


# ------------------------------------------------------------------ screens of the fix round E (F1 / F2 / F3)

def deeper_violations(gk, g3, sl, mode=None):
    """F3 / F3': features whose V4 violation is deeper than allowed on some frame of the slice -> {feature: max extra excess over V3}.
    mode 'strict': allowed = V3 + MAG_EPS on every pair.  mode 'unit_tol' (default MAG_MODE): a pair already violated in V3 (excess > 1e-9) may deepen by
    MAG_TOL[feature_unit]; a pair clean in V3 stays at V3 + MAG_EPS."""
    mode = mode or MAG_MODE
    out = {}
    for k, ex in gk['viol'].items():
        e3 = g3['viol'][k][sl]
        d = ex[sl] - e3
        if not d.size:
            continue
        if mode == 'unit_tol':
            over = d - np.where(e3 > 1e-9, MAG_TOL[feature_unit(k)], 0.0)
        elif mode == 'strict':
            over = d
        else:
            raise ValueError('MAG_MODE %r' % mode)
        if float(over.max()) > MAG_EPS:
            out[k] = round(float(d.max()), 4)
    return out


def new_failing_frames(gk, g3, sl):
    """frames of the slice that pass the gate in V3 and fail in V4 (round F: rejected in both modes; the strict rule implied it)."""
    return [int(x) + (sl.start or 0) for x in np.where(g3['ok'][sl] & ~gk['ok'][sl])[0]]


def extra_screens(B, ctx, seg, W4):
    """round F screens on the interior frames c+1..g-1 of the assembled clip: strike intent (vs V3 + INTENT_TOL_CM) and arm-through-torso (proxy vs V3) -> (reasons, measures)."""
    c, g = seg['c'], seg['g']
    reasons, ms = [], {}
    if g - c < 2:
        return reasons, ms
    if 'intent' in SCREENS:
        dv = RI.dev(B, ctx['d']['W'][c + 1:g], W4[c + 1:g], ratio=ctx['ratio'])
        worst = -1e9
        for S in ('Left', 'Right'):
            over = dv[S]['world'] - (ctx['idev3'][S][c + 1:g] + INTENT_TOL_CM)
            worst = max(worst, float(over.max()))
            if over.max() > 0:
                t = int(over.argmax())
                reasons.append('intent dev %.1f > v3 %.1f + %.0f cm f%d %s' % (dv[S]['world'][t], ctx['idev3'][S][c + 1 + t], INTENT_TOL_CM, c + 1 + t, S[0]))
        ms['intent_over_v3_max'] = round(worst + INTENT_TOL_CM, 2)
    if 'pen' in SCREENS:
        cnt = RPN.proxy().counts(W4, range(c + 1, g))
        p3 = ctx['pen3'][c + 1:g]
        bad = cnt > np.maximum(RPN.proxy().thr, p3)
        ms['pen_max'], ms['pen_v3_max'] = int(cnt.max()), int(p3.max())
        if bad.any():
            t = int(np.argmax(bad))
            reasons.append('penetration %d > max(%d, v3 %d) f%d' % (cnt[t], RPN.proxy().thr, p3[t], c + 1 + t))
    return reasons, ms


def continuity(B, ctx, seg, W4, gk):
    """F1 / F2 (and the per-frame knee part of F3) screens of segment seg on the assembled clip W4 (V3 = ctx['W3'], gk = frame_gate(W4)) -> (reasons, measures).
    Interior frames c+1..g-1: leg reach, knee extension, per-frame knee bend vs V3, knee bend side.  Window c-1..g+1: lower-body acceleration (frames c..g).
    Keys c / g: UpLeg local-rotation step and knee position step (c -> c+1, g-1 -> g; 'inner' = steps between interior frames), hips velocity change and
    Spine1..Head angular-velocity change at the key frame (only where both neighbours exist; 'inner' = frames c+1..g-1)."""
    c, g = seg['c'], seg['g']
    W3, T = ctx['W3'], len(ctx['W3'])
    ix = B.ix
    g3f = ctx['gate3']['feat']
    reasons, ms = [], {}
    if g - c < 2:
        return reasons, dict(low_acc_max=0.0, hips_kink_max=0.0)
    # (a) reach
    rc = leg_reach(B, W4[c + 1:g])
    ms['reach_max'] = round(float(rc.max()), 4)
    if rc.max() > REACH_MAX:
        t, k = np.unravel_index(int(rc.argmax()), rc.shape)
        reasons.append('leg reach %.3f > %.3f f%d %s' % (rc.max(), REACH_MAX, c + 1 + t, 'LR'[k]))
    # (b) knee extension, per-frame knee bend (F3 / L1)
    ext_m, bend_m = 1e9, -1e9
    for s in 'LR':
        k4 = np.asarray(gk['feat']['knee_flex_' + s][c + 1:g], float)
        k3 = np.asarray(g3f['knee_flex_' + s][c + 1:g], float)
        m = k4 - np.maximum(KNEE_MIN_DEG, k3 - KNEE_DROP_DEG)
        if m.min() < ext_m:
            ext_m = float(m.min())
        if m.min() < 0:
            t = int(m.argmin())
            reasons.append('knee extension %.1f < %.1f deg f%d %s' % (k4[t], max(KNEE_MIN_DEG, k3[t] - KNEE_DROP_DEG), c + 1 + t, s))
        u = k4 - (k3 + KNEE_PLUS_DEG)
        bend_m = max(bend_m, float(u.max()))
        if u.max() > 0:
            t = int(u.argmax())
            reasons.append('knee bend %.1f > v3 %.1f + %.0f deg f%d %s' % (k4[t], k3[t], KNEE_PLUS_DEG, c + 1 + t, s))
    ms['knee_ext_margin_min'], ms['knee_bend_over_v3_max'] = round(ext_m, 2), round(bend_m + KNEE_PLUS_DEG, 2)
    # (c) bend side
    sd = np.abs(knee_side_dev(B, W4[c + 1:g], W3[c + 1:g]))
    ms['knee_side_max'] = round(float(sd.max()), 2)
    if sd.max() > KNEE_SIDE_DEG:
        t, k = np.unravel_index(int(sd.argmax()), sd.shape)
        reasons.append('knee side %.1f > %.0f deg f%d %s' % (sd.max(), KNEE_SIDE_DEG, c + 1 + t, 'LR'[k]))
    # (d) lower-body acceleration over the window (keys included)
    lo, hi = max(0, c - 1), min(T - 1, g + 1)
    la4, la3 = lower_accel(B, W4[lo:hi + 1]), lower_accel(B, W3[lo:hi + 1])
    bound = max(LOW_ACC_ABS, ACC_RATIO * float(la3.max()) + ACC_ABS)
    ms['low_acc_max'], ms['low_acc_v3'], ms['low_acc_bound'] = round(float(la4.max()), 3), round(float(la3.max()), 3), round(bound, 3)
    if la4.max() > bound:
        reasons.append('lower-body accel %.2f > %.2f cm/f2 f%d' % (la4.max(), bound, lo + int(la4.argmax())))
    # (e) UpLeg rotation step / knee position step at the keys
    ms['upleg_step'], ms['knee_step'] = [], []
    for S in ('Left', 'Right'):
        j = ix[S + 'UpLeg']
        s4 = rot_angle_deg(local_rot(B, W4[c:g + 1], j)[:-1], local_rot(B, W4[c:g + 1], j)[1:])
        s3 = rot_angle_deg(local_rot(B, W3[c:g + 1], j)[:-1], local_rot(B, W3[c:g + 1], j)[1:])
        inner = float(s4[1:-1].max()) if len(s4) > 2 else 0.0
        kl = ix[S + 'Leg']
        n4 = np.linalg.norm(np.diff(W4[c:g + 1, kl, :3, 3], axis=0), axis=1) * 100.0
        n3 = np.linalg.norm(np.diff(W3[c:g + 1, kl, :3, 3], axis=0), axis=1) * 100.0
        for name, i, f0 in (('start', 0, c), ('end', -1, g - 1)):
            b = max(float(s3[i]), inner) + LEG_STEP_DEG
            ms['upleg_step'].append((S[0], name, round(float(s4[i]), 3), round(float(s3[i]), 3), round(inner, 3)))
            if s4[i] > b:
                reasons.append('upleg step %.2f > %.2f deg f%d->%d %s' % (s4[i], b, f0, f0 + 1, S[0]))
            ms['knee_step'].append((S[0], name, round(float(n4[i]), 3), round(float(n3[i]), 3)))
            if n4[i] > n3[i] + KNEE_STEP_CM:
                reasons.append('knee step %.2f > v3 %.2f + %.0f cm f%d->%d %s' % (n4[i], n3[i], KNEE_STEP_CM, f0, f0 + 1, S[0]))
    # F2: hips velocity change and Spine1..Head angular-velocity change at the key frames
    o = lo
    Wa, Wb = W4[lo:hi + 1], W3[lo:hi + 1]
    n = len(Wa)

    def hips_dv(Wx):
        p = Wx[:, ix['Hips'], :3, 3] * 100.0
        out = np.zeros(n)
        if n > 2:
            out[1:-1] = np.linalg.norm(p[2:] - 2 * p[1:-1] + p[:-2], axis=1)
        return out

    def chain_dw(Wx):
        out = np.zeros(n)
        for jn in UPCHAIN:
            Rl = local_rot(B, Wx, ix[jn])
            av = np.degrees(R.from_matrix(np.einsum('tji,tjk->tik', Rl[:-1], Rl[1:])).as_rotvec())       # [n-1] step i: frame o+i -> o+i+1
            if n > 2:
                out[1:-1] = np.maximum(out[1:-1], np.linalg.norm(av[1:] - av[:-1], axis=1))
        return out
    hv4, hv3, cw4, cw3 = hips_dv(Wa), hips_dv(Wb), chain_dw(Wa), chain_dw(Wb)
    inn = [f - o for f in range(c + 1, g) if 0 < f - o < n - 1]
    hin = float(hv4[inn].max()) if inn else 0.0
    cin = float(cw4[inn].max()) if inn else 0.0
    ms['hips_kink'], ms['chain_kink'] = [], []
    hk = 0.0
    for f in (c, g):
        i = f - o
        if not (0 < f < T - 1 and 0 < i < n - 1):
            continue
        hk = max(hk, float(hv4[i]))
        bh, bc = max(float(hv3[i]), hin) + HIPS_KINK_CM, max(float(cw3[i]), cin) + CHAIN_KINK_DEG
        ms['hips_kink'].append((f, round(float(hv4[i]), 3), round(float(hv3[i]), 3), round(hin, 3)))
        ms['chain_kink'].append((f, round(float(cw4[i]), 3), round(float(cw3[i]), 3), round(cin, 3)))
        if hv4[i] > bh:
            reasons.append('hips kink %.2f > %.2f cm/f f%d' % (hv4[i], bh, f))
        if cw4[i] > bc:
            reasons.append('chain kink %.2f > %.2f deg/f f%d' % (cw4[i], bc, f))
    ms['hips_kink_max'] = round(hk, 3)
    return reasons, ms


# ------------------------------------------------------------------ one segment of one seed

def mb_chain(B, br, ctx, seg, Gg, hg):
    """SOMA span (D+1 frames, key frames forced) -> locals [D+1,73,4,4] of frames c..g: V3 locals everywhere except Hips + Spine..Head = MotionBricks (through
    game_skeleton_bridge inv; a smooth offset makes the round trip exact at both keys).  Also the raw cp_inv result (unpinned MotionBricks legs).
    F4: the Hips local is the MotionBricks hips world transform taken relative to the V3 rig parent of Hips (Root, identity in every clip; the clip keeps V3's Root),
    the same frame as the V3 locals L3; it was relative to the V3 Trajectory joint, which is a fixed yaw in some clips (48.6 deg in default_melee_weapon
    fall_attack*), so the MotionBricks hips delta came out rotated by that yaw.  (cp_inv's own Root / Trajectory follow the generated hips, so its Hips local
    relative to its own Root would drop the hips motion.)"""
    c, g = seg['c'], seg['g']
    W3, L3 = ctx['W3'], ctx['L3']
    ix = B.ix
    n = g - c + 1
    Winv = CP.cp_inv(br, dict(W=W3, parents=np.array(B.par)), Gg, hg, list(range(c, g + 1)))
    ph = B.par[ix['Hips']]
    assert ph >= 0 and B.names[ph] == 'Root', 'Hips is expected to be parented to Root'
    Linv = np.stack([B.locals_of(Winv[t]) for t in range(n)])
    for t in range(n):      # F4: MotionBricks hips WORLD transform in the frame of the V3 rig parent (Root, kept in the clip); cp_inv moves its own Root with the hips
        Linv[t, ix['Hips']] = np.linalg.inv(W3[c + t, ph]) @ Winv[t, ix['Hips']]
    w = np.linspace(0.0, 1.0, n)
    L = L3[c:g + 1].copy()
    for jn in CHAIN:
        j = ix[jn]
        Rinv = Linv[:, j, :3, :3]
        Da, Db = Rinv[0].T @ L3[c, j, :3, :3], Rinv[-1].T @ L3[g, j, :3, :3]
        Dm = Slerp([0, 1], R.from_matrix(np.stack([Da, Db])))(w).as_matrix()
        L[:, j, :3, :3] = np.einsum('tij,tjk->tik', Rinv, Dm)
        if jn == 'Hips':
            ta, tb = L3[c, j, :3, 3] - Linv[0, j, :3, 3], L3[g, j, :3, 3] - Linv[-1, j, :3, 3]
            L[:, j, :3, 3] = Linv[:, j, :3, 3] + (1 - w)[:, None] * ta + w[:, None] * tb
    return L, Winv


def ease_weights(n, beta, ease=EASE_FRAMES):
    """F2: [n] weight of the MotionBricks delta per frame of a span (n = D + 1): beta x smoothstep(min(t, n-1-t) / (ease + 1)); 0 on the keys, beta from frame ease+1."""
    t = np.arange(n, dtype=float)
    r = np.clip(np.minimum(t, n - 1 - t) / float(ease + 1), 0.0, 1.0)
    return beta * r * r * (3.0 - 2.0 * r)


def build_span(B, ctx, seg, Lmb, beta):
    """Weight beta x ease(t) of the MotionBricks chain (slerp / lerp of the Hips + Spine..Head locals towards V3; F2: eased in / out at the keys), V3 arms (local
    lock: every other local is V3's), legs pinned to the V3 feet by hinge IK from the resulting hips on the interior frames -> W [D+1,73,4,4] (frames c..g),
    leg IK residual (cm).  F1: knee pole = the V3 thigh direction (not turned with the pelvis delta), no two_bone continuity memo."""
    c, g = seg['c'], seg['g']
    W3, L3 = ctx['W3'], ctx['L3']
    ix = B.ix
    n = g - c + 1
    w = ease_weights(n, beta)
    L = Lmb.copy()
    for jn in CHAIN:
        j = ix[jn]
        for t in range(n):
            if w[t] >= 1.0 - 1e-12:
                continue                                    # full MotionBricks local
            if w[t] <= 1e-12:
                L[t, j, :3, :3] = L3[c + t, j, :3, :3]
                continue
            qa, qb = R.from_matrix(L3[c + t, j, :3, :3]), R.from_matrix(Lmb[t, j, :3, :3])
            L[t, j, :3, :3] = (qa * R.from_rotvec((qa.inv() * qb).as_rotvec() * w[t])).as_matrix()
        if jn == 'Hips':
            L[:, j, :3, 3] = (1 - w)[:, None] * L3[c:g + 1, j, :3, 3] + w[:, None] * Lmb[:, j, :3, 3]
    out = np.empty((n, len(B.names), 4, 4))
    resid = 0.0
    for t in range(n):
        f = c + t
        if 0 < t < n - 1:
            for S in ('Left', 'Right'):
                ul, lg, ft = ix[S + 'UpLeg'], ix[S + 'Leg'], ix[S + 'Foot']
                pole = W3[f, lg, :3, 3] - W3[f, ul, :3, 3]
                r, _ = RS.two_bone(B, L[t], ul, lg, ft, W3[f, ft, :3, 3], pole, end_world=W3[f, ft, :3, :3], memo=None)
                resid = max(resid, r)
        out[t] = B.fk(L[t])
    return out, resid * 100.0


def assemble(W3, segs_done):
    """V3 with the interior frames of the accepted segments replaced: segs_done = [(c, g, Wspan [g-c+1,...])]."""
    W = W3.copy()
    for c, g, Ws in segs_done:
        if g - c > 1:
            W[c + 1:g] = Ws[1:-1]
    return W


def window_problems(B, ctx, seg, W4, gk):
    """Gate / acceleration / continuity problems of the assembled clip inside the window of a segment (V3 is the reference): [] when fine.
    F2 / F3: also the per-frame violation depth, the lower-body acceleration, the key continuity and velocity-kink screens (continuity())."""
    c, g = seg['c'], seg['g']
    T = len(ctx['W3'])
    w0, w1 = max(0, c - 1), min(T - 1, g + 1)
    sl = slice(w0, w1 + 1)
    g3 = ctx['gate3']
    out = []
    if int(gk['ok'][sl].sum()) < int(g3['ok'][sl].sum()):
        out.append('gate pass')
    for k, ex in gk['viol'].items():
        if int((ex[sl] > 1e-9).sum()) > int((g3['viol'][k][sl] > 1e-9).sum()):
            out.append('new violation ' + k)
    dp = deeper_violations(gk, g3, sl)
    if dp:
        out.append('deeper violation %s' % dp)
    nf = new_failing_frames(gk, g3, sl)
    if nf:
        out.append('new failing frames %s' % nf)
    a4, _ = joint_accel(W4[max(0, c - 1):min(T, g + 2)])
    a3, _ = joint_accel(ctx['W3'][max(0, c - 1):min(T, g + 2)])
    if len(a4) and float(a4.max()) > ACC_RATIO * float(a3.max()) + ACC_ABS:
        out.append('joint accel')
    out += continuity(B, ctx, seg, W4, gk)[0]
    out += extra_screens(B, ctx, seg, W4)[0]
    return out


def evaluate(B, al, ctx, seg, Wspan, resid, Winv, gate3):
    c, g = seg['c'], seg['g']
    T = len(ctx['W3'])
    W3 = ctx['W3']
    W4 = assemble(W3, [(c, g, Wspan)])
    gk = GB.frame_gate(W4, B.rg, al=al)
    w0, w1 = max(0, c - 1), min(T - 1, g + 1)
    sl = slice(w0, w1 + 1)
    pass3, pass4 = int(gate3['ok'][sl].sum()), int(gk['ok'][sl].sum())
    new_viol = {}
    for k, ex in gk['viol'].items():
        n4, n3 = int((ex[sl] > 1e-9).sum()), int((gate3['viol'][k][sl] > 1e-9).sum())
        if n4 > n3:
            new_viol[k] = (n3, n4)
    deeper = deeper_violations(gk, gate3, sl)
    newfail = new_failing_frames(gk, gate3, sl)
    hz = float(np.abs(W4[w0:w1 + 1, B.ix['Hips'], 2, 3] - W3[w0:w1 + 1, B.ix['Hips'], 2, 3]).max() * 100)
    k4 = max(float(gk['feat']['knee_flex_L'][sl].max()), float(gk['feat']['knee_flex_R'][sl].max()))
    k3 = max(float(gate3['feat']['knee_flex_L'][sl].max()), float(gate3['feat']['knee_flex_R'][sl].max()))
    a4, _ = joint_accel(W4[max(0, c - 1):min(T, g + 2)])
    a3, _ = joint_accel(W3[max(0, c - 1):min(T, g + 2)])
    a4m, a3m = float(a4.max()) if len(a4) else 0.0, float(a3.max()) if len(a3) else 0.0
    rs4 = float(gk['feat']['rot_step'][sl].max()) if 'rot_step' in gk['feat'] else 0.0
    rs3 = float(gate3['feat']['rot_step'][sl].max()) if 'rot_step' in gate3['feat'] else 0.0
    raw = foot_stats(B, assemble(W3, [(c, g, Winv)]), ctx['ground'])
    ix = B.ix
    chest_dev = float(rot_angle_deg(W4[sl, ix['Spine3'], :3, :3], W3[sl, ix['Spine3'], :3, :3]).max())
    hand_dev = max(float(np.linalg.norm(W4[sl, ix[S + 'Hand'], :3, 3] - W3[sl, ix[S + 'Hand'], :3, 3], axis=1).max() * 100) for S in ('Left', 'Right'))
    cont, cm = continuity(B, ctx, seg, W4, gk)
    reasons = []
    if pass4 < pass3:
        reasons.append('gate pass %d < v3 %d' % (pass4, pass3))
    if new_viol:
        reasons.append('new violations %s' % new_viol)
    if deeper:
        reasons.append('deeper violation %s' % deeper)
    if newfail:
        reasons.append('new failing frames %s' % newfail)
    if hz > HIPS_Z_CM:
        reasons.append('hips z dev %.1f cm' % hz)
    if k4 > k3 + KNEE_PLUS_DEG:
        reasons.append('knee flex %.1f vs v3 %.1f' % (k4, k3))
    if resid > IK_RESID_CM:
        reasons.append('leg IK residual %.2f cm' % resid)
    if a4m > ACC_RATIO * a3m + ACC_ABS:
        reasons.append('joint accel %.2f vs v3 %.2f cm/f2' % (a4m, a3m))
    if chest_dev > CHEST_DEV_DEG:
        reasons.append('chest dev %.1f deg vs v3' % chest_dev)
    if hand_dev > HAND_DEV_CM:
        reasons.append('hand world dev %.1f cm vs v3' % hand_dev)
    reasons += cont
    if not reasons:     # round F screens (the penetration proxy is the costly one: only for otherwise passing candidates)
        xr, xm = extra_screens(B, ctx, seg, W4)
        reasons += xr
        cm = dict(cm, **xm)
    score = a4m + 0.1 * rs4 + 0.2 * hz + SCORE_LOW_W * cm['low_acc_max'] + SCORE_KINK_W * cm['hips_kink_max']
    return dict(ok=not reasons, reasons=reasons, pass_v3=pass3, pass_v4=pass4, window=[w0, w1], accel_max=round(a4m, 3), accel_v3=round(a3m, 3), rot_step_max=round(rs4, 2),
                rot_step_v3=round(rs3, 2), hips_z_dev_cm=round(hz, 2), knee_max=round(k4, 1), knee_v3=round(k3, 1), ik_resid_cm=round(resid, 3),
                chest_dev_deg=round(chest_dev, 1), hand_dev_cm=round(hand_dev, 1), raw_mb_slide_cm=raw['slide_cm'], raw_mb_min_z_rel_cm=raw['foot_min_z_rel_cm'],
                deeper=deeper, cont=cm, score=round(score, 3))


# ------------------------------------------------------------------ prepare / finish

def seed_label_to_trav(s):
    return -1 if int(s) == 0 else int(s)


def prepare(B, al, s, c, seeds, workdir, max_comp=MAX_COMP, no_mb=False):
    t0 = time.time()
    if MC.RIG_OF_SET[s] != RIG:
        raise SystemExit('set %s belongs to rig %s but UTPP_RIG=%s' % (s, MC.RIG_OF_SET[s], RIG))
    d, W3, gv, intent, sinfo = make_v3(B, al, s, c)
    T = len(W3)
    assert list(d['names']) == list(B.names), 'dump joints differ from the rig'
    ph = manifest().get((s, c), {}).get('phases') or {}
    keys, ksrc = pick_keys(B, W3, ph, T, max_len=MAX_LEN, nudge=KEY_NUDGE)
    segs = plan_segments(keys, max_comp)
    os.makedirs(workdir, exist_ok=True)
    v3p = os.path.join(workdir, 'v3.npz')
    np.savez(v3p, W=W3, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
    ctx = dict(set=s, clip=c, T=T, d=d, W3=W3, L3=locals_all(B, W3), gv=gv, intent=intent, sinfo=sinfo, keys=keys, keys_src=ksrc, segs=segs, seeds=list(seeds),
               workdir=workdir, tm=dict(), no_mb=no_mb)
    ctx['gate3'] = GB.frame_gate(W3, B.rg, al=al)
    ctx['ground'] = {j: float(W3[:, j, 2, 3].min()) for j in range(len(B.names))}
    ctx['ratio'] = RS.arm_ratio(B, d['W'])
    ctx['idev3'] = {S: x['world'] for S, x in RI.dev(B, d['W'], W3, ratio=ctx['ratio']).items()}       # round F: V3's strike-intent deviation per frame
    ctx['pen3'] = RPN.proxy().counts(W3) if 'pen' in SCREENS else None
    jobs = []
    if not no_mb and any(sg['mode'] == 'mb' for sg in segs):
        br, _ = CP.load_cp_bridge(TPOSE, v3p)
        G, hips, _pos = br.fwd(W3)
        G, hips = G.astype(np.float64), hips.astype(np.float64)
        ctx.update(br=br, G=G, hips=hips)
        snames, kk = np.array(br.snames), br.k
        for i, sg in enumerate(segs):
            if sg['mode'] != 'mb':
                continue
            cc, gg, N, gap = sg['c'], sg['g'], sg['N'], sg['gap']
            ctxG, tgtG = MK.pad_take(G, cc - 3, cc), MK.pad_take(G, gg, gg + 3)
            ch, th = MK.pad_take(hips, cc - 3, cc), MK.pad_take(hips, gg, gg + 3)
            sc = os.path.join(workdir, 'seg%d.scene.soma.npz' % i)
            np.savez(sc, G=np.concatenate([ctxG, np.repeat(ctxG[-1:], gap - 1, 0), tgtG]), hips=np.concatenate([ch, np.repeat(ch[-1:], gap - 1, 0), th]), names=snames, k=kk, fps=30)
            jobs.append(dict(soma=sc, out=os.path.join(workdir, 'seg%d' % i), gaps=[dict(c=3, N=N, targets=4)], seeds=[seed_label_to_trav(x) for x in seeds], jitter_cm=JITTER_CM))
    ctx['tm']['prepare'] = round(time.time() - t0, 2)
    return ctx, jobs


def _demotions(bad, pool, sel, segs):
    """F7: adjacent flagged segments share a key, so per run of adjacent flagged segments only the one with the worse (higher) score is demoted this round."""
    out, run = [], []
    for i in bad:
        if run and segs[run[-1]]['g'] == segs[i]['c']:
            run.append(i)
        else:
            if run:
                out.append(max(run, key=lambda x: pool[x][sel[x]]['score']))
            run = [i]
    if run:
        out.append(max(run, key=lambda x: pool[x][sel[x]]['score']))
    return out


def finish(B, al, ctx, out_dir, keep_tmp=False):
    t0 = time.time()
    s, c, T, W3 = ctx['set'], ctx['clip'], ctx['T'], ctx['W3']
    prefix = os.path.join(out_dir, '%s__%s' % (s, c))
    np.savez(prefix + '.v3.npz', W=W3, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
    segs_info, pool, sel = [], {}, {}
    for i, sg in enumerate(ctx['segs']):
        si = dict(sg)
        si['status'] = 'v3'
        if sg['mode'] == 'mb' and not ctx['no_mb']:
            cands = []
            for sd in ctx['seeds']:
                gp = os.path.join(ctx['workdir'], 'seg%d.g0.s%d.gen.npz' % (i, seed_label_to_trav(sd)))
                if not os.path.exists(gp):
                    cands.append(dict(seed=sd, ok=False, reasons=['no MotionBricks output']))
                    continue
                gen = np.load(gp)
                cc, gg = sg['c'], sg['g']
                Gg, hg = MK.resample_span(gen['G'].astype(np.float64), gen['hips'].astype(np.float64), gg - cc + 1)
                Gg[0], hg[0] = ctx['G'][cc], ctx['hips'][cc]
                Gg[-1], hg[-1] = ctx['G'][gg], ctx['hips'][gg]
                Lmb, Winv = mb_chain(B, ctx['br'], ctx, sg, Gg, hg)
                for beta in BETAS:
                    Ws, resid = build_span(B, ctx, sg, Lmb, beta)
                    ev = evaluate(B, al, ctx, sg, Ws, resid, Winv, ctx['gate3'])
                    ev['seed'], ev['beta'] = sd, beta
                    ev['_W'] = Ws
                    cands.append(ev)
            good = sorted([e for e in cands if e.get('ok')], key=lambda e: (-e['beta'], e['score']))
            if good:
                pool[i] = good
                sel[i] = 0
                si['status'] = 'mb'
            else:
                si['status'] = 'fallback'
            si['candidates'] = [{k: v for k, v in e.items() if k != '_W'} for e in cands]
        else:
            si['status'] = 'v3' if sg['mode'] != 'mb' else 'v3_no_mb'
        segs_info.append(si)
    # joins: segments were screened one at a time with V3 around them; two neighbouring accepted segments change BOTH sides of the shared key frame.  Re-check every
    # window on the assembled clip (gate pass frames, no new / deeper violation, accelerations, key continuity, velocity kinks) and step a violating segment down to its
    # next candidate (smaller beta / other seed), finally to V3.  F7: bounded by the total candidate count + 1 (each round demotes >= 1 candidate -> always converges).
    demoted, it, converged = 0, 0, False
    bound = sum(len(v) for v in pool.values()) + 1
    probs = {}
    for it in range(1, bound + 1):
        done = [(ctx['segs'][i]['c'], ctx['segs'][i]['g'], pool[i][sel[i]]['_W']) for i in sorted(pool)]
        W4 = assemble(W3, done)
        gk = GB.frame_gate(W4, B.rg, al=al)
        probs = {i: window_problems(B, ctx, ctx['segs'][i], W4, gk) for i in sorted(pool)}
        bad = [i for i in sorted(pool) if probs[i]]
        if not bad:
            converged = True
            break
        for i in _demotions(bad, pool, sel, ctx['segs']):
            segs_info[i].setdefault('join_reasons', []).append(probs[i][:4])
            sel[i] += 1
            demoted += 1
            if sel[i] >= len(pool[i]):
                del pool[i]
                segs_info[i]['status'] = 'fallback'
                segs_info[i]['why'] = 'join check'
    if not converged:       # never expected (see the bound); safety: every segment still flagged goes back to V3
        for i in [i for i in sorted(pool) if probs.get(i)]:
            del pool[i]
            segs_info[i]['status'] = 'fallback'
            segs_info[i]['why'] = 'join check (not converged)'
    for i in pool:
        segs_info[i]['seed'], segs_info[i]['beta'] = pool[i][sel[i]]['seed'], pool[i][sel[i]]['beta']
        segs_info[i]['status'] = 'mb'
        segs_info[i]['selected'] = {k: v for k, v in pool[i][sel[i]].items() if k not in ('_W', 'reasons')}
    done = [(ctx['segs'][i]['c'], ctx['segs'][i]['g'], pool[i][sel[i]]['_W']) for i in sorted(pool)]
    W4 = assemble(W3, done)
    g3, g4 = ctx['gate3'], GB.frame_gate(W4, B.rg, al=al)
    f3, f4 = foot_stats(B, W3, ctx['ground']), foot_stats(B, W4, ctx['ground'])
    a3, a4 = joint_accel(W3)[0], joint_accel(W4)[0]
    np.savez(prefix + '.v4.npz', W=W4, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
    n_mb = sum(1 for x in segs_info if x['status'] == 'mb')
    n_fb = sum(1 for x in segs_info if x['status'] == 'fallback')
    info = dict(set=s, clip=c, rig=RIG, frames=T, keys=ctx['keys'], keys_source=ctx['keys_src'], seeds=ctx['seeds'], segments=segs_info,
                join_demotions=demoted, join_check=dict(iterations=it, bound=bound, converged=bool(converged or not bound), demotions=demoted),
                segments_mb=n_mb, segments_fallback=n_fb, segments_v3=len(segs_info) - n_mb - n_fb,
                gate=dict(v3=GB.summary(g3), v4=GB.summary(g4)), foot=dict(v3=f3, v4=f4),
                spikes=dict(accel_max_v3=round(float(a3.max()), 3), accel_max_v4=round(float(a4.max()), 3)),
                rot_step=dict(v3=round(float(g3['feat']['rot_step'].max()), 2), v4=round(float(g4['feat']['rot_step'].max()), 2)),
                legs=dict(v3=leg_summary(B, W3, W3, g3), v4=leg_summary(B, W4, W3, g4)), deeper_than_v3=deeper_violations(g4, g3, slice(0, T), mode='strict'),
                deeper_over_tol=deeper_violations(g4, g3, slice(0, T)), new_failing_frames=new_failing_frames(g4, g3, slice(0, T)),
                added_motion=added_motion(B, W3, W4), identical_to_v3=bool(np.array_equal(W3, W4)), settings=settings(),
                timing_s=dict(ctx['tm'], finish=round(time.time() - t0, 2)))
    MC.jdump(info, prefix + '.v4.info.json')
    print('REDO_MB %s/%s frames %d keys %s segs mb %d fb %d v3 %d | gate v3 %d v4 %d /%d | slide v3 %.2f v4 %.2f cm min z rel v3 %.2f cm | accel v3 %.2f v4 %.2f | lower v3 %.2f v4 %.2f'
          ' | hips dev max %.2f cm | join it %d%s'
          % (s, c, T, ctx['keys'], n_mb, n_fb, info['segments_v3'], g3['ok'].sum(), g4['ok'].sum(), T, f3['slide_cm'], f4['slide_cm'], f4['foot_min_z_rel_cm'],
             a3.max(), a4.max(), info['legs']['v3']['lower_accel_max'], info['legs']['v4']['lower_accel_max'], info['added_motion']['hips_dev_max_cm'], it,
             '' if info['join_check']['converged'] else ' NOT CONVERGED'))
    if not keep_tmp:
        shutil.rmtree(ctx['workdir'], ignore_errors=True)
    return info, W4


def run_mb_jobs(jobs, path, log=print):
    if not jobs:
        return 0.0
    json.dump(jobs, open(path, 'w'))
    t = time.time()
    msg = MK.run_mb(path)
    dt = time.time() - t
    log('MB run: %d jobs, %.1f s (%s)' % (len(jobs), dt, msg))
    return dt


def run_clips(pairs, seeds, out_dir, no_mb=False, max_comp=MAX_COMP, keep_tmp=False, log=print, B=None, al=None):
    """prepare every clip, ONE MotionBricks run for all of them, finish every clip -> list of info dicts."""
    B = B or RS.Body()
    al = al or GB.allowed()
    os.makedirs(out_dir, exist_ok=True)
    tmp = os.path.join(os.path.dirname(os.path.abspath(out_dir)), '_tmp')     # round F: scratch next to the out dir (default out MB4\\clips -> MB4\\_tmp as before)
    os.makedirs(tmp, exist_ok=True)
    ctxs, jobs = [], []
    for s, c in pairs:
        ctx, jb = prepare(B, al, s, c, seeds, os.path.join(tmp, '%s__%s' % (s, c)), max_comp, no_mb)
        ctxs.append(ctx)
        jobs += jb
    dt = run_mb_jobs(jobs, os.path.join(tmp, 'jobs_%d.json' % os.getpid()), log)
    n_spans = sum(len(j['gaps']) * len(j['seeds']) for j in jobs)
    out = []
    for ctx in ctxs:
        ctx['tm']['mb_share'] = round(dt / max(len(ctxs), 1), 2)
        ctx['tm']['mb_total'] = round(dt, 1)
        ctx['tm']['mb_spans_total'] = n_spans
        info, W4 = finish(B, al, ctx, out_dir, keep_tmp)
        out.append(info)
    return out


def set_mag(a):
    """--mag strict|unit_tol (round F; default MAG_MODE = unit_tol); --screens pen,intent|intent|pen|none (round F; default both)."""
    global MAG_MODE, SCREENS
    if '--mag' in a:
        MAG_MODE = a[a.index('--mag') + 1]
        assert MAG_MODE in ('strict', 'unit_tol'), MAG_MODE
    if '--screens' in a:
        v = a[a.index('--screens') + 1]
        SCREENS = tuple() if v == 'none' else tuple(v.split(','))
        assert set(SCREENS) <= {'intent', 'pen'}, SCREENS
    return MAG_MODE


def main(a):
    if len(a) < 2 or a[0] in ('-h', '--help'):
        print(__doc__)
        return
    s, c = a[0], a[1]
    set_mag(a)
    seeds = [int(x) for x in a[a.index('--seeds') + 1].split(',')] if '--seeds' in a else [0, 7, 11]
    out = os.path.abspath(a[a.index('--out') + 1]) if '--out' in a else os.path.join(MB4, 'clips')
    mc = float(a[a.index('--max-comp') + 1]) if '--max-comp' in a else MAX_COMP
    infos = run_clips([(s, c)], seeds, out, no_mb='--no-mb' in a, max_comp=mc, keep_tmp='--keep-tmp' in a)
    if '--demo-dir' in a:
        import melee_regenerate_demo as DM
        DM.make(s, c, os.path.abspath(a[a.index('--demo-dir') + 1]), out, infos[0])


if __name__ == '__main__':
    main(sys.argv[1:])
