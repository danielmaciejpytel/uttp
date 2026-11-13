"""Third-person KNIFE / KATANA clips of V from the game's NPC melee animations (npc1), 2026-11-09.

  python -I npc_blade_build.py --weapon knife|katana [--rig woman|man] [--only clip,clip] [--lunge-start 0] [--out DIR]
Output: <OUT>\\clips\\<set>__<clip>.npc1.npz (dump layout of melee_dump.py: W [T,73,4,4] game axes, Root = identity, + lt / lr / W_extra ...),
        <set>__<clip>.npc1.info.json (donors, grip, time map, strike frames, capsule, feet, two-hand IK, gates), <set>__<clip>.npc1.twohand.npz
        (katana: left-hand-in-weapon relation + weight per frame), <OUT>\\build_report_<set>.json.  OUT = Build\\work\\melee-npc-blades-20261109.
Method = the fists method of npc_build.py (W1, imported unchanged; its helpers are reused), per weapon:
  * donors: the weapon's NPC sets (wa_/ma_ gang_knife / corpo_katana action_combat), retargeted by npc_retarget.Donor (NPC locals by name);
    the NPC WeaponRight local TRANSLATION is kept too (npc_blade_common.BSeq), i.e. the exact NPC grip (weapon bone + fingers);
  * time map: the donor's swing peak (fastest blade tip in its loop phase) -> the vanilla strike frame (fastest blade tip in the vanilla
    active window, clamped to its middle half and to >= MIN_PRE frames of wind-up); before it the donor plays at <= SPEEDUP x (fastest try
    that passes the base gate, joint spikes, the wrist-speed rule and the blade gate), after it 1:1, then blend to the guard;
  * guard = the set's NPC idle_combat frame 0 after the anatomy fixes; frame 0 of attacks = guard (heavies: hold pose), fixes ramped in over
    START_RAMP frames (no pin pop), last frame = guard;
  * katana: the left hand follows the hilt by arm IK wherever the NPC holds the katana with both hands (target = V's WeaponRight x the NPC's
    left-hand-in-WeaponRight relation of that frame, weight smoothed); the fist-separation push of the anatomy fixes is off (FIST_GAP 0,
    runtime override of the imported constant, the hands share the hilt); hand-hand clearance is gated only on one-hand frames;
  * gates on every frame: anatomy_gate_base, redo_pen (arm vs torso), blade proxy vs body / head (npc_blade_common.blade_pen = 0 vertices),
    fist / head clearance, feet (slide, sole), joint spikes incl. the first / last step, loop seams, first hand step of attacks
    (f0->f1 <= max(1.3 x f1->f2, 1 cm)), two-hand hilt error.
Capsule (assumption, as fists: the vanilla knife / katana clips carry no root motion): combo / heavy 2.5 cm over frames 0-2.5; finishers /
sprint attack 0.70 m in 0.18 s from --lunge-start.
"""
import json
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blade_common as BC  # noqa: E402  (imports npc_build -> base_body, which consumes --rig)
import hinge_ik as HI  # noqa: E402
import melee_common as MC  # noqa: E402

NB = BC.NB
GB = NB.GB


def opt(a, k, d=None):
    return a[a.index(k) + 1] if k in a else d


WEAPON = opt(sys.argv, '--weapon', 'knife')
assert WEAPON in BC.WEAPONS, WEAPON
RIG = BC.RIG
P = BC.P
IX = P.ix
SET = BC.player_set(WEAPON)
ACT = BC.npc_set(WEAPON)
TWO = BC.WEAPONS[WEAPON]['two_hand']
SUFFIX = 'npc1'
OUT = BC.OUTW
MIN_PRE = 5
GUARD_BY = NB.GUARD_BY
if TWO:
    NB.FIST_GAP = 0.0          # local override (this process only): no fist-separation push, the two hands share the hilt
LEFT_ARM = [IX[n] for n in P.names if n.startswith('Left') and ('Arm' in n or 'Hand' in n or n == 'LeftShoulder')]

A = ACT
# NPC combo parts pN end AT the impact (strike = last frames): the follow-through comes from the NPC continuation clip (pN_miss /
# *_miss_to_idle) as a second donor part; donor_strike searches the strike in the first part only.
if WEAPON == 'knife':
    SPEC = {
        'combo_attack_01': dict(kind='attack', donors=[(A, 'melee_combo_close_01_p1'), (A, 'melee_combo_close_01_p1_miss')], note='left-to-right slash'),
        'combo_attack_02': dict(kind='attack', donors=[(A, 'melee_combo_infinite_startup_01')], note='right-to-left descending slash'),
        'combo_attack_03': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_05')], note='rising backhand slash'),
        'combo_attack_04': dict(kind='attack', donors=[(A, 'melee_combo_infinite_startup_02')], note='right-to-left slash'),
        'final_attack_03': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_02')], lunge=True, note='rising forehand slash, lunge'),
        'final_attack_04': dict(kind='attack', donors=[(A, 'melee_combo_02_p1_from_idle')], lunge=True, note='lunging horizontal slash'),
        'final_attack_05': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_02')], lunge=True,
                                note='rising forehand slash, lunge (vanilla final_attack_05 data == final_attack_03; infinite_attack_04 failed the start step)'),
        'heavy_attack_01': dict(kind='heavy', donors=[(A, 'melee_combo_01_p1_from_idle'), (A, 'melee_combo_01_p1_miss')], note='power stab from the hold pose'),
        'heavy_attack_02': dict(kind='heavy', donors=[(A, 'melee_combo_01_p4'), (A, 'melee_combo_01_p4_miss')],
                                note='power left-to-right slash from the hold pose (melee_combo_02_p2 failed: 21 m/s wrist, rot steps)'),
        'safe_attack': dict(kind='attack', donors=[(A, 'melee_combo_close_01_p1'), (A, 'melee_combo_close_01_p1_miss')],
                            note='left-to-right slash (vanilla data == combo_attack_01)'),
        'equip_attack': dict(kind='attack', donors=[(A, 'melee_combo_infinite_startup_01')], note='descending slash (vanilla data == combo_attack_02)'),
        'sprint_attack': dict(kind='attack', donors=[(A, 'melee_charge_01_sprint_to_attack'), (A, 'melee_charge_01_hit_to_idle')], lunge=True,
                              note='charge stab out of the sprint, lunge'),
        'jump_attack': dict(kind='attack', donors=[(A, 'melee_jump_attack_01_jump_to_attack'), (A, 'melee_jump_attack_01_hit_to_idle')],
                            calm_hips=True, note='downward reverse-grip stab (NPC grip flip; airborne legs: DLL; donor jump not in the hips)'),
        'crouch_attack': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_01')], note='descending slash (standing guard; crouch: DLL)'),
    }
    DEFLECT = {'to_right': (A, 'deflect_rightarm_to_right'), 'to_left': (A, 'deflect_rightarm_to_left')}
else:
    SPEC = {
        'combo_attack_01': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_01')], note='right-to-left descending cut'),
        'combo_attack_02': dict(kind='attack', donors=[(A, 'melee_combo_01_p3'), (A, 'melee_combo_01_p3_miss')], note='left-to-right rising cut'),
        'combo_attack_03': dict(kind='attack', donors=[(A, 'melee_combo_close_01_p3'), (A, 'melee_combo_close_01_p3_miss')], note='right-to-left rising cut'),
        'combo_attack_04': dict(kind='attack', donors=[(A, 'melee_heavy_01_idle_to_attack'), (A, 'melee_heavy_01_miss_to_idle')],
                                note='left-to-right cut (vanilla combo_attack_04 data == final_attack_04; melee_combo_close_01_p2 has no follow-through)'),
        'final_attack_03': dict(kind='attack', donors=[(A, 'melee_combo_close_01_p1'), (A, 'melee_combo_close_01_p1_miss')], lunge=True,
                                note='right-to-left cut, lunge'),
        'final_attack_04': dict(kind='attack', donors=[(A, 'melee_heavy_01_idle_to_attack'), (A, 'melee_heavy_01_miss_to_idle')], lunge=True,
                                note='power horizontal cut, lunge'),
        'final_attack_05': dict(kind='attack', donors=[(A, 'melee_combo_infinite_attack_03')], lunge=True, note='descending diagonal cut, lunge'),
        'heavy_attack_01': dict(kind='heavy', donors=[(A, 'melee_jump_attack_01_jump_to_attack'), (A, 'melee_jump_attack_01_hit_to_idle')],
                                calm_hips=True, note='overhead chop from the hold pose (donor jump not in the hips)'),
        'heavy_attack_02': dict(kind='heavy', donors=[(A, 'melee_combo_infinite_attack_02')], note='forward downward cut from the hold pose'),
        'safe_attack': dict(kind='attack', donors=[(A, 'melee_combo_02_p3'), (A, 'melee_combo_02_p3_miss')], note='left-to-right cut'),
        'equip_attack': dict(kind='attack', donors=[(A, 'melee_combo_01_p3'), (A, 'melee_combo_01_p3_miss')],
                             note='left-to-right rising cut (melee_combo_01_p2 has no follow-through)'),
        'sprint_attack': dict(kind='attack', donors=[(A, 'melee_charge_01_sprint_to_attack'), (A, 'melee_charge_01_hit_to_idle')], lunge=True,
                              note='charge cut out of the sprint, lunge'),
        'jump_attack': dict(kind='attack', donors=[(A, 'melee_jump_attack_01_jump_to_attack'), (A, 'melee_jump_attack_01_hit_to_idle')],
                            calm_hips=True, note='overhead chop (airborne legs: DLL; donor jump not in the hips)'),
        'crouch_attack': dict(kind='attack', donors=[(A, 'melee_combo_02_p3'), (A, 'melee_combo_02_p3_miss')],
                              note='left-to-right cut (standing guard; crouch: DLL; melee_combo_01_p2 has no follow-through)'),
        'block_attack': dict(kind='attack', donors=[(A, 'melee_heavy_01_idle_to_attack'), (A, 'melee_heavy_01_miss_to_idle')], start='block',
                             note='horizontal cut out of the block'),
        'block_start': dict(kind='block_start', donors=[(A, 'idle_block')]),
        'block': dict(kind='block', donors=[(A, 'idle_block')]),
        'block_end': dict(kind='block_end', donors=[(A, 'idle_block')]),
        'block_deflect_left': dict(kind='react', start='block', donors=[(A, 'block_left')], note='NPC block hit reaction, back to the guard'),
        'block_deflect_right': dict(kind='react', start='block', donors=[(A, 'block_right')], note='NPC block hit reaction, back to the guard'),
    }
    DEFLECT = {'to_right': (A, 'deflect_rightarm_to_right'), 'to_left': (A, 'deflect_rightarm_to_left')}
BASES = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04', 'final_attack_03', 'final_attack_04', 'final_attack_05',
         'heavy_attack_01', 'heavy_attack_02', 'safe_attack', 'equip_attack', 'sprint_attack', 'jump_attack', 'crouch_attack']
SPEC.update({
    'melee_idlepose': dict(kind='pose'),
    'melee_safepose': dict(kind='safepose', donors=[(A, 'ovr_idle')]),
    'guard_idle_loop': dict(kind='idle_loop', donors=[(A, 'idle_combat')], frames=120),
    'hold_start_01': dict(kind='hold_start', of='heavy_attack_01'),
    'hold_loop_01': dict(kind='hold_loop', of='heavy_attack_01'),
    'hold_loop_02': dict(kind='hold_loop', of='heavy_attack_02'),
    'hold_start_offscreen_01': dict(kind='hold_start', of='heavy_attack_01', start='safepose'),
    'hold_start_offscreen_02': dict(kind='hold_start', of='heavy_attack_02', start='safepose'),
    'melee_equip': dict(kind='equip', donors=[(A, 'ovr_equip')]),
    'melee_unequip': dict(kind='unequip', donors=[(A, 'ovr_unequip')]),
    'parried_left': dict(kind='react', start='auto', donors=[DEFLECT['to_left']], note='weapon arm knocked away'),
    'parried_right': dict(kind='react', start='auto', donors=[DEFLECT['to_right']], note='weapon arm knocked away'),
})
for _b in BASES:
    SPEC[_b + '_hit'] = dict(kind='hit', of=_b)
    SPEC[_b + '_deflect'] = dict(kind='react', start=(_b, None), donors='deflect', note='weapon arm deflected')
ORDER = (['guard_idle_loop', 'melee_idlepose', 'melee_safepose'] + (['block', 'block_start', 'block_end'] if TWO else [])
         + BASES + (['block_attack'] if TWO else [])
         + ['hold_start_01', 'hold_loop_01', 'hold_loop_02', 'hold_start_offscreen_01', 'hold_start_offscreen_02', 'melee_equip', 'melee_unequip',
            'parried_left', 'parried_right'] + (['block_deflect_left', 'block_deflect_right'] if TWO else [])
         + [b + v for b in BASES for v in ('_hit', '_deflect')])
LOOPS = ('hold_loop_01', 'hold_loop_02', 'guard_idle_loop') + (('block',) if TWO else ())
CDIR = None
HOLDS = {}
SPEEDUP = 1.5
EASE0, EASE_FR = 0.35, 3.0
EASE_MODE = 'time'       # per try: 'time' (time-map ease-in) or 'pose' (start_ease re-timing of the upper body, frames 0-4)      # attack time map: first donor steps at 35 % of the nominal pre-impact speed, full speed after 3 frames
SPEED_TRIES = (1.5, 1.35, 1.2, 1.05, 0.9)


# ------------------------------------------------------------------ two-hand grip (katana)
def rel_blend(Ma, Mb, w):
    """per-frame blend of two relative transforms [T,4,4] (slerp rotation, lerp translation), w [T] (0 = Ma)."""
    qa = R.from_matrix(Ma[:, :3, :3]).as_quat()
    qb = R.from_matrix(Mb[:, :3, :3]).as_quat()
    q = NB.qslerp(qa, qb, w)
    M = np.repeat(np.eye(4)[None], len(w), 0)
    M[:, :3, :3] = R.from_quat(q).as_matrix()
    M[:, :3, 3] = Ma[:, :3, 3] * (1 - w[:, None]) + Mb[:, :3, 3] * w[:, None]
    return M


def smooth_w(w):
    w = np.asarray(w, float)
    return np.clip(NB.smooth(NB.dilate(w, 1), 1.5), 0, 1) if len(w) > 4 else w


def two_hand_ik(lt, lr, relM, w):
    """left hand -> V's WeaponRight x relM (position by arm hinge IK, then world rotation), weight w [T]. -> max residual (cm, w > 0.5)."""
    if not TWO or w.max() <= 1e-3:
        return 0.0
    w = NB.sstep((np.asarray(w, float) - 0.2) / 0.3)     # full IK from w 0.5 on (hand on the hilt), release / grasp over w 0.2..0.5
    W = NB.fk(lt, lr)
    tgt = W[:, BC.WR] @ relM
    # reach: V's left arm is shorter than the NPC's relative to the weapon reach -> where the hilt is out of the left hand's reach (97 % of
    # the arm length from the shoulder) the weapon hand moves towards the left shoulder by the deficit (smoothed in time, arm hinge IK)
    sh = W[:, IX['LeftArm'], :3, 3]
    arm = (np.linalg.norm(W[0, IX['LeftForeArm'], :3, 3] - W[0, IX['LeftArm'], :3, 3])
           + np.linalg.norm(W[0, IX['LeftHand'], :3, 3] - W[0, IX['LeftForeArm'], :3, 3]))
    dv = tgt[:, :3, 3] - sh
    dist = np.linalg.norm(dv, axis=1)
    deficit = np.maximum(dist - 0.97 * arm, 0) * (w > 1e-3)
    if deficit.max() > 1e-4:
        T = len(lt)
        prof = np.maximum(NB.smooth(NB.dilate(deficit, 2), 1.5), deficit) if T > 4 else deficit
        u = -dv / np.maximum(dist, 1e-9)[:, None]
        rh = W[:, IX['RightHand'], :3, 3]
        HI.solve(NB.fk, NB.PAR, NB.REST_LR, lt, lr, IX['RightArm'], IX['RightForeArm'], IX['RightHand'], rh + u * prof[:, None],
                 weight=(prof > 1e-5).astype(float), end_mode='world')
        W = NB.fk(lt, lr)
        tgt = W[:, BC.WR] @ relM
    HI.solve(NB.fk, NB.PAR, NB.REST_LR, lt, lr, IX['LeftArm'], IX['LeftForeArm'], IX['LeftHand'], tgt[:, :3, 3], weight=w, end_mode='world')
    W = NB.fk(lt, lr)
    q0 = R.from_matrix(W[:, IX['LeftHand'], :3, :3]).as_quat()
    q1 = R.from_matrix(tgt[:, :3, :3]).as_quat()
    NB.set_world_rot(lt, lr, IX['LeftHand'], R.from_quat(NB.qslerp(q0, q1, w)).as_matrix())
    NB.wrist_to_forearm(lt, lr, 'Left')
    return hilt_err(NB.fk(lt, lr), relM, w)


def hilt_err(W, relM, w):
    if not TWO or w.max() <= 0.5:
        return 0.0
    tgt = W[:, BC.WR] @ relM
    e = np.linalg.norm(tgt[:, :3, 3] - W[:, IX['LeftHand'], :3, 3], axis=1) * 100
    return round(float(e[w > 0.5].max()), 2)


# ------------------------------------------------------------------ post-ramp corrections (frame 0 / last frame locked)
def lock_w(T, ends=True):
    """correction weight [T]: 0 on frame 0 and the last frame (start / end poses stay exact), smooth fade over 2 frames."""
    t = np.arange(T, dtype=float)
    if T <= 2 or not ends:
        return np.ones(T)
    return NB.sstep(t / 2.0) * NB.sstep((T - 1 - t) / 2.0)


SWIVEL_GRID = [10, -10, 20, -20, 30, -30, 45, -45, 60, -60]
ARM_FEATS = {s: [k for k in NB.AL if k.endswith('_' + s[0]) and any(p in k for p in ('uparm', 'elbow', 'forearm', 'wrist', 'hand'))]
             for s in ('Left', 'Right')}


def arm_swivel(lt, lr, side, phi_deg):
    """rotate the whole arm rigidly about the shoulder -> wrist axis by phi [T] deg (elbow moves on its circle); wrist position and hand
    world rotation kept (weapon unchanged)."""
    W = NB.fk(lt, lr)
    a, h = IX[side + 'Arm'], IX[side + 'Hand']
    u = W[:, h, :3, 3] - W[:, a, :3, 3]
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    Rh = W[:, h, :3, :3].copy()
    Rr = R.from_rotvec(u * np.radians(phi_deg)[:, None]).as_matrix()
    NB.set_world_rot(lt, lr, a, Rr @ W[:, a, :3, :3])
    NB.set_world_rot(lt, lr, h, Rh)


def swivel_fix(lt, lr, L, log):
    """per arm, only where V's base gate fails on an arm feature (raw NPC upper-arm twist reaches 117 deg vs the 105 deg limit): the elbow
    swivel (grid, mildest first) that lowers the arm violation most, smoothed in time, weighted by the lock L."""
    T = len(lt)
    for side in ('Left', 'Right'):
        feats = ARM_FEATS[side]
        bv = NB.viol_of(NB.fk(lt, lr), feats)
        if not (bv > 1e-9).any():
            continue
        v0 = bv.copy()
        cand = {}
        for sg in (1.0, -1.0):          # one swivel direction per arm and clip (no sign flip between frames)
            best, bvs = np.zeros(T), v0.copy()
            for g in (10, 20, 30, 45, 60):
                lt2, lr2 = lt.copy(), lr.copy()
                arm_swivel(lt2, lr2, side, np.full(T, sg * g) * L)
                NB.wrist_to_forearm(lt2, lr2, side)
                v = NB.viol_of(NB.fk(lt2, lr2), feats)
                imp = v < bvs - 1e-6
                best[imp] = g
                bvs = np.where(imp, v, bvs)
            cand[sg] = (float(bvs.sum()), best)
        sgn = 1.0 if cand[1.0][0] <= cand[-1.0][0] else -1.0
        best = cand[sgn][1]
        phi = sgn * np.maximum(NB.smooth(NB.dilate(best, 2), 1.5), best) * L
        if np.abs(phi).max() > 0:
            arm_swivel(lt, lr, side, phi)
            NB.wrist_to_forearm(lt, lr, side)
        log['swivel_max_deg_' + side[0]] = round(float(np.abs(phi).max()), 1)


def blade_push(lt, lr, relM, w2h, L, log):
    """blade proxy inside V's body proxy: move the hand away along the contact normal (arm hinge IK, hand world rotation kept), smoothed
    in time, weighted by L.  Off-hand contact (one-hand frames) moves the off hand; any other contact moves the weapon hand (katana: the
    left hand follows the hilt again)."""
    T = len(lt)
    moved = 0.0
    z0, z1, _ = BC.BLADE[WEAPON]
    for it in range(4):
        W = NB.fk(lt, lr)
        cnt, clr, bp, qp, lb = BC.blade_pen(W, WEAPON, w2h, detail=True)
        depth = np.maximum(-clr / 100.0 + 0.012, 0) * (cnt > 0)
        if not depth.any():
            break
        off = np.array([str(x).startswith('Left') for x in lb]) & (w2h < 0.5)
        arm = off & np.array([('Arm' in str(x)) for x in lb])      # off-arm contact (fore / upper arm): the hand moves by the lever ratio
        if arm.any():
            sh = W[:, IX['LeftArm'], :3, 3]
            lev = np.linalg.norm(W[:, IX['LeftHand'], :3, 3] - sh, axis=1) / np.maximum(np.linalg.norm(qp - sh, axis=1), 0.08)
            depth = np.where(arm, depth * np.clip(lev, 1.0, 3.0), depth)
        n = qp - bp
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
        if off.any():           # off hand: away from the weapon (weapon fist centre -> blade tip) through the off-hand centre
            fcs = NB.fist_centers(W)
            hc = fcs['Left']
            M = W[:, BC.WR]
            a = fcs['Right']
            ab = (M[:, :3, 3] + z1 * M[:, :3, 2]) - a
            u = np.clip(np.sum((hc - a) * ab, 1) / np.sum(ab * ab, 1), 0, 1)
            dv = hc - (a + u[:, None] * ab)
            dv /= np.maximum(np.linalg.norm(dv, axis=1, keepdims=True), 1e-9)
            n[off] = -dv[off]
        for side, mask, sgn in (('Right', ~off, 1.0), ('Left', off, -1.0)):
            dm = depth * mask
            if not dm.any():
                continue
            prof = NB.smooth(NB.dilate(dm, 2), 1.5)
            prof = np.maximum(prof, dm) * L
            k = np.where(dm > 0)[0]
            nn = n[[k[np.argmin(np.abs(k - t))] for t in range(T)]]
            hnd = W[:, IX[side + 'Hand'], :3, 3]
            tgt = hnd + sgn * nn * prof[:, None]
            if side == 'Left' and not TWO:       # the off hand keeps the fist gap to the weapon fist (no hand-in-hand)
                fcs = NB.fist_centers(W)
                fl = fcs['Left'] + sgn * nn * prof[:, None]
                dv = fl - fcs['Right']
                dn_ = np.linalg.norm(dv, axis=1)
                gap = 2 * NB.FIST_R + 0.012
                deficit = NB.smooth(NB.dilate(np.maximum(gap - dn_, 0), 2), 1.5) if T > 4 else np.maximum(gap - dn_, 0)   # smooth in time (no 1-frame pop)
                tgt = tgt + dv / np.maximum(dn_, 1e-9)[:, None] * deficit[:, None]
            HI.solve(NB.fk, NB.PAR, NB.REST_LR, lt, lr, IX[side + 'Arm'], IX[side + 'ForeArm'], IX[side + 'Hand'], tgt,
                     weight=(prof > 1e-5).astype(float), end_mode='world')
            moved = max(moved, float(prof.max()))
        if TWO:
            two_hand_ik(lt, lr, relM, w2h)
    log['blade_push_cm'] = round(moved * 100, 1)


def elbow_hinge(lt, lr, L, log):
    """elbow_dev outside V's range (non-hinge NPC elbow): the deviation of the forearm (ZYX y, as hinge_ik) is removed with a smooth weight,
    the wrist goes back to its position by arm hinge IK, hand world rotation kept."""
    for side in ('Left', 'Right'):
        v = NB.viol_of(NB.fk(lt, lr), ['elbow_dev_' + side[0]])
        if not (v > 1e-9).any():
            continue
        m = np.clip(NB.smooth(NB.dilate((v > 1e-9).astype(float), 2), 1.5) * 1.5, 0, 1) * L
        W = NB.fk(lt, lr)
        h, b = IX[side + 'Hand'], IX[side + 'ForeArm']
        tgt, Rh = W[:, h, :3, 3].copy(), W[:, h, :3, :3].copy()
        Lr = R.from_quat(np.repeat(NB.REST_LR[b][None], len(lt), 0))
        e = (Lr.inv() * R.from_quat(lr[:, b])).as_euler('ZYX')
        e[:, 1] *= (1 - m)
        lr[:, b] = (Lr * R.from_euler('ZYX', e)).as_quat()
        HI.solve(NB.fk, NB.PAR, NB.REST_LR, lt, lr, IX[side + 'Arm'], b, h, tgt, weight=m, end_mode='world')
        NB.set_world_rot(lt, lr, h, Rh)
        log['elbow_hinge_' + side[0]] = round(float(m.max()), 2)


def start_ease(lt, lr, K=4, s0=0.3):
    """re-time frames 0..K (all joints) with h(u) = s0 u + 2(1-s0) u^2 - (1-s0) u^3: frame 0 and frame K unchanged, the first step is
    ~s0 of the original one (no full-speed start out of the guard / hold pose)."""
    if len(lt) <= K + 2:
        return lt, lr
    u = np.arange(K + 1) / K
    fr = K * (s0 * u + 2 * (1 - s0) * u ** 2 - (1 - s0) * u ** 3)
    a, b = NB.resample(lt, lr, fr)
    lt, lr = lt.copy(), lr.copy()
    U = NB.UPPER                   # upper body only: hips / legs stay on the planned capsule-relative foot timeline
    lt[:K + 1, U], lr[:K + 1, U] = a[:, U], b[:, U]
    return lt, NB.qcont(lr)


SKIP = set((os.environ.get('UTPP_BLADE_SKIP') or '').split(','))      # debug switch: skip post-fix steps (light, hinge, swivel, push)


def light_fix(lt, lr, L, log):
    """anatomy fixes of npc_build.anatomy_fix WITHOUT the hips follow (spine / neck / clavicle towards V's base body, clavicle retract /
    elevate) on a copy, blended in by the lock L on the upper body (frame 0 / last frame unchanged); wrist twist into the forearm."""
    T = len(lt)
    if not (NB.viol_of(NB.fk(lt, lr), sorted({f for g in NB.FIX_GROUPS for f in g[1]})) > 1e-9).any():
        return
    lt2, lr2 = lt.copy(), lr.copy()
    for rnd in range(2):
        for name, feats, joints, keep in NB.FIX_GROUPS:
            s, _ = NB.choose_scale(lt2, lr2, lambda a, b, s, j=joints, k=keep: NB.to_base(a, b, j, s, k), feats, NB.BASE_GRID)
            s = NB.tsmooth(s, T)
            if s.max() > 0:
                NB.to_base(lt2, lr2, joints, s, keep)
    for sd in ('Left', 'Right'):
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        for fn, grid in ((NB.clav_retract, NB.RETRACT_GRID), (NB.clav_elevate, NB.ELEV_GRID)):
            a_, _ = NB.choose_scale(lt2, lr2, lambda a, b, s, sd=sd, fn=fn: fn(a, b, sd, s), feats, grid)
            a_ = NB.tsmooth(a_ / grid[-1], T) * grid[-1]
            if a_.max() > 0:
                fn(lt2, lr2, sd, a_)
    U = NB.UPPER
    lr[:, U] = NB.qslerp(lr[:, U], lr2[:, U], np.repeat(L[:, None], len(U), 1))
    for sd in ('Left', 'Right'):
        NB.wrist_to_forearm(lt, lr, sd)
    log['light_fix'] = True


ROLL_GRID = (0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0)
ROLL_FEATS = ['wrist_flex_L', 'wrist_dev_L', 'wrist_twist_L', 'pronation_L', 'forearm_roll_L', 'elbow_flex_L', 'uparm_twist_L', 'elbow_torso_cm_L',
              'forearm_torso_cm_L', 'upperarm_torso_cm_L']


def hilt_roll(lt, lr, relM, w2h, log, L=None):
    """katana: the left hand may turn around the hilt axis (WeaponRight +Z through its origin): per frame the roll of ROLL_GRID with the
    smallest left-arm violation after the hilt IK (the mildest roll on ties), smoothed in time; relM is rotated IN PLACE (target and the
    saved / gated relation stay consistent)."""
    if not TWO or w2h.max() <= 1e-3:
        return
    T = len(lt)
    vs = []
    for th in ROLL_GRID:
        Rz = np.eye(4)
        Rz[:3, :3] = R.from_rotvec([0, 0, np.radians(th)]).as_matrix()
        lt2, lr2 = lt.copy(), lr.copy()
        two_hand_ik(lt2, lr2, np.einsum('ij,tjk->tik', Rz, relM), w2h)
        vs.append(NB.viol_of(NB.fk(lt2, lr2), ROLL_FEATS) + 1e-4 * abs(th))
    vs = np.array(vs)
    th = np.array(ROLL_GRID)[np.argmin(vs, 0)] * (w2h > 1e-3)
    th = NB.smooth(th, 1.5) if T > 4 else th
    if L is not None:
        th = th * L               # frame 0 / last frame: the guard's grip (no roll), no pop against melee_idlepose
    rot = R.from_rotvec(np.stack([np.zeros(T), np.zeros(T), np.radians(th)], 1)).as_matrix()
    relM[:, :3, :3] = np.einsum('tij,tjk->tik', rot, relM[:, :3, :3])
    relM[:, :3, 3] = np.einsum('tij,tj->ti', rot, relM[:, :3, 3])
    log['hilt_roll_deg'] = [round(float(th.min()), 1), round(float(th.max()), 1)]


def post_fix(lt, lr, relM, w2h, L, log):
    """fixes after the build, weighted by the lock L (frame 0 / last frame keep the start / guard pose); the hilt IK of the katana left hand
    is NOT locked (the hand stays on the hilt on every frame; the guard / start poses are on the hilt already)."""
    if 'light' not in SKIP:
        light_fix(lt, lr, L, log)
    if 'roll' not in SKIP:
        hilt_roll(lt, lr, relM, w2h, log, L)
    log['hilt_err_cm'] = two_hand_ik(lt, lr, relM, w2h)
    if 'hinge' not in SKIP:
        elbow_hinge(lt, lr, L, log)
    if 'swivel' not in SKIP:
        swivel_fix(lt, lr, L, log)
    if 'push' not in SKIP:
        blade_push(lt, lr, relM, w2h, L, log)
    if TWO:
        two_hand_ik(lt, lr, relM, w2h)
        swivel_fix(lt, lr, L, log)
        two_hand_ik(lt, lr, relM, w2h)
        if 'flip' not in SKIP:
            flip_repair(lt, lr, relM, w2h, log)
        if 'wrist' not in SKIP:
            wrist_relax(lt, lr, w2h, log, L)


FLIP_JOINTS = ('LeftArm', 'LeftForeArm', 'LeftHand', 'RightArm', 'RightForeArm', 'RightHand')


def _qang(a, b):
    return np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(a * b, -1)), 0, 1)))


def flip_repair(lt, lr, relM, w2h, log):
    """katana: one- or two-frame twist flips of the arm chain (local rotation out by > 45 deg and back: the hilt IK / wrist twist
    distribution picks the other side of the 180 deg twist) -> the frames become the slerp between their neighbours; the left hand is put
    back on the hilt afterwards."""
    T = len(lt)
    fixed = []
    for name in FLIP_JOINTS:
        j = IX[name]
        for _ in range(4):
            q = NB.qcont(lr[:, j][:, None])[:, 0]
            done = False
            for t in range(1, T - 1):
                for k in (1, 2):
                    if t + k >= T:
                        continue
                    jump = _qang(q[t], q[t - 1])
                    if jump > 45.0 and _qang(q[t + k], q[t - 1]) < 0.5 * jump:
                        for i in range(k):
                            u = (i + 1.0) / (k + 1.0)
                            lr[t + i, j] = NB.qslerp(q[t - 1][None], q[t + k][None], np.array([u]))[0]
                        fixed.append((name, t, k, round(float(jump), 1)))
                        done = True
                        break
                if done:
                    break
            if not done:
                break
    if fixed:             # hand position back on the hilt by arm hinge IK, keeping the repaired hand world rotation (no new twist pick)
        w = NB.sstep((np.asarray(w2h, float) - 0.2) / 0.3)
        tgt = NB.fk(lt, lr)[:, BC.WR] @ relM
        HI.solve(NB.fk, NB.PAR, NB.REST_LR, lt, lr, IX['LeftArm'], IX['LeftForeArm'], IX['LeftHand'], tgt[:, :3, 3], weight=w, end_mode='world')
        log['flip_repair'] = fixed[:12]


WRIST_FEATS_L = ['wrist_flex_L', 'wrist_dev_L', 'wrist_twist_L', 'pronation_L']
WRIST_GRID = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)


def wrist_relax(lt, lr, w2h, log, L=None):
    """katana: where the left wrist still breaks V's limits with the hand on the hilt, the hand rotation (not its position) is slerped
    towards the neutral wrist (rest local) by the smallest WRIST_GRID share that clears the wrist features, smoothed in time; the hand stays
    on the hilt (position), only its angle on the hilt changes."""
    if not (NB.viol_of(NB.fk(lt, lr), WRIST_FEATS_L) > 1e-9).any():
        return
    j = IX['LeftHand']
    T = len(lt)

    def apply(a, b, s, j=j):
        b[:, j] = NB.qslerp(b[:, j], np.repeat(NB.REST_LR[j][None], len(b), 0), np.asarray(s, float))
        NB.wrist_to_forearm(a, b, 'Left')
    s_, _ = NB.choose_scale(lt, lr, apply, WRIST_FEATS_L, WRIST_GRID)
    s_ = np.maximum(NB.smooth(NB.dilate(s_, 1), 1.0), s_) * (np.asarray(w2h) > 1e-3) if T > 4 else s_
    if L is not None:
        s_ = s_ * L
    if s_.max() > 0:
        apply(lt, lr, s_)
    log['wrist_relax_max'] = round(float(s_.max()), 2)


# ------------------------------------------------------------------ guard
class BGuard:
    def __init__(self):
        sq = BC.BSeq([(ACT, 'idle_combat')])
        self.raw_lt, self.raw_lr = sq.lt[:1].copy(), sq.lr[:1].copy()
        W = NB.fk(self.raw_lt, self.raw_lr)
        self.feet = {s: W[0, IX[s + 'Foot'], :3, 3].copy() for s in ('Left', 'Right')}
        self.foot_R = {s: W[0, IX[s + 'Foot'], :3, :3].copy() for s in ('Left', 'Right')}
        self.toe = {s: self.raw_lr[0, IX[s + 'ToeBase']].copy() for s in ('Left', 'Right')}
        for s_ in ('Left', 'Right'):
            NB.TOE_LOCAL[s_] = self.foot_R[s_].T @ (W[0, IX[s_ + 'ToeBase'], :3, 3] - W[0, IX[s_ + 'Foot'], :3, 3])
        self.hips = W[0, IX['Hips'], :3, 3].copy()
        self.mid = (self.feet['Left'] + self.feet['Right']) / 2
        self.rel = sq.rel_L[:1].copy()
        self.two = float(sq.on2h[0]) if TWO else 0.0
        lt, lr = self.raw_lt.copy(), self.raw_lr.copy()
        log = {}
        NB.anatomy_fix(lt, lr, log)
        log['hilt_err_cm'] = two_hand_ik(lt, lr, self.rel, np.full(1, self.two))
        tr = {s: self.feet[s][None].copy() for s in self.feet}
        NB.solve_legs(lt, lr, tr, np.zeros((1, 3)), {s: self.foot_R[s][None] for s in self.feet}, self.toe)
        self.lt, self.lr, self.log = lt, lr, log
        self.W = NB.fk(lt, lr)[0]
        Wi = NB.BB.fkL(NB.BB.compose_base()[2])
        self.idle_feet = {s: Wi[IX[s + 'Foot'], :3, 3].copy() for s in ('Left', 'Right')}
        self.idle_hips = Wi[IX['Hips'], :3, 3].copy()

    def relT(self, T):
        return np.repeat(self.rel, T, 0)


# ------------------------------------------------------------------ vanilla
def vanilla_strike(name):
    d, ph = BC.vanilla(SET, name)
    W = MC.world_all(d)
    T = len(W)
    a, b = ph.get('active', (0, T - 1))
    f, v = BC.strike_frame(W, WEAPON, a, b)
    o = int(round(np.clip(f, a + 0.25 * (b - a), a + 0.75 * (b - a))))
    o = max(o, min(MIN_PRE, b - 1))
    return o, f, T, ph


def donor_strike(sq):
    lo, hi = (sq.loop[0] - 1, sq.loop[1] + 1) if sq.loop else (0, sq.parts[0]['frames'] - 1)
    return BC.strike_frame(sq.W, WEAPON, lo, hi)[0]


def resolve_donors(name, spec):
    if spec.get('donors') == 'deflect':
        b = built_clip(spec['start'][0])
        o = b['info'].get('impact_vanilla', len(b['W']) // 2)
        u, _ = BC.tip_dir(b['W'], WEAPON, int(o))
        return [DEFLECT['to_right'] if u[0] < 0 else DEFLECT['to_left']]
    return spec['donors']


# ------------------------------------------------------------------ attacks
FINGERS = [IX[n] for n in P.names if 'Hand' in n and n not in ('LeftHand', 'RightHand')]


def finger_end(lr, G, w_e):
    """finger joints blended to the guard's fingers with the end blend w_e (the guard blend of build_body leaves the donor fingers: the
    last frame differed from the guard by up to 7.3 cm at the fingertips -> a finger pop into the idle)."""
    T = len(lr)
    lr[:, FINGERS] = NB.qslerp(lr[:, FINGERS], np.repeat(G.lr[0][FINGERS][None], T, 0), np.repeat(np.asarray(w_e, float)[:, None], len(FINGERS), 1))


END_FR = 8.0


def end_match(lt, lr, G, relM, w2h, log):
    """the last frame = the guard (melee_idlepose): the residual of the upper body + fingers vs the guard at the last frame (left after the
    fixes of npc_build.finish, e.g. a 45 deg left-hand turn = 7.3 cm at the fingertips) fades in over the last END_FR frames (no pop
    into the idle); katana: the left hand back on the hilt."""
    T = len(lr)
    J = sorted(set(NB.UPPER) | set(FINGERS))
    w = NB.sstep((np.arange(T) - (T - 1 - END_FR)) / END_FR)[::-1]       # 1 at the last frame (reversed below)
    lr[:, J] = NB.offset_start(lr[::-1, J], G.lr[0][J], w)[::-1]
    lr[:] = NB.qcont(lr)
    if TWO:
        two_hand_ik(lt, lr, relM, w2h)
    log['end_match_frames'] = int(END_FR)


def start_hand_step(W):
    out = {}
    for s in ('Left', 'Right'):
        h = W[:, IX[s + 'Hand'], :3, 3]
        s01, s12 = np.linalg.norm(h[1] - h[0]) * 100, np.linalg.norm(h[2] - h[1]) * 100
        out[s] = (round(float(s01), 2), round(float(s12), 2), bool(s01 <= max(1.3 * s12, 1.0)))
    return out


def _attack_once(G, name, spec, lunge_start):
    o_imp, v_strike, T, ph = vanilla_strike(name)
    sq = BC.BSeq(spec['donors'])
    d_imp = donor_strike(sq)
    tau0 = max(0.0, d_imp - SPEEDUP * o_imp)
    tau = np.empty(T)
    e0_ = EASE0 if EASE_MODE == 'time' else 1.0
    ramp = e0_ + (1.0 - e0_) * NB.sstep((np.arange(o_imp) + 0.5) / EASE_FR)     # time-map ease-in: no full-speed first step
    cum = np.concatenate([[0.0], np.cumsum(ramp)]) / ramp.sum()
    for i in range(T):
        tau[i] = tau0 + (d_imp - tau0) * cum[i] if i <= o_imp else min(d_imp + (i - o_imp), sq.n - 1)
    i = np.arange(T, dtype=float)
    w_s = 1.0 - NB.sstep(i / max(o_imp - 1, 1))
    e0 = o_imp + 3
    e1 = min(T - 2, o_imp + GUARD_BY)
    w_e = NB.sstep((i - e0) / max(e1 - e0, 1))
    st = start_state(G, spec['start'], name) if spec.get('start') else None
    s_lr = HOLDS[name]['lr'] if spec['kind'] == 'heavy' else (st['lr'] if st else None)
    lt, lr = NB.build_body(G, sq, tau, w_s, w_e, start_lr=s_lr)
    c, cmodel = NB.capsule_path(spec['kind'], T, spec.get('lunge'), lunge_start)
    mode = 'lunge' if spec.get('lunge') else 'planted'
    tracks, steps = NB.plan_feet(T, c, G.feet, G.feet, mode, f_settle=int(e0 + 2), lunge_start=lunge_start)
    hips = NB.hips_path(G, sq, tau, tracks, c, w_e, **({'k_xy': 0.1, 'k_z': 0.0} if spec.get('calm_hips') else {}))
    if st:
        hips = hips + (st['hips'] - hips[0])[None] * (1.0 - NB.sstep(i / 8.0))[:, None]
    log = {}
    lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
    w0 = 1.0 - NB.sstep(i / NB.START_RAMP)
    tgt = HOLDS[name]['lr'] if spec['kind'] == 'heavy' else (st['lr'] if st else G.lr[0])
    lr[:, NB.UPPER] = NB.offset_start(lr[:, NB.UPPER], tgt[NB.UPPER], w0)
    lr = NB.qcont(lr)
    finger_end(lr, G, w_e)
    WT = [BC.WR, BC.WL]           # weapon attach points: NPC translations, start / end offset like the rotations (WeaponLeft popped 4 cm at f0)
    lt0 = st['lt'] if st else G.lt[0]
    lt[:, WT] += (lt0[WT] - lt[0, WT])[None] * w0[:, None, None]
    lt[:, WT] += (G.lt[0][WT] - lt[:, WT]) * w_e[:, None, None]
    for sd in ('Left', 'Right'):
        log['ramp_wrist_twist_moved_deg_' + sd[0]] = round(NB.wrist_to_forearm(lt, lr, sd), 1)
    relD, wD = sq.rel_at(tau)
    wD = smooth_w(wD)
    rel0 = st['rel'] if st else G.relT(1)
    w0h = st['two'] if st else G.two
    relM = rel_blend(rel_blend(relD, np.repeat(rel0, T, 0), w_s), G.relT(T), w_e)
    w2h = ((1 - w_s) * wD + w_s * w0h) * (1 - w_e) + w_e * G.two
    if EASE_MODE == 'pose' and 'ease' not in SKIP:
        lt, lr = start_ease(lt, lr)
    post_fix(lt, lr, relM, w2h, lock_w(T), log)
    end_match(lt, lr, G, relM, w2h, log)
    info = dict(ease=EASE_MODE, kind=spec['kind'], note=spec.get('note'), donors=sq.parts, seam_raw_deg=sq.seam_raw_deg, seam_deg=sq.seam_deg, donor_despiked=sq.despiked, impact_vanilla=o_imp, vanilla_tip_peak=v_strike,
                impact_donor=d_imp, tau=np.round(tau, 2).tolist(), speedup_pre_impact=round((d_imp - tau0) / max(o_imp, 1), 2),
                guard_blend=[int(e0), int(e1)], capsule=cmodel, feet_mode=mode, steps=steps, fixes=log, vanilla_phases=ph,
                two_hand_frames=int((w2h > 0.5).sum()))
    return lt, lr, c, info, relM, w2h


def attack_clip(G, name, spec, lunge_start):
    global SPEEDUP, EASE_MODE
    keep = SPEEDUP
    tries, res = [], []
    chosen = None
    try:
        for sp, em in [(s_, e_) for s_ in SPEED_TRIES for e_ in ('time', 'pose')]:
            SPEEDUP, EASE_MODE = sp, em
            lt, lr, c, info, relM, w2h = _attack_once(G, name, spec, lunge_start)
            W = NB.fk(lt, lr)
            oi = info['impact_vanilla']
            pre, post = NB.wrist_speed(W[:oi + 2]), NB.wrist_speed(W[oi + 1:])
            g = gates_b(W, c, False, w2h, spec['kind'], relM)
            tries.append(dict(speedup=sp, ease=em, spike_cm=g['spike_max_cm'], wrist_pre_m_s=round(pre, 1), wrist_post_m_s=round(post, 1),
                              base_gate_pass=g['base_gate_pass'], blade_pen_frames=g['blade_pen_frames'], start_step=g.get('start_hand_step_cm'),
                              gates_pass=g['PASS']))
            clean = g['PASS']
            res.append((clean, pre, (lt, lr, c, info, relM, w2h)))
            if clean and pre <= max(NB.WRIST_MAX_M_S, post):
                chosen = len(res) - 1
                break
    finally:
        SPEEDUP, EASE_MODE = keep, 'time'
    if chosen is None:
        ok = [k for k, r in enumerate(res) if r[0]]
        chosen = min(ok, key=lambda k: res[k][1]) if ok else len(res) - 1
    out = res[chosen][2]
    out[3]['speed_tries'] = tries
    out[3]['speed_chosen'] = tries[chosen]['speedup']
    return out


# ------------------------------------------------------------------ holds, poses, loops, block, equip
def hold_lr(G, k):
    lr = NB.hold_synth(G, 'Right', k)
    if TWO and G.two > 0:      # the synthesised wind-up moves the right hand: the left hand back on the hilt
        lt1, lr1 = G.lt.copy(), lr[None].copy()
        two_hand_ik(lt1, lr1, G.rel, np.full(1, G.two))
        lr = lr1[0]
    return lr


def other_clip(G, name, spec, lunge_start):
    kind = spec['kind']
    if kind == 'pose':
        T = 2
    elif kind == 'idle_loop':
        T = spec['frames'] + 1
    else:
        T = len(BC.vanilla(SET, name)[0]['W'])
    i = np.arange(T, dtype=float)
    c = np.zeros((T, 3))
    log, info = {}, dict(kind=kind)
    tracks = NB.static_tracks(G, T, c)
    relM, w2h = G.relT(T), np.full(T, G.two)
    if kind == 'pose':
        return np.repeat(G.lt, T, 0), np.repeat(G.lr, T, 0), c, dict(kind=kind, source='guard pose (NPC idle_combat frame 0, fixed)'), relM, w2h
    if kind == 'safepose':
        sq = BC.BSeq(spec['donors'])
        lt, lr = NB.resample(sq.lt, sq.lr, np.zeros(T))
        lr = NB.blend_to(lr, G.lr[0], np.full(T, 0.4))
        hips = NB.hips_path(G, sq, np.zeros(T), tracks, c, np.ones(T))
        lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
        relD, wD = sq.rel_at(np.zeros(T))
        w2h = np.full(T, float(wD[0]))
        relM = relD
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        info.update(source='NPC ovr_idle frame 0 (weapon lowered) blended 40 % towards the guard', fixes=log)
        return lt, lr, c, info, relM, w2h
    if kind == 'idle_loop':
        sq = BC.BSeq(spec['donors'])
        best = None
        for L in range(60, min(sq.n - 1, 108) + 1):
            e = float(np.mean(np.abs(np.sum(sq.lr[0, NB.BODY_IX] * sq.lr[L, NB.BODY_IX], -1))))
            if best is None or e > best[0]:
                best = (e, L)
        L = best[1]
        tau = np.linspace(0, L, T)
        lt, lr = NB.resample(sq.lt, sq.lr, tau)
        lr = NB.qslerp(np.repeat(sq.lr[:1], T, 0), lr, np.full(lr.shape[:2], NB.IDLE_AMP))
        lr = NB.circ_smooth(NB.loop_close(NB.qcont(lr)), 2.0)
        lr[:, IX['Root']] = [0, 0, 0, 1.0]
        lt[:, IX['Root']] = 0
        hips = NB.hips_path(G, sq, tau, tracks, c, None, k_xy=0.35 * NB.IDLE_AMP, k_z=0.6 * NB.IDLE_AMP)
        hips = hips - np.linspace(0, 1, T)[:, None] * (hips[-1] - hips[0])[None]
        lt, lr = NB.finish_loop(G, lt, lr, hips, tracks, c, log)
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        lt[-1], lr[-1] = lt[0], lr[0]
        info.update(source='NPC idle_combat frames 0..%d resampled to %d, deviation x %.2f, loop closed + circular smoothing, feet planted' % (
            L, T, NB.IDLE_AMP), fixes=log)
        return lt, lr, c, info, relM, w2h
    if kind == 'hold_start':
        h = HOLDS[spec['of']]
        w = NB.sstep(i / (T - 1))
        st = start_state(G, spec['start'], name) if spec.get('start') else dict(lr=G.lr[0], hips=G.hips, lt=G.lt[0], rel=G.rel, two=G.two)
        lt = np.repeat(st['lt'][None], T, 0)
        lr = NB.qslerp(np.repeat(st['lr'][None], T, 0), np.repeat(h['lr'][None], T, 0), np.repeat(w[:, None], G.lr.shape[1], 1))
        lr[-1] = h['lr']
        hips = st['hips'][None] + (G.hips - st['hips'])[None] * w[:, None]
        lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
        relM = rel_blend(np.repeat(st['rel'], T, 0), G.relT(T), w)
        w2h = st['two'] * (1 - w) + G.two * w
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        info.update(source='%s -> hold pose of %s (synthesised wind-up from the guard: chest %.0f deg, weapon hand pulled back %.0f cm)' % (
            spec.get('start', 'guard'), spec['of'], NB.HOLD_WIND['Right'], NB.HOLD_PULL_CM), fixes=log)
        return lt, lr, c, info, relM, w2h
    if kind == 'hold_loop':
        h = HOLDS[spec['of']]
        u = 0.5 * (1.0 - np.cos(2 * np.pi * i / (T - 1)))
        lt = np.repeat(G.lt, T, 0)
        lr = NB.qslerp(np.repeat(h['lr'][None], T, 0), np.repeat(h['lr2'][None], T, 0), np.repeat(u[:, None], G.lr.shape[1], 1))
        lr[0] = lr[-1] = h['lr']
        lt, lr = NB.finish_loop(G, lt, lr, np.repeat(G.hips[None], T, 0), tracks, c, log)
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        lt[-1], lr[-1] = lt[0], lr[0]
        info.update(source='hold pose of %s with a cosine pulse to %.0f%% more wind-up (closed loop)' % (spec['of'], 100 * NB.HOLD_PULSE), fixes=log)
        return lt, lr, c, info, relM, w2h
    sq = BC.BSeq(spec['donors'])
    if kind in ('block_start', 'block', 'block_end'):
        cover_f = 0.0
        relB, wB = sq.rel_at(np.full(T, cover_f))
        if kind == 'block_start':
            w = NB.sstep(i / (T * 0.6))
            lt, lr = NB.build_body(G, sq, np.full(T, cover_f), None, None)
            lr = NB.blend_to(lr, G.lr[0], 1 - w)
            relM, w2h = rel_blend(G.relT(T), relB, w), G.two * (1 - w) + wB * w
        elif kind == 'block':
            lt, lr = NB.build_body(G, sq, np.full(T, cover_f), None, None)
            relM, w2h = relB, wB
        else:
            w = NB.sstep((i - 2) / (T * 0.7))
            lt, lr = NB.build_body(G, sq, np.full(T, cover_f), None, w)
            relM, w2h = rel_blend(relB, G.relT(T), w), wB * (1 - w) + G.two * w
        hips = NB.hips_path(G, sq, np.full(T, cover_f), tracks, c, None)
        lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        if kind == 'block':
            lt[-1], lr[-1] = lt[0], lr[0]
        info.update(source='NPC idle_block frame 0 %s' % {'block_start': 'blended in from the guard over 60 %', 'block': 'held',
                                                         'block_end': 'blended out to the guard'}[kind], fixes=log)
        return lt, lr, c, info, relM, w2h
    if kind in ('equip', 'unequip'):
        tau = np.linspace(0, sq.n - 1, T)
        sq.lt[:, BC.WR] = G.lt[0, BC.WR]          # the weapon stays in the hand with the guard grip (the NPC draws it from a holster)
        sq.lr[:, BC.WR] = G.lr[0, BC.WR]
        sq.W = NB.fk(sq.lt, sq.lr)
        if kind == 'equip':
            lt, lr = NB.build_body(G, sq, tau, None, NB.sstep((i - T * 0.55) / (T * 0.4)))
            start, end = G.idle_feet, G.feet
            fs = {'Left': (int(T * 0.15), int(T * 0.15) + 7), 'Right': (int(T * 0.4), int(T * 0.4) + 7)}
            off = (G.idle_hips - (G.idle_feet['Left'] + G.idle_feet['Right']) / 2) - (G.hips - G.mid)
            hw = (off, 1 - NB.sstep(i / (T * 0.8)))
            w2h = G.two * NB.sstep((i - T * 0.6) / (T * 0.3))
        else:
            lt, lr = NB.build_body(G, sq, tau, 1 - NB.sstep(i / (T * 0.35)), None)
            start, end = G.feet, G.idle_feet
            fs = {'Right': (int(T * 0.3), int(T * 0.3) + 5), 'Left': (int(T * 0.55), min(T - 1, int(T * 0.55) + 5))}
            off = (G.idle_hips - (G.idle_feet['Left'] + G.idle_feet['Right']) / 2) - (G.hips - G.mid)
            hw = (off, NB.sstep(i / (T * 0.9)))
            w2h = G.two * (1 - NB.sstep(i / (T * 0.3)))
        tracks, steps = NB.plan_feet(T, c, start, end, 'transfer', f_settle=fs)
        hips = NB.hips_path(G, sq, tau, tracks, c, np.ones(T), start=hw)
        lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
        post_fix(lt, lr, relM, w2h, lock_w(T, kind != 'safepose'), log)
        info.update(source='%s resampled %d -> %d frames (%.2fx), weapon kept in the hand (guard grip), feet step between V idle stance and the guard' % (
            sq.parts[0]['clip'], sq.n, T, (sq.n - 1) / (T - 1)), steps=steps, fixes=log)
        return lt, lr, c, info, relM, w2h
    raise ValueError(kind)


# ------------------------------------------------------------------ follow-on clips
_BUILT = {}


def built_clip(clip):
    if clip not in _BUILT:
        base = os.path.join(CDIR, '%s__%s.%s' % (SET, clip, SUFFIX))
        z = np.load(base + '.npz')
        lt, lr = z['lt'].astype(float), NB.qcont(z['lr'].astype(float))
        th = np.load(base + '.twohand.npz')
        _BUILT[clip] = dict(lt=lt, lr=lr, W=NB.fk(lt, lr), info=json.load(open(base + '.info.json')), rel=th['rel'], w=th['w'])
    return _BUILT[clip]


def branch_frame(name, clip):
    a, b = MC.world_all(BC.vanilla(SET, clip)[0]), MC.world_all(BC.vanilla(SET, name)[0])
    J = [IX[n] for n in ('LeftHand', 'RightHand', 'Head', 'LeftArm', 'RightArm')]

    def rel(W, f):
        return W[f, J, :3, 3] - W[f, IX['Spine3'], :3, 3]
    d = [float(np.linalg.norm(rel(a, f) - rel(b, 0), axis=1).max()) for f in range(len(a))]
    f = int(np.argmin(d))
    return f, round(d[f] * 100, 2)


def auto_start(name):
    best = None
    for b in ('combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04'):
        f, dcm = branch_frame(name, b)
        if best is None or dcm < best[2]:
            best = (b, f, dcm)
    return best


def start_state(G, start, name):
    if start in ('block', 'safepose'):
        clip, f, dcm = {'block': 'block', 'safepose': 'melee_safepose'}[start], 0, None
    elif start == 'auto':
        clip, f, dcm = auto_start(name)
    else:
        clip = start[0]
        f, dcm = branch_frame(name, clip)
    b = built_clip(clip)
    W = b['W']
    K = 0
    for stp in (b['info'].get('steps') or {}).values():
        for x in stp:
            if x['frames'][0] < f < x['frames'][1]:
                K = max(K, x['frames'][1] - f)
    path = {s: W[f:f + K + 1, IX[s + 'Foot'], :3, 3].copy() for s in ('Left', 'Right')}
    return dict(clip=clip, frame=f, match_cm=dcm, lt=b['lt'][f].copy(), lr=b['lr'][f].copy(), hips=W[f, IX['Hips'], :3, 3].copy(), feet_path=path,
                W=W[f].copy(), K=K, rel=b['rel'][f:f + 1].copy(), two=float(b['w'][f]))


def hit_clip(G, name, spec):
    """<attack>_hit: the built attack from the vanilla branch frame to its last frame, re-timed to the vanilla length (hit-stop: the first steps
    slower); the capsule path of the attack continues (a lunge attack's planted feet stay planted); the strongest hit-stop of HIT_SLOWS that
    passes every gate (local variant of npc_build.hit_clip)."""
    res = []
    for slow in HIT_SLOWS:
        out = hit_once(G, name, spec, slow)
        g = gates_b(NB.fk(out[0], out[1]), out[2], False, out[5], 'hit', out[4])
        out[3]['hit_stop_tries'] = [dict(hit_stop=r[2], PASS=r[0]) for r in res] + [dict(hit_stop=slow, PASS=g['PASS'])]
        res.append((g['PASS'], g['base_gate_pass'], slow, out))
        if g['PASS']:
            break
    return max(res, key=lambda r: r[:2])[3]


def hit_once(G, name, spec, slow):
    d, ph = BC.vanilla(SET, name)
    T = len(d['W'])
    f, dcm = branch_frame(name, spec['of'])
    b = built_clip(spec['of'])
    rem = len(b['lt']) - 1 - f
    t = np.arange(T - 1, dtype=float)
    v = slow + (1.0 - slow) * NB.sstep((t - 1.0) / HIT_SLOW_FR)
    s = np.concatenate([[0.0], np.cumsum(v)])
    s *= rem / s[-1]
    fr = f + s
    lt, lr = NB.resample(b['lt'], b['lr'], fr)
    relM = rel_blend(b['rel'][np.floor(fr).astype(int)], b['rel'][np.minimum(np.floor(fr).astype(int) + 1, len(b['rel']) - 1)], fr - np.floor(fr))
    w2h = NB.interp_rows(b['w'], fr)
    log = {}
    post_fix(lt, lr, relM, w2h, lock_w(T), log)
    cy = np.asarray(b['info'].get('capsule_path_cm') or [0.0] * len(b['lt']), float) / 100.0
    c = np.zeros((T, 3))
    c[:, 1] = NB.interp_rows(cy[:, None], fr)[:, 0] - NB.interp_rows(cy[:, None], fr[:1])[0, 0]
    info = dict(kind='hit', of=spec['of'], start=dict(clip=spec['of'], frame=f, match_vanilla_cm=dcm), built_frames=np.round(fr, 2).tolist(),
                hit_stop=slow, source='%s frames %d..%d re-timed to %d frames (%.2fx overall, first steps %.0f %% of the later speed: hit-stop)' % (
                    spec['of'], f, len(b['lt']) - 1, T, rem / (T - 1), 100 * slow),
                capsule='rest of the attack capsule path (%.1f cm)' % (c[-1, 1] * 100), vanilla_phases=ph, fixes=log)
    return lt, NB.qcont(lr), c, info, relM, w2h


REACT_SPAN = 1.4
HIT_SLOWS, HIT_SLOW_FR = (0.5, 0.7, 1.0), 4.0     # hit clips (values of npc_build.HIT_SLOWS 2026-11-09; local copy)


def react_clip(G, name, spec):
    d, ph = BC.vanilla(SET, name)
    T = len(d['W'])
    st = start_state(G, spec['start'], name)
    donors = resolve_donors(name, spec)
    sq = BC.BSeq(donors)
    i = np.arange(T, dtype=float)
    tau = np.linspace(0, min(sq.n - 1, REACT_SPAN * (T - 1)), T)
    w_s = 1.0 - NB.sstep(i / NB.REACT_IN)
    e0, e1 = int(round(T * 0.45)), T - 4
    w_e = NB.sstep((i - e0) / max(e1 - e0, 1))
    lt, lr = NB.build_body(G, sq, tau, w_s, w_e, start_lr=st['lr'])
    c = np.zeros((T, 3))
    tracks, steps = NB.cont_tracks(T, st, G.feet, f_settle=max(e0, st['K'] + 2))
    hips = NB.hips_path(G, sq, tau, tracks, c, w_e)
    hips = hips + (st['hips'] - hips[0])[None] * (1.0 - NB.sstep(i / 8.0))[:, None]
    log = {}
    lt, lr = NB.finish(G, lt, lr, hips, tracks, c, log)
    w0 = 1.0 - NB.sstep(i / NB.START_RAMP)
    lr[:, NB.UPPER] = NB.offset_start(lr[:, NB.UPPER], st['lr'][NB.UPPER], w0)
    lr = NB.qcont(lr)
    finger_end(lr, G, w_e)
    for sd in ('Left', 'Right'):
        log['ramp_wrist_twist_moved_deg_' + sd[0]] = round(NB.wrist_to_forearm(lt, lr, sd), 1)
    relD, wD = sq.rel_at(tau)
    wD = smooth_w(wD)
    relM = rel_blend(rel_blend(relD, np.repeat(st['rel'], T, 0), w_s), G.relT(T), w_e)
    w2h = ((1 - w_s) * wD + w_s * st['two']) * (1 - w_e) + w_e * G.two
    post_fix(lt, lr, relM, w2h, lock_w(T), log)
    end_match(lt, lr, G, relM, w2h, log)
    info = dict(kind='react', note=spec.get('note'), donors=sq.parts, start=dict(clip=st['clip'], frame=st['frame'], match_vanilla_cm=st['match_cm']),
                tau=np.round(tau, 2).tolist(), guard_blend=[e0, e1], capsule='static', feet_mode='continue + settle', steps=steps, fixes=log,
                vanilla_phases=ph)
    return lt, lr, c, info, relM, w2h


def start_continuity(name, spec, W):
    s = spec.get('start') or ((spec['of'], None) if spec['kind'] == 'hit' else None)
    if not s:
        return None
    if s in ('block', 'safepose'):
        Ws = built_clip({'block': 'block', 'safepose': 'melee_safepose'}[s])['W'][0]
    elif s == 'auto':
        b, f, _ = auto_start(name)
        Ws = built_clip(b)['W'][f]
    else:
        Ws = built_clip(s[0])['W'][branch_frame(name, s[0])[0]]
    return dict(pos_cm=round(float(np.linalg.norm(W[0, NB.BODY_IX, :3, 3] - Ws[NB.BODY_IX, :3, 3], axis=1).max() * 100), 3),
                rot_deg=round(float(MC.rot_delta_deg(W[0, NB.BODY_IX, :3, :3], Ws[NB.BODY_IX, :3, :3]).max()), 3))


# ------------------------------------------------------------------ gates
HAND_CROSS_2H = ('hand_cross_cm_L', 'hand_cross_cm_R')     # two-hand grip: the hilt sets where both hands are (vanilla katana: up to 40 cm)


def gates_b(W, c, loop, w2h, kind, relM):
    g = NB.gates(W, c, loop=loop)
    T = len(W)
    if TWO and (w2h > 0.5).any():       # hand_cross_* is not counted on two-hand frames (arm-torso penetration / clearance gates still are)
        fg = NB.GB.frame_gate(W[:, :73], NB.RG, steps=True, al=NB.AL)
        okf = np.array(fg['ok'], bool).copy()
        fails = {}
        for t in range(T):
            bad = [k for k, v in fg['viol'].items() if float(np.asarray(v).reshape(-1)[t]) > 1e-9]
            if not okf[t] and w2h[t] > 0.5 and bad and all(k in HAND_CROSS_2H for k in bad):
                okf[t] = True
            if not okf[t]:
                for k in (bad or [str(fg['worst'][t])]):
                    fails[k] = fails.get(k, 0) + 1
        if int(okf.sum()) != g['base_gate_pass']:
            g['base_gate_pass_raw'], g['base_gate_fail_raw'] = g['base_gate_pass'], g['base_gate_fail']
            g['base_gate_pass'], g['base_gate_fail'] = int(okf.sum()), fails
            g['hand_cross_2h_exempt_frames'] = int(okf.sum()) - g['base_gate_pass_raw']
    fc = NB.fist_centers(W)
    ffv = (np.linalg.norm(fc['Left'] - fc['Right'], axis=1) - 2 * NB.FIST_R) * 100
    one = w2h < 0.5
    ff1 = round(float(ffv[one].min()), 1) if one.any() else None
    cnt, clr = BC.blade_pen(W, WEAPON, w2h)
    g['blade_pen_frames'] = int((cnt > 0).sum())
    g['blade_pen_max_vertices'] = int(cnt.max())
    g['blade_clear_min_cm'] = round(float(clr.min()), 1)
    g['blade_clear_frame'] = int(np.argmin(clr))
    g['hand_hand_one_hand_min_cm'] = ff1
    g['two_hand_frames'] = int((~one).sum())
    g['hilt_err_max_cm'] = hilt_err(W, relM, w2h)
    tv = BC.tip_speed(W, WEAPON)
    g['tip_max_m_s'] = round(float(tv.max()), 1)
    ok = (g['base_gate_pass'] == T and g['pen_frames'] == 0 and g['head_clear_min_cm'] >= 0 and (ff1 is None or ff1 >= 0)
          and max(g['slide_max_cm'].values()) <= NB.SLIDE_MAX_CM and g['sole_min_cm'] >= NB.FLOOR_CM and g['spike_max_cm'] <= NB.JUMP_CM
          and g['root_identity'] and g['blade_pen_frames'] == 0 and g['hilt_err_max_cm'] <= 1.5)
    if loop:
        ok = ok and g['loop_gap_cm'] <= NB.LOOP_CM and g['loop_gap_deg'] <= NB.LOOP_DEG
    if kind in ('attack', 'heavy') and T > 2:
        hs = start_hand_step(W)
        g['start_hand_step_cm'] = {k: v[:2] for k, v in hs.items()}
        ok = ok and all(v[2] for v in hs.values())
    g['PASS'] = bool(ok)
    return g


# ------------------------------------------------------------------ main
def main(a):
    global CDIR
    t0 = time.time()
    out_dir = opt(a, '--out', OUT)
    cdir = os.path.join(out_dir, 'clips')
    os.makedirs(cdir, exist_ok=True)
    CDIR = cdir
    only = opt(a, '--only').split(',') if '--only' in a else None
    lunge_start = float(opt(a, '--lunge-start', 0.0))
    G = BGuard()
    print('GUARD', WEAPON, RIG, 'fixes', G.log, 'two-hand', G.two, flush=True)
    for h in ('heavy_attack_01', 'heavy_attack_02'):
        HOLDS[h] = dict(lr=hold_lr(G, 1.0), lr2=hold_lr(G, 1.0 + NB.HOLD_PULSE), side='Right')
    rep_path = os.path.join(out_dir, 'build_report_%s.json' % SET)
    rep = json.load(open(rep_path)) if os.path.exists(rep_path) and only else {}
    for name in ORDER:
        if only and name not in only:
            continue
        spec = SPEC[name]
        t1 = time.time()
        try:
            if spec['kind'] in ('hold_start', 'hold_loop'):
                built_clip(spec['of'])          # the hold pose belongs to its heavy attack: rejected heavy attack -> hold clips stay vanilla
            if spec['kind'] in ('attack', 'heavy'):
                lt, lr, c, info, relM, w2h = attack_clip(G, name, spec, lunge_start)
            elif spec['kind'] == 'hit':
                lt, lr, c, info, relM, w2h = hit_clip(G, name, spec)
            elif spec['kind'] == 'react':
                lt, lr, c, info, relM, w2h = react_clip(G, name, spec)
            else:
                lt, lr, c, info, relM, w2h = other_clip(G, name, spec, lunge_start)
        except FileNotFoundError as e:
            print('%-24s SKIP (missing input: %s)' % (name, e), flush=True)
            stem = '%s__%s.%s' % (SET, name, SUFFIX)
            for ext in ('.npz', '.twohand.npz', '.info.json'):      # stale own output of an earlier run: the clip stays vanilla now
                if os.path.exists(os.path.join(cdir, stem + ext)):
                    os.remove(os.path.join(cdir, stem + ext))
            rep[name] = dict(PASS=False, skipped='input clip rejected -> vanilla: %s' % os.path.basename(str(e)))
            continue
        lr = NB.qcont(lr)
        meta = dict(set=SET, clip=name, rig=RIG, animation_type='Normal', additive=False, root_motion_type='None', frames=len(lt),
                    duration=(len(lt) - 1) / 30.0, events=[], source='npc_blade_build.py ' + SUFFIX, weapon=WEAPON)
        d = BC.NR.make_dump(P, lt, lr, meta)
        W = NB.fk(lt, lr)
        loop = name in LOOPS
        g = gates_b(W, c, loop, w2h, spec['kind'], relM)
        if name != 'guard_idle_loop':
            dv, _ = BC.vanilla(SET, name)
            assert len(dv['W']) == len(lt), (name, len(dv['W']), len(lt))
        if spec['kind'] in ('attack', 'heavy'):
            info['strike_built'] = int(BC.strike_frame(W, WEAPON, 0, len(W) - 1)[0])
            info['guard_dist_f0_fN_cm'] = [round(float(np.linalg.norm(W[k, NB.BODY_IX, :3, 3] - G.W[NB.BODY_IX, :3, 3], axis=1).max() * 100), 1)
                                           for k in (0, len(W) - 1)]
        info['grip'] = 'NPC grip: WeaponRight local rotation + translation and fingers of the NPC donor frame'
        info['capsule_path_cm'] = np.round(c[:, 1] * 100, 2).tolist()
        sc = start_continuity(name, spec, W)
        if sc:
            info['start_continuity'] = sc
        info['gates'] = {k: v for k, v in g.items() if not k.startswith('_')}
        info['failing_frames'] = [dict(frame=int(t), worst=g['_worst'][t]) for t in np.where(~g['_ok_frames'])[0]][:40]
        _BUILT.pop(name, None)
        rdir = os.path.join(out_dir, 'clips_rejected')       # clips failing a gate: kept for inspection, never used as input (stay vanilla)
        tgt_dir, other_dir = (cdir, rdir) if g['PASS'] else (rdir, cdir)
        os.makedirs(tgt_dir, exist_ok=True)
        stem = '%s__%s.%s' % (SET, name, SUFFIX)
        for ext in ('.npz', '.twohand.npz', '.info.json'):
            if os.path.exists(os.path.join(other_dir, stem + ext)):
                os.remove(os.path.join(other_dir, stem + ext))       # stale result of an earlier run of this clip (own build output)
        base = os.path.join(tgt_dir, stem)
        BC.NR.save_dump(base + '.npz', d)
        np.savez_compressed(base + '.twohand.npz', rel=relM.astype(np.float64), w=np.asarray(w2h, float))
        MC.jdump(info, base + '.info.json')
        rep[name] = dict(PASS=g['PASS'], **{k: v for k, v in info['gates'].items() if k != 'PASS'}, donors=[p['clip'] for p in info.get('donors', [])],
                         impact=[info.get('impact_vanilla'), info.get('strike_built')], speed=info.get('speed_chosen'), seconds=round(time.time() - t1, 1))
        print('%-24s %-4s base %3d/%3d fail=%s pen=%d blade=%d(%.1fcm) head=%.1f ff1=%s slide=%s sole=%.1f spike=%.1f hilt=%.1f %s %s %.1fs' % (
            name, 'PASS' if g['PASS'] else 'FAIL', g['base_gate_pass'], g['frames'], g['base_gate_fail'], g['pen_frames'], g['blade_pen_frames'],
            g['blade_clear_min_cm'], g['head_clear_min_cm'], g['hand_hand_one_hand_min_cm'], g['slide_max_cm'], g['sole_min_cm'], g['spike_max_cm'],
            g['hilt_err_max_cm'], g.get('start_hand_step_cm', ''), ('loop %.2fcm/%.2fdeg' % (g['loop_gap_cm'], g['loop_gap_deg'])) if 'loop_gap_cm' in g else '',
            time.time() - t1), flush=True)
    MC.jdump(rep, rep_path)
    print('BUILD done %.1f s -> %s' % (time.time() - t0, cdir))


if __name__ == '__main__':
    main(sys.argv[1:])
