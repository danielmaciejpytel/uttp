"""Third-person fists clips of V built from the NPC melee animations (wa_/ma_gang_unarmed_action_combat), 2026-11-09.

  python -I npc_build.py [--rig woman|man] [--only clip,clip] [--lunge-start 0] [--out DIR]
Output: <OUT>\\clips\\<set>__<clip>.npc1.npz (dump layout of melee_dump.py: W [T,73,4,4] game axes, Root = identity, + lt / lr / W_extra ...)
        <OUT>\\clips\\<set>__<clip>.npc1.info.json (donors, mirror, time map, impact frames, root travel, feet, capsule model, gates)
        <OUT>\\clips\\<set>__guard_idle_loop.npc1.npz (extra guard idle loop, not a vanilla clip name) and <OUT>\\build_report.json.
Method (per clip, game axes, 30 fps, frame count = the vanilla clip's):
  1 donor(s) retargeted by npc_retarget.Donor (NPC locals by name on the player rig), concatenated (attack part + its recover clip);
  2 time map tau(i): attacks align the donor impact (striking fist furthest forward along +Y relative to Spine3, root frame, near the loop
    phase) to the vanilla impact (vanilla striking hand furthest forward in the active window); before the impact the donor's late wind-up
    is played at <= SPEEDUP x, after it the follow-through at 1x;
  3 one common guard pose G (idle_combat frame 0, the start pose of the 'from idle' donors) at frame 0 (offset blend decaying to the
    impact) and back to G after the follow-through (smooth blend, finished by GUARD_BY frames after the impact = early in the recover
    window, where the game chains the next attack); heavy attacks start at the hold pose (vanilla heavy frame 0 == hold_loop pose);
  4 feet are SYNTHESISED in world under the simulated capsule path (planted = exact pins, steps = lift arcs): planted guard stance for
    combo / heavy / block / hold / idle, a lunge step (lead foot with the capsule, rear foot following) for the finishers, settle steps
    for any capsule residual; hips = feet midpoint + the guard offset + a scaled donor weight shift / height change; legs by hinge IK
    (hinge_ik.solve, knee pole of the guard legs), feet keep the guard orientation;
  5 anatomy fixes towards anatomy_gate_base (V's base-body gate) where the NPC pose is outside it: spine bend / twist spread over
    Spine..Spine3 (chest world rotation kept), hips follow the chest yaw (the gate's shoulder width is measured along the hips axis),
    head turn / clavicle rotation scaled down per frame (mildest scale that passes), wrist twist moved into the forearm (fist world
    orientation kept), wrist kept >= EYE_MIN from the eyes (arm hinge IK);
  6 gates on every frame: anatomy_gate_base.frame_gate (unchanged limits), redo_pen proxy (arm vs torso), fist vs head / fist vs fist
    clearance, feet (planted-foot slide under the capsule path <= 2 cm, sole not below -1 cm), single-frame jumps, loop seams;
  7 crossed hands (final2 2026-11-09): SPEC 'uncross' clips get the guard hand moved aside where the hands cross (uncross(), before the
    gates); every clip records info['hands'] (lateral gaps of the hands / fists, crossed frames, arm-to-arm clearance), 'CROSSED' is printed.
The groundslam clips of the packs are written afterwards by npc_groundslam.py on the same --out folder.
Capsule model (pilot test 2026-11-09, no enemy): every combo / heavy attack 2.5 cm forward over frames 0-2; finishers 0.70 m in 0.18 s
starting at --lunge-start (default frame 0: the pilot gives speed and duration, not the phase); others 0.
"""
import json
import os
import sys
import time
import types

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate_base as GB  # noqa: E402  (base_body resolves --rig)
import base_body as BB  # noqa: E402
import contact as CT  # noqa: E402
import hinge_ik as HI  # noqa: E402
import melee_common as MC  # noqa: E402
import npc_retarget as NR  # noqa: E402

RIG = BB.RIG_NAME
SET = NR.PLAYER_SET[RIG]
OUT = NR.OUT
SUFFIX = 'npc1'
SPEEDUP = 1.5
GUARD_BY = 16            # frames after the impact by which the pose is back at the guard
EYE_MIN = 0.275          # wrist - eyes (gate 25 cm + margin)
SPINE_CUM = (0.2, 0.45, 0.75, 1.0)
SHIFT_K, SHIFT_MAX, DZ_K = 0.25, 0.12, 0.6
STEP_EPS = 0.004         # residual (m) below which a planted foot simply stays
SLIDE_MAX_CM, FLOOR_CM, JUMP_CM, LOOP_CM, LOOP_DEG = 2.0, -1.0, 3.0, 0.5, 2.0
HEAD_R, FIST_R = 0.105, 0.045
if RIG == 'man':
    HEAD_R = 0.114           # male V head: Head - eyes 10.0 cm vs 9.2 cm (female) -> 0.105 x 1.086; hands, arms, shoulders equal on both rigs

DESPIKE_CM = 15.0        # Seq(despike=True): donor frames sticking out of their neighbours by more than this (sagitta over half chord)
L_DON = 'melee_combo_infinite_'
SPEC = {
    # attacks: donors [(npc clip, mirror)], striking side, finisher lunge
    'combo_attack_01': dict(kind='attack', donors=[('melee_combo_close_01_p1', False)], side='Right', note='rear-hand cross', align=True,
                            fix_release=2, fix_release_end=3, speeds=(1.5, 1.35, 1.2, 1.1, 1.05, 1.0, 0.95, 0.9)),   # polish: full extension on the vanilla impact f6
    'combo_attack_02': dict(kind='attack', donors=[(L_DON + 'attack_02', False)], side='Left', note='lead-hand jab'),
    'combo_attack_03': dict(kind='attack', donors=[(L_DON + 'attack_05', False)], side='Right', note='rear-hand hook'),
    'combo_attack_04': dict(kind='attack', donors=[(L_DON + 'attack_03', False)], side='Left', note='lead-hand uppercut'),
    'combo_attack_05': dict(kind='attack', donors=[(L_DON + 'attack_04', False)], side='Right', note='rear-hand uppercut'),
    'final_attack_03': dict(kind='attack', donors=[('melee_combo_close_02_p3', False), ('melee_combo_close_02_p3_recover', False)], side='Right',
                            lunge=True, note='power cross, lunge'),
    'final_attack_04': dict(kind='attack', donors=[(L_DON + 'startup_03', False)], side='Left', lunge=True, note='crouching lead uppercut, lunge'),
    'final_attack_05': dict(kind='attack', donors=[(L_DON + 'attack_07', False)], side='Right', lunge=True, note='rising rear uppercut, lunge'),
    # stage 2 step 6 (stronger heavy_01, same donor / style): faster pre-impact tempo allowed (up to 2.0x instead of 1.5x).  Tried and
    # rejected: NPC heavy_04 (162 deg chest spin, wrist ~20 m/s), NPC heavy_01 (melee_heavy_01_hit_to_idle frame 1: the right hand 39 cm up
    # and back in one frame; despiked it fails the spike gate and is weaker: wrist 8.5 m/s, reach 47.3 cm)
    # polish 2026-11-09 (stronger heavy_01): the NPC cross melee_combo_close_01_p1 (the strongest clean straight of the unarmed set) from the
    # charged hold pose, fix tails released before the impact, re-timed so the full extension lands on the vanilla impact (frame 0 pinned to the
    # hold pose), doubled weight shift.  Rejected (report): NPC heavy_02 from sprint (overhand, fist above the head), charge_01 / charge_02 (the
    # off arm flung wide), heavy_04 / combo_03_p3 (wrist 16-22 m/s), heavy_01 (+miss: upper-arm twist gate), block_break / parry_break (weak).
    'heavy_attack_01': dict(kind='heavy', donors=[('melee_combo_close_01_p1', False)], side='Right',
                            speeds=(2.0, 1.8, 1.65, 1.5, 1.35, 1.2, 1.1, 1.05, 1.0, 0.95, 0.9), hips_seam_tries=(False,), align=True,
                            fix_release=2, fix_release_end=3, shift_k=0.5, shift_max=0.18,
                            note='charged rear cross from the hold pose'),
    'heavy_attack_02': dict(kind='heavy', donors=[(L_DON + 'attack_06', False)], side='Left', note='charged lead straight from the hold pose'),
    'hold_start_01': dict(kind='hold_start', of='heavy_attack_01'),
    'hold_loop_01': dict(kind='hold_loop', of='heavy_attack_01'),
    'hold_loop_02': dict(kind='hold_loop', of='heavy_attack_02'),
    'block_start': dict(kind='block_start', donors=[('idle_block', False)]),
    'block': dict(kind='block', donors=[('idle_block', False)]),
    'block_end': dict(kind='block_end', donors=[('idle_block', False)]),
    'melee_equip': dict(kind='equip', donors=[('ovr_equip', False)]),
    'melee_unequip': dict(kind='unequip', donors=[('ovr_unequip', False)]),
    'melee_idlepose': dict(kind='pose'),
    'melee_safepose': dict(kind='safepose'),
    'guard_idle_loop': dict(kind='idle_loop', donors=[('idle_combat', False)], frames=120),
}
ORDER = ['guard_idle_loop', 'melee_idlepose', 'melee_safepose', 'combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04',
         'combo_attack_05', 'final_attack_03', 'final_attack_04', 'final_attack_05', 'heavy_attack_01', 'heavy_attack_02', 'hold_start_01',
         'hold_loop_01', 'hold_loop_02', 'block_start', 'block', 'block_end', 'melee_equip', 'melee_unequip']

# ---- stage 2 (2026-11-09): the rest of the fists set.  Vanilla continuity (measured on the vanilla clips): <attack>_hit / _deflect frame 0
# == the vanilla attack at frame ~6-8 (the branch frame, found per clip), their last frame == melee_idlepose; parried_left / _right frame 0 ==
# combo_attack_02 f6 / combo_attack_05 f7; block_attack / block_deflect_* frame 0 == block; hold_start_offscreen_* end on the hold poses;
# safe_attack_hit is a whole attack from the guard (vanilla: safe_attack 2 frames later); jump_attack == safe_attack (vanilla data equal).
# NPC donors: the attacker-side deflect reactions (deflect_<arm>arm_to_<dir>: the striking arm extended on frame 0, knocked away, back to
# the stance), the NPC block hit reactions (block_left / block_right).  Kinds: 'hit' = the built attack from the branch frame on, re-timed to
# the vanilla length (hit-stop at 0.5x over the first frames); 'react' = start pose (built clip frame) -> NPC reaction -> guard.
DEFLECT_DONOR = {'Right': ('deflect_rightarm_to_down', False), 'Left': ('deflect_leftarm_to_right', False)}
CROSS = [('melee_combo_close_01_p1', False)]
SPEC.update({
    'safe_attack': dict(kind='attack', donors=CROSS, side='Right', note='quick rear cross'),
    'jump_attack': dict(kind='attack', donors=CROSS, side='Right', note='quick rear cross (vanilla data == safe_attack; airborne legs: DLL mask)'),
    'safe_attack_hit': dict(kind='attack', donors=CROSS, side='Right', note='vanilla: the whole safe attack again, from the guard'),
    'sprint_attack': dict(kind='attack', donors=SPEC['final_attack_03']['donors'], side='Right', lunge=True,
                          note='power cross with lunge (vanilla frame 0 == final_attack_03 frame 0)'),
    'crouch_attack': dict(kind='attack', donors=[(L_DON + 'attack_04', False)], side='Right', note='rear uppercut (standing guard; crouch: DLL mask)'),
    'block_attack': dict(kind='attack', donors=CROSS, side='Right', start='block', note='rear cross out of the block'),
    'block_deflect_left': dict(kind='react', start='block', donors=[('block_left', False)], layer=True,
                               note='NPC block hit reaction laid on the block pose, back to the guard'),
    'block_deflect_right': dict(kind='react', start='block', donors=[('block_left', True)], layer=True,
                               note='NPC block hit reaction laid on the block pose, back to the guard'),
    'parried_left': dict(kind='react', start=('combo_attack_02', None), donors=[('deflect_leftarm_to_left', False)], note='lead arm knocked away'),
    'parried_right': dict(kind='react', start=('combo_attack_05', None), donors=[('deflect_rightarm_to_right', False)],
                          note='rear arm knocked away'),
    'hold_start_offscreen_01': dict(kind='hold_start', of='heavy_attack_01', start='safepose'),
    'hold_start_offscreen_02': dict(kind='hold_start', of='heavy_attack_02', start='safepose'),
})
FOLLOW_BASES = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04', 'combo_attack_05', 'final_attack_03', 'final_attack_04',
                'final_attack_05', 'heavy_attack_01', 'heavy_attack_02', 'safe_attack', 'sprint_attack', 'jump_attack', 'crouch_attack']
for _b in FOLLOW_BASES:
    SPEC.setdefault(_b + '_hit', dict(kind='hit', of=_b))
    _sd = SPEC[_b]['side']
    _ot = DEFLECT_DONOR['Left' if _sd == 'Right' else 'Right']
    SPEC[_b + '_deflect'] = dict(kind='react', start=(_b, None), donors=[DEFLECT_DONOR[_sd]], alts=[[DEFLECT_DONOR[_sd]], [(_ot[0], True)]],
                                 note='striking arm deflected (the same-arm NPC deflect, or the other arm mirrored: the stronger that passes)')
if RIG == 'man':        # Male V: deflect_rightarm_to_right fails the male gates at every amp (spike 13.8..7.6 cm at amp 1.0..0.7, shoulder
    SPEC['parried_right']['alts'] = [[('deflect_rightarm_to_right', False)], [('deflect_leftarm_to_left', True)]]   # width below 0.6)
# final2 2026-11-09 (hard anatomy rule: no crossed hands).  The deflect donor deflect_leftarm_to_right (direct or mirrored) knocks the struck
# arm across the body past the guard hand (left hand right of the right hand: Female crouch_attack / combo_attack_05 / combo_attack_03 /
# heavy_attack_02 / combo_attack_04 _deflect -10.6..-4.8 cm, Male final_attack_04 / combo_attack_04 / sprint_attack / heavy_attack_02 _deflect
# -13.2..-4.6 cm) and final_attack_04's rear hand drops onto the rising lead uppercut (-1.9 / -0.6 cm, forearms interpenetrating).  'uncross'
# = the guard hand (the arm that is not striking / struck) moved aside after the build, see uncross(); same donor, amp and timing; a no-op
# (bit-identical clip) on the body where the clip does not cross.
UNCROSS_CLIPS = ['crouch_attack_deflect', 'combo_attack_05_deflect', 'combo_attack_03_deflect', 'heavy_attack_02_deflect', 'combo_attack_04_deflect',
                 'final_attack_04_deflect', 'sprint_attack_deflect', 'final_attack_04']
for _c in UNCROSS_CLIPS:
    _b = _c[:-len('_deflect')] if _c.endswith('_deflect') else _c
    SPEC[_c]['uncross'] = 'Left' if SPEC[_b]['side'] == 'Right' else 'Right'
ORDER2 = (['safe_attack', 'jump_attack', 'safe_attack_hit', 'sprint_attack', 'crouch_attack', 'block_attack', 'block_deflect_left', 'block_deflect_right',
           'parried_left', 'parried_right', 'hold_start_offscreen_01', 'hold_start_offscreen_02']
          + [b + v for b in FOLLOW_BASES for v in ('_hit', '_deflect') if b + v not in ('safe_attack_hit',)])
ORDER = ORDER + ORDER2
# polish 2026-11-09: the 15 vanilla first-person clips the npc1 packs leave in (sprint 3, crouch 4, slide 3, groundslam 4, air sprint 1).  In TPP
# the game shows them only through the melee upper-body mask (attack / pose states and the 0.3 s release hold, e.g. crouch / sprint after
# crouch_attack / sprint_attack); there every vanilla frame fails V's base gate (FPP spine bend, shoulders, wrists; arms up to 60 cm wide).
# Third-person version: the guard upper body held for the vanilla length (legs: motion matching under the mask).
FPP15 = ['melee_sprint_start', 'melee_sprint_loop', 'melee_sprint_stop', 'melee_crouchpose', 'melee_idle_to_crouch', 'melee_crouch_to_idle',
         'melee_crouch_to_sprint', 'melee_slide_start', 'melee_slide_loop', 'melee_slide_end', 'melee_groundslam_attack',
         'melee_groundslam_attack_air_start', 'melee_groundslam_attack_air_idle', 'melee_groundslam_attack_air_exit', 'melee_air_sprint']
for _c in FPP15:
    SPEC[_c] = dict(kind='guardhold', note='guard upper body held (vanilla length); shown in TPP only under the melee upper-body mask')
ORDER = ORDER + FPP15
CDIR = None              # clips folder of this build (start poses of the stage-2 clips are read from the built clips)

P = NR.Player(RIG)
IX = P.ix
PAR = P.par
REST_LR = P.ref['rest_lr']
REST_W = MC.fk_world(PAR, P.ref['rest_lt'][None], P.ref['rest_lr'][None])[0]
RG = BB.rig_for_gate()
AL = GB.allowed()
BODY73 = [n for n in P.names[:73] if n not in ('Root', 'Trajectory', 'reference_joint')]
BODY_IX = [IX[n] for n in BODY73]
LEGS = [IX[n] for n in MC.L_LEG + MC.R_LEG]
UPPER = [j for j in range(len(P.names)) if j not in set(LEGS) | {IX['Root'], IX['Hips']}]
START_RAMP = 3.0


# ------------------------------------------------------------------ small helpers
def fk(lt, lr):
    return MC.fk_world(PAR, lt, lr)


def sstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def qslerp(a, b, w):
    """a, b [...,4] xyzw, w broadcastable [...] -> slerp."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    d = np.sum(a * b, -1, keepdims=True)
    b = np.where(d < 0, -b, b)
    d = np.abs(d)
    w = np.asarray(w, float)[..., None]
    th = np.arccos(np.clip(d, -1, 1))
    s = np.sin(th)
    small = s < 1e-6
    wa = np.where(small, 1 - w, np.sin((1 - w) * th) / np.where(small, 1, s))
    wb = np.where(small, w, np.sin(w * th) / np.where(small, 1, s))
    q = wa * a + wb * b
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def qmul(a, b):
    return (R.from_quat(a.reshape(-1, 4)) * R.from_quat(b.reshape(-1, 4))).as_quat().reshape(np.broadcast_shapes(a.shape, b.shape))


def qinv(a):
    q = np.array(a, float)
    q[..., :3] *= -1
    return q


def resample(lt, lr, fr):
    fr = np.clip(np.asarray(fr, float), 0, len(lt) - 1)
    i0 = np.floor(fr).astype(int)
    i1 = np.minimum(i0 + 1, len(lt) - 1)
    u = fr - i0
    lt2 = lt[i0] * (1 - u[:, None, None]) + lt[i1] * u[:, None, None]
    return lt2, qslerp(lr[i0], lr[i1], np.repeat(u[:, None], lr.shape[1], 1))


def interp_rows(x, fr):
    fr = np.clip(np.asarray(fr, float), 0, len(x) - 1)
    i0 = np.floor(fr).astype(int)
    i1 = np.minimum(i0 + 1, len(x) - 1)
    u = (fr - i0).reshape((-1,) + (1,) * (x.ndim - 1))
    return x[i0] * (1 - u) + x[i1] * u


def set_world_rot(lt, lr, j, Rw):
    """local rotation of joint j from desired world rotations Rw [T,3,3] (parents as they are now)."""
    W = fk(lt, lr)
    p = PAR[j]
    Rp = W[:, p, :3, :3] if p >= 0 else np.repeat(np.eye(3)[None], len(lt), 0)
    lr[:, j] = R.from_matrix(np.swapaxes(Rp, -1, -2) @ Rw).as_quat()


def delta(W, j):
    return W[:, j, :3, :3] @ REST_W[j, :3, :3].T


# ------------------------------------------------------------------ donors
class Seq:
    """concatenated donor clips (game-axes locals, NPC root path continued across the parts)."""

    def __init__(self, donors, despike=False, hips_seam=False):
        lts, lrs, rts, rRs, self.parts = [], [], [], [], []
        off_t, off_R = np.zeros(3), np.eye(3)
        n0 = 0
        self.hips_seam_cm = []
        for k, dspec in enumerate(donors):
            clip, mirror = dspec[0], dspec[1]
            set_name = dspec[2] if len(dspec) > 2 else None        # polish: optional NPC set of the donor (default: the gang unarmed set)
            dn = NR.Donor(P, clip, set_name=set_name, mirror=mirror)
            rR = np.einsum('ij,tjk->tik', dn.root_R[0].T, dn.root_R)
            rt = np.einsum('ij,tj->ti', dn.root_R[0].T, dn.root_t - dn.root_t[0])
            s = 0 if k == 0 else 1                      # the follow-up clip starts on the previous clip's last frame
            h0 = fk(dn.lt[:1], dn.lr[:1])[0, IX['Hips'], :3, 3]
            if k and hips_seam:     # split NPC clips (melee_heavy_02_idle_to_attack -> _hit_to_idle) do not continue: the hips jump 30 cm
                d = off_R @ (h_prev - h0)          # forward / 6 cm up at the seam -> place the follow-up part so the donor hips continue
                off_t = off_t + d
                self.hips_seam_cm.append(np.round(d * 100, 1).tolist())
            lts.append(dn.lt[s:])
            lrs.append(dn.lr[s:])
            rts.append((np.einsum('ij,tj->ti', off_R, rt) + off_t)[s:])
            rRs.append(np.einsum('ij,tjk->tik', off_R, rR)[s:])
            self.parts.append(dict(clip=clip, mirror=mirror, start=n0, frames=dn.n, phases=dn.phases, root_end_cm=np.round(rt[-1] * 100, 1).tolist(),
                                   root_yaw_end_deg=round(float(np.degrees(np.arctan2(rR[-1, 1, 0], rR[-1, 0, 0]))), 1)))
            off_t, off_R = rts[-1][-1].copy(), rRs[-1][-1].copy()
            h_prev = fk(dn.lt[-1:], dn.lr[-1:])[0, IX['Hips'], :3, 3]
            n0 += dn.n - s
        self.lt = np.concatenate(lts)
        self.lr = qcont(np.concatenate(lrs))
        self.root_t, self.root_R = np.concatenate(rts), np.concatenate(rRs)
        self.n = len(self.lt)
        self.W = fk(self.lt, self.lr)
        self.despiked = []
        while despike:       # one-frame out-and-back outliers of the donor (e.g. melee_heavy_01_hit_to_idle frame 1: the right hand 39 cm up
            Pp = self.W[:, BODY_IX][:, :, :3, 3]                    # and back): the frame becomes the halfway pose of its neighbours
            sag = (np.linalg.norm(Pp[1:-1] - (Pp[:-2] + Pp[2:]) / 2, axis=-1) - np.linalg.norm(Pp[2:] - Pp[:-2], axis=-1) / 2).max(1)
            t = int(np.argmax(sag)) + 1
            if sag[t - 1] <= DESPIKE_CM / 100.0 or len(self.despiked) > 8:
                break
            self.lt[t] = (self.lt[t - 1] + self.lt[t + 1]) / 2
            self.lr[t] = qslerp(self.lr[t - 1], self.lr[t + 1], np.full(self.lr.shape[1], 0.5))
            self.lr = qcont(self.lr)
            self.W = fk(self.lt, self.lr)
            self.despiked.append(dict(frame=t, sagitta_cm=round(float(sag[t - 1]) * 100, 1)))
        self.seam_deg = [round(float(rot_step_max(self.lr[p['start'] - 1:p['start'] + 1])), 1) for p in self.parts[1:]]
        self.loop = self.parts[0]['phases'].get('loop')

    def hips_world(self, fr):
        """donor hips position in the NPC world (root path applied) at fractional frames."""
        h = self.W[:, IX['Hips'], :3, 3]
        hw = np.einsum('tij,tj->ti', self.root_R, h) + self.root_t
        return interp_rows(hw, fr)


def qcont(q):
    q = q.copy()
    for i in range(1, len(q)):
        flip = np.sum(q[i] * q[i - 1], -1) < 0
        q[i][flip] *= -1
    return q


def rot_step_max(lr2):
    d = np.abs(np.sum(lr2[0, BODY_IX] * lr2[1, BODY_IX], -1))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1))).max()


def fist_forward(W, side):
    """forward (+Y) offset of the wrist from Spine3 [T] (root / capsule frame)."""
    return W[:, IX[side + 'Hand'], 1, 3] - W[:, IX['Spine3'], 1, 3]


def donor_impact(sq, side):
    f = fist_forward(sq.W, side)
    lo, hi = 0, sq.parts[0]['frames'] - 1
    if sq.loop:
        lo, hi = max(0, sq.loop[0] - 3), min(sq.n - 1, sq.loop[1] + 3)
    return lo + int(np.argmax(f[lo:hi + 1]))


def vanilla(clip):
    d = MC.load_dump(MC.clip_path(SET, clip))
    ph = {e['eventName']: (e['startFrame'], e['startFrame'] + e['durationInFrames']) for e in d['meta']['events'] if e.get('type') == 'Phase'}
    return d, ph


def vanilla_impact(clip):
    d, ph = vanilla(clip)
    W = MC.world_all(d)
    a, b = ph.get('active', (0, len(W) - 1))
    best = None
    for s in ('Left', 'Right'):
        y = W[:, IX[s + 'Hand'], 1, 3]
        f = a + int(np.argmax(y[a:b + 1]))
        g = float(y[f] - y[a])
        if best is None or g > best[2]:
            best = (s, f, g)
    return best[0], best[1], len(W), ph


# ------------------------------------------------------------------ capsule model
def capsule_path(kind, T, lunge, lunge_start):
    c = np.zeros((T, 3))
    i = np.arange(T, dtype=float)
    if kind in ('attack', 'heavy'):
        if lunge:
            dur = 0.18 * 30.0
            c[:, 1] = 0.70 * np.clip((i - lunge_start) / dur, 0, 1)
            model = 'lunge 0.70 m in 0.18 s (3.9 m/s) from frame %g' % lunge_start
        else:
            c[:, 1] = 0.025 * np.clip(i / 2.5, 0, 1)
            model = '2.5 cm over frames 0-2.5 (0.97 m/s impulse)'
    else:
        model = 'static'
    return c, model


# ------------------------------------------------------------------ feet
def foot_track(T, p0, steps):
    """world ankle path [T,3] from a start position and steps [(f0, f1, p_to, lift_m)] (smoothstep along, sine lift)."""
    pos = np.repeat(np.asarray(p0, float)[None], T, 0)
    cur = np.asarray(p0, float).copy()
    for a, b, to, lift in steps:
        a, b = int(a), int(b)
        to = np.asarray(to, float)
        for f in range(a, min(b, T - 1) + 1):
            u = (f - a) / max(b - a, 1)
            pos[f] = cur + sstep(u) * (to - cur)
            pos[f, 2] += lift * np.sin(np.pi * u)
        if b + 1 < T:
            pos[b + 1:] = to
        cur = to
    return pos


def plan_feet(T, c, start, end, mode, f_settle=None, lunge_start=0):
    """start / end: {side: ankle position in the CAPSULE frame at frame 0 / at the last frame}.  -> {side: world track}, steps info."""
    tracks, info = {}, {}
    lead, rear = 'Left', 'Right'
    for side in (lead, rear):
        p0 = start[side] + c[0]
        p1 = end[side] + c[-1]
        steps = []
        if mode == 'lunge':
            a = lunge_start + (0 if side == lead else 5)
            b = a + (8 if side == lead else 9)
            mid = p0 + (c[min(T - 1, int(lunge_start + 6))] - c[0]) + np.array([0, 0.0, 0])
            steps.append((a, b, mid, 0.07 if side == lead else 0.05))
            if np.linalg.norm((p1 - mid)[:2]) > STEP_EPS and f_settle is not None:
                s0 = f_settle + (0 if side == lead else 5)
                steps.append((s0, s0 + 7, p1, max(0.05, 0.02 + 0.3 * np.linalg.norm((p1 - mid)[:2]))))
        elif mode == 'transfer':        # equip / unequip: one step each from the start stance to the end stance
            a, b = f_settle[side]
            if np.linalg.norm((p1 - p0)[:2]) > STEP_EPS:
                steps.append((a, b, p1, max(0.05, 0.025 + 0.25 * np.linalg.norm((p1 - p0)[:2]))))
        else:
            if np.linalg.norm((p1 - p0)[:2]) > STEP_EPS and f_settle is not None:
                s0 = f_settle + (0 if side == lead else 5)
                steps.append((s0, min(T - 1, s0 + 7), p1, max(0.05, 0.015 + 0.3 * np.linalg.norm((p1 - p0)[:2]))))
        tracks[side] = foot_track(T, p0, steps)
        info[side] = [dict(frames=[int(s[0]), int(s[1])], dist_cm=round(float(np.linalg.norm((np.asarray(s[2]) - (p0 if k == 0 else steps[k - 1][2]))[:2]) * 100), 1),
                           lift_cm=round(float(s[3]) * 100, 1)) for k, s in enumerate(steps)]
    return tracks, info


# ------------------------------------------------------------------ anatomy fixes
def fix_spine(lt, lr, D3=None):
    """spread the Hips -> Spine3 rotation over the spine (SPINE_CUM); Spine3 world rotation kept (or set to D3 = its world delta)."""
    W = fk(lt, lr)
    Dh = delta(W, IX['Hips'])
    D3 = delta(W, IX['Spine3']) if D3 is None else D3
    rv = R.from_matrix(np.swapaxes(Dh, -1, -2) @ D3).as_rotvec()
    for b, f in zip(('Spine', 'Spine1', 'Spine2', 'Spine3'), SPINE_CUM):
        Db = Dh @ R.from_rotvec(rv * f).as_matrix()
        set_world_rot(lt, lr, IX[b], Db @ REST_W[IX[b], :3, :3])


def yaw_of(v):
    return np.degrees(np.arctan2(v[..., 1], v[..., 0]))


def chest_hips_yaw(W):
    ch = W[:, IX['RightArm'], :3, 3] - W[:, IX['LeftArm'], :3, 3]
    hp = W[:, IX['RightUpLeg'], :3, 3] - W[:, IX['LeftUpLeg'], :3, 3]
    return (yaw_of(ch) - yaw_of(hp) + 180) % 360 - 180


def hips_follow(lt, lr, frac):
    """rotate the hips about world Z by frac x (chest yaw - hips yaw), chest world rotation kept."""
    W = fk(lt, lr)
    D3 = delta(W, IX['Spine3'])
    psi = np.radians(chest_hips_yaw(W) * frac)
    Rz = R.from_rotvec(np.stack([np.zeros_like(psi), np.zeros_like(psi), psi], 1)).as_matrix()
    h = IX['Hips']
    lr[:, h] = R.from_matrix(Rz @ R.from_quat(lr[:, h]).as_matrix()).as_quat()
    fix_spine(lt, lr, D3)


def scale_chain(lt, lr, base, chain, s):
    """scale the world rotation of the chain joints relative to `base` (rest-relative deltas) by s [T] (1 = unchanged)."""
    W = fk(lt, lr)
    Db = delta(W, IX[base])
    targets = []
    for j in chain:
        rv = R.from_matrix(np.swapaxes(Db, -1, -2) @ delta(W, IX[j])).as_rotvec() * s[:, None]
        targets.append(Db @ R.from_rotvec(rv).as_matrix() @ REST_W[IX[j], :3, :3])
    for j, Rw in zip(chain, targets):
        set_world_rot(lt, lr, IX[j], Rw)


def scale_clav(lt, lr, side, s):
    W = fk(lt, lr)
    j, a = IX[side + 'Shoulder'], IX[side + 'Arm']
    Ra = W[:, a, :3, :3].copy()
    rest = R.from_quat(np.repeat(REST_LR[j][None], len(lt), 0))
    rel = (rest.inv() * R.from_quat(lr[:, j])).as_rotvec() * s[:, None]
    lr[:, j] = (rest * R.from_rotvec(rel)).as_quat()
    set_world_rot(lt, lr, a, Ra)


def wrist_to_forearm(lt, lr, side, keep_deg=20.0):
    """move hand twist beyond keep_deg (about the hand's local X, rest-relative) into the forearm; hand world rotation kept."""
    W = fk(lt, lr)
    h, f = IX[side + 'Hand'], IX[side + 'ForeArm']
    Rh = W[:, h, :3, :3].copy()
    rest_h = R.from_quat(np.repeat(REST_LR[h][None], len(lt), 0))
    rel = (rest_h.inv() * R.from_quat(lr[:, h])).as_matrix()
    _, tw = AG_swing_twist(rel)
    ex = np.sign(tw) * np.maximum(np.abs(tw) - keep_deg, 0)
    if not np.any(ex):
        return 0.0
    Rx = R.from_rotvec(np.stack([np.radians(ex), np.zeros_like(ex), np.zeros_like(ex)], 1))
    lr[:, f] = (R.from_quat(lr[:, f]) * Rx).as_quat()
    set_world_rot(lt, lr, h, Rh)
    return float(np.abs(ex).max())


def AG_swing_twist(Rm):
    return GB.AG.swing_twist(Rm, 0)


def eye_push(lt, lr):
    """wrist >= EYE_MIN from the eyes midpoint (arm hinge IK, fist world rotation kept)."""
    W = fk(lt, lr)
    eyes = (W[:, IX['LeftEye'], :3, 3] + W[:, IX['RightEye'], :3, 3]) / 2
    moved = 0.0
    for side in ('Left', 'Right'):
        hnd = W[:, IX[side + 'Hand'], :3, 3]
        v = hnd - eyes
        d = np.linalg.norm(v, axis=1)
        need = d < EYE_MIN
        if not need.any():
            continue
        # smooth the push over time (no pops): push distance profile, dilated + smoothed
        push = np.maximum(EYE_MIN - d, 0)
        push = smooth(dilate(push, 2), 2)
        tgt = hnd + v / np.maximum(d, 1e-6)[:, None] * push[:, None]
        w = (push > 1e-4).astype(float)
        HI.solve(fk, PAR, REST_LR, lt, lr, IX[side + 'Arm'], IX[side + 'ForeArm'], IX[side + 'Hand'], tgt, weight=w, end_mode='world')
        moved = max(moved, float(push.max()))
    return moved * 100


def dilate(x, n):
    y = x.copy()
    for k in range(1, n + 1):
        y[k:] = np.maximum(y[k:], x[:-k])
        y[:-k] = np.maximum(y[:-k], x[k:])
    return y


def smooth(x, sig):
    if sig <= 0 or len(x) < 3:
        return x
    r = int(np.ceil(3 * sig))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sig) ** 2)
    k /= k.sum()
    xp = np.pad(x, (r, r), mode='edge')
    return np.convolve(xp, k, mode='valid')


GROUPS = {
    'neck': ['neck_twist', 'neck_bend'],
    'clav_L': ['clav_swing_L', 'clav_twist_L', 'clav_shift_L_cm'],
    'clav_R': ['clav_swing_R', 'clav_twist_R', 'clav_shift_R_cm'],
    'chest': ['shoulder_width_cm', 'spine_twist_total'],
}


def viol_of(W, feats):
    g = GB.frame_gate(W[:, :73], RG, steps=False, al=AL)
    return np.array([sum(float(g['viol'][k][t]) for k in feats if k in g['viol']) for t in range(len(W))])


def choose_scale(lt, lr, apply, feats, grid):
    """per frame the grid value with the smallest violation of the group's features (grid ordered mildest first; a later value must improve
    the violation strictly).  apply(lt, lr, s [T]) modifies in place.  -> (value [T], residual violation [T])."""
    T = len(lt)
    best = np.full(T, grid[0], float)
    bv = None
    for gval in grid:
        lt2, lr2 = lt.copy(), lr.copy()
        apply(lt2, lr2, np.full(T, gval))
        v = viol_of(fk(lt2, lr2), feats)
        if bv is None:
            bv = v
            if not (v > 1e-9).any():
                break
            continue
        imp = v < bv - 1e-6
        best[imp] = gval
        bv = np.where(imp, v, bv)
        if not (bv > 1e-9).any():
            break
    return best, bv


_BASE_Q = None


def base_q():
    """local rotations (xyzw, game axes) of V's melee base body (base_body.compose_base: shipped ready stance + upright idle spine)."""
    global _BASE_Q
    if _BASE_Q is None:
        L = BB.compose_base()[0]
        assert BB.rig()[0] == P.names[:73]
        _BASE_Q = R.from_matrix(L[:, :3, :3]).as_quat()
    return _BASE_Q


def to_base(lt, lr, joints, s, keep_world=()):
    """slerp the local rotations of `joints` towards the base body by s [T] (0 = unchanged); world rotation of keep_world joints kept."""
    W0 = fk(lt, lr) if keep_world else None
    bq = base_q()
    for j in joints:
        lr[:, IX[j]] = qslerp(lr[:, IX[j]], np.repeat(bq[IX[j]][None], len(lt), 0), s)
    for j in keep_world:
        set_world_rot(lt, lr, IX[j], W0[:, IX[j], :3, :3])


FIX_GROUPS = [   # (name, features, joints, keep-world joints)
    ('spine', ['spine_bend_Spine', 'spine_bend_Spine1', 'spine_bend_Spine2', 'spine_bend_Spine3', 'spine_twist_Spine', 'spine_twist_Spine1',
               'spine_twist_Spine2', 'spine_twist_Spine3', 'spine_bend_total', 'spine_twist_total', 'shoulder_width_cm', 'torso_lean_deg'],
     ['Spine', 'Spine1', 'Spine2', 'Spine3'], ()),
    ('neck', ['neck_twist', 'neck_bend'], ['Neck', 'Neck1', 'Head'], ()),
    ('clav_L', ['clav_swing_L', 'clav_twist_L', 'clav_shift_L_cm', 'clav_gap_cm', 'shoulder_width_cm'], ['LeftShoulder'], ('LeftArm',)),
    ('clav_R', ['clav_swing_R', 'clav_twist_R', 'clav_shift_R_cm', 'clav_gap_cm', 'shoulder_width_cm'], ['RightShoulder'], ('RightArm',)),
]
HIPS_GRID = [0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9]
BASE_GRID = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]


def tsmooth(s, T):
    return np.clip(np.maximum(smooth(dilate(s, 2), 1.5), s), 0, 1) if T > 4 else s     # never below the per-frame choice



RETRACT_GRID = [0.0, 3.0, 6.0, 9.0, 12.0]


def clav_retract(lt, lr, side, ang):
    """rotate the clavicle back about world Z by ang [T] degrees (shoulder joint moves back / out); upper-arm world rotation kept."""
    W = fk(lt, lr)
    j, a = IX[side + 'Shoulder'], IX[side + 'Arm']
    Ra = W[:, a, :3, :3].copy()
    th = np.radians(ang) * (1.0 if side == 'Left' else -1.0)
    Rz = R.from_rotvec(np.stack([np.zeros_like(th), np.zeros_like(th), th], 1)).as_matrix()
    set_world_rot(lt, lr, j, Rz @ W[:, j, :3, :3])
    set_world_rot(lt, lr, a, Ra)


ELEV_GRID = [0.0, 2.0, 4.0, 6.0, 8.0]


def clav_elevate(lt, lr, side, ang):
    """raise the shoulder joint: rotate the clavicle by ang [T] degrees about the horizontal axis normal to it; upper-arm world rotation kept."""
    W = fk(lt, lr)
    j, a = IX[side + 'Shoulder'], IX[side + 'Arm']
    Ra = W[:, a, :3, :3].copy()
    d = W[:, a, :3, 3] - W[:, j, :3, 3]
    ax = np.cross(d, np.array([0.0, 0.0, 1.0]))
    ax /= np.maximum(np.linalg.norm(ax, axis=1, keepdims=True), 1e-9)
    Rr = R.from_rotvec(ax * np.radians(ang)[:, None]).as_matrix()
    set_world_rot(lt, lr, j, Rr @ W[:, j, :3, :3])
    set_world_rot(lt, lr, a, Ra)


FIX_RELEASE = None     # polish 2026-11-09 (opt-in per SPEC 'fix_release'): (a, b) frames; the time-smoothing tail of the spine / neck /
                       # clavicle fixes is faded out between a and b (the per-frame need is always kept), so a fix needed in the wind-up
                       # does not bleed into the strike and hold the fist back (combo_attack_01: spine to_base 0.71 at the impact frame
                       # cut the extension from 74 to 56 cm; the pose at the impact passes the gate without it)


def _release(s, s_raw):
    if FIX_RELEASE is None:
        return s
    a, b = FIX_RELEASE
    t = np.arange(len(s), dtype=float)
    win = 1.0 - sstep((t - a) / max(b - a, 1e-6))
    return np.maximum(s_raw, s * win)


def anatomy_fix(lt, lr, log):
    """only where V's base-body gate fails: hips take part of the chest twist (feet pivot with them in solve_legs), then spine / neck /
    clavicles are pulled towards the base body (mildest weight that passes, per frame, time-smoothed), wrist twist into the forearm, wrist
    away from the eyes."""
    T = len(lt)
    if FIX_RELEASE is not None:
        return anatomy_fix_release(lt, lr, log)
    fr, _ = choose_scale(lt, lr, lambda a, b, s: hips_follow(a, b, s), ['shoulder_width_cm', 'spine_twist_total'], HIPS_GRID)
    fr = tsmooth(fr, T)
    if fr.max() > 0:
        hips_follow(lt, lr, fr)
    log['hips_follow_max'] = round(float(fr.max()), 2)
    for rnd in range(2):                # second round: frames a later group / the smoothing left outside the gate
        for name, feats, joints, keep in FIX_GROUPS:
            s, res = choose_scale(lt, lr, lambda a, b, s, j=joints, k=keep: to_base(a, b, j, s, k), feats, BASE_GRID)
            s = tsmooth(s, T)
            if s.max() > 0:
                to_base(lt, lr, joints, s, keep)
            log['to_base_max_%s_r%d' % (name, rnd)] = round(float(s.max()), 2)
    for sd in ('Left', 'Right'):
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        a_, _ = choose_scale(lt, lr, lambda a, b, s, sd=sd: clav_retract(a, b, sd, s), feats, RETRACT_GRID)
        a_ = tsmooth(a_ / RETRACT_GRID[-1], len(lt)) * RETRACT_GRID[-1]
        if a_.max() > 0:
            clav_retract(lt, lr, sd, a_)
        log['clav_retract_max_deg_' + sd[0]] = round(float(a_.max()), 1)
    for sd in ('Left', 'Right'):        # clavicle floor (swing from bind >= V's minimum) without narrowing: a small shrug
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        a_, _ = choose_scale(lt, lr, lambda a, b, s, sd=sd: clav_elevate(a, b, sd, s), feats, ELEV_GRID)
        a_ = tsmooth(a_ / ELEV_GRID[-1], len(lt)) * ELEV_GRID[-1]
        if a_.max() > 0:
            clav_elevate(lt, lr, sd, a_)
        log['clav_elevate_max_deg_' + sd[0]] = round(float(a_.max()), 1)
    for sd in ('Left', 'Right'):
        log['wrist_twist_moved_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)
    log['eye_push_cm'] = round(eye_push(lt, lr), 1)
    log['fist_sep_push_cm'] = round(fist_separate(lt, lr), 1)
    for sd in ('Left', 'Right'):        # the arm IK above keeps the fist's world rotation: twist that came back into the wrist goes to the forearm again
        log['wrist_twist_moved2_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)


def anatomy_fix_release(lt, lr, log):
    """anatomy_fix with the smoothing tails released by FIX_RELEASE (same steps / order / grids; per-frame minimum kept)."""
    T = len(lt)
    fr, _ = choose_scale(lt, lr, lambda a, b, s: hips_follow(a, b, s), ['shoulder_width_cm', 'spine_twist_total'], HIPS_GRID)
    fr = _release(tsmooth(fr, T), fr)
    if fr.max() > 0:
        hips_follow(lt, lr, fr)
    log['hips_follow_max'] = round(float(fr.max()), 2)
    for rnd in range(2):
        for name, feats, joints, keep in FIX_GROUPS:
            s0, res = choose_scale(lt, lr, lambda a, b, s, j=joints, k=keep: to_base(a, b, j, s, k), feats, BASE_GRID)
            s = _release(tsmooth(s0, T), s0)
            if s.max() > 0:
                to_base(lt, lr, joints, s, keep)
            log['to_base_max_%s_r%d' % (name, rnd)] = round(float(s.max()), 2)
    for sd in ('Left', 'Right'):
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        a0, _ = choose_scale(lt, lr, lambda a, b, s, sd=sd: clav_retract(a, b, sd, s), feats, RETRACT_GRID)
        a_ = _release(tsmooth(a0 / RETRACT_GRID[-1], len(lt)), a0 / RETRACT_GRID[-1]) * RETRACT_GRID[-1]
        if a_.max() > 0:
            clav_retract(lt, lr, sd, a_)
        log['clav_retract_max_deg_' + sd[0]] = round(float(a_.max()), 1)
    for sd in ('Left', 'Right'):
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        a0, _ = choose_scale(lt, lr, lambda a, b, s, sd=sd: clav_elevate(a, b, sd, s), feats, ELEV_GRID)
        a_ = _release(tsmooth(a0 / ELEV_GRID[-1], len(lt)), a0 / ELEV_GRID[-1]) * ELEV_GRID[-1]
        if a_.max() > 0:
            clav_elevate(lt, lr, sd, a_)
        log['clav_elevate_max_deg_' + sd[0]] = round(float(a_.max()), 1)
    for sd in ('Left', 'Right'):
        log['wrist_twist_moved_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)
    log['eye_push_cm'] = round(eye_push(lt, lr), 1)
    log['fist_sep_push_cm'] = round(fist_separate(lt, lr), 1)
    for sd in ('Left', 'Right'):
        log['wrist_twist_moved2_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)
    log['fix_release'] = list(FIX_RELEASE)


FIST_GAP = 2 * HEAD_R * 0 + 2 * 0.045 + 0.012


def fist_separate(lt, lr):
    """fists >= FIST_GAP apart (centre to centre): the slower fist (the guard hand) moves away along the separation (arm hinge IK)."""
    W = fk(lt, lr)
    fc = fist_centers(W)
    d = fc['Right'] - fc['Left']
    n = np.linalg.norm(d, axis=1)
    deficit = smooth(dilate(np.maximum(FIST_GAP - n, 0), 2), 1.5) if len(lt) > 4 else np.maximum(FIST_GAP - n, 0)
    if deficit.max() <= 1e-4:
        return 0.0
    sp = {s: np.r_[0, np.linalg.norm(np.diff(fc[s], axis=0), axis=1)] for s in fc}
    for s in ('Left', 'Right'):
        mine = (sp[s] <= sp['Right' if s == 'Left' else 'Left']) & (deficit > 1e-4)
        if not mine.any():
            continue
        u = (-d if s == 'Left' else d) / np.maximum(n, 1e-6)[:, None]
        hnd = W[:, IX[s + 'Hand'], :3, 3]
        tgt = hnd + u * (deficit * mine)[:, None]
        HI.solve(fk, PAR, REST_LR, lt, lr, IX[s + 'Arm'], IX[s + 'ForeArm'], IX[s + 'Hand'], tgt, weight=mine.astype(float), end_mode='world')
    return float(deficit.max() * 100)


# ------------------------------------------------------------------ crossed hands (final2 2026-11-09, SPEC 'uncross')
UNCROSS_WRIST = 0.05     # m: lateral gap right - left of the hand joints kept (check_cross rule: crossed = left hand right of the right hand)
UNCROSS_FIST = 0.005     # m: lateral gap of the fist centres (the fists may not swap sides either; 5 mm over the 0.5 mm convergence of the
                         # loop, so a fixed frame never ends at -0.0x cm; the clips without crossing: >= 0.2 cm)
UNCROSS_ARM = 0.02       # m: arm-to-arm clearance of the capsules below (the clips without crossing: >= 1.5 cm)
UNCROSS_NEAR = 0.15      # m: the fix acts only where the fist centres are closer laterally than this
UNCROSS_RAMP = 3.0       # frames of smooth in / out around the frames that need it (max of smoothstep bumps)
ARM_R_UP, ARM_R_FORE = 0.05, 0.04      # m: capsule radii upper arm / forearm-to-fist-centre (arm-to-arm clearance)


def hands_lateral(W):
    """lateral axis x = the shoulder line LeftArm -> RightArm in the ground plane (per frame); lateral coordinates (m, from the chest midline)
    of the hand joints and of the fist centres -> x, left hand, right hand, left fist, right fist."""
    P = W[:, :, :3, 3]
    la, ra = P[:, IX['LeftArm']], P[:, IX['RightArm']]
    mid = (la + ra) / 2
    x = ra - la
    x[:, 2] = 0
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    fc = fist_centers(W)

    def lat(p):
        return np.einsum('ti,ti->t', p - mid, x)
    return x, lat(P[:, IX['LeftHand']]), lat(P[:, IX['RightHand']]), lat(fc['Left']), lat(fc['Right'])


def seg_dist(p1, q1, p2, q2):
    """per frame minimum distance between the segments p1-q1 and p2-q2 ([T,3] each; closest points clamped to the segments)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e = np.einsum('ti,ti->t', d1, d1), np.einsum('ti,ti->t', d2, d2)
    f, c, b = np.einsum('ti,ti->t', d2, r), np.einsum('ti,ti->t', d1, r), np.einsum('ti,ti->t', d1, d2)
    den = a * e - b * b
    s = np.where(den > 1e-12, np.clip((b * f - c * e) / np.maximum(den, 1e-12), 0, 1), 0.0)
    t = (b * s + f) / np.maximum(e, 1e-12)
    s = np.where(t < 0, np.clip(-c / np.maximum(a, 1e-12), 0, 1), np.where(t > 1, np.clip((b - c) / np.maximum(a, 1e-12), 0, 1), s))
    t = np.clip(t, 0, 1)
    return np.linalg.norm((p1 + d1 * s[:, None]) - (p2 + d2 * t[:, None]), axis=1)


def arm_clearance(W):
    """per frame minimum surface clearance (m) between the left and the right arm: capsules upper arm (Arm -> ForeArm, ARM_R_UP) and forearm
    (ForeArm -> fist centre, ARM_R_FORE); < 0 = the arms interpenetrate."""
    P = W[:, :, :3, 3]
    fc = fist_centers(W)
    seg = {s: [(P[:, IX[s + 'Arm']], P[:, IX[s + 'ForeArm']], ARM_R_UP), (P[:, IX[s + 'ForeArm']], fc[s], ARM_R_FORE)] for s in ('Left', 'Right')}
    out = np.full(len(W), np.inf)
    for a0, a1, ra in seg['Left']:
        for b0, b1, rb in seg['Right']:
            out = np.minimum(out, seg_dist(a0, a1, b0, b1) - ra - rb)
    return out


def hands_cross(W):
    """crossing record of a clip: minimum lateral gaps right - left (cm) of the hand joints / fist centres, their crossed frames (< 0), minimum
    arm-to-arm clearance (cm)."""
    _, lh, rh, lf, rf = hands_lateral(W)
    ac = arm_clearance(W)
    return dict(wrist_gap_min_cm=round(float((rh - lh).min() * 100), 2), fist_gap_min_cm=round(float((rf - lf).min() * 100), 2),
                crossed_frames=np.nonzero(rh < lh)[0].tolist(), fists_crossed_frames=np.nonzero(rf < lf)[0].tolist(),
                arm_clear_min_cm=round(float(ac.min() * 100), 2))


def uncross_need(W):
    """lateral shift (m) per frame that uncrosses the hands: wrist gap >= UNCROSS_WRIST, fist-centre gap >= UNCROSS_FIST and >= what keeps the
    fist centres FIST_GAP apart given their height / depth separation (fists on one level pass each other laterally), arm-to-arm clearance
    >= UNCROSS_ARM; only where the fist centres are within UNCROSS_NEAR laterally."""
    x, lh, rh, lf, rf = hands_lateral(W)
    fc = fist_centers(W)
    d = fc['Right'] - fc['Left']
    dperp = np.linalg.norm(d - np.einsum('ti,ti->t', d, x)[:, None] * x, axis=1)
    f3 = np.sqrt(np.maximum(FIST_GAP ** 2 - dperp ** 2, 0.0))
    ac = arm_clearance(W)
    need = np.maximum.reduce([UNCROSS_WRIST - (rh - lh), np.maximum(UNCROSS_FIST, f3) - (rf - lf), np.where(ac < UNCROSS_ARM, UNCROSS_ARM - ac, 0.0),
                              np.zeros(len(W))])
    need[(rf - lf) > UNCROSS_NEAR] = 0.0
    return need, x


def uncross(lt, lr, side, iters=12):
    """crossed hands -> the guard hand `side` (the arm that is not striking / struck) is moved sideways to its own side along the shoulder line
    (ground plane) by the need of uncross_need(), smooth in / out over UNCROSS_RAMP frames (max of smoothstep bumps around every frame that
    needs it), arm hinge IK with the fist world rotation kept; repeated on the residual.  Frame 0 and the last frame are never moved (start /
    guard continuity).  In place; a clip without crossing is not touched.  -> log (frames, shift, hand displacement, crossing before / after)."""
    T = len(lt)
    sg = -1.0 if side == 'Left' else 1.0
    t = np.arange(T, dtype=float)
    W = fk(lt, lr)
    W0 = W
    log = dict(hand=side, before=hands_cross(W))
    tot = np.zeros(T)
    for _ in range(iters):
        need, x = uncross_need(W)
        if need.max() < 5e-4:
            break
        env = np.zeros(T)
        for s in np.nonzero(need > 0)[0]:
            env = np.maximum(env, need[s] * (1.0 - sstep(np.abs(t - s) / UNCROSS_RAMP)))
        if env[0] > 0 or env[-1] > 0:
            log['ends_kept'] = True
            env[0] = env[-1] = 0.0
        tgt = W[:, IX[side + 'Hand'], :3, 3] + sg * x * env[:, None]
        HI.solve(fk, PAR, REST_LR, lt, lr, IX[side + 'Arm'], IX[side + 'ForeArm'], IX[side + 'Hand'], tgt, weight=(env > 1e-6).astype(float),
                 end_mode='world')
        tot += env
        W = fk(lt, lr)
    h = IX[side + 'Hand']
    log.update(frames=np.nonzero(tot > 1e-6)[0].tolist(), shift_max_cm=round(float(tot.max() * 100), 1),
               hand_moved_max_cm=round(float(np.linalg.norm(W[:, h, :3, 3] - W0[:, h, :3, 3], axis=1).max() * 100), 1), after=hands_cross(W))
    return log


HIPS_YAW_CAP = np.radians(25.0)   # hips yaw change vs the guard is soft-capped (tanh); the rest goes into the spine (chest world rotation kept)


def limit_hips_yaw(G, lt, lr):
    W = fk(lt, lr)
    d = np.unwrap(hips_yaw(W)) - hips_yaw(G.W[None])[0]
    d = (d + np.pi) % (2 * np.pi) - np.pi
    delta_ = HIPS_YAW_CAP * np.tanh(d / HIPS_YAW_CAP) - d
    Rz = lambda a: R.from_rotvec(np.stack([np.zeros_like(a), np.zeros_like(a), a], 1)).as_matrix()
    Wsp = {b: W[:, IX[b], :3, :3].copy() for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')}
    h = IX['Hips']
    lr[:, h] = R.from_matrix(Rz(delta_) @ R.from_quat(lr[:, h]).as_matrix()).as_quat()
    for b, f in zip(('Spine', 'Spine1', 'Spine2', 'Spine3'), SPINE_CUM):
        set_world_rot(lt, lr, IX[b], Rz(delta_ * (1 - f)) @ Wsp[b])
    return float(np.degrees(np.abs(delta_)).max())


def hips_yaw(W):
    hp = W[:, IX['RightUpLeg'], :3, 3] - W[:, IX['LeftUpLeg'], :3, 3]
    return np.radians(yaw_of(hp))


FOOT_PIVOT = 0.8      # share of the hips yaw change (vs the guard) the planted feet follow (pivot about the ankle: no ankle slide)



TOE_LOCAL = {}


def knee_to_pole(lt, lr, side, toe_dir):
    """turn the thigh about the hip -> ankle axis so the knee points over the toes (horizontal foot direction); ankle position unchanged."""
    W = fk(lt, lr)
    A, B, C = (W[:, IX[side + n], :3, 3] for n in ('UpLeg', 'Leg', 'Foot'))
    u = C - A
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    pd = np.array(toe_dir, float)
    pd[:, 2] = 0
    vcur = (B - A) - np.sum((B - A) * u, 1, keepdims=True) * u
    vdes = pd - np.sum(pd * u, 1, keepdims=True) * u
    ang = np.arctan2(np.sum(np.cross(vcur, vdes) * u, 1), np.sum(vcur * vdes, 1))
    Rt = R.from_rotvec(u * ang[:, None]).as_matrix()
    set_world_rot(lt, lr, IX[side + 'UpLeg'], Rt @ W[:, IX[side + 'UpLeg'], :3, :3])

# ------------------------------------------------------------------ legs
def solve_legs(lt, lr, tracks, c, foot_R, toe_lr):
    """ankle targets (capsule frame) = world tracks - capsule; hips lowered where a leg cannot reach; foot world rotation = foot_R."""
    T = len(lt)
    h = IX['Hips']
    for it in range(3):
        W = fk(lt, lr)
        need = np.zeros(T)
        for side in ('Left', 'Right'):
            a, b, cc = IX[side + 'UpLeg'], IX[side + 'Leg'], IX[side + 'Foot']
            l1 = np.linalg.norm(REST_W[b, :3, 3] - REST_W[a, :3, 3])
            l2 = np.linalg.norm(REST_W[cc, :3, 3] - REST_W[b, :3, 3])
            tgt = tracks[side] - c
            dist = np.linalg.norm(tgt - W[:, a, :3, 3], axis=1)
            need = np.maximum(need, dist - 0.975 * (l1 + l2))
        if need.max() <= 1e-4:
            break
        lt[:, h, 2] -= smooth(dilate(np.maximum(need, 0), 2), 1.5) * 1.15
    res = {}
    for side in ('Left', 'Right'):
        a, b, cc = IX[side + 'UpLeg'], IX[side + 'Leg'], IX[side + 'Foot']
        res[side] = HI.solve(fk, PAR, REST_LR, lt, lr, a, b, cc, tracks[side] - c, end_mode='local', pole=True)
        knee_to_pole(lt, lr, side, np.einsum('tij,j->ti', foot_R[side], TOE_LOCAL[side]))
        set_world_rot(lt, lr, cc, foot_R[side])
        lr[:, IX[side + 'ToeBase']] = toe_lr[side]
    return res


# ------------------------------------------------------------------ guard pose
class Guard:
    def __init__(self):
        sq = Seq([('idle_combat', False)])
        self.raw_lt, self.raw_lr = sq.lt[:1].copy(), sq.lr[:1].copy()
        W = fk(self.raw_lt, self.raw_lr)
        self.feet = {s: W[0, IX[s + 'Foot'], :3, 3].copy() for s in ('Left', 'Right')}
        self.foot_R = {s: W[0, IX[s + 'Foot'], :3, :3].copy() for s in ('Left', 'Right')}
        self.toe = {s: self.raw_lr[0, IX[s + 'ToeBase']].copy() for s in ('Left', 'Right')}
        for s_ in ('Left', 'Right'):
            TOE_LOCAL[s_] = self.foot_R[s_].T @ (W[0, IX[s_ + 'ToeBase'], :3, 3] - W[0, IX[s_ + 'Foot'], :3, 3])
        self.hips = W[0, IX['Hips'], :3, 3].copy()
        self.mid = (self.feet['Left'] + self.feet['Right']) / 2
        lt, lr = self.raw_lt.copy(), self.raw_lr.copy()
        log = {}
        anatomy_fix(lt, lr, log)
        tr = {s: self.feet[s][None].copy() for s in self.feet}
        solve_legs(lt, lr, tr, np.zeros((1, 3)), {s: self.foot_R[s][None] for s in self.feet}, self.toe)
        self.lt, self.lr, self.log = lt, lr, log
        self.W = fk(lt, lr)[0]
        Wi = BB.fkL(BB.compose_base()[2])       # V's idle stance (73 joints, game axes)
        self.idle_feet = {s: Wi[IX[s + 'Foot'], :3, 3].copy() for s in ('Left', 'Right')}
        self.idle_hips = Wi[IX['Hips'], :3, 3].copy()


# ------------------------------------------------------------------ assembly
def offset_start(lr, G_lr, w):
    """rotation offset (G vs frame 0) decaying with w [T] (1 at frame 0)."""
    d = qmul(G_lr[None], qinv(lr[:1]))                                   # [1,N,4]
    dd = qslerp(np.repeat(np.array([0, 0, 0, 1.0])[None, None], lr.shape[1], 1), d, 1.0)[0]
    off = qslerp(np.repeat(np.array([0, 0, 0, 1.0])[None, None], len(lr), 0).repeat(lr.shape[1], 1), np.repeat(dd[None], len(lr), 0),
                 np.repeat(w[:, None], lr.shape[1], 1))
    return qmul(off, lr)


def blend_to(lr, G_lr, w):
    return qslerp(lr, np.repeat(G_lr[None], len(lr), 0), np.repeat(w[:, None], lr.shape[1], 1))


def build_body(G, sq, tau, w_s, w_e, start_lr=None, end_lr=None, hips_mode='shift'):
    """upper body / hips rotation from the donor at tau, start offset towards start_lr (default G) with weight w_s, end blend to end_lr with w_e."""
    lt, lr = resample(sq.lt, sq.lr, tau)
    lr = qcont(lr)
    s_lr = G.lr[0] if start_lr is None else start_lr
    e_lr = G.lr[0] if end_lr is None else end_lr
    keep = [IX['Root']]
    if w_s is not None and w_s.max() > 0:
        lr = offset_start(lr, s_lr, w_s)
    if w_e is not None and w_e.max() > 0:
        lr = blend_to(lr, e_lr, w_e)
    for j in keep:
        lr[:, j] = [0, 0, 0, 1.0]
        lt[:, j] = 0
    return lt, qcont(lr)


def hips_path(G, sq, tau, tracks, c, w_e, k_xy=SHIFT_K, k_z=DZ_K, start=None, cap=SHIFT_MAX):
    """hips position in the capsule frame: feet midpoint (world) + guard offset + scaled donor weight shift / height change."""
    T = len(tau)
    hw = sq.hips_world(tau)
    dh = hw - hw[0]
    sh = dh[:, :2] * k_xy
    n = np.linalg.norm(sh, axis=1, keepdims=True)
    sh = sh * np.minimum(1.0, cap / np.maximum(n, 1e-9))
    fade = 1.0 - (w_e if w_e is not None else np.zeros(T))
    mid_w = (tracks['Left'] + tracks['Right']) / 2
    mid_w[:, 2] = G.mid[2]
    base = mid_w + (G.hips - G.mid)
    out = base.copy()
    out[:, :2] += sh * fade[:, None]
    out[:, 2] += dh[:, 2] * k_z * fade
    if start is not None:           # (hips offset at frame 0, weight [T]) e.g. equip from V's idle stance
        out += start[0][None] * start[1][:, None]
    return out - c


def finish(G, lt, lr, hips_cap, tracks, c, log, fix=True):
    lt[:, IX['Hips']] = hips_cap
    lr[:, LEGS] = G.lr[0, LEGS]          # IK source = the guard legs (knee pole of the guard; the donor legs step / lunge)
    log['hips_yaw_cut_max_deg'] = round(limit_hips_yaw(G, lt, lr), 1)
    if fix:
        anatomy_fix(lt, lr, log)
    dpsi = FOOT_PIVOT * (np.unwrap(hips_yaw(fk(lt, lr))) - hips_yaw(G.W[None])[0])
    dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
    Rz = R.from_rotvec(np.stack([np.zeros_like(dpsi), np.zeros_like(dpsi), dpsi], 1)).as_matrix()
    foot_R = {s: Rz @ G.foot_R[s][None] for s in G.feet}
    log['foot_pivot_max_deg'] = round(float(np.degrees(np.abs(dpsi)).max()), 1)
    log['leg_ik_residual_cm'] = {k: round(v * 100, 2) for k, v in solve_legs(lt, lr, tracks, c, foot_R, G.toe).items()}
    return lt, qcont(lr)


def _attack_once(G, name, spec, lunge_start, hold=None):
    side_v, o_imp, T, ph = vanilla_impact(name)
    side = spec['side']
    assert side == side_v, (name, side, side_v)
    sq = Seq(spec['donors'], despike=spec.get('despike', False), hips_seam=spec.get('hips_seam', False))
    d_imp = donor_impact(sq, side)
    tau0 = max(0.0, d_imp - SPEEDUP * o_imp)
    tau = np.empty(T)
    for i in range(T):
        tau[i] = tau0 + (d_imp - tau0) * i / o_imp if i <= o_imp else min(d_imp + (i - o_imp), sq.n - 1)
    i = np.arange(T, dtype=float)
    w_s = 1.0 - sstep(i / max(o_imp - 1, 1))          # heavies: offset towards the synthesised hold pose (frame 0 == hold pose)
    e0 = o_imp + 3
    e1 = min(T - 2, o_imp + GUARD_BY)
    w_e = sstep((i - e0) / max(e1 - e0, 1))
    st = start_state(G, spec['start'], name) if spec.get('start') else None
    s_lr = HOLDS[name]['lr'] if spec['kind'] == 'heavy' else (st['lr'] if st else None)
    lt, lr = build_body(G, sq, tau, w_s, w_e, start_lr=s_lr)
    c, cmodel = capsule_path(spec['kind'], T, spec.get('lunge'), lunge_start)
    mode = 'lunge' if spec.get('lunge') else 'planted'
    tracks, steps = plan_feet(T, c, G.feet, G.feet, mode, f_settle=int(e0 + 2), lunge_start=lunge_start)
    hips = hips_path(G, sq, tau, tracks, c, w_e, k_xy=spec.get('shift_k', SHIFT_K), cap=spec.get('shift_max', SHIFT_MAX))   # polish: opt-in weight shift
    if st:                   # start pose hips (e.g. the block) faded into the planned path over 8 frames
        hips = hips + (st['hips'] - hips[0])[None] * (1.0 - sstep(i / 8.0))[:, None]
    log = {}
    global FIX_RELEASE
    keep_rel = FIX_RELEASE
    if spec.get('fix_release') is not None:      # polish: release the fix smoothing tails k frames before the vanilla impact
        FIX_RELEASE = (float(o_imp - spec['fix_release']), float(o_imp + spec.get('fix_release_end', 0)))
    try:
        lt, lr = finish(G, lt, lr, hips, tracks, c, log)
    finally:
        FIX_RELEASE = keep_rel
    # frame 0 == the guard (attacks) / the hold pose (heavies): the fixes of the first frames are faded in over START_RAMP frames on the
    # upper body (legs / hips stay as solved), so frame 0 is the exact pose and the first steps carry no pop
    w0 = 1.0 - sstep(i / START_RAMP)
    tgt = HOLDS[name]['lr'] if spec['kind'] == 'heavy' else (st['lr'] if st else G.lr[0])
    lr[:, UPPER] = offset_start(lr[:, UPPER], tgt[UPPER], w0)
    lr = qcont(lr)
    for sd in ('Left', 'Right'):        # the fade mixes per-joint offsets: wrist twist above the limit goes back into the forearm
        log['ramp_wrist_twist_moved_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)
    info = dict(kind=spec['kind'], note=spec.get('note'), donors=sq.parts, seam_deg=sq.seam_deg, donor_despiked=sq.despiked, donor_hips_seam_cm=sq.hips_seam_cm, side=side, impact_vanilla=o_imp, impact_donor=d_imp,
                tau=np.round(tau, 2).tolist(), speedup_pre_impact=round((d_imp - tau0) / max(o_imp, 1), 2), guard_blend=[int(e0), int(e1)],
                capsule=cmodel, feet_mode=mode, steps=steps, fixes=log, vanilla_phases=ph)
    return lt, lr, c, info



WRIST_MAX_M_S = 12.5     # fastest wrist speed accepted when choosing the pre-impact speed-up (elite punches peak around 9-12 m/s)


def wrist_speed(W):
    v = [np.linalg.norm(np.diff(W[:, IX[s + 'Hand'], :3, 3], axis=0), axis=1).max() for s in ('Left', 'Right')]
    return float(max(v) * 30.0)

SPEED_TRIES = (1.5, 1.35, 1.2, 1.05, 0.9)



def finish_loop(G, lt, lr, hips, tracks, c, log):
    """loops: the fixes / IK run on three periods and the middle one is kept, so the seam sees the same smoothing as any other frame."""
    n = len(lt) - 1
    tile = lambda x: np.concatenate([x[:-1]] * 3 + [x[:1]])
    lt3, lr3 = finish(G, tile(lt), qcont(tile(lr)), tile(hips), {s: tile(tracks[s]) for s in tracks}, tile(c), log)
    lt2, lr2 = lt3[n:2 * n + 1].copy(), lr3[n:2 * n + 1].copy()
    log['loop_end_mismatch_deg'] = round(float(rot_step_max(np.stack([lr2[0], lr2[-1]]))), 3)
    lt2[-1], lr2[-1] = lt2[0], lr2[0]
    return lt2, qcont(lr2)


def attack_clip(G, name, spec, lunge_start, hold=None):
    """the fastest pre-impact speed-up (<= 1.5x) whose clip passes V's base gate on every frame (incl. the one-frame rotation step), the
    joint-spike limit and the wrist-speed limit; if none passes all, the gate- and spike-clean try with the slowest wrist, else the last."""
    global SPEEDUP
    keep = SPEEDUP
    tries, res = [], []
    sps = list(spec.get('speeds', SPEED_TRIES))
    chosen = None
    seams = spec.get('hips_seam_tries', (spec.get('hips_seam', False),))     # per speed: donor parts joined at the NPC root, then hips-continuous
    try:
        for sp in sps:
            for hs in seams:
                SPEEDUP = sp
                lt, lr, c, info = _attack_once(G, name, dict(spec, hips_seam=hs), lunge_start, hold)
                W = fk(lt, lr)
                f = GB.AG.features(W[:, :73], RG, step_frames=True)
                oi = info['impact_vanilla']
                rs, sk = float(np.max(f['rot_step'])), spikes_cm(W)
                pre, post = wrist_speed(W[:oi + 2]), wrist_speed(W[oi + 1:])
                gok = bool(GB.frame_gate(W[:, :73], RG, steps=True, al=AL)['ok'].all())
                tries.append(dict(speedup=sp, hips_seam=hs, rot_step=round(rs, 1), spike_cm=round(sk, 1), wrist_pre_m_s=round(pre, 1),
                                  wrist_post_m_s=round(post, 1), base_gate_all=gok))
                clean = gok and rs <= AL['rot_step'][1] and sk <= JUMP_CM
                res.append((clean, pre, (lt, lr, c, info)))
                if clean and pre <= max(WRIST_MAX_M_S, post):
                    chosen = len(res) - 1
                    break
            if chosen is not None:
                break
    finally:
        SPEEDUP = keep
    if chosen is None:
        ok = [k for k, r in enumerate(res) if r[0]]
        chosen = min(ok, key=lambda k: res[k][1]) if ok else len(res) - 1
    lt, lr, c, info = res[chosen][2]
    info['speed_tries'] = tries
    info['speed_chosen'] = tries[chosen]['speedup']
    info['hips_seam_chosen'] = tries[chosen]['hips_seam']
    if spec.get('align'):
        lt, lr, info['align'] = align_impact(lt, lr, info, spec['side'])
    return lt, lr, c, info


def align_impact(lt, lr, info, side):
    """stage 2 step 6: re-time so the full extension (max fist_forward in the vanilla active window) lands on the vanilla impact frame
    (or as close after it as the gates allow: target o, o+1, o+2, o+3).  Frames before the punch keep their timing; walking back from the
    target, each output frame takes the latest built time from which no body joint moves more than its own fastest built step (>= 1 cm)
    to the next output frame, until that meets the original timing; after the target the rest is spread uniformly to the vanilla length
    (start, end and frame count unchanged).  The first target whose clip passes every gate is kept, else the clip is left unchanged."""
    W = fk(lt, lr)
    T = len(lt)
    o0 = info['impact_vanilla']
    a, b = info['vanilla_phases'].get('active', (0, T - 1))
    peak = a + int(np.argmax(fist_forward(W, side)[a:b + 1]))
    if peak <= o0:
        return lt, lr, dict(peak_built=peak, impact_vanilla=o0, changed=False)
    Pj = W[:, BODY_IX][:, :, :3, 3]
    st = np.linalg.norm(np.diff(Pj, axis=0), axis=-1)                      # [T-1, J]
    path = np.concatenate([np.zeros((1, st.shape[1])), np.cumsum(st + 1e-9, axis=0)])
    lim = np.maximum(st.max(0), 0.01)
    fr = np.arange(T, dtype=float)
    tries = []
    for o in range(o0, min(peak, o0 + 4)):
        s = fr.copy()
        s[o] = peak
        k = o
        while k > 0:
            pk = np.array([np.interp(s[k], fr, path[:, j]) for j in range(path.shape[1])])
            tb = max(float(np.interp(pk[j] - lim[j], path[:, j], fr)) for j in range(path.shape[1]))
            if tb <= k - 1:
                break
            s[k - 1] = tb
            k -= 1
        if s[0] > 1e-6:      # polish 2026-11-09: frame 0 stays the start pose (guard / hold; the walk back reached frame 0 before meeting the
            s[:o + 1] = s[:o + 1] - s[0] * (1.0 - sstep(fr[:o + 1] / float(o)))    # original timing): the rest is spread over frames 0..o
        s[o:] = peak + (fr[o:] - o) * (T - 1 - peak) / (T - 1 - o)
        lt2, lr2 = resample(lt, lr, s)
        lr2 = qcont(lr2)
        g = gates(fk(lt2, lr2), np.zeros((T, 3)) + capsule_path('attack', T, False, 0)[0])
        tries.append(dict(target=o, PASS=g['PASS'], spike_cm=g['spike_max_cm'], base_gate_pass=g['base_gate_pass']))
        if g['PASS']:
            return lt2, lr2, dict(peak_built=peak, impact_vanilla=o0, target=o, changed=True, built_time=np.round(s, 2).tolist(), tries=tries)
    return lt, lr, dict(peak_built=peak, impact_vanilla=o0, changed=False, tries=tries)


HOLDS = {}
HOLD_WIND = {'Right': -18.0, 'Left': 12.0}   # hold: chest wound up about world Z (deg), the striking shoulder goes back
HOLD_PULL_CM = 5.0                           # hold: striking fist pulled back along the chest's backward axis (and 2 cm down)
HOLD_PULSE = 0.2                             # hold loop: extra wind-up at the pulse peak


def hold_synth(G, side, k=1.0):
    """charged hold pose from the guard: chest wound up (striking shoulder back), striking fist pulled back towards the shoulder, the
    other fist kept where the guard has it.  The NPC heavy wind-ups drop a hand to the hip / both arms low, so they are not used here.
    -> local rotations [N,4]."""
    lt, lr = G.lt.copy(), G.lr.copy()
    W0 = fk(lt, lr)
    th = np.radians(HOLD_WIND[side] * k)
    Wsp = {b: W0[:, IX[b], :3, :3].copy() for b in ('Spine', 'Spine1', 'Spine2', 'Spine3')}
    for b, f in zip(('Spine', 'Spine1', 'Spine2', 'Spine3'), SPINE_CUM):
        set_world_rot(lt, lr, IX[b], R.from_rotvec([0.0, 0.0, th * f]).as_matrix()[None] @ Wsp[b])
    W1 = fk(lt, lr)
    sl = W1[0, IX['RightArm'], :3, 3] - W1[0, IX['LeftArm'], :3, 3]
    fwd = np.cross([0.0, 0.0, 1.0], sl)
    fwd /= np.linalg.norm(fwd)
    other = 'Left' if side == 'Right' else 'Right'
    tg = {side: W1[:, IX[side + 'Hand'], :3, 3] - fwd * (HOLD_PULL_CM / 100.0 * k) - np.array([0.0, 0.0, 0.02 * k]),
          other: W0[:, IX[other + 'Hand'], :3, 3]}
    for s_, t_ in tg.items():
        HI.solve(fk, PAR, REST_LR, lt, lr, IX[s_ + 'Arm'], IX[s_ + 'ForeArm'], IX[s_ + 'Hand'], t_, weight=np.ones(1), end_mode='world')
    return lr[0].copy()


def hold_pose(G, spec_heavy, name_heavy):
    side_v, o_imp, T, ph = vanilla_impact(name_heavy)
    sq = Seq(spec_heavy['donors'])
    d_imp = donor_impact(sq, spec_heavy['side'])
    tau_h = max(0.0, d_imp - 1.35 * o_imp)
    lt, lr = resample(sq.lt, sq.lr, np.array([tau_h]))
    return dict(tau=tau_h, sq=sq, lr=lr[0], d_imp=d_imp)



IDLE_AMP = 0.4
HOLD_PULL = 0.6      # hold loop: donor-frame excursion behind the hold pose (the wind-up moves fast there: ~11 cm per donor frame)


def circ_smooth(lr, sig):
    """circular gaussian smoothing of a closed loop of quaternions (last frame == first), component-wise + renormalised."""
    n = len(lr) - 1
    q = qcont(lr[:-1])
    r = int(np.ceil(3 * sig))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sig) ** 2)
    k /= k.sum()
    out = np.zeros_like(q)
    for o, w in zip(range(-r, r + 1), k):
        sh = np.roll(q, -o, axis=0)
        sh = np.where(np.sum(sh * q, -1, keepdims=True) < 0, -sh, sh)
        out += w * sh
    out /= np.linalg.norm(out, axis=-1, keepdims=True)
    return np.concatenate([out, out[:1]])


def loop_close(lr):
    """distribute the last->first mismatch over the loop (last frame == first frame)."""
    T = len(lr)
    d = qmul(lr[:1], qinv(lr[-1:]))[0]
    w = np.arange(T) / (T - 1)
    off = qslerp(np.repeat(np.array([0, 0, 0, 1.0])[None, None], T, 0).repeat(lr.shape[1], 1), np.repeat(d[None], T, 0),
                 np.repeat(w[:, None], lr.shape[1], 1))
    out = qmul(off, lr)
    out[-1] = lr[0]
    return qcont(out)


def static_tracks(G, T, c):
    return {s: np.repeat(G.feet[s][None], T, 0) + c for s in G.feet}


def other_clip(G, name, spec, holds, lunge_start):
    kind = spec['kind']
    if kind in ('pose', 'safepose'):
        T = 2
    elif kind == 'guardhold':
        T = len(vanilla(name)[0]['W'])
    elif kind == 'idle_loop':
        T = spec['frames'] + 1
    else:
        T = vanilla_impact(name)[2]
    i = np.arange(T, dtype=float)
    c = np.zeros((T, 3))
    log, info = {}, dict(kind=kind)
    tracks = static_tracks(G, T, c)
    if kind in ('pose', 'guardhold'):
        lt, lr = np.repeat(G.lt, T, 0), np.repeat(G.lr, T, 0)
        info['source'] = 'guard pose G (idle_combat frame 0, fixed)' + (', held for the vanilla length' if kind == 'guardhold' else '')
        return lt, lr, c, info
    if kind == 'safepose':
        sq = Seq([('ovr_unequip', False)])
        lt, lr = resample(sq.lt, sq.lr, np.full(T, 0.45 * (sq.n - 1)))
        lr = blend_to(lr, G.lr[0], np.full(T, 0.55))
        hips = hips_path(G, sq, np.zeros(T), tracks, c, np.ones(T))
        lt, lr = finish(G, lt, lr, hips, tracks, c, log)
        info.update(source='ovr_unequip at 45 % blended 55 % towards G (lowered guard)', fixes=log)
        return lt, lr, c, info
    if kind == 'idle_loop':
        sq = Seq(spec['donors'])
        best = None
        for L in range(60, min(sq.n - 1, 108) + 1):
            e = float(np.mean(np.abs(np.sum(sq.lr[0, BODY_IX] * sq.lr[L, BODY_IX], -1))))
            if best is None or e > best[0]:
                best = (e, L)
        L = best[1]
        tau = np.linspace(0, L, T)
        lt, lr = resample(sq.lt, sq.lr, tau)
        lr = qslerp(np.repeat(sq.lr[:1], T, 0), lr, np.full(lr.shape[:2], IDLE_AMP))      # calm: IDLE_AMP x the NPC idle's deviation from its frame 0
        lr = circ_smooth(loop_close(qcont(lr)), 2.0)
        lr[:, IX['Root']] = [0, 0, 0, 1.0]
        lt[:, IX['Root']] = 0
        hips = hips_path(G, sq, tau, tracks, c, None, k_xy=0.35 * IDLE_AMP, k_z=0.6 * IDLE_AMP)
        hips = hips - np.linspace(0, 1, T)[:, None] * (hips[-1] - hips[0])[None]
        lt, lr = finish_loop(G, lt, lr, hips, tracks, c, log)
        info.update(source='idle_combat frames 0..%d resampled to %d, deviation from its frame 0 x %.2f (calm), loop closed + circular smoothing, feet planted (no stepping)' % (L, T, IDLE_AMP), donor_seam_dot=round(best[0], 4),
                    fixes=log)
        return lt, lr, c, info
    if kind == 'hold_start':           # guard -> charged hold pose (both synthesised from the guard; no donor wind-up)
        h = holds[spec['of']]
        w = sstep(i / (T - 1))
        st = start_state(G, spec['start'], name) if spec.get('start') else dict(lr=G.lr[0], hips=G.hips, lt=G.lt[0])
        lt = np.repeat(st['lt'][None], T, 0)
        lr = qslerp(np.repeat(st['lr'][None], T, 0), np.repeat(h['lr'][None], T, 0), np.repeat(w[:, None], G.lr.shape[1], 1))
        lr[-1] = h['lr']
        hips = st['hips'][None] + (G.hips - st['hips'])[None] * w[:, None]
        lt, lr = finish(G, lt, lr, hips, tracks, c, log)
        info.update(source='%s -> hold pose of %s (chest wound up %.0f deg, %s fist pulled back %.0f cm), smoothstep over %d frames' % (
            spec.get('start', 'guard'), spec['of'], HOLD_WIND[h['side']], h['side'], HOLD_PULL_CM, T), fixes=log)
        return lt, lr, c, info
    if kind == 'hold_loop':
        h = holds[spec['of']]
        u = 0.5 * (1.0 - np.cos(2 * np.pi * i / (T - 1)))
        lt = np.repeat(G.lt, T, 0)
        lr = qslerp(np.repeat(h['lr'][None], T, 0), np.repeat(h['lr2'][None], T, 0), np.repeat(u[:, None], G.lr.shape[1], 1))
        lr[0] = lr[-1] = h['lr']
        lt, lr = finish_loop(G, lt, lr, np.repeat(G.hips[None], T, 0), tracks, c, log)
        info.update(source='hold pose of %s with a cosine pulse to %.0f%% more wind-up (closed loop, at rest at the seam)' % (spec['of'], 100 * HOLD_PULSE),
                    fixes=log)
        return lt, lr, c, info
    sq = Seq(spec['donors'])
    if kind in ('block_start', 'block', 'block_end'):
        cover_f = 14.0
        if kind == 'block_start':
            tau = np.full(T, cover_f)
            w = sstep(i / (T * 0.6))
            lt, lr = build_body(G, sq, tau, None, None)
            lr = blend_to(lr, G.lr[0], 1 - w)
        elif kind == 'block':
            tau = np.full(T, cover_f)
            lt, lr = build_body(G, sq, tau, None, None)
        else:
            tau = cover_f + np.minimum(i, 8) * 0.5
            lt, lr = build_body(G, sq, tau, None, sstep((i - 2) / (T * 0.7)))
        hips = hips_path(G, sq, np.full(T, cover_f), tracks, c, None)
        lt, lr = finish(G, lt, lr, hips, tracks, c, log)
        info.update(source='idle_block frame %g (high cover) %s' % (cover_f, {'block_start': 'blended in from G over 60 %', 'block': 'held',
                                                                              'block_end': 'blended out to G'}[kind]), fixes=log)
        return lt, lr, c, info
    if kind in ('equip', 'unequip'):
        tau = np.linspace(0, sq.n - 1, T)
        if kind == 'equip':
            lt, lr = build_body(G, sq, tau, None, sstep((i - T * 0.55) / (T * 0.4)))
            start, end = G.idle_feet, G.feet
            fs = {'Left': (int(T * 0.15), int(T * 0.15) + 7), 'Right': (int(T * 0.4), int(T * 0.4) + 7)}
            off = (G.idle_hips - (G.idle_feet['Left'] + G.idle_feet['Right']) / 2) - (G.hips - G.mid)
            hw = (off, 1 - sstep(i / (T * 0.8)))
        else:
            lt, lr = build_body(G, sq, tau, 1 - sstep(i / (T * 0.35)), None)
            start, end = G.feet, G.idle_feet
            fs = {'Right': (int(T * 0.3), int(T * 0.3) + 5), 'Left': (int(T * 0.55), min(T - 1, int(T * 0.55) + 5))}
            off = (G.idle_hips - (G.idle_feet['Left'] + G.idle_feet['Right']) / 2) - (G.hips - G.mid)
            hw = (off, sstep(i / (T * 0.9)))
        tracks, steps = plan_feet(T, c, start, end, 'transfer', f_settle=fs)
        hips = hips_path(G, sq, tau, tracks, c, np.ones(T), start=hw)
        lt, lr = finish(G, lt, lr, hips, tracks, c, log)
        info.update(source='%s resampled %d -> %d frames (%.2fx), feet step between V idle stance and the guard stance' % (
            sq.parts[0]['clip'], sq.n, T, (sq.n - 1) / (T - 1)), steps=steps, fixes=log)
        return lt, lr, c, info
    raise ValueError(kind)


# ------------------------------------------------------------------ stage 2: follow-on clips (hit / deflect / parried / block reactions)
_BUILT = {}


def built_clip(clip):
    """lt / lr / W / info of a clip built earlier into this run's clips folder (start poses of the stage-2 clips)."""
    if clip not in _BUILT:
        base = os.path.join(CDIR, '%s__%s.%s' % (SET, clip, SUFFIX))
        z = np.load(base + '.npz')
        assert [str(x) for x in z['names152']] == list(P.names), 'joint order of %s' % base
        lt, lr = z['lt'].astype(float), qcont(z['lr'].astype(float))
        _BUILT[clip] = dict(lt=lt, lr=lr, W=fk(lt, lr), info=json.load(open(base + '.info.json')))
    return _BUILT[clip]


def branch_frame(name, clip):
    """frame of the vanilla <clip> whose pose (hands, head, upper arms vs Spine3) matches vanilla <name> frame 0 -> (frame, distance cm)."""
    a, b = MC.world_all(vanilla(clip)[0]), MC.world_all(vanilla(name)[0])
    J = [IX[n] for n in ('LeftHand', 'RightHand', 'Head', 'LeftArm', 'RightArm')]

    def rel(W, f):
        return W[f, J, :3, 3] - W[f, IX['Spine3'], :3, 3]
    d = [float(np.linalg.norm(rel(a, f) - rel(b, 0), axis=1).max()) for f in range(len(a))]
    f = int(np.argmin(d))
    return f, round(d[f] * 100, 2)


def start_state(G, start, name):
    """pose a stage-2 clip starts from: 'block' / 'safepose' (frame 0 of the built clip) or (clip, None) = the built clip at the vanilla branch
    frame.  Foot steps of that clip still in progress at the branch frame continue (feet_path, capsule frame)."""
    if start in ('block', 'safepose'):
        clip, f, dcm = {'block': 'block', 'safepose': 'melee_safepose'}[start], 0, None
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
                W=W[f].copy(), K=K)


def cont_tracks(T, st, end, f_settle):
    """feet: the start clip's step in progress continues, then one settle step each (lead first) to the end stance."""
    tracks, info = {}, {}
    for side in ('Left', 'Right'):
        pre = st['feet_path'][side][:T]
        K = len(pre) - 1
        p0, p1 = pre[-1], end[side]
        dist = float(np.linalg.norm((p1 - p0)[:2]))
        steps = []
        if dist > STEP_EPS:
            s0 = min(max(K + 1, f_settle + (0 if side == 'Left' else 5)), T - 3)
            steps.append((s0, min(T - 1, s0 + 7), p1, max(0.05, 0.015 + 0.3 * dist)))
        tr = foot_track(T, p0, steps)
        tr[:K + 1] = pre
        tracks[side] = tr
        info[side] = dict(continued_frames=K, steps=[dict(frames=[int(s[0]), int(s[1])], dist_cm=round(dist * 100, 1), lift_cm=round(s[3] * 100, 1))
                                                     for s in steps])
    return tracks, info


HIT_SLOWS, HIT_SLOW_FR = (0.5, 0.7, 1.0), 4.0   # hit clips: the first steps at 0.5x of the rest (hit-stop), smoothstep to full speed over 4
                                                # frames; a weaker hit-stop only if the clip fails a gate (e.g. a slowed foot lift-off reads as slide)


def hit_once(name, spec, slow):
    d, ph = vanilla(name)
    T = len(d['W'])
    f, dcm = branch_frame(name, spec['of'])
    b = built_clip(spec['of'])
    rem = len(b['lt']) - 1 - f
    t = np.arange(T - 1, dtype=float)
    v = slow + (1.0 - slow) * sstep((t - 1.0) / HIT_SLOW_FR)
    s = np.concatenate([[0.0], np.cumsum(v)])
    s *= rem / s[-1]
    fr = f + s
    lt, lr = resample(b['lt'], b['lr'], fr)
    info = dict(kind='hit', of=spec['of'], start=dict(clip=spec['of'], frame=f, match_vanilla_cm=dcm), built_frames=np.round(fr, 2).tolist(),
                hit_stop=slow, source='%s frames %d..%d re-timed to %d frames (%.2fx overall, first steps at %.0f %% of the later speed: hit-stop)' % (
                    spec['of'], f, len(b['lt']) - 1, T, rem / (T - 1), 100 * slow), capsule='static', vanilla_phases=ph)
    return lt, qcont(lr), np.zeros((T, 3)), info


def hit_clip(G, name, spec):
    """<attack>_hit: the built attack from the vanilla branch frame to its last frame (the guard), re-timed to the vanilla length; the
    strongest hit-stop of HIT_SLOWS that passes every gate, else the try with the most passing frames."""
    tries, res = [], []
    for slow in HIT_SLOWS:
        lt, lr, c, info = hit_once(name, spec, slow)
        g = gates(fk(lt, lr), c)
        tries.append(dict(hit_stop=slow, PASS=g['PASS'], base_gate_pass=g['base_gate_pass'], fail=g['base_gate_fail'], slide_cm=max(g['slide_max_cm'].values())))
        res.append((g['PASS'], g['base_gate_pass'], (lt, lr, c, info)))
        if g['PASS']:
            break
    k = max(range(len(res)), key=lambda j: res[j][:2])
    lt, lr, c, info = res[k][2]
    info['hit_stop_tries'] = tries
    return lt, lr, c, info


REACT_IN = 8.0           # react clips: frames over which the start offset (start pose vs the donor's frame 0) fades out
REACT_RAMP = 6.0         # after the fixes: the rest of the difference to the start pose (upper body + hips) faded in over these frames
REACT_TEMPO = 1.0        # donor frames per output frame (the NPC's own reaction speed); the guard blend ends the clip
REACT_AMPS = (1.0, 0.85, 0.7, 0.55, 0.4)   # donor deviation from its frame 0: the largest that passes every gate on every frame


def legs_again(G, lt, lr, tracks, c, log):
    """leg IK once more after a late change of the hips / upper body (same foot pivot rule as finish())."""
    lr[:, LEGS] = G.lr[0, LEGS]
    dpsi = FOOT_PIVOT * (np.unwrap(hips_yaw(fk(lt, lr))) - hips_yaw(G.W[None])[0])
    dpsi = (dpsi + np.pi) % (2 * np.pi) - np.pi
    Rz = R.from_rotvec(np.stack([np.zeros_like(dpsi), np.zeros_like(dpsi), dpsi], 1)).as_matrix()
    foot_R = {s: Rz @ G.foot_R[s][None] for s in G.feet}
    log['leg_ik_residual2_cm'] = {k: round(v * 100, 2) for k, v in solve_legs(lt, lr, tracks, c, foot_R, G.toe).items()}


def react_once(G, name, spec, amp):
    d, ph = vanilla(name)
    T = len(d['W'])
    st = start_state(G, spec['start'], name)
    sq = Seq(spec['donors'])
    i = np.arange(T, dtype=float)
    tempo = spec.get('tempo', REACT_TEMPO)
    span = min(sq.n - 1.0, (T - 1) * tempo)
    tau = np.linspace(0, span, T)
    src = types.SimpleNamespace(lt=sq.lt, lr=qslerp(np.repeat(sq.lr[:1], sq.n, 0), sq.lr, np.full(sq.lr.shape[:2], amp)) if amp < 1 else sq.lr)
    # 'layer': the donor's change from its frame 0 is laid on the start pose for the whole clip (keeps the start pose's arm twist; blocks),
    # else the offset to the start pose fades out over REACT_IN frames (the donor's own pose takes over: deflect / parried)
    w_s = np.ones(T) if spec.get('layer') else 1.0 - sstep(i / REACT_IN)
    e0, e1 = int(round(T * 0.45)), T - 4
    w_e = sstep((i - e0) / max(e1 - e0, 1))
    lt, lr = build_body(G, src, tau, w_s, w_e, start_lr=st['lr'])
    c = np.zeros((T, 3))
    tracks, steps = cont_tracks(T, st, G.feet, f_settle=max(e0, st['K'] + 2))
    hips = hips_path(G, sq, tau, tracks, c, w_e, k_xy=SHIFT_K * amp, k_z=DZ_K * amp)
    hips = hips + (st['hips'] - hips[0])[None] * (1.0 - sstep(i / 8.0))[:, None]
    log = {}
    lt, lr = finish(G, lt, lr, hips, tracks, c, log)
    J = UPPER + [IX['Hips']]
    lr[:, J] = offset_start(lr[:, J], st['lr'][J], 1.0 - sstep(i / REACT_RAMP))
    lr = qcont(lr)
    legs_again(G, lt, lr, tracks, c, log)
    for sd in ('Left', 'Right'):
        log['ramp_wrist_twist_moved_deg_' + sd[0]] = round(wrist_to_forearm(lt, lr, sd), 1)
    info = dict(kind='react', note=spec.get('note'), donors=sq.parts, start=dict(clip=st['clip'], frame=st['frame'], match_vanilla_cm=st['match_cm']),
                amp=amp, tempo=tempo, donor_span=[0.0, round(span, 2)], tau=np.round(tau, 2).tolist(), guard_blend=[e0, e1], capsule='static',
                feet_mode='continue + settle', steps=steps, fixes=log, vanilla_phases=ph)
    return lt, lr, c, info


def react_clip(G, name, spec):
    """start pose (built clip frame) -> NPC reaction at its own speed (deviation from the donor's frame 0 x amp) -> guard; per donor
    alternative (spec 'alts') the largest amp of REACT_AMPS whose clip passes every gate on every frame; the passing try with the largest
    amp over the alternatives (ties: the first alternative), else the try with the most passing frames."""
    tries, res = [], []
    for alt in spec.get('alts') or [spec['donors']]:
        sp = dict(spec, donors=alt)
        for amp in REACT_AMPS:
            lt, lr, c, info = react_once(G, name, sp, amp)
            g = gates(fk(lt, lr), c)
            tries.append(dict(donors=['%s%s' % (d[0], '(m)' if d[1] else '') for d in alt], amp=amp, PASS=g['PASS'], base_gate_pass=g['base_gate_pass'],
                              frames=g['frames'], fail=g['base_gate_fail'], spike_cm=g['spike_max_cm'], wrist_m_s=g['wrist_max_m_s']))
            res.append((g['PASS'], amp if g['PASS'] else 0.0, g['base_gate_pass'], -g['spike_max_cm'], -len(res), (lt, lr, c, info)))
            if g['PASS']:
                break
    k = max(range(len(res)), key=lambda j: res[j][:5])
    lt, lr, c, info = res[k][5]
    info['amp_tries'] = tries
    return lt, lr, c, info


def start_continuity(name, spec, W):
    """frame 0 vs the start pose: max body-joint distance (cm) and rotation (deg)."""
    s = spec.get('start') or ((spec['of'], None) if spec['kind'] == 'hit' else None)
    if not s:
        return None
    if s in ('block', 'safepose'):
        Ws = built_clip({'block': 'block', 'safepose': 'melee_safepose'}[s])['W'][0]
    else:
        Ws = built_clip(s[0])['W'][branch_frame(name, s[0])[0]]
    return dict(pos_cm=round(float(np.linalg.norm(W[0, BODY_IX, :3, 3] - Ws[BODY_IX, :3, 3], axis=1).max() * 100), 3),
                rot_deg=round(float(MC.rot_delta_deg(W[0, BODY_IX, :3, :3], Ws[BODY_IX, :3, :3]).max()), 3))


# ------------------------------------------------------------------ gates
def spikes_cm(W):
    Pp = W[:, BODY_IX][:, :, :3, 3]
    if len(Pp) < 3:
        return 0.0
    v = np.linalg.norm(np.diff(Pp, axis=0), axis=-1)          # [T-1, J]
    sp = v[1:-1] - 2 * np.maximum(v[:-2], v[2:])      # pop: one step more than twice both neighbours (a punch's speed peak is not)
    edge = np.concatenate([v[0] - 2 * v[1], v[-1] - 2 * v[-2]])     # first / last step against their only neighbour
    return float(max(sp.max() if len(sp) else 0.0, edge.max(), 0) * 100)


def fist_centers(W):
    out = {}
    for s in ('Left', 'Right'):
        h = W[:, IX[s + 'Hand'], :3, 3]
        m = W[:, IX[s + 'HandMiddle1'], :3, 3]
        out[s] = h + 0.9 * (m - h)
    return out


def head_center(W):
    eyes = (W[:, IX['LeftEye'], :3, 3] + W[:, IX['RightEye'], :3, 3]) / 2
    hd = W[:, IX['Head'], :3, 3]
    return eyes + 0.55 * (hd - eyes) + np.array([0, 0, 0.02])


def gates(W, c, loop=False):
    import redo_pen as RPN
    T = len(W)
    g = GB.frame_gate(W[:, :73], RG, steps=True, al=AL)
    sm = GB.summary(g)
    cnt, pen = RPN.pen_frames(W[:, :73])
    fc = fist_centers(W)
    hc = head_center(W)
    head_cl = min(float((np.linalg.norm(fc[s] - hc, axis=1) - HEAD_R - FIST_R).min()) for s in fc) * 100
    ff = float((np.linalg.norm(fc['Left'] - fc['Right'], axis=1) - 2 * FIST_R).min()) * 100
    Wt_world = W[:, :, :3, 3] + c[:, None]
    sole = NR.sole_heights(P, Wt_world)
    con = CT.detect_contacts(sole)
    slides = {}
    for k, s in enumerate(('Left', 'Right')):
        segs = CT.slide_segments(con[:, k], Wt_world[:, IX[s + 'Foot'], :2], min_len=3)
        slides[s] = round(max([x['drift_cm'] for x in segs] or [0.0]), 2)
    floor = float(sole.min() * 100)
    jump = spikes_cm(W)
    out = dict(frames=T, base_gate_pass=int(sm['pass_frames']), base_gate_fail=sm['fail_features'], margin_min=round(sm['margin_min'], 3),
               pen_proxy_max=int(cnt.max()), pen_frames=int(pen.sum()), pen_thr=int(RPN.proxy().thr), head_clear_min_cm=round(head_cl, 1),
               fist_fist_min_cm=round(ff, 1), slide_max_cm=slides, sole_min_cm=round(floor, 2), spike_max_cm=round(jump, 2),
               root_identity=bool(np.abs(W[:, IX['Root']] - np.eye(4)).max() < 1e-6), wrist_max_m_s=round(wrist_speed(W), 1) if T > 1 else 0.0)
    if loop:        # last frame == first frame; the wrap step (T-2 -> 0) may not exceed its neighbouring steps by more than LOOP_CM / LOOP_DEG
        def step(a, b):
            return (float(np.linalg.norm(W[a, BODY_IX, :3, 3] - W[b, BODY_IX, :3, 3], axis=1).max() * 100),
                    float(MC.rot_delta_deg(W[a, BODY_IX, :3, :3], W[b, BODY_IX, :3, :3]).max()))
        wp, wr = step(T - 2, 0)
        n1, n2 = step(T - 3, T - 2), step(0, 1)
        out['loop_first_last_cm'] = round(step(0, T - 1)[0], 4)
        out['loop_gap_cm'] = round(max(0.0, wp - max(n1[0], n2[0])), 3)
        out['loop_gap_deg'] = round(max(0.0, wr - max(n1[1], n2[1])), 3)
    ok = (out['base_gate_pass'] == T and out['pen_frames'] == 0 and head_cl >= 0 and ff >= 0 and max(slides.values()) <= SLIDE_MAX_CM
          and floor >= FLOOR_CM and jump <= JUMP_CM and out['root_identity'])
    if loop:
        ok = ok and out['loop_gap_cm'] <= LOOP_CM and out['loop_gap_deg'] <= LOOP_DEG
    out['PASS'] = bool(ok)
    out['_ok_frames'] = g['ok']
    out['_worst'] = [str(x) for x in g['worst']]
    return out


def built_impact(W, side, ph):
    a, b = ph.get('active', (0, len(W) - 1))
    y = fist_forward(W, side)
    ya = W[:, IX[side + 'Hand'], 1, 3]
    return [a + int(np.argmax(y[a:b + 1])), a + int(np.argmax(ya[a:b + 1]))]


def main(a):
    global CDIR
    t0 = time.time()
    out_dir = a[a.index('--out') + 1] if '--out' in a else OUT
    cdir = os.path.join(out_dir, 'clips')
    os.makedirs(cdir, exist_ok=True)
    CDIR = cdir
    only = a[a.index('--only') + 1].split(',') if '--only' in a else None
    lunge_start = float(a[a.index('--lunge-start') + 1]) if '--lunge-start' in a else 0.0
    if '--spec' in a:          # polish: per-clip SPEC overrides from a json file {clip: {key: value}} (donors as [clip, mirror(, set)])
        for k, v in json.load(open(a[a.index('--spec') + 1])).items():
            v = dict(v)
            for dk in ('donors',):
                if dk in v:
                    v[dk] = [tuple(x) for x in v[dk]]
            if 'speeds' in v:
                v['speeds'] = tuple(v['speeds'])
            if 'hips_seam_tries' in v:
                v['hips_seam_tries'] = tuple(v['hips_seam_tries'])
            for dk in [x for x, y in v.items() if y is None]:
                SPEC[k].pop(dk, None)
                v.pop(dk)
            SPEC[k].update(v)
            print('SPEC override', k, SPEC[k], flush=True)
    G = Guard()
    print('GUARD fixes', G.log, 'idle->guard feet (cm):', {s: np.round((G.feet[s] - G.idle_feet[s]) * 100, 1).tolist() for s in G.feet}, flush=True)
    for h in ('heavy_attack_01', 'heavy_attack_02'):       # charged hold poses (heavy frame 0 == hold_loop pose == hold_start last frame)
        sd = SPEC[h]['side']
        HOLDS[h] = dict(lr=hold_synth(G, sd, 1.0), lr2=hold_synth(G, sd, 1.0 + HOLD_PULSE), side=sd)
    holds = HOLDS
    rep_path = os.path.join(out_dir, 'build_report.json')
    rep = json.load(open(rep_path)) if os.path.exists(rep_path) and only else {}
    for name in ORDER:
        if only and name not in only:
            continue
        spec = SPEC[name]
        t1 = time.time()
        if spec['kind'] in ('attack', 'heavy'):
            lt, lr, c, info = attack_clip(G, name, spec, lunge_start)
        elif spec['kind'] == 'hit':
            lt, lr, c, info = hit_clip(G, name, spec)
        elif spec['kind'] == 'react':
            lt, lr, c, info = react_clip(G, name, spec)
        else:
            lt, lr, c, info = other_clip(G, name, spec, holds, lunge_start)
        if spec.get('uncross'):      # final2: crossed hands -> the guard hand moved aside (same donor / amp / timing; no-op without a crossing)
            info['uncross'] = uncross(lt, lr, spec['uncross'])
        lr = qcont(lr)
        lt_g, lr_g = lt, lr
        meta = dict(set=SET, clip=name, rig=RIG, animation_type='Normal', additive=False, root_motion_type='None', frames=len(lt),
                    duration=(len(lt) - 1) / 30.0, events=[], source='npc_build.py ' + SUFFIX)
        d = NR.make_dump(P, lt_g, lr_g, meta)
        W = fk(lt_g, lr_g)
        g = gates(W, c, loop=name in ('hold_loop_01', 'hold_loop_02', 'guard_idle_loop'))
        if spec['kind'] in ('attack', 'heavy'):
            info['impact_built'] = built_impact(W, spec['side'], info['vanilla_phases'])
        if spec['kind'] in ('attack', 'heavy'):
            for sd in ('Left', 'Right'):
                pass
        if name != 'guard_idle_loop':
            dv, _ = vanilla(name)
            assert len(dv['W']) == len(lt), (name, len(dv['W']), len(lt))
        info['capsule_path_cm'] = np.round(c[:, 1] * 100, 2).tolist()
        sc = start_continuity(name, spec, W)
        if sc:
            info['start_continuity'] = sc
        info['gates'] = {k: v for k, v in g.items() if not k.startswith('_')}
        info['hands'] = hands_cross(W)          # crossed-hands record (every clip); the rule: no frame with the left hand right of the right hand
        if info['hands']['crossed_frames'] or info['hands']['fists_crossed_frames']:
            print('CROSSED %s hands %s fists %s (wrist gap min %.1f cm, fist gap min %.1f cm)' % (
                name, info['hands']['crossed_frames'], info['hands']['fists_crossed_frames'], info['hands']['wrist_gap_min_cm'],
                info['hands']['fist_gap_min_cm']), flush=True)
        _BUILT.pop(name, None)
        info['failing_frames'] = [dict(frame=int(t), worst=g['_worst'][t]) for t in np.where(~g['_ok_frames'])[0]][:40]
        base = os.path.join(cdir, '%s__%s.%s' % (SET, name, SUFFIX))
        NR.save_dump(base + '.npz', d)
        MC.jdump(info, base + '.info.json')
        rep[name] = dict(PASS=g['PASS'], **{k: v for k, v in info['gates'].items() if k != 'PASS'}, donors=[p['clip'] for p in info.get('donors', [])],
                         impact=[info.get('impact_vanilla'), info.get('impact_built')], fixes=info.get('fixes'), seconds=round(time.time() - t1, 1),
                         hands=info['hands'], **({'uncross': info['uncross']} if 'uncross' in info else {}))
        print('%-16s %-4s base %3d/%3d fail=%s pen=%d head=%.1f ff=%.1f slide=%s sole=%.1f spike=%.1f %s %.1fs' % (
            name, 'PASS' if g['PASS'] else 'FAIL', g['base_gate_pass'], g['frames'], g['base_gate_fail'], g['pen_frames'], g['head_clear_min_cm'],
            g['fist_fist_min_cm'], g['slide_max_cm'], g['sole_min_cm'], g['spike_max_cm'],
            ('loop %.2fcm/%.2fdeg' % (g['loop_gap_cm'], g['loop_gap_deg'])) if 'loop_gap_cm' in g else '', time.time() - t1), flush=True)
    MC.jdump(rep, rep_path)
    print('BUILD done %.1f s -> %s' % (time.time() - t0, cdir))


if __name__ == '__main__':
    main(sys.argv[1:])
