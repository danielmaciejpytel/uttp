"""Dual-wield TPP pilot clips of Female V from the game's NPC dual-pistol animations (ma_gang_handgundual_action_combat), 2026-11-09.

  python -I dw_build.py [--only clip,clip]
Output: Build\\work\\dualwield-npc-20261109\\clips\\pwa_dual__<clip>.dw1.npz (dump layout of melee_dump.py: W [T,73,4,4] game axes, Root = identity,
        W_extra, lt / lr [T,152] ...) + .dw1.info.json (donor, time map, gates); clips\\build_report.json.
Method (game axes, 30 fps; frame counts = the vanilla Female V Lexington dual-wield clips where a vanilla clip exists):
  1 donor: npc_retarget.Donor (NPC LOCAL rotations by name on V's 152-joint rig, V's bone lengths, Root = identity). Copy-locals test
    (dw_research.py rig): world joint rotations exact (0.000 deg), bone directions <= 0.86 deg, V's arm length == man_average's (56.57 cm);
  2 layering as planned for the game (melee precedent: MM owns Hips / Spine / legs): Root, Hips, Spine and the legs are ONE planted lower
    body for every clip (the NPC aim pose aim_ovr_idle f0, hips raised 6.04 cm: man_average's legs are shorter); Spine1..Head, clavicles,
    arms, hands come from the NPC per frame; the upper body is yawed (Spine1/2/3 = 1/3, 2/3, 1 of -11 deg) and both hands get a constant
    wrist correction (~13 deg) so that both barrels of A0 are parallel, level, along the hips forward (square_align);
  3 grip: the weapon bone locals are REBASED on the vanilla player grip: L(t) = L_van_aim * L_npc_aim^-1 * L_npc(t) (vanilla gun-in-hand at the
    aim, the NPC's in-hand motion kept); fingers = the vanilla grip (idlepose_shoulder f0, static, matches that gun position);
  4 the joints the NPC rig lacks (arms_* spaces, Torso_* chains, fpp camera ...) = the vanilla clip's locals (same frame), then the hand IK
    helpers are re-anchored on EVERY frame on the new hands / guns with the vanilla per-frame offset (arm_<s>_ik, Torso_<S>Arm_Hand_IK_JNT,
    Torso_World_<S>Arm_Hand_IK_JNT on the hand; arm_<s>_ik_offset keeps its vanilla local; arm_<s>_ik_weapon on Weapon<S>);
  5 anatomy fixes only where V's base gate fails (fix_anatomy: melee functions + swivel / clavicle pass / hand-gun push of this file), then the
    ends are pinned to the fixed A0 (ramp RAMP frames);
  6 clips: idlepose_shoulder (7 fr, static A0 like vanilla), aim_idle_loop (extra, NPC breathing loop 81 fr), fire_right / fire_left (extra
    13 fr: A0 + the NPC recoil additive aim_recoil_add on ONE arm chain, half of its torso part, its one-frame kick eased in over 3 frames;
    the game's per-hand split is a graph mask), reload (70 fr, NPC ovr_reload, hands-closest frame -> vanilla mag_in f27, f0 / fN = A0),
    equip (21 fr, NPC ovr_equip from the first frame after the cross-draw whose guns stay >= 1 cm apart (f20) to f28, fN = A0);
  7 gates on every frame: anatomy_gate_base (V base body, steps), redo_pen (arm vs torso), feet (slide / sole), single-frame spikes incl. the
    first and last step, hand-vs-head clearance, gun vs body (box proxies, holding hand excluded, holding forearm excluded from the rear
    slide + grip boxes), gun vs gun, gun vs head, helpers vs hands / guns (<= 1.5 cm on every frame incl. f0 / fN), hand-on-gun (palm in the
    weapon frame vs the vanilla grip).
Gun proxy (assumption, no weapon mesh read): per gun three boxes in the weapon-bone frame (barrel +Y, up +Z, palm at z ~ -4 cm): slide
x +-1.4 / y -5..16 / z -1.5..2.0 cm (split at y -1.5: front / rear), grip x +-1.3 / y -5.5..-1.5 / z -9..-1.5 cm.
"""
import json
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dw_common as DC  # noqa: E402  (sets npc_retarget.NPC_DIR to the dual-wield extraction)
import npc_build as NB  # noqa: E402  (melee tools, imported unchanged: gates, fk, slerp, resample)
import hinge_ik as HI  # noqa: E402

MC, NR = DC.MC, DC.NR
DC._P['woman'] = NB.P
P, IX, PAR = NB.P, NB.IX, NB.PAR
NPC = DC.NPC_DUAL['man_average']
SET = 'pwa_dual'
SUFFIX = 'dw1'
CLIPS = os.path.join(DC.OUT, 'clips')
LOWER = ['Root', 'Hips', 'Spine'] + MC.L_LEG + MC.R_LEG
FINGER = [n for n in P.names if 'Hand' in n and any(k in n for k in NR.FINGER_KEYS)]
WPN = {s: 'Weapon' + s for s in DC.SIDES}
ARM = {s: [s + 'Shoulder', s + 'Arm', s + 'ForeArm', s + 'Hand'] for s in DC.SIDES}
TORSO_UP = ['Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head']
ANCHOR_HAND = ['arm_%s_ik', 'Torso_%sArm_Hand_IK_JNT', 'Torso_World_%sArm_Hand_IK_JNT']
HELPER_LIMIT_CM = 1.5          # = HAND_LIMIT_CM of the melee exporter (helper vs bone, every frame)
RAMP = 6                       # frames over which the start / end offset to A0 fades (reload / equip)
FIRE_FRAMES = 13
FIRE_TORSO = 0.5               # share of the additive's torso (Spine1..Head) part for one gun
FIRE_EASE = [0.0, 0.3, 0.65]   # additive time of the first output frames (then frame 1 .. n-1 linearly)
EQUIP_EXP = 1.6                # equip time warp: tau = start + (n-1-start) * u^EXP (eases in from rest, ~x0.7 native speed at the end)
PUSH_MARGIN = 0.005            # m, hand / gun vs the other gun (separation push)
GUN_BOXES = [((-0.014, -0.015, -0.015), (0.014, 0.16, 0.02)),     # slide / barrel in front of the grip (weapon frame, m)
             ((-0.014, -0.05, -0.015), (0.014, -0.015, 0.02)),     # slide behind the grip front (over the hand web)
             ((-0.013, -0.055, -0.09), (0.013, -0.015, -0.015))]   # grip
OWN_FORE_FREE = (1, 2)         # boxes the HOLDING forearm is not tested against (its proxy wrist end lies in the hand-web / grip zone)
I4 = np.eye(4)
SQUARE = True                  # upper body yawed so that the mean barrel of the aim pose points along the hips forward (MM hips in game)
ALIGN_BARRELS = True           # constant per-hand wrist correction: both barrels of the aim pose parallel, horizontal, along the hips forward


# ------------------------------------------------------------------ small helpers
def mat(lt, lr):
    return MC.mat4(lt, lr)


def to_tq(M):
    return M[..., :3, 3].copy(), R.from_matrix(M[..., :3, :3].reshape(-1, 3, 3)).as_quat().reshape(M.shape[:-2] + (4,))


def qmul(a, b):
    return NB.qmul(np.asarray(a, float), np.asarray(b, float))


def qinv(a):
    return NB.qinv(a)


def ramp(T, n, at_end=False):
    """weight [T]: 1 at the pinned end, smoothstep to 0 over n frames."""
    x = np.arange(T, dtype=float)
    w = 1 - NB.sstep(x / max(n, 1)) if not at_end else 1 - NB.sstep((T - 1 - x) / max(n, 1))
    return w


_VAN = {}


def vanilla(clip):
    if clip not in _VAN:
        _VAN[clip] = DC.player_clip(clip)
    return _VAN[clip]


_DON = {}


def donor(clip):
    """(Donor on V's rig, NPC weapon locals {side: [T,4,4]} on the NPC skeleton)."""
    if clip not in _DON:
        dn = NR.Donor(P, clip, NPC)
        raw = DC.npc_raw(NPC, clip)
        wl = {s: mat(raw['lt'][:, raw['ix'][WPN[s]]], raw['lr'][:, raw['ix'][WPN[s]]]) for s in DC.SIDES}
        _DON[clip] = (dn, wl, raw)
    return _DON[clip]


def resample_mats(M, fr):
    lt, lr = to_tq(M)
    lt2, lr2 = NB.resample(lt[:, None], lr[:, None], fr)
    return mat(lt2[:, 0], lr2[:, 0])


# ------------------------------------------------------------------ assembly
NPC_JOINTS = None
A0 = {}


def shared_upper():
    nsk = donor('aim_ovr_idle')[0]
    names = set(NR.skeleton(os.path.join(DC.NPC_GLB, NPC + '.anims.glb')).names)
    return [IX[n] for n in P.names if n in names and n not in LOWER and n not in FINGER and n not in WPN.values()]


def aim_pose():
    """A0: lower body, NPC upper body, vanilla fingers / grip of the aim pose (single frame locals [152])."""
    if A0:
        return A0
    dn, wl, raw = donor('aim_ovr_idle')
    van = vanilla('idlepose_shoulder')
    A0['lower'] = {IX[n]: (dn.lt[0, IX[n]].copy(), dn.lr[0, IX[n]].copy()) for n in LOWER}
    A0['lower'][IX['Root']] = (np.zeros(3), np.array([0, 0, 0, 1.0]))
    # man_average hips rest 6.7 cm lower and legs 3.2 cm shorter than V's: raise the planted hips until the lower sole is on the ground
    sole = NR.sole_heights(P, dn.W[:1, :, :3, 3])
    A0['hips_raise_cm'] = round(float(-sole.min() * 100), 2)
    t_, q_ = A0['lower'][IX['Hips']]
    A0['lower'][IX['Hips']] = (t_ + np.array([0, 0, -sole.min()]), q_)
    A0['fing'] = {IX[n]: (van['lt'][0, IX[n]].copy(), van['lr'][0, IX[n]].copy()) for n in FINGER}
    A0['wvan'] = {s: mat(van['lt'][0, IX[WPN[s]]], van['lr'][0, IX[WPN[s]]]) for s in DC.SIDES}
    A0['wnpc'] = {s: wl[s][0].copy() for s in DC.SIDES}
    A0['upper'] = shared_upper()
    return A0


def assemble(up_lt, up_lr, wnpc, van_lt, van_lr):
    """up_lt / up_lr [T,152] = donor locals on V's rig (time-mapped), wnpc {side: [T,4,4]} NPC weapon locals (time-mapped),
    van_lt / van_lr [T,152] vanilla locals for the joints the NPC lacks -> lt, lr [T,152] (helpers not yet re-anchored)."""
    A = aim_pose()
    T = len(up_lt)
    lt, lr = van_lt.copy(), van_lr.copy()
    U = A['upper']
    lt[:, U], lr[:, U] = up_lt[:, U], up_lr[:, U]
    for j, (t_, q_) in list(A['lower'].items()) + list(A['fing'].items()):
        lt[:, j], lr[:, j] = t_, q_
    for s in DC.SIDES:
        L = A['wvan'][s] @ np.linalg.inv(A['wnpc'][s]) @ wnpc[s] if wnpc is not None else np.repeat(A['wvan'][s][None], T, 0)
        lt[:, IX[WPN[s]]], lr[:, IX[WPN[s]]] = to_tq(L)
    lr = NR.S.qcontinuous(lr)
    if SQUARE or ALIGN_BARRELS:
        square_align(lt, lr)
    return lt, lr


def rz(deg):
    return R.from_rotvec([0, 0, np.radians(deg)]).as_matrix()


def square_align(lt, lr):
    """constant offsets (computed once on the aim pose, applied to every frame): (1) Spine1 / Spine2 / Spine3 world yaw + 1/3, 2/3, 1 x theta
    so that the mean barrel of the aim pose points along the hips forward; (2) per hand a constant LOCAL post-rotation C_s that turns that
    barrel onto the hips forward, horizontal (the vanilla grip turns the guns ~10.5 deg against the NPC grip; the NPC guns themselves
    differ by 8 deg in yaw and 9 deg in pitch)."""
    A = aim_pose()
    if 'theta' not in A:
        # the aim pose itself (frame 0 of the aim donor, assembled without the offsets)
        A['theta'], A['C'] = 0.0, {s: np.array([0, 0, 0, 1.0]) for s in DC.SIDES}
        base_lt, base_lr = assemble_raw_aim()
        W = NB.fk(base_lt, base_lr)
        H, _ = DC.body_frame(W, IX)
        yaws = []
        for s in DC.SIDES:
            b = H[0].T @ W[0, IX[WPN[s]], :3, 1]
            yaws.append(np.degrees(np.arctan2(b[0], b[1])))
        A['theta'] = float(np.mean(yaws)) if SQUARE else 0.0      # yaw > 0 = barrel right of the hips forward; Rz(+theta) brings it back
        A['barrel_yaw_before'] = [round(float(y), 1) for y in yaws]
        apply_square(base_lt, base_lr, A['theta'])
        W = NB.fk(base_lt, base_lr)
        H, _ = DC.body_frame(W, IX)
        for s in DC.SIDES:
            if not ALIGN_BARRELS:
                continue
            b = W[0, IX[WPN[s]], :3, 1]
            d = H[0][:, 1]
            ax = np.cross(b, d)
            ang = np.arcsin(np.clip(np.linalg.norm(ax), -1, 1))
            Q = R.from_rotvec(ax / max(np.linalg.norm(ax), 1e-9) * ang).as_matrix() if ang > 1e-6 else np.eye(3)
            Rh = W[0, IX[s + 'Hand'], :3, :3]
            A['C'][s] = R.from_matrix(Rh.T @ Q @ Rh).as_quat()
            A.setdefault('wrist_corr_deg', {})[s] = round(float(np.degrees(ang)), 1)
    apply_square(lt, lr, A['theta'])
    for s in DC.SIDES:
        j = IX[s + 'Hand']
        lr[:, j] = qmul(lr[:, j], np.repeat(A['C'][s][None], len(lr), 0))


def apply_square(lt, lr, deg):
    if abs(deg) < 1e-9:
        return
    W0 = NB.fk(lt, lr)
    for nm, f in (('Spine1', 1 / 3.0), ('Spine2', 2 / 3.0), ('Spine3', 1.0)):
        NB.set_world_rot(lt, lr, IX[nm], np.einsum('ij,tjk->tik', rz(f * deg), W0[:, IX[nm], :3, :3]))


def assemble_raw_aim():
    """aim donor frame 0 assembled (lower body, grip, fingers, vanilla extras) WITHOUT the square / align offsets."""
    global SQUARE, ALIGN_BARRELS
    sv = (SQUARE, ALIGN_BARRELS)
    SQUARE = ALIGN_BARRELS = False
    try:
        dn, wl, raw = donor('aim_ovr_idle')
        van = vanilla('idlepose_shoulder')
        return assemble(dn.lt[:1], dn.lr[:1], {s: wl[s][:1] for s in DC.SIDES}, van['lt'][:1], van['lr'][:1])
    finally:
        SQUARE, ALIGN_BARRELS = sv


def pin(lt, lr, A_lt, A_lr, w):
    """offset blend towards the target pose A (single frame [152]) on the moving joints: lr' = slerp(lr, lr * lr_end^-1 * A, w);
    w [T] (1 = exactly A at the pinned frame).  end = the pinned frame (first or last, where w == 1)."""
    T = len(lr)
    k = int(np.argmax(w))
    d = qmul(qinv(lr[k]), A_lr[None].repeat(1, 0)[0])            # [152,4] offset at the pinned frame (local right-multiply)
    tgt = qmul(lr, np.repeat(d[None], T, 0))
    lr2 = NB.qslerp(lr, tgt, np.repeat(w[:, None], lr.shape[1], 1))
    lt2 = lt + w[:, None, None] * (A_lt[None] - lt[k][None])
    return lt2, NR.S.qcontinuous(lr2)


def reanchor(lt, lr, van_lt, van_lr):
    """hand IK helpers on the new hands / guns on every frame with the vanilla per-frame offset (world -> local under the current parents)."""
    Wv = NB.fk(van_lt, van_lr)
    for s in DC.SIDES:
        sl = s.lower()
        hand, wpn = IX[s + 'Hand'], IX[WPN[s]]
        todo = [(h % (sl if h.startswith('arm') else s), hand) for h in ANCHOR_HAND] + [('arm_%s_ik_weapon' % sl, wpn)]
        for name, anc in todo:
            j = IX[name]
            W = NB.fk(lt, lr)
            rel = np.linalg.inv(Wv[:, anc]) @ Wv[:, j]
            tgt = W[:, anc] @ rel
            L = np.linalg.inv(W[:, PAR[j]]) @ tgt
            lt[:, j], lr[:, j] = to_tq(L)
    return lt, NR.S.qcontinuous(lr)


# ------------------------------------------------------------------ clips
def clip_idle():
    van = vanilla('idlepose_shoulder')
    T = len(van['lt'])
    a_lt, a_lr = A_frame()
    return (np.repeat(a_lt[None], T, 0), np.repeat(a_lr[None], T, 0), van,
            dict(donor='%s:aim_ovr_idle f0 (static A0, like the vanilla 7-frame pose)' % NPC, time_map='f0 x %d' % T, fixed=True))


def clip_loop():
    dn, wl, raw = donor('aim_ovr_idle')
    van = vanilla('idlepose_shoulder')
    T = dn.n
    vl, vr = np.repeat(van['lt'][:1], T, 0), np.repeat(van['lr'][:1], T, 0)
    lt, lr = assemble(dn.lt, dn.lr, wl, vl, vr)
    return lt, lr, dict(lt=vl, lr=vr), dict(donor='%s:aim_ovr_idle (all %d frames, NPC loop)' % (NPC, T), time_map='1:1', loop=True, pins=('start', 'end'))


def clip_fire(side):
    """A0 + the NPC recoil additive (aim_recoil_add, frame 0 == rest) on one arm chain + FIRE_TORSO of its torso part."""
    dn0, wl0, _ = donor('aim_ovr_idle')
    add = NR.Donor(P, 'aim_recoil_add', NPC)
    van = vanilla('idlepose_shoulder')
    T = FIRE_FRAMES
    lt0, lr0 = np.repeat(dn0.lt[:1], T, 0), np.repeat(dn0.lr[:1], T, 0)
    n = T
    # the NPC kick is a one-frame step (~9.5 cm hand); eased in over 3 frames (additive at fractional frames), the recovery x1.1 faster
    tau = np.r_[FIRE_EASE, np.linspace(1.0, add.n - 1, T - len(FIRE_EASE))]
    _, alr = NB.resample(add.lt, add.lr, tau)
    d = qmul(qinv(np.repeat(add.lr[:1], n, 0)), alr)                 # local delta vs the additive's frame 0 (== rest)
    lr0 = lr0.copy()
    for nm in ARM[side]:
        j = IX[nm]
        lr0[:n, j] = qmul(lr0[:n, j], d[:, j])
    for nm in TORSO_UP:
        j = IX[nm]
        dj = NB.qslerp(np.repeat(np.array([[0, 0, 0, 1.0]]), n, 0), d[:, j], np.full(n, FIRE_TORSO))
        lr0[:n, j] = qmul(lr0[:n, j], dj)
    vl, vr = np.repeat(van['lt'][:1], T, 0), np.repeat(van['lr'][:1], T, 0)
    lt, lr = assemble(lt0, lr0, {s: np.repeat(wl0[s][:1], T, 0) for s in DC.SIDES}, vl, vr)
    info = dict(pins=('start', 'end'), donor='%s:aim_ovr_idle f0 + aim_recoil_add (%d fr) on the %s arm chain, torso x %.1f' % (NPC, add.n, side, FIRE_TORSO),
                time_map='additive time %s' % np.round(tau, 2).tolist(),
                game_clip='add_recoil_single_shot_shoulder' + ('_l' if side == 'Left' else '') + ' (vanilla Lexington: 3 fr additive)',
                additive_rule='local post-multiply: q = q_A0 * q_add(0)^-1 * q_add(t) (assumption: local-space additive)')
    return lt, lr, dict(lt=vl, lr=vr), info


def clip_reload():
    A = aim_pose()
    dn, wl, raw = donor('ovr_reload')
    van = vanilla('reload')
    T = len(van['lt'])
    sep = np.linalg.norm(dn.W[:, IX['LeftHand'], :3, 3] - dn.W[:, IX['RightHand'], :3, 3], axis=1)
    k = int(np.argmin(sep))
    MAG_IN = 27
    tau = np.interp(np.arange(T), [0, MAG_IN, T - 1], [0, k, dn.n - 1])
    ult, ulr = NB.resample(dn.lt, dn.lr, tau)
    wn = {s: resample_mats(wl[s], tau) for s in DC.SIDES}
    lt, lr = assemble(ult, ulr, wn, van['lt'], van['lr'])
    return lt, lr, van, dict(pins=('start', 'end'), donor='%s:ovr_reload (%d fr)' % (NPC, dn.n), time_map='piecewise linear [0, %d, %d] -> NPC [0, %d, %d]; NPC hands-closest '
                             'frame %d (%.1f cm) on vanilla mag_in f%d' % (MAG_IN, T - 1, k, dn.n - 1, k, sep[k] * 100, MAG_IN),
                             pinned='f0 and f%d = A0 (offset ramps %d fr)' % (T - 1, RAMP))


def clip_equip():
    dn, wl, raw = donor('ovr_equip')
    van = vanilla('equip')
    T = len(van['lt'])
    # the NPC cross-draws from holsters (weapon bone ~60 cm from the hand at f0, hands crossed until ~f18); V has no holsters in TPP and the
    # game puts the guns in the hands: only the NPC raise AFTER the draw is used (first frame with both guns in the hands and the hands
    # >= 10 cm uncrossed), gun in the hand with the vanilla grip on every frame (wnpc=None)
    rw, ix = raw['W'], raw['ix']
    H, _ = DC.body_frame(rw, ix)
    lx = np.einsum('tj,tj->t', H[:, :, 0], rw[:, ix['LeftHand'], :3, 3])
    rx = np.einsum('tj,tj->t', H[:, :, 0], rw[:, ix['RightHand'], :3, 3])
    wh = np.max([np.linalg.norm(rw[:, ix[WPN[s_]], :3, 3] - rw[:, ix[s_ + 'Hand'], :3, 3], axis=1) for s_ in DC.SIDES], 0)
    bad = np.where((lx > rx - 0.10) | (wh > 0.12))[0]
    st_draw = int(bad.max()) + 1
    T0 = dn.n
    lt0, lr0 = assemble(dn.lt, dn.lr, None, np.repeat(van['lt'][:1], T0, 0), np.repeat(van['lr'][:1], T0, 0))
    W0 = NB.fk(lt0, lr0)
    gg = np.array([gun_gun_clear(W0[t]) for t in range(T0)])
    close = np.where(gg < 0.01)[0]
    st = max(st_draw, int(close.max()) + 1 if len(close) else 0)
    u = np.linspace(0, 1, T)
    tau = st + (dn.n - 1 - st) * u ** EQUIP_EXP
    ult, ulr = NB.resample(dn.lt, dn.lr, tau)
    lt, lr = assemble(ult, ulr, None, van['lt'], van['lr'])
    return lt, lr, van, dict(pins=('end',), donor='%s:ovr_equip f%d..f%d (the raise after the cross-draw; draw ends f%d, guns >= 1 cm apart from '
                             'f%d), gun in hand with the vanilla grip on every frame' % (NPC, st, dn.n - 1, st_draw, st), time_map='tau = %d + %d * u^%.1f over %d frames' % (st, dn.n - 1 - st, EQUIP_EXP, T),
                             pinned='f%d = A0 (offset ramp %d fr)' % (T - 1, RAMP))


SPINE_FEATS = ['spine_bend_Spine1', 'spine_bend_Spine2', 'spine_bend_Spine3', 'spine_twist_Spine1', 'spine_twist_Spine2', 'spine_twist_Spine3',
               'spine_bend_total', 'spine_twist_total', 'torso_lean_deg']


def fix_anatomy(lt, lr, log, pinned=()):
    """only where V's base-body gate fails (melee tools, unchanged): spine bend / twist redistributed over Spine1 / Spine2 with the chest
    (Spine3) world rotation KEPT (guns unchanged); if the chest twist still breaks shoulder_width: Spine1..3 towards the base body (chest
    moves); neck / head and clavicles towards the base body (upper arm world kept); wrist twist into the forearm (hand world kept); wrists
    >= 27.5 cm from the eyes (arm IK, hand world rotation kept).  pinned frames (A0 ends) are restored afterwards by pin()."""
    T = len(lt)
    for sd in DC.SIDES:
        log['wrist_twist_to_forearm_deg_' + sd[0]] = round(NB.wrist_to_forearm(lt, lr, sd), 1)
    s, _ = NB.choose_scale(lt, lr, lambda a, b, s: NB.to_base(a, b, ['Spine1', 'Spine2'], s, ('Spine3',)), SPINE_FEATS, NB.BASE_GRID)
    s = NB.tsmooth(s, T)
    if s.max() > 0:
        NB.to_base(lt, lr, ['Spine1', 'Spine2'], s, ('Spine3',))
    log['spine12_to_base_max'] = round(float(s.max()), 2)
    s, _ = NB.choose_scale(lt, lr, lambda a, b, s: NB.to_base(a, b, ['Spine1', 'Spine2', 'Spine3'], s), SPINE_FEATS + ['shoulder_width_cm'],
                           NB.BASE_GRID)
    s = NB.tsmooth(s, T)
    if s.max() > 0:
        NB.to_base(lt, lr, ['Spine1', 'Spine2', 'Spine3'], s)
    log['spine123_to_base_max'] = round(float(s.max()), 2)
    for name, feats, joints, keep in NB.FIX_GROUPS[1:]:
        s, _ = NB.choose_scale(lt, lr, lambda a, b, s, j=joints, k=keep: NB.to_base(a, b, j, s, k), feats, NB.BASE_GRID)
        s = NB.tsmooth(s, T)
        if s.max() > 0:
            NB.to_base(lt, lr, joints, s, keep)
        log['to_base_max_' + name] = round(float(s.max()), 2)
    for sd in DC.SIDES:
        feats = [f % sd[0] for f in ('clav_swing_%s', 'clav_twist_%s', 'clav_shift_%s_cm')] + ['clav_gap_cm', 'shoulder_width_cm']
        a_, _ = NB.choose_scale(lt, lr, lambda a, b, s, sd=sd: NB.clav_retract(a, b, sd, s), feats, NB.RETRACT_GRID)
        a_ = NB.tsmooth(a_ / NB.RETRACT_GRID[-1], T) * NB.RETRACT_GRID[-1]
        if a_.max() > 0:
            NB.clav_retract(lt, lr, sd, a_)
        log['clav_retract_max_deg_' + sd[0]] = round(float(a_.max()), 1)
    clav_pass2(lt, lr, log)
    hand_gun_push(lt, lr, log)
    log['eye_push_cm'] = round(NB.eye_push(lt, lr), 1)
    for sd in DC.SIDES:
        log['wrist_twist_to_forearm2_deg_' + sd[0]] = round(NB.wrist_to_forearm(lt, lr, sd), 1)
    swivel_fix(lt, lr, log)                                       # last: elbow swivel keeps the wrists and hands (guns, eye distance)
    return lt, lr


ARM_FEATS = {s: [k for k in NB.AL if k.endswith('_' + s[0]) and any(p in k for p in ('uparm', 'elbow', 'forearm', 'wrist', 'hand'))]
             for s in DC.SIDES}


def arm_swivel(lt, lr, side, phi_deg):
    """(= npc_blade_build.arm_swivel) whole arm rigidly about the shoulder -> wrist axis by phi [T] deg; wrist and hand world rotation kept."""
    W = NB.fk(lt, lr)
    a, h = IX[side + 'Arm'], IX[side + 'Hand']
    u = W[:, h, :3, 3] - W[:, a, :3, 3]
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    Rh = W[:, h, :3, :3].copy()
    Rr = R.from_rotvec(u * np.radians(phi_deg)[:, None]).as_matrix()
    NB.set_world_rot(lt, lr, a, Rr @ W[:, a, :3, :3])
    NB.set_world_rot(lt, lr, h, Rh)


def swivel_fix(lt, lr, log):
    """(= npc_blade_build.swivel_fix without the lock) per arm, only where an arm feature of V's base gate fails: the elbow swivel (grid,
    mildest first, one sign per arm and clip) that lowers the arm violation most, smoothed in time; gun / hand unchanged."""
    T = len(lt)
    for side in DC.SIDES:
        feats = ARM_FEATS[side]
        v0 = NB.viol_of(NB.fk(lt, lr), feats)
        if not (v0 > 1e-9).any():
            continue
        cand = {}
        for sg in (1.0, -1.0):
            best, bvs = np.zeros(T), v0.copy()
            for g in (10, 20, 30, 45, 60):
                lt2, lr2 = lt.copy(), lr.copy()
                arm_swivel(lt2, lr2, side, np.full(T, sg * g) * (v0 > 1e-9))
                NB.wrist_to_forearm(lt2, lr2, side)
                v = NB.viol_of(NB.fk(lt2, lr2), feats)
                imp = v < bvs - 1e-6
                best[imp] = g
                bvs = np.where(imp, v, bvs)
            cand[sg] = (float(bvs.sum()), best)
        sgn = 1.0 if cand[1.0][0] <= cand[-1.0][0] else -1.0
        best = cand[sgn][1]
        phi = sgn * np.maximum(NB.smooth(NB.dilate(best, 2), 1.5), best) if T > 4 else sgn * best
        if np.abs(phi).max() > 0:
            arm_swivel(lt, lr, side, phi)
            NB.wrist_to_forearm(lt, lr, side)
        log['swivel_max_deg_' + side[0]] = round(float(np.abs(phi).max()), 1)


def clav_pass2(lt, lr, log):
    """clavicles again after the retract (its time dilation can push clav_swing below V's range).  clav_swing is not monotonic along the
    slerp to the base body (small s passes through a near-neutral clavicle, below the range), so choose_scale + dilation can break the
    neighbours: iterative instead - failing frames step up the grid, then tsmooth, until no frame of the group fails (high s = base)."""
    T = len(lt)
    for name, feats, joints, keep in NB.FIX_GROUPS[2:]:
        s_ = np.zeros(T)
        for it in range(len(NB.BASE_GRID) + 1):
            a, b = lt.copy(), lr.copy()
            if s_.max() > 0:
                NB.to_base(a, b, joints, s_, keep)
            bad = NB.viol_of(NB.fk(a, b), feats) > 1e-9
            if not bad.any():
                break
            nxt = np.array([min([g for g in NB.BASE_GRID if g > x + 1e-9] or [1.0]) for x in s_])
            s_ = NB.tsmooth(np.where(bad, nxt, s_), T)
        if s_.max() > 0:
            NB.to_base(lt, lr, joints, s_, keep)
        log['to_base2_max_' + name] = round(float(s_.max()), 2)


def hand_gun_push(lt, lr, log, iters=4):
    """hand (proxy hand + finger vertices) inside / closer than PUSH_MARGIN to the OTHER gun, or the guns closer than PUSH_MARGIN: the
    penetrating hand (gun vs gun: the slower hand) moves away from the other hand (arm hinge IK, hand world rotation kept, so its gun moves
    with it), deficit dilated + smoothed in time (no pops)."""
    import redo_pen as RPN
    px = RPN.proxy()
    names = np.array(px.names)[px.lab].astype(str)
    hv = {s: np.char.startswith(names, s + 'Hand') for s in DC.SIDES}
    oth = {'Left': 'Right', 'Right': 'Left'}
    moved = {s: 0.0 for s in DC.SIDES}
    it = 0
    for it in range(iters):
        W = NB.fk(lt, lr)
        T = len(W)
        need = {s: np.zeros(T) for s in DC.SIDES}
        hc = {s: W[:, IX[s + 'Hand'], :3, 3] for s in DC.SIDES}
        sp = {s: np.r_[0, np.linalg.norm(np.diff(hc[s], axis=0), axis=1)] for s in DC.SIDES}
        for t in range(T):
            V = px.verts(W[t, :73])
            for s in DC.SIDES:
                sd = float(gun_sdf(to_local(W[t, IX[WPN[oth[s]]]], V[hv[s]])).min())
                need[s][t] = max(need[s][t], PUSH_MARGIN - sd)
            gg = gun_gun_clear(W[t])
            slow = 'Left' if sp['Left'][t] <= sp['Right'][t] else 'Right'
            need[slow][t] = max(need[slow][t], PUSH_MARGIN - gg)
        if max(need[s].max() for s in DC.SIDES) <= 1e-4:
            break
        for s in DC.SIDES:
            dfc = NB.smooth(NB.dilate(np.maximum(need[s], 0), 2), 1.5) if T > 4 else np.maximum(need[s], 0)
            if dfc.max() <= 1e-4:
                continue
            d = hc[s] - hc[oth[s]]
            u = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-6)
            tgt = hc[s] + u * dfc[:, None]
            HI.solve(NB.fk, PAR, NB.REST_LR, lt, lr, IX[s + 'Arm'], IX[s + 'ForeArm'], IX[s + 'Hand'], tgt, weight=(dfc > 1e-4).astype(float),
                     end_mode='world')
            moved[s] = max(moved[s], float(dfc.max()))
    log['hand_gun_push_cm'] = {s[0]: round(moved[s] * 100, 1) for s in DC.SIDES}
    log['hand_gun_push_iters'] = it + 1


_AF = []


def A_frame():
    """A0 = the aim pose (aim donor f0, assembled, squared / aligned, anatomy-fixed) as locals [152] (helpers re-anchored later)."""
    if not _AF:
        dn, wl, raw = donor('aim_ovr_idle')
        van = vanilla('idlepose_shoulder')
        lt, lr = assemble(dn.lt[:1], dn.lr[:1], {s: wl[s][:1] for s in DC.SIDES}, van['lt'][:1], van['lr'][:1])
        log = {}
        lt, lr = fix_anatomy(lt, lr, log)
        aim_pose()['A0_fix'] = log
        _AF.extend([lt[0].copy(), lr[0].copy()])
    return _AF[0], _AF[1]


BUILDERS = {'idlepose_shoulder': clip_idle, 'aim_idle_loop': clip_loop, 'fire_right': lambda: clip_fire('Right'),
            'fire_left': lambda: clip_fire('Left'), 'reload': clip_reload, 'equip': clip_equip}
ORDER = ['idlepose_shoulder', 'aim_idle_loop', 'fire_right', 'fire_left', 'reload', 'equip']


# ------------------------------------------------------------------ gun proxies and gates
def box_sdf(p, lo, hi):
    c, h = (np.array(lo) + np.array(hi)) / 2, (np.array(hi) - np.array(lo)) / 2
    q = np.abs(p - c) - h
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def gun_sdf(p_local, boxes=None):
    return np.min([box_sdf(p_local, lo, hi) for lo, hi in (boxes or GUN_BOXES)], axis=0)


def gun_surface(n=7):
    pts = []
    for lo, hi in GUN_BOXES:
        g = [np.linspace(lo[a], hi[a], n) for a in range(3)]
        X, Y, Z = np.meshgrid(*g, indexing='ij')
        P3 = np.stack([X, Y, Z], -1).reshape(-1, 3)
        on = np.any(np.isclose(P3, lo) | np.isclose(P3, hi), axis=1)
        pts.append(P3[on])
    return np.concatenate(pts)


GS = gun_surface()


def gun_gun_clear(Wt):
    """one frame [J,4,4]: min signed distance (m) of either gun's proxy surface points to the other gun's proxy."""
    ML, MR = Wt[IX['WeaponLeft']], Wt[IX['WeaponRight']]
    return min(float(gun_sdf(to_local(MR, GS @ ML[:3, :3].T + ML[:3, 3])).min()),
               float(gun_sdf(to_local(ML, GS @ MR[:3, :3].T + MR[:3, 3])).min()))


def to_local(M, p):
    """world points p [N,3] into frame M [4,4]."""
    return (p - M[:3, 3]) @ M[:3, :3]


def gun_gates(W):
    """per frame: gun-vs-body vertices inside (holding hand + fingers excluded) and clearance, gun-vs-gun clearance, gun-vs-head clearance."""
    import redo_pen as RPN
    px = RPN.proxy()
    names = np.array(px.names)[px.lab]
    excl = {s: np.char.startswith(names.astype(str), s + 'Hand') for s in DC.SIDES}
    fore = {s: names.astype(str) == s + 'ForeArm' for s in DC.SIDES}
    head = np.isin(names, ['Head', 'LeftEye', 'RightEye', 'Neck1'])
    T = len(W)
    out = dict(body_in=np.zeros(T, int), body_clear=np.zeros(T), gun_gun=np.zeros(T), head_clear=np.zeros(T))
    for t in range(T):
        V = px.verts(W[t, :73])
        bc, bi, hc = 1e9, 0, 1e9
        for s in DC.SIDES:
            M = W[t, IX[WPN[s]]]
            Vl = to_local(M, V)
            sd = np.minimum(gun_sdf(Vl, [b for i, b in enumerate(GUN_BOXES) if i not in OWN_FORE_FREE]),
                            np.where(fore[s], 1e9, gun_sdf(Vl, [GUN_BOXES[i] for i in OWN_FORE_FREE])))
            keep = ~excl[s]
            bi += int((sd[keep] < 0).sum())
            bc = min(bc, float(sd[keep].min()))
            hc = min(hc, float(sd[head].min()))
        gg = gun_gun_clear(W[t])
        out['body_in'][t], out['body_clear'][t], out['gun_gun'][t], out['head_clear'][t] = bi, bc, gg, hc
    return out


def helper_gate(W, Wv):
    """per frame |helper - anchor| new vs the vanilla distance (cm) for every hand IK helper incl. arm_*_ik_offset; max over frames."""
    res, worst = {}, 0.0
    for s in DC.SIDES:
        sl = s.lower()
        for h, anc in [('arm_%s_ik' % sl, s + 'Hand'), ('arm_%s_ik_offset' % sl, s + 'Hand'), ('Torso_%sArm_Hand_IK_JNT' % s, s + 'Hand'),
                       ('Torso_World_%sArm_Hand_IK_JNT' % s, s + 'Hand'), ('arm_%s_ik_weapon' % sl, WPN[s])]:
            dn_ = np.linalg.norm(W[:, IX[h], :3, 3] - W[:, IX[anc], :3, 3], axis=1) * 100
            dv = np.linalg.norm(Wv[:, IX[h], :3, 3] - Wv[:, IX[anc], :3, 3], axis=1) * 100
            dv = dv if len(dv) == len(dn_) else np.repeat(dv[:1], len(dn_))
            dev = np.abs(dn_ - dv)
            res[h] = dict(max_cm=round(float(dn_.max()), 2), first_cm=round(float(dn_[0]), 2), last_cm=round(float(dn_[-1]), 2),
                          dev_vs_vanilla_max_cm=round(float(dev.max()), 2))
            worst = max(worst, float(dev.max()), float(dn_.max()) if h != 'arm_%s_ik_offset' % sl else 0.0)
    return res, worst


def palm_gate(W):
    """palm (Hand + Middle1)/2 in the weapon frame vs the vanilla aim grip (cm, max over frames) and the barrel angle to the hips forward at f0."""
    van = vanilla('idlepose_shoulder')['W']
    out = {}
    for s in DC.SIDES:
        def palm_in(Wx):
            p = (Wx[:, IX[s + 'Hand'], :3, 3] + Wx[:, IX[s + 'HandMiddle1'], :3, 3]) / 2
            M = Wx[:, IX[WPN[s]]]
            return np.einsum('tji,tj->ti', M[:, :3, :3], p - M[:, :3, 3])
        dv = np.linalg.norm(palm_in(W) - palm_in(van[:1]), axis=1) * 100
        out[s] = round(float(dv.max()), 2)
    return out


def barrels(W, f=0):
    H, _ = DC.body_frame(W, IX)
    res = {}
    for s in DC.SIDES:
        b = W[f, IX[WPN[s]], :3, 1]
        bb = H[f].T @ b
        res[s] = dict(yaw_deg=round(float(np.degrees(np.arctan2(bb[0], bb[1]))), 1), pitch_deg=round(float(np.degrees(np.arcsin(np.clip(bb[2], -1, 1)))), 1))
    return res


def chest_yaw(W):
    """Spine3 heading vs the hips heading (deg) per frame, from the shoulders line."""
    H = MC.heading_frame(W[:, IX['LeftUpLeg'], :3, 3], W[:, IX['RightUpLeg'], :3, 3])
    C = MC.heading_frame(W[:, IX['LeftArm'], :3, 3], W[:, IX['RightArm'], :3, 3])
    return np.degrees(np.arctan2(np.einsum('ti,ti->t', C[:, :, 1], H[:, :, 0]), np.einsum('ti,ti->t', C[:, :, 1], H[:, :, 1])))


def run_gates(W, van_W, loop=False):
    g = NB.gates(W, np.zeros((len(W), 3)), loop=loop)
    gg = gun_gates(W)
    hres, hworst = helper_gate(W, van_W)
    out = {k: v for k, v in g.items() if not k.startswith('_') and k not in ('fist_fist_min_cm', 'PASS')}
    hands = np.linalg.norm(W[:, IX['LeftHand'], :3, 3] - W[:, IX['RightHand'], :3, 3], axis=1) * 100
    out.update(gun_body_in_max=int(gg['body_in'].max()), gun_body_frames=int((gg['body_in'] > 0).sum()),
               gun_body_clear_min_cm=round(float(gg['body_clear'].min() * 100), 2), gun_gun_clear_min_cm=round(float(gg['gun_gun'].min() * 100), 2),
               gun_head_clear_min_cm=round(float(gg['head_clear'].min() * 100), 2), helpers_worst_cm=round(hworst, 2), helpers=hres,
               palm_vs_vanilla_grip_max_cm=palm_gate(W), hands_dist_min_cm=round(float(hands.min()), 1), barrels_f0=barrels(W),
               chest_yaw_vs_hips_deg=[round(float(chest_yaw(W).min()), 1), round(float(chest_yaw(W).max()), 1)])
    ok = (out['base_gate_pass'] == len(W) and out['pen_frames'] == 0 and out['head_clear_min_cm'] >= 0 and max(out['slide_max_cm'].values()) <= NB.SLIDE_MAX_CM
          and out['sole_min_cm'] >= NB.FLOOR_CM and out['spike_max_cm'] <= NB.JUMP_CM and out['root_identity'] and out['gun_body_in_max'] == 0
          and out['gun_gun_clear_min_cm'] >= 0 and out['gun_head_clear_min_cm'] >= 0 and hworst <= HELPER_LIMIT_CM)
    if loop:
        ok = ok and out['loop_gap_cm'] <= NB.LOOP_CM and out['loop_gap_deg'] <= NB.LOOP_DEG
    out['PASS'] = bool(ok)
    out['_margin'] = g['_ok_frames']
    out['_worst'] = g['_worst']
    out['_gun'] = {k: np.round(v * (100 if k != 'body_in' else 1), 2).tolist() for k, v in gg.items()}
    return out


# ------------------------------------------------------------------ main
def build(name):
    lt, lr, van, info = BUILDERS[name]()
    if not info.get('fixed'):
        log = {}
        lt, lr = fix_anatomy(lt, lr, log)
        info['anatomy_fix'] = log
        a_lt, a_lr = A_frame()
        for end in info.get('pins', ()):
            lt, lr = pin(lt, lr, a_lt, a_lr, ramp(len(lt), RAMP, at_end=(end == 'end')))
    info['A0'] = {k: v for k, v in aim_pose().items() if k in ('hips_raise_cm', 'theta', 'barrel_yaw_before', 'wrist_corr_deg', 'A0_fix')}
    lt, lr = reanchor(lt, lr, van['lt'], van['lr'])
    lt[:, IX['Root']] = 0
    lr[:, IX['Root']] = [0, 0, 0, 1.0]
    W = NB.fk(lt, lr)
    van_W = NB.fk(van['lt'], van['lr'])
    g = run_gates(W, van_W, loop=bool(info.get('loop')))
    meta = dict(set=SET, clip=name, rig='woman', animation_type='Normal', additive=False, root_motion_type='None', frames=len(lt),
                duration=(len(lt) - 1) / 30.0, events=[], npc_set=NPC, info=info)
    os.makedirs(CLIPS, exist_ok=True)
    base = os.path.join(CLIPS, '%s__%s.%s' % (SET, name, SUFFIX))
    NR.save_dump(base + '.npz', NR.make_dump(P, lt, lr, meta))
    info = dict(info, frames=len(lt), gates={k: v for k, v in g.items() if not k.startswith('_')})
    MC.jdump(dict(info, per_frame=dict(base_ok=[bool(x) for x in g['_margin']], worst=g['_worst'], gun=g['_gun'])), base + '.info.json')
    return info


def main(a):
    only = a[a.index('--only') + 1].split(',') if '--only' in a else ORDER
    t0 = time.time()
    rep = {}
    for name in only:
        info = build(name)
        g = info['gates']
        rep[name] = info
        print('%-18s %3d %s base %3d/%-3d fail=%s pen=%d head=%.1f slide=%s sole=%.1f spike=%.2f gunbody=%d (clr %.1f) gungun=%.1f gunhead=%.1f helpers=%.2f '
              'palm=%s hands_min=%.1f barrels=%s chest=%s' % (
                  name, info['frames'], 'PASS' if g['PASS'] else 'FAIL', g['base_gate_pass'], info['frames'], g['base_gate_fail'] or '-',
                  g['pen_frames'], g['head_clear_min_cm'], g['slide_max_cm'], g['sole_min_cm'], g['spike_max_cm'], g['gun_body_in_max'],
                  g['gun_body_clear_min_cm'], g['gun_gun_clear_min_cm'], g['gun_head_clear_min_cm'], g['helpers_worst_cm'],
                  g['palm_vs_vanilla_grip_max_cm'], g['hands_dist_min_cm'], g['barrels_f0'], g['chest_yaw_vs_hips_deg']))
    prev = {}
    rp = os.path.join(CLIPS, 'build_report.json')
    if os.path.exists(rp) and only != ORDER:
        prev = json.load(open(rp, encoding='utf-8'))
    prev.update(rep)
    MC.jdump(prev, rp)
    print('done %.1f s' % (time.time() - t0))


if __name__ == '__main__':
    main(sys.argv[1:])
