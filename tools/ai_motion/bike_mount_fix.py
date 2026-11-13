"""Motorcycle ENTER (mount) clips rebuilt from three sources (bike13, 2026-11-09): the leg that goes over the saddle must not pass
through the bike, and the hand IK helpers must sit on the hands.

Why. Measured against the real bike meshes (bike_model.py): the installed c9 clips (utpp_tpp_p?a_sportbike*_enter_?, bike2..bike12)
bring the hips over the centre line while V still stands (frame 9 of 43), both legs then run THROUGH the bike body (worst -8.5 cm,
100-250 surface crossings per leg) and the swing foot crosses the centre line 40 cm below the seat point. The vanilla FPP player clip
(p?a_sportbike*.anims enter_l/r) has the right hips path, step and leg-over timing for the player's seat frame but swings the foot
LOW behind the seat (through the player Apollo's rear top case, -6 cm) and its seated torso is the FPP one (spine compressed by 10 cm).
The vanilla NPC clip (?a_sportbike*.anims enter_l/r, 45-51 frames) has the TPP torso and a high leg-over (knee above the hips, foot
+16 cm over the saddle in front of the case, toe tip grazing the case by 3-5 cm) but walks 117 cm and ends with both feet down.
Its frames n = f + k follow the FPP hips lateral path (k = 17-18 for pwa_sportbike3 enter_l; fitted per clip).

Frames. Everything is designed in the SEAT frame (clip Root of the last frame = the vehicle's seat_front bone, common to all three
clips) and written under the installed clip's Root track (the c9 builder's smooth root path, start yaw -10 deg; validated in game).

Build (player rig, 43 frames):
  * Hips: position FPP until --hips-blend a, then eased into the installed clip's hips (= the TPP drive / idle_stand_bike pose) by b;
    orientation: the installed clip's (facing the approach direction) eased into the FPP's (turned to the bike, turning forward with
    the leg-over) over --hips-yaw-blend, installed again from a..b;
  * torso (spine .. head, shoulders, arms, hands, fingers): the NPC's LOCAL rotations at n = f + k (hips-relative), eased into the
    installed clip's over --torso-blend; two-bone arm IK holds a hand on its grip (the installed clip's final hand spot) while the
    NPC hand is on its grip;
  * stance leg: FPP until its foot starts for the peg (auto), then eased into the installed clip's leg with the ankle pinned (IK) from
    the FPP plant to the final TPP foot spot;
  * swing leg: FPP until lift-off, then the leg-over DONOR's joint angles (--npc-leg, default the female Apollo NPC clip of the same
    side: high clean swing; the male and Arch NPC clips swing low behind the saddle through the case / tail) under the new hips, the
    ankle driven by two-bone IK along the donor's hips-relative ankle path (scaled by the leg length ratio; lift-off from the FPP spot,
    landing on the installed peg spot), knee pole = the donor's knee direction, fold capped at --knee-max; the airborne path is raised /
    moved (and the knee pole raised for a thigh / upper-shin deficit) where the leg would come closer than --margin to the bike mesh
    (iterated against bike_model), then the landed pose is held and eased into the installed clip's leg on the peg (ankle pinned);
  * limb IK helpers re-anchored on the new limbs with the vanilla (FPP) offsets on EVERY frame (exit_steps.reanchor_helpers: feet /
    hand targets on the bones, knee / elbow poles in the limb plane); arm_*_ik_offset / _weapon keep the vanilla locals (the bike12
    rule: a stale target glues the hand to the hip in game);
  * FootPlant events recomputed, the other events / tracks of the installed clip kept.
Report: alignment, key frames, swing / stance leg clearance (min cm, surface crossings) before (installed) and after, knee flexion, joint
acceleration, rotation steps, helper distances, pole side, end-pose error vs the installed clip.

  python -I bike_mount_fix.py --inst <installed glb:clip> --fpp <vanilla player glb:clip> --npc <vanilla NPC glb:clip> --set pwa_sportbike3 --side l --out <glb> [--report json]
No game, no install. Requires trimesh / scipy (Tools\\ai-motion\\envs\\kimodo).
"""
import argparse
import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import skel as S  # noqa: E402
import glbanim as G  # noqa: E402
import ik as IK  # noqa: E402
import build_entry_clip as B  # noqa: E402
import exit_steps as ES  # noqa: E402
import bike_model as BM  # noqa: E402

CR = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # game (x right, y fwd, z up) -> glTF (Y up)
FPS = 30.0
SKIP = ('Root', 'Trajectory', 'reference_joint')
ACCEL_JOINTS = ['Hips', 'Spine3', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot', 'LeftForeArm', 'RightForeArm', 'LeftLeg', 'RightLeg']


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def ramp(T, a, b):
    """(T,) 0 before a, smoothstep a..b, 1 from b"""
    if b <= a:
        return (np.arange(T) >= a).astype(float)
    return smoothstep((np.arange(T) - a) / float(b - a))


def is_leg(n):
    return (n.startswith('Left') or n.startswith('Right')) and any(k in n for k in ('UpLeg', 'Leg', 'Foot', 'Toe', 'Heel'))


def leg_bones(side):
    return [side + x for x in ('UpLeg', 'Leg', 'Foot', 'ToeBase')]


class Clip:
    def __init__(self, spec, times=None):
        glb, clip = spec.rsplit(':', 1)
        self.glb, self.clip = glb, clip
        self.sk = S.Skeleton.from_glb(glb)
        self.c = self.sk.clip(clip)
        self.dur, t = B.frames_of(self.sk, self.c)
        self.t = t if times is None else times * (self.dur / max(times[-1], 1e-9))
        self.lt, self.lr = self.sk.sample_local(self.c, self.t)
        self.lr = S.qcontinuous(self.lr)
        self.Wt, self.Wr = self.sk.world(self.lt, self.lr)
        ri = self.sk.index['Root']
        self.R0, self.t0 = self.Wr[-1, ri].copy(), self.Wt[-1, ri].copy()
        self.n = len(self.t)
        self.ix = self.sk.index
        self.extras = copy.deepcopy(self.sk.gj['animations'][self.c].get('extras', {}))

    def mirror(self):
        """left <-> right mirrored pose (build_entry_clip.mirror_pose; the Root yaw flips with it)"""
        self.lt, self.lr = B.mirror_pose(self.sk, self.lt, self.lr)
        self.lr = S.qcontinuous(self.lr)
        self.Wt, self.Wr = self.sk.world(self.lt, self.lr)
        ri = self.sk.index['Root']
        self.R0, self.t0 = self.Wr[-1, ri].copy(), self.Wt[-1, ri].copy()
        self.mirrored = True

    def seat(self, Wt=None):
        """world (glTF) positions -> SEAT frame (Root of the last frame), game axes, metres"""
        Wt = self.Wt if Wt is None else Wt
        return np.einsum('ij,...j->...i', CR.T @ self.R0.T, Wt - self.t0)

    def to_world(self, P):
        """seat frame (game axes, m) -> this clip's world (glTF)"""
        return np.einsum('ij,...j->...i', self.R0 @ CR, P) + self.t0

    def rot_to(self, other_R0, Wr):
        """world rotations of another clip -> this clip's world (the seat frames coincide)"""
        return np.einsum('ij,...jk->...ik', self.R0 @ other_R0.T, Wr)


def align_offset(x_new, x_npc, f0=3, f1=27):
    """frame offset k with NPC hips x(f + k) closest to the new clip's hips x(f) over f0..f1 (seat frame)"""
    best = None
    for k in range(0, max(1, len(x_npc) - f1)):
        idx = np.minimum(np.arange(f0, f1 + 1) + k, len(x_npc) - 1)
        e = float(np.mean((x_npc[idx] - x_new[f0:f1 + 1]) ** 2))
        if best is None or e < best[0]:
            best = (e, k)
    return best[1], np.sqrt(best[0]) * 100


def spline_path(keys, T, v0=None):
    """keys [(frame, (x, y, z))] -> (T, 3) cubic path (scipy CubicSpline, clamped: v0 per frame at the first key, rest at the last); outside the keys the ends hold"""
    from scipy.interpolate import CubicSpline
    fr = np.array([k[0] for k in keys], float)
    pts = np.array([k[1] for k in keys], float)
    cs = CubicSpline(fr, pts, bc_type=((1, v0 if v0 is not None else np.zeros(3)), (1, np.zeros(3))))
    f = np.clip(np.arange(T, dtype=float), fr[0], fr[-1])
    return cs(f)


def knee_flexion_deg(P, ix, side):
    a, b, c = P[:, ix[side + 'UpLeg']], P[:, ix[side + 'Leg']], P[:, ix[side + 'Foot']]
    u, v = a - b, c - b
    cosang = np.sum(u * v, -1) / np.maximum(np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1), 1e-9)
    return 180.0 - np.degrees(np.arccos(np.clip(cosang, -1, 1)))


def set_bone_world(sk, lt, lr, j, Wt_new, Wr_new):
    """local translation / rotation of bone j from a desired world transform (parents unchanged)"""
    Wt, Wr = sk.world(lt, lr)
    p = sk.parent[j]
    if p < 0:
        lt[:, j] = Wt_new
        lr[:, j] = S.qcontinuous(S.m2q(Wr_new))
        return
    inv = np.transpose(Wr[:, p], (0, 2, 1))
    lt[:, j] = np.einsum('tij,tj->ti', inv, Wt_new - Wt[:, p])
    lr[:, j] = S.qcontinuous(S.m2q(inv @ Wr_new))


def pole_side(Wt, ix, side):
    """per frame: cos of the angle between (pole - knee) and the knee bend direction (+ = pole on the knee side), both poles"""
    a, k, e = Wt[:, ix[side + 'UpLeg']], Wt[:, ix[side + 'Leg']], Wt[:, ix[side + 'Foot']]
    u = e - a
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    v = (k - a) - np.sum((k - a) * u, 1, keepdims=True) * u
    bent = np.linalg.norm(v, axis=1) > 0.02
    v /= np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9)
    out = {}
    for pn in ('Torso_%sLeg_PoleVector_JNT' % side, 'Torso_World_%sLeg_PoleVector_JNT' % side):
        if pn in ix:
            d = Wt[:, ix[pn]] - k
            d = d - np.sum(d * u, 1, keepdims=True) * u
            d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
            c = np.sum(d * v, 1)
            out[pn] = dict(min_cos_bent=round(float(c[bent].min()), 3) if bent.any() else None, bent_frames=int(bent.sum()))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--inst', required=True, help='installed clip glb:clip (player rig, Root track, end pose, events)')
    p.add_argument('--fpp', required=True, help='vanilla FPP player clip glb:clip (hips path, legs, helper offsets)')
    p.add_argument('--npc', required=True, help='vanilla NPC clip glb:clip (torso, knee direction, foot orientation)')
    p.add_argument('--set', required=True, help='p?a_sportbike[2|3] (bike geometry; sportbike1 has none: --no-mesh)')
    p.add_argument('--side', required=True, choices=('l', 'r'))
    p.add_argument('--out', required=True)
    p.add_argument('--report')
    p.add_argument('--npc-offset', type=int, default=None, help='frame offset k (NPC n = f + k); default fitted on the hips lateral path')
    p.add_argument('--hips-blend', default='27:36', help='a:b frames, FPP hips -> installed hips')
    p.add_argument('--hips-yaw-blend', default='0:0', help='a:b frames, installed hips orientation (facing the approach direction) -> FPP hips orientation (turned to the bike, the leg-over geometry of FPP / NPC needs it); default 0:0 = FPP orientation from the first frame (UTPP blends the mount start over 0.2 s)')
    p.add_argument('--torso-blend', default='30:40', help='a:b frames, NPC torso -> installed torso')
    p.add_argument('--torso-in', default='3:15', help='a:b frames, installed torso (body facing the approach direction) -> NPC torso (body turned to the bike)')
    p.add_argument('--stance-blend-end', type=int, default=36)
    p.add_argument('--npc-leg', default=None, help='leg-over donor glb:clip (joint angles, hips-relative ankle path, knee direction); default: the female Apollo NPC clip of the same side (wa_sportbike3 enter_l/r: high clean swing, the male / Arch NPC clips swing low behind the saddle through the case / tail)')
    p.add_argument('--npc-leg-offset', type=int, default=None, help='frame offset of the leg donor (default fitted on the hips lateral path)')
    p.add_argument('--leg-mirror', default='auto', help="auto: mirror the leg donor when its side differs from --side (the enter_r donor clips tuck the heel at the hip); 1 / 0 force")
    p.add_argument('--knee-max', type=float, default=150.0, help='deg: swing knee flexion cap (the ankle target is pushed away from the hip joint)')
    p.add_argument('--path-lift', type=float, default=0.0, help='m: smooth lift of the whole airborne ankle path (sine window lift-off .. landing): the leg folds more and the knee rises everywhere (male Arch: lower hips, higher seat)')
    p.add_argument('--margin', type=float, default=0.06, help='m: minimum shin / foot clearance from the bike mesh while the swing foot is in the air (iterated)')
    p.add_argument('--max-bump', type=float, default=0.15, help='m: cap of the clearance push of the ankle target per frame (direction: the outward normal of the nearest bike surface blended with up)')
    p.add_argument('--iters', type=int, default=10)
    p.add_argument('--land-rot-frames', type=int, default=10, help='frames before the landing over which the swing foot orientation eases into the installed (peg) one')
    p.add_argument('--grip-pin', type=int, default=1, help='1: arm IK holds a hand on its grip (installed final hand spot) while the NPC hand is on the grip')
    p.add_argument('--no-mesh', action='store_true', help='no clearance iteration (sportbike1)')
    p.add_argument('--max-extend', type=float, default=0.985)
    p.add_argument('--bend-sigma', type=float, default=1.5, help='frames: gaussian smoothing of the swing knee bend direction over time (two_bone)')
    a = p.parse_args()

    inst = Clip(a.inst)
    T = inst.n
    if a.npc_leg is None:
        a.npc_leg = os.path.join(os.path.dirname(a.npc.rsplit(':', 1)[0]), 'wa_sportbike3.anims.glb') + ':enter_l'
    fpp = Clip(a.fpp, times=inst.t)
    npc = Clip(a.npc)
    sk, ix = inst.sk, inst.ix
    assert fpp.n == T and list(fpp.sk.names) == list(sk.names), 'FPP clip must be on the same rig / frame count'
    ri, hip = ix['Root'], ix['Hips']
    root_diff = float(np.abs(inst.lt[:, ri] - fpp.lt[:, ri]).max()) * 100
    swing = 'Right' if a.side == 'l' else 'Left'
    stance = 'Left' if a.side == 'l' else 'Right'
    sx = 1.0 if a.side == 'l' else -1.0
    hA, hB = (int(x) for x in a.hips_blend.split(':'))
    yA, yB = (int(x) for x in a.hips_yaw_blend.split(':'))
    tA, tB = (int(x) for x in a.torso_blend.split(':'))
    rep = dict(inst=a.inst, fpp=a.fpp, npc=a.npc, set=a.set, side=a.side, frames=T, swing=swing, root_track_diff_vs_fpp_cm=round(root_diff, 1))

    Pf, Pi, Pn = fpp.seat(), inst.seat(), npc.seat()
    if a.npc_offset is None:
        k, kerr = align_offset(Pf[:, hip, 0], Pn[:, npc.ix['Hips'], 0])
    else:
        k, kerr = a.npc_offset, float('nan')
    nmap = np.minimum(np.arange(T) + k, npc.n - 1)
    rep['npc_offset'] = dict(k=int(k), fit_rms_cm=round(float(kerr), 1), npc_frames=int(npc.n))

    lt, lr = inst.lt.copy(), inst.lr.copy()
    # 1. hips (world blend in the seat frame, then local under the installed Root)
    wH = ramp(T, hA, hB)[:, None]
    wY = ramp(T, yA, yB)[:, None]
    hips_seat_blend = (1 - wH) * Pf[:, hip] + wH * Pi[:, hip]
    wHz = ramp(T, hA - 6, hB - 6)
    hips_seat_blend[:, 2] = (1 - wHz) * Pf[:, hip, 2] + wHz * Pi[:, hip, 2]     # the height arrives 6 frames earlier (entry gate c: the FPP sits deeper than the TPP idle)
    hips_pos = inst.to_world(hips_seat_blend)
    q_f = S.qcontinuous(S.m2q(inst.rot_to(fpp.R0, fpp.Wr[:, hip])))
    q_i = S.qcontinuous(S.m2q(inst.Wr[:, hip]))
    q_hips = G.slerp(G.slerp(q_i, q_f, wY), q_i, wH)
    set_bone_world(sk, lt, lr, hip, hips_pos, S.q2m(q_hips))

    # 2. torso: the installed clip's locals while the body still faces the approach direction (the NPC arms reach for the bars with the
    #    body turned to the bike), eased into the NPC locals at n (hips-relative) over --torso-in, eased back into the installed torso over --torso-blend
    nA, nB = (int(x) for x in a.torso_in.split(':'))
    wN = ramp(T, nA, nB)[:, None]
    wT = ramp(T, tA, tB)[:, None]
    torso = [n for n in npc.sk.names if n not in SKIP and n != 'Hips' and not is_leg(n) and n in ix]
    for n in torso:
        j, jn = ix[n], npc.ix[n]
        q_mid = G.slerp(inst.lr[:, j], S.qcontinuous(npc.lr[nmap, jn]), wN)
        lr[:, j] = G.slerp(q_mid, inst.lr[:, j], wT)
    lr = S.qcontinuous(lr)
    rep['torso_bones'] = len(torso)

    # 3. hands on the grips while the NPC hand is on its grip (and the NPC torso is in)
    if a.grip_pin:
        grip = {}
        for sd in ('Left', 'Right'):
            hn = npc.ix[sd + 'Hand']
            dn = np.linalg.norm(Pn[nmap, hn] - Pn[-1, hn], axis=1) * 100
            near = np.where(dn < 20.0)[0]
            f_grip = int(near[0]) if len(near) else T
            w = ramp(T, f_grip, f_grip + 8) * (1 - wT[:, 0]) * wN[:, 0]
            tgt = np.repeat(inst.Wt[-1:, ix[sd + 'Hand']], T, 0)
            Wt_a, _ = sk.world(lt, lr)
            IK.two_bone(sk, lt, lr, ix[sd + 'Arm'], ix[sd + 'ForeArm'], ix[sd + 'Hand'], tgt, weight=w, max_extend=0.995, bend_sigma=4.0, pole=Wt_a[:, ix[sd + 'ForeArm']].copy(), pole_weight=np.full(T, 0.7))
            grip[sd] = dict(frames_pinned=int((w > 0.5).sum()), first_pin=int(np.argmax(w > 0.5)) if (w > 0.5).any() else None)
        rep['grip_pin'] = grip

    # 4a. stance leg: FPP locals, then eased into the installed leg with the ankle pinned from the FPP plant to the final spot (an arc when the
    #     foot goes up to the peg: it rises first, then moves in)
    sw, st = swing, stance
    fo_f = ix[st + 'Foot']
    fo_s, kn_s, up_s, to_s = (ix[sw + x] for x in ('Foot', 'Leg', 'UpLeg', 'ToeBase'))
    f_cross = next((f for f in range(1, T) if np.sign(Pf[f, fo_s, 0]) != np.sign(Pf[f - 1, fo_s, 0])), T // 3)
    zf = Pf[:, fo_f, 2]
    zpl = zf[f_cross:].min()
    fs0 = next((f for f in range(f_cross, T) if zf[f] > zpl + 0.02), T - 8) - 1
    fs0 = int(max(fs0, f_cross + 2))
    fs1 = int(max(a.stance_blend_end, fs0 + 4))
    wS = ramp(T, fs0, fs1)[:, None]
    fr = np.arange(T)
    early = fr <= fs0
    for n in leg_bones(st):
        j = ix[n]
        held = np.repeat(fpp.lr[fs0:fs0 + 1, j], T, 0)
        q = G.slerp(held, inst.lr[:, j], wS)
        lr[early, j] = fpp.lr[early, j]
        lr[~early, j] = q[~early]
        lt[:, j] = np.where(early[:, None], fpp.lt[:, j], inst.lt[:, j])
    lr = S.qcontinuous(lr)
    plant_seat, final_seat = Pf[fs0, fo_f], Pi[-1, fo_f]
    rise = float(final_seat[2] - plant_seat[2])
    if rise > 0.08:   # ground -> peg: up first, then in, with a lift over the peg mount
        wz = ramp(T, fs0, int(round(fs0 + 0.6 * (fs1 - fs0))))[:, None]
        wxy = ramp(T, int(round(fs0 + 0.3 * (fs1 - fs0))), fs1)[:, None]
        tgt_seat = plant_seat + (final_seat - plant_seat) * np.concatenate([wxy, wxy, wz], 1)
        tgt_seat[:, 2] += 0.06 * np.sin(np.pi * wS[:, 0])
    else:
        tgt_seat = plant_seat + (final_seat - plant_seat) * wS
    tgtS = inst.to_world(tgt_seat)
    wpin = ((fr > fs0) & (fr <= fs1)).astype(float)
    Rst = S.q2m(G.slerp(S.qcontinuous(S.m2q(inst.rot_to(fpp.R0, fpp.Wr[:, fo_f]))), S.qcontinuous(S.m2q(inst.Wr[:, fo_f])), wS))
    Wt_, Wr_ = sk.world(lt, lr)
    Rcur = Wr_[:, fo_f].copy()
    Rcur[~early] = Rst[~early]
    IK.set_world_rot(sk, lt, lr, fo_f, Rcur)
    poleS = inst.to_world(np.repeat(Pi[-1:, ix[st + 'Leg']], T, 0))
    errS = IK.two_bone(sk, lt, lr, ix[st + 'UpLeg'], ix[st + 'Leg'], fo_f, tgtS, weight=wpin, max_extend=a.max_extend, bend_sigma=2.0, pole=poleS, pole_weight=wS[:, 0] * wpin)
    rep['stance'] = dict(hold_until=fs0, blend_end=fs1, ik_err_cm=round(errS * 100, 2), plant_seat=np.round(plant_seat * 100, 1).tolist(),
                         final_seat=np.round(final_seat * 100, 1).tolist(), rise_cm=round(rise * 100, 1))

    # 4b. swing leg: the leg-over DONOR (--npc-leg, default the female Apollo NPC clip of the same side: a high, clean swing) gives the joint angles,
    #     the hips-relative ankle path (scaled by the leg length ratio) and the knee direction; two-bone IK follows the path; the fold is capped
    #     (--knee-max); the airborne path is raised / moved where the shin / foot would come closer than --margin to the bike mesh
    npcL = Clip(a.npc_leg) if a.npc_leg else npc
    donor_side = 'l' if a.npc_leg.endswith('_l') else ('r' if a.npc_leg.endswith('_r') else a.side)
    leg_mirrored = (a.leg_mirror == 1) or (a.leg_mirror == 'auto' and donor_side != a.side)
    if leg_mirrored:
        npcL.mirror()
    PnL = npcL.seat()
    xd = PnL[:, npcL.ix[sw + 'Foot'], 0]
    n_cross = next((n for n in range(1, npcL.n) if np.sign(xd[n]) != np.sign(xd[n - 1]) and abs(xd[n] - xd[n - 1]) > 0.02), None)
    if a.npc_leg_offset is not None:
        kL, kLerr = a.npc_leg_offset, float('nan')
    elif n_cross is not None:
        kL, kLerr = int(max(0, n_cross - f_cross)), 0.0
    else:
        kL, kLerr = align_offset(Pf[:, hip, 0], PnL[:, npcL.ix['Hips'], 0])
    nmapL = np.minimum(np.arange(T) + kL, npcL.n - 1)
    Lp = float(np.linalg.norm(Pi[0, kn_s] - Pi[0, up_s]) + np.linalg.norm(Pi[0, fo_s] - Pi[0, kn_s]))
    Ln = float(np.linalg.norm(PnL[0, npcL.ix[sw + 'Leg']] - PnL[0, npcL.ix[sw + 'UpLeg']]) + np.linalg.norm(PnL[0, npcL.ix[sw + 'Foot']] - PnL[0, npcL.ix[sw + 'Leg']]))
    sc = Lp / max(Ln, 1e-6)
    rep['npc_leg'] = dict(clip=a.npc_leg or a.npc, mirrored=bool(leg_mirrored), k=int(kL), donor_cross=n_cross, fpp_cross=int(f_cross), fit_rms_cm=round(float(kLerr), 1), frames=int(npcL.n), leg_scale=round(sc, 4))
    zs = Pf[:, fo_s, 2]
    f_lift = int(max(next((f for f in range(1, T) if zs[f] > zs[0] + 0.03), 1) - 1, 0))
    d_inst = np.linalg.norm(Pi[:, fo_s] - Pi[-1, fo_s], axis=1) * 100
    f_inst = next((f for f in range(T) if (d_inst[f:] < 1.5).all()), T - 6)
    vs = np.r_[0, np.linalg.norm(np.diff(Pf[:, fo_s], axis=0), axis=1)] * FPS
    f_land = next((f for f in range(f_cross + 4, T) if vs[f] < 0.4), f_cross + 9)
    f_land = int(min(max(f_land, f_cross + 6), T - 10))
    f_pin_end = int(min(max(hB, f_inst + 1), T - 2))
    air = (fr > f_lift) & (fr < f_land - 2)
    wIn = ramp(T, f_lift, f_lift + 8)[:, None]           # FPP leg -> donor leg at the lift (joint angles: the foot orientation turns ~45 deg/frame over 5)
    wPath = ramp(T, f_lift, f_lift + 5)[:, None]         # FPP ankle spot -> the donor's hips-relative ankle path (5 frames: a slower hand-over kept the foot low on the Arch, the knee skimmed the seat edge)
    wPeg = ramp(T, f_land - 5, f_land)[:, None]
    for n in leg_bones(sw):
        j = ix[n]
        q = G.slerp(S.qcontinuous(fpp.lr[:, j]), S.qcontinuous(npcL.lr[nmapL, npcL.ix[n]]), wIn)
        pre = fr <= f_lift
        lr[pre, j] = fpp.lr[pre, j]
        lr[~pre, j] = q[~pre]
        lt[:, j] = np.where(pre[:, None], fpp.lt[:, j], inst.lt[:, j])
    lr = S.qcontinuous(lr)
    Wt_, Wr_ = sk.world(lt, lr)
    hips_seat = inst.seat(Wt_[:, hip])
    hipj = inst.seat(Wt_[:, up_s])
    P_cur = inst.seat(Wt_[:, fo_s])
    peg = Pi[-1, fo_s]
    rel = (PnL[nmapL, npcL.ix[sw + 'Foot']] - PnL[nmapL, npcL.ix['Hips']]) * sc
    path = (1 - wPath) * np.repeat(P_cur[f_lift:f_lift + 1], T, 0) + wPath * (hips_seat + rel)
    path = (1 - wPeg) * path + wPeg * peg
    la = float(np.linalg.norm(Pi[0, kn_s] - Pi[0, up_s])); lb = float(np.linalg.norm(Pi[0, fo_s] - Pi[0, kn_s]))
    dmin = float(np.sqrt(la * la + lb * lb - 2 * la * lb * np.cos(np.radians(180.0 - a.knee_max))))
    dvec = path - hipj
    dn = np.linalg.norm(dvec, axis=1)
    clamp = (dn < dmin) & (fr > f_lift) & (fr < f_land)
    path[clamp] = hipj[clamp] + dvec[clamp] / dn[clamp, None] * dmin
    rep['fold_clamp'] = dict(frames=[int(f) for f in fr[clamp]], dmin_cm=round(dmin * 100, 1))
    vknee = (PnL[nmapL, npcL.ix[sw + 'Leg']] - PnL[nmapL, npcL.ix[sw + 'UpLeg']]) * sc
    pole_seat = hipj + vknee
    post = fr >= f_land
    wK = ramp(T, f_land - 4, f_land)[:, None]
    pole_seat = (1 - wK) * pole_seat + wK * np.repeat(Pi[-1:, kn_s], T, 0)   # eased into the installed clip's FINAL knee (a fixed forward-outward point: stable for a nearly straight landing leg)
    pole_seat0 = pole_seat.copy()
    pole_w = np.where(post, 1.0, ramp(T, f_lift + 2, f_lift + 5))        # the donor's knee direction through the whole swing (so a knee-pole lift acts before the crossing too)
    wB = ramp(T, f_land - a.land_rot_frames, f_land)[:, None]
    Rank = S.q2m(S.qcontinuous(G.slerp(S.qcontinuous(S.m2q(Wr_[:, fo_s])), S.qcontinuous(S.m2q(inst.Wr[:, fo_s])), wB)))
    lr[:, to_s] = G.slerp(S.qcontinuous(lr[:, to_s]), S.qcontinuous(inst.lr[:, to_s]), wB)
    # the final spot must be reachable from the hip joint when the foot is pinned there (a foot landing on the GROUND waits for the hips to come down)
    reach = a.max_extend * (la + lb) * 0.97
    far = np.linalg.norm(peg - hipj, axis=1) > reach
    while f_land < f_pin_end - 3 and far[f_land]:
        f_land += 1
    f_land = int(f_land)
    air = (fr > f_lift) & (fr < f_land - 2)
    wPeg = ramp(T, f_land - 5, f_land)[:, None]
    path = (1 - wPath) * np.repeat(P_cur[f_lift:f_lift + 1], T, 0) + wPath * (hips_seat + rel)
    path = (1 - wPeg) * path + wPeg * peg
    if a.path_lift > 0:
        u_ = np.clip((fr - (f_lift + 1)) / float(max(f_land - 2 - (f_lift + 1), 1)), 0.0, 1.0)
        path[:, 2] += a.path_lift * np.sin(np.pi * u_) * ((fr > f_lift) & (fr < f_land - 1))
    dvec = path - hipj
    dn = np.linalg.norm(dvec, axis=1)
    clamp = (dn < dmin) & (fr > f_lift) & (fr < f_land)
    path[clamp] = hipj[clamp] + dvec[clamp] / dn[clamp, None] * dmin
    path0 = path.copy()
    wIK = ((fr > f_lift) & (fr <= f_pin_end)).astype(float)
    post = fr >= f_land
    tgtW = inst.to_world(path)
    tgtW[post] = inst.Wt[-1, fo_s]
    model = None if a.no_mesh else BM.BikeModel.for_set(a.set)
    rep['keys'] = dict(f_lift=f_lift, f_cross=f_cross, f_land=f_land, f_pin_end=f_pin_end, f_inst=f_inst)
    base_lt, base_lr = lt.copy(), lr.copy()
    pushes = []
    disp_total = np.zeros((T, 3))
    knee_total = np.zeros((T, 3))
    up = np.array([0.0, 0.0, 1.0])
    for it in range(max(1, a.iters)):
        lt[:], lr[:] = base_lt, base_lr
        IK.set_world_rot(sk, lt, lr, fo_s, Rank)
        err = IK.two_bone(sk, lt, lr, up_s, kn_s, fo_s, tgtW, weight=wIK, max_extend=a.max_extend, bend_sigma=a.bend_sigma, pole=inst.to_world(pole_seat), pole_weight=pole_w)
        if model is None:
            break
        P = inst.seat(sk.world(lt, lr)[0])
        dn = np.linalg.norm(path - hipj, axis=1)
        rows = BM.clearance_report(model, P, ix, sw, frames=[int(f) for f in fr[air]], thigh=False)
        bad = [(r['frame'], a.margin * 100 - r['clear_cm'], r['seg'], r['u'], np.asarray(r['nrm'])) for r in rows if r['clear_cm'] < a.margin * 100 - 0.05]
        if not bad:
            break
        # push the ankle target away from the nearest surface (outward face normal blended with up), gaussian-spread over the neighbours
        # (max over frames: no kink); a deficit at the knee end of the shin also moves the knee pole the same way
        disp, kdisp = np.zeros((T, 3)), np.zeros((T, 3))
        amp, kamp = np.zeros(T), np.zeros(T)
        for f, d, seg, u, nrm in bad:
            g = np.exp(-0.5 * ((fr - f) / 1.5) ** 2)
            if seg.startswith(sw + 'Leg') and u < 0.3:
                # the knee itself (the Arch: the knee sweeps over the tank / seat front): lift the knee pole, up with a little of the surface normal
                outb = np.array([-sx, 0.0, 0.0])                      # the rider's standing side
                kdir = (0.6 * up + 0.4 * outb) if f < f_cross else (0.8 * up + 0.2 * np.array([nrm[0], nrm[1], 0.0]))
                kdir /= max(np.linalg.norm(kdir), 1e-9)
                ks = g * 3.0 * (d + 1.0) > kamp
                kdisp[ks] = kdir * (3.0 * (d + 1.0) * g[ks])[:, None]; kamp[ks] = 3.0 * (d + 1.0) * g[ks]
                adir = (0.6 * up + 0.4 * outb) if f < f_cross else (0.75 * up + 0.25 * nrm)
                adir /= max(np.linalg.norm(adir), 1e-9)
                gain = 1.5 if dn[f] > 0.45 * (la + lb) else 0.5    # a hanging (long) leg: a pole cannot lift the knee along the leg axis, the foot has to come up (fold)
                sel = g * gain * d > amp
                disp[sel] = adir * (gain * d * g[sel])[:, None]; amp[sel] = gain * d * g[sel]
                continue
            dirv = 0.7 * nrm + 0.3 * up
            dirv /= max(np.linalg.norm(dirv), 1e-9)
            sel = g * (d + 0.5) > amp
            disp[sel] = dirv * ((d + 0.5) * g[sel])[:, None]; amp[sel] = (d + 0.5) * g[sel]
        disp[~air] = 0.0; kdisp[~air] = 0.0
        disp = IK.smooth_rows(disp, 1.0); kdisp = IK.smooth_rows(kdisp, 1.0)       # a smooth field in time (the normals differ frame to frame)
        disp[~air] = 0.0; kdisp[~air] = 0.0
        room = np.maximum(a.max_bump * 100 - np.linalg.norm(disp_total, axis=1), 0.0)
        scale = np.minimum(1.0, room / np.maximum(np.linalg.norm(disp, axis=1), 1e-9))
        disp *= scale[:, None]
        kroom = np.maximum(3.0 * a.max_bump * 100 - np.linalg.norm(knee_total, axis=1), 0.0)
        kscale = np.minimum(1.0, kroom / np.maximum(np.linalg.norm(kdisp, axis=1), 1e-9))
        kdisp *= kscale[:, None]
        disp_total += disp; knee_total += kdisp
        path = path0 + IK.smooth_rows(disp_total, 0.8) / 100.0
        pole_seat = pole_seat0 + IK.smooth_rows(knee_total, 0.8) / 100.0
        # fold cap again (a push can bring the target back toward the hip joint), as a smooth field
        dvec = path - hipj
        dn = np.linalg.norm(dvec, axis=1)
        clamp = (dn < dmin) & (fr > f_lift) & (fr < f_land)
        corr = np.zeros((T, 3))
        corr[clamp] = dvec[clamp] / dn[clamp, None] * dmin - dvec[clamp]
        path += IK.smooth_rows(corr, 0.8)
        tgtW = inst.to_world(path)
        tgtW[post] = inst.Wt[-1, fo_s]
        pushes.append(dict(it=it, frames=[int(f) for f, _, _, _, _ in bad], max_deficit_cm=round(max(d for _, d, _, _, _ in bad), 2), knee_moved=bool(np.linalg.norm(kdisp, axis=1).max() > 0)))
        if np.linalg.norm(disp, axis=1).max() <= 1e-6 and np.linalg.norm(kdisp, axis=1).max() <= 1e-6:
            break
    # landed: hold the landed pose, ease into the installed leg once the installed foot is on its peg, the ankle pinned at the final spot meanwhile
    land_lr = lr.copy()
    f_blend0 = int(max(f_land, f_inst - 3))
    wLand = ramp(T, f_blend0, f_pin_end)[:, None]
    for n in (sw + 'UpLeg', sw + 'Leg'):
        j = ix[n]
        held = np.repeat(land_lr[f_land:f_land + 1, j], T, 0)
        q = G.slerp(held, inst.lr[:, j], wLand)
        post = fr >= f_land
        lr[post, j] = q[post]
    lr = S.qcontinuous(lr)
    IK.set_world_rot(sk, lt, lr, fo_s, Rank)
    err2 = IK.two_bone(sk, lt, lr, up_s, kn_s, fo_s, tgtW, weight=wIK * (fr >= f_land), max_extend=a.max_extend, bend_sigma=2.0, pole=inst.to_world(pole_seat), pole_weight=pole_w * (fr >= f_land))
    rep['clearance_iterations'] = pushes
    rep['path_raise_max_cm'] = round(float(np.linalg.norm(disp_total, axis=1).max()), 1)
    rep['knee_raise_max_cm'] = round(float(np.linalg.norm(knee_total, axis=1).max()), 1)
    rep['swing_ik_err_cm'] = dict(air=round(err * 100, 2), landed=round(err2 * 100, 2))

    # 5. helpers on the new limbs (every frame), vanilla offsets of the FPP clip (anchor-relative: frame independent)
    ES.reanchor_helpers(sk, lt, lr, fpp.Wt, fpp.Wr, -1, T)
    fixed = []
    for sd_ in ('left', 'right'):
        for nm_, src_ in (('arm_%s_ik_offset' % sd_, fpp), ('arm_%s_ik_weapon' % sd_, inst)):
            if nm_ in ix:
                j_ = ix[nm_]
                lt[:, j_], lr[:, j_] = src_.lt[:, j_], src_.lr[:, j_]
                fixed.append(nm_)
    lr = S.qcontinuous(lr)
    keep = [ix[n] for n in ('Root', 'Trajectory') if n in ix]
    mv = np.ones(len(sk.names), bool); mv[keep] = False
    lt[0, mv], lr[0, mv] = lt[1, mv], lr[1, mv]        # static first frame (the entry gate f: no bone step at the endpoints)
    rep['arm_ik_chain_vanilla_locals'] = dict(bones=fixed, note='offset = FPP vanilla (identity), weapon = installed (grip offsets eased at both ends)')

    # 6. events
    extras = copy.deepcopy(inst.extras)
    extras['fallbackFrameIndices'] = []
    ls, rs = B.plant_frames(sk, lt, lr, 'LeftFoot'), B.plant_frames(sk, lt, lr, 'RightFoot')
    evs = [dict(e) for e in (extras.get('animEvents') or []) if e['type'] != 'FootPlant']
    tmpl = next((e for e in (inst.extras.get('animEvents') or []) if e['type'] == 'FootPlant'), None)
    if tmpl:
        for side_, starts in (('Left', ls), ('Right', rs)):
            for f in starts:
                if f == 0:
                    continue
                e = dict(tmpl)
                e['eventName'] = 'leftFootOnGround' if side_ == 'Left' else 'rightFootOnGround'
                e['side'] = side_
                e['startFrame'] = int(f)
                evs.append(e)
    extras['animEvents'] = sorted(evs, key=lambda e: e['startFrame'])
    rep['events'] = [(e['startFrame'], e['type'], e.get('eventName')) for e in extras['animEvents']]
    rep['plants'] = dict(left=ls, right=rs)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    B.write_out(sk, a.out, inst.clip, inst.t, lt, lr, extras)

    # 7. diagnostics
    Wt, Wr = sk.world(lt, lr)
    P = inst.seat(Wt)
    diag = {}
    if model is not None:
        for leg, lab in ((sw, 'swing'), (st, 'stance')):
            rows_after = BM.clearance_report(model, P, ix, leg, thigh=False)
            cr_after = BM.sweep_crossings(model, P, ix, leg, thigh=False)
            rows_before = BM.clearance_report(model, Pi, ix, leg, thigh=False)
            cr_before = BM.sweep_crossings(model, Pi, ix, leg, thigh=False)
            wa = min(rows_after, key=lambda r: r['clear_cm']); wb = min(rows_before, key=lambda r: r['clear_cm'])
            win = [r for r in rows_after if r['frame'] < f_land] if lab == 'swing' else [r for r in rows_after if r['frame'] <= fs0]
            wwin = min(win, key=lambda r: r['clear_cm'])
            diag[lab] = dict(before=dict(min_clear_cm=wb['clear_cm'], frame=wb['frame'], part=wb['part'], crossings=int(sum(cr_before))),
                             after=dict(min_clear_cm=wa['clear_cm'], frame=wa['frame'], part=wa['part'], seg=wa['seg'], crossings=int(sum(cr_after)),
                                        crossings_air=int(sum(c for f, c in zip(range(T), cr_after) if (f < f_land if lab == 'swing' else f <= fs0))),
                                        min_clear_air_cm=wwin['clear_cm'], frame_air=wwin['frame']),
                             after_per_frame=[(r['frame'], r['clear_cm'], r['part'].split('_01')[0], int(c)) for r, c in zip(rows_after, cr_after)])
    kf = knee_flexion_deg(P, ix, sw)
    diag['swing_knee_flexion_max_deg'] = round(float(kf.max()), 1)
    diag['swing_knee_flexion_at_cross_deg'] = round(float(kf[f_cross]), 1)
    steps = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[1:] * lr[:-1], axis=-1)), 0, 1)))
    core = [i for i, n in enumerate(sk.names) if n in npc.ix and n not in SKIP]
    j = int(np.argmax(steps[:, core].max(0)))
    diag['max_rot_step_deg'] = dict(deg=round(float(steps[:, core].max()), 2), bone=sk.names[core[j]], frame=int(np.argmax(steps[:, core[j]])) + 1)
    J = [ix[n] for n in ACCEL_JOINTS]
    Pc = (Wt[:, J] - Wt[:, ri][:, None]) * 100
    acc = np.linalg.norm(Pc[2:] - 2 * Pc[1:-1] + Pc[:-2], axis=-1)
    f_, j_ = np.unravel_index(int(np.argmax(acc)), acc.shape)
    diag['joint_accel_max_cm'] = dict(cm=round(float(acc.max()), 2), joint=ACCEL_JOINTS[j_], frame=int(f_) + 1)
    hel = {}
    for sd in ('Left', 'Right'):
        lo = sd.lower()
        for src, names in ((sd + 'Hand', ['arm_%s_ik' % lo, 'arm_%s_ik_offset' % lo, 'Torso_%sArm_Hand_IK_JNT' % sd, 'Torso_World_%sArm_Hand_IK_JNT' % sd]),
                           (sd + 'Foot', ['leg_%s_ik' % lo, 'Torso_%sLeg_Foot_IK_JNT' % sd, 'Torso_World_%sLeg_Foot_IK_JNT' % sd])):
            for h in names:
                if h in ix:
                    hel[h] = round(float(np.linalg.norm(Wt[:, ix[h]] - Wt[:, ix[src]], axis=1).max() * 100), 2)
    diag['helpers_max_cm'] = hel
    diag['helpers_ok'] = bool(max(hel.values()) < 1.0)
    diag['poles'] = {sd: pole_side(Wt, ix, sd) for sd in ('Left', 'Right')}
    core_j = [ix[n] for n in npc.ix if n in ix]
    e = np.linalg.norm(Wt[-1, core_j] - inst.Wt[-1, core_j], axis=1) * 100
    diag['end_pose_err_cm'] = dict(cm=round(float(e.max()), 2), bone=sk.names[core_j[int(np.argmax(e))]])
    diag['hips_seat_cm'] = [np.round(P[f, hip] * 100, 1).tolist() for f in range(0, T, 3)]
    diag['swing_foot_seat_cm'] = [np.round(P[f, fo_s] * 100, 1).tolist() for f in range(0, T, 3)]
    diag['swing_knee_seat_cm'] = [np.round(P[f, kn_s] * 100, 1).tolist() for f in range(0, T, 3)]
    rep['diag'] = diag
    rep['out'] = a.out
    if a.report:
        json.dump(rep, open(a.report, 'w', encoding='utf-8'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
    swd = diag.get('swing', {}).get('after', {})
    std = diag.get('stance', {}).get('after', {})
    print('MOUNTFIX %s side=%s k=%d lift=%d cross=%d land=%d inst=%d raise=%.1f | swing air clear %.1f cm (f%s) crossings air %s all %s | stance crossings air %s all %s | knee max %.0f cross %.0f | step %.1f deg (%s f%d) accel %.1f (%s) | helpers %.2f cm | end err %.2f cm (%s) | %s' % (
        a.set, a.side, k, f_lift, f_cross, f_land, f_inst, rep['path_raise_max_cm'], swd.get('min_clear_air_cm', float('nan')), swd.get('frame_air', '-'), swd.get('crossings_air', '-'), swd.get('crossings', '-'),
        std.get('crossings_air', '-'), std.get('crossings', '-'), diag['swing_knee_flexion_max_deg'], diag['swing_knee_flexion_at_cross_deg'], diag['max_rot_step_deg']['deg'], diag['max_rot_step_deg']['bone'], diag['max_rot_step_deg']['frame'],
        diag['joint_accel_max_cm']['cm'], diag['joint_accel_max_cm']['joint'], max(hel.values()), diag['end_pose_err_cm']['cm'], diag['end_pose_err_cm']['bone'], a.out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
