"""Offline check of the UTPP bike entry clips (build_bike.ps1): the last frame of enter_l / enter_r must equal frame 0 of the TPP
pose the game continues with, the clip must not be the collapsed FPP seat pose, and the motion must meet the continuity limits
of diag 2026-10-28 c9 (WP3).

  python -I check_bike_entry.py --src-rig <wa_sportbike.anims.glb> --enter-l <enter_l.anims.glb> --enter-r <enter_r.anims.glb> --tpp <tpp_pwa_sportbike.anims.glb>
         [--pose-clip-l idle_stand_bike_begin_l] [--pose-clip-r idle_stand_bike_begin_l] [--build-report-l r.json --build-report-r r.json]
         [--fpp <pwa_sportbike.anims.glb>] [--against-fpp] [--selftest-old] [--report out.json]

Checks (exit 0 = all pass), bones compared relative to Root (Root excluded):
  (a) duration 1.4 s (the vanilla clip's PSM timing), 43 frames at 30 fps;
  (b) last frame vs the TPP pose: every non-helper bone local rotation < 3 deg and world position (Root-relative) < 2 cm;
  (c) last 0.4 s: Hips height >= TPP Hips height - 3 cm and Spine3 pitch within 25 deg of the TPP Spine3 pitch (10 deg in the last 0.2 s) (the vanilla FPP
      enter_l ends ~9 cm lower with the Spine3 ~28 deg more pitched: that is the collapsed pose this clip replaces);
  (d) the local rotation of every retargeted bone changes by <= --max-step-deg (15) per frame (30 fps) over the whole clip;
  (e) root yaw: max |yaw(t) - yaw(0)| <= 15 deg (the engine slides the entity to seat * inverse(root delta) before the clip plays,
      so the total root yaw is the angle V is turned away from the bike during that slide);
  (f) endpoint steps: every bone except Root / Trajectory / reference_joint (helpers included) changes its local rotation by
      < 1 deg between frames 0 and 1 and between the last two frames (no velocity at the hand-over in either direction);
  (g) end feet: LeftFoot and RightFoot of the last frame within 1 cm (Root-relative) of the TPP pose;
  (h) root speed: |v_i - v_(i-1)| <= 20 % of max(v_i, v_(i-1), 1 cm/frame) for consecutive root segment speeds (cm/frame, 30 fps),
      entity at rest before and after the clip. The 1 cm/frame floor makes the rule defined at rest (below it the speed may
      change by 0.2 cm/frame per frame); the c8 build's mid-climb kink 3.30 -> 6.79 cm/frame fails it;
  (i) joint acceleration: second difference of the Root-frame positions of Hips, Spine3, Head, hands, feet, forearms and lower
      legs <= --max-joint-accel-cm (default 16 cm/frame^2 at 30 fps = the vanilla FPP enter clips' maximum, decision 2026-10-28;
      the 2 cm/frame^2 of the c9 review is not reachable for a ~2 m leg swing over the seat in 1.4 s);
  (j) planted feet (needs --build-report-*): the builder's foot_slide_planted_final_max_m <= 1 cm/frame, measured on the final
      pose over the builder's own plant intervals (--plant-hold keyframes; re-plant steps and plants shortened for reach are
      swings there). The slide over the frames the source foot is planted (foot_slide_source_planted_final_max_m) is reported.
  (k) limb IK helpers (the game's player leg IK, and the arm IK while the hands are in world space, pull the limbs onto them;
      vanilla: on the bone <= 0.2 cm feet / <= 0.84 cm hands): leg_*_ik, Torso_*Leg_Foot_IK_JNT, Torso_World_*Leg_Foot_IK_JNT
      within 0.5 cm and 1 deg of the foot; arm_*_ik, arm_*_ik_offset (2026-11-09), Torso_*Arm_Hand_IK_JNT, Torso_World_*Arm_Hand_IK_JNT within 1.5 cm of the
      hand; Torso_*Leg_PoleVector_JNT in the knee plane (UpLeg, Leg, Foot) within 2 deg on every bent frame (knee >= 2 cm
      off the hip-foot line), every frame. Arm pole planes are reported.
  (m) foot contact on the FINAL clip (opt-in, needs --ref-l / --ref-r = the shipped clip of the same set; world motion incl. root, bike_steps_gen rules; added 2026-11-07 after the c9 clips passed (j) but
      skated: (j) reads the builder's own numbers, it cannot see a foot that drags 1-3 cm above the floor):
      m_glide: foot glide (contact + vertical-speed rule, floors of the shipped clip) <= --max-glide-cm (6, the exit gate) AND not above the shipped clip;
      m_scrape: frames where an ankle moves > 2 cm / frame while the lower of ankle / toe is < 2.5 cm above its local ground (lowest height inside +-8 frames): <= --max-scrape (6 = median of the 12 vanilla FPP p?a_sportbike enter clips measured with this rule: 1 4 4 5 6 6 7 8 9 10 11 15; the 12 shipped c9 clips have 9..19) and not above the shipped clip;
      m_hop: frames where BOTH feet are > 6 cm above their local ground <= shipped + 3 (a lifted support foot reads as a hop);
      m_clearance: bike_steps_gen.clearance in the SEAT frame (Root of the last frame; inward / downward <= 5 cm, hands <= 6 cm against the shipped path of joints next to the bike centre line: the proxy of the exit gates; no bike mesh offline);
      j_final: with --build-report-*: the planted intervals of the builder report re-measured on the final clip (world ankle speed <= --max-planted-slide-m per frame; (j) itself reads the report's numbers).
  Reported only: max Root-frame joint speed in [1.0, 1.2] s (the PSM mount->ride switch at t ~ 1.10 stalls one frame).
  Pose reference: idle_stand_bike_begin_l for both clips (left foot on the ground, right foot on the peg). The player graph
  base\\gameplay\\anim_graphs\\player_vehicle.animgraph chooses idle_stand_bike_begin / _begin_l with an animAnimNode_Switch whose
  latched weight is (vehicleRoll < -1) or (vehicleRoll < 1 and inputLR < -0.3) - not the entry side; in-game traces of
  2026-10-28 (female V sportbike1) show _begin_l after enter_l and after enter_r.
  Bones compared in (b)/(d) = the retargeted ones (present in the NPC wa_sportbike rig); the helper bones (arms_*_space, IK
  targets) are re-anchored by build_entry_clip.py.
--against-fpp compares (b)/(c) with the vanilla FPP clip's last frame instead (must FAIL: the self-test / mutation of the check).
--selftest-old: exit 0 only if every check (e)..(h) and (k) FAILS on both clips (run it on a pre-c9 build: proves the checks are not vacuous);
  (i) at 16 does not separate the pre-c9 clips (7.4-15.0) and is only reported there; the c8 mid-climb kink is caught by (h).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402

FPS = 30.0
SKIP = ('Root', 'reference_joint', 'Trajectory')
ACCEL_JOINTS = ['Hips', 'Spine3', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot', 'LeftForeArm', 'RightForeArm', 'LeftLeg',
                'RightLeg']
NEW_CHECKS = ('e_root_yaw_ok', 'f_endpoint_steps_ok', 'g_end_feet_ok', 'h_root_speed_ok', 'i_joint_accel_ok', 'k_limb_ik_helpers_ok')
SELFTEST_OLD_CHECKS = NEW_CHECKS[:4] + ('k_limb_ik_helpers_ok',)


def pose_world(sk, lt, lr):
    lt = lt.copy()
    lr = lr.copy()
    ri = sk.index['Root']
    lt[:, ri] = 0.0
    lr[:, ri] = np.array([0.0, 0.0, 0.0, 1.0])
    Wt, Wr = sk.world(lt, lr)
    return Wt, Wr


def qang(a, b):
    d = np.abs(np.sum(a * b, axis=-1)).clip(0, 1)
    return np.degrees(2 * np.arccos(d))


def pitch_deg(Wr, j):
    """angle between the bone's local +Y axis (spine direction in the glTF rig) and the world up axis"""
    ax = Wr[:, j][..., :, 1]
    return np.degrees(np.arccos(np.clip(ax[..., 1], -1, 1)))


def plane_dev_deg(A, K, F, P):
    """Angle (deg) between the bend direction of the chain A-K-F and the pole P around the A-F axis; bent = knee >= 2 cm off."""
    u = (F - A) / np.linalg.norm(F - A, axis=-1, keepdims=True)
    kp = (K - A) - np.sum((K - A) * u, -1, keepdims=True) * u
    pp = (P - A) - np.sum((P - A) * u, -1, keepdims=True) * u
    c = np.sum(kp * pp, -1) / (np.linalg.norm(kp, axis=-1) * np.linalg.norm(pp, axis=-1) + 1e-12)
    return np.degrees(np.arccos(np.clip(c, -1, 1))), np.linalg.norm(kp, axis=-1) >= 0.02


def limb_ik_checks(sk, lt, lr, foot_m=0.005, foot_deg=1.0, hand_m=0.015, pole_deg=2.0):   # hand 1.5 cm since 2026-11-09: the vanilla pma_sportbike2 FPP enter_l has Torso_LeftArm_Hand_IK_JNT 1.0 cm off the hand
    """(k) on one clip (Root-relative world pose)."""
    W, R = pose_world(sk, lt, lr)
    I = sk.index
    res, ok = {}, True
    for s in ('Left', 'Right'):
        lo = s.lower()
        f, h = I[s + 'Foot'], I[s + 'Hand']
        for n in ('leg_%s_ik' % lo, 'Torso_%sLeg_Foot_IK_JNT' % s, 'Torso_World_%sLeg_Foot_IK_JNT' % s):
            dp = float(np.linalg.norm(W[:, I[n]] - W[:, f], axis=-1).max())
            Rr = np.einsum('tji,tjk->tik', R[:, f], R[:, I[n]])
            da = float(np.degrees(np.arccos(np.clip((np.trace(Rr, axis1=1, axis2=2) - 1) / 2, -1, 1))).max())
            res[n] = dict(cm=round(dp * 100, 3), deg=round(da, 2))
            ok = ok and dp < foot_m and da < foot_deg
        for n in ('arm_%s_ik' % lo, 'arm_%s_ik_offset' % lo, 'Torso_%sArm_Hand_IK_JNT' % s, 'Torso_World_%sArm_Hand_IK_JNT' % s):   # arm_*_ik_offset added 2026-11-09 (bike12/13: a stale offset glued the hand to the hip in game)
            dp = float(np.linalg.norm(W[:, I[n]] - W[:, h], axis=-1).max())
            res[n] = dict(cm=round(dp * 100, 3))
            ok = ok and dp < hand_m
        for limb, (a_, k_, e_) in (('Leg', (s + 'UpLeg', s + 'Leg', s + 'Foot')), ('Arm', (s + 'Arm', s + 'ForeArm', s + 'Hand'))):
            n = 'Torso_%s%s_PoleVector_JNT' % (s, limb)
            ang, bent = plane_dev_deg(W[:, I[a_]], W[:, I[k_]], W[:, I[e_]], W[:, I[n]])
            mx = float(ang[bent].max()) if bent.any() else 0.0
            res[n] = dict(plane_deg=round(mx, 2), bent_frames=int(bent.sum()))
            if limb == 'Leg':
                ok = ok and mx < pole_deg
    return {'k_limb_ik_helpers': res, 'k_limb_ik_helpers_ok': bool(ok)}


def motion_checks(sk, t, lt, lr, max_yaw, step_lim, feet_lim_m, accel_lim, ref_feet):
    """(e)..(i) on one clip (sampled at 30 fps). ref_feet: Root-relative LeftFoot / RightFoot of the pose reference."""
    res = {}
    ri = sk.index['Root']
    yaw = np.degrees(np.unwrap(np.radians(B.yaw_of(lr[:, ri]))))
    res['e_root_yaw_total_deg'] = float(yaw[-1] - yaw[0])
    res['e_root_yaw_max_deg'] = float(np.abs(yaw - yaw[0]).max())
    res['e_root_yaw_ok'] = bool(res['e_root_yaw_max_deg'] <= max_yaw)
    steps = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[1:] * lr[:-1], axis=-1)), 0, 1)))  # (T-1, J)
    bones = [i for i, n in enumerate(sk.names) if n not in SKIP]
    s0, s1 = steps[0, bones], steps[-1, bones]
    res.update(f_first_step_max_deg=float(s0.max()), f_first_step_bone=sk.names[bones[int(np.argmax(s0))]],
               f_last_step_max_deg=float(s1.max()), f_last_step_bone=sk.names[bones[int(np.argmax(s1))]])
    res['f_endpoint_steps_ok'] = bool(s0.max() < step_lim and s1.max() < step_lim)
    Wt, Wr = pose_world(sk, lt, lr)
    fe = {k: float(np.linalg.norm(Wt[-1, sk.index[k]] - ref_feet[k])) * 100 for k in ('LeftFoot', 'RightFoot')}
    res['g_end_feet_err_cm'] = fe
    res['g_end_feet_ok'] = bool(max(fe.values()) < feet_lim_m * 100)
    v = np.linalg.norm(np.diff(lt[:, ri], axis=0), axis=-1) * 100
    ex = B.speed_rule_violations(v)
    k = int(np.argmax(ex))
    vv = np.r_[0.0, v, 0.0]
    res.update(h_root_speed_max_cm=float(v.max()), h_root_speed_excess_cm=float(ex.max()),
               h_root_speed_worst=dict(frame=k, from_cm=float(vv[k]), to_cm=float(vv[k + 1])))
    res['h_root_speed_ok'] = bool(ex.max() <= 1e-3)
    J = [sk.index[j] for j in ACCEL_JOINTS]
    P = Wt[:, J] * 100  # Root frame (Root zeroed), cm
    acc = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=-1)
    f, j = np.unravel_index(int(np.argmax(acc)), acc.shape)
    res.update(i_joint_accel_max_cm=float(acc.max()), i_joint_accel_worst=dict(frame=int(f) + 1, t=round(float(t[f + 1]), 3), joint=ACCEL_JOINTS[j]),
               i_joint_accel_per_joint=dict(zip(ACCEL_JOINTS, np.round(acc.max(0), 2).tolist())))
    res['i_joint_accel_ok'] = bool(acc.max() <= accel_lim)
    sp = np.linalg.norm(np.diff(P, axis=0), axis=-1)  # (T-1, J), segment i between frames i and i+1
    tm = (t[1:] + t[:-1]) / 2
    win = (tm >= 1.0 - 1e-6) & (tm <= 1.2 + 1e-6)
    res['psm_switch_joint_speed_max_cm'] = float(sp[win].max()) if win.any() else None
    return res


def contact_gates(path, ref, cn, plants, a):
    """(m) gates on one final clip against the shipped clip; plants = builder report plant_segments (or None)"""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'ai_motion'))
    import bike_steps_gen as BSG
    Wc, names = BSG.load('%s:%s' % (path, cn))
    Wr, _ = BSG.load('%s:%s' % (ref, cn))
    fl = BSG.floors_of(Wr, names)
    gc, gr = BSG.glide(Wc, names, None, fl), BSG.glide(Wr, names, None, fl)

    def local(W, side):
        ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
        za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
        lf = lambda z: np.array([z[max(0, f - 8):f + 9].min() for f in range(len(z))])
        return np.minimum(za - lf(za), zt - lf(zt)) * 100

    def scrape(W):
        n = 0
        for sd in ('Left', 'Right'):
            ja = names.index(sd + 'Foot')
            v = np.r_[0.0, np.linalg.norm(np.diff(W[:, ja, :2, 3], axis=0), axis=1)] * 100
            n += int(((v > 2.0) & (local(W, sd) < 2.5)).sum())
        return n

    def hop(W):
        return int(((local(W, 'Left') > 6.0) & (local(W, 'Right') > 6.0)).sum())
    res = dict(m_glide_cm=gc['glide_total_cm'], m_glide_ref_cm=gr['glide_total_cm'], m_scrape=scrape(Wc), m_scrape_ref=scrape(Wr), m_hop=hop(Wc), m_hop_ref=hop(Wr))
    res['m_glide_ok'] = bool(gc['glide_total_cm'] <= a.max_glide_cm and gc['glide_total_cm'] <= gr['glide_total_cm'])
    res['m_scrape_ok'] = bool(res['m_scrape'] <= a.max_scrape and res['m_scrape'] <= res['m_scrape_ref'])
    res['m_hop_ok'] = bool(res['m_hop'] <= res['m_hop_ref'] + 3)
    Ms = np.linalg.inv(Wr[-1, 0])
    cl = BSG.clearance(np.einsum('ij,tnjk->tnik', Ms, Wr), np.einsum('ij,tnjk->tnik', np.linalg.inv(Wc[-1, 0]), Wc), names)
    res['m_clearance'] = cl
    res['m_clearance_ok'] = bool(cl['ok'])
    if plants:
        worst = 0.0
        for sd in ('Left', 'Right'):
            ja = names.index(sd + 'Foot')
            for lo, hi in plants[sd + 'Foot']['plants']:
                for f in range(max(lo + 1, 1), hi + 1):   # speed f-1 -> f inside the plant interval (lo itself is the landing frame)
                    worst = max(worst, float(np.linalg.norm((Wc[f, ja, :2, 3] - Wc[f - 1, ja, :2, 3]))))
        res['j_final_planted_slide_cm'] = round(worst * 100, 3)
        res['j_final_ok'] = bool(worst <= a.max_planted_slide_m)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--enter-l', required=True)
    ap.add_argument('--enter-r', required=True)
    ap.add_argument('--tpp', required=True)
    ap.add_argument('--src-rig', required=True, help='the NPC wa_sportbike glb: its bones are the retargeted (non-helper) ones')
    ap.add_argument('--pose-clip-l', default='idle_stand_bike_begin_l')
    ap.add_argument('--pose-clip-r', default='idle_stand_bike_begin_l')
    ap.add_argument('--build-report-l')
    ap.add_argument('--build-report-r')
    ap.add_argument('--fpp')
    ap.add_argument('--against-fpp', action='store_true')
    ap.add_argument('--selftest-old', action='store_true')
    ap.add_argument('--expect-fail', help='comma list of check keys (e.g. k_limb_ik_helpers_ok): exit 0 only if each fails on both clips')
    ap.add_argument('--max-step-deg', type=float, default=15.0, help='max local rotation step per frame (30 fps), any retargeted bone')
    ap.add_argument('--max-root-yaw-deg', type=float, default=15.0)
    ap.add_argument('--endpoint-step-deg', type=float, default=1.0)
    ap.add_argument('--end-feet-m', type=float, default=0.01)
    ap.add_argument('--max-joint-accel-cm', type=float, default=16.0)
    ap.add_argument('--max-planted-slide-m', type=float, default=0.01)
    ap.add_argument('--ref-l', help='(m) shipped enter_l clip glb (floors / baseline of the contact gates); with --ref-r turns the (m) gates on')
    ap.add_argument('--ref-r')
    ap.add_argument('--max-glide-cm', type=float, default=6.0)
    ap.add_argument('--max-scrape', type=int, default=6, help='(m_scrape) frames; 6 = the median of the 12 vanilla FPP p?a_sportbike enter clips measured with the same rule (1 4 4 5 6 6 7 8 9 10 11 15 on 2026-11-07)')
    ap.add_argument('--report')
    a = ap.parse_args()
    tpp = S.Skeleton.from_glb(a.tpp)
    src_names = set(S.Skeleton.from_glb(a.src_rig).names)
    core = [n for n in tpp.names if n in src_names and n not in SKIP]
    rep = {'clips': {}}
    ok_all = True
    old_all_fail = True
    for tag, path, cn, pclip, brep in (('enter_l', a.enter_l, 'enter_l', a.pose_clip_l, a.build_report_l),
                                       ('enter_r', a.enter_r, 'enter_r', a.pose_clip_r, a.build_report_r)):
        pc = tpp.clip(pclip)
        _, pt = B.frames_of(tpp, pc)
        plt_, plr_ = tpp.sample_local(pc, pt[:1])
        ref_sk, ref_lt, ref_lr, label = tpp, plt_, plr_, pclip
        if a.against_fpp:
            fpp = S.Skeleton.from_glb(a.fpp)
            fc = fpp.clip(cn)
            _, ft = B.frames_of(fpp, fc)
            ref_sk = fpp
            ref_lt, ref_lr = fpp.sample_local(fc, ft[-1:])
            label = 'FPP %s last frame' % cn
        rWt, rWr = pose_world(ref_sk, ref_lt, ref_lr)
        tWt, _ = pose_world(tpp, plt_, plr_)
        ref_feet = {k: tWt[0, tpp.index[k]] for k in ('LeftFoot', 'RightFoot')}
        sk = S.Skeleton.from_glb(path)
        clip = sk.clip(cn)
        dur, t = B.frames_of(sk, clip)
        lt, lr = sk.sample_local(clip, t)
        lr = S.qcontinuous(lr)
        Wt, Wr = pose_world(sk, lt, lr)
        res = {'pose': label, 'duration_s': float(dur), 'frames': int(len(t))}
        res['a_duration_ok'] = abs(dur - 1.4) < 1e-3 and len(t) == 43
        pos_max, rot_max, pos_bone, rot_bone = 0.0, 0.0, '', ''
        for n in core:
            if n not in sk.index or n not in ref_sk.index:
                continue
            i, j = sk.index[n], ref_sk.index[n]
            dp = float(np.linalg.norm(Wt[-1, i] - rWt[0, j]))
            dr = float(qang(lr[-1, i], ref_lr[0, j]))
            if dp > pos_max:
                pos_max, pos_bone = dp, n
            if dr > rot_max:
                rot_max, rot_bone = dr, n
        res.update(b_pos_max_cm=pos_max * 100, b_pos_bone=pos_bone, b_rot_max_deg=rot_max, b_rot_bone=rot_bone)
        res['b_end_pose_ok'] = pos_max < 0.02 and rot_max < 3.0
        hi, si = sk.index['Hips'], sk.index['Spine3']
        last = t >= t[-1] - 0.4 - 1e-6
        hips_h = Wt[last, hi, 1]
        ref_h = float(rWt[0, ref_sk.index['Hips'], 1])
        p = pitch_deg(Wr, si)
        ref_p = float(pitch_deg(rWr, ref_sk.index['Spine3'])[0])
        res.update(c_hips_min_m=float(hips_h.min()), c_ref_hips_m=ref_h, c_spine3_pitch_dev_max_deg=float(np.abs(p[last] - ref_p).max()))
        res['c_not_collapsed_ok'] = bool(hips_h.min() >= ref_h - 0.03 and np.abs(p[last] - ref_p).max() < 25.0 and np.abs(p[p.shape[0] - 7:] - ref_p).max() < 10.0)
        steps = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[1:] * lr[:-1], axis=-1)), 0, 1)))  # (T-1, J)
        cn_in = [n for n in core if n in sk.index]
        smax = steps[:, [sk.index[n] for n in cn_in]].max(axis=0)
        k = int(np.argmax(smax))
        res.update(d_step_max_deg=float(smax[k]), d_step_bone=cn_in[k])
        res['d_step_ok'] = bool(smax[k] <= a.max_step_deg)
        res.update(motion_checks(sk, t, lt, lr, a.max_root_yaw_deg, a.endpoint_step_deg, a.end_feet_m, a.max_joint_accel_cm, ref_feet))
        res.update(limb_ik_checks(sk, lt, lr))
        checks = ['a_duration_ok', 'b_end_pose_ok', 'c_not_collapsed_ok', 'd_step_ok'] + list(NEW_CHECKS)
        if a.against_fpp:  # the mutation must fail on the pose comparison itself, not on an unrelated check
            checks = ['b_end_pose_ok', 'c_not_collapsed_ok']
        elif brep:
            br = json.load(open(brep, encoding='utf-8'))
            fs = br['duck'].get('foot_slide_planted_final_max_m') or br['duck']['foot_slide_planted_max_m']
            res['j_planted_slide_cm'] = {kk: vv * 100 for kk, vv in fs.items()}
            src = br['duck'].get('foot_slide_source_planted_final_max_m')
            res['j_source_planted_slide_cm'] = {kk: vv * 100 for kk, vv in src.items()} if src else None
            res['j_plants'] = br['duck'].get('plant_segments')
            res['j_planted_slide_ok'] = bool(max(fs.values()) <= a.max_planted_slide_m)
            res['build_start_in_seat_frame'] = br.get('start_in_seat_frame')
            checks.append('j_planted_slide_ok')
        refp = a.ref_l if tag == 'enter_l' else a.ref_r
        if refp and os.path.abspath(refp) != os.path.abspath(path):   # the (m) gates judge a CHANGED clip against the shipped one; the shipped clip itself is not judged by them
            plants = json.load(open(brep, encoding='utf-8'))['duck'].get('plant_segments') if brep else None
            res.update(contact_gates(path, refp, cn, plants, a))
            checks += ['m_glide_ok', 'm_scrape_ok', 'm_hop_ok', 'm_clearance_ok'] + (['j_final_ok'] if plants else [])
        res['ok'] = bool(all(res[c] for c in checks))
        res['old_all_new_fail'] = bool(not any(res[c] for c in SELFTEST_OLD_CHECKS))
        ok_all = ok_all and res['ok']
        old_all_fail = old_all_fail and res['old_all_new_fail']
        rep['clips'][tag] = res
        print(tag, json.dumps({kk: (round(vv, 3) if isinstance(vv, float) else vv) for kk, vv in res.items()}))
    rep['ok'] = bool(ok_all)
    if a.report:
        json.dump(rep, open(a.report, 'w'), indent=1)
    if a.expect_fail:
        keys = a.expect_fail.split(',')
        allfail = all(not rep['clips'][c].get(k, True) for c in rep['clips'] for k in keys)
        print('EXPECT-FAIL', a.expect_fail, 'PASS (fails on both clips)' if allfail else 'FAIL (passes on a clip)')
        return 0 if allfail else 1
    if a.selftest_old:
        print('SELFTEST-OLD', 'PASS (every new check fails on the old clips)' if old_all_fail else 'FAIL (a new check passes on the old clips)')
        return 0 if old_all_fail else 1
    print('RESULT', 'PASS' if ok_all else 'FAIL')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
