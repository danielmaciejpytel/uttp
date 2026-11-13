"""Offline check of one UTPP third-person CAR entry clip (build_car.ps1): the round-tripped player-rig clip against the vanilla
player 'enter' clip of the same set (pwa_/pma_<class>_<seat>), whose first-person end pose the game continues with.

  python -I check_car_entry.py --clip <utpp_car_*.anims.glb> --vanilla <p?a_<class>_<seat>.anims.glb> --src-rig <w?a_..glb>
         [--build-report b.json] [--tpp-idle <same set glb>] [--report out.json] [--expect-fail k_limb_ik_helpers_ok,...]
         [--clip-name enter] [--max-joint-accel-cm X] [--accel-factor 1.0]

Checks (exit 0 = all pass), bones compared relative to Root (Root excluded) unless noted:
  (a) duration and frame count equal to the vanilla clip (the entry workspot timing, 2.00 s / 61 frames at 30 fps);
  (b) last frame vs the vanilla last frame: every non-helper bone local rotation < 3 deg and world position < 2 cm (the vanilla
      seated end pose, which the seated idle continues from: the game logged a 0 cm seat step after the vanilla enter);
  (c) start in the seat frame (where the engine slides the entity before the clip plays: seat * inverse(root delta)): position
      within 5 cm and yaw within 2 deg of the vanilla clip, so the approach spot beside the door is unchanged;
  (d) the local rotation of every retargeted bone changes by <= 15 deg per frame;
  (f) endpoint steps: every bone except Root / Trajectory / reference_joint changes its local rotation by < 1 deg between frames 0
      and 1 and between the last two frames;
  (g) end feet: LeftFoot and RightFoot of the last frame within 1 cm of the vanilla last frame;
  (h) root speed rule of check_bike_entry (h);
  (i) joint acceleration: second difference of the Root-frame positions of Hips, Spine3, Head, hands, feet, forearms and lower legs
      <= the same maximum measured on the vanilla clip (x --accel-factor), or --max-joint-accel-cm;
  (j) planted feet, measured on THIS clip: displacement per frame of each foot over the plant intervals listed in the build report
      <= 1 cm/frame (needs --build-report);
  (n) ANATOMICAL limit of the duck (since 2026-11-03; was 'not deeper than the vanilla clip' - the vanilla first-person clips pass through the roof of low cars themselves): trunk lean <= 75 deg,
      neck flexion <= 55 deg, knee flexion <= 150 deg, thigh-vs-trunk (hip) flexion <= 160 deg (anatomy.py LIMITS, measured on the game's own NPC clips) and the pelvis over the cushion (within
      0.3 m of the seated pelvis) never lower than the seated pelvis - 20 cm. With --build-report also HARD on the frames without a ceiling above the head (builder roof model): Head / Hips at most
      --overduck-tol under the lowest vanilla height at the same depth (a crouch outside the car = a cap that sees the wrong depth). Mutations: --duck-mut (pelvis + UpLegs lower), --lean-mut
      (trunk bent further), --neck-mut (head bent about the neck) must be rejected.
  (l) head below the roof line: once the head is >= 0.8 m deep into the car (-Z of the clip world, the depth axis of the builder),
      its height is <= the vanilla clip's head height envelope at that depth (max height of the vanilla frames at least as deep,
      -5 cm tolerance), +1 cm; computed here from the two clips, independent of the builder's own cap;
  (k) limb IK helpers on the foot / hand (check_bike_entry.limb_ik_checks); the limits are max(bike limit, vanilla clip's own helper
      offset + 0.2 cm / + 0.3 deg): pma_sport2_driver itself has its leg IK helper 0.54 cm / 0.73 deg and the standard2 driver its arm IK
      helper 1.05 cm off the bone, the car check must not demand more than the game's own clips deliver.
  (m) seat settle (2026-11-01, hard): with --tpp-idle <set glb> [--tpp-idle-clip tpp_idle] the clip's LAST frame is within 2 cm (Hips,
      Spine3, Head, both hands, both feet, root-relative world positions - not Hips alone: standard2 / standard3 have Hips 0 cm but
      Spine3 7 cm) of frame 0 of that set's seated clip, which the game switches to ~0.4-0.6 s after seating (drivers: the class
      'tpp_idle'; passengers: no 'tpp_idle' in their set, the game falls back to player_default_vehicle.anims tpp_idle (254) - the UTPP
      passenger set carries the class 'idle' renamed 'tpp_idle', so --tpp-idle-clip idle). Core bone rotations < 3 deg like (b).
      Without --tpp-idle the check is not run. With --tpp-idle (and no --end-ref) the end pose reference of (b) and (g) is that same
      frame instead of the vanilla enter's last frame (the builder's --end-pose <glb>:<clip> goes to the same pose).
  --end-ref <glb>:<clip>[@last|@first]: explicit reference frame for (b) / (g) (default: frame 0 of the --tpp-idle clip, else the
      vanilla enter's last frame).
  Reported only: end pose vs frame 0 of the vanilla set's tpp_idle (tpp_idle_f0_vs_end_cm, all 7 bones), vanilla maxima, head
  clearance margin.
--expect-fail k,l,...: exit 0 only if each listed check FAILS (mutation proof that the check is not vacuous).
--head-raise M [--head-raise-skip-end N]: mutation for (l) only - the clip's head is taken M m higher on every frame except the last N
  (proves the check bites on this clip during the entry motion even where the builder's head cap is not binding, e.g. a tall cab
  whose source head already stays under the vanilla envelope; skipping the end-blend frames keeps the seated end pose, which (b)
  pins to the vanilla clip, from deciding the result on its own).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
import glbanim as G  # noqa: E402
import check_bike_entry as CB  # noqa: E402
import anatomy as AN  # noqa: E402

FPS = 30.0
SKIP = CB.SKIP
CHECKS = ('a_duration_ok', 'b_end_pose_ok', 'c_start_in_seat_ok', 'd_step_ok', 'f_endpoint_steps_ok', 'g_end_feet_ok', 'h_root_speed_ok',
          'i_joint_accel_ok', 'j_planted_slide_ok', 'l_head_roof_ok', 'k_limb_ik_helpers_ok', 'm_seat_settle_ok', 'n_no_overduck_ok')
SETTLE_BONES = ('Hips', 'Spine3', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot')
SETTLE_MAX_CM = 2.0


def sample(sk, clip_name):
    c = sk.clip(clip_name)
    dur, t = B.frames_of(sk, c)
    lt, lr = sk.sample_local(c, t)
    return dur, t, lt, S.qcontinuous(lr)


def ref_frame(spec):
    """'glb:clip' (frame 0), 'glb:clip@first', 'glb:clip@last' -> (skeleton, world positions (N,3), local rotations (N,4), label)."""
    path, _, clip = spec.rpartition(':')
    if len(path) == 1:  # a drive letter was split off ('C:\\x.glb' without a clip)
        raise SystemExit('bad reference %r: expected <glb>:<clip>[@last]' % spec)
    last = clip.endswith('@last')
    clip = clip[:-5] if last else (clip[:-6] if clip.endswith('@first') else clip)
    rsk = S.Skeleton.from_glb(path)
    try:
        _, rt = B.frames_of(rsk, rsk.clip(clip))
    except KeyError:
        return rsk, None, None, '%s:%s' % (path, clip)
    sel = rt[-1:] if last else rt[:1]
    rlt, rlr = rsk.sample_local(rsk.clip(clip), sel)
    rlr = S.qcontinuous(rlr)
    rWt, _ = CB.pose_world(rsk, rlt, rlr)
    return rsk, rWt[0], rlr[0], '%s:%s%s' % (path, clip, '@last' if last else '')


def settle_cm(sk, Wt_last, rsk, rWt):
    return {n: round(float(np.linalg.norm(Wt_last[sk.index[n]] - rWt[rsk.index[n]])) * 100, 2) for n in SETTLE_BONES}


def start_in_seat(sk, lt, lr):
    ri = sk.index['Root']
    ye = float(B.yaw_of(lr[-1, ri]))
    d = G.qrot(B.qyaw(np.array([-ye])), (lt[0, ri] - lt[-1, ri])[None])[0]
    yaw = np.degrees(np.unwrap(np.radians(B.yaw_of(lr[:, ri]))))
    return d, float(yaw[0] - yaw[-1])


def joint_accel(sk, lt, lr):
    Wt, _ = CB.pose_world(sk, lt, lr)
    J = [sk.index[j] for j in CB.ACCEL_JOINTS]
    P = Wt[:, J] * 100
    acc = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=-1)
    f, j = np.unravel_index(int(np.argmax(acc)), acc.shape)
    return acc, float(acc.max()), CB.ACCEL_JOINTS[j], int(f) + 1


def head_envelope_check(sk, lt, lr, vsk, vlt, vlr, start=0.8, tol=0.05, margin=0.01, raise_m=0.0, raise_skip_end=0):
    Wt, _ = sk.world(lt, lr)
    vWt, _ = vsk.world(vlt, vlr)
    hi, vhi = sk.index['Head'], vsk.index['Head']
    ri_, vri_ = sk.index['Root'], vsk.index['Root']
    # depth in the seat frame: a clip that starts farther out (--start-out) has its root end shifted by delta against the vanilla clip's,
    # so the vanilla-equivalent depth of a point is -(p - delta).z, not -p.z
    shift_z = float(Wt[-1, ri_, 2] - vWt[-1, vri_, 2])
    d, h = -Wt[:, hi, 2] + shift_z, Wt[:, hi, 1].copy()
    h[:max(0, len(h) - raise_skip_end)] += raise_m
    vd, vh = -vWt[:, vhi, 2], vWt[:, vhi, 1]
    worst, wf = -9.0, -1
    n_in = 0
    for i in range(len(d)):
        if d[i] < start:
            continue
        n_in += 1
        m = vd >= d[i] - tol
        cap = vh[m].max() if m.any() else vh[-1]
        over = h[i] - cap
        if over > worst:
            worst, wf = float(over), i
    return dict(frames_inside=n_in, worst_over_cap_cm=worst * 100, worst_frame=wf,
                head_peak_m=float(h.max()), vanilla_head_peak_m=float(vh.max()), ok=bool(n_in > 0 and worst <= margin))


def overduck_vs_vanilla(sk, lt, lr, vsk, vlt, vlr, depth_tol=0.05, open_mask=None):
    """REPORT ONLY (was the hard check (n) until 2026-11-03): how far Head / Hips sit under the LOWEST height of the vanilla clip at the same seat-frame depth.
    The vanilla first-person clips themselves pass through the roof of low cars (roof penetration 8 - 14 cm, entry-paths report), so they are no
    bound for ducking; the hard check (n) is the anatomical one below."""
    Wt, _ = sk.world(lt, lr)
    vWt, _ = vsk.world(vlt, vlr)
    ri, vri = sk.index['Root'], vsk.index['Root']
    shift_z = float(Wt[-1, ri, 2] - vWt[-1, vri, 2])
    out = {}
    worst_open = 0.0
    for j in ('Head', 'Hips'):
        h = Wt[:, sk.index[j], 1]
        d = -Wt[:, sk.index[j], 2] + shift_z
        vd, vh = -vWt[:, vsk.index[j], 2], vWt[:, vsk.index[j], 1]
        worst = 0.0
        for i in range(len(d)):
            m = np.abs(vd - d[i]) <= depth_tol
            if m.any():
                u = float(vh[m].min() - h[i])
                worst = max(worst, u)
                if open_mask is not None and open_mask[i]:
                    worst_open = max(worst_open, u)
        out[j] = round(worst * 100, 2)
    out['open_air_worst'] = round(worst_open * 100, 2)  # frames without a ceiling above the head: there the vanilla depth bound is HARD
    return out


def overduck_check(sk, lt, lr, mut_m=0.0, mut_lean_deg=0.0, mut_neck_deg=0.0, mut_skip_end=18, limits=None, hips_below_m=None, near_m=0.30):
    """(n) ANATOMICAL limit of the duck (replaces 'not deeper than the vanilla clip': the vanilla first-person clips go through the roof of
    low cars themselves). The pose must stay inside what a person entering a low car does (anatomy.py LIMITS):
      trunk lean (Hips -> Neck against vertical) <= 75 deg, neck flexion (Spine3 -> Neck vs Neck -> Head) <= 55 deg,
      knee flexion <= 150 deg and hip flexion (thigh vs trunk) <= 160 deg on either leg,
      and the pelvis over the cushion (within near_m horizontally of the SEATED pelvis, in the clip's own last-frame frame) never lower than the
      seated pelvis height - anatomy.HIPS_BELOW_SEATED_M (a person cannot sit lower than the cushion).
    Mutations: mut_m lowers Hips and both UpLeg joints by that much on every frame except the last mut_skip_end (the pelvis floor and the knee /
    hip flexion must reject it); mut_lean_deg bends Spine3 / Neck / Head forward about the Hips by that angle (trunk lean / neck flexion)."""
    lim = dict(AN.LIMITS)
    if limits:
        lim.update(limits)
    hb = AN.HIPS_BELOW_SEATED_M if hips_below_m is None else hips_below_m
    Wt, _ = sk.world(lt, lr)
    Wt = Wt.copy()
    T = len(Wt)
    sel = np.arange(T) < max(0, T - mut_skip_end)
    if mut_m:
        for n in ('Hips', 'LeftUpLeg', 'RightUpLeg'):
            Wt[sel, sk.index[n], 1] -= mut_m
    if mut_lean_deg:
        h = Wt[:, sk.index['Hips']]
        for n in ('Spine3', 'Neck', 'Head'):
            v = Wt[:, sk.index[n]] - h
            hor = np.array(v)
            hor[:, 1] = 0.0
            hn = np.linalg.norm(hor, axis=-1, keepdims=True)
            fwd = np.where(hn > 1e-6, hor / np.maximum(hn, 1e-9), np.array([0.0, 0.0, -1.0]))  # lean towards the horizontal drift of the trunk (or -Z)
            ax = np.cross(np.array([0.0, 1.0, 0.0]), fwd)
            ax /= np.maximum(np.linalg.norm(ax, axis=-1, keepdims=True), 1e-9)
            a = np.radians(mut_lean_deg) * sel[:, None]
            c, s_ = np.cos(a), np.sin(a)
            vr = v * c + np.cross(ax, v) * s_ + ax * np.sum(ax * v, axis=-1, keepdims=True) * (1 - c)
            Wt[:, sk.index[n]] = h + vr
    if mut_neck_deg:  # Head bent forward about the Neck joint (neck flexion only; the trunk is untouched)
        for i in np.where(sel)[0]:
            nk = Wt[i, sk.index['Neck']]
            v = Wt[i, sk.index['Head']] - nk
            hor = np.array([v[0], 0.0, v[2]])
            hn = np.linalg.norm(hor)
            fwd = hor / hn if hn > 1e-6 else np.array([0.0, 0.0, -1.0])
            ax = np.cross(np.array([0.0, 1.0, 0.0]), fwd)
            ax /= max(np.linalg.norm(ax), 1e-9)
            a_, c_, s_ = np.radians(mut_neck_deg), np.cos(np.radians(mut_neck_deg)), np.sin(np.radians(mut_neck_deg))
            Wt[i, sk.index['Head']] = nk + v * c_ + np.cross(ax, v) * s_ + ax * np.dot(ax, v) * (1 - c_)
    m = AN.metrics(sk, Wt)
    per = {}
    ok = True
    for k, lm in lim.items():
        mx = float(m[k].max())
        per[k] = dict(max_deg=round(mx, 2), limit_deg=lm, frame=int(np.argmax(m[k])), ok=bool(mx <= lm))
        ok &= per[k]['ok']
    hi = sk.index['Hips']
    hy = Wt[:, hi, 1]
    near = np.linalg.norm(Wt[:, hi][:, [0, 2]] - Wt[-1, hi][[0, 2]], axis=-1) <= near_m
    low = np.where(near, hy - (Wt[-1, hi, 1] - hb), 9.0)
    k_ = int(np.argmin(low))
    per['hips_floor'] = dict(min_over_floor_cm=round(float(low[k_]) * 100, 2) if near.any() else None, frame=k_, floor_below_seated_cm=hb * 100,
                             frames_near_seat=int(near.sum()), ok=bool(not near.any() or low[k_] >= 0))
    ok &= per['hips_floor']['ok']
    return dict(per=per, ok=bool(ok))


def plant_slide(sk, lt, lr, build_report):
    br = json.load(open(build_report, encoding='utf-8'))
    segs = br['duck'].get('plant_segments')
    if not segs:
        return None
    Wt, _ = sk.world(lt, lr)
    out = {}
    for foot, seg in segs.items():
        p = Wt[:, sk.index[foot]]
        step = np.linalg.norm(np.diff(p, axis=0), axis=-1)
        m = np.zeros(len(step), bool)
        for a, b in seg['plants']:
            m[a:b] = True
        out[foot] = float(np.where(m, step, 0).max()) * 100
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--vanilla', required=True)
    ap.add_argument('--src-rig', required=True)
    ap.add_argument('--clip-name', default='enter')
    ap.add_argument('--build-report')
    ap.add_argument('--tpp-idle', help='glb of the vanilla set; the last frame of the clip must settle into frame 0 of its seated clip '
                    '(--tpp-idle-clip) within 2 cm on 7 bones (check m, hard)')
    ap.add_argument('--tpp-idle-clip', default='tpp_idle', help="the seated clip the game switches to: 'tpp_idle' (driver), 'idle' (passenger)")
    ap.add_argument('--end-ref', help='<glb>:<clip>[@last]: reference frame of (b) / (g); default frame 0 of the --tpp-idle clip, else vanilla enter@last')
    ap.add_argument('--report')
    ap.add_argument('--max-joint-accel-cm', type=float)
    ap.add_argument('--accel-factor', type=float, default=1.0)
    ap.add_argument('--expect-fail')
    ap.add_argument('--max-step-deg', type=float, default=15.0)
    ap.add_argument('--head-raise', type=float, default=0.0)
    ap.add_argument('--head-raise-skip-end', type=int, default=0)
    ap.add_argument('--expect-start-shift', type=float, default=0.0,
                    help='m along the seat-frame x: the start spot of this clip is deliberately this far from the vanilla clip start (build_car --start-out * out-axis); (c) then asserts exactly that shift')
    ap.add_argument('--expect-start-shift-z', type=float, default=0.0,
                    help='m along the seat-frame z (build_car --start-back): with --expect-start-shift the start spot is shifted by (x, z)')
    ap.add_argument('--overduck-tol', type=float, default=0.25, help='m: (n) on frames WITHOUT a ceiling above the head (builder roof model, needs --build-report): Head / Hips at most this far under the lowest vanilla height at the same depth')
    ap.add_argument('--duck-mut', type=float, default=0.0, help='mutation for (n): Hips and both UpLeg joints taken this much lower (except the last 18 frames)')
    ap.add_argument('--neck-mut', type=float, default=0.0, help='mutation for (n): Head bent forward about the Neck joint by this many degrees (except the last 18 frames)')
    ap.add_argument('--lean-mut', type=float, default=0.0, help='mutation for (n): Spine3 / Neck / Head bent forward about the Hips by this many degrees (except the last 18 frames)')
    a = ap.parse_args()
    sk = S.Skeleton.from_glb(a.clip)
    vsk = S.Skeleton.from_glb(a.vanilla)
    core = [n for n in vsk.names if n in set(S.Skeleton.from_glb(a.src_rig).names) and n not in SKIP]
    dur, t, lt, lr = sample(sk, a.clip_name)
    vdur, vt, vlt, vlr = sample(vsk, 'enter')
    res = {'clip': a.clip, 'vanilla': a.vanilla, 'duration_s': float(dur), 'frames': int(len(t))}
    res['a_duration_ok'] = bool(abs(dur - vdur) < 1e-3 and len(t) == len(vt))
    # (b) end pose vs the vanilla last frame
    Wt, Wr = CB.pose_world(sk, lt, lr)
    vWt, vWr = CB.pose_world(vsk, vlt, vlr)
    # the end pose reference: the vanilla enter's last frame, or (--end-ref / --tpp-idle) the seated clip the game settles into
    endspec = a.end_ref or ('%s:%s' % (a.tpp_idle, a.tpp_idle_clip) if a.tpp_idle else None)
    end_missing = None
    if endspec:
        esk, eWt1, elr1, elabel = ref_frame(endspec)
        if eWt1 is None:
            # a normal failure, not a crash: the JSON / RESULT lines are still printed (build_car.ps1 captures stdout only). Typical cause:
            # --tpp-idle on a passenger set without --tpp-idle-clip idle (passenger sets carry no tpp_idle)
            end_missing = 'no clip in %s (passenger sets: --tpp-idle-clip idle)' % elabel
            elabel = a.vanilla + ':enter@last'
            eWt, elr, esk_ = vWt[-1], vlr[-1], vsk
        else:
            eWt, elr, esk_ = eWt1, elr1, esk
    else:
        elabel = a.vanilla + ':enter@last'
        eWt, elr, esk_ = vWt[-1], vlr[-1], vsk
    res['end_ref'] = elabel
    if end_missing:
        res['end_ref_error'] = end_missing
    pos_max, rot_max, pb, rb = 0.0, 0.0, '', ''
    for n in core:
        if n not in sk.index:
            continue
        i, j = sk.index[n], esk_.index[n]
        dp = float(np.linalg.norm(Wt[-1, i] - eWt[j]))
        dr = float(CB.qang(lr[-1, i], elr[j]))
        if dp > pos_max:
            pos_max, pb = dp, n
        if dr > rot_max:
            rot_max, rb = dr, n
    res.update(b_pos_max_cm=pos_max * 100, b_pos_bone=pb, b_rot_max_deg=rot_max, b_rot_bone=rb)
    res['b_end_pose_ok'] = bool(pos_max < 0.02 and rot_max < 3.0 and not end_missing)
    # (c) start in the seat frame
    d_new, y_new = start_in_seat(sk, lt, lr)
    d_van, y_van = start_in_seat(vsk, vlt, vlr)
    res['c_start_in_seat'] = dict(new=dict(pos=np.round(d_new, 3).tolist(), yaw=round(y_new, 2)),
                                  vanilla=dict(pos=np.round(d_van, 3).tolist(), yaw=round(y_van, 2)))
    res['c_start_pos_err_cm'] = float(np.linalg.norm(d_new - d_van)) * 100
    res['c_start_yaw_err_deg'] = abs(y_new - y_van)
    if a.expect_start_shift or a.expect_start_shift_z:
        # the start moved by exactly expect_start_shift along the seat-frame x and expect_start_shift_z along z (+-1 cm each); y and the yaw keep the normal tolerances
        e = d_new - d_van - np.array([a.expect_start_shift, 0.0, a.expect_start_shift_z])
        res['c_start_shift_err_cm'] = [round(float(v) * 100, 2) for v in e]
        res['c_start_in_seat_ok'] = bool(abs(e[0]) <= 0.01 and abs(e[2]) <= 0.01 and abs(e[1]) < 0.05 and res['c_start_yaw_err_deg'] < 2.0)
    else:
        res['c_start_in_seat_ok'] = bool(res['c_start_pos_err_cm'] < 5.0 and res['c_start_yaw_err_deg'] < 2.0)
    # (d), (f)
    steps = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[1:] * lr[:-1], axis=-1)), 0, 1)))
    cn_in = [n for n in core if n in sk.index]
    smax = steps[:, [sk.index[n] for n in cn_in]].max(axis=0)
    k = int(np.argmax(smax))
    res.update(d_step_max_deg=float(smax[k]), d_step_bone=cn_in[k])
    res['d_step_ok'] = bool(smax[k] <= a.max_step_deg)
    ri = sk.index['Root']
    bones = [i for i, n in enumerate(sk.names) if n not in SKIP]
    s0, s1 = steps[0, bones], steps[-1, bones]
    res.update(f_first_step_max_deg=float(s0.max()), f_first_step_bone=sk.names[bones[int(np.argmax(s0))]],
               f_last_step_max_deg=float(s1.max()), f_last_step_bone=sk.names[bones[int(np.argmax(s1))]])
    res['f_endpoint_steps_ok'] = bool(s0.max() < 1.0 and s1.max() < 1.0)
    # (g)
    fe = {kk: float(np.linalg.norm(Wt[-1, sk.index[kk]] - eWt[esk_.index[kk]])) * 100 for kk in ('LeftFoot', 'RightFoot')}
    res['g_end_feet_err_cm'] = fe
    res['g_end_feet_ok'] = bool(max(fe.values()) < 1.0 and not end_missing)
    # (h)
    v = np.linalg.norm(np.diff(lt[:, ri], axis=0), axis=-1) * 100
    ex = B.speed_rule_violations(v)
    kk = int(np.argmax(ex))
    vv = np.r_[0.0, v, 0.0]
    res.update(h_root_speed_max_cm=float(v.max()), h_root_speed_excess_cm=float(ex.max()),
               h_root_speed_worst=dict(frame=kk, from_cm=float(vv[kk]), to_cm=float(vv[kk + 1])))
    res['h_root_speed_ok'] = bool(ex.max() <= 1e-3)
    vv_ = np.linalg.norm(np.diff(vlt[:, vsk.index['Root']], axis=0), axis=-1) * 100
    res['h_vanilla_root_speed_excess_cm'] = float(B.speed_rule_violations(vv_).max())
    # (i)
    acc, amax, aj, af = joint_accel(sk, lt, lr)
    vacc, vmax, vj, vf = joint_accel(vsk, vlt, vlr)
    lim = a.max_joint_accel_cm if a.max_joint_accel_cm else vmax * a.accel_factor
    res.update(i_joint_accel_max_cm=amax, i_joint_accel_worst=dict(frame=af, t=round(float(t[af]), 3), joint=aj),
               i_vanilla_joint_accel_max_cm=vmax, i_vanilla_worst=dict(joint=vj, frame=vf), i_limit_cm=lim)
    res['i_joint_accel_ok'] = bool(amax <= lim)
    # (j)
    if a.build_report:
        ps = plant_slide(sk, lt, lr, a.build_report)
        res['j_planted_slide_cm'] = ps
        res['j_planted_slide_ok'] = bool(ps is not None and max(ps.values()) <= 1.0)
    else:
        res['j_planted_slide_ok'] = True
        res['j_note'] = 'no build report: not checked'
    # (l)
    he = head_envelope_check(sk, lt, lr, vsk, vlt, vlr, raise_m=a.head_raise, raise_skip_end=a.head_raise_skip_end)
    res['l_head_roof'] = he
    res['l_head_roof_ok'] = he['ok']
    od = overduck_check(sk, lt, lr, mut_m=a.duck_mut, mut_lean_deg=a.lean_mut, mut_neck_deg=a.neck_mut)
    res['n_anatomy'] = od['per']
    res['n_no_overduck_ok'] = od['ok']
    open_mask = None
    if a.build_report:
        cy = json.load(open(a.build_report, encoding='utf-8'))['duck'].get('roof')
        if cy and len(cy['cap_y']) == len(t):
            open_mask = np.asarray(cy['cap_y']) >= 8.0  # no ceiling above the head in the builder's roof model
    res['n_report_under_vanilla_cm'] = overduck_vs_vanilla(sk, lt, lr, vsk, vlt, vlr, open_mask=open_mask)
    if open_mask is not None:
        # a head / pelvis far under the vanilla clip while there is no ceiling above it is a crouch outside the car (a cap that sees the wrong depth): hard, class tolerance a.overduck_tol
        res['n_open_air_ok'] = bool(res['n_report_under_vanilla_cm']['open_air_worst'] <= a.overduck_tol * 100)
        res['n_no_overduck_ok'] = bool(res['n_no_overduck_ok'] and res['n_open_air_ok'])
    # (k)
    # the vanilla player clip's own arm IK helpers sit up to ~1.05 cm off the hand (pwa/pma_standard2_driver arm_left_ik): the hand
    # limit is max(1 cm, vanilla maximum + 0.2 cm); feet and the pole plane keep the bike limits
    vk = CB.limb_ik_checks(vsk, vlt, vlr)['k_limb_ik_helpers']
    v_hand = max(v['cm'] for n_, v in vk.items() if ('Arm_Hand' in n_ or n_.startswith('arm_')) and 'cm' in v)
    v_foot = max(v['cm'] for n_, v in vk.items() if ('Leg_Foot' in n_ or n_.startswith('leg_')) and 'cm' in v)
    v_foot_deg = max(v['deg'] for n_, v in vk.items() if 'deg' in v)
    hand_m = max(0.01, (v_hand + 0.2) / 100.0)
    foot_m = max(0.005, (v_foot + 0.2) / 100.0)
    foot_deg = max(1.0, v_foot_deg + 0.3)
    res.update(k_vanilla_hand_helper_max_cm=v_hand, k_vanilla_foot_helper_max_cm=v_foot, k_vanilla_foot_helper_max_deg=v_foot_deg)
    res.update(CB.limb_ik_checks(sk, lt, lr, foot_m=foot_m, foot_deg=foot_deg, hand_m=hand_m))
    # (m) seat settle: the last frame against frame 0 of the seated clip the game switches to (hard); tpp_idle_f0_vs_end_cm stays as the
    # report of the driver sets' tpp_idle
    if a.tpp_idle:
        tsk = S.Skeleton.from_glb(a.tpp_idle)
        try:
            tc = tsk.clip('tpp_idle')
        except KeyError:
            tc = None
        if tc is None:
            res['tpp_idle_f0_vs_end_cm'] = 'no tpp_idle clip in this set'
        else:
            _, tt = B.frames_of(tsk, tc)
            tlt, tlr = tsk.sample_local(tc, tt[:1])
            tWt, _ = CB.pose_world(tsk, tlt, S.qcontinuous(tlr))
            res['tpp_idle_f0_vs_end_cm'] = settle_cm(sk, Wt[-1], tsk, tWt[0])
        ssk, sWt1, slr1, slabel = ref_frame('%s:%s' % (a.tpp_idle, a.tpp_idle_clip))
        if sWt1 is None:
            res['m_seat_settle_cm'] = 'no clip %s in %s (passenger sets: --tpp-idle-clip idle)' % (a.tpp_idle_clip, a.tpp_idle)
            res['m_seat_settle_ok'] = False
        else:
            sd = settle_cm(sk, Wt[-1], ssk, sWt1)
            srot = max(float(CB.qang(lr[-1, sk.index[n]], slr1[ssk.index[n]])) for n in core if n in sk.index)
            res.update(m_seat_settle_ref=slabel, m_seat_settle_cm=sd, m_seat_settle_max_cm=max(sd.values()),
                       m_seat_settle_worst=max(sd, key=sd.get), m_seat_settle_rot_max_deg=srot)
            res['m_seat_settle_ok'] = bool(max(sd.values()) <= SETTLE_MAX_CM and srot < 3.0)
    else:
        res['m_seat_settle_ok'] = True
        res['m_note'] = 'no --tpp-idle: seat settle not checked'
    res['ok'] = bool(all(res[c] for c in CHECKS))
    print(json.dumps({kk_: (round(vv_, 3) if isinstance(vv_, float) else vv_) for kk_, vv_ in res.items()}))
    if a.report:
        json.dump(res, open(a.report, 'w'), indent=1)
    if a.expect_fail:
        keys = a.expect_fail.split(',')
        allfail = all(not res.get(kk_, True) for kk_ in keys)
        print('EXPECT-FAIL', a.expect_fail, 'PASS (fails)' if allfail else 'FAIL (passes: ' + ','.join(k_ for k_ in keys if res.get(k_, True)) + ')')
        return 0 if allfail else 1
    print('RESULT', 'PASS' if res['ok'] else 'FAIL: ' + ','.join(c for c in CHECKS if not res[c]))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
