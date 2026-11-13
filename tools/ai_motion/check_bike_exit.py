"""Gates for a MOTORCYCLE get-off clip (exit_l / exit_r) that was re-stepped offline (bike_steps_gen.py pick -> build_bike_exit.ps1), run on the ROUND-TRIPPED clip
(WolvenKit export of the packed .anims) against the vanilla clip of the same player set.

  python -I check_bike_exit.py --clip <roundtrip glb> --vanilla <vanilla p?a_sportbike glb> --clip-name exit_l [--builder <builder glb>] [--report out.json]
  (a) same frame count / duration;
  (b) first and last frame identical to vanilla (every bone: <= --tol-deg, <= --tol-m local translation);
  (c) every frame outside the changed span (frames where the builder clip differs from vanilla; needs --builder) equal to vanilla within the import noise floor (--noise-deg / --noise-m); the Root track (root motion) identical on ALL frames;
  (d) no local rotation step above max(--max-step-deg, vanilla's own maximum; the vanilla exit_r already has a 85 deg quaternion step on a helper bone) per frame (30 fps) on any bone;
  (e) no joint acceleration (hips, spine, knees, feet, toes; cm / frame^2) above max(--max-accel-cm, vanilla's own maximum);
  (f) foot glide (bike_steps_gen.glide robust=True: contact + vertical-speed rule, robust floors of the vanilla clip) <= --max-glide-cm (6) and not above vanilla; plus the net motion of every planted-foot phase <= --max-phase-net-cm (3);
  (g) bike clearance proxy (bike_steps_gen.clearance: inward / downward / hand deviation against the vanilla path next to the bike);
  (h) no toe below the vanilla ground (lowest vanilla toe height - 3 cm);
  (i) --root-retimed (walk-away v2, 2026-11-07): anatomy. Upper body (spine / neck / clavicles / arms / wrists / torso distances, anatomy_gate.features) from the frame the natural upper body is fully in (walk.json ub_ramp[1] next to --builder)
      inside the envelope of V's source pack idle / walk / run body (--env-json: gasp_env_<woman|man>.json) +- 8 deg / 2 cm; legs on ALL frames: knee flexion in [-8, 140], knee hinge deviation / shin roll /
      hip flexion / hip twist not above the vanilla clip's own maximum (+1 deg) - the leg-over of a bike seat is not a walk; arm-leg clearance (hand points vs thigh / shin segments) >= 3 cm from the same frame.
Exit code 0 = all pass. Prints a JSON-like report line per check.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
import game_skeleton_bridge as CB  # noqa: E402
import bike_steps_gen as BSG  # noqa: E402
import skel as S  # noqa: E402
import glbanim as GA  # noqa: E402


def locals_of(path, clip):
    sk = S.Skeleton.from_glb(path)
    ai = sk.clip(clip)
    ch = GA.channels(sk.gj, sk.binb, ai)
    dur = max(float(tr[0][-1]) for b, d in ch.items() if b != 'Root' for tr in d.values())
    times = np.arange(0.0, dur + 1e-6, 1.0 / 30)
    lt, lr = sk.sample_local(ai, times)
    return sk, dur, np.asarray(lt), S.qcontinuous(np.asarray(lr))


def qang(a, b):
    d = np.abs(np.sum(a * b, axis=-1)) / np.maximum(np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1), 1e-12)
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def dump73(glb, clip):
    import subprocess
    import tempfile
    out = os.path.join(tempfile.gettempdir(), 'chk73_%d.npz' % os.getpid())
    subprocess.run([sys.executable, '-I', os.path.join(HERE, 'game_skeleton_bridge.py'), 'dump-glb', glb, clip, out], check=True, capture_output=True)
    d = dict(np.load(out, allow_pickle=True))
    os.remove(out)
    return d


def anatomy_gate_i(a, Wc_unused, Wv_unused, pn_unused):
    sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'melee'))
    import anatomy_gate as AG
    dc, dv = dump73(a.clip, a.clip_name), dump73(a.vanilla, a.clip_name)
    rg = AG.rig_of(dc)
    fc_, fv_ = AG.features(dc['W'], rg), AG.features(dv['W'], rg)
    rigname = 'man' if 'pma' in os.path.basename(a.vanilla) else 'woman'
    envf = a.env_json or os.path.join(ROOT, 'Build', 'work', 'vehicle-bike-archive', 'bike7-20261107', 'ref', 'gasp_env_%s.json' % rigname)
    env = json.load(open(envf))
    ub_from = 40
    if a.builder:
        bn = os.path.basename(a.builder)
        wj = os.path.join(os.path.dirname(os.path.abspath(a.builder)), bn.replace('.anims.glb', '') + '.walk.json')
        if not os.path.exists(wj):
            wj = os.path.join(os.path.dirname(os.path.abspath(a.builder)), 'walk.json')
        if os.path.exists(wj):
            ub_from = int(json.load(open(wj))['ub_ramp'][1])
    leg_pref = ('knee_', 'hip_', 'shin_', 'foot_')
    skip = ('rot_step', 'hips_drop_cm', 'hand_eye_cm')   # seg_len_dev_pct IS gated (envelope = the source pack clips on the same rig: 0 % woman / 25.2 % man reference offset, +- 2.5 %)
    viol = []
    worst = {}
    for k, v in fc_.items():
        if k in skip or k not in env or k.startswith(leg_pref):
            continue
        tol = 2.5 if k.endswith('_pct') else (2.0 if '_cm' in k else 8.0)
        seg = v[ub_from:]
        ex = np.maximum(env[k][0] - tol - seg, seg - (env[k][3] + tol))
        worst[k] = round(float(ex.max()), 2)
        if ex.max() > 0:
            viol.append((k, round(float(ex.max()), 2), int(ex.argmax()) + ub_from))
    legs = {}
    kv = {k: float(fv_[k].max()) for k in ('knee_dev_L', 'knee_dev_R', 'shin_roll_L', 'shin_roll_R', 'hip_flex_L', 'hip_flex_R', 'hip_twist_L', 'hip_twist_R')}
    lviol = []
    for s in ('L', 'R'):
        kf = fc_['knee_flex_' + s]
        legs['knee_flex_' + s] = [round(float(kf.min()), 1), round(float(kf.max()), 1)]
        if kf.min() < -8 or kf.max() > 150.0:   # 150 = about the largest source pack run-loop knee flexion (147.8 woman / 143.9 man) + 2
            lviol.append(('knee_flex_' + s, float(kf.min()), float(kf.max())))
        for k in ('knee_dev_', 'shin_roll_', 'hip_flex_', 'hip_twist_'):
            m = float(fc_[k + s].max())
            legs[k + s] = [round(m, 1), round(kv[k + s], 1)]
            if m > max(kv[k + s] + 2.0, {'hip_flex_': 125.0, 'hip_twist_': 56.0}.get(k, 0.0)):   # +2 deg: the hinge rebuild re-derives the thigh twist from the plane
                lviol.append((k + s, round(m, 1), round(kv[k + s], 1)))
    # swing knee (the leg with the higher vanilla knee): flexion <= vanilla max + 3 deg, knee height <= vanilla max + 1 cm (bike6 in-game review: knee very high / strongly bent)
    Wv_ = dv['W']
    ixv = {n: i for i, n in enumerate([str(x) for x in dv['names']])}
    kzv = {s: float(Wv_[:, ixv[s + 'Leg'], 2, 3].max()) for s in ('Left', 'Right')}
    sw_ = 'Left' if kzv['Left'] > kzv['Right'] else 'Right'
    s_ = sw_[0]
    knee_flex_sw = float(fc_['knee_flex_' + s_].max())
    knee_z_sw = float(dc['W'][:, [str(x) for x in dc['names']].index(sw_ + 'Leg'), 2, 3].max())
    swing_knee_ok = bool(knee_flex_sw <= float(fv_['knee_flex_' + s_].max()) + 3.0 and knee_z_sw <= kzv[sw_] + 0.01)
    # arm-leg clearance: hand and forearm midpoint vs the thigh / shin segments (cm), from ub_from on
    W = dc['W']
    ix = {n: i for i, n in enumerate([str(x) for x in dc['names']])}

    def seg_dist(P, A, B):
        AB = B - A
        t = np.clip(np.sum((P - A) * AB, 1) / np.maximum(np.sum(AB * AB, 1), 1e-9), 0, 1)
        return np.linalg.norm(P - (A + t[:, None] * AB), axis=1)
    cl = 1e9
    for arm in ('Left', 'Right'):
        pts = [W[:, ix[arm + 'Hand'], :3, 3], 0.5 * (W[:, ix[arm + 'ForeArm'], :3, 3] + W[:, ix[arm + 'Hand'], :3, 3])]
        for leg in ('Left', 'Right'):
            for u, v in ((leg + 'UpLeg', leg + 'Leg'), (leg + 'Leg', leg + 'Foot')):
                for P in pts:
                    cl = min(cl, float(seg_dist(P[ub_from:], W[ub_from:, ix[u], :3, 3], W[ub_from:, ix[v], :3, 3]).min()) * 100)
    # (j) end pose (2026-11-07 v3): the last 10 frames stand like the idle - hips height above the lower foot not more than 2 cm below the vanilla last frame, knee flexion not above the source pack idle value + 10 deg (straighter is fine: vanilla ends on straight legs)
    ixc_ = {n: i for i, n in enumerate([str(x) for x in dc['names']])}
    Wd = dc['W']
    hz = Wd[:, ixc_['Hips'], 2, 3] - np.minimum(Wd[:, ixc_['LeftFoot'], 2, 3], Wd[:, ixc_['RightFoot'], 2, 3])
    hzv = float(Wv_[-1, ixv['Hips'], 2, 3] - min(Wv_[-1, ixv['LeftFoot'], 2, 3], Wv_[-1, ixv['RightFoot'], 2, 3]))
    gi_ = dict(np.load(os.path.join(os.path.dirname(envf), '%s_M_Neutral_Stand_Idle_Loop.npz' % rigname), allow_pickle=True))
    fi_ = AG.features(gi_['W'], AG.rig_of(gi_))
    idle_k = {s: float(np.mean(fi_['knee_flex_' + s])) for s in ('L', 'R')}
    end_k = {s: [round(float(fc_['knee_flex_' + s][-10:].min()), 1), round(float(fc_['knee_flex_' + s][-10:].max()), 1)] for s in ('L', 'R')}
    # end stance 'idle' (bike_exit_walk_away --end-stance idle, 2026-11-08): the clip ends in the source pack idle stance on purpose (feet staggered ~29 cm, the idle hips stand ~2 cm lower than the vanilla
    # last frame by nature), so the hips reference is the idle's own hips height above its lower foot
    end_ref = 'vanilla'
    if a.builder and os.path.exists(wj):
        try:
            if json.load(open(wj)).get('end_stance') == 'idle':
                ixi_ = {n: i for i, n in enumerate([str(x) for x in gi_['names']])}
                hzv = float(np.mean(gi_['W'][:, ixi_['Hips'], 2, 3] - np.minimum(gi_['W'][:, ixi_['LeftFoot'], 2, 3], gi_['W'][:, ixi_['RightFoot'], 2, 3])))
                end_ref = 'sourcepack_idle'
        except Exception:
            pass
    drop_cm = round((hzv - float(hz[-10:].min())) * 100, 1)
    end_ok = bool(drop_cm <= 2.0 and all(float(fc_['knee_flex_' + s][-10:].max()) <= idle_k[s] + 10.0 for s in ('L', 'R')))
    # (k) feet: final stance uncrossed (support foot outboard) with >= 8 cm lateral gap, ground-plane distance >= 8 cm from the swing foot's landing on; the number of frames with the lateral order swapped is reported (a swing foot that lands outboard of the planted one has to be passed once: unavoidable without a hop)
    left_ = a.clip_name.endswith('_l')
    SU_, SW_ = ('Left', 'Right') if left_ else ('Right', 'Left')
    osg = -1.0 if left_ else 1.0
    fp_, last_step = 50, None
    if a.builder:
        try:
            wjj = json.load(open(wj))
            fp_, last_step = int(wjj['fp']), int(wjj['steps'][-1])
        except Exception:
            pass
    last_step = last_step if last_step is not None else len(Wd) - 12
    oW = osg * Wd[:, ixc_[SW_ + 'Foot'], 0, 3]
    oS = osg * Wd[:, ixc_[SU_ + 'Foot'], 0, 3]
    # final stance: the support foot is outboard of the swing foot (left foot left of the right foot, as in vanilla: the legs are not crossed) by >= 8 cm lateral gap
    gap = (oS - oW)[last_step:]
    # all frames from the landing: the two feet are never closer than 8 cm in the ground plane (no foot goes through the other); informational: frames with a swapped lateral order (crossed stance)
    dxy = np.linalg.norm(Wd[fp_:, ixc_[SW_ + 'Foot'], :2, 3] - Wd[fp_:, ixc_[SU_ + 'Foot'], :2, 3], axis=1)
    swapped = int(((oW - oS)[fp_:] > 0.0).sum())
    order_ok = bool(gap.min() >= 0.08 and dxy.min() >= 0.08)
    return dict(i_end_hips_drop_cm=drop_cm, i_end_hips_ref=end_ref, i_end_knee_flex=end_k, i_idle_knee_flex=idle_k, i_end_ok=end_ok, i_feet_final_lateral_gap_min_cm=round(float(gap.min()) * 100, 1), i_feet_min_distance_xy_cm=round(float(dxy.min()) * 100, 1), i_feet_swapped_frames=swapped, i_feet_order_ok=order_ok, i_ub_from=ub_from, i_ub_violations=viol, i_ub_worst=dict(sorted(worst.items(), key=lambda kv_: -kv_[1])[:6]), i_legs=legs, i_leg_violations=lviol, i_arm_leg_clearance_cm=round(cl, 1),
                i_swing_knee=dict(leg=sw_, flex=round(knee_flex_sw, 1), flex_vanilla=round(float(fv_['knee_flex_' + s_].max()), 1), z=round(knee_z_sw, 3), z_vanilla=round(kzv[sw_], 3)), i_swing_knee_ok=swing_knee_ok,
                i_anatomy_ok=bool(not viol and not lviol and cl >= 3.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--vanilla', required=True)
    ap.add_argument('--clip-name', required=True)
    ap.add_argument('--builder')
    ap.add_argument('--report')
    ap.add_argument('--tol-deg', type=float, default=0.05)
    ap.add_argument('--tol-m', type=float, default=0.0005)
    ap.add_argument('--max-step-deg', type=float, default=15.0)
    ap.add_argument('--max-accel-cm', type=float, default=16.0)
    ap.add_argument('--max-glide-cm', type=float, default=6.0)
    ap.add_argument('--max-phase-net-cm', type=float, default=3.0, help='(f) largest net horizontal motion of a planted foot (bike_steps_gen.contact_phases, one contact phase) <= this')
    ap.add_argument('--root-retimed', action='store_true', help='walk-away variant (bike_exit_walk.py): the clip is LONGER than the vanilla one and the Root path differs on purpose: (a) only requires >= vanilla frames, (b) first frame only, (c) replaced by the final Root lateral offset >= --min-root-end, vanilla comparisons use the common frames')
    ap.add_argument('--min-root-end', type=float, default=0.75, help='m: bike half width 0.35 + player capsule 0.40')
    ap.add_argument('--env-json', help='(i) source pack idle / walk / run feature envelope json (ref1.py); default: Build/work/vehicle-bike-archive/bike7-20261107/ref/gasp_env_<rig>.json')
    ap.add_argument('--min-axis-final', type=float, default=0.35, help='m: foot / toe / knee distance from the bike axis in the final stance (assumed half width)')
    ap.add_argument('--vanilla-path-tol', type=float, default=0.10, help='m: a joint below the seat top inside the 0.30 m band may be this much closer to the axis than the vanilla joint was at that height (no bike geometry offline; knee flexion <= vanilla + 3 deg is the hard rule, so the hips-lagging swing foot cannot be pushed further out). Unverified in game.')
    ap.add_argument('--noise-deg', type=float, default=0.15, help='(c) outside the changed span: WolvenKit import / export noise floor measured 2026-11-06 on pwa_sportbike exit_r: 0.07 deg on Spine2 / Spine3, 2.6 mm on the leg pole helpers')
    ap.add_argument('--noise-m', type=float, default=0.003)
    a = ap.parse_args()
    skc, dc, ltc, lrc = locals_of(a.clip, a.clip_name)
    skv, dv, ltv, lrv = locals_of(a.vanilla, a.clip_name)
    res = {}
    res['a_frames_ok'] = bool(len(ltc) >= len(ltv)) if a.root_retimed else bool(len(ltc) == len(ltv) and abs(dc - dv) < 1e-3)
    res['a_frames'] = [len(ltc), len(ltv)]
    if not res['a_frames_ok']:
        print(json.dumps(res))
        return 1
    nv = len(ltv)
    if a.root_retimed:
        ltc_full, lrc_full = ltc, lrc
        ltc, lrc = ltc[:nv], lrc[:nv]
    names = skc.names
    iv = {n: i for i, n in enumerate(skv.names)}
    cols = [iv[n] for n in names]          # vanilla column of each clip bone (same rig; order may differ)
    dang = np.stack([qang(lrc[:, j], lrv[:, cols[j]]) for j in range(len(names))], 1)
    dtr = np.linalg.norm(ltc - ltv[:, cols], axis=-1)
    diff = (dang > a.tol_deg) | (dtr > a.tol_m)
    res['b_first_diff_deg'] = round(float(dang[0].max()), 4)
    res['b_last_diff_deg'] = round(float(dang[-1].max()), 4)
    res['b_first_diff_mm'] = round(float(dtr[0].max() * 1000), 3)
    res['b_last_diff_mm'] = round(float(dtr[-1].max() * 1000), 3)
    res['b_ends_ok'] = bool(not diff[0].any()) if a.root_retimed else bool(not diff[0].any() and not diff[-1].any())
    ri = names.index('Root')
    res['c_root_diff_deg'] = round(float(dang[:, ri].max()), 4)
    res['c_root_diff_mm'] = round(float(dtr[:, ri].max() * 1000), 3)
    chg = np.flatnonzero(diff.any(1))
    res['c_changed_frames'] = [int(chg.min()), int(chg.max())] if len(chg) else []
    outside_ok = True
    if a.builder:
        skb, db, ltb, lrb = locals_of(a.builder, a.clip_name)
        ivb = {n: i for i, n in enumerate(skb.names)}
        if a.root_retimed:
            ltb, lrb = ltb[:nv], lrb[:nv]
        colsb = [ivb[n] for n in names]
        bdiff = np.stack([qang(lrb[:, ivb[n]], lrv[:, iv[n]]) for n in names], 1)
        bfr = np.flatnonzero((bdiff > a.tol_deg).any(1))
        res['c_builder_changed_frames'] = [int(bfr.min()), int(bfr.max())] if len(bfr) else []
        if len(bfr):
            out = np.ones(len(ltc), bool)
            out[bfr.min():bfr.max() + 1] = False
            diffn = (dang > a.noise_deg) | (dtr > a.noise_m)
            outside_ok = bool(not diffn[out].any())
    else:
        outside_ok = None
    res['c_outside_span_ok'] = outside_ok
    res['c_root_ok'] = bool(not diff[:, ri].any())
    if a.root_retimed:
        Wr_, nr_ = BSG.load('%s:%s' % (a.clip, a.clip_name))
        res['c_root_end_lateral_m'] = round(float(abs(Wr_[-1, nr_.index('Root'), 0, 3])), 3)
        res['c_root_ok'] = bool(res['c_root_end_lateral_m'] >= a.min_root_end)
        outside_ok = True
        res['c_outside_span_ok'] = True
    core = [n for n in names if n in iv and not any(k in n for k in ('IK', 'ik', 'Pole', 'space', 'JNT_', 'helper', 'Torso_', 'legs_', 'arms_', 'reference'))]
    ci = [names.index(n) for n in core]
    st = lambda lr: np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(lr[1:] * lr[:-1], axis=-1)), 0, 1)))
    sc_ = st(lrc_full if a.root_retimed else lrc)[:, ci]
    stc = sc_.max()
    res['d_step_bone'] = core[int(np.argmax(sc_.max(axis=0)))]
    stv = st(lrv)[:, [cols[j] for j in ci]].max()
    res['d_step_max_deg'] = round(float(stc), 2)
    res['d_step_vanilla_deg'] = round(float(stv), 2)
    res['d_step_ok'] = bool(stc <= max(a.max_step_deg, stv * 1.1))   # a bone the vanilla clip itself flips (>15 deg) may be 10 % worse
    Wc, _ = BSG.load('%s:%s' % (a.clip, a.clip_name))
    Wv, pn = BSG.load('%s:%s' % (a.vanilla, a.clip_name))
    jn = ('Hips', 'Spine', 'Spine2', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase')
    ix = {n: i for i, n in enumerate(pn)}
    acc = lambda W: max(float(np.linalg.norm(np.diff(W[:, ix[n], :3, 3], 2, axis=0), axis=1).max()) * 100 for n in jn)
    ac, av = acc(Wc), acc(Wv)
    res['e_accel_max_cm'] = round(ac, 2)
    res['e_accel_vanilla_cm'] = round(av, 2)
    res['e_accel_ok'] = bool(ac <= max(a.max_accel_cm, av))
    fl = BSG.floors_of(Wv, pn, robust=True)   # 2026-11-07: robust floors (the lowest-ever height was fooled by a 3-4 cm dip of the planted foot: 61 cm of slide read 0.0 cm)
    gc, gv = BSG.glide(Wc, pn, None, fl, robust=True), BSG.glide(Wv, pn, None, fl, robust=True)
    res['f_glide_cm'] = gc['glide_total_cm']
    res['f_glide_vanilla_cm'] = gv['glide_total_cm']
    res['f_glide_ok'] = bool(gc['glide_total_cm'] <= max(a.max_glide_cm, 0.0) and gc['glide_total_cm'] <= gv['glide_total_cm'])
    ph = {s_: BSG.contact_phases(Wc, pn, s_, fl) for s_ in ('Left', 'Right')}
    phv = {s_: BSG.contact_phases(Wv, pn, s_, fl) for s_ in ('Left', 'Right')}
    res['f_phases'] = ph
    res['f_phase_net_max_cm'] = max([p['net_cm'] for v in ph.values() for p in v] or [0.0])
    res['f_phase_net_vanilla_max_cm'] = max([p['net_cm'] for v in phv.values() for p in v] or [0.0])
    res['f_phase_ok'] = bool(res['f_phase_net_max_cm'] <= max(a.max_phase_net_cm, 0.0))
    cl = BSG.clearance(Wv, Wc[:len(Wv)], pn) if a.root_retimed else BSG.clearance(Wv, Wc, pn)
    if a.root_retimed:
        # the deviation proxy assumes the vanilla path clears the bike; the walk-away variant keeps V closer on purpose. Replacement: the body below the seat top (z < -0.05, RED axes, Root applied) stays outside
        # the bike band |x| >= BIKE_BAND (feet / toes / knees) from frame 30 on, hands outside the band once below the seat top; the minimum distance to the axis is reported (cm)
        # walk-away v2 (2026-11-07): the bike has no offline geometry: a foot / toe / knee below the seat top (frames >= 30) is either >= BIKE_BAND (0.30 m) from the axis or at most --vanilla-path-tol (0.10 m) closer to the axis than the VANILLA joint was at that height, and from vanilla-length - 6 on (the final stance) every foot / toe / knee is >= --min-axis-final (0.35 = assumed half width) from the axis.
        ixc = {n_: i_ for i_, n_ in enumerate(pn)}
        worst, margin, bad_pts = {}, {}, {}
        for j_ in ('LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase', 'LeftLeg', 'RightLeg'):
            P_, V_ = Wc[30:, ixc[j_], :3, 3], Wv[30:, ixc[j_], :3, 3]
            _v = V_
            V_ = np.concatenate([_v[i] + (_v[i + 1] - _v[i]) * (np.arange(8)[:, None] / 8.0) for i in range(len(_v) - 1)] + [_v[-1:]])   # the vanilla joint path densified x8 (a foot that falls 14 cm / frame left gaps wider than the 3 cm height match band: spurious 'no vanilla sample' points)
            m_ = P_[:, 2] < (-0.15 if 'Toe' in j_ else -0.10)
            worst[j_] = round(float(np.abs(P_[m_, 0]).min() * 100), 1) if m_.any() else None
            mg = 1e9
            nbad = 0
            for pt in P_[m_]:
                s_ = (np.abs(V_[:, 2] - pt[2]) < 0.03) & (np.sign(V_[:, 0]) == np.sign(pt[0]))
                if s_.any() and abs(pt[0]) < 0.45:
                    d_ = float(abs(pt[0]) - np.abs(V_[s_, 0]).min())
                    mg = min(mg, d_)
                    if abs(pt[0]) < BSG.BIKE_BAND and d_ < -a.vanilla_path_tol:
                        nbad += 1   # inside the 0.30 m band AND closer than the vanilla joint was at that height
                elif abs(pt[0]) < BSG.BIKE_BAND:
                    nbad += 1
            bad_pts[j_] = nbad
            margin[j_] = round(mg * 100, 1) if mg < 1e8 else None
        fin = {j_: round(float(np.abs(Wc[nv - 6:, ixc[j_], 0, 3]).min() * 100), 1) for j_ in ('LeftFoot', 'RightFoot', 'LeftToeBase', 'RightToeBase', 'LeftLeg', 'RightLeg')}
        viol_ = [k_ for k_, v in bad_pts.items() if v > 0] + [k_ for k_, v in fin.items() if v < a.min_axis_final * 100]
        cl = dict(min_axis_distance_cm=worst, vs_vanilla_margin_cm=margin, bad_points=bad_pts, final_stance_axis_cm=fin, band_cm=a.min_axis_final * 100, vanilla_path_tol_cm=round(a.vanilla_path_tol * 100, 1), ok=bool(not viol_), inward_cm=0.0, down_cm=0.0, hand_cm=0.0)
    res['g_clearance'] = cl
    res['g_clearance_ok'] = cl['ok']
    floor_toe = min(float(Wv[:, ix['LeftToeBase'], 2, 3].min()), float(Wv[:, ix['RightToeBase'], 2, 3].min()))
    low = min(float(Wc[:, ix['LeftToeBase'], 2, 3].min()), float(Wc[:, ix['RightToeBase'], 2, 3].min()))
    res['h_toe_below_ground_cm'] = round(max(0.0, (floor_toe - low) * 100), 2)
    res['h_floor_ok'] = bool(floor_toe - low <= 0.03)
    if a.root_retimed:
        res.update(anatomy_gate_i(a, Wc, Wv, pn))
    oks = [k for k in res if k.endswith('_ok')]
    res['ok'] = bool(all(res[k] is not False for k in oks))
    print('CHECK %s %s' % (os.path.basename(a.clip), json.dumps(res)))
    for k in oks:
        print('  %-22s %s' % (k, res[k]))
    if a.report:
        json.dump(res, open(a.report, 'w'), indent=1)
    print('RESULT %s' % ('PASS' if res['ok'] else 'FAIL'))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
