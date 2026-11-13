"""Walk-away variant of a MOTORCYCLE left get-off (exit_l): V stays where she planted the left foot during the leg-over, then takes two real steps away from the bike (offline, no game).

  python -I bike_exit_walk.py --glb <vanilla tpp_p?a_sportbike glb> --clip exit_l --out <pick.anims.glb> [--root-end 0.80] [--frames 96] [--report r.json]

Why: the vanilla clip moves Root 1.14 m while the skeleton keeps the support foot nearly still (it glides); the previous fix (bike_exit_replant.py) pinned the foot and hopped ~1 m. Pinned at its touchdown spot the foot is only
~0.5 m from the bike axis, which is inside the player capsule + bike body (bike half width 0.35 + capsule 0.40 = Root >= 0.75 m), so after the leg-over V walks away.
  * frames 0..58: the vanilla locals (hips, spine, arms, head bit for bit) with a NEW Root path (the vanilla one until --decel-from, then an ease-out that stops with both feet down at the touchdown spot);
    left foot: plant (vanilla touch-down spot) - short re-place while still seated - plant at the spot the leg reaches (vanilla frame --left-ref); right foot: the vanilla swing, then outboard first at leg-over height, then down (never through the bike),
    landing beside the left foot (--right-dx inboard, --right-dy behind); both pinned (position + world rotation) = no slide;
  * frames 59..N-1 (default 96): vanilla last-frame upper body held; step 1 left foot (lift --apex-cm, smooth, zero speed at both ends), step 2 right foot; hips x = mean of the two feet, i.e. Root is DERIVED from the pinned feet
    (no gliding), hips lowered where the legs would overstretch (reach 0.985 x leg length), final stance clear of the bike (Root x = --root-end);
  * two-bone leg IK on the new hips (ik.two_bone, knee pole near the natural knee), limb helpers re-anchored (exit_steps.reanchor_helpers + place_leg_poles of bike_exit_replant).
Output: GLB with N frames (clip name unchanged), Root retimed -> build_bike_exit.ps1 -RootRetimed (motion extraction re-encoded from the GLB Root, check_bike_exit.py --root-retimed).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bike_exit_replant as R  # noqa: E402
import skel as S  # noqa: E402
import ik as IK  # noqa: E402
import build_entry_clip as B  # noqa: E402
import exit_steps as ES  # noqa: E402
import bike_steps_gen as BSG  # noqa: E402

sm = R.smooth01


def ease_out_progress(p_s, v0, f_s, R_end, f):
    """lateral progress p(f): the vanilla speed v0 at frame f_s decelerates (constant deceleration) to rest at R_end"""
    D = R_end - p_s
    T = 2.0 * D / v0
    t = np.clip(f - f_s, 0, T)
    return p_s + v0 * t - v0 * t * t / (2.0 * T), f_s + T


def segment(Tp, Tr, f0, f1, PA, RA, PB, RB, apex=0.0, delay=0.08, lift=0.3):
    """frames f0+1..f1-1 of a move from (PA, RA) at f0 to (PB, RB) at f1 (xy smooth, height arc with zero speed at both ends)"""
    for f in range(f0 + 1, f1):
        u = (f - f0) / float(f1 - f0)
        s = sm((u - delay) / (1.0 - 2.0 * delay))
        Tp[f, :2] = (1 - s) * PA[:2] + s * PB[:2]
        Tp[f, 2] = (1 - sm(u)) * PA[2] + sm(u) * PB[2] + apex * sm(u / lift) * sm((1.0 - u) / lift)
        Tr[f] = R.slerp_mats(RA, RB, sm(u))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb', required=True)
    p.add_argument('--clip', default='exit_l')
    p.add_argument('--out', required=True)
    p.add_argument('--report')
    p.add_argument('--frames', type=int, default=96)
    p.add_argument('--root-end', type=float, default=0.80, help='m: final Root lateral offset from the seat origin (bike half width 0.35 + capsule 0.40 = 0.75 minimum)')
    p.add_argument('--decel-from', type=int, default=30)
    p.add_argument('--left-ref', type=int, default=22, help='frame whose vanilla left ankle spot is the leg-over plant (the farthest the leg reaches while the hips are seated)')
    p.add_argument('--left-hop', default='16:24', help='re-place of the left foot while still seated: lift f0, plant from f1')
    p.add_argument('--right-dx', type=float, default=0.12)
    p.add_argument('--right-dy', type=float, default=0.29)
    p.add_argument('--right-keys', default='38:45:51', help='right foot: end of the natural swing / outboard at height / landed')
    p.add_argument('--steps', default='54:68:68:82', help='left step f0:f1, right step f0:f1')
    p.add_argument('--apex-cm', type=float, default=7.0)
    p.add_argument('--reach', type=float, default=0.985)
    p.add_argument('--max-extend', type=float, default=0.9998)
    a = p.parse_args()
    rig = R.Rig(a.glb, a.clip)
    tgt = rig.tgt
    n0, N = rig.n, a.frames
    ri = tgt.index['Root']
    assert tgt.parent[ri] == -1 and a.clip.endswith('_l')
    out_sign = -1.0   # left = -X
    # ---- extended locals: frames >= n0 hold the vanilla last frame
    lt = np.concatenate([rig.lt0, np.repeat(rig.lt0[-1:], N - n0, 0)])
    lr = np.concatenate([rig.lr0, np.repeat(rig.lr0[-1:], N - n0, 0)])
    times = np.arange(N) / 30.0
    # ---- Root: vanilla lateral until decel-from, ease-out to rest at R1 (set below from the planted feet), then derived from the feet
    rx0 = rig.lt0[:, ri, 0].copy()
    prog = out_sign * rx0
    fs = a.decel_from
    v0 = float(prog[fs] - prog[fs - 1])
    # vanilla world foot data (root unchanged before fs, so these are also the new world positions there)
    WtR = R.red(rig.Wt0)
    LF, RF = tgt.index['LeftFoot'], tgt.index['RightFoot']
    P_L = WtR[a.left_ref, LF].copy()
    Rl_ref = rig.Wr0[a.left_ref, LF].copy()
    P_L1 = WtR[9, LF].copy()
    Rl_1 = rig.Wr0[9, LF].copy()
    R_fin_l = rig.Wr0[n0 - 1, LF].copy()
    R_fin_r = rig.Wr0[n0 - 1, RF].copy()
    z_ground_l = WtR[n0 - 1, LF, 2]
    z_ground_r = WtR[n0 - 1, RF, 2]
    P_R = np.array([P_L[0] - out_sign * a.right_dx, P_L[1] - a.right_dy, z_ground_r])   # inboard of the left foot, behind it
    # end stance from Root end: hips mean of the feet; final left/right foot x with the vanilla stance width 0.18 (right inboard)
    hips_local_x = float(WtR[n0 - 1, tgt.index['Hips'], 0] - rig.lt0[n0 - 1, ri, 0])
    hips_end = out_sign * a.root_end + hips_local_x
    width = float(WtR[n0 - 1, RF, 0] - WtR[n0 - 1, LF, 0])      # right inboard of the left foot (+x)
    L3x = hips_end - width / 2.0
    R3x = hips_end + width / 2.0
    L3 = np.array([L3x, P_L[1], z_ground_l])
    R3 = np.array([R3x, P_L[1] - 0.19, z_ground_r])
    # R1: root where both feet are down (hips over the mean of the two feet)
    mean_x1 = 0.5 * (P_L[0] + P_R[0])
    R1 = out_sign * (mean_x1 - hips_local_x)
    R1 = abs(R1)
    p_new, f_stop = ease_out_progress(prog[fs], v0, fs, R1, np.arange(N, dtype=float))
    prog_new = np.where(np.arange(N) <= fs, np.concatenate([prog, np.full(N - n0, prog[-1])])[:N], p_new)
    rxn = out_sign * prog_new
    lt[:, ri, 0] = rxn
    # ---- natural world (new root) of both feet, RED axes
    Wt1, Wr1 = tgt.world(lt, lr)
    Wr = R.red(Wt1)
    ap = {s: (Wr[:, tgt.index[s + 'Foot']].copy(), Wr1[:, tgt.index[s + 'Foot']].copy()) for s in ('Left', 'Right')}
    # ---- left foot targets
    Tp, Tr = ap['Left'][0].copy(), ap['Left'][1].copy()
    t0, ramp = 6, 3
    h0, h1 = (int(x) for x in a.left_hop.split(':'))
    Tp[t0:h0 + 1] = P_L1
    Tr[t0:h0 + 1] = Rl_1
    for f in range(t0 - ramp, t0):
        w = sm((f - (t0 - ramp) + 1) / float(ramp + 1))
        Tp[f, :2] = (1 - w) * ap['Left'][0][f, :2] + w * P_L1[:2]
        Tr[f] = R.slerp_mats(ap['Left'][1][f], Rl_1, w)
    for f in range(t0, t0 + 4):
        w = sm((f - t0 + 1) / 4.0)
        Tp[f, 2] = (1 - w) * ap['Left'][0][f, 2] + w * P_L1[2]
    ls0, ls1, rs0, rs1 = (int(x) for x in a.steps.split(':'))
    segment(Tp, Tr, h0, h1, P_L1, Rl_1, P_L, Rl_ref, apex=a.apex_cm / 100.0)
    Tp[h1:ls0 + 1] = P_L
    Tr[h1:ls0 + 1] = Rl_ref
    segment(Tp, Tr, ls0, ls1, P_L, Rl_ref, L3, R_fin_l, apex=a.apex_cm / 100.0)
    Tp[ls1:] = L3
    Tr[ls1:] = R_fin_l
    # ---- right foot targets: natural swing until fa, outboard at height until fb, down to the landing spot at fc, pinned, step 2
    fa, fb, fc = (int(x) for x in a.right_keys.split(':'))
    Rp, Rr = ap['Right'][0].copy(), ap['Right'][1].copy()
    A_ = Rp[fa].copy()
    B_ = np.array([P_R[0], P_R[1], min(A_[2], -0.20)])
    for f in range(fa + 1, fb + 1):
        u = (f - fa) / float(fb - fa)
        s = sm(u)
        Rp[f] = (1 - s) * A_ + s * B_
        Rp[f, 2] = A_[2] + (B_[2] - A_[2]) * sm((u - 0.70) / 0.30)   # stays at leg-over height (>= natural) until the foot is past the bike side, only then down
        Rr[f] = R.slerp_mats(ap['Right'][1][fa], R_fin_r, s)
    for f in range(fb + 1, fc + 1):
        u = (f - fb) / float(fc - fb)
        s = sm(u)
        Rp[f] = (1 - s) * B_ + s * P_R
        Rr[f] = R_fin_r
    Rp[fc:rs0 + 1] = P_R
    Rr[fc:rs0 + 1] = R_fin_r
    segment(Rp, Rr, rs0, rs1, P_R, R_fin_r, R3, R_fin_r, apex=a.apex_cm / 100.0)
    Rp[rs1:] = R3
    Rr[rs1:] = R_fin_r
    # ---- hips: x = mean of the two feet from the frame both are down, Root derived (no gliding)
    hips_now = Wr[:, tgt.index['Hips']]
    mean_x = 0.5 * (Tp[:, 0] + Rp[:, 0])
    fm = fc
    w_h = np.clip((np.arange(N) - (fc - 6)) / 6.0, 0, 1)
    w_h = w_h * w_h * (3 - 2 * w_h)
    # root x needed so that the hips are at the mean: rx = rx_now + (mean - hips_now)
    rx_derived = rxn + (mean_x - hips_now[:, 0])
    rxn2 = np.where(np.arange(N) >= fm, rx_derived, rxn)
    # smooth blend over the 6 frames before fc (the ease-out already ends at R1 = the mean at fc: no jump)
    lt[:, ri, 0] = rxn2
    # ---- hips lowering where a leg would overstretch (needs the world after the new Root): per frame max over the feet, then smoothed
    Wt2, Wr2 = tgt.world(lt, lr)
    W2 = R.red(Wt2)
    uleg = {s: W2[:, tgt.index[s + 'UpLeg']] for s in ('Left', 'Right')}
    Lleg = float(np.linalg.norm(rig.Wt0[0, tgt.index['LeftLeg']] - rig.Wt0[0, tgt.index['LeftUpLeg']]) + np.linalg.norm(rig.Wt0[0, LF] - rig.Wt0[0, tgt.index['LeftLeg']]))
    need = np.zeros(N)
    for s_, T_ in (('Left', Tp), ('Right', Rp)):
        d = uleg[s_] - T_
        h = np.hypot(d[:, 0], d[:, 1])
        vmax = np.sqrt(np.maximum((a.reach * Lleg) ** 2 - h ** 2, 0.0))
        need = np.maximum(need, np.maximum(d[:, 2] - vmax, 0.0))
    need[:fa] = 0.0   # the leg-over keeps the vanilla hips
    k = np.ones(7) / 7.0
    nm = np.maximum.reduce([np.roll(need, i) for i in (-3, -2, -1, 0, 1, 2, 3)])
    need_s = np.convolve(np.pad(nm, 3, mode='edge'), k, mode='valid')
    need_s[0] = 0.0
    hi = tgt.index['Hips']
    lt[:, hi, 1] -= need_s   # glTF y = up (hips' parent Root has no tilt)
    # ---- leg IK on the final hips
    rep = dict(root_end=float(a.root_end), R1=float(R1), frames=N, hips_lowering_max_cm=round(float(need_s.max()) * 100, 1), left_plant=[float(x) for x in P_L], right_plant=[float(x) for x in P_R],
               final_left=[float(x) for x in L3], final_right=[float(x) for x in R3])
    Wt3, Wr3 = tgt.world(lt, lr)
    for sd, (T_, R_) in (('Left', (Tp, Tr)), ('Right', (Rp, Rr))):
        a_, b_, c_ = (tgt.index[sd + x] for x in ('UpLeg', 'Leg', 'Foot'))
        # knee pole: the natural knee while the leg follows the vanilla swing / plant, a point in front of the hip (knees forward) from the right-foot descent / the steps on (a flipping natural knee gave a 60 deg / frame knee step)
        w0_, w1_ = (fa - 2, fa + 4) if sd == 'Right' else (ls0 - 6, ls0)
        w_ = sm((np.arange(N) - w0_) / float(w1_ - w0_))[:, None]
        pole_fwd = Wt3[:, a_] + R.gltf(np.array([0.0, 0.6, 0.0]))
        err = IK.two_bone(tgt, lt, lr, a_, b_, c_, R.gltf(T_), max_extend=a.max_extend, bend_sigma=2.0, pole=(1 - w_) * Wt3[:, b_] + w_ * pole_fwd)
        IK.set_world_rot(tgt, lt, lr, c_, R_)
        rep['ik_max_err_cm_' + sd] = round(float(err) * 100, 2)
    keep = np.zeros(N, bool)
    keep[:t0 - ramp] = True
    lt[keep, :], lr[keep, :] = np.concatenate([rig.lt0, np.repeat(rig.lt0[-1:], N - n0, 0)])[keep], np.concatenate([rig.lr0, np.repeat(rig.lr0[-1:], N - n0, 0)])[keep]
    lt[keep, ri, 0] = rxn2[keep]
    # helpers: vanilla offsets on the new limbs (feet, knees, arms: the arms moved with Root / hips)
    vWt = np.concatenate([rig.Wt0, np.repeat(rig.Wt0[-1:], N - n0, 0)])
    vWr = np.concatenate([rig.Wr0, np.repeat(rig.Wr0[-1:], N - n0, 0)])
    ES.reanchor_helpers(tgt, lt, lr, vWt, vWr, t0 - ramp - 1, N - 1)
    R.place_leg_poles(tgt, lt, lr, type('RigV', (), dict(Wt0=vWt, Wr0=vWr))(), ['Left', 'Right'], t0 - ramp, N - 2)
    lr[:] = S.qcontinuous(lr)
    # start frames bit for bit
    rep['first_frames_identical'] = bool(np.all(lt[:t0 - ramp] == np.concatenate([rig.lt0, np.repeat(rig.lt0[-1:], N - n0, 0)])[:t0 - ramp]))
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
    print(json.dumps({k_: v for k_, v in rep.items() if k_ != 'phases'}, default=str)[:1500])
    print('PHASES', {s: [(q['f0'], q['f1'], q['net_cm']) for q in v] for s, v in rep['phases'].items()})


if __name__ == '__main__':
    main()
