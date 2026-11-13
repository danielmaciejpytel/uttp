"""Recipes of the traversal / jump gap clips (variants of scenes: source pack pieces + MotionBricks gaps), used by trav_pipe.py.

Every recipe: variants(T, pose_dist, speed) -> [dict(id, items, meta{trim:[t0,t1], ...}, [soma_edit])], trim(sel) -> (t0, t1),
optional extra_score / root_z. Pieces are source pack dumps (poses only are used as seeds; the gap frames are generated).
"""
import math

import numpy as np

CR = 'M_Neutral_Crouch_Idle_Loop'
SIDE = ('Lfoot', 'Rfoot')


def best_starts(T, pose_dist, DA, fa, DB, lo, hi, step, k, spacing):
    cand = sorted((pose_dist(DA, fa, DB, s), s) for s in range(lo, hi, step))
    out = []
    for d, s in cand:
        if all(abs(s - o[1]) >= spacing for o in out):
            out.append((d, s))
        if len(out) == k:
            break
    return out


def gap_dist(T, speed, DA, fa, DB, fb, N, vmin=0.0):
    vA, vB = speed(DA, max(0, fa - 5), fa), speed(DB, fb, min(len(DB.W) - 1, fb + 5))
    return (4 * N - 8 + 1) / 30.0 * 0.5 * (vA + vB) * 100.0, vA, vB


def v_land_crouch_stand(T, pose_dist, speed):
    out = []
    for side in SIDE:
        A = 'M_Neutral_Jump_F_Land_Stand_Heavy_' + side
        DA, DB = T.Dump(A), T.Dump(CR)
        b = 36
        for d, s in best_starts(T, pose_dist, DA, b, DB, 0, 150, 3, 2, 15):
            for N in (6, 7, 8):
                D, vA, vB = gap_dist(T, speed, DA, b, DB, s, N)
                items = [dict(kind='piece', src=A, a=24, b=b, ref=b, xy=(0, 0), tag='A'), dict(kind='gap', N=N),
                         dict(kind='piece', src=CR, a=s, b=s + 40, ref=s, xy=(0, -D), tag='B')]
                n_a = b - 24 + 1
                out.append(dict(id='%s_N%d_s%d' % (side[0], N, s), items=items,
                                meta=dict(family='land_to_crouch', variant='stand', src_a=A, src_b=CR, b_start=s, N=N, dist_cm=D, d_pose=d,
                                          entry_speed_ms=vA, exit_speed_ms=vB, trim=[0, n_a + 4 * N - 8 + 40])))
    return out


def v_land_crouch_run(T, pose_dist, speed):
    out = []
    B = 'M_Neutral_Crouch_Loop_F'
    for side in SIDE:
        A = 'M_Neutral_Jump_F_Land_Run_Light_' + side
        DA, DB = T.Dump(A), T.Dump(B)
        b = 28
        for d, s in best_starts(T, pose_dist, DA, b, DB, 0, 70, 2, 2, 12):
            for N in (7, 8, 9, 10):
                D, vA, vB = gap_dist(T, speed, DA, b, DB, s, N)
                items = [dict(kind='piece', src=A, a=10, b=b, ref=b, xy=(0, 0), tag='A'), dict(kind='gap', N=N),
                         dict(kind='piece', src=B, a=s, b=min(s + 40, len(DB.W) - 1), ref=s, xy=(0, -D), tag='B')]
                n_a = b - 10 + 1
                nb = min(s + 40, len(DB.W) - 1) - s + 1
                out.append(dict(id='%s_N%d_s%d' % (side[0], N, s), items=items,
                                meta=dict(family='land_to_crouch', variant='run', src_a=A, src_b=B, b_start=s, N=N, dist_cm=D, d_pose=d,
                                          entry_speed_ms=vA, exit_speed_ms=vB, trim=[0, n_a + 4 * N - 8 + nb - 1])))
    return out


def v_crouch_jump_stand(T, pose_dist, speed):
    out = []
    for side in SIDE:
        B = 'M_Neutral_Jump_F_Start_Stand_' + side
        DA, DB = T.Dump(CR), T.Dump(B)
        c = 60
        az = np.minimum(DB.W[:, DB.idx['foot_l'], 2, 3], DB.W[:, DB.idx['foot_r'], 2, 3])
        b_end = int(next(f for f in range(24, len(az)) if az[f] < 22.0)) - 1  # last airborne frame: after it the clip's root keeps falling through the take-off floor
        for g0 in (12, 14, 16):
            for N in (6, 7, 8):
                D = 0.0
                items = [dict(kind='piece', src=CR, a=c - 12, b=c, ref=c, xy=(0, 0), tag='A'), dict(kind='gap', N=N),
                         dict(kind='piece', src=B, a=g0, b=b_end, ref=g0, xy=(0, -D), tag='B')]
                n_a = 13
                nb = b_end - g0 + 1
                out.append(dict(id='%s_g%d_N%d' % (side[0], g0, N), items=items,
                                meta=dict(family='jump_from_crouch', variant='stand', src_a=CR, src_b=B, b_start=g0, N=N, dist_cm=D,
                                          entry_speed_ms=0.0, exit_speed_ms=0.0, trim=[0, n_a + 4 * N - 8 + nb - 1])))
    return out


def trim(sel):
    return tuple(sel['trim'])


RECIPES = {
    'LC_stand': dict(variants=v_land_crouch_stand, trim=trim),
    'LC_run': dict(variants=v_land_crouch_run, trim=trim),
    'CJ_stand': dict(variants=v_crouch_jump_stand, trim=trim),
}


# ---------------------------------------------------------------- low hurdle (10-50 cm): run-up | gap | apex window (blend) | gap | run-out
CLEAR_TARGET = 22.0  # cm: aimed lowest ankle height over the obstacle top at the apex (low hurdle)
OB_DEPTH = 40.0  # cm, obstacle footprint along the travel axis
CLEAR = {'LeftFoot': 13.0, 'RightFoot': 13.0, 'LeftToeBase': 5.0, 'RightToeBase': 5.0, 'LeftShin': 10.0, 'RightShin': 10.0,
         'LeftLeg': 25.0, 'RightLeg': 25.0}  # soma joint -> required height over the obstacle top (cm): ankle / toe / knee / hip


def clear_extra(T, sk, Pm, c, g, meta, ground):
    ob = meta.get('obst')
    if not ob or 'fit' not in meta:
        return 0.0, {}
    za, ya = meta['fit']['za'], meta['fit']['ya']
    viol, cmin = 0.0, 1e9
    for j, req in CLEAR.items():
        P = Pm[c:g + 1, sk.idx[j]]
        z = za[0] * P[:, 1] + za[1]
        y = ya[0] * P[:, 2] + ya[1]
        inside = np.abs(y - ob['yc']) < OB_DEPTH / 2 + 6.0
        if inside.any():
            m = float((z[inside] - ob['h'] - req).min())
            cmin = min(cmin, m)
            viol = max(viol, -m)
    return 0.4 * viol, dict(clear_min=round(cmin if cmin < 1e8 else 0.0, 1))


def v_hurdle(T, pose_dist, speed, heights=(25, 45), gait='run', Ns=((6, 6), (7, 7)), sides=SIDE, sfun=None):
    out = []
    loop = 'M_Neutral_Run_Loop_F' if gait == 'run' else 'M_Neutral_Walk_Loop_F'
    DL = T.Dump(loop)
    for side in sides:
        H = 'M_Neutral_Traversal_Hurdle_1_0_%s_F_V2_%s' % (gait, side)
        DH = T.Dump(H)
        m = int(DH.W[:, DH.pel, 2, 3].argmax())
        vH = speed(DH, m - 2, m + 3)
        for N1, N2 in Ns:
            fh = m - 2 - (4 * N1 - 7) - 3
            if fh < 0:
                continue
            cA = min(range(14, len(DL.W) - 4), key=lambda c: pose_dist(DL, c, DH, fh))
            vA = speed(DL, cA - 5, cA)
            # run window matching the apex window (phase / lead leg), 6 frames
            r = min(range(0, len(DL.W) - 6), key=lambda r: pose_dist(DL, r + 3, DH, m - 2))
            t_b = m + 3 + (4 * N2 - 7)
            if t_b >= len(DH.W):
                continue
            sB = min(range(0, len(DL.W) - 26), key=lambda s: pose_dist(DH, t_b - 1, DL, s))
            vB = speed(DL, sB, sB + 5)
            D1 = (4 * N1 - 7) / 30.0 * 0.5 * (vA + vH) * 100.0
            ym = -(D1 + 2 * vH / 30.0 * 100.0)
            ye = ym - 3 * vH / 30.0 * 100.0
            D2 = (4 * N2 - 7) / 30.0 * 0.5 * (vH + vB) * 100.0
            yB = ye - D2
            Wh = T.place_window(DH, m - 2, m + 3, m, (0, ym))
            Wr = T.place_window(DL, r, r + 5, r + 2, (0, ym))
            for h in heights:
                s = sfun(h / 100.0) if sfun else min(1.0, 0.25 + h / 100.0)
                Wb = T.blend_windows(Wr, Wh, DH.parents, s)
                zs = {}
                ix = DH.idx
                req = {'foot_l': 13.0, 'foot_r': 13.0, 'ball_l': 5.0, 'ball_r': 5.0, 'calf_l': 10.0, 'calf_r': 10.0, 'thigh_l': 25.0, 'thigh_r': 25.0}
                dz = max(0.0, max(h + rq - Wb[:, ix[j], 2, 3].min() for j, rq in req.items()))
                Wb = Wb.copy()
                Wb[:, :, 2, 3] += dz
                items = [dict(kind='piece', src=loop, a=cA - 12, b=cA, ref=cA, xy=(0, 0), tag='A'), dict(kind='gap', N=N1),
                         dict(kind='raw', W=Wb, tag='X', src='blend:%s[%d:%d]+%s[%d:%d]' % (H, m - 2, m + 3, loop, r, r + 5)), dict(kind='gap', N=N2),
                         dict(kind='piece', src=loop, a=sB, b=sB + 25, ref=sB, xy=(0, yB), tag='B')]
                n_total = 13 + (4 * N1 - 8) + 6 + (4 * N2 - 8) + 26
                out.append(dict(id='%s_h%d_N%d%d' % (side[0], h, N1, N2), items=items,
                                meta=dict(family='low_hurdle', gait=gait, src_apex=H, src_run=loop, h_cm=h, blend_s=s, lift_cm=dz, N=[N1, N2],
                                          obst=dict(h=h, yc=ym - 1.0 * vH / 30.0 * 100.0 * 0.0, depth=OB_DEPTH), entry_speed_ms=vA, exit_speed_ms=vB,
                                          apex_frames=[13 + 4 * N1 - 8, 13 + 4 * N1 - 8 + 5], trim=[0, n_total - 1])))
    return out


RECIPES['LH_run'] = dict(variants=lambda T, pd, sp: v_hurdle(T, pd, sp, (25, 45), 'run', ((6, 6), (7, 7))), trim=trim, extra_score=clear_extra)
RECIPES['LH_walk'] = dict(variants=lambda T, pd, sp: v_hurdle(T, pd, sp, (25,), 'walk', ((6, 6), (7, 7))), trim=trim, extra_score=clear_extra)

RECIPES['LHD_real'] = dict(variants=lambda T, pd, sp: v_hurdle(T, pd, sp, (60,), 'run', ((6, 6), (7, 7)), sfun=lambda h: 1.0), trim=trim, extra_score=clear_extra)
RECIPES['LHD_s3'] = dict(variants=lambda T, pd, sp: v_hurdle(T, pd, sp, (15, 25), 'run', ((6, 8), (7, 9)), sfun=lambda h: 0.3), trim=trim, extra_score=clear_extra)


def v_hurdle2(T, pose_dist, speed, heights, gait, Ns, smax_fun, sides=SIDE):
    """Low hurdle v2: the REAL flight of source pack Hurdle_1_0_<gait>_F_V2 (take-off plant .. 2 frames after touch-down, frames m-6..m+15, m = pelvis
    apex) is reduced with a smooth bump: local-space blend toward the phase-matched run / walk loop (s(t) = smax * bump) and a lift
    dz * bump that only restores the obstacle clearance. Run-up and run-out are MotionBricks gaps to the real loop (A / B)."""
    out = []
    loop = 'M_Neutral_Run_Loop_F' if gait == 'run' else 'M_Neutral_Walk_Loop_F'
    DL = T.Dump(loop)
    req = {'foot_l': 13.0, 'foot_r': 13.0, 'ball_l': 5.0, 'ball_r': 5.0, 'calf_l': 10.0, 'calf_r': 10.0, 'thigh_l': 25.0, 'thigh_r': 25.0}
    for side in sides:
        H = 'M_Neutral_Traversal_Hurdle_1_0_%s_F_V2_%s' % (gait, side)
        DH = T.Dump(H)
        m = int(DH.W[:, DH.pel, 2, 3].argmax())
        x0, x1 = m - 6, m + 15
        nx = x1 - x0 + 1
        vH = speed(DH, m - 2, m + 3)
        bump = np.sin(np.linspace(0, np.pi, nx)) ** 2
        # obstacle centre = middle of the airborne interval of the source hurdle (both ankles > 30 cm), not the pelvis apex (which is early)
        Wprobe = T.place_window(DH, x0, x1, x0, (0, 0))
        air = np.where(np.minimum(Wprobe[:, DH.idx['foot_l'], 2, 3], Wprobe[:, DH.idx['foot_r'], 2, 3]) > 30.0)[0]
        ap = int(round(0.5 * (air.min() + air.max())))
        for N1, N2 in Ns:
            fh = x0 - (4 * N1 - 7) - 3
            t_b = x1 + (4 * N2 - 7)
            if fh < 0 or t_b >= len(DH.W):
                continue
            cA = min(range(14, len(DL.W) - 4), key=lambda c: pose_dist(DL, c, DH, fh))
            vA = speed(DL, cA - 5, cA)
            ankz = np.minimum(DL.W[:, DL.idx['foot_l'], 2, 3], DL.W[:, DL.idx['foot_r'], 2, 3])
            r = min(range(0, len(DL.W) - nx), key=lambda r: pose_dist(DL, r + 3, DH, x0) + 4.0 * max(0.0, 16.0 - float(ankz[r + ap - 2:r + ap + 3].min())))
            sB = min(range(0, len(DL.W) - 26), key=lambda s: pose_dist(DH, t_b - 1, DL, s))
            vB = speed(DL, sB, sB + 5)
            D1 = (4 * N1 - 7) / 30.0 * 0.5 * (vA + vH) * 100.0
            Wh = T.place_window(DH, x0, x1, x0, (0, -D1))
            ym = float(Wh[ap, DH.pel, 1, 3])
            Wr = T.place_window(DL, r, r + nx - 1, r, (0, -D1))
            D2 = (4 * N2 - 7) / 30.0 * 0.5 * (vH + vB) * 100.0
            yB = float(Wh[-1, DH.pel, 1, 3]) - D2
            ix = DH.idx

            def make(smax):
                return T.blend_windows(Wr, Wh, DH.parents, smax * bump)  # per-frame weights, continuous rotation path

            def lowest_ankle(Wb):
                m_ = 1e9
                for j in ('foot_l', 'foot_r'):
                    ins = np.abs(Wb[:, ix[j], 1, 3] - ym) < OB_DEPTH / 2 + 6.0
                    if ins.any():
                        m_ = min(m_, float(Wb[ins, ix[j], 2, 3].min()))
                return m_

            cache = {}
            for h in heights:
                if smax_fun is None:  # solve the blend so that the lower ankle passes ~clear_cm over the obstacle top at the apex
                    lo, hi = 0.0, 1.0
                    tgt = h + CLEAR_TARGET
                    for _ in range(6):
                        mid = 0.5 * (lo + hi)
                        if lowest_ankle(make(mid)) < tgt:
                            lo = mid
                        else:
                            hi = mid
                    smax = hi if lowest_ankle(make(0.0)) < tgt else 0.0
                else:
                    smax = smax_fun(h / 100.0)
                Wb = make(smax)
                dz = 0.0
                for j, rq in req.items():
                    z = Wb[:, ix[j], 2, 3]
                    y = Wb[:, ix[j], 1, 3]
                    inside = np.abs(y - ym) < OB_DEPTH / 2 + 6.0
                    for i in np.where(inside & (bump > 0.05))[0]:
                        dz = max(dz, (h + rq - z[i]) / max(bump[i], 0.3))
                dz = max(dz, 0.0)
                Wb = Wb.copy()
                Wb[:, :, 2, 3] += dz * bump[:, None]
                items = [dict(kind='piece', src=loop, a=cA - 12, b=cA, ref=cA, xy=(0, 0), tag='A'), dict(kind='gap', N=N1),
                         dict(kind='raw', W=Wb, tag='X', src='blend:%s[%d:%d]+%s[%d:%d]' % (H, x0, x1, loop, r, r + nx - 1)), dict(kind='gap', N=N2),
                         dict(kind='piece', src=loop, a=sB, b=sB + 25, ref=sB, xy=(0, yB), tag='B')]
                n_total = 13 + (4 * N1 - 8) + nx + (4 * N2 - 8) + 26
                out.append(dict(id='%s_h%d_N%d%d' % (side[0], h, N1, N2), items=items,
                                meta=dict(family='low_hurdle', gait=gait, src_apex=H, src_run=loop, h_cm=h, blend_smax=smax, lift_cm=dz, N=[N1, N2],
                                          obst=dict(h=h, yc=ym, depth=OB_DEPTH), entry_speed_ms=vA, exit_speed_ms=vB,
                                          flight_frames=[13 + 4 * N1 - 8, 13 + 4 * N1 - 8 + nx - 1], obst_center_idx=ap, trim=[0, n_total - 1])))
    return out


RECIPES['LH2_run'] = dict(variants=lambda T, pd, sp: v_hurdle2(T, pd, sp, (15, 25, 40), 'run', ((6, 6), (6, 8)), None, sides=('Rfoot',)), trim=trim, extra_score=clear_extra)
RECIPES['LH2_walk'] = dict(variants=lambda T, pd, sp: v_hurdle2(T, pd, sp, (15, 30), 'walk', ((6, 6), (6, 7)), None), trim=trim, extra_score=clear_extra)
