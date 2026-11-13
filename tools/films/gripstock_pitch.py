"""Rifle grip + stock 2026-11-10, round 7 - weapon / stock penetration into the skin at aim pitch -40 / 0 / +40 (game mesh vs skinned body).
Same method as gripstock_bodycheck.py: skin vertices of the torso (Hips / Spine* / Neck* / Head / shoulders = chest + shoulder) and of the
upper arms inside the real weapon mesh (nearest face sign for points within 6 cm of a mesh vertex; the worst frame of each region
cross-checked with the generalized winding number, gripstock_common.LocalWinding).  Forearms / hands hold the weapon and are excluded.
Timeline: ready 0.6 s, aim + 3 shots 2.4 s (as the accepted pitch-0 table: 'film' timeline aim phase), the aim point at the given pitch for
the whole run (target y 25 m, z = 1.4 + 25 tan(pitch); pitch 0 = the default target z 1.45).  Modes: before = gripstock_eval_before
(state before the fix = without the feature), after = gripstock_eval_after (current tree).
  python -I gripstock_pitch.py <family> <bodies woman,man> <fems 0.025[,0,0.05]> <pitches -40,0,40> [--every N]
-> stdout + Build\\work\\rifle-grip-stock-20261110\\r7_pitch_<family>_<bodies>.json
"""
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripstock_common as G  # noqa: E402
import weapons_common as WC  # noqa: E402
from gripstock_bodycheck import TORSO, Inside  # noqa: E402

PARTS = {'copperhead': ('nokota_copperhead__base1_01.glb', 'nokota_copperhead__mag_std_01.glb', 'nokota_copperhead__rcv1_01.glb',
                        'nokota_copperhead__rcv1_01_stock_01.glb')}  # = the accepted round-4 table (bodycheck_copperhead_*.log header)


def timeline(rec, fem, pitch):
    tz = 1.45 if pitch == 0 else 1.4 + 25.0 * math.tan(math.radians(pitch))
    tl = WC.Timeline('Rifle', '-', (0, fem), (0.0, 25.0, tz))
    marks, shots = [], []

    def seg(sec, aim, clip=None, label=''):
        if clip is not None and clip != tl.clip:
            tl.switch_clip(clip, 0.0, 0.3)
        marks.append((tl.now_step() * WC.DT, label))
        tl.run(sec, aim=aim, moving=0)

    seg(0.6, 'ready', WC.IDLE, 'ready')
    t = tl.now_step() * WC.DT
    seg(2.4, 'aim', None, 'aim + shots')
    shots += [t + 0.6, t + 1.0, t + 1.4]
    for r in tl.rows:
        r[11] = rec
    return tl, shots, marks


def main():
    a = list(sys.argv[1:])
    every = 2
    if '--every' in a:
        i = a.index('--every')
        every = int(a[i + 1])
        del a[i:i + 2]
    fam, bodies = a[0], a[1].split(',')
    fems = [float(x) for x in a[2].split(',')]
    pitches = [int(x) for x in a[3].split(',')]
    if fam in PARTS:
        G.MESHDIR[fam] = (G.MESHDIR[fam][0], PARTS[fam])
    V, F, files = G.weapon_mesh(fam)
    print('mesh', [os.path.basename(f) for f in files], len(V), 'vertices', len(F), 'faces', flush=True)
    ins = Inside(V, F)
    lw = G.LocalWinding(V, F)
    sc = json.load(open(os.path.join(G.WORK, 'stock_calib.json')))
    contact = sc.get(fam, {}).get('contact_y')
    rec = '%s_single_shot_ironsight.utpprecoil' % fam
    out = {}
    lo, hi = V.min(0) - 0.01, V.max(0) + 0.01
    for body in bodies:
        P = G.player(body)
        ix, names = P.ix, P.names
        joints = [j for j in range(73) if names[j] not in ('Root', 'Trajectory', 'reference_joint', 'WeaponLeft', 'WeaponRight', 'LeftEye',
                                                           'RightEye')]
        reg = np.array([0 if names[j] in TORSO else 1 if names[j] == 'RightArm' else 2 if names[j] == 'LeftArm' else 3 for j in joints])
        for fem in (fems if body == 'woman' else [0.0]):
            for pitch in pitches:
                for mode in ('before', 'after'):
                    tl, shots, marks = timeline(rec, fem, pitch)
                    name = 'r7_%s_%s_%s_%g_p%+d' % (fam, mode, body, fem, pitch)
                    r = G.run(mode, name, tl, shots, body, G.PROFILE[fam])
                    W = r['W']
                    T = len(W)
                    lab = G.phase_of(marks, T)
                    aim = [f for f in range(T) if lab[f] == 'aim + shots']
                    steady = aim[len(aim) // 3:]
                    frames = aim[::every]
                    B = G.skin(W[frames], body, os.path.join(G.TMP, name + '_skin'))
                    M = W[:, ix['WeaponRight']]
                    dep = np.zeros((len(frames), 3))
                    nin = np.zeros((len(frames), 3), int)
                    pts = {}
                    for i, f in enumerate(frames):
                        _, own = cKDTree(W[f, joints, :3, 3]).query(B[i])
                        rg = reg[own]
                        Bw = (B[i] - M[f, :3, 3]) @ M[f, :3, :3]
                        box = np.all((Bw > lo) & (Bw < hi), 1) & (rg < 3)
                        if not box.any():
                            continue
                        d = ins.depth(Bw[box])
                        for k in range(3):
                            s = (rg[box] == k) & (d > 0)
                            if s.any():
                                nin[i, k] = int(s.sum())
                                dep[i, k] = float(d[s].max() * 100)
                                pts[(i, k)] = Bw[box][s]
                    st = np.array([f in steady for f in frames])
                    res = {}
                    for k, rn in enumerate(('torso', 'rarm', 'larm')):
                        i = int(np.argmax(dep[:, k]))
                        wn = lw(pts[(i, k)]) if (i, k) in pts else np.zeros(0)
                        res[rn] = dict(max=round(float(dep[:, k].max()), 2), median_steady=round(float(np.median(dep[st, k])), 2),
                                       max_steady=round(float(dep[st, k].max()), 2), verts_max=int(nin[:, k].max()), worst_frame=frames[i],
                                       worst_inside_face=int(len(wn)), worst_inside_winding=int(np.sum(wn > 0.5)))
                    d = r['diag']
                    sg = np.asarray(d['stockGap'], float)[steady] * 100
                    rs = np.asarray(d['reachShift'], float)[steady] * 100
                    pg = None
                    if contact is not None:
                        pk = np.array(sc['pocket_' + body]['local'])
                        RA = W[steady][:, ix['RightArm']]
                        pocket = RA[:, :3, 3] + np.einsum('tij,j->ti', RA[:, :3, :3], pk)
                        Ms = M[steady]
                        pg = (np.einsum('ti,ti->t', Ms[:, :3, 3] - pocket, Ms[:, :3, 1]) + contact) * 100
                        # the shoulder pocket's distance from the stock centre line (barrel axis through WeaponRight) = the test's
                        # PocketOffAxisCm
                        rel = pocket - Ms[:, :3, 3]
                        perp = rel - np.einsum('ti,ti->t', rel, Ms[:, :3, 1])[:, None] * Ms[:, :3, 1]
                        res['pocket_off_axis_max'] = round(float(np.linalg.norm(perp, axis=1).max() * 100), 2)
                    m = G.measure(W[steady], None, body, fam)
                    res['grip_cm_max'] = round(float(np.nanmax(m['grip_err_cm'])), 2)
                    res['dll_stock_gap_min'] = round(float(sg.min()), 2) if mode == 'after' else None
                    res['reach_shift_max'] = round(float(rs.max()), 2) if mode == 'after' else None
                    res['pocket_gap_min'] = round(float(pg.min()), 2) if pg is not None else None
                    key = '%s|fem %g|p %+d|%s' % (body, fem, pitch, mode)
                    out[key] = res
                    print(key, json.dumps(res), flush=True)
    WC.jdump(out, os.path.join(G.WORK, 'r7_pitch_%s_%s.json' % (fam, '_'.join(bodies))))


if __name__ == '__main__':
    main()
