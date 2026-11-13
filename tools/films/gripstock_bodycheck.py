"""Rifle grip + stock 2026-11-10 - weapon against the body, per frame, before / after (design choices: Hercules reach shift up
to its 10 cm cap; Copperhead hand priority vs stock contact). Film timeline (relaxed, ready, aim + shots, ready, reload stand-in, walk,
run, sprint, stop), Female V (width 0.025) and Male V:
  grip_err_cm, left / right elbow flexion (deg, 0 = straight), reach shift / need and stock gap / shift (DLL diag), and the real game
  mesh of the weapon (WolvenKit glb) against the skinned body: skin vertices inside the weapon by region (torso = Hips / Spine* / Neck* /
  Head / shoulders, upper arms = LeftArm / RightArm; forearms and hands hold the weapon and are excluded) with the max depth; per-frame
  steps of the left hand and of the weapon origin in the Spine3 frame (transitions), after against before.
  python -I gripstock_bodycheck.py <family> <mesh parts, comma> [bodies=woman,man] [--nostock] [--every N] [--fem W] [--tag T]
    regions: torso, R upper arm (RightArm), L upper arm (LeftArm); pocketGap = the DLL's stock gap along the barrel on the OUTPUT pose
    (pocket = RightArm frame + stock_calib.json pocket, contact = stock_calib.json contact_y; < 0 = contact past the pocket point);
    the worst frame of each region is cross-checked with the generalized winding number (gripstock_common.LocalWinding).
    mesh parts: glb file name suffixes in MESHDIR[family]'s folder (hercules: Extracted\\game\\weapon-mesh-hercules-20261110)
-> stdout + Build\\work\\rifle-grip-stock-20261110\\bodycheck_<family>[_nostock]_<bodies>.json (one body per process: skin_W
   holds one rig per process)
Inside test: nearest face sign (trimesh closest_point) for body points within 6 cm of a mesh vertex (the guns are <= 10.5 cm thick).
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripstock_common as G  # noqa: E402

G.MESHDIR['hercules'] = (os.path.join(G.WC.ROOT, 'Extracted', 'game', 'weapon-mesh-hercules-20261110'), None)
import json  # noqa: E402

TORSO = ('Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder')
UPPER = ('LeftArm', 'RightArm')


class Inside:
    def __init__(self, V, F):
        import trimesh
        self.m = trimesh.Trimesh(vertices=V, faces=F, process=False)
        self.n = self.m.face_normals
        self.vt = cKDTree(V)

    def depth(self, P):
        import trimesh
        out = np.zeros(len(P))
        if not len(P):
            return out
        cand = self.vt.query(P, distance_upper_bound=0.06)[0] < 0.06
        if not cand.any():
            return out
        Q = P[cand]
        c, d, tri = trimesh.proximity.closest_point(self.m, Q)
        s = np.einsum('ij,ij->i', Q - c, self.n[tri])
        o = np.where(s < -0.0005, d, 0.0)
        out[np.nonzero(cand)[0]] = o
        return out


def main():
    a = [x for x in sys.argv[1:]]
    nostock = '--nostock' in a
    every = 1
    if '--every' in a:
        i = a.index('--every')
        every = int(a[i + 1])
        del a[i:i + 2]
    fem, tagx = 0.025, ''
    if '--fem' in a:
        i = a.index('--fem')
        fem = float(a[i + 1])
        del a[i:i + 2]
    if '--tag' in a:
        i = a.index('--tag')
        tagx = '_' + a[i + 1]
        del a[i:i + 2]
    a = [x for x in a if x != '--nostock']
    sc = json.load(open(os.path.join(G.WORK, 'stock_calib.json')))
    fam = a[0]
    parts = a[1].split(',')
    bodies = (a[2] if len(a) > 2 else 'woman,man').split(',')
    G.MESHDIR[fam] = (G.MESHDIR[fam][0], tuple(p if p.endswith('.glb') else p + '.glb' for p in parts))
    V, F, files = G.weapon_mesh(fam)
    print('mesh', files, len(V), 'vertices', len(F), 'faces; extent cm', np.round((V.max(0) - V.min(0)) * 100, 1), flush=True)
    ins = Inside(V, F)
    lw = G.LocalWinding(V, F)
    contact = sc.get(a[0], {}).get('contact_y')
    rec = '%s_single_shot_ironsight.utpprecoil' % fam
    tag = fam + ('_nostock' if nostock else '') + tagx
    res = {}
    for body in bodies:
        P = G.player(body)
        ix = P.ix
        names = P.names
        for mode in ('before', 'after'):
            tl, shots, marks = G.timeline('film', rec, fem if body == 'woman' else 0.0)
            name = 'bc_%s_%s_%s' % (tag, mode, body)
            r = G.run(mode, name, tl, shots, body, G.PROFILE[fam], ('--nostock',) if nostock and mode == 'after' else ())
            W = r['W']
            B = G.skin(W, body, os.path.join(G.TMP, 'bc_%s_%s_%s_%g_skin' % (fam, mode, body, fem) if mode == 'before' else name + '_skin'))
            m = G.measure(W, None, body, fam)
            T = min(len(W), len(B))
            ra, rb, rc = W[:, ix['RightArm'], :3, 3], W[:, ix['RightForeArm'], :3, 3], W[:, ix['RightHand'], :3, 3]
            u, v = rb - ra, rc - rb
            relbow = np.degrees(np.arccos(np.clip(np.sum(u * v, 1) / (np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1)), -1, 1)))
            joints = [j for j in range(73) if names[j] not in ('Root', 'Trajectory', 'reference_joint', 'WeaponLeft', 'WeaponRight',
                                                               'LeftEye', 'RightEye')]
            jn = [names[j] for j in joints]
            reg = np.array([0 if n in TORSO else 1 if n == 'RightArm' else 2 if n == 'LeftArm' else 3 for n in jn])  # 3 = excluded
            M = W[:, ix['WeaponRight']]
            S3 = W[:, ix['Spine3']]
            lh = np.einsum('tji,tj->ti', S3[:, :3, :3], W[:, ix['LeftHand'], :3, 3] - S3[:, :3, 3])
            wo = np.einsum('tji,tj->ti', S3[:, :3, :3], M[:, :3, 3] - S3[:, :3, 3])
            gt = M[:, :3, 3] + np.einsum('tij,j->ti', M[:, :3, :3], np.array(G.game_grip(fam)['left_t']))
            gp = np.einsum('tji,tj->ti', S3[:, :3, :3], gt - S3[:, :3, 3])  # the weapon's game grip point (handguard)
            NR_ = 3
            n_in = np.full((T, NR_), np.nan)
            dep = np.full((T, NR_), np.nan)
            pts = {}  # (frame, region) -> inside points in the weapon frame (winding cross-check of the worst frame)
            pocket_gap = np.full(T, np.nan)
            if contact is not None:
                pk = np.array(sc['pocket_' + body]['local'])
                RA = W[:, ix['RightArm']]
                pocket = RA[:, :3, 3] + np.einsum('tij,j->ti', RA[:, :3, :3], pk)
                pocket_gap = (np.einsum('ti,ti->t', M[:, :3, 3] - pocket, M[:, :3, 1]) + contact)[:T] * 100
            lo, hi = V.min(0) - 0.01, V.max(0) + 0.01
            for f in range(0, T, every):
                _, own = cKDTree(W[f, joints, :3, 3]).query(B[f])
                rg = reg[own]
                Bw = (B[f] - M[f, :3, 3]) @ M[f, :3, :3]
                box = np.all((Bw > lo) & (Bw < hi), 1) & (rg < NR_)
                n_in[f], dep[f] = 0, 0.0
                if not box.any():
                    continue
                d = ins.depth(Bw[box])
                for k in range(NR_):
                    s = rg[box] == k
                    if s.any():
                        n_in[f, k] = int(np.sum(d[s] > 0))
                        dep[f, k] = float(d[s].max() * 100)
                        if n_in[f, k] > 0:
                            pts[(f, k)] = Bw[box][s][d[s] > 0]
            d = r['diag']
            cols = dict(grip=m['grip_err_cm'][:T], elbowL=m['elbow_flex'][:T], elbowR=relbow[:T],
                        shift=np.asarray(d['reachShift'], float)[:T] * 100, need=np.asarray(d['reachNeed'], float)[:T] * 100,
                        stockGap=np.asarray(d['stockGap'], float)[:T] * 100, stockShift=np.asarray(d['stockShift'], float)[:T] * 100,
                        supW=np.asarray(d['supportW'], float)[:T], pocketGap=pocket_gap, torso_in=n_in[:, 0], torso_cm=dep[:, 0],
                        rarm_in=n_in[:, 1], rarm_cm=dep[:, 1], larm_in=n_in[:, 2], larm_cm=dep[:, 2])
            lab = G.phase_of(marks, T)
            res['%s:%s' % (body, mode)] = dict(frames={k: [None if not np.isfinite(x) else round(float(x), 3) for x in v]
                                                       for k, v in cols.items()}, phase=lab, lh_s3=lh[:T].tolist(), w_s3=wo[:T].tolist(),
                                               g_s3=gp[:T].tolist())
            print('==', body, mode, T, 'frames', flush=True)
            for k, rn in enumerate(('torso', 'R upper arm', 'L upper arm')):
                if not np.any(np.nan_to_num(dep[:, k]) > 0):
                    continue
                f = int(np.nanargmax(dep[:, k]))
                wn = lw(pts[(f, k)])
                print('  worst %s frame %d (%s): %d inside by the face sign, %d by the winding number (> 0.5), depth %.2f cm' % (
                    rn, f, G.phase_of(marks, T)[f], len(wn), int(np.sum(wn > 0.5)), dep[f, k]), flush=True)
            for ph in dict.fromkeys(lab):
                s = np.array([x == ph for x in lab])
                print('  %-12s ' % ph + ' '.join('%s %s' % (k, G.summ(v[s])) for k, v in cols.items()), flush=True)
        A, Bf = res['%s:after' % body], res['%s:before' % body]
        # left hand against max(hand before, grip point before) = utpp_supportgrip_test (4) rule (a hand on the handguard moves with it)
        sa = np.linalg.norm(np.diff(np.array(A['lh_s3']), axis=0), axis=1) * 100
        ref = np.maximum(np.linalg.norm(np.diff(np.array(Bf['lh_s3']), axis=0), axis=1),
                         np.linalg.norm(np.diff(np.array(Bf['g_s3']), axis=0), axis=1)) * 100
        ex = sa - ref
        k = int(np.argmax(ex))
        print('  left hand step over max(hand, grip point) before: extra max %.2f cm at frame %d (%s)' % (ex[k], k + 1, A['phase'][k + 1]))
        res['%s:lh_vs_grip_step' % body] = dict(extra_max=float(ex[k]), frame=k + 1)
        for key in ('lh_s3', 'w_s3'):
            sa = np.linalg.norm(np.diff(np.array(A[key]), axis=0), axis=1) * 100
            sb = np.linalg.norm(np.diff(np.array(Bf[key]), axis=0), axis=1) * 100
            ex = sa - sb
            k = int(np.argmax(ex))
            print('  %s step per frame: after max %.2f cm, before max %.2f cm, extra max %.2f cm at frame %d (%s)' % (
                key, sa.max(), sb.max(), ex[k], k + 1, A['phase'][k + 1]), flush=True)
            res['%s:%s_step' % (body, key)] = dict(after_max=float(sa.max()), before_max=float(sb.max()), extra_max=float(ex[k]), frame=k + 1)
    G.WC.jdump(res, os.path.join(G.WORK, 'bodycheck_%s_%s.json' % (tag, '_'.join(bodies))))


if __name__ == '__main__':
    main()
