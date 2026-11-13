"""Traversal / jump gap clips: scene build, candidate selection and assembly (stages of trav_run.sh).

python -I trav_pipe.py scene <recipe>      -> <WORK>/scn/<recipe>/<variant>.{dump.npz,json,soma.npz}, <WORK>/scn/<recipe>/jobs.json
python -I trav_pipe.py assemble <recipe>   -> pin + score every (variant, seed) per gap, best per variant, ranking table,
                                              <WORK>/asm/<recipe>/<variant>.{soma.npz,world.npz,meta.json}
Recipes are in trav_recipes.py (RECIPES). MotionBricks itself runs in segment_generator.py (uv env); Blender steps in trav_run.sh.
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trav_lib as T  # noqa: E402
from trav_recipes import RECIPES  # noqa: E402

SEEDS = [-1, 10, 11, 12, 13, 14]
JOINTS = ['foot_l', 'foot_r', 'ball_l', 'ball_r', 'hand_l', 'hand_r', 'head', 'calf_l', 'calf_r', 'thigh_l', 'thigh_r',
          'spine_03', 'lowerarm_l', 'lowerarm_r']


def rel_pose(D, f0, f1):
    """Joint positions relative to the pelvis in the heading frame, frames f0..f1 -> [n, J, 3] (cm)."""
    out = []
    for f in range(f0, f1 + 1):
        h = D.heading(f)
        Rz = np.array([[h[1], -h[0], 0], [h[0], h[1], 0], [0, 0, 1.0]])  # rotates the heading (-Y up to sign) onto a fixed axis
        pel = D.W[f, D.pel, :3, 3]
        out.append(np.array([Rz @ (D.W[f, D.idx[j], :3, 3] - pel) for j in JOINTS]))
    return np.array(out)


def pose_dist(DA, fa, DB, fb):
    """Distance of the 4-frame windows ending at fa (A) and starting at fb (B): pose + relative velocity (cm)."""
    pa, pb = rel_pose(DA, fa - 3, fa), rel_pose(DB, fb, fb + 3)
    d_pose = np.linalg.norm(pa[-1] - pb[0], axis=-1).mean()
    d_vel = np.linalg.norm((pa[-1] - pa[0]) - (pb[-1] - pb[0]), axis=-1).mean()
    dz = abs(DA.pel_pos(fa)[2] - DB.pel_pos(fb)[2])
    return d_pose + 0.5 * d_vel + 0.5 * dz


def speed(D, f0, f1):
    p = D.W[f0:f1 + 1, D.pel, :2, 3]
    return np.linalg.norm(np.diff(p, axis=0), axis=1).mean() * 30.0 / 100.0 if len(p) > 1 else 0.0


def stage_scene(rname, args):
    R = RECIPES[rname]
    out = os.path.join(T.WORK, 'scn', rname)
    os.makedirs(out, exist_ok=True)
    jobs = []
    for var in R['variants'](T, pose_dist, speed):
        sc = T.build_scene(var['items'])
        base = os.path.join(out, var['id'])
        T.save_scene(sc, base + '.dump.npz')
        T.to_soma(base + '.dump.npz', base + '.soma.npz')
        if 'soma_edit' in var:  # recipe edit of the bridged scene (e.g. hurdle apex window) before MotionBricks sees it
            s = dict(np.load(base + '.soma.npz'))
            var['soma_edit'](s, sc, T)
            np.savez(base + '.soma.npz', **s)
        sm = np.load(base + '.soma.npz')
        pos = sm['pos'] if 'pos' in sm.files else None
        if pos is not None:  # soma -> source pack linear maps (ankle height, travel axis) for clearance checks
            sn = [str(x) for x in sm['names']]
            Wm = sc['W']
            ix = {n: i for i, n in enumerate(sc['names'])}
            za = np.polyfit(pos[:, sn.index('LeftFoot'), 1], Wm[:, ix['foot_l'], 2, 3], 1)
            ya = np.polyfit(pos[:, sn.index('Hips'), 2], Wm[:, ix['pelvis'], 1, 3], 1)
            var.setdefault('meta', {})['fit'] = dict(za=list(za), ya=list(ya))
        json.dump(dict(var.get('meta', {}), id=var['id'], spans=sc['spans'], gaps=sc['gaps']), open(base + '.json', 'w'), indent=1, default=float)
        jobs.append(dict(soma=base + '.soma.npz', out=os.path.join(out, var['id']), gaps=[dict(c=g['c'], N=g['N'], targets=4) for g in sc['gaps']],
                         seeds=SEEDS, jitter_cm=2.0))
        print('SCENE %s %s frames %d gaps %s' % (rname, var['id'], len(sc['W']), [(g['c'], g['N']) for g in sc['gaps']]))
    json.dump(jobs, open(os.path.join(out, 'jobs.json'), 'w'), indent=1)


def planted_slide(P, sk, ground, thr=0.03, min_run=4):
    """Per planted run (lowest foot joint within thr of its floor level for >= min_run frames with the same pivot joint):
    mean horizontal speed in cm/frame. Swing frames that merely skim the floor are not planted runs."""
    v = []
    for side in ('Left', 'Right'):
        a, t = sk.idx[side + 'Foot'], sk.idx[side + 'ToeBase']
        ha, ht = P[:, a, 1] - ground[a], P[:, t, 1] - ground[t]
        low = np.minimum(ha, ht) < thr
        f = 0
        while f < len(P):
            if not low[f]:
                f += 1
                continue
            e = f
            while e + 1 < len(P) and low[e + 1]:
                e += 1
            if e - f + 1 >= min_run:
                piv = np.where(ha[f:e + 1] <= ht[f:e + 1], a, t)
                d = [np.linalg.norm((P[k, piv[k - f]] - P[k - 1, piv[k - f]])[[0, 2]]) * 100 for k in range(f + 1, e + 1) if piv[k - f] == piv[k - 1 - f]]
                if d:
                    v.append(float(np.mean(d)))
            f = e + 1
    return v


def accel_j(P, js):
    return np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=-1)[:, js].max(1) * 900  # frame f+1


def stage_assemble(rname, args):
    sk = T.Skel(T.TP)
    R = RECIPES[rname]
    scn = os.path.join(T.WORK, 'scn', rname)
    out = os.path.join(T.WORK, 'asm', rname)
    os.makedirs(out, exist_ok=True)
    from motion_metrics import J30
    j30 = [sk.idx[n] for n in J30]
    table = []
    for vf in sorted(f for f in os.listdir(scn) if f.endswith('.soma.npz')):
        vid = vf[:-9]
        meta = json.load(open(os.path.join(scn, vid + '.json')))
        scene = np.load(os.path.join(scn, vf))
        G, hips = scene['G'].astype(np.float64).copy(), scene['hips'].astype(np.float64).copy()
        Ps = T.fk_all(sk, G, hips)
        ground = Ps[:, :, 1].min(0)
        Pc = Ps.copy()
        chosen, cand_rows = [], []
        for gi, gp in enumerate(meta['gaps']):
            c, g = gp['c'], gp['g']
            best = None
            ref_acc = accel_j(Ps, j30)
            keep = np.ones(len(ref_acc), bool)
            for q in meta['gaps']:  # frames around every gap (placeholders / splice joins) are not the clips' own motion
                keep[max(0, q['c'] - 3):q['g'] + 2] = False
            outside = ref_acc[keep]
            ref99 = np.percentile(outside, 99) if len(outside) else 100.0
            for seed in SEEDS:
                gen = os.path.join(scn, '%s.g%d.s%d.gen.npz' % (vid, gi, seed))
                if not os.path.isfile(gen):
                    continue
                pin = gen[:-8] + '.pin.npz'
                T.pin_span(gen, os.path.join(scn, vf), pin, labels=True)
                s = np.load(pin)
                Pm = Ps.copy()
                Pm[c:g + 1] = T.fk_all(sk, s['G'], s['hips'])
                acc = accel_j(Pm, j30)
                bnd = max(acc[f - 1] for f in list(range(c - 1, c + 2)) + list(range(g - 1, g + 2)) if 1 <= f < len(Pm) - 1)
                sl = planted_slide(Pm[c:g + 1], sk, ground)
                hy = Pm[c:g + 1, sk.idx['Hips'], 1]
                hipv = np.abs(hy[2:] - 2 * hy[1:-1] + hy[:-2]).max() * 900 if len(hy) > 2 else 0
                pen = max(0.0, -(Pm[c:g + 1][:, [sk.idx[n] for n in ('LeftToeBase', 'RightToeBase', 'LeftFoot', 'RightFoot')], 1] - ground[[sk.idx[n] for n in ('LeftToeBase', 'RightToeBase', 'LeftFoot', 'RightFoot')]]).min()) * 100
                hd = Pm[:, sk.idx['Head']] - Pm[:, sk.idx['Hips']]
                lean = np.degrees(np.arccos(np.clip(hd[:, 1] / np.linalg.norm(hd, axis=1), -1, 1)))
                src = np.ones(len(lean), bool)
                src[c + 1:g] = False
                lean_gap, lean_src = float(lean[~src].max()) if (~src).any() else 0.0, float(lean[src].max())
                extra = R.get('extra_score', lambda *a: (0.0, {}))(T, sk, Pm, c, g, meta, ground)
                score = (max(sl) / 2.0 if sl else 0.0) + 0.6 * max(0.0, bnd / max(ref99, 1) - 1.0) + 0.02 * hipv + 0.5 * pen + 0.1 * max(0.0, lean_gap - (lean_src + 15.0)) + extra[0]
                row = dict(var=vid, gap=gi, seed=seed, score=round(score, 3), slide_max=round(max(sl), 2) if sl else 0.0,
                           slide_mean=round(float(np.mean(sl)), 2) if sl else 0.0, bnd=round(bnd, 1), ref99=round(ref99, 1), hipv=round(hipv, 1),
                           pen_cm=round(pen, 2), lean_gap=round(lean_gap, 1), lean_src=round(lean_src, 1), **extra[1])
                cand_rows.append(row)
                if best is None or score < best[0]:
                    best = (score, seed, s, row)
            if best is None:
                print('NO CANDIDATE', vid, gi)
                continue
            s = best[2]
            Pc[gp['c']:gp['g'] + 1] = T.fk_all(sk, s['G'], s['hips'])
            G[gp['c']:gp['g'] + 1], hips[gp['c']:gp['g'] + 1] = s['G'], s['hips']
            chosen.append(best[3])
        np.savez(os.path.join(out, vid + '.soma.npz'), G=G, hips=hips, names=scene['names'], k=scene['k'], fps=30)
        tot = sum(r['score'] for r in chosen)
        table.append((tot, vid, chosen))
        print('ASM %s total %.2f | %s' % (vid, tot, ' | '.join('g%d s%d sc%.2f slide %.1f/%.1f bnd %.0f/%.0f' % (r['gap'], r['seed'], r['score'], r['slide_max'], r['slide_mean'], r['bnd'], r['ref99']) for r in chosen)))
        json.dump(dict(meta, chosen=chosen, candidates=cand_rows), open(os.path.join(out, vid + '.sel.json'), 'w'), indent=1, default=float)
    table.sort(key=lambda r: r[0])
    json.dump([dict(var=v, total=t, chosen=c) for t, v, c in table], open(os.path.join(out, 'ranking.json'), 'w'), indent=1, default=float)


def stage_world(rname, args):
    """Selected variants (ids on the command line, default: best total) -> source pack world npz + FBX-ready dump (trim t0..t1 from the recipe)."""
    R = RECIPES[rname]
    out = os.path.join(T.WORK, 'asm', rname)
    ids = args or [json.load(open(os.path.join(out, 'ranking.json')))[0]['var']]
    for vid in ids:
        sel = json.load(open(os.path.join(out, vid + '.sel.json')))
        s = np.load(os.path.join(out, vid + '.soma.npz'))
        t0, t1 = R['trim'](sel)
        br, d = T.load_bridge(T.TP, os.path.join(T.WORK, 'scn', rname, vid + '.dump.npz'))
        frames = list(range(t0, t1 + 1))
        W = br.inv(s['G'][t0:t1 + 1], s['hips'][t0:t1 + 1], d['W'], frames, leg_ik=True)
        ri = [str(x) for x in d['names']].index('attach')
        W[:, ri, 2, 3] = 0.0 if 'root_z' not in R else R['root_z'](sel, t0, len(W))  # root z = ground height under the character
        np.savez(os.path.join(out, vid + '.world.npz'), W=W, names=d['names'], parents=d['parents'], fps=30)
        print('WORLD', vid, 'frames', len(W), 't0', t0, 't1', t1)


if __name__ == '__main__':
    {'scene': stage_scene, 'assemble': stage_assemble, 'world': stage_world}[sys.argv[1]](sys.argv[2], sys.argv[3:])
