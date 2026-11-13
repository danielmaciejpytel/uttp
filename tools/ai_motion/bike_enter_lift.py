"""Motorcycle get-on clips: a foot that moves along the ground (glide) becomes a short step. GENTLE correction, nothing else changes: the horizontal ankle path, the hips, the spine and
the arms stay; a glide episode of a foot in ground contact (bike_steps_gen.slide_list rule: consecutive contact frames, pivot speed > 1 cm / frame) gets a vertical arc
h * 4 m (1 - m), ONLY while the other foot is planted or V sits on the saddle (supported(): otherwise the episode is skipped and reported, a lifted support foot would make V hop) (m = progress over the episode, one frame of margin on both sides, so lift-off and landing stay on the ground), h = clamp(0.35 * travel, 3, 7 cm); two-bone leg IK
(exit_steps_gen.ik_to: the knee keeps its bending plane, the foot its world orientation) follows. The result goes through exit_steps.splice (build_exit_duck.py --identity --steps: the
locals of Hips / spine / legs of the frames c..g, the limb IK helpers re-anchored on the new feet, Root / events / tracks untouched, every frame outside c..g bit-identical).

  python -I bike_enter_lift.py <builder enter glb> <clip> <out glb> [--report out.json] [--min-travel-cm 3] [--h-frac 0.35] [--h-min 3] [--h-max 7] [--step-max-deg 12]
Prints LIFT lines; exit code 0 also when nothing needs a lift (the output is then the unchanged builder clip written through the same writer: use --copy-if-none to copy the input instead).
Opt-in step of build_bike.ps1 (-EnterLift). No game, no install.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, HERE)
import game_skeleton_bridge as CB  # noqa: E402
import exit_steps_gen as ESG  # noqa: E402
import bike_steps_gen as BSG  # noqa: E402

TP = os.path.join(HERE, 'data', 'body_tpose_generator.npz')
BUILD_DUCK = os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry', 'build_exit_duck.py')


def episodes(W, names, side):
    """glide episodes of one foot: list of (first frame, last frame, travel cm); gaps of one frame are bridged"""
    ja, jt = names.index(side + 'Foot'), names.index(side + 'ToeBase')
    za, zt = W[:, ja, 2, 3], W[:, jt, 2, 3]
    ha, ht = za - za.min(), zt - zt.min()
    piv = np.where(ha <= ht, ja, jt)
    hp = np.minimum(ha, ht)
    P = np.array([W[f, piv[f], :3, 3] for f in range(len(W))])
    sp = np.zeros(len(W))
    for f in range(1, len(W)):
        if hp[f] < 0.03 and hp[f - 1] < 0.03 and piv[f] == piv[f - 1] and abs(P[f, 2] - P[f - 1, 2]) < 0.005:
            sp[f] = np.linalg.norm((P[f] - P[f - 1])[:2]) * 100
    fr = [f for f in range(1, len(W)) if sp[f] > 1.0]
    eps = []
    for f in fr:
        if eps and f - eps[-1][1] <= 2:
            eps[-1][1] = f
        else:
            eps.append([f, f])
    return [(a, b, float(sp[a:b + 1].sum())) for a, b in eps]


def supported(W, names, side, f0, f1, hips_last_z):
    """frames t in f0..f1 where V is supported while `side` is lifted: the OTHER foot is planted (horizontal ankle speed < 1 cm / frame and vertical < 0.7 cm / frame to both neighbours,
    ground or peg) or V sits on the saddle (hips within 3 cm of the last frame's hips height). Returns the number of unsupported frames and their list."""
    other = 'Right' if side == 'Left' else 'Left'
    ja = names.index(other + 'Foot')
    hz = names.index('Hips')
    bad = []
    for t in range(f0, f1 + 1):
        ok = abs(W[t, hz, 2, 3] - hips_last_z) < 0.03
        if not ok:
            planted = True
            for u in (t - 1, t):
                if 0 <= u < len(W) - 1:
                    d = W[u + 1, ja, :3, 3] - W[u, ja, :3, 3]
                    planted &= bool(np.linalg.norm(d[:2]) * 100 < 1.0 and abs(d[2]) * 100 < 0.7)
            ok = planted
        if not ok:
            bad.append(t)
    return len(bad), bad


def leg_step(W, par, names, f0, f1):
    """largest local rotation step (deg / frame) of the leg bones over f0..f1"""
    out = 0.0
    for sd in ESG.SIDES:
        for b in ('UpLeg', 'Leg', 'Foot', 'ToeBase'):
            j = names.index(sd + b)
            Lr = np.einsum('tji,tjk->tik', W[f0:f1 + 1, par[j], :3, :3], W[f0:f1 + 1, j, :3, :3])
            rel = np.einsum('tij,tkj->tik', Lr[1:], Lr[:-1])
            c = (np.trace(rel, axis1=1, axis2=2) - 1) / 2
            out = max(out, float(np.degrees(np.arccos(np.clip(c, -1, 1))).max()))
    return out


def main(a):
    glb, clip, out = a[:3]
    opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
    min_cm, hfrac, hmin, hmax = opt('--min-travel-cm', 3.0), opt('--h-frac', 0.35), opt('--h-min', 3.0), opt('--h-max', 7.0)
    step_max = opt('--step-max-deg', 12.0)
    W, info = CB.world_from_glb(glb, clip)
    names, par, ref = CB.load_rig()
    rest = CB.rest_world(par, ref)
    tmp = tempfile.mkdtemp(prefix='bike_lift_')
    scene = os.path.join(tmp, 'scene.npz')
    CB.save_dump(scene, W, names, par, rest, src=os.path.basename(glb))
    br, d = CB.load_cp_bridge(TP, scene)
    W2 = d['W'].copy()
    rep = dict(glb=glb, clip=clip, episodes=[])
    last = len(W2) - 1
    for side in ESG.SIDES:
        fo = br.uidx[br.legs[side][2]]
        for (e0, e1, trav) in episodes(W, names, side):
            if trav < min_cm:
                continue
            h = float(np.clip(hfrac * trav, hmin, hmax)) / 100.0
            best = None
            hips_last = float(W2[last, names.index('Hips'), 2, 3])
            thr = max(step_max, leg_step(W2, par, names, max(e0 - 7, 0), min(e1 + 7, last)) + 0.5)   # never below the clip's own steps there
            for pad in range(1, 7):     # the smallest margin whose leg rotation steps stay <= step_max (check_bike_entry (d) limit 15 deg / frame)
                s0, s1 = max(e0 - pad, 0), min(e1 + pad, last)
                Wt = W2.copy()
                for t in range(s0 + 1, s1):
                    m = (t - s0) / float(s1 - s0)
                    goal = Wt[t, fo, :3, 3].copy()
                    goal[2] += h * 4.0 * m * (1.0 - m)
                    ESG.ik_to(br, Wt[t], side, goal)
                st = leg_step(Wt, par, names, max(s0 - 1, 0), min(s1 + 1, last))
                if best is None or st < best[0]:
                    best = (st, pad, Wt)
                if st <= thr:
                    break
            st, pad, Wb = best
            s0, s1 = max(e0 - pad, 0), min(e1 + pad, last)
            nbad, bad = supported(W2, names, side, s0 + 1, s1 - 1, hips_last)
            if nbad:   # the other foot is moving and V is not on the saddle: this foot carries the weight, lifting it would make V hop -> left unchanged (reported)
                rep['episodes_skipped'] = rep.get('episodes_skipped', []) + [dict(side=side, glide_frames=[int(e0), int(e1)], travel_cm=round(trav, 1), unsupported_frames=bad)]
                print('LIFT %s %s frames %d..%d travel %.1f cm SKIPPED: %d unsupported frames %s' % (os.path.basename(glb), side, e0, e1, trav, nbad, bad))
                continue
            W2 = Wb
            rep['episodes'].append(dict(side=side, unsupported_frames=0, frames=[int(s0), int(s1)], glide_frames=[int(e0), int(e1)], travel_cm=round(trav, 1), lift_cm=round(h * 100, 1), pad=pad, leg_step_deg=round(st, 1)))
            print('LIFT %s %s frames %d..%d travel %.1f cm lift %.1f cm pad %d leg step %.1f deg' % (os.path.basename(glb), side, e0, e1, trav, h * 100, pad, st))
    if not rep['episodes']:
        shutil.copyfile(glb, out)
        print('LIFT %s none' % os.path.basename(glb))
        json.dump(rep, open(a[a.index('--report') + 1], 'w'), indent=1) if '--report' in a else None
        return 0
    eps_all = [e['frames'] for e in rep['episodes']]
    c = max(min(e[0] for e in eps_all) - 5, 0)
    g = min(max(e[1] for e in eps_all) + 5, last)
    steps = os.path.join(tmp, 'lift.steps.npz')
    np.savez(steps, W=W2[c:g + 1], c=c, g=g, names=np.array(names), parents=par, src='bike_enter_lift', leg_ik=True)
    r = subprocess.run([sys.executable, '-I', BUILD_DUCK, '--glb', glb, '--clip', clip, '--name', clip, '--out', out, '--identity', '--steps', steps,
                        '--report', out + '.steps-report.json'], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout[-400:], r.stderr[-600:])
        return 1
    rep['c'], rep['g'] = c, g
    if '--report' in a:
        json.dump(rep, open(a[a.index('--report') + 1], 'w'), indent=1)
    shutil.rmtree(tmp, ignore_errors=True)
    W3, _ = BSG.load('%s:%s' % (out, clip))
    for s_, nm in (('before', W), ('after', W3)):
        g_ = BSG.glide(nm, names, None, BSG.floors_of(W, names))   # the before clip's ground for both
        print('LIFT %s glide total %.1f cm (L %.1f R %.1f)' % (s_, g_['glide_total_cm'], g_['Left']['glide_cm'], g_['Right']['glide_cm']))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
