"""Arm/hand-to-torso penetration estimate (female baseline vs male) for the Nue reload sets. LBS math preview only.
For subsampled arm-mesh vertices (a0_*), the nearest torso (t0_*, chest/shoulder region) vertex is found and the arm vertex is
counted inside when it lies behind that vertex's skinned normal.  Reports worst depth per clip/body and the frame.
python -I check_nue_reload_man_penetration.py --female F.glb --male M.glb --report R.json [--step 3]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import render_nue_reload_man as R  # noqa: E402
import skin_cache as SC  # noqa: E402
import skel as S  # noqa: E402
import glbanim as G  # noqa: E402

CLIPS = ['utpp_nue_reload_aim', 'utpp_nue_reload_ready', 'utpp_nue_reload_2h_aim', 'utpp_nue_reload_2h_ready',
         'utpp_nue_empty_reload_aim', 'utpp_nue_empty_reload_ready', 'utpp_nue_empty_reload_2h_aim', 'utpp_nue_empty_reload_2h_ready']


def vnormals(v, f):
    n = np.zeros_like(v)
    tri = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], tri)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--female', required=True)
    ap.add_argument('--male', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--step', type=int, default=3)
    a = ap.parse_args()
    meshes = {'woman': SC.load_meshes('woman'), 'man': SC.load_meshes('man')}
    rep = {}
    for clip in CLIPS:
        rep[clip] = {}
        for body, path in (('woman', a.female), ('man', a.male)):
            sk = S.Skeleton.from_glb(path)
            ci = sk.clip(clip)
            ch = G.channels(sk.gj, sk.binb, ci)
            t = ch['LeftHand']['rotation'][0]
            sel = np.arange(0, len(t), a.step)
            lt, lr = sk.sample_local(ci, t[sel])
            ri = sk.index['Root']
            lt[:, ri, 0] = 0
            lt[:, ri, 2] = 0
            Wt, Wr = sk.world(lt, S.qcontinuous(lr))
            parts = meshes[body]
            torso = [p for p in parts if p['name'].startswith('t0_')]
            arms = [p for p in parts if p['name'].startswith('a0_')]
            tf = torso[0]['faces']
            worst = dict(depth=0.0, frame=-1, part='')
            for k in range(len(sel)):
                tv = R.skin(torso, sk, Wt, Wr, k)[0][1]
                tn = vnormals(tv, tf)
                if ((tv - tv.mean(0)) * tn).sum(1).mean() < 0:
                    tn = -tn  # winding convention: make normals point outward
                up = tv[:, 1] > 0.9  # glTF Y-up here (skin() output is model space)
                tvu, tnu = tv[up][::2], tn[up][::2]
                for name, av in R.skin(arms, sk, Wt, Wr, k):
                    av = av[::5]
                    d2 = ((av[:, None, :] - tvu[None, :, :]) ** 2).sum(-1)
                    j = d2.argmin(1)
                    dist = np.sqrt(d2[np.arange(len(av)), j])
                    signed = ((av - tvu[j]) * tnu[j]).sum(1)
                    near = dist < 0.06
                    if near.any():
                        depth = float(max(0.0, -signed[near].min()))
                        if depth > worst['depth']:
                            worst = dict(depth=depth, frame=int(sel[k]), part=name)
            rep[clip][body] = worst
        w, m = rep[clip]['woman'], rep[clip]['man']
        print(clip, 'F %.4f m (f%d %s) | M %.4f m (f%d %s)' % (w['depth'], w['frame'], w['part'].split(':')[0], m['depth'], m['frame'], m['part'].split(':')[0]), flush=True)
    Path(a.report).write_text(json.dumps(rep, indent=2) + '\n')


if __name__ == '__main__':
    main()
