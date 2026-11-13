"""Joint tracks for fists_guard_figures.py (2026-11-13). Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.

  python -I fists_guard_extract.py ship --glb <utpp_pwa_fists.anims.glb> --out <hands_ship_pwa.npz>
      Every clip of an exported fists set (WolvenKit export of the .anims, e.g. tools\\anim\\vehicle-entry\\export_glb.ps1 on the file
      unbundled from the shipped utpp_melee.archive): the 11 joints below in model space, cm, game axes (x right, y forward, z up).
  python -I fists_guard_extract.py mm --rig woman|man --out <mm_hands_woman.npz>
      The same joints of the locomotion source clips (Animations\\retarget\\out_sourcepack_set3_<rig>: M_Neutral_Stand_Idle_Loop, Walk_Loop_F,
      Run_Loop_F, Sprint_Loop_F) on the V rig, in-place frame (hips at the capsule axis).
npz keys: names, clip__<name> -> [T, 11, 3].  Needs the Source checkout layout (tools\\melee\\base_body.py resolves the project root).
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = ['LeftHand', 'RightHand', 'LeftForeArm', 'RightForeArm', 'Hips', 'Spine3', 'Head', 'LeftFoot', 'RightFoot', 'LeftArm', 'RightArm']


def ship(a):
    sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
    sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'melee-export'))
    import melee_export_lib as L
    import skel as S
    import glbanim as G
    sk = S.Skeleton.from_glb(a.glb)
    idx = [sk.index[n] for n in JOINTS]
    res = {}
    for name, ai in G.clips(sk.gj).items():
        ch = G.channels(sk.gj, sk.binb, ai)
        tmax = max(float(t[-1]) for b, d in ch.items() if b != 'Root' for (t, v, it) in d.values())
        nfr = int(round(tmax * 30.0)) + 1
        lt, lr, _, _ = L.sample_clip(sk, name, nfr, tmax)
        Wt, _ = sk.world(lt, lr)
        P = Wt[:, idx, :]                                   # glTF axes (y up)
        res[name] = (np.stack([P[..., 0], -P[..., 2], P[..., 1]], axis=-1) * 100.0).astype(np.float32)
    np.savez_compressed(a.out, names=np.array(JOINTS), **{'clip__' + k: v for k, v in res.items()})
    print('clips', len(res), '->', a.out)


def mm(a):
    os.environ['UTPP_RIG'] = a.rig
    sys.path.insert(0, HERE)
    sys.argv = [sys.argv[0]]
    import base_body as BB
    names, par, ref, rest = BB.rig()
    nm = [str(n) for n in names]
    ix = [nm.index(w) for w in JOINTS]
    res = {}
    for c in ('M_Neutral_Stand_Idle_Loop', 'M_Neutral_Walk_Loop_F', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F'):
        Wi = BB.inplace(BB.sourcepack_W(c), names)
        res[c] = (Wi[:, ix][:, :, :3, 3] * 100.0).astype(np.float32)   # NOTE: index the joint axis first (mixed fancy indexing reorders axes)
    np.savez_compressed(a.out, names=np.array(JOINTS), **{'clip__' + k: v for k, v in res.items()})
    print('clips', len(res), '->', a.out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('ship')
    s.add_argument('--glb', required=True)
    s.add_argument('--out', required=True)
    m = sub.add_parser('mm')
    m.add_argument('--rig', required=True, choices=['woman', 'man'])
    m.add_argument('--out', required=True)
    args = ap.parse_args()
    {'ship': ship, 'mm': mm}[args.cmd](args)
