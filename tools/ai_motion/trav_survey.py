"""Survey of source pack dumps (mannequin_dump.py npz): frames, pelvis / feet heights, travel, speed. Used to pick the seed clips.

python -I trav_survey.py <dump dir> [name filter substring ...]
"""
import glob
import os
import sys

import numpy as np


def info(path):
    d = np.load(path)
    names = [str(x) for x in d['names']]
    W = d['W']
    pel = names.index('pelvis')
    P = W[:, pel, :3, 3]
    fl, fr = names.index('foot_l'), names.index('foot_r')
    zl, zr = W[:, fl, 2, 3], W[:, fr, 2, 3]
    ll, lr = names.index('hand_l'), names.index('hand_r')
    zh = np.maximum(W[:, ll, 2, 3], W[:, lr, 2, 3])
    sp = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1) * 30 / 100
    return dict(n=len(W), pz=(P[:, 2].min(), P[:, 2].max()), travel=np.linalg.norm(P[-1, :2] - P[0, :2]) / 100,
                z0=P[0, 2], z1=P[-1, 2], fmax=max(zl.max(), zr.max()), handmax=zh.max(),
                v0=sp[:5].mean() if len(sp) else 0, v1=sp[-5:].mean() if len(sp) else 0)


if __name__ == '__main__':
    flt = sys.argv[2:]
    for p in sorted(glob.glob(os.path.join(sys.argv[1], '*.dump.npz'))):
        nm = os.path.basename(p)[:-9]
        if flt and not any(f in nm for f in flt):
            continue
        i = info(p)
        print('%-52s n=%3d pelZ %5.1f..%5.1f (%5.1f -> %5.1f) travel %5.2f m  footmax %5.1f handmax %6.1f  v %4.2f -> %4.2f m/s'
              % (nm[10:], i['n'], i['pz'][0], i['pz'][1], i['z0'], i['z1'], i['travel'], i['fmax'], i['handmax'], i['v0'], i['v1']))
