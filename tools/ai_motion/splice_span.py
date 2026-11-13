"""Splice a generated span (soma npz with c, g) into a full bridged clip; optionally undo the Kimodo canonical frame.

python splice_span.py <full_soma.npz> <span.npz|-> <out.npz> [--canonical-from C]
--canonical-from C: the span is in constraint_builder.py canonical space built at clip frame C (Hips XZ at the origin,
heading +Z); its c/g are relative to C-3, so the span replaces clip frames C .. C+(g-c).
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


def main(a):
    f = np.load(a[0])
    G, h = f['G'].copy(), f['hips'].copy()
    if a[1] != '-':
        s = np.load(a[1])
        sG, sh = s['G'].astype(np.float64), s['hips'].astype(np.float64)
        c, g = int(s['c']), int(s['g'])
        if '--canonical-from' in a:
            C = int(a[a.index('--canonical-from') + 1])
            names = [str(x) for x in f['names']]
            fwd = G[C, names.index('Hips')] @ np.array([0, 0, 1.0])
            Ry = R.from_euler('y', -np.arctan2(fwd[0], fwd[2])).as_matrix()
            o = h[C].copy(); o[1] = 0
            sG = Ry.T[None, None] @ sG
            sh = (Ry.T @ sh.T).T + o
            c, g = C, C + (g - c)
        G[c:g + 1], h[c:g + 1] = sG, sh
    np.savez(a[2], G=G, hips=h, names=f['names'], k=f['k'], fps=30)
    print('SPLICE', a[2])


if __name__ == '__main__':
    main(sys.argv[1:])
