"""Self-consistency check of the stage 7 fixtures (read only): FK of the stored locals reproduces the stored hand positions, and every
guarded joint of the 'clamp' pose decomposes inside its limits.

  python -I check_fixtures.py woman|man [apply]      -> fixtures\\<rig>[\\apply]
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

rig = sys.argv[1]
D = os.path.join(SC.WORK, 'fixtures', rig, *sys.argv[2:3])
import guard_limits as GL  # noqa: E402

lim = json.load(open(os.path.join(D, 'index.json'), encoding='utf-8'))['limits']
worst_fk, worst_lim, n = 0.0, -1e9, 0
for p in sorted(glob.glob(os.path.join(D, '[0-9]*.json'))):
    fx = json.load(open(p, encoding='utf-8'))
    names = [j['name'] for j in fx['joints']]
    par = [j['parent'] for j in fx['joints']]
    restq = np.array([j['rest_q'] for j in fx['joints']])
    for fr in fx['frames']:
        for kind in ('vanilla', 'clamp', 'guard'):
            t, Rm = np.array(fr[kind]['t']), R.from_quat(np.array(fr[kind]['q'])).as_matrix()
            Wr, Wp = np.zeros((len(names), 3, 3)), np.zeros((len(names), 3))
            for j, pj in enumerate(par):
                if pj < 0:
                    Wr[j], Wp[j] = Rm[j], t[j]
                else:
                    Wr[j], Wp[j] = Wr[pj] @ Rm[j], Wp[pj] + Wr[pj] @ t[j]
            if kind in ('vanilla', 'guard'):
                for S in ('Left', 'Right'):
                    ref = np.array(fr['hands_cm'][S][kind])
                    worst_fk = max(worst_fk, float(np.linalg.norm(Wp[names.index(S + 'Hand')] * 100 - ref)))
            if kind == 'clamp':
                for jn, L in lim.items():
                    j = names.index(jn)
                    tw, sy, sz, _ = GL.decompose((R.from_quat(restq[j]).as_matrix().T @ Rm[j])[None])
                    for v, k in ((tw[0], 'twist'), (sy[0], 'swing_y'), (sz[0], 'swing_z')):
                        worst_lim = max(worst_lim, L[k][0] - v, v - L[k][1])
        n += 1
print('%s: %d frames, FK hand mismatch max %.4f cm, clamp pose max distance beyond limits %.3f deg (< 0 = inside)' % (D, n, worst_fk, worst_lim))
