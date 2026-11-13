"""python -I fs_knee_check.py <woman|man> <src dir> <fixed dir> : knee hinge purity and ankle angle change of written clips.
off-hinge = angle of the local Leg rotation vector orthogonal to the clip's dominant hinge axis (SVD of the source rotation vectors);
foot = max angle between source and fixed local Foot quaternion over frames (ankle flex change)."""
import os, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L


def rotvec(q):
    w = np.clip(q[..., 3], -1, 1)
    s = np.where(w < 0, -1.0, 1.0)
    q = q * s[..., None]
    ang = 2 * np.arccos(np.clip(q[..., 3], -1, 1))
    n = np.linalg.norm(q[..., :3], axis=-1, keepdims=True)
    ax = np.where(n > 1e-9, q[..., :3] / np.maximum(n, 1e-9), 0.0)
    return ax * ang[..., None]


def off_hinge(rv, a):
    p = rv - (rv @ a)[:, None] * a[None, :]
    return np.degrees(np.linalg.norm(p, axis=1))


def analyse(src, fx, g):
    out = []
    for f in sorted(os.listdir(fx)):
        if not f.endswith('.utppclip.json'):
            continue
        a, b = L.Clip(os.path.join(src, f), g), L.Clip(os.path.join(fx, f), g)
        r = dict(name=f[:-14], off0=0.0, off1=0.0, foot=0.0, thigh=0.0)
        for side in ('Left', 'Right'):
            jl, jf, ju = a.bi[side + 'Leg'], a.bi[side + 'Foot'], a.bi[side + 'UpLeg']
            rv0 = rotvec(a.Q[:, jl])
            u, s, vt = np.linalg.svd(rv0, full_matrices=False)
            ax = vt[0]
            r['off0'] = max(r['off0'], float(off_hinge(rv0, ax).max()))
            r['off1'] = max(r['off1'], float(off_hinge(rotvec(b.Q[:, jl]), ax).max()))
            for key, j in (('foot', jf), ('thigh', ju)):
                d = np.abs(np.sum(a.Q[:, j] * b.Q[:, j], -1))
                r[key] = max(r[key], float(np.degrees(2 * np.arccos(np.clip(d, -1, 1))).max()))
        out.append(r)
    return out


if __name__ == '__main__':
    g, src, fx = sys.argv[1:4]
    rows = analyse(src, fx, g)
    print('clips', len(rows))
    print('knee off-hinge max deg: source %.1f -> fixed %.1f (mean of per-clip max %.1f -> %.1f); max growth %.1f'
          % (max(r['off0'] for r in rows), max(r['off1'] for r in rows), np.mean([r['off0'] for r in rows]), np.mean([r['off1'] for r in rows]), max(r['off1'] - r['off0'] for r in rows)))
    print('foot local angle change vs source: max %.1f deg, mean of max %.1f; thigh change max %.1f' % (max(r['foot'] for r in rows), np.mean([r['foot'] for r in rows]), max(r['thigh'] for r in rows)))
    for r in sorted(rows, key=lambda r: -(r['off1'] - r['off0']))[:5]:
        print('  %-40s off-hinge %.1f -> %.1f  foot %.1f thigh %.1f' % (r['name'].replace('M_Neutral_', ''), r['off0'], r['off1'], r['foot'], r['thigh']))
