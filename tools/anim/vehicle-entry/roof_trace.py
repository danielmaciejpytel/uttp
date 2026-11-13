"""Per-frame roof trace of one car entry clip (diagnostic for the head-through-roof work, no gates).

  python -I roof_trace.py --class sport1 --seat driver <clip.anims.glb> [--every 1] [--min-pen-cm -3]

Per clip frame (30 fps): clip time, head sphere centre in the VEHICLE frame (x right, y forward, z up, m), roof penetration (cm), the closest roof point (vehicle frame) and the
height of the roof directly above the head sphere centre (vertical clearance cm), trunk lean / neck flexion (anatomy.py), hips height above the seated hips, root yaw.
Prints only frames with penetration >= --min-pen-cm unless --all.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import check_door_clearance as C  # noqa: E402
import anatomy as AN  # noqa: E402


def trace(clip, cls, seat):
    m = C.Model(clip, cls, seat)
    rc = m.bodyd['roof']
    n = len(m.t)
    sk, t, Wt, Wr = C.load_clip(clip)
    an = AN.metrics(sk, Wt)
    hi = sk.index['Hips']
    rows = []
    for i in range(n):
        hc = m.head_c[i]
        v = (hc @ C.GLTF_TO_VEH.T).astype(np.float32)
        d = np.linalg.norm(rc - v, axis=1)
        j = int(d.argmin())
        pen = (C.ROOF_HEAD_R_M + C.ROOF_HAIR_M - float(d[j])) * 100
        near = np.flatnonzero((np.abs(rc[:, 0] - v[0]) < 0.06) & (np.abs(rc[:, 1] - v[1]) < 0.06) & (rc[:, 2] > v[2]))
        above = (float(rc[near, 2].min()) - float(v[2])) * 100 if len(near) else float('nan')
        rows.append(dict(i=i, t=float(t[i]), head=v.tolist(), pen=pen, near=rc[j].tolist(), above=above,
                         lean=float(an['trunk_lean'][i]), neck=float(an['neck_flex'][i]), hipy=float(Wt[i, hi][1])))
    return m, rows


DIRS = {'down': (0, 0, -1), 'in': None, 'out': None, 'fwd': (0, 1, 0), 'back': (0, -1, 0), 'up': (0, 0, 1)}


def probe(m, rows):
    rc = m.bodyd['roof']
    gate = float(m.gate_cm('roof')[0]) / 100.0
    xin = 1.0 if m.side == 'left' else -1.0  # vehicle x toward the car centre line (driver sits at x < 0)
    dirs = {'down': (0, 0, -1), 'in': (xin, 0, 0), 'out': (-xin, 0, 0), 'fwd': (0, 1, 0), 'back': (0, -1, 0), 'up': (0, 0, 1),
            'down+in': (xin, 0, -1), 'down+out': (-xin, 0, -1), 'down+fwd': (0, 1, -1), 'down+back': (0, -1, -1), 'up+out': (-xin, 0, 1)}
    print('gate %.2f cm; per over frame the smallest move (cm) per direction that reaches pen <= gate - 0.2 cm:' % (gate * 100))
    print('frame  pen   ' + ' '.join('%9s' % k for k in dirs))
    for r in rows:
        if r['pen'] <= gate * 100:
            continue
        v0 = np.array(r['head'], np.float32)
        out = []
        for k, d in dirs.items():
            d = np.array(d, np.float32)
            d /= np.linalg.norm(d)
            found = None
            for s_ in np.arange(0.0, 0.40, 0.005):
                q = (v0 + d * s_)[None]
                dist = np.linalg.norm(rc - q, axis=1).min()
                if (C.ROOF_HEAD_R_M + C.ROOF_HAIR_M - dist) <= gate - 0.002:
                    found = s_
                    break
            out.append('%9s' % ('--' if found is None else '%.1f' % (found * 100)))
        print('f%02d  %6.2f ' % (r['i'], r['pen']) + ' '.join(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', required=True)
    ap.add_argument('--min-pen-cm', type=float, default=-3.0)
    ap.add_argument('--all', action='store_true')
    ap.add_argument('--probe', action='store_true', help='per over-gate frame: the smallest head-centre move (m, vehicle frame) that gets the sphere under the gate - 0.2 cm, per direction')
    ap.add_argument('clip')
    a = ap.parse_args()
    m, rows = trace(a.clip, a.cls, a.seat)
    if a.probe:
        probe(m, rows)
        return 0
    hs = rows[-1]['hipy']
    print('seat z %.3f; seated head %s; roof pts %d' % (m.bodyd['seat_z'], np.round(rows[-1]['head'], 3), len(m.bodyd['roof'])))
    for r in rows:
        if a.all or r['pen'] >= a.min_pen_cm:
            print('f%02d t%.2f head(x%.2f y%.2f z%.2f) pen %6.2f near(x%.2f y%.2f z%.2f) above %6.1f lean %4.1f neck %4.1f hips+%.2f' % (
                r['i'], r['t'], *r['head'], r['pen'], *r['near'], r['above'], r['lean'], r['neck'], r['hipy'] - hs))
    return 0


if __name__ == '__main__':
    sys.exit(main())
