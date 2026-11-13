"""Where a capsule meets the jamb strip: prints, for chosen capsules and times, the capsule end points (vehicle frame) and the nearest strip points.

  python -I jamb_where.py --clip <glb> --class utility --seat driver --caps shin_r,hand_r --times 1.3,1.75
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

EX = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\glb')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', required=True)
    ap.add_argument('--caps', default='shin_r,hand_r')
    ap.add_argument('--times', default='1.3,1.75')
    a = ap.parse_args()
    name = os.path.basename(a.clip).replace('.anims.glb', '').replace('utpp_car_', '')
    m = C.Model(a.clip, a.cls, a.seat, rear_ref=os.path.join(EX, name + '.anims.glb'))
    jc = m.jcloud
    cl = jc['cloud']
    print('seat', m.bodyd['json']['seat'], 'seat_z', m.bodyd['seat_z'], 'y_edge', jc['y_edge'])
    for t in [float(x) for x in a.times.split(',')]:
        x = min(t * C.FPS, len(m.t) - 1.0)
        i0 = int(x)
        i1 = min(i0 + 1, len(m.t) - 1)
        u = x - i0
        V = m.V[i0] * (1 - u) + m.V[i1] * u
        Vv = V @ C.GLTF_TO_VEH.T
        for cn in a.caps.split(','):
            ci = m.names.index(cn)
            s = m._starts[ci]
            e = m._starts[ci + 1] if ci + 1 < len(m._starts) else len(Vv)
            q = Vv[s:e]
            d2 = ((q[:, None, :] - cl[None, :, :]) ** 2).sum(-1)
            j = np.unravel_index(d2.argmin(), d2.shape)
            print('t=%.2f %-8s r=%.3f axis a=(%.2f %.2f %.2f) b=(%.2f %.2f %.2f) dmin=%.3f at pt (%.2f %.2f %.2f) from axis pt (%.2f %.2f %.2f)' % (
                t, cn, m.rad[s], q[0][0], q[0][1], q[0][2], q[-1][0], q[-1][1], q[-1][2], np.sqrt(d2.min()), cl[j[1]][0], cl[j[1]][1], cl[j[1]][2], q[j[0]][0], q[j[0]][1], q[j[0]][2]))
        print('  Hips', (m.J['Hips'][i0] @ C.GLTF_TO_VEH.T).round(2), 'Spine', (m.J['Spine'][i0] @ C.GLTF_TO_VEH.T).round(2))


if __name__ == '__main__':
    main()
