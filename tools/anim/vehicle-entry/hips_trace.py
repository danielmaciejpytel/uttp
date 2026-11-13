"""Hips (and optional joints) path of one or more clips in the vehicle frame, per clip frame (x right, y forward, z up), for sink-slide / jamb diagnosis.

  python -I hips_trace.py --class utility --seat driver --clip a.glb [--clip b.glb ...] [--joints Hips,RightLeg] [--step 2]
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
import start_gate as SG  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', required=True)
    ap.add_argument('--clip', action='append', required=True)
    ap.add_argument('--joints', default='Hips')
    ap.add_argument('--step', type=int, default=2)
    a = ap.parse_args()
    ms = [C.Model(c, a.cls, a.seat, no_rear=True, no_env=True) for c in a.clip]
    js = a.joints.split(',')
    n = max(len(m.t) for m in ms)
    for i in range(0, n, a.step):
        row = ['%3d %.2f' % (i, i / 30.0)]
        for m in ms:
            k = min(i, len(m.t) - 1)
            for j in js:
                p = m.J[j][k] @ C.GLTF_TO_VEH.T if j in m.J else m.root_g[k] @ C.GLTF_TO_VEH.T
                row.append('%s(%.3f %.3f %.3f)' % (j[:5], p[0], p[1], p[2]))
        print(' | '.join(row))
    for c, m in zip(a.clip, ms):
        y, z = SG._y_z(m, 'Hips')
        print(os.path.basename(c), 'sink slide Hips %.3f back excursion %.3f' % (SG.sink_slide(y, z), SG.back_excursion(y)))


if __name__ == '__main__':
    main()
