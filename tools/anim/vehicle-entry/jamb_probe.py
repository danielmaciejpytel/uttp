"""Per-capsule probe of the jamb phase and the rearward gate of one car entry clip (round 7 utility rows).

  python -I jamb_probe.py --clip <glb> --class utility --seat driver [--van auto] [--trace capsule] [--frames]

Prints the jamb strip (cloud size, y range, x range), the gate and the worst penetration per capsule (cm, time, frame) and the Hips / Spine y,z at a few frames
(vehicle frame) so the leg / arm path through the jamb can be read off. Read-only; no gate is changed.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
import start_gate as SG  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

EX = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\glb')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', required=True)
    ap.add_argument('--van', default='auto')
    ap.add_argument('--trace', default=None, help='capsule name: print its jamb penetration per 0.05 s')
    ap.add_argument('--joint', default=None, help='capsule/joint name for a position trace (capsule centre points)')
    a = ap.parse_args()
    name = os.path.basename(a.clip).replace('.anims.glb', '').replace('utpp_car_', '')
    van = os.path.join(EX, name + '.anims.glb') if a.van == 'auto' else a.van
    m = C.Model(a.clip, a.cls, a.seat, rear_ref=van)
    jc = m.jcloud
    print('jamb strip: n=%d y_edge=%.3f strip=%.3f x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]' % (jc['n'], jc['y_edge'], jc['strip_m'], jc['cloud'][:, 0].min(), jc['cloud'][:, 0].max(),
          jc['cloud'][:, 1].min(), jc['cloud'][:, 1].max(), jc['cloud'][:, 2].min(), jc['cloud'][:, 2].max()))
    js = m.jamb_series()
    gate = m.gate_cm('jamb', 1.0)
    pv = js[0].max(axis=0) * 100
    tv = js[0].argmax(axis=0) * C.DT
    print('capsule      worst_cm  gate_cm  over   t_s   seated_cm  game_cm')
    ref = m.jamb_ref_caps()
    for c, nm in enumerate(m.names):
        print('%-12s %7.2f %7.2f %6.2f %6.3f %8.2f %8.2f' % (nm, pv[c], gate[c], pv[c] - gate[c], tv[c], js[1][c] * 100, (ref[c] * 100 if ref is not None else float('nan'))))
    if a.trace:
        c = m.names.index(a.trace)
        for i in range(0, js[0].shape[0], 6):
            P = m.V[min(int(i * C.DT * C.FPS), len(m.V) - 1)]
            print('  t=%.3f pen=%6.2f cm' % (i * C.DT, js[0][i, c] * 100))
    rep = SG.evaluate(a.clip, a.cls, a.seat, van, exempt=False)
    rw = rep['rearward']['rows']
    print('start', {k: rep['start'][k] for k in ('dy_vs_vanilla_m', 'dx_vs_vanilla_m', 'slide_m', 'ok')})
    print('rear', {k: (v['excursion_m'], v['game_m'], v['ok']) for k, v in rw.items()})
    print('joint rule ok', rep['joint_rule']['ok'])


if __name__ == '__main__':
    main()
