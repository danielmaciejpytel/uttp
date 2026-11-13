"""Glide of the ROUND-TRIPPED clips of an entry gate run (what would ship): the roundtrip GLB of every row of <gate dir> against the shipped roundtrip GLB, same plant intervals
(base/<row>/plants.json) and the same metric as entry_steps_gen.py measure. Kimodo-env python.

  python -I entry_steps_rt.py <gate dir (entry_steps_gates.py / build_car.ps1 -Out parent)> [--rows a,b]
Prints per row: glide max/mean before -> after, planted frames glide>1 before -> after, dv, ankle shift (cm) and KEEP / DROP (keep only if both the worst glide and the mean drop)."""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import entry_steps_gen as E  # noqa: E402
import game_skeleton_bridge as CB  # noqa: E402

BASE = E.ROOT + r'\Build\work\vehentry-mb-20261106\base'


def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()[:12] if os.path.exists(p) else '-'


def main(a):
    gd = a[0]
    rows = a[a.index('--rows') + 1].split(',') if '--rows' in a else sorted(f[:-4] for f in os.listdir(gd) if f.endswith('.log'))
    ship = dict(E.rows())
    for row in rows:
        row = row if row.startswith('utpp_car_') else 'utpp_car_' + row
        rt = os.path.join(gd, row, 'roundtrip', row + '.anims.glb')
        if not os.path.exists(rt):
            print(row[9:], 'no roundtrip'); continue
        names = [str(x) for x in np.load(os.path.join(BASE, row, 'cur.dump.npz'), allow_pickle=True)['names']]
        W0 = np.load(os.path.join(BASE, row, 'cur.dump.npz'), allow_pickle=True)['W']
        W1, _ = CB.world_from_glb(rt, 'enter')
        pl = json.load(open(os.path.join(BASE, row, 'plants.json')))
        r0, r1 = E.plant_summary(W0, names, pl), E.plant_summary(W1, names, pl)
        f0, f1 = E.feet_motion(W0, names), E.feet_motion(W1, names)
        ix = {n: i for i, n in enumerate(names)}
        sh = max(float(np.linalg.norm(W1[:, ix[s + 'Foot'], :3, 3] - W0[:, ix[s + 'Foot'], :3, 3], axis=1).max()) * 100 for s in E.SIDES)
        m0, m1 = max(r0[s]['max'] for s in E.SIDES), max(r1[s]['max'] for s in E.SIDES)
        a0, a1 = np.mean([r0[s]['mean'] for s in E.SIDES]), np.mean([r1[s]['mean'] for s in E.SIDES])
        keep = m1 < m0 and a1 < a0
        print('%-28s glide max %.2f -> %.2f | mean %.3f -> %.3f | >1cm/fr frames %d -> %d | dv %.1f -> %.1f | shift %.1f cm | %s | anims sha %s' % (
            row[9:], m0, m1, a0, a1, sum(len(r0[s]['over']) for s in E.SIDES), sum(len(r1[s]['over']) for s in E.SIDES), max(f0[s][1] for s in E.SIDES), max(f1[s][1] for s in E.SIDES), sh,
            'KEEP' if keep else 'DROP', sha(os.path.join(gd, row, 'utpp_vehicle_car', 'utpp', 'vehicle', row + '.anims'))))


if __name__ == '__main__':
    main(sys.argv[1:])
