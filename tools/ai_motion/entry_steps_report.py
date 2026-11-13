"""Per-clip table of the entry glide pin (2026-11-06): glide before / after, pinned runs, swings, ankle shift, gate exit codes, solved door close vs shipped, run dir.
Kimodo-env python.   python -I entry_steps_report.py <batch dir (entry_steps_batch.py output)> <gate dir (entry_steps_gates.py output)> [--gate-rows-from <other gate dir>:<row,row>]
"""
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import entry_steps_gen as E  # noqa: E402


def main(a):
    bd, gd = a[0], a[1]
    shipped = dict(E.rows())
    out = []
    for row in sorted(f for f in os.listdir(gd) if f.endswith('.log')):
        row = row[:-4]
        rd = os.path.join(bd, row)
        if not os.path.exists(os.path.join(rd, 'after_pin.dump.npz')):
            continue
        d0 = np.load(os.path.join(rd, 'cur.dump.npz'), allow_pickle=True)
        d1 = np.load(os.path.join(rd, 'after_pin.dump.npz'), allow_pickle=True)
        names = [str(x) for x in d0['names']]
        pl = json.load(open(os.path.join(rd, 'plants.json')))
        r0, r1 = E.plant_summary(d0['W'], names, pl), E.plant_summary(d1['W'], names, pl)
        ix = {n: i for i, n in enumerate(names)}
        sh = max(float(np.linalg.norm(d1['W'][:, ix[s + 'Foot'], :3, 3] - d0['W'][:, ix[s + 'Foot'], :3, 3], axis=1).max()) * 100 for s in E.SIDES)
        f0, f1 = E.feet_motion(d0['W'], names), E.feet_motion(d1['W'], names)
        sw0, _ = E.swings(d0['W'], names, pl)
        sw1, _ = E.swings(d1['W'], names, pl)
        n0 = sum(1 for s in E.SIDES for x in sw0[s] if x[2] >= 20)
        n1 = sum(1 for s in E.SIDES for x in sw1[s] if x[2] >= 20)
        txt = open(os.path.join(gd, row + '.log'), errors='replace').read()
        m = re.search(r'^CHECKS: (.*)$', txt, re.M)
        checks = m.group(1) if m else 'NO CHECKS LINE (run failed)'
        st = json.load(open(os.path.join(rd, 'splice-report.json')))
        spans = [(s['c'], s['g']) for s in st['spans']]
        ca_new = ca_old = None
        fn = os.path.join(gd, row, 'door-close-%s.json' % row)
        if os.path.exists(fn):
            ca_new = json.load(open(fn)).get('close_after_s')
        fo = os.path.join(shipped[row], 'door-close-%s.json' % row)
        if os.path.exists(fo):
            ca_old = json.load(open(fo)).get('close_after_s')
        out.append('%s\n  glide max/mean (cm/fr) %.2f/%.2f -> %.2f/%.2f | planted frames glide>1: %d -> %d | speed dv max %.1f -> %.1f | steps(swing>=20cm) %d -> %d | ankle shift max %.1f cm\n'
                   '  splice span %s changed frames %s..%s | %s | close_after new %s shipped %s | run %s' % (
                       row[9:], max(r0[s]['max'] for s in E.SIDES), np.mean([r0[s]['mean'] for s in E.SIDES]), max(r1[s]['max'] for s in E.SIDES), np.mean([r1[s]['mean'] for s in E.SIDES]),
                       sum(len(r0[s]['over']) for s in E.SIDES), sum(len(r1[s]['over']) for s in E.SIDES), max(f0[s][1] for s in E.SIDES), max(f1[s][1] for s in E.SIDES), n0, n1, sh,
                       spans, st['changed_frames'][:1], st['changed_frames'][-1:], checks, ca_new, ca_old, os.path.join(gd, row)))
    print('\n'.join(out))


if __name__ == '__main__':
    main(sys.argv[1:])
