"""Rifle grip + stock 2026-11-10 - probe / evaluation (offline): per-phase metrics of gripstock_common.measure for one evaluator build.
  python -I gripstock_probe.py <before|after> [family=ajax] [scn=main] [bodies=woman,man] [fems=0,0.025,0.05]
-> Build\\work\\rifle-grip-stock-20261110\\probe_<mode>_<family>_<scn>.json (+ .npz with the per-frame arrays) and a stdout table.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripstock_common as G  # noqa: E402

KEYS = ('grip_err_cm', 'reach', 'reach_target', 'elbow_flex', 'stock_gap_cm', 'stock_gap_pocket_cm', 'stock_in', 'hand_in', 'hand_depth_cm')


def main():
    a = sys.argv[1:]
    mode = a[0]
    fam = a[1] if len(a) > 1 else 'ajax'
    scn = a[2] if len(a) > 2 else 'main'
    bodies = (a[3] if len(a) > 3 else 'woman,man').split(',')
    fems = [float(x) for x in (a[4] if len(a) > 4 else '0,0.025,0.05').split(',')]
    V, F, files = G.weapon_mesh(fam)
    rec = 'ajax_single_shot_ironsight.utpprecoil' if fam == 'ajax' else '%s_single_shot_ironsight.utpprecoil' % fam
    res, arrays = dict(mesh=files), {}
    for body in bodies:
        for fem in (fems if body == 'woman' else [0.0]):
            tl, shots, marks = G.timeline(scn, rec, fem)
            name = '%s_%s_%s_%s_%g' % (mode, fam, scn, body, fem)
            r = G.run(mode, name, tl, shots, body, G.PROFILE[fam])
            W = r['W']
            B = G.skin(W, body, os.path.join(G.TMP, name + '_skin'))
            m = G.measure(W, B, body, fam, mesh=(V, F))
            lab = G.phase_of(marks, len(W))
            diag = r['diag']
            m['twist_after'] = np.asarray(diag['twistAfter'], float)
            m['support_w'] = np.asarray(diag['supportW'], float)
            m['stock_w'] = np.asarray(diag['stockW'], float)
            m['stock_shift_cm'] = np.asarray(diag['stockShift'], float) * 100
            m['grip_left_err_cm'] = np.asarray(diag['gripLeftErr'], float) * 100
            m['shoot_scale'] = np.where(np.asarray(diag['shootApplied'], float) > 0, np.asarray(diag['shootScale'], float), np.nan)
            key = '%s_%g' % (body, fem)
            res[key] = {}
            for ph in dict.fromkeys(lab):
                sel = np.array([x == ph for x in lab])
                row = {k: G.summ(np.asarray(m[k], float)[sel]) for k in KEYS + ('twist_after', 'support_w', 'stock_w', 'stock_shift_cm',
                                                                                  'grip_left_err_cm', 'shoot_scale')}
                bones = [m['stock_gap_bone'][f] for f in np.nonzero(sel)[0] if m['stock_gap_bone'][f]]
                row['gap_bone'] = {b: bones.count(b) for b in sorted(set(bones))}
                res[key][ph] = row
                for k in ('weapon_in_rshoulder_cm', 'weapon_in_spine3_cm'):
                    row[k] = np.round(np.median(np.asarray(m[k])[sel], 0), 1).tolist()
                print('%-10s %-12s grip %-20s reachT %-20s gap %-20s pocket %-20s stock_in %-14s hand_in %-14s depth %-16s twist %s' % (
                    key, ph, row['grip_err_cm'], row['reach_target'], row['stock_gap_cm'], row['stock_gap_pocket_cm'], row['stock_in'], row['hand_in'], row['hand_depth_cm'],
                    row['twist_after']), 'wpn@RS', row['weapon_in_rshoulder_cm'], 'shift', row['stock_shift_cm'], 'scale', row['shoot_scale'])
            arrays[key] = m
            np.savez(os.path.join(G.WORK, 'arr_%s.npz' % name), W=W.astype(np.float32), lab=np.array(lab),
                     **{k: np.asarray(v, float) for k, v in m.items() if k != 'stock_gap_bone'})
    G.WC.jdump(res, os.path.join(G.WORK, 'probe_%s_%s_%s.json' % (mode, fam, scn)))


if __name__ == '__main__':
    main()
