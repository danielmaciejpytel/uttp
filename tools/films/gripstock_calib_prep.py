"""Rifle grip + stock 2026-11-10 - fast input for gripstock_stock_calib.py: evaluator run + skin only (no per-frame mesh metrics of the probe).
  python -I gripstock_calib_prep.py <mode> <family> <body:fem>[,<body:fem>...]
-> Build\\work\\rifle-grip-stock-20261110\\arr_<mode>_<family>_main_<body>_<fem>.npz (W, lab) + _tmp\\<same>_skin.* (shared skin cache)
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripstock_common as G  # noqa: E402


def main():
    mode, fam = sys.argv[1], sys.argv[2]
    rec = '%s_single_shot_ironsight.utpprecoil' % fam
    for spec in sys.argv[3].split(','):
        body, fem = spec.split(':')
        fem = float(fem)
        name = '%s_%s_main_%s_%g' % (mode, fam, body, fem)
        out = os.path.join(G.WORK, 'arr_%s.npz' % name)
        if os.path.exists(out):
            print('exists', out)
            continue
        tl, shots, marks = G.timeline('main', rec, fem)
        r = G.run(mode, 'cp_' + name, tl, shots, body, G.PROFILE[fam])
        W = r['W']
        G.skin(W, body, os.path.join(G.TMP, name + '_skin'))
        np.savez(out, W=W.astype(np.float32), lab=np.array(G.phase_of(marks, len(W))))
        print('wrote', out)


if __name__ == '__main__':
    main()
