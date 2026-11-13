"""Rifle / 2H hand fix 2026-11-09 - bug 3 check of the "mesh origin = WeaponRight" assumption: the REAL Ajax stock (mesh vertices within
4 cm of the rearmost point, weapon frame) placed by WeaponRight on the game's OWN Ajax pose (weapon-film game side) and on the UTPP output;
per frame the distance from the stock to the skinned body (vertices owned by the right clavicle / upper arm / chest bones) and which bone
owns the nearest vertex. If the assumption holds, the game pose puts the butt into the shoulder pocket (about 0-3 cm from the skin).
Uses the skin caches of the weapon films (Build\\work\\weapon-films-20261109\\_tmp\\f2_ajax_{game,utpp}.npz/.pc2).
  python -I weapons_fix_stock.py -> stdout + Build\\work\\rifle-hand-fix-20261109\\stock.json
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_fix_common as FC  # noqa: E402
import weapons_sheet as WS  # noqa: E402


def main():
    V, _ = FC.ajax_mesh()
    rear = V[:, 1].min()
    stock = V[V[:, 1] < rear + 0.04]
    P = WC.player()
    ix = P.ix
    joints = [j for j in range(73) if P.names[j] not in WS.SKIP_SEG]
    res = dict(stock_rear_y_cm=round(float(rear * 100), 1), stock_points=int(len(stock)))
    for name in ('f2_ajax_game', 'f2_ajax_utpp'):
        W = np.load(os.path.join(WC.TMP, name + '.world.npz'))['W'].astype(float)
        sk = np.load(os.path.join(WC.TMP, name + '.npz'))
        B = WS.pc2(os.path.join(WC.TMP, name + '.pc2'), len(sk['rest'])).astype(float)
        T = min(len(W), len(B))
        dmin, owner = np.zeros(T), []
        for f in range(T):
            M = W[f, ix['WeaponRight']]
            S = stock @ M[:3, :3].T + M[:3, 3]
            jp = W[f, joints, :3, 3]
            _, own = cKDTree(jp).query(B[f])
            keep = np.array([('Hand' not in P.names[joints[o]]) and ('ForeArm' not in P.names[joints[o]]) for o in own])
            d, k = cKDTree(B[f][keep]).query(S)
            dmin[f] = d.min() * 100
            owner.append(P.names[joints[own[keep][k[int(np.argmin(d))]]]])
        u, c = np.unique(owner, return_counts=True)
        res[name] = dict(frames=T, stock_to_skin_cm=FC.summ(dmin), frames_lt_1cm=int(np.sum(dmin < 1.0)),
                         nearest_bone=dict(zip(u.tolist(), c.tolist())))
        print(name, res[name])
    print('rear of the stock (weapon frame y, cm):', res['stock_rear_y_cm'])
    WC.jdump(res, os.path.join(FC.WORK, 'stock.json'))


if __name__ == '__main__':
    main()
