"""Rifle grip + stock 2026-11-10 - stock contact calibration (offline, read-only inputs, writes stock_calib.json).

(1) Shoulder pocket per body, on the skinned UTPP output of the BEFORE build (gripstock_calib_prep.py / gripstock_probe.py arrays + skin
    caches), rifle AIM frames only (the overlay's Ready / Relaxed / sprint carry holds the butt 10-14 cm above / beside the shoulder: no
    contact along the barrel there, so the stock pass is an Aim-only pass): per frame the slide s_c of the REAL stock mesh back along the
    barrel until it touches the body skin (non-hand / forearm vertices, contact = 0.3 cm) and the touched skin vertex P_c. P_c in the
    RightArm frame (the touched skin is RightArm-owned in every aim frame; across shoulder widths 0 / 5 cm the RightArm point keeps the
    median residual of the predicted slide at -0.11 / +0.13 cm, the RightShoulder point -0.61 / +0.77 cm); median = the pocket.
(2) stockY of every family = median weapon-frame y of the stock point that touches the pocket, measured with that family's REAL mesh
    (base + default stock + standard magazine) on the same aim frames. Families without an extracted mesh get no stockY (stock pass off).
    The earlier stockY from the game's FPP idlepose_shoulder was dropped: it measures the FPP body fit, not the stock (Saratoga -34.5 cm
    there vs -27.6 cm on the mesh; copperhead -22.3 woman vs -18.6 man for the same weapon).
  python -I gripstock_stock_calib.py <run names...>   (e.g. before_ajax_main_woman_0 before_ajax_main_woman_0.05 before_ajax_main_man_0)
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gripstock_common as G  # noqa: E402

BONE = 'RightArm'
AIM = ('aim + shots', 'walk aim')
CONTACT = 0.003


def slide_contact(W, B, body, fam, frames):
    """per frame: (frame, slide m or None, touched skin point, weapon-frame y of the touching stock point)."""
    P = G.player(body)
    ix = P.ix
    V, F, _ = G.weapon_mesh(fam)
    rear = V[:, 1].min()
    stock = V[V[:, 1] < rear + 0.04][::2]
    joints = [j for j in range(73) if P.names[j] not in ('Root', 'Trajectory', 'reference_joint', 'WeaponLeft', 'WeaponRight', 'LeftEye', 'RightEye')]
    rows = []
    for f in frames:
        _, own = cKDTree(W[f, joints, :3, 3]).query(B[f])
        on = [P.names[joints[o]] for o in own]
        keep = np.array([('Hand' not in n) and ('ForeArm' not in n) for n in on])
        body_pts = B[f][keep]
        tree = cKDTree(body_pts)
        M = W[f, ix['WeaponRight']]
        d = M[:3, 1]
        S = stock @ M[:3, :3].T + M[:3, 3]
        sc, pc, yr = None, None, None
        for s in np.arange(-0.03, 0.20, 0.0025):
            dist, k = tree.query(S - s * d)
            if dist.min() < CONTACT:
                i = int(np.argmin(dist))
                sc, pc, yr = s, body_pts[k[i]], stock[i, 1]
                break
        rows.append((f, sc, pc, yr))
    return rows


def load(name):
    z = np.load(os.path.join(G.WORK, 'arr_%s.npz' % name))
    W = z['W'].astype(float)
    lab = list(z['lab'])
    skn = os.path.join(G.TMP, name + '_skin')
    nv = len(np.load(skn + '.npz')['rest'])
    B = np.fromfile(skn + '.pc2', dtype='<f4', offset=32).reshape(-1, nv, 3).astype(float)
    return W, lab, B


def main():
    names = sys.argv[1:]
    res = dict(runs=names, contact_cm=CONTACT * 100, bone=BONE, frames='aim only')
    runs = {}
    for name in names:
        body = 'man' if '_man_' in name else 'woman'
        W, lab, B = load(name)
        frames = [f for f in range(0, min(len(W), len(B)), 2) if lab[f] in AIM]
        runs[name] = (body, W, B, frames)
    # (1) pocket per body (Ajax mesh: the arrays are the Ajax runs)
    for body in ('woman', 'man'):
        data = []
        for name, (b, W, B, frames) in runs.items():
            if b == body:
                data += [(name, W[f], sc, pc, yr) for f, sc, pc, yr in slide_contact(W, B, body, 'ajax', frames) if sc is not None]
        if not data:
            continue
        ix = G.player(body).ix
        loc = np.array([(pc - Wf[ix[BONE], :3, 3]) @ Wf[ix[BONE], :3, :3] for _, Wf, sc, pc, yr in data])
        pl = np.median(loc, 0)
        resid = {}
        for name, Wf, sc, pc, yr in data:
            pk = Wf[ix[BONE], :3, :3] @ pl + Wf[ix[BONE], :3, 3]
            M = Wf[ix['WeaponRight']]
            resid.setdefault(name, []).append(((M[:3, 3] - pk) @ M[:3, 1] + yr - sc) * 100)
        allr = np.concatenate([np.array(v) for v in resid.values()])
        res['pocket_' + body] = dict(bone=BONE, local=np.round(pl, 5).tolist(), resid_std_cm=round(float(allr.std()), 2),
                                     resid_cm={n: G.summ(v) for n, v in resid.items()}, frames=len(data))
        print(body, 'pocket', BONE, 'local cm', np.round(pl * 100, 2), 'frames', len(data), 'residual cm',
              {n.split('_')[-1]: G.summ(v) for n, v in resid.items()})
    # (2) contact y of every family with an extracted mesh, on the woman runs (same overlay pose for every rifle family)
    for fam in G.PROFILE:
        if fam not in G.MESHDIR:
            print('stockY', fam, 'no mesh -> stock pass off')
            continue
        ys, slides = [], []
        for name, (b, W, B, frames) in runs.items():
            for f, sc, pc, yr in slide_contact(W, B, b, fam, frames):
                if sc is not None:
                    ys.append(yr)
                    slides.append(sc)
        V, F, files = G.weapon_mesh(fam)
        if not ys:
            print('stockY', fam, 'no contact')
            continue
        if np.median(slides) > 0.08:  # no plausible stock (compact SMG / missing stock part): the base rear is not a stock
            print('stockY', fam, 'implausible: median slide %.1f cm > 8 cm -> stock pass off' % (np.median(slides) * 100))
            continue
        res[fam] = dict(contact_y=round(float(np.median(ys)), 5), rear_y=round(float(V[:, 1].min()), 5),
                        slide_cm=G.summ(np.array(slides) * 100), frames=len(ys), mesh=[os.path.basename(f) for f in files])
        print('stockY', fam, round(float(np.median(ys)) * 100, 2), 'cm (rear %.2f)' % (V[:, 1].min() * 100), 'slide cm', G.summ(np.array(slides) * 100),
              'frames', len(ys))
    G.WC.jdump(res, os.path.join(G.WORK, 'stock_calib.json'))


if __name__ == '__main__':
    main()
