"""Gun proxy boxes vs V's SKINNED body (not the capsule proxy) for the dw1 clips and the film timeline, 2026-11-09.

  python -I dw_skin_check.py   -> Build\\work\\dualwield-npc-20261109\\skin_gun_check.json
Skin: npc_retarget.skin_W (offline skin, the same as the media) of all dw1 clips back to back (_tmp\\skincheck\\clips_all) and the film's
right-hand cache (_tmp\\film\\gamefilm_new, if present).  Vertex labels: nearest bone segment (parent -> child of the 73 body joints) on
the first frame (A0: arms away from the torso).  Exclusions as in dw_build.gun_gates: the gun's own hand (labels <Side>Hand* and the
metacarpals <Side>InHand*), the own
forearm (<Side>ForeArm*) only against the rear slide + grip boxes.  Per clip and gun: frames with vertices inside, max count, min clearance.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dw_build as B  # noqa: E402

DC, NB, MC, NR, IX, P = B.DC, B.NB, B.MC, B.NR, B.IX, B.P
TMP = os.path.join(DC.OUT, '_tmp', 'skincheck')


def labels_for(V0, W0):
    """nearest bone segment (parent -> child) of the 73 body joints for every vertex of one posed frame -> label = parent joint name."""
    segs = [(p, j) for j, p in enumerate(B.PAR[:73]) if p >= 0]
    best, lab = np.full(len(V0), np.inf), np.zeros(len(V0), int)
    for p, j in segs:
        a, b = W0[p, :3, 3], W0[j, :3, 3]
        ab = b - a
        t = np.clip((V0 - a) @ ab / max(float(ab @ ab), 1e-12), 0, 1)
        d = np.linalg.norm(V0 - (a + t[:, None] * ab), axis=1)
        upd = d < best
        best, lab = np.where(upd, d, best), np.where(upd, p, lab)
    return np.array(P.names, dtype=object)[lab].astype(str)


def check(V, W, names):
    """V [T,N,3] skin, W [T,J,4,4] -> per gun: inside count [T], min clearance [T] (m)."""
    T = len(V)
    out = {}
    own_hand = {s: np.char.startswith(names, s + 'Hand') | np.char.startswith(names, s + 'InHand') for s in DC.SIDES}   # palm: <Side>InHand* metacarpals
    own_fore = {s: np.char.startswith(names, s + 'ForeArm') for s in DC.SIDES}
    front = [b for i, b in enumerate(B.GUN_BOXES) if i not in B.OWN_FORE_FREE]
    rear = [B.GUN_BOXES[i] for i in B.OWN_FORE_FREE]
    for s in DC.SIDES:
        cnt, clr = np.zeros(T, int), np.zeros(T)
        for t in range(T):
            Vl = B.to_local(W[t, IX[B.WPN[s]]], V[t])
            sd = np.minimum(B.gun_sdf(Vl, front), np.where(own_fore[s], 1e9, B.gun_sdf(Vl, rear)))
            sd = sd[~own_hand[s]]
            cnt[t], clr[t] = int((sd < 0).sum()), float(sd.min())
        out[s] = (cnt, clr)
    return out


def summary(res, frames, tag):
    rep = {}
    for s, (cnt, clr) in res.items():
        k = int(np.argmin(clr))
        rep[s] = dict(frames_inside=int((cnt > 0).sum()), max_inside=int(cnt.max()), min_clear_cm=round(float(clr.min() * 100), 2),
                      min_clear_frame=frames[k] if frames is not None else k)
    print(tag, rep)
    return rep


def main():
    os.makedirs(TMP, exist_ok=True)
    Ws, spans = [], {}
    n0 = 0
    for c in B.ORDER:
        z = np.load(os.path.join(B.CLIPS, '%s__%s.%s.npz' % (B.SET, c, B.SUFFIX)))
        W = NB.fk(z['lt'].astype(float), z['lr'].astype(float))
        Ws.append(W)
        spans[c] = (n0, n0 + len(W))
        n0 += len(W)
    W = np.concatenate(Ws)
    base = os.path.join(TMP, 'clips_all')
    NR.skin_W(W[:, :73], P.ref['rest'], NB.RIG, base)
    sk = np.load(base + '.npz')
    jn = [str(x) for x in sk['joint_names']]
    jdev = float(np.abs(sk['joints'] - W[:, [IX[n] for n in jn]][:, :, :3, 3]).max())
    V = np.fromfile(base + '.pc2', dtype='<f4', offset=32).reshape(len(W), -1, 3).astype(float)
    names = labels_for(V[0], W[0])
    hist = {k: int(v) for k, v in zip(*np.unique(names, return_counts=True)) if 'Hand' in k or 'ForeArm' in k}
    res = check(V, W, names)
    rep = dict(skin_joints_vs_W_max_dev_m=round(jdev, 6), vertices=int(V.shape[1]), hand_forearm_label_counts=hist, clips={})
    for c, (a, b) in spans.items():
        rep['clips'][c] = summary({s: (res[s][0][a:b], res[s][1][a:b]) for s in res}, list(range(b - a)), c)
    fb = os.path.join(DC.OUT, '_tmp', 'film', 'gamefilm_new')
    if os.path.exists(fb + '.pc2'):
        import dw_film as F
        Wf = F.timeline('new')
        Vf = np.fromfile(fb + '.pc2', dtype='<f4', offset=32).reshape(len(Wf), -1, 3).astype(float)
        rep['film_right_side'] = summary(check(Vf, Wf, names), None, 'film')
    MC.jdump(rep, os.path.join(DC.OUT, 'skin_gun_check.json'))
    print('skin joints vs W max dev m', rep['skin_joints_vs_W_max_dev_m'], 'labels', hist)


if __name__ == '__main__':
    main()
