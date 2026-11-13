"""Rifle / 2H hand fix 2026-11-09 - prototype (read-only) of the support-hand twist limit planned for ForearmTwist.hpp:
after the forearm roll moved into the hand, the left upper arm + forearm turn rigidly about the shoulder -> wrist axis (swivel) until the
hand twist about the forearm axis is at most LIMIT (hand model transform unchanged).  Also the anatomical forearm rotation
(palm normal vs elbow hinge about the forearm axis; left arm: < 0 supination, > 0 pronation, 0 = thumb up).
  python -I weapons_fix_proto.py [limit_deg] [max_swivel_deg] -> stdout + Build\\work\\rifle-hand-fix-20261109\\proto.json
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_fix_common as FC  # noqa: E402

FE = ['wrist_twist_L', 'wrist_flex_L', 'wrist_dev_L', 'elbow_torso_cm_L', 'upperarm_torso_cm_L', 'forearm_torso_cm_L', 'pronation_L']


def twist_x(Rloc):
    """signed swing-twist angle (deg) of local rotation matrices about local +X."""
    q = R.from_matrix(Rloc).as_quat()
    q = np.where(q[..., 3:4] < 0, -q, q)
    return np.degrees(2 * np.arctan2(q[..., 0], q[..., 3]))


def anat(W):
    ix = WC.player().ix
    S, E, H = (W[:, ix[b], :3, 3] for b in ('LeftArm', 'LeftForeArm', 'LeftHand'))
    I, Pk, M = (W[:, ix[b], :3, 3] for b in ('LeftHandIndex1', 'LeftHandPinky1', 'LeftHandMiddle1'))
    u = (E - S) / np.linalg.norm(E - S, axis=1, keepdims=True)
    f = (H - E) / np.linalg.norm(H - E, axis=1, keepdims=True)
    h = np.cross(u, f)
    h /= np.maximum(np.linalg.norm(h, axis=1, keepdims=True), 1e-9)
    n = np.cross(M - H, I - Pk)
    n -= f * np.sum(n * f, 1, keepdims=True)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    return np.degrees(np.arctan2(np.sum(np.cross(h, n) * f, 1), np.sum(h * n, 1)))


def limit(W, lim, maxsw, weight=1.0):
    """W [T,N,4,4] -> (W', swivel deg [T], twist before [T], twist after [T])"""
    ix = WC.player().ix
    W = W.copy()
    T = len(W)
    sw, tb, ta = np.zeros(T), np.zeros(T), np.zeros(T)
    for t in range(T):
        S = W[t, ix['LeftArm'], :3, 3]
        H = W[t, ix['LeftHand'], :3, 3]
        a = (H - S) / np.linalg.norm(H - S)
        Rfo, Rh = W[t, ix['LeftForeArm'], :3, :3], W[t, ix['LeftHand'], :3, :3]

        def tw(phi):
            Rr = R.from_rotvec(a * np.radians(phi)).as_matrix()
            return float(twist_x((Rr @ Rfo).T @ Rh))
        t0 = tw(0.0)
        tb[t] = ta[t] = t0
        if abs(t0) <= lim:
            continue
        d = -1.0 if abs(tw(-1.0)) < abs(tw(1.0)) else 1.0
        lo, hi = 0.0, maxsw
        if abs(tw(d * hi)) > lim:
            phi = d * hi
        else:
            for _ in range(24):
                mid = 0.5 * (lo + hi)
                if abs(tw(d * mid)) > lim:
                    lo = mid
                else:
                    hi = mid
            phi = d * hi
        phi *= weight
        Rr = R.from_rotvec(a * np.radians(phi)).as_matrix()
        for b in ('LeftArm', 'LeftForeArm'):
            Mx = W[t, ix[b]].copy()
            W[t, ix[b], :3, :3] = Rr @ Mx[:3, :3]
            W[t, ix[b], :3, 3] = S + Rr @ (Mx[:3, 3] - S)
        sw[t] = phi
        ta[t] = tw(phi)
    return W, sw, tb, ta


def main():
    lim = float(sys.argv[1]) if len(sys.argv) > 1 else 85.0
    maxsw = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    out = {}
    ix = WC.player().ix
    for name in ('f2_nue_game', 'f2_ajax_game'):
        W = np.load(os.path.join(WC.TMP, name + '.world.npz'))['W'].astype(float)
        out[name] = dict(anat=FC.summ(anat(W)), wrist_twist=FC.summ(FC.feats(W)['wrist_twist_L']))
        print(name, out[name])
    for name in ('k5_global', 'f2_nue_utpp', 'f2_ajax_utpp'):
        W = np.load(os.path.join(WC.TMP, name + '.world.npz'))['W'].astype(float)
        W2, sw, tb, ta = limit(W, lim, maxsw)
        f0, f1 = FC.feats(W), FC.feats(W2)
        a0, a1 = anat(W), anat(W2)
        el = np.linalg.norm(W2[:, ix['LeftForeArm'], :3, 3] - W[:, ix['LeftForeArm'], :3, 3], axis=1) * 100
        hand = np.abs(W2[:, ix['LeftHand']] - W[:, ix['LeftHand']]).max()
        out[name] = dict(swivel=FC.summ(sw), elbow_move_cm=FC.summ(el), hand_err=float(hand), anat_before=FC.summ(a0), anat_after=FC.summ(a1),
                         before={k: FC.summ(f0[k]) for k in FE}, after={k: FC.summ(f1[k]) for k in FE})
        print(name, json.dumps(out[name]))
        np.savez_compressed(os.path.join(FC.WORK, 'proto_%s.npz' % name), W=W2.astype(np.float32), swivel=sw)
    FC.WC.jdump(dict(limit=lim, max_swivel=maxsw, res=out), os.path.join(FC.WORK, 'proto.json'))


if __name__ == '__main__':
    main()
