"""Rifle grip + stock 2026-11-10 (offline, read-only): support-hand grip and stock placement of the game's own player weapon poses.

For every long-gun recoil family of src/mm/RecoilProfiles.hpp (the DLL's weapon identity) the game's player animset of that family
(Extracted\\game\\player-anims\\glb\\{base,ep1}\\...\\pwa_w_*.anims.glb = Female V, pma_w_* = Male V; WolvenKit glb exports) is sampled:
idlepose_shoulder (hip / Ready) and idlepose_ironsight (ADS), every frame.  Per pose, in the WeaponRight frame (game axes, metres):
  - LeftHand position + rotation (the support-hand grip the game's own pose uses), spread over the frames;
  - RightHand position (pistol-grip hand) and the LeftHand children locals (LeftInHand* + finger locals: the game's grip fingers);
  - stock: WeaponRight relative to Spine3 and to RightShoulder.
  python -I gripstock_extract.py -> Build\\work\\rifle-grip-stock-20261110\\grip_game.json + stdout table
"""
import glob
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402

NR = WC.NR
ROOTG = os.path.join(WC.ROOT, 'Extracted', 'game', 'player-anims', 'glb')
WORK = os.path.join(WC.ROOT, 'Build', 'work', 'rifle-grip-stock-20261110')
# recoil profile id -> (family, animset key)
FAMILIES = [(12, 'ajax', 'rifle_assault__militech_ajax'), (13, 'copperhead', 'rifle_assault__nokota_copperhead'),
            (14, 'sidewinder', 'rifle_assault__nokota_sidewinder'), (15, 'masamune', 'rifle_assault__arasaka_masamune'),
            (16, 'shingen', 'smg__arasaka_shingen'), (17, 'pulsar', 'smg__darra_pulsar'), (18, 'saratoga', 'smg__militech_saratoga'),
            (19, 'senkoh', 'smg__senkoh_lx'), (23, 'defender', 'lmg__constitutional_defender'), (24, 'dian', 'special_kangtao_dian'),
            (25, 'warden', 'smg__krausser_warden'), (26, 'borg4a', 'smg__midnight_borg'), (27, 'hercules', 'special__militech_hercules')]
FINGERS = ['LeftInHandThumb', 'LeftInHandIndex', 'LeftInHandMiddle', 'LeftInHandRing', 'LeftInHandPinky', 'LeftHandThumb1', 'LeftHandThumb2',
           'LeftHandIndex1', 'LeftHandIndex2', 'LeftHandIndex3', 'LeftHandMiddle1', 'LeftHandMiddle2', 'LeftHandMiddle3', 'LeftHandRing1',
           'LeftHandRing2', 'LeftHandRing3', 'LeftHandPinky1', 'LeftHandPinky2', 'LeftHandPinky3']


def find(prefix, key):
    fs = [f for f in glob.glob(os.path.join(ROOTG, '**', prefix + '_w_' + key + '.anims.glb'), recursive=True)]
    return fs[0] if fs else None


def rel(W, a, b):
    """pose of bone b in the frame of bone a: (t [T,3], q [T,4] xyzw)."""
    Ma, Mb = W[:, a], W[:, b]
    Rt = np.swapaxes(Ma[:, :3, :3], 1, 2)
    t = np.einsum('tij,tj->ti', Rt, Mb[:, :3, 3] - Ma[:, :3, 3])
    q = R.from_matrix(Rt @ Mb[:, :3, :3]).as_quat()
    return t, q


def qmean(q):
    q = np.asarray(q, float)
    q = q * np.sign(np.sum(q * q[:1], 1))[:, None]
    m = q.mean(0)
    return m / np.linalg.norm(m)


def main():
    os.makedirs(WORK, exist_ok=True)
    out = {}
    for body, prefix in (('woman', 'pwa'), ('man', 'pma')):
        P = NR.Player(body)
        ix = P.ix
        for pid, fam, key in FAMILIES:
            path = find(prefix, key)
            if not path:
                print('MISSING', body, fam)
                continue
            sk = NR.skeleton(path)
            if list(sk.names) != P.names:
                print('RIG DIFFERS', body, fam)
                continue
            for clip in ('idlepose_shoulder', 'idlepose_ironsight'):
                try:
                    lt, lr, _, _ = NR.sample_clip(sk, clip)
                except Exception as e:  # noqa: BLE001
                    print('NO CLIP', body, fam, clip, e)
                    continue
                ltg, lrg = NR.locals_to_game(lt, lr)
                W = WC.MC.fk_world(P.par, ltg, NR.S.qcontinuous(lrg))
                tL, qL = rel(W, ix['WeaponRight'], ix['LeftHand'])
                tR, _ = rel(W, ix['WeaponRight'], ix['RightHand'])
                tS3, qS3 = rel(W, ix['Spine3'], ix['WeaponRight'])
                tRS, _ = rel(W, ix['RightShoulder'], ix['WeaponRight'])
                qm = qmean(qL)
                ang = np.degrees(2 * np.arccos(np.clip(np.abs(qL @ qm), -1, 1)))
                fing = {n: [round(float(x), 6) for x in qmean(lrg[:, ix[n]])] for n in FINGERS}
                rec = dict(pid=pid, family=fam, file=os.path.relpath(path, WC.ROOT), frames=len(W),
                           left_t=np.round(tL.mean(0), 5).tolist(), left_t_spread_cm=round(float(np.linalg.norm(tL - tL.mean(0), axis=1).max() * 100), 2),
                           left_q=np.round(qm, 6).tolist(), left_q_spread_deg=round(float(ang.max()), 2),
                           right_t=np.round(tR.mean(0), 5).tolist(), weapon_in_spine3=np.round(tS3.mean(0), 4).tolist(),
                           weapon_in_rshoulder=np.round(tRS.mean(0), 4).tolist(), fingers=fing)
                out['%s:%s:%s' % (body, fam, clip)] = rec
                print('%-5s %-10s %-18s L(cm) %6.1f %6.1f %6.1f spread %4.1f cm %4.1f deg | R(cm) %6.1f %6.1f %6.1f' % (
                    body, fam, clip, *(tL.mean(0) * 100), rec['left_t_spread_cm'], rec['left_q_spread_deg'], *(tR.mean(0) * 100)))
    WC.jdump(out, os.path.join(WORK, 'grip_game.json'))


if __name__ == '__main__':
    main()
