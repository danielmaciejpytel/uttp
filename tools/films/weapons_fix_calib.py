"""Rifle / 2H hand fix 2026-11-09 - probe 5 (read-only): left-hand twist measures on the game's own Female V weapon poses
(Extracted\\game\\player-anims\\glb\\...\\pwa_w_*.anims.glb, idlepose_shoulder / idlepose_ironsight frame 0) vs the UTPP film outputs:
  gate  = anatomy_gate wrist_twist_L (|Euler ZYX x| of the hand, rest-relative),
  swing = signed swing-twist angle of the hand local rotation about the forearm axis (+X; rest = identity) = the DLL limit measure,
  anat  = palm normal vs elbow hinge about the forearm axis (left arm: < 0 supination).
Also dumps the left-hand finger locals of every pose (for the support-grip finger pose).
  python -I weapons_fix_calib.py -> stdout + Build\\work\\rifle-hand-fix-20261109\\calib.json
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_fix_common as FC  # noqa: E402
import weapons_fix_proto as PR  # noqa: E402

NR = WC.NR
ROOTG = os.path.join(WC.ROOT, 'Extracted', 'game', 'player-anims', 'glb', 'base', 'animations', 'weapon')
KEYS = ('handgun__', 'revolver__', 'rifle_', 'smg__', 'shotgun', 'lmg__', 'special__militech_hmg')
GRIP_BONES = ['LeftInHandThumb', 'LeftInHandIndex', 'LeftInHandMiddle', 'LeftInHandRing', 'LeftInHandPinky'] + FC.FINGERS_L


def swing(W):
    ix = WC.player().ix
    Rl = np.swapaxes(W[:, ix['LeftForeArm'], :3, :3], -1, -2) @ W[:, ix['LeftHand'], :3, :3]
    return PR.twist_x(Rl)


def measures(W):
    f = FC.feats(W)
    return dict(gate=np.asarray(f['wrist_twist_L'], float), swing=swing(W), anat=PR.anat(W),
                elbow=np.asarray(f['elbow_flex_L'], float), flex=np.asarray(f['wrist_flex_L'], float))


def main():
    P = WC.player()
    out = dict(game={}, utpp={})
    files = sorted(f for f in glob.glob(os.path.join(ROOTG, '**', 'pwa_w_*.anims.glb'), recursive=True)
                   if any(k in os.path.basename(f) for k in KEYS) and 'dual_wield' not in f)
    for path in files:
        sk = NR.skeleton(path)
        if list(sk.names) != P.names:
            continue
        for clip in ('idlepose_shoulder', 'idlepose_ironsight'):
            try:
                lt, lr, _, _ = NR.sample_clip(sk, clip)
            except Exception:
                continue
            lt, lr = NR.locals_to_game(lt[:1], lr[:1])
            W = WC.fk(lt, NR.S.qcontinuous(lr))
            m = measures(W)
            key = os.path.basename(path).replace('.anims.glb', '') + ':' + clip
            out['game'][key] = {k: round(float(v[0]), 1) for k, v in m.items()}
            out['game'][key]['grip'] = {b: [round(float(x), 6) for x in lr[0, P.ix[b]]] for b in GRIP_BONES}
            print('GAME %-62s gate %6.1f swing %7.1f anat %7.1f elbow %6.1f flex %6.1f' % (key, m['gate'][0], m['swing'][0], m['anat'][0],
                                                                                          m['elbow'][0], m['flex'][0]))
    g = out['game']
    for k in ('gate', 'swing', 'anat'):
        v = np.array([x[k] for x in g.values()])
        out['game_range_' + k] = [round(float(v.min()), 1), round(float(v.max()), 1)]
    print('GAME RANGE', {k: out['game_range_' + k] for k in ('gate', 'swing', 'anat')})
    for name in ('k5_global', 'f2_nue_utpp', 'f2_ajax_utpp'):
        W = np.load(os.path.join(WC.TMP, name + '.world.npz'))['W'].astype(float)
        m = measures(W)
        out['utpp'][name] = {k: FC.summ(v) for k, v in m.items()}
        print('UTPP', name, json.dumps(out['utpp'][name]))
    FC.WC.jdump(out, os.path.join(FC.WORK, 'calib.json'))


if __name__ == '__main__':
    main()
