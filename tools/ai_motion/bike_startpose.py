"""Start pose of the motorcycle get-off sources: frame 0 of the FPP exit (p?a_sportbike exit_l / exit_r) and of the TPP exit (tpp_p?a_sportbike exit_l / exit_r) against the TPP riding pose
(tpp_p?a_sportbike drive_slow_bike frame 0, the pose V rides in under UTPP third person). Game rig dumps (seat frame, metres in the print as cm): max distance over Hips / Spine3 / Head / both ankles /
both hands, Spine3 pitch difference (deg).

  python -I bike_startpose.py <glb dir (Extracted\\game\\vehicle-bike\\glb)>   -> one line per set / gender / side
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import game_skeleton_bridge as CB  # noqa: E402

J = ('Hips', 'Spine3', 'Head', 'LeftFoot', 'RightFoot', 'LeftHand', 'RightHand')


def pose(glb, clip):
    W, _ = CB.world_from_glb(glb, clip, 0.0, 0.0)
    names, _, _ = CB.load_rig()
    return W[0], {n: i for i, n in enumerate(names)}


def pitch(M):
    f = M[:3, 1]   # bone forward axis in game space (rig dependent, only differences are used)
    return float(np.degrees(np.arctan2(f[2], np.linalg.norm(f[:2]))))


def main(a):
    d = a[0]
    for g in ('w', 'm'):
        for b in ('', '2', '3'):
            tpp = os.path.join(d, 'tpp_p%sa_sportbike%s.anims.glb' % (g, b))
            fpp = os.path.join(d, 'p%sa_sportbike%s.anims.glb' % (g, b))
            R, ix = pose(tpp, 'drive_slow_bike')
            for s in ('l', 'r'):
                row = []
                for tag, glb in (('FPP', fpp), ('TPP', tpp)):
                    P, _ = pose(glb, 'exit_' + s)
                    dist = {j: float(np.linalg.norm(P[ix[j], :3, 3] - R[ix[j], :3, 3])) * 100 for j in J}
                    wj = max(dist, key=dist.get)
                    dp = abs(pitch(P[ix['Spine3']]) - pitch(R[ix['Spine3']]))
                    row.append('%s max %.1f cm (%s) hips %.1f hands %.1f/%.1f spine3 pitch %.1f deg' % (tag, dist[wj], wj, dist['Hips'], dist['LeftHand'], dist['RightHand'], dp))
                print('p%sa_sportbike%s exit_%s | %s' % (g, b, s, ' | '.join(row)))


if __name__ == '__main__':
    main(sys.argv[1:])
