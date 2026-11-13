"""Top-out (pull-up onto a ledge above the 2.5 m climb, e.g. 3.0 / 3.5 m) from the source pack Climb_Start_2_5 pull-up: the hands already hold
the ledge, so the pull-up is the same movement for a taller wall - the whole window is shifted up by (ledge - 2.5 m). MotionBricks is NOT used
here (no mid-air keyframes; the grab of a ledge > 2.75 m is beyond a jump's reach), the clip only reuses source pack poses, rigidly raised.

python -I trav_top.py            -> asm/TO/<var>.{world,soma}.npz + .sel.json for ledge heights 3.0 / 3.5 m, both foot variants
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import trav_lib as T  # noqa: E402

LEDGE_SRC = 2.5  # m: ledge height of the source clip (attach / root z = 250 cm, standing ankles 259 cm)


def main():
    out = os.path.join(T.WORK, 'asm', 'TO')
    scn = os.path.join(T.WORK, 'scn', 'TO')
    os.makedirs(out, exist_ok=True)
    os.makedirs(scn, exist_ok=True)
    for side in ('Lfoot', 'Rfoot'):
        src = 'M_Neutral_Traversal_Climb_Start_2_5_stand_F_' + side
        for L in (3.0, 3.5):
            vid = '%s_L%d' % (side[0], round(L * 100))
            dz = (L - LEDGE_SRC) * 100.0
            a, b = 20, 92
            sc = T.build_scene([dict(kind='piece', src=src, a=a, b=b, ref=a, xy=(0, 0), dz=dz, tag='A')])
            T.save_scene(sc, os.path.join(scn, vid + '.dump.npz'))
            T.to_soma(os.path.join(scn, vid + '.dump.npz'), os.path.join(out, vid + '.soma.npz'))
            np.savez(os.path.join(out, vid + '.world.npz'), W=sc['W'], names=np.array(sc['names']), parents=sc['parents'], fps=30)
            n = len(sc['W'])
            json.dump(dict(id=vid, family='top_out', gaps=[], spans=sc['spans'], trim=[0, n - 1], chosen=[], candidates=[],
                           ledge_height_m=L, source_ledge_m=LEDGE_SRC, shift_cm=dz, src=src, frames_src=[a, b],
                           note='GASP pull-up rigidly raised; hands on the ledge from the first frame (entry = hang, MM/inertialization hands in); attach z = ledge height as in the source clip',
                           obst=None), open(os.path.join(out, vid + '.sel.json'), 'w'), indent=1)
            print('TOPOUT', vid, 'frames', n, 'shift %.0f cm' % dz)


if __name__ == '__main__':
    main()
