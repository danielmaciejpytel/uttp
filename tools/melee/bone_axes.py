"""Print the local axes of the major bones in the reference (T) pose, in character axes (R = +X right, L = -X, F = +Y forward,
B = -Y back, U = +Z up, D = -Z down), for the 'local' space of keypose_schema.json, plus the direction of the bone (towards its child).

  python -I bone_axes.py [woman|man]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import melee_common as MC  # noqa: E402

BONES = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
         'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'RightUpLeg', 'RightLeg', 'RightFoot']
CHILD = {'Hips': 'Spine', 'Spine': 'Spine1', 'Spine1': 'Spine2', 'Spine2': 'Spine3', 'Spine3': 'Neck', 'Neck': 'Neck1', 'Neck1': 'Head',
         'Head': 'LeftEye', 'LeftShoulder': 'LeftArm', 'LeftArm': 'LeftForeArm', 'LeftForeArm': 'LeftHand', 'LeftHand': 'LeftHandMiddle1',
         'RightShoulder': 'RightArm', 'RightArm': 'RightForeArm', 'RightForeArm': 'RightHand', 'RightHand': 'RightHandMiddle1',
         'LeftUpLeg': 'LeftLeg', 'LeftLeg': 'LeftFoot', 'LeftFoot': 'LeftToeBase', 'RightUpLeg': 'RightLeg', 'RightLeg': 'RightFoot',
         'RightFoot': 'RightToeBase'}


def label(v):
    i = int(np.argmax(np.abs(v)))
    if abs(v[i]) < 0.8:
        return '~' + ''.join(l for l, k in sorted(zip('RFU', range(3)), key=lambda z: -abs(v[z[1]])) if abs(v[k]) > 0.3)
    return ('RL', 'FB', 'UD')[i][0 if v[i] > 0 else 1]


def main(rig):
    import glob
    p = glob.glob(os.path.join(MC.WORK, 'npz', 'pwa_katana' if rig == 'woman' else 'pma_katana', '*.npz'))[0]
    d = MC.load_dump(p)
    ix = MC.idx(d)
    Wr = MC.fk_world(d['parents152'], d['rest_lt'][None], d['rest_lr'][None])[0]
    print('| bone | local +X | local +Y | local +Z | bone points |')
    print('|---|---|---|---|---|')
    for b in BONES:
        R = Wr[ix[b], :3, :3]
        dirc = Wr[ix[CHILD[b]], :3, 3] - Wr[ix[b], :3, 3] if b in CHILD else np.zeros(3)
        dirc = dirc / max(np.linalg.norm(dirc), 1e-9)
        print('| %s | %s | %s | %s | %s |' % (b, label(R[:, 0]), label(R[:, 1]), label(R[:, 2]), label(dirc)))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'woman')
