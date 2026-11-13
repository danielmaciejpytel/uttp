"""Rig/clip facts needed for the retarget: rest-pose differences of common bones (NPC woman_base vs player rig),
which player-only bones move in the player 'enter' clip and how IK helper bones relate to the limbs."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402
import skel  # noqa: E402
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

ROOT = Path(project_root())
GLB = ROOT / 'Extracted/game/vehicle-entry/glb'


def main():
    npc = skel.Skeleton.from_glb(GLB / 'wa_sport2_passenger_r_entry.anims.glb')
    ply = skel.Skeleton.from_glb(GLB / 'pwa_sport2_driver.anims.glb')
    common = [b for b in npc.names if b in ply.index]
    print('npc bones', len(npc.names), 'player bones', len(ply.names), 'common', len(common))
    worst = []
    for b in common:
        qa = npc.rest_r[npc.index[b]]
        qb = ply.rest_r[ply.index[b]]
        ang = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(qa, qb))))))
        dt = float(np.linalg.norm(npc.rest_t[npc.index[b]] - ply.rest_t[ply.index[b]]))
        worst.append((ang, dt, b))
    worst.sort(reverse=True)
    print('max rest local rotation diff (deg, dt m, bone):', [(round(a, 2), round(d, 4), b) for a, d, b in worst[:10]])
    # player clip: which player-only bones animate
    clip = ply.clip('enter')
    T = np.arange(0, 61) / 30.0
    loc_t, loc_r = ply.sample_local(clip, T)
    only = [b for b in ply.names if b not in npc.index]
    moving = []
    for b in only:
        i = ply.index[b]
        dr = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(loc_r[:, i] * loc_r[:1, i], axis=-1)), 0, 1))).max()
        dtt = np.linalg.norm(loc_t[:, i] - loc_t[:1, i], axis=-1).max()
        dr0 = np.degrees(2 * np.arccos(min(1, abs(float(np.dot(loc_r[0, i], ply.rest_r[i]))))))
        dt0 = float(np.linalg.norm(loc_t[0, i] - ply.rest_t[i]))
        moving.append((b, ply.names[ply.parent[i]] if ply.parent[i] >= 0 else None, round(float(dr), 2), round(float(dtt), 4), round(dr0, 2), round(dt0, 4)))
    print('player-only bones: name, parent, max rot change deg, max trans change m, frame0-vs-rest rot deg, trans m')
    for m in moving:
        print('  ', m)
    # IK helper vs effector world distance
    W_t, W_r = ply.world(loc_t, loc_r)
    pairs = [('Torso_World_LeftLeg_Foot_IK_JNT', 'LeftFoot'), ('Torso_World_RightLeg_Foot_IK_JNT', 'RightFoot'),
             ('leg_left_ik', 'LeftFoot'), ('leg_right_ik', 'RightFoot'),
             ('Torso_World_LeftArm_Hand_IK_JNT', 'LeftHand'), ('Torso_World_RightArm_Hand_IK_JNT', 'RightHand'),
             ('Torso_LeftLeg_Foot_IK_JNT', 'LeftFoot'), ('Torso_COG_Control_JNT', 'Hips')]
    for a, b in pairs:
        if a in ply.index and b in ply.index:
            d = np.linalg.norm(W_t[:, ply.index[a]] - W_t[:, ply.index[b]], axis=-1)
            print('dist', a, b, 'min %.3f max %.3f f0 %.3f' % (d.min(), d.max(), d[0]))


if __name__ == '__main__':
    main()
