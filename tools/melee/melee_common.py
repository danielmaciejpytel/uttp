"""Shared helpers of the melee animation analysis tools (numpy / scipy only; run with
Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I).

Work area: Build\\work\\melee-anim-20261107 (WORK).  Dump npz (one per clip, npz\\<set>\\<clip>.npz), written by melee_dump.py:
  W        [T,73,4,4]  world matrices of the first 73 rig joints, GAME axes (z up, character faces +Y, left = -X), metres,
                       root motion included.  Same layout as game_skeleton_bridge.save_dump, so `game_skeleton_bridge.py fwd` accepts it.
  names    [73]  parents [73]  rest [73,4,4] (bind world matrices of the rig, game axes)  fps (=30)
  W_extra  [T,79,4,4]  the remaining joints 73..151 (IK helpers, Torso_fppCamera_*, arms_* spaces), game axes
  names_extra [79]
  lt [T,152,3], lr [T,152,4 xyzw]  LOCAL transforms of all 152 joints in game axes (L_game = C^-1 L_gltf C), fk of these = W
  names152, parents152, rest_lt [152,3], rest_lr [152,4]  (reference pose locals of the clip rig)
  meta     json string: set, clip, rig ('woman'|'man'), animation_type ('Normal'|'Additive'|'AdditiveWithoutFirstFrame'),
           additive (bool), root_motion_type, frames, duration, events (list of {name,start,duration} when present)
"""
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
WORK = os.path.join(ROOT, 'Build', 'work', 'melee-anim-20261107')
PY = os.path.join(ROOT, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
SETS = ['default_melee_weapon', 'pma_default_melee_weapon', 'pwa_fists', 'pma_fists', 'pwa_katana', 'pma_katana',
        'pwa_knife', 'pma_knife', 'pwa_one_handed_blunt', 'pma_one_handed_blunt', 'pwa_two_handed_blunt',
        'pma_two_handed_blunt']
RIG_OF_SET = {s: ('man' if s.startswith('pma_') else 'woman') for s in SETS}  # default_melee_weapon = woman rig (checked in the WolvenKit log)
FPS = 30.0

# game <-> glTF axes (C: game Z up -> glTF Y up), as in game_skeleton_bridge / skin_clip
C = np.eye(4)
C[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
CI = np.linalg.inv(C)

# bone groups of the 73-joint player rig (names as in the rig)
L_ARM = ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand']
R_ARM = ['RightShoulder', 'RightArm', 'RightForeArm', 'RightHand']
L_LEG = ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase']
R_LEG = ['RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase']
SPINE = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3']
HEADNECK = ['Neck', 'Neck1', 'Head']
FINGERS_L = [n for n in ('LeftHandThumb1 LeftHandIndex1 LeftHandMiddle1 LeftHandRing1 LeftHandPinky1 LeftHandThumb2 '
                         'LeftHandIndex2 LeftHandMiddle2 LeftHandRing2 LeftHandPinky2 LeftHandIndex3 LeftHandMiddle3 '
                         'LeftHandRing3 LeftHandPinky3').split()]
FINGERS_R = [n.replace('Left', 'Right') for n in FINGERS_L]
GROUPS = {'spine': SPINE, 'head_neck': HEADNECK, 'left_arm': L_ARM, 'right_arm': R_ARM, 'left_leg': L_LEG,
          'right_leg': R_LEG, 'fingers_left': FINGERS_L, 'fingers_right': FINGERS_R}
FPP_BONES = ['Torso_fppCamera_Control_GRP', 'Torso_fppCamera_Aim_JNT', 'Torso_fppCamera_Target_JNT',
             'Torso_fppCamera_UpOffset_GRP', 'Torso_fppCamera_Up_GRP']
LOWER_GROUPS = ('left_leg', 'right_leg')

# human joint limits used as flags (degrees); soft limits, not anatomy-exact
LIMITS = {'elbow_flex': (0.0, 150.0), 'knee_flex': (0.0, 150.0), 'wrist_bend': (-70.0, 80.0),
          'shoulder_elev': (0.0, 180.0), 'spine_total_bend': (0.0, 60.0), 'neck_bend': (0.0, 50.0)}


def clip_path(set_name, clip, base=None):
    return os.path.join(base or os.path.join(WORK, 'npz'), set_name, clip + '.npz')


def load_dump(path):
    z = np.load(path, allow_pickle=False)
    d = {k: z[k] for k in z.files}
    d['meta'] = json.loads(str(d['meta']))
    d['names'] = [str(x) for x in d['names']]
    d['names152'] = [str(x) for x in d['names152']]
    return d


def idx(d):
    return {n: i for i, n in enumerate(d['names152'])}


def mat4(t, q):
    m = np.broadcast_to(np.eye(4), t.shape[:-1] + (4, 4)).copy()
    m[..., :3, :3] = R.from_quat(q.reshape(-1, 4)).as_matrix().reshape(t.shape[:-1] + (3, 3))
    m[..., :3, 3] = t
    return m


def fk_world(parents, lt, lr):
    """lt [T,N,3], lr [T,N,4 xyzw] -> world matrices [T,N,4,4] (parents before children)."""
    T, N = lt.shape[:2]
    L = mat4(lt, lr)
    W = np.empty_like(L)
    for i in range(N):
        p = int(parents[i])
        W[:, i] = L[:, i] if p < 0 else W[:, p] @ L[:, i]
    return W


def world_all(d):
    """World matrices [T,152,4,4] of every joint from the dump (W + W_extra, original joint order)."""
    T = d['W'].shape[0]
    out = np.empty((T, 152, 4, 4))
    names = d['names152']
    ix = {n: i for i, n in enumerate(names)}
    for j, n in enumerate(d['names']):
        out[:, ix[n]] = d['W'][:, j]
    for j, n in enumerate(d['names_extra']):
        out[:, ix[str(n)]] = d['W_extra'][:, j]
    return out


def heading_frame(P_left_up_leg, P_right_up_leg):
    """Hips-heading frame [T,3,3], columns = (right, forward, up) in game axes: right = LeftUpLeg->RightUpLeg projected to the
    ground, up = +Z, forward = up x right. (The rig's hips bone axes are NOT aligned with the character, so positions are used.)"""
    r = np.array(P_right_up_leg, dtype=float) - np.array(P_left_up_leg, dtype=float)
    r[..., 2] = 0.0
    r /= np.maximum(np.linalg.norm(r, axis=-1, keepdims=True), 1e-9)
    up = np.broadcast_to(np.array([0.0, 0.0, 1.0]), r.shape)
    f = np.cross(up, r)
    return np.stack([r, f, up], axis=-1)


def euler_xyz(deg):
    return R.from_euler('xyz', np.asarray(deg, dtype=float), degrees=True).as_matrix()


def angle_between(a, b):
    a = a / np.maximum(np.linalg.norm(a, axis=-1, keepdims=True), 1e-9)
    b = b / np.maximum(np.linalg.norm(b, axis=-1, keepdims=True), 1e-9)
    return np.degrees(np.arccos(np.clip(np.sum(a * b, axis=-1), -1, 1)))


def rot_delta_deg(Ra, Rb):
    """Angle (deg) of Ra^T Rb, broadcast over leading dims."""
    M = np.swapaxes(Ra, -1, -2) @ Rb
    tr = np.trace(M, axis1=-2, axis2=-1)
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))


def jdump(obj, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, indent=1, ensure_ascii=False, default=lambda o: o.item() if hasattr(o, 'item') else str(o))
