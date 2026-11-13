"""Shared code of the locomotion comparison films (2026-11-09).  Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.

Imports the melee tools (Source\\tools\\melee: npc_build / npc_retarget / melee_common) WITHOUT editing them: player rig (152 joints,
first 73 = the game rig of the v2 database), FK, skinning (npc_retarget.skin_W), base anatomy gate.
Right side (UTPP): the dump of the replay copy (loco_replay_build.py / loco_replay_run.py): per 60 Hz frame the FootLocker output pose
  (game-local transforms of the 73 database bones, Root / Trajectory identity) placed at the visual root (visX, visY, visYaw), i.e. the
  DLL's character frame of the leg tail.  World = [Rz(visYaw) | (visX, visY, 0)] @ FK(pose)  (integ yaw: forward(yaw) = (-sin, cos)).
Left side (game): the Female V player_locomotion.anims of the game (2.5, Extracted\\game\\player-anims\\raw\\...\\female\\body), exported to
  glb WITHOUT root motion (Build\\work\\loco-films-20261109\\glb, WolvenKit, AnimationExportArgs__incRootMotion=false: the root-motion
  export of this file fails with a duplicate key), see loco_game.py.
Axes: game (z up, the character faces +Y, +X = her right), metres.
"""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'melee'))
import npc_build as NB  # noqa: E402  (melee tools, unchanged: rig P, gate GB / RG / AL, fk, qslerp, sstep)

NR, MC, P = NB.NR, NB.MC, NB.P
WORK = os.path.join(ROOT, 'Build', 'work', 'loco-films-20261109')
TMP = os.path.join(WORK, '_tmp')
GLB = os.path.join(WORK, 'glb', 'player_locomotion.anims.glb')
RAW_ANIMS = os.path.join(ROOT, 'Extracted', 'game', 'player-anims', 'raw', 'base', 'animations', 'player', 'female', 'body',
                         'player_locomotion.anims')
MEDIA_MP4 = os.path.join(ROOT, 'Media', 'mp4', '2026-11-09_loco-films')
MEDIA_PNG = os.path.join(ROOT, 'Media', 'png', '2026-11-09_loco-films')
FPS = 60.0


def load_dump(path):
    """replay dump -> dict(names, parents, meta [list of dict], lt [T,N,3], lr [T,N,4], toe [T,2,2] FootLocker world xy, toe_bone [2],
    contact [T,2])."""
    names, parents, meta, poses, toes, cont, pre = [], [], [], [], [], [], []
    toe_bone = [-1, -1]
    cur_t, cur_c = None, None
    for ln in open(path, encoding='utf-8'):
        k = ln[:2]
        if k == 'B ':
            _, i, n, p = ln.split()
            names.append(n)
            parents.append(int(p))
        elif k == 'F ':
            s = ln.split()
            meta.append(dict(feed=s[1], i=int(s[2]), t=float(s[3]), cap=(float(s[4]), float(s[5])), actor_yaw=float(s[6]),
                             vis=(float(s[7]), float(s[8])), vis_yaw=float(s[9]), clip=s[10], clip_t=float(s[11]), stance=int(s[12]),
                             gait=int(s[13]), mode=int(s[14]), jumped=int(s[15]), db=int(s[16])))
            cur_t, cur_c = [[np.nan, np.nan], [np.nan, np.nan]], [0, 0]
            toes.append(cur_t)
            cont.append(cur_c)
        elif k == 'P ':
            poses.append(np.array(ln.split()[1:], float).reshape(-1, 7))
        elif k == 'R ':
            pre.append(np.array(ln.split()[1:], float).reshape(-1, 7))
        elif k == 'T ':
            _, s, lb, x, y, c = ln.split()
            s = int(s)
            toe_bone[s] = int(lb)
            cur_t[s] = [float(x), float(y)]
            cur_c[s] = int(c)
    A = np.stack(poses)
    assert len(A) == len(meta), (len(A), len(meta))
    out = dict(names=names, parents=np.array(parents), meta=meta, lt=A[:, :, :3], lr=A[:, :, 3:], toe=np.array(toes, float),
               toe_bone=toe_bone, contact=np.array(cont, int))
    if pre:                                    # R lines: the FootLocker input pose (pre leg tail), same placement
        B = np.stack(pre)
        assert len(B) == len(A)
        out['pre_lt'], out['pre_lr'] = B[:, :, :3], B[:, :, 3:]
    return out


def yaw_mat(yaw):
    """[T] yaw (rad, integ convention) -> [T,4,4] rotation about +Z."""
    c, s = np.cos(yaw), np.sin(yaw)
    M = np.zeros((len(yaw), 4, 4))
    M[:, 0, 0], M[:, 0, 1], M[:, 1, 0], M[:, 1, 1] = c, -s, s, c
    M[:, 2, 2] = M[:, 3, 3] = 1.0
    return M


def place(Wm, x, y, yaw):
    """model-space matrices [T,N,4,4] -> world: [Rz(yaw) | (x, y, 0)] @ Wm."""
    M = yaw_mat(np.asarray(yaw, float))
    M[:, 0, 3], M[:, 1, 3] = x, y
    return np.einsum('tij,tnjk->tnik', M, Wm)


def utpp_world(d, pre=False):
    """dump -> world matrices [T,73,4,4] of the 73 game-rig bones (names checked against the melee player rig); pre=True: the
    FootLocker input pose (R lines) at the same placement."""
    assert d['names'] == list(P.names[:len(d['names'])]), 'database bone names differ from the player rig'
    lt, lr = (d['pre_lt'].copy(), d['pre_lr'].copy()) if pre else (d['lt'].copy(), d['lr'].copy())
    lt[:, :2] = 0.0
    lr[:, :2] = [0, 0, 0, 1.0]
    Wm = MC.fk_world(d['parents'], lt, lr)
    vx = np.array([m['vis'][0] for m in d['meta']])
    vy = np.array([m['vis'][1] for m in d['meta']])
    vyaw = np.array([m['vis_yaw'] for m in d['meta']])
    return place(Wm, vx, vy, vyaw)


def placement_check(d, W):
    """max xy distance (m) between the FK world position of each foot's lock bone and the FootLocker's own world position (T lines)."""
    out = []
    for s in (0, 1):
        b = d['toe_bone'][s]
        dd = np.linalg.norm(W[:, b, :2, 3] - d['toe'][:, s], axis=1)
        out.append(float(np.nanmax(dd)))
    return out


def cam_path(x, y, follow):
    """smoothed (moving average over `follow` frames, edge padded) capsule path [T,2] shared by both film sides."""
    xy = np.stack([x, y], 1)
    w = max(1, int(follow))
    pad = np.pad(xy, ((w // 2, w - 1 - w // 2), (0, 0)), mode='edge')
    return np.stack([np.convolve(pad[:, a], np.ones(w) / w, mode='valid') for a in (0, 1)], 1)


def qslerp(a, b, w):
    """per-element slerp of xyzw quaternions a, b [..., 4] with weight w [...] (shortest arc)."""
    a = np.asarray(a, float)
    b = np.asarray(b, float).copy()
    d = np.sum(a * b, -1)
    b[d < 0] *= -1
    d = np.abs(d)
    w = np.asarray(w, float)
    th = np.arccos(np.clip(d, -1, 1))
    s = np.sin(th)
    small = s < 1e-6
    ka = np.where(small, 1 - w, np.sin((1 - w) * th) / np.where(small, 1, s))
    kb = np.where(small, w, np.sin(w * th) / np.where(small, 1, s))
    q = ka[..., None] * a + kb[..., None] * b
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def rot_angle_deg(qa, qb):
    """angle (deg) between xyzw quaternions [..., 4]."""
    d = np.abs(np.sum(qa * qb, -1))
    return np.degrees(2 * np.arccos(np.clip(d, 0, 1)))


def world_rot_steps(W):
    """per frame and joint the world rotation change (deg) to the previous frame [T,N] (row 0 = 0)."""
    Rm = W[:, :, :3, :3]
    rel = np.einsum('tnji,tnjk->tnik', Rm[:-1], Rm[1:])
    tr = np.clip((np.trace(rel, axis1=2, axis2=3) - 1) / 2, -1, 1)
    out = np.zeros(W.shape[:2])
    out[1:] = np.degrees(np.arccos(tr))
    return out


def sole_heights(W73):
    """sole height [T,2] (left, right) of world joint matrices of the first 73 joints (melee helper, needs 152 joint positions)."""
    pts = np.zeros((len(W73), len(P.names), 3))
    pts[:, :73] = W73[:, :, :3, 3]
    return NR.sole_heights(P, pts)
