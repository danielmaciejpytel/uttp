"""Hinge-first two-bone IK for the melee key-pose corrections (arms and legs of the V rig, local axes of bone_axes.py).

The shared ik.two_bone (vehicle entry tools) turns the middle bone with a minimal swing: the elbow / knee gets a sideways deviation and a
roll, the end bone keeps its WORLD rotation (wrist flexes 100+ deg) and a straight leg has no defined bend side (knees bent backwards).
This solver keeps the joints anatomical:
  1. elbow / knee: flexion about the local Z hinge only (law of cosines, always >= 0 = forward / the natural side), deviation about
     local Y forced to 0, the roll about the bone axis (local X) of the source pose is kept;
  2. the parent bone (shoulder joint / hip) swings the whole chain so the end reaches the target (minimal swing), then twists about the
     parent->target axis so the elbow stays on the side where the source elbow was (pole = source middle joint; skipped when the source
     limb is nearly straight);
  3. the end bone (hand / foot): mode 'local' keeps its local rotation (wrist stays as in the source), 'world' keeps its world
     rotation (planted foot).
No-op when the effector is already within NOOP_M of the target (so a zero correction reproduces the source exactly).
Returns the residual distance (m): > 0 when the target is out of reach (the chain is then fully extended towards it).
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

NOOP_M = 0.002


def _rot_between(a, b):
    a = a / max(np.linalg.norm(a), 1e-9)
    b = b / max(np.linalg.norm(b), 1e-9)
    c = float(np.dot(a, b))
    if c > 1 - 1e-9:
        return np.eye(3)
    if c < -1 + 1e-9:
        ax = np.cross(a, [1.0, 0, 0])
        if np.linalg.norm(ax) < 1e-6:
            ax = np.cross(a, [0, 1.0, 0])
        return R.from_rotvec(ax / np.linalg.norm(ax) * np.pi).as_matrix()
    ax = np.cross(a, b)
    return R.from_rotvec(ax / np.linalg.norm(ax) * np.arccos(c)).as_matrix()


def solve(fk, parents, rest_lr, lt, lr, a, b, c, target, weight=1.0, end_mode='local', pole=True):
    """In-place on lt / lr ([F,N,3] / [F,N,4]); fk(lt, lr) -> world matrices [F,N,4,4]. target [F,3]; weight scalar or [F]."""
    F = lt.shape[0]
    w = np.broadcast_to(np.asarray(weight, float), (F,))
    W0 = fk(lt, lr)
    res = np.zeros(F)
    pa, pb = parents[a], parents[b]
    Lb_rest = R.from_quat(rest_lr[b])
    for f in range(F):
        A, B, C = W0[f, a, :3, 3], W0[f, b, :3, 3], W0[f, c, :3, 3]
        tgt = C + (np.asarray(target[f], float) - C) * w[f]
        if np.linalg.norm(tgt - C) < NOOP_M:
            continue
        la, lb = np.linalg.norm(B - A), np.linalg.norm(C - B)
        dv = tgt - A
        d = float(np.clip(np.linalg.norm(dv), abs(la - lb) + 1e-4, la + lb - 1e-4))
        u = dv / max(np.linalg.norm(dv), 1e-9)
        cos_b = np.clip((la * la + lb * lb - d * d) / (2 * la * lb), -1, 1)
        flex = np.pi - np.arccos(cos_b)                         # 0 = straight
        # --- 1 hinge: rel = Rz(-flex) Rx(roll) (ZYX Euler of the rest-relative local rotation, deviation y = 0)
        Lb = R.from_quat(lr[f, b])
        z, y, x = (Lb_rest.inv() * Lb).as_euler('ZYX')
        newb = Lb_rest * R.from_euler('ZYX', [-flex, 0.0, x])
        # a clean hinge model needs the original chain to be (nearly) a hinge too; the deviation y is dropped (that is the point)
        lr[f, b] = newb.as_quat()
        W1 = fk(lt[f:f + 1], lr[f:f + 1])[0]
        B1, C1 = W1[b, :3, 3], W1[c, :3, 3]
        # --- 2 swing the parent so that A->C' points at the target
        Rsw = _rot_between(C1 - A, u)
        Ra = W1[a, :3, :3]
        Ra2 = Rsw @ Ra
        # pole: turn about u so the middle joint lies on the source side
        if pole:
            Bs = Rsw @ (B1 - A)
            vcur = Bs - np.dot(Bs, u) * u
            vsrc = (B - A) - np.dot(B - A, u) * u
            if np.linalg.norm(vsrc) > 0.15 * la and np.linalg.norm(vcur) > 1e-6:
                vcur /= np.linalg.norm(vcur)
                vsrc /= np.linalg.norm(vsrc)
                ang = np.arctan2(np.dot(np.cross(vcur, vsrc), u), np.dot(vcur, vsrc))
                Rt = R.from_rotvec(u * ang).as_matrix()
                Ra2 = Rt @ Ra2
        Rp = W1[pa, :3, :3] if pa >= 0 else np.eye(3)
        lr[f, a] = R.from_matrix(Rp.T @ Ra2).as_quat()
        if end_mode == 'world':
            W2 = fk(lt[f:f + 1], lr[f:f + 1])[0]
            lr[f, c] = R.from_matrix(W2[b, :3, :3].T @ W0[f, c, :3, :3]).as_quat()
        W3 = fk(lt[f:f + 1], lr[f:f + 1])[0]
        res[f] = np.linalg.norm(W3[c, :3, 3] - tgt)
    return float(res.max())
