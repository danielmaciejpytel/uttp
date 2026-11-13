"""G0 checks for a soma npz (from skeleton_bridge.py fwd).

  anat <soma.npz> <dump.npz>       SOMA joint offsets from Hips vs source pack offsets from pelvis (axes + k), per joint, cm
  kimodo <soma.npz>                 FK through Kimodo's own SOMASkeleton77 (locals = parent^T G) vs soma npz pos
  mb <soma.npz>                     FK through MotionBricks' skeleton (local transforms) vs soma npz pos
"""
import sys

import numpy as np

CM = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])
PAIRS = [('Head', 'head'), ('LeftArm', 'upperarm_l'), ('LeftForeArm', 'lowerarm_l'), ('LeftHand', 'hand_l'),
         ('RightHand', 'hand_r'), ('LeftShin', 'calf_l'), ('LeftFoot', 'foot_l'), ('LeftToeBase', 'ball_l'),
         ('RightFoot', 'foot_r'), ('RightToeBase', 'ball_r')]


def anat(sp, dp):
    s, d = np.load(sp), np.load(dp)
    sn, un = [str(x) for x in s['names']], [str(x) for x in d['names']]
    k = float(s['k'])
    for a, b in PAIRS:
        vs = s['pos'][:, sn.index(a)] - s['pos'][:, sn.index('Hips')]
        vu = (CM @ (d['W'][:, un.index(b), :3, 3] - d['W'][:, un.index('pelvis'), :3, 3]).T).T / 100.0 * k
        e = np.linalg.norm(vs - vu, axis=1) * 100
        print('ANAT %-12s vs %-11s mean %5.1f max %5.1f cm (offset len soma %.1f cm)' % (
            a, b, e.mean(), e.max(), np.linalg.norm(vs, axis=1).mean() * 100))


def local_from_global(G, parents):
    L = G.copy()
    for j, p in enumerate(parents):
        if p >= 0:
            L[:, j] = np.swapaxes(G[:, p], -1, -2) @ G[:, j]
    return L


def kimodo(sp):
    import torch
    from kimodo.skeleton.definitions import SOMASkeleton77
    s = np.load(sp)
    sk = SOMASkeleton77()
    names = [str(x) for x in s['names']]
    assert names == list(sk.bone_order_names), 'joint order differs'
    L = local_from_global(s['G'], sk.joint_parents.numpy())
    _, P, _ = sk.fk(torch.tensor(L, dtype=torch.float32), torch.tensor(s['hips'], dtype=torch.float32))
    e = np.linalg.norm(P.numpy() - s['pos'], axis=-1) * 100
    print('KIMODO_FK max %.2f cm mean %.3f cm (soma npz must be built with body_tpose_constraint.npz)' % (e.max(), e.mean()))


def mb(sp):
    import warp as wp
    from scipy.spatial.transform import Rotation as R
    from motionbricks.io.usd import load_skeleton_from_usd
    from motionbricks.io.utils import SOMA_SKELETON_PRIM_PATH, get_asset_file
    wp.init()
    s = np.load(sp)
    sk = load_skeleton_from_usd(get_asset_file("human", "soma_base_skel_minimal_no_root.usd"), SOMA_SKELETON_PRIM_PATH)
    poses = to_mb_local(s['G'], s['hips'], sk)
    # FK via the library's own world transform computation
    out = []
    for t in range(len(poses)):
        loc = wp.array(poses[t].astype(np.float32), dtype=wp.transform, device='cuda:0')
        wt = wp.empty_like(loc)
        sk.compute_world_transforms(loc, wt)
        wp.synchronize()
        out.append(wt.numpy()[:, :3])
    P = np.array(out)
    e = np.linalg.norm(P - s['pos'], axis=-1) * 100
    print('MB_FK max %.2f cm mean %.3f cm (soma npz must be built with body_tpose_generator.npz)' % (e.max(), e.mean()))


def to_mb_local(G, hips, sk):
    """soma G (T-pose deltas) -> MotionBricks local transforms [T,J,7] (p xyz, q xyzw), reference offsets kept."""
    from scipy.spatial.transform import Rotation as R
    ref = sk.reference_local_transforms.numpy()
    par = sk.parent_indices.numpy() if hasattr(sk.parent_indices, 'numpy') else np.array(sk.parent_indices)
    J = len(ref)
    Rrest = [None] * J
    for j in range(J):
        q = R.from_quat(ref[j, 3:7]).as_matrix()
        Rrest[j] = q if par[j] < 0 else Rrest[par[j]] @ q
    Rrest = np.array(Rrest)
    Gw = G @ Rrest[None]  # world rotation of each MB joint frame
    out = np.zeros((len(G), J, 7))
    for j in range(J):
        p = par[j]
        Lr = Gw[:, j] if p < 0 else np.swapaxes(Gw[:, p], -1, -2) @ Gw[:, j]
        out[:, j, 3:7] = R.from_matrix(Lr).as_quat()
        out[:, j, :3] = hips if p < 0 else ref[j, :3]
    return out


if __name__ == '__main__':
    a = sys.argv[1:]
    {'anat': lambda: anat(a[1], a[2]), 'kimodo': lambda: kimodo(a[1]), 'mb': lambda: mb(a[1])}[a[0]]()
