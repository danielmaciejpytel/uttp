"""Held-out in-betweening with MotionBricks on a bridged source pack clip (soma npz built with body_tpose_generator.npz).

Context = soma frames c-3..c, target = frames g..g+3 with g = c + 4N - 7 (N tokens -> 4N-6 generated frames that start
at the context's last pose and end at the target's first pose). Writes the generated span (frames c..g) as a soma npz,
plus a naive baseline (per-joint global slerp + linear hips) and the ground truth for the same span.

Run from Tools/ai-motion/motionbricks:
  uv run python <this> <soma_mb.npz> <c> <N> <out_prefix> [--targets 1..4] [--no-correct] [--jitter SEED [--jitter-cm S]]
"""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_check import to_mb_local  # noqa: E402


def mb_rest_rots(sk):
    ref = sk.reference_local_transforms.numpy()
    par = np.asarray(sk.parent_indices.numpy() if hasattr(sk.parent_indices, 'numpy') else sk.parent_indices)
    Rr = [None] * len(ref)
    for j in range(len(ref)):
        q = R.from_quat(ref[j, 3:7]).as_matrix()
        Rr[j] = q if par[j] < 0 else Rr[par[j]] @ q
    return np.array(Rr), par


def mb_local_to_soma(poses, sk):
    """MotionBricks local transforms [T,J,7] -> soma G [T,J,3,3] (T-pose deltas) and hips [T,3]."""
    Rr, par = mb_rest_rots(sk)
    T, J = poses.shape[:2]
    Gw = np.zeros((T, J, 3, 3))
    for j in range(J):
        L = R.from_quat(poses[:, j, 3:7]).as_matrix()
        Gw[:, j] = L if par[j] < 0 else Gw[:, par[j]] @ L
    return Gw @ np.swapaxes(Rr, -1, -2)[None], poses[:, 0, :3].copy()


def main(a):
    import warp as wp
    from motionbricks.io.usd import load_skeleton_from_usd
    from motionbricks.io.utils import SOMA_SKELETON_PRIM_PATH, get_asset_file
    from motionbricks.model.config import MotionBricksInferenceConfig
    from motionbricks.model.soma.contract import SOMA_N1_FP32_ONNX_PATH
    from motionbricks.model.soma.engine import MotionBricksSOMAEngine

    src, c, N, out = a[0], int(a[1]), int(a[2]), a[3]
    ntgt = int(a[a.index('--targets') + 1]) if '--targets' in a else 4
    g = c + 4 * N - 7
    s = np.load(src)
    G, hips = s['G'], s['hips']
    assert c >= 3 and g + ntgt <= len(G), 'span outside the clip (len %d, c %d, g %d)' % (len(G), c, g)
    wp.init()
    wp.set_device('cuda:0')
    sk = load_skeleton_from_usd(get_asset_file('human', 'soma_base_skel_minimal_no_root.usd'), SOMA_SKELETON_PRIM_PATH)
    L = to_mb_local(G, hips, sk).astype(np.float32)
    if '--jitter' in a:  # sample diversity (the model itself is deterministic): seeded noise on the context history c-3..c-1 (frame c, the last context pose, stays exact)
        seed = int(a[a.index('--jitter') + 1])
        sig = float(a[a.index('--jitter-cm') + 1]) / 100.0 if '--jitter-cm' in a else 0.02
        rng = np.random.default_rng(seed)
        for f in range(c - 3, c):
            L[f, 0, :3] += rng.normal(0, sig, 3).astype(np.float32)
            q = R.from_quat(L[f, :, 3:7]) * R.from_rotvec(rng.normal(0, np.radians(1.5), (L.shape[1], 3)))
            L[f, :, 3:7] = q.as_quat().astype(np.float32)
    cfg = MotionBricksInferenceConfig(allowed_token_counts=(N,), correct_generated_diff='--no-correct' not in a)
    with MotionBricksSOMAEngine(SOMA_N1_FP32_ONNX_PATH) as engine, wp.ScopedStream(engine.stream):
        ctx, tgt, outb = engine.create_input(sk), engine.create_input(sk), engine.create_output(sk)
        ctx.update_from_poses(wp.array(L[c - 3:c + 1], dtype=wp.transform), stream=engine.stream)
        tgt.update_from_poses(wp.array(L[g:g + ntgt], dtype=wp.transform), stream=engine.stream)
        engine.generate_animation(ctx, tgt, outb, config=cfg)
        engine.synchronize()
        n = outb.num_frames
        gen = outb.poses[:n].numpy()
        fc = outb.foot_contacts.numpy()[:n] if hasattr(outb, 'foot_contacts') else None
    assert n == g - c + 1, 'expected %d frames, got %d' % (g - c + 1, n)
    Gg, hg = mb_local_to_soma(gen.astype(np.float64), sk)
    # naive baseline: per-joint slerp of global rotations, linear hips, from frame c to frame g
    w = np.linspace(0, 1, n)
    Gb = np.zeros_like(Gg)
    for j in range(G.shape[1]):
        sl = Slerp([0, 1], R.from_matrix(np.stack([G[c, j], G[g, j]])))
        Gb[:, j] = sl(w).as_matrix()
    hb = hips[c][None] * (1 - w)[:, None] + hips[g][None] * w[:, None]
    meta = dict(names=s['names'], k=s['k'], fps=30, c=c, g=g, N=N, src=src)
    np.savez(out + '.gen.npz', G=Gg, hips=hg, foot_contacts=fc if fc is not None else np.zeros(0), **meta)
    np.savez(out + '.base.npz', G=Gb, hips=hb, **meta)
    np.savez(out + '.gt.npz', G=G[c:g + 1], hips=hips[c:g + 1], **meta)
    print('MB_INBETWEEN frames %d (c=%d g=%d N=%d targets=%d) -> %s.{gen,base,gt}.npz' % (n, c, g, N, ntgt, out))


if __name__ == '__main__':
    main(sys.argv[1:])
