"""MotionBricks smoke test without NVIDIA sample clips (evaluation-only licence).

Context = 4x SOMA reference pose, target = same pose moved 1.5 m along +X (model space, Y up).
Checks the TensorRT build/run path on this GPU and saves the generated poses to an .npz.
Run from Tools/ai-motion/motionbricks:  uv run python <this file> --out <file.npz>
"""
import argparse
import time

import numpy as np
import warp as wp

from motionbricks.io.usd import load_skeleton_from_usd
from motionbricks.io.utils import SOMA_SKELETON_PRIM_PATH, get_asset_file
from motionbricks.model.contract import MODEL_POSES_PER_TOKEN, MODEL_SAMPLE_RATE
from motionbricks.model.soma.contract import SOMA_N1_FP32_ONNX_PATH
from motionbricks.model.soma.engine import MotionBricksSOMAEngine


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--dist", type=float, default=1.5)
    a = ap.parse_args()
    wp.init()
    wp.set_device("cuda:0")
    skel = load_skeleton_from_usd(get_asset_file("human", "soma_base_skel_minimal_no_root.usd"), SOMA_SKELETON_PRIM_PATH)
    ref = skel.reference_local_transforms.numpy()
    print("joints", ref.shape, "root", ref[0].round(3))
    t0 = time.time()
    with MotionBricksSOMAEngine(SOMA_N1_FP32_ONNX_PATH) as engine, wp.ScopedStream(engine.stream):
        print("engine ready in %.1f s" % (time.time() - t0))
        ctx, tgt, out = engine.create_input(skel), engine.create_input(skel), engine.create_output(skel)
        c = np.repeat(ref[None], MODEL_POSES_PER_TOKEN, axis=0)
        g = c.copy()
        g[:, 0, 0] += a.dist
        ctx.update_from_poses(wp.array(c, dtype=wp.transform), stream=engine.stream)
        tgt.update_from_poses(wp.array(g, dtype=wp.transform), stream=engine.stream)
        t1 = time.time()
        engine.generate_animation(ctx, tgt, out)
        engine.synchronize()
        n = out.num_frames
        poses = out.poses[:n].numpy()
        root = out.root_transforms()[:n, :3]
        print("generated %d frames (%.2f s) in %.3f s; root %s -> %s" % (
            n, n / MODEL_SAMPLE_RATE, time.time() - t1, root[0].round(3), root[-1].round(3)))
        np.savez(a.out, poses=poses, joint_names=np.array(skel.joint_names if hasattr(skel, "joint_names") else []))
        print("SMOKE_OK", a.out)


if __name__ == "__main__":
    main()
