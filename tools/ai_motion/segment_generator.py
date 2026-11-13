"""Batch runner of MotionBricks in-betweening for many scenes / gaps / jitter seeds with ONE engine instance.
Same generation call as clip_inbetween.py (imported helpers), only the engine is opened once.

Run from Tools/ai-motion/motionbricks (uv run python <this> <jobs.json>):
  jobs.json = [{"soma": <scene soma npz>, "out": <prefix>, "gaps": [{"c": c, "N": N, "targets": 4}, ...], "seeds": [10, 11, ...],
                "jitter_cm": 2.0}]
Output per job / gap index i / seed s: <out>.g<i>.s<s>.gen.npz (G, hips, foot_contacts, c, g, N, names, k, seed).
Gap spans are independent (context and target frames are real scene frames), so one scene may hold several gaps.
"""
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from clip_inbetween import mb_local_to_soma  # noqa: E402
from scene_check import to_mb_local  # noqa: E402


def main(jobs_path):
    import warp as wp
    from motionbricks.io.usd import load_skeleton_from_usd
    from motionbricks.io.utils import SOMA_SKELETON_PRIM_PATH, get_asset_file
    from motionbricks.model.config import MotionBricksInferenceConfig
    from motionbricks.model.soma.contract import SOMA_N1_FP32_ONNX_PATH
    from motionbricks.model.soma.engine import MotionBricksSOMAEngine

    jobs = json.load(open(jobs_path))
    wp.init()
    wp.set_device('cuda:0')
    sk = load_skeleton_from_usd(get_asset_file('human', 'soma_base_skel_minimal_no_root.usd'), SOMA_SKELETON_PRIM_PATH)
    n_done = 0
    with MotionBricksSOMAEngine(SOMA_N1_FP32_ONNX_PATH) as engine, wp.ScopedStream(engine.stream):
        for job in jobs:
            s = np.load(job['soma'])
            G, hips = s['G'], s['hips']
            L0 = to_mb_local(G, hips, sk).astype(np.float32)
            for gi, gap in enumerate(job['gaps']):
                c, N, ntgt = int(gap['c']), int(gap['N']), int(gap.get('targets', 4))
                g = c + 4 * N - 7
                assert c >= 3 and g + ntgt <= len(G), 'span outside the scene (len %d, c %d, g %d)' % (len(G), c, g)
                for seed in job['seeds']:
                    L = L0.copy()
                    if seed >= 0:  # seeded noise on the context history c-3..c-1 only (frame c stays exact)
                        rng = np.random.default_rng(int(seed) * 1000 + gi)
                        sig = float(job.get('jitter_cm', 2.0)) / 100.0
                        for f in range(c - 3, c):
                            L[f, 0, :3] += rng.normal(0, sig, 3).astype(np.float32)
                            q = R.from_quat(L[f, :, 3:7]) * R.from_rotvec(rng.normal(0, np.radians(1.5), (L.shape[1], 3)))
                            L[f, :, 3:7] = q.as_quat().astype(np.float32)
                    cfg = MotionBricksInferenceConfig(allowed_token_counts=(N,), correct_generated_diff=True)
                    ctx, tgt, outb = engine.create_input(sk), engine.create_input(sk), engine.create_output(sk)
                    ctx.update_from_poses(wp.array(L[c - 3:c + 1], dtype=wp.transform), stream=engine.stream)
                    tgt.update_from_poses(wp.array(L[g:g + ntgt], dtype=wp.transform), stream=engine.stream)
                    engine.generate_animation(ctx, tgt, outb, config=cfg)
                    engine.synchronize()
                    n = outb.num_frames
                    gen = outb.poses[:n].numpy()
                    fc = outb.foot_contacts.numpy()[:n] if hasattr(outb, 'foot_contacts') else np.zeros((n, 4))
                    assert n == g - c + 1, 'expected %d frames, got %d' % (g - c + 1, n)
                    Gg, hg = mb_local_to_soma(gen.astype(np.float64), sk)
                    np.savez('%s.g%d.s%d.gen.npz' % (job['out'], gi, seed), G=Gg, hips=hg, foot_contacts=fc, names=s['names'], k=s['k'],
                             fps=30, c=c, g=g, N=N, seed=seed)
                    n_done += 1
    print('TRAV_MB done %d spans' % n_done)


if __name__ == '__main__':
    main(sys.argv[1])
