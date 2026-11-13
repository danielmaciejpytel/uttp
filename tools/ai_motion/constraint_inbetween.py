"""Kimodo held-out in-betweening on constraints from constraint_builder.py (source env.sh first: text encoder on CPU).

python constraint_inbetween.py <case_dir> "<prompt>" [--samples 3] [--steps 100] [--seed 1] [--no-post] [--tag name]
case_dir holds constraints.json + gt_canonical.npz. Writes <case_dir>/<tag>_<i>.npz as soma npz spans (frames c..g of the
canonical clip, same keys as clip_inbetween.py) so motion_metrics.py can score them against gt_canonical.npz.
Checks that the model's global_rot_mats are T-pose deltas (bridge FK must reproduce posed_joints).
"""
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(a):
    import torch
    from kimodo.constraints import load_constraints_lst
    from kimodo.model.load_model import load_model
    from skeleton_bridge import Bridge

    case, prompt = a[0], a[1]
    opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    ns, steps, seed, tag = opt('--samples', 3), opt('--steps', 100), opt('--seed', 1), opt('--tag', 'kimodo')
    gt = np.load(os.path.join(case, 'gt_canonical.npz'))
    c, g = int(gt['c']), int(gt['g'])
    con = json.load(open(os.path.join(case, 'constraints.json')))
    nf = max(con[0]['frame_indices']) + 1
    t0 = time.time()
    model = load_model('kimodo-soma-rp', device='cuda:0')
    print('model loaded %.0f s' % (time.time() - t0))
    cl = load_constraints_lst(os.path.join(case, 'constraints.json'), model.skeleton, device='cuda:0')
    torch.manual_seed(seed)
    t1 = time.time()
    out = model(prompt, nf, constraint_lst=cl, num_denoising_steps=steps, num_samples=ns,
                post_processing='--no-post' not in a, return_numpy=True)
    print('generated %d x %d frames in %.0f s' % (ns, nf, time.time() - t1))
    t = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'body_tpose_constraint.npz'))
    br = Bridge.__new__(Bridge)
    br.snames = [str(x) for x in t['names']]; br.sidx = {n: i for i, n in enumerate(br.snames)}
    br.spar = np.asarray(t['parents']); br.spos = np.asarray(t['pos'], dtype=np.float64)
    hi = br.sidx['Hips']
    for i in range(ns):
        G = out['global_rot_mats'][i].astype(np.float64)
        P = out['posed_joints'][i].astype(np.float64)
        hips = P[:, hi]
        fkerr = max(np.linalg.norm(br.soma_fk(G[f], hips[f]) - P[f], axis=-1).max() for f in range(len(G))) * 100
        path = os.path.join(case, '%s_%d.npz' % (tag, i))
        np.savez(path, G=G[c:g + 1], hips=hips[c:g + 1], names=gt['names'], k=gt['k'], fps=30, c=c, g=g,
                 foot_contacts=out.get('foot_contacts', np.zeros(0))[i] if 'foot_contacts' in out else np.zeros(0),
                 prompt=prompt)
        print('KIMODO_SAMPLE %s | FK consistency %.2f cm' % (path, fkerr))


if __name__ == '__main__':
    main(sys.argv[1:])
