"""Metrics for a generated span (soma npz) against the held-out ground truth of the same clip.

python motion_metrics.py <full_soma.npz> <tpose.npz> <span.npz> [<span2.npz> ...]
Each span npz carries c, g (the span replaces frames c..g of the full clip).
  mpjpe   mean joint position error on the 30 model joints, cm (span interior, world)
  root    mean Hips position error, cm
  slide   mean horizontal foot speed while the ground-truth foot is planted (cm/s); gt value printed for reference
  jerk    max joint acceleration in the 3 frames around each boundary of the spliced clip, cm/s^2 (gt for reference)
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skeleton_bridge import Bridge  # noqa: E402

J30 = ['Hips', 'Spine1', 'Spine2', 'Chest', 'Neck1', 'Neck2', 'Head', 'LeftShoulder', 'LeftArm', 'LeftForeArm',
       'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'LeftLeg', 'LeftShin', 'LeftFoot',
       'LeftToeBase', 'RightLeg', 'RightShin', 'RightFoot', 'RightToeBase']
FEET = ['LeftFoot', 'LeftToeBase', 'RightFoot', 'RightToeBase']
FPS = 30.0


class FK:
    def __init__(self, tpose):
        t = np.load(tpose)
        self.b = Bridge.__new__(Bridge)
        self.b.snames = [str(x) for x in t['names']]
        self.b.sidx = {n: i for i, n in enumerate(self.b.snames)}
        self.b.spar = np.asarray(t['parents'])
        self.b.spos = np.asarray(t['pos'], dtype=np.float64)

    def __call__(self, G, hips):
        return np.array([self.b.soma_fk(G[t], hips[t]) for t in range(len(G))])

    def idx(self, names):
        return [self.b.sidx[n] for n in names]


def planted(P, fi):
    """ground-truth contact: low (within 4 cm of the clip minimum) and slow (< 30 cm/s horizontally)"""
    h = P[:, fi, 1]
    v = np.zeros(len(P))
    v[1:] = np.linalg.norm(np.diff(P[:, fi][:, [0, 2]], axis=0), axis=1) * FPS
    return (h < h.min() + 0.04) & (v < 0.30), v


def main(a):
    full = np.load(a[0])
    fk = FK(a[1])
    Pfull = fk(full['G'], full['hips'])
    j30, feet = fk.idx(J30), fk.idx(FEET)
    for sp in a[2:]:
        s = np.load(sp)
        c, g = int(s['c']), int(s['g'])
        Ps = fk(s['G'], s['hips'])
        gt = Pfull[c:g + 1]
        inner = slice(1, len(Ps) - 1)
        mpjpe = np.linalg.norm(Ps[inner][:, j30] - gt[inner][:, j30], axis=-1).mean() * 100
        root = np.linalg.norm(Ps[inner, 0] - gt[inner, 0], axis=-1).mean() * 100
        # spliced clip for slide and boundary acceleration
        Pspl = Pfull.copy()
        Pspl[c:g + 1] = Ps
        slides, slides_gt = [], []
        for fi in feet:
            mask, _ = planted(Pfull[c:g + 1], fi)
            _, vs = planted(Pspl[c:g + 1], fi)
            _, vg = planted(Pfull[c:g + 1], fi)
            if mask[1:].any():
                slides.append(vs[1:][mask[1:]].mean())
                slides_gt.append(vg[1:][mask[1:]].mean())
        acc = lambda P, f: np.linalg.norm(P[f + 1] - 2 * P[f] + P[f - 1], axis=-1)[j30].max() * FPS * FPS * 100
        jerk = max(acc(Pspl, f) for f in list(range(c - 1, c + 2)) + list(range(g - 1, g + 2)))
        jerk_gt = max(acc(Pfull, f) for f in list(range(c - 1, c + 2)) + list(range(g - 1, g + 2)))
        print('METRICS %-28s frames %3d | mpjpe %5.1f cm | root %5.1f cm | slide %5.1f cm/s (gt %4.1f) | '
              'boundary acc %6.0f cm/s2 (gt %5.0f)' % (os.path.basename(sp), len(Ps), mpjpe, root,
                                                       np.mean(slides) * 100 if slides else -1,
                                                       np.mean(slides_gt) * 100 if slides_gt else -1, jerk, jerk_gt))


if __name__ == '__main__':
    main(sys.argv[1:])
