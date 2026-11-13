"""Plausibility score for a generated transition (no ground truth): the span spliced into the composed clip.

python -I transition_score.py <composite_soma.npz> <tpose.npz> <span.npz> [<span2.npz> ...]
  bnd   max joint acceleration over the 30 model joints in the 3 frames around each span boundary, m/s^2
        (ref = the same statistic over the source frames outside the span: the clips' own motion)
  slide mean horizontal speed of the contact joint while the foot is on the ground inside the span, cm/s
        (foot_pin.contacts; ground level from the composite)
  hipv  max vertical hips acceleration inside the span, m/s^2
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foot_pin import SIDES, Skel, contacts  # noqa: E402
from motion_metrics import J30  # noqa: E402


def main(a):
    comp = np.load(a[0])
    sk = Skel(a[1])
    Pc = np.array([sk.fk(comp['G'][t], comp['hips'][t]) for t in range(len(comp['G']))])
    ground = Pc[:, :, 1].min(0)
    j30 = [sk.idx[n] for n in J30]
    for sp in a[2:]:
        s = np.load(sp)
        c, g = int(s['c']), int(s['g'])
        P = Pc.copy()
        P[c:g + 1] = np.array([sk.fk(s['G'][t], s['hips'][t]) for t in range(len(s['G']))])
        acc = np.linalg.norm(P[2:] - 2 * P[1:-1] + P[:-2], axis=-1)[:, j30].max(1) * 900  # index f -> frame f+1
        bnd = max(acc[f - 1] for f in list(range(c - 1, c + 2)) + list(range(g - 1, g + 2)) if 1 <= f < len(P) - 1)
        outside = np.r_[acc[:max(0, c - 3)], acc[g + 2:]]
        ref = np.percentile(outside, 99) if len(outside) else -1
        Ps = P[c:g + 1]
        sl = []
        for side in SIDES:
            m, piv = contacts(Ps, sk, side, ground, 0.04, 1.2)
            for f in range(1, len(Ps)):
                if m[f] and m[f - 1]:
                    sl.append(np.linalg.norm((Ps[f, piv[f]] - Ps[f - 1, piv[f]])[[0, 2]]) * 30)
        hy = Ps[:, sk.idx['Hips'], 1]
        hipv = np.abs(hy[2:] - 2 * hy[1:-1] + hy[:-2]).max() * 900 if len(hy) > 2 else 0
        print('SCORE %-34s bnd %6.1f (ref %5.1f) m/s2 | slide %5.1f cm/s | hipv %5.1f m/s2'
              % (os.path.basename(sp), bnd, ref, np.mean(sl) * 100 if sl else -1, hipv))


if __name__ == '__main__':
    main(sys.argv[1:])
