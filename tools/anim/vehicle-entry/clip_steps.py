"""Per-bone continuity check of a clip in a GLB: max world rotation step (deg/frame) and the frame, top N bones."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import glbanim as G  # noqa: E402


def steps(path, clip, top=8, bones=None):
    sk = S.Skeleton.from_glb(path)
    c = sk.clip(clip)
    ch = G.channels(sk.gj, sk.binb, c)
    dur = max(float(tr[0][-1]) for d in ch.values() for tr in d.values())
    t = np.arange(int(round(dur * 30)) + 1) / 30
    lt, lr = sk.sample_local(c, t)
    Wt, Wr = sk.world(lt, S.qcontinuous(lr))
    res = []
    for i, n in enumerate(sk.names):
        if bones and n not in bones:
            continue
        R = np.einsum('tji,tjk->tik', Wr[:-1, i], Wr[1:, i])
        a = np.degrees(np.arccos(np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)))
        res.append((float(a.max()), int(a.argmax()) + 1, n))
    res.sort(reverse=True)
    return res[:top]


if __name__ == '__main__':
    bones = None
    if len(sys.argv) > 3:
        bones = set(sys.argv[3].split(','))
    for r in steps(sys.argv[1], sys.argv[2], bones=bones):
        print('%.1f deg at frame %d  %s' % r)
