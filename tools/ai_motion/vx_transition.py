"""Scene of the car-exit -> idle transition clip (VX_Exit_Idle_N10_MB) for a NEW exit clip, reusing the idle half of the existing scene.

python -I vx_transition.py <new_exit.dump.npz> <old_scene.dump.npz> <out_scene.dump.npz> [--endA 60] [--N 10] [--inplace]
--inplace: the idle half is moved in x/y onto the exit end's ankle midpoint (no walk in the gap; see the comment in main).

The pilot scene (compose_transition.py --rig cp --align facing: exit | 4N-8 placeholder frames | source pack M_Neutral_Stand_Idle_Loop placed at the exit end, game axes, floor shifted so the end feet
stand on z = rest ankle height) is kept for its idle half (frames g..); only the exit part (frames 0..endA) is replaced by the new exit clip, shifted by the same floor offset, and the gap placeholders
are re-copied from its last frame. The exit's last frame is the vanilla one in both clips, so the idle half stays valid (checked: the last frames of the old / new exit differ by < 0.05 cm).
"""
import sys

import numpy as np


def main(a):
    A = np.load(a[0], allow_pickle=True)
    O = np.load(a[1], allow_pickle=True)
    endA = int(a[a.index('--endA') + 1]) if '--endA' in a else 60
    N = int(a[a.index('--N') + 1]) if '--N' in a else 10
    names = [str(x) for x in A['names']]
    assert names == [str(x) for x in O['names']], 'different skeletons'
    WA, WO = A['W'], O['W']
    fa = [names.index('LeftFoot'), names.index('RightFoot')]
    # the old scene was shifted by dz; recover it from the last exit frame (identical in both clips) and verify
    dz = float((WO[endA, :, 2, 3] - WA[endA, :, 2, 3]).mean())
    shifted = WA[endA].copy()
    shifted[:, 2, 3] += dz
    last_diff_cm = float(np.abs(shifted[:, :3, 3] - WO[endA][:, :3, 3]).max() * 100)
    gap = 4 * N - 8
    g = endA + 4 * N - 7
    assert int(O['g']) == g and int(O['c']) == endA, 'old scene c / g differ'
    Wn = WA[:endA + 1].copy()
    Wn[:, :, 2, 3] += dz
    B = WO[g:].copy()
    if '--inplace' in a:
        # 2026-11-05 (an unnecessary step after the exit was observed): the pilot placed the idle D ahead of the exit end (D from the gliding
        # exit tail speed, ~35 cm), so the generator walked two steps to reach it. --inplace moves the idle half rigidly in x/y so its
        # ankle midpoint at g lies on the exit end's ankle midpoint (headings already matched by --align facing): the gap only shifts
        # weight / adjusts the feet. Without the option the scene is unchanged.
        mA = Wn[endA, fa, :2, 3].mean(0)
        mB = B[0, fa, :2, 3].mean(0)
        B[:, :, :2, 3] += (mA - mB)
        print('VX_INPLACE idle moved by (%.3f, %.3f) m' % tuple(mA - mB))
    W = np.concatenate([Wn, np.repeat(Wn[endA:endA + 1], gap, 0), B])
    assert len(W) == len(WO), (len(W), len(WO))
    keep = {k: O[k] for k in O.files if k not in ('W', 'frames')}
    np.savez(a[2], W=W, frames=len(W), **keep)
    print('VX_SCENE %s frames %d c=%d g=%d N=%d | dz %.4f m | last exit frame vs old scene: max %.4f cm' % (a[2], len(W), endA, g, N, dz, last_diff_cm))
    if last_diff_cm > 0.05:
        sys.exit('last exit frame differs from the old scene: the idle half would not match')


if __name__ == '__main__':
    main(sys.argv[1:])
