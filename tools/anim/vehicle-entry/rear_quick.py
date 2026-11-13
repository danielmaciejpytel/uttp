"""One-line start / rearward / joint-rule screen of clips (start_gate.evaluate without the row exemptions) plus the largest per-frame local rotation step of the leg bones.

  python -I rear_quick.py <utpp_car_p?a_<class>_<seat>.anims.glb> [...]
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402
import start_gate as SG  # noqa: E402
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402


def leg_steps(glb):
    tgt = S.Skeleton.from_glb(glb)
    clip = tgt.clip('enter')
    dur, times = B.frames_of(tgt, clip)
    lt, lr = tgt.sample_local(clip, times)
    R = S.q2m(S.qcontinuous(lr))
    out = []
    for bn in ('RightLeg', 'LeftLeg', 'RightUpLeg', 'LeftUpLeg'):
        j = tgt.index[bn]
        d = np.einsum('tji,tjk->tik', R[:-1, j], R[1:, j])
        out.append(float(np.degrees(np.arccos(np.clip((np.trace(d, axis1=1, axis2=2) - 1) / 2, -1, 1))).max()))
    return max(out)


def main():
    for p in sys.argv[1:]:
        r = SG.evaluate(p, None, None, 'auto', exempt=False)
        rw = r['rearward']['rows']
        st = r['start']
        print('%-60s start dy %+.2f slide %.2f %s | rear %s H %.3f/%.3f S %.3f/%.3f sink %.3f/%.3f | joints %s | leg step %.1f' % (
            os.path.relpath(p, os.path.join(project_root(), r'Build\work\vehicle-entry-car\r7util'))[:60], st['dy_vs_vanilla_m'], st['slide_m'], 'ok' if st['ok'] else 'FAIL',
            'ok' if r['rearward']['ok'] else 'FAIL', rw['Hips']['excursion_m'], rw['Hips']['limit_m'], rw['Spine']['excursion_m'], rw['Spine']['limit_m'],
            rw['sink_slide_Hips']['excursion_m'], rw['sink_slide_Hips']['limit_m'], 'ok' if r['joint_rule']['ok'] else 'FAIL', leg_steps(p)), flush=True)


if __name__ == '__main__':
    main()
