"""Real stepping for a car ENTRY clip (2026-11-06): splice the bones of a generated 'steps' npz (Source\\tools\\ai_motion\\entry_steps_gen.py assemble) into the BUILDER GLB of a shipped
entry run (<run>\\glb\\utpp_car_p?a_<class>_<seat>.anims.glb, clip 'enter'). Same splice as the exit (exit_steps.splice / reanchor_helpers): Hips, spine to Spine2 and both legs come from the
generator on the frames c..g (blended with the existing pose over --ramp frames at both ends, weight 0 at c and g), Spine3 keeps its world orientation, the limb IK helpers of those frames
are re-anchored on the new limbs with the clip's own per-frame offsets. Root (root motion), the arms / head / neck, events, float tracks and every frame outside c..g stay the input clip
bit for bit (the report states the largest difference per frame / bone).

  python -I build_entry_steps.py --glb <builder .anims.glb> --steps <steps.npz> [--steps ...more spans] --out <new .anims.glb> [--clip enter] [--name enter] [--ramp 3] [--bones lower] [--report r.json]
"""
import argparse
import copy
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
import exit_steps as ES  # noqa: E402


def quat_angle_deg(a, b):
    d = np.abs(np.sum(a * b, axis=-1)) / np.maximum(np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1), 1e-12)
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb', required=True)
    p.add_argument('--clip', default='enter')
    p.add_argument('--name', default='enter')
    p.add_argument('--steps', action='append', required=True, help='steps npz (one per span, spans must not overlap)')
    p.add_argument('--out', required=True)
    p.add_argument('--ramp', type=int, default=3)
    p.add_argument('--bones', default='lower', choices=('lower', 'all'))
    p.add_argument('--report')
    a = p.parse_args()
    tgt = S.Skeleton.from_glb(a.glb)
    c = tgt.clip(a.clip)
    dur, times = B.frames_of(tgt, c)
    lt0, lr0 = tgt.sample_local(c, times)
    lr0 = S.qcontinuous(lr0)
    lt, lr = lt0.copy(), lr0.copy()
    rep = dict(glb=str(a.glb), clip=a.clip, frames=len(times), duration_s=float(dur), spans=[])
    for sp in a.steps:
        st = np.load(sp, allow_pickle=True)
        r = ES.splice(tgt, lt, lr, st, a.ramp, a.bones)
        vWt, vWr = tgt.world(lt0, lr0)
        r['helpers'] = ES.reanchor_helpers(tgt, lt, lr, vWt, vWr, int(st['c']), int(st['g']), (int(st['c_gen']), int(st['g_gen'])) if 'c_gen' in st.files else None)
        r['steps_file'] = str(sp)
        rep['spans'].append(r)
    ri = tgt.index['Root']
    dang = np.stack([quat_angle_deg(lr[:, j], lr0[:, j]) for j in range(lr.shape[1])], 1)
    dtr = np.linalg.norm(lt - lt0, axis=-1)
    chg = (dang > 1e-3) | (dtr > 1e-5)
    rep['changed_frames'] = [int(t) for t in np.flatnonzero(chg.any(1))]
    rep['changed_bones'] = sorted({tgt.names[j] for j in np.flatnonzero(chg.any(0))})
    rep['first_frame_max_diff'] = dict(rot_deg=float(dang[0].max()), trans_m=float(dtr[0].max()))
    rep['last_frame_max_diff'] = dict(rot_deg=float(dang[-1].max()), trans_m=float(dtr[-1].max()))
    rep['root_max_diff'] = dict(rot_deg=float(dang[:, ri].max()), trans_m=float(dtr[:, ri].max()))
    extras = copy.deepcopy(tgt.gj['animations'][c].get('extras', {}))
    B.write_out(tgt, a.out, a.name, times, lt, lr, extras)
    if a.report:
        json.dump(rep, open(a.report, 'w'), indent=1)
    print('spans %s changed frames %s..%s bones %d first %s last %s root %s' % ([(s['c'], s['g']) for s in rep['spans']], rep['changed_frames'][:1], rep['changed_frames'][-1:], len(rep['changed_bones']),
          rep['first_frame_max_diff'], rep['last_frame_max_diff'], rep['root_max_diff']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
