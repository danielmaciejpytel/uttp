"""Demo material of the MotionBricks stage (v4) for the demo clips of demo_metrics.CLIPS.

  python -I melee_regenerate_demo.py [--only <clip>] [--seeds 0,7,11] [--no-render] [--no-run] [--rig man] [--clips set/clip,set/clip] [--variant final]
  --variant final (round F): the clips are skinned / rendered as 'final' (own colour in redo_render.COL), {3q,side}_{base,orig} copied from the v3b demo,
  penetration counted with redo_pen.skinned (pose labels); default 'v4' unchanged.
For every demo clip (demo_metrics.CLIPS for woman, CLIPS_MAN for man; ONE MotionBricks run for all of them) -> <mb4>\\demo3\\<set>__<clip>\\
(<mb4> = melee_regenerate.MB4: mb4 / mb4_man per rig; for the Male V demo prefer melee_regenerate_man.py demo, which also copies the base / orig renders of redo_man\\demo3) :
  copies of redo\\demo3\\<set>__<clip>\\{base,orig,redo}.skin.* and *.dump.npz (never moved), v4.dump.npz, v4.skin.* (demo_skin.skin, read-only use),
  {3q,side}_v4\\f_####.png (+ {3q,side}_redo\\ from the same render, same camera) 424x720 through redo_render.py, and <mb4>\\demo3\\report.json.
redo_render.py has no 'v4' colour: the v4 skin is rendered as variant 'mb' (blue) in a scratch dir (--variants redo,mb --camvar redo, camera fixed to the V3 hips so the
hips sway of V4 stays visible) and the folders are renamed *_mb -> *_v4.
"""
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_regenerate as M  # noqa: E402  (sets UTPP_RIG / --rig before base_body)
import redo_demo as RD  # noqa: E402
import redo_pen_check as RPC  # noqa: E402
import redo_skin_check as RSC  # noqa: E402
import demo_skin  # noqa: E402
import melee_common as MC  # noqa: E402
import anatomy_gate_base as GB  # noqa: E402
import redo_solve as RS  # noqa: E402
import redo_pen as RPN  # noqa: E402
from demo_metrics import CLIPS, CLIPS_MAN  # noqa: E402

SRC_DEMO = os.path.join(M.BB.REDO, 'demo3')
VAR = 'final' if '--variant' in sys.argv and sys.argv[sys.argv.index('--variant') + 1] == 'final' else 'v4'


def make(s, c, demo_root, clips_dir, info, render=True):
    B = RS.Body()
    al = GB.allowed()
    rig = MC.RIG_OF_SET[s]
    out = os.path.join(demo_root, '%s__%s' % (s, c))
    os.makedirs(out, exist_ok=True)
    src = os.path.join(SRC_DEMO, '%s__%s' % (s, c))
    for n in ('base', 'orig', 'redo'):
        for ext in ('.skin.npz', '.skin.pc2', '.skin.utppclip.json'):
            if os.path.exists(os.path.join(src, n + ext)) and not os.path.exists(os.path.join(out, n + ext)):
                shutil.copy2(os.path.join(src, n + ext), os.path.join(out, n + ext))
    for n in ('base.dump.npz', 'redo.dump.npz'):
        if os.path.exists(os.path.join(src, n)):
            shutil.copy2(os.path.join(src, n), os.path.join(out, n))
    v4 = os.path.join(clips_dir, '%s__%s.v4.npz' % (s, c))
    V = VAR             # 'v4' (default, round E) | 'final' (round F, --variant final: own colour; base / orig / redo rendered in the same call, same camera)
    shutil.copy2(v4, os.path.join(out, V + '.dump.npz'))
    demo_skin.skin(os.path.join(out, V + '.dump.npz'), os.path.join(out, V + '.skin'), rig, c + '_' + V)
    if render:
        rv = 'mb' if V == 'v4' else V
        extra = [] if V == 'v4' else ['base', 'orig']      # the frames of the v3b demo were moved to Media\ (media rule): render them again from the copied skins
        tmp = os.path.join(M.MB4, '_tmp', 'render_%s__%s' % (s, c))
        shutil.rmtree(tmp, ignore_errors=True)
        os.makedirs(tmp)
        for ext in ('.skin.npz', '.skin.pc2'):
            for n in extra + ['redo']:
                shutil.copy2(os.path.join(out, n + ext), os.path.join(tmp, n + ext))
            shutil.copy2(os.path.join(out, V + ext), os.path.join(tmp, rv + ext))
        p = subprocess.run([RD.BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'redo_render.py'), '--', tmp, '--res', '424,720', '--variants',
                            ','.join(extra + ['redo', rv]), '--camvar', 'redo'], capture_output=True, text=True)
        if 'RENDERED' not in p.stdout:
            raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-600:])
        for view in ('3q', 'side'):
            for a, b in [(view + '_' + rv, view + '_' + V), (view + '_redo', view + '_redo')] + [(view + '_' + n, view + '_' + n) for n in extra]:
                dst = os.path.join(out, b)
                shutil.rmtree(dst, ignore_errors=True)
                shutil.move(os.path.join(tmp, a), dst)
        shutil.rmtree(tmp, ignore_errors=True)
    skin = {V: RSC.check(out, V), 'redo': RSC.check(out, 'redo')}
    if V == 'v4':
        pen = dict(v4=RPC.count(os.path.join(out, 'v4.skin')), redo=RPC.count(os.path.join(out, 'redo.skin')))
    else:               # round F: the pose-labelled count of redo_pen (redo_pen_check labels against the T-pose, see redo_pen.py)
        pen = {V: RPN.skinned(os.path.join(out, V + '.skin'), np.load(v4)['W']),
               'redo': RPN.skinned(os.path.join(out, 'redo.skin'), np.load(os.path.join(out, 'redo.dump.npz'))['W'])}
    W4 = np.load(v4)['W']
    d = MC.load_dump(MC.clip_path(s, c))
    return dict(frames=info['frames'], keys=info['keys'], v3_final_pass=info['gate']['v3'], v4_final_pass=info['gate']['v4'],
                segments=[{k: v for k, v in x.items() if k != 'candidates'} for x in info['segments']], segments_mb=info['segments_mb'],
                segments_fallback=info['segments_fallback'], foot=info['foot'], spikes=info['spikes'], rot_step=info['rot_step'], added_motion=info['added_motion'],
                pen_max={V: int(pen[V].max()), 'v3': int(pen['redo'].max())}, pen_frames_over_0={V: int((pen[V] > 0).sum()), 'v3': int((pen['redo'] > 0).sum())},
                skin_check=skin, intent_v4=RD.intent_metrics(B, d, W4), timing_s=info['timing_s'])


def main(a):
    only = a[a.index('--only') + 1] if '--only' in a else None
    seeds = [int(x) for x in a[a.index('--seeds') + 1].split(',')] if '--seeds' in a else [0, 7, 11]
    clips_dir = os.path.join(M.MB4, 'clips')
    demo_root = os.path.join(M.MB4, 'demo3')
    if '--clips' in a:      # explicit list (e.g. the male demo clips): set/clip,set/clip
        pairs = [tuple(x.split('/')) for x in a[a.index('--clips') + 1].split(',')]
    else:
        pairs = [(s, c) for s, c in (CLIPS if M.RIG == 'woman' else CLIPS_MAN) if (not only or c == only) and MC.RIG_OF_SET[s] == M.RIG]
    assert all(MC.RIG_OF_SET[s] == M.RIG for s, c in pairs), 'clip set of another rig than UTPP_RIG=%s' % M.RIG
    if '--no-run' not in a:
        M.run_clips(pairs, seeds, clips_dir, keep_tmp=False)
    rep_path = os.path.join(demo_root, 'report.json')
    rep = json.load(open(rep_path)) if os.path.exists(rep_path) else {}
    for s, c in pairs:
        info = json.load(open(os.path.join(clips_dir, '%s__%s.v4.info.json' % (s, c))))
        rep['%s/%s' % (s, c)] = make(s, c, demo_root, clips_dir, info, render='--no-render' not in a)
        print('DEMO', s, c, 'v3', info['gate']['v3']['pass_frames'], 'v4', info['gate']['v4']['pass_frames'], '/', info['frames'])
    MC.jdump(rep, rep_path)


if __name__ == '__main__':
    main(sys.argv[1:])
