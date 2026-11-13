"""Redo demo (v3): per clip solve (redo_solve), per-frame gate (anatomy_gate_base), skin base / orig / redo on Female / Male V, skin check vs the BASE body, Blender panels.

  python -I redo_demo.py [--rig man] [--root DIR] [--only <clip>] [--no-render] [--no-skin]
Per clip of demo_metrics.CLIPS (woman) / CLIPS_MAN (man) -> <root>\\<set>__<clip>\\{base,orig,redo}.skin.*  and  {3q,side}_{base,orig,redo}\\f_####.png (redo_render.py), <root>\\report.json.
Panels: base = neutral V idle (static, the reference), orig = game clip, redo = redone clip.  Encoding: redo_compose.py.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate_base as GB  # noqa: E402
import base_body as BB  # noqa: E402
import demo_skin  # noqa: E402
import melee_common as MC  # noqa: E402
import redo_skin_check as RSC  # noqa: E402
import redo_solve as RS  # noqa: E402
from demo_metrics import CLIPS, CLIPS_MAN  # noqa: E402

BLENDER = os.environ.get('UTPP_BLENDER_EXE') or os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Steam', 'steamapps', 'common', 'Blender', 'blender.exe')


def intent_metrics(B, d, W):
    """How much of the strike survives: hand positions relative to the chest, game vs redo (cm), and the frame of the peak hand speed."""
    ix = B.ix
    res = {}
    for S in ('Left', 'Right'):
        hv = d['W'][:, ix[S + 'Hand'], :3, 3] - d['W'][:, ix['Spine3'], :3, 3]
        hr = W[:, ix[S + 'Hand'], :3, 3] - W[:, ix['Spine3'], :3, 3]
        sv = np.linalg.norm(np.diff(d['W'][:, ix[S + 'Hand'], :3, 3], axis=0), axis=1)
        sr = np.linalg.norm(np.diff(W[:, ix[S + 'Hand'], :3, 3], axis=0), axis=1)
        # trajectory shape: correlation of the hand speed curves, peak frames, travelled distance ratio
        cc = float(np.corrcoef(sv, sr)[0, 1]) if sv.std() > 1e-9 and sr.std() > 1e-9 else 1.0
        res[S] = dict(peak_frame_game=int(sv.argmax()), peak_frame_redo=int(sr.argmax()), speed_corr=round(cc, 3),
                      path_cm_game=round(float(sv.sum() * 100), 1), path_cm_redo=round(float(sr.sum() * 100), 1))
    return res


def main(a):
    root = os.path.abspath(a[a.index('--root') + 1]) if '--root' in a else os.path.join(BB.REDO, 'demo3')
    only = a[a.index('--only') + 1] if '--only' in a else None
    B = RS.Body()
    al = GB.allowed()
    rep = {}
    for s, c in (CLIPS if BB.RIG_NAME == 'woman' else CLIPS_MAN):
        if only and c != only:
            continue
        out = os.path.join(root, s + '__' + c)
        os.makedirs(out, exist_ok=True)
        rig = MC.RIG_OF_SET[s]
        assert rig == BB.RIG_NAME, (s, rig, BB.RIG_NAME)
        d = MC.load_dump(MC.clip_path(s, c))
        gv = GB.frame_gate(d['W'], B.rg, al=al)
        intent = RS.plan_intent(B, d)
        Wr, info, intent = RS.solve_refined(B, d, intent)
        gr = GB.frame_gate(Wr, B.rg, al=al)
        # keep game frames that pass the gate, redo the rest, smoothed transitions
        Wk, wg = RS.blend_keep(B, d['W'], Wr, RS.keep_mask(B, d['W'], gv['ok']))
        gk = GB.frame_gate(Wk, B.rg, al=al)
        T = len(Wk)
        Tw = intent['traj0']
        Wbase = Tw[None, None] @ np.repeat(B.Wi[None], T, 0)
        Wbase[:, 0] = np.eye(4)
        Wbase[:, 1] = Tw
        paths = {}
        for v, W in (('base', Wbase), ('redo', Wk)):
            p = os.path.join(out, v + '.dump.npz')
            np.savez(p, W=W, rest=B.rest, names=np.array(B.names), parents=np.array(B.par), fps=30)
            paths[v] = p
        np.savez(os.path.join(out, 'redo_solved.npz'), W=Wr, wg=wg)
        if '--no-skin' not in a:
            demo_skin.skin(paths['base'], os.path.join(out, 'base.skin'), rig, c + '_base')
            demo_skin.skin(MC.clip_path(s, c), os.path.join(out, 'orig.skin'), rig, c + '_orig')
            demo_skin.skin(paths['redo'], os.path.join(out, 'redo.skin'), rig, c + '_redo')
        if '--no-render' not in a and '--no-skin' not in a:
            p = subprocess.run([BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'redo_render.py'), '--', out, '--res', '424,720',
                                '--variants', 'base,orig,redo', '--camvar', 'redo'], capture_output=True, text=True)
            if 'RENDERED' not in p.stdout:
                raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-600:])
        skin = {}
        if '--no-skin' not in a:
            skin = dict(redo=RSC.check(out, 'redo'), orig=RSC.check(out, 'orig'))
        ex_keys = ('clav_shift_L_cm', 'clav_shift_R_cm', 'spine_bend_Spine1', 'spine_bend_Spine2')
        ok_rel = np.ones(T, bool)
        for k, v in gv['viol'].items():
            if k not in ex_keys:
                ok_rel &= v <= 1e-9
        toe = [float(np.linalg.norm(np.ptp(Wk[:, B.ix[S + 'ToeBase'], :2, 3], axis=0)) * 100) for S in ('Left', 'Right')]
        rep['%s/%s' % (s, c)] = dict(game_pass_if_clav_shift_and_spine_bend_ignored=int(ok_rel.sum()), toe_base_drift_cm=[round(x, 3) for x in toe], skin_check=skin, frames=T, game_pass=GB.summary(gv), redo_solved_pass=GB.summary(gr), final_pass=GB.summary(gk),
                                     game_frames_kept=int((wg >= 1 - 1e-6).sum()), game_frames_blended=int(((wg > 1e-6) & (wg < 1 - 1e-6)).sum()),
                                     redone_frames=int((wg <= 1e-6).sum()), solve=info, intent=intent_metrics(B, d, Wk))
        print(s, c, json.dumps({k: rep['%s/%s' % (s, c)][k] for k in ('frames', 'game_frames_kept', 'game_frames_blended', 'redone_frames')}),
              'solved pass', gr['ok'].sum(), '/', T, 'final pass', gk['ok'].sum(), '/', T, GB.summary(gk)['fail_features'])
    MC.jdump(rep, os.path.join(root, 'report%s.json' % ('_' + only if only else '')))


if __name__ == '__main__':
    main(sys.argv[1:])
