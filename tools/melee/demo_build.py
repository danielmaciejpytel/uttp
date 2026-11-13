"""Build the gated melee demo (v2): skin ORIGINAL + CORRECTED on Female V, skin check, Blender panels, per clip.

  python -I demo_build.py [--root Build\...\demo2] [--only <clip>] [--build] [--no-render]
  --build: also (re)runs apply_keyposes (gate + skin acceptance) and melee_keyposes (gated seeds, arms locked to the corrected source) per clip.
Per clip of demo_metrics.CLIPS: corrected = <root>\corrected\<set>\<clip>.mb.dump.npz (clips without a generated span: .timed.npz), skinned
with demo_skin.skin (same deformation graph as the sheets), anatomy_gate.skin_check against the original at the mapped frame, then
demo_render.py (blender) -> <root>\<set>__<clip>\{3q,side}_{orig,mb}. Writes <root>\skin_check.json. Encoding: demo_compose.py --demo-dir <root>.
"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import demo_skin  # noqa: E402
import melee_common as MC  # noqa: E402
from demo_metrics import CLIPS  # noqa: E402

BLENDER = os.environ.get('UTPP_BLENDER_EXE') or os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Steam', 'steamapps', 'common', 'Blender', 'blender.exe')


def corrected_dump(root, s, c):
    p = os.path.join(root, 'corrected', s, c + '.mb.dump.npz')
    return p if os.path.exists(p) else os.path.join(root, 'corrected', s, c + '.timed.npz')


CORR = {('pwa_katana', 'combo_attack_04_hit'): 'demo\corr', ('pwa_two_handed_blunt', 'heavy_attack_01'): 'demo\corr'}   # refined authoring for the demo; the others: corrections\
SEEDS = [7, 11, 3, 1, 23, 5, 13]


def corr_path(s, c):
    return os.path.join(MC.WORK, CORR.get((s, c), 'corrections'), s, c + '.json')


def ref_skin(s, c, dv, keys, d):
    """Skin of the vanilla clip RE-TIMED with the key time map (what the corrected clip is compared with): a pure retiming is no deformation."""
    os.makedirs(d, exist_ok=True)
    Wv = AG.vanilla_on_timeline(dv, keys)
    np.savez(os.path.join(d, 'ref.npz'), W=Wv, rest=dv['rest'])
    demo_skin.skin(os.path.join(d, 'ref.npz'), os.path.join(d, 'ref.skin'), MC.RIG_OF_SET[s], c + '_ref')


def apply_with_skin(s, c, root, dv):
    """apply_keyposes (anatomy gate) + the skinned-mesh stretch check on the timed clip as a second acceptance test."""
    import apply_keyposes as AK
    tmp = os.path.join(root, '_chk', s + '__' + c)
    os.makedirs(tmp, exist_ok=True)
    corr = json.load(open(corr_path(s, c), encoding='utf-8'))
    keys0 = [dict(src_frame=k['src_frame'], time_frame=k.get('time_frame', k['src_frame'])) for k in corr['keys']]
    ref_skin(s, c, dv, keys0, tmp)

    def extra(timed, n_out):
        AK.save_dump(os.path.join(tmp, 'chk.npz'), timed)
        demo_skin.skin(os.path.join(tmp, 'chk.npz'), os.path.join(tmp, 'chk.skin'), MC.RIG_OF_SET[s], c + '_chk')
        keys = [dict(src_frame=k['src_frame'], time_frame=k.get('time_frame', k['src_frame'])) for k in corr['keys']]
        r = AG.skin_check(tmp, dv['W'][0], AG.rig_of(dv), np.arange(n_out, dtype=float), corr_name='chk', vanilla_name='ref')
        fr = r['worst'][0][1] if r['worst'] else 0
        return r['passed'], fr, max(r['excess_max'], 0.0)
    info, _, _ = AK.run(corr_path(s, c), os.path.join(root, 'corrected', s), extra=extra)
    return info


def run_mb(s, c, root, seed_args):
    cmd = [MC.PY, '-I', os.path.join(HERE, 'melee_keyposes.py'), os.path.join(root, 'corrected', s, c + '.keys.json')] + seed_args
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('melee_keyposes failed: ' + (p.stdout + p.stderr)[-800:])
    return p.stdout


def main(a):
    root = os.path.abspath(a[a.index('--root') + 1]) if '--root' in a else os.path.join(MC.WORK, 'demo2')
    only = a[a.index('--only') + 1] if '--only' in a else None
    res = {}
    for s, c in CLIPS:
        if only and c != only:
            continue
        out = os.path.join(root, s + '__' + c)
        os.makedirs(out, exist_ok=True)
        rig = MC.RIG_OF_SET[s]
        dv = MC.load_dump(MC.clip_path(s, c))
        demo_skin.skin(MC.clip_path(s, c), os.path.join(out, 'orig.skin'), rig, c + '_orig')
        if '--build' in a:
            info = apply_with_skin(s, c, root, dv)
            print('APPLY %s/%s gate %s %s' % (s, c, info['gate']['status'], info['gate']['key_scales']))
        kj = json.load(open(os.path.join(root, 'corrected', s, c + '.keys.json'), encoding='utf-8'))
        ref_skin(s, c, dv, kj['keys'], out)
        src = np.arange(kj['output_frames'], dtype=float)
        has_mb = any(x['mode'] == 'mb' for x in kj['segments'])
        r = None
        if '--build' in a and has_mb:      # first MotionBricks seed that passes the anatomy gate AND the skin check; otherwise the offset layer
            for seed in SEEDS:
                run_mb(s, c, root, ['--seed', str(seed)])
                inf = json.load(open(os.path.join(root, 'corrected', s, c + '.mb.info.json')))
                if inf.get('gate', {}).get('mode') == 'offset_fallback':
                    break
                demo_skin.skin(corrected_dump(root, s, c), os.path.join(out, 'mb.skin'), rig, c + '_mb')
                r = AG.skin_check(out, dv['W'][0], AG.rig_of(dv), src, vanilla_name='ref')
                print('  seed %d skin %s %.3f' % (seed, r['passed'], r['excess_max']))
                if r['passed']:
                    break
            else:
                run_mb(s, c, root, ['--no-mb', '--no-gate'])
                r = None
        demo_skin.skin(corrected_dump(root, s, c), os.path.join(out, 'mb.skin'), rig, c + '_mb')
        r = AG.skin_check(out, dv['W'][0], AG.rig_of(dv), src, vanilla_name='ref')
        res['%s/%s' % (s, c)] = r
        print('SKIN_CHECK %s/%s %s max %.3f %s' % (s, c, 'PASS' if r['passed'] else 'FAIL', r['excess_max'], r['worst'][:3]))
        if '--no-render' not in a:
            p = subprocess.run([BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'demo_render.py'), '--', out], capture_output=True, text=True)
            if 'RENDERED' not in p.stdout:
                raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-600:])
    MC.jdump(res, os.path.join(root, 'skin_check.json'))


if __name__ == '__main__':
    main(sys.argv[1:])
