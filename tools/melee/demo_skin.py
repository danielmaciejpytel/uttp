"""Skin the ORIGINAL (game) and the CORRECTED (MotionBricks) version of a melee clip on Female V (graph deformation, world cache).

  python -I demo_skin.py <set> <clip> [--mb corrected\\<set>\\<clip>.mb.dump.npz] [--out demo\\<clip>]
Writes <out>\\orig.skin.{npz,pc2} and <out>\\mb.skin.{npz,pc2} (+ the utppclip json), same path as melee_sheet.py (dump W -> utppclip,
Trajectory = hips ground point/heading window 1 -> skin_clip.py --world --deform graph).
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402


def skin(dump_path, out_base, rig, label, deform='graph'):
    import numpy as np
    os.environ['UTPP_RIG'] = rig
    sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion'))
    sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
    import game_skeleton_bridge as CP
    assert CP.RIG_NAME == rig, 'game_skeleton_bridge was imported for %s, cannot skin a %s clip in this process (set UTPP_RIG / --rig first)' % (CP.RIG_NAME, rig)
    z = np.load(dump_path)
    names, par, ref = CP.load_rig()
    clip = CP.to_utppclip(z['W'], names, par, ref, z['rest'], label, win=1)
    cj = out_base + '.utppclip.json'
    json.dump(clip, open(cj, 'w'), separators=(',', ':'))
    p = subprocess.run([MC.PY, '-I', os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion', 'skin_clip.py'), cj, out_base, '--world',
                        '--deform', deform, '--body', rig, '--strict'], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('skin_clip failed: ' + p.stderr[-800:])
    print(p.stdout.strip())


def main(a):
    s, c = a[0], a[1]
    out = a[a.index('--out') + 1] if '--out' in a else os.path.join(MC.WORK, 'demo', s + '__' + c)
    from demo_metrics import mb_prefix
    mb = a[a.index('--mb') + 1] if '--mb' in a else mb_prefix(s, c) + '.mb.dump.npz'
    os.makedirs(out, exist_ok=True)
    rig = MC.RIG_OF_SET[s]
    dfm = a[a.index('--deform') + 1] if '--deform' in a else 'graph'
    skin(MC.clip_path(s, c), os.path.join(out, 'orig.skin'), rig, c + '_orig', dfm)
    skin(mb, os.path.join(out, 'mb.skin'), rig, c + '_mb', dfm)


if __name__ == '__main__':
    main(sys.argv[1:])
