"""Contact sheet of selected frames of one melee clip: skinned V body (woman / man per rig), 3/4 third-person view.

  python -I melee_sheet.py <set> <clip> --frames 0,5,9,... --out <sheet.png> [--deform graph|rigid] [--cols 4] [--cam dx,dy,dz]
                          [--source npz\\<set>\\<clip>.npz | <other dump.npz>] [--keep]
Pipeline (only the requested frames are skinned): dump W -> utppclip (game_skeleton_bridge.to_utppclip, Trajectory = hips ground point
and heading) -> skin_clip.py --world (LBS + the game's body deformation graph, --deform rigid is faster) -> Blender Workbench
stills (render_sheet.py) -> ffmpeg tile. Temporary files go to sheets\\_tmp\\ and are removed unless --keep.
--source: any dump npz with the dump layout (e.g. a corrected / generated clip written by apply_keyposes / melee_keyposes).
The root yaw of the hips is kept; the camera is a fixed world offset from the hips (it does not turn with the character).
"""
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

BLENDER = os.environ.get('UTPP_BLENDER_EXE') or os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Steam', 'steamapps', 'common', 'Blender', 'blender.exe')
FFMPEG = os.path.join(MC.ROOT, 'Tools', 'pylibs', 'imageio_ffmpeg', 'binaries', 'ffmpeg-win-x86_64-v7.1.exe')


def opt(a, k, default=None):
    return a[a.index(k) + 1] if k in a else default


def build_sheet(dump_path, frames, out_png, rig, deform='graph', cols=4, cam=None, keep=False, tag=None, label_prefix=''):
    import numpy as np
    os.environ['UTPP_RIG'] = rig
    sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion'))
    sys.path.insert(0, os.path.join(MC.ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
    import game_skeleton_bridge as CP
    t0 = time.time()
    z = np.load(dump_path)
    names, par, ref = CP.load_rig()
    W = z['W'][frames]
    clip = CP.to_utppclip(W, names, par, ref, z['rest'], 'sheet', win=1)
    tmp = os.path.join(MC.WORK, 'sheets', '_tmp', tag or '%s_%d' % (os.path.splitext(os.path.basename(out_png))[0], os.getpid()))  # pid: parallel processes render same-named clips of different sets
    os.makedirs(tmp, exist_ok=True)
    cj = os.path.join(tmp, 'clip.utppclip.json')
    json.dump(clip, open(cj, 'w'), separators=(',', ':'))
    t1 = time.time()
    base = os.path.join(tmp, 'skin')
    p = subprocess.run([MC.PY, '-I', os.path.join(MC.ROOT, 'Source', 'tools', 'ai_motion', 'skin_clip.py'), cj, base, '--world',
                        '--deform', deform, '--body', rig], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('skin_clip failed: ' + p.stderr[-600:])
    t2 = time.time()
    rcmd = [BLENDER, '-b', '--factory-startup', '-P', os.path.join(HERE, 'render_sheet.py'), '--', base, tmp,
            ','.join(label_prefix + str(f) for f in frames)] + (['--cam', cam] if cam else [])
    p = subprocess.run(rcmd, capture_output=True, text=True)
    if 'RENDERED' not in p.stdout:
        raise RuntimeError('blender failed: ' + (p.stdout + p.stderr)[-800:])
    t3 = time.time()
    rows = (len(frames) + cols - 1) // cols
    cmd = [FFMPEG, '-y', '-loglevel', 'error', '-framerate', '1', '-i', os.path.join(tmp, 'f_%03d.png'), '-vf',
           'tile=%dx%d:padding=2:color=white' % (cols, rows), '-frames:v', '1', out_png]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg failed: ' + p.stderr[-400:])
    t4 = time.time()
    if not keep:
        shutil.rmtree(tmp, ignore_errors=True)
    return dict(utppclip=round(t1 - t0, 1), skin=round(t2 - t1, 1), render=round(t3 - t2, 1), tile=round(t4 - t3, 1), total=round(t4 - t0, 1))


def main(a):
    s, c = a[0], a[1]
    frames = [int(x) for x in opt(a, '--frames').split(',')]
    src = opt(a, '--source') or MC.clip_path(s, c)
    out = opt(a, '--out') or os.path.join(MC.WORK, 'sheets', s, c + '.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    t = build_sheet(src, frames, out, MC.RIG_OF_SET[s], opt(a, '--deform', 'graph'), int(opt(a, '--cols', 4)), opt(a, '--cam'), '--keep' in a)
    print('SHEET %s frames %d timing_s %s' % (out, len(frames), t))


if __name__ == '__main__':
    main(sys.argv[1:])
