"""Offline traversal media on the skinned V body (graph deformation): worst-frame sheet of a .utpptrav and a 1x film of clips.

  python -I trav_media_offline.py tiles --rig woman|man --trav FILE.utpptrav --gate GATE.json --work DIR [--n 2]
      picks per rig the n worst frames by anatomy excess (without shoulder_width), n by body penetration (no wrists) and n by spike
      (trav_gate_offline.py results), skins only those frames (skin_clip.py --world --deform graph --strict) and renders a side view
      (orthographic, from the character's right) and a 3/4 front view with the obstacle box -> DIR\\tiles_<rig>\\...
  python -I trav_media_offline.py sheet --work DIR --out PNG
      composes every tiles_<rig>.json in DIR into one sheet (label strip per tile: rig, montage, frame, reason, value).
  python -I trav_media_offline.py film --rig woman|man --clips DIR --gate GATE.json --names a,b,c --work DIR --out MP4
      whole retargeted rtz clips (Trajectory z := root_z), skinned, one shot per clip (camera from the right and behind, following the
      hips), concatenated at 30 fps = 1x, no text.
Obstacle boxes from the gate JSON: top H, front edge s_f, back edge s_back (hurdle depth clamped to 0.20..0.60 m, the chooser range),
platform tops open to the end of the clip; ground slabs at z = 0 before the obstacle and at the far floor behind it (none for the
vault-and-fall family).  One process per rig (base_body reads UTPP_RIG at import).
"""
import json
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root, local_path  # noqa: E402

ROOT = project_root()
HERE = os.path.dirname(os.path.abspath(__file__))
PY = os.path.join(ROOT, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
BLENDER = local_path('blender_exe') or 'blender'   # UTPP_BLENDER_EXE or "blender_exe" in local_paths.json, else blender from PATH
FFMPEG = os.path.join(ROOT, 'Tools', 'pylibs', 'imageio_ffmpeg', 'binaries', 'ffmpeg-win-x86_64-v7.1.exe')
SKIN = os.path.join(ROOT, 'Source', 'tools', 'ai_motion', 'skin_clip.py')
RENDER = os.path.join(HERE, 'trav_render_offline.py')
C_OBST, C_GROUND = [0.78, 0.62, 0.42], [0.84, 0.84, 0.84]


def opt(a, k, d=None):
    return a[a.index(k) + 1] if k in a else d


def _rig_from_argv():
    r = opt(sys.argv, '--rig')
    if r:
        os.environ['UTPP_RIG'] = r
    return r


def run(cmd, log):
    with open(log, 'w', encoding='utf-8') as f:
        p = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if p.returncode:
        raise RuntimeError('failed (%d): %s, see %s' % (p.returncode, os.path.basename(cmd[min(len(cmd) - 1, 3)]), log))


def axes(R):
    y0 = float(R[0, 3])
    u = np.array([-math.sin(y0), math.cos(y0)])
    right = np.array([math.cos(y0), math.sin(y0)])
    return y0, u, right


def boxes_for(ob, fam, R, name):
    """Obstacle + ground slabs (world boxes) from a gate obstacle record."""
    y0, u, right = axes(R)
    s = R[:, :2] @ u
    lc = float(np.mean(R[:, :2] @ right))
    H, sf, sb = ob['top_H_m'], ob['front_s_m'], ob['back_s_m']
    if sf is None:
        return [], y0, u, right
    smin, smax = float(s.min()) - 1.5, float(s.max()) + 1.5
    if fam in ('hurdle', 'vault_fall'):
        depth = 0.40 if '_V2_' in name else 0.20
        if sb is not None:
            depth = min(max(sb - sf, 0.20), 0.60)
        sb = sf + depth
    else:
        sb = smax
    out = []

    def box(a, b, z0, z1, col, width=4.0):
        mid = 0.5 * (a + b)
        c = u * mid + right * lc
        out.append(dict(center=[float(c[0]), float(c[1]), 0.5 * (z0 + z1)], size=[width, float(b - a), float(z1 - z0)], yaw=y0, color=col))
    box(sf, sb, 0.0 if not ob.get('floor_before_m') else ob['floor_before_m'], H, C_OBST, 3.0)
    box(smin, sf, -0.02, 0.0, C_GROUND, 6.0)
    fa = ob.get('floor_after_m')
    if fam in ('hurdle',) and fa is not None and sb < smax:
        box(sb, smax, fa - 0.02, fa, C_GROUND, 6.0)
    return out, y0, u, right


def skin(clip, base, rig, logdir):
    os.makedirs(os.path.dirname(base), exist_ok=True)
    cj = base + '.utppclip.json'
    json.dump(clip, open(cj, 'w'), separators=(',', ':'))
    run([PY, '-I', SKIN, cj, base, '--world', '--deform', 'graph', '--body', rig, '--strict'], os.path.join(logdir, os.path.basename(base) + '.skin.log'))


def render(base, scene, logdir):
    """scene['skin_base'] (optional) = the skin cache; base names the scene / log files."""
    sp = base + '.scene.json'
    json.dump(scene, open(sp, 'w'), indent=1)
    run([BLENDER, '-b', '--factory-startup', '-P', RENDER, '--', os.path.abspath(scene.get('skin_base', base)), os.path.abspath(sp)],
        os.path.join(logdir, os.path.basename(base) + '.render.log'))


def mkv_frames(path, w, h):
    """Decode an MKV to RGB frames in memory (no image files)."""
    p = subprocess.run([FFMPEG, '-loglevel', 'error', '-i', path, '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], capture_output=True)
    if p.returncode:
        raise RuntimeError('ffmpeg decode %s: %s' % (path, p.stderr[-300:]))
    return np.frombuffer(p.stdout, np.uint8).reshape(-1, h, w, 3)


def tiles(a):
    rig = _rig_from_argv()
    sys.path.insert(0, HERE)
    import trav_gate_offline as TG
    work, n = os.path.abspath(opt(a, '--work')), int(opt(a, '--n', '2'))
    gate = {r['name']: r for r in json.load(open(opt(a, '--gate'), encoding='utf-8'))['items']}
    pb, mons = TG.parse_trav(opt(a, '--trav'))
    mons = {m['name']: m for m in mons}
    picks = []

    def add(name, f, why, val):
        if f is None or any(p[0] == name and p[1] == f for p in picks):
            return
        picks.append((name, int(f), why, val))
    an = sorted(gate.values(), key=lambda r: -(r['anatomy']['worst_excess_wo_shoulder_width'] or 0))
    for r in an[:n]:
        feats = ','.join(list(r['anatomy']['fail_features_wo_shoulder_width'])[:2])
        add(r['name'], r['anatomy']['worst_frame_wo_shoulder_width'], 'anatomy %s' % feats, r['anatomy']['worst_excess_wo_shoulder_width'])
    pe = sorted(gate.values(), key=lambda r: -r['obstacle']['penetration'].get('body_max_cm', 0))
    for r in pe[:n]:
        w = r['obstacle']['penetration'].get('body_worst')
        if w:
            add(r['name'], int(w.split('@')[1]), 'penetration %s' % w.split('@')[0], r['obstacle']['penetration']['body_max_cm'])
    sp = sorted(gate.values(), key=lambda r: -max(r['spikes']['world_cm'], r['spikes']['rel_cm']))
    for r in sp[:n]:
        x = r['spikes']
        rel = x['rel_cm'] >= x['world_cm']
        add(r['name'], x['rel_frame'] if rel else x['world_frame'], 'spike %s' % (x['rel_joint'] if rel else x['world_joint']), max(x['world_cm'], x['rel_cm']))
    tdir = os.path.join(work, 'tiles_%s' % rig)
    logdir = os.path.join(work, 'logs')
    os.makedirs(logdir, exist_ok=True)
    rows = []
    for name in sorted(set(p[0] for p in picks)):
        m, g = mons[name], gate[name]
        fr = [p for p in picks if p[0] == name]
        ti = pb.index('Trajectory')
        frames = []
        for (_, f, _, _) in fr:
            x, y, z, yaw = (float(v) for v in m['R'][f])
            bones = [list(map(float, m['P'][f, b, 0:7])) + [1.0, 1.0, 1.0] for b in range(len(pb))]
            bones[ti] = [x, y, z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2), 1.0, 1.0, 1.0]
            frames.append(dict(bones=bones))
        clip = dict(name=name, fps=m['fps'], num_frames=len(frames), bone_names=list(pb), frames=frames)
        short = name.replace('AM_M_Neutral_Traversal_', '')
        base = os.path.join(tdir, short)
        skin(clip, base, rig, logdir)
        bx, y0, u, right = boxes_for(g['obstacle'], g['family'], m['R'], name)
        side = list(map(float, np.r_[right * 4.5, 0.0]))
        q3 = list(map(float, np.r_[u * 2.4 + right * 1.6, 0.45]))
        # one short MKV per view (cache frame k = pick k), no intermediate image files (media rule); the sheet decodes them
        for vname, view in (('side', dict(name='side', ortho=3.4, offset=side)), ('3q', dict(name='3q', lens=35, offset=q3))):
            render(base + '_' + vname, dict(mode='video', res=[480, 540], out=base + '_%s.mkv' % vname, boxes=bx, views=[view], follow=1,
                                            target_dz=-0.1, skin_base=base), logdir)
        for k, (_, f, why, val) in enumerate(fr):
            rows.append(dict(rig=rig, montage=name, frame=f, reason=why, value=val, mkv=[base + '_side.mkv', base + '_3q.mkv'], k=k))
        print('TILES', rig, short, [p[1] for p in fr])
    json.dump(rows, open(os.path.join(work, 'tiles_%s.json' % rig), 'w'), indent=1)


def sheet(a):
    from PIL import Image, ImageDraw
    work, out = os.path.abspath(opt(a, '--work')), os.path.abspath(opt(a, '--out'))
    rows = []
    for rig in ('woman', 'man'):
        p = os.path.join(work, 'tiles_%s.json' % rig)
        if os.path.isfile(p):
            rows += json.load(open(p, encoding='utf-8'))
    cache = {}

    def frame(p, k):
        if p not in cache:
            cache[p] = mkv_frames(p, 480, 540)
        return Image.fromarray(cache[p][k])
    ims = [[frame(x, r['k']) for x in r['mkv']] for r in rows]
    w, h = ims[0][0].size
    strip = 22
    per_row = 2
    nrow = (len(rows) + per_row - 1) // per_row
    S = Image.new('RGB', (per_row * 2 * w, nrow * (h + strip)), (255, 255, 255))
    dr = ImageDraw.Draw(S)
    for i, (r, pair) in enumerate(zip(rows, ims)):
        x0, y0 = (i % per_row) * 2 * w, (i // per_row) * (h + strip)
        for k, im in enumerate(pair):
            S.paste(im, (x0 + k * w, y0 + strip))
        lab = '%s V | %s f%d | %s %s' % ('Female' if r['rig'] == 'woman' else 'Male', r['montage'].replace('AM_M_Neutral_Traversal_', ''), r['frame'],
                                         r['reason'], ('%.1f cm' % r['value']) if 'anatomy' not in r['reason'] else 'excess %.2f' % r['value'])
        dr.text((x0 + 6, y0 + 5), lab, fill=(0, 0, 0))
        dr.line([(x0, y0), (x0, y0 + h + strip)], fill=(160, 160, 160))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    S.save(out)
    print('SHEET', out, len(rows), S.size)


def film(a):
    rig = _rig_from_argv()
    sys.path.insert(0, HERE)
    import trav_gate_offline as TG
    work, clips, out = os.path.abspath(opt(a, '--work')), os.path.abspath(opt(a, '--clips')), os.path.abspath(opt(a, '--out'))
    gate = {r['name']: r for r in json.load(open(opt(a, '--gate'), encoding='utf-8'))['items']}
    names = opt(a, '--names').split(',')
    fdir = os.path.join(work, 'film_%s' % rig)
    logdir = os.path.join(work, 'logs')
    os.makedirs(logdir, exist_ok=True)
    mkvs = []
    for nm in names:
        p = os.path.join(clips, nm + '.utppclip.json')
        c = json.load(open(p, encoding='utf-8'))
        ti = c['bone_names'].index('Trajectory')
        rz = c.get('root_z')
        if rz is not None:
            for f, fr in enumerate(c['frames']):
                fr['bones'][ti][2] = float(rz[f])
        _, m = TG.clip_as_montage(p)
        g = gate[nm]
        base = os.path.join(fdir, nm.replace('M_Relaxed_Traversal_', 'R_').replace('M_Neutral_Traversal_', 'N_'))
        skin(c, base, rig, logdir)
        bx, y0, u, right = boxes_for(g['obstacle'], g['family'], m['R'], nm)
        H = g['obstacle']['top_H_m']
        off = list(map(float, np.r_[right * 3.6 - u * 1.6, 0.35 + (0.4 if H > 2 else 0.0)]))
        mkv = base + '.mkv'
        render(base, dict(mode='video', res=[960, 720], out=mkv, boxes=bx, views=[dict(name='film', lens=30, offset=off)], follow=15, target_dz=-0.05), logdir)
        mkvs.append(mkv)
        print('SHOT', nm, len(c['frames']))
    inp = []
    for x in mkvs:
        inp += ['-i', x]
    fc = ''.join('[%d:v]' % i for i in range(len(mkvs))) + 'concat=n=%d:v=1:a=0,format=yuv420p[v]' % len(mkvs)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    p = subprocess.run([FFMPEG, '-y', '-loglevel', 'error'] + inp + ['-filter_complex', fc, '-map', '[v]', '-r', '30', '-c:v', 'libx264', '-crf', '18',
                                                                    '-movflags', '+faststart', out], capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError('ffmpeg: ' + p.stderr[-600:])
    print('FILM', out, names)


if __name__ == '__main__':
    cmd = sys.argv[1]
    {'tiles': tiles, 'sheet': sheet, 'film': film}[cmd](sys.argv[2:])
