"""Blender (-b --factory-startup -P) video renderer of a skin cache for the dual-wield pilot = a copy of melee
pc_film_render.py (unchanged
there) plus an optional gun cache: --guns <base> (<base>.npz rest / faces + <base>.pc2, world points per frame, the proxy boxes of both
guns built from the Weapon bone matrices by dw_film.py), dark material.

blender -b --factory-startup -P dw_film_render.py -- <skin_base> <out.mkv> <labels.json> [--res W,H] [--cam dx,dy,dz] [--follow N]
        [--title TEXT] [--color r,g,b] [--title-size S] [--title-align center] [--label-size S] [--guns <base>]
Renders straight to one video file (FFMPEG, Matroska / H.264 near-lossless, 30 fps), one video frame per cache frame, no image files.
Camera: offset --cam from the hips (game axes: the character faces +Y, +X = her right), following the hips (moving average over N frames).
"""
import glob
import json
import os
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
base, out, labels = argv[0], os.path.abspath(argv[1]), json.load(open(argv[2], encoding='utf-8'))
opts = {argv[i]: argv[i + 1] for i in range(3, len(argv) - 1) if argv[i].startswith('--')}
cam_off = Vector([float(x) for x in opts.get('--cam', '1.3,-2.0,1.3').split(',')])
resx, resy = (int(x) for x in opts.get('--res', '640,720').split(','))
follow = int(opts.get('--follow', '15'))
title = opts.get('--title', '')
col = tuple(float(x) for x in opts.get('--color', '0.30,0.50,0.85').split(',')) + (1.0,)
title_size = float(opts.get('--title-size', '0.055'))      # optional: bigger, centred title (presentation films)
title_center = opts.get('--title-align', 'left') == 'center'
label_size = float(opts.get('--label-size', '0.045'))

d = np.load(base + '.npz')
T = len(d['joints'])
assert len(labels) == T, (len(labels), T)
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene


def mat(name, rgba):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


me = bpy.data.meshes.new('skin')
me.from_pydata([tuple(v) for v in d['rest'].tolist()], [], [tuple(f) for f in d['faces'].tolist()])
me.validate()
me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
me.materials.append(mat('body', col))
o = bpy.data.objects.new('skin', me)
sc.collection.objects.link(o)
mc = o.modifiers.new('cache', 'MESH_CACHE')
mc.cache_format = 'PC2'
mc.filepath = base + '.pc2'
mc.time_mode = 'FRAME'
mc.play_mode = 'SCENE'
mc.frame_start = 0
sc.frame_start, sc.frame_end = 0, T - 1

if opts.get('--guns'):
    gb = opts['--guns']
    g = np.load(gb + '.npz')
    gm = bpy.data.meshes.new('guns')
    gm.from_pydata([tuple(v) for v in g['rest'].tolist()], [], [tuple(f) for f in g['faces'].tolist()])
    gm.validate()
    gm.materials.append(mat('gun', (0.10, 0.10, 0.11, 1)))
    go = bpy.data.objects.new('guns', gm)
    sc.collection.objects.link(go)
    gmc = go.modifiers.new('cache', 'MESH_CACHE')
    gmc.cache_format = 'PC2'
    gmc.filepath = gb + '.pc2'
    gmc.time_mode = 'FRAME'
    gmc.play_mode = 'SCENE'
    gmc.frame_start = 0

pc = np.fromfile(base + '.pc2', dtype='<f4', offset=32).reshape(-1, len(d['rest']), 3)
gz = float(pc[:, :, 2].min())
hips = d['joints'][:, 0].astype(float)
lo, hi = hips[:, :2].min(0) - 4.0, hips[:, :2].max(0) + 4.0
ctr, ext = (lo + hi) / 2, hi - lo
bpy.ops.mesh.primitive_plane_add(size=1, location=(float(ctr[0]), float(ctr[1]), gz - 0.003))
pl = bpy.context.active_object
pl.scale = (float(ext[0]), float(ext[1]), 1)
pl.data.materials.append(mat('ground', (0.80, 0.80, 0.80, 1)))
sm = mat('stripe', (0.52, 0.52, 0.52, 1))
for axis in (0, 1):                 # 0.5 m grid lines, fixed in world (aligned to multiples of 0.5 m)
    for v in np.arange(np.floor(lo[axis] * 2) / 2, hi[axis] + 1e-6, 0.5):
        loc = [float(ctr[0]), float(ctr[1]), gz]
        loc[axis] = float(v)
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
        s = bpy.context.active_object
        s.scale = (0.01, float(ext[1]), 0.002) if axis == 0 else (float(ext[0]), 0.01, 0.002)
        s.data.materials.append(sm)

cam_d = bpy.data.cameras.new('cam')
cam_d.lens = 38
cam = bpy.data.objects.new('cam', cam_d)
sc.collection.objects.link(cam)
sc.camera = cam
tgt = bpy.data.objects.new('tgt', None)
sc.collection.objects.link(tgt)
con = cam.constraints.new('TRACK_TO')
con.target = tgt
con.track_axis = 'TRACK_NEGATIVE_Z'
con.up_axis = 'UP_Y'
half_h = 1.4 * 18.0 / cam_d.lens * (1.0 if resy >= resx else resy / resx)
half_w = half_h * resx / resy
txts = {}
for k, (y, size) in {'title': (half_h - 0.035 - title_size, title_size), 'label': (-half_h + 0.04, label_size)}.items():
    bpy.ops.object.text_add()
    t = bpy.context.active_object
    t.parent = cam
    t.data.size = size
    t.data.materials.append(mat('txt_' + k, (0.05, 0.05, 0.05, 1)))
    t.location = (-half_w + 0.04, y, -1.4)
    if k == 'title' and title_center:
        t.data.align_x = 'CENTER'
        t.location = (0.0, y, -1.4)
    txts[k] = t
txts['title'].data.body = title

w = max(1, follow)
pad = np.pad(hips[:, :2], ((w // 2, w - 1 - w // 2), (0, 0)), mode='edge')
cxy = np.stack([np.convolve(pad[:, a], np.ones(w) / w, mode='valid') for a in (0, 1)], 1)
for f in range(T):
    tgt.location = (float(cxy[f, 0]), float(cxy[f, 1]), gz + 0.95)
    cam.location = (float(cxy[f, 0]) + cam_off.x, float(cxy[f, 1]) + cam_off.y, gz + cam_off.z)
    tgt.keyframe_insert('location', frame=f)
    cam.keyframe_insert('location', frame=f)
for obj in (tgt, cam):
    ad = obj.animation_data.action
    try:
        curves = ad.fcurves
    except AttributeError:          # Blender 5 layered actions
        curves = [fc for ly in ad.layers for st in ly.strips for cb in st.channelbags for fc in cb.fcurves]
    for fc in curves:
        for kp in fc.keyframe_points:
            kp.interpolation = 'CONSTANT'


def on_frame(scene, *_):
    f = min(max(scene.frame_current, 0), T - 1)
    txts['label'].data.body = labels[f]


bpy.app.handlers.frame_change_pre.append(on_frame)

sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = resx, resy
sc.render.resolution_percentage = 100
sc.render.fps, sc.render.fps_base = 30, 1.0
im = sc.render.image_settings
if hasattr(im, 'media_type'):
    try:
        im.media_type = 'VIDEO'
    except Exception:
        pass
im.file_format = 'FFMPEG'
ff = sc.render.ffmpeg
ff.format = 'MKV'
ff.codec = 'H264'
crf_items = [i.identifier for i in ff.bl_rna.properties['constant_rate_factor'].enum_items]
ff.constant_rate_factor = next(c for c in ('PERC_LOSSLESS', 'HIGH', 'MEDIUM') if c in crf_items)
stem = os.path.splitext(out)[0] + '_'
sc.render.filepath = stem
bpy.ops.render.render(animation=True)
made = sorted(glob.glob(glob.escape(stem) + '*.mkv'), key=os.path.getmtime)
if not made:
    raise SystemExit('no video written for ' + stem)
os.replace(made[-1], out)
print('RENDERED', out, T)
