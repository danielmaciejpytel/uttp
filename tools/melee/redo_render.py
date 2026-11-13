"""Blender (-b --factory-startup -P) panel renderer for the melee demo: V body caches from demo_skin.py, two views, half-frame steps.

blender -b --factory-startup -P demo_render.py -- <clip_demo_dir> [--res 640,720] [--views 3q,side] [--only 3q]
For each view (3q = behind-right 3/4 perspective like the in-game camera, side = orthographic from the character's right) and each
variant (orig, mb) renders <dir>\\<view>_<variant>\\f_####.png at half-frame steps (k = 0..2T-2 <-> clip frame k/2; the mesh cache is
linearly interpolated at the in-between steps, used only for the 0.5x slow-motion). Camera follows the hips of the rendered
variant (z fixed), ground + 0.5 m grid are fixed in world space so foot slide and hips drops stay visible.
"""
import os
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
base = os.path.abspath(argv[0])
opts = {argv[i]: argv[i + 1] for i in range(1, len(argv) - 1) if argv[i].startswith('--')}
resx, resy = (int(x) for x in opts.get('--res', '640,720').split(','))
views = opts.get('--views', '3q,side').split(',')
VARIANTS = opts.get('--variants', 'orig,mb').split(',')
COL = {'orig': (0.62, 0.62, 0.66, 1), 'mb': (0.25, 0.52, 0.92, 1), 'redo': (0.25, 0.62, 0.45, 1), 'base': (0.85, 0.68, 0.30, 1),
       'v4': (0.25, 0.52, 0.92, 1), 'v4b': (0.55, 0.35, 0.80, 1),     # v4 = v3b + MotionBricks body (same blue as 'mb'); v4b = the stronger-body experiment
       'final': (0.86, 0.36, 0.52, 1)}   # final = round F clips (round F: game hand path, torso surface, closed loops, unit tolerance)
CAMVAR = opts.get('--camvar', None)

data = {v: np.load(os.path.join(base, v + '.skin.npz')) for v in VARIANTS}
T = min(len(d['joints']) for d in data.values())
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.frame_start, sc.frame_end = 0, T - 1


def mat(name, rgba):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


objs = {}
for v in VARIANTS:
    d = data[v]
    me = bpy.data.meshes.new('skin_' + v)
    me.from_pydata([tuple(p) for p in d['rest'].tolist()], [], [tuple(f) for f in d['faces'].tolist()])
    me.validate()
    me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
    me.materials.append(mat('m_' + v, COL[v]))
    o = bpy.data.objects.new('skin_' + v, me)
    sc.collection.objects.link(o)
    mc = o.modifiers.new('cache', 'MESH_CACHE')
    mc.cache_format = 'PC2'
    mc.filepath = os.path.join(base, v + '.skin.pc2')
    mc.time_mode = 'FRAME'
    mc.play_mode = 'SCENE'
    mc.frame_start = 0
    mc.interpolation = 'LINEAR'
    objs[v] = o

gz = min(float(np.fromfile(os.path.join(base, v + '.skin.pc2'), dtype='<f4', offset=32).reshape(-1, len(data[v]['rest']), 3)[:, :, 2].min())
         for v in VARIANTS)
allh = np.concatenate([data[v]['joints'][:, 0] for v in VARIANTS])
cx, cy = float(allh[:, 0].mean()), float(allh[:, 1].mean())
bpy.ops.mesh.primitive_plane_add(size=1, location=(cx, cy, gz - 0.003))
pl = bpy.context.active_object
pl.scale = (14, 14, 1)
pl.data.materials.append(mat('ground', (0.86, 0.86, 0.86, 1)))
sm = mat('stripe', (0.55, 0.55, 0.55, 1))
for k in range(-14, 15):
    for axis in (0, 1):
        loc = [cx, cy, gz]
        loc[axis] = round((cx, cy)[axis] * 2) / 2 + k * 0.5
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
        s = bpy.context.active_object
        s.scale = (0.012, 14, 0.002) if axis == 0 else (14, 0.012, 0.002)
        s.data.materials.append(sm)

cam_d = bpy.data.cameras.new('cam')
cam = bpy.data.objects.new('cam', cam_d)
sc.collection.objects.link(cam)
sc.camera = cam
tgt = bpy.data.objects.new('tgt', None)
sc.collection.objects.link(tgt)
con = cam.constraints.new('TRACK_TO')
con.target = tgt
con.track_axis = 'TRACK_NEGATIVE_Z'
con.up_axis = 'UP_Y'
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = resx, resy
sc.render.image_settings.file_format = 'PNG'

VIEW = {'3q': dict(off=(1.3, -2.0, 1.3), ortho=None, lens=38, tz=0.95),
        'front': dict(off=(-1.3, 2.0, 1.3), ortho=None, lens=38, tz=0.95),
        'side': dict(off=(4.0, -0.3, 1.7), ortho=2.15, lens=50, tz=0.95)}
for view in views:
    vw = VIEW[view]
    if vw['ortho']:
        cam_d.type = 'ORTHO'
        cam_d.ortho_scale = vw['ortho']
    else:
        cam_d.type = 'PERSP'
        cam_d.lens = vw['lens']
    for v in VARIANTS:
        for o in objs.values():
            o.hide_render = True
        objs[v].hide_render = False
        hips = data[CAMVAR or v]['joints'][:, 0]
        out = os.path.join(base, '%s_%s' % (view, v))
        os.makedirs(out, exist_ok=True)
        for k in range(0, 2 * T - 1):
            fi, sub = divmod(k, 2)
            f1 = min(fi + 1, T - 1)
            c = Vector(hips[fi]) * (1 - 0.5 * sub) + Vector(hips[f1]) * (0.5 * sub)
            tgt.location = (c.x, c.y, gz + vw['tz'])
            cam.location = (c.x + vw['off'][0], c.y + vw['off'][1], gz + vw['off'][2])
            sc.frame_set(fi, subframe=0.5 * sub)
            bpy.context.view_layer.update()
            sc.render.filepath = os.path.join(out, 'f_%04d.png' % k)
            bpy.ops.render.render(write_still=True)
print('RENDERED', base, T, views)
