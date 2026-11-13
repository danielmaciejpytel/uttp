"""Side-by-side Workbench video of V body caches in game world space (skin_clip.py --world), PNG frames.

blender -b --factory-startup -P render_skin.py -- <frame_dir> <gap_first> <gap_last> <title> LABEL=<skin_base> [...]
  [--cam dx,dy,dz] [--frames a:b]
gap frames (clip frame numbers, 0-based) show a 'LUKA' marker. Variants are spaced 1.6 m apart along X; ground grid
every 0.5 m at the lowest vertex of the first variant; the camera follows the mean Hips position (joints[:, 0]).
Encode the frames with the project ffmpeg afterwards.
"""
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
out, g0, g1, title = argv[0], int(argv[1]), int(argv[2]), argv[3]
opts = {argv[i]: argv[i + 1] for i in range(4, len(argv) - 1) if argv[i].startswith('--')}
variants = [x.split('=', 1) for x in argv[4:] if '=' in x and not x.startswith('--')]
cam_off = Vector([float(x) for x in opts.get('--cam', '3.2,-3.6,1.5').split(',')])
COLORS = [(0.55, 0.55, 0.55, 1), (0.20, 0.45, 0.90, 1), (0.20, 0.70, 0.35, 1), (0.95, 0.55, 0.15, 1)]
SPACING = 1.6

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene


def mat(name, rgba):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


data = [np.load(p + '.npz') for _, p in variants]
T = min(len(d['joints']) for d in data)
f0, f1 = (int(x) for x in opts['--frames'].split(':')) if '--frames' in opts else (0, T - 1)
sc.frame_start, sc.frame_end = f0, min(f1, T - 1)
hips = np.mean([d['joints'][:T, 0] + [i * SPACING, 0, 0] for i, d in enumerate(data)], axis=0)
labels = []
for i, ((label, base), d) in enumerate(zip(variants, data)):
    m = mat('m_' + label, COLORS[i % len(COLORS)])
    me = bpy.data.meshes.new('skin_' + label)
    me.from_pydata([tuple(v) for v in d['rest'].tolist()], [], [tuple(f) for f in d['faces'].tolist()])
    me.validate()
    me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
    me.materials.append(m)
    o = bpy.data.objects.new('skin_' + label, me)
    sc.collection.objects.link(o)
    o.location.x = i * SPACING
    mc = o.modifiers.new('cache', 'MESH_CACHE')
    mc.cache_format = 'PC2'
    mc.filepath = base + '.pc2'
    mc.time_mode = 'FRAME'
    mc.play_mode = 'SCENE'
    mc.frame_start = 0
    bpy.ops.object.text_add()
    t = bpy.context.active_object
    t.data.body = label
    t.data.size = 0.17
    t.data.align_x = 'CENTER'
    t.data.materials.append(m)
    head = d['joints'][:T, 1] + [i * SPACING, 0, 0]
    for f in range(sc.frame_start, sc.frame_end + 1):
        t.location = Vector(head[f]) + Vector((0, 0, 0.32))
        t.keyframe_insert('location', frame=f)
    labels.append(t)

# ground: lowest vertex of the first variant's cache
pc = np.fromfile(variants[0][1] + '.pc2', dtype='<f4', offset=32).reshape(-1, len(data[0]['rest']), 3)
gz = float(pc[:, :, 2].min())
cx, cy = float(hips[:, 0].mean()), float(hips[:, 1].mean())
gm, sm = mat('ground', (0.82, 0.82, 0.82, 1)), mat('stripe', (0.5, 0.5, 0.5, 1))
bpy.ops.mesh.primitive_plane_add(size=1, location=(cx, cy, gz - 0.003))
pl = bpy.context.active_object; pl.scale = (14, 14, 1); pl.data.materials.append(gm)
for k in range(-14, 15):
    for axis in (0, 1):
        loc = [cx, cy, gz]
        loc[axis] = round((cx, cy)[axis] * 2) / 2 + k * 0.5
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
        s = bpy.context.active_object
        s.scale = (0.02, 14, 0.002) if axis == 0 else (14, 0.02, 0.002)
        s.data.materials.append(sm)

cam_d = bpy.data.cameras.new('cam'); cam_d.lens = 30
cam = bpy.data.objects.new('cam', cam_d); sc.collection.objects.link(cam); sc.camera = cam
tgt = bpy.data.objects.new('tgt', None); sc.collection.objects.link(tgt)
con = cam.constraints.new('TRACK_TO'); con.target = tgt; con.track_axis = 'TRACK_NEGATIVE_Z'; con.up_axis = 'UP_Y'
for f in range(sc.frame_start, sc.frame_end + 1):
    c = Vector(hips[f])
    tgt.location = (c.x, c.y, gz + 0.8); tgt.keyframe_insert('location', frame=f)
    cam.location = (c.x + cam_off.x, c.y + cam_off.y, gz + cam_off.z); cam.keyframe_insert('location', frame=f)
for t in labels:
    c2 = t.constraints.new('TRACK_TO'); c2.target = cam; c2.track_axis = 'TRACK_Z'; c2.up_axis = 'UP_Y'

bpy.ops.object.text_add(); info = bpy.context.active_object; info.name = 'info'
info.data.materials.append(mat('info', (0.1, 0.1, 0.1, 1)))
bpy.ops.object.text_add(); gap = bpy.context.active_object; gap.name = 'gap'
gap.data.body = 'LUKA: ruch wygenerowany'
gap.data.materials.append(mat('gap', (0.85, 0.1, 0.1, 1)))
for o, y in ((info, 0.30), (gap, -0.31)):
    o.parent = cam
    o.data.size = 0.035
    o.location = (-0.52, y, -1.0)


def text_update(scene, *_):
    info.data.body = '%s   klatka %d' % (title, scene.frame_current)
    gap.hide_render = not (g0 <= scene.frame_current <= g1)


bpy.app.handlers.frame_change_post.append(text_update)
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = 1280, 720
sc.render.fps = 30
sc.render.image_settings.file_format = 'PNG'
sc.render.filepath = out.rstrip('/\\') + '/f_####'
bpy.ops.render.render(animation=True)
print('RENDERED', out, sc.frame_start, sc.frame_end)
