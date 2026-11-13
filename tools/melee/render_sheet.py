"""Blender (-b --factory-startup -P) still renderer of a skin cache (skin_clip.py --world) from a 3/4 third-person view.

blender -b --factory-startup -P render_sheet.py -- <skin_base> <out_dir> <label0,label1,...> [--cam dx,dy,dz] [--res W,H]
One PNG per cache frame (f_000.png ...). The character faces +Y; the default camera sits behind-right and above (game axes:
+X = the character's right) and follows the hips. The label (clip frame number) is drawn in the image corner.
"""
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
base, out, labels = argv[0], argv[1], argv[2].split(',')
opts = {argv[i]: argv[i + 1] for i in range(3, len(argv) - 1) if argv[i].startswith('--')}
cam_off = Vector([float(x) for x in opts.get('--cam', '1.5,-2.3,1.35').split(',')])
resx, resy = (int(x) for x in opts.get('--res', '480,560').split(','))

d = np.load(base + '.npz')
T = len(d['joints'])
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
me.materials.append(mat('body', (0.30, 0.50, 0.85, 1)))
o = bpy.data.objects.new('skin', me)
sc.collection.objects.link(o)
mc = o.modifiers.new('cache', 'MESH_CACHE')
mc.cache_format = 'PC2'
mc.filepath = base + '.pc2'
mc.time_mode = 'FRAME'
mc.play_mode = 'SCENE'
mc.frame_start = 0
sc.frame_start, sc.frame_end = 0, T - 1

pc = np.fromfile(base + '.pc2', dtype='<f4', offset=32).reshape(-1, len(d['rest']), 3)
gz = float(pc[:, :, 2].min())
hips = d['joints'][:, 0]
bpy.ops.mesh.primitive_plane_add(size=1, location=(float(hips[:, 0].mean()), float(hips[:, 1].mean()), gz - 0.003))
pl = bpy.context.active_object
pl.scale = (12, 12, 1)
pl.data.materials.append(mat('ground', (0.80, 0.80, 0.80, 1)))
sm = mat('stripe', (0.55, 0.55, 0.55, 1))
cx, cy = float(hips[:, 0].mean()), float(hips[:, 1].mean())
for k in range(-6, 7):  # 0.5 m grid
    for axis in (0, 1):
        loc = [cx, cy, gz]
        loc[axis] = round((cx, cy)[axis] * 2) / 2 + k * 0.5
        bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
        s = bpy.context.active_object
        s.scale = (0.01, 6, 0.002) if axis == 0 else (6, 0.01, 0.002)
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
bpy.ops.object.text_add()
txt = bpy.context.active_object
txt.parent = cam
txt.data.size = 0.07
txt.data.materials.append(mat('txt', (0.05, 0.05, 0.05, 1)))
txt.location = (-0.36, 0.38, -1.4)
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = resx, resy
sc.render.image_settings.file_format = 'PNG'
for f in range(T):
    c = Vector(hips[f])
    tgt.location = (c.x, c.y, gz + 0.95)
    cam.location = (c.x + cam_off.x, c.y + cam_off.y, gz + cam_off.z)
    txt.data.body = 'frame ' + labels[f]
    sc.frame_set(f)
    bpy.context.view_layer.update()
    sc.render.filepath = '%s/f_%03d.png' % (out.rstrip('/\\'), f)
    bpy.ops.render.render(write_still=True)
print('RENDERED', out, T)
