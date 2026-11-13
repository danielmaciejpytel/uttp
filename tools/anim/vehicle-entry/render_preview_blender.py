"""Blender (background) renderer for skin_cache.py output: one PNG grid (rows = clips, columns = clip-time fractions)
per camera view. Workbench engine, orthographic camera, ground plane, a thin reference bar at --ref-height metres.

blender.exe -b --factory-startup -P render_preview_blender.py -- <cache base> <out dir> [--ref-height 1.30]
"""
import json
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
base, outdir = argv[0], Path(argv[1])
ref_h = float(argv[argv.index('--ref-height') + 1]) if '--ref-height' in argv else 1.30
outdir.mkdir(parents=True, exist_ok=True)
layout = json.loads(Path(base + '.json').read_text())
data = np.load(base + '.npz')
faces = data['faces'].tolist()

scene = bpy.context.scene
for ob in list(scene.objects):
    bpy.data.objects.remove(ob, do_unlink=True)
scene.render.engine = 'BLENDER_WORKBENCH'
scene.display.shading.light = 'STUDIO'
scene.display.shading.color_type = 'OBJECT'
scene.display.shading.show_shadows = False
scene.display.shading.show_cavity = True
scene.render.film_transparent = False
scene.world.color = (0.9, 0.9, 0.9)

rows = layout['rows']
cols = len(layout['frames'])
SPX, SPZ = 1.6, 2.4  # grid spacing (horizontal in view, vertical)
colors = [(0.85, 0.45, 0.25, 1), (0.35, 0.55, 0.85, 1), (0.45, 0.75, 0.45, 1), (0.7, 0.5, 0.8, 1)]


def add_mesh(name, verts, color, loc):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts.tolist(), [], faces)
    me.update()
    for p in me.polygons:
        p.use_smooth = True
    ob = bpy.data.objects.new(name, me)
    ob.color = color
    ob.location = loc
    scene.collection.objects.link(ob)
    return ob


def add_box(name, center, size, color):
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    ob = bpy.context.active_object
    ob.name = name
    ob.scale = size
    ob.color = color
    return ob


views = {'side': Vector((-1, 0, 0)), 'front': Vector((0, 1, 0))}
for view, vdir in views.items():
    objs = []
    # horizontal axis in the image: for the side view (camera on -X looking +X) that is +Y; front view: X
    hx = (-vdir).cross(Vector((0, 0, 1))).normalized()  # image-right direction for this camera
    for r, row in enumerate(rows):
        for k in range(cols):
            loc = hx * (k * SPX) + Vector((0, 0, -r * SPZ))
            objs.append(add_mesh('%s_%d' % (row['label'], k), data['verts_%d_%d' % (r, k)], colors[r % len(colors)], loc))
            objs.append(add_box('ground_%d_%d' % (r, k), loc + Vector((0, 0, -0.01)), (1.4 if view == 'front' else 0.05, 1.4 if view == 'side' else 0.05, 0.02), (0.3, 0.3, 0.3, 1)))
            objs.append(add_box('ref_%d_%d' % (r, k), loc + Vector((0, 0, ref_h)), (1.4 if view == 'front' else 0.02, 1.4 if view == 'side' else 0.02, 0.008), (0.9, 0.1, 0.1, 1)))
    cam_data = bpy.data.cameras.new('cam_' + view)
    cam = bpy.data.objects.new('cam_' + view, cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam_data.type = 'ORTHO'
    width = cols * SPX
    height = len(rows) * SPZ
    center = hx * ((cols - 1) * SPX / 2) + Vector((0, 0, 1.0 - (len(rows) - 1) * SPZ / 2))
    cam.location = center + vdir * 10
    cam.rotation_euler = (center - cam.location).to_track_quat('-Z', 'Y').to_euler()
    cam_data.ortho_scale = max(width, height) * 1.02
    cam_data.clip_end = 100
    scene.render.resolution_x = 1800
    scene.render.resolution_y = int(1800 * height / width)
    scene.render.filepath = str(outdir / (Path(base).name + '_' + view + '.png'))
    bpy.ops.render.render(write_still=True)
    print('WROTE', scene.render.filepath)
    for ob in objs + [cam]:
        bpy.data.objects.remove(ob, do_unlink=True)
