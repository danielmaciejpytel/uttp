"""Blender (-b --factory-startup -P) renderer for the offline traversal media: a skinned V cache (skin_clip.py --world) + obstacle boxes.

blender -b --factory-startup -P trav_render_offline.py -- <skin_base> <scene.json>
scene.json (written by trav_media_offline.py, game axes = Blender Z-up metres):
  {"mode": "stills" | "video", "res": [w, h], "out": PATH (stills: folder, video: .mkv),
   "boxes":  [{"center": [x, y, z], "size": [sx, sy, sz], "yaw": rad, "color": [r, g, b]}, ...]   obstacle / ground slabs,
   "frames": [cache frame, ...]                                   (stills only),
   "views":  [{"name": "side", "ortho": 3.4 | null, "offset": [dx, dy, dz], "lens": 38}, ...]   camera = hips + offset, looks at the hips,
   "follow": N (video: moving average of the hips over N frames), "target_dz": 0.0 (look-at height above the hips)}
Workbench, studio light, material colours; no text in the image (labels, if any, are added by the caller).
"""
import json
import os
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
base, scene_path = argv[0], argv[1]
S = json.load(open(scene_path, encoding='utf-8'))
d = np.load(base + '.npz')
T = len(d['joints'])
bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene


def mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.diffuse_color = tuple(rgb) + (1.0,)
    return m


me = bpy.data.meshes.new('skin')
me.from_pydata([tuple(v) for v in d['rest'].tolist()], [], [tuple(f) for f in d['faces'].tolist()])
me.validate()
me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
me.materials.append(mat('body', S.get('body_color', [0.30, 0.50, 0.85])))
o = bpy.data.objects.new('skin', me)
sc.collection.objects.link(o)
mc = o.modifiers.new('cache', 'MESH_CACHE')
mc.cache_format = 'PC2'
mc.filepath = base + '.pc2'
mc.time_mode = 'FRAME'
mc.play_mode = 'SCENE'
mc.frame_start = 0
sc.frame_start, sc.frame_end = 0, T - 1

for i, b in enumerate(S.get('boxes', [])):
    bpy.ops.mesh.primitive_cube_add(size=1, location=tuple(b['center']))
    c = bpy.context.active_object
    c.scale = tuple(b['size'])
    c.rotation_euler = (0.0, 0.0, float(b.get('yaw', 0.0)))
    c.data.materials.append(mat('box%d' % i, b.get('color', [0.80, 0.80, 0.80])))

hips = d['joints'][:, 0].astype(float)
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
tdz = float(S.get('target_dz', 0.0))

sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = S['res']
sc.render.resolution_percentage = 100
sc.render.fps, sc.render.fps_base = 30, 1.0


def set_view(v):
    if v.get('ortho'):
        cam_d.type = 'ORTHO'
        cam_d.ortho_scale = float(v['ortho'])
    else:
        cam_d.type = 'PERSP'
        cam_d.lens = float(v.get('lens', 38))


if S['mode'] == 'stills':
    os.makedirs(S['out'], exist_ok=True)
    sc.render.image_settings.file_format = 'PNG'
    for f in S['frames']:
        f = int(min(max(f, 0), T - 1))
        sc.frame_set(f)
        h = Vector(hips[f].tolist())
        for v in S['views']:
            set_view(v)
            tgt.location = h + Vector((0.0, 0.0, tdz))
            cam.location = h + Vector(v['offset'])
            bpy.context.view_layer.update()
            sc.render.filepath = os.path.join(S['out'], 'f%04d_%s.png' % (f, v['name']))
            bpy.ops.render.render(write_still=True)
    print('RENDERED stills', S['out'], S['frames'])
else:
    v = S['views'][0]
    set_view(v)
    w = max(1, int(S.get('follow', 15)))
    pad = np.pad(hips, ((w // 2, w - 1 - w // 2), (0, 0)), mode='edge')
    sm = np.stack([np.convolve(pad[:, a], np.ones(w) / w, mode='valid') for a in (0, 1, 2)], 1)
    for f in range(T):
        h = Vector(sm[f].tolist())
        tgt.location = h + Vector((0.0, 0.0, tdz))
        cam.location = h + Vector(v['offset'])
        tgt.keyframe_insert('location', frame=f)
        cam.keyframe_insert('location', frame=f)
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
    out = os.path.abspath(S['out'])
    stem = os.path.splitext(out)[0] + '_'
    sc.render.filepath = stem
    bpy.ops.render.render(animation=True)
    import glob
    made = sorted(glob.glob(glob.escape(stem) + '*.mkv'), key=os.path.getmtime)
    if not made:
        raise SystemExit('no video written for ' + stem)
    os.replace(made[-1], out)
    print('RENDERED video', out, T)
