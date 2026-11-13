"""Blender (Workbench) close-up renders of dumped deformation cases: female (left) vs male (right), vertex colour = edge stretch.

blender -b --factory-startup -P render_body_deformation.py -- <dump dir> <out dir> <case>:<region>:<view>[+<view>] ...
  case   = label of measure_body_deformation.py --dump (<dump>/woman_<case>.npz and man_<case>.npz)
  region = shoulder|elbow|neck|knee (focus: the most stretched vertices of that region, on the side of the male maximum)
  view   = front|back|side (relative to the facing direction of the body, estimated from chest vs shoulder blade vertices)
Colour: grey-blue = no change vs the REF pose, yellow = 1.5x, red >= 2.5x (max of stretch and 1/compression per vertex, REF-relative);
black overlay = mesh wireframe. Output <out dir>/<case>_<region>_<view>.png (1600x900).
"""
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
dump, out = os.path.abspath(argv[0]), os.path.abspath(argv[1])
specs = [a.split(':') for a in argv[2:]]
os.makedirs(out, exist_ok=True)


def load(body, case):
    z = np.load(os.path.join(dump, '%s_%s.npz' % (body, case)))
    return {k: z[k] for k in z.files}


def facing(d):
    dom, v = d['domname'], d['verts']
    ch = np.array([('chest_front' in n) for n in dom])
    sc = np.array([('scapula' in n or 'latissimus' in n) for n in dom])
    f = v[ch].mean(0) - v[sc].mean(0)
    f[2] = 0
    return f / np.linalg.norm(f)


def colors(st):
    s = np.clip((np.log(st) / math.log(2.5)), 0, 1)  # 0 at 1.0x .. 1 at 2.5x
    base = np.array([0.55, 0.62, 0.85])
    mid = np.array([0.95, 0.85, 0.2])
    hi = np.array([0.9, 0.1, 0.05])
    c = np.where((s < 0.5)[:, None], base + (mid - base) * (s * 2)[:, None], mid + (hi - mid) * ((s - 0.5) * 2)[:, None])
    return np.c_[c, np.ones(len(c))]


def make(name, d, wire=False):
    me = bpy.data.meshes.new(name)
    me.from_pydata(d['verts'].tolist(), [], d['faces'].tolist())
    me.validate()
    me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
    col = np.tile([0.0, 0.0, 0.0, 1.0], (len(d['verts']), 1)) if wire else colors(d['stretch'])
    ca = me.color_attributes.new('c', 'FLOAT_COLOR', 'POINT')
    ca.data.foreach_set('color', col.astype(np.float32).ravel())
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    if wire:
        m = o.modifiers.new('w', 'WIREFRAME')
        m.thickness = 0.0006
        m.use_replace = True
    return o


def text(s, loc, size=0.04):
    bpy.ops.object.text_add()
    t = bpy.context.active_object
    t.data.body = s
    t.data.size = size
    t.data.align_x = 'CENTER'
    t.location = loc
    t.rotation_euler = cam_rot_euler
    return t


for case, region, views in specs:
    dw, dm = load('woman', case), load('man', case)
    for view in views.split('+'):
        bpy.ops.wm.read_factory_settings(use_empty=True)
        sc = bpy.context.scene
        sc.render.engine = 'BLENDER_WORKBENCH'
        sc.render.resolution_x, sc.render.resolution_y = 1600, 900
        sc.display.shading.light = 'STUDIO'
        sc.display.shading.color_type = 'VERTEX'
        sc.display.shading.show_cavity = False
        sc.view_settings.view_transform = 'Standard'
        sc.world = bpy.data.worlds.new('w')
        sc.world.color = (0.93, 0.93, 0.95)
        sc.render.film_transparent = False
        fm = facing(dm)
        up = Vector((0, 0, 1))
        fv = Vector(fm.tolist())
        side = fv.cross(up)  # lateral axis
        d_view = {'front': -fv, 'back': fv, 'side': side}[view]
        # focus: most stretched vertices of the region, on the side of the male maximum
        def focus(d, ref_sign=None):
            m = d['mask_' + region] & (d['stretch'] > 0)
            idx = np.where(m)[0]
            top = idx[np.argsort(-d['stretch'][idx])]
            hip = d['joints'][0]
            lat = (d['verts'][:, :3] - hip) @ np.array(side.to_tuple())
            sign = np.sign(lat[top[0]]) if ref_sign is None else ref_sign
            sel = [i for i in top if np.sign(lat[i]) == sign][:40]
            return d['verts'][sel].mean(0), sign
        fcm, sgn = focus(dm)
        fcw, _ = focus(dw, sgn)
        right = d_view.cross(up).normalized()
        rot = d_view.to_track_quat('-Z', 'Y').to_euler()
        cam_rot_euler = rot
        scale = {'shoulder': 0.9, 'elbow': 0.7, 'neck': 0.7}.get(region, 1.1)  # horizontal spacing of the two figures = scale/2 * 2 / ... (see offsets)
        for body, d, fc, sx in (('woman', dw, fcw, -0.5), ('man', dm, fcm, 0.5)):
            for wire in (False, True):
                o = make('%s_%s' % (body, 'w' if wire else 's'), d, wire)
                o.location = (-Vector(fc.tolist())) + right * (sx * scale * 0.5)
        # centre the camera laterally on the region of both figures
        lat_c = np.mean([float(np.dot(d['verts'][d['mask_' + region]].mean(0) - fc, np.array(right.to_tuple())) + sx * scale * 0.5)
                         for d, fc, sx in ((dw, fcw, -0.5), (dm, fcm, 0.5))])
        cam = bpy.data.cameras.new('cam')
        cam.type = 'ORTHO'
        cam.ortho_scale = scale * 1.55
        co = bpy.data.objects.new('cam', cam)
        sc.collection.objects.link(co)
        co.rotation_euler = rot
        co.location = -d_view * 3.0 + right * lat_c
        sc.camera = co
        for body, sx, lab in (('woman', -0.5, 'F'), ('man', 0.5, 'M')):
            mm = {}
            import json
            mm = json.loads(str((dw if body == 'woman' else dm)['metrics']))
            upc = rot.to_matrix() @ Vector((0, 1, 0))
            t = text('%s  sev %.1f' % (lab, mm[region]['sev_p999']),
                     right * (lat_c + sx * scale * 0.45) + upc * (scale * 0.3) - d_view * 1.0)
            m = bpy.data.materials.new('t')
            m.diffuse_color = (0, 0, 0, 1)
            t.data.materials.append(m)
        sc.render.filepath = os.path.join(out, '%s_%s_%s.png' % (case, region, view))
        bpy.ops.render.render(write_still=True)
        print('RENDERED', sc.render.filepath)
