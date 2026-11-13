"""Side-by-side comparison video of retargeted V clips (retarget_batch.py preview .blend files), Workbench, mp4.

blender -b --factory-startup -P render_compare.py -- <frame_dir> <gap_first> <gap_last> <title> LABEL=preview.blend[|skin] [...]
gap frames are preview frame numbers (shown with a 'LUKA' marker). Each figure: capsules between V joints, coloured per
variant, ground stripes every 0.5 m (foot slide is visible against them). Camera follows the mean Hips position.
|skin: base path of skin_clip.py output (<skin>.npz + <skin>.pc2, armature space of the same preview clip): the figure
is V's body mesh (MESH_CACHE modifier, parented to the preview armature object) instead of capsules.
"""
import math
import sys

import bpy
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index('--') + 1:]
out, g0, g1, title = argv[0], int(argv[1]), int(argv[2]), argv[3]
variants = [x.split('=', 1) for x in argv[4:]]
COLORS = [(0.55, 0.55, 0.55, 1), (0.20, 0.45, 0.90, 1), (0.20, 0.70, 0.35, 1), (0.95, 0.55, 0.15, 1), (0.7, 0.3, 0.8, 1)]
SEG = [('Hips', 'Spine'), ('Spine', 'Spine1'), ('Spine1', 'Spine2'), ('Spine2', 'Spine3'), ('Spine3', 'Neck'),
       ('Neck', 'Neck1'), ('Neck1', 'Head'), ('Hips', 'LeftUpLeg'), ('Hips', 'RightUpLeg')]
for S in ('Left', 'Right'):
    SEG += [('Spine3', S + 'Shoulder'), (S + 'Shoulder', S + 'Arm'), (S + 'Arm', S + 'ForeArm'), (S + 'ForeArm', S + 'Hand'),
            (S + 'Hand', S + 'HandMiddle1'), (S + 'UpLeg', S + 'Leg'), (S + 'Leg', S + 'Foot'), (S + 'Foot', S + 'ToeBase')]
SPACING = 1.3

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
arms = []
skins = {}
for i, (label, path) in enumerate(variants):
    if '|' in path:
        path, skins[label] = path.split('|', 1)
    with bpy.data.libraries.load(path) as (src, dst):
        dst.objects = ['CP_world_preview']
    a = dst.objects[0]
    a.name = 'V_' + label
    sc.collection.objects.link(a)
    off = bpy.data.objects.new('off_' + label, None)
    sc.collection.objects.link(off)
    off.location.x = i * SPACING
    a.parent = off
    arms.append((label, a, COLORS[i % len(COLORS)]))
    if i == 0:
        fr = a.animation_data.action.frame_range
        sc.frame_start, sc.frame_end = int(fr[0]), int(fr[1])


def mat(name, rgba):
    m = bpy.data.materials.new(name)
    m.diffuse_color = rgba
    return m


def capsule(name, p0, p1, r, m):
    d = p1 - p0
    L = d.length
    bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=r, depth=max(L, 1e-3))
    o = bpy.context.active_object
    o.name = name
    q = Vector((0, 0, 1)).rotation_difference(d.normalized() if L > 1e-6 else Vector((0, 0, 1)))
    o.matrix_world = Matrix.Translation((p0 + p1) / 2) @ q.to_matrix().to_4x4()
    o.data.materials.append(m)
    return o


def sphere(name, p, r, m):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=r, location=p)
    o = bpy.context.active_object
    o.name = name
    o.data.materials.append(m)
    return o


def bone_parent(o, arm, bname):
    """parent keeping the rest-pose placement (geometry built from bone rest heads)."""
    b = arm.data.bones[bname]
    pm = arm.matrix_world @ b.matrix_local @ Matrix.Translation((0, b.length, 0))
    w = o.matrix_world.copy()
    o.parent, o.parent_type, o.parent_bone = arm, 'BONE', bname
    o.matrix_parent_inverse = pm.inverted()
    o.matrix_basis = w


def skin_mesh(label, a, base, m):
    import numpy as np
    d = np.load(base + '.npz')
    me = bpy.data.meshes.new('skin_' + label)
    me.from_pydata([tuple(v) for v in d['rest'].tolist()], [], [tuple(f) for f in d['faces'].tolist()])
    me.validate()
    me.polygons.foreach_set('use_smooth', [True] * len(me.polygons))
    me.materials.append(m)
    o = bpy.data.objects.new('skin_' + label, me)
    sc.collection.objects.link(o)
    o.parent = a  # cache is in armature space; root motion lives on the armature object
    mc = o.modifiers.new('cache', 'MESH_CACHE')
    mc.cache_format = 'PC2'
    mc.filepath = base + '.pc2'
    mc.time_mode = 'FRAME'
    mc.play_mode = 'SCENE'
    mc.frame_start = sc.frame_start  # scene frame_start -> cache sample 0
    return o


labels = []
for label, a, col in arms:
    m = mat('m_' + label, col)
    if label in skins:
        skin_mesh(label, a, skins[label], m)
        bpy.ops.object.text_add()
        t = bpy.context.active_object
        t.data.body = label
        t.data.size = 0.17
        t.data.align_x = 'CENTER'
        t.data.materials.append(m)
        t.name = 'lbl_' + label
        labels.append((t, a))
        continue
    a.data.pose_position = 'REST'
    bpy.context.view_layer.update()
    head = lambda n: a.matrix_world @ a.data.bones[n].head_local
    for b0, b1 in SEG:
        if b0 not in a.data.bones or b1 not in a.data.bones:
            continue
        r = 0.045 if b0.startswith(('Hips', 'Spine')) else 0.03 if 'Up' in b0 or b0.endswith('Leg') else 0.022
        bone_parent(capsule('c_%s_%s' % (label, b1), head(b0), head(b1), r, m), a, b0)
        bone_parent(sphere('j_%s_%s' % (label, b1), head(b1), r * 1.15, m), a, b1)
    bone_parent(sphere('head_%s' % label, head('Head') + Vector((0, 0, 0.1)), 0.105, m), a, 'Head')
    a.data.pose_position = 'POSE'
    bpy.ops.object.text_add()
    t = bpy.context.active_object
    t.data.body = label
    t.data.size = 0.17
    t.data.align_x = 'CENTER'
    t.data.materials.append(m)
    t.name = 'lbl_' + label
    labels.append((t, a))

# ground stripes along the path (Y) every 0.5 m
gm = mat('ground', (0.82, 0.82, 0.82, 1)); sm = mat('stripe', (0.45, 0.45, 0.45, 1))
bpy.ops.mesh.primitive_plane_add(size=1, location=((len(arms) - 1) * SPACING / 2, 6, -0.002))
pl = bpy.context.active_object; pl.scale = (len(arms) * SPACING + 2, 40, 1); pl.data.materials.append(gm)
for k in range(-8, 60):
    bpy.ops.mesh.primitive_cube_add(size=1, location=((len(arms) - 1) * SPACING / 2, k * 0.5, 0))
    s = bpy.context.active_object; s.scale = (len(arms) * SPACING + 2, 0.02, 0.002); s.data.materials.append(sm)

# info text + gap marker
bpy.ops.object.text_add(); info = bpy.context.active_object; info.data.size = 0.13; info.name = 'info'
info.data.materials.append(mat('info', (0.1, 0.1, 0.1, 1)))
gapm = mat('gap', (0.85, 0.1, 0.1, 1))
bpy.ops.object.text_add(); gap = bpy.context.active_object; gap.data.size = 0.16; gap.data.body = 'LUKA: ruch wygenerowany'
gap.data.materials.append(gapm); gap.name = 'gap'

cam_d = bpy.data.cameras.new('cam'); cam_d.lens = 30
cam = bpy.data.objects.new('cam', cam_d); sc.collection.objects.link(cam); sc.camera = cam
tgt = bpy.data.objects.new('tgt', None); sc.collection.objects.link(tgt)
con = cam.constraints.new('TRACK_TO'); con.target = tgt; con.track_axis = 'TRACK_NEGATIVE_Z'; con.up_axis = 'UP_Y'
# screen-fixed texts: children of the camera, 1 m in front of the lens
for o, y in ((info, 0.30), (gap, -0.31)):
    o.parent = cam
    o.data.size = 0.035
    o.location = (-0.52, y, -1.0)
    o.rotation_euler = (0, 0, 0)
for t, _ in labels:
    c2 = t.constraints.new('TRACK_TO'); c2.target = cam; c2.track_axis = 'TRACK_Z'; c2.up_axis = 'UP_Y'


def text_update(scene, *_):
    info.data.body = '%s   klatka %d' % (title, scene.frame_current)
    gap.hide_render = not (g0 <= scene.frame_current <= g1)


# pass 1: evaluated positions per frame -> keyframes for the camera target and the labels (no lag at render time)
keys = []
for f in range(sc.frame_start, sc.frame_end + 1):
    sc.frame_set(f)
    dg = bpy.context.evaluated_depsgraph_get()
    hp, hd = [], []
    for _, a, _ in arms:
        ae = a.evaluated_get(dg)
        hp.append(ae.matrix_world @ ae.pose.bones['Hips'].head)
        hd.append(ae.matrix_world @ ae.pose.bones['Head'].head)
    keys.append((f, sum(hp, Vector()) / len(hp), hd))
for f, c, hd in keys:
    tgt.location = (c.x, c.y, 0.75); tgt.keyframe_insert('location', frame=f)
    cam.location = (c.x + 2.3, c.y + 4.4, 1.45); cam.keyframe_insert('location', frame=f)
    for (t, _), h in zip(labels, hd):
        t.location = (h.x, h.y, h.z + 0.32); t.keyframe_insert('location', frame=f)
bpy.app.handlers.frame_change_post.append(text_update)
sc.render.engine = 'BLENDER_WORKBENCH'
sc.display.shading.light = 'STUDIO'
sc.display.shading.color_type = 'MATERIAL'
sc.display.shading.show_shadows = False
sc.display.shading.show_cavity = False
sc.world = bpy.data.worlds.new('w') if sc.world is None else sc.world
sc.world.color = (0.95, 0.95, 0.95)
sc.render.resolution_x, sc.render.resolution_y = 1280, 720
sc.render.fps = 30
sc.render.image_settings.file_format = 'PNG'
sc.render.filepath = out.rstrip('/\\') + '/f_####'  # PNG frames; encode with the project ffmpeg afterwards
bpy.ops.render.render(animation=True)
print('RENDERED', out, sc.frame_start, sc.frame_end)
