"""Write source pack bone world matrices (npz W [T,B,4,4] cm, from skeleton_bridge.py inv) onto the template FBX skeleton.

blender -b --factory-startup -P mannequin_write.py -- <template.fbx> <world.npz> <out.fbx>
Root motion goes on the armature object (as in the UE export); a rest-pose frame before the clip becomes the bind
pose; keys shifted by -1 frame to compensate the FBX writer/reader offset (same writer as the ActionForge test).
"""
import sys

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

argv = sys.argv[sys.argv.index('--') + 1:]
tmpl, wpath, out = argv
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=tmpl)
O = [o for o in bpy.data.objects if o.type == 'ARMATURE'][0]
for o in list(bpy.data.objects):
    if o.type == 'MESH':
        bpy.data.objects.remove(o)
f0 = int(round(O.animation_data.action.frame_range[0]))
d = np.load(wpath)
names = [str(x) for x in d['names']]
W = d['W']
KO = -1
S100, S001 = Matrix.Scale(100.0, 4), Matrix.Scale(0.01, 4)
sc = bpy.context.scene
sc.render.fps, sc.render.fps_base = 30, 1.0
act = bpy.data.actions.new('ai_motion')
O.animation_data.action = act
O.rotation_mode = 'QUATERNION'
rootb = O.data.bones[0]
frames = list(range(f0, f0 + len(W)))
for fi, f in enumerate(frames):
    T = {n: Matrix(W[fi, i].tolist()) for i, n in enumerate(names)}
    loc_r, rot_r, _ = T[rootb.name].decompose()
    objw = S001 @ Matrix.LocRotScale(loc_r, rot_r, Vector((1, 1, 1))) @ rootb.matrix_local.inverted_safe()
    O.location, O.rotation_quaternion, O.scale = objw.decompose()
    for p in ('location', 'rotation_quaternion', 'scale'):
        O.keyframe_insert(p, frame=f + KO)
    inv = (S100 @ objw).inverted_safe()
    T = {n: inv @ m for n, m in T.items()}
    for pb in O.pose.bones:
        b = pb.bone
        if pb.parent:
            basis = (b.parent.matrix_local.inverted_safe() @ b.matrix_local).inverted_safe() @ T[pb.parent.name].inverted_safe() @ T[pb.name]
        else:
            basis = b.matrix_local.inverted_safe() @ T[pb.name]
        loc, rot, scl = basis.decompose()
        pb.rotation_mode = 'QUATERNION'
        pb.location, pb.rotation_quaternion, pb.scale = loc, rot, scl
        for p in ('location', 'rotation_quaternion', 'scale'):
            pb.keyframe_insert(p, frame=f + KO)
fb = frames[0] - 1 + KO
O.location, O.rotation_quaternion, O.scale = Vector((0, 0, 0)), Quaternion(), Vector((0.01, 0.01, 0.01))
for p in ('location', 'rotation_quaternion', 'scale'):
    O.keyframe_insert(p, frame=fb)
for pb in O.pose.bones:
    pb.location, pb.rotation_quaternion, pb.scale = Vector((0, 0, 0)), Quaternion(), Vector((1, 1, 1))
    for p in ('location', 'rotation_quaternion', 'scale'):
        pb.keyframe_insert(p, frame=fb)
sc.frame_start, sc.frame_end = frames[0] + KO, frames[-1] + KO
sc.frame_set(fb)
bpy.ops.object.select_all(action='DESELECT'); O.select_set(True); bpy.context.view_layer.objects.active = O
bpy.ops.export_scene.fbx(filepath=out, use_selection=True, object_types={'ARMATURE'}, add_leaf_bones=False,
                         bake_anim=True, bake_anim_use_all_actions=False, bake_anim_use_nla_strips=False,
                         bake_anim_force_startend_keying=True, bake_anim_simplify_factor=0.0)
print('WROTE', out, 'frames', len(W))
