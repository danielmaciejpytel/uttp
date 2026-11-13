"""Build a Blender armature that matches a Cyberpunk 2077 animRig exactly (run inside Blender).

The rig JSON is a WolvenKit 9 export (e.g. Extracted\\game\\json\\vanilla\\player_woman_skeleton.rig.json).
Each edit bone's matrix is set to the bone's model-space reference transform, so a pose bone's
matrix_basis is the rotation relative to the game's own local frame; the game local transform of a
posed bone is then rest_local @ matrix_basis (see export_game_pose in retarget scripts).
Units: metres, Z up (same as Blender). Custom properties keep the game's local rest transforms.

Usage (Blender): blender --background --python build_cp_armature.py -- <rig.json> <out.blend> [name]
"""
import json
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector

argv = sys.argv[sys.argv.index('--') + 1:]
rig_path, out_path = argv[0], argv[1]
name = argv[2] if len(argv) > 2 else 'CP_player_woman_skeleton'

rig = json.load(open(rig_path, encoding='utf-8'))['Data']['RootChunk']
names = [b['$value'] for b in rig['boneNames']]
parents = rig['boneParentIndexes']
local = []
for t in rig['boneTransforms']:
    q = t['Rotation']
    p = t['Translation']
    s = t['Scale']
    local.append((Vector((p['X'], p['Y'], p['Z'])), Quaternion((q['r'], q['i'], q['j'], q['k'])), Vector((s['X'], s['Y'], s['Z']))))

# Model-space reference pose by forward kinematics (parents precede children in CP rigs).
model = []
for i, (pos, rot, scl) in enumerate(local):
    m_local = Matrix.LocRotScale(pos, rot, scl)
    model.append(m_local if parents[i] < 0 else model[parents[i]] @ m_local)

for obj in list(bpy.data.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
arm = bpy.data.armatures.new(name)
obj = bpy.data.objects.new(name, arm)
bpy.context.scene.collection.objects.link(obj)
bpy.context.view_layer.objects.active = obj
bpy.ops.object.mode_set(mode='EDIT')
edit = {}
for i, n in enumerate(names):
    eb = arm.edit_bones.new(n)
    eb.head = (0, 0, 0)
    eb.tail = (0, 0.05, 0)  # bone length along local +Y; the matrix below places/orients it
    eb.matrix = model[i].normalized()
    edit[i] = eb
for i, p in enumerate(parents):
    if p >= 0:
        edit[i].parent = edit[p]
        edit[i].use_connect = False
bpy.ops.object.mode_set(mode='OBJECT')

for i, n in enumerate(names):
    pb = obj.pose.bones[n]
    pos, rot, scl = local[i]
    pb['cp_index'] = i
    pb['cp_parent'] = parents[i]
    pb['cp_rest_pos'] = list(pos)
    pb['cp_rest_rot_wxyz'] = list(rot)
obj['cp_rig_source'] = rig_path
obj['cp_bone_count'] = len(names)

bpy.ops.wm.save_as_mainfile(filepath=out_path)
print(f'UTPP_ARMATURE bones={len(names)} saved={out_path}')
