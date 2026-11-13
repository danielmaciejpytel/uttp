"""Print bone hierarchy/rest data of a source pack FBX and of the CP armature .blend (run inside Blender, background).

Usage: blender --background --factory-startup --python inspect_rigs.py -- <sourcepack.fbx> <cp_armature.blend> <out.json>
"""
import json
import sys

import bpy

argv = sys.argv[sys.argv.index('--') + 1:]
fbx, cp_blend, out = argv


def dump(arm_obj):
    bones = {}
    for b in arm_obj.data.bones:
        m = arm_obj.matrix_world @ b.matrix_local
        bones[b.name] = {'parent': b.parent.name if b.parent else None,
                         'head_world': [round(x, 4) for x in m.translation],
                         'y_axis_world': [round(x, 4) for x in m.col[1][:3]]}
    return bones


bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=fbx, automatic_bone_orientation=False, use_anim=True)
src = next(o for o in bpy.context.scene.objects if o.type == 'ARMATURE')
act = src.animation_data.action if src.animation_data else None
res = {'sourcepack': {'object': src.name, 'scale': list(src.scale), 'rotation': list(src.rotation_euler),
                'action': act.name if act else None, 'frame_range': list(act.frame_range) if act else None,
                'bones': dump(src)}}
bpy.ops.wm.open_mainfile(filepath=cp_blend)
cp = next(o for o in bpy.context.scene.objects if o.type == 'ARMATURE')
res['cp'] = {'object': cp.name, 'bones': dump(cp)}
json.dump(res, open(out, 'w'), indent=1)
print('UTPP_INSPECT sourcepack_bones=%d cp_bones=%d' % (len(res['sourcepack']['bones']), len(res['cp']['bones'])))
