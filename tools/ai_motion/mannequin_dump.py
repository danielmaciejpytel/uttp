"""Dump a source pack (UEFN mannequin) FBX clip to npz: per-frame bone world matrices in cm, Blender world (Z up).

blender -b --factory-startup -P mannequin_dump.py -- <in.fbx> <out.npz>
npz: names [B], parents [B] (-1 root), W [T,B,4,4] world cm, rest [B,4,4] bind world cm (object at identity), fps
The armature object carries the root motion (object loc/rot, scale 0.01), as in the UE export.
"""
import sys

import bpy
import numpy as np
from mathutils import Matrix

argv = sys.argv[sys.argv.index('--') + 1:]
src, out = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=src)
arm = [o for o in bpy.data.objects if o.type == 'ARMATURE'][0]
sc = bpy.context.scene
sc.render.fps, sc.render.fps_base = 30, 1.0
f0, f1 = (int(round(x)) for x in arm.animation_data.action.frame_range)
names = [b.name for b in arm.data.bones]
idx = {n: i for i, n in enumerate(names)}
parents = np.array([idx[b.parent.name] if b.parent else -1 for b in arm.data.bones])
S100 = Matrix.Scale(100.0, 4)
rest = np.array([np.array(b.matrix_local) for b in arm.data.bones])  # armature space, cm (bones are in cm inside the 0.01 object)
W = []
for f in range(f0, f1 + 1):
    sc.frame_set(f)
    M = S100 @ arm.matrix_world
    W.append([np.array(M @ pb.matrix) for pb in (arm.pose.bones[n] for n in names)])
np.savez(out, names=np.array(names), parents=parents, W=np.array(W, dtype=np.float64), rest=rest, fps=30, frames=np.arange(f0, f1 + 1))
print('DUMP', src, 'frames', f1 - f0 + 1, 'bones', len(names))
