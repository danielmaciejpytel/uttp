"""Blender (background) tool: motion signatures of FBX animations and pairwise duplicate / mirror detection.

Usage: blender --background --factory-startup --python fbx_anim_signature.py -- <folder> <out.json>
For every *.fbx under <folder> (recursive): import, sample world positions of key bones (relative to the hips at frame 0)
over the whole action, store a resampled signature (32 samples). Then compares all pairs:
  identical  : same frame count (+-1) and max position delta < 1 cm
  mirror     : identical after swapping Left/Right bones and negating X
  different  : otherwise
Output JSON: per file {frames, fps, duration, bones}, and lists of identical / mirror pairs with the max delta.
Read-only on the input files.
"""
import json
import math
import os
import sys

import bpy

KEYS = ['Hips', 'LeftFoot', 'RightFoot', 'LeftHand', 'RightHand', 'Head', 'LeftLeg', 'RightLeg']
N = 32


def find_bone(arm, key):
    for b in arm.pose.bones:
        n = b.name.split(':')[-1]
        if n == key:
            return b
    return None


def signature(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=path, automatic_bone_orientation=False)
    arms = [o for o in bpy.data.objects if o.type == 'ARMATURE']
    if not arms:
        return None
    arm = arms[0]
    act = arm.animation_data.action if arm.animation_data else None
    if act is None:
        return None
    f0, f1 = act.frame_range
    sc = bpy.context.scene
    bones = {k: find_bone(arm, k) for k in KEYS}
    frames = []
    for i in range(N):
        fr = f0 + (f1 - f0) * i / (N - 1)
        sc.frame_set(int(math.floor(fr)), subframe=fr - math.floor(fr))
        row = {}
        for k, b in bones.items():
            if b is None:
                continue
            p = arm.matrix_world @ b.head
            row[k] = (p.x, p.y, p.z)
        frames.append(row)
    h0 = frames[0].get('Hips', (0, 0, 0))
    rel = [{k: (v[0] - h0[0], v[1] - h0[1], v[2] - h0[2]) for k, v in r.items()} for r in frames]
    return {'frames': int(f1 - f0 + 1), 'fps': sc.render.fps, 'sig': rel}


def dist(a, b, mirror=False):
    worst = 0.0
    for ra, rb in zip(a, b):
        for k, va in ra.items():
            kb = k
            if mirror:
                kb = k.replace('Left', '#').replace('Right', 'Left').replace('#', 'Right')
            vb = rb.get(kb)
            if vb is None:
                continue
            if mirror:
                vb = (-vb[0], vb[1], vb[2])
            d = math.sqrt(sum((va[i] - vb[i]) ** 2 for i in range(3)))
            worst = max(worst, d)
    return worst


def main():
    argv = sys.argv[sys.argv.index('--') + 1:]
    folder, out = argv[0], argv[1]
    files = []
    for root, _, names in os.walk(folder):
        for n in sorted(names):
            if n.lower().endswith('.fbx'):
                files.append(os.path.join(root, n))
    sigs = {}
    for p in files:
        try:
            s = signature(p)
        except Exception as e:  # keep going, report
            s = {'error': str(e)}
        sigs[os.path.relpath(p, folder)] = s
    rel = list(sigs)
    ident, mirr = [], []
    for i in range(len(rel)):
        for j in range(i + 1, len(rel)):
            a, b = sigs[rel[i]], sigs[rel[j]]
            if not a or not b or 'sig' not in a or 'sig' not in b:
                continue
            if abs(a['frames'] - b['frames']) > 1:
                continue
            d = dist(a['sig'], b['sig'])
            if d < 0.01:
                ident.append([rel[i], rel[j], round(d, 4)])
                continue
            dm = dist(a['sig'], b['sig'], mirror=True)
            if dm < 0.02:
                mirr.append([rel[i], rel[j], round(dm, 4)])
    meta = {k: ({'frames': v['frames'], 'fps': v['fps']} if v and 'sig' in v else v) for k, v in sigs.items()}
    json.dump({'files': meta, 'identical': ident, 'mirror': mirr}, open(out, 'w'), indent=1)
    print('files', len(files), 'identical pairs', len(ident), 'mirror pairs', len(mirr))


if __name__ == '__main__':
    main()
