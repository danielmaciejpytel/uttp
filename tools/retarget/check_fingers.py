"""Quantitative finger check on retarget preview .blend files (run inside Blender, background).

For every mapped finger/hand chain (bonemap 'bones' entries whose CP name contains Hand/InHand) it compares, per
frame, the flexion angle between a bone and its mapped child in the source vs the CP armature (world space) and the
angle between each bone's direction and the hand's direction. Reports mean/max absolute differences in degrees.

Usage: blender --background --factory-startup --python check_fingers.py -- <bonemap.json> <out.json> <preview1.blend> [...]
"""
import json
import math
import sys

import bpy

argv = sys.argv[sys.argv.index('--') + 1:]
bonemap = json.load(open(argv[0], encoding='utf-8'))
out_path, previews = argv[1], argv[2:]

pairs = [(e['cp'], e['src']) for e in bonemap['bones'] if 'Hand' in e['cp'] and e.get('src')]
src_of = dict(pairs)


def world_heads(obj):
    return {pb.name: (obj.matrix_world @ pb.head) for pb in obj.pose.bones}


def angle(a, b):
    if a.length < 1e-6 or b.length < 1e-6:
        return None
    return math.degrees(a.angle(b))


report = {}
for path in previews:
    bpy.ops.wm.open_mainfile(filepath=path)
    arms = [o for o in bpy.context.scene.objects if o.type == 'ARMATURE']
    cp = next(o for o in arms if 'Hips' in o.pose.bones)
    src = next(o for o in arms if 'pelvis' in o.pose.bones)
    scene = bpy.context.scene
    f0, f1 = int(scene.frame_start), int(scene.frame_end)
    diffs = {}
    for f in range(f0, f1 + 1):
        scene.frame_set(f)
        hc, hs = world_heads(cp), world_heads(src)
        for cpn, srcn in pairs:
            cpb = cp.pose.bones.get(cpn)
            if cpb is None or not cpb.children:
                continue
            child = next((c for c in cpb.children if c.name in src_of), None)
            if child is None or cpb.parent is None or cpb.parent.name not in src_of:
                continue
            par = cpb.parent.name
            a_cp = angle(hc[cpn] - hc[par], hc[child.name] - hc[cpn])
            a_src = angle(hs[srcn] - hs[src_of[par]], hs[src_of[child.name]] - hs[srcn])
            if a_cp is None or a_src is None:
                continue
            diffs.setdefault(cpn, []).append(abs(a_cp - a_src))
    clip = path.replace('\\', '/').rsplit('/', 1)[-1]
    report[clip] = {b: {'mean_deg': round(sum(v) / len(v), 2), 'max_deg': round(max(v), 2)} for b, v in sorted(diffs.items())}
json.dump(report, open(out_path, 'w'), indent=1)
allv = [v['mean_deg'] for r in report.values() for v in r.values()]
print('UTPP_FINGERS clips=%d joints=%d mean_of_means=%.2f max_mean=%.2f' % (len(report), len(allv), sum(allv) / max(1, len(allv)), max(allv or [0])))
