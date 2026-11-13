"""Dump UE (source skeleton) bone tracks needed to validate the AnimModifier recompute of mmbake_motion.

Run inside Blender (background):
  blender --background --factory-startup --python dump_ue_tracks.py -- --fbx-dir D --meta-dir D --out F
      [--clips a,b,...] [--list FILE] [--bones root,ball_l,ball_r]

For every clip: imports <fbx-dir>/<clip>.fbx exactly as Source/tools/retarget/retarget_batch.py does
(automatic_bone_orientation off, scene fps 30) and samples num_frames (from <meta-dir>/<clip>.json) frames.
Per frame and bone it writes the source world transform O @ pose_bone.matrix (O = armature object world, which
carries the root motion of the UE root track) as location (Blender units) + quaternion wxyz, plus the armature
object transform itself ("__object__" = the UE root bone: the FBX importer turns the root node
into the armature object). Output: one JSON {format, clips: {name: {num_frames, units, bones: {bone:
[[x, y, z, qw, qx, qy, qz], ...]}}}}. The output is source-pack-derived data: write it
under Extracted/, not under Source/.
"""
import argparse
import json
import os
import sys
import time

import bpy


def parse_args():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser(prog='dump_ue_tracks.py')
    ap.add_argument('--fbx-dir', required=True)
    ap.add_argument('--meta-dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--clips', default='')
    ap.add_argument('--list', default='')
    ap.add_argument('--bones', default='ball_l,ball_r,pelvis')
    return ap.parse_args(argv)


def clip_names(a):
    names = [c.strip() for c in a.clips.split(',') if c.strip()]
    if a.list:
        with open(a.list, encoding='utf-8') as f:
            names += [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith('#')]
    if not names:
        names = sorted(os.path.splitext(f)[0] for f in os.listdir(a.fbx_dir) if f.lower().endswith('.fbx'))
    out = []
    for n in names:
        if n not in out:
            out.append(n)
    return out


def tr(m):
    loc, rot, _ = m.decompose()
    rot = rot.normalized()
    return [round(loc.x, 7), round(loc.y, 7), round(loc.z, 7), round(rot.w, 8), round(rot.x, 8), round(rot.y, 8),
            round(rot.z, 8)]


def main():
    a = parse_args()
    bones = [b for b in a.bones.split(',') if b]
    res = {'format': 'utpp-ue-tracks', 'version': 1,
           'licence': 'Licensed animation data',
           'generator': 'Source/tools/mmbake/dump_ue_tracks.py (Blender %s)' % bpy.app.version_string,
           'method': 'FBX import as retarget_batch.py; world = armature object world @ pose_bone.matrix at frame f+1',
           'clips': {}, 'errors': {}}
    t0 = time.time()
    names = clip_names(a)
    for i, cn in enumerate(names):
        try:
            meta = json.load(open(os.path.join(a.meta_dir, cn + '.json'), encoding='utf-8'))
            n = int(meta['num_frames'])
            bpy.ops.wm.read_factory_settings(use_empty=True)
            bpy.ops.import_scene.fbx(filepath=os.path.join(a.fbx_dir, cn + '.fbx'), automatic_bone_orientation=False,
                                     use_anim=True)
            obj = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
            sc = bpy.context.scene
            sc.render.fps, sc.render.fps_base = 30, 1.0
            out = {b: [] for b in bones}
            out['__object__'] = []
            for f in range(n):
                sc.frame_set(f + 1)
                O = obj.matrix_world.copy()
                out['__object__'].append(tr(O))
                for b in bones:
                    out[b].append(tr(O @ obj.pose.bones[b].matrix))
            res['clips'][cn] = {'num_frames': n, 'object_scale': list(obj.matrix_world.to_scale()), 'bones': out}
        except Exception as e:  # noqa: BLE001  (reported, never silent)
            res['errors'][cn] = repr(e)[:300]
        if (i + 1) % 50 == 0:
            print('UTPP_DUMP %d/%d %.0fs' % (i + 1, len(names), time.time() - t0), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(res, f, separators=(',', ':'))
    print('UTPP_DUMP_DONE clips=%d errors=%d seconds=%.0f -> %s' % (len(res['clips']), len(res['errors']),
                                                                    time.time() - t0, a.out), flush=True)


if __name__ == '__main__':
    main()
