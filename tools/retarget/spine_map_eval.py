"""Evaluate spine bone-map variants of the source pack (UEFN) -> CP retarget on one or more CP rigs (run inside Blender, background).

Why: the CP Spine..Spine3 heads sit at different normalized heights on player_woman_skeleton and player_man_skeleton
(compare_cp_rigs.py), so the nearest-head rule of bonemap_source_to_game_woman.json can pick different source pack spine bones.
This script solves the same source frames with the unchanged retarget_batch.Retargeter for each (rig, bonemap variant)
and measures, per frame of the corrected in-place source:
  chest_err_deg  angle between the CP Spine3 world delta (from its pose on the source pack bind pose, R_ref) and the source pack
                 spine_05 world delta (from its bind pose): how well the chest (parent of clavicles and neck) tracks;
  torso_err_deg  angle between CP Hips->Neck and source pack pelvis->neck_01;
  shape_err_deg  mean over Spine1/Spine2/Spine3 of the angle between CP Hips->joint and source pack pelvis->point on the source pack
                 spine polyline (pelvis, spine_01..05, neck_01) at the same rest arc-length fraction as the CP joint
                 on (Hips, Spine, Spine1, Spine2, Spine3, Neck): bend distribution along the spine.
Variants override only the 'src' (and Spine3 roll source) of the four CP spine entries; everything else is the bonemap.
Read-only (no clip output).

Usage: blender --background --factory-startup --python spine_map_eval.py -- --fbx-dir D --meta-dir D --list F
          --rig NAME=rig.json [--rig ...] --bonemap F [--json out.json]
"""
import argparse
import copy
import json
import math
import os
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import retarget_batch as rb  # noqa: E402

SPINE_CP = ('Spine', 'Spine1', 'Spine2', 'Spine3')
VARIANTS = {
    # CP Spine, Spine1, Spine2, Spine3 <- source pack; Spine3 roll always from spine_05 (chest bone)
    'W_01_02_03_04': ('spine_01', 'spine_02', 'spine_03', 'spine_04'),   # bonemap_source_to_game_woman.json
    'N_01_02_04_05': ('spine_01', 'spine_02', 'spine_04', 'spine_05'),   # nearest heads on player_man_skeleton
    'X_01_02_03_05': ('spine_01', 'spine_02', 'spine_03', 'spine_05'),
}
SRC_CHAIN = ('pelvis', 'spine_01', 'spine_02', 'spine_03', 'spine_04', 'spine_05', 'neck_01')
CP_CHAIN = ('Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck')


def qang(a, b):
    return math.degrees(2.0 * math.acos(min(1.0, abs(a.normalized().dot(b.normalized())))))


def vang(a, b):
    if a.length < 1e-9 or b.length < 1e-9:
        return 0.0
    return math.degrees(a.angle(b))


def variant_map(bonemap, srcs):
    m = copy.deepcopy(bonemap)
    for e in m['bones']:
        if e['cp'] in SPINE_CP:
            e['src'] = srcs[SPINE_CP.index(e['cp'])]
            if e['cp'] == 'Spine3':
                e['secondary'] = {'type': 'carry_axis', 'axis': [0, 1, 0], 'src_bone': 'spine_05'}
    return m


def chain_fracs(points):
    seg = [(points[i + 1] - points[i]).length for i in range(len(points) - 1)]
    tot = sum(seg)
    acc, out = 0.0, [0.0]
    for s in seg:
        acc += s
        out.append(acc / tot)
    return out


def point_at(points, fracs, f):
    for i in range(len(points) - 1):
        if f <= fracs[i + 1] or i == len(points) - 2:
            t = (f - fracs[i]) / max(fracs[i + 1] - fracs[i], 1e-9)
            return points[i].lerp(points[i + 1], t)
    return points[-1]


def main():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser(prog='spine_map_eval.py')
    ap.add_argument('--fbx-dir', required=True)
    ap.add_argument('--meta-dir', required=True)
    ap.add_argument('--list', required=True)
    ap.add_argument('--rig', action='append', required=True, help='NAME=path to rig json')
    ap.add_argument('--bonemap', required=True)
    ap.add_argument('--json', default=None)
    a = ap.parse_args(argv)
    rigs = {}
    for r in a.rig:
        k, p = r.split('=', 1)
        rigs[k] = rb.GameRig(p)
    base_map = json.load(open(a.bonemap, encoding='utf-8'))
    clips = rb.requested_clips(argparse.Namespace(clips='', list=a.list))
    # CP rest arc-length fractions of Spine1..Spine3 per rig
    cp_fr = {k: chain_fracs([g.wpos[g.index[n]] for n in CP_CHAIN]) for k, g in rigs.items()}
    results = {}
    for clip in clips:
        meta = json.load(open(os.path.join(a.meta_dir, clip + '.json'), encoding='utf-8'))
        n = int(meta['num_frames'])
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.import_scene.fbx(filepath=os.path.join(a.fbx_dir, clip + '.fbx'), automatic_bone_orientation=False, use_anim=True)
        src_obj = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
        sc = bpy.context.scene
        sc.render.fps, sc.render.fps_base = rb.FPS, 1.0
        S = Matrix.Diagonal((0.01, 0.01, 0.01, 1.0))
        needed = sorted(set(e['src'] for e in base_map['bones']) | set(SRC_CHAIN))
        src_rest = {}
        for bn in needed:
            loc, rot, _ = (S @ src_obj.data.bones[bn].matrix_local).decompose()
            src_rest[bn] = (loc, rot.normalized())
        frames = rb.sample_source(src_obj, needed, n)
        poses = []
        for root_loc, yaw, bones in frames:
            P_inv = Quaternion((0, 0, 1), -yaw)
            planar = Vector((root_loc.x, root_loc.y, 0.0))
            poses.append({bn: (rb.C_ROT @ (P_inv @ (p - planar)), (rb.C_ROT @ P_inv @ q).normalized()) for bn, (p, q) in bones.items()})
        results[clip] = {}
        for rk, rig in rigs.items():
            for vk, srcs in VARIANTS.items():
                rt = rb.Retargeter(rig, variant_map(base_map, srcs), src_rest)
                s3 = rig.index['Spine3']
                cp_bind_s3 = (rt.R_ref['Spine3'] @ rig.wrot[s3]).normalized()
                q05_rest = rt.src_rest['spine_05'][1]
                src_fr = chain_fracs([rt.src_rest[b][0] for b in SRC_CHAIN])
                ch, to, sh = [], [], []
                for pose in poses:
                    wpos, wrot, _, _ = rt.solve_frame(pose)
                    d_cp = wrot[s3] @ cp_bind_s3.inverted()
                    d_src = pose['spine_05'][1] @ q05_rest.inverted()
                    ch.append(qang(d_cp, d_src))
                    hp = wpos[rig.index['Hips']]
                    to.append(vang(wpos[rig.index['Neck']] - hp, pose['neck_01'][0] - pose['pelvis'][0]))
                    spts = [pose[b][0] for b in SRC_CHAIN]
                    errs = []
                    for nm in ('Spine1', 'Spine2', 'Spine3'):
                        sp = point_at(spts, src_fr, cp_fr[rk][CP_CHAIN.index(nm)])
                        errs.append(vang(wpos[rig.index[nm]] - hp, sp - pose['pelvis'][0]))
                    sh.append(sum(errs) / len(errs))
                mean = lambda v: sum(v) / len(v)
                results[clip]['%s|%s' % (rk, vk)] = {
                    'chest_err_deg_mean': round(mean(ch), 3), 'chest_err_deg_max': round(max(ch), 3),
                    'torso_err_deg_mean': round(mean(to), 3), 'torso_err_deg_max': round(max(to), 3),
                    'shape_err_deg_mean': round(mean(sh), 3), 'shape_err_deg_max': round(max(sh), 3),
                    'rest_align_deg': {k: rt.rest_align_deg[k] for k in SPINE_CP}}
        print('UTPP_SPINE_EVAL %s done' % clip, flush=True)
    # aggregate: mean over clips of per-clip means
    agg = {}
    keys = sorted({k for c in results.values() for k in c})
    for k in keys:
        rows = [results[c][k] for c in results if k in results[c]]
        agg[k] = {m: round(sum(r[m] for r in rows) / len(rows), 3) for m in
                  ('chest_err_deg_mean', 'chest_err_deg_max', 'torso_err_deg_mean', 'torso_err_deg_max', 'shape_err_deg_mean', 'shape_err_deg_max')}
    out = {'variants': VARIANTS, 'cp_rest_arc_fracs': {k: [round(x, 4) for x in v] for k, v in cp_fr.items()},
           'aggregate_mean_over_clips': agg, 'clips': results, 'licence': 'Licensed animation data'}
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1)
    for k, v in agg.items():
        print('UTPP_SPINE_AGG %-28s chest %6.2f/%6.2f torso %6.2f/%6.2f shape %6.2f/%6.2f' % (
            k, v['chest_err_deg_mean'], v['chest_err_deg_max'], v['torso_err_deg_mean'], v['torso_err_deg_max'],
            v['shape_err_deg_mean'], v['shape_err_deg_max']), flush=True)


main()
