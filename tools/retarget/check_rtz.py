"""Verify a retarget_batch.py --rtz output dir against the source FBX root track, the UE meta root track ends and the
non-RTZ output of the same clips (run inside Blender, background; spec RTZ.txt).

Usage:
  blender --background --factory-startup --python check_rtz.py -- --rtz-dir D --ref-dir D --fbx-dir D --meta-dir D
      [--list F] [--groups F] [--rig-json F] [--vert-tol 0.25] [--root-tol 0.01] [--json F] [--txt F]

Per clip:
  root_z: length == num_frames; root_z / retarget.root_z.scale against the FBX root (armature object) Z read from the
    action keyframes (location[2], frame f+1, transformed by the object's parent; independent of the frame_set/
    matrix_world path used by the retarget, which is reported only as a cross-check) and against the UE meta
    root_track_start / root_track_end Z (cm); max error <= --root-tol (m, source units);
  identity with the non-RTZ clip (--ref-dir): Trajectory (horizontal root motion, yaw) and Root bit-identical, curves,
    notifies and every other top-level key equal, retarget equal except retarget.root_z, Hips X/Y and rotation and
    all other bones equal (max abs diff reported), Hips Z(ref) - Hips Z(rtz) == root_z (max abs error);
  Hips: max |Hips Z - CP rest Hips Z| (Hips parent = Root = identity) against --vert-tol, split into above / below rest;
    the source pelvis height above its root (min/max) is reported to show where a remaining deviation comes from.
Only reads the inputs; writes --json / --txt.
"""
import argparse
import json
import math
import os
import sys

import bpy
from mathutils import Vector


def parse_args():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    ap = argparse.ArgumentParser(prog='check_rtz.py')
    ap.add_argument('--rtz-dir', required=True)
    ap.add_argument('--ref-dir', required=True)
    ap.add_argument('--fbx-dir', required=True)
    ap.add_argument('--meta-dir', required=True)
    ap.add_argument('--list', default='')
    ap.add_argument('--groups', default='', help='JSON with clips[].clip / clips[].rtz_group (sourcepack-rtz-clips.json)')
    ap.add_argument('--rig-json', default=os.path.join(os.environ.get('UTPP_PROJECT_ROOT') or os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')), 'Extracted', 'game', 'json', 'vanilla', 'player_woman_skeleton.rig.json'))
    ap.add_argument('--vert-tol', type=float, default=0.25)
    ap.add_argument('--root-tol', type=float, default=0.01)
    ap.add_argument('--json', default='')
    ap.add_argument('--txt', default='')
    return ap.parse_args(argv)


def load(p):
    with open(p, encoding='utf-8') as fh:
        return json.load(fh)


def action_fcurves(action):
    fcs = []
    if getattr(action, 'layers', None) is not None and len(action.layers):
        for layer in action.layers:
            for strip in layer.strips:
                for cb in strip.channelbags:
                    fcs.extend(cb.fcurves)
    elif hasattr(action, 'fcurves'):
        fcs.extend(action.fcurves)
    return fcs


def fbx_root(path, n):
    """Root Z per frame from the armature object's location keys (+ parent), the matrix_world Z at frame f+1 and the
    pelvis world Z (both via frame_set), metres."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=path, automatic_bone_orientation=False, use_anim=True)
    obj = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
    act = obj.animation_data.action
    fcs = {(fc.data_path, fc.array_index): fc for fc in action_fcurves(act)}
    loc = []
    for ax in range(3):
        fc = fcs.get(('location', ax))
        if fc is None:
            loc.append([obj.location[ax]] * n)
            continue
        kp = fc.keyframe_points
        vals = []
        for f in range(n):
            if f < len(kp) and abs(kp[f].co[0] - (f + 1)) < 1e-6:
                vals.append(kp[f].co[1])
            else:
                vals.append(fc.evaluate(f + 1))
        loc.append(vals)
    P = obj.parent.matrix_world @ obj.matrix_parent_inverse if obj.parent else None
    key_z = []
    for f in range(n):
        v = Vector((loc[0][f], loc[1][f], loc[2][f]))
        key_z.append((P @ v).z if P is not None else v.z)
    scene = bpy.context.scene
    eval_z, pelvis_rel = [], []
    for f in range(n):
        scene.frame_set(f + 1)
        O = obj.matrix_world
        eval_z.append(O.translation.z)
        pelvis_rel.append((O @ obj.pose.bones['pelvis'].matrix).translation.z - O.translation.z)
    rest_pelvis = (obj.matrix_world.to_3x3() @ obj.data.bones['pelvis'].head_local).z
    return key_z, eval_z, pelvis_rel, rest_pelvis, obj.parent is not None


def main():
    a = parse_args()
    rc = load(a.rig_json)['Data']['RootChunk']
    names = [b['$value'] for b in rc['boneNames']]
    hi = names.index('Hips')
    assert rc['boneParentIndexes'][hi] == 0
    rest_z = rc['boneTransforms'][hi]['Translation']['Z']
    groups = {}
    if a.groups:
        groups = {c['clip']: c.get('rtz_group', '?') for c in load(a.groups)['clips']}
    if a.list:
        clips = []
        with open(a.list, encoding='utf-8-sig') as fh:
            for line in fh:
                line = line.split('#', 1)[0].strip()
                if line:
                    clips.append(line)
    else:
        clips = sorted(f[:-len('.utppclip.json')] for f in os.listdir(a.rtz_dir) if f.endswith('.utppclip.json'))
    rows = []
    for clip in clips:
        row = {'clip': clip, 'group': groups.get(clip, '?')}
        try:
            R = load(os.path.join(a.rtz_dir, clip + '.utppclip.json'))
            O = load(os.path.join(a.ref_dir, clip + '.utppclip.json'))
            meta = load(os.path.join(a.meta_dir, clip + '.json'))
            n = R['num_frames']
            rz = R['root_z']
            spec = R['retarget']['root_z']
            scale = spec['scale']
            row['frames'] = n
            row['root_z_len_ok'] = len(rz) == n
            row['scale'] = scale
            row['scale_eq_hip_ratio'] = scale == R['retarget']['hip_ratio']
            key_z, eval_z, pel_rel, rest_pel, has_parent = fbx_root(os.path.join(a.fbx_dir, clip + '.fbx'), n)
            row['fbx_root_parented'] = has_parent
            e_key = max(abs(rz[f] / scale - key_z[f]) for f in range(n))
            e_eval = max(abs(rz[f] / scale - eval_z[f]) for f in range(n))
            e_key_cp = max(abs(rz[f] - key_z[f] * scale) for f in range(n))
            ms = meta['root_track_start']['translation'][2] / 100.0
            me = meta['root_track_end']['translation'][2] / 100.0
            e_meta = max(abs(rz[0] / scale - ms), abs(rz[-1] / scale - me))
            row.update({'root_err_vs_fbx_keys_m': e_key, 'root_err_vs_fbx_keys_cp_m': e_key_cp,
                        'root_err_vs_fbx_eval_m': e_eval, 'root_err_vs_meta_ends_m': e_meta,
                        'meta_root_z_start_end_m': [ms, me],
                        'src_root_z_min_max_m': [round(min(key_z), 5), round(max(key_z), 5)],
                        'src_pelvis_above_root_min_max_m': [round(min(pel_rel), 4), round(max(pel_rel), 4)],
                        'src_pelvis_rest_m': round(rest_pel, 4)})
            row['root_ok'] = row['root_z_len_ok'] and max(e_key, e_meta) <= a.root_tol
            # identity with the non-RTZ output
            fr_r, fr_o = R['frames'], O['frames']
            traj_diff = sum(1 for f in range(n) if fr_r[f]['bones'][1] != fr_o[f]['bones'][1])
            root_diff = sum(1 for f in range(n) if fr_r[f]['bones'][0] != fr_o[f]['bones'][0])
            hips_z_err, hips_other, other = 0.0, 0.0, 0.0
            for f in range(n):
                br, bo = fr_r[f]['bones'], fr_o[f]['bones']
                hips_z_err = max(hips_z_err, abs((bo[hi][2] - br[hi][2]) - rz[f]))
                hips_other = max(hips_other, max(abs(x - y) for k, (x, y) in enumerate(zip(br[hi], bo[hi])) if k != 2))
                for i in range(len(br)):
                    if i in (0, 1, hi):
                        continue
                    other = max(other, max(abs(x - y) for x, y in zip(br[i], bo[i])))
            keys_r = set(R) - {'root_z'}
            top_equal = keys_r == set(O) and all(R[k] == O[k] for k in keys_r if k not in ('frames', 'retarget'))
            ret_r = {k: v for k, v in R['retarget'].items() if k != 'root_z'}
            row.update({'trajectory_frames_differ': traj_diff, 'root_bone_frames_differ': root_diff,
                        'curves_equal': R['curves'] == O['curves'], 'curve_count': len(R['curves']),
                        'notifies_equal': R['notifies'] == O['notifies'], 'top_level_equal': top_equal,
                        'retarget_equal_except_root_z': ret_r == O['retarget'],
                        'hips_z_identity_err_m': hips_z_err, 'hips_xy_rot_max_diff': hips_other,
                        'other_bones_max_diff': other})
            row['identity_ok'] = (traj_diff == 0 and root_diff == 0 and row['curves_equal'] and row['notifies_equal']
                                  and top_equal and row['retarget_equal_except_root_z'] and hips_z_err <= 1e-5
                                  and hips_other <= 1e-5 and other <= 1e-5)
            dev = [fr['bones'][hi][2] - rest_z for fr in fr_r]
            dev_o = [fr['bones'][hi][2] - rest_z for fr in fr_o]
            row.update({'hips_dev_min_m': round(min(dev), 4), 'hips_dev_max_m': round(max(dev), 4),
                        'hips_dev_abs_max_m': round(max(abs(d) for d in dev), 4),
                        'hips_dev_abs_max_ref_m': round(max(abs(d) for d in dev_o), 4)})
            row['hips_within_tol'] = row['hips_dev_abs_max_m'] <= a.vert_tol
            row['hips_above_within_tol'] = row['hips_dev_max_m'] <= a.vert_tol
            row['status'] = 'ok'
        except Exception as ex:  # keep going, report
            row['status'] = 'failed'
            row['error'] = repr(ex)
        rows.append(row)
        print('UTPP_CHECK_RTZ %s %s root=%s ident=%s hips=%s' % (clip, row['status'], row.get('root_ok'),
                                                                row.get('identity_ok'), row.get('hips_dev_abs_max_m')), flush=True)
    ok = [r for r in rows if r['status'] == 'ok']
    summ = {'clips': len(rows), 'ok': len(ok), 'failed': [r['clip'] for r in rows if r['status'] != 'ok'],
            'vert_tol_m': a.vert_tol, 'root_tol_m': a.root_tol, 'rest_hips_z_m': rest_z, 'groups': {}}
    for g in sorted(set(r['group'] for r in ok)) + ['ALL']:
        gr = [r for r in ok if g == 'ALL' or r['group'] == g]
        summ['groups'][g] = {
            'clips': len(gr),
            'root_ok': sum(r['root_ok'] for r in gr),
            'root_err_vs_fbx_keys_max_m': max(r['root_err_vs_fbx_keys_m'] for r in gr),
            'root_err_vs_fbx_eval_max_m': max(r['root_err_vs_fbx_eval_m'] for r in gr),
            'root_err_vs_meta_ends_max_m': max(r['root_err_vs_meta_ends_m'] for r in gr),
            'identity_ok': sum(r['identity_ok'] for r in gr),
            'trajectory_frames_differ_total': sum(r['trajectory_frames_differ'] for r in gr),
            'curves_equal': sum(r['curves_equal'] for r in gr),
            'notifies_equal': sum(r['notifies_equal'] for r in gr),
            'hips_z_identity_err_max_m': max(r['hips_z_identity_err_m'] for r in gr),
            'other_bones_max_diff': max(max(r['other_bones_max_diff'], r['hips_xy_rot_max_diff']) for r in gr),
            'hips_within_tol': sum(r['hips_within_tol'] for r in gr),
            'hips_above_within_tol': sum(r['hips_above_within_tol'] for r in gr),
            'hips_dev_min_m': min(r['hips_dev_min_m'] for r in gr),
            'hips_dev_max_m': max(r['hips_dev_max_m'] for r in gr),
            'hips_dev_abs_max_ref_min_max_m': [min(r['hips_dev_abs_max_ref_m'] for r in gr),
                                               max(r['hips_dev_abs_max_ref_m'] for r in gr)],
            'hips_over_tol': [r['clip'] for r in gr if not r['hips_within_tol']],
        }
    out = {'summary': summ, 'clips': rows}
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1)
    lines = ['check_rtz.py: ' + json.dumps({k: v for k, v in summ.items() if k != 'groups'})]
    for g, s in summ['groups'].items():
        lines.append('%s: %s' % (g, json.dumps({k: v for k, v in s.items() if k != 'hips_over_tol'})))
    lines.append('clip | group | root err fbx keys / eval / meta ends (m) | identity | hips dev min/max (ref |max|) | src pelvis above root min/max (rest)')
    for r in rows:
        if r['status'] != 'ok':
            lines.append('%s | FAILED %s' % (r['clip'], r.get('error')))
            continue
        lines.append('%s | %s | %.2e / %.2e / %.2e | %s | %+.3f / %+.3f (%.3f) | %.3f / %.3f (%.3f)' % (
            r['clip'], r['group'], r['root_err_vs_fbx_keys_m'], r['root_err_vs_fbx_eval_m'], r['root_err_vs_meta_ends_m'],
            r['identity_ok'], r['hips_dev_min_m'], r['hips_dev_max_m'], r['hips_dev_abs_max_ref_m'],
            r['src_pelvis_above_root_min_max_m'][0], r['src_pelvis_above_root_min_max_m'][1], r['src_pelvis_rest_m']))
    txt = '\n'.join(lines) + '\n'
    if a.txt:
        with open(a.txt, 'w', encoding='utf-8') as fh:
            fh.write(txt)
    print(txt)
    print('UTPP_CHECK_RTZ_DONE ok=%d failed=%d' % (len(ok), len(rows) - len(ok)), flush=True)


if __name__ == '__main__':
    main()
