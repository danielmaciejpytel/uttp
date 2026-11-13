"""Blender (background) tool: per-clip motion statistics for locomotion inventory (read-only on inputs).

Usage:
  blender --background --factory-startup --python anim_motion_stats.py -- <out.json> [--skeleton <skel.json>] <file|folder> ...
Inputs: *.bvh, *.fbx, *.glb (folders are scanned recursively; a .txt list file = one path per line).
For every action: frames, fps, duration, hips (pelvis) trajectory sampled at ~30 Hz in world space, and
  - leg length L (thigh head -> foot head, rest), hips height at the first sample,
  - planar speed stats in L/s and an estimate in m/s assuming L = 0.80 m (adult hip->ankle),
  - fraction of moving samples going forward / backward / left / right relative to hips facing,
  - total |yaw| turned, net heading change, max hips rise and min hips height ratio (jump / crouch hints).
--skeleton dumps the armature (bone names, parents, rest heads in Blender world, Z-up) of the FIRST file per input.
"""
import json
import math
import os
import sys

import bpy
from mathutils import Vector

ASSUMED_LEG_M = 0.80


def norm(n):
    return n.split(':')[-1].replace('_', '').replace('.', '').replace(' ', '').lower()


def find(arm, cands):
    names = {norm(b.name): b for b in arm.pose.bones}
    for c in cands:
        if c in names:
            return names[c]
    return None


HIPS = ['hips', 'pelvis', 'hip', 'root']
LTH = ['leftupleg', 'lefthip', 'thighl', 'lthigh', 'leftthigh', 'upperlegl', 'leftupperleg', 'upperlegleft']
RTH = ['rightupleg', 'righthip', 'thighr', 'rthigh', 'rightthigh', 'upperlegr', 'rightupperleg', 'upperlegright']
LFT = ['leftfoot', 'leftankle', 'footl', 'lfoot', 'ankle_l', 'anklel']
RFT = ['rightfoot', 'rightankle', 'footr', 'rfoot', 'ankler']


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_file(p):
    ext = p.lower().rsplit('.', 1)[-1]
    if ext == 'bvh':
        bpy.ops.import_anim.bvh(filepath=p, update_scene_fps=True, update_scene_duration=True)
    elif ext == 'fbx':
        bpy.ops.import_scene.fbx(filepath=p, automatic_bone_orientation=False)
    elif ext in ('glb', 'gltf'):
        bpy.ops.import_scene.gltf(filepath=p)
    arms = [o for o in bpy.data.objects if o.type == 'ARMATURE']
    return arms[0] if arms else None


def assign(arm, act):
    if arm.animation_data is None:
        arm.animation_data_create()
    arm.animation_data.action = act
    slots = getattr(act, 'slots', None)
    if slots and len(slots) and getattr(arm.animation_data, 'action_slot', None) is None:
        try:
            arm.animation_data.action_slot = slots[0]
        except Exception:
            pass


def wpos(arm, b):
    return arm.matrix_world @ b.head


def stats(arm, act, fps_scene):
    sc = bpy.context.scene
    f0, f1 = int(round(act.frame_range[0])), int(round(act.frame_range[1]))
    nfr = f1 - f0 + 1
    fps = fps_scene
    step = max(1, int(round(fps / 30.0)))
    hips, lt, rt = find(arm, HIPS), find(arm, LTH), find(arm, RTH)
    lf, rf = find(arm, LFT), find(arm, RFT)
    res = {'frames': nfr, 'fps': round(fps, 3), 'duration_s': round((nfr - 1) / fps, 3)}
    if not (hips and lt and rt):
        res['error'] = 'bones not found'
        return res
    up = Vector((0, 0, 1))
    P, F, H = [], [], []
    leg = None
    for fr in range(f0, f1 + 1, step):
        sc.frame_set(fr)
        h = wpos(arm, hips)
        l, r = wpos(arm, lt), wpos(arm, rt)
        if leg is None and lf and rf:
            leg = ((wpos(arm, lf) - l).length + (wpos(arm, rf) - r).length) / 2
        rv = r - l
        rv.z = 0
        fwd = up.cross(rv)
        fwd = fwd.normalized() if fwd.length > 1e-9 else Vector((0, 1, 0))
        P.append(h.copy())
        F.append(fwd)
        H.append(h.z)
    dt = step / fps
    # leg length from rest (more robust than first frame)
    ab = arm.data.bones
    try:
        lrest = ((ab[lf.name].head_local - ab[lt.name].head_local).length + (ab[rf.name].head_local - ab[rt.name].head_local).length) / 2
        lrest *= arm.matrix_world.to_scale()[0]
        if lrest > 1e-6:
            leg = lrest
    except Exception:
        pass
    leg = leg or 1.0
    sp, dirs, yaw_tot = [], {'fwd': 0, 'back': 0, 'left': 0, 'right': 0}, 0.0
    moving = 0
    for i in range(1, len(P)):
        v = P[i] - P[i - 1]
        v.z = 0
        s = v.length / dt / leg
        sp.append(s)
        a0 = math.atan2(F[i - 1].y, F[i - 1].x)
        a1 = math.atan2(F[i].y, F[i].x)
        d = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        yaw_tot += abs(d)
        if s > 0.25:  # ~0.2 m/s
            moving += 1
            f = F[i]
            ang = math.degrees(math.atan2(f.x * v.y - f.y * v.x, f.x * v.x + f.y * v.y))
            if abs(ang) <= 45:
                dirs['fwd'] += 1
            elif abs(ang) >= 135:
                dirs['back'] += 1
            elif ang > 0:
                dirs['left'] += 1
            else:
                dirs['right'] += 1
    srt = sorted(sp) or [0]
    q = lambda p: srt[min(len(srt) - 1, int(p * (len(srt) - 1)))]
    a_start = math.atan2(F[0].y, F[0].x)
    a_end = math.atan2(F[-1].y, F[-1].x)
    hs = sorted(H)
    hmed = hs[len(hs) // 2]
    href = H[0]
    disp = P[-1] - P[0]
    disp.z = 0
    res.update({
        'leg_len_units': round(leg, 4),
        'hips_h0_over_leg': round(href / leg, 3),
        'speed_med_ms': round(q(0.5) * ASSUMED_LEG_M, 3),
        'speed_p90_ms': round(q(0.9) * ASSUMED_LEG_M, 3),
        'speed_max_ms': round(srt[-1] * ASSUMED_LEG_M, 3),
        'moving_frac': round(moving / max(1, len(sp)), 3),
        'dir_frac': {k: round(v / max(1, moving), 3) for k, v in dirs.items()},
        'yaw_total_deg': round(math.degrees(yaw_tot), 1),
        'yaw_net_deg': round(math.degrees((a_end - a_start + math.pi) % (2 * math.pi) - math.pi), 1),
        'net_disp_m': round(disp.length / leg * ASSUMED_LEG_M, 3),
        'hips_rise_over_leg': round((hs[-1] - hmed) / leg, 3),
        'hips_min_over_med': round(hs[0] / hmed, 3) if hmed > 1e-6 else None,
        'hips_med_over_leg': round(hmed / leg, 3),
    })
    return res


def skeleton(arm):
    out = []
    mw = arm.matrix_world
    for b in arm.data.bones:
        out.append({'name': b.name, 'parent': b.parent.name if b.parent else None,
                    'head': [round(x, 5) for x in (mw @ b.head_local)],
                    'tail': [round(x, 5) for x in (mw @ b.tail_local)]})
    return {'armature': arm.name, 'scale': [round(x, 5) for x in mw.to_scale()], 'bones': out}


def collect(args):
    files = []
    for a in args:
        if os.path.isdir(a):
            for root, _, names in os.walk(a):
                for n in sorted(names):
                    if n.lower().endswith(('.bvh', '.fbx', '.glb')):
                        files.append(os.path.join(root, n))
        elif a.lower().endswith('.txt'):
            files += [l.strip() for l in open(a, encoding='utf-8') if l.strip()]
        else:
            files.append(a)
    return files


def main():
    argv = sys.argv[sys.argv.index('--') + 1:]
    out = argv.pop(0)
    skel_out = None
    if '--skeleton' in argv:
        i = argv.index('--skeleton')
        skel_out = argv[i + 1]
        del argv[i:i + 2]
    files = collect(argv)
    result, skels = {}, {}
    for p in files:
        try:
            reset()
            arm = import_file(p)
            if arm is None:
                result[p] = {'error': 'no armature'}
                continue
            fps = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base
            if skel_out and not skels:
                skels[p] = skeleton(arm)
            acts = list(bpy.data.actions)
            if p.lower().endswith('.glb') and len(acts) > 1:
                per = {}
                for act in acts:
                    assign(arm, act)
                    per[act.name] = stats(arm, act, fps)
                result[p] = {'actions': per}
            else:
                act = arm.animation_data.action if arm.animation_data and arm.animation_data.action else (acts[0] if acts else None)
                if act is None:
                    result[p] = {'error': 'no action'}
                    continue
                assign(arm, act)
                result[p] = stats(arm, act, fps)
        except Exception as e:
            result[p] = {'error': repr(e)}
        print('done', p, flush=True)
    json.dump(result, open(out, 'w', encoding='utf-8'), indent=1)
    if skel_out:
        json.dump(skels, open(skel_out, 'w', encoding='utf-8'), indent=1)
    print('files', len(files))


if __name__ == '__main__':
    main()
