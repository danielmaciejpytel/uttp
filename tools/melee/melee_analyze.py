"""Per-clip metrics of a vanilla melee clip, from its dump (melee_dump.py). Read-only on the data.

  python -I melee_analyze.py <set> <clip> [--out FILE] [--sheet] [--sheet-frames N] [--quiet]
  python -I melee_analyze.py --group <group id> [--sheet]          every clip of a manifest group
  python -I melee_analyze.py --set <set> --quiet                   every clip of a set (batch check)
Default output: Build\\work\\melee-anim-20261107\\analysis\\metrics\\<set>\\<clip>.json (also printed as one summary line).
--sheet: contact sheet PNG of the skinned V body (melee_sheet.py) -> sheets\\<set>\\<clip>.png (N evenly spaced frames, default 12,
plus the spike frames).

Axes: game axes, z up, the character faces +Y, left = -X. Angles in degrees, lengths in cm unless a key says _m.
Additive clips (add_*, AdditiveWithoutFirstFrame): flagged, pose metrics skipped (their tracks are offsets, not poses).
Conventions used below (from the rig: T-pose reference, ForeArm/Leg are pure hinges about local Z):
  elbow_flex / knee_flex  = -rotvec_z of the rest-relative local rotation (0 = straight, ~145 = fully bent, NEGATIVE = hyperextension)
  wrist_bend              = angle between the forearm direction (ForeArm->Hand) and the hand direction (Hand->HandMiddle1)
  shoulder_elev           = angle between the upper arm direction (Arm->ForeArm) and 'down' of the torso (Hips->Neck inverted)
  torso_twist             = yaw of Spine3 relative to Hips (rest-relative world rotations of the bones' +X), + = counter-clockwise seen from above
  neck_stretch_pct        = |Spine3->Head| relative to the rig reference pose - 1 (the crouch 'long neck' artefact)
  spikes                  = frames where one joint's local rotation jumps by >= SPIKE_MIN deg in one frame AND >= SPIKE_RATIO x the
                            larger neighbouring step (an isolated snap, not a fast swing)
shoulders               = joint width along the hips' right axis: SHOULDERS_NARROW / CLAVICLES_CROSSED (the FPP rigs squeeze or cross the shoulder girdle)
severity: low / med / high -- automatic hints for the reviewer, not verdicts; look at the sheet before accepting any of them.
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import contact as K  # noqa: E402
import melee_common as MC  # noqa: E402

SPIKE_MIN = 10.0     # deg in one frame
SPIKE_RATIO = 2.5
ANIM_DEG = 2.0       # a bone counts as animated above this deviation from its frame-0 local rotation
ANIM_CM = 1.5
NEAR_FACE_CM = 25.0
BODY_BONES = MC.SPINE + MC.HEADNECK + MC.L_ARM + MC.R_ARM + MC.L_LEG + MC.R_LEG


def runs(mask):
    out, s = [], None
    for i, m in enumerate(mask):
        if m and s is None:
            s = i
        elif not m and s is not None:
            out.append([s, i - 1])
            s = None
    if s is not None:
        out.append([s, len(mask) - 1])
    return out


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def horizontal_right(Pl, Pr):
    r = Pr - Pl
    r[..., 2] = 0
    return unit(r)


def yaw_of(right):
    fwd = np.stack([-right[:, 1], right[:, 0], np.zeros(len(right))], 1)  # up x right
    return np.degrees(np.arctan2(fwd[:, 1], fwd[:, 0]))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def rot_rel(d, ix, b, rest_loc):
    """rest-relative local rotation (rotvec deg, rest-frame axes) of bone b over time [T,3]."""
    i = ix[b]
    rr = R.from_quat(d['rest_lr'][i])
    return np.degrees((rr.inv() * R.from_quat(d['lr'][:, i].astype(float))).as_rotvec())


def ang_speed(d, ix, bones):
    """deg per frame of the local rotation between consecutive frames, [T,B] (row 0 = 0)."""
    out = np.zeros((d['lr'].shape[0], len(bones)))
    for k, b in enumerate(bones):
        q = d['lr'][:, ix[b]].astype(float)
        dot = np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)
        out[1:, k] = np.degrees(2 * np.arccos(dot))
    return out


def find_spikes(sp, bones, min_deg=SPIKE_MIN, ratio=SPIKE_RATIO, top=12):
    T = sp.shape[0]
    ev = []
    for f in range(1, T):
        for k, b in enumerate(bones):
            v = sp[f, k]
            if v < min_deg:
                continue
            nb = max(sp[f - 1, k] if f - 1 >= 1 else 0.0, sp[f + 1, k] if f + 1 < T else 0.0)
            if v >= ratio * max(nb, 1e-6):
                ev.append(dict(frame=f, bone=b, deg=round(float(v), 1), neighbour_deg=round(float(nb), 1)))
    ev.sort(key=lambda e: -e['deg'])
    return ev[:top]


def issue(out, code, sev, frames, value, thr, note, bone=None):
    out.append(dict(code=code, severity=sev, frames=[int(frames[0]), int(frames[1])], value=round(float(value), 2),
                    threshold=thr, bone=bone, note=note))


def analyze(set_name, clip, base=None):
    d = MC.load_dump(MC.clip_path(set_name, clip, base))
    m = d['meta']
    T = d['W'].shape[0]
    res = dict(set=set_name, clip=clip, rig=m['rig'], frames=T, fps=30, duration_s=round(m['duration'], 4),
               additive=m['additive'], animation_type=m['animation_type'], root_motion_type=m['root_motion_type'],
               key_end_s=round(m.get('key_end_s', m['duration']), 3), events=m['events'])
    ix = MC.idx(d)
    Wl = MC.world_all(d)
    P = Wl[:, :, :3, 3]
    Wrest = MC.fk_world(d['parents152'], d['rest_lt'][None], d['rest_lr'][None])[0]
    issues = []
    res['issues'] = issues
    # ----- which bones animate (valid for additive clips too: the offsets are non-zero where the clip acts)
    dev = {}
    for b in BODY_BONES + MC.FINGERS_L + MC.FINGERS_R + MC.FPP_BONES:
        q = R.from_quat(d['lr'][:, ix[b]].astype(float))
        dev[b] = float(np.degrees((q[0].inv() * q).magnitude()).max())
    hips = P[:, ix['Hips']]
    heading_r = horizontal_right(P[:, ix['LeftUpLeg']], P[:, ix['RightUpLeg']])
    fwd_h = np.stack([-heading_r[:, 1], heading_r[:, 0], np.zeros(T)], 1)
    up = np.array([0.0, 0.0, 1.0])
    groups = {}
    for g, bones in MC.GROUPS.items():
        end = {'spine': 'Spine3', 'head_neck': 'Head', 'left_arm': 'LeftHand', 'right_arm': 'RightHand',
               'left_leg': 'LeftFoot', 'right_leg': 'RightFoot'}.get(g)
        md = max(dev[b] for b in bones)
        pr = None
        if end:
            rel = P[:, ix[end]] - hips
            loc = np.stack([np.sum(rel * heading_r, 1), np.sum(rel * fwd_h, 1), rel[:, 2]], 1)
            pr = float(np.linalg.norm(loc.max(0) - loc.min(0)) * 100)
        groups[g] = dict(animated=bool(md > ANIM_DEG or (pr is not None and pr > ANIM_CM and g != 'spine')),
                         max_rot_dev_deg=round(md, 1), end_bone_range_cm=None if pr is None else round(pr, 1),
                         bones_animated=[b for b in bones if dev[b] > ANIM_DEG])
    res['groups'] = groups
    res['fpp_bones'] = {b: round(dev[b], 1) for b in MC.FPP_BONES}
    hz = hips[:, 2]
    hips_range = float(np.linalg.norm(hips.max(0) - hips.min(0)) * 100)
    res['hips'] = dict(rest_z_m=round(float(Wrest[ix['Hips'], 2, 3]), 3), z_min_m=round(float(hz.min()), 3),
                       z_max_m=round(float(hz.max()), 3), z_mean_m=round(float(hz.mean()), 3),
                       drop_cm=round(float((Wrest[ix['Hips'], 2, 3] - hz.min()) * 100), 1),
                       xy_path_cm=round(float(np.linalg.norm(np.diff(hips[:, :2], axis=0), axis=1).sum() * 100), 1),
                       xy_net_cm=round(float(np.linalg.norm(hips[-1, :2] - hips[0, :2]) * 100), 1), range_3d_cm=round(hips_range, 1))
    tr = P[:, ix['Trajectory']]
    res['root'] = dict(travel_xy_cm=round(float(np.linalg.norm(tr[-1, :2] - tr[0, :2]) * 100), 1),
                       travel_z_cm=round(float((tr[-1, 2] - tr[0, 2]) * 100), 1), root_motion=m['root_motion_type'])
    lower_static = (not groups['left_leg']['animated']) and (not groups['right_leg']['animated']) and hips_range < 2.0
    res['lower_body_static'] = bool(lower_static)
    if m['additive']:
        res['pose_metrics'] = 'skipped (additive clip: tracks are offsets on top of another pose)'
        return res
    if lower_static:
        issue(issues, 'LOWER_BODY_STATIC', 'info', (0, T - 1), hips_range, '<2 cm hips range and no leg animation',
              'legs and hips do not animate (FPP clip): the lower body must come from motion matching / corrected key poses')
    # ----- angular velocity + spikes
    sp = ang_speed(d, ix, BODY_BONES)
    fm = sp.max(1)
    kmax = int(np.argmax(sp.max(0)))
    fmax = int(np.argmax(sp[:, kmax]))
    spikes = find_spikes(sp, BODY_BONES)
    fingers = ang_speed(d, ix, MC.FINGERS_L + MC.FINGERS_R)
    res['angvel'] = dict(max_deg_per_frame=round(float(sp.max()), 1), max_bone=BODY_BONES[kmax], max_frame=fmax,
                         p95_frame_max=round(float(np.percentile(fm, 95)), 1), frame_max_series=[round(float(x), 1) for x in fm],
                         finger_max_deg_per_frame=round(float(fingers.max()), 1), spikes=spikes)
    for e in spikes:
        sev = 'high' if e['deg'] >= 30 and e['deg'] >= 4 * max(e['neighbour_deg'], 1e-6) else 'med' if e['deg'] >= 18 else 'low'
        issue(issues, 'SPIKE', sev, (e['frame'] - 1, e['frame']), e['deg'], 'step >= %.0f deg and >= %.1fx neighbours' % (SPIKE_MIN, SPIKE_RATIO),
              'isolated rotation snap of %s (neighbour step %.1f deg)' % (e['bone'], e['neighbour_deg']), e['bone'])
    # ----- hinge joints
    lim = {}
    for side in ('Left', 'Right'):
        for joint, bone, mx in (('elbow', side + 'ForeArm', 150.0), ('knee', side + 'Leg', 150.0)):
            rv = rot_rel(d, ix, bone, None)
            flex = -rv[:, 2]
            off = np.linalg.norm(rv[:, :2], axis=1)
            key = '%s_flex_%s' % (joint, side.lower())
            lim[key] = dict(min=round(float(flex.min()), 1), max=round(float(flex.max()), 1), max_frame=int(flex.argmax()),
                            off_axis_max=round(float(off.max()), 1))
            for a, b in runs(flex > mx):
                issue(issues, joint.upper() + '_OVERFLEX', 'high' if flex[a:b + 1].max() > mx + 15 else 'med', (a, b), flex[a:b + 1].max(),
                      '>%.0f' % mx, '%s %s bent beyond %.0f deg' % (side, joint, mx), bone)
            for a, b in runs(flex < -8.0):
                issue(issues, joint.upper() + '_HYPEREXT', 'high', (a, b), flex[a:b + 1].min(), '<-8',
                      '%s %s hyperextended (negative flexion)' % (side, joint), bone)
    fore = {s: unit(P[:, ix[s + 'Hand']] - P[:, ix[s + 'ForeArm']]) for s in ('Left', 'Right')}
    chest_up = unit(P[:, ix['Neck']] - hips)
    for side in ('Left', 'Right'):
        hand = unit(P[:, ix[side + 'HandMiddle1']] - P[:, ix[side + 'Hand']])
        wb = MC.angle_between(fore[side], hand)
        lim['wrist_bend_' + side.lower()] = dict(max=round(float(wb.max()), 1), max_frame=int(wb.argmax()))
        for a, b in runs(wb > 70):
            issue(issues, 'WRIST_BEND', 'high' if wb[a:b + 1].max() > 90 else 'med', (a, b), wb[a:b + 1].max(), '>70',
                  '%s wrist bent %.0f deg away from the forearm line' % (side, wb[a:b + 1].max()), side + 'Hand')
        arm = unit(P[:, ix[side + 'ForeArm']] - P[:, ix[side + 'Arm']])
        el = MC.angle_between(arm, -chest_up)
        lim['shoulder_elev_' + side.lower()] = dict(min=round(float(el.min()), 1), max=round(float(el.max()), 1), max_frame=int(el.argmax()))
        for a, b in runs(el > 160):
            issue(issues, 'ARM_OVERHEAD', 'med', (a, b), el[a:b + 1].max(), '>160', '%s upper arm raised beyond 160 deg' % side, side + 'Arm')
    # ----- torso
    def delta_right(b):  # bone's own 'right' (+X of the reference pose) after its rest-relative world rotation
        Rd = Wl[:, ix[b], :3, :3] @ Wrest[ix[b], :3, :3].T
        r = Rd @ np.array([1.0, 0.0, 0.0])
        r[:, 2] = 0
        return unit(r)
    twist = wrap(yaw_of(delta_right('Spine3')) - yaw_of(delta_right('Hips')))
    v = chest_up
    lean_f = np.degrees(np.arctan2(np.sum(v * fwd_h, 1), v[:, 2]))
    lean_s = np.degrees(np.arctan2(np.sum(v * heading_r, 1), v[:, 2]))
    b1 = unit(P[:, ix['Spine']] - hips)
    b2 = unit(P[:, ix['Neck']] - P[:, ix['Spine3']])
    bend = MC.angle_between(b1, b2)
    rest_neck = np.linalg.norm(Wrest[ix['Head'], :3, 3] - Wrest[ix['Spine3'], :3, 3])
    stretch = (np.linalg.norm(P[:, ix['Head']] - P[:, ix['Spine3']], axis=1) / rest_neck - 1) * 100
    res['torso'] = dict(twist_min=round(float(twist.min()), 1), twist_max=round(float(twist.max()), 1),
                        lean_fwd_min=round(float(lean_f.min()), 1), lean_fwd_max=round(float(lean_f.max()), 1),
                        lean_side_abs_max=round(float(np.abs(lean_s).max()), 1), spine_bend_max=round(float(bend.max()), 1),
                        neck_stretch_pct_min=round(float(stretch.min()), 1), neck_stretch_pct_max=round(float(stretch.max()), 1))
    for a, b in runs(np.abs(twist) > 50):
        issue(issues, 'TORSO_TWIST', 'med', (a, b), np.abs(twist[a:b + 1]).max(), '>50', 'chest turned %.0f deg against the hips' % twist[a:b + 1][np.abs(twist[a:b + 1]).argmax()], 'Spine3')
    for a, b in runs(bend > 45):
        issue(issues, 'SPINE_BEND', 'med', (a, b), bend[a:b + 1].max(), '>45', 'spine curved %.0f deg between hips and neck' % bend[a:b + 1].max(), 'Spine')
    for a, b in runs(np.abs(stretch) > 8):
        issue(issues, 'NECK_STRETCH', 'high' if np.abs(stretch[a:b + 1]).max() > 15 else 'med', (a, b), stretch[a:b + 1][np.abs(stretch[a:b + 1]).argmax()],
              '|x|>8 %', 'Spine3->Head length differs from the reference by that percent', 'Head')
    # shoulder girdle: width of the upper-arm joints / clavicle joints along the hips' right axis (reference pose: 36 cm / 4 cm). FPP clips
    # animate the clavicle translations, so the joints can cross the midline (left joint on the right side)
    aw = np.sum((P[:, ix['RightArm']] - P[:, ix['LeftArm']]) * heading_r, 1) * 100
    cw = np.sum((P[:, ix['RightShoulder']] - P[:, ix['LeftShoulder']]) * heading_r, 1) * 100
    rest_aw = float(Wrest[ix['RightArm'], 0, 3] - Wrest[ix['LeftArm'], 0, 3]) * 100
    res['shoulders'] = dict(arm_joint_width_cm_min=round(float(aw.min()), 1), arm_joint_width_cm_max=round(float(aw.max()), 1),
                            clavicle_width_cm_min=round(float(cw.min()), 1), clavicle_width_cm_max=round(float(cw.max()), 1),
                            rest_arm_joint_width_cm=round(rest_aw, 1), rest_clavicle_width_cm=4.0)
    for a, b in runs(cw < -1.0):
        issue(issues, 'CLAVICLES_CROSSED', 'med', (a, b), cw[a:b + 1].min(), '<-1 cm', 'left clavicle joint is on the right side of the right one (FPP camera framing)', 'LeftShoulder')
    for a, b in runs(aw < 0.55 * rest_aw):
        issue(issues, 'SHOULDERS_NARROW', 'med' if aw[a:b + 1].min() > 0 else 'high', (a, b), aw[a:b + 1].min(), '<%.0f cm' % (0.55 * rest_aw),
              'upper-arm joints only %.0f cm apart (reference %.0f cm): shoulders pulled together / crossed' % (aw[a:b + 1].min(), rest_aw), 'LeftArm')
    res['limits'] = lim
    # ----- hands vs face, hand separation
    eyes = (P[:, ix['LeftEye']] + P[:, ix['RightEye']]) / 2
    prox = {}
    for name in ('LeftHand', 'RightHand', 'WeaponLeft', 'WeaponRight'):
        dd = np.linalg.norm(P[:, ix[name]] - eyes, axis=1) * 100
        prox[name] = dict(min_cm=round(float(dd.min()), 1), min_frame=int(dd.argmin()), frames_below=int((dd < NEAR_FACE_CM).sum()))
        for a, b in runs(dd < NEAR_FACE_CM):
            issue(issues, 'HAND_NEAR_FACE', 'high' if (b - a + 1) >= 4 and dd[a:b + 1].min() < 18 else 'med' if (b - a + 1) >= 3 else 'low', (a, b),
                  dd[a:b + 1].min(), '<%d cm' % NEAR_FACE_CM, '%s within %.0f cm of the eyes' % (name, dd[a:b + 1].min()), name)
    sep = np.linalg.norm(P[:, ix['LeftHand']] - P[:, ix['RightHand']], axis=1) * 100
    res['hands'] = dict(near_face=prox, separation_min_cm=round(float(sep.min()), 1), separation_max_cm=round(float(sep.max()), 1),
                        separation_start_cm=round(float(sep[0]), 1), separation_end_cm=round(float(sep[-1]), 1))
    for a, b in runs(sep < 6):
        issue(issues, 'HANDS_TOUCH', 'low', (a, b), sep[a:b + 1].min(), '<6 cm', 'hands (joints) closer than 6 cm', 'LeftHand')
    hv = {}
    for name in ('LeftHand', 'RightHand'):
        v = np.linalg.norm(np.diff(P[:, ix[name]], axis=0), axis=1) * 30
        hv[name] = round(float(v.max()) if len(v) else 0.0, 2)
    res['hands']['peak_speed_m_s'] = hv
    # ----- feet
    feet = {}
    cm = {}
    sole = []
    for side in ('Left', 'Right'):
        hz_, tz_ = P[:, ix[side + 'Heel'], 2], P[:, ix[side + 'ToeBase'], 2]
        sole.append(K.sole_height(hz_, tz_, Wrest[ix[side + 'Heel'], 2, 3], Wrest[ix[side + 'ToeBase'], 2, 3]))
    sole = np.stack(sole, 1)
    con = K.detect_contacts(sole)
    airborne = (sole.min(1) > 0.10) & (con.sum(1) == 0)
    for fi, side in enumerate(('Left', 'Right')):
        segs = K.slide_segments(con[:, fi], P[:, ix[side + 'Foot'], :2])
        feet[side.lower()] = dict(contact_ratio=round(float(con[:, fi].mean()), 2), contact_runs=runs(con[:, fi]),
                                  sole_height_range_cm=[round(float(sole[:, fi].min() * 100), 1), round(float(sole[:, fi].max() * 100), 1)],
                                  slides=[dict(s, drift_cm=round(s['drift_cm'], 1), max_speed_cm_s=round(s['max_speed_cm_s'], 1)) for s in segs])
        for s in segs:
            if s['drift_cm'] > 5:
                issue(issues, 'FOOT_SLIDE', 'high' if s['drift_cm'] > 12 else 'med', (s['start'], s['end']), s['drift_cm'], '>5 cm',
                      '%s foot drifts %.1f cm during a planted phase' % (side, s['drift_cm']), side + 'Foot')
    feet['airborne_runs'] = runs(airborne)
    feet['both_feet_off_ratio'] = round(float((con.sum(1) == 0).mean()), 2)
    res['feet'] = feet
    # ----- fpp camera bones: how far they move and where the camera aim points relative to the head
    aim, tgt = P[:, ix['Torso_fppCamera_Aim_JNT']], P[:, ix['Torso_fppCamera_Target_JNT']]
    av = unit(tgt - aim)
    Rh = Wl[:, ix['Head'], :3, :3] @ Wrest[ix['Head'], :3, :3].T
    head_fwd = Rh @ np.array([0.0, 1.0, 0.0])
    aim_vs_head = MC.angle_between(av, head_fwd)
    res['fpp'] = dict(local_rot_dev_deg=res['fpp_bones'], aim_pitch_min=round(float(np.degrees(np.arcsin(av[:, 2])).min()), 1),
                      aim_pitch_max=round(float(np.degrees(np.arcsin(av[:, 2])).max()), 1),
                      aim_vs_head_deg_min=round(float(aim_vs_head.min()), 1), aim_vs_head_deg_max=round(float(aim_vs_head.max()), 1),
                      head_rot_dev_deg=round(dev['Head'], 1),
                      note='Head is animated only when head_rot_dev_deg > 2; the camera aim then comes from the fppCamera helpers')
    for a, b in runs(aim_vs_head > 40):
        issue(issues, 'FPP_AIM_OFF_HEAD', 'low', (a, b), aim_vs_head[a:b + 1].max(), '>40', 'fppCamera aim differs from the head forward by that much', 'Torso_fppCamera_Aim_JNT')
    # ----- loop closure (first vs last frame)
    q0, q1 = d['lr'][0, [ix[b] for b in BODY_BONES]].astype(float), d['lr'][-1, [ix[b] for b in BODY_BONES]].astype(float)
    dq = np.degrees((R.from_quat(q0).inv() * R.from_quat(q1)).magnitude())
    res['loop_gap'] = dict(max_rot_deg=round(float(dq.max()), 1), bone=BODY_BONES[int(dq.argmax())],
                           hands_cm=[round(float(np.linalg.norm(P[0, ix[h]] - P[-1, ix[h]]) * 100), 1) for h in ('LeftHand', 'RightHand')])
    # frames worth looking at
    res['spike_frames'] = sorted({e['frame'] for e in spikes})
    sev_rank = {'high': 3, 'med': 2, 'low': 1, 'info': 0}
    issues.sort(key=lambda i: (-sev_rank[i['severity']], i['frames'][0]))
    return res


def summary(r):
    if r['additive']:
        return '%s/%s ADDITIVE %d fr' % (r['set'], r['clip'], r['frames'])
    n = {s: sum(1 for i in r['issues'] if i['severity'] == s) for s in ('high', 'med', 'low')}
    return '%s/%s %d fr lower_static=%s angvel_max=%.0f issues high=%d med=%d low=%d' % (
        r['set'], r['clip'], r['frames'], r['lower_body_static'], r['angvel']['max_deg_per_frame'], n['high'], n['med'], n['low'])


def metrics_path(set_name, clip):
    return os.path.join(MC.WORK, 'analysis', 'metrics', set_name, clip + '.json')


def sheet(r, n_frames):
    frames = sorted(set(np.linspace(0, r['frames'] - 1, min(n_frames, r['frames'])).round().astype(int).tolist()) | set(r.get('spike_frames', [])[:4]))
    out = os.path.join(MC.WORK, 'sheets', r['set'], r['clip'] + '.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cmd = [MC.PY, '-I', os.path.join(HERE, 'melee_sheet.py'), r['set'], r['clip'], '--frames', ','.join(map(str, frames)), '--out', out]
    p = subprocess.run(cmd, capture_output=True, text=True)
    print((p.stdout.strip().splitlines() or [''])[-1])
    if p.returncode:
        print(p.stderr[-800:])
    return out


def main(a):
    if not a or a[0] in ('-h', '--help'):
        print(__doc__)
        return
    n_frames = int(a[a.index('--sheet-frames') + 1]) if '--sheet-frames' in a else 12
    if a[0] == '--set':
        man = json.load(open(os.path.join(MC.WORK, 'manifest.json')))
        targets = [(c['set'], c['clip']) for c in man['clips'] if c['set'] == a[1]]
    elif a[0] == '--group':
        man = json.load(open(os.path.join(MC.WORK, 'manifest.json')))
        g = next(x for x in man['groups'] if x['id'] == a[1])
        targets = [(g['set'], c) for c in g['clips']]
    else:
        targets = [(a[0], a[1])]
    for s, c in targets:
        r = analyze(s, c)
        out = a[a.index('--out') + 1] if '--out' in a and len(targets) == 1 else metrics_path(s, c)
        MC.jdump(r, out)
        if '--sheet' in a and not r['additive']:
            r['sheet'] = sheet(r, n_frames)
            MC.jdump(r, out)
        if '--quiet' not in a:
            print(summary(r), '->', out)


if __name__ == '__main__':
    main(sys.argv[1:])
