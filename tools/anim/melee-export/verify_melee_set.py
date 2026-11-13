"""Round-trip gate of one built melee set (2026-11-09).

  python -I verify_melee_set.py --set pwa_fists --built-glb <built glb> --roundtrip-glb <export of the final .anims, -RootMotion>
                                --vanilla-glb <vanilla set glb, -RootMotion> --vanilla-json <vanilla json> --final-json <serialized final .anims>
                                --report <json> [--max-t-mm 1.0] [--max-r-deg 0.15]

Per clip of the built GLB, on EVERY frame (t = i / 30, numFrames of the vanilla clip):
  (a) local transform of every one of the 152 joints, round trip vs built: translation <= max-t-mm, rotation <= max-r-deg
      (joints whose built local translation is longer than 10 m - the parked weapon IK target - are judged relative: <= 1e-6 x length);
  (b) root motion: Root of the round trip (motion extraction applied by the export) vs the vanilla clip's Root (<= 0.1 mm / 0.01 deg);
  (c) hand / foot IK helpers vs their bone on the round trip: HAND helpers (arm_*_ik, arm_*_ik_offset, Torso_(World_)*Arm_Hand_IK_JNT)
      <= 1.5 cm + HAND_RT_TOL_CM (round-trip compression only) on every frame, the last one included, NO vanilla allowance (2026-11-09: the
      builder puts far hand helpers on the hand, build_melee_set.snap_hand_helpers; the game's arm IK follows them); FOOT helpers <= 1.5 cm, or
      <= the vanilla distance on the same frame + 0.1 cm where vanilla itself is farther (inherited frames are listed); reported per hand
      helper: max / last-frame distance, max rotation vs the hand; arm_*_ik_weapon vs the hand (information, not gated);
      (c2) arm_*_ik_weapon vs its weapon joint (WeaponRight / WeaponLeft): <= 1.5 cm + HAND_RT_TOL_CM on every frame for the sets of
      melee_export_lib.WEAPON_HELPER_SETS (2026-11-10, hammer), reported only for the other sets;
  (d) FPP camera helpers in world vs vanilla <= max-t-mm;
  (e) events / float tracks / types of the round-trip GLB extras == vanilla, fallback frames EMPTY (cleared on purpose, build_melee_set.py /
      patch_melee_json.py); final .anims json: the clip set == built set, rig == vanilla, numFrames / duration == vanilla, every field of the
      clip except animBuffer == vanilla (motion extraction verbatim), animBuffer.fallbackFrameIndices == [].
Exit 0 only when every clip passes (a)-(e).
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402
import skel as S  # noqa: E402
from patch_melee_json import strip  # noqa: E402

HAND_RT_TOL_CM = 0.05   # (c) hands: the importer's compression on top of the built distance (<= 1.5 cm in the built clip)
EXTRA_KEYS = ('animationType', 'rootMotionType', 'frameClamping', 'frameClampingStartFrame', 'frameClampingEndFrame', 'numExtraJoints',
              'numExtraTracks', 'constTrackKeys', 'trackKeys', 'fallbackFrameIndices', 'optimizationHints', 'animEvents')


def norm_json(x):
    return json.dumps(x, sort_keys=True)


def keys_equal(a, b, t_tol=1e-3, v_tol=1e-5):
    """float track keys: same track / value; key time within t_tol s (the importer re-quantises the time of constant keys by ~1e-5 s)."""
    a, b = a or [], b or []
    if len(a) != len(b):
        return False
    sa = sorted(a, key=lambda k: (k['trackIndex'], k['time']))
    sb = sorted(b, key=lambda k: (k['trackIndex'], k['time']))
    return all(x['trackIndex'] == y['trackIndex'] and abs(x['time'] - y['time']) <= t_tol and abs(x['value'] - y['value']) <= v_tol for x, y in zip(sa, sb))


def extras_differ(exr, exv):
    out = []
    for k in EXTRA_KEYS:
        if k == 'fallbackFrameIndices':     # cleared on purpose (the vanilla fallback frames are first-person poses)
            if exr.get(k) not in (None, []):
                out.append('fallbackFrameIndices not cleared')
            continue
        if k in ('constTrackKeys', 'trackKeys'):
            if not keys_equal(exr.get(k), exv.get(k)):
                out.append(k)
        elif norm_json(exr.get(k)) != norm_json(exv.get(k)):
            out.append(k)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', required=True)
    ap.add_argument('--built-glb', required=True)
    ap.add_argument('--roundtrip-glb', required=True)
    ap.add_argument('--vanilla-glb', required=True)
    ap.add_argument('--vanilla-json', required=True)
    ap.add_argument('--final-json', required=True)
    ap.add_argument('--roundtrip-nort-glb', required=True, help='export of the final .anims WITHOUT root motion (anim buffer Root)')
    ap.add_argument('--vanilla-glb-nort', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--max-t-mm', type=float, default=1.0)
    ap.add_argument('--max-r-deg', type=float, default=0.15)
    a = ap.parse_args()
    vclips, vrig = L.anims_json_clips(a.vanilla_json)
    fclips, frig = L.anims_json_clips(a.final_json)
    sb, sr, sv, srn, svn = (S.Skeleton.from_glb(p) for p in (a.built_glb, a.roundtrip_glb, a.vanilla_glb, a.roundtrip_nort_glb, a.vanilla_glb_nort))
    assert sb.names == sr.names == sv.names == srn.names == svn.names, 'joint lists differ'
    built = list(L.G.clips(sb.gj).keys())
    rep = dict(set=a.set, limits=dict(t_mm=a.max_t_mm, r_deg=a.max_r_deg), clips={}, fail=[])
    # (e) json level
    van = json.load(open(a.vanilla_json, encoding='utf-8'))['Data']['RootChunk']['animations']
    fin = json.load(open(a.final_json, encoding='utf-8'))['Data']['RootChunk']['animations']
    vmap = {e['Data']['animation']['Data']['name']['$value']: e for e in van}
    fmap = {e['Data']['animation']['Data']['name']['$value']: e for e in fin}
    rep['json'] = dict(rig_equal=frig == vrig, rig=frig, clip_set_equal=sorted(fmap) == sorted(built), final_clips=len(fmap))
    if not (rep['json']['rig_equal'] and rep['json']['clip_set_equal']):
        rep['fail'].append('json: rig / clip set')
    ix = sb.index
    cam = [ix[n] for n in L.FPP_CAMERA]
    for c in built:
        row, why = {}, []
        T, dur = int(vclips[c]['numFrames']), float(vclips[c]['duration'])
        lb, rb, exb, _ = L.sample_clip(sb, c, T, dur, root_seconds=True)
        lr_, rr_, exr, _ = L.sample_clip(sr, c, T, dur)
        lv, rv, exv, _ = L.sample_clip(sv, c, T, dur)
        # (a) locals
        long_ = np.linalg.norm(lb, axis=-1).max(0) > 10.0
        dt = np.linalg.norm(lr_ - lb, axis=-1) * 1000
        rel = dt / np.maximum(np.linalg.norm(lb, axis=-1) * 1000, 1e-9)
        dr = L.quat_angle_deg(rr_, rb)
        dt_n = dt.copy()
        dt_n[:, long_] = 0
        dt_n[:, 0] = 0      # Root: built = anim buffer Root, round trip = motion extraction -> checked in (b) and (f)
        dr[:, 0] = 0
        jt, jr = int(np.argmax(dt_n.max(0))), int(np.argmax(dr.max(0)))
        row['local_t_mm'] = dict(max=round(float(dt_n.max()), 4), bone=sb.names[jt], frame=int(np.argmax(dt_n[:, jt])))
        row['local_r_deg'] = dict(max=round(float(dr.max()), 4), bone=sb.names[jr], frame=int(np.argmax(dr[:, jr])))
        if long_.any():
            row['long_translation_bones'] = {sb.names[j]: dict(len_m=round(float(np.linalg.norm(lb[:, j], axis=-1).max()), 2),
                                                               rel_err=float(rel[:, j].max())) for j in np.nonzero(long_)[0]}
            if max(v['rel_err'] for v in row['long_translation_bones'].values()) > 1e-6:
                why.append('long translation rel err')
        if row['local_t_mm']['max'] > a.max_t_mm:
            why.append('local translation %.3f mm (%s)' % (row['local_t_mm']['max'], row['local_t_mm']['bone']))
        if row['local_r_deg']['max'] > a.max_r_deg:
            why.append('local rotation %.3f deg (%s)' % (row['local_r_deg']['max'], row['local_r_deg']['bone']))
        # (b) root motion vs vanilla
        rt_t = float(np.linalg.norm(lr_[:, 0] - lv[:, 0], axis=-1).max() * 1000)
        rt_r = float(L.quat_angle_deg(rr_[:, 0], rv[:, 0]).max())
        row['root_vs_vanilla'] = dict(t_mm=round(rt_t, 4), r_deg=round(rt_r, 5), end_m=np.round(lr_[-1, 0], 4).tolist())
        if rt_t > 0.1 or rt_r > 0.01:
            why.append('root motion differs from vanilla (%.3f mm %.4f deg)' % (rt_t, rt_r))
        # (f) anim buffer Root (export without root motion) == vanilla buffer Root
        ln_, rn_, _, _ = L.sample_clip(srn, c, T, dur, root_seconds=True)
        lvn, rvn, _, _ = L.sample_clip(svn, c, T, dur, root_seconds=True)
        bt_ = float(np.linalg.norm(ln_[:, 0] - lvn[:, 0], axis=-1).max() * 1000)
        br_ = float(L.quat_angle_deg(rn_[:, 0], rvn[:, 0]).max())
        row['buffer_root_vs_vanilla'] = dict(t_mm=round(bt_, 4), r_deg=round(br_, 5), vanilla_max_cm=round(float(np.linalg.norm(lvn[:, 0], axis=-1).max() * 100), 4))
        if bt_ > 0.1 or br_ > 0.01:
            why.append('anim buffer Root differs from vanilla (%.3f mm %.4f deg)' % (bt_, br_))
        # (c) helpers, (d) fpp camera
        Wr_t, Wr_r = sr.world(lr_, rr_)
        Wv_t, _ = sv.world(lv, rv)
        hel = {}
        for h, b in L.GATE_HELPERS.items():
            dn = np.linalg.norm(Wr_t[:, ix[h]] - Wr_t[:, ix[b]], axis=-1) * 100
            dv = np.linalg.norm(Wv_t[:, ix[h]] - Wv_t[:, ix[b]], axis=-1) * 100
            hand = 'Hand' in b
            lim = L.HAND_LIMIT_CM if hand else L.FOOT_LIMIT_CM
            allowed = np.full(dn.shape, lim + HAND_RT_TOL_CM) if hand else np.maximum(lim, dv + 0.1)
            bad = np.nonzero(dn > allowed)[0]
            inh = np.nonzero(dn > lim)[0]
            hel[h] = dict(max_cm=round(float(dn.max()), 3), frames_over_1p5_inherited=inh.tolist(), fail_frames=bad.tolist())
            if hand:
                Ro = np.transpose(Wr_r[:, ix[b]], (0, 2, 1)) @ Wr_r[:, ix[h]]
                ang = np.degrees(np.arccos(np.clip((np.trace(Ro, axis1=1, axis2=2) - 1) / 2, -1, 1)))
                hel[h].update(last_cm=round(float(dn[-1]), 3), rot_max_deg=round(float(ang.max()), 2), vanilla_max_cm=round(float(dv.max()), 3))
            if len(bad):
                why.append('helper %s %.2f cm at frames %s' % (h, float(dn[bad].max()), bad.tolist()[:5]))
        row['helpers'] = hel
        row['helpers_max_cm'] = max(v['max_cm'] for v in hel.values())
        hv = [v for h, v in hel.items() if 'Hand' in L.GATE_HELPERS[h]]
        row['hand_helpers_max_cm'] = max(v['max_cm'] for v in hv)
        row['hand_helpers_last_frame_max_cm'] = max(v['last_cm'] for v in hv)
        row['hand_helpers_rot_max_deg'] = max(v['rot_max_deg'] for v in hv)
        row['ik_weapon_vs_hand_cm'] = {h: round(float((np.linalg.norm(Wr_t[:, ix[h]] - Wr_t[:, ix[b]], axis=-1) * 100).max()), 3)
                                       for h, b in (('arm_left_ik_weapon', 'LeftHand'), ('arm_right_ik_weapon', 'RightHand')) if h in ix}
        # (c2) arm_*_ik_weapon vs its WEAPON JOINT (vanilla convention): gated for L.WEAPON_HELPER_SETS (<= limit + round-trip tolerance on
        # every frame incl. the last), information for the other sets
        wrow = {}
        for h, b in L.WEAPON_HELPERS.items():
            if h not in ix or b not in ix:
                continue
            dw = np.linalg.norm(Wr_t[:, ix[h]] - Wr_t[:, ix[b]], axis=-1) * 100
            bad = np.nonzero(dw > L.WEAPON_HELPER_LIMIT_CM + HAND_RT_TOL_CM)[0]
            wrow[h] = dict(max_cm=round(float(dw.max()), 3), last_cm=round(float(dw[-1]), 3), frames_over=len(bad))
            if a.set in L.WEAPON_HELPER_SETS and len(bad):
                why.append('weapon helper %s %.2f cm from %s at frames %s' % (h, float(dw[bad].max()), b, bad.tolist()[:5]))
        row['ik_weapon_vs_weapon_joint_cm'] = wrow
        row['ik_weapon_gated'] = a.set in L.WEAPON_HELPER_SETS
        camd = float(np.linalg.norm(Wr_t[:, cam] - Wv_t[:, cam], axis=-1).max() * 1000)
        row['fpp_camera_vs_vanilla_mm'] = round(camd, 4)
        if camd > a.max_t_mm:
            why.append('fpp camera helpers moved %.3f mm' % camd)
        # (e) extras + json
        diff_ex = extras_differ(exr, exv)
        row['extras_differ'] = diff_ex
        if diff_ex:
            why.append('extras differ: %s' % diff_ex)
        fe, ve = fmap.get(c), vmap.get(c)
        if fe is None:
            why.append('clip missing in final json')
        else:
            fa, va = fe['Data']['animation']['Data'], ve['Data']['animation']['Data']
            nf, nv = fclips[c]['numFrames'], vclips[c]['numFrames']
            row['numFrames'], row['numFrames_vanilla'] = nf, nv
            row['duration'], row['duration_vanilla'] = fclips[c]['duration'], vclips[c]['duration']
            row['buffer'], row['buffer_vanilla'] = fclips[c]['buffer'], vclips[c]['buffer']
            if nf != nv or abs(float(row['duration']) - float(row['duration_vanilla'])) > 1e-6:
                why.append('numFrames / duration %s/%s vs vanilla %s/%s' % (nf, row['duration'], nv, row['duration_vanilla']))
            fd = [k for k in set(fa) | set(va) if k not in ('animBuffer',) and strip(fa.get(k)) != strip(va.get(k))]
            fd += ['entry.' + k for k in set(fe['Data']) | set(ve['Data']) if k != 'animation' and strip(fe['Data'].get(k)) != strip(ve['Data'].get(k))]
            row['json_fields_differ'] = sorted(fd)
            if fd:
                why.append('json fields differ: %s' % sorted(fd))
            ffi = ((fa.get('animBuffer') or {}).get('Data') or {}).get('fallbackFrameIndices')
            row['final_fallback_frames'] = len(ffi or [])
            if ffi:
                why.append('final json: fallbackFrameIndices not cleared (%d)' % len(ffi))
        row['ok'] = not why
        row['why'] = why
        rep['clips'][c] = row
        if why:
            rep['fail'].append('%s: %s' % (c, '; '.join(why[:3])))
    cl = rep['clips'].values()
    rep['summary'] = dict(clips=len(rep['clips']), passed=sum(1 for r in cl if r['ok']),
                          local_t_mm_max=max(r['local_t_mm']['max'] for r in cl), local_r_deg_max=max(r['local_r_deg']['max'] for r in cl),
                          root_t_mm_max=max(r['root_vs_vanilla']['t_mm'] for r in cl), helpers_max_cm=max(r['helpers_max_cm'] for r in cl),
                          buffer_root_t_mm_max=max(r['buffer_root_vs_vanilla']['t_mm'] for r in cl),
                          helper_frames_over_1p5_inherited=sum(len(h['frames_over_1p5_inherited']) for r in cl for h in r['helpers'].values()),
                          hand_helpers_max_cm=max(r['hand_helpers_max_cm'] for r in cl),
                          hand_helpers_last_frame_max_cm=max(r['hand_helpers_last_frame_max_cm'] for r in cl),
                          hand_helpers_rot_max_deg=max(r['hand_helpers_rot_max_deg'] for r in cl),
                          hand_helper_frames_over_1p5=sum(len(h['frames_over_1p5_inherited']) for r in cl for k, h in r['helpers'].items()
                                                          if 'Hand' in L.GATE_HELPERS[k]),
                          fpp_camera_mm_max=max(r['fpp_camera_vs_vanilla_mm'] for r in cl),
                          ik_weapon_vs_weapon_joint_max_cm=max([v['max_cm'] for r in cl for v in r['ik_weapon_vs_weapon_joint_cm'].values()] or [0.0]),
                          ik_weapon_gated=a.set in L.WEAPON_HELPER_SETS,
                          numFrames_differ=sum(1 for r in cl if r.get('numFrames') != r.get('numFrames_vanilla')))
    L.jdump(rep, a.report)
    s = rep['summary']
    print('MELEE_VERIFY %s %d/%d pass | local %.4f mm %.4f deg | root %.4f mm buffer root %.4f mm | helpers max %.2f cm (inherited frames over 1.5: %d) | '
          'hand helpers max %.2f cm, last frame %.2f cm, rot %.2f deg, frames over 1.5: %d | fpp cam %.4f mm | numFrames differ %d | %s' % (
              a.set, s['passed'], s['clips'], s['local_t_mm_max'], s['local_r_deg_max'], s['root_t_mm_max'], s['buffer_root_t_mm_max'], s['helpers_max_cm'],
              s['helper_frames_over_1p5_inherited'], s['hand_helpers_max_cm'], s['hand_helpers_last_frame_max_cm'], s['hand_helpers_rot_max_deg'],
              s['hand_helper_frames_over_1p5'], s['fpp_camera_mm_max'], s['numFrames_differ'], 'OK' if not rep['fail'] else 'FAIL ' + ' | '.join(rep['fail'][:4])))
    return 0 if not rep['fail'] else 1


if __name__ == '__main__':
    sys.exit(main())
