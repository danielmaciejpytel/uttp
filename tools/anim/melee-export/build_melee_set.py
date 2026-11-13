"""Build the import GLB of ONE melee set for the third-person (full body) clips (2026-11-09).

  python -I build_melee_set.py --set pwa_fists --input <clips dir> --vanilla-glb <set>.anims.glb --vanilla-json <set>.anims.json
                               --out-glb <dir>\\utpp_<set>.anims.glb --report <json> [--suffix v4] [--exclude-json registration.json]
                               [--skip set/clip ...] [--identity] [--only clip,clip]

Per non-additive clip of the vanilla set that has <input>\\<set>__<clip>.<suffix>.npz (W [T,73,4,4] world matrices of the 73 body joints, GAME axes,
dump layout of Source\\tools\\melee\\melee_dump.py / melee_regenerate.py), minus the exclusions (registration_evidence.py: clip names another entry of the
same priority window holds) and --skip:
  * the 73 body joints come from the clip file (world -> local under the rig parents), EXCEPT Root / Trajectory / reference_joint, which stay
    vanilla (root motion / trajectory are never taken from the re-solved clip);
  * the other 79 joints keep their vanilla LOCAL transforms (the arms_* spaces ride on the new Spine3, the Torso_* / fppCamera chains hang
    from Root and stay vanilla in world), except
  * the limb IK helpers (feet / hand targets, knee / elbow poles; build_entry_clip.LIMB_IK_HELPERS) are re-anchored on the new limbs on EVERY
    frame with the vanilla per-frame offset in the anchor frame (exit_steps.reanchor_helpers; poles in the limb plane); their distance to the
    bone is therefore the vanilla distance on every frame; arm_*_ik_offset / arm_*_ik_weapon keep their vanilla locals (they follow arm_*_ik);
  * hand IK helpers (2026-11-09; the game's arm IK follows arm_*_ik -> arm_*_ik_offset -> arm_*_ik_weapon and the Torso_* hand targets; a stale
    target pulls V's hand to it in game): on every frame where a hand helper of the assembled clip is farther than HAND_LIMIT_CM from the final
    hand (vanilla first-person frames with a far helper: pma_fists up to 41.8 cm), it is put back on the final hand (snap_hand_helpers:
    arm_*_ik and the Torso_* hand targets = the hand's world pose, arm_*_ik_offset = identity local, the vanilla convention); frames within the
    limit are not touched (a clip without such frames is built bit-identical to the previous builder). Hard gate: every hand helper
    <= HAND_LIMIT_CM on every frame, the last one included (exit 3 otherwise);
  * events, float tracks (constTrackKeys / trackKeys), root motion type and every other GLB extra = the vanilla clip's, EXCEPT
    fallbackFrameIndices = [] (the set-level fallback frames are the vanilla first-person poses; as in the vehicle-entry builder,
    build_entry_clip.py); frames / duration = the vanilla clip's (asserted against the serialized .anims);
  * SIMD (uncompressed) vanilla buffers: the last key time is moved up by one float32 step so that the importer's frame count
    floor(t_last * 30) + 1 equals numFrames (a SIMD buffer cannot get its frame count restored afterwards, patch_melee_json.py);
  * SIMD buffers (2026-11-10): every key quaternion gets w >= 0 (L.canonical_w_nonneg; the importer corrupted some w < 0 keys of uncompressed
    clips), report simd_w_neg_keys; --no-simd-canon switches that off (test_simd_roundtrip.ps1 negative control only);
  * arm_*_ik_weapon -> its weapon joint for the sets of L.WEAPON_HELPER_SETS (snap_weapon_helpers);
  * UTPP-only clip names (parried_<side>_<attack>) are built when the vanilla json given here already holds their clone (aug_vanilla.py, called by
    run_melee_export.ps1); input clips the vanilla set lacks are listed in the report (input_not_in_vanilla) and not built;
--reject-json {clip: reason}: clips that failed a gate in a previous attempt (run_melee_export.ps1 retry): not built, they stay vanilla;
  * everything is built in the ANIM BUFFER frame: the vanilla clip is sampled from the export WITHOUT root motion (Root = the buffer Root,
    identity in every vanilla melee clip; every other local is bit-identical to the export with root motion), the Root track written to the
    GLB is that buffer Root, the root motion stays in motionExtraction (copied verbatim by patch_melee_json.py) and moves the entity in game.
    The re-solved clips have Root = identity on every frame (redo_solve.py places the pose at the frame-0 character frame, the character
    motion removed), i.e. the buffer Root; hard gate: input Root vs buffer Root <= ROOT_GATE_CM, otherwise the clip is not built.
--identity: the vanilla clip itself is used as the input (pipeline identity test: the archive must reproduce vanilla).
Writes ONE GLB with one animation per built clip (all tracks in seconds) + a report (per clip: inputs sha256, body error, helper distances).
"""
import argparse
import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402
import skel as S  # noqa: E402
import exit_steps as ES  # noqa: E402  (vehicle-entry; re-anchoring of the limb IK helpers)

DEFAULT_SKIP = ('pma_katana/finisher_katana_02',)  # degenerate dump (every joint at the origin), redo_batch.py skips it
ROOT_GATE_CM = 0.1  # input Root vs the vanilla buffer Root (a solver that baked root motion into Root would apply it twice)


def helper_dist(Wt, sk):
    out = {}
    for h, b in L.GATE_HELPERS.items():
        if h in sk.index:
            out[h] = np.linalg.norm(Wt[:, sk.index[h]] - Wt[:, sk.index[b]], axis=-1) * 100
    return out


# hand IK helpers in dependency order (arm_*_ik_offset is the child of arm_*_ik: it is judged after arm_*_ik is final).  In the vanilla
# clips they sit ON the hand (identity rotation; p99 0.5 cm, max 1.46 cm on the frames within the limit; arm_*_ik_offset local = identity)
# except some first-person frames (pma_fists: combo_attack_02 Torso_* 27.9 / 41.8 cm, arm_*_ik_offset 3.5-23.2 cm in 8 clips), which the
# re-anchoring with the vanilla per-frame offset carries over to the new hand.
HAND_SNAP_ORDER = ('arm_left_ik', 'arm_right_ik', 'arm_left_ik_offset', 'arm_right_ik_offset', 'Torso_LeftArm_Hand_IK_JNT',
                   'Torso_RightArm_Hand_IK_JNT', 'Torso_World_LeftArm_Hand_IK_JNT', 'Torso_World_RightArm_Hand_IK_JNT')
HAND_WEAPON = {'arm_left_ik_weapon': 'LeftHand', 'arm_right_ik_weapon': 'RightHand'}   # grip offset under arm_*_ik_offset: reported, not gated


def snap_hand_helpers(sk, lt, lr):
    """lt / lr (T,N,*) modified in place ONLY on the frames where a hand helper is farther than HAND_LIMIT_CM from its (final) hand:
    arm_*_ik / Torso_*Arm_Hand_IK_JNT / Torso_World_*Arm_Hand_IK_JNT -> the hand's world pose (local from the current parent world),
    arm_*_ik_offset -> identity local (= arm_*_ik, the vanilla convention; arm_*_ik is within the limit by then), arm_*_ik_weapon keeps its
    local (follows).  Returns {helper: [frames]} (empty when nothing was farther: lt / lr untouched)."""
    out = {}
    for h in HAND_SNAP_ORDER:
        if h not in sk.index:
            continue
        j, b = sk.index[h], sk.index[L.GATE_HELPERS[h]]
        Wt, Wr = sk.world(lt, lr)
        m = np.linalg.norm(Wt[:, j] - Wt[:, b], axis=-1) * 100 > L.HAND_LIMIT_CM
        if not m.any():
            continue
        if h.endswith('_ik_offset'):
            lt[m, j] = 0.0
            lr[m, j] = [0.0, 0.0, 0.0, 1.0]
        else:
            p = int(sk.parent[j])
            Rpi = np.transpose(Wr[m, p], (0, 2, 1))
            lt[m, j] = np.einsum('tij,tj->ti', Rpi, Wt[m, b] - Wt[m, p])
            lr[m, j] = S.m2q(Rpi @ Wr[m, b])
        out[h] = np.nonzero(m)[0].tolist()
    return out


def snap_weapon_helpers(sk, lt, lr, limit_cm=L.WEAPON_HELPER_LIMIT_CM):
    """Only for the sets of L.WEAPON_HELPER_SETS (2026-11-10, hammer).  Vanilla convention of every melee set: arm_right_ik_weapon ==
    WeaponRight, arm_left_ik_weapon == WeaponLeft (0-1.4 cm, same rotation).  The rebuilt clips move the weapon in the hand (sliding grip),
    so the vanilla local of arm_*_ik_weapon under arm_*_ik_offset no longer lands on the weapon joint: on every frame where it is farther
    than limit_cm from its weapon joint it is put on the weapon joint (world pose; local from the current parent).  Frames within the limit
    are not touched.  -> {helper: [frames]}"""
    out = {}
    for h, b in L.WEAPON_HELPERS.items():
        if h not in sk.index or b not in sk.index:
            continue
        j, bj = sk.index[h], sk.index[b]
        Wt, Wr = sk.world(lt, lr)
        m = np.linalg.norm(Wt[:, j] - Wt[:, bj], axis=-1) * 100 > limit_cm
        if not m.any():
            continue
        p = int(sk.parent[j])
        Rpi = np.transpose(Wr[m, p], (0, 2, 1))
        lt[m, j] = np.einsum('tij,tj->ti', Rpi, Wt[m, bj] - Wt[m, p])
        lr[m, j] = S.m2q(Rpi @ Wr[m, bj])
        out[h] = np.nonzero(m)[0].tolist()
    return out


def hand_helper_report(Wt, Wr, vWt, sk):
    """per hand helper: max / last-frame distance to the hand (cm), max rotation vs the hand (deg); arm_*_ik_weapon vs the hand (info)."""
    out = {}
    for h in HAND_SNAP_ORDER:
        if h not in sk.index:
            continue
        j, b = sk.index[h], sk.index[L.GATE_HELPERS[h]]
        d = np.linalg.norm(Wt[:, j] - Wt[:, b], axis=-1) * 100
        Ro = np.transpose(Wr[:, b], (0, 2, 1)) @ Wr[:, j]
        ang = np.degrees(np.arccos(np.clip((np.trace(Ro, axis1=1, axis2=2) - 1) / 2, -1, 1)))
        out[h] = dict(max_cm=round(float(d.max()), 3), last_cm=round(float(d[-1]), 3), rot_max_deg=round(float(ang.max()), 2),
                      frames_over_limit=np.nonzero(d > L.HAND_LIMIT_CM + 1e-6)[0].tolist())
    for h, bn in HAND_WEAPON.items():
        if h in sk.index:
            j, b = sk.index[h], sk.index[bn]
            d = np.linalg.norm(Wt[:, j] - Wt[:, b], axis=-1) * 100
            dv = np.linalg.norm(vWt[:, j] - vWt[:, b], axis=-1) * 100
            out[h] = dict(max_cm=round(float(d.max()), 3), last_cm=round(float(d[-1]), 3), vanilla_max_cm=round(float(dv.max()), 3), gated=False)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--set', required=True)
    ap.add_argument('--input')
    ap.add_argument('--suffix', default='v4')
    ap.add_argument('--vanilla-glb', required=True)
    ap.add_argument('--vanilla-json', required=True)
    ap.add_argument('--vanilla-glb-nort', required=True, help='the vanilla set exported WITHOUT root motion: its Root track = the anim buffer Root '
                                                              '(identity in the vanilla clips; the motion lives only in motionExtraction)')
    ap.add_argument('--out-glb', required=True)
    ap.add_argument('--report', required=True)
    ap.add_argument('--exclude-json')
    ap.add_argument('--skip', action='append', default=[])
    ap.add_argument('--identity', action='store_true')
    ap.add_argument('--only')
    ap.add_argument('--no-simd-canon', action='store_true', help='TEST ONLY (negative control of test_melee_export.py): keep the sign of the '
                                                                  'w < 0 keys of SIMD clips, i.e. the pre-2026-11-10 behaviour that the importer corrupts')
    ap.add_argument('--reject-json', help='{clip: reason} from a previous attempt (round-trip / import gate): not built, the clip stays vanilla')
    a = ap.parse_args()
    rej = json.load(open(a.reject_json, encoding='utf-8-sig')) if a.reject_json else {}
    s = a.set
    vclips, rig = L.anims_json_clips(a.vanilla_json)
    sk = S.Skeleton.from_glb(a.vanilla_glb)
    skn = S.Skeleton.from_glb(a.vanilla_glb_nort)
    assert len(sk.names) == 152 and sk.names == skn.names, len(sk.names)
    excl = {}
    if a.exclude_json:
        ej = json.load(open(a.exclude_json, encoding='utf-8'))
        excl = {c: r for c, r in (ej.get('exclusion_reasons') or {}).get(s, {}).items()}
    skip = set(DEFAULT_SKIP) | set(a.skip)
    only = set(a.only.split(',')) if a.only else None
    body = [i for i in range(73) if sk.names[i] not in L.KEEP_VANILLA_BODY]
    rep = dict(set=s, rig=L.RIG_OF_SET[s], rig_path=rig, identity=bool(a.identity), input=a.input, suffix=a.suffix,
               vanilla_glb=a.vanilla_glb, vanilla_glb_sha256=L.sha256(a.vanilla_glb), vanilla_json=a.vanilla_json, clips={}, not_built={})
    out_clips = []
    if a.input and not a.identity and os.path.isdir(a.input):
        # input clips whose name the (augmented) vanilla set does not have: not built (listed; e.g. an NPC-only clip name, see
        # L.UTPP_CLIP_TEMPLATES / aug_vanilla.py for the UTPP-only names that are accepted)
        pre, post = '%s__' % s, '.%s.npz' % a.suffix
        rep['input_not_in_vanilla'] = sorted(f[len(pre):-len(post)] for f in os.listdir(a.input)
                                             if f.startswith(pre) and f.endswith(post) and f[len(pre):-len(post)] not in vclips)
    for c, info in vclips.items():
        if only and c not in only:
            continue
        if info['animationType'] in ('Additive', 'AdditiveWithoutFirstFrame'):
            rep['not_built'][c] = 'additive'
            continue
        if '%s/%s' % (s, c) in skip:
            rep['not_built'][c] = 'skipped (degenerate input / --skip)'
            continue
        if c in excl:
            rep['not_built'][c] = 'excluded (priority window): %s' % '; '.join(sorted(set(excl[c])))
            continue
        if c in rej:
            rep['not_built'][c] = 'rejected after attempt 1: %s' % rej[c]
            continue
        src = None
        if not a.identity:
            src = os.path.join(a.input, '%s__%s.%s.npz' % (s, c, a.suffix))
            if not os.path.exists(src):
                rep['not_built'][c] = 'no input clip'
                continue
        T, dur = int(info['numFrames']), float(info['duration'])
        if abs(dur - (T - 1) / L.FPS) > 1e-3:
            rep['not_built'][c] = 'duration %.4f != (numFrames-1)/30 (%d)' % (dur, T)
            continue
        ltm, _, ex, chans = L.sample_clip(sk, c, T, dur)                         # export with root motion: extras + the root motion (report only)
        lt0, lr0, _, _ = L.sample_clip(skn, c, T, dur, root_seconds=True)       # anim buffer frame (Root = buffer Root)
        vWt, vWr = skn.world(lt0, lr0)
        Wt, Wr = vWt.copy(), vWr.copy()
        crow = dict(frames=T, duration=dur, vanilla_root_motion_cm=round(float(np.linalg.norm(ltm[:, 0], axis=-1).max() * 100), 4),
                    buffer_root_max_cm=round(float(np.linalg.norm(lt0[:, 0], axis=-1).max() * 100), 4))
        if a.identity:
            g = None
        else:
            z = np.load(src)
            W = z['W'].astype(np.float64)
            names = [str(x) for x in z['names']]
            assert names == sk.names[:73], 'joint order of %s differs from the rig' % src
            if W.shape[0] != T:
                rep['not_built'][c] = 'input frames %d != vanilla numFrames %d' % (W.shape[0], T)
                continue
            if np.abs(W[:, :, :3, 3]).max() < 1e-6:
                rep['not_built'][c] = 'degenerate input (every joint at the origin)'
                continue
            g = L.C[None, None] @ W @ L.CI[None, None]
            crow['input_sha256'] = L.sha256(src)
            info_p = src[:-4] + '.info.json'
            if os.path.exists(info_p):
                inf = json.load(open(info_p, encoding='utf-8'))
                crow['input_info'] = dict(identical_to_v3=inf.get('identical_to_v3'), segments_mb=inf.get('segments_mb'), segments_fallback=inf.get('segments_fallback'),
                                          segments_v3=inf.get('segments_v3'))
            rdev = float(np.abs(g[:, 0, :3, 3] - vWt[:, 0]).max() * 100)
            rmat = float(np.abs(g[:, 0, :3, :3] - vWr[:, 0]).max())
            crow['root_input_vs_buffer_cm'] = round(rdev, 4)
            crow['root_input_vs_buffer_rotmat'] = rmat
            if rdev > ROOT_GATE_CM or rmat > 1e-3:
                rep['not_built'][c] = 'input Root differs from the vanilla buffer Root by %.4f cm / rotation-matrix %.2e (gate %.2f cm / 1e-3)' % (rdev, rmat, ROOT_GATE_CM)
                continue
            for nm in ('Trajectory', 'reference_joint'):
                j = sk.index[nm]
                crow['%s_input_vs_vanilla_cm' % nm] = round(float(np.linalg.norm(g[:, j, :3, 3] - vWt[:, j], axis=-1).max() * 100), 2)
            for j in body:
                Wt[:, j] = g[:, j, :3, 3]
                Wr[:, j] = g[:, j, :3, :3]
        flt, flr = sk.local_from_world(Wt, Wr)
        lt, lr = lt0.copy(), lr0.copy()
        lt[:, body] = flt[:, body]
        lr[:, body] = flr[:, body]
        done = ES.reanchor_helpers(sk, lt, lr, vWt, vWr, -1, T)
        for nm in ('arm_left_ik_offset', 'arm_right_ik_offset', 'arm_left_ik_weapon', 'arm_right_ik_weapon'):
            j = sk.index[nm]
            lt[:, j], lr[:, j] = lt0[:, j], lr0[:, j]
        snapped = snap_hand_helpers(sk, lt, lr)       # hand helpers farther than HAND_LIMIT_CM from the final hand -> on the hand
        if s in L.WEAPON_HELPER_SETS:                 # arm_*_ik_weapon -> on its weapon joint (sliding-grip sets only)
            crow['weapon_helpers_snapped'] = {k: len(v) for k, v in snap_weapon_helpers(sk, lt, lr).items()}
        lr = S.qcontinuous(lr)
        # checks on the built clip (before import; anim buffer frame)
        nWt, nWr = sk.world(lt, lr)
        # the Root track written to the GLB = the vanilla anim buffer Root (Root is not a body joint, so lt / lr keep lt0 / lr0 there): the importer
        # stores the GLB Root track in the buffer, a Root carrying the root motion would apply it twice in game (motionExtraction is copied verbatim)
        assert np.array_equal(lt[:, 0], lt0[:, 0]), 'Root track must stay the vanilla buffer Root'
        lt_out, lr_out = lt, lr
        if g is not None:
            eb = np.linalg.norm(nWt[:, body] - g[:, body][:, :, :3, 3], axis=-1).max() * 1000
            crow['body_vs_input_mm'] = round(float(eb), 4)
        cam = [sk.index[n] for n in L.FPP_CAMERA]
        crow['fpp_camera_vs_vanilla_mm'] = round(float(np.linalg.norm(nWt[:, cam] - vWt[:, cam], axis=-1).max() * 1000), 4)
        reanch = set(done) | {'arm_left_ik_offset', 'arm_right_ik_offset', 'arm_left_ik_weapon', 'arm_right_ik_weapon'}
        other = [j for j in range(73, 152) if sk.names[j] not in reanch]
        crow['other_locals_vs_vanilla'] = dict(t_mm=round(float(np.abs(lt[:, other] - lt0[:, other]).max() * 1000), 4),
                                               r_deg=round(float(L.quat_angle_deg(lr[:, other], lr0[:, other]).max()), 4))
        dn, dv = helper_dist(nWt, sk), helper_dist(vWt, sk)
        hel = {}
        for h in dn:
            lim = L.HAND_LIMIT_CM if 'Hand' in L.GATE_HELPERS[h] else L.FOOT_LIMIT_CM
            over = np.nonzero(dn[h] > lim + 1e-6)[0]
            hel[h] = dict(max_cm=round(float(dn[h].max()), 3), vanilla_max_cm=round(float(dv[h].max()), 3),
                          dev_from_vanilla_cm=round(float(np.abs(dn[h] - dv[h]).max()), 4),
                          frames_over_limit=over.tolist(), over_limit_inherited=bool(len(over) == 0 or (dv[h][over] > lim + 1e-6).all()))
        crow['helpers'] = hel
        crow['reanchored'] = sorted(done)
        crow['hand_helpers_snapped'] = {h: dict(n=len(f), frames=f) for h, f in snapped.items()}
        crow['hand_helpers'] = hand_helper_report(nWt, nWr, vWt, sk)
        times = np.arange(T) / L.FPS
        crow['buffer'] = info.get('buffer')
        if info.get('buffer') == 'animAnimationBufferSimd':
            t32 = np.nextafter(np.float32((T - 1) / L.FPS), np.float32(np.inf))
            times[-1] = float(t32)
            crow['simd_last_key_s'] = float(t32)
        ex_out = copy.deepcopy(ex)
        crow['fallback_frames_vanilla'] = len(ex_out.get('fallbackFrameIndices') or [])
        ex_out['fallbackFrameIndices'] = []
        lq = S.qcontinuous(lr_out)
        if info.get('buffer') == 'animAnimationBufferSimd' and not a.no_simd_canon:
            # uncompressed (SIMD) buffers: every key w >= 0 (L.canonical_w_nonneg; keys with w < 0 came back wrong from the importer)
            crow['simd_w_neg_keys'] = int((lq[..., 3] < 0).sum())
            lq = L.canonical_w_nonneg(lq)
        out_clips.append((c, times, lt_out, lq, ex_out))
        rep['clips'][c] = crow
    if not out_clips:
        raise SystemExit('nothing to build for %s' % s)
    L.write_glb_multi(sk, a.out_glb, out_clips)
    rep['out_glb'] = a.out_glb
    rep['out_glb_sha256'] = L.sha256(a.out_glb)
    rep['built'] = [c for c, *_ in out_clips]
    hmax = max(h['max_cm'] for r in rep['clips'].values() for h in r['helpers'].values())
    hdev = max(h['dev_from_vanilla_cm'] for r in rep['clips'].values() for h in r['helpers'].values())
    over = sum(1 for r in rep['clips'].values() for h in r['helpers'].values() if h['frames_over_limit'])
    newover = sum(1 for r in rep['clips'].values() for h in r['helpers'].values() if not h['over_limit_inherited'])
    hh = [(c, h, v) for c, r in rep['clips'].items() for h, v in r['hand_helpers'].items() if h in HAND_SNAP_ORDER]
    hand_over = sum(len(v['frames_over_limit']) for _, _, v in hh)
    rep['summary'] = dict(built=len(out_clips), not_built=len(rep['not_built']), helper_max_cm=hmax, helper_dev_from_vanilla_cm=hdev,
                          helper_tracks_over_limit=over, helper_tracks_over_limit_not_inherited=newover,
                          hand_helper_max_cm=max(v['max_cm'] for _, _, v in hh), hand_helper_last_frame_max_cm=max(v['last_cm'] for _, _, v in hh),
                          hand_helper_rot_max_deg=max(v['rot_max_deg'] for _, _, v in hh), hand_helper_frames_over_limit=hand_over,
                          hand_helper_tracks_snapped=sum(len(r['hand_helpers_snapped']) for r in rep['clips'].values()),
                          hand_helper_frames_snapped=sum(v['n'] for r in rep['clips'].values() for v in r['hand_helpers_snapped'].values()),
                          hand_helper_clips_snapped=sorted(c for c, r in rep['clips'].items() if r['hand_helpers_snapped']),
                          body_vs_input_mm=max((r.get('body_vs_input_mm', 0) for r in rep['clips'].values()), default=0),
                          fpp_camera_vs_vanilla_mm=max(r['fpp_camera_vs_vanilla_mm'] for r in rep['clips'].values()),
                          other_locals_t_mm=max(r['other_locals_vs_vanilla']['t_mm'] for r in rep['clips'].values()),
                          other_locals_r_deg=max(r['other_locals_vs_vanilla']['r_deg'] for r in rep['clips'].values()))
    L.jdump(rep, a.report)
    sm = rep['summary']
    print('MELEE_BUILD %s built %d not built %d | body %.4f mm | fpp cam %.4f mm | other locals %.4f mm %.4f deg | helpers max %.2f cm, dev vs vanilla %.4f cm, '
          'tracks over limit %d (not inherited %d) | hand helpers max %.2f cm (last frame %.2f cm), over %.1f cm: %d frames, put on the hand: %d '
          'tracks / %d frames in %d clips -> %s' % (s, len(out_clips), len(rep['not_built']), sm['body_vs_input_mm'], sm['fpp_camera_vs_vanilla_mm'],
                                                    sm['other_locals_t_mm'], sm['other_locals_r_deg'], hmax, hdev, over, newover, sm['hand_helper_max_cm'],
                                                    sm['hand_helper_last_frame_max_cm'], L.HAND_LIMIT_CM, hand_over, sm['hand_helper_tracks_snapped'],
                                                    sm['hand_helper_frames_snapped'], len(sm['hand_helper_clips_snapped']), a.out_glb))
    return 0 if newover == 0 and hand_over == 0 else 3


if __name__ == '__main__':
    sys.exit(main())
