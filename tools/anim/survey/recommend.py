"""Stage 6 note (contact sheet) and stage 7 RECOMMENDATIONS for the DLL anatomy-guard round -> report.txt (appended once per stage).

  python -I recommend.py
Reads guard_limits_{woman,man}.json, proto_<rig>.json, fixtures\\<rig>\\index.json of Build\\work\\anim-survey-20261109.
"""
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

W = SC.WORK
PNG = os.path.join(SC.ROOT, 'Media', 'png', '2026-11-09_anim-deformation-survey', 'female_worst_vanilla_vs_guard.png')


def load(name):
    p = os.path.join(W, name)
    return json.load(open(p, encoding='utf-8')) if os.path.exists(p) else None


def state_of(k, v):
    st = v['state']
    if 'exploration' in st:
        c = k.split('/')[1]
        st = 'ladder' if c.startswith('ladder') else 'knockdown' if 'knockdown' in c else 'death' if c.startswith('death') else \
            'swim' if 'swim' in c else 'vault/climb'
    return st


def tier(v):
    if v['verdict'] != 'GO':
        return 'NO-GO'
    return 'GO-fix' if v['fail_pct_guard'] <= 25.0 and v['fail_pct_guard'] <= v['fail_pct_vanilla'] else 'SAFE-NO-FIX'


def stage6():
    pr = load('proto_woman.json')
    keys = sorted(pr, key=lambda k: -pr[k]['fail_pct_vanilla'] - 10 * pr[k]['skin_excess_vanilla'])[:12]
    L = ['PNG: %s (1840 x 1926, survey_sheet.py --rig woman --n 12)' % PNG,
         '12 rows = the 12 prototype clips with the highest vanilla fail%% + 10 x skin excess; per row: vanilla 3/4 front | guard 3/4 front |',
         'vanilla side | guard side at the worst frame (middle of the +-1 triple), label = clip, frame, verdict, fail%, skin, hand cm, pop.',
         'rows: ' + '; '.join('%s (%s)' % (k, pr[k]['verdict']) for k in keys),
         'caveats: software render (PIL painter, flat shading, orthographic) of the OFFLINE skinned body (demo_skin graph deformation of',
         'the female V body mesh), no clothes / hair / game shaders - shapes only, not a game capture. The Cerberus takedown sync frame',
         '(pwa_005__militech_cerberus_takedown_sync.ep1/finisher, f57) is upside down in BOTH tiles: that is the world orientation of the',
         'frame in the dump (sync animation relative to the mech), not a guard effect. Visible = derived from code (stage 1), not from game.']
    SC.report('STAGE 6 CONTACT SHEET (female, vanilla vs guard)', '\n'.join(L))


def limits_table():
    a, b = load('guard_limits_woman.json'), load('guard_limits_man.json')
    L = ['Files: %s\\guard_limits_woman.json and guard_limits_man.json (also copied into fixtures\\<rig>\\index.json "limits").' % W,
         'Representation (both bodies, the rig\'s own joint frames, no world-space features): Rel = Lrest^T * L (local rotation relative to',
         'the rest local); q = swing * twist; twist = signed angle about local +X (the bone axis: +X on the left side / spine, -X on the',
         'right side, checked from the child rest translation); swing = rotation vector (0, swing_y, swing_z). Limits = [p0.5 - tol, p99.5',
         '+ tol] over the GASP-ALS corpus on the V rig of the body (%s frames: base_body.RANGE_PATTERNS + shipped overlay poses), tol 5 deg,'
         % (a and a['frames']),
         'legs 15 deg; knee swing_z lower bound -150 (human flexion); hand twist = pronation bound [-130, 130] (game convention: forearm =',
         'pure hinge, pronation roll in the hand, ForearmTwist.hpp). trans = max deviation of the local translation from the rest local.',
         '%-14s | %-48s | %-48s' % ('joint', 'WOMAN  twist / swing_y / swing_z (deg), trans cm', 'MAN  twist / swing_y / swing_z (deg), trans cm')]
    for n in a['joints']:
        def cell(d):
            if not d or n not in d['joints']:
                return 'n/a'
            j = d['joints'][n]
            return '[%6.1f %6.1f] [%6.1f %6.1f] [%6.1f %6.1f] %.2f' % (*j['twist'], *j['swing_y'], *j['swing_z'], j['trans_dev_cm'])
        L.append('%-14s | %-48s | %-48s' % (n, cell(a), cell(b)))
    L.append('structural band in the Spine3 frame (survey only, not a clamp): shoulder_width_s3 woman %s man %s cm, clav_gap_s3 >= %s / %s cm'
             % (a['shoulder_width_s3'], b and b['shoulder_width_s3'], a['clav_gap_s3'][0], b and b['clav_gap_s3'][0]))
    return L


def stage7():
    L = ['--- 7.1 per-joint local limits table (DLL clamp table) ---'] + limits_table()
    L += ['', '--- 7.2 translation reset rules ---',
          'Every guarded joint (Spine..Head, clavicles, arms, forearms, hands, up legs, legs, feet): the local translation deviation from the',
          'rest local is soft-limited to trans_dev_cm = 0.50 cm (corpus p99.5 = 0.00 cm on every guarded joint of both bodies: the GASP-ALS',
          'skeleton is rigid) with a tanh knee from 0.25 cm, i.e. in practice a reset to the rest local. This removes the FPP stretch that no',
          'rotation clamp can fix (female: clavicle local shifts 5-37 cm, car body_out LeftShoulder 31.4 cm, Spine3 17.3 cm, seg_len 168%).',
          'Never touch Root / Hips (root motion, pelvis placement), fingers, twist / roll helpers, weapon and IK helper bones (arm_*_ik*):',
          'the guard restores the hand world position instead, so the IK helpers stay valid only where the hand residual is ~0 (GO states).',
          '', '--- 7.3 smoothing / continuity constants (measured on the prototype unless marked proposed) ---',
          'K = 5 deg tanh knee per axis (identity inside [lo + 5, hi - 5] = the corpus p0.5..p99.5 band, C1, saturates at the limit; same',
          'shape as MeshArmClavicleLimit SoftLimitAngleDeg). Clamp alone adds at most 5.7 deg one-frame rotation vs vanilla (female 23 clips).',
          'Twist branch: per-joint previous-frame state (prototype: np.unwrap from the clip start); reset the state on state entry.',
          'Hinge IK: elbow / knee bent only about local Z (flex = -swing_z, never reversed), hinge side latched at the episode start, pole',
          'blended with the previous frame w = min(1, |pole| / 5 cm)^2; restore the hand world rotation, then the wrist soft clamp.',
          'Pop rule used for GO: guard rot_step - vanilla rot_step <= 10 deg per frame at 30 fps.',
          'Proposed (not measured): guard weight ramp 0.15 s on state enter / exit (the game pose itself jumps between FPP-authored states),',
          'and a per-frame rate limit on the guard correction so single-frame data flips are not chased.']
    L += ['', '--- 7.4 states to apply / skip (prototype per body, ONE representative clip per state) ---',
          'tiers: GO-fix = hand <= 2 cm, skin in band, pop <= 10 deg AND the guard removes the gate failures (guard fail% <= 25 and not',
          'above vanilla); SAFE-NO-FIX = the same no-harm checks pass but the deformation remains (fail% stays high or grows): harmless, NOT',
          'a fix; NO-GO = hand / skin / pop check fails. The skin check rarely decides (vanilla worst frames mostly inside the band, blind to',
          'rigid shifts), so the PROTO "GO" (STAGE 5 blocks) means "hands kept, no pop", not "deformation removed".']
    T = {}
    for rig in ('woman', 'man'):
        for k, v in (load('proto_%s.json' % rig) or {}).items():
            T.setdefault((v['cls'], state_of(k, v)), {})[rig] = (tier(v), v, k)

    def ent(c, t, v):
        if t == 'GO-fix':
            return '%s %s (%.0f->%.0f%%)' % (c[0], c[1], v['fail_pct_vanilla'], v['fail_pct_guard'])
        if t == 'SAFE-NO-FIX':
            return '%s %s (%.0f->%.0f%%, residual %s %.1f)' % (c[0], c[1], v['fail_pct_vanilla'], v['fail_pct_guard'], *v['worst_guard'])
        return '%s %s (hand %.1f cm, pop %.0f)' % (c[0], c[1], v['hand_residual_cm_max'], v['pop_deg'])
    for rig in ('woman', 'man'):
        for t in ('GO-fix', 'SAFE-NO-FIX', 'NO-GO'):
            L.append('%s %s: %s' % (rig, t, '; '.join(ent(c, t, d[rig][1]) for c, d in sorted(T.items()) if rig in d and d[rig][0] == t)))
    both = [c for c, d in sorted(T.items()) if all(r in d and d[r][0] == 'GO-fix' for r in ('woman', 'man'))]
    unres = [c for c, d in sorted(T.items()) if c not in both and any(x[0] == 'GO-fix' for x in d.values())]
    safe = [c for c, d in sorted(T.items()) if c not in both and c not in unres and any(x[0] == 'SAFE-NO-FIX' for x in d.values())]
    L.append('RECOMMENDED APPLY (GO-fix on both bodies): ' + '; '.join('%s %s' % c for c in both))
    L.append('UNRESOLVED (GO-fix on one body only / missing / different representative clips): ' + '; '.join(
        '%s %s [%s]' % (c[0], c[1], ', '.join('%s %s %s' % (r, T[c][r][0], T[c][r][2].split('/', 1)[1]) for r in ('woman', 'man') if r in T[c]))
        for c in unres))
    L.append('HARMLESS BUT NOT FIXED (do not count as fixed; rebuild or a better guard): ' + '; '.join('%s %s' % c for c in safe))
    L.append('everything else: NO-GO -> offline TPP rebuild.')
    L += ['Apply only on the bones the game owns in that state (GameIntegration.hpp masks): release states (kPsmReleaseMask: vehicle, scene,',
          'swimming, workspot, takedown, carry, vitals; ladder details 10-13, knockdown / felled 29 / 31; climb / vault only when v2 does not',
          'own the traversal) = whole body; melee-mask states = IsMaskLowerBone false (Spine1 and up) only; weapon action states =',
          'IsWeaponActionBone (clavicles, arms, hands) only. Never on MM-owned joints. Held states (jump, fall, land, dodge, slide, stagger,',
          'menu) do not show the game pose: no guard. Firearm aim / idle with the shipped defaults (mm.motionDriver = true, aimGameUpper = false) = ALS',
          'overlays (class C): no guard; with aimGameUpper = true they become class B and are NOT covered by this survey.',
          'GO evidence is one clip per state: run the fixtures and a per-state in-game check before enabling a state.',
          '', '--- 7.5 interplay with the existing corrections (stage 2) ---',
          'Host: FkHook.cpp ApplyPoseSnapshot (FK hook, after the game pose is final for game-owned bones), as a new header-only module next',
          'to the other corrections; NativePoseCorrection is not a natural host. Reload / quick melee: NativeReloadContacts already fixes the',
          'contacts and limits clavicle reach to <= 20 deg -> NO guard there (prototype NO-GO: FPP hand targets out of reach after the',
          'translation reset, IK pops up to 104 deg); no double correction. FeminineShoulders / MeshArmClavicleLimit act on the MM pose only',
          '(no overlap if the guard is masked to game-owned bones). MaleBodyCorrection and NativePoseCorrection have no call sites (inactive).',
          'TPP-authored sets (8, triage class D) and the NPC-based TPP melee rebuild need no guard.',
          '', '--- 7.6 risks ---',
          '1. Unreachable FPP hand targets: after the clavicle translation reset the hand target leaves the rigid arm reach (female residual',
          '   5-49 cm: car body_out / switch_seat, sniper nest, takedown leap, vault, hammer, blueprint) -> hands leave doors, wheel, weapons.',
          '2. IK pops (hinge-plane flips, wrist branch cut) up to 129 deg where the clamped elbow plane differs strongly from the vanilla one;',
          '   needs an elbow swivel-angle limit, per-joint previous-frame state and the rate limit.',
          '3. Independent swing_y / swing_z clamps form a box, not a cone: corner poses beyond the corpus pass (cone / ellipse limit later).',
          '4. Residual upper-arm twist from the IK (cw_safe 72 deg, ladder look-at) and leg-IK hip twist not limited yet.',
          '5. Data: 102 female clips have one-frame body-bone flips > 120 deg (paired within 10 frames, suspected export / decompression',
          '   artefacts, glbanim slerp is already shortest-path) and 6 finisher exports are degenerate (all translations zero): verify in',
          '   game before treating them as real; the guard must not chase single-frame flips.',
          '6. The skinned-strain statistic (redo_skin_check, within-region edges) is blind to rigid region shifts; the all-edge variant',
          '   (skin_all.py) is not discriminative (reference band 0.88-1.97): translation stretch is confirmed by the joint metric only.',
          '7. Visibility and the B hybrid are derived from code (masks, static MM base pose), not from the game; the prototype is offline',
          '   numpy, not the DLL; an unexplained ~7.4 % seg_len residual remains in some guarded clips.',
          '8. The hinge IK is not re-clamped (check_ik_limits.py, female prototype): after the IK, 20 of 23 clips end with an arm / forearm /',
          '   hand local > 5 deg beyond the guard\'s own limits (vanilla: also most); worst = upper-arm TWIST: cw_safe 135 deg, ladder look-at',
          '   130, switch_seat 124, vault 121 (= the uparm_twist residuals, a real deformation); in the GO-fix clips it is upper-arm SWING',
          '   20-40 deg beyond the locomotion corpus band (blocks, carry, throws), which the human-limit gate accepts. The DLL needs an elbow',
          '   swivel choice that keeps the Arm twist inside its limits (pole from the clamped pose), or a re-clamp that gives up hand contact.']
    for rig, sub in [(r, s) for r in ('woman', 'man') for s in ('', 'apply')]:
        ix = load(os.path.join('fixtures', rig, sub, 'index.json'))
        if not ix:
            continue
        L += ['', '--- 7.7 C++ test fixtures (%s%s): %s (index.json + %d files, make_fixtures.py) ---' % (
            rig, ', guard-applied states' if sub else ', top severity', os.path.join(W, 'fixtures', rig, sub), len(ix['fixtures'])),
              'worst frame +-1 as LOCAL poses (73 joints: name, parent, rest local t / q; per frame vanilla / clamp / guard local t / q,',
              'decomposition wrapped + unwrapped, gate excess, hand residual); selection: %s' % ix['selection']]
        for f in ix['fixtures']:
            L.append('  %-70s %s %-38s f%-4d sev %5.2f %-20s hand %.2f cm' % (f['file'][:70], f['cls'], f['state'][:38], f['worst_frame'],
                                                                              f['severity'], f['worst_feature'], f['hand_residual_cm']))
    SC.report('STAGE 7 RECOMMENDATIONS FOR THE DLL GUARD ROUND', '\n'.join(L))


def drop_block(prefix):
    """remove an earlier block of this script ('==== <prefix>...') so a re-run replaces it instead of appending a duplicate."""
    rep = open(SC.REPORT, encoding='utf-8').read()
    i = rep.find('\n==== ' + prefix)
    if i >= 0:
        j = rep.find('\n==== ', i + 1)
        with open(SC.REPORT, 'w', encoding='utf-8') as f:
            f.write(rep[:i] + (rep[j:] if j >= 0 else '\n'))


def stage8():
    m, pm, pw = load('metrics_man.json'), load('proto_man.json'), load('proto_woman.json')
    a, b = load('guard_limits_woman.json'), load('guard_limits_man.json')
    if not m:
        return
    vis = [v for v in m.values() if v['cls'] in ('A', 'B', 'E') and not v.get('degenerate_export')]
    L = ['male rig (player_man_skeleton, pma_* + male player_* sets): %d sets dumped, %d clips (%d visible A/B/E, degenerate excluded); tables in the',
         'STAGE 4 METRICS MAN block above (survey_clips_man.csv, survey_sets_man.csv), prototype in STAGE 5 ... MAN (proto_man.json).',
         'offline numbers only (no renders): skinned strain uses the male body + <REDO_MAN>\\skin_ref.json, not visually checked.']
    L[0] = L[0] % (len({v['setkey'] for v in m.values()}), len(m), len(vis))
    if pm and pw:
        sm, sw = collections.defaultdict(list), collections.defaultdict(list)
        for d, s in ((pm, sm), (pw, sw)):
            for k, v in d.items():
                s[(v['cls'], state_of(k, v))].append(tier(v))
        L.append('GO man %d / %d, woman %d / %d. Per state (woman | man):' % (sum(v['verdict'] == 'GO' for v in pm.values()), len(pm),
                                                                       sum(v['verdict'] == 'GO' for v in pw.values()), len(pw)))
        for c in sorted(set(sm) | set(sw)):
            fl = '' if (sw.get(c) == sm.get(c) or not sw.get(c) or not sm.get(c)) else '   <- differs'
            L.append('  %s %-40s %-10s | %-10s%s' % (c[0], c[1], '/'.join(sw.get(c, ['-'])), '/'.join(sm.get(c, ['-'])), fl))
    if a and b:
        L.append('limits: the 7.1 table (man column). Rest local translations (cm) woman vs man, the bone lengths behind the bands:')
        for n in ('LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'Neck', 'LeftUpLeg', 'LeftLeg', 'LeftFoot'):
            L.append('  %-13s %-26s %s' % (n, a['joints'][n]['rest_local_t_cm'], b['joints'][n]['rest_local_t_cm']))
        L.append('note: shoulder_width_s3 band woman %s vs man %s cm is near-identical because the two player rigs share the shoulder'
                 % (a['shoulder_width_s3'], b['shoulder_width_s3']))
        L.append('chain (clavicle offset 14.3 / 14.1 cm, clavicle 17.42, upper arm 29.04, forearm 27.53 cm in both); they differ in neck and legs.')
        L.append('The male FPP clips are near copies of the female ones (same worst frames and severities on reloads, car seats, doors).')
    drop_block('STAGE 8 ')
    SC.report('STAGE 8 MALE RIG (offline numbers)', '\n'.join(L))


def final():
    import hashlib
    import shutil
    src = os.path.dirname(os.path.abspath(__file__))
    import glob
    L = ['extraction / dumps (stage 3): %d raw .anims, %d json, %d glb (female 133 / 134: player_locomotion.f fails in WolvenKit export,'
         % tuple(len(glob.glob(os.path.join(d, '**', x), recursive=True)) for d, x in ((SC.RAW, '*.anims'), (SC.JSON_DIR, '*.json'),
                                                                                       (SC.GLB, '*.glb'))),
         'duplicate root-motion key, class C; male 128 / 128); dumps woman %d sets / %d clips, man %d sets / %d clips; rest check of the'
         % tuple(n for r in ('woman', 'man') for n in (len(glob.glob(os.path.join(SC.DUMPS, r, '*', '_index.json'))),
                                                         len(glob.glob(os.path.join(SC.DUMPS, r, '*', '*.npz'))))),
         'dumps: rest locals == rig rest (0.0000 deg, 0.00000 m).',
         'scripts (Source\\tools\\anim\\survey, SHA-256):']
    for f in sorted(os.listdir(src)):
        p = os.path.join(src, f)
        if os.path.isfile(p) and f.endswith(('.py', '.ps1')):
            L.append('  %s  %s' % (hashlib.sha256(open(p, 'rb').read()).hexdigest(), f))

    def du(p):
        return sum(os.path.getsize(os.path.join(r, x)) for r, _, fs in os.walk(p) for x in fs) if os.path.exists(p) else 0
    parts = [(SC.EXT, 'Extracted\\game\\player-anims')] + [(os.path.join(SC.EXT, k), '  ' + k) for k in ('raw', 'json', 'glb', 'logs')] + \
        [(W, 'Build\\work\\anim-survey-20261109')] + [(os.path.join(W, k), '  ' + k) for k in ('dumps', 'skin', 'fixtures')] + \
        [(os.path.dirname(PNG), 'Media\\png\\2026-11-09_anim-deformation-survey')]
    L.append('disk used:')
    for p, n in parts:
        L.append('  %-48s %8.2f GB' % (n, du(p) / 1e9))
    L.append('free on D: %.0f GB' % (shutil.disk_usage(SC.ROOT).free / 1e9))
    drop_block('RUN SUMMARY')
    SC.report('RUN SUMMARY (scripts, disk)', '\n'.join(L))


if __name__ == '__main__':
    txt = open(SC.REPORT, encoding='utf-8').read()
    if '==== STAGE 6 ' not in txt:
        stage6()
    if '--final' in sys.argv:
        final()
    else:
        drop_block('STAGE 7 ')
        stage7()
        stage8()
    print('report updated', SC.REPORT)
