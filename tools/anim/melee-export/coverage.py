"""Coverage table of a melee export run (for the DLL coverage table; 2026-11-09).

  python -I coverage.py --run <run dir> --registration <evidence\\registration.json> --dumps <melee-anim-20261107\\npz> --out <prefix>

Per set: clips total / non-additive / additive, redone (in the packed .anims and verified), of those fallback (re-solved clip identical to v3b:
info 'identical_to_v3'), not built (no input / excluded / degenerate / failed), the vanilla registration paths (variables, priority, mirrored or
not) and the item types they are meant for (INFERRED, see ITEM_TYPES); per additive clip the bone groups it animates, from the vanilla dump
(rotation > 0.5 deg or translation > 0.2 cm away from the reference on any frame) with a flag for Hips / legs (a full-body handover exposes
them). The exported additive clips hold ABSOLUTE locals (frame 0 of add_melee_idle = the rest pose), so the reference is the rest pose
(npz rest_lt / rest_lr) for 'Additive' and frame 0 for 'AdditiveWithoutFirstFrame' (assumed engine semantics, not verified in game).
Clips without input: the re-solve batch (Source\\tools\\melee\\redo_batch.py) skips clips with fewer than 6 frames (held pose clips).
Writes <prefix>.json and <prefix>.txt.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402

# set variable -> gamedataItemType (INFERRED from the set names and the default fists item Items.w_melee_004__fists_a of
# equipmentSystem.script; not read from TweakDB). The game selects the set by the VARIABLE; key the DLL table on it.
ITEM_TYPES = {
    '': 'fallback for every melee weapon (clip names missing in the weapon set)',
    'w_melee_fists': 'Wea_Fists (inferred)', 'StrongArms': 'Cyb_StrongArms (inferred)', 'w_melee_katana': 'Wea_Katana (inferred; other blades unverified)',
    'w_melee_knife': 'Wea_Knife (inferred)', 'knife_thumb_override': 'unknown', 'w_melee_one_hand_blunt': 'Wea_OneHandedClub (inferred)',
    'additives': 'unknown (extra source for other weapons)', 'w_two_hand_blunt': 'Wea_TwoHandedClub (inferred)',
    'w_melee_hammer': 'Wea_Hammer (TweakDB Base_Two_Hand_Hammer: hammers, Caretaker / shovel)',
}
GROUPS = {
    'hips': ['Hips'], 'spine': ['Spine', 'Spine1', 'Spine2', 'Spine3'], 'neck_head': ['Neck', 'Neck1', 'Head'],
    'legs': ['LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase'],
    'arms': ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand'],
    'root': ['Root', 'Trajectory'],
}


def additive_bones(path, anim_type):
    z = np.load(path, allow_pickle=False)
    names = [str(x) for x in z['names152']]
    lt, lr = z['lt'].astype(np.float64), z['lr'].astype(np.float64)
    if anim_type == 'AdditiveWithoutFirstFrame':
        rt, rr, ref = lt[0], lr[0], 'frame 0'
    else:
        rt, rr, ref = z['rest_lt'].astype(np.float64), z['rest_lr'].astype(np.float64), 'rest pose'
    ang = L.quat_angle_deg(lr, np.broadcast_to(rr, lr.shape)).max(0)
    tr = np.linalg.norm(lt - rt[None], axis=-1).max(0) * 100
    out = dict(reference=ref)
    for g, bones in GROUPS.items():
        ix = [names.index(b) for b in bones if b in names]
        a_, t_ = float(ang[ix].max()), float(tr[ix].max())
        out[g] = dict(rot_deg=round(a_, 2), trans_cm=round(t_, 2), acts=bool(a_ > 0.5 or t_ > 0.2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--registration', required=True)
    ap.add_argument('--dumps', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    reg = json.load(open(a.registration, encoding='utf-8'))
    man = json.load(open(os.path.join(a.run, 'manifest.json'), encoding='utf-8-sig'))
    staged = {os.path.basename(x['path']) for x in (man.get('staged') or [])}
    res = dict(run=a.run, archive=man.get('archive'), sets={})
    for s in L.SETS:
        if not os.path.exists(os.path.join(a.dumps, s, '_index.json')) or s not in reg['plan']:
            continue        # a set the dumps / the registration do not cover (e.g. baton, StrongArms in a run of the first export)
        idx = json.load(open(os.path.join(a.dumps, s, '_index.json'), encoding='utf-8'))
        add = sorted(c for c, v in idx.items() if v['additive'])
        bp, vp = os.path.join(a.run, 'reports', 'build-%s.json' % s), os.path.join(a.run, 'reports', 'verify-%s.json' % s)
        built = json.load(open(bp, encoding='utf-8')) if os.path.exists(bp) else {'clips': {}, 'not_built': {}}
        ver = json.load(open(vp, encoding='utf-8')) if os.path.exists(vp) else {'clips': {}}
        packed = L.utpp_name(s) in staged
        redone = sorted(c for c in built['clips'] if packed and ver['clips'].get(c, {}).get('ok'))
        fallback = sorted(c for c in redone if (built['clips'][c].get('input_info') or {}).get('identical_to_v3'))
        nb = {c: r for c, r in built.get('not_built', {}).items() if r != 'additive'}
        failed = sorted(c for c in built['clips'] if c not in redone)
        addb = {}
        for c in add:
            p = os.path.join(a.dumps, s, c + '.npz')
            if os.path.exists(p):
                addb[c] = additive_bones(p, idx[c].get('animation_type'))
        noin = {c: dict(frames=idx[c]['frames'], duration=idx[c]['duration'],
                        why='fewer than 6 frames: the re-solve batch (redo_batch.py) skips held pose clips' if idx[c]['frames'] < 6 else 'not in the input folder')
                for c, r in nb.items() if r == 'no input clip' and c in idx}
        lower = sorted(c for c, g in addb.items() if g['hips']['acts'] or g['legs']['acts'])
        paths = [dict(vars=r['vanilla_vars'], priority=r['vanilla_priority'], component=r['component'], mirrored=r['mirrored'],
                      utpp_vars=r.get('utpp_vars'), utpp_priority=r.get('utpp_priority'), item_types=ITEM_TYPES.get(','.join(r['vanilla_vars']), 'unknown'))
                 for r in reg['plan'][s] if not r['entity'].endswith('_ep1')]
        res['sets'][s] = dict(rig=L.RIG_OF_SET[s], vanilla=L.VANILLA_DEPOT[s], utpp=(L.UTPP_DEPOT_DIR + '\\' + L.utpp_name(s)) if packed else None,
                              clips_total=len(idx), non_additive=len(idx) - len(add), additive=len(add), redone=len(redone), fallback_v4_eq_v3b=len(fallback),
                              not_built=len(nb), failed=len(failed), redone_clips=redone, fallback_clips=fallback, not_built_clips=nb, failed_clips=failed,
                              no_input_clips=noin, rejected_clips={c: r for c, r in nb.items() if r.startswith('rejected')},
                              additive_clips_lower_body=lower, additive_bones=addb, paths=paths)
    L.jdump(res, a.out + '.json')
    with open(a.out + '.txt', 'w', encoding='utf-8') as f:
        f.write('MELEE COVERAGE  run %s\n' % a.run)
        if res['archive']:
            f.write('archive %s sha256 %s size %s\n' % (res['archive']['path'], res['archive']['sha256'], res['archive']['size']))
        f.write('\nFOR THE DLL: non-additive clips WITHOUT a re-solved version (vanilla first-person body incl. static legs under a full-body handover):\n')
        for s, r in res['sets'].items():
            if r['no_input_clips']:
                f.write('  %-26s %s\n' % (s, ', '.join('%s (%d fr)' % (c, v['frames']) for c, v in sorted(r['no_input_clips'].items()))))
        whys = sorted({v['why'] for r in res['sets'].values() for v in r['no_input_clips'].values()})
        f.write('  reason: %s\n' % ' / '.join(whys))
        f.write('REJECTED by a gate in attempt 1 (rebuilt without them; they stay vanilla):\n')
        for s, r in res['sets'].items():
            for c, why in sorted(r['rejected_clips'].items()):
                f.write('  %-26s %-28s %s\n' % (s, c, why[:220]))
        f.write('\n')
        f.write('%-26s %5s %6s %5s | %6s %8s %8s %6s | additive acting on hips/legs\n' % ('set', 'total', 'nonadd', 'add', 'redone', 'fallback', 'notbuilt', 'failed'))
        for s, r in res['sets'].items():
            f.write('%-26s %5d %6d %5d | %6d %8d %8d %6d | %d: %s\n' % (s, r['clips_total'], r['non_additive'], r['additive'], r['redone'], r['fallback_v4_eq_v3b'],
                                                                       r['not_built'], r['failed'], len(r['additive_clips_lower_body']), ','.join(r['additive_clips_lower_body'])))
        f.write('\nSET -> registration paths (vars @ vanilla priority -> UTPP entry) -> item types (inferred)\n')
        for s, r in res['sets'].items():
            for p in r['paths']:
                f.write('  %-26s %-28s @%d -> %-48s %s\n' % (s, p['vars'], p['priority'], ('%s @%d' % (p['utpp_vars'], p['utpp_priority'])) if p['mirrored'] else 'vanilla (not mirrored)',
                                                         p['item_types']))
        f.write('\nNOT BUILT (stay vanilla in the UTPP set path, i.e. the vanilla clip plays)\n')
        for s, r in res['sets'].items():
            for c, why in sorted(r['not_built_clips'].items()):
                f.write('  %-26s %-40s %s\n' % (s, c, why))
    print('MELEE_COVERAGE ' + ' | '.join('%s %d/%d' % (s, r['redone'], r['non_additive']) for s, r in res['sets'].items()) + ' -> ' + a.out)


if __name__ == '__main__':
    main()
