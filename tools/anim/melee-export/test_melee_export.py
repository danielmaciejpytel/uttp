"""Offline checks of the melee export tools (2026-11-10; no WolvenKit, no game).  Python = Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I

  python -I test_melee_export.py [--vanilla <frozen vanilla dir>]

  1 canonical_w_nonneg: every key w >= 0, same rotation, keys with w >= 0 bit-identical (SIMD sign fix; the importer-level round trip of a clip
    with negative-w keys is test_simd_roundtrip.ps1)
  2 UTPP-only clip names (L.template_clip allowlist): accepted names map to parried_<side>, everything else is refused
  3 aug_vanilla.plan: new allowlisted names get a template, unknown names are ignored, a missing template is reported
  4 aug_vanilla end to end (needs the frozen vanilla dir with pma_knife; skipped without): clones in json / glb / glb-nort, unique handle and
    buffer ids, vanilla dir untouched, the original clip entries unchanged, sets without new names copied byte for byte
  5 WEAPON_HELPER_SETS: the merged list (hammer, one-handed blunt, baton, both default sets, knife, katana), and NOT fists / StrongArms /
    two-handed blunt / mantis blades; every set has a depot path and a rig and is known to run_melee_export.ps1
Exit 0 = all pass.
"""
import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402
import aug_vanilla as AV  # noqa: E402

FAILS = []


def check(cond, what):
    print(('PASS ' if cond else 'FAIL ') + what)
    if not cond:
        FAILS.append(what)


def test_canonical():
    rng = np.random.default_rng(7)
    q = rng.normal(size=(2000, 4))
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    q[:50, 3] = 0.0                                           # w == 0 stays as it is (not negative)
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    c = L.canonical_w_nonneg(q)
    check(bool((c[:, 3] >= 0).all()), 'canonical_w_nonneg: every w >= 0 (%d of %d keys had w < 0)' % (int((q[:, 3] < 0).sum()), len(q)))
    check(bool(np.allclose(np.abs(np.sum(c * q, axis=-1)), 1.0)), 'canonical_w_nonneg: same rotation (|dot| = 1)')
    keep = q[:, 3] >= 0
    check(bool(np.array_equal(c[keep], q[keep])), 'canonical_w_nonneg: keys with w >= 0 are bit-identical')
    t = q.reshape(40, 50, 4)
    check(L.canonical_w_nonneg(t).shape == t.shape, 'canonical_w_nonneg: keeps the (T, N, 4) shape')


def test_names():
    ok = ['parried_left_combo_01', 'parried_right_combo_04', 'parried_left_final_03', 'parried_right_final_05', 'parried_left_heavy_02',
          'parried_left_block', 'parried_right_safe', 'parried_left_crouch', 'parried_right_jump', 'parried_left_sprint', 'parried_right_equip']
    bad = ['parried_left', 'parried_right', 'parried_up_combo_01', 'parried_left_', 'parried_left_combo_1', 'parried_left_combo_001',
           'parried_left_combo_01_hit', 'parried_left_foo', 'guard_idle_loop', 'combo_attack_01', 'combo_attack_01_parried', 'xparried_left_block',
           'parried_left_combo_01 ', 'Parried_left_block', '']
    check(all(L.template_clip(n) == n.split('_')[0] + '_' + n.split('_')[1] for n in ok), 'template_clip: %d allowlisted names -> parried_<side>' % len(ok))
    check(all(L.template_clip(n) is None for n in bad), 'template_clip: %d unknown names refused' % len(bad))


def test_plan():
    v = {'parried_left', 'parried_right', 'combo_attack_01', 'parried_left_combo_01'}
    new, ign, mis = AV.plan(v, ['parried_left_combo_01', 'parried_right_combo_01', 'parried_left_block', 'guard_idle_loop', 'combo_attack_01'])
    check(new == [('parried_left_block', 'parried_left'), ('parried_right_combo_01', 'parried_right')] and ign == ['guard_idle_loop'] and not mis,
          'aug plan: new names get their template (existing names untouched), unknown ignored')
    new, ign, mis = AV.plan({'combo_attack_01'}, ['parried_left_combo_01'])
    check(not new and mis == [('parried_left_combo_01', 'parried_left')], 'aug plan: a missing template is reported')


def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()


def test_aug(vanilla):
    s = 'pma_knife'
    if not os.path.exists(os.path.join(vanilla, 'json', s + '.anims.json')):
        print('SKIP aug end-to-end: %s has no %s' % (vanilla, s))
        return
    import glbanim as G
    tmp = tempfile.mkdtemp(prefix='melee_aug_test_')
    try:
        inp = os.path.join(tmp, 'in')
        os.makedirs(inp)
        names = ['parried_left_combo_01', 'parried_right_combo_01', 'parried_left_safe', 'parried_right_safe', 'guard_idle_loop']
        for n in names:
            open(os.path.join(inp, '%s__%s.t.npz' % (s, n)), 'wb').close()
        before = {sub: sha(os.path.join(vanilla, sub, s + '.anims.' + ('json' if sub == 'json' else 'glb'))) for sub in ('json', 'glb', 'glb-nort')}
        out = os.path.join(tmp, 'out')
        rc = AV.main_args(['--vanilla', vanilla, '--input', inp, '--suffix', 't', '--sets', s, '--out', out])
        check(rc == 0, 'aug end-to-end: exit 0')
        after = {sub: sha(os.path.join(vanilla, sub, s + '.anims.' + ('json' if sub == 'json' else 'glb'))) for sub in ('json', 'glb', 'glb-nort')}
        check(before == after, 'aug end-to-end: the vanilla dir is untouched')
        van = json.load(open(os.path.join(vanilla, 'json', s + '.anims.json'), encoding='utf-8'))['Data']['RootChunk']['animations']
        aug = json.load(open(os.path.join(out, 'json', s + '.anims.json'), encoding='utf-8'))['Data']['RootChunk']['animations']
        an = [e['Data']['animation']['Data']['name']['$value'] for e in aug]
        check(an[:len(van)] == [e['Data']['animation']['Data']['name']['$value'] for e in van] and
              an[len(van):] == ['parried_left_combo_01', 'parried_right_combo_01', 'parried_left_safe', 'parried_right_safe'],
              'aug end-to-end: json = vanilla entries + 4 clones (per attack, left before right), guard_idle_loop not added')
        ids = {'HandleId': [], 'BufferId': []}
        st = [aug]
        while st:
            x = st.pop()
            if isinstance(x, dict):
                for k in ids:
                    if k in x:
                        ids[k].append(x[k])
                st.extend(x.values())
            elif isinstance(x, list):
                st.extend(x)
        check(all(len(v) == len(set(v)) for v in ids.values()), 'aug end-to-end: HandleId / BufferId unique in the augmented json')
        byn = {e['Data']['animation']['Data']['name']['$value']: e for e in aug}
        check(byn['parried_left_safe']['Data']['animation']['Data']['duration'] == byn['parried_left']['Data']['animation']['Data']['duration'],
              'aug end-to-end: a clone has the duration of its template')
        for sub in ('glb', 'glb-nort'):
            g, _ = G.read_glb(os.path.join(out, sub, s + '.anims.glb'))
            gn = [a['name'] for a in g['animations']]
            check(all(n in gn for n in names[:4]) and 'guard_idle_loop' not in gn, 'aug end-to-end: %s has the 4 clones' % sub)
        # a set without new names is copied unchanged
        rc = AV.main_args(['--vanilla', vanilla, '--input', inp, '--suffix', 'nomatch', '--sets', s, '--out', os.path.join(tmp, 'out2')])
        check(rc == 0 and sha(os.path.join(tmp, 'out2', 'json', s + '.anims.json')) == before['json'], 'aug end-to-end: no new names -> byte copy')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_helper_sets():
    ws = set(L.WEAPON_HELPER_SETS)
    want = {'pwa_two_handed_hammer', 'pma_two_handed_hammer', 'pwa_one_handed_blunt', 'pma_one_handed_blunt', 'pwa_baton', 'pma_baton',
            'default_melee_weapon', 'pma_default_melee_weapon', 'pwa_knife', 'pma_knife', 'pwa_katana', 'pma_katana'}
    check(ws == want, 'WEAPON_HELPER_SETS = hammer + one-handed blunt + baton + both default sets + knife + katana')
    out = {'pwa_fists', 'pma_fists', 'pwa_strongarms_melee_fists', 'pma_strongarms_melee_fists',
           'pwa_two_handed_blunt', 'pma_two_handed_blunt', 'pwa_mantisblade', 'pma_mantisblade'}
    check(not (ws & out), 'WEAPON_HELPER_SETS leaves fists / StrongArms / two-handed blunt / mantis blades out')
    check(L.VANILLA_DEPOT.get('pwa_mantisblade') == r'base\animations\cyberware\mantisblade\pwa_mantisblade.anims'
          and L.VANILLA_DEPOT.get('pma_mantisblade') == r'base\animations\cyberware\mantisblade\pma_mantisblade.anims'
          and L.RIG_OF_SET.get('pwa_mantisblade') == 'woman' and L.RIG_OF_SET.get('pma_mantisblade') == 'man'
          and 'pwa_mantisblade' not in L.ALL_SETS, 'SETS: mantis blade sets (extra sets, base depot paths, rigs)')
    with open(os.path.join(HERE, 'run_melee_export.ps1'), encoding='utf-8') as f:
        ps1 = f.read()
    check(all("'%s'" % s in ps1 for s in L.SETS), 'run_melee_export.ps1 knows every set of melee_export_lib.SETS')
    check(all(s in L.SETS and s in L.VANILLA_DEPOT and s in L.RIG_OF_SET for s in L.SETS) and set(L.SETS) == set(L.ALL_SETS) | set(L.EXTRA_SETS),
          'SETS: every set has a depot path and a rig (baton and StrongArms included)')
    check(L.RIG_OF_SET['pma_baton'] == 'man' and L.RIG_OF_SET['pwa_strongarms_melee_fists'] == 'woman', 'SETS: rigs of the extra sets')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--vanilla', default=os.path.join(L.ROOT, 'Build', 'work', 'melee-export-20261109', 'vanilla-frozen'))
    a = ap.parse_args()
    test_canonical()
    test_names()
    test_plan()
    test_aug(a.vanilla)
    test_helper_sets()
    print('MELEE_EXPORT_TEST %s (%d failed)' % ('FAIL' if FAILS else 'OK', len(FAILS)))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
