"""Unit tests of the anatomical gate + the hinge IK (no GPU, no game):  python -I test_anatomy_gate.py
1 vanilla vs itself passes; 2 a zero-scale correction reproduces the vanilla clip (so scaling down always ends at a passing clip);
3 hips drop with fixed feet on the straight FPP legs bends the knees FORWARD (flex > 0, no deviation) with the hinge IK;
4 regression: the corrections of the REJECTED demo v1 (v1 outputs of the old IK, kept under Build\\...\\demo and corrected) fail the gate.
"""
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import apply_keyposes as AK  # noqa: E402
import melee_common as MC  # noqa: E402


def test_vanilla_passes():
    for s, c in (('pwa_katana', 'combo_attack_01'), ('pwa_fists', 'combo_attack_01'), ('pwa_two_handed_blunt', 'heavy_attack_01')):
        d = MC.load_dump(MC.clip_path(s, c))
        r = AG.gate(d['W'], d['W'], np.arange(len(d['W']), dtype=float), AG.rig_of(d))
        assert r['passed'], AG.report(r)


def test_zero_scale_is_vanilla():
    d = MC.load_dump(MC.clip_path('pwa_katana', 'combo_attack_04_hit'))
    corr = json.load(open(os.path.join(MC.WORK, 'demo', 'corr', 'pwa_katana', 'combo_attack_04_hit.json'), encoding='utf-8'))
    for k in corr['keys']:        # no retiming: the source timeline
        k.pop('time_frame', None)
    cs = {c: 0.0 for c in AK.components(corr)}
    off, timed, meta, n_out = AK.build(d, AK.scaled_corr(corr, cs))
    assert np.abs(timed['W'] - d['W']).max() < 1e-3, np.abs(timed['W'] - d['W']).max()


def test_hips_drop_bends_knees_forward():
    d = MC.load_dump(MC.clip_path('pwa_katana', 'combo_attack_01'))
    corr = dict(schema='utpp-keypose-correction/1', set='pwa_katana', clip='combo_attack_01', window=2, keys=[
        dict(id='a', src_frame=3), dict(id='b', src_frame=10, hips_offset_cm=[0, 0, -2.5], keep=['LeftFoot', 'RightFoot']), dict(id='c', src_frame=20)])
    off, timed, meta, n_out = AK.build(d, corr)
    rg = AG.rig_of(d)
    f = AG.features(off['W'], rg)
    assert 5 < f['knee_flex_L'][10] < 40 and 5 < f['knee_flex_R'][10] < 40, (f['knee_flex_L'][10], f['knee_flex_R'][10])
    assert f['knee_dev_L'][10] < 3 and f['knee_dev_R'][10] < 3, (f['knee_dev_L'][10], f['knee_dev_R'][10])
    feet = np.abs(off['W'][10, [rg.ix['LeftFoot'], rg.ix['RightFoot']], :3, 3] - d['W'][10, [rg.ix['LeftFoot'], rg.ix['RightFoot']], :3, 3]).max()
    assert feet < 3e-3, feet


def test_rejected_v1_fails():
    """The 6 clips of the REJECTED demo v1 (old ik.two_bone, no gate; copies in Build\test-data\melee-v1-rejected\<set>\<clip>.{mb.dump.npz,keys.json})."""
    base = os.path.join(MC.ROOT, 'Build', 'test-data', 'melee-v1-rejected')
    cases = [('pwa_katana', 'combo_attack_04_hit'), ('pwa_fists', 'combo_attack_01'), ('pwa_fists', 'block_deflect_right'),
             ('pwa_two_handed_blunt', 'heavy_attack_01'), ('pwa_two_handed_blunt', 'melee_equip'), ('pwa_katana', 'combo_attack_01')]
    ran = 0
    for s, c in cases:
        pre = os.path.join(base, s, c)
        assert os.path.exists(pre + '.mb.dump.npz') and os.path.exists(pre + '.keys.json'), 'missing fixture ' + pre
        r = AG.gate_files(pre + '.mb.dump.npz', s, c, pre + '.keys.json')
        assert not r['passed'], '%s/%s (rejected v1) must fail the gate' % (s, c)
        ran += 1
    assert ran == len(cases)


if __name__ == '__main__':
    for n, f in list(globals().items()):
        if n.startswith('test_'):
            f()
            print('ok', n)
    print('anatomy_gate tests OK')
