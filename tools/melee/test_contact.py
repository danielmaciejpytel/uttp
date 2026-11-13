"""Unit tests of contact.py (python -I test_contact.py): the local-stance-ground rule vs a global-minimum rule."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # -I ignores the current directory
import numpy as np  # noqa: E402

import contact as K  # noqa: E402


def step_path():
    """One foot: stance on level 0 (frames 0..29), swing up onto a 0.30 m ledge (30..39), stance on the ledge (40..99)."""
    z = np.zeros(100)
    z[30:40] = 0.30 * np.sin(np.linspace(0, np.pi / 2, 10)) ** 2 + 0.06 * np.sin(np.linspace(0, np.pi, 10))
    z[40:] = 0.30
    return z


def test_global_minimum_fails_on_a_step_local_rule_passes():
    z = step_path()
    global_rule = (z - z.min()) < 0.04
    assert not global_rule[40:].any(), 'the global-minimum rule must miss the ledge stance (that is why it is not used)'
    c = K.detect_contacts(z, window=10)
    assert c[:30].all(), 'first stance'
    assert not c[33:38].any(), 'swing'
    assert c[40 + 11:].all(), 'ledge stance is a contact once the old ground left the +-10 frame window'
    assert c[40 + 11:].sum() == 49


def test_two_feet_alternate_with_lowered_hips_offset():
    # walk on the flat, feet alternate; the ground the clip stands on is z = 0.5 (a platform): absolute heights must not matter
    t = np.arange(120)
    ph = (t / 30.0) * 2 * np.pi
    left = 0.5 + np.maximum(0, np.sin(ph)) * 0.12
    right = 0.5 + np.maximum(0, -np.sin(ph)) * 0.12
    c = K.detect_contacts(np.stack([left, right], 1))
    assert c[:, 0][int(30 * 0.75)] and not c[:, 0][int(30 * 0.25)]
    assert c[:, 1][int(30 * 0.25)] and not c[:, 1][int(30 * 0.75)]
    assert 0.35 < c.mean() < 0.75


def test_jump_has_no_contact():
    t = np.linspace(0, 1, 20)
    z = 0.6 * 4 * t * (1 - t) + 0.20  # both feet airborne for the whole clip, apex 0.8 m
    c = K.detect_contacts(np.stack([z, z], 1))
    assert not c.any(), 'apex frames (vz ~ 0) must not become a contact'


def test_sole_height_uses_the_lower_point():
    # flat foot: heel 0.083 / toe 0.018 above the sole -> 0; heel lifted 5 cm (toe stance) -> still 0; toe lifted -> heel carries
    assert abs(K.sole_height(0.083, 0.018, 0.083, 0.018)) < 1e-9
    assert abs(K.sole_height(0.133, 0.018, 0.083, 0.018)) < 1e-9
    assert abs(K.sole_height(0.083, 0.068, 0.083, 0.018)) < 1e-9
    assert abs(K.sole_height(0.20, 0.15, 0.083, 0.018) - 0.117) < 1e-9


def test_slide_segments():
    xy = np.zeros((30, 2))
    xy[10:20, 0] = np.linspace(0, 0.10, 10)  # 10 cm drift inside the planted run
    xy[20:, 0] = 0.10
    contact = np.ones(30, dtype=bool)
    seg = K.slide_segments(contact, xy)
    assert len(seg) == 1 and abs(seg[0]['drift_cm'] - 10.0) < 1e-6 and seg[0]['max_speed_cm_s'] > 30
    assert K.slide_segments(contact, np.zeros((30, 2)))[0]['drift_cm'] == 0.0


if __name__ == '__main__':
    n = 0
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            v()
            n += 1
    print('test_contact OK (%d tests)' % n)
