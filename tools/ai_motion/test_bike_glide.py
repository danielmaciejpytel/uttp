"""Offline tests of the planted-foot measurement of bike_steps_gen (floors_of(robust=True), glide(robust=True), contact_phases).

  Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I Source\\tools\\ai_motion\\test_bike_glide.py

Synthetic clips (59 frames, RED axes): the planted foot of sportbike3 exit_l slid 61 cm while its height sat 4-6 cm above a 3-frame dip; the legacy lowest-ever floor then counted no frame as contact
(0.0 cm), the robust floor must see the slide, and a pinned foot / a foot that hops between two pins must read <= 3 cm per phase.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bike_steps_gen as BSG  # noqa: E402

NAMES = ['LeftFoot', 'LeftToeBase', 'RightFoot', 'RightToeBase']
N = 59


def clip(left_x, left_z, dip=True):
    """left foot: ankle x(t) / z(t); toe 10 cm ahead (y) and 9 cm lower; right foot parked high (never in contact)"""
    W = np.tile(np.eye(4), (N, len(NAMES), 1, 1))
    z = np.array(left_z, float).copy()
    if dip:
        z[14:17] -= 0.035   # the 3-frame dip of the real clip (3.5 cm below the stance level)
    W[:, 0, 0, 3], W[:, 0, 2, 3] = left_x, z
    W[:, 1, 0, 3], W[:, 1, 1, 3], W[:, 1, 2, 3] = left_x, 0.1, z - 0.09
    W[:, 2, 2, 3] = 0.5
    W[:, 3, 2, 3] = 0.41
    return W


def ground(z0=-0.75):
    z = np.full(N, z0)
    z[:5] = np.linspace(-0.45, z0, 5)   # the foot comes down from a peg
    return z


def test_slide_with_dip():
    x = np.zeros(N)
    x[10:50] = np.linspace(0.0, 0.6, 40)   # 60 cm slide, 1.5 cm / frame
    x[50:] = 0.6
    W = clip(x, ground())
    legacy = BSG.glide(W, NAMES, None, BSG.floors_of(W, NAMES))
    rob = BSG.glide(W, NAMES, None, BSG.floors_of(W, NAMES, robust=True), robust=True)
    assert legacy['glide_total_cm'] < 10.0, legacy   # documents the old blind spot
    assert 55.0 < rob["Left"]["glide_cm"] < 62.0, rob
    ph = BSG.contact_phases(W, NAMES, 'Left')
    assert max(p['net_cm'] for p in ph) > 50.0, ph


def test_pinned_foot_is_clean():
    W = clip(np.full(N, 0.3), ground())
    ph = BSG.contact_phases(W, NAMES, 'Left')
    assert BSG.glide(W, NAMES, None, robust=True)['glide_total_cm'] == 0.0
    assert ph and max(p['net_cm'] for p in ph) <= 0.1 and max(p['path_cm'] for p in ph) <= 0.1, ph


def test_hop_splits_two_plants():
    x = np.full(N, 0.0)
    z = ground()
    for f in range(30, 40):
        u = (f - 29) / 11.0
        x[f] = 0.9 * (u * u * (3 - 2 * u))
        z[f] += 0.10 * np.sin(np.pi * u)
    x[40:] = 0.9
    W = clip(x, z, dip=False)
    ph = BSG.contact_phases(W, NAMES, 'Left')
    assert len(ph) == 2 and max(p['net_cm'] for p in ph) <= 3.0, ph
    assert BSG.glide(W, NAMES, None, robust=True)['glide_total_cm'] <= 3.0


def test_legacy_defaults_unchanged():
    """the entry tools call floors_of / glide / slide_list without robust: same numbers as before (lowest-ever floor, pivot chain)"""
    x = np.zeros(N)
    x[10:50] = np.linspace(0.0, 0.6, 40)
    x[50:] = 0.6
    W = clip(x, ground(), dip=False)
    fl = BSG.floors_of(W, NAMES)
    assert abs(fl['Left'][0] - W[:, 0, 2, 3].min()) < 1e-12 and abs(fl['Left'][1] - W[:, 1, 2, 3].min()) < 1e-12
    g = BSG.glide(W, NAMES, None, fl)
    assert 50.0 < g['Left']['glide_cm'] < 70.0, g


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_') and callable(v):
            v()
            print('PASS', k)
