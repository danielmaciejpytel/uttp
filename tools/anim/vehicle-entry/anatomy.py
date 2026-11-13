"""Joint-range metrics of a player-rig clip for the anatomical limits of check_car_entry.py (n): how far a person can bend to get under a car roof.

Per frame, from the world pose (rig space, Y up):
  knee_flex   deg, max of both legs: 180 - interior angle at the knee (UpLeg - Leg - Foot); 0 = straight leg
  hip_flex    deg, max of both legs: angle between the thigh (UpLeg -> Leg) and the trunk-down axis (Neck -> Hips); 0 = standing, 90 = sitting
  trunk_lean  deg: angle of Hips -> Neck against world up (0 = upright, 90 = horizontal)
  neck_flex   deg: bend between the upper-spine direction (Spine3 -> Neck) and the neck/head direction (Neck -> Head)
  hips_rel    m: Hips height minus the Hips height of the SEATED end pose (>= 0: above the cushion)
Limits (LIMITS, documented in README 'Roof head cap / anatomical limits'): textbook joint ranges, then checked against what the game's own
entry clips (vanilla player clips and the NPC source clips of the 6 classes) reach - the game's animators never exceed them either; a clip
that ducks for a low roof by bending beyond them is not a person entering a car any more.
"""
import numpy as np

KNEE = ('LeftUpLeg', 'LeftLeg', 'LeftFoot'), ('RightUpLeg', 'RightLeg', 'RightFoot')


def _ang(a, b):
    a = a / np.maximum(np.linalg.norm(a, axis=-1, keepdims=True), 1e-9)
    b = b / np.maximum(np.linalg.norm(b, axis=-1, keepdims=True), 1e-9)
    return np.degrees(np.arccos(np.clip(np.sum(a * b, axis=-1), -1, 1)))


def metrics(sk, Wt):
    """Wt: (T, N, 3) world positions of the skeleton `sk`. Returns dict of (T,) arrays."""
    P = lambda n: Wt[:, sk.index[n]]  # noqa: E731
    out = {}
    knee, hip = [], []
    trunk_down = P('Hips') - P('Neck')
    for u, l, f in KNEE:
        knee.append(180.0 - _ang(P(u) - P(l), P(f) - P(l)))
        hip.append(_ang(P(l) - P(u), trunk_down))
    out['knee_flex'] = np.maximum(*knee)
    out['hip_flex'] = np.maximum(*hip)
    up = np.zeros_like(trunk_down)
    up[:, 1] = 1.0
    out['trunk_lean'] = _ang(P('Neck') - P('Hips'), up)
    out['neck_flex'] = _ang(P('Neck') - P('Spine3'), P('Head') - P('Neck'))
    return out


# How the limits were chosen (measured 2026-11-03 over the 24 vanilla player clips pwa / pma_<class>_<seat> 'enter', the 24 NPC clips wa / ma_<class>_<seat> 'enter' and the
# 24 shipped clips): the textbook ranges (AAOS / Kapandji: active knee flexion 135-150 deg, hip flexion 120-140 deg, lumbar +
# thoracic flexion ~80 deg, cervical flexion 45-60 deg) are the guide, but the limits are the NPC clips' maxima rounded up - the game's own animators (who key people getting
# into cars) never exceed them: knee 149.4 -> 150, thigh-vs-trunk (hip_flex, which INCLUDES lumbar flexion because the trunk axis is Neck -> Hips: that is why it exceeds the
# textbook hip range) 159.3 -> 160, trunk lean 65.6 -> 75 (textbook ~80), neck flexion 37.0 -> 55 (textbook 45-60). The vanilla PLAYER clips are NOT bounded by them
# (standard3 knee 161-170 while the leg is lifted onto the high step; the player clips also pass through the roof of low cars) - they are no reference for the duck.
LIMITS = dict(knee_flex=150.0, hip_flex=160.0, trunk_lean=75.0, neck_flex=55.0)
# the pelvis over the cushion never drops below the seated pelvis height minus this before the sit (a person cannot sit lower than the cushion; the
# game's own player clips dip up to 18.3 cm under their seated pelvis height right before the sit, the NPC clips 13.7 cm: measured limit + 2 cm)
HIPS_BELOW_SEATED_M = 0.20
