"""Neutral (zero-delta) versions of the first-person fists additives the graph plays on top of the guard in idle / walk / jog (2026-11-09).

python -I npc_additive_neutral.py [--set pwa_fists|pma_fists] [--out <dir>]      (default out: Build\\work\\melee-npc-20261109\\additives)

The fists graph adds add_melee_idle / add_melee_idle_combat (idle, Switch on isHolding), add_walk_melee_0 (speed > 0.1), add_jog_melee_0
(speed > 3) and add_inertia_*_melee (turn / strafe inertia) to melee_idlepose (guard_research.txt).  In third person they would move the TPP
guard by first-person deltas.  A neutral clip holds the REST pose (rest_lt / rest_lr of the dump) on EVERY frame, for all 152 joints: its
frame 0 is then the rest pose too, so the delta is zero whether the engine's reference is the rest pose or frame 0 ('Additive' /
'AdditiveWithoutFirstFrame'; for the vanilla 'Additive' clips frame 0 == rest, checked here).  World matrices W / W_extra recomputed by FK.
Frame count, duration, type and events stay vanilla (the exporter keeps them).
Output per clip: <set>__<clip>.neutral.npz (the dump layout of melee_dump.py: W [T,73,4,4], lt / lr [T,152,*], names152, rest_lt / rest_lr,
meta) + additives_<set>.json (type, frames, frame-0 vs rest, vanilla motion that is removed).  NOT exportable yet: build_melee_set.py skips
additive clips (see the report).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

CLIPS = ['add_melee_idle', 'add_melee_idle_combat', 'add_walk_melee_0', 'add_jog_melee_0', 'add_inertia_rotation_down_melee',
         'add_inertia_rotation_left_melee', 'add_inertia_rotation_right_melee', 'add_inertia_rotation_up_melee', 'add_inertia_strafe_270_melee',
         'add_inertia_strafe_90_melee']


def rot_deg(qa, qb):
    d = np.abs(np.sum(qa * qb, -1))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1)))


def main(a):
    st = a[a.index('--set') + 1] if '--set' in a else 'pwa_fists'
    out = a[a.index('--out') + 1] if '--out' in a else os.path.join(MC.ROOT, 'Build', 'work', 'melee-npc-20261109', 'additives')
    os.makedirs(out, exist_ok=True)
    rep = {}
    for c in CLIPS:
        p = MC.clip_path(st, c)
        if not os.path.exists(p):
            rep[c] = dict(error='no vanilla dump %s' % p)
            continue
        d = MC.load_dump(p)
        T = len(d['lt'])
        typ = d['meta'].get('animation_type')
        lt0, lr0 = d['lt'][:1], d['lr'][:1]
        f0_rest_cm = float(np.linalg.norm(lt0[0] - d['rest_lt'], axis=-1).max() * 100)
        f0_rest_deg = float(rot_deg(lr0[0], d['rest_lr']).max())
        body = list(range(73))
        mv_cm = float(np.linalg.norm(d['W'][:, body, :3, 3] - d['W'][:1, body, :3, 3], axis=-1).max() * 100)
        mv_deg = float(rot_deg(d['lr'], lr0).max())
        par = np.asarray(d['parents152'])
        names = [str(x) for x in d['names152']]
        assert names[:73] == [str(x) for x in d['names']] and names[73:] == [str(x) for x in d['names_extra']], 'joint order'
        Wv = MC.fk_world(par, d['lt'][:1].astype(float), d['lr'][:1].astype(float))[0]
        fk_check_mm = float(np.abs(Wv[:73, :3, 3] - d['W'][0, :, :3, 3]).max() * 1000)       # FK reproduces the dump's world matrices
        n = dict(d)
        n['lt'] = np.repeat(d['rest_lt'][None], T, 0).astype(d['lt'].dtype)
        n['lr'] = np.repeat(d['rest_lr'][None], T, 0).astype(d['lr'].dtype)
        Wr = MC.fk_world(par, n['lt'][:1].astype(float), n['lr'][:1].astype(float))[0]
        n['W'] = np.repeat(Wr[None, :73], T, 0).astype(d['W'].dtype)
        n['W_extra'] = np.repeat(Wr[None, 73:], T, 0).astype(d['W_extra'].dtype)
        meta = dict(d['meta'], source='npc_additive_neutral.py: every frame = the rest pose, all 152 joints (zero additive delta)')
        n['meta'] = json.dumps(meta)
        dst = os.path.join(out, '%s__%s.neutral.npz' % (st, c))
        np.savez_compressed(dst, **{k: (np.asarray(v) if k != 'meta' else v) for k, v in n.items()})
        rep[c] = dict(type=typ, frames=T, vanilla_frame0_vs_rest_cm=round(f0_rest_cm, 4), vanilla_frame0_vs_rest_deg=round(f0_rest_deg, 4), fk_check_mm=round(fk_check_mm, 4),
                      vanilla_body_motion_removed_cm=round(mv_cm, 2), vanilla_local_rot_removed_deg=round(mv_deg, 2), out=dst,
                      neutral='rest pose on every frame (152 joints)')
        print('%-34s %-26s T=%4d vanilla f0-rest %.3f cm %.3f deg | removed %.1f cm %.1f deg | fk check %.4f mm' % (c, typ, T, f0_rest_cm, f0_rest_deg, mv_cm, mv_deg, fk_check_mm))
    MC.jdump(rep, os.path.join(out, 'additives_%s.json' % st))


if __name__ == '__main__':
    main(sys.argv[1:])
