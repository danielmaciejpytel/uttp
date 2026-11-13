"""Pops next to the game clips that STAY in the one-handed blunt / baton sets (npc1 guard vs the game clips' first / last frame).  Offline,
read-only on clips (2026-11-09; the method of npc_blade_mixpop.py, which is a knife / katana script and stays unchanged).
python -I npc_blunt_mixpop.py [--rig man]   -> prints a summary, writes <OUTW>\\notes\\mixed_pops_<rig>.txt
Two distances (cm):
  all  = max over the 73 body joints (incl. legs and weapon joints) of the position difference, positions relative to joint 0
         (the measure of npc_blade_mixpop.py: directly comparable with the knife / katana numbers);
  mask = max over the joints the game takes from a melee clip (Spine1..Head, both arm chains incl. fingers, WeaponRight), positions in
         the frame of the Spine bone (motion matching owns Hips / Spine / legs): the part a transition can show in TPP.
Kept game clips: every non-additive clip of the vanilla set without an npc1 clip.  'masked' = the clip plays in a state of the DLL melee
mask (kMaskStateMelee: Parried 5 .. Deflect 10, the attacks 11 .. 21, groundslam / throw; the upper body then comes from the game);
the others (equip, crouch / sprint / slide / air locomotion) play outside the mask, where the upper body stays on motion matching.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blunt_film as BF  # noqa: E402  (npc_blunt_common: weapons, --rig, OUT per rig)
import npc_blunt_coverage as COV  # noqa: E402

F, BC, MC = BF.F, BF.BC, BF.MC
rig = BC.RIG
OUT = F.OUT
NOTES = os.path.join(BF.UC.OUTW, 'notes', 'mixed_pops_%s.txt' % rig)
MASKED_STATES = set(range(5, 22))
MASKED = {c for sid, _, cs in COV.CV.STATES if sid in MASKED_STATES for c in cs}


def is_masked(c):
    return c in MASKED or 'groundslam' in c or c.startswith('throw')


def mask_ix(names):
    up = ('Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'WeaponRight')
    return [i for i, n in enumerate(names) if n in up or (n[:4] in ('Left', 'Righ') and any(k in n for k in ('Shoulder', 'Arm', 'Hand')))]


def pos(variant, weapon, c):
    W, names = F.load(variant, weapon, c)[:2]
    W = np.asarray(W)
    p = W[:, :, :3, 3]
    sp = names.index('Spine')
    R, t = W[:, sp, :3, :3], W[:, sp, :3, 3]
    mi = mask_ix(names)
    pm = np.einsum('tji,tkj->tki', R, p[:, mi] - t[:, None])      # R^T (p - t)
    return p - p[:, :1, :], pm, names, [names[i] for i in mi]


def dj(a, b):
    n = np.linalg.norm(a - b, axis=-1)
    j = int(n.argmax())
    return float(n[j] * 100), j


full, summ = [], []
for weapon in BF.WEAPONS:
    st = BC.player_set(weapon)
    van = COV.vanilla_names(st)
    built = {f[len(st) + 2:-len('.npc1.npz')] for f in os.listdir(os.path.join(OUT, 'clips'))
             if f.startswith(st + '__') and f.endswith('.npc1.npz')}
    Gn, Gnm, names, mnames = pos('new', weapon, 'melee_idlepose')
    Gv, Gvm = pos('van', weapon, 'melee_idlepose')[:2]
    Gn, Gv, Gnm, Gvm = Gn[0], Gv[0], Gnm[0], Gvm[0]
    gd, gj = dj(Gn, Gv)
    gdm, gjm = dj(Gnm, Gvm)
    summ.append('%s (%s): npc1 clips %d; npc1 guard vs game guard: all %.1f cm (%s), mask %.1f cm (%s)'
                % (st, rig, len(built), gd, names[gj], gdm, mnames[gjm]))
    keep = sorted(c for c in van if c not in built and not c.startswith('add_') and os.path.exists(MC.clip_path(st, c)))
    rows = []
    for c in keep:
        P, Pm = pos('van', weapon, c)[:2]
        r = dict(clip=c, masked=is_masked(c))
        r['f0n'], j0 = dj(P[0], Gn)
        r['f0v'] = dj(P[0], Gv)[0]
        r['fNn'], jN = dj(P[-1], Gn)
        r['fNv'] = dj(P[-1], Gv)[0]
        r['m0n'], k0 = dj(Pm[0], Gnm)
        r['m0v'] = dj(Pm[0], Gvm)[0]
        r['mNn'], kN = dj(Pm[-1], Gnm)
        r['mNv'] = dj(Pm[-1], Gvm)[0]
        r['j'] = '%s/%s' % (mnames[k0], mnames[kN])
        rows.append(r)
    full.append('%s  game clips kept (%d): guard(npc1) -> clip f0 | clip fN -> guard(npc1), cm; in brackets the same against the game guard'
                % (st, len(rows)))
    for r in rows:
        full.append('  %-34s %-8s all f0 %5.1f (%5.1f) fN %5.1f (%5.1f) | mask f0 %5.1f (%5.1f) fN %5.1f (%5.1f) %s'
                    % (r['clip'], 'MASKED' if r['masked'] else 'outside', r['f0n'], r['f0v'], r['fNn'], r['fNv'], r['m0n'], r['m0v'],
                       r['mNn'], r['mNv'], r['j']))
    for tag, sel in (('all kept', rows), ('kept, in the melee mask', [r for r in rows if r['masked']])):
        if not sel:
            summ.append('  %s: none' % tag)
            continue
        a = np.array([[r['f0n'], r['f0v'], r['fNn'], r['fNv'], r['m0n'], r['m0v'], r['mNn'], r['mNv']] for r in sel])
        big = [r['clip'] for r in sel if r['m0n'] - r['m0v'] > 10.0 or r['mNn'] - r['mNv'] > 10.0]
        summ.append('  %s (%d): all-joint median f0 %.1f (game guard %.1f), max %.1f | mask median f0 %.1f (game guard %.1f), fN %.1f (%.1f), '
                    'max f0 %.1f, fN %.1f; mask pop added > 10 cm (%d): %s'
                    % (tag, len(sel), np.median(a[:, 0]), np.median(a[:, 1]), a[:, 0].max(), np.median(a[:, 4]), np.median(a[:, 5]),
                       np.median(a[:, 6]), np.median(a[:, 7]), a[:, 4].max(), a[:, 6].max(), len(big), ', '.join(big)))
    full.append('')
os.makedirs(os.path.dirname(NOTES), exist_ok=True)
open(NOTES, 'w', encoding='utf-8').write('\n'.join(summ + [''] + full) + '\n')
print('\n'.join(summ))
