"""Pops at transitions in MIXED sets (npc1 idle / clips next to clips that stay the game's own). Offline, read-only on clips.
python -I npc_blade_mixpop.py [--rig man]   -> prints a summary, writes notes/mixed_pops_<rig>.txt
Distance = max over the 73 body joints (incl. weapon joints) of the position difference, positions relative to joint 0, in cm.
"""
import os, sys, json
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from utpp_paths import project_root  # noqa: E402
ROOT = project_root()
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'melee'))
man = '--rig' in sys.argv and sys.argv[sys.argv.index('--rig') + 1] == 'man'
sys.argv = ['x', 'check'] + (['--rig', 'man'] if man else [])
import npc_blade_film as BF  # noqa: E402
import npc_blade_common as BC  # noqa: E402
import npc_blade_coverage as COV  # noqa: E402

rig = 'man' if man else 'woman'
OUT = BC.OUTW
NOTES = os.path.join(OUT, 'notes', 'mixed_pops_%s.txt' % rig)


def pos(variant, weapon, c):
    W, names = BF.load(variant, weapon, c)[:2]
    p = np.asarray(W)[:, :, :3, 3]
    return p - p[:, :1, :], names


def dj(a, b):
    n = np.linalg.norm(a - b, axis=-1)
    j = int(n.argmax())
    return float(n[j] * 100), j


full, summ = [], []
for weapon in ('knife', 'katana'):
    st = BC.player_set(weapon)
    van = COV.vanilla_names(st)
    built = {f[len(st) + 2:-len('.npc1.npz')] for f in os.listdir(os.path.join(OUT, 'clips'))
             if f.startswith(st + '__') and f.endswith('.npc1.npz')}
    rep = json.load(open(os.path.join(OUT, 'build_report_%s.json' % st)))
    npass = [k for k, v in rep.items() if v.get('PASS')]
    exo = [k for k in npass if rep[k].get('base_gate_pass_raw') is not None and rep[k]['base_gate_pass_raw'] < rep[k]['frames']]
    game_built = [k for k in npass if k in van]
    summ.append('%s: PASS entries %d / %d (strict, without the 2H hand_cross exemption: %d); game clips replaced %d (strict %d)'
                % (st, len(npass), len(rep), len(npass) - len(exo), len(game_built), len([k for k in game_built if k not in exo])))
    Gn, names = pos('new', weapon, 'melee_idlepose')
    Gv = pos('van', weapon, 'melee_idlepose')[0]
    gsrc = BF.load('new', weapon, 'melee_idlepose')[3]
    Gn, Gv = Gn[0], Gv[0]
    gd, gj = dj(Gn, Gv)
    summ.append('  idle used: melee_idlepose %s; npc1 idle vs game idle: %.1f cm (%s)' % (gsrc, gd, names[gj]))
    seen = []
    for _, _, cs in COV.STATES:
        for c in cs:
            if c not in seen:
                seen.append(c)
    keep = [c for c in seen if c in van and c not in built and os.path.exists(BC.MC.clip_path(st, c))]
    rows = []
    for c in keep:
        P = pos('van', weapon, c)[0]
        f0n, j0 = dj(P[0], Gn)
        f0v = dj(P[0], Gv)[0]
        fNn, jN = dj(P[-1], Gn)
        fNv = dj(P[-1], Gv)[0]
        rows.append((c, f0n, f0v, names[j0], fNn, fNv, names[jN]))
    full.append('%s  game clips kept (%d): idle(npc1) -> clip f0 | clip fN -> idle(npc1); in brackets the same against the game idle' % (st, len(rows)))
    for r in rows:
        full.append('  %-26s f0 %5.1f (%5.1f) %-16s fN %5.1f (%5.1f) %s' % r)
    if rows:
        a = np.array([[r[1], r[2], r[4], r[5]] for r in rows])
        big = [r[0] for r in rows if r[1] - r[2] > 10.0 or r[4] - r[5] > 10.0]
        summ.append('  game clips kept next to the npc1 idle: %d; median f0 %.1f cm (vs game idle %.1f), median fN %.1f (%.1f); max f0 %.1f, fN %.1f'
                    % (len(rows), np.median(a[:, 0]), np.median(a[:, 1]), np.median(a[:, 2]), np.median(a[:, 3]), a[:, 0].max(), a[:, 2].max()))
        summ.append('  added pop > 10 cm vs the all-game baseline (%d): %s' % (len(big), ', '.join(big)))
    pairs = [('combo_attack_%02d' % k, 'combo_attack_%02d' % (k + 1), 'end') for k in (1, 2, 3)]
    pairs += [('block_start', 'block', 'end'), ('block', 'block_end', 'end'), ('block', 'block_attack', 'end'),
              ('block', 'block_deflect_left', 'end'), ('block', 'block_deflect_right', 'end'),
              ('hold_start_01', 'hold_loop_01', 'end'), ('hold_start_offscreen_01', 'hold_loop_01', 'end'),
              ('hold_start_offscreen_02', 'hold_loop_02', 'end'), ('hold_loop_01', 'heavy_attack_01', 'end'), ('hold_loop_02', 'heavy_attack_02', 'end')]
    pairs += [(b, b + s, 'min') for b in COV.BASES for s in ('_hit', '_deflect')]
    full.append('%s  chains (a -> b; end: a fN vs b f0; min: closest frame of a vs b f0): mixed vs all-game' % st)
    mixl = []
    for a_, b_, mode in pairs:
        if not all((x in van or x in built) and (x in built or os.path.exists(BC.MC.clip_path(st, x))) for x in (a_, b_)):
            continue
        if (a_ in built) == (b_ in built):
            continue
        Am, Bm = pos('new', weapon, a_)[0], pos('new', weapon, b_)[0]
        Av, Bv = pos('van', weapon, a_)[0], pos('van', weapon, b_)[0]
        if mode == 'end':
            m, jm = dj(Am[-1], Bm[0])
            v = dj(Av[-1], Bv[0])[0]
        else:
            m, jm = min(dj(x, Bm[0]) for x in Am)
            v = min(dj(x, Bv[0])[0] for x in Av)
        tag = '%s(%s) -> %s(%s)' % (a_, 'npc1' if a_ in built else 'game', b_, 'npc1' if b_ in built else 'game')
        full.append('  %-62s %-3s %5.1f cm (all-game %5.1f) %s' % (tag, mode, m, v, names[jm]))
        mixl.append((m - v, tag, m, v))
    mixl.sort(reverse=True)
    summ.append('  mixed chain pairs: %d; added > 10 cm: %d; worst: %s' % (
        len(mixl), sum(1 for x in mixl if x[0] > 10.0), '; '.join('%s %.0f (game %.0f)' % (t, m, v) for _, t, m, v in mixl[:4])))
open(NOTES, 'w', encoding='utf-8').write('\n'.join(summ + [''] + full) + '\n')
print('\n'.join(summ))
print('FULL', NOTES)
