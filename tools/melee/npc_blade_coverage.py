"""coverage_blades.txt: which PSM.MeleeWeapon states have NPC-based TPP clips (npc1) per weapon and rig (2026-11-09).

  python -I npc_blade_coverage.py [--out <txt>]

State -> clip mapping by clip name (INFERRED from the vanilla clip names and Source\\plugin\\UTPP\\src\\mm\\MeleeFullBody.hpp, not read from
the game's graph).  Per state and set: clips built (npc1, passed every gate, in clips\\), clips that stay the game's own (vanilla: rejected,
skipped or not made) and names the set does not have.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blade_common as BC  # noqa: E402

OUT = BC.OUTW
BASES = ['combo_attack_01', 'combo_attack_02', 'combo_attack_03', 'combo_attack_04', 'final_attack_03', 'final_attack_04', 'final_attack_05',
         'heavy_attack_01', 'heavy_attack_02', 'safe_attack', 'equip_attack', 'sprint_attack', 'jump_attack', 'crouch_attack']
hit = lambda bs: [b + '_hit' for b in bs]
STATES = [
    (0, 'NotReady', []),
    (1, 'Equipping', ['melee_equip', 'melee_first_equip']),
    (2, 'Idle', ['melee_idlepose']),
    (3, 'Safe', ['melee_safepose']),
    (4, 'PublicSafe', ['melee_safepose']),
    (5, 'Parried', ['parried_left', 'parried_right']),
    (6, 'Hold', ['hold_start_01', 'hold_start_offscreen_01', 'hold_start_offscreen_02', 'hold_loop_01', 'hold_loop_02']),
    (7, 'ChargedHold', ['hold_loop_01', 'hold_loop_02']),
    (8, 'Block', ['block_start', 'block', 'block_end']),
    (9, 'Targeting', ['melee_idlepose']),
    (10, 'Deflect', ['block_deflect_left', 'block_deflect_right']),
    (11, 'ComboAttack', BASES[:4] + hit(BASES[:4])),
    (12, 'FinalAttack', BASES[4:7] + hit(BASES[4:7])),
    (13, 'StrongAttack', BASES[7:9] + hit(BASES[7:9])),
    (14, 'SafeAttack', ['safe_attack', 'safe_attack_hit']),
    (15, 'BlockAttack', ['block_attack', 'block_attack_hit']),
    (16, 'SprintAttack', ['sprint_attack', 'sprint_attack_hit']),
    (17, 'CrouchAttack', ['crouch_attack', 'crouch_attack_hit']),
    (18, 'JumpAttack', ['jump_attack', 'jump_attack_hit']),
    (19, 'ThrowAttack', ['throw_start', 'throw_attack', 'throw_end_from_attack']),
    (20, 'DeflectAttack', [b + '_deflect' for b in BASES]),
    (21, 'EquipAttack', ['equip_attack', 'equip_attack_hit']),
]
FULL_BODY_DEFAULT = set(range(11, 22)) - {19}      # MeleeFullBody.hpp kFullBodyDefaultStates (0x37F800)


def vanilla_names(st):
    d = json.load(open(os.path.join(BC.MC.WORK, 'npz', st, '_index.json')))
    if isinstance(d, dict):
        d = d.get('clips', d)
    return set(d.keys()) if isinstance(d, dict) else {x if isinstance(x, str) else x.get('name') for x in d}


def main(a):
    out = os.path.abspath(a[a.index('--out') + 1]) if '--out' in a else os.path.join(OUT, 'coverage_blades.txt')
    lines = ['COVERAGE (PSM.MeleeWeapon state -> NPC-based TPP clips npc1), %s' % OUT,
             'state -> clip mapping INFERRED from the clip names; npc1 = built, every gate passed (clips\\); VAN = stays the game clip '
             '(rejected / skipped / not made); "-" = the set has no such clip.',
             'DLL (MeleeFullBody.hpp): full body by default only for the attack states 11-18, 20, 21 (19 ThrowAttack excluded); Block / Hold etc. '
             'only when added by script; the DLL guard pose is fists-only (blades: idle = melee_idlepose of the set, npc1 if the game plays it; f0 of an attack may pop against other game poses).', '']
    for weapon in ('knife', 'katana'):
        for rig, pre in (('woman', 'pwa'), ('man', 'pma')):
            st = '%s_%s' % (pre, weapon)
            van = vanilla_names(st)
            built = {f[len(st) + 2:-len('.npc1.npz')] for f in os.listdir(os.path.join(OUT, 'clips'))
                     if f.startswith(st + '__') and f.endswith('.npc1.npz')}
            rp = os.path.join(OUT, 'build_report_%s.json' % st)
            if not os.path.exists(rp):
                lines += ['%s: not built (no build report)' % st, '']
                continue
            lines.append('%s  (%s V)  npc1 clips: %d   vanilla clips in the set: %d' % (st, 'Female' if rig == 'woman' else 'Male', len(built - {'guard_idle_loop'}), len(van)))
            for sid, sname, clips in STATES:
                if not clips:
                    lines.append('  %2d %-14s -' % (sid, sname))
                    continue
                n = [c for c in clips if c in built]
                v = [c for c in clips if c in van and c not in built]
                m = [c for c in clips if c not in van and c not in built]
                tag = 'FULL' if v == [] and n else ('PART' if n else ('VAN' if v else '-'))
                lines.append('  %2d %-14s %-4s %s npc1: %s%s%s' % (sid, sname, tag, '(DLL full body)' if sid in FULL_BODY_DEFAULT else '               ',
                                                                 ', '.join(n) or '-', ('  | VAN: ' + ', '.join(v)) if v else '',
                                                                 ('  | absent: ' + ', '.join(m)) if m else ''))
            other = sorted(van - built - {c for _, _, cs in STATES for c in cs})
            lines.append('  other clips of the set, game clips (VAN; MotionBricks / later candidates): ' + ', '.join(other))
            lines.append('')
    open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('COVERAGE', out)


if __name__ == '__main__':
    main(sys.argv[1:])
