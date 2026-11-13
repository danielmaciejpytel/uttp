"""coverage_blunt_baton.txt: which PSM.MeleeWeapon states have NPC-based TPP clips (npc1) for the one-handed blunt and baton sets, per rig
(2026-11-09).  npc_blade_coverage.py imported UNCHANGED (STATES, FULL_BODY_DEFAULT); Female clips in <OUTW>\\clips, Male in <OUTW>\\man\\clips.

  python -I npc_blunt_coverage.py [--out <txt>]

State -> clip mapping by clip name (INFERRED from the vanilla clip names and Source\\plugin\\UTPP\\src\\mm\\MeleeFullBody.hpp, not read from
the game's graph).
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_blunt_common as UC  # noqa: E402
import npc_blade_coverage as CV  # noqa: E402

BC = UC.BC


def vanilla_names(st):
    d = BC.vanilla_index(st)
    if isinstance(d, dict):
        d = d.get('clips', d)
    return set(d.keys()) if isinstance(d, dict) else {x if isinstance(x, str) else x.get('name') for x in d}


def main(a):
    out = os.path.abspath(a[a.index('--out') + 1]) if '--out' in a else os.path.join(UC.OUTW, 'coverage_blunt_baton.txt')
    lines = ['COVERAGE (PSM.MeleeWeapon state -> NPC-based TPP clips npc1), %s' % UC.OUTW,
             'state -> clip mapping INFERRED from the clip names; npc1 = built, every gate passed and shipped (clips\\); VAN = stays the game clip '
             '(rejected / chain-reverted / not made); "-" = the set has no such clip.',
             'DLL (MeleeFullBody.hpp): full body by default only for the attack states 11-18, 20, 21 (19 ThrowAttack excluded); Block / Hold etc. '
             'only when added by script.  TODAY the DLL / utpp_melee.reds route only w_melee_fists to the TPP clips: blunt / baton need the '
             'changes listed in report.txt.', '']
    for weapon, sets in (('blunt', ('pwa_one_handed_blunt', 'pma_one_handed_blunt')), ('baton', ('pwa_baton', 'pma_baton'))):
        for st in sets:
            rig = 'man' if st.startswith('pma_') else 'woman'
            od = os.path.join(UC.OUTW, 'man') if rig == 'man' else UC.OUTW
            cd = os.path.join(od, 'clips')
            van = vanilla_names(st)
            built = {f[len(st) + 2:-len('.npc1.npz')] for f in (os.listdir(cd) if os.path.isdir(cd) else [])
                     if f.startswith(st + '__') and f.endswith('.npc1.npz') and not f.endswith('.twohand.npz')}
            if not os.path.exists(os.path.join(od, 'build_report_%s.json' % st)):
                lines += ['%s: not built (no build report)' % st, '']
                continue
            lines.append('%s  (%s V, %s)  npc1 clips: %d   vanilla clips in the set: %d' % (
                st, 'Female' if rig == 'woman' else 'Male', weapon, len(built - {'guard_idle_loop'}), len(van)))
            for sid, sname, clips in CV.STATES:
                if not clips:
                    lines.append('  %2d %-14s -' % (sid, sname))
                    continue
                n = [c for c in clips if c in built]
                v = [c for c in clips if c in van and c not in built]
                m = [c for c in clips if c not in van and c not in built]
                tag = 'FULL' if v == [] and n else ('PART' if n else ('VAN' if v else '-'))
                lines.append('  %2d %-14s %-4s %s npc1: %s%s%s' % (sid, sname, tag, '(DLL full body)' if sid in CV.FULL_BODY_DEFAULT else '               ',
                                                                 ', '.join(n) or '-', ('  | VAN: ' + ', '.join(v)) if v else '',
                                                                 ('  | absent: ' + ', '.join(m)) if m else ''))
            other = sorted(van - built - {c for _, _, cs in CV.STATES for c in cs})
            lines.append('  other clips of the set, game clips (VAN): ' + ', '.join(other))
            lines.append('')
    open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('COVERAGE', out)


if __name__ == '__main__':
    main(sys.argv[1:])
