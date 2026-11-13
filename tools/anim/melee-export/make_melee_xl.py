"""Write Source\\plugin\\UTPP\\archive\\utpp_melee_tpp.archive.xl from the registration plan (registration_evidence.py) and a run manifest.

  python -I make_melee_xl.py --registration <evidence\\registration.json> --manifest <run\\manifest.json> --out <xl path> [--copy-to <dir>]

Only sets whose .anims is in the run's archive (manifest 'staged') are registered.  One entry per (rig, set, path): the base and the ep1 entity of
the rig, component and vars of the vanilla entry + utpp_foot_tpp (+ the weapon variable of the family where the vanilla entry has none, see the
plan), priority = vanilla - 1.

The comment block of the written .xl ships in the mod package: it describes the entries that are actually written (HEADER_TOP + one generated
line per entry / per path that is not mirrored + HEADER_BOTTOM) and must not contain local paths or user names (checked before writing).
"""
import argparse
import json
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402

HEADER_TOP = """# UTPP third-person melee (full body). New animation data: utpp\\melee\\utpp_<set>.anims (archive utpp_melee.archive). Each UTPP set holds only
# the NON-ADDITIVE clips that were re-solved for the third-person body, under the vanilla clip names (same rig, duration, events, float tracks and
# root motion as the vanilla clip). Additive clips (add_*) and the clips listed below stay in the vanilla sets.
# Registration (static analysis of the 2.5 player entities):
#   the 2.5 player entities (player_wa_fpp.ent / player_ma_fpp.ent and the ep1 *_ep1.ent, which carry the same entries) register the vanilla
#   sets on the root animated component: default_melee_weapon / pma_default_melee_weapon at 200 without variables, and per weapon
#   p?a_<weapon>.anims at 128 with the weapon variable (w_melee_fists, w_melee_katana, w_melee_knife, w_melee_one_hand_blunt, w_two_hand_blunt);
#   the ep1 entities add the Gorilla Arms set p?a_strongarms_melee_fists.anims at 128 [StrongArms] on the extension component "EP1 animsets".
#   Each entry written below mirrors one of those vanilla entries: same entities and component, the variables of the vanilla entry + the UTPP
#   wrapper variable utpp_foot_tpp{EXTRA_A}, priority one lower (199 / 127). "Lower number wins" (static evidence in the 2.5 entities, see also
#   utpp_foot_tpp.archive.xl): specific entries carry lower numbers than generic ones (dual-wield pistol sets 64 [dual_wield, <weapon>] < dual-wield
#   base 100 < per-weapon sets 128 < base-weapon sets 150 < default sets 200); the hacking crouch seen in game came from a 199 set winning over
#   the 200 locomotion set.
#   Several variables in one entry must ALL be set (static evidence: the dual-wield entries [dual_wield, w_handgun_X] at 64 sit next to the
#   single-weapon entries [w_handgun_X] at 128; if one variable were enough, every single-handed pistol would play the dual-wield clips; the
#   UTPP vehicle entries [utpp_vehicle_tpp, <seat variable>] pick the clip of the seat's class / bike only - these live in the vehicle workspot
#   override, not on this component). So an entry below is active only when the vanilla entry is active AND utpp_foot_tpp is set{EXTRA_B}, and it then wins exactly for the clip names it holds; every other clip name still resolves as in
#   vanilla.
#   No other entry of the entities sits between the vanilla priority and the new one for these clip names, except the cases handled below.
# FPP / vanilla: the UTPP body script sets utpp_foot_tpp to 1 only while the UTPP third-person body is wanted (and 0 during a vanilla
#   take-over); with 0 this file has no effect and the vanilla sets apply unchanged.
# Entries written in this file (set | component | variables | priority | entities):
{ENTRIES}
"""

EXTRA_A = (' (the default set, which has no variable in vanilla, gets the variable of the weapon family it serves\n'
           '#   so that it does not reach other weapons)')
EXTRA_B = ' (and, for the\n#   default set, the weapon variable listed)'

HEADER_BOTTOM = """# Not mirrored (the clips of these paths stay vanilla): paths gated by a modifier variable of the vanilla entities - the fists set gated by
#   StrongArms on the root component (the ep1 entities add the Gorilla Arms set at the same priority on an extension component; a mirrored entry
#   would outrank or tie with it), the one-handed blunt set gated by additives (an extra
#   source for other weapons; a mirrored entry would tie with the other weapons' new entries), the knife set gated by knife_thumb_override
#   (meaning not established). The Gorilla Arms set itself is mirrored only where an entry above names it.
{UNMIRRORED}
# Clip names that stay vanilla (another entry that can be active at the same time has the same priority as the new entry or as the vanilla
#   one, so the order would be unknown): melee_first_equip in the weapon sets (p?a_one_handed_blunt_dildo.anims at 127 [dildo]) and,
#   male default set only, vault (pma_default_weapon.anims at 200, no variable). Held pose clips of fewer than 6 frames (melee_idlepose,
#   melee_safepose, melee_crouchpose, block, melee_air_sprint, ...) stay vanilla unless the built set supplies a re-solved version (the
#   fists sets do for melee_idlepose = the guard every fist clip starts and ends in, melee_safepose and melee_crouchpose; block and
#   melee_air_sprint stay vanilla).
# Root motion: unchanged (the vanilla motionExtraction, copied verbatim); the new clips are built in the anim buffer frame (Root = the vanilla
#   buffer Root, identity), so the engine moves the entity, and the new body with it, exactly as in vanilla.
# Known risks (not verified in game): (1) entries gated by a MODIFIER variable at the vanilla priority 128 that hold the same clip names
#   (knife_thumb_override, additives, StrongArms). If such a variable is set together with a weapon variable and its entry won over the weapon's
#   own set in vanilla, the new weapon entry now wins instead (e.g. fists clips instead of Gorilla Arms clips of the same name). Where a Gorilla
#   Arms entry (StrongArms, component "EP1 animsets") is listed above next to a fists entry (component root), both sit at priority 127 on
#   different components; which of them wins for a shared clip name is not established.
#   (2) Fallback data: the per-clip fallback frame indices of every replaced clip are cleared (as for the vehicle-entry clips); the set-level
#   fallback data (fallbackAnimDataBuffer / fallbackAnimFrameDescs / fallbackAnimDescIndexes) is still the vanilla one (first-person poses
#   of the full vanilla set) and is no longer referenced by the replaced clips. Not verified in game for these sets.
# Clips rejected by a gate of the export (round trip, SIMD frame count; listed in the run manifest, set_details) also stay vanilla.
# NOT verified in game: the winning direction for these entries, that a script-set wrapper variable gates an ArchiveXL-merged entry of this
#   component, that an entry on the "EP1 animsets" component is merged and gated the same way, and the visual result. Verify: draw each weapon
#   type in third person (utpp_foot_tpp = 1) and in first person, compare with the vanilla clip; strong-arms cyberware with fists (including the
#   charged one_punch heavy attack); the first equip of a weapon (stays vanilla); body vs capsule alignment on the fists clips with root motion
#   (melee_slide_start / melee_slide_end, jump_attack, safe_attack).
animations:
"""

# the comment block ships in the mod package: no local paths, drive letters or user names
LOCAL_PATH_RX = re.compile(r'(?i)(source\\|build\\|research\\|backups\\|[a-z]:\\|[a-z]:/|/users/|\\users\\|' + re.escape(os.environ.get('USERNAME') or os.environ.get('USER') or '@') + ')')


def header(groups, plan, staged_sets):
    ent_lines = []
    for s in L.SETS:
        for (s2, comp, vars_, pr), ents in sorted(groups.items(), key=lambda kv: (kv[0][3], kv[0][2])):
            if s2 != s:
                continue
            names = ', '.join(os.path.basename(e.replace('\\', '/')) for e in sorted(set(ents), key=lambda x: (x.startswith('ep1'), x)))
            ent_lines.append('#   %s\\%s | %s | [%s] | %d | %s' % (L.UTPP_DEPOT_DIR, L.utpp_name(s), comp, ', '.join(vars_), pr, names))
    unm = []
    seen = set()
    for s in L.SETS:
        if s not in staged_sets:
            continue
        for r in plan.get(s, []):
            if r['mirrored']:
                continue
            k = (s, r['component'], tuple(r['vanilla_vars']), r.get('why_not'))
            if k in seen:
                continue
            seen.add(k)
            unm.append('#   vanilla set of %s, component %s, variables [%s]: %s' % (s, r['component'], ', '.join(r['vanilla_vars']), r.get('why_not')))
    # an entry may carry variables beyond the vanilla ones + the wrapper (the default set gets its weapon family variable): describe that only then
    extra = any(set(r.get('utpp_vars') or []) - set(r['vanilla_vars']) - {'utpp_foot_tpp'}
                for s in staged_sets for r in plan.get(s, []) if r['mirrored'])
    top = HEADER_TOP.replace('{EXTRA_A}', EXTRA_A if extra else '').replace('{EXTRA_B}', EXTRA_B if extra else '')
    return (top.replace('{ENTRIES}', '\n'.join(ent_lines)) +
            HEADER_BOTTOM.replace('{UNMIRRORED}', ('# Not mirrored in this file (set, path of the vanilla entity):\n' + '\n'.join(unm)) if unm else
                                  '# (no path of the sets of this file is left unmirrored)'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--registration', required=True)
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--copy-to')
    a = ap.parse_args()
    reg = json.load(open(a.registration, encoding='utf-8'))
    man = json.load(open(a.manifest, encoding='utf-8-sig'))
    staged = {os.path.basename(x['path']) for x in (man.get('staged') or [])}
    # 2026-11-09 review (finding 8): only sets whose pack succeeded. The run's archive must exist with archive.ok true
    # (run_melee_export.ps1: no packed != unpacked file, no unexpected file) and each staged set must be read back equal
    # (unbundle_compare packed_equals_unpacked true); otherwise nothing is written.
    arch = man.get('archive')
    if not isinstance(arch, dict):
        raise SystemExit("MELEE_XL REFUSED: the run manifest has no 'archive' (not packed: -NoPack or nothing staged); nothing written")
    if arch.get('ok') is not True:
        raise SystemExit('MELEE_XL REFUSED: archive.ok = %r (mismatches %r, unexpected files %r) for %s; nothing written'
                         % (arch.get('ok'), arch.get('mismatches'), arch.get('unexpected_files'), arch.get('path')))
    packed_ok = {c.get('file') for c in (arch.get('unbundle_compare') or []) if c.get('packed_equals_unpacked') is True}
    refused = []
    for s in reg['plan']:
        name = L.utpp_name(s)
        if name in staged and name not in packed_ok:
            refused.append('%s (%s: not read back equal from the packed archive)' % (s, name))
    if refused:
        raise SystemExit('MELEE_XL REFUSED sets whose pack failed: %s; nothing written' % '; '.join(refused))
    groups = {}
    for s, rows in reg['plan'].items():
        if L.utpp_name(s) not in staged:
            continue
        for r in rows:
            if not r['mirrored']:
                continue
            k = (s, r['component'], tuple(r['utpp_vars']), r['utpp_priority'])
            groups.setdefault(k, []).append(r['entity_depot'])
    staged_sets = {s for s in reg['plan'] if L.utpp_name(s) in staged}
    lines = [header(groups, reg['plan'], staged_sets).rstrip('\n')]
    for s in L.SETS:
        for (s2, comp, vars_, pr), ents in sorted(groups.items(), key=lambda kv: (kv[0][3], kv[0][2])):
            if s2 != s:
                continue
            lines.append('  - entity:')
            for e in sorted(set(ents), key=lambda x: (x.startswith('ep1'), x)):
                lines.append('      - %s' % e)
            lines.append('    component: %s' % comp)
            lines.append('    set: %s\\%s' % (L.UTPP_DEPOT_DIR, L.utpp_name(s)))
            lines.append('    vars: [%s]' % ', '.join(vars_))
            lines.append('    priority: %d' % pr)
    txt = '\n'.join(lines) + '\n'
    # the comment block (everything before 'animations:') must be comments only and free of local paths
    head = txt.split('\nanimations:\n', 1)[0]
    bad = [l for l in head.split('\n') if not l.startswith('#') or LOCAL_PATH_RX.search(l)]
    if bad:
        raise SystemExit('MELEE_XL REFUSED: comment block has a non-comment line or a local path: %r; nothing written' % bad[:3])
    # validation: YAML parse (when PyYAML is available) and the entries read back equal the plan
    try:
        import yaml
        y = yaml.safe_load(txt)
        back = {(e['set'].split('\\')[-1], e['component'], tuple(e['vars']), int(e['priority'])): sorted(e['entity']) for e in y['animations']}
        want = {(L.utpp_name(k[0]), k[1], k[2], k[3]): sorted(set(v)) for k, v in groups.items()}
        if back != want:
            raise SystemExit('xl read back differs from the plan')
        yaml_ok = True
    except ImportError:
        yaml_ok = None
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(txt)
    print('MELEE_XL yaml read-back %s' % ('OK' if yaml_ok else 'not checked (no PyYAML)'))
    if a.copy_to:
        shutil.copy2(a.out, os.path.join(a.copy_to, os.path.basename(a.out)))
    print('MELEE_XL entries %d sets %d -> %s sha256 %s' % (len(groups), len({k[0] for k in groups}), a.out, L.sha256(a.out)))


if __name__ == '__main__':
    main()
