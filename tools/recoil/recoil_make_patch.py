"""Step 5 of recoil-missing-20261109: write the integration patch WITHOUT applying it.

Reads (never writes) Source\\plugin\\UTPP\\src\\mm\\RecoilProfiles.hpp, Source\\plugin\\UTPP\\scripts\\utpp_motion.reds,
Source\\tools\\analysis\\export_nue_native_recoil_curves.py, Source\\tools\\release\\make_release.py and the final set
<work>\\out\\woman. Writes below <work>\\patch: orig\\ (copies), A\\, A-bridge\\, AB-bridge\\ (patched copies) and the diffs
  A   (recommended)  families whose class already gets a fallback kick today (Handgun -> Liberty, SMG -> Saratoga, Rifle/LMG -> Ajax):
                     their own curve replaces a wrong family's curve, nothing else changes;
  bridge             reds only: a record without a usable family tag resolves through the animation wrapper of its record
                     (friendlyName, the gate of its player animset) before the class fallback;
  B   (optional)     shotguns, precision/sniper rifles, HMG: today profile 0 on purpose (utpp_motion.reds FallbackRecoilProfile: their
                     native shot carries a pump/bolt cycle); needs a design decision and a runtime check, applies on top of A + bridge.
Diffs keep the line ends of the originals (CRLF stays CRLF).
"""
from __future__ import annotations

import difflib
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recoil_common as rc  # noqa: E402
import export_missing_recoil as em  # noqa: E402

WORK = rc.ROOT / 'Build/work/recoil-missing-20261109'
SRC = {'RecoilProfiles.hpp': rc.ROOT / 'Source/plugin/UTPP/src/mm/RecoilProfiles.hpp',
       'utpp_motion.reds': rc.ROOT / 'Source/plugin/UTPP/scripts/utpp_motion.reds',
       'export_nue_native_recoil_curves.py': rc.ROOT / 'Source/tools/analysis/export_nue_native_recoil_curves.py',
       'make_release.py': rc.ROOT / 'Source/tools/release/make_release.py'}
GROUP_A = ['kenshin', 'omaha', 'ticon', 'defender', 'dian', 'warden', 'borg4a', 'hercules']  # tagless last: drop 26-27 without bridge
GROUP_B = ['crusher', 'achilles', 'sor22', 'pozhar', 'grad', 'nekomata', 'rasetsu', 'carnage', 'tactician', 'zhuo', 'igla', 'palica',
           'satara', 'testera', 'hmg', 'kolac', 'osprey']                                     # tagless last (43, 44)
TAG = {'kenshin': 'Kenshin', 'omaha': 'Omaha', 'ticon': 'Ticon', 'defender': 'Defender', 'dian': 'Dian', 'warden': 'Warden',
       'borg4a': 'Borg4a', 'hercules': 'Hercules', 'crusher': 'Crusher', 'achilles': 'Achilles', 'sor22': 'Sor22', 'pozhar': 'Pozhar',
       'grad': 'Grad', 'nekomata': 'Nekomata', 'rasetsu': 'Rasetsu', 'carnage': 'Carnage', 'tactician': 'Tactician', 'zhuo': 'Zhuo',
       'igla': 'Igla', 'palica': 'Palica', 'satara': 'Satara', 'testera': 'Testera', 'hmg': 'HMG', 'kolac': 'Kolac', 'osprey': 'Osprey'}
TAGLESS = {'borg4a', 'hercules', 'kolac', 'osprey'}
# friendlyName of the record (= animation wrapper that gates the player animset in player_wa_fpp(_ep1).ent) -> family, weapon(s)
BRIDGE_A = [('w_handgun_arasaka_yukimura', 'Yukimura', 'Kappa'), ('w_lmg_constitutional_defender', 'Defender', 'MA70'),
            ('w_rifle_assault_nokota_copperhead', 'Copperhead', 'Umbra'),
            ('w_rifle_assault_arasaka_masamune', 'Masamune', 'Masamune presets without the Masamune tag'),
            ('w_submachinegun_militech_saratoga', 'Saratoga', 'Guillotine (same profile as its fallback today)'),
            ('w_smg_midnight_borg', 'Borg4a', 'Borg4a'), ('w_special__militech_hercules', 'Hercules', 'Hercules')]
BRIDGE_B = [('w_rifle_precision_rostovic_kolac', 'Kolac', 'Kolac'), ('w_rifle_sniper_nokota_osprey', 'Osprey', 'Osprey'),
            ('w_rifle_sniper_tsunami_rasetsu', 'Rasetsu', 'Rasetsu preset without the Rasetsu tag')]
FIRST_NEW_ID = 20


def read(path: Path):
    raw = path.read_bytes().decode('utf-8')
    eol = '\r\n' if '\r\n' in raw else '\n'
    return raw.replace('\r\n', '\n'), eol


def sub(text: str, old: str, new: str, what: str):
    if text.count(old) != 1:
        raise SystemExit(f'anchor not unique/found ({what}): {old[:80]!r}')
    return text.replace(old, new)


def hpp(text: str, fams: list, first_id: int, size: int, comment: str):
    rows = ''.join(f'    {{"{f}", "{f}_single_shot_shoulder.utpprecoil", "{f}_single_shot_ironsight.utpprecoil", '
                   f'{"true" if em.PLAN[f][3] else "false"}}}, // {first_id + i} {TAG[f]}\n' for i, f in enumerate(fams))
    text = sub(text, '}};\ninline constexpr int32_t kSlots', comment + rows + '}};\ninline constexpr int32_t kSlots', 'kTable end')
    old_size = text.split('inline constexpr std::array<Entry, ', 1)[1].split('>', 1)[0]
    return sub(text, f'inline constexpr std::array<Entry, {old_size}> kTable', f'inline constexpr std::array<Entry, {size}> kTable',
               'kTable size')


def reds_lines(text: str, fams: list, first_id: int, comment: str):
    lines = ''.join(f'  if Equals(family, n"{TAG[f]}") {{ return {first_id + i}; }}\n' for i, f in enumerate(fams))
    anchor = '  if Equals(family, n"Senkoh") { return 19; }\n'
    tail = text.split(anchor, 1)[1]
    end = tail.index('  return 0;\n}\n')
    return text.replace(anchor + tail[:end], anchor + tail[:end] + comment + lines, 1)


def bridge_fn(entries):
    body = ''.join(f'  if Equals(name, "{n}") {{ return n"{fam}"; }} // {who}\n' for n, fam, who in entries)
    return ('\n// Family of a weapon record that carries no usable family tag, from the animation wrapper of the record (Item_Record.FriendlyName,\n'
            '// the variable that gates its player animset in player_wa_fpp(_ep1).ent). Resolved after the tags and before the class\n'
            '// fallback; a Nue variant is never listed (the Nue stays bound to Items.Preset_Nue_Default). n"" = no family.\n'
            'public func UTPPMotionRecoilFamilyOfAnimset(name: String) -> CName {\n' + body + '  return n"";\n}\n')


def exporter(text: str, fams: list, b_group: bool):
    entries = ''.join(f"    '{f}': '{em.PLAN[f][0]}',\n" for f in fams)
    note = ('    # 2.5 additions (recoil-missing-20261109, group B: shotguns, precision/sniper rifles, HMG)\n' if b_group else
            '    # 2.5 additions (recoil-missing-20261109): own single-shot clips; overrides in PROFILE_CLIP_OVERRIDES / PROFILE_BUDGET_FACTOR\n')
    if b_group:
        text = sub(text, "\n}\n# Nue is not a --profiles family", '\n' + note + entries.rstrip('\n') + '\n}\n# Nue is not a --profiles family',
                   'PROFILE_SOURCES end')
    else:
        text = sub(text, "    'senkoh': 'pwa_w_smg__senkoh_lx.anims.glb',\n}",
                   "    'senkoh': 'pwa_w_smg__senkoh_lx.anims.glb',\n" + note + entries + '}', 'PROFILE_SOURCES end')
    over = {f: {k: c for k, c, d in (('shoulder', em.PLAN[f][1], em.SHOULDER), ('ironsight', em.PLAN[f][2], em.IRON)) if c != d}
            for f in fams}
    over = {f: v for f, v in over.items() if v}
    gains = {k: v for k, v in em.CLIP_GAIN.items() if k[0] in fams}
    if not b_group:
        add = ("\n# Clip of a family whose animset has no add_recoil_single_shot_ironsight (tech weapons aim with the charged shot) or no ADS\n"
               "# clip at all; per-family budget factor on the final peak (hand-IK reach, recoil-missing-20261109): budget = factor x peak.\n"
               "PROFILE_CLIP_OVERRIDES = {\n" + ''.join(f"    '{f}': {v!r},\n" for f, v in over.items()) + '}\n'
               "PROFILE_BUDGET_FACTOR = {\n" + ''.join(f"    {k!r}: {v},\n" for k, v in gains.items()) + '}\n'
               "# male pma_* files whose name differs from the pwa_* source by more than the prefix\n"
               "PMA_NAME_FIXES = {'pwa_w_rifle_precision_midnight_sor22.anims.glb': 'pwa_w_rifle_precision__midnight_sor22.anims.glb'}\n")
        text = sub(text, "PROFILE_CLIPS = (('shoulder', 'add_recoil_single_shot_shoulder'), ('ironsight', 'add_recoil_single_shot_ironsight'))\n",
                   "PROFILE_CLIPS = (('shoulder', 'add_recoil_single_shot_shoulder'), ('ironsight', 'add_recoil_single_shot_ironsight'))\n" + add,
                   'PROFILE_CLIPS')
        text = sub(text, "        for kind, clip in PROFILE_CLIPS:\n            if clip not in animation.clips:\n",
                   "        for kind, clip in PROFILE_CLIPS:\n            clip = PROFILE_CLIP_OVERRIDES.get(name, {}).get(kind, clip)\n"
                   "            if clip not in animation.clips:\n", 'run_profiles clip loop')
        text = sub(text, "            rows, clip_report = weapon_curve(animation, clip, samples, fade, reference, limit_amplitude, start_ramp, budget)\n"
                         "            curves[kind] = (rows, clip_report)\n",
                   "            rows, clip_report = weapon_curve(animation, clip, samples, fade, reference, limit_amplitude, start_ramp, budget)\n"
                   "            factor = PROFILE_BUDGET_FACTOR.get((name, kind))\n"
                   "            if factor:\n"
                   "                peak_t, peak_r = row_peaks(rows)\n"
                   "                rows, clip_report = weapon_curve(animation, clip, samples, fade, reference, limit_amplitude, start_ramp,\n"
                   "                                                 (factor * peak_t, factor * peak_r))\n"
                   "                clip_report['familyBudgetFactor'] = factor\n"
                   "            curves[kind] = (rows, clip_report)\n", 'run_profiles budget factor')
        text = sub(text, "                name = 'pwa_' + name[len(prefix) + 1:]\n",
                   "                name = 'pwa_' + name[len(prefix) + 1:]\n                name = PMA_NAME_FIXES.get(name, name)\n", 'find_sources')
        # the summary line split every clip name at 'single_shot_' ([1] raised IndexError for add_recoil_charged_* after the files
        # and the report were written)
        text = sub(text, "c['clip'].split('single_shot_')[1]}", "c['clip'].split('single_shot_')[-1]}", 'summary print')
        # --compare-male (COMPARE_SOURCES includes every PROFILE_SOURCES family) reads the same clips as --profiles
        text = sub(text, "        for kind, clip in PROFILE_CLIPS:\n            if clip not in fa.clips or clip not in ma.clips:\n",
                   "        for kind, clip in PROFILE_CLIPS:\n            clip = PROFILE_CLIP_OVERRIDES.get(name, {}).get(kind, clip)\n"
                   "            if clip not in fa.clips or clip not in ma.clips:\n", 'compare_profiles clip loop')
    else:
        start = text.index('PROFILE_CLIP_OVERRIDES = {\n')
        end = text.index('}\n', start)
        text = text[:end] + ''.join(f"    '{f}': {v!r},\n" for f, v in over.items()) + text[end:]
    return text


def release(text: str, fams: list, count_after: int):
    files = []
    for f in fams:
        for kind in ('ironsight', 'shoulder'):
            p = WORK / 'out/woman' / f'{f}_single_shot_{kind}.utpprecoil'
            files.append(f'{p.name} {p.stat().st_size} {rc.sha256_file(p)}')
    head, rest = text.split('RECOIL = """\\\n', 1)
    block, tail = rest.split('"""', 1)
    lines = sorted(set(block.strip('\n').split('\n')) | set(files))
    head = head.rsplit('\n# ', 1)
    old_comment = head[1]
    if not old_comment.split(' ', 1)[0].isdigit():
        raise SystemExit('unexpected RECOIL comment: ' + old_comment)
    comment = (f'{count_after} weapon families x {{shoulder, ironsight}}; recoil profiles extracted from the game\'s own weapon animations '
               f'(2.5 additions: recoil-missing-20261109).\n')
    return head[0] + '\n# ' + comment + 'RECOIL = """\\\n' + '\n'.join(lines) + '\n"""' + tail


def write(folder: str, name: str, text: str, eol: str):
    p = WORK / 'patch' / folder / name
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(text.replace('\n', eol).encode('utf-8'))
    return p


def diff(a_label: str, a: str, b_label: str, b: str, eol: str, out: Path):
    lines = difflib.unified_diff(a.replace('\n', eol).splitlines(keepends=True), b.replace('\n', eol).splitlines(keepends=True),
                                 a_label, b_label, n=3)
    out.write_bytes(''.join(lines).encode('utf-8'))
    return out


def main():
    texts = {k: read(v) for k, v in SRC.items()}
    patch = WORK / 'patch'
    for k, v in SRC.items():
        (patch / 'orig').mkdir(parents=True, exist_ok=True)
        shutil.copyfile(v, patch / 'orig' / k)
    rel = {k: str(v.relative_to(rc.ROOT)).replace('\\', '/') for k, v in SRC.items()}
    a_id, b_id = FIRST_NEW_ID, FIRST_NEW_ID + len(GROUP_A)
    total = b_id + len(GROUP_B)
    comment_a = ('    // 2.5 additions (recoil-missing-20261109), each from its own single-shot clips of the 2.5 animset (same exporter settings).\n'
                 '    // Kenshin, Omaha, Ticon aim with add_recoil_charged_ironsight (no single_shot_ironsight in their animsets); Kenshin ADS\n'
                 '    // and Omaha use a per-family budget of 0.5 x their peak (hand-IK reach). Borg4a and Hercules records carry no family tag:\n'
                 '    // reached only through utpp_motion.reds UTPPMotionRecoilFamilyOfAnimset (drop ids 26-27 if that bridge is not taken).\n')
    comment_b = ('    // 2.5 group B (optional): shotguns, precision/sniper rifles, HMG from their own single-shot clips. The curve carries the\n'
                 '    // WeaponRight motion only; the hand cycle (pump/bolt) of the native shot is not in it (utpp_motion.reds FallbackRecoilProfile).\n'
                 '    // Achilles/Satara aim with the charged ironsight clip, Nekomata uses the charged clips, Carnage has no ADS clip (hip clip).\n'
                 '    // Kolac and Osprey records carry no family tag (bridge).\n')
    out = {}
    h, eol = texts['RecoilProfiles.hpp']
    out['hpp_A'] = hpp(h, GROUP_A, a_id, b_id, comment_a)
    out['hpp_AB'] = hpp(out['hpp_A'], GROUP_B, b_id, total, comment_b)
    write('A', 'RecoilProfiles.hpp', out['hpp_A'], eol)
    write('AB-bridge', 'RecoilProfiles.hpp', out['hpp_AB'], eol)
    diff('a/' + rel['RecoilProfiles.hpp'], h, 'b/' + rel['RecoilProfiles.hpp'], out['hpp_A'], eol, patch / 'RecoilProfiles.hpp.diff')
    diff('a/' + rel['RecoilProfiles.hpp'], out['hpp_A'], 'b/' + rel['RecoilProfiles.hpp'], out['hpp_AB'], eol,
         patch / 'RecoilProfiles.hpp.B.diff')

    r, eol = texts['utpp_motion.reds']
    reds_a = reds_lines(r, GROUP_A, a_id, '  // 2.5 additions (recoil-missing-20261109); Borg4a and Hercules records carry no tag: UTPPMotionRecoilFamilyOfAnimset\n')
    fn_end = reds_a.index('\n}\n', reds_a.index('public func UTPPMotionRecoilProfileOfFamily(family: CName) -> Int32 {')) + 3
    reds_bridge = reds_a[:fn_end] + bridge_fn(BRIDGE_A) + reds_a[fn_end:]
    reds_bridge = sub(reds_bridge, '    if profile == 0 {\n      return this.FallbackRecoilProfile(weapon);\n    }\n',
                      '    if profile == 0 {\n      // no usable family tag: the animation wrapper of the record (Kappa, MA70, Umbra, untagged presets)\n'
                      '      profile = UTPPMotionRecoilProfileOfFamily(UTPPMotionRecoilFamilyOfAnimset(rec.FriendlyName()));\n    }\n'
                      '    if profile == 0 {\n      return this.FallbackRecoilProfile(weapon);\n    }\n', 'ResolveRecoilProfile')
    reds_b = reds_lines(reds_bridge, GROUP_B, b_id,
                        '  // 2.5 group B (shotguns, precision/sniper rifles, HMG; own curves); Kolac and Osprey records carry no tag\n')
    last_a = f'  if Equals(name, "{BRIDGE_A[-1][0]}") {{ return n"{BRIDGE_A[-1][1]}"; }} // {BRIDGE_A[-1][2]}\n'
    reds_b = sub(reds_b, last_a, last_a + ''.join(f'  if Equals(name, "{n}") {{ return n"{fam}"; }} // {who}\n' for n, fam, who in BRIDGE_B),
                 'bridge B entries')
    reds_b = sub(reds_b,
                 '    // Shotguns, precision/sniper rifles and HMGs stay 0 on purpose: their native shot animation carries a bolt/pump cycle that an\n'
                 '    // Ajax kick would replace (review 2026-10-23); add them only with a profile that keeps that cycle.\n',
                 '    // Shotguns, precision/sniper rifles and HMGs without a family profile stay 0 on purpose: their native shot animation carries\n'
                 '    // a bolt/pump cycle that an Ajax kick would replace. The 2.5 group B families have their own curve.\n'
                 '    // It carries the weapon motion only, not the hand cycle - enabled only after a game test.\n',
                 'fallback comment')
    write('A', 'utpp_motion.reds', reds_a, eol)
    write('A-bridge', 'utpp_motion.reds', reds_bridge, eol)
    write('AB-bridge', 'utpp_motion.reds', reds_b, eol)
    lab = rel['utpp_motion.reds']
    diff('a/' + lab, r, 'b/' + lab, reds_a, eol, patch / 'utpp_motion.reds.diff')
    diff('a/' + lab, reds_a, 'b/' + lab, reds_bridge, eol, patch / 'utpp_motion.reds.bridge.diff')
    diff('a/' + lab, reds_bridge, 'b/' + lab, reds_b, eol, patch / 'utpp_motion.reds.B.diff')

    x, eol = texts['export_nue_native_recoil_curves.py']
    x_a = exporter(x, GROUP_A, False)
    x_b = exporter(x_a, GROUP_B, True)
    write('A', 'export_nue_native_recoil_curves.py', x_a, eol)
    write('AB-bridge', 'export_nue_native_recoil_curves.py', x_b, eol)
    lab = rel['export_nue_native_recoil_curves.py']
    diff('a/' + lab, x, 'b/' + lab, x_a, eol, patch / 'export_nue_native_recoil_curves.py.diff')
    diff('a/' + lab, x_a, 'b/' + lab, x_b, eol, patch / 'export_nue_native_recoil_curves.py.B.diff')

    m, eol = texts['make_release.py']
    m_a = release(m, GROUP_A, 16 + len(GROUP_A))
    m_b = release(m_a, GROUP_B, 16 + len(GROUP_A) + len(GROUP_B))
    write('A', 'make_release.py', m_a, eol)
    write('AB-bridge', 'make_release.py', m_b, eol)
    lab = rel['make_release.py']
    diff('a/' + lab, m, 'b/' + lab, m_a, eol, patch / 'make_release.py.diff')
    diff('a/' + lab, m_a, 'b/' + lab, m_b, eol, patch / 'make_release.py.B.diff')

    manifest = dict(generator='Source/tools/recoil/recoil_make_patch.py', groupA=GROUP_A, groupB=GROUP_B,
                    ids={f: FIRST_NEW_ID + i for i, f in enumerate(GROUP_A + GROUP_B)}, tagless=sorted(TAGLESS),
                    bridgeA=BRIDGE_A, bridgeB=BRIDGE_B, sources={k: dict(path=rel[k], sha256=rc.sha256_file(v)) for k, v in SRC.items()},
                    diffs={p.name: rc.sha256_file(p) for p in sorted(patch.glob('*.diff'))})
    (patch / 'patch-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    for p in sorted(patch.glob('*.diff')):
        print(f'{p.name:<44} {p.stat().st_size:>6} bytes')


if __name__ == '__main__':
    main()
