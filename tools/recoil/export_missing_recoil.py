"""Step 3 of recoil-missing-20261109: export the missing weapon recoil profiles (woman pwa_* and man pma_*, game 2.5).

Same numeric core and settings as the shipped 0.1.0 profiles (recoil_common.SETTINGS -> export_nue_native_recoil_curves.weapon_curve,
sampler_text, CRLF), same file names (<family>_single_shot_{shoulder,ironsight}.utpprecoil), same units (seconds, unit quaternion,
metres, weapon-local CP basis). Sources: Extracted\\game\\player-anims\\glb (2.5 content/ep1). Only the clip choice differs where an
animset has no add_recoil_single_shot_ironsight (see PLAN, column ads) and the optional --window (seconds) that limits a clip to its
first part (evaluated for the revolver hammer-cock; not used by the default plan).

Writes <out>\\woman\\*, <out>\\man\\* and <out>\\export-report.json. Read only on every source.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # python -I does not add the script folder
import recoil_common as rc  # noqa: E402

SHOULDER, IRON = 'add_recoil_single_shot_shoulder', 'add_recoil_single_shot_ironsight'
CH_IRON, CH_HIP = 'add_recoil_charged_ironsight', 'add_recoil_charged'
# family (lower-case reds key) -> (pwa animset file, hip clip, ADS clip, longGun, item type of the records, note)
PLAN = {
    # handguns / revolvers (Pistol pose, longGun false)
    'kenshin': ('pwa_w_handgun__arasaka_kenshin.anims.glb', SHOULDER, CH_IRON, False, 'Wea_Handgun', 'tech: ADS fires the charged shot'),
    'omaha': ('pwa_w_handgun__militech_omaha.anims.glb', SHOULDER, CH_IRON, False, 'Wea_Handgun', 'tech: ADS fires the charged shot'),
    'grit': ('pwa_w_handgun__krausser_grit.anims.glb', SHOULDER, CH_IRON, False, 'Wea_Handgun', 'ep1, tech'),
    'ticon': ('pwa_w_handgun__militech_ticon.anims.glb', SHOULDER, CH_IRON, False, 'Wea_Handgun', 'ep1, tech'),
    'silverhand': ('pwa_w_handgun__malorian_silverhand.anims.glb', SHOULDER, CH_IRON, False, 'Wea_Handgun', 'no family tag'),
    'metel': ('pwa_w_revolver__techtronika_metel.anims.glb', SHOULDER, IRON, False, 'Wea_Revolver', 'ep1 revolver'),
    'nova': ('pwa_w_revolver__darra_nova.anims.glb', SHOULDER, IRON, False, 'Wea_Revolver', 'retired id 8 (hammer-cock)'),
    'overture': ('pwa_w_revolver__malorian_overture.anims.glb', SHOULDER, IRON, False, 'Wea_Revolver', 'retired id 9 (hammer-cock)'),
    'burya': ('pwa_w_revolver__techtronika_burya.anims.glb', SHOULDER, IRON, False, 'Wea_Revolver', 'retired id 10 (hammer-cock)'),
    # long guns (Rifle pose, longGun true)
    'crusher': ('pwa_w_revolver__militech_crusher.anims.glb', SHOULDER, IRON, True, 'Wea_Shotgun', 'revolver-folder animset, shotgun type'),
    'defender': ('pwa_w_lmg__constitutional_defender.anims.glb', SHOULDER, IRON, True, 'Wea_LightMachineGun', ''),
    'dian': ('pwa_w_special_kangtao_dian.anims.glb', SHOULDER, IRON, True, 'Wea_SubmachineGun', ''),
    'warden': ('pwa_w_smg__krausser_warden.anims.glb', SHOULDER, IRON, True, 'Wea_SubmachineGun', 'ep1'),
    'borg4a': ('pwa_w_smg__midnight_borg.anims.glb', SHOULDER, IRON, True, 'Wea_SubmachineGun', 'ep1, no family tag'),
    'hercules': ('pwa_w_special__militech_hercules.anims.glb', SHOULDER, IRON, True, 'Wea_Rifle', 'ep1, no family tag'),
    'achilles': ('pwa_w_rifle_precision__militech_achilles.anims.glb', SHOULDER, CH_IRON, True, 'Wea_PrecisionRifle', 'tech: ADS charged'),
    'sor22': ('pwa_w_rifle_precision__midnight_sor22.anims.glb', SHOULDER, IRON, True, 'Wea_PrecisionRifle', ''),
    'kolac': ('pwa_w_rifle_precision__rostovic_kolac.anims.glb', SHOULDER, IRON, True, 'Wea_PrecisionRifle', 'no family tag'),
    'pozhar': ('pwa_w_rifle_precision__techtronika_pozhar.anims.glb', SHOULDER, IRON, True, 'Wea_Shotgun', 'precision animset, shotgun type'),
    'ashura': ('pwa_w_rifle_sniper__tsunami_ashura.anims.glb', SHOULDER, IRON, True, 'Wea_SniperRifle', ''),
    'grad': ('pwa_w_rifle_sniper__techtronika_grad.anims.glb', SHOULDER, IRON, True, 'Wea_SniperRifle', ''),
    'nekomata': ('pwa_w_rifle_sniper__tsunami_nekomata.anims.glb', CH_HIP, CH_IRON, True, 'Wea_SniperRifle', 'charge only: both clips charged'),
    'osprey': ('pwa_w_rifle_sniper__nokota_osprey.anims.glb', SHOULDER, IRON, True, 'Wea_SniperRifle', 'ep1, no family tag'),
    'rasetsu': ('pwa_w_rifle_sniper__tsunami_rasetsu.anims.glb', SHOULDER, IRON, True, 'Wea_SniperRifle', 'ep1, tag on 1 of 2 presets'),
    'carnage': ('pwa_w_shotgun__budget_carnage.anims.glb', SHOULDER, SHOULDER, True, 'Wea_Shotgun', 'no ADS clip at all: shoulder clip for ADS'),
    'tactician': ('pwa_w_shotgun__constitutional_arms_tactician.anims.glb', SHOULDER, IRON, True, 'Wea_Shotgun', ''),
    'zhuo': ('pwa_w_special__kangtao_zhuo.anims.glb', SHOULDER, IRON, True, 'Wea_Shotgun', ''),
    'igla': ('pwa_w_shotgun_dual__rostovic_igla.anims.glb', SHOULDER, IRON, True, 'Wea_ShotgunDual', ''),
    'palica': ('pwa_w_shotgun_dual__rostovic_palica.anims.glb', SHOULDER, IRON, True, 'Wea_ShotgunDual', ''),
    'satara': ('pwa_w_shotgun_dual__rostovic_satara.anims.glb', SHOULDER, CH_IRON, True, 'Wea_ShotgunDual', 'tech: ADS charged'),
    'testera': ('pwa_w_shotgun_dual__rostovic_testera.anims.glb', SHOULDER, IRON, True, 'Wea_ShotgunDual', ''),
    'hmg': ('pwa_w_special__militech_hmg.anims.glb', SHOULDER, IRON, True, 'Wea_HeavyMachineGun', 'heavy weapon (TakeAndEquip)'),
}
MALE_NAME = {'pwa_w_rifle_precision__midnight_sor22.anims.glb': 'pma_w_rifle_precision_midnight_sor22.anims.glb'}  # single '_' in 2.5
RAW = dict(rc.SETTINGS, start_ramp=0.0, budget=None)  # shape diagnostics only (same fade/reference, no ramp, no budget)
# Final set (step 4 evidence, Build\work\recoil-missing-20261109\report.txt): families left out of the patch and why.
EXCLUDED = {
    'grit': 'hip clip fails the hand-IK reach in every variant (full, 0.25 s window, budget x0.5 / x0.25, first-key reading); '
            'stays on the Handgun fallback (Liberty)',
    'metel': 'revolver like the retired ids 8-10: hip and ADS fail the reach (full, 0.25 s window, budget x0.5); stays on profile 0',
    'nova': 'retired id 8 (revolver hammer-cock): fails the reach again (full, 0.25 s window, window + budget x0.5); stays on profile 0',
    'overture': 'retired id 9 (same hip clip as Nova): fails the reach (full, 0.25 s window, window + budget x0.5); stays on profile 0',
    'burya': 'retired id 10: fails the reach (full, 0.25 s window, window + budget x0.5); stays on profile 0',
    'ashura': 'male curve outside the parity bounds of the shared file (hip 15.5 mm / 0.056 rad, ADS 0.034 rad > 15 mm / 0.02 rad); '
              'needs per-body recoil in the DLL; stays on profile 0',
    'silverhand': 'male ADS curve 0.0226 rad > 0.02 rad parity bound (hip passes); stays on the Handgun fallback (Liberty)',
}
# Per-family budget below the shipped 8 cm / 0.50 rad cap: (family, kind) -> factor on the final peak of the shipped method
# (the budget becomes factor x that peak; the same uniform-scale mechanism, recorded as amplitudeScale). Evidence: reach run V-1
# (Kenshin ADS, Omaha) and the patched RecoilProfilesTest burst step (Defender).
CLIP_GAIN = {('kenshin', 'ironsight'): 0.5, ('omaha', 'shoulder'): 0.5, ('omaha', 'ironsight'): 0.5,
             # Defender: 0.1 s kick, burst step 5.16 / 5.32 deg per frame > 5 deg at full peak (RecoilProfilesTest)
             ('defender', 'shoulder'): 0.85, ('defender', 'ironsight'): 0.85}


def shape(rows):
    """Diagnostics of an un-ramped, un-budgeted curve: peak, timing, late motion (cycle) against the early kick."""
    t = [r[0] for r in rows]
    tr = [math.sqrt(r[5] ** 2 + r[6] ** 2 + r[7] ** 2) for r in rows]
    an = [2 * math.atan2(math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2), abs(r[4])) for r in rows]
    i = max(range(len(tr)), key=tr.__getitem__)
    early = [k for k in range(len(t)) if t[k] <= 0.15]
    late = [k for k in range(len(t)) if t[k] > 0.15]
    e_t, l_t = max(tr[k] for k in early), max((tr[k] for k in late), default=0.0)
    e_r, l_r = max(an[k] for k in early), max((an[k] for k in late), default=0.0)
    comp = [max(abs(r[5 + c]) for r in rows) for c in range(3)]
    return dict(peakT=tr[i], tPeakT=t[i], peakR=max(an), tPeakR=t[max(range(len(an)), key=an.__getitem__)],
                earlyT=e_t, lateT=l_t, earlyR=e_r, lateR=l_r, lateToEarlyT=l_t / e_t if e_t > 1e-6 else float('inf'),
                lateToEarlyR=l_r / e_r if e_r > 1e-6 else float('inf'), peakComponentsT=comp)


def export(families, out: Path, window=None, gain=None, suffix='', reference=None):
    """window: sample only the first `window` seconds; gain: extra uniform factor on the shipped-method curve (budget = gain x its
    final peak); suffix: appended to the family part of the file name (variant study, e.g. kenshin-g50_single_shot_ironsight)."""
    index_w = rc.index_glbs([rc.GLB_25], 'pwa')
    index_m = rc.index_glbs([rc.GLB_25], 'pma')
    result = []
    for fam in families:
        pwa, hip, ads, long_gun, item_type, note = PLAN[fam]
        pma = MALE_NAME.get(pwa, 'pma_' + pwa[4:])
        for sex, glb in (('woman', index_w.get(pwa)), ('man', index_m.get(pma))):
            entry = dict(family=fam, sex=sex, source=str(glb.relative_to(rc.ROOT)) if glb else None, longGun=long_gun,
                         itemType=item_type, note=note, files=[], clips=[], reasons=[])
            result.append(entry)
            if glb is None:
                entry['reasons'].append('animset GLB missing: ' + (pwa if sex == 'woman' else pma))
                continue
            entry['sourceSha256'] = rc.sha256_file(glb)
            clips = rc.clips_of(glb)
            for kind, clip in (('shoulder', hip), ('ironsight', ads)):
                if clip not in clips:
                    entry['reasons'].append(f'{kind}: clip {clip} missing')
                    continue
                if window:
                    duration, channels = clips[clip]
                    clips[clip] = (min(duration, window), channels)  # same core, sampled over the first `window` seconds
                data, rows, rep = rc.export_clip(glb, clip, dict(rc.SETTINGS, reference=reference or rc.SETTINGS['reference']))
                clip_gain = gain if gain is not None else CLIP_GAIN.get((fam, kind))
                if clip_gain:
                    peak_t, peak_r = rc.core.row_peaks(rows)
                    data, rows, rep = rc.export_clip(glb, clip, dict(rc.SETTINGS, reference=reference or rc.SETTINGS['reference']), budget_override=(clip_gain * peak_t, clip_gain * peak_r))
                    rep = dict(rep, familyBudgetFactor=clip_gain)
                _, raw_rows, _ = rc.export_clip(glb, clip, dict(RAW, reference=reference or RAW['reference']))
                if window:
                    clips[clip] = (duration, channels)
                rep = {k: v for k, v in rep.items() if k != 'reasons'} | dict(reasons=rep['reasons'], kind=kind, shape=shape(raw_rows))
                entry['clips'].append(rep)
                entry['reasons'] += [f'{kind} {clip}: {r}' for r in rep['reasons']]
                dest = out / sex / f'{fam}{suffix}_single_shot_{kind}.utpprecoil'
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
                entry['files'].append(dict(path=str(dest.relative_to(rc.ROOT)), size=len(data), sha256=rc.sha256_bytes(data)))
            c = {x['kind']: x for x in entry['clips']}
            print(f"{fam:<11} {sex:<5} " + ' | '.join(
                f"{k[:4]} {x['clip'].replace('add_recoil_', '')[:22]:<22} {x['duration']:.3f}s raw {x['rawPeakTranslationM'] * 100:5.1f}cm/"
                f"{x['rawPeakRotationRad']:.3f} x{x['amplitudeScale']:.2f} late/early {x['shape']['lateToEarlyT']:.2f}"
                f"{'*' if x['impulseStart'] else ''}" for k, x in c.items()) + (' REASONS: ' + '; '.join(entry['reasons']) if entry['reasons'] else ''))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=rc.ROOT / 'Build/work/recoil-missing-20261109/out')
    ap.add_argument('--families', default='all', help='comma separated PLAN keys, "all" (final set: PLAN minus EXCLUDED) or "every"')
    ap.add_argument('--window', type=float, help='seconds: export only the first part of each clip (variant study, not the plan)')
    ap.add_argument('--gain', type=float, help='extra uniform amplitude factor (0..1) on top of the shipped budget (variant study)')
    ap.add_argument('--suffix', default='', help='appended to the family part of every file name (variant study)')
    ap.add_argument('--reference', choices=('bind', 'first'), help='curve reference (default bind = shipped method; first = Nue reading)')
    ap.add_argument('--report', type=Path, help='JSON report (default <out>\\export-report.json)')
    a = ap.parse_args()
    fams = [f for f in PLAN if f not in EXCLUDED] if a.families == 'all' else list(PLAN) if a.families == 'every' else a.families.split(',')
    out = a.out.resolve()
    if not out.is_relative_to(rc.ROOT):
        raise SystemExit('output must stay inside the project')
    result = export(fams, out, a.window, a.gain, a.suffix, a.reference)
    report = (a.report or out / 'export-report.json').resolve()
    report.write_text(json.dumps(dict(generator='Source/tools/recoil/export_missing_recoil.py', settings=rc.SETTINGS, window=a.window,
                                      gain=a.gain, suffix=a.suffix, reference=a.reference,
                                      excluded=EXCLUDED if a.families == 'all' else None, clipGain=None if a.gain is not None else
                                      {f'{k[0]}/{k[1]}': v for k, v in CLIP_GAIN.items()},
                                      core='Source/tools/analysis/export_nue_native_recoil_curves.py weapon_curve + sampler_text',
                                      profiles=result), indent=2) + '\n', encoding='utf-8')
    print('report:', report)


if __name__ == '__main__':
    main()
