"""Step 1 of recoil-missing-20261109: reproduce the 0.1.0 recoil profiles with the generator that made them.

For every shipped family (PROFILE_SOURCES of export_nue_native_recoil_curves.py) and the Nue:
  * 2.31 sources (the GLB recorded in Data\\recoil\\profile-export-20261021-budget.json, SHA-256 checked) -> bytes compared with
    Data\\recoil and with the RECOIL pins of make_release.py;
  * 2.5 sources (Extracted\\game\\player-anims\\glb, content/ep1 export 2026-11-09) -> bytes compared, else curve difference;
  * the same for the male pma_* GLB against Data\\recoil_male (2.31 = weapon-recoil-male-20261106, 2.5 = player-anims).
Read only except the JSON report (--report).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # python -I does not add the script folder
import recoil_common as rc  # noqa: E402

KINDS = (('shoulder', 'add_recoil_single_shot_shoulder'), ('ironsight', 'add_recoil_single_shot_ironsight'))
FIRST_KEY = dict(samples=97, fade=0.15, reference='first', limit_amplitude=False, start_ramp=0.0, budget=None)  # Nue: run()


def recorded_sources(json_path: Path):
    doc = json.loads(json_path.read_text(encoding='utf-8'))
    return {p['profile']: (p.get('source'), p.get('sourceSha256')) for p in doc['profiles']}


def compare(label, glb, clip, settings, shipped: Path, pins, rows_out):
    data, _, report = rc.export_clip(glb, clip, settings)
    ref = shipped.read_bytes() if shipped.is_file() else None
    row = dict(label=label, glb=str(glb.relative_to(rc.ROOT)), glbSha256=rc.sha256_file(glb), clip=clip,
               shipped=str(shipped.relative_to(rc.ROOT)), generatedSha256=rc.sha256_bytes(data),
               shippedSha256=rc.sha256_bytes(ref) if ref else None, byteIdentical=ref == data,
               pinSha256=pins.get(shipped.name, (None, None))[1], amplitudeScale=report['amplitudeScale'])
    row['pinIdentical'] = row['pinSha256'] == row['generatedSha256'] if row['pinSha256'] else None
    if ref is not None and ref != data:
        row['curveDiff'] = rc.curve_diff(ref, data)
    rows_out.append(row)
    diff = row.get('curveDiff')
    print(f"{label:<34} {'IDENTICAL' if row['byteIdentical'] else 'DIFFERS  '} "
          + (f"pin={'ok' if row['pinIdentical'] else 'no'} " if row['pinSha256'] else '')
          + (f"maxDiff {diff['maxDiffT'] * 1000:.3f} mm / {diff['maxDiffR']:.5f} rad, dur {diff['durationA']:.4f}/{diff['durationB']:.4f}"
             if diff else ''))
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report', type=Path, default=rc.ROOT / 'Build/work/recoil-missing-20261109/method-check.json')
    a = ap.parse_args()
    pins = rc.release_recoil_pins()
    recorded = recorded_sources(rc.ROOT / 'Data/recoil/profile-export-20261021-budget.json')
    w231 = rc.index_glbs(rc.GLB_231_WOMAN, 'pwa')
    m231 = rc.index_glbs(rc.GLB_231_MAN, 'pma')
    w25 = rc.index_glbs([rc.GLB_25], 'pwa')
    m25 = rc.index_glbs([rc.GLB_25], 'pma')
    rows, source_checks = [], []
    families = dict(rc.core.PROFILE_SOURCES)
    for fam, pwa_name in families.items():
        pma_name = 'pma_' + pwa_name[4:]
        rec_path, rec_sha = recorded.get(fam, (None, None))
        g = w231.get(pwa_name)
        actual = rc.sha256_file(g) if g else None
        source_checks.append(dict(profile=fam, recorded=rec_path, recordedSha256=rec_sha,
                                  found=str(g.relative_to(rc.ROOT)) if g else None, sha256=actual, matches=actual == rec_sha))
        print(f"source {fam:<14} 2.31 GLB sha {'matches' if actual == rec_sha else 'DIFFERS from'} the export JSON")
        for kind, clip in KINDS:
            name = f'{fam}_single_shot_{kind}.utpprecoil'
            if g:
                compare(f'{fam} {kind} woman 2.31', g, clip, None, rc.ROOT / 'Data/recoil' / name, pins, rows)
            if w25.get(pwa_name):
                compare(f'{fam} {kind} woman 2.5', w25[pwa_name], clip, None, rc.ROOT / 'Data/recoil' / name, pins, rows)
            if m231.get(pma_name):
                compare(f'{fam} {kind} man 2.31', m231[pma_name], clip, None, rc.ROOT / 'Data/recoil_male' / name, {}, rows)
            if m25.get(pma_name):
                compare(f'{fam} {kind} man 2.5', m25[pma_name], clip, None, rc.ROOT / 'Data/recoil_male' / name, {}, rows)
    # Nue: first-key reading of run() (no ramp / budget), files add_recoil_single_shot_*.utpprecoil
    nue = 'pwa_w_handgun__tsunami_nue.anims.glb'
    for kind, clip in KINDS:
        name = f'add_recoil_single_shot_{kind}.utpprecoil'
        for tag, index, data_dir, pin in (('woman 2.31', w231, 'Data/recoil', pins), ('woman 2.5', w25, 'Data/recoil', pins),
                                          ('man 2.31', m231, 'Data/recoil_male', {}), ('man 2.5', m25, 'Data/recoil_male', {})):
            key = nue if tag.startswith('woman') else 'pma_' + nue[4:]
            if index.get(key):
                compare(f'nue {kind} {tag}', index[key], clip, FIRST_KEY, rc.ROOT / data_dir / name, pin, rows)
    summary = {}
    for r in rows:
        group = r['label'].split(' ', 2)[2]
        s = summary.setdefault(group, dict(files=0, identical=0, worstT=0.0, worstR=0.0))
        s['files'] += 1
        s['identical'] += 1 if r['byteIdentical'] else 0
        if r.get('curveDiff'):
            s['worstT'] = max(s['worstT'], r['curveDiff']['maxDiffT'])
            s['worstR'] = max(s['worstR'], r['curveDiff']['maxDiffR'])
    print('\nsummary:')
    for k, v in summary.items():
        print(f"  {k:<12} {v['identical']}/{v['files']} byte-identical, worst diff {v['worstT'] * 1000:.3f} mm / {v['worstR']:.5f} rad")
    a.report.parent.mkdir(parents=True, exist_ok=True)
    a.report.write_text(json.dumps(dict(settings=rc.SETTINGS, firstKeySettingsNue=FIRST_KEY, sourceChecks=source_checks,
                                        rows=rows, summary=summary), indent=2) + '\n', encoding='utf-8')
    print('report:', a.report)


if __name__ == '__main__':
    main()
