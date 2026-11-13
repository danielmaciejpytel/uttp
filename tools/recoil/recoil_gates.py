"""Step 4 of recoil-missing-20261109: sanity gates of the final new recoil profiles (woman + man), male parity through the real test
exe, identity of the shipped 0.1.0 profiles.

Gates per file (the loader / RecoilProfilesTest bounds, re-checked in Python on the exact bytes):
  format     UTPP_RECOIL 1, 97 samples, 0.05 s <= duration <= 1.5 s, t from 0 to duration, increasing, CRLF
  unit       every quaternion unit length (|1 - |q|| <= 1e-4)
  endpoints  identity at t = 0 and t = duration (recovery to zero): <= 1e-6 m, <= 1e-4 rad
  budget     peak <= 8 cm / 0.50 rad per shot (the shipped budget; the loader limit is 0.25 m / 0.70 rad)
  opening    the first sample interval carries <= 25% of the peak (RecoilProfilesTest OpensSmoothly)
  steps      (information) one shot replayed at 30, 60 and 144 fps without the driver admission, frame steps; no spikes:
             the worst deviation of a sample from the mean of its neighbours, relative to the peak (WARN above 0.10)
  fidelity   for t >= start ramp the file equals the raw source curve (bind reading, fade) scaled by the recorded amplitudeScale
  rate       duration against the weapon cycle time (CycleTimeModifier of the 2.5 tweak sources): shots overlap when > 1 (information;
             bursts are covered by RecoilProfilesTest)
Parity: Build\\UTPP\\utpp_authored_recoil_test.exe <fixture> <stage\\recoil> <stage\\recoil_male> on staged copies (Data\\recoil +
final woman, Data\\recoil_male + final man): 15 mm / 0.02 rad curve, 8 mm / 0.02 rad peak, same duration.
Identity: Data\\recoil *.utpprecoil against the make_release.py RECOIL pins; no new name collides with a shipped name.
Read only on Data\\, the release tool and the plugin; writes only below the work folder.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recoil_common as rc  # noqa: E402
import export_missing_recoil as em  # noqa: E402

WORK = rc.ROOT / 'Build/work/recoil-missing-20261109'
PARITY_EXE = rc.ROOT / 'Build/UTPP/utpp_authored_recoil_test.exe'
BUDGET_T, BUDGET_R, STEP_T, STEP_R = 0.08, 0.50, 0.03, math.radians(5.0)


def turn(q):
    return 2 * math.atan2(math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2), abs(q[3]))


def file_gates(data: bytes, ramp: float):
    g, notes = {}, []
    duration, rows = rc.parse_sampler(data)
    g['format'] = (data.startswith(b'UTPP_RECOIL 1\r\n') and b'\n' not in data.replace(b'\r\n', b'') and len(rows) == 97
                   and 0.05 <= duration <= 1.5 and abs(rows[0][0]) < 1e-9 and abs(rows[-1][0] - duration) < 1e-6
                   and all(rows[k + 1][0] > rows[k][0] for k in range(len(rows) - 1)))
    g['unit'] = max(abs(1 - math.sqrt(sum(c * c for c in r[1:5]))) for r in rows) <= 1e-4
    end = [(math.hypot(*r[5:8]), turn(r[1:5])) for r in (rows[0], rows[-1])]
    g['endpoints'] = all(t <= 1e-6 and a <= 1e-4 for t, a in end)
    tr = [math.hypot(*r[5:8]) for r in rows]
    an = [turn(r[1:5]) for r in rows]
    peak_t, peak_r = max(tr), max(an)
    g['budget'] = peak_t <= BUDGET_T + 1e-6 and peak_r <= BUDGET_R + 1e-6
    q1, t1 = rc.sample(rows, duration, duration / 96.0)
    g['opening'] = math.hypot(*t1) <= .25 * peak_t + 1e-4 and turn(q1) <= .25 * peak_r + 1e-3
    worst = [0.0, 0.0]
    for fps in (30.0, 60.0, 144.0):
        prev = ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0))
        for f in range(1, int(duration * fps) + 3):
            cur = rc.sample(rows, duration, f / fps)
            worst[0] = max(worst[0], math.dist(cur[1], prev[1]))
            worst[1] = max(worst[1], rc.angle_between(prev[0], cur[0]))
            prev = cur
    if worst[0] > STEP_T or worst[1] > STEP_R:  # information: no driver admission here; the burst gate is RecoilProfilesTest
        notes.append('raw frame step above 3 cm / 5 deg without the driver admission')
    spike = 0.0
    for k in range(1, len(rows) - 1):
        mid_t = [(rows[k - 1][5 + c] + rows[k + 1][5 + c]) / 2 for c in range(3)]
        spike = max(spike, math.dist(rows[k][5:8], mid_t) / peak_t if peak_t > 1e-6 else 0.0)
    if spike > 0.10:
        notes.append(f'spike {spike:.2f} of the peak')
    fade_start = duration - min(rc.SETTINGS['fade'], duration * 0.5)
    q_f, t_f = rc.sample(rows, duration, fade_start)
    fade_load = max(math.hypot(*t_f) / peak_t if peak_t > 1e-9 else 0.0, turn(q_f) / peak_r if peak_r > 1e-9 else 0.0)
    return g, notes, dict(duration=duration, peakCm=peak_t * 100, peakRad=peak_r, frameStepCm=worst[0] * 100,
                          frameStepDeg=math.degrees(worst[1]), spike=spike, fadeLoad=fade_load,
                          firstIntervalShareT=math.hypot(*t1) / peak_t if peak_t > 1e-9 else 0.0)


def fidelity(glb: Path, clip: str, rows_file, scale: float, ramp: float):
    _, raw, _ = rc.export_clip(glb, clip, em.RAW)
    scaled = rc.core.scale_rows(raw, scale)
    dt = dr = 0.0
    for a, b in zip(rows_file, scaled):
        if a[0] < ramp - 1e-9:
            continue
        dt = max(dt, math.dist(a[5:8], b[5:8]))
        dr = max(dr, rc.angle_between(a[1:5], b[1:5]))
    return dt, dr


def cycle_times():
    inv = json.loads((WORK / 'weapon-inventory-25.json').read_text(encoding='utf-8'))
    out = {}
    for x in inv['families']:
        name = x['record'].split('Base_', 1)[-1].lower()
        out[name] = (x.get('cycleTime') or {}).get('CycleTimeModifier')
    return out


def stage(out_dir: Path, work: Path):
    pairs = (('recoil', rc.ROOT / 'Data/recoil', out_dir / 'woman'), ('recoil_male', rc.ROOT / 'Data/recoil_male', out_dir / 'man'))
    for name, shipped, new in pairs:
        dst = work / 'stage' / name
        if dst.exists():
            shutil.rmtree(dst)
        dst.mkdir(parents=True)
        for src in list(shipped.glob('*.utpprecoil')) + list(new.glob('*.utpprecoil')):
            if (dst / src.name).exists():
                raise SystemExit(f'name collision while staging {name}: {src.name}')
            shutil.copyfile(src, dst / src.name)
    return work / 'stage' / 'recoil', work / 'stage' / 'recoil_male'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=WORK / 'out', help='final set (export-report.json, woman\\, man\\)')
    ap.add_argument('--work', type=Path, default=WORK)
    a = ap.parse_args()
    out_dir, work = a.out.resolve(), a.work.resolve()
    report = json.loads((out_dir / 'export-report.json').read_text(encoding='utf-8'))
    cycles = cycle_times()
    ramp = rc.SETTINGS['start_ramp']
    rows_out, fails = [], []
    for p in report['profiles']:
        glb = rc.ROOT / p['source']
        for c in p['clips']:
            path = out_dir / p['sex'] / f"{p['family']}_single_shot_{c['kind']}.utpprecoil"
            data = path.read_bytes()
            g, notes, m = file_gates(data, ramp)
            _, frows = rc.parse_sampler(data)
            fid_t, fid_r = fidelity(glb, c['clip'], frows, c['amplitudeScale'], ramp)
            g['fidelity'] = fid_t <= 1e-5 and fid_r <= 1e-4
            ct = cycles.get(p['family'])
            row = dict(family=p['family'], sex=p['sex'], kind=c['kind'], clip=c['clip'], file=str(path.relative_to(rc.ROOT)),
                       size=len(data), sha256=rc.sha256_bytes(data), amplitudeScale=c['amplitudeScale'],
                       familyBudgetFactor=c.get('familyBudgetFactor'), gates=g, notes=notes, metrics=m,
                       fidelityMm=fid_t * 1000, fidelityRad=fid_r, cycleTime=ct,
                       durationOverCycle=(m['duration'] / ct) if ct else None)
            rows_out.append(row)
            fails += [f"{p['family']}/{p['sex']}/{c['kind']}: {k}" for k, ok in g.items() if not ok]
    # identity of the shipped profiles + name collisions
    pins = rc.release_recoil_pins()
    shipped = {f.name: f for f in (rc.ROOT / 'Data/recoil').glob('*.utpprecoil')}
    identity = {n: (shipped[n].stat().st_size == s and rc.sha256_file(shipped[n]) == h) if n in shipped else False
                for n, (s, h) in pins.items()}
    collisions = sorted(set(f.name for f in (out_dir / 'woman').glob('*.utpprecoil')) & set(pins))
    # plan bytes (reach runs A/B) against the final bytes
    plan = json.loads((WORK / 'out-rejected/export-report-plan-every.json').read_text(encoding='utf-8'))
    plan_sha = {Path(f['path']).name + '|' + p['sex']: f['sha256'] for p in plan['profiles'] for f in p['files']}
    same_as_plan = {r['file']: plan_sha.get(Path(r['file']).name + '|' + r['sex']) == r['sha256'] for r in rows_out}
    # male parity through the real exe on staged copies
    stage_w, stage_m = stage(out_dir, work)
    exe = work / 'bin' / PARITY_EXE.name
    exe.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(PARITY_EXE, exe)
    fixture = work / 'fixtures' / 'parity-final'
    proc = subprocess.run([str(exe), str(fixture), str(stage_w), str(stage_m)], cwd=str(exe.parent), capture_output=True, text=True,
                          timeout=900)
    log = work / 'parity-final.log'
    log.write_text(proc.stdout + '\n--- stderr ---\n' + proc.stderr, encoding='utf-8')
    parity_lines = [ln for ln in proc.stdout.splitlines() if 'parity' in ln.lower() and ('FAIL' in ln or 'male parity' in ln)]
    parity_fail = [ln for ln in proc.stdout.splitlines() if ln.startswith('FAIL')]
    result = dict(generator='Source/tools/recoil/recoil_gates.py', out=str(out_dir.relative_to(rc.ROOT)), files=rows_out,
                  gateFailures=fails, shippedIdentity=identity, shippedIdentityAll=all(identity.values()) and len(identity) == 32,
                  nameCollisions=collisions, sameBytesAsPlanExport=same_as_plan,
                  parity=dict(exe=str(PARITY_EXE.relative_to(rc.ROOT)), exeSha256=rc.sha256_file(PARITY_EXE), exitCode=proc.returncode,
                              stageWoman=str(stage_w.relative_to(rc.ROOT)), stageMan=str(stage_m.relative_to(rc.ROOT)),
                              summary=parity_lines[-3:], failures=parity_fail, log=str(log.relative_to(rc.ROOT))))
    (work / 'gates.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(f"{'family':<11}{'sex':<6}{'kind':<10}{'dur':>6}{'cm':>6}{'rad':>6}{'xS':>6}{'stepCm':>7}{'stepDeg':>8}{'spike':>6}{'fade':>6}"
          f"{'fidMm':>8}{'cyc':>7}{'d/c':>6}  gates")
    for r in rows_out:
        m = r['metrics']
        print(f"{r['family']:<11}{r['sex']:<6}{r['kind']:<10}{m['duration']:6.3f}{m['peakCm']:6.2f}{m['peakRad']:6.3f}"
              f"{r['amplitudeScale']:6.2f}{m['frameStepCm']:7.2f}{m['frameStepDeg']:8.2f}{m['spike']:6.2f}{m['fadeLoad']:6.2f}"
              f"{r['fidelityMm']:8.4f}{(r['cycleTime'] or 0):7.3f}{(r['durationOverCycle'] or 0):6.2f}  "
              f"{'ok' if all(r['gates'].values()) else 'FAIL ' + ','.join(k for k, v in r['gates'].items() if not v)}"
              f"{' ' + ';'.join(r['notes']) if r['notes'] else ''}")
    print('gate failures:', fails or 'none')
    print('shipped identity (32 pins):', result['shippedIdentityAll'], '| name collisions:', collisions or 'none')
    print('changed vs plan export:', [k for k, v in same_as_plan.items() if not v])
    print('parity exit', proc.returncode, '|', *result['parity']['summary'], sep='\n  ')
    for ln in parity_fail[:40]:
        print('  ', ln[:240])
    print('json:', work / 'gates.json')


if __name__ == '__main__':
    main()
