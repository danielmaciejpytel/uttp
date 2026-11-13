"""Step 4 of recoil-missing-20261109: hand-IK reach of new recoil profiles through the EXISTING driver test (no build).

utpp_motion_driver_recoil_reach_test (Build\\UTPP, built from the current sources) runs every RecoilProfiles.hpp id 2..19 through the real
MotionDriver + reach solver: hip (shoulder clip, hipfire-<rifle|pistol1h>-pid<N>-profile) and ADS (ironsight clip,
nue-recoil-ads-<rifle|1h>-pid<N>); PASS needs shootScale >= 0.7 on every applied frame, no clamp, <= 3 cm / 5 deg per frame and owned
arms. The table is compiled in, but the curves are read from <data>\\recoil\\<family>_single_shot_*.utpprecoil. A candidate is
therefore tested by writing its curve under the file name of a table slot with the same pose class (pistol slots 2-7, 11; long-gun
slots 12-19) inside a MIRROR data folder; slots 2, 5, 11, 12, 15, 17 also run on the male bundle (-man).

Nothing of the project is modified: the exe is copied to <work>\\bin (its children and temp outputs stay there), the needed data files
are copied to <work>\\data (Data\\ is only read), runs are logged to <work>\\runs\\*.log and <work>\\reach-study.json.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import recoil_common as rc  # noqa: E402

EXE = rc.ROOT / 'Build/UTPP/utpp_motion_driver_recoil_reach_test.exe'
DATA = rc.ROOT / 'Data'
NEEDED = ['utpp_motion_woman.utppmm', 'utpp_motion_man.utppmm', 'utpp_motion_woman.chooser.json',
          'utpp_motion_man.chooser.json', 'utpp_base.utppmm', 'overlay/utpp_overlays_woman.utppov',
          'overlay/utpp_overlays_man.utppov', 'overlay/utpp_style_weights_woman.txt', 'overlay/utpp_style_weights_man.txt',
          'traversal/utpp_traversal_woman_motion.utpptrav', 'traversal/utpp_traversal_man_motion.utpptrav', 'traversal/traversal_table.json']
PISTOL_SLOTS = [(2, 'liberty'), (3, 'unity'), (4, 'yukimura'), (5, 'chao'), (6, 'lexington'), (7, 'slaughtomatic'), (11, 'quasar')]
LONG_SLOTS = [(12, 'ajax'), (13, 'copperhead'), (14, 'sidewinder'), (15, 'masamune'), (16, 'shingen'), (17, 'pulsar'),
              (18, 'saratoga'), (19, 'senkoh')]
LINE = re.compile(r'^(PASS|FAIL) (\S+?pid(\d+)[^\s:]*):?\s(.*)$')


def mirror(work: Path):
    for rel in NEEDED:
        src, dst = DATA / rel, work / 'data' / rel
        if not dst.is_file() or dst.stat().st_size != src.stat().st_size:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, dst)
    exe = work / 'bin' / EXE.name
    exe.parent.mkdir(parents=True, exist_ok=True)
    if not exe.is_file() or rc.sha256_file(exe) != rc.sha256_file(EXE):
        shutil.copyfile(EXE, exe)
    return exe


def run(work: Path, exe: Path, name: str, assign: dict, sources: dict):
    """assign: slot id -> candidate family (None = shipped curve). sources: family -> folder of its sampler files."""
    rec = work / 'data' / 'recoil'
    if rec.exists():
        shutil.rmtree(rec)
    shutil.copytree(DATA / 'recoil', rec, ignore=shutil.ignore_patterns('*.json'))
    slots = dict(PISTOL_SLOTS + LONG_SLOTS)
    for slot, fam in assign.items():
        for kind in ('shoulder', 'ironsight'):
            shutil.copyfile(sources[fam] / f'{fam}_single_shot_{kind}.utpprecoil', rec / f'{slots[slot]}_single_shot_{kind}.utpprecoil')
    log = work / 'runs' / f'{name}.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run([str(exe), str(work / 'data')], cwd=str(exe.parent), capture_output=True, text=True, timeout=1800)
    log.write_text(proc.stdout + '\n--- stderr ---\n' + proc.stderr, encoding='utf-8')
    rows = []
    for line in proc.stdout.splitlines():
        m = LINE.match(line)
        if not m:
            continue
        slot = int(m.group(3))
        fam = assign.get(slot) or slots.get(slot)
        rows.append(dict(result=m.group(1), variant=m.group(2), slot=slot, family=fam, substituted=slot in assign,
                         man=m.group(2).endswith('-man'), ads=m.group(2).startswith('nue-recoil-ads'), detail=m.group(4)[:400]))
    summary = re.search(r'recoil reach: (\d+) PASS (\d+) FAIL', proc.stdout)
    return dict(run=name, exitCode=proc.returncode, assign={str(k): v for k, v in assign.items()},
                pass_=int(summary.group(1)) if summary else None, fail=int(summary.group(2)) if summary else None, rows=rows,
                log=str(log.relative_to(rc.ROOT)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--work', type=Path, default=rc.ROOT / 'Build/work/recoil-missing-20261109/reach')
    ap.add_argument('--candidates', type=Path, default=rc.ROOT / 'Build/work/recoil-missing-20261109/out/woman',
                    help='folder of the candidate sampler files (default out\\woman)')
    ap.add_argument('--pistols', default='', help='comma separated candidate families for the pistol slots')
    ap.add_argument('--longguns', default='', help='comma separated candidate families for the long-gun slots')
    ap.add_argument('--baseline', action='store_true', help='first run the unmodified table (control)')
    ap.add_argument('--tag', default='run')
    a = ap.parse_args()
    work = a.work.resolve()
    exe = mirror(work)
    sources = {}
    pistols = [f for f in a.pistols.split(',') if f]
    longs = [f for f in a.longguns.split(',') if f]
    for f in pistols + longs:
        sources[f] = a.candidates.resolve()
    runs = []
    if a.baseline:
        runs.append(run(work, exe, a.tag + '-baseline', {}, sources))
    while pistols or longs:
        assign = {}
        for (slot, _), fam in zip(PISTOL_SLOTS, pistols[:len(PISTOL_SLOTS)]):
            assign[slot] = fam
        for (slot, _), fam in zip(LONG_SLOTS, longs[:len(LONG_SLOTS)]):
            assign[slot] = fam
        pistols, longs = pistols[len(PISTOL_SLOTS):], longs[len(LONG_SLOTS):]
        runs.append(run(work, exe, f'{a.tag}-{len(runs)}', assign, sources))
    out = work / f'reach-study-{a.tag}.json'
    out.write_text(json.dumps(dict(exe=str(EXE.relative_to(rc.ROOT)), exeSha256=rc.sha256_file(EXE), candidates=str(a.candidates),
                                   runs=runs), indent=2) + '\n', encoding='utf-8')
    for r in runs:
        print(f"== {r['run']}: exit {r['exitCode']}, {r['pass_']} PASS / {r['fail']} FAIL, log {r['log']}")
        per = {}
        for row in r['rows']:
            key = (row['family'], row['substituted'])
            per.setdefault(key, []).append(('ok ' if row['result'] == 'PASS' else 'FAIL ') + ('ads' if row['ads'] else 'hip') +
                                           ('-man' if row['man'] else ''))
        for (fam, sub), res in sorted(per.items(), key=lambda kv: (not kv[0][1], kv[0][0])):
            print(f"   {'*' if sub else ' '} {fam:<13} {' '.join(res)}")
        for row in r['rows']:
            if row['result'] == 'FAIL':
                print(f"     FAIL {row['family']}: {row['variant']} | {row['detail'][:260]}")
    print('json:', out)


if __name__ == '__main__':
    main()
