"""Re-solve the door verdict (check_door_clearance.py --solve --fragment, the CURRENT tools: body / roof phases, start gate, joint rule, waiver audit) of rows whose clips are unchanged
(round 4 startfwd: the verdict directory next to the pack, so every shipped row has a verdict with the body / waiver keys and a start_gate record).

  python -I resolve_rows.py --sources <sources-final.txt> --out <dir> [--rows a,b] [--jobs 6] [--body-waivers <json>]

Per row the settings are those the clip was built / verified with: the old verdict's open / opening / body / roof tolerances (the documented waivers of build_car.ps1; none when the old
verdict has none), --rear-ref = the game's clip of the set, --tol-cm 1.0. Prints one line per row (exit code of --solve: 0 = feasible, door gates and start gate pass).

Waivers (r10): the body waivers are the PER-CAPSULE ones of build_car.ps1 bodyWaive, checked in as body_waivers.json next to this file (the file the shipped verdicts of startfwd2 / r7util were solved with;
--body-waivers PATH replaces it; rows without an entry get none). Door waivers: sport2 w:passenger_r 6.0 (build_car.ps1 doorWaive). RESOLVE_NO_BODY_WAIVER / RESOLVE_NO_DOOR_WAIVER (raw verdicts) are kept;
the old RESOLVE_BODY_WAIVERS environment variable and the flat numbers that were hard-coded here are gone.
"""
import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BODY_WAIVERS = os.path.join(HERE, 'body_waivers.json')
EX = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car')


def job(a):
    n, run, out, body_waivers = a
    import re
    g, cls, seat = re.match(r'^utpp_car_p([wm])a_([a-z0-9]+)_(driver|passenger_r)$', n).groups()
    clip = os.path.join(run, 'roundtrip', n + '.anims.glb')
    van = os.path.join(EX, 'glb', n.replace('utpp_car_', '') + '.anims.glb')
    old = {}
    for cand in (os.path.join(run, 'door-close-%s.json' % n),):
        if os.path.exists(cand):
            old = json.load(open(cand, encoding='utf-8'))
    cmd = [sys.executable, '-I', os.path.join(HERE, 'check_door_clearance.py'), '--clip', clip, '--class', cls, '--seat', seat, '--tol-cm', '1.0', '--rear-ref', van, '--solve',
           '--fragment', os.path.join(out, 'door-close-%s.json' % n)]
    # the documented waivers of the CURRENT build_car.ps1 table (override the old verdict's numbers): doorWaive sport2 w:passenger_r 6.0; per-CAPSULE bodyWaive = body_waivers.json (checked in)
    door_w = {'sport2:w:passenger_r': 6.0}
    body_w = json.load(open(body_waivers, encoding='utf-8'))
    key = '%s:%s:%s' % (cls, g, seat)
    if os.environ.get('RESOLVE_NO_BODY_WAIVER'):
        body_w = {}
    if os.environ.get('RESOLVE_NO_DOOR_WAIVER'):
        door_w = {}
    old = dict(old, open_tol_cm=door_w.get(key, 2.0), opening_tol_cm=door_w.get(key, 1.0), body_tol_cm=body_w.get(key), roof_tol_cm=None)
    if old.get('open_tol_cm') is not None and float(old['open_tol_cm']) != 2.0:
        cmd += ['--open-tol-cm', str(old['open_tol_cm'])]
    if old.get('opening_tol_cm') is not None and float(old['opening_tol_cm']) != 1.0:
        cmd += ['--opening-tol-cm', str(old['opening_tol_cm'])]
    if old.get('body_tol_cm') is not None:
        bt = old['body_tol_cm']
        cmd += ['--body-tol-cm', ','.join('%s=%s' % kv for kv in bt.items()) if isinstance(bt, dict) else str(bt)]
    if old.get('roof_tol_cm') is not None:
        cmd += ['--roof-tol-cm', str(old['roof_tol_cm'])]
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    open(os.path.join(out, 'solve-%s.txt' % n), 'w', encoding='utf-8').write(r.stdout + r.stderr)
    last = [ln for ln in r.stdout.splitlines() if ln.startswith('door_close_solve at')]
    return n, r.returncode, (last[-1][:230] if last else (r.stdout + r.stderr)[-300:]), ' '.join(cmd[7:]).split('--rear-ref')[0][:80]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--rows', default='')
    ap.add_argument('--jobs', type=int, default=6)
    ap.add_argument('--body-waivers', default=DEFAULT_BODY_WAIVERS, help='per-capsule body waivers by row key (default: body_waivers.json next to this file)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    jobs = []
    for ln in open(a.sources, encoding='utf-8').read().splitlines():
        if ln.strip():
            n, d = ln.split('  ', 1)
            if a.rows and not any(r in n for r in a.rows.split(',')):
                continue
            jobs.append((n.strip(), d.strip(), a.out, os.path.abspath(a.body_waivers)))
    with ThreadPoolExecutor(a.jobs) as ex:
        for n, rc, last, opts in ex.map(job, jobs):
            print('%-38s rc=%d %s' % (n.replace('utpp_car_', ''), rc, last), flush=True)


if __name__ == '__main__':
    main()
