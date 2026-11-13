"""Screen a head-duck setting on the BASELINE clip of one row (packed-plus set) without the builder: head_duck.apply on the finished clip -> candidate GLB -> row_compare.

  python -I duck_try.py ROW [--body-tol-cm X] [--no-compare] [--tag T] -- <head_duck options: --duck-chain ... --duck-az ... --duck-max-deg ...>

ROW = pwa_sport1_passenger_r (utpp_car_ prefix optional). The candidate goes to Build\\work\\vehicle-entry-car\\roof-pack2\\proto\\<ROW>[.T].glb. The builder (build_entry_clip.py --head-duck) runs the same
head_duck.apply at the end of the pipeline; this screen is on the round-tripped baseline clip (head within 1 mm of the builder's).
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import roof_table as RT  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

OUT = os.environ.get('UTPP_DUCK_OUT', os.path.join(project_root(), r'Build\work\vehicle-entry-car\roof-pack3\proto'))  # env UTPP_DUCK_OUT overrides (round 2 wrote to roof-pack2\proto)
BODY = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\body-data')
SRC = os.path.join(project_root(), r'Build\work\vehicle-entry-car\roof-pack\sources-final-B.txt')


def main():
    argv = sys.argv[1:]
    k = argv.index('--') if '--' in argv else len(argv)
    ap = argparse.ArgumentParser()
    ap.add_argument('row')
    ap.add_argument('--body-tol-cm', type=float)
    ap.add_argument('--no-compare', action='store_true')
    ap.add_argument('--tag', default='')
    ap.add_argument('--clip', help='source clip instead of the baseline round-trip')
    a = ap.parse_args(argv[:k])
    duck = argv[k + 1:]
    row = a.row if a.row.startswith('utpp_car_') else 'utpp_car_' + a.row
    g, cls, seat = RT.parse(row)
    srcs = RT.read_sources(SRC)
    clip = a.clip or os.path.join(srcs[row], 'roundtrip', row + '.anims.glb')
    os.makedirs(OUT, exist_ok=True)
    out = os.path.join(OUT, row[9:] + ('.' + a.tag if a.tag else '') + '.glb')
    cmd = [sys.executable, '-I', os.path.join(HERE, 'head_duck.py'), '--clip', clip, '--out', out, '--duck-roof-json', os.path.join(BODY, cls + '.json'),
           '--duck-roof-seat', 'left' if seat == 'driver' else 'right'] + duck
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    lines = r.stdout.splitlines()
    for ln in lines:
        if ln.startswith(('target', 'bend', 'pen ', 'repair')):
            print(ln[:900])
    if r.returncode:
        print(r.stderr[-1500:])
        return 2
    sys.stdout.flush()
    subprocess.run([sys.executable, '-I', os.path.join(HERE, 'roof_report.py'), '--class', cls, '--seat', seat, out])
    if not a.no_compare:
        cmd = [sys.executable, '-I', os.path.join(HERE, 'row_compare.py'), '--row', row, '--new', out]
        if a.body_tol_cm is not None:
            cmd += ['--body-tol-cm', str(a.body_tol_cm)]
        subprocess.run(cmd)
    return 0


if __name__ == '__main__':
    sys.exit(main())
