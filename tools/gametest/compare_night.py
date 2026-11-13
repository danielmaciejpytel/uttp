"""Compare harness step metrics between two night cycles (read-only).

  python Source\tools\gametest\compare_night.py <cycleA> <cycleB> [--metric footlock_corr_max_cm,offset_saturated_s]

cycleX = folder under Research\logs\night (or a full path). Prints per (scenario, step, metric) the values of A and B,
the delta, and a summary (how many steps improved / worsened / unchanged, counts above the thresholds).
"""
import glob
import os
import re
import sys
import collections

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
NIGHT = os.path.join(ROOT, 'Research', 'logs', 'night')
STEP_RE = re.compile(r'=== step (\S+)\s+(\S+ \S+) -> (\S+)\s+(PASS|FAIL|SKIP)\s*(?:\((.*)\))?')
KV_RE = re.compile(r'(\w+)=([-\d.]+)')


def metrics_of(cycle_dir):
    out = {}
    for f in sorted(glob.glob(os.path.join(cycle_dir, 'gametest', '*_r*.txt'))):
        sc = os.path.basename(f).rsplit('_r', 1)[0]
        step = None
        for ln in open(f, encoding='utf-8', errors='replace'):
            m = STEP_RE.match(ln)
            if m:
                step = m.group(1)
                continue
            if step and ln.startswith('    ') and 'duration_s=' in ln:
                for k, v in KV_RE.findall(ln):
                    try:
                        out[(sc, step, k)] = float(v)
                    except ValueError:
                        pass
                step = None
    return out


def resolve(name):
    return name if os.path.isdir(name) else os.path.join(NIGHT, name)


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    a, b = resolve(argv[1]), resolve(argv[2])
    wanted = None
    if '--metric' in argv:
        wanted = set(argv[argv.index('--metric') + 1].split(','))
    ma, mb = metrics_of(a), metrics_of(b)
    keys = sorted(k for k in set(ma) | set(mb) if (wanted is None or k[2] in wanted))
    thresholds = {'footlock_corr_max_cm': 15.0, 'offset_saturated_s': 0.5}
    better = worse = same = 0
    over = collections.Counter()
    print(f'{"scenario":26} {"step":16} {"metric":24} {"A":>8} {"B":>8} {"delta":>8}')
    for k in keys:
        va, vb = ma.get(k), mb.get(k)
        if va is None or vb is None:
            continue
        d = vb - va
        tag = ''
        thr = thresholds.get(k[2])
        if thr is not None:
            if va > thr:
                over['A ' + k[2]] += 1
            if vb > thr:
                over['B ' + k[2]] += 1
        if abs(d) < 1e-6:
            same += 1
        elif d < 0:
            better += 1
            tag = ' better'
        else:
            worse += 1
            tag = ' worse'
        print(f'{k[0]:26} {k[1]:16} {k[2]:24} {va:8.2f} {vb:8.2f} {d:+8.2f}{tag}')
    print(f'\nlower-is-better deltas: better={better} worse={worse} same={same}')
    for key, n in sorted(over.items()):
        print(f'  steps over threshold {key}: {n}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
