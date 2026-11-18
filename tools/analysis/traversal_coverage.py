"""Traversal coverage from game logs: aggregates the "UTPP trav:" plan lines into acceptance per action and obstacle kind, rejection
reasons, chosen montages, and lists the action / kind pairs that no log exercised (with what a test scenario would need).

  python -I traversal_coverage.py [--out coverage.json] [root ...]

Roots default to the project's Research/logs/game, Research/logs/night and Research/logs/gametest. Every *.log / *.txt below them is read;
a plan line seen in several files (a session log and the test-run copy of the same line) counts once: the key is the time of day plus the
plan fields before the montage name (copies may cut the line short). Paths in the output are relative to the root they were found under.
"""
import argparse
import collections
import concurrent.futures
import json
import os
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

MARK = b'UTPP trav:'
ACTIONS = {8: 'climb', 9: 'vault'}
KINDS = {0: 'static', 1: 'vehicle', 2: 'entity'}
TIME_RE = re.compile(r'(\d\d:\d\d:\d\d\.\d{3})')
PLAN_RE = re.compile(r'UTPP trav: action=(\d+) (.*?)kind=(\d+) decision=(\S+) result=(\w+)\(([^)]*)\)(?: montage=(\S*))?')
FIELD_RE = re.compile(r'(\w+)=(-?[\d.]+)')
RELEASE_RE = re.compile(r'UTPP trav: released to game animation \(detail=(\d+)\)')
# what a scenario needs for an action / kind pair that no log contains (game rules: climb / vault tweaks, motion decision table)
NEEDS = {
    ('climb', 'static'): 'a static wall or ledge 0.6-2.75 m above the feet, approached within 30 deg of its normal, 0.45-1.0 m away',
    ('climb', 'vehicle'): 'a parked car or van roof / bonnet 1.0-2.0 m high, V standing next to it facing the body',
    ('climb', 'entity'): 'a movable prop (crate, container) 0.6-2.0 m high that the game reports as an entity',
    ('vault', 'static'): ('a static railing, fence or low wall 0.5-1.25 m high and under 0.59 m deep, floor behind within 0.40 m of the '
                          'start level, approached at a sprint (>= 3.5 m/s) straight at it; the motion Hurdle row needs >= 0.5 m drop '
                          'behind the edge, 0.1-0.5 m is the low-hurdle route'),
    ('vault', 'vehicle'): 'a motorcycle on level ground approached at a sprint (the game vaults bikes, not cars); goes to the game animation',
    ('vault', 'entity'): 'a movable prop 0.5-1.25 m high, under 0.59 m deep, approached at a sprint',
}


def scan_file(path):
    """(plans, releases): plans = list of (key, record); releases = list of (key, detail)."""
    try:
        data = path.read_bytes()
    except OSError:
        return [], []
    if MARK not in data:
        return [], []
    plans, releases = [], []
    for raw in data.splitlines():
        if MARK not in raw:
            continue
        line = raw.decode('utf-8', 'replace')
        tm = TIME_RE.search(line)
        t = tm.group(1) if tm else ''
        m = PLAN_RE.search(line)
        if m:
            fields = {k: float(v) for k, v in FIELD_RE.findall(m.group(2))}
            rec = {'action': int(m.group(1)), 'kind': int(m.group(3)), 'decision': m.group(4), 'result': m.group(5),
                   'reason': m.group(6), 'montage': m.group(7) or '', 'time': t}
            rec.update(fields)
            key = (t, m.group(1), m.group(2).strip(), m.group(3), m.group(4), m.group(5), m.group(6))
            plans.append((key, rec))
            continue
        r = RELEASE_RE.search(line)
        if r:
            releases.append(((t, 'release', r.group(1)), int(r.group(1))))
    return plans, releases


def default_roots():
    try:
        import utpp_paths
        base = pathlib.Path(utpp_paths.project_root()) / 'Research' / 'logs'
    except Exception:
        base = HERE.parents[2] / 'Research' / 'logs'
    return [base / 'game', base / 'night', base / 'gametest']


def collect(roots, jobs=16):
    files = []
    for root in roots:
        root = pathlib.Path(root)
        if root.is_file():
            files.append((root.parent, root))
            continue
        for dirpath, _dirs, names in os.walk(root):
            for n in names:
                if n.endswith(('.log', '.txt')):
                    files.append((root, pathlib.Path(dirpath) / n))
    plans, releases = {}, {}
    sources = collections.Counter()
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
        for (root, path), (p, r) in zip(files, ex.map(lambda f: scan_file(f[1]), files)):
            rel = path.relative_to(root).as_posix() if path.is_relative_to(root) else path.name
            for key, rec in p:
                old = plans.get(key)
                if old is None or (not old['montage'] and rec['montage']) or len(rec['montage']) > len(old['montage']):
                    rec = dict(rec, file=rel)
                    plans[key] = rec
                sources[rel] += 1
            for key, d in r:
                releases[key] = d
    return list(plans.values()), list(releases.values()), sources


def summarize(plans, releases):
    by = collections.defaultdict(lambda: {'plans': 0, 'ok': 0, 'results': collections.Counter(), 'montages': collections.Counter()})
    for p in plans:
        k = '%s/%s' % (ACTIONS.get(p['action'], str(p['action'])), KINDS.get(p['kind'], str(p['kind'])))
        b = by[k]
        b['plans'] += 1
        b['ok'] += p['result'] == 'Ok'
        b['results']['%s(%s)' % (p['result'], p['reason'])] += 1
        if p['result'] == 'Ok' and p['montage']:
            b['montages'][p['montage']] += 1
    out = {'plans': len(plans), 'releases': dict(collections.Counter(releases)), 'by_action_kind': {}, 'never_exercised': []}
    for k in sorted(by):
        b = by[k]
        out['by_action_kind'][k] = {'plans': b['plans'], 'ok': b['ok'], 'acceptance': round(b['ok'] / b['plans'], 3),
                                    'results': dict(b['results'].most_common()), 'montages': dict(b['montages'].most_common())}
    for a in ACTIONS.values():
        for kd in KINDS.values():
            k = '%s/%s' % (a, kd)
            if k not in by:
                out['never_exercised'].append({'pair': k, 'needs': NEEDS[(a, kd)]})
    feet = collections.Counter('L' if p['montage'].endswith('Lfoot') else 'R' if p['montage'].endswith('Rfoot') else '-'
                               for p in plans if p['result'] == 'Ok' and p['montage'])
    out['ok_montage_foot'] = dict(feet)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('roots', nargs='*', help='log folders or files (default: the project game / night / gametest logs)')
    ap.add_argument('--out', help='write the summary as JSON')
    ap.add_argument('--jobs', type=int, default=16)
    a = ap.parse_args(argv)
    roots = [pathlib.Path(r) for r in a.roots] if a.roots else default_roots()
    plans, releases, sources = collect([r for r in roots if r.exists()], a.jobs)
    s = summarize(plans, releases)
    s['files_with_plans'] = len(sources)
    print('traversal plans: %d (unique), files with plan lines: %d, releases to the game animation: %s'
          % (s['plans'], len(sources), s['releases']))
    for k, b in s['by_action_kind'].items():
        print('  %-15s %3d plans, Ok %3d (%.0f %%): %s' % (k, b['plans'], b['ok'], 100 * b['acceptance'],
                                                          ', '.join('%s %d' % kv for kv in b['results'].items())))
    print('  Ok montage foot: %s' % s['ok_montage_foot'])
    for n in s['never_exercised']:
        print('  never exercised: %s -> needs %s' % (n['pair'], n['needs']))
    if a.out:
        with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(s, f, indent=1, sort_keys=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
