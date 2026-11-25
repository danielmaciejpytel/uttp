"""Round-trip check of a built stage: unpack the archive, serialize the cr2w files back to JSON and compare
the key fields with the JSON that was generated.  Also lists the stage contents.

    python -I validate_playground_stage.py [--stage-name utpp_playground_spike]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from utpp_paths import project_root  # noqa: E402

WORK_DIR = 'playground-20261124'


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
    print('exit=%d  %s' % (p.returncode, ' '.join(os.path.basename(c) for c in cmd[:3])))
    if p.returncode != 0:
        raise SystemExit((p.stdout + p.stderr)[-800:])
    return p.stdout


def close(a, b, tol=1e-5):
    return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(a)))


def fnv1a64(text):
    h = 0xcbf29ce484222325
    for c in text.encode():
        h ^= c
        h = (h * 0x100000001b3) & 0xffffffffffffffff
    return h


def compare(label, expected, actual, problems, path=''):
    if (isinstance(expected, dict) and expected.get('$type') == 'CName' and isinstance(actual, dict)
            and expected.get('$storage') == 'string' and actual.get('$storage') == 'uint64'):
        # a CName written as text is stored as its hash
        if str(fnv1a64(expected['$value'])) != str(actual['$value']):
            problems.append('%s: CName %r -> %r' % (path, expected['$value'], actual['$value']))
        return
    if isinstance(expected, dict):
        for k, v in expected.items():
            if k in ('ArchiveFileName', 'ExportedDateTime', 'Header'):
                continue
            if not isinstance(actual, dict) or k not in actual:
                problems.append('%s%s: missing in round trip' % (path, k))
                continue
            compare(label, v, actual[k], problems, path + k + '.')
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            problems.append('%s: list length %s -> %s' % (path, len(expected), len(actual) if isinstance(actual, list) else '?'))
            return
        for i, (e, a) in enumerate(zip(expected, actual)):
            compare(label, e, a, problems, '%s%d.' % (path, i))
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if not close(expected, actual):
            problems.append('%s: %r -> %r' % (path, expected, actual))
    elif expected != actual:
        problems.append('%s: %r -> %r' % (path, expected, actual))


def main():
    root = project_root()
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage-name', default='utpp_playground_spike')
    args = ap.parse_args()
    wk = os.path.join(root, 'Tools', 'WolvenKit.CLI', 'WolvenKit.CLI.exe')
    stage = os.path.join(root, 'Build', 'stages', args.stage_name)
    work = os.path.join(root, 'Build', 'work', WORK_DIR, '_tmp', args.stage_name)
    check = os.path.join(work, 'roundtrip')
    if os.path.isdir(check):
        shutil.rmtree(check)
    os.makedirs(os.path.join(check, 'unpacked'))
    os.makedirs(os.path.join(check, 'json'))

    print('stage contents:')
    for d, _dirs, files in os.walk(stage):
        for f in files:
            p = os.path.join(d, f)
            print('  %8d  %s' % (os.path.getsize(p), os.path.relpath(p, stage)))
    archive = os.path.join(stage, 'archive', 'pc', 'mod', 'utpp_playground.archive')
    print('archive listing:')
    out = run([wk, 'archive', archive, '-l'])
    print('\n'.join('  ' + l for l in out.splitlines() if 'utpp_playground' in l.lower() and '\\' in l))
    run([wk, 'unbundle', archive, '-o', os.path.join(check, 'unpacked')])
    problems = []
    for name in ('playground.streamingsector', 'playground.streamingblock'):
        cr2w = os.path.join(check, 'unpacked', 'utpp_playground', 'world', name)
        run([wk, 'convert', 'serialize', cr2w, '-o', os.path.join(check, 'json')])
        with open(os.path.join(work, 'json', name + '.json'), encoding='utf-8') as f:
            expected = json.load(f)['Data']['RootChunk']
        with open(os.path.join(check, 'json', name + '.json'), encoding='utf-8') as f:
            actual = json.load(f)['Data']['RootChunk']
        before = len(problems)
        compare(name, expected, actual, problems, name + ':')
        print('%s: %d differences' % (name, len(problems) - before))
    for p in problems[:40]:
        print('  DIFF', p)
    print('round trip: %s' % ('OK' if not problems else '%d differences' % len(problems)))
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
