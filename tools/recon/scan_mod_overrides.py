"""Find which installed .archive files contain given depot paths (read-only scan).

Runs `WolvenKit.CLI archive -l -r <regex>` per archive, so one unreadable archive does not stop the scan.
Usage: python scan_mod_overrides.py <paths.txt> <out.json> <archive_dir> [<archive_dir> ...]
"""
import json
import os
import re
import subprocess
import sys

WK = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'Tools', 'WolvenKit.CLI', 'WolvenKit.CLI.exe')
paths = [l.strip() for l in open(sys.argv[1], encoding='utf-8') if l.strip()]
wanted = {p.lower() for p in paths}
regex = '^(' + '|'.join(re.escape(p) for p in paths) + ')$'
result = {'paths': paths, 'archives': {}, 'errors': {}}
archives = []
for d in sys.argv[3:]:
    for root, _, files in os.walk(d):
        archives += [os.path.join(root, f) for f in files if f.lower().endswith('.archive')]
for i, a in enumerate(sorted(archives)):
    try:
        out = subprocess.run([WK, 'archive', '-l', '-r', regex, a], capture_output=True, text=True,
                             encoding='utf-8', errors='replace', timeout=300).stdout
    except subprocess.TimeoutExpired:
        result['errors'][a] = 'timeout'
        continue
    hits = sorted({l.strip() for l in out.splitlines() if l.strip().lower() in wanted})
    if 'Exception' in out:
        result['errors'][a] = out.strip().splitlines()[-1][:300]
    if hits:
        result['archives'][a] = hits
    if i % 50 == 0:
        print(f'{i}/{len(archives)}', flush=True)
json.dump(result, open(sys.argv[2], 'w', encoding='utf-8'), indent=2)
by_path = {}
for a, hs in result['archives'].items():
    for h in hs:
        by_path.setdefault(h, []).append(os.path.basename(a))
print(f'scanned {len(archives)} archives, {len(result["archives"])} with hits, {len(result["errors"])} errors')
for p in sorted(by_path):
    print(f'{len(by_path[p])}  {p}: {", ".join(by_path[p])}')
