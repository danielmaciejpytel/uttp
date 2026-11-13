"""Compare archive listings (WolvenKit `archive -l` output) with an extracted folder.

Usage: python compare_paths.py <requested_paths.txt> <extracted_dir>
Prints requested paths that are missing in the extracted folder and extra files found there.
"""
import os
import sys


def norm(p):
    return p.strip().replace('/', '\\').lower()


requested = {norm(l) for l in open(sys.argv[1], encoding='utf-8', errors='replace') if l.strip()}
root = sys.argv[2]
found = set()
for d, _, files in os.walk(root):
    for f in files:
        found.add(norm(os.path.relpath(os.path.join(d, f), root)))
print('requested', len(requested), 'found', len(found))
for p in sorted(requested - found):
    print('MISSING', p)
for p in sorted(found - requested):
    print('EXTRA', p)
