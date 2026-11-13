"""List redscript annotations (@wrapMethod/@replaceMethod/@addMethod/@addField/...) with target class and member.

Usage: python reds_hooks.py <dir_with_reds>
"""
import os
import re
import sys

ANN = re.compile(r'^\s*@(wrapMethod|replaceMethod|addMethod|addField|replaceGlobal|wrapConstructor)\(([^)]*)\)')
MEMBER = re.compile(r'\b(?:func|let|cb\s+func)\s+([A-Za-z0-9_]+)')
rows = []
for root, _, files in os.walk(sys.argv[1]):
    for f in sorted(files):
        if not f.endswith('.reds'):
            continue
        lines = open(os.path.join(root, f), encoding='utf-8', errors='replace').read().splitlines()
        for i, l in enumerate(lines):
            m = ANN.match(l)
            if not m:
                continue
            member = '?'
            for nxt in lines[i + 1:i + 4]:
                mm = MEMBER.search(nxt)
                if mm:
                    member = mm.group(1)
                    break
            rows.append((f, i + 1, m.group(1), m.group(2), member))
print(f'{len(rows)} annotations')
for kind in sorted({r[2] for r in rows}):
    print(f'  {kind}: {sum(1 for r in rows if r[2] == kind)}')
for f, line, kind, target, member in rows:
    print(f'{f}:{line}\t@{kind}({target}) {member}')
