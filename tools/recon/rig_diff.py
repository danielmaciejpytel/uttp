"""Compare two WolvenKit JSON exports of an animRig: bone names/parents and other fields.

Usage: python rig_diff.py <a.rig.json> <b.rig.json>
"""
import json
import sys

sys.path.insert(0, __import__('os').path.dirname(__file__))
from ent_diff import cname, flatten  # noqa: E402


def load(p):
    return json.load(open(p, encoding='utf-8'))['Data']['RootChunk']


a, b = load(sys.argv[1]), load(sys.argv[2])
ba = [str(cname(n)) for n in a.get('boneNames', [])]
bb = [str(cname(n)) for n in b.get('boneNames', [])]
print(f'bones: A={len(ba)} B={len(bb)} same_order={ba == bb}')
print('  only in A:', [x for x in ba if x not in bb])
print('  only in B:', [x for x in bb if x not in ba])
pa, pb = a.get('boneParentIndexes', []), b.get('boneParentIndexes', [])
if ba == bb and pa != pb:
    print('  parent changes:', [(ba[i], pa[i], pb[i]) for i in range(min(len(pa), len(pb))) if pa[i] != pb[i]])
fa, fb = flatten({k: v for k, v in a.items() if k not in ('boneNames',)}), flatten({k: v for k, v in b.items() if k not in ('boneNames',)})
changed = [k for k in sorted(set(fa) | set(fb)) if fa.get(k) != fb.get(k)]
groups = {}
for k in changed:
    groups.setdefault(k.split('[')[0].split('.')[0], []).append(k)
for g, ks in groups.items():
    print(f'field group {g}: {len(ks)} differing entries, e.g. {ks[0]}: {fa.get(ks[0])} -> {fb.get(ks[0])}')
print('A top-level keys:', sorted(a.keys()))
