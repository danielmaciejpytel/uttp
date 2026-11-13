"""Summary diff of two WolvenKit JSON exports of an animgraph (or any CR2W JSON).

Compares counts of every $type and the sets of CName / ResourcePath string values.
Usage: python graph_summary_diff.py <a.json> <b.json>
"""
import collections
import json
import sys


def walk(node, types, names, paths):
    if isinstance(node, dict):
        t = node.get('$type')
        if t:
            types[t] += 1
        if t == 'CName' and isinstance(node.get('$value'), str):
            names[node['$value']] += 1
        if t == 'ResourcePath' and isinstance(node.get('$value'), str):
            paths[node['$value']] += 1
        for v in node.values():
            walk(v, types, names, paths)
    elif isinstance(node, list):
        for v in node:
            walk(v, types, names, paths)


def summarize(p):
    types, names, paths = collections.Counter(), collections.Counter(), collections.Counter()
    walk(json.load(open(p, encoding='utf-8'))['Data']['RootChunk'], types, names, paths)
    return types, names, paths


(ta, na, pa), (tb, nb, pb) = summarize(sys.argv[1]), summarize(sys.argv[2])
print('== $type count changes (A -> B)')
for t in sorted(set(ta) | set(tb)):
    if ta[t] != tb[t]:
        print(f'  {t}: {ta[t]} -> {tb[t]} ({tb[t] - ta[t]:+d})')
print('== CNames only in B (count)')
for n in sorted(set(nb) - set(na)):
    print(f'  {n} ({nb[n]})')
print('== CNames only in A (count)')
for n in sorted(set(na) - set(nb)):
    print(f'  {n} ({na[n]})')
print('== ResourcePaths only in B')
for n in sorted(set(pb) - set(pa)):
    print(f'  {n}')
print('== ResourcePaths only in A')
for n in sorted(set(pa) - set(pb)):
    print(f'  {n}')
