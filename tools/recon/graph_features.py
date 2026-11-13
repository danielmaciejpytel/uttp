"""List anim features and variables declared by an animgraph JSON export (WolvenKit).

Usage: python graph_features.py <graph.animgraph.json>
"""
import json
import sys


def walk(n, out):
    if isinstance(n, dict):
        t = n.get('$type')
        if t == 'animAnimFeatureEntry':
            name = (n.get('name') or {}).get('$value')
            ftype = (n.get('className') or {})
            ftype = ftype.get('$value') if isinstance(ftype, dict) else ftype
            out['features'].add((name, ftype))
        elif t in ('animAnimVariableFloat', 'animAnimVariableBool', 'animAnimVariableInt', 'animAnimVariableVector', 'animAnimVariableQuaternion', 'animAnimVariableTransform'):
            out['variables'].add((t.replace('animAnimVariable', ''), (n.get('name') or {}).get('$value')))
        for v in n.values():
            walk(v, out)
    elif isinstance(n, list):
        for v in n:
            walk(v, out)


out = {'features': set(), 'variables': set()}
walk(json.load(open(sys.argv[1], encoding='utf-8'))['Data']['RootChunk'], out)
print(f'features ({len(out["features"])}):')
for name, ftype in sorted(out['features'], key=lambda x: str(x[0])):
    print(f'  {name}: {ftype}')
print(f'variables ({len(out["variables"])}):')
for t, name in sorted(out['variables'], key=lambda x: (x[0], str(x[1]))):
    print(f'  {t} {name}')
