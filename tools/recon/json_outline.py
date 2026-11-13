"""Print a compact outline of a WolvenKit CR2W JSON file (keys, types, list lengths) to a given depth.

Usage: python json_outline.py <file.json> [max_depth] [start.key.path]
"""
import json
import sys

data = json.load(open(sys.argv[1], encoding='utf-8'))
max_depth = int(sys.argv[2]) if len(sys.argv) > 2 else 4
node = data
if len(sys.argv) > 3:
    for k in sys.argv[3].split('.'):
        node = node[int(k)] if isinstance(node, list) else node[k]


def show(n, depth, indent, label):
    pad = '  ' * indent
    if isinstance(n, dict):
        t = n.get('$type', '')
        print(f'{pad}{label}: dict{(" " + t) if t else ""} ({len(n)} keys)')
        if depth < max_depth:
            for k, v in n.items():
                if k != '$type':
                    show(v, depth + 1, indent + 1, k)
    elif isinstance(n, list):
        print(f'{pad}{label}: list[{len(n)}]')
        if depth < max_depth and n:
            show(n[0], depth + 1, indent + 1, '[0]')
    else:
        s = repr(n)
        print(f'{pad}{label}: {s[:80]}')


show(node, 0, 0, 'root')
