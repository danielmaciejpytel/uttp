"""Dump animgraph nodes of given types (WolvenKit JSON export), shallow: nested node handles are shown
as '<Type#HandleId debugName>' instead of being expanded. Read-only analysis helper.

Usage: python -I graph_node_dump.py <graph.animgraph.json> <NodeType>[,<NodeType>...] [max_per_type]
NodeType without the 'animAnimNode_' prefix, e.g. BlendByMaskDynamic,AddSnapToTerrainIkRequest
"""
import json
import sys


def name_of(v):
    if isinstance(v, dict) and v.get('$type') == 'CName':
        return v.get('$value')
    return None


def short(data):
    t = data.get('$type', '?').replace('animAnimNode_', '')
    dn = name_of(data.get('debugName'))
    extra = ''
    for k in ('name', 'variableName', 'animFeatureName', 'animation', 'stateName', 'boneName', 'slotName'):
        nv = name_of(data.get(k))
        if nv and nv != 'None':
            extra += f' {k}={nv}'
    return f'<{t}{(" " + dn) if dn and dn != "None" else ""}{extra}>'


def shallow(v, depth=0):
    if isinstance(v, dict):
        if 'HandleId' in v and 'Data' in v:
            return f"#{v['HandleId']}{short(v['Data'])}"
        if 'HandleRefId' in v:
            return f"ref#{v['HandleRefId']}"
        if v.get('$type') == 'CName':
            return v.get('$value')
        if depth > 6:
            return '{...}'
        return {k: shallow(x, depth + 1) for k, x in v.items() if k != '$type' or True}
    if isinstance(v, list):
        if len(v) > 40:
            return [shallow(x, depth + 1) for x in v[:40]] + [f'... {len(v) - 40} more']
        return [shallow(x, depth + 1) for x in v]
    return v


def main():
    path, types = sys.argv[1], ['animAnimNode_' + t for t in sys.argv[2].split(',')]
    limit = int(sys.argv[3]) if len(sys.argv) > 3 else 50
    root = json.load(open(path, encoding='utf-8'))['Data']['RootChunk']
    seen = {t: 0 for t in types}

    def walk(n):
        if isinstance(n, dict):
            if 'HandleId' in n and isinstance(n.get('Data'), dict):
                d = n['Data']
                t = d.get('$type')
                if t in seen and seen[t] < limit:
                    seen[t] += 1
                    print(f"=== {t} #{n['HandleId']}")
                    body = {k: shallow(x) for k, x in d.items() if k != '$type'}
                    print(json.dumps(body, indent=1, ensure_ascii=False))
            for x in n.values():
                walk(x)
        elif isinstance(n, list):
            for x in n:
                walk(x)

    walk(root)
    print('counts:', seen)


if __name__ == '__main__':
    main()
