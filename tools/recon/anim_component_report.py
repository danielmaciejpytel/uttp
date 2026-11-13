"""Report entAnimatedComponent setup in two .ent JSON exports and diff anim sets as sets.

Usage: python anim_component_report.py <a.ent.json> <b.ent.json>
"""
import json
import sys

sys.path.insert(0, __import__('os').path.dirname(__file__))
from ent_diff import cname, flatten  # noqa: E402

SET_LISTS = ('gameplay', 'cinematic')


def load(p):
    return json.load(open(p, encoding='utf-8'))['Data']['RootChunk']


def anim_entries(comp, kind):
    out = []
    for e in (comp.get('animations') or {}).get(kind) or []:
        path = cname(e.get('animSet', {}).get('DepotPath'))
        names = tuple(cname(v) for v in e.get('variableNames') or [])
        out.append((str(path), e.get('priority'), names))
    return out


def scalar_fields(comp):
    c = dict(comp)
    anims = c.pop('animations', None)
    flat = flatten(c)
    if anims:
        for k, v in anims.items():
            if k not in SET_LISTS and k != '$type':
                flat.update(flatten(v, 'animations.' + k))
    return flat


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    pick = lambda r: {str(cname(c.get('name'))): c for c in r['components'] if c.get('$type') == 'entAnimatedComponent'}
    ca, cb = pick(a), pick(b)
    print('entAnimatedComponent A:', sorted(ca), ' B:', sorted(cb))
    for name in sorted(set(ca) | set(cb)):
        x, y = ca.get(name), cb.get(name)
        print(f'\n=== {name} ===')
        for label, c in (('A', x), ('B', y)):
            if c is None:
                print(f'  {label}: absent')
                continue
            graph = cname((c.get('graph') or {}).get('DepotPath'))
            rig = cname((c.get('rig') or {}).get('DepotPath'))
            print(f'  {label}: graph={graph} rig={rig} gameplay={len(anim_entries(c, "gameplay"))} cinematic={len(anim_entries(c, "cinematic"))}')
        if x is None or y is None:
            continue
        for kind in SET_LISTS:
            ea, eb = anim_entries(x, kind), anim_entries(y, kind)
            removed = [e for e in ea if e not in eb]
            added = [e for e in eb if e not in ea]
            if removed or added:
                print(f'  animations.{kind}: removed {len(removed)}, added {len(added)}')
                for e in removed:
                    print(f'    - prio={e[1]} vars={list(e[2])} {e[0]}')
                for e in added:
                    print(f'    + prio={e[1]} vars={list(e[2])} {e[0]}')
        fa, fb = scalar_fields(x), scalar_fields(y)
        for k in sorted(set(fa) | set(fb)):
            if fa.get(k, '<absent>') != fb.get(k, '<absent>'):
                print(f'  field {k}: {fa.get(k, "<absent>")} -> {fb.get(k, "<absent>")}')


main()
