"""Semantic diff of two WolvenKit JSON exports of an .ent (entEntityTemplate).

Components are matched by (name, $type); HandleId/HandleRefId numbering is ignored.
Usage: python ent_diff.py <a.ent.json> <b.ent.json> [--max N]
"""
import json
import sys

IGNORED_KEYS = {'HandleId', 'HandleRefId', 'Header', 'BufferId'}


def load(p):
    return json.load(open(p, encoding='utf-8'))['Data']['RootChunk']


def cname(v):
    if isinstance(v, dict) and '$value' in v:
        return v['$value']
    return v


def flatten(node, prefix=''):
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            if k in IGNORED_KEYS:
                continue
            out.update(flatten(v, f'{prefix}.{k}' if prefix else k))
        if not node:
            out[prefix] = '{}'
    elif isinstance(node, list):
        for i, v in enumerate(node):
            out.update(flatten(v, f'{prefix}[{i}]'))
        if not node:
            out[prefix] = '[]'
    else:
        out[prefix] = node
    return out


def comp_key(c):
    return (str(cname(c.get('name'))), c.get('$type'))


def diff_flat(a, b, limit):
    fa, fb = flatten(a), flatten(b)
    lines = []
    for k in sorted(set(fa) | set(fb)):
        va, vb = fa.get(k, '<absent>'), fb.get(k, '<absent>')
        if va != vb:
            lines.append(f'      {k}: {str(va)[:120]!s}  ->  {str(vb)[:120]!s}')
    return lines[:limit], len(lines)


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    limit = int(sys.argv[sys.argv.index('--max') + 1]) if '--max' in sys.argv else 40
    ca = {comp_key(c): c for c in a['components']}
    cb = {comp_key(c): c for c in b['components']}
    print(f'components: A={len(ca)} B={len(cb)}')
    for k in sorted(set(ca) - set(cb)):
        print(f'  REMOVED in B: {k[1]} "{k[0]}"')
    for k in sorted(set(cb) - set(ca)):
        print(f'  ADDED in B:   {k[1]} "{k[0]}"')
    for k in sorted(set(ca) & set(cb)):
        lines, total = diff_flat(ca[k], cb[k], limit)
        if total:
            print(f'  CHANGED: {k[1]} "{k[0]}" ({total} fields)')
            print('\n'.join(lines))
            if total > limit:
                print(f'      ... {total - limit} more')
    da = {str(cname(d.get('DepotPath', d))) for d in a.get('resolvedDependencies', [])}
    db = {str(cname(d.get('DepotPath', d))) for d in b.get('resolvedDependencies', [])}
    print(f'resolvedDependencies: A={len(da)} B={len(db)}')
    for d in sorted(da - db):
        print('  - ', d)
    for d in sorted(db - da):
        print('  + ', d)
    for key in ('appearances', 'includes', 'defaultAppearance', 'compiledEntityLODFlags'):
        if a.get(key) != b.get(key):
            print(f'top-level {key} differs')
    ent_lines, ent_total = diff_flat(a['entity'], b['entity'], limit)
    print(f'entity (PlayerPuppet) differing fields: {ent_total}')
    if ent_total:
        print('\n'.join(ent_lines))


if __name__ == '__main__':
    main()
