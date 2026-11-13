"""Print the first differences between two WolvenKit JSON files after handle ids are canonicalised by first appearance
(same canonical form as ragdoll\\json_same.py). Diagnostic only. usage: python -I json_first_diff.py a.json b.json [max]"""
import json
import sys


def canon(doc):
    ids = {}

    def rec(o):
        if isinstance(o, dict):
            r = {}
            for k, v in o.items():
                if k in ('HandleId', 'HandleRefId'):
                    r[k] = ids.setdefault(v, len(ids))
                else:
                    r[k] = rec(v)
            return r
        if isinstance(o, list):
            return [rec(v) for v in o]
        return o
    return rec(doc['Data'])


out = []
limit = int(sys.argv[3]) if len(sys.argv) > 3 else 20


def cmp(a, b, path):
    if len(out) >= limit:
        return
    if type(a) is not type(b):
        out.append('%s: type %s != %s (%s | %s)' % (path, type(a).__name__, type(b).__name__, str(a)[:120], str(b)[:120]))
        return
    if isinstance(a, dict):
        for k in list(dict.fromkeys(list(a) + list(b))):
            if k not in a or k not in b:
                out.append('%s.%s: only in %s (%s)' % (path, k, 'a' if k in a else 'b', str(a.get(k, b.get(k)))[:160]))
            else:
                cmp(a[k], b[k], path + '.' + k)
    elif isinstance(a, list):
        if len(a) != len(b):
            out.append('%s: len %d != %d' % (path, len(a), len(b)))
        for i, (x, y) in enumerate(zip(a, b)):
            cmp(x, y, '%s[%d]' % (path, i))
    elif a != b:
        out.append('%s: %r != %r' % (path, a, b))


A, B = (canon(json.load(open(p, encoding='utf-8'))) for p in sys.argv[1:3])
cmp(A, B, 'Data')
print('\n'.join(out) if out else 'identical')
