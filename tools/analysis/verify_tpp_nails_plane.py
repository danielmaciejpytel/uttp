"""Verify nails plane archive: json(vanilla) vs json(unbundled+serialized candidate).
Only /renderingPlaneAnimationParam/$value of target nail components may differ (plus Header noise).
usage: verify_tpp_nails_plane.py <vanilla_json_dir> <cand_json_dir> [--female-only]"""
import json, sys, re, collections
sys.path.insert(0, __import__('os').path.dirname(__file__))
from json_semdiff import walk
from patch_tpp_nails_plane import FEMALE, MALE, RESOURCES


def get(d, path):
    for tok in re.findall(r'/([^/\[\]]+)|\[(\d+)\]', path):
        d = d[tok[0]] if tok[0] else d[int(tok[1])]
    return d


def main():
    van, cand = sys.argv[1], sys.argv[2]
    target = FEMALE if '--female-only' in sys.argv else FEMALE | MALE
    ok = True
    for name in RESOURCES:
        a = json.load(open('%s/%s.json' % (van, name), encoding='utf-8'))
        b = json.load(open('%s/%s.json' % (cand, name), encoding='utf-8'))
        out = []; walk(a, b, '', out)
        stat = collections.Counter(); bad = []
        for p, x, y in out:
            if p.startswith('/Header/'): continue
            if p.endswith('/renderingPlaneAnimationParam/$value') and y == 'None' and x in ('renderPlane', 'renderPlaneLeftArm'):
                comp = get(a, p.rsplit('/renderingPlaneAnimationParam', 1)[0])
                n = comp['name']['$value']
                if comp['$type'] == 'entMorphTargetSkinnedMeshComponent' and n in target:
                    stat[(n, x)] += 1; continue
            bad.append((p, x, y))
        # completeness: every target component in vanilla must have been changed
        total = collections.Counter()
        def cnt(x):
            if isinstance(x, dict):
                if x.get('$type') == 'entMorphTargetSkinnedMeshComponent' and x['name']['$value'] in FEMALE | MALE:
                    total[x['name']['$value']] += 1
                for y in x.values(): cnt(y)
            elif isinstance(x, list):
                for y in x: cnt(y)
        cnt(a)
        print(name, 'intended', sum(stat.values()), 'unexpected', len(bad), dict(stat))
        print('  vanilla component counts', dict(total))
        for t in bad[:10]: print('  BAD', t)
        ok = ok and not bad and all(total[t] == sum(v for (n, _), v in stat.items() if n == t) for t in target)
    print('RESULT', 'PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if __name__ == '__main__': main()
