"""Deterministic JSON patch: bind magazine decal meshes to the magazine visibility track their sibling mesh uses.

  python -I patch_magazine_visibility.py patch  <targets.json> <vanilla-json-root> <out-json-root> <report.json>
  python -I patch_magazine_visibility.py verify <targets.json> <vanilla-json-root> <roundtrip-json-root> <report.json>

Roots mirror depot paths: <root>\\<depot path>.json (WolvenKit `convert serialize` output).

patch: for every component id listed in targets.json, every serialized copy of that entSkinnedMeshComponent (an
.ent's components + compiledData chunks; every appearance's components + compiledData chunks of an .app, i.e. the
weapon root .app the player weapon is built from and the magazine part .app) gets visibilityAnimationParam
"None" -> the pinned parameter. Nothing else is written. Aborts unless: the value before is "None"; in an .app the
component is skinned to an animated component of the same appearance whose animParameters contain the pinned
parameter, and another mesh skinned to that component (the magazine mesh) already uses it; in an .ent every bound
sibling uses the pinned parameter and, with its own animated component, the parameter is one of its animParameters;
the per-file hit count equals the pinned count.

verify: flatten Data of vanilla vs round-tripped (patched JSON -> binary -> JSON) and require that the ONLY differing
leaves are .../visibilityAnimationParam/$value going "None" -> pinned parameter, exactly expectedHits per file.
"""
import json
import os
import sys

SKINNED = 'entSkinnedMeshComponent'
FIELD = 'visibilityAnimationParam'


def cname(v):
    return v.get('$value') if isinstance(v, dict) else v


def load(path):
    with open(path, encoding='utf-8-sig') as f:
        return json.load(f)


def stem(path):
    return os.path.splitext(os.path.basename(path or ''))[0]


def containers(rc):
    """(appearance, component list, animated-component params of that appearance)."""
    if rc.get('$type') == 'appearanceAppearanceResource':
        for ap in rc.get('appearances') or []:
            a = ap.get('Data', ap)
            comps = a.get('components') or []
            chunks = ((a.get('compiledData') or {}).get('Data') or {}).get('Chunks') or []
            params = anim_params(comps)
            params.update(anim_params(chunks))
            yield cname(a.get('name')), comps, params
            yield cname(a.get('name')), chunks, params
    else:
        yield '', rc.get('components') or [], {}
        yield '', ((rc.get('compiledData') or {}).get('Data') or {}).get('Chunks') or [], {}


def anim_params(comps):
    return {cname(c.get('name')): [cname(p.get('parameterName')) for p in c.get('animParameters') or []]
            for c in comps if c.get('$type') == 'entAnimatedComponent'}


def handles(o, out=None):
    """HandleId -> Data; the components list of a cooked .app refers to the compiledData bindings by HandleRefId."""
    if out is None:
        out = {}
    if isinstance(o, dict):
        if 'HandleId' in o and 'Data' in o:
            out[o['HandleId']] = o['Data']
        for v in o.values():
            handles(v, out)
    elif isinstance(o, list):
        for v in o:
            handles(v, out)
    return out


def bind_name(h, comp):
    ref = comp.get('skinning')
    if not isinstance(ref, dict):
        return None
    data = ref.get('Data') if 'Data' in ref else h.get(ref.get('HandleRefId'))
    name = cname((data or {}).get('bindName'))
    return None if name in (None, 'None') else name


def fail(msg):
    raise SystemExit('PATCH ABORT: ' + msg)


def patch_file(spec, doc):
    rc = doc['Data']['RootChunk']
    is_app = rc.get('$type') == 'appearanceAppearanceResource'
    want = {c['id']: c for c in spec['components']}
    hits = []
    h = handles(rc)
    for appearance, comps, aparams in containers(rc):
        skinned = [c for c in comps if c.get('$type') == SKINNED and FIELD in c]
        for c in skinned:
            t = want.get(c.get('id'))
            if t is None:
                continue
            before = cname(c[FIELD])
            if before != 'None':
                fail('%s %s: %s is %r, expected "None"' % (spec['path'], c.get('id'), FIELD, before))
            if stem((c.get('mesh') or {}).get('DepotPath', {}).get('$value')) != stem(t['mesh']):
                fail('%s %s: mesh differs from pinned %s' % (spec['path'], c.get('id'), t['mesh']))
            sib = [s for s in skinned if s is not c and cname(s[FIELD]) not in ('None', None)]
            if is_app:
                # The decal must be skinned to an animated component of this appearance that carries the pinned
                # parameter, and a sibling skinned to the same component (the magazine mesh) must already use it.
                b = bind_name(h, c)
                if b is None or t['param'] not in (aparams.get(b) or []):
                    fail('%s %s [%s]: skinned to %s with params %s, pinned %s' % (
                        spec['path'], c.get('id'), appearance, b, aparams.get(b), t['param']))
                if not [s for s in sib if bind_name(h, s) == b and cname(s[FIELD]) == t['param']]:
                    fail('%s %s [%s]: no sibling skinned to %s uses %s' % (spec['path'], c.get('id'), appearance, b, t['param']))
            else:
                for s in sib:
                    if cname(s[FIELD]) != t['param']:
                        fail('%s %s: sibling in same .ent uses %s, pinned %s' % (spec['path'], c.get('id'), cname(s[FIELD]), t['param']))
                own = [cname(p.get('parameterName')) for a in comps if a.get('$type') == 'entAnimatedComponent'
                       for p in a.get('animParameters') or []]
                if own and t['param'] not in own:
                    fail('%s: %s not among the .ent animParameters %s' % (spec['path'], t['param'], own))
            c[FIELD]['$value'] = t['param']
            hits.append({'appearance': appearance, 'id': c.get('id'), 'param': t['param']})
    for cid, t in want.items():
        n = sum(1 for h in hits if h['id'] == cid)
        if n != t['hits']:
            fail('%s %s: %d hits, pinned %d' % (spec['path'], cid, n, t['hits']))
    if len(hits) != spec['expectedHits']:
        fail('%s: %d hits, pinned %d' % (spec['path'], len(hits), spec['expectedHits']))
    return hits


def flat(o, p='', out=None):
    if out is None:
        out = {}
    if isinstance(o, dict):
        for k, v in o.items():
            flat(v, p + '/' + k, out)
    elif isinstance(o, list):
        for i, v in enumerate(o):
            flat(v, p + '/' + str(i), out)
    else:
        out[p] = o
    return out


def main():
    mode, targets, src_root, dst_root, report = sys.argv[1:6]
    spec = load(targets)
    rows = []
    for f in spec['files']:
        src = os.path.join(src_root, f['path'] + '.json')
        if mode == 'patch':
            doc = load(src)
            hits = patch_file(f, doc)
            dst = os.path.join(dst_root, f['path'] + '.json')
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with open(dst, 'w', encoding='utf-8', newline='\n') as fh:
                json.dump(doc, fh, indent=2, ensure_ascii=False)
            rows.append({'path': f['path'], 'hits': len(hits), 'detail': hits})
        elif mode == 'verify':
            a = flat(load(src)['Data'])
            b = flat(load(os.path.join(dst_root, f['path'] + '.json'))['Data'])
            params = {c['param'] for c in f['components']}
            diff = sorted(k for k in set(a) | set(b) if a.get(k, '<missing>') != b.get(k, '<missing>'))
            bad = [k for k in diff if not k.endswith('/' + FIELD + '/$value') or a.get(k) != 'None' or b.get(k) not in params]
            if bad or len(diff) != f['expectedHits']:
                raise SystemExit('VERIFY FAIL %s: %d differing leaves (pinned %d), unexpected: %s' % (
                    f['path'], len(diff), f['expectedHits'], bad[:5]))
            rows.append({'path': f['path'], 'changedLeaves': len(diff)})
        else:
            raise SystemExit('mode must be patch or verify')
    total = sum(r.get('hits', r.get('changedLeaves', 0)) for r in rows)
    with open(report, 'w', encoding='utf-8', newline='\n') as fh:
        json.dump({'mode': mode, 'files': len(rows), 'total': total, 'rows': rows}, fh, indent=1)
    print('%s OK files=%d changed=%d' % (mode.upper(), len(rows), total))


if __name__ == '__main__':
    main()
