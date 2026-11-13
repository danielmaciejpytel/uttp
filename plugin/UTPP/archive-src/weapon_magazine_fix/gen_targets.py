"""Generate targets.json (the pinned patch list) from the scan results. Run only after a deliberate review.

Usage: python -I gen_targets.py <candidates.json> <extract-root> <tracks.json> <targets.json>

Sequence after a game patch: unbundle + serialize every firearm .app/.ent (see scan_mag_visibility.py), run
scan_mag_visibility.py -> candidates.json, export the weapon .anims/.rig and run check_show_tracks.py -> tracks.json,
review EXCLUDE below, then run this script and re-pin the targets.json SHA256 in build_weapon_magazine_fix.ps1.
Inputs of the 2026-10-28 v3 pin: <extract-root> = Build\\stages\\weapon-decal-fix\\scan2\\extract (every firearm .app/.ent of
content\\ + ep1\\), candidates = Build\\stages\\weapon-decal-fix\\scan2\\candidates.json, tracks =
Build\\stages\\weapon-decal-fix\\scan3\\tracks.json from check_show_tracks.py over Build\\stages\\weapon-decal-fix\\scan\\anim_raw +
anim_glb (base\\animations\\weapon\\ from basegame_4_animation/basegame_4_gamedata and ep1\\animations\\weapon\\ *__mag_std
sets + rigs from ep1_2_gamedata; ep1 archives hold no re-cooked base\\animations\\weapon sets/rigs on 2.5), made with
  WolvenKit.CLI unbundle <game>\\archive\\pc\\ep1\\ep1_2_gamedata.archive -o scan\\anim_raw\\ep1_2_gamedata
      -r '^ep1\\\\(animations\\\\weapon\\\\firearms\\\\.*__mag_std\\.anims|weapons\\\\firearms\\\\.*\\\\rig\\\\.*__mag_std\\.rig)$'
  WolvenKit.CLI convert serialize scan\\anim_raw\\ep1_2_gamedata -r '\\.(anims|rig)$'
  WolvenKit.CLI export scan\\anim_raw\\ep1_2_gamedata -o scan\\anim_glb\\ep1_2_gamedata -gp <game>   (rig/json "Failed" lines are expected)
After
re-pinning targets/patcher, also re-pin the payload SHA256 printed by a review build.

Rules: a candidate (weapon root .app or magazine part .app) is included only when its parameter hides the magazine in
at least one player weapon clip (tracks.json); the part .ent that defines the same component id is included only when
that component uses the same mesh; a (file, component) pair listed by several candidates is pinned once and must agree
on the parameter. Scanner "unresolved" entries (no single magazine parameter / no bound sibling, e.g. the side-less
Palica decal) are recorded as excluded.

Several copies of one depot path: identical copies are used as is. When the content\\ and ep1\\ copies differ, the ep1
copy is the winner ONLY when it is a strict superset of the content copy (every content root field, appearance field
and component is present and identical in the ep1 copy; resolvedDependencies and AppearanceVisualController
appearanceDependency entries compared as multisets because ep1 inserts entries): the Phantom Liberty archive group
carries these base\\ resources only to extend them with ep1\\ meshes (NUSA / spy / krausser ... appearances), so it
must override content when Phantom Liberty is installed, and an override derived from the superset keeps every
component of the content copy. Anything else is excluded (winning copy not proven). Every copy (archive + SHA256) and
the winner are pinned in targets.json; the build re-checks all of them.
"""
import hashlib
import json
import os
import sys
from collections import Counter

EXCLUDE = {}
MULTISET_LISTS = ('resolvedDependencies', 'appearanceDependency')
ARCH = {'content_basegame_4_gamedata': 'archive\\pc\\content\\basegame_4_gamedata.archive',
        'content_basegame_3_nightcity': 'archive\\pc\\content\\basegame_3_nightcity.archive',
        'ep1_ep1_1_nightcity': 'archive\\pc\\ep1\\ep1_1_nightcity.archive',
        'ep1_ep1_2_gamedata': 'archive\\pc\\ep1\\ep1_2_gamedata.archive'}


def sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest().upper()


def cname(v):
    return v.get('$value') if isinstance(v, dict) else v


def strip(o):
    """Drop WolvenKit handle numbering (it shifts when ep1 inserts components)."""
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k not in ('HandleId', 'HandleRefId')}
    if isinstance(o, list):
        return [strip(v) for v in o]
    return o


def js(o):
    return json.dumps(strip(o), sort_keys=True)


def item_within(x, y):
    """x (content) is contained in y (ep1): identical, except that the multiset lists may gain entries in y."""
    if js(x) == js(y):
        return True
    if not (isinstance(x, dict) and isinstance(y, dict)) or set(x) - set(y):
        return False
    for k, v in x.items():
        if k in MULTISET_LISTS and isinstance(v, list) and isinstance(y.get(k), list):
            if Counter(js(e) for e in v) - Counter(js(e) for e in y[k]):
                return False
        elif js(v) != js(y.get(k)):
            return False
    return True


def comps_within(la, lb):
    pool = {}
    for y in lb:
        pool.setdefault((y.get('id'), cname(y.get('name'))), []).append(y)
    return all(any(item_within(x, y) for y in pool.get((x.get('id'), cname(x.get('name'))), [])) for x in la)


def chunks(a):
    return ((a.get('compiledData') or {}).get('Data') or {}).get('Chunks') or []


def superset(c, e):
    """True when RootChunk e (ep1 copy) strictly contains RootChunk c (content copy); see the module docstring."""
    for k in set(c) | set(e):
        if k not in ('appearances', 'components', 'compiledData') and js(c.get(k)) != js(e.get(k)):
            return False
    if not comps_within(c.get('components') or [], e.get('components') or []) or not comps_within(chunks(c), chunks(e)):
        return False
    ea = {cname(a.get('Data', a).get('name')): a.get('Data', a) for a in e.get('appearances') or []}
    for a in c.get('appearances') or []:
        a = a.get('Data', a)
        b = ea.get(cname(a.get('name')))
        if b is None:
            return False
        for k in set(a) | set(b):
            if k not in ('components', 'compiledData') and not item_within({k: a.get(k)}, {k: b.get(k)}):
                return False
        if not comps_within(a.get('components') or [], b.get('components') or []) or not comps_within(chunks(a), chunks(b)):
            return False
    return True


def main():
    scan = json.load(open(sys.argv[1], encoding='utf-8'))
    cand = scan['candidates']
    extract, tracks_path, out = sys.argv[2], sys.argv[3], sys.argv[4]
    tracks = json.load(open(tracks_path, encoding='utf-8'))
    tops = sorted(os.listdir(extract))

    def copies(depot):
        found = [t for t in tops if os.path.isfile(os.path.join(extract, t, depot))]
        if not found:
            raise SystemExit('GEN ABORT: %s not extracted' % depot)
        return found

    resolved = {}

    def winner(depot):
        """{'archive', 'sha256', 'copies', 'winnerReason'} of the copy the game loads, or None when not proven."""
        if depot in resolved:
            return resolved[depot]
        found = copies(depot)
        cp = [{'archive': ARCH[t], 'sha256': sha(os.path.join(extract, t, depot))} for t in found]
        res = None
        if len({x['sha256'] for x in cp}) == 1:
            res = {'archive': cp[0]['archive'], 'sha256': cp[0]['sha256'], 'copies': cp, 'winnerReason': 'all copies identical'}
        else:
            con = [t for t in found if t.startswith('content_')]
            ep1 = [t for t in found if t.startswith('ep1_')]
            if len(con) == 1 and len(ep1) == 1 and len(found) == 2:
                rc = [json.load(open(os.path.join(extract, t, depot + '.json'), encoding='utf-8-sig'))['Data']['RootChunk'] for t in (con[0], ep1[0])]
                if superset(rc[0], rc[1]):
                    w = [x for x in cp if x['archive'] == ARCH[ep1[0]]][0]
                    res = {'archive': w['archive'], 'sha256': w['sha256'], 'copies': cp,
                           'winnerReason': 'content and ep1 copies differ; the ep1 copy (Phantom Liberty group, overrides content) '
                                           'is a strict superset of the content copy'}
        resolved[depot] = res
        return res

    def hides(c, param):
        for r in tracks:
            if r['app'] == c['app'] and r['mesh'] == c['mesh']:
                return any(param in d and any(v < 0.5 for _, v in d[param])
                           for s in r['sets'] for d in (s.get('clips') or {}).values())
        raise SystemExit('GEN ABORT: no track result for %s %s' % (c['app'], c['mesh']))

    files, excluded = {}, []

    def add(depot, cid, param, hits, mesh):
        w = winner(depot)
        f = files.setdefault(depot, {'path': depot, 'archive': w['archive'], 'sha256': w['sha256'], 'copies': w['copies'],
                                     'winnerReason': w['winnerReason'], 'components': []})
        same = [x for x in f['components'] if x['id'] == cid]
        if same:
            if same[0]['param'] != param or same[0]['mesh'] != mesh or same[0]['hits'] != hits:
                raise SystemExit('GEN ABORT: %s %s pinned twice with different data' % (depot, cid))
            return
        f['components'].append({'id': cid, 'mesh': mesh, 'param': param, 'hits': hits})

    def differ_reason(depot):
        return 'copies differ between %s and the ep1 copy is not a strict superset; winning copy not proven' % ', '.join(copies(depot))

    for c in cand:
        if c['componentId'] in EXCLUDE:
            excluded.append({'app': c['app'], 'componentId': c['componentId'], 'mesh': c['mesh'], 'reason': EXCLUDE[c['componentId']]})
            continue
        if len(c['siblingParams']) != 1:
            raise SystemExit('GEN ABORT: %s %s has parameters %s' % (c['app'], c['mesh'], c['siblingParams']))
        param = c['siblingParams'][0]
        if not hides(c, param):
            excluded.append({'app': c['app'], 'componentId': c['componentId'], 'mesh': c['mesh'],
                             'reason': '%s never hides the magazine in an evaluated player weapon clip (or the anim set was not evaluated)' % param})
            continue
        if winner(c['app']) is None:
            excluded.append({'app': c['app'], 'componentId': c['componentId'], 'mesh': c['mesh'], 'reason': differ_reason(c['app'])})
            continue
        add(c['app'], c['componentId'], param, c['containers'], c['mesh'])
        for e in c['partEnts']:
            w = winner(e)
            if w is None:
                excluded.append({'ent': e, 'componentId': c['componentId'], 'mesh': c['mesh'], 'reason': differ_reason(e)})
                continue
            top = [t for t in copies(e) if ARCH[t] == w['archive']][0]
            rc = json.load(open(os.path.join(extract, top, e + '.json'), encoding='utf-8'))['Data']['RootChunk']
            meshes = [x['mesh']['DepotPath']['$value'] for x in rc['components'] if x.get('id') == c['componentId']]
            if meshes != [c['mesh']]:
                if not any(x.get('ent') == e and x.get('componentId') == c['componentId'] for x in excluded):
                    excluded.append({'ent': e, 'componentId': c['componentId'], 'mesh': meshes,
                                     'reason': 'same component id as the .app component, but a different mesh that no live 2.5 appearance uses'})
                continue
            add(e, c['componentId'], param, 2, c['mesh'])
    seen = set()
    for u in scan.get('unresolved') or []:
        key = (u['app'], u['componentId'], u['mesh'])
        if key in seen:
            continue
        seen.add(key)
        excluded.append({'app': u['app'], 'componentId': u['componentId'], 'mesh': u['mesh'],
                         'reason': 'unresolved: bound to %s with parameters %s and no single magazine parameter used by a sibling' % (u['bind'], u['params'])})
    for f in files.values():
        f['components'].sort(key=lambda x: x['id'])
        f['expectedHits'] = sum(x['hits'] for x in f['components'])
    res = {'gameProductVersion': '2.5', 'field': 'visibilityAnimationParam', 'from': 'None',
           'files': [files[k] for k in sorted(files)], 'excluded': excluded}
    json.dump(res, open(out, 'w', encoding='utf-8', newline='\n'), indent=1)
    print(len(files), 'files', sum(f['expectedHits'] for f in files.values()), 'hits', len(excluded), 'excluded')


if __name__ == '__main__':
    main()
