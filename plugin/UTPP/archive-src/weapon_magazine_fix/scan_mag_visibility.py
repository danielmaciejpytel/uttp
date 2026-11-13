"""Scan serialized (WolvenKit JSON) weapon .app/.ent files for magazine meshes without a visibility binding.

Usage: python -I scan_mag_visibility.py <extract-root> <report.json>

<extract-root> holds one sub-folder per game archive (WolvenKit unbundle + convert serialize output). Extract every
firearm .app AND .ent, not only the magazine part files:
  WolvenKit.CLI unbundle <archive> -o <extract-root>\\<dir>_<archive-basename> -r '^(base|ep1)\\\\weapons\\\\firearms\\\\.*\\.(app|ent)$'
  WolvenKit.CLI convert serialize <extract-root> -r '\\.(app|ent)$'
for every archive under archive\\pc\\content and archive\\pc\\ep1.

Why every .app: on 2.5 the player weapon is instantiated from the weapon ROOT appearance resource (e.g.
nokota_sidewinder\\w_rifle_assault__nokota_sidewinder.app). That cooked .app carries inline copies of the magazine
components (Magazine / MagazineReload animated components and every mesh skinned to them); the magazine part .app/.ent
files are separate resources. A fix applied only to the part files does not reach the player weapon (2026-10-28,
recording 00-20-31: the Sidewinder flag still floated with the part files patched).

Rule (per appearance; components + compiledData chunks of a cooked .app share their binding handles): an
entSkinnedMeshComponent is a CANDIDATE when
  - its visibilityAnimationParam is the literal "None",
  - its skinning binding names an entAnimatedComponent of the same appearance that has exactly one animParameter P
    (a Left/Right pair is resolved only by a _left_/_right_ token in the mesh name), and P is a magazine visibility
    track (showMagazine*),
  - at least one other entSkinnedMeshComponent with the same skinning binding already uses P.
P is the parameter to bind. Meshes skinned to animated components without a magazine parameter (receiver, barrel,
iron sights on visibleWithScope, ...) are never candidates. "unresolved" lists None meshes skinned to a magazine
component with no bound sibling or several parameters (review only). The part .ent files that define the same
component id are reported too. Read-only.
"""
import json
import os
import sys
from collections import defaultdict

SKINNED = 'entSkinnedMeshComponent'
ANIMATED = 'entAnimatedComponent'
MAG_PARAM = 'showMagazine'


def cname(v):
    return v.get('$value') if isinstance(v, dict) else v


def depot(ref):
    return cname((ref or {}).get('DepotPath'))


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)['Data']['RootChunk']


def handles(o, out=None):
    """HandleId -> Data for every handle in the document (HandleRefId entries point back here)."""
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


def animated_params(comps):
    return {cname(c.get('name')): [cname(p.get('parameterName')) for p in c.get('animParameters') or []]
            for c in comps if c.get('$type') == ANIMATED}


def pick_param(params, mesh):
    """The single parameter of the bound animated component; with a Left/Right pair (dual-magazine Palica) the side
    token in the mesh file name (_left_ / _right_) must select exactly one. None when undecidable."""
    if len(params) == 1:
        return params[0]
    name = os.path.basename(mesh or '')
    side = 'Left' if '_left_' in name else 'Right' if '_right_' in name else None
    sel = [p for p in params if side and p.endswith(side)]
    return sel[0] if len(sel) == 1 else None


def animated(comps):
    out = []
    for c in comps or []:
        if c.get('$type') == ANIMATED:
            out.append({'name': cname(c.get('name')), 'rig': depot(c.get('rig')),
                        'params': [cname(p.get('parameterName')) for p in c.get('animParameters') or []],
                        'animSets': [depot(e.get('animSet')) for e in (c.get('animations') or {}).get('gameplay') or []]})
    return out


def appearances(rc):
    """(appearance name, components, compiledData chunks) of an .app; one unnamed entry for an .ent."""
    if rc.get('$type') == 'appearanceAppearanceResource':
        for ap in rc.get('appearances') or []:
            a = ap.get('Data', ap)
            yield cname(a.get('name')), a.get('components') or [], ((a.get('compiledData') or {}).get('Data') or {}).get('Chunks') or []
    else:
        yield '', rc.get('components') or [], ((rc.get('compiledData') or {}).get('Data') or {}).get('Chunks') or []


def main():
    root, report = sys.argv[1], sys.argv[2]
    files = defaultdict(list)  # depot path -> [(archive folder, root chunk)]
    for top in sorted(os.listdir(root)):
        base = os.path.join(root, top)
        for dp, _, fs in os.walk(base):
            for f in sorted(fs):
                if f.endswith('.json'):
                    full = os.path.join(dp, f)
                    files[os.path.relpath(full, base)[:-5]].append((top, load(full)))
    # component id -> part .ent files defining a skinned mesh with that id
    ent_by_id = defaultdict(set)
    for path, copies in files.items():
        if path.endswith('.ent'):
            for _, comps, chunks in appearances(copies[-1][1]):
                for c in comps + chunks:
                    if c.get('$type') == SKINNED:
                        ent_by_id[c.get('id')].add(path)
    hits, unresolved = {}, set()
    n_app = n_appearance = 0
    for path in sorted(files):
        if not path.endswith('.app'):
            continue
        n_app += 1
        rc = files[path][-1][1]
        h = handles(rc)
        for appearance, comps, chunks in appearances(rc):
            n_appearance += 1
            params = animated_params(comps)
            params.update(animated_params(chunks))
            for label, lst in (('components', comps), ('compiledData', chunks)):
                sk = [(c, bind_name(h, c)) for c in lst if c.get('$type') == SKINNED and 'visibilityAnimationParam' in c]
                for c, b in sk:
                    if cname(c['visibilityAnimationParam']) != 'None' or b is None or \
                            not any(p.startswith(MAG_PARAM) for p in params.get(b) or []):
                        continue
                    ps = params[b]
                    p = pick_param(ps, depot(c.get('mesh')))
                    sib = [s for s, sb in sk if s is not c and sb == b and cname(s['visibilityAnimationParam']) == p]
                    if p is None or not p.startswith(MAG_PARAM) or not sib:
                        unresolved.add((path, appearance, c.get('id'), depot(c.get('mesh')), b, tuple(ps)))
                        continue
                    ps = [p]
                    key = (path, c.get('id'), depot(c.get('mesh')) or '', b)
                    r = hits.setdefault(key, {'app': path, 'archives': [t for t, _ in files[path]], 'componentId': c.get('id'),
                                              'component': cname(c.get('name')), 'mesh': depot(c.get('mesh')), 'bind': b,
                                              'siblingParams': set(), 'appearances': set(), 'containers': 0,
                                              'partEnts': sorted(ent_by_id.get(c.get('id'), ())),
                                              'animated': animated(comps) or animated(chunks)})
                    r['siblingParams'].add(ps[0])
                    r['appearances'].add(appearance)
                    r['containers'] += 1
    rows = []
    for key in sorted(hits):
        r = hits[key]
        r['siblingParams'] = sorted(r['siblingParams'])
        r['appearances'] = sorted(r['appearances'])
        rows.append(r)
        print('%-95s %-60s bind=%s params=%s appearances=%s containers=%d ents=%d' % (
            r['app'], os.path.basename(r['mesh'] or ''), r['bind'], ','.join(r['siblingParams']), ','.join(r['appearances']),
            r['containers'], len(r['partEnts'])))
    un = [dict(zip(('app', 'appearance', 'componentId', 'mesh', 'bind', 'params'), u)) for u in sorted(unresolved)]
    for u in un:
        print('UNRESOLVED %s [%s] %s bind=%s params=%s' % (u['app'], u['appearance'], os.path.basename(u['mesh'] or ''), u['bind'], list(u['params'])))
    print('apps', n_app, 'appearances', n_appearance, 'candidates', len(rows), 'unresolved', len(un))
    with open(report, 'w', encoding='utf-8', newline='\n') as f:
        json.dump({'apps': n_app, 'appearances': n_appearance, 'candidates': rows, 'unresolved': un}, f, indent=1, sort_keys=True)


if __name__ == '__main__':
    main()
