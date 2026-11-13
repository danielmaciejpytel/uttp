"""Registration evidence for the TPP melee sets (read-only; 2026-11-09).

  python -I registration_evidence.py --retail <evidence\\retail> --out <evidence dir> [--clips-dir <input clips dir> --suffix v4]

Inputs: <retail>\\raw (unbundled game files: every set the four 2.5 player entities register + the entities, extract_retail.ps1),
<retail>\\json (WolvenKit serialize, flat) and <retail>\\json-mirror (files whose base name collides, serialized one by one under their depot path).
Writes <out>\\registration.json / registration.txt:
  1 sha256 of the 12 retail melee sets vs Extracted\\game\\vanilla (the bytes the melee dumps were exported from) and of the four retail
    entities vs the 2.5 extraction of 2026-11-01 (Extracted\\game\\vehicle-entry-car\\provenance.txt);
  2 every registration of the 12 sets (entity, component, priority, vars);
  3 OVERLAP: per registration of a melee set (priority p) every other entry of the same entity (any component, incl. the UTPP .xl files in
    Source\\plugin\\UTPP\\archive) whose set holds one of the melee set's NON-ADDITIVE clip names; class 'window' = priority p-1 or p
    (equal priority = unknown order: the new entry at p-1 would tie with it or jump over it), 'below' (< p-1, still wins), 'above' (> p, already loses);
  4 exclusions (clip names that stay vanilla because a 'window' entry holds them; union over entities and registration paths) and the
    registration plan (vanilla vars + utpp_foot_tpp at p-1; paths whose window conflicts cover the whole subset are reported).
"""
import argparse
import glob
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_export_lib as L  # noqa: E402

ENTS = {'player_wa_fpp': r'base\characters\entities\player\player_wa_fpp.ent', 'player_ma_fpp': r'base\characters\entities\player\player_ma_fpp.ent',
        'player_wa_fpp_ep1': r'ep1\characters\entities\player\player_wa_fpp_ep1.ent', 'player_ma_fpp_ep1': r'ep1\characters\entities\player\player_ma_fpp_ep1.ent'}
WRAPPER = 'utpp_foot_tpp'
# registration paths that are mirrored: the default sets (no variable) and the one primary weapon variable of each set.  Paths gated by a
# modifier variable (StrongArms: the fists set for the strong-arms cyberware, which ep1 overrides with its own set at the same priority;
# additives: the one-handed blunt set as an extra source for any weapon; knife_thumb_override) are NOT mirrored: they stay vanilla.
PRIMARY = ('w_melee_fists', 'w_melee_katana', 'w_melee_knife', 'w_melee_one_hand_blunt', 'w_two_hand_blunt', 'w_melee_hammer', 'w_melee_baton')


def weapon_var(v):
    """variables that name the drawn weapon (one at a time): weapon item variables and the arm cyberware weapons."""
    return v.startswith('w_') or v in ('mono_wires', 'mantis_blade', 'projectile_launcher')


def coactive(va, vb):
    """False only when both entries are gated by DIFFERENT weapon variables (one weapon is drawn at a time); a modifier variable or an
    empty list can be true together with anything."""
    wa, wb = {v for v in va if weapon_var(v)}, {v for v in vb if weapon_var(v)}
    return not (wa and wb and wa.isdisjoint(wb))


def mirrored(vars_):
    return len(vars_) == 0 or (len(vars_) == 1 and vars_[0] in PRIMARY)


def entity_rows(doc):
    rows, seen = [], set()

    def walk(v, comp=None):
        if isinstance(v, dict):
            t = v.get('$type')
            if t and 'Component' in t and 'name' in v:
                nm = v['name'].get('$value') if isinstance(v['name'], dict) else v['name']
                comp = nm
            if t == 'animAnimSetupEntry':
                dp = (v.get('animSet') or {}).get('DepotPath', {}).get('$value')
                vars_ = [x.get('$value') if isinstance(x, dict) else x for x in (v.get('variableNames') or [])]
                k = (comp, v.get('priority'), dp, tuple(vars_))
                if k not in seen:
                    seen.add(k)
                    rows.append(dict(component=comp, priority=int(v.get('priority')), set=dp, vars=vars_))
            for x in v.values():
                walk(x, comp)
        elif isinstance(v, list):
            for x in v:
                walk(x, comp)
    walk(doc)
    return rows


def utpp_xl_rows():
    """entries of the shipped UTPP .xl files (simple line parser for the 'animations:' lists used there)."""
    rows = []
    for p in sorted(glob.glob(os.path.join(L.ROOT, 'Source', 'plugin', 'UTPP', 'archive', '*.archive.xl'))):
        if os.path.basename(p) == 'utpp_melee_tpp.archive.xl':
            continue
        cur, ents, in_ent = None, [], False
        for line in open(p, encoding='utf-8'):
            s = line.split('#', 1)[0].rstrip()
            if re.match(r'^\s*-\s*entity:', s):
                if cur:
                    rows.append(cur)
                cur, in_ent = dict(file=os.path.basename(p), entities=[], component=None, set=None, vars=[], priority=None), True
                continue
            if cur is None:
                continue
            m = re.match(r'^\s*-\s*(\S+\.ent)\s*$', s)
            if in_ent and m:
                cur['entities'].append(m.group(1))
                continue
            in_ent = False
            for key in ('component', 'set', 'priority'):
                m = re.match(r'^\s*%s:\s*(.+)$' % key, s)
                if m:
                    cur[key] = int(m.group(1)) if key == 'priority' else m.group(1).strip()
            m = re.match(r'^\s*vars:\s*\[(.*)\]', s)
            if m:
                cur['vars'] = [x.strip() for x in m.group(1).split(',') if x.strip()]
        if cur:
            rows.append(cur)
    return [r for r in rows if r['set'] and r['priority'] is not None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--retail', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--clips-dir', help='input clips dir: only clips with a <set>__<clip>.<suffix>.npz are counted as replaced')
    ap.add_argument('--suffix', default='v4')
    a = ap.parse_args()
    raw, js, jm = (os.path.join(a.retail, x) for x in ('raw', 'json', 'json-mirror'))
    collide = {}
    for p in glob.glob(os.path.join(raw, '**', '*.*'), recursive=True):
        collide.setdefault(os.path.basename(p).lower(), []).append(p)

    def json_of(depot):
        b = os.path.basename(depot)
        if len(collide.get(b.lower(), [])) > 1:
            p = os.path.join(jm, depot + '.json')
        else:
            p = os.path.join(js, b + '.json')
        return p if os.path.exists(p) else None

    clip_cache = {}

    def clips_of(depot):
        if depot not in clip_cache:
            p = json_of(depot)
            if p is None:
                clip_cache[depot] = None
            else:
                doc = json.load(open(p, encoding='utf-8'))
                out = {}
                for e in doc['Data']['RootChunk'].get('animations') or []:
                    d = e['Data']['animation']['Data']
                    out[d['name']['$value']] = d.get('animationType')
                clip_cache[depot] = out
        return clip_cache[depot]

    rep = dict(hashes={}, entities={}, registrations={}, overlap={}, exclusions={}, plan={}, unresolved_sets=[])
    # 1 hashes
    van = os.path.join(L.ROOT, 'Extracted', 'game', 'vanilla')
    for s, dp in L.VANILLA_DEPOT.items():
        r_ = os.path.join(raw, dp)
        v_ = os.path.join(van, dp)
        hr, hv = L.sha256(r_), (L.sha256(v_) if os.path.exists(v_) else None)
        rep['hashes'][s] = dict(depot=dp, retail_sha256=hr, retail_size=os.path.getsize(r_), extracted_vanilla_sha256=hv, identical=hr == hv)
    prov = {}
    pp = os.path.join(L.ROOT, 'Extracted', 'game', 'vehicle-entry-car', 'provenance.txt')
    for line in open(pp, encoding='ascii', errors='replace'):
        parts = line.split()
        if len(parts) >= 3:
            prov[parts[2].lower()] = parts[0].upper()
    for e, dp in ENTS.items():
        h = L.sha256(os.path.join(raw, dp))
        rep['entities'][e] = dict(depot=dp, retail_sha256=h, sha256_20261101=prov.get(dp.lower()), identical=h == prov.get(dp.lower()))
    # 2 + 3
    xl = utpp_xl_rows()
    rep['utpp_xl_entries'] = xl
    melee_depots = {dp.lower(): s for s, dp in L.VANILLA_DEPOT.items()}
    replaced = {}
    for s, dp in L.VANILLA_DEPOT.items():
        cl = clips_of(dp)
        nonadd = sorted(c for c, t in cl.items() if t not in ('Additive', 'AdditiveWithoutFirstFrame'))
        if a.clips_dir:
            nonadd_in = [c for c in nonadd if os.path.exists(os.path.join(a.clips_dir, '%s__%s.%s.npz' % (s, c, a.suffix)))]
        else:
            nonadd_in = nonadd
        replaced[s] = dict(non_additive=nonadd, replaced=nonadd_in, total=len(cl))
    rep['clip_counts'] = {s: dict(total=v['total'], non_additive=len(v['non_additive']), with_input=len(v['replaced'])) for s, v in replaced.items()}
    excl = {s: {} for s in L.SETS}
    for e, dp in ENTS.items():
        doc = json.load(open(os.path.join(js, os.path.basename(dp) + '.json'), encoding='utf-8'))
        rows = entity_rows(doc)
        ent_short = dp
        for r in xl:
            if any(x.lower() == ent_short.lower() for x in r['entities']):
                rows.append(dict(component=r['component'], priority=r['priority'], set=r['set'], vars=r['vars'], utpp_xl=r['file']))
        rep['entities'][e]['entries'] = len(rows)
        for r in rows:
            if r['set'] and clips_of(r['set']) is None and r['set'] not in rep['unresolved_sets']:
                rep['unresolved_sets'].append(r['set'])
        mine = [r for r in rows if (r['set'] or '').lower() in melee_depots and not r.get('utpp_xl')]
        rep['registrations'][e] = mine
        for m in mine:
            s = melee_depots[m['set'].lower()]
            names = set(replaced[s]['replaced'])
            p = m['priority']
            key = '%s | %s | %d | %s' % (e, s, p, ','.join(m['vars']))
            lst = []
            for r in rows:
                if r is m or (r['set'] or '').lower() == m['set'].lower() and not r.get('utpp_xl'):
                    continue
                cl = clips_of(r['set']) if r['set'] else None
                if not cl:
                    continue
                shared = sorted(names & {c for c, t in cl.items() if t not in ('Additive', 'AdditiveWithoutFirstFrame')})
                if not shared:
                    continue
                po = r['priority']
                if po < p - 1:
                    cls = 'below'          # more specific: still wins over the new entry when active
                elif po > p:
                    cls = 'above'          # already loses against the vanilla entry
                elif not coactive(m['vars'], r['vars']):
                    cls = 'exclusive'      # gated by another weapon variable: never active together
                elif po == p - 1:
                    cls = 'tie'            # same priority as the new entry: unknown order -> clip names stay vanilla
                elif not r['vars']:
                    cls = 'always-same'    # unconditioned entry at the vanilla priority: unknown order -> clip names stay vanilla
                else:
                    cls = 'modifier-same'  # modifier-gated entry at the vanilla priority: the new entry wins where the vanilla one won (risk listed)
                lst.append(dict(component=r['component'], priority=po, set=r['set'], vars=r['vars'], shared=len(shared), cls=cls,
                                shared_names=shared, utpp_xl=r.get('utpp_xl')))
                if mirrored(m['vars']) and cls in ('tie', 'always-same'):
                    for c in shared:
                        excl[s].setdefault(c, []).append('%s: %s %d %s (%s)' % (e, r['set'], po, r['vars'], cls))
            order = {'tie': 0, 'always-same': 1, 'modifier-same': 2, 'exclusive': 3, 'below': 4, 'above': 5}
            rep['overlap'][key] = sorted(lst, key=lambda x: (order[x['cls']], x['priority']))
            if mirrored(m['vars']):
                rep.setdefault('risks', {})[key] = [dict(set=x['set'], priority=x['priority'], vars=x['vars'], component=x['component'], shared=x['shared'])
                                                     for x in lst if x['cls'] == 'modifier-same']
    rep['exclusions'] = {s: sorted(v) for s, v in excl.items()}
    rep['exclusion_reasons'] = {s: v for s, v in excl.items() if v}
    # plan
    for e, mine in rep['registrations'].items():
        for m in mine:
            s = melee_depots[m['set'].lower()]
            keep = [c for c in replaced[s]['replaced'] if c not in excl[s]]
            row = dict(entity=e, entity_depot=ENTS[e], component=m['component'], vanilla_priority=m['priority'], vanilla_vars=m['vars'],
                       mirrored=mirrored(m['vars']), clips=len(keep))
            if row['mirrored']:
                row.update(utpp_priority=m['priority'] - 1, utpp_vars=m['vars'] + [WRAPPER])
            else:
                row['why_not'] = 'modifier-gated path stays vanilla'
            rep['plan'].setdefault(s, []).append(row)
    os.makedirs(a.out, exist_ok=True)
    L.jdump(rep, os.path.join(a.out, 'registration.json'))
    with open(os.path.join(a.out, 'registration.txt'), 'w', encoding='utf-8') as f:
        f.write('HASHES (retail 2.5 vs Extracted\\game\\vanilla)\n')
        for s, h in rep['hashes'].items():
            f.write('  %-26s %s %s %s\n' % (s, h['retail_sha256'][:16], 'IDENTICAL' if h['identical'] else 'DIFFERENT', h['depot']))
        f.write('ENTITIES (retail now vs 2026-11-01 extraction)\n')
        for e, h in rep['entities'].items():
            f.write('  %-20s %s %s entries %d\n' % (e, h['retail_sha256'][:16], 'IDENTICAL' if h['identical'] else 'DIFFERENT', h.get('entries', 0)))
        f.write('CLIPS per set: total / non-additive / with input\n')
        for s, c in rep['clip_counts'].items():
            f.write('  %-26s %d / %d / %d\n' % (s, c['total'], c['non_additive'], c['with_input']))
        f.write('REGISTRATIONS of the 12 sets\n')
        for e, mine in rep['registrations'].items():
            for m in mine:
                f.write('  %-18s %-6s %4d %-75s %s\n' % (e, m['component'], m['priority'], m['set'], m['vars']))
        f.write('PLAN (mirrored paths: vanilla vars + %s at vanilla priority - 1)\n' % WRAPPER)
        for s, rows in rep['plan'].items():
            for r in rows:
                f.write('  %-26s %-18s %4d %-40s -> %s\n' % (s, r['entity'], r['vanilla_priority'], r['vanilla_vars'],
                                                         ('%d %s clips %d' % (r['utpp_priority'], r['utpp_vars'], r['clips'])) if r['mirrored'] else 'NOT MIRRORED (%s)' % r['why_not']))
        f.write('RISKS (mirrored path vs modifier-gated entries of the vanilla priority sharing clip names)\n')
        for k, v in (rep.get('risks') or {}).items():
            for x in v:
                f.write('  %-60s %-80s %4d %s shared %d (%s)\n' % (k, x['set'], x['priority'], x['vars'], x['shared'], x['component']))
        f.write('OVERLAP (other entries holding replaced clip names; classes: tie / always-same (excluded) / modifier-same (risk) / exclusive / below / above)\n')
        for k, lst in rep['overlap'].items():
            f.write('  %s\n' % k)
            for x in lst:
                f.write('     %-6s %-22s %4d %-90s %-40s shared %3d %s%s\n' % (x['cls'], (x['component'] or '')[:22], x['priority'], x['set'], x['vars'], x['shared'],
                                                                         ','.join(x['shared_names'][:6]), (' [%s]' % x['utpp_xl']) if x['utpp_xl'] else ''))
        f.write('EXCLUSIONS (stay vanilla)\n')
        for s, v in rep['exclusions'].items():
            f.write('  %-26s %d %s\n' % (s, len(v), v))
        f.write('UNRESOLVED registered sets (no json): %s\n' % rep['unresolved_sets'])
    print('REGISTRATION_EVIDENCE hashes identical %d/12, entities identical %d/4, exclusions %s, unresolved %d -> %s' % (
        sum(h['identical'] for h in rep['hashes'].values()), sum(h['identical'] for h in rep['entities'].values()),
        {s: len(v) for s, v in rep['exclusions'].items() if v}, len(rep['unresolved_sets']), a.out))


if __name__ == '__main__':
    main()
