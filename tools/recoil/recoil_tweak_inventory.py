"""Step 2 of recoil-missing-20261109: which player firearms of game 2.5 get which recoil profile today.

Inputs (read only):
  * TweakDB sources shipped with the game for REDmod: <game>\\tools\\redmod\\tweaks\\{base,ep1}\\...\\items\\weapons\\ranged\\**\\*.tweak
    (record inheritance resolved here: itemType = nearest definition, tags = '=' resets / '+=' appends along the chain);
  * the retail 2.5 r6\\cache\\tweakdb.bin + tweakdb_ep1.bin: every Items.Preset_* / Items.Base_* name is looked up by its TweakDBID
    (CRC32 of the name + length byte) to prove the record still exists in 2.5 (tags/itemType are NOT re-read from the binary);
  * Source\\plugin\\UTPP\\scripts\\utpp_motion.reds: UTPPMotionRecoilProfileOfFamily (family tag -> id) and FallbackRecoilProfile (item type);
  * Extracted\\game\\player-anims\\glb: the 2.5 pwa_/pma_ weapon animsets and their recoil clips (names only, from the GLB JSON chunk).
Writes the JSON inventory given by --out.
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # the tools folder
from utpp_paths import game_dir  # noqa: E402

GAME = Path(game_dir())
ROOT = Path(__file__).resolve().parents[3]
GENERIC = {'Preload', 'Weapon', 'WeaponAttachment', 'HasFirstEquip', 'T1_Inject_Weapon', 'T2_Inject_Weapon', 'T3_Inject_Weapon',
           'DiscardOnEmpty', 'AutoScalingItem', 'Handgun', 'Revolver', 'SMG', 'Shotgun', 'Shotgun Dual', 'Rifle Assault',
           'Rifle Precision', 'Rifle Sniper', 'Unique', 'Dogtown_Weapon', 'Techtronika', 'AnimCycleRound', 'Special',
           'Grenade Launcher', 'DualWield', 'OneHandedRangedWeapon', 'DriverCombatRangedWeapon', 'DriverCombatBikeWeapon',
           'Power', 'Tech', 'Smart', 'IconicWeapon', 'Pimp', 'Neon', 'Military', 'Default'}
RECORD = re.compile(r'^([A-Za-z_][\w]*)\s*(?::\s*([A-Za-z_][\w]*))?\s*$')
TAGS = re.compile(r'(?:CName\[\]\s*)?\btags\s*(\+=|=)\s*\[(.*?)\]', re.S)
ITEMTYPE = re.compile(r'itemType\s*=\s*"ItemType\.(\w+)"')
PRIMARY = re.compile(r'primaryTriggerMode\s*=\s*"TriggerMode\.(\w+)"')
MODES = re.compile(r'triggerModes\s*=\s*\[(.*?)\]', re.S)
FRIENDLY = re.compile(r'friendlyName\s*=\s*"([^"]*)"')
SETS = ROOT / 'Build/work/melee-export-20261109/evidence'  # player entity animsets + wrapper variables (2.5 retail, read only)
CYCLE = re.compile(r'value\s*=\s*([-\d.]+)f?\s*;\s*\}\s*:\s*(CycleTimeModifier|CycleTime_BurstModifier|NumShotsInBurstModifier)\b')
ANIM_ALIAS = {'borg4a': 'midnight_borg', 'tactician': 'constitutional_arms_tactician', 'rasetsu': 'tsunami_rasetsu', 'sor22': 'midnight_sor22'}


def parse_records(path: Path):
    text = path.read_text(encoding='utf-8', errors='replace').replace('\r', '')
    lines = text.split('\n')
    out = {}
    i = 0
    depth = 0
    while i < len(lines):
        line = lines[i]
        m = RECORD.match(line.strip()) if depth == 0 else None
        if m and i + 1 < len(lines) and lines[i + 1].strip() == '{':
            # collect the body until the matching closing brace
            body, d, j = [], 0, i + 1
            while j < len(lines):
                body.append(lines[j])
                d += lines[j].count('{') - lines[j].count('}')
                if d == 0:
                    break
                j += 1
            out[m.group(1)] = dict(parent=m.group(2), body='\n'.join(body), file=str(path))
            i = j + 1
            continue
        depth += line.count('{') - line.count('}')
        i += 1
    return out


def top_level(body: str, regex):
    """matches whose start lies at brace depth 1 of the record body (not inside inline records)."""
    depth_at, d = [], 0
    for ch in body:
        if ch == '{':
            d += 1
        elif ch == '}':
            d -= 1
        depth_at.append(d)
    return [m for m in regex.finditer(body) if depth_at[m.start()] == 1]


def names(list_text: str):
    return re.findall(r'"([^"]+)"', list_text)


def tdbid5(name: str) -> bytes:
    return struct.pack('<IB', zlib.crc32(name.encode()) & 0xffffffff, len(name))


def reds_mapping(reds: Path):
    text = reds.read_text(encoding='utf-8')
    fam = dict((k, int(v)) for k, v in re.findall(r'Equals\(family, n"(\w+)"\) \{ return (\d+); \}', text))
    fb = text[text.find('func FallbackRecoilProfile'):]
    fb = fb[:fb.find('\n  }\n')]
    fallback = {}
    for block in re.finditer(r'if (.*?)\{\s*return (\d+);', fb, re.S):
        for t in re.findall(r'gamedataItemType\.(\w+)', block.group(1)):
            fallback[t] = int(block.group(2))
    return fam, fallback


def glb_recoil_clips(path: Path):
    with open(path, 'rb') as f:
        f.read(12)
        clen, _ = struct.unpack('<II', f.read(8))
        doc = json.loads(f.read(clen))
    clips = {}
    for a in doc.get('animations', []):
        if 'recoil' in a['name']:
            dur = max((doc['accessors'][s['input']].get('max', [0])[0] for s in a['samplers']), default=0.0)
            clips[a['name']] = (a.get('extras', {}).get('animationType'), round(float(dur), 4))
    return clips


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--game', type=Path, default=GAME)
    ap.add_argument('--out', type=Path, default=ROOT / 'Build/work/recoil-missing-20261109/weapon-inventory-25.json')
    a = ap.parse_args()
    tweaks = a.game / 'tools/redmod/tweaks'
    records = {}
    files = sorted(p for p in tweaks.rglob('*.tweak') if 'items' in p.parts and 'weapons' in p.parts and 'ranged' in p.parts)
    for p in files:
        for name, rec in parse_records(p).items():
            rec['file'] = str(p.relative_to(tweaks))
            records[name] = rec
    resolved = {}

    def resolve(name, seen=()):
        if name in resolved:
            return resolved[name]
        rec = records.get(name)
        if rec is None or name in seen:
            return dict(itemType=None, tags=[], chain=[])
        base = resolve(rec['parent'], seen + (name,)) if rec['parent'] else dict(itemType=None, tags=[], chain=[])
        tags = list(base['tags'])
        for m in top_level(rec['body'], TAGS):
            tags = names(m.group(2)) if m.group(1) == '=' else tags + [t for t in names(m.group(2)) if t not in tags]
        it = top_level(rec['body'], ITEMTYPE)
        prim = top_level(rec['body'], PRIMARY)
        friendly = top_level(rec['body'], FRIENDLY)
        modes = top_level(rec['body'], MODES)
        r = dict(itemType=it[-1].group(1) if it else base['itemType'], tags=tags, chain=[name] + base['chain'],
                 primaryTriggerMode=prim[-1].group(1) if prim else base.get('primaryTriggerMode'),
                 triggerModes=[x.split('.')[-1] for x in names(modes[-1].group(1))] if modes else base.get('triggerModes'),
                 file=rec['file'], friendlyName=friendly[-1].group(1) if friendly else base.get('friendlyName'))
        resolved[name] = r
        return r

    fam_map, fallback = reds_mapping(ROOT / 'Source/plugin/UTPP/scripts/utpp_motion.reds')
    fam_lower = {k.lower(): (k, v) for k, v in fam_map.items()}
    blobs = [(p.name, p.read_bytes()) for p in (a.game / 'r6/cache/tweakdb.bin', a.game / 'r6/cache/tweakdb_ep1.bin') if p.is_file()]
    glb_root = ROOT / 'Extracted/game/player-anims/glb'
    animsets = {p.name: p for p in glb_root.rglob('pwa_w_*.anims.glb') if 'dual_wield' not in p.name and '_explosives_' not in p.name}

    var_sets = {}
    for ent in ('player_wa_fpp_ep1', 'player_ma_fpp_ep1'):
        f = SETS / (ent + '.sets.json')
        if f.is_file():
            for row in json.loads(f.read_text(encoding='utf-8')):
                for v in row.get('vars', []):
                    var_sets.setdefault(v, []).append(row['set'].replace(chr(92), '/').split('/')[-1])
    if not var_sets:
        doc = json.loads((SETS / 'player_entity_sets_unique.json').read_text(encoding='utf-8'))
        for ent in ('player_wa_fpp_ep1', 'player_ma_fpp_ep1'):
            for row in doc.get(ent, []):
                for v in row.get('vars', []):
                    var_sets.setdefault(v, []).append(row['set'].replace(chr(92), '/').split('/')[-1])

    def descendants(name):
        out = []
        for n, x in records.items():
            if x['parent'] == name and n.startswith('Preset_'):
                out += [n] + descendants(n)
        return out

    families = []
    for name in sorted(records):
        rec = records[name]
        if not name.startswith('Base_') or '_Stats' in name or '_NPC' in name or '/bases/' in rec['file'].replace('\\', '/'):
            continue
        r = resolve(name)
        if not r['itemType'] or not r['itemType'].startswith('Wea_'):
            continue
        parent = resolve(rec['parent']) if rec['parent'] else dict(tags=[])
        added = [t for t in r['tags'] if t not in parent['tags'] and t not in GENERIC]
        tech = records.get(name + '_Technical_Stats')
        cycle = {}
        if tech:
            for m in CYCLE.finditer(tech['body']):
                cycle.setdefault(m.group(2), float(m.group(1)))
        # profile resolution of utpp_motion.reds today (Nue = exact preset id, then family tags, then the item-type fallback)
        hits = {fam_map[t] for t in r['tags'] if t in fam_map}
        if name == 'Base_Nue':
            pid, how = 1, 'preset Items.Preset_Nue_Default'
        elif len(hits) == 1:
            pid, how = hits.pop(), 'family tag'
        elif len(hits) > 1:
            pid, how = 0, 'conflicting family tags'
        else:
            pid, how = fallback.get(r['itemType'], 0), 'item-type fallback'
        key = name[len('Base_'):].lower()
        anim_key = ANIM_ALIAS.get(key, key)
        own = sorted(n for n in animsets if re.search(r'(^|_)' + re.escape(anim_key) + r'\.anims\.glb$', n))
        presets = sorted(set(descendants(name)))
        preset_rows = []
        for pn in presets:
            pr = resolve(pn)
            ph = {fam_map[t] for t in pr['tags'] if t in fam_map}
            if pn == 'Preset_Nue_Default':
                pp, phow = 1, 'preset id'
            elif len(ph) == 1:
                pp, phow = ph.pop(), 'family tag'
            elif len(ph) > 1:
                pp, phow = 0, 'conflicting tags'
            else:
                pp, phow = fallback.get(pr['itemType'], 0), 'item-type fallback'
            preset_rows.append(dict(record='Items.' + pn, itemType=pr['itemType'], profile=pp, how=phow,
                                    friendlyName=pr.get('friendlyName'),
                                    familyTags=[t for t in pr['tags'] if t not in GENERIC and t not in parent['tags']]))
        fname = r.get('friendlyName')
        sets = sorted(set(var_sets.get(fname, []))) if fname else []
        pwa_sets = [x for x in sets if x.startswith('pwa_')]
        if pwa_sets:
            own = sorted(n for n in animsets if n.replace('.glb', '') in pwa_sets)
        exists = {b: [p for p in ['Items.' + name] + ['Items.' + x for x in presets] if tdbid5(p) in blob] for b, blob in blobs}
        families.append(dict(record='Items.' + name, file=r['file'], itemType=r['itemType'], chain=r['chain'],
                             familyTags=added, allTags=r['tags'], triggerModes=r['triggerModes'],
                             primaryTriggerMode=r['primaryTriggerMode'], cycleTime=cycle, presets=presets,
                             existsIn25={b: len(v) for b, v in exists.items()}, presetCount=len(presets) + 1,
                             currentProfile=pid, currentHow=how, friendlyName=fname, playerSets=sets,
                             presetProfiles=sorted({(p['profile'], p['how']) for p in preset_rows}), presetRows=preset_rows,
                             animset=[str(animsets[n].relative_to(ROOT)) for n in own],
                             recoilClips=glb_recoil_clips(animsets[own[0]]) if own else {}))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(dict(tweakSources=str(tweaks), familyMap=fam_map, itemTypeFallback=fallback, families=families),
                                indent=2) + '\n', encoding='utf-8')
    print(f"{'record':<22} {'itemType':<20} {'family tags':<26} {'cur':>3} {'how':<20} {'cycle':>6} {'in2.5':>6} animset / ADS clip")
    for f in families:
        clips = f['recoilClips']
        ads = ('ironsight' if 'add_recoil_single_shot_ironsight' in clips else
               'charged_ironsight' if 'add_recoil_charged_ironsight' in clips else 'NONE') if clips else '-'
        hip = 'shoulder' if 'add_recoil_single_shot_shoulder' in clips else ('NONE' if clips else '-')
        ex = '/'.join(str(v) for v in f['existsIn25'].values())
        pp = ','.join(sorted({str(p) for p, _ in f['presetProfiles']})) or str(f['currentProfile'])
        how = ','.join(sorted({h.split()[0] for _, h in f['presetProfiles']})) or f['currentHow']
        ptags = sorted({t for p in f['presetRows'] for t in p['familyTags']} | set(f['familyTags']))
        print(f"{f['record'][6:]:<22} {f['itemType']:<20} {','.join(ptags)[:26]:<26} {pp:>5} "
              f"{how[:20]:<20} {f['cycleTime'].get('CycleTimeModifier', 0):>6.3f} {ex + '/' + str(f['presetCount']):>6} "
              f"{(Path(f['animset'][0]).name.replace('.anims.glb', '') if f['animset'] else '-')} hip={hip} ads={ads}")


if __name__ == '__main__':
    sys.exit(main())
