"""For every scan candidate, check whether the magazine visibility track it would be bound to ever hides the magazine.

Usage: python -I check_show_tracks.py <candidates.json> <anim_raw root> <anim_glb root> <report.json>

<anim_raw root>: WolvenKit unbundle + convert serialize output of the weapon .anims/.rig (one sub-folder per archive).
<anim_glb root>: WolvenKit export of the same .anims (glb with trackKeys/constTrackKeys extras), same layout.
Track indices are mapped to names through the rig the .anims file itself references (rig.trackNames), never through a
fixed table. Only player weapon sets (base\\animations\\weapon\\... and the Phantom Liberty sets under
ep1\\animations\\weapon\\..., e.g. Metel, Osprey, Rasetsu) are evaluated; NPC sets are ignored. A candidate HIDES when its
parameter track takes a value < 0.5 in at least one clip. Read-only.
"""
import json
import os
import struct
import sys

PLAYER_WEAPON_SETS = ('base\\animations\\weapon\\', 'ep1\\animations\\weapon\\')


def cname(v):
    return v.get('$value') if isinstance(v, dict) else v


def read_glb_json(path):
    data = open(path, 'rb').read()
    length = struct.unpack_from('<I', data, 12)[0]
    return json.loads(data[20:20 + length].decode('utf-8'))


def index(root, suffix):
    out = {}
    for top in sorted(os.listdir(root)):
        base = os.path.join(root, top)
        for dp, _, fs in os.walk(base):
            for f in fs:
                if f.endswith(suffix):
                    out[os.path.relpath(os.path.join(dp, f), base)[:-len(suffix)]] = os.path.join(dp, f)
    return out


def main():
    cand_path, raw_root, glb_root, report = sys.argv[1:5]
    cands = json.load(open(cand_path, encoding='utf-8'))['candidates']
    raw_json = index(raw_root, '.json')
    glbs = index(glb_root, '.glb')
    rig_names = {}

    def track_names(rig):
        if rig not in rig_names:
            rc = json.load(open(raw_json[rig], encoding='utf-8'))['Data']['RootChunk']
            rig_names[rig] = [cname(t) for t in rc.get('trackNames') or []]
        return rig_names[rig]

    out = []
    for c in cands:
        params = c['siblingParams']
        res = {'app': c['app'], 'mesh': c['mesh'], 'params': params, 'sets': []}
        for a in c['animated'] or []:
            if not set(params) & set(a['params']):
                continue
            for s in a['animSets']:
                if not s.startswith(PLAYER_WEAPON_SETS):
                    continue
                if s not in raw_json or s not in glbs:
                    res['sets'].append({'set': s, 'error': 'not extracted'})
                    continue
                arc = json.load(open(raw_json[s], encoding='utf-8'))['Data']['RootChunk']
                rig = cname((arc.get('rig') or {}).get('DepotPath'))
                names = track_names(rig)
                gj = read_glb_json(glbs[s])
                clips = {}
                for anim in gj.get('animations') or []:
                    ex = anim.get('extras') or {}
                    for p in params:
                        if p not in names:
                            continue
                        ti = names.index(p)
                        keys = sorted((k['time'], k['value']) for k in ex.get('trackKeys') or [] if k['trackIndex'] == ti)
                        keys += [(k['time'], k['value']) for k in ex.get('constTrackKeys') or [] if k['trackIndex'] == ti]
                        if not keys:
                            continue
                        segs, prev = [], None
                        for t, v in sorted(keys):
                            v = round(v, 3)
                            if v != prev:
                                segs.append([round(t, 3), v])
                                prev = v
                        clips.setdefault(anim['name'], {})[p] = segs
                hiding = sorted({n for n, d in clips.items() for p, segs in d.items() if any(v < 0.5 for _, v in segs)})
                res['sets'].append({'set': s, 'rig': rig, 'trackNames': names, 'clips': clips, 'hidingClips': hiding})
        res['hides'] = any(x.get('hidingClips') for x in res['sets'])
        out.append(res)
        print('%-60s %-62s hides=%s %s' % (os.path.basename(c['app']), os.path.basename(c['mesh']), res['hides'],
              '; '.join('%s:%s' % (os.path.basename(x['set']), ','.join(x.get('hidingClips') or [x.get('error', '-')])) for x in res['sets'])))
    with open(report, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(out, f, indent=1, sort_keys=True)


if __name__ == '__main__':
    main()
