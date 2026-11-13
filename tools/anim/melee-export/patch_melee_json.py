"""After the WolvenKit bridge import of a melee set: put back the vanilla clip metadata (2026-11-09).

  python -I patch_melee_json.py <imported.anims.json> <vanilla.anims.json> <out.anims.json> [--report r.json]

For every clip of the imported set (the vanilla set reduced to the replaced clips + the new bone data):
  * motionExtraction = the vanilla clip's, VERBATIM (Data copied; a null vanilla block stays null) - the importer writes motion extraction
    keys for all joints and without the axis conversion (vehicle-entry README, root-motion bug b);
  * every other field of the set entry / animation that differs from vanilla (events, tags, ...) is replaced by the vanilla value; handles inside
    the copied value get fresh HandleIds (internal HandleRefIds remapped, a reference to a handle outside the copied value is an error);
  * animBuffer (the new bone data incl. float tracks) is the importer's, except: numFrames = vanilla (compressed buffers; see below) and
    fallbackFrameIndices = [] (2026-11-09: the set-level fallback frames are the vanilla first-person poses; like the vehicle-entry clips,
    build_entry_clip.py, a replaced clip has no fallback frames).
  * SIMD buffers (uncompressed frames, e.g. pwa_knife throw_start): the importer writes floor(duration * 30) + 1 frames; when that differs
    from vanilla, numFrames is NOT restored (the reader would run past the frame data: WolvenKit 'Unable to read beyond the end of the
    stream'); the clip is listed under 'unpatchable' and the exit code is 4 (run_melee_export.ps1 rebuilds the set without it).
Exit 1 when a clip is missing in either file, 4 when a clip is unpatchable (the output is written in both cases 0 / 4).
"""
import argparse
import copy
import json
import sys
from pathlib import Path

SKIP_KEYS = ('HandleId', 'HandleRefId', 'BufferId', 'BufferRefId', 'Flags')


def strip(o):
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k not in SKIP_KEYS}
    if isinstance(o, list):
        return [strip(x) for x in o]
    return o


def max_handle(doc):
    m, st = 0, [doc]
    while st:
        v = st.pop()
        if isinstance(v, dict):
            if 'HandleId' in v:
                m = max(m, int(v['HandleId']))
            st.extend(v.values())
        elif isinstance(v, list):
            st.extend(v)
    return m


def renumber(value, counter):
    v = copy.deepcopy(value)
    mp, refs, st = {}, [], [v]
    while st:
        x = st.pop()
        if isinstance(x, dict):
            if 'HandleId' in x:
                counter[0] += 1
                mp[x['HandleId']] = str(counter[0])
                x['HandleId'] = str(counter[0])
            if 'HandleRefId' in x and x['HandleRefId'] != '-1':
                refs.append(x)
            st.extend(x.values())
        elif isinstance(x, list):
            st.extend(x)
    for r in refs:
        if r['HandleRefId'] not in mp:
            raise SystemExit('copied value references an outside handle %s' % r['HandleRefId'])
        r['HandleRefId'] = mp[r['HandleRefId']]
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('imported')
    ap.add_argument('vanilla')
    ap.add_argument('out')
    ap.add_argument('--report')
    a = ap.parse_args()
    doc = json.loads(Path(a.imported).read_text(encoding='utf-8'))
    van = json.loads(Path(a.vanilla).read_text(encoding='utf-8'))
    vmap = {e['Data']['animation']['Data']['name']['$value']: e for e in van['Data']['RootChunk']['animations']}
    counter = [max_handle(doc) + 1000]
    rep = dict(imported=a.imported, vanilla=a.vanilla, clips={}, unpatchable={})
    bad = []
    for e in doc['Data']['RootChunk']['animations']:
        ad = e['Data']['animation']['Data']
        c = ad['name']['$value']
        if c not in vmap:
            bad.append(c)
            continue
        ve = vmap[c]
        vad = ve['Data']['animation']['Data']
        row = dict(transplanted=[])
        # motion extraction (verbatim)
        vme, ime = vad.get('motionExtraction'), ad.get('motionExtraction')
        if strip(vme) != strip(ime):
            if vme is None or (isinstance(vme, dict) and vme.get('Data') is None and 'HandleId' not in vme):
                ad['motionExtraction'] = copy.deepcopy(vme)
            elif isinstance(ime, dict) and 'HandleId' in ime and 'HandleId' in vme:
                ime['Data'] = renumber(vme['Data'], counter)
            else:
                ad['motionExtraction'] = renumber(vme, counter)
            row['transplanted'].append('animation.motionExtraction')
        # the other fields of the animation object and of the set entry
        for obj, vobj, prefix in ((ad, vad, 'animation.'), (e['Data'], ve['Data'], '')):
            for k in sorted(set(obj) | set(vobj)):
                if k in SKIP_KEYS or k in ('animation', 'animBuffer', 'motionExtraction', '$type'):
                    continue
                if strip(obj.get(k)) != strip(vobj.get(k)):
                    if k in vobj:
                        obj[k] = renumber(vobj[k], counter)
                    else:
                        del obj[k]
                    row['transplanted'].append(prefix + k)
        ib, vb = (ad.get('animBuffer') or {}).get('Data') or {}, (vad.get('animBuffer') or {}).get('Data') or {}
        row['animBuffer_keys_differ'] = sorted(k for k in set(ib) | set(vb) if k not in SKIP_KEYS and strip(ib.get(k)) != strip(vb.get(k)))
        row['numFrames_imported'] = ib.get('numFrames')
        if ib.get('numFrames') != vb.get('numFrames'):
            if ib.get('$type') != 'animAnimationBufferCompressed':
                # uncompressed (SIMD) frame data: the frame count is the data size, it cannot be restored
                rep['unpatchable'][c] = 'SIMD anim buffer: the importer wrote %s frames, vanilla %s (restoring numFrames makes the buffer unreadable)' % (
                    ib.get('numFrames'), vb.get('numFrames'))
            else:
                # WolvenKit importer: numFrames = floor(duration * 30) + 1, one short when duration * 30 is a hair under an integer (e.g.
                # 1.79999995 s -> 54 instead of 55). The compressed keys are time based (the export of the imported clip has all frames, see
                # verify), so the vanilla frame count is restored (events are frame indices of the vanilla clip).
                ib['numFrames'] = vb.get('numFrames')
                row['transplanted'].append('animBuffer.numFrames')
        ff = ib.get('fallbackFrameIndices')
        row['fallback_frames_vanilla'] = len(vb.get('fallbackFrameIndices') or [])
        if ff:
            ib['fallbackFrameIndices'] = []
            row['transplanted'].append('animBuffer.fallbackFrameIndices=[]')
        row.update(numFrames=ib.get('numFrames'), numFrames_vanilla=vb.get('numFrames'), duration=ad.get('duration'), duration_vanilla=vad.get('duration'),
                   buffer=ib.get('$type'), buffer_vanilla=vb.get('$type'), numJoints=ib.get('numJoints'), numJoints_vanilla=vb.get('numJoints'),
                   numTracks=ib.get('numTracks'), numTracks_vanilla=vb.get('numTracks'))
        rep['clips'][c] = row
    if bad:
        print('PATCH_FAIL clips not in vanilla: %s' % bad)
        return 1
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(doc, separators=(',', ':')) + '\n', encoding='utf-8')
    tp = {}
    for r in rep['clips'].values():
        for t in r['transplanted']:
            tp[t] = tp.get(t, 0) + 1
    nf = sum(1 for r in rep['clips'].values() if r['numFrames'] != r['numFrames_vanilla'])
    bk = {}
    for r in rep['clips'].values():
        for k in r['animBuffer_keys_differ']:
            bk[k] = bk.get(k, 0) + 1
    rep['summary'] = dict(clips=len(rep['clips']), transplanted=tp, numFrames_differ_after=nf, animBuffer_keys_differ=bk,
                          buffer_types=sorted({'%s<-%s' % (r['buffer'], r['buffer_vanilla']) for r in rep['clips'].values()}),
                          unpatchable=sorted(rep['unpatchable']))
    if a.report:
        Path(a.report).write_text(json.dumps(rep, indent=1) + '\n', encoding='utf-8')
    print('PATCH_MELEE clips %d transplanted %s numFrames differ after %d | animBuffer keys differing from vanilla (imported) %s | buffers %s | unpatchable %s' % (
        len(rep['clips']), tp, nf, bk, rep['summary']['buffer_types'], sorted(rep['unpatchable']) or 'none'))
    return 4 if rep['unpatchable'] else 0


if __name__ == '__main__':
    sys.exit(main())
