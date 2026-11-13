"""Keep only selected clips of a WolvenKit-serialized .anims JSON (and optionally rename them).

All animationDataChunks / buffers are retained (address/BufferId safety, same approach as
the earlier isolate_nue_custom_animset.py, since removed; see git history); handle and buffer references are validated.

Usage (python -I):
  animset_subset.py <in.anims.json> <out.anims.json> --keep enter [--keep switch_seat] [--rename enter=utpp_x]
                    [--list]
"""
import argparse
import hashlib
import json
from pathlib import Path


def clip_name(entry):
    return entry['Data']['animation']['Data']['name']['$value']


def refs_ok(doc):
    definitions, references, buffers, buffer_refs = set(), set(), set(), set()
    stack = [doc]
    while stack:
        v = stack.pop()
        if isinstance(v, dict):
            if 'HandleId' in v:
                definitions.add(v['HandleId'])
            if 'HandleRefId' in v and v['HandleRefId'] != '-1':
                references.add(v['HandleRefId'])
            if 'BufferId' in v:
                buffers.add(v['BufferId'])
            if 'BufferRefId' in v:
                buffer_refs.add(v['BufferRefId'])
            stack.extend(v.values())
        elif isinstance(v, list):
            stack.extend(v)
    return references - definitions, buffer_refs - buffers


def main():
    p = argparse.ArgumentParser()
    p.add_argument('src')
    p.add_argument('dst', nargs='?')
    p.add_argument('--keep', action='append', default=[])
    p.add_argument('--rename', action='append', default=[])
    p.add_argument('--list', action='store_true')
    a = p.parse_args()
    doc = json.loads(Path(a.src).read_text(encoding='utf-8'))
    root = doc['Data']['RootChunk']
    anims = root['animations']
    if a.list:
        for e in anims:
            d = e['Data']['animation']['Data']
            print(clip_name(e), d.get('duration'), d.get('animationType'))
        return
    keep = set(a.keep)
    sel = [e for e in anims if clip_name(e) in keep]
    missing = keep - {clip_name(e) for e in sel}
    if missing:
        raise SystemExit('missing clips: %s' % sorted(missing))
    root['animations'] = sel
    ren = dict(r.split('=', 1) for r in a.rename)
    for e in sel:
        n = clip_name(e)
        if n in ren:
            e['Data']['animation']['Data']['name']['$value'] = ren[n]
    dangling_h, dangling_b = refs_ok(doc)
    # Removing clips can leave HandleRefIds that pointed into removed clips (shared handles): fail loudly.
    if dangling_h or dangling_b:
        raise SystemExit('dangling refs handles=%s buffers=%s' % (sorted(dangling_h)[:10], sorted(dangling_b)[:10]))
    out = Path(a.dst)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, separators=(',', ':')) + '\n', encoding='utf-8')
    print(json.dumps(dict(src=a.src, dst=str(out), clips=[clip_name(e) for e in sel],
                          sha256=hashlib.sha256(out.read_bytes()).hexdigest())))


if __name__ == '__main__':
    main()
