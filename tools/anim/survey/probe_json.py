"""Probe: which raw sets did not serialize; key structure of one serialized set; rig paths histogram."""
import collections
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

raws = [p[len(SC.RAW) + 1:] for p in glob.glob(os.path.join(SC.RAW, '**', '*.anims'), recursive=True)]
missing = [r for r in raws if not os.path.exists(SC.json_path(r))]
print('raw', len(raws), 'no json', len(missing))
for m in missing:
    print('  NOJSON', m, os.path.getsize(os.path.join(SC.RAW, m)))
names = collections.Counter(os.path.basename(r) for r in raws)
print('duplicate file names:', [n for n, c in names.items() if c > 1])
rigs = collections.Counter()
types = collections.Counter()
for r in raws:
    js = SC.load_set_json(r)
    if js is None:
        continue
    rig, cl = SC.clips_of(js)
    rigs[rig] += 1
    for c in cl:
        types[c['type']] += 1
print(rigs.most_common())
print(types.most_common())
js = SC.load_set_json(raws[0])
a = js['Data']['RootChunk']['animations'][0]['Data']['animation']['Data']
print(sorted(a.keys()))
print(sorted(a['animBuffer']['Data'].keys()))
