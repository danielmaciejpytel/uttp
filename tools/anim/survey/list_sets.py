"""Compact listing of the 317 registered player sets: path | bodies | priorities | vars | clips (first names)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

doc = json.load(open(SC.SETS_JSON, encoding='utf-8'))
rows = {}
for ent, lst in doc.items():
    for r in lst:
        k = r['set'].lower()
        e = rows.setdefault(k, dict(ents=set(), pr=set(), vars=set(), comp=set()))
        e['ents'].add(ent.replace('player_', '').replace('_fpp', ''))
        e['pr'].add(r['priority'])
        e['vars'].update(r.get('vars') or [])
        e['comp'].add(r['component'].split('(')[0])
filt = sys.argv[1] if len(sys.argv) > 1 else ''
for k in sorted(rows):
    if filt and filt not in k:
        continue
    e = rows[k]
    js = SC.load_set_json(k)
    ncl, rig, first = '-', '', ''
    if js:
        rig, cl = SC.clips_of(js)
        ncl = len(cl)
        first = ','.join(c['name'] for c in cl[:6])
    short = k.replace('base\\animations\\', 'b:').replace('ep1\\animations\\', 'e:')
    print('%s|%s|%s|%s|%s|%s|%s|%s' % (short, '+'.join(sorted(e['ents'])), ','.join(map(str, sorted(e['pr']))), ','.join(sorted(e['vars']))[:60],
                                    ','.join(sorted(e['comp'])), os.path.basename(rig).replace('.rig', ''), ncl, first[:90]))
