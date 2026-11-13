"""Print clip names (non-additive marked '*', frames) of the given sets (substring match on the depot path)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402

doc = json.load(open(SC.SETS_JSON, encoding='utf-8'))
paths = sorted({r['set'].lower() for lst in doc.values() for r in lst})
for pat in sys.argv[1:]:
    for p in paths:
        if pat in p:
            rig, cl = SC.clips_of(SC.load_set_json(p))
            add = sum(1 for c in cl if c['type'] in ('Additive', 'AdditiveWithoutFirstFrame'))
            print('## %s (%d clips, %d additive)' % (p, len(cl), add))
            print(' '.join(('%s:%d' % (c['name'], c['frames'])) for c in cl if c['type'] not in ('Additive', 'AdditiveWithoutFirstFrame')))
