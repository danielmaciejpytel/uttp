"""python -I fs_contacts.py <woman|man> <clipdir> : contact-curve vs geometry disagreement over a whole clip dir (frames with contact>=0.5 while heel AND toe are >5 cm
above their ground level = 'false contact'). The runtime FootLocker reads these curves (FeedReplayMotion.cpp FlFrame: db.Contact)."""
import os, re, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L
g, d = sys.argv[1:3]
rows = []
for f in sorted(os.listdir(d)):
    if not f.endswith('.utppclip.json') or f.startswith('AO_') or '_AO_' in f:
        continue
    c = L.Clip(os.path.join(d, f), g)
    if 'contact_l' not in c.c:
        continue
    gh, gt = L.ground_levels(c)
    n = 0
    for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
        a, h, t = c.foot_points(side)
        n += int(((c.c[key] >= 0.5) & (h[:, 2] > gh + 0.05) & (t[:, 2] > gt + 0.05)).sum())
    rows.append((n, c.N, f[:-14]))
bad = [r for r in rows if r[0] >= 3]
print('%s: %d clips, %d with >=3 false-contact frames, %d with >=10; worst:' % (g, len(rows), len(bad), sum(r[0] >= 10 for r in rows)))
for r in sorted(rows, reverse=True)[:8]:
    print('  %3d/%3d %s' % r)
