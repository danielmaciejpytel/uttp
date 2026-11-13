"""python -I fs_measure.py <woman|man> <clipdir[;clipdir]> <out.json> [regex]  -> per-clip contact slide table (cm/frame at 30 fps)."""
import json, os, re, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L

g, dirs, out = sys.argv[1], sys.argv[2], sys.argv[3]
rx = re.compile(sys.argv[4] if len(sys.argv) > 4 else r'(Turn|Pivot|Reface|turn_|Walk|Run|Crouch|Sprint)')
rows = {}
for d in dirs.split(';'):
    for f in sorted(os.listdir(d)):
        if not f.endswith('.utppclip.json'):
            continue
        n = f[:-len('.utppclip.json')]
        if not rx.search(n) or n.startswith('AO_') or '_AO_' in n:
            continue
        c = L.Clip(os.path.join(d, f), g)
        if 'contact_l' not in c.c:
            continue
        m = L.measure(c)
        s = L.source_footspeed(c)
        rows[n] = dict(N=c.N, L=m['Left'], R=m['Right'], srcL=s['Left'], srcR=s['Right'])
json.dump(rows, open(out, 'w'))
print(len(rows), 'clips')
