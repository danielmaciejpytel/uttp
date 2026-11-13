"""python -I fs_split.py <woman|man> <replay_out.txt> <clipdir[;clipdir...]> : runtime (replay FLCLIP pre) vs intrinsic clip toe speed in
the same contact segments, per key and for the worst clips. Intrinsic = clip contact segments, 30 fps, trim 2 each end."""
import os, re, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L
g, rep, dirs = sys.argv[1], sys.argv[2], sys.argv[3]
paths = {}
for d in dirs.split(';'):
    for f in os.listdir(d):
        if f.endswith('.utppclip.json'):
            paths.setdefault(f[:-14], os.path.join(d, f))
rows = []
for ln in open(rep):
    f = ln.split()
    if len(f) == 6 and f[0] == 'FLCLIP' and f[1] != 'clip':
        rows.append((f[1], int(f[2]), float(f[3]), float(f[4]), float(f[5])))
def key(c):
    has = lambda s: s in c
    gait = 'Sprint' if has('Sprint') else 'Crouch' if has('Crouch') else 'Run' if has('Run_') else 'Walk' if has('Walk') else 'Idle' if has('Idle') else 'Trans' if has('Transition') else 'Other'
    typ = 'Reface' if has('Reface') else 'Pivot' if has('Pivot') else 'TIP' if has('TurnInPlace') else 'Start' if has('_Start') else 'Stop' if has('_Stop') else 'Turn' if has('_Turn_') else 'Loop' if has('Loop') else 'Idle' if has('Idle') else 'Trans' if has('Transition') else 'Other'
    return gait + '/' + typ
agg = {}
det = []
for name, n, pre, p50, post in rows:
    if name not in paths:
        continue
    c = L.Clip(paths[name], g)
    if 'contact_l' not in c.c:
        continue
    ni, mi, _ = L.toe_contact_speed_trimmed(c)
    k = key(name)
    a = agg.setdefault(k, [0, 0.0, 0.0, 0.0, 0, 0.0])
    if ni == 0:
        a[0] += 0; a[4] += n   # no intrinsic data (contact segments too short): excluded from both sides of the ratio
        continue
    a[0] += n; a[1] += n * pre; a[2] += n * mi; a[3] += n * p50
    det.append((n * (pre - mi), name, n, pre, p50, mi))
print('%-14s %6s %9s %9s %9s %9s' % ('key', 'n', 'runtime', 'rt_p50', 'intrinsic', 'excess'))
for k, a in sorted(agg.items()):
    n = a[0]
    if n == 0:
        print('%-14s   no intrinsic data (%d replay samples)' % (k, a[4]))
        continue
    print('%-14s %6d %9.1f %9.1f %9.1f %9.1f   (no-data samples %d)' % (k, n, a[1] / n, a[3] / n, a[2] / n, (a[1] - a[2]) / n, a[4]))
tn = sum(a[0] for a in agg.values())
print('ALL n=%d runtime %.1f intrinsic %.1f' % (tn, sum(a[1] for a in agg.values()) / tn, sum(a[2] for a in agg.values()) / tn))
print('top clips by (runtime-intrinsic)*n:')
for x in sorted(det, reverse=True)[:12]:
    print('  %-45s n=%4d runtime %7.1f p50 %7.1f intrinsic %6.1f' % (x[1], x[2], x[3], x[4], x[5]))
