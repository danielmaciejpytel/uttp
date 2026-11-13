"""python -I fs_verify.py <woman|man> <src dir> <fixed dir> : independent check of written clips: everything but the leg quaternions identical,
Root/Trajectory/Hips bit-identical, and before/after contact slide re-measured from the written files (cm/frame, plant-gated)."""
import json, os, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L
g, src, fx = sys.argv[1:4]
rows = []
bad = 0
for f in sorted(os.listdir(fx)):
    a = L.Clip(os.path.join(src, f), g)
    b = L.Clip(os.path.join(fx, f), g)
    legs = {a.bi[s + n] for s in ('Left', 'Right') for n in ('UpLeg', 'Leg', 'Foot')}
    ok = np.array_equal(a.B[..., 0:3], b.B[..., 0:3]) and np.array_equal(a.B[..., 7:10], b.B[..., 7:10])
    for j in range(a.B.shape[1]):
        if j not in legs and not np.array_equal(a.B[:, j, 3:7], b.B[:, j, 3:7]):
            ok = False
    ok = ok and all(a.d[k] == b.d[k] for k in ('curves', 'notifies', 'bone_names', 'fps', 'num_frames', 'source_clip', 'rig'))
    for j in (0, 1, 2):
        ok = ok and np.array_equal(a.B[:, j], b.B[:, j])
    bad += (not ok)
    ma, mb = L.measure(a), L.measure(b)
    rows.append((f[:-14], ok, max(ma['Left']['worst_seg'], ma['Right']['worst_seg']), max(mb['Left']['worst_seg'], mb['Right']['worst_seg']),
                 ma['Left']['path'] + ma['Right']['path'], mb['Left']['path'] + mb['Right']['path'],
                 np.mean([ma[s]['mean'] for s in 'LR' if False] or [ (ma['Left']['mean']+ma['Right']['mean'])/2 ]),
                 (mb['Left']['mean']+mb['Right']['mean'])/2, max(ma['Left']['max'], ma['Right']['max']), max(mb['Left']['max'], mb['Right']['max'])))
json.dump(rows, open(sys.argv[4] if len(sys.argv) > 4 else os.devnull, 'w'))
print('clips', len(rows), 'identity violations', bad)
print('sum worst %.1f -> %.1f cm; sum path %.1f -> %.1f cm' % (sum(r[2] for r in rows), sum(r[3] for r in rows), sum(r[4] for r in rows), sum(r[5] for r in rows)))
