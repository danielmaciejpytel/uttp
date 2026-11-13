"""python -I fs_audit.py <woman|man> <clipdir[;clipdir]> <out.json>: per-clip (a) ankle slide in planted frames vs the source pack footspeed curve (same mask),
(b) contact-curve vs geometry disagreement, (c) worst single-plant drift. Families: Turn/Pivot/Reface/Idle turn."""
import json, os, re, sys
import numpy as np
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L
g, dirs, out = sys.argv[1], sys.argv[2], sys.argv[3]
rx = re.compile(r'(_Turn_|_Pivot_|_Reface_|Idle_turn|Idle_Turn)')
res = {}
for d in dirs.split(';'):
    for f in sorted(os.listdir(d)):
        if not f.endswith('.utppclip.json'):
            continue
        n = f[:-14]
        if not rx.search(n) or n.startswith('AO_'):
            continue
        c = L.Clip(os.path.join(d, f), g)
        gh, gt = L.ground_levels(c)
        hr = float(c.d['retarget'].get('hip_ratio', 1.0))
        r = dict(N=c.N, hip_ratio=hr)
        fa, fm, sa, ratio_num, ratio_den = 0, 0, 0, 0.0, 0.0
        worst, tot = 0.0, 0.0
        for side, fk in (('Left', 'footspeed_l'), ('Right', 'footspeed_r')):
            a, h, t = c.foot_points(side)
            pl = L.planted(c, side, gh=gh, gt=gt)
            pair = pl[:-1] & pl[1:]
            ank = np.linalg.norm(np.diff(a[:, :2], axis=0), axis=1) * 100
            fs = np.asarray(c.d['curves'].get(fk, np.zeros(c.N)), dtype=float) / L.FPS
            ratio_num += ank[pair].sum() / hr
            ratio_den += 0.5 * (fs[:-1] + fs[1:])[pair].sum()
            s = L.contact_point_slide(a, h, t)
            for (b, e) in L.segments(pair):
                worst = max(worst, float(s[b:e + 1].sum()))
            tot += float(s[pair].sum())
            geo = (h[:, 2] < gh + L.PLANT_H) | (t[:, 2] < gt + L.PLANT_H)
            cm = c.c['contact_l' if side == 'Left' else 'contact_r'] >= 0.5
            fa += int((cm & ~(h[:, 2] < gh + 0.05) & ~(t[:, 2] < gt + 0.05)).sum())   # contact flag but foot clearly in the air
            fm += int((~cm & geo & (np.r_[0, np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1)] * 100 < 0.5)).sum())  # planted+still but flag off
        r.update(false_contact=fa, missed_contact=fm, ank_over_src=(ratio_num / ratio_den if ratio_den > 0 else None), worst_seg_cm=worst, path_cm=tot)
        res[n] = r
json.dump(res, open(out, 'w'))
rows = sorted(res.items(), key=lambda kv: -kv[1]['worst_seg_cm'])
print(len(res), 'clips; ank/src (median over clips with data): %.2f' % np.median([v['ank_over_src'] for v in res.values() if v['ank_over_src']]))
print('clips with false-contact frames: %d, with missed-contact frames: %d' % (sum(v['false_contact'] > 0 for v in res.values()), sum(v['missed_contact'] > 2 for v in res.values())))
for n, v in rows[:15]:
    print('%-44s worst %5.1f cm path %5.1f ank/src %s false %d missed %d' % (n, v['worst_seg_cm'], v['path_cm'], '%.2f' % v['ank_over_src'] if v['ank_over_src'] else '-', v['false_contact'], v['missed_contact']))
