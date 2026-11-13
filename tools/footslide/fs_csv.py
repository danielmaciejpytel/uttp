"""python -I fs_csv.py <out.csv> : per-clip before/after table from Build/work/turnslide/fix_<rig>.txt (gate verdicts) and ver_<rig>.json (fs_verify re-measure
of the written clips; cm/frame = mean / max horizontal contact-point displacement per 30 fps frame over planted frame pairs)."""
import csv
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402
ROOT = os.path.join(project_root(), 'Build', 'work', 'turnslide')
rows = []
for g in ('woman', 'man'):
    ver = {r[0]: r for r in json.load(open(os.path.join(ROOT, 'ver_%s.json' % g)))}
    for ln in open(os.path.join(ROOT, 'fix_%s.txt' % g)):
        m = re.match(r'(\S+)\s+(\S+(?: \S+)?)\s+worst\s+([-\d.]+)->\s*([-\d.]+) path\s+([-\d.]+)->\s*([-\d.]+) off\s+([-\d.]+) cm', ln)
        if not m:
            continue
        n, verdict = m.group(1), m.group(2)
        if verdict.startswith('fail:worst<') or verdict == 'no plant':
            continue
        v = ver.get(n)
        f = lambda i, d=1: ('%.*f' % (d, v[i])) if v else ''
        rows.append([g, n, verdict, 'written' if v else 'not written', m.group(3), f(3), m.group(5), f(5), f(6, 3), f(7, 3), f(8, 2), f(9, 2), m.group(7)])
with open(sys.argv[1], 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['rig', 'clip', 'gate', 'status', 'worst_plant_cm_src', 'worst_plant_cm_pinned', 'path_cm_src', 'path_cm_pinned',
                'mean_cm_per_frame_src', 'mean_cm_per_frame_pinned', 'max_cm_per_frame_src', 'max_cm_per_frame_pinned', 'max_ankle_offset_cm'])
    w.writerows(rows)
print(len(rows), 'rows')
