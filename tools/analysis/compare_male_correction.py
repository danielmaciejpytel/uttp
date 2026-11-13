"""Before / after / female table from male_body_correction_eval.py measure outputs.

python -I compare_male_correction.py <woman.json> <man-before.json> <man-after.json> [region ...]
Per pose and region: worst-frame sev_p999 (F / M before / M after), mean S, mean extreme-edge count; summary medians and the number of regressions
(after worse than before by > 10 %).
"""
import json
import sys

import numpy as np

W, B, A = (json.load(open(p, encoding='utf-8')) for p in sys.argv[1:4])
regs = sys.argv[4:] or ['shoulder', 'elbow', 'neck', 'knee']
for reg in regs:
    print('\n### %s  (worst-frame sev_p999: F / M before / M after;  mean S: before -> after; mean extreme edges: before -> after)' % reg)
    print('| pose | family | sev F | sev M before | sev M after | S before | S after | extreme before | extreme after |\n|---|---|---|---|---|---|---|---|---|')
    sb, sa, sf, regress = [], [], [], 0
    for pw, pb, pa in zip(W['poses'], B['poses'], A['poses']):
        wb, wa, ww = pb['worst'][reg]['sev_p999'], pa['worst'][reg]['sev_p999'], pw['worst'][reg]['sev_p999']
        mb, ma = pb['mean'][reg], pa['mean'][reg]
        sb.append(wb); sa.append(wa); sf.append(ww)
        flag = ''
        if wa > wb * 1.1 or ma['S'] > mb['S'] * 1.1 or ma['extreme'] > mb['extreme'] * 1.1 + 2:
            regress += 1
            flag = ' **'
        print('| %s | %s | %.1f | %.1f | %.1f%s | %.0f | %.0f | %.0f | %.0f |' % (pb['label'], pa.get('family') or '-', ww, wb, wa, flag, mb['S'], ma['S'], mb['extreme'], ma['extreme']))
    print('median worst sev: F %.2f | M before %.2f | M after %.2f ; max: F %.2f | %.2f | %.2f ; poses with regression (** = worst sev, mean S or mean extreme-edge count more than +10%%): %d' % (
        np.median(sf), np.median(sb), np.median(sa), max(sf), max(sb), max(sa), regress))
