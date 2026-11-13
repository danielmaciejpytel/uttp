"""Female vs male comparison tables from measure_body_deformation.py output.

python -I rank_body_deformation.py <measure-woman.json> <measure-man.json> [<dump dir> <label,label,...>]
Prints markdown: REF control row, male ranking by severity (sev_p999), male/female ratio ranking, and (with a dump dir) the bone
attribution of the extreme edges of each dumped case (female and male).
"""
import collections
import json
import sys

import numpy as np

REG = ['shoulder', 'elbow', 'neck', 'knee']


def main():
    W, M = (json.load(open(p, encoding='utf-8')) for p in sys.argv[1:3])
    rows = []
    for pw, pm in zip(W['poses'], M['poses']):
        assert pw['label'] == pm['label']
        for reg in REG:
            a, b = pw['worst'][reg], pm['worst'][reg]
            rows.append(dict(pose=pm['label'], clip=pm['clip'], reg=reg, f=a['sev_p999'], m=b['sev_p999'], x=b['sev_p999'] / a['sev_p999'],
                             mf=b['frame'], ff=a['frame'], mp99=b['edge_p99'], fp99=a['edge_p99'], mmax=b['edge_max'], fmax=a['edge_max'],
                             marea=b['area_min'], farea=a['area_min'], mb=b['sev_bind'], fb=a['sev_bind'], mcm=b['stretch_cm_max'], fcm=a['stretch_cm_max'], mcc=b['compress_cm_max'], fcc=a['compress_cm_max']))
    print('REF (bind-relative, rig reference pose; sev_p999): female %s | male %s' % (
        {k: round(W['ref'][k]['sev_p999'], 2) for k in REG}, {k: round(M['ref'][k]['sev_p999'], 2) for k in REG}))
    hdr = '| pose | region | male sev | female sev | M/F | male frame | male edge p99 | female edge p99 | male area_min | female area_min | male sev_bind | female sev_bind |\n|---|---|---|---|---|---|---|---|---|---|---|---|'
    fmt = lambda r: '| %s | %s | %.2f | %.2f | %.2f | %d | %.2f | %.2f | %s | %s | %.1f | %.1f |' % (  # noqa: E731
        r['pose'], r['reg'], r['m'], r['f'], r['x'], r['mf'], r['mp99'], r['fp99'],
        '%.2f' % r['marea'] if r['marea'] is not None else '-', '%.2f' % r['farea'] if r['farea'] is not None else '-', r['mb'], r['fb'])
    print('\n### Male worst by severity\n' + hdr)
    for r in sorted(rows, key=lambda r: -r['m'])[:15]:
        print(fmt(r))
    print('\n### Male vs female (male sev >= 3), by M/F\n' + hdr)
    for r in sorted([r for r in rows if r['m'] >= 3], key=lambda r: -r['x'])[:15]:
        print(fmt(r))
    print('\n### Male vs female by bind-relative severity (robust where the REF pose itself is collapsed, e.g. elbow)\n' + hdr)
    for r in sorted([r for r in rows if r['mb'] >= 3], key=lambda r: -r['mb'] / r['fb'])[:12]:
        print(fmt(r))
    print('\n### Absolute edge stretch / compression vs REF in cm (max over region edges at the sev_p999 frame), male worst by stretch')
    print('| pose | region | male stretch cm | female stretch cm | male compress cm | female compress cm |\n|---|---|---|---|---|---|')
    for r in sorted(rows, key=lambda r: -r['mcm'])[:12]:
        print('| %s | %s | %.2f | %.2f | %.2f | %.2f |' % (r['pose'], r['reg'], r['mcm'], r['fcm'], r['mcc'], r['fcc']))
    print('\n### Per region medians over all poses (sev_p999)')
    for reg in REG:
        f = [r['f'] for r in rows if r['reg'] == reg]
        m = [r['m'] for r in rows if r['reg'] == reg]
        print('- %s: female median %.2f max %.2f | male median %.2f max %.2f | male>female in %d of %d poses' % (
            reg, np.median(f), max(f), np.median(m), max(m), sum(1 for r in rows if r['reg'] == reg and r['x'] > 1.0), len(f)))
    if len(sys.argv) > 4:
        d, labels = sys.argv[3], sys.argv[4].split(',')
        print('\n### Bone attribution of extreme edges (REF-relative ratio > 1.5 or < 0.5), by dominant joint of the endpoints')
        for lab in labels:
            for body in ('woman', 'man'):
                z = np.load('%s/%s_%s.npz' % (d, body, lab))
                e, r, dom = z['edges'], z['edge_ratio'], z['domname']
                for reg in REG:
                    m = z['mask_' + reg]
                    em = m[e[:, 0]] & m[e[:, 1]]
                    sel = em & ((r > 1.5) | (r < 0.5))
                    c = collections.Counter()
                    for k in np.where(sel)[0]:
                        a, b = dom[e[k, 0]], dom[e[k, 1]]
                        c[a if a == b else ' / '.join(sorted((a, b)))] += 1
                    top = ', '.join('%s x%d' % (n.replace('_JNT', ''), v) for n, v in c.most_common(3))
                    print('- %s %s %s: %d extreme edges of %d (max %.2f, min %.2f)%s' % (
                        lab, body, reg, sel.sum(), em.sum(), r[em].max(), r[em].min(), ('; ' + top) if top else ''))


if __name__ == '__main__':
    main()
