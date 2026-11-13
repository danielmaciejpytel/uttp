"""Stage 4 tables: survey_clips.csv / survey_sets.csv (+ ranking block in report.txt) from metrics_<rig>.json (+ skin_<rig>.json).

  python -I survey_tables.py --rig woman [--no-report]
"""
import collections
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

RIG = sys.argv[sys.argv.index('--rig') + 1] if '--rig' in sys.argv else 'woman'


def main():
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    sp = os.path.join(SC.WORK, 'skin_%s.json' % RIG)
    skin = json.load(open(sp, encoding='utf-8')) if os.path.exists(sp) else {}
    tri = {r['setkey']: r for r in csv.DictReader(open(os.path.join(SC.WORK, 'triage_sets.csv'), encoding='utf-8'))}
    rows = []
    for k, m in met.items():
        s = skin.get(k, {})
        reload_fix = 'partly (native reload contacts, FkHook.cpp:59-96)' if (m['mode'] == 'action' and 'reload' in m['clip']) else ''
        rows.append(dict(body=RIG, set=m['set'], setkey=m['setkey'], clip=m['clip'], cls=m['cls'], state=m['state'], shown=m['mode'],
                         frames=m['frames'], fail_pct=m['fail_pct'], worst_feature=m['worst_feature'], worst_excess=m['worst_excess'],
                         worst_frame=m['worst_frame'], severity=m['severity'], fail_features=' '.join('%s:%d' % kv for kv in m['fail_features'].items()),
                         trans_fail_pct=m['trans_fail_pct'], trans_excess=' '.join('%s:%s' % kv for kv in m['trans_excess'].items()),
                         rot_step_excess=m['rot_step_max'], glitch_frames=m.get('glitch_frames', 0), degenerate_export=m.get('degenerate_export', False),
                         plausibility=' '.join('%s:%d' % kv for kv in m['plaus_frames'].items()), fpp_cam_animated=m['fpp_cam_animated'],
                         fpp_set=m['fpp_set'], vanilla_tpp_camera_look=m['tpp_camera_look'],
                         skin_frames=' '.join(map(str, s.get('frames', []))), skin_excess_max=s.get('skin_excess_max', ''),
                         skin_region=s.get('skin_region', ''), skin_frame=s.get('skin_frame', ''), skin_confirmed=s.get('confirmed', ''),
                         already_corrected=reload_fix))
    rows.sort(key=lambda r: (r['cls'] not in ('A', 'B', 'E'), r['degenerate_export'], -float(r['severity'])))
    cp = os.path.join(SC.WORK, 'survey_clips_%s.csv' % RIG)
    with open(cp, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    sets = collections.OrderedDict()
    for r in rows:
        sets.setdefault(r['setkey'], []).append(r)
    srows = []
    for sk, rs in sets.items():
        ok = [r for r in rs if not r['degenerate_export']]
        vis = [r for r in ok if r['cls'] in ('A', 'B', 'E')]
        feats = collections.Counter()
        for r in vis:
            for kv in r['fail_features'].split():
                feats[kv.split(':')[0]] += int(kv.split(':')[1])
        worst = max(vis, key=lambda r: float(r['severity'])) if vis else None
        sk_conf = [r for r in vis if r['skin_confirmed'] is True]
        sk_test = [r for r in vis if r['skin_confirmed'] in (True, False)]
        t = tri.get(sk, {})
        srows.append(dict(body=RIG, setkey=sk, set=rs[0]['set'], cls=t.get('cls', ''), state=t.get('state', ''), shown=rs[0]['shown'],
                          clips_measured=len(rs), visible_clips=len(vis), degenerate_clips=len(rs) - len(ok), glitch_clips=sum(1 for r in ok if r['glitch_frames']),
                          clips_failing=sum(1 for r in vis if float(r['fail_pct']) > 0), mean_fail_pct=round(sum(float(r['fail_pct']) for r in vis) / max(len(vis), 1), 1),
                          max_severity=worst['severity'] if worst else 0, worst_clip=worst['clip'] if worst else '',
                          worst_feature=worst['worst_feature'] if worst else '', dominant_features=' '.join(k for k, _ in feats.most_common(4)),
                          trans_clips=sum(1 for r in vis if float(r['trans_fail_pct']) > 0),
                          skin_tested=len(sk_test), skin_confirmed=len(sk_conf), fpp_authored=any(r['fpp_set'] for r in rs),
                          vanilla_tpp_camera_look=rs[0]['vanilla_tpp_camera_look'],
                          already_corrected_at_runtime='reload clips partly (contacts only)' if any(r['already_corrected'] for r in rs) else 'no'))
    srows.sort(key=lambda r: -float(r['max_severity']))
    spp = os.path.join(SC.WORK, 'survey_sets_%s.csv' % RIG)
    with open(spp, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(srows[0].keys()))
        w.writeheader()
        w.writerows(srows)
    # report block
    vis = [r for r in rows if r['cls'] in ('A', 'B', 'E') and not r['degenerate_export']]
    lines = ['%s: %d clips measured (%d visible A/B/E, %d degenerate exports excluded, %d clips with glitch frames excluded from their stats)' % (
        RIG, len(rows), len(vis), sum(1 for r in rows if r['degenerate_export']), sum(1 for r in rows if r['glitch_frames']))]
    lines.append('visible clips failing the deformation gate on >= 1 frame: %d / %d; on >= 50%% of frames: %d' % (
        sum(1 for r in vis if float(r['fail_pct']) > 0), len(vis), sum(1 for r in vis if float(r['fail_pct']) >= 50)))
    lines.append('visible clips with translation stretch (seg_len / clavicle shift beyond the rigid GASP bones): %d' % sum(1 for r in vis if float(r['trans_fail_pct']) > 0))
    fc = collections.Counter()
    for r in vis:
        for kv in r['fail_features'].split():
            fc[kv.split(':')[0]] += 1
    lines.append('dominant failing features (clips): ' + ', '.join('%s %d' % kv for kv in fc.most_common(10)))
    if skin:
        st = [r for r in vis if r['skin_confirmed'] in (True, False)]
        lines.append('skinned confirmation (worst frames +-1 of the most severe visible clips): %d tested, %d confirmed (strain over skin_ref + margin, hands excluded)' % (
            len(st), sum(1 for r in st if r['skin_confirmed'] is True)))
    lines.append('top visible deforming SETS (max severity = p90 of the normalised exceedance; 1.0 = 10 deg / 2 cm / 2.5 %% beyond the base-body band):')
    for r in [r for r in srows if r['cls'] in ('A', 'B', 'E') and r['visible_clips']][:25]:
        lines.append('  %-6s %s %-45s sev %5.2f fail clips %3d/%-3d skin %d/%d  worst %s (%s)  dominant: %s%s%s' % (
            r['cls'], RIG[0], r['setkey'][:45], float(r['max_severity']), r['clips_failing'], r['visible_clips'], r['skin_confirmed'], r['skin_tested'],
            r['worst_clip'], r['worst_feature'], r['dominant_features'], '  [vanilla TPP-camera set]' if r['vanilla_tpp_camera_look'] else '',
            '  [reload partly corrected]' if r['already_corrected_at_runtime'] != 'no' else ''))
    txt = '\n'.join(lines)
    print(txt)
    if '--no-report' not in sys.argv:
        SC.report('STAGE 4 METRICS %s (survey_clips_%s.csv, survey_sets_%s.csv)' % (RIG.upper(), RIG, RIG), txt)


if __name__ == '__main__':
    main()
