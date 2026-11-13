"""Fast gate screen of a builder / round-tripped clip (no WolvenKit round trip, no approach gate):

  python -I quick_eval.py --clip <glb> [--van auto] [--close-after S | --solve] [--tag name]

One line: start gate (dy / slide), rearward rows (root / Hips / Spine / sink slide against the game's clip), joint rule, door phases at the close time (clearance cm,
'!' = over its gate; body / roof / jamb are ABSOLUTE gates without waivers), plus the capsules over their gate in the jamb / body / roof phases. Same models as the build gate.
"""
import argparse
import os
import sys
import time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
EX = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\glb')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clip', required=True)
    ap.add_argument('--van', default='auto')
    ap.add_argument('--close-after', type=float, default=None)
    ap.add_argument('--solve', action='store_true')
    ap.add_argument('--tag', default=None)
    ap.add_argument('--gate-detail', action='store_true')
    a = ap.parse_args()
    import check_door_clearance as C
    import start_gate as SG
    g, cls, seat = C.parse_name(a.clip)
    van = os.path.join(EX, 'p%sa_%s_%s.anims.glb' % (g, cls, seat)) if a.van == 'auto' else a.van
    t0 = time.time()
    m = C.Model(a.clip, cls, seat, rear_ref=van)
    ca = a.close_after
    if a.solve or ca is None:
        sv = C.solve(m)
        ca = sv.get('close_after_s')
    r = C.analyse(m, ca) if ca else None
    ph = (r or {}).get('phases', {})
    s = ' '.join('%s %.1f%s' % (p[:5], v['clearance_cm'], '' if v['ok'] else '!') for p, v in ph.items())
    sg = SG.evaluate(a.clip, cls, seat, van, exempt=False)
    st = sg['start']
    rw = sg['rearward']['rows']
    print('%s | start dy %+.2f slide %.2f %s | rear %s root %.2f/%.2f H %.2f/%.2f S %.2f/%.2f sink %.2f/%.2f | joints %s | close_after %s | %s | %.0fs' % (
        a.tag or os.path.basename(a.clip), st['dy_vs_vanilla_m'], st['slide_m'], 'ok' if st['ok'] else 'FAIL', 'ok' if sg['rearward']['ok'] else 'FAIL',
        rw['root']['excursion_m'], rw['root']['game_m'], rw['Hips']['excursion_m'], rw['Hips']['game_m'], rw['Spine']['excursion_m'], rw['Spine']['game_m'],
        rw['sink_slide_Hips']['excursion_m'], rw['sink_slide_Hips']['game_m'], 'ok' if sg['joint_rule']['ok'] else 'FAIL', ca, s, time.time() - t0), flush=True)
    if a.gate_detail:
        for pn, v in ph.items():
            if pn == 'by_capsule' or v.get('ok', True) and pn not in ('jamb',):
                continue
            bc = v.get('by_capsule', {})
            print('  phase', pn, 'worst %.2f %s | over gate:' % (v['clearance_cm'], v['capsule']), ' '.join('%s %.2f/g%.2f@%.2f' % (c, x['max_pen_cm'], x['gate_cm'], x['t_clip_s']) for c, x in bc.items() if not x['ok']))


if __name__ == '__main__':
    main()
