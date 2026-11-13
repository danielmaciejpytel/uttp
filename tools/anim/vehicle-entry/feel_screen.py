"""Candidate screen of the driver-side entry feel (r13d): startfwd_eval.build (build_car.ps1 -BuilderOnly, -DoorOverride / -ExtraArgs / -DuckExtra) + entry_feel E1-E6 + the
start gate + the door phases at the solved close (+ optionally the approach gate relative to the shipped set). One line per candidate.

  python -I feel_screen.py --class standard3 --gender m --door 0.7,0.9,0.3,0.7 [--extra=--hips-path,0.02,0.01,2] [--duck-extra=...] [--approach] [--tag name]

The row's body / roof waivers are NOT applied (raw numbers, as startfwd_eval). The glb is the builder GLB (no WolvenKit round trip): the final gates run in build_car.ps1.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import startfwd_eval as SE  # noqa: E402
import entry_feel as EF  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

SHIPPED = os.path.join(project_root(), r'Build\work\vehicle-entry-car\pk-door7-turbo-anims2\sources.txt')
OUT_ROOT = os.path.join(project_root(), r'Build\work\vehicle-entry-car\r13\r13d\scr')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--gender', default='w')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--door', required=True)
    ap.add_argument('--extra', action='append', default=[])
    ap.add_argument('--duck-extra', action='append', default=[])
    ap.add_argument('--no-roof', action='store_true')
    ap.add_argument('--no-duck', action='store_true')
    ap.add_argument('--approach', action='store_true')
    ap.add_argument('--full', action='store_true')
    ap.add_argument('--baseline-sources', default=SHIPPED)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--out-root', default=OUT_ROOT)
    a = ap.parse_args()
    os.makedirs(a.out_root, exist_ok=True)
    out, glb, result, stdout = SE.build(a, a.tag)
    import check_door_clearance as C
    import start_gate as SG
    g, cls, seat = a.gender, a.cls, a.seat
    van = os.path.join(os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\glb'), 'p%sa_%s_%s.anims.glb' % (g, cls, seat))
    sg = SG.evaluate(glb, cls, seat, van)
    m = C.Model(glb, cls, seat, rear_ref=van)
    sv = C.solve(m)
    ca = sv.get('close_after_s')
    r = C.analyse(m, ca) if ca else None
    ph = (r or {}).get('phases', {})
    s = ' '.join('%s %.1f%s' % (p[:5], v['clearance_cm'], '' if v['ok'] else '!') for p, v in ph.items())
    st = sg['start']
    ef = EF.evaluate(glb, cls, seat)
    print(a.tag, '|', EF.fmt(ef).split('|', 0)[0][35:], flush=True)
    print('   start dy %+.2f (vs %s) slide %.2f %s | rear %s | close_after %s | %s | %s' % (st['dy_vs_reference_m'], st['reference'], st['slide_m'], 'ok' if st['ok'] else 'FAIL', 'ok' if sg['rearward']['ok'] else 'FAIL', ca, s, result), flush=True)
    if a.approach:
        print('   ' + SE.approach(a, glb, cls, seat, g, van, out), flush=True)
    json.dump(ef, open(os.path.join(a.out_root, a.tag + '.feel.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
