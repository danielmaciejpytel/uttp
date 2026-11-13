"""Screening of ONE candidate clip of a car entry row against its baseline (packed-plus set): roof, body shell per capsule, door phases per capsule at the SHIPPED close time,
the checks a..n of check_car_entry.py. No WolvenKit round trip, no approach sweep (those stay in build_car.ps1).

  python -I row_compare.py --row utpp_car_pwa_sport1_passenger_r --new <candidate.anims.glb> [--sources <sources.txt>] [--build-report <json>] [--json out.json]

Regression rule (per capsule, phase): the candidate is worse than the baseline by > --tol-cm (default 0.3) AND beyond that capsule's gate. Door phases are judged at
door_timing.json's shipped close (vanilla + redscript delta, G.seat_delta); the gate of each phase is the one of check_door_clearance (open: max(2, seated-vs-open + 0.5)).
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import check_door_clearance as C  # noqa: E402
import gen_door_timing as G  # noqa: E402
import roof_table as RT  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

SRC = os.path.join(project_root(), r'Build\work\vehicle-entry-car\roof-pack\sources-final-B.txt')
GLB = RT.GLB
CACHE = os.environ.get('UTPP_ROW_CACHE', os.path.join(project_root(), r'Build\work\vehicle-entry-car\roof-pack3\basecache'))  # baseline cache; env UTPP_ROW_CACHE overrides (round 2 used roof-pack2\basecache)


def door_phases(clip, g, cls, seat, open_tol=2.0, close_after=None, body_tol=None, roof_tol=None):
    tj = C.load_timing()
    m = C.Model(clip, cls, seat, rear_ref=os.path.join(GLB, 'p%sa_%s_%s.anims.glb' % (g, cls, seat)))
    m.body_tol_cm, m.roof_tol_cm = body_tol, roof_tol
    ca = close_after
    if ca is None:
        ca = G.vanilla_close_after(tj, cls, seat) + G.seat_delta(tj, cls, seat)
    r = C.analyse(m, ca, None, tol_cm=1.0, open_tol_cm=open_tol)
    out = {}
    for ph, v in r['phases'].items():
        out[ph] = dict(clear=v['clearance_cm'], ok=bool(v['ok']), pen=v['max_pen_cm'], gate=v['gate_cm'], worst=v.get('capsule'),
                       by={k: (round(x['max_pen_cm'], 2), round(x['gate_cm'], 2)) for k, x in v['by_capsule'].items()})
    return ca, out


def screen(row, clip, build_report=None, sources=SRC, tol=0.3, body_tol=None, roof_tol=None, open_tol=2.0, close_after=None):
    srcs = RT.read_sources(sources)
    g, cls, seat = RT.parse(row)
    base_run = srcs[row]
    base = os.path.join(base_run, 'roundtrip', row + '.anims.glb')
    res = dict(row=row, clip=clip)
    cache = os.path.join(CACHE, '%s.%s.json' % (row, 'x' if body_tol is None else str(body_tol)))
    for tag, c, run in (('base', base, base_run), ('new', clip, base_run)):
        if tag == 'base' and os.path.exists(cache):
            res['base'] = json.load(open(cache))
            continue
        st = RT.static(c, cls, seat)
        ck = RT.checks(c, row, run if (tag == 'base' or not build_report) else os.path.dirname(os.path.dirname(build_report))) if True else None
        ca, dp = door_phases(c, g, cls, seat, open_tol, close_after, body_tol, roof_tol)
        res[tag] = dict(roof_pen=round(st['roof_pen'], 2), roof_gate=round(st['roof_gate'], 2), body=st['body'], body_gate=st['body_gate'], checks=ck, door=dp, close_after=ca)
        if tag == 'base':
            os.makedirs(CACHE, exist_ok=True)
            json.dump(res['base'], open(cache, 'w'), default=float)
    reg = []
    for k, v in res['new']['body'].items():
        b = res['base']['body'][k]
        gate = max(res['new']['body_gate'][k], body_tol if body_tol is not None else 0.0)
        if v > b + tol and v > gate:
            reg.append(('body', k, round(v, 2), round(b, 2)))
    for ph, v in res['new']['door'].items():
        bv = res['base']['door'][ph]
        for k, (pn, gt) in v['by'].items():
            pb = bv['by'][k][0]
            if pn > pb + tol and pn > gt:
                reg.append((ph, k, pn, pb))
    res['regress'] = reg
    res['check_fail'] = [k for k, v in res['new']['checks'].items() if k.endswith('_ok') and not v]
    res['check_fail_base'] = [k for k, v in res['base']['checks'].items() if k.endswith('_ok') and not v]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--row', required=True)
    ap.add_argument('--new', required=True)
    ap.add_argument('--sources', default=SRC)
    ap.add_argument('--build-report')
    ap.add_argument('--tol-cm', type=float, default=0.3)
    ap.add_argument('--body-tol-cm', type=float)
    ap.add_argument('--roof-tol-cm', type=float)
    ap.add_argument('--open-tol-cm', type=float, default=2.0)
    ap.add_argument('--json')
    a = ap.parse_args()
    row = a.row if a.row.startswith('utpp_car_') else 'utpp_car_' + a.row
    r = screen(row, a.new, a.build_report, a.sources, a.tol_cm, a.body_tol_cm, a.roof_tol_cm, a.open_tol_cm)
    n, b = r['new'], r['base']
    print('%s roof %.2f/%.2f (base %.2f) | close_after %.3f' % (row[9:], n['roof_pen'], n['roof_gate'], b['roof_pen'], n['close_after']))
    print('  body worst new %s base %s' % (sorted(((round(v, 2), k) for k, v in n['body'].items()), reverse=True)[:3], sorted(((round(v, 2), k) for k, v in b['body'].items()), reverse=True)[:3]))
    print('  door clear(cm) new  %s' % ' '.join('%s %.1f%s' % (p[:6], v['clear'], '' if v['ok'] else '!') for p, v in n['door'].items()))
    print('  door clear(cm) base %s' % ' '.join('%s %.1f%s' % (p[:6], v['clear'], '' if v['ok'] else '!') for p, v in b['door'].items()))
    print('  regress vs base:', r['regress'] or 'none')
    print('  checks failing new %s | base %s' % (r['check_fail'], r['check_fail_base']))
    if a.json:
        json.dump(r, open(a.json, 'w'), indent=1, default=float)
    return 0 if not r['regress'] and not r['check_fail'] and n['roof_pen'] <= n['roof_gate'] else 1


if __name__ == '__main__':
    sys.exit(main())
