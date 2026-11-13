"""Roof / body shell penetration of car entry clips (static phases of check_door_clearance.py, no door timeline, no WolvenKit round trip needed).

  python -I roof_report.py --class sport1 --seat driver <clip.anims.glb> [<clip2> ...] [--json out.json] [--roof-tol-cm X] [--body-tol-cm X]

Per clip: roof (head sphere vs ceiling) penetration cm, its ABSOLUTE gate (max(1 cm, seated head penetration + 0.5 cm) = the roof model limit of the seated pose itself),
body-shell worst capsule. 'ok' uses the absolute gates unless a waiver cm is passed. Exit 0 when every clip passes both gates.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as C  # noqa: E402


def report(clip, cls, seat, body_tol=None, roof_tol=None):
    m = C.Model(clip, cls, seat)
    m.body_tol_cm, m.roof_tol_cm = body_tol, roof_tol
    if m.bodyd is None:
        return dict(clip=clip, error='no body-data for %s' % cls, ok=False)
    out = dict(clip=os.path.basename(clip))
    ok = True
    for ph in ('roof', 'body'):
        v_ok, who, pen, gate, cannot = C.static_verdict(m, ph)
        out[ph] = dict(ok=v_ok, worst=who, pen_cm=round(pen, 2), gate_cm=round(gate, 2), ungated=cannot)
        ok &= v_ok
    sb, sr = m.static_series()[3], m.static_series()[4]
    out['seated_roof_pen_cm'] = round(float(sr) * 100, 2)
    out['ok'] = bool(ok)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--seat', required=True, choices=['driver', 'passenger_r'])
    ap.add_argument('--body-tol-cm', type=float)
    ap.add_argument('--roof-tol-cm', type=float)
    ap.add_argument('--json')
    ap.add_argument('clips', nargs='+')
    a = ap.parse_args()
    res = []
    for c in a.clips:
        r = report(c, a.cls, a.seat, a.body_tol_cm, a.roof_tol_cm)
        res.append(r)
        if 'error' in r:
            print('%s: %s' % (os.path.basename(c), r['error']))
            continue
        print('%-38s roof %6.2f/%5.2f cm %s | body %6.2f/%5.2f cm (%s) %s | seated roof %.2f' % (
            r['clip'][:-10], r['roof']['pen_cm'], r['roof']['gate_cm'], 'ok' if r['roof']['ok'] else 'FAIL', r['body']['pen_cm'], r['body']['gate_cm'],
            r['body']['worst'], 'ok' if r['body']['ok'] else 'FAIL', r['seated_roof_pen_cm']), flush=True)
    if a.json:
        json.dump(res, open(a.json, 'w'), indent=1)
    return 0 if all(r.get('ok') for r in res) else 1


if __name__ == '__main__':
    sys.exit(main())
