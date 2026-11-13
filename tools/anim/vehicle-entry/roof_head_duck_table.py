"""Per-row decision table of the roofpack round 2 (head duck layer): 24 rows, new set vs the packed-plus set (round 1) and the roof gate.

  python -I roof_head_duck_table.py --new <sources-final-C.txt> [--base <sources-final-B.txt>] [--json out.json] [--txt out.txt]

For every row: the roof penetration / gate of the BASE clip and of the NEW clip (round-tripped GLB of the build run); for the rows whose run dir changed: the body shell worst capsule, the door phases at the
shipped close time (clearance cm, '!' = over its gate), the regression list of row_compare.screen (per capsule vs the base, 0.3 cm), the failing checks a..n of check_car_entry.py (with the new build report),
the approach gate verdict of the run ('RESULT approach' line) and the head duck numbers of the build report (largest total bend, Hips sink, neck flexion / trunk lean peaks).
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import row_compare as RC  # noqa: E402
import roof_table as RT  # noqa: E402

BODYWAIVE = {  # build_car.ps1 bodyWaive after round 2 (body cm); None = absolute gate
}


def approach_line(run, name):
    p = os.path.join(run, 'approach-%s.txt' % name)
    if not os.path.exists(p):
        return 'n/a'
    b = open(p, 'rb').read()
    t = b.decode('utf-16') if b[:2] == bytes([255, 254]) else b.decode('utf-8', 'replace')
    m = re.findall(r'RESULT approach[^\n]*', t)
    return m[-1][:140] if m else 'n/a'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--new', required=True)
    ap.add_argument('--base', default=RC.SRC)
    ap.add_argument('--json')
    ap.add_argument('--txt')
    ap.add_argument('--rows')
    a = ap.parse_args()
    new, base = RT.read_sources(a.new), RT.read_sources(a.base)
    rows = a.rows.split(',') if a.rows else sorted(base)
    out, lines = {}, []
    for n in rows:
        g, cls, seat = RT.parse(n)
        cb = os.path.join(base[n], 'roundtrip', n + '.anims.glb')
        cn = os.path.join(new[n], 'roundtrip', n + '.anims.glb')
        sb = RT.static(cb, cls, seat)
        if new[n] == base[n]:
            ln = '%-34s UNCHANGED (packed-plus)  roof %6.2f / gate %5.2f %s' % (n[9:], sb['roof_pen'], sb['roof_gate'], 'OVER by %.2f' % (sb['roof_pen'] - sb['roof_gate']) if sb['roof_pen'] > sb['roof_gate'] else 'ok')
            out[n] = dict(changed=False, roof_base=sb['roof_pen'], roof_gate=sb['roof_gate'])
            lines.append(ln)
            print(ln, flush=True)
            continue
        br = os.path.join(new[n], 'glb', n + '.build-report.json')
        r = RC.screen(n, cn, br)
        rep = json.load(open(br))
        hd = (rep.get('duck') or {}).get('head_duck') or {}
        ap_ = approach_line(new[n], n)
        nb, bb = r['new'], r['base']
        ln = ('%-34s CHANGED  roof %6.2f -> %6.2f / gate %5.2f %s | body worst %s (base %s) | door clear new [%s] base [%s] | regress %s | checks failing %s | duck max bend %.1f deg sink %.1f cm neck %.0f lean %.0f | %s' % (
            n[9:], bb['roof_pen'], nb['roof_pen'], nb['roof_gate'], 'OK' if nb['roof_pen'] <= nb['roof_gate'] else 'OVER',
            sorted(((round(v, 2), k) for k, v in nb['body'].items()), reverse=True)[0], sorted(((round(v, 2), k) for k, v in bb['body'].items()), reverse=True)[0],
            ' '.join('%s %.1f%s' % (p[:5], v['clear'], '' if v['ok'] else '!') for p, v in nb['door'].items()),
            ' '.join('%s %.1f%s' % (p[:5], v['clear'], '' if v['ok'] else '!') for p, v in bb['door'].items()),
            r['regress'] or 'none', r['check_fail'] or 'none', hd.get('max_bend_deg', 0.0), hd.get('max_sink_cm', 0.0),
            max(hd.get('neck_flex_after', [0] or [0])), max(hd.get('trunk_lean_after', [0] or [0])), ap_))
        out[n] = dict(changed=True, result=r, duck={k: v for k, v in hd.items() if k in ('max_bend_deg', 'max_sink_cm', 'chain', 'az', 'target_cm', 'gate_cm', 'bend_deg')}, approach=ap_)
        lines.append(ln)
        print(ln, flush=True)
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1, default=float)
    if a.txt:
        open(a.txt, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
