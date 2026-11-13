"""Smallest body / roof waivers (build_car.ps1 bodyWaive = @(body cm, roof cm)) that make the static phases of check_door_clearance.py pass for a set of clips, and which capsules a waiver of that size still gates.

  python -I waiver_table.py --sources <sources.txt> [--slack-cm 0.3]

A waiver W is a flat gate: capsule c passes when its penetration <= max(W, seated baseline + 0.5 cm). The required waiver of a phase is the largest penetration among the capsules that exceed their
absolute gate (1 cm, or the seated baseline + 0.5 cm); None = the absolute gates pass, no waiver needed. A waiver at or above a capsule's radius cannot fail that capsule (roof sphere 15 cm; body radii in
check_door_clearance.BODY_R): such a row is reported 'GATES NOTHING'. Output: one line per clip plus a JSON (--json).
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np  # noqa: E402
import check_door_clearance as C  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources', required=True)
    ap.add_argument('--slack-cm', type=float, default=0.3)
    ap.add_argument('--json')
    a = ap.parse_args()
    out = {}
    for ln in open(a.sources, encoding='utf-8').read().splitlines():
        if not ln.strip():
            continue
        n, d = ln.split('  ', 1)
        n, d = n.strip(), d.strip()
        m_ = re.match(r'utpp_car_p([wm])a_(\w+?)_(driver|passenger_r)$', n)
        g, cls, seat = m_.groups()
        m = C.Model(d + '\\roundtrip\\' + n + '.anims.glb', cls, seat)
        row = {}
        for ph in ('body', 'roof'):
            r = m.static_phase(ph)
            pc = r[0] * 100
            base = m.gate_cm(ph, 1.0)  # absolute gate per capsule (no waiver set)
            bad = pc > base + 1e-6
            req = float(pc[bad].max()) if bad.any() else None
            rad = np.full(len(m.names), 15.0) if ph == 'roof' else m.rad * 100
            if req is None:
                row[ph] = dict(required_cm=None, waiver_cm=None)
            else:
                w = math.ceil((req + a.slack_cm) * 10) / 10.0
                idx = [m.names.index('head')] if ph == 'roof' else range(len(m.names))
                nothing = [m.names[c] for c in idx if w >= rad[c] - 1e-9]
                row[ph] = dict(required_cm=round(req, 2), waiver_cm=w, worst=m.names[int(np.argmax(np.where(bad, pc, -99)))], capsules_not_gated=nothing)
        out[n] = row
        print('%-36s body %s | roof %s' % (n[9:], row['body'], row['roof']), flush=True)
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
