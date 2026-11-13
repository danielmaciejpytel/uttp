"""24-row comparison table for the roof head cap (new vs shipped): roof penetration / gate, the worst body-shell capsules, and the checks a..n of check_car_entry.py.

  python -I roof_table.py --new <sources.txt> --shipped <sources.txt> [--json out.json] [--rows pwa_sport1_driver,...]

sources.txt: lines '<clip name>  <run dir>' (build_car.ps1 runs: the clip is <run dir>\\roundtrip\\<clip>.anims.glb, its build report <run dir>\\glb\\<clip>.build-report.json, the check
JSON of the run <run dir>\\check-<clip>.json whose c_start_in_seat gives the start shift). Both sets are checked with the CURRENT check_car_entry.py (shift assertion taken from the
shipped / new run's own c_start_in_seat) and the CURRENT roof / body phases of check_door_clearance (roof_report.py), so the numbers are comparable. 'regress' = a body capsule or the roof
penetrates more than the other set by > 0.3 cm AND beyond its absolute gate.
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
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

GLB = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car\glb')


def read_sources(p):
    out = {}
    for ln in open(p, encoding='utf-8').read().splitlines():
        if ln.strip():
            n, d = ln.split('  ', 1)
            out[n.strip()] = d.strip()
    return out


def parse(name):
    m = re.match(r'utpp_car_p([wm])a_(\w+?)_(driver|passenger_r)$', name)
    return m.group(1), m.group(2), m.group(3)


def static(clip, cls, seat):
    m = C.Model(clip, cls, seat)
    bp, rp, bw, sb, sr, hi = m.static_series()
    body = m.static_phase('body')
    gate_b = m.gate_cm('body', 1.0)
    gate_r = float(m.gate_cm('roof', 1.0)[m.names.index('head')])
    pen_b = {n: float(body[0][i]) * 100 for i, n in enumerate(m.names)}
    return dict(roof_pen=float(rp.max()) * 100, roof_gate=gate_r, body=pen_b, body_gate={n: float(gate_b[i]) for i, n in enumerate(m.names)}, seated_roof=float(sr) * 100)


def checks(clip, name, rundir):
    g, cls, seat = parse(name)
    tgt = '%s\\p%sa_%s_%s.anims.glb' % (GLB, g, cls, seat)
    npc = '%s\\%sa_%s_passenger_r.anims.glb' % (GLB, g, cls)
    endc = 'tpp_idle' if seat == 'driver' else 'idle'
    cmd = [sys.executable, '-I', os.path.join(HERE, 'check_car_entry.py'), '--clip', clip, '--vanilla', tgt, '--src-rig', npc, '--tpp-idle', tgt, '--tpp-idle-clip', endc]
    br = os.path.join(rundir, 'glb', name + '.build-report.json')
    if os.path.exists(br):
        cmd += ['--build-report', br]
    cj = os.path.join(rundir, 'check-' + name + '.json')
    if os.path.exists(cj):
        c = json.load(open(cj))['c_start_in_seat']
        d = np.array(c['new']['pos']) - np.array(c['vanilla']['pos'])
        if np.linalg.norm(d) > 0.005:
            cmd += ['--expect-start-shift', '%.4f' % d[0], '--expect-start-shift-z', '%.4f' % d[2]]
    cmd += ['--report', os.path.join(os.environ.get('TEMP', '.'), 'rt_%s.json' % name)]
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    res = json.load(open(cmd[-1]))
    return {k: v for k, v in res.items() if k.endswith('_ok') or k in ('n_anatomy', 'n_report_under_vanilla_cm')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--new', required=True)
    ap.add_argument('--shipped', required=True)
    ap.add_argument('--rows')
    ap.add_argument('--json')
    a = ap.parse_args()
    new, ship = read_sources(a.new), read_sources(a.shipped)
    rows = a.rows.split(',') if a.rows else sorted(set(new) & set(ship))
    rows = [r if r.startswith('utpp_car_') else 'utpp_car_' + r for r in rows]
    out = {}
    for n in rows:
        g, cls, seat = parse(n)
        r = {}
        for tag, srcs in (('new', new), ('shipped', ship)):
            clip = srcs[n] + '\\roundtrip\\' + n + '.anims.glb'
            st = static(clip, cls, seat)
            ck = checks(clip, n, srcs[n])
            r[tag] = dict(run=srcs[n], roof_pen=round(st['roof_pen'], 2), roof_gate=round(st['roof_gate'], 2), seated_roof=round(st['seated_roof'], 2),
                          body_worst=sorted(((round(v, 2), k) for k, v in st['body'].items()), reverse=True)[:3], body=st['body'], body_gate=st['body_gate'], checks=ck)
        reg = []
        for k, v in r['new']['body'].items():
            if v > r['shipped']['body'][k] + 0.3 and v > r['new']['body_gate'][k]:
                reg.append((k, round(v, 2), round(r['shipped']['body'][k], 2)))
        r['body_regress'] = reg
        r['roof_better'] = r['new']['roof_pen'] < r['shipped']['roof_pen'] - 0.3
        r['roof_regress'] = bool(r['new']['roof_pen'] > r['shipped']['roof_pen'] + 0.3 and r['new']['roof_pen'] > r['new']['roof_gate'])
        out[n] = r
        sc, nc = r['shipped']['checks'], r['new']['checks']
        print('%-36s roof %6.2f/%5.2f (shipped %6.2f) %s | body worst %s (shipped %s) regress=%s | checks new fail=%s shipped fail=%s' % (
            n[9:], r['new']['roof_pen'], r['new']['roof_gate'], r['shipped']['roof_pen'], 'BETTER' if r['roof_better'] else ('WORSE' if r['roof_regress'] else 'same'),
            r['new']['body_worst'][0], r['shipped']['body_worst'][0], reg, [k for k, v in nc.items() if k.endswith('_ok') and not v], [k for k, v in sc.items() if k.endswith('_ok') and not v]), flush=True)
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
