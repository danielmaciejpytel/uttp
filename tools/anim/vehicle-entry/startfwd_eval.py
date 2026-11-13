"""Candidate screen for the start-forward rework (round 4): builds one row with build_car.ps1 -BuilderOnly (-DoorOverride / -ExtraArgs) and judges the builder GLB
with every gate that does not need the WolvenKit round trip:

  python -I startfwd_eval.py --class sport2 --gender w --seat driver --door 0,0.1,0.2,0.6 [--extra=--start-hold,0.3] [--approach] [--tag name] [--out-root <dir>]

Prints one line: start gate (dy / slide vs the game's clip start), joint rule, the door phases at the solved close (closed / opening / open / closing / closed_after /
rear / body / roof clearance in cm, '!' = over its gate), check_car_entry RESULT, and with --approach the approach gate verdict relative to the shipped baseline.
The row's body / roof waivers are NOT applied (raw numbers; build_car.ps1 -NoBodyWaive semantics): a row passes only with an absolute gate.
"""
import argparse
import json
import os
import subprocess
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = project_root()
VB = ROOT + r'\Build\work\vehicle-entry-car'
DEFAULT_OUT = VB + r'\startfwd\scr'
BASELINE_SOURCES = VB + chr(92) + 'roof-pack3' + chr(92) + 'sources-final-C3.txt'  # the CURRENTLY shipped set (roof round 3 conditional set); --baseline-sources overrides


def build(a, tag):
    out = os.path.join(a.out_root, tag)
    if os.path.exists(out):
        import shutil
        shutil.rmtree(out)
    cmd = ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', os.path.join(HERE, 'build_car.ps1'), '-Classes', a.cls, '-Genders', a.gender, '-Seats', a.seat,
           '-BuilderOnly', '-NoBodyWaive', '-Out', out, '-DoorOverride', '%s:%s:%s=%s' % (a.cls, a.gender, a.seat, a.door)]
    if a.extra:
        cmd += ['-ExtraArgs', ','.join(a.extra)]
    if a.duck_extra:
        cmd += ['-DuckExtra', ','.join(a.duck_extra)]
    if a.no_roof:
        cmd += ['-NoRoofCap']
    if a.no_duck:
        cmd += ['-NoDuck']
    r = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
    glb = os.path.join(out, 'glb', 'utpp_car_p%sa_%s_%s.anims.glb' % (a.gender, a.cls, a.seat))
    if r.returncode != 0 or not os.path.exists(glb):
        raise RuntimeError('build failed: ' + (r.stdout + r.stderr)[-1500:])
    res = [ln for ln in r.stdout.splitlines() if ln.startswith('RESULT')]
    return out, glb, (res[-1] if res else 'RESULT ?'), r.stdout


def screen(a):
    import check_door_clearance as C
    import start_gate as SG
    tag = a.tag or 'c_p%sa_%s_%s_%s%s' % (a.gender, a.cls, a.seat, a.door.replace(',', '_'), ('_' + '_'.join(x.strip('-') for x in a.extra)) if a.extra else '')
    out, glb, result, stdout = build(a, tag)
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
    rw = sg['rearward']['rows']
    line = '%s | start dy %+.2f slide %.2f %s | rear %s H %.2f/%.2f sink %.2f/%.2f | joints %s | close_after %s | %s | %s' % (
        tag, st['dy_vs_vanilla_m'], st['slide_m'], 'ok' if st['ok'] else 'FAIL', 'ok' if sg['rearward']['ok'] else 'FAIL', rw['Hips']['excursion_m'], rw['Hips']['game_m'], rw['sink_slide_Hips']['excursion_m'], rw['sink_slide_Hips']['game_m'], 'ok' if sg['joint_rule']['ok'] else 'FAIL', ca, s, result)
    if a.approach:
        line += ' | ' + approach(a, glb, cls, seat, g, van, out)
    print(line, flush=True)
    return line


def approach(a, glb, cls, seat, g, van, out):
    import check_door_clearance as C
    import door_approach as A
    m = C.Model(glb, cls, seat, rear_ref=van)
    tj = C.load_timing()
    fx = A.load_fixtures()
    fps = A.FPS_SET if a.full else (50.0, 60.0)
    dl = None if a.full else [round(float(m.tm['open_start_s']), 4)]

    def worst_of(model):
        ap = A.Approach(model, tj, fx)
        res = {}
        for st in A.fixture_starts(fx, cls, seat):
            sw = ap.sweep(st, True, aligns=('T',), delays=dl, fps_set=fps)
            res[st['name']] = {k: {cn: w['excess_cm'] for cn, w in d.items()} for k, d in sw['worst_cap'].items() if k in A.HARD}
        return res
    runs = {}
    for l in open(a.baseline_sources or BASELINE_SOURCES, encoding='utf-8'):
        n_, r_ = l.strip().split('  ', 1)
        runs[n_] = r_
    n = 'utpp_car_p%sa_%s_%s' % (g, cls, seat)
    base = worst_of(C.Model(runs[n] + '/roundtrip/' + n + '.anims.glb', cls, seat, rear_ref=van))
    cand = worst_of(m)
    rec = lambda d: {k: {cn: {'excess_cm': ex} for cn, ex in c.items()} for k, c in d.items()}
    worse = {stn: A.relative_worse(rec(cw), rec(base[stn])) for stn, cw in cand.items()}
    worse = {stn: w for stn, w in worse.items() if w}
    absw = {stn: cw for stn, cw in cand.items() if cw}
    return 'approach %s vs shipped (%d starts) abs %s worse %s' % ('PASS' if not worse else 'FAIL', len(cand), absw, worse)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--gender', default='w')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--door', required=True, help='out,back[,rampA,rampB] (the class table door row)')
    ap.add_argument('--extra', action='append', default=[], help='builder arguments, one per --extra=...')
    ap.add_argument('--duck-extra', action='append', default=[], help='head_duck.py options appended after the class table duck options (later wins), one per --duck-extra=...')
    ap.add_argument('--no-roof', action='store_true')
    ap.add_argument('--no-duck', action='store_true')
    ap.add_argument('--approach', action='store_true')
    ap.add_argument('--full', action='store_true')
    ap.add_argument('--baseline-sources', default=None)
    ap.add_argument('--tag', default=None)
    ap.add_argument('--out-root', default=DEFAULT_OUT)
    a = ap.parse_args()
    os.makedirs(a.out_root, exist_ok=True)
    screen(a)


if __name__ == '__main__':
    main()
