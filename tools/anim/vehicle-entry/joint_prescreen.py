"""Joint pre-screen of the start-back lever with the back ramp (no WolvenKit round trip): builds the clip with build_entry_clip.py for candidate
(--start-back, --back-ramp A B, hands follow) and runs ALL THREE gates on the builder GLB:
  front door  analyse() at the solved close, door_timing.json V-lag set (check_door_clearance.py)
  rear        phase 'rear' (rear door skin / glass / B-pillar block), relative to the vanilla clip of the set (--rear-ref)
  approach    check_door_clearance --approach (cap on, fixtures of the class + seat, alignment T, nominal open delay unless --full) - only when requested

  python -I joint_prescreen.py --class standard3 --gender w --seat passenger_r --back 0.2 0.4 0.6 --ramp 0.15,0.85 0.15,0.7 [--approach] [--jobs 4] [--scratch <dir>]

Prints one line per candidate; the smallest start-back that passes every selected gate is the one for build_car.ps1 (class table: door = @(out, back, rampA, rampB)).
The builder arguments are door_prescreen.py's (= build_car.ps1's).
"""
import argparse
import itertools
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import door_prescreen as P  # noqa: E402
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

EX = P.EX
_BASE = {}
_LOCK = __import__('threading').Lock()
BASELINE_SOURCES = [os.path.join(P.EX, '..', '..', 'Build', 'work', 'vehicle-entry-car', 'pk-door6-anims', 'sources.txt')]  # the CURRENTLY SHIPPED set (--baseline-sources)
SRL = [4.0, 0.0, 0.0, 0.0]  # srl, srlg, srl2, keep  # --smooth-rot-limit of the ramp builds (--srl)


def job(j):
    cls, g, seat, back, ramp, hfs, scratch, appr, full, extra, out, oramp, hout = j
    import check_door_clearance as C
    tag = 'jp_p%sa_%s_%s_o%03d_b%03d_r%03d-%03d_h%d' % (g, cls, seat, round(out * 100), round(back * 100), round(ramp[0] * 100), round(ramp[1] * 100), round(hfs * 10)) + ('' if SRL[0] == 4.0 else '_s%g' % SRL[0]) + ('' if SRL[1] <= 0 else '_g%g' % SRL[1]) + ('' if SRL[2] <= 0 else '_k%g-%g' % (SRL[2], SRL[3])) + ('' if not hout else '_ho%03d-%03d-%03d%s' % (round(hout[0] * 100), round(hout[1] * 100), round(hout[2] * 100), '' if hout[3] == 'both' else hout[3][0])) + ('' if not oramp else '_or%03d-%03d' % (round(oramp[0] * 100), round(oramp[1] * 100)))
    sdir = os.path.join(scratch, tag)
    os.makedirs(sdir, exist_ok=True)
    ex = list(extra)
    if ramp[1] > 0:
        ex += ['--back-ramp', str(ramp[0]), str(ramp[1]), '--smooth-rot-limit', str(SRL[0])] + (['--smooth-rot-limit-global', str(SRL[1])] if SRL[1] > 0 else []) + (['--smooth-rot-limit2', str(SRL[2]), '--smooth-keep-first', str(SRL[3])] if SRL[2] > 0 else [])  # check (d): the forward move steps a foot; sport1 passenger uses the same limit
        if hfs > 0:
            ex += ['--hand-follow-shift', str(hfs)]
    if hout:
        ex += ['--hand-out', str(hout[0]), '--hand-out-window', str(hout[1]), str(hout[2])] + ([] if hout[3] == 'both' else ['--hand-out-side', hout[3]]) + (['--out-axis', '-1' if seat == 'driver' else '1'] if out <= 0 else [])
    if oramp:
        ex += ['--out-ramp', str(oramp[0]), str(oramp[1])]
        if ramp[1] <= 0:
            ex += ['--smooth-rot-limit', str(SRL[0])] + (['--smooth-rot-limit-global', str(SRL[1])] if SRL[1] > 0 else []) + (['--smooth-rot-limit2', str(SRL[2]), '--smooth-keep-first', str(SRL[3])] if SRL[2] > 0 else [])
    glb = os.path.join(sdir, 'ps_p%sa_%s_%s_o%03d_b%s%03d.anims.glb' % (g, cls, seat, round(out * 100), 'n' if back < 0 else '', abs(round(back * 100))))
    if not os.path.exists(glb):
        glb, rep = P.build(cls, g, seat, out, back, sdir, ex)
    van = '%s\\glb\\p%sa_%s_%s.anims.glb' % (EX, g, cls, seat)
    m = C.Model(glb, cls, seat, rear_ref=van)
    sv = C.solve(m)
    ca = sv.get('close_after_s')
    r = C.analyse(m, ca) if ca else None
    ph = (r or {}).get('phases', {})
    s = ' '.join('%s %.1f%s' % (p[:6], v['clearance_cm'], '' if v['ok'] else '!') for p, v in ph.items())
    rear = ph.get('rear', {})
    res = dict(tag=tag, back=back, ramp=ramp, hfs=hfs, door_ok=bool(r and r['door_clearance_ok']), close_after=ca, phases=s, rear_cm=rear.get('clearance_cm'),
               rear_by=(rear.get('capsule'), rear.get('door_bone'), rear.get('t_clip_s')))
    rear_ok = rear.get('ok', True) if rear else True
    res['rear_ok'] = bool(rear_ok)
    if appr and res['door_ok'] and rear_ok:
        import door_approach as A
        import door_approach_cli as AC
        tj = C.load_timing()
        fx = A.load_fixtures()
        fps = A.FPS_SET if full else (50.0, 60.0)
        dl = None if full else [round(float(m.tm['open_start_s']), 4)]

        def worst_of(model):  # {start: {class: {capsule: excess_cm}}} - the verdict is per (class, capsule), see door_approach.relative_worse
            ap = A.Approach(model, tj, fx)
            out = {}
            for st in A.fixture_starts(fx, cls, seat):
                sw = ap.sweep(st, True, aligns=('T',), delays=dl, fps_set=fps)
                out[st['name']] = {k: {cn: w['excess_cm'] for cn, w in d.items()} for k, d in sw['worst_cap'].items() if k in A.HARD}
            return out
        key = (cls, g, seat, full, BASELINE_SOURCES[0])
        _LOCK.acquire()
        if key not in _BASE:  # the SHIPPED clip under the same settings: the approach gate is not calibrated as an absolute gate (rides 12, 15), so a candidate must not be WORSE
            runs = {}
            for l in open(BASELINE_SOURCES[0], encoding='ascii'):
                n_, r_ = l.strip().split('  ', 1)
                runs[n_] = r_
            n = 'utpp_car_p%sa_%s_%s' % (g, cls, seat)
            _BASE[key] = worst_of(C.Model(runs[n] + '/roundtrip/' + n + '.anims.glb', cls, seat, rear_ref=van))
        _LOCK.release()
        cand = worst_of(m)
        rec = lambda d: {k: {cn: {'excess_cm': ex} for cn, ex in c.items()} for k, c in d.items()}
        worse = {st: A.relative_worse(rec(cw), rec(_BASE[key][st])) for st, cw in cand.items()}
        worse = {st: w for st, w in worse.items() if w}
        res['approach'] = 'PASS' if not worse else 'FAIL'
        res['approach_abs'] = {st: cw for st, cw in cand.items() if cw}  # absolute worst per class + capsule (before the baseline comparison)
        res['approach_shipped'] = {st: cw for st, cw in _BASE[key].items() if cw}
        res['approach_worse'] = worse  # {start: {class: {capsule: (excess, baseline excess or None)}}}
        res['approach_worst'] = 'worse than shipped (per class + capsule): %s | absolute: %s' % (worse, res['approach_abs']) if worse else 'not worse than shipped (abs %s)' % res['approach_abs']
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--class', dest='cls', required=True)
    ap.add_argument('--gender', default='w')
    ap.add_argument('--seat', default='driver')
    ap.add_argument('--out', type=float, nargs='+', default=[0.0])
    ap.add_argument('--back', type=float, nargs='+', default=[0.0])
    ap.add_argument('--ramp', nargs='+', default=['0,0'], help='A,B seconds (0,0 = the common window)')
    ap.add_argument('--hfs', type=float, nargs='+', default=[1.0], help='hand follow weight of the start-back ramp')
    ap.add_argument('--oramp', nargs='+', default=['0,0'], help='A,B seconds: own window of the start-out part (0,0 = follows the start-back window)')
    ap.add_argument('--srl', type=float, default=4.0, help='--smooth-rot-limit of the ramp builds (check d needed 4 with the ramp)')
    ap.add_argument('--srlg', type=float, default=0.0, help='--smooth-rot-limit-global of the ramp builds (0 = same as --srl)')
    ap.add_argument('--srl2', type=float, default=0.0, help='--smooth-rot-limit2 (second global pass)')
    ap.add_argument('--keep', type=float, default=0.0, help='--smooth-keep-first of the second pass, s')
    ap.add_argument('--hout', nargs='+', default=['0'], help='M,A,B[,Left|Right|both]: --hand-out metres, full until A s, gone at B s (0 = off)')
    ap.add_argument('--baseline-sources', default=None, help='approach: <clip>  <run dir> lines of the baseline set (default: the shipped Build\\work\\vehicle-entry-car\\pk-door6-anims\\sources.txt)')
    ap.add_argument('--approach', action='store_true')
    ap.add_argument('--full', action='store_true', help='approach: swept open delays and sample rates')
    ap.add_argument('--extra', default='', help='extra builder args, comma separated: --extra=--hand-follow-min,1.0')
    ap.add_argument('--scratch', default=os.path.join(project_root(), r'Build\work\vehicle-entry-car\joint-prescreen'))
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--jsonl', default=None, help='append one json line per candidate')
    a = ap.parse_args()
    if a.baseline_sources:
        BASELINE_SOURCES[0] = a.baseline_sources
    SRL[0] = a.srl
    SRL[1] = a.srlg
    SRL[2] = a.srl2
    SRL[3] = a.keep
    os.makedirs(a.scratch, exist_ok=True)
    ramps = [tuple(float(x) for x in r.split(',')) for r in a.ramp]
    oramps = [tuple(float(x) for x in r.split(',')) for r in a.oramp]
    houts = [None if h == '0' else (float(h.split(',')[0]), float(h.split(',')[1]), float(h.split(',')[2]), (h.split(',') + ['both'])[3]) for h in a.hout]
    jobs = [(a.cls, a.gender, a.seat, b, r, h, a.scratch, a.approach, a.full, [x for x in a.extra.split(',') if x], o, (None if orr[1] <= 0 else orr), ho) for o, b, r, h, orr, ho in itertools.product(a.out, a.back, ramps, a.hfs, oramps, houts)]
    with ThreadPoolExecutor(a.jobs) as ex:
        for j_, res in zip(jobs, ex.map(job, jobs)):
            res.update(cls=a.cls, gender=a.gender, seat=a.seat, out=j_[10], oramp=j_[11], shift=round((j_[10] ** 2 + j_[3] ** 2) ** 0.5, 3), ok_all=bool(res['door_ok'] and res['rear_ok'] and res.get('approach', 'PASS') == 'PASS'))
            if a.jsonl:
                open(a.jsonl, 'a', encoding='utf-8').write(json.dumps(res) + chr(10))
            print('%s | out/back %.2f ramp %s hfs %.1f | door_ok=%s close_after=%s | %s | approach=%s %s' % (res['tag'], res['back'], res['ramp'], res['hfs'], res['door_ok'], res['close_after'],
                  res['phases'], res.get('approach', '-'), res.get('approach_worst', '')), flush=True)


if __name__ == '__main__':
    main()
