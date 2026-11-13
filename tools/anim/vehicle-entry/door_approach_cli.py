"""Command line of the approach gate (check_door_clearance.py --approach / --calibrate); the model itself is door_approach.py.

  python -I check_door_clearance.py --approach --clip <roundtrip glb> --class standard3 --seat driver [--hold-cap on|off|both] [--align T|door|both]
         [--open-delay S] [--fps-set 40,50,60,90] [--jobs N] [--no-grid] [--grid-r 0.1 --grid-deg 5] [--only name,name] [--report out.json]
         [--open-tol-cm X --opening-tol-cm X]            (the doorWaive of build_car.ps1, never more)
  python -I check_door_clearance.py --approach --calibrate --sources <pk-door4-anims\\sources.txt> [--hold-cap off|on|both] [--align both|T|door] [--jobs N]
         calibration on the 30 measured rides with the shipped female clips: must FAIL 4, 30, 31 and PASS 5, 12, 15, 16, 24, 26 without the cap (with the cap 4, 30 must
         PASS); 9, 28, 29 reported. A logged vt_mode 'none' is modelled as NO hold (never as a jump). Exit code 1 when any ride DISAGREES.
Exit code: 1 when a start has a 'hold' or 'entity' failure at the selected cap setting(s), else 0 ('slide' warnings never change the code and are always printed).
With --approach-baseline the verdict is RELATIVE, per (class, capsule): a candidate fails when ANY capsule's excess is more than 0.05 cm above the excess the baseline clip has
for the SAME capsule of the same class (a capsule absent from the baseline = -inf). The absolute worst per class (before any filtering) is printed and written next to it.
"""
import json
import os

import numpy as np

import door_approach as A
import gen_door_timing as G

CALIB_FAIL = (4, 30, 31)
CALIB_PASS = (5, 12, 15, 16, 24, 26)
CALIB_REPORT = (9, 28, 29)
_W = {}


def waiver(cls, g, seat):
    """build_car.ps1 doorWaive (opening / open phase tolerance, cm) of the shipped clips; it never covers anything but the opening / open door phases."""
    W = {('sport2', 'w', 'passenger_r'): 6.0}  # the sport1 (Turbo-R) waivers 15 / 8.5 cm were removed in pk-door7 (door7-turbo, README)
    w = W.get((cls, g, seat))
    return {} if w is None else dict(open_tol_cm=w, opening_tol_cm=w)


def _worker(job):
    import check_door_clearance as C
    clip, cls, seat, gk, start, cap_on, aligns, delays, fps_set, mk = job
    key = (clip, cls, seat, json.dumps(gk, sort_keys=True), json.dumps(mk, sort_keys=True))
    if key not in _W:
        tj = C.load_timing()
        _W[key] = A.Approach(C.Model(clip, cls, seat, timing=tj, **mk), tj, None, **gk)
    if delays == 'nominal':  # the class open delay of door_timing.json (no +-0.03 s sweep)
        delays = [round(float(_W[key].m.tm['open_start_s']), 4)]
    sw = _W[key].sweep(start, cap_on, aligns=aligns, delays=delays, fps_set=fps_set)
    keep = ('mode', 'trig', 'tEnd', 't_edge', 'cap_state', 'cap_events', 'anchor', 'engage', 'peak', 'align', 'delay', 'fps', 'speed')
    return start['name'], cap_on, dict(worst=sw['worst'], worst_cap=sw['worst_cap'], trig=sw['trig'], modes=sw['modes'], runs=[{k: r[k] for k in keep} for r in sw['runs']], speed=sw['speed'])


def run_jobs(jobs, nproc):
    if nproc <= 1:
        return [_worker(j) for j in jobs]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(nproc) as ex:
        return list(ex.map(_worker, jobs, chunksize=1))


SPEED_GATE = os.environ.get('UTPP_APPROACH_SPEED_GATE', '1') != '0'  # --calibrate: a start over the speed limit fails the run (exit code 1); '0' = report only
SPEED_GATE_CLIP = os.environ.get('UTPP_APPROACH_SPEED_GATE') == '1'  # per-clip --approach (build_car.ps1, joint_prescreen.py): report only unless '1' (the shipped clips' own walk exceeds the limit on 2 rides)


def fmt_speed(sp):
    """Visible root speed of a start (door_approach.visible_speed, worst case of the sweep): peak from 0.06 s, the pre-engage peak, the release closing speed, the hold end."""
    if not sp:
        return 'speed n/a'
    return 'speed %.2f m/s at %.2f s (entity %.2f; first 0.06 s %.2f, release closes %.2f, offset <= %.2f m until %.2f s)%s' % (
        sp['peak'], sp['at'], sp['ent'], sp['early'], sp['closing'], sp['peak_off'], sp['end'], ' [door cap sent: exempt]' if sp['capSent'] else '')


def speed_fails(sp):
    return bool(sp) and not sp['capSent'] and sp['peak'] > A.SPEED_MAX_MPS and sp['peak'] > A.SPEED_WALK * sp['ent']


def fmt_worst(w):
    return '; '.join('%s +%.1fcm (%s %s t=%.2f door %.0f deg, %s, d=%.3f, fps %s)' % (
        k, v['excess_cm'], v['phase'], v['capsule'], v['t'], v['door_deg'], v['align'], v['delay'], '/'.join('%d' % f for f in v['fps'])) for k, v in sorted(w.items())) or 'clear'


def summarize(w):
    return 'FAIL' if any(k in w for k in A.HARD) else 'PASS'


def fmt_worse(worse):
    """The capsules that are worse than the baseline: class capsule +excess (baseline excess)."""
    return ', '.join('%s %s %+.2f (base %s)' % (k, cn, ex, 'absent' if be is None else '%+.2f' % be) for k, d in sorted(worse.items()) for cn, (ex, be) in sorted(d.items()))


def approach_cli(a, tj):
    """a: argparse namespace of check_door_clearance.py. Returns the exit code."""
    import check_door_clearance as C
    fx = A.load_fixtures()
    caps = {'on': (True,), 'off': (False,), 'both': (False, True)}[a.hold_cap]
    aligns = {'T': ('T',), 'door': ('door',), 'both': ('T', 'door')}[a.align]
    fps_set = tuple(float(x) for x in a.fps_set.split(',')) if a.fps_set else A.FPS_SET
    gk = dict(tol_cm=a.tol_cm, open_tol_cm=a.open_tol_cm, opening_tol_cm=a.opening_tol_cm)
    mk = dict(shift_rear=a.shift_rear, no_rear=a.no_rear, rear_pillar=a.rear_pillar)
    if a.rear_ref and not a.calibrate:
        import check_door_clearance as C0
        mk['rear_ref'] = C0.rear_ref_for(a, a.clip, a.cls, a.seat)
    delays = [a.open_delay] if a.open_delay is not None else ('nominal' if a.nominal_delay else None)
    if a.approach_baseline and not a.rear_ref:
        pass
    if a.calibrate:
        mk = dict(mk, no_rear=True)  # the calibration is about the FRONT door approach; the rear gate has its own non-vacuity check
        runs = {}
        for l in open(a.sources, encoding='ascii'):
            n, r = l.strip().split('  ', 1)
            runs[n] = r
        jobs = []
        for r in fx['rides']:
            n = 'utpp_car_pwa_%s_%s' % (r['cls'], r['seat'])
            clip = runs[n] + '/roundtrip/' + n + '.anims.glb'
            st = dict([s for s in A.fixture_starts(fx, r['cls'], r['seat']) if s['ride'] == r['ride']][0], use_label=True)
            for cap in caps:
                jobs.append((clip, r['cls'], r['seat'], dict(gk, **waiver(r['cls'], 'w', r['seat'])), st, cap, aligns, delays, fps_set, mk))
        res = run_jobs(jobs, a.jobs)
        disagree = 0
        verdict = {}
        for (name, cap, sw), j in zip(res, jobs):
            ride = j[4]['ride']
            verdict[(ride, cap)] = (summarize(sw['worst']), sw)
            print('ride %2d %-16s %-9s %-4s cap %-3s logged %-9s cap applied=%-5s %s | %s' % (ride, j[4]['name'].split('_', 1)[1], j[1] + '/' + j[2][:4], summarize(sw['worst']),
                                                                                               'on' if cap else 'off', j[4]['logged']['mode'], sw['trig'], fmt_worst(sw['worst']),
                                                                                               fmt_speed(sw['speed'])))
        for cap in caps:
            bad = []
            for ride in CALIB_FAIL:
                want = 'PASS' if cap and ride in (4, 30) else 'FAIL'
                if verdict[(ride, cap)][0] != want:
                    bad.append('ride %d expected %s' % (ride, want))
            for ride in CALIB_PASS:
                if verdict[(ride, cap)][0] != 'PASS':
                    bad.append('ride %d expected PASS' % ride)
            print('CALIBRATION cap %s alignments %s: FAIL set %s / PASS set %s -> %s%s' % ('on' if cap else 'off', '+'.join(aligns), CALIB_FAIL, CALIB_PASS,
                                                                                         'AGREES' if not bad else 'DISAGREES', '' if not bad else ': ' + '; '.join(bad)))
            print('  reported, no waiver: ' + ', '.join('ride %d %s' % (r, verdict[(r, cap)][0]) for r in CALIB_REPORT))
            print('  the native applied the cap (state 2 / 3) on rides: %s' % [r for (r, c), v in sorted(verdict.items()) if c == cap and v[1]['trig']])
            disagree += bool(bad)
            over = [r for (r, c), v in sorted(verdict.items()) if c == cap and speed_fails(v[1]['speed'])]
            exempt = [r for (r, c), v in sorted(verdict.items()) if c == cap and v[1]['speed'] and v[1]['speed']['capSent']]
            print('SPEED GATE cap %s (visible planar root speed at the native feed instants from %.2f s on <= %.2f m/s, glide limit %s): %d rides, %d over the limit %s, %d with a door cap sent '
                  '(exempt) %s' % ('on' if cap else 'off', A.SPEED_FROM_S, A.SPEED_MAX_MPS, 'on' if A.GLIDE else 'OFF', len([1 for (r, c) in verdict if c == cap]), len(over), over, len(exempt), exempt))
            disagree += bool(over) and SPEED_GATE
        if a.report:
            json.dump({'%d:%s' % (k[0], 'cap' if k[1] else 'nocap'): dict(verdict=v[0], worst=v[1]['worst'], worst_cap=v[1]['worst_cap'], trig=v[1]['trig'], modes=v[1]['modes'],
                                                                         runs=v[1]['runs'], speed=v[1]['speed']) for k, v in verdict.items()}, open(a.report, 'w'), indent=1, default=str)
        return 1 if disagree else 0
    if not (a.clip and a.cls and a.seat):
        raise SystemExit('--approach needs --clip, --class and --seat')
    m = C.Model(a.clip, a.cls, a.seat, timing=tj, **mk)
    arc = G.arc_of(tj, a.cls, m.side)
    starts = A.fixture_starts(fx, a.cls, a.seat)
    z = float(np.median([s['xyz'][2] for s in starts])) if starts else -0.35
    if not a.no_grid:
        starts += A.grid_starts(arc, a.seat, z, step_r=a.grid_r, step_deg=a.grid_deg)
    if getattr(a, 'arrival', False):  # r13: the arrival ring around the clip's own start spot (the native approach walk ends within 0.15 m of it)
        r0 = m.root_g[0] @ C.GLTF_TO_VEH.T
        yaw0 = float(np.degrees(np.arctan2(-(m.fwd0_g @ C.GLTF_TO_VEH.T)[0], (m.fwd0_g @ C.GLTF_TO_VEH.T)[1])))
        starts += A.arrival_starts((float(r0[0]), float(r0[1])), yaw0, z)
    if a.only:
        starts = [s for s in starts if s['name'] in a.only.split(',')]
    jobs = [(a.clip, a.cls, a.seat, gk, s, cap, aligns, delays, fps_set, mk) for s in starts for cap in caps]
    print('approach %s %s/%s: %d starts (%d fixtures, %d grid) x cap %s x alignments %s; door arc %s; hold constants from %s' % (
        os.path.basename(a.clip), a.cls, a.seat, len(starts), sum(1 for s in starts if s['kind'] == 'fixture'), sum(1 for s in starts if s['kind'] == 'grid') + sum(1 for s in starts if s['kind'] == 'arrival'),
        '+'.join('on' if c else 'off' for c in caps), '+'.join(aligns), 'valid' if arc else 'NONE (no cap, no grid)', A.K_SRC), flush=True)
    res = run_jobs(jobs, a.jobs)
    base = {}
    if a.approach_baseline:  # the approach gate is not calibrated as an ABSOLUTE gate (rides 12 / 15 / 16 of the shipped clips): a candidate must not be WORSE than the baseline clip
        bm = dict(mk, rear_ref=C.rear_ref_for(a, a.approach_baseline, a.cls, a.seat) if a.rear_ref else None)
        bjobs = [(a.approach_baseline, a.cls, a.seat, gk, s, cap, aligns, delays, fps_set, bm) for s in starts for cap in caps]
        for (name, cap, sw), j in zip(run_jobs(bjobs, a.jobs), bjobs):
            base[(name, cap)] = sw
    by = {}
    for (name, cap, sw), j in zip(res, jobs):
        sw['absolute_worst'] = dict(sw['worst'])  # snapshot BEFORE any baseline filtering
        sw['relative_worst'] = None
        sw['worse'] = {}
        if base:
            b = base[(name, cap)]
            # per (class, capsule): worse when ANY capsule's excess > the baseline's excess of the SAME capsule + 0.05 cm (absent from the baseline = -inf)
            worse = A.relative_worse(sw['worst_cap'], b['worst_cap'])
            rel = {k: v for k, v in sw['worst'].items() if k not in A.HARD}  # slide warnings stay (vanilla, never waived)
            for k, d in worse.items():
                cn = max(d, key=lambda c: d[c][0])
                rel[k] = dict(sw['worst_cap'][k][cn], baseline_excess_cm=d[cn][1], worse_capsules={c: list(v) for c, v in d.items()})
            sw['baseline_worst'] = b['worst']
            sw['baseline_worst_cap'] = b['worst_cap']
            sw['relative_worst'] = rel
            sw['worse'] = worse
            sw['worst'] = rel  # the gating result
        by[(name, cap)] = (j[4], sw)
    tally = {}
    for (name, cap), (s, sw) in sorted(by.items(), key=lambda kv: (kv[1][0]['kind'], kv[0][0], kv[0][1])):
        v = summarize(sw['worst'])
        va = summarize(sw['absolute_worst'])
        t = tally.setdefault(cap, dict(n=0, fail=0, slide=0, abs_fail=0, over=0, exempt=0))
        t['n'] += 1
        t['over'] += speed_fails(sw['speed'])
        t['exempt'] += bool(sw['speed'] and sw['speed']['capSent'])
        t['fail'] += v == 'FAIL'
        t['abs_fail'] += va == 'FAIL'
        t['slide'] += 'slide' in sw['worst']
        if s['kind'] == 'fixture' or v != 'PASS' or 'slide' in sw['worst']:
            if base:
                print('%-8s %-26s cap %-3s RELATIVE %-4s ABSOLUTE %-4s cap applied=%-5s modes=%s | relative: %s | absolute: %s' % (
                    s['kind'], name, 'on' if cap else 'off', v, va, sw['trig'], ','.join(sw['modes']), fmt_worse(sw['worse']) or 'not worse than the baseline',
                    fmt_worst(sw['absolute_worst'])))
            else:
                print('%-8s %-26s cap %-3s %-4s cap applied=%-5s modes=%s %s | %s' % (s['kind'], name, 'on' if cap else 'off', v, sw['trig'], ','.join(sw['modes']),
                                                                                         fmt_worst(sw['worst']), fmt_speed(sw['speed'])))
    rc = 0
    for cap in caps:
        t = tally[cap]
        print('RESULT approach cap %s alignments %s: %d starts, %d FAIL (hold / entity)%s, %d with slide warnings' % (
            'on' if cap else 'off', '+'.join(aligns), t['n'], t['fail'],
            ' RELATIVE to the baseline (per class + capsule), %d FAIL ABSOLUTE' % t['abs_fail'] if base else '', t['slide']))
        print('SPEED GATE approach cap %s: %d starts, %d over %.2f m/s (visible planar root speed from %.2f s on, glide limit %s), %d with a door cap sent (exempt)' % (
            'on' if cap else 'off', t['n'], t['over'], A.SPEED_MAX_MPS, A.SPEED_FROM_S, 'on' if A.GLIDE else 'OFF', t['exempt']))
        rc |= 1 if t['fail'] else 0
        rc |= 1 if t['over'] and SPEED_GATE_CLIP else 0
    if a.report:
        json.dump({'%s:%s' % (n, 'cap' if c else 'nocap'): dict(start=s, worst=sw['worst'], absolute_worst=sw['absolute_worst'], absolute_worst_cap=sw['worst_cap'],
                                                                relative_worst=sw['relative_worst'], worse=sw['worse'], baseline_worst=sw.get('baseline_worst'),
                                                                baseline_worst_cap=sw.get('baseline_worst_cap'), trig=sw['trig'], modes=sw['modes'], runs=sw['runs'], speed=sw['speed'])
                   for (n, c), (s, sw) in by.items()}, open(a.report, 'w'), indent=1, default=str)
    return rc
