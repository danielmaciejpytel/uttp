"""Merge the per-clip door-close solutions into door_timing.json and generate the redscript table.

  python -I gen_door_timing.py --merge <fragment.json | dir | glob> [...] [--reset] [--allow-single-gender]
                                                                              fragments written by check_door_clearance.py --solve --fragment
  python -I gen_door_timing.py [--emit-only]                                  regenerate Source\\plugin\\UTPP\\scripts\\utpp_vehicle_door_timing.reds from door_timing.json
  python -I gen_door_timing.py --check                                        exit 1 when the .reds is not what door_timing.json generates (stale) or the
                                                                              stored door arcs break their invariants (signs, hinge vs probe, fit)
  python -I gen_door_timing.py --arcs [--door-data <dir>]                     (re)compute the door arcs of door_timing.json 'arcs' from the extracted door-data
  python -I gen_door_timing.py --check-arcs-data [--door-data <dir>] [--timing <copy of door_timing.json>]
                                                                              exit 1 when the cap invariant (below) fails for the stored arcs or they differ from
                                                                              what the extracted door-data gives (--timing: judge a stored copy, e.g. the pre-review arcs)

door_timing.json 'clips'.<class>.<seat>.<p?a> = {close_after_s, earliest_close_after_s, vanilla_close_after_s, timing_status, source clip, ...}.
Contract with the redscript (utpp_vehicle_door.reds): the close is scheduled from the OnVehicleDoorOpen event with the event's own (vanilla)
autoCloseTime, so the generated table holds only the extra delay
    UTPPCarDoorCloseDeltaS(seatVar) = max over the female / male clip of (close_after_s - vanilla_close_after_s of the class / door side),
                                      clamped at >= 0, rounded UP to the millisecond (the shipped close is never earlier than the solved one),
and the redscript uses closeAfter = evt.autoCloseTime + delta when delta > 0. Both numbers are measured from the same door event, so the anchor
(event vs first detected door motion) cancels. -1.0 = seat without UTPP timing (no entry). Seat variable class: utility -> utility4 (make_car_workspot.py
VAR_CLASS). The value the offline gate must clear is vanilla_close_after_s + delta (check_door_clearance.py --table --close-from-timing).

DOOR ARC contract (2026-11-02, for the redscript cap of the root hold; generated next to UTPPCarDoorCloseDeltaS):
    public struct UTPPCarDoorArc { valid, hingeX, hingeY, length, closedDirDeg, openDeg, openDelayS, openDurS }
    public func UTPPCarDoorArcOf(seatVar: CName) -> UTPPCarDoorArc            (same seat variables as UTPPCarDoorCloseDeltaS; valid=false otherwise)
  Frame: the vehicle frame (x right, y forward, z up, m, vehicle origin); door-data glTF (gx,gy,gz) -> vehicle (gx,-gz,gy). hinge = hinge axis XY (the
  door bone origin; it does not move during a swing door's opening); length = hinge -> the farthest point of any door part (planar); closedDirDeg =
  atan2(dy, dx) in degrees of the trailing-edge direction (rear-most skin point) rotated toward the opening by leadDeg (see CAP INVARIANT; CCW positive seen from above,
  0 = +x/right, -90 = straight back) with the door closed; openDeg = signed peak
  swing (left door negative = clockwise, right door positive; the animation overshoots to 70 deg and settles at ~67 deg, the PEAK is stored = the larger
  sector); the edge direction at t seconds after the mount line is
      closedDirDeg + openDeg * smoothstep(clamp((t - openDelayS) / openDurS, 0, 1)).
  openDelayS = the EARLIEST measured door-motion start after the mount line of the class (door_timing.json classes.<c>.measured_open_start_range_s[0]);
  openDurS = the largest duration for which that smoothstep is never LATER than the animation (door-data startup phase mapped linearly onto the class
  open_dur_s) at any swing fraction 0.10..0.95 (so t_edge from the arc is never later than reality; the model door is early by 'fit.max_early_s').
  valid=false: Turbo-R (lifting door: the hinge travels 1.2 m, the swing is 9 deg) and every seat variable that is not in the table.
  Computed once from the extracted door-data by --arcs, stored in door_timing.json 'arcs' (with the sha256 of the door-data json and the independent
  probe hinge), rendered from the json alone (--check needs no extraction).
  CAP INVARIANT (review 2026-11-02; arc_invariant, checked by --arcs before writing and by --check-arcs-data / --check): the arcs are fitted on ALL points of ALL door
  parts (skin, glass, sill) over ALL startup frames, not on the single rear-most point. closedDirDeg = the trailing-edge direction (rear-most point of the skin,
  'trailingDirDeg') rotated toward the opening by 'leadDeg' (the door skin leads the rear-most point by up to ~15-25 deg near the hinge on the 4-door classes, less at the tail: one
  rotation has to serve every radius; fitted 12-17 deg on standard2 / standard25 / sport2, 0.5-2.5 deg on standard3 / utility), length = the largest radius of any part, openDurS = min(the rear-most-point fit, 0.95 * the largest duration for which the
  invariant holds). Exact criterion: for every anchor of the polar grid about the hinge (r = 0.4 m .. length + 0.2 m in 0.05 m steps, theta = -20 deg .. |peak| + 65 deg in
  1.5 deg steps from the DATA's trailing-edge direction, 4 grids shifted by 0 / 1/4 / 1/2 / 3/4 step; the fit sees only 0 and 1/2) that lies at least 0.10 m outboard of
  the outermost door point (the car side) and is not already within 0.15 m of a door point with the door closed, whose first-touch time t_data (any point of any part
  within 0.15 m planar, delay + startup frame mapped linearly onto the class open duration, distance interpolated between frames) exists: the redscript's UTPPDoorCapOf
  on the RENDERED arc numbers must return why 0 (or 4) with t_edge <= t_data. Anchors inside the car are not places V can stand; anchors in the closed door
  cannot be helped by any cap. The reds' own 0.20 m disk (vs 0.15 m contact) and the 0.05 s cap margin are extra safety on top.
  HOLD CAP semantics the numbers are meant for (check_door_clearance.py --approach, README): release end <= t_edge(anchor) - margin, release >= 0.15 s
  or no planar hold, jump path capped the same way (see check_door_clearance.hold_visible).

Merge rules (all checked BEFORE door_timing.json is written; a rejected merge leaves it byte-identical): every fragment must be feasible, carry
at_close_after.door_clearance_ok == true, not be LATE (close_after_s + event_clip_s > clip_end_s + 0.5, as build_car.ps1), match the timing table
(vanilla_close_after_s), have been solved with the full settings (V lag set == required_lags: v_lag_set_s + the class lag T + clip gap - clip start, T = the vanilla
slide of the class incl. the measured spread, Turbo-R 0.82 s; margin_s and window_s == door_timing.json; tol_cm recorded and <= 1.0; open / opening tolerance == the
documented waivers; the 'rear' phase for the classes with rear-data), appear once per class / seat / gender; both genders (pwa + pma) of every class / seat touched must be in the merge
(--allow-single-gender lifts that); without --reset the existing entries of every touched seat are dropped first (no stale gender survives).
A directory argument is globbed for door-close-*.json only.
"""
import argparse
import glob
import json
import math
import os
import re
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TIMING = os.path.join(HERE, 'door_timing.json')
DOOR_DATA = os.path.join(os.path.join(project_root(), r'Extracted\game\vehicle-entry-car'), 'door-data')
REDS = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'scripts', 'utpp_vehicle_door_timing.reds'))
VAR_CLASS = {'utility': 'utility4'}
SEAT_SIDE = {'driver': 'front_left', 'passenger_r': 'front_right'}
DOOR_SIDE = {'driver': 'left', 'passenger_r': 'right'}
SEAT_SIDE_BY_DOOR = {'left': 'front_left', 'right': 'front_right'}
GENDERS = ('pwa', 'pma')
LATE_SLACK_S = 0.5  # build_car.ps1: a close later than the clip end + 0.5 s is refused
GATE_TOL_CM = 1.0  # check_door_clearance.py --tol-cm: the gate floor of the closed / opening / closing phases (a fragment solved with a larger one is refused)
OPEN_TOL_CM = 2.0  # --open-tol-cm default (the fully open door)
# Documented opening / open tolerance waivers (build_car.ps1 doorWaive, README 'Model limits'): (class, gender or '*', seat) -> cm. A fragment must have been solved
# with exactly these (never with a bigger one); they never cover the closing sweep / the closed door.
DOOR_WAIVERS = {('sport2', 'pwa', 'passenger_r'): 6.0}  # the sport1 (Turbo-R) waivers 15 / 8.5 cm were removed in pk-door7 (they gated nothing in the opening / open phases)
REAR_CLASSES = ('standard2', 'standard25', 'standard3')  # classes with rear-data (extract_door_data.py --rear): their fragments must carry the 'rear' phase
APPROACH_FIXTURES = os.path.join(HERE, 'approach_fixtures.json')
DOOR_APPROACH = os.path.join(HERE, 'door_approach.py')


def slide_t_s(cls):
    """Vanilla slide time T (s) of the class the clip gate has to cover: the larger of door_approach.py SLIDE_T (parsed, not imported: that file is edited
    elsewhere; unlisted classes 0.30 like door_approach) and the largest measured sample of approach_fixtures.json slide_T_samples (the measured spread)."""
    import ast
    import re
    t = 0.30
    try:
        m = re.search(r'^SLIDE_T\s*=\s*(\{[^}]*\})', open(DOOR_APPROACH, encoding='utf-8').read(), re.M)
        t = float(ast.literal_eval(m.group(1)).get(cls, 0.30))
    except (OSError, AttributeError, ValueError, SyntaxError):
        pass
    try:
        t = max([t] + [float(x) for x in json.load(open(APPROACH_FIXTURES, encoding='utf-8')).get('slide_T_samples', {}).get(cls, [])])
    except (OSError, ValueError):
        pass
    return t


def clip_gap_s():
    """door_approach.py CLIP_GAP_S (clip time 0 = mount + T + this); parsed, 0.02 when unreadable."""
    import re
    try:
        return float(re.search(r'^CLIP_GAP_S\s*=\s*([0-9.]+)', open(DOOR_APPROACH, encoding='utf-8').read(), re.M).group(1))
    except (OSError, AttributeError, ValueError):
        return 0.02


def class_lag_s(tj, cls):
    """The V lag (s, V's body behind the model clock, ms rounded UP) that the measured clip clock of the class implies: the clip's frame 0 plays at
    mount + T + clip gap, the model clock starts at mount + clip_start_after_mount_s, so lag = T + gap - clip_start_after_mount_s."""
    return math.ceil((slide_t_s(cls) + clip_gap_s() - float(tj['clip_start_after_mount_s'])) * 1000.0 - 1e-6) / 1000.0


def required_lags(tj, cls):
    """The V lag set the door verdict / solve of a class must cover: door_timing.json v_lag_set_s plus the class lag when it exceeds all of them."""
    own = tj.get('classes', {}).get(cls, {}).get('lag_set_s')
    if own:  # a class whose measured clip clock excludes the legacy lags (door_timing.json classes.<c>.lag_set_s + lag_set_note), used as it is
        return sorted(set(round(float(x), 3) for x in own))
    base = sorted(set(round(float(x), 3) for x in tj.get('v_lag_set_s', [tj.get('v_lag_s', 0.0)])))
    extra = class_lag_s(tj, cls)
    if extra > max(base) + 1e-9:
        base.append(extra)
    return base


def vanilla_close_after(tj, cls, seat):
    """vanilla close delay after the door event of the class / door side of the seat (the redscript's evt.autoCloseTime equivalent)."""
    d = tj['classes'][cls][DOOR_SIDE[seat]]
    return d['vanilla_close_start_s'] - d['open_start_s']


def seat_delta(tj, cls, seat):
    """Seconds to ADD to the vanilla close delay for the seat (>= 0, ms rounded UP), or None without clip entries."""
    ents = tj.get('clips', {}).get(cls, {}).get(seat, {})
    vals = [g['close_after_s'] for g in ents.values() if g.get('close_after_s') is not None]
    if not vals:
        return None
    d = max(vals) - vanilla_close_after(tj, cls, seat)
    return max(0.0, math.ceil(d * 1000.0 - 1e-6) / 1000.0)


def validate_fragment(fr, path, tj=None):
    """Raise SystemExit with the reason when a --solve fragment must not enter the table / a staged set. Returns (class, seat, gender)."""
    for k in ('class', 'seat', 'gender'):
        if k not in fr:
            raise SystemExit('fragment %s: no "%s" (not a check_door_clearance.py --solve fragment)' % (path, k))
    if not fr.get('feasible') or fr.get('close_after_s') is None:
        raise SystemExit('fragment %s has no feasible close time (the seated pose collides with the closing door)' % path)
    if (fr.get('at_close_after') or {}).get('door_clearance_ok') is not True:
        raise SystemExit('fragment %s: the full door verdict at its close_after_s is not PASS (at_close_after.door_clearance_ok != true)' % path)
    if 'event_clip_s' not in fr or 'clip_end_s' not in fr:
        raise SystemExit('fragment %s: no event_clip_s / clip_end_s (cannot check LATE)' % path)
    if fr['close_after_s'] + fr['event_clip_s'] > fr['clip_end_s'] + LATE_SLACK_S:
        raise SystemExit('fragment %s: LATE - close_after %.3f s + event %.3f s is later than the clip end %.3f s + %.1f s' % (
            path, fr['close_after_s'], fr['event_clip_s'], fr['clip_end_s'], LATE_SLACK_S))
    if fr['seat'] not in DOOR_SIDE or fr['gender'] not in GENDERS:
        raise SystemExit('fragment %s: unknown seat / gender %s / %s' % (path, fr['seat'], fr['gender']))
    if tj is not None:
        try:
            van = vanilla_close_after(tj, fr['class'], fr['seat'])
        except KeyError:
            raise SystemExit('fragment %s: class %s is not in door_timing.json' % (path, fr['class']))
        if abs(van - fr['vanilla_close_after_s']) > 0.0015:
            raise SystemExit('fragment %s: solved against vanilla close %.3f s, door_timing.json says %.3f s (stale fragment)' % (path, fr['vanilla_close_after_s'], van))
        # solved with the full settings? (a weaker lag set / margin / window / tolerance / no rear gate would let a clip into the table that the gate refuses)
        ac = fr.get('at_close_after') or {}
        want = required_lags(tj, fr['class'])
        got = fr.get('lags_s') or ac.get('lags_s')
        if got is None or [round(float(x), 3) for x in got] != want:
            raise SystemExit('fragment %s: solved with the V lag set %s, required %s (v_lag_set_s + the class lag T + gap - clip start; re-solve)' % (path, got, want))
        if abs(float(fr['margin_s']) - float(tj['margin_s'])) > 1e-9:
            raise SystemExit('fragment %s: solved with margin_s %s, door_timing.json says %s' % (path, fr['margin_s'], tj['margin_s']))
        win = fr.get('window_s', ac.get('window_s'))
        if win is None or abs(float(win) - float(tj['window_s'])) > 1e-9:
            raise SystemExit('fragment %s: solved with window_s %s, door_timing.json says %s' % (path, win, tj['window_s']))
        if fr.get('tol_cm') is None or float(fr['tol_cm']) > GATE_TOL_CM + 1e-9:
            raise SystemExit('fragment %s: solved with tol_cm %s (recorded? required <= %.1f; re-solve with the current check_door_clearance.py)' % (path, fr.get('tol_cm'), GATE_TOL_CM))
        wv = DOOR_WAIVERS.get((fr['class'], fr['gender'], fr['seat']), DOOR_WAIVERS.get((fr['class'], '*', fr['seat'])))
        want_open = OPEN_TOL_CM if wv is None else wv
        want_opening = GATE_TOL_CM if wv is None else wv
        if fr.get('open_tol_cm') is None or fr.get('opening_tol_cm') is None or abs(float(fr['open_tol_cm']) - want_open) > 1e-9 or abs(float(fr['opening_tol_cm']) - want_opening) > 1e-9:
            raise SystemExit('fragment %s: solved with open / opening tolerance %s / %s cm, the documented values are %s / %s cm' % (
                path, fr.get('open_tol_cm'), fr.get('opening_tol_cm'), want_open, want_opening))
        if fr['class'] in REAR_CLASSES and 'rear' not in (ac.get('phases') or {}):
            raise SystemExit('fragment %s: class %s has rear-data but the fragment has no "rear" phase (solved with --no-rear or without rear-data; re-solve)' % (path, fr['class']))
    return fr['class'], fr['seat'], fr['gender']


def merge(paths, reset=False, allow_single_gender=False):
    raw = open(TIMING, encoding='utf-8').read()
    tj = json.loads(raw)
    files = []
    for p in paths:
        files += sorted(glob.glob(os.path.join(p, 'door-close-*.json'))) if os.path.isdir(p) else sorted(glob.glob(p))
    if not files:
        raise SystemExit('no fragment matches ' + ' '.join(paths))
    frs, seen = [], {}
    for f in files:
        fr = json.load(open(f, encoding='utf-8'))
        key = validate_fragment(fr, f, tj)
        if key in seen:
            raise SystemExit('fragment %s: %s/%s/%s already given by %s' % (f, key[0], key[1], key[2], seen[key]))
        seen[key] = f
        frs.append((f, fr))
    seats = sorted(set((c, s) for c, s, _ in seen))
    if not allow_single_gender:
        for c, s in seats:
            miss = [g for g in GENDERS if (c, s, g) not in seen]
            if miss:
                raise SystemExit('merge touches %s/%s but has no fragment for %s (both genders are required; --allow-single-gender)' % (c, s, ','.join(miss)))
    if reset:
        tj['clips'] = {}  # no stale entry of an earlier set survives
    else:
        for c, s in seats:
            tj.get('clips', {}).get(c, {}).pop(s, None)  # the touched seat is replaced as a whole
    for f, fr in frs:
        d = tj.setdefault('clips', {}).setdefault(fr['class'], {}).setdefault(fr['seat'], {})
        d[fr['gender']] = {'close_after_s': fr['close_after_s'], 'earliest_close_after_s': round(fr['earliest_close_after_s'], 3),
                           'vanilla_close_after_s': round(fr['vanilla_close_after_s'], 3), 'timing_status': fr['timing_status'],
                           'v_lag_s': fr.get('at_close_after', {}).get('v_lag_s'), 'lags_s': fr.get('lags_s'), 'margin_s': fr['margin_s'], 'tol_cm': fr.get('tol_cm'),
                           'clip': os.path.basename(fr.get('clip', '')), 'clip_sha256': fr.get('clip_sha256')}  # r10: the sha256 of the round-tripped clip the verdict was solved on (check_pack_manifest.py ties it to the packed clip)
    json.dump(tj, open(TIMING, 'w', encoding='utf-8'), indent=2)
    print('merged %d fragments (%d seats) into %s' % (len(frs), len(seats), TIMING))


def stamp_sha(paths, allow_missing_entry=False):
    """--stamp-sha: add 'clip_sha256' to EXISTING door_timing.json entries from verdict fragments (door-close-*.json) WITHOUT touching any timing value. A fragment is accepted only when
    its clip name equals the entry's and the entry's close_after_s is NOT EARLIER than the fragment's solved close (0.5 ms; a later stored close is conservative - the door closes later - and is
    printed): the sha is then the file the stored close was solved on (rows merged before r10 carry no sha). A seat with an entry for only one gender (--allow-single-gender merge) has no entry
    to stamp for the other gender: that fragment is reported as 'no entry' (not an error) when --allow-missing-entry is given."""
    tj = json.load(open(TIMING, encoding='utf-8'))
    files = []
    for p in paths:
        files += sorted(glob.glob(os.path.join(p, 'door-close-*.json'))) if os.path.isdir(p) else sorted(glob.glob(p))
    if not files:
        raise SystemExit('no fragment matches ' + ' '.join(paths))
    bad, n = [], 0
    for f in files:
        fr = json.load(open(f, encoding='utf-8'))
        e = tj.get('clips', {}).get(fr.get('class'), {}).get(fr.get('seat'), {}).get(fr.get('gender'))
        if e is None:
            if allow_missing_entry:
                print('no door_timing entry %s/%s/%s (single-gender seat, covered by the other gender): not stamped' % (fr.get('class'), fr.get('seat'), fr.get('gender')))
            else:
                bad.append('%s: no door_timing entry %s/%s/%s' % (f, fr.get('class'), fr.get('seat'), fr.get('gender')))
            continue
        sha = str(fr.get('clip_sha256') or '').upper()
        if not re.fullmatch(r'[0-9A-F]{64}', sha):
            bad.append('%s: fragment has no clip_sha256' % f)
        elif e.get('clip') != os.path.basename(fr.get('clip', '')):
            bad.append('%s: clip %s != entry clip %s' % (f, os.path.basename(fr.get('clip', '')), e.get('clip')))
        elif float(e['close_after_s']) < float(fr['close_after_s']) - 0.0005:
            bad.append('%s: door_timing.json close_after_s %.3f is EARLIER than the solved %.3f (%s/%s/%s)' % (f, e['close_after_s'], fr['close_after_s'], fr['class'], fr['seat'], fr['gender']))
        elif e.get('clip_sha256') not in (None, sha):
            bad.append('%s: entry already carries another clip_sha256 (%s)' % (f, e.get('clip_sha256')))
        else:
            if float(e['close_after_s']) > float(fr['close_after_s']) + 0.0005:
                print('conservative: %s/%s/%s door_timing.json close %.3f s is later than the solved %.3f s' % (fr['class'], fr['seat'], fr['gender'], e['close_after_s'], fr['close_after_s']))
            e['clip_sha256'] = sha
            n += 1
    if bad:
        raise SystemExit('stamp-sha refused (door_timing.json untouched):\n  ' + '\n  '.join(bad))
    json.dump(tj, open(TIMING, 'w', encoding='utf-8'), indent=2)
    print('stamped %d entries with clip_sha256 in %s' % (n, TIMING))


def table(tj):
    """{seat variable: (close_after_s shipped, delta s)} - close_after = vanilla + delta of the seat's class / door side."""
    rows = {}
    for cls, seats in tj.get('clips', {}).items():
        for seat in seats:
            d = seat_delta(tj, cls, seat)
            if d is None:
                continue
            rows['%s_vehicle_seat_%s' % (VAR_CLASS.get(cls, cls), SEAT_SIDE[seat])] = (vanilla_close_after(tj, cls, seat) + d, d)
    return rows


ARC_PHI = (0.10, 0.95)  # swing fractions the smoothstep is fitted on (the tails of the animation are flat: a tiny angle change is a large time change)
ARC_HINGE_TOL_M = 0.05  # |door-data hinge - independent probe hinge| (the probe reads the door component position through CET, 2-5 cm noise)
ARC_SIDES = ('left', 'right')
# Independent hinge readings: median over all closed-door ('yaw ~ 0') probe positions of the car-door2 session 2026-11-02
# (probe log lines <vehicle>\*-lines.txt: door fl / fr moving ... from rest | settled ... yaw=-0.0 pos=(x,y,z)); vehicle frame. Turbo-R is not compared (lifting door).
PROBE_HINGE_XY = {
    'standard2': {'left': (-0.985, 1.190), 'right': (1.000, 1.180)},
    'standard25': {'left': (-0.930, 1.370), 'right': (0.950, 1.370)},
    'standard3': {'left': (-0.970, 0.950), 'right': (1.010, 0.950)},
    'sport2': {'left': (-0.830, 0.665), 'right': (0.870, 0.660)},
    'utility': {'left': (-1.515, 4.520), 'right': (1.560, 4.490)},
}
GLTF_TO_VEHICLE = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))  # vehicle = M @ glTF: (gx, gy, gz) -> (gx, -gz, gy)


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def smoothstep_inv(y):
    lo, hi = 0.0, 1.0
    for _ in range(60):
        m = 0.5 * (lo + hi)
        if smoothstep(m) < y:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi)


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ---- door arc invariant (2026-11-02, review fix): the redscript cap model against the door-data points ---------------------------------------------
# Constants of the invariant. The reds side (utpp_vehicle_door.reds UTPPDoorCapOf) uses a 0.20 m disk (body 0.15 + margin 0.05) and cap = t_edge - 0.05;
# the invariant compares the model's t_edge with the first time the DATA comes within INV_CONTACT_M (the bare body radius), so the 0.05 m disk margin
# and the 0.05 s cap margin of the reds are real extra safety on top of "never later than the data".
INV_CONTACT_M = 0.15  # planar distance door-data point <-> anchor that counts as the door touching the body
INV_BODY_M = 0.20  # UTPPDoorCapBodyM()
INV_KEPT_S = 1.5  # UTPPDoorCapHoldKeptS() (0.75 until 2026-11-03, native glide limit)
INV_R_MIN = 0.40  # nearest anchor to the hinge axis (V's body radius 0.15 m + the car side: nothing closer can be a pre-mount spot)
INV_THETA_LO = -20.0  # anchor angle grid, deg from the data's trailing-edge direction (toward the opening positive)
INV_THETA_PAD = 25.0  # ... up to |peak swing| + INV_LEAD_CAP + this
INV_LEAD_CAP = 40.0
INV_OUT_M = 0.10  # an anchor (V's pre-mount spot) must lie at least this far outboard of the outermost door point (= the car side): |x| >= |x_side| + this
INV_DR, INV_DTH = 0.05, 1.5  # grid steps (m, deg); grid 1 is staggered by half a step in both (the fit uses grid 0, --check-arcs-data checks both)
INV_DUR_SAFETY = 0.95  # the fitted duration is multiplied by this before the ms floor (anchors / door frames the fit grids do not sample)
LEAD_STEP = 0.25
INV_LEAD_GUARD = 0.5  # a rotation is only taken when L - 0.5 and L + 0.5 are feasible too (not on the edge of the feasible band)
INV_DUR_MIN = 0.10  # shortest duration accepted (shorter = an absurdly early model)


def _round_arc(a):
    """The arc as the redscript renders it (3 decimals, closedDirDeg / openDeg 2): the invariant judges these numbers, not the raw floats."""
    return {'hingeX': round(float(a['hingeX']), 3), 'hingeY': round(float(a['hingeY']), 3), 'length': round(float(a['length']), 3),
            'closedDirDeg': round(float(a['closedDirDeg']), 2), 'openDeg': round(float(a['openDeg']), 2),
            'openDelayS': round(float(a['openDelayS']), 3), 'openDurS': round(float(a['openDurS']), 3)}


def _inv_smoothstep_np(s):
    import numpy as np
    c = np.clip(s, 0.0, 1.0)
    return 0.5 - np.sin(np.arcsin(1.0 - 2.0 * c) / 3.0)  # = UTPPDoorInvSmoothstep


def reds_geom_np(a, X, Y):
    """Vectorised first half of UTPPDoorCapOf for anchors (X, Y) in the vehicle frame: (why, x) with why 0 = door reaches the footprint,
    2 = anchor beyond the door radius, 3 = the door never reaches the footprint; x = max(theta - delta, 0) / |openDeg| (valid where why == 0)."""
    import numpy as np
    dx, dy = X - a['hingeX'], Y - a['hingeY']
    r = np.hypot(dx, dy)
    cd = math.radians(a['closedDirDeg'])
    ax, ay = math.cos(cd), math.sin(cd)
    dot = ax * dx + ay * dy
    cross = ax * dy - ay * dx
    ang = np.degrees(np.arccos(np.clip(dot / np.maximum(r, 0.0001), -1.0, 1.0)))
    ang = np.where(cross < 0.0, -ang, ang)
    sgn = -1.0 if a['openDeg'] < 0.0 else 1.0
    th = ang * sgn
    delta = np.degrees(np.arcsin(np.minimum(1.0, INV_BODY_M / np.maximum(r, INV_BODY_M))))
    op = abs(a['openDeg'])
    why = np.zeros(len(X), dtype=int)
    why[(th + delta < 0.0) | (th - delta >= op)] = 3
    why[r > a['length'] + INV_BODY_M] = 2
    x = np.maximum(th - delta, 0.0) / op
    return why, x


def reds_cap_np(a, X, Y):
    """UTPPDoorCapOf for many anchors: (why, t_edge) with why 0 = cap, 2 / 3 as above, 4 = t_edge > INV_KEPT_S (t_edge is +inf for why 2 / 3)."""
    import numpy as np
    why, x = reds_geom_np(a, X, Y)
    t = np.where(why == 0, a['openDelayS'] + a['openDurS'] * _inv_smoothstep_np(x), np.inf)
    why = np.where((why == 0) & (t > INV_KEPT_S), 4, why)
    return why, t


def door_geometry(dj, side):
    """Door-data of one front door as planar vehicle-frame points per startup frame (all parts) + the rear-most-point swing series (the pre-review arc)."""
    import numpy as np
    sys.path.insert(0, HERE)
    import skel as S
    M = np.array(GLTF_TO_VEHICLE)
    sd = dj['door'][side]
    su = int(sd['phase_frames']['startup'][1])
    parts = []
    for p in sd['parts']:
        parts.append((np.array(p['frames_t']), np.array(p['frames_q']), np.array(p['points'])))
    ft0, fq0, pts0 = parts[0]
    H = np.array([(M @ ft0[f]) for f in range(su + 1)])  # hinge (door bone origin) per frame, 3D
    h0 = H[0]
    W = []
    for f in range(su + 1):
        W.append(np.concatenate([(M @ ((S.q2m(fq[f]) @ pts.T).T + ft[f]).T).T[:, :2] for ft, fq, pts in parts]))
    # the pre-review arc: the rear-most point of the door skin (parts[0]) defines the closed direction; its angle series is the swing
    wv = (M @ ((S.q2m(fq0[0]) @ pts0.T).T + ft0[0]).T).T
    d = wv[:, :2] - h0[:2]
    r0 = np.linalg.norm(d, axis=1)
    i = int(np.argmax(r0))
    cdir = math.degrees(math.atan2(d[i, 1], d[i, 0]))
    ang, hinge_move = [], 0.0
    for f in range(su + 1):
        e = M @ (S.q2m(fq0[f]) @ pts0[i] + ft0[f])
        dd = e[:2] - H[f][:2]
        hinge_move = max(hinge_move, float(np.linalg.norm(H[f][:2] - h0[:2])))
        ang.append(_wrap180(math.degrees(math.atan2(dd[1], dd[0])) - cdir))
    ang = np.array(ang)
    k = int(np.argmax(np.abs(ang)))
    length = max(float(np.linalg.norm(W[f] - H[f][:2], axis=1).max()) for f in range(su + 1))
    side_sgn = -1.0 if h0[0] < 0 else 1.0
    x_side = float(max(side_sgn * W[0][:, 0]))  # outermost lateral extent of the closed door (|x|): the car side
    return {'su': su, 'H': H, 'W': W, 'x_side': x_side, 'side_sgn': side_sgn, 'cdir0': cdir, 'ang': ang, 'peak': float(ang[k]), 'peak_frame': k, 'settled': float(ang[su]),
            'hinge_move': hinge_move, 'length': length, 'length_rearmost': float(r0[i])}


def anchor_grid(g, frac=0.0):
    """The anchor grid of the invariant, polar about the rendered hinge, angles from the DATA's trailing-edge direction (not from the arc under test:
    old and new arcs face the same anchors); `frac` shifts the grid by that fraction of a step in r and theta. Returns X, Y, theta0 (deg, toward the
    opening positive), r."""
    import numpy as np
    sgn = 1.0 if g['peak'] > 0 else -1.0
    h = np.round(g['H'][0][:2], 3)
    rs = np.arange(INV_R_MIN + frac * INV_DR, g['length'] + INV_BODY_M + 1e-9, INV_DR)
    ths = np.arange(INV_THETA_LO + frac * INV_DTH, abs(g['peak']) + INV_LEAD_CAP + INV_THETA_PAD + 1e-9, INV_DTH)
    R, T = np.meshgrid(rs, ths, indexing='ij')
    a = np.radians(g['cdir0'] + sgn * T)
    return (h[0] + R * np.cos(a)).ravel(), (h[1] + R * np.sin(a)).ravel(), T.ravel(), R.ravel()


def _min_dist_capped(P, A, cell=0.30):
    """Planar min distance of every anchor A (n, 2) to the points P (N, 2), exact up to `cell` (cells of `cell` m, 3 x 3 neighbourhood), `cell` beyond."""
    import numpy as np
    off, mul = 1000, 4096
    pc = np.floor(P / cell).astype(np.int64)
    ac = np.floor(A / cell).astype(np.int64)
    pk = (pc[:, 0] + off) * mul + (pc[:, 1] + off)
    ak = (ac[:, 0] + off) * mul + (ac[:, 1] + off)
    order = np.argsort(pk, kind='stable')
    pks = pk[order]
    out = np.full(len(A), cell)
    for k in np.unique(ak):
        ai = np.flatnonzero(ak == k)
        cx, cy = int(k) // mul, int(k) % mul
        sel = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                kk = (cx + dx) * mul + (cy + dy)
                lo, hi = np.searchsorted(pks, kk, 'left'), np.searchsorted(pks, kk, 'right')
                if hi > lo:
                    sel.append(order[lo:hi])
        if not sel:
            continue
        Q = P[np.concatenate(sel)]
        d = np.sqrt(((A[ai, None, :] - Q[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
        out[ai] = np.minimum(d, cell)
    return out


def touch_times(g, delay, open_dur, X, Y):
    """First time (s after the mount line: delay + startup frame mapped linearly onto open_dur) at which ANY door point of ANY part comes within
    INV_CONTACT_M (planar) of each anchor; linear interpolation of the distance between the two frames (distances beyond 0.30 m are clipped to 0.30, which
    only moves the estimate EARLIER). +inf = never during the startup phase, nan = already within INV_CONTACT_M with the door closed (frame 0: V stands
    in the door, no cap can help; exempt)."""
    import numpy as np
    su = g['su']
    n = len(X)
    A = np.stack([X, Y], axis=1)
    D = np.empty((su + 1, n))
    for f in range(su + 1):
        D[f] = _min_dist_capped(g['W'][f], A)
    tt = delay + np.arange(su + 1) / float(su) * open_dur
    hit = D <= INV_CONTACT_M
    first = hit.argmax(axis=0)
    anyhit = hit.any(axis=0)
    t = np.full(n, np.inf)
    for j in np.flatnonzero(anyhit & (first > 0)):
        f = int(first[j])
        d1, d2 = D[f - 1, j], D[f, j]
        t[j] = tt[f - 1] + (d1 - INV_CONTACT_M) / (d1 - d2) * (tt[f] - tt[f - 1])
    t[hit[0]] = np.nan
    return t


FIT_FRACS = (0.0, 0.5)  # grids the lead / duration are fitted on
CHECK_FRACS = (0.0, 0.25, 0.5, 0.75)  # grids the invariant is checked on (0.25 / 0.75 are not seen by the fit)


def grid_tt(g, delay, open_dur, frac, cache):
    """(X, Y, theta0, r, touch times, n outside-domain) of one grid, restricted to the DOMAIN of the invariant: anchors outboard of the car side
    (side_sgn * x >= x_side + INV_OUT_M; an anchor inside the car body is not a place V can stand). cache = dict per door / timing, so the expensive touch
    table is computed once per process."""
    import numpy as np
    key = (round(float(delay), 6), round(float(open_dur), 6), frac)
    if key not in cache:
        X, Y, T0, R = anchor_grid(g, frac)
        keep = g['side_sgn'] * X >= g['x_side'] + INV_OUT_M
        X, Y, T0, R = X[keep], Y[keep], T0[keep], R[keep]
        cache[key] = (X, Y, T0, R, touch_times(g, delay, open_dur, X, Y), int((~keep).sum()))
    return cache[key]


def arc_invariant(g, delay, open_dur, arc, fracs=CHECK_FRACS, cache=None):
    """The invariant: for every anchor of the grids that the door reaches (touch time finite, i.e. not already inside the closed door) the redscript
    model (UTPPDoorCapOf on the RENDERED numbers of `arc`) must give why == 0 or 4 with t_edge <= the first touch time of the data. Returns a dict
    with the violation count, the worst violations and the earliness statistics (touch time - model t_edge, > 0 = model early = safe)."""
    import numpy as np
    cache = {} if cache is None else cache
    a = _round_arc(arc)
    n_all = n_touch = n_exempt = n_outside = 0
    bad_rows, early = [], []
    for fr in fracs:
        X, Y, T0, R, tt, n_out = grid_tt(g, delay, open_dur, fr, cache)
        n_outside += n_out
        why, te = reds_cap_np(a, X, Y)
        m = np.isfinite(tt)
        late = m & (te > tt + 1e-9)
        n_all += len(X)
        n_touch += int(m.sum())
        n_exempt += int(np.isnan(tt).sum())
        with np.errstate(invalid='ignore'):
            early.append((tt - te)[m & ~late])
        for j in np.flatnonzero(late):
            bad_rows.append((-(1e9 if np.isinf(te[j]) else te[j] - tt[j]), {'x': round(float(X[j]), 3), 'y': round(float(Y[j]), 3), 'r': round(float(R[j]), 3),
                             'theta0': round(float(T0[j]), 1), 'why': int(why[j]), 't_model': None if np.isinf(te[j]) else round(float(te[j]), 3),
                             't_data': round(float(tt[j]), 3)}))
    early = np.concatenate(early) if early else np.zeros(0)
    bad_rows.sort(key=lambda r: r[0])
    return {'anchors': n_all, 'touched': n_touch, 'exempt_in_closed_door': n_exempt, 'outside_domain_inside_car': n_outside, 'violations': len(bad_rows), 'worst': [r[1] for r in bad_rows[:5]],
            'early_median_s': round(float(np.median(early)), 3) if len(early) else None, 'early_max_s': round(float(early.max()), 3) if len(early) else None}


def fit_lead(g, delay, open_dur, dur_cap, cache=None):
    """Candidate rotations L (deg, toward the opening, LEAD_STEP) of the closed direction, each with the largest duration for which the invariant holds
    on the FIT grids (capped at dur_cap, times INV_DUR_SAFETY, floored to the ms) and the median earliness of the resulting model over the anchors the door
    reaches (smaller = closer to the data). Returns table = [(L, feasible, dur, median earliness)]."""
    import numpy as np
    cache = {} if cache is None else cache
    parts = [grid_tt(g, delay, open_dur, fr, cache) for fr in FIT_FRACS]
    X = np.concatenate([p[0] for p in parts])
    Y = np.concatenate([p[1] for p in parts])
    tt = np.concatenate([p[4] for p in parts])
    m = np.isfinite(tt)
    sgn = 1.0 if g['peak'] > 0 else -1.0
    op = round(abs(g['peak']), 2)
    table = []
    for L in np.arange(0.0, 45.0 + 1e-9, LEAD_STEP):
        a = _round_arc({'hingeX': g['H'][0][0], 'hingeY': g['H'][0][1], 'length': g['length'], 'closedDirDeg': g['cdir0'] + sgn * L,
                        'openDeg': sgn * op, 'openDelayS': delay, 'openDurS': 1.0})
        why, x = reds_geom_np(a, X, Y)
        if (m & (why != 0)).any():
            table.append((float(L), False, None, None))
            continue
        inv = _inv_smoothstep_np(x[m])
        room = tt[m] - delay
        need = inv > 1e-9
        dmax = float(np.min(room[need] / inv[need])) if need.any() else 1.5
        dur = min(float(dur_cap), math.floor(dmax * INV_DUR_SAFETY * 1000.0) / 1000.0)
        med = float(np.median(room - dur * inv))
        table.append((float(L), True, dur, med))
    return table


def door_arc_from_data(dj, side, timing_side, open_delay, g=None, cache=None):
    """Arc numbers of one front door from a door-data json (extract_door_data.py). valid=False for a lifting door.
    Review fix: the closed direction is the trailing-edge direction (rear-most point of the skin) rotated toward the opening by a lead fitted on all points
    of all parts (radius dependent: 10-26 deg on the 4-door classes), length = the largest radius over all parts and frames, the duration is fitted so the
    redscript's t_edge is never later than the first time any door-data point of any part is within INV_CONTACT_M of the anchor (arc_invariant)."""
    import numpy as np
    g = g or door_geometry(dj, side)
    cache = {} if cache is None else cache
    h0 = g['H'][0]
    peak = g['peak']
    out = {'hingeX': round(float(h0[0]), 4), 'hingeY': round(float(h0[1]), 4), 'hingeZ': round(float(h0[2]), 4), 'length': round(g['length'], 4),
           'closedDirDeg': round(g['cdir0'], 3), 'openDeg': round(peak, 2), 'settledDeg': round(g['settled'], 2), 'peakFrame': g['peak_frame'],
           'startupFrames': g['su'], 'hingeMoveM': round(g['hinge_move'], 4), 'openDelayS': round(float(open_delay), 4)}
    if g['hinge_move'] > 0.02 or abs(peak) < 30.0:
        out['valid'] = False
        out['invalid_reason'] = 'lifting door: the hinge travels %.2f m during the startup phase, swing %.1f deg (no swing-door arc)' % (g['hinge_move'], peak)
        return out
    out['valid'] = True
    su = g['su']
    od = float(timing_side['open_dur_s'])
    # (1) pre-review fit on the rear-most point's swing: the model never reaches a swing fraction later than the animation of that point
    runmax = np.maximum.accumulate(np.abs(g['ang']) / abs(peak))
    tt_anim = np.arange(su + 1) / float(su) * od
    phis = np.linspace(ARC_PHI[0], ARC_PHI[1], 18)
    t_anim = np.interp(phis, runmax, tt_anim)
    inv = np.array([smoothstep_inv(p) for p in phis])
    dur_anim = math.floor(float(np.min(t_anim / inv)) * 1000.0) / 1000.0
    # (2) the lead and the duration from the invariant over every point of every part (planar, 0.15 m)
    table = fit_lead(g, float(open_delay), od, dur_anim, cache)
    ok = {round(l, 2): (dur, med) for l, f, dur, med in table if f}
    cands = sorted((med, l, dur) for l, (dur, med) in ok.items() if round(l - INV_LEAD_GUARD, 2) in ok and round(l + INV_LEAD_GUARD, 2) in ok and dur >= INV_DUR_MIN)
    sgn = 1.0 if peak > 0 else -1.0
    best = None
    for med, l, dur in cands:  # the least early rotation whose model also holds on the grids the fit did not see
        cand = dict(out, closedDirDeg=g['cdir0'] + sgn * l, openDurS=dur)
        if arc_invariant(g, float(open_delay), od, cand, CHECK_FRACS, cache)['violations'] == 0:
            best = (l, dur)
            break
    if best is None:
        out['valid'] = False
        out['invalid_reason'] = 'no rotation of the closed direction (0..45 deg) makes the cap model reach every anchor the door sweeps'
        return out
    L, dur = best
    out['trailingDirDeg'] = round(g['cdir0'], 3)
    out['leadDeg'] = round(L, 2)
    out['closedDirDeg'] = round(g['cdir0'] + sgn * L, 3)
    out['lengthRearMostM'] = round(g['length_rearmost'], 4)
    out['openDurS'] = round(dur, 3)
    early = t_anim - dur * inv
    out['fit'] = {'phi_range': list(ARC_PHI), 'anim_open_dur_s': od, 'max_early_s': round(float(early.max()), 3), 'max_late_s': round(float(max(0.0, -early.min())), 4),
                  'dur_rearmost_fit_s': round(dur_anim, 3), 'dur_safety': INV_DUR_SAFETY,
                  'rule': 'closedDir = trailing-edge direction rotated toward the opening by the lead (%.2f deg step) with the smallest median earliness among those for which '
                          'every anchor outboard of the car that the door reaches is covered (also at +-%.1f deg and on the unseen check grids); duration = min(rear-most-point fit, '
                          'INV_DUR_SAFETY * largest duration for which the cap model is never later than the data)' % (LEAD_STEP, INV_LEAD_GUARD)}
    return out


def _arc_task(job):
    """One door side (process pool worker): the fitted arc from the door-data and the invariant of the fresh and (if given) the stored arc, on shared grids."""
    cls, side, path, tside, delay, stored = job
    dj = json.load(open(path, encoding='utf-8'))
    g = door_geometry(dj, side)
    cache = {}
    fresh = door_arc_from_data(dj, side, tside, delay, g, cache)
    od = float(tside['open_dur_s'])
    res = {'cls': cls, 'side': side, 'fresh': fresh}
    if fresh.get('valid'):
        res['inv_fresh'] = arc_invariant(g, delay, od, fresh, CHECK_FRACS, cache)
    if stored is not None and stored.get('valid'):
        res['inv_stored'] = arc_invariant(g, delay, od, stored, CHECK_FRACS, cache)
    return res


def run_arc_jobs(tj, data_dir, stored=None, jobs=8):
    """-> [result dict of _arc_task per class / side] (process pool; stored = tj['arcs'] to judge the stored arcs as well)."""
    from concurrent.futures import ProcessPoolExecutor
    todo = []
    for cls in tj['classes']:
        path = os.path.join(data_dir, cls + '.json')
        for side in ARC_SIDES:
            lo = tj['classes'][cls].get('measured_open_start_range_s', [tj['classes'][cls][side]['open_start_s']])[0]
            todo.append((cls, side, path, tj['classes'][cls][side], lo, None if stored is None else stored.get(cls, {}).get(side)))
    with ProcessPoolExecutor(max_workers=max(1, min(jobs, len(todo)))) as ex:
        return list(ex.map(_arc_task, todo))


def compute_arcs(tj, data_dir, results=None):
    """The 'arcs' block of door_timing.json from the door-data (results = run_arc_jobs output, computed when None). Every valid arc carries the summary of
    its invariant check ('invariant')."""
    import hashlib
    results = results if results is not None else run_arc_jobs(tj, data_dir)
    by = {(r['cls'], r['side']): r for r in results}
    arcs = {'_note': 'front-door arcs, vehicle frame (x right, y forward); see the gen_door_timing.py docstring (DOOR ARC contract). Computed by gen_door_timing.py --arcs.'}
    for cls in tj['classes']:
        path = os.path.join(data_dir, cls + '.json')
        c = {'door_data_file': 'door-data\\%s.json' % cls, 'door_data_sha256': hashlib.sha256(open(path, 'rb').read()).hexdigest().upper()}
        for side in ARC_SIDES:
            r = by[(cls, side)]
            a = r['fresh']
            if 'inv_fresh' in r:
                iv = r['inv_fresh']
                a['invariant'] = {'contact_m': INV_CONTACT_M, 'body_m': INV_BODY_M, 'r_min_m': INV_R_MIN, 'outboard_m': INV_OUT_M, 'grids': list(CHECK_FRACS),
                                  'anchors': iv['anchors'], 'touched': iv['touched'], 'violations': iv['violations'],
                                  'early_median_s': iv['early_median_s'], 'early_max_s': iv['early_max_s']}
            pr = PROBE_HINGE_XY.get(cls, {}).get(side)
            if pr is not None:
                a['probe_hinge_xy'] = list(pr)
                a['probe_hinge_delta_m'] = round(math.hypot(a['hingeX'] - pr[0], a['hingeY'] - pr[1]), 4)
            c[side] = a
        arcs[cls] = c
    return arcs


def invariant_problems(results, key):
    """Problem lines for the invariant results of run_arc_jobs (key 'inv_stored' | 'inv_fresh'); prints one summary line per door."""
    bad = []
    for r in results:
        iv = r.get(key)
        if iv is None:
            continue
        print('  invariant %-5s %-10s %-5s: %d violations in %d anchors (%d reached by the door), model early by median %s s max %s s%s' % (
            key[4:], r['cls'], r['side'], iv['violations'], iv['anchors'], iv['touched'], iv['early_median_s'], iv['early_max_s'],
            '' if not iv['violations'] else '  worst %s' % iv['worst'][0]))
        if iv['violations']:
            bad.append('arcs: %s/%s breaks the cap invariant at %d anchors (worst %s)' % (r['cls'], r['side'], iv['violations'], iv['worst'][0]))
    return bad


def validate_arcs(tj):
    """Invariants of the stored arcs (no extraction needed). Returns a list of problems."""
    bad = []
    arcs = tj.get('arcs')
    if not arcs:
        return ['door_timing.json has no "arcs" (run gen_door_timing.py --arcs)']
    for cls in tj['classes']:
        if cls not in arcs:
            bad.append('arcs: class %s missing' % cls)
            continue
        for side in ARC_SIDES:
            a = arcs[cls].get(side)
            if a is None:
                bad.append('arcs: %s/%s missing' % (cls, side))
                continue
            if not a.get('valid'):
                if cls != 'sport1':
                    bad.append('arcs: %s/%s is not valid (%s); only the lifting Turbo-R door may be invalid' % (cls, side, a.get('invalid_reason')))
                continue
            if cls == 'sport1':
                bad.append('arcs: sport1/%s must be invalid (lifting door)' % side)
            want = -1.0 if side == 'left' else 1.0
            if a['openDeg'] * want <= 30.0:
                bad.append('arcs: %s/%s openDeg %.1f has the wrong sign / is too small (left must be negative = clockwise, right positive)' % (cls, side, a['openDeg']))
            if (a['hingeX'] < 0) != (side == 'left'):
                bad.append('arcs: %s/%s hinge x %.3f is on the wrong side of the vehicle' % (cls, side, a['hingeX']))
            pr = a.get('probe_hinge_xy')
            if pr is None:
                bad.append('arcs: %s/%s has no independent probe hinge' % (cls, side))
            elif math.hypot(a['hingeX'] - pr[0], a['hingeY'] - pr[1]) > ARC_HINGE_TOL_M:
                bad.append('arcs: %s/%s hinge is %.3f m from the probe hinge (> %.2f m)' % (cls, side, math.hypot(a['hingeX'] - pr[0], a['hingeY'] - pr[1]), ARC_HINGE_TOL_M))
            if not (0.9 < a['length'] < 1.6) or not (-130.0 < a['closedDirDeg'] < -50.0):
                bad.append('arcs: %s/%s length %.3f / closedDirDeg %.1f outside the plausible range of a front door that opens rearwards' % (cls, side, a['length'], a['closedDirDeg']))
            lo = tj['classes'][cls]['measured_open_start_range_s'][0]
            if abs(a['openDelayS'] - lo) > 1e-6:
                bad.append('arcs: %s/%s openDelayS %.4f != measured range lower bound %.4f (stale arcs)' % (cls, side, a['openDelayS'], lo))
            sg = -1.0 if side == 'left' else 1.0
            if 'leadDeg' not in a or 'trailingDirDeg' not in a:
                bad.append('arcs: %s/%s has no leadDeg / trailingDirDeg (arcs of the pre-review fit: re-run gen_door_timing.py --arcs)' % (cls, side))
            elif not (0.0 <= a['leadDeg'] <= 45.0) or abs(a['closedDirDeg'] - (a['trailingDirDeg'] + sg * a['leadDeg'])) > 0.01:
                bad.append('arcs: %s/%s closedDirDeg %.3f != trailingDirDeg %.3f + %s lead %.2f' % (cls, side, a['closedDirDeg'], a['trailingDirDeg'], '-' if sg < 0 else '+', a['leadDeg']))
            if (a.get('invariant') or {}).get('violations') != 0:
                bad.append('arcs: %s/%s has no passing cap invariant record (violations %s)' % (cls, side, (a.get('invariant') or {}).get('violations')))
            if not (0.05 <= a['openDurS'] <= 1.5) or a['fit']['max_late_s'] > 0.001:
                bad.append('arcs: %s/%s openDurS %.3f / fit lateness %.4f s: the smoothstep is later than the animation' % (cls, side, a['openDurS'], a['fit']['max_late_s']))
    return bad


def arc_rows(tj):
    """{seat variable: arcs entry} of the VALID front-door arcs."""
    rows = {}
    for cls, c in tj.get('arcs', {}).items():
        if cls.startswith('_'):
            continue
        for side in ARC_SIDES:
            a = c.get(side)
            if a and a.get('valid'):
                rows['%s_vehicle_seat_%s' % (VAR_CLASS.get(cls, cls), SEAT_SIDE_BY_DOOR[side])] = a
    return rows


def arc_of(tj, cls, side):
    """The valid arcs entry of a class / door side or None (check_door_clearance.py)."""
    a = tj.get('arcs', {}).get(cls, {}).get(side)
    return a if a and a.get('valid') else None


def render(tj):
    rows = table(tj)
    L = ['// GENERATED by Source\\tools\\anim\\vehicle-entry\\gen_door_timing.py from door_timing.json - do not edit',
         '// Extra seconds the front door close of a UTPP third-person car entry is delayed past the vanilla close: the redscript uses',
         '// closeAfter = evt.autoCloseTime + delta where delta > 0 (delta = 0: the vanilla close already clears V). Per seat: the max over the',
         '// female / male entry clip of (solved close_after - vanilla close after the door event of that class / door side), margin included',
         '// (door_timing.json: convention, v_lag_s, margin_s), rounded up to the ms. -1.0 = seat without UTPP timing.',
         '// Classes without a measured door ride (door_timing.json status derived_from_left / UNVERIFIED) use the swing-door / left-door numbers.',
         '//   %-42s %-12s %s' % ('seat variable', 'close_after_s', 'delta_s')]
    for k in sorted(rows):
        L.append('//   %-42s %-12.3f %.3f' % (k, rows[k][0], rows[k][1]))
    L += ['', 'public func UTPPCarDoorCloseDeltaS(seatVar: CName) -> Float {']
    for k in sorted(rows):
        L.append('  if Equals(seatVar, n"%s") { return %.3f; }' % (k, rows[k][1]))
    L += ['  return -1.0;', '}', '']
    arcs = arc_rows(tj)
    L += ['// Front-door swing arcs (door_timing.json "arcs", gen_door_timing.py docstring: DOOR ARC contract). Vehicle frame: x right, y forward, metres.',
          '// hinge = hinge axis XY; length = hinge -> rear edge of the closed door; closedDirDeg = atan2(dy, dx) of that direction (door closed, CCW positive);',
          '// openDeg = signed peak swing (left door negative, right door positive); the edge direction t seconds after the mount line is',
          '//   closedDirDeg + openDeg * smoothstep(clamp((t - openDelayS) / openDurS, 0, 1)).',
          '// openDelayS = earliest measured door start after the mount line; openDurS = never later than the door animation. valid = false: no swing arc',
          '// (Turbo-R lifting door, unknown seat variable).',
          '//   %-42s %8s %8s %7s %9s %8s %7s %7s' % ('seat variable', 'hingeX', 'hingeY', 'length', 'closedDir', 'openDeg', 'delay', 'dur')]
    for k in sorted(arcs):
        a = arcs[k]
        L.append('//   %-42s %8.3f %8.3f %7.3f %9.2f %8.2f %7.3f %7.3f' % (k, a['hingeX'], a['hingeY'], a['length'], a['closedDirDeg'], a['openDeg'], a['openDelayS'], a['openDurS']))
    L += ['', 'public struct UTPPCarDoorArc { public let valid: Bool; public let hingeX: Float; public let hingeY: Float; public let length: Float; '
          'public let closedDirDeg: Float; public let openDeg: Float; public let openDelayS: Float; public let openDurS: Float; }', '',
          'public func UTPPCarDoorArcOf(seatVar: CName) -> UTPPCarDoorArc {', '  let a: UTPPCarDoorArc;']
    for k in sorted(arcs):
        a = arcs[k]
        L.append('  if Equals(seatVar, n"%s") {' % k)
        L.append('    a.valid = true; a.hingeX = %.3f; a.hingeY = %.3f; a.length = %.3f; a.closedDirDeg = %.2f; a.openDeg = %.2f; a.openDelayS = %.3f; a.openDurS = %.3f;'
                 % (a['hingeX'], a['hingeY'], a['length'], a['closedDirDeg'], a['openDeg'], a['openDelayS'], a['openDurS']))
        L.append('    return a;')
        L.append('  }')
    L += ['  a.valid = false;', '  return a;', '}', '']
    return '\n'.join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--merge', nargs='+')
    ap.add_argument('--reset', action='store_true', help='with --merge: clear the clips table first')
    ap.add_argument('--allow-single-gender', action='store_true', help='with --merge: accept a seat with only one gender fragment')
    ap.add_argument('--stamp-sha', nargs='+', metavar='FRAGMENT', help='add clip_sha256 to existing entries from verdict fragments whose clip matches and whose solved close is not later than the entry (no timing value changes)')
    ap.add_argument('--allow-missing-entry', action='store_true', help='with --stamp-sha: a fragment of a gender without a door_timing entry (single-gender seat) is skipped, not an error')
    ap.add_argument('--emit-only', action='store_true')
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--arcs', action='store_true', help='(re)compute door_timing.json "arcs" from the extracted door-data')
    ap.add_argument('--check-arcs-data', action='store_true', help='exit 1 when the stored arcs differ from the extracted door-data')
    ap.add_argument('--door-data', default=DOOR_DATA)
    ap.add_argument('--timing', default=None, help='use this door_timing.json instead (checks / --arcs on a copy; not with --merge, whose output is the repo table)')
    ap.add_argument('--jobs', type=int, default=8, help='--arcs / --check-arcs-data: worker processes (one door side each)')
    a = ap.parse_args()
    global TIMING
    if a.timing:
        if a.merge:
            raise SystemExit('--timing: not with --merge')
        TIMING = os.path.abspath(a.timing)
    if a.merge:
        merge(a.merge, a.reset, a.allow_single_gender)
    if a.stamp_sha:
        stamp_sha(a.stamp_sha, a.allow_missing_entry)
    tj = json.load(open(TIMING, encoding='utf-8'))
    if a.arcs:
        res = run_arc_jobs(tj, a.door_data, None, a.jobs)
        tj['arcs'] = compute_arcs(tj, a.door_data, res)
        bad = validate_arcs(tj) + invariant_problems(res, 'inv_fresh')
        if bad:
            raise SystemExit('arcs rejected (door_timing.json untouched):\n  ' + '\n  '.join(bad))
        json.dump(tj, open(TIMING, 'w', encoding='utf-8'), indent=2)
        print('arcs written to %s' % TIMING)
    if a.check_arcs_data:
        res = run_arc_jobs(tj, a.door_data, tj.get('arcs') or {}, a.jobs)
        print('cap invariant of the STORED arcs (criterion: gen_door_timing.py arc_invariant; contact %.2f m, anchors outboard of the car side, r %.1f..length+%.1f m):' % (
            INV_CONTACT_M, INV_R_MIN, INV_BODY_M))
        bad = invariant_problems(res, 'inv_stored')
        fresh = compute_arcs(tj, a.door_data, res)
        same = json.dumps(fresh, sort_keys=True) == json.dumps(tj.get('arcs'), sort_keys=True)
        print('stored arcs invariant %s; stored arcs %s the extracted door-data' % ('FAIL' if bad else 'ok', 'match' if same else 'DIFFER from'))
        sys.exit(0 if same and not bad else 1)
    text = render(tj)
    if a.check:
        cur = open(REDS, encoding='utf-8').read() if os.path.exists(REDS) else ''
        ok = cur == text
        bad = validate_arcs(tj)
        for b in bad:
            print('ARC INVALID: ' + b)
        print('utpp_vehicle_door_timing.reds %s; arcs %s' % ('up to date' if ok else 'STALE', 'ok' if not bad else 'INVALID'))
        sys.exit(0 if ok and not bad else 1)
    with open(REDS, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print('wrote %s (%d seat keys)' % (REDS, len(table(tj))))


if __name__ == '__main__':
    main()
