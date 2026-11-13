"""Door geometry and position feedback for car_cases.ps1 (offline helper, no game input, no game launch).

The car cases used to walk V open loop (stick x time) next to the car and then press X. 2026-11-01 runs: the walk of the
'behind' case ends ~2 m short of the door on every car, the Bratsk parallel case ended too far from the cab, and the
drive / brake / exit inputs then fired on foot (RT drew the pistol, LT aimed, the B hold dodged). This helper
  * keeps a per-car preset table (TweakDB record -> rig seat / door numbers and the verified spawn correction),
  * reads the newest "[night] POS" line (GetMod('utpp_core').NightAuto, started by the spawn command) and turns V's
    pose into the car frame (gt_night.car_frame: lon along the car forward, lat > 0 = car LEFT = driver side),
  * answers "at the door" or writes one short corrective walk step (camera-relative stick toward the door point),
  * splits every scenario of scenarios_car_cases.json at its 'enter' step (<name>__pre, <name>__x, <name>__tail) so
    car_cases.ps1 can check the mount before it sends drive / brake / exit.

CLI (all output is one JSON line):
  car_cases_nav.py preset <record>
  car_cases_nav.py split <scenarios.json> <out.json>
  car_cases_nav.py nav --log <utpp log> --marker "<text>" --vehicle <record> --side driver|passenger
                       [--mode check|retry|face] --out-scen <file> [--max-age 8] [--mag 0.55] [--fraction 0.85]
                       [--since <mark>]
  car_cases_nav.py newest --dir <log dir>             current utpp-<start>.log of the newest session
  car_cases_nav.py logmark --log <utpp log>           {'token': mark at the end of the session} (opaque string for --since)
  car_cases_nav.py found --log <utpp log> --since <mark> --rx <regex>      {'found': bool} any line after the mark matches
  car_cases_nav.py lines --log <utpp log> --since <mark> --marker "<text>" --rx <regex> --out <file>
                       the case's lines: from the LAST marker after the mark to the next '[car] SPAWN case=', filtered by the regex

The UTPP log rotates at 10 MB (utpp-<start>.log -> .1.log -> .2.log, a new .log starts). Every reader here goes through
gt_log, which reads the current file and its rotated siblings of the session as one stream, so a mark or a marker taken
before a rotation is still found afterwards. --since scopes a case to its own run: a marker of an earlier run (same case
name, other car) is never taken (no fallback to older markers).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_night as gn  # noqa: E402

# Hella reference numbers of the original case geometry (rig seat_front_left y, door x).
REF_SEAT_Y, REF_DOOR_X = 0.14, 0.99

# key = record without 'Vehicle.' and without the '_player' suffix (exact match only: kaukaz_bratsk_extended is another rig).
# rig = (seat_front_left y, door_front_left_a |x|, door_front_left_a y) from Extracted\game\vehicle-entry-car\vehrigs\json
# fwd / away = spawn correction in the car's frame that mounted in the
# 2026-11-01 runs (Hella: the case geometry itself; turbo 0.55 and bratsk -3.49 / 0.56 are the 20261101b values).
PRESETS: dict[str, dict] = {
    'v_standard2_archer_hella': dict(label='Archer Hella', rig=(0.14, 0.99, 1.17), fwd=0.0, away=0.0, settle=4.0),
    'v_standard25_villefort_columbus': dict(label='Villefort Columbus', rig=(0.22, 0.94, 1.36), fwd=0.0, away=0.0, settle=4.0),
    'v_standard3_chevalier_emperor': dict(label='Chevalier Emperor', rig=(-0.08, 0.99, 0.93), fwd=0.0, away=0.0, settle=4.0),
    'v_sport1_quadra_turbo': dict(label='Quadra Turbo-R', rig=(-0.41, 0.77, 0.46), fwd=0.55, away=0.0, settle=4.0),
    # the tall cab: spawn settle is a HYPOTHESIS (the first Bratsk spawn of both 2026-11-01 runs never mounted in the
    # driver-side parallel case, door and passenger cases did); the feedback walk decides, not this number
    'v_utility4_kaukaz_bratsk': dict(label='Kaukaz Bratsk', rig=(3.63, 1.55, 4.54), fwd=-3.49, away=0.56, settle=7.0, lat_max=2.65),
    # no sport2 rig is extracted (vehrigs\json has none) and no run used one: numbers unknown, generic geometry
    'v_sport2_porsche_911turbo': dict(label='Porsche 911 Turbo (rig UNKNOWN)', rig=None, fwd=0.0, away=0.0, settle=4.0),
}
ALIASES = {'v_sport1_quadra_turbo_r': 'v_sport1_quadra_turbo'}
GENERIC = dict(label='unknown record (generic geometry)', rig=None, fwd=0.0, away=0.0, settle=4.0)

# Working envelope measured on 2026-11-01 (V lon relative to the seat line seat_y - 0.14 +- 0.25 mounted on Hella /
# Columbus / Emperor / Turbo with the 0.55 correction; +0.41 -> nothing mounted on the uncorrected Turbo).
LON_TOL = 0.25
LAT_NEAR, LAT_FAR = -0.10, 1.70      # allowed V lat beyond the door x: [door_x - 0.10, door_x + 1.70] (Hella parallel start 1.61)
LAT_ABS_MAX = 2.7                    # the case start (R 2.6 with a 0 correction) mounted on Turbo (door 0.77) in 20261101b
LAT_TARGET = 0.45                    # where a corrective walk ends: door_x + 0.45 (night.py door lat 1.15 for a 0.99 door)
LAT_CLAMP = (0.45, 1.35)             # a corrective walk keeps V's own lat inside this band beyond the door x
SPEED_PER_STICK = 1.8                # m/s per unit of stick magnitude; 2026-11-01: |v| 0.82 m/s at stick 0.5 incl. start-up
START_PENALTY_S = 0.3
STEP_FRACTION = 0.85                # a corrective walk covers ~85 % of the distance (the stop glide does the rest)
CHUNK_MAX_S = 1.6
FAR_M = 12.0


def normalize(record: str) -> str:
    k = (record or '').strip()
    if k.lower().startswith('vehicle.'):
        k = k[8:]
    k = k.lower()
    if k.endswith('_player'):
        k = k[:-7]
    return ALIASES.get(k, k)


def preset(record: str) -> dict:
    key = normalize(record)
    p = dict(PRESETS.get(key, GENERIC))
    p['key'] = key
    p['known'] = key in PRESETS
    p['rig_known'] = p.get('rig') is not None
    seat_y, door_x, door_y = p['rig'] if p['rig'] else (REF_SEAT_Y, REF_DOOR_X, 1.17)
    p['seat_y'], p['door_x'], p['door_y'] = seat_y, door_x, door_y
    # V lon that gave the Hella case (seat 0.14, V at the origin): seat line - 0.14
    # upper lat bound of the no-correction window: the small cars mounted from lat 2.6 (the case start); explicit 'lat_max'
    # of a preset narrows it (Bratsk: V was ~2.5-3 m from the cab in the parallel case that never mounted, ~0.8 m where it did)
    p['lat_max'] = p.get('lat_max') or round(max(door_x + LAT_FAR, LAT_ABS_MAX), 3)
    p['lon_target'] = round(seat_y - 0.14, 3)
    p['lat_target'] = round(door_x + LAT_TARGET, 3)
    # spawn correction FROM THE RIG (car_cases.ps1 docs): forward = 0.14 - seat y, away = door x - 0.99
    p['fwd_formula'] = round(REF_SEAT_Y - seat_y, 3)
    p['away_formula'] = round(door_x - REF_DOOR_X, 3)
    return p


def in_window(lon: float, lat_s: float, p: dict) -> bool:
    """lat_s = lat * side sign (> 0 on the door side)."""
    return (abs(lon - p['lon_target']) <= LON_TOL
            and p['door_x'] + LAT_NEAR <= lat_s <= p['lat_max'])


def _pos_from_log(text: str, marker: str, max_age: float, now: datetime | None = None) -> tuple[dict | None, str]:
    i = text.rfind(marker) if marker else 0
    if marker and i < 0:
        return None, 'marker not in the log'
    seg = text[max(i, 0):]
    last = None
    for line in seg.splitlines():
        if '[night] POS p=' in line:
            last = line
    if last is None:
        return None, 'no [night] POS line after the marker (NightAuto not running?)'
    pos = gn.parse_pos(last)
    m = gn.LOGTS_RE.match(last)
    if pos is None or not m:
        return None, 'POS line not parsable'
    ts = datetime.strptime(m.group(1)[:23], '%Y-%m-%d %H:%M:%S.%f')
    age = ((now or datetime.now()) - ts).total_seconds()
    pos['age'] = age
    if age > max_age:
        return None, f'newest POS line is {age:.1f} s old'
    return pos, ''


SPAWN_NEXT = '[car] SPAWN case='


def case_lines(text: str, marker: str, rx) -> list | None:
    """Lines of one case: from the last `marker` in `text` to the next '[car] SPAWN case=' line, matching `rx`. None when
    the marker is not in the text (no fallback: the caller scoped the text to the case's own run)."""
    i = text.rfind(marker)
    if i < 0:
        return None
    seg = text[i:]
    j = seg.find(SPAWN_NEXT, 20)
    if j > 0:
        seg = seg[:j]
    return [l for l in seg.splitlines() if rx.search(l)]


def read_log_text(log: str, since: str, marker: str) -> str:
    """Log text for the nav helper: everything after the mark `since`; without a mark the last 12 MB of the session and,
    when the marker is not in it, the whole session."""
    if since:
        return gt_log.read_session_text(log, since)
    text = gt_log.read_session_text(log, tail_bytes=12_000_000)
    if marker and marker not in text:
        text = gt_log.read_session_text(log)
    return text


def decide(pos: dict, p: dict, side: str, mode: str = 'check', mag: float = 0.55, fraction: float = STEP_FRACTION,
           lon_bias: float = 0.0) -> dict:
    """Pure decision from one POS dict: {'state', 'lon', 'lat', ...} plus 'step' (scenario step) when a walk is needed.

    lon_bias shifts the wanted V lon: the 'door' case (car turned 90 deg, V faces the door) mounted on all five cars of
    2026-11-01 with V 0.5 m ahead of the parallel pose (lon = seat y + 0.36), so its window is centred 0.5 m further on.
    """
    p = dict(p, lon_target=round(p['lon_target'] + lon_bias, 3))
    if pos.get('mounted') or pos.get('src') == 'mounted':
        return {'state': 'mounted'}
    if not pos.get('v') or not pos.get('vr'):
        return {'state': 'nocar', 'why': 'POS has no vehicle (tag not spawned yet?)'}
    sgn = 1.0 if side == 'driver' else -1.0
    lon, lat = gn.car_frame(pos['p'][:2], pos['v'][:2], pos['vr'])
    lat_s = lat * sgn
    out = {'lon': round(lon, 2), 'lat': round(lat, 2), 'lat_door_side': round(lat_s, 2),
           'lon_target': p['lon_target'], 'lat_target': p['lat_target'], 'age': round(pos.get('age', 0.0), 2)}
    if math.hypot(lon, lat) > FAR_M:
        return dict(out, state='far', why=f'V {math.hypot(lon, lat):.1f} m from the car')
    v_xy, vr = pos['v'][:2], pos['vr']
    me = pos['p'][:2]
    if mode == 'face':
        tgt = gn.from_car_frame(lon, 0.0, v_xy, vr)           # the car's centre line at V's lon
        m, dur = 0.45, 0.4
    else:
        if mode == 'check':
            if in_window(lon, lat_s, p):
                return dict(out, state='ok')
            # the walk target stays inside the working window (Bratsk: door_x + 1.35 = 2.90 > lat_max 2.65 never mounted)
            lat_t = min(max(lat_s, p['door_x'] + LAT_CLAMP[0]), p['door_x'] + LAT_CLAMP[1], p['lat_max'])
            lon_t = p['lon_target']
        else:  # retry: the exact door point; already there -> nudge 0.35 m forward along the car
            lat_t, lon_t = p['door_x'] + LAT_TARGET, p['lon_target']
            if math.hypot(lon - lon_t, lat_s - lat_t) < 0.35:
                lon_t = lon + 0.35
        tgt = gn.from_car_frame(lon_t, sgn * lat_t, v_xy, vr)
        m = mag
        dist = math.hypot(tgt[0] - me[0], tgt[1] - me[1])
        dur = min(CHUNK_MAX_S, max(0.25, fraction * dist / (SPEED_PER_STICK * m) + START_PENALTY_S))   # undershoot, not ping-pong
        if mode == 'check' and dist < 0.12 and in_window(lon, lat_s, p):
            return dict(out, state='ok')
    sx, sy, dist = gn.approach_stick(me, pos['cf'], tgt, m)
    dur = round(dur, 2)
    step = {'name': 'nav', 'duration': round(dur + 0.7, 2), 'interp': 'linear', 'metrics': False,
            'ls': [[0, 0.0, 0.0], [0.05, sx, sy], [dur, sx, sy], [round(dur + 0.05, 2), 0.0, 0.0]],
            'rs': [[0, 0.0, 0.0]], 'lt': [[0, 0.0]], 'rt': [[0, 0.0]]}
    return dict(out, state='move' if mode != 'face' else 'face', stick=[sx, sy], dur=dur, dist=round(dist, 2), step=step)


def split_scenarios(raw: dict) -> dict:
    """Copy of the scenario file + <name>__pre / __x / __tail for each scenario that has an 'enter' step."""
    out = json.loads(json.dumps(raw))
    scn = out['scenarios']
    for name, sc in list(raw['scenarios'].items()):
        steps = sc.get('steps', [])
        idx = next((i for i, s in enumerate(steps) if s.get('name') == 'enter'), None)
        if idx is None:
            continue
        for suffix, part, what in (('__pre', steps[:idx], 'steps before the enter step'),
                                   ('__x', steps[idx:idx + 1], 'the enter step (X tap + seated time)'),
                                   ('__tail', steps[idx + 1:], 'steps after the enter step')):
            if part:
                scn[name + suffix] = {'description': f"{name}: {what} (split by car_cases_nav.py)",
                                      'steps': json.loads(json.dumps(part))}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('preset')
    s.add_argument('record')
    s = sub.add_parser('split')
    s.add_argument('src')
    s.add_argument('dst')
    s = sub.add_parser('newest')
    s.add_argument('--dir', required=True)
    s = sub.add_parser('logmark')
    s.add_argument('--log', required=True)
    s = sub.add_parser('found')
    s.add_argument('--log', required=True)
    s.add_argument('--since', default='')
    s.add_argument('--rx', required=True)
    s = sub.add_parser('lines')
    s.add_argument('--log', required=True)
    s.add_argument('--since', default='')
    s.add_argument('--marker', required=True)
    s.add_argument('--rx', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('nav')
    s.add_argument('--since', default='', help='mark from logmark: only the log after it is searched (no older markers)')
    s.add_argument('--log', required=True)
    s.add_argument('--marker', default='')
    s.add_argument('--vehicle', default='')
    s.add_argument('--side', choices=('driver', 'passenger'), default='driver')
    s.add_argument('--mode', choices=('check', 'retry', 'face'), default='check')
    s.add_argument('--out-scen', required=True)
    s.add_argument('--max-age', type=float, default=8.0)
    s.add_argument('--mag', type=float, default=0.55)
    s.add_argument('--lon-bias', type=float, default=0.0, help='shift of the wanted lon (the door case: 0.5)')
    s.add_argument('--fraction', type=float, default=STEP_FRACTION, help='share of the distance one walk covers (the loop lowers it to damp ping-pong)')
    a = ap.parse_args(argv)
    if a.cmd == 'preset':
        print(json.dumps(preset(a.record)))
        return 0
    if a.cmd == 'split':
        with open(a.src, encoding='utf-8') as f:
            raw = json.load(f)
        with open(a.dst, 'w', encoding='utf-8') as f:
            json.dump(split_scenarios(raw), f, indent=1)
        print(json.dumps({'ok': True, 'dst': a.dst}))
        return 0
    if a.cmd == 'newest':
        print(json.dumps({'path': gt_log.newest_log(a.dir)}))
        return 0
    if a.cmd == 'logmark':
        m = gt_log.log_mark(a.log)
        print(json.dumps({'token': m.token() if m is not None else ''}))
        return 0
    if a.cmd == 'found':
        text = gt_log.read_session_text(a.log, a.since or None)
        print(json.dumps({'found': re.search(a.rx, text, re.IGNORECASE) is not None}))
        return 0
    if a.cmd == 'lines':
        got = case_lines(gt_log.read_session_text(a.log, a.since or None), a.marker, re.compile(a.rx, re.IGNORECASE))
        if got is None:
            print(json.dumps({'ok': False, 'why': 'marker not in the log after the mark'}))
            return 0
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write('\n'.join(got) + ('\n' if got else ''))
        print(json.dumps({'ok': True, 'lines': len(got), 'out': a.out}))
        return 0
    text = read_log_text(a.log, a.since, a.marker)
    pos, why = _pos_from_log(text, a.marker, a.max_age)
    if pos is None:
        print(json.dumps({'state': 'nopos', 'why': why}))
        return 0
    res = decide(pos, preset(a.vehicle), a.side, a.mode, a.mag, a.fraction, a.lon_bias)
    step = res.pop('step', None)
    if step is not None:
        with open(a.out_scen, 'w', encoding='utf-8') as f:
            json.dump({'defaults': {'rate_hz': 60, 'interp': 'smooth'},
                       'scenarios': {'car_nav': {'description': 'one corrective walk step toward the door (car_cases_nav.py)',
                                                 'steps': [step]}}}, f, indent=1)
        res['scenario'] = a.out_scen
    print(json.dumps(res))
    return 0


if __name__ == '__main__':
    sys.exit(main())
