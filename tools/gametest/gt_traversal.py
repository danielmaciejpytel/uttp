"""Traversal set of the night runner: vault and climb at the summoned car (set "traversal", not part of "all").

The summoned car of the vehicle set is the obstacle, so no special place is needed: V stands at the driver side of the car (the
approach of the vehicle macro, with another car-frame spot), pushes the stick into the car and taps jump. Vault: at the hood height
spot; climb: at the cabin. The result is judged from the log lines of the attempt ("UTPP trav:", "UTPP MMTRAV", the vehicle vault
diagnostic line) and from the motion feed column `detailed` (8 = climb, 9 = vault) when a feed file is being written. Each attempt
is recorded as a labelled phase window for the pose meter.

  python Source\\tools\\gametest\\night.py --dry-run --set traversal
"""
from __future__ import annotations

import glob
import math
import os
import re
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gt_input import file_size, read_since  # noqa: E402

ACTION_CLIMB, ACTION_VAULT = 8, 9
PARTS = ['trav_vault_jump', 'trav_climb_jump']
SET_NAME = 'traversal'


def env_spot(name: str, default: tuple | None) -> tuple | None:
    """A car-frame spot "lon,lat" from the environment (a vehicle of another shape, e.g. a motorcycle, needs another place to
    stand at); unset or not two numbers = the default."""
    try:
        lon, lat = (float(v) for v in os.environ.get(name, '').split(','))
    except ValueError:
        return default
    return (lon, lat)


def env_aim(name: str, default: tuple | None) -> tuple | None:
    """A car-frame aim point "lon,lat" from the environment; "off" = no aim (the approach's own facing of the vehicle centre)."""
    if os.environ.get(name, '').strip().lower() == 'off':
        return None
    return env_spot(name, default)


# car-frame spot (lon, lat) to stand at (lat > 0 = driver side, 0.9 .. 1.7 is where the approach stops), the car-frame point the
# camera is turned to before the jump (None = keep the approach's facing of the vehicle centre), the pad part and the label of the
# pose window. Vault: the motorcycle of the test setup is vaulted only when V's path crosses its axis near lon -0.25 .. 0.0 at a
# diagonal: from the spot (1.3, 1.2) facing the vehicle centre the jump crosses the axis near lon +0.35 and the game plays the
# vault (2026-11-18, first try; the same stand as the vault seen 2026-11-15); a crossing near lon 0 from (0.9, 1.2) was planned
# as a climb twice, further back (lon <= -0.55, the low tail) is a plain jump over it with no traversal at all. Facing the
# vehicle centre with the approach's 25 deg tolerance spread the crossing from +0.6 to -0.9, so the vault aims with a tight
# tolerance and is played only when V stands within STAND_TOL_M of the spot and
# the camera is within AIM_TOL_DEG of the aim point (a fresh position after the alignment; one re-approach and one re-alignment,
# else the attempt is not played and the reason is recorded). Climb: beside the cabin, the approach's facing (passes as it is).
# From the environment: UTPP_NIGHT_VAULT_SPOT, UTPP_NIGHT_CLIMB_SPOT, UTPP_NIGHT_VAULT_AIM ("lon,lat" or "off").
KINDS = {
    'vault': {'action': ACTION_VAULT, 'spot': env_spot('UTPP_NIGHT_VAULT_SPOT', (1.3, 1.2)),
              'aim': env_aim('UTPP_NIGHT_VAULT_AIM', (0.0, 0.0)), 'part': 'trav_vault_jump', 'label': 'trav_vault'},
    'climb': {'action': ACTION_CLIMB, 'spot': env_spot('UTPP_NIGHT_CLIMB_SPOT', (-0.2, 1.3)), 'aim': None,
              'part': 'trav_climb_jump', 'label': 'trav_climb'},
}
# the aim before the jump. From the default spot the crossing point is ~1.56 m away and the path crosses the axis ~40 deg off
# the perpendicular (50 deg to the axis), so
# 1 deg of aim error moves the crossing ~0.027 m off the path and ~0.035 m along the axis: 8 deg gave +-0.29 m along the axis,
# wider than the whole vault window (-0.25 .. 0.0, 0.25 m); 3 deg gives about +-0.11 m, inside it from its middle (-0.1).
AIM_TOL_DEG = 3.0
AIM_PULSES = 12           # camera pulse budget of one alignment (the fine pulses need a few more than the coarse ones)
# where V stands moves the crossing as much as the aim does: the vault is played only within this distance of the spot (car
# frame, from a fresh position after the alignment); the approach is asked to stop at STAND_RADIUS_M (night.py
# door_radius_override, the last metre walked), so a stop at its edge still passes
STAND_TOL_M = 0.3
STAND_RADIUS_M = 0.2


def car_frame_point(pos: dict | None, vehicle: dict | None = None) -> tuple | None:
    """V's (lon, lat) in the frame of the vehicle (`vehicle` = the POS that carries the vehicle, default `pos` itself). Pure."""
    vehicle = vehicle or pos
    if not (pos and pos.get('p') and vehicle and vehicle.get('v') and vehicle.get('vr')):
        return None
    import gt_night as gn  # late: gt_night imports this module
    return gn.car_frame(pos['p'][:2], vehicle['v'][:2], vehicle['vr'])


def crossing_lon(before: tuple | None, after: tuple | None) -> float | None:
    """The lon where V's straight path from `before` to `after` (car-frame (lon, lat), V on the lat > 0 side first) crosses the
    vehicle's long axis (lat 0); None when V did not cross it or a point is missing. Pure."""
    if not before or not after or before[1] <= 0.0 or after[1] > 0.0:
        return None
    t = before[1] / (before[1] - after[1])
    return before[0] + t * (after[0] - before[0])


def path_note(before: tuple | None, after: tuple | None) -> str:
    """Where V went in the attempt, for the night log and the failure text: the crossing of the vehicle axis or the end place."""
    if not before or not after:
        return ''
    x = crossing_lon(before, after)
    if x is not None:
        return f'V crossed the vehicle axis at lon {x:+.2f} (stood at lon {before[0]:+.2f} lat {before[1]:.2f})'
    return f'V did not cross the vehicle (lon {before[0]:+.2f} lat {before[1]:.2f} -> lon {after[0]:+.2f} lat {after[1]:.2f})'
ATTEMPTS = 2              # per kind
APPROACH_OK_M = 2.0       # the approach must end within this distance of the spot, else the attempt is not played (the vault
                          # then also needs STAND_TOL_M from a fresh position: prepare_vault)
SETTLE_S = 1.2            # log flush and landing after the jump
TIME_LIMIT_S = 420.0
VAULT_LOG_ON = ("local ok,e=pcall(function() return Game.UTPPSetVehicleVaultLog(true) end) "
                "Game.UTPPLogLine('[night] TRAVLOG ok='..tostring(ok))")
VAULT_LOG_OFF = VAULT_LOG_ON.replace('(true)', '(false)')

TRAV_RE = re.compile(r'UTPP trav: action=(\d+) h=[-\d.]+ .*?result=(\w+)\(([^)]*)\)')
RELEASE_RE = re.compile(r'UTPP trav: released to game animation \(detail=(-?\d+)\)')
MMTRAV_RE = re.compile(r'UTPP MMTRAV ')
# the game's own obstacle probe found no vault here: the vault line carries this text, the attempt says nothing about the mod
VAULT_NOT_VALID = 'vaultValid=false'
VEHICLE_VAULT_RE = re.compile(r'\[utpp_vehicle_vault\] (vanilla vault over a car|assisted vault over a car|no vault over a car)'
                              r'(?: \((.*?)\))?')


def evidence(text: str) -> dict:
    """The traversal lines of a plugin log text: {'mm': [{'action', 'result', 'reason'}], 'released': [detail], 'vehicle_vault':
    [{'kind': 'vanilla' | 'assisted' | 'none', 'why'}], 'mmtrav': n, 'vault_not_valid': number of vault lines that report the
    game's vault data as not valid}. Pure."""
    out: dict = {'mm': [], 'released': [], 'vehicle_vault': [], 'mmtrav': 0, 'vault_not_valid': 0}
    for line in text.splitlines():
        m = TRAV_RE.search(line)
        if m:
            out['mm'].append({'action': int(m.group(1)), 'result': m.group(2), 'reason': m.group(3)})
            continue
        m = RELEASE_RE.search(line)
        if m:
            out['released'].append(int(m.group(1)))
            continue
        m = VEHICLE_VAULT_RE.search(line)
        if m:
            out['vehicle_vault'].append({'kind': m.group(1).split()[0], 'why': m.group(2) or ''})
            if VAULT_NOT_VALID in line:
                out['vault_not_valid'] += 1
            continue
        if MMTRAV_RE.search(line):
            out['mmtrav'] += 1
    return out


def feed_detailed(path: str | None, seconds: float) -> set | None:
    """The values of the feed column `detailed` over the last `seconds` of feed time (the rows' dt summed from the end), or None
    when there is no feed file or it lacks the column. Reads the header line and the tail of the file only."""
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, 'rb') as f:
            head = f.readline().decode('utf-8', 'replace')
            size = os.path.getsize(path)
            f.seek(max(0, size - 400000))
            tail = f.read().decode('utf-8', 'replace').splitlines()
    except OSError:
        return None
    cols = head.lstrip('#').split()
    if cols and cols[0] == 'cols':
        cols = cols[1:]
    if 'detailed' not in cols or 'dt' not in cols:
        return None
    di, ti = cols.index('detailed'), cols.index('dt')
    seen, spent = set(), 0.0
    for row in reversed(tail):
        parts = row.split()
        if len(parts) < len(cols) or row.startswith('#'):
            continue
        try:
            seen.add(int(float(parts[di])))
            spent += float(parts[ti])
        except ValueError:
            continue
        if spent >= seconds:
            break
    return seen


def newest_feed(log_dir: str | None) -> str | None:
    files = sorted(glob.glob(os.path.join(log_dir or '', 'utpp-feed-*.txt')), key=os.path.getmtime)
    return files[-1] if files else None


def judge(kind: str, ev: dict, feed: set | None = None) -> dict:
    """-> {'ok': bool, 'by': [evidence names], 'why': text when not ok, 'invalid': True when a vault did not happen because the
    game reported its vault data as not valid at this place (not a failure of the mod)}. Vault / climb counts when the motion plan started the action
    (`UTPP trav: action=<8|9> ... result=Ok`), when the game's own animation was released for it (detail = the action), when the
    vehicle vault line says the vault happened (vault only) or when the feed column `detailed` shows the action. Pure."""
    action = KINDS[kind]['action']
    by = []
    if any(e['action'] == action and e['result'] == 'Ok' for e in ev['mm']):
        by.append('trav line Ok')
    if action in ev['released']:
        by.append('released to game animation')
    if kind == 'vault' and any(v['kind'] in ('vanilla', 'assisted') for v in ev['vehicle_vault']):
        by.append('vehicle vault line')
    if feed is not None and action in feed:
        by.append(f'feed detailed={action}')
    if by:
        return {'ok': True, 'by': by, 'why': '', 'invalid': False}
    bits = []
    rej = [f'{e["result"]}({e["reason"]})' for e in ev['mm'] if e['action'] == action]
    if rej:
        bits.append('the motion plan rejected it: ' + ', '.join(rej[:3]))
    elif ev['mm']:
        bits.append('only the other action was planned (trav action ' + ','.join(str(e['action']) for e in ev['mm']) + ')')
    no = [v['why'] for v in ev['vehicle_vault'] if v['kind'] == 'no']
    if no:
        bits.append('vehicle vault: ' + no[0])
    if not ev['mm'] and not ev['vehicle_vault'] and not ev['released']:
        bits.append('no traversal line at all (the jump met no obstacle: V not at the car?)'
                    + ('' if feed is None else f'; feed detailed values {sorted(feed)}'))
    invalid = kind == 'vault' and bool(ev.get('vault_not_valid'))
    if invalid:
        bits.append('the game reports the vault as not valid here (' + VAULT_NOT_VALID + ')')
    return {'ok': False, 'by': [], 'why': '; '.join(bits), 'invalid': invalid}


def _stamp(t: datetime) -> str:
    return t.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]


def stand_and_aim(pos: dict | None, vehicle: dict | None, spot: tuple, aim: tuple) -> dict:
    """Where V stands and how far the camera is off the aim point, from one position record: {'stand': (lon, lat) | None,
    'offset_m': distance of the stand from `spot` | None, 'bearing_deg': signed angle of the car-frame `aim` point from the camera
    forward | None}. `vehicle` = the record that carries the vehicle when `pos` lacks it. Pure."""
    veh = pos if pos and pos.get('v') and pos.get('vr') else vehicle
    stand = car_frame_point(pos, veh) if pos else None
    if not stand:
        return {'stand': None, 'offset_m': None, 'bearing_deg': None}
    import gt_night as gn  # late: gt_night imports this module
    target = gn.from_car_frame(aim[0], aim[1], veh['v'][:2], veh['vr'])
    bearing = gn.bearing_deg(pos['p'][:2], pos['cf'], target) if pos.get('cf') else None
    return {'stand': stand, 'offset_m': math.hypot(stand[0] - spot[0], stand[1] - spot[1]), 'bearing_deg': bearing}


def _prep_fields(prep: dict) -> dict:
    """The attempt record fields of a prepare_vault result."""
    rnd = lambda v, n: None if v is None else round(v, n)  # noqa: E731
    return {'stand': [round(c, 2) for c in prep['stand']] if prep.get('stand') else None,
            'stand_offset_m': rnd(prep.get('offset_m'), 2), 'aim_bearing_deg': rnd(prep.get('bearing_deg'), 1),
            'reapproached': prep.get('reapproached', False), 'realigned': prep.get('realigned', False)}


def prepare_vault(night, pad, guard, plan, pos: dict, ep: dict, spec: dict) -> dict:
    """Approach to the vault spot with the tight stop radius, the fine camera alignment to the aim point, then a fresh position:
    the jump may only follow when V stands within STAND_TOL_M of the spot (one re-approach) and the camera is within
    AIM_TOL_DEG (one re-alignment). -> {'ok', 'why' (when not ok), 'dist' (the approach's own end distance), 'pos' (the fresh
    position, with the vehicle), 'stand', 'offset_m', 'bearing_deg', 'reapproached', 'realigned'}."""
    import gt_night as gn  # late: gt_night imports this module
    out = {'ok': False, 'why': '', 'dist': None, 'pos': None, 'stand': None, 'offset_m': None, 'bearing_deg': None,
           'reapproached': False, 'realigned': False}
    for leg in (1, 2):
        night.door_spot_override, night.door_radius_override = spec['spot'], STAND_RADIUS_M
        try:
            out['dist'] = night.approach(pad, guard, plan, pos, ep)
        finally:
            night.door_spot_override = night.door_radius_override = None
        known = getattr(night, 'last_known', None) or pos
        if out['dist'] > APPROACH_OK_M:
            out['why'] = f'V could not stand at the car (approach ended {out["dist"]:.1f} m away)'
            return out
        if not (known and known.get('v') and known.get('vr')):
            out['why'] = 'no vehicle in the position after the approach'
            return out
        for align in (1, 2):
            target = gn.from_car_frame(spec['aim'][0], spec['aim'][1], known['v'][:2], known['vr'])
            known = night.camera_align(pad, guard, plan, known, target, ep, tol=AIM_TOL_DEG, why='trav-aim', fine=True,
                                       pulses=AIM_PULSES) or known
            fresh = night.poll_pos(guard, pad, newer_than=datetime.now())   # a POS line written after the last pulse
            chk = stand_and_aim(fresh, known, spec['spot'], spec['aim'])
            if fresh and not fresh.get('v') and known.get('v'):
                fresh = dict(fresh, v=known['v'], vr=known['vr'])
            out.update(chk, pos=fresh)
            if chk['stand'] is None or chk['offset_m'] > STAND_TOL_M or chk['bearing_deg'] is None:
                break                           # the stand decides first: a re-approach also re-aims
            if abs(chk['bearing_deg']) <= AIM_TOL_DEG:
                out['ok'] = True
                return out
            if align == 1:
                out['realigned'] = True
                night.log(f'[traversal] vault: camera {chk["bearing_deg"]:+.1f} deg off the aim after the alignment '
                          f'(tol {AIM_TOL_DEG:.0f}): aligning once more')
                known = fresh
        if chk['stand'] is not None and chk['offset_m'] <= STAND_TOL_M:
            out['why'] = (f'aim not reached: the camera stayed {chk["bearing_deg"]:+.1f} deg off the aim point after two '
                          f'alignments (tol {AIM_TOL_DEG:.0f} deg); no jump') if chk['bearing_deg'] is not None else \
                'no camera forward in the position after the alignment; no jump'
            return out
        where = ('no position after the alignment' if chk['stand'] is None else
                 f'V stood {chk["offset_m"]:.2f} m from the spot (lon {chk["stand"][0]:+.2f} lat {chk["stand"][1]:.2f}, '
                 f'tol {STAND_TOL_M:.1f} m)')
        if leg == 1:
            out['reapproached'] = True
            night.log(f'[traversal] vault: {where}: approaching once more')
            pos = fresh if fresh and fresh.get('v') else known
            continue
        out['why'] = f'{where} after a second approach; no jump'
    return out


def run_traversal_macro(night, pad, guard, plan, out: dict | None = None) -> dict:
    """Vault and climb at the car, ATTEMPTS tries each. `out` (the scenario record) is filled while it runs; returns `out` with
    'verdict' PASS | FAIL | INVALID (a kind that only failed because the game reports the vault as not valid; not scored, the text
    is in 'invalid_fails' and 'invalid_cause') | SKIP (no car), 'fails', 'phase_windows', 'attempts' (evidence per attempt)."""
    out = {} if out is None else out
    for key, empty in (('fails', []), ('invalid_fails', []), ('warnings', []), ('phase_windows', []), ('attempts', []),
                       ('results', {})):
        out.setdefault(key, empty)
    ep: dict = {'events': []}
    ledger = getattr(night, 'settings', None)
    entry, logged = None, False
    try:
        txt = night.console_verified(pad, guard, VAULT_LOG_ON, lambda t: '[night] TRAVLOG ok=' in t, 'traversal/log_on')
        logged = bool(txt and '[night] TRAVLOG ok=true' in txt)
        if ledger is not None and txt is not None:
            entry = ledger.register('vehicle.vaultLog', 'off', 'on', SET_NAME, volatile=True, restore=(VAULT_LOG_OFF, 'TRAVLOG'))
        if not logged:
            out['warnings'].append('the vehicle vault log could not be switched on: the vault is judged from the other lines')
        pos = night.poll_pos(guard, pad)
        if not (pos and pos.get('v') and pos.get('vr') and pos.get('src') != 'mounted'):
            pos = night.locate_vehicle(pad, guard, plan, ep)
            if pos:
                pos, _ = night.wait_car_stopped(pad, guard, ep, pos)
        if not (pos and pos.get('v') and pos.get('vr')):
            out.update(verdict='SKIP', skip_reason='no car located (nothing to vault or climb)')
            night.log('[traversal] no car located: skipped')
            return out
        for kind, spec in KINDS.items():
            done = False
            for attempt in range(1, ATTEMPTS + 1):
                night.check_continue()
                prep = None
                if spec.get('aim'):
                    # the vault: tight stand, fine aim, checked on a fresh position; no jump when either is not reached
                    prep = prepare_vault(night, pad, guard, plan, pos, ep, spec)
                    dist = prep['dist'] if prep['dist'] is not None else float('inf')
                    aim_note = (f'stand lon {prep["stand"][0]:+.2f} lat {prep["stand"][1]:.2f}, {prep["offset_m"]:.2f} m from the '
                                f'spot' if prep['stand'] else 'stand unknown') + \
                        (f', camera {prep["bearing_deg"]:+.1f} deg off the aim' if prep['bearing_deg'] is not None else '')
                    if not prep['ok']:
                        out['attempts'].append({'kind': kind, 'attempt': attempt, 'played': False,
                                                'approach_dist_m': round(dist, 2) if prep['dist'] is not None else None,
                                                'why': prep['why'], **_prep_fields(prep)})
                        night.log(f'[traversal] {kind} #{attempt}: attempt not played: {prep["why"]} [{aim_note}]')
                        pos = night.poll_pos(guard, pad) or pos
                        continue
                    night.log(f'[traversal] {kind} #{attempt}: ready to jump [{aim_note}]')
                    known = prep['pos']
                else:
                    night.door_spot_override = spec['spot']
                    try:
                        dist = night.approach(pad, guard, plan, pos, ep)
                    finally:
                        night.door_spot_override = None
                    if dist > APPROACH_OK_M:
                        out['attempts'].append({'kind': kind, 'attempt': attempt, 'played': False, 'approach_dist_m': round(dist, 2),
                                                'why': f'V could not stand at the car (approach ended {dist:.1f} m away)'})
                        night.log(f'[traversal] {kind} #{attempt}: V ended {dist:.1f} m from the spot: attempt not played')
                        pos = night.poll_pos(guard, pad) or pos
                        continue
                    known = getattr(night, 'last_known', None) or pos
                before = car_frame_point(known)
                off, t0 = file_size(night.utpp_log), datetime.now()
                night.play(pad, guard, spec['part'], plan)
                guard.wait(SETTLE_S)
                t1 = datetime.now()
                ev = evidence(read_since(night.utpp_log, off))
                feed = feed_detailed(newest_feed(getattr(getattr(night, 'args', None), 'log_dir', None)), (t1 - t0).total_seconds())
                res = judge(kind, ev, feed)
                new = night.poll_pos(guard, pad)
                after = car_frame_point(new, known if known and known.get('v') else new)
                note = path_note(before, after)
                why = res['why'] + (f'; {note}' if note and not res['ok'] else '')
                out['phase_windows'].append({'label': f'{spec["label"]}_{attempt}', 'start': _stamp(t0), 'end': _stamp(t1)})
                out['attempts'].append({'kind': kind, 'attempt': attempt, 'played': True, 'approach_dist_m': round(dist, 2),
                                        'ok': res['ok'], 'by': res['by'], 'why': why, 'invalid': res['invalid'], 'evidence': ev,
                                        'feed_detailed': sorted(feed) if feed is not None else None,
                                        'stand': [round(c, 2) for c in before] if before else None,
                                        'end': [round(c, 2) for c in after] if after else None,
                                        'crossing_lon': (round(crossing_lon(before, after), 2)
                                                         if crossing_lon(before, after) is not None else None),
                                        **(_prep_fields(prep) if prep else {})})
                night.log(f'[traversal] {kind} #{attempt}: ' + ('OK by ' + ', '.join(res['by']) if res['ok'] else 'NOT done: ' + why)
                          + (f' [{note}]' if note and res['ok'] else ''))
                pos = new or pos
                if res['ok']:
                    done = True
                    break
            out['results'][kind] = done
            if not done:
                last = [a for a in out['attempts'] if a['kind'] == kind]
                played = [a for a in last if a.get('played')]
                text = f'{kind}: ' + (played[-1]['why'] if played else last[-1].get('why') or
                                      f'V could not stand at the car (approach ended {last[-1]["approach_dist_m"]} m away)')
                if played and all(a.get('invalid') for a in played):
                    # every played attempt met the game's "no vault data" answer: not scored
                    out['results'][kind] = 'invalid'
                    out['invalid_fails'].append(text)
                    out['invalid_cause'] = f'{kind}: the game reports the vault as not valid ({VAULT_NOT_VALID})'
                    night.log(f'[traversal] {kind}: INVALID, not scored: {VAULT_NOT_VALID}')
                else:
                    out['fails'].append(text)
    finally:
        if logged or entry is not None:
            try:
                txt = night.console_verified(pad, guard, VAULT_LOG_OFF, lambda t: '[night] TRAVLOG ok=' in t, 'traversal/log_off')
                if entry is not None and txt and '[night] TRAVLOG ok=true' in txt:
                    ledger.mark_restored(entry)
            except Exception as e:  # noqa: BLE001 - the result must survive; the ledger restores it before the quit
                out['warnings'].append(f'the vehicle vault log was not switched off: {type(e).__name__}')
    if out.get('verdict') != 'SKIP':
        out['verdict'] = 'FAIL' if out['fails'] else 'INVALID' if out['invalid_fails'] else 'PASS'
    return out


def dry_run_lines(scenarios: dict | None = None) -> list[str]:
    lines = [f'== {SET_NAME}: vault and climb at the summoned car (not in "all"; the car of the vehicle set is the obstacle)',
             '   console: Game.UTPPSetVehicleVaultLog(true) before, (false) after (ledger entry vehicle.vaultLog)',
             '   car     : the position of the car in the POS lines, else summoned the way the vehicle macro does it']
    for kind, spec in KINDS.items():
        sc = scenarios.get(spec['part']) if scenarios else None
        dur = f', {sc.duration:.1f} s of pad input' if sc is not None else ''
        lines.append(f'   {kind:6}  : approach to car-frame lon {spec["spot"][0]:+.1f} lat {spec["spot"][1]:.1f} (side of the driver), '
                     + (f'stop radius {STAND_RADIUS_M:.2f} m (last metre walked), camera to car-frame lon {spec["aim"][0]:+.2f} '
                        f'lat {spec["aim"][1]:.2f} (fine pulses, tol {AIM_TOL_DEG:.0f} deg); jump only when a fresh position '
                        f'is within {STAND_TOL_M:.1f} m of the spot and {AIM_TOL_DEG:.0f} deg of the aim (one re-approach, one '
                        f're-alignment, else not played), '
                        if spec.get('aim') else '')
                     + f'pad part {spec["part"]}{dur}; up to {ATTEMPTS} attempts; ok when the log shows a `UTPP trav: action={spec["action"]} '
                     f'... result=Ok` line, a release to the game animation with detail={spec["action"]}, '
                     + ('the vehicle vault line, ' if kind == 'vault' else '')
                     + f'or the feed column detailed = {spec["action"]}; pose window {spec["label"]}_<attempt>')
    return lines


def main(argv=None) -> int:
    from gt_scenario import load_scenarios
    scenarios = load_scenarios(os.path.join(HERE, 'scenarios.json'))
    missing = [p for p in PARTS if p not in scenarios]
    print('\n'.join(dry_run_lines(scenarios)))
    if missing:
        print(f'MISSING pad parts in scenarios.json: {missing}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
