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
# car-frame spot (lon, lat) to stand at (lat > 0 = driver side, 0.9 .. 1.7 is where the approach stops), the pad part and the label of
# the pose window. Vault: beside the hood (low edge); climb: beside the cabin (higher than a vault allows).
KINDS = {
    'vault': {'action': ACTION_VAULT, 'spot': (1.4, 1.3), 'part': 'trav_vault_jump', 'label': 'trav_vault'},
    'climb': {'action': ACTION_CLIMB, 'spot': (-0.2, 1.3), 'part': 'trav_climb_jump', 'label': 'trav_climb'},
}
ATTEMPTS = 2              # per kind
APPROACH_OK_M = 2.0       # the approach must end within this distance of the spot, else the attempt is not played
SETTLE_S = 1.2            # log flush and landing after the jump
TIME_LIMIT_S = 420.0
VAULT_LOG_ON = ("local ok,e=pcall(function() return Game.UTPPSetVehicleVaultLog(true) end) "
                "Game.UTPPLogLine('[night] TRAVLOG ok='..tostring(ok))")
VAULT_LOG_OFF = VAULT_LOG_ON.replace('(true)', '(false)')

TRAV_RE = re.compile(r'UTPP trav: action=(\d+) h=[-\d.]+ .*?result=(\w+)\(([^)]*)\)')
RELEASE_RE = re.compile(r'UTPP trav: released to game animation \(detail=(-?\d+)\)')
MMTRAV_RE = re.compile(r'UTPP MMTRAV ')
VEHICLE_VAULT_RE = re.compile(r'\[utpp_vehicle_vault\] (vanilla vault over a car|assisted vault over a car|no vault over a car)'
                              r'(?: \((.*?)\))?')


def evidence(text: str) -> dict:
    """The traversal lines of a plugin log text: {'mm': [{'action', 'result', 'reason'}], 'released': [detail], 'vehicle_vault':
    [{'kind': 'vanilla' | 'assisted' | 'none', 'why'}], 'mmtrav': n}. Pure."""
    out: dict = {'mm': [], 'released': [], 'vehicle_vault': [], 'mmtrav': 0}
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
    """-> {'ok': bool, 'by': [evidence names], 'why': text when not ok}. Vault / climb counts when the motion plan started the action
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
        return {'ok': True, 'by': by, 'why': ''}
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
    return {'ok': False, 'by': [], 'why': '; '.join(bits)}


def _stamp(t: datetime) -> str:
    return t.strftime('%Y-%m-%d %H:%M:%S.%f')[:23]


def run_traversal_macro(night, pad, guard, plan, out: dict | None = None) -> dict:
    """Vault and climb at the car, ATTEMPTS tries each. `out` (the scenario record) is filled while it runs; returns `out` with
    'verdict' PASS | FAIL | SKIP (no car), 'fails', 'phase_windows', 'attempts' (evidence per attempt)."""
    out = {} if out is None else out
    for key, empty in (('fails', []), ('warnings', []), ('phase_windows', []), ('attempts', []), ('results', {})):
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
                night.door_spot_override = spec['spot']
                try:
                    dist = night.approach(pad, guard, plan, pos, ep)
                finally:
                    night.door_spot_override = None
                if dist > APPROACH_OK_M:
                    out['attempts'].append({'kind': kind, 'attempt': attempt, 'played': False, 'approach_dist_m': round(dist, 2)})
                    night.log(f'[traversal] {kind} #{attempt}: V ended {dist:.1f} m from the spot: attempt not played')
                    pos = night.poll_pos(guard, pad) or pos
                    continue
                off, t0 = file_size(night.utpp_log), datetime.now()
                night.play(pad, guard, spec['part'], plan)
                guard.wait(SETTLE_S)
                t1 = datetime.now()
                ev = evidence(read_since(night.utpp_log, off))
                feed = feed_detailed(newest_feed(getattr(getattr(night, 'args', None), 'log_dir', None)), (t1 - t0).total_seconds())
                res = judge(kind, ev, feed)
                out['phase_windows'].append({'label': f'{spec["label"]}_{attempt}', 'start': _stamp(t0), 'end': _stamp(t1)})
                out['attempts'].append({'kind': kind, 'attempt': attempt, 'played': True, 'approach_dist_m': round(dist, 2),
                                        'ok': res['ok'], 'by': res['by'], 'why': res['why'], 'evidence': ev,
                                        'feed_detailed': sorted(feed) if feed is not None else None})
                night.log(f'[traversal] {kind} #{attempt}: ' + ('OK by ' + ', '.join(res['by']) if res['ok'] else 'NOT done: ' + res['why']))
                pos = night.poll_pos(guard, pad) or pos
                if res['ok']:
                    done = True
                    break
            out['results'][kind] = done
            if not done:
                last = [a for a in out['attempts'] if a['kind'] == kind]
                played = [a for a in last if a.get('played')]
                out['fails'].append(f'{kind}: ' + (played[-1]['why'] if played else
                                                   f'V could not stand at the car (approach ended {last[-1]["approach_dist_m"]} m away)'))
    finally:
        if logged or entry is not None:
            try:
                txt = night.console_verified(pad, guard, VAULT_LOG_OFF, lambda t: '[night] TRAVLOG ok=' in t, 'traversal/log_off')
                if entry is not None and txt and '[night] TRAVLOG ok=true' in txt:
                    ledger.mark_restored(entry)
            except Exception as e:  # noqa: BLE001 - the result must survive; the ledger restores it before the quit
                out['warnings'].append(f'the vehicle vault log was not switched off: {type(e).__name__}')
    if out.get('verdict') != 'SKIP':
        out['verdict'] = 'FAIL' if out['fails'] else 'PASS'
    return out


def dry_run_lines(scenarios: dict | None = None) -> list[str]:
    lines = [f'== {SET_NAME}: vault and climb at the summoned car (not in "all"; the car of the vehicle set is the obstacle)',
             '   console: Game.UTPPSetVehicleVaultLog(true) before, (false) after (ledger entry vehicle.vaultLog)',
             '   car     : the position of the car in the POS lines, else summoned the way the vehicle macro does it']
    for kind, spec in KINDS.items():
        sc = scenarios.get(spec['part']) if scenarios else None
        dur = f', {sc.duration:.1f} s of pad input' if sc is not None else ''
        lines.append(f'   {kind:6}  : approach to car-frame lon {spec["spot"][0]:+.1f} lat {spec["spot"][1]:.1f} (side of the driver), '
                     f'pad part {spec["part"]}{dur}; up to {ATTEMPTS} attempts; ok when the log shows a `UTPP trav: action={spec["action"]} '
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
