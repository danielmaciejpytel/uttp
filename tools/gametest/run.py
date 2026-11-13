"""UTPP in-game analog locomotion test harness (virtual Xbox 360 pad + UTPP trace analysis).

  python Source\\tools\\gametest\\run.py --list
  python Source\\tools\\gametest\\run.py --scenarios slow_walk_half_stick,flick_180_run --repeat 3 --cet-enable-mm --dry-run
  python Source\\tools\\gametest\\run.py --scenarios slow_walk_half_stick,flick_180_run --repeat 3 --cet-enable-mm
  python Source\\tools\\gametest\\run.py --analyze Research\\logs\\gametest\\<timestamp> [--log <utpp.log>]
  python Source\\tools\\gametest\\run.py --cet-reload            (only click CET 'Reload all mods')
  python Source\\tools\\gametest\\run.py --scenarios ... --keep-pad   (persistent padd daemon: no disconnect dialog)
  python Source\\tools\\gametest\\run.py --padd-status | --padd-stop | --dismiss-dialog

Pad lifetime: without --keep-pad the virtual pad is destroyed at the end and the game shows a modal "controller
disconnected" dialog; the harness then presses Enter (scan 0x1C, guarded) and verifies that MMTRACE lines resume.
CET: console toggled with the overlay key as a scan code (grave 0x29), click into the console input at
--cet-input-rel of the game client rect, KEYEVENTF_UNICODE typing, Enter; UTPP commands are verified by their
'<Name>(' echo in the UTPP log (one retry), scenarios are not started when a command is not confirmed.

Safety: input is sent only while the foreground window belongs to Cyberpunk2077.exe (checked before every pad report,
key and click, plus a process liveness check every second). On focus loss, game exit, ESC/Pause pressed by the user,
Ctrl+C or any error the pad is zeroed and the run aborts. --dry-run never sends input and never touches ViGEmBus.
Outputs go to Research\\logs\\gametest\\<timestamp>\\ (per-run trace extracts, summary.csv, summary.json, plan.json).

Scoring precondition (2026-10-23): MM v2 is the default driver, so a run is scored only when 'UTPP MOTION' lines exist in
the scored span (after the CET UTPPSetMMTrace(1) echo); otherwise every scored step is INVALID and the overall verdict
FAIL (summary.json 'mm_precondition'). --allow-base scores a legacy v1 log with the v1 metrics instead.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
from gt_input import DEFAULT_CONSOLE_INPUT_REL, PADD_PORT  # noqa: E402
from gt_scenario import describe, load_scenarios  # noqa: E402
from gt_win32 import ForegroundGuard, GuardError, TimerResolution  # noqa: E402

PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
DEFAULT_OUT_ROOT = os.path.join(PROJECT, 'Research', 'logs', 'gametest')
DEFAULT_SCENARIOS = os.path.join(HERE, 'scenarios.json')
DEFAULT_THRESHOLDS = os.path.join(HERE, 'thresholds.json')
TS_FMT = '%Y-%m-%d %H:%M:%S.%f'
# the last two switch on the one-line-per-episode reload log and the vehicle vault log (off by default in release builds)
CET_ENABLE_MM = ['Game.UTPPSetMMEnabled(true)', 'Game.UTPPSetMMTrace(true)', 'Game.UTPPMotionSetReloadDiag(true)',
                 'Game.UTPPSetVehicleVaultLog(true)']


# ---------------------------------------------------------------------------------------------- execution

def run_scenario(sc, pad, guard, clock=time.perf_counter, now=datetime.now) -> dict:
    """Play one scenario. Returns {'steps': [{name, start, end, metrics_enabled}], 'aborted': str|None}."""
    dt = 1.0 / sc.rate_hz
    steps = [{'name': s.name, 'start': None, 'end': None, 'metrics': s.metrics} for s in sc.steps]
    aborted = None
    t0 = clock()
    cur = -1
    try:
        for i, fr in sc.sample(dt):
            target = t0 + fr.t
            while True:
                left = target - clock()
                if left <= 0:
                    break
                time.sleep(min(left, 0.004))
            if i != cur:
                stamp = now()
                if cur >= 0:
                    steps[cur]['end'] = stamp
                steps[i]['start'] = stamp
                cur = i
            pad.apply(fr)
    except GuardError as e:
        aborted = str(e)
    except KeyboardInterrupt:
        aborted = 'Ctrl+C'
    finally:
        pad.release()
        if cur >= 0:
            steps[cur]['end'] = now()
    return {'steps': steps, 'aborted': aborted}


def serialise_plan(plan: dict) -> dict:
    def conv(v):
        if isinstance(v, datetime):
            return v.strftime(TS_FMT)
        if isinstance(v, dict):
            return {k: conv(x) for k, x in v.items()}
        if isinstance(v, list):
            return [conv(x) for x in v]
        return v
    return conv(plan)


def load_plan(path: str) -> dict:
    with open(path, encoding='utf-8') as f:
        plan = json.load(f)
    for run in plan.get('runs', []):
        for st in run['steps']:
            for k in ('start', 'end'):
                if st.get(k):
                    st[k] = datetime.strptime(st[k], TS_FMT)
    return plan


# ---------------------------------------------------------------------------------------------- analysis

METRIC_COLUMNS = ['duration_s', 'mm_driver', 'samples', 'trace_coverage', 'motion_samples', 'motion_changes',
                  'motion_transitions_per_s', 'motion_cost_mean', 'motion_cost_max', 'motion_crouch_fraction', 'base_trace_frozen',
                  'trans_lines', 'transitions_per_s', 'cost_mean',
                  'cost_max', 'offset_saturated_s', 'offset_max_cm', 'footlock_corr_max_cm', 'rate_min',
                  'rate_mean', 'rate_max', 'speed_mean', 'speed_max', 'bodyturn_active_s', 'bodyturn_cmds',
                  'bodyturn_err_max_deg', 'cam_yaw_net_deg', 'cam_yaw_abs_deg', 'cam_pl_gap_max_deg',
                  'loco_fraction', 'crouch_fraction', 'stand_fraction', 'gate_lines', 'stick_min', 'stick_max',
                  'root_off_median_cm', 'root_off_max_cm', 'root_off_moving_median_cm',
                  'motion_db_walk_loops_fraction', 'motion_db_walk_starts_fraction', 'motion_clip_transition_fraction',
                  'motion_walk_loop_clip_fraction', 'motion_walk_nonloop_fraction', 'motion_walk_loop_share', 'motion_steady_s',
                  'motion_steady_switches', 'motion_steady_ct_wraps', 'motion_steady_ct_jumps', 'motion_steady_reselect_per_s',
                  'motion_steady_reselect_nowrap_per_s']
INVALID_VERDICT = 'INVALID'   # step not scored: the MM v2 precondition failed (see mm_precondition)


BLOCKED_SPEED_MAX = 0.3     # m/s: on every tick with the stick pushed (gt_log.PUSHED_STICK) the capsule stayed below
BLOCKED_MIN_TICKS = 2       # pushed ticks needed to judge (ticks are ~1 Hz)


def step_blocked(m: dict) -> bool:
    """True when V pushed the left stick on foot but the capsule did not move on any of those ticks (Female V
    2026-11-11 17:37: walked into a bush, |v| 0.07-0.18 m/s with the stick at 0.8; the slowest free walk, aim_walk,
    0.52-0.83 m/s). Ticks after a release (stop steps) do not count."""
    vmax, n, loco = m.get('pushed_speed_max'), m.get('pushed_ticks') or 0, m.get('loco_fraction')
    return (vmax is not None and n >= BLOCKED_MIN_TICKS and vmax < BLOCKED_SPEED_MAX
            and loco is not None and loco >= 0.5)


def scored_span(plan: dict):
    """(first step start, last step end) over all runs, or None."""
    st = [s for r in plan.get('runs', []) for s in r['steps'] if s.get('start') and s.get('end')]
    return (min(s['start'] for s in st), max(s['end'] for s in st)) if st else None


def apply_precondition(plan: dict, rows: list, lines: list, allow_base: bool = False, clock_offset_s: float = 0.0) -> dict:
    """MM v2 precondition before scoring: without MOTION lines in the scored span every scored step becomes INVALID
    (overall FAIL) instead of being judged on the frozen v1 fields. allow_base keeps the v1 verdicts (legacy logs)."""
    span = scored_span(plan)
    if span is None:
        pre = {'ok': None, 'motion_lines': 0, 'span': None, 'trace_echo': None, 'reason': 'no step windows'}
    else:
        off = timedelta(seconds=clock_offset_s)
        pre = gt_log.motion_precondition(lines, span[0] + off, span[1] + off)
    pre['allow_base'] = allow_base
    if pre['ok'] is False and not allow_base:
        for r in rows:
            if r['verdict'] != 'SKIP':
                r['verdict'] = INVALID_VERDICT
                r['fails'] = [f'precondition: {pre["reason"]}'] + r['fails']
    plan['mm_precondition'] = {k: (v.strftime(TS_FMT) if isinstance(v, datetime) else
                                   [x.strftime(TS_FMT) for x in v] if isinstance(v, (list, tuple)) and v and
                                   isinstance(v[0], datetime) else v) for k, v in pre.items()}
    return plan['mm_precondition']


def analyze(plan: dict, lines: list, base_thresholds: dict, scenarios: dict, clock_offset_s: float = 0.0) -> list:
    """Per-step metrics + verdicts. Returns list of rows (dicts).

    Steps whose left-stick magnitude goes below gt_log.SLOW_STICK (partial stick, ramps, stops) are 'slow' and use
    the '<metric>@slow' thresholds (e.g. rate_min@slow: the intended slow-walk rate floor 0.2-0.25).
    """
    off = timedelta(seconds=clock_offset_s)
    rows = []
    for run in plan['runs']:
        sc = scenarios.get(run['scenario'])
        thr = gt_log.merge_thresholds(base_thresholds, sc.thresholds if sc else {})
        stick = sc.step_stick_stats() if sc else {}
        started = [s for s in run['steps'] if s.get('start')]
        last_started = started[-1] if started else None
        for st in run['steps']:
            if not st.get('start') or not st.get('end'):
                continue
            t0, t1 = st['start'] + off, st['end'] + off
            m = gt_log.compute_metrics(lines, t0, t1)
            smin, smax = stick.get(st['name'], (None, None))
            m['stick_min'], m['stick_max'] = smin, smax
            slow = smin is not None and smin < gt_log.SLOW_STICK
            fails = gt_log.evaluate(m, thr, st['name'], slow=slow) if st.get('metrics', True) else []
            verdict = 'SKIP' if not st.get('metrics', True) else ('FAIL' if fails else 'PASS')
            if run.get('aborted') and st is last_started and verdict != 'SKIP':
                verdict = 'ABORTED'
            blocked = verdict in ('PASS', 'FAIL') and step_blocked(m)
            if blocked:
                # V pushed the stick on foot but did not move (an obstacle): the step measured nothing -> not scored
                m['unscored_fails'] = fails
                fails = [f'blocked: V did not move (|v| <= {m["pushed_speed_max"]} m/s < {BLOCKED_SPEED_MAX} on {m["pushed_ticks"]} ticks with the '
                         f'stick at {smax})']
                verdict = INVALID_VERDICT
            # True: no selection threshold (motion_transitions_per_s / costs) applied, so the verdict says nothing about jerks
            unscored = None if verdict == 'SKIP' else gt_log.selection_unscored(m, thr, st['name'], slow=slow)
            rows.append({'scenario': run['scenario'], 'run': run['run'], 'step': st['name'], 't0': t0, 't1': t1,
                         'verdict': verdict, 'fails': fails, 'selection_unscored': unscored, 'metrics': m})
    return rows


def write_outputs(out_dir: str, plan: dict, rows: list, lines: list, log_path: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    by_run: dict = {}
    for r in rows:
        by_run.setdefault((r['scenario'], r['run']), []).append(r)
    for (scn, run), rr in by_run.items():
        aborted = next((x.get('aborted') for x in plan['runs'] if x['scenario'] == scn and x['run'] == run), None)
        with open(os.path.join(out_dir, f'{scn}_r{run}.txt'), 'w', encoding='utf-8') as f:
            f.write(f'scenario {scn} run {run}  log {log_path}\n')
            if aborted:
                f.write(f'ABORTED: {aborted}\n')
            for r in rr:
                f.write(f'\n=== step {r["step"]}  {r["t0"].strftime(TS_FMT)[:23]} -> {r["t1"].strftime(TS_FMT)[11:23]}'
                        f'  {r["verdict"]}' + (f'  ({"; ".join(r["fails"])})' if r['fails'] else '') + '\n')
                m = r['metrics']
                f.write('    ' + ' '.join(f'{k}={m[k]}' for k in METRIC_COLUMNS if m.get(k) is not None) + '\n')
                f.write(f'    clips={",".join(m["clips"])}  classes={m["trans_classes"]}'
                        + (f'  dbs={",".join(m["motion_dbs"])}' if m.get('motion_dbs') else '') + '\n')
                if m.get('speed_by_mv'):
                    f.write('    speed_by_mv: ' + '  '.join(f'{k}: n={b["n"]} med={b["median"]} p90={b["p90"]}'
                                                            for k, b in m['speed_by_mv'].items()) + '\n')
                if m.get('not_applicable'):
                    f.write(f'    n/a under MM v2 (v1 matcher fields, not scored): {",".join(m["not_applicable"])}'
                            + ('  [base tick frozen]' if m.get('base_trace_frozen') else '') + '\n')
                if m.get('mm_driver_note'):
                    f.write(f'    driver: {m["mm_driver_note"]}\n')
                if r.get('selection_unscored'):
                    f.write('    selection_unscored: no selection threshold applied (verdict says nothing about jerks)\n')
                for l in gt_log.step_lines(lines, r['t0'], r['t1']):
                    f.write('  ' + l.compact() + '\n')
    with open(os.path.join(out_dir, 'summary.csv'), 'w', encoding='utf-8', newline='') as f:
        w = csv.writer(f)
        w.writerow(['scenario', 'run', 'step', 'verdict'] + METRIC_COLUMNS + ['clip_classes', 'fails'])
        for r in rows:
            m = r['metrics']
            w.writerow([r['scenario'], r['run'], r['step'], r['verdict']] + [m.get(k) for k in METRIC_COLUMNS] +
                       ['|'.join(m['clip_classes_played']), '; '.join(r['fails'])])
    total = {'PASS': 0, 'FAIL': 0, 'SKIP': 0, 'ABORTED': 0}
    for r in rows:
        total[r['verdict']] = total.get(r['verdict'], 0) + 1
    aborted_runs = [f'{x["scenario"]}#{x["run"]}: {x["aborted"]}' for x in plan['runs'] if x.get('aborted')]
    overall = 'FAIL' if total['FAIL'] or total.get(INVALID_VERDICT) or aborted_runs or not rows else 'PASS'
    summary = {'overall': overall, 'counts': total, 'aborted_runs': aborted_runs, 'log': log_path,
               'mm_precondition': plan.get('mm_precondition'),
               # True when at least one scored step had no selection threshold applied (see the per-step field)
               'selection_unscored': any(r.get('selection_unscored') for r in rows),
               'cet_commands': plan.get('cet_commands'), 'pad': plan.get('pad'),
               'dialog_dismiss': plan.get('dialog_dismiss'), 'guard_events': plan.get('guard_events'),
               'steps': [{**{k: v for k, v in r.items() if k not in ('t0', 't1')},
                          't0': r['t0'].strftime(TS_FMT), 't1': r['t1'].strftime(TS_FMT)} for r in rows]}
    with open(os.path.join(out_dir, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)
    return summary


def print_table(rows: list, summary: dict) -> None:
    # tr/s and cmax come from the MOTION lines under v2 and from MMTRACE TRANS under v1; 'n/a' = v1-only metric under v2
    cols = [('mm_driver', 'mm_driver'), ('motion_transitions_per_s', 'transitions_per_s'), ('motion_cost_max', 'cost_max'),
            ('offset_saturated_s', 'offset_saturated_s'), ('footlock_corr_max_cm', 'footlock_corr_max_cm'),
            ('rate_min', 'rate_min'), ('crouch_fraction', 'crouch_fraction'), ('bodyturn_cmds', 'bodyturn_cmds'),
            ('cam_yaw_net_deg', 'cam_yaw_net_deg'), ('samples', 'samples'), ('motion_samples', 'motion_samples')]
    short = ['drv', 'tr/s', 'cmax', 'sat_s', 'flcm', 'rmin', 'crch', 'btC', 'camYaw', 'n', 'nMotion']
    pre = summary.get('mm_precondition') or {}
    if pre.get('ok') is False:
        print(f'[gametest] PRECONDITION FAILED: {pre.get("reason")}' +
              (' (--allow-base: base verdicts kept)' if pre.get('allow_base') else ' -> scored steps INVALID'))
    print(f'{"scenario":26} {"r":>2} {"step":14} {"verdict":7} ' + ' '.join(f'{s:>6}' for s in short) + '  classes')
    for r in rows:
        m = r['metrics']
        v2 = m.get('mm_driver') == 'v2'
        na = set(m.get('not_applicable') or ())

        def cell(c2, c1):
            k = c2 if v2 else c1
            v = m.get(k)
            return 'n/a' if k in na else ('-' if v is None else v)
        vals = ' '.join(f'{cell(a, b)!s:>6}' for a, b in cols)
        print(f'{r["scenario"][:26]:26} {r["run"]:>2} {r["step"][:14]:14} {r["verdict"]:7} {vals}  '
              + ','.join(m['clip_classes_played']) + (f'  !! {"; ".join(r["fails"])}' if r['fails'] else ''))
    print(f'OVERALL {summary["overall"]} {summary["counts"]}' +
          (f'  aborted: {summary["aborted_runs"]}' if summary['aborted_runs'] else ''))


def load_thresholds(path: str) -> dict:
    base = dict(gt_log.DEFAULT_THRESHOLDS)
    if path and os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        base = gt_log.merge_thresholds(base, {k: v for k, v in data.items() if not k.startswith('_')})
    return base


# ---------------------------------------------------------------------------------------------- CLI

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--scenarios', default='', help='comma-separated scenario names (see --list)')
    p.add_argument('--repeat', type=int, default=1)
    p.add_argument('--list', action='store_true')
    p.add_argument('--dry-run', action='store_true', help='print the plan and read-only status; send nothing')
    p.add_argument('--scenario-file', default=DEFAULT_SCENARIOS)
    p.add_argument('--thresholds', default=DEFAULT_THRESHOLDS)
    p.add_argument('--log', default='', help='UTPP log (default: newest utpp-*.log in --log-dir)')
    p.add_argument('--log-dir', default=gt_log.DEFAULT_LOG_DIR)
    p.add_argument('--out', default='', help='output dir (default Research\\logs\\gametest\\<timestamp>)')
    p.add_argument('--analyze', default='', help='re-analyse an existing output dir (plan.json) without input')
    p.add_argument('--clock-offset', type=float, default=0.0, help='log time minus wall time [s]')
    p.add_argument('--allow-base', action='store_true',
                   help='score a log without "UTPP MOTION" lines with the base metrics (legacy logs); default: such '
                        'steps are INVALID because MM motion is the default driver')
    p.add_argument('--cet-enable-mm', action='store_true', help='type ' + ' ; '.join(CET_ENABLE_MM))
    p.add_argument('--cet', default='', help='extra CET commands, ";;"-separated')
    p.add_argument('--cet-scenario-cmds', action='store_true', help='also type each scenario\'s "cet" list')
    p.add_argument('--cet-reload', action='store_true', help='click CET "Reload all mods" before the run')
    p.add_argument('--cet-ui-scale', type=float, default=1.0)
    p.add_argument('--cet-no-click', action='store_true', help='do not click into the CET console input')
    p.add_argument('--cet-reload-offset', type=float, default=None, help='px from toolbar right edge')
    p.add_argument('--cet-reload-rel', default='', help='reload button as "fx,fy" of the game client rect')
    p.add_argument('--cet-input-rel', default='%.3f,%.3f' % DEFAULT_CONSOLE_INPUT_REL,
                   help='console input click as "fx,fy" of the game client rect (default %(default)s)')
    p.add_argument('--cet-input-from-layout', action='store_true',
                   help='take the console input point from CET layout.ini instead of --cet-input-rel')
    p.add_argument('--cet-key-scan', default='', help='overlay toggle scan code(s), e.g. 0x29 (default from '
                   'bindings.json via MapVirtualKey)')
    p.add_argument('--cet-no-verify', action='store_true', help='do not verify UTPP commands in the UTPP log')
    p.add_argument('--cet-no-auto-verify', action='store_true',
                   help='do not append a [cetv:<id>] marker to commands that are not verifiable otherwise (default: appended, '
                        'so every typed command is confirmed in the UTPP log, retyped once, and aborts the run when lost)')
    p.add_argument('--wake', default='rs', help='pad wake input: rs | ls | none | button:<NAME>')
    p.add_argument('--pad-settle', type=float, default=2.0, help='seconds after plugging the pad')
    p.add_argument('--gap', type=float, default=2.0, help='neutral seconds between runs')
    p.add_argument('--flush-wait', type=float, default=2.5, help='seconds to wait for the log after the run')
    p.add_argument('--focus-grace', type=float, default=0.0,
                   help='tolerate a foreground loss up to N s (pad neutral meanwhile) instead of aborting at once')
    p.add_argument('--keep-pad', action='store_true',
                   help='use the persistent padd daemon (started if needed): no "controller disconnected" dialog')
    p.add_argument('--padd-port', type=int, default=PADD_PORT)
    p.add_argument('--padd-status', action='store_true', help='print padd daemon status')
    p.add_argument('--padd-stop', action='store_true',
                   help='stop the padd daemon (unplugs the pad) and dismiss the dialog')
    p.add_argument('--no-dismiss', action='store_true',
                   help='do not press Enter to close the "controller disconnected" dialog after unplugging')
    p.add_argument('--dismiss-dialog', action='store_true',
                   help='only: focus the game, press Enter (scan 0x1C) and verify trace resumes')
    return p


def parse_pair(s: str) -> tuple[float, float] | None:
    if not s:
        return None
    a, b = (float(v) for v in s.split(','))
    if not (0.0 <= a < 1.0 and 0.0 <= b < 1.0):
        raise SystemExit(f'relative point must be in [0,1): {s}')
    return a, b


def make_overlay(args, guard):
    from gt_input import CetOverlay
    scans = [int(v, 0) for v in args.cet_key_scan.split(',')] if args.cet_key_scan else None
    return CetOverlay(guard, click_console=not args.cet_no_click, ui_scale=args.cet_ui_scale,
                      reload_offset_px=args.cet_reload_offset, toggle_scans=scans,
                      input_rel=None if args.cet_input_from_layout else parse_pair(args.cet_input_rel),
                      reload_rel=parse_pair(args.cet_reload_rel), auto_marker=not args.cet_no_auto_verify)


def select(scenarios: dict, names: str) -> list:
    out = []
    for n in [x.strip() for x in names.split(',') if x.strip()]:
        if n not in scenarios:
            raise SystemExit(f'unknown scenario {n!r}; use --list')
        out.append(scenarios[n])
    return out


def cet_commands(args, chosen) -> list:
    cmds = list(CET_ENABLE_MM) if args.cet_enable_mm else []
    cmds += [c.strip() for c in args.cet.split(';;') if c.strip()]
    if args.cet_scenario_cmds:
        for sc in chosen:
            cmds += [c for c in sc.cet if c not in cmds]
    return cmds


def resolve_log(args) -> str | None:
    return args.log or gt_log.newest_log(args.log_dir)


def dry_run(args, chosen) -> int:
    from gt_input import padd_ping, read_cet_overlay_key
    print('DRY RUN - no input is sent, ViGEmBus is not touched\n')
    for sc in chosen:
        print(describe(sc))
        print()
    print(f'repeat={args.repeat} wake={args.wake} pad_settle={args.pad_settle}s gap={args.gap}s '
          f'keep_pad={args.keep_pad} focus_grace={args.focus_grace}s')
    total = sum(sc.duration for sc in chosen) * args.repeat + args.gap * max(0, len(chosen) * args.repeat - 1)
    print(f'estimated input time {total:.1f} s')
    cmds = cet_commands(args, chosen)
    print('CET commands: ' + (' ; '.join(cmds) if cmds else '(none)'))
    guard = ForegroundGuard() if sys.platform == 'win32' else None
    if guard is not None:
        print(f'guard status (read-only): {guard.status()}')
        print(f'foreground now: {guard.describe_foreground()}')
    ov_key = read_cet_overlay_key()
    print(f'CET overlay key VKs: {[hex(v) for v in ov_key] if ov_key else "unknown (default 0xC0)"}')
    if (cmds or args.cet_reload) and guard is not None:
        ov = make_overlay(args, guard)
        print(f'CET toggle scan codes: {[hex(s) for s in ov.toggle_scans]}')
        print(f'CET console input click (screen px): {ov.console_click_screen()}')
        print(f'CET reload button: rel={ov.reload_rel} layout point={ov.reload_button_point()}')
    print(f'padd daemon: {padd_ping(port=args.padd_port) or "not running"}')
    log = resolve_log(args)
    print(f'UTPP log: {log or "not found"}')
    return 0


CREATE_NO_WINDOW = 0x08000000
CREATE_NEW_PROCESS_GROUP = 0x00000200


def padd_spawn_kwargs() -> dict:
    """Popen options that start padd.py without any visible window and without stealing focus.

    DETACHED_PROCESS (the earlier choice) leaves the child without a console; a venv launcher python.exe (e.g. the
    kimodo environment) then starts the real interpreter, which allocates a fresh console -> Windows Terminal opens a
    window over the game and the harness aborts with "foreground window is not the game" (2026-11-10).
    CREATE_NO_WINDOW gives the whole process tree one hidden console; STARTUPINFO SW_HIDE and a
    new process group (no Ctrl+C from the parent console) keep it quiet. The process is not tied to run.py, so
    padd stays alive after run.py exits."""
    import subprocess
    kw: dict = {'creationflags': CREATE_NO_WINDOW | CREATE_NEW_PROCESS_GROUP, 'close_fds': True,
                'stdin': subprocess.DEVNULL}
    if hasattr(subprocess, 'STARTUPINFO'):
        si = subprocess.STARTUPINFO()
        si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        si.wShowWindow = 0  # SW_HIDE
        kw['startupinfo'] = si
    return kw


def padd_command_line(port: int) -> list:
    return [sys.executable, os.path.join(HERE, 'padd.py'), '--port', str(port)]


def spawn_padd(port: int, timeout: float = 8.0) -> dict:
    """Start padd.py hidden and detached from run.py (log: Research\\logs\\gametest\\padd.log), wait for its ping."""
    import subprocess
    from gt_input import padd_ping
    os.makedirs(DEFAULT_OUT_ROOT, exist_ok=True)
    logf = open(os.path.join(DEFAULT_OUT_ROOT, 'padd.log'), 'a', encoding='utf-8')
    subprocess.Popen(padd_command_line(port), stdout=logf, stderr=subprocess.STDOUT, cwd=HERE, **padd_spawn_kwargs())
    logf.close()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = padd_ping(port=port)
        if st:
            return st
        time.sleep(0.25)
    raise GuardError('padd daemon did not start (see Research\\logs\\gametest\\padd.log)')


def open_pad(args, guard):
    """Returns (driver, newly_plugged)."""
    from gt_input import PadDriver, PaddClient, RemotePadDriver, padd_ping
    if not args.keep_pad:
        return PadDriver(guard), True
    fresh = False
    if not padd_ping(port=args.padd_port):
        spawn_padd(args.padd_port)
        fresh = True
    return RemotePadDriver(guard, PaddClient(port=args.padd_port)), fresh


def dismiss(guard, log_path, plan: dict | None = None) -> dict:
    from gt_input import dismiss_disconnect_dialog
    try:
        guard.ensure_focus()
        res = dismiss_disconnect_dialog(guard, log_path)
    except GuardError as e:
        res = {'sent': 0, 'verified': None, 'error': str(e)}
    state = {True: 'game running again (new MMTRACE lines)', False: 'NOT verified - check the game',
             None: 'sent, not verifiable (trace off)'}[res.get('verified')]
    print(f'[gametest] disconnect dialog: Enter x{res["sent"]} -> {res.get("error") or state}')
    if plan is not None:
        plan['dialog_dismiss'] = res
    return res


def live_run(args, chosen) -> int:
    out_dir = args.out or os.path.join(DEFAULT_OUT_ROOT, datetime.now().strftime('%Y%m%d-%H%M%S'))
    guard = ForegroundGuard(focus_grace=args.focus_grace)
    log_path = resolve_log(args)
    # rotation-safe start position (token string: file identity + offset; survives a 10 MB rotation of the UTPP log)
    mark = gt_log.log_mark(log_path) if gt_log.is_session_log(log_path) else None
    log_offset = mark.token() if mark is not None else (os.path.getsize(log_path) if log_path and os.path.isfile(log_path) else 0)
    plan = {'created': datetime.now(), 'log': log_path, 'log_offset': log_offset, 'runs': [],
            'args': {k: v for k, v in vars(args).items()}}
    pad = None
    error = None
    try:
        guard.ensure_focus()
        cmds = cet_commands(args, chosen)
        if args.cet_reload or cmds:
            ov = make_overlay(args, guard)
            if args.cet_reload:
                ok = ov.reload_all_mods()
                print(f'[gametest] CET reload all mods: {"confirmed in CET log" if ok else "NOT confirmed"}')
                plan['cet_reload_confirmed'] = ok
            if cmds:
                res = ov.run_commands(cmds, log_path=None if args.cet_no_verify else log_path)
                plan['cet_commands'] = res
                tries = getattr(ov, 'last_attempts', {})
                plan['cet_attempts'] = dict(tries)
                for c, v in res.items():
                    n = tries.get(c, 1)
                    print(f'[gametest] CET {c}: ' + {True: 'confirmed in UTPP log', False: 'NOT confirmed',
                                                     None: 'sent (not verifiable)'}[v]
                          + (f' (typed {n}x)' if n > 1 else ''))
                lost = [c for c, v in res.items() if v is False]
                if lost:
                    short = '; '.join(c[:70] + ('...' if len(c) > 70 else '') for c in lost[:3])
                    raise GuardError(f'CET command(s) not confirmed in the UTPP log after {max(tries.values(), default=1)} '
                                     f'attempt(s): {len(lost)} lost ({short}) - fix the console click/key '
                                     '(--cet-input-rel, --cet-key-scan) before running scenarios')
        if chosen:
            pad, fresh = open_pad(args, guard)
            plan['pad'] = {'persistent': pad.persistent, 'freshly_plugged': fresh}
            if fresh:
                guard.wait(args.pad_settle)
            pad.wake(args.wake)
            with TimerResolution():
                for sc in chosen:
                    for r in range(1, args.repeat + 1):
                        if plan['runs']:
                            guard.wait(args.gap)
                        print(f'[gametest] {sc.name} run {r}/{args.repeat} ({sc.duration:.1f} s)')
                        res = run_scenario(sc, pad, guard)
                        plan['runs'].append({'scenario': sc.name, 'run': r, **res})
                        if res['aborted']:
                            raise GuardError(res['aborted'])
    except GuardError as e:
        error = str(e)
        print(f'[gametest] ABORT: {e}', file=sys.stderr)
    except KeyboardInterrupt:
        error = 'Ctrl+C'
        print('[gametest] ABORT: Ctrl+C', file=sys.stderr)
    finally:
        if pad is not None:
            pad.close()
            if not pad.persistent and not args.no_dismiss and error != 'Ctrl+C':
                dismiss(guard, log_path, plan)
    plan['error'] = error
    plan['guard_events'] = guard.events
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'plan.json'), 'w', encoding='utf-8') as f:
        json.dump(serialise_plan(plan), f, indent=1, default=str)
    if not plan['runs']:
        print(f'[gametest] no scenario runs recorded; plan in {out_dir}')
        return 2 if error else 0
    time.sleep(args.flush_wait)
    return analyze_dir(out_dir, args, log_override=args.log)


def padd_command(args) -> int:
    from gt_input import PaddClient, padd_ping
    st = padd_ping(port=args.padd_port)
    if args.padd_status:
        print(f'padd: {st or "not running"}')
        return 0
    if not st:
        print('padd: not running')
        return 0
    c = PaddClient(port=args.padd_port)
    try:
        print(f'padd quit: {c.call({"op": "quit"})}')
    finally:
        c.close()
    if not args.no_dismiss and sys.platform == 'win32':
        dismiss(ForegroundGuard(), resolve_log(args))
    return 0


def analyze_dir(out_dir: str, args, log_override: str = '') -> int:
    plan = load_plan(os.path.join(out_dir, 'plan.json'))
    log_path = log_override or plan.get('log')
    if not log_path or not gt_log.session_files(log_path):
        print(f'[gametest] UTPP log not found: {log_path}', file=sys.stderr)
        return 2
    newest = gt_log.newest_log(os.path.dirname(log_path))
    if newest and os.path.normcase(newest) != os.path.normcase(log_path):
        print(f'[gametest] WARNING: a newer UTPP log exists ({newest}); the game may have restarted', file=sys.stderr)
    offset = plan.get('log_offset', 0) if not log_override else 0
    lines = gt_log.parse_file(log_path, offset)   # session log: rotated siblings included; a position is never reset to 0
    scenarios = load_scenarios(args.scenario_file)
    rows = analyze(plan, lines, load_thresholds(args.thresholds), scenarios, args.clock_offset)
    apply_precondition(plan, rows, lines, allow_base=getattr(args, 'allow_base', False), clock_offset_s=args.clock_offset)
    summary = write_outputs(out_dir, plan, rows, lines, log_path)
    print_table(rows, summary)
    print(f'[gametest] outputs: {out_dir}')
    return 0 if summary['overall'] == 'PASS' else 1


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    scenarios = load_scenarios(args.scenario_file)
    if args.list:
        for n, sc in scenarios.items():
            print(f'{n:26} {sc.duration:5.1f} s  {len(sc.steps)} steps  {sc.description}')
        return 0
    if args.analyze:
        return analyze_dir(args.analyze, args, log_override=args.log)
    if args.padd_status or (args.padd_stop and not args.dry_run):
        return padd_command(args)
    if args.dismiss_dialog and not args.dry_run:
        if sys.platform != 'win32':
            raise SystemExit('needs Windows')
        res = dismiss(ForegroundGuard(), resolve_log(args))
        return 0 if res.get('verified') is not False and not res.get('error') else 1
    chosen = select(scenarios, args.scenarios)
    if not chosen and not args.cet_reload and not args.cet_enable_mm and not args.cet:
        print('nothing to do (use --scenarios, --list, --cet-reload, --cet-enable-mm, --analyze, --padd-status, '
              '--padd-stop or --dismiss-dialog; the last two are not available with --dry-run)')
        return 1
    if args.repeat < 1:
        raise SystemExit('--repeat must be >= 1')
    if args.dry_run:
        return dry_run(args, chosen)
    if sys.platform != 'win32':
        raise SystemExit('live runs need Windows')
    return live_run(args, chosen)


if __name__ == '__main__':
    sys.exit(main())
