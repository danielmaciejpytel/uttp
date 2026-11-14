"""The pose meter inside a test run: smoothness and body shape per scenario, from the pose recording the run collected.

In a run (night.py) the work happens in worker processes (below normal priority, so the game quit and the other post-processing
never wait for it):
  steps  pose_metrics.analyze with one window per scenario (<name>#<run>) and per step (<name>#<run>/<step>), the windows the runner has
  auto   pose_metrics.analyze with windows='auto' (steady gaits, head look switches, clip changes, kinds of input, hitches)
  helper_coverage / helper_residual   tools\\analysis\\helper_coverage_report.py / helper_residual_report.py, only when they exist
The results are merged into pose_metrics.json (run folder), a readable table into pose_table.txt, and per-scenario lines go into the
run summary. Columns (ROW_COLUMNS): frame pacing and hitches, head / chest speed and its change, single-frame spikes per second with
their cause tags, clip changes per second, helper bone mismatch per bone group, joint anatomy share.

Limits: an optional "pose_limits" block per scenario in scenarios.json, {"<column>": 400} (a maximum) or {"<column>": {"max": 400,
"min": 0, "max_vs_baseline_pct": 20}}; none by default = report only. They are checked on the scenario window <name>#<run>.
--pose-baseline <run folder> prints before / after per scenario and per column with the change; the exit code is non-zero only when
a configured limit is exceeded.

  python -I gt_posemeter.py run --run-dir RUN [--out-dir OUT] [--collect-from GAME_LOG_DIR] [--pose-baseline RUN2]
        reads RUN\\cycle.json, RUN\\gametest\\plan.json and the recording files of RUN\\logs (or those collected from GAME_LOG_DIR
        into OUT\\logs), writes OUT\\pose_metrics.json and OUT\\pose_table.txt (default OUT = RUN)
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
TOOLS = os.path.dirname(HERE)
PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
ANALYSIS = os.path.join(TOOLS, 'analysis')
HELPER_REPORTS = {'helper_coverage': 'helper_coverage_report.py', 'helper_residual': 'helper_residual_report.py'}
PROBE_PREFIXES = ('clear_probe',)             # the runner's own look-ahead probes are not scenarios under test
HELPER_GROUP_NAMES = ('shoulder_chest', 'arm', 'leg', 'neck_head', 'hand', 'other')
# the meter's tag 'none' = a single-frame jump with no clip change, layer switch or hitch near it (cause not found), not "no spikes"
CAUSE_TEXT = {'none': 'unexplained'}
BELOW_NORMAL = 0x00004000
NO_WINDOW = 0x08000000


# ------------------------------------------------------------------------------------------------------------------ rows
def _g(d, *path):
    for p in path:
        if not isinstance(d, dict) or p not in d:
            return None
        d = d[p]
    return d


def _joint_top(w, n=2):
    joints = _g(w, 'deformation', 'anatomy', 'final', 'joints') or {}
    top = sorted(((v.get('share_outside') or 0.0, k) for k, v in joints.items()), reverse=True)[:n]
    return ', '.join(f'{k} {s * 100:.1f}%' for s, k in top if s > 0) or '-'


def window_row(w: dict) -> dict:
    """One flat row of scalar columns (numbers, None, or short text) out of a pose_metrics window result."""
    fin = _g(w, 'smoothness', 'final') or {}
    sm = lambda bone, k: _g(fin, bone, k)  # noqa: E731
    hm = _g(w, 'deformation', 'helper_mismatch', 'final') or {}
    an = _g(w, 'deformation', 'anatomy', 'final') or {}
    tags = _g(w, 'events', 'discontinuities_by_tag') or {}
    row = {
        'label': w.get('label'), 'start': w.get('start'), 'duration_s': w.get('duration_s'), 'movement_s': w.get('movement_s'),
        'frames': _g(w, 'frame_pacing', 'frames'), 'fps': _g(w, 'frame_pacing', 'fps'),
        'dt_p95_ms': _g(w, 'frame_pacing', 'dt_p95_ms'), 'dt_max_ms': _g(w, 'frame_pacing', 'dt_max_ms'),
        'hitch_share': _g(w, 'frame_pacing', 'hitch_share'), 'hitches': _g(w, 'frame_pacing', 'hitches'),
        'step_p95_ms': _g(w, 'frame_pacing', 'step_p95_ms'),
        'head_speed_p95': sm('Head.model', 'speed_deg_s_p95'), 'head_speed_max': sm('Head.model', 'speed_deg_s_max'),
        'head_change_p99': sm('Head.model', 'change_deg_s2_p99'), 'head_change_max': sm('Head.model', 'change_deg_s2_max'),
        'chest_speed_p95': sm('Spine3.model', 'speed_deg_s_p95'), 'chest_speed_max': sm('Spine3.model', 'speed_deg_s_max'),
        'chest_change_p99': sm('Spine3.model', 'change_deg_s2_p99'), 'chest_change_max': sm('Spine3.model', 'change_deg_s2_max'),
        'head_vs_chest_speed_max': sm('Head_vs_Spine3.chain', 'speed_deg_s_max'),
        'head_vs_chest_change_p99': sm('Head_vs_Spine3.chain', 'change_deg_s2_p99'),
        'spike_frames': _g(w, 'smoothness', 'final_spike_frames'),
        'spikes_per_s': _g(w, 'smoothness', 'final_spike_frames_per_move_s'),
        'spike_causes': ', '.join(f'{CAUSE_TEXT.get(k, k)} {v}' for k, v in sorted(tags.items())) or '-',
        'clip_changes': _g(w, 'events', 'clip_changes'), 'clip_changes_per_s': _g(w, 'events', 'clip_changes_per_move_s'),
        'helper_max_cm': hm.get('max_cm'), 'helper_p95_cm': hm.get('p95_cm'), 'helper_samples': hm.get('samples'),
        'anatomy_share': an.get('frames_any_share'), 'anatomy_worst_deg': an.get('worst_deg'),
        'anatomy_worst_joint': an.get('worst_joint'), 'anatomy_top_joints': _joint_top(w),
    }
    for g in HELPER_GROUP_NAMES:
        row[f'helper_{g}_p95_cm'] = _g(hm, 'groups', g, 'p95_cm')
        row[f'helper_{g}_max_cm'] = _g(hm, 'groups', g, 'max_cm')
    return row


def numeric_columns(rows) -> list[str]:
    cols = []
    for r in rows:
        for k, v in r.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool) and k not in cols and k not in ('frames', 'duration_s', 'movement_s',
                                                                                                     'helper_samples'):
                cols.append(k)
    return cols


def is_scenario_label(label: str) -> bool:
    return '/' not in label


# ------------------------------------------------------------------------------------------------------------------ windows
def _dt(x):
    if isinstance(x, datetime):
        return x
    return datetime.fromisoformat(str(x))


MACRO_SUFFIX = '~macro'


def build_windows(scenarios, plan_runs) -> list[tuple[str, datetime, datetime]]:
    """[(label, start, end)] from the runner's records.
    '<scenario>#<run>/<step>' for every step of a plan run (the runner's own probe walks are left out).
    '<scenario>#<run>' = the scenario as played: first step start to last step end of its plan run(s), so the walks the runner makes
    around a scenario (look-ahead probe, walk back, recovery) are not part of it.
    A scenario record without a plan run of its own name (a macro: it plays other scenarios and commands) gets '<name>#<run>~macro',
    its whole time span including what the runner did; limits are not checked on it. The 'phase_windows' of a macro record add one row
    '<label>#<run>' per labelled phase (the ranged weapon families: rf_<family>_<phase>), also without a limit."""
    out = []
    steps_of: dict[tuple[str, int], list] = {}
    for r in plan_runs or []:
        if r['scenario'].startswith(PROBE_PREFIXES):
            continue
        for st in r.get('steps', []):
            if st.get('start') and st.get('end'):
                steps_of.setdefault((r['scenario'], r['run']), []).append((st['name'], _dt(st['start']), _dt(st['end'])))
    for s in scenarios or []:
        if not (s.get('end') and s.get('start')) or s['name'].startswith(PROBE_PREFIXES):
            continue
        own = steps_of.get((s['name'], s['run']))
        if own:
            out.append((f'{s["name"]}#{s["run"]}', min(a for _, a, _ in own), max(b for _, _, b in own)))
        else:
            out.append((f'{s["name"]}#{s["run"]}{MACRO_SUFFIX}', _dt(s['start']), _dt(s['end'])))
    for s in scenarios or []:                  # labelled phases of a macro (rf_<family>_<phase>): one row per phase, report only
        for w in s.get('phase_windows') or []:
            if w.get('label') and w.get('start') and w.get('end'):
                out.append((f'{w["label"]}#{s["run"]}', _dt(w['start']), _dt(w['end'])))
    for (name, run), steps in steps_of.items():
        for sname, a, b in steps:
            out.append((f'{name}#{run}/{sname}', a, b))
    return out


# ------------------------------------------------------------------------------------------------------------------ workers
def meter_python() -> str:
    """An interpreter that has numpy (the meter needs it): this one, else the project's tool environment."""
    try:
        import numpy  # noqa: F401
        return sys.executable
    except Exception:  # noqa: BLE001
        cand = os.path.join(PROJECT, 'Tools', 'ai-motion', 'envs', 'kimodo', 'Scripts', 'python.exe')
        return cand if os.path.isfile(cand) else sys.executable


def worker_main(argv) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=['steps', 'auto'])
    ap.add_argument('--pose', nargs='+', required=True)
    ap.add_argument('--windows-json')
    ap.add_argument('--log')
    ap.add_argument('--feed')
    ap.add_argument('--out', required=True)
    ap.add_argument('--tz-hours', type=float, help='local-minus-UTC hours of the log time base (only for recordings without a stamped name)')
    a = ap.parse_args(argv)
    import pose_metrics as pm
    t0 = time.time()
    try:
        if a.kind == 'steps':
            with open(a.windows_json, encoding='utf-8') as f:
                wins = [(l, _dt(s), _dt(e)) for l, s, e in json.load(f)]
            res = pm.analyze(a.pose, a.log, wins, a.feed, tz_hours=a.tz_hours)
        else:
            res = pm.analyze(a.pose, a.log, 'auto', a.feed, tz_hours=a.tz_hours)
        res['analysis_seconds'] = round(time.time() - t0, 1)
        res['ok'] = True
    except Exception as e:  # noqa: BLE001 - the parent reads the error from the file
        res = {'ok': False, 'error': repr(e), 'analysis_seconds': round(time.time() - t0, 1)}
    tmp = a.out + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(res, f, default=str)
    os.replace(tmp, a.out)
    return 0 if res.get('ok') else 1


def helper_script(kind: str) -> str | None:
    p = os.path.join(ANALYSIS, HELPER_REPORTS[kind])
    return p if os.path.isfile(p) else None


HELPER_RC = {'helper_coverage': {0: 'OK', 1: 'UNDER_LIMIT', 2: 'NO_DATA'},
             'helper_residual': {0: 'PASS_OR_NO_MOD_WRITES', 1: 'FAIL', 2: 'NOT_VERIFIABLE'}}


def helper_command(kind: str, python: str, script: str, pose_dir: str, log_file: str | None, plan_json: str | None, work_dir: str):
    """(command, result path) of a helper report. helper_coverage_report.py reads the mod log (coverage counters per scenario run
    of plan.json); helper_residual_report.py reads the pose recording folder (stale helper bones per plan step)."""
    if kind == 'helper_coverage':
        out = os.path.join(work_dir, 'helper_coverage.json')
        return [python, '-I', script, log_file or '', '--json', out] + (['--plan', plan_json] if plan_json else []), out
    out_dir = os.path.join(work_dir, 'helper_residual')
    return ([python, '-I', script, pose_dir, '--out-dir', out_dir] + (['--plan', plan_json] if plan_json else []),
            os.path.join(out_dir, 'helper_residual.json'))


def helper_verdict(kind: str, rc, data) -> str:
    d = data[0] if isinstance(data, list) and data else data
    status = d.get('status') if isinstance(d, dict) else None
    return status or HELPER_RC.get(kind, {}).get(rc, f'rc {rc}')


class MeterJob:
    """The worker processes of one run. start() returns at once; finish(timeout) waits for them."""

    def __init__(self, work_dir: str, pose_files: list[str], windows, log_file=None, feed_file=None, python: str | None = None,
                 popen=subprocess.Popen, helper_reports: bool = True, tz_hours: float | None = None):
        self.work_dir = work_dir
        self.pose_files = list(pose_files)
        self.windows = list(windows)
        self.log_file, self.feed_file = log_file, feed_file
        self.python = python or meter_python()
        self.popen = popen
        self.helper_reports = helper_reports
        self.tz_hours = tz_hours
        self.procs: dict[str, tuple] = {}              # kind -> (Popen, out path)
        self.started = None

    def _spawn(self, kind: str, cmd: list[str], out: str) -> None:
        flags = (BELOW_NORMAL | NO_WINDOW) if os.name == 'nt' else 0
        try:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            err = open(out + '.err', 'w', encoding='utf-8')
            p = self.popen(cmd, stdout=err, stderr=subprocess.STDOUT, creationflags=flags)
            self.procs[kind] = (p, out, err)
        except OSError as e:
            self.procs[kind] = (None, out, repr(e))

    def start(self) -> 'MeterJob':
        os.makedirs(self.work_dir, exist_ok=True)
        self.started = time.time()
        wj = os.path.join(self.work_dir, 'windows.json')
        with open(wj, 'w', encoding='utf-8') as f:
            json.dump([(l, a.isoformat(), b.isoformat()) for l, a, b in self.windows], f)
        me = os.path.abspath(__file__)
        base = [self.python, '-I', me, 'worker']
        extra = ((['--log', self.log_file] if self.log_file else []) + (['--feed', self.feed_file] if self.feed_file else [])
                 + (['--tz-hours', str(self.tz_hours)] if self.tz_hours is not None else []))
        self._spawn('steps', base + ['steps', '--pose', *self.pose_files, '--windows-json', wj, *extra,
                                     '--out', os.path.join(self.work_dir, 'meter_steps.json')],
                    os.path.join(self.work_dir, 'meter_steps.json'))
        self._spawn('auto', base + ['auto', '--pose', *self.pose_files, *extra, '--out', os.path.join(self.work_dir, 'meter_auto.json')],
                    os.path.join(self.work_dir, 'meter_auto.json'))
        return self

    def start_helper_reports(self, plan_json: str | None = None) -> list[str]:
        """Starts helper_coverage_report.py / helper_residual_report.py (tools/analysis) when they exist; returns the kinds started.
        Their verdicts are attached to the result; they never decide the run."""
        started = []
        if not self.helper_reports:
            return started
        pose_dir = os.path.dirname(os.path.abspath(self.pose_files[0]))
        log_file = self.log_file
        if not log_file:
            import gt_collect
            log_file = gt_collect.session_log(pose_dir)
        for kind in HELPER_REPORTS:
            script = helper_script(kind)
            if not script or (kind == 'helper_coverage' and not log_file):
                continue
            cmd, out = helper_command(kind, self.python, script, pose_dir, log_file, plan_json, self.work_dir)
            self._spawn(kind, cmd, out)
            started.append(kind)
        return started

    def finish(self, timeout: float = 900.0) -> dict:
        """-> {'steps': result|None, 'auto': result|None, 'helper_reports': {kind: {...}}, 'errors': [...]}"""
        deadline = time.time() + timeout
        out = {'steps': None, 'auto': None, 'helper_reports': {}, 'errors': []}
        for kind, (p, path, err) in self.procs.items():
            if p is None:
                out['errors'].append(f'{kind}: not started ({err})')
                continue
            try:
                p.wait(timeout=max(1.0, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
                out['errors'].append(f'{kind}: timed out after {timeout:.0f} s (killed)')
            err.close()
            data = None
            if os.path.isfile(path):
                try:
                    with open(path, encoding='utf-8') as f:
                        data = json.load(f)
                except (OSError, ValueError) as e:
                    out['errors'].append(f'{kind}: result unreadable ({e!r})')
            tail = ''
            try:
                with open(path + '.err', encoding='utf-8', errors='replace') as f:
                    tail = f.read()[-1500:].strip()
            except OSError:
                pass
            if kind in ('steps', 'auto'):
                if data and data.get('ok'):
                    out[kind] = data
                else:
                    out['errors'].append(f'{kind}: ' + ((data or {}).get('error') or f'no result (rc {p.returncode}) {tail}'))
            else:
                hr = {'rc': p.returncode, 'verdict': helper_verdict(kind, p.returncode, data), 'result_file': path if data is not None else None,
                      'output_tail': tail}
                out['helper_reports'][kind] = hr
        return out


# ------------------------------------------------------------------------------------------------------------------ assembly
def assemble(run_name: str, job_out: dict, windows, recordings: dict | None = None, load: dict | None = None) -> dict:
    steps = job_out.get('steps')
    auto = job_out.get('auto')
    rows = [window_row(w) for w in (steps or {}).get('windows', [])]
    got = {r['label'] for r in rows}
    res = {'run': run_name, 'created': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
           'pose_files': (steps or auto or {}).get('pose_files'), 'frames': (steps or auto or {}).get('frames'),
           'duration_s': (steps or auto or {}).get('duration_s'), 'body': (steps or auto or {}).get('body'),
           'feed_alignment': (steps or auto or {}).get('feed_alignment'), 'notes': (steps or auto or {}).get('notes'),
           'post_helper_stream': (steps or auto or {}).get('post_helper_stream'),
           'median_frame_ms': (steps or auto or {}).get('median_frame_ms'),
           'rows': rows, 'windows_requested': len(windows),
           'windows_without_result': [l for l, _, _ in windows if l not in got],
           'step_windows': (steps or {}).get('windows'), 'errors': job_out.get('errors', []),
           'helper_reports': job_out.get('helper_reports', {}), 'recordings': recordings, 'machine_load': load}
    if auto:
        arows = [window_row(w) for w in auto.get('windows', [])]
        groups = {}
        for r in arows:
            groups.setdefault(r['label'], []).append(r)
        res['auto'] = {'windows': len(arows), 'groups': {g: aggregate_rows(v) for g, v in sorted(groups.items())},
                       'analysis_seconds': auto.get('analysis_seconds')}
    res['steps_analysis_seconds'] = (steps or {}).get('analysis_seconds')
    return res


def aggregate_rows(rows) -> dict:
    import numpy as np
    out = {'windows': len(rows)}
    for c in numeric_columns(rows):
        x = np.array([r[c] for r in rows if isinstance(r.get(c), (int, float))], dtype=float)
        if len(x):
            out[c] = {'n': int(len(x)), 'p50': round(float(np.percentile(x, 50)), 3), 'p95': round(float(np.percentile(x, 95)), 3),
                      'max': round(float(x.max()), 3)}
    return out


# ------------------------------------------------------------------------------------------------------------------ text
def _f(v, nd=1, none='-'):
    if v is None:
        return none
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def scenario_lines(res: dict) -> list[str]:
    """One line per scenario window for the run summary."""
    lines = []
    for r in res.get('rows', []):
        if not is_scenario_label(r['label']):
            continue
        worst_h = max(((r.get(f'helper_{g}_max_cm') or 0.0, g) for g in HELPER_GROUP_NAMES), default=(0.0, '-'))
        lines.append(
            f'pose {r["label"]}: {_f(r["duration_s"])} s | head {_f(r["head_speed_p95"], 0)}/{_f(r["head_speed_max"], 0)} deg/s '
            f'(p95/max), change p99 {_f(r["head_change_p99"], 0)} | chest {_f(r["chest_speed_p95"], 0)}/{_f(r["chest_speed_max"], 0)}, '
            f'change p99 {_f(r["chest_change_p99"], 0)} | spikes {_f(r["spikes_per_s"], 2)}/s [{r["spike_causes"]}] | '
            f'clip changes {_f(r["clip_changes_per_s"], 2)}/s | hitches {_f((r["hitch_share"] or 0) * 100, 1)} % dt p95 {_f(r["dt_p95_ms"])} ms | '
            f'helper p95 {_f(r["helper_p95_cm"], 2)} cm max {_f(r["helper_max_cm"], 2)} ({worst_h[1] if worst_h[0] else "-"}) | '
            f'anatomy {_f((r["anatomy_share"] or 0) * 100, 1)} % frames, worst {_f(r["anatomy_worst_deg"])} deg ({r["anatomy_worst_joint"] or "-"})')
    return lines


def _table(headers: list[str], rows: list[list[str]]) -> list[str]:
    w = [max(len(h), *(len(r[i]) for r in rows)) if rows else len(h) for i, h in enumerate(headers)]
    out = ['  '.join(h.ljust(w[i]) if i < 2 else h.rjust(w[i]) for i, h in enumerate(headers)),
           '  '.join('-' * w[i] for i in range(len(headers)))]
    for r in rows:
        out.append('  '.join(c.ljust(w[i]) if i < 2 else c.rjust(w[i]) for i, c in enumerate(r)))
    return out


def format_tables(res: dict) -> list[str]:
    """The readable tables: scenario / step, then smoothness, frame pacing, deformation. '-' = no value (e.g. no movement)."""
    rows = res.get('rows', [])

    def split(label):
        sc, _, st = label.partition('/')
        return sc, st or '(whole)'
    out = [f'Pose meter  run {res.get("run")}  body {res.get("body")}  {res.get("frames")} frames  {res.get("duration_s")} s  '
           f'median frame {res.get("median_frame_ms")} ms  feed: {res.get("feed_alignment")}  '
           f'post-helper stream in the recording: {res.get("post_helper_stream")}', '']
    out.append('Rows: <scenario>#<run> = the scenario as played (its first step start to its last step end, without the runner\'s own '
               'probe walks), <scenario>#<run>/<step> = one step; "<name>~macro" = the whole span of a macro scenario, including what the '
               'runner did.')
    out.append('Spike causes: clip_change / layer_switch / hitch = a single-frame jump next to that event; unexplained = a jump with none '
               'of them near. "-" in a column = no value (for example under 0.5 s of movement), never zero.')
    out.append('Helper mismatch = the recorded skin helper bones against the helper bones recomputed from the recorded core pose, per bone '
               'group (cm); it measures how far the recorded helpers are from their recomputed values, not a rendered deformation.')
    empty = [g for g in HELPER_GROUP_NAMES if rows and all(r.get(f'helper_{g}_p95_cm') is None for r in rows)]
    if empty:
        out.append('No helper samples for the group(s): ' + ', '.join(empty) + ' (no sampled helper bone of the recording is in that group).')
    out.append('')
    out.append('A. Smoothness (final pose; deg/s = angular speed, change = its change per second; spikes = single-frame jumps per movement second)')
    out += _table(['scenario', 'step', 's', 'head p95', 'head max', 'head chg99', 'chest p95', 'chest max', 'chest chg99', 'head/chest max',
                   'spikes/s', 'spike causes', 'clip chg/s'],
                  [[*split(r['label']), _f(r['duration_s']), _f(r['head_speed_p95'], 0), _f(r['head_speed_max'], 0),
                    _f(r['head_change_p99'], 0), _f(r['chest_speed_p95'], 0), _f(r['chest_speed_max'], 0), _f(r['chest_change_p99'], 0),
                    _f(r['head_vs_chest_speed_max'], 0), _f(r['spikes_per_s'], 2), r['spike_causes'], _f(r['clip_changes_per_s'], 2)]
                   for r in rows])
    out += ['', 'B. Frame pacing']
    out += _table(['scenario', 'step', 'frames', 'fps', 'dt p95 ms', 'dt max ms', 'hitch %', 'hitches', 'step p95 ms'],
                  [[*split(r['label']), _f(r['frames']), _f(r['fps']), _f(r['dt_p95_ms']), _f(r['dt_max_ms']),
                    _f((r['hitch_share'] or 0) * 100, 1) if r['hitch_share'] is not None else '-', _f(r['hitches']), _f(r['step_p95_ms'])]
                   for r in rows])
    out += ['', 'C. Deformation (helper mismatch p95 / max in cm per bone group; anatomy = share of frames with a joint outside its band)']
    out += _table(['scenario', 'step', *[f'{g[:11]} p95' for g in HELPER_GROUP_NAMES], 'helper max', 'anat %', 'worst deg', 'worst joint'],
                  [[*split(r['label']), *[_f(r[f'helper_{g}_p95_cm'], 2) for g in HELPER_GROUP_NAMES], _f(r['helper_max_cm'], 2),
                    _f((r['anatomy_share'] or 0) * 100, 1) if r['anatomy_share'] is not None else '-', _f(r['anatomy_worst_deg']),
                    r['anatomy_worst_joint'] or '-'] for r in rows])
    if res.get('windows_without_result'):
        out += ['', 'No result (window under 3 frames or outside the recording): ' + ', '.join(res['windows_without_result'])]
    for e in res.get('errors', []):
        out.append('ERROR ' + e)
    return out


# ------------------------------------------------------------------------------------------------------------------ baseline / limits
def compare_rows(after: list[dict], before: list[dict], scenarios_only: bool = False) -> list[dict]:
    """[{label, metric, before, after, change, pct}] for every label both runs have and every numeric column with a value in either."""
    b = {r['label']: r for r in before}
    out = []
    for r in after:
        if scenarios_only and not is_scenario_label(r['label']):
            continue
        o = b.get(r['label'])
        if not o:
            continue
        for c in numeric_columns([r, o]):
            x, y = o.get(c), r.get(c)
            if x is None and y is None:
                continue
            ch = None if x is None or y is None else round(y - x, 4)
            pct = None if ch is None or not x else round(100.0 * ch / abs(x), 1)
            out.append({'label': r['label'], 'metric': c, 'before': x, 'after': y, 'change': ch, 'pct': pct})
    return out


def format_comparison(cmp_rows: list[dict], only_changed: bool = False) -> list[str]:
    lines = []
    label = None
    for c in cmp_rows:
        if only_changed and not c['change']:
            continue
        if c['label'] != label:
            label = c['label']
            lines.append(f'{label}')
        lines.append(f'    {c["metric"]:<28} before {_f(c["before"], 3):>10}  after {_f(c["after"], 3):>10}  change '
                     f'{_f(c["change"], 3):>10}' + (f'  ({c["pct"]:+.1f} %)' if c['pct'] is not None else ''))
    return lines


def load_pose_limits(scenario_file: str) -> dict:
    """{scenario: {column: spec}} of the optional pose_limits blocks; no block = nothing is checked."""
    try:
        with open(scenario_file, encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return {}
    return {n: dict(s['pose_limits']) for n, s in (data.get('scenarios') or {}).items() if isinstance(s, dict) and s.get('pose_limits')}


def check_limits(rows: list[dict], limits: dict, baseline_rows: list[dict] | None = None) -> list[dict]:
    """Violations [{label, metric, value, limit, kind}]; only scenario windows of scenarios that configure limits are checked."""
    base = {r['label']: r for r in (baseline_rows or [])}
    bad = []
    for r in rows:
        if not is_scenario_label(r['label']) or MACRO_SUFFIX in r['label']:
            continue                                   # steps and macro spans (they include the runner's own actions) are not limited
        spec = limits.get(r['label'].split('#')[0])
        if not spec:
            continue
        for col, lim in spec.items():
            v = r.get(col)
            if v is None:
                continue
            lim = {'max': lim} if isinstance(lim, (int, float)) else dict(lim)
            if 'max' in lim and v > lim['max']:
                bad.append({'label': r['label'], 'metric': col, 'value': v, 'limit': lim['max'], 'kind': 'max'})
            if 'min' in lim and v < lim['min']:
                bad.append({'label': r['label'], 'metric': col, 'value': v, 'limit': lim['min'], 'kind': 'min'})
            if 'max_vs_baseline_pct' in lim and r['label'] in base:
                o = base[r['label']].get(col)
                if o not in (None, 0) and v > o * (1 + lim['max_vs_baseline_pct'] / 100.0):
                    bad.append({'label': r['label'], 'metric': col, 'value': v, 'limit': round(o * (1 + lim['max_vs_baseline_pct'] / 100.0), 4),
                                'kind': f'baseline +{lim["max_vs_baseline_pct"]} %'})
    return bad


def load_rows(run_dir: str) -> list[dict]:
    with open(os.path.join(run_dir, 'pose_metrics.json'), encoding='utf-8') as f:
        return json.load(f).get('rows', [])


def load_body(run_dir: str) -> str | None:
    """The body ('woman' / 'man') of a run folder's pose_metrics.json, None = not recorded in it."""
    try:
        with open(os.path.join(run_dir, 'pose_metrics.json'), encoding='utf-8') as f:
            return json.load(f).get('body') or None
    except (OSError, ValueError):
        return None


def write_outputs(out_dir: str, res: dict) -> None:
    os.makedirs(out_dir, exist_ok=True)
    tmp = os.path.join(out_dir, 'pose_metrics.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1, default=str)
    os.replace(tmp, os.path.join(out_dir, 'pose_metrics.json'))
    with open(os.path.join(out_dir, 'pose_table.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(format_tables(res)) + '\n')


# ------------------------------------------------------------------------------------------------------------------ CLI
def run_main(argv) -> int:
    ap = argparse.ArgumentParser(prog='gt_posemeter.py run')
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--out-dir')
    ap.add_argument('--collect-from', help='game log folder: the recording files are collected from here into OUT\\logs first')
    ap.add_argument('--scenario-file', default=os.path.join(HERE, 'scenarios.json'))
    ap.add_argument('--pose-baseline')
    ap.add_argument('--no-helper-reports', action='store_true')
    ap.add_argument('--timeout', type=float, default=1800.0)
    a = ap.parse_args(argv)
    out_dir = a.out_dir or a.run_dir
    with open(os.path.join(a.run_dir, 'cycle.json'), encoding='utf-8') as f:
        cycle = json.load(f)
    plan_runs = []
    pj = os.path.join(a.run_dir, 'gametest', 'plan.json')
    if os.path.isfile(pj):
        with open(pj, encoding='utf-8') as f:
            plan_runs = json.load(f).get('runs', [])
    import gt_collect
    logs = os.path.join(out_dir, 'logs')
    recordings = None
    if a.collect_from:
        log0 = gt_collect.session_log(a.collect_from)
        coll = gt_collect.SessionCollector(a.collect_from, logs, gt_collect.session_since(log0),
                                           (lambda: [log0] if log0 else []))
        coll.collect('replay', final=True)
        recordings = coll.summary()
        print('\n'.join(gt_collect.format_manifest(recordings)))
    else:
        logs = os.path.join(a.run_dir, 'logs')
    files = gt_collect.pose_files(logs)
    if not files:
        print(f'no pose recording in {logs}')
        return 2
    windows = build_windows(cycle.get('scenarios'), plan_runs)
    t0 = time.time()
    job = MeterJob(os.path.join(out_dir, '_tmp'), files, windows, helper_reports=not a.no_helper_reports).start()
    if os.path.isfile(pj):
        job.start_helper_reports(pj)
    jo = job.finish(a.timeout)
    res = assemble(os.path.basename(os.path.abspath(a.run_dir)), jo, windows, recordings)
    res['seconds'] = round(time.time() - t0, 1)
    write_outputs(out_dir, res)
    print('\n'.join(format_tables(res)))
    rc = 0
    base = load_rows(a.pose_baseline) if a.pose_baseline else None
    if base is not None:
        print('\n' + '\n'.join(format_comparison(compare_rows(res['rows'], base), only_changed=False)))
    bad = check_limits(res['rows'], load_pose_limits(a.scenario_file), base)
    for b in bad:
        print(f'LIMIT EXCEEDED {b["label"]} {b["metric"]} = {b["value"]} ({b["kind"]} {b["limit"]})')
    if res['errors']:
        print('METER ERROR ' + '; '.join(res['errors']))
        return 3
    return 1 if bad else rc


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == 'worker':
        return worker_main(argv[1:])
    if argv and argv[0] == 'run':
        return run_main(argv[1:])
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
