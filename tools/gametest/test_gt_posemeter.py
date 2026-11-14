"""Offline tests of gt_posemeter.py (the pose meter inside a test run) and gt_load.py (machine load).

  python Source\\tools\\gametest\\test_gt_posemeter.py -v

The pure parts (rows, windows, baseline comparison, limits) run on synthetic windows. The worker processes and the command line run
on the small real slice in testdata\\ (pose_metrics_core.utpppose with its log and feed excerpts), arranged like a run folder.
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_load  # noqa: E402
import gt_posemeter as gp  # noqa: E402
import pose_metrics as pm  # noqa: E402

DATA = os.path.join(HERE, 'testdata')
CORE = os.path.join(DATA, 'pose_metrics_core.utpppose')
LOG = os.path.join(DATA, 'pose_metrics_core.session.txt')
FEED = os.path.join(DATA, 'pose_metrics_core.feed.txt')
TZ = 1.0


def window(label, **over):
    w = {'label': label, 'start': '01:43:53.000', 'duration_s': 5.0, 'movement_s': 4.0,
         'frame_pacing': {'frames': 300, 'fps': 60.0, 'dt_p95_ms': 18.0, 'dt_max_ms': 25.0, 'hitch_share': 0.01, 'hitches': 3, 'step_p95_ms': 18.5},
         'smoothness': {'final': {'Head.model': {'speed_deg_s_p95': 120.0, 'speed_deg_s_max': 300.0, 'change_deg_s2_p99': 900.0, 'change_deg_s2_max': 1500.0},
                                  'Spine3.model': {'speed_deg_s_p95': 40.0, 'speed_deg_s_max': 90.0, 'change_deg_s2_p99': 200.0, 'change_deg_s2_max': 400.0},
                                  'Head_vs_Spine3.chain': {'speed_deg_s_max': 250.0, 'change_deg_s2_p99': 700.0}},
                        'final_spike_frames': 2, 'final_spike_frames_per_move_s': 0.5},
         'events': {'clip_changes': 4, 'clip_changes_per_move_s': 1.0, 'discontinuities_by_tag': {'clip_change': 1, 'none': 1}},
         'deformation': {'helper_mismatch': {'final': {'samples': 8, 'max_cm': 2.0, 'p95_cm': 1.0,
                                                       'groups': {'arm': {'p95_cm': 0.5, 'max_cm': 2.0}, 'leg': {'p95_cm': 0.2, 'max_cm': 0.4}}}},
                         'anatomy': {'final': {'frames_any_share': 0.1, 'worst_deg': 12.0, 'worst_joint': 'LeftForeArm',
                                               'joints': {'LeftForeArm': {'share_outside': 0.1}, 'Neck': {'share_outside': 0.0}}}}}}
    for k, v in over.items():
        w[k] = v
    return w


class RowTests(unittest.TestCase):
    def test_row_columns(self):
        r = gp.window_row(window('walk#1'))
        self.assertEqual((r['head_speed_p95'], r['head_speed_max'], r['head_change_p99']), (120.0, 300.0, 900.0))
        self.assertEqual((r['chest_speed_p95'], r['chest_change_p99']), (40.0, 200.0))
        self.assertEqual((r['spikes_per_s'], r['spike_causes'], r['clip_changes_per_s']), (0.5, 'clip_change 1, unexplained 1', 1.0))
        self.assertEqual((r['hitch_share'], r['dt_p95_ms'], r['step_p95_ms']), (0.01, 18.0, 18.5))
        self.assertEqual((r['helper_arm_p95_cm'], r['helper_arm_max_cm'], r['helper_shoulder_chest_p95_cm']), (0.5, 2.0, None))
        self.assertEqual((r['anatomy_share'], r['anatomy_worst_joint'], r['anatomy_top_joints']), (0.1, 'LeftForeArm', 'LeftForeArm 10.0%'))
        json.dumps(r)

    def test_missing_values_stay_none_not_zero(self):
        r = gp.window_row({'label': 'x'})
        self.assertIsNone(r['spikes_per_s'])
        self.assertIsNone(r['clip_changes_per_s'])
        self.assertIsNone(r['head_speed_p95'])
        line = gp.scenario_lines({'rows': [r]})[0]
        self.assertIn('spikes -/s', line)                                  # a dash, never "0.00"
        self.assertIn('clip changes -/s', line)

    def test_tables_are_readable_and_complete(self):
        rows = [gp.window_row(window('walk#1')), gp.window_row(window('walk#1/taps')), gp.window_row({'label': 'idle#1/rest'})]
        text = '\n'.join(gp.format_tables({'run': 'r', 'body': 'woman', 'frames': 1, 'duration_s': 1, 'feed_alignment': 'aligned', 'rows': rows,
                                           'windows_without_result': ['short#1/x'], 'errors': ['auto: boom']}))
        for needle in ('A. Smoothness', 'B. Frame pacing', 'C. Deformation', 'walk#1', '(whole)', 'taps', 'rest', 'clip_change 1, unexplained 1', 'post-helper stream', 'Helper mismatch =',
                       'No helper samples for the group(s)', 'never zero',
                       'short#1/x', 'ERROR auto: boom', 'LeftForeArm'):
            self.assertIn(needle, text)
        self.assertEqual(gp.format_tables({'rows': []})[0].split()[0], 'Pose')


class WindowTests(unittest.TestCase):
    def test_windows_come_from_the_runner_records_and_skip_the_probes(self):
        sc = [{'name': 'kbd_taps', 'run': 1, 'start': '2026-11-14 01:43:53.911309', 'end': datetime(2026, 11, 14, 1, 44, 1)},
              {'name': 'open', 'run': 1, 'start': datetime(2026, 11, 14, 1, 44, 2), 'end': None}]
        runs = [{'scenario': 'clear_probe', 'run': 3, 'steps': [{'name': 'probe', 'start': '2026-11-14 01:43:50', 'end': '2026-11-14 01:43:51'}]},
                {'scenario': 'kbd_taps', 'run': 1, 'steps': [{'name': 'settle', 'start': '2026-11-14 01:43:53.926403', 'end': '2026-11-14 01:43:55.426665'},
                                                           {'name': 'taps', 'start': '2026-11-14 01:43:55.426665', 'end': '2026-11-14 01:44:00.426847'}]}]
        w = gp.build_windows(sc, runs)
        self.assertEqual([x[0] for x in w], ['kbd_taps#1', 'kbd_taps#1/settle', 'kbd_taps#1/taps'])
        self.assertIsInstance(w[0][1], datetime)
        # the scenario window is its steps (first start to last end), NOT the runner's record that includes its own probe walk
        self.assertEqual(w[0][1], datetime(2026, 11, 14, 1, 43, 53, 926403))
        self.assertEqual(w[0][2], datetime(2026, 11, 14, 1, 44, 0, 426847))

    def test_scenario_window_equals_the_sum_of_its_steps_and_macros_are_marked(self):
        t = datetime(2026, 11, 14, 2, 0, 0)
        at = lambda s: t + timedelta(seconds=s)   # noqa: E731
        scs = [{'name': 'walk', 'run': 1, 'start': at(0), 'end': at(20)},      # includes a 5 s probe walk before it
               {'name': 'melee_fists', 'run': 1, 'start': at(30), 'end': at(50)}]
        runs = [{'scenario': 'walk', 'run': 1, 'steps': [{'name': 'a', 'start': at(5), 'end': at(9)},
                                                       {'name': 'b', 'start': at(9), 'end': at(15)}]},
                {'scenario': 'melee_punch', 'run': 1, 'steps': [{'name': 'x', 'start': at(32), 'end': at(36)}]}]
        w = {l: (a, b) for l, a, b in gp.build_windows(scs, runs)}
        self.assertEqual((w['walk#1'][1] - w['walk#1'][0]).total_seconds(), 10.0)
        steps = (w['walk#1/a'][1] - w['walk#1/a'][0]) + (w['walk#1/b'][1] - w['walk#1/b'][0])
        self.assertEqual(w['walk#1'][1] - w['walk#1'][0], steps)
        self.assertIn('melee_fists#1~macro', w)                                       # no plan run of its own name: the whole span
        self.assertEqual((w['melee_fists#1~macro'][1] - w['melee_fists#1~macro'][0]).total_seconds(), 20.0)
        self.assertIn('melee_punch#1/x', w)
        rows = [dict(gp.window_row({'label': 'melee_fists#1~macro', 'duration_s': 20}), head_speed_p95=999.0)]
        self.assertEqual(gp.check_limits(rows, {'melee_fists': {'head_speed_p95': 1}}), [])   # no limits on a macro span


class BaselineAndLimitTests(unittest.TestCase):
    def rows(self, head, spikes=0.5):
        return [dict(gp.window_row(window('kbd_taps#1')), head_speed_p95=head, spikes_per_s=spikes),
                dict(gp.window_row(window('kbd_taps#1/taps')), head_speed_p95=head)]

    def test_comparison_before_after_change(self):
        cmp = gp.compare_rows(self.rows(150.0), self.rows(120.0, spikes=None))
        by = {(c['label'], c['metric']): c for c in cmp}
        c = by[('kbd_taps#1', 'head_speed_p95')]
        self.assertEqual((c['before'], c['after'], c['change'], c['pct']), (120.0, 150.0, 30.0, 25.0))
        s = by[('kbd_taps#1', 'spikes_per_s')]
        self.assertEqual((s['before'], s['after'], s['change']), (None, 0.5, None))     # a missing value is not a change from zero
        self.assertEqual(gp.compare_rows(self.rows(1.0), self.rows(1.0), scenarios_only=True)[0]['label'], 'kbd_taps#1')
        self.assertEqual({c['label'] for c in gp.compare_rows(self.rows(1.0), [dict(self.rows(1.0)[0], label='other#1')])}, set())
        text = '\n'.join(gp.format_comparison(cmp))
        self.assertIn('head_speed_p95', text)
        self.assertIn('+25.0 %', text)

    def test_no_limits_configured_means_nothing_is_ever_exceeded(self):
        self.assertEqual(gp.check_limits(self.rows(99999.0), {}), [])
        self.assertEqual(gp.check_limits(self.rows(99999.0), {'other_scenario': {'head_speed_p95': 1}}), [])

    def test_limits_max_min_and_baseline_percent_on_the_scenario_window_only(self):
        lim = {'kbd_taps': {'head_speed_p95': 140, 'chest_speed_p95': {'min': 50}, 'head_change_p99': {'max_vs_baseline_pct': 10}}}
        bad = gp.check_limits(self.rows(150.0), lim, self.rows(150.0))
        kinds = {(b['metric'], b['kind']) for b in bad}
        self.assertEqual(kinds, {('head_speed_p95', 'max'), ('chest_speed_p95', 'min')})        # the baseline is not exceeded: same value
        self.assertTrue(all(b['label'] == 'kbd_taps#1' for b in bad))                           # the step row is not checked
        base = self.rows(150.0)
        base[0]['head_change_p99'] = 500.0
        bad = gp.check_limits(self.rows(150.0), lim, base)
        self.assertIn('head_change_p99', {b['metric'] for b in bad})
        self.assertEqual(gp.check_limits(self.rows(100.0), {'kbd_taps': {'head_speed_p95': 140}}), [])

    def test_limits_are_read_from_scenarios_json_and_none_by_default(self):
        self.assertEqual(gp.load_pose_limits(os.path.join(HERE, 'scenarios.json')), {})          # no limit is invented
        d = tempfile.mkdtemp()
        try:
            p = os.path.join(d, 's.json')
            with open(p, 'w') as f:
                json.dump({'scenarios': {'a': {'pose_limits': {'head_speed_p95': 10}}, 'b': {}}}, f)
            self.assertEqual(gp.load_pose_limits(p), {'a': {'head_speed_p95': 10}})
            self.assertEqual(gp.load_pose_limits(os.path.join(d, 'missing.json')), {})
        finally:
            shutil.rmtree(d)


class HelperReportTests(unittest.TestCase):
    def test_commands_follow_the_reports_command_lines(self):
        cmd, out = gp.helper_command('helper_coverage', 'py', 'cov.py', 'D:/logs', 'D:/logs/utpp-1.log', 'D:/plan.json', 'D:/w')
        self.assertEqual(cmd[:4], ['py', '-I', 'cov.py', 'D:/logs/utpp-1.log'])
        self.assertEqual(cmd[4:], ['--json', out, '--plan', 'D:/plan.json'])
        cmd, out = gp.helper_command('helper_residual', 'py', 'res.py', 'D:/logs', None, None, 'D:/w')
        self.assertEqual(cmd, ['py', '-I', 'res.py', 'D:/logs', '--out-dir', os.path.join('D:/w', 'helper_residual')])
        self.assertTrue(out.endswith('helper_residual.json'))

    def test_verdicts_from_status_or_exit_code(self):
        self.assertEqual(gp.helper_verdict('helper_residual', 1, None), 'FAIL')
        self.assertEqual(gp.helper_verdict('helper_residual', 2, None), 'NOT_VERIFIABLE')
        self.assertEqual(gp.helper_verdict('helper_coverage', 1, {}), 'UNDER_LIMIT')
        self.assertEqual(gp.helper_verdict('helper_residual', 0, [{'status': 'NO_MOD_WRITES'}]), 'NO_MOD_WRITES')
        self.assertEqual(gp.helper_verdict('helper_coverage', 9, None), 'rc 9')

    def test_present_reports_are_started_and_their_verdict_attached(self):
        d = tempfile.mkdtemp()
        orig = gp.helper_script
        try:
            fake = os.path.join(d, 'fake_report.py')
            with open(fake, 'w') as f:
                f.write('import sys\nprint("report ran")\nsys.exit(1)\n')
            logs = os.path.join(d, 'logs')
            os.makedirs(logs)
            pose = os.path.join(logs, 'utpp-pose-2026-11-14-01-43-37.utpppose')
            open(pose, 'wb').close()
            open(os.path.join(logs, 'utpp-2026-11-14-01-42-41.log'), 'w').close()
            gp.helper_script = lambda kind: fake
            job = gp.MeterJob(os.path.join(d, 'w'), [pose], [], helper_reports=True)
            os.makedirs(job.work_dir)
            self.assertEqual(job.start_helper_reports(None), ['helper_coverage', 'helper_residual'])
            out = job.finish(60)
            self.assertEqual(out['helper_reports']['helper_residual']['verdict'], 'FAIL')
            self.assertEqual(out['helper_reports']['helper_coverage']['verdict'], 'UNDER_LIMIT')
            self.assertIn('report ran', out['helper_reports']['helper_coverage']['output_tail'])
            gp.helper_script = lambda kind: None                    # reports not written yet: nothing is called
            self.assertEqual(gp.MeterJob(d, [pose], []).start_helper_reports(None), [])
            self.assertEqual(gp.MeterJob(d, [pose], [], helper_reports=False).start_helper_reports(None), [])
        finally:
            gp.helper_script = orig
            shutil.rmtree(d, ignore_errors=True)


class LoadTests(unittest.TestCase):
    def test_sampler_statistics_and_compiler_count(self):
        vals = iter([10.0, 20.0, 90.0, 80.0])
        procs = iter([[], ['cl.exe'], ['cl.exe', 'clang-cl.exe', 'cl.exe'], []])
        s = gt_load.LoadSampler(cpu_probe=lambda: next(vals), proc_probe=lambda: next(procs), proc_every=1, period=0.01)
        for n in range(4):
            s.sample(n)
        r = s.result()
        self.assertEqual((r['samples'], r['cpu_mean_pct'], r['cpu_max_pct']), (4, 50.0, 90.0))
        self.assertEqual((r['compiler_processes_max'], r['compiler_names']), (3, ['cl.exe', 'clang-cl.exe']))
        self.assertTrue(r['high'])                                                               # a compiler was running
        self.assertIn('WARNING', gt_load.format_load(r))
        self.assertEqual(r['high_cpu_threshold_pct'], gt_load.HIGH_CPU_PERCENT)                  # the threshold is printed with the result

    def test_quiet_machine_has_no_warning(self):
        s = gt_load.LoadSampler(cpu_probe=lambda: 12.0, proc_probe=lambda: [], period=0.01)
        s.sample(0)
        r = s.result()
        self.assertFalse(r['high'])
        self.assertNotIn('WARNING', gt_load.format_load(r))
        self.assertIn('no samples', gt_load.format_load(gt_load.LoadSampler(cpu_probe=lambda: None).result()))

    def test_thread_start_stop(self):
        s = gt_load.LoadSampler(cpu_probe=lambda: 5.0, proc_probe=lambda: [], period=0.01).start()
        import time
        time.sleep(0.15)
        r = s.stop()
        self.assertGreaterEqual(r['samples'], 3)
        self.assertIsNotNone(r['seconds'])

    def test_real_probes_answer_without_raising(self):
        s = gt_load.LoadSampler(period=0.01)
        s.sample(0)
        s.sample(1)
        self.assertIsNotNone(s.result()['method'])

    def test_idle_others_touches_only_other_build_and_test_processes(self):
        class P:
            def __init__(self, pid, name, kids=(), nice=32, fail=False):
                self.pid, self.info, self.kids, self._nice, self.fail = pid, {'pid': pid, 'name': name}, list(kids), nice, fail

            def children(self, recursive=False):
                out = list(self.kids)
                for k in self.kids:
                    out += k.children(True)
                return out

            def nice(self, v=None):
                if self.fail:
                    raise PermissionError('denied')
                if v is None:
                    return self._nice
                self._nice = v

        class Ps:
            IDLE_PRIORITY_CLASS = 64

            def __init__(self, procs):
                self.procs = {p.pid: p for p in procs}

            def Process(self, pid):
                return self.procs[pid]              # KeyError for a process that ended

            def process_iter(self, attrs):
                return list(self.procs.values())

        rec = P(12, 'ffmpeg.exe')                                            # the run's own recorder
        meter = P(13, 'python.exe')                                          # the run's own pose meter worker
        runner = P(10, 'python.exe', kids=[rec, meter])
        game = P(20, 'Cyberpunk2077.exe')
        others = [P(30, 'cl.exe'), P(31, 'Ninja.exe'), P(32, 'utpp_mm_loco_test.exe'), P(33, 'python.exe'), P(34, 'ctest.exe'),
                  P(35, 'link.exe', nice=64), P(36, 'cl.exe', fail=True)]
        bystanders = [P(40, 'explorer.exe'), P(41, 'steam.exe'), P(42, 'chrome.exe')]
        ps = Ps([runner, rec, meter, game] + others + bystanders)
        own = gt_load.process_tree(ps, [10, 20, 999])                        # 999 ended meanwhile
        self.assertEqual(own, {10, 12, 13, 20})
        self.assertEqual(gt_load.idle_others(ps, own), 5)                    # 30-34; 35 is idle already, 36 cannot be changed
        self.assertEqual([p._nice for p in others], [64, 64, 64, 64, 64, 64, 32])
        self.assertTrue(all(p._nice == 32 for p in [runner, rec, meter, game] + bystanders))
        self.assertEqual(gt_load.idle_others(ps, own), 0)                    # nothing left to change
        idler = gt_load.OthersIdler(lambda: [10, 20], period=0.01, ps=Ps([runner, rec, meter, game, P(50, 'cl.exe')]))
        self.assertEqual(idler.tick(), 1)
        idler.start()
        self.assertEqual(idler.stop(), {'available': True, 'ticks': idler.ticks, 'processes_set_idle': 1})
        self.assertEqual(gt_load.OthersIdler(lambda: [], ps=type('NoClass', (), {})()).tick(), 0)   # no priority class: nothing

    def test_typeperf_fallback_parser(self):
        class R:
            stdout = '"(PDH-CSV 4.0)","\\\\PC\\Processor(_Total)\\% Processor Time"\n"11/14/2026 02:00:00.000","37.250000"\n'
        self.assertEqual(gt_load.cpu_percent_typeperf(runner=lambda *a, **k: R()), 37.25)
        self.assertIsNone(gt_load.cpu_percent_typeperf(runner=lambda *a, **k: (_ for _ in ()).throw(OSError('x'))))

        class T:
            stdout = '"cl.exe","1","Console","1","10 K"\n"notepad.exe","2","Console","1","5 K"\n"ninja.exe","3","Console","1","5 K"\n'
        self.assertEqual(gt_load.compilers_tasklist(runner=lambda *a, **k: T()), ['cl.exe', 'ninja.exe'])


class RunFolderTests(unittest.TestCase):
    """A run folder built from the real slice: logs\\ with stamped names, cycle.json, gametest\\plan.json."""

    @classmethod
    def setUpClass(cls):
        cls.root = tempfile.mkdtemp(prefix='gpm_')
        probe = pm.analyze(CORE, LOG, None, feed_file=FEED, helpers=False, tz_hours=TZ)
        cls.start = datetime.strptime(probe['start'], '%Y-%m-%d %H:%M:%S.%f')
        cls.rundir = os.path.join(cls.root, 'run')
        logs = os.path.join(cls.rundir, 'logs')
        os.makedirs(os.path.join(cls.rundir, 'gametest'))
        os.makedirs(logs)
        st = cls.start.strftime('%Y-%m-%d-%H-%M-%S')
        early = (cls.start - timedelta(seconds=30)).strftime('%Y-%m-%d-%H-%M-%S')
        shutil.copy(CORE, os.path.join(logs, f'utpp-pose-{st}.utpppose'))
        shutil.copy(LOG, os.path.join(logs, f'utpp-{early}.log'))
        shutil.copy(FEED, os.path.join(logs, f'utpp-feed-{st}.txt'))
        t = cls.start
        cls.sc = {'name': 'slow_walk_half_stick', 'run': 1, 'start': t + timedelta(seconds=0.3), 'end': t + timedelta(seconds=4.7)}
        with open(os.path.join(cls.rundir, 'cycle.json'), 'w') as f:
            json.dump({'name': 'run', 'scenarios': [dict(cls.sc, start=str(cls.sc['start']), end=str(cls.sc['end']))]}, f)
        runs = [{'scenario': 'slow_walk_half_stick', 'run': 1, 'steps': [
            {'name': 'ramp', 'start': str(t + timedelta(seconds=0.3)), 'end': str(t + timedelta(seconds=2.5))},
            {'name': 'hold', 'start': str(t + timedelta(seconds=2.5)), 'end': str(t + timedelta(seconds=4.7))},
            {'name': 'blip', 'start': str(t + timedelta(seconds=4.7)), 'end': str(t + timedelta(seconds=4.71))}]}]
        with open(os.path.join(cls.rundir, 'gametest', 'plan.json'), 'w') as f:
            json.dump({'runs': runs}, f)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root, ignore_errors=True)

    def cli(self, *args):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rc = gp.main(['run', '--run-dir', self.rundir, '--no-helper-reports', *args])
        return rc, out.getvalue()

    def test_meter_job_runs_the_workers_and_assembles(self):
        import gt_collect
        files = gt_collect.pose_files(os.path.join(self.rundir, 'logs'))
        with open(os.path.join(self.rundir, 'gametest', 'plan.json')) as f:
            plan_runs = json.load(f)['runs']
        windows = gp.build_windows([self.sc], plan_runs)
        work = os.path.join(self.root, 'work')
        job = gp.MeterJob(work, files, windows, helper_reports=False).start()
        out = job.finish(300)
        self.assertEqual(out['errors'], [])
        res = gp.assemble('run', out, windows)
        labels = [r['label'] for r in res['rows']]
        self.assertEqual(labels, ['slow_walk_half_stick#1', 'slow_walk_half_stick#1/ramp', 'slow_walk_half_stick#1/hold'])
        self.assertEqual(res['windows_without_result'], ['slow_walk_half_stick#1/blip'])        # under 3 frames: reported, not silent
        self.assertIn('auto', res)
        self.assertGreater(res['auto']['windows'], 0)
        scen = res['rows'][0]
        self.assertTrue(55 < scen['fps'] < 65, scen['fps'])
        self.assertIsNotNone(scen['head_speed_p95'])
        self.assertTrue(gp.scenario_lines(res)[0].startswith('pose slow_walk_half_stick#1:'))
        gp.write_outputs(os.path.join(self.root, 'out'), res)
        with open(os.path.join(self.root, 'out', 'pose_table.txt'), encoding='utf-8') as f:
            self.assertIn('slow_walk_half_stick#1', f.read())
        with open(os.path.join(self.root, 'out', 'pose_metrics.json'), encoding='utf-8') as f:
            self.assertEqual(len(json.load(f)['rows']), 3)

    def test_worker_error_is_reported_not_raised(self):
        job = gp.MeterJob(os.path.join(self.root, 'work2'), [os.path.join(self.root, 'nothing.utpppose')], [], helper_reports=False).start()
        out = job.finish(120)
        self.assertTrue(out['errors'])
        self.assertIsNone(out['steps'])

    def test_cli_replay_collects_meters_compares_and_checks_limits(self):
        out_dir = os.path.join(self.root, 'replay')
        rc, text = self.cli('--out-dir', out_dir)
        self.assertEqual(rc, 0, text)
        self.assertIn('A. Smoothness', text)
        self.assertTrue(os.path.isfile(os.path.join(out_dir, 'pose_metrics.json')))
        # the second run against the first as a baseline: before / after printed, no limit configured -> exit 0
        out2 = os.path.join(self.root, 'replay2')
        rc, text = self.cli('--out-dir', out2, '--pose-baseline', out_dir)
        self.assertEqual(rc, 0, text)
        self.assertIn('before', text)
        self.assertIn('change', text)
        # a configured limit that is exceeded -> non-zero
        scen = os.path.join(self.root, 'limits.json')
        with open(os.path.join(HERE, 'scenarios.json'), encoding='utf-8') as f:
            data = json.load(f)
        data['scenarios']['slow_walk_half_stick']['pose_limits'] = {'fps': {'max': 1}}
        with open(scen, 'w', encoding='utf-8') as f:
            json.dump(data, f)
        rc, text = self.cli('--out-dir', os.path.join(self.root, 'replay3'), '--scenario-file', scen)
        self.assertEqual(rc, 1, text)
        self.assertIn('LIMIT EXCEEDED slow_walk_half_stick#1 fps', text)

    def test_cli_can_collect_from_a_game_log_folder(self):
        out_dir = os.path.join(self.root, 'collected')
        game = os.path.join(self.root, 'game')
        shutil.copytree(os.path.join(self.rundir, 'logs'), game)
        rc, text = self.cli('--out-dir', out_dir, '--collect-from', game)
        self.assertEqual(rc, 0, text)
        self.assertTrue(os.path.isfile(os.path.join(out_dir, 'logs', os.listdir(game)[0])))
        self.assertIn('copied', text)


if __name__ == '__main__':
    unittest.main()
