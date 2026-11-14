"""Offline tests of the data and measurement side of night.py: recording safety, --diag and the settings ledger, the pose meter hook
and the machine load. No input is sent, no process of the game is touched; the game log folder is a temp dir.

  python Source\\tools\\gametest\\test_night_data.py -v
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_collect as gc  # noqa: E402
import gt_settings as gs  # noqa: E402
import night  # noqa: E402
import test_gt_settings as tgs  # noqa: E402
import test_night as tn  # noqa: E402

night.padd_ping = lambda *a, **k: None


class _Guard:
    def __init__(self, fail=None):
        self.fail = fail
        self.api = None

    def wait(self, s):
        night.time.sleep(s)

    def ensure_focus(self):
        if self.fail:
            raise self.fail

    def check(self):
        pass


class Base(unittest.TestCase):
    def setUp(self):
        self._time = night.time
        night.time = tn.FakeTime()
        self.root = tempfile.mkdtemp(prefix='nd_')
        self.game = os.path.join(self.root, 'game_logs')
        self.run_dir = os.path.join(self.root, 'run')
        os.makedirs(self.game)
        os.makedirs(self.run_dir)
        self.log = os.path.join(self.game, 'utpp-2026-11-14-01-42-41.log')
        with open(self.log, 'wb') as f:
            f.write(b'log\n')
        self.n = self.make()

    def tearDown(self):
        night.time = self._time
        shutil.rmtree(self.root, ignore_errors=True)

    def make(self, *extra):
        args = night.build_parser().parse_args(['--out-root', os.path.join(self.root, 'out'), '--log-dir', self.game, *extra])
        n = night.Night(args, runner=lambda c, **k: type('R', (), {'stdout': '', 'returncode': 0})())
        n.cycle = {'name': 't', 'steps': [], 'start': datetime(2026, 11, 14, 1, 42, 30), 'scenarios': []}
        n.cycle_dir = self.run_dir
        n.utpp_log = self.log
        n.settings.pending_path = os.path.join(self.root, 'out', 'pending_restore.json')
        return n

    def wire_settings(self, n, io):
        """console_verified goes to the fake settings API; returns the list of every command sent through it."""
        sent = []

        def cv(pad, g, cmd, echo, label, timeout=3.0):
            sent.append(cmd)
            if "[night] SETGET" in cmd:
                return io.run(cmd, 'SETGET', label)
            if "[night] SETSET" in cmd:
                return io.run(cmd, 'SETSET', label)
            return f'[night] FINAL ok=true {cmd[:30]}'
        n.console_verified = cv
        n.console = lambda pad, g, cmds, settle=None: sent.extend(cmds) or {}
        return sent

    def put(self, name, data):
        with open(os.path.join(self.game, name), 'wb') as f:
            f.write(data)


class DiagTests(Base):
    def test_keep_touches_nothing(self):
        io = tgs.FakeIo(tgs.START)
        sent = self.wire_settings(self.n, io)
        self.n.apply_diag(_Guard(), {})
        self.assertEqual(sent, [])
        self.assertEqual(self.n.settings.record(), [])

    def test_set_at_arm_and_put_back_before_exit_game_ahead_of_the_final_commands(self):
        n = self.make('--diag', 'until', '--cet-final', "Game.UTPPLogLine('[night] final one')")
        io = tgs.FakeIo(tgs.START, n.settings.pending_path)
        sent = self.wire_settings(n, io)
        info = {}
        n.apply_diag(_Guard(), info)
        self.assertEqual(io.store[gs.DIAG_KEY], 'until')
        self.assertEqual(info['diag']['state'], 'applied')
        self.assertEqual(info['diag']['previous'][0], [gs.DIAG_KEY, 'off'])
        self.assertTrue(os.path.isfile(n.settings.pending_path))
        res = {}
        n.send_final_commands(None, _Guard(), res)
        self.assertEqual(io.store[gs.DIAG_KEY], 'off')                       # back, with the held switches as they were
        self.assertEqual(io.store['mm.trace'], True)
        self.assertEqual(res['restored_settings']['restored'], 1)
        self.assertEqual(sent[-1], "Game.UTPPLogLine('[night] final one')")   # the user's final commands come after
        self.assertEqual(sum("[night] SETSET" in c for c in sent), 2)         # the change and the restore
        self.assertFalse(os.path.exists(n.settings.pending_path))
        self.assertEqual(n.cycle['settings_restore']['restored'], 1)

    def test_diag_that_cannot_be_read_is_not_changed_and_the_run_says_so(self):
        n = self.make('--diag', 'timed')
        io = tgs.FakeIo(tgs.START, lose=['get'])
        self.wire_settings(n, io)
        info = {}
        n.apply_diag(_Guard(), info)
        self.assertEqual(info['diag']['state'], 'not_applied')
        self.assertEqual(io.store[gs.DIAG_KEY], 'off')

    def test_game_gone_before_the_restore_is_recorded_loudly_and_kept_for_the_next_launch(self):
        n = self.make('--diag', 'until')
        io = tgs.FakeIo(tgs.START, n.settings.pending_path)
        self.wire_settings(n, io)
        n.apply_diag(_Guard(), {})
        n.pids = lambda: []
        res = {'cet_sent': False}
        out = n._quit_wait(n.step('quit'), res, night.time.monotonic(), True, True)
        self.assertEqual(out['settings_left'], [gs.DIAG_KEY])
        left = n.cycle['settings_left']
        self.assertTrue(left['diagnostic_logs_left_on'])
        self.assertEqual(left['note'], 'diagnostic logs left ON - restore needed')
        self.assertTrue(os.path.isfile(n.settings.pending_path))
        # THE NEXT LAUNCH of the runner: the pending file is restored before arm sends anything else
        n2 = self.make()
        io2 = tgs.FakeIo({**tgs.START, gs.DIAG_KEY: 'until', 'core.log': True}, n2.settings.pending_path)
        sent = self.wire_settings(n2, io2)
        order = []
        n2.type_console = lambda g, cmds, verify_log=None, fast=False: order.append(('arm', len(sent))) or {c: True for c in cmds}
        n2.wait_calibration = lambda g, log, off, info: True
        n2.trace_flowing = lambda g, seconds=3.0: True
        n2.guard = lambda: _Guard()
        n2.args.night_auto = False
        n2.arm(self.log)
        self.assertEqual(order[0][1], 2)                               # the read and the restore were sent before the arm commands
        self.assertEqual(io2.store[gs.DIAG_KEY], 'off')
        self.assertEqual(n2.cycle['pending_settings_restore']['result'], 'restored')
        self.assertFalse(os.path.exists(n2.settings.pending_path))

    def test_unowned_settings_changes_in_cet_commands_are_listed(self):
        n = self.make('--cet', "GetMod('utpp_core').Set('mm.debug', true)")
        io = tgs.FakeIo(tgs.START)
        self.wire_settings(n, io)
        n.type_console = lambda g, cmds, verify_log=None, fast=False: {c: True for c in cmds}
        n.wait_calibration = lambda g, log, off, info: True
        n.trace_flowing = lambda g, seconds=3.0: True
        n.guard = lambda: _Guard()
        n.args.night_auto = False
        n.arm(self.log)
        self.assertEqual(n.cycle['unowned_settings'], ['mm.debug'])

    def test_leave_restores_through_the_file_channel_only(self):
        n = self.make('--diag', 'until', '--cmd-channel', 'file')
        io = tgs.FakeIo(tgs.START, n.settings.pending_path)
        self.wire_settings(n, io)
        n.apply_diag(_Guard(), {})
        n.restore_settings_leave('run stopped, the game stays open')
        self.assertEqual(io.store[gs.DIAG_KEY], 'off')
        n3 = self.make('--diag', 'until')                                   # console channel: nothing is typed, the file stays
        io3 = tgs.FakeIo(tgs.START, n3.settings.pending_path)
        sent3 = self.wire_settings(n3, io3)
        n3.apply_diag(_Guard(), {})
        before = len(sent3)
        n3.restore_settings_leave('run stopped, the game stays open')
        self.assertEqual(len(sent3), before)
        self.assertTrue(n3.cycle['settings_left']['diagnostic_logs_left_on'])
        self.assertTrue(os.path.isfile(n3.settings.pending_path))


class RecordingSafetyTests(Base):
    def ring(self, minute, data):
        self.put(f'utpp-pose-ring-2026-11-14-01-{minute:02d}-37.utpppose', data)

    def test_between_final_and_after_quit(self):
        n = self.n
        self.ring(43, b'a' * 10)
        self.ring(44, b'b' * 10)
        self.put('utpp-feed-2026-11-14-01-43-37.txt', b'feed')
        n.secure_session_files('between scenarios')
        names = {e['name'] for e in n.cycle['recordings']['files']}
        self.assertEqual(names, {'utpp-pose-ring-2026-11-14-01-43-37.utpppose'})        # the open one waits
        os.remove(os.path.join(self.game, 'utpp-pose-ring-2026-11-14-01-43-37.utpppose'))   # the ring drops it
        n.secure_session_files('scenarios end', final=True)
        rec = n.cycle['recordings']
        self.assertEqual({e['name'] for e in rec['files']}, {'utpp-pose-ring-2026-11-14-01-43-37.utpppose',
                                                             'utpp-pose-ring-2026-11-14-01-44-37.utpppose',
                                                             'utpp-feed-2026-11-14-01-43-37.txt', 'utpp-2026-11-14-01-42-41.log'})
        self.assertEqual(rec['bytes'], 10 + 10 + 4 + 4)
        self.ring(44, b'b' * 10 + b'tail')
        n.secure_session_files('collect', final=True)
        sizes = {e['name']: e['bytes'] for e in n.cycle['recordings']['files']}
        self.assertEqual(sizes['utpp-pose-ring-2026-11-14-01-44-37.utpppose'], 14)
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'logs', 'utpp-pose-ring-2026-11-14-01-43-37.utpppose')))
        with open(os.path.join(self.run_dir, 'cycle.json'), encoding='utf-8') as f:        # the record is saved with the list
            self.assertEqual(len(json.load(f)['recordings']['files']), 4)

    def test_a_previous_session_is_not_collected(self):
        self.put('utpp-pose-ring-2026-11-14-01-40-37.utpppose', b'old')
        self.ring(43, b'new')
        self.n.secure_session_files('x', final=True)
        names = {e['name'] for e in self.n.cycle['recordings']['files']}
        self.assertNotIn('utpp-pose-ring-2026-11-14-01-40-37.utpppose', names)

    def test_the_scenario_phase_secures_the_recording_on_every_way_out(self):
        self.ring(43, b'a' * 5)
        # a failure in the scenario phase (focus lost / abort key): the finally copies before anything else
        n = self.n
        n.guard = lambda: _Guard(fail=night.GuardError('the game lost the focus'))
        with self.assertRaises(night.CycleError):
            n.run_scenarios({'runs': []})
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'logs', 'utpp-pose-ring-2026-11-14-01-43-37.utpppose')))
        self.assertIn('machine_load', n.cycle)                        # the sampler was stopped and recorded
        # the stop file / graceful stop comes through the same finally
        n2 = self.make()
        self.put('utpp-pose-ring-2026-11-14-01-44-37.utpppose', b'b')
        n2.stop_reason = 'STOP file'
        n2.names = ['kbd_taps']
        n2.guard = lambda: _Guard()
        n2.level_camera = lambda *a, **k: None
        n2.ensure_unmounted = lambda *a, **k: False
        night.gt_run.open_pad = lambda *a, **k: (type('P', (), {'wake': lambda s, w: None, 'close': lambda s: None, 'release': lambda s: None})(), False)
        try:
            with self.assertRaises(night.GracefulStop):
                n2.run_scenarios({'runs': []})
        finally:
            night.gt_run.open_pad = tn._no_pad
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'logs', 'utpp-pose-ring-2026-11-14-01-44-37.utpppose')))

    def test_a_listing_problem_never_raises(self):
        self.n.args.log_dir = os.path.join(self.root, 'does_not_exist')
        self.assertIsNotNone(self.n.secure_session_files('x', final=True))
        self.n.utpp_log = None
        self.n.collector = None
        self.assertIsNone(self.n.secure_session_files('x'))


class LoadAndMeterHookTests(Base):
    def test_load_line_is_next_to_the_perf_line_and_warns_on_a_busy_machine(self):
        import gt_load
        s = gt_load.LoadSampler(cpu_probe=lambda: 85.0, proc_probe=lambda: ['cl.exe', 'cl.exe', 'link.exe'], period=0.01)
        s.sample(0)
        s.sample(1)
        res = s.result()
        self.assertTrue(res['high'])
        self.assertEqual(res['compiler_processes_max'], 3)
        self.n.cycle['machine_load'] = res
        lines = []
        self.n.log = lines.append
        self.n.report_load()
        self.assertIn('WARNING', lines[0])
        self.assertIn('cl.exe', lines[0])

    def test_pose_meter_hook_writes_the_outputs_the_summary_and_the_limit_verdict(self):
        import gt_posemeter as gp
        rows = [dict(gp.window_row({'label': 'slow_walk_half_stick#1', 'duration_s': 8.0}), head_speed_p95=10.0)]
        steps = {'ok': True, 'windows': [], 'pose_files': ['a'], 'frames': 5, 'duration_s': 1.0, 'body': 'woman'}

        class FakeJob:
            def __init__(self, work, files, windows, **k):
                self.windows = windows

            def start(self):
                return self

            def finish(self, timeout):
                return {'steps': steps, 'auto': None, 'helper_reports': {}, 'errors': []}
        orig_job, orig_assemble = night.gp.MeterJob, night.gp.assemble
        night.gp.MeterJob = FakeJob
        night.gp.assemble = lambda name, out, windows, rec=None, load=None: {
            'run': name, 'rows': rows, 'errors': [], 'windows_without_result': [], 'helper_reports': {}, 'body': 'woman', 'frames': 5,
            'duration_s': 1.0, 'feed_alignment': 'x'}
        try:
            os.makedirs(os.path.join(self.run_dir, 'logs'))
            self.put('x', b'')
            shutil.copy(os.path.join(HERE, 'testdata', 'pose_metrics_core.utpppose'),
                        os.path.join(self.run_dir, 'logs', 'utpp-pose-2026-11-14-01-43-37.utpppose'))
            base = os.path.join(self.root, 'base')
            os.makedirs(base)
            with open(os.path.join(base, 'pose_metrics.json'), 'w') as f:
                json.dump({'rows': [dict(rows[0], head_speed_p95=8.0)]}, f)
            scen = os.path.join(self.root, 'sc.json')
            with open(os.path.join(HERE, 'scenarios.json'), encoding='utf-8') as f:
                data = json.load(f)
            data['scenarios']['slow_walk_half_stick']['pose_limits'] = {'head_speed_p95': {'max': 9.0}}
            with open(scen, 'w', encoding='utf-8') as f:
                json.dump(data, f)
            n = self.make('--pose-baseline', base, '--scenario-file', scen)
            n.cycle['scenarios'] = [{'name': 'slow_walk_half_stick', 'run': 1, 'start': datetime(2026, 11, 14, 1, 43, 53), 'end': datetime(2026, 11, 14, 1, 44, 1)}]
            n.start_pose_meter({'runs': []}, os.path.join(self.run_dir, 'logs'))
            self.assertIsNotNone(n.meter_job)
            summary = {}
            n.finish_pose_meter(summary)
        finally:
            night.gp.MeterJob, night.gp.assemble = orig_job, orig_assemble
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'pose_metrics.json')))
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'pose_table.txt')))
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'pose_compare.txt')))
        self.assertEqual(summary['pose_meter']['limits_exceeded'][0]['metric'], 'head_speed_p95')
        self.assertEqual(n.cycle['pose_limits_exceeded'][0]['limit'], 9.0)
        self.assertTrue(any(l.startswith('pose slow_walk_half_stick#1') for l in summary['pose_meter']['lines']))

    def test_no_recording_means_no_meter_and_a_clear_line(self):
        os.makedirs(os.path.join(self.run_dir, 'logs'))
        lines = []
        self.n.log = lines.append
        self.n.start_pose_meter({'runs': []}, os.path.join(self.run_dir, 'logs'))
        self.assertIsNone(self.n.meter_job)
        self.assertIn('no pose recording', lines[0])

    def test_arguments(self):
        a = night.build_parser().parse_args([])
        self.assertEqual((a.diag, a.pose_baseline, a.no_pose_meter), ('keep', '', False))
        a = night.build_parser().parse_args(['--diag', 'until', '--pose-baseline', 'nowhere'])
        self.assertTrue(any('no pose_metrics.json' in e for e in night.validate_args(a)))
        import contextlib
        import io
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            night.build_parser().parse_args(['--diag', 'sometimes'])


if __name__ == '__main__':
    unittest.main()
