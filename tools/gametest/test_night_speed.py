"""Offline tests of the speed and mode items of night.py: the recorder stop, the quit limit, the early stop, the post-processing
(--fast, the pool, the clip limit), the arguments of --body / --graphics / --background and the three session command lines.
No input is sent, no game process is touched; every folder is a temp dir.

  python Source\\tools\\gametest\\test_night_speed.py -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time as real_time
import unittest
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_night as gn  # noqa: E402
import gt_session  # noqa: E402
import night  # noqa: E402
import test_night as tn  # noqa: E402
from test_night_data import Base, _Guard  # noqa: E402

# Session command lines the runner is used with directly; the tests parse and validate them.
SESSION_LINES = {
    'keyboard_and_movement_female': ['--set', 'kbd,loco', '--body', 'female', '--diag', 'until', '--cet',
                                     'Game.UTPPSetMMTurnCommit(true)', '--cet-final', 'Game.UTPPSetMMTurnCommit(false)', '--fast',
                                     '--pose-baseline', 'BASELINE_RUN'],
    'fists_enemy_weapons_both_bodies': ['--set', 'melee_fists,melee_fists_target,melee_weapons', '--body', 'both', '--diag', 'until',
                                        '--fast'],
    'background_input_probe': ['--background', '--set', 'bg_probe', '--diag', 'until', '--fast'],
}


class ArgsTests(unittest.TestCase):
    def parse(self, *a):
        return night.finalize_args(night.build_parser().parse_args(['--go', *a]))

    def test_defaults(self):
        a = night.build_parser().parse_args([])
        self.assertEqual((a.quit_timeout, a.recorder_grace), (60.0, 8.0))
        self.assertEqual((a.post_workers, a.clips_per_cause, a.fast, a.early_stop), (0, 6, False, 2))
        self.assertEqual((a.body, a.graphics, a.min_idle, a.cmd_channel_explicit), ('keep', 'keep', None, False))

    def test_background_without_a_channel_uses_auto_and_needs_no_idle_time(self):
        a = self.parse('--background')
        self.assertEqual(a.cmd_channel, 'auto')
        self.assertEqual(gt_session.effective_min_idle(a), 0.0)
        self.assertEqual(night.validate_args(a), [])
        self.assertEqual(self.parse('--background', '--cmd-channel', 'file').cmd_channel, 'file')
        c = self.parse('--background', '--cmd-channel', 'console')
        self.assertEqual(c.cmd_channel, 'console')
        self.assertTrue(any('--background needs --cmd-channel file or auto' in e for e in night.validate_args(c)))
        self.assertEqual(self.parse().cmd_channel, 'console')                      # no background: nothing changes
        self.assertEqual(gt_session.effective_min_idle(self.parse()), 120.0)

    def test_background_run_is_not_stopped_by_a_busy_user(self):
        out = tempfile.mkdtemp(prefix='np_')
        self.addCleanup(lambda: __import__('shutil').rmtree(out, ignore_errors=True))
        n = night.Night(self.parse('--background', '--out-root', out))
        with mock.patch.object(night, 'idle_seconds', return_value=0.5):
            n.check_user_idle()                                                    # user input half a second ago: no UserActive
        n2 = night.Night(self.parse('--out-root', out))
        with mock.patch.object(night, 'idle_seconds', return_value=0.5), self.assertRaises(night.UserActive):
            n2.check_user_idle()

    def test_negative_values_are_refused(self):
        for flag in ('--clips-per-cause', '--early-stop', '--post-workers'):
            self.assertTrue(night.validate_args(self.parse(flag, '-1')), flag)

    def test_session_command_lines_parse_and_validate(self):
        base = tempfile.mkdtemp(prefix='bl_')
        self.addCleanup(lambda: __import__('shutil').rmtree(base, ignore_errors=True))
        with open(os.path.join(base, 'pose_metrics.json'), 'w') as f:
            json.dump({'body': 'woman', 'rows': []}, f)
        for name, argv in SESSION_LINES.items():
            a = self.parse(*[base if x == 'BASELINE_RUN' else x for x in argv])
            errs = gn.validate_sets(gn.resolve_sets(a.set), night.load_scenarios(a.scenario_file))
            self.assertEqual(errs, [], name)
            self.assertEqual(a.diag, 'until', name)
            self.assertTrue(a.fast, name)

    def test_the_three_sets_cover_what_the_lines_promise(self):
        kbd_loco = gn.resolve_sets('kbd,loco')
        self.assertEqual(kbd_loco[:6], gn.SETS['kbd'])
        self.assertEqual(kbd_loco[6:], gn.SETS['loco'])
        names = gn.resolve_sets('melee_fists,melee_fists_target,melee_weapons')
        self.assertEqual(names, ['melee_fists', 'melee_fists_target', 'melee_weapons'])
        self.assertEqual(gn.MACROS['melee_weapons'] if 'melee_weapons' in gn.MACROS else None,
                         list(night.gw.PARTS))
        self.assertEqual(night.gw.WEAPON_SETS['melee_weapons'], ['knife', 'katana', 'blunt', 'baton'])    # the four pack weapons
        self.assertEqual(gn.resolve_sets('bg_probe'), ['slow_walk_half_stick', 'run_arc_left', 'run_arc_right'])


class FakeProc:
    def __init__(self, ignore_q=False, exited=None):
        self.ignore_q, self.rc = ignore_q, exited
        self.returncode = exited
        self.stdin = SimpleNamespace(write=lambda b: setattr(self, 'q', b), flush=lambda: None)
        self.killed = self.terminated = False
        self.waits = []

    def poll(self):
        return self.rc

    def wait(self, timeout=None):
        self.waits.append(timeout)
        if self.ignore_q and not self.terminated:
            raise subprocess.TimeoutExpired('ffmpeg', timeout)
        self.rc = self.returncode = 0
        return 0

    def terminate(self):
        self.terminated = True

    def kill(self):
        self.killed = True


class RecorderTests(Base):
    def rec(self, proc, **kw):
        path = os.path.join(self.run_dir, 'rec.mp4')
        with open(path, 'wb') as f:
            f.write(b'0' * 64)
        errp = path[:-4] + '.ffmpeg.log'
        open(errp, 'w').close()
        base = datetime(2026, 11, 14, 1, 43, 38)
        errf = open(errp, 'a')
        self.addCleanup(errf.close)
        return dict({'path': path, 'errp': errp, 'errf': errf, 'proc': proc, 'markers': [], 'spawn_wall': base,
                     'json': path[:-4] + '.json', 'cmd': ['ffmpeg'], 'expected_wh': (1920, 1080)}, **kw)

    def test_q_is_taken_the_stop_takes_its_time_and_nothing_else(self):
        n = self.make()
        rec = self.rec(FakeProc())
        n.stop_recorder_process(rec)
        self.assertEqual((rec['stop'], rec['stopped']), ('q', True))
        self.assertEqual(rec['proc'].q, b'q')
        self.assertEqual(rec['proc'].waits, [8.0])                                  # --recorder-grace
        self.assertFalse(rec['proc'].terminated)
        self.assertIn('stop_seconds', rec)

    def test_a_recorder_that_ignores_q_is_ended_after_the_grace_only(self):
        n = self.make('--recorder-grace', '3')
        rec = self.rec(FakeProc(ignore_q=True))
        n.stop_recorder_process(rec)
        self.assertEqual(rec['stop'], 'terminate')
        self.assertEqual(rec['proc'].waits[0], 3.0)                                 # the grace, not a minute
        self.assertTrue(rec['proc'].terminated)
        self.assertFalse(rec['proc'].killed)                                        # it ended after terminate

    def test_an_already_ended_recorder_is_recorded_as_such(self):
        n = self.make()
        rec = self.rec(FakeProc(exited=1))
        n.stop_recorder_process(rec)
        self.assertIn('had exited (rc 1)', rec['stop'])
        self.assertEqual(rec['proc'].waits, [])

    def test_the_stop_without_check_leaves_the_decode_for_after_the_quit(self):
        n = self.make()
        calls = []

        def runner(cmd, **kw):
            calls.append(cmd)
            return SimpleNamespace(returncode=0, stdout='', stderr='  Duration: 00:07:10.33, start: 0.0\n'
                                   '  Stream #0:0: Video: h264, yuv420p, 1920x1080, 30 fps\nframe= 1 YAVG=100.0\n')
        n.runner = runner
        rec = self.rec(FakeProc(), stop='q', stop_wall=datetime(2026, 11, 14, 1, 50, 49), stopped=True)
        info = n.check_recording(rec, deep=False)
        self.assertFalse(any('-map' in c for c in calls), calls)                    # no full decode now
        self.assertIsNone(info['decode_ok'])
        self.assertEqual((rec['duration'], rec['duration_source']), (430.33, 'container'))
        self.assertIn('clock', rec)                                                 # the clock exists before the sheets need it
        self.assertTrue(rec['finalized'])
        n.start_deep_check(rec)
        n.finish_deep_check(rec)
        self.assertTrue(any('-map' in c for c in calls))
        self.assertIn('deep', rec)

    def test_the_decode_result_is_merged_and_an_estimated_length_is_replaced(self):
        n = self.make()
        deep_err = 'frame=100 time=00:00:12.50 bitrate=1kbits/s\n'
        n.runner = lambda cmd, **kw: SimpleNamespace(returncode=0, stdout='', stderr=deep_err)
        rec = self.rec(FakeProc(), stop='terminate', stop_wall=datetime(2026, 11, 14, 1, 43, 58), stopped=True, finalized=True,
                       duration=20.0, duration_source='wall time', input_start=None,
                       check={'problems': [], 'decode_ok': None})
        n.start_deep_check(rec)
        n.finish_deep_check(rec)
        self.assertEqual((rec['duration'], rec['duration_source']), (12.5, 'decoded'))
        self.assertEqual(rec['clock']['duration_s'], 12.5)
        self.assertTrue(rec['check']['decode_ok'])
        # a file that does not decode is a recording problem
        n2 = self.make()
        n2.runner = lambda cmd, **kw: SimpleNamespace(returncode=1, stdout='', stderr='Invalid data found')
        rec2 = self.rec(FakeProc(), stop='q', finalized=True, duration=5.0, duration_source='container', check={'problems': []})
        n2.start_deep_check(rec2)
        n2.finish_deep_check(rec2)
        self.assertFalse(rec2['check']['decode_ok'])
        self.assertTrue(any('decode failed' in p for p in rec2['check']['problems']))

    def test_estimated_length_comes_from_the_progress_stream_or_the_wall_time(self):
        rec = {'progress': SimpleNamespace(samples=[(1.0, 2.0), (2.0, 7.5)]), 'stop_wall': datetime(2026, 1, 1, 0, 0, 10),
               'spawn_wall': datetime(2026, 1, 1, 0, 0, 0)}
        self.assertEqual(night.Night.estimate_duration(rec), (7.5, 'progress'))
        self.assertEqual(night.Night.estimate_duration(dict(rec, progress=None)), (10.0, 'wall time'))
        self.assertEqual(night.Night.estimate_duration({}), (None, None))


class QuitLimitTests(Base):
    def test_a_game_that_needs_35_seconds_after_the_echo_is_not_killed(self):
        n = self.make()
        runner_calls = []
        n.runner = lambda cmd, **kw: runner_calls.append(cmd) or SimpleNamespace(stdout='', returncode=0)
        t0 = night.time.monotonic()
        n.pids = lambda: [1] if night.time.monotonic() - t0 < 35.0 else []
        n.settings.pending_path = None
        st = n.step('quit')
        res = n._quit_wait(st, {'path': 'x', 'cet_sent': True}, t0, False, False)
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertFalse([c for c in runner_calls if c and c[0] == 'taskkill'], runner_calls)
        self.assertGreaterEqual(res['seconds'], 35.0)

    def test_without_the_echo_the_kill_is_immediate(self):
        n = self.make()
        runner_calls = []
        n.runner = lambda cmd, **kw: runner_calls.append(cmd) or SimpleNamespace(stdout='', returncode=0)
        t0 = night.time.monotonic()
        state = {'alive': True}
        n.pids = lambda: [1] if state['alive'] else []
        n.runner = lambda cmd, **kw: (runner_calls.append(cmd), state.update(alive=False)) and SimpleNamespace(stdout='', returncode=0)
        st = n.step('quit')
        n._quit_wait(st, {'path': 'x', 'cet_sent': False}, t0, False, False)
        self.assertEqual(runner_calls[0], ['taskkill', '/IM', 'Cyberpunk2077.exe'])


class EarlyStopTests(Base):
    def prepare(self, names, causes, *flags):
        n = self.make('--repeat', '1', *flags)
        n.names = names
        n.guard = lambda: _Guard()
        n.level_camera = lambda *a, **k: None
        n.ensure_unmounted = lambda *a, **k: False
        n.recover = lambda *a, **k: {}
        n.played = []
        queue = list(causes)

        def play_walking(pad, guard, plan, name, key_crouch=False, clear=True):
            n.played.append(name)
            return {'res': {'steps': []}, 'cfails': [], 'off': 0, 'start': datetime.now(), 'discarded': None}
        n.play_walking = play_walking
        n.live_invalid_cause = lambda res, off: queue.pop(0) if queue else None
        n.budget = night.gb.Budget(clock=lambda: night.time.monotonic())             # the tests run on the fake clock
        pad = type('P', (), {'wake': lambda s, w: None, 'close': lambda s: None, 'release': lambda s: None})()
        self._open = night.gt_run.open_pad
        night.gt_run.open_pad = lambda *a, **k: (pad, False)
        self.addCleanup(lambda: setattr(night.gt_run, 'open_pad', self._open))
        return n

    def test_two_walking_scenarios_invalid_for_one_cause_end_the_cycle(self):
        n = self.prepare(['kbd_switch_90', 'kbd_switch_180', 'kbd_diagonals', 'kbd_strafe_weave'], ['V stuck', 'V stuck'])
        with self.assertRaises(night.EarlyStop) as cm:
            n.run_scenarios({'runs': []})
        self.assertEqual(n.played, ['kbd_switch_90', 'kbd_switch_180'])              # the rest is not played
        self.assertIn('first 2 walking scenarios', str(cm.exception))
        self.assertIn('V stuck', str(cm.exception))
        skipped = n.cycle['skipped_scenarios']
        self.assertEqual([s['name'] for s in skipped], ['kbd_diagonals', 'kbd_strafe_weave'])
        self.assertTrue(all(s['reason'].count('\n') == 0 for s in skipped))
        self.assertEqual(len(n.cycle['scenarios']), 2)                               # both played ones are in the record
        with open(os.path.join(self.run_dir, 'night.log'), encoding='utf-8') as f:
            self.assertIn('[early-stop]', f.read())

    def test_a_valid_first_scenario_plays_everything(self):
        n = self.prepare(['kbd_switch_90', 'kbd_switch_180', 'kbd_diagonals'], [None, 'V stuck', 'V stuck'])
        n.run_scenarios({'runs': []})
        self.assertEqual(len(n.played), 3)
        self.assertFalse(n.early.watching)

    def test_off_with_zero(self):
        n = self.prepare(['kbd_switch_90', 'kbd_switch_180', 'kbd_diagonals'], ['x', 'x', 'x'], '--early-stop', '0')
        n.run_scenarios({'runs': []})
        self.assertEqual(len(n.played), 3)

    def test_the_budget_has_a_row_per_scenario(self):
        n = self.prepare(['kbd_switch_90', 'kbd_switch_180'], [None])
        n.run_scenarios({'runs': []})
        self.assertEqual([r['scenario'] for r in n.budget.rows()], ['kbd_switch_90#1', 'kbd_switch_180#1'])
        self.assertGreater(n.budget.rows()[0]['wait'], 0.0)                         # the gap before every scenario


class LiveCauseTests(Base):
    def run_with(self, n, pre_ok=True, metrics=None):
        steps = [{'name': f's{i}', 'start': datetime(2026, 11, 14, 1, 0, i), 'end': datetime(2026, 11, 14, 1, 0, i + 1), 'metrics': True}
                 for i in range(2)]
        ms = list(metrics or [])
        with mock.patch.object(night.gt_log, 'parse_text', return_value=[]), \
                mock.patch.object(night.gt_log, 'motion_precondition', return_value={'ok': pre_ok}), \
                mock.patch.object(night.gt_log, 'compute_metrics', side_effect=lambda lines, a, b: ms.pop(0)):
            return n.live_invalid_cause({'steps': steps}, 0)

    BLOCKED = {'pushed_ticks': 4, 'pushed_speed_max': 0.05, 'loco_fraction': 1.0}
    MOVING = {'pushed_ticks': 4, 'pushed_speed_max': 1.4, 'loco_fraction': 1.0}

    def test_no_motion_lines(self):
        self.assertIn('no "UTPP MOTION" lines', self.run_with(self.make(), pre_ok=False))

    def test_blocked_on_every_pushed_step(self):
        n = self.make()
        n.log_tail = lambda *a, **k: ''
        self.assertIn('V does not move', self.run_with(n, metrics=[self.BLOCKED, self.BLOCKED]))
        self.assertIsNone(self.run_with(n, metrics=[self.BLOCKED, self.MOVING]))     # one step moved: valid
        self.assertIsNone(self.run_with(n, metrics=[self.MOVING, self.MOVING]))

    def test_mounted_is_named(self):
        n = self.make()
        n.log_tail = lambda *a, **k: ''
        with mock.patch.object(night.gn, 'mounted_state', return_value=True):
            self.assertIn('(V is in a vehicle)', self.run_with(n, metrics=[self.BLOCKED, self.BLOCKED]))

    def test_no_steps_or_an_error_is_no_verdict(self):
        n = self.make()
        self.assertIsNone(n.live_invalid_cause({'steps': []}, 0))
        n.log_tail = lambda *a, **k: ''
        with mock.patch.object(night.gt_log, 'parse_text', side_effect=ValueError('x')):
            self.assertIsNone(n.live_invalid_cause({'steps': [{'start': datetime.now(), 'end': datetime.now(), 'metrics': True}]}, 0))


class CollectPostTests(Base):
    """collect() with a recording: --fast makes no sheets and no clips and writes the pending list; the normal path goes through the pool."""

    def setUp(self):
        super().setUp()
        self.t0 = datetime(2026, 11, 14, 1, 43, 40)

    def n_with(self, *flags):
        patch = mock.patch.object(night, 'CET_DIR', os.path.join(self.root, 'cet'))
        patch.start()
        self.addCleanup(patch.stop)
        n = self.make('--no-media-move', *flags)
        n.cycle['start'] = datetime(2026, 11, 14, 1, 42, 30)
        n.cycle['scenarios'] = [{'name': 'kbd_taps', 'run': 1, 'start': self.t0 + timedelta(seconds=10), 'end': self.t0 + timedelta(seconds=18),
                                 'verdict': 'TIMEOUT', 'fails': ['scenario exceeded 180 s']}]
        video = os.path.join(self.run_dir, 'rec.mp4')
        open(video, 'wb').close()
        rec = {'path': video, 'mode': 'window', 'stop': 'q', 'finalized': True, 'stopped': True, 'duration': 60.0,
               'duration_source': 'container', 'check': {'problems': [], 'decode_ok': True}, 'markers': [],
               'clock': {'video_start': self.t0, 'video_end': self.t0 + timedelta(seconds=60), 'duration_s': 60.0, 'clock_offset_s': 0.0,
                         'start_method': 'x'}}
        n.start_pose_meter = lambda plan, ld: None
        n.collector = None
        return n, rec

    def test_fast_makes_no_sheets_and_no_clips_and_lists_them(self):
        n, rec = self.n_with('--fast')
        with mock.patch.object(night.gt_post, 'run_sheets', side_effect=AssertionError('no sheets with --fast')), \
                mock.patch.object(night.gt_post, 'cut_pool', side_effect=AssertionError('no clips with --fast')):
            summary = n.collect(rec, {'runs': []})
        pend = summary['pending_media']
        self.assertEqual([s['sheet'] for s in pend['sheets']], ['full', 'kbd_taps_r1_start', 'kbd_taps_r1_end'])
        self.assertEqual(pend['sheets'][1]['video_s'], [7.0, 13.0])                  # video seconds around the scenario start
        self.assertEqual([c['name'] for c in pend['clips']], ['kbd_taps_r1'])
        self.assertEqual(pend['clips'][0]['video_s'], [8.0, 20.0])
        self.assertIn('gt_post.py" sheets', pend['commands']['sheets'])
        self.assertIn('failure_clips.py" ', pend['commands']['clips'])
        self.assertIn('--per-cause 0', pend['commands']['clips'])
        self.assertIn('not made', summary['sheets'])
        with open(os.path.join(self.run_dir, 'summary.json'), encoding='utf-8') as f:
            self.assertIn('pending_media', json.load(f))
        self.assertTrue(os.path.isfile(rec['path']))                                  # the recording itself is untouched

    def test_normal_path_uses_the_pool_for_sheets_and_clips_with_the_limit(self):
        n, rec = self.n_with('--clips-per-cause', '4')
        seen = {}

        def run_sheets(jobs, sd, workers=0, runner=None):
            seen['sheet_jobs'] = jobs
            return {j['sheet']: 'ok' for j in jobs}, {'workers': 5, 'seconds': 1.0, 'jobs': len(jobs), 'busy_s': 3.0, 'slowest': []}

        def cut_pool(video, jobs, ffmpeg, small, workers=0, scratch='', runner=None):
            seen['clips'] = [p for _, p in jobs]
            return {p: None for _, p in jobs}
        with mock.patch.object(night.gt_post, 'run_sheets', run_sheets), mock.patch.object(night.gt_post, 'cut_pool', cut_pool), \
                mock.patch.object(night.gt_post, 'pool_size', return_value=5):
            summary = n.collect(rec, {'runs': []})
        self.assertEqual({j['sheet'] for j in seen['sheet_jobs']}, {'full', 'kbd_taps_r1_start', 'kbd_taps_r1_end'})
        self.assertEqual(summary['sheets']['full'], 'ok')
        self.assertEqual(summary['sheets_timing']['workers'], 5)
        self.assertEqual(len(summary['failure_clips']), 1)
        self.assertEqual(len(seen['clips']), 1)
        self.assertNotIn('pending_media', summary)
        self.assertIn('sheets (wait)', n.budget.phases)
        self.assertIn('failure clips', n.budget.phases)

    def test_no_sheets_still_gives_the_clips_their_clock(self):
        n, rec = self.n_with('--no-sheets')
        with mock.patch.object(night.gt_post, 'run_sheets', side_effect=AssertionError), \
                mock.patch.object(night.gt_post, 'cut_pool', lambda v, jobs, *a, **k: {p: None for _, p in jobs}):
            summary = n.collect(rec, {'runs': []})
        self.assertEqual(summary['sheets'], {})
        self.assertEqual(len(summary['failure_clips']), 1)

    def test_budget_table_is_written_at_the_end_of_a_cycle(self):
        n, rec = self.n_with('--fast')
        n.cycle['steps'] = [{'name': 'scenarios', 'start': self.t0, 'end': self.t0 + timedelta(seconds=60), 'status': 'ok'}]
        n.budget.begin('kbd_taps#1')
        n.budget.end(8.0)
        n.report_budget()
        with open(os.path.join(self.run_dir, 'time_budget.txt'), encoding='utf-8') as f:
            text = f.read()
        self.assertIn('kbd_taps#1', text)
        self.assertIn('phases (s): scenarios 60.0', text)
        self.assertIn('time_budget', n.cycle)
        with open(os.path.join(self.run_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump({'cycle': 't'}, f)
        n.report_budget()
        with open(os.path.join(self.run_dir, 'summary.json'), encoding='utf-8') as f:
            self.assertEqual(json.load(f)['time_budget']['phases']['scenarios'], 60.0)


class MainBodyTests(unittest.TestCase):
    """night.main with --body: refuse a running game, select and restore around the cycles, one summary for both bodies."""

    def run_main(self, argv, running=(), results=None):
        d = tempfile.mkdtemp(prefix='mb_')
        self.addCleanup(lambda: __import__('shutil').rmtree(d, ignore_errors=True))
        out_root = os.path.join(d, 'night')
        events = []
        saves = os.path.join(d, 'saves')
        for name in ('ManualSave-106', 'ManualSave-115'):
            os.makedirs(os.path.join(saves, name))
            with open(os.path.join(saves, name, 'sav.dat'), 'wb') as f:
                f.write(b'x')
        real_session = gt_session.SaveSession

        def session(out_root_, **kw):
            return real_session(out_root_, saves_dir=saves, state_file=os.path.join(d, 'state.json'), backup_dir=os.path.join(d, 'bk'),
                                running=lambda: [], out=lambda m: None)

        def run_cycle(self_, n):
            events.append(('cycle', self_.body, gt_session.select_save.newest_ns(
                gt_session.select_save.scan_saves(saves)[gt_session.BODY_SAVES[self_.body]]) if self_.body else None))
            self_.cycle = {'name': f'c{len(events)}', 'result': (results or {}).get(self_.body, 'ok')}
            return self_.cycle
        with mock.patch.object(night, 'IS_WINDOWS', True), mock.patch.object(gt_session, 'SaveSession', session), \
                mock.patch.object(night.Night, 'run_cycle', run_cycle), mock.patch.object(night, 'keep_awake'), \
                mock.patch.object(night.Night, 'pids', lambda self_: list(running)), \
                mock.patch.object(night, 'write_body_summary', lambda a, r: events.append(('summary', [x['body'] for x in r])) or []), \
                mock.patch('gt_win32.Win32Api', lambda: SimpleNamespace()), mock.patch.object(night, 'install_signal_handlers'):
            rc = night.main(['--go', '--out-root', out_root, '--log-dir', d, *argv])
        return rc, events, saves

    def test_both_bodies_are_selected_one_after_the_other_and_restored(self):
        rc, events, saves = self.run_main(['--body', 'both', '--set', 'kbd_taps'])
        self.assertEqual(rc, 0)
        cycles = [e for e in events if e[0] == 'cycle']
        self.assertEqual([c[1] for c in cycles], ['female', 'male'])
        self.assertEqual(events[-1], ('summary', ['female', 'male']))
        # the save of the selected body was the newest while its cycle ran; afterwards the state is restored
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(saves), 'state.json')))

    def test_the_results_file_keeps_what_the_saves_went_through(self):
        rc, events, saves = self.run_main(['--body', 'both', '--set', 'kbd_taps'])
        out_root = os.path.join(os.path.dirname(saves), 'night')
        files = sorted(f for f in os.listdir(out_root) if f.startswith('night_') and f.endswith('.json'))
        with open(os.path.join(out_root, files[-1]), encoding='utf-8') as f:
            results = json.load(f)
        self.assertEqual([r['body'] for r in results], ['female', 'male'])
        for r, save in zip(results, ('ManualSave-115', 'ManualSave-106')):
            self.assertEqual(r['saves']['select']['save'], save)
            self.assertEqual((r['saves']['restore']['code'], r['saves']['restore']['restored']), (0, True))

    def test_a_running_game_refuses_the_start_before_anything_is_selected(self):
        rc, events, saves = self.run_main(['--body', 'male', '--set', 'kbd_taps'], running=[77])
        self.assertEqual(rc, 4)
        self.assertEqual(events, [])
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(saves), 'state.json')))

    def test_a_failed_cycle_still_restores_and_gives_exit_code_1(self):
        rc, events, saves = self.run_main(['--body', 'both', '--set', 'kbd_taps'], results={'female': 'failed: early stop: x'})
        self.assertEqual(rc, 1)
        self.assertEqual([e[1] for e in events if e[0] == 'cycle'], ['female', 'male'])
        self.assertFalse(os.path.exists(os.path.join(os.path.dirname(saves), 'state.json')))

    def test_an_exception_in_a_cycle_restores_on_the_way_out(self):
        d = tempfile.mkdtemp(prefix='mb_')
        self.addCleanup(lambda: __import__('shutil').rmtree(d, ignore_errors=True))
        saves = os.path.join(d, 'saves')
        os.makedirs(os.path.join(saves, 'ManualSave-115'))
        with open(os.path.join(saves, 'ManualSave-115', 'sav.dat'), 'wb') as f:
            f.write(b'x')
        state = os.path.join(d, 'state.json')
        real_session = gt_session.SaveSession
        session = lambda root, **kw: real_session(root, saves_dir=saves, state_file=state, backup_dir=os.path.join(d, 'bk'),  # noqa: E731
                                                  running=lambda: [], out=lambda m: None)

        def boom(self_, n):
            self.assertTrue(os.path.exists(state))                                  # selected while the cycle runs
            raise RuntimeError('boom')
        with mock.patch.object(night, 'IS_WINDOWS', True), mock.patch.object(gt_session, 'SaveSession', session), \
                mock.patch.object(night.Night, 'run_cycle', boom), mock.patch.object(night, 'keep_awake'), \
                mock.patch.object(night.Night, 'pids', lambda self_: []), mock.patch('gt_win32.Win32Api', lambda: SimpleNamespace()), \
                mock.patch.object(night, 'install_signal_handlers'):
            with self.assertRaises(RuntimeError):
                night.main(['--go', '--out-root', os.path.join(d, 'night'), '--log-dir', d, '--body', 'female', '--set', 'kbd_taps'])
        self.assertFalse(os.path.exists(state))                                    # restored although the cycle blew up


class PoseBaselineBodyTests(Base):
    """--pose-baseline is used only against the body it was recorded on (a run of both bodies compares one of them)."""

    def finish(self, base_body, run_body):
        import gt_posemeter as gp
        base = os.path.join(self.root, 'base')
        os.makedirs(base, exist_ok=True)
        row = gp.window_row({'label': 'slow_walk_half_stick#1', 'duration_s': 8.0})
        with open(os.path.join(base, 'pose_metrics.json'), 'w') as f:
            json.dump(dict({'rows': [dict(row, head_speed_p95=8.0)]}, **({'body': base_body} if base_body else {})), f)
        n = self.make('--pose-baseline', base)
        lines = []
        n.log = lines.append
        n.meter_job = SimpleNamespace(windows=[], finish=lambda timeout: {'steps': None, 'auto': None, 'helper_reports': {}, 'errors': []})
        res = {'run': 't', 'rows': [dict(row, head_speed_p95=10.0)], 'errors': [], 'windows_without_result': [], 'helper_reports': {},
               'body': run_body, 'frames': 5, 'duration_s': 1.0, 'feed_alignment': 'x'}
        summary = {}
        with mock.patch.object(night.gp, 'assemble', lambda *a, **k: res):
            n.finish_pose_meter(summary)
        return summary['pose_meter'], lines

    def test_same_body_is_compared(self):
        rec, lines = self.finish('woman', 'woman')
        self.assertNotIn('baseline_skipped', rec)
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'pose_compare.txt')))

    def test_other_body_is_skipped_with_a_line(self):
        rec, lines = self.finish('man', 'woman')
        self.assertIn('baseline recorded on man, this run is woman', rec['baseline_skipped'])
        self.assertFalse(os.path.exists(os.path.join(self.run_dir, 'pose_compare.txt')))
        self.assertTrue(any('not used' in l for l in lines))

    def test_a_baseline_without_a_recorded_body_is_compared_as_before(self):
        rec, _ = self.finish(None, 'woman')
        self.assertNotIn('baseline_skipped', rec)
        self.assertTrue(os.path.isfile(os.path.join(self.run_dir, 'pose_compare.txt')))


class BodyCheckTests(PoseBaselineBodyTests):
    """The requested body is checked against the body the pose recording shows (report only)."""

    def meter(self, requested, run_body):
        import gt_posemeter as gp
        n = self.make()
        n.body = requested
        lines = []
        n.log = lines.append
        row = gp.window_row({'label': 'slow_walk_half_stick#1', 'duration_s': 8.0})
        n.meter_job = SimpleNamespace(windows=[], finish=lambda timeout: {'steps': None, 'auto': None, 'helper_reports': {}, 'errors': []})
        res = {'run': 't', 'rows': [row], 'errors': [], 'windows_without_result': [], 'helper_reports': {}, 'body': run_body,
               'frames': 5, 'duration_s': 1.0, 'feed_alignment': 'x'}
        summary = {}
        with mock.patch.object(night.gp, 'assemble', lambda *a, **k: res):
            n.finish_pose_meter(summary)
        return n, summary, lines

    def test_a_wrong_body_is_reported_loudly(self):
        n, summary, lines = self.meter('male', 'woman')
        self.assertIn('asked for the man body but the pose recording is of the woman body', summary['body_mismatch'])
        self.assertEqual(n.cycle['body_mismatch'], summary['body_mismatch'])
        self.assertEqual(summary['pose_meter']['body_mismatch'], summary['body_mismatch'])
        self.assertTrue(any('BODY MISMATCH' in l for l in lines))

    def test_the_right_body_or_no_request_is_quiet(self):
        for requested, body in (('female', 'woman'), ('male', 'man'), (None, 'woman')):
            n, summary, lines = self.meter(requested, body)
            self.assertNotIn('body_mismatch', summary, (requested, body))
            self.assertFalse(any('MISMATCH' in l for l in lines))

    def test_an_unnamed_body_cannot_be_checked_and_says_so(self):
        n, summary, lines = self.meter('female', None)
        self.assertNotIn('body_mismatch', summary)
        self.assertTrue(any('could not be checked' in l for l in lines))

    def test_the_summary_of_both_bodies_shows_a_mismatch_and_the_save_results(self):
        import gt_session as gsn
        digests = {'female': {'run': 'r1', 'result': 'ok', 'pose_body': 'woman', 'scenarios': {}, 'perf': None, 'skipped': None,
                              'body_mismatch': None, 'saves': {'select': {'save': 'ManualSave-115'},
                                                               'restore': {'code': 0, 'restored': True, 'lines': []}}},
                   'male': {'run': 'r2', 'result': 'ok', 'pose_body': 'woman', 'scenarios': {}, 'perf': None, 'skipped': None,
                            'body_mismatch': 'x', 'saves': {'select': {'save': 'ManualSave-106'},
                                                            'restore': {'code': 3, 'restored': True,
                                                                        'lines': ['ManualSave-106  changed by the game since it was selected']}}}}
        text = chr(10).join(gsn.side_by_side_lines(digests))
        self.assertIn('male: !!! BODY MISMATCH', text)
        self.assertNotIn('female: !!! BODY MISMATCH', text)
        self.assertIn('female: save ManualSave-115 selected; restore code 0, restored=True', text)
        self.assertIn('male: save ManualSave-106 selected; restore code 3', text)
        self.assertIn('!!! ManualSave-106  changed by the game since it was selected', text)


class GraphicsHookTests(Base):
    def test_low_is_applied_before_the_launch_and_only_logged(self):
        n = self.make('--graphics', 'low')
        n.graphics_settings, n.graphics_state_dir = os.path.join(self.root, 'UserSettings.json'), os.path.join(self.root, 'gstate')
        with open(n.graphics_settings, 'w', encoding='utf-8') as f:
            json.dump({'data': [{'group_name': '/graphics/basic', 'options': [{'name': 'FilmGrain', 'value': True}]}]}, f)
        with mock.patch.object(gt_session.graphics_profile, 'game_running', lambda: False):
            n.apply_graphics()
        self.assertEqual(n.cycle['graphics']['code'], 0)
        self.assertIn('graphics profile: low', n.cycle['graphics']['status'])
        with open(n.graphics_settings, encoding='utf-8') as f:
            self.assertFalse(json.load(f)['data'][0]['options'][0]['value'])
        # keep: nothing
        k = self.make()
        k.apply_graphics()
        self.assertNotIn('graphics', k.cycle)


if __name__ == '__main__':
    unittest.main()
