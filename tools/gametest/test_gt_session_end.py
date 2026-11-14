"""Offline tests of the end of a night session: the player's graphics values go back when the session ends (--graphics low, not
--graphics-keep), on every exit path, and never while the game runs. Only temp files are written; the settings file, the state folder
and the game process are fakes.

  python Source\\tools\\gametest\\test_gt_session_end.py -v
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import graphics_profile  # noqa: E402
import gt_session  # noqa: E402
import gt_win32  # noqa: E402
import night  # noqa: E402
import test_night_data as tnd  # noqa: E402
from test_gametest import FakeApi  # noqa: E402

night.padd_ping = lambda *a, **k: None

OPTIONS = {'FilmGrain': True, 'Vignette': True, 'MotionBlur': 'High'}


class SessionEnd(tnd.Base):
    def setUp(self):
        super().setUp()
        self.settings = os.path.join(self.root, 'UserSettings.json')
        self.state = os.path.join(self.root, 'graphics_state')
        with open(self.settings, 'w', encoding='utf-8') as f:
            json.dump({'data': [{'group_name': '/graphics/basic', 'options': [
                {'name': 'FilmGrain', 'value': True}, {'name': 'Vignette', 'value': True},
                {'name': 'MotionBlur', 'value': 'High', 'index': 3, 'values': ['Off', 'Low', 'Medium', 'High']}]},
                {'group_name': '/audio', 'options': [{'name': 'Volume', 'value': 70}]}]}, f)
        self._saved = (graphics_profile.game_running, night.process_path, gt_session.apply_graphics, gt_session.restore_graphics,
                       night.keep_awake, night.install_signal_handlers, night.run_cycles, gt_win32.Win32Api)
        self.game_running = False
        graphics_profile.game_running = lambda: self.game_running
        night.process_path = lambda pid: ''
        apply_real, restore_real = gt_session.apply_graphics, gt_session.restore_graphics
        gt_session.apply_graphics = lambda mode, settings=None, state_dir=None: apply_real(mode, self.settings, self.state)
        gt_session.restore_graphics = lambda settings=None, state_dir=None: restore_real(self.settings, self.state)
        night.keep_awake = lambda on: None
        night.install_signal_handlers = lambda n: None
        self.api = FakeApi()
        self.api.running = False
        gt_win32.Win32Api = lambda: self.api

    def tearDown(self):
        (graphics_profile.game_running, night.process_path, gt_session.apply_graphics, gt_session.restore_graphics,
         night.keep_awake, night.install_signal_handlers, night.run_cycles, gt_win32.Win32Api) = self._saved
        super().tearDown()

    def values(self) -> dict:
        with open(self.settings, encoding='utf-8') as f:
            return {o['name']: o['value'] for g in json.load(f)['data'] for o in g['options']}

    def night(self, *extra):
        n = self.make('--graphics', 'low', *extra)
        n.api = self.api
        return n

    # -- the method --
    def test_applied_and_the_game_closed_the_values_come_back_with_one_counted_line(self):
        n = self.night()
        n.apply_graphics()
        self.assertEqual(self.values()['FilmGrain'], False)
        lines: list = []
        n.log = lines.append
        res = n.finish_graphics()
        self.assertEqual(res['restored'], 3)
        self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})
        self.assertEqual(lines, ['[graphics] graphics profile restored: 3 options (the player\'s values are back)'])
        self.assertFalse(n.graphics_applied)
        self.assertIsNone(n.finish_graphics())                      # nothing left to do the second time

    def test_graphics_keep_leaves_the_profile_for_a_chain_of_sessions(self):
        n = self.night('--graphics-keep')
        n.apply_graphics()
        lines: list = []
        n.log = lines.append
        self.assertEqual(n.finish_graphics(), {'kept': True})
        self.assertEqual(self.values()['FilmGrain'], False)
        self.assertTrue(os.path.isfile(os.path.join(self.state, 'state.json')))
        self.assertIn('--graphics-keep', lines[0])
        n2 = self.night()                                           # the next session of the chain restores the first values
        n2.apply_graphics()
        n2.finish_graphics()
        self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})

    def test_a_game_that_still_runs_is_never_written_to(self):
        n = self.night()
        n.apply_graphics()
        self.api.running = True
        lines: list = []
        n.log = lines.append
        res = n.finish_graphics()
        self.assertEqual(res, {'restored': None, 'game_running': True})
        self.assertEqual(self.values()['FilmGrain'], False)
        self.assertIn('NOT restored', lines[0])
        self.assertIn('graphics_profile.py --restore', lines[0])
        self.api.running = False                                    # closed later: the same call restores
        n.finish_graphics()
        self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})

    def test_an_unreadable_process_list_counts_as_running(self):
        n = self.night()
        n.apply_graphics()

        def boom(exe):
            raise OSError('no access')
        self.api.list_pids = boom
        n.log = lambda m: None
        self.assertEqual(n.finish_graphics(), {'restored': None, 'game_running': True})
        self.assertEqual(self.values()['FilmGrain'], False)

    def test_nothing_applied_nothing_restored(self):
        n = self.make('--graphics', 'keep')
        n.api = self.api
        n.apply_graphics()
        self.assertIsNone(n.finish_graphics())
        self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})
        self.assertFalse(os.path.exists(self.state))
        self.game_running = True                                    # the apply was refused: nothing to put back
        n2 = self.night()
        n2.api = self.api
        n2.apply_graphics()
        self.assertFalse(n2.graphics_applied)
        self.assertIsNone(n2.finish_graphics())

    # -- main(): every exit path of the session --
    def run_main(self, cycles, *extra):
        night.run_cycles = cycles
        argv = ['--go', '--out-root', os.path.join(self.root, 'out'), '--log-dir', self.game, '--graphics', 'low', *extra]
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                return night.main(argv), out.getvalue()
            except BaseException as e:      # noqa: BLE001 - KeyboardInterrupt is part of the cases
                return e, out.getvalue()

    def cycles_that(self, outcome):
        def run_cycles(n, args, results, body):
            n.apply_graphics()
            self.assertEqual(self.values()['FilmGrain'], False)         # the game "runs" with the low profile
            if outcome == 'ctrl-c':
                raise KeyboardInterrupt()
            if outcome == 'defect':
                raise RuntimeError('defect of the runner')
            results.append({'cycle': 'c1', 'body': body, 'result': {'ok': 'ok', 'failed': 'failed: scenario failed',
                                                                    'stop': 'stopped: STOP file'}[outcome]})
            return outcome == 'stop'
        return run_cycles

    def test_after_the_last_cycle_ok_failed_or_stopped(self):
        for outcome in ('ok', 'failed', 'stop'):
            with self.subTest(outcome):
                rc, out = self.run_main(self.cycles_that(outcome))
                self.assertEqual(rc, 0 if outcome == 'ok' else 1)
                self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})
                self.assertEqual(out.count('graphics profile restored: 3 options'), 1, out)

    def test_after_ctrl_c_and_after_a_defect_of_the_runner(self):
        for outcome, exc in (('ctrl-c', KeyboardInterrupt), ('defect', RuntimeError)):
            with self.subTest(outcome):
                rc, out = self.run_main(self.cycles_that(outcome))
                self.assertIsInstance(rc, exc)                              # still raised: nothing is swallowed
                self.assertEqual(self.values(), {**OPTIONS, 'Volume': 70})
                self.assertIn('graphics profile restored: 3 options', out)

    def test_ctrl_c_with_the_game_still_running_leaves_the_profile_and_says_how_to_put_it_back(self):
        def run_cycles(n, args, results, body):
            n.apply_graphics()
            self.api.running = True                 # the interrupted cycle left the game open
            raise KeyboardInterrupt()
        rc, out = self.run_main(run_cycles)
        self.assertIsInstance(rc, KeyboardInterrupt)
        self.assertEqual(self.values()['FilmGrain'], False)
        self.assertIn('NOT restored', out)
        self.assertIn('graphics_profile.py --restore', out)

    def test_graphics_keep_in_a_session(self):
        rc, out = self.run_main(self.cycles_that('ok'), '--graphics-keep')
        self.assertEqual(rc, 0)
        self.assertEqual(self.values()['FilmGrain'], False)
        self.assertIn('--graphics-keep', out)
        self.assertNotIn('restored: ', out)


if __name__ == '__main__':
    unittest.main()
