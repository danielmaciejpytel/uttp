"""Offline tests for the night runner (night.py / gt_night.py). No input is sent, no process is started or killed.

  python Source\\tools\\gametest\\test_night.py -v
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import tempfile
from unittest.mock import ANY  # noqa: F401


def approx(x, tol=1e-6):
    class _A:
        def __eq__(self, o):
            return abs(o - x) <= tol
        def __repr__(self):
            return f'~{x}'
    return _A()
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_night as gn  # noqa: E402
import gt_settings as gs  # noqa: E402
import night  # noqa: E402
from gt_scenario import BUTTONS, load_scenarios, parse_scenario  # noqa: E402
from test_gametest import GAME_PID, OTHER_HWND, FakeApi  # noqa: E402

night.padd_ping = lambda *a, **k: None          # tests never reach the real padd daemon / ViGEm pad


def _no_pad(*a, **k):
    raise AssertionError('tests must not open a pad')


night.gt_run.open_pad = _no_pad

PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
LOG_057 = os.path.join(PROJECT, 'Research', 'logs', 'utpp-0.5.7-user-play.log')
SCEN = os.path.join(HERE, 'scenarios.json')

GDIGRAB_STDERR = """Input #0, gdigrab, from 'desktop':
  Duration: N/A, start: 1791344963.384235, bitrate: 7962636 kb/s
  Stream #0:0: Video: bmp, bgra, 3840x2160, 7962636 kb/s, 30 fps, 1000k tbr, 1000k tbn
"""



def no_crash_probe(calls):
    """ensure_closed() ends with a read-only `tasklist CrashReporter.exe` probe (crash-reporter cleanup); drop it."""
    return [c for c in calls if 'CrashReporter.exe' not in ' '.join(map(str, c))]

class SetTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)

    def test_sets_resolve_and_validate(self):
        self.assertEqual(len(gn.resolve_sets('loco')), 11)
        self.assertEqual(gn.resolve_sets('crouch_turns'), ['crouch_walk_turns'])
        self.assertEqual(gn.resolve_sets('loco,crouch_turns').count('crouch_walk_turns'), 1)
        for name in gn.SETS:
            self.assertEqual(gn.validate_sets(gn.resolve_sets(name), self.sc), [], name)
        self.assertTrue(gn.validate_sets(['nope'], self.sc))
        missing = {k: v for k, v in self.sc.items() if k != 'veh_exit'}
        self.assertIn('veh_exit', gn.validate_sets(['vehicle_mount_exit'], missing)[0])

    def test_new_scenarios(self):
        dash = self.sc['sprint_dash']
        taps = [b for s in dash.steps for b in s.buttons if b.button == 'B']
        self.assertEqual(len(taps), 4)
        self.assertTrue(all(b.hold < 0.12 for b in taps), 'dodge must be a tap (B = Dodge; crouch = R3 tap under the Alternative scheme)')
        self.assertNotIn('weapon_draw_aim_holster', self.sc)          # now a night.py macro
        self.assertIn('weapon_draw_aim_holster', gn.MACROS)
        draw = [b for s in self.sc['weapon_draw'].steps for b in s.buttons]
        self.assertEqual([(b.button, b.hold) for b in draw], [('Y', 0.1)])
        hol = [b for s in self.sc['weapon_holster'].steps for b in s.buttons]
        self.assertEqual([b.button for b in hol], ['Y', 'Y'])           # double tap
        self.assertAlmostEqual(hol[1].t - hol[0].t, 0.15)
        self.assertTrue(hol[0].t + hol[0].hold < hol[1].t)              # released between the taps
        aim = self.sc['weapon_aim']
        st = next(s for s in aim.steps if s.name == 'aim_walk')
        frames = [f for i, f in aim.sample(1 / 30) if aim.steps[i] is st]
        self.assertGreater(max(f.lt for f in frames), 0.99)
        self.assertEqual(self.sc['veh_enter'].steps[0].buttons[0].button, 'X')
        self.assertEqual(self.sc['veh_exit'].steps[0].buttons[0].button, 'B')
        self.assertIn('DPAD_RIGHT', BUTTONS)
        for n in gn.VEHICLE_PARTS + gn.RECOVERY_PARTS + ['weapon_draw', 'weapon_holster']:
            self.assertTrue(all(not s.metrics for s in self.sc[n].steps), n)

    def test_aim_probe_set_and_step(self):
        self.assertEqual(gn.resolve_sets('aim_probe'), ['aim_probe'])
        self.assertEqual(gn.MACROS['aim_probe'], gn.AIM_PARTS)
        self.assertNotIn('aim_probe', self.sc)                                      # a night.py macro
        self.assertNotIn('aim_probe', gn.SETS['all'])                               # opt-in only
        self.assertEqual(gn.validate_sets(['aim_probe'], self.sc), [])
        missing = {k: v for k, v in self.sc.items() if k != 'aim_probe_hold'}
        self.assertIn('aim_probe_hold', gn.validate_sets(['aim_probe'], missing)[0])
        sc = self.sc['aim_probe_hold']
        self.assertEqual(len(sc.steps), 1)
        self.assertAlmostEqual(sc.steps[0].duration, 4.5)
        self.assertFalse(sc.steps[0].metrics)
        fr = [(i / 60, f) for i, (_, f) in enumerate(sc.sample(1 / 60))]
        self.assertTrue(all(tuple(f.ls) == (0, 0) and tuple(f.rs) == (0, 0) for _, f in fr))               # stick zero
        lt_on = [ts for ts, f in fr if f.lt > 0.99]
        self.assertAlmostEqual(min(lt_on), 0.2, delta=0.03)
        self.assertAlmostEqual(max(lt_on), 4.0, delta=0.03)
        self.assertLess(next(f.lt for ts, f in fr if ts >= 0.1 - 1e-9), 0.6)       # a ramp, not a jump
        taps, prev = [], 0.0
        for ts, f in fr:
            if f.rt > 0.5 >= prev:
                taps.append(ts)
            prev = f.rt
        self.assertEqual(len(taps), 2)
        self.assertAlmostEqual(taps[0], 1.5, delta=0.03)
        self.assertAlmostEqual(taps[1], 2.5, delta=0.03)
        self.assertAlmostEqual(sum(1 for _, f in fr if f.rt > 0.5) / 60, 2 * 0.15, delta=0.06)
        self.assertTrue(all(0.2 < ts < 4.0 for ts in taps))                        # the shots happen while aiming

    def test_melee_fists_set_and_steps(self):
        self.assertEqual(gn.resolve_sets('melee_fists'), ['melee_fists'])
        self.assertEqual(gn.MACROS['melee_fists'], gn.MELEE_PARTS)
        self.assertNotIn('melee_fists', self.sc)                                   # a night.py macro
        al = gn.SETS['all']
        self.assertEqual(al[al.index('weapon_draw_aim_holster') + 1], 'melee_fists')
        self.assertEqual(al[0], 'vehicle_mount_exit')                              # the car first: V starts next to it
        self.assertEqual(gn.validate_sets(gn.resolve_sets('all'), self.sc), [])
        missing = {k: v for k, v in self.sc.items() if k != 'melee_block'}
        self.assertIn('melee_block', gn.validate_sets(['melee_fists'], missing)[0])

        def rt_taps(name, ch='rt'):
            sc = self.sc[name]
            fr = [f for _, f in sc.sample(1 / 60)]
            taps, prev = [], 0.0
            for i, f in enumerate(fr):
                v = getattr(f, ch)
                if v > 0.5 >= prev:
                    taps.append(i / 60)
                prev = v
            return taps, sum(1 for f in fr if getattr(f, ch) > 0.5) / 60
        taps, held = rt_taps('melee_draw')
        self.assertEqual(len(taps), 1)
        self.assertAlmostEqual(held, 0.15, delta=0.03)
        self.assertGreaterEqual(self.sc['melee_draw'].steps[-1].duration, 0.25 + 1.2 - 0.01)   # tap + 1.2 s idle
        taps, held = rt_taps('melee_punch')
        self.assertEqual(len(taps), 3)
        self.assertAlmostEqual(taps[1] - taps[0], 0.7, delta=0.03)
        self.assertAlmostEqual(taps[2] - taps[1], 0.7, delta=0.03)
        self.assertAlmostEqual(held, 3 * 0.15, delta=0.06)
        self.assertFalse(gn.commands_movement(self.sc['melee_punch']))
        taps, _ = rt_taps('melee_punch_walk')
        self.assertEqual(len(taps), 2)
        self.assertTrue(gn.commands_movement(self.sc['melee_punch_walk'], 0.45))
        self.assertFalse(gn.commands_movement(self.sc['melee_punch_walk'], 0.6))        # stick 0.5, a walk
        _, held = rt_taps('melee_block', 'lt')
        self.assertAlmostEqual(held, 2.0, delta=0.2)                                  # 2.0 s at full + ramps
        hol = [(b.button, b.t, b.hold) for s_ in self.sc['melee_holster'].steps for b in s_.buttons]
        wh = [(b.button, b.t, b.hold) for s_ in self.sc['weapon_holster'].steps for b in s_.buttons]
        self.assertEqual(hol, wh)                                                       # same double tap Y
        for n in gn.MELEE_PARTS:
            self.assertTrue(all(b.button != 'Y' or b.hold < 0.15 for st in self.sc[n].steps for b in st.buttons), n)
            expect_metrics = n == 'melee_punch_walk'                                    # like weapon_walk
            self.assertTrue(all(st.metrics == expect_metrics for st in self.sc[n].steps), n)

    def test_no_y_hold_anywhere(self):
        """A Y hold opens the weapon wheel (first night cycle, 2026-10-07 06:04): only taps."""
        for n, sc in self.sc.items():
            for st in sc.steps:
                for b in st.buttons:
                    if b.button == 'Y':
                        self.assertLess(b.hold, 0.15, f'{n}.{st.name}')

    def test_movement_commands(self):
        self.assertTrue(gn.commands_movement(self.sc['slow_walk_half_stick']))
        self.assertTrue(gn.commands_movement(self.sc['wheel_probe']))
        self.assertFalse(gn.commands_movement(self.sc['veh_enter']))
        self.assertFalse(gn.commands_movement(self.sc['weapon_holster']))


class FfmpegTests(unittest.TestCase):
    RECT, SCREEN = (960, 540, 2880, 1620), (0, 0, 3840, 2160)

    def test_desktop_crop_default(self):
        c = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', rect=self.RECT, screen=self.SCREEN)
        self.assertEqual(c[c.index('-f') + 1], 'gdigrab')
        self.assertEqual(c[c.index('-i') + 1], 'desktop')
        self.assertEqual((c[c.index('-offset_x') + 1], c[c.index('-offset_y') + 1], c[c.index('-video_size') + 1]),
                         ('960', '540', '1920x1080'))
        self.assertEqual(c[c.index('-c:v') + 1], 'h264_nvenc')
        self.assertEqual(c[c.index('-movflags') + 1], '+frag_keyframe+empty_moov+default_base_moof')
        self.assertEqual(c[-1], 'o.mp4')
        self.assertNotIn('drawtext', ' '.join(c))
        self.assertFalse(any(x.startswith('title=') for x in c))
        # covering the monitor: still cropped (multi-monitor desktops)
        c2 = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', rect=self.SCREEN, screen=self.SCREEN, encoder='libx264')
        self.assertEqual(c2[c2.index('-video_size') + 1], '3840x2160')
        self.assertIn('ultrafast', c2)
        over = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', rect=(2561, 0, 3847, 2101), screen=self.SCREEN)
        self.assertEqual(over[over.index('-video_size') + 1], '1278x2100')   # clamped to the monitor, even
        odd = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', rect=(100, 50, 1637, 915))
        self.assertEqual(odd[odd.index('-video_size') + 1], '1536x864')

    def test_ddagrab_crop(self):
        d = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', 'ddagrab', rect=self.RECT, screen=self.SCREEN)
        self.assertTrue(d[d.index('-i') + 1].startswith('ddagrab=output_idx=0:framerate=30'))
        self.assertEqual(d[d.index('-vf') + 1], 'hwdownload,format=bgra,crop=1920:1080:960:540,'
                                                'scale=trunc(iw/2)*2:trunc(ih/2)*2')
        full = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', 'ddagrab', rect=self.SCREEN, screen=self.SCREEN)
        self.assertNotIn('crop=', full[full.index('-vf') + 1])
        second = gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', 'ddagrab', rect=(4000, 100, 5920, 1180),
                                     screen=(3840, 0, 7680, 2160))
        self.assertIn('crop=1920:1080:160:100', second[second.index('-vf') + 1])

    def test_errors(self):
        with self.assertRaises(ValueError):
            gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', 'title')
        with self.assertRaises(ValueError):
            gn.build_ffmpeg_cmd('ff.exe', 'o.mp4', encoder='x265')

    def test_parsers_and_check(self):
        err = ("Input #0, mov,mp4, from 'o.mp4':\n  Duration: 00:00:03.00, start: 0.000000\n"
               "  Stream #0:0[0x1](und): Video: h264 (Main) (avc1 / 0x31637661), yuv420p(tv, progressive), "
               "1920x1080, 1200 kb/s, 30 fps\nframe=   60 fps=0.0 q=-0.0 size=N/A time=00:00:01.50 bitrate=N/A\n"
               "frame=   90 fps=0.0 q=-0.0 Lsize=N/A time=00:00:02.00 bitrate=N/A\n")
        self.assertEqual(gn.parse_frame_size(err), (1920, 1080))
        self.assertEqual(gn.parse_decoded_seconds(err), 2.0)
        self.assertIsNone(gn.parse_decoded_seconds('nothing'))
        self.assertEqual(gn.recording_check((1920, 1080), (1920, 1080), [40.0, 44.0]), [])
        black = gn.recording_check((1920, 1080), (1920, 1080), [0.0, 0.1, 0.0])
        self.assertTrue(black and black[0].startswith('BLACK'))
        self.assertIn('frame size 3840x2160 != game client 1920x1080',
                      gn.recording_check((3840, 2160), (1920, 1080), [40.0]))
        self.assertTrue(gn.recording_check(None, (1920, 1080), []))


class ClockTests(unittest.TestCase):
    def test_parse_start(self):
        self.assertEqual(gn.parse_input_start(GDIGRAB_STDERR), ('gdigrab', 1791344963.384235))
        self.assertIsNone(gn.parse_input_start('no banner'))

    def test_epoch_start_and_marker_same_clock(self):
        spawn = datetime.fromtimestamp(1791344963.285)
        stop = spawn + timedelta(seconds=100.2)
        ck = gn.recording_clock(spawn, stop, 100.0, ('gdigrab', 1791344963.384235),
                                marker_send=spawn + timedelta(seconds=5), marker_log=spawn + timedelta(seconds=5.04))
        self.assertEqual(ck['start_method'], 'gdigrab_epoch_start')
        self.assertAlmostEqual((ck['video_start'] - spawn).total_seconds(), 0.099, places=3)
        self.assertEqual(ck['video_end'] - ck['video_start'], timedelta(seconds=100))
        self.assertAlmostEqual(ck['start_vs_stop_delta_s'], 0.101, places=2)
        self.assertEqual(ck['clock_offset_s'], 0.0)          # 40 ms = input latency, same clock
        self.assertAlmostEqual(ck['log_minus_wall_s'], 0.04)
        self.assertLess(ck['uncertainty_s'], 0.1)

    def test_fallbacks(self):
        spawn = datetime(2026, 10, 7, 23, 0, 0)
        ck = gn.recording_clock(spawn, spawn + timedelta(seconds=61), 60.0, ('gdigrab', 12.5))  # not an epoch
        self.assertEqual(ck['start_method'], 'spawn_time')
        self.assertEqual(ck['video_start'], spawn)
        self.assertGreaterEqual(ck['uncertainty_s'], 1.0)
        ck2 = gn.recording_clock(spawn, spawn + timedelta(seconds=61), 60.0, None,
                                 spawn + timedelta(seconds=3), spawn + timedelta(seconds=3) - timedelta(hours=2))
        self.assertEqual(ck2['clock_offset_s'], -7200.0)      # log in another clock (e.g. UTC): apply it

    def test_sheet_windows(self):
        v0 = datetime(2026, 10, 7, 23, 0, 0)
        w = gn.sheet_windows(v0, 30.0, [{'name': 'a', 'start': v0 + timedelta(seconds=1),
                                         'end': v0 + timedelta(seconds=29)}])
        self.assertEqual([(x['name'], x['start'], x['end']) for x in w], [('a_start', 0.0, 4.0), ('a_end', 26.0, 29.95)])


class QuitTests(unittest.TestCase):
    def test_quit_action(self):
        q = gn.quit_action
        self.assertEqual(q(False, 0, True, False), 'done')
        self.assertEqual(q(True, 10, True, False), 'wait')
        self.assertEqual(q(True, 60, True, False), 'taskkill')
        self.assertEqual(q(True, 61, True, True), 'wait')
        self.assertEqual(q(True, 81, True, True), 'taskkill_force')
        self.assertEqual(q(True, 0, False, False), 'taskkill')      # console unavailable: straight to taskkill
        self.assertEqual(q(True, 5, False, True), 'wait')
        self.assertEqual(q(True, 21, False, True), 'taskkill_force')


class FakeTime:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s


class EnsureClosedTests(unittest.TestCase):
    quit_echo = True

    def setUp(self):
        self._pp, self._time = night.process_path, night.time
        night.process_path = lambda pid: ''
        night.time = FakeTime()

    def tearDown(self):
        night.process_path, night.time = self._pp, self._time

    def make(self, api, runner, log_dir=None):
        # isolated log dir: never read the real red4ext logs in tests
        args = night.build_parser().parse_args(['--quit-timeout', '3', '--kill-grace', '2', '--out-root', 'unused',
                                                '--log-dir', log_dir or tempfile.mkdtemp()])
        n = night.Night(args, runner=runner)
        n.settings.pending_path = None          # tests never write the pending restore file
        log = night.gt_log.newest_log(args.log_dir) or os.path.join(args.log_dir, 'utpp-2026-10-07-23-00-00.log')
        open(log, 'a').close()

        def echo(scans):  # the game logs '[night] QUIT exitgame ok=...' when the console runs QUIT_CMD
            typed = ''.join(c[1] for c in api.calls if c[0] == 'char')
            if scans == (0x1C,) and typed.endswith(gn.QUIT_CMD) and self.quit_echo:
                with open(log, 'a', encoding='utf-8') as f:
                    f.write('[2026-10-07 23:00:05.000] [info] [1] [UTPP] UTPP CET [night] QUIT exitgame ok=true nil'
                            + chr(10))
        api.on_scan.insert(0, echo)
        orig_sleep = api.sleep

        def sleep(sec):  # advance the fake clock used by the guard and the log waits
            orig_sleep(sec)
            if isinstance(night.time, FakeTime):
                night.time.sleep(sec)
        api.sleep = sleep
        n.api = api
        n.cycle = {'name': 't', 'steps': []}
        n.overlay_grab = lambda guard: None     # no screen capture in tests (OverlayTests use fake frames)
        return n

    def test_cet_path(self):
        api = FakeApi()
        api.on_scan.append(lambda scans: setattr(api, 'running', False) if scans == (0x1C,) else None)
        calls = []
        res = self.make(api, lambda c, **k: calls.append(c)).ensure_closed('quit')
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertTrue(res['cet_sent'])
        self.assertEqual(no_crash_probe(calls), [])  # the CrashReporter probe after the quit is not a close action
        typed = ''.join(c[1] for c in api.calls if c[0] == 'char')
        self.assertIn('GetSystemRequestsHandler():ExitGame()', typed)
        self.assertTrue(all(c[2] == 11 for c in api.calls if c[0] in ('char', 'keys')))  # all into the game window

    def test_final_commands_are_typed_before_exit_game(self):
        api = FakeApi()
        api.on_scan.append(lambda scans: setattr(api, 'running', False)
                           if scans == (0x1C,) and ''.join(c[1] for c in api.calls if c[0] == 'char').endswith(gn.QUIT_CMD)
                           else None)
        n = self.make(api, lambda c, **k: None)
        n.args.cet_final = "Game.UTPPLogLine('[night] final one');;Game.UTPPLogLine('[night] final two')"
        res = n.ensure_closed('quit')
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertEqual(res['final_commands'], 2)
        typed = ''.join(c[1] for c in api.calls if c[0] == 'char')
        self.assertLess(typed.index('final one'), typed.index('final two'))
        self.assertLess(typed.index('final two'), typed.index('ExitGame()'))    # the game closes after them
        # no final commands: nothing extra is typed
        api2 = FakeApi()
        api2.on_scan.append(lambda scans: setattr(api2, 'running', False) if scans == (0x1C,) else None)
        res2 = self.make(api2, lambda c, **k: None).ensure_closed('quit')
        self.assertNotIn('final_commands', res2)

    def test_final_commands_failure_does_not_block_the_quit(self):
        api = FakeApi()
        api.on_scan.append(lambda scans: setattr(api, 'running', False) if scans == (0x1C,) else None)
        n = self.make(api, lambda c, **k: None)
        n.args.cet_final_file = os.path.join(tempfile.mkdtemp(), 'missing.txt')
        res = n.ensure_closed('quit')
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertIn('final_commands_error', res)

    def test_quit_command_lost_three_times_goes_to_taskkill(self):
        self.quit_echo = False
        api = FakeApi()
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            api.running = False
        n = self.make(api, runner)
        res = n.ensure_closed('quit')
        self.assertFalse(res['cet_sent'])
        self.assertEqual(res['path'], 'taskkill')
        self.assertEqual(n.cet_stats['quit'], [0])
        typed = ''.join(c[1] for c in api.calls if c[0] == 'char')
        self.assertEqual(typed.count('ExitGame()'), 3)                    # 3 attempts (settle 0.5/1.0/1.5)

    def test_taskkill_fallbacks(self):
        api = FakeApi()
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            if '/F' in cmd:
                api.running = False
        res = self.make(api, runner).ensure_closed('quit')
        self.assertEqual(res['path'], 'taskkill_force')
        self.assertEqual(no_crash_probe(calls), [['taskkill', '/IM', 'Cyberpunk2077.exe'], ['taskkill', '/F', '/IM', 'Cyberpunk2077.exe']])

    def test_windowless_leftover_is_force_killed_without_typing(self):
        api = FakeApi()
        api.find_main_window = lambda pids: 0       # the user closed the game, the process stayed alive
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            if '/F' in cmd:
                api.running = False
        res = self.make(api, runner).ensure_closed('ensure_closed')
        self.assertTrue(res['windowless'])
        self.assertEqual(res['path'], 'taskkill_force')
        self.assertEqual(no_crash_probe(calls), [['taskkill', '/F', '/IM', 'Cyberpunk2077.exe']])
        self.assertFalse([c for c in api.calls if c[0] in ('char', 'keys', 'click', 'focus')])

    def test_no_focus_goes_to_taskkill_without_typing(self):
        api = FakeApi()
        api.fg, api.focus_makes_foreground = OTHER_HWND, False
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            api.running = False
        res = self.make(api, runner).ensure_closed('quit')
        self.assertFalse(res['cet_sent'])
        self.assertEqual(res['path'], 'taskkill')
        self.assertFalse([c for c in api.calls if c[0] in ('char', 'keys', 'click')])

    def _mounted_quit(self, react: bool):
        d = tempfile.mkdtemp()
        log = os.path.join(d, 'utpp-2026-10-07-23-00-00.log')
        with open(log, 'w', encoding='utf-8') as f:
            f.write('[2026-10-07 23:00:00.000] [info] [1] [UTPP] UTPP CET [utpp_vehiclecam] t=1.00 keepTpp mount -> ride'
                    + chr(10))

        class Api(_KeyApi):
            def sleep(self_api, sec):
                night.time.sleep(sec)

            def _send(self_api, evs):
                self_api.calls.append(('raw', tuple(evs), self_api.fg))
                if react and evs[0] == (gn.SCAN_F, True):
                    with open(log, 'a', encoding='utf-8') as f:
                        f.write('[2026-10-07 23:00:01.000] [info] [1] [UTPP] UTPP CET [utpp_vehiclecam] t=2.00 '
                                'keepTpp ride -> exit' + chr(10) + '[2026-10-07 23:00:02.000] [info] [1] [UTPP] '
                                'UTPP CET [utpp_vehiclecam] t=3.00 keepTpp exit -> off' + chr(10))
        api = Api()
        api.on_scan.append(lambda scans: setattr(api, 'running', False) if scans == (0x1C,) and
                           ''.join(c[1] for c in api.calls if c[0] == 'char').endswith(gn.QUIT_CMD) else None)
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            api.running = False
        n = self.make(api, runner, log_dir=d)
        n.wait_seated = lambda pad, guard, ep=None: {'seated_at': datetime.now()}   # seated (covered elsewhere)
        n.poll_pos = lambda guard, pad=None, timeout=None: None   # no NightPos answer: log phase decides
        n.args.cet_unmount = False   # scripted unmount covered in MacroTests (real typing here is slow)
        return n.ensure_closed('quit'), api, calls

    def test_quit_exits_vehicle_first(self):
        res, api, calls = self._mounted_quit(True)
        self.assertTrue(res.get('exited_vehicle'))
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertEqual(no_crash_probe(calls), [])
        raw = [c for c in api.calls if c[0] == 'raw']
        typed = ''.join(c[1] for c in api.calls if c[0] == 'char')
        self.assertEqual(raw[0][1], ((gn.SCAN_F, False),))                  # F before the console
        self.assertIn('ExitGame()', typed)

    def test_quit_still_mounted_goes_to_taskkill(self):
        res, api, calls = self._mounted_quit(False)
        self.assertTrue(res.get('still_mounted'))
        self.assertEqual(sum(1 for c in api.calls if c[0] == 'raw' and c[1] == ((gn.SCAN_F, False),)), 8)  # 2 holds + 6 taps
        self.assertFalse(res['cet_sent'])
        self.assertEqual(res['path'], 'taskkill')
        self.assertNotIn('ExitGame', ''.join(c[1] for c in api.calls if c[0] == 'char'))
        self.assertEqual(night.build_parser().parse_args([]).quit_timeout, night.QUIT_TIMEOUT_S)
        self.assertEqual(night.QUIT_TIMEOUT_S, 60.0)

    def test_quit_focus_loss_is_not_fatal(self):
        api = FakeApi()
        api.on_scan.append(lambda scans: setattr(api, 'running', False) if scans == (0x1C,) and
                           ''.join(c[1] for c in api.calls if c[0] == 'char').endswith(gn.QUIT_CMD) else None)
        n = self.make(api, lambda *a, **k: None)
        real_guard, fails = n.guard, {'n': 0}

        def guard():
            g = real_guard()
            if fails['n'] < 2:   # the first two attempts lose the foreground (other.exe)
                fails['n'] += 1
                def bad():
                    raise night.GuardError('foreground window is not the game (focus lost: exe=other.exe)')
                g.ensure_focus = bad
            return g
        n.guard = guard
        res = n.ensure_closed('quit')
        self.assertEqual(res['path'], 'cet_exitgame')
        self.assertTrue(res['cet_sent'])

    def test_quit_focus_never_back_goes_to_taskkill(self):
        api = FakeApi()
        api.fg, api.focus_makes_foreground = OTHER_HWND, False
        calls = []

        def runner(cmd, **k):
            calls.append(cmd)
            api.running = False
        n = self.make(api, runner)
        res = n.ensure_closed('quit')              # no StopNight: refocus x3, then taskkill
        self.assertEqual(res['path'], 'taskkill')
        self.assertEqual(calls[0], ['taskkill', '/IM', 'Cyberpunk2077.exe'])
        self.assertTrue(issubclass(night.UserActive, night.StopNight) and night.UserActive.leave_game)

    def test_not_running(self):
        api = FakeApi()
        api.running = False
        self.assertEqual(self.make(api, None).ensure_closed('quit')['path'], 'not_running')

    def test_install_refused_while_running(self):
        api = FakeApi()
        n = self.make(api, lambda *a, **k: self.fail('installer must not run'))
        n.args.cet_mods = 'utpp_probe'
        with self.assertRaises(night.CycleError):
            n.install()
        self.assertEqual(GAME_PID, api.list_pids('Cyberpunk2077.exe')[0])

    def test_main_requires_go(self):
        self.assertEqual(night.main(['--set', 'loco']), 2)

    def test_main_refuses_with_stop_file(self):
        d = tempfile.mkdtemp()
        open(os.path.join(d, 'STOP'), 'w').close()
        self.assertEqual(night.main(['--go', '--out-root', d]), 3)   # before any process/input/launch

    def test_stop_file_and_deadline(self):
        d = tempfile.mkdtemp()
        n = self.make(FakeApi(), None)
        n.args.out_root = d
        n.check_continue()                                           # nothing requested
        open(os.path.join(d, 'STOP'), 'w').close()
        with self.assertRaises(night.GracefulStop):
            n.check_continue()
        self.assertFalse(os.path.exists(os.path.join(d, 'STOP')))    # consumed (renamed)
        self.assertTrue([f for f in os.listdir(d) if f.startswith('STOP.consumed-')])
        n2 = self.make(FakeApi(), None)
        n2.args.out_root = d
        n2.signal_reason = 'signal SIGBREAK'
        with self.assertRaises(night.GracefulStop):
            n2.check_continue()
        n3 = self.make(FakeApi(), None)
        n3.args.out_root = d
        n3.deadline = night.time.monotonic() - 1
        with self.assertRaisesRegex(night.GracefulStop, 'max-wall-hours'):
            n3.check_continue()
        self.assertTrue(issubclass(night.GracefulStop, night.CycleError))
        self.assertEqual(night.build_parser().parse_args([]).console_settles, '0.5,1.0,1.5')

    def test_scenario_timeout_and_args(self):
        n = self.make(FakeApi(), None)
        n.check_scenario_time()
        n.scenario_deadline = night.time.monotonic() - 1
        with self.assertRaises(night.ScenarioTimeout):
            n.check_scenario_time()
        a = night.build_parser().parse_args([])
        self.assertEqual((a.capture, a.wheel_close, a.on_stop), ('desktop', 'B', 'quit'))
        self.assertEqual(night.validate_args(a), [])
        bad = night.build_parser().parse_args(['--wheel-close', 'Q', '--vehicle-spawn', 'codeware,teleport'])
        self.assertEqual(len(night.validate_args(bad)), 2)
        self.assertEqual(night.validate_args(night.build_parser().parse_args(['--wheel-close', 'key:0x01'])), [])


class LogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(LOG_057, encoding='utf-8', errors='replace') as f:
            cls.raw = f.read()
        cls.lines = gt_log.parse_text(cls.raw)

    def test_night_metrics_match_raw_counts(self):
        m = gn.night_metrics(self.lines)
        costs = [float(x) for x in re.findall(r'MMTRACE TRANS .*?\bcost=([\d.]+)', self.raw)]
        self.assertEqual(m['trans_total'], len(costs))
        self.assertEqual(m['trans_cost_gt_60'], sum(1 for c in costs if c > 60))
        self.assertGreater(m['trans_cost_gt_60'], 0)
        vw1 = [float(v) for v, w in re.findall(r'MMTRACE .*?\|v\|=([\d.]+) .*?\bw=([\d.]+)', self.raw)
               if float(w) >= 0.999]
        self.assertAlmostEqual(m['max_v_w1'], round(max(vw1), 3))
        self.assertGreater(m['max_v_w1'], 8.0)     # the dash episode of 0.5.7 (|v| 8-17.6 m/s)
        self.assertEqual(m['gate_lines'], self.raw.count('MMGATE'))
        self.assertGreater(m['gate_flips_per_min'], 0)
        self.assertEqual(m['utpp_body_failed'], len(re.findall(r'\[utpp_body\].*FAILED', self.raw)))
        self.assertEqual(m['perspective_policy_difference'], self.raw.count('policy difference'))
        eps = m['vehiclecam_episodes']
        self.assertGreaterEqual(len(eps), 4)
        self.assertEqual(sum(e['samples'] for e in eps), self.raw.count('cam veh=('))
        self.assertEqual(m['vehiclecam_max_jump_m'], max(e['max_jump_m'] for e in eps))

    def test_windowed_metrics(self):
        t0, t1 = datetime(2026, 10, 7, 5, 6, 40), datetime(2026, 10, 7, 5, 7, 0)
        m = gn.night_metrics(self.lines, t0, t1)
        self.assertEqual(m['gate_lines'], sum(1 for l in self.lines if l.kind == 'GATE' and t0 <= l.ts <= t1))
        self.assertTrue(m['vehiclecam_episodes'])

    def test_events(self):
        self.assertEqual(gn.gameplay_seen('x re-arming\n'), None)
        self.assertEqual(gn.gameplay_seen(self.raw), 'player animated object/tracks changed, re-arming')
        db = re.search(r'speed calibration \[([^\]]+)\]', self.raw).group(1)  # the database named in the recorded log
        self.assertTrue(gn.speed_calibrated(self.raw, db))
        self.assertFalse(gn.speed_calibrated(self.raw, db + '_other'))
        self.assertEqual(gn.find_line_ts(self.raw, gn.MOUNT_RIDE), datetime(2026, 10, 7, 5, 6, 46, 982000))
        self.assertEqual(gn.find_line_ts(self.raw, gn.EXIT_PULL_END), datetime(2026, 10, 7, 5, 6, 53, 90000))


class VehicleGeometryTests(unittest.TestCase):
    def test_parse_pos(self):
        t = ('[2026-10-07 23:00:00.000] [info] [1] [UTPP] UTPP CET [night] POS p=1.00,2.00,3.00 cf=0.000,1.000 '
             'v=5.00,2.00,3.00,0.000,-1.000 src=tag\n')
        p = gn.parse_pos(t)
        self.assertEqual(p['p'], (1.0, 2.0, 3.0))
        self.assertEqual(p['v'], (5.0, 2.0, 3.0))
        self.assertEqual(p['src'], 'tag')
        self.assertIsNone(gn.parse_pos('nothing'))
        self.assertIsNone(gn.parse_pos(t.replace('v=5.00,2.00,3.00,0.000,-1.000', 'v=nil'))['v'])
        # vehicle right = (0,-1) -> driver door on the left = +y side
        self.assertEqual(gn.door_point(p, 1.8), (5.0, 3.8))

    def test_approach_stick(self):
        sx, sy, d = gn.approach_stick((0, 0), (0, 1), (0, 10))
        self.assertEqual((sx, sy), (0.0, 0.6))
        self.assertEqual(d, 10)
        sx, sy, _ = gn.approach_stick((0, 0), (0, 1), (10, 0), mag=1.0)
        self.assertEqual((sx, sy), (1.0, 0.0))             # target east, camera north -> stick right
        sx, sy, _ = gn.approach_stick((0, 0), (1, 0), (0, 10), mag=1.0)
        self.assertEqual((sx, sy), (-1.0, 0.0))            # camera east, target north -> stick left
        self.assertEqual(gn.approach_stick((0, 0), (0, 1), (0, 0))[:2], (0.0, 0.0))

    def test_live_pos_record_2026_10_07(self):
        """POS of the first night cycle (06:05:23): the Codeware car was spawned 3 m right / 2 m ahead of V."""
        t = ('[2026-10-07 06:05:23.689] [info    ] [ 66708] [UTPP] UTPP CET [night] POS p=1505.59,2205.67,186.32 '
             'cf=-0.925,-0.381 v=1502.60,2207.69,186.79,-0.381,0.925 src=tag')
        p = gn.parse_pos(t)
        self.assertIsNone(p['weapon'])                      # inline probe: no w=/m= fields
        sx, sy, d = gn.approach_stick(p['p'][:2], p['cf'], p['v'][:2], mag=1.0)
        # camera right = (cf.y, -cf.x) = (-0.381, 0.925) equals the car's GetWorldRight (the car got V's orientation)
        self.assertAlmostEqual(sx, 3.008 / 3.608, delta=0.005)   # 3.0 m to the right of the camera direction
        self.assertAlmostEqual(sy, 1.996 / 3.608, delta=0.005)   # 2.0 m ahead
        self.assertAlmostEqual(d, 3.6, places=1)
        door = gn.door_point(p, 1.8)                        # left side of the car = towards V
        dx, dy, dd = gn.approach_stick(p['p'][:2], p['cf'], door, mag=1.0)
        self.assertAlmostEqual(dd, (1.19 ** 2 + 2.01 ** 2) ** 0.5, places=1)
        self.assertGreater(dy, dx)                          # door is ahead-right, nearer than the centre
        self.assertLess(dd, d)

    def test_nightpos_flags_and_mount(self):
        t = '[night] POS p=1.00,2.00,3.00 cf=0.000,1.000 v=nil src=none w=1 m=0'
        p = gn.parse_pos(t)
        self.assertEqual((p['weapon'], p['mounted']), (True, False))
        self.assertIsNone(gn.parse_pos(t.replace('w=1 m=0', 'w=- m=-'))['weapon'])
        self.assertEqual(gn.weapon_state(p, '', True), (True, 'NightPos w=1'))
        self.assertEqual(gn.weapon_state(p, '', False)[0], False)
        log = '[2026-10-07 06:04:43.935] [info] [1] [UTPP] UTPP CET [utpp_perspective] t=245.15 FPP <- weapon (weapon drawn) instant [active]'
        self.assertTrue(gn.weapon_state(None, log, True)[0])
        self.assertIsNone(gn.weapon_state(None, log, False)[0])
        self.assertIsNone(gn.weapon_state(None, '[x] UTPP CET [utpp_perspective] t=1 TPP <- user instant [active]', False)[0])
        self.assertTrue(gn.mount_seen('[2026-10-07 05:06:46.982] UTPP CET [utpp_vehiclecam] t=182.68 keepTpp mount -> ride'))
        h = '[2026-10-07 05:10:00.100] [info] UTPP CET [utpp_vehiclecam] t=617.36 handover 0.53 s after the vehicle TPP perspective'
        self.assertTrue(gn.mount_seen(h))
        self.assertEqual(gn.mount_ts(h), datetime(2026, 10, 7, 5, 10, 0, 100000))
        self.assertFalse(gn.mount_seen('[utpp_perspective] handover'))

    def test_movement_seen(self):
        lines = gt_log.parse_text('\n'.join([
            '[2026-10-07 06:05:30.000] [info] [1] [UTPP] UTPP MMTRACE tick clip=Idle f=1 cost=0 |v|=0.00 rate=1 w=1.00',
            '[2026-10-07 06:05:31.000] [info] [1] [UTPP] UTPP MMTRACE tick clip=Idle f=2 cost=0 |v|=0.02 rate=1 w=1.00',
            '[2026-10-07 06:05:40.000] [info] [1] [UTPP] UTPP MMTRACE tick clip=Run f=2 cost=0 |v|=3.10 rate=1 w=1.00']))
        t = datetime(2026, 10, 7, 6, 5, 29)
        self.assertFalse(gn.movement_seen(lines, t, t + timedelta(seconds=3)))   # stuck (weapon wheel)
        self.assertTrue(gn.movement_seen(lines, t, t + timedelta(seconds=12)))
        self.assertIsNone(gn.movement_seen(lines, t + timedelta(seconds=20), t + timedelta(seconds=30)))

    def test_commands_reference_verified_api(self):
        self.assertIn("GetSingleton('inkMenuScenario'):GetSystemRequestsHandler():ExitGame()", gn.QUIT_CMD)
        self.assertIn('GetDynamicEntitySystem()', gn.SPAWN_CODEWARE_CMD)
        self.assertIn('SpawnActivePlayerVehicle()', gn.SPAWN_SUMMON_CMD)
        self.assertEqual(gn.arm_cmds('v3')[-1], 'Game.UTPPLoadDatabase("v3")')
        self.assertEqual(gn.NIGHTPOS_CMD, 'GetMod("utpp_core").NightPos("UTPPNight")')
        self.assertIn('mf:Unmount(r)', gn.UNMOUNT_CMD)
        self.assertIn('d.isInstant=true', gn.UNMOUNT_CMD)
        for c in (gn.QUIT_CMD, gn.SPAWN_CODEWARE_CMD, gn.SPAWN_SUMMON_CMD, gn.POS_FUNC_DEF, gn.NIGHTPOS_CMD,
                  gn.UNMOUNT_CMD):
            self.assertNotIn('\n', c)
            self.assertTrue(all(32 <= ord(ch) < 127 for ch in c))


def _mm(sec, body):
    return f'[2026-10-07 23:00:{sec:06.3f}] [info    ] [ 1] [UTPP] UTPP ' + body


def _mask(sec, on, reasons=1, was=0, w=1.00):
    return _mm(sec, f'MMMASK {"on" if on else "off"} reasons=0x{reasons:X} (was 0x{was:X}) gate=0x0 covered=0x0 loco=1 '
                    f'upperBody=0 melee={int(reasons & 1)} exit=0.00 w={w:.2f} mask=0.00')


def _gate(sec, w=1.00, melee=1, hold=0, mask=1):
    return _mm(sec, f'MMGATE gate=0x0 (was 0x0) psm=0x0 detail=-1 loco=1 tpp=1 w={w:.2f} debounced=0 melee=0x{melee:X} '
                    f'meleeState={melee} meleeHold={hold} upperBody=0 mountCut=0 mask=0x{mask:X} hold=0')


def _tick(sec, w=1.00, mask=1, blend=0.50):
    return _mm(sec, f'MMTRACE tick clip=Stand_Idle f=1 cost=0.001 |v|=0.00 rate=1.00 yawRate=0.00 sUs=3 maxSUs=0 w={w:.2f} '
                    f'gate=0x0 loco=1 tpp=1 searches=8 mask={blend:.2f}/0x{mask:X} hold=0')


def _pos(sec, weapon=1, mounted=0):
    return _mm(sec, f'CET [night] POS p=1.00,2.00,3.00 cf=0.000,1.000 v=nil src=none w={weapon} m={mounted} n=1')


class MeleeStateTests(unittest.TestCase):
    def st(self, *lines, pos=(), active=False, **kw):
        return gn.melee_state(list(pos), chr(10).join(lines), active, **kw)

    def test_mask_on_off_parsing(self):
        r = self.st(_mask(1.0, True), _tick(2.0), _mask(3.0, False, 0, 1))
        self.assertEqual((r['on'], r['off'], r['active_now'], r['episodes']), (True, True, False, 1))
        self.assertEqual((r['min_w'], r['samples'], r['bad'], r['first_on']), (1.0, 1, [], '23:00:01.000'))
        self.assertTrue(self.st(_mask(1.0, True))['active_now'])                       # still on at the window end
        aim = self.st(_mask(1.0, True, 0x2), _tick(2.0, mask=2, w=0.0))                   # weapon aim bit 2: not melee
        self.assertEqual((aim['on'], aim['active_now'], aim['samples'], aim['bad']), (False, False, 0, []))
        both = self.st(_mask(1.0, True, 0x3), _mask(2.0, False, 0x2, 0x3))              # melee bit dropped, aim stays
        self.assertEqual((both['on'], both['off'], both['active_now']), (True, True, False))
        self.assertEqual(self.st()['min_w'], None)

    def test_gate_and_tick_lines_set_the_state(self):
        r = self.st(_gate(1.0, melee=0x4, mask=0), _tick(2.0))
        self.assertTrue(r['on'])                                                        # MMGATE melee=0x4 alone
        self.assertFalse(self.st(_gate(1.0, melee=0, hold=0, mask=0))['on'])
        self.assertTrue(self.st(_gate(1.0, melee=0, hold=1, mask=0))['on'])             # melee hold = still melee
        self.assertTrue(self.st(_gate(1.0, melee=0, hold=0, mask=1))['on'])             # mask=0x1 on the gate line
        r = self.st(_gate(1.0), _gate(2.0, melee=0, hold=0, mask=0, w=1.0))
        self.assertEqual((r['on'], r['off'], r['active_now']), (True, True, False))
        r = self.st(_tick(1.0, mask=1), _tick(2.0, mask=0, blend=0.0, w=0.0))           # tick mask=../0x0 ends melee
        self.assertEqual((r['on'], r['off'], r['bad']), (True, True, []))

    def test_weight_threshold(self):
        ok = self.st(_mask(1.0, True), _tick(2.0, 1.0), _tick(3.0, 0.95), _gate(3.5, w=0.90))
        self.assertEqual((ok['min_w'], ok['bad'], ok['samples']), (0.90, [], 3))
        bad = self.st(_mask(1.0, True), _tick(2.0, 1.0), _tick(3.0, 0.0))
        self.assertEqual(bad['bad'], [('23:00:03.000', 0.0)])
        self.assertEqual(bad['min_w'], 0.0)
        self.assertEqual(len(self.st(_mask(1.0, True), _tick(2.0, 0.89), w_min=0.8)['bad']), 0)
        gate = self.st(_gate(1.0, w=0.0), _gate(2.0, w=0.0))                            # pre-0.6.2: gate closed, w -> 0
        self.assertEqual(len(gate['bad']), 1)                                          # first line is inside the blend

    def test_blend_tolerance_and_carried_state(self):
        r = self.st(_mask(1.0, True, w=0.0), _tick(1.1, 0.0), _tick(1.29, 0.2), _tick(1.5, 1.0))
        self.assertEqual((r['bad'], r['samples'], r['min_w']), ([], 1, 1.0))           # first 0.3 s are the mask blend
        r = self.st(_tick(1.31, 0.2), _mask(1.0, True))                                 # sorted by time, not by order
        self.assertEqual(len(r['bad']), 1)
        r = self.st(_tick(1.0, 0.0), active=True)                                       # carried from the previous phase
        self.assertEqual((r['on'], len(r['bad'])), (False, 1))
        self.assertEqual(self.st(_tick(1.0, 0.0), active=False)['bad'], [])             # not masked: w is not judged
        late = self.st(_mask(1.0, True), _mask(2.0, False, 0, 1, w=0.0), _tick(3.0, 0.0, mask=0))
        self.assertEqual(late['bad'], [])                                               # off line + later lines unmasked
        self.assertEqual(self.st(_mask(1.0, True), _tick(1.2, 0.0), blend_s=0.1)['bad'][0][1], 0.0)

    def test_pos_lines_are_not_the_mm_weight(self):
        pos = [_pos(2.0, weapon=0), _pos(2.5, weapon=1)]                                # NightPos w= is the weapon flag
        r = self.st(_mask(1.0, True), _tick(2.0, 1.0), pos=pos)
        self.assertEqual((r['bad'], r['pos_n'], r['mounted']), ([], 2, False))
        self.assertTrue(self.st(_mask(1.0, True), pos=[_pos(2.0, mounted=1)])['mounted'])
        self.assertEqual(gn.melee_state(None, '', False)['pos_n'], 0)


def _ap(kind, rest, ts='23:00:01.000'):
    return f'[2026-10-07 {ts}] [info] [1] [UTPP] UTPP CET [utpp_probe] aimprobe {kind} {rest}'


AP_SAMPLE = 'n=3 cam=(1,2,3) fpp=(1,2,3.1) muzzle=(1,2.1,3) aim=1'
AP_SHOT = ('n=1 origin=(10.5,20.25,30) dir=(0.1,0.2,-0.97) dCam=0.45 dFpp=0.62 dMuzzle=0.03 angCam=1.5 angFpp=2.5 '
           'hit=(11,22,33) src=muzzle')
AP_SUMMARY = ('samples=12 shots=2 originNearest=muzzle meanDCam=0.45 meanDFpp=0.62 meanDMuzzle=0.03 meanAngCam=1.5 '
              'meanAngFpp=2.5')


class AimProbeStateTests(unittest.TestCase):
    def test_parses_samples_shots_summary(self):
        text = '\n'.join([_ap('sample', AP_SAMPLE)] * 5 + [_ap('shot', AP_SHOT), _ap('shot', AP_SHOT.replace('n=1', 'n=2')
                         .replace('hit=(11,22,33)', 'hit=none')), _ap('summary', AP_SUMMARY)])
        st = gn.aimprobe_state(text)
        self.assertEqual(st['samples'], 5)
        self.assertEqual(len(st['shots']), 2)
        s0, s1 = st['shots']
        self.assertEqual((s0['n'], s0['origin'], s0['dir'], s0['hit'], s0['src']),
                         (1, (10.5, 20.25, 30.0), (0.1, 0.2, -0.97), (11.0, 22.0, 33.0), 'muzzle'))
        self.assertEqual((s0['dCam'], s0['dFpp'], s0['dMuzzle'], s0['angCam'], s0['angFpp']), (0.45, 0.62, 0.03, 1.5, 2.5))
        self.assertEqual((s1['n'], s1['hit']), (2, None))                              # hit=none
        self.assertEqual(st['summary'], {'samples': 12, 'shots': 2, 'originNearest': 'muzzle', 'meanDCam': 0.45,
                                         'meanDFpp': 0.62, 'meanDMuzzle': 0.03, 'meanAngCam': 1.5, 'meanAngFpp': 2.5})
        self.assertFalse(st['tppDuringAim'])
        self.assertEqual(gn.aimprobe_fails(dict(st, samples=4)), [])
        json.dumps(st)                                                                  # cycle.json serialisable

    def test_nan_inf_and_missing_values(self):
        shot = _ap('shot', 'n=1 origin=(nan,2,3) dir=(0,0,1) dCam=nan dFpp=inf dMuzzle=-inf angCam=x hit=none')
        summ = _ap('summary', 'samples=3 shots=1 originNearest=unknown meanDCam=nan meanDFpp=0.5')
        st = gn.aimprobe_state(shot + '\n' + summ)
        s = st['shots'][0]
        self.assertIsNone(s['origin'])                                                  # a nan component
        self.assertEqual(s['dir'], (0.0, 0.0, 1.0))
        self.assertEqual((s['dCam'], s['dFpp'], s['dMuzzle'], s['angCam'], s['angFpp'], s['hit'], s['src']),
                         (None,) * 7)
        self.assertEqual((st['summary']['meanDCam'], st['summary']['meanDFpp'], st['summary']['meanDMuzzle'],
                          st['summary']['originNearest']), (None, 0.5, None, 'unknown'))
        self.assertEqual(st['summary']['shots'], 1)
        json.dumps(st, allow_nan=False)                                                 # strict JSON: no NaN anywhere

    def test_missing_parts(self):
        self.assertEqual(gn.aimprobe_state(''), {'samples': 0, 'shots': [], 'summary': None, 'tppDuringAim': False})
        self.assertEqual(gn.aimprobe_state(None)['samples'], 0)
        self.assertEqual(gn.aimprobe_fails(gn.aimprobe_state('')), ['no samples', 'no shots (no "aimprobe shot" line)',
                                                                    'no summary (no "aimprobe summary" line)'])
        st = gn.aimprobe_state('\n'.join([_ap('sample', AP_SAMPLE)] * 3 + [_ap('shot', AP_SHOT)]))
        fails = gn.aimprobe_fails(st)
        self.assertEqual(len(fails), 2)
        self.assertIn('only 3 aimprobe samples', fails[0])
        self.assertIn('no summary', fails[1])
        st = gn.aimprobe_state('\n'.join([_ap('sample', AP_SAMPLE)] * 6 + [_ap('summary', AP_SUMMARY)]))
        self.assertEqual(gn.aimprobe_fails(st), ['no shots (no "aimprobe shot" line)'])
        st = gn.aimprobe_state('\n'.join([_ap('sample', AP_SAMPLE)] * 6 + [_ap('shot', 'n=1')]))   # a bare shot line
        self.assertEqual(st['shots'][0]['n'], 1)
        self.assertIsNone(st['shots'][0]['origin'])

    def test_other_utpp_lines_and_wrong_tag_are_ignored(self):
        text = '\n'.join([_trace('23:00:01.000', 'run'), '[utpp_probe] aimprobe on', '[x] aimprobe sample n=1',
                          '[utpp_probe] aimprobes sample', _ap('sample', AP_SAMPLE)])
        self.assertEqual(gn.aimprobe_state(text)['samples'], 1)

    def test_tpp_during_aim(self):
        mask = _mask(1.0, True, reasons=2)
        self.assertTrue(gn.aimprobe_state(mask)['tppDuringAim'])                        # MMMASK on reasons=0x2
        self.assertFalse(gn.aimprobe_state(_mask(1.0, True, reasons=1))['tppDuringAim'])    # melee only
        self.assertFalse(gn.aimprobe_state(_mask(1.0, False, reasons=2, was=2))['tppDuringAim'])   # off event
        gate = '[2026-10-07 23:00:01.000] [info] [1] [UTPP] UTPP MMGATE w=1.00 upperBody=6 foo=1 tpp=1'
        self.assertTrue(gn.aimprobe_state(gate)['tppDuringAim'])
        self.assertFalse(gn.aimprobe_state(gate.replace('tpp=1', 'tpp=0'))['tppDuringAim'])
        self.assertFalse(gn.aimprobe_state(gate.replace('upperBody=6', 'upperBody=0'))['tppDuringAim'])

    def test_commands(self):
        for c in (gn.aiming_cmd('tpp'), gn.aiming_cmd('fpp'), gn.aimprobe_cmd(True), gn.aimprobe_cmd(False)):
            self.assertNotIn(chr(10), c)
            self.assertTrue(all(32 <= ord(ch) < 127 for ch in c))
            self.assertTrue(c.startswith('local ok,e=pcall(function()'))
        self.assertIn('GetMod("utpp_core").Set("perspective.aiming","tpp",{noSave=true})', gn.aiming_cmd('tpp'))
        self.assertIn('"fpp",{noSave=true}', gn.aiming_cmd('fpp'))
        self.assertIn("[night] AIMPROBE aiming=tpp ok='..tostring(ok)..' '..tostring(e)", gn.aiming_cmd('tpp'))
        self.assertIn("[night] AIMPROBE aiming=fpp ok=", gn.aiming_cmd('fpp'))
        self.assertIn('GetMod("utpp_probe").AimProbe(true)', gn.aimprobe_cmd(True))
        self.assertIn('GetMod("utpp_probe").AimProbe(false)', gn.aimprobe_cmd(False))
        self.assertIn('[night] AIMPROBE on ok=', gn.aimprobe_cmd(True))
        self.assertIn('[night] AIMPROBE off ok=', gn.aimprobe_cmd(False))
        self.assertEqual(gn.night_echo('[night] AIMPROBE on ok=true nil', 'AIMPROBE on'), (True, 'AIMPROBE on ok=true nil'))
        self.assertEqual(gn.night_echo('[night] AIMPROBE off ok=true nil', 'AIMPROBE on'), (None, ''))   # distinct echoes
        self.assertEqual(gn.night_echo('[night] AIMPROBE aiming=fpp ok=false x', 'AIMPROBE aiming=tpp'), (None, ''))



class _KeyApi(FakeApi):
    """FakeApi + the raw key down/up calls used by Night.hold_key (Win32Api._scan/_send)."""

    @staticmethod
    def _scan(scan, up):
        return (scan, up)

    def _send(self, evs):
        self.calls.append(('raw', tuple(evs), self.fg))


class _G:
    """Guard stand-in for macro tests: no sleeping; all input lands in api.calls."""
    def __init__(self):
        self.api = _KeyApi()

    def wait(self, s):
        night.time.sleep(s)

    def check(self):
        pass


class _Pad:
    def release(self):
        pass


def _trace(ts, loco, clip='Stand_Idle', v=0.0):
    return (f'[2026-10-07 {ts}] [info] [1] [UTPP] UTPP MMTRACE tick clip={clip} f=1 cost=0 |v|={v:.2f} rate=1 w=1.00 '
            f'loco={loco}')


class MacroTests(unittest.TestCase):
    """weapon/vehicle/crouch flows with stubbed play/poll/console; the UTPP log is a temp file; fake clock."""

    def setUp(self):
        self._time = night.time
        night.time = FakeTime()
        fd, self.log = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.n = EnsureClosedTests.make(self, FakeApi(), None)
        self.n.utpp_log = self.log
        self.n.scen_log_off = 0
        self.n.cycle = {'name': 't', 'steps': []}
        self.played, self.typed, self.drives = [], [], []
        self.n.type_console = lambda g, cmds, verify_log=None, fast=False: self.typed.extend(cmds) or {}
        self.n.recover = lambda *a, **k: {'suspect': False}

    def tearDown(self):
        night.time = self._time
        os.remove(self.log)

    def exit_lines(self):
        self.cet('[utpp_vehiclecam] t=9.00 keepTpp ride -> exit')
        self.cet("[utpp_vehiclecam] t=9.50 exit pull: end (utpp_camera's orbit)")
        self.cet('[utpp_vehiclecam] t=9.60 keepTpp exit -> off')

    def append(self, text):
        with open(self.log, 'a', encoding='utf-8') as f:
            f.write(text + chr(10))

    def cet(self, text):
        self.append('[2026-10-07 23:00:00.000] [info] [1] [UTPP] UTPP CET ' + text)

    def test_weapon_pass_and_fail(self):
        polls = iter([{'weapon': True}, {'weapon': False}])          # confirmed at the first poll
        self.n.poll_pos = lambda g, pad=None, timeout=None: next(polls)
        self.n.play = lambda pad, g, name, plan, sc=None: self.played.append(name) or {'steps': []}
        self.n.clear_path = lambda pad, g, plan, where: self.played.append('clear:' + where) or {'result': 'clear'}
        r = self.n.weapon_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.played, ['weapon_draw', 'clear:weapon_aim', 'weapon_aim', 'weapon_holster',
                                       'clear:weapon_walk', 'weapon_walk'])
        self.played.clear()
        polls = iter([{'weapon': True}] + [{'weapon': True}] * 6)   # holster never confirmed: 2 x 3 polls
        self.n.poll_pos = lambda g, pad=None, timeout=None: next(polls)
        r = self.n.weapon_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(self.played.count('weapon_holster'), 2)              # one retry
        self.assertIn('holster not confirmed', r['fails'][0])

    def test_weapon_fire_macro_fires_and_reloads_between_draw_and_holster(self):
        polls = iter([{'weapon': True}, {'weapon': False}])
        self.n.poll_pos = lambda g, pad=None, timeout=None: next(polls)
        self.n.play = lambda pad, g, name, plan, sc=None: self.played.append(name) or {'steps': []}
        self.n.clear_path = lambda pad, g, plan, where: self.played.append('clear:' + where) or {'result': 'clear'}
        r = self.n.weapon_macro(_Pad(), _G(), {'runs': []}, fire=True)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.played, ['weapon_draw', 'weapon_fire', 'weapon_reload', 'weapon_holster',
                                       'clear:weapon_walk', 'weapon_walk'])
        self.assertEqual(self.played[1:3] + self.played[3:4], gn.WEAPON_FIRE_PARTS[1:4])
        self.assertIn({'reload_episode_s': None}, r['checks'])        # no reload diagnostics in the log: report only
        # a draw that is not there (w=0) is a failure and nothing is fired
        self.played.clear()
        polls = iter([{'weapon': False}] * 3 + [{'weapon': False}])
        self.n.poll_pos = lambda g, pad=None, timeout=None: next(polls)
        r = self.n.weapon_macro(_Pad(), _G(), {'runs': []}, fire=True)
        self.assertNotIn('weapon_fire', self.played)
        self.assertIn('draw not confirmed', r['fails'][0])

    def test_clear_path_turns_away_from_an_obstacle(self):
        # probe 1: V moves 0.05 m (bush, Female V 17:37) -> camera +90 deg; probe 2: 1.2 m -> clear
        pts = iter([{'p': (0, 0, 0), 'cf': (0, 1)}, {'p': (0, 0.05, 0), 'cf': (0, 1)},
                    {'p': (0, 0.05, 0), 'cf': (1, 0)}, {'p': (1.2, 0.05, 0), 'cf': (1, 0)}])
        self.n.poll_pos = lambda g, pad=None, timeout=None: next(pts)
        self.n.play = lambda pad, g, name, plan, sc=None: self.played.append(name) or {'steps': []}
        self.n.scenarios = {}      # no route estimate -> the short 0.6 s probe (weapon_walk is a long route: LookAheadTests)
        turns = []
        self.n.camera_align = lambda pad, g, plan, known, target, ep, tol=None, why='': turns.append(target) or known
        r = self.n.clear_path(_Pad(), _G(), {'runs': []}, 'weapon_walk')
        self.assertEqual(r['result'], 'clear')
        self.assertEqual([p['moved_m'] for p in r['probes']], [0.05, 1.2])
        self.assertEqual(len(turns), 1)
        self.assertEqual(self.played, ['clear_probe', 'clear_probe'])

    def _melee_stub(self, script, fists_echo='true', off_echo='true'):
        """play() / console_verified() stubs for melee_macro. `script(name)` gives the log lines for a played scenario
        name or for the pseudo names 'fists_cmd' / 'fists_off_cmd' (the console commands); an echo value of None =
        the console command was lost (no echo)."""
        self.played.clear()
        self.order, self.cv = [], []

        def play(pad, g, name, plan, sc=None):
            self.played.append(name)
            self.order.append(name)
            for line in script(name):
                self.append(line)
            return {'steps': []}

        def console_verified(pad, g, cmd, echo, label, timeout=3.0):
            self.cv.append((label, cmd))
            name, tag, val = (('fists_cmd', 'FISTS', fists_echo) if cmd == gn.FISTS_CMD
                              else ('fists_off_cmd', 'FISTS_OFF', off_echo))
            self.order.append(name)
            for line in script(name):
                self.append(line)
            if val is None:
                return None
            line = self.cet(f'[night] {tag} ok={val} eqsystem') or ''
            return open(self.log, encoding='utf-8').read()
        self.n.play = play
        self.n.console_verified = console_verified
        self.n.clear_path = lambda pad, g, plan, where: self.order.append('clear:' + where) or {'result': 'clear'}

    @staticmethod
    def _melee_ok(name, w_punch=1.00):
        return {'melee_draw': [_mask(1.0, True), _gate(1.05)],
                'melee_punch': [_tick(2.0), _tick(3.0)],
                'melee_punch_walk': [_tick(4.0, w_punch), _gate(4.5, w=w_punch)],
                'melee_block': [_tick(6.0)],
                'fists_off_cmd': [_mask(7.0, False, 0, 1)],
                'melee_holster': [_mask(8.0, False, 0, 1)]}.get(name, [])

    def test_melee_pass(self):
        self._melee_stub(self._melee_ok)
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.played, ['melee_draw', 'melee_punch', 'melee_punch_walk', 'melee_block'])  # no Y needed
        # the walking part gets a clear-path probe first (2026-11-12: V walked into an obstacle); standing parts do not
        self.assertEqual(self.order, ['fists_cmd', 'melee_draw', 'melee_punch', 'clear:melee_punch_walk',
                                      'melee_punch_walk', 'melee_block', 'fists_off_cmd'])
        self.assertEqual([c['phase'] for c in r['checks']], ['fists_cmd', 'melee_draw', 'melee_punch',
                         'melee_punch_walk', 'melee_block', 'fists_off_cmd', 'melee_fists_off'])
        self.assertEqual([c for _, c in self.cv], [gn.FISTS_CMD, gn.FISTS_OFF_CMD])
        self.assertEqual(r['checks'][0]['ok'], True)
        self.assertIn('FISTS ok=true', r['checks'][0]['echo'])
        self.assertEqual((r['mask_seen'], r['min_w'], r['fails'], r['warnings']), (True, 1.0, [], []))
        self.assertTrue(all(c['mask_seen'] for c in r['checks'] if 'mask_seen' in c))      # the mask is carried over
        self.assertTrue(r['checks'][-1]['mask_off_ok'])
        self.assertTrue(r['checks'][1]['mask_on'])
        json.dumps(r)                                                                  # cycle.json serialisable

    def test_melee_fists_cmd_goes_first_and_waits(self):
        self._melee_stub(self._melee_ok)
        t = {}
        orig = self.n.play

        def play(pad, g, name, plan, sc=None):
            t.setdefault(name, night.time.monotonic())
            return orig(pad, g, name, plan, sc)
        cv0 = self.n.console_verified

        def cv(*a, **k):
            t.setdefault('cmd', night.time.monotonic())
            return cv0(*a, **k)
        self.n.play, self.n.console_verified = play, cv
        self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertGreaterEqual(t['melee_draw'] - t['cmd'], 1.0)                       # ~1 s between fists and RT tap

    def test_melee_fists_commands(self):
        for c in (gn.FISTS_CMD, gn.FISTS_OFF_CMD):
            self.assertNotIn(chr(10), c)
            self.assertTrue(all(32 <= ord(ch) < 127 for ch in c))
            self.assertTrue(c.startswith('local ok,e=pcall(function()'))
            self.assertIn("Game.GetPlayer()", c)
        self.assertIn('RequestEquipFists()', gn.FISTS_CMD)
        self.assertIn('GetQuickSlotsManager()', gn.FISTS_CMD)
        self.assertIn("Game.UTPPLogLine('[night] FISTS ok='..tostring(ok)..' '..tostring(e))", gn.FISTS_CMD)
        self.assertIn("Enum.new('EquipmentManipulationAction','RequestFists')", gn.FISTS_CMD)       # the fallback
        self.assertIn('EquipmentSystemWeaponManipulationRequest.new()', gn.FISTS_CMD)
        self.assertIn("Get('EquipmentSystem'):QueueRequest(r)", gn.FISTS_CMD)
        self.assertIn("Enum.new('EquipmentManipulationAction','UnequipWeapon')", gn.FISTS_OFF_CMD)
        self.assertIn("Get('EquipmentSystem'):QueueRequest(r)", gn.FISTS_OFF_CMD)
        self.assertIn("[night] FISTS_OFF ok=", gn.FISTS_OFF_CMD)
        self.assertNotIn('[night] FISTS ok=', gn.FISTS_OFF_CMD)                         # distinct echoes
        self.assertEqual(gn.night_echo('x [night] FISTS ok=true quickslots', 'FISTS'),
                         (True, 'FISTS ok=true quickslots'))
        self.assertEqual(gn.night_echo('[night] FISTS ok=false boom', 'FISTS')[0], False)
        self.assertEqual(gn.night_echo('[night] FISTS_OFF ok=true eqsystem', 'FISTS'), (None, ''))
        self.assertEqual(gn.night_echo(None, 'FISTS'), (None, ''))

    def test_melee_fists_cmd_failure_or_lost_echo_fails(self):
        self._melee_stub(self._melee_ok, fists_echo='false')
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('fists_cmd: the command failed', r['fails'][0])
        self.assertEqual(r['checks'][0]['ok'], False)
        self._melee_stub(self._melee_ok, fists_echo=None)
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('no "[night] FISTS ok=" echo', r['fails'][0])
        self.assertIsNone(r['checks'][0]['ok'])

    def test_melee_double_y_is_the_second_try(self):
        script = lambda n: [] if n == 'fists_off_cmd' else self._melee_ok(n)           # UnequipWeapon does nothing
        self._melee_stub(script)
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.played[-1], 'melee_holster')
        self.assertFalse(r['checks'][-2]['mask_off_ok'])
        self.assertTrue(r['checks'][-1]['mask_off_ok'])
        self.assertEqual(r['warnings'], ['fists_off_cmd did not clear melee; the Y double tap did'])
        self._melee_stub(lambda n: [] if n in ('fists_off_cmd', 'melee_holster') else self._melee_ok(n))
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('still active', r['fails'][0])
        self.assertIn('FISTS_OFF and the Y double tap', r['fails'][0])

    def test_melee_no_cet_fists_opt_out(self):
        self.n.args.cet_fists = False
        self._melee_stub(self._melee_ok)
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(self.cv, [])                                                  # no console command at all
        self.assertEqual(self.played, ['melee_draw', 'melee_punch', 'melee_punch_walk', 'melee_block', 'melee_holster'])
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertNotIn('fists_cmd', [c['phase'] for c in r['checks']])
        self.assertTrue(night.build_parser().parse_args(['--no-cet-fists']).cet_fists is False)
        self.assertTrue(night.build_parser().parse_args([]).cet_fists)

    def test_melee_weight_drop_fails(self):
        self._melee_stub(lambda n: self._melee_ok(n, w_punch=0.0))
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(len(r['fails']), 1, r['fails'])
        self.assertIn('melee_punch_walk: MM weight 0.00', r['fails'][0])
        self.assertEqual(r['min_w'], 0.0)

    def test_melee_blend_is_tolerated(self):
        self._melee_stub(lambda n: [_mask(1.0, True, w=0.0), _tick(1.10, 0.0), _tick(1.28, 0.1)] if n == 'melee_draw'
                         else self._melee_ok(n))
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)

    def test_melee_no_mask_fails(self):
        self._melee_stub(lambda n: [])                                                  # nothing in the log at all
        t0 = night.time.monotonic()
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(len(r['fails']), 1)                                           # the holster is vacuously fine
        self.assertIn('melee_draw: no "MMMASK on reasons=0x1"', r['fails'][0])
        self.assertFalse(r['mask_seen'])
        self.assertEqual(len(r['warnings']), 3)                                        # punch / walk / block unverified
        self.assertGreaterEqual(night.time.monotonic() - t0, self.n.args.weapon_window)  # waited the whole window

    def test_melee_draw_by_gate_line_only(self):
        self._melee_stub(lambda n: [_gate(1.0, melee=0x2, mask=0)] if n == 'melee_draw' else self._melee_ok(n))
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)

    def _aim_stub(self, hold_lines=None, echoes=None, weapon=True, play_error=None, aiming_before='fpp'):
        """play() / console_verified() / poll_pos() stubs for aim_probe_macro. `echoes` maps the echo tag ('aiming=tpp',
        'on', 'off', 'aiming=fpp') to 'true' / 'false' / None (= console command lost, no echo). `aiming_before` is what the
        settings read answers for perspective.aiming (None = no echo: the value cannot be read)."""
        self.played.clear()
        self.order = []
        ech = {'aiming=tpp': 'true', 'on': 'true', 'off': 'true', 'aiming=fpp': 'true'}
        ech.update(echoes or {})
        cmds = {gn.aiming_cmd('tpp'): 'aiming=tpp', gn.aimprobe_cmd(True): 'on', gn.aimprobe_cmd(False): 'off',
                gn.aiming_cmd('fpp'): 'aiming=fpp'}
        self.get_cmds = []
        self.n.poll_pos = lambda g, pad=None, timeout=None: {'weapon': weapon}

        def play(pad, g, name, plan, sc=None):
            self.played.append(name)
            self.order.append(name)
            if name == 'aim_probe_hold':
                for line in hold_lines or []:
                    self.append(line)
                if play_error:
                    raise play_error
            return {'steps': []}

        def console_verified(pad, g, cmd, echo, label, timeout=3.0):
            if cmd == gs.get_cmd(['perspective.aiming']):                               # the settings read (not a probe command)
                self.get_cmds.append(cmd)
                if aiming_before is None:
                    return None
                self.cet(f'[night] SETGET ok=true perspective.aiming=string:{aiming_before}')
                return open(self.log, encoding='utf-8').read()
            key = cmds[cmd]
            self.order.append(key)
            val = ech[key]
            if key == 'on':                                                             # the probe starts sampling
                self.append(_ap('sample', 'n=0 pre'))
            if val is None:
                return None
            self.cet(f'[night] AIMPROBE {key} ok={val} nil')
            return open(self.log, encoding='utf-8').read()
        self.n.play = play
        self.n.console_verified = console_verified

    @staticmethod
    def _aim_ok(samples=6, shots=2, summary=True):
        lines = [_ap('sample', AP_SAMPLE) for _ in range(samples)]
        lines += [_ap('shot', AP_SHOT.replace('n=1', f'n={i + 1}')) for i in range(shots)]
        if summary:
            lines.append(_ap('summary', AP_SUMMARY))
        return lines + [_mask(2.0, True, reasons=2)]

    def test_aim_probe_pass(self):
        self._aim_stub(self._aim_ok())
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(r['fails'], [])
        self.assertEqual(self.order, ['aiming=tpp', 'on', 'aim_probe_hold', 'off', 'aiming=fpp'])
        self.assertEqual(self.played, ['aim_probe_hold'])                               # weapon already in hand
        self.assertEqual([c['phase'] for c in r['checks']], ['aiming_tpp', 'aimprobe_on', 'aimprobe_off', 'aiming_fpp'])
        self.assertTrue(all(c['ok'] is True for c in r['checks']))
        ap = r['aimprobe']
        self.assertEqual((ap['samples'], len(ap['shots']), ap['tppDuringAim']), (7, 2, True))   # incl. the pre sample
        self.assertEqual(ap['summary']['originNearest'], 'muzzle')
        json.dumps(r, allow_nan=False)                                                  # cycle.json serialisable

    def test_aim_probe_log_window_ends_at_the_off_echo(self):
        self._aim_stub(self._aim_ok(samples=1, shots=1, summary=False))
        orig = self.n.console_verified

        def cv(pad, g, cmd, echo, label, timeout=3.0):
            out = orig(pad, g, cmd, echo, label, timeout)
            if cmd == gn.aimprobe_cmd(False):                                           # lines after the off echo
                for line in self._aim_ok(samples=5, shots=2):
                    self.append(line)
            return out
        self.n.console_verified = cv
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(r['aimprobe']['samples'], 2)                                   # pre + 1, the late ones ignored
        self.assertIn('only 2 aimprobe samples', r['fails'][0])
        self.assertIsNone(r['aimprobe']['summary'])

    def test_aim_probe_draws_when_no_weapon(self):
        self._aim_stub(self._aim_ok(), weapon=False)
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.order[:2], ['weapon_draw', 'aiming=tpp'])
        self.assertEqual(self.played, ['weapon_draw', 'aim_probe_hold'])
        self.assertEqual(r['checks'][0]['phase'], 'weapon_draw_first')
        self._aim_stub(self._aim_ok(), weapon=None)                                      # unknown flag: no draw
        self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(self.played, ['aim_probe_hold'])

    def test_aim_probe_lost_echo_fails(self):
        for tag, phase in (('on', 'aimprobe_on'), ('aiming=tpp', 'aiming_tpp'), ('off', 'aimprobe_off'),
                           ('aiming=fpp', 'aiming_fpp')):
            self._aim_stub(self._aim_ok(), echoes={tag: None})
            r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
            self.assertEqual(r['verdict'], 'FAIL', tag)
            self.assertEqual(len(r['fails']), 1, r['fails'])
            self.assertIn(f'{phase}: no "[night]', r['fails'][0])
            self.assertIn('console command lost', r['fails'][0])
            self.assertIsNone(next(c['ok'] for c in r['checks'] if c['phase'] == phase))
            self.assertEqual(self.order[-2:], ['off', 'aiming=fpp'])                    # the restore is always sent
        self._aim_stub(self._aim_ok(), echoes={'aiming=tpp': 'false'})
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('aiming_tpp: the command failed', r['fails'][0])

    def test_aim_probe_no_shots_no_samples_no_summary(self):
        self._aim_stub(self._aim_ok(shots=0))
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual((r['verdict'], r['fails']), ('FAIL', ['no shots (no "aimprobe shot" line)']))
        self._aim_stub(self._aim_ok(summary=False))
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual((r['verdict'], r['fails']), ('FAIL', ['no summary (no "aimprobe summary" line)']))
        self._aim_stub([])                                                               # AimProbe printed nothing
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(len(r['fails']), 3, r['fails'])                                 # pre sample only: < 4 samples
        self.assertIn('only 1 aimprobe samples', r['fails'][0])
        self.assertFalse(r['aimprobe']['tppDuringAim'])

    def test_aim_probe_restores_fpp_on_failure(self):
        for err in (night.ScenarioTimeout('timeout'), night.CycleError('boom'), RuntimeError('x')):
            self._aim_stub(self._aim_ok(), play_error=err)
            with self.assertRaises(type(err)):
                self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
            self.assertEqual(self.order, ['aiming=tpp', 'on', 'aim_probe_hold', 'off', 'aiming=fpp'], type(err))
        # a failing AimProbe(false) command must not stop the aiming=fpp restore
        self._aim_stub(self._aim_ok(), play_error=night.ScenarioTimeout('t'))
        orig = self.n.console_verified

        def cv(pad, g, cmd, echo, label, timeout=3.0):
            if cmd == gn.aimprobe_cmd(False):
                self.order.append('off')
                raise RuntimeError('console died')
            return orig(pad, g, cmd, echo, label, timeout)
        self.n.console_verified = cv
        with self.assertRaises(night.ScenarioTimeout):
            self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(self.order[-2:], ['off', 'aiming=fpp'])

    def test_aim_probe_restores_the_value_it_read_not_a_fixed_one(self):
        self._aim_stub(self._aim_ok(), aiming_before='tpp')
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.order, ['aiming=tpp', 'on', 'aim_probe_hold', 'off', 'aiming=tpp'])   # put back to tpp, not fpp
        self.assertEqual(r['aiming_restore'], {'previous': 'tpp', 'restore_to': 'tpp'})
        self.assertEqual(len(self.get_cmds), 1)
        self.assertEqual(self.n.settings.open_entries(), [])                              # the restore echo was seen
        self.assertEqual([e['state'] for e in self.n.settings.record()], ['restored'])
        self._aim_stub(self._aim_ok(), aiming_before='fpp')
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(self.order[-1], 'aiming=fpp')

    def test_aim_probe_unreadable_aiming_falls_back_to_fpp_and_says_so(self):
        self._aim_stub(self._aim_ok(), aiming_before=None)
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['aiming_restore'], {'previous': None, 'restore_to': 'fpp'})
        self.assertEqual(self.order[-1], 'aiming=fpp')
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual([(e['volatile'], e['state']) for e in self.n.settings.record()], [(True, 'restored')])   # never in the pending file

    def test_aim_probe_lost_restore_stays_open_and_is_sent_again_before_exit(self):
        self._aim_stub(self._aim_ok(), aiming_before='tpp', echoes={'aiming=tpp': None})
        r = self.n.aim_probe_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual([e['key'] for e in self.n.settings.open_entries()], ['perspective.aiming'])
        sent = []

        def cv(pad, g, cmd, echo, label, timeout=3.0):
            sent.append(cmd)
            self.cet('[night] SETSET ok=true all')
            return open(self.log, encoding='utf-8').read()
        self.n.console_verified = cv
        res = {}
        self.n.send_final_commands(_Pad(), _G(), res)
        self.assertEqual(sent, [gs.set_cmd([('perspective.aiming', 'tpp')])])
        self.assertEqual(res['restored_settings']['restored'], 1)
        self.assertEqual(self.n.settings.open_entries(), [])

    def test_melee_fists_are_in_the_ledger_and_put_away_even_when_the_macro_dies(self):
        self._melee_stub(self._melee_ok)
        r = self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual([(e['owner'], e['state']) for e in self.n.settings.record()], [('melee_fists', 'restored')])
        self.n.settings.entries = []
        self._melee_stub(self._melee_ok)
        orig = self.n.play

        def play(pad, g, name, plan, sc=None):
            if name == 'melee_punch':
                raise night.ScenarioTimeout('timeout')
            return orig(pad, g, name, plan, sc)
        self.n.play = play
        with self.assertRaises(night.ScenarioTimeout):
            self.n.melee_macro(_Pad(), _G(), {'runs': []})
        self.assertEqual([e['owner'] for e in self.n.settings.open_entries()], ['melee_fists'])
        self.cv.clear()
        res = {}
        self.n.send_final_commands(_Pad(), _G(), res)                                    # before ExitGame
        self.assertEqual([c for _, c in self.cv], [gn.FISTS_OFF_CMD])
        self.assertEqual(res['restored_settings']['restored'], 1)

    def test_aim_probe_fails_land_in_summary_verdicts(self):
        self._aim_stub(self._aim_ok())
        rec = {'name': 'aim_probe', 'run': 1}
        rec.update(self.n.aim_probe_macro(_Pad(), _G(), {'runs': []}))
        json.dumps(rec, allow_nan=False)
        self.assertEqual((rec['verdict'], rec['aimprobe']['summary']['meanDCam']), ('PASS', 0.45))


    GAME_SPEEDS = {'forward': 3.8, 'sprint': 8.0, 'side': 1.5, 'back': 0.3}   # stub game: backwards is slow
    GAME_CAM_RATE = 150.0                                                      # deg/s, differs from the 120 guess

    def _vehicle(self, p0, cf0, car, car_right, miss=lambda k: False, mount_on='x', spawn_drop=0, exit_on='f_hold2',
                 seat_delay=13.0, drive_m=6.0, exit_ready_after=0.0, exit_silent=False, unmount_ok=False,
                 car_moves_s=0.0, car_speed=3.0, cet_mount_on=None):
        """Stub game: V moves along the camera-relative stick with GAME_SPEEDS, the right stick yaws the camera at
        GAME_CAM_RATE; poll k (0-based) is lost when miss(k); the first `spawn_drop` spawn commands are lost."""
        n = self.n
        n.args.vehicle_repeats = 1
        n.args.vehicle_spawn = 'codeware'          # the stub game implements the Codeware spawn
        world = {'p': p0, 'cf': cf0, 'polls': 0, 'spawns': 0, 'car': car, 'mount_t': None, 'appr0': None,
                 't0': night.time.monotonic()}

        def car_now():  # the car drives (+x, car_speed m/s) until car_moves_s after the start, then parks
            dt = min(night.time.monotonic() - world['t0'], car_moves_s)
            return (world['car'][0] + car_speed * dt, world['car'][1])

        def seated():
            return world['mount_t'] is not None and night.time.monotonic() >= world['mount_t'] + seat_delay

        def mount_line():
            world['mount_t'] = night.time.monotonic()
            self.cet('[utpp_vehiclecam] t=1.00 keepTpp off -> mount')
            self.cet('[utpp_vehiclecam] t=1.00 keepTpp mount -> ride')
        self.mount_line, self.seated = mount_line, seated

        def exit_ok():
            return seated() and night.time.monotonic() >= world['mount_t'] + seat_delay + exit_ready_after

        def do_exit():
            if exit_silent:              # the game exits but utpp_vehiclecam logs nothing: only NightPos m=0 tells
                world['mount_t'] = None
            else:
                self.exit_lines()

        def console(pad, g, cmds, settle=None):
            for c in cmds:
                self.typed.append(c)
                if c == gn.NIGHTPOS_CMD:
                    world['polls'] += 1
                    k = world['polls'] - 1 - world['appr0'] if world['appr0'] is not None else -1
                    if k < 0 or not miss(k):   # misses counted from the first approach/camera step
                        self.cet('[night] POS p=%.2f,%.2f,186.00 cf=%.3f,%.3f v=%.2f,%.2f,186.00,%.3f,%.3f src=tag w=0 m=%d'
                                 % (world['p'][0], world['p'][1], world['cf'][0], world['cf'][1], car_now()[0],
                                    car_now()[1], car_right[0], car_right[1], 1 if seated() else 0))
                elif c == gn.UNMOUNT_CMD and unmount_ok and world['mount_t'] is not None:
                    world['mount_t'] = None
                    self.cet('[night] UNMOUNT ok=true nil')
                elif c in (gn.MOUNT_CMD, gn.MOUNT_INSTANT_CMD) and cet_mount_on is not None:
                    if cet_mount_on == 'dead':        # no vehicle found by the Lua
                        self.cet('[night] MOUNT ok=false [string]:1: no vehicle')
                    else:
                        self.cet('[night] MOUNT ok=true tag instant=%s' % ('true' if c == gn.MOUNT_INSTANT_CMD else 'false'))
                        if cet_mount_on == ('instant' if c == gn.MOUNT_INSTANT_CMD else 'animated') or \
                                cet_mount_on == 'both':
                            mount_line()
                elif c == gn.SPAWN_CODEWARE_CMD:
                    world['spawns'] += 1
                    if world['spawns'] > spawn_drop:
                        self.cet('[night] SPAWN codeware ok=true userdata: 0x1')
            night.time.sleep(0.6)
            return {}
        n.console = console
        n.pos_mode = 'nightpos'

        def play(pad, g, name, plan, sc=None):
            self.played.append(name)
            if name in ('veh_approach', 'cam_turn') and world['appr0'] is None:
                world['appr0'] = world['polls']
            if name in ('veh_approach', 'cam_turn'):
                st = sc.steps[0]
                if name == 'veh_approach':
                    _, sx, sy = st.tracks['ls'].keys[1]
                    dur = st.tracks['ls'].keys[2][0] - 0.05
                    sprint = any(b.button == 'LS' for b in st.buttons)
                    v = self.GAME_SPEEDS[gn.stick_class(sx, sy, sprint)]
                    x, y = gn.dead_reckon(world['p'], world['cf'], (sx, sy), dur, v)
                    world['p'] = (x, y)
                else:
                    keys = st.tracks['rs'].keys
                    rx, dur = keys[1][1], keys[2][0] - keys[1][0]
                    world['cf'] = gn.rotate_stick(*world['cf'], rx * self.GAME_CAM_RATE * dur)
                night.time.sleep(st.duration)
            if name == 'veh_enter' and mount_on == 'x' and world['mount_t'] is None:
                mount_line()
            if name == 'veh_drive' and seated():
                world['car'] = (world['car'][0] + drive_m, world['car'][1])
                self.drives.append(night.time.monotonic() - world['mount_t'])
            if name == 'veh_exit_hold' and exit_on == 'b_hold2' and exit_ok() and sc.steps[0].buttons[0].hold >= 1.9:
                do_exit()
            return {'steps': []}
        n.play = play
        g = _G()
        down = {}

        def send(evs, api=g.api):
            api.calls.append(('raw', tuple(evs), api.fg))
            scan, up = evs[0]
            if not up:
                down[scan] = night.time.monotonic()
                return
            held = night.time.monotonic() - down.get(scan, night.time.monotonic())
            if scan != gn.SCAN_F:
                return
            mounted = gn.vehiclecam_phase(night.read_since(self.log, 0)) in ('mount', 'ride')
            if not mounted and mount_on == 'f':
                self.cet('[utpp_vehiclecam] t=2.00 handover 0.53 s after the vehicle TPP perspective')
                mount_line()
            elif mounted and exit_ok() and ((exit_on == 'f_tap' and held < 0.5) or
                                            (exit_on == 'f_hold2' and held >= 1.9) or
                                            (exit_on == 'f_hold3' and held >= 2.9)):
                do_exit()
        g.api._send = send
        t0 = night.time.monotonic()
        r = n.vehicle_macro(_Pad(), g, {'runs': []})
        return r, g, world, night.time.monotonic() - t0

    # cycle 2 geometry: car 3.6 m away, 56 deg right of the camera forward
    C2 = dict(p0=(1500.00, 2201.90), cf0=(-0.920, -0.392), car=(1496.98, 2203.88), car_right=(-0.392, 0.920))
    # cycle 3 geometry: summoned car 46 m exactly behind the camera
    C3 = dict(p0=(1498.58, 2200.57), cf0=(-0.921, -0.389), car=(1541.55, 2216.92), car_right=(-0.356, 0.935))

    def events(self, r, key):
        return [e for e in r['episodes'][0]['events'] if key in e]

    def test_vehicle_cycle2_turns_camera_then_mounts(self):
        r, g, world, _ = self._vehicle(**self.C2)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertTrue(self.events(r, 'camera'))                         # 56 deg > 35 -> camera first
        d = [e['dist_m'] for e in self.events(r, 'dist_m')]
        lon, lat = gn.car_frame(world['p'], self.C2['car'], self.C2['car_right'])
        self.assertTrue(gn.at_door(lon, lat), (lon, lat, d))
        self.assertEqual(r['episodes'][0]['mounted_by'], 'pad_x#1')
        self.assertIn(gn.DESPAWN_CMD, self.typed)

    def test_vehicle_target_behind_turns_camera_first_and_sprints(self):
        r, g, world, elapsed = self._vehicle(**self.C3)
        self.assertEqual(r['verdict'], 'PASS', r)
        cams = self.events(r, 'camera')
        self.assertTrue(cams and abs(cams[0]['bearing']) > 150, cams)     # target behind -> turn first
        runs = self.events(r, 'dist_m')
        self.assertTrue(all(gn.stick_class(*e['stick']) != 'back' for e in runs), runs)   # never walks backwards
        self.assertTrue(any(e['sprint'] for e in runs))                    # 46 m -> sprint
        self.assertLess(elapsed, gn.approach_timebox(46.0) + 30)
        self.assertGreater(self.n.cam_rate, 120.0)                         # learned towards the stub's 150 deg/s

    def test_vehicle_dead_reckoning_corrected_by_real_pos(self):
        # a lost poll (and its re-send) -> dead reckoning; the next real POS must replace the estimate
        r, g, world, _ = self._vehicle(**self.C3, miss=lambda k: k in (5, 6, 7, 11))   # 5-7: all 3 attempts lost; 11: retry ok
        self.assertEqual(r['verdict'], 'PASS', r)
        srcs = [e['src'] for e in self.events(r, 'src')]
        self.assertIn('dead-reckon (poll miss)', srcs)
        i = srcs.index('dead-reckon (poll miss)')
        self.assertIn('pos', srcs[i + 1:])
        self.assertGreater(self.n.pos_stats['miss'], 0)
        st = self.n.cet_stats['pos/nightpos']
        self.assertIn(0, st)                                               # one LOST poll (3 attempts)
        self.assertIn(2, st)                                               # one poll ok at the 2nd attempt
        self.assertNotEqual(self.n.speeds['sprint'], gn.SPEEDS['sprint'])  # learned from real displacement

    def test_vehicle_never_blocks_without_pos(self):
        r, g, world, elapsed = self._vehicle(**self.C2, miss=lambda k: k >= 1)
        srcs = [e['src'] for e in self.events(r, 'src')]
        self.assertTrue(all(s != 'pos' for s in srcs[1:]), srcs)
        self.assertLess(elapsed, gn.approach_timebox(3.6) + 40)
        self.assertIn(r['verdict'], ('PASS', 'FAIL'))

    def test_codeware_spawn_retyped_once(self):
        r, g, world, _ = self._vehicle(**self.C2, spawn_drop=1)
        self.assertEqual(self.typed.count(gn.SPAWN_CODEWARE_CMD), 2)
        ev = [e for e in r['episodes'][0]['events'] if 'spawn_log' in e][0]
        self.assertEqual(ev['typed'], 2)
        self.assertTrue(ev['spawn_log'].startswith('SPAWN codeware ok=true'))
        self.assertEqual(r['verdict'], 'PASS')

    def test_vehicle_mount_by_keyboard_f(self):
        self.n.args.mount_try_f = True
        r, g, _, _ = self._vehicle(**self.C2, mount_on='f')
        self.assertEqual(r['episodes'][0]['mounted_by'], 'key_f#3')     # 2 X presses first, then F
        self.assertIn(('raw', ((gn.SCAN_F, False),), 11), g.api.calls)

    def test_vehicle_exit_paths(self):
        for exit_on, path in (('f_hold2', 'key_f_hold2#1'), ('f_hold3', 'key_f_hold3#2'), ('b_hold2', 'pad_b_hold2#3'),
                              ('f_tap', 'key_f_tap#5')):
            with self.subTest(exit_on=exit_on):
                open(self.log, 'w').close()
                self.played.clear()
                r, g, _, _ = self._vehicle(**self.C2, exit_on=exit_on)
                ep = r['episodes'][0]
                self.assertEqual(r['verdict'], 'PASS', r)
                self.assertEqual(ep['exit']['path'], path)
                self.assertIsNotNone(ep['exit_pull_end'])
        open(self.log, 'w').close()
        self.played.clear()
        r, g, _, _ = self._vehicle(**self.C2, exit_on=None)           # cycle 4/6: nothing exits
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('exit not started', r['fails'][0])
        self.assertEqual(self.n.cycle['exits'][-1]['why'], 'end of vehicle_mount_exit')   # forced again at the end

    def test_cycle5_seated_before_drive_and_exit_retries(self):
        # seated 13 s after the mount line; the game accepts exit only 16 s after being seated (drive ~9 s + a few ignored F taps)
        r, g, world, _ = self._vehicle(**self.C2, seat_delay=13.0, exit_ready_after=16.0, exit_on='f_tap')
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertTrue(self.drives and self.drives[0] >= 13.0 + self.n.args.seat_settle - 1e-6, self.drives)
        self.assertGreater(ep['drive_moved_m'], 1.0)
        self.assertIsNotNone(ep['seat']['seated_at'])
        self.assertGreater(ep['seat']['polls'], 3)                      # polled every 2 s until m=1
        tries = ep['exit']['tries']
        self.assertGreater(len(tries), 1)                               # early holds ignored, a later one works
        self.assertEqual([t['how'] for t in tries][:2], ['key_f_hold2', 'key_f_hold3'])
        self.assertTrue(ep['exit']['path'].startswith('key_f_tap#'))   # this stub game only takes taps
        gaps = [(b['at'] - a['at']).total_seconds() for a, b in zip(tries, tries[1:])]
        self.assertTrue(all(gp >= 0 for gp in gaps))

    def test_cycle9_no_exit_input_works_scripted_unmount_last(self):
        r, g, world, _ = self._vehicle(**self.C2, exit_on=None, unmount_ok=True)
        ep = r['episodes'][0]
        tries = [t['how'] for t in ep['exit']['tries']]
        self.assertEqual(tries[:4], ['key_f_hold2', 'key_f_hold3', 'pad_b_hold2', 'pad_b_hold3'])
        self.assertEqual(tries[-1], 'cet_unmount')                       # only after every input try
        self.assertTrue(ep['exit']['path'].startswith('cet_unmount#') and ep['exit']['path'].endswith('(scripted)'))
        self.assertTrue(ep['exit_scripted'])
        self.assertIn('SCRIPTED', ep['warnings'][-1])
        self.assertEqual(r['verdict'], 'PASS')
        self.assertIn(gn.UNMOUNT_CMD, self.typed)

    def test_unmount_disabled(self):
        self.n.args.cet_unmount = False
        r, g, world, _ = self._vehicle(**self.C2, exit_on=None, unmount_ok=True)
        self.assertNotIn(gn.UNMOUNT_CMD, self.typed)
        self.assertEqual(r['verdict'], 'FAIL')

    def test_background_run_calls_the_vehicle_without_a_key(self):
        n = self.n
        n.logged = []
        n.log = n.logged.append
        n.args.vehicle_spawn, n.args.background = 'vtap,vhold,summon,codeware', False
        self.assertEqual(n.vehicle_spawn_modes(), ['vtap', 'vhold', 'summon', 'codeware'])      # a foreground run: as configured
        n.args.background = True
        self.assertEqual(n.vehicle_spawn_modes(), ['dpad', 'summon', 'codeware'])               # no key: the pad's call first
        self.assertTrue(any('keyboard spawn modes left out' in m for m in n.logged), n.logged)
        n.args.vehicle_spawn = 'codeware,dpad'
        self.assertEqual(n.vehicle_spawn_modes(), ['codeware', 'dpad'])                         # an order without keys is kept

    def test_summon_first_far_goes_to_vhold(self):
        n = self.n
        self.assertEqual(night.build_parser().parse_args([]).vehicle_spawn, 'vtap,vhold,summon,codeware')
        n.args.vehicle_spawn, n.args.vehicle_arrive_wait = 'summon,vhold', 3.0
        n.wait_arrival = lambda pad, g, ep, mode: None if spawned[-1] == 'summon' else dict(near)   # arrival tested below
        spawned, typed = [], []
        n.console = lambda pad, g, cmds, settle=None: typed.extend(cmds)
        n.spawn_vehicle = lambda pad, g, plan, mode, ep: spawned.append(mode) or ep['events'].append(
            {'spawn': mode, 'spawn_log': 'x'})
        far = {'p': (0.0, 0.0, 0), 'cf': (0, 1), 'v': (0.0, 2629.0, 0), 'vr': (1, 0), 'src': 'summon'}
        near = {'p': (0.0, 0.0, 0), 'cf': (0, 1), 'v': (30.0, 35.0, 0), 'vr': (1, 0), 'src': 'summon'}
        n.poll_pos = lambda g, pad=None, timeout=None: dict(near if spawned[-1] == 'vhold' else far)
        ep = {'events': []}
        pos = n.locate_vehicle(_Pad(), _G(), {'runs': []}, ep)
        self.assertEqual(spawned, ['summon', 'vhold'])
        self.assertEqual(pos['v'][:2], (30.0, 35.0))
        self.assertEqual(typed[0], gn.DESPAWN_CMD)                        # old Codeware car removed first
        mounted = dict(near, src='mounted')
        n.poll_pos = lambda g, pad=None, timeout=None: dict(mounted)
        del n.wait_arrival                       # the real arrival wait ignores src=mounted
        n.args.arrival_max = 3.0
        spawned.clear()
        self.assertIsNone(n.locate_vehicle(_Pad(), _G(), {'runs': []}, {'events': []}))   # src=mounted is not a target

    def test_exit_seen_only_by_nightpos_m0(self):
        r, g, world, _ = self._vehicle(**self.C2, exit_silent=True)
        ep = r['episodes'][0]
        self.assertEqual(ep['exit']['path'], 'key_f_hold2#1 (NightPos m=0)')

    def test_drive_measured_with_mounted_vehicle(self):
        r, g, world, _ = self._vehicle(**self.C2)
        self.assertAlmostEqual(r['episodes'][0]['drive_moved_m'], 6.0, places=1)

    def test_not_seated_is_a_failure_and_no_drive(self):
        r, g, world, _ = self._vehicle(**self.C2, seat_delay=1000.0)
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('not seated', ep['error'])
        self.assertEqual(self.drives, [])
        self.assertNotIn('veh_drive', self.played)

    def test_vehicle_from_behind_right_reaches_the_door(self):
        car, cr = self.C2['car'], self.C2['car_right']
        p0 = gn.from_car_frame(-3.0, -1.5, car, cr)
        r, g, world, _ = self._vehicle(p0=p0, cf0=(-0.920, -0.392), car=car, car_right=cr)
        self.assertEqual(r['verdict'], 'PASS', r)
        wps = [e['waypoints'] for e in r['episodes'][0]['events'] if 'waypoints' in e]
        self.assertTrue(wps and len(wps[0]) >= 3 and wps[0][0][0] <= -3.0, wps)
        lon, lat = gn.car_frame(world['p'], car, cr)
        self.assertTrue(gn.at_door(lon, lat), (lon, lat))

    def test_summoned_car_still_parking_wait_then_approach(self):
        # cycle 11: the summoned car keeps driving/parking; V waits (neutral) until it is still for 3 s
        r, g, world, _ = self._vehicle(**self.C2, car_moves_s=8.0)
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertGreaterEqual(ep['car_stopped']['after_s'], 8.0 + self.n.args.car_stop_hold - 1.0)
        car_final = (self.C2['car'][0] + 3.0 * 8.0, self.C2['car'][1])
        lon, lat = gn.car_frame(world['p'], car_final, self.C2['car_right'])
        self.assertTrue(gn.at_door(lon, lat), (lon, lat))
        first_move = self.played.index('cam_turn') if 'cam_turn' in self.played else self.played.index('veh_approach')
        self.assertNotIn('veh_approach', self.played[:first_move])

    def test_mount_replans_when_car_moves_or_lat_wrong(self):
        n = self.n
        good = {'p': (0.0, 1.3, 0), 'cf': (0, 1), 'v': (0.0, 0.0, 0), 'vr': (0.0, -1.0), 'src': 'tag'}   # lat 1.3
        wrong = dict(good, p=(0.0, 0.4, 0))                                                         # lat 0.4
        seq = iter([(0.9, good), (0.0, wrong), (0.0, good)])
        n.car_speed_now = lambda guard, pad=None: next(seq)
        calls = []
        n.wait_car_stopped = lambda pad, guard, ep, pos: (calls.append('wait') or pos, True)
        n.approach = lambda pad, guard, plan, pos, ep: calls.append('approach') or 0.5
        n.last_known = good
        def play(pad, g, name, plan, sc=None):
            self.played.append(name)
            if name == 'veh_enter':
                self.cet('[utpp_vehiclecam] t=1.00 keepTpp mount -> ride')
            return {'steps': []}
        n.play = play
        ep = {'events': []}
        self.assertIsNotNone(n.mount(_Pad(), _G(), {'runs': []}, good, ep))
        self.assertEqual(calls, ['wait', 'approach', 'approach'])        # moving -> wait + re-plan, lat -> re-plan
        self.assertEqual([p for p in self.played if p != 'cam_turn'], ['veh_enter'])   # camera aim along the car, then X
        self.assertEqual([e['mount_replan'] for e in ep['events'] if 'mount_replan' in e], [1, 2])

    def _arrival(self, track, v_player=(0.0, 0.0), max_s=120.0):
        """Run wait_arrival against a vehicle track: track(t) -> (x, y) | None, sampled every 0.5 s of fake time."""
        n = self.n
        n.args.arrival_max = max_s
        t0 = night.time.monotonic()

        def poll(g, pad=None, timeout=None):
            t = night.time.monotonic() - t0
            v = track(t)
            return {'p': (v_player[0], v_player[1], 0), 'cf': (0, 1), 'v': None if v is None else (v[0], v[1], 0),
                    'vr': (1, 0), 'src': 'summon' if v else 'none'}
        n.poll_pos = poll
        ep = {'events': []}
        pos = n.wait_arrival(_Pad(), _G(), ep, 'vhold')
        return pos, ep, night.time.monotonic() - t0

    def test_arrival_real_car_drives_in_and_parks(self):
        # cycle 10: phantom at 50 m (never moves) for 60 s, then the real car appears 300 m away, drives in at 10 m/s,
        # parks 3 m from V
        def track(t):
            if t < 60:
                return (50.0, 0.0)
            x = max(3.0, 300.0 - 10.0 * (t - 60))
            return (x, 0.0)
        pos, ep, waited = self._arrival(track)
        self.assertIsNotNone(pos)
        self.assertEqual(ep['arrival']['state'], 'arrived')
        self.assertLessEqual(ep['arrival']['dist'], 15.0)
        self.assertGreater(waited, 60 + 29.7)                          # 297 m at 10 m/s + 3 s parked
        self.assertTrue(any('stale_vehicle' in e for e in ep['events']))  # the phantom was marked stale

    def test_arrival_phantom_only_times_out(self):
        pos, ep, waited = self._arrival(lambda t: (50.1, 0.0), max_s=30.0)   # cycle 12: frozen position
        self.assertIsNone(pos)
        self.assertTrue(any('stale_vehicle' in e for e in ep['events']))
        self.assertTrue(any('arrival_timeout' in e for e in ep['events']))

    def test_arrival_parks_too_far_or_no_vehicle(self):
        # drove in and parked 40 m away (beyond --arrival-dist): used after FAR_PARK_HOLD_S still (Male V 2026-11-11)
        pos, ep, _ = self._arrival(lambda t: (max(40.0, 200.0 - 10.0 * t), 0.0), max_s=40.0)
        self.assertIsNotNone(pos)
        self.assertTrue(ep['arrival']['parked_far'])
        pos, ep, _ = self._arrival(lambda t: (max(150.0, 200.0 - 10.0 * t), 0.0), max_s=40.0)   # > FAR_PARK_MAX_M
        self.assertIsNone(pos)
        pos, ep, _ = self._arrival(lambda t: None, max_s=10.0)
        self.assertIsNone(pos)

    def test_arrival_car_already_parked_next_to_v_is_used(self):
        # Male V save 2026-11-12: the car stands 2.7 m away and never moves -> used, not treated as a phantom
        pos, ep, waited = self._arrival(lambda t: (2.7, 0.0), max_s=60.0)
        self.assertIsNotNone(pos)
        self.assertTrue(ep['arrival']['parked_near'])
        self.assertLess(waited, 30.0)

    def test_vtap_and_vhold_key_sequences(self):
        n = self.n
        g = _G()
        ep = {'events': []}
        n.spawn_vehicle(_Pad(), g, {'runs': []}, 'vtap', ep)
        raw = [c for c in g.api.calls if c[0] == 'raw']
        self.assertEqual([c[1] for c in raw], [((gn.SCAN_V, False),), ((gn.SCAN_V, True),)])
        g = _G()
        n.spawn_vehicle(_Pad(), g, {'runs': []}, 'vhold', ep)
        seq = [(c[0], c[1]) for c in g.api.calls if c[0] in ('raw', 'keys')]
        self.assertEqual(seq, [('raw', ((gn.SCAN_V, False),)), ('keys', (gn.SCAN_DOWN,)), ('raw', ((gn.SCAN_V, True),))])
        self.assertNotIn(('keys', (0x1C,)), seq)                        # no Enter
        self.assertEqual([e['spawn'] for e in ep['events']], ['vtap', 'vhold'])
        self.assertEqual(night.validate_args(night.build_parser().parse_args(['--vehicle-spawn', 'vtap,vhold'])), [])

    def test_far_vehicle_is_none_and_mounted_forces_exit_before_next_attempt(self):
        n = self.n
        n.args.vehicle_repeats = 2
        self.cet('[utpp_vehiclecam] t=1.00 keepTpp mount -> ride')          # still in the car from before
        g = _G()
        exits = []
        n.wait_seated = lambda pad, guard, ep=None: {'seated_at': datetime.now()}
        n.exit_vehicle = lambda pad, guard, plan, why: exits.append(why) or self.cet(
            '[utpp_vehiclecam] t=2.00 keepTpp exit -> off') or {'path': 'key_f_tap', 'done_at': None}
        n.spawn_vehicle = lambda pad, guard, plan, mode, ep: ep['events'].append({'spawn': mode, 'spawn_log': 'x'})
        n.args.vehicle_spawn, n.args.spawn_wait = 'summon', 2.0
        n.args.vehicle_arrive_wait, n.args.arrival_max = 2.0, 12.0
        n.poll_pos = lambda guard, pad=None, timeout=None: {'p': (1500.0, 2200.0, 0), 'cf': (0, 1),
                                                             'v': (1500.0, 4829.0, 0), 'vr': (1, 0), 'src': 'summon'}
        n.play = lambda pad, gg, name, plan, sc=None: self.played.append(name) or {'steps': []}
        r = n.vehicle_macro(_Pad(), g, {'runs': []})
        self.assertEqual(exits[0], 'before attempt 1')
        self.assertTrue(all('no vehicle located' in e['error'] for e in r['episodes']))
        self.assertFalse(self.played.count('veh_approach'))
        evs = r['episodes'][0]['events']
        self.assertTrue(any('stale_vehicle' in e for e in evs) and any('arrival_timeout' in e for e in evs), evs)

    def test_vehicle_no_mount_two_x_with_a_walk_between(self):
        r, g, _, _ = self._vehicle(**self.C2, mount_on=None)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(self.played.count('veh_enter'), 2)
        self.assertGreaterEqual(self.played.count('veh_nudge'), 1)      # the walk along the car between the presses
        self.assertEqual(sum(1 for c in g.api.calls if c[0] == 'raw' and c[1] == ((gn.SCAN_F, False),)), 0)
        self.assertTrue(r['episodes'][0]['no_car_suspected'])           # 'no car?' -> remaining tries skipped
        self.assertNotIn('veh_drive', self.played)
        self.assertIn('mount not detected', r['fails'][0])

    def test_mount_second_x_after_a_walk_along_the_car(self):
        # Male V, Quadra (night 20261112-102617): no prompt where V arrived (lon 0.35, pressed to the body), the prompt
        # was at lon -0.79 lat 1.85 with the camera along the car -> X, walk along the car, X
        n = self.n
        n.args.cet_mount = False
        world = {'p': (0.35, 1.59)}
        x_at, aligns = [], []

        def pos():
            return {'p': (world['p'][0], world['p'][1], 0.0), 'cf': (0.0, -1.0), 'v': (0.0, 0.0, 0.0), 'vr': (0.0, -1.0),
                    'src': 'tag', 'mounted': False}
        n.poll_pos = lambda g, pad=None, timeout=None: pos()
        n.car_speed_now = lambda guard, pad=None: (0.0, pos())
        n.camera_align = lambda pad, g, plan, known, target, ep, tol=None, why='': aligns.append(why) or pos()

        def play(pad, g, name, plan, sc=None):
            self.played.append(name)
            if name == 'veh_nudge':
                _, sx, sy = sc.steps[0].tracks['ls'].keys[1]
                dur = sc.steps[0].tracks['ls'].keys[2][0] - 0.03
                world['p'] = gn.dead_reckon(world['p'], (0.0, -1.0), (sx, sy), dur, 1.0 * math.hypot(sx, sy) / 0.45)
            if name == 'veh_enter':
                x_at.append(gn.car_frame(world['p'], (0.0, 0.0), (0.0, -1.0)))
                if abs(x_at[-1][0] - gn.X_SEEK_LONS[0]) <= 0.4 and x_at[-1][1] >= 1.6:
                    self.cet('[utpp_vehiclecam] t=1.00 keepTpp mount -> ride')
            return {'steps': []}
        n.play = play
        ep = {'events': []}
        self.assertIsNotNone(n.mount(_Pad(), _G(), {'runs': []}, pos(), ep))
        self.assertEqual(len(x_at), 2)                                   # arrival X, then the X after the walk
        self.assertAlmostEqual(x_at[0][0], 0.35, places=2)
        self.assertLess(abs(x_at[1][0] - gn.X_SEEK_LONS[0]), 0.4)        # walked towards the rear part of the door
        self.assertGreaterEqual(x_at[1][1], 1.6)                         # and off the body, not pressed into it
        self.assertEqual(aligns, ['x-aim', 'x-aim'])                     # camera along the car before every X
        self.assertEqual(ep['mounted_by'], 'pad_x#2')
        self.assertEqual(night.build_parser().parse_args([]).mount_tries, 2)

    def test_mount_start_line_counts_and_door_spot_memory(self):
        start = '[2026-11-12 11:14:42.739] [info    ] [ 1] [UTPP] UTPP CET [utpp_vehiclecam] t=166.79 keepTpp off -> mount (mount start)'
        self.assertTrue(gn.mount_seen(start))                            # the ride line comes 4 s later
        self.assertEqual(gn.mount_ts(start), datetime(2026, 11, 12, 11, 14, 42, 739000))
        self.assertFalse(gn.mount_seen('[utpp_vehiclecam] t=1 obs state=0'))
        self.assertEqual(gn.body_in_log('x body=woman y\nz body=man db=loaded(man)'), 'man')
        self.assertIsNone(gn.body_in_log('nothing'))
        d = tempfile.mkdtemp()
        f = os.path.join(d, 'spots.json')
        self.assertEqual(gn.load_mount_spots(f), {})
        gn.save_mount_spot(f, 'man', -0.174, 1.561)
        gn.save_mount_spot(f, 'woman', 0.31, 1.05)
        self.assertEqual(gn.load_mount_spots(f), {'man': (-0.17, 1.56), 'woman': (0.31, 1.05)})
        n = self.n
        n.body = 'man'
        old = night.MOUNT_SPOTS_FILE
        try:
            night.MOUNT_SPOTS_FILE = f
            self.assertEqual(n.door_spot(), (-0.17, 1.56))               # the remembered spot first
            night.MOUNT_SPOTS_FILE = os.path.join(d, 'none.json')
            self.assertEqual(n.door_spot(), gn.DOOR_BY_BODY['man'])      # else the default of this body
            n.body = None
            n.utpp_log = None
            self.assertEqual(n.door_spot(), (n.args.door_lon, n.args.door_lat))
        finally:
            night.MOUNT_SPOTS_FILE = old

    def test_plan_path_avoids_the_blocked_end(self):
        # night 20261112-040756: V stuck behind the rear (lon -2.7, lat -2.2); the plan chose the rear end again
        self.assertEqual(gn.plan_path(-2.7, -2.2)[0], (-3.2, 2.2))
        self.assertEqual(gn.plan_path(-2.7, -2.2, blocked_end=-1.0)[0][0], 3.2)
        self.assertEqual(gn.plan_path(-2.7, -2.2, blocked_end=1.0)[0], (-3.2, 2.2))

    def test_approach_stops_when_stuck_and_remembers_the_end(self):
        n = self.n
        pos = {'p': (-2.7, -2.2, 0.0), 'cf': (0.0, 1.0), 'v': (0.0, 0.0, 0.0), 'vr': (0.0, -1.0), 'src': 'tag'}
        n.poll_pos = lambda g, pad=None, timeout=None: dict(pos)
        n.play = lambda pad, g, name, plan, sc=None: self.played.append(name) or {'steps': []}
        ep = {'events': []}
        n.approach(_Pad(), _G(), {'runs': []}, dict(pos), ep)
        self.assertEqual(self.played.count('veh_approach'), night.APPROACH_STUCK_CHUNKS)
        self.assertEqual(n.blocked_end, -1.0)
        self.assertTrue(any('approach_stuck' in e for e in ep['events']))

    def test_cet_mount_flag_default_and_off(self):
        a = night.build_parser().parse_args([])
        self.assertTrue(a.cet_mount)
        self.assertEqual(a.cet_mount_wait, 6.0)
        self.assertFalse(night.build_parser().parse_args(['--no-cet-mount']).cet_mount)

    def test_mount_cmd_content(self):
        for cmd, instant in ((gn.MOUNT_CMD, 'false'), (gn.MOUNT_INSTANT_CMD, 'true')):
            self.assertIn('seat_front_left', cmd)
            self.assertIn('[night] MOUNT ok=', cmd)
            self.assertIn(gn.NIGHT_TAG, cmd)
            self.assertIn('d.isInstant=' + instant, cmd)
            self.assertIn('Game.GetMountingFacility():Mount(r)', cmd)
            self.assertIn('r.preservePositionAfterMounting=false', cmd)
            self.assertIn('d.mountParentEntityId=v:GetEntityID()', cmd)
            self.assertIn('i.slotId=s', cmd)
            self.assertNotIn(chr(10), cmd)

    def test_cet_mount_fallback_after_x_presses(self):
        r, g, world, _ = self._vehicle(**self.C2, mount_on=None, cet_mount_on='animated')
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'PASS', r)                        # a scripted mount alone is not a FAIL
        self.assertEqual(self.played.count('veh_enter'), 2)             # every X press came first
        self.assertEqual(self.typed.count(gn.MOUNT_CMD), 1)
        self.assertNotIn(gn.MOUNT_INSTANT_CMD, self.typed)
        self.assertEqual(ep['mounted_by'], 'cet_mount#1 (scripted)')
        self.assertEqual(ep['mount']['how'], 'cet_mount')
        self.assertTrue(ep['mount']['scripted'] and ep['mount_scripted'])
        self.assertIn('mount was SCRIPTED', ep['warnings'][0])
        self.assertNotIn('no_car_suspected', ep)
        self.assertIsNotNone(ep['seat']['seated_at'])                   # the ride/exit steps continued
        self.assertIsNotNone(ep['exit_pull_end'])

    def test_cet_mount_instant_second_attempt(self):
        r, g, world, _ = self._vehicle(**self.C2, mount_on=None, cet_mount_on='instant')
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(self.typed.count(gn.MOUNT_CMD), 1)
        self.assertEqual(self.typed.count(gn.MOUNT_INSTANT_CMD), 1)
        self.assertEqual(ep['mounted_by'], 'cet_mount#2 (scripted)')
        self.assertEqual([a['how'] for a in ep['mount']['attempts']], ['animated', 'instant'])

    def test_cet_mount_no_vehicle_is_a_fail(self):
        r, g, world, _ = self._vehicle(**self.C2, mount_on=None, cet_mount_on='dead')
        ep = r['episodes'][0]
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('mount not detected', r['fails'][0])
        self.assertNotIn(gn.MOUNT_INSTANT_CMD, self.typed)              # Lua error: no point in the second variant
        self.assertNotIn('mount', ep)
        self.assertTrue(ep['no_car_suspected'])
        self.assertIn('MOUNT ok=false', ep['cet_mount_attempts'][0]['log'])

    def test_cet_mount_off_keeps_old_behaviour(self):
        self.n.args.cet_mount = False
        r, g, world, _ = self._vehicle(**self.C2, mount_on=None, cet_mount_on='animated')
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertNotIn(gn.MOUNT_CMD, self.typed)
        self.assertNotIn('mount', r['episodes'][0])

    def test_cet_mount_after_f_when_enabled(self):
        self.n.args.mount_try_f = True
        r, g, world, _ = self._vehicle(**self.C2, mount_on=None, cet_mount_on='animated')
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(sum(1 for c in g.api.calls if c[0] == 'raw' and c[1] == ((gn.SCAN_F, False),)) >= 1, True)
        self.assertEqual(r['episodes'][0]['mounted_by'], 'cet_mount#1 (scripted)')

    def test_key_crouch_replaces_pad_crouch_toggle(self):
        n = self.n
        sc = n.scenarios['crouch_walk_turns']
        self.assertEqual([sc.steps[i].name for i in gn.crouch_toggle_steps(sc)], ['crouch', 'uncrouch'])
        # 2026-10-23: the pad toggle is an R3 tap 0.08 s (InputSchemesMode=Alternative); legacy B holds >= 0.3 s still count
        self.assertEqual({(b.button, b.hold) for i in gn.crouch_toggle_steps(sc) for b in sc.steps[i].buttons}, {('R3', 0.08)})
        legacy = parse_scenario('legacy', {'steps': [
            {'name': 'c', 'duration': 1.0, 'buttons': [{'t': 0.1, 'button': 'B', 'hold': 0.35}], 'metrics': False},
            {'name': 'dodge', 'duration': 1.0, 'buttons': [{'t': 0.1, 'button': 'B', 'hold': 0.08}], 'metrics': False}]})
        self.assertEqual(gn.crouch_toggle_steps(legacy), [0])
        g = _G()
        state = {'n': 0}

        def send(evs, api=g.api):
            api.calls.append(('raw', tuple(evs), api.fg))
            if evs and evs[0] == (gn.SCAN_C, True):
                state['n'] += 1
                self.append(_trace('23:00:0%d.000' % state['n'], 2 if state['n'] == 1 else 1,
                                   'Crouch_Idle' if state['n'] == 1 else 'Stand_Idle'))
        g.api._send = send
        calls = []

        def fake_run(sub, pad, guard):
            calls.append([s.name for s in sub.steps])
            now = datetime(2026, 10, 7, 23, 0, 0)
            return {'steps': [{'name': s.name, 'start': now, 'end': now, 'metrics': s.metrics} for s in sub.steps],
                    'aborted': None}
        orig = night.gt_run.run_scenario
        night.gt_run.run_scenario = fake_run
        try:
            plan = {'runs': []}
            rec, fails = n.play_with_key_crouch(_Pad(), g, 'crouch_walk_turns', plan)
        finally:
            night.gt_run.run_scenario = orig
        self.assertEqual(fails, [])
        self.assertEqual(calls, [['settle'], ['walk', 'arc_right', 'arc_left', 'stop'], ['stand_check']])
        self.assertEqual([s['name'] for s in rec['steps']], [s.name for s in sc.steps])   # analysis keys intact
        self.assertEqual(state['n'], 2)                                  # one C per toggle, no retry needed
        self.assertFalse([c for c in g.api.calls if c[0] != 'raw'])      # no pad B, no chords

    def test_key_crouch_unconfirmed_is_a_mechanics_fail(self):
        n = self.n
        g = _G()
        orig = night.gt_run.run_scenario
        night.gt_run.run_scenario = lambda sub, pad, guard: {'steps': [], 'aborted': None}
        try:
            rec, fails = n.play_with_key_crouch(_Pad(), g, 'crouch_walk_turns', {'runs': []})
        finally:
            night.gt_run.run_scenario = orig
        self.assertEqual(len(fails), 2)
        self.assertEqual(sum(1 for c in g.api.calls if c[1] == ((gn.SCAN_C, False),)), 4)   # 2 toggles x 2 taps
        mech, metr = gn.split_failures([], [{'name': 'crouch_walk_turns', 'run': 1, 'fails': fails}])
        self.assertEqual(len(mech), 2)
        self.assertEqual(metr, [])


class Round3Tests(unittest.TestCase):
    def test_cycle2_stick_direction(self):
        """cf=(-0.920,-0.392), car at (-3.02,+1.98) from V -> stick right 0.83, forward 0.55."""
        sx, sy, d = gn.approach_stick((1500.00, 2201.90), (-0.920, -0.392), (1496.98, 2203.88), mag=1.0)
        self.assertAlmostEqual(sx, 0.83, delta=0.01)
        self.assertAlmostEqual(sy, 0.55, delta=0.01)
        self.assertAlmostEqual(d, 3.61, delta=0.01)
        # dead reckoning along that stick moves V straight towards the car
        x, y = gn.dead_reckon((1500.00, 2201.90), (-0.920, -0.392), (sx, sy), 1.0, 3.0)
        self.assertAlmostEqual(math.hypot(1496.98 - x, 2203.88 - y), d - 3.0, delta=0.02)
        self.assertEqual(gn.rotate_stick(0.0, 1.0, 90), (1.0, 0.0))

    def test_round4_helpers(self):
        self.assertAlmostEqual(gn.bearing_deg((1500.00, 2201.90), (-0.920, -0.392), (1496.98, 2203.88)), 56.3, delta=0.5)
        self.assertAlmostEqual(abs(gn.bearing_deg((1498.58, 2200.57), (-0.921, -0.389), (1541.55, 2216.92))), 179, delta=2)
        self.assertAlmostEqual(gn.turned_deg((0, 1), (1, 0)), 90.0)
        self.assertAlmostEqual(gn.turned_deg((0, 1), (-1, 0)), -90.0)
        self.assertEqual([gn.stick_class(0, 1), gn.stick_class(0, 1, True), gn.stick_class(1, 0), gn.stick_class(0, -1)],
                         ['forward', 'sprint', 'side', 'back'])
        self.assertEqual(gn.speed_for(0, -1), 1.5)
        self.assertEqual(gn.approach_timebox(3.6), 25.0)
        self.assertAlmostEqual(gn.approach_timebox(46.0), 46 / 1.5 + 15)
        rx, dur = gn.camera_pulse(-90.0)
        self.assertEqual(rx, -1.0)
        self.assertAlmostEqual(dur, 0.6)                    # capped
        self.assertEqual(gn.camera_pulse(30.0, sign=-1.0)[0], -1.0)
        self.assertAlmostEqual(gn.camera_pulse(30.0)[1], 0.2)
        # camera pitch levelling (Male V save starts at -65 deg)
        trace = ('x [utpp_camera] camTrace t=1.00 body=55.8 out=55.8 pitch=-65.0 mode=free a\n'
                 'x [utpp_camera] camTrace t=1.10 body=55.8 out=55.8 pitch=-12.5 mode=free a\n')
        self.assertEqual(gn.latest_cam_pitch(trace), -12.5)
        self.assertIsNone(gn.latest_cam_pitch('no trace here'))
        ry, dur = gn.pitch_pulse(-65.0)
        self.assertEqual(ry, 0.5)                           # looking down -> stick up
        self.assertAlmostEqual(dur, 1.2)                    # capped
        self.assertEqual(gn.pitch_pulse(30.0)[0], -0.5)
        self.assertEqual(gn.pitch_pulse(-30.0, sign=-1.0)[0], -0.5)
        self.assertAlmostEqual(gn.pitch_pulse(15.0)[1], 0.4)

    def test_cycle10_car_frame_and_direct_slide(self):
        # POS 09:30:11: V=(1537.41,2214.00), car=(1540.05,2214.31), car forward (-0.394,0.918) -> right (0.918,0.394)
        v, vr = (1540.05, 2214.31), (0.918, 0.394)
        f, l = gn.car_axes(vr)
        self.assertAlmostEqual(f[0], -0.394, places=2)
        self.assertAlmostEqual(f[1], 0.918, places=2)
        self.assertAlmostEqual(l[0], -f[1], places=6)                  # left = (-fy, fx)
        self.assertAlmostEqual(l[1], f[0], places=6)
        lon, lat = gn.car_frame((1537.41, 2214.00), v, vr)
        self.assertAlmostEqual(lon, 0.76, delta=0.01)                  # 0.76 m ahead of the centre
        self.assertAlmostEqual(lat, 2.55, delta=0.01)                  # 2.55 m to the left (driver side)
        self.assertEqual(gn.plan_path(lon, lat), [(gn.DOOR_LON, gn.DOOR_LAT)])   # direct slide to the door
        door = gn.door_target({'v': v, 'vr': vr})
        self.assertEqual(tuple(round(x, 6) for x in gn.car_frame(door, v, vr)), (gn.DOOR_LON, gn.DOOR_LAT))
        self.assertFalse(gn.at_door(lon, lat))
        self.assertTrue(gn.at_door(0.5, 1.3))
        self.assertFalse(gn.at_door(0.35, 0.6))                         # too close to the body
        self.assertEqual(gn.car_frame(gn.from_car_frame(-1.2, 3.4, v, vr), v, vr)[0], approx(-1.2))

    def test_behind_right_goes_around_the_rear(self):
        wps = gn.plan_path(-3.0, -1.5)
        self.assertEqual(wps[-1], (gn.DOOR_LON, gn.DOOR_LAT))
        self.assertTrue(all(w[0] <= -3.0 for w in wps[:2]), wps)        # behind the car first
        self.assertGreaterEqual(wps[1][1], 2.0)                         # then out to the door side
        self.assertTrue(all(abs(w[1]) >= gn.CLEAR_LAT for w in wps[:-1]), wps)
        front = gn.plan_path(2.6, 0.5)                                  # at the front end -> around the front
        self.assertTrue(front[0][0] >= 3.0 and front[1][1] >= 2.0, front)
        hug = gn.plan_path(-1.8, 1.05)                                   # door side but hugging the body far back
        self.assertEqual(hug[0], (-1.8, gn.SIDE_LAT))
        for lon0, lat0 in ((-3.0, -1.5), (2.6, 0.5), (1.0, -2.0), (-1.8, 1.05), (0.76, 2.55)):
            pts = [(lon0, lat0)] + gn.plan_path(lon0, lat0)
            for (a0, b0), (a1, b1) in zip(pts, pts[1:]):                # no segment crosses the body (5 x 2 m box)
                for k in range(21):
                    a, b = a0 + (a1 - a0) * k / 20, b0 + (b1 - b0) * k / 20
                    inside = abs(a) < 2.3 and abs(b) < 1.0
                    self.assertFalse(inside, (lon0, lat0, a, b))

    def test_stationary_duration(self):
        moving = [(t * 0.5, (t * 1.5, 0.0)) for t in range(6)]                 # 3 m/s
        self.assertEqual(gn.stationary_duration(moving), 0.0)
        parked = moving + [(3.0 + t * 0.5, (7.5 + t * 0.02, 0.0)) for t in range(1, 8)]   # 0.04 m/s
        self.assertAlmostEqual(gn.stationary_duration(parked), 4.0)   # from the last moving sample (t=2.5) to 6.5
        self.assertEqual(gn.stationary_duration(parked[:1]), 0.0)

    def test_arrival_state(self):
        drive = [(t * 0.5, (100.0 - 5.0 * t, 0.0)) for t in range(10)] + [(5.0 + t * 0.5, (55.0, 0.0)) for t in range(1, 8)]
        self.assertEqual(gn.arrival_state(drive, (60.0, 0.0))['state'], 'arrived')
        self.assertEqual(gn.arrival_state(drive, (0.0, 0.0))['state'], 'waiting')       # parked 55 m away
        self.assertEqual(gn.arrival_state(drive[:8], (60.0, 0.0))['state'], 'driving')
        frozen = [(t * 0.5, (50.1, 0.0)) for t in range(30)]
        self.assertEqual(gn.arrival_state(frozen, (0.0, 0.0))['state'], 'stale')
        jump = [(0.0, (50.0, 0.0)), (0.5, (3.0, 0.0))] + [(0.5 + t * 0.5, (3.0, 0.0)) for t in range(1, 9)]
        self.assertNotEqual(gn.arrival_state(jump, (0.0, 0.0))['state'], 'arrived')     # one jump is not driving
        self.assertEqual(gn.arrival_state([], (0.0, 0.0))['state'], 'none')

    def test_round5_helpers(self):
        t = ('[utpp_vehiclecam] t=322.55 keepTpp off -> mount' + chr(10) + '[utpp_vehiclecam] t=325.27 keepTpp mount -> ride')
        self.assertEqual(gn.vehiclecam_phase(t), 'ride')
        self.assertTrue(gn.mounted_state(None, t))
        self.assertFalse(gn.mounted_state({'mounted': False}, t))          # NightPos m= wins
        self.assertIsNone(gn.mounted_state(None, 'nothing'))
        self.assertFalse(gn.mounted_state(None, t + chr(10) + '[utpp_vehiclecam] keepTpp exit -> off'))
        # a save that loads into a first-person ride logs only this line: V is seated although no mount was logged
        game = '[utpp_vehiclecam] t=7.90 keepTpp off -> game (driver ride without the hold)'
        self.assertEqual(gn.vehiclecam_phase(game), 'game')
        self.assertTrue(gn.mounted_state(None, game))
        self.assertTrue(gn.mounted_state(None, t + chr(10) + '[utpp_vehiclecam] t=9.1 keepTpp mount -> chase (retake)'))
        self.assertTrue(gn.exit_done("x exit pull: end (utpp_camera's orbit)"))
        p = {'p': (1498.58, 2200.57), 'v': (1541.55, 2216.92)}
        self.assertTrue(gn.vehicle_in_range(p))                            # 46 m
        self.assertFalse(gn.vehicle_in_range({'p': (0, 0), 'v': (0, 2629)}))
        self.assertFalse(gn.vehicle_in_range({'p': (0, 0), 'v': None}))

    def test_wheel_suspect_thresholds(self):
        sc = load_scenarios(SCEN)
        t0 = datetime(2026, 10, 7, 6, 23, 29)
        walk = sc['flick_180_walk']                        # |stick| 0.45: never a wheel suspect (cycle-2 false positive)
        self.assertEqual(gn.stick_windows(walk), [])
        steps = [{'name': 'x', 'start': t0}]
        still = gt_log.parse_text(chr(10).join(_trace('06:23:%02d.000' % s, 1) for s in range(29, 50)))
        self.assertIsNone(gn.wheel_suspect(walk, steps, still))
        run = sc['flick_180_run']
        self.assertTrue(gn.stick_windows(run))
        self.assertTrue(gn.wheel_suspect(run, steps, still))
        moving = gt_log.parse_text(chr(10).join(_trace('06:23:%02d.000' % s, 1, 'Run', 4.0) for s in range(29, 50)))
        self.assertFalse(gn.wheel_suspect(run, steps, moving))
        self.assertIsNone(gn.wheel_suspect(run, steps, []))
        probe = sc['wheel_probe']
        self.assertTrue(gn.stick_windows(probe))           # the probe itself qualifies (>= 1.5 s at >= 0.7)

    def test_split_failures_cycle2(self):
        harness = [{'scenario': 'crouch_walk_turns', 'run': 1, 'step': 'walk',
                    'fails': ['offset_saturated_s=0.556 > 0.5', 'crouch not engaged: crouch_fraction=0.0 < 0.8']},
                   {'scenario': 'flick_180_walk', 'run': 1, 'step': 'accel', 'fails': ['footlock_corr_max_cm=24.1 > 15.0']},
                   {'scenario': 'x', 'run': 1, 'step': 'y', 'fails': ['rate_min@slow=0.1 < 0.2']}]
        recs = [{'name': 'vehicle_mount_exit', 'run': 1, 'fails': ['mount not detected (...)']},
                {'name': 'sprint_dash', 'run': 1}]
        mech, metr = gn.split_failures(harness, recs)
        self.assertEqual(mech, ['crouch_walk_turns#1.walk: crouch not engaged: crouch_fraction=0.0 < 0.8',
                                'vehicle_mount_exit#1: mount not detected (...)'])
        self.assertEqual(len(metr), 3)

    def test_stance(self):
        lines = gt_log.parse_text(chr(10).join([_trace('06:24:33.000', 2, 'Crouch_Walk'),
                                                _trace('06:24:34.000', 1, 'Run_Loop')]))
        self.assertEqual([gn.stance_of(l) for l in lines], ['crouched', 'standing'])
        self.assertTrue(gn.stance_seen(lines, 'crouched'))
        idle = gt_log.parse_text(_trace('06:24:35.000', 0, 'M_Neutral_Crouch_Idle_Loop'))
        self.assertEqual(gn.stance_of(idle[0]), 'crouched')     # gate closed while idle: clip name decides


FRAMES = os.path.join(PROJECT, 'Research', 'logs', 'night', '20261007-073804', 'sheets', 'vehicle_exit_window', 'frames')
# f0002 07:43:57.18 and f0010 07:44:01.18: overlay visible; f0001 07:43:56.68 and f0006 07:43:59.18: closed
CYCLE5_FRAMES = (('f0002.png', 'open'), ('f0003.png', 'open'), ('f0010.png', 'open'), ('f0017.png', 'open'),
                 ('f0001.png', 'closed'), ('f0006.png', 'closed'), ('f0012.png', 'closed'), ('f0020.png', 'closed'))
# The guard needs the frame FILES: the PNGs of FRAMES are gone (the folder is empty, last modified 2026-10-30), so
# ffmpeg returned 0 bytes and the test FAILed on missing data instead of skipping. Source video of the cycle (frames can
# be re-extracted from it): Media\mp4\2026-10-07_night-073804\rec_20261007-073853.151.mp4.
CYCLE5_PRESENT = all(os.path.isfile(os.path.join(FRAMES, n)) for n, _ in CYCLE5_FRAMES)


def _img(panel: int, ref: int, w: int = 160, h: int = 90, text: bool = False) -> bytes:
    """Synthetic gray frame: `panel` luma in the top part, `ref` in the bottom part (+ bright 'text' rows)."""
    px = bytearray()
    for y in range(h):
        for x in range(w):
            v = panel if y < h * 0.5 else ref
            if text and y < h * 0.5 and y % 6 == 0 and x % 3:
                v = 200
            px.append(v)
    return bytes(px)


def _frame_gray(name: str):
    """Real night-runner frame (320x180 image above the 120 px text pad) as 8-bit gray, via ffmpeg."""
    import subprocess
    import video_sheets
    p = subprocess.run([video_sheets.FFMPEG, '-v', 'error', '-i', os.path.join(FRAMES, name), '-vf',
                        'crop=320:180:0:0,format=gray', '-f', 'rawvideo', '-'], capture_output=True)
    return p.stdout, 320, 180


class OverlayTests(unittest.TestCase):
    def test_synthetic(self):
        self.assertEqual(gn.classify_overlay(_img(150, 110), 160, 90)[0], 'closed')       # sky + scenery
        self.assertEqual(gn.classify_overlay(_img(22, 110, text=True), 160, 90)[0], 'open')  # dark console + text
        self.assertIsNone(gn.classify_overlay(_img(20, 20), 160, 90)[0])                    # night: cannot tell
        self.assertIsNone(gn.classify_overlay(_img(75, 110), 160, 90)[0])                   # in between

    @unittest.skipUnless(CYCLE5_PRESENT and os.path.isfile(__import__('video_sheets').FFMPEG),
                         'night cycle-5 frame PNGs missing (folder empty since 2026-10-30; see CYCLE5_FRAMES) or ffmpeg missing')
    def test_real_frames_cycle5(self):
        for name, want in CYCLE5_FRAMES:
            g, w, h = _frame_gray(name)
            self.assertEqual(len(g), w * h, name)
            state, st = gn.classify_overlay(g, w, h)
            self.assertEqual(state, want, f'{name} {st}')

    def test_toggle_effect(self):
        a, b = {'panel_mean': 40.0}, {'panel_mean': 146.0}
        self.assertEqual(gn.overlay_toggle_effect(a, b, 'closed'), 'fixed')
        self.assertEqual(gn.overlay_toggle_effect(b, a, 'closed'), 'wrong')
        self.assertEqual(gn.overlay_toggle_effect(b, a, 'open'), 'fixed')
        self.assertEqual(gn.overlay_toggle_effect(a, {'panel_mean': 45.0}, 'closed'), 'none')

    def _night(self, frames):
        t = EnsureClosedTests()
        api = FakeApi()
        n = EnsureClosedTests.make(t, api, None)
        seq = iter(frames)
        n.overlay_grab = lambda guard: (next(seq), 160, 90)
        g = n.guard()
        g.refresh_game()
        return n, g, api

    def toggles(self, api):
        return sum(1 for c in api.calls if c[0] == 'keys' and c[1] == (0x29,))

    def setUp(self):
        self._time = night.time
        night.time = FakeTime()

    def tearDown(self):
        night.time = self._time

    def test_found_open_toggled_closed(self):
        n, g, api = self._night([_img(22, 110, text=True), _img(150, 110)])
        self.assertEqual(n.ensure_overlay(g, 'closed', 'test'), 'closed')
        self.assertEqual(self.toggles(api), 1)
        self.assertEqual(n.overlay_stats['toggled'], 1)

    def test_already_closed_or_unknown_no_key(self):
        n, g, api = self._night([_img(150, 110), _img(20, 20)])
        self.assertEqual(n.ensure_overlay(g, 'closed'), 'closed')
        self.assertIsNone(n.ensure_overlay(g, 'closed'))
        self.assertEqual(self.toggles(api), 0)

    def test_false_positive_is_undone(self):
        # 'open' by the classifier, but the toggle made the panel area darker -> it was closed: undo
        n, g, api = self._night([_img(35, 110, text=True), _img(10, 110, text=True)])
        self.assertIsNone(n.ensure_overlay(g, 'closed'))
        self.assertEqual(self.toggles(api), 2)
        self.assertEqual(n.overlay_stats['undone'], 1)

    def test_lost_toggle_key_retried(self):
        open_ = _img(22, 110, text=True)
        n, g, api = self._night([open_, open_, open_, _img(150, 110)])
        self.assertEqual(n.ensure_overlay(g, 'closed'), 'closed')
        self.assertEqual(self.toggles(api), 3)

    def test_console_checks_before_open_and_after(self):
        closed, open_ = _img(150, 110), _img(22, 110, text=True)
        n, g, api = self._night([closed, open_, closed])
        n.type_console(g, ['Game.UTPPLogLine("x")'])
        self.assertEqual(self.toggles(api), 2)                        # only the console's own open + close
        n, g, api = self._night([closed, closed, open_, closed, closed])   # open key lost -> fixed by the check
        n.type_console(g, ['Game.UTPPLogLine("x")'])
        self.assertEqual(self.toggles(api), 3)
        self.assertEqual(n.overlay_stats['toggled'], 1)


class NightAutoTests(unittest.TestCase):
    def test_latest_pos(self):
        now = datetime(2026, 10, 7, 23, 0, 1, 200000)
        t = chr(10).join([
            '[2026-10-07 23:00:00.000] [info] [1] [UTPP] UTPP CET [night] POS p=1.00,2.00,3.00 cf=0.000,1.000 v=nil src=none w=0 m=0 n=7',
            '[2026-10-07 23:00:00.500] [info] [1] [UTPP] UTPP CET [night] POS p=1.50,2.00,3.00 cf=0.000,1.000 v=nil src=none w=0 m=1 n=8',
            '[2026-10-07 23:00:00.600] [info] [1] [UTPP] UTPP MMTRACE tick clip=x |v|=0.00'])
        p = gn.latest_pos(t, now)
        self.assertEqual((p['p'][0], p['n'], p['mounted']), (1.5, 8, True))
        self.assertIsNone(gn.latest_pos(t, now.replace(second=5)))       # 4.5 s old -> stale
        self.assertIsNone(gn.latest_pos('nothing', now))
        self.assertEqual(gn.nightauto_cmd(0.5), 'GetMod("utpp_core").NightAuto(true, 0.50, "UTPPNight")')

    def test_poll_pos_reads_the_log_without_console(self):
        fd, log = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        try:
            ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:23]
            with open(log, 'w', encoding='utf-8') as f:
                f.write(f'[{ts}] [info] [1] [UTPP] UTPP CET [night] POS p=5.00,6.00,0.00 cf=0.000,1.000 '
                        f'v=nil src=none w=1 m=0 n=42' + chr(10))
            n = EnsureClosedTests.make(EnsureClosedTests(), FakeApi(), None)
            n.utpp_log, n.auto_pos = log, True
            n.console = lambda *a, **k: self.fail('no console with NightAuto')
            p = n.poll_pos(_G())
            self.assertEqual((p['p'][:2], p['n'], p['weapon']), ((5.0, 6.0), 42, True))
        finally:
            os.remove(log)


class MediaMoveTests(unittest.TestCase):
    """Media rule: the recording and the sheet images of a night run go to Media\\mp4 / Media\\png (temp dirs only)."""
    STAMP = '20261112-040756'

    def setUp(self):
        import failure_clips
        import shutil
        self.fc, self.shutil = failure_clips, shutil
        self.tmp = tempfile.mkdtemp()
        self.run = os.path.join(self.tmp, 'night', self.STAMP)
        self.media = os.path.join(self.tmp, 'Media')
        for ext in ('mp4', 'png'):
            os.makedirs(os.path.join(self.media, ext))
            with open(os.path.join(self.media, ext, 'README.md'), 'wb') as f:
                f.write(b'# Media\n\n| folder | files | MB |\n|---|---|---|\n| 2026-11-11_x | 1 | 1.0 |\n')
        os.makedirs(os.path.join(self.run, 'sheets', 'full', 'frames'))
        os.makedirs(os.path.join(self.run, 'sheets', 'door_r1', 'frames'))
        self.rec = os.path.join(self.run, 'rec_20261112-040927.986.mp4')
        files = {self.rec: b'v' * 2000,
                 os.path.join(self.run, 'sheets', 'full', 'sheet_01.png'): b'a',
                 os.path.join(self.run, 'sheets', 'full', 'sheet_02.png'): b'b',
                 os.path.join(self.run, 'sheets', 'door_r1', 'sheet_01.png'): b'c',
                 os.path.join(self.run, 'sheets', 'full', 'index.txt'): b'i',
                 os.path.join(self.run, 'sheets', 'full', 'frames', 'f0001.png'): b'f',
                 os.path.join(self.run, 'sheets', 'door_r1', 'frames', 'f0001.png'): b'g'}
        for p, data in files.items():
            with open(p, 'wb') as f:
                f.write(data)
        self.logs = []

    def tearDown(self):
        self.shutil.rmtree(self.tmp, ignore_errors=True)

    def move(self, summary):
        return self.fc.move_to_media(self.run, summary, self.media, log=self.logs.append)

    def readme(self, ext):
        with open(os.path.join(self.media, ext, 'README.md'), encoding='utf-8') as f:
            return f.read()

    def test_moves_recording_and_sheets_and_updates_paths_and_readmes(self):
        summary = {'recording': {'path': self.rec, 'mode': 'desktop'}}
        notes = self.move(summary)
        mp4 = os.path.join(self.media, 'mp4', '2026-11-12_night-' + self.STAMP, os.path.basename(self.rec))
        self.assertEqual(notes, {'recording': mp4, 'sheet_images': 3, 'errors': []})
        self.assertTrue(os.path.isfile(mp4))
        self.assertFalse(os.path.exists(self.rec))
        self.assertEqual((summary['recording']['path'], summary['recording']['recording_original']), (mp4, self.rec))
        pngdir = os.path.join(self.media, 'png', '2026-11-12_night-' + self.STAMP)
        self.assertEqual(sorted(os.listdir(pngdir)),
                         ['door_r1__sheet_01.png', 'full__sheet_01.png', 'full__sheet_02.png'])
        self.assertEqual([(s['name'], os.path.basename(s['path'])) for s in summary['sheet_images']],
                         [('door_r1', 'door_r1__sheet_01.png'), ('full', 'full__sheet_01.png'),
                          ('full', 'full__sheet_02.png')])
        self.assertTrue(all(s['original'].endswith('.png') and not os.path.exists(s['original'])
                            for s in summary['sheet_images']))
        # intermediate files stay: frames\ and index.txt are not moved
        self.assertTrue(os.path.isfile(os.path.join(self.run, 'sheets', 'full', 'frames', 'f0001.png')))
        self.assertTrue(os.path.isfile(os.path.join(self.run, 'sheets', 'door_r1', 'frames', 'f0001.png')))
        self.assertTrue(os.path.isfile(os.path.join(self.run, 'sheets', 'full', 'index.txt')))
        self.assertIn(f'| 2026-11-12_night-{self.STAMP} | 1 | 0.0 |', self.readme('mp4'))
        self.assertIn(f'| 2026-11-12_night-{self.STAMP} | 3 | 0.0 |', self.readme('png'))
        self.assertIn('| 2026-11-11_x | 1 | 1.0 |', self.readme('png'))            # existing rows untouched
        json.dumps(summary)

    def test_second_run_moves_nothing_and_does_not_duplicate_rows(self):
        summary = {'recording': {'path': self.rec}}
        self.move(summary)
        before = (self.readme('mp4'), self.readme('png'), json.dumps(summary, sort_keys=True))
        notes = self.move(summary)
        self.assertEqual((notes['recording'], notes['sheet_images'], notes['errors']), (None, 0, []))
        self.assertEqual((self.readme('mp4'), self.readme('png'), json.dumps(summary, sort_keys=True)), before)

    def test_killed_cycle_without_summary_recording_still_moves_what_exists(self):
        summary = {}                                       # collect never got to write summary['recording']
        notes = self.move(summary)
        self.assertIsNotNone(notes['recording'])
        self.assertEqual(summary['recording']['recording_original'], self.rec)
        self.assertEqual(notes['sheet_images'], 3)

    def test_failed_move_is_logged_and_does_not_raise(self):
        summary = {'recording': {'path': self.rec}}
        orig = self.fc.shutil.move

        def boom(src, dst):
            raise PermissionError('locked')
        self.fc.shutil.move = boom
        try:
            notes = self.move(summary)
        finally:
            self.fc.shutil.move = orig
        self.assertEqual(len(notes['errors']), 4)         # the recording and three sheets
        self.assertTrue(os.path.isfile(self.rec))
        self.assertEqual(summary['recording']['path'], self.rec)
        self.assertNotIn('recording_original', summary['recording'])
        self.assertNotIn('sheet_images', summary)
        self.assertTrue(any('move failed' in l for l in self.logs))

    def test_existing_file_in_media_is_never_overwritten(self):
        out = os.path.join(self.media, 'mp4', '2026-11-12_night-' + self.STAMP)
        os.makedirs(out)
        taken = os.path.join(out, os.path.basename(self.rec))
        with open(taken, 'wb') as f:
            f.write(b'keep')
        summary = {'recording': {'path': self.rec}}
        self.move(summary)
        with open(taken, 'rb') as f:
            self.assertEqual(f.read(), b'keep')
        self.assertTrue(summary['recording']['path'].endswith('~2.mp4'))
        self.assertTrue(os.path.isfile(summary['recording']['path']))

    def test_date_comes_from_the_stamp_not_from_today(self):
        self.assertTrue(self.fc.media_folder(self.run, 'png', self.media).endswith('2026-11-12_night-' + self.STAMP))
        self.assertTrue(self.fc.media_folder(os.path.join(self.tmp, '20250102-030405'), 'mp4', self.media)
                        .endswith(os.path.join('mp4', '2025-01-02_night-20250102-030405')))

    def test_collect_finds_a_recording_that_was_already_moved(self):
        summary = {'recording': {'path': self.rec}, 'clock': {'video_start': '2026-11-12 04:09:30.000',
                                                              'duration_s': 100.0}}
        self.move(summary)
        with open(os.path.join(self.run, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f)
        with open(os.path.join(self.run, 'cycle.json'), 'w', encoding='utf-8') as f:
            json.dump({'scenarios': []}, f)
        video, items = self.fc.collect(self.run, 2.0)
        self.assertEqual(video, summary['recording']['path'])
        self.assertEqual(items, [])
        moved = summary['recording']['path']
        # a summary that kept only the old location in 'path' and the new one under 'recording_original'
        summary['recording'] = {'path': self.rec, 'recording_original': moved}
        with open(os.path.join(self.run, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f)
        self.assertEqual(self.fc.collect(self.run, 2.0)[0], moved)
        summary['recording'] = {'path': self.rec}                            # gone everywhere: no usable recording
        with open(os.path.join(self.run, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f)
        with self.assertRaises(SystemExit):
            self.fc.collect(self.run, 2.0)

    def test_options(self):
        a = night.build_parser().parse_args([])
        self.assertEqual((a.keep_sheet_frames, a.no_media_move), (False, False))   # frames go once the sheet exists
        a = night.build_parser().parse_args(['--keep-sheet-frames', '--no-media-move'])
        self.assertEqual((a.keep_sheet_frames, a.no_media_move), (True, True))


class CalibrationWaitTests(unittest.TestCase):
    def setUp(self):
        self._time = night.time
        night.time = FakeTime()
        fd, self.log = tempfile.mkstemp(suffix='.log')
        os.close(fd)

    def tearDown(self):
        night.time = self._time
        os.remove(self.log)

    def make(self, db):
        n = EnsureClosedTests.make(EnsureClosedTests(), FakeApi(), None)
        n.args.db = db
        n.logged = []
        n.log = n.logged.append
        return n

    def test_db_none_does_not_wait(self):
        for db in ('none', 'None'):
            n = self.make(db)
            t0, info = night.time.monotonic(), {}
            self.assertIsNone(n.wait_calibration(_G(), self.log, 0, info))
            self.assertEqual(night.time.monotonic(), t0)               # no waiting at all
            self.assertIsNone(info['calibration_seen'])
            self.assertEqual(len(n.logged), 1)
            self.assertIn('none', n.logged[0])
            self.assertNotIn('WARNING', n.logged[0])

    def test_real_db_still_waits_and_warns(self):
        n = self.make('utpp_base')
        t0, info = night.time.monotonic(), {}
        self.assertFalse(n.wait_calibration(_G(), self.log, 0, info))
        self.assertGreaterEqual(night.time.monotonic() - t0, n.args.calib_timeout)
        self.assertIs(info['calibration_seen'], False)
        self.assertIn('WARNING', n.logged[0])

    def test_real_db_seen(self):
        with open(self.log, 'a', encoding='utf-8') as f:
            f.write('UTPP speed calibration [utpp_base] done' + chr(10))
        n = self.make('utpp_base')
        info = {}
        self.assertTrue(n.wait_calibration(_G(), self.log, 0, info))
        self.assertIs(info['calibration_seen'], True)
        self.assertEqual(n.logged, [])


class PerfStatusTests(unittest.TestCase):
    OK = '[2026-11-12 03:40:00.000] [info] [1] [UTPP] UTPP CET [night] PERF true  motion enabled=1 frames=9 feedP95us=87.5 budget=250 trav=0'
    OLD = '[2026-11-12 03:30:00.000] [info] [1] [UTPP] UTPP mm.motionDriver status: motion enabled=1 frames=5 feedP95us=70.0 budget=250 trav=0'

    def setUp(self):
        self._time = night.time
        night.time = FakeTime()
        fd, self.log = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.n = EnsureClosedTests.make(EnsureClosedTests(), FakeApi(), None)
        self.n.utpp_log = self.log
        self.n.cycle = {'name': 't', 'steps': []}
        self.n.logged = []
        self.n.log = self.n.logged.append
        self.n.guard = lambda: _PerfG()
        self.n.ensure_overlay = lambda g, expected='closed', where='': self.calls.append(('overlay', expected))
        self.calls = []

    def tearDown(self):
        night.time = self._time
        os.remove(self.log)

    def stub(self, results):
        it = iter(results)

        def cv(pad, g, cmd, echo, label, timeout=3.0):
            self.calls.append(('cv', cmd))
            return next(it)
        self.n.console_verified = cv

    def test_console_first_try(self):
        self.stub([self.OK + chr(10)])
        self.n.perf_status()
        self.assertEqual(self.n.cycle['perf_source'], 'console')
        self.assertTrue(self.n.cycle['perf_status'].startswith('true'))
        self.assertIn('feedP95us=87.5', self.n.cycle['perf_status'])
        self.assertEqual(self.calls, [('overlay', 'closed'), ('cv', gn.PERF_STATUS_CMD)])   # overlay checked first

    def test_budget_fields_come_from_their_own_line(self):
        tail = 'wallP95us=412.0 budgetTrips=2 budgetPauseS=0.00 budgetCoolS=10 threadTime=1'
        self.stub([self.OK + chr(10) + '[2026-11-12 03:40:00.001] [info] [1] [UTPP] UTPP CET [night] PERFB ' + tail + chr(10)])
        self.n.perf_status()
        self.assertEqual(self.n.cycle['perf_budget'], tail)
        self.assertTrue(self.n.cycle['perf_status'].endswith('| ' + tail))
        self.assertIn("string.find(tostring(s),'wallP95us='", gn.PERF_STATUS_CMD)
        # a status without those fields (an older build): the status line alone, no error
        self.stub([self.OK + chr(10)])
        self.n.cycle = {'name': 't', 'steps': []}
        self.n.perf_status()
        self.assertNotIn('perf_budget', self.n.cycle)

    def test_retries_then_succeeds(self):
        self.stub([None, None, self.OK])
        self.n.perf_status()
        self.assertEqual(self.n.cycle['perf_source'], 'console')
        self.assertEqual([c for c in self.calls if c[0] == 'cv'].__len__(), 3)
        self.assertEqual(sum(1 for c in self.calls if c[0] == 'overlay'), 3)

    def test_lost_falls_back_to_log_line(self):
        with open(self.log, 'w', encoding='utf-8') as f:
            f.write(self.OLD + chr(10) + '[x] unrelated' + chr(10))
        self.stub([None, None, None])
        self.n.perf_status()
        self.assertEqual(self.n.cycle['perf_source'], 'log')
        self.assertTrue(self.n.cycle['perf_status'].startswith('motion enabled=1'))
        self.assertIn('feedP95us=70.0', self.n.cycle['perf_status'])
        self.assertEqual(sum(1 for c in self.calls if c[0] == 'cv'), 3)
        json.dumps(self.n.cycle)

    def test_lost_and_no_log_line(self):
        self.stub([None, None, None])
        self.n.perf_status()
        self.assertEqual((self.n.cycle['perf_status'], self.n.cycle['perf_source']), (None, None))
        self.assertTrue(any('no motion status line' in m for m in self.n.logged))

    def test_exception_is_not_fatal(self):
        def boom(*a, **k):
            raise RuntimeError('focus lost')
        self.n.console_verified = boom
        self.n.perf_status()
        self.assertEqual(self.n.cycle['perf_source'], None)

    def test_newest_motion_status_picks_last(self):
        with open(self.log, 'w', encoding='utf-8') as f:
            f.write(self.OLD + chr(10) + self.OK + chr(10))
        self.assertIn('feedP95us=87.5', night.newest_motion_status(self.log))
        self.assertIsNone(night.newest_motion_status(self.log + '.missing'))
        self.assertIsNone(night.newest_motion_status(None))


class _PerfG:
    def ensure_focus(self):
        pass

    def wait(self, s):
        night.time.sleep(s)


class _World:
    """Stub game world for the look-ahead / blocked-walk tests: V on a plane, camera forward cf (a camera turn
    rotates it 90 deg clockwise), an obstacle predicate blocked(x, y), and a play() that integrates the stick tracks
    like the game would (speed_for per stick class), writes a NightAuto POS line every 0.5 s of play time and
    records the run in plan['runs'] exactly like Night.play."""

    def __init__(self, night_obj, log, blocked=None):
        self.n, self.log = night_obj, log
        self.p, self.cf = [0.0, 0.0], (0.0, 1.0)
        self.t = datetime(2026, 11, 12, 10, 0, 0)
        self.blocked = blocked or (lambda x, y: False)
        self.played = []

    def pos(self):
        return {'p': (self.p[0], self.p[1], 0.0), 'cf': self.cf, 'v': None, 'vr': None}

    def poll(self, g, pad=None, timeout=None):
        return self.pos()

    def camera(self, pad, g, plan, known, target, ep, tol=None, why=''):
        self.cf = gn.rotate_stick(*self.cf, 90.0)
        return known

    def write_pos(self, ts):
        line = (f'[{ts.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}] [info] [1] [UTPP] UTPP CET [night] POS '
                f'p={self.p[0]:.2f},{self.p[1]:.2f},0.00 cf={self.cf[0]:.4f},{self.cf[1]:.4f} v=nil src=auto w=0 m=0')
        with open(self.log, 'a', encoding='utf-8') as f:
            f.write(line + chr(10))

    def move(self, ls, dt):
        sx, sy = ls
        mag = math.hypot(sx, sy)
        if mag < 0.05:
            return
        fx, fy = self.cf
        wx, wy = sx * fy + sy * fx, -sx * fx + sy * fy
        n = math.hypot(wx, wy)
        # third person: V turns to the stick and runs at his forward speed in every direction (12:14 run of
        # 2026-11-12: the walk back covered the way out in about the same time), so the forward class speed
        step = gn.speed_for(0.0, mag, False, gn.SPEEDS) * dt
        nx, ny = self.p[0] + wx / n * step, self.p[1] + wy / n * step
        if not self.blocked(nx, ny):
            self.p = [nx, ny]

    def play(self, pad, g, name, plan, sc=None):
        sc = sc or self.n.scenarios[name]
        t0 = self.t
        steps = [{'name': s.name, 'start': t0 + timedelta(seconds=a), 'end': t0 + timedelta(seconds=b),
                  'metrics': s.metrics} for s, (_, a, b) in zip(sc.steps, sc.step_bounds())]
        next_pos = 0.0
        for _, fr in sc.sample(1.0 / 30):
            self.move(fr.ls, 1.0 / 30)
            if fr.t + 1e-9 >= next_pos:
                self.write_pos(t0 + timedelta(seconds=fr.t))
                next_pos += 0.5
        self.t = t0 + timedelta(seconds=sc.duration + 0.3)
        night.time.sleep(sc.duration)
        res = {'steps': steps, 'aborted': None}
        plan['runs'].append({'scenario': sc.name, 'run': 1 + sum(1 for r in plan['runs'] if r['scenario'] == sc.name),
                             **res})
        self.played.append(sc.name)
        return res


def _walk_scenario(name, stick, seconds):
    return parse_scenario(name, {'steps': [{'name': 'walk', 'duration': seconds, 'interp': 'step',
                                            'ls': [[0, 0, 0], [0.02, 0, stick], [seconds, 0, stick]],
                                            'rs': [[0, 0, 0]]}]})


class RouteEstimateTests(unittest.TestCase):
    def test_straight_run_and_short_walk(self):
        est = gn.route_estimate(_walk_scenario('run', 0.9, 4.0), gn.SPEEDS)
        self.assertAlmostEqual(est['leg_m'], 15.2, delta=0.3)
        self.assertEqual(est['leg_heading_deg'], 0.0)
        plan = gn.lookahead_plan(est, gn.SPEEDS)
        self.assertLessEqual(plan['duration'], gn.LOOKAHEAD_MAX_S)                 # capped at 4 s ...
        self.assertLess(plan['duration'], 3.0)                                      # ... and out + back <= 9 s
        self.assertIsNone(gn.lookahead_plan(gn.route_estimate(_walk_scenario('short', 0.4, 3.0), gn.SPEEDS), gn.SPEEDS))

    def test_camera_turn_rotates_the_route_and_sprint_is_faster(self):
        sc = parse_scenario('turn', {'steps': [{'name': 'a', 'duration': 4.0, 'interp': 'step',
                                                'ls': [[0, 0, 0], [0.02, 0, 0.9]], 'rs': [[0, 0, 0], [0.5, 1, 0]]}]})
        est = gn.route_estimate(sc, gn.SPEEDS, cam_rate=90.0)
        self.assertLess(est['net_m'], est['path_m'])                              # the route curves
        sp = parse_scenario('sprint', {'steps': [{'name': 'a', 'duration': 2.0, 'interp': 'step',
                                                  'ls': [[0, 0, 0.9]], 'buttons': [{'t': 0.1, 'button': 'L3'}]}]})
        self.assertGreater(gn.route_estimate(sp, gn.SPEEDS)['path_m'], 1.5 * gn.route_estimate(
            _walk_scenario('w', 0.9, 2.0), gn.SPEEDS)['path_m'])

    def test_real_scenarios_have_a_route(self):
        scs = load_scenarios(SCEN)
        self.assertGreater(gn.route_estimate(scs['weapon_walk'], gn.SPEEDS)['leg_m'], gn.ROUTE_LONG_M)
        self.assertIsNone(gn.lookahead_plan(gn.route_estimate(scs['melee_punch_walk'], gn.SPEEDS), gn.SPEEDS))

    def test_blocked_report(self):
        t0 = datetime(2026, 11, 12, 10, 0, 0)
        at = lambda s: t0 + timedelta(seconds=s)
        still = [(at(s * 0.5), (1.0, 1.0)) for s in range(0, 9)]              # standing still for 4 s
        self.assertTrue(gn.blocked_report(still, [(at(0), at(4))])['blocked'])
        self.assertFalse(gn.blocked_report(still, [])['blocked'])               # stick not pushed: not blocked
        self.assertFalse(gn.blocked_report(still, [(at(3), at(4))])['blocked'])  # only 1 s pushed
        moving = [(at(s * 0.5), (s * 1.0, 0.0)) for s in range(0, 9)]
        self.assertFalse(gn.blocked_report(moving, [(at(0), at(4))])['blocked'])
        txt = chr(10).join(f'[2026-11-12 10:00:0{s}.000] [info] [1] [UTPP] UTPP CET [night] POS p={s}.50,2.00,0.00 '
                           'cf=0.0000,1.0000 v=nil src=auto w=0 m=0' for s in range(3)) + chr(10) + 'noise' + chr(10)
        self.assertEqual([p for _, p in gn.pos_samples(txt)], [(0.5, 2.0), (1.5, 2.0), (2.5, 2.0)])

    def test_aiming_is_no_pushed_window_for_the_stall_check(self):
        # the aim walk of the weapon macro moves at about 0.3 m/s at part stick: slow by design, not a stall
        sc = load_scenarios(SCEN)['weapon_aim']
        t0 = datetime(2026, 11, 14, 8, 0, 0)
        bounds = sc.step_bounds()                                                # (name, start, end)
        starts = [{'start': t0 + timedelta(seconds=b[1])} for b in bounds]
        aim0 = t0 + timedelta(seconds=next(b[1] for b in bounds if b[0] == 'aim_walk'))
        iv = gn.pushed_intervals(sc, starts)
        self.assertTrue(iv)                                                      # the armed walk before the aim is checked
        held = [(a, b) for a, b in iv if b > aim0 + timedelta(seconds=0.3) and a < aim0 + timedelta(seconds=2.0)]
        self.assertEqual(held, [], iv)                                           # nothing inside the held trigger
        self.assertLess(gn.AIM_HELD_MIN, 1.0)


class LookAheadTests(unittest.TestCase):
    """clear_path look-ahead and play_walking repeat on a stub world (no game, no input)."""

    def setUp(self):
        self._time = night.time
        night.time = FakeTime()
        fd, self.log = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.n = EnsureClosedTests.make(EnsureClosedTests(), FakeApi(), None)
        self.n.utpp_log = self.log
        self.n.auto_pos = True
        self.n.cycle = {'name': 't', 'steps': []}
        self.n.logged = []
        self.n.log = self.n.logged.append
        self.n.recoveries = []
        self.n.recover = lambda *a, **k: self.n.recoveries.append(a[3]) or {'suspect': False}

    def tearDown(self):
        night.time = self._time
        os.remove(self.log)

    def world(self, scenarios, blocked=None):
        self.n.scenarios = {s.name: s for s in scenarios}
        w = _World(self.n, self.log, blocked)
        self.n.poll_pos, self.n.play, self.n.camera_align = w.poll, w.play, w.camera
        return w

    def test_wall_two_metres_ahead_is_seen_and_the_start_turns_away(self):
        w = self.world([_walk_scenario('long_route', 0.9, 4.0)], blocked=lambda x, y: y > 2.0)
        plan = {'runs': []}
        out = self.n.clear_path(_Pad(), _G(), plan, 'long_route')
        self.assertEqual(out['result'], 'clear')
        walks = out['lookahead']['walks']
        self.assertEqual([k['result'] for k in walks], ['blocked', 'clear'])
        self.assertGreaterEqual(walks[0]['stall_s'], gn.LOOKAHEAD_STALL_S)
        self.assertEqual(w.cf, (1.0, 0.0))                                      # camera turned +90 deg: free ground
        self.assertLessEqual(walks[1]['back']['residual_m'], gn.LOOKAHEAD_BACK_TOL_M)   # back at the start (residual logged)
        self.assertTrue(all(r['scenario'] == 'clear_probe' for r in plan['runs']))
        self.assertTrue(any('probe ' in m and ' s' in m and 'look-ahead' in m for m in self.n.logged), self.n.logged)
        self.assertGreater(out['lookahead']['probe_s'], 0)

    def test_route_on_which_v_leaves_the_ground_is_treated_like_an_obstacle(self):
        w = self.world([_walk_scenario('long_route', 0.9, 4.0)])
        play, state = w.play, {'n': 0}

        def play_with_drop(*a, **k):
            res = play(*a, **k)
            state['n'] += 1
            if state['n'] == 1:                       # the first look-ahead walk crosses a ledge
                with open(self.log, 'a', encoding='utf-8') as f:
                    f.write('[2026-11-14 05:57:03.701] [info] [1] [UTPP] UTPP MMAIR phase=LandApproach was=Ground action=land '
                            'onG=0 vz=-1.95\n')
            return res
        self.n.play = play_with_drop
        out = self.n.clear_path(_Pad(), _G(), {'runs': []}, 'long_route')
        self.assertEqual(out['result'], 'clear')
        walks = out['lookahead']['walks']
        self.assertEqual([k['result'] for k in walks], ['blocked', 'clear'])
        self.assertTrue(walks[0].get('drop'))
        self.assertEqual(w.cf, (1.0, 0.0))                                      # camera turned away from the drop
        self.assertTrue(any('a drop on the route' in m for m in self.n.logged), self.n.logged)
        self.assertFalse(gn.left_ground('UTPP MMAIR phase=Ground was=Recover action=handBack onG=1'))   # coming back is no drop

    def test_clear_field_long_route_costs_one_look_ahead_and_returns_to_the_start(self):
        w = self.world([_walk_scenario('long_route', 0.9, 4.0)])
        out = self.n.clear_path(_Pad(), _G(), {'runs': []}, 'long_route')
        self.assertEqual(out['result'], 'clear')
        self.assertEqual(w.cf, (0.0, 1.0))                                      # camera untouched
        self.assertLess(math.hypot(*w.p), gn.LOOKAHEAD_BACK_TOL_M)              # residual start offset
        self.assertLess(out['lookahead']['probe_s'], 30.0)

    def test_short_route_keeps_the_short_probe(self):
        w = self.world([_walk_scenario('short_route', 0.4, 3.0)])
        plan = {'runs': []}
        out = self.n.clear_path(_Pad(), _G(), plan, 'short_route')
        self.assertEqual(out['result'], 'clear')
        self.assertNotIn('lookahead', out)
        self.assertEqual(len(plan['runs']), 1)                                  # one 1.2 s probe, no walk back
        self.assertAlmostEqual(plan['runs'][0]['steps'][0]['end'].timestamp() - plan['runs'][0]['steps'][0]['start'].timestamp(),
                               night.CLEAR_PROBE_S + 0.6, places=3)

    def test_second_look_ahead_from_the_same_spot_is_skipped_and_only_the_face_push_is_played(self):
        w = self.world([_walk_scenario('long_route', 0.9, 4.0), _walk_scenario('again', 0.9, 4.0)])
        plan = {'runs': []}
        first = self.n.clear_path(_Pad(), _G(), plan, 'long_route')
        walked = len(plan['runs'])
        self.assertNotIn('skipped', first['lookahead'])
        self.assertEqual([k['p0'] for k in first['lookahead']['walks']], [[0.0, 0.0]])      # start and camera are recorded
        self.assertEqual(first['lookahead']['walks'][0]['cf'], [0.0, 1.0])
        out = self.n.clear_path(_Pad(), _G(), plan, 'again')
        self.assertEqual(out['result'], 'clear')
        self.assertIn('skipped', out['lookahead'])
        self.assertEqual(out['lookahead']['skipped']['where'], 'long_route')
        self.assertEqual([r['steps'][0]['name'] for r in plan['runs'][walked:]], ['face'])   # no probe walk, no walk back
        self.assertTrue(any('SKIPPED' in m and 'face push only' in m for m in self.n.logged), self.n.logged)
        self.assertEqual(self.n.probe_cache.skipped, 1)

    def test_probe_is_repeated_when_v_is_elsewhere_or_the_camera_turned_or_the_route_differs(self):
        w = self.world([_walk_scenario('long_route', 0.9, 4.0), _walk_scenario('again', 0.9, 4.0),
                        _walk_scenario('side', 0.9, 4.0)])
        plan = {'runs': []}
        self.n.clear_path(_Pad(), _G(), plan, 'long_route')
        w.p = [3.0, 0.0]                                                         # 3 m from the probe start
        far = self.n.clear_path(_Pad(), _G(), plan, 'again')
        self.assertNotIn('skipped', far['lookahead'])
        self.n.turn_camera_90(_Pad(), _G(), plan, w.pos())                       # the camera turn forgets every probe
        self.assertEqual(self.n.probe_cache.entries, [])
        self.assertIn('camera turned (clear path)', self.n.probe_cache.invalidated)
        w.p = [0.0, 0.0]
        again = self.n.clear_path(_Pad(), _G(), plan, 'side')
        self.assertNotIn('skipped', again['lookahead'])
        self.assertEqual(self.n.probe_cache.skipped, 0)

    def test_blocked_attempt_forgets_the_probes_and_the_budget_counts_the_retry(self):
        w = self.world([_walk_scenario('mid', 0.4, 3.0)], blocked=lambda x, y: y > 2.8 and w.cf == (0.0, 1.0))
        self.n.probe_cache.remember('earlier', (0.0, 0.0), (0.0, 1.0), {'heading_deg': 0.0, 'duration': 2.0, 'reach_m': 6.0}, 5.0)
        self.n.budget.begin('mid#1')
        self.n.play_walking(_Pad(), _G(), {'runs': []}, 'mid')
        self.assertIn('mid was blocked', self.n.probe_cache.invalidated)
        row = self.n.budget.rows()[0]
        self.assertTrue(row['retry'].startswith('1/'), row)

    def test_blocked_mid_route_is_repeated_once_and_only_the_second_attempt_is_scored(self):
        w = self.world([_walk_scenario('mid', 0.4, 3.0)], blocked=lambda x, y: y > 2.8 and w.cf == (0.0, 1.0))
        plan = {'runs': []}
        att = self.n.play_walking(_Pad(), _G(), plan, 'mid')
        mids = [r for r in plan['runs'] if r['scenario'] == 'mid']
        self.assertEqual([r['run'] for r in mids], [1])                         # the repeat is run 1: the first is gone
        self.assertNotIn('blocked_repeat', mids[0])
        disc = plan['discarded_runs']
        self.assertEqual(len(disc), 1)
        self.assertTrue(disc[0]['blocked_repeat'])
        self.assertGreaterEqual(disc[0]['blocked']['stall_s'], gn.BLOCK_MIN_S)
        self.assertEqual(w.played.count('mid'), 2)
        self.assertEqual(w.cf, (1.0, 0.0))                                      # the repeat went into free ground
        self.assertIs(att['res']['steps'], mids[0]['steps'])
        self.assertEqual(att['discarded'], self.n.cycle['blocked_repeats'][0])
        self.assertGreaterEqual(att['start'], att['discarded']['end'])          # the kept attempt is the later one
        self.assertEqual(self.n.recoveries, ['blocked mid'])
        self.assertTrue(any(m.startswith('[blocked] mid') for m in self.n.logged))
        json.dumps(night.gt_run.serialise_plan(plan), default=str)               # plan.json stays serialisable

    def test_blocked_twice_keeps_the_second_attempt_for_run_py(self):
        w = self.world([_walk_scenario('boxed', 0.4, 3.0)], blocked=lambda x, y: math.hypot(x, y) > 0.2)
        plan = {'runs': []}
        att = self.n.play_walking(_Pad(), _G(), plan, 'boxed')
        boxed = [r for r in plan['runs'] if r['scenario'] == 'boxed']
        self.assertEqual(len(boxed), 1)                                          # exactly one entry: the second attempt
        self.assertIn('blocked_live', boxed[0])
        self.assertEqual(len(plan['discarded_runs']), 1)
        self.assertEqual(w.played.count('boxed'), 2)                             # played twice, never a third time
        self.assertTrue(any('blocked again' in m for m in self.n.logged))
        self.assertEqual(len(self.n.cycle['blocked_repeats']), 1)

    def test_unblocked_scenario_leaves_the_plan_untouched(self):
        w = self.world([_walk_scenario('free', 0.4, 3.0)])
        plan = {'runs': []}
        att = self.n.play_walking(_Pad(), _G(), plan, 'free')
        self.assertIsNone(att['discarded'])
        self.assertEqual(set(plan), {'runs'})                                    # no discarded_runs key
        self.assertNotIn('blocked_repeats', self.n.cycle)
        run = next(r for r in plan['runs'] if r['scenario'] == 'free')
        self.assertEqual(set(run), {'scenario', 'run', 'steps', 'aborted'})      # no new keys on the scored run
        self.assertEqual(self.n.recoveries, [])

    def test_seated_v_leaves_the_vehicle_before_a_walking_scenario(self):
        w = self.world([_walk_scenario('free', 0.4, 3.0)])
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(self.log, 'a', encoding='utf-8') as f:                         # the newest NightAuto line: seated
            f.write(f'[{stamp}] [info    ] [ 80716] [UTPP] UTPP CET [night] POS p=1574.85,2258.13,182.53 '
                    'cf=0.424,-0.906 v=1574.86,2258.15,182.14,-0.905,-0.425 src=mounted w=0 m=1 freeMove=0/0x9 '
                    'psmW=0 ub=0 wi=none n=1' + chr(10))
        calls = []
        self.n.ensure_unmounted = (lambda pad, g, plan, why, pos=None:
                                   calls.append((why, pos.get('mounted'), list(w.played))) or True)
        self.n.play_walking(_Pad(), _G(), {'runs': []}, 'free', clear=False)
        self.assertEqual(calls, [('before free', True, [])])                    # asked once, before the scenario played
        self.assertTrue(self.n.left_vehicle_this_cycle)
        self.assertEqual(w.played.count('free'), 1)
        self.assertTrue(any('sits in a vehicle' in m for m in self.n.logged))

    def test_without_night_auto_nothing_is_detected(self):
        self.n.auto_pos = False
        w = self.world([_walk_scenario('mid', 0.4, 3.0)], blocked=lambda x, y: y > 2.8)
        plan = {'runs': []}
        att = self.n.play_walking(_Pad(), _G(), plan, 'mid', clear=False)
        self.assertEqual(w.played.count('mid'), 1)
        self.assertNotIn('discarded_runs', plan)

    def test_run_py_ignores_discarded_runs(self):
        t0 = datetime(2026, 11, 12, 10, 0, 0)
        run = {'scenario': 'mid', 'run': 1, 'aborted': None,
               'steps': [{'name': 'walk', 'start': t0, 'end': t0 + timedelta(seconds=3), 'metrics': True}]}
        dead = dict(run, run=1, blocked_repeat=True)
        base = night.gt_run.analyze({'runs': [run]}, [], {}, {})
        with_disc = night.gt_run.analyze({'runs': [run], 'discarded_runs': [dead]}, [], {}, {})
        self.assertEqual(len(base), 1)
        self.assertEqual(json.dumps(base, default=str), json.dumps(with_disc, default=str))


class EnemyPureTests(unittest.TestCase):
    """gt_night: the fist fight target (melee_fists_target) - sets, command texts, echo parsing, verdict."""

    def test_set_and_macro_parts(self):
        sc = load_scenarios(SCEN)
        self.assertEqual(gn.resolve_sets('melee_fists_target'), ['melee_fists_target'])
        self.assertEqual(gn.MACROS['melee_fists_target'], gn.ENEMY_PARTS)
        self.assertNotIn('melee_fists_target', gn.SETS['all'])                      # needs a Codeware NPC: explicit only
        self.assertEqual(gn.validate_sets(['melee_fists_target'], sc), [])
        self.assertNotIn('melee_fists_target', sc)                                   # a night.py macro, no pad scenario
        missing = {k: v for k, v in sc.items() if k != 'melee_block'}
        self.assertIn('melee_block', gn.validate_sets(['melee_fists_target'], missing)[0])
        self.assertTrue(all(part in sc for *_, part in gn.ENEMY_PHASES))
        self.assertEqual([p[0] for p in gn.ENEMY_PHASES], ['far', 'near', 'orbit', 'block'])
        self.assertEqual([p[1] for p in gn.ENEMY_PHASES], [1.8, 1.2, 1.2, 1.5])
        self.assertEqual([p[0] for p in gn.ENEMY_PHASES if p[2]], ['near', 'orbit'])  # the health drop required at 1.2 m

    def test_commands_are_one_ascii_line_with_their_own_tag(self):
        cmds = [gn.enemy_spawn_cmd('Character.x'), gn.enemy_cmd('far', 2.0, 1, define=True),
                gn.enemy_place_cmd('Character.x', 1.2, 'after_far', define=True), gn.enemy_off_cmd(), gn.ENEMY_DEFINE]
        for c in cmds:
            self.assertNotIn(chr(10), c)
            self.assertTrue(all(32 <= ord(ch) < 127 for ch in c))
        self.assertNotEqual(gn.ENEMY_TAG, gn.NIGHT_TAG)                              # NightPos / mount read NIGHT_TAG as the car
        self.assertNotIn(gn.NIGHT_TAG, ''.join(cmds))
        sp = gn.enemy_spawn_cmd('Character.humanoid_tutorial_idle_ma', 6.0)
        for part in ("TweakDBID.new('Character.humanoid_tutorial_idle_ma')", 's.spawnInView=true', 's.active=true',
                     's.persistState=false', 's.persistSpawn=false', "s.tags={'" + gn.ENEMY_TAG + "','" + gn.ENEMY_TAG + "'..UTPPENEMYSEQ}",
                     'a.x+x*6.00',
                     "TweakDB:GetRecord(id)", 'record missing', "[night] ENEMY_SPAWN ok="):
            self.assertIn(part, sp)
        self.assertIn('Immortal', gn.ENEMY_DEFINE)
        self.assertIn('AIA_Neutral', gn.ENEMY_DEFINE)
        self.assertNotIn('Invulnerable', gn.ENEMY_DEFINE + gn.enemy_off_cmd())      # Invulnerable would block the damage
        self.assertNotIn('Teleport', ''.join(cmds))                                  # a target is placed by spawning it
        self.assertIn('RemoveGodMode', gn.enemy_off_cmd())
        self.assertIn("DeleteTagged('" + gn.ENEMY_TAG + "')", gn.enemy_off_cmd())
        self.assertTrue(gn.enemy_off_cmd().endswith("Game.UTPPLogLine('[night] ENEMY_OFF ok=true')"))
        c = gn.enemy_cmd('near', 1.2, 1)
        self.assertTrue(c.startswith('local ok,e=pcall(function() return UTPPENEMY(1.20,1) end)'))
        self.assertIn('label=near', c)
        self.assertTrue(gn.enemy_cmd('x', define=True).startswith('UTPPENEMY=function(W,P)'))
        self.assertIn('UTPPENEMY(nil,nil)', gn.enemy_cmd('wait'))
        pl = gn.enemy_place_cmd('Character.x', 1.2, 'after_far')
        self.assertTrue(pl.startswith('local ok,e=pcall(function() return UTPPENEMY(nil,nil) end)'))   # the old target is read first
        self.assertIn('label=after_far', pl)
        self.assertTrue(pl.endswith(gn.enemy_spawn_cmd('Character.x', 1.2)[-80:]))                        # then the spawn, its echo last
        self.assertLess(pl.index("'[night] ENEMY ok='"), pl.index('DeleteTagged'))                      # read before the delete
        self.assertNotIn('UTPPENEMY=function', pl)
        self.assertTrue(gn.enemy_place_cmd('Character.x', 1.2, 'x', define=True).startswith('UTPPENEMY=function'))
        # typed at ~12 ms per character without the file channel: later calls are short, the one-off commands stay small
        self.assertLess(len(gn.enemy_cmd('placed', 2.0, 1)), 200)
        self.assertLess(len(gn.ENEMY_DEFINE), 2200)
        self.assertLess(len(gn.enemy_spawn_cmd('Character.humanoid_tutorial_idle_ma')), 900)
        self.assertLess(len(gn.enemy_place_cmd('Character.humanoid_tutorial_idle_ma', 1.2, 'after_orbit')), 1200)
        self.assertLess(len(gn.enemy_off_cmd()), 600)
        self.assertEqual(gn.nightauto_cmd(0.5), 'GetMod("utpp_core").NightAuto(true, 0.50, "UTPPNight")')   # unchanged
        self.assertEqual(gn.nightauto_cmd(0.2, True, gn.ENEMY_TAG),
                         'GetMod("utpp_core").NightAuto(true, 0.20, "' + gn.ENEMY_TAG + '")')

    def test_parse_enemy(self):
        txt = ('[2026-11-13 10:00:00.000] [info] [1] [UTPP] UTPP CET [night] ENEMY ok=true label=placed found=1 pre_d2=2.31 '
               'pre_hv=177.0 pre_hp=88.5 att=2 act=true at=1 god=true att2=2')
        e = gn.parse_enemy(txt)
        self.assertEqual((e['ok'], e['label'], e['found'], e['d2'], e['hp'], e['hv']), (True, 'placed', True, 2.31, 88.5, 177.0))
        self.assertEqual((e['att'], e['att_name'], e['act'], e['at'], e['god'], e['att_after']), (2, 'hostile', True, True, True, 2))
        self.assertIsNone(e['attset'])
        e = gn.parse_enemy('[night] ENEMY ok=true label=p found=1 pre_d2=6.00 att=0 act=true at=0')
        self.assertEqual((e['at'], e['god'], e['att_after']), (False, None, None))
        e = gn.parse_enemy('[night] ENEMY ok=true label=p found=1 pre_d2=2.00 att=AIA_Friendly act=true at=1 god=true '
                           'attset=true att2=nil')
        self.assertEqual((e['att'], e['attset'], e['att_after']), (0, True, None))
        e = gn.parse_enemy('[night] ENEMY ok=true label=w found=1 pre_d2=6.03 pre_hv=482.4 pre_hp=100.0 act=true')
        self.assertEqual((e['att'], e['at']), (None, None))                          # the first real echo: no attitude, no distance asked
        self.assertIsNone(gn.parse_enemy('[night] ENEMY_SPAWN ok=true Character.x'))     # another tag is not this echo
        self.assertIsNone(gn.parse_enemy(''))
        gone = gn.parse_enemy("[night] ENEMY ok=false label=far [string]:1: attempt to call global 'UTPPENEMY' (a nil value)")
        self.assertFalse(gone['ok'])
        self.assertFalse(gn.parse_enemy('[night] ENEMY ok=true label=w found=0')['found'])
        e = gn.parse_enemy('[night] ENEMY ok=true label=w found=1 pre_d2=1.00 pre_hp=- pre_hv=- att=AIA_Friendly act=0')
        self.assertEqual((e['hp'], e['hv'], e['att'], e['act']), (None, None, 0, False))   # unreadable / enum text
        last = gn.parse_enemy('[night] ENEMY ok=true label=a found=1\n[night] ENEMY ok=true label=b found=0')
        self.assertEqual(last['label'], 'b')

    def test_hp_drop(self):
        self.assertEqual(gn.enemy_hp_drop({'hv': 100.0, 'hp': 100.0}, {'hv': 70.0, 'hp': 70.0}), 30.0)
        self.assertEqual(gn.enemy_hp_drop({'hv': None, 'hp': 100.0}, {'hv': None, 'hp': 91.0}), 9.0)   # percent fallback
        self.assertIsNone(gn.enemy_hp_drop({'hv': None, 'hp': None}, {'hv': 1.0, 'hp': 1.0}))
        self.assertIsNone(gn.enemy_hp_drop(None, {'hv': 1.0}))
        self.assertIsNone(gn.enemy_hp_drop({'hv': 1.0}, None))

    def test_meleeaim_stats_and_distances(self):
        def aim(src, tp, ty, p=0.0, y=0.0, rp=None, ry=None):
            return (f'[2026-11-13 10:00:00.000] [info    ] [ 1] [UTPP] UTPP MELEEAIM src={src} tiltW=1.00 tgtPitch={tp} tgtYaw={ty} '
                    f'rawPitch={tp if rp is None else rp} rawYaw={ty if ry is None else ry} pitch={p} yaw={y} flags=0x0 '
                    'maxUp=20 maxDown=35 hz=10 calls=1 applied=1 busy=0 reapplied=0 fails=0')
        text = chr(10).join([aim('cam', -14.0, 1.0), aim('lock', -3.5, 12.0, -3.0, 11.0),
                             aim('lock', 0.5, -9.0, 0.5, -8.0, rp=2.5, ry=-61.5)])     # tgtYaw is clamped, rawYaw is not
        st = gn.meleeaim_stats(text)
        self.assertEqual((st['lines'], st['src'], st['lock']), (3, {'cam': 1, 'lock': 2}, 2))
        self.assertEqual((st['lock_pitch'], st['lock_yaw']), ((-3.5, 0.5), (-9.0, 12.0)))
        self.assertEqual((st['lock_raw_pitch'], st['lock_raw_yaw']), ((-3.5, 2.5), (-61.5, 12.0)))
        self.assertEqual((st['applied_pitch_max'], st['applied_yaw_max']), (3.0, 11.0))
        empty = gn.meleeaim_stats('nothing')
        self.assertEqual((empty['lines'], empty['lock'], empty['lock_pitch'], empty['lock_raw_yaw']), (0, 0, None, None))
        pos = chr(10).join([
            '[2026-11-13 10:00:00.000] [info] [1] [UTPP] UTPP CET [night] POS p=0.00,0.00,0.00 cf=0.000,1.000 v=0.00,2.00,0.00,1.000,0.000 src=tag w=0 m=0 n=1',
            '[2026-11-13 10:00:00.200] [info] [1] [UTPP] UTPP CET [night] POS p=0.00,0.50,0.00 cf=0.000,1.000 v=0.00,2.00,0.00,1.000,0.000 src=tag w=0 m=0 n=2',
            '[2026-11-13 10:00:00.400] [info] [1] [UTPP] UTPP CET [night] POS p=0.00,0.50,0.00 cf=0.000,1.000 v=nil src=none w=0 m=0 n=3'])
        ds = gn.target_distance_stats(pos)
        self.assertEqual(ds, {'n': 2, 'first': 2.0, 'min': 1.5, 'max': 2.0, 'last': 1.5})
        self.assertIsNone(gn.target_distance_stats('[night] POS p=0.00,0.00,0.00 cf=0.000,1.000 v=nil src=none'))

    def test_evaluate(self):
        spawn = {'found': True, 'att': 2, 'act': True}

        def ph(name, lock, drop, required=False, placed=True, lines=3):
            return {'name': name, 'required': required, 'lock': lock, 'drop': drop, 'placed': placed,
                    'aim': {'lines': lines, 'src': {k: v for k, v in (('lock', lock), ('cam', lines - lock)) if v}}}
        good = [ph('far', 0, 0.0), ph('near', 2, 30.0, True), ph('orbit', 1, 30.0, True), ph('block', 0, 0.0)]
        r = gn.enemy_evaluate(spawn, good)
        self.assertEqual((r['fails'], r['warns'], r['locks']), ([], [], 3))
        self.assertEqual(r['hits'], {'far': False, 'near': True, 'orbit': True, 'block': False})
        r = gn.enemy_evaluate(spawn, [ph('far', 0, 0.0), ph('near', 0, 30.0, True)])
        self.assertIn('no `src=lock`', r['fails'][0])
        self.assertIn("{'cam': 6}", r['fails'][0])
        r = gn.enemy_evaluate(spawn, [ph('near', 0, 30.0, True, lines=0)])
        self.assertIn('no `MELEEAIM` line at all', r['fails'][0])
        r = gn.enemy_evaluate(spawn, [ph('near', 1, 0.0, True)])
        self.assertTrue(r['fails'] and 'health did not drop' in r['fails'][0])
        r = gn.enemy_evaluate(spawn, [ph('near', 1, None, True)])
        self.assertIn('health unreadable', r['fails'][0])
        r = gn.enemy_evaluate(spawn, [ph('near', 1, 30.0, True), ph('orbit', 0, 0.0, True)])
        self.assertEqual(len(r['fails']), 1)                                         # orbit is a close-range phase too
        self.assertIn('orbit: health did not drop', r['fails'][0])
        self.assertIn('does not follow the camera', r['fails'][0])
        r = gn.enemy_evaluate(spawn, [ph('near', 1, 30.0, True), ph('far', 0, 0.0), ph('block', 0, 0.0)])
        self.assertEqual((r['fails'], r['warns']), ([], []))                         # far / block: report only
        r = gn.enemy_evaluate(spawn, [ph('near', 1, 0.0, True, placed=False)])
        self.assertEqual(r['fails'], ['near: the target was not placed at the wanted distance'])   # no lock / health noise
        bad = dict(ph('near', 0, None, True, placed=False, lines=0), why='the target is 6.0 m from V, wanted 1.2 m')
        r = gn.enemy_evaluate(spawn, [bad, dict(bad, name='far', required=False)])
        self.assertEqual(r['fails'], ['near: the target is 6.0 m from V, wanted 1.2 m'])
        self.assertEqual(r['warns'], ['far: the target is 6.0 m from V, wanted 1.2 m'])
        r = gn.enemy_evaluate(dict(spawn, att=0), [])
        self.assertIn('Friendly', r['fails'][0])
        self.assertEqual(len(r['fails']), 1)                                         # no phases: no lock noise
        r = gn.enemy_evaluate(None, [])
        self.assertIn('spawn failed', r['fails'][0])
        r = gn.enemy_evaluate({'found': False}, [])
        self.assertIn('never found', r['fails'][0])
        r = gn.enemy_evaluate({'found': False, 'why': 'target: never placed (x)'}, [])
        self.assertEqual(r['fails'], ['target: never placed (x)'])
        r = gn.enemy_evaluate(dict(spawn, act=False, att=None), [ph('near', 1, 30.0, True)])
        self.assertEqual(r['fails'], [])
        self.assertEqual(len(r['warns']), 2)

    def test_distance_stats_of_a_target_that_never_came_close(self):
        # POS lines of a run where the target stayed on its spawn point while V drifted towards it by punch lunges
        pos = chr(10).join(
            f'[2026-11-14 00:43:{t}] [info    ] [ 60760] [UTPP] UTPP CET [night] POS p=1576.08,{y},181.90 cf=0.099,-0.976 '
            'v=1576.69,2251.14,182.00,0.995,0.101 src=tag w=1 m=0 freeMove=1/0x0 psmW=0 ub=0 wi=Items.w_melee_004__fists_a n=1'
            for t, y in (('52.139', '2257.11'), ('56.373', '2257.17'), ('58.038', '2256.23'), ('62.468', '2255.92')))
        ds = gn.target_distance_stats(pos)
        self.assertEqual(ds['n'], 4)
        self.assertLess(abs(ds['first'] - 6.0), 0.05)                                # never near the wanted 2.0 / 1.2 m
        self.assertGreater(ds['min'], 4.5)
        e = gn.parse_enemy('[2026-11-14 00:43:59.163] [info    ] [ 60760] [UTPP] UTPP CET [night] ENEMY ok=true label=near '
                           'found=1 pre_d2=5.11 pre_hv=482.4 pre_hp=100.0 act=true heal=true moved=true')
        self.assertTrue(e['ok'] and e['found'])
        self.assertIsNone(e['at'])                                                   # no distance asked: no verdict on it

    def test_punch_report_cmd(self):
        self.assertEqual(gn.punch_report_cmd('py', 'a.py', 'x.log'), ['py', '-I', 'a.py', 'x.log'])

    def test_options(self):
        a = night.build_parser().parse_args(['--set', 'melee_fists_target'])
        self.assertEqual(a.enemy_record.split(',')[0], 'Character.humanoid_tutorial_idle_ma')
        self.assertEqual((a.enemy_spawn_wait, a.punch_report), (4.0, False))
        self.assertFalse(hasattr(a, 'enemy_spawn_dist'))
        a = night.build_parser().parse_args(['--enemy-record', 'Character.a,Character.b', '--punch-report'])
        self.assertEqual(a.enemy_record, 'Character.a,Character.b')
        self.assertTrue(a.punch_report)


def _lua_dll():
    d = HERE
    for _ in range(6):
        cand = os.path.join(d, 'Tools', 'lua-5.1.5', 'lua51.dll')
        if os.path.isfile(cand):
            return cand
        d = os.path.dirname(d)
    return None


# A tiny CET: just enough of the game API for the enemy commands (their real names and call shapes). The commands are
# plain Lua 5.1, so the official interpreter runs them: syntax, control flow, string building and the echo are checked here;
# only the engine side (what the real Game / TweakDB do) stays an assumption. An entity only ever sits where it was spawned.
_LUA_STUB = r"""
LOG={} MISSING={} ATT=2 TAGGED={} HEALTH=80 ATTFAIL=false ATTTXT=nil
Game={} function Game.UTPPLogLine(s) LOG[#LOG+1]=s end
local function V4(x,y,z,w) local v={x=x,y=y,z=z,w=w} v.ToRotation=function(self) return {yaw=math.deg(math.atan2(-self.x,self.y))} end return v end
Vector4={new=V4}
EulerAngles={new=function(p,r,y) return {yaw=y,ToQuat=function() return {yaw=y} end} end}
TweakDBID={new=function(n) return {name=n} end}
TweakDB={GetRecord=function(self,id) if MISSING[id.name] then return nil end return {name=id.name} end}
Enum={new=function(t,v) return t..'.'..v end}
EnumInt=function(x) return x end
CName={new=function(n) return n end}
DynamicEntitySpec={new=function() return {} end}
PLAYER={pos=V4(0,0,0,1)}
function PLAYER:GetWorldPosition() return self.pos end
function PLAYER:GetAttitudeAgent() return 'pagent' end
Game.GetPlayer=function() return PLAYER end
CAMF=V4(0,1,0,0)
Game.GetCameraSystem=function() return {GetActiveCameraForward=function() return CAMF end} end
local function npc(pos,yaw)
  local n={pos=pos,yaw=yaw}
  function n:GetWorldPosition() return self.pos end
  function n:GetEntityID() return 'npc1' end
  function n:GetAttitudeTowards(p) if ATTFAIL then error('no attitude') end return ATT end
  function n:IsActive() return true end
  function n:GetAttitudeAgent() return {SetAttitudeTowards=function(self,a,att) ATT=1 ATTTXT=nil end,
    GetAttitudeTowards=function(self,o) if ATTTXT then return ATTTXT end return ATT end} end
  return n
end
ASYNC=false QUEUE={}
function TICK() for _,f in ipairs(QUEUE) do f() end QUEUE={} end
local function later(f) if ASYNC then QUEUE[#QUEUE+1]=f else f() end end
DES={DeleteTagged=function(self,t) later(function() TAGGED[t]=nil end) end,
  CreateEntity=function(self,s) later(function() local e={npc(s.position,s.orientation.yaw)}
    for _,tg in ipairs(s.tags) do TAGGED[tg]=e end end) return 'id' end,
  GetTagged=function(self,t) return TAGGED[t] or {} end}
Game.GetDynamicEntitySystem=function() return DES end
POOL={GetStatPoolValue=function(self,i,t,perc) return HEALTH end}
Game.GetStatPoolsSystem=function() return POOL end
Game.GetGodModeSystem=function() return {AddGodMode=function(self,id,t,s) GODSET=t..'/'..s end,
  RemoveGodMode=function(self,id,t,s) GODREM=t..'/'..s end} end
"""


class EnemyLuaTests(unittest.TestCase):
    """The enemy console commands executed by the official Lua 5.1 against the stub above (skipped without the DLL)."""

    @classmethod
    def setUpClass(cls):
        import ctypes
        dll = _lua_dll()
        if dll is None:
            raise unittest.SkipTest('Tools\\lua-5.1.5\\lua51.dll not found')
        cls.ct = ctypes
        lua = ctypes.CDLL(dll)
        lua.luaL_newstate.restype = ctypes.c_void_p
        lua.luaL_openlibs.argtypes = [ctypes.c_void_p]
        lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lua.lua_pcall.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        lua.lua_tolstring.restype = ctypes.c_char_p
        lua.lua_settop.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lua.lua_close.argtypes = [ctypes.c_void_p]
        cls.lua = lua

    def setUp(self):
        self.L = self.lua.luaL_newstate()
        self.lua.luaL_openlibs(self.L)
        self.run_chunk(_LUA_STUB)

    def tearDown(self):
        self.lua.lua_close(self.L)

    def run_chunk(self, code, want=False):
        lua = self.lua
        rc = lua.luaL_loadstring(self.L, code.encode('ascii'))
        if rc == 0:
            rc = lua.lua_pcall(self.L, 0, 1 if want else 0, 0)
        out = lua.lua_tolstring(self.L, -1, None) if (rc or want) else b''
        if rc:
            self.fail(f'Lua error: {out.decode(errors="replace")}\n{code[:200]}')
        lua.lua_settop(self.L, 0)
        return out.decode()

    def log(self):
        return self.run_chunk("return table.concat(LOG, '\\n')", want=True)

    def lua_str(self, expr):
        return self.run_chunk(f'return tostring({expr})', want=True)

    def entity(self, field):
        return self.lua_str(f"string.format('%.2f', TAGGED['{gn.ENEMY_TAG}'][1].{field})")

    def test_place_verify_prep_off(self):
        self.run_chunk(gn.enemy_place_cmd('Character.humanoid_tutorial_idle_ma', 2.0, 'after_start', define=True))
        log = self.log()
        self.assertIn('[night] ENEMY ok=true label=after_start found=0', log)       # nothing to read yet
        self.assertIn('[night] ENEMY_SPAWN ok=true', log)
        self.assertTrue(log.rstrip().endswith('ok=true nil'))                        # the spawn echo is the last line
        # spawned 2 m ahead along the camera forward (0,1), facing V (yaw 180), 0.1 m above V
        self.assertEqual((self.entity('pos.y'), self.entity('pos.x'), self.entity('pos.z')), ('2.00', '0.00', '0.10'))
        self.assertEqual(self.entity('yaw'), '180.00')
        self.run_chunk('ATT=0')                                                      # Friendly
        self.run_chunk(gn.enemy_cmd('placed', 2.0, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['ok'], e['label'], e['found'], e['at'], e['d2'], e['hp'], e['hv'], e['act']),
                         (True, 'placed', True, True, 2.0, 80.0, 80.0, True))
        self.assertEqual((e['att'], e['god'], e['attset'], e['att_after']), (0, True, True, 1))   # Neutral now
        self.assertEqual(self.lua_str('ATT'), '1')
        self.assertEqual(self.lua_str('GODSET'), 'gameGodModeType.Immortal/' + gn.ENEMY_TAG)
        self.run_chunk(gn.enemy_place_cmd('Character.humanoid_tutorial_idle_ma', 1.2, 'after_far'))   # the global is defined
        old = gn.parse_enemy([l for l in self.log().split('\n') if 'label=after_far' in l][0])
        self.assertEqual((old['label'], old['found'], old['d2'], old['hv']), ('after_far', True, 2.0, 80.0))   # the old target
        self.assertEqual(self.entity('pos.y'), '1.20')                               # a new one, never a moved one
        self.run_chunk(gn.enemy_off_cmd())
        log = self.log()
        self.assertTrue(log.rstrip().endswith('[night] ENEMY_OFF ok=true'))
        self.assertEqual(gn.parse_enemy(log)['label'], 'off')
        self.assertEqual(self.lua_str(f"TAGGED['{gn.ENEMY_TAG}']"), 'nil')            # deleted
        self.assertEqual(self.lua_str('GODREM'), 'gameGodModeType.Immortal/' + gn.ENEMY_TAG)

    def test_a_target_that_is_still_being_deleted_is_never_taken_for_the_new_one(self):
        self.run_chunk(gn.enemy_place_cmd('Character.x', 2.0, 'after_start', define=True))
        self.run_chunk('ASYNC=true')                                                 # delete and create take effect a frame later
        self.run_chunk(gn.enemy_place_cmd('Character.x', 1.2, 'after_far'))
        self.assertEqual(self.entity('pos.y'), '2.00')                               # the old one is still the tagged one
        self.run_chunk(gn.enemy_cmd('placed', 2.0, 1))                               # the old one IS at the wanted distance
        e = gn.parse_enemy(self.log())
        self.assertFalse(e['found'])                                                 # ... but only the new one counts
        self.assertEqual(self.lua_str('GODSET'), 'nil')                              # and nothing was prepared on the old one
        self.run_chunk('TICK()')
        self.run_chunk(gn.enemy_cmd('placed', 1.2, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['found'], e['at'], e['d2'], e['god']), (True, True, 1.2, True))
        self.assertEqual(self.entity('pos.y'), '1.20')
        self.assertEqual(self.lua_str('GODSET'), 'gameGodModeType.Immortal/' + gn.ENEMY_TAG)

    def test_not_at_the_wanted_distance_is_reported_and_not_prepared(self):
        self.run_chunk(gn.enemy_place_cmd('Character.x', 6.0, 'after_start', define=True))
        self.run_chunk(gn.enemy_cmd('placed', 2.0, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['found'], e['d2'], e['at'], e['god'], e['att_after']), (True, 6.0, False, None, None))
        self.assertEqual(self.lua_str('GODSET'), 'nil')                              # no preparation of a target that is not there
        self.run_chunk(gn.enemy_cmd('placed', 6.3, 0))                               # within 0.4 m: placed, god mode only
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['at'], e['god'], e['attset']), (True, True, None))

    def test_hostile_target_is_left_hostile_and_the_camera_forward_is_followed(self):
        self.run_chunk('CAMF=Vector4.new(0.8660254,0.5,0,0)')                         # camera turned 60 deg (the orbit phase)
        self.run_chunk(gn.enemy_place_cmd('Character.x', 1.2, 'after_near', define=True))
        self.assertEqual((self.entity('pos.x'), self.entity('pos.y')), ('1.04', '0.60'))
        self.run_chunk(gn.enemy_cmd('placed', 1.2, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['at'], e['att'], e['attset'], e['att_after']), (True, 2, None, 2))
        self.assertEqual(self.lua_str('ATT'), '2')                                   # Hostile stays: only Friendly / unknown change

    def test_attitude_read_fallbacks(self):
        self.run_chunk(gn.enemy_place_cmd('Character.x', 2.0, 'after_start', define=True))
        self.run_chunk("ATTFAIL=true ATTTXT='AIA_Friendly'")                         # the first read fails, the agent read works
        self.run_chunk(gn.enemy_cmd('placed', 2.0, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['att'], e['attset'], e['att_after']), (0, True, 1))
        self.run_chunk("ATTFAIL=true ATTTXT=nil TAGGED['" + gn.ENEMY_TAG + "'][1].GetAttitudeAgent=function() error('x') end")
        self.run_chunk(gn.enemy_cmd('placed', 2.0, 1))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['ok'], e['att'], e['at'], e['god']), (True, None, True, True))   # unreadable: logged, not fatal
        self.assertEqual(e['attset'], False)                                         # tried (the agent is broken here), reported

    def test_missing_record_and_no_entity(self):
        self.run_chunk("MISSING['Character.gone']=true")
        self.run_chunk(gn.enemy_spawn_cmd('Character.gone'))
        self.assertIn('[night] ENEMY_SPAWN ok=false', self.log())
        self.assertIn('record missing', self.log())
        self.assertEqual(self.lua_str(f"TAGGED['{gn.ENEMY_TAG}']"), 'nil')
        self.run_chunk(gn.enemy_cmd('wait', define=True))
        self.assertFalse(gn.parse_enemy(self.log())['found'])                        # nothing spawned: found=0
        self.run_chunk(gn.enemy_off_cmd())                                           # off without a target: still echoes
        self.assertTrue(self.log().rstrip().endswith('[night] ENEMY_OFF ok=true'))

    def test_call_without_definition_fails_cleanly_and_unreadable_health(self):
        self.run_chunk(gn.enemy_cmd('far', 2.0, 1))                                  # no UTPPENEMY global yet
        e = gn.parse_enemy(self.log())
        self.assertFalse(e['ok'])
        self.assertIn('UTPPENEMY', e['raw'])
        self.run_chunk(gn.enemy_spawn_cmd('Character.x', 2.0))
        self.run_chunk('POOL.GetStatPoolValue=function() error("no pool") end')
        self.run_chunk(gn.enemy_cmd('prep', 2.0, 1, define=True))
        e = gn.parse_enemy(self.log())
        self.assertEqual((e['ok'], e['found'], e['hp'], e['hv'], e['at'], e['att']), (True, True, None, None, True, 2))


class EnemyMacroTests(unittest.TestCase):
    """enemy_macro with stubbed console / play / camera: the target is a small world (see _EnemyWorld)."""

    def setUp(self):
        self._time = night.time
        night.time = FakeTime()
        fd, self.log = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        self.n = EnsureClosedTests.make(self, FakeApi(), None)
        self.n.utpp_log = self.log
        self.n.scen_log_off = 0
        self.n.cycle = {'name': 't', 'steps': []}
        self.n.auto_pos = True
        self.n.recover = lambda *a, **k: {'suspect': False}
        self._rel = gn.PUNCH_REPORT_REL

    def tearDown(self):
        night.time = self._time
        gn.PUNCH_REPORT_REL = self._rel
        os.remove(self.log)

    def world(self, **kw):
        return _EnemyWorld(self, **kw)

    def run_macro(self, w, args=()):
        self.n.args = night.build_parser().parse_args(['--quit-timeout', '3', '--out-root', 'unused',
                                                       '--log-dir', self.n.args.log_dir, *args])
        return self.n.enemy_macro(_Pad(), _G(), {'runs': []})

    def test_pass(self):
        w = self.world()
        r = self.run_macro(w)
        self.assertEqual((r['verdict'], r['fails']), ('PASS', []), r)
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward', 'melee_punch', 'melee_punch', 'melee_punch', 'melee_block'])
        labels = [lab for lab, _ in w.cmds]
        self.assertEqual(labels, ['nightauto_enemy', 'fists_cmd',
                                  'enemy_place/far', 'enemy/placed', 'enemy_place/near', 'enemy/placed',
                                  'enemy_place/orbit', 'enemy/placed', 'enemy_place/block', 'enemy/placed',
                                  'enemy_off', 'nightauto_restore', 'fists_off_cmd'])      # fists first, no target in the world
        self.assertLess(labels.index('fists_cmd'), labels.index('enemy_place/far'))
        cmds = dict((lab, c) for lab, c in w.cmds)
        rec = gn.ENEMY_RECORDS[0]
        self.assertEqual(cmds['nightauto_enemy'], gn.nightauto_cmd(0.2, True, gn.ENEMY_TAG))
        self.assertEqual(cmds['nightauto_restore'], gn.nightauto_cmd(self.n.args.night_auto_period))   # the vehicle tag again
        self.assertEqual(cmds['enemy_place/far'], gn.enemy_place_cmd(rec, 1.8, 'after_start', define=True))
        self.assertEqual(cmds['enemy_place/near'], gn.enemy_place_cmd(rec, 1.2, 'after_far'))
        self.assertEqual(cmds['enemy_place/orbit'], gn.enemy_place_cmd(rec, 1.2, 'after_near'))
        self.assertEqual(cmds['enemy_place/block'], gn.enemy_place_cmd(rec, 1.5, 'after_orbit'))
        self.assertEqual(cmds['enemy/placed'], gn.enemy_cmd('placed', 1.5, 1))          # the last one sent
        self.assertEqual(cmds['enemy_off'], gn.enemy_off_cmd())
        self.assertEqual(sum('UTPPENEMY=function' in c for _, c in w.cmds), 1)       # defined once (the first placement)
        self.assertEqual(w.state['n_spawn'], 4)                                      # one fresh target per phase, never moved
        ph = {p['name']: p for p in r['enemy']['phases']}
        self.assertEqual({k: p['lock'] for k, p in ph.items()}, {'far': 2, 'near': 2, 'orbit': 2, 'block': 0})
        self.assertEqual(r['enemy']['hits'], {'far': False, 'near': True, 'orbit': True, 'block': False})
        self.assertEqual((ph['near']['drop'], ph['far']['drop'], ph['block']['drop']), (30.0, 0.0, 0.0))
        self.assertEqual(ph['orbit']['orbit_done_deg'], 60.0)
        self.assertEqual({k: p['placed_d2'] for k, p in ph.items()}, {'far': 1.8, 'near': 1.2, 'orbit': 1.2, 'block': 1.5})
        self.assertEqual(ph['near']['dist']['min'], 1.2)
        self.assertEqual(r['enemy']['record'], rec)
        self.assertEqual(r['enemy']['spawn']['att_name'], 'hostile')
        self.assertTrue(r['enemy']['locks'] >= 1)
        json.dumps(r, default=str)                                                   # cycle.json serialisable
        self.assertEqual(w.state['spawned'], None)                                   # the target is gone at the end
        # what the macro changed is listed in the run record and was put back by its own teardown
        self.assertEqual({e['key']: e['state'] for e in self.n.settings.record()},
                         {'night_auto.tag': 'restored', 'enemy.target': 'restored', 'equipment.fists': 'restored'})
        self.assertEqual(self.n.settings.open_entries(), [])
        text = open(self.log, encoding='utf-8').read()
        self.assertEqual(text.count('label=after_start found=0'), 1)
        self.assertIn('god=true', text)

    def test_target_that_stays_at_its_spawn_point_is_not_punched(self):
        w = self.world(spawn_at=6.0)                     # the spawn lands 6 m away whatever distance was asked
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward'])                                 # no punch into the air
        self.assertTrue(any('never placed' in f and '6.0 m from V, wanted 1.8 m' in f for f in r['fails']), r['fails'])
        self.assertNotIn('no `src=lock`', ' '.join(r['fails']))
        self.assertEqual([p['name'] for p in r['enemy']['phases']], ['far'])         # nothing to try again with no record that worked
        self.assertIn('enemy_off', [lab for lab, _ in w.cmds])                       # cleaned up all the same
        self.assertEqual(w.state['spawned'], None)

    def test_one_phase_without_its_target_is_skipped_and_the_rest_runs(self):
        w = self.world(spawn_at_nth={3: 6.0})            # the orbit target does not come
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward', 'melee_punch', 'melee_punch', 'melee_block'])   # orbit not punched
        self.assertEqual([f for f in r['fails'] if f.startswith('orbit:')],
                         ['orbit: the target is 6.0 m from V, wanted 1.2 m (+-0.4 m)'])
        self.assertEqual(r['enemy']['hits']['near'], True)
        ph = {p['name']: p for p in r['enemy']['phases']}
        self.assertFalse(ph['orbit']['placed'])
        self.assertTrue(ph['block']['placed'])

    def test_primary_record_missing_uses_the_fallback(self):
        w = self.world(missing=(gn.ENEMY_RECORDS[0],))
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(r['enemy']['tried'], list(gn.ENEMY_RECORDS[:2]))
        self.assertEqual(r['enemy']['record'], gn.ENEMY_RECORDS[1])
        places = [c for lab, c in w.cmds if lab == 'enemy_place/far']
        self.assertEqual(places, [gn.enemy_place_cmd(gn.ENEMY_RECORDS[0], 1.8, 'after_start', define=True),
                                  gn.enemy_place_cmd(gn.ENEMY_RECORDS[1], 1.8, 'after_start', define=True)])
        self.assertEqual([c['ok'] for c in r['checks'] if c['phase'] == 'enemy_place/far'], [False, True])
        later = [c for lab, c in w.cmds if lab == 'enemy_place/near']
        self.assertEqual(later, [gn.enemy_place_cmd(gn.ENEMY_RECORDS[1], 1.2, 'after_far')])   # the record that worked

    def test_state_echo_lost_then_found_and_global_lost(self):
        w = self.world(lose=['enemy/placed'], wait_polls=2)
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        labels = [lab for lab, _ in w.cmds]
        self.assertEqual(labels.count('enemy/placed'), 3 + 2 + 2 + 2)                # far: lost, resent, not yet, found
        first = [c for lab, c in w.cmds if lab == 'enemy/placed'][:2]
        self.assertNotIn('UTPPENEMY=function', first[0])
        self.assertIn('UTPPENEMY=function', first[1])                                # the lost echo: sent again WITH the definition
        w = self.world(drop_global_at='placed')
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        placed = [c for lab, c in w.cmds if lab == 'enemy/placed']
        self.assertEqual(len(placed), 5)                                             # the global was gone: sent again, once
        self.assertNotIn('UTPPENEMY=function', placed[0])
        self.assertIn('UTPPENEMY=function', placed[1])

    def test_lost_placement_echo_is_sent_once_more(self):
        w = self.world(lose=['enemy_place/near'])
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual([lab for lab, _ in w.cmds].count('enemy_place/near'), 2)

    def test_no_entity_within_the_wait_tries_the_next_record_and_fails(self):
        w = self.world(wait_polls=10 ** 6)
        r = self.run_macro(w, ['--enemy-spawn-wait', '1.0'])
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(r['enemy']['tried'], list(gn.ENEMY_RECORDS))
        self.assertTrue(any('never placed' in f and 'no target entity found' in f for f in r['fails']), r['fails'])
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward'])                                 # no punch without a target
        self.assertIn('enemy_off', [lab for lab, _ in w.cmds])                       # cleaned up all the same

    def test_friendly_target_fails_when_neutral_does_not_take(self):
        w = self.world(att=0, att_after_neutral=0)
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertTrue(any('Friendly' in f for f in r['fails']), r['fails'])
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward'])                                 # phases skipped
        self.assertEqual(len(r['fails']), 1)                                         # no lock / health noise
        self.assertIn('enemy_off', [lab for lab, _ in w.cmds])

    def test_friendly_target_made_neutral_runs_the_phases(self):
        w = self.world(att=0, att_after_neutral=1)
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(r['enemy']['spawn']['att'], 1)
        self.assertEqual(len(w.played), 6)

    def test_unreadable_attitude_gets_neutral_and_is_a_warning_only_when_still_unknown(self):
        w = self.world(att=None, att_after_neutral=1)
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertEqual(r['enemy']['spawn']['att'], 1)
        self.assertEqual(r['enemy']['spawn']['att_read'], None)

    def test_cleanup_when_a_phase_raises(self):
        w = self.world(raise_in=('melee_punch', 2))
        with self.assertRaises(night.CycleError):
            self.run_macro(w)
        labels = [lab for lab, _ in w.cmds]
        self.assertEqual(labels[-3:], ['enemy_off', 'nightauto_restore', 'fists_off_cmd'])    # the finally ran everything
        self.assertEqual(labels.count('enemy_off'), 1)
        self.assertEqual(w.state['spawned'], None)

    def test_failures_no_lock_no_drop_unreadable_health(self):
        w = self.world(lock=())
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertIn('no `src=lock`', ' '.join(r['fails']))
        self.assertEqual(r['enemy']['locks'], 0)
        w = self.world(drops={'far': 0.0, 'near': 0.0, 'orbit': 0.0, 'block': 0.0})
        r = self.run_macro(w)
        self.assertTrue(any('near: health did not drop' in f for f in r['fails']), r['fails'])
        self.assertTrue(any('orbit: health did not drop' in f for f in r['fails']), r['fails'])
        w = self.world(hp_unreadable=True)
        r = self.run_macro(w)
        self.assertTrue(any('health unreadable' in f for f in r['fails']), r['fails'])

    def test_orbit_without_a_hit_fails_but_far_does_not(self):
        w = self.world(drops={'far': 0.0, 'near': 30.0, 'orbit': 0.0, 'block': 0.0})
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(len(r['fails']), 1)
        self.assertIn('orbit: health did not drop', r['fails'][0])
        self.assertEqual(r['enemy']['hits'], {'far': False, 'near': True, 'orbit': False, 'block': False})
        w = self.world(drops={'far': 30.0, 'near': 30.0, 'orbit': 30.0, 'block': 0.0})
        r = self.run_macro(w)
        self.assertEqual((r['verdict'], r['fails']), ('PASS', []))
        self.assertEqual(r['enemy']['hits']['far'], True)                            # reported, not required

    def test_cleanup_after_a_scenario_timeout_disarms_the_deadline(self):
        self.n.scenario_deadline = 1e12
        w = self.world(raise_in=('melee_punch', 2, night.ScenarioTimeout))
        with self.assertRaises(night.ScenarioTimeout):
            self.run_macro(w)
        self.assertIsNone(self.n.scenario_deadline)                                  # the cleanup could not re-raise it
        labels = [lab for lab, _ in w.cmds]
        self.assertEqual(labels[-3:], ['enemy_off', 'nightauto_restore', 'fists_off_cmd'])
        self.assertEqual(w.state['spawned'], None)

    def test_health_before_is_read_at_placement_and_after_before_the_delete(self):
        w = self.world(drops={'far': 0.0, 'near': 12.5, 'orbit': 30.0, 'block': 0.0})
        r = self.run_macro(w)
        self.assertEqual((r['verdict'], r['fails']), ('PASS', []))
        self.assertEqual({p['name']: p['drop'] for p in r['enemy']['phases']},
                         {'far': 0.0, 'near': 12.5, 'orbit': 30.0, 'block': 0.0})

    def test_god_failure_is_a_warning(self):
        w = self.world(god_ok=False)
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS')                                       # the drop is measured from the state reads
        self.assertEqual(sum('Immortal' in x for x in r['warnings']), 1)             # said once, not per phase

    def test_no_cet_fists_skips_the_fists_commands(self):
        w = self.world()
        r = self.run_macro(w, ['--no-cet-fists'])
        self.assertEqual(r['verdict'], 'PASS', r)
        self.assertNotIn('fists_cmd', [lab for lab, _ in w.cmds])
        self.assertNotIn('fists_off_cmd', [lab for lab, _ in w.cmds])

    def test_without_night_auto_the_tag_is_not_switched(self):
        self.n.auto_pos = False
        w = self.world()
        r = self.run_macro(w)
        self.assertNotIn('nightauto_enemy', [lab for lab, _ in w.cmds])
        self.assertNotIn('nightauto_restore', [lab for lab, _ in w.cmds])

    def test_fists_wait_ends_at_the_first_pos_line_with_the_weapon(self):
        w = self.world()
        r = self.run_macro(w)
        self.assertFalse(any('no POS line with the weapon' in x for x in r['warnings']), r['warnings'])
        w = self.world(fists_pos=False)                  # the weapon never shows in a POS line: a warning, no endless wait
        t0 = night.time.monotonic()
        r = self.run_macro(w)
        self.assertEqual(r['verdict'], 'PASS')
        self.assertTrue(any('no POS line with the weapon' in x for x in r['warnings']), r['warnings'])
        self.assertLess(night.time.monotonic() - t0, 120.0)

    def test_punch_report_hook(self):
        w = self.world()
        gn.PUNCH_REPORT_REL = os.path.join(tempfile.gettempdir(), 'no_such_punch_direction_report.py')
        r = self.run_macro(w, ['--punch-report'])
        self.assertEqual(r['enemy']['report'], {'ran': False, 'skipped': 'script missing'})
        self.assertEqual(r['verdict'], 'PASS')
        fd, script = tempfile.mkstemp(suffix='.py')
        os.close(fd)
        try:
            gn.PUNCH_REPORT_REL = script
            calls = []

            class _P:
                returncode, stdout, stderr = 0, 'ok', ''
            self.n.runner = lambda cmd, **k: calls.append(cmd) or _P()
            w = self.world()
            r = self.run_macro(w, ['--punch-report'])
            self.assertEqual(calls, [gn.punch_report_cmd(sys.executable, script, self.log)])
            self.assertEqual((r['enemy']['report']['ran'], r['enemy']['report']['rc']), (True, 0))
            w = self.world()
            r = self.run_macro(w)                                                    # option off: nothing is run
            self.assertEqual(len(calls), 1)
            self.assertIsNone(r['enemy']['report'])
        finally:
            os.remove(script)


class EnemyBackgroundTests(unittest.TestCase):
    """--background with the file command channel: the whole enemy macro goes through commands and pad scenarios only."""

    def setUp(self):
        self._pp, self._time = night.process_path, night.time
        night.process_path = lambda pid: ''
        night.time = FakeTime()

    def tearDown(self):
        night.process_path, night.time = self._pp, self._time

    def test_macro_sends_no_key_mouse_or_focus_call(self):
        n, g, api, mod = CommandChannelNightTests.make(self, '--cmd-channel', 'file', '--background')
        self.assertIsInstance(g, night.BackgroundGuard)
        n.scen_log_off = 0
        n.cycle = {'name': 't', 'steps': []}
        n.auto_pos = True
        n.recover = lambda *a, **k: {'suspect': False}
        holder = type('H', (), {'n': n, 'log': n.utpp_log})()
        w = _EnemyWorld(holder, stub_console=False)
        mod.answer = lambda seq, chunk: (w.handle(chunk), True)[1]                   # the CET module runs the chunk
        n.overlay_grab = lambda guard: self.fail('the overlay is never looked at with the file channel')
        r = n.enemy_macro(_Pad(), g, {'runs': []})
        self.assertEqual((r['verdict'], r['fails']), ('PASS', []), r)
        self.assertEqual(w.played, ['melee_draw', 'melee_face_forward', 'melee_punch', 'melee_punch', 'melee_punch', 'melee_block'])
        self.assertEqual(w.state['n_spawn'], 4)
        self.assertEqual([c for c in api.calls if c[0] in ('keys', 'char', 'click', 'cursor', 'focus', 'raw')], [])
        self.assertTrue(any('UTPPENEMY=function' in chunk for _, chunk in mod.ran))   # the commands really went through the file


class _EnemyWorld:
    """Stub game for enemy_macro: handle() answers the harness commands from a small world (target position, health,
    attitude, spawn state) the way the real commands do - a target only ever sits where it was spawned - play() writes the
    log lines of a played pad part (mask lines, MELEEAIM, POS with the target in v=) and applies the punch damage of the
    current phase (the n-th spawn is the n-th phase)."""

    def __init__(self, test, missing=(), wait_polls=1, att=2, att_after_neutral=1, god_ok=True, drops=None,
                 lock=('far', 'near', 'orbit'), lose=(), drop_global_at=None, raise_in=None, hp_unreadable=False,
                 spawn_at=None, spawn_at_nth=None, stub_console=True, fists_pos=True):
        self.t, self.n, self.log = test, test.n, test.log
        open(self.log, 'w').close()                                                  # a fresh log per world
        self.missing, self.wait_polls, self.att, self.att_neutral = set(missing), wait_polls, att, att_after_neutral
        self.god_ok, self.lock, self.lose = god_ok, set(lock), list(lose)
        self.drops = drops if drops is not None else {'far': 0.0, 'near': 30.0, 'orbit': 30.0, 'block': 0.0}
        self.drop_global_at, self.raise_in, self.hp_unreadable = drop_global_at, raise_in, hp_unreadable
        self.spawn_at, self.spawn_at_nth = spawn_at, dict(spawn_at_nth or {})        # where a spawn really lands (wrong)
        self.k, self.fists_pos = 0.0, fists_pos
        self.cmds, self.played = [], []
        self.state = {'spawned': None, 'polls': 0, 'hv': 100.0, 'dist': 6.0, 'neutral': False, 'defined': False,
                      'n_spawn': 0}
        test.n.play = self.play
        if stub_console:
            test.n.console_verified = self.console_verified
        test.n.poll_pos = lambda g, pad=None, timeout=None: {'p': (0.0, 0.0, 0.0), 'cf': (0.0, 1.0), 'v': None, 'vr': None}
        test.n.camera_align = lambda pad, g, plan, known, target, ep, tol=None, why='': dict(
            known, cf=gn.rotate_stick(*known['cf'], 60.0))

    def emit(self, body, cet=False):
        self.k += 0.02
        ts = f'[2026-10-07 23:00:{self.k:06.3f}] [info    ] [ 1] [UTPP] UTPP '
        with open(self.log, 'a', encoding='utf-8') as f:
            f.write(ts + ('CET ' if cet else '') + body + chr(10))

    def text(self):
        with open(self.log, encoding='utf-8') as f:
            return f.read()

    def enemy_body(self, label, call):
        s = self.state
        old = label.startswith('after_') or label == 'off'
        found = s['spawned'] is not None and (old or s['polls'] >= self.wait_polls)
        if not found:
            return f'[night] ENEMY ok=true label={label} found=0'
        att = self.att_neutral if s['neutral'] else self.att
        hp = '-' if self.hp_unreadable else f'{s["hv"]:.1f}'
        body = (f'[night] ENEMY ok=true label={label} found=1 pre_d2={s["dist"]:.2f} pre_hv={hp} pre_hp={hp} '
                f'att={"nil" if att is None else att} act=true')
        want, prep = call
        if want != 'nil':
            at = abs(s['dist'] - float(want)) <= gn.ENEMY_PLACE_TOL
            body += f' at={1 if at else 0}'
            if at and prep != 'nil':
                body += f' god={"true" if self.god_ok else "false"}'
                if prep == '1' and att in (0, None):
                    body += ' attset=true'
                    s['neutral'] = True
                    att = self.att_neutral
                body += f' att2={"nil" if att is None else att}'
        return body

    def handle(self, cmd):
        """Run one harness command: write its echo lines to the log (what the CET command would print)."""
        s = self.state
        if cmd == gn.FISTS_CMD:
            if self.fists_pos:
                self.emit('[night] POS p=0.00,0.00,0.00 cf=0.000,1.000 v=nil src=none w=1 m=0 n=1', cet=True)
            self.emit('[night] FISTS ok=true quickslots', cet=True)
        elif cmd == gn.FISTS_OFF_CMD:
            self.emit('[night] FISTS_OFF ok=true eqsystem', cet=True)
            self.emit('MMMASK off reasons=0x0 (was 0x1) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=0 exit=0.00 w=1.00 mask=0.00')
        elif cmd.startswith('GetMod("utpp_core").NightAuto('):
            self.emit('[night] auto on period=0.20 tag=x', cet=True)
        elif '[night] ENEMY_SPAWN ok=' in cmd:                                       # a placement: read the old one, spawn
            if 'UTPPENEMY=function' in cmd:
                s['defined'] = True
            lab = cmd.split('label=', 1)[1].split(' ', 1)[0]
            self.emit_state(lab, ('nil', 'nil'))
            rec = cmd.split("TweakDBID.new('", 1)[1].split("'", 1)[0]
            if rec in self.missing:
                self.emit('[night] ENEMY_SPAWN ok=false [string]:1: record missing', cet=True)
            else:
                s['n_spawn'] += 1
                dist = float(cmd.split('a.x+x*', 1)[1].split(',', 1)[0])
                if s['n_spawn'] in self.spawn_at_nth:
                    dist = self.spawn_at_nth[s['n_spawn']]
                elif self.spawn_at is not None:
                    dist = self.spawn_at
                s.update(spawned=rec, polls=0, hv=100.0, neutral=False, dist=dist)
                self.emit(f'[night] ENEMY_SPAWN ok=true {rec}', cet=True)
        elif '[night] ENEMY_OFF ok=' in cmd:
            self.emit(self.enemy_body('off', ('nil', 'nil')), cet=True)
            s['spawned'] = None
            self.emit('[night] ENEMY_OFF ok=true', cet=True)
        elif 'UTPPENEMY(' in cmd:
            call = tuple(cmd.split('return UTPPENEMY(', 1)[1].split(')', 1)[0].split(','))
            lab = cmd.split('label=', 1)[1].split(' ', 1)[0]
            if 'UTPPENEMY=function' in cmd:
                s['defined'] = True
            if self.drop_global_at == lab and s['defined'] and 'UTPPENEMY=function' not in cmd:
                self.drop_global_at = None
                s['defined'] = False
            self.emit_state(lab, call)

    def emit_state(self, lab, call):
        s = self.state
        if not s['defined']:
            self.emit(f"[night] ENEMY ok=false label={lab} [string]:1: attempt to call global 'UTPPENEMY' (a nil value)", cet=True)
            return
        if lab == 'placed':
            s['polls'] += 1
        self.emit(self.enemy_body(lab, call), cet=True)

    def console_verified(self, pad, g, cmd, echo, label, timeout=3.0):
        self.cmds.append((label, cmd))
        night.time.sleep(1.0)                                                        # a typed command takes a moment
        if label in self.lose:
            self.lose.remove(label)
            return None
        self.handle(cmd)
        return self.text()

    def play(self, pad, g, name, plan, sc=None):
        self.played.append(name)
        s = self.state
        if self.raise_in and self.raise_in[0] == name and self.played.count(name) == self.raise_in[1]:
            raise (self.raise_in[2] if len(self.raise_in) > 2 else night.CycleError)('boom')
        night.time.sleep(2.0)
        if name == 'melee_draw':
            self.emit('MMMASK on reasons=0x1 (was 0x0) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=1 exit=0.00 w=1.00 mask=0.00')
        elif name == 'melee_holster':
            self.emit('MMMASK off reasons=0x0 (was 0x1) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=0 exit=0.00 w=1.00 mask=0.00')
        elif name in ('melee_punch', 'melee_block'):
            cur = gn.ENEMY_PHASES[s['n_spawn'] - 1][0]
            for i in range(4):
                src = 'lock' if (cur in self.lock and i >= 2) else 'cam'
                self.emit(f'MELEEAIM src={src} tiltW=1.00 tgtPitch=-3.0 tgtYaw={5.0 * i:.1f} rawPitch=-3.0 rawYaw=0.0 pitch=-3.0 yaw=0.0 '
                          'flags=0x0 maxUp=20 maxDown=35 hz=10 calls=1 applied=1 busy=0 reapplied=0 fails=0')
                self.emit(f'[night] POS p=0.00,0.00,0.00 cf=0.000,1.000 v=0.00,{s["dist"]:.2f},0.00,1.000,0.000 src=tag w=1 m=0 n={i}',
                          cet=True)
            self.emit('MMMELEEFULL on why=on full=0.00 speed=0.00 loco=1 rawLoco=1 detail=-1 vz=0.00 reasons=0x1 gate=0x400 stick=0 '
                      'ground=1 fastT=0.00 fastOff=0 airT=0.00 state=6 states=0x37FDE0 guard=0')
            if name == 'melee_punch':
                s['hv'] -= self.drops.get(cur, 0.0)
        return {'steps': []}


class CommandChannelNightTests(unittest.TestCase):
    """--cmd-channel file / auto and --background in the night runner (no keys, no overlay)."""

    def setUp(self):
        self._pp, self._time = night.process_path, night.time
        night.process_path = lambda pid: ''
        night.time = FakeTime()

    def tearDown(self):
        night.process_path, night.time = self._pp, self._time

    def make(self, *flags, ready=True):
        from test_gametest import FakeTestcmdModule
        api = FakeApi()
        n = EnsureClosedTests.make(EnsureClosedTests(), api, None)
        n.args = night.build_parser().parse_args(['--log-dir', n.args.log_dir, '--out-root', 'unused'] + list(flags))
        n.utpp_log = night.gt_log.newest_log(n.args.log_dir)
        if ready:
            with open(n.utpp_log, 'a', encoding='utf-8') as f:
                f.write('[2026-11-14 10:00:00.000] [info] [1] [UTPP] UTPP CET [testcmd] READY seq=0 loader=loadstring\n')
        d = os.path.join(tempfile.mkdtemp(), 'mods', 'utpp_testcmd')
        os.makedirs(d)
        n.testcmd_path = os.path.join(d, 'cmd.txt')
        mod = FakeTestcmdModule(n.testcmd_path, n.utpp_log)
        api.on_sleep.append(mod.tick)
        g = n.guard()
        g.refresh_game()
        return n, g, api, mod

    def test_file_channel_sends_without_keys_or_overlay(self):
        n, g, api, mod = self.make('--cmd-channel', 'file')
        n.overlay_grab = lambda guard: self.fail('the overlay is never looked at with the file channel')
        res = n.type_console(g, night.gn.arm_cmds('utpp_base'), verify_log=n.utpp_log)
        self.assertEqual(list(res.values()), [True, True, True])
        self.assertEqual([c for _, c in mod.ran], night.gn.arm_cmds('utpp_base'))
        txt = n.console_verified(None, g, 'Game.UTPPLogLine("[night] X")', lambda t: 'ACK' in t, 'x')
        self.assertIsNotNone(txt)
        self.assertEqual(len(mod.ran), 4)                       # one send, no settle retries
        self.assertEqual([c for c in api.calls if c[0] in ('keys', 'char', 'click', 'cursor', 'focus')], [])

    def test_auto_resolves_from_ready_and_background_needs_it(self):
        n, g, api, mod = self.make('--cmd-channel', 'auto')
        self.assertTrue(n.use_file())
        n, g, api, mod = self.make('--cmd-channel', 'auto', ready=False)
        self.assertFalse(n.use_file())
        n, g, api, mod = self.make('--cmd-channel', 'auto', '--background', ready=False)
        with self.assertRaises(night.CycleError):
            n.use_file()
        self.assertFalse(night.build_parser().parse_args([]).background)
        self.assertEqual(night.build_parser().parse_args([]).cmd_channel, 'console')
        self.assertTrue(night.validate_args(night.build_parser().parse_args(['--background'])))
        self.assertEqual(night.validate_args(night.build_parser().parse_args(['--background', '--cmd-channel', 'file'])), [])

    def test_background_guard_and_keyboard_scenarios(self):
        n, g, api, mod = self.make('--cmd-channel', 'file', '--background')
        self.assertIsInstance(g, night.BackgroundGuard)
        api.fg = OTHER_HWND                                       # the game is in the background
        res = n.type_console(g, ['Game.UTPPSetMMEnabled(true)'], verify_log=n.utpp_log)
        self.assertEqual(res, {'Game.UTPPSetMMEnabled(true)': True})
        with self.assertRaises(night.NeedsKeyboard):
            n.hold_key(g, 0x2E, 0.05)
        self.assertTrue(n.scenario_needs_keyboard('shoulder_key'))
        crouch = next(k for k, sc in n.scenarios.items() if gn.crouch_toggle_steps(sc))
        self.assertTrue(n.scenario_needs_keyboard(crouch))
        self.assertFalse(n.scenario_needs_keyboard('walk_forward_stop') if 'walk_forward_stop' in n.scenarios else False)
        # vehicle exit without keys: straight to the scripted unmount
        n.cycle = {'steps': []}
        n.args.exit_pre, n.args.exit_done_wait = 0.1, 0.1
        n.console_verified = lambda pad, g_, cmd, echo, label, timeout=3.0: '[night] UNMOUNT ok=true'
        n.poll_pos = lambda g_, pad=None, timeout=None: {'mounted': False}
        ex = n.exit_vehicle(None, g, None, 'test')
        self.assertEqual([t['how'] for t in ex['tries']], ['cet_unmount'])
        self.assertTrue(ex['scripted'])

    def test_background_input_switch(self):
        """--background: UTPPTestHost switched on after arming (status checked), off at the end / stop, plain sleeps."""
        from gt_input import parse_testcmd_lines
        answers = {}

        class Module:
            def __init__(self, cmd_path, log_path):
                self.cmd_path, self.log_path, self.last, self.ran = cmd_path, log_path, 0, []

            def tick(self):
                if not os.path.isfile(self.cmd_path):
                    return
                with open(self.cmd_path, encoding='utf-8') as f:
                    entries = parse_testcmd_lines(f.read())
                for seq, chunk in entries:
                    if seq > self.last:
                        self.last = seq
                        self.ran.append(chunk)
                        ok, text = answers[chunk]
                        with open(self.log_path, 'a', encoding='utf-8') as f:
                            f.write(f'[2026-11-14 10:00:00.000] [info] [1] [UTPP] UTPP CET [testcmd] ACK seq={seq} '
                                    f'ok={"true" if ok else "false"} {text}\n')

        class TickTime(FakeTime):
            def __init__(self, mod):
                super().__init__()
                self.mod = mod

            def sleep(self, s):
                super().sleep(s)
                self.mod.tick()

        n, g, api, _ = self.make('--cmd-channel', 'file', '--background')
        mod = Module(n.testcmd_path, n.utpp_log)
        night.time = TickTime(mod)
        api.on_sleep.clear()                                     # only the plain sleep may drive the module
        n.cycle = {'steps': []}
        off = {'ok': None, 'result': ''}
        self.assertEqual(n.bg_input(False, 'nothing on'), off)   # never switched on: nothing is sent
        self.assertEqual(mod.ran, [])
        answers[night.BG_INPUT_ON] = (True, 'UTPPTestHost on gate=open hooks=8/8 fg=0')
        answers[night.BG_INPUT_OFF] = (True, 'UTPPTestHost off gate=original')
        res = n.bg_input(True, 'after arming')
        self.assertTrue(res['ok'])
        self.assertTrue(n.bg_input_active)
        api.keys_down = {0x13}                                   # Pause held: the switch-off still goes through
        self.assertTrue(n.bg_input(False, 'stop')['ok'])
        self.assertFalse(n.bg_input_active)
        self.assertEqual(mod.ran, [night.BG_INPUT_ON, night.BG_INPUT_OFF])
        for ok, text, want in ((False, "[string \"testcmd\"]:1: attempt to call a nil value (field 'UTPPTestBackgroundInput')",
                                'nil value'),
                               (True, 'UTPPTestHost refused: gate bytes at RVA 0x20849AA differ', 'refused')):
            answers[night.BG_INPUT_ON] = (ok, text)
            with self.assertRaises(night.CycleError) as cm:
                n.bg_input(True, 'after arming')
            self.assertIn(want, str(cm.exception))
            self.assertIn('UTPPTestHost', str(cm.exception))
            self.assertFalse(n.bg_input_active)
        api.running = False                                      # no game: the switch-on fails without writing
        before = len(mod.ran)
        with self.assertRaises(night.CycleError):
            n.bg_input(True)
        self.assertEqual(len(mod.ran), before)


if __name__ == '__main__':
    unittest.main()
