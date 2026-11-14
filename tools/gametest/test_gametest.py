"""Offline tests for the UTPP game-test harness (no input is sent; ViGEmBus is never touched).

  python Source\\tools\\gametest\\test_gametest.py -v
"""
from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_input  # noqa: E402
import gt_log  # noqa: E402
import gt_scenario as gs  # noqa: E402
import run as gt_run  # noqa: E402
import video_sheets  # noqa: E402
from gt_win32 import ForegroundGuard, GuardError  # noqa: E402

PROJECT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
LOG_038 = os.path.join(PROJECT, 'Research', 'logs', 'utpp-0.3.8-ingame-user.log')
LOG_041 = os.path.join(PROJECT, 'Research', 'logs', 'utpp-0.4.1-ingame-user.log')

GAME_PID, OTHER_PID = 100, 200
GAME_HWND, OTHER_HWND = 11, 22


class FakeApi:
    """Stands in for gt_win32.Win32Api; records every state-changing call."""

    def __init__(self):
        self.fg = GAME_HWND
        self.alive = True
        self.running = True
        self.keys_down = set()
        self.calls = []
        self.focus_makes_foreground = True
        self.client = (0, 0, 1920, 1080)
        self.on_scan = []
        self.on_sleep = []

    def foreground_window(self):
        return self.fg

    def window_pid(self, hwnd):
        return {GAME_HWND: GAME_PID, OTHER_HWND: OTHER_PID}.get(hwnd, 0)

    def process_name(self, pid):
        return {GAME_PID: 'Cyberpunk2077.exe', OTHER_PID: 'notepad.exe'}.get(pid, '')

    def process_alive(self, pid):
        return self.alive and pid == GAME_PID

    def list_pids(self, exe):
        return [GAME_PID] if self.running else []

    def find_main_window(self, pids):
        return GAME_HWND if GAME_PID in pids else 0

    def key_down_now(self, vk):
        return vk in self.keys_down

    def focus(self, hwnd):
        self.calls.append(('focus', hwnd))
        if self.focus_makes_foreground:
            self.fg = hwnd

    def sleep(self, s):
        for hook in self.on_sleep:
            hook()

    def client_rect_screen(self, hwnd):
        return self.client

    def root_window_at(self, x, y):
        return GAME_HWND

    def send_scan_chord(self, scans):
        self.calls.append(('keys', tuple(scans), self.fg))
        for hook in self.on_scan:
            hook(tuple(scans))

    def vk_to_scan(self, vk):
        return {0xC0: 0x29, 0x0D: 0x1C}.get(vk, 0)

    def window_title(self, hwnd):
        return {GAME_HWND: 'Cyberpunk 2077 (C) 2020 by CD Projekt RED', OTHER_HWND: 'Untitled - Notepad'}.get(hwnd, '')

    def send_unicode(self, ch):
        self.calls.append(('char', ch, self.fg))

    def set_cursor(self, x, y):
        self.calls.append(('cursor', x, y))

    def send_left_click(self):
        self.calls.append(('click', self.fg))


class FakePad:
    def __init__(self):
        self.reports = []
        self.report = None
        self.resets = 0

    def reset(self):
        self.report = {'buttons': 0, 'ls': (0, 0), 'rs': (0, 0), 'lt': 0, 'rt': 0}
        self.resets += 1

    def left_joystick_float(self, x, y):
        self.report['ls'] = (x, y)

    def right_joystick_float(self, x, y):
        self.report['rs'] = (x, y)

    def left_trigger_float(self, v):
        self.report['lt'] = v

    def right_trigger_float(self, v):
        self.report['rt'] = v

    def press_button(self, button):
        self.report['buttons'] |= button

    def update(self):
        self.reports.append(dict(self.report))


def make_guard(api=None, interval=0.0):
    t = [0.0]

    def clock():
        t[0] += 0.01
        return t[0]
    g = ForegroundGuard(api=api or FakeApi(), process_check_interval=interval, clock=clock)
    g.refresh_game()
    return g


# ---------------------------------------------------------------------------------------------- scenarios

class ScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.all = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))

    def test_required_scenarios_present_and_valid(self):
        for n in ('slow_walk_half_stick', 'run_arc_left', 'run_arc_right', 'slalom_stick', 'flick_180_walk',
                  'flick_180_run', 'flick_180_sprint', 'camera_whip_180', 'crouch_walk_turns', 'sprint_start_stop',
                  'idle_then_start_sideways'):
            self.assertIn(n, self.all)
            sc = self.all[n]
            frames = list(sc.sample())
            self.assertGreater(len(frames), sc.duration * 50)
            for _, fr in frames:
                self.assertLessEqual(math.hypot(*fr.ls), 1.0 + 1e-9)
                self.assertLessEqual(math.hypot(*fr.rs), 1.0 + 1e-9)
            last = frames[-1][1]
            self.assertAlmostEqual(last.t, sc.duration, places=6)

    def test_crouch_input_timeline(self):
        """Offline: when R3 (crouch toggle under InputSchemesMode=Alternative, verified in game 2026-10-23 run
        20261023-190115: R3 tap 0.08 s crouches, R3 hold 0.35 s and B hold 0.35 s (= Dodge) do not) is pressed in
        crouch_walk_turns relative to stick movement."""
        sc = self.all['crouch_walk_turns']
        frames = [fr for _, fr in sc.sample()]
        r3 = gs.BUTTONS['R3']
        presses, cur = [], None            # [(start t, end t)] of contiguous R3 presses
        for fr in frames:
            down = bool(fr.buttons & r3)
            if down and cur is None:
                cur = [fr.t, fr.t]
            elif down:
                cur[1] = fr.t
            elif cur is not None:
                presses.append(tuple(cur))
                cur = None
        self.assertIsNone(cur)
        self.assertEqual(len(presses), 2, 'exactly two toggles: crouch and uncrouch')
        bounds = {n: (a, b) for n, a, b in sc.step_bounds()}
        (c0, c1), (u0, u1) = presses
        for st in sc.steps:                                                       # a TAP: the 0.35 s hold did not toggle
            for b in st.buttons:
                self.assertEqual((b.button, b.hold), ('R3', 0.08), st.name)
        self.assertLess(c1 - c0, 0.12)
        self.assertLess(u1 - u0, 0.12)
        self.assertGreaterEqual(c0, bounds['settle'][1])                         # pad already awake and settled
        self.assertLess(c1, bounds['walk'][0] - 1.0)                            # >=1 s for the stance change to finish
        self.assertGreaterEqual(u0, bounds['stop'][1])                           # uncrouch only after the stop
        self.assertLess(u1, bounds['stand_check'][0])
        for fr in frames:                                                         # pressing never overlaps movement
            self.assertFalse(fr.buttons & gs.BUTTONS['B'], 'B is Dodge under the Alternative scheme')
            if fr.buttons & r3:
                self.assertEqual(fr.ls, (0.0, 0.0))
                self.assertEqual(fr.buttons, r3)                                 # no other button (no slide/sprint)
        walk_t = [fr.t for fr in frames if math.hypot(*fr.ls) > 0.1]
        self.assertGreaterEqual(min(walk_t), c1)
        self.assertLessEqual(max(walk_t), u0)
        # the verification thresholds cover exactly the movement steps and the final stand check
        thr = sc.thresholds
        for k in ('walk', 'arc_right', 'arc_left'):
            self.assertEqual(thr[k + '.crouch_fraction']['min'], 0.8)
        self.assertIn('stand_check.stand_fraction', thr)
        self.assertTrue(sc.steps[-1].metrics)

    def test_other_scenarios_have_no_crouch_button(self):
        # crouch = R3 only in crouch_walk_turns; B on purpose: sprint_dash = dodge taps (< 0.12 s), veh_exit = mounted exit;
        # slide_sprint presses R3 while sprinting (a slide) and once to stand up (its steps: test_gt_states)
        intentional = {'sprint_dash', 'veh_exit'}
        for n, sc in self.all.items():
            if n in ('crouch_walk_turns', 'shoulder_swap_input', 'slide_sprint'):
                continue
            for st in sc.steps:
                self.assertFalse([b for b in st.buttons if b.button in ('R3', 'RS')], f'{n}.{st.name}')
                bs = [b for b in st.buttons if b.button == 'B']
                if n == 'sprint_dash':
                    self.assertTrue(all(b.hold < 0.12 for b in bs), f'{n}.{st.name}')
                elif n not in intentional:
                    self.assertFalse(bs, f'{n}.{st.name}')

    def test_acceptance_crouch_is_r3_tap(self):
        acc = gs.load_scenarios(os.path.join(HERE, 'scenarios_locomotion_acceptance.json'))['locomotion_acceptance']
        btn = {st.name: [(b.t, b.button, b.hold) for b in st.buttons] for st in acc.steps if st.buttons}
        self.assertEqual(btn['run_to_crouch'], [(1.0, 'R3', 0.08)])
        self.assertEqual(btn['crouch_to_run'], [(1.0, 'R3', 0.08)])
        self.assertFalse([s for s, bs in btn.items() if any(b[1] == 'B' for b in bs)])

    def test_half_stick_range(self):
        sc = self.all['slow_walk_half_stick']
        mags = [math.hypot(*fr.ls) for i, fr in sc.sample() if sc.steps[i].name == 'half_stick']
        self.assertGreaterEqual(min(mags), 0.40 - 1e-6)
        self.assertLessEqual(max(mags), 0.55 + 1e-6)

    def test_arc_rotation_rates(self):
        for name, sign in (('run_arc_left', -1), ('run_arc_right', 1)):
            sc = self.all[name]
            for step, rate in (('arc_60', 60), ('arc_90', 90), ('arc_120', 120)):
                st = next(s for s in sc.steps if s.name == step)
                tr = st.tracks['ls_polar']
                a0, _ = tr.value(1.0)
                a1, m1 = tr.value(1.5)
                self.assertAlmostEqual((a1 - a0) / 0.5, sign * rate, places=6)
                self.assertAlmostEqual(m1, 1.0)

    def test_polar_direction_convention(self):
        self.assertEqual(tuple(round(v, 6) for v in gs.polar_to_xy(0, 1)), (0.0, 1.0))
        self.assertEqual(tuple(round(v, 6) for v in gs.polar_to_xy(90, 1)), (1.0, 0.0))
        self.assertEqual(tuple(round(v, 6) for v in gs.polar_to_xy(-90, 0.5)), (-0.5, 0.0))

    def test_loop_track_repeats(self):
        tr = gs.Track([(0, 0, 1), (0.5, 45, 1), (1.5, -45, 1), (2.0, 0, 1)], loop=True, interp='linear', polar=True)
        self.assertAlmostEqual(tr.value(0.5)[0], 45)
        self.assertAlmostEqual(tr.value(4.5)[0], 45)
        self.assertAlmostEqual(tr.value(5.5)[0], -45)

    def test_smooth_and_linear_interp(self):
        lin = gs.Track([(0, 0, 0), (1, 0, 1)], interp='linear')
        smo = gs.Track([(0, 0, 0), (1, 0, 1)], interp='smooth')
        self.assertAlmostEqual(lin.value(0.25)[1], 0.25)
        self.assertAlmostEqual(smo.value(0.25)[1], 0.15625)
        self.assertAlmostEqual(smo.value(0.5)[1], 0.5)
        self.assertEqual(lin.value(-1)[1], 0)
        self.assertEqual(lin.value(9)[1], 1)

    def test_carry_over_and_buttons(self):
        sc = gs.parse_scenario('t', {'steps': [
            {'name': 'a', 'duration': 1.0, 'ls': [[0, 0, 0], [0.5, 0.3, 0.6]]},
            {'name': 'b', 'duration': 1.0, 'buttons': [{'t': 0.2, 'button': 'b', 'hold': 0.1}]},
        ]})
        frames = [(i, fr) for i, fr in sc.sample(0.05)]
        b_frames = [fr for i, fr in frames if i == 1]
        self.assertTrue(all(abs(fr.ls[0] - 0.3) < 1e-9 and abs(fr.ls[1] - 0.6) < 1e-9 for fr in b_frames))
        pressed = [round(fr.t, 2) for fr in b_frames if fr.buttons & gs.BUTTONS['B']]
        self.assertEqual(pressed, [1.2, 1.25])
        self.assertEqual(sc.step_bounds(), [('a', 0.0, 1.0), ('b', 1.0, 2.0)])

    def test_flick_reverses_quickly(self):
        sc = self.all['flick_180_run']
        st = next(s for s in sc.steps if s.name == 'flick')
        self.assertAlmostEqual(st.tracks['ls'].value(0.05)[1], 0.0)
        self.assertAlmostEqual(st.tracks['ls'].value(0.1)[1], -1.0)

    def test_parse_errors(self):
        bad = [
            {'steps': []},
            {'steps': [{'name': 'a', 'duration': 0}]},
            {'steps': [{'name': 'a', 'duration': 1, 'ls': [[0, 0]]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'ls': [[1, 0, 0], [0, 0, 1]]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'ls': [[0, 0, 0]], 'ls_polar': [[0, 0, 0]]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'buttons': [{'t': 0, 'button': 'Q'}]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'buttons': [{'t': 2, 'button': 'A'}]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'lx': [[0, 0]]}]},
            {'steps': [{'name': 'a', 'duration': 1, 'interp': 'cubic', 'ls': [[0, 0, 0]]}]},
            {'steps': [{'name': 'a', 'duration': 1}, {'name': 'a', 'duration': 1}]},
        ]
        for raw in bad:
            with self.assertRaises(gs.ScenarioError, msg=str(raw)):
                gs.parse_scenario('x', raw)

    def test_clamp(self):
        x, y = gs.clamp_stick(1.0, 1.0)
        self.assertAlmostEqual(math.hypot(x, y), 1.0)

    def test_describe_runs(self):
        txt = gs.describe(self.all['flick_180_sprint'])
        self.assertIn('LS@0.40s', txt)


# ---------------------------------------------------------------------------------------------- logs

def _trace(ts, kind='tick', clip='M_Neutral_Run_Loop_F', cost=0.0, off=(0.0, 0.0), L=0.0, R=0.0, rate=1.0,
           bt=0, btc=0, cam=0, pl=0, err=0.0, loco=1):
    return (f'[2026-10-07 {ts}] [info    ] [ 1] [UTPP] UTPP MMTRACE {"TRANS" if kind == "TRANS" else "tick"} '
            f'clip={clip} f=1 cost={cost:.3f} |v|=3.00 rate={rate:.2f} loco={loco} trans=1 mv=(0.00,1.00) '
            f'cam={cam} pl={pl} L=1/1.00/{L:.1f}cm R=0/0.00/{R:.1f}cm bt={bt} tgt=0 err={err:.1f} '
            f'btCmds={btc} off=({off[0]:.1f},{off[1]:.1f})cm')


class LogTests(unittest.TestCase):
    def test_real_logs_parse(self):
        for path, n_trace, n_trans in ((LOG_038, 322, 91), (LOG_041, 154, 34)):
            if not os.path.isfile(path):
                self.skipTest(f'missing {path}')
            lines = gt_log.parse_file(path)
            self.assertEqual(sum(1 for l in lines if l.is_trace), n_trace)
            self.assertEqual(sum(1 for l in lines if l.kind == 'TRANS'), n_trans)
            self.assertEqual(lines, sorted(lines, key=lambda l: l.ts))
            self.assertTrue(any(l.kind == 'CET' for l in lines))
            tr = next(l for l in lines if l.kind == 'TRANS')
            self.assertIsNotNone(tr.pair('off'))
            self.assertIsNotNone(tr.foot('L'))
            self.assertIn('clip=', tr.compact())

    def test_real_log_window_metrics(self):
        if not os.path.isfile(LOG_041):
            self.skipTest('missing log')
        lines = gt_log.parse_file(LOG_041)
        m = gt_log.compute_metrics(lines, datetime(2026, 10, 7, 0, 49, 58), datetime(2026, 10, 7, 0, 50, 20))
        self.assertEqual(m['trans_lines'], 29)
        self.assertAlmostEqual(m['transitions_per_s'], 29 / 22, places=3)
        for c in ('Start', 'Stop', 'Turn', 'Reface', 'Loop', 'Transition'):
            self.assertIn(c, m['clip_classes_played'])
        self.assertGreater(m['offset_saturated_s'], 0)
        self.assertGreaterEqual(m['offset_max_cm'], 24)
        self.assertGreater(m['bodyturn_cmds'], 0)
        self.assertEqual(m['mm_driver'], 'v1')                            # 0.4.1 log: no MOTION lines
        self.assertEqual(m['not_applicable'], [])
        # no carry-over: the window has no sample until its first in-window line (here 0.77 s after t0)
        self.assertGreater(m['trace_coverage'], 0.9)
        self.assertLess(m['trace_coverage'], 1.0)

    def test_synthetic_metrics(self):
        text = '\n'.join([
            _trace('10:00:00.000', cam=170, pl=170, btc=5),
            _trace('10:00:01.000', 'TRANS', 'M_Neutral_Run_Start_F_Lfoot', cost=2.0, off=(30, 0), L=4.0, bt=1,
                   btc=7, cam=-170, pl=160, rate=0.7, err=-12),
            _trace('10:00:01.500', 'TRANS', 'M_Neutral_Run_Turn_L_135_Lfoot', cost=4.0, off=(10, 0), R=-6.5,
                   btc=9, cam=-150, pl=-150, rate=1.3),
            _trace('10:00:02.500', cam=-150, pl=-150, btc=12),
            '[2026-10-07 10:00:02.000] [info    ] [ 1] [UTPP] UTPP MMGATE gate=0x100 (was 0x0) psm=0x1',
            'garbage line',
        ])
        lines = gt_log.parse_text(text)
        self.assertEqual(len(lines), 5)
        m = gt_log.compute_metrics(lines, datetime(2026, 10, 7, 10, 0, 0, 500000), datetime(2026, 10, 7, 10, 0, 3))
        self.assertEqual(m['trans_lines'], 2)
        self.assertAlmostEqual(m['transitions_per_s'], 0.8)
        self.assertAlmostEqual(m['cost_mean'], 3.0)
        self.assertAlmostEqual(m['cost_max'], 4.0)
        self.assertEqual(m['trans_classes'], {'Start': 1, 'Turn': 1})
        # no carry-over (2026-10-23): the 00.000 tick before the window adds no clip, no time and no counter baseline
        self.assertEqual(m['clips'], ['Run_Start_F_Lfoot', 'Run_Turn_L_135_Lfoot', 'Run_Loop_F'])
        self.assertAlmostEqual(m['offset_saturated_s'], 0.5)              # 01.0 -> 01.5 at |off| = 30
        self.assertAlmostEqual(m['footlock_corr_max_cm'], 6.5)
        self.assertAlmostEqual(m['rate_min'], 0.7)
        self.assertAlmostEqual(m['rate_max'], 1.3)
        self.assertEqual(m['bodyturn_cmds'], 12 - 5)                       # the 00.000 tick is the delta baseline only
        self.assertAlmostEqual(m['bodyturn_active_s'], 0.5)
        self.assertAlmostEqual(m['cam_yaw_net_deg'], 40.0)                 # baseline 170 -> -170 (+20) -> -150 (+20)
        self.assertAlmostEqual(m['cam_pl_gap_max_deg'], 30.0)              # -170 vs 160 wraps to 30
        self.assertEqual(m['gate_lines'], 1)
        self.assertAlmostEqual(m['trace_coverage'], 0.8)                   # 01.0 -> 03.0 of 00.5 -> 03.0
        self.assertEqual(m['mm_driver'], 'v1')

    def test_crouch_metrics_and_verdicts(self):
        base = datetime(2026, 10, 7, 10, 0, 0)
        thr = gt_log.merge_thresholds(gt_log.DEFAULT_THRESHOLDS,
                                      {'walk.crouch_fraction': {'min': 0.8, 'msg': 'crouch not engaged'},
                                       'stand_check.stand_fraction': {'min': 0.8, 'msg': 'crouch not released'}})

        def metrics(locos):
            lines = gt_log.parse_text('\n'.join(_trace(f'10:00:{i:02d}.000', loco=v) for i, v in enumerate(locos)))
            return gt_log.compute_metrics(lines, base, base + timedelta(seconds=len(locos)))
        crouched = metrics([2, 2, 2, 2, 2])
        self.assertAlmostEqual(crouched['crouch_fraction'], 1.0)
        self.assertAlmostEqual(crouched['stand_fraction'], 0.0)
        self.assertEqual([f for f in gt_log.evaluate(crouched, thr, 'walk') if 'crouch' in f], [])
        stood = metrics([1, 1, 1, 1, 1])                  # the 2026-10-07 symptom: loco=1 all the time
        self.assertAlmostEqual(stood['crouch_fraction'], 0.0)
        fails = gt_log.evaluate(stood, thr, 'walk')
        self.assertTrue(any(f.startswith('crouch not engaged: crouch_fraction=0.0 < 0.8') for f in fails), fails)
        self.assertEqual([f for f in gt_log.evaluate(stood, thr, 'stand_check') if 'crouch' in f], [])
        late = metrics([2, 2, 2, 2, 2])                    # still crouched at the end -> not released
        fails = gt_log.evaluate(late, thr, 'stand_check')
        self.assertTrue(any(f.startswith('crouch not released: stand_fraction=0.0 < 0.8') for f in fails), fails)
        partial = metrics([1, 2, 2, 2, 2])                 # time weighted: 4 of 5 s crouched = 0.8 passes
        self.assertAlmostEqual(partial['crouch_fraction'], 0.8)
        self.assertEqual([f for f in gt_log.evaluate(partial, thr, 'walk') if 'crouch' in f], [])
        self.assertEqual([f for f in gt_log.evaluate(metrics([1, 1, 2, 2, 2]), thr, 'walk') if 'crouch' in f][:1],
                         ['crouch not engaged: crouch_fraction=0.6 < 0.8'])
        empty = gt_log.compute_metrics([], base, base + timedelta(seconds=3))
        self.assertIsNone(empty['crouch_fraction'])        # no samples -> samples<1 already fails, no false verdict

    def test_empty_window(self):
        m = gt_log.compute_metrics([], datetime(2026, 1, 1), datetime(2026, 1, 1, 0, 0, 5))
        self.assertEqual(m['samples'], 0)
        self.assertEqual(gt_log.evaluate(m, gt_log.DEFAULT_THRESHOLDS), ['samples=0 < 1'])

    def test_clip_class(self):
        cases = {'M_Neutral_Stand_Idle_Loop': 'Idle', 'M_Neutral_Run_Loop_F': 'Loop',
                 'M_Neutral_Sprint_Stop_FL_Rfoot': 'Stop', 'M_Neutral_Walk_Start_F_Rfoot': 'Start',
                 'M_Neutral_Walk_Turn_R_090_Rfoot': 'Turn', 'M_Neutral_Run_Reface_Start_F_L_090': 'Reface',
                 'M_Neutral_Transition_Walk_to_Run_Rfoot': 'Transition', 'M_Neutral_Run_Pivot_L_180': 'Pivot',
                 'M_Neutral_Crouch_Idle_Turn_L_090': 'Turn', 'X_Stand_Turn_L_090_srcX': 'Turn', 'Foo': 'Other'}
        for clip, cls in cases.items():
            self.assertEqual(gt_log.clip_class(clip), cls, clip)

    def test_thresholds(self):
        thr = gt_log.merge_thresholds(gt_log.DEFAULT_THRESHOLDS, {'rate_min': None, 'stop.transitions_per_s':
                                                                 {'max': 5}})
        self.assertIsNone(thr['rate_min'])
        m = {'transitions_per_s': 4.0, 'samples': 3, 'rate_min': 0.1}
        self.assertEqual(gt_log.evaluate(m, thr, 'stop'), [])
        self.assertEqual(gt_log.evaluate(m, thr, 'walk'), ['transitions_per_s=4.0 > 3.0'])
        file_thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        self.assertIn('cost_max', file_thr)
        self.assertNotIn('_note', file_thr)

    def test_newest_log(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(gt_log.newest_log(d))
            for i, n in enumerate(('utpp-a.log', 'utpp-b.log', 'other.log')):
                p = os.path.join(d, n)
                open(p, 'w').close()
                os.utime(p, (1000 + i, 1000 + i))
            self.assertTrue(gt_log.newest_log(d).endswith('utpp-b.log'))

    def test_offset_read(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'utpp-x.log')
            first = _trace('10:00:00.000') + '\n'
            with open(p, 'w', encoding='utf-8', newline='\n') as f:
                f.write(first + _trace('10:00:01.000', 'TRANS') + '\n')
            lines = gt_log.parse_file(p, len(first.encode()))
            self.assertEqual([l.kind for l in lines], ['TRANS'])

    def test_feet_footstep_costcap_lines(self):
        # loco C0 'MMTRACE feet' (after every TRANS / tick line, same ms), 'MMTRACE footstep' (per foot event) and 0.5.8
        # 'MMTRACE costcap' lines parse into their own kinds and never become trace samples
        def feet(ts):
            return (f'[2026-10-07 {ts}] [info    ] [ 1] [UTPP] UTPP MMTRACE feet L=P cL=1.00 offL=4.2cm corrL=3.1cm wL=1.00 '
                    'slipsL=12 driftL=0 stepsL=0 R=F cR=0.00 offR=0.0cm corrR=0.0cm wR=0.00 slipsR=3 driftR=1 stepsR=0 '
                    'locks=9 relocks=1 clamps=0 settles=40 events=1 fl=1 crouchExcl=1')
        step = ('[2026-10-07 10:00:01.200] [info    ] [ 1] [UTPP] UTPP MMTRACE footstep side=R reason=drift dist=31.5cm '
                'clip=M_Neutral_Crouch_Start_F_Lfoot f=12 loco=2 |v|=1.20')
        cap = ('[2026-10-07 10:00:02.200] [info    ] [ 1] [UTPP] UTPP MMTRACE costcap hold clip=M_Neutral_Run_Loop_F f=3 '
               'best=M_Neutral_Run_Stop_F_Lfoot bestF=0 bestCost=80.0 cap=60.0 |v|=3.00 loco=1 w=1.00')
        stamps = ('10:00:00.000', '10:00:01.000', '10:00:02.000', '10:00:03.000', '10:00:04.000')
        main = [_trace(ts, 'TRANS' if i == 1 else 'tick', 'M_Neutral_Crouch_Idle_Loop', off=(30.0, 0.0), bt=1, loco=2)
                for i, ts in enumerate(stamps)]
        with_side = [x for i, ts in enumerate(stamps) for x in (main[i], feet(ts))] + [step, cap]
        lines = gt_log.parse_text('\n'.join(with_side))
        kinds = [l.kind for l in lines]
        self.assertEqual({k: kinds.count(k) for k in set(kinds)}, {'tick': 4, 'TRANS': 1, 'FEET': 5, 'STEP': 1, 'COSTCAP': 1})
        self.assertEqual(sum(1 for l in lines if l.is_trace), 5)
        f0 = next(l for l in lines if l.kind == 'FEET')
        self.assertEqual((f0.fields['L'], f0.fields['offL'], f0.fields['R'], f0.fields['crouchExcl']), ('P', '4.2cm', 'F', '1'))
        self.assertEqual((f0.num('slipsL'), f0.num('driftR'), f0.num('cL')), (12.0, 1.0, 1.0))
        st = next(l for l in lines if l.kind == 'STEP')
        self.assertEqual((st.fields['side'], st.fields['reason'], st.fields['dist'], st.fields['clip']),
                         ('R', 'drift', '31.5cm', 'M_Neutral_Crouch_Start_F_Lfoot'))
        self.assertEqual((st.num('f'), st.num('loco'), st.num('|v|')), (12.0, 2.0, 1.2))
        cc = next(l for l in lines if l.kind == 'COSTCAP')
        self.assertEqual((cc.fields['best'], cc.num('bestCost'), cc.num('cap')), ('M_Neutral_Run_Stop_F_Lfoot', 80.0, 60.0))
        self.assertIn('FEET  L=P cL=1.00', f0.compact())
        self.assertIn('STEP  side=R reason=drift', st.compact())
        self.assertIn('COSTCAP hold clip=', cc.compact())
        # metrics and verdicts: identical with and without the side lines (the guards keep firing)
        t0, t1 = datetime(2026, 10, 7, 10, 0, 0), datetime(2026, 10, 7, 10, 0, 5)
        m_side = gt_log.compute_metrics(lines, t0, t1)
        m_main = gt_log.compute_metrics(gt_log.parse_text('\n'.join(main)), t0, t1)
        self.assertEqual(m_side, m_main)
        self.assertEqual((m_side['samples'], m_side['offset_saturated_s'], m_side['bodyturn_active_s'], m_side['crouch_fraction']),
                         (5, 5.0, 5.0, 1.0))
        self.assertIn('offset_saturated_s=5.0 > 0.5', gt_log.evaluate(m_side, gt_log.DEFAULT_THRESHOLDS))
        ann = video_sheets.frame_annotation(lines, datetime(2026, 10, 7, 10, 0, 4, 500000), 0.25)
        self.assertIn('clip=Crouch_Idle_Loop', ann[0])
        # Source\tools\recon\mmtrace_summary.py: the side lines keep their own label, the tick / TRANS lines are unchanged
        import contextlib
        import io
        import mmtrace_summary  # noqa: E402 (path added by gt_log)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'utpp-feet.log')
            with open(p, 'w', encoding='utf-8', newline='\n') as f:
                f.write('\n'.join(with_side) + '\n')
            out = io.StringIO()
            argv = sys.argv
            try:
                sys.argv = ['mmtrace_summary.py', p]
                with contextlib.redirect_stdout(out):
                    mmtrace_summary.main()
            finally:
                sys.argv = argv
        rows = out.getvalue().splitlines()
        self.assertEqual(len(rows), 12)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith('feet  L=P')), 5)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith('step  side=R')), 1)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith('ccap  hold')), 1)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith(('tick  clip=Crouch_Idle_Loop', 'TRANS clip=Crouch'))), 5)

    def test_mmair_lines(self):
        # AIR HOLD stage: 'UTPP MMAIR' phase-change lines parse into their own kind 'AIR' (never trace samples, metrics unchanged)
        # and keep their own label in mmtrace_summary
        air = [('[2026-10-07 10:00:01.100] [info    ] [ 1] [UTPP] UTPP MMAIR phase=TakeOff was=Ground action=none mode=hold detail=18 '
                'onG=0 stale=0 vz=5.56 dist=-2.00 fall=0 ttl=1.50 known=0 own=1 contact=0.00 rate=0.50 ep=1 to=1 falls=0 lands=0 hb=0 '
                'resets=0'),
               ('[2026-10-07 10:00:01.800] [info    ] [ 1] [UTPP] UTPP MMAIR phase=Ground was=LandApproach action=handBack mode=hold '
                'detail=23 onG=1 stale=0 vz=0.00 dist=-2.00 fall=1 ttl=-1.00 known=0 own=0 contact=0.42 rate=1.00 ep=1 to=1 falls=0 '
                'lands=1 hb=1 resets=0')]
        stamps = ('10:00:00.000', '10:00:01.000', '10:00:02.000')
        main = [_trace(ts, 'TRANS' if i == 1 else 'tick', 'M_Neutral_Run_Loop_F') for i, ts in enumerate(stamps)]
        lines = gt_log.parse_text('\n'.join(main + air))
        kinds = [l.kind for l in lines]
        self.assertEqual({k: kinds.count(k) for k in set(kinds)}, {'tick': 2, 'TRANS': 1, 'AIR': 2})
        self.assertEqual(sum(1 for l in lines if l.is_trace), 3)
        a0, a1 = [l for l in lines if l.kind == 'AIR']
        self.assertEqual((a0.fields['phase'], a0.fields['was'], a0.fields['mode'], a0.num('detail'), a0.num('vz')),
                         ('TakeOff', 'Ground', 'hold', 18.0, 5.56))
        self.assertEqual((a1.fields['action'], a1.num('hb'), a1.num('contact'), a1.num('rate')), ('handBack', 1.0, 0.42, 1.0))
        self.assertIn('AIR   phase=TakeOff was=Ground', a0.compact())
        t0, t1 = datetime(2026, 10, 7, 10, 0, 0), datetime(2026, 10, 7, 10, 0, 3)
        self.assertEqual(gt_log.compute_metrics(lines, t0, t1), gt_log.compute_metrics(gt_log.parse_text('\n'.join(main)), t0, t1))
        import contextlib
        import io
        import mmtrace_summary  # noqa: E402 (path added by gt_log)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'utpp-air.log')
            with open(p, 'w', encoding='utf-8', newline='\n') as f:
                f.write('\n'.join(main + air) + '\n')
            out = io.StringIO()
            argv = sys.argv
            try:
                sys.argv = ['mmtrace_summary.py', p]
                with contextlib.redirect_stdout(out):
                    mmtrace_summary.main()
            finally:
                sys.argv = argv
        rows = out.getvalue().splitlines()
        self.assertEqual(len(rows), 5)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith('air   phase=')), 2)
        self.assertEqual(sum(1 for r in rows if r[12:].startswith(('tick  clip=Run_Loop_F', 'TRANS clip=Run_Loop_F'))), 3)

    def test_strafe_appended_fields(self):
        # B2 strafe S1 appends 'strafe= dir= gait= rateSrc=' at the END of every MMTRACE TRANS / tick line (MotionHost.cpp):
        # they parse as plain fields, do not shadow rate= / f= / turnRate=, and change neither the metrics nor the summary
        tail = ' mask=1.00/0x0 hold=0 turnRate=0.00/0.00'
        cases = ((' strafe=1 dir=-90 gait=0 rateSrc=own', ('1', '-90', '0', 'own')),
                 (' strafe=0 dir=0 gait=1 rateSrc=map', ('0', '0', '1', 'map')),
                 (' strafe=0 dir=0 gait=0 rateSrc=off', ('0', '0', '0', 'off')))
        stamps = ('10:00:00.000', '10:00:01.000', '10:00:02.000')
        base = [_trace(ts, 'TRANS' if i == 0 else 'tick', rate=0.75, off=(12.0, 0.0), bt=1) + tail for i, ts in enumerate(stamps)]
        appended = [b + c[0] for b, c in zip(base, cases)]
        old, new = gt_log.parse_text('\n'.join(base)), gt_log.parse_text('\n'.join(appended))
        self.assertEqual([l.kind for l in new], ['TRANS', 'tick', 'tick'])
        self.assertTrue(all(l.is_trace for l in new))
        for l, (_, want) in zip(new, cases):
            self.assertEqual((l.fields['strafe'], l.fields['dir'], l.fields['gait'], l.fields['rateSrc']), want)
            self.assertEqual((l.num('rate'), l.num('f'), l.fields['turnRate'], l.pair('off')), (0.75, 1.0, '0.00/0.00', (12.0, 0.0)))
            self.assertEqual(l.num('dir'), float(want[1]))
        for a, b in zip(old, new):
            self.assertEqual({k: v for k, v in b.fields.items() if k not in ('strafe', 'dir', 'gait', 'rateSrc')}, a.fields)
            self.assertEqual(a.compact(), b.compact())
        t0, t1 = datetime(2026, 10, 7, 10, 0, 0), datetime(2026, 10, 7, 10, 0, 3)
        self.assertEqual(gt_log.compute_metrics(new, t0, t1), gt_log.compute_metrics(old, t0, t1))
        import contextlib
        import io
        import mmtrace_summary  # noqa: E402 (path added by gt_log)
        outs = []
        with tempfile.TemporaryDirectory() as d:
            for name, rows in (('old', base), ('new', appended)):
                p = os.path.join(d, f'utpp-{name}.log')
                with open(p, 'w', encoding='utf-8', newline='\n') as f:
                    f.write('\n'.join(rows) + '\n')
                out = io.StringIO()
                argv = sys.argv
                try:
                    sys.argv = ['mmtrace_summary.py', p]
                    with contextlib.redirect_stdout(out):
                        mmtrace_summary.main()
                finally:
                    sys.argv = argv
                outs.append(out.getvalue())
        self.assertEqual(outs[0], outs[1])
        self.assertEqual(len(outs[1].splitlines()), 3)
        self.assertIn('rate=0.75', outs[1])

    def test_chest_appended_fields(self):
        # B2 strafe S2 appends ' chestDev= chestCorr=' after 'rateSrc=' on MMTRACE TRANS / tick lines, only on feeds where
        # ChestAlign ran with valid diagnostics: plain fields, nothing else shadowed, metrics and the summary unchanged
        tail = ' mask=1.00/0x0 hold=0 turnRate=0.00/0.00 strafe=1 dir=90 gait=0 rateSrc=own'
        chest = (' chestDev=-31.4 chestCorr=24.0', '', ' chestDev=7.5 chestCorr=-6.2')
        stamps = ('10:00:00.000', '10:00:01.000', '10:00:02.000')
        base = [_trace(ts, 'TRANS' if i == 0 else 'tick', rate=0.75, off=(12.0, 0.0), bt=1) + tail for i, ts in enumerate(stamps)]
        appended = [b + c for b, c in zip(base, chest)]
        old, new = gt_log.parse_text('\n'.join(base)), gt_log.parse_text('\n'.join(appended))
        self.assertEqual([l.kind for l in new], ['TRANS', 'tick', 'tick'])
        self.assertEqual((new[0].num('chestDev'), new[0].num('chestCorr')), (-31.4, 24.0))
        self.assertNotIn('chestDev', new[1].fields)
        self.assertEqual((new[2].num('chestDev'), new[2].num('chestCorr')), (7.5, -6.2))
        for a, b in zip(old, new):
            self.assertEqual({k: v for k, v in b.fields.items() if k not in ('chestDev', 'chestCorr')}, a.fields)
            self.assertEqual((b.fields['rateSrc'], b.num('rate'), b.num('dir')), ('own', 0.75, 90.0))
            self.assertEqual(a.compact(), b.compact())
        t0, t1 = datetime(2026, 10, 7, 10, 0, 0), datetime(2026, 10, 7, 10, 0, 3)
        self.assertEqual(gt_log.compute_metrics(new, t0, t1), gt_log.compute_metrics(old, t0, t1))
        import contextlib
        import io
        import mmtrace_summary  # noqa: E402 (path added by gt_log)
        outs = []
        with tempfile.TemporaryDirectory() as d:
            for name, rows in (('old', base), ('new', appended)):
                p = os.path.join(d, f'utpp-{name}.log')
                with open(p, 'w', encoding='utf-8', newline='\n') as f:
                    f.write('\n'.join(rows) + '\n')
                out = io.StringIO()
                argv = sys.argv
                try:
                    sys.argv = ['mmtrace_summary.py', p]
                    with contextlib.redirect_stdout(out):
                        mmtrace_summary.main()
                finally:
                    sys.argv = argv
                outs.append(out.getvalue())
        self.assertEqual(outs[0], outs[1])
        self.assertEqual(len(outs[1].splitlines()), 3)


MOTION_EXCERPT = os.path.join(HERE, 'testdata', 'utpp-motion-20261023-182525-sprint_stop.txt')  # .txt: *.log is gitignored


class MotionLogTests(unittest.TestCase):
    """Real MM v2 log excerpt (game 2.5, 2026-10-23 run, step sprint_stop
    18:25:56.902485 -> 18:25:59.902242; source utpp-2026-10-23-18-15-42.log, MOTIONPOSE lines trimmed): the MMTRACE tick
    carries frozen v1 placeholders (clip=M_Neutral_Crouch_Idle_Loop f=0 searches=0 trans=0) while 'UTPP MOTION' shows the
    real selection."""
    T0 = datetime(2026, 10, 23, 18, 25, 56, 902485)
    T1 = datetime(2026, 10, 23, 18, 25, 59, 902242)

    @classmethod
    def setUpClass(cls):
        cls.lines = gt_log.parse_file(MOTION_EXCERPT)

    def test_motion_parsing(self):
        v2 = [l for l in self.lines if l.kind == 'MOTION']
        self.assertEqual(len(v2), 12)
        self.assertFalse(any(l.is_trace for l in v2))
        first = v2[0]
        self.assertEqual((first.fields['db'], first.motion_name('db'), first.motion_name('clip')),
                         ('24:PSD_Dense_Stand_Sprint_Starts', 'PSD_Dense_Stand_Sprint_Starts', 'M_Neutral_Sprint_Start_F_Lfoot'))
        self.assertEqual((first.num('ct'), first.num('cost')), (1.893, -0.037))
        self.assertEqual(first.motion_ov(), (0, 0, 0, 1))                  # overlay/aim/crouch moving
        self.assertIn('MOTION  db=24:PSD_Dense_Stand_Sprint_Starts clip=483:Sprint_Start_F_Lfoot', first.compact())
        self.assertEqual(sum(1 for l in self.lines if l.kind == 'tick'), 5)

    def test_motion_db_relabel_is_no_change(self):
        # 2026-11-11 16:06:54.8-55.4: Walk reface -> Run reface (Run_Starts) -> the same clip under Run_SpinTransition with ct
        # 0.167 -> 0.339 in 0.184 s (no animation change) -> Sprint reface -> Walk_to_Sprint: 3 changes, not 4
        lines = gt_log.parse_file(os.path.join(HERE, 'testdata', 'utpp-motion-20261111-160654-reface_relabel.txt'))
        v2 = [l for l in lines if l.kind == 'MOTION']
        m = {}
        gt_log._motion_metrics(m, v2[1:], v2[0], v2[1].ts, v2[-1].ts, 1.0)
        self.assertEqual(m['motion_changes'], 3)
        # a jump inside the same clip under another label still counts
        jumped = gt_log.parse_file(os.path.join(HERE, 'testdata', 'utpp-motion-20261111-160654-reface_relabel.txt'))
        v2j = [l for l in jumped if l.kind == 'MOTION']
        for l in v2j:
            if 'SpinTransition' in l.fields.get('db', ''):
                l.fields['ct'] = '0.900'
        m = {}
        gt_log._motion_metrics(m, v2j[1:], v2j[0], v2j[1].ts, v2j[-1].ts, 1.0)
        self.assertEqual(m['motion_changes'], 4)

    def test_motion_step_metrics_and_placeholders(self):
        m = gt_log.compute_metrics(self.lines, self.T0, self.T1)
        self.assertEqual(m['mm_driver'], 'v2')
        # v1-only metrics are not applicable (not silently passed): None and listed
        for k in gt_log.BASE_ONLY_METRICS:
            self.assertIsNone(m[k], k)
        self.assertEqual(m['not_applicable'], list(gt_log.BASE_ONLY_METRICS))
        self.assertTrue(m['base_trace_frozen'])                          # placeholder detection
        self.assertEqual(gt_log.base_trace_frozen([l for l in self.lines if l.is_trace]), True)
        # selection from MOTION: Sprint_Loop_F (18:25:56.446, reference only) -> Run_to_Walk -> Sprint_Stop
        self.assertEqual(m['motion_samples'], 7)
        self.assertEqual(m['motion_changes'], 2)
        self.assertAlmostEqual(m['motion_transitions_per_s'], round(2 / 2.999757, 3))
        self.assertEqual(m['clips'], ['Transition_Run_to_Walk_Rfoot', 'Sprint_Stop_F_Rfoot'])
        self.assertEqual(m['trans_classes'], {'Transition': 1, 'Stop': 1})
        self.assertEqual(m['motion_dbs'], ['PSD_Dense_Stand_Walk_Loops', 'PSD_Dense_Stand_Sprint_Stops'])
        self.assertEqual((m['motion_cost_mean'], m['motion_cost_max']), (round((0.065 + 0.048 - 0.2 * 5) / 7, 3), 0.065))
        self.assertEqual(m['motion_crouch_fraction'], 0.0)
        self.assertNotIn('Idle', m['clip_classes_played'])             # the frozen tick clip is never used
        # tick fields that do not come from the v1 matcher still count: in-window ticks only (no carry-over)
        self.assertEqual(m['samples'], 3)
        self.assertEqual(m['speed_max'], 0.0)                          # the 8.21 m/s sprint tick at 56.558 is not carried
        self.assertAlmostEqual(m['trace_coverage'], round((self.T1 - datetime(2026, 10, 23, 18, 25, 57, 574000))
                                                          .total_seconds() / 2.999757, 3))
        self.assertEqual(m['stand_fraction'], 1.0)
        self.assertAlmostEqual(m['footlock_corr_max_cm'], 5.0)            # R=1/1.00/5.0cm at 57.574
        self.assertEqual(gt_log.evaluate(m, gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json')), 'sprint_stop'),
                         [])

    def test_no_carry_over_on_base_fields(self):
        # same excerpt scored as a v1 log (MOTION lines removed): the prior tick (56.558, off=(0.2,-30.0)) used to be held
        # into the window and gave offset_saturated_s=0.672 (a false FAIL); in-window offsets are only 0.5 / 0.0 cm
        v1 = [l for l in self.lines if l.kind != 'MOTION']
        m = gt_log.compute_metrics(v1, self.T0, self.T1)
        self.assertEqual(m['mm_driver'], 'v1')
        self.assertEqual(m['offset_saturated_s'], 0.0)
        self.assertEqual(m['offset_max_cm'], 0.5)
        self.assertEqual(m['clips'], ['Crouch_Idle_Loop'])
        self.assertIsNone(m['base_trace_frozen'])                        # only computed under v2

    def test_window_without_motion_lines_but_recent_heartbeat_is_motion(self):
        t0 = datetime(2026, 10, 23, 18, 25, 59, 600000)
        m = gt_log.compute_metrics(self.lines, t0, t0 + timedelta(seconds=0.3))
        self.assertEqual((m['mm_driver'], m['motion_samples'], m['motion_changes']), ('v2', 0, 0))
        self.assertIn('no MOTION lines in window: motion_samples=0 < 1', gt_log.evaluate(m, gt_log.DEFAULT_THRESHOLDS))

    def test_window_shorter_than_the_trace_period_may_fall_between_two_trace_lines(self):
        thr = gt_log.DEFAULT_THRESHOLDS
        short = {'duration_s': 0.9, 'samples': 0, 'motion_samples': 2}
        self.assertTrue(gt_log.trace_gap(short))
        self.assertEqual(gt_log.evaluate(short, thr), [])
        # a window of a full trace period must hold a sample; no motion lines is still a failure of a short window
        self.assertEqual(gt_log.evaluate({**short, 'duration_s': 1.0}, thr), ['samples=0 < 1'])
        self.assertEqual(gt_log.evaluate({**short, 'motion_samples': 0}, thr),
                         ['samples=0 < 1', 'no MOTION lines in window: motion_samples=0 < 1'])
        self.assertEqual(gt_log.evaluate({**short, 'motion_samples': None}, thr), ['samples=0 < 1'])
        # the other limits of the window still apply
        self.assertEqual(gt_log.evaluate({**short, 'motion_transitions_per_s': 3.3}, thr), ['motion_transitions_per_s=3.3 > 3.0'])

    def test_frozen_tick_without_motion_is_never_base(self):
        # review L1: in a v2 log, a step whose window (and lookback) has no MOTION line but a frozen v1 tick is scored as v2
        # (motion_samples=0 fails) instead of silently as v1
        t0 = datetime(2026, 10, 23, 18, 26, 1)
        tick = gt_log.parse_text('[2026-10-23 18:26:01.500] [info    ] [ 1] [UTPP] UTPP MMTRACE tick '
                                 'clip=M_Neutral_Crouch_Idle_Loop f=0 cost=0.000 |v|=0.00 rate=1.00 loco=1 searches=0 trans=0 '
                                 'L=1/1.00/0.0cm R=1/1.00/0.0cm off=(0.0,30.0)cm')
        m = gt_log.compute_metrics(self.lines + tick, t0, t0 + timedelta(seconds=2))
        self.assertEqual(m['mm_driver'], 'v2')
        self.assertIn('no MOTION lines in window', m['mm_driver_note'])
        self.assertIsNone(m['offset_saturated_s'])
        self.assertIn('no MOTION lines in window: motion_samples=0 < 1', gt_log.evaluate(m, gt_log.DEFAULT_THRESHOLDS))
        # the same tick in a log without any MOTION line stays a v1 step
        self.assertEqual(gt_log.compute_metrics(tick, t0, t0 + timedelta(seconds=2))['mm_driver'], 'v1')

    def test_cost_sentinel_only(self):
        # review L2: only the exact 'no match' sentinel -1.000 is dropped; real negative costs (-0.200) are kept
        base = [l for l in self.lines if l.kind == 'MOTION' and self.T0 <= l.ts <= self.T1]
        s = base[0].raw.replace(' cost=0.065 ', ' cost=-1.000 ')
        self.assertNotEqual(s, base[0].raw)
        lines = sorted([l for l in self.lines if l is not base[0]] + gt_log.parse_text(s), key=lambda l: l.ts)
        m = gt_log.compute_metrics(lines, self.T0, self.T1)
        self.assertEqual((m['motion_cost_mean'], m['motion_cost_max']), (round((0.048 - 0.2 * 5) / 6, 3), 0.048))

    def test_selection_unscored(self):
        # review M2: motion_transitions_per_s max 3.0 (provisional) applies -> scored; disabling it -> selection_unscored
        thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        m = gt_log.compute_metrics(self.lines, self.T0, self.T1)
        self.assertEqual(thr['motion_transitions_per_s'], {'max': 3.0})
        self.assertFalse(gt_log.selection_unscored(m, thr, 'sprint_stop'))
        off = gt_log.merge_thresholds(thr, {'sprint_stop.motion_transitions_per_s': None})
        self.assertTrue(gt_log.selection_unscored(m, off, 'sprint_stop'))
        self.assertIn('motion_transitions_per_s=4.0 > 3.0', gt_log.evaluate({**m, 'motion_transitions_per_s': 4.0}, thr, 'x'))
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        for t, want in ((thr, False), (off, True)):
            plan = self._plan()
            rows = gt_run.analyze(plan, self.lines, t, scen)
            self.assertEqual(rows[0]['selection_unscored'], want)
            with tempfile.TemporaryDirectory() as d:
                summary = gt_run.write_outputs(d, plan, rows, self.lines, MOTION_EXCERPT)
            self.assertEqual(summary['selection_unscored'], want)
            self.assertEqual(summary['steps'][0]['selection_unscored'], want)

    def _plan(self):
        return {'runs': [{'scenario': 'sprint_start_stop', 'run': 1, 'aborted': None, 'steps': [
            {'name': 'sprint_stop', 'start': self.T0, 'end': self.T1, 'metrics': True}]}]}

    def test_precondition(self):
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        plan = self._plan()
        rows = gt_run.analyze(plan, self.lines, thr, scen)
        pre = gt_run.apply_precondition(plan, rows, self.lines)
        self.assertTrue(pre['ok'])
        self.assertEqual(pre['motion_lines'], 7)
        self.assertEqual(pre['trace_echo'], '2026-10-23 18:25:27.738000')
        self.assertEqual(rows[0]['verdict'], 'PASS')
        # no MOTION lines: not scored (INVALID, overall FAIL), unless --allow-base
        v1 = [l for l in self.lines if l.kind != 'MOTION']
        plan = self._plan()
        rows = gt_run.analyze(plan, v1, thr, scen)
        pre = gt_run.apply_precondition(plan, rows, v1)
        self.assertFalse(pre['ok'])
        self.assertIn('mm.motionDriver not active', pre['reason'])
        self.assertEqual(rows[0]['verdict'], gt_run.INVALID_VERDICT)
        self.assertTrue(rows[0]['fails'][0].startswith('precondition: mm.motionDriver not active'))
        with tempfile.TemporaryDirectory() as d:
            summary = gt_run.write_outputs(d, plan, rows, v1, MOTION_EXCERPT)
            self.assertEqual(summary['overall'], 'FAIL')
            self.assertEqual(summary['counts'][gt_run.INVALID_VERDICT], 1)
            self.assertFalse(summary['mm_precondition']['ok'])
        plan = self._plan()
        rows = gt_run.analyze(plan, v1, thr, scen)
        gt_run.apply_precondition(plan, rows, v1, allow_base=True)
        self.assertEqual(rows[0]['verdict'], 'PASS')

    def test_step_file_marks_not_applicable(self):
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        plan = self._plan()
        rows = gt_run.analyze(plan, self.lines, gt_log.DEFAULT_THRESHOLDS, scen)
        gt_run.apply_precondition(plan, rows, self.lines)
        with tempfile.TemporaryDirectory() as d:
            gt_run.write_outputs(d, plan, rows, self.lines, MOTION_EXCERPT)
            with open(os.path.join(d, 'sprint_start_stop_r1.txt'), encoding='utf-8') as f:
                txt = f.read()
        self.assertIn('mm_driver=v2', txt)
        self.assertIn('n/a under MM v2 (v1 matcher fields, not scored): trans_lines,transitions_per_s', txt)
        self.assertIn('[base tick frozen]', txt)
        self.assertIn('MOTION  db=25:PSD_Dense_Stand_Sprint_Stops clip=486:Sprint_Stop_F_Rfoot', txt)


GAIT_EXCERPT = os.path.join(HERE, 'testdata', 'utpp-motion-20261031-104737-walk_run.txt')  # .txt: *.log is gitignored


class GaitMetricsTests(unittest.TestCase):
    """G6 gait-quality metrics on a real MM v2 log excerpt (utpp-2026-10-31-10-43-27.log, only 'MMTRACE tick' and
    'UTPP MOTION ' lines, copied unchanged): G-walk 10:47:36-10:47:52 (|v| 2.72, walk loops db plays the clip
    Transition_Run_to_Walk_Rfoot) and run 10:49:13-10:49:23 (|v| 3.84, root offset saturated at 30 cm).
    Expected values were counted by hand from the excerpt (see the comments)."""
    W0, W1 = datetime(2026, 10, 31, 10, 47, 37, 0), datetime(2026, 10, 31, 10, 47, 51, 0)
    R0, R1 = datetime(2026, 10, 31, 10, 49, 15, 0), datetime(2026, 10, 31, 10, 49, 22, 0)

    @classmethod
    def setUpClass(cls):
        cls.lines = gt_log.parse_file(GAIT_EXCERPT)
        cls.walk = gt_log.compute_metrics(cls.lines, cls.W0, cls.W1)
        cls.runm = gt_log.compute_metrics(cls.lines, cls.R0, cls.R1)

    def test_parsing(self):
        self.assertEqual((sum(1 for l in self.lines if l.kind == 'MOTION'), sum(1 for l in self.lines if l.kind == 'tick')),
                         (62, 26))
        self.assertEqual(self.walk['mm_driver'], 'v2')

    def test_speed_by_mv_walk(self):
        # ticks with |mv|>0.05 in [37.0, 51.0]: 38.421 .. 44.439 (7), 46.455 (0.43), 47.471 .. 50.480 (4) = 12 values:
        # 0.43, five 2.72 (38.421 39.421 40.422 44.439 48.472), six 2.73 -> median (2.72+2.73)/2, nearest-rank p90 =
        # 11th of 12 = 2.73. The stopped ticks (mv 0) and the ticks at 37.406 / 45.455 / 51.481 are not in a bucket.
        self.assertEqual(self.walk['speed_by_mv'], {'1.00': {'n': 12, 'median': 2.725, 'p90': 2.73}})

    def test_speed_by_mv_run(self):
        # ticks 16.210 (1.13), 17.227 (3.84), 18.233 (3.84), 19.251 (3.85), 20.266 (3.83), 21.266 (3.84); 22.267 is outside
        # sorted 1.13 3.83 3.84 3.84 3.84 3.85 -> median 3.84, p90 = ceil(5.4)=6th = 3.85
        self.assertEqual(self.runm['speed_by_mv'], {'1.00': {'n': 6, 'median': 3.84, 'p90': 3.85}})

    def test_mv_bucket_rounding(self):
        t = ('[2026-10-31 11:00:0{s}.100] [info    ] [ 1] [UTPP] UTPP MMTRACE tick |v|={v} mv=({x},{y}) off=(0.0,0.0)cm')
        text = '\n'.join(t.format(s=i, v=v, x=x, y=y) for i, (v, x, y) in
                          enumerate([(1.0, '0.00', '0.50'), (1.2, '0.30', '0.40'), (2.0, '0.00', '0.52'), (0.0, '0.00', '0.03')]))
        m = gt_log.compute_metrics(gt_log.parse_text(text), datetime(2026, 10, 31, 11, 0, 0), datetime(2026, 10, 31, 11, 0, 9))
        self.assertEqual(m['speed_by_mv'], {'0.50': {'n': 3, 'median': 1.2, 'p90': 2.0}})   # 0.03 <= 0.05: no bucket

    def test_root_offset_walk(self):
        # |off| of the 14 ticks in [37.0, 51.0] (hypot): 0.0 0.1 0.51 2.2 3.7 9.7 9.8 10.4 14.6 14.6 15.4 15.5 17.6 18.5
        # median (9.8 + 10.4005)/2 = 10.1, max 18.5; the 12 moving ticks (without 37.406 and 45.455): 0.51 2.2 3.7 9.7 9.8
        # 10.4 14.6 14.6 15.4 15.5 17.6 18.5 -> median (10.4005 + 14.6003)/2 = 12.5
        self.assertEqual((self.walk['root_off_median_cm'], self.walk['root_off_max_cm'],
                          self.walk['root_off_moving_median_cm']), (10.1, 18.5, 12.5))
        # the v1-only offset metrics stay not applicable under v2
        self.assertIsNone(self.walk['offset_max_cm'])

    def test_root_offset_run_saturated(self):
        # 7 ticks 15.203 (0.0), 16.210 (6.5), 17.227 .. 21.266 five times hypot(0.1, 30.0) = 30.0002
        self.assertEqual((self.runm['root_off_median_cm'], self.runm['root_off_max_cm'],
                          self.runm['root_off_moving_median_cm']), (30.0, 30.0, 30.0))

    def test_db_and_clip_shares_walk(self):
        # held MOTION time in [37.0, 51.0] = 13.795 s (first line 37.205): Idles 37.205-37.656 = 0.451, Walk_Starts
        # 37.656-38.521 = 0.865, Walk_Loops 38.521-44.708 = 6.187, Run_Stops 44.708-46.340 = 1.632, Walk_Starts
        # 46.340-47.205 = 0.865, Walk_Loops 47.205-51.000 = 3.795. The Walk_Loops db plays only clip 508 Transition.
        w = self.walk
        self.assertEqual(w['motion_db_walk_loops_fraction'], round(9.982 / 13.795, 3))      # 0.724
        self.assertEqual(w['motion_db_walk_starts_fraction'], round(1.730 / 13.795, 3))     # 0.125
        self.assertEqual(w['motion_clip_transition_fraction'], round(9.982 / 13.795, 3))
        self.assertEqual(w['motion_walk_loop_clip_fraction'], 0.0)                           # no real loop clip
        self.assertEqual(w['motion_walk_nonloop_fraction'], round(11.712 / 13.795, 3))      # 0.849
        self.assertEqual(w['motion_walk_loop_share'], 0.0)
        self.assertEqual((w['motion_db_walk_loops_fraction'], w['motion_db_walk_starts_fraction']), (0.724, 0.125))

    def test_shares_with_real_loop_clip(self):
        # same excerpt with the clip of the Walk_Loops db replaced by a loop clip: the db share is unchanged, the clip
        # axis moves from Transition to loop (and the loop clip is no longer a non-loop one)
        with open(GAIT_EXCERPT, encoding='utf-8') as f:
            txt = f.read().replace('clip=508:M_Neutral_Transition_Run_to_Walk_Rfoot', 'clip=520:M_Neutral_Walk_Loop_F')
        m = gt_log.compute_metrics(gt_log.parse_text(txt), self.W0, self.W1)
        self.assertEqual((m['motion_db_walk_loops_fraction'], m['motion_clip_transition_fraction']), (0.724, 0.0))
        self.assertEqual(m['motion_walk_loop_clip_fraction'], 0.724)
        self.assertEqual(m['motion_walk_nonloop_fraction'], round(1.730 / 13.795, 3))        # only the Walk_Starts db: 0.125
        self.assertEqual(m['motion_walk_loop_share'], round(9.982 / 11.712, 3))              # 0.852

    def test_reselection_walk(self):
        # steady = moving + '...Loops' db: Walk_Loops 38.521-44.708 and 47.205-51.000 = 9.982 s (Starts at 37.656 / 46.340,
        # Stops at 44.708 are not steady). Pairs of steady lines: switches 0; ct drops 4.095 -> 1.753 (41.588), 4.083 ->
        # 1.749 (44.132), 4.094 -> 1.743 (50.264), each with a feed dt ~0.515 s and new ct > 0.515*1.8+0.02: 3 jumps.
        # The Starts->Loops clip changes (38.521, 47.205) and the stop (44.708) are not counted.
        w = self.walk
        self.assertEqual((w['motion_steady_s'], w['motion_steady_switches'], w['motion_steady_ct_wraps'], w['motion_steady_ct_jumps']),
                         (9.982, 0, 0, 3))
        self.assertEqual(w['motion_steady_reselect_per_s'], round(3 / 9.982, 3))             # 0.301
        self.assertEqual(w['motion_steady_reselect_nowrap_per_s'], 0.301)

    def test_reselection_run(self):
        # steady = Run_Loops 18.516 -> 21.952 (first line after the Run_Start) = 3.436 s. ct 2.312 -> 0.285 at 20.067 with a
        # feed dt of 0.517 s is a natural loop wrap (0.285 <= 0.517*1.8+0.02): 1 wrap, 0 jumps, 0 switches.
        r = self.runm
        self.assertEqual((r['motion_steady_s'], r['motion_steady_switches'], r['motion_steady_ct_wraps'], r['motion_steady_ct_jumps']),
                         (3.436, 0, 1, 0))
        self.assertEqual(r['motion_steady_reselect_per_s'], round(1 / 3.436, 3))             # 0.291
        self.assertEqual(r['motion_steady_reselect_nowrap_per_s'], 0.0)
        self.assertEqual((r['motion_db_walk_loops_fraction'], r['motion_walk_nonloop_fraction']), (0.0, 0.0))

    def test_clip_switch_in_steady_counts(self):
        # Walk_Loops -> Run_Loops while moving is a switch between two steady lines (the two real lines are paired
        # directly; they are 1.5 minutes apart in the log, only the (db, clip) change matters)
        run_first = next(l for l in self.lines if l.kind == 'MOTION' and 'Run_Loops' in l.raw)
        walk_last = [l for l in self.lines if l.kind == 'MOTION' and 'Walk_Loops' in l.raw][-1]
        m = {}
        gt_log._gait_motion(m, [walk_last, run_first], walk_last.ts, run_first.ts + timedelta(seconds=0.5))
        self.assertEqual((m['motion_steady_switches'], m['motion_steady_ct_wraps'], m['motion_steady_ct_jumps']), (1, 0, 0))

    def test_base_log_and_no_steady_time(self):
        v1 = [l for l in self.lines if l.kind != 'MOTION']
        m = gt_log.compute_metrics(v1, self.W0, self.W1)
        self.assertEqual(m['mm_driver'], 'v1')
        for k in gt_log.GAIT_MOTION_KEYS:
            self.assertIsNone(m[k], k)
        self.assertEqual(m['speed_by_mv']['1.00']['n'], 12)                              # tick metrics work for any driver
        self.assertEqual(m['root_off_max_cm'], 18.5)
        # idle window: no steady time -> rates None, fractions still defined
        m = gt_log.compute_metrics(self.lines, datetime(2026, 10, 31, 10, 49, 13, 0), datetime(2026, 10, 31, 10, 49, 15, 0))
        self.assertEqual((m['motion_steady_s'], m['motion_steady_reselect_per_s'], m['motion_steady_reselect_nowrap_per_s']),
                         (0.0, None, None))
        self.assertEqual(m['motion_walk_loop_share'], None)
        self.assertEqual(m['speed_by_mv'], {})

    def test_default_thresholds_unchanged(self):
        # report-only: no threshold on the new metrics, selection_unscored logic unchanged
        for k in list(self.walk):
            if k.startswith(('speed_by_mv', 'root_off', 'motion_steady', 'motion_walk', 'motion_db_walk', 'motion_clip_transition')):
                self.assertNotIn(k, gt_log.DEFAULT_THRESHOLDS)
        self.assertEqual(gt_log.evaluate(self.walk, gt_log.DEFAULT_THRESHOLDS), [])

    def test_step_outputs_carry_metrics(self):
        plan = {'runs': [{'scenario': 'walk', 'run': 1, 'aborted': None, 'steps': [
            {'name': 'walk', 'start': self.W0, 'end': self.W1, 'metrics': True}]}]}
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        rows = gt_run.analyze(plan, self.lines, gt_log.DEFAULT_THRESHOLDS, scen)
        with tempfile.TemporaryDirectory() as d:
            summary = gt_run.write_outputs(d, plan, rows, self.lines, GAIT_EXCERPT)
            with open(os.path.join(d, 'walk_r1.txt'), encoding='utf-8') as f:
                txt = f.read()
            with open(os.path.join(d, 'summary.csv'), encoding='utf-8') as f:
                header = f.readline().strip().split(',')
        self.assertEqual(summary['steps'][0]['metrics']['speed_by_mv']['1.00']['median'], 2.725)
        self.assertIn('speed_by_mv: 1.00: n=12 med=2.725 p90=2.73', txt)
        self.assertIn('motion_steady_reselect_per_s=0.301', txt)
        self.assertIn('root_off_max_cm', header)


# ---------------------------------------------------------------------------------------------- guard

class GuardTests(unittest.TestCase):
    def test_ok_when_game_foreground(self):
        g = make_guard()
        g.check()
        self.assertTrue(g.foreground_is_game())

    def test_other_app_foreground_aborts(self):
        api = FakeApi()
        g = make_guard(api)
        api.fg = OTHER_HWND
        with self.assertRaises(GuardError):
            g.check()
        api.fg = 0
        with self.assertRaises(GuardError):
            g.check()

    def test_process_exit_aborts(self):
        api = FakeApi()
        g = make_guard(api)
        api.alive = False
        with self.assertRaises(GuardError):
            g.check()

    def test_user_abort_key(self):
        api = FakeApi()
        g = make_guard(api)
        api.keys_down.add(0x1B)
        with self.assertRaises(GuardError):
            g.check()

    def test_ensure_focus(self):
        api = FakeApi()
        api.fg = OTHER_HWND
        g = make_guard(api)
        g.ensure_focus()
        self.assertEqual(api.calls[0], ('focus', GAME_HWND))
        self.assertEqual(api.fg, GAME_HWND)

    def test_ensure_focus_fails_without_sending(self):
        api = FakeApi()
        api.fg = OTHER_HWND
        api.focus_makes_foreground = False
        g = make_guard(api)
        with self.assertRaises(GuardError):
            g.ensure_focus()
        self.assertTrue(all(c[0] == 'focus' for c in api.calls))

    def test_game_not_running(self):
        api = FakeApi()
        api.running = False
        g = ForegroundGuard(api=api)
        with self.assertRaises(GuardError):
            g.ensure_focus()
        self.assertFalse(g.status()['game_running'])

    def test_name_spoof_by_other_pid_rejected(self):
        api = FakeApi()
        g = make_guard(api)
        api.process_name = lambda pid: 'Cyberpunk2077.exe'   # another process with the same image name
        api.fg = OTHER_HWND
        with self.assertRaises(GuardError):
            g.check()                                        # pid not in the game pid list


# ---------------------------------------------------------------------------------------------- drivers

class DriverTests(unittest.TestCase):
    def test_pad_apply_checks_guard_first(self):
        api = FakeApi()
        g = make_guard(api)
        pad = FakePad()
        drv = gt_input.PadDriver(g, pad=pad)
        fr = gs.Frame(0, (0.5, 0.5), (0, 0), 0.2, 0, gs.BUTTONS['A'] | gs.BUTTONS['LS'])
        drv.apply(fr)
        self.assertEqual(pad.reports[-1]['ls'], (0.5, 0.5))
        self.assertEqual(pad.reports[-1]['buttons'], 0x1040)
        api.fg = OTHER_HWND
        with self.assertRaises(GuardError):
            drv.apply(fr)
        self.assertEqual(len(pad.reports), 1)        # nothing sent after focus loss
        drv.release()
        self.assertEqual(pad.reports[-1]['ls'], (0, 0))

    def test_run_scenario_aborts_and_releases(self):
        api = FakeApi()
        g = make_guard(api)
        pad = FakePad()
        drv = gt_input.PadDriver(g, pad=pad)
        sc = gs.parse_scenario('t', {'rate_hz': 200, 'steps': [
            {'name': 'a', 'duration': 0.05, 'ls': [[0, 0, 1]]}, {'name': 'b', 'duration': 0.05}]})
        n = [0]
        orig = g.check

        def flaky():
            n[0] += 1
            if n[0] > 5:
                api.fg = OTHER_HWND
            orig()
        g.check = flaky
        res = gt_run.run_scenario(sc, drv, g)
        self.assertIn('foreground', res['aborted'])
        self.assertEqual(len([r for r in pad.reports if r['ls'] == (0.0, 1.0)]), 5)
        self.assertEqual(pad.reports[-1]['ls'], (0, 0))   # released
        self.assertIsNotNone(res['steps'][0]['start'])
        self.assertIsNotNone(res['steps'][0]['end'])

    def test_run_scenario_completes(self):
        g = make_guard()
        pad = FakePad()
        sc = gs.parse_scenario('t', {'rate_hz': 100, 'steps': [
            {'name': 'a', 'duration': 0.05, 'ls': [[0, 0, 0], [0.05, 0, 1]]}, {'name': 'b', 'duration': 0.05}]})
        res = gt_run.run_scenario(sc, gt_input.PadDriver(g, pad=pad), g)
        self.assertIsNone(res['aborted'])
        self.assertTrue(all(s['start'] and s['end'] and s['end'] >= s['start'] for s in res['steps']))
        self.assertEqual(pad.reports[-1]['ls'], (0, 0))

    def test_cet_key_decode(self):
        self.assertEqual(gt_input.decode_cet_key(54043195528445952), [0xC0])
        self.assertEqual(gt_input.decode_cet_key((0x11 << 48) | (0x43 << 32)), [0x11, 0x43])

    def _cet_dir(self, d):
        with open(os.path.join(d, 'layout.ini'), 'w', encoding='utf-8') as f:
            f.write('[Window][Cyber Engine Tweaks]\nPos=34,96\nSize=1977,64\nCollapsed=0\n\n'
                    '[Window][Console]\nPos=196,170\nSize=780,460\nCollapsed=0\n\n[Docking][Data]\n')
        with open(os.path.join(d, 'bindings.json'), 'w', encoding='utf-8') as f:
            json.dump({'cet': {'overlay_key': 54043195528445952}}, f)
        with open(os.path.join(d, 'cyber_engine_tweaks.log'), 'w', encoding='utf-8') as f:
            f.write('old\n')

    def test_cet_type_command_guarded(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0)
            self.assertEqual(ov.overlay_vks, [0xC0])
            self.assertEqual(ov.toggle_scans, [0x29])                # grave as hardware scan code
            self.assertEqual(ov.console_input_point(), (586.0, 612))
            api.client = (768, 432, 768 + 1536, 432 + 864)           # working window
            self.assertEqual(ov.console_click_screen(), (1229, 920))
            res = ov.run_commands(['Game.X(1)'])
            self.assertEqual(res, {'Game.X(1)': None})
            kinds = [c[0] for c in api.calls]
            self.assertEqual(api.calls[0], ('keys', (0x29,), GAME_HWND))   # overlay open
            self.assertIn(('cursor', 1229, 920), api.calls)
            self.assertIn('click', kinds)
            self.assertEqual(''.join(c[1] for c in api.calls if c[0] == 'char'), 'Game.X(1)')
            self.assertIn(('keys', (0x1C,), GAME_HWND), api.calls)          # Enter
            self.assertEqual(api.calls[-1], ('keys', (0x29,), GAME_HWND))   # overlay closed
            self.assertTrue(all(c[-1] == GAME_HWND for c in api.calls if c[0] in ('keys', 'char', 'click')))

    def test_cet_layout_click_mode(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0, input_rel=None,
                                     toggle_scans=[0x29])
            ov.run_commands(['x'])
            self.assertIn(('cursor', 586, 612), api.calls)

    def _utpp_log(self, d, text=''):
        p = os.path.join(d, 'utpp-x.log')
        with open(p, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        return p

    @staticmethod
    def _append(p, line):
        with open(p, 'a', encoding='utf-8', newline='\n') as f:
            f.write(line + '\n')

    def test_cet_commands_verified_in_utpp_log(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            typed = []
            api.send_unicode = lambda ch: typed.append(ch)

            def on_scan(scans):
                if scans == (0x1C,):
                    cmd = ''.join(typed)
                    typed.clear()
                    if cmd == 'Game.UTPPSetMMEnabled(true)':
                        self._append(log, '[2026-10-07 10:00:01.000] [info    ] [ 1] [UTPP] UTPPSetMMEnabled(1)')
            api.on_scan.append(on_scan)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0)
            res = ov.run_commands(['Game.UTPPSetMMEnabled(true)', 'Game.UTPPSetMMTrace(true)', 'print(1)'],
                                  log_path=log, verify_timeout=0.2, retries=1)
            self.assertEqual(res, {'Game.UTPPSetMMEnabled(true)': True, 'Game.UTPPSetMMTrace(true)': False,
                                   'print(1)': None})
            enters = [c for c in api.calls if c[0] == 'keys' and c[1] == (0x1C,)]
            self.assertEqual(len(enters), 4)          # 3 commands + 1 retry of the unconfirmed one
            self.assertFalse(ov.is_open)

    def test_cet_utpplogline_verified_by_its_message_across_a_log_rotation(self):
        # 2026-11-01: Game.UTPPLogLine('<msg>') writes 'UTPP CET <msg>' (no '<Name>(' echo): it was always 'NOT confirmed'.
        # The log may also rotate (utpp-x.log -> utpp-x.1.log, new utpp-x.log) between the typed command and the check.
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            typed = []
            api.send_unicode = lambda ch: typed.append(ch)
            msg = '[car] NOT MOUNTED case=a scenario=b after 2 X presses'
            cmd = "Game.UTPPLogLine('" + msg + "')"

            def on_scan(scans):
                if scans == (0x1C,):
                    c = ''.join(typed)
                    typed.clear()
                    if c == cmd:
                        os.replace(log, os.path.join(d, 'utpp-x.1.log'))
                        self._utpp_log(d, '')
                        self._append(log, '[2026-10-07 10:00:01.000] [info    ] [ 1] [UTPP] UTPP CET ' + msg)
            api.on_scan.append(on_scan)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0)
            res = ov.run_commands([cmd, "Game.UTPPLogLine('[car] other')"], log_path=log, verify_timeout=0.2, retries=0)
            self.assertEqual(res, {cmd: True, "Game.UTPPLogLine('[car] other')": False})

    # ------------------------------------------------------------------ 2026-11-09: reliable CET typing (cetv marker, pause, retry)

    def test_cetv_marker_detection(self):
        cmd = ("local ok,e=pcall(function() return 1 end) Game.UTPPLogLine('[cetv:ab12cd34ef][melee] FISTS ok='..tostring(ok))")
        self.assertEqual(gt_input.cetv_marker(cmd), 'ab12cd34ef')
        self.assertEqual(gt_input.cetv_marker('Game.UTPPLogLine("[cetv:ab12-cd_34.ef]x")'), 'ab12-cd_34.ef')
        # only the explicit tag counts: a plain / conditional UTPPLogLine and natives are not markers
        self.assertIsNone(gt_input.cetv_marker("Game.UTPPLogLine('[car] CMD home case=x')"))
        self.assertIsNone(gt_input.cetv_marker("if a then Game.UTPPLogLine('[car] HOME set') end"))
        self.assertIsNone(gt_input.cetv_marker('Game.UTPPSetMMTrace(true)'))
        self.assertIsNone(gt_input.cetv_marker("Game.UTPPLogLine('[cetv:ab]x')"))       # id too short
        line = '[2026-11-09 16:27:13.000] [info    ] [ 1] [UTPP] UTPP CET [cetv:ab12cd34ef][melee] FISTS ok=true quickslots'
        self.assertTrue(gt_input.cetv_seen(line, 'ab12cd34ef'))
        self.assertFalse(gt_input.cetv_seen(line, 'ab12cd34e'[:-1] + 'X'))
        self.assertFalse(gt_input.cetv_seen('UTPP CET [melee] FISTS ok=true', 'ab12cd34ef'))

    def test_cet_enter_pause_scales_with_length_and_is_applied_before_enter(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0, click_console=False)
            self.assertEqual(ov.enter_pause('x' * 5), ov.enter_pause_min)               # short: the minimum
            self.assertAlmostEqual(ov.enter_pause('x' * 583), 0.583)                     # the command that lost 7 characters
            self.assertEqual(ov.enter_pause('x' * 100000), ov.enter_pause_max)           # capped
            self.assertGreater(ov.enter_pause('x' * 1200), ov.enter_pause('x' * 600))
            events = []
            orig_wait = g.wait
            g.wait = lambda s, step=0.05: (events.append(('wait', round(s, 3))), orig_wait(s, step))[1]
            api.on_scan.append(lambda s: events.append(('scan', s)))
            cmd = 'y' * 583
            ov.type_command(cmd)
            i = events.index(('scan', (0x1C,)))
            self.assertEqual(events[i - 1], ('wait', 0.583))          # the pause is the last thing before Enter
            self.assertEqual(''.join(c[1] for c in api.calls if c[0] == 'char'), cmd)

    def _typing_overlay(self, d, api, g, log, behave, auto=False):
        """Overlay whose typed commands are answered by behave(cmd, attempt) -> list of UTPP log lines to append."""
        self._cet_dir(d)
        typed, count = [], {}
        api.send_unicode = lambda ch: typed.append(ch)

        def on_scan(scans):
            if scans == (0x1C,):
                cmd = ''.join(typed)
                typed.clear()
                count[cmd] = count.get(cmd, 0) + 1
                for ln in behave(cmd, count[cmd]):
                    self._append(log, '[2026-11-09 16:27:13.000] [info    ] [ 1] [UTPP] ' + ln)
        api.on_scan.append(on_scan)
        return gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0, auto_marker=auto)

    def test_cet_marker_command_confirmed_without_retry(self):
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            cmd = "local ok=1 Game.UTPPLogLine('[cetv:aaaa1111bb][melee] FISTS ok='..tostring(ok))"
            ov = self._typing_overlay(d, api, g, log, lambda c, n: ['UTPP CET [cetv:aaaa1111bb][melee] FISTS ok=true'])
            res = ov.run_commands([cmd], log_path=log, verify_timeout=0.2, retries=1)
            self.assertEqual(res, {cmd: True})
            self.assertEqual(ov.last_attempts, {cmd: 1})

    def test_cet_lost_command_is_retyped_once_then_confirmed(self):
        # run melee-cases-20261109-161452: a command arrived cut short ('syntax error') or not at all and nobody noticed
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            cmd = "local ok=1 Game.UTPPLogLine('[cetv:bbbb2222cc][melee] GUARD_SET ok='..tostring(ok))"
            ov = self._typing_overlay(d, api, g, log,
                                      lambda c, n: [] if n == 1 else ['UTPP CET [cetv:bbbb2222cc][melee] GUARD_SET ok=true'])
            res = ov.run_commands([cmd], log_path=log, verify_timeout=0.2, retries=1)
            self.assertEqual(res, {cmd: True})
            self.assertEqual(ov.last_attempts, {cmd: 2})
            # only the toggle (grave) and Enter scan codes are sent: no extra keys that could reach the game on a retry
            self.assertEqual({c[1] for c in api.calls if c[0] == 'keys'}, {(0x29,), (0x1C,)})
            enters = [c for c in api.calls if c[0] == 'keys' and c[1] == (0x1C,)]
            self.assertEqual(len(enters), 2)
            self.assertFalse(ov.is_open)

    def test_cet_command_lost_twice_is_reported_false_and_only_that_one_is_retyped(self):
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            good = "Game.UTPPLogLine('[cetv:cccc3333dd][melee] start mounted='..tostring(1))"
            bad = "Game.UTPPLogLine('[cetv:dddd4444ee][melee] GUN_PRIME ok='..tostring(1))"
            ov = self._typing_overlay(d, api, g, log,
                                      lambda c, n: ['UTPP CET [cetv:cccc3333dd][melee] start mounted=false'] if c == good else [])
            res = ov.run_commands([good, bad], log_path=log, verify_timeout=0.2, retries=1)
            self.assertEqual(res, {good: True, bad: False})
            self.assertEqual(ov.last_attempts, {good: 1, bad: 2})
            # a marker seen only BEFORE the typing (older run) must not confirm a new command with another id
            self.assertFalse(gt_input.cetv_seen('UTPP CET [cetv:cccc3333dd]', 'dddd4444ee'))

    def test_cet_auto_marker_makes_every_other_command_verifiable(self):
        # --cet default: commands that no check recognised ('local ... end', print(...)) get a unique [cetv:<id>] appended
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            seen_wire = []

            def behave(wire, n):
                seen_wire.append(wire)
                m = gt_input._CETV_RE.search(wire)
                if wire.startswith('lost') or not m:
                    return []
                return ['UTPP CET [cetv:' + m.group('id') + ']']
            ov = self._typing_overlay(d, api, g, log, behave, auto=True)
            plain = "local ok=pcall(function() return 1 end)"
            lost = "lost = 1"
            native = 'Game.UTPPSetMMTrace(true)'
            comment = 'print(1) -- trailing comment would swallow an appended call'
            leading = "Game.UTPPLogLine('[car] x')"
            res = ov.run_commands([plain, lost, native, comment, leading], log_path=log, verify_timeout=0.2, retries=1)
            self.assertIs(res[plain], True)
            self.assertIs(res[lost], False)
            self.assertIsNone(res[comment])                  # not auto-markable: reported as not verifiable
            self.assertIs(res[leading], False)               # old check (leading literal) unchanged: no echo in this fake log
            self.assertIs(res[native], False)
            self.assertEqual(ov.last_attempts[plain], 1)
            self.assertEqual(ov.last_attempts[lost], 2)
            marked = [w for w in seen_wire if gt_input.cetv_marker(w)]
            self.assertTrue(all(w.startswith(('local ok=pcall', 'lost')) for w in marked))
            self.assertNotIn(native + " Game.UTPPLogLine", ''.join(seen_wire))      # natives keep their own echo check
            self.assertFalse(any(w.startswith('print(1) --') and 'cetv' in w for w in seen_wire))
            lost_wires = [w for w in seen_wire if w.startswith('lost')]
            self.assertEqual(len(lost_wires), 2)
            self.assertEqual(lost_wires[0], lost_wires[1])   # same id on the retry
            # off by default (CetOverlay): old behaviour, the plain command stays 'not verifiable'
            omotion = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0)
            self.assertEqual(omotion.run_commands(['print(1)'], log_path=log, verify_timeout=0.1), {'print(1)': None})

    def test_cet_marker_command_without_log_is_not_verifiable(self):
        with tempfile.TemporaryDirectory() as d:
            api = FakeApi()
            g = make_guard(api)
            self._cet_dir(d)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0)
            cmd = "Game.UTPPLogLine('[cetv:eeee5555ff]x')"
            self.assertEqual(ov.run_commands([cmd], log_path=None), {cmd: None})
            self.assertEqual(ov.last_attempts, {cmd: 1})

    def test_live_run_aborts_with_a_readable_reason_when_a_cet_command_is_lost(self):
        class FakeOverlay:
            last_attempts: dict = {}

            def run_commands(self, cmds, log_path=None):
                FakeOverlay.last_attempts = {c: 2 for c in cmds}
                return {c: False for c in cmds}

        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            out = os.path.join(d, 'out')
            args = gt_run.build_parser().parse_args(['--cet', "Game.UTPPLogLine('[cetv:ffff6666aa][melee] FISTS ok='..1)",
                                                      '--out', out, '--log', log])
            saved = (gt_run.ForegroundGuard, gt_run.make_overlay)
            gt_run.ForegroundGuard = lambda focus_grace=0.0: make_guard(FakeApi())
            gt_run.make_overlay = lambda a, g: FakeOverlay()
            try:
                import io
                from contextlib import redirect_stderr, redirect_stdout
                so, se = io.StringIO(), io.StringIO()
                with redirect_stdout(so), redirect_stderr(se):
                    rc = gt_run.live_run(args, [])
            finally:
                gt_run.ForegroundGuard, gt_run.make_overlay = saved
            self.assertEqual(rc, 2)                                           # exit 2 = the caller (ps1) throws
            self.assertIn('NOT confirmed (typed 2x)', so.getvalue())
            self.assertIn('ABORT', se.getvalue())
            self.assertIn('not confirmed in the UTPP log after 2 attempt(s)', se.getvalue())
            self.assertIn('[cetv:ffff6666aa]', se.getvalue())
            with open(os.path.join(out, 'plan.json'), encoding='utf-8') as f:
                plan = json.load(f)
            self.assertIn('not confirmed', plan['error'])

    def test_native_echo_helpers(self):
        self.assertEqual(gt_input.native_echo_name('Game.UTPPSetMMTrace(true)'), 'UTPPSetMMTrace')
        self.assertIsNone(gt_input.native_echo_name('print(Game.UTPPGetStatus())'))
        line = '[2026-10-07 00:49:50.744] [info    ] [ 31544] [UTPP] UTPPSetMMTrace(1)'
        self.assertTrue(gt_input.echo_seen(line, 'UTPPSetMMTrace'))
        self.assertFalse(gt_input.echo_seen(line, 'UTPPSetMMEnabled'))
        # REDScript-side switches echo through UTPPLogLine ('UTPP CET <Name>(...)', seen 2026-10-24 on the 2.5 build)
        cet_line = '[2026-10-24 13:32:09.694] [info    ] [ 29888] [UTPP] UTPP CET UTPPMotionSetReloadDiag(1)'
        self.assertTrue(gt_input.echo_seen(cet_line, 'UTPPMotionSetReloadDiag'))
        self.assertFalse(gt_input.echo_seen(cet_line, 'UTPPSetVehicleVaultLog'))

    def test_dismiss_dialog_verified_by_trace(self):
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            api.on_scan.append(lambda s: self._append(log, _trace('10:00:05.000')) if s == (0x1C,) else None)
            res = gt_input.dismiss_disconnect_dialog(g, log, appear_wait=0, verify_timeout=0.5)
            self.assertEqual(res, {'sent': 1, 'verified': True})
            self.assertEqual(api.calls, [('keys', (0x1C,), GAME_HWND)])

    def test_dismiss_dialog_unverifiable_and_failed(self):
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, '[2026-10-07 10:00:00.000] [info    ] [ 1] [UTPP] UTPP ready\n')
            g = make_guard(FakeApi())
            self.assertEqual(gt_input.dismiss_disconnect_dialog(g, log, appear_wait=0), {'sent': 1, 'verified': None})
            log2 = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            res = gt_input.dismiss_disconnect_dialog(g, log2, appear_wait=0, verify_timeout=0.1, attempts=2)
            self.assertEqual(res, {'sent': 2, 'verified': False})

    def test_dismiss_dialog_not_sent_to_other_app(self):
        with tempfile.TemporaryDirectory() as d:
            log = self._utpp_log(d, _trace('10:00:00.000') + '\n')
            api = FakeApi()
            g = make_guard(api)
            api.fg = OTHER_HWND
            with self.assertRaises(GuardError):
                gt_input.dismiss_disconnect_dialog(g, log, appear_wait=0)
            self.assertEqual(api.calls, [])

    def test_cet_typing_stops_on_focus_loss(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, key_delay=0, open_wait=0, click_console=False)
            orig = api.send_unicode

            def send(ch):
                orig(ch)
                if ch == 'm':
                    api.fg = OTHER_HWND
            api.send_unicode = send
            with self.assertRaises(GuardError):
                ov.run_commands(['Game.UTPP()'])
            sent = [c for c in api.calls if c[0] in ('keys', 'char')]
            self.assertTrue(all(c[-1] == GAME_HWND for c in sent))     # nothing reached the other app
            self.assertEqual(''.join(c[1] for c in sent if c[0] == 'char'), 'Gam')

    def test_cet_click_outside_client_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            api.client = (0, 0, 800, 600)
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, open_wait=0)
            pt = ov.reload_button_point()
            self.assertTrue(34 + 1977 - 1977 / 7 < pt[0] < 34 + 1977)
            with self.assertRaises(GuardError):
                ov.click(*pt)
            self.assertNotIn('click', [c[0] for c in api.calls])

    def test_cet_reload_confirmed_by_log(self):
        with tempfile.TemporaryDirectory() as d:
            self._cet_dir(d)
            api = FakeApi()
            g = make_guard(api)
            ov = gt_input.CetOverlay(g, cet_dir=d, open_wait=0)

            def click():
                api.calls.append(('click', api.fg))
                with open(os.path.join(d, 'cyber_engine_tweaks.log'), 'a', encoding='utf-8') as f:
                    f.write('[info] Reloaded all mods\n')
            api.send_left_click = click
            self.assertTrue(ov.reload_all_mods(timeout=1.0))
            self.assertFalse(ov.is_open)


# ---------------------------------------------------------------------------------------------- end to end

class AnalyzeTests(unittest.TestCase):
    def test_analyze_real_log_plan(self):
        if not os.path.isfile(LOG_041):
            self.skipTest('missing log')
        lines = gt_log.parse_file(LOG_041)
        t = datetime(2026, 10, 7, 0, 49, 58)
        plan = {'runs': [{'scenario': 'flick_180_run', 'run': 1, 'aborted': None, 'steps': [
            {'name': 'settle', 'start': t - timedelta(seconds=2), 'end': t, 'metrics': False},
            {'name': 'cruise', 'start': t, 'end': t + timedelta(seconds=10), 'metrics': True},
            {'name': 'stop', 'start': t + timedelta(seconds=10), 'end': t + timedelta(seconds=22), 'metrics': True},
        ]}]}
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        rows = gt_run.analyze(plan, lines, gt_log.DEFAULT_THRESHOLDS, scen)
        self.assertEqual([r['verdict'] for r in rows][0], 'SKIP')
        self.assertIn(rows[1]['verdict'], ('PASS', 'FAIL'))
        with tempfile.TemporaryDirectory() as d:
            summary = gt_run.write_outputs(d, plan, rows, lines, LOG_041)
            for n in ('flick_180_run_r1.txt', 'summary.csv', 'summary.json'):
                self.assertTrue(os.path.isfile(os.path.join(d, n)), n)
            with open(os.path.join(d, 'summary.json'), encoding='utf-8') as f:
                self.assertEqual(json.load(f)['overall'], summary['overall'])
            with open(os.path.join(d, 'flick_180_run_r1.txt'), encoding='utf-8') as f:
                txt = f.read()
            self.assertIn('TRANS clip=Run_Start_F_Lfoot', txt)
            # plan round trip
            with open(os.path.join(d, 'plan.json'), 'w', encoding='utf-8') as f:
                json.dump(gt_run.serialise_plan(plan), f)
            back = gt_run.load_plan(os.path.join(d, 'plan.json'))
            self.assertEqual(back['runs'][0]['steps'][1]['start'], t)

    def test_cli_dry_run_and_list(self):
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(gt_run.main(['--list']), 0)
        self.assertIn('flick_180_run', buf.getvalue())
        with self.assertRaises(SystemExit):
            gt_run.main(['--scenarios', 'nope', '--dry-run'])


class FixTests(unittest.TestCase):
    """Fixes after the first in-game runs (2026-10-07 01:31-01:36)."""

    def test_abort_message_names_foreground_app(self):
        api = FakeApi()
        g = make_guard(api)
        api.fg = OTHER_HWND
        with self.assertRaises(GuardError) as cm:
            g.check()
        self.assertIn('exe=notepad.exe', str(cm.exception))
        self.assertEqual(g.events[-1]['type'], 'focus_lost')
        api.fg = 0
        self.assertIn('hwnd=0', g.describe_foreground())

    def test_focus_grace_glitch_recovers_with_input_released(self):
        api = FakeApi()
        t = [0.0]

        def clock():
            t[0] += 0.01
            return t[0]
        g = ForegroundGuard(api=api, process_check_interval=1e9, clock=clock, focus_grace=0.3)
        g.refresh_game()
        released = []
        g.on_violation = lambda: released.append(api.fg)
        api.fg = OTHER_HWND
        api.on_sleep.append(lambda: setattr(api, 'fg', GAME_HWND))
        g.check()                                    # recovers, no exception
        self.assertEqual(released, [OTHER_HWND])
        self.assertEqual(g.events[-1]['type'], 'focus_glitch')

    def test_focus_grace_expires(self):
        api = FakeApi()
        t = [0.0]

        def clock():
            t[0] += 0.01
            return t[0]
        g = ForegroundGuard(api=api, process_check_interval=1e9, clock=clock, focus_grace=0.1)
        g.refresh_game()
        api.fg = OTHER_HWND
        with self.assertRaises(GuardError):
            g.check()

    def test_slow_thresholds(self):
        # thresholds.json: rate_min 0.4 is report only since 2026-10-23 (contradicts the intended slow-walk rate 0.25);
        # the '@slow' key mechanics are checked with an explicit 0.4 limit
        file_thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        self.assertIsNone(file_thr['rate_min'])
        self.assertEqual(file_thr, gt_log.DEFAULT_THRESHOLDS)          # built-in defaults match the file (review L7)
        m = {'rate_min': 0.25, 'samples': 5}
        self.assertEqual(gt_log.evaluate(m, file_thr, 'cruise', slow=False), [])
        self.assertEqual(gt_log.evaluate({'rate_min': 0.1}, file_thr, 'half_stick', slow=True), ['rate_min=0.1 < 0.2'])
        thr = gt_log.merge_thresholds(gt_log.DEFAULT_THRESHOLDS, {'rate_min': {'min': 0.4}})
        self.assertEqual(gt_log.evaluate(m, thr, 'half_stick', slow=True), [])
        self.assertEqual(gt_log.evaluate(m, thr, 'cruise', slow=False), ['rate_min=0.25 < 0.4'])
        thr2 = gt_log.merge_thresholds(thr, {'half_stick.rate_min@slow': {'min': 0.3}, 'cruise.rate_min': None})
        self.assertEqual(gt_log.evaluate(m, thr2, 'half_stick', slow=True), ['rate_min=0.25 < 0.3'])
        self.assertEqual(gt_log.evaluate(m, thr2, 'other', slow=True), [])
        self.assertEqual(gt_log.evaluate(m, thr2, 'cruise', slow=False), [])

    def test_step_stick_stats(self):
        sc = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        st = sc['slow_walk_half_stick'].step_stick_stats()
        self.assertEqual(st['half_stick'], (0.4, 0.55))
        self.assertEqual(st['ramp_in'][0], 0.0)
        arc = sc['run_arc_left'].step_stick_stats()
        self.assertEqual(arc['arc_90'], (1.0, 1.0))

    def test_rerun_analysis_of_first_ingame_run(self):
        """The 01:34:59 run (base log): rate 0.25 in slow steps does not fail (rate_min is report only since 2026-10-23)."""
        d = os.path.join(PROJECT, 'Research', 'logs', 'gametest', '20261007-013459')
        game_log = os.path.join(gt_log.DEFAULT_LOG_DIR, 'utpp-2026-10-07-01-30-58.log')
        if not (os.path.isfile(os.path.join(d, 'plan.json')) and os.path.isfile(game_log)):
            self.skipTest('first in-game run artefacts not available')
        plan = gt_run.load_plan(os.path.join(d, 'plan.json'))
        lines = gt_log.parse_file(game_log, plan.get('log_offset', 0))
        rows = gt_run.analyze(plan, lines, gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json')),
                              gs.load_scenarios(os.path.join(HERE, 'scenarios.json')))
        for r in rows:
            self.assertFalse(any(f.startswith('rate_min') for f in r['fails']), (r['step'], r['fails']))
        hs = next(r for r in rows if r['scenario'] == 'slow_walk_half_stick' and r['step'] == 'half_stick')
        self.assertEqual(hs['metrics']['stick_min'], 0.4)


class PaddTests(unittest.TestCase):
    def _server(self, api):
        import padd
        g = make_guard(api)
        pad = FakePad()
        return padd.PadServer(pad, g, watchdog=0.05, log=lambda m: None), pad

    def test_server_refreshes_stale_pid_before_new_session_input(self):
        api = FakeApi()
        srv, pad = self._server(api)
        srv.guard.game_pids = [GAME_PID + 1000]  # old game process, daemon still alive
        srv.guard._last_proc_check = -1e9
        frame = gt_input.frame_to_msg(gs.Frame(0, (0, 0), (0, 0), 1, 0, 0))
        self.assertEqual(srv.handle(frame), {'ok': True})  # first report must succeed
        self.assertEqual(srv.guard.game_pids, [GAME_PID])
        self.assertEqual(pad.reports[-1]['lt'], 1.0)
        srv.handle({'op': 'release'})
        srv.guard.game_pids = [GAME_PID + 1000]
        api.fg = OTHER_HWND
        result = srv.handle(frame)
        self.assertFalse(result['ok'])
        self.assertIn('foreground', result['error'])
        self.assertEqual(pad.reports[-1]['lt'], 0.0)  # refresh cannot bypass focus guard

    def test_server_guards_non_neutral_frames(self):
        api = FakeApi()
        srv, pad = self._server(api)
        fr = gt_input.frame_to_msg(gs.Frame(0, (0, 1), (0, 0), 0, 0, gs.BUTTONS['B']))
        self.assertEqual(srv.handle(fr), {'ok': True})
        self.assertEqual(pad.reports[-1]['ls'], (0.0, 1.0))
        self.assertEqual(pad.reports[-1]['buttons'], gs.BUTTONS['B'])
        api.fg = OTHER_HWND
        r = srv.handle(fr)
        self.assertFalse(r['ok'])
        self.assertIn('foreground', r['error'])
        self.assertEqual(pad.reports[-1]['ls'], (0.0, 0.0))        # released, not the requested frame
        self.assertEqual(srv.handle(gt_input.frame_to_msg(gt_input.NEUTRAL)), {'ok': True})  # neutral allowed
        self.assertFalse(srv.handle({'op': 'frame', 'ls': ['x', 0]})['ok'])

    def test_server_watchdog_and_quit(self):
        import time as _t
        api = FakeApi()
        srv, pad = self._server(api)
        srv.handle(gt_input.frame_to_msg(gs.Frame(0, (0.5, 0), (0, 0), 0, 0, 0)))
        _t.sleep(0.08)
        srv.tick_watchdog()
        self.assertEqual(pad.reports[-1]['ls'], (0.0, 0.0))
        self.assertTrue(srv.handle({'op': 'ping'})['ok'])
        self.assertEqual(srv.handle({'op': 'quit'}), {'ok': True})
        self.assertFalse(srv.running)

    def test_remote_driver_over_socket(self):
        import socket as _s
        import threading
        api = FakeApi()
        srv, pad = self._server(api)
        probe = _s.socket()
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
        probe.close()
        th = threading.Thread(target=srv.serve, args=('127.0.0.1', port), daemon=True)
        th.start()
        for _ in range(50):
            if gt_input.padd_ping(port=port):
                break
            _t_sleep(0.02)
        g = make_guard(api)
        drv = gt_input.RemotePadDriver(g, gt_input.PaddClient(port=port))
        self.assertTrue(drv.persistent)
        drv.apply(gs.Frame(0, (0.25, 0.5), (0, 0), 0, 0, 0))
        self.assertEqual(pad.reports[-1]['ls'], (0.25, 0.5))
        drv.close()                                   # disconnect only: daemon keeps the pad
        self.assertEqual(pad.reports[-1]['ls'], (0.0, 0.0))
        self.assertTrue(gt_input.padd_ping(port=port)['ok'])
        c = gt_input.PaddClient(port=port)
        c.call({'op': 'quit'})
        c.close()
        th.join(2)
        self.assertFalse(th.is_alive())


class PaddSpawnTests(unittest.TestCase):
    """spawn_padd must never open a window over the game (2026-11-10: DETACHED_PROCESS + venv launcher python.exe
    -> Windows Terminal window, focus lost, run aborted)."""

    def test_spawn_kwargs_hidden_no_detached(self):
        import subprocess
        kw = gt_run.padd_spawn_kwargs()
        flags = kw['creationflags']
        self.assertTrue(flags & 0x08000000)            # CREATE_NO_WINDOW
        self.assertTrue(flags & 0x00000200)            # CREATE_NEW_PROCESS_GROUP
        self.assertFalse(flags & 0x00000008)           # not DETACHED_PROCESS (console-less launcher child allocates a window)
        self.assertTrue(kw['startupinfo'].dwFlags & subprocess.STARTF_USESHOWWINDOW)
        self.assertEqual(kw['startupinfo'].wShowWindow, 0)  # SW_HIDE
        self.assertEqual(kw['stdin'], subprocess.DEVNULL)

    def test_spawn_padd_uses_hidden_kwargs_and_waits_for_ping(self):
        import subprocess
        from unittest import mock
        calls = []

        class FakeProc:
            pass

        def fake_popen(cmd, **kw):
            calls.append((cmd, kw))
            return FakeProc()

        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(gt_run, 'DEFAULT_OUT_ROOT', td), \
                mock.patch.object(subprocess, 'Popen', fake_popen), \
                mock.patch.object(gt_input, 'padd_ping', lambda port=0: {'ok': True}):
            st = gt_run.spawn_padd(49123, timeout=1.0)
        self.assertEqual(st, {'ok': True})
        (cmd, kw), = calls
        self.assertEqual(cmd[1:], [os.path.join(gt_run.HERE, 'padd.py'), '--port', '49123'])
        self.assertTrue(kw['creationflags'] & 0x08000000)
        self.assertFalse(kw['creationflags'] & 0x00000008)
        self.assertEqual(kw['startupinfo'].wShowWindow, 0)
        self.assertEqual(kw['cwd'], gt_run.HERE)


def _t_sleep(s):
    import time as _t
    _t.sleep(s)


class VideoTests(unittest.TestCase):
    def test_parse_duration(self):
        self.assertAlmostEqual(video_sheets.parse_duration('  Duration: 00:01:23.45, start: 0'), 83.45)
        self.assertIsNone(video_sheets.parse_duration('no info'))

    def test_annotation(self):
        lines = gt_log.parse_text('\n'.join([_trace('10:00:00.000'), _trace('10:00:00.600', 'TRANS',
                                                                                'M_Neutral_Run_Stop_F_Lfoot')]))
        ann = video_sheets.frame_annotation(lines, datetime(2026, 10, 7, 10, 0, 0, 500000), 0.25)
        self.assertTrue(ann[0].startswith('[-0.50s]'))
        self.assertTrue(any('Run_Stop_F_Lfoot' in a for a in ann))
        ann2 = video_sheets.frame_annotation(lines, datetime(2026, 10, 7, 9, 0, 0), 0.25)
        self.assertEqual(ann2, ['(no trace sample before this time)'])

    def test_motion_annotation_takes_priority_over_stale_base_clip(self):
        lines = gt_log.parse_text('\n'.join([
            _trace('10:00:00.000', 'TRANS', 'M_Neutral_Sprint_Stop_FL_Rfoot'),
            '[2026-10-07 10:00:00.500] [info] [1] [UTPP] UTPP MOTION t=1 db=22:Sprint clip=462:M_Neutral_Sprint_Loop_FR ov=1/1/00 lay=spine1.00,armL1.00 aimW=0.00',
        ]))
        ann = video_sheets.frame_annotation(lines, datetime(2026, 10, 7, 10, 0, 1), 0.25)
        self.assertIn('motion clip=462:M_Neutral_Sprint_Loop_FR', ann[0])
        self.assertTrue(any('base ' in line and 'Sprint_Stop_FL_Rfoot' in line for line in ann))
        stale = video_sheets.frame_annotation(lines, datetime(2026, 10, 7, 10, 0, 3), 0.25)
        self.assertNotIn('motion ', ' '.join(stale))


class FakeTestcmdModule:
    """Stands in for the utpp_testcmd CET module: tick() runs the cmd.txt lines above its last sequence and appends an
    ACK line per line to the log (`answer` decides: True / False = ok flag, None = no ACK)."""

    def __init__(self, cmd_path, log_path, answer=lambda seq, chunk: True):
        self.cmd_path, self.log_path, self.answer = cmd_path, log_path, answer
        self.last = 0
        self.ran = []

    def tick(self):
        if not os.path.isfile(self.cmd_path):
            return
        with open(self.cmd_path, encoding='utf-8') as f:
            entries = gt_input.parse_testcmd_lines(f.read())
        for seq, chunk in entries:
            if seq > self.last:
                self.last = seq
                self.ran.append((seq, chunk))
                ok = self.answer(seq, chunk)
                if ok is not None:
                    with open(self.log_path, 'a', encoding='utf-8') as f:
                        f.write(f'[2026-11-14 10:00:00.000] [info] [1] [UTPP] UTPP CET [testcmd] ACK seq={seq} '
                                f'ok={"true" if ok else "false"} r{seq}\n')


class FileChannelTests(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp()
        self.dir = os.path.join(self.root, 'mods', 'utpp_testcmd')
        os.makedirs(self.dir)
        self.cmd = os.path.join(self.dir, 'cmd.txt')
        self.log = os.path.join(self.root, 'test.log')
        open(self.log, 'w').close()
        self.t = [0.0]

    def read(self):
        with open(self.cmd, encoding='utf-8') as f:
            return f.read()

    def channel(self, module=None, now=lambda: 5000, **kw):
        def sleep(s):
            self.t[0] += s
            if module is not None:
                module.tick()
        return gt_input.FileCommandChannel(self.cmd, sleep=sleep, clock=lambda: self.t[0], now_ms=now, **kw)

    def test_path_guard(self):
        self.assertTrue(gt_input.testcmd_path_ok(self.cmd))
        self.assertTrue(gt_input.testcmd_path_ok(os.path.join(self.root, 'mods', 'x', '..', 'utpp_testcmd', 'cmd.txt')))
        for bad in (os.path.join(self.dir, 'other.txt'), os.path.join(self.dir, 'cmd.txt', '..', 'init.lua'),
                    os.path.join(self.root, 'mods', 'utpp_core', 'cmd.txt'), ''):
            self.assertFalse(gt_input.testcmd_path_ok(bad), bad)
        with self.assertRaises(ValueError):
            gt_input.FileCommandChannel(os.path.join(self.root, 'mods', 'utpp_core', 'init.lua'))
        self.assertTrue(gt_input.testcmd_path_ok(gt_input.TESTCMD_FILE))

    def test_sequence_numbers_rise_above_file_and_state(self):
        with open(os.path.join(self.dir, 'state.txt'), 'w') as f:
            f.write('7000\n')
        with open(self.cmd, 'w') as f:
            f.write('6500 print(1)\n')
        ch = self.channel(keep_lines=3)
        seqs = [ch.post(f'print({i})') for i in range(4)]
        self.assertEqual(seqs, [7001, 7002, 7003, 7004])               # clock (5000) is below the state: state + 1
        entries = gt_input.parse_testcmd_lines(self.read())
        self.assertEqual([s for s, _ in entries], [7002, 7003, 7004])  # last keep_lines lines only
        self.assertFalse(os.path.exists(self.cmd + '.tmp'))
        self.assertGreater(self.channel(now=lambda: 9000).post('x()'), 9000 - 1)
        for bad in ('a()\nb()', '  ', 'a()\r'):
            with self.assertRaises(ValueError):
                ch.post(bad)

    def test_ack_match_is_exact(self):
        mod = FakeTestcmdModule(self.cmd, self.log)
        ch = self.channel(mod)
        with open(self.log, 'a') as f:      # an older ACK whose number is a prefix of the new one must not count
            f.write('[x] UTPP CET [testcmd] ACK seq=500 ok=true old\n')
        r = ch.send('Game.UTPPLogLine("a")', self.log)
        self.assertEqual((r['seq'], r['ok'], r['result'], r['attempts']), (5000, True, 'r5000', 1))
        self.assertEqual(gt_input.testcmd_acks('[testcmd] ACK seq=12 ok=false boom'), {12: (False, 'boom')})
        self.assertEqual(list(gt_input.testcmd_acks('[testcmd] ACK seq=50001 ok=true x')), [50001])
        self.assertEqual(gt_input.testcmd_ready('a [testcmd] READY seq=3 loader=loadstring\n'
                                                'b [testcmd] READY seq=9 loader=none'), (9, 'none'))
        self.assertIsNone(gt_input.testcmd_ready('nothing'))

    def test_timeout_retry_rewrites_under_a_new_number(self):
        calls = {'n': 0}

        def answer(seq, chunk):
            calls['n'] += 1
            return None if calls['n'] == 1 else True     # the first line runs without an ACK (lost)
        mod = FakeTestcmdModule(self.cmd, self.log, answer)
        ch = self.channel(mod, ack_timeout=1.0)
        r = ch.send('spawn()', self.log)
        self.assertEqual((r['ok'], r['attempts'], len(r['seqs'])), (True, 2, 2))
        self.assertEqual([s for s, _ in gt_input.parse_testcmd_lines(self.read())], [r['seqs'][1]])

    def test_no_ack_and_error_ack(self):
        ch = self.channel(FakeTestcmdModule(self.cmd, self.log, lambda s, c: None), ack_timeout=0.5)
        res = ch.run_commands(['a()', 'b()'], log_path=self.log)
        self.assertEqual(res, {'a()': False, 'b()': False})
        self.assertEqual(ch.last_attempts, {'a()': 2, 'b()': 2})
        ch = self.channel(FakeTestcmdModule(self.cmd, self.log, lambda s, c: 'bad' not in c))
        self.assertEqual(ch.run_commands(['bad()', 'good()'], log_path=self.log), {'bad()': False, 'good()': True})
        self.assertEqual(self.channel().run_commands(['c()']), {'c()': None})        # no log: written, not verified

    def test_replace_retried_while_the_module_reads(self):
        fails = {'n': 2}

        def replace(a, b):
            if fails['n']:
                fails['n'] -= 1
                raise PermissionError('in use')
            os.replace(a, b)
        ch = self.channel(replace=replace)
        ch.post('a()')
        self.assertEqual(fails['n'], 0)
        self.assertIn(' a()', self.read())
        self.assertTrue(ch.clear())
        self.assertEqual(self.read(), '')
        os.remove(self.cmd)
        os.rmdir(self.dir)
        with self.assertRaises(GuardError):              # module not installed: nothing is created
            ch.post('a()')
        self.assertFalse(os.path.exists(self.dir))


class BackgroundGuardTests(unittest.TestCase):
    def test_no_foreground_check_keys_refused_abort_kept(self):
        api = FakeApi()
        api.fg = OTHER_HWND
        g = gt_input.BackgroundGuard(api=api)
        g.ensure_focus()                                   # does not focus the game
        g.check()
        g.wait(0.2)
        self.assertNotIn('focus', [c[0] for c in api.calls])
        for call in (lambda: g.api.send_unicode('a'), lambda: g.api.send_scan_chord([0x1C]),
                     lambda: g.api.send_left_click(), lambda: g.api._send([])):
            with self.assertRaises(gt_input.NeedsKeyboard):
                call()
        self.assertFalse(issubclass(gt_input.NeedsKeyboard, GuardError))
        self.assertEqual(api.calls, [])
        api.keys_down.add(0x1B)                            # ESC belongs to other applications in a background run
        g.check()
        api.keys_down.add(0x13)                            # Pause stops it
        with self.assertRaisesRegex(GuardError, 'abort key'):
            g.check()
        api.keys_down.clear()
        api.alive = False
        g._last_proc_check = -1e9
        with self.assertRaisesRegex(GuardError, 'not running'):
            g.check()


if __name__ == '__main__':
    unittest.main()


class BlockedStepTests(unittest.TestCase):
    """A walking step whose capsule never moved (V against an obstacle) is not scored (INVALID, mechanics failure)."""

    def test_blocked_walk(self):
        # Female V 2026-11-11 17:37:53 weapon_aim/walk_armed: stick 0.8, |v| <= 0.18 m/s (a bush)
        m = {'pushed_speed_max': 0.18, 'pushed_ticks': 3, 'loco_fraction': 1.0}
        self.assertTrue(gt_run.step_blocked(m))

    def test_free_and_other_steps_are_scored(self):
        base = {'pushed_speed_max': 0.18, 'pushed_ticks': 3, 'loco_fraction': 1.0}
        for change in ({'pushed_speed_max': 0.52},   # free aim walk 16:08:53 (slowest walking step)
                       {'pushed_speed_max': None, 'pushed_ticks': 0},   # stick never pushed (settle, stop)
                       {'pushed_ticks': 1},          # one tick cannot tell
                       {'loco_fraction': 0.0}):      # in a vehicle
            self.assertFalse(gt_run.step_blocked(dict(base, **change)), change)

    def test_pushed_speed_from_ticks(self):
        # the sprint_stop excerpt: the stick is released, so its 0.0 m/s ticks are not 'pushed'
        lines = gt_log.parse_file(MOTION_EXCERPT)
        m = gt_log.compute_metrics(lines, MotionLogTests.T0, MotionLogTests.T1)
        self.assertFalse(gt_run.step_blocked(m))

    def test_blocked_is_a_mechanics_failure(self):
        import gt_night as gn
        mech, metr = gn.split_failures([{'scenario': 'weapon_aim', 'run': 1, 'step': 'walk_armed',
                                         'fails': ['blocked: V did not move (speed_max 0.18 m/s < 0.3 with the stick at 0.8)']}], [])
        self.assertEqual((len(mech), len(metr)), (1, 0))
