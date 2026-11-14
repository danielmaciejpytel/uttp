"""Offline tests for the state scenarios of the night runner: jumps, slide, weapon fire and reload, the melee fast combo.

  python Source\\tools\\gametest\\test_gt_states.py -v
"""
from __future__ import annotations

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_melee_weapons as gw  # noqa: E402
import gt_night as gn  # noqa: E402
from gt_scenario import load_scenarios  # noqa: E402

SCEN = os.path.join(HERE, 'scenarios.json')
GROUND_PARTS = ('stand', 'land', 'run', 'sprint', 'stop', 'stand_check')


def pressed(sc, button):
    """[(scenario time, hold)] of the button events of a scenario."""
    out, t0 = [], 0.0
    for st in sc.steps:
        out += [(round(t0 + b.t, 3), b.hold, st.name) for b in st.buttons if b.button == button]
        t0 += st.duration
    return out


def taps(sc, channel):
    """Start times of the presses (value crossing 0.5 upwards) of the rt / lt channel."""
    out, prev = [], 0.0
    for _, f in sc.sample(1 / 120):
        v = getattr(f, channel)
        if v > 0.5 >= prev:
            out.append(round(f.t, 2))
        prev = v
    return out


class StateSetTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)

    def test_states_set_resolves_and_validates(self):
        names = gn.resolve_sets('states')
        self.assertEqual(names, ['jump_running', 'jump_in_place', 'slide_sprint', 'weapon_fire_reload'])
        self.assertEqual(gn.validate_sets(names, self.sc), [])
        self.assertEqual(gn.MACROS['weapon_fire_reload'], gn.WEAPON_FIRE_PARTS + gn.RECOVERY_PARTS)
        self.assertEqual(gn.WEAPON_FIRE_PARTS, ['weapon_draw', 'weapon_fire', 'weapon_reload', 'weapon_holster', 'weapon_walk'])
        self.assertNotIn('weapon_fire_reload', self.sc)                  # a night.py macro
        self.assertNotIn('states', gn.resolve_sets('all'))               # opt-in only
        missing = {k: v for k, v in self.sc.items() if k != 'weapon_reload'}
        self.assertIn('weapon_reload', gn.validate_sets(['weapon_fire_reload'], missing)[0])

    def test_no_state_scenario_needs_the_keyboard(self):
        # --background: pad only. A pad R3 step that also drives the stick is not a crouch toggle (which the keyboard crouch
        # mode would replace by a key press, or skip with --background)
        for name in gn.STATE_SCENARIOS + gn.WEAPON_FIRE_PARTS:
            self.assertEqual(gn.crouch_toggle_steps(self.sc[name]), [], name)
        slide = self.sc['slide_sprint']
        plain = [st for st in slide.steps if st.name.startswith('pad_')]
        self.assertEqual(len(plain), 2)
        self.assertTrue(all(st.buttons and not st.metrics for st in plain))       # without the prefix both would be toggle steps
        renamed = [type(st)(st.name[4:], st.duration, st.tracks, st.buttons, st.metrics) for st in slide.steps]
        self.assertEqual(len(gn.crouch_toggle_steps(type(slide)('x', '', renamed))), 2)
        self.assertEqual([self.sc['crouch_walk_turns'].steps[i].name for i in gn.crouch_toggle_steps(self.sc['crouch_walk_turns'])],
                         ['crouch', 'uncrouch'])


class JumpTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)

    def air_steps_are_not_scored(self, sc):
        scored = [s.name for s in sc.steps if s.metrics]
        for s in sc.steps:
            if s.name == 'jump':
                self.assertFalse(s.metrics)
        return scored

    def test_jump_in_place(self):
        sc = self.sc['jump_in_place']
        self.assertEqual([s.name for s in sc.steps], ['settle', 'stand', 'jump', 'land'])
        self.assertEqual(self.air_steps_are_not_scored(sc), ['stand', 'land'])
        (t, hold, step), = pressed(sc, 'A')
        self.assertEqual(step, 'jump')
        self.assertLessEqual(hold, 0.15)                              # a tap: a held A charges a leg cyberware jump
        self.assertTrue(all(f.ls == (0.0, 0.0) for _, f in sc.sample(1 / 30)))   # the stick is never touched
        self.assertEqual(gn.stick_windows(sc), [])

    def test_jump_running(self):
        sc = self.sc['jump_running']
        self.assertEqual([s.name for s in sc.steps], ['settle', 'run', 'jump', 'land', 'stop'])
        self.assertEqual(self.air_steps_are_not_scored(sc), ['run', 'land', 'stop'])
        (t, hold, step), = pressed(sc, 'A')
        self.assertEqual(step, 'jump')
        self.assertLessEqual(hold, 0.15)
        self.assertEqual(pressed(sc, 'LS'), [])                       # a run, not a sprint
        bounds = {n: (a, b) for n, a, b in sc.step_bounds()}
        frames = {i: [] for i in range(len(sc.steps))}
        for i, f in sc.sample(1 / 60):
            frames[i].append(f)
        # the stick is at full tilt from the end of the ramp until the stop step: the jump and the landing happen running
        for name in ('jump', 'land'):
            i = [s.name for s in sc.steps].index(name)
            self.assertTrue(all((f.ls[0] ** 2 + f.ls[1] ** 2) ** 0.5 > 0.99 for f in frames[i]), name)
        self.assertEqual(len(gn.stick_windows(sc)), 1)                # one walking window: open ground is cleared first
        self.assertGreater(bounds['stop'][0], bounds['jump'][1])


class SlideTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)['slide_sprint']

    def test_steps_and_buttons(self):
        self.assertEqual([s.name for s in self.sc.steps],
                         ['settle', 'sprint', 'pad_slide', 'crouched_idle', 'pad_stand_up', 'stand_check'])
        self.assertEqual([(n, h) for _, h, n in pressed(self.sc, 'LS')], [('sprint', 0.12)])          # L3 tap = sprint
        self.assertEqual([(n, h) for _, h, n in pressed(self.sc, 'R3')], [('pad_slide', 0.08), ('pad_stand_up', 0.08)])
        self.assertEqual(self.sc.thresholds['stand_check.stand_fraction']['min'], 0.8)

    def test_slide_is_pressed_while_sprinting_with_the_stick_held(self):
        bounds = {n: (a, b) for n, a, b in self.sc.step_bounds()}
        (t, _, _), _ = pressed(self.sc, 'R3')
        slide_frames = [f for i, f in self.sc.sample(1 / 60) if self.sc.steps[i].name == 'pad_slide']
        self.assertTrue(all((f.ls[0] ** 2 + f.ls[1] ** 2) ** 0.5 > 0.99 for f in slide_frames))
        sprint_len = bounds['sprint'][1] - bounds['sprint'][0]
        self.assertGreaterEqual(sprint_len, 2.5)                        # the slide needs >= 4.5 m/s
        self.assertFalse(self.sc.steps[2].metrics)
        self.assertFalse(self.sc.steps[3].metrics)
        self.assertTrue(self.sc.steps[1].metrics and self.sc.steps[5].metrics)

    def test_stick_released_before_the_stand_up_tap(self):
        i = [s.name for s in self.sc.steps].index('pad_stand_up')
        self.assertTrue(all(f.ls == (0.0, 0.0) for k, f in self.sc.sample(1 / 60) if k == i))


class WeaponFireTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)

    def test_fire_part(self):
        sc = self.sc['weapon_fire']
        shots = taps(sc, 'rt')
        self.assertEqual(len(shots), 3)
        self.assertTrue(all(b - a >= 1.0 for a, b in zip(shots, shots[1:])))   # single shots, well apart
        aim_on = taps(sc, 'lt')
        self.assertEqual(len(aim_on), 1)
        self.assertLess(aim_on[0], shots[0] - 0.5)                    # aimed before the first shot
        lt_after = [f.lt for _, f in sc.sample(1 / 60) if f.t > shots[-1] + 0.2 and f.lt > 0.5]
        self.assertGreater(len(lt_after), 10)                          # still aiming after the last shot
        self.assertEqual(sc.sample(1 / 60).__next__()[1].ls, (0.0, 0.0))
        last = list(sc.sample(1 / 60))[-1][1]
        self.assertEqual((last.lt, last.rt), (0.0, 0.0))              # everything released at the end
        self.assertEqual(gn.stick_windows(sc), [])
        self.assertTrue(all(s.metrics for s in sc.steps))

    def test_reload_part(self):
        sc = self.sc['weapon_reload']
        (t, hold, _), = pressed(sc, 'X')
        self.assertLessEqual(hold, 0.2)
        self.assertGreaterEqual(sc.duration - t, 6.0)                 # a slow reload has ended before the next part
        self.assertTrue(all(not s.metrics for s in sc.steps))
        self.assertEqual(max(f.rt for _, f in sc.sample(1 / 60)), 0.0)  # no shot while the reload runs

    def test_reload_episode_reader(self):
        text = ('[x] UTPP UTPPMotion RELOAD episode t0=10.00 t1=12.60 dur=2.60 rec=Items.X state=1 refill=0 mag=3->30\n'
                '[x] UTPP UTPPMotion RELOAD refill-no-state t=3.00 rec=Items.X\n'
                '[x] UTPP UTPPMotion RELOAD episode t0=20.00 t1=23.10 dur=3.10 rec=Items.X state=1\n')
        self.assertEqual(gn.reload_episodes(text), [2.6, 3.1])
        self.assertEqual(gn.reload_episodes('nothing here'), [])


class FastComboTests(unittest.TestCase):
    def setUp(self):
        self.sc = load_scenarios(SCEN)

    def test_cadences(self):
        slow, fast = taps(self.sc['mw_light_combo'], 'rt'), taps(self.sc['mw_light_combo_fast'], 'rt')
        self.assertEqual([round(b - a, 2) for a, b in zip(slow, slow[1:])], [0.65] * 4)
        gaps = [round(b - a, 2) for a, b in zip(fast, fast[1:])]
        self.assertTrue(gaps and all(0.25 <= g <= 0.35 for g in gaps), gaps)     # early enough to be buffered
        self.assertGreaterEqual(len(fast), 5)                                    # enough clicks to reach the last strike
        self.assertEqual(self.sc['mw_light_combo_fast'].steps[0].name, 'settle')
        self.assertTrue(all(not s.metrics for s in self.sc['mw_light_combo_fast'].steps))

    def test_phase_follows_the_normal_combo_for_every_pack_weapon(self):
        names = [p['name'] for p in gw.PHASES]
        self.assertEqual(names[names.index('light_combo') + 1], 'light_combo_fast')
        self.assertIn('mw_light_combo_fast', gw.PARTS)
        phase = next(p for p in gw.PHASES if p['name'] == 'light_combo_fast')
        self.assertEqual((phase['part'], phase['kind'], phase['strike'], phase['expect_full']),
                         ('mw_light_combo_fast', 'pad', True, 'on'))
        for tag in gw.PACK_WEAPONS:
            plan = gw.phase_plan(tag)
            self.assertEqual(plan[plan.index('light_combo') + 1], 'light_combo_fast', tag)
        self.assertNotIn('light_combo_fast', gw.phase_plan('strong_arms'))      # the fists cyberware keeps its three phases
        self.assertTrue(all(p in self.sc for p in gw.PARTS))


if __name__ == '__main__':
    unittest.main()
