"""Offline tests for the traversal set (gt_traversal.py): vault and climb at the summoned car.

The log lines are shaped like the ones the plugin writes ("UTPP trav:", the vehicle vault line, the release line). FakeNight plays the
parts of the night runner the macro uses; the script says which lines the game writes for a pad part. No input is sent.

  python Source\\tools\\gametest\\test_gt_traversal.py -v
"""
from __future__ import annotations

import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_night as gn  # noqa: E402
import gt_settings as gs  # noqa: E402
import gt_traversal as gt  # noqa: E402
from gt_scenario import load_scenarios  # noqa: E402

TRAV_OK = lambda action: (f'UTPP trav: action={action} h=0.90 ext=1.20 depth=1.00 back=0.00 v=3.0 kind=1 decision=Vault '  # noqa: E731
                          'result=Ok(-) montage=Vault_Low')
TRAV_REJECTED = ('UTPP trav: action=9 h=0.90 ext=1.20 depth=1.00 back=0.00 v=0.4 kind=1 decision=- '
                 'result=Rejected(too slow) montage=-')
VEHICLE_NO = '[utpp_vehicle_vault] no vault over a car (vanilla: jump pressed too far from the edge; assist: speed) top=0.9'
VEHICLE_ASSIST = '[utpp_vehicle_vault] assisted vault over a car (vanilla: jump pressed too far from the edge) top=0.9'
RELEASED = lambda d: f'UTPP trav: released to game animation (detail={d})'  # noqa: E731


class FakeGuard:
    def wait(self, s):
        pass


class FakeNight:
    def __init__(self, tmp, script, car=True, approach_dist=0.5):
        self.utpp_log = os.path.join(tmp, 'session.log')
        open(self.utpp_log, 'w').close()
        self.args = types.SimpleNamespace(log_dir=tmp)
        self.settings = gs.Ledger(None, log=lambda m: None)
        self.script, self.car, self.approach_dist = script, car, approach_dist
        self.door_spot_override = None
        self.calls, self.logs, self.plays = [], [], {}

    def log(self, msg):
        self.logs.append(msg)

    def append(self, lines):
        with open(self.utpp_log, 'a', encoding='utf-8') as f:
            for ln in lines:
                f.write('[2026-11-14 19:00:00.000] [info    ] [ 1] [UTPP] ' + ln + '\n')

    def check_continue(self):
        pass

    def console_verified(self, pad, guard, cmd, echo, label, timeout=3.0):
        self.calls.append(('console', label))
        self.append(['CET [night] TRAVLOG ok=true'])
        return '[night] TRAVLOG ok=true'

    def poll_pos(self, guard, pad=None, timeout=None):
        if not self.car:
            return None
        return {'p': (0.0, 0.0, 0.0), 'cf': (1.0, 0.0), 'v': (5.0, 0.0, 0.0), 'vr': (0.0, 1.0), 'src': 'summon'}

    def locate_vehicle(self, pad, guard, plan, ep):
        self.calls.append(('locate',))
        return None

    def wait_car_stopped(self, pad, guard, ep, pos):
        return pos, True

    def approach(self, pad, guard, plan, pos, ep):
        self.calls.append(('approach', self.door_spot_override))
        return self.approach_dist

    def play(self, pad, guard, name, plan, sc=None):
        self.calls.append(('play', name))
        self.plays[name] = self.plays.get(name, 0) + 1
        self.append(self.script(name, self.plays[name]))
        return {}


def success(part, n):
    return [TRAV_OK(9 if part == 'trav_vault_jump' else 8)]


class TraversalSetTests(unittest.TestCase):
    def test_the_set_is_known_and_not_part_of_all(self):
        self.assertEqual(gn.resolve_sets('traversal'), ['traversal'])
        self.assertNotIn('traversal', gn.resolve_sets('all'))
        scen = load_scenarios(os.path.join(HERE, 'scenarios.json'))
        self.assertEqual(gn.validate_sets(['traversal'], scen), [])
        for part in gt.PARTS:
            self.assertGreater(scen[part].duration, 2.0)
            self.assertFalse(any(st.metrics for st in scen[part].steps), part)      # judged from the log, not by limits

    def test_each_part_taps_jump_while_the_stick_pushes_into_the_car(self):
        scen = load_scenarios(os.path.join(HERE, 'scenarios.json'))
        for part in gt.PARTS:
            jump = [st for st in scen[part].steps if st.name == 'jump'][0]
            self.assertTrue([b for b in jump.buttons if b.button == 'A'], part)
            self.assertTrue(any(f.ls[1] > 0.9 for _, f in scen[part].sample(0.1)), part)


class EvidenceTests(unittest.TestCase):
    def test_lines_are_read(self):
        text = '\n'.join(['noise', TRAV_REJECTED, TRAV_OK(8), RELEASED(9), VEHICLE_NO, VEHICLE_ASSIST,
                          'UTPP MMTRAV travRelease on detail=9 gate=0x0 motionOwns=1 w=1.00'])
        ev = gt.evidence(text)
        self.assertEqual([(e['action'], e['result']) for e in ev['mm']], [(9, 'Rejected'), (8, 'Ok')])
        self.assertEqual(ev['mm'][0]['reason'], 'too slow')
        self.assertEqual(ev['released'], [9])
        self.assertEqual([v['kind'] for v in ev['vehicle_vault']], ['no', 'assisted'])
        self.assertEqual(ev['mmtrav'], 1)

    def test_judge_vault_and_climb(self):
        self.assertEqual(gt.judge('vault', gt.evidence(TRAV_OK(9)))['by'], ['trav line Ok'])
        self.assertTrue(gt.judge('vault', gt.evidence(RELEASED(9)))['ok'])
        self.assertTrue(gt.judge('vault', gt.evidence(VEHICLE_ASSIST))['ok'])
        self.assertTrue(gt.judge('climb', gt.evidence(TRAV_OK(8)))['ok'])
        self.assertFalse(gt.judge('climb', gt.evidence(VEHICLE_ASSIST))['ok'])       # the vehicle line is a vault only
        self.assertFalse(gt.judge('climb', gt.evidence(TRAV_OK(9)))['ok'])           # the other action does not count
        rej = gt.judge('vault', gt.evidence(TRAV_REJECTED + '\n' + VEHICLE_NO))
        self.assertFalse(rej['ok'])
        self.assertIn('Rejected(too slow)', rej['why'])
        self.assertIn('too far from the edge', rej['why'])
        self.assertIn('no traversal line at all', gt.judge('vault', gt.evidence(''))['why'])

    def test_feed_column_detailed(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'utpp-feed-2026-11-14-19-00-00.txt')
            rows = ['# cols t dt vx detailed onGround']
            for i in range(40):                                  # 0.05 s per row; vault (9) in the last second only
                rows.append(f'{i * 0.05:.3f} 0.05 0 {9 if i >= 25 else 0} 1')
            with open(path, 'w') as f:
                f.write('\n'.join(rows) + '\n')
            self.assertEqual(gt.feed_detailed(path, 0.5), {9})
            self.assertEqual(gt.feed_detailed(path, 5.0), {0, 9})
            self.assertEqual(gt.newest_feed(d), path)
            self.assertTrue(gt.judge('vault', gt.evidence(''), {9})['ok'])
            self.assertFalse(gt.judge('climb', gt.evidence(''), {9})['ok'])
            self.assertIsNone(gt.feed_detailed(os.path.join(d, 'none.txt'), 1.0))
            bad = os.path.join(d, 'utpp-feed-x.txt')
            with open(bad, 'w') as f:
                f.write('# cols t dt\n0 0.1\n')
            self.assertIsNone(gt.feed_detailed(bad, 1.0))


class MacroTests(unittest.TestCase):
    def run_macro(self, script, **kw):
        with tempfile.TemporaryDirectory() as d:
            night = FakeNight(d, script, **kw)
            out = gt.run_traversal_macro(night, None, FakeGuard(), {}, {})
        return night, out

    def test_both_traversals_happen_at_the_first_attempt(self):
        night, out = self.run_macro(success)
        self.assertEqual(out['verdict'], 'PASS')
        self.assertEqual(out['results'], {'vault': True, 'climb': True})
        self.assertEqual([c[1] for c in night.calls if c[0] == 'approach'], [(1.4, 1.3), (-0.2, 1.3)])   # the spot of each kind
        self.assertEqual([c[1] for c in night.calls if c[0] == 'play'], ['trav_vault_jump', 'trav_climb_jump'])
        self.assertEqual([w['label'] for w in out['phase_windows']], ['trav_vault_1', 'trav_climb_1'])
        self.assertTrue(all(w['start'] <= w['end'] for w in out['phase_windows']))
        self.assertIsNone(night.door_spot_override)                                  # the vehicle macro's door is back
        self.assertEqual(night.calls[0], ('console', 'traversal/log_on'))
        self.assertEqual(night.calls[-1], ('console', 'traversal/log_off'))
        self.assertEqual([e['state'] for e in night.settings.entries], ['restored'])  # the vault log is switched off again
        # phase windows are what the pose meter and the sheets read
        self.assertEqual([w['name'] for w in gn.phase_windows_of([dict(out, name='traversal', run=1)])],
                         ['trav_vault_1_r1', 'trav_climb_1_r1'])

    def test_a_second_attempt_can_succeed(self):
        def script(part, n):
            return [TRAV_REJECTED] if part == 'trav_vault_jump' and n == 1 else success(part, n)
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'PASS')
        vault = [a for a in out['attempts'] if a['kind'] == 'vault']
        self.assertEqual([a['ok'] for a in vault], [False, True])
        self.assertEqual(night.plays['trav_vault_jump'], 2)

    def test_a_traversal_that_never_happens_fails_with_the_reason(self):
        def script(part, n):
            return success(part, n) if part == 'trav_vault_jump' else [VEHICLE_NO]
        night, out = self.run_macro(script)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual(out['results'], {'vault': True, 'climb': False})
        self.assertEqual(night.plays['trav_climb_jump'], gt.ATTEMPTS)
        self.assertEqual(len(out['fails']), 1)
        self.assertTrue(out['fails'][0].startswith('climb: '))
        self.assertIn('too far from the edge', out['fails'][0])

    def test_no_car_is_a_skip(self):
        night, out = self.run_macro(success, car=False)
        self.assertEqual(out['verdict'], 'SKIP')
        self.assertIn(('locate',), night.calls)                                    # the vehicle macro's summon was tried first
        self.assertEqual([c for c in night.calls if c[0] == 'play'], [])
        self.assertEqual(night.calls[-1], ('console', 'traversal/log_off'))

    def test_v_that_cannot_reach_the_spot_does_not_play_the_part(self):
        night, out = self.run_macro(success, approach_dist=6.0)
        self.assertEqual(out['verdict'], 'FAIL')
        self.assertEqual([c for c in night.calls if c[0] == 'play'], [])
        self.assertTrue(all(not a['played'] for a in out['attempts']))
        self.assertIn('could not stand at the car', out['fails'][0])

    def test_the_macro_failure_counts_in_the_night_verdict(self):
        night, out = self.run_macro(lambda part, n: [])
        rec = dict(out, name='traversal', run=1)
        fails = gn.macro_failures([rec])
        self.assertEqual([f['scenario'] for f in fails], ['traversal#1'])
        self.assertTrue(gn.macro_result_text(fails, []).startswith('macro FAIL: traversal#1'))


if __name__ == '__main__':
    unittest.main()
