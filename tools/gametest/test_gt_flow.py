"""Tests of gt_flow (look-ahead probe cache, early stop) and gt_budget (time budget table). Pure code, no game.

  python Source\\tools\\gametest\\test_gt_flow.py -v
"""
from __future__ import annotations

import os
import sys
import unittest
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_budget as gb  # noqa: E402
import gt_flow as gf  # noqa: E402

LA = {'heading_deg': 0.0, 'duration': 2.5, 'reach_m': 8.6}
CF = (0.0, 1.0)


class ProbeCacheTests(unittest.TestCase):
    def cache(self, moved=5.9):
        c = gf.ProbeCache()
        c.remember('first', (10.0, 20.0, 1.0), CF, LA, moved)
        return c

    def test_same_spot_direction_and_reach_hit(self):
        hit = self.cache().lookup((10.4, 20.3), CF, LA)
        self.assertIsNotNone(hit)
        self.assertEqual(hit['where'], 'first')
        self.assertLessEqual(hit['dist_m'], 1.0)
        self.assertEqual((hit['cam_diff_deg'], hit['route_diff_deg']), (0.0, 0.0))

    def test_each_criterion_alone_is_enough_to_miss(self):
        c = self.cache()
        self.assertIsNone(c.lookup((11.2, 20.0), CF, LA), 'V more than 1 m from where the probe started')
        turned = (0.2588, 0.9659)                                                        # camera 15 deg right
        self.assertIsNotNone(c.lookup((10.0, 20.0), turned, dict(LA, heading_deg=-15.0)), 'world heading identical: still a hit')
        self.assertIsNone(c.lookup((10.0, 20.0), (0.3420, 0.9397), LA), 'camera 20 deg apart')
        self.assertIsNone(c.lookup((10.0, 20.0), CF, dict(LA, heading_deg=20.0)), 'route heading 20 deg apart')
        self.assertIsNone(c.lookup((10.0, 20.0), (0.2588, 0.9659), dict(LA, heading_deg=14.0)),
                          'each within 15 deg but the world heading 29 deg apart')

    def test_a_longer_probe_is_not_covered_by_a_shorter_one(self):
        c = self.cache(moved=5.9)
        self.assertIsNotNone(c.lookup((10, 20), CF, dict(LA, duration=2.0)))
        self.assertIsNone(c.lookup((10, 20), CF, dict(LA, duration=4.0)), 'would cover 9.4 m, the earlier walk 5.9 m')

    def test_invalidate_forgets_everything_and_keeps_the_reason(self):
        c = self.cache()
        c.invalidate('camera turned (clear path)')
        self.assertIsNone(c.lookup((10, 20), CF, LA))
        c.invalidate('nothing to forget')                                                # no entry: nothing recorded
        self.assertEqual(c.invalidated, ['camera turned (clear path)'])

    def test_same_probe_replaces_its_entry(self):
        c = self.cache()
        c.remember('second', (10.2, 20.1), CF, LA, 6.5)
        self.assertEqual(len(c.entries), 1)
        self.assertEqual(c.entries[0]['where'], 'second')


class EarlyStopTests(unittest.TestCase):
    def test_two_invalid_for_one_cause_stop(self):
        e = gf.EarlyStop(2)
        self.assertIsNone(e.note('kbd_switch_90#1', 'V does not move'))
        why = e.note('kbd_switch_180#1', 'V does not move')
        self.assertIn('first 2 walking scenarios (kbd_switch_90#1, kbd_switch_180#1)', why)
        self.assertIn('V does not move', why)
        self.assertNotIn('\n', why)                                                       # one line
        self.assertFalse(e.watching)
        self.assertIsNone(e.note('kbd_diagonals#1', 'V does not move'))                   # decided: nothing more is asked

    def test_a_valid_scenario_or_different_causes_never_stop(self):
        e = gf.EarlyStop(2)
        self.assertIsNone(e.note('a#1', None))
        self.assertIsNone(e.note('b#1', 'V does not move'))
        self.assertFalse(e.watching)
        e = gf.EarlyStop(2)
        e.note('a#1', 'V does not move')
        self.assertIsNone(e.note('b#1', 'no motion lines'))
        e = gf.EarlyStop(2)
        e.note('a#1', 'V does not move')
        self.assertIsNone(e.note('b#1', None))

    def test_zero_switches_it_off_and_three_waits_for_three(self):
        self.assertFalse(gf.EarlyStop(0).watching)
        self.assertIsNone(gf.EarlyStop(0).note('a#1', 'x'))
        e = gf.EarlyStop(3)
        self.assertIsNone(e.note('a#1', 'x'))
        self.assertIsNone(e.note('b#1', 'x'))
        self.assertIsNotNone(e.note('c#1', 'x'))


class BudgetTests(unittest.TestCase):
    def test_columns_add_up_and_play_is_the_rest(self):
        b = gb.Budget()
        b.begin('kbd_taps#1')
        b.add('wait', 2.0)
        b.add('probe', 6.8)
        b.add('copy', 0.4)
        b.add('retry_s', 5.0)
        b.count('retry_n')
        b.end(30.0)
        r = b.rows()[0]
        self.assertEqual((r['total'], r['probe'], r['wait'], r['copy'], r['retry']), (30.0, 6.8, 2.0, 0.4, '1/5.0'))
        self.assertAlmostEqual(r['play'], 30.0 - 6.8 - 2.0 - 0.4 - 5.0, places=1)

    def test_nothing_is_added_outside_a_scenario(self):
        b = gb.Budget()
        b.add('probe', 3.0)
        self.assertEqual(b.rows(), [])

    def test_phase_totals_from_the_steps_and_the_table(self):
        t = datetime(2026, 11, 14, 1, 0, 0)

        def st(name, s, e):
            return {'name': name, 'start': t + timedelta(seconds=s), 'end': t + timedelta(seconds=e)}
        steps = [st('ensure_closed', 0, 1), st('launch#1', 1, 51), st('arm', 51, 67), st('record_start', 67, 69),
                 st('scenarios', 69, 489), st('record_stop', 489, 490), st('quit', 490, 515), st('collect', 515, 640)]
        b = gb.Budget(clock=iter([0.0, 3.0]).__next__)
        with b.phase('sheets (wait)', note='72 jobs'):
            pass
        b.begin('kbd_taps#1')
        b.end(10.0)
        ph = b.phase_totals(steps)
        self.assertEqual(ph['launch'], 50.0)
        self.assertEqual(ph['recorder stop'], 1.0)
        self.assertEqual(ph['quit'], 25.0)
        self.assertEqual(ph['collect'], 125.0)
        text = '\n'.join(b.table(steps))
        self.assertIn('kbd_taps#1', text)
        self.assertIn('phases (s):', text)
        self.assertIn('sheets (wait) 3.0 (72 jobs)', text)
        self.assertIn('total 640.0', text)


if __name__ == '__main__':
    unittest.main()
