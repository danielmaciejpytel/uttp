"""Offline tests of gate_eval.py on synthetic results (no tools run).

  python Source\\tools\\replay\\test_gate_eval.py -v
"""
import copy
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gate_eval as ge  # noqa: E402
import compare_runs as cr  # noqa: E402


def side(**over):
    m = {'rt_clip_changes_s': 0.8, 'rt_clip_step_max': 2.0, 'rt_clip_step_missing': 0.0, 'rt_clip_win_max': 2.0, 'rt_corr_p95': 9.0,
         'rt_corr_max': 14.0, 'rt_feed_p95': 100.0,
         'startPlantedHipKeptShare': 0.7, 'startPlantedHipOffP50Cm': 3.0, 'stopPlantedHipKeptShare': 0.1,
         'stopPlantedHipOffP50Cm': 20.0}
    for i, c in enumerate(ge.CLASSES):
        m['ws_%s_n' % c] = 1000.0 * (i + 1)
        m['ws_%s_p50' % c] = 10.0 + i
        m['ws_%s_p95' % c] = 100.0 + 10 * i
        m['ws_%s_path_p95' % c] = 50.0
    m.update(over)
    return m


def result(before, after, body='woman'):
    return {'label': 't', 'bodies': {body: {'before': before, 'after': after}}}


class GateTest(unittest.TestCase):
    def gate(self, after, body='woman', margin=None, before=None):
        return ge.evaluate(result(before or side(), after, body), margin)[body]

    def test_identity_passes(self):
        g = self.gate(side())
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual(g['exceptions'], [])
        self.assertEqual(g['total'], 0.0)
        self.assertFalse(g['improves'])

    def test_minor_without_total_gain_fails(self):
        g = self.gate(side(ws_pivot_p50=12.0 * 1.015))   # pivot p50 12 -> 12.18 (+1.5 %), nothing else better
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertEqual([r['key'] for r in g['minor']], ['ws_pivot_p50'])

    def test_minor_with_total_gain_passes_and_is_listed(self):
        g = self.gate(side(ws_pivot_p50=12.0 * 1.015, ws_loop_p50=10.0, rt_clip_changes_s=0.7))
        self.assertTrue(g['improves'])
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual([r['key'] for r in g['exceptions']], ['ws_pivot_p50'])
        self.assertAlmostEqual(g['exceptions'][0]['rel'], 0.015, places=6)

    def test_more_than_two_percent_with_gain_is_conflicting(self):
        # a row worse beyond 2 % while the body total improves: not a plain FAIL any more but CONFLICT (1x film, the user decides)
        g = self.gate(side(ws_pivot_p50=12.0 * 1.03, ws_loop_p50=5.0, rt_clip_changes_s=0.5))
        self.assertTrue(g['improves'])
        self.assertEqual(g['verdict'], 'CONFLICT')
        self.assertEqual([r['key'] for r in g['worse']], ['ws_pivot_p50'])
        self.assertEqual([r['key'] for r in g['film_rows']], ['ws_pivot_p50'])
        self.assertIn(ge.CONFLICT_TEXT, ge.report('t', {'woman': g}))

    def test_more_than_two_percent_without_gain_fails(self):
        g = self.gate(side(ws_pivot_p50=12.0 * 1.03))
        self.assertFalse(g['improves'])
        self.assertEqual(g['verdict'], 'FAIL')
        g = self.gate(side(ws_pivot_p50=12.0 * 1.03, ws_loop_p50=5.0, rt_feed_p95=300.0))   # gain, but a hard limit: FAIL
        self.assertEqual(g['verdict'], 'FAIL')

    def test_higher_is_better_keys(self):
        g = self.gate(side(stopPlantedHipKeptShare=0.2))
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual([r['key'] for r in g['better']], ['stopPlantedHipKeptShare'])
        g = self.gate(side(startPlantedHipKeptShare=0.6))
        self.assertEqual(g['verdict'], 'FAIL')

    def test_hard_limits(self):
        # Feed cost stays absolute: the same value on both sides still fails
        g = self.gate(side(rt_feed_p95=260.0), before=side(rt_feed_p95=260.0))
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertTrue(g['hard'][0].startswith('rt_feed_p95'))
        # clip changes (mean and busiest scored step) are judged against the baseline (user decision 2026-11-23): the same value
        # above the limit on both sides is pre-existing; worse than such a baseline, or crossing the limit from below, fails
        for key, v in (('rt_clip_changes_s', 3.2), ('rt_clip_step_max', 3.67)):
            g = self.gate(side(**{key: v}), before=side(**{key: v}))
            self.assertEqual(g['hard'], [], key)
            self.assertEqual(len(g['preexisting']), 1, key)
            self.assertIn('already in the baseline', g['preexisting'][0])
            g = self.gate(side(**{key: v + 0.3}), before=side(**{key: v}))
            self.assertEqual(g['verdict'], 'FAIL', key)
            self.assertIn('worse than the baseline', g['hard'][0])
            g = self.gate(side(**{key: v}), before=side(**{key: 2.5}))
            self.assertEqual(g['verdict'], 'FAIL', key)
            self.assertTrue(g['hard'][0].startswith(key))
        g = ge.evaluate(result(side(rt_corr_max=15.5), side(rt_corr_max=15.5)), None, corr_limit=35.0)['woman']
        self.assertEqual(g['verdict'], 'PASS')

    def test_corr_max_judged_against_the_baseline(self):
        # above the limit in the baseline and not worse: pre-existing, not a FAIL (it used to fail every change)
        g = self.gate(side(rt_corr_max=30.0), before=side(rt_corr_max=30.0))
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual(g['hard'], [])
        self.assertEqual(len(g['preexisting']), 1)
        self.assertIn('already in the baseline', g['preexisting'][0])
        g = self.gate(side(rt_corr_max=30.03), before=side(rt_corr_max=30.0))      # within the 0.051 resolution
        self.assertEqual(g['hard'], [])
        g = self.gate(side(rt_corr_max=28.0), before=side(rt_corr_max=30.0))       # better, still above: pre-existing
        self.assertEqual(g['hard'], [])
        self.assertEqual(len(g['preexisting']), 1)
        # worse than a baseline already above: FAIL (no 2 % allowance on a hard limit)
        g = self.gate(side(rt_corr_max=30.3), before=side(rt_corr_max=30.0))
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertIn('worse than the baseline', g['hard'][0])
        # a baseline below the limit: crossing it fails
        g = self.gate(side(rt_corr_max=15.5), before=side(rt_corr_max=14.0))
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertTrue(g['hard'][0].startswith('rt_corr_max 15.50 > 15.0'))
        # the A/A margin widens the tolerance of the comparison with the baseline
        g = self.gate(side(rt_corr_max=30.3), before=side(rt_corr_max=30.0), margin={'woman': {'rt_corr_max': 0.5}})
        self.assertEqual(g['hard'], [])

    def test_vertical_correction(self):
        # not measured (no ground stage): reads 0, a note, no limit applied
        g = self.gate(side(rt_corr_v_max=0.0, rt_corr_v_measured=0.0))
        self.assertEqual(g['verdict'], 'PASS')
        self.assertTrue(any('vertical foot correction not measured' in n for n in g['notes']))
        g = self.gate(side())                                                      # older results: no key at all
        self.assertTrue(any('not measured' in n for n in g['notes']))
        # measured: limit 50 cm (user decision 2026-11-23, stairs), judged against the baseline like the horizontal one
        g = self.gate(side(rt_corr_v_max=29.0, rt_corr_v_measured=1.0), before=side(rt_corr_v_max=5.0, rt_corr_v_measured=1.0))
        self.assertEqual(g['hard'], [])
        self.assertFalse(any('vertical' in n for n in g['notes']))
        g = self.gate(side(rt_corr_v_max=51.0, rt_corr_v_measured=1.0), before=side(rt_corr_v_max=5.0, rt_corr_v_measured=1.0))
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertTrue(g['hard'][0].startswith('rt_corr_v_max 51.00 > 50.0'))
        g = ge.evaluate(result(side(rt_corr_v_max=5.0, rt_corr_v_measured=1.0), side(rt_corr_v_max=51.0, rt_corr_v_measured=1.0)),
                        None, corr_v_limit=55.0)['woman']
        self.assertEqual(g['hard'], [])
        # the horizontal limit is not affected by the vertical one
        g = self.gate(side(rt_corr_max=16.0, rt_corr_v_max=16.0, rt_corr_v_measured=1.0))
        self.assertEqual(len(g['hard']), 1)
        self.assertTrue(g['hard'][0].startswith('rt_corr_max'))

    def test_clip_changes_per_step(self):
        # the arc case: 3.67 / s in the scored 3 s arc step on the before side, a session mean near 1 / s
        b = side(rt_clip_changes_s=0.90, rt_clip_step_max=3.67)
        g = self.gate(side(rt_clip_changes_s=0.86, rt_clip_step_max=2.67), before=b)
        self.assertEqual(g['hard'], [])
        self.assertEqual(len(g['hard_before']), 1)
        self.assertTrue(g['hard_before'][0].startswith('rt_clip_step_max 3.670 > 3.0'))
        g = self.gate(side(rt_clip_changes_s=0.86, rt_clip_step_max=3.33), before=b)     # better, still above: pre-existing
        self.assertEqual(g['hard'], [])
        g = self.gate(side(rt_clip_changes_s=0.86, rt_clip_step_max=3.70), before=b)     # worse than the baseline: FAIL
        self.assertEqual(g['verdict'], 'FAIL')
        # the busiest window anywhere is a report only: no hard limit
        g = self.gate(side(rt_clip_win_max=9.3))
        self.assertEqual(g['hard'], [])
        self.assertTrue(any(n.startswith('clip changes in the busiest') for n in g['notes']))
        # an older result without the per-step measure: verdict kept, marked incomplete (never a complete pass)
        a = side()
        del a['rt_clip_step_max']
        g = self.gate(a)
        self.assertEqual(g['verdict'], 'PASS')
        self.assertTrue(g['incomplete'])
        self.assertIn('incomplete', ge.verdict_text(g))
        g = self.gate(side(rt_clip_step_missing=1.0))                    # a session without harness steps
        self.assertTrue(g['incomplete'])

    def test_where_names_the_session(self):
        b = side(rt_corr_max=30.0)
        b['_windows'] = {'s1': {'corrMax': '12.00', 'corrAt': '5.00s', 'corrCls': 'loop', 'corrClip': 'A'},
                         's2': {'corrMax': '30.00', 'corrAt': '81.20s', 'corrCls': 'stop', 'corrClip': 'Run_Stop',
                                'clipWinMax': '3.667', 'clipWinAt': '10.00-12.90s', 'clipWinCls': 'loop', 'clipWinClips': 'A>B',
                                'stepMax': '3.667', 'stepN': '11', 'stepDur': '3.00', 'stepLabel': 'run_arc_left#1/arc_120',
                                'stepGame': '3.667'}}
        g = self.gate(side(rt_corr_max=30.0), before=b)
        self.assertIn('s2 81.20s stop Run_Stop', g['preexisting'][0])
        self.assertEqual(ge.where(b, 'rt_clip_win_max'), ' at s2 10.00-12.90s loop (A>B)')
        self.assertEqual(ge.where(b, 'rt_clip_step_max'), ' at s2 run_arc_left#1/arc_120 (11 changes in 3.00 s; in game 3.667)')
        self.assertEqual(ge.where(side(), 'rt_corr_max'), '')

    def test_reliability(self):
        r = result(side(), side(ws_loop_p50=10.0, rt_clip_changes_s=0.7))
        self.assertEqual(ge.evaluate(r)['woman']['verdict'], 'PASS')
        # the baseline measured twice differs on a correctness row: RERUN, the measured verdict kept
        r['repeat'] = {'woman': {'ws_loop_p50': 0.5, 'rt_feed_p95': 300.0}}
        g = ge.evaluate(r)['woman']
        self.assertEqual(g['verdict'], 'RERUN')
        self.assertEqual(g['verdict_measured'], 'PASS')
        self.assertEqual(len(g['unreliable']), 1)
        self.assertIn('ws_loop_p50', g['unreliable'][0])
        # only the cost spread (wall time): reproducible
        r['repeat'] = {'woman': {'ws_loop_p50': 0.0, 'rt_feed_p95': 300.0}}
        self.assertEqual(ge.evaluate(r)['woman']['verdict'], 'PASS')
        # cost budget trips on either side: RERUN
        r2 = result(side(rt_budget_trips=2.0), side())
        self.assertEqual(ge.evaluate(r2)['woman']['verdict'], 'RERUN')
        # a busy machine alone is a warning, the verdict stays
        r3 = result(side(), side())
        r3['load'] = {'high': True, 'cpu_mean_pct': 85.0, 'cpu_max_pct': 100.0, 'compiler_processes_max': 12}
        g = ge.evaluate(r3)['woman']
        self.assertEqual(g['verdict'], 'PASS')
        self.assertTrue(g['warnings'][0].startswith('busy machine'))
        # ... but a Feed cost over its limit measured on a busy machine is not trusted: RERUN (FAIL as measured)
        r4 = result(side(), side(rt_feed_p95=600.0))
        r4['load'] = r3['load']
        g = ge.evaluate(r4)['woman']
        self.assertEqual((g['verdict'], g['verdict_measured']), ('RERUN', 'FAIL'))
        r4['load'] = {'high': False}
        self.assertEqual(ge.evaluate(r4)['woman']['verdict'], 'FAIL')

    def test_wait_for_idle(self):
        loads = iter([90.0, 80.0, 30.0])
        self.assertEqual(ge.wait_for_idle(60, probe=lambda: next(loads), period=5.0), (10.0, 30.0))
        self.assertEqual(ge.wait_for_idle(5, probe=lambda: 95.0, period=5.0), (5.0, 95.0))

    def test_invalid_rows_reported_not_decided(self):
        g = self.gate(side(ws_stop_p95=200.0))      # stop p95 100+10 -> 200: invalid on both bodies
        self.assertEqual(g['verdict'], 'PASS')
        self.assertTrue(g['depends_on_invalid'])
        self.assertEqual(g['verdict_with_invalid'], 'FAIL')
        self.assertIn('ws_stop_p95', [r['key'] for r in g['invalid_rows'] if r['state'] == 'worse'])
        g = self.gate(side(ws_loop_p95=200.0), body='woman')
        self.assertEqual(g['verdict'], 'PASS')
        g = self.gate(side(ws_loop_p95=200.0), body='man')
        self.assertEqual(g['verdict'], 'FAIL')

    def test_invalid_rows_left_out_of_the_total(self):
        g = self.gate(side(ws_stop_p95=10.0))       # big "gain" on an invalid row must not count
        self.assertEqual(g['total'], 0.0)

    def test_total_is_sample_weighted(self):
        b = side()
        a = side(ws_other_p50=b['ws_other_p50'] - 1.0)   # other has the largest weight (7000)
        g = self.gate(a)
        w = sum(b['ws_%s_n' % c] for c in ge.CLASSES)
        p50 = sum(b['ws_%s_n' % c] * b['ws_%s_p50' % c] for c in ge.CLASSES) / w
        self.assertAlmostEqual(g['total_parts']['P50']['rel'], -7000.0 / w / p50, places=9)
        self.assertAlmostEqual(g['total'], g['total_parts']['P50']['rel'] / 3.0, places=9)

    def test_total_ignores_class_changes_within_resolution(self):
        b = side()
        # start moves by 0.01 (within its 0.011 resolution: unchanged), pivot by 0.02 (beyond it: counted)
        a = side(ws_start_p50=b['ws_start_p50'] + 0.01, ws_pivot_p50=b['ws_pivot_p50'] - 0.02)
        g = self.gate(a)
        w = sum(b['ws_%s_n' % c] for c in ge.CLASSES)
        p50 = sum(b['ws_%s_n' % c] * b['ws_%s_p50' % c] for c in ge.CLASSES) / w
        self.assertAlmostEqual(g['total_parts']['P50']['rel'], -0.02 * b['ws_pivot_n'] / w / p50, places=9)
        self.assertTrue(g['improves'])
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual([r['key'] for r in g['better']], ['ws_pivot_p50'])

    def test_margin_widens_tolerance(self):
        g = self.gate(side(rt_corr_p95=9.5), margin={'woman': {'rt_corr_p95': 0.6}})
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual(g['minor'] + g['worse'], [])

    def test_zero_baseline(self):
        g = self.gate(side(ws_idle_p50=0.5), before=side(ws_idle_p50=0.0))
        self.assertEqual(g['verdict'], 'FAIL')
        g = self.gate(side(ws_idle_p50=0.01), before=side(ws_idle_p50=0.0))   # rounding unit: same
        self.assertEqual(g['verdict'], 'PASS')

    def test_table_text_parse(self):
        text = '\n'.join([
            'before woman 1 #0 clip/s 0.660 ...',
            '',
            '== woman (20261118-121140)',
            '%-48s %10s %10s %10s %10s' % ('metric', 'before', 'after', 'delta', ''),
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: clip changes / s', 0.66, 0.65, -0.01),
            '%-48s %10.3f %10.3f %10.3f ' % ('world slide pivot p50, cm/s', 14.59, 14.0, -0.59),
            '%-48s %10.3f %10.3f %10.3f ' % ('world slide pivot locks', 84, 80, -4),
            '%-48s %10s %10.3f %10s ' % ('runtime: Feed us p95 (median of repeats)', '-', 99.0, '-'),
            '== man (20261118-130756)',
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: foot correction max, cm', 14.0, 14.1, 0.1),
        ])
        r = ge.parse_table_text(text)
        self.assertEqual(r['woman']['before']['rt_clip_changes_s'], 0.66)
        self.assertEqual(r['woman']['after']['ws_pivot_p50'], 14.0)
        self.assertEqual(r['woman']['before']['ws_pivot_n'], 84.0)   # lock counts as weights in text mode
        self.assertNotIn('rt_feed_p95', r['woman']['before'])
        self.assertEqual(r['woman']['after']['rt_feed_p95'], 99.0)
        self.assertEqual(r['man']['after']['rt_corr_max'], 14.1)

    def test_table_text_window_rows(self):
        w = dict(window_s=3.0, changes=40, secs=120.0, clip_rate=11 / 3.0, clip_n=11, clip_t0=50.0, clip_t1=52.9, clip_cls='loop',
                 clip_names=['A', 'B', 'C'], corr_max=30.0, corr_t=81.2, corr_cls='stop', corr_clip='Run_Stop', corr_v_max=None,
                 budget_trips=0)
        text = '\n'.join([
            cr.window_line('before', 'woman', '20261122-132705', w),
            cr.window_line('aa1', 'woman', '20261122-132705', w),          # A/A side: not a before / after value
            '== woman (20261122-132705)',
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: clip changes / s, busiest window', 3.667, 2.667, -1.0),
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: foot correction vertical max, cm', 0.0, 0.0, 0.0),
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: vertical foot correction measured', 0.0, 0.0, 0.0),
            '%-48s %10.3f %10.3f %10.3f ' % ('runtime: cost budget trips', 0.0, 1.0, 1.0),
        ])
        r = ge.parse_table_text(text)['woman']
        self.assertEqual(r['before']['rt_clip_win_max'], 3.667)
        self.assertEqual(r['after']['rt_budget_trips'], 1.0)
        self.assertEqual(r['after']['rt_corr_v_measured'], 0.0)
        wb = r['before']['_windows']['20261122-132705']
        self.assertEqual((wb['clipWinMax'], wb['clipWinAt'], wb['clipWinCls'], wb['clipWinClips']), ('3.667', '50.00-52.90s', 'loop',
                                                                                                      'A>B>C'))
        self.assertEqual((wb['corrMax'], wb['corrAt'], wb['corrClip'], wb['corrVMax']), ('30.00', '81.20s', 'Run_Stop', '-'))
        self.assertNotIn('_windows', r['after'])

    def test_planted_hip_aggregate(self):
        line = ('PICKS run=x startPicks=5 startPlantedHipN=10 startPlantedHipKept=8 startPlantedHipOffP50Cm=2.00 '
                'stopPlantedHipN=20 stopPlantedHipKept=2 stopPlantedHipOffP50Cm=20.00')
        line2 = line.replace('startPlantedHipN=10', 'startPlantedHipN=30').replace('startPlantedHipOffP50Cm=2.00',
                                                                                      'startPlantedHipOffP50Cm=4.00')
        agg = ge.aggregate_planted_hip([ge.planted_hip_from_stages(line), ge.planted_hip_from_stages(line2)])
        self.assertEqual(agg['startPlantedHipN'], 40)
        self.assertAlmostEqual(agg['startPlantedHipKeptShare'], 16 / 40)
        self.assertAlmostEqual(agg['startPlantedHipOffP50Cm'], (10 * 2 + 30 * 4) / 40)
        self.assertAlmostEqual(agg['stopPlantedHipKeptShare'], 4 / 40)

    def test_eval_cli_with_baseline(self):
        d = tempfile.mkdtemp()
        try:
            base = result(side(), side())
            var = result(side(ws_loop_p50=1.0), side(ws_loop_p50=9.0))   # its own before is the variant data: replaced
            pb, pv, po = (os.path.join(d, n) for n in ('b.json', 'v.json', 'o.json'))
            for p, obj in ((pb, base), (pv, var)):
                with open(p, 'w', encoding='utf-8') as f:
                    json.dump(obj, f)
            import contextlib
            import io
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(ge.main(['eval', pv, '--baseline', pb, '--json-out', po]), 0)
            with open(po, encoding='utf-8') as f:
                out = json.load(f)
            g = out[0]['gates']['woman']
            self.assertEqual(g['verdict'], 'PASS')
            self.assertIn('ws_loop_p50', [r['key'] for r in g['better']])
            # the own before side differed from the stored baseline (ws_loop_p50 1.0 vs 10.0): a warning, the verdict stays
            self.assertTrue(any(w.startswith('own before side differs') and 'ws_loop_p50' in w for w in g['warnings']))
        finally:
            for n in os.listdir(d):
                os.remove(os.path.join(d, n))
            os.rmdir(d)

    def test_unchanged_input_not_mutated(self):
        b, a = side(), side(rt_clip_changes_s=0.7)
        r = result(b, a)
        snap = copy.deepcopy(r)
        ge.evaluate(r)
        self.assertEqual(r, snap)


class WindowTest(unittest.TestCase):
    """compare_runs per-row window measures (session_windows) on synthetic stage meta arrays."""

    def test_clip_change_rows_follow_the_runtime_count(self):
        # a row without a pick neither counts nor resets the last pick: A, -, A is no change
        self.assertEqual(cr.clip_change_rows([-1, 1, 1, -1, 1, 2, 2, -1, 3]), [5, 8])
        self.assertEqual(cr.clip_change_rows([-1, -1]), [])

    def test_busiest_window(self):
        t = [0.1 * i for i in range(100)]                 # 10 s at 10 Hz
        rows = [5, 30, 31, 33, 35, 38, 41, 90]            # 6 changes in 3.10 .. 4.10 s
        self.assertEqual(cr.busiest_window(t, rows, 3.0), (6, 30, 41))
        self.assertEqual(cr.busiest_window(t, rows, 0.5), (3, 30, 33))   # first of the equal windows
        self.assertEqual(cr.busiest_window(t, [], 3.0), (0, None, None))

    def test_session_windows(self):
        import numpy as np
        n = 300                                           # 10 s at 30 Hz
        t = np.arange(n) / 30.0
        clip = np.zeros(n)
        # session: one burst of 11 changes in 3 s (the running arc), else steady
        for k, i in enumerate(range(150, 240, 9)):
            clip[i:] = k + 1
        clip[20:25] = np.nan                              # no pick for a moment: no change
        corrL = np.zeros(n)
        corrR = np.zeros(n)
        corrR[200] = 31.0
        cls = np.array(['loop'] * n, dtype=object)
        cls[200] = 'stop'
        names = ['c%d' % i for i in range(20)]
        m = dict(t=t, clip=clip, corrL_cm=corrL, corrR_cm=corrR)
        w = cr.session_windows(m, cls, names, 3.0)
        self.assertEqual(w['changes'], 10)
        self.assertEqual(w['clip_n'], 10)
        self.assertAlmostEqual(w['clip_rate'], 10 / 3.0)
        self.assertEqual(w['clip_names'][:3], ['c0', 'c1', 'c2'])
        self.assertEqual((w['corr_max'], w['corr_cls'], w['corr_clip']), (31.0, 'stop', 'c6'))
        self.assertAlmostEqual(w['corr_t'], 200 / 30.0)
        self.assertIsNone(w['corr_v_max'])                # no ground stage in the output
        self.assertIsNone(w['budget_trips'])              # no trace file
        m['corrVL_cm'] = np.zeros(n)
        m['corrVL_cm'][50] = -12.0
        w = cr.session_windows(m, cls, names, 3.0)
        self.assertEqual(w['corr_v_max'], 12.0)
        line = cr.window_line('before', 'woman', 'x', w)
        self.assertIn('clipWinMax=3.333', line)
        self.assertIn('corrVMax=12.00', line)

    def test_steps_on_the_feed_clock(self):
        # a recorded session: harness steps in wall time, the game log and the replay trace share the driver time t=; the feed clock
        # starts 31.8 s before the driver (the replay trace is stamped with the feed time as 2000-01-01 wall time)
        import datetime
        import numpy as np
        d = tempfile.mkdtemp()
        try:
            os.makedirs(os.path.join(d, 'gametest'))
            t0 = datetime.datetime(2026, 11, 22, 13, 28, 14, 5000)     # wall time of feed time 0
            steps = [dict(scenario='run_arc_left', run=1, step='arc_120', verdict='FAIL', metrics={'motion_transitions_per_s': 3.667},
                          t0=str(t0 + datetime.timedelta(seconds=70)), t1=str(t0 + datetime.timedelta(seconds=73))),
                     dict(scenario='clear_probe', run=1, step='leave', verdict='SKIP', metrics={},
                          t0=str(t0 + datetime.timedelta(seconds=10)), t1=str(t0 + datetime.timedelta(seconds=11))),
                     dict(scenario='slalom_stick', run=1, step='slalom', verdict='PASS', metrics={'motion_transitions_per_s': 2.0},
                          t0=str(t0 + datetime.timedelta(seconds=100)), t1=str(t0 + datetime.timedelta(seconds=108)))]
            with open(os.path.join(d, 'gametest', 'summary.json'), 'w', encoding='utf-8') as f:
                json.dump({'steps': steps}, f)
            st = cr.harness_steps(d)
            self.assertEqual([s['label'] for s in st], ['run_arc_left#1/arc_120', 'slalom_stick#1/slalom'])   # SKIP left out
            self.assertIsNone(cr.harness_steps(os.path.join(d, 'missing')))

            def motion(wall, t, clip=495):
                return ('[%s] [info    ] [ 1] [UTPP] UTPP MOTION t=%.3f db=11:PSD_Dense_Stand_Idles clip=%d:M_Neutral_Stand_Idle_Loop '
                        'ct=8.133 cost=0.113 blend=0.00(live) rot=strafe start=0 pivot=0\n' % (wall, t, clip))
            game, rep = os.path.join(d, 'game.log'), os.path.join(d, 'x.motion.log')
            with open(game, 'w', encoding='utf-8') as fg, open(rep, 'w', encoding='utf-8') as fr:
                for k in range(5):
                    dt = 31.8 + 0.5 * k
                    fg.write(motion((t0 + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], 0.5 * k + 0.015))
                    fr.write(motion((datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f'),
                                    0.5 * k + 0.015))
            origin, n, spread = cr.feed_clock_origin(rep, game)
            self.assertEqual(n, 5)
            self.assertLess(abs((origin - t0).total_seconds()), 2e-3)
            t = np.arange(0, 120, 0.01)
            rows = [int(round(x * 100)) for x in (69.9, 70.1, 70.5, 71.0, 72.9, 73.0, 101.0, 105.0)]   # 4 changes in [70, 73)
            r = cr.step_rates(t, rows, st, origin)
            self.assertEqual([(x[1], round(x[2], 2), x[3]) for x in r], [(4, 3.0, 'run_arc_left#1/arc_120'), (2, 8.0, 'slalom_stick#1/slalom')])
            self.assertAlmostEqual(r[0][0], 4 / 3.0, places=3)
            self.assertEqual(r[0][4], 3.667)

            # the driver started at another feed row in the game: no t= pairs, the pick changes of the same clips place the clock
            def change(wall, t, clip):
                return motion(wall, t, clip).replace(' pivot=0', ' pivot=0 chg')
            with open(game, 'w', encoding='utf-8') as fg, open(rep, 'w', encoding='utf-8') as fr:
                for k, clip in enumerate((495, 414, 417, 495, 414, 300)):
                    dt = 31.8 + 2.0 * k
                    fg.write(change((t0 + datetime.timedelta(seconds=dt + 0.004 * (k % 2))).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                                    2.0 * k + 0.016, clip))
                    fr.write(change((datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f'),
                                    2.0 * k + 0.023, clip))
                fg.write(change((t0 + datetime.timedelta(seconds=50)).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], 18.2, 999))   # game only
            origin, n, spread = cr.feed_clock_origin(rep, game)
            self.assertGreaterEqual(n, 6)
            self.assertLess(abs((origin - t0).total_seconds()), 5e-3)
            self.assertLess(spread, cr.CLOCK_TOLERANCE_S)
            # driver time offset 0.98 s: a few 2 Hz lines of both traces land on the same t= by chance and agree with each other at the
            # wrong origin; the pick changes of the same clips place the clock and win
            with open(game, 'w', encoding='utf-8') as fg, open(rep, 'w', encoding='utf-8') as fr:
                for k, clip in enumerate((495, 414, 417, 495, 414, 300)):
                    dt = 31.8 + 2.0 * k
                    fg.write(change((t0 + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], 2.0 * k + 0.016 + 0.98, clip))
                    fr.write(change((datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f'),
                                    2.0 * k + 0.016, clip))
                for k in range(4):
                    dt = 60.0 + 0.5 * k
                    fg.write(motion((t0 + datetime.timedelta(seconds=dt + 0.98)).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3], 0.5 * k + 40.0))
                    fr.write(motion((datetime.datetime(2000, 1, 1) + datetime.timedelta(seconds=dt)).strftime('%Y-%m-%d %H:%M:%S.%f'),
                                    0.5 * k + 40.0))
            origin, n, spread = cr.feed_clock_origin(rep, game)
            self.assertLess(abs((origin - t0).total_seconds()), 5e-3)   # the chance t= pairs alone would put it 0.98 s late
            self.assertGreaterEqual(n, 6)
            with open(game, 'w', encoding='utf-8') as fg:   # unrelated changes: no agreement, no clock
                fg.write(change('2026-11-22 13:29:00.000', 1.0, 1) + change('2026-11-22 13:29:07.000', 2.0, 2))
            self.assertIsNone(cr.feed_clock_origin(rep, game))
        finally:
            import shutil
            shutil.rmtree(d)

    def test_budget_trips_counted_from_the_trace(self):
        import numpy as np
        d = tempfile.mkdtemp()
        p = os.path.join(d, 'x.motion.log')
        try:
            with open(p, 'w', encoding='utf-8') as f:
                f.write('[2000-01-01 00:00:01.000000] UTPP MOTION db=0:x clip=1:a\n'
                        '[2000-01-01 00:00:02.000000] UTPP mm.motionDriver budget trip #1 p95us=600.0 wallP95us=610.0 limit=250\n')
            m = dict(t=np.arange(10) / 30.0, clip=np.zeros(10))
            self.assertEqual(cr.session_windows(m, np.array(['idle'] * 10), ['a'], 3.0, p)['budget_trips'], 1)
        finally:
            os.remove(p)
            os.rmdir(d)


if __name__ == '__main__':
    unittest.main()
