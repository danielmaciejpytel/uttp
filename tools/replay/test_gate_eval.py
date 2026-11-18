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


def side(**over):
    m = {'rt_clip_changes_s': 0.8, 'rt_corr_p95': 9.0, 'rt_corr_max': 14.0, 'rt_feed_p95': 100.0,
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

    def test_more_than_two_percent_fails_even_with_gain(self):
        g = self.gate(side(ws_pivot_p50=12.0 * 1.03, ws_loop_p50=5.0, rt_clip_changes_s=0.5))
        self.assertTrue(g['improves'])
        self.assertEqual(g['verdict'], 'FAIL')
        self.assertEqual([r['key'] for r in g['worse']], ['ws_pivot_p50'])

    def test_higher_is_better_keys(self):
        g = self.gate(side(stopPlantedHipKeptShare=0.2))
        self.assertEqual(g['verdict'], 'PASS')
        self.assertEqual([r['key'] for r in g['better']], ['stopPlantedHipKeptShare'])
        g = self.gate(side(startPlantedHipKeptShare=0.6))
        self.assertEqual(g['verdict'], 'FAIL')

    def test_hard_limits(self):
        for over, key in (({'rt_clip_changes_s': 3.2}, 'rt_clip_changes_s'), ({'rt_corr_max': 15.5}, 'rt_corr_max'),
                          ({'rt_feed_p95': 260.0}, 'rt_feed_p95')):
            before = side(**over)        # same before: no relative change, only the absolute limit can fail
            g = self.gate(side(**over), before=before)
            self.assertEqual(g['verdict'], 'FAIL', key)
            self.assertTrue(g['hard'][0].startswith(key))
        g = ge.evaluate(result(side(rt_corr_max=15.5), side(rt_corr_max=15.5)), None, corr_limit=35.0)['woman']
        self.assertEqual(g['verdict'], 'PASS')

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


if __name__ == '__main__':
    unittest.main()
