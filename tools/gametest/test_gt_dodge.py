"""Offline tests for the dodge push check of the game-test harness (gt_log.dodge_metrics, run.analyze, the scenario "dodge" flag).

The fixtures in testdata are slices of real game logs of the sprint_dash scenario (dash_fwd / dash_left / dash_right / dash_back
press B with a stick direction): two runs of a build that writes 'UTPP MOTIONDODGE push' lines and one run of an older build that
does not. The failing cases are synthetic edits of the first fixture and say so. No input is sent.

  python Source\\tools\\gametest\\test_gt_dodge.py -v
"""
from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_scenario as gs  # noqa: E402
import run as gt_run  # noqa: E402

TESTDATA = os.path.join(HERE, 'testdata')
FIXED_A = os.path.join(TESTDATA, 'utpp-dodge-20261114-171313-sprint_dash.txt')
FIXED_B = os.path.join(TESTDATA, 'utpp-dodge-20261114-171759-sprint_dash.txt')
OLD_BUILD = os.path.join(TESTDATA, 'utpp-dodge-20261114-141814-sprint_dash.txt')

# the step windows of the three runs (plan.json of the runs the fixtures were cut from)
WINDOWS = {
    FIXED_A: ('2026-11-14 17:14:{s}', [('dash_fwd', '41.475', '43.475'), ('dash_left', '43.475', '45.475'),
                                       ('dash_right', '45.475', '47.475'), ('dash_back', '47.475', '49.475')]),
    FIXED_B: ('2026-11-14 17:19:{s}', [('dash_fwd', '13.884', '15.884'), ('dash_left', '15.884', '17.884'),
                                       ('dash_right', '17.884', '19.884'), ('dash_back', '19.884', '21.884')]),
    OLD_BUILD: ('2026-11-14 14:28:{s}', [('dash_fwd', '13.178', '15.178'), ('dash_left', '15.178', '17.178'),
                                         ('dash_right', '17.178', '19.178'), ('dash_back', '19.178', '21.178')]),
}
# the pressed direction of the four steps, camera relative (+90 = left)
PRESSED = {'dash_fwd': 0.0, 'dash_left': 90.0, 'dash_right': -90.0, 'dash_back': 180.0}
DODGE_THRESHOLDS = {k: v for k, v in gt_log.DEFAULT_THRESHOLDS.items() if k.startswith('dodge_')}


def _t(fmt: str, s: str) -> datetime:
    return datetime.strptime(fmt.format(s=s), '%Y-%m-%d %H:%M:%S.%f')


def read(path: str) -> str:
    with open(path, encoding='utf-8') as f:
        return f.read()


def plan_of(path: str) -> dict:
    fmt, steps = WINDOWS[path]
    return {'runs': [{'scenario': 'sprint_dash', 'run': 1, 'aborted': None, 'steps': [
        {'name': n, 'start': _t(fmt, a), 'end': _t(fmt, b), 'metrics': True} for n, a, b in steps]}]}


def analyze(text: str, path: str, thresholds=None):
    scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
    lines = gt_log.parse_text(text)
    rows = gt_run.analyze(plan_of(path), lines, thresholds or DODGE_THRESHOLDS, scen)
    return {r['step']: r for r in rows}, lines


def replace_once(text: str, old: str, new: str) -> str:
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)


class ParseTests(unittest.TestCase):
    def test_push_line_fields(self):
        lines = gt_log.parse_text(read(FIXED_A))
        dodge = [l for l in lines if l.kind == 'DODGE']
        self.assertEqual([l.fields['sub'] for l in dodge], ['push', 'push', 'start', 'push', 'end', 'push'])
        p = dodge[1]
        self.assertEqual((p.num('served'), p.num('want'), p.num('err'), p.num('cam')), (90.0, -65.0, 0.0, -155.0))
        self.assertEqual(p.dodge_travel(), (-64.9, 2.27))
        self.assertEqual(p.fields['clip'], 'none')
        self.assertEqual(dodge[3].fields['clip'], 'B')
        self.assertIn('DODGE push t=17.744', p.compact())

    def test_start_line_is_not_a_motion_line(self):
        for l in gt_log.parse_text(read(FIXED_A)):
            if 'MOTIONDODGE' in l.raw:
                self.assertEqual(l.kind, 'DODGE')
        self.assertTrue(gt_log.has_dodge_push(gt_log.parse_text(read(FIXED_A))))
        # the older build writes start / end lines only
        old = gt_log.parse_text(read(OLD_BUILD))
        self.assertTrue(any(l.kind == 'DODGE' for l in old))
        self.assertFalse(gt_log.has_dodge_push(old))

    def test_push_line_without_a_served_heading(self):
        line = ('[2026-11-14 17:14:44.233] [info    ] [ 1] [UTPP] UTPP MOTIONDODGE push t=17.744 kind=ground cam=-155.0 served=- '
                'travel=-64.9/2.27m facing=-86.8 pick=F clip=none rel=18.9')
        l = gt_log.parse_line(line)
        self.assertEqual(l.kind, 'DODGE')
        self.assertIsNone(l.num('served'))
        self.assertIsNone(l.num('err'))
        self.assertEqual(l.dodge_travel(), (-64.9, 2.27))


class ScenarioTests(unittest.TestCase):
    def setUp(self):
        self.scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))

    def test_presses_and_the_stick_held_at_them(self):
        presses = self.scen['sprint_dash'].dodge_presses()
        self.assertEqual(list(presses), ['dash_fwd', 'dash_left', 'dash_right', 'dash_back'])
        for name, want in PRESSED.items():
            self.assertEqual(len(presses[name]), 1, name)
            heading = gt_log.dodge_expected_heading(presses[name][0]['ls'])
            self.assertAlmostEqual(abs(gt_log._wrap180(heading - want)), 0.0, places=6, msg=name)
        self.assertAlmostEqual(presses['dash_left'][0]['t'], 0.35, delta=0.02)
        self.assertAlmostEqual(presses['dash_fwd'][0]['t'], 0.3, delta=0.02)

    def test_a_stick_at_rest_holds_no_direction(self):
        self.assertIsNone(gt_log.dodge_expected_heading((0.0, 0.1)))
        self.assertEqual(gt_log.dodge_expected_heading((-1.0, 0.0)), 90.0)
        self.assertEqual(gt_log.dodge_expected_heading((1.0, 0.0)), -90.0)

    def test_every_short_b_tap_of_a_measured_step_is_flagged(self):
        # B is also the vehicle exit button (a long hold, no metrics): only a short tap of a measured step is a dodge
        for name, sc in self.scen.items():
            for st in sc.steps:
                for b in st.buttons:
                    if b.button == 'B' and b.hold <= 0.3 and st.metrics:
                        self.assertTrue(st.dodge, f'{name}.{st.name} taps B but is not flagged "dodge"')
        exit_steps = [st for sc in self.scen.values() for st in sc.steps if st.name == 'exit']
        self.assertTrue(exit_steps and not any(st.dodge for st in exit_steps))
        self.assertEqual(self.scen['kbd_taps'].dodge_presses(), {})

    def test_flag_needs_a_b_press(self):
        raw = {'steps': [{'name': 'x', 'duration': 1.0, 'dodge': True, 'buttons': [{'t': 0.1, 'button': 'X'}]}]}
        with self.assertRaises(gs.ScenarioError):
            gs.parse_scenario('bad', raw)


class FixedBuildTests(unittest.TestCase):
    def test_both_fixed_runs_pass(self):
        for path in (FIXED_A, FIXED_B):
            rows, _ = analyze(read(path), path)
            for name, want in PRESSED.items():
                r = rows[name]
                m = r['metrics']
                self.assertEqual(r['verdict'], 'PASS', (path, name, r['fails']))
                self.assertTrue(m['dodge_applicable'])
                self.assertEqual((m['dodge_presses'], m['dodge_pushes'], m['dodge_push_missing']), (1, 1, 0), name)
                self.assertEqual(m['dodge_served_dev_deg'], 0.0, name)
                self.assertEqual(m['dodge_err_deg'], 0.0, name)
                self.assertGreater(m['dodge_travel_m'], 1.5, name)

    def test_the_dodge_clip_is_counted_and_only_informs(self):
        rows, _ = analyze(read(FIXED_A), FIXED_A)
        self.assertEqual([rows[n]['metrics']['dodge_clip'] for n in PRESSED], ['none', 'none', 'B', 'none'])
        self.assertEqual([rows[n]['metrics']['dodge_no_clip'] for n in PRESSED], [1, 1, 0, 1])
        self.assertEqual(rows['dash_left']['info'], ['no dodge clip (1 of 1 dodges played none)'])
        self.assertEqual(rows['dash_right']['info'], [])
        for r in rows.values():
            self.assertEqual(r['fails'], [])

    def test_the_default_thresholds_pass_the_first_run(self):
        # all the step limits at once (the first run is clean in every other metric too)
        rows, _ = analyze(read(FIXED_A), FIXED_A, gt_log.DEFAULT_THRESHOLDS)
        for n in PRESSED:
            self.assertFalse([f for f in rows[n]['fails'] if 'dodge' in f], (n, rows[n]['fails']))

    def test_steps_without_a_dodge_carry_no_dodge_values(self):
        scen = gs.load_scenarios(os.path.join(HERE, 'scenarios.json'))
        lines = gt_log.parse_text(read(FIXED_A))
        plan = {'runs': [{'scenario': 'sprint_dash', 'run': 1, 'aborted': None, 'steps': [
            {'name': 'sprint_again', 'start': _t('2026-11-14 17:14:{s}', '49.475'), 'end': _t('2026-11-14 17:14:{s}', '52.475'),
             'metrics': True}]}]}
        row = gt_run.analyze(plan, lines, DODGE_THRESHOLDS, scen)[0]
        self.assertNotIn('dodge_presses', row['metrics'])
        self.assertEqual((row['fails'], row['info']), ([], []))


class FailingDodgeTests(unittest.TestCase):
    """Synthetic edits of the first fixture (the fixture itself is a real log)."""

    def test_a_travel_78_degrees_off_the_press_fails_plainly(self):
        text = replace_once(read(FIXED_A), 'travel=-64.9/2.27m err=0.0', 'travel=13.0/2.27m err=78.0')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['verdict'], 'FAIL')
        self.assertEqual(rows['dash_left']['fails'], ['dodge went 78 deg off the pressed direction (limit 15)'])
        self.assertEqual(rows['dash_left']['metrics']['dodge_err_deg'], 78.0)
        for n in ('dash_fwd', 'dash_right', 'dash_back'):
            self.assertEqual(rows[n]['verdict'], 'PASS', n)

    def test_a_negative_error_counts_by_its_size(self):
        text = replace_once(read(FIXED_A), 'travel=-64.9/2.27m err=0.0', 'travel=-143.0/2.27m err=-78.0')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['fails'], ['dodge went 78 deg off the pressed direction (limit 15)'])

    def test_a_wrong_served_heading_fails(self):
        # dash_left presses the stick to the left (+90) and the game was served -90
        text = replace_once(read(FIXED_A), 'served=90.0 engine=90.0', 'served=-90.0 engine=-90.0')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['fails'],
                         ['the game was asked for a dodge direction 180 deg off the pressed one (limit 20)'])

    def test_served_within_the_tolerance_passes(self):
        text = replace_once(read(FIXED_A), 'served=90.0 engine=90.0', 'served=75.0 engine=75.0')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['fails'], [])
        self.assertEqual(rows['dash_left']['metrics']['dodge_served_dev_deg'], 15.0)
        text = replace_once(read(FIXED_A), 'served=90.0 engine=90.0', 'served=60.0 engine=60.0')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['fails'],
                         ['the game was asked for a dodge direction 30 deg off the pressed one (limit 20)'])

    def test_back_is_the_same_heading_at_plus_and_minus_180(self):
        for served in ('180.0', '-180.0'):
            text = replace_once(read(FIXED_A), 'served=180.0 engine=-180.0', f'served={served} engine={served}')
            rows, _ = analyze(text, FIXED_A)
            self.assertEqual(rows['dash_back']['metrics']['dodge_served_dev_deg'], 0.0, served)

    def test_too_short_a_travel_fails(self):
        text = replace_once(read(FIXED_A), 'travel=-64.9/2.27m', 'travel=-64.9/0.31m')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['fails'], ['dodge moved only 0.31 m (needs at least 0.50)'])

    def test_a_missing_push_line_fails_on_a_build_that_writes_them(self):
        lines = read(FIXED_A).splitlines()
        keep = [ln for ln in lines if not ('MOTIONDODGE push' in ln and 'served=90.0' in ln)]
        self.assertEqual(len(keep), len(lines) - 1)
        rows, _ = analyze('\n'.join(keep) + '\n', FIXED_A)
        self.assertEqual(rows['dash_left']['verdict'], 'FAIL')
        self.assertEqual(rows['dash_left']['fails'], ['dodge pressed but the plugin logged no push line for it'])
        self.assertEqual(rows['dash_left']['metrics']['dodge_push_missing'], 1)
        for n in ('dash_fwd', 'dash_right', 'dash_back'):
            self.assertEqual(rows[n]['verdict'], 'PASS', n)

    def test_a_push_line_of_the_next_step_is_not_taken(self):
        # the dash_left push line moved into the dash_right window: dash_right takes its own, dash_left finds none
        text = replace_once(read(FIXED_A), '[2026-11-14 17:14:44.233]', '[2026-11-14 17:14:47.300]')
        rows, _ = analyze(text, FIXED_A)
        self.assertEqual(rows['dash_left']['metrics']['dodge_push_missing'], 1)
        self.assertEqual(rows['dash_right']['metrics']['dodge_pushes'], 1)
        self.assertEqual(rows['dash_right']['metrics']['dodge_served_dev_deg'], 0.0)

    def test_a_push_line_without_served_fails_and_is_checked_by_its_travel(self):
        text = replace_once(
            read(FIXED_A),
            'served=90.0 engine=90.0 rw=1 want=-65.0 travel=-64.9/2.27m err=0.0 facing=-86.8 pick=F clip=none rel=18.9',
            'served=- travel=-64.9/2.27m facing=-86.8 pick=F clip=none rel=18.9')
        rows, _ = analyze(text, FIXED_A)
        m = rows['dash_left']['metrics']
        self.assertEqual(rows['dash_left']['fails'], ['the dodge push line carries no served heading (the heading probe was not fresh)'])
        self.assertEqual(m['dodge_err_deg'], 0.1)          # travel -64.9 against camera -155 + pressed 90 = -65
        text = text.replace('travel=-64.9/2.27m', 'travel=20.0/2.27m')
        rows, _ = analyze(text, FIXED_A)
        self.assertIn('dodge went 85 deg off the pressed direction (limit 15)', rows['dash_left']['fails'])


class OlderBuildTests(unittest.TestCase):
    def test_no_push_line_in_the_log_is_not_applicable(self):
        rows, _ = analyze(read(OLD_BUILD), OLD_BUILD)
        for n in PRESSED:
            r = rows[n]
            m = r['metrics']
            self.assertEqual(r['verdict'], 'PASS', (n, r['fails']))
            self.assertFalse(m['dodge_applicable'])
            self.assertTrue(m['dodge_note'].startswith('n/a: no MOTIONDODGE push line anywhere in the log'))
            self.assertIsNone(m['dodge_push_missing'])
            self.assertIsNone(m['dodge_served_dev_deg'])
            self.assertEqual(m['dodge_presses'], 1)
            self.assertEqual(r['info'], [])

    def test_table_and_files_report_not_applicable(self):
        rows, lines = analyze(read(OLD_BUILD), OLD_BUILD)
        out = io.StringIO()
        summary = {'overall': 'PASS', 'counts': {}, 'aborted_runs': [], 'mm_precondition': None}
        with contextlib.redirect_stdout(out):
            gt_run.print_table(list(rows.values()), summary)
        table = out.getvalue().splitlines()
        self.assertIn('dSrv', table[0])
        row = next(ln for ln in table if ' dash_left ' in ln)
        self.assertGreaterEqual(row.count('n/a'), 4)        # dSrv dErr dTrv dClip (the v1 columns are n/a under MM v2 as before)


class NightSummaryTests(unittest.TestCase):
    def test_direction_and_travel_failures_are_metrics_and_a_missing_line_is_mechanics(self):
        import gt_night as gn
        text = replace_once(read(FIXED_A), 'travel=-64.9/2.27m err=0.0', 'travel=13.0/2.27m err=78.0')
        text = replace_once(text, 'served=180.0 engine=-180.0', 'served=120.0 engine=120.0')
        text = '\n'.join(ln for ln in text.splitlines() if not ('MOTIONDODGE push' in ln and 'served=90.0' in ln))
        rows, _ = analyze(text + '\n', FIXED_A)
        steps = [{'scenario': 'sprint_dash', 'run': 1, 'step': n, 'fails': r['fails']} for n, r in rows.items()]
        mech, metr = gn.split_failures(steps, [])
        self.assertEqual(len(mech), 1, mech)
        self.assertIn('no push line', mech[0])
        self.assertEqual(len(metr), 1, metr)
        self.assertIn('deg off the pressed one', metr[0])
        text2 = replace_once(read(FIXED_A), 'travel=-64.9/2.27m err=0.0', 'travel=13.0/0.31m err=78.0')
        rows, _ = analyze(text2, FIXED_A)
        mech, metr = gn.split_failures([{'scenario': 's', 'run': 1, 'step': 'dash_left', 'fails': rows['dash_left']['fails']}], [])
        self.assertEqual((len(mech), len(metr)), (0, 2))


class OutputTests(unittest.TestCase):
    def test_table_files_and_csv_of_a_fixed_run(self):
        rows, lines = analyze(read(FIXED_A), FIXED_A)
        out = io.StringIO()
        summary = {'overall': 'PASS', 'counts': {}, 'aborted_runs': [], 'mm_precondition': None}
        with contextlib.redirect_stdout(out):
            gt_run.print_table(list(rows.values()), summary)
        table = out.getvalue().splitlines()
        head = table[0]
        for col in ('dSrv', 'dErr', 'dTrv', 'dClip'):
            self.assertIn(col, head)
        left = next(ln for ln in table if ' dash_left ' in ln)
        self.assertIn('~ no dodge clip (1 of 1 dodges played none)', left)
        self.assertNotIn('!!', left)
        # the columns line up with the header: served 0.0, error 0.0, travel 2.27, clip none
        cols = {n: head.index(n) for n in ('dSrv', 'dErr', 'dTrv', 'dClip')}
        for name, i in cols.items():
            self.assertTrue(left[:i + len(name)].split()[-1] in ('0.0', '2.27', 'none'), (name, left))
        with tempfile.TemporaryDirectory() as d:
            gt_run.write_outputs(d, plan_of(FIXED_A), list(rows.values()), lines, FIXED_A)
            with open(os.path.join(d, 'sprint_dash_r1.txt'), encoding='utf-8') as f:
                txt = f.read()
            with open(os.path.join(d, 'summary.csv'), encoding='utf-8') as f:
                csv_head = f.readline()
        self.assertIn('dodge_served_dev_deg=0.0', txt)
        self.assertIn('dodge_travel_m=2.27', txt)
        self.assertIn('info: no dodge clip (1 of 1 dodges played none)', txt)
        self.assertIn('DODGE push t=17.744', txt)
        self.assertIn('dodge_err_deg', csv_head)

    def test_the_default_thresholds_and_the_file_agree_on_the_dodge_limits(self):
        thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        for k, v in DODGE_THRESHOLDS.items():
            self.assertEqual(thr[k], v, k)
        self.assertEqual({k: v['max'] for k, v in DODGE_THRESHOLDS.items() if 'max' in v},
                         {'dodge_served_dev_deg': 20.0, 'dodge_err_deg': 15.0, 'dodge_push_missing': 0, 'dodge_served_missing': 0})
        self.assertEqual(DODGE_THRESHOLDS['dodge_travel_m']['min'], 0.5)

    def test_a_limit_without_text_keeps_the_old_wording(self):
        self.assertEqual(gt_log.evaluate({'a': 5.0}, {'a': {'max': 3.0}}), ['a=5.0 > 3.0'])
        self.assertEqual(gt_log.evaluate({'a': 1.0}, {'a': {'min': 3.0, 'msg': 'too low'}}), ['too low: a=1.0 < 3.0'])


if __name__ == '__main__':
    unittest.main()
