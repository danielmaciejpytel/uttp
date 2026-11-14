"""Tests of helper_coverage_report.py on a synthetic log in the line format of the DLL (Main.cpp LogHelperCoverageIfDue).

  python -I test_helper_coverage_report.py
"""
import io
import json
import os
import pathlib
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from datetime import datetime, timedelta

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import helper_coverage_report as hc  # noqa: E402

WRITERS = ('driver', 'host', 'vehtrans', 'neck', 'vault', 'vehguard', 'none')
T0 = datetime(2026, 11, 14, 12, 0, 0)


def line(sec: float, calls: int, w: dict, c: dict, o: dict | None = None, state: str = 'src=host w=1.00 body=0 fallback=none skip=none detail=0'):
    """One coverage line exactly as the DLL writes it (cumulative counts)."""
    ts = (T0 + timedelta(seconds=sec)).strftime('%Y-%m-%d %H:%M:%S.%f')[:23]
    ws = ' '.join(f'{k}={w.get(k, 0)}' for k in WRITERS)
    cs = ' '.join(f'{k}={c.get(k, 0)}' for k in WRITERS)
    os_ = ''.join(f' o.{k}[' + ' '.join(f'{a}={b}' for a, b in v.items()) + ']' for k, v in (o or {}).items())
    return (f'[{ts}] [info    ] [ 60760] [UTPP] UTPP skin helpers coverage: calls={calls} w[{ws}] c[{cs}]{os_} state[{state}]')


def fixture_lines():
    """0-20 s: driver fully covered; 20-30 s: fists on the host pose with every host frame refused for 'body';
    30-40 s: host covered again; a neck guard at weight 0 (never covered) all along from 10 s."""
    L = []
    L.append(line(1, 60, {'driver': 60}, {'driver': 60}))
    L.append(line(10, 600, {'driver': 600}, {'driver': 600}))
    L.append(line(19, 1140, {'driver': 1140, 'neck': 30}, {'driver': 1140}, {'neck': {'gate': 30}}))
    L.append(line(25, 1500, {'driver': 1140, 'host': 360, 'neck': 60}, {'driver': 1140}, {'host': {'body': 360}, 'neck': {'gate': 60}},
                  'src=host w=1.00 body=-1 fallback=none skip=overlay-release detail=0'))
    L.append(line(29, 1740, {'driver': 1140, 'host': 600, 'neck': 90}, {'driver': 1140}, {'host': {'body': 600}, 'neck': {'gate': 90}},
                  'src=host w=1.00 body=-1 fallback=none skip=overlay-release detail=0'))
    L.append(line(35, 2100, {'driver': 1140, 'host': 960, 'neck': 120}, {'driver': 1140, 'host': 360}, {'host': {'body': 600}, 'neck': {'gate': 120}}))
    L.append(line(39, 2340, {'driver': 1140, 'host': 1200, 'neck': 150}, {'driver': 1140, 'host': 600}, {'host': {'body': 600}, 'neck': {'gate': 150}}))
    return L


def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)


class ParseTest(unittest.TestCase):
    def test_parse_and_deltas(self):
        text = '\n'.join(['[2026-11-14 12:00:00.000] [info    ] [1] [UTPP] UTPP skin helpers counters (tpp): evaluated=2 rejected=2'] +
                         fixture_lines())
        lines = hc.parse_lines(text)
        self.assertEqual(len(lines), 7)                       # the older counter line is not a coverage line
        self.assertEqual(lines[3]['o']['host'], {'body': 360})
        self.assertEqual(lines[3]['state']['skip'], 'overlay-release')
        ds = hc.deltas(lines)
        self.assertEqual(ds[0]['calls'], 60)                  # the first line counts from zero
        self.assertEqual(ds[3]['w']['host'], 360)
        self.assertEqual(ds[3]['c'].get('host', 0), 0)
        self.assertEqual(ds[5]['c']['host'], 360)
        self.assertEqual(ds[5]['o']['host']['body'], 0)

    def test_counter_reset(self):
        L = fixture_lines()[:2] + [line(12, 30, {'host': 30}, {'host': 30})]
        ds = hc.deltas(hc.parse_lines('\n'.join(L)))
        self.assertTrue(ds[2]['reset'])
        self.assertEqual(ds[2]['calls'], 30)
        self.assertEqual(ds[2]['w'], {'host': 30} | {k: 0 for k in WRITERS if k != 'host'})


class ReportTest(unittest.TestCase):
    def run_main(self, args):
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = hc.main(args)
        return code, buf.getvalue()

    def write(self, d, lines):
        p = os.path.join(d, 'utpp-2026-11-14-12-00-00.log')
        with open(p, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
        return p

    def test_fixed_windows(self):
        with tempfile.TemporaryDirectory() as d:
            log = self.write(d, fixture_lines())
            js = os.path.join(d, 'rep.json')
            code, out = self.run_main([log, '--window', '10', '--json', js])
            self.assertEqual(code, 1, out)
            rep = load(js)
            # windows from the first line (1 s): [1,11) [11,21) [21,31) [31,41)
            self.assertEqual(len(rep['windows']), 4)
            self.assertEqual(rep['under_limit'], [rep['windows'][2]['window']])
            w3 = rep['windows'][2]
            self.assertEqual(w3['writers']['host']['written'], 600)
            self.assertEqual(w3['writers']['host']['committed'], 0)
            self.assertEqual(w3['writers']['host']['outcomes'], {'body': 600})
            self.assertEqual(w3['under_limit'], ['host'])
            self.assertEqual(rep['windows'][3]['writers']['host']['coverage'], 100.0)
            # the weight-0 writer is reported but not held to the limit by default
            self.assertEqual(rep['total']['writers']['neck']['committed'], 0)
            self.assertNotIn('neck', w3['under_limit'])
            self.assertIn('UNDER host', out)
            self.assertIn('body=600', out)

    def test_writers_and_limit(self):
        with tempfile.TemporaryDirectory() as d:
            log = self.write(d, fixture_lines())
            code, _ = self.run_main([log, '--writers', 'driver'])
            self.assertEqual(code, 0)                         # the driver is fully covered everywhere
            code, out = self.run_main([log, '--writers', 'driver,neck'])
            self.assertEqual(code, 1)
            self.assertIn('UNDER neck', out)
            code, _ = self.run_main([log, '--since', '12:00:30'])
            self.assertEqual(code, 0)                         # after the refused span
            code, _ = self.run_main([log, '--limit', '0'])
            self.assertEqual(code, 0)
            self.assertEqual(self.run_main([log, '--writers', 'bogus'])[0], 2)

    def test_plan_windows(self):
        fmt = '%Y-%m-%d %H:%M:%S.%f'
        plan = {'runs': [
            {'scenario': 'walk', 'run': 1, 'steps': [
                {'name': 'a', 'start': (T0 + timedelta(seconds=2)).strftime(fmt), 'end': (T0 + timedelta(seconds=10)).strftime(fmt)},
                {'name': 'b', 'start': (T0 + timedelta(seconds=10)).strftime(fmt), 'end': (T0 + timedelta(seconds=20)).strftime(fmt)}]},
            {'scenario': 'fists', 'run': 1, 'steps': [
                {'name': 'guard', 'start': (T0 + timedelta(seconds=20)).strftime(fmt), 'end': (T0 + timedelta(seconds=30)).strftime(fmt)},
                {'name': 'unused'}]}]}
        with tempfile.TemporaryDirectory() as d:
            log = self.write(d, fixture_lines())
            pp = os.path.join(d, 'plan.json')
            with open(pp, 'w', encoding='utf-8') as f:
                json.dump(plan, f)
            js = os.path.join(d, 'rep.json')
            code, out = self.run_main([log, '--plan', pp, '--json', js])
            self.assertEqual(code, 1, out)
            rep = load(js)
            self.assertEqual([w['window'] for w in rep['windows']], ['walk#1', 'fists#1'])
            self.assertEqual(rep['under_limit'], ['fists#1'])
            self.assertEqual(rep['windows'][0]['writers']['driver']['coverage'], 100.0)
            self.assertEqual(rep['outside_windows']['lines'], 3)   # the lines at 1 s, 35 s and 39 s
            # steps and a clock offset that moves every window 10 s later
            code, _ = self.run_main([log, '--plan', pp, '--steps', '--clock-offset', '10', '--json', js])
            rep = load(js)
            self.assertEqual([w['window'] for w in rep['windows']], ['walk#1/a', 'walk#1/b', 'fists#1/guard'])
            self.assertEqual(rep['windows'][1]['under_limit'], ['host'])   # 20-30 s
            self.assertEqual(rep['windows'][2]['under_limit'], [])       # 30-40 s: the 600 host calls all committed
            self.assertEqual(rep['windows'][2]['writers']['host']['written'], 600)
            self.assertEqual(code, 1)

    def test_no_lines(self):
        with tempfile.TemporaryDirectory() as d:
            log = self.write(d, ['[2026-11-14 12:00:00.000] [info    ] [1] [UTPP] something else'])
            self.assertEqual(self.run_main([log])[0], 2)


if __name__ == '__main__':
    unittest.main()
