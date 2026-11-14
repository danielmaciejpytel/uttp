"""Tests of melee_pose_log.py on synthetic mod log lines.

  python -I test_melee_pose_log.py
"""
import io
import os
import pathlib
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import melee_pose_log as ml  # noqa: E402


def line(sec, calls, written, guard_passed=0, busy=0, planted=0):
    h, m, s = 3, 5, sec
    return ('[2026-11-14 %02d:%02d:%06.3f] [info    ] [ 22452] [UTPP] UTPP MELEEPOSE calls=%d written=%d held=0 relocks=1 frozen=0 toBase=0 '
            'busy=%d noTable=0 noWeapon=0 border=%d tail=0 planted=%d steps=0 hipsLimited=0 guardCalls=%d guardReplaced=2 guardPassed=%d '
            'guardBusy=0 w=1.00 state=11\n' % (h, m, s, calls, written, busy, written, planted, calls, guard_passed))


class MeleePoseLogTest(unittest.TestCase):
    def write(self, lines):
        fd, path = tempfile.mkstemp(suffix='.log')
        os.close(fd)
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write('[2026-11-14 03:05:00.000] [info    ] [ 1] [UTPP] UTPP MMMELEEFULL on why=on\n')
            fh.writelines(lines)
        self.addCleanup(os.remove, path)
        return path

    def run_main(self, args):
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = ml.main(args)
        return code, buf.getvalue()

    def test_parse_keys(self):
        s = ml.parse([line(1.5, 60, 50, 1, 2, 30)])
        self.assertEqual(len(s), 1)
        t, v = s[0]
        self.assertAlmostEqual(t, 3 * 3600 + 5 * 60 + 1.5)
        self.assertEqual(v['calls'], 60)
        self.assertEqual(v['guardPassed'], 1)
        self.assertEqual(v['planted'], 30)
        self.assertEqual(v['state'], 11)

    def test_windows_sum_to_the_span(self):
        lines = [line(1.0 + k, 60 * (k + 1), 50 * (k + 1)) for k in range(25)]
        wins, resets = ml.windows(ml.parse(lines), 10.0)
        self.assertEqual(resets, [])
        self.assertEqual(len(wins), 3)
        self.assertEqual(sum(w['inc']['calls'] for w in wins), 60 * 24)
        self.assertEqual(wins[0]['inc']['written'], 50 * 9)
        self.assertEqual(wins[1]['inc']['written'], 50 * 10)

    def test_reload_restarts(self):
        lines = [line(1.0, 600, 500), line(2.0, 660, 550), line(3.0, 30, 20), line(4.0, 90, 70)]
        wins, resets = ml.windows(ml.parse(lines), 10.0)
        self.assertEqual(len(resets), 1)
        self.assertEqual(wins[0]['inc']['calls'], 60 + 30 + 60)

    def test_checks(self):
        path = self.write([line(1.0 + k, 60 * (k + 1), 50 * (k + 1), guard_passed=k, busy=k) for k in range(5)])
        code, out = self.run_main([path, '--min-written', '100', '--max-guard-passed', '10'])
        self.assertEqual(code, 0, out)
        self.assertIn('[PASS] written', out)
        code, out = self.run_main([path, '--max-guard-passed', '2'])
        self.assertEqual(code, 1, out)
        self.assertIn('[FAIL] guard passed', out)
        code, out = self.run_main([path, '--max-busy-share', '0.001'])
        self.assertEqual(code, 1, out)

    def test_no_lines(self):
        path = self.write([])
        code, out = self.run_main([path])
        self.assertEqual(code, 0)
        self.assertIn('no UTPP MELEEPOSE lines', out)
        code, _ = self.run_main([path, '--min-written', '1'])
        self.assertEqual(code, 1)


if __name__ == '__main__':
    unittest.main()
