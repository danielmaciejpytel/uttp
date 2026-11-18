"""Tests of traversal_coverage.py on synthetic log files (no game logs).

  python -I test_traversal_coverage.py
"""
import json
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import traversal_coverage as tc  # noqa: E402

LOG = """[2026-11-15 02:47:50.025] [info    ] [ 59592] [UTPP] UTPP trav: action=8 h=1.11 ext=0.88 depth=0.86 back=1.11 v=0.0 kind=1 decision=Mantle result=Ok(-) montage=AM_M_Neutral_Traversal_Mantle_1_0_stand_F_Lfoot
[2026-11-15 02:47:52.000] [info    ] [ 59592] [UTPP] UTPP trav: released to game animation (detail=8)
[2026-11-15 02:48:11.327] [info    ] [ 54588] [UTPP] UTPP trav: action=8 h=1.21 ext=0.96 depth=0.93 back=0.02 v=4.0 kind=0 decision=Mantle result=Rejected(warp) montage=AM_M_Neutral_Traversal_Mantle_1_0_run_F_Rfoot
[2026-11-15 02:49:00.100] [info    ] [ 54588] [UTPP] UTPP trav: action=9 h=1.00 ext=0.30 depth=0.30 back=1.00 v=5.0 kind=1 decision=Hurdle result=Unsupported(vehicle) montage=
[2026-11-15 02:49:30.100] [info    ] [ 54588] [UTPP] UTPP trav: vault shoulder limiter off (applied=6 saturated=0 minWidth=30.1 maxShift=1.6 cm)
"""
# test-run copy: the same first plan line, time of day only and cut short
COPY = """  02:47:50.025 UTPP  UTPP trav: action=8 h=1.11 ext=0.88 depth=0.86 back=1.11 v=0.0 kind=1 decision=Mantle result=Ok(-) montage=AM_M_Neutral_Traversal_Mantle_1_0
  03:10:00.000 UTPP  UTPP trav: action=9 h=0.90 ext=0.20 depth=0.20 back=0.90 v=4.5 kind=0 decision=Hurdle result=Ok(-) montage=AM_M_Neutral_Traversal_Hurdle_1_0_run_F_Lfoot
"""


class CoverageTest(unittest.TestCase):
    def run_on(self, files):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            for name, text in files.items():
                p = root / name
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(text, encoding='utf-8', newline='\n')
            (root / 'other.bin').write_bytes(b'UTPP trav: action=9 kind=0')  # not a .log / .txt: ignored
            plans, releases, sources = tc.collect([root], jobs=2)
            out = root / 'cov.json'
            tc.main([str(root), '--out', str(out), '--jobs', '2'])
            return plans, releases, sources, json.loads(out.read_text(encoding='utf-8'))

    def test_dedupe_and_acceptance(self):
        plans, releases, sources, s = self.run_on({'game/utpp-a.log': LOG, 'night/run/gametest/trav_r1.txt': COPY})
        self.assertEqual(len(plans), 4)  # 3 in the log + 1 new in the copy; the cut copy of the first line is the same plan
        first = [p for p in plans if p['time'] == '02:47:50.025'][0]
        self.assertTrue(first['montage'].endswith('_Lfoot'))  # the full line wins over the cut copy
        self.assertEqual(first['file'], 'game/utpp-a.log')  # relative to the root, no machine path
        self.assertEqual(releases, [8])
        k = s['by_action_kind']
        self.assertEqual(k['climb/vehicle'], {'plans': 1, 'ok': 1, 'acceptance': 1.0, 'results': {'Ok(-)': 1},
                                              'montages': {'AM_M_Neutral_Traversal_Mantle_1_0_stand_F_Lfoot': 1}})
        self.assertEqual(k['climb/static']['results'], {'Rejected(warp)': 1})
        self.assertEqual(k['climb/static']['acceptance'], 0.0)
        self.assertEqual(k['vault/vehicle']['results'], {'Unsupported(vehicle)': 1})
        self.assertEqual(k['vault/static']['ok'], 1)
        never = [n['pair'] for n in s['never_exercised']]
        self.assertEqual(never, ['climb/entity', 'vault/entity'])
        self.assertEqual(s['ok_montage_foot'], {'L': 2})

    def test_never_exercised_static_vault(self):
        _p, _r, _s, s = self.run_on({'game/utpp-a.log': LOG})
        pairs = {n['pair']: n['needs'] for n in s['never_exercised']}
        self.assertIn('vault/static', pairs)
        self.assertIn('sprint', pairs['vault/static'])
        self.assertNotIn('climb/static', pairs)


if __name__ == '__main__':
    unittest.main()
