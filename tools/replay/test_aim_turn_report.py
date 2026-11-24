"""Offline tests of aim_turn_report.py (turns in place while aiming).

  python Source\\tools\\replay\\test_aim_turn_report.py -v

Builds a fake session folder (feed dump with camera / entity yaw, plugin log with MOTION lines, one step file) in a temp dir;
nothing outside it is touched.
"""
import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import aim_turn_report as atr  # noqa: E402
import speed_calibration as scal  # noqa: E402

START = datetime(2026, 1, 2, 10, 0, 0)
DT = 1.0 / 30.0


def wall_text(epoch):
    d = datetime.fromtimestamp(epoch)
    return d.strftime('%Y-%m-%d %H:%M:%S.') + '%03d' % (d.microsecond // 1000)


def feed_row(t, cam, yaw, x=0.0, upper=6):
    cols = ['%.6f' % t, '%.4f' % DT, '0', '0', '0', '0', '0', '%.3f' % yaw, '%.3f' % cam, '1', '0', '0', '-', '-', '-', '-', '-', '0',
            '0', '%.5f' % x, '0.00000', '0', '0', '0', '0', str(upper), '-', '1', '-', '-', '0']
    assert len(cols) == scal.FEED_COLS
    return ' '.join(cols)


def motion_line(epoch, clip):
    return ('[%s] [info    ] [ 1] [UTPP] UTPP MOTION t=1.000 db=11:PSD_Dense_Stand_Idles clip=3:%s ct=0.100 cost=0.005 '
            'blend=0.20(live) rot=orient start=0 pivot=0 inYaw=0.0 velYaw=0.0 target=0 ov=1/1/00 w=1:1.00 '
            'lay=spine0.00,armL0.00,armR0.00,legs0.00,head0.00,pelvis0.00 aimW=1.00 body=woman bundle=woman chg'
            % (wall_text(epoch), clip))


class AimTurns(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.run = os.path.join(self.tmp, '20260102-100000')
        os.makedirs(os.path.join(self.run, 'logs'))
        os.makedirs(os.path.join(self.run, 'gametest'))
        t0 = START.timestamp()
        # 0-3 s: camera turns +60 deg in 0.5 s from t=0.5, the body follows by 2.0 s; 3-6 s: camera -100 deg, the body stays 40 deg behind
        rows = []
        for k in range(int(6.0 / DT) + 1):
            t = k * DT
            if t < 3.0:
                cam = 60.0 * min(1.0, max(0.0, (t - 0.5) / 0.5))
                yaw = 60.0 * min(1.0, max(0.0, (t - 0.8) / 1.2))
            else:
                cam = 60.0 - 100.0 * min(1.0, max(0.0, (t - 3.5) / 0.8))
                yaw = 60.0 - 60.0 * min(1.0, max(0.0, (t - 3.8) / 1.0))
            rows.append(feed_row(t, cam, yaw, x=0.001 * k if t >= 3.0 else 0.0))
        with open(os.path.join(self.run, 'logs', 'utpp-feed-2026-01-02-10-00-00.txt'), 'w') as f:
            f.write('# cols t dt ...\n' + '\n'.join(rows) + '\n')
        with open(os.path.join(self.run, 'logs', 'utpp-2026-01-02-10-00-00.log'), 'w') as f:
            f.write(motion_line(t0 + 0.7, 'M_Pistol_Stand_Turn_060_R') + '\n')
            f.write(motion_line(t0 + 2.5, 'M_Pistol_Stand_Idle_Loop') + '\n')
            f.write(motion_line(t0 + 4.0, 'M_Pistol_Stand_Idle_Loop') + '\n')
        with open(os.path.join(self.run, 'gametest', 'rf_handgun_aim_turns_r1.txt'), 'w') as f:
            f.write('=== step aim  %s -> %s  PASS\n' % (wall_text(t0 - 1.0), wall_text(t0)[11:]))
            f.write('=== step turn_060_r  %s -> %s  PASS\n' % (wall_text(t0), wall_text(t0 + 3.0)[11:]))
            f.write('=== step turn_090_l  %s -> %s  PASS\n' % (wall_text(t0 + 3.0), wall_text(t0 + 6.0)[11:]))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_turns_are_measured_and_classed(self):
        res = atr.report([self.run])
        turns = {t['step']: t for t in res['turns']}
        self.assertEqual(sorted(turns), ['turn_060_r', 'turn_090_l'])     # the 'aim' step is not a turn
        r, l = turns['turn_060_r'], turns['turn_090_l']
        self.assertAlmostEqual(r['pan_deg'], 60.0, delta=0.5)
        self.assertAlmostEqual(l['pan_deg'], -100.0, delta=0.5)
        self.assertEqual((r['class'], l['class']), ('60', '90'))           # the measured angle decides, not the step name
        self.assertTrue(r['tip'])
        self.assertEqual(r['tip_clip'], 'M_Pistol_Stand_Turn_060_R')
        self.assertAlmostEqual(r['tip_delay_s'], 0.7, delta=0.01)
        self.assertFalse(l['tip'])
        self.assertLess(r['lag_end_deg'], 0.5)
        self.assertAlmostEqual(l['lag_end_deg'], 40.0, delta=0.5)
        self.assertGreater(l['capsule_cm'], 5.0)                          # the capsule moved during the second turn
        self.assertEqual(r['aim_fraction'], 1.0)
        s = res['summary']
        self.assertEqual(s['at_least_45']['n'], 2)
        self.assertAlmostEqual(s['at_least_45']['tip_share'], 0.5)
        self.assertEqual(s['unaligned_runs'], ['20260102-100000'])         # no POS lines: reported, not hidden

    def test_a_turn_in_a_rotated_log_part_is_found(self):
        # the plugin log rotated at 10 MB: the turn clip is only in the older part .1.log (.2 is older still, name order .1, .2, current)
        logs = os.path.join(self.run, 'logs')
        cur = os.path.join(logs, 'utpp-2026-01-02-10-00-00.log')
        with open(cur, encoding='utf-8') as f:
            lines = f.read().splitlines()
        t0 = START.timestamp()
        with open(os.path.join(logs, 'utpp-2026-01-02-10-00-00.2.log'), 'w') as f:
            f.write(motion_line(t0 - 5.0, 'M_Pistol_Stand_Idle_Loop') + '\n')
        with open(os.path.join(logs, 'utpp-2026-01-02-10-00-00.1.log'), 'w') as f:
            f.write(lines[0] + '\n')
        with open(cur, 'w') as f:
            f.write('\n'.join(lines[1:]) + '\n')
        turns = {t['step']: t for t in atr.report([self.run])['turns']}
        self.assertTrue(turns['turn_060_r']['tip'])
        self.assertEqual(turns['turn_060_r']['tip_clip'], 'M_Pistol_Stand_Turn_060_R')
        run = scal.Run(self.run)
        self.assertEqual([round(m['wall'] - t0, 1) for m in run.log['motion']], [-5.0, 0.7, 2.5, 4.0])   # time order

    def test_angle_classes(self):
        self.assertEqual([atr.angle_class(a) for a in (20, -44, 50, 70, -100, 181)], ['30', '45', '45', '60', '90', '180+'])


if __name__ == '__main__':
    unittest.main()
