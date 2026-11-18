"""Offline tests of speed_calibration.py (steady capsule speeds, leg / capsule ratios, stop overshoot).

  python Source\\tools\\replay\\test_speed_calibration.py -v

Builds a fake session folder (feed dump, plugin log with MOTION and POS lines) and a minimal motion database in temp dirs;
nothing outside them is touched.
"""
import math
import os
import shutil
import struct
import sys
import tempfile
import unittest
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import speed_calibration as sc  # noqa: E402

FPS = 30.0
START = datetime(2026, 1, 2, 10, 0, 0)
OFFSET = 0.4          # wall = feed file stamp + t + OFFSET


def write_db(path, clips):
    """Minimal version 2 database: header, section table, STRS, CLIP, ROOT. clips: [(name, loop, [(dx, dy) per frame])]."""
    strs = b''
    clip_recs = b''
    root = b''
    first = 0
    for name, loop, steps in clips:
        noff = len(strs)
        strs += name.encode() + b'\0'
        n = len(steps)
        clip_recs += struct.pack('<IIIIIIQ', noff, first, n, 1 if loop else 0, n - 1 if loop else 0, n, 0)
        for dx, dy in steps:
            root += struct.pack('<4f', dx, dy, 0.0, 0.0)
        first += n
    sections = [(b'STRS', strs), (b'CLIP', clip_recs), (b'ROOT', root)]
    table_off = 128
    data_off = table_off + 32 * len(sections)
    table = b''
    payload = b''
    for sid, blob in sections:
        table += struct.pack('<IIIIQQ', struct.unpack('<I', sid)[0], len(blob), 1, 0, data_off + len(payload), len(blob))
        payload += blob
    h = [b'UTPPMMDB', 2, 128, data_off + len(payload), 0, 0, first, 1, len(clips)] + [0] * 8 + [FPS, len(sections), table_off] \
        + [0] * 5 + [0, 0]
    with open(path, 'wb') as f:
        f.write(struct.pack(sc.ClipPaths.HEADER_FMT, *h))
        f.write(table)
        f.write(payload)


def loop_steps(speed, frames):
    """Loop clip of `frames` frames (period frames - 1) moving forward at `speed` m/s."""
    return [(0.0, speed / FPS)] * frames


def feed_row(t, vx, vy, mvx, mvy, loco, x, y, upper=0, gnd=1, walk=0):
    cols = ['%.6f' % t, '0.0166', '%.4f' % vx, '%.4f' % vy, '0', '%.4f' % mvx, '%.4f' % mvy, '0', '0', str(loco), '0', '0', '-', '-',
            '-', '-', '-', '0', '0', '%.5f' % x, '%.5f' % y, '0', '0', '0', '0', str(upper), '-', str(gnd), '-', '-', str(walk)]
    assert len(cols) == sc.FEED_COLS
    return ' '.join(cols)


def wall_text(epoch):
    d = datetime.fromtimestamp(epoch)
    return d.strftime('%Y-%m-%d %H:%M:%S.') + '%03d' % (d.microsecond // 1000)


def motion_line(epoch, t, clip, ct, chg):
    return ('[%s] [info    ] [ 1] [UTPP] UTPP MOTION t=%.3f db=15:PSD_Dense_Stand_Run_Loops clip=3:%s ct=%.3f cost=0.005 '
            'blend=0.50(live) rot=orient start=0 pivot=0 inYaw=0.0 velYaw=0.0 target=1 ov=0/0/01 w=1:1.00 '
            'lay=spine0.00,armL0.00,armR0.00,legs0.00,head0.00,pelvis0.00 aimW=0.00 body=woman bundle=woman%s'
            % (wall_text(epoch), t, clip, ct, ' chg' if chg else ''))


class SessionFixture:
    """A run: 1 s standing, 5 s running forward at 3 m/s (full stick), stick release, capsule stops in 0.2 s, 1 s standing.
    The run loop (2 m path per 0.5 s cycle... see RUN_SPEED) plays at RATE; the stop clip is picked at the release."""
    RUN_SPEED = 3.0
    CLIP_SPEED = 5.0
    RATE = 0.8
    LOOP_FRAMES = 31       # period 30 frames = 1.0 s
    STOP_FRAMES = 31       # 1.0 s, root moves 0.6 m over its first 0.5 s, then stands

    def __init__(self, root):
        self.folder = os.path.join(root, 'night', 'run1')
        logs = os.path.join(self.folder, 'logs')
        os.makedirs(logs)
        self.db = os.path.join(root, 'db.utppmm')
        stop_steps = [(0.0, 1.2 / FPS)] * 15 + [(0.0, 0.0)] * 16
        write_db(self.db, [('M_Neutral_Stand_Idle_Loop', True, [(0.0, 0.0)] * 31),
                           ('M_Neutral_Run_Loop_F', True, loop_steps(self.CLIP_SPEED, self.LOOP_FRAMES)),
                           ('M_Neutral_Run_Stop_F_Lfoot', False, stop_steps)])
        t0 = START.timestamp()
        rows, x, y = [], 0.0, 0.0
        dt = 1.0 / 60.0
        t = 0.0
        self.release_t = 6.0
        while t < 7.5:
            if 1.0 <= t < self.release_t:
                v, st = self.RUN_SPEED, 1.0
            elif self.release_t <= t < self.release_t + 0.2:
                v, st = self.RUN_SPEED * (1.0 - (t - self.release_t) / 0.2), 0.0
            else:
                v, st = 0.0, 0.0
            y += v * dt
            if st > 0.5:
                y_held = y                        # capsule position at the last row with the stick held
            rows.append(feed_row(t, 0.0, v, 0.0, st, 1, x, y))
            t += dt
        self.stop_distance = y - y_held
        name = 'utpp-feed-%s.txt' % START.strftime('%Y-%m-%d-%H-%M-%S')
        with open(os.path.join(logs, name), 'w', encoding='utf-8') as f:
            f.write('# cols t dt vx vy vz ...\n' + '\n'.join(rows) + '\n')
        lines = []
        ypos = lambda tt: self.RUN_SPEED * min(max(tt - 1.0, 0.0), self.release_t - 1.0)  # noqa: E731
        for k in range(12):                       # POS lines while running (2 Hz)
            tt = 1.5 + 0.4 * k
            lines.append('[%s] [info    ] [ 1] [UTPP] UTPP CET [night] POS p=0.00,%.3f,0.00 cf=0,1 v=0,0,0,0,0 src=none w=0 m=0 n=%d'
                         % (wall_text(t0 + tt + OFFSET), ypos(tt), k))
        ct = 0.0
        tt = 1.0
        lines.append(motion_line(t0 + tt + OFFSET, tt, 'M_Neutral_Run_Loop_F', 0.0, True))
        while tt + 0.5 < self.release_t:
            tt += 0.5
            ct = (ct + 0.5 * self.RATE) % 1.0
            lines.append(motion_line(t0 + tt + OFFSET, tt, 'M_Neutral_Run_Loop_F', ct, False))
        lines.append(motion_line(t0 + self.release_t + 0.003 + OFFSET, self.release_t + 0.003, 'M_Neutral_Run_Stop_F_Lfoot', 0.25, True))
        with open(os.path.join(logs, 'utpp-%s.log' % START.strftime('%Y-%m-%d-%H-%M-%S')), 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')


class ClassifyTest(unittest.TestCase):
    def test_sector(self):
        self.assertEqual(sc.sector(0.0, 1.0), 'F')
        self.assertEqual(sc.sector(0.7, 0.7), 'FD')
        self.assertEqual(sc.sector(-1.0, 0.0), 'S')
        self.assertEqual(sc.sector(0.7, -0.7), 'BD')
        self.assertEqual(sc.sector(0.0, -1.0), 'B')

    def test_classify(self):
        self.assertEqual(sc.classify(1, 0, 0, 1.0), ('free', 'stand_run'))
        self.assertEqual(sc.classify(1, 0, 0, 0.3), ('free', 'stand_walk'))
        self.assertEqual(sc.classify(1, 0, 1, 1.0), ('walk_toggle', 'stand_run'))
        self.assertEqual(sc.classify(3, 0, 0, 1.0), ('free', 'sprint'))
        self.assertEqual(sc.classify(0, 6, 0, 0.27), ('aim', 'stand_walk'))   # aiming feeds loco 0
        self.assertEqual(sc.classify(2, 6, 0, 0.27), ('aim', 'crouch'))
        self.assertIsNone(sc.classify(0, 0, 0, 1.0))


class ClipPathTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, 'db.utppmm')
        write_db(self.path, [('loop', True, loop_steps(3.0, 31)), ('once', False, [(0.3, 0.4)] * 11)])

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def test_loop_distance_and_unwrap(self):
        db = sc.ClipPaths(self.path)
        c = db.get('loop')
        self.assertTrue(c['loop'])
        self.assertAlmostEqual(c['length'], 1.0, places=5)
        self.assertAlmostEqual(db.mean_speed(c), 3.0, places=4)
        self.assertAlmostEqual(db.distance(c, 0.5), 1.5, places=4)
        self.assertAlmostEqual(db.distance(c, 2.25), 6.75, places=4)       # past the loop length: continues
        ct2 = sc.unwrap_ct(c, 0.8, 0.2)                                     # wrapped once
        self.assertAlmostEqual(ct2, 1.2, places=6)
        self.assertAlmostEqual(db.distance(c, ct2) - db.distance(c, 0.8), 1.2, places=4)

    def test_non_loop(self):
        db = sc.ClipPaths(self.path)
        c = db.get('once')
        self.assertFalse(c['loop'])
        self.assertAlmostEqual(db.distance(c, 10.0), 10 * 0.5, places=4)    # clamped to the end (10 steps of 0.5 m)
        self.assertIsNone(sc.unwrap_ct(c, 0.3, 0.1))


class SessionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        cls.fx = SessionFixture(cls.tmp)
        cls.session = sc.Run(cls.fx.folder)
        cls.session.body = 'female'
        cls.dbs = {'female': sc.ClipPaths(cls.fx.db)}

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp)

    def test_alignment(self):
        self.assertTrue(self.session.aligned)
        self.assertAlmostEqual(self.session.offset, OFFSET, delta=0.02)

    def test_steady_window(self):
        f = self.session.feed
        t = f['t']
        steady_t = t[self.session.steady]
        self.assertGreater(len(steady_t), 100)
        self.assertGreaterEqual(steady_t.min(), 1.0 + sc.SETTLE_S - 1e-6)   # nothing before the settle time
        self.assertLess(steady_t.max(), self.fx.release_t)                   # nothing in the braking
        sp = sc.measure_speeds([self.session], sc.blocked_segments([self.session]))
        cell = sp[('female', 'free', 'stand_run', 'F', 1.0)]
        self.assertAlmostEqual(cell['median'], 3.0, places=3)
        self.assertEqual(cell['segments'], 1)
        self.assertFalse(cell['enough'])                                     # one segment only

    def test_ratio(self):
        cells, pairs = sc.measure_ratios([self.session], self.dbs)
        self.assertGreaterEqual(len(pairs), 5)
        c = cells[('female', 'free', 'stand_run', 'F', 1.0)]
        self.assertAlmostEqual(c['rate'], self.fx.RATE, places=2)
        self.assertAlmostEqual(c['leg_speed'], self.fx.CLIP_SPEED * self.fx.RATE, delta=0.02)
        self.assertAlmostEqual(c['ratio'], self.fx.CLIP_SPEED * self.fx.RATE / self.fx.RUN_SPEED, delta=0.01)
        self.assertTrue(any(p['clip_speed'] > 4.9 for p in pairs))           # wrapped pairs are unwrapped, not dropped

    def test_stop_overshoot(self):
        summary, events = sc.measure_stops([self.session], self.dbs)
        self.assertEqual(len(events), 1)
        e = events[0]
        self.assertEqual(e['stop_clip'], 'M_Neutral_Run_Stop_F_Lfoot')
        self.assertAlmostEqual(e['capsule_stop_m'], self.fx.stop_distance, delta=0.002)  # linear braking over 0.2 s
        # stop clip picked at ct 0.25: its root still moves 1.2 m/s for 0.25 s = 0.3 m
        self.assertAlmostEqual(e['clip_remaining_m'], 0.3, delta=0.01)
        self.assertAlmostEqual(e['overshoot_cm'], 100.0 * (0.3 - e['capsule_remaining_m']), delta=0.2)
        self.assertLess(abs(e['overshoot_cm']), 6.0)
        self.assertIn(('female', 'free/stand_run', 1.0), summary)


if __name__ == '__main__':
    unittest.main()
