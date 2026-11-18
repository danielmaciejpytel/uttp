"""Tests of runtime_world_slide.py helpers: replay option detection, the variant database path and the contact track rebuilt from the
replay's MOTION trace file (the line stamp the replay tool writes: feed time as a 2000-01-01 wall time).

    python -I test_runtime_world_slide.py
"""
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import runtime_world_slide as rws  # noqa: E402

ws = rws.ws
DT = 1.0 / 60.0
NAMES = ['Walk_Loop', 'Walk_Stop']


def stamp(t):
    """The stamp utpp_replay_pose_stages --motion-log writes for feed time t (s)."""
    whole = int(t)
    micros = min(999999, int((t - whole) * 1e6 + 0.5))
    return '[2000-01-01 %02d:%02d:%02d.%06d]' % ((whole // 3600) % 24, (whole // 60) % 60, whole % 60, micros)


def motion_line(t, clip, ct, blend=0.2, chg=False):
    return ('%s UTPP MOTION t=%.3f db=0:base clip=%d:%s ct=%.3f cost=0.100 blend=%.2f rot=orient start=0 pivot=0%s'
            % (stamp(t), t, NAMES.index(clip), clip, ct, blend, ' chg' if chg else ''))


def contact_db():
    # clip 0: loop of 31 frames (1 s at 30 Hz), left contact on the first half; clip 1: one-shot of 11 frames, right contact
    c0 = np.zeros((31, 2))
    c0[:16, 0] = 1.0
    c0[30] = c0[0]
    c1 = np.zeros((11, 2))
    c1[:, 1] = 1.0
    return ws.ContactDB.from_arrays(NAMES, [31, 11], [True, False], [c0, c1], fps=30.0)


class OptionTests(unittest.TestCase):
    def test_options_are_detected_only_with_a_non_zero_value(self):
        self.assertTrue(rws.motion_log_requested('--capsule-stats 1 --air-feed 1 --motion-log 1'))
        self.assertFalse(rws.motion_log_requested('--capsule-stats 1 --motion-log 0'))
        self.assertFalse(rws.motion_log_requested('--capsule-stats 1'))
        self.assertFalse(rws.motion_log_requested(''))
        self.assertTrue(rws.capsule_stats_requested('--motion-log 1 --capsule-stats 1'))
        self.assertTrue(rws.air_feed_requested('--capsule-stats 1 --air-feed 1'))
        self.assertFalse(rws.air_feed_requested('--air-feed 0 --motion-log 1'))

    def test_the_world_slide_database_follows_the_root(self):
        root = os.path.join('X', 'variant')
        self.assertEqual(rws.db_path_of(root, 'man'), os.path.join(root, 'Data', 'utpp_motion_man.utppmm'))


class TraceContactTests(unittest.TestCase):
    def track(self, lines, n=150):
        with tempfile.TemporaryDirectory() as d:
            prefix = os.path.join(d, 'run')
            with open(prefix + '.motion.log', 'w', encoding='utf-8') as f:
                f.write('%s UTPP mm.motionDriver load done #1 body=woman\n' % stamp(0.0))
                for ln in lines:
                    f.write(ln + '\n')
            return rws.trace_contact_track(prefix, contact_db(), np.arange(n) * DT)

    def test_a_pick_change_in_the_trace_crossfades_the_contact_on_the_feed_time_base(self):
        tr = self.track([motion_line(0.0, 'Walk_Loop', 0.0), motion_line(0.5, 'Walk_Loop', 0.5),
                         motion_line(1.0, 'Walk_Stop', 0.0, blend=0.5, chg=True), motion_line(1.5, 'Walk_Stop', 0.5)])
        f = int(round(1.25 / DT))                     # half way through the blend: loop ct 0.25 (left 1) under the stop (right 1)
        self.assertEqual(int(tr.state[f]), ws.ST_BLEND)
        self.assertAlmostEqual(float(tr.cont[f, 0]), 0.5, delta=0.01)
        self.assertAlmostEqual(float(tr.cont[f, 1]), 0.5, delta=0.01)
        self.assertEqual(int(tr.state[int(round(0.9 / DT))]), ws.ST_OK)    # before the change line: the loop alone
        self.assertEqual(tr.cont[int(round(1.6 / DT))].tolist(), [0.0, 1.0])
        self.assertEqual(tr.notes['blends'], 1)

    def test_a_trace_without_motion_lines_is_refused(self):
        with self.assertRaises(ValueError):
            self.track([])


if __name__ == '__main__':
    unittest.main()
