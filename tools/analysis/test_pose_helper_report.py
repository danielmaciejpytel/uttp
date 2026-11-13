"""Tests of pose_helper_report.py on a tiny synthetic recording written with the reference encoder of test_pose_capture_to_csv.py.

  python -I test_pose_helper_report.py
"""
import os
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_helper_report as ph  # noqa: E402
import test_pose_capture_to_csv as tp  # noqa: E402

NAMES = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Head', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand',
         'RightShoulder', 'RightArm', 'l_scapula_A_top_out_JNT', 'r_scapula_A_top_out_JNT', 'l_butterfly_top_CRV_bot_out_JNT',
         'l_calm_deltoid_JNT']
PARENT = [-1, 0, 1, 2, 3, 4, 5, 4, 7, 8, 9, 4, 11, 4, 4, 4, 8]
FRAMES = 60
HZ = 1000
BAD = 40                 # frames 40..59 carry the anomalies (a third of the recording: the median reference stays the calm pose)
NAME = 'utpp-pose-2026-11-11-10-00-00.utpppose'     # local stamp; the header holds UTC 09:00:00 (+60 min)


def quat_z(deg):
    a = np.radians(deg) / 2.0
    return [0.0, 0.0, np.sin(a), np.cos(a)]


class Rec:
    """Slice-like object for tp.encode_header / tp.encode_block."""

    def __init__(self, shift=0.0, with_post=False):
        n = len(NAMES)
        self.names = NAMES
        self.game_index = list(range(n))
        self.bones = [(i, 0x1000 + i, PARENT[i]) for i in range(n)]
        self.qpc_hz, self.arm_ticks, self.utc = HZ, 0, (2026, 11, 11, 9, 0, 0, 0)
        self.max_frames, self.rig, self.rig_count = 10000, 0xABC, n
        self.frames = FRAMES
        v = np.zeros((FRAMES, n, 24), dtype=np.float32)
        for st in (0, 1):
            v[:, :, st * 12 + 7] = 1.0                    # identity quaternion w
            v[:, :, st * 12 + 8:st * 12 + 12] = 1.0       # scale
        i = {nm: k for k, nm in enumerate(NAMES)}

        def put(bone, frames, deg=None, cm=None, stages=(0, 1)):
            for st in stages:
                for f in frames:
                    if deg is not None:
                        v[f, i[bone], st * 12 + 4:st * 12 + 8] = quat_z(deg)
                    if cm is not None:
                        v[f, i[bone], st * 12 + 0] = cm / 100.0

        bad = range(BAD, FRAMES)
        put('l_scapula_A_top_out_JNT', bad, deg=90.0 - shift, stages=(1,))            # extreme only after the mod's write; drivers at rest
        put('RightShoulder', bad, deg=90.0)                                             # driven pair: clavicle and scapula rotate together
        put('r_scapula_A_top_out_JNT', bad, deg=90.0)
        put('l_butterfly_top_CRV_bot_out_JNT', bad, cm=6.0)                             # translation extreme, drivers calm
        put('LeftArm', range(0, FRAMES), deg=0.0)
        put('l_calm_deltoid_JNT', range(0, FRAMES, 2), deg=3.0)                         # small, harmless motion
        self.values = v
        self.meta = np.zeros((FRAMES, 4), dtype=np.int64)
        self.meta[:, 0] = np.arange(FRAMES) * 100         # 0.1 s per frame at 1 kHz
        self.meta[:, 1] = 7
        self.meta[:, 2] = 3
        self.meta[:, 3] = 11
        self.with_post = with_post


def write_recording(path, rec, post_offset_deg=None):
    streams = (0, 1) if post_offset_deg is not None else (0,)
    data = tp.encode_header(rec, 0, 0, streams=streams) + tp.encode_block(rec, 0, 0, 30, 0) + tp.encode_block(rec, 0, 30, FRAMES, 0)
    if post_offset_deg is not None:
        pv = rec.values[:, :, 12:24].copy()
        k = NAMES.index('l_scapula_A_top_out_JNT')
        for f in range(BAD, FRAMES):
            pv[f, k, 4:8] = quat_z(90.0 - post_offset_deg)          # the engine's helper pass moves it away from the pose it was given
        pm = np.zeros((FRAMES, 8), dtype=np.int64)
        pm[:, :4] = rec.meta
        pm[:, 0] += 1                                               # 1 tick (1 ms) after the matching frame of stream 0
        full = np.zeros((FRAMES, len(NAMES), 24), dtype=np.float32)
        full[:, :, :12] = pv
        data += tp.encode_block(rec, 1, 0, FRAMES, 0, values=full, meta=pm)
    data += tp.encode_trailer((FRAMES, FRAMES), dropped=(0, 0))
    pathlib.Path(path).write_bytes(data)
    return str(path)


def rows_of(R, name, phase):
    return [r for r in ph.bones_rows(R) if r['bone'] == name and r['phase'] == phase][0]


def cons(R, name, phase):
    return [r for r in ph.consistency_rows(R) if r['bone'] == name and r['phase'] == phase][0]


class HelperReportTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=tp.scratch_root())
        self.addCleanup(self.tmp.cleanup)
        self.dir = pathlib.Path(self.tmp.name)
        self.path = write_recording(self.dir / NAME, Rec())
        self.out = str(self.dir / 'out')

    def run_report(self, cfg=None, **kw):
        R, _ = ph.run([self.path], self.out, cfg or ph.Config(), log=lambda s: None, **kw)
        return R[0]

    def test_classification_and_phase_names(self):
        R = self.run_report()
        lay = R.layout
        self.assertEqual(R.phases, ['before_mm', 'after_mm_and_aim'])
        self.assertEqual([lay.names[k] for k in lay.core],
                         ['Spine1', 'Spine2', 'Spine3', 'Neck', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm', 'LeftForeArm', 'LeftHand'])
        helpers = {lay.names[k] for k in lay.helpers}
        self.assertEqual(helpers, {'l_scapula_A_top_out_JNT', 'r_scapula_A_top_out_JNT', 'l_butterfly_top_CRV_bot_out_JNT', 'l_calm_deltoid_JNT'})
        self.assertIn('RightHand', lay.missing_core)

    def test_table_values_and_phase_difference(self):
        R = self.run_report()
        r = rows_of(R, 'l_scapula_A_top_out_JNT', 'after_mm_and_aim')
        self.assertAlmostEqual(float(r['rot_max_deg']), 90.0, delta=0.05)
        self.assertAlmostEqual(float(r['rot_min_deg']), 0.0, delta=0.05)
        self.assertAlmostEqual(float(r['rot_median_deg']), 0.0, delta=0.05)
        self.assertEqual(r['frames'], FRAMES)
        self.assertAlmostEqual(float(rows_of(R, 'l_scapula_A_top_out_JNT', 'before_mm')['rot_max_deg']), 0.0, delta=0.05)
        b = rows_of(R, 'l_butterfly_top_CRV_bot_out_JNT', 'before_mm')
        self.assertAlmostEqual(float(b['trans_max_cm']), 6.0, delta=0.01)
        d = [x for x in ph.phasediff_rows(R) if x['bone'] == 'l_scapula_A_top_out_JNT'][0]
        self.assertEqual(d['pair'], 'before_mm vs after_mm_and_aim')
        self.assertAlmostEqual(float(d['rot_diff_max_deg']), 90.0, delta=0.05)
        self.assertAlmostEqual(float(d['rot_diff_median_deg']), 0.0, delta=0.05)

    def test_flags(self):
        R = self.run_report()
        a = cons(R, 'l_scapula_A_top_out_JNT', 'after_mm_and_aim')
        self.assertEqual(a['jump_frames'], 1)
        self.assertEqual(a['undriven_jump_frames'], 1)
        self.assertEqual(a['extreme_with_normal_drivers_frames'], FRAMES - BAD)
        self.assertEqual(a['flags'], 'UNDRIVEN_JUMP EXTREME_WITH_NORMAL_DRIVERS')
        self.assertIn('10:00:04.000', a['worst_undriven_jump'])           # frame 40 = 4.0 s after the 10:00:00 stamp (local time)
        self.assertEqual(cons(R, 'l_scapula_A_top_out_JNT', 'before_mm')['flags'], '')
        # translation extreme with calm drivers
        self.assertEqual(cons(R, 'l_butterfly_top_CRV_bot_out_JNT', 'after_mm_and_aim')['flags'], 'UNDRIVEN_JUMP EXTREME_WITH_NORMAL_DRIVERS')
        # the scapula that rotates together with its clavicle: the jump is driven and the drivers are not in the normal range
        c = cons(R, 'r_scapula_A_top_out_JNT', 'after_mm_and_aim')
        self.assertEqual((c['undriven_jump_frames'], c['extreme_with_normal_drivers_frames'], c['flags']), (0, 0, 'JUMP'))
        self.assertEqual(c['jump_frames'], 1)
        # harmless small motion: nothing
        self.assertEqual(cons(R, 'l_calm_deltoid_JNT', 'after_mm_and_aim')['flags'], '')

    def test_window_cuts_the_frames(self):
        cfg = ph.Config(window=ph.make_window('10:00:01', '10:00:02'))
        R = self.run_report(cfg)
        self.assertEqual(R.frames, [11, 11])                               # frames 10..20 inclusive
        r = rows_of(R, 'l_scapula_A_top_out_JNT', 'after_mm_and_aim')
        self.assertEqual(r['frames'], 11)
        self.assertAlmostEqual(float(r['rot_max_deg']), 0.0, delta=0.05)    # the anomaly (from 4 s) is outside the window
        self.assertEqual(cons(R, 'l_scapula_A_top_out_JNT', 'after_mm_and_aim')['flags'], '')
        late = self.run_report(ph.Config(window=ph.make_window('10:00:04.5', '10:00:05')))
        self.assertAlmostEqual(float(rows_of(late, 'l_scapula_A_top_out_JNT', 'after_mm_and_aim')['rot_min_deg']), 90.0, delta=0.05)
        utc = self.run_report(ph.Config(window=ph.make_window('09:00:01', '09:00:02'), tz_seconds=0))
        self.assertEqual(utc.frames, [11, 11])

    def test_post_helper_stream(self):
        path = write_recording(self.dir / 'post' / NAME, Rec(), post_offset_deg=15.0) if (self.dir / 'post').mkdir() is None else None
        R, _ = ph.run([path], str(self.dir / 'out2'), ph.Config(), log=lambda s: None)
        R = R[0]
        self.assertEqual(R.phases, ['before_mm', 'after_mm_and_aim', 'post_helper_apply_after_commit'])
        self.assertEqual(R.matched, FRAMES)
        d = [x for x in ph.phasediff_rows(R) if x['bone'] == 'l_scapula_A_top_out_JNT' and x['pair'].endswith('post_helper_apply_after_commit')][0]
        self.assertAlmostEqual(float(d['rot_diff_max_deg']), 15.0, delta=0.05)
        self.assertEqual(R.frames, [FRAMES] * 3)

    def test_compare_sorted_by_difference(self):
        other = write_recording(self.dir / 'b' / NAME, Rec(shift=30.0)) if (self.dir / 'b').mkdir() is None else None
        R, C = ph.run([self.path], self.out, ph.Config(), compare=[other], log=lambda s: None)
        rows, only_a, only_b = ph.compare_rows(R[0], C)
        self.assertEqual((only_a, only_b), ([], []))
        self.assertEqual(rows[0]['bone'], 'l_scapula_A_top_out_JNT')
        self.assertEqual(rows[0]['phase'], 'after_mm_and_aim')
        self.assertAlmostEqual(float(rows[0]['rot_max_diff']), 30.0, delta=0.1)
        keys = [abs(float(r['rot_p95_diff'])) for r in rows]
        self.assertEqual(keys, sorted(keys, reverse=True))
        self.assertTrue(os.path.exists(os.path.join(self.out, 'pose_helper_report_compare.csv')))

    def test_outputs_written(self):
        self.run_report()
        for n in ('bones', 'consistency', 'phasediff'):
            self.assertTrue(os.path.getsize(os.path.join(self.out, 'pose_helper_report_%s.csv' % n)) > 100)
        self.assertIn('helper bones (4)', (pathlib.Path(self.out) / 'pose_helper_report_summary.txt').read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
