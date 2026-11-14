"""Tests of pose_clip_match.py on synthetic clips, a synthetic text recording and a synthetic compact recording.

  python -I test_pose_clip_match.py
"""
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_clip_match as pm  # noqa: E402
import test_pose_capture_to_csv as tp  # noqa: E402
import test_pose_helper_report as th  # noqa: E402

NAMES = list(pm.MATCH_BONES) + ['Head', 'Root']


def quat_axis(axis, deg):
    a = np.radians(deg) / 2.0
    v = np.asarray(axis, float)
    v = v / np.linalg.norm(v)
    return np.r_[v * np.sin(a), np.cos(a)]


def make_clip(seed, frames=40, drift=30.0):
    """A clip whose bones rotate by `drift` degrees over its length about a seed dependent axis."""
    rng = np.random.default_rng(seed)
    q = np.zeros((frames, len(NAMES), 4), np.float32)
    for b in range(len(NAMES)):
        axis = rng.normal(size=3)
        base = rng.uniform(0, 60)
        for f in range(frames):
            q[f, b] = quat_axis(axis, base + drift * f / (frames - 1))
    return q


class MatchTest(unittest.TestCase):
    def setUp(self):
        self.bank = pm.ClipBank()
        self.clips = {name: make_clip(seed) for name, seed in (('idle', 1), ('steal', 2), ('tpp_idle_twin', 1))}
        for name, q in self.clips.items():
            self.bank.add('setA', name, NAMES, q)
        self.bank.finish()

    def test_quat_angle(self):
        self.assertAlmostEqual(float(pm.quat_angle_deg(quat_axis([0, 0, 1], 0), quat_axis([0, 0, 1], 25))), 25.0, places=3)
        self.assertAlmostEqual(float(pm.quat_angle_deg(quat_axis([0, 0, 1], 0), -quat_axis([0, 0, 1], 0))), 0.0, places=3)   # q and -q are one rotation

    def test_best_clip_frame_and_ties(self):
        rec = self.clips['steal'][[5, 20, 33]]
        bank_q = pm.MATCH_BONES
        ix = [NAMES.index(b) for b in bank_q]
        res = pm.match_recording(rec[:, ix], self.bank)
        for r, f in zip(res, (5, 20, 33)):
            self.assertEqual((r['clip'], r['frame']), ('steal', f))
            self.assertLess(r['resid_deg'], 0.05)
            self.assertEqual(r['verdict'], 'match')
            self.assertGreater(r['margin_deg'], 1.0)

    def test_identical_clips_are_reported_as_ties(self):
        ix = [NAMES.index(b) for b in pm.MATCH_BONES]
        r = pm.match_recording(self.clips['idle'][[10]][:, ix], self.bank)[0]
        self.assertIn(r['clip'], ('idle', 'tpp_idle_twin'))
        self.assertAlmostEqual(r['margin_deg'], 0.0, places=3)
        other = 'tpp_idle_twin' if r['clip'] == 'idle' else 'idle'
        self.assertIn(('setA', other), r['ties'])

    def test_no_match_above_residual_limit(self):
        ix = [NAMES.index(b) for b in pm.MATCH_BONES]
        far = np.stack([quat_axis([1, 1, 0], 150)] * len(ix))[None]
        r = pm.match_recording(far, self.bank, max_resid=8.0)[0]
        self.assertEqual(r['verdict'], 'no match')
        self.assertGreater(r['resid_deg'], 8.0)

    def test_noise_keeps_the_clip(self):
        rng = np.random.default_rng(7)
        ix = [NAMES.index(b) for b in pm.MATCH_BONES]
        q = self.clips['steal'][[12]][:, ix].copy()
        q[..., :3] += rng.normal(scale=0.01, size=q[..., :3].shape)      # about 1 degree of jitter
        r = pm.match_recording(q, self.bank)[0]
        self.assertEqual((r['clip'], r['frame']), ('steal', 12))
        self.assertLess(r['resid_deg'], 2.0)


CSV_HEADER = ('ticks,desc,thread,rig,seq,publish,action,overlay,aiming,clip,weight,upper,bone,gameIndex,masked,' +
              ','.join('s%d_%d' % (s, k) for s in range(3) for k in range(12)) + ',' + ','.join('mm_%d' % k for k in range(7)))


def csv_row(ticks, action, overlay, bone, game_index, q0, q2):
    s = [0.0] * 36
    s[0:3] = [0.1, 0.2, 0.3]
    s[4:8] = q0
    s[24:27] = [0.4, 0.5, 0.6]
    s[28:32] = q2
    return ','.join(str(x) for x in [ticks, 1, 2, 3, 4, 5, action, overlay, 0, 495, 1, 1, bone, game_index, 0] + s + [0] * 7)


class CsvTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=tp.scratch_root())
        self.addCleanup(self.tmp.cleanup)
        self.path = pathlib.Path(self.tmp.name) / 'utpp-pose-123.csv'
        lines = ['# qpc_hz=1000 max_frames=108000 stages=game,blend,aim local_records=pos4,quat4,scale4', CSV_HEADER]
        q_id, q_z = [0, 0, 0, 1], list(quat_axis([0, 0, 1], 90))
        # frames at 0..9 (ticks 10 apart = 10 ms): action 0 for 0..2, reload (2) for 3..6, 0 for 7..9; bone 0 has no game index
        for f in range(10):
            act = 2 if 3 <= f <= 6 else 0
            for bone, gi in ((0, -1), (1, 2), (2, 4)):
                lines.append(csv_row(10 * (f + 1), act, 3 if act else 1, bone, gi, q_id, q_z))
        lines.append('# frames=10 dropped=0 failed=0')
        self.path.write_text('\n'.join(lines) + '\n')

    def test_episodes(self):
        eps, qpc = pm.csv_episodes(str(self.path), action=2)
        self.assertEqual(qpc, 1000.0)
        self.assertEqual(len(eps), 1)
        self.assertEqual((eps[0]['t0'], eps[0]['t1'], eps[0]['n'], eps[0]['overlay']), (40, 70, 4, 3))
        self.assertAlmostEqual(eps[0]['dur_s'], 0.03)

    def test_window_maps_game_index_to_names_and_stages(self):
        rec = pm.load_csv_window(str(self.path), 40, 70, {2: 'Hips', 4: 'Spine'}, ['Hips', 'Spine', 'Spine1'])
        self.assertEqual(rec.lr.shape, (4, 3, 3, 4))
        self.assertAlmostEqual(float(pm.quat_angle_deg(rec.lr[0, 0, 0], [0, 0, 0, 1])), 0.0, places=3)        # stage 0 identity
        self.assertAlmostEqual(float(pm.quat_angle_deg(rec.lr[0, 2, 1], quat_axis([0, 0, 1], 90))), 0.0, places=3)   # stage 2 rotated
        self.assertTrue(np.all(np.isnan(rec.lr[:, :, 2])))                                                    # a bone without rows stays nan
        self.assertAlmostEqual(float(rec.lt[0, 2, 0, 0]), 0.4, places=5)


class CompactTest(unittest.TestCase):
    def test_window_of_a_compact_recording(self):
        with tempfile.TemporaryDirectory(dir=tp.scratch_root()) as d:
            path = th.write_recording(pathlib.Path(d) / th.NAME, th.Rec())
            full = pm.load_compact(path)
            self.assertEqual(len(full.sec), th.FRAMES)
            self.assertEqual(full.lr.shape[1:3], (2, len(th.NAMES)))
            win = pm.load_compact(path, full.sec[10], full.sec[19])
            self.assertEqual(len(win.sec), 10)
            ix = th.NAMES.index('RightShoulder')
            late = pm.load_compact(path, full.sec[th.BAD], full.sec[-1])
            self.assertAlmostEqual(float(pm.quat_angle_deg(late.lr[0, 0, ix], th.quat_z(90.0))), 0.0, places=2)
            tbl = pm.index_name_table(path)
            self.assertEqual(tbl[0], th.NAMES[0])


if __name__ == '__main__':
    unittest.main()
