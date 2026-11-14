"""Tests of pose_metrics.py (smoothness and deformation meter for the compact pose recordings).

  python -I test_pose_metrics.py

The fixtures in testdata/ are short slices of a real session: pose_metrics_core.utpppose (295 frames, the bones the metrics read), the matching
log and feed dump excerpts, and pose_metrics_full.utpppose (12 frames with every bone, for the helper comparison). Synthetic variants are written
into a temporary folder: a single-frame pop of the head, an overturned arm, a rotation that follows the clock versus one that follows the frame
count. Needs numpy and Windows (the recordings are compressed with the Windows compression API). The helper comparison needs the serialized
deformer graph and is skipped without it.
"""
import datetime
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_pose_metrics_fixture as mk  # noqa: E402
import pose_metrics as pm  # noqa: E402

DATA = os.path.join(HERE, 'testdata')
CORE = os.path.join(DATA, 'pose_metrics_core.utpppose')
FULL = os.path.join(DATA, 'pose_metrics_full.utpppose')
LOG = os.path.join(DATA, 'pose_metrics_core.session.txt')
FEED = os.path.join(DATA, 'pose_metrics_core.feed.txt')
TZ = 1.0                                           # the log excerpt carries local time of UTC+1


def load_core():
    """-> (header, game_index, meta [N, 4], values [N, bones, 2, 12], bone names of the mapped bones)."""
    h = pm.phr.read_header(CORE)
    metas, vals = [], []
    for item in pm.pc.read_capture(CORE):
        if item[0] == 'block':
            metas.append(item[2])
            vals.append(item[3])
    meta = np.concatenate(metas)
    nm = len(h.mapped())
    values = np.concatenate(vals).reshape(len(meta), nm, 2, 12)
    return h, list(h.game_index), meta, values, [h.names[i] for i in h.mapped()]


def write_variant(path, h, gi, meta, values):
    mk.write_capture(path, h, gi, meta, values.reshape(len(values), -1), algorithm=0)


def rotate_local(values, names, bone, frame, axis, degrees, stage=1):
    """Left-multiplies the local rotation of a bone in one frame by a rotation about a local axis (axis index 0..2)."""
    k = names.index(bone)
    q = values[frame, k, stage, 4:8].astype(np.float64)
    half = np.radians(degrees) / 2
    r = np.zeros(4)
    r[axis], r[3] = np.sin(half), np.cos(half)
    values[frame, k, stage, 4:8] = pm.qmul(q, r)


class QuaternionHelpers(unittest.TestCase):
    def test_twist_swing_roundtrip(self):
        tw = pm.qmul(np.array([0.0, np.sin(np.radians(20)), 0, np.cos(np.radians(20))]), np.array([np.sin(np.radians(15)), 0, 0, np.cos(np.radians(15))]))
        t, sy, sz = pm.twist_swing_deg(tw)
        self.assertAlmostEqual(float(t), 30.0, places=3)
        self.assertAlmostEqual(float(sy), 40.0, places=3)
        self.assertAlmostEqual(float(sz), 0.0, places=3)

    def test_rotvec_matches_angle(self):
        q = np.array([0.0, 0.0, np.sin(np.radians(25)), np.cos(np.radians(25))])
        self.assertAlmostEqual(float(np.linalg.norm(pm.qrotvec_deg(q))), 50.0, places=3)


class Fixture(unittest.TestCase):
    def test_fixture_is_a_real_slice(self):
        h, gi, meta, values, names = load_core()
        self.assertEqual(len(meta), 295)
        for bone in pm.TRACK + pm.GUARD_ORDER:
            self.assertIn(bone, names)
        self.assertLess(len(names), 80)                    # only the bones the metrics need are mapped
        self.assertLess(os.path.getsize(CORE) + os.path.getsize(FULL) + os.path.getsize(LOG) + os.path.getsize(FEED), 400000)


class Pacing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.res = pm.analyze(CORE, LOG, windows=None, feed_file=FEED, helpers=False, tz_hours=TZ)

    def test_frame_pacing(self):
        w = self.res['windows'][0]
        self.assertEqual(self.res['frames'], 295)
        self.assertEqual(self.res['body'], 'woman')
        fp = w['frame_pacing']
        self.assertTrue(55 < fp['fps'] < 65, fp)
        self.assertTrue(14 < fp['dt_median_ms'] < 19, fp)
        self.assertLess(fp['hitch_share'], 0.1)
        self.assertLess(fp['step_vs_timestamp_mean_abs_ms'], 2.0)         # feed step and timestamps describe the same frames

    def test_feed_is_aligned(self):
        self.assertTrue(self.res['feed_alignment'].startswith('aligned'), self.res['feed_alignment'])

    def test_log_events(self):
        e = self.res['windows'][0]['events']
        self.assertTrue(e['log_events_available'])
        self.assertGreaterEqual(e['clip_changes'], 1)
        self.assertGreaterEqual(len(e['headlook_switches']), 3)
        self.assertEqual(self.res['log'], os.path.basename(LOG))

    def test_without_log_or_feed(self):
        res = pm.analyze(CORE, False, None, feed_file=False, helpers=False, tz_hours=TZ)
        self.assertFalse(res['windows'][0]['events']['log_events_available'])
        self.assertEqual(res['windows'][0]['events']['clip_changes'], 0)
        self.assertEqual(res['feed_alignment'], 'no feed dump')

    def test_windows_forms_select_the_same_frames(self):
        start = datetime.datetime.strptime(self.res['start'], '%Y-%m-%d %H:%M:%S.%f')
        a = start + datetime.timedelta(seconds=1.0)
        b = start + datetime.timedelta(seconds=2.0)
        by_dt = pm.analyze(CORE, False, [('w', a, b)], feed_file=False, helpers=False, tz_hours=TZ)['windows'][0]
        by_text = pm.analyze(CORE, False, [('w', a.strftime('%H:%M:%S.%f')[:-3], b.strftime('%H:%M:%S.%f')[:-3])], feed_file=False, helpers=False, tz_hours=TZ)['windows'][0]
        by_float = pm.analyze(CORE, False, [('w', 1.0, 2.0)], feed_file=False, helpers=False, tz_hours=TZ)['windows'][0]
        self.assertTrue(55 <= by_dt['frames'] <= 65, by_dt['frames'])
        self.assertLessEqual(abs(by_dt['frames'] - by_text['frames']), 1)
        self.assertLessEqual(abs(by_dt['frames'] - by_float['frames']), 1)

    def test_auto_windows(self):
        res = pm.analyze(CORE, LOG, 'auto', feed_file=FEED, helpers=False, tz_hours=TZ)
        labels = {w['label'].split('_to_')[0] for w in res['windows']}
        self.assertTrue(any(x.startswith('headlook') for x in labels), labels)


class Discontinuities(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.h, self.gi, self.meta, self.values, self.names = load_core()

    def analyze_variant(self, values, meta=None, **kw):
        path = os.path.join(self.tmp.name, 'variant.utpppose')
        write_variant(path, self.h, self.gi, self.meta if meta is None else meta, values)
        return pm.analyze(path, False, None, feed_file=False, helpers=False, tz_hours=TZ, **kw)

    def smooth(self, res, key, stage='final'):
        return res['windows'][0]['smoothness'][stage][key]

    def test_single_frame_pop_is_flagged_with_its_size(self):
        base = self.analyze_variant(self.values.copy())
        v = self.values.copy()
        rotate_local(v, self.names, 'Head', 150, 0, 25.0)            # one frame only: the next frame is back on the original pose
        res = self.analyze_variant(v)
        before, after = self.smooth(base, 'Head.parent'), self.smooth(res, 'Head.parent')
        self.assertGreater(after['spikes'], before['spikes'])
        self.assertGreater(after['spike_max_deg'], 18.0)
        head = [d for d in res['windows'][0]['worst_discontinuities'] if d['bone'].startswith('Head')]
        self.assertTrue(head)
        t_pop = float(self.meta[150, 0] - self.meta[0, 0]) / self.h.qpc_hz
        self.assertTrue(any(abs(d['t_s'] - t_pop) < 0.05 for d in head), (head, t_pop))

    def test_hips_rotation_and_translation_are_both_reported(self):
        smooth = self.smooth(self.analyze_variant(self.values.copy()), 'Hips.model')
        self.assertIn('speed_deg_s_p99', smooth)
        win = self.analyze_variant(self.values.copy())['windows'][0]['smoothness']['final']
        self.assertIn('Hips_pos.model', win)
        self.assertIn('speed_cm_s_p99', win['Hips_pos.model'])
        self.assertIn('speed_deg_s_p99', win['Hips.model'])

    def test_smooth_motion_has_no_pop_of_its_own(self):
        base = self.analyze_variant(self.values.copy())
        self.assertLessEqual(self.smooth(base, 'Head.parent')['spikes'], 3)

    def test_rotation_following_the_clock_is_smooth_with_uneven_frames(self):
        rng = np.random.RandomState(7)
        n = len(self.meta)
        dts = rng.uniform(0.008, 0.030, n)                        # frame times between 8 and 30 ms
        secs = np.cumsum(dts)
        meta = self.meta.copy()
        meta[:, 0] = (self.meta[0, 0] + secs * self.h.qpc_hz).astype(np.int64)
        base = np.repeat(self.values[:1], n, axis=0)              # a still body ...
        k = self.names.index('Head')
        follow_clock, follow_frames = base.copy(), base.copy()
        q0 = self.values[0, k, 1, 4:8].astype(np.float64)
        for f in range(n):
            for arr, ang in ((follow_clock, 60.0 * secs[f]), (follow_frames, 1.0 * f)):
                half = np.radians(ang) / 2
                arr[f, k, 1, 4:8] = pm.qmul(q0, np.array([0.0, np.sin(half), 0.0, np.cos(half)]))
        a = self.smooth(self.analyze_variant(follow_clock, meta), 'Head.parent')
        b = self.smooth(self.analyze_variant(follow_frames, meta), 'Head.parent')
        self.assertLess(a['speed_deg_s_p99'] / a['speed_deg_s_p50'], 1.2)          # steady angular velocity whatever the frame times are
        self.assertGreater(b['speed_deg_s_p99'] / b['speed_deg_s_p50'], 2.0)       # a rotation tied to the frame count is visibly uneven
        self.assertEqual(a['spikes'], 0)

    def test_overturned_arm_violates_the_anatomy_band(self):
        base = self.analyze_variant(self.values.copy())
        v = self.values.copy()
        for f in range(100, 200):
            rotate_local(v, self.names, 'LeftArm', f, 0, 150.0)
        res = self.analyze_variant(v)
        a = res['windows'][0]['deformation']['anatomy']['final']
        b = base['windows'][0]['deformation']['anatomy']['final']
        self.assertGreater(a['worst_deg'], 60.0)
        self.assertEqual(a['worst_joint'], 'LeftArm')
        self.assertGreater(a['joints']['LeftArm']['share_outside'], 0.3)
        self.assertLess(b['joints']['LeftArm']['share_outside'], 0.05)

    def test_discontinuity_at_a_head_look_switch_is_tagged(self):
        log = pm.parse_log(LOG)
        rec, data = pm.load_recording(CORE, {'Head'}, TZ)
        ev = pm.Events(rec, log)
        t_event, _ = ev.headlook[len(ev.headlook) // 2]
        frame = int(np.argmin(np.abs(data['sec'] - t_event)))
        self.assertTrue(10 < frame < len(self.meta) - 10)
        v = self.values.copy()
        rotate_local(v, self.names, 'Head', frame, 0, 30.0)
        path = os.path.join(self.tmp.name, 'tagged.utpppose')
        write_variant(path, self.h, self.gi, self.meta, v)
        res = pm.analyze(path, LOG, None, feed_file=False, helpers=False, tz_hours=TZ)
        tags = res['windows'][0]['events']['discontinuities_by_tag']
        self.assertGreaterEqual(tags.get('layer_switch', 0), 1, tags)


ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))


@unittest.skipUnless(all(os.path.isfile(os.path.join(ROOT, *p.split('/'))) for p in (
    'Extracted/game/json/vanilla-extra/player_woman_base_deformations.animgraph.json',
    'Extracted/game/json/vanilla-extra/player_woman_base_deformations.rig.json',
    'Animations/retarget/out_sourcepack_woman/Pose_Pistol2H_Stand_Aim_Idle.utppclip.json')), 'serialized deformer graph not available')
class HelperMismatch(unittest.TestCase):
    def test_stage_before_the_mod_matches_the_recomputed_helpers(self):
        res = pm.analyze(FULL, False, None, feed_file=False, helpers=True, helper_hz=1000.0, tz_hours=TZ)
        if not res['helper_samples']:
            self.skipTest('helper evaluation unavailable: %s' % res['notes'])
        self.assertEqual(res['helper_samples']['final'], res['frames'])
        hm = res['windows'][0]['deformation']['helper_mismatch']
        self.assertLess(hm['pre_mod']['p95_cm'], 0.5)
        self.assertGreater(hm['final']['max_cm'], 3.0 * max(hm['pre_mod']['max_cm'], 0.1))
        self.assertIn('shoulder_chest', hm['final']['groups'])


class InputKinds(unittest.TestCase):
    def feed(self, segments, dt=1.0 / 60):
        """segments: [(seconds, mvx, mvy, camera degrees per second)] -> feed dict as read_feed returns it."""
        t, mx, my, cam = [], [], [], []
        now, cam_deg = 0.0, 0.0
        for seconds, x, y, rate in segments:
            for _ in range(int(round(seconds / dt))):
                now += dt
                cam_deg += rate * dt
                t.append(now), mx.append(x), my.append(y), cam.append(cam_deg)
        return dict(t=np.array(t), mvx=np.array(mx), mvy=np.array(my), camDeg=np.array(cam))

    def kinds(self, segments):
        feed = self.feed(segments)
        r2f = np.arange(len(feed['t']))
        sec = feed['t']
        return {w[0] for w in pm.input_windows(feed, r2f, sec)}

    def test_start_straight_stop(self):
        k = self.kinds([(1.0, 0, 0, 0), (3.0, 0, 1, 0), (1.5, 0, 0, 0)])
        self.assertIn('in_start', k)
        self.assertIn('in_stop', k)
        self.assertIn('in_straight', k)

    def test_tap(self):
        self.assertIn('in_tap', self.kinds([(1.0, 0, 0, 0), (0.2, 0, 1, 0), (1.5, 0, 0, 0)]))

    def test_direction_switches(self):
        k = self.kinds([(1.0, 0, 0, 0), (1.0, 0, 1, 0), (1.5, 0, -1, 0), (1.0, 1, 0, 0), (1.5, 0, 0, 0)])
        self.assertIn('in_switch_reverse', k)
        self.assertIn('in_switch_quarter', k)

    def test_arc_strafe_backward(self):
        self.assertIn('in_arc', self.kinds([(1.0, 0, 0, 0), (2.5, 0, 1, 60.0), (1.0, 0, 0, 0)]))
        self.assertIn('in_strafe', self.kinds([(1.0, 0, 0, 0), (2.5, 1, 0, 0), (1.0, 0, 0, 0)]))
        self.assertIn('in_backward', self.kinds([(1.0, 0, 0, 0), (2.5, 0, -1, 0), (1.0, 0, 0, 0)]))

    def test_stepped_and_bridged_direction_changes(self):
        s = 0.7071
        k = self.kinds([(1.0, 0, 0, 0), (1.0, 0, 1, 0), (0.1, s, s, 0), (1.0, 1, 0, 0), (1.5, 0, 0, 0)])           # W, W+D, D: two 45 degree steps
        self.assertIn('in_switch_quarter', k)
        k = self.kinds([(1.0, 0, 0, 0), (0.6, 0, 1, 0), (0.05, 0, 0, 0), (0.6, 1, 0, 0), (1.5, 0, 0, 0)])           # W, short release, D
        self.assertIn('in_switch_quarter', k)
        self.assertNotIn('in_tap', k)
        self.assertEqual(len([w for w in pm.input_windows(self.feed([(1.0, 0, 0, 0), (0.6, 0, 1, 0), (0.05, 0, 0, 0), (0.6, 1, 0, 0), (1.5, 0, 0, 0)]),
                                                          np.arange(int(round(3.75 * 60))), np.arange(int(round(3.75 * 60))) / 60.0)
                              if w[0] == 'in_stop']), 1)

    def test_unmatched_rows_give_no_windows(self):
        feed = self.feed([(1.0, 0, 0, 0), (3.0, 0, 1, 0), (1.5, 0, 0, 0)])
        self.assertEqual(pm.input_windows(feed, np.full(len(feed['t']), -1), feed['t']), [])


class TextCapture(unittest.TestCase):
    def test_text_capture_matches_the_binary_one(self):
        with tempfile.TemporaryDirectory() as d:
            written = pm.pc.convert([CORE], d)
            csv_path = written[0]
            rec_a, a = pm.load_recording(CORE, {'Head', 'LeftHand'}, TZ)
            rec_b, b = pm.load_csv_recording(csv_path, {'Head', 'LeftHand'}, TZ)
            self.assertEqual(rec_a.names, rec_b.names)
            self.assertEqual(rec_a.parent, rec_b.parent)
            self.assertEqual(a['keep'], b['keep'])
            self.assertEqual(len(a['sec']), len(b['sec']))
            np.testing.assert_allclose(a['sec'], b['sec'], atol=1e-6)
            np.testing.assert_allclose(a['q'], b['q'], atol=1e-4)
            np.testing.assert_allclose(a['t'], b['t'], atol=1e-4)
            res = pm.analyze(csv_path, False, None, feed_file=False, tz_hours=TZ)
            self.assertEqual(res['frames'], 295)
            self.assertTrue(any('text capture' in n for n in res['notes']))


class CommandLine(unittest.TestCase):
    def test_cli_writes_json(self):
        with tempfile.TemporaryDirectory() as d:
            out = os.path.join(d, 'r.json')
            self.assertEqual(pm.main([CORE, '--log', LOG, '--feed', FEED, '--windows', 'all', '--no-helpers', '--tz-hours', '1', '--json', out]), 0)
            self.assertTrue(os.path.getsize(out) > 1000)
            self.assertEqual(pm.parse_cli_windows(['a=23:04:12-23:04:14']), [('a', '23:04:12', '23:04:14')])


if __name__ == '__main__':
    unittest.main()
