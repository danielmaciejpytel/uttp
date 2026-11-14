"""Tests of pose_stage_pops.py on a synthetic stage file (driver and final pose per feed) and synthetic clip frames.

  python -I test_pose_stage_pops.py
"""
import pathlib
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_stage_pops as psp  # noqa: E402

# Root -> Hips -> Spine chain -> Neck -> Head, both arms down to the hand, both legs down to the foot.
BONES = [('Root', -1), ('Hips', 0), ('Spine', 1), ('Spine1', 2), ('Spine2', 3), ('Spine3', 4), ('Neck', 5), ('Head', 6),
         ('LeftArm', 5), ('LeftHand', 8), ('RightArm', 5), ('RightHand', 10), ('LeftUpLeg', 1), ('LeftFoot', 12),
         ('RightUpLeg', 1), ('RightFoot', 14)]
META = ['t', 'dt', 'owned', 'published', 'tailed', 'clip', 'clipTime', 'playRate', 'mvx', 'mvy']
N = 120
DT = 1.0 / 60.0


def quat_z(deg):
    a = np.radians(deg) / 2.0
    return np.array([0.0, 0.0, np.sin(a), np.cos(a)])


def base_pose():
    """[N, nb, 7]: identity rotations, bones 10 cm apart; the left foot turns smoothly at 1 deg per feed."""
    pose = np.zeros((N, len(BONES), 7))
    pose[:, :, 6] = 1.0
    pose[:, 1:, 2] = 0.1
    for f in range(N):
        pose[f, BONES.index(('LeftFoot', 12)), 3:7] = quat_z(f * 1.0)
    return pose


def write_stages(prefix, meta, driver, final, clips):
    with open(prefix + '.txt', 'w', encoding='utf-8', newline='\n') as f:
        f.write('UTPP_POSE_STAGES 1\nbones %d\nmeta %d %s\n' % (len(BONES), len(META), ' '.join(META)))
        for i, (name, parent) in enumerate(BONES):
            f.write('bone %d %s %d\n' % (i, name, parent))
        for cid, name in clips.items():
            f.write('clip %d %s\n' % (cid, name))
    rows = np.concatenate([np.stack([meta[k] for k in META], 1), driver.reshape(N, -1), final.reshape(N, -1)], 1)
    rows.astype(np.float32).tofile(prefix + '.bin')


def default_meta():
    return dict(t=np.arange(N) * DT, dt=np.full(N, DT), owned=np.ones(N), published=np.ones(N), tailed=np.ones(N),
                clip=np.zeros(N), clipTime=np.arange(N) * DT, playRate=np.ones(N), mvx=np.ones(N), mvy=np.zeros(N))


class StagePopsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.prefix = str(pathlib.Path(self.tmp.name) / 'stages')

    def tearDown(self):
        self.tmp.cleanup()

    def test_classify_and_size_floor(self):
        self.assertEqual(psp.classify('Hips_pos'), 'hips_pos')
        self.assertEqual(psp.classify('RightFoot'), 'foot')
        self.assertEqual(psp.classify('LeftHand'), 'hand')
        self.assertEqual(psp.classify('Neck'), 'axial')
        self.assertTrue(psp.is_big('Hips_pos', 3.0))
        self.assertFalse(psp.is_big('Hips_pos', 2.9))
        self.assertTrue(psp.is_big('LeftFoot', 6.0))
        self.assertFalse(psp.is_big('LeftFoot', 5.9))

    def test_round_trip(self):
        driver, final = base_pose(), base_pose()
        write_stages(self.prefix, default_meta(), driver, final, {0: 'Run_Loop'})
        st = psp.load_stages(self.prefix)
        self.assertEqual(st.names, [b for b, _ in BONES])
        self.assertEqual(st.parents, [p for _, p in BONES])
        self.assertEqual(st.clips, {0: 'Run_Loop'})
        self.assertEqual(st.pose[0].shape, (N, len(BONES), 7))
        np.testing.assert_allclose(st.meta['dt'], DT, rtol=1e-6)
        q, p = st.fk(1, ['Head'])['Head']
        np.testing.assert_allclose(p[0], [0.0, 0.0, 0.7], atol=1e-6)  # Hips + 6 bones of 10 cm

    def test_a_pop_of_the_final_stage_is_found_and_attributed_to_the_leg_tail(self):
        driver, final = base_pose(), base_pose()
        k = 60
        final[k:, BONES.index(('RightFoot', 14)), 3:7] = quat_z(12.0)  # one 12 deg step only after the driver
        write_stages(self.prefix, default_meta(), driver, final, {0: 'Run_Loop'})
        st = psp.load_stages(self.prefix)
        valid = psp.moving_mask(st)
        pops_d, ser_d = psp.stage_pops(st, 0, valid)
        pops_f, ser_f = psp.stage_pops(st, 1, valid)
        self.assertEqual([p for p in pops_d if p[1] == 'RightFoot'], [])
        hit = [p for p in pops_f if p[0] == k and p[1] == 'RightFoot']
        self.assertTrue(hit, pops_f)
        self.assertAlmostEqual(hit[0][3], 12.0, delta=0.2)
        # the smooth 1 deg per feed turn of the left foot is no pop
        self.assertEqual([p for p in pops_f if p[1] == 'LeftFoot'], [])
        att = psp.attribute(st, {0: ser_d, 1: ser_f}, k, 'RightFoot', hit[0][2])
        self.assertEqual(att['where'], 'leg_tail')
        self.assertAlmostEqual(att['final'], 12.0, delta=0.2)
        self.assertLess(att['driver'], 0.1)

    def test_driver_step_against_the_clip_content(self):
        # clip frames (30 fps): clip 0 holds a 10 deg step of RightFoot between frames 30 and 31; clip 1 is smooth
        clip_n = 60
        meta = dict(t=np.zeros(N), dt=np.zeros(N), owned=np.ones(N), published=np.ones(N), tailed=np.ones(N),
                    clip=np.zeros(N), clipTime=np.zeros(N), playRate=np.ones(N), mvx=np.zeros(N), mvy=np.zeros(N))
        frames = base_pose()
        for f in range(N):
            c, i = divmod(f, clip_n)
            meta['clip'][f], meta['t'][f], meta['dt'][f] = c, i / 30.0, 1 / 30.0
            frames[f, BONES.index(('RightFoot', 14)), 3:7] = quat_z(10.0 if (c == 0 and i > 30) else 0.0)
        clips_prefix = str(pathlib.Path(self.tmp.name) / 'clips')
        write_stages(clips_prefix, meta, frames, frames, {0: 'StepClip', 1: 'SmoothClip'})
        content = psp.ClipContent(clips_prefix)
        self.assertAlmostEqual(content.step(0, 30.5 / 30, 31.0 / 30, 'RightFoot', 'parent'), 5.0, delta=0.05)
        self.assertAlmostEqual(content.step(1, 30.5 / 30, 31.0 / 30, 'RightFoot', 'parent'), 0.0, delta=0.05)
        self.assertIsNone(content.step(0, 70.0, 71.0, 'RightFoot', 'parent'))
        # the driver plays clip c at 60 fps and steps 10 deg at feed k: content of clip 0 explains it, clip 1 does not
        k = 62
        for cid, expected in ((0, 'clip_content'), (1, 'driver_blend')):
            driver = base_pose()
            driver[k:, BONES.index(('RightFoot', 14)), 3:7] = quat_z(10.0)
            m = default_meta()
            m['clip'][:] = cid
            m['clipTime'][:] = np.arange(N) * DT - k * DT + 31.0 / 30  # feed k ends at clip time 31/30 s
            write_stages(self.prefix, m, driver, driver.copy(), {0: 'StepClip', 1: 'SmoothClip'})
            st = psp.load_stages(self.prefix)
            _, ser_d = psp.stage_pops(st, 0)
            _, ser_f = psp.stage_pops(st, 1)
            att = psp.attribute(st, {0: ser_d, 1: ser_f}, k, 'RightFoot', 'parent', content)
            self.assertEqual(att['where'], expected, att)
        # without the clip frames the step stays with the driver
        att = psp.attribute(st, {0: ser_d, 1: ser_f}, k, 'RightFoot', 'parent')
        self.assertEqual(att['where'], 'driver')

    def test_hips_position_pop(self):
        driver, final = base_pose(), base_pose()
        final[:, 1, 1] = np.arange(N) * 0.01  # 1 cm per feed forward
        final[70:, 1, 1] += 0.08  # an 8 cm jump at feed 70
        write_stages(self.prefix, default_meta(), driver, final, {0: 'Run_Loop'})
        st = psp.load_stages(self.prefix)
        pops, _ = psp.stage_pops(st, 1, psp.moving_mask(st))
        hit = [p for p in pops if p[0] == 70 and p[1] == 'Hips_pos']
        self.assertTrue(hit, pops)
        self.assertAlmostEqual(hit[0][3], 8.0, delta=0.1)
        self.assertEqual(psp.summarize(hit, 2.0)['per_class_big'], {'hips_pos': 1})


class WorldStepsTest(unittest.TestCase):
    """world_steps / capsule_track: the drawn body in world space, the capsule's own steps and the steps relative to it."""

    @staticmethod
    def turn(n, rate_deg, body_offset_deg):
        """Model-space Hips / Head (q, p) turned by body_offset_deg [n] against the capsule; the capsule turns at rate_deg per feed."""
        q = np.array([quat_z(a) for a in body_offset_deg])
        p = np.zeros((n, 3))
        p[:, 2] = 1.0
        yaw = np.cumsum(np.full(n, rate_deg))
        return {'Hips': (q, p), 'Head': (q.copy(), p + 0.6)}, yaw

    def test_capsule_track_next_is_the_following_row(self):
        x, y, yaw = psp.capsule_track([0, 1, 2], [0, 0, 0], [0, 10, 20], 'next')
        self.assertEqual(list(x), [1, 2, 2])
        self.assertAlmostEqual(float(np.degrees(yaw[0])), 10.0)
        x, _, _ = psp.capsule_track([0, 1, 2], [0, 0, 0], [0, 10, 20], 'fed')
        self.assertEqual(list(x), [0, 1, 2])

    def test_a_body_step_that_cancels_a_capsule_step_is_clean_in_world_space(self):
        n = 120
        dt = np.full(n, DT)
        offset = np.zeros(n)
        offset[60] = -12.0  # the body turns back against the capsule for one frame ...
        fk, yaw = self.turn(n, 3.0, offset)
        yaw[60:] += 12.0    # ... exactly while the capsule jumps ahead (one frame early, then on time again)
        yaw[61:] -= 12.0
        res = psp.world_steps(fk, np.zeros(n), np.zeros(n), np.radians(yaw), dt)
        self.assertEqual([f for f, _, _ in res['world']], [])
        self.assertIn(60, res['relative'])
        self.assertIn(60, res['capsule']['yaw'])

    def test_a_body_step_alone_is_a_world_step_relative_to_the_capsule(self):
        n = 120
        dt = np.full(n, DT)
        offset = np.zeros(n)
        offset[60] = 12.0
        fk, yaw = self.turn(n, 3.0, offset)
        res = psp.world_steps(fk, np.zeros(n), np.zeros(n), np.radians(yaw), dt)
        rows = [f for f, _, _ in res['world']]
        self.assertIn(60, rows)
        table = psp.world_table(res, n * DT)
        self.assertEqual(table['axial']['relative'], len(rows))
        self.assertEqual(table['capsule_steps'], {'yaw': 0, 'pos': 0})


if __name__ == '__main__':
    unittest.main()
