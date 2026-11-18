"""Tests of replay_deformation.py on synthetic stage files and a stand-in skinning rig (no meshes, no replay tool).

  python -I test_replay_deformation.py
"""
import pathlib
import sys
import tempfile
import types
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import replay_deformation as rd  # noqa: E402

BONES = [('Root', -1), ('Trajectory', 0), ('Hips', 0), ('LeftUpLeg', 2), ('LeftLeg', 3), ('RightUpLeg', 2), ('RightLeg', 5)]
META = ['t', 'dt', 'published', 'tailed', 'clip', 'clipTime']
N = 40
CLIPS = {0: 'M_Neutral_Walk_Loop_F', 1: 'M_Neutral_Walk_Stop_F_Lfoot'}


def write_stages(prefix, meta, raw, final):
    with open(prefix + '.txt', 'w', encoding='utf-8', newline='\n') as f:
        f.write('UTPP_POSE_STAGES 1\nbones %d\nmeta %d %s\n' % (len(BONES), len(META), ' '.join(META)))
        for i, (name, parent) in enumerate(BONES):
            f.write('bone %d %s %d\n' % (i, name, parent))
        for cid, name in CLIPS.items():
            f.write('clip %d %s\n' % (cid, name))
    rows = np.concatenate([np.stack([meta[k] for k in META], 1), raw.reshape(N, -1), final.reshape(N, -1)], 1)
    rows.astype(np.float32).tofile(prefix + '.bin')


def poses(knee_deg):
    """[N, nb, 7]: identity rotations except the left knee (x axis) at knee_deg[f]."""
    p = np.zeros((N, len(BONES), 7))
    p[:, :, 6] = 1.0
    p[:, 2:, 2] = -0.4
    a = np.radians(np.asarray(knee_deg, dtype=np.float64)) / 2.0
    p[:, 4, 3] = np.sin(a)
    p[:, 4, 6] = np.cos(a)
    return p


def default_meta():
    clip = np.zeros(N)
    clip[30:] = 1
    pub = np.ones(N)
    pub[5] = 0
    return dict(t=np.arange(N) / 60.0, dt=np.full(N, 1 / 60.0), published=pub, tailed=np.ones(N), clip=clip,
                clipTime=np.arange(N) / 60.0)


class FakeRig:
    """Skin = the knee local angle; region values grow with it (knee) or stay at 1 (others)."""

    def __init__(self):
        self.calls = 0

    def skin(self, core):
        self.calls += 1
        q = core['LeftLeg'][:3, :3]
        ang = np.degrees(np.arccos(np.clip(q[1, 1], -1, 1)))
        return ang, {'Root': core['Root']}

    def metrics(self, ang, W, refW, refstate):
        m = {r: dict(sev_p999=1.0, sev=1.0, area_min=1.0, stretch_cm_max=0.0) for r in rd.REGIONS}
        m['knee'] = dict(sev_p999=1.0 + ang / 10.0, sev=1.0 + ang / 5.0, area_min=1.0 / (1.0 + ang / 10.0), stretch_cm_max=ang / 20.0)
        return m, None, None


def mat(t, q, s=(1, 1, 1)):
    x, y, z, w = q
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


class ReplayDeformationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = pathlib.Path(self.tmp.name)
        self.rig = FakeRig()
        fake_mb = types.SimpleNamespace(SK=types.SimpleNamespace(mat=mat))
        rd._RIGS['woman'] = (fake_mb, self.rig, None, None, {n for n, _ in BONES})

    def tearDown(self):
        rd._RIGS.pop('woman', None)
        self.tmp.cleanup()

    def write(self, name, knee_final, knee_raw=None):
        prefix = str(self.dir / name)
        knee_raw = np.zeros(N) if knee_raw is None else knee_raw
        write_stages(prefix, default_meta(), poses(knee_raw), poses(knee_final))
        return prefix

    def test_stage_file_reads_meta_and_both_stages(self):
        knee = np.linspace(0, 60, N)
        sf = rd.StageFile(self.write('a', knee, knee_raw=np.full(N, 7.0)))
        self.assertEqual(sf.n, N)
        self.assertEqual(sf.names[:3], ['Root', 'Trajectory', 'Hips'])
        self.assertEqual(sf.clip_name(sf.meta('clip')[35]), CLIPS[1])
        self.assertEqual(sf.clip_name(np.nan), '-')
        self.assertAlmostEqual(np.degrees(2 * np.arcsin(sf.pose(20, 'final')[4, 3])), knee[20], places=3)
        self.assertAlmostEqual(np.degrees(2 * np.arcsin(sf.pose(20, 'raw')[4, 3])), 7.0, places=3)

    def test_stage_core_keeps_root_and_trajectory(self):
        sf = rd.StageFile(self.write('a', np.zeros(N)))
        core = rd.stage_core(sf.names, sf.pose(0), {'Root', 'Trajectory', 'LeftLeg'}, mat)
        self.assertEqual(set(core), {'Root', 'Trajectory', 'LeftLeg'})
        np.testing.assert_allclose(core['LeftLeg'][:3, 3], [0, 0, -0.4], atol=1e-6)

    def test_rows_sample_and_windows(self):
        valid = rd.valid_rows(default_meta()['published'], np.ones(N))
        self.assertFalse(valid[5])
        s = rd.sample_rows(valid, 4)
        self.assertEqual(s.tolist(), list(range(0, N, 4)))
        valid2 = valid.copy()
        valid2[8] = False
        self.assertNotIn(8, rd.sample_rows(valid2, 4).tolist())
        scores = np.zeros(len(s))
        scores[s.tolist().index(4)] = 5.0
        w = rd.window_rows(s, scores, valid, 1, 2)
        self.assertEqual(w.tolist(), [2, 3, 4, 6])  # row 5 unpublished
        self.assertEqual(rd.window_rows(s, scores, valid, 0, 2).tolist(), [])

    def test_identity_gives_zero_deltas_and_skins_once(self):
        knee = np.linspace(0, 80, N)
        a, b = self.write('a', knee), self.write('b', knee)
        rows = np.arange(0, N, 2)
        vals = rd.skin_rows(('woman', [a, b], rows, 'final', ''))
        self.assertEqual(self.rig.calls, len(rows))  # the identical after pose is not skinned again
        np.testing.assert_array_equal(vals[0], vals[1])
        cls = np.array(['loop'] * len(rows))
        pd = rd.paired_delta(vals[0], vals[1], cls)
        for r in rd.REGIONS:
            self.assertEqual(pd['all'][r]['max'], 0.0)
            self.assertEqual(pd['all'][r]['worse_share'], 0.0)
        st = rd.pool_stats(vals[0], vals[0], cls, cls)
        d = rd.stat_delta(st, st)
        self.assertTrue(all(v == 0.0 for v in d['all']['knee'].values()))

    def test_worse_after_side_is_reported(self):
        knee = np.linspace(0, 80, N)
        a, b = self.write('a', knee), self.write('b', knee + np.where(np.arange(N) >= 30, 10.0, 0.0))
        rows = np.arange(0, N, 2)
        vals = rd.skin_rows(('woman', [a, b], rows, 'final', ''))
        sf = rd.StageFile(a)
        cls = np.array([rd.clip_class(sf.clip_name(c)) for c in sf.meta('clip')[rows]])
        self.assertEqual(set(cls), {'loop', 'stop'})
        pd = rd.paired_delta(vals[0], vals[1], cls)
        self.assertAlmostEqual(pd['stop']['knee']['max'], 1.0, places=3)
        self.assertEqual(pd['loop']['knee']['max'], 0.0)
        self.assertEqual(pd['all']['elbow']['max'], 0.0)
        before = rd.pool_stats(vals[0], vals[0], cls, cls)
        after = rd.pool_stats(vals[1], vals[1], cls, cls)
        self.assertAlmostEqual(after['stop']['knee']['max'] - before['stop']['knee']['max'], 1.0, places=3)
        d = rd.stat_delta(before, after)
        self.assertGreater(d['stop']['knee']['max'], 0.99)
        self.assertGreater(d['stop']['knee']['max_excess_pct'], 0.0)
        self.assertNotIn('pivot', after)

    def test_pool_uses_sample_for_percentiles_and_all_rows_for_max(self):
        sample = np.ones((10, len(rd.REGIONS), len(rd.STATS)))
        every = np.concatenate([sample, np.full((1, len(rd.REGIONS), len(rd.STATS)), 9.0)])
        st = rd.pool_stats(sample, every, np.array(['loop'] * 10), np.array(['loop'] * 11))
        self.assertEqual(st['all']['knee']['p999'], 1.0)
        self.assertEqual(st['all']['knee']['max'], 9.0)
        self.assertEqual(st['all']['knee']['n'], 10)


if __name__ == '__main__':
    unittest.main()
