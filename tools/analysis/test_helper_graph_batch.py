"""Tests of helper_graph_batch.py: the frame-batched deformer graph evaluator must give the scalar evaluator's numbers.

  python -I test_helper_graph_batch.py

The synthetic graph below holds one node of every operator the evaluator supports (set, point, orient, both aims, both splines, bounce with
position / scale / rotation channels and a track, twist, translation limit, a stack extender) on a small rig; it needs no game data. The
last test compares the real graphs of both bodies when the data files of the deformer graphs are present (it is skipped otherwise).
"""
import pathlib
import sys
import types
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import evaluate_body_deformer_graph as E  # noqa: E402
import helper_graph_batch as hgb  # noqa: E402

# rig: name, parent, local translation (m), core?
RIG = [('Hips', -1, (0, 0, 1.0), True), ('Spine1', 0, (0, 0, 0.12), True), ('Spine3', 1, (0, 0.01, 0.2), True),
       ('LeftArm', 2, (0.18, 0, 0.1), True), ('RightArm', 2, (-0.18, 0, 0.1), True),
       ('l_SHL_0_JNT', 2, (0.05, 0, 0.1), False), ('l_chest_front_A_bot_out_JNT', 2, (0.04, 0.08, 0.05), False),
       ('l_knee_mscl_JNT', 2, (0.06, 0.02, 0.0), False), ('l_elbow_back_mscl_JNT', 2, (0.1, -0.05, 0.1), False),
       ('l_wrist_A_mscl_JNT', 2, (0.12, 0.0, 0.05), False), ('r_SHL_0_JNT', 2, (-0.05, 0, 0.1), False),
       ('r_deltoid_back_top_out_JNT', 2, (-0.1, -0.04, 0.1), False), ('r_butterfly_top_CRV_top_out_JNT', 2, (-0.03, 0.05, 0.12), False),
       ('l_Wrist_0_JNT', 3, (0.15, 0, 0), False), ('l_thumb_side_mscl_JNT', 2, (0.2, 0.1, 0.1), False)]
NAMES = [r[0] for r in RIG]
HELPERS = [r[0] for r in RIG if not r[3]]


def nm(n):
    return {'name': {'$value': n}}


def xf(t=(0, 0, 0), q=(0, 0, 0, 1), s=(1, 1, 1)):
    return {'Translation': dict(zip('XYZ', t)), 'Rotation': dict(zip(['i', 'j', 'k', 'r'], q)), 'Scale': dict(zip('XYZ', s))}


def vec3(v):
    return dict(zip('XYZ', v))


def node(t, **kw):
    d = {'$type': 'animAnimNode_' + t, 'weight': 1}
    d.update(kw)
    return d


def synthetic_evaluator():
    """A scalar Evaluator object (no files read) holding the synthetic graph; execution order = descending handle."""
    ev = E.Evaluator.__new__(E.Evaluator)
    ev.allow_rotation_bounce = True
    ev.body = 'man'
    ev.names = NAMES
    ev.parents = [r[1] for r in RIG]
    ev.refLocal = {r[0]: E.tm(xf(r[2])) for r in RIG}
    ev.coreNames = {r[0] for r in RIG if r[3]}
    ev.poseChainAssertions = []
    nodes = []
    nodes.append(node('StackTransformsExtender', snapMethods=['NoSnapping'], snapToReferenceValues=[False], offsetToReferenceValues=[False],
                      offsetSpaceBones=[nm('None')], offsets=[xf()],
                      transformInfos=[{'name': {'$value': 'logical_a'}, 'parentName': {'$value': 'Spine3'}, 'referenceTransformLs': xf((0.0, 0.03, 0.02))},
                                      {'name': {'$value': 'logical_b'}, 'parentName': {'$value': 'logical_a'}, 'referenceTransformLs': xf((0.01, 0.0, 0.0))}]))
    nodes.append(node('SetBoneTransform', entries=[
        dict(transformToChange=nm('l_SHL_0_JNT'), sourceBone=nm('LeftArm'), offsetSpaceBone=nm('Spine3'), setMethod='RotationOnly',
             snapToReference=False, offsetToReference=False, offset=xf((0.0, 0.0, 0.01), (0.0, 0.0, 0.0871557, 0.9961947))),
        dict(transformToChange=nm('l_SHL_0_JNT'), sourceBone=nm('logical_b'), offsetSpaceBone=nm('None'), setMethod='TranslationOnly',
             snapToReference=False, offsetToReference=False, offset=xf()),
        dict(transformToChange=nm('l_thumb_side_mscl_JNT'), sourceBone=nm('RightArm'), offsetSpaceBone=nm('None'), setMethod='WholeTransform',
             snapToReference=False, offsetToReference=False, offset=xf())]))
    nodes.append(node('PointConstraint', transformIndex=nm('l_chest_front_A_bot_out_JNT'),
                      inputWeightedTransforms=[{'transform': nm('Spine3')}, {'transform': nm('LeftArm')}, {'transform': nm('l_SHL_0_JNT')}],
                      preprocessedWeights=[0.25, 0.5, 0.25]))
    nodes.append(node('OrientConstraint', transformIndex=nm('l_knee_mscl_JNT'),
                      inputWeightedTransforms=[{'transform': nm('Spine3')}, {'transform': nm('LeftArm')}, {'transform': nm('RightArm')}],
                      preprocessedWeights=[1.0, 0.4, 0.3]))
    nodes.append(node('AimConstraint_ObjectUp', transformIndex=nm('l_elbow_back_mscl_JNT'), targetTransform=nm('LeftArm'), upTransform=nm('Spine3'),
                      forwardAxisLS=vec3((1, 0, 0)), upAxisLS=vec3((0, 0, 1))))
    nodes.append(node('AimConstraint_ObjectRotationUp', transformIndex=nm('l_wrist_A_mscl_JNT'), targetTransform=nm('l_chest_front_A_bot_out_JNT'),
                      upTransform=nm('Spine3'), forwardAxisLS=vec3((0, 1, 0)), upAxisLS=vec3((0, 0, 1)), upTransformVector=vec3((0, 0, 1))))
    nodes.append(node('SimpleSpline', constrainedTransform=nm('r_SHL_0_JNT'), startTransform=nm('Spine3'), middleTransform=nm('RightArm'),
                      endTransform=nm('LeftArm'), progressMode='Static', defaultProgress=0.35, progressTrack=nm('None')))
    nodes.append(node('SimpleBounce', startTransform=nm('Spine3'), endTransform=nm('RightArm'), multiplier=3.0, negativeMultiplier=-1.5,
                      delay=0.02, smoothStep=0.05, offset=0.5, outputDriverTrack=nm('None'),
                      trackOutputs=[{'targetTrack': nm('drive_a'), 'multiplier': 0.4}],
                      transformOutputs=[
                          dict(targetTransform=nm('r_deltoid_back_top_out_JNT'), parentTransform=nm('Spine3'),
                               channelEntries=[dict(transformChannel='PosZ', multiplier=0.5), dict(transformChannel='RotX', multiplier=40.0),
                                               dict(transformChannel='RotY', multiplier=-25.0)]),
                          dict(targetTransform=nm('r_butterfly_top_CRV_top_out_JNT'), parentTransform=nm('Spine3'),
                               channelEntries=[dict(transformChannel='PosY', multiplier=-0.3), dict(transformChannel='ScaleX', multiplier=1.0)])]))
    nodes.append(node('SimpleSpline', constrainedTransform=nm('r_butterfly_top_CRV_top_out_JNT'), startTransform=nm('Spine3'),
                      middleTransform=nm('LeftArm'), endTransform=nm('RightArm'), progressMode='FloatTrack', defaultProgress=0.0,
                      progressTrack=nm('drive_a')))
    nodes.append(node('TwistConstraint', frontAxis='PositiveX', transformA=nm('Spine3'), transformB=nm('LeftArm'),
                      outputs=[dict(twistedTransform=nm('l_Wrist_0_JNT'), twistAxis='PositiveX', positiveScale=0.6, negativeScale=0.4)]))
    lim = lambda a, b: dict(useMin=True, min=a, useMax=True, max=b)           # noqa: E731
    nodes.append(node('TranslationLimit', constrainedTransform=nm('l_thumb_side_mscl_JNT'), parentTransform=nm('None'),
                      limitOnXAxis=lim(-0.1, 0.05), limitOnYAxis=dict(useMin=False, min=0, useMax=True, max=0.02), limitOnZAxis=lim(0.0, 0.5)))
    nodes.append(node('Root'))                                                # skipped type
    ev.nodes = {h: n for h, n in zip(range(len(nodes), 0, -1), nodes)}        # handle order = list order (descending handle first)
    return ev


def random_core_locals(ev, n, seed, max_deg=35.0, max_cm=3.0):
    return hgb.perturbed_core_locals(ev, n, seed, max_deg=max_deg, max_cm=max_cm)


def model_matrices(ev, locs):
    """Scalar reference for every frame: [{name: 4x4}]."""
    n = next(iter(locs.values())).shape[0]
    out = []
    for k in range(n):
        out.append(ev.run({b: m[k] for b, m in locs.items() if b in ev.coreNames}))
    return out


class SyntheticGraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ev = synthetic_evaluator()
        cls.batch = hgb.BatchEvaluator('man', cls.ev)

    def test_matches_scalar_evaluator_on_random_poses(self):
        locs = random_core_locals(self.ev, 60, 7)
        models, bad = self.batch.run(locs)
        self.assertFalse(bad.any())
        ref = model_matrices(self.ev, locs)
        worst_p = worst_r = 0.0
        compared = 0
        for k, r in enumerate(ref):
            for name, mat in r.items():
                got = models[name][k]
                worst_p = max(worst_p, float(np.linalg.norm(got[:3, 3] - mat[:3, 3])))
                worst_r = max(worst_r, float(hgb.rotation_error_deg(got, mat)))
                compared += 1
        self.assertGreater(compared, 60 * len(NAMES))                      # logical nodes included
        self.assertLess(worst_p, 1e-9)
        self.assertLess(worst_r, 1e-6)

    def test_helpers_actually_move(self):
        """The synthetic graph is not trivial: the helpers differ between a pose and a rotated pose."""
        a = random_core_locals(self.ev, 4, 1)
        b = random_core_locals(self.ev, 4, 2)
        ma, _ = self.batch.run(a, want=HELPERS)
        mb, _ = self.batch.run(b, want=HELPERS)
        moved = [h for h in HELPERS if np.abs(ma[h] - mb[h]).max() > 1e-3]
        self.assertGreaterEqual(len(moved), len(HELPERS) - 1)

    def test_chunk_independence(self):
        locs = random_core_locals(self.ev, 20, 3)
        full, _ = self.batch.run(locs, want=HELPERS)
        half = {k: v[7:12] for k, v in locs.items()}
        part, _ = self.batch.run(half, want=HELPERS)
        for h in HELPERS:
            np.testing.assert_allclose(part[h], full[h][7:12], atol=1e-12)

    def test_nonfinite_frames_are_flagged_not_dropped(self):
        locs = random_core_locals(self.ev, 6, 4)
        locs['LeftArm'][2, 0, 0] = np.nan
        models, bad = self.batch.run(locs, want=HELPERS)
        self.assertEqual(list(np.nonzero(bad)[0]), [2])
        ok = [k for k in range(6) if k != 2]
        ref = model_matrices(self.ev, {b: m[ok] for b, m in locs.items()})
        for j, k in enumerate(ok):
            for h in HELPERS:
                np.testing.assert_allclose(models[h][k], ref[j][h], atol=1e-9)

    def test_degenerate_aim_skips_only_its_own_write(self):
        locs = random_core_locals(self.ev, 5, 5)
        arm = locs['LeftArm']                                              # aim target == aim source position -> zero direction
        # the aimed bone is a child of Spine3; put the LeftArm model position exactly on it: make Spine3 local identity and LeftArm
        # local equal to the elbow helper's reference local
        locs['Spine3'][1] = np.eye(4)
        arm[1] = self.ev.refLocal['l_elbow_back_mscl_JNT']
        models, bad = self.batch.run(locs, want=HELPERS)
        self.assertFalse(bad.any())
        ref = model_matrices(self.ev, locs)
        self.ev.run({b: m[1] for b, m in locs.items() if b in self.ev.coreNames})
        self.assertTrue(any(s['reason'] == 'degenerate aim' for s in self.ev.skipped))   # the degenerate branch was really taken
        for k in range(5):
            for h in HELPERS:
                np.testing.assert_allclose(models[h][k], ref[k][h], atol=1e-9)

    def test_antipodal_twist_is_refused_like_the_scalar_evaluator(self):
        locs = random_core_locals(self.ev, 3, 6)
        # B's twist axis flipped against A's: swing of two opposite vectors (c < -0.999999) -> the scalar evaluator raises
        a = np.eye(4)
        b = np.eye(4)
        b[:3, :3] = np.diag([-1.0, -1.0, 1.0])                             # 180 degrees about z: front axis X maps to -X
        for nme, m in (('Spine1', a), ('Spine3', a)):
            locs[nme][0] = m
        locs['Hips'][0] = np.eye(4)
        locs['LeftArm'][0] = b
        models, bad = self.batch.run(locs, want=HELPERS)
        with self.assertRaises(ValueError):
            self.ev.run({k: v[0] for k, v in locs.items() if k in self.ev.coreNames})
        self.assertTrue(bad[0])
        self.assertFalse(bad[1:].any())

    def test_missing_core_input_is_an_error(self):
        locs = random_core_locals(self.ev, 2, 8)
        del locs['RightArm']
        with self.assertRaises(ValueError):
            self.batch.run(locs)

    def test_rotation_error_is_sign_safe_and_small_angle_accurate(self):
        a = np.eye(4)
        th = np.radians(0.003)
        b = np.eye(4)
        b[:3, :3] = [[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]]
        self.assertAlmostEqual(float(hgb.rotation_error_deg(a, b)), 0.003, places=9)
        b2 = b.copy()
        b2[:3, :3] *= 2.0                                                   # scale does not count
        self.assertAlmostEqual(float(hgb.rotation_error_deg(a, b2)), 0.003, places=9)

    def test_quat_matrices_roundtrip(self):
        q = np.array([0.1, -0.2, 0.3, 0.9])
        q /= np.linalg.norm(q)
        row = np.array([0.1, 0.2, 0.3, 0.0, *q, 1.5, 1.0, 0.5, 0.0])
        m = hgb.quat_matrices(row)
        np.testing.assert_allclose(m[:3, 3], [0.1, 0.2, 0.3])
        np.testing.assert_allclose(np.linalg.norm(m[:3, :3], axis=0), [1.5, 1.0, 0.5])
        r = m[:3, :3] / np.array([1.5, 1.0, 0.5])
        q2 = hgb._quat_from_rot(r)
        self.assertAlmostEqual(abs(float(np.dot(q, q2))), 1.0, places=9)


class RealGraphTest(unittest.TestCase):
    def test_real_graphs_match_the_scalar_evaluator(self):
        for body in ('man', 'woman'):
            graph, rig, core = E.BODY_FILES[body]
            if not (pathlib.Path(graph).is_file() and pathlib.Path(rig).is_file() and pathlib.Path(core).is_file()):
                self.skipTest('the data files of the %s deformer graph are not present' % body)
            res = hgb.compare_with_scalar(body, n=16, seed=3)
            self.assertEqual(res['compared'] + res['scalar_refused'], 16)
            self.assertGreater(res['compared'], 8)
            self.assertLess(res['max_pos_cm'], 1e-8, res)
            self.assertLess(res['max_rot_deg'], 1e-6, res)


if __name__ == '__main__':
    unittest.main(verbosity=2)
