"""Tests of skin_displacement_report.py (no game): skinning arithmetic on a stub mesh, the sheet renderer, and the whole report on a synthetic
recording (rig and graph of test_helper_graph_batch.py, recording of test_helper_residual_report.py). A last test loads the real body meshes
when their data files are present (skipped otherwise).

  python -I test_skin_displacement_report.py
"""
import json
import os
import pathlib
import sys
import tempfile
import types
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'ai_motion'))
import helper_graph_batch as hgb  # noqa: E402
import helper_residual_report as hrr  # noqa: E402
import skin_displacement_report as sdr  # noqa: E402
import test_helper_graph_batch as tg  # noqa: E402
import test_helper_residual_report as tr  # noqa: E402
import test_pose_capture_to_csv as tp  # noqa: E402
import skin_clip  # noqa: E402


def stub_assets():
    """One part, one vertex per rig bone (weight 1 on it) plus one vertex shared 50 / 50 by an upper and a lower helper."""
    names = list(tg.NAMES)
    n = len(names)
    pos = np.zeros((n + 1, 3))
    pos[:, 0] = np.linspace(-0.3, 0.3, n + 1)
    pos[:, 1] = 1.2
    js = np.zeros((n + 1, 2), dtype=np.int64)
    ws = np.zeros((n + 1, 2))
    js[:n, 0] = np.arange(n)
    ws[:n, 0] = 1.0
    js[n] = [names.index('l_SHL_0_JNT'), names.index('l_knee_mscl_JNT')]
    ws[n] = [0.5, 0.5]
    faces = np.array([[i, (i + 1) % n, n] for i in range(n)], dtype=np.int64)
    part = dict(name='stub', pos=pos, js=js, ws=ws, faces=faces)
    pre = np.stack([np.eye(4)] * n)
    return types.SimpleNamespace(SK=skin_clip, parts=[part], binds=[(names, pre)], faces=faces, n_vertices=n + 1, names=names)


def identity_models(m=1):
    return {n: np.broadcast_to(np.eye(4), (m, 4, 4)).copy() for n in tg.NAMES}


class SkinnerTest(unittest.TestCase):
    def setUp(self):
        self.sk = stub_assets()
        self.groups = ['upper', 'lower', 'elbow']
        self.joint_group = {'l_SHL_0_JNT': 0, 'r_SHL_0_JNT': 0, 'l_knee_mscl_JNT': 1, 'l_elbow_back_mscl_JNT': 2}
        self.skinner = sdr.Skinner(self.sk, self.groups, self.joint_group)

    def moved(self, shifts):
        before, after = identity_models(), identity_models()
        for n, t in shifts.items():
            after[n][0, :3, 3] = t
        return before, after

    def test_displacement_equals_the_bone_shift_times_weight_per_group(self):
        before, after = self.moved({'l_SHL_0_JNT': (0.01, 0, 0), 'l_knee_mscl_JNT': (0, 0.02, 0)})
        disp, weight = self.skinner.group_displacement(before, after, 0)
        names = self.sk.names
        v_shl, v_knee, v_shared, v_core = names.index('l_SHL_0_JNT'), names.index('l_knee_mscl_JNT'), len(names), names.index('Spine3')
        self.assertAlmostEqual(disp[0][v_shl], 1.0, places=9)                  # upper: 1 cm
        self.assertAlmostEqual(disp[1][v_knee], 2.0, places=9)                 # lower: 2 cm
        self.assertAlmostEqual(disp[0][v_shared], 0.5, places=9)               # shared vertex: half of each shift
        self.assertAlmostEqual(disp[1][v_shared], 1.0, places=9)
        self.assertAlmostEqual(disp[3][v_shared], np.hypot(0.5, 1.0), places=9)    # total of both groups, vector sum
        self.assertEqual(disp[:, v_core].max(), 0.0)                           # a vertex on a core bone does not move
        self.assertAlmostEqual(disp[0][v_knee], 0.0, places=12)                # a group moves only its own vertices
        self.assertGreater(weight[0][v_shared], sdr.AFFECT_WEIGHT)
        self.assertEqual(disp[2].max(), 0.0)                                   # the elbow group did not change

    def test_positions_follow_the_bones(self):
        before, after = self.moved({'l_SHL_0_JNT': (0.01, 0, 0)})
        p0 = self.skinner.positions(before, 0)
        p1 = self.skinner.positions(after, 0)
        i = self.sk.names.index('l_SHL_0_JNT')
        self.assertAlmostEqual(float(np.linalg.norm(p1[i] - p0[i])), 0.01, places=9)
        self.assertEqual(float(np.abs(p1[0] - p0[0]).max()), 0.0)

    def test_stats(self):
        disp = np.array([0.0, 0.2, 0.6, 1.2])
        s = sdr._stats(disp, np.array([0.0, 1.0, 1.0, 1.0]))
        self.assertEqual((s['max'], s['affected_vertices'], s['over_0p5_cm'], s['over_1_cm']), (1.2, 3, 2, 1))
        self.assertAlmostEqual(s['mean_affected'], 2.0 / 3.0, places=4)

    def test_sheet_is_composed_in_memory_and_written_once(self):
        with tempfile.TemporaryDirectory(dir=tp.scratch_root()) as d:
            before, after = self.moved({'l_SHL_0_JNT': (0.02, 0, 0)})
            disp, _ = self.skinner.group_displacement(before, after, 0)
            rows = [dict(label='frame 1', before=self.skinner.positions(before, 0), after=self.skinner.positions(after, 0), disp=disp[-1])]
            path = os.path.join(d, 'sheet.png')
            sdr.render_sheet(path, 'synthetic', rows, self.sk.faces, 2.0)
            self.assertEqual(os.listdir(d), ['sheet.png'])
            from PIL import Image
            with Image.open(path) as im:
                self.assertEqual(im.format, 'PNG')
                self.assertGreater(im.size[0], 1000)


class WholeReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.batch = hgb.BatchEvaluator('man', tg.synthetic_evaluator())
        cls._orig = (hrr.evaluator_for, sdr.assets_for)
        hrr.evaluator_for = lambda body: cls.batch
        sdr.assets_for = lambda body: stub_assets()

    @classmethod
    def tearDownClass(cls):
        hrr.evaluator_for, sdr.assets_for = cls._orig

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=tp.scratch_root())
        self.addCleanup(self.tmp.cleanup)
        self.dir = pathlib.Path(self.tmp.name)
        self.bundle = self.dir / 'bundle'
        self.bundle.mkdir()
        (self.bundle / 'owned.txt').write_text('\n'.join(tr.OWNED) + '\n', encoding='utf-8')

    def analyse(self, post, **kw):
        path = tr.write_recording(self.dir / tr.NAME, post=post)
        a = dict(out_dir=str(self.dir / 'out'), prefix='skin', bundle_dir=str(self.bundle), plan=None, clock_offset=0.0, window_seconds=1.0,
                 frames_per_window=4, limit_cm={'man': 0.8, 'woman': 0.9}, source='auto', sheets='all', sheet_frames=2, max_sheets=10,
                 heat_max_cm=2.0, jobs=1, sheet_dir=None)
        a.update(kw)
        os.makedirs(a['out_dir'], exist_ok=True)
        groups = hrr.ph.load_groups([path], hrr.ph.Config())
        return sdr.analyze_recording(a, groups[0]), a

    def test_graph_exact_post_stream_passes_with_zero_displacement(self):
        r, a = self.analyse('graph')
        self.assertEqual(r['status'], 'PASS')
        self.assertEqual(r['source'], 'post')
        self.assertEqual(r['covered_groups'], ['upper', 'lower', 'elbow', 'wrist_muscle'])
        for w in r['windows']:
            if w['verdict'] in ('PASS', 'NO_FRAMES'):
                self.assertLess(w['covered_max_cm'], 0.01, w['label'])
        pngs = [f for f in os.listdir(a['out_dir']) if f.endswith('.png')]
        self.assertTrue(pngs)
        self.assertEqual(sorted(w['sheet'] for w in r['windows'] if w.get('sheet')), sorted(pngs))

    def test_stale_post_stream_fails_with_the_displacement_in_the_json(self):
        r, a = self.analyse('stale')
        self.assertEqual(r['status'], 'FAIL')
        w = [x for x in r['windows'] if x['label'] == 'all'][0]
        self.assertEqual(w['verdict'], 'FAIL')
        self.assertIn('upper', w['over_limit_groups'])
        self.assertGreater(w['group_max_cm']['upper'], 5.0)               # the arms moved 4-9 cm; upper helpers hang off them
        fr = w['frames_checked'][0]
        self.assertTrue(fr['written'])
        self.assertGreater(fr['groups']['upper']['affected_vertices'], 0)
        json.dumps(r)                                                      # serialisable

    def test_without_stream_1_nothing_is_proved(self):
        r, a = self.analyse(None)
        self.assertEqual(r['status'], 'NOT_VERIFIABLE')
        self.assertEqual(r['source'], 'stale')
        w = [x for x in r['windows'] if x['label'] == 'all'][0]
        self.assertEqual(w['verdict'], 'NOT_VERIFIABLE')
        self.assertGreater(w['group_max_cm']['upper'], 5.0)               # the staleness the recompute has to remove

    def test_worst_frame_of_a_group_outside_the_bundle_is_added_and_sheets_can_go_elsewhere(self):
        sheets = self.dir / 'sheets'
        r, a = self.analyse('owned_only', sheet_dir=str(sheets))
        self.assertEqual(r['status'], 'PASS')                                # uncovered groups never gate
        w = [x for x in r['windows'] if x['label'] == 'all'][0]
        reasons = {why for fr in w['frames_checked'] for why in fr['picked_for']}
        self.assertTrue(any(why.startswith('largest residual of group ') for why in reasons), reasons)
        self.assertGreater(w['group_max_cm']['wrist_roll'] + w['group_max_cm']['hand'], 1.0)
        self.assertEqual([f for f in os.listdir(a['out_dir']) if f.endswith('.png')], [])
        self.assertTrue([f for f in os.listdir(sheets) if f.endswith('.png')])

    def test_limit_is_an_argument_and_sheets_can_be_switched_off(self):
        r, a = self.analyse('stale', limit_cm={'man': 1000.0, 'woman': 1000.0}, sheets='none')
        self.assertEqual(r['status'], 'PASS')
        self.assertEqual([f for f in os.listdir(a['out_dir']) if f.endswith('.png')], [])


class RealMeshTest(unittest.TestCase):
    def test_real_meshes_load_and_a_zero_change_moves_nothing(self):
        for body in ('man', 'woman'):
            try:
                sk = sdr.SkinAssets(body)
            except (FileNotFoundError, OSError):
                self.skipTest('the %s body meshes are not present (data files / UTPP_PROJECT_ROOT)' % body)
            try:
                ev = hgb.BatchEvaluator(body)
            except (FileNotFoundError, OSError):
                self.skipTest('the %s deformer graph files are not present' % body)
            helpers = [n for n in ev.names if n not in ev.core_names]
            groups = ['upper']
            skinner = sdr.Skinner(sk, groups, {n: 0 for n in helpers})
            W = {n: np.broadcast_to(np.eye(4), (1, 4, 4)).copy() for n in ev.names}
            disp, _ = skinner.group_displacement(W, W, 0)
            self.assertEqual(float(disp.max()), 0.0)
            self.assertEqual(disp.shape[1], sk.n_vertices)
            self.assertGreater(sk.n_vertices, 10000)


if __name__ == '__main__':
    unittest.main(verbosity=2)
