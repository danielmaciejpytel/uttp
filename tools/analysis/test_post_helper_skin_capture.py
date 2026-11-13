"""Focused provenance and geometry checks for final LOCAL helper captures."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from replay_body_skin import cname, read_capture, replay


class PostHelperSkinCaptureTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[3] / 'Build' / 'post-helper-parser-tests'
        root.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=root)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def capture(self, post=True, last=99):
        path = self.root / ('post.csv' if post else 'legacy.csv')
        prefix = 'local' if post else 's0'
        fields = ['ticks','desc','thread','rig','generation','rigCount','name','gameIndex']
        row = dict(zip(fields, [100,200,3,400,1,1,'Root',0]))
        if post:
            row.update(receiptGeneration=2, receiptCycle=3, receiptStartedTicks=90, receiptLastTicks=last)
        row.update({f'{prefix}_{i}': v for i,v in enumerate([.1,0,0,0,0,0,0,1,1,1,1,0])})
        with path.open('w', newline='') as f:
            if post:
                f.write('# stage=post_helper_apply_after_commit\n')
            f.write(f'# rig_bone index=0 hash={cname("Root"):X} parent=-1\n')
            writer = csv.DictWriter(f, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
        return path

    def test_stage_cannot_be_silently_relabelled(self):
        with self.assertRaises(ValueError):
            read_capture(self.capture(), 1)
        with self.assertRaises(ValueError):
            read_capture(self.capture(False), 'post-helper')

    def test_future_receipt_rejected(self):
        with self.assertRaisesRegex(ValueError, 'receipt'):
            read_capture(self.capture(last=101), 'post-helper')

    def test_missing_local_record_rejected(self):
        path = self.capture()
        path.write_text(path.read_text().replace('local_11', 'unknown_11'))
        with self.assertRaisesRegex(ValueError, 'schema'):
            read_capture(path, 'post-helper')

    def test_final_pose_is_actually_skinned(self):
        skin = self.root / 'skin.npz'
        meta = self.root / 'skin.json'
        vertices = np.array([[0.,0,0],[0,1,0],[0,0,1]])
        np.savez(skin, jointBindMatrices=np.eye(4)[None], inverseBindMatrices=np.eye(4)[None],
                 primitive_0_positions=vertices, primitive_0_joints=np.zeros((3,1), dtype=int),
                 primitive_0_weights=np.ones((3,1)), primitive_0_indices=np.array([0,1,2]))
        meta.write_text(json.dumps({'jointNames':['Root'], 'primitives':[
            {'key':'primitive_0','meshNodeTransform':np.eye(4).tolist()}]}))
        out = self.root / 'out'
        report = replay(skin, meta, self.capture(), out, [0], 'post-helper')
        self.assertEqual(report['stage'], 'post-helper')
        with np.load(out/'frame-0000-post-helper.npz') as data:
            np.testing.assert_allclose(data['primitive_0_positions'], vertices + [.1,0,0], atol=1e-7)
        self.assertFalse((out/'frame-0000-s1.npz').exists())


if __name__ == '__main__':
    unittest.main()
