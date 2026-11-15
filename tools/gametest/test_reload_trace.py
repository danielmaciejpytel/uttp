"""Tests of reload_trace.py on a synthetic record file (no game, no data files).

  python -I test_reload_trace.py
"""
from __future__ import annotations

import io
import math
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reload_trace as rt  # noqa: E402

HZ = 10_000_000
Q0 = 5_000_000_000
LOCAL = (2026, 11, 15, 2, 14, 10, 0)
OWN_L = [0.143, -0.087, 0.020]
OWN_R = [0.143, -0.087, -0.020]


def frame(k: int, **over) -> bytes:
    """Frame k of a standing rifle reload at 60 Hz: accepted, V's own clavicles, a smooth left elbow."""
    ang = math.radians(2.0 * k)
    f = dict(ticks=Q0 + HZ + k * HZ // 60, sequence=k, pushNumber=k, path=1, exit=10, overlay=1, action=2,
             flags=(1 << rt.FLAG_BITS['ownerMapping']) | (1 << rt.FLAG_BITS['written']),
             mask=0b11111111100,  # arms, hands, chest, head, weapon; clavicles cleared
             snapWeight=1.0, upperScale=0.0, lowerScale=1.0, contactWeight=1.0, corrIn=1.0, corrFinal=1.0, placementGate=1.0,
             girdleWeight=1.0, holdAmp=1.0, holdWeight=1.0, holdGirdleWeight=1.0,
             clavGame=OWN_L + OWN_R, clavIn=OWN_L + OWN_R, clavOwn=OWN_L + OWN_R, clavOut=OWN_L + OWN_R,
             shoulder=[0.18, 0.0, 1.45], elbowSolve=[0.18 + 0.28 * math.cos(ang), 0.28 * math.sin(ang), 1.30],
             elbowOut=[0.18 + 0.28 * math.cos(ang), 0.28 * math.sin(ang), 1.30],
             handGameRot=[0, 0, 0, 1], handTargetRot=[0, 0, 0, 1], handOutRot=[0, 0, 0, 1], chestRot=[0, 0, 0, 1],
             anat=[10.0, 20.0, 30.0] * 5)
    f.update(over)
    return rt.pack_record(**f)


def write(path: str, frames: list[bytes], record_bytes: int = rt.RECORD_BYTES, tail: bytes = b'') -> None:
    with open(path, 'wb') as fo:
        fo.write(rt.pack_header(HZ, Q0, LOCAL, record_bytes))
        for b in frames:
            fo.write(b)
        fo.write(tail)


class LayoutTest(unittest.TestCase):
    def test_offsets_match_the_plugin(self):
        # the same numbers as tests/ReloadTraceTest.cpp (offsetof in the plugin's Record)
        off = rt.lane_offsets()
        want = dict(ticks=0, sequence=8, flags=16, mask=20, path=24, action=28, snapWeight=32, corrIn=48, holdAmp=76, stockShift=92,
                    clavGame=100, clavOut=172, jointReqDeg=196, jointMaxDeg=220, jointLimited=224, shoulder=228, guide=300,
                    elbowSolve=312, handOutRot=348, reachClavDeg=364, anat=372, _end=432)
        for k, v in want.items():
            self.assertEqual(off[k], v, k)
        self.assertEqual(len(rt.pack_record()), rt.RECORD_BYTES)
        self.assertEqual(len(rt.pack_header(HZ, Q0, LOCAL)), rt.HEADER_BYTES)


class ReaderTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, 'utpp-reload-2026-11-15-02-14-10.bin')

    def tearDown(self):
        self.dir.cleanup()

    def episode(self) -> list[bytes]:
        frames = []
        for k in range(20):
            over = {}
            if 3 <= k <= 14:  # the game's girdle shown: left clavicle 5 cm off V's own
                over['clavOut'] = [OWN_L[0] - 0.05, OWN_L[1], OWN_L[2]] + OWN_R
            if k >= 5:  # the clavicle bits come back into the hand-back mask
                over['mask'] = 0b11111111111
            if k == 8:  # the solved elbow swings 60 deg about the shoulder
                a = math.radians(2.0 * k + 60.0)
                over['elbowSolve'] = [0.18 + 0.28 * math.cos(a), 0.28 * math.sin(a), 1.30]
            if k == 10:  # solve reject, replayed
                over.update(exit=9, solveReject=5, flags=(1 << rt.FLAG_BITS['ownerMapping']) | (1 << rt.FLAG_BITS['written'])
                            | (1 << rt.FLAG_BITS['replay']))
            if k >= 15:  # correction weight steps 1 -> 0.5
                over.update(corrFinal=0.5)
            frames.append(frame(k, **over))
        return frames

    def test_flags_per_frame(self):
        write(self.path, self.episode())
        header, recs = rt.read_file(self.path)
        self.assertEqual(len(recs), 20)
        self.assertEqual(header['truncatedBytes'], 0)
        self.assertEqual(recs[0]['time'].strftime('%H:%M:%S.%f')[:-3], '02:14:11.000')
        eps = rt.episodes(recs)
        self.assertEqual(len(eps), 1)
        rows = rt.analyse(eps[0])
        flagged = {k: set(d['flags']) for k, d in enumerate(rows) if d['flags']}
        self.assertEqual({k for k, f in flagged.items() if 'CLAV' in f}, set(range(3, 15)))
        self.assertEqual({k for k, f in flagged.items() if 'MASK' in f}, {5})
        self.assertEqual({k for k, f in flagged.items() if 'ELB' in f}, {8, 9})  # out and back
        self.assertEqual({k for k, f in flagged.items() if 'REJ' in f}, {10})
        self.assertIn('REPLAY', flagged[10])
        self.assertEqual({k for k, f in flagged.items() if 'WSTEP' in f}, {15})
        self.assertAlmostEqual(rows[5]['dL'], 0.05, places=4)
        self.assertEqual(rt.mask_text(recs[5]['mask']), 'CAFHSNW')
        self.assertEqual(rt.mask_text(recs[0]['mask']), '.AFHSNW')

    def test_episodes_split_on_gaps_and_cli(self):
        frames = self.episode()
        later = [frame(k + 120) for k in range(5)]  # 2 s later
        write(self.path, frames + later)
        _, recs = rt.read_file(self.path)
        self.assertEqual([len(e) for e in rt.episodes(recs)], [20, 5])
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = rt.main([self.dir.name, '--flagged'])
        out = buf.getvalue()
        self.assertEqual(code, 0)
        self.assertIn('episode 1:', out)
        self.assertIn('episode 2:', out)
        self.assertIn('owner mapping', out)
        self.assertIn('solve/contact', out)
        buf = io.StringIO()
        with redirect_stdout(buf):
            rt.main([self.path, '--from', '02:14:13', '--summary'])
        self.assertIn('episode 1:', buf.getvalue())
        self.assertNotIn('episode 2:', buf.getvalue())

    def test_elbow_azimuth_separates_guide_and_solve(self):
        # hand target straight down from the shoulder; frame 1: the solved elbow swings 90 deg about that axis, the guide stays
        common = dict(handTarget=[0.18, 0.0, 1.15], guide=[0.18, 0.25, 1.30])
        frames = [frame(0, elbowSolve=[0.18, 0.25, 1.30], **common), frame(1, elbowSolve=[0.43, 0.0, 1.30], **common)]
        write(self.path, frames)
        _, recs = rt.read_file(self.path)
        rows = rt.analyse(recs)
        self.assertAlmostEqual(rows[1]['eaz'], 90.0, places=3)
        self.assertAlmostEqual(rows[1]['gaz'], 0.0, places=3)
        self.assertIn('ELB', rows[1]['flags'])

    def test_push_number_gap_is_a_drop(self):
        # pushes 0..9, the ring dropped 4 and 5
        frames = [frame(k) for k in range(10) if k not in (4, 5)]
        write(self.path, frames)
        _, recs = rt.read_file(self.path)
        self.assertEqual([r['pushNumber'] for r in recs][:5], [0, 1, 2, 3, 6])
        rows = rt.analyse(rt.episodes(recs)[0])
        self.assertEqual([k for k, d in enumerate(rows) if 'DROP' in d['flags']], [4])
        self.assertIn('(2 dropped)', rt.episode_summary(1, recs, rows))
        # the 24-bit push number wraps without a false drop
        write(self.path, [frame(k, pushNumber=(rt.PUSH_NUMBER_WRAP - 2 + k) % rt.PUSH_NUMBER_WRAP) for k in range(4)])
        _, recs = rt.read_file(self.path)
        rows = rt.analyse(recs)
        self.assertFalse(any({'DROP', 'ORDER'} & set(d['flags']) for d in rows))
        self.assertIn('(0 dropped)', rt.episode_summary(1, recs, rows))
        # two pushes that land in the file in the other order: ORDER, not a drop of ~2^24 records
        write(self.path, [frame(0), frame(1, pushNumber=2), frame(2, pushNumber=1), frame(3)])
        _, recs = rt.read_file(self.path)
        rows = rt.analyse(recs)
        self.assertEqual([k for k, d in enumerate(rows) if 'ORDER' in d['flags']], [2])
        self.assertIn('(0 dropped)', rt.episode_summary(1, recs, rows))

    def test_weight_step_threshold_follows_the_frame_time(self):
        # a 0.3 step is a jump at 60 Hz (limit 0.208) but the plugin's 0.12 s ramp at 30 Hz (limit 0.417); 0.5 is a jump at both
        for hz, step, flagged in ((60, 0.3, True), (30, 0.3, False), (30, 0.5, True)):
            frames = [frame(k, ticks=Q0 + HZ + k * HZ // hz, corrFinal=1.0 - (step if k >= 2 else 0.0)) for k in range(4)]
            write(self.path, frames)
            _, recs = rt.read_file(self.path)
            rows = rt.analyse(recs)
            self.assertEqual('WSTEP' in rows[2]['flags'], flagged, (hz, step))

    def test_truncated_last_record_is_ignored(self):
        frames = self.episode()
        write(self.path, frames, tail=frames[0][:100])
        header, recs = rt.read_file(self.path)
        self.assertEqual(len(recs), 20)
        self.assertEqual(header['truncatedBytes'], 100)

    def test_other_record_size_is_refused(self):
        write(self.path, [], record_bytes=rt.RECORD_BYTES + 4)
        with self.assertRaises(ValueError):
            rt.read_file(self.path)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(rt.main([self.path]), 1)


if __name__ == '__main__':
    unittest.main()
