"""Checks of fist_base_fixture.py on synthetic data (no game files, no replay exe): the command line helpers, the clip input guard against the
header of src/mm/MeleeBaseData.cpp, the window cut (hard reset at the start of each window, picking a window by its mask-on time, melee=0 mask lines
ignored for the blend windows), the "MBO2" / "MBO3" writers read back with a parser that mirrors tests/MeleeBaseReplay.hpp (bone subset = needed
bones + all ancestors, renumbering, frame tail), and the per-clip table.   python -I test_fist_base_fixture.py"""
import hashlib
import math
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import fist_base_blend_fixture as fbf  # noqa: E402
import fist_base_fixture as fx  # noqa: E402
import fist_base_offline as fbo  # noqa: E402

SKELETON = [('Hips', -1), ('Spine', 0), ('Spine1', 1), ('Spine2', 2), ('Spine3', 3), ('Neck', 4), ('Neck1', 5), ('Head', 6),
            ('RightShoulder', 4), ('RightArm', 8), ('RightForeArm', 9), ('RightHand', 10),
            ('LeftShoulder', 4), ('LeftArm', 12), ('LeftForeArm', 13), ('LeftHand', 14),
            ('LeftUpLeg', 0), ('LeftLeg', 16), ('LeftFoot', 17), ('RightUpLeg', 0), ('RightLeg', 19), ('RightFoot', 20),
            ('Prop', 11), ('PropTip', 22), ('HairA', 7)]   # Prop / PropTip / HairA are not needed by the glue and must be dropped
FPS = 60.0


def fake_recording(seconds=12.0, t0=1000.0):
    """a Recording without a pose file: 60 Hz frames, arm / chest rotations vary with time so frames differ"""
    rec = fbo.Recording.__new__(fbo.Recording)
    rec.names = [n for n, _ in SKELETON]
    rec.cat = {n: k for k, n in enumerate(rec.names)}
    rec.parent = [p for _, p in SKELETON]
    nb = len(rec.names)
    n = int(seconds * FPS)
    rec.t = t0 + np.arange(n) / FPS
    rng = np.random.default_rng(7)
    V = np.zeros((n, nb, 2, 12), dtype=np.float32)
    for stage in range(2):
        V[:, :, stage, 0:3] = rng.normal(size=(n, nb, 3)) * 0.1
        q = rng.normal(size=(n, nb, 4))
        q /= np.linalg.norm(q, axis=-1, keepdims=True)
        V[:, :, stage, 4:8] = q
    rec.V = V
    rec.steps = {'settle': (t0 + 0.0, t0 + 2.0)}
    rec.order = list(range(nb))
    return rec


def write_log(path, lines):
    with open(path, 'w', encoding='utf-8') as fh:
        for sec, text in lines:
            fh.write('[2026-11-14 %s] [info    ] [ 1] [UTPP] UTPP %s\n' % (fx.fmt_clock(sec), text))


def read_fixture(path):
    """mirror of Replay::Load in tests/MeleeBaseReplay.hpp (without the renumbering): returns a dict"""
    with open(path, 'rb') as fh:
        d = fh.read()
    magic = d[:4]
    assert magic in (b'MBO2', b'MBO3')
    nb = struct.unpack_from('<i', d, 4)[0]
    off = 8
    par, nm = [], []
    for _ in range(nb):
        par.append(struct.unpack_from('<h', d, off)[0])
        nm.append(d[off + 2:off + 34].split(b'\0')[0].decode())
        off += 34
    body, n_low = struct.unpack_from('<ii', d, off)
    off += 8
    low = list(struct.unpack_from('<%di' % n_low, d, off))
    off += 4 * n_low
    up = []
    if magic == b'MBO3':
        n_up = struct.unpack_from('<i', d, off)[0]
        off += 4
        up = list(struct.unpack_from('<%di' % n_up, d, off))
        off += 4 * n_up
    nf = struct.unpack_from('<i', d, off)[0]
    off += 4
    frames = []
    for _ in range(nf):
        off += nb * 12 + nb * 16 + n_low * 12 + n_low * 16 + len(up) * 28
        mw, ls, us, base, dt, ref, side = struct.unpack_from('<fffIffi', d, off)
        off += 28
        frames.append(dict(meleeW=mw, lowerScale=ls, upperScale=us, base=base, dt=dt, ref=ref, side=side))
    assert off == len(d), 'file size does not match the layout (%d vs %d)' % (off, len(d))
    return dict(magic=magic, names=nm, parents=par, body=body, lower=low, upper=up, frames=frames)


class ClockAndWindows(unittest.TestCase):
    def test_parse_clock(self):
        self.assertAlmostEqual(fx.parse_clock('00:43:35.772'), 2615.772)
        self.assertAlmostEqual(fx.parse_clock('1:02:03'), 3723.0)
        with self.assertRaises(ValueError):
            fx.parse_clock('43:35')
        a, b = fx.parse_range('00:43:29.0-00:43:30.6')
        self.assertAlmostEqual(a, 2609.0)
        self.assertAlmostEqual(b, 2610.6)
        self.assertEqual(fx.fmt_clock(2615.772), '00:43:35.772')

    def test_find_window(self):
        ws = [(10.0, 11.0), (12.0, 15.0), (12.02, 13.0)]
        self.assertEqual(fx.find_window(ws, 10.01), 0)
        with self.assertRaises(ValueError):
            fx.find_window(ws, 12.01)   # two windows within the tolerance
        with self.assertRaises(ValueError):
            fx.find_window(ws, 20.0)

    def test_mask_windows_of_the_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, 'x.log')
            write_log(log, [
                (100.0, 'MMMASK on reasons=0x4 (was 0x0) melee=0 w=0.00'),            # another reason: not a melee window
                (100.5, 'MMMASK off reasons=0x0 (was 0x4) melee=0 w=1.00'),
                (110.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 w=1.00'),
                (110.0, 'MMMELEEFULL on why=on full=0.00'),
                (111.0, 'MMMASK on reasons=0x3 (was 0x1) melee=1 w=1.00'),           # second 'on' inside the window: ignored
                (112.5, 'MMMELEEFULL off why=state full=1.00'),
                (113.0, 'MMMASK off reasons=0x0 (was 0x1) melee=0 w=1.00'),
                (120.0, 'MMMASK on reasons=0x2 (was 0x0) melee=0 w=1.00'),            # upper body only
                (121.0, 'MMMASK on reasons=0x3 (was 0x2) melee=1 w=1.00'),            # melee joins an open window: counted as its start
                (123.0, 'MMMASK off reasons=0x0 (was 0x1) melee=0 w=1.00')])
            mask, full = fbf.melee_windows(log)
            self.assertEqual([(round(a, 3), round(b, 3)) for a, b in mask], [(110.0, 113.0), (121.0, 123.0)])
            self.assertEqual([(round(a, 3), round(b, 3)) for a, b in full], [(110.0, 112.5)])
            anyr = fbo.windows(log, 'MMMASK')
            self.assertEqual([(round(a, 3), round(b, 3)) for a, b in anyr], [(100.0, 100.5), (110.0, 113.0), (120.0, 123.0)])
            fb = fbo.windows(log, 'MMMELEEFULL')
            self.assertEqual(len(fb), 1)

    def test_steps_text(self):
        s = fbo.parse_steps('settle=00:43:29-00:43:30.6,punches=00:43:35.772-00:44:00.0')
        self.assertAlmostEqual(s['settle'][0], 2609.0)
        self.assertAlmostEqual(s['punches'][1], 2640.0)


class ClipInputGuard(unittest.TestCase):
    def make(self, tmp, contents):
        cpp = os.path.join(tmp, 'table.cpp')
        with open(cpp, 'w') as fh:
            fh.write('// GENERATED - inputs:\n')
            for name, data in contents.items():
                with open(os.path.join(tmp, name), 'wb') as cf:
                    cf.write(data)
                fh.write('//   %s  %s\n' % (hashlib.sha256(data).hexdigest()[:16], name))
            fh.write('#include "x.hpp"\n// not a clip line  deadbeefdeadbeef  nothing.npz\n')
        return cpp

    def test_matching_files_pass_and_changed_or_missing_files_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            cpp = self.make(tmp, {'a.npc1.npz': b'one', 'b.npc1.npz': b'two'})
            self.assertEqual(fx.check_clip_inputs(cpp, tmp), 2)
            with open(os.path.join(tmp, 'b.npc1.npz'), 'wb') as fh:
                fh.write(b'two, replaced')
            with self.assertRaises(RuntimeError):
                fx.check_clip_inputs(cpp, tmp)
            os.remove(os.path.join(tmp, 'a.npc1.npz'))
            with self.assertRaises(RuntimeError):
                fx.check_clip_inputs(cpp, tmp)

    def test_the_tree_table_lists_its_inputs(self):
        listed = fx.table_inputs(fx.TABLE_CPP)
        self.assertGreater(len(listed), 50)
        self.assertTrue(all(len(d) == 16 and n.endswith('.npz') for d, n in listed))
        self.assertTrue(any(n.startswith('pwa_fists__') for _, n in listed) and any(n.startswith('pma_fists__') for _, n in listed))


class WindowCut(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.rec = fake_recording()
        # settle 1000..1002; standing window mask on 1004.0 (full body from 1004.0, off 1006.0), walking window mask on 1008.0
        self.log = os.path.join(self.tmp.name, 'x.log')
        write_log(self.log, [
            (1003.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 w=1.00'),   # an earlier window that must not be picked
            (1003.4, 'MMMASK off reasons=0x0 (was 0x1) melee=0 w=1.00'),
            (1004.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 w=1.00'),
            (1004.0, 'MMMELEEFULL on why=on full=0.00'),
            (1006.0, 'MMMELEEFULL off why=state full=1.00'),
            (1006.5, 'MMMASK off reasons=0x0 (was 0x1) melee=0 w=1.00'),
            (1008.0, 'MMMASK on reasons=0x1 (was 0x0) melee=1 w=1.00'),
            (1010.0, 'MMMASK off reasons=0x0 (was 0x1) melee=0 w=1.00')])
        self.rec.steps['punches'] = (1004.0, 1130.0)
        self.rec.steps['punch_walk'] = (1008.0, 1130.0)

    def tearDown(self):
        self.tmp.cleanup()

    def test_picked_windows_start_with_a_hard_reset_and_have_the_asked_length(self):
        frames, lower = fbo.build_frames(self.rec, self.log, ('punches', 'punch_walk'), 40, 25, pick={'punches': 1004.0, 'punch_walk': 1008.0})
        self.assertEqual(len(frames), 65)
        resets = [i for i, f in enumerate(frames) if f['dt'] < 0]
        self.assertEqual(resets, [0, 40])
        self.assertAlmostEqual(frames[0]['t'] - frames[0]['since'], 1004.0, places=3)
        self.assertAlmostEqual(frames[40]['t'] - frames[40]['since'], 1008.0, places=3)
        self.assertAlmostEqual(frames[0]['since'], -0.1, delta=1 / FPS)       # starts 0.1 s before the mask
        self.assertTrue(all(f['dt'] > 0 for i, f in enumerate(frames) if i not in resets))
        self.assertEqual(frames[0]['meleeW'], 0.0)
        self.assertAlmostEqual(frames[20]['meleeW'], 1.0)                         # the 0.15 s mask blend is over
        self.assertEqual(frames[0]['lowerScale'], 1.0)
        self.assertAlmostEqual(frames[39]['lowerScale'], 0.0)                     # full body: the lower weight faded out
        self.assertEqual(frames[40]['lowerScale'], 1.0)                           # walking window: no melee full body
        self.assertEqual(set(lower), {n for n in self.rec.names if fbo.is_lower(n)})

    def test_without_pick_the_first_window_of_the_step_is_cut(self):
        frames, _ = fbo.build_frames(self.rec, self.log, ('punches',), 30, None)
        starts = {round(f['t'] - f['since'], 2) for f in frames}
        self.assertEqual(starts, {1004.0})        # the 1003.0 window is outside (step start 1004 - 1 s tolerance only reaches 1003.0 exclusive)

    def test_blend_frames_come_from_the_melee_windows_picked_by_time(self):
        mask, _ = fbf.melee_windows(self.log)
        self.assertEqual(len(mask), 3)
        idx = fx.find_window(mask, 1008.0)
        self.assertEqual(idx, 2)
        frames, lower = fbf.build_frames(self.rec, self.log, [fx.find_window(mask, 1004.0), idx], 20)
        self.assertEqual(len(frames), 40)
        self.assertEqual([i for i, f in enumerate(frames) if f['dt'] < 0], [0, 20])
        self.assertAlmostEqual(frames[0]['since'], 0.0, delta=1 / FPS)
        self.assertAlmostEqual(frames[0]['upperScale'] + frames[0]['meleeW'], 1.0)


class Writers(unittest.TestCase):
    def frames(self, rec, lower, n=6, blend=False):
        out = []
        nb = len(rec.names)
        for i in range(n):
            f = dict(i=i, t=1000.0 + i / FPS, since=i / FPS, w='punches', P=np.full((nb, 3), 0.01 * i), Q=np.tile([0, 0, 0, 1.0], (nb, 1)),
                     mmP=np.zeros((len(lower), 3)), mmQ=np.tile([0, 0, 0, 1.0], (len(lower), 1)), meleeW=min(1.0, i / 3), lowerScale=0.5,
                     upperScale=1.0 - min(1.0, i / 3), base=1, dt=-1.0 if i == 0 else 1 / FPS,
                     refElev=float('nan') if i % 2 == 0 else 12.5, side=i % 2)
            if blend:
                f.update(Peng=f['P'].copy(), Qeng=f['Q'].copy(), mmUP=np.zeros((len(fbf.UPPER_BLEND), 3)), mmUQ=np.tile([0, 0, 0, 1.0], (len(fbf.UPPER_BLEND), 1)))
            out.append(f)
        return out

    def check_subset(self, got, rec, lower, expect_upper=()):
        names = got['names']
        need = set(fbo.NEEDED) | set(lower) | set(expect_upper)
        closure = set()
        for n in need:
            b = rec.cat[n]
            while b >= 0:
                closure.add(rec.names[b])
                b = rec.parent[b]
        self.assertEqual(set(names), closure)
        self.assertNotIn('Prop', names)
        self.assertNotIn('PropTip', names)
        self.assertNotIn('HairA', names)
        for k, p in enumerate(got['parents']):
            self.assertTrue(p < k)                                            # parents come first (no renumbering needed by the reader)
            if p >= 0:
                self.assertEqual(rec.names[rec.parent[rec.cat[names[k]]]], names[p])
        self.assertEqual([names[i] for i in got['lower']], lower)

    def test_mbo2_subset_round_trip(self):
        rec = fake_recording(1.0)
        lower = [n for n in rec.names if fbo.is_lower(n)]
        frames = self.frames(rec, lower)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'f.bin')
            fbo.write_mbo2(p, rec, 'F', frames, lower, subset=True)
            got = read_fixture(p)
            full = os.path.join(tmp, 'full.bin')
            fbo.write_mbo2(full, rec, 'M', frames, lower, subset=False)
            got_full = read_fixture(full)
        self.assertEqual(got['magic'], b'MBO2')
        self.assertEqual(got['body'], 0)
        self.assertEqual(got_full['body'], 1)
        self.assertEqual(len(got_full['names']), len(rec.names))
        self.assertLess(len(got['names']), len(rec.names))
        self.check_subset(got, rec, lower)
        self.assertEqual(len(got['frames']), 6)
        self.assertEqual(got['frames'][0]['dt'], -1.0)
        self.assertTrue(math.isnan(got['frames'][0]['ref']))
        self.assertAlmostEqual(got['frames'][1]['ref'], 12.5)
        self.assertEqual([f['side'] for f in got['frames']], [0, 1, 0, 1, 0, 1])
        self.assertEqual(got['frames'][5]['base'], 1)
        self.assertAlmostEqual(got['frames'][3]['meleeW'], 1.0)

    def test_mbo3_subset_round_trip_with_the_blended_upper_bones(self):
        rec = fake_recording(1.0)
        lower = [n for n in rec.names if fbo.is_lower(n)]
        frames = self.frames(rec, lower, 5, blend=True)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'b.bin')
            fbf.write_mbo3(p, rec, 'M', frames, lower, fbf.UPPER_BLEND, subset=True)
            got = read_fixture(p)
        self.assertEqual(got['magic'], b'MBO3')
        self.assertEqual(got['body'], 1)
        self.check_subset(got, rec, lower, fbf.UPPER_BLEND)
        self.assertEqual([got['names'][i] for i in got['upper']], fbf.UPPER_BLEND)
        self.assertEqual(len(got['frames']), 5)
        self.assertEqual(got['frames'][0]['dt'], -1.0)


class Report(unittest.TestCase):
    def test_references_and_clip_table(self):
        frames = [dict(since=0.05, refElev=0.0), dict(since=0.2, refElev=0.0), dict(since=0.3, refElev=0.0), dict(since=0.3, refElev=0.0)]
        rows = [dict(side='Right', ec=3.0, ext=50.0, e0=20.0, e1=4.0, cname='a', so=[0] * 18 + [1]), dict(side='Left', ec=5.0, ext=50.0, e0=25.0, e1=7.0, cname='a', so=[0] * 19),
                dict(side='Right', ec=None, ext=60.0, e0=1.0, e1=1.0, cname='-', so=[0] * 19), dict(side='Right', ec=2.0, ext=30.0, e0=1.0, e1=1.0, cname='b', so=[0] * 19)]
        fx.set_references(frames, rows)
        self.assertEqual([f['side'] for f in frames], [0, 1, 0, 0])
        self.assertEqual(frames[0]['refElev'], 3.0)
        self.assertEqual(frames[1]['refElev'], 5.0)
        self.assertTrue(math.isnan(frames[2]['refElev']))        # no clip match
        self.assertTrue(math.isnan(frames[3]['refElev']))        # the arm is not stretched
        fx.set_references(frames, rows, since_min=0.12)
        self.assertTrue(math.isnan(frames[0]['refElev']))        # the blend is not over yet
        self.assertEqual(frames[1]['refElev'], 5.0)
        fx.set_references(frames, rows)
        per = fx.clip_table(frames, rows)
        self.assertEqual(per['a']['n'], 2)
        self.assertEqual(per['a']['after'], [1.0, 2.0])
        self.assertEqual(per['b']['ref'], [])
        text = fx.format_clip_table(frames, rows, 'x')
        self.assertIn('(all)', text)
        self.assertIn('flags=1', text)             # the largest error (frame 1, 2 deg) is listed with its limit flags
        self.assertEqual(fx.pct([1.0, 5.0, 2.0, 4.0, 3.0], 0.95), 4.0)   # sorted[floor(0.95 * 4)], as the glue test
        self.assertIn('a ', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
