"""Tests of pose_capture_to_csv.py (compact pose capture -> text CSV of the continuous capture).

  python -I test_pose_capture_to_csv.py

The reference encoder below is written from the format description (src/mm/PoseCaptureFormat.hpp), independent of the C++ writer. It turns the
slice of a real recording (Build\\test-data\\posecapture\\utpp-bodytrace-slice.csv, cut with make_pose_capture_fixture.py) into compact files
(Windows XPRESS-Huffman blocks, stored blocks, several segments, several rigs, a post-helper stream, a file cut short); the converter must give
back the CSV in its exact shape: the same `#` lines, column line and row count, the same text in every non-value field, values within the
quantization error of the block range, exact timestamps. When UTPP_POSE_GOLDEN names a folder with *.utpppose files written by the C++ side
(ctest utpp_pose_capture_converter_test: `utpp_pose_capture_tool encode`), those files are converted and compared the same way: that is the
cross-check between the two implementations.
"""
import copy
import os
import pathlib
import struct
import sys
import tempfile
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pose_capture_to_csv as pc  # noqa: E402

ROOT = HERE.parents[2]
SLICE = ROOT / 'Build' / 'test-data' / 'posecapture' / 'utpp-bodytrace-slice.csv'
ROWS = 185


def scratch_root():
    """Own scratch folder (Build/pose-capture-converter-tests); every test removes its sub-folder again."""
    d = ROOT / 'Build' / 'pose-capture-converter-tests'
    d.mkdir(parents=True, exist_ok=True)
    return str(d)


# ---------------------------------------------------------------------------------------------------------------------- CSV slice
class Slice:
    def __init__(self, path):
        self.head, self.rows, self.trailer = [], [], None
        for line in pathlib.Path(path).read_text(encoding='utf-8').split('\n'):
            if not line:
                continue
            if line.startswith('# frames='):
                self.trailer = line
            elif line.startswith('#') or line.startswith('ticks,'):
                self.head.append(line)
            else:
                self.rows.append(line.split(','))
        first = self.head[0]
        self.qpc_hz = int(first.split('qpc_hz=')[1].split()[0])
        self.arm_ticks = int(first.split('arm_ticks=')[1].split()[0])
        utc = first.split('utc=')[1].split()[0]          # 2026-11-10T12:00:50.694Z
        d, t = utc.rstrip('Z').split('T')
        y, mo, da = (int(x) for x in d.split('-'))
        hh, mi, rest = t.split(':')
        ss, ms = rest.split('.')
        self.utc = (y, mo, da, int(hh), int(mi), int(ss), int(ms))
        self.max_frames = int(first.split('max_frames=')[1].split()[0])
        self.bones = [(int(l.split('index=')[1].split()[0]), int(l.split('hash=')[1].split()[0], 16), int(l.split('parent=')[1])) for l in self.head
                      if l.startswith('# rig_bone')]
        self.frames = len(self.rows) // ROWS
        self.names = [r[6] for r in self.rows[:ROWS]]
        self.game_index = [int(r[7]) for r in self.rows[:ROWS]]
        self.rig = int(self.rows[0][3])
        self.rig_count = int(self.rows[0][5])
        self.values = np.array([[float(x) for x in r[8:]] for r in self.rows], dtype=np.float32).reshape(self.frames, ROWS, 24)
        self.meta = np.array([[int(self.rows[f * ROWS][0]), int(self.rows[f * ROWS][1]), int(self.rows[f * ROWS][2]), int(self.rows[f * ROWS][4])]
                              for f in range(self.frames)], dtype=np.int64)


# ---------------------------------------------------------------------------------------------------------------------- reference encoder
def _varint(v):
    out = bytearray()
    while v >= 0x80:
        out.append((v & 0x7F) | 0x80)
        v >>= 7
    out.append(v)
    return bytes(out)


def _zz64(v):
    return ((v << 1) ^ (v >> 63)) & ((1 << 64) - 1)


def _string(s):
    b = s.encode('utf-8')
    return bytes([len(b)]) + b


def encode_header(sl, algorithm, segment_index, game_index=None, rig=None, hashes=None, streams=(0,)):
    game_index = sl.game_index if game_index is None else game_index
    hashes = [h for _, h, _ in sl.bones] if hashes is None else hashes
    parents = [sl.bones[i % len(sl.bones)][2] for i in range(len(hashes))]
    body = bytearray()
    body += pc.MAGIC
    body += struct.pack('<6I', pc.FORMAT_VERSION, 0, 0, algorithm, segment_index, sl.max_frames)
    body += struct.pack('<qq', sl.qpc_hz, sl.arm_ticks)
    body += struct.pack('<8H', *sl.utc, 0)
    body += struct.pack('<QI', sl.rig if rig is None else rig, sl.rig_count)
    body += struct.pack('<B', len(streams))
    for sid in streams:
        names = ['before_mm', 'after_mm_and_aim'] if sid == 0 else ['post_helper_apply_after_commit']
        body += struct.pack('<3B', sid, len(names), 4 if sid == 0 else 8)
        for n in names:
            body += _string(n)
    body += struct.pack('<I', len(sl.names))
    for n, g in zip(sl.names, game_index):
        body += _string(n) + struct.pack('<i', g)
    body += struct.pack('<I', len(hashes))
    for h, p in zip(hashes, parents):
        body += struct.pack('<Qh', h, p)
    struct.pack_into('<I', body, 12, len(body))
    return bytes(body)


def encode_block(sl, stream, f0, f1, algorithm, game_index=None, values=None, meta=None):
    """Payload built from the format description; returns the framed block."""
    game_index = sl.game_index if game_index is None else game_index
    values = sl.values if values is None else values
    meta = sl.meta if meta is None else meta
    mapped = [n for n, g in enumerate(game_index) if g >= 0]
    stages = 2 if stream == 0 else 1
    n = f1 - f0
    sel = values[f0:f1][:, mapped, :stages * 12].reshape(n, -1).astype(np.float64)      # [N, C]
    cols = sel.shape[1]
    metas = meta.shape[1]
    mb = bytearray()
    for m in range(metas):
        prev = 0
        for f in range(n):
            v = int(meta[f0 + f, m])
            mb += _varint(_zz64(v - prev))
            prev = v
    finite = np.isfinite(sel)
    mn = np.where(finite, sel, np.inf).min(axis=0)
    mx = np.where(finite, sel, -np.inf).max(axis=0)
    none = ~finite.any(axis=0)
    mn[none] = 0.0
    mx[none] = 0.0
    mn32, mx32 = mn.astype('<f4'), mx.astype('<f4')
    rng = mx32.astype(np.float64) - mn32.astype(np.float64)
    inv = np.where(rng > 0, pc.MAX_CODE / np.where(rng > 0, rng, 1.0), 0.0)
    q = np.floor((sel - mn32.astype(np.float64)) * inv + 0.5)
    q = np.clip(np.where(finite, q, pc.NAN_CODE), 0, 65535).astype(np.int64)      # [N, C]
    q[~finite] = pc.NAN_CODE
    q = q.T                                                                         # [C, N]
    diff = np.zeros_like(q)
    diff[:, 1:] = (q[:, 1:] - q[:, :-1]) & 0xFFFF
    signed = np.where(diff >= 32768, diff - 65536, diff)
    zz = ((signed << 1) ^ (signed >> 63)) & 0xFFFF
    stored = zz.copy()
    stored[:, 0] = q[:, 0]
    raw = struct.pack('<I', len(mb)) + bytes(mb) + mn32.tobytes() + mx32.tobytes() + (stored & 0xFF).astype(np.uint8).tobytes() + \
        (stored >> 8).astype(np.uint8).tobytes()
    body, compressed = raw, 0
    if algorithm != 0:
        c = pc.cabinet().compress(algorithm, raw)
        if len(c) < len(raw):
            body, compressed = c, 1
    return struct.pack('<BBIIIB', pc.KIND_DATA, stream, n, len(raw), len(body), compressed) + body


def encode_trailer(frames, dropped=(0, 0), failed=False):
    return struct.pack('<BIQQQQ', pc.KIND_TRAILER, 1 if failed else 0, frames[0], frames[1], dropped[0], dropped[1])


def write_file(path, data):
    pathlib.Path(path).write_bytes(data)
    return str(path)


# ---------------------------------------------------------------------------------------------------------------------- comparison
def compare_csv(test, sl, csv_path, block_frames, frames=None, trailer=None):
    """The converted CSV equals the slice: header text, column line, row text fields, values within the block-range bound."""
    frames = sl.frames if frames is None else frames
    lines = pathlib.Path(csv_path).read_text(encoding='utf-8').split('\n')
    test.assertEqual(lines[-1], '')
    lines = lines[:-1]
    heads = [l for l in lines if l.startswith('#') and not l.startswith('# frames=')]
    cols = [l for l in lines if l.startswith('ticks,')]
    rows = [l.split(',') for l in lines if not l.startswith('#') and not l.startswith('ticks,')]
    test.assertEqual(heads, [h for h in sl.head if h.startswith('#')], 'header lines')
    test.assertEqual(cols, [h for h in sl.head if h.startswith('ticks,')], 'column line')
    test.assertEqual(len(rows), frames * ROWS, 'row count')
    test.assertTrue(lines[-1].startswith('# frames='))
    test.assertEqual(lines[-1], trailer or sl.trailer)
    got = np.array([[float(x) for x in r[8:]] for r in rows], dtype=np.float32).astype(np.float64).reshape(frames, ROWS, 24)   # the text holds float32 values
    for f in range(frames):
        for n in (0, 7, 100, 184):
            ref = sl.rows[f * ROWS + n]
            test.assertEqual(rows[f * ROWS + n][:8], ref[:8], 'ticks,desc,thread,rig,generation,rigCount,name,gameIndex')
    nan_ref = ~np.isfinite(sl.values[:frames])
    test.assertTrue(np.array_equal(np.isnan(got), nan_ref), 'nan positions')
    worst = 0.0
    for b0 in range(0, frames, block_frames):
        b1 = min(frames, b0 + block_frames)
        ref = sl.values[b0:b1].astype(np.float64)
        fin = np.isfinite(ref)
        rng = np.where(fin, ref, -np.inf).max(axis=0) - np.where(fin, ref, np.inf).min(axis=0)
        rng = np.where(np.isfinite(rng), rng, 0.0)
        err = np.abs(got[b0:b1] - ref)
        err = np.where(fin, err, 0.0)
        ulp = np.abs(np.spacing(ref.astype(np.float32))).astype(np.float64)
        excess = np.where(fin, np.maximum(err - ulp, 0.0), 0.0)
        bound = rng / 65535.0
        test.assertTrue(np.all(excess <= bound + 1e-300), 'a value is outside the quantization bound (block %d)' % b0)
        const = rng == 0
        test.assertTrue(np.all(err[:, const] == 0.0), 'a constant column is not exact')
        with np.errstate(invalid='ignore', divide='ignore'):
            worst = max(worst, float(np.nanmax(np.where(bound > 0, excess / np.where(bound > 0, bound, 1.0), 0.0))))
    return worst


@unittest.skipUnless(SLICE.exists(), 'fixture slice missing (make_pose_capture_fixture.py)')
class ConverterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sl = Slice(SLICE)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=scratch_root())
        self.addCleanup(self.tmp.cleanup)
        self.dir = pathlib.Path(self.tmp.name)

    def blocks(self, sl, stream, algorithm, per):
        return b''.join(encode_block(sl, stream, f, min(sl.frames, f + per), algorithm) for f in range(0, sl.frames, per))

    def test_slice_xpress_roundtrip(self):
        sl = self.sl
        data = encode_header(sl, 4, 0) + self.blocks(sl, 0, 4, 50) + encode_trailer((sl.frames, 0))
        path = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-50.utpppose', data)
        out = pc.convert([path], str(self.dir))
        self.assertEqual([os.path.basename(p) for p in out], ['utpp-bodytrace-2026-11-10-12-00-50.csv'])
        worst = compare_csv(self, sl, out[0], 50)
        self.assertLessEqual(worst, 1.0)
        self.assertLess(len(data), os.path.getsize(path) + 1)

    def test_ring_segment_name(self):
        sl = self.sl
        data = encode_header(sl, 4, 0) + self.blocks(sl, 0, 4, 50) + encode_trailer((sl.frames, 0))
        path = write_file(self.dir / 'utpp-pose-ring-2026-11-10-12-00-50.utpppose', data)
        out = pc.convert([path], str(self.dir))
        self.assertEqual([os.path.basename(p) for p in out], ['utpp-bodytrace-ring-2026-11-10-12-00-50.csv'])
        compare_csv(self, sl, out[0], 50)

    def test_stored_blocks_and_lzms(self):
        sl = self.sl
        for alg in (0, 5):
            data = encode_header(sl, alg, 0) + self.blocks(sl, 0, alg, 30) + encode_trailer((sl.frames, 0))
            d = self.dir / ('alg%d' % alg)
            d.mkdir()
            path = write_file(d / 'utpp-pose-2026-11-10-12-00-50.utpppose', data)
            out = pc.convert([path], str(d))
            compare_csv(self, sl, out[0], 30)

    def test_segments_are_joined_in_time_order(self):
        sl = self.sl
        # three segments written out of name order; indices and start times order them
        cuts = [(0, 40), (40, 80), (80, 120)]
        names = ['utpp-pose-2026-11-10-12-00-50.utpppose', 'utpp-pose-2026-11-10-12-01-50.utpppose', 'utpp-pose-2026-11-10-12-02-50.utpppose']
        for i, ((a, b), name) in enumerate(zip(cuts, names)):
            seg = copy.copy(sl)
            seg.utc = (2026, 11, 10, 12, i, 50, 694)
            data = encode_header(seg, 4, i) + encode_block(seg, 0, a, b, 4) + encode_trailer((b - a, 0), dropped=(i + 1, 0))
            write_file(self.dir / name, data)
        out = pc.convert([str(self.dir)], str(self.dir / 'out'))
        self.assertEqual(len(out), 1)
        # block boundaries are at 40 frames: the per-block range bound is checked with that size
        compare_csv(self, sl, out[0], 40, trailer='# frames=120 dropped=6 failed=0')

    def test_topology_change_starts_a_new_csv(self):
        sl = self.sl
        a = encode_header(sl, 4, 0) + encode_block(sl, 0, 0, 60, 4) + encode_trailer((60, 0))
        b = encode_header(sl, 4, 1, rig=sl.rig + 1) + encode_block(sl, 0, 60, 120, 4) + encode_trailer((60, 0))
        pa = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-50.utpppose', a)
        pb = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-51.utpppose', b)
        out = pc.convert([pa, pb], str(self.dir / 'out'))
        self.assertEqual(len(out), 2)
        self.assertNotEqual(out[0], out[1])
        first = pathlib.Path(out[0]).read_text(encoding='utf-8').split('\n')
        second = pathlib.Path(out[1]).read_text(encoding='utf-8').split('\n')
        self.assertEqual(sum(1 for l in first if l and not l.startswith('#') and not l.startswith('ticks,')), 60 * ROWS)
        self.assertEqual(sum(1 for l in second if l and not l.startswith('#') and not l.startswith('ticks,')), 60 * ROWS)
        self.assertEqual(first[-2], '# frames=60 dropped=0 failed=0')
        self.assertEqual(second[-2], '# frames=60 dropped=0 failed=0')

    def test_file_cut_short_reads_the_complete_blocks(self):
        sl = self.sl
        blocks = [encode_block(sl, 0, f, f + 40, 4) for f in range(0, 120, 40)]
        data = encode_header(sl, 4, 0) + blocks[0] + blocks[1] + blocks[2][:len(blocks[2]) // 2]       # crash in the middle of the third block
        path = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-50.utpppose', data)
        logs = []
        out = pc.convert([path], str(self.dir), log=logs.append)
        self.assertTrue(logs and 'cut short' in logs[0])
        lines = pathlib.Path(out[0]).read_text(encoding='utf-8').split('\n')
        self.assertEqual(sum(1 for l in lines if l and not l.startswith('#') and not l.startswith('ticks,')), 80 * ROWS)
        self.assertEqual(lines[-2], '# frames=80 dropped=0 failed=0')

    def test_post_helper_stream_gets_its_own_csv_in_its_own_shape(self):
        sl = self.sl
        post_values = sl.values[:, :, :12].copy()
        post_meta = np.zeros((sl.frames, 8), dtype=np.int64)
        post_meta[:, :4] = sl.meta
        post_meta[:, 4] = 5
        post_meta[:, 5] = np.arange(sl.frames) + 2 ** 40
        post_meta[:, 6] = sl.meta[:, 0] - 100
        post_meta[:, 7] = sl.meta[:, 0] - 50
        data = encode_header(sl, 4, 0, streams=(0, 1)) + encode_block(sl, 0, 0, 40, 4) + \
            encode_block(sl, 1, 0, 40, 4, values=post_values, meta=post_meta) + encode_trailer((40, 40))
        path = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-50.utpppose', data)
        out = pc.convert([path], str(self.dir / 'out'))
        self.assertEqual([os.path.basename(p) for p in out], ['utpp-bodytrace-2026-11-10-12-00-50.csv', 'utpp-bodytrace-posthelper-2026-11-10-12-00-50.csv'])
        lines = pathlib.Path(out[1]).read_text(encoding='utf-8').split('\n')
        self.assertTrue(lines[0].startswith('# qpc_hz=') and lines[0].endswith(
            'continuous=1 stage=post_helper_apply_after_commit scope=main_rig_helpers; correlate by ticks,desc,rig,thread with legacy file'))
        cols = [l for l in lines if l.startswith('ticks,')][0]
        self.assertEqual(cols, 'ticks,desc,thread,rig,generation,rigCount,receiptGeneration,receiptCycle,receiptStartedTicks,receiptLastTicks,name,gameIndex,' +
                         ','.join('local_%d' % v for v in range(12)))
        rows = [l.split(',') for l in lines if l and not l.startswith('#') and not l.startswith('ticks,')]
        self.assertEqual(len(rows), 40 * ROWS)
        self.assertEqual(len(rows[0]), 12 + 12)
        self.assertEqual(int(rows[0][6]), 5)
        self.assertEqual(int(rows[0][7]), 2 ** 40)
        self.assertEqual(int(rows[0][8]), int(sl.meta[0, 0]) - 100)
        self.assertEqual(int(rows[0][9]), int(sl.meta[0, 0]) - 50)
        self.assertEqual(lines[-2], '# frames=40 dropped=0 failed=0')

    def test_header_of_any_size(self):
        sl = self.sl
        hashes = [0xABC000 + i for i in range(3000)]                       # a rig far bigger than the 645 bones of the recording
        data = encode_header(sl, 4, 0, hashes=hashes) + encode_block(sl, 0, 0, 20, 4) + encode_trailer((20, 0))
        self.assertGreater(len(encode_header(sl, 4, 0, hashes=hashes)), 20000)
        path = write_file(self.dir / 'utpp-pose-2026-11-10-12-00-50.utpppose', data)
        out = pc.convert([path], str(self.dir / 'out'))
        lines = pathlib.Path(out[0]).read_text(encoding='utf-8').splitlines()
        self.assertEqual(sum(1 for l in lines if l.startswith('# rig_bone')), 3000)
        self.assertEqual(sum(1 for l in lines if l and not l.startswith('#') and not l.startswith('ticks,')), 20 * ROWS)

    def test_rejects_other_input(self):
        bad = self.dir / 'bad.utpppose'
        bad.write_bytes(b'not a capture' * 20)
        with self.assertRaises(SystemExit):
            pc.convert([str(bad)], str(self.dir))
        data = bytearray(encode_header(self.sl, 4, 0))
        struct.pack_into('<I', data, 8, 2)
        bad.write_bytes(bytes(data))
        with self.assertRaises(SystemExit):
            pc.convert([str(bad)], str(self.dir))


@unittest.skipUnless(os.environ.get('UTPP_POSE_GOLDEN') and SLICE.exists(), 'UTPP_POSE_GOLDEN not set (the C++ writer output is checked by ctest utpp_pose_capture_converter_test)')
class GoldenTest(unittest.TestCase):
    def test_files_written_by_the_cpp_side(self):
        sl = Slice(SLICE)
        golden = pathlib.Path(os.environ['UTPP_POSE_GOLDEN'])
        files = sorted(golden.glob('*.utpppose'))
        self.assertGreaterEqual(len(files), 2, 'expected the XPRESS and the LZMS file from utpp_pose_capture_tool')
        with tempfile.TemporaryDirectory(dir=scratch_root()) as tmp:
            for f in files:
                d = pathlib.Path(tmp) / f.stem
                d.mkdir()
                out = pc.convert([str(f)], str(d))
                self.assertEqual(len(out), 1)
                compare_csv(self, sl, out[0], 50)


if __name__ == '__main__':
    unittest.main(verbosity=2)
