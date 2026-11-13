"""Converts compact pose captures (utpp-pose-*.utpppose, written by the mod's UTPPPoseCaptureStart) into the text CSV of the continuous
helper-bone capture (utpp-bodytrace-*.csv), in exactly that shape: the `#` header lines, the `# rig_bone` topology lines, the column names, one
row per bone per frame (185 rows per frame, %.9g values) and the `# frames= dropped= failed=` trailer. The existing analysis tools
(replay_body_skin.py, vaulttorso_fixture.py, the films) read the result unchanged.

  python -I pose_capture_to_csv.py INPUT [INPUT ...] [--out-dir DIR]

INPUT is a file, a folder (every *.utpppose in it) or a wildcard. Segments of one capture (keepMinutes > 0 writes ~1-minute files) are joined
in time order into one CSV as long as the rig and its topology stay the same; a different rig starts a new CSV. The output is named after the
first file: utpp-pose-<stamp>.utpppose -> utpp-bodytrace-<stamp>.csv (+ utpp-bodytrace-posthelper-<stamp>.csv when the capture has the
post-helper stream). The ring segments of the permanent mode are named utpp-pose-ring-<stamp>.utpppose -> utpp-bodytrace-ring-<stamp>.csv. Default folder = the folder of the first input. A file cut short by a crash is read up to its last complete block.

Requires numpy and Windows (the blocks are compressed with the Compression API built into Windows; it is called through ctypes on
cabinet.dll). Differences to the original text capture: values carry the 16-bit quantization error of the format (at most 1/65535 of a
column's range inside a 5-second block; constant columns are exact), and a value that was inf is written as nan.

The file format is described at the top of src/mm/PoseCaptureFormat.hpp.
"""
import argparse
import ctypes
import glob
import os
import struct
import sys

import numpy as np

MAGIC = b'UTPPPOSE'
FORMAT_VERSION = 1
KIND_DATA, KIND_TRAILER = 1, 2
ALGORITHM_STORED = 0
STREAM_LEGACY, STREAM_POST = 0, 1
STAGES = {STREAM_LEGACY: 2, STREAM_POST: 1}
META = {STREAM_LEGACY: 4, STREAM_POST: 8}
NAN_CODE = 65535
MAX_CODE = 65534


# --------------------------------------------------------------------------------------------------------------------- Windows compression
class _Cabinet:
    def __init__(self):
        self.dll = ctypes.WinDLL('cabinet')
        v = ctypes.c_void_p
        sz = ctypes.c_size_t
        self.dll.CreateDecompressor.argtypes = [ctypes.c_uint32, v, ctypes.POINTER(v)]
        self.dll.Decompress.argtypes = [v, v, sz, v, sz, ctypes.POINTER(sz)]
        self.dll.CloseDecompressor.argtypes = [v]
        self.dll.CreateCompressor.argtypes = [ctypes.c_uint32, v, ctypes.POINTER(v)]
        self.dll.Compress.argtypes = [v, v, sz, v, sz, ctypes.POINTER(sz)]
        self.dll.CloseCompressor.argtypes = [v]
        for name in ('CreateDecompressor', 'CreateCompressor', 'Decompress', 'Compress'):
            getattr(self.dll, name).restype = ctypes.c_int
        self.dll.CloseDecompressor.restype = ctypes.c_int
        self.dll.CloseCompressor.restype = ctypes.c_int
        self._d = {}
        self._c = {}

    def decompress(self, algorithm, data, raw_size):
        h = self._d.get(algorithm)
        if h is None:
            h = ctypes.c_void_p()
            if not self.dll.CreateDecompressor(algorithm, None, ctypes.byref(h)):
                raise OSError('CreateDecompressor failed for algorithm %d' % algorithm)
            self._d[algorithm] = h
        out = ctypes.create_string_buffer(raw_size)
        got = ctypes.c_size_t(0)
        src = ctypes.create_string_buffer(bytes(data), len(data))
        if not self.dll.Decompress(h, src, len(data), out, raw_size, ctypes.byref(got)) or got.value != raw_size:
            raise ValueError('block does not decompress (algorithm %d)' % algorithm)
        return out.raw[:raw_size]

    def compress(self, algorithm, data):
        h = self._c.get(algorithm)
        if h is None:
            h = ctypes.c_void_p()
            if not self.dll.CreateCompressor(algorithm, None, ctypes.byref(h)):
                raise OSError('CreateCompressor failed for algorithm %d' % algorithm)
            self._c[algorithm] = h
        src = ctypes.create_string_buffer(bytes(data), len(data))
        need = ctypes.c_size_t(0)
        self.dll.Compress(h, src, len(data), None, 0, ctypes.byref(need))
        out = ctypes.create_string_buffer(need.value)
        got = ctypes.c_size_t(0)
        if not self.dll.Compress(h, src, len(data), out, need.value, ctypes.byref(got)):
            raise ValueError('Compress failed (algorithm %d)' % algorithm)
        return out.raw[:got.value]


_cabinet = None


def cabinet():
    global _cabinet
    if _cabinet is None:
        _cabinet = _Cabinet()
    return _cabinet


# --------------------------------------------------------------------------------------------------------------------- reading
class Header:
    """Parsed file header (see the format description in src/mm/PoseCaptureFormat.hpp)."""

    def __init__(self):
        self.version = self.header_bytes = self.flags = self.algorithm = self.segment_index = self.max_frames = 0
        self.qpc_hz = self.arm_ticks = 0
        self.utc = (0,) * 7
        self.rig = self.rig_count = 0
        self.streams = {}          # id -> list of stage names
        self.names = []
        self.game_index = []
        self.hashes = []
        self.parents = []

    def mapped(self):
        return [n for n, i in enumerate(self.game_index) if i >= 0]

    def columns(self, stream):
        return len(self.mapped()) * STAGES[stream] * 12

    def topology_key(self):
        return (self.rig, self.rig_count, tuple(self.hashes), tuple(self.parents), tuple(self.game_index), tuple(sorted(self.streams)))

    def start_key(self):
        return tuple(self.utc) + (self.segment_index,)


class _Cursor:
    def __init__(self, buf, pos=0):
        self.buf, self.pos = buf, pos

    def take(self, fmt):
        size = struct.calcsize(fmt)
        if self.pos + size > len(self.buf):
            raise ValueError('truncated header')
        vals = struct.unpack_from(fmt, self.buf, self.pos)
        self.pos += size
        return vals

    def string(self):
        n = self.take('<B')[0]
        if self.pos + n > len(self.buf):
            raise ValueError('truncated header')
        s = bytes(self.buf[self.pos:self.pos + n]).decode('utf-8')
        self.pos += n
        return s


def parse_header(buf):
    if bytes(buf[:8]) != MAGIC:
        raise ValueError('not a compact pose capture (bad magic)')
    c = _Cursor(buf, 8)
    h = Header()
    h.version, h.header_bytes, h.flags, h.algorithm, h.segment_index, h.max_frames = c.take('<6I')
    if h.version != FORMAT_VERSION:
        raise ValueError('unsupported format version %d' % h.version)
    h.qpc_hz, h.arm_ticks = c.take('<qq')
    utc = c.take('<8H')
    h.utc = tuple(utc[:7])
    h.rig, h.rig_count = c.take('<QI')
    for _ in range(c.take('<B')[0]):
        sid, stages, metas = c.take('<3B')
        if sid not in STAGES or stages != STAGES[sid] or metas != META[sid]:
            raise ValueError('unexpected stream layout %d/%d/%d' % (sid, stages, metas))
        h.streams[sid] = [c.string() for _ in range(stages)]
    bones = c.take('<I')[0]
    for _ in range(bones):
        h.names.append(c.string())
        h.game_index.append(c.take('<i')[0])
    for _ in range(c.take('<I')[0]):
        hv, pv = c.take('<Qh')
        h.hashes.append(hv)
        h.parents.append(pv)
    if c.pos != h.header_bytes:
        raise ValueError('header size mismatch')
    return h, c.pos


def _varints(buf, pos, count):
    out = []
    for _ in range(count):
        shift = 0
        v = 0
        while True:
            b = buf[pos]
            pos += 1
            v |= (b & 0x7F) << shift
            if not b & 0x80:
                break
            shift += 7
        out.append(v)
    return out, pos


def decode_block(header, stream, frames, raw):
    """-> (meta int64 [frames, metaCount], values float32 [frames, columns]) of one block payload."""
    n, cols, metas = frames, header.columns(stream), META[stream]
    (meta_bytes,) = struct.unpack_from('<I', raw, 0)
    pos = 4
    meta_end = pos + meta_bytes
    meta = np.zeros((n, metas), dtype=np.int64)
    for m in range(metas):
        vals, pos = _varints(raw, pos, n)
        prev = 0
        for f in range(n):
            z = vals[f]
            d = (z >> 1) ^ -(z & 1)
            prev = (prev + d + (1 << 63)) % (1 << 64) - (1 << 63)
            meta[f, m] = prev
    if pos != meta_end:
        raise ValueError('meta section size mismatch')
    mn = np.frombuffer(raw, dtype='<f4', count=cols, offset=pos).astype(np.float64)
    pos += 4 * cols
    mx = np.frombuffer(raw, dtype='<f4', count=cols, offset=pos).astype(np.float64)
    pos += 4 * cols
    if len(raw) - pos != 2 * cols * n:
        raise ValueError('code planes size mismatch')
    low = np.frombuffer(raw, dtype=np.uint8, count=cols * n, offset=pos).reshape(cols, n).astype(np.int64)
    high = np.frombuffer(raw, dtype=np.uint8, count=cols * n, offset=pos + cols * n).reshape(cols, n).astype(np.int64)
    stored = low | (high << 8)
    delta = (stored >> 1) ^ -(stored & 1)
    delta[:, 0] = stored[:, 0]
    codes = np.cumsum(delta, axis=1) & 0xFFFF
    step = (mx - mn) / float(MAX_CODE)
    values = mn[:, None] + codes.astype(np.float64) * step[:, None]
    values[codes == NAN_CODE] = np.nan
    return meta, np.ascontiguousarray(values.T.astype(np.float32))


class Trailer:
    def __init__(self, failed, frames, dropped):
        self.failed, self.frames, self.dropped = failed, frames, dropped


def read_capture(path):
    """Yields ('header', Header), then ('block', stream, meta, values)* and finally ('trailer', Trailer or None, truncated: bool)."""
    with open(path, 'rb') as f:
        buf = f.read()
    header, pos = parse_header(buf)
    yield 'header', header
    trailer = None
    truncated = False
    cab = None
    while pos < len(buf):
        kind = buf[pos]
        if kind == KIND_DATA:
            if pos + 15 > len(buf):
                truncated = True
                break
            stream, frames, raw_size, stored_size, compressed = struct.unpack_from('<BIIIB', buf, pos + 1)
            body_at = pos + 15
            if stream not in STAGES:
                raise ValueError('unknown stream %d' % stream)
            if body_at + stored_size > len(buf):
                truncated = True
                break
            body = bytes(buf[body_at:body_at + stored_size])
            if compressed:
                cab = cab or cabinet()
                body = cab.decompress(header.algorithm, body, raw_size)
            elif len(body) != raw_size:
                raise ValueError('stored block size mismatch')
            meta, values = decode_block(header, stream, frames, body)
            yield 'block', stream, meta, values
            pos = body_at + stored_size
        elif kind == KIND_TRAILER:
            if pos + 37 > len(buf):
                truncated = True
                break
            flags, f0, f1, d0, d1 = struct.unpack_from('<IQQQQ', buf, pos + 1)
            trailer = Trailer(bool(flags & 1), (f0, f1), (d0, d1))
            pos += 37
        else:
            raise ValueError('unknown block kind %d at offset %d' % (kind, pos))
    yield 'trailer', trailer, truncated


# --------------------------------------------------------------------------------------------------------------------- CSV writing
def _fmt9(v, _f='%.9g'.__mod__):
    return _f(v)


class CsvWriter:
    """Writes one stream of one rig topology in the text shape of the continuous capture."""

    def __init__(self, path, header, stream):
        self.path, self.header, self.stream = path, header, stream
        self.f = open(path, 'w', encoding='utf-8', newline='')
        self.frames = 0
        self.dropped = 0
        self.failed = False
        self.mapped = header.mapped()
        stages = STAGES[stream]
        self.values_per_row = stages * 12
        self.prefix_rig = '%d,%d' % (header.rig, header.rig_count)
        u = header.utc
        utc = '%04u-%02u-%02uT%02u:%02u:%02u.%03uZ' % tuple(u)
        if stream == STREAM_LEGACY:
            self.f.write('# qpc_hz=%d arm_ticks=%d utc=%s max_frames=%d continuous=1 stages=%s scope=main_rig_helpers\n' %
                         (header.qpc_hz, header.arm_ticks, utc, header.max_frames, ','.join(header.streams[STREAM_LEGACY])))
        else:
            self.f.write('# qpc_hz=%d arm_ticks=%d utc=%s max_frames=%d continuous=1 stage=%s scope=main_rig_helpers; '
                         'correlate by ticks,desc,rig,thread with legacy file\n' %
                         (header.qpc_hz, header.arm_ticks, utc, header.max_frames, header.streams[STREAM_POST][0]))
        for i, (hv, pv) in enumerate(zip(header.hashes, header.parents)):
            self.f.write('# rig_bone index=%d hash=%016X parent=%d\n' % (i, hv, pv))
        if stream == STREAM_LEGACY:
            self.f.write('ticks,desc,thread,rig,generation,rigCount,name,gameIndex' +
                         ''.join(',s%d_%d' % (s, v) for s in range(2) for v in range(12)) + '\n')
        else:
            self.f.write('ticks,desc,thread,rig,generation,rigCount,receiptGeneration,receiptCycle,receiptStartedTicks,receiptLastTicks,name,gameIndex' +
                         ''.join(',local_%d' % v for v in range(12)) + '\n')
        self.row_tail = ['%s,%d' % (name, header.game_index[n]) for n, name in enumerate(header.names)]
        self.nan_row = ','.join(['nan'] * self.values_per_row)

    def add_block(self, meta, values):
        h = self.header
        vpr = self.values_per_row
        slot_of = {n: k for k, n in enumerate(self.mapped)}
        mask64 = (1 << 64) - 1
        out = []
        for f in range(meta.shape[0]):
            m = [int(x) for x in meta[f]]
            ticks, desc, thread, gen = m[0], m[1] & mask64, m[2] & 0xFFFFFFFF, m[3] & mask64
            if self.stream == STREAM_LEGACY:
                lead = '%d,%d,%d,%d,%d,%d' % (ticks, desc, thread, h.rig, gen, h.rig_count)
            else:
                lead = '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d' % (ticks, desc, thread, h.rig, gen, h.rig_count, m[4] & mask64, m[5] & mask64, m[6], m[7])
            row = values[f].tolist()
            for n in range(len(h.names)):
                k = slot_of.get(n)
                body = self.nan_row if k is None else ','.join(map(_fmt9, row[k * vpr:(k + 1) * vpr]))
                out.append('%s,%s,%s\n' % (lead, self.row_tail[n], body))
            self.frames += 1
        self.f.write(''.join(out))

    def close(self, frames=None, dropped=None, failed=None):
        self.f.write('# frames=%d dropped=%d failed=%d\n' % (self.frames if frames is None else frames, self.dropped if dropped is None else dropped,
                                                             1 if (self.failed if failed is None else failed) else 0))
        self.f.close()


def _expand(inputs):
    files = []
    for item in inputs:
        if os.path.isdir(item):
            files += sorted(glob.glob(os.path.join(item, '*.utpppose')))
        elif any(c in item for c in '*?['):
            files += sorted(glob.glob(item))
        else:
            files.append(item)
    if not files:
        raise SystemExit('no .utpppose input found')
    return files


def _csv_names(first_path, group_no, out_dir):
    base = os.path.basename(first_path)
    stem = base[:-len('.utpppose')] if base.endswith('.utpppose') else os.path.splitext(base)[0]
    if stem.startswith('utpp-pose-'):
        stem = stem[len('utpp-pose-'):]
    suffix = '' if group_no == 0 else '-g%d' % (group_no + 1)
    return (os.path.join(out_dir, 'utpp-bodytrace-%s%s.csv' % (stem, suffix)),
            os.path.join(out_dir, 'utpp-bodytrace-posthelper-%s%s.csv' % (stem, suffix)))


def convert(inputs, out_dir=None, log=print):
    """Converts the given files / folders / wildcards; returns the CSV paths written (legacy first, post-helper after it per group)."""
    files = _expand(inputs)
    loaded = []
    for p in files:
        with open(p, 'rb') as f:
            head = f.read(16)
            if len(head) == 16 and head[:8] == MAGIC:
                head += f.read(struct.unpack_from('<I', head, 12)[0] - 16)    # headerBytes: the header may be any size (large rigs)
        try:
            h, _ = parse_header(head)
        except ValueError as e:
            raise SystemExit('%s: %s' % (p, e))
        loaded.append((h.start_key(), p, h))
    loaded.sort(key=lambda t: (t[0], t[1]))
    out_dir = out_dir or os.path.dirname(os.path.abspath(loaded[0][1]))
    os.makedirs(out_dir, exist_ok=True)
    written = []
    writers = {}
    group_key = None
    group_no = -1

    def close_group():
        for w in writers.values():
            w.close()
        writers.clear()

    for _, path, header in loaded:
        key = header.topology_key()
        if key != group_key:
            close_group()
            group_key = key
            group_no += 1
            legacy_path, post_path = _csv_names(path, group_no, out_dir)
            writers[STREAM_LEGACY] = CsvWriter(legacy_path, header, STREAM_LEGACY)
            written.append(legacy_path)
            if STREAM_POST in header.streams:
                writers[STREAM_POST] = CsvWriter(post_path, header, STREAM_POST)
                written.append(post_path)
        for item in read_capture(path):
            if item[0] == 'block':
                _, stream, meta, values = item
                writers[stream].add_block(meta, values)
            elif item[0] == 'trailer':
                _, trailer, truncated = item
                if trailer is None or truncated:
                    log('%s: no trailer / truncated block (the capture was cut short); read up to the last complete block' % os.path.basename(path))
                if trailer is not None:
                    for s, w in writers.items():
                        w.dropped += trailer.dropped[s]
                        w.failed = w.failed or trailer.failed
    close_group()
    return written


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('inputs', nargs='+', help='.utpppose file(s), folder(s) or wildcard(s)')
    ap.add_argument('--out-dir', help='output folder (default: the folder of the first input)')
    a = ap.parse_args(argv)
    for p in convert(a.inputs, a.out_dir):
        print('wrote %s (%.1f MB)' % (p, os.path.getsize(p) / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
