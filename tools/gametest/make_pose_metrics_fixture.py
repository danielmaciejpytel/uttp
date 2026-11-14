"""Cuts the small fixtures of test_pose_metrics.py out of a real session.

  python -I make_pose_metrics_fixture.py POSE_FILE LOG_FILE FEED_FILE --from HH:MM:SS.fff --seconds 5 [--full-frames 12] [--out-dir DIR]

Writes into --out-dir (default: testdata next to this file)
  pose_metrics_core.utpppose   the frames of the window with only the bones the metrics need mapped (the other catalogue entries get game index -1)
  pose_metrics_full.utpppose   a few frames spread over the window with every catalogue bone (for the helper bone comparison)
  pose_metrics_core.session.txt        the log lines of the window that the meter reads
  pose_metrics_core.feed.txt   the feed dump rows of the window
The block format is the one of src/mm/PoseCaptureFormat.hpp (blocks compressed with the Windows XPRESS algorithm).
"""
import argparse
import datetime
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pose_metrics as pm  # noqa: E402  (also puts tools/analysis on the import path)

pc = pm.pc


def varint(v):
    out = bytearray()
    while v >= 0x80:
        out.append((v & 0x7F) | 0x80)
        v >>= 7
    out.append(v)
    return bytes(out)


def zigzag64(v):
    return ((v << 1) ^ (v >> 63)) & 0xFFFFFFFFFFFFFFFF


def encode_payload(stream, meta, values):
    """meta [N, metaCount] int64, values [N, C] float32 -> block payload bytes (format of PoseCaptureFormat.hpp)."""
    n, c = values.shape
    mc = pc.META[stream]
    mb = bytearray()
    for m in range(mc):
        prev = 0
        for f in range(n):
            v = int(meta[f, m])
            mb += varint(zigzag64(v - prev))
            prev = v
    out = bytearray(struct.pack('<I', len(mb)) + bytes(mb))
    mn = np.zeros(c, dtype=np.float32)
    mx = np.zeros(c, dtype=np.float32)
    fin = np.isfinite(values)
    for j in range(c):
        col = values[fin[:, j], j]
        if len(col):
            mn[j], mx[j] = col.min(), col.max()
    out += mn.tobytes() + mx.tobytes()
    rng = mx.astype(np.float64) - mn.astype(np.float64)
    inv = np.where(rng > 0, pc.MAX_CODE / np.where(rng > 0, rng, 1), 0.0)
    codes = np.floor((values.astype(np.float64) - mn) * inv + 0.5)
    codes = np.clip(np.where(fin, codes, pc.NAN_CODE), 0, 65535).astype(np.int64)
    prev = np.vstack([np.zeros((1, c), dtype=np.int64), codes[:-1]])
    diff = ((codes - prev + 32768) & 0xFFFF) - 32768
    stored = np.where(np.arange(n)[:, None] == 0, codes, ((diff << 1) ^ (diff >> 15)) & 0xFFFF).astype(np.int64)
    out += (stored & 0xFF).astype(np.uint8).T.tobytes() + (stored >> 8).astype(np.uint8).T.tobytes()
    return bytes(out)


def header_bytes(h, keep_game_index, algorithm):
    """Rebuilds a file header from a parsed one, with the given gameIndex list (length = catalogue size)."""
    b = bytearray(b'UTPPPOSE' + struct.pack('<II', pc.FORMAT_VERSION, 0))
    b += struct.pack('<IIII', h.flags, algorithm, h.segment_index, h.max_frames)
    b += struct.pack('<qq', h.qpc_hz, h.arm_ticks)
    b += struct.pack('<8H', *h.utc, 0)
    b += struct.pack('<QIB', h.rig, h.rig_count, len(h.streams))
    for sid in sorted(h.streams):
        b += struct.pack('<3B', sid, pc.STAGES[sid], pc.META[sid])
        for name in h.streams[sid]:
            e = name.encode()
            b += struct.pack('<B', len(e)) + e
    b += struct.pack('<I', len(h.names))
    for name, gi in zip(h.names, keep_game_index):
        e = name.encode()
        b += struct.pack('<B', len(e)) + e + struct.pack('<i', gi)
    b += struct.pack('<I', len(h.hashes))
    for hv, pv in zip(h.hashes, h.parents):
        b += struct.pack('<Qh', hv, pv)
    struct.pack_into('<I', b, 12, len(b))
    return bytes(b)


def write_capture(path, h, game_index, meta, values, algorithm=4, block=300):
    """h: parsed header (names, hashes, parents, utc, qpc, ...); game_index: per catalogue entry; meta [N, 4]; values [N, C] (C = mapped * 2 * 12)."""
    data = bytearray(header_bytes(h, game_index, algorithm))
    cab = pc.cabinet()
    for a in range(0, len(values), block):
        payload = encode_payload(0, meta[a:a + block], values[a:a + block])
        body = cab.compress(algorithm, payload) if algorithm else payload
        compressed = 1 if algorithm and len(body) < len(payload) else 0
        body = body if compressed else payload
        data += struct.pack('<BBIIIB', 1, 0, len(values[a:a + block]), len(payload), len(body), compressed) + body
    data += struct.pack('<BIQQQQ', 2, 0, len(values), 0, 0, 0)
    with open(path, 'wb') as f:
        f.write(bytes(data))
    return len(data)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('pose')
    ap.add_argument('log')
    ap.add_argument('feed')
    ap.add_argument('--from', dest='t_from', required=True)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--full-frames', type=int, default=12)
    ap.add_argument('--out-dir', default=os.path.join(HERE, 'testdata'))
    a = ap.parse_args(argv)
    group = pm.phr.load_groups([a.pose], pm.phr.Config())[0]
    h = group['header']
    lo = pm.phr.parse_clock(a.t_from)
    hi = lo + a.seconds
    keep_names = set(pm.TRACK) | set(pm.GUARD_ORDER)
    mapped = h.mapped()
    names = [h.names[i] for i in mapped]
    rigidx = [h.game_index[i] for i in mapped]
    r2c = {g: k for k, g in enumerate(rigidx)}
    parent = [r2c.get(h.parents[g], -1) if h.parents[g] >= 0 else -1 for g in rigidx]
    need = set()
    for n in keep_names:
        if n in names:
            k = names.index(n)
            while k >= 0:
                need.add(k)
                k = parent[k]
    need = sorted(need)
    frames, metas, gids, allsec = [], [], [], []
    for stream, ticks, sec, gidx, v in pm.phr.iter_blocks(group):
        if stream != 0:
            continue
        allsec.append(sec)
        ix = np.nonzero((sec >= lo) & (sec < hi))[0]
        if len(ix):
            frames.append(v[ix])
            metas.append(ticks[ix])
            gids.append(gidx[ix])
    full = np.concatenate(frames)
    ticks = np.concatenate(metas)
    meta = np.zeros((len(ticks), 4), dtype=np.int64)
    meta[:, 0] = ticks
    os.makedirs(a.out_dir, exist_ok=True)
    gi_core = [h.game_index[i] if (i in mapped and mapped.index(i) in need) else -1 for i in range(len(h.names))]
    core = full[:, need].reshape(len(full), -1)
    n1 = write_capture(os.path.join(a.out_dir, 'pose_metrics_core.utpppose'), h, gi_core, meta, core)
    pick = np.linspace(0, len(full) - 1, a.full_frames).astype(int)
    gi_full = list(h.game_index)
    n2 = write_capture(os.path.join(a.out_dir, 'pose_metrics_full.utpppose'), h, gi_full, meta[pick], full.reshape(len(full), -1)[pick])
    print('core %d frames %d bytes, full %d frames %d bytes' % (len(full), n1, len(pick), n2))
    # log and feed excerpts
    day = datetime.datetime.strptime(group['day'], '%Y-%m-%d')
    w0, w1 = day + datetime.timedelta(seconds=lo - 2.0), day + datetime.timedelta(seconds=hi + 1.0)
    keep_re = re.compile(r'UTPP MOTION t=|UTPPSetHeadLook\(|UTPP MMGATE ')
    with open(a.log, 'r', encoding='utf-8', errors='replace') as f, open(os.path.join(a.out_dir, 'pose_metrics_core.session.txt'), 'w', encoding='utf-8', newline='\n') as o:
        for ln in f:
            m = pm.TS_RE.match(ln)
            if m and w0 <= pm._ts(m) <= w1 and keep_re.search(ln):
                o.write(re.sub(r'(UTPP MOTION .*?clip=\S+ ct=\S+ ).*? (body=\w+).*', r'\1cost=0 \2', ln.rstrip('\n')) + '\n' if 'UTPP MOTION' in ln else ln)
    feed = pm.read_feed(a.feed)
    rows_to_frame, info = pm.align_feed(np.concatenate(allsec), feed)
    gids = np.concatenate(gids)
    with open(a.feed, 'r', encoding='utf-8', errors='replace') as f, open(os.path.join(a.out_dir, 'pose_metrics_core.feed.txt'), 'w', encoding='utf-8', newline='\n') as o:
        o.write(f.readline())
        j = 0
        for ln in f:
            if ln.startswith('#'):
                continue
            if rows_to_frame is not None and gids[0] <= rows_to_frame[j] <= gids[-1]:
                o.write(ln)
            j += 1
    print('feed:', info)
    return 0


if __name__ == '__main__':
    sys.exit(main())
