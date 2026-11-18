#!/usr/bin/env python3
"""Replace one alpha curve of a baked motion database (.utppmm v2) with a curve of a reference curve dump.

  patch_curve_from_reference.py --bundle IN.utppmm --curves anim_curves.json --out OUT.utppmm
                                [--target Enable_OrientationWarping] [--source enable_warping]
                                [--prefix M_Neutral] [--report REPORT.json]

For every clip whose name starts with --prefix and that has the --source curve in the dump
({"curves": {asset_path: {curve: {"t": [...], "v": [...]}}}}), the --target curve of every frame of the clip becomes
the source curve sampled at frame / fps (linear, clamped at the ends). The presence bit of the curve is set on those
frames and in the clip's CLP2 curve_mask. Nothing else in the file changes; the payload and header CRCs are rewritten.
The report lists every patched clip with the share of frames whose 0.5 threshold decision changed.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mmbake_motion as mm  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bundle", required=True)
    ap.add_argument("--curves", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--target", default="Enable_OrientationWarping")
    ap.add_argument("--source", default="enable_warping")
    ap.add_argument("--prefix", default="M_Neutral")
    ap.add_argument("--report", default=None)
    a = ap.parse_args()

    b, h, s = mm.read_v2(a.bundle)
    b = bytearray(b)
    so = s["STRS"][2]

    def name(o):
        return bytes(b[so + o:b.index(b"\0", so + o)]).decode()

    C, nfr, fps = h["num_curves"], h["num_frames"], float(h["fps"])
    crvn = [name(struct.unpack_from("<I", b, s["CRVN"][2] + 8 * k)[0]) for k in range(C)]
    if a.target not in crvn:
        raise SystemExit(f"curve {a.target} not in the database ({crvn})")
    k = crvn.index(a.target)
    ccnt, cesz, coff, _ = s["CURV"]
    if ccnt != nfr or cesz != 4 * (C + 1):
        raise SystemExit("unexpected CURV layout")
    raw = np.frombuffer(bytes(b[coff:coff + nfr * cesz]), np.uint32).reshape(nfr, C + 1).copy()
    vals = raw[:, 1:].view(np.float32)
    ref = {p.rsplit(".", 1)[-1]: v for p, v in json.load(open(a.curves, encoding="utf-8"))["curves"].items()}
    clip_cnt, clip_esz, clip_off, _ = s["CLIP"]
    p2cnt, p2esz, p2off, _ = s["CLP2"]
    report = []
    for i in range(clip_cnt):
        no, ff, nf = struct.unpack_from("<III", b, clip_off + clip_esz * i)
        nm = name(no)
        if not nm.startswith(a.prefix):
            continue
        d = ref.get(nm, {})
        src = d.get(a.source)
        if not src:
            continue
        new = np.interp(np.arange(nf) / fps, src["t"], src["v"]).astype(np.float32)
        old_has = bool(np.all(raw[ff:ff + nf, 0] & (1 << k)))
        old = vals[ff:ff + nf, k].copy() if old_has else np.zeros(nf, np.float32)
        vals[ff:ff + nf, k] = new
        raw[ff:ff + nf, 0] |= np.uint32(1 << k)
        m_off = p2off + p2esz * i + 12
        struct.pack_into("<I", b, m_off, struct.unpack_from("<I", b, m_off)[0] | (1 << k))
        flips = float(np.mean((old >= 0.5) != (new >= 0.5)))
        report.append({"clip": nm, "frames": int(nf), "had_curve": old_has, "threshold_flip_share": round(flips, 4),
                       "max_abs_change": round(float(np.max(np.abs(old - new))), 4)})
    b[coff:coff + nfr * cesz] = raw.tobytes()
    payload_start = mm.HEADER_SIZE
    struct.pack_into("<I", b, 92, zlib.crc32(bytes(b[payload_start:])) & 0xFFFFFFFF)
    struct.pack_into("<I", b, 96, 0)
    struct.pack_into("<I", b, 96, zlib.crc32(bytes(b[:mm.HEADER_SIZE])) & 0xFFFFFFFF)
    with open(a.out, "wb") as f:
        f.write(b)
    flipped = [r for r in report if r["threshold_flip_share"] > 0]
    summary = {"bundle": os.path.basename(a.bundle), "target": a.target, "source": a.source,
               "clips_patched": len(report), "clips_threshold_changed": len(flipped),
               "clips_value_changed": sum(r["max_abs_change"] > 1e-3 for r in report),
               "clips_curve_added": sum(not r["had_curve"] for r in report)}
    print(json.dumps(summary))
    if a.report:
        with open(a.report, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"summary": summary, "clips": report}, f, indent=1)


if __name__ == "__main__":
    main()
