#!/usr/bin/env python3
"""Tests for .utppmm format v2 (FORMAT.txt part "VERSION 2").

The reader below is written from FORMAT.txt and does NOT import mmbake_motion (independent of the producer). Checks:
  format_doc     FORMAT.txt v2 header table, section ids / order / element sizes and record field lists vs the
                 file and this reader
  integrity      header, CRCs, section table, padding, strings, counts and every cross-section range
  determinism    (--determinism) second bake with the summary's bake_args, byte-identical file / summary / chooser
  normalization  (--raw-dump) UE mean-deviation normalization (PoseSearchDerivedData.cpp 231-405, 654-712)
                 recomputed from the raw features: deviation, scale, mean and DFEA, rel. err <= 1e-4
  trajectory     trajectory channels recomputed from POSE (Trajectory bone) + RTZ_ only (loops wrap, non-loops
                 extrapolate), compared with DFEA
  search         AABB search == brute force (same row, bitwise-equal cost, ties -> lowest row) on N queries
  block          BlockTransition: synthetic case (continuing pose may enter / stay in a block, a search never
                 starts in one) + real rows: search never returns a BLOCK row, DROW BLOCK == UE notify-state rule
  curve_lag      MotionExtractor speed curves vs the bone / root speed recomputed from POSE: cross-corr. lag 0;
                 a nonzero peak within 0.01 correlation of lag 0 (retargeted foot shape, not a time shift) is refereed
                 with the UE source-skeleton motion (data file sources.ue_tracks) and must have lag 0 there
  rtz            RTZ_ rows == root_z of the RTZ clip files (vz = central difference x fps), flags, zero rows
  rig_identity   header rig_hash == identity hash from BONE + RIGR and from the rig json; other rig refused
  v1             (--v1-rebake) v1 bake of dense_full is byte-identical to Data/utpp_base_dense_full.utppmm
Every check prints passed / failed / not run with numbers. Exit 0 only if nothing failed. Requires numpy.
source-pack-derived inputs: database, raw dump, clips, anims.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
RIGS = {"player_woman_skeleton": os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json"),
        "player_man_skeleton": os.path.join(ROOT, "Extracted", "game", "json", "vanilla-man", "player_man_skeleton.rig.json")}
TOPOLOGY_HASH_0_72 = 0x72CB314EB01F562D
V1_DENSE_FULL = os.path.join(ROOT, "Data", "utpp_base_dense_full.utppmm")

u4, i4, f4, u8 = "<u4", "<i4", "<f4", "<u8"

# ============================================================================== reader (FORMAT.txt, VERSION 2)

HEADER = [(0, 8, "char[8]", "magic"), (8, 4, "u32", "version"), (12, 4, "u32", "header_size"),
          (16, 8, "u64", "file_size"), (24, 8, "u64", "rig_hash"), (32, 4, "u32", "flags"),
          (36, 4, "u32", "num_frames"), (40, 4, "u32", "num_bones"), (44, 4, "u32", "num_clips"),
          (48, 4, "u32", "num_schemas"), (52, 4, "u32", "num_channels"), (56, 4, "u32", "num_norms"),
          (60, 4, "u32", "num_databases"), (64, 4, "u32", "num_entries"), (68, 4, "u32", "num_rows"),
          (72, 4, "u32", "num_curves"), (76, 4, "u32", "num_events"), (80, 4, "f32", "fps"),
          (84, 4, "u32", "num_sections"), (88, 4, "u32", "section_table_offset"), (92, 4, "u32", "payload_crc32"),
          (96, 4, "u32", "header_crc32"), (100, 4, "u32", "bound_sm_size"), (104, 4, "u32", "bound_lr_size"),
          (108, 4, "u32", "max_dims"), (112, 8, "u64", "dfea_floats"), (120, 8, "u64", "dbox_floats")]
STRUCT = {"char[8]": "8s", "u32": "I", "u64": "Q", "f32": "f", "i32": "i"}

DT = {
    "BONE": np.dtype([("name_off", u4), ("parent", i4), ("name_len", u4), ("pad", u4)]),
    "RIGR": np.dtype([(k, f4) for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")]),
    "CLIP": np.dtype([("name_off", u4), ("first", u4), ("n", u4), ("flags", u4), ("loop_period", u4), ("nsel", u4),
                      ("db_mask", u8)]),
    "CLP2": np.dtype([("first_event", u4), ("num_events", u4), ("rtz_source", u4), ("curve_mask", u4),
                      ("hip_ratio", f4), ("length", f4), ("added", u4), ("recomputed", u4)]),
    "SCHM": np.dtype([("name_off", u4), ("dims", u4), ("first_channel", u4), ("num_channels", u4), ("rate", f4),
                      ("preproc", u4), ("wsum", f4), ("pad", u4)]),
    "SCHC": np.dtype([("kind", u4), ("comp", u4), ("bone", i4), ("origin", i4), ("t_sample", f4), ("t_origin", f4),
                      ("vel_space", u4), ("input_query", u4), ("weight", f4), ("weight_norm", f4), ("offset", u4),
                      ("card", u4), ("group_off", u4), ("label_off", u4), ("ax", f4), ("ay", f4), ("az", f4),
                      ("flags", u4), ("pad", u4)]),
    "NORM": np.dtype([("schema", u4), ("dims", u4), ("vofs", u4), ("nmem", u4), ("mask", u8), ("rows", u4),
                      ("pad", u4)]),
    "DBAS": np.dtype([("name_off", u4), ("schema", u4), ("norm", u4), ("family", u4), ("tags", u4), ("flags", u4),
                      ("base", f4), ("cont", f4), ("loop", f4), ("ex_min", f4), ("ex_max", f4), ("first_entry", u4),
                      ("num_entries", u4), ("first_row", u4), ("num_rows", u4), ("dims", u4), ("feat_off", u8),
                      ("box_off", u8), ("nsm", u4), ("nlr", u4), ("min_add", f4), ("nsel", u4)]),
    "DENT": np.dtype([("db", u4), ("clip", u4), ("flags", u4), ("rmin", f4), ("rmax", f4), ("first_row", u4),
                      ("num_rows", u4), ("pad", u4)]),
    "DROW": np.dtype([("frame", u4), ("add", f4), ("cont", f4), ("entry", u4), ("flags", u4)]),
    "CRVN": np.dtype([("name_off", u4), ("unit", u4)]),
    "EVNT": np.dtype([("clip", u4), ("time", f4), ("dur", f4), ("tag", u4), ("cls", u4), ("flags", u4)]),
    "RTZ_": np.dtype([("z", f4), ("vz", f4)]),
    "ROOT": np.dtype([("dx", f4), ("dy", f4), ("dyaw", f4), ("pad", f4)]),
    "CONT": np.dtype([("l", f4), ("r", f4)]),
    "FFLG": np.dtype(u4), "NRMV": np.dtype(f4), "DFEA": np.dtype(f4), "DBOX": np.dtype(f4),
}
ORDER = ["STRS", "BONE", "RIGR", "CLIP", "CLP2", "FFLG", "SCHM", "SCHC", "NORM", "NRMV", "DBAS", "DENT", "DROW", "DFEA",
         "DBOX", "CRVN", "CURV", "EVNT", "RTZ_", "POSE", "ROOT", "CONT", "SCAL"]

# flags (FORMAT.txt v2)
CF_LOOP, CF_RTZ_UNKNOWN, CF_ADDED, CF_RTZ_PRESENT, CF_RTZ_NONE = 1, 8, 16, 32, 64
FF_SELECTABLE, FF_EXCLUDED_NOTIFY, FF_HORIZON, FF_BLOCK, FF_INDEXED = 1, 2, 4, 16, 512
RF_BLOCK, RF_LOOPING = 1, 4
EF_LOOPING, EF_ADDED = 4, 8
DF_RTZ_PENDING = 1
CHF_TRAJECTORY, CHF_Z_FROM_RTZ = 1, 8
K_POS, K_VEL, K_VELDIR, K_HEAD = 1, 2, 3, 4
VS_WORLD = 2


class V2:
    def __init__(self, path):
        self.path = path
        with open(path, "rb") as f:
            self.b = f.read()
        b = self.b
        self.h = {}
        for off, size, typ, name in HEADER:
            v = struct.unpack_from("<" + STRUCT[typ], b, off)[0]
            self.h[name] = v
        self.sec = {}
        self.order = []
        for i in range(self.h["num_sections"]):
            sid, cnt, esz, res, off, size = struct.unpack_from("<IIIIQQ", b, self.h["section_table_offset"] + 32 * i)
            k = struct.pack("<I", sid).decode("ascii")
            self.order.append(k)
            self.sec[k] = dict(count=cnt, elem=esz, reserved=res, off=off, size=size)

    def raw(self, sid):
        s = self.sec[sid]
        return self.b[s["off"]:s["off"] + s["size"]]

    def arr(self, sid, dtype=None):
        s = self.sec[sid]
        dt = np.dtype(dtype) if dtype is not None else DT[sid]
        n = s["size"] // dt.itemsize
        return np.frombuffer(self.b, dt, n, s["off"])

    def s(self, off):
        st = self.sec["STRS"]
        e = self.b.index(b"\x00", st["off"] + off)
        return self.b[st["off"] + off:e].decode("utf-8")

    # convenience views
    def load(self):
        nb, nc = self.h["num_bones"], self.h["num_curves"]
        self.bone = self.arr("BONE")
        self.rigr = self.arr("RIGR")
        self.clip = self.arr("CLIP")
        self.clp2 = self.arr("CLP2")
        self.fflg = self.arr("FFLG")
        self.schm = self.arr("SCHM")
        self.schc = self.arr("SCHC")
        self.norm = self.arr("NORM")
        self.nrmv = self.arr("NRMV")
        self.dbas = self.arr("DBAS")
        self.dent = self.arr("DENT")
        self.drow = self.arr("DROW")
        self.dfea = self.arr("DFEA")
        self.dbox = self.arr("DBOX")
        self.crvn = self.arr("CRVN")
        curv = self.arr("CURV", np.dtype(f4)).reshape(self.h["num_frames"], nc + 1)
        self.cmask = curv.view(np.uint32)[:, 0]
        self.curv = curv[:, 1:]
        self.evnt = self.arr("EVNT")
        self.rtz = self.arr("RTZ_")
        self.pose = self.arr("POSE", np.dtype(f4)).reshape(self.h["num_frames"], nb, 7)
        self.root = self.arr("ROOT")
        self.cont = self.arr("CONT")
        self.names = [self.s(int(x)) for x in self.bone["name_off"]]
        self.parents = [int(x) for x in self.bone["parent"]]
        self.clip_names = [self.s(int(x)) for x in self.clip["name_off"]]
        self.db_names = [self.s(int(x)) for x in self.dbas["name_off"]]
        self.schema_names = [self.s(int(x)) for x in self.schm["name_off"]]
        self.curve_names = [self.s(int(x)) for x in self.crvn["name_off"]]
        return self

    def channels(self, si):
        m = self.schm[si]
        return self.schc[int(m["first_channel"]):int(m["first_channel"]) + int(m["num_channels"])]

    def db_feats(self, d):
        m = self.dbas[d]
        n, D = int(m["num_rows"]), int(m["dims"])
        return self.dfea[int(m["feat_off"]):int(m["feat_off"]) + n * D].reshape(n, D)

    def db_boxes(self, d):
        m = self.dbas[d]
        D = int(m["dims"])
        rec = 2 * D + 1
        o = int(m["box_off"])
        sm = self.dbox[o:o + int(m["nsm"]) * rec].reshape(int(m["nsm"]), rec)
        o += int(m["nsm"]) * rec
        lr = self.dbox[o:o + int(m["nlr"]) * rec].reshape(int(m["nlr"]), rec)
        return sm, lr

    def db_rows(self, d):
        m = self.dbas[d]
        return self.drow[int(m["first_row"]):int(m["first_row"]) + int(m["num_rows"])]

    def norm_vectors(self, ni):
        n = self.norm[ni]
        D, o = int(n["dims"]), int(n["vofs"])
        return self.nrmv[o:o + D], self.nrmv[o + D:o + 2 * D], self.nrmv[o + 2 * D:o + 3 * D]


# ============================================================================== results

class Results:
    def __init__(self):
        self.items = []

    def add(self, name, status, detail):
        self.items.append((name, status, detail))
        print(f"[{status.upper():7s}] {name}: {detail}", flush=True)

    def check(self, name, ok, detail):
        self.add(name, "passed" if ok else "failed", detail)
        return ok

    def failed(self):
        return [i for i in self.items if i[1] == "failed"]


def fnv1a64(data):
    h = 0xCBF29CE484222325
    for x in data:
        h ^= x
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def sha256(b):
    return hashlib.sha256(b).hexdigest()


# ============================================================================== format_doc

def parse_format_v2(path):
    text = open(path, encoding="utf-8").read()
    k = text.index("\nVERSION 2\n")
    v2 = text[k:].splitlines()
    hdr, secs, recs = [], [], {}
    mode = None
    cur = None
    for line in v2:
        if line.startswith("Header (128 bytes)"):
            mode = "hdr"
            continue
        if line.startswith("Rig identity"):
            mode = None
        if line.startswith("Sections (written in this order"):
            mode = "sec"
            continue
        if line.startswith("Feature values"):
            mode = None
        if mode == "hdr":
            m = re.match(r"^\s*(\d+)\s+(\d+)\s+(char\[8\]|u32|u64|f32|i32)\s+(\w+)", line)
            if m:
                hdr.append((int(m.group(1)), int(m.group(2)), m.group(3), m.group(4)))
        elif mode == "sec":
            m = re.match(r"^([A-Z0-9_]{4})\s+(\S+)\s+(\S+)\s+(.*)$", line)
            if m:
                cur = m.group(1)
                secs.append((cur, m.group(2), m.group(3)))
                recs[cur] = m.group(4)
            elif cur and line.startswith(" "):
                recs[cur] += "\n" + line.strip()
    return hdr, secs, recs


def record_types(desc):
    """'{u32 a, i32 b, f32 x, y}' -> ['u32', 'i32', 'f32', 'f32']; None when the record has no typed list."""
    m = re.search(r"\{([^}]*)\}", desc, re.S)
    if not m:
        return None
    out, cur = [], None
    for item in m.group(1).replace("\n", " ").split(","):
        t = item.strip().split()
        if not t:
            continue
        if t[0] in ("u32", "i32", "f32", "u64"):
            cur = t[0]
        elif cur is None:
            return None
        out.append(cur)
    return out


def offset_types(desc):
    return [(int(a), b) for a, b in re.findall(r"(?<![\w.])(\d+) (u32|i32|f32|u64) \w", desc)]


def dtype_types(dt):
    m = {"<u4": "u32", "<i4": "i32", "<f4": "f32", "<u8": "u64"}
    return [m[dt.fields[n][0].str] for n in dt.names]


def dtype_offsets(dt):
    m = {"<u4": "u32", "<i4": "i32", "<f4": "f32", "<u8": "u64"}
    return [(dt.fields[n][1], m[dt.fields[n][0].str]) for n in dt.names]


def test_format_doc(R, db, fmt):
    hdr, secs, recs = parse_format_v2(fmt)
    ok_h = hdr == HEADER
    R.check("format_doc.header", ok_h, f"{len(hdr)} header fields parsed from FORMAT.txt v2, equal to the reader: {ok_h}")
    doc_order = [s[0] for s in secs]
    file_order = db.order
    exp = [s for s in doc_order if s in file_order]
    R.check("format_doc.section_order", file_order == exp and doc_order == ORDER,
            f"file {len(file_order)} sections in FORMAT order: {file_order == exp}; FORMAT ids == reader {doc_order == ORDER}")
    env = {"num_bones": db.h["num_bones"], "C": db.h["num_curves"]}
    bad = []
    for sid, cnt, elem in secs:
        if sid not in db.sec:
            continue
        e = int(eval(elem, {"__builtins__": {}}, env))
        if e != db.sec[sid]["elem"]:
            bad.append(f"{sid} elem doc {e} file {db.sec[sid]['elem']}")
        if sid in DT and DT[sid].itemsize != e:
            bad.append(f"{sid} elem doc {e} reader {DT[sid].itemsize}")
        if cnt in db.h and db.h[cnt] != db.sec[sid]["count"]:
            bad.append(f"{sid} count {db.sec[sid]['count']} != header {cnt} {db.h[cnt]}")
        t = record_types(recs[sid])
        if t is not None and sid in DT and DT[sid].names is not None and t != dtype_types(DT[sid]):
            bad.append(f"{sid} field types doc {t} reader {dtype_types(DT[sid])}")
    for sid in ("SCHC", "DBAS"):
        ot = offset_types(recs[sid])
        if ot != dtype_offsets(DT[sid]):
            bad.append(f"{sid} offset table doc {len(ot)} fields != reader")
    R.check("format_doc.records", not bad, f"element sizes, counts, typed field lists and SCHC/DBAS offset tables vs "
                                           f"FORMAT.txt: {len(bad)} mismatches {bad[:5]}")


# ============================================================================== integrity

def test_integrity(R, db):
    b, h = db.b, db.h
    errs = []
    if h["magic"] != b"UTPPMMDB" or h["version"] != 2 or h["header_size"] != 128:
        errs.append("magic/version/header_size")
    if h["file_size"] != len(b) or len(b) % 16:
        errs.append("file_size")
    hb = bytearray(b[:128])
    hb[96:100] = b"\x00\x00\x00\x00"
    if zlib.crc32(bytes(hb)) & 0xFFFFFFFF != h["header_crc32"]:
        errs.append("header crc")
    if zlib.crc32(b[128:]) & 0xFFFFFFFF != h["payload_crc32"]:
        errs.append("payload crc")
    if h["section_table_offset"] != 128 or h["num_sections"] > 64:
        errs.append("section table")
    if (h["flags"] & ~1) or (("SCAL" in db.sec) != bool(h["flags"] & 1)):
        errs.append("header flags / SCAL")
    # sections: alignment, bounds, size, order, no overlap, zero gaps
    pos = 128 + 32 * h["num_sections"]
    covered = 0
    for k in db.order:
        s = db.sec[k]
        if s["off"] % 16 or s["size"] != s["count"] * s["elem"] or s["reserved"] or s["off"] < pos \
                or s["off"] + s["size"] > len(b):
            errs.append(f"section {k} bounds")
        if any(b[pos:s["off"]]):
            errs.append(f"non-zero gap before {k}")
        pos = s["off"] + s["size"]
        covered += s["size"]
    if any(b[pos:]):
        errs.append("non-zero tail")
    if len(set(db.order)) != len(db.order):
        errs.append("duplicate section ids")
    req = [k for k in ORDER if k != "SCAL"]
    if [k for k in db.order if k != "SCAL"] != req:
        errs.append(f"sections {db.order}")
    R.check("integrity.container", not errs, f"size {len(b)} bytes, {len(db.order)} sections, header/payload CRC, "
                                             f"alignment, zero padding, order: {len(errs)} errors {errs[:5]}")
    if errs:
        return False
    db.load()
    e2 = []
    NF, nb = h["num_frames"], h["num_bones"]
    st = db.raw("STRS")
    if not st or st[0] != 0 or st[-1] != 0 or len(st) % 4:
        e2.append("STRS termination / padding")
    for i, (n, p) in enumerate(zip(db.names, db.parents)):
        if (i == 0) != (p < 0) or p >= i or db.bone["name_len"][i] != len(n.encode("utf-8")) or db.bone["pad"][i]:
            e2.append(f"BONE {i}")
    counts = dict(num_clips=len(db.clip), num_schemas=len(db.schm), num_channels=len(db.schc), num_norms=len(db.norm),
                  num_databases=len(db.dbas), num_entries=len(db.dent), num_rows=len(db.drow), num_curves=len(db.crvn),
                  num_events=len(db.evnt))
    for k, v in counts.items():
        if h[k] != v:
            e2.append(f"{k} {h[k]} != {v}")
    if len(db.rigr) != nb or len(db.clp2) != len(db.clip) or len(db.fflg) != NF or len(db.rtz) != NF \
            or len(db.root) != NF or len(db.cont) != NF:
        e2.append("per-bone / per-clip / per-frame section counts")
    if h["dfea_floats"] != len(db.dfea) or h["dbox_floats"] != len(db.dbox) or h["num_databases"] > 64 \
            or h["num_curves"] > 32 or h["fps"] != 30.0:
        e2.append("dfea/dbox floats, limits, fps")
    if h["bound_sm_size"] <= 0 or h["bound_lr_size"] % h["bound_sm_size"]:
        e2.append("box sizes")
    # clips cover the frames contiguously in name order
    first = 0
    for i, c in enumerate(db.clip):
        if int(c["first"]) != first or c["n"] < 2:
            e2.append(f"clip {i} coverage")
        if (c["loop_period"] != (c["n"] - 1 if c["flags"] & CF_LOOP else 0)):
            e2.append(f"clip {i} loop_period")
        fl = db.fflg[first:first + int(c["n"])]
        if int(((fl & FF_SELECTABLE) != 0).sum()) != int(c["nsel"]):
            e2.append(f"clip {i} num_selectable")
        nr = sum(1 for x in (CF_RTZ_UNKNOWN, CF_RTZ_PRESENT, CF_RTZ_NONE) if c["flags"] & x)
        if nr != 1:
            e2.append(f"clip {i} RTZ flags")
        first += int(c["n"])
        ce = db.clp2[i]
        if int(ce["first_event"]) + int(ce["num_events"]) > len(db.evnt):
            e2.append(f"clip {i} events range")
    if first != NF or db.clip_names != sorted(db.clip_names):
        e2.append("clip coverage / order")
    if np.any(db.fflg & FF_HORIZON):
        e2.append("FFLG bit2 set")
    # schemas / channels
    md = 0
    for si, m in enumerate(db.schm):
        ch = db.channels(si)
        D = int(m["dims"])
        md = max(md, D)
        if int(m["first_channel"]) + int(m["num_channels"]) > len(db.schc) or m["preproc"] not in (0, 1, 2):
            e2.append(f"schema {si}")
        off = 0
        for c in ch:
            if int(c["offset"]) != off or int(c["card"]) != {7: 3, 3: 2, 4: 1}[int(c["comp"])]:
                e2.append(f"schema {si} channel layout")
            off += int(c["card"])
        if off != D or abs(float((ch["weight"] * ch["card"]).sum()) - float(m["wsum"])) > 1e-3 * max(1.0, float(m["wsum"])):
            e2.append(f"schema {si} dims / weight_sum")
    if md != h["max_dims"]:
        e2.append("max_dims")
    for ni, n in enumerate(db.norm):
        if int(n["dims"]) != int(db.schm[int(n["schema"])]["dims"]) or int(n["vofs"]) + 3 * int(n["dims"]) > len(db.nrmv):
            e2.append(f"norm {ni}")
    mean_ok = bool(np.all(np.isfinite(db.nrmv))) and all(np.all(db.norm_vectors(i)[1] > 0) for i in range(len(db.norm)))
    if not mean_ok:
        e2.append("NRMV finite / positive scale")
    # databases, entries, rows, features, boxes
    e_cursor = r_cursor = f_cursor = b_cursor = 0
    for d, m in enumerate(db.dbas):
        D = int(m["dims"])
        if D != int(db.schm[int(m["schema"])]["dims"]) or int(db.norm[int(m["norm"])]["schema"]) != int(m["schema"]):
            e2.append(f"db {d} schema / norm")
        if int(m["first_entry"]) != e_cursor or int(m["first_row"]) != r_cursor or int(m["feat_off"]) != f_cursor \
                or int(m["box_off"]) != b_cursor:
            e2.append(f"db {d} ranges not contiguous")
        nr = int(m["num_rows"])
        sm, lr = h["bound_sm_size"], h["bound_lr_size"]
        if int(m["nsm"]) != (nr + sm - 1) // sm or int(m["nlr"]) != (nr + lr - 1) // lr:
            e2.append(f"db {d} box counts")
        rows = db.db_rows(d)
        ents = db.dent[e_cursor:e_cursor + int(m["num_entries"])]
        if np.any(ents["db"] != d):
            e2.append(f"db {d} entries")
        rc = r_cursor
        for e in ents:
            if int(e["first_row"]) != rc or int(e["clip"]) >= len(db.clip):
                e2.append(f"db {d} entry rows")
            er = db.drow[int(e["first_row"]):int(e["first_row"]) + int(e["num_rows"])]
            c = db.clip[int(e["clip"])]
            if len(er) and (np.any(er["frame"] < c["first"]) or np.any(er["frame"] >= c["first"] + c["n"])):
                e2.append(f"db {d} row outside its clip")
            if not (db.clip["db_mask"][int(e["clip"])] >> np.uint64(d)) & np.uint64(1):
                e2.append(f"db {d} clip db_mask")
            rc += int(e["num_rows"])
        if rc != r_cursor + nr:
            e2.append(f"db {d} entry rows sum")
        sel = (rows["flags"] & RF_BLOCK) == 0
        if int(sel.sum()) != int(m["nsel"]) or (sel.any() and np.float32(rows["add"][sel].min()) != m["min_add"]):
            e2.append(f"db {d} num_selectable / min_addend")
        F = db.db_feats(d)
        if not np.all(np.isfinite(F)):
            e2.append(f"db {d} features")
        # boxes contain their non-block rows
        for size, boxes in ((sm, db.db_boxes(d)[0]), (lr, db.db_boxes(d)[1])):
            for bi in range(len(boxes)):
                sl = slice(bi * size, min((bi + 1) * size, nr))
                ok = sel[sl]
                if ok.any():
                    rr = F[sl][ok]
                    if np.any(rr < boxes[bi, :D]) or np.any(rr > boxes[bi, D:2 * D]) \
                            or np.float32(rows["add"][sl][ok].min()) != boxes[bi, 2 * D]:
                        e2.append(f"db {d} box {bi} size {size}")
                        break
                elif np.any(boxes[bi, :2 * D] != 0) or boxes[bi, 2 * D] != np.finfo(np.float32).max:
                    e2.append(f"db {d} empty box {bi}")
                    break
        e_cursor += int(m["num_entries"])
        r_cursor += nr
        f_cursor += nr * D
        b_cursor += (int(m["nsm"]) + int(m["nlr"])) * (2 * D + 1)
    if e_cursor != len(db.dent) or r_cursor != len(db.drow) or f_cursor != len(db.dfea) or b_cursor != len(db.dbox):
        e2.append("database ranges do not cover DENT / DROW / DFEA / DBOX")
    if np.any(db.drow["frame"] >= NF) or np.any(db.drow["entry"] >= len(db.dent)):
        e2.append("DROW frame / entry")
    # entry index of the rows
    ent_of_row = np.repeat(np.arange(len(db.dent)), db.dent["num_rows"].astype(np.int64))
    if len(ent_of_row) != len(db.drow) or np.any(db.drow["entry"] != ent_of_row):
        e2.append("DROW entry != DENT ranges")
    # FFLG derived bits vs rows
    idx = np.zeros(NF, bool)
    idx[db.drow["frame"]] = True
    selm = np.zeros(NF, bool)
    selm[db.drow["frame"][(db.drow["flags"] & RF_BLOCK) == 0]] = True
    if np.any(((db.fflg & FF_INDEXED) != 0) != idx) or np.any(((db.fflg & FF_SELECTABLE) != 0) != selm):
        e2.append("FFLG INDEXED / SELECTABLE vs DROW")
    q = db.pose[..., 3:7].astype(np.float64)
    qn = np.abs(np.linalg.norm(q, axis=-1) - 1.0).max()
    if qn > 1e-5 or not np.all(np.isfinite(db.pose)):
        e2.append(f"POSE quaternions {qn:.2e}")
    for k in ("ROOT", "CONT", "RTZ_", "CURV", "RIGR"):
        if not np.all(np.isfinite(db.arr(k, np.dtype(f4)))):
            e2.append(f"{k} not finite")
    if np.any(db.evnt["clip"] >= len(db.clip)):
        e2.append("EVNT clip")
    R.check("integrity.content", not e2, f"{NF} frames, {len(db.clip)} clips, {len(db.dbas)} databases, "
                                         f"{len(db.dent)} entries, {len(db.drow)} rows, {len(db.schm)} schemas, "
                                         f"{len(db.norm)} norms; POSE max |q|-1 = {qn:.1e}: {len(e2)} errors {e2[:6]}")
    return not e2


# ============================================================================== normalization (UE formula)

UE_CLASS = {K_POS: "Position", K_VEL: "Velocity", K_VELDIR: "Velocity", K_HEAD: "Heading"}


def norm_groups(db, chans):
    groups = []
    for c in chans:
        for g in groups:
            a = g[0]
            if int(a["card"]) != int(c["card"]) or UE_CLASS[int(a["kind"])] != UE_CLASS[int(c["kind"])]:
                continue
            ga = None if a["group_off"] == 0xFFFFFFFF else db.s(int(a["group_off"]))
            gc = None if c["group_off"] == 0xFFFFFFFF else db.s(int(c["group_off"]))
            if (ga is not None and ga == gc) or db.s(int(a["label_off"])) == db.s(int(c["label_off"])):
                g.append(c)
                break
        else:
            groups.append([c])
    return groups


def test_normalization(R, db, raw_path):
    if not raw_path or not os.path.exists(raw_path):
        R.add("normalization", "not run", f"raw dump {raw_path!r} not available (bake with --raw-dump)")
        return
    raw = np.load(raw_path)
    worst_dev = worst_scale = worst_mean = worst_feat = 0.0
    n_rows = 0
    for ni, n in enumerate(db.norm):
        si = int(n["schema"])
        sname = db.schema_names[si]
        X_all = raw[f"raw_{sname}"]
        chans = db.channels(si)
        D = int(n["dims"])
        members = [d for d in range(len(db.dbas)) if (int(n["mask"]) >> d) & 1]
        frames = np.concatenate([db.db_rows(d)["frame"].astype(np.int64) for d in members])
        if len(frames) != int(n["rows"]):
            R.check("normalization", False, f"norm {ni}: rows {n['rows']} != member rows {len(frames)}")
            return
        X = X_all[frames].astype(np.float64)
        dev = np.ones(D)
        for g in norm_groups(db, chans):
            V = np.concatenate([X[:, int(c["offset"]):int(c["offset"]) + int(c["card"])] for c in g], axis=0)
            mad = float(np.linalg.norm(V - V.mean(axis=0), axis=1).mean())
            # UE: deviation > 0.1 UE units else 1; UE units = cm for metres, unitless for directions / headings
            unitless = int(g[0]["kind"]) in (K_VELDIR, K_HEAD)
            ue = mad if unitless else mad * 100.0
            d_ue = ue if ue > 0.1 else 1.0
            dv = d_ue if unitless else d_ue / 100.0
            for c in g:
                dev[int(c["offset"]):int(c["offset"]) + int(c["card"])] = dv
        w = np.zeros(D)
        for c in chans:
            w[int(c["offset"]):int(c["offset"]) + int(c["card"])] = float(c["weight"])
        scale = np.sqrt(w / w.sum()) / dev
        mean = X.mean(axis=0)
        fm, fs, fd = db.norm_vectors(ni)
        worst_dev = max(worst_dev, float(np.max(np.abs(fd - dev) / dev)))
        worst_scale = max(worst_scale, float(np.max(np.abs(fs - scale) / scale)))
        worst_mean = max(worst_mean, float(np.max(np.abs(fm - mean) / np.maximum(1.0, np.abs(mean)))))
    for d in range(len(db.dbas)):
        m = db.dbas[d]
        ni = int(m["norm"])
        si = int(m["schema"])
        X = raw[f"raw_{db.schema_names[si]}"][db.db_rows(d)["frame"].astype(np.int64)].astype(np.float64)
        fm, fs, _ = db.norm_vectors(ni)
        exp = (X - fm.astype(np.float64)) * fs.astype(np.float64)
        F = db.db_feats(d).astype(np.float64)
        worst_feat = max(worst_feat, float(np.max(np.abs(F - exp) / np.maximum(1.0, np.abs(exp)))) if len(F) else 0.0)
        n_rows += len(F)
    ok = max(worst_dev, worst_scale, worst_mean, worst_feat) <= 1e-4
    R.check("normalization", ok, f"{len(db.norm)} norms / {len(db.dbas)} databases / {n_rows} rows vs UE mean-deviation "
                                 f"formula: max rel err deviation {worst_dev:.2e}, scale {worst_scale:.2e}, mean "
                                 f"{worst_mean:.2e}, DFEA {worst_feat:.2e} (limit 1e-4)")


# ============================================================================== trajectory recompute (POSE + RTZ)

def p_comp(a, b):
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([a[..., 0] + c * b[..., 0] - s * b[..., 1], a[..., 1] + s * b[..., 0] + c * b[..., 1], a[..., 2] + b[..., 2]], -1)


def p_inv(a):
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([-(c * a[..., 0] + s * a[..., 1]), -(-s * a[..., 0] + c * a[..., 1]), -a[..., 2]], -1)


class ClipRoot:
    """Trajectory (x, y, yaw) and root_z of one clip on the 60 Hz grid, from POSE bone 1 and RTZ_ (FORMAT.txt v2)."""

    def __init__(self, db, ci):
        c = db.clip[ci]
        a, n = int(c["first"]), int(c["n"])
        p = db.pose[a:a + n, 1].astype(np.float64)
        self.T = np.stack([p[:, 0], p[:, 1], np.unwrap(2.0 * np.arctan2(p[:, 5], p[:, 6]))], -1)
        self.z = db.rtz["z"][a:a + n].astype(np.float64)
        self.N = n
        self.loop = bool(c["flags"] & CF_LOOP)
        self.memo = {}

    def at(self, h):
        """(x, y, yaw, z) at half-frame index h (int)."""
        if h not in self.memo:
            self.memo[h] = self._at(h)
        return self.memo[h]

    def _at(self, h):
        H = 2 * (self.N - 1)
        T, z = self.T, self.z

        def inside(k):
            if k % 2 == 0:
                return np.append(T[k // 2], z[k // 2])
            i = k // 2
            return np.append(0.5 * (T[i] + T[i + 1]), 0.5 * (z[i] + z[i + 1]))
        if self.loop:
            q, r = divmod(h, H)
            base = inside(r)
            D = p_comp(T[-1], p_inv(T[0]))
            acc = np.zeros(3)
            for _ in range(abs(q)):
                acc = p_comp(D if q > 0 else p_inv(D), acc)
            out = p_comp(acc, base[:3])
            return np.append(out, base[3] + q * (z[-1] - z[0]))
        if 0 <= h <= H:
            return inside(h)
        if h > H:
            e = h - H
            d = p_comp(p_inv(T[-2]), T[-1])
            dz = z[-1] - z[-2]
            acc = np.zeros(3)
            for _ in range(e // 2):
                acc = p_comp(acc, d)
            if e % 2:
                acc = p_comp(acc, 0.5 * d)
            return np.append(p_comp(T[-1], acc), z[-1] + 0.5 * e * dz)
        e = -h
        d0 = p_comp(p_inv(T[1]), T[0])
        dz0 = z[0] - z[1]
        acc = np.zeros(3)
        for _ in range(e // 2):
            acc = p_comp(acc, d0)
        if e % 2:
            acc = p_comp(acc, 0.5 * d0)
        return np.append(p_comp(T[0], acc), z[0] + 0.5 * e * dz0)


def traj_value(cr, kind, hs, ho, vel_world, axis):
    def rel(a, b):
        A, B = cr.at(a), cr.at(b)
        c, s = math.cos(-B[2]), math.sin(-B[2])
        dx, dy = A[0] - B[0], A[1] - B[1]
        return np.array([c * dx - s * dy, s * dx + c * dy, A[3] - B[3]])
    if kind == K_POS:
        return rel(hs, ho)
    if kind in (K_VEL, K_VELDIR):
        v = (rel(hs, ho) - rel(hs - 1, ho if vel_world else ho - 1)) * 60.0
        if kind == K_VELDIR:
            u = v * 100.0
            n = float(np.linalg.norm(u))
            v = u / n if n > 1.0 else u
        return v
    A, B = cr.at(hs), cr.at(ho)
    dyaw = A[2] - B[2]
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([c * axis[0] - s * axis[1], s * axis[0] + c * axis[1], axis[2]])


def strip(v, comp):
    return v if comp == 7 else (v[:2] if comp == 3 else v[2:3])


def test_trajectory(R, db, max_rows_per_db):
    clip_first = db.clip["first"].astype(np.int64)
    roots = {}
    stats = {}
    n_checked = n_extrap = 0
    rng = np.random.default_rng(7)
    for d in range(len(db.dbas)):
        m = db.dbas[d]
        si = int(m["schema"])
        chans = [c for c in db.channels(si) if int(c["bone"]) < 0 and int(c["origin"]) < 0]
        if not chans:
            continue
        fm, fs, _ = db.norm_vectors(int(m["norm"]))
        F = db.db_feats(d)
        rows = db.db_rows(d)
        idx = np.arange(len(rows))
        if len(idx) > max_rows_per_db:
            idx = np.sort(rng.choice(idx, max_rows_per_db, replace=False))
        for r in idx:
            fr = int(rows["frame"][r])
            ci = int(np.searchsorted(clip_first, fr, side="right") - 1)
            if ci not in roots:
                roots[ci] = ClipRoot(db, ci)
            cr = roots[ci]
            i = fr - int(clip_first[ci])
            ext = False
            for c in chans:
                hs = 2 * i + int(round(float(c["t_sample"]) * 60))
                ho = 2 * i + int(round(float(c["t_origin"]) * 60))
                ext |= (not cr.loop) and (hs > 2 * (cr.N - 1) or hs - 1 < 0)
                v = strip(traj_value(cr, int(c["kind"]), hs, ho, int(c["vel_space"]) == VS_WORLD,
                                     (float(c["ax"]), float(c["ay"]), float(c["az"]))), int(c["comp"]))
                if int(c["kind"]) == K_HEAD:
                    n2 = float(v @ v)
                    if n2 > 1e-8:
                        v = v / math.sqrt(n2)
                o, k = int(c["offset"]), int(c["card"])
                exp = (v - fm[o:o + k]) * fs[o:o + k]
                err = float(np.max(np.abs(F[r, o:o + k] - exp)))
                key = ("Pos", "Vel", "VelDir", "Head")[int(c["kind"]) - 1] + ("Z" if int(c["flags"]) & CHF_Z_FROM_RTZ else "")
                stats[key] = max(stats.get(key, 0.0), err)
            n_checked += 1
            n_extrap += ext
    lim = {"Pos": 1e-3, "PosZ": 1e-3, "Vel": 5e-3, "VelZ": 5e-3, "VelDir": 2e-2, "VelDirZ": 2e-2, "Head": 1e-3}
    ok = all(v <= lim.get(k, 1e-3) for k, v in stats.items())
    R.check("trajectory", ok, f"{n_checked} rows ({n_extrap} with extrapolated root) of every database: trajectory "
                              f"channels from POSE Trajectory + RTZ_ vs DFEA, max abs err (normalized units) "
                              + ", ".join(f"{k} {v:.1e}" for k, v in sorted(stats.items()))
                              + " (limits Pos/Head 1e-3, Vel 5e-3, VelDir 2e-2)")


# ============================================================================== search

def seq_cost(F, q):
    acc = np.zeros(len(F), np.float32)
    for k in range(F.shape[1]):
        dk = (F[:, k] - q[k]).astype(np.float32)
        acc = (acc + dk * dk).astype(np.float32)
    return acc


def seq_lb(mn, mx, q):
    acc = np.zeros(len(mn), np.float32)
    zero = np.float32(0)
    for k in range(mn.shape[1]):
        dk = np.maximum(zero, np.maximum((mn[:, k] - q[k]).astype(np.float32), (q[k] - mx[:, k]).astype(np.float32)))
        acc = (acc + dk * dk).astype(np.float32)
    return acc


def brute(F, add, block, q):
    c = (seq_cost(F, q) + add).astype(np.float32)
    c[block] = np.float32(np.inf)
    i = int(np.argmin(c))           # first minimum = lowest row
    return (i, c[i]) if np.isfinite(c[i]) else (-1, np.float32(np.inf))


def aabb(F, add, block, sm_boxes, lr_boxes, sm, lr, q, best_row=-1, best=np.float32(np.inf)):
    D = F.shape[1]
    lb_l = (seq_lb(lr_boxes[:, :D], lr_boxes[:, D:2 * D], q) + lr_boxes[:, 2 * D]).astype(np.float32)
    lb_s = (seq_lb(sm_boxes[:, :D], sm_boxes[:, D:2 * D], q) + sm_boxes[:, 2 * D]).astype(np.float32)
    per = lr // sm
    for L in range(len(lr_boxes)):
        if not lb_l[L] < best:
            continue
        for s in range(L * per, min((L + 1) * per, len(sm_boxes))):
            if not lb_s[s] < best:
                continue
            a, b = s * sm, min((s + 1) * sm, len(F))
            c = (seq_cost(F[a:b], q) + add[a:b]).astype(np.float32)
            for j in range(b - a):
                if not block[a + j] and c[j] < best:
                    best, best_row = c[j], a + j
    return best_row, best


def test_search(R, db, nq, seed):
    rng = np.random.default_rng(seed)
    sm, lr = db.h["bound_sm_size"], db.h["bound_lr_size"]
    dbs = [d for d in range(len(db.dbas)) if int(db.dbas[d]["nsel"]) > 0]
    mism = []
    ties = exact = 0
    for qi in range(nq):
        d = int(rng.choice(dbs))
        F = db.db_feats(d)
        rows = db.db_rows(d)
        add = rows["add"].astype(np.float32)
        block = (rows["flags"] & RF_BLOCK) != 0
        smb, lrb = db.db_boxes(d)
        r0 = int(rng.integers(len(F)))
        q = F[r0].astype(np.float32).copy()
        if qi % 2:
            q = (q + rng.normal(0.0, 0.3, size=q.shape)).astype(np.float32)
        else:
            exact += 1
        bi, bc = brute(F, add, block, q)
        ai, ac = aabb(F, add, block, smb, lrb, sm, lr, q)
        full = (seq_cost(F, q) + add).astype(np.float32)
        full[block] = np.inf
        if np.sum(full == bc) > 1:
            ties += 1
        if bi != ai or np.float32(bc).tobytes() != np.float32(ac).tobytes() or (bi >= 0 and block[bi]):
            mism.append((d, r0, bi, ai, float(bc), float(ac)))
    R.check("search", not mism, f"{nq} queries over {len(dbs)} databases ({exact} exact row copies, {nq - exact} noisy; "
                                f"{ties} with tied minimum cost): AABB == brute force (row, bitwise cost, lowest row on "
                                f"ties) {nq - len(mism)}/{nq}; mismatches {mism[:3]}")


# ============================================================================== BlockTransition

def search_with_continuing(F, add, block, sm, lr, q, cont_row=None, cont_add=np.float32(0)):
    """Continuing pose first (allowed inside a block), then the AABB search over non-block rows (strict <)."""
    best_row, best = -1, np.float32(np.inf)
    if cont_row is not None:
        best = np.float32(seq_cost(F[cont_row:cont_row + 1], q)[0] + np.float32(cont_add))
        best_row = cont_row
    smb = boxes_of(F, add, block, sm)
    lrb = boxes_of(F, add, block, lr)
    return aabb(F, add, block, smb, lrb, sm, lr, q, best_row, best)


def boxes_of(F, add, block, size):
    n, D = F.shape
    nb = (n + size - 1) // size
    out = np.zeros((nb, 2 * D + 1), np.float32)
    out[:, 2 * D] = np.finfo(np.float32).max
    for b in range(nb):
        sl = slice(b * size, min((b + 1) * size, n))
        ok = ~block[sl]
        if ok.any():
            out[b, :D] = F[sl][ok].min(0)
            out[b, D:2 * D] = F[sl][ok].max(0)
            out[b, 2 * D] = add[sl][ok].min()
    return out


def test_block(R, db, sourcepack_dir, data_file, max_queries_per_db):
    # synthetic database: 100 rows on a smooth curve, rows 30..49 BlockTransition
    n, D = 100, 4
    t = np.arange(n, dtype=np.float64) / 10.0
    F = np.stack([np.sin(t), np.cos(t), 0.1 * t, np.sin(2 * t)], -1).astype(np.float32)
    add = np.zeros(n, np.float32)
    block = np.zeros(n, bool)
    block[30:50] = True
    fails = []
    for r in range(n):
        row, _ = search_with_continuing(F, add, block, 16, 64, F[r])
        if row < 0 or block[row]:
            fails.append(f"search from row {r} returned {row}")
    row, _ = search_with_continuing(F, add, block, 16, 64, F[40], cont_row=40)
    if row != 40:
        fails.append(f"continuing pose inside the block was replaced by {row}")
    row, _ = search_with_continuing(F, add, block, 16, 64, F[30], cont_row=30)        # continuation 29 -> 30 enters
    if row != 30:
        fails.append(f"continuing pose entering the block was replaced by {row}")
    row, _ = search_with_continuing(F, add, block, 16, 64, F[40], cont_row=40, cont_add=np.float32(5.0))
    if row < 0 or block[row]:
        fails.append(f"expensive continuing pose: search returned block row {row}")
    R.check("block.synthetic", not fails, f"100-row database, rows 30..49 BLOCK: {n} searches never start in the block, "
                                          f"continuing pose enters / stays in it, replaced only by a non-block row when "
                                          f"cheaper: {len(fails)} failures {fails[:3]}")
    # real rows: query = a BLOCK row's own features
    sm, lr = db.h["bound_sm_size"], db.h["bound_lr_size"]
    nq = bad = 0
    rng = np.random.default_rng(3)
    for d in range(len(db.dbas)):
        rows = db.db_rows(d)
        blk = (rows["flags"] & RF_BLOCK) != 0
        if not blk.any() or not (~blk).any():
            continue
        Fd = db.db_feats(d)
        smb, lrb = db.db_boxes(d)
        cand = np.nonzero(blk)[0]
        if len(cand) > max_queries_per_db:
            cand = rng.choice(cand, max_queries_per_db, replace=False)
        for r in cand:
            row, _ = aabb(Fd, rows["add"].astype(np.float32), blk, smb, lrb, sm, lr, Fd[r].astype(np.float32))
            nq += 1
            if row < 0 or blk[row]:
                bad += 1
    R.check("block.real_rows", bad == 0, f"{nq} searches with a BLOCK row's features as the query: a BLOCK row "
                                         f"returned {bad} times")
    # DROW BLOCK flag == UE rule from the source pack notify states (anims.json), window 1/120 s centred on min(i/30, L)
    anims_p = os.path.join(sourcepack_dir, "anims.json")
    if not os.path.exists(anims_p) or not os.path.exists(data_file):
        R.add("block.ue_rule", "not run", f"{anims_p} or {data_file} missing")
        return
    cls = set(json.load(open(data_file, encoding="utf-8"))["notify_classes"]["block"])
    anims = {a["name"]: a for a in json.load(open(anims_p, encoding="utf-8"))["anims"]}
    rows_checked = mism = 0
    for ei, e in enumerate(db.dent):
        c = int(e["clip"])
        name = db.clip_names[c]
        er = db.drow[int(e["first_row"]):int(e["first_row"]) + int(e["num_rows"])]
        if db.clp2[c]["added"]:
            exp = np.zeros(len(er), bool)
        else:
            a = anims[name]
            st_ = [(float(x["time"]), float(x.get("duration", 0.0))) for x in a.get("notifies") or [] if x.get("class") in cls]
            L = float(np.float32(a["length_s"]))
            i = er["frame"].astype(np.int64) - int(db.clip[c]["first"])
            tt = np.minimum((i.astype(np.float32) / np.float32(30.0)).astype(np.float64), L)
            lo, hi = tt - 1.0 / 240.0, tt + 1.0 / 240.0
            exp = np.zeros(len(er), bool)
            for t0, du in st_:
                exp |= (t0 <= hi) & (t0 + du > lo)
        got = (er["flags"] & RF_BLOCK) != 0
        rows_checked += len(er)
        mism += int((got != exp).sum())
    R.check("block.ue_rule", mism == 0, f"{rows_checked} rows: DROW BLOCK vs BlockTransition notify states active in "
                                        f"the 1/120 s window (anims.json): {mism} mismatches; "
                                        f"{int(((db.drow['flags'] & RF_BLOCK) != 0).sum())} block rows")


# ============================================================================== curve lag

def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


def qrot(q, v):
    u, w = q[..., :3], q[..., 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def model_pos(db, bone, sl):
    chain = []
    b = bone
    while b >= 0:
        chain.append(b)
        b = db.parents[b]
    chain.reverse()
    P = db.pose[sl].astype(np.float64)
    gp = np.zeros((P.shape[0], 3))
    gq = np.zeros((P.shape[0], 4))
    gq[:, 3] = 1.0
    for b in chain:
        gp = gp + qrot(gq, P[:, b, :3])
        gq = qmul(gq, P[:, b, 3:7])
    return gp


def lag_of(a, b, max_lag=10):
    a = a - a.mean()
    b = b - b.mean()
    n = len(a)
    best, arg = -np.inf, 0
    for lag in range(-min(max_lag, n - 2), min(max_lag, n - 2) + 1):
        x = a[max(0, lag):n + min(0, lag)]
        y = b[max(0, -lag):n + min(0, -lag)]
        c = float(np.dot(x, y)) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-12)
        if c > best + 1e-12:
            best, arg = c, lag
    return arg, best


def test_curve_lag(R, db, sourcepack_dir, data_file, bonemap_file):
    try:
        cfg = json.load(open(data_file, encoding="utf-8"))
        anims = json.load(open(os.path.join(sourcepack_dir, "anims.json"), encoding="utf-8"))["anims"]
        bm = json.load(open(bonemap_file, encoding="utf-8"))
    except (OSError, ValueError) as ex:
        R.add("curve_lag", "not run", f"inputs missing: {ex}")
        return
    ue_to_game = {e["src"].lower(): e["cp"] for e in bm["bones"]}
    # UE source-skeleton tracks (same frame time base as POSE, no retarget distortion): referee for near-tie peaks
    ut_path = os.path.join(ROOT, cfg["sources"].get("ue_tracks", "")) if cfg["sources"].get("ue_tracks") else None
    ue_tracks = json.load(open(ut_path, encoding="utf-8"))["clips"] if ut_path and os.path.exists(ut_path) else {}
    inst = {}
    for a in anims:
        for u in (a.get("props") or {}).get("asset_user_data") or []:
            for m in u.get("animation_modifier_instances") or []:
                inst.setdefault(m["_class"], m)
    speed = {}
    for mcls, p in cfg["modifiers"].items():
        if p["kind"] == "motion_extractor" and p.get("feasible", True) and mcls in inst and p["curve"] in db.curve_names:
            m = inst[mcls]
            if m.get("motion_type") == "TRANSLATION_SPEED":
                bn = m["bone_name"]
                gb = -1 if bn == "root" else db.names.index(ue_to_game[bn.lower()])
                speed[db.curve_names.index(p["curve"])] = (gb, m.get("axis", "XYZ"), bn)
    if not speed:
        R.add("curve_lag", "not run", "no MotionExtractor speed curve in CRVN")
        return
    TIE = 0.01          # correlation difference below which the POSE peak is ambiguous (retargeted foot shape)
    n = zero = flat = weak = 0
    nonzero = []
    tie_ue0 = []
    for ci, c in enumerate(db.clip):
        a, N = int(c["first"]), int(c["n"])
        sl = slice(a, a + N)
        tr = ClipRoot(db, ci)
        for k, (gb, axis, ue_bone) in speed.items():
            if not (int(db.clp2[ci]["curve_mask"]) >> k) & 1:
                continue
            if gb < 0:
                W = np.stack([tr.T[:, 0], tr.T[:, 1], tr.z], -1)
            else:
                gp = model_pos(db, gb, sl)
                c2, s2 = np.cos(tr.T[:, 2]), np.sin(tr.T[:, 2])
                W = np.stack([c2 * gp[:, 0] - s2 * gp[:, 1] + tr.T[:, 0], s2 * gp[:, 0] + c2 * gp[:, 1] + tr.T[:, 1],
                              gp[:, 2] + tr.z], -1)
            sel = [i for i, ax in enumerate("XYZ") if ax in axis]
            v = np.linalg.norm(np.diff(W[:, sel], axis=0), axis=-1) * 30.0
            ref = np.concatenate([v[:1], v])
            cv = db.curv[sl, k].astype(np.float64)
            if np.ptp(cv) < 1e-3 or np.ptp(ref) < 1e-3:
                flat += 1
                continue
            lag, corr = lag_of(cv, ref)
            n += 1
            if corr < 0.5:
                weak += 1
            if lag == 0:
                zero += 1
                continue
            c0 = lag_of(cv, ref, max_lag=0)[1]
            item = (db.clip_names[ci], db.curve_names[k], lag, round(corr, 4), round(c0, 4))
            nonzero.append(item)
            bt = ue_tracks.get(db.clip_names[ci], {}).get("bones", {}).get("__object__" if ue_bone == "root" else ue_bone)
            if corr - c0 <= TIE and bt is not None and len(bt) == N:
                P = np.asarray(bt, float)[:, :3]
                u = np.linalg.norm(np.diff(P[:, sel], axis=0), axis=-1)
                lag_u, corr_u = lag_of(cv, np.concatenate([u[:1], u]))
                if lag_u == 0:
                    tie_ue0.append(item + (round(corr_u, 4),))
    bad = [x for x in nonzero if x[3] >= 0.5 and x[:2] not in [t[:2] for t in tie_ue0]]
    R.check("curve_lag", not bad, f"{n} speed curves ({', '.join(db.curve_names[k] for k in speed)}) vs bone / root speed "
                                  f"from POSE (+RTZ): lag 0 for {zero}, nonzero {len(nonzero)}; of these, near-tie peaks "
                                  f"(corr(peak) - corr(0) <= {TIE}) with lag 0 against the UE source motion: {len(tie_ue0)} "
                                  f"{tie_ue0[:4]}; unexplained nonzero (corr >= 0.5): {len(bad)} {bad[:4]}; peak "
                                  f"correlation < 0.5: {weak}, flat skipped {flat}")


# ============================================================================== RTZ

def test_rtz(R, db, rtz_dirs):
    files = {}
    for d in [x for x in (rtz_dirs or "").split(";") if x.strip()]:
        dd = d if os.path.isabs(d) else os.path.join(ROOT, d)
        for f in sorted(os.listdir(dd)):
            if f.endswith(".utppclip.json"):
                files[f[:-len(".utppclip.json")]] = os.path.join(dd, f)
    flags = db.clip["flags"]
    present = [i for i in range(len(db.clip)) if flags[i] & CF_RTZ_PRESENT]
    unknown = [i for i in range(len(db.clip)) if flags[i] & CF_RTZ_UNKNOWN]
    none = [i for i in range(len(db.clip)) if flags[i] & CF_RTZ_NONE]
    errs = []
    src_exp = {CF_RTZ_NONE: 0, CF_RTZ_UNKNOWN: 1, CF_RTZ_PRESENT: 2}
    for i in range(len(db.clip)):
        f = int(flags[i])
        for bit, v in src_exp.items():
            if f & bit and int(db.clp2[i]["rtz_source"]) != v:
                errs.append(f"{db.clip_names[i]} rtz_source")
    zmax = 0.0
    baked = 0
    hips_dev = 0.0
    rest_hips_z = float(db.rigr["pz"][2])
    for name, path in files.items():
        if name not in db.clip_names:
            continue
        baked += 1
        ci = db.clip_names.index(name)
        if not flags[ci] & CF_RTZ_PRESENT:
            errs.append(f"{name} not RTZ_PRESENT")
            continue
        c = json.load(open(path, encoding="utf-8"))
        z = np.asarray(c["root_z"], np.float64)
        a, N = int(db.clip[ci]["first"]), int(db.clip[ci]["n"])
        if len(z) != N:
            errs.append(f"{name} length")
            continue
        vz = np.empty(N)
        vz[1:-1] = (z[2:] - z[:-2]) * 30.0 / 2.0
        vz[0] = (z[1] - z[0]) * 30.0
        vz[-1] = (z[-1] - z[-2]) * 30.0
        if np.any(db.rtz["z"][a:a + N] != z.astype(np.float32)):
            errs.append(f"{name} root_z != RTZ_ z")
        if np.any(np.abs(db.rtz["vz"][a:a + N] - vz) > 1e-6 * np.maximum(1.0, np.abs(vz))):
            errs.append(f"{name} vz")
        fr = np.asarray([x["bones"] for x in c["frames"]], np.float64)
        qn = fr[:, :, 3:7] / np.linalg.norm(fr[:, :, 3:7], axis=-1, keepdims=True)
        if np.any(db.pose[a:a + N, :, :3] != fr[:, :, :3].astype(np.float32)) \
                or np.abs(db.pose[a:a + N, :, 3:7] - qn).max() > 1e-6:
            errs.append(f"{name} POSE is not the RTZ clip")
        zmax = max(zmax, float(np.abs(z).max()))
        hips_dev = max(hips_dev, float(np.abs(db.pose[a:a + N, 2, 2] - rest_hips_z).max()))
    zero_rows = all(not np.any(db.rtz["z"][int(db.clip[i]["first"]):int(db.clip[i]["first"]) + int(db.clip[i]["n"])])
                    and not np.any(db.rtz["vz"][int(db.clip[i]["first"]):int(db.clip[i]["first"]) + int(db.clip[i]["n"])])
                    for i in none + unknown)
    if not zero_rows:
        errs.append("non-zero RTZ_ rows of a clip without root_z")
    if len(present) != baked:
        errs.append(f"RTZ_PRESENT clips {len(present)} != RTZ files baked {baked}")
    pend = [d for d in range(len(db.dbas)) if db.dbas[d]["flags"] & DF_RTZ_PENDING]
    if unknown == [] and pend:
        errs.append("RTZ_PENDING without RTZ_UNKNOWN clips")
    not_baked = sorted(set(files) - set(db.clip_names))
    R.check("rtz", not errs, f"RTZ files {len(files)}, baked {baked} (RTZ_ z == root_z bit-exact, vz central difference, "
                             f"POSE == RTZ clip), not baked (no database entry) {len(not_baked)}; clips RTZ_PRESENT "
                             f"{len(present)}, RTZ_UNKNOWN {len(unknown)}, RTZ_NONE {len(none)} (rows 0: {zero_rows}); "
                             f"databases RTZ_PENDING {len(pend)}; max |root_z| {zmax:.2f} m; max |Hips Z - rest| of RTZ "
                             f"clips {hips_dev:.3f} m (info); errors {len(errs)} {errs[:4]}")
    return not_baked


# ============================================================================== rig identity

def rig_json(path, nb):
    rc = json.load(open(path, encoding="utf-8"))["Data"]["RootChunk"]
    names = [n["$value"] if isinstance(n, dict) else str(n) for n in rc["boneNames"]][:nb]
    parents = [int(p) for p in rc["boneParentIndexes"]][:nb]
    rest = np.array([[t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"], t["Rotation"]["i"],
                      t["Rotation"]["j"], t["Rotation"]["k"], t["Rotation"]["r"]] for t in rc["boneTransforms"][:nb]],
                    np.float64).astype(np.float32)
    return names, parents, rest


def topo_hash(names, parents):
    return fnv1a64(b"".join(n.encode("utf-8") + b"\x00" + struct.pack("<i", p) for n, p in zip(names, parents)))


def identity_hash(names, parents, rest32):
    buf = bytearray()
    for n, p, t in zip(names, parents, rest32):
        q = [int(math.floor(float(np.float32(x)) * 1000.0 + 0.5)) for x in t[:3]]
        buf += n.encode("utf-8") + b"\x00" + struct.pack("<iiii", p, *q)
    return fnv1a64(bytes(buf))


def loader_accepts(db, live_names, live_parents, live_rest):
    """What a fail-closed v2 loader does with the live rig (FORMAT.txt v2 'Rig identity')."""
    if identity_hash(db.names, db.parents, np.stack([db.rigr[k] for k in ("px", "py", "pz")], -1)) != db.h["rig_hash"]:
        return False
    return identity_hash(live_names, live_parents, live_rest) == db.h["rig_hash"]


def test_rig(R, db, rig_path, other_rig_path):
    nb = db.h["num_bones"]
    rec = np.stack([db.rigr[k] for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")], -1)
    h_file = identity_hash(db.names, db.parents, rec)
    th = topo_hash(db.names, db.parents)
    n1, p1, r1 = rig_json(rig_path, nb)
    h_rig = identity_hash(n1, p1, r1)
    ok = h_file == db.h["rig_hash"] == h_rig and n1 == db.names and p1 == db.parents and np.array_equal(r1, rec) \
        and th == TOPOLOGY_HASH_0_72 and loader_accepts(db, n1, p1, r1)
    det = (f"header 0x{db.h['rig_hash']:016x}, from BONE+RIGR 0x{h_file:016x}, from {os.path.basename(rig_path)} "
           f"0x{h_rig:016x}; RIGR == rig json {np.array_equal(r1, rec)}; topology 0x{th:016x}")
    if other_rig_path and os.path.exists(other_rig_path):
        n2, p2, r2 = rig_json(other_rig_path, nb)
        h2 = identity_hash(n2, p2, r2)
        refused = not loader_accepts(db, n2, p2, r2)
        same_topo = topo_hash(n2, p2) == th
        ok = ok and refused and h2 != db.h["rig_hash"]
        det += (f"; other rig {os.path.basename(other_rig_path)} identity 0x{h2:016x} (same topology {same_topo}) "
                f"-> refused {refused}")
    R.check("rig_identity", ok, det)


# ============================================================================== determinism / v1

def run(cmd, log):
    print("  $ " + " ".join(cmd), flush=True)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if log:
        with open(log, "a", encoding="utf-8") as f:
            f.write(p.stdout + p.stderr)
    return p.returncode, p.stdout + p.stderr


def test_determinism(R, db, summary, tmp_dir, log):
    ba = summary.get("bake_args")
    if not ba:
        R.add("determinism", "not run", "summary has no bake_args")
        return
    os.makedirs(tmp_dir, exist_ok=True)
    out = os.path.join(tmp_dir, os.path.basename(db.path))
    cmd = [sys.executable, os.path.join(HERE, "mmbake_base.py"), "--format", "v2", "--clips",
           ";".join(os.path.join(ROOT, d) for d in ba["clips"].split(";") if d), "--rig", os.path.join(ROOT, ba["rig"]),
           "--bonemap", os.path.join(ROOT, ba["bonemap"]), "--motion-src", ba["motion_src"], "--out", out]
    if ba.get("rtz_clips"):
        cmd += ["--rtz-clips", ";".join(os.path.join(ROOT, d) for d in ba["rtz_clips"].split(";") if d)]
    if ba.get("sourcepack_only"):
        cmd += ["--sourcepack-only"]
    rc, _ = run(cmd, log)
    if rc != 0:
        R.check("determinism", False, f"second bake failed (exit {rc})")
        return
    b2 = open(out, "rb").read()
    same = b2 == db.b
    base = os.path.splitext(db.path)[0]
    s_same = open(base + ".summary.json", "rb").read() == open(os.path.splitext(out)[0] + ".summary.json", "rb").read()
    c1, c2 = base + ".chooser.json", os.path.splitext(out)[0] + ".chooser.json"
    c_same = os.path.exists(c1) and open(c1, "rb").read() == open(c2, "rb").read()
    R.check("determinism", same and s_same and c_same,
            f"second bake {len(b2)} bytes sha256 {sha256(b2)[:16]}..: database identical {same}, summary identical "
            f"{s_same}, chooser identical {c_same}")
    for f in os.listdir(tmp_dir):
        os.remove(os.path.join(tmp_dir, f))
    os.rmdir(tmp_dir)


def test_v1(R, tmp_dir, log):
    exp = json.load(open(os.path.splitext(V1_DENSE_FULL)[0] + ".summary.json", encoding="utf-8"))["sha256"]
    have = sha256(open(V1_DENSE_FULL, "rb").read())
    os.makedirs(tmp_dir, exist_ok=True)
    out = os.path.join(tmp_dir, "v1_dense_full.utppmm")
    rc, _ = run([sys.executable, os.path.join(HERE, "mmbake_base.py"), "--clips",
                 os.path.join(ROOT, "Animations", "retarget", "out"), "--out", out, "--no-summary"], log)
    got = sha256(open(out, "rb").read()) if rc == 0 else None
    R.check("v1_unchanged", rc == 0 and got == exp == have,
            f"v1 rebake of dense_full sha256 {got}, expected {exp} (file on disk {have[:16]}..)")
    for f in os.listdir(tmp_dir):
        os.remove(os.path.join(tmp_dir, f))
    os.rmdir(tmp_dir)


# ============================================================================== main

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", default=os.path.join(ROOT, "Data", "utpp_motion_woman.utppmm"))
    ap.add_argument("--format-txt", default=os.path.join(HERE, "FORMAT.txt"))
    ap.add_argument("--raw-dump", default=None, help="raw feature dump of the bake (mmbake_base.py --raw-dump)")
    ap.add_argument("--data-file", default=os.path.join(ROOT, "Research", "inventory", "mmbake-sourcepack.json"))
    ap.add_argument("--sourcepack-dir", default=os.path.join(ROOT, "Extracted", "sourcepack", "sourcepack"))
    ap.add_argument("--other-rig", default=None, help="rig json that must be refused (default: the other player rig)")
    ap.add_argument("--queries", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--traj-rows", type=int, default=100000, help="max rows per database for the trajectory check")
    ap.add_argument("--block-queries", type=int, default=300, help="max BLOCK-row queries per database")
    ap.add_argument("--determinism", action="store_true", help="rebake with the summary's bake_args and compare bytes")
    ap.add_argument("--v1-rebake", action="store_true", help="rebake v1 dense_full and compare with its sha256")
    ap.add_argument("--tmp", default=os.path.join(ROOT, "Build", "work", "mmbake", "_test_v2_tmp"))
    ap.add_argument("--log", default=None)
    a = ap.parse_args()
    R = Results()
    db = V2(a.db)
    print(f"database {a.db}: {len(db.b)} bytes sha256 {sha256(db.b)}", flush=True)
    sp = os.path.splitext(a.db)[0] + ".summary.json"
    summary = json.load(open(sp, encoding="utf-8")) if os.path.exists(sp) else {}
    test_format_doc(R, db, a.format_txt)
    if not test_integrity(R, db):
        print("integrity failed: content checks skipped")
    else:
        rig_name = summary.get("rig") or "player_woman_skeleton"
        rig_path = os.path.join(ROOT, summary["rig_identity"]["rig_json"]) if summary.get("rig_identity") else RIGS[rig_name]
        other = a.other_rig or next((p for n, p in RIGS.items() if n != rig_name), None)
        test_rig(R, db, rig_path, other)
        if summary:
            R.check("summary", summary.get("sha256") == sha256(db.b) and summary.get("num_frames") == db.h["num_frames"]
                    and summary.get("num_databases") == db.h["num_databases"],
                    f"summary sha256 / frames / databases match the file")
        bonemap = os.path.join(ROOT, (summary.get("bake_args") or {}).get("bonemap")
                               or "Source/tools/retarget/bonemap_source_to_game_woman.json")
        test_rtz(R, db, (summary.get("bake_args") or {}).get("rtz_clips"))
        test_normalization(R, db, a.raw_dump)
        test_trajectory(R, db, a.traj_rows)
        test_search(R, db, a.queries, a.seed)
        test_block(R, db, a.sourcepack_dir, a.data_file, a.block_queries)
        test_curve_lag(R, db, a.sourcepack_dir, a.data_file, bonemap)
        R.check("size", len(db.b) <= 220 * 1024 * 1024, f"file {len(db.b) / 2**20:.1f} MiB (budget 220 MB)")
        if a.determinism:
            test_determinism(R, db, summary, a.tmp, a.log)
        else:
            R.add("determinism", "not run", "use --determinism")
    if a.v1_rebake:
        test_v1(R, a.tmp + "_v1", a.log)
    else:
        R.add("v1_unchanged", "not run", "use --v1-rebake")
    n_p = sum(1 for i in R.items if i[1] == "passed")
    n_f = len(R.failed())
    n_n = sum(1 for i in R.items if i[1] == "not run")
    print(f"SUMMARY passed {n_p} failed {n_f} not run {n_n}")
    if a.log:
        with open(a.log, "a", encoding="utf-8") as f:
            for name, st, det in R.items:
                f.write(f"[{st.upper():7s}] {name}: {det}\n")
            f.write(f"SUMMARY passed {n_p} failed {n_f} not run {n_n}\n")
    return 1 if n_f else 0


if __name__ == "__main__":
    sys.exit(main())
