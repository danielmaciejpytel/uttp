#!/usr/bin/env python3
"""Reference dump of a .utppmm VERSION 2 database for the C++ reader test (tests/MotionDatabaseTest.cpp).

Independent reader written from Source/tools/mmbake/FORMAT.txt part "VERSION 2" (does not import mmbake_motion or
test_mmbake_motion). Writes a small text file with
  HDR <field> <value>                     header fields (rig_hash as 0x hex)
  CHK <name> <value>                      independent checks: header / payload CRC, rig identity from BONE + RIGR
  CNT <name> <value>                      counts derived from the sections (FFLG flag counts, RTZ clip flags,
                                          refused databases = Z_FROM_RTZ schema with an RTZ_UNKNOWN clip)
  FEAT <db> <local_row> <global_row> <frame> <dims> <float32 bits as 8 hex digits> x dims
  RTZ <frame> <clip> <root_z bits> <vz bits>
  END
for N random DFEA rows (seeded) and M RTZ_ rows (frames of RTZ_PRESENT clips first). Floats are written as raw
float32 bit patterns so the C++ comparison is exact.

The database and the dump are source-pack-derived data; keep them outside Source/.

Usage:
  python dump_utppmm_motion.py Data\\utpp_motion_woman.utppmm --out Build\\work\\mmbake\\utpp_motion_woman.dump.txt
"""
import argparse
import math
import os
import random
import struct
import sys
import zlib

HEADER_FMT = "<8sIIQQI" + "I" * 11 + "f" + "IIIIIII" + "QQ"
HEADER_NAMES = ["magic", "version", "header_size", "file_size", "rig_hash", "flags", "num_frames", "num_bones",
                "num_clips", "num_schemas", "num_channels", "num_norms", "num_databases", "num_entries", "num_rows",
                "num_curves", "num_events", "fps", "num_sections", "section_table_offset", "payload_crc32",
                "header_crc32", "bound_sm_size", "bound_lr_size", "max_dims", "dfea_floats", "dbox_floats"]
assert struct.calcsize(HEADER_FMT) == 128

CF_RTZ_UNKNOWN, CF_RTZ_PRESENT, CF_RTZ_NONE = 1 << 3, 1 << 5, 1 << 6
CH_Z_FROM_RTZ = 1 << 3
FF = {"selectable": 1 << 0, "excluded_notify": 1 << 1, "loop_duplicate": 1 << 3, "block_transition": 1 << 4,
      "branch_in": 1 << 5, "clip_start": 1 << 6, "clip_end": 1 << 7, "excluded_tail": 1 << 8, "indexed": 1 << 9}


def fnv1a64(data, h=0xCBF29CE484222325):
    for b in data:
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def bits(x):
    return "%08x" % struct.unpack("<I", struct.pack("<f", x))[0]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("db")
    ap.add_argument("--out", required=True)
    ap.add_argument("--features", type=int, default=20)
    ap.add_argument("--rtz", type=int, default=40)
    ap.add_argument("--seed", type=int, default=20261013)
    a = ap.parse_args(argv)

    with open(a.db, "rb") as f:
        b = f.read()
    h = dict(zip(HEADER_NAMES, struct.unpack_from(HEADER_FMT, b, 0)))
    if h["magic"] != b"UTPPMMDB" or h["version"] != 2:
        print("ERROR: not a .utppmm version 2 file", file=sys.stderr)
        return 2
    hz = bytearray(b[:128])
    hz[96:100] = b"\0\0\0\0"
    chk = {"header_crc_ok": int(zlib.crc32(bytes(hz)) == h["header_crc32"]),
           "payload_crc_ok": int(zlib.crc32(b[128:]) == h["payload_crc32"]),
           "file_size_ok": int(len(b) == h["file_size"])}
    secs = {}
    for i in range(h["num_sections"]):
        sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, 128 + 32 * i)
        secs[struct.pack("<I", sid).decode("ascii")] = (cnt, esz, off, size)

    def sec(name):
        return secs[name]

    strs_off = sec("STRS")[2]

    def string(off):
        if off == 0xFFFFFFFF:
            return ""
        e = b.index(b"\0", strs_off + off)
        return b[strs_off + off:e].decode("utf-8")

    nb, nf, nc = h["num_bones"], h["num_frames"], h["num_clips"]
    # rig identity from BONE + RIGR
    bone_off, rigr_off = sec("BONE")[2], sec("RIGR")[2]
    ident = 0xCBF29CE484222325
    for i in range(nb):
        name_off, parent, _, _ = struct.unpack_from("<IiII", b, bone_off + 16 * i)
        px, py, pz = struct.unpack_from("<3f", b, rigr_off + 28 * i)
        ident = fnv1a64(string(name_off).encode("utf-8") + b"\0" + struct.pack("<i", parent), ident)
        for t in (px, py, pz):
            ident = fnv1a64(struct.pack("<i", int(math.floor(float(t) * 1000.0 + 0.5))), ident)
    chk["rig_identity_from_bone_rigr"] = "0x%016x" % ident
    chk["rig_identity_ok"] = int(ident == h["rig_hash"])

    # clips, frame flags
    clip_off, clp2_off = sec("CLIP")[2], sec("CLP2")[2]
    clips = [struct.unpack_from("<IIIIIIQ", b, clip_off + 32 * c) for c in range(nc)]
    clip2 = [struct.unpack_from("<IIIIffII", b, clp2_off + 32 * c) for c in range(nc)]
    fflg = struct.unpack_from("<%dI" % nf, b, sec("FFLG")[2])
    cnt = {"frames_" + k: sum(1 for x in fflg if x & m) for k, m in FF.items()}
    cnt["clips_rtz_present"] = sum(1 for c in clips if c[3] & CF_RTZ_PRESENT)
    cnt["clips_rtz_unknown"] = sum(1 for c in clips if c[3] & CF_RTZ_UNKNOWN)
    cnt["clips_rtz_none"] = sum(1 for c in clips if c[3] & CF_RTZ_NONE)
    # databases
    nd = h["num_databases"]
    dbas_off, schm_off, schc_off = sec("DBAS")[2], sec("SCHM")[2], sec("SCHC")[2]
    dent_off, drow_off, dfea_off = sec("DENT")[2], sec("DROW")[2], sec("DFEA")[2]
    dbs = []
    for d in range(nd):
        rec = struct.unpack_from("<6I5f5IQQIIfI", b, dbas_off + 96 * d)
        dbs.append(dict(name=string(rec[0]), schema=rec[1], flags=rec[5], first_entry=rec[11], num_entries=rec[12],
                        first_row=rec[13], num_rows=rec[14], dims=rec[15], feat_offset=rec[16]))
    refused = 0
    pending = 0
    for d, db in enumerate(dbs):
        _, dims, fc, ncch, _, _, _, _ = struct.unpack_from("<4IfIfI", b, schm_off + 32 * db["schema"])
        zr = any(struct.unpack_from("<I", b, schc_off + 76 * k + 68)[0] & CH_Z_FROM_RTZ for k in range(fc, fc + ncch))
        unk = False
        for e in range(db["first_entry"], db["first_entry"] + db["num_entries"]):
            clip = struct.unpack_from("<I", b, dent_off + 32 * e + 4)[0]
            unk = unk or bool(clips[clip][3] & CF_RTZ_UNKNOWN)
        refused += int(zr and unk)
        pending += int(bool(db["flags"] & 1))
    cnt["databases_refused_rtz"] = refused
    cnt["databases_rtz_pending_flag"] = pending
    cnt["string_bytes"] = sec("STRS")[0]

    rng = random.Random(a.seed)
    lines = ["UTPPMM2DUMP 1", "# generator Source/tools/mmbake/dump_utppmm_motion.py (independent of mmbake_motion), seed %d"
             % a.seed, "# source-pack-derived data"]
    for k in HEADER_NAMES[1:]:
        v = h[k]
        if k == "rig_hash":
            lines.append("HDR %s 0x%016x" % (k, v))
        elif k == "fps":
            lines.append("HDR %s %s" % (k, bits(v)))
        else:
            lines.append("HDR %s %d" % (k, v))
    for k, v in chk.items():
        lines.append("CHK %s %s" % (k, v))
    for k, v in cnt.items():
        lines.append("CNT %s %d" % (k, v))
    # features: databases weighted by rows
    total = sum(db["num_rows"] for db in dbs)
    for _ in range(min(a.features, total)):
        r = rng.randrange(total)
        d = 0
        while r >= dbs[d]["num_rows"]:
            r -= dbs[d]["num_rows"]
            d += 1
        db = dbs[d]
        g = db["first_row"] + r
        frame = struct.unpack_from("<I", b, drow_off + 20 * g)[0]
        vals = struct.unpack_from("<%df" % db["dims"], b, dfea_off + 4 * (db["feat_offset"] + r * db["dims"]))
        lines.append("FEAT %d %d %d %d %d %s" % (d, r, g, frame, db["dims"], " ".join(bits(x) for x in vals)))
    # RTZ rows: frames of RTZ_PRESENT clips first, then random frames
    rtz_off = sec("RTZ_")[2]
    present = [c for c in range(nc) if clips[c][3] & CF_RTZ_PRESENT]
    frames = []
    if present:
        for _ in range(min(a.rtz * 3 // 4, sum(clips[c][2] for c in present))):
            c = rng.choice(present)
            frames.append(clips[c][1] + rng.randrange(clips[c][2]))
    while len(frames) < a.rtz:
        frames.append(rng.randrange(nf))
    clip_of = {}
    for c in range(nc):
        clip_of[c] = (clips[c][1], clips[c][1] + clips[c][2])
    for fr in frames:
        c = next(c for c, (s, e) in clip_of.items() if s <= fr < e)
        rz, vz = struct.unpack_from("<2f", b, rtz_off + 8 * fr)
        lines.append("RTZ %d %d %s %s" % (fr, c, bits(rz), bits(vz)))
    lines.append("END")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("dump: %d header fields, %d checks (%s), %d counts, %d FEAT rows, %d RTZ rows -> %s"
          % (len(HEADER_NAMES) - 1, len(chk), ", ".join("%s=%s" % kv for kv in chk.items()), len(cnt),
             min(a.features, total), len(frames), os.path.abspath(a.out)))
    return 0 if chk["header_crc_ok"] and chk["payload_crc_ok"] and chk["file_size_ok"] and chk["rig_identity_ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
