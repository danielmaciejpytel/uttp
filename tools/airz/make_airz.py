"""make_airz.py - root_z sidecar (.airz) generator / verifier for a UTPP motion matching database.

The .utppmm (format v1, also accepts v2 headers) is read with an own minimal reader that follows
Source/tools/mmbake/FORMAT.txt (it does not import mmbake). root_z comes from the source pack RTZ retarget outputs
(<rtz-dir>/<Clip>.utppclip.json, key "root_z", spec Source/tools/retarget/RTZ.txt).

Consistency rule per clip (RTZ.txt "Identity with the non-RTZ clip"):
    DB Hips_Z(f) - root_z(f) ~= RTZ Hips_Z(f)   within --tol (default 0.01 m) on ALL frames, equal frame counts.
A failing clip is skipped and logged. Clips without an RTZ file are not written (runtime keeps today's behaviour).

.airz layout (little-endian):
    char[8] magic "UTPPAIRZ", u32 version 1, u8[32] SHA-256 of the .utppmm file bytes, u32 clipCount,
    per clip (ascending clipIndex): u32 clipIndex, u16 nameLen, UTF-8 name[nameLen], u32 numFrames,
    f32 root_z[numFrames].

Usage:
    python -I make_airz.py [--db X.utppmm] [--rtz-dir DIR] [--out X.airz] [--log X.txt] [--tol 0.01] [--strict]
Exit codes: 0 ok (generate: skipped clips are logged, rc stays 0 unless --strict), 1 fail, 2 DB unreadable / no Hips bone.
    python -I make_airz.py --verify [--db X.utppmm] [--out X.airz] [--rtz-dir DIR] [--log X.txt]
"""
import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402
PROJECT = project_root()
DEF_DB = os.path.join(PROJECT, r"Data\utpp_base.utppmm")
DEF_RTZ = os.path.join(PROJECT, r"Animations\retarget\out_sourcepack_woman_rtz")
DEF_LIST = os.path.join(PROJECT, r"Research\inventory\sourcepack-rtz-clips.json")
DEF_LOGDIR = os.path.join(PROJECT, r"Research\logs")

AIRZ_MAGIC = b"UTPPAIRZ"
AIRZ_VERSION = 1

FAMILIES = [
    ("jump_start", re.compile(r"_Jump_(?:B|F|LL|RL)_Start")),
    ("jump_loop_fall", re.compile(r"_Jump_Loop_Fall$")),
    ("land", re.compile(r"_Jump_\w*?_Land_")),
    ("f_off_drop", re.compile(r"_Jump_F_Off_")),
    ("traversal", re.compile(r"_Traversal_")),
]


def family(name):
    for fam, rx in FAMILIES:
        if rx.search(name):
            return fam
    return "other"


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def sha256_file(path):
    with open(path, "rb") as fh:
        return sha256_bytes(fh.read())


class DbError(Exception):
    pass


class MiniDb:
    """Minimal read-only .utppmm reader: header, section table, STRS, BONE, CLIP, POSE (FORMAT.txt)."""

    def __init__(self, path):
        with open(path, "rb") as fh:
            self.data = fh.read()
        d = self.data
        self.path = path
        self.sha256 = hashlib.sha256(d).digest()
        if len(d) < 128 or d[0:8] != b"UTPPMMDB":
            raise DbError("bad magic")
        self.version, hsize = struct.unpack_from("<II", d, 8)
        if self.version not in (1, 2) or hsize != 128:
            raise DbError("unsupported version %d / header size %d" % (self.version, hsize))
        (file_size,) = struct.unpack_from("<Q", d, 16)
        if file_size != len(d):
            raise DbError("file_size %d != actual %d" % (file_size, len(d)))
        payload_crc, header_crc = struct.unpack_from("<II", d, 92)
        hdr = bytearray(d[0:128])
        hdr[96:100] = b"\0\0\0\0"
        if zlib.crc32(bytes(hdr)) & 0xFFFFFFFF != header_crc:
            raise DbError("header CRC mismatch")
        if zlib.crc32(d[128:]) & 0xFFFFFFFF != payload_crc:
            raise DbError("payload CRC mismatch")
        self.num_frames, self.num_bones = struct.unpack_from("<II", d, 36)
        if self.version == 1:
            (self.num_clips,) = struct.unpack_from("<I", d, 48)
        else:
            (self.num_clips,) = struct.unpack_from("<I", d, 44)
        (self.fps,) = struct.unpack_from("<f", d, 80)
        num_sections, sec_off = struct.unpack_from("<II", d, 84)
        if sec_off != 128 or num_sections > 64 or sec_off + 32 * num_sections > len(d):
            raise DbError("bad section table")
        self.secs = {}
        for i in range(num_sections):
            sid, cnt, esz, _r, off, size = struct.unpack_from("<IIIIQQ", d, sec_off + 32 * i)
            name = struct.pack("<I", sid).decode("latin-1")
            if size != cnt * esz or off + size > len(d) or name in self.secs:
                raise DbError("bad section %r" % name)
            self.secs[name] = (cnt, esz, off, size)
        for req in ("STRS", "BONE", "CLIP", "POSE"):
            if req not in self.secs:
                raise DbError("missing section " + req)
        _, _, soff, ssize = self.secs["STRS"]
        self.strs = d[soff:soff + ssize]
        cnt, esz, boff, _ = self.secs["BONE"]
        if cnt != self.num_bones or esz != 16:
            raise DbError("BONE size")
        self.bone_names = []
        for b in range(cnt):
            no, _parent, nlen, _z = struct.unpack_from("<IiII", d, boff + 16 * b)
            self.bone_names.append(self._str(no))
        cnt, esz, coff, _ = self.secs["CLIP"]
        if cnt != self.num_clips or esz != 32:
            raise DbError("CLIP size")
        self.clips = []  # (name, first_frame, num_frames)
        for c in range(cnt):
            no, ff, nf = struct.unpack_from("<III", d, coff + 32 * c)
            if ff + nf > self.num_frames:
                raise DbError("clip %d out of range" % c)
            self.clips.append((self._str(no), ff, nf))
        cnt, esz, poff, _ = self.secs["POSE"]
        if cnt != self.num_frames or esz != self.num_bones * 28:
            raise DbError("POSE size")
        self.pose_off, self.pose_esz = poff, esz

    def _str(self, off):
        end = self.strs.index(b"\0", off)
        return self.strs[off:end].decode("utf-8")

    def bone_index(self, name):
        try:
            return self.bone_names.index(name)
        except ValueError:
            raise DbError("bone %r not found in BONE section" % name) from None

    def bone_local_z(self, frame, bone):
        return struct.unpack_from("<f", self.data, self.pose_off + frame * self.pose_esz + bone * 28 + 8)[0]


def _is_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def check_clip(db, ci, hips, rtz_path, tol):
    """Return (ok, reason, max_dev, root_z_list)."""
    name, ff, nf = db.clips[ci]
    try:
        with open(rtz_path, "r", encoding="utf-8") as fh:
            j = json.load(fh)
    except (OSError, ValueError) as e:
        return False, "unreadable RTZ json: %s" % e, None, None
    if not isinstance(j, dict):
        return False, "RTZ json is not an object", None, None
    rz = j.get("root_z")
    frames = j.get("frames")
    if not isinstance(rz, list) or not isinstance(frames, list):
        return False, "RTZ json has no root_z/frames", None, None
    fps = j.get("fps")
    if fps is not None:
        if not _is_num(fps) or not math.isfinite(fps):
            return False, "RTZ fps %r is not a finite number" % (fps,), None, None
        if abs(float(fps) - db.fps) > 1e-6:
            return False, "fps %s != db %s" % (fps, db.fps), None, None
    names = j.get("bone_names") or []
    if not isinstance(names, list) or "Hips" not in names:
        return False, "RTZ json has no Hips bone", None, None
    jh = names.index("Hips")
    if len(rz) != nf or len(frames) != nf:
        return False, "frame count db=%d rtz root_z=%d frames=%d" % (nf, len(rz), len(frames)), None, None
    max_dev = 0.0
    worst = 0
    out = []
    for f in range(nf):
        z = rz[f]
        if not _is_num(z) or not math.isfinite(z):
            return False, "non-finite root_z at frame %d" % f, None, None
        try:
            rh = frames[f]["bones"][jh][2]
        except (KeyError, IndexError, TypeError) as e:
            return False, "malformed RTZ frame %d (frames[f]['bones'][Hips][2]): %s: %s" % (f, type(e).__name__, e), None, None
        if not _is_num(rh):
            return False, "malformed RTZ frame %d: Hips Z %r is not a number" % (f, rh), None, None
        dev = abs(db.bone_local_z(ff + f, hips) - z - rh)
        if not math.isfinite(dev):
            return False, "non-finite Hips at frame %d" % f, None, None
        if dev > max_dev:
            max_dev, worst = dev, f
        try:
            out.append(f32(float(z)))
        except OverflowError:
            return False, "root_z at frame %d out of float32 range" % f, None, None
    if max_dev > tol:
        return False, "Hips identity fails: max dev %.4f m at frame %d > tol %.3f" % (max_dev, worst, tol), max_dev, None
    return True, "ok", max_dev, out


def write_airz(path, db_sha, entries):
    parts = [AIRZ_MAGIC, struct.pack("<I", AIRZ_VERSION), db_sha, struct.pack("<I", len(entries))]
    for ci, name, rz in entries:
        nb = name.encode("utf-8")
        parts.append(struct.pack("<IH", ci, len(nb)))
        parts.append(nb)
        parts.append(struct.pack("<I", len(rz)))
        parts.append(struct.pack("<%df" % len(rz), *rz))
    blob = b"".join(parts)
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(blob)
    os.replace(tmp, path)
    return blob


def read_airz(path):
    with open(path, "rb") as fh:
        b = fh.read()
    if len(b) < 48 or b[0:8] != AIRZ_MAGIC:
        raise ValueError("bad magic")
    (ver,) = struct.unpack_from("<I", b, 8)
    if ver != AIRZ_VERSION:
        raise ValueError("unsupported version %d" % ver)
    db_sha = b[12:44]
    (count,) = struct.unpack_from("<I", b, 44)
    p = 48
    entries = []
    for _ in range(count):
        if p + 6 > len(b):
            raise ValueError("truncated clip header")
        ci, nlen = struct.unpack_from("<IH", b, p)
        p += 6
        if p + nlen + 4 > len(b):
            raise ValueError("truncated name")
        name = b[p:p + nlen].decode("utf-8")
        p += nlen
        (nf,) = struct.unpack_from("<I", b, p)
        p += 4
        if p + 4 * nf > len(b):
            raise ValueError("truncated root_z of %s" % name)
        rz = list(struct.unpack_from("<%df" % nf, b, p))
        p += 4 * nf
        entries.append((ci, name, rz))
    if p != len(b):
        raise ValueError("%d trailing bytes" % (len(b) - p))
    return db_sha, entries, b


def verify(args, log):
    errors = []
    db = MiniDb(args.db)
    log("db       %s  sha256=%s  version=%d clips=%d frames=%d" % (args.db, db.sha256.hex(), db.version, db.num_clips, db.num_frames))
    try:
        db_sha, entries, blob = read_airz(args.out)
    except (OSError, ValueError) as e:
        log("FAIL cannot read airz: %s" % e)
        return 1
    log("airz     %s  sha256=%s  clips=%d" % (args.out, sha256_bytes(blob), len(entries)))
    if db_sha != db.sha256:
        errors.append("db SHA-256 mismatch: airz %s vs db %s" % (db_sha.hex(), db.sha256.hex()))
    last = -1
    hips = db.bone_index("Hips")
    for ci, name, rz in entries:
        if ci <= last:
            errors.append("clip indices not strictly ascending at %d" % ci)
        last = ci
        if ci >= db.num_clips:
            errors.append("clipIndex %d out of range" % ci)
            continue
        dname, ff, nf = db.clips[ci]
        if dname != name:
            errors.append("clip %d name %r != db %r" % (ci, name, dname))
        if len(rz) != nf:
            errors.append("clip %d %s frames %d != db %d" % (ci, name, len(rz), nf))
        if not all(math.isfinite(z) for z in rz):
            errors.append("clip %d %s non-finite root_z" % (ci, name))
        if args.rtz_dir:
            rp = os.path.join(args.rtz_dir, name + ".utppclip.json")
            ok, why, dev, ref = check_clip(db, ci, hips, rp, args.tol)
            if not ok:
                errors.append("clip %d %s RTZ recheck: %s" % (ci, name, why))
            elif ref != rz:
                errors.append("clip %d %s root_z differs from RTZ json (float32)" % (ci, name))
    for e in errors:
        log("FAIL " + e)
    log("verify: %s (%d clips, %d errors)" % ("PASS" if not errors else "FAIL", len(entries), len(errors)))
    return 0 if not errors else 1


def generate(args, log):
    db = MiniDb(args.db)
    hips = db.bone_index("Hips")
    log("UTPP .airz generator (source pack derived data)")
    log("inputs:")
    log("  db       %s" % args.db)
    log("           sha256=%s format v%d clips=%d frames=%d bones=%d fps=%g Hips=bone %d"
        % (db.sha256.hex(), db.version, db.num_clips, db.num_frames, db.num_bones, db.fps, hips))
    log("  rtz dir  %s" % args.rtz_dir)
    if os.path.exists(args.clip_list):
        log("  list     %s  sha256=%s" % (args.clip_list, sha256_file(args.clip_list)))
    log("  tol      %.4f m (DB Hips_Z - root_z vs RTZ Hips_Z, all frames)" % args.tol)
    log("")
    entries = []
    stats = {}
    rtz_files = set(n[:-len(".utppclip.json")] for n in os.listdir(args.rtz_dir) if n.endswith(".utppclip.json"))
    used = set()
    log("per clip (db index, name, family, frames, result, max dev m, rtz json sha256):")
    for ci, (name, ff, nf) in enumerate(db.clips):
        fam = family(name)
        st = stats.setdefault(fam, {"db": 0, "rtz": 0, "written": 0, "skipped": 0})
        st["db"] += 1
        if name not in rtz_files:
            continue
        used.add(name)
        st["rtz"] += 1
        rp = os.path.join(args.rtz_dir, name + ".utppclip.json")
        ok, why, dev, rz = check_clip(db, ci, hips, rp, args.tol)
        devs = "-" if dev is None else "%.5f" % dev
        if ok:
            entries.append((ci, name, rz))
            st["written"] += 1
            log("  %4d %-50s %-14s %4d OK   %s  %s" % (ci, name, fam, nf, devs, sha256_file(rp)))
        else:
            st["skipped"] += 1
            log("  %4d %-50s %-14s %4d SKIP %s  %s  reason: %s" % (ci, name, fam, nf, devs, sha256_file(rp), why))
    unused = sorted(rtz_files - used)
    log("")
    log("RTZ files without a clip in the DB (%d): %s" % (len(unused), ", ".join(unused) if unused else "-"))
    log("")
    log("family coverage (db clips / with RTZ file / written / skipped):")
    for fam in [f for f, _ in FAMILIES] + ["other"]:
        if fam in stats:
            s = stats[fam]
            log("  %-14s %4d / %4d / %4d / %4d" % (fam, s["db"], s["rtz"], s["written"], s["skipped"]))
    tot_w = sum(s["written"] for s in stats.values())
    tot_s = sum(s["skipped"] for s in stats.values())
    log("totals: db clips %d, with RTZ %d, written %d, skipped %d, frames written %d"
        % (db.num_clips, tot_w + tot_s, tot_w, tot_s, sum(len(e[2]) for e in entries)))
    if args.strict and tot_s:
        log("STRICT: %d clip(s) failed the identity check -> nothing written, rc 1" % tot_s)
        return 1
    blob = write_airz(args.out, db.sha256, entries)
    log("output   %s  bytes=%d  sha256=%s" % (args.out, len(blob), sha256_bytes(blob)))
    if tot_s:
        log("NOTE: %d clip(s) skipped; rc stays 0 (use --strict to fail and refuse to write)" % tot_s)
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--db", default=DEF_DB)
    ap.add_argument("--rtz-dir", default=DEF_RTZ)
    ap.add_argument("--clip-list", default=DEF_LIST)
    ap.add_argument("--out", default=None, help="default: <db without .utppmm>.airz")
    ap.add_argument("--log", default=None, help="default: Research\\logs\\airz-<db stem>-gen.txt (verify: -verify.txt)")
    ap.add_argument("--tol", type=float, default=0.01)
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--strict", action="store_true",
                    help="generate: rc 1 and write nothing if any clip with an RTZ file is skipped")
    ap.add_argument("--no-rtz-recheck", action="store_true", help="verify: skip the per-clip RTZ json recheck")
    args = ap.parse_args(argv)
    stem = os.path.splitext(os.path.basename(args.db))[0]
    if stem.startswith("utpp_"):
        stem = stem[len("utpp_"):]
    if args.out is None:
        args.out = os.path.splitext(args.db)[0] + ".airz"
    if args.log is None:
        args.log = os.path.join(DEF_LOGDIR, "airz-%s-%s.txt" % (stem, "verify" if args.verify else "gen"))
    if args.verify and args.no_rtz_recheck:
        args.rtz_dir = None
    lines = []

    def log(s):
        print(s)
        lines.append(s)

    try:
        rc = verify(args, log) if args.verify else generate(args, log)
    except DbError as e:
        log("FAIL database: %s" % e)
        rc = 2
    except OSError as e:
        log("FAIL i/o: %s" % e)
        rc = 1
    if args.log != "-":
        with open(args.log, "w", encoding="utf-8", newline="\r\n") as fh:
            fh.write("\n".join(lines) + "\n")
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
