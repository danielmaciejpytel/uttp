#!/usr/bin/env python3
"""Tests for mmbake (Phase 3.3): determinism, counts vs inputs, independent feature recomputation.

The .utppmm is parsed here with a separate minimal struct-based reader (not mmbake's writer code), and a few
feature values are recomputed from the raw clip JSON with scalar pure-Python math (own quaternion / FK code,
no numpy, no mmbake helpers) to cross-check the vectorised baker.
"""
import argparse
import glob
import json
import math
import os
import struct
import subprocess
import sys
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DB = os.path.join(ROOT, "Data", "utpp_base_dense_subset.utppmm")
CLIPS = os.path.join(ROOT, "Animations", "retarget", "out")
META = os.path.join(ROOT, "Animations", "sourcepack", "SourcePack_export", "meta")
TMP = os.path.join(ROOT, "Build", "work", "mmbake")
BAKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mmbake_base.py")
# Databases the current meta lists for clips of a delivered file that has no tag for them (added to the meta after that file
# was baked), per file name: {database: clips}. Any other difference between the meta and a file's tags fails.
META_ADDED_AFTER_BAKE = {
    "utpp_base_dense_subset.utppmm": {
        "PSD_Dense_Stand_Walk_SpinTransition": {f"M_Neutral_Walk_Reface_Start_{d}_{s}_{a}"
                                                for d in "BF" for s in "LR" for a in ("090", "180")}},
}

fails = 0
passes = 0


def check(name, ok, detail=""):
    global fails, passes
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""))
    if ok:
        passes += 1
    else:
        fails += 1


# ---------------------------------------------------------------- independent reader

def read_db(path):
    b = open(path, "rb").read()
    magic, ver, hsz = struct.unpack_from("<8sII", b, 0)
    fsize, rh, flags = struct.unpack_from("<QQI", b, 16)
    keys = ["frames", "bones", "features", "clips", "ranges", "groups", "tags", "sm", "lr", "nsm", "nlr", "fps",
            "nsec", "table", "pcrc"]
    vals = struct.unpack_from("<IIIIIIIIIIIfIII", b, 36)
    h = dict(zip(keys, vals))
    h.update(magic=magic, version=ver, header_size=hsz, file_size=fsize, rig_hash=rh, flags=flags,
             hcrc=struct.unpack_from("<I", b, 96)[0])
    secs = {}
    for i in range(h["nsec"]):
        sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, h["table"] + 32 * i)
        secs[struct.pack("<I", sid).decode()] = (cnt, esz, off, size)
    return b, h, secs


def sec_floats(b, secs, sid):
    cnt, esz, off, size = secs[sid]
    return struct.unpack_from(f"<{size // 4}f", b, off)


def cstr(b, secs, o):
    off = secs["STRS"][2] + o
    return b[off:b.index(b"\x00", off)].decode()


# ---------------------------------------------------------------- scalar math (independent)

def qm(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qv(q, v):
    p = qm(qm(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return p[:3]


def fk_bone(frame, parents, bone):
    chain = []
    b = bone
    while b >= 0:
        chain.append(b)
        b = parents[b]
    pos, rot = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    for b in reversed(chain):
        lp = frame[b][0:3]
        lq = frame[b][3:7]
        n = math.sqrt(sum(x * x for x in lq))
        lq = tuple(x / n for x in lq)
        d = qv(rot, lp)
        pos = (pos[0] + d[0], pos[1] + d[1], pos[2] + d[2])
        rot = qm(rot, lq)
    return pos


def traj(frame):
    p = frame[1]
    return p[0], p[1], 2.0 * math.atan2(p[5], p[6])


def to_local(yaw, dx, dy):
    c, s = math.cos(-yaw), math.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy


def world(frame, parents, bone, pose_frame=None):
    x, y, yaw = traj(frame)
    m = fk_bone(pose_frame if pose_frame is not None else frame, parents, bone)
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * m[0] - s * m[1], y + s * m[0] + c * m[1], m[2])


def meta_path(name):
    for d in [x for x in META.split(";") if x]:
        p = os.path.join(d, name + ".json")
        if os.path.exists(p):
            return p
    raise FileNotFoundError(name)


def main():
    global DB, CLIPS, META
    ap = argparse.ArgumentParser(description="mmbake tests; defaults = Neutral subset (Data/utpp_base_dense_subset.utppmm)")
    ap.add_argument("--db", default=DB)
    ap.add_argument("--clips", default=CLIPS, help="clip dir(s), ';'-separated (as mmbake --clips)")
    ap.add_argument("--meta", default=META, help="meta dir(s), ';'-separated (as mmbake --meta)")
    a = ap.parse_args()
    DB, CLIPS, META = a.db, a.clips, a.meta
    bake_args = [] if (a.clips, a.meta) == (ap.get_default("clips"), ap.get_default("meta")) else ["--clips", CLIPS, "--meta", META]
    os.makedirs(TMP, exist_ok=True)
    # the clip set of the database under test is the one its file lists (the clip folder holds more clips than this subset):
    # the fresh bakes take exactly those clips (mmbake --clip-names-from)
    b, h, secs = read_db(DB)
    db_names = [cstr(b, secs, struct.unpack_from("<I", b, secs["CLIP"][2] + 32 * i)[0]) for i in range(h["clips"])]
    names_json = os.path.join(TMP, "db_clip_names.json")
    with open(names_json, "w", encoding="utf-8") as f:
        json.dump({"clips": [{"name": nm} for nm in db_names]}, f)
    bake_args += ["--clip-names-from", names_json]
    # ---- determinism: two fresh bakes + the delivered file
    outs = []
    for k in range(2):
        o = os.path.join(TMP, f"determinism_{k}.utppmm")
        r = subprocess.run([sys.executable, BAKER, "--out", o, "--summary", o + ".summary.json"] + bake_args,
                           capture_output=True, text=True)
        check(f"bake run {k} exit 0", r.returncode == 0, r.stderr.strip()[-300:])
        outs.append(open(o, "rb").read())
    delivered = open(DB, "rb").read()
    check("determinism: bake twice -> identical bytes", outs[0] == outs[1], f"{len(outs[0])} bytes")
    # delivered vs fresh bake: the data sections must be byte-identical. The name / tag sections may differ only by databases the
    # current meta lists and the delivered file has no tag for (a database added to the meta after the bake); every tag the
    # delivered file has must be set on exactly the same clips.
    fb, fh, fsecs = read_db(os.path.join(TMP, "determinism_0.utppmm"))
    name_secs = {"STRS", "TAGS", "CLIP", "GRUP"}
    data_diff = sorted(k for k in set(secs) | set(fsecs) if k not in name_secs and
                       (k not in secs or k not in fsecs or b[secs[k][2]:secs[k][2] + secs[k][3]] != fb[fsecs[k][2]:fsecs[k][2] + fsecs[k][3]]))
    tags_d = [cstr(b, secs, struct.unpack_from("<I", b, secs["TAGS"][2] + 4 * i)[0]) for i in range(h["tags"])]
    tags_f = [cstr(fb, fsecs, struct.unpack_from("<I", fb, fsecs["TAGS"][2] + 4 * i)[0]) for i in range(fh["tags"])]
    allowed = META_ADDED_AFTER_BAKE.get(os.path.basename(DB), {})
    rec_bad, tag_bad, extra_bad, newer = 0, 0, [], set()
    for i in range(min(h["clips"], fh["clips"])):
        rd_ = struct.unpack_from("<IIIIIIQ", b, secs["CLIP"][2] + 32 * i)
        rf_ = struct.unpack_from("<IIIIIIQ", fb, fsecs["CLIP"][2] + 32 * i)
        nm_ = cstr(b, secs, rd_[0])
        if nm_ != cstr(fb, fsecs, rf_[0]) or rd_[1:6] != rf_[1:6]:
            rec_bad += 1
        got_d = {t for j, t in enumerate(tags_d) if rd_[6] >> j & 1}
        got_f = {t for j, t in enumerate(tags_f) if rf_[6] >> j & 1}
        if got_f & set(tags_d) != got_d:
            tag_bad += 1
        for t in got_f - set(tags_d):
            newer.add(t)
            if nm_ not in allowed.get(t, ()):
                extra_bad.append(f"{nm_}:{t}")
    same = outs[0] == delivered
    check("determinism: delivered Data file == fresh bake (byte-identical, or identical data sections + clip records + every "
          "delivered tag, differing only by the listed databases added to the meta after the bake)",
          same or (not data_diff and h["clips"] == fh["clips"] and h["frames"] == fh["frames"] and rec_bad == 0 and tag_bad == 0
                   and not extra_bad),
          "byte-identical" if same else f"data sections differing {data_diff or 'none'}; clip records differing {rec_bad}; "
          f"clips with a changed delivered tag {tag_bad}; databases only in the current meta {sorted(newer) or 'none'}; "
          f"not listed as added after the bake {extra_bad[:5] or 'none'}")
    s0 = open(os.path.join(TMP, "determinism_0.utppmm.summary.json"), "rb").read().replace(b"determinism_0", b"X")
    s1 = open(os.path.join(TMP, "determinism_1.utppmm.summary.json"), "rb").read().replace(b"determinism_1", b"X")
    check("determinism: summary JSON identical", s0 == s1)

    # ---- header / CRC via independent reader
    check("magic/version/header size", h["magic"] == b"UTPPMMDB" and h["version"] == 1 and h["header_size"] == 128)
    check("file_size field", h["file_size"] == len(b))
    check("payload CRC32 (zlib)", zlib.crc32(b[128:]) & 0xFFFFFFFF == h["pcrc"])
    hb = bytearray(b[:128])
    hb[96:100] = b"\x00\x00\x00\x00"
    check("header CRC32 (zlib)", zlib.crc32(bytes(hb)) & 0xFFFFFFFF == h["hcrc"])
    check("sections 16-byte aligned", all(v[2] % 16 == 0 for v in secs.values()), ",".join(secs))

    # ---- rig hash from rig.json (independent FNV implementation)
    rig = json.load(open(os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json"), encoding="utf-8"))

    def find(o, k):
        if isinstance(o, dict):
            if k in o:
                return o[k]
            for v in o.values():
                r = find(v, k)
                if r is not None:
                    return r
        if isinstance(o, list):
            for v in o:
                r = find(v, k)
                if r is not None:
                    return r
        return None
    names = [n["$value"] for n in find(rig, "boneNames")][:73]
    parents = [int(p) for p in find(rig, "boneParentIndexes")][:73]
    hval = 0xCBF29CE484222325
    for n, p in zip(names, parents):
        for byte in n.encode() + b"\x00" + struct.pack("<i", p):
            hval = ((hval ^ byte) * 0x100000001B3) % (1 << 64)
    check("rig hash == FNV-1a64(rig.json bones 0..72)", hval == h["rig_hash"], f"0x{hval:016x}")

    # ---- counts vs inputs
    # the inputs of the database = the clip files of the names it lists (each must be in the clip folder)
    want_names = set(db_names)
    files = sorted(f for d in CLIPS.split(";") if d for f in glob.glob(os.path.join(d, "*.utppclip.json"))
                   if os.path.basename(f)[:-len(".utppclip.json")] in want_names)
    clips = {}
    for f in files:
        c = json.load(open(f, encoding="utf-8"))
        clips[c["source_clip"]] = c
    total = sum(len(c["frames"]) for c in clips.values())
    check("every clip the database lists is in the clip folder", want_names <= set(clips),
          f"{len(want_names & set(clips))} of {len(want_names)}; missing {sorted(want_names - set(clips))[:5]}")
    check("clip count", h["clips"] == len(clips), f"{h['clips']} vs {len(clips)}")
    check("frame count", h["frames"] == total, f"{h['frames']} vs {total}")
    check("bone count 73 / feature count 27", h["bones"] == 73 and h["features"] == 27)
    check("POSE section size", secs["POSE"][3] == total * 73 * 7 * 4)
    check("SCAL omitted (all scales 1)", "SCAL" not in secs and (h["flags"] & 1) == 0)
    crec = []
    for i in range(h["clips"]):
        no, first, n, fl, per, ns, mask = struct.unpack_from("<IIIIIIQ", b, secs["CLIP"][2] + 32 * i)
        crec.append((cstr(b, secs, no), first, n, fl, per, ns, mask))
    tags = [cstr(b, secs, struct.unpack_from("<I", b, secs["TAGS"][2] + 4 * i)[0]) for i in range(h["tags"])]
    ok_frames = all(clips[nm]["num_frames"] == n for nm, _, n, *_ in crec)
    check("per-clip frame counts match inputs", ok_frames)
    # every database the file has a tag for must match the meta exactly; a database the meta lists beyond the file's tag table
    # must be one of META_ADDED_AFTER_BAKE on exactly its clips
    ok_tags = True
    meta_only = set()
    allowed = META_ADDED_AFTER_BAKE.get(os.path.basename(DB), {})
    seen_added = {t: set() for t in allowed}
    for nm, _, _, _, _, _, mask in crec:
        want = set(json.load(open(meta_path(nm), encoding="utf-8"))["databases"])
        got = sorted(t for i, t in enumerate(tags) if mask >> i & 1)
        ok_tags &= sorted(want & set(tags)) == got
        for t in want - set(tags):
            meta_only.add(t)
            if t in seen_added:
                seen_added[t].add(nm)
            else:
                ok_tags = False
    ok_tags &= all(seen_added[t] == set(allowed[t]) for t in allowed)
    check("per-clip database tags match meta (the file's tag table exactly; beyond it only the databases listed as added after "
          "the bake, on exactly their clips)", ok_tags,
          ", ".join(tags) + (f"; databases only in the current meta: {', '.join(sorted(meta_only))}" if meta_only else ""))

    # ---- exclusion check: notify ranges independently
    F = h["features"]
    flags = struct.unpack_from(f"<{h['frames']}I", b, secs["FFLG"][2])
    bad = 0
    for nm, first, n, fl, *_ in crec:
        for nt in clips[nm]["notifies"]:
            if nt["notify_name"] != "PoseSearchExcludeFromDatabase":
                continue
            s, e = nt["trigger_time"], nt["trigger_time"] + nt["duration"]
            for i in range(n):
                if s + 1e-3 < i / 30.0 < e - 1e-3 and (flags[first + i] & 1 or not flags[first + i] & 2):
                    bad += 1
    check("frames inside PoseSearchExcludeFromDatabase are unsearchable", bad == 0, f"{bad} violations")
    nonloop_tail_bad = sum(1 for nm, first, n, fl, *_ in crec if not fl & 1
                           for i in range(max(0, n - 30), n) if flags[first + i] & 1)
    check("non-loop frames with horizon past clip end are unsearchable", nonloop_tail_bad == 0)

    # ---- independent feature recomputation
    off = sec_floats(b, secs, "FOFF")
    scl = sec_floats(b, secs, "FSCL")
    featoff = secs["FEAT"][2]

    def feat(g):
        v = struct.unpack_from(f"<{F}f", b, featoff + 4 * F * g)
        return [v[j] * scl[j] + off[j] for j in range(F)]

    cmap = {nm: (first, n, fl, per) for nm, first, n, fl, per, *_ in crec}
    LF, RF, HIPS = names.index("LeftFoot"), names.index("RightFoot"), names.index("Hips")
    maxerr = 0.0
    cases = [("M_Neutral_Walk_Loop_F", [0, 40, 100, 119]), ("M_Neutral_Walk_Start_F_Lfoot", [5, 30, 60]),
             ("M_Neutral_Run_Loop_LL", [2, 50, 70]), ("M_Neutral_Stand_Turn_090_L", [3, 20])]
    if not all(nm in clips for nm, _ in cases):  # other clip sets: two loops + two non-loops (or the first clips)
        loops = [nm for nm, _, n, fl, *_ in crec if fl & 1 and n > 12]
        other = [nm for nm, _, n, fl, *_ in crec if not fl & 1 and n > 12]
        pick = (loops[:2] + other[:2]) or [nm for nm, _, n, *_ in crec if n > 12][:4]
        cases = [(nm, sorted({0, 1, cmap[nm][1] // 2, cmap[nm][1] - 2})) for nm in pick]
        print("feature cases (auto): " + ", ".join(nm for nm, _ in cases))
    for nm, idxs in cases:
        c = clips[nm]
        fr = [x["bones"] for x in c["frames"]]
        first, n, fl, per = cmap[nm]
        loop = bool(fl & 1)
        for i in idxs:
            got = feat(first + i)
            x, y, yaw = traj(fr[i])
            exp = list(fk_bone(fr[i], parents, LF)) + list(fk_bone(fr[i], parents, RF))
            if 1 <= i <= n - 2:
                for bone in (LF, RF, HIPS):
                    # loop semantics: frame n-1 duplicates frame 0 -> its body pose is taken from frame 0 (period n-1)
                    pn = fr[(i + 1) % per] if loop else None
                    a, bb = world(fr[i - 1], parents, bone), world(fr[i + 1], parents, bone, pn)
                    v = [(bb[k] - a[k]) * 15.0 for k in range(3)]
                    lx, ly = to_local(yaw, v[0], v[1])
                    exp += [lx, ly, v[2]]
            else:
                exp += got[6:15]  # end frames use loop wrap / extrapolation; not recomputed here
            for hz in (10, 20, 30):
                j = i + hz
                if loop:
                    q, r = divmod(j, per)
                    tx, ty, tyaw = traj(fr[r])
                    # apply loop offset q times: D = T[n-1] * T[0]^-1 (T[0] treated generally)
                    for _ in range(q):
                        x0, y0, yw0 = traj(fr[0])
                        x1, y1, yw1 = traj(fr[n - 1])
                        lx0, ly0 = to_local(yw0, tx - x0, ty - y0)
                        c1, s1 = math.cos(yw1), math.sin(yw1)
                        tx, ty, tyaw = x1 + c1 * lx0 - s1 * ly0, y1 + s1 * lx0 + c1 * ly0, tyaw - yw0 + yw1
                else:
                    tx, ty, tyaw = traj(fr[min(j, n - 1)])
                exp_p = to_local(yaw, tx - x, ty - y)
                d = tyaw - yaw
                exp += [exp_p[0], exp_p[1]]
                exp += [-math.sin(d), math.cos(d)]
            # reorder trajectory part: file layout = 3 positions then 3 directions
            tp = exp[15:]
            exp = exp[:15] + [tp[0], tp[1], tp[4], tp[5], tp[8], tp[9]] + [tp[2], tp[3], tp[6], tp[7], tp[10], tp[11]]
            err = max(abs(a - e) for a, e in zip(got, exp))
            maxerr = max(maxerr, err)
            if err > 1e-3:
                print(f"  {nm}[{i}] err {err:.2e}\n   got {[round(v, 4) for v in got]}\n   exp {[round(v, 4) for v in exp]}")
    check("independent recomputation of features (foot pos/vel, hip vel, trajectory pos/dir)", maxerr < 1e-3,
          f"max abs err {maxerr:.2e} over {sum(len(i) for _, i in cases)} frames")

    # ---- root delta & pose spot check
    nm = cases[0][0]
    first, n, fl, per = cmap[nm]
    fr = [x["bones"] for x in clips[nm]["frames"]]
    rd = struct.unpack_from("<4f", b, secs["ROOT"][2] + 16 * (first + 10))
    x0, y0, w0 = traj(fr[10])
    x1, y1, w1 = traj(fr[11])
    lx, ly = to_local(w0, x1 - x0, y1 - y0)
    check("root delta frame 10->11", abs(rd[0] - lx) < 1e-5 and abs(rd[1] - ly) < 1e-5 and abs(rd[2] - (w1 - w0)) < 1e-5,
          f"({rd[0]:.5f},{rd[1]:.5f},{rd[2]:.5f})")
    pose = struct.unpack_from("<7f", b, secs["POSE"][2] + (first + 10) * 73 * 28 + 11 * 28)
    check("pose spot check LeftFoot frame 10", max(abs(a - e) for a, e in zip(pose, fr[10][11][:7])) < 1e-6)

    print(f"SUMMARY: {passes} passed, {fails} failed")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
