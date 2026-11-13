"""test_travbake_motion.py - independent check of the FORMAT_TRAVERSAL version 2 files written by travbake.py --format 2
(trav: Data/traversal/utpp_traversal_{woman,man}_motion.utpptrav = --variant base, the clips of
utpp_motion_<rig>.utppmm; utpp_traversal_{woman,man}_motion_knee.utpptrav = --variant knee, the clips of the staged
utpp_motion_<rig>_knee.utppmm). Every check below runs for both variants against their own clips and database.

Reads the files by the spec only (own reader: the version 1 grammar + POSEBONES + P lines) and checks:
  every version 1 part (M / SEC / W / R / C) with test_travbake.check_rig_file against the knee retarget clips
  (out_sourcepack_<rig>_knee / _rtz_knee), POSEBONES = the clip bone list (subset of the rig, no duplicates), one P line per
  montage frame with 7 floats per pose bone = the clip's local bone transform of that frame (quaternion normalised),
  negative controls (missing / short P line, missing POSEBONES, version 2 header on a version 1 file), determinism (two
  fresh bakes byte-identical to each other and to the files under test) and the no-overwrite rule (a format 2 bake into a
  folder that already has the output fails and leaves the file unchanged).

usage: python -I test_travbake_motion.py [--root <project>] [--no-rebake]
exit 0 = all checks passed; prints PASS / FAIL counts.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_travbake as tb  # noqa: E402  (shared spec reader pieces, independent FK, check counters)

BASE = {rig: tuple(v) for rig, v in tb.RIGS.items()}
KNEE = {rig: (v[0], v[1] + "_knee", v[2] + "_knee") for rig, v in tb.RIGS.items()}
VARIANTS = {"base": ("", BASE), "knee": ("_knee", KNEE)}  # suffix of the file / database name, clip dirs
# The shipped generation: the knee clips with the root-z clips of the fixed leg-twist retarget. The installed
# traversal files are baked from it (woman: Data/traversal/utpp_traversal_woman_motion.utpptrav; man: installed copy only).
# Rebuild: travbake.py --format 2 --variant knee --rig <rig> --rtz-dir Animations/retarget/out_sourcepack_<rig>_rtz_knee_twistfix
# --out-dir <dir>  (writes utpp_traversal_<rig>_motion_knee.utpptrav; woman = 473871a0..., man = c7620f82...).
SHIPPED = {rig: (v[0], v[1] + "_knee", v[2] + "_knee_twistfix") for rig, v in tb.RIGS.items()}
# clip generations a file of a variant may come from; the one whose clips reproduce the file's first montage is used
GENERATIONS = {"base": [("base", BASE), ("shipped", SHIPPED)], "knee": [("knee", KNEE)]}


def read_trav_format2(path):
    raw = open(path, "rb").read()
    raw.decode("ascii")
    if b"\r" in raw or not raw.endswith(b"\n"):
        raise ValueError("not LF text")
    lines = raw.decode("ascii").split("\n")[:-1]
    if not lines or lines[0] != "utpp-traversal 2":
        raise ValueError("header")
    if len(lines) < 3 or not lines[2].startswith("POSEBONES "):
        raise ValueError("POSEBONES")
    t = lines[2].split(" ")
    nb = int(t[1])
    bones = t[2:]
    if len(bones) != nb or nb < 1 or len(set(bones)) != nb or any(not b for b in bones):
        raise ValueError("POSEBONES list")
    # strip the P lines per montage block (they sit between C and E), then the rest is version 1 grammar
    v1, poses, cur, in_block = [lines[0].replace(" 2", " 1", 1), lines[1]], [], None, False
    for ln in lines[3:]:
        if ln.startswith("M "):
            cur = []
            poses.append(cur)
        if ln.startswith("P "):
            if cur is None:
                raise ValueError("P outside a montage")
            tk = ln.split(" ")
            if len(tk) != 1 + 7 * nb or ln != ln.strip() or "  " in ln:
                raise ValueError("P tokens %d" % len(tk))
            cur.append([tb.fl(x) for x in tk[1:]])
            continue
        v1.append(ln)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "v1.utpptrav")
        open(p, "wb").write(("\n".join(v1) + "\n").encode("ascii"))
        data = tb.read_trav(p)
    if len(poses) != len(data["montages"]):
        raise ValueError("pose blocks")
    for m, ps in zip(data["montages"], poses):
        if len(ps) != m["frames"]:
            raise ValueError("%s: %d P lines != %d frames" % (m["name"], len(ps), m["frames"]))
        m["pose"] = ps
    # P lines must come after C and right before E: re-scan the order
    state = None
    for ln in lines[3:]:
        k = ln.split(" ", 1)[0]
        if k == "C":
            state = "C"
        elif k == "P":
            if state not in ("C", "P"):
                raise ValueError("P line not after C")
            state = "P"
        elif k == "E":
            if state != "P":
                raise ValueError("E without P lines")
            state = None
    data["pose_bones"] = bones
    return data


def pick_generation(root, rig, data, gens):
    """first generation whose clips reproduce the P lines of the first montage (fallback: the first generation)"""
    m = data["montages"][0]
    for name, dirs in gens:
        _, cdir, rdir = dirs[rig]
        rp = os.path.join(root, rdir, m["seq"] + ".utppclip.json")
        cp = rp if os.path.isfile(rp) else os.path.join(root, cdir, m["seq"] + ".utppclip.json")
        if not os.path.isfile(cp):
            continue
        clip = json.load(open(cp, encoding="utf-8"))
        if clip["bone_names"] != data["pose_bones"]:
            continue
        worst = 0.0
        for k, row in enumerate(m["pose"]):
            for i, b in enumerate(clip["frames"][k]["bones"]):
                n = math.sqrt(sum(float(b[3 + j]) ** 2 for j in range(4)))
                worst = max(worst, max(abs(row[7 * i + j] - float(b[j])) for j in range(3)),
                            max(abs(row[7 * i + 3 + j] - float(b[3 + j]) / n) for j in range(4)))
        if worst <= 6e-7:
            return name, dirs
    return gens[0]


def check_poses(root, rig, data, dirs):
    rig_json, cdir, rdir = dirs[rig]
    rnames, _, _ = tb.rig_hash(os.path.join(root, rig_json))
    pb = data["pose_bones"]
    tb.check(all(b in rnames for b in pb), "%s pose bones in the rig" % rig)
    worst_p = worst_q = worst_n = 0.0
    frames = 0
    for m in data["montages"]:
        rp = os.path.join(root, rdir, m["seq"] + ".utppclip.json")
        clip = json.load(open(rp if os.path.isfile(rp) else os.path.join(root, cdir, m["seq"] + ".utppclip.json"),
                              encoding="utf-8"))
        tb.check(clip["bone_names"] == pb, "%s %s POSEBONES = clip bone list" % (rig, m["name"]))
        if clip["bone_names"] != pb:
            continue
        for k, row in enumerate(m["pose"]):
            src = clip["frames"][k]["bones"]  # segment starts at frame 0 (checked by check_rig_file: frames)
            for i in range(len(pb)):
                x = row[7 * i:7 * i + 7]
                b = src[i]
                n = math.sqrt(sum(float(b[3 + j]) ** 2 for j in range(4)))
                worst_p = max(worst_p, max(abs(x[j] - float(b[j])) for j in range(3)))
                worst_q = max(worst_q, max(abs(x[3 + j] - float(b[3 + j]) / n) for j in range(4)))
                worst_n = max(worst_n, abs(math.sqrt(sum(x[3 + j] ** 2 for j in range(4))) - 1.0))
            frames += 1
    tb.check(worst_p <= 6e-7 and worst_q <= 6e-7, "%s P lines = clip local pose (max |dp| %.2e, |dq| %.2e)" % (
        rig, worst_p, worst_q))
    tb.check(worst_n <= 5e-6, "%s P quaternions unit (max |1-|q|| %.2e)" % (rig, worst_n))
    print("%s: %d pose bones, %d pose frames, max |dp| %.2e |dq| %.2e |1-|q|| %.2e" % (
        rig, len(pb), frames, worst_p, worst_q, worst_n))
    return frames


def db_clip_poses(path):
    """Own minimal .utppmm v2 reader (header layout FORMAT.txt v2): bone names, {clip name: (first, frames)}, POSE rows."""
    import struct
    b = open(path, "rb").read()
    nf, nb, nc = struct.unpack_from("<3I", b, 36)
    nsec = struct.unpack_from("<I", b, 84)[0]
    secs = {}
    for i in range(nsec):
        sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, 128 + 32 * i)
        secs[struct.pack("<I", sid).decode("ascii")] = (cnt, esz, off, size)
    so = secs["STRS"][2]

    def s(off):
        return b[so + off:b.index(b"\0", so + off)].decode("utf-8")

    bones = [s(struct.unpack_from("<IiII", b, secs["BONE"][2] + 16 * i)[0]) for i in range(nb)]
    clips = {}
    for c in range(nc):
        name, first, n = struct.unpack_from("<III", b, secs["CLIP"][2] + 32 * c)
        clips[s(name)] = (first, n)
    cnt, esz, off, _ = secs["POSE"]
    if cnt != nf or esz != nb * 28:
        raise ValueError("POSE section shape")

    def pose(frame, bone):
        return struct.unpack_from("<7f", b, off + frame * esz + bone * 28)

    return bones, clips, pose


def check_vs_db(root, rig, data, suffix):
    """P lines of the montages whose sequence is a clip of the variant's database = that clip's POSE rows (float32)."""
    path = os.path.join(root, "Data/utpp_motion_%s%s.utppmm" % (rig, suffix))
    if not os.path.isfile(path):
        tb.check(False, "%s missing" % path)
        return
    bones, clips, pose = db_clip_poses(path)
    pb = data["pose_bones"]
    tb.check(all(x in pb for x in bones), "%s database bones all in POSEBONES" % rig)
    idx = [pb.index(x) for x in bones if x in pb]
    n_in, worst = 0, 0.0
    for m in data["montages"]:
        if m["seq"] not in clips:
            continue
        first, n = clips[m["seq"]]
        tb.check(n >= m["frames"], "%s %s db clip frames %d >= %d" % (rig, m["name"], n, m["frames"]))
        n_in += 1
        for k in range(min(n, m["frames"])):
            row = m["pose"][k]
            for bi, pi in enumerate(idx):
                d = pose(first + k, bi)
                x = row[7 * pi:7 * pi + 7]
                worst = max(worst, max(abs(x[j] - d[j]) for j in range(7)))
    tb.check(n_in >= 10 and worst <= 2e-6, "%s P lines = database POSE of the %d montages that are db clips (max %.2e)" % (
        rig, n_in, worst))
    print("%s: %d of %d montages are clips of %s, max |P - POSE| %.2e" % (
        rig, n_in, len(data["montages"]), os.path.basename(path), worst))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      "..", "..", "..")))
    ap.add_argument("--no-rebake", action="store_true")
    a = ap.parse_args()
    root = a.root
    table = json.load(open(os.path.join(root, "Data/traversal/traversal_table.json"), encoding="utf-8"))
    found = set()

    def walk(x):
        if isinstance(x, dict):
            for v in x.values():
                walk(v)
        elif isinstance(x, list):
            for v in x:
                walk(v)
        elif isinstance(x, str) and x.startswith("AM_") and "Traversal" in x:
            found.add(x)

    walk(table)
    tb.check(len(found) == 34, "traversal table montages %d != 34" % len(found))
    files = {}
    gens = {}  # (variant, rig) -> (generation name, clip dirs of that rig)
    active = []  # variants whose data exist; the optional knee variant is skipped cleanly when its files are absent
    for var, (suffix, _) in VARIANTS.items():
        if var != "base":
            missing = [os.path.join("Data/traversal", "utpp_traversal_%s_motion%s.utpptrav" % (rig, suffix))
                       for rig in ("woman", "man")] + ["Data/utpp_motion_%s%s.utppmm" % (rig, suffix)
                                                       for rig in ("woman", "man")]
            missing = [x for x in missing if not os.path.isfile(os.path.join(root, x))]
            if missing:
                print("SKIP variant %s: data absent (%s)" % (var, ", ".join(missing)))
                continue
        active.append(var)
        for rig in ("woman", "man"):
            p = os.path.join(root, "Data/traversal/utpp_traversal_%s_motion%s.utpptrav" % (rig, suffix))
            try:
                data = read_trav_format2(p)
            except Exception as e:  # fail closed
                tb.check(False, "%s: read failed: %s" % (p, e))
                continue
            files[(var, rig)] = (p, data)
            gname, dirs = pick_generation(root, rig, data, GENERATIONS[var])
            gens[(var, rig)] = (gname, dirs)
            tb.RIGS[rig] = dirs[rig]  # check_rig_file reads the picked generation's clips for the version 1 parts
            nwin = tb.check_rig_file(root, rig, data, found)
            nf = check_poses(root, rig, data, dirs)
            check_vs_db(root, rig, data, suffix)
            tb.check(len(data["montages"]) == 34 and nf == sum(m["frames"] for m in data["montages"]),
                     "%s %s: 34 montages with poses" % (var, rig))
            print("%s %s (%s clips): %d montages, %d warp windows" % (var, rig, gname, len(data["montages"]), nwin))
    # negative controls: the reader must reject corrupted version 2 files
    if ("base", "woman") in files:
        raw = open(files[("base", "woman")][0], "rb").read()
        lines = raw.split(b"\n")
        first_p = next(i for i, ln in enumerate(lines) if ln.startswith(b"P "))
        bads = [
            b"\n".join(lines[:first_p] + lines[first_p + 1:]),                       # one P line missing
            b"\n".join(lines[:first_p] + [lines[first_p].rsplit(b" ", 1)[0]] + lines[first_p + 1:]),  # short P line
            b"\n".join(lines[:2] + lines[3:]),                                        # no POSEBONES
            open(os.path.join(root, "Data/traversal/utpp_traversal_woman.utpptrav"), "rb").read().replace(
                b"utpp-traversal 1", b"utpp-traversal 2", 1),                         # v2 header on a v1 body
        ]
        for i, bad in enumerate(bads):
            with tempfile.TemporaryDirectory() as td:
                bp = os.path.join(td, "bad%d.utpptrav" % i)
                open(bp, "wb").write(bad)
                try:
                    read_trav_format2(bp)
                    tb.check(False, "corrupt version 2 file %d accepted" % i)
                except Exception:
                    tb.check(True, "")
    if not a.no_rebake:
        baker = os.path.join(os.path.dirname(os.path.abspath(__file__)), "travbake.py")

        def bake(od, var, rig):
            """bake one rig of one variant the way its generation was made; returns (completed process, output file)"""
            gname = gens.get((var, rig), (GENERATIONS[var][0][0], None))[0]
            if gname == "shipped":  # knee clips, root-z clips of the fixed leg-twist retarget
                args = ["--variant", "knee", "--rtz-dir", SHIPPED[rig][2]]
                suffix = "_knee"
            else:
                args = ["--variant", "knee" if var == "knee" else "base"]
                suffix = VARIANTS[var][0]
            r = subprocess.run([sys.executable, "-I", baker, "--root", root, "--out-dir", od, "--format", "2",
                                "--rig", rig] + args, capture_output=True, text=True)
            return r, os.path.join(od, "utpp_traversal_%s_motion%s.utpptrav" % (rig, suffix))

        outs = []
        with tempfile.TemporaryDirectory() as td:
            for k in range(2):
                od = os.path.join(td, "run%d" % k)
                os.makedirs(od)
                got = {}
                for var in active:
                    for rig in ("woman", "man"):
                        r, fp = bake(od, var, rig)
                        tb.check(r.returncode == 0, "rebake %d %s %s exit %d %s" % (k, var, rig, r.returncode,
                                                                                   r.stderr[-300:]))
                        if os.path.isfile(fp):
                            got[(var, rig)] = open(fp, "rb").read()
                outs.append(got)
            # no overwrite: run1 already holds every output
            od = os.path.join(td, "run1")
            r, tgt = bake(od, active[0], "woman")
            before = open(tgt, "rb").read()
            open(tgt, "ab").write(b"#")
            r, tgt = bake(od, active[0], "woman")
            after = open(tgt, "rb").read()
            tb.check(r.returncode != 0 and after == before + b"#", "format 2 bake refuses to overwrite (exit %d)" %
                     r.returncode)
            shutil.rmtree(od, ignore_errors=True)
        for key in [(v, r) for v in active for r in ("woman", "man")]:
            tb.check(key in outs[0] and outs[0].get(key) == outs[1].get(key), "%s %s two bakes byte-identical" % key)
            if key in files:
                tb.check(outs[0].get(key) == open(files[key][0], "rb").read(), "%s %s file under test == fresh bake" % key)
        if "knee" in active:
            tb.check(outs[0].get(("base", "woman")) != outs[0].get(("knee", "woman")), "base / knee files differ")
    print("PASS %d FAIL %d" % (tb.PASS, tb.FAIL))
    return 0 if tb.FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
