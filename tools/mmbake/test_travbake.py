"""test_travbake.py - independent check of the .utpptrav files written by travbake.py (FORMAT_TRAVERSAL.txt v1).

Reads the files by the spec only (own parser, own FK with rotation matrices, own rig hash) and checks:
  format / token grammar / float format, rig identity (name, FNV-1a 64 hash, bone count) against the rig json,
  montage set = traversal montages of Data/traversal/traversal_table.json, M line vs the montage meta (length, rate,
  blend in/out, auto blend out, trigger time, sequence, frames of the single segment), W lines vs the meta
  AnimNotifyState_MotionWarping notifies (count, order, target, times, frames, flags, provider, bone), contact bone
  offset vs an independent FK of the clip pose, the planted-hand rule, R lines vs the retarget clip (Trajectory x y yaw,
  root_z of the *_rtz clip), the Distance_From_Ledge curve vs the clip, woman / man consistency and determinism
  (two fresh bakes into temporary directories are byte-identical to each other and to the files under test).

usage: python -I test_travbake.py [--root <project>] [--no-rebake]
exit 0 = all checks passed; prints PASS / FAIL counts.
"""
import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import tempfile

RIGS = {
    "woman": ("Extracted/game/json/vanilla/player_woman_skeleton.rig.json", "Animations/retarget/out_sourcepack_woman",
              "Animations/retarget/out_sourcepack_woman_rtz"),
    "man": ("Extracted/game/json/vanilla-man/player_man_skeleton.rig.json", "Animations/retarget/out_sourcepack_man",
            "Animations/retarget/out_sourcepack_man_rtz"),
}
FLOAT_RE = re.compile(r"^-?\d+\.\d{6}$")
OPTS = {"LINEAR": "Linear", "CUBIC": "Cubic", "HERMITE_CUBIC": "HermiteCubic", "SINUSOIDAL": "Sinusoidal",
        "QUADRATIC_IN_OUT": "QuadraticInOut", "CUBIC_IN_OUT": "CubicInOut", "QUARTIC_IN_OUT": "QuarticInOut",
        "QUINTIC_IN_OUT": "QuinticInOut", "CIRCULAR_IN": "CircularIn", "CIRCULAR_OUT": "CircularOut",
        "CIRCULAR_IN_OUT": "CircularInOut", "EXP_IN": "ExpIn", "EXP_OUT": "ExpOut", "EXP_IN_OUT": "ExpInOut"}

PASS = 0
FAIL = 0


def check(ok, what):
    global PASS, FAIL
    if ok:
        PASS += 1
    else:
        FAIL += 1
        if FAIL <= 60:
            print("FAIL:", what)
    return ok


def fl(s):
    if not FLOAT_RE.match(s) or s == "-0.000000":
        raise ValueError("bad float token %r" % s)
    return float(s)


# ------------------------------------------------------------------------------------------------ reader (by spec)
def read_trav(path):
    raw = open(path, "rb").read()
    raw.decode("ascii")
    if b"\r" in raw or not raw.endswith(b"\n"):
        raise ValueError("not LF text")
    lines = raw.decode("ascii").split("\n")[:-1]
    it = iter(enumerate(lines, 1))

    def nxt():
        i, ln = next(it)
        if ln != ln.strip() or "  " in ln:
            raise ValueError("line %d: whitespace" % i)
        return ln.split(" ")

    t = nxt()
    if t != ["utpp-traversal", "1"]:
        raise ValueError("header")
    t = nxt()
    if t[0] != "RIG" or len(t) != 4 or not re.match(r"^0x[0-9a-f]{16}$", t[2]):
        raise ValueError("RIG")
    out = {"rig": t[1], "hash": int(t[2], 16), "bones": int(t[3]), "montages": []}
    t = nxt()
    if t[0] != "COUNT" or len(t) != 2:
        raise ValueError("COUNT")
    count = int(t[1])
    for _ in range(count):
        t = nxt()
        if t[0] != "M" or len(t) != 15:
            raise ValueError("M line %r" % t[:2])
        m = {"name": t[1], "seq": t[2], "length": fl(t[3]), "rate": fl(t[4]), "bi": fl(t[5]), "bio": t[6],
             "bo": fl(t[7]), "boo": t[8], "auto": int(t[9]), "trig": fl(t[10]), "action": t[11], "fps": fl(t[12]),
             "frames": int(t[13]), "rtz": int(t[14]), "sec": [], "win": [], "root": [], "dfl": None}
        if m["auto"] not in (0, 1) or m["rtz"] not in (0, 1) or m["frames"] < 2:
            raise ValueError("M flags")
        t = nxt()
        while t[0] == "SEC":
            if len(t) != 4:
                raise ValueError("SEC")
            m["sec"].append((t[1], fl(t[2]), t[3]))
            t = nxt()
        if not m["sec"]:
            raise ValueError("no SEC")
        while t[0] == "W":
            if len(t) != 20:
                raise ValueError("W tokens %d" % len(t))
            w = {"target": t[1], "start": fl(t[2]), "end": fl(t[3]), "sf": int(t[4]), "ef": int(t[5]), "cls": t[6],
                 "wt": int(t[7]), "wr": int(t[8]), "iz": int(t[9]), "feet": int(t[10]), "rot": t[11],
                 "rmul": fl(t[12]), "prov": t[13], "ubone": t[14], "bone": t[15],
                 "off": (fl(t[16]), fl(t[17]), fl(t[18])), "clamp": fl(t[19])}
            m["win"].append(w)
            t = nxt()
        for k in range(m["frames"]):
            if k:
                t = nxt()
            if t[0] != "R" or len(t) != 5:
                raise ValueError("R line %d of %s" % (k, m["name"]))
            m["root"].append(tuple(fl(x) for x in t[1:]))
        t = nxt()
        if t[0] != "C" or t[1] != "Distance_From_Ledge":
            raise ValueError("C line")
        if t[2:] != ["-"]:
            if len(t) != 2 + m["frames"]:
                raise ValueError("C length")
            m["dfl"] = [fl(x) for x in t[2:]]
        t = nxt()
        if t != ["E"]:
            raise ValueError("E")
        out["montages"].append(m)
    if nxt() != ["END"]:
        raise ValueError("END")
    if next(it, None) is not None:
        raise ValueError("trailing lines")
    return out


# ------------------------------------------------------------------------------------------------ independent math
def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def rig_hash(rig_path):
    rc = json.load(open(rig_path, encoding="utf-8"))["Data"]["RootChunk"]
    names = [b["$value"] if isinstance(b, dict) else b for b in rc["boneNames"]]
    parents = [int(p) for p in rc["boneParentIndexes"]]
    h = 0xcbf29ce484222325
    for n, p, tr in zip(names, parents, rc["boneTransforms"]):
        rec = n.encode("utf-8") + b"\x00" + struct.pack("<i", p)
        for ax in ("X", "Y", "Z"):
            rec += struct.pack("<i", math.floor(f32(tr["Translation"][ax]) * 1000.0 + 0.5))
        for byte in rec:
            h ^= byte
            h = (h * 0x100000001b3) % (1 << 64)
    return names, parents, h


def qmat(x, y, z, w):
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def mmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mvec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def fk_world_chain(bones, clip_names, rig_names, rig_parents, bone):
    chain = []
    i = rig_names.index(bone)
    while i >= 0:
        chain.append(rig_names[i])
        i = rig_parents[i]
    pos, rot = (0.0, 0.0, 0.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    for name in reversed(chain):
        b = bones[clip_names.index(name)]
        lp = mvec(rot, (b[0], b[1], b[2]))
        pos = (pos[0] + lp[0], pos[1] + lp[1], pos[2] + lp[2])
        rot = mmul(rot, qmat(b[3], b[4], b[5], b[6]))
    return pos


def yaw_of(x, y, z, w):
    m = qmat(x, y, z, w)
    return math.atan2(m[1][0], m[0][0])


def rnd(x):
    return int(math.floor(x + 0.5))


# ------------------------------------------------------------------------------------------------ checks
def check_rig_file(root, rig, data, montage_names):
    rig_json, cdir, rdir = RIGS[rig]
    rnames, rpar, h = rig_hash(os.path.join(root, rig_json))
    check(data["rig"] == os.path.basename(rig_json).split(".")[0], "%s rig name %s" % (rig, data["rig"]))
    check(data["hash"] == h, "%s rig hash 0x%016x != 0x%016x" % (rig, data["hash"], h))
    check(data["bones"] == len(rnames), "%s bone count" % rig)
    got = [m["name"] for m in data["montages"]]
    check(got == sorted(montage_names), "%s montage set/order (%d vs %d)" % (rig, len(got), len(montage_names)))
    nwin = 0
    for m in data["montages"]:
        name = m["name"]
        meta = json.load(open(os.path.join(root, "Animations/sourcepack/als_export/meta", name + ".json"), encoding="utf-8"))
        check(m["seq"] == meta["referenced_sequences"][0], name + " seq")
        check(abs(m["length"] - meta["length_s"]) < 1e-6, name + " length")
        check(abs(m["rate"] - meta.get("rate_scale", 1.0)) < 1e-6, name + " rate")
        check(abs(m["bi"] - meta["blend_in"]["blend_time"]) < 1e-6, name + " blend in")
        check(m["bio"] == OPTS.get(meta["blend_in"]["blend_option"], meta["blend_in"]["blend_option"]), name + " bio")
        check(abs(m["bo"] - meta["blend_out"]["blend_time"]) < 1e-6, name + " blend out")
        check(m["boo"] == OPTS.get(meta["blend_out"]["blend_option"], meta["blend_out"]["blend_option"]), name + " boo")
        check(m["auto"] == (1 if meta.get("enable_auto_blend_out", True) else 0), name + " auto blend out")
        check(abs(m["trig"] - meta.get("blend_out_trigger_time", -1.0)) < 1e-6, name + " trigger")
        exp_act = next((a for k, a in (("_Catch_", "Catch"), ("_Climb_Start", "ClimbStart"), ("_Hurdle_", "Hurdle"),
                                       ("_Vault_", "Vault"), ("_Mantle_", "Mantle")) if k in name), "Other")
        check(m["action"] == exp_act, name + " action")
        secs = meta.get("composite_sections") or []
        check(len(m["sec"]) == max(1, len(secs)), name + " sections")
        # clip
        rp = os.path.join(root, rdir, m["seq"] + ".utppclip.json")
        has_rtz = os.path.isfile(rp)
        check(m["rtz"] == (1 if has_rtz else 0), name + " rtz flag")
        clip = json.load(open(rp if has_rtz else os.path.join(root, cdir, m["seq"] + ".utppclip.json"),
                              encoding="utf-8"))
        check(clip["rig"] == data["rig"], name + " clip rig")
        fps = float(clip["fps"])
        check(abs(m["fps"] - fps) < 1e-9, name + " fps")
        seg = meta["slot_anim_tracks"][0]["anim_track"]["anim_segments"][0]
        f0 = rnd(seg["anim_start_time"] * fps)
        f1 = min(clip["num_frames"] - 1, rnd(seg["anim_end_time"] * fps))
        check(m["frames"] == f1 - f0 + 1, "%s frames %d != %d" % (name, m["frames"], f1 - f0 + 1))
        cn = clip["bone_names"]
        ti = cn.index("Trajectory")
        bad = 0
        for k, r in enumerate(m["root"]):
            b = clip["frames"][f0 + k]["bones"][ti]
            z = clip["root_z"][f0 + k] if has_rtz else b[2]
            yw = yaw_of(b[3], b[4], b[5], b[6])
            dy = abs(math.remainder(r[3] - yw, 2 * math.pi))
            if max(abs(r[0] - b[0]), abs(r[1] - b[1]), abs(r[2] - z), dy) > 2e-6:
                bad += 1
        check(bad == 0, "%s root track: %d frames differ from the retarget clip" % (name, bad))
        cur = clip.get("curves", {}).get("distance_from_ledge")
        if cur is None or len(cur) != clip["num_frames"]:
            check(m["dfl"] is None, name + " Distance_From_Ledge should be '-'")
        else:
            check(m["dfl"] is not None and all(abs(a - b) < 1e-5 for a, b in zip(m["dfl"], cur[f0:f1 + 1])),
                  name + " Distance_From_Ledge vs clip")
        # warp windows vs meta notifies
        notes = sorted([x for x in meta["notifies"] if x.get("class") == "AnimNotifyState_MotionWarping"],
                       key=lambda x: x["trigger_time"])
        check(len(m["win"]) == len(notes), "%s windows %d != notifies %d" % (name, len(m["win"]), len(notes)))
        nwin += len(m["win"])
        prev_start = -1.0
        for w, nt in zip(m["win"], notes):
            rm = nt["props"]["root_motion_modifier"]
            tag = "%s %s@%.3f" % (name, w["target"], w["start"])
            check(w["start"] >= prev_start, tag + " sorted")
            prev_start = w["start"]
            check(w["target"] == rm.get("warp_target_name") and w["target"] in ("FrontLedge", "BackLedge", "BackFloor"),
                  tag + " target")
            check(abs(w["start"] - nt["trigger_time"]) < 1e-6 and abs(w["end"] - nt["end_time"]) < 1e-6, tag + " t")
            sf = min(m["frames"] - 1, rnd((seg["anim_start_time"] + nt["trigger_time"]) * fps) - f0)
            ef = min(m["frames"] - 1, rnd((seg["anim_start_time"] + nt["end_time"]) * fps) - f0)
            check(w["sf"] == sf and w["ef"] == ef and 0 <= sf <= ef, tag + " frames %d %d vs %d %d" % (
                w["sf"], w["ef"], sf, ef))
            check(w["cls"] == rm.get("_class"), tag + " modifier class")
            check((w["wt"], w["wr"], w["iz"], w["feet"]) == (
                int(bool(rm.get("warp_translation"))), int(bool(rm.get("warp_rotation"))),
                int(bool(rm.get("ignore_z_axis"))), int(bool(rm.get("warp_to_feet_location")))), tag + " flags")
            check(w["rot"] == rm.get("rotation_type") and abs(w["rmul"] - rm.get("warp_rotation_time_multiplier", 1.0))
                  < 1e-6, tag + " rotation settings")
            check(w["prov"] == rm.get("warp_point_anim_provider") and w["ubone"] == str(
                rm.get("warp_point_anim_bone_name") or "-"), tag + " provider")
            check(abs(w["clamp"] - rm.get("max_speed_clamp_ratio", 0.0)) < 1e-6, tag + " clamp")
            if w["target"] == "BackFloor":
                check(w["bone"] == "Root" and w["off"] == (0.0, 0.0, 0.0), tag + " BackFloor root")
                continue
            check(w["bone"] in ("LeftHand", "RightHand"), tag + " contact bone")

            def world(k, hand):
                p = fk_world_chain(clip["frames"][f0 + k]["bones"], cn, rnames, rpar, hand)
                r = m["root"][k]
                c, s = math.cos(r[3]), math.sin(r[3])
                return (r[0] + c * p[0] - s * p[1], r[1] + s * p[0] + c * p[1], r[2] + p[2])

            a, c = max(0, ef - 1), min(m["frames"] - 1, ef + 1)
            dl = math.dist(world(a, "LeftHand"), world(c, "LeftHand"))
            dr = math.dist(world(a, "RightHand"), world(c, "RightHand"))
            exp_bone = "LeftHand" if dl <= dr + 1e-9 else "RightHand"
            if abs(dl - dr) > 1e-6:
                check(w["bone"] == exp_bone, tag + " planted hand %s (dl %.4f dr %.4f)" % (w["bone"], dl, dr))
            p = fk_world_chain(clip["frames"][f0 + ef]["bones"], cn, rnames, rpar, w["bone"])
            check(max(abs(p[i] - w["off"][i]) for i in range(3)) < 2e-5, tag + " offset %s vs FK %s" % (
                w["off"], tuple(round(v, 6) for v in p)))
            # world contact position at the window end must equal root + RotZ(yaw) off (the warp point definition)
            r = m["root"][ef]
            cs, sn = math.cos(r[3]), math.sin(r[3])
            wp = (r[0] + cs * w["off"][0] - sn * w["off"][1], r[1] + sn * w["off"][0] + cs * w["off"][1],
                  r[2] + w["off"][2])
            check(math.dist(wp, world(ef, w["bone"])) < 5e-5, tag + " warp point = world hand")
    return nwin


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
    check(len(found) == 34, "traversal table montages %d != 34" % len(found))
    files = {}
    for rig in ("woman", "man"):
        p = os.path.join(root, "Data/traversal/utpp_traversal_%s.utpptrav" % rig)
        try:
            data = read_trav(p)
        except Exception as e:  # fail closed
            check(False, "%s: read failed: %s" % (p, e))
            continue
        files[rig] = (p, data)
        nwin = check_rig_file(root, rig, data, found)
        print("%s: %d montages, %d warp windows" % (rig, len(data["montages"]), nwin))
    if len(files) == 2:
        w, m = files["woman"][1], files["man"][1]
        check(w["hash"] != m["hash"], "woman / man rig hash differ")
        for mw, mm in zip(w["montages"], m["montages"]):
            check(mw["name"] == mm["name"] and [(x["target"], x["start"], x["end"], x["sf"], x["ef"]) for x in mw["win"]]
                  == [(x["target"], x["start"], x["end"], x["sf"], x["ef"]) for x in mm["win"]],
                  mw["name"] + " woman / man windows")
    # negative control: the reader must reject a corrupted file
    if files:
        raw = open(files["woman"][0], "rb").read()
        with tempfile.TemporaryDirectory() as td:
            for i, bad in enumerate((raw.replace(b"\nE\n", b"\n", 1), raw.replace(b"utpp-traversal 1", b"utpp-traversal 2"),
                                     raw[:-4], raw.replace(b"\nR ", b"\nR 1.0 ", 1))):
                bp = os.path.join(td, "bad%d.utpptrav" % i)
                open(bp, "wb").write(bad)
                try:
                    read_trav(bp)
                    check(False, "corrupt file %d accepted" % i)
                except Exception:
                    check(True, "")
    # determinism: two fresh bakes
    if not a.no_rebake:
        baker = os.path.join(os.path.dirname(os.path.abspath(__file__)), "travbake.py")
        outs = []
        with tempfile.TemporaryDirectory() as td:
            for k in range(2):
                od = os.path.join(td, "run%d" % k)
                os.makedirs(od)
                r = subprocess.run([sys.executable, "-I", baker, "--root", root, "--out-dir", od],
                                   capture_output=True, text=True)
                check(r.returncode == 0, "rebake %d exit %d %s" % (k, r.returncode, r.stderr[-300:]))
                outs.append({rig: open(os.path.join(od, "utpp_traversal_%s.utpptrav" % rig), "rb").read()
                             for rig in ("woman", "man") if os.path.isfile(os.path.join(od, "utpp_traversal_%s.utpptrav" % rig))})
        for rig in ("woman", "man"):
            check(rig in outs[0] and outs[0].get(rig) == outs[1].get(rig), rig + " two bakes byte-identical")
            if rig in files:
                check(outs[0].get(rig) == open(files[rig][0], "rb").read(), rig + " file under test == fresh bake")
    print("PASS %d FAIL %d" % (PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
