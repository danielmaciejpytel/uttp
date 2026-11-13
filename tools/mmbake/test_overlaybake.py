"""Independent reader/check of .utppov v1 (FORMAT_OVERLAY.txt) against the retargeted utppclip files.
Run: python -I test_overlaybake.py   (needs Data/overlay/utpp_overlays_{woman,man}.utppov)"""
import json
import os
import struct
import subprocess
import sys
import tempfile
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
FILES = {"woman": ("Data/overlay/utpp_overlays_woman.utppov", "Animations/retarget/out_sourcepack_woman",
                   "Extracted/game/json/vanilla/player_woman_skeleton.rig.json", "Feminine"),
         "man": ("Data/overlay/utpp_overlays_man.utppov", "Animations/retarget/out_sourcepack_man",
                 "Extracted/game/json/vanilla-man/player_man_skeleton.rig.json", "Masculine")}
TOL_DEG, TOL_M = 0.05, 1e-4
# Deliberate curve overrides of the woman overlay (see CURVE_OVERRIDES in overlaybake.py): relaxed 1H pistol idle keeps the
# unarmed lower body. Written out here on purpose: the test must notice if the baker table or the source clip changes.
EXP_OVERRIDES = {"woman": {"Pose_Pistol1H_Stand_Relax_Idle": {"layering_legs": 0.0, "layering_pelvis": 0.0}}}
# plain bake (no overrides) of the woman rig, kept as a fixture for the C++ tests
UNPATCHED_FIXTURE = "Build/test-data/overlay/utpp_overlays_woman_unpatched.utppov"
FAILS = []


def P(*a):
    return os.path.join(ROOT, *[x.replace("/", os.sep) for x in a])


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


def cstr(b):
    return b.split(b"\x00", 1)[0].decode("utf-8")


def read(path):
    d = open(path, "rb").read()
    magic = d[0:8]
    ver, hsz, rh, nb, nsec, fsz, pcrc, hcrc = struct.unpack_from("<IIQIIIII", d, 8)
    h = bytearray(d[:64]); struct.pack_into("<I", h, 40, 0)
    assert magic == b"UTPPOV\x00\x00" and ver == 1 and hsz == 64 and fsz == len(d)
    assert zlib.crc32(bytes(h)) & 0xFFFFFFFF == hcrc and zlib.crc32(d[64:]) & 0xFFFFFFFF == pcrc
    S = {}
    for i in range(nsec):
        tag = d[64 + 16 * i:68 + 16 * i].decode()
        off, size, cnt = struct.unpack_from("<III", d, 68 + 16 * i)
        S[tag] = (d[off:off + size], cnt)
    bones = [(cstr(S["BONE"][0][36 * i:36 * i + 32]), struct.unpack_from("<i", S["BONE"][0], 36 * i + 32)[0])
             for i in range(S["BONE"][1])]
    curves = [cstr(S["CURV"][0][48 * i:48 * i + 48]) for i in range(S["CURV"][1])]
    poses = []
    for i in range(S["POSE"][1]):
        r = S["POSE"][0][104 * i:104 * i + 104]
        g, first, n, add, bp, bf, bk, _, mask = struct.unpack_from("<IIIIiIIIQ", r, 64)
        poses.append(dict(name=cstr(r[:64]), group=g, first=first, n=n, add=add, base=bp, bframe=bf, bkind=bk, mask=mask))
    fl = nb * 17 + len(curves)
    fr = np.frombuffer(S["FRAM"][0], "<f4").reshape(S["FRAM"][1], fl)
    deltas = []
    for i in range(S["DELT"][1]):
        r = S["DELT"][0][80 * i:80 * i + 80]
        k, t, tf, n, b, bf, sp, first = struct.unpack_from("<IiIIiIII", r, 48)
        deltas.append(dict(name=cstr(r[:48]), kind=k, target=t, tfirst=tf, n=n, base=b, bframe=bf, space=sp, first=first))
    df = np.frombuffer(S["DFRM"][0], "<f4").reshape(S["DFRM"][1], nb, 14)
    ov = []
    for i in range(S["OVLY"][1]):
        r = S["OVLY"][0][48 * i:48 * i + 48]
        ov.append((cstr(r[:32]),) + struct.unpack_from("<IIII", r, 32))
    masks = {}
    for i in range(S["MASK"][1]):
        r = S["MASK"][0][(16 + nb * 4) * i:(16 + nb * 4) * (i + 1)]
        masks[cstr(r[:16])] = np.frombuffer(r[16:], "<f4").astype(np.float64)
    return dict(rh=rh, nb=nb, bones=bones, curves=curves, poses=poses, fr=fr, deltas=deltas, df=df, ov=ov,
                masks=masks, raw=d)


# Expected region masks on the CP rig, written from CP bone names (independent of the baker's bonemap path):
# ABP_LayerBlending BranchFilters pelvis (-thighs), spine_01 BlendDepth 5 (ramp re-spread over Spine..Spine3),
# neck_01, clavicle_x, fingers 01 + thumb 01 + ik_hand_x (-> WeaponLeft / WeaponRight).
EXP_RAMP = {"Spine": 0.25, "Spine1": 0.5, "Spine2": 0.75, "Spine3": 1.0}
EXP_REGIONS = {
    "pelvis": (["Hips"], ["LeftUpLeg", "RightUpLeg"]),
    "spine": (["Spine"], []),
    "head": (["Neck"], []),
    "arm_l": (["LeftShoulder"], []),
    "arm_r": (["RightShoulder"], []),
    "hand_l": (["LeftInHandThumb", "LeftHandIndex1", "LeftHandMiddle1", "LeftHandRing1", "LeftHandPinky1",
                "WeaponLeft"], []),
    "hand_r": (["RightInHandThumb", "RightHandIndex1", "RightHandMiddle1", "RightHandRing1", "RightHandPinky1",
                "WeaponRight"], []),
}


def check_masks(F, key):
    names = [n for n, _ in F["bones"]]
    parents = [p for _, p in F["bones"]]

    def under(b, roots):
        while b >= 0:
            if names[b] in roots:
                return True
            b = parents[b]
        return False
    n_bad = 0
    check(sorted(F["masks"]) == sorted(EXP_REGIONS), key + ": mask names")
    for k, (inc, exc) in EXP_REGIONS.items():
        w = F["masks"].get(k)
        if w is None:
            continue
        for b in range(F["nb"]):
            exp = 1.0 if under(b, inc) and not under(b, exc) else 0.0
            if k == "spine" and names[b] in EXP_RAMP:
                exp = EXP_RAMP[names[b]]
            if not check(abs(w[b] - exp) < 1e-6, "%s: mask %s bone %s %.4f != %.4f" % (key, k, names[b], w[b], exp)):
                n_bad += 1
    hl = F["masks"].get("hand_l")
    if hl is not None:
        check(hl[names.index("WeaponLeft")] == 1.0 and hl[names.index("LeftInHandIndex")] == 0.0
              and hl[names.index("LeftHand")] == 0.0, key + ": hand_l = fingers + WeaponLeft, no metacarpal/hand")
    return n_bad


def qm(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0); bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz], -1)


def qang(a, b):
    dot = np.abs(np.sum(a / np.linalg.norm(a, axis=-1, keepdims=True) * b / np.linalg.norm(b, axis=-1, keepdims=True), -1))
    return np.degrees(2 * np.arccos(np.clip(dot, -1, 1)))


def fk(Pl, Ql, parents):
    gq = np.zeros_like(Ql); gp = np.zeros_like(Pl)
    for b, p in enumerate(parents):
        if p < 0:
            gq[b], gp[b] = Ql[b], Pl[b]
        else:
            gq[b] = qm(gq[p], Ql[b])
            u = gq[p][:3]; w = gq[p][3]; v = Pl[b]; t = 2 * np.cross(u, v)
            gp[b] = gp[p] + v + w * t + np.cross(u, t)
    return gp, gq


def rig_hash(rig_path, nb):
    rc = json.load(open(rig_path, encoding="utf-8"))["Data"]["RootChunk"]
    names = [n["$value"] if isinstance(n, dict) else n for n in rc["boneNames"]][:nb]
    h = 0xCBF29CE484222325
    buf = bytearray()
    for b in range(nb):
        t = rc["boneTransforms"][b]["Translation"]
        q = [int(np.floor(float(np.float32(t[k])) * 1000.0 + 0.5)) for k in "XYZ"]
        buf += names[b].encode() + b"\x00" + struct.pack("<iiii", int(rc["boneParentIndexes"][b]), *q)
    for x in buf:
        h = ((h ^ x) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def test_file(key):
    path, cdir, rig, style = FILES[key]
    F = read(P(path))
    nb = F["nb"]
    parents = [p for _, p in F["bones"]]
    check(F["rh"] == rig_hash(P(rig), nb), key + ": rig identity hash")
    other = [k for k in FILES if k != key][0]
    check(F["rh"] != rig_hash(P(FILES[other][2]), nb), key + ": rig hash distinguishes the other rig")
    clips = {}
    stats = dict(max_local_deg=0.0, max_local_m=0.0, max_ms_deg=0.0, max_ms_m=0.0, max_delta_deg=0.0,
                 max_delta_m=0.0, max_delta_scale=0.0, curve_max_abs=0.0, curves_checked=0, poses=0, frames=0,
                 overrides_checked=0)
    exp_ovr = EXP_OVERRIDES.get(key, {})
    for p in F["poses"]:
        c = json.load(open(P(cdir, p["name"] + ".utppclip.json"), encoding="utf-8"))
        check(c["rig"] == ("player_%s_skeleton" % key), p["name"] + ": rig")
        A = np.array([f["bones"][:nb] for f in c["frames"]], np.float64)
        clips[p["name"]] = A
        check(p["n"] == len(A), p["name"] + ": frame count")
        stats["poses"] += 1
        for f in range(p["n"]):
            row = F["fr"][p["first"] + f].astype(np.float64)
            B = row[:nb * 17].reshape(nb, 17)
            Ql = A[f, :, 3:7] / np.linalg.norm(A[f, :, 3:7], axis=-1, keepdims=True)
            stats["max_local_deg"] = max(stats["max_local_deg"], qang(B[:, 3:7], Ql).max())
            stats["max_local_m"] = max(stats["max_local_m"], np.abs(B[:, 0:3] - A[f, :, 0:3]).max())
            gp, gq = fk(A[f, :, 0:3], Ql, parents)
            stats["max_ms_deg"] = max(stats["max_ms_deg"], qang(B[:, 10:14], gq).max())
            stats["max_ms_m"] = max(stats["max_ms_m"], np.abs(B[:, 14:17] - gp).max())
            for ci, cn in enumerate(F["curves"]):
                src = {k.lower(): v for k, v in c.get("curves", {}).items()}
                present = bool(p["mask"] >> ci & 1)
                check(present == (cn in src), "%s: curve %s presence" % (p["name"], cn))
                if present:
                    want = src[cn][f]
                    if cn in exp_ovr.get(p["name"], {}):
                        # the source clip must still carry 1.0 here, otherwise the override is stale
                        check(want == 1.0, "%s: %s %s frame %d source %.4f != 1.0 (override stale?)"
                              % (key, p["name"], cn, f, want))
                        want = exp_ovr[p["name"]][cn]
                        stats["overrides_checked"] += 1
                    stats["curve_max_abs"] = max(stats["curve_max_abs"], abs(row[nb * 17 + ci] - want))
                    stats["curves_checked"] += 1
            stats["frames"] += 1
    for d in F["deltas"]:
        X = clips[F["poses"][d["target"]]["name"]]
        Bc = clips[F["poses"][d["base"]]["name"]][d["bframe"]]
        Qb = Bc[:, 3:7] / np.linalg.norm(Bc[:, 3:7], axis=-1, keepdims=True)
        _, gqb = fk(Bc[:, 0:3], Qb, parents)
        for i in range(d["n"]):
            Xf = X[d["tfirst"] + i]
            Qx = Xf[:, 3:7] / np.linalg.norm(Xf[:, 3:7], axis=-1, keepdims=True)
            _, gqx = fk(Xf[:, 0:3], Qx, parents)
            inv = np.array([-1, -1, -1, 1.0])
            D = F["df"][d["first"] + i].astype(np.float64)
            stats["max_delta_deg"] = max(stats["max_delta_deg"], qang(D[:, 0:4], qm(gqx, gqb * inv)).max(),
                                         qang(D[:, 4:8], qm(Qx, Qb * inv)).max())
            stats["max_delta_m"] = max(stats["max_delta_m"], np.abs(D[:, 8:11] - (Xf[:, 0:3] - Bc[:, 0:3])).max())
            stats["max_delta_scale"] = max(stats["max_delta_scale"], np.abs(D[:, 11:14] - (Xf[:, 7:10] / Bc[:, 7:10] - 1)).max())
    for k in ("max_local_deg", "max_ms_deg", "max_delta_deg"):
        check(stats[k] <= TOL_DEG, "%s: %s %.6f > %.2f deg" % (key, k, stats[k], TOL_DEG))
    for k in ("max_local_m", "max_ms_m", "max_delta_m"):
        check(stats[k] <= TOL_M, "%s: %s %.7f > 0.1 mm" % (key, k, stats[k]))
    check(stats["max_delta_scale"] <= 1e-5, key + ": delta scale")
    check(stats["curve_max_abs"] <= 1e-5, key + ": curve values")
    n_ovr = sum(len(cv) * next(p["n"] for p in F["poses"] if p["name"] == pn) for pn, cv in exp_ovr.items())
    check(stats["overrides_checked"] == n_ovr, "%s: %d override values checked, expected %d"
          % (key, stats["overrides_checked"], n_ovr))
    # every chosen overlay state present
    names = {p["name"] for p in F["poses"]}
    ovn = [o[0] for o in F["ov"]]
    for o in ["BasePoses", style, "Default", "Pistol1H", "Pistol2H", "Rifle", "Injured", "Transitions"]:
        check(o in ovn, key + ": overlay " + o)
    req = ["Pose_Base_Stand", "Pose_Base_Crouch", "Pose_Transition", "Stand_Transition", "Poses_" + style]
    req += ["Pose_Neutral_%s_%s" % (s, m) for s in ("Stand", "Crouch") for m in ("Idle", "Move")]
    req += ["Pose_%s_%s" % (style, s) for s in ("Stand_Idle", "Stand_Move", "Crouch")]
    for o in ("Pistol1H", "Pistol2H", "Rifle"):
        req += ["AO_%s_%s_Sweep" % (o, s) for s in ("Stand", "Crouch")] + ["Poses_" + o]
        req += ["Pose_%s_Crouch_%s" % (o, s) for s in ("Relax", "Ready", "Aim")]
        req += ["Pose_%s_Stand_%s" % (o, s) for s in ("Ready_Idle", "Aim_Idle", "Aim_Move")]
    req += ["Pose_Rifle_%s" % s for s in ("Run", "Run_Arms", "Sprint", "Sprint_Arms", "Sprint_Impulse", "Sprint_Impulse_Arms")]
    req += ["Pose_Injured_Stand_Idle", "Pose_Injured_Stand_Move", "Pose_Injured_Crouch", "Poses_Injured"]
    for r in req:
        check(r in names, key + ": pose " + r)
    sweeps = [p for p in F["poses"] if p["name"].startswith("AO_")]
    check(all(p["add"] == 1 and p["bkind"] == 2 and p["bframe"] == 15 and F["poses"][p["base"]]["name"] == p["name"]
              for p in sweeps) and len(sweeps) == 6, key + ": sweeps base = own frame 15, mesh rotation offset")
    st = [p for p in F["poses"] if p["name"] in ("Pose_Transition", "Stand_Transition")]
    check(all(p["add"] == 1 and p["bkind"] == 1 and F["poses"][p["base"]]["name"] == "Pose_Base_Stand" for p in st),
          key + ": transitions base Pose_Base_Stand")
    check(sum(1 for d in F["deltas"] if d["kind"] == 1) == 3, key + ": 3 style dP")
    stats["mask_bad_weights"] = check_masks(F, key)
    stats["mask_spine_ramp"] = [round(float(F["masks"]["spine"][[n for n, _ in F["bones"]].index(b)]), 4)
                                for b in EXP_RAMP] if "spine" in F["masks"] else None
    stats["file_bytes"] = len(F["raw"])
    return F, stats


def main():
    out = {}
    for k in FILES:
        F, s = test_file(k)
        out[k] = s
        print(k, json.dumps({a: (round(b, 7) if isinstance(b, float) else b) for a, b in s.items()}))
    # determinism: rebake into a temp dir, byte-compare (normal bake == shipped data; --no-overrides == plain bake)
    bake_py = os.path.join(HERE, "overlaybake.py")
    with tempfile.TemporaryDirectory() as td, tempfile.TemporaryDirectory() as td2, tempfile.TemporaryDirectory() as td3:
        subprocess.check_call([sys.executable, "-I", bake_py, "--out-dir", td], stdout=subprocess.DEVNULL)
        subprocess.check_call([sys.executable, "-I", bake_py, "--out-dir", td2], stdout=subprocess.DEVNULL)
        subprocess.check_call([sys.executable, "-I", bake_py, "--no-overrides", "--out-dir", td3],
                              stdout=subprocess.DEVNULL)
        for k, (path, *_r) in FILES.items():
            name = os.path.basename(path)
            rebake = open(os.path.join(td, name), "rb").read()
            same = rebake == open(P(path), "rb").read()
            check(same, k + ": deterministic rebake")
            print(k, "deterministic rebake:", same)
            check(rebake == open(os.path.join(td2, name), "rb").read(), k + ": two bakes differ")
            plain = open(os.path.join(td3, name), "rb").read()
            check((plain == rebake) == (not EXP_OVERRIDES.get(k)), k + ": --no-overrides result vs overrides table")
            if k == "woman":
                fx = P(UNPATCHED_FIXTURE)
                if os.path.exists(fx):
                    ok = plain == open(fx, "rb").read()
                    check(ok, k + ": plain bake != unpatched fixture")
                    print(k, "plain bake == unpatched fixture:", ok)
                else:
                    print(k, "unpatched fixture missing, plain-bake check skipped")
    print("FAILS:", len(FAILS))
    for f in FAILS[:40]:
        print("  FAIL", f)
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
