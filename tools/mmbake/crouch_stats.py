#!/usr/bin/env python3
"""Offline crouch-clip statistics from a baked .utppmm (read-only): planar root speed, root-motion skew against the
hip / shoulder line, hip and head height and trunk lean, per clip, against stand-idle and crouch-idle references.

Usage: python crouch_stats.py --db Data/utpp_base.utppmm [--out table.txt] [--match Crouch] [--json out.json]
Frame conventions: see FORMAT.txt (POSE = game-local transforms, ROOT = step in the Trajectory frame of that frame,
+Z up, +Y forward, +X right, yaw CCW). Angles: 0 = forward, positive = to the LEFT (CCW), +-90 lateral, 180 backward.
skew = deviation of the (net) motion direction from the nearest cardinal direction (0/+-90/180) measured against the
hip line (LeftUpLeg -> RightUpLeg); 'skewS' the same against the shoulder line (LeftArm -> RightArm).
"""
import argparse
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mmbake_base import fk_model  # noqa: E402

FOURCC = lambda s: sum(ord(c) << (8 * i) for i, c in enumerate(s))  # noqa: E731


def load_db(path):
    mm = np.memmap(path, dtype=np.uint8, mode="r")
    magic = bytes(mm[:8])
    if magic != b"UTPPMMDB":
        raise SystemExit("not a utppmm: " + str(path))
    nsec = struct.unpack_from("<I", mm, 84)[0]
    toff = struct.unpack_from("<I", mm, 88)[0]
    nframes, nbones, _, nclips = struct.unpack_from("<4I", mm, 36)
    fps = struct.unpack_from("<f", mm, 80)[0]
    sec = {}
    for i in range(nsec):
        sid, cnt, esz, _, off, size = struct.unpack_from("<4IQQ", mm, toff + 32 * i)
        sec[sid] = (cnt, esz, off)
    def arr(name, dtype):
        cnt, esz, off = sec[FOURCC(name)]
        return np.frombuffer(mm, dtype=dtype, count=cnt * esz // np.dtype(dtype).itemsize, offset=off)
    strs = bytes(arr("STRS", np.uint8))
    def s(o):
        return strs[o:strs.index(b"\0", o)].decode("utf-8")
    bone = arr("BONE", np.uint32).reshape(-1, 4)
    names = [s(int(r[0])) for r in bone]
    parents = [int(np.int32(r[1])) for r in bone]
    clip = arr("CLIP", np.uint32).reshape(-1, 8)
    clips = [{"name": s(int(r[0])), "first": int(r[1]), "n": int(r[2]), "flags": int(r[3]), "period": int(r[4])} for r in clip]
    pose = arr("POSE", np.float32).reshape(nframes, nbones, 7)
    root = arr("ROOT", np.float32).reshape(nframes, 4)
    return {"names": names, "parents": parents, "clips": clips, "pose": pose, "root": root, "fps": fps}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def ang(v):
    """Angle of planar vectors (x right, y forward): 0 forward, CCW (left) positive."""
    return np.arctan2(-v[..., 0], v[..., 1])


def line_fwd(l):
    """Forward vector (xy) of a body line given as left->right vector l."""
    return np.stack([-l[..., 1], l[..., 0]], -1)


def clip_stats(db, c, idx):
    f0, n = c["first"], c["n"]
    P = np.asarray(db["pose"][f0:f0 + n, :, :3], dtype=np.float64)
    Q = np.asarray(db["pose"][f0:f0 + n, :, 3:], dtype=np.float64)
    gp, _ = fk_model(P, Q, db["parents"])
    R = np.asarray(db["root"][f0:f0 + n], dtype=np.float64)
    loop = bool(c["flags"] & 1) and c["period"] > 0
    m = c["period"] if loop else n - 1            # steps used (loop: frame N-1 == frame 0)
    m = max(m, 1)
    step = R[:m, :2]
    speed = np.linalg.norm(step, axis=1) * db["fps"]
    out = {"name": c["name"], "n": n, "loop": loop, "v_mean": float(speed.mean()), "v_med": float(np.median(speed)),
           "v_min": float(speed.min()), "v_max": float(speed.max()), "net_m": float(np.linalg.norm(step.sum(0))),
           "dur": (m / db["fps"])}
    out["v_net"] = out["net_m"] / out["dur"]
    out["yaw_deg"] = float(np.degrees(R[:m, 2].sum()))
    for key, (a, b) in (("hip", (idx["LeftUpLeg"], idx["RightUpLeg"])), ("sho", (idx["LeftArm"], idx["RightArm"]))):
        fwd = line_fwd(gp[:m, b, :2] - gp[:m, a, :2])
        body = ang(fwd)                             # body-line forward angle in the character frame
        mot = ang(step)
        rel = wrap(mot - body)
        w = np.linalg.norm(step, axis=1)
        vec = np.stack([np.sum(w * np.cos(rel)), np.sum(w * np.sin(rel))])
        net = float(np.degrees(np.arctan2(vec[1], vec[0]))) if w.sum() > 1e-9 else float("nan")
        # nearest cardinal direction
        card = round(net / 90.0) * 90.0 if net == net else float("nan")
        if card == -180.0:
            card = 180.0
        skew = float(np.degrees(wrap(np.radians(net - card)))) if net == net else float("nan")
        mov = w * db["fps"] > 0.3
        pf = np.degrees(rel[mov]) if mov.any() else np.array([0.0])
        out[key + "_net"] = net
        out[key + "_card"] = card
        out[key + "_skew"] = skew
        out[key + "_p5"] = float(np.nanpercentile(pf, 5))
        out[key + "_p95"] = float(np.nanpercentile(pf, 95))
        out["bodyrel_" + key] = float(np.degrees(np.arctan2(np.mean(np.sin(body)), np.mean(np.cos(body)))))  # body line vs Trajectory frame
    w = np.linalg.norm(step, axis=1)
    mt = ang(step)
    out["trk_net"] = float(np.degrees(np.arctan2(np.sum(w * np.sin(mt)), np.sum(w * np.cos(mt))))) if w.sum() > 1e-9 else float("nan")
    hz = gp[:, idx["Hips"], 2]
    hdz = gp[:, idx["Head"], 2]
    out["hipz_mean"] = float(hz.mean())
    out["hipz_min"] = float(hz.min())
    out["hipz_max"] = float(hz.max())
    out["hipz_first"] = float(hz[0])
    out["hipz_last"] = float(hz[-1])
    out["headz_mean"] = float(hdz.mean())
    v = gp[:, idx["Neck"]] - gp[:, idx["Hips"]]
    lean = np.degrees(np.arctan2(np.linalg.norm(v[:, :2], axis=1), v[:, 2]))
    out["lean_mean"] = float(lean.mean())
    out["lean_max"] = float(lean.max())
    foot = np.minimum(gp[:, idx["LeftFoot"], 2], gp[:, idx["RightFoot"], 2])
    out["foot_min"] = float(foot.min())
    return out


def kind_of(name):
    return "idle" if ("Idle" in name or "Transition" in name or "Stand_to" in name) else "loco"


GROUPS = ("M_Neutral_Crouch_Box_", "M_Neutral_Crouch_Diamond_", "M_Neutral_Crouch_Hourglass_", "M_Neutral_Crouch_Pivot_",
          "M_Neutral_Crouch_Start_", "M_Neutral_Crouch_Stop_", "M_Neutral_Crouch_Turn_", "M_Neutral_Crouch_Reface_Start_")


def fmt_table(rows, refs, speed_map):
    L = []
    h = ("clip", "L", "n", "v_mean", "v_med", "vmin..max", "%map", "netH", "skewH", "netS", "netT", "hipZ", "%stand", "headZ", "lean")
    L.append("%-52s %1s %4s %6s %6s %-11s %4s %6s %6s %6s %6s %6s %6s %6s %5s" % h)
    for r in rows:
        L.append("%-52s %1s %4d %6.2f %6.2f %-11s %3.0f%% %6.1f %6.1f %6.1f %6.1f %6.3f %5.0f%% %6.3f %5.1f" % (
            r["name"][:52], "L" if r["loop"] else "-", r["n"], r["v_mean"], r["v_med"],
            "%.2f..%.2f" % (r["v_min"], r["v_max"]), 100 * r["v_mean"] / speed_map, r["hip_net"], r["hip_skew"],
            r["sho_net"], r["trk_net"], r["hipz_mean"], 100 * r["hipz_mean"] / refs["stand"],
            r["headz_mean"], r["lean_mean"]))
    return L


def fmt_groups(allr, refs):
    L = ["%-34s %4s %7s %7s %7s %7s %6s %6s" % ("group (Neutral crouch, summed)", "n", "hipZmin", "hipZmean", "hipZmax", "%stand", "headZ", "lean")]
    for g in GROUPS:
        rs = [r for n, r in allr.items() if n.startswith(g)]
        if not rs:
            continue
        mean = float(np.mean([r["hipz_mean"] for r in rs]))
        L.append("%-34s %4d %7.3f %7.3f %7.3f %6.0f%% %6.3f %6.1f" % (
            g, len(rs), min(r["hipz_min"] for r in rs), mean, max(r["hipz_max"] for r in rs), 100 * mean / refs["stand"],
            float(np.mean([r["headz_mean"] for r in rs])), float(np.mean([r["lean_mean"] for r in rs]))))
    return L


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--match", default="Crouch", help="substring of clip names to report (plus references)")
    ap.add_argument("--speed-map", type=float, default=2.46, help="crouch speed map (m/s) for the %%map column")
    a = ap.parse_args()
    db = load_db(a.db)
    idx = {n: i for i, n in enumerate(db["names"])}
    allr = {}
    want = [c for c in db["clips"] if a.match in c["name"] or "Stand_Idle_Loop" in c["name"] or "Transition" in c["name"]
            and "rouch" in c["name"]]
    for c in want:
        allr[c["name"]] = clip_stats(db, c, idx)
    stand_names = [n for n in allr if "Stand_Idle_Loop" in n]
    refs = {"stand": float(np.mean([allr[n]["hipz_mean"] for n in stand_names]))}
    rows_m = sorted(((n, r) for n, r in allr.items() if a.match in n and not n.startswith(GROUPS)), key=lambda t: t[0])
    rows = [r for _, r in rows_m]
    L = ["# crouch statistics from %s (frames at %.0f fps; m, m/s, deg)" % (a.db, db["fps"]),
         "# columns: L = loop; v_* planar root speed; %%map = v_mean / %.2f m/s; netH = net root-motion direction vs the HIP line "
         "(0 fwd, + left, 180 back), skewH = its deviation from the nearest cardinal direction (0/90/180); netS = net direction vs the "
         "SHOULDER line (LeftArm->RightArm); netT = net direction vs the Trajectory frame (the capsule); hipZ = mean Hips height "
         "(model space, m); %%stand = hipZ / mean stand-idle hipZ; headZ = mean Head height; lean = mean angle of "
         "Hips->Neck from vertical (deg)." % a.speed_map,
         "# references (hipZ): stand idles (%s) %.3f m" % (", ".join(sorted(stand_names)), refs["stand"]), ""]
    L += fmt_table(rows, refs, a.speed_map)
    L.append("")
    L += fmt_groups(allr, refs)
    L.append("")
    L.append("# stand idle references: " + "; ".join("%s hipZ %.3f headZ %.3f lean %.1f" % (n, allr[n]["hipz_mean"], allr[n]["headz_mean"], allr[n]["lean_mean"]) for n in sorted(stand_names)))
    L.append("# hips-line yaw vs the Trajectory frame (deg, circular mean): " + "; ".join("%s %.1f" % (n, allr[n]["bodyrel_hip"]) for n in ("M_Neutral_Stand_Idle_Loop", "M_Neutral_Crouch_Idle_Loop", "M_Neutral_Crouch_Loop_F") if n in allr))
    text = "\n".join(L)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n", encoding="utf-8")
    if a.json:
        json.dump({"refs": refs, "clips": allr}, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
