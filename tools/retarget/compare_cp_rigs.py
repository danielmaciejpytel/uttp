#!/usr/bin/env python3
"""Compare two Cyberpunk 2077 animRig JSONs (WolvenKit 9 'convert serialize'), e.g. player_man_skeleton vs
player_woman_skeleton, for the per-rig retarget.

Reports: bone counts, names / order / parents (0..72 = exported body bones, 73.. = helpers), the UTPP rig hash
(mmbake.rig_hash: FNV-1a 64 over names + parents of bones 0..72), local rest-pose differences per bone (translation m,
rotation deg, scale), model-space proportions (heights, widths, limb lengths), LOD start indices, parts (masks),
tracks / extra tracks and tags. Read-only; writes only the --json result.

Usage: python compare_cp_rigs.py --a <rig A .rig.json> --b <rig B .rig.json> [--json out.json]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
import mmbake_base as mb  # noqa: E402  (rig_hash, fk_model)

NB = 73


def cname(x):
    return x["$value"] if isinstance(x, dict) else str(x)


def load(path):
    rc = json.load(open(path, encoding="utf-8"))["Data"]["RootChunk"]
    names = [cname(b) for b in rc["boneNames"]]
    parents = [int(p) for p in rc["boneParentIndexes"]]
    P, Q, S = [], [], []
    for t in rc["boneTransforms"]:
        p, q, s = t["Translation"], t["Rotation"], t["Scale"]
        P.append((p["X"], p["Y"], p["Z"]))
        Q.append((q["i"], q["j"], q["k"], q["r"]))
        S.append((s["X"], s["Y"], s["Z"]))
    P, Q, S = np.array(P, float), np.array(Q, float), np.array(S, float)
    Q /= np.linalg.norm(Q, axis=1, keepdims=True)
    gp, gq = mb.fk_model(P[None], Q[None], parents)
    return dict(rc=rc, names=names, parents=parents, P=P, Q=Q, S=S, gp=gp[0], gq=gq[0])


def qangle(a, b):
    d = abs(float(np.dot(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def proportions(r):
    bi = {n: i for i, n in enumerate(r["names"])}
    g = lambda n: r["gp"][bi[n]]
    dist = lambda a, b: float(np.linalg.norm(g(a) - g(b)))
    out = {
        "hips_z_m": float(g("Hips")[2]),
        "spine3_z_m": float(g("Spine3")[2]),
        "neck_z_m": float(g("Neck")[2]),
        "head_z_m": float(g("Head")[2]),
        "eye_z_m": float((g("LeftEye")[2] + g("RightEye")[2]) / 2),
        "shoulder_width_arm_heads_m": dist("LeftArm", "RightArm"),
        "clavicle_len_m": (dist("LeftShoulder", "LeftArm") + dist("RightShoulder", "RightArm")) / 2,
        "hip_width_upleg_heads_m": dist("LeftUpLeg", "RightUpLeg"),
        "thigh_len_m": (dist("LeftUpLeg", "LeftLeg") + dist("RightUpLeg", "RightLeg")) / 2,
        "shin_len_m": (dist("LeftLeg", "LeftFoot") + dist("RightLeg", "RightFoot")) / 2,
        "foot_len_ankle_toe_m": (dist("LeftFoot", "LeftToeBase") + dist("RightFoot", "RightToeBase")) / 2,
        "ankle_z_m": float((g("LeftFoot")[2] + g("RightFoot")[2]) / 2),
        "upperarm_len_m": (dist("LeftArm", "LeftForeArm") + dist("RightArm", "RightForeArm")) / 2,
        "forearm_len_m": (dist("LeftForeArm", "LeftHand") + dist("RightForeArm", "RightHand")) / 2,
        "hand_len_wrist_middle1_m": (dist("LeftHand", "LeftHandMiddle1") + dist("RightHand", "RightHandMiddle1")) / 2,
        "hips_to_neck_m": dist("Hips", "Neck"),
        "head_above_arm_heads_m": float(g("Head")[2] - (g("LeftArm")[2] + g("RightArm")[2]) / 2),
    }
    hz, nz = out["hips_z_m"], out["neck_z_m"]
    out["spine_norm_heights"] = {n: round((float(g(n)[2]) - hz) / (nz - hz), 3) for n in ("Spine", "Spine1", "Spine2", "Spine3")}
    out["leg_len_m"] = out["thigh_len_m"] + out["shin_len_m"]
    return out


def parts_summary(rc):
    res = {}
    for p in rc.get("parts", []):
        res[cname(p["name"])] = {
            "tree": [(cname(t["rootBone"]), t.get("weight")) for t in p.get("treeBones", [])],
            "single": [cname(s.get("bone", s)) if isinstance(s, dict) else s for s in p.get("singleBones", [])],
            "rot_ms": [cname(b) for b in p.get("bonesWithRotationInModelSpace", [])],
            "mask_len": len(p.get("mask", [])),
        }
    return res


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(prog="compare_cp_rigs.py")
    ap.add_argument("--a", required=True, help="rig A json (e.g. player_man_skeleton.rig.json)")
    ap.add_argument("--b", required=True, help="rig B json (e.g. player_woman_skeleton.rig.json)")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    A, B = load(a.a), load(a.b)
    na, nb = A["names"], B["names"]
    res = {"a": a.a, "b": a.b, "a_json_sha256": sha256(a.a), "b_json_sha256": sha256(a.b)}
    res["bone_count"] = [len(na), len(nb)]
    res["names_0_72_identical"] = na[:NB] == nb[:NB]
    res["names_all_identical"] = na == nb
    res["names_only_in_a"] = sorted(set(na) - set(nb))
    res["names_only_in_b"] = sorted(set(nb) - set(na))
    res["name_order_diff_idx"] = [i for i in range(min(len(na), len(nb))) if na[i] != nb[i]]
    res["parents_0_72_identical"] = A["parents"][:NB] == B["parents"][:NB]
    res["parents_all_identical"] = A["parents"] == B["parents"]
    res["parents_diff_idx"] = [i for i in range(min(len(na), len(nb))) if A["parents"][i] != B["parents"][i]]
    ha = mb.rig_hash(na[:NB], A["parents"][:NB])
    hb = mb.rig_hash(nb[:NB], B["parents"][:NB])
    res["rig_hash_0_72"] = ["0x%016X" % ha, "0x%016X" % hb]
    res["rig_hash_equal"] = ha == hb
    # local rest pose per bone (by index; names identical is checked above)
    rows = []
    n = min(len(na), len(nb))
    for i in range(n):
        dt = float(np.linalg.norm(A["P"][i] - B["P"][i]))
        dr = qangle(A["Q"][i], B["Q"][i])
        ds = float(np.abs(A["S"][i] - B["S"][i]).max())
        la, lb = float(np.linalg.norm(A["P"][i])), float(np.linalg.norm(B["P"][i]))
        dms = float(np.linalg.norm(A["gp"][i] - B["gp"][i]))
        dmr = qangle(A["gq"][i], B["gq"][i])
        rows.append({"i": i, "name": na[i], "parent": A["parents"][i], "dt_local_m": round(dt, 5),
                     "len_a_m": round(la, 5), "len_b_m": round(lb, 5), "len_ratio_a_b": round(la / lb, 4) if lb > 1e-6 else None,
                     "drot_local_deg": round(dr, 3), "dscale_max": round(ds, 6),
                     "dpos_model_m": round(dms, 4), "drot_model_deg": round(dmr, 3)})
    res["rest_local_per_bone"] = rows
    body = rows[:NB]
    res["rest_summary_0_72"] = {
        "max_dt_local_m": max(r["dt_local_m"] for r in body),
        "max_drot_local_deg": max(r["drot_local_deg"] for r in body),
        "bones_drot_local_gt_1deg": [r["name"] for r in body if r["drot_local_deg"] > 1.0],
        "bones_dt_local_gt_1cm": [r["name"] for r in body if r["dt_local_m"] > 0.01],
        "max_dscale": max(r["dscale_max"] for r in body),
        "scale_non_unit_a": [na[i] for i in range(NB) if np.abs(A["S"][i] - 1).max() > 1e-4],
        "scale_non_unit_b": [nb[i] for i in range(NB) if np.abs(B["S"][i] - 1).max() > 1e-4],
    }
    pa, pb = proportions(A), proportions(B)
    res["proportions"] = {k: [pa[k], pb[k], (round(pa[k] / pb[k], 4) if isinstance(pa[k], float) and pb[k] else None)]
                          for k in pa}
    ra, rb = A["rc"], B["rc"]
    res["lod_start_indices"] = [ra["levelOfDetailStartIndices"], rb["levelOfDetailStartIndices"]]
    res["distance_lod_map"] = [ra["distanceCategoryToLodMap"], rb["distanceCategoryToLodMap"]]
    sa, sb = parts_summary(ra), parts_summary(rb)
    res["parts"] = {"a": sorted(sa), "b": sorted(sb), "only_a": sorted(set(sa) - set(sb)), "only_b": sorted(set(sb) - set(sa)),
                    "differ": sorted(k for k in set(sa) & set(sb) if sa[k] != sb[k])}
    res["track_names_identical"] = [cname(t) for t in ra["trackNames"]] == [cname(t) for t in rb["trackNames"]]
    res["reference_tracks_identical"] = ra["referenceTracks"] == rb["referenceTracks"]
    res["extra_tracks"] = [[(cname(t["name"]), t.get("referenceValue")) for t in ra["rigExtraTracks"]],
                           [(cname(t["name"]), t.get("referenceValue")) for t in rb["rigExtraTracks"]]]
    res["tags"] = [[cname(t) for t in ra["tags"]["tags"]], [cname(t) for t in rb["tags"]["tags"]]]
    out = json.dumps(res, indent=1)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            fh.write(out + "\n")
    short = {k: v for k, v in res.items() if k != "rest_local_per_bone"}
    print(json.dumps(short, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
