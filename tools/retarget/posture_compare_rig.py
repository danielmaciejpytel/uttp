#!/usr/bin/env python3
"""Posture metrics of a retargeted idle clip vs the game reference idle of the same CP rig, with and without a posture
offsets file applied: evidence for whether a rig needs posture offsets (posture_offsets.json was
measured for the female V only).

For each --case NAME=<rig json>,<clips dir>,<reference glb> it prints / writes posture_offsets.pose_metrics (shoulder width,
head above shoulders, clavicle / neck elevation, hand clearance) for: the rig rest pose, the reference idle (--ref-anim, glb
animation on that rig), the retargeted clip --clip raw, and the clip with --offsets applied in memory, plus the
deltas clip - reference. Read-only except --json. Clip names come from the
command line (no source pack names in the repo).

Usage: python posture_compare_rig.py --clip <name> --ref-anim fpp_idle_stand --offsets posture_offsets.json
           --case woman=<rig>,<dir>,<glb> --case man=<rig>,<dir>,<glb> [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
import mmbake_base as mb  # noqa: E402
import posture_offsets as po  # noqa: E402

NB = 73
KEYS = ("width_x_m", "width_3d_m", "head_above_shoulders_m", "clavicle_elev_deg", "clavicle_elev_R_deg", "neck_elev_deg",
        "min_hand_Left_to_thigh_cm", "min_hand_Right_to_thigh_cm")


def rest_pose(rig_path):
    rc = json.load(open(rig_path, encoding="utf-8"))["Data"]["RootChunk"]
    P = np.array([[t["Translation"][k] for k in "XYZ"] for t in rc["boneTransforms"][:NB]], float)[None]
    Q = np.array([[t["Rotation"][k] for k in "ijkr"] for t in rc["boneTransforms"][:NB]], float)[None]
    return P, Q


def metrics(P, Q, parents, bi):
    gp, _ = mb.fk_model(P, Q, parents)
    m = po.pose_metrics(gp, bi)
    return {k: round(float(m[k]), 4) for k in KEYS}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="posture_compare_rig.py")
    ap.add_argument("--clip", required=True, help="retargeted idle clip name (<dir>/<clip>.utppclip.json)")
    ap.add_argument("--ref-anim", default="fpp_idle_stand", help="reference glb animation name")
    ap.add_argument("--offsets", default=os.path.join(HERE, "posture_offsets.json"))
    ap.add_argument("--case", action="append", required=True, help="NAME=<rig json>,<clips dir>,<reference glb>")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    off = json.load(open(a.offsets, encoding="utf-8"))
    out = {"clip": a.clip, "ref_anim": a.ref_anim, "offsets": os.path.basename(a.offsets), "cases": {},
           "licence": "Licensed animation data"}
    for case in a.case:
        name, rest = case.split("=", 1)
        rig, cdir, glb = rest.split(",")
        names, parents = mb.load_rig(rig, NB)
        bi = {n: i for i, n in enumerate(names)}
        res = {"rig": os.path.basename(rig), "clips_dir": cdir, "glb": os.path.basename(glb)}
        P, Q = rest_pose(rig)
        res["rest"] = metrics(P, Q, parents, bi)
        po.REF_GLB = glb   # read_ref reads this module global
        Pi, Qi = po.read_ref(a.ref_anim, names)
        n = Pi.shape[0] - 1 if Pi.shape[0] > 40 else Pi.shape[0]   # same frame rule as posture_offsets.py metrics
        res["ref"] = metrics(Pi[:n], Qi[:n], parents, bi)
        res["ref_frames"] = int(n)
        c, B = po.read_clip(os.path.join(cdir, a.clip + ".utppclip.json"))
        if c.get("rig") != os.path.basename(rig).split(".")[0]:
            raise SystemExit("clip %s rig %s != %s" % (a.clip, c.get("rig"), rig))
        n = B.shape[0] - 1 if "_Loop" in a.clip else B.shape[0]
        Pc, Qc = B[:n, :, 0:3], B[:n, :, 3:7]
        res["clip_frames"] = int(n)
        res["clip_raw"] = metrics(Pc, Qc, parents, bi)
        res["clip_offsets"] = metrics(po.apply_offsets_P(Pc, names, off), po.apply_offsets_array(Qc, names, off), parents, bi)
        res["delta_raw_minus_ref"] = {k: round(res["clip_raw"][k] - res["ref"][k], 4) for k in KEYS}
        res["delta_offsets_minus_ref"] = {k: round(res["clip_offsets"][k] - res["ref"][k], 4) for k in KEYS}
        out["cases"][name] = res
    rows = [("rest", "rest"), ("ref", "REF idle"), ("clip_raw", "clip raw"), ("clip_offsets", "clip+offsets"),
            ("delta_raw_minus_ref", "raw-REF"), ("delta_offsets_minus_ref", "offs-REF")]
    hdr = "metric".ljust(28) + "".join(("%s %s" % (cn, lab))[:18].rjust(19) for cn in out["cases"] for _, lab in rows)
    print(hdr)
    for k in KEYS:
        print(k.ljust(28) + "".join("%19.4f" % out["cases"][cn][key][k] for cn in out["cases"] for key, _ in rows))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump(out, fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
