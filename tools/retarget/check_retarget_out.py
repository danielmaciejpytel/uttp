#!/usr/bin/env python3
"""Independent checks of a retarget output dir (retarget_batch.py --out-dir) for pilot / batch review, offline (numpy).

Per clip (report.json of the dir + <clip>.utppclip.json + <meta-dir>/<clip>.json):
  status, FK (check A) and root-motion pass from the report, FBX key layout;
  structure: frames == meta num_frames, 73 bones x 10 values per frame, bone_names == rig 0..72, rig_bone_count;
  finite values (own scan, independent of the report 'nan' flag);
  Hips: model-space Z min/max, max |Hips Z - CP rest Hips Z| against --vert-tol (jump/land clips) and the
  vertical-root flag vertical_root_in_hips (tightened 2026-10-13): the clip is in the RTZ scope (purpose group of
  --clip-map: jump_land + orphan jump_f, traversal, from_traversal) and max |Hips Z - rest| > --vert-tol, i.e. the
  source root height was carried into Hips and the clip needs the re-retarget with the root vertical in RTZ); clips without a clip-map entry keep the old rule. Backward-compatible field
  vertical_root_in_hips_legacy = the old rule (Hips Z < 0.2 m or > rest + 0.6 m, any clip). --rtz-out writes the
  RTZ list (Research/inventory/sourcepack-rtz-clips.json);
  ankles: min model-space Z; foot slide from the report (ankle mean cm/s during contact > 0.5, pivot = min(ankle, toe),
  excess over the scaled source; flag > 5 cm/s) + contact source of the meta (utpp_contacts.source);
  posture (posture_offsets.py pose_metrics, raw and with posture_offsets.json applied in memory): shoulder width 3D,
  head above shoulders, min hand -> thigh clearance;
  curves: every list-valued curve of the meta (lower-case name) must be in the clip with the same values and one value
  per frame (curves_missing / curves_changed / curves_bad_length); --require-all-curves makes a gap an exit code 1
  (pipeline with retarget_batch.py --keep-all-curves).
Full-pose pairs (--pair CLIP:BASE, e.g. an additive AO / overlay asset and its base, or --pair-set NAME = the list
retarget_check.pair_sets.NAME of sourcepack-names.json): the retargeted asset must be a full
pose on the CP rig (needed for later delta computation), not a near-zero delta: Hips Z and ankle Z in a standing/crouching
range, lower-body rotation close to the base, and the mean / max upper-body angle to the base reported.
Knee gate (--knee-cache DIR = knee_metrics.py extract-ue npz cache of the UE source; bake-blocking, exit code 2):
per clip with a cache entry, knee_metrics.leg_metrics of UE and CP, both legs; a clip fails when
  flip:   a frame step with UE and CP flex > --knee-flex-gate (20 deg [P]) on both frames where the CP knee direction vs
          toes (theta) changes by more than --knee-flip-deg (30 deg/frame [P]) MORE than the UE source does
          (|dtheta_cp| - |dtheta_ue| > flip; relative to the source because UE itself reaches 31 deg/frame in 2 clips);
  hinge:  p95 over gated frames of the CP hinge off-plane (off_bone: bind lateral axis carried by the shin vs the
          hip-knee-ankle plane normal) > --knee-hinge-deg (10 deg [P]; UE fitted-hinge max 6.9 deg measured);
  seam:   loop clips (cache loop flag): CP flex or theta jump N-2 -> 0 exceeds the UE jump by more than --knee-flip-deg.
Clips without a cache entry are listed in summary.knee.not_measured (not a failure).
This script only reads.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
import mmbake_base as mb  # noqa: E402
import posture_offsets as po  # noqa: E402

RIG = os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json")
NAMES = os.path.join(ROOT, "Research", "inventory", "sourcepack-names.json")
CLIP_MAP = os.path.join(ROOT, "Research", "inventory", "sourcepack-clip-map.json")
# RTZ scope: clip-map purpose group -> RTZ group; the orphan Jump_F land / start clips (subgroup jump_f)
# belong to the jump/land family
RTZ_GROUPS = ("jump_land", "traversal", "from_traversal")
RTZ_ORPHAN_SUBGROUPS = {"jump_f": "jump_land"}
NB = 73
LOWER = ("Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot")
UPPER = ("Spine", "Spine1", "Spine2", "Spine3", "Neck", "Head", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
         "RightShoulder", "RightArm", "RightForeArm", "RightHand")


def qangle(a, b):
    d = np.abs(np.sum(a * b, axis=-1))
    return np.degrees(2.0 * np.arccos(np.clip(d, 0.0, 1.0)))


def load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def clip_arrays(c):
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    return B


def load_purposes(path):
    """clip name -> (purpose group, subgroup) of the clip map, {} when the map is missing."""
    if not path or not os.path.isfile(path):
        return {}
    return {c["name"]: (c["purpose"]["group"], c["purpose"]["subgroup"]) for c in load(path)["clips"]}


def rtz_group(purpose):
    """RTZ group of a clip-map purpose (group, subgroup), None outside the RTZ scope."""
    if purpose is None:
        return None
    g, sub = purpose
    if g in RTZ_GROUPS:
        return g
    if g == "orphan":
        return RTZ_ORPHAN_SUBGROUPS.get(sub)
    return None


def check_clip(name, out_dir, meta_dir, rep, names, parents, rest_hips_z, offsets, vert_tol, purpose=None):
    r = {"clip": name}
    if purpose is not None:
        r["purpose_group"] = list(purpose)
    r["rtz_group"] = rtz_group(purpose)
    rr = rep.get(name)
    r["status"] = rr.get("status") if rr else "missing_in_report"
    if rr and rr.get("status") != "ok":
        r["error"] = rr.get("error")
        return r
    path = os.path.join(out_dir, name + ".utppclip.json")
    if not os.path.isfile(path):
        r["status"] = "missing_clip_file"
        return r
    c = load(path)
    meta = load(os.path.join(meta_dir, name + ".json")) if os.path.isfile(os.path.join(meta_dir, name + ".json")) else {}
    B = clip_arrays(c)
    n = B.shape[0]
    r["frames"] = n
    r["frames_meta"] = meta.get("num_frames")
    r["frames_ok"] = meta.get("num_frames") is not None and int(meta["num_frames"]) == n == int(c.get("num_frames", -1))
    r["bones"] = int(B.shape[1]) if B.ndim == 3 else None
    r["bones_ok"] = B.ndim == 3 and B.shape[1] == NB and B.shape[2] == 10 and list(c["bone_names"]) == names
    r["rig_bone_count"] = c.get("rig_bone_count")
    r["rig"] = c.get("rig")
    r["finite"] = bool(np.isfinite(B).all())
    if rr:
        r["fk_pass"] = rr["check_fk_vs_blender"]["pass"]
        r["fk_max_pos_err_m"] = rr["check_fk_vs_blender"]["max_pos_err_m"]
        r["root_pass"] = rr["root_motion"]["pass"]
        r["root_disp_err_m"] = rr["root_motion"]["disp_err_m"]
        r["traj_disp_m"] = rr["root_motion"]["trajectory_disp_m"]
        r["fbx_keys_ok"] = rr["fbx_keys"]["ok"]
        r["fbx_keys"] = [rr["fbx_keys"].get("keys"), rr["fbx_keys"].get("expected_keys")]
        s = rr["foot_slide"]
        r["slide_ankle_cm_s"] = [s[k]["cp"]["mean_cm_s"] for k in ("l", "r")]
        r["slide_src_cm_s"] = [s[k]["source_scaled"]["mean_cm_s"] for k in ("l", "r")]
        r["slide_pivot_cm_s"] = [s[k]["cp_pivot"]["mean_cm_s"] for k in ("l", "r")]
        r["slide_contact_frames"] = [s[k]["cp"]["frames"] for k in ("l", "r")]
        r["slide_flag_gt5"] = rr["foot_slide_flag"]
        r["slide_pivot_flag_gt5"] = rr["foot_slide_pivot_flag"]
        r["slide_excess_flag_gt5"] = rr["foot_slide_excess_vs_source_flag"]
        r["ik_clamped_leg_frames"] = rr["ik_clamped_leg_frames"]
        r["hip_drop_max_cm"] = rr.get("hip_drop_max_cm")
    r["contact_source"] = (meta.get("utpp_contacts") or {}).get("source", "sourcepack_export_meta" if "contact_l" in {k.lower() for k in (meta.get("curves") or {})} else "none")
    mc = {}
    for k, v in (meta.get("curves") or {}).items():
        if isinstance(v, list):
            mc.setdefault(k.lower(), v)
    oc = c.get("curves") or {}
    r["curves_meta"] = len(mc)
    r["curves_out"] = len(oc)
    r["curves_missing"] = sorted(set(mc) - set(oc))
    r["curves_changed"] = sorted(k for k in set(mc) & set(oc) if mc[k] != oc[k])
    r["curves_bad_length"] = sorted(k for k, v in oc.items() if not isinstance(v, list) or len(v) != n)
    r["curves_ok"] = not (r["curves_missing"] or r["curves_changed"] or r["curves_bad_length"])
    if not r["finite"] or not r["bones_ok"]:
        return r
    P, Q = B[:, :, 0:3], B[:, :, 3:7]
    gp, gq = mb.fk_model(P, Q, parents)
    bi = {nm: i for i, nm in enumerate(names)}
    hz = gp[:, bi["Hips"], 2]
    r["hips_z_min_max_m"] = [round(float(hz.min()), 4), round(float(hz.max()), 4)]
    r["hips_z_dev_max_m"] = round(float(np.abs(hz - rest_hips_z).max()), 4)
    r["hips_z_dev_within_tol"] = bool(r["hips_z_dev_max_m"] <= vert_tol)   # criterion meant for jump/land clips
    # legacy rule (backward-compatible field): hips far below a deep crouch or far above a jump apex of the pelvis
    r["vertical_root_in_hips_legacy"] = bool(hz.min() < 0.2 or hz.max() > rest_hips_z + 0.6)
    # tightened rule: RTZ scope of the clip map + tolerance; no clip-map entry -> legacy rule
    if purpose is None:
        r["vertical_root_rule"] = "legacy (no clip-map entry)"
        r["vertical_root_in_hips"] = r["vertical_root_in_hips_legacy"]
    else:
        r["vertical_root_rule"] = "rtz_scope" if r["rtz_group"] else "outside_rtz_scope"
        r["vertical_root_in_hips"] = bool(r["rtz_group"] is not None and r["hips_z_dev_max_m"] > vert_tol)
    r["ankle_z_min_m"] = round(float(min(gp[:, bi["LeftFoot"], 2].min(), gp[:, bi["RightFoot"], 2].min())), 4)
    pm_raw = po.pose_metrics(gp, bi)
    Qo = po.apply_offsets_array(Q, names, offsets)
    Po = po.apply_offsets_P(P, names, offsets)
    gpo, _ = mb.fk_model(Po, Qo, parents)
    pm_off = po.pose_metrics(gpo, bi)
    keys = ("width_3d_m", "head_above_shoulders_m", "min_hand_Left_to_thigh_cm", "min_hand_Right_to_thigh_cm")
    r["posture_raw"] = {k: round(pm_raw[k], 4) for k in keys}
    r["posture_offsets_applied"] = {k: round(pm_off[k], 4) for k in keys}
    r["_gp"], r["_gq"] = gp, gq
    return r


def pair_check(a, b, bi):
    ga, qa, gb, qb = a["_gp"], a["_gq"], b["_gp"], b["_gq"]
    fa, fb = 0, 0   # first frame of each (single poses / additive base frame 0)
    low = [float(qangle(qa[fa, bi[n]], qb[fb, bi[n]])) for n in LOWER]
    up = [float(qangle(qa[fa, bi[n]], qb[fb, bi[n]])) for n in UPPER]
    hz = float(ga[fa, bi["Hips"], 2])
    az = float(min(ga[fa, bi["LeftFoot"], 2], ga[fa, bi["RightFoot"], 2]))
    res = {"clip": a["clip"], "base": b["clip"], "frame": [fa, fb], "hips_z_m": round(hz, 4),
           "hips_z_base_m": round(float(gb[fb, bi["Hips"], 2]), 4), "ankle_z_min_m": round(az, 4),
           "lower_body_max_angle_to_base_deg": round(max(low), 3), "upper_body_mean_angle_to_base_deg": round(sum(up) / len(up), 3),
           "upper_body_max_angle_to_base_deg": round(max(up), 3)}
    res["full_pose"] = bool(0.3 < hz < 1.3 and az < 0.3 and abs(hz - res["hips_z_base_m"]) < 0.15)
    return res


def rtz_summary(rows, vert_tol):
    """Per RTZ group: clips in scope, checked (status ok + Hips measured), over --vert-tol (= RTZ list)."""
    out = {"vert_tol_m": vert_tol, "groups": {}}
    for r in rows.values():
        g = r.get("rtz_group")
        if not g:
            continue
        s = out["groups"].setdefault(g, {"in_scope": 0, "checked": 0, "over_tol": 0})
        s["in_scope"] += 1
        if r.get("status") == "ok" and r.get("hips_z_dev_max_m") is not None:
            s["checked"] += 1
            s["over_tol"] += 1 if r["hips_z_dev_max_m"] > vert_tol else 0
    out["groups"] = dict(sorted(out["groups"].items()))
    out["over_tol_total"] = sum(s["over_tol"] for s in out["groups"].values())
    return out


def rel(p):
    try:
        return os.path.relpath(os.path.abspath(p), ROOT).replace("\\", "/")
    except ValueError:      # other drive
        return os.path.abspath(p).replace("\\", "/")


def write_rtz_list(a, rows, rest_hips_z, rep_path):
    """RTZ list: clips of the RTZ scope whose max |Hips Z - CP rest Hips Z| > --vert-tol."""
    keys = ("hips_z_min_max_m", "hips_z_dev_max_m", "vertical_root_in_hips_legacy")
    scope = [r for r in rows.values() if r.get("rtz_group")]
    sel = [r for r in scope if r.get("vertical_root_in_hips")]
    outside = [r for r in rows.values() if not r.get("rtz_group") and r.get("vertical_root_in_hips_legacy")]
    out = {
        "format": "utpp-gaspals-rtz-clips", "version": 1,
        "generator": "Source/tools/retarget/check_retarget_out.py --rtz-out",
        "licence": "Licensed animation data",
        "rule": "clips of the clip-map purpose groups %s (+ orphan subgroups %s -> jump_land) with max |Hips Z - CP "
                "rest Hips Z| > %g m in any frame: the source root height was carried "
                "into Hips -> re-retarget and bake with the root vertical in RTZ"
                % (", ".join(RTZ_GROUPS), ", ".join(sorted(RTZ_ORPHAN_SUBGROUPS)), a.vert_tol),
        "source": {"out_dir": rel(a.out_dir), "report": rel(rep_path), "meta_dir": rel(a.meta_dir),
                   "clip_map": rel(a.clip_map), "rig": rel(a.rig), "rest_hips_z_m": round(rest_hips_z, 4),
                   "vert_tol_m": a.vert_tol},
        "counts": dict(sorted(collections.Counter(r["rtz_group"] for r in sel).items())),
        "count": len(sel),
        "scope": rtz_summary(rows, a.vert_tol)["groups"],
        "in_scope_within_tol": sorted(r["clip"] for r in scope if r.get("status") == "ok"
                                      and r.get("hips_z_dev_max_m") is not None and not r.get("vertical_root_in_hips")),
        "in_scope_not_checked": sorted([r["clip"], r.get("status")] for r in scope if r.get("hips_z_dev_max_m") is None),
        "outside_scope_flagged_by_legacy_rule": [
            dict({"clip": r["clip"], "purpose_group": r.get("purpose_group")}, **{k: r.get(k) for k in keys[:2]})
            for r in sorted(outside, key=lambda x: x["clip"])],
        "clips": [dict({"clip": r["clip"], "rtz_group": r["rtz_group"], "purpose_group": r.get("purpose_group")},
                       **{k: r.get(k) for k in keys}) for r in sorted(sel, key=lambda x: (x["rtz_group"], x["clip"]))],
    }
    os.makedirs(os.path.dirname(os.path.abspath(a.rtz_out)), exist_ok=True)
    tmp = a.rtz_out + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    os.replace(tmp, a.rtz_out)
    print("RTZ_LIST %d clips %s -> %s" % (len(sel), json.dumps(out["counts"]), a.rtz_out))


def knee_gate(a, want, out_dir):
    """Bake-blocking knee gate (see module doc). Returns (summary dict, {clip: row})."""
    sys.path.insert(0, HERE)
    import knee_metrics as km
    rigs, rows, failed, not_measured = {}, {}, [], []
    for clip in want:
        npz = os.path.join(a.knee_cache, clip + ".npz")
        cpp = os.path.join(out_dir, clip + ".utppclip.json")
        if not os.path.isfile(npz) or not os.path.isfile(cpp):
            not_measured.append(clip)
            continue
        z = dict(np.load(npz))
        gp, gq, rp, rq, _ = km.load_cp_clip(cpp, km.ROOT_DEFAULT, rigs)
        n = z["pos"].shape[0]
        if gp.shape[0] != n:
            not_measured.append(clip)
            continue
        mu = km.leg_metrics(z["pos"], z["quat"], z["rest_pos"], z["rest_quat"])
        mc = km.leg_metrics(gp, gq, rp, rq)
        row, why = {}, []
        for side in ("l", "r"):
            g = (mu[side]["flex"] > a.knee_flex_gate) & (mc[side]["flex"] > a.knee_flex_gate)
            gg = np.zeros_like(g)
            gg[1:] = g[1:] & g[:-1]
            ex = np.where(gg, np.abs(mc[side]["dtheta"]) - np.abs(np.nan_to_num(mu[side]["dtheta"])), np.nan)
            flips = [int(i) for i in np.nonzero(ex > a.knee_flip_deg)[0]]
            off = np.where(mc[side]["flex"] > a.knee_flex_gate, mc[side]["off_bone"], np.nan)
            off = off[np.isfinite(off)]
            p95 = float(np.percentile(off, 95)) if off.size else 0.0
            r = {"flip_frames": flips[:20], "flip_count": len(flips),
                 "dtheta_excess_max": None if not np.isfinite(ex).any() else round(float(np.nanmax(ex)), 2),
                 "off_bone_p95": round(p95, 2)}
            if flips:
                why.append("%s flip %d" % (side, len(flips)))
            if p95 > a.knee_hinge_deg:
                why.append("%s hinge p95 %.1f" % (side, p95))
            if bool(z.get("loop", False)) and n >= 3:
                su, sc = km.seam(mu[side], n), km.seam(mc[side], n)
                r["seam_flex_excess"] = round(sc["flex_jump"] - su["flex_jump"], 2)
                r["seam_theta_excess"] = round((sc["theta_jump"] or 0.0) - (su["theta_jump"] or 0.0), 2)
                if r["seam_flex_excess"] > a.knee_flip_deg or r["seam_theta_excess"] > a.knee_flip_deg:
                    why.append("%s seam flex+%.1f theta+%.1f" % (side, r["seam_flex_excess"], r["seam_theta_excess"]))
            row[side] = r
        row["fail"] = why
        rows[clip] = row
        if why:
            failed.append([clip, why])
    summ = {"cache": a.knee_cache, "flex_gate_deg": a.knee_flex_gate, "flip_deg": a.knee_flip_deg,
            "hinge_deg": a.knee_hinge_deg, "measured": len(rows), "not_measured": not_measured,
            "failed": failed, "pass": not failed}
    return summ, rows


def main(argv=None):
    ap = argparse.ArgumentParser(prog="check_retarget_out.py")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--meta-dir", required=True)
    ap.add_argument("--report", default=None, help="default <out-dir>/report.json")
    ap.add_argument("--clips", default="", help="comma separated; default every clip of the report")
    ap.add_argument("--list", default="", help="clip list file (one per line, # comments)")
    ap.add_argument("--pair", action="append", default=[], help="CLIP:BASE full-pose check (repeatable)")
    ap.add_argument("--pair-set", action="append", default=[],
                    help="named list of CLIP:BASE pairs, retarget_check.pair_sets.<NAME> of --names (repeatable)")
    ap.add_argument("--names", default=NAMES, help="source pack names data file (default %(default)s)")
    ap.add_argument("--require-all-curves", action="store_true",
                    help="exit 1 if a clip lacks a meta curve or changed it (pipeline with --keep-all-curves)")
    ap.add_argument("--offsets", default=os.path.join(HERE, "posture_offsets.json"))
    ap.add_argument("--vert-tol", type=float, default=0.25, help="m, max |Hips Z - rest| (jump/land clips)")
    ap.add_argument("--clip-map", default=CLIP_MAP,
                    help="source pack clip map (purpose groups -> RTZ scope of vertical_root_in_hips; default %(default)s)")
    ap.add_argument("--rtz-out", default=None,
                    help="write the RTZ list here (Research/inventory/sourcepack-rtz-clips.json): RTZ-scope clips with "
                         "max |Hips Z - rest| > --vert-tol")
    ap.add_argument("--rig", default=RIG, help="target rig json of the clips (default %(default)s; male V: "
                    "Extracted/game/json/vanilla-man/player_man_skeleton.rig.json); clips whose 'rig' field differs are "
                    "listed in summary.rig_mismatch")
    ap.add_argument("--json", default=None, help="write the results here")
    ap.add_argument("--knee-cache", default=None, help="knee_metrics.py UE npz cache: enables the bake-blocking knee gate (exit 2)")
    ap.add_argument("--knee-flex-gate", type=float, default=20.0, help="deg [P] runtime-session proposal")
    ap.add_argument("--knee-flip-deg", type=float, default=30.0, help="deg/frame over the UE source [P] runtime-session proposal")
    ap.add_argument("--knee-hinge-deg", type=float, default=10.0, help="deg p95 CP hinge off-plane [P] (UE max 6.9 measured)")
    a = ap.parse_args(argv)
    names, parents = mb.load_rig(a.rig, NB)
    rig = load(a.rig)["Data"]["RootChunk"]
    rig_stem = os.path.basename(a.rig).split(".")[0]
    rest_hips_z = float(rig["boneTransforms"][names.index("Hips")]["Translation"]["Z"])
    rep_path = a.report or os.path.join(a.out_dir, "report.json")
    rep_all = load(rep_path)
    rep = {x["clip"]: x for x in rep_all["clips"]}
    want = [c.strip() for c in a.clips.split(",") if c.strip()]
    if a.list:
        with open(a.list, encoding="utf-8-sig") as fh:
            want += [ln.split("#", 1)[0].strip() for ln in fh if ln.split("#", 1)[0].strip()]
    want = want or sorted(rep)
    if a.pair_set:
        sets = load(a.names)["retarget_check"]["pair_sets"]
        for s in a.pair_set:
            if s not in sets:
                ap.error("--pair-set %s: not in %s (known: %s)" % (s, a.names, ", ".join(sorted(sets))))
            a.pair += [p for p in sets[s] if p not in a.pair]
    for p in a.pair:
        for x in p.split(":"):
            if x not in want:
                want.append(x)
    offsets = load(a.offsets)
    purposes = load_purposes(a.clip_map)
    if not purposes:
        print("WARNING clip map %s missing: vertical_root_in_hips uses the legacy rule for every clip" % a.clip_map)
    if a.rtz_out and not purposes:
        ap.error("--rtz-out needs the clip map (--clip-map %s)" % a.clip_map)
    rows = {w: check_clip(w, a.out_dir, a.meta_dir, rep, names, parents, rest_hips_z, offsets, a.vert_tol,
                          purposes.get(w)) for w in want}
    bi = {nm: i for i, nm in enumerate(names)}
    pairs = []
    for p in a.pair:
        x, y = p.split(":")
        if "_gp" in rows.get(x, {}) and "_gp" in rows.get(y, {}):
            pairs.append(pair_check(rows[x], rows[y], bi))
        else:
            pairs.append({"clip": x, "base": y, "full_pose": None, "why": "clip or base missing / failed"})
    for r in rows.values():
        r.pop("_gp", None)
        r.pop("_gq", None)
    ok = [r for r in rows.values() if r.get("status") == "ok"]
    summary = {
        "clips": len(rows), "ok": len(ok), "failed": [r["clip"] for r in rows.values() if r.get("status") != "ok"],
        "structure_ok": sum(1 for r in ok if r.get("frames_ok") and r.get("bones_ok")),
        "finite": sum(1 for r in ok if r.get("finite")),
        "fk_pass": sum(1 for r in ok if r.get("fk_pass")), "root_pass": sum(1 for r in ok if r.get("root_pass")),
        "fbx_keys_ok": sum(1 for r in ok if r.get("fbx_keys_ok")),
        "rig": rig_stem,
        "rig_mismatch": [r["clip"] for r in ok if r.get("rig") != rig_stem],
        "slide_flag_gt5": [r["clip"] for r in ok if r.get("slide_flag_gt5")],
        "slide_pivot_flag_gt5": [r["clip"] for r in ok if r.get("slide_pivot_flag_gt5")],
        "slide_excess_flag_gt5": [r["clip"] for r in ok if r.get("slide_excess_flag_gt5")],
        "slide_not_measured_no_contact": [r["clip"] for r in ok if not any(r.get("slide_contact_frames") or [0])],
        "vertical_root_in_hips": [r["clip"] for r in ok if r.get("vertical_root_in_hips")],
        "vertical_root_in_hips_legacy": [r["clip"] for r in ok if r.get("vertical_root_in_hips_legacy")],
        "rtz": rtz_summary(rows, a.vert_tol),
        "pairs_full_pose": [[p["clip"], p["base"], p.get("full_pose")] for p in pairs],
        "curves_all_present": sum(1 for r in ok if r.get("curves_ok")),
        "curves_gaps": [[r["clip"], {k: r[k] for k in ("curves_missing", "curves_changed", "curves_bad_length") if r.get(k)}]
                        for r in ok if "curves_ok" in r and not r["curves_ok"]],
    }
    knee_rows = None
    if a.knee_cache:
        summary["knee"], knee_rows = knee_gate(a, [w for w in want if rows.get(w, {}).get("status") == "ok"], a.out_dir)
    print(json.dumps(summary, indent=1))
    hdr = "clip | st | frames(meta) | bones | fin | fk | root | keys | hipsZ min/max (dev) | ankle zmin | slide L/R [pivot] (src) contact | posture raw->off width/head/handL/handR"
    print(hdr)
    for r in rows.values():
        if r.get("status") != "ok":
            print("%s | %s %s" % (r["clip"], r.get("status"), r.get("error", "")))
            continue
        po_r, po_o = r.get("posture_raw") or {}, r.get("posture_offsets_applied") or {}
        print("%s | ok | %s(%s) | %s | %s | %s | %s | %s %s | %s (%s) | %s | %s [%s] (%s) %s | %s/%s/%s/%s -> %s/%s/%s/%s" % (
            r["clip"], r["frames"], r["frames_meta"], r["bones"], r["finite"], r.get("fk_pass"), r.get("root_pass"),
            r.get("fbx_keys_ok"), r.get("fbx_keys"), r.get("hips_z_min_max_m"), r.get("hips_z_dev_max_m"), r.get("ankle_z_min_m"),
            r.get("slide_ankle_cm_s"), r.get("slide_pivot_cm_s"), r.get("slide_src_cm_s"), r.get("contact_source"),
            po_r.get("width_3d_m"), po_r.get("head_above_shoulders_m"), po_r.get("min_hand_Left_to_thigh_cm"), po_r.get("min_hand_Right_to_thigh_cm"),
            po_o.get("width_3d_m"), po_o.get("head_above_shoulders_m"), po_o.get("min_hand_Left_to_thigh_cm"), po_o.get("min_hand_Right_to_thigh_cm")))
    for p in pairs:
        print("PAIR", json.dumps(p))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump({"summary": summary, "clips": list(rows.values()), "pairs": pairs, "knee": knee_rows,
                       "licence": "Licensed animation data"}, fh, indent=1)
    if a.rtz_out:
        write_rtz_list(a, rows, rest_hips_z, rep_path)
    if a.knee_cache and not summary["knee"]["pass"]:
        print("KNEE_GATE_FAIL %d clip(s): bake blocked" % len(summary["knee"]["failed"]))
        return 2
    if a.require_all_curves and summary["curves_gaps"]:
        print("CURVES_GAPS %d clip(s) lack or changed meta curves" % len(summary["curves_gaps"]))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
