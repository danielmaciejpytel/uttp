#!/usr/bin/env python3
"""Foot-contact curves for the source pack export.

source pack clips carry FootSpeed_L/R (cm/s, AM_FootSpeed_* motion-extractor modifiers) but no contact_l/r curves; the
retarget foot-slide metric (retarget_batch.py: ankle speed while contact > 0.5) and mmbake need contact_l/r.

  validate -> on the clips shared with Animations/sourcepack/SourcePack_export (clip-map contact policy "gasp58_export_meta", same frame
              count) derive contact = FootSpeed < threshold and compare with the source pack 5.8 contact_l/r (binary at 0.5):
              per-frame agreement and touchdown timing error (frames; touchdown = rising edge after frame 2, a contact
              active at the clip start is not a touchdown), threshold sweep (default 20..80 cm/s step 5),
              separately for Stops (name contains contacts.stop_contains of Research/inventory/sourcepack-names.json) and
              the rest. Writes a JSON report. Derived contacts are accepted when the
              agreement at the build thresholds (--review-threshold / --review-threshold-stops, default 60 / 80 cm/s =
              run_retarget_woman.cmd) is >= --accept-agreement (0.95); every shared clip with a touchdown
              error > 2 frames or a missed touchdown at those thresholds is listed in --review-out
              (Research/inventory/sourcepack-contacts-review.json). The build prefers source pack 5.8 contacts wherever the
              clip map has them (policy gasp58_export_meta), so the review list mainly qualifies the derived contacts
              of clips without a source pack 5.8 counterpart.
  build    -> merged meta dir for retarget_batch.py --meta-dir: every <clip>.json of the source pack export meta dir that has
              an FBX, copied with curves contact_l/contact_r added:
                gasp58  : contact_l/r of Animations/sourcepack/SourcePack_export/meta/<clip>.json when the clip map policy is
                          "gasp58_export_meta" and the frame count matches;
                derived : 1.0 where FootSpeed_<side> < threshold (Stops: --threshold-stops), else 0.0;
                none    : no FootSpeed curve (single poses, AO, ...) -> no contact curve (slide metric not applicable).
              Provenance per clip in meta['utpp_contacts'] and in <out>/contacts_build.json. The export dir is not modified.

Outputs stay outside Source/.
Requires numpy.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
CLIP_MAP = os.path.join(ROOT, "Research", "inventory", "sourcepack-clip-map.json")
CURVES = os.path.join(ROOT, "Extracted", "sourcepack", "sourcepack", "anim_curves.json")
SOURCEPACK_META = os.path.join(ROOT, "Animations", "sourcepack", "SourcePack_export", "meta")
EXPORT = os.path.join(ROOT, "Animations", "sourcepack", "als_export")
NAMES = os.path.join(ROOT, "Research", "inventory", "sourcepack-names.json")
FPS = 30.0
POLICY_KEY = "contact_l/contact_r"
SIDES = (("l", "FootSpeed_L"), ("r", "FootSpeed_R"))
TD_TOL_FRAMES = 2
START_EDGE_FRAMES = 2       # events(): rising edges at frames 0..2 are the clip start, not touchdowns
STOP_CONTAINS = None        # contacts.stop_contains of the names file, set in main()


def is_stop(name):
    if STOP_CONTAINS is None:
        raise RuntimeError("names file not loaded")
    return STOP_CONTAINS in name


def load_json(p):
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


def eval_keys(t, v, n):
    """Curve keys (s, value) -> values at frame times i/FPS, i = 0..n-1 (linear, clamped like UE at the ends)."""
    return np.interp(np.arange(n) / FPS, np.asarray(t, dtype=np.float64), np.asarray(v, dtype=np.float64))


def events(b, skip_start=START_EDGE_FRAMES):
    """Touchdown (rising) frame indices of a boolean array; rising edges at frames <= skip_start are ignored.

    A contact that is already active at frame 0, or switches on within the first skip_start frames, is the clip start,
    not a touchdown: without this rule one side active at frame 0 and the other side rising at frame 1-2 is scored as a
    touchdown error of a whole step (14 of the 72 review clips of 2026-10-13 were only this artefact)."""
    b = np.asarray(b, dtype=bool)
    e = np.nonzero(b[1:] & ~b[:-1])[0] + 1
    return e[e > skip_start]


def derive(speed, thr):
    return (np.asarray(speed, dtype=np.float64) < thr).astype(np.float64)


# ---------------------------------------------------------------- validate

def shared_clips(clip_map, curves, sourcepack_meta):
    by_name = {}
    for path, cv in curves.items():
        by_name[path.rsplit(".", 1)[-1]] = cv
    out = []
    for c in clip_map["clips"]:
        if (c.get("curve_src") or {}).get(POLICY_KEY) != "gasp58_export_meta":
            continue
        cv = by_name.get(c["name"])
        mp = os.path.join(sourcepack_meta, c["name"] + ".json")
        if cv is None or not all(k in cv for _, k in SIDES) or not os.path.isfile(mp):
            continue
        g = load_json(mp)
        n = int(g["num_frames"])
        gc = {k.lower(): v for k, v in (g.get("curves") or {}).items()}
        if "contact_l" not in gc or "contact_r" not in gc or len(gc["contact_l"]) != n:
            continue
        if int(c.get("num_frames") or -1) + 1 != n:   # clip map stores UE frames, source pack meta keys = frames + 1
            continue
        out.append({"name": c["name"], "n": n, "stop": is_stop(c["name"]),
                    "speed": {s: eval_keys(cv[k]["t"], cv[k]["v"], n) for s, k in SIDES},
                    "sourcepack": {s: np.asarray(gc["contact_" + s], dtype=np.float64) > 0.5 for s, _ in SIDES}})
    return out


def score(clips, thr, shift=0):
    agree = total = 0
    td_err, td_miss, td_n = [], 0, 0
    per_clip = []
    for c in clips:
        ca = ct = 0
        for s, _ in SIDES:
            sp = c["speed"][s]
            if shift:
                sp = np.roll(sp, shift)
                if shift > 0:
                    sp[:shift] = sp[shift]
                else:
                    sp[shift:] = sp[shift - 1]
            d = derive(sp, thr) > 0.5
            g = c["sourcepack"][s]
            ca += int((d == g).sum())
            ct += len(g)
            ge, de = events(g), events(d)
            for e in ge:
                td_n += 1
                if len(de) == 0:
                    td_miss += 1
                    continue
                td_err.append(int(np.min(np.abs(de - e))))
        agree += ca
        total += ct
        per_clip.append((ca / ct if ct else 1.0, c["name"]))
    td = np.asarray(td_err) if td_err else np.zeros(0)
    per_clip.sort()
    return {"threshold_cm_s": thr, "shift_frames": shift, "clips": len(clips), "frames": total,
            "agreement": round(agree / total, 5) if total else None,
            "touchdowns": td_n, "touchdown_missed": td_miss,
            "touchdown_err_mean_frames": round(float(td.mean()), 3) if len(td) else None,
            "touchdown_err_p95_frames": round(float(np.percentile(td, 95)), 3) if len(td) else None,
            "touchdown_err_max_frames": int(td.max()) if len(td) else None,
            "touchdown_within_2_frames": round(float((td <= 2).sum()) / td_n, 5) if td_n else None,
            "worst_clips": [[n, round(a, 4)] for a, n in per_clip[:5]]}


def clip_review(c, thr):
    """Per-clip touchdown comparison (shift 0) of the derived contacts at thr against the source pack 5.8 contacts."""
    agree = total = 0
    errs, missed, n = [], 0, 0
    for s, _ in SIDES:
        d = derive(c["speed"][s], thr) > 0.5
        g = c["sourcepack"][s]
        agree += int((d == g).sum())
        total += len(g)
        ge, de = events(g), events(d)
        for e in ge:
            n += 1
            if len(de) == 0:
                missed += 1
                continue
            err = int(np.min(np.abs(de - e)))
            errs.append({"side": s, "sourcepack_frame": int(e), "err_frames": err})
    over = [x for x in errs if x["err_frames"] > TD_TOL_FRAMES]
    return {"clip": c["name"], "stop": c["stop"], "threshold_cm_s": thr, "frames": c["n"],
            "agreement": round(agree / total, 5) if total else None, "touchdowns": n, "touchdown_missed": missed,
            "touchdown_err_max_frames": max((x["err_frames"] for x in errs), default=None),
            "touchdowns_over_%d_frames" % TD_TOL_FRAMES: len(over), "over": over}


def cmd_validate(a):
    clip_map, curves = load_json(a.clip_map), load_json(a.curves)["curves"]
    clips = shared_clips(clip_map, curves, a.sourcepack_meta)
    groups = {"stops": [c for c in clips if c["stop"]], "other": [c for c in clips if not c["stop"]]}
    thrs = list(np.arange(a.min, a.max + 1e-9, a.step))
    rep = {"format": "utpp-gaspals-contacts-validation", "version": 1, "generator": "Source/tools/retarget/export_foot_contacts.py validate",
           "licence": "Licensed animation data",
           "criterion": "agreement with GASP 5.8 contact_l/r (binary at 0.5) >= 0.95 of frames and touchdown error <= 2 frames",
           "method": "contact = FootSpeed_<side>(t = i/30, keys interpolated, clamped) < threshold; shift = FootSpeed shifted by k frames; "
                     "touchdown = rising edge of the binary contact after frame %d (edges at frames 0..%d are the clip "
                     "start, not touchdowns)" % (START_EDGE_FRAMES, START_EDGE_FRAMES),
           "clips_shared": len(clips), "groups": {}}
    for gname, gcl in groups.items():
        rows = [score(gcl, float(t), sh) for sh in a.shifts for t in thrs]
        ok = [r for r in rows if r["agreement"] is not None]
        best = max(ok, key=lambda r: (r["agreement"], -(r["touchdown_err_mean_frames"] or 99))) if ok else None
        crit = bool(best and best["agreement"] >= 0.95 and best["touchdown_err_max_frames"] is not None
                    and best["touchdown_within_2_frames"] == 1.0 and best["touchdown_missed"] == 0)
        rep["groups"][gname] = {"clips": len(gcl), "sweep": rows, "best": best, "criterion_pass": crit}
        print("%-5s clips=%d best thr=%s shift=%s agreement=%s td mean/p95/max=%s/%s/%s within2=%s missed=%s pass=%s" % (
            gname, len(gcl), best and best["threshold_cm_s"], best and best["shift_frames"], best and best["agreement"],
            best and best["touchdown_err_mean_frames"], best and best["touchdown_err_p95_frames"],
            best and best["touchdown_err_max_frames"], best and best["touchdown_within_2_frames"],
            best and best["touchdown_missed"], crit))
        for r in rows:
            if r["shift_frames"] == 0:
                print("   thr %5.1f agreement %.4f td mean %s max %s within2 %s" % (
                    r["threshold_cm_s"], r["agreement"], r["touchdown_err_mean_frames"], r["touchdown_err_max_frames"],
                    r["touchdown_within_2_frames"]))
    # acceptance at the build thresholds + review list of clips with touchdown errors > 2 frames
    review_thr = {"stops": a.review_threshold_stops, "other": a.review_threshold}
    rep["d3_acceptance"] = {"rule": "accept derived contacts when the agreement at the build "
                                    "threshold is >= %g; clips with touchdowns off by > %d frames -> review list"
                                    % (a.accept_agreement, TD_TOL_FRAMES), "groups": {}}
    review = []
    for gname, gcl in groups.items():
        s = score(gcl, float(review_thr[gname]), 0)
        acc = bool(s["agreement"] is not None and s["agreement"] >= a.accept_agreement)
        rep["d3_acceptance"]["groups"][gname] = {"threshold_cm_s": review_thr[gname], "score": s, "accepted": acc}
        print("accept %-5s thr %s agreement %s within2 %s missed %s accepted=%s" % (
            gname, review_thr[gname], s["agreement"], s["touchdown_within_2_frames"], s["touchdown_missed"], acc))
        for c in gcl:
            cr = clip_review(c, float(review_thr[gname]))
            if cr["touchdowns_over_%d_frames" % TD_TOL_FRAMES] or cr["touchdown_missed"]:
                cr["group"] = gname
                review.append(cr)
    review.sort(key=lambda x: (x["agreement"] if x["agreement"] is not None else 2.0, x["clip"]))
    rep["d3_acceptance"]["accepted"] = all(g["accepted"] for g in rep["d3_acceptance"]["groups"].values())
    rep["d3_acceptance"]["review_clips"] = len(review)
    if a.review_out:
        rv = {"format": "utpp-gaspals-contacts-review", "version": 1,
              "generator": "Source/tools/retarget/export_foot_contacts.py validate",
              "licence": "Licensed animation data",
              "rule": rep["d3_acceptance"]["rule"],
              "touchdown_rule": "rising edge of the binary contact after frame %d; a contact active at frame 0 or "
                                "switching on at frames 1..%d is the clip start, not a touchdown" % (START_EDGE_FRAMES,
                                                                                                    START_EDGE_FRAMES),
              "thresholds_cm_s": review_thr, "accepted": rep["d3_acceptance"]["accepted"],
              "group_scores": {g: {k: v for k, v in x["score"].items() if k != "worst_clips"}
                               for g, x in rep["d3_acceptance"]["groups"].items()},
              "contact_source_in_build": "these shared clips get the GASP 5.8 contact_l/r in 'build' (clip-map policy "
                                         "gasp58_export_meta, same frame count); the list qualifies the FootSpeed "
                                         "derivation used for clips without a GASP 5.8 counterpart",
              "clips_shared": len(clips), "count": len(review), "clips": review}
        os.makedirs(os.path.dirname(os.path.abspath(a.review_out)), exist_ok=True)
        with open(a.review_out, "w", encoding="utf-8") as fh:
            json.dump(rv, fh, indent=1)
        print("review: %d clips with touchdowns off by > %d frames or missed -> %s" % (len(review), TD_TOL_FRAMES,
                                                                                         a.review_out))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, indent=1)
        print("written", a.out)
    return 0


# ---------------------------------------------------------------- build

def cmd_build(a):
    meta_dir, fbx_dir = os.path.join(a.export, "meta"), os.path.join(a.export, "fbx")
    if os.path.normcase(os.path.abspath(a.out)) in (os.path.normcase(os.path.abspath(meta_dir)), os.path.normcase(os.path.abspath(a.sourcepack_meta))):
        sys.exit("--out must differ from the source meta dirs")
    policy = {c["name"]: (c.get("curve_src") or {}).get(POLICY_KEY) for c in load_json(a.clip_map)["clips"]}
    os.makedirs(a.out, exist_ok=True)
    counts, rows = {}, []
    for fbx in sorted(glob.glob(os.path.join(fbx_dir, "*.fbx"))):
        name = os.path.splitext(os.path.basename(fbx))[0]
        mp = os.path.join(meta_dir, name + ".json")
        if not os.path.isfile(mp):
            rows.append({"clip": name, "source": "error", "why": "no meta json"})
            counts["error"] = counts.get("error", 0) + 1
            continue
        m = load_json(mp)
        n = int(m["num_frames"])
        curves = m.setdefault("curves", {})
        low = {k.lower(): k for k in curves}
        for k in ("contact_l", "contact_r"):
            if k in low:
                del curves[low[k]]
        pol = policy.get(name)
        src, why, thr = None, "", None
        gp = os.path.join(a.sourcepack_meta, name + ".json")
        if pol == "gasp58_export_meta" and os.path.isfile(gp):
            g = load_json(gp)
            gc = {k.lower(): v for k, v in (g.get("curves") or {}).items()}
            if int(g.get("num_frames", -1)) == n and len(gc.get("contact_l", [])) == n and len(gc.get("contact_r", [])) == n:
                curves["contact_l"], curves["contact_r"] = gc["contact_l"], gc["contact_r"]
                src, why = "gasp58", "clip-map policy gasp58_export_meta, same frame count"
            else:
                why = "policy gasp58_export_meta but frame count differs (%s vs %s): fallback" % (g.get("num_frames"), n)
        if src is None:
            fs = {s: next((curves[k] for k in curves if k.lower() == fk.lower()), None) for s, fk in SIDES}
            if all(v is not None and len(v) == n for v in fs.values()):
                thr = a.threshold_stops if is_stop(name) else a.threshold
                for s, _ in SIDES:
                    curves["contact_" + s] = [float(x) for x in derive(fs[s], thr)]
                src = "derived"
                why = (why + "; " if why else "") + "FootSpeed < %g cm/s (policy: %s)" % (thr, pol)
            else:
                src = "none"
                why = (why + "; " if why else "") + "no FootSpeed_L/R with %d values (policy: %s)" % (n, pol)
        m["utpp_contacts"] = {"source": src, "why": why, "threshold_cm_s": thr,
                              "note": "added by Source/tools/retarget/export_foot_contacts.py build; GASP-ALS export meta otherwise unchanged"}
        with open(os.path.join(a.out, name + ".json"), "w", encoding="utf-8") as fh:
            json.dump(m, fh, separators=(",", ":"))
        counts[src] = counts.get(src, 0) + 1
        rows.append({"clip": name, "source": src, "why": why, "frames": n,
                     "contact_frames": [int(sum(1 for x in curves.get("contact_" + s, []) if x > 0.5)) for s, _ in SIDES]})
    summary = {"format": "utpp-gaspals-contacts-build", "version": 1, "export": a.export, "sourcepack_meta": a.sourcepack_meta,
               "clip_map": a.clip_map, "threshold_cm_s": a.threshold, "threshold_stops_cm_s": a.threshold_stops,
               "counts": counts, "clips": rows,
               "licence": "Licensed animation data"}
    with open(os.path.join(a.out, "contacts_build.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=1)
    print("UTPP_CONTACTS_BUILD", json.dumps(counts), "->", a.out)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="export_foot_contacts.py")
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate")
    v.add_argument("--clip-map", default=CLIP_MAP)
    v.add_argument("--curves", default=CURVES)
    v.add_argument("--sourcepack-meta", default=SOURCEPACK_META)
    v.add_argument("--min", type=float, default=20.0)
    v.add_argument("--max", type=float, default=80.0)
    v.add_argument("--step", type=float, default=5.0)
    v.add_argument("--shifts", type=int, nargs="+", default=[0])
    v.add_argument("--out", default=os.path.join(ROOT, "Research", "inventory", "sourcepack-contacts-validation.json"))
    v.add_argument("--review-threshold", type=float, default=60.0, help="cm/s, non-Stop clips (= build --threshold)")
    v.add_argument("--review-threshold-stops", type=float, default=80.0, help="cm/s, Stop clips (= build --threshold-stops)")
    v.add_argument("--accept-agreement", type=float, default=0.95, help="acceptance of derived contacts")
    v.add_argument("--review-out", default=os.path.join(ROOT, "Research", "inventory", "sourcepack-contacts-review.json"))
    b = sub.add_parser("build")
    b.add_argument("--export", default=EXPORT, help="source pack export dir with fbx\\ and meta\\")
    b.add_argument("--clip-map", default=CLIP_MAP)
    b.add_argument("--sourcepack-meta", default=SOURCEPACK_META)
    b.add_argument("--threshold", type=float, required=True, help="cm/s, from 'validate' (non-Stop clips)")
    b.add_argument("--threshold-stops", type=float, required=True, help="cm/s, from 'validate' (Stop clips)")
    b.add_argument("--out", default=os.path.join(ROOT, "Animations", "retarget", "meta_sourcepack"))
    for p in (v, b):
        p.add_argument("--names", default=NAMES, help="source pack names data file (default %(default)s)")
    a = ap.parse_args(argv)
    global STOP_CONTAINS
    if not os.path.isfile(a.names):
        sys.exit("names file missing: %s (source pack names are kept outside the git repo)" % a.names)
    STOP_CONTAINS = load_json(a.names)["contacts"]["stop_contains"]
    return cmd_validate(a) if a.cmd == "validate" else cmd_build(a)


if __name__ == "__main__":
    sys.exit(main())
