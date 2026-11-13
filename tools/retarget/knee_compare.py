"""knee_compare.py - before/after table of knee_metrics.py reports (knee_<label>.json), optionally restricted to a clip regex.

  python knee_compare.py --report A.json --report B.json [--regex RE] [--flex-gate 20]
Per report and leg: clips with >= 1 CP flip frame, total CP flip frames, max CP |dtheta|, median of per-clip CP off_bone p95,
median |theta CP - UE| (gated mean), loop seams with CP flex jump over UE > 30 deg [P], max seam flex excess, mean CP flex
excess over UE (flex_max). Read-only; source-pack-derived numbers stay local.
"""
import argparse
import json
import re
import statistics as st


def leg_rows(rep, rx):
    for r in rep["rows"]:
        if rx and not re.search(rx, r["clip"]):
            continue
        for s in ("l", "r"):
            yield r, s, r[s]


def summarize(rep, rx):
    out = {}
    for side in ("l", "r"):
        rows = [(r, d) for r, s, d in leg_rows(rep, rx) if s == side]
        cp = [d["cp"] for _, d in rows]
        ue = [d["ue"] for _, d in rows]
        seams = [(d["cp"]["seam"]["flex_jump"] - d["ue"]["seam"]["flex_jump"]) for _, d in rows
                 if "seam" in d["cp"] and d["cp"]["seam"] and d["ue"]["seam"]]
        th = [abs(d["theta_cp_minus_ue_mean_gated"]) for _, d in rows if d["theta_cp_minus_ue_mean_gated"] is not None]
        offb = [c["off_bone_p95"] for c in cp if c["off_bone_p95"] is not None]
        out[side] = {
            "legs": len(rows),
            "clips_with_flip": sum(1 for c in cp if c["flip_frames"]),
            "flip_frames": sum(c["flip_frames"] for c in cp),
            "ue_clips_with_flip": sum(1 for c in ue if c["flip_frames"]),
            "dtheta_max": max((c["dtheta_max"] for c in cp if c["dtheta_max"] is not None), default=None),
            "off_bone_p95_median": round(st.median(offb), 2) if offb else None,
            "theta_vs_ue_abs_median": round(st.median(th), 2) if th else None,
            "theta_vs_ue_abs_max": round(max(th), 2) if th else None,
            "seams": len(seams), "seams_flex_over_ue_gt30": sum(1 for x in seams if x > 30.0),
            "seam_flex_over_ue_max": round(max(seams), 2) if seams else None,
            "flex_max_minus_ue_mean": round(st.mean(c["flex_max"] - u["flex_max"] for c, u in zip(cp, ue)), 2) if cp else None,
        }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="append", required=True)
    ap.add_argument("--regex", default="")
    a = ap.parse_args()
    for p in a.report:
        rep = json.load(open(p, encoding="utf-8"))
        s = summarize(rep, a.regex)
        print("%s (%s clips%s)" % (rep["label"], rep["clips"], ", regex " + a.regex if a.regex else ""))
        for side in ("l", "r"):
            print("  %s %s" % (side, json.dumps(s[side])))


if __name__ == "__main__":
    main()
