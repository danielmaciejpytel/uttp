"""report_compare.py - compare two retarget_batch.py report.json files (same clips): foot slide (ankle / pivot / excess
over source, cm/s), hip drop, IK clamps, FK / root pass. Read-only.

  python report_compare.py --before <dir>/report.json --after <dir>/report.json [--out file.txt]
"""
import argparse
import json
import statistics as st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    b = {r["clip"]: r for r in json.load(open(a.before, encoding="utf-8"))["clips"] if r["status"] == "ok"}
    f = {r["clip"]: r for r in json.load(open(a.after, encoding="utf-8"))["clips"] if r["status"] == "ok"}
    common = sorted(set(b) & set(f))
    L = ["before %s (%d ok) / after %s (%d ok), common %d" % (a.before, len(b), a.after, len(f), len(common))]
    for key, get in (("ankle_mean", lambda r, s: r["foot_slide"][s]["cp"]["mean_cm_s"]),
                     ("pivot_mean", lambda r, s: r["foot_slide"][s]["cp_pivot"]["mean_cm_s"]),
                     ("matched_cp_mean", lambda r, s: r["foot_slide"][s]["matched"]["cp_mean_cm_s"])):
        diffs = []
        for c in common:
            for s in ("l", "r"):
                x, y = get(b[c], s), get(f[c], s)
                if x is not None and y is not None:
                    diffs.append(y - x)
        if diffs:
            L.append("%-16s legs %d: after-before mean %+.4f, max |d| %.4f cm/s, changed >0.01: %d" % (
                key, len(diffs), st.mean(diffs), max(abs(d) for d in diffs), sum(1 for d in diffs if abs(d) > 0.01)))
    for key in ("foot_slide_flag", "foot_slide_pivot_flag", "foot_slide_excess_vs_source_flag"):
        L.append("%-34s before %d after %d" % (key, sum(1 for c in common if b[c][key]), sum(1 for c in common if f[c][key])))
    for key in ("hip_drop_max_cm", "hip_drop_max_step_cm", "hip_drop_frames", "ik_clamped_leg_frames"):
        xb = [b[c][key] for c in common]
        xf = [f[c][key] for c in common]
        L.append("%-22s before mean %.3f max %.3f | after mean %.3f max %.3f" % (key, st.mean(xb), max(xb), st.mean(xf), max(xf)))
    L.append("fk pass before %d after %d; root pass before %d after %d" % (
        sum(1 for c in common if b[c]["check_fk_vs_blender"]["pass"]), sum(1 for c in common if f[c]["check_fk_vs_blender"]["pass"]),
        sum(1 for c in common if b[c]["root_motion"]["pass"]), sum(1 for c in common if f[c]["root_motion"]["pass"])))
    top = sorted(common, key=lambda c: -f[c]["hip_drop_max_cm"])[:10]
    L.append("top hip drop after (cm, before): " + ", ".join("%s %.1f (%.1f)" % (c, f[c]["hip_drop_max_cm"], b[c]["hip_drop_max_cm"]) for c in top))
    txt = "\n".join(L)
    print(txt)
    if a.out:
        open(a.out, "w", encoding="utf-8").write(txt + "\n")


if __name__ == "__main__":
    main()
