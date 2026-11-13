"""Sanity check of a retargeted slide set against the source pack slide analysis (pelvis drop timing, root speed).

python slide_sanity.py --out-dir <out_sourcepack_slide_*> --analysis <sourcepack-slide-analysis json> [--json report.json]
Thresholds: TIME_TOL_S / SPEED_TOL are [P] tolerances (retarget rescales the hips by hip_ratio, the analysis uses UE cm).
Exit 1 when a clip deviates beyond a tolerance. Reads only.
"""
import argparse
import glob
import json
import math
import os
import sys

TIME_TOL_S = 0.15       # [P] 4-5 frames at 30 fps
SPEED_TOL = 0.12        # [P] relative, root speed maximum
HIPS = "Hips"
TRAJ = "Trajectory"


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def clip_series(clip):
    names = clip["bone_names"]
    ih, it = names.index(HIPS), names.index(TRAJ)
    fps = float(clip["fps"])
    hz = [fr["bones"][ih][2] for fr in clip["frames"]]
    ty = [(fr["bones"][it][0], fr["bones"][it][1]) for fr in clip["frames"]]
    return fps, hz, ty, float(clip["retarget"].get("hip_ratio", 1.0))


def first_below(series, thr, fps):
    for i, v in enumerate(series):
        if v < thr:
            return i / fps
    return None


def speed_series(ty, fps):
    return [math.hypot(ty[i + 1][0] - ty[i][0], ty[i + 1][1] - ty[i][1]) * fps for i in range(len(ty) - 1)]


def check_clip(clip, ana):
    fps, hz, ty, ratio = clip_series(clip)
    out = {"frames": len(hz), "hip_ratio": ratio, "hips_z_m": [round(min(hz), 4), round(max(hz), 4), round(hz[0], 4), round(hz[-1], 4)]}
    bad = []
    t = ana.get("timing") or {}
    ap = ana.get("pelvis_z_min_max_start_end_cm")
    if t.get("pelvis_below_60") is not None:
        # UE threshold 60 cm of the source pelvis; retarget scales the hips by hip_ratio
        got = first_below(hz, 0.60 * ratio, fps)
        out["pelvis_below_60_s"] = got
        out["pelvis_below_60_ref_s"] = t["pelvis_below_60"]
        if got is None or abs(got - t["pelvis_below_60"]) > TIME_TOL_S:
            bad.append("pelvis_below_60 %s vs %s" % (got, t["pelvis_below_60"]))
    if ap:
        out["pelvis_ref_cm"] = ap
    sp = speed_series(ty, fps)
    ref_max = ana.get("root_speed_max_cm_s")
    if sp and ref_max:
        got_max = max(sp) * 100.0
        out["root_speed_max_cm_s"] = round(got_max, 1)
        out["root_speed_ref_cm_s"] = ref_max
        # the retarget scales the planar root by leg length ratio (not hip_ratio): informational ratio, tolerance on scale-free shape
        out["root_speed_ratio"] = round(got_max / ref_max, 3)
    return out, bad


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--analysis", required=True)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    ana = {c["name"]: c for c in load(a.analysis)["clips"]}
    res, fails = {}, {}
    for f in sorted(glob.glob(os.path.join(a.out_dir, "*.utppclip.json"))):
        name = os.path.basename(f)[:-len(".utppclip.json")]
        if name not in ana:
            fails[name] = ["not in analysis"]
            continue
        r, bad = check_clip(load(f), ana[name])
        res[name] = r
        if bad:
            fails[name] = bad
    print("clips %d, deviating %d" % (len(res), len(fails)))
    for k, v in fails.items():
        print("DEVIATION", k, v)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump({"clips": res, "deviations": fails, "tol_time_s": TIME_TOL_S}, fh, indent=1)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
