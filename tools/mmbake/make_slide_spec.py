"""Author the source pack slide spec (--slide-spec of mmbake_base.py --format v2) from the PoseSearch data dumps of the
slide export and the retargeted clips.

python make_slide_spec.py --set neutral|relaxed --clips Animations/retarget/out_sourcepack_slide_woman --out Research/inventory/mmbake-v2-slide.json

What is read (all source pack):
  Animations/sourcepack/slide_export/meta/_data__*.json   schemas, database biases (ContinuingPoseCostBias / BaseCostBias),
                                                         mirror/reselection flags are copied by the baker from these dumps
  --clips                                                the retargeted clips (frame counts, hips height for the exit windows)
Windows (the Neutral clips carry no PoseSearch notifies; the Relaxed ones are authored the same way, [P] = authored here):
  Into_*   EXCLUDE_FROM [P] 2.1 s .. end: the loop-like tail (root speed constant) must not be entered mid-slide; the Loop clip
           is the loop. The 1.2 Relaxed Into clips exclude the same region.
  Out_*    searched from 0 (whole clip); BlockTransition [P] from the later of (a) the first time the hips have recovered RECOVER [P] = 0.9 of
           the way from the clip minimum to the clip end height (afterwards the clip is ordinary locomotion / idle that the
           regular databases already cover; the pose may continue there but a search never starts there).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
if os.path.basename(ROOT).lower() == "build":
    ROOT = os.path.dirname(ROOT)

DUMP_DIR = "Animations/sourcepack/slide_export/meta"
INTO_EXCLUDE_FROM_S = 2.1      # [P] analysis: Into_* loop-like tail from ~2.1 s
SLIDE_CONT_BIAS = -2.0        # [P] ContinuingPoseCostBias of FeetOut / KneesOut (1.2: -0.02); stops the Into_Lfoot <-> Into_Rfoot flicker
RECOVER = 0.9                  # [P] fraction of the hips rise after which an exit clip is blocked
SETTLE_TOL_M_S = 0.15          # [P] root speed within 0.15 m/s of the terminal speed = settled
MIN_BLOCK_S = 0.1              # [P] shorter blocked tails are not authored
EXITS = [("StandIdle", "Out_Idle_Stand"), ("CrouchIdle", "Out_Idle_Crouch"), ("CrouchWalk", "Out_Moving_Crouch"),
         ("Walk", "Out_Moving_Walk"), ("Run", "Out_Moving_Run"), ("Sprint", "Out_Moving_Sprint")]
VARIANTS = ("FootOut", "KneesOut")
FAMILIES = ["CHT_PoseSearchDatabases_Dense", "CHT_PoseSearchDatabases_Sparse"]
EXIT_TABLES = {"StandIdle": "Stand Idles", "CrouchIdle": "Crouch Idle", "CrouchWalk": "Crouch Moving",
               "Walk": "Stand Walks", "Run": "Stand Runs", "Sprint": "Stand Sprint"}


def load(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def clip_info(clips_dir, name):
    c = load(os.path.join(clips_dir, name + ".utppclip.json"))
    ih = c["bone_names"].index("Hips")
    it = c["bone_names"].index("Trajectory")
    tr = [(fr["bones"][it][0], fr["bones"][it][1]) for fr in c["frames"]]
    fps = float(c["fps"])
    sp = [((tr[i + 1][0] - tr[i][0]) ** 2 + (tr[i + 1][1] - tr[i][1]) ** 2) ** 0.5 * fps for i in range(len(tr) - 1)]
    return (int(c["num_frames"]) - 1) / fps, [fr["bones"][ih][2] for fr in c["frames"]], fps, sp


def settle_time(sp, fps):
    """First time after which the root speed stays within SETTLE_TOL_M_S of its terminal value (the exit clip is then
    ordinary locomotion / idle). The slide hands over at the crouch-walk speed (~2.7 m/s), far below the 5-7 m/s the exit
    clips start at, so the searchable window must reach the part of the clip with that speed."""
    term = sp[-1]
    for i in range(len(sp)):
        if all(abs(v - term) <= SETTLE_TOL_M_S for v in sp[i:]):
            return i / fps
    return None


def recover_time(hz, fps):
    lo, end = min(hz), hz[-1]
    if end - lo < 0.05:
        return None
    thr = lo + RECOVER * (end - lo)
    i_min = hz.index(lo)
    for i in range(i_min, len(hz)):
        if hz[i] >= thr:
            return i / fps
    return None


def build(clips_dir, which, relaxed_names=None):
    style = "Neutral" if which == "neutral" else "Relaxed"
    names = sorted(f[:-len(".utppclip.json")] for f in os.listdir(clips_dir) if f.endswith(".utppclip.json"))
    names = [n for n in names if n.startswith(f"M_{style}_Slide_")]

    def pick(variant, role_prefix):
        return [n for n in names if f"_{variant}_" in n and n.split(f"Slide_{variant}_", 1)[1].startswith(role_prefix)]

    def e(clip, **kw):
        length, hz, fps, sp = clip_info(clips_dir, clip)
        r = {"clip": clip, "sampling_range": [0.0, 0.0], "disable_reselection": True, "looping": None}
        if "_Into_" in clip and length > INTO_EXCLUDE_FROM_S:
            r["exclude"] = [[INTO_EXCLUDE_FROM_S, round(length, 6)]]
        if "_Out_" in clip:
            t = recover_time(hz, fps)
            ts = settle_time(sp, fps)
            if t is not None and ts is not None:
                pass                 # settle time is reported only (pose of the early exit rows matches the slide pose; speed is mapped)
            if t is not None and length - t >= MIN_BLOCK_S:
                r["block"] = [[round(t, 6), round(length, 6)]]
        return r

    dbs = []
    for db, variant in (("PSD_Slide_FeetOut", "FootOut"), ("PSD_Slide_KneesOut", "KneesOut")):
        ents = pick(variant, "Into_") + pick(variant, "Loop")
        if which == "relaxed":
            other = "KneesOut" if variant == "FootOut" else "FootOut"
            ents += [n for n in names if n.endswith(f"_{other}_trans_{variant}")]
        dbs.append({"name": db, "dump": "PSD_Relaxed_Slide_" + variant.replace("FootOut", "FeetOut"), "schema": "PSS_Relaxed_Slide",
                    "normalization_set": "PSN_Utpp_Slide", "continuing_pose_cost_bias": SLIDE_CONT_BIAS, "entries": [e(c) for c in ents]})
    proxy = []
    for key, role in EXITS:
        ents = [n for v in VARIANTS for n in pick(v, role)]
        if not ents:                   # Neutral has no Sprint exit: the Run exits stand in (documented in the progress file)
            ents = [n for v in VARIANTS for n in pick(v, "Out_Moving_Run")]
            proxy.append(key)
        dbs.append({"name": "PSD_Slide_ExitTo" + key, "dump": "PSD_Relaxed_Slide_FeetOut_ExitTo" + key,
                    "schema": "PSS_Relaxed_SlideExit", "normalization_set": "PSN_Utpp_SlideExit", "entries": [e(c) for c in ents]})
    spec = {
        "format": "utpp-mmbake-v2-slide-spec", "version": 1, "set": which,
        "licence": "Licensed animation data",
        "dump_dir": DUMP_DIR,
        "schemas": ["PSS_Relaxed_Slide", "PSS_Relaxed_SlideExit"],
        "normalization_sets": {"PSN_Utpp_Slide": [d["name"] for d in dbs if d["schema"] == "PSS_Relaxed_Slide"],
                               "PSN_Utpp_SlideExit": [d["name"] for d in dbs if d["schema"] == "PSS_Relaxed_SlideExit"]},
        "databases": dbs,
        "authoring": {"into_exclude_from_s": INTO_EXCLUDE_FROM_S, "exit_recover_fraction": RECOVER, "exit_min_block_s": MIN_BLOCK_S,
                      "exit_proxy_for": proxy, "clips_dir": os.path.relpath(clips_dir, ROOT).replace("\\", "/")},
        "chooser": {
            "movement_mode_input": "MovementMode", "sliding_value": 2,
            "direction_input": "MovementDirection", "direction_values": {"Forward": 0, "Other": 1},
            "last_mode_input": "MovementMode_LastFrame",
            "slide_table_label": "Slide",
            "slide_table": [{"direction": "Forward", "db": "PSD_Slide_FeetOut"}, {"direction": "Other", "db": "PSD_Slide_KneesOut"}],
            "families": FAMILIES,
            "exit_rows": [{"table": EXIT_TABLES[k], "db": "PSD_Slide_ExitTo" + k} for k, _ in EXITS],
            "databases": [d["name"] for d in dbs],
        },
    }
    return spec


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", choices=("neutral", "relaxed"), default="neutral")
    ap.add_argument("--clips", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    spec = build(os.path.abspath(a.clips), a.set)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(spec, f, indent=1)
        f.write("\n")
    for d in spec["databases"]:
        print(d["name"], [(x["clip"].replace("M_Neutral_Slide_", "").replace("M_Relaxed_Slide_", ""), x.get("exclude"), x.get("block")) for x in d["entries"]])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
