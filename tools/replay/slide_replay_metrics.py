"""Metrics of the slide replay csv (utpp_motion_replay --slide --csv): per feed the clip sequence while mode == Sliding (2) and
after it, jumps, the speed at which the exit clip starts.

python slide_replay_metrics.py --csv slide_replay.csv --list slide_list.json [--json out.json]
Checks per feed: Sliding frames exist; sequence order Into -> Loop (Into_* before Slide_*_Loop); the first clip after the
slide ends is an exit clip (Out_*) or a regular clip; exit clip start speed (game m/s). Exit 0 if no feed violates
the hard checks (a sliding frame exists, no Into after a Loop, Into only first).
"""
import argparse
import csv
import json
import sys
from collections import defaultdict


def kind(c):
    if "_Slide_" not in c:
        return "other"
    for k in ("Into", "Loop", "Out_Idle_Stand", "Out_Idle_Crouch", "Out_Moving_Crouch", "Out_Moving_Walk", "Out_Moving_Run"):
        if "_" + k in c:
            return "Out" if k.startswith("Out") else k
    return "slide?"


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--list", required=True)
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    by = defaultdict(list)
    for r in csv.DictReader(open(a.csv, encoding="utf-8")):
        by[r["feed"]].append(r)
    meta = {m["file"]: m for m in json.load(open(a.list))}
    res, bad = {}, []
    for feed, rows in sorted(by.items()):
        m = meta.get(feed, {})
        seq, last = [], None
        mode_frames = sum(1 for r in rows if r["mode"] == "2")
        exit_start = None
        jumps_in_slide = 0
        for i, r in enumerate(rows):
            c = r["motionclip"]
            k = kind(c)
            if c != last:
                seq.append((round(float(r["t"]), 2), c, k, r["mode"], round(float(r["speed"]), 2)))
                if k == "Out" and exit_start is None:
                    exit_start = (round(float(r["t"]), 2), round(float(r["speed"]), 2), c)
                last = c
            if r["mode"] == "2" and r["jumped"] == "1" and i > 0:
                jumps_in_slide += 1
        kinds = [s[2] for s in seq if s[2] in ("Into", "Loop", "Out")]
        order_ok = "Loop" not in kinds or "Into" not in kinds[kinds.index("Loop"):]
        res[feed] = {"slide_window": [m.get("slide_start"), m.get("slide_end")], "sliding_frames": mode_frames, "sequence": seq,
                     "exit_start(t,speed,clip)": exit_start, "jumps_in_slide": jumps_in_slide, "order_ok": order_ok}
        if mode_frames == 0 or not order_ok:
            bad.append(feed)
    for f, v in res.items():
        print(f, "frames", v["sliding_frames"], "order_ok", v["order_ok"], "exit", v["exit_start(t,speed,clip)"], "jumps", v["jumps_in_slide"])
        print("   ", [(s[0], s[2]) for s in v["sequence"] if s[2] != "other"])
    print("violations", bad)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
