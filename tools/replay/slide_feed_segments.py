"""Cut the recorded slides of the analysis (game_slide_measurements.events) out of the game feed dumps into .feed files for
utpp_motion_replay (columns: t dt vx vy vz mvx mvy yawDeg camDeg loco psm trans baseClip baseFlags baseCost baseMicros step).

python slide_feed_segments.py --analysis <analysis json> --logs Research/logs/game --out <dir> [--pre 1.5] [--post 3.0]
Each event -> slide_<n>.feed (frames t0-pre .. t0+duration+post, time rebased to 0 at the cut start) and slide_list.json
(file, event, slide start/end relative to the cut). Entry-from-sprint events only unless --all.
"""
import argparse
import glob
import json
import os
import sys


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--analysis", required=True)
    ap.add_argument("--logs", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--pre", type=float, default=1.5)
    ap.add_argument("--post", type=float, default=3.0)
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args(argv)
    ev = json.load(open(a.analysis, encoding="utf-8"))["game_slide_measurements"]["events"]
    os.makedirs(a.out, exist_ok=True)
    lst = []
    for n, e in enumerate(ev):
        if not a.all and "sprint -> slide" not in e["entry"]:
            continue
        base = e["log"].replace("\\", "/").split("/")[-1]
        hits = sorted(glob.glob(os.path.join(a.logs, "**", base), recursive=True))
        if not hits:
            print("missing", e["log"])
            continue
        t0, t1 = e["t0"] - a.pre, e["t0"] + e["duration_s"] + a.post
        rows = []
        for line in open(hits[0], encoding="utf-8", errors="replace"):
            if line.startswith("#"):
                continue
            c = line.split()
            if len(c) < 12:
                continue
            try:
                t = float(c[0])
            except ValueError:
                continue
            if t0 <= t <= t1:
                rows.append(c)
        if len(rows) < 10:
            print("short", e["log"], e["t0"], len(rows))
            continue
        name = "slide_%02d.feed" % n
        with open(os.path.join(a.out, name), "w", newline="\n") as f:
            f.write("# feed slide-cut log=%s t0=%.3f\n" % (e["log"], t0))
            for c in rows:
                f.write("%.6f %s %s %s %s %s %s %s %s %s %s 0 - 0 0 0 -\n" % (float(c[0]) - t0, c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9], c[10]))
        lst.append({"file": name, "log": e["log"], "t0": e["t0"], "slide_start": a.pre, "slide_end": a.pre + e["duration_s"], "frames": len(rows)})
    json.dump(lst, open(os.path.join(a.out, "slide_list.json"), "w"), indent=1)
    print(len(lst), "segments")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
