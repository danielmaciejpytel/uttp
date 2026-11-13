"""Cut a long gameplay recording + its UTPP log into analysis jobs (sheets + log slice per job) for parallel review.

  python recording_package.py --video V.mp4 --log L1 --log L2 --start "2026-10-25 01:36:20.5" --duration 736 --out DIR

Jobs: one per RELOAD episode (12 fps around it, 3x2 tiles) and fixed-length general segments (3 fps, 4x2 tiles).
Every tile carries t=<video seconds>; the log slices are re-keyed to the same video time (tv=).
"""
import argparse
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
SHEETS = os.path.join(HERE, "video_sheets.py")
RX_TS = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]")
KEEP = ("RELOAD episode", "MMGATE", "MMAIR", "MMHOLD", "UTPPMotion", "utpp_vehicle", "vault", "heading lock", "perspective",
        "weapon", "slot", "hip", "aim", "reload", "Reload")
RX_DB = re.compile(r"\bdb=(\d+:\S+)")
RX_CLIP = re.compile(r"\bclip=(\d+:\S+)")


def wall(ts: str) -> datetime:
    return datetime.strptime(ts, "%Y-%m-%d %H:%M:%S.%f")


def field(line: str, key: str, pat: str = r"\S+") -> str:
    m = re.search(r"(?:^|\s)" + re.escape(key) + r"=(" + pat + ")", line)
    return m.group(1) if m else "-"


def slim(line: str) -> str:
    """One compact line for an MOTION / MOTIONPOSE log line."""
    if "UTPP MOTIONPOSE" in line:
        m = re.search(r"compose=(\S+)", line)
        return "POSE " + (m.group(1) if m else "")
    db = RX_DB.search(line)
    clip = RX_CLIP.search(line)
    return ("MM db=" + (db.group(1) if db else "-") + " clip=" + (clip.group(1) if clip else "-") + " ov=" + field(line, "ov")
            + " act=" + field(line, "act") + " reloadUpper=" + field(line, "reloadUpper") + " aimW=" + field(line, "aimW")
            + " lay=" + field(line, "lay") + " air=" + field(line, "air") + " trav=" + field(line, "trav"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--log", action="append", required=True)
    ap.add_argument("--start", required=True, help="wall clock of video t=0")
    ap.add_argument("--duration", type=float, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seg", type=float, default=32.0)
    ap.add_argument("--jobs", type=int, default=4, help="parallel ffmpeg processes")
    ap.add_argument("--pre", type=float, default=1.5)
    ap.add_argument("--post", type=float, default=1.2)
    ap.add_argument("--windows-json", help="reload windows from the pose CSV [{v0,dur,clip,ov}] in video seconds: build ONLY merged reload jobs from them")
    ap.add_argument("--min-v0", type=float, default=0.0)
    ap.add_argument("--merge-gap", type=float, default=2.5)
    ap.add_argument("--max-span", type=float, default=14.0)
    ap.add_argument("--prefix", default="n")
    a = ap.parse_args()
    t_start = wall(a.start)
    lines = []
    for p in a.log:
        with open(p, encoding="utf-8", errors="replace") as f:
            lines += f.read().splitlines()
    parsed = []
    for l in lines:
        m = RX_TS.match(l)
        if m:
            parsed.append((wall(m.group(1)), l))
    parsed.sort(key=lambda x: x[0])

    def tv(w):
        return (w - t_start).total_seconds()

    jobs = []
    for w, l in parsed:
        if "RELOAD episode" in l:
            dur = float(field(l, "dur"))
            end = tv(w)
            if 0 <= end <= a.duration + 2:
                jobs.append({"kind": "reload", "t0": max(0.0, end - dur - a.pre), "t1": min(a.duration, end + a.post), "end": end,
                             "dur": dur, "weapon": field(l, "rec"), "mag": field(l, "mag"),
                             "start_state": "loco=" + field(l, "start:loco") + " upper=" + field(l, "upper"),
                             "text": l.split("] ", 4)[-1][:300]})
    jobs.sort(key=lambda x: x["t0"])
    for i, j in enumerate(jobs):
        j["name"] = "r%02d_%s_%ds" % (i + 1, j["weapon"].split(".")[-1][:14], int(j["t0"]))
    busy = [(j["t0"], j["t1"]) for j in jobs]
    if a.windows_json:
        wins = [w for w in json.load(open(a.windows_json)) if w["v0"] >= a.min_v0 and w["v0"] < a.duration]
        wins.sort(key=lambda w: w["v0"])
        jobs, groups = [], []
        for w in wins:
            if groups and w["v0"] - groups[-1]["end"] <= a.merge_gap and (w["v0"] + w["dur"] + a.post) - groups[-1]["t0"] <= a.max_span:
                groups[-1]["end"] = w["v0"] + w["dur"]
                groups[-1]["n"] += 1
                groups[-1].setdefault("info", []).append("%s %s start=%s" % (w.get("weapon", "?"), w.get("mag", "?"), w.get("state", "?")))
            else:
                groups.append({"t0": max(0.0, w["v0"] - a.pre), "end": w["v0"] + w["dur"], "n": 1, "ov": w["ov"],
                               "info": ["%s %s start=%s" % (w.get("weapon", "?"), w.get("mag", "?"), w.get("state", "?"))]})
        for i, g in enumerate(groups):
            jobs.append({"kind": "reload", "t0": g["t0"], "t1": min(a.duration, g["end"] + a.post), "end": g["end"], "dur": g["end"] - g["t0"],
                         "weapon": "; ".join(g["info"]) + " (episodes: %d)" % g["n"], "mag": "?", "start_state": "?",
                         "text": "reload window(s) from log/pose CSV", "name": "%s%02d_%ds" % (a.prefix, i + 1, int(g["t0"]))})
        busy = []
    else:
        t, gi = 0.0, 0
        while t < a.duration:
            t1 = min(a.duration, t + a.seg)
            gi += 1
            jobs.append({"kind": "general", "t0": t, "t1": t1, "name": "g%02d_%ds" % (gi, int(t)),
                         "reload_windows_inside": [(round(b0, 1), round(b1, 1)) for b0, b1 in busy if b1 > t and b0 < t1]})
            t = t1
    os.makedirs(a.out, exist_ok=True)
    cmds = []
    for j in jobs:
        d = os.path.join(a.out, j["name"])
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "log.txt"), "w", encoding="utf-8") as f:
            f.write("# log slice keyed to VIDEO seconds (tv). video t=0 = wall %s. %s job %s t=%.1f..%.1f\n" % (
                a.start, j["kind"], j["name"], j["t0"], j["t1"]))
            for w, l in parsed:
                x = tv(w)
                if j["t0"] - 0.5 <= x <= j["t1"] + 0.5:
                    if "UTPP MOTION t=" in l or "UTPP MOTIONPOSE" in l:
                        f.write("tv=%7.2f %s\n" % (x, slim(l)))
                    elif any(k in l for k in KEEP) and "MMTRACE" not in l and "head look" not in l:
                        f.write("tv=%7.2f EVT %s\n" % (x, l.split("] ", 4)[-1][:260]))
        json.dump(j, open(os.path.join(d, "job.json"), "w"), indent=1)
        if j["kind"] == "reload":
            cmds.append([sys.executable, "-I", SHEETS, a.video, d, "sheet", "%.3f" % j["t0"], "%.3f" % j["t1"], "12",
                         "--cols", "3", "--rows", "2", "--crop", "900:900:420:130", "--tile-w", "640"])
        else:
            cmds.append([sys.executable, "-I", SHEETS, a.video, d, "sheet", "%.3f" % j["t0"], "%.3f" % j["t1"], "3",
                         "--cols", "4", "--rows", "2", "--crop", "1000:900:380:130", "--tile-w", "480"])

    def run(c):
        r = subprocess.run(c, capture_output=True, text=True)
        return (os.path.basename(c[4]), r.returncode, (r.stdout + r.stderr).strip()[-100:])

    with ThreadPoolExecutor(a.jobs) as ex:
        for res in ex.map(run, cmds):
            print(res)
    json.dump(jobs, open(os.path.join(a.out, "jobs.json"), "w"), indent=1)
    print("jobs:", len(jobs), "reload:", sum(1 for j in jobs if j["kind"] == "reload"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
