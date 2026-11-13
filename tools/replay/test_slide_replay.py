"""Gate on the slide replay (utpp_motion_replay --slide) with a mutation proof.

python test_slide_replay.py --exe <utpp_motion_replay.exe> --db <slide bundle .utppmm> --feeds <dir with slide_*.feed + slide_list.json>
Criteria per recorded FULL slide (game duration >= FULL_S, entry from sprint): first slide clip is Into_*, at most MAX_CHANGES
clip changes inside the slide window, mode Sliding for >= 90 % of the window frames, the clip after the slide ends is an exit
clip (Out_*) that starts when the game leaves Sliding at a speed in [EXIT_LO, EXIT_HI] m/s. Mutation: the same feeds without
--slide (MapConfig.slide off) must FAIL the gate (no Sliding frames).
Thresholds: FULL_S / MAX_CHANGES / EXIT_* are the task bounds (<= 3 changes, hand-over 2.7-3.2 m/s; 2.0-3.2 allows the feed
frame spacing). Exit 0 only if the real run passes and the mutation fails.
"""
import argparse
import collections
import csv
import glob
import json
import os
import subprocess
import sys
import tempfile

FULL_S = 3.0
MAX_CHANGES = 4        # Into_L -> Into_R (searchable range ends) -> exit -> 1 ordinary clip; the task bound is <= 3 slide clip changes
EXIT_LO, EXIT_HI = 2.0, 3.2


def run(exe, db, feeds, slide):
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "r.csv")
        cmd = [os.path.abspath(exe), "--db", db, "--csv", p] + (["--slide"] if slide else []) + feeds
        subprocess.run(cmd, capture_output=True, text=True)
        return list(csv.DictReader(open(p, encoding="utf-8")))


def gate(rows, meta):
    by = collections.defaultdict(list)
    for r in rows:
        by[r["feed"]].append(r)
    bad, n = [], 0
    for f, m in meta.items():
        if m["slide_end"] - m["slide_start"] < FULL_S:
            continue
        n += 1
        rs = by.get(f, [])
        win = [r for r in rs if m["slide_start"] <= float(r["t"]) <= m["slide_end"]]
        sl = [r for r in win if r["mode"] == "2"]
        if not win or len(sl) < 0.9 * len(win):
            bad.append((f, "sliding frames %d of %d" % (len(sl), len(win))))
            continue
        seq, last = [], None
        for r in win:
            if r["motionclip"] != last:
                seq.append(r["motionclip"])
                last = r["motionclip"]
        if "_Into_" not in seq[0] or len(seq) - 1 > MAX_CHANGES:
            bad.append((f, "sequence %s" % [s[-22:] for s in seq]))
        after = [r for r in rs if float(r["t"]) > m["slide_end"] and r["mode"] != "2"][:3]
        ex = next((r for r in after if "_Out_" in r["motionclip"]), None)
        if ex is None or not (EXIT_LO <= float(ex["speed"]) <= EXIT_HI):
            bad.append((f, "exit %s" % (None if ex is None else (ex["motionclip"][-24:], ex["speed"]))))
    return n, bad


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", required=True)
    ap.add_argument("--db", required=True)
    ap.add_argument("--feeds", required=True)
    a = ap.parse_args(argv)
    meta = {m["file"]: m for m in json.load(open(os.path.join(a.feeds, "slide_list.json")))}
    feeds = sorted(glob.glob(os.path.join(a.feeds, "slide_*.feed")))
    n, bad = gate(run(a.exe, a.db, feeds, True), meta)
    print("real run: %d full slides, %d violations" % (n, len(bad)))
    for b in bad:
        print("  VIOLATION", b)
    n2, bad2 = gate(run(a.exe, a.db, feeds, False), meta)
    print("mutation (slide off): %d full slides, %d violations (must be > 0)" % (n2, len(bad2)))
    ok = n >= 3 and not bad and len(bad2) > 0
    print("passed" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
