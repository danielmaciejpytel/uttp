#!/usr/bin/env python3
"""UTPP Phase 5.3: speed-vs-time curves of start/stop clips from a .utppmm database (ROOT section).

Reads only header, section table, STRS, CLIP and ROOT (works on the 176 MB full DB without loading POSE).
For every clip whose name matches the start/stop patterns it computes the planar root speed per frame
(|ROOT.dx,dy| * fps), then:
  starts: t10/t50/t90 = first time speed reaches 10/50/90 % of the clip's plateau speed (median of the
          last 25 % of the clip), mean acceleration 10->90 %, peak acceleration (5-frame smoothed);
  stops:  initial speed (median of the first 3 frames), time to 10 % of it and to < 0.05 m/s,
          mean and peak deceleration.
Per group (Walk/Run/Sprint/Crouch start/stop) the medians are reported and converted to game units with
the UTPP per-state speed mapping (game speed = db speed * game_loop / db_loop, i.e. the same time base;
the runtime time-scales the animation by rate = game/db, so the game-time duration of the clip is
t_db / rate and the game acceleration = a_db * ratio^2 / ... see GameAccel below).

Usage: python measure_root_speed.py <db.utppmm> [--json out.json] [--curves]
"""
import argparse
import json
import math
import re
import statistics
import struct
import sys

# Game <- database loop speeds (MMDriver g_speedMap defaults, measured in game 2.31 TPP).
SPEED_MAP = {
    "Walk": (1.6, 2.19),
    "Run": (3.85, 5.46),
    "Sprint": (8.2, 7.65),
    "Crouch": (2.7, 2.46),
}
RATE_MIN, RATE_MAX = 0.6, 1.4

GROUPS = [
    ("Walk_Start", re.compile(r"Stand_.*Walk.*Start|_Walk_Start", re.I)),
    ("Run_Start", re.compile(r"_Run_Start", re.I)),
    ("Sprint_Start", re.compile(r"_Sprint_Start", re.I)),
    ("Crouch_Start", re.compile(r"Crouch.*_Start", re.I)),
    ("Walk_Stop", re.compile(r"_Walk_Stop", re.I)),
    ("Run_Stop", re.compile(r"_Run_Stop", re.I)),
    ("Sprint_Stop", re.compile(r"_Sprint_Stop", re.I)),
    ("Crouch_Stop", re.compile(r"Crouch.*_Stop", re.I)),
]
EXCLUDE = re.compile(r"Reface|Turn|Pivot|_to_|Lean|Jump|Land|Traversal", re.I)


def read_db(path):
    with open(path, "rb") as f:
        hdr = f.read(128)
        if hdr[:8] != b"UTPPMMDB":
            raise SystemExit("not a utppmm file")
        num_frames, = struct.unpack_from("<I", hdr, 36)
        num_clips, = struct.unpack_from("<I", hdr, 48)
        fps, = struct.unpack_from("<f", hdr, 80)
        nsec, tab = struct.unpack_from("<II", hdr, 84)
        f.seek(tab)
        secs = {}
        for _ in range(nsec):
            sid, count, esz, _r, off, size = struct.unpack("<IIIIQQ", f.read(32))
            secs[struct.pack("<I", sid).decode("ascii")] = (count, esz, off, size)

        def blob(name):
            c, e, o, s = secs[name]
            f.seek(o)
            return f.read(s)

        strs = blob("STRS")
        clips = []
        cb = blob("CLIP")
        for i in range(num_clips):
            noff, first, n, flags, period, nsearch, tags = struct.unpack_from("<IIIIIIQ", cb, i * 32)
            name = strs[noff:strs.index(b"\0", noff)].decode("utf-8")
            clips.append({"name": name, "first": first, "n": n, "loop": bool(flags & 1)})
        rb = blob("ROOT")
        root = [struct.unpack_from("<ffff", rb, i * 16) for i in range(num_frames)]
    return fps, clips, root


def smooth(v, k=2):
    out = []
    for i in range(len(v)):
        a, b = max(0, i - k), min(len(v), i + k + 1)
        out.append(sum(v[a:b]) / (b - a))
    return out


def first_time(sp, thr, fps):
    for i, s in enumerate(sp):
        if s >= thr:
            if i == 0:
                return 0.0
            p = sp[i - 1]
            frac = (thr - p) / (s - p) if s != p else 0.0
            return (i - 1 + frac) / fps
    return None


def first_below(sp, thr, fps):
    for i, s in enumerate(sp):
        if s <= thr:
            return i / fps
    return None


def analyse(fps, clips, root, curves=False):
    res = {}
    for gname, rx in GROUPS:
        rows = []
        for c in clips:
            if c["loop"] or not rx.search(c["name"]) or EXCLUDE.search(c["name"]):
                continue
            n = c["n"] - 1  # last ROOT step is an extrapolation copy
            if n < 6:
                continue
            sp = [math.hypot(*root[c["first"] + i][:2]) * fps for i in range(n)]
            ss = smooth(sp)
            acc = [(ss[i + 1] - ss[i]) * fps for i in range(n - 1)]
            row = {"clip": c["name"], "frames": c["n"], "dur": round(n / fps, 3)}
            if "Start" in gname:
                plateau = statistics.median(sp[max(0, n - max(3, n // 4)):])
                row["plateau"] = round(plateau, 3)
                t10, t50, t90 = (first_time(ss, p * plateau, fps) for p in (0.1, 0.5, 0.9))
                row.update(t10=t10, t50=t50, t90=t90)
                if t10 is not None and t90 is not None and t90 > t10:
                    row["a_mean"] = round(0.8 * plateau / (t90 - t10), 3)
                row["a_peak"] = round(max(acc), 3)
            else:
                # Stop clips begin with a constant-speed lead-in; times are measured from the braking onset
                # (first frame of the raw speed below 95 % of v0) and the distance from the onset as well.
                v0 = statistics.median(sp[:3])
                row["v0"] = round(v0, 3)
                onset = next((i for i, s in enumerate(sp) if s < 0.95 * v0), 0)
                row["onset"] = round(onset / fps, 3)
                rest = sp[onset:]
                t10 = first_below(rest, 0.1 * v0, fps)
                row["t10"] = t10
                row["t_stop"] = first_below(rest, 0.05, fps)
                if t10:
                    row["d_mean"] = round(0.85 * v0 / t10, 3)  # from 95 % to 10 %
                row["d_peak"] = round(-min(acc), 3)
                row["dist"] = round(sum(rest) / fps, 3)
            if curves:
                row["speed"] = [round(s, 3) for s in sp]
            rows.append(row)
        res[gname] = rows
    return res


def med(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    return statistics.median(vals) if vals else None


def game_scale(gname):
    """Return (speed ratio game/db, time scale) for the state of the group.
    Runtime plays the clip at rate = clamp(game/db) (MMDriver), so a db duration T lasts T/rate in game time and a
    db speed v corresponds to game speed v*ratio. Game acceleration = a_db * ratio * rate."""
    base = gname.split("_")[0]
    g, d = SPEED_MAP[base]
    ratio = g / d
    rate = min(RATE_MAX, max(RATE_MIN, ratio))
    return ratio, rate


def summary(res):
    out = {}
    for gname, rows in res.items():
        if not rows:
            out[gname] = None
            continue
        ratio, rate = game_scale(gname)
        s = {"clips": len(rows), "ratio": round(ratio, 3), "rate": round(rate, 3)}
        if "Start" in gname:
            for k in ("plateau", "t10", "t50", "t90", "a_mean", "a_peak"):
                s[k] = med(rows, k)
            s["game_plateau"] = s["plateau"] * ratio
            for k in ("t10", "t50", "t90"):
                s["game_" + k] = s[k] / rate if s[k] is not None else None
            s["game_a_mean"] = s["a_mean"] * ratio * rate if s["a_mean"] else None
        else:
            for k in ("v0", "t10", "t_stop", "d_mean", "d_peak", "dist"):
                s[k] = med(rows, k)
            s["game_v0"] = s["v0"] * ratio
            s["game_t10"] = s["t10"] / rate if s["t10"] else None
            s["game_d_mean"] = s["d_mean"] * ratio * rate if s["d_mean"] else None
            s["game_dist"] = s["dist"] * ratio if s["dist"] else None
        out[gname] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("--json")
    ap.add_argument("--curves", action="store_true")
    a = ap.parse_args()
    fps, clips, root = read_db(a.db)
    res = analyse(fps, clips, root, a.curves)
    summ = summary(res)
    print(f"db={a.db} fps={fps} clips={len(clips)}")
    for g, rows in res.items():
        print(f"== {g}: {len(rows)} clips")
        for r in rows:
            print("  " + " ".join(f"{k}={v}" for k, v in r.items() if k != "speed"))
        print("  SUMMARY " + json.dumps(summ[g]))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"db": a.db, "fps": fps, "speed_map": SPEED_MAP, "summary": summ, "clips": res}, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
