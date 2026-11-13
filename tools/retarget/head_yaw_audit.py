#!/usr/bin/env python3
"""Head-yaw audit and `headlock` posture class (posture_offsets.json -> classes.headlock), offline.

  measure  -> per clip of --dirs (clip directories; optional --keep list, see head_yaw_clip_list.py) the head (and Neck) yaw
              relative to Spine3: max |yaw|, mean |yaw|, frames > threshold, clip family (Turn/Pivot/Reface/Loop/Start/Stop/
              Other); table sorted by max |head yaw|
  apply    -> staging dir (default out_posture_headlock_v5) with every clip of the list; clips whose max |head yaw| > threshold
              (source pack clips only with --all-sources) get Neck/Neck1/Head yaw relative to Spine3
              zeroed (pitch/roll kept), weighted: loops w = 1 everywhere, other clips a 0.1 s smoothstep ramp at both edges with at most
              edge_residual_deg of yaw left at an edge frame (see weights()). Everything else is copied byte for byte.

Definition (game-local x,y,z,w quaternions, Z up, +Y forward, FK from mmbake): for B in Neck/Neck1/Head
    R_B = conj(G_Spine3) * G_B            (B in the Spine3 frame)
    D_B = R_B * conj(R0_B)                (deviation from the neutral relative pose R0_B = rig bind pose Spine3 -> B)
    yaw = heading of D_B * f0 about the chest axis a (a = Spine3 -> Neck bind translation = local +X, f0 = rig forward
          = local -Y); positive = counter-clockwise seen from above (head turns to the left)
    (Not relative to the source pack idle: source pack stand idle itself turns the head ~22 deg against the chest.)
    correction: R_B' = Rot(a, -w*yaw) * R_B  (so heading is zeroed, pitch/roll untouched, Neck, Neck1 and Head each relative to Spine3).
Originals are never overwritten (dst must differ from every source dir).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
import mmbake_base as mb  # noqa: E402  (qmul, qrot, fk_model, load_rig)

RIG = os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json")
OFFSETS = os.path.join(HERE, "posture_offsets.json")
R_ = os.path.join(ROOT, "Animations", "retarget")
DST = os.path.join(R_, "out_posture_headlock_v5")
LOGS = os.path.join(ROOT, "Research", "logs")
NB = 73
CLIP_SUFFIX = ".utppclip.json"
SPINE = "Spine3"
CHAIN = ("Neck", "Neck1", "Head")
FAMILIES = ("Pivot", "Turn", "Reface", "Loop", "Start", "Stop")


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qnorm(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def family(name):
    for f in FAMILIES:
        if re.search(r"_%s(_|$)" % f, name):
            return f
    return "Other"


def load_class(path=OFFSETS):
    return json.load(open(path, encoding="utf-8"))["classes"]["headlock"]


def read_clip(path):
    c = json.load(open(path, encoding="utf-8"))
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    return c, B


def find_clips(dirs, keep_path):
    keep = None
    if keep_path:
        keep = [c["name"] for c in json.load(open(keep_path, encoding="utf-8"))["clips"]]
    files = {}
    for d in dirs:
        for f in glob.glob(os.path.join(d, "*" + CLIP_SUFFIX)):
            n = os.path.basename(f)[:-len(CLIP_SUFFIX)]
            if n in files:
                raise SystemExit("duplicate clip across dirs: " + n)
            files[n] = f
    names = sorted(files) if keep is None else sorted(keep)
    miss = [n for n in names if n not in files]
    if miss:
        raise SystemExit("clips of the list not found: %s" % miss[:5])
    return [(n, files[n]) for n in names]


def qmean(q):
    q = q * np.where(np.sum(q * q[0], axis=-1) < 0, -1.0, 1.0)[:, None]
    return qnorm(q.mean(axis=0))


class Frame:
    """Neutral relative poses R0_B (rig bind pose), chest axis a and neutral forward f0 in the Spine3 frame."""

    def __init__(self, names, parents):
        self.names, self.parents = names, parents
        self.bi = {n: i for i, n in enumerate(names)}
        rig = json.load(open(RIG, encoding="utf-8"))
        bt = mb.find_key(rig, "boneTransforms")[:NB]
        lq = np.array([[t["Rotation"][k] for k in ("i", "j", "k", "r")] for t in bt], dtype=np.float64)
        lp = np.array([[t["Translation"][k] for k in ("X", "Y", "Z")] for t in bt], dtype=np.float64)
        s = self.bi[SPINE]
        self.R0 = {}
        r = np.array([0.0, 0.0, 0.0, 1.0])
        for n in CHAIN:                                        # Spine3 -> Neck -> Neck1 -> Head (direct parent chain)
            r = mb.qmul(r[None], lq[self.bi[n]][None])[0]
            self.R0[n] = r
        assert parents[self.bi["Neck"]] == s and parents[self.bi["Neck1"]] == self.bi["Neck"]             and parents[self.bi["Head"]] == self.bi["Neck1"]
        a = lp[self.bi["Neck"]]                                # Spine3 -> Neck, Spine3 frame (bone axis, local +X)
        self.a = a / np.linalg.norm(a)
        f = np.array([0.0, -1.0, 0.0])                         # rig forward = local -Y (Hips local Y -> model -Y)
        f = f - np.dot(f, self.a) * self.a
        self.f0 = f / np.linalg.norm(f)

    def yaw(self, gq, bone):
        """(N,) heading (rad) of the bone relative to Spine3 (see module doc) and R (N,4)."""
        R = mb.qmul(qconj(gq[:, self.bi[SPINE]]), gq[:, self.bi[bone]])
        D = mb.qmul(R, qconj(self.R0[bone])[None])
        v = mb.qrot(D, np.broadcast_to(self.f0, (len(D), 3)))
        hv = v - np.sum(v * self.a, axis=-1, keepdims=True) * self.a
        s = np.sum(self.a * np.cross(self.f0, hv), axis=-1)
        c = np.sum(self.f0 * hv, axis=-1)
        return np.arctan2(s, c), R

    def rot_a(self, ang):
        q = np.zeros(ang.shape + (4,))
        q[..., 0:3] = self.a * np.sin(ang / 2)[..., None]
        q[..., 3] = np.cos(ang / 2)
        return q


def edge_weight(n, fps, blend_s):
    i = np.arange(n, dtype=np.float64)
    k = max(blend_s * fps, 1e-6)
    return smoothstep(np.minimum(i, n - 1 - i) / k)


def weights(y, fps, loop, cls):
    """Per-frame correction weight. Loops: 1 everywhere (the correction is a pure function of the frame pose, so a closing loop
    keeps closing and there is no seam whip). Other clips: 0.1 s smoothstep ramp at both edges (edge frames keep the original
    yaw), but never leaving more than edge_residual_deg of yaw at a frame (a clip that starts/ends with a large constant yaw
    would otherwise whip by that angle within 0.1 s): w = max(ramp, 1 - edge_residual_deg/|yaw|)."""
    n = len(next(iter(y.values())))
    if loop:
        return np.ones(n)
    ymax = np.max([np.abs(v) for v in y.values()], axis=0)
    cap = np.clip(1.0 - cls["edge_residual_deg"] / np.maximum(ymax, 1e-9), 0.0, 1.0)
    return np.maximum(edge_weight(n, fps, cls["blend_s"]), cap)


def clip_yaws(fr, B, names, parents):
    gp, gq = mb.fk_model(B[..., 0:3], B[..., 3:7], parents)
    return {n: np.degrees(fr.yaw(gq, n)[0]) for n in CHAIN}


def correct(P, Q, fr, w):
    """Zero Neck/Neck1/Head yaw relative to Spine3 with weight w (N,). Returns corrected local Q (N,73,4)."""
    names, parents, bi = fr.names, fr.parents, fr.bi
    gp, gq = mb.fk_model(P, Q, parents)
    Q2 = Q.copy()
    G2 = gq.copy()
    for n in CHAIN:
        b = bi[n]
        yaw, R = fr.yaw(gq, n)
        Rn = mb.qmul(fr.rot_a(-w * yaw), R)
        tgt = mb.qmul(gq[:, bi[SPINE]], Rn)
        p = parents[b]
        Q2[:, b] = mb.qmul(qconj(G2[:, p]), tgt)
        G2[:, b] = tgt
    return qnorm(Q2)


# ---------------------------------------------------------------- measure

def table(rows, thr, title, extra_cols=None):
    L = [title,
         "%-4s %-60s %-5s %4s %7s %7s %6s %6s %7s %7s" % (
             "#", "clip", "fam", "fr", "maxHead", "meanHd", "fr>%d" % thr, "frac%", "maxNeck", "atFrame")]
    for i, r in enumerate(rows, 1):
        L.append("%-4d %-60s %-5s %4d %6.1fd %6.1fd %6d %5.0f%% %6.1fd %7d" % (
            i, r["name"], r["fam"][:5], r["n"], r["max"], r["mean"], r["over"], 100.0 * r["over"] / r["n"],
            r["neck_max"], r["argmax"]))
    return L


def measure_dirs(dirs, keep, offsets, ref_dir=None):
    names, parents = mb.load_rig(RIG, NB)
    items = find_clips(dirs, keep)
    fr = Frame(names, parents)
    thr = load_class(offsets)["yaw_threshold_deg"]
    rows = []
    for n, f in items:
        c, B = read_clip(f)
        y = clip_yaws(fr, B, names, parents)
        h = np.abs(y["Head"])
        rows.append({"name": n, "fam": family(n), "n": len(h), "max": float(h.max()),
                     "mean": float(h.mean()), "over": int((h > thr).sum()), "neck_max": float(np.abs(y["Neck"]).max()),
                     "argmax": int(h.argmax()), "yaw": y["Head"]})
    rows.sort(key=lambda r: -r["max"])
    return rows, fr, thr


def cmd_measure(a):
    dirs = a.dirs.split(";")
    rows, fr, thr = measure_dirs(dirs, a.keep, a.offsets)
    L = ["# Head yaw relative to Spine3 (deg, |yaw|), definition: see head_yaw_audit.py docstring. Neutral = rig bind pose; chest "
         "axis a = Spine3->Neck = local +X, forward f0 = local -Y; threshold %g deg." % thr,
         "# dirs: " + "; ".join(os.path.relpath(d, ROOT) for d in dirs) + "; list: %s; clips: %d" % (
             os.path.relpath(a.keep, ROOT) if a.keep else "all", len(rows)),
         "# columns: maxHead = max |Head yaw|, meanHd = mean |Head yaw|, fr>25 = frames over threshold, maxNeck = max |Neck yaw|, "
         "atFrame = frame of the max", ""]
    L += table(rows, thr, "## all clips, sorted by max |head yaw|")
    # per family summary
    L += ["", "## per family (max of max, mean of mean, clips with max > threshold)"]
    keys = sorted({r["fam"] for r in rows})
    L.append("%-7s %4s %8s %8s %6s" % ("family", "n", "maxMax", "meanMean", "n>thr"))
    for f in keys:
        g = [r for r in rows if r["fam"] == f]
        L.append("%-7s %4d %7.1fd %7.1fd %6d" % (f, len(g), max(r["max"] for r in g),
                                                    float(np.mean([r["mean"] for r in g])), sum(r["max"] > thr for r in g)))
    txt = "\n".join(L) + "\n"
    print("\n".join(L[:3] + [""] + L[4:4 + 1 + a.top]) if a.top else txt)
    out = a.out
    if out is None:
        out = os.path.join(LOGS, "head-yaw-v4_clips.txt")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)
    print("MEASURE_OK %d clips -> %s" % (len(rows), out))


# ---------------------------------------------------------------- apply

def cmd_apply(a):
    dirs = a.dirs.split(";")
    if any(os.path.abspath(a.dst) == os.path.abspath(d) for d in dirs):
        raise SystemExit("dst must differ from every source dir (originals are never overwritten)")
    cls = load_class(a.offsets)
    names, parents = mb.load_rig(RIG, NB)
    items = find_clips(dirs, a.keep)
    fr = Frame(names, parents)
    thr = cls["yaw_threshold_deg"]
    os.makedirs(a.dst, exist_ok=True)
    report = []
    ncopy = 0
    for n, f in items:
        out = os.path.join(a.dst, n + CLIP_SUFFIX)
        c, B = read_clip(f)
        y = clip_yaws(fr, B, names, parents)
        mx = float(np.abs(y["Head"]).max())
        by_thr = mx > thr and (a.all_sources or re.search(cls["threshold_exclude_regex"], n) is None)
        if not by_thr:
            shutil.copyfile(f, out)
            ncopy += 1
            continue
        P, Q = B[..., 0:3], B[..., 3:7]
        is_loop = re.search(cls["loop_regex"], n) is not None
        w = weights(y, c.get("fps", 30), is_loop, cls)
        Q2 = correct(P, Q, fr, w)
        for frm, pf, qf in zip(c["frames"], P, Q2):
            frm["bones"] = [[float(x) for x in pp] + [float(x) for x in qq] + list(bb[7:10])
                            for pp, qq, bb in zip(pf, qf, frm["bones"])]
        B2 = np.array([x["bones"] for x in c["frames"]], dtype=np.float64)
        y2 = clip_yaws(fr, B2, names, parents)
        mx2 = float(np.abs(y2["Head"]).max())
        c["posture_class"] = {"class": "headlock", "file": os.path.basename(a.offsets), "reason": "regex" if by_regex else "threshold", "loop": is_loop, "w_min": float(w.min()),
                              "head_yaw_max_before_deg": round(mx, 3), "head_yaw_max_after_deg": round(mx2, 3)}
        with open(out, "w", encoding="utf-8", newline="\n") as fo:
            json.dump(c, fo, separators=(",", ":"))
        report.append({"name": n, "reason": c["posture_class"]["reason"], "loop": is_loop, "before": mx, "after": mx2,
                       "mean_before": float(np.abs(y["Head"]).mean()), "mean_after": float(np.abs(y2["Head"]).mean()),
                       "neck_before": float(np.abs(y["Neck"]).max()), "neck_after": float(np.abs(y2["Neck"]).max())})
    report.sort(key=lambda r: -r["before"])
    L = ["# headlock apply: modified %d, copied unchanged %d -> %s" % (len(report), ncopy, os.path.relpath(a.dst, ROOT)),
         "# max |head yaw| (deg) before/after; after > 0 only for non-loop clips (0.1 s edge ramp, residual <= edge_residual_deg); "
         "loops are zeroed on every frame",
         "%-4s %-60s %-9s %-4s %8s %8s %8s %8s %8s %8s" % ("#", "clip", "reason", "loop", "maxBefor", "maxAfter", "meanBef", "meanAft",
                                                       "neckBef", "neckAft")]
    for i, r in enumerate(report, 1):
        L.append("%-4d %-60s %-9s %-4s %7.1fd %7.1fd %7.1fd %7.1fd %7.1fd %7.1fd" % (
            i, r["name"], r["reason"], "yes" if r["loop"] else "no", r["before"], r["after"], r["mean_before"], r["mean_after"], r["neck_before"],
            r["neck_after"]))
    txt = "\n".join(L) + "\n"
    print(txt)
    out = a.out or os.path.join(LOGS, "head-yaw-v5_apply.txt")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)
    print("APPLY_OK modified %d, copied unchanged %d -> %s" % (len(report), ncopy, a.dst))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    for nm, fn in (("measure", cmd_measure), ("apply", cmd_apply)):
        p = sp.add_parser(nm)
        p.add_argument("--dirs", required=True, help="';'-separated clip dirs")
        p.add_argument("--keep", default="", help="clip list (.summary.json/.keep.json with 'clips'); '' = every clip in --dirs")
        p.add_argument("--offsets", default=OFFSETS)
        p.add_argument("--out", default=None)
        p.set_defaults(fn=fn)
        if nm == "apply":
            p.add_argument("--dst", default=DST)
            p.add_argument("--all-sources", action="store_true",
                           help="apply the threshold rule to source pack clips too (default: threshold_exclude_regex of the class skips them)")
        else:
            p.add_argument("--top", type=int, default=0, help="print only the top N rows (the file always has all)")
    a = ap.parse_args()
    a.keep = a.keep or None
    a.fn(a)


if __name__ == "__main__":
    main()
