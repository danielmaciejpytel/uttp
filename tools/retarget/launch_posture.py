#!/usr/bin/env python3
"""Run/sprint start launch-lean cap for the source pack standing start clips (posture_offsets.json -> classes.launch), offline.

  apply    -> staging dir of *.utppclip.json: every clip of --src is copied; clips matching the class regex get the
              correction (trunk lean soft-capped by a backward pitch over pelvis + spine, hips raised, leg IK keeping the ankles)
  measure  -> per-clip table (lean max before/after, frames changed, hips raise, knee flexion, ankle / toe error, head lift,
              knee gate) of --src vs --dst clip dirs, or a lean table of baked .utppmm files (--db)

Why: the source pack M_Neutral_Run_Start_F_* / Sprint_Start_F_* clips lean the trunk (Hips->Neck vs vertical) to 57-61 deg for
~0.3 s with the hips ~20 cm low (source FBX and baked DBs agree); the run cycle that follows leans 20-31 deg and the source pack
Relaxed run start peaks at 41.6 deg. In game the launch reads as V folding almost horizontally (2026-11-09 gametest).

The correction is a pure function of each frame's ORIGINAL pose. lean = angle(Neck - Hips, +Z) in the model (character)
frame; frames whose lean is at or below knee_deg are bit-exact, so the run / sprint cycle at the end of a start (and every
other clip) stays untouched. Above the knee the lean is mapped to knee + (cap - knee) * tanh((lean - knee) / (cap - knee))
(C1 at the knee, asymptote cap_deg) and the excess is removed by a backward pitch about the horizontal axis perpendicular to
the frame's lean direction: pelvis_share of it on Hips (pivot = Hips joint), the rest over spine_chain (cumulative fraction
per bone, model-space pitch added to the ORIGINAL global orientation); the applied angle is iterated so the result lands on
the capped lean. Neck, head, shoulders and arms keep their pose relative to the chest (the head lifts with the torso).
Hips raised by raise_m_per_deg * removed lean (never above the clip's frame-0 height - min_drop_m, and never so far that a
leg would straighten past reach_max of its length, smoothed over lift_smooth_frames). Legs: analytic two-bone
IK keeps both ankles at their ORIGINAL model-space positions with the original knee hinge carried (crouch_posture knee-fix
construction); foot orientation restored, so feet / toes (the DB foot features) do not move.
Frame data: game-local x,y,z,w quaternions, Z up, +Y forward (mmbake FORMAT). Originals are never overwritten (dst must
differ from src).
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
sys.path.insert(0, HERE)
import crouch_posture as cp  # noqa: E402  (mb, qconj, qnorm, qmul_, arc, basis_q, read_clip, knee_gate_clip, ROOT)

mb = cp.mb
ROOT = cp.ROOT
RIGS = {"woman": os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json"),
        "man": os.path.join(ROOT, "Extracted", "game", "json", "vanilla-man", "player_man_skeleton.rig.json")}
OFFSETS = cp.OFFSETS
NB = cp.NB
CLIP_SUFFIX = cp.CLIP_SUFFIX
ITERATIONS = 4


def load_class(path=OFFSETS):
    return json.load(open(path, encoding="utf-8"))["classes"]["launch"]


def matches(name, cls):
    return re.search(cls["include_regex"], name) is not None and re.search(cls["exclude_regex"], name) is None


def rest_global(rig_path, parents):
    rc = json.load(open(rig_path, encoding="utf-8"))["Data"]["RootChunk"]
    P0 = np.zeros((1, NB, 3))
    Q0 = np.zeros((1, NB, 4))
    for b, t in enumerate(rc["boneTransforms"][:NB]):
        P0[0, b] = [t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"]]
        Q0[0, b] = [t["Rotation"]["i"], t["Rotation"]["j"], t["Rotation"]["k"], t["Rotation"]["r"]]
    gp, gq = mb.fk_model(P0, Q0, parents)
    return gp[0], gq[0]


def axis_angle(axis, ang):
    """(N,3) unit axes, (N,) radians -> (N,4) x,y,z,w."""
    s = np.sin(ang / 2.0)[:, None]
    return np.concatenate([axis * s, np.cos(ang / 2.0)[:, None]], axis=-1)


def softcap(theta, knee, cap):
    out = np.array(theta, dtype=np.float64, copy=True)
    m = out > knee
    span = cap - knee
    out[m] = knee + span * np.tanh((out[m] - knee) / span)
    return out


def lean_deg(gp, bi):
    v = gp[:, bi["Neck"]] - gp[:, bi["Hips"]]
    return np.degrees(np.arctan2(np.linalg.norm(v[:, :2], axis=1), v[:, 2])), v


def build(P, Q, gp, gq, names, parents, cls, axis, ang, lift, rest_gp, rest_gq):
    """Pose with the pitch ang (N,) rad about axis (N,3) distributed over Hips + spine and the hips lifted by lift (N,) m;
    legs re-solved to the original ankles. Returns local P2, Q2 and model-space GP2, G2."""
    bi = {n: i for i, n in enumerate(names)}
    N = P.shape[0]
    P2, Q2 = P.copy(), Q.copy()
    G2, GP2 = gq.copy(), gp.copy()
    share = float(cls["pelvis_share"])
    frac = {bi["Hips"]: share}
    for n, f in cls["spine_chain"].items():
        frac[bi[n]] = share + (1.0 - share) * float(f)
    lat_rest = rest_gp[bi["LeftUpLeg"]] - rest_gp[bi["RightUpLeg"]]
    lat_rest = lat_rest / np.linalg.norm(lat_rest)
    legs = {}
    for side in ("Left", "Right"):
        legs[bi[side + "UpLeg"]] = (side, "up")
        legs[bi[side + "Leg"]] = (side, "leg")
        legs[bi[side + "Foot"]] = (side, "foot")
    knee_new = {}
    hips = bi["Hips"]
    for b, p in enumerate(parents):
        if p < 0:
            continue
        if b in frac:
            tgt = cp.qnorm(cp.qmul_(axis_angle(axis, frac[b] * ang), gq[:, b]))
            Q2[:, b] = cp.qmul_(cp.qconj(G2[:, p]), tgt)
            G2[:, b] = tgt
            if b == hips:
                pos = gp[:, b].copy()
                pos[:, 2] += lift
                P2[:, b] = mb.qrot(cp.qconj(G2[:, p]), pos - GP2[:, p])
                GP2[:, b] = pos
            else:
                GP2[:, b] = GP2[:, p] + mb.qrot(G2[:, p], P[:, b])
        elif b in legs:
            side, kind = legs[b]
            up, kn, ft = bi[side + "UpLeg"], bi[side + "Leg"], bi[side + "Foot"]
            if kind == "up":
                hip = GP2[:, p] + mb.qrot(G2[:, p], P[:, b])     # new thigh root (Hips pitched / lifted)
                a_len = np.linalg.norm(P[:, kn], axis=-1)
                b_len = np.linalg.norm(P[:, ft], axis=-1)
                tgt = gp[:, ft]                                    # ankle stays where it was
                v = tgt - hip
                d = np.clip(np.linalg.norm(v, axis=-1), np.abs(a_len - b_len) + 1e-4, a_len + b_len - 1e-4)
                u = v / np.linalg.norm(v, axis=-1, keepdims=True)
                x = (a_len ** 2 - b_len ** 2 + d ** 2) / (2 * d)
                h = np.sqrt(np.maximum(a_len ** 2 - x ** 2, 0.0))
                a_loc = mb.qrot(cp.qconj(rest_gq[up][None]), lat_rest[None])[0]
                n0 = mb.qrot(gq[:, up], np.broadcast_to(a_loc, (N, 3)).copy())
                u0 = gp[:, ft] - gp[:, up]
                n = mb.qrot(cp.arc(u0, u), n0)
                n = n - np.sum(n * u, -1, keepdims=True) * u
                n = n / np.linalg.norm(n, axis=-1, keepdims=True)
                pole = np.cross(u, n)
                knee = hip + x[:, None] * u + h[:, None] * pole
                dir0 = gp[:, kn] - gp[:, up]
                R = cp.qmul_(cp.basis_q(knee - hip, n), cp.qconj(cp.basis_q(dir0, n0)))
                g_up = cp.qnorm(cp.qmul_(R, gq[:, up]))
                Q2[:, b] = cp.qmul_(cp.qconj(G2[:, p]), g_up)
                G2[:, b] = g_up
                GP2[:, b] = hip
                knee_new[side] = (knee, tgt, n, n0)
            elif kind == "leg":
                knee, tgt, n, n0 = knee_new[side]
                sh0 = gp[:, ft] - gp[:, kn]
                R = cp.qmul_(cp.basis_q(tgt - knee, n), cp.qconj(cp.basis_q(sh0, n0)))
                g_leg = cp.qnorm(cp.qmul_(R, gq[:, b]))
                Q2[:, b] = cp.qmul_(cp.qconj(G2[:, p]), g_leg)
                G2[:, b] = g_leg
                GP2[:, b] = GP2[:, p] + mb.qrot(G2[:, p], P[:, b])
            else:  # foot: original model-space orientation
                Q2[:, b] = cp.qmul_(cp.qconj(G2[:, p]), gq[:, b])
                G2[:, b] = gq[:, b]
                GP2[:, b] = GP2[:, p] + mb.qrot(G2[:, p], P[:, b])
        else:
            G2[:, b] = cp.qmul_(G2[:, p], Q2[:, b])
            GP2[:, b] = GP2[:, p] + mb.qrot(G2[:, p], P2[:, b])
    Q2 = cp.qnorm(Q2)
    return P2, Q2, GP2, G2


def reach_lift_cap(P, gp, GP0, bi, reach_max):
    """Largest hips lift (N,) m >= 0 that keeps both ankles reachable: |ankle - (hip + lift Z)| <= max(reach_max, r0) *
    (thigh + shin), r0 = reach of the pitched pose without lift (the lift never straightens a leg past reach_max or past
    where the pitch alone put it). GP0 = model positions of the pitched pose with lift 0 (thigh root = hip + lift Z)."""
    cap = np.full(P.shape[0], np.inf)
    for s in ("Left", "Right"):
        up, kn, ft = bi[s + "UpLeg"], bi[s + "Leg"], bi[s + "Foot"]
        L = np.linalg.norm(P[:, kn], axis=-1) + np.linalg.norm(P[:, ft], axis=-1)
        w = GP0[:, up] - gp[:, ft]
        lim = np.maximum(reach_max, np.linalg.norm(w, axis=-1) / L) * L
        disc = w[:, 2] ** 2 - np.sum(w * w, axis=-1) + lim ** 2
        cap = np.minimum(cap, np.maximum(-w[:, 2] + np.sqrt(np.maximum(disc, 0.0)), 0.0))
    return cap


def smooth_cut(deficit, r):
    """Max filter then box blur, both radius r (truncated at the clip ends): result >= deficit on every frame, smooth."""
    if r <= 0:
        return deficit
    n = len(deficit)
    dil = np.array([deficit[max(0, i - r):i + r + 1].max() for i in range(n)])
    return np.array([dil[max(0, i - r):i + r + 1].mean() for i in range(n)])


def correct(P, Q, names, parents, cls, rest_gp, rest_gq):
    """P (N,73,3), Q (N,73,4) local -> corrected copies (frames at or below the knee bit-exact), info dict."""
    bi = {n: i for i, n in enumerate(names)}
    gp, gq = mb.fk_model(P, Q, parents)
    lean0, v = lean_deg(gp, bi)
    target = softcap(lean0, float(cls["knee_deg"]), float(cls["cap_deg"]))
    excess = lean0 - target                                     # deg, >= 0, 0 at / below the knee
    on = excess > 0.0
    vh = v.copy()
    vh[:, 2] = 0.0
    ln = np.linalg.norm(vh, axis=1, keepdims=True)
    d = np.where(ln > 1e-9, vh / np.maximum(ln, 1e-12), np.array([[0.0, 1.0, 0.0]]))
    axis = np.stack([d[:, 1], -d[:, 0], np.zeros(len(d))], axis=1)  # (lean dir) x Z: +angle tilts the trunk back up
    z0 = gp[0, bi["Hips"], 2]
    hz = gp[:, bi["Hips"], 2]
    lift = np.minimum(float(cls["raise_m_per_deg"]) * excess, np.maximum(z0 - float(cls["min_drop_m"]) - hz, 0.0))
    lift[~on] = 0.0
    ang = np.radians(excess)
    for _ in range(ITERATIONS):
        P2, Q2, GP2, G2 = build(P, Q, gp, gq, names, parents, cls, axis, ang, lift, rest_gp, rest_gq)
        lean2, _ = lean_deg(GP2, bi)
        got = lean0 - lean2
        m = on & (got > 1e-6)
        ang[m] *= excess[m] / got[m]
    # the lift translates the whole upper body, so the lean (and ang) does not depend on it; cap it where a nearly straight
    # leg could no longer reach its ankle (push-off of the sprint starts), smoothly so the hips height has no kink
    GP0 = build(P, Q, gp, gq, names, parents, cls, axis, ang, np.zeros_like(lift), rest_gp, rest_gq)[2]
    deficit = np.maximum(lift - reach_lift_cap(P, gp, GP0, bi, float(cls.get("reach_max", 0.995))), 0.0)
    lift_capped = int(np.sum(deficit > 0.0))
    if lift_capped:
        lift = np.maximum(lift - smooth_cut(deficit, int(cls.get("lift_smooth_frames", 4))), 0.0)
        lift[~on] = 0.0
    P2, Q2, GP2, G2 = build(P, Q, gp, gq, names, parents, cls, axis, ang, lift, rest_gp, rest_gq)
    P2[~on] = P[~on]
    Q2[~on] = Q[~on]
    lean2, _ = lean_deg(mb.fk_model(P2, Q2, parents)[0], bi)
    info = {"lean_in_max": float(lean0.max()), "lean_out_max": float(lean2.max()), "frames": int(len(lean0)),
            "frames_changed": int(on.sum()), "excess_max": float(excess.max()), "lift_max_m": float(lift.max()),
            "lift_reach_capped_frames": lift_capped,
            "applied_deg_max": float(np.degrees(ang[on]).max()) if on.any() else 0.0,
            "target_err_deg": float(np.abs(lean2 - target)[on].max()) if on.any() else 0.0}
    return P2, Q2, info


def apply_dir(src, dst, cls, rig_path, check_gate=True):
    names, parents = mb.load_rig(rig_path, NB)
    rest_gp, rest_gq = rest_global(rig_path, parents)
    if os.path.abspath(dst) == os.path.abspath(src):
        raise SystemExit("dst must differ from src (originals are never overwritten)")
    os.makedirs(dst, exist_ok=True)
    nmod = ncopy = 0
    report, gate_fail = {}, {}
    for f in sorted(glob.glob(os.path.join(src, "*" + CLIP_SUFFIX))):
        name = os.path.basename(f)[:-len(CLIP_SUFFIX)]
        out = os.path.join(dst, os.path.basename(f))
        if not matches(name, cls):
            shutil.copyfile(f, out)
            ncopy += 1
            continue
        c, B = cp.read_clip(f)
        P, Q = B[..., 0:3], B[..., 3:7]
        P2, Q2, info = correct(P, Q, names, parents, cls, rest_gp, rest_gq)
        if check_gate:
            fl = cp.knee_gate_clip(P, Q, P2, Q2, names, parents, rest_gp, rest_gq)
            info["knee_gate"] = fl
            if fl["l"] or fl["r"]:
                gate_fail[name] = fl
        report[name] = info
        if info["frames_changed"] == 0:
            shutil.copyfile(f, out)
            ncopy += 1
            continue
        changed = np.any(P2 != P, axis=(1, 2)) | np.any(Q2 != Q, axis=(1, 2))
        for fr, ch, pf, qf in zip(c["frames"], changed, P2, Q2):
            if not ch:
                continue
            fr["bones"] = [[float(x) for x in pp] + [float(x) for x in qq] + list(bb[7:10])
                           for pp, qq, bb in zip(pf, qf, fr["bones"])]
        c["posture_class"] = {"class": "launch", "file": os.path.basename(OFFSETS), "knee_deg": cls["knee_deg"],
                              "cap_deg": cls["cap_deg"], "lean_in_max": info["lean_in_max"],
                              "lean_out_max": info["lean_out_max"], "frames_changed": info["frames_changed"]}
        with open(out, "w", encoding="utf-8", newline="\n") as fo:
            json.dump(c, fo, separators=(",", ":"))
        nmod += 1
    with open(os.path.join(dst, "launch_posture_report.json"), "w", encoding="utf-8", newline="\n") as fo:
        json.dump({"class": cls, "rig": os.path.relpath(rig_path, ROOT), "src": os.path.relpath(src, ROOT),
                   "clips": report, "knee_gate_failed": gate_fail}, fo, indent=1)
    return nmod, ncopy, report, gate_fail


def cmd_apply(a):
    cls = load_class(a.offsets)
    nmod, ncopy, report, gate_fail = apply_dir(a.src, a.dst, cls, RIGS[a.rig])
    for k in sorted(report):
        r = report[k]
        print("%-44s lean %5.1f -> %5.1f  frames %3d/%3d  lift %4.1f cm  applied %5.1f deg  err %.3f" % (
            k, r["lean_in_max"], r["lean_out_max"], r["frames_changed"], r["frames"], 100 * r["lift_max_m"],
            r["applied_deg_max"], r["target_err_deg"]))
    if gate_fail:
        print(f"KNEE_GATE_FAIL {len(gate_fail)} clip(s) {sorted(gate_fail)[:10]} (launch_posture_report.json): do not bake")
        print(f"APPLY_DONE modified {nmod}, copied unchanged {ncopy} -> {a.dst}")
        return 2
    print(f"APPLY_OK modified {nmod}, copied unchanged {ncopy} -> {a.dst}")
    return 0


# ---------------------------------------------------------------- measure

def clip_metrics(P, Q, names, parents):
    bi = {n: i for i, n in enumerate(names)}
    gp, gq = mb.fk_model(P, Q, parents)
    g = lambda n: gp[:, bi[n]]  # noqa: E731
    lean, _ = lean_deg(gp, bi)
    kn = []
    for s in ("Left", "Right"):
        t = g(s + "Leg") - g(s + "UpLeg")
        sh = g(s + "Foot") - g(s + "Leg")
        c = np.sum(t * sh, axis=1) / (np.linalg.norm(t, axis=1) * np.linalg.norm(sh, axis=1))
        kn.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    hd = g("Head") - g("Neck")
    return {"lean": lean, "hips_z": g("Hips")[:, 2], "knee": np.maximum(kn[0], kn[1]),
            "feet": np.stack([g("LeftFoot"), g("RightFoot")], 1), "toes": np.stack([g("LeftToeBase"), g("RightToeBase")], 1),
            "head_pitch": np.degrees(np.arctan2(np.linalg.norm(hd[:, :2], axis=1), hd[:, 2])),
            "hands": np.stack([g("LeftHand"), g("RightHand")], 1)}


def cmd_measure(a):
    cls = load_class(a.offsets)
    names, parents = mb.load_rig(RIGS[a.rig], NB)
    L = ["# lean = Hips->Neck vs vertical (deg); hipsZ = Hips height (m); knee = max knee flexion of both legs (deg, 0 = "
         "straight); ankErr / toeErr = max ankle / toe position change (mm); headUp = max head (Neck->Head) lift vs before (deg); "
         "changed = frames whose pose differs; other = clips outside the class with any byte change"]
    if a.db:
        sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
        import crouch_stats as cs
        for d in a.db:
            db = cs.load_db(d)
            try:
                shown = os.path.relpath(d, ROOT)
            except ValueError:  # another drive (installed game DB)
                shown = os.path.abspath(d)
            L.append("## DB " + shown)
            for c in db["clips"]:
                if matches(c["name"], cls):
                    f0, n = c["first"], c["n"]
                    m = clip_metrics(np.asarray(db["pose"][f0:f0 + n, :, :3], dtype=np.float64),
                                     np.asarray(db["pose"][f0:f0 + n, :, 3:], dtype=np.float64), names, parents)
                    L.append("%-44s lean max %5.1f  hipsZ min %.3f  knee max %5.1f" % (
                        c["name"], m["lean"].max(), m["hips_z"].min(), m["knee"].max()))
    else:
        L.append("%-44s %11s %7s %13s %11s %6s %6s %6s %7s" % ("clip", "lean in>out", "changed", "hipsZmin in>out",
                                                              "knee in>out", "ankErr", "toeErr", "headUp", "handMv"))
        other = 0
        for f in sorted(glob.glob(os.path.join(a.src, "*" + CLIP_SUFFIX))):
            name = os.path.basename(f)[:-len(CLIP_SUFFIX)]
            g = os.path.join(a.dst, os.path.basename(f))
            if not os.path.exists(g):
                L.append("%-44s MISSING in dst" % name)
                continue
            if not matches(name, cls):
                if open(f, "rb").read() != open(g, "rb").read():
                    other += 1
                    L.append("%-44s CHANGED although outside the class" % name)
                continue
            _, B0 = cp.read_clip(f)
            _, B1 = cp.read_clip(g)
            m0 = clip_metrics(B0[..., 0:3], B0[..., 3:7], names, parents)
            m1 = clip_metrics(B1[..., 0:3], B1[..., 3:7], names, parents)
            ch = int(np.sum(np.any(B0[..., :7] != B1[..., :7], axis=(1, 2))))
            L.append("%-44s %5.1f>%5.1f %3d/%3d %6.3f>%6.3f %5.1f>%5.1f %6.2f %6.2f %6.1f %6.1fcm" % (
                name, m0["lean"].max(), m1["lean"].max(), ch, len(B0), m0["hips_z"].min(), m1["hips_z"].min(),
                m0["knee"].max(), m1["knee"].max(), 1000 * np.abs(m1["feet"] - m0["feet"]).max(),
                1000 * np.abs(m1["toes"] - m0["toes"]).max(), (m0["head_pitch"] - m1["head_pitch"]).max(),
                100 * np.linalg.norm(m1["hands"] - m0["hands"], axis=-1).max()))
        L.append("clips outside the class changed: %d" % other)
    print("\n".join(L))
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(L) + "\n")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("apply")
    p.add_argument("--src", required=True)
    p.add_argument("--dst", required=True)
    p.add_argument("--rig", choices=sorted(RIGS), default="woman")
    p.add_argument("--offsets", default=OFFSETS)
    m = sp.add_parser("measure")
    m.add_argument("--src")
    m.add_argument("--dst")
    m.add_argument("--rig", choices=sorted(RIGS), default="woman")
    m.add_argument("--offsets", default=OFFSETS)
    m.add_argument("--db", nargs="*", help="lean table of baked .utppmm files instead of clip dirs")
    m.add_argument("--out", default=None)
    a = ap.parse_args()
    return {"apply": cmd_apply, "measure": cmd_measure}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
