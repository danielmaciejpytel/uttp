#!/usr/bin/env python3
"""Moving-crouch posture class for the source pack crouch clips (posture_offsets.json -> classes.crouch), offline.

  apply    -> staging dir of *.utppclip.json: every clip of --src is copied; clips matching the class regex get the
              correction (hips lowered with leg IK keeping the ankles in place, torso pitched upright, head/arms level)
  measure  -> per-family table (hips z, trunk lean, knee flexion, ankle error, head tilt, hand clearance) of --src vs --dst
              clip dirs (before/after), or of one or more baked .utppmm (--db)

The correction is a pure function of each frame's ORIGINAL hips height (weight w, 0 at idle depth), so a loop's first/last
frames stay identical and idle-depth frames are bit-exact. Frame data: game-local x,y,z,w quaternions, Z up, +Y forward,
+X right (mmbake FORMAT). Originals are never overwritten (dst must differ from src).
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
if os.path.basename(ROOT).lower() == "build":   # worktree Build/wt-<name>/tools/retarget
    ROOT = os.path.dirname(ROOT)
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
import mmbake_base as mb  # noqa: E402  (qmul, qrot, fk_model, load_rig)

RIG = os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json")
OFFSETS = os.path.join(HERE, "posture_offsets.json")
SRC = os.path.join(ROOT, "Animations", "retarget", "out_posture")
DST = os.path.join(ROOT, "Animations", "retarget", "out_posture_crouch_v4")
NB = 73
CLIP_SUFFIX = ".utppclip.json"


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qnorm(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def qmul_(a, b):
    return mb.qmul(a, b)


def rx(angle):
    """Quaternion (x,y,z,w) of a rotation about the model X axis; angle (N,) rad. +angle tilts +Z towards -Y (backwards)."""
    q = np.zeros(angle.shape + (4,))
    q[..., 0] = np.sin(angle / 2)
    q[..., 3] = np.cos(angle / 2)
    return q


def arc(a, b):
    """Shortest-arc quaternion taking direction a to direction b (both (N,3))."""
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    d = np.sum(a * b, axis=-1)
    return qnorm(np.concatenate([np.cross(a, b), (1.0 + d)[..., None]], axis=-1))


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def load_class(path=OFFSETS):
    return json.load(open(path, encoding="utf-8"))["classes"]["crouch"]


def matches(name, cls):
    return re.search(cls["include_regex"], name) is not None and re.search(cls["exclude_regex"], name) is None


def correct(P, Q, names, parents, cls):
    """P (N,73,3), Q (N,73,4) local -> corrected copies (frames with w == 0 untouched) and the weight w (N,)."""
    bi = {n: i for i, n in enumerate(names)}
    gp, gq = mb.fk_model(P, Q, parents)
    hz = gp[:, bi["Hips"], 2]
    w = smoothstep((hz - cls["z_lo_m"]) / (cls["z_hi_m"] - cls["z_lo_m"]))
    N = len(hz)
    P2, Q2 = P.copy(), Q.copy()
    G2 = gq.copy()                                     # new global orientations, filled in bone order (parents first)
    delta = np.radians(cls["spine_bend_deg"]) * w      # (N,) pitch applied at the top of the spine chain
    alpha = {bi[n]: f for n, f in cls["spine_chain"].items()}
    alpha.update({bi[n]: f for n, f in cls["neck_chain"].items()})
    alpha.update({bi[n]: f for n, f in cls["arm_chain"].items()})
    # never lower below the crouch-idle depth (hips_floor_m): keeps the knee flexion within what source pack idle/turn clips already use
    drop = np.minimum(w * cls["hips_drop_m"], np.maximum(hz - cls["hips_floor_m"], 0.0))
    P2[:, bi["Hips"], 2] = P[:, bi["Hips"], 2] - drop
    shift = np.zeros((N, 3))
    shift[:, 2] = -drop
    legs = {}
    for side in ("Left", "Right"):
        legs[bi[side + "UpLeg"]] = (side, "up")
        legs[bi[side + "Leg"]] = (side, "leg")
        legs[bi[side + "Foot"]] = (side, "foot")
    knee_new = {}
    for b, p in enumerate(parents):
        if p < 0:
            continue
        if b in alpha:
            tgt = qmul_(rx(alpha[b] * delta), gq[:, b])
            Q2[:, b] = qmul_(qconj(G2[:, p]), tgt)
            G2[:, b] = tgt
        elif b in legs:
            side, kind = legs[b]
            up, kn, ft = bi[side + "UpLeg"], bi[side + "Leg"], bi[side + "Foot"]
            if kind == "up":
                hip = gp[:, up] + shift
                a_len = np.linalg.norm(P[:, kn], axis=-1)           # UpLeg -> Leg local translation length
                b_len = np.linalg.norm(P[:, ft], axis=-1)
                tgt = gp[:, ft]                                     # ankle stays where it was
                v = tgt - hip
                d = np.clip(np.linalg.norm(v, axis=-1), np.abs(a_len - b_len) + 1e-4, a_len + b_len - 1e-4)
                u = v / np.linalg.norm(v, axis=-1, keepdims=True)
                x = (a_len ** 2 - b_len ** 2 + d ** 2) / (2 * d)
                h = np.sqrt(np.maximum(a_len ** 2 - x ** 2, 0.0))
                dir0 = gp[:, kn] - gp[:, up]                        # original knee offset from the hip
                pole = dir0 - np.sum(dir0 * u, axis=-1, keepdims=True) * u
                pole /= np.linalg.norm(pole, axis=-1, keepdims=True)
                knee = hip + x[:, None] * u + h[:, None] * pole
                g_up = qmul_(arc(dir0, knee - hip), gq[:, up])
                Q2[:, b] = qmul_(qconj(G2[:, p]), g_up)
                G2[:, b] = g_up
                knee_new[side] = (knee, tgt)
            elif kind == "leg":
                knee, tgt = knee_new[side]
                g_cur = qmul_(G2[:, p], Q[:, b])
                cur = mb.qrot(g_cur, P[:, ft])
                g_leg = qmul_(arc(cur, tgt - knee), g_cur)
                Q2[:, b] = qmul_(qconj(G2[:, p]), g_leg)
                G2[:, b] = g_leg
            else:  # foot: original model-space orientation
                Q2[:, b] = qmul_(qconj(G2[:, p]), gq[:, b])
                G2[:, b] = gq[:, b]
        else:
            G2[:, b] = qmul_(G2[:, p], Q2[:, b])
    Q2 = qnorm(Q2)
    keep = w <= 0.0
    P2[keep] = P[keep]
    Q2[keep] = Q[keep]
    return P2, Q2, w


# ---------------------------------------------------------------- knee-fixed variant (apply --knee-fix)

KNEE_LOW_FLEX_DEG = 15.0   # [P] runtime session: reduce the hip drop where the input knee flex is below ~15 deg
KNEE_FLIP_DEG = 30.0       # [P] runtime session: fail when knee direction vs toes changes > 30 deg/frame ...
KNEE_FLEX_GATE_DEG = 20.0  # [P] ... at flex > 20 deg (measured relative to the input clip)


def mat_to_q(m):
    """(N,3,3) rotation matrices -> (N,4) x,y,z,w (Shepperd)."""
    q = np.zeros(m.shape[:-2] + (4,))
    for i in range(m.shape[0]):
        r = m[i]
        t = np.trace(r)
        if t > 0:
            k = np.sqrt(t + 1.0) * 2
            q[i] = [(r[2, 1] - r[1, 2]) / k, (r[0, 2] - r[2, 0]) / k, (r[1, 0] - r[0, 1]) / k, 0.25 * k]
        else:
            j = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
            if j == 0:
                k = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
                q[i] = [0.25 * k, (r[0, 1] + r[1, 0]) / k, (r[0, 2] + r[2, 0]) / k, (r[2, 1] - r[1, 2]) / k]
            elif j == 1:
                k = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
                q[i] = [(r[0, 1] + r[1, 0]) / k, 0.25 * k, (r[1, 2] + r[2, 1]) / k, (r[0, 2] - r[2, 0]) / k]
            else:
                k = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
                q[i] = [(r[0, 2] + r[2, 0]) / k, (r[1, 2] + r[2, 1]) / k, 0.25 * k, (r[1, 0] - r[0, 1]) / k]
    return qnorm(q)


def basis_q(d, s):
    """Rotation (x,y,z,w) of the orthonormal frame with columns [d, s_perp, d x s_perp] (N,3 each)."""
    d = d / np.linalg.norm(d, axis=-1, keepdims=True)
    so = s - np.sum(s * d, -1, keepdims=True) * d
    so = so / np.linalg.norm(so, axis=-1, keepdims=True)
    return mat_to_q(np.stack([d, so, np.cross(d, so)], -1))


def rest_global(names, parents):
    rc = json.load(open(RIG, encoding="utf-8"))["Data"]["RootChunk"]
    P0 = np.zeros((1, NB, 3))
    Q0 = np.zeros((1, NB, 4))
    for b, t in enumerate(rc["boneTransforms"][:NB]):
        P0[0, b] = [t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"]]
        Q0[0, b] = [t["Rotation"]["i"], t["Rotation"]["j"], t["Rotation"]["k"], t["Rotation"]["r"]]
    gp, gq = mb.fk_model(P0, Q0, parents)
    return gp[0], gq[0]


KNEE_LOW_BLUR_RADIUS = 6   # frames [P]: min-filter + triangle blur of the low-flex drop factor (no hip pops)


def blur_seq(x, P, Q, radius=KNEE_LOW_BLUR_RADIUS):
    """Min-filter then triangle-blur x (N,); result <= x (the drop is only ever reduced further). Circular over the
    period N-1 when the last frame duplicates frame 0 (loop export), so the loop stays closed; clamped otherwise."""
    n = len(x)
    loop = n > 2 and np.allclose(P[0, 2:], P[-1, 2:], atol=2e-3) and np.allclose(np.abs(np.sum(Q[0, 2:] * Q[-1, 2:], -1)), 1.0, atol=1e-5)  # bones 0-1 = Root, Trajectory (moves)
    m = n - 1 if loop else n
    ks = range(-radius, radius + 1)
    at = (lambda s, i: s[i % m]) if loop else (lambda s, i: s[min(max(i, 0), n - 1)])
    ero = np.array([min(at(x, i + k) for k in ks) for i in range(m)])
    wts = np.array([radius + 1 - abs(k) for k in ks], dtype=np.float64)
    bl = np.array([sum(w * at(ero, i + k) for w, k in zip(wts, ks)) for i in range(m)]) / wts.sum()
    out = np.minimum(bl, x[:m])
    if loop:
        out = np.append(out, out[0])
    return out


def correct_kneefix(P, Q, names, parents, cls, rest_gp, rest_gq):
    """correct() (crouch_v4) with the runtime-session knee fix: the knee pole is the ORIGINAL knee hinge (bind lateral
    axis LeftUpLeg-RightUpLeg in the bind thigh frame, carried by the original thigh) carried by FromTo(old hip->ankle
    axis, new axis); thigh and shin are re-aimed with that hinge as secondary axis (the shin turns about the knee hinge
    only, no shortest-arc twist); the hip drop is scaled by smoothstep(input flex of the straighter leg /
    KNEE_LOW_FLEX_DEG). Pure per-frame function of the input frame, so loops stay closed."""
    bi = {n: i for i, n in enumerate(names)}
    gp, gq = mb.fk_model(P, Q, parents)
    hz = gp[:, bi["Hips"], 2]
    w = smoothstep((hz - cls["z_lo_m"]) / (cls["z_hi_m"] - cls["z_lo_m"]))
    N = len(hz)
    P2, Q2 = P.copy(), Q.copy()
    G2 = gq.copy()
    delta = np.radians(cls["spine_bend_deg"]) * w
    alpha = {bi[n]: f for n, f in cls["spine_chain"].items()}
    alpha.update({bi[n]: f for n, f in cls["neck_chain"].items()})
    alpha.update({bi[n]: f for n, f in cls["arm_chain"].items()})
    flex0 = []
    for side in ("Left", "Right"):
        t = gp[:, bi[side + "Leg"]] - gp[:, bi[side + "UpLeg"]]
        sh = gp[:, bi[side + "Foot"]] - gp[:, bi[side + "Leg"]]
        c = np.sum(t * sh, -1) / (np.linalg.norm(t, axis=-1) * np.linalg.norm(sh, axis=-1))
        flex0.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    low = blur_seq(smoothstep(np.minimum(flex0[0], flex0[1]) / KNEE_LOW_FLEX_DEG), P, Q)
    drop = np.minimum(w * cls["hips_drop_m"] * low, np.maximum(hz - cls["hips_floor_m"], 0.0))
    P2[:, bi["Hips"], 2] = P[:, bi["Hips"], 2] - drop
    shift = np.zeros((N, 3))
    shift[:, 2] = -drop
    lat_rest = rest_gp[bi["LeftUpLeg"]] - rest_gp[bi["RightUpLeg"]]
    lat_rest = lat_rest / np.linalg.norm(lat_rest)
    legs = {}
    for side in ("Left", "Right"):
        legs[bi[side + "UpLeg"]] = (side, "up")
        legs[bi[side + "Leg"]] = (side, "leg")
        legs[bi[side + "Foot"]] = (side, "foot")
    knee_new = {}
    for b, p in enumerate(parents):
        if p < 0:
            continue
        if b in alpha:
            tgt = qmul_(rx(alpha[b] * delta), gq[:, b])
            Q2[:, b] = qmul_(qconj(G2[:, p]), tgt)
            G2[:, b] = tgt
        elif b in legs:
            side, kind = legs[b]
            up, kn, ft = bi[side + "UpLeg"], bi[side + "Leg"], bi[side + "Foot"]
            if kind == "up":
                hip = gp[:, up] + shift
                a_len = np.linalg.norm(P[:, kn], axis=-1)
                b_len = np.linalg.norm(P[:, ft], axis=-1)
                tgt = gp[:, ft]
                v = tgt - hip
                d = np.clip(np.linalg.norm(v, axis=-1), np.abs(a_len - b_len) + 1e-4, a_len + b_len - 1e-4)
                u = v / np.linalg.norm(v, axis=-1, keepdims=True)
                x = (a_len ** 2 - b_len ** 2 + d ** 2) / (2 * d)
                h = np.sqrt(np.maximum(a_len ** 2 - x ** 2, 0.0))
                a_loc = mb.qrot(qconj(rest_gq[up][None]), lat_rest[None])[0]
                n0 = mb.qrot(gq[:, up], np.broadcast_to(a_loc, (N, 3)).copy())
                u0 = gp[:, ft] - gp[:, up]
                n = mb.qrot(arc(u0, u), n0)                        # FromTo(old axis, new axis) carry
                n = n - np.sum(n * u, -1, keepdims=True) * u
                n = n / np.linalg.norm(n, axis=-1, keepdims=True)
                pole = np.cross(u, n)                              # plane normal (knee-hip) x (ankle-knee) = n
                knee = hip + x[:, None] * u + h[:, None] * pole
                dir0 = gp[:, kn] - gp[:, up]
                R = qmul_(basis_q(knee - hip, n), qconj(basis_q(dir0, n0)))
                g_up = qnorm(qmul_(R, gq[:, up]))
                Q2[:, b] = qmul_(qconj(G2[:, p]), g_up)
                G2[:, b] = g_up
                knee_new[side] = (knee, tgt, n, n0)
            elif kind == "leg":
                knee, tgt, n, n0 = knee_new[side]
                sh0 = gp[:, ft] - gp[:, kn]
                R = qmul_(basis_q(tgt - knee, n), qconj(basis_q(sh0, n0)))
                g_leg = qnorm(qmul_(R, gq[:, b]))
                Q2[:, b] = qmul_(qconj(G2[:, p]), g_leg)
                G2[:, b] = g_leg
            else:
                Q2[:, b] = qmul_(qconj(G2[:, p]), gq[:, b])
                G2[:, b] = gq[:, b]
        else:
            G2[:, b] = qmul_(G2[:, p], Q2[:, b])
    Q2 = qnorm(Q2)
    keep = w <= 0.0
    P2[keep] = P[keep]
    Q2[keep] = Q[keep]
    return P2, Q2, w


def knee_gate_clip(P0, Q0, P1, Q1, names, parents, rest_gp, rest_gq):
    """Fail rule: frames where, with input and output flex > KNEE_FLEX_GATE_DEG on both frames,
    |dtheta_out| - |dtheta_in| > KNEE_FLIP_DEG (knee direction vs toes, knee_metrics.leg_metrics). {side: [frames]}"""
    sys.path.insert(0, HERE)
    import knee_metrics as km
    idx = [names.index(b) for b in km.CP_BONES]
    ms = []
    for P, Q in ((P0, Q0), (P1, Q1)):
        gp, gq = mb.fk_model(P, Q, parents)
        ms.append(km.leg_metrics(gp[:, idx], gq[:, idx], rest_gp[idx], rest_gq[idx]))
    res = {}
    for side in ("l", "r"):
        g = (ms[0][side]["flex"] > KNEE_FLEX_GATE_DEG) & (ms[1][side]["flex"] > KNEE_FLEX_GATE_DEG)
        gg = np.zeros_like(g)
        gg[1:] = g[1:] & g[:-1]
        ex = np.where(gg, np.abs(ms[1][side]["dtheta"]) - np.abs(np.nan_to_num(ms[0][side]["dtheta"])), np.nan)
        res[side] = [int(i) for i in np.nonzero(ex > KNEE_FLIP_DEG)[0]]
    return res


# ---------------------------------------------------------------- apply

def read_clip(path):
    c = json.load(open(path, encoding="utf-8"))
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    return c, B


def cmd_apply(a):
    names, parents = mb.load_rig(RIG, NB)
    cls = load_class(a.offsets)
    if os.path.abspath(a.dst) == os.path.abspath(a.src):
        raise SystemExit("dst must differ from src (originals are never overwritten)")
    os.makedirs(a.dst, exist_ok=True)
    nmod = ncopy = 0
    gate_fail = {}
    if a.knee_fix:
        rest_gp, rest_gq = rest_global(names, parents)
    for f in sorted(glob.glob(os.path.join(a.src, "*" + CLIP_SUFFIX))):
        name = os.path.basename(f)[:-len(CLIP_SUFFIX)]
        out = os.path.join(a.dst, os.path.basename(f))
        if not matches(name, cls):
            shutil.copyfile(f, out)
            ncopy += 1
            continue
        c, B = read_clip(f)
        if a.knee_fix:
            P2, Q2, w = correct_kneefix(B[..., 0:3], B[..., 3:7], names, parents, cls, rest_gp, rest_gq)
            fl = knee_gate_clip(B[..., 0:3], B[..., 3:7], P2, Q2, names, parents, rest_gp, rest_gq)
            if fl["l"] or fl["r"]:
                gate_fail[name] = fl
        else:
            P2, Q2, w = correct(B[..., 0:3], B[..., 3:7], names, parents, cls)
        for fr, pf, qf in zip(c["frames"], P2, Q2):
            fr["bones"] = [[float(x) for x in pp] + [float(x) for x in qq] + list(bb[7:10])
                           for pp, qq, bb in zip(pf, qf, fr["bones"])]
        c["posture_class"] = {"class": "crouch", "file": os.path.basename(a.offsets),
                              "w_max": float(w.max()), "w_mean": float(w.mean())}
        if a.knee_fix:
            c["posture_class"]["knee_fix"] = {"version": 1, "low_flex_deg": KNEE_LOW_FLEX_DEG,
                                              "gate": "flip > %g deg/frame over the input at flex > %g [P]" % (KNEE_FLIP_DEG, KNEE_FLEX_GATE_DEG)}
        with open(out, "w", encoding="utf-8", newline="\n") as fo:
            json.dump(c, fo, separators=(",", ":"))
        nmod += 1
    if a.knee_fix:
        with open(os.path.join(a.dst, "knee_gate.json"), "w", encoding="utf-8") as fo:
            json.dump({"flip_deg": KNEE_FLIP_DEG, "flex_gate_deg": KNEE_FLEX_GATE_DEG, "failed": gate_fail}, fo, indent=1)
        if gate_fail:
            print(f"KNEE_GATE_FAIL {len(gate_fail)} clip(s) {sorted(gate_fail)[:10]} (knee_gate.json): do not bake")
            print(f"APPLY_DONE modified {nmod}, copied unchanged {ncopy} -> {a.dst}")
            return 2
    print(f"APPLY_OK modified {nmod}, copied unchanged {ncopy} -> {a.dst}")
    return 0


# ---------------------------------------------------------------- measure

FAMILIES = [
    ("Stand_Idle_Loop (ref)", r"^M_Neutral_Stand_Idle_Loop$"),
    ("Crouch_Idle_Loop", r"^M_Neutral_Crouch_Idle_Loop$"),
    ("Crouch_Idle_Turn_*", r"^M_Neutral_Crouch_Idle_Turn_"),
    ("Crouch_Loop_*", r"^M_Neutral_Crouch_Loop_"),
    ("Crouch_Start_*", r"^M_Neutral_Crouch_Start_"),
    ("Crouch_Stop_*", r"^M_Neutral_Crouch_Stop_"),
    ("Crouch_Pivot_*", r"^M_Neutral_Crouch_Pivot_"),
    ("Crouch_Turn_*", r"^M_Neutral_Crouch_Turn_"),
    ("Crouch_Reface_Start_*", r"^M_Neutral_Crouch_Reface_Start_"),
    ("Crouch_Box/Diamond/Hourglass", r"^M_Neutral_Crouch_(Box|Diamond|Hourglass)_"),
    ("Transition_Stand<->Crouch", r"^M_Neutral_Transition_(Stand_to_Crouch|Crouch_to_Stand)$"),
]


def clip_metrics(P, Q, names, parents):
    bi = {n: i for i, n in enumerate(names)}
    gp, gq = mb.fk_model(P, Q, parents)
    g = lambda n: gp[:, bi[n]]  # noqa: E731
    v = g("Neck") - g("Hips")
    out = {"hips_z": g("Hips")[:, 2], "lean": np.degrees(np.arctan2(np.linalg.norm(v[:, :2], axis=1), v[:, 2]))}
    kn = []
    for s in ("Left", "Right"):
        t = g(s + "Leg") - g(s + "UpLeg")
        sh = g(s + "Foot") - g(s + "Leg")
        c = np.sum(t * sh, axis=1) / (np.linalg.norm(t, axis=1) * np.linalg.norm(sh, axis=1))
        kn.append(np.degrees(np.arccos(np.clip(c, -1, 1))))               # knee flexion: 0 = straight leg
    out["knee_flex"] = np.maximum(kn[0], kn[1])
    hz = g("Head") - g("Neck")
    out["head_tilt"] = np.degrees(np.arctan2(np.linalg.norm(hz[:, :2], axis=1), hz[:, 2]))
    hc = []
    for s in ("Left", "Right"):
        h, a, b = g(s + "Hand"), g(s + "UpLeg"), g(s + "Leg")
        ab = b - a
        t = np.clip(np.sum((h - a) * ab, axis=1) / np.sum(ab * ab, axis=1), 0, 1)
        hc.append(np.linalg.norm(h - (a + t[:, None] * ab), axis=1))
    out["hand_thigh"] = np.minimum(hc[0], hc[1])
    out["foot_min_z"] = np.minimum(g("LeftFoot")[:, 2], g("RightFoot")[:, 2])
    out["feet"] = np.stack([g("LeftFoot"), g("RightFoot")], 1)
    out["head_q"] = gq[:, bi["Head"]]
    out["arm_q"] = gq[:, bi["LeftArm"]]
    return out


def in_families(name):
    return any(re.search(r, name) for _, r in FAMILIES)


def collect_dir(d, names, parents):
    res = {}
    for f in sorted(glob.glob(os.path.join(d, "M_Neutral_*" + CLIP_SUFFIX))):
        name = os.path.basename(f)[:-len(CLIP_SUFFIX)]
        if in_families(name):
            _, B = read_clip(f)
            res[name] = clip_metrics(B[..., 0:3], B[..., 3:7], names, parents)
    return res


def collect_db(path, names, parents):
    sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
    import crouch_stats as cs
    db = cs.load_db(path)
    res = {}
    for c in db["clips"]:
        if in_families(c["name"]):
            f0, n = c["first"], c["n"]
            P = np.asarray(db["pose"][f0:f0 + n, :, :3], dtype=np.float64)
            Q = np.asarray(db["pose"][f0:f0 + n, :, 3:], dtype=np.float64)
            res[c["name"]] = clip_metrics(P, Q, names, parents)
    return res


def qang_deg(a, b):
    return np.degrees(2 * np.arccos(np.abs(np.sum(a * b, axis=-1)).clip(0, 1)))


def family_rows(res, ref=None):
    rows = []
    for fam, rg in FAMILIES:
        ks = [k for k in res if re.search(rg, k)]
        if not ks:
            continue
        cat = lambda key: np.concatenate([res[k][key] for k in ks])  # noqa: E731
        mean = lambda key: float(np.mean([res[k][key].mean() for k in ks]))  # noqa: E731
        hz = cat("hips_z")
        row = {"fam": fam, "n": len(ks), "hz_mean": mean("hips_z"), "hz_min": float(hz.min()), "hz_max": float(hz.max()),
               "lean_mean": mean("lean"), "lean_max": float(cat("lean").max()), "knee_mean": mean("knee_flex"),
               "knee_max": float(cat("knee_flex").max()), "hand_min": float(cat("hand_thigh").min()),
               "foot_zmin": float(cat("foot_min_z").min()), "head_tilt": mean("head_tilt")}
        if ref is not None:
            row["foot_err"] = float(max(np.abs(res[k]["feet"] - ref[k]["feet"]).max() for k in ks))
            row["head_err"] = float(max(qang_deg(res[k]["head_q"], ref[k]["head_q"]).max() for k in ks))
            row["arm_err"] = float(max(qang_deg(res[k]["arm_q"], ref[k]["arm_q"]).max() for k in ks))
        rows.append(row)
    return rows


def fmt_rows(title, rows, stand):
    L = [title, "%-29s %3s %6s %5s %13s %5s %6s %6s %6s %5s %8s %7s %7s %7s %7s" % (
        "family", "n", "hipZ", "%stnd", "hipZ min..max", "lean", "leanMx", "kneeFl", "kneeMx", "headT", "handThg", "footZ",
        "ankErr", "headErr", "armErr")]
    for r in rows:
        L.append("%-29s %3d %6.3f %4.0f%% %6.3f..%5.3f %5.1f %6.1f %6.1f %6.1f %5.1f %6.1fcm %5.1fcm %7s %7s %7s" % (
            r["fam"], r["n"], r["hz_mean"], 100 * r["hz_mean"] / stand, r["hz_min"], r["hz_max"], r["lean_mean"],
            r["lean_max"], r["knee_mean"], r["knee_max"], r["head_tilt"], 100 * r["hand_min"], 100 * r["foot_zmin"],
            ("%.4fm" % r["foot_err"]) if "foot_err" in r else "-", ("%.2fd" % r["head_err"]) if "head_err" in r else "-",
            ("%.2fd" % r["arm_err"]) if "arm_err" in r else "-"))
    return L


def cmd_measure(a):
    names, parents = mb.load_rig(RIG, NB)
    stand = load_class(a.offsets)["stand_hips_z_m"]
    L = ["# hipZ = mean Hips height (model space, m); %%stnd vs stand_hips_z_m=%.3f (the 0.974 reference); "
         "lean = Hips->Neck vs vertical (deg);" % stand,
         "# kneeFl = knee flexion (0 = straight, max of both legs); headT = Neck->Head vs vertical (deg); handThg = min hand to "
         "thigh-segment distance;",
         "# footZ = lowest ankle height; ankErr = max ankle position change vs before; headErr/armErr = max model-space "
         "orientation change of Head / LeftArm (deg)", ""]
    if a.db:
        for d in a.db:
            L += fmt_rows("## DB " + os.path.relpath(d, ROOT), family_rows(collect_db(d, names, parents)), stand) + [""]
    else:
        before = collect_dir(a.src, names, parents)
        after = collect_dir(a.dst, names, parents)
        L += fmt_rows("## BEFORE " + os.path.relpath(a.src, ROOT), family_rows(before), stand) + [""]
        L += fmt_rows("## AFTER " + os.path.relpath(a.dst, ROOT), family_rows(after, before), stand) + [""]
    print("\n".join(L))
    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    p = sp.add_parser("apply")
    p.add_argument("--src", default=SRC)
    p.add_argument("--dst", default=DST)
    p.add_argument("--offsets", default=OFFSETS)
    p.add_argument("--knee-fix", action="store_true",
                   help="knee-fixed variant (correct_kneefix + fail gate, exit 2); use a NEW --dst")
    m = sp.add_parser("measure")
    m.add_argument("--src", default=SRC)
    m.add_argument("--dst", default=DST)
    m.add_argument("--offsets", default=OFFSETS)
    m.add_argument("--db", nargs="*", help="measure baked .utppmm files instead of clip dirs")
    m.add_argument("--out", default=None)
    a = ap.parse_args()
    return {"apply": cmd_apply, "measure": cmd_measure}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
