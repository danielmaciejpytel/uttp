#!/usr/bin/env python3
"""Posture ("retarget pose") offsets: pull the male source pack shoulder/neck posture of the retargeted clips towards the
game female (reference) posture, as a constant per-bone local rotation post-process (no Blender needed).

  compute  -> posture_offsets.json : C_B = inverse(local_DB_idle[B]) * local_REF_idle[B]  (game-local quaternions)
  apply    -> new folder of *.utppclip.json with local[B] = local[B] * C_B for every frame (originals untouched;
              the Trajectory/Root/Hips/legs/spine and all translations are not modified; children keep their locals)
  metrics  -> posture / hand-clearance metrics of a DB clip (optionally with an offsets JSON applied in memory),
              and of a reference glb animation, for before/after comparison

Coordinates: .utppclip.json locals are CP game-local (Z up). The reference glb (WolvenKit export of the game animations) stores the same locals
with a Y-up root conversion: glb (x,y,z) = game (x,z,-y); quaternion glb (i,j,k,r) = game (i,k,-j,r) -- verified
against player_woman_skeleton.rig.json boneTransforms (Hips/Spine3/LeftShoulder/Neck/LeftArm identical after the
conversion). Requires numpy.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(HERE, "..", "mmbake"))
sys.path.insert(0, os.path.join(HERE, "..", "recon"))
import mmbake_base as mb  # noqa: E402  (fk_model, load_rig, qmul)
import glb_anim_pose as gl  # noqa: E402

RIG = os.path.join(ROOT, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json")
REF_GLB = os.path.join(ROOT, "Extracted", "game", "glb", "reference", "tpp_fem_v_locomotion.anims.glb")
CLIPS = os.path.join(ROOT, "Animations", "retarget", "out")
DEFAULT_OFFSETS = os.path.join(HERE, "posture_offsets.json")
NB = 73


# ---------------------------------------------------------------- quaternion helpers (x,y,z,w)

def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qmean(qs):
    qs = np.asarray(qs, dtype=np.float64)
    ref = qs[0]
    s = np.where((qs @ ref)[:, None] < 0, -qs, qs).sum(axis=0)
    return s / np.linalg.norm(s)


def qangle_deg(q):
    q = q / np.linalg.norm(q)
    return math.degrees(2.0 * math.acos(min(1.0, abs(q[3]))))


def qaxis(q):
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    s = np.linalg.norm(q[:3])
    return q[:3] / s if s > 1e-9 else np.zeros(3)


# ---------------------------------------------------------------- inputs

def load_rig_names():
    return mb.load_rig(RIG, NB)


def read_clip(path):
    c = json.load(open(path, encoding="utf-8"))
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    return c, B  # (N, 73, 10)


def read_ref(anim, names):
    """Reference glb animation -> (P (N,73,3), Q (N,73,4)) in game-local convention for bones 0..72 (by name)."""
    js, binb = gl.load(REF_GLB)
    a = next(x for x in js["animations"] if x.get("name") == anim)
    nodes = js["nodes"]
    nidx = {n.get("name"): i for i, n in enumerate(nodes)}
    n = max(len(gl.accessor(js, binb, s["input"])) for s in a["samplers"])
    P = np.zeros((n, len(names), 3))
    Q = np.zeros((n, len(names), 4))
    chans = {}
    for ch in a["channels"]:
        p = ch["target"]["path"]
        if p in ("translation", "rotation"):
            chans[(ch["target"]["node"], p)] = np.array(gl.accessor(js, binb, a["samplers"][ch["sampler"]]["output"]))
    for b, nm in enumerate(names):
        i = nidx.get(nm)
        node = nodes[i] if i is not None else {}
        t = np.array(node.get("translation", [0, 0, 0]), dtype=np.float64)
        r = np.array(node.get("rotation", [0, 0, 0, 1]), dtype=np.float64)
        tt = chans.get((i, "translation"))
        rr = chans.get((i, "rotation"))
        for f in range(n):
            tv = tt[min(f, len(tt) - 1)] if tt is not None else t
            rv = rr[min(f, len(rr) - 1)] if rr is not None else r
            P[f, b] = (tv[0], -tv[2], tv[1])
            Q[f, b] = (rv[0], -rv[2], rv[1], rv[3])
    return P, Q


def apply_offsets_P(P, names, offsets):
    """P (N,73,3) local translations; scales posture bones' translation by t_scale (direction kept)."""
    out = P.copy()
    bi = {n: i for i, n in enumerate(names)}
    for bn, c in offsets["offsets"].items():
        if "t_scale" in c:
            out[:, bi[bn]] *= c["t_scale"]
    return out


# ---------------------------------------------------------------- compute

def apply_offsets_array(Q, names, offsets):
    """Q (N,73,4) local quats; returns corrected copy (local[B] = local[B] * C_B)."""
    out = Q.copy()
    bi = {n: i for i, n in enumerate(names)}
    for bn, c in offsets["offsets"].items():
        b = bi[bn]
        if c["angle_deg"] == 0.0:
            continue
        out[:, b] = mb.qmul(out[:, b], np.array(c["q"]))
        out[:, b] /= np.linalg.norm(out[:, b], axis=-1, keepdims=True)
    return out


def qmirror(q):
    """Mirror a spatial (model-space) rotation across the X=0 plane (character lateral axis)."""
    return np.array([q[0], -q[1], -q[2], q[3]])


def chain_global(Qloc, parents, b):
    """Model-space orientation of bone b from local quats Qloc (73,4)."""
    chain = []
    while b >= 0:
        chain.append(b)
        b = parents[b]
    g = np.array([0.0, 0.0, 0.0, 1.0])
    for k in reversed(chain):
        g = mb.qmul(g, Qloc[k])
    return g


def cmd_compute(a):
    names, parents = load_rig_names()
    bi = {n: i for i, n in enumerate(names)}
    c, B = read_clip(os.path.join(a.clips, a.db_clip + ".utppclip.json"))
    _, Qi = read_ref(a.ref_anim, names)
    nloop = B.shape[0] - 1  # last frame duplicates frame 0 in loops
    nref = Qi.shape[0] - 1
    if a.mode == "frame0":
        Qdb = B[0, :, 3:7].copy()
        Qit = Qi[0].copy()
    else:
        Qdb = np.array([qmean(B[:nloop, b, 3:7]) for b in range(NB)])
        Qit = np.array([qmean(Qi[:nref, b]) for b in range(NB)])
    todo = sorted({bi[n] for n in a.bones})
    work = Qdb.copy()  # DB idle locals with the corrections applied so far (parents first)
    res = {}
    ident = np.array([0.0, 0.0, 0.0, 1.0])

    def raw_E(b):
        """Model-space correction E (E * G_cur = G_target), target local = reference local under the current parent."""
        gp = chain_global(work, parents, parents[b]) if parents[b] >= 0 else ident
        d = mb.qmul(Qit[b], qconj(Qdb[b]))        # local delta in the parent frame: reference * inverse(DB)
        return mb.qmul(mb.qmul(gp, d), qconj(gp)), gp

    done = set()
    for b in todo:
        if b in done:
            continue
        nm = names[b]
        partner = bi.get("Right" + nm[4:]) if nm.startswith("Left") else None
        if partner is not None and partner not in todo:
            partner = None
        E, gp = raw_E(b)
        if partner is not None:
            Er, gpr = raw_E(partner)
            pairs = [(b, gp), (partner, gpr)]
            if a.symmetrize:
                Es = qmean([E, qmirror(Er)])
                Ess = {b: Es, partner: qmirror(Es)}
            else:
                Ess = {b: E, partner: Er}
        else:
            pairs = [(b, gp)]
            Ess = {b: qmean([E, qmirror(E)]) if a.symmetrize else E}
        for bb, g in pairs:
            lc = Qdb[bb]
            # local_new = inverse(G_p) E' G_p local_DB = local_DB * C  ->  C = inverse(local_DB) inverse(G_p) E' G_p local_DB
            cq = mb.qmul(mb.qmul(mb.qmul(qconj(lc), qconj(g)), mb.qmul(Ess[bb], g)), lc)
            cq /= np.linalg.norm(cq)
            if cq[3] < 0:
                cq = -cq
            craw = mb.qmul(qconj(lc), Qit[bb])
            craw /= np.linalg.norm(craw)
            if craw[3] < 0:
                craw = -craw
            res[names[bb]] = {"q": [float(x) for x in cq], "angle_deg": qangle_deg(cq),
                              "axis_local": [float(x) for x in qaxis(cq)],
                              "unsymmetrized_q": [float(x) for x in craw], "unsymmetrized_angle_deg": qangle_deg(craw)}
            work[bb] = mb.qmul(lc, cq)
            work[bb] /= np.linalg.norm(work[bb])
            done.add(bb)
    # translation (bone length) scale: the reference animation carries its own, shorter neck / clavicle local translations
    Pit = read_ref(a.ref_anim, names)[0]
    for bn in a.scale_bones:
        b = bi[bn]
        td = np.linalg.norm(B[:nloop, b, 0:3], axis=1).mean()
        ti = np.linalg.norm(Pit[:nref, b], axis=1).mean()
        ratio = float(ti / td)
        res.setdefault(bn, {"q": [0.0, 0.0, 0.0, 1.0], "angle_deg": 0.0, "axis_local": [0.0, 0.0, 0.0],
                            "unsymmetrized_q": [0.0, 0.0, 0.0, 1.0], "unsymmetrized_angle_deg": 0.0})
        res[bn]["t_scale_raw"] = ratio
        res[bn]["t_len_db_m"] = float(td)
        res[bn]["t_len_ref_m"] = float(ti)
    for bn in a.scale_bones:
        r = res[bn]["t_scale_raw"]
        if a.symmetrize and (bn.startswith("Left") or bn.startswith("Right")):
            o = ("Right" + bn[4:]) if bn.startswith("Left") else ("Left" + bn[5:])
            if o in res and "t_scale_raw" in res[o]:
                r = 0.5 * (r + res[o]["t_scale_raw"])
        res[bn]["t_scale"] = float(r)
    # mirror sanity: model-space rotation delta of L/R pairs after applying (angle between left and mirrored right)
    Pn = B[:1, :, 0:3].copy()
    Qn = Qdb[None].copy()
    _, gq0 = mb.fk_model(Pn, Qn, parents)
    _, gq1 = mb.fk_model(Pn, apply_offsets_array(Qn, names, {"offsets": res}), parents)
    mirror = {}
    for bn in res:
        if bn.startswith("Left") and ("Right" + bn[4:]) in res:
            rn = "Right" + bn[4:]
            dl = mb.qmul(gq1[0, bi[bn]], qconj(gq0[0, bi[bn]]))
            dr = mb.qmul(gq1[0, bi[rn]], qconj(gq0[0, bi[rn]]))
            mirror[bn] = {"model_delta_left_deg": qangle_deg(dl), "model_delta_right_deg": qangle_deg(dr),
                          "angle_between_left_and_mirrored_right_deg": qangle_deg(mb.qmul(qconj(dl), qmirror(dr)))}
    out = {
        "format": "utpp_posture_offsets", "version": 1,
        "rule": "local[B] = local[B] * C_B for every frame; C_B ~ inverse(local_DB[B]) * local_REF[B] (game-local x,y,z,w), "
                "re-expressed as a model-space correction and mirror-symmetrized (left/right averaged) when symmetrize=true",
        "symmetrize": bool(a.symmetrize),
        "db_clip": a.db_clip, "ref_anim": a.ref_anim, "ref_glb": os.path.relpath(REF_GLB, ROOT).replace("\\", "/"),
        "mode": a.mode, "db_frames": f"0..{nloop - 1}" if a.mode == "avg" else "0",
        "ref_frames": f"0..{nref - 1}" if a.mode == "avg" else "0",
        "note": "Offsets are measured vs the game reference %s idle. "
                "unsymmetrized_* = plain inverse(local_DB)*local_REF." % ("female" if "fem" in os.path.basename(REF_GLB) else
                                                                          os.path.basename(REF_GLB)),
        "rig": os.path.basename(RIG).split(".")[0],
        "offsets": {k: res[k] for k in sorted(res, key=lambda n: bi[n])}, "mirror_check": mirror,
    }
    if os.path.exists(a.out):  # keep hand-written posture classes (e.g. classes.crouch, applied by crouch_posture.py)
        prev = json.load(open(a.out, encoding="utf-8")).get("classes")
        if prev:
            out["classes"] = prev
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    for bn, v in out["offsets"].items():
        print(f"{bn:14s} {v['angle_deg']:7.3f} deg (unsymmetrized {v['unsymmetrized_angle_deg']:7.3f})  "
              f"axis(local) {np.round(v['axis_local'], 3)}")
    for k, v in mirror.items():
        print(f"mirror {k}: L {v['model_delta_left_deg']:.3f} R {v['model_delta_right_deg']:.3f} "
              f"L-vs-mirror(R) {v['angle_between_left_and_mirrored_right_deg']:.3f} deg")


# ---------------------------------------------------------------- apply

def cmd_apply(a):
    names, _ = load_rig_names()
    off = json.load(open(a.offsets, encoding="utf-8"))
    os.makedirs(a.dst, exist_ok=True)
    if os.path.abspath(a.dst) == os.path.abspath(a.src):
        raise SystemExit("dst must differ from src (originals are never overwritten)")
    files = sorted(glob.glob(os.path.join(a.src, "*.utppclip.json")))
    for k, fpath in enumerate(files):
        c = json.load(open(fpath, encoding="utf-8"))
        if list(c["bone_names"]) != names:
            raise SystemExit(f"{fpath}: bone_names differ")
        Q = np.array([[bn[3:7] for bn in fr["bones"]] for fr in c["frames"]], dtype=np.float64)
        Qn = apply_offsets_array(Q, names, off)
        Pa = np.array([[bn[0:3] for bn in fr["bones"]] for fr in c["frames"]], dtype=np.float64)
        Pn = apply_offsets_P(Pa, names, off)
        for fr, qf, pf in zip(c["frames"], Qn, Pn):
            for b in {names.index(n) for n in off["offsets"]}:
                fr["bones"][b][3:7] = [float(x) for x in qf[b]]
                fr["bones"][b][0:3] = [float(x) for x in pf[b]]
        c["posture_offsets"] = {"file": os.path.basename(a.offsets), "bones": sorted(off["offsets"])}
        with open(os.path.join(a.dst, os.path.basename(fpath)), "w", encoding="utf-8", newline="\n") as f:
            json.dump(c, f, separators=(",", ":"))
        if (k + 1) % 100 == 0:
            print(f"{k + 1}/{len(files)}", flush=True)
    print(f"APPLY_OK {len(files)} clips -> {a.dst}")


# ---------------------------------------------------------------- metrics

def elev(v):
    return math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))


def seg_dist(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return float(np.linalg.norm(p - (a + t * ab)))


def pose_metrics(gp, bi):
    """gp (N,73,3) model-space head positions; returns dict of per-frame-averaged metrics + hand clearance minima."""
    g = lambda n: gp[:, bi[n]]
    arms = (g("LeftArm") + g("RightArm")) / 2
    m = {
        "width_x_m": float(np.mean(np.abs(g("LeftArm")[:, 0] - g("RightArm")[:, 0]))),
        "width_3d_m": float(np.mean(np.linalg.norm(g("LeftArm") - g("RightArm"), axis=1))),
        "head_above_shoulders_m": float(np.mean(g("Head")[:, 2] - arms[:, 2])),
        "clavicle_elev_deg": float(np.mean([elev(v) for v in g("LeftArm") - g("LeftShoulder")])),
        "clavicle_elev_R_deg": float(np.mean([elev(v) for v in g("RightArm") - g("RightShoulder")])),
        "neck_elev_deg": float(np.mean([elev(v) for v in g("Neck") - g("Spine3")])),
    }
    # hand clearance: min over frames of distance hand -> thigh segment (UpLeg->Leg) and -> torso axis (Hips->Spine3), cm
    for side in ("Left", "Right"):
        h = g(side + "Hand")
        th = [seg_dist(h[f], g(side + "UpLeg")[f], g(side + "Leg")[f]) for f in range(len(h))]
        to = [seg_dist(h[f], g("Hips")[f], g("Spine3")[f]) for f in range(len(h))]
        m[f"min_hand_{side}_to_thigh_cm"] = 100 * min(th)
        m[f"min_hand_{side}_to_torso_axis_cm"] = 100 * min(to)
    return m


def cmd_metrics(a):
    names, parents = load_rig_names()
    bi = {n: i for i, n in enumerate(names)}
    rows = []
    off = json.load(open(a.offsets, encoding="utf-8")) if a.offsets else None
    tag = os.path.basename(a.offsets) if a.offsets else "orig"
    for cn in a.clip or []:
        c, B = read_clip(os.path.join(a.clips, cn + ".utppclip.json"))
        n = B.shape[0] - 1 if cn.find("_Loop") >= 0 else B.shape[0]
        P, Q = B[:n, :, 0:3], B[:n, :, 3:7].copy()
        if off:
            Q = apply_offsets_array(Q, names, off)
            P = apply_offsets_P(P, names, off)
        gp, _ = mb.fk_model(P, Q, parents)
        rows.append((f"DB {cn} [{tag}]", pose_metrics(gp, bi)))
    for an in a.ref_anim or []:
        P, Q = read_ref(an, names)
        n = P.shape[0] - 1 if P.shape[0] > 40 else P.shape[0]
        gp, _ = mb.fk_model(P[:n], Q[:n], parents)
        rows.append((f"REF {an}", pose_metrics(gp, bi)))
    keys = list(rows[0][1].keys())
    print("metric".ljust(34) + "".join(r[0][:34].rjust(36) for r in rows))
    for k in keys:
        print(k.ljust(34) + "".join(f"{r[1][k]:36.3f}" for r in rows))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rig", default=None, help="target rig json (default: player_woman_skeleton; male V: "
                    "Extracted/game/json/vanilla-man/player_man_skeleton.rig.json); give before the sub-command")
    ap.add_argument("--ref-glb", default=None, help="reference glb of the same rig (default: tpp_fem_v_locomotion; male V: "
                    "Extracted/game/glb/reference/tpp_male_v_locomotion.anims.glb); give before the sub-command")
    sp = ap.add_subparsers(dest="cmd", required=True)
    c = sp.add_parser("compute")
    c.add_argument("--clips", default=CLIPS)
    c.add_argument("--db-clip", default="M_Neutral_Stand_Idle_Loop")
    c.add_argument("--ref-anim", default="fpp_idle_stand")
    c.add_argument("--bones", nargs="+", default=["LeftShoulder", "RightShoulder", "Neck", "Neck1"])
    c.add_argument("--mode", choices=["avg", "frame0"], default="avg")
    c.add_argument("--scale-bones", nargs="*", default=["LeftShoulder", "RightShoulder", "LeftArm", "RightArm", "Neck", "Neck1", "Head"])
    c.add_argument("--no-symmetrize", dest="symmetrize", action="store_false")
    c.add_argument("--out", default=DEFAULT_OFFSETS)
    p = sp.add_parser("apply")
    p.add_argument("--src", default=CLIPS)
    p.add_argument("--dst", default=os.path.join(ROOT, "Animations", "retarget", "out_posture"))
    p.add_argument("--offsets", default=DEFAULT_OFFSETS)
    m = sp.add_parser("metrics")
    m.add_argument("--clips", default=CLIPS)
    m.add_argument("--clip", nargs="*")
    m.add_argument("--ref-anim", nargs="*")
    m.add_argument("--offsets", default=None)
    a = ap.parse_args()
    global RIG, REF_GLB
    if a.rig:
        RIG = a.rig
    if a.ref_glb:
        REF_GLB = a.ref_glb
    {"compute": cmd_compute, "apply": cmd_apply, "metrics": cmd_metrics}[a.cmd](a)


if __name__ == "__main__":
    main()
