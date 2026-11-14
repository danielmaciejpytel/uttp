"""dodgebake.py - write the player's ground dodge clips as utpp_dodge_<body>.utpptrav (.utpptrav version 2, getup grammar).

Reads the player animation set player_exploration.anims (unbundled glTF under Extracted/game/player-anims, never modified):
fpp_dodge_front / back / left / right on the player rig of the body, and writes per body:
  <out>/utpp_dodge_<body>.utpptrav      4 montages Dodge_F, Dodge_B, Dodge_L, Dodge_R in that order
  <out>/dodge_report_<body>.json        measured numbers (hips offset removed, gates, push-off time, mirror comparison)

File grammar = FORMAT_TRAVERSAL.txt version 2 as the get-up files (separate file, the traversal planner never sees it):
  RIG / POSEBONES   player rig identity; POSEBONES = the first 73 joints of the player rig (the motion database bones).
  M line            length = (frames-1)/fps, rate 1.0, blend in 0.08 / out 0.2 SmoothStep, action Other, fps 30, rtz 0.
  SEC               Default 0; PushOff = time of the peak hips speed in the clip's own world path (root + hips);
                    Recover = first time after PushOff where the hips world speed falls below 30 % of the peak.
  R lines           the clip's Root track (x y 0 yaw), reference only: the runtime never applies it (the capsule moves the body).
  P lines           local pose, Root = identity, in place. The constant offset of the hips behind the Root (the mean hips Y
                    in the Root frame over the clip) is removed from the Hips local translation; nothing else changes.
Gates (any failure = no file): finite values, unit quaternions, every non-Hips local translation equal to the rig reference
(bone lengths), the hard anatomy gate (tools/melee/anatomy_gate.py, absolute features; the delta budgets are off because the
only edit is the rigid hips shift) of the written pose against the source clip, no joint step above the source clip's own
largest step (the source knees turn up to 43 deg per 30 fps frame), soles above -0.03 m.

usage: python -I dodgebake.py --out <dir> [--body woman|man ...] [--native-left]
  default         Dodge_L = the mirrored fpp_dodge_right: the native left clip pushes off later (0.267 s vs 0.133-0.200 s),
                  lasts longer (0.633 s vs 0.433 s) and is slower (hips peak 2.4 vs 3.0 m/s); the report lists both.
  --native-left   Dodge_L = the native fpp_dodge_left.
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "melee"))
import travbake as T  # noqa: E402

CLIPS = (("F", "fpp_dodge_front"), ("B", "fpp_dodge_back"), ("L", "fpp_dodge_left"), ("R", "fpp_dodge_right"))
SET = "player_exploration"
SUB = {"woman": "female", "man": "male"}
RIG_JSON = {"woman": "Extracted/game/json/vanilla/player_woman_skeleton.rig.json",
            "man": "Extracted/game/json/vanilla-man/player_man_skeleton.rig.json"}
NUM_POSE_BONES = 73
BLEND_IN = 0.08
BLEND_OUT = 0.2
MIN_SOLE = -0.03
BONE_TOL = 1e-3
RECOVER_FRAC = 0.30


def _libs():
    import anatomy_gate as AG  # noqa: E402
    import melee_common as MC  # noqa: E402
    import npc_retarget as NR  # noqa: E402
    return AG, MC, NR


def quat_angle_deg(a, b):
    d = np.abs(np.sum(a * b, axis=-1))
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def mat_yaw(Rm):
    """yaw about +Z of the +Y axis of a 3x3 rotation (0 = +Y, CCW positive)."""
    f = Rm[:, 1]
    return math.atan2(-f[0], f[1])


def push_off(world_hips, fps):
    """(push-off time, recover time, peak speed) of the hips world path [T,3]."""
    v = np.zeros(len(world_hips))
    v[1:] = np.linalg.norm(np.diff(world_hips[:, :2], axis=0), axis=1) * fps
    k = int(np.argmax(v))
    r = len(v) - 1
    for i in range(k + 1, len(v)):
        if v[i] < RECOVER_FRAC * v[k]:
            r = i
            break
    return k / fps, r / fps, float(v[k]), v


def clip_data(AG, MC, NR, P, clip, mirror, rig_rest_t):
    d = NR.Donor(P, clip, SET, mirror=mirror)
    lt = np.array(d.lt, dtype=np.float64)[:, :NUM_POSE_BONES].copy()
    lq = np.array(d.lr, dtype=np.float64)[:, :NUM_POSE_BONES].copy()
    par = P.par[:NUM_POSE_BONES]
    names = P.names[:NUM_POSE_BONES]
    ix = {n: i for i, n in enumerate(names)}
    hips = ix["Hips"]
    if any(p >= NUM_POSE_BONES for p in par):
        raise SystemExit("pose bone parent outside the 73 bones")
    if par[hips] != ix["Root"]:
        raise SystemExit("Hips parent is not Root")
    W0 = MC.fk_world(par, lt, lq)
    # world path of the hips (clip Root track applied) for the push-off time
    hw = np.einsum("tij,tj->ti", d.root_R, W0[:, hips, :3, 3]) + d.root_t
    t_push, t_rec, v_peak, _ = push_off(hw, MC.FPS)
    # only the offset behind the Root (Y): the lateral hips path (X) is the push-off weight shift of the side clips
    off = np.array([0.0, lt[:, hips, 1].mean()])
    lt[:, hips, 1] -= off[1]
    W = MC.fk_world(par, lt, lq)
    # gates
    err = []
    if not (np.all(np.isfinite(lt)) and np.all(np.isfinite(lq))):
        err.append("non-finite")
    qn = np.linalg.norm(lq, axis=-1)
    if np.max(np.abs(qn - 1.0)) > 1e-4:
        err.append("quaternion norm %.6f" % float(np.max(np.abs(qn - 1.0))))
    lq = lq / qn[..., None]
    for i, n in enumerate(names):
        if n in ("Root", "Hips"):
            continue
        dev = float(np.max(np.linalg.norm(lt[:, i] - rig_rest_t[n], axis=-1)))
        if dev > BONE_TOL:
            err.append("bone %s translation off the rig reference by %.4f m" % (n, dev))
    lq0 = np.array(d.lr, dtype=np.float64)[:, :NUM_POSE_BONES]
    step = quat_angle_deg(lq[1:], lq[:-1]).max() if len(lq) > 1 else 0.0
    step_src = quat_angle_deg(lq0[1:], lq0[:-1]).max() if len(lq0) > 1 else 0.0
    if step > step_src + 0.01:
        err.append("joint step %.1f deg above the source clip (%.1f)" % (step, step_src))
    Wt = W[:, :, :3, 3]
    soles = NR.sole_heights(P, np.concatenate([Wt, np.zeros((len(Wt), len(P.names) - NUM_POSE_BONES, 3))], axis=1))
    if soles.min() < MIN_SOLE:
        err.append("sole %.3f m" % float(soles.min()))
    rg = AG.Rig(list(P.ref["names"]), P.ref["parents"], P.ref["rest"])
    # anatomy gate without the delta budgets: the only edit is the rigid hips XY shift (delta_hips_cm = the removed offset)
    g = AG.gate(W, W0, np.arange(len(W), dtype=float), rg, deltas=False)
    if not g["passed"]:
        err.append("anatomy gate: %s" % json.dumps(g["violations"][:3]))
    traj = np.zeros((len(lt), 4))
    traj[:, :2] = d.root_t[:, :2]
    traj[:, 3] = [mat_yaw(Rm) for Rm in d.root_R]
    hip_rel = W[:, hips, :3, 3]
    rep = dict(clip=clip, mirror=mirror, frames=len(lt), dur=round((len(lt) - 1) / MC.FPS, 4), missing=len(d.missing),
               hips_offset_removed=[round(float(x), 4) for x in off], push_off_s=round(t_push, 4), recover_s=round(t_rec, 4),
               hips_peak_speed=round(v_peak, 3), root_disp=[round(float(x), 3) for x in d.root_t[-1] - d.root_t[0]],
               hips_z_first_last=[round(float(hip_rel[0, 2]), 4), round(float(hip_rel[-1, 2]), 4)],
               hips_xy_first_last=[[round(float(x), 4) for x in hip_rel[0, :2]], [round(float(x), 4) for x in hip_rel[-1, :2]]],
               max_joint_step_deg=round(float(step), 2), source_max_joint_step_deg=round(float(step_src), 2), sole_min=round(float(soles.min()), 4),
               anatomy_gate=dict(passed=g["passed"], margin_min=g["margin_min"], worst=g["worst_margins"][:3]), errors=err)
    return dict(lt=lt, lq=lq, traj=traj, names=names, rep=rep, t_push=t_push, t_rec=t_rec)


def montage_lines(name, seq, c):
    frames = len(c["lt"])
    length = (frames - 1) / 30.0
    lines = ["M %s %s %s 1.000000 %s SmoothStep %s SmoothStep 1 -1.000000 Other 30.000000 %d 0" % (
        name, seq, T.f6(length), T.f6(BLEND_IN), T.f6(BLEND_OUT), frames)]
    lines.append("SEC Default 0.000000 -")
    lines.append("SEC PushOff %s -" % T.f6(c["t_push"]))
    lines.append("SEC Recover %s -" % T.f6(c["t_rec"]))
    for f in range(frames):
        tr = c["traj"][f]
        lines.append("R %s %s %s %s" % (T.f6(tr[0]), T.f6(tr[1]), T.f6(0.0), T.f6(tr[3])))
    lines.append("C Distance_From_Ledge -")
    lt, lq = c["lt"], c["lq"]
    for f in range(frames):
        lines.append(T.pose_line([[lt[f, b, 0], lt[f, b, 1], lt[f, b, 2], lq[f, b, 0], lq[f, b, 1], lq[f, b, 2], lq[f, b, 3]]
                                  for b in range(len(c["names"]))]))
    lines.append("E")
    return lines


def rig_rest(root, body):
    rc = T.load_json(os.path.join(root, RIG_JSON[body]))["Data"]["RootChunk"]
    names = [b["$value"] if isinstance(b, dict) else b for b in rc["boneNames"]]
    out = {}
    for n, t in zip(names, rc["boneTransforms"]):
        tr = t["Translation"]
        out[n] = np.array([float(tr["X"]), float(tr["Y"]), float(tr["Z"])])
    return out


def bake_body(root, body, out_dir, mirror_left=True):
    AG, MC, NR = _libs()
    P = NR.Player(body)
    NR.NPC_DIR = os.path.join(root, "Extracted", "game", "player-anims", "glb", "base", "animations", "player", SUB[body], "body")
    rig_names, _, rig_hash = T.rig_info(os.path.join(root, RIG_JSON[body]))
    if list(rig_names) != list(P.names):
        raise SystemExit("player glb joint order differs from the rig")
    rest = rig_rest(root, body)
    clips = {}
    for k, clip in CLIPS:
        clips[k] = clip_data(AG, MC, NR, P, clip, False, rest)
    mirrored = clip_data(AG, MC, NR, P, "fpp_dodge_right", True, rest)
    report = {"body": body, "set": SET, "clips": {k: clips[k]["rep"] for k, _ in CLIPS}, "mirrored_right_as_left": mirrored["rep"]}
    if mirror_left:
        clips["L"] = mirrored
    report["left_source"] = "mirrored fpp_dodge_right" if mirror_left else "fpp_dodge_left"
    bad = {k: c["rep"]["errors"] for k, c in clips.items() if c["rep"]["errors"]}
    if bad:
        raise SystemExit("%s: gates failed %s" % (body, json.dumps(bad)))
    rig_name = os.path.basename(RIG_JSON[body]).split(".")[0]
    names = clips["F"]["names"]
    out = ["utpp-traversal 2", "RIG %s 0x%016x %d" % (rig_name, rig_hash, len(rig_names)),
           "POSEBONES %d %s" % (len(names), " ".join(names)), "COUNT 4"]
    for k, clip in CLIPS:
        out += montage_lines("Dodge_" + k, ("mirror_" if (k == "L" and mirror_left) else "") + (
            "fpp_dodge_right" if (k == "L" and mirror_left) else clip), clips[k])
    out.append("END")
    data = ("\n".join(out) + "\n").encode("ascii")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "utpp_dodge_%s.utpptrav" % body)
    with open(path, "wb") as fh:
        fh.write(data)
    report["file"] = {"name": os.path.basename(path), "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    with open(os.path.join(out_dir, "dodge_report_%s.json" % body), "wb") as fh:
        fh.write(json.dumps(report, indent=1).encode("utf-8"))
    return report


def project_root(arg):
    if arg:
        return arg
    p = HERE
    for _ in range(8):
        if os.path.isdir(os.path.join(p, "Extracted")):
            return p
        p = os.path.dirname(p)
    raise SystemExit("project root not found (--root)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root")
    ap.add_argument("--out", required=True)
    ap.add_argument("--body", action="append", choices=("woman", "man"))
    ap.add_argument("--native-left", action="store_true")
    a = ap.parse_args()
    root = project_root(a.root)
    for body in a.body or ["woman", "man"]:
        r = bake_body(root, body, a.out, not a.native_left)
        print(body, r["file"]["name"], r["file"]["bytes"], r["file"]["sha256"])
        for k, c in r["clips"].items():
            print("  %s %s dur %.3f push %.3f rec %.3f peak %.2f off %s z %s step %.1f sole %.3f gate %s" % (
                k, c["clip"], c["dur"], c["push_off_s"], c["recover_s"], c["hips_peak_speed"], c["hips_offset_removed"],
                c["hips_z_first_last"], c["max_joint_step_deg"], c["sole_min"], c["anatomy_gate"]["margin_min"]))
        m = r["mirrored_right_as_left"]
        print("  L(mirror) dur %.3f push %.3f rec %.3f peak %.2f gate %s errors %s" % (
            m["dur"], m["push_off_s"], m["recover_s"], m["hips_peak_speed"], m["anatomy_gate"]["margin_min"], m["errors"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
