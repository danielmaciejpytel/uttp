"""knee_metrics.py - knee diagnostics of the source pack(-ALS) -> CP retarget: UE source vs retarget output.

Two modes:
  1) extract-ue (inside Blender, headless):
       blender -b --factory-startup -P knee_metrics.py -- extract-ue --fbx-dir <fbx> --meta-dir <meta>
               --cache <dir> [--list clips.txt | --clip-map map.json --group locomotion] [--limit N]
     imports every UE FBX (same import options and frame sampling as retarget_batch.py: Blender frames 1..num_frames)
     and stores the armature-space thigh/calf/foot/ball heads and world rotations (+ bind pose) of both legs to
     <cache>/<clip>.npz. Resumable (existing npz files are skipped). source-pack-derived data: keep the cache outside git.
  2) metrics (plain python + numpy):
       python knee_metrics.py metrics --cache <dir> --out-dir <retarget out> --label woman --report-dir <dir>
               [--list ... | --clip-map ... --group locomotion]
     computes per clip / frame / leg for the UE source and the retarget output (CP rig, FK of the utppclip local
     transforms; rig json picked from the clip 'rig' field):
       flex          angle between thigh (hip->knee) and shin (knee->ankle), 0 = straight leg
       off_bone      knee hinge off-plane: the shin's anatomical hinge axis (bind-pose lateral hip axis
                     thigh_l-thigh_r expressed in the shin bone frame, carried by the animated shin) vs the normal
                     of the hip-knee-ankle plane; 0 = the knee bends exactly about its rigged hinge
       off_delta     same, with the axis of the knee's rotation relative to its bind pose (thigh-local delta)
       theta         knee direction vs toes: signed angle, around the hip->ankle axis, from the foot forward
                     (ankle->toe) to the knee bend direction (knee minus its projection on the hip-ankle line);
                     + = knee turned outward (towards the leg's own side of the pelvis)
       dtheta        per-frame change of theta (wrapped)
       seam          looping clips (meta loop=True): last unique frame (N-2; frame N-1 duplicates frame 0 in the
                     export) -> frame 0 jumps of flex / theta, plus the mismatch of the duplicate frame N-1 vs 0
     writes <report-dir>/knee_<label>.json (per clip summary + aggregates + worst clips), knee_<label>_frames.csv
     (every frame / leg, UE and CP side by side) and knee_<label>.txt.

Gates: angle metrics of the knee direction / hinge are only meaningful when the knee is bent; aggregates use
flex > --flex-gate (default 20 deg) and the per-frame flip test counts |dtheta| > --flip-deg (default 30 deg) with
both frames above the gate. Both defaults are the runtime-session proposal [P] (bake-fail rule "knee direction vs
toes changes > ~30 deg per source frame at flex > 20"), not tuned here.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DEFAULT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
if os.path.basename(os.path.abspath(os.path.join(HERE, "..", "..", ".."))).lower() == "build":   # worktree Build/wt-<name>/tools/retarget
    ROOT_DEFAULT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))

UE_BONES = ("thigh_l", "calf_l", "foot_l", "ball_l", "thigh_r", "calf_r", "foot_r", "ball_r")
CP_BONES = ("LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "RightToeBase")
LEGS = (("l", 0, 4), ("r", 4, 0))   # side, first index of the leg's 4 bones, first index of the other leg
NB = 73
RIGS = {"player_woman_skeleton": os.path.join("Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json"),
        "player_man_skeleton": os.path.join("Extracted", "game", "json", "vanilla-man", "player_man_skeleton.rig.json")}


# ---------------------------------------------------------------- clip selection

def clip_list(a):
    if a.list:
        with open(a.list, encoding="utf-8-sig") as fh:
            return [ln.split("#", 1)[0].strip() for ln in fh if ln.split("#", 1)[0].strip()]
    if a.clips:
        return [c.strip() for c in a.clips.split(",") if c.strip()]
    with open(a.clip_map, encoding="utf-8") as fh:
        cm = json.load(fh)["clips"]
    return sorted(c["name"] for c in cm if c["purpose"]["group"] in a.group.split(","))


# ---------------------------------------------------------------- Blender extraction

def extract_ue(a):
    import bpy  # noqa: F401  (only inside Blender)
    os.makedirs(a.cache, exist_ok=True)
    clips = clip_list(a)
    if a.limit:
        clips = clips[:a.limit]
    done = skipped = failed = 0
    t_all = time.time()
    for i, clip in enumerate(clips):
        out = os.path.join(a.cache, clip + ".npz")
        if os.path.isfile(out):
            skipped += 1
            continue
        try:
            t0 = time.time()
            with open(os.path.join(a.meta_dir, clip + ".json"), encoding="utf-8") as fh:
                meta = json.load(fh)
            n = int(meta["num_frames"])
            bpy.ops.wm.read_factory_settings(use_empty=True)
            bpy.ops.import_scene.fbx(filepath=os.path.join(a.fbx_dir, clip + ".fbx"),
                                     automatic_bone_orientation=False, use_anim=True)
            obj = next(o for o in bpy.data.objects if o.type == "ARMATURE")
            scene = bpy.context.scene
            scene.render.fps, scene.render.fps_base = 30, 1.0
            rp = np.array([list(obj.data.bones[b].head_local) for b in UE_BONES], dtype=np.float64)
            rq = np.array([_xyzw(obj.data.bones[b].matrix_local.to_quaternion()) for b in UE_BONES], dtype=np.float64)
            P = np.zeros((n, len(UE_BONES), 3))
            Q = np.zeros((n, len(UE_BONES), 4))
            for f in range(n):
                scene.frame_set(f + 1)
                for j, b in enumerate(UE_BONES):
                    pb = obj.pose.bones[b]
                    P[f, j] = list(pb.head)
                    Q[f, j] = _xyzw(pb.matrix.to_3x3().normalized().to_quaternion())
            np.savez(out, pos=P, quat=Q, rest_pos=rp, rest_quat=rq, loop=bool(meta.get("loop")),
                     fbx_md5=str(meta.get("fbx_md5", "")))
            done += 1
            print("UTPP_KNEE_EXTRACT %d/%d %s n=%d %.2fs" % (i + 1, len(clips), clip, n, time.time() - t0), flush=True)
        except Exception as e:  # keep going, report at the end
            failed += 1
            print("UTPP_KNEE_EXTRACT_FAIL %s %r" % (clip, e), flush=True)
    print("UTPP_KNEE_EXTRACT_DONE clips=%d extracted=%d skipped=%d failed=%d %.1fs"
          % (len(clips), done, skipped, failed, time.time() - t_all), flush=True)


def _xyzw(q):
    return [q.x, q.y, q.z, q.w]


# ---------------------------------------------------------------- math

def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz], -1)


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qrot(q, v):
    u = q[..., :3]
    w = q[..., 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def unit(v, eps=1e-12):
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(n, eps), n[..., 0]


def angle(a, b):
    a, _ = unit(a)
    b, _ = unit(b)
    return np.degrees(np.arccos(np.clip(np.sum(a * b, -1), -1.0, 1.0)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def fk_model(P, Q, parents):
    gp = np.zeros_like(P)
    gq = np.zeros_like(Q)
    for b, p in enumerate(parents):
        if p < 0:
            gp[:, b], gq[:, b] = P[:, b], Q[:, b]
        else:
            gq[:, b] = qmul(gq[:, p], Q[:, b])
            gp[:, b] = gp[:, p] + qrot(gq[:, p], P[:, b])
    return gp, gq


# ---------------------------------------------------------------- per-rig leg metrics

def leg_metrics(pos, quat, rest_pos, rest_quat, hinge_local=None):
    """pos (n,8,3), quat (n,8,4) xyzw, rest (8,3)/(8,4) in one consistent space; bone order of UE_BONES/CP_BONES.
    Returns {side: {metric: (n,) array}}."""
    out = {}
    for side, i0, o0 in LEGS:
        hip, knee, ankle, toe = (pos[:, i0 + k] for k in range(4))
        q_thigh, q_shin = quat[:, i0], quat[:, i0 + 1]
        u_th, _ = unit(knee - hip)
        u_sh, _ = unit(ankle - knee)
        flex = angle(u_th, u_sh)
        n_pl, n_len = unit(np.cross(knee - hip, ankle - knee))
        valid_plane = flex > 1.0
        # anatomical hinge: bind-pose lateral pelvis axis in the bind shin frame, carried by the animated shin
        lat_rest, _ = unit(rest_pos[i0] - rest_pos[o0])
        a_local = qrot(qconj(rest_quat[i0 + 1]), lat_rest)
        ax_bone = qrot(q_shin, np.broadcast_to(a_local, q_shin[..., :3].shape))
        off_bone = np.degrees(np.arccos(np.clip(np.abs(np.sum(unit(ax_bone)[0] * n_pl, -1)), 0.0, 1.0)))
        # delta hinge: axis of (thigh^-1 shin)(t) * (thigh^-1 shin)_rest^-1, in thigh space -> world
        rel = qmul(qconj(q_thigh), q_shin)
        rel0 = qmul(qconj(rest_quat[i0]), rest_quat[i0 + 1])
        d = qmul(rel, qconj(np.broadcast_to(rel0, rel.shape)))
        d = d * np.sign(d[..., 3:4] + 1e-15)
        dang = np.degrees(2.0 * np.arccos(np.clip(d[..., 3], -1.0, 1.0)))
        ax_d = qrot(q_thigh, unit(d[..., :3])[0])
        off_delta = np.degrees(np.arccos(np.clip(np.abs(np.sum(unit(ax_d)[0] * n_pl, -1)), 0.0, 1.0)))
        off_delta = np.where(dang > 5.0, off_delta, np.nan)
        # fitted hinge (optional): fixed shin-local axis fitted over all gated frames of the rig (fit_hinges)
        n_local = qrot(qconj(q_shin), n_pl)
        if hinge_local is not None and side in hinge_local:
            ax_f = qrot(q_shin, np.broadcast_to(hinge_local[side], q_shin[..., :3].shape))
            off_fit = np.degrees(np.arccos(np.clip(np.abs(np.sum(ax_f * n_pl, -1)), 0.0, 1.0)))
        else:
            off_fit = np.full_like(flex, np.nan)
        # knee direction vs toes around the hip->ankle axis; + = outward (towards this leg's side of the pelvis)
        ha, _ = unit(ankle - hip)
        k = knee - hip
        k_perp = k - np.sum(k * ha, -1, keepdims=True) * ha
        k_ok = np.linalg.norm(k_perp, axis=-1) / np.maximum(np.linalg.norm(k, axis=-1), 1e-12) > 1e-4
        # (a) theta_toe: foot forward = ankle->ball head (degenerates when the toes point along the hip-ankle line,
        #     e.g. heel kicked up behind at flex > ~120 deg)
        f = toe - ankle
        f_perp = f - np.sum(f * ha, -1, keepdims=True) * ha
        lat = pos[:, i0] - pos[:, o0]
        lat_perp = lat - np.sum(lat * ha, -1, keepdims=True) * ha
        s = np.sign(np.sum(ha * np.cross(f_perp, lat_perp), -1))
        s = np.where(s == 0, 1.0, s)
        th_toe = np.degrees(np.arctan2(np.sum(ha * np.cross(f_perp, k_perp), -1), np.sum(f_perp * k_perp, -1))) * s
        th_toe = np.where(k_ok, th_toe, np.nan)
        # (b) theta: foot forward from the foot's lateral (ankle hinge) axis: f_ref = c * (lat_foot x hip->ankle),
        #     lat_foot = bind-pose outward pelvis axis carried by the foot bone; independent of plantar flexion
        lat_out_rest, _ = unit(rest_pos[i0] - rest_pos[o0])
        lf_local = qrot(qconj(rest_quat[i0 + 2]), lat_out_rest)
        lat_foot = qrot(quat[:, i0 + 2], np.broadcast_to(lf_local, quat[:, i0 + 2, :3].shape))
        ha_rest = rest_pos[i0 + 2] - rest_pos[i0]
        c = 1.0 if np.dot(np.cross(lat_out_rest, ha_rest), rest_pos[i0 + 3] - rest_pos[i0 + 2]) >= 0 else -1.0
        f_ref = c * np.cross(lat_foot, ha)
        lf_perp = lat_foot - np.sum(lat_foot * ha, -1, keepdims=True) * ha
        s2 = np.sign(np.sum(ha * np.cross(f_ref, lf_perp), -1))
        s2 = np.where(s2 == 0, 1.0, s2)
        th = np.degrees(np.arctan2(np.sum(ha * np.cross(f_ref, k_perp), -1), np.sum(f_ref * k_perp, -1))) * s2
        th = np.where(k_ok, th, np.nan)
        dth = np.full_like(th, np.nan)
        dth[1:] = wrap(th[1:] - th[:-1])
        out[side] = {"flex": flex, "off_bone": np.where(valid_plane, off_bone, np.nan),
                     "off_delta": np.where(valid_plane, off_delta, np.nan),
                     "off_fit": np.where(valid_plane, off_fit, np.nan),
                     "theta": th, "dtheta": dth, "theta_toe": th_toe, "_n_local": n_local}
    return out


def fit_hinges(items, gate):
    """items: list of leg_metrics results of one rig; per side the shin-local unit axis that best matches the
    hip-knee-ankle plane normal over every frame with flex > gate (principal eigenvector of sum n n^T)."""
    res = {}
    for side in ("l", "r"):
        M = np.zeros((3, 3))
        cnt = 0
        for m in items:
            g = m[side]["flex"] > gate
            nl = m[side]["_n_local"][g]
            M += nl.T @ nl
            cnt += int(g.sum())
        if cnt:
            w, v = np.linalg.eigh(M)
            res[side] = v[:, -1]
        res[side + "_frames"] = cnt
    return res


def load_cp_rig(root, rig_name, cache):
    if rig_name in cache:
        return cache[rig_name]
    path = os.path.join(root, RIGS[rig_name])
    with open(path, encoding="utf-8") as fh:
        rc = json.load(fh)["Data"]["RootChunk"]
    names = [n["$value"] if isinstance(n, dict) else str(n) for n in rc["boneNames"]][:NB]
    parents = [int(p) for p in rc["boneParentIndexes"]][:NB]
    P = np.zeros((1, NB, 3))
    Q = np.zeros((1, NB, 4))
    for b, t in enumerate(rc["boneTransforms"][:NB]):
        P[0, b] = [t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"]]
        Q[0, b] = [t["Rotation"]["i"], t["Rotation"]["j"], t["Rotation"]["k"], t["Rotation"]["r"]]
    gp, gq = fk_model(P, Q, parents)
    idx = [names.index(b) for b in CP_BONES]
    cache[rig_name] = (names, parents, gp[0, idx], gq[0, idx], idx)
    return cache[rig_name]


def load_cp_clip(path, root, rigs):
    with open(path, encoding="utf-8") as fh:
        c = json.load(fh)
    names, parents, rp, rq, idx = load_cp_rig(root, c["rig"], rigs)
    if list(c["bone_names"]) != names:
        raise ValueError("bone_names differ from rig %s" % c["rig"])
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    gp, gq = fk_model(B[:, :, 0:3], B[:, :, 3:7], parents)
    return gp[:, idx], gq[:, idx], rp, rq, c["rig"]


# ---------------------------------------------------------------- summaries

def _r(x, nd=2):
    return None if x is None or not np.isfinite(x) else round(float(x), nd)


def _nanmax(x):
    x = x[np.isfinite(x)]
    return float(x.max()) if x.size else None


def _nanmean(x):
    x = x[np.isfinite(x)]
    return float(x.mean()) if x.size else None


def _p95(x):
    x = x[np.isfinite(x)]
    return float(np.percentile(x, 95)) if x.size else None


def leg_summary(m, gate, flip):
    flex, th = m["flex"], m["theta"]
    g = flex > gate
    gg = np.zeros_like(g)
    gg[1:] = g[1:] & g[:-1]
    adth_g = np.where(gg, np.abs(m["dtheta"]), np.nan)
    dtoe = np.full_like(th, np.nan)
    dtoe[1:] = np.abs(wrap(m["theta_toe"][1:] - m["theta_toe"][:-1]))
    dtoe_g = np.where(gg, dtoe, np.nan)
    flex_step = np.abs(np.diff(flex))
    r = {"flex_max": _r(flex.max()), "flex_f0": _r(flex[0]), "theta_f0": _r(th[0]), "theta_toe_f0": _r(m["theta_toe"][0]),
         "off_bone_f0": _r(m["off_bone"][0]), "off_fit_f0": _r(m["off_fit"][0]),
         "gated_frames": int(g.sum())}
    for k in ("off_bone", "off_delta", "off_fit"):
        r[k + "_max"] = _r(_nanmax(np.where(g, m[k], np.nan)))
        r[k + "_p95"] = _r(_p95(np.where(g, m[k], np.nan)))
    r.update({"theta_mean": _r(_nanmean(np.where(g, th, np.nan))),
              "dtheta_max": _r(_nanmax(adth_g)),
              "dtheta_max_frame": int(np.nanargmax(adth_g)) if np.isfinite(adth_g).any() else None,
              "flip_frames": int(np.sum(adth_g > flip)),
              "flip_frame_list": [int(i) for i in np.nonzero(adth_g > flip)[0]][:20],
              "dtheta_toe_max": _r(_nanmax(dtoe_g)),
              "flip_frames_toe": int(np.sum(dtoe_g > flip)),
              "flex_step_max": _r(flex_step.max()) if flex_step.size else None,
              "flex_step_max_frame": int(np.argmax(flex_step) + 1) if flex_step.size else None})
    return r


def seam(m, n):
    """loop seam: last unique frame N-2 -> frame 0; duplicate check N-1 vs 0."""
    if n < 3:
        return None
    flex, th = m["flex"], m["theta"]
    return {"flex_jump": _r(abs(flex[0] - flex[n - 2])), "theta_jump": _r(abs(wrap(th[0] - th[n - 2]))),
            "interior_flex_step_max": _r(np.abs(np.diff(flex[:n - 1])).max()),
            "flex_dup_mismatch": _r(abs(flex[n - 1] - flex[0])), "theta_dup_mismatch": _r(abs(wrap(th[n - 1] - th[0])))}


def metrics(a):
    clips = clip_list(a)
    os.makedirs(a.report_dir, exist_ok=True)
    rigs = {}
    loaded, problems = [], []
    for clip in clips:
        npz = os.path.join(a.cache, clip + ".npz")
        cpp = os.path.join(a.out_dir, clip + ".utppclip.json")
        if not os.path.isfile(npz) or not os.path.isfile(cpp):
            problems.append({"clip": clip, "why": "missing ue cache" if not os.path.isfile(npz) else "missing retarget clip"})
            continue
        z = dict(np.load(npz))
        try:
            cp = load_cp_clip(cpp, a.root, rigs)
        except Exception as e:
            problems.append({"clip": clip, "why": repr(e)})
            continue
        n = z["pos"].shape[0]
        if cp[0].shape[0] != n:
            problems.append({"clip": clip, "why": "frame count ue %d cp %d" % (n, cp[0].shape[0])})
            continue
        loaded.append((clip, z, cp))
    # pass 1: fitted hinge per rig / side (fixed shin-local axis, all gated frames of the run)
    ue_items = [leg_metrics(z["pos"], z["quat"], z["rest_pos"], z["rest_quat"]) for _, z, _ in loaded]
    cp_items = {}
    for _, _, cp in loaded:
        cp_items.setdefault(cp[4], []).append(leg_metrics(cp[0], cp[1], cp[2], cp[3]))
    hinge_ue = fit_hinges(ue_items, a.flex_gate)
    hinge_cp = {rig: fit_hinges(items, a.flex_gate) for rig, items in cp_items.items()}
    del ue_items, cp_items
    hinges = {"ue": {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in hinge_ue.items()},
              "cp": {rig: {k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in h.items()} for rig, h in hinge_cp.items()}}
    # pass 2
    rows = []
    csv_path = os.path.join(a.report_dir, "knee_%s_frames.csv" % a.label)
    fcsv = open(csv_path, "w", newline="", encoding="utf-8")
    w = csv.writer(fcsv)
    cols = ("flex", "off_bone", "off_delta", "off_fit", "theta", "dtheta", "theta_toe")
    w.writerow(["clip", "frame", "leg"] + ["ue_" + c for c in cols] + ["cp_" + c for c in cols] + ["theta_cp_minus_ue"])
    for clip, z, cp in loaded:
        n = z["pos"].shape[0]
        mu = leg_metrics(z["pos"], z["quat"], z["rest_pos"], z["rest_quat"], hinge_ue)
        mc = leg_metrics(cp[0], cp[1], cp[2], cp[3], hinge_cp[cp[4]])
        loop = bool(z["loop"])
        row = {"clip": clip, "frames": n, "loop": loop, "rig": cp[4]}
        for side in ("l", "r"):
            dth = wrap(mc[side]["theta"] - mu[side]["theta"])
            dtoe = wrap(mc[side]["theta_toe"] - mu[side]["theta_toe"])
            gate = (mu[side]["flex"] > a.flex_gate) & (mc[side]["flex"] > a.flex_gate)
            row[side] = {"ue": leg_summary(mu[side], a.flex_gate, a.flip_deg),
                         "cp": leg_summary(mc[side], a.flex_gate, a.flip_deg),
                         "theta_cp_minus_ue_f0": _r(dth[0]),
                         "theta_toe_cp_minus_ue_f0": _r(dtoe[0]),
                         "theta_cp_minus_ue_mean_gated": _r(_nanmean(np.where(gate, dth, np.nan))),
                         "theta_cp_minus_ue_absmax_gated": _r(_nanmax(np.abs(np.where(gate, dth, np.nan))))}
            if loop:
                row[side]["ue"]["seam"] = seam(mu[side], n)
                row[side]["cp"]["seam"] = seam(mc[side], n)
            for f in range(n):
                w.writerow([clip, f, side] + [_r(mu[side][c][f], 3) for c in cols] + [_r(mc[side][c][f], 3) for c in cols]
                           + [_r(dth[f], 3)])
        rows.append(row)
    fcsv.close()
    rep = {"tool": "knee_metrics.py", "label": a.label, "out_dir": a.out_dir, "cache": a.cache,
           "flex_gate_deg": a.flex_gate, "flip_deg": a.flip_deg, "clips_requested": len(clips), "clips": len(rows),
           "fitted_hinges_shin_local": hinges,
           "problems": problems, "aggregates": aggregates(rows), "worst": worst(rows, a.top),
           "rows": rows}
    jp = os.path.join(a.report_dir, "knee_%s.json" % a.label)
    with open(jp, "w", encoding="utf-8") as fh:
        json.dump(rep, fh, indent=1)
    txt = text_report(rep)
    with open(os.path.join(a.report_dir, "knee_%s.txt" % a.label), "w", encoding="utf-8") as fh:
        fh.write(txt)
    print(txt)
    print("UTPP_KNEE_METRICS_DONE %s clips=%d problems=%d" % (a.label, len(rows), len(problems)))


def _vals(rows, side, src, key, sub=None):
    v = []
    for r in rows:
        x = r[side][src] if src in ("ue", "cp") else r[side]
        if sub:
            x = x.get(sub) or {}
        x = x.get(key)
        if x is not None:
            v.append(x)
    return np.array(v, dtype=np.float64)


def _stats(v):
    if v.size == 0:
        return None
    return {"n": int(v.size), "mean": _r(v.mean()), "p50": _r(np.percentile(v, 50)), "p95": _r(np.percentile(v, 95)),
            "max": _r(v.max())}


def aggregates(rows):
    ag = {}
    for side in ("l", "r"):
        s = {}
        for src in ("ue", "cp"):
            s[src] = {k: _stats(_vals(rows, side, src, k)) for k in
                      ("off_bone_max", "off_bone_p95", "off_delta_max", "off_fit_max", "off_fit_p95", "theta_mean",
                       "dtheta_max", "dtheta_toe_max", "flex_step_max")}
            s[src]["clips_with_flip_frames"] = int(sum(1 for r in rows if r[side][src]["flip_frames"] > 0))
            s[src]["flip_frames_total"] = int(sum(r[side][src]["flip_frames"] for r in rows))
            s[src]["clips_with_flip_frames_toe"] = int(sum(1 for r in rows if r[side][src]["flip_frames_toe"] > 0))
            s[src]["seam_flex_jump"] = _stats(_vals(rows, side, src, "flex_jump", "seam"))
            s[src]["seam_theta_jump"] = _stats(_vals(rows, side, src, "theta_jump", "seam"))
            s[src]["seam_flex_dup_mismatch"] = _stats(_vals(rows, side, src, "flex_dup_mismatch", "seam"))
        s["theta_cp_minus_ue_mean_gated"] = _stats(_vals(rows, side, None, "theta_cp_minus_ue_mean_gated"))
        s["theta_cp_minus_ue_f0"] = _stats(_vals(rows, side, None, "theta_cp_minus_ue_f0"))
        ag[side] = s
    return ag


def worst(rows, top):
    def key_list(fn, label):
        sc = []
        for r in rows:
            for side in ("l", "r"):
                v = fn(r, side)
                if v is not None:
                    sc.append((v, r["clip"], side))
        sc.sort(reverse=True)
        return [{"value": v, "clip": c, "leg": s} for v, c, s in sc[:top]]

    def cpv(k):
        return lambda r, s: r[s]["cp"].get(k)

    def excess(k):
        return lambda r, s: (None if r[s]["cp"].get(k) is None or r[s]["ue"].get(k) is None
                             else round(r[s]["cp"][k] - r[s]["ue"][k], 2))

    def seamj(k):
        return lambda r, s: (None if not r["loop"] else round(r[s]["cp"]["seam"][k] - r[s]["ue"]["seam"][k], 2))

    return {"cp_dtheta_max_minus_ue": key_list(excess("dtheta_max"), "dtheta"),
            "cp_flip_frames": key_list(cpv("flip_frames"), "flip"),
            "cp_off_bone_max": key_list(cpv("off_bone_max"), "off"),
            "cp_off_fit_max": key_list(cpv("off_fit_max"), "offfit"),
            "cp_flex_step_max_minus_ue": key_list(excess("flex_step_max"), "flexstep"),
            "theta_cp_minus_ue_absmax_gated": key_list(lambda r, s: r[s]["theta_cp_minus_ue_absmax_gated"], "dth"),
            "seam_flex_jump_cp_minus_ue": key_list(seamj("flex_jump"), "seam"),
            "seam_theta_jump_cp_minus_ue": key_list(seamj("theta_jump"), "seamth")}


def text_report(rep):
    L = ["knee_metrics %s: %d clips (%d requested), flex gate %.0f deg, flip %.0f deg/frame; problems %d"
         % (rep["label"], rep["clips"], rep["clips_requested"], rep["flex_gate_deg"], rep["flip_deg"], len(rep["problems"]))]
    for side in ("l", "r"):
        s = rep["aggregates"][side]
        L.append("leg %s" % side)
        for src in ("ue", "cp"):
            for k, v in s[src].items():
                L.append("  %s %-26s %s" % (src, k, v))
        L.append("  theta_cp_minus_ue_mean_gated %s" % s["theta_cp_minus_ue_mean_gated"])
        L.append("  theta_cp_minus_ue_f0         %s" % s["theta_cp_minus_ue_f0"])
    for k, lst in rep["worst"].items():
        L.append("worst %s:" % k)
        for e in lst:
            L.append("  %8s  %-55s %s" % (e["value"], e["clip"], e["leg"]))
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------- CLI

def main(argv):
    ap = argparse.ArgumentParser(prog="knee_metrics.py")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("extract-ue", "metrics"):
        p = sub.add_parser(name)
        p.add_argument("--root", default=ROOT_DEFAULT)
        p.add_argument("--cache", required=True, help="UE npz cache dir")
        p.add_argument("--list", default="")
        p.add_argument("--clips", default="")
        p.add_argument("--clip-map", default=os.path.join(ROOT_DEFAULT, "Research", "inventory", "sourcepack-clip-map.json"))
        p.add_argument("--group", default="locomotion", help="clip-map purpose group(s), comma separated")
        if name == "extract-ue":
            p.add_argument("--fbx-dir", required=True)
            p.add_argument("--meta-dir", required=True)
            p.add_argument("--limit", type=int, default=0)
        else:
            p.add_argument("--out-dir", required=True, help="retarget output dir (*.utppclip.json)")
            p.add_argument("--label", required=True)
            p.add_argument("--report-dir", required=True)
            p.add_argument("--flex-gate", type=float, default=20.0, help="deg [P] runtime-session proposal")
            p.add_argument("--flip-deg", type=float, default=30.0, help="deg per frame [P] runtime-session proposal")
            p.add_argument("--top", type=int, default=15)
    a = ap.parse_args(argv)
    if a.cmd == "extract-ue":
        extract_ue(a)
    else:
        metrics(a)


if __name__ == "__main__":
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
    main(args)
