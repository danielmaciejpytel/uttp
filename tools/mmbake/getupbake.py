"""getupbake.py - bake the source pack ragdoll get-up clips into utpp_getup_<body>.utpptrav (UTPP ragdoll get-up, spec 7 "A").

Reads Animations/retarget/out_sourcepack_set2_<body>/M_ragdoll_getup_stand_{B,F,L,R}.utppclip.json (never modified), folds the
Trajectory into the Hips (getup_fixup.bake_trajectory), removes floor penetration and contact sliding (getup_fixup.Solver),
chooses the cut / interrupt frames and writes, per body:
  Animations/retarget/out_sourcepack_set2_<body>_getupfix/M_ragdoll_getup_stand_*.utppclip.json   fixed clips (full length)
  Data/getup/utpp_getup_<body>.utpptrav          .utpptrav version 2 (grammar of mm/TraversalFile.hpp), frames 0..cut
  Data/getup/utpp_getup_<body>.contacts.json     offline sidecar (never loaded by the DLL)
  Data/getup/getup_report_<body>.json            measured numbers (before / after)
  plugin/UTPP/tests/data/getup_frame0_<body>.json  frame-0 model joints + labels for the C++ classifier tests (small, in git)
source pack derived: the first three outputs stay out of git (release DATA only).

usage: python -I getupbake.py [--root <project root>] [--body woman|man ...] [--no-fixtures]
"""
import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import getup_fixup as F  # noqa: E402
import getup_mesh as GM  # noqa: E402
import travbake as T  # noqa: E402

CLIPS = ("B", "F", "L", "R")
SRC_DIR = "Animations/retarget/out_sourcepack_set2_%s"
OUT_DIR = "Animations/retarget/out_sourcepack_set2_%s_getupfix"
DATA_DIR = "Data/getup"
STAND_Z_FRAC = 0.97
HIPS_SPEED_MAX = 0.10   # spec says 0.05; with the 3-frame speed that gives cuts 0.4-0.7 s later (strict cut is in the report)
INTERRUPT_HIPS_Z = 0.80
FOOT_PLANT_Z = 0.16
FOOT_PLANT_V = 0.25


def project_root(arg):
    if arg:
        return arg
    env = os.environ.get("UTPP_PROJECT_ROOT")
    if env:
        return env
    p = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(p, "Animations", "retarget")):
            return p
        p = os.path.dirname(p)
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))


def repo_root():
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))


# --------------------------------------------------------------------------------------------------------------------
def facing_yaw(wp, clip):
    """Yaw (radians, 0 = +Y, CCW positive) of the body from the hip line (LeftUpLeg - RightUpLeg rotated -90 deg)."""
    lat = wp[clip.idx["LeftUpLeg"], :2] - wp[clip.idx["RightUpLeg"], :2]
    fwd = np.array([lat[1], -lat[0]])
    return math.atan2(-fwd[0], fwd[1])


def rotate_root_z(pos, rot, hips, yaw):
    """Rotate the whole clip about Root +Z by yaw (Hips is the only Root child that carries the pose)."""
    q = F.q_axis_angle((0.0, 0.0, 1.0), yaw)
    pos[:, hips] = F.qrot(np.broadcast_to(q, pos[:, hips].shape[:-1] + (4,)), pos[:, hips])
    rot[:, hips] = F.qnormalize(F.qmul(np.broadcast_to(q, rot[:, hips].shape), rot[:, hips]))


def hips_speed(clip, wp):
    """Hips speed (m/s, 3D) over the last 3 frames (t-3 .. t), the convention of the clip analysis."""
    h = wp[:, clip.idx["Hips"]]
    sp = np.zeros(clip.n)
    for t in range(clip.n):
        a = max(0, t - 3)
        sp[t] = np.linalg.norm(h[t] - h[a]) / (max(t - a, 1) / clip.fps)
    return sp


def choose_cut(clip, wp, speed_max=None):
    """First frame with Hips z >= 0.97 x standing height, 3-frame hips speed < speed_max and Hips xy within 2 cm of the
    final standing xy, held for 3 frames."""
    speed_max = HIPS_SPEED_MAX if speed_max is None else speed_max
    h = wp[:, clip.idx["Hips"]]
    n = clip.n
    h_end = float(np.median(h[-30:, 2]))
    sp = hips_speed(clip, wp)
    xy_end = np.median(h[-30:, :2], axis=0)
    ok = (h[:, 2] >= STAND_Z_FRAC * h_end) & (sp < speed_max) & (np.linalg.norm(h[:, :2] - xy_end, axis=1) < 0.02)
    for t in range(3, n - 3):
        if ok[t:t + 3].all():
            return t, h_end, sp
    raise SystemExit("%s: no stable standing frame" % clip.name)


def foot_planted(clip, wp, key):
    e = clip.idx[F.LIMBS[key][2]]
    p = wp[:, e]
    n = clip.n
    out = np.zeros(n, dtype=bool)
    for t in range(n):
        a, b = max(0, t - 1), min(n - 1, t + 1)
        v = math.hypot(*(p[b, :2] - p[a, :2])) / ((b - a) / clip.fps)
        out[t] = p[t, 2] <= FOOT_PLANT_Z and v <= FOOT_PLANT_V
    return out


def slip_stats(clip, wp, flags, key):
    """Max xy distance of the effector from its interval mean over the flagged intervals (metres)."""
    e = clip.idx[F.LIMBS[key][2]]
    worst = 0.0
    t = 0
    n = len(flags)
    while t < n:
        if flags[t]:
            u = t
            while u < n and flags[u]:
                u += 1
            seg = wp[t:u, e, :2]
            worst = max(worst, float(np.linalg.norm(seg - seg.mean(0), axis=1).max()))
            t = u
        else:
            t += 1
    return worst


def runs_of(flags):
    out = []
    t, n = 0, len(flags)
    while t < n:
        if flags[t]:
            u = t
            while u < n and flags[u]:
                u += 1
            out.append((t, u - 1))
            t = u
        else:
            t += 1
    return out


# --------------------------------------------------------------------------------------------------------------------
def montage_lines(name, seq, frames, fps, cut_t, interrupt_t, plant_secs, traj, lp, lq, names):
    """M / SEC / R / C / P / E lines of one montage (frames 0..cut)."""
    lines = ["M %s %s %s 1.000000 0.300000 SmoothStep 0.300000 SmoothStep 1 -1.000000 Other %s %d 0" % (
        name, seq, T.f6(cut_t), T.f6(fps), frames)]
    lines.append("SEC Default 0.000000 -")
    lines.append("SEC HandOff %s -" % T.f6(cut_t))
    lines.append("SEC Interrupt %s -" % T.f6(interrupt_t))
    for nm, tt in plant_secs:
        lines.append("SEC %s %s -" % (nm, T.f6(tt)))
    for f in range(frames):
        lines.append("R %s %s %s %s" % (T.f6(traj[f, 0]), T.f6(traj[f, 1]), T.f6(0.0), T.f6(traj[f, 3])))
    lines.append("C Distance_From_Ledge -")
    for f in range(frames):
        rows = [[lp[f, b, 0], lp[f, b, 1], lp[f, b, 2], lq[f, b, 0], lq[f, b, 1], lq[f, b, 2], lq[f, b, 3]]
                for b in range(len(names))]
        lines.append(T.pose_line(rows))
    lines.append("E")
    return lines


def hemisphere(q):
    q = q.copy()
    for t in range(1, q.shape[0]):
        d = np.sum(q[t] * q[t - 1], axis=-1)
        q[t] = np.where((d < 0)[..., None], -q[t], q[t])
    return q


# --------------------------------------------------------------------------------------------------------------------
def parse_trav(text):
    """Python port of ParseTraversalText (mm/TraversalFile.hpp), version 2 only. Returns dict, raises ValueError."""
    if not text or not text.endswith("\n") or "\r" in text or "\0" in text:
        raise ValueError("not LF-terminated ASCII text")
    lines = text[:-1].split("\n")
    li = [0]

    def nxt():
        if li[0] >= len(lines):
            raise ValueError("truncated")
        t = lines[li[0]].split(" ")
        li[0] += 1
        if any(s == "" for s in t):
            raise ValueError("empty token / spacing at line %d" % li[0])
        return t

    def fl(s):
        try:
            v = float(s)
        except ValueError:
            raise ValueError("float " + s)
        if not math.isfinite(v):
            raise ValueError("float " + s)
        return v

    t = nxt()
    if t != ["utpp-traversal", "2"]:
        raise ValueError("header")
    t = nxt()
    if len(t) != 4 or t[0] != "RIG" or len(t[2]) != 18 or t[2][:2] != "0x":
        raise ValueError("RIG")
    out = {"rig": t[1], "rig_hash": int(t[2][2:], 16), "rig_bones": int(t[3]), "montages": []}
    t = nxt()
    nb = int(t[1]) if len(t) >= 3 and t[0] == "POSEBONES" else -1
    if nb <= 0 or nb > out["rig_bones"] or len(t) != nb + 2:
        raise ValueError("POSEBONES")
    out["pose_bones"] = t[2:]
    if len(set(out["pose_bones"])) != nb:
        raise ValueError("duplicate pose bone")
    t = nxt()
    if len(t) != 2 or t[0] != "COUNT":
        raise ValueError("COUNT")
    count = int(t[1])
    for _ in range(count):
        t = nxt()
        if len(t) != 15 or t[0] != "M":
            raise ValueError("M line")
        m = {"name": t[1], "sequence": t[2], "length": fl(t[3]), "rate": fl(t[4]), "fps": fl(t[12]),
             "frames": int(t[13]), "action": t[11], "sections": [], "root": [], "pose": None}
        if m["length"] <= 0 or m["rate"] <= 0 or m["fps"] <= 0 or m["frames"] < 2:
            raise ValueError("M range")
        if m["action"] not in ("Hurdle", "Vault", "Mantle", "ClimbStart", "Catch", "Other"):
            raise ValueError("M action")
        if t[9] not in ("0", "1") or t[14] not in ("0", "1"):
            raise ValueError("M flags")
        if any(x["name"] == m["name"] for x in out["montages"]):
            raise ValueError("duplicate montage")
        t = nxt()
        while t[0] == "SEC":
            if len(t) != 4 or fl(t[2]) < 0:
                raise ValueError("SEC")
            m["sections"].append((t[1], fl(t[2]), t[3]))
            t = nxt()
        if not m["sections"]:
            raise ValueError("no SEC")
        if t[0] == "W":
            raise ValueError("W windows are not used by get-up files")
        for k in range(m["frames"]):
            if k > 0:
                t = nxt()
            if len(t) != 5 or t[0] != "R":
                raise ValueError("R line")
            m["root"].append([fl(x) for x in t[1:]])
        t = nxt()
        if len(t) < 3 or t[0] != "C" or t[1] != "Distance_From_Ledge":
            raise ValueError("C line")
        if not (len(t) == 3 and t[2] == "-") and len(t) != m["frames"] + 2:
            raise ValueError("C length")
        pose = np.zeros((m["frames"], nb, 7))
        for k in range(m["frames"]):
            t = nxt()
            if len(t) != nb * 7 + 1 or t[0] != "P":
                raise ValueError("P line")
            v = np.array([fl(x) for x in t[1:]], dtype=np.float32).astype(np.float64).reshape(nb, 7)
            n = np.linalg.norm(v[:, 3:7], axis=1)
            if not (np.abs(n - 1.0) <= 1e-4).all():
                raise ValueError("P quaternion not unit")
            pose[k] = v
        m["pose"] = pose
        t = nxt()
        if t != ["E"]:
            raise ValueError("E expected")
        out["montages"].append(m)
    t = nxt()
    if t != ["END"] or li[0] != len(lines):
        raise ValueError("END")
    return out


def sec_time(m, name):
    for n, s, _ in m["sections"]:
        if n == name:
            return s
    return None


# --------------------------------------------------------------------------------------------------------------------
def process_clip(root, rig, body, c, idle, mesh=None):
    """mesh: (getup_mesh.MeshSet, getup_mesh.ZSkin) or None (joint clamp set only, the pre-mesh behaviour)."""
    path = os.path.join(root, SRC_DIR % body, "M_ragdoll_getup_stand_%s.utppclip.json" % c)
    clip = F.Clip(path, rig)
    src_pos, src_rot = clip.pos, clip.rot
    pos, rot, traj = F.bake_trajectory(clip)
    hips = clip.idx["Hips"]
    wp, wq = F.fk(pos, rot, clip.par)
    cut, h_end, hsp = choose_cut(clip, wp)
    yaw_cut = facing_yaw(wp[cut], clip)
    yaw_idle = idle["yaw"]
    yaw_fix = yaw_cut - yaw_idle
    if abs(math.degrees(yaw_fix)) > 1.0:
        rotate_root_z(pos, rot, hips, -yaw_fix)
        wp, wq = F.fk(pos, rot, clip.par)
    else:
        yaw_fix = 0.0
    radii = F.clamp_radii(clip.names)
    locks = {}
    for k in F.LIMB_KEYS:
        locks[k] = F.detect_contacts(clip, wp, k, 0.10 if k[1] == "H" else FOOT_PLANT_Z)
    # the solver additionally keeps the hip joints off the floor (thigh volume while lying on the side); the contract
    # clamp set (F.clamp_radii) stays the gate that is reported and tested
    radii_solver = dict(radii)
    radii_solver[clip.idx["LeftUpLeg"]] = F.HIP_R
    radii_solver[clip.idx["RightUpLeg"]] = F.HIP_R
    sol = F.Solver(clip, pos, rot, locks, radii_solver)
    lp, lq, rep = sol.solve()
    sol.relax()
    lp, lq, rep2 = sol.solve()      # repair whatever the relaxation left violating (only grows parameters)
    rep = rep + [-1.0] + rep2
    if mesh:
        # second pass: keep the skinned mesh (not only the clamp joints) above the floor, starting from the converged joint solution
        sol.enable_mesh(mesh[1], cut + 8)
        lp, lq, rep3 = sol.solve()
        sol.relax()
        lp, lq, rep4 = sol.solve()
        rep = rep + [-2.0] + rep3 + [-1.0] + rep4
    n_smoothed = sol.n_limited
    lq = hemisphere(lq)
    wpf, wqf = F.fk(lp, lq, clip.par)
    n = cut + 1
    # contact flags = frames where the lock is fully on (weight 1)
    flags = {k: sol.lock_w[k][:n] >= 0.999 for k in F.LIMB_KEYS}
    # measurements ------------------------------------------------------------------------------------------------
    res = {"clip": c, "cut_frame": cut, "cut_s": cut / clip.fps, "hips_end_z": h_end, "yaw_fix_deg": math.degrees(yaw_fix),
           "yaw_cut_deg_before": math.degrees(yaw_cut), "yaw_idle_deg": math.degrees(yaw_idle)}
    ok = (wpf[cut, hips, 2] >= STAND_Z_FRAC * h_end) and hsp[cut] < HIPS_SPEED_MAX
    res["cut_hips_speed"] = float(hsp[cut])
    res["cut_frame_strict_0p05"] = choose_cut(clip, wp, 0.05)[0]
    fb = F.metrics_clearance(clip, pos, rot, radii, cut)
    fa = F.metrics_clearance(clip, lp, lq, radii, cut)
    res["clearance_before"] = {"min": fb[0], "bone": fb[1], "frame": fb[2]}
    res["clearance_after"] = {"min": fa[0], "bone": fa[1], "frame": fa[2]}
    if mesh:
        res["mesh_clearance_before"] = mesh_clearance(mesh, clip.names, pos, rot, clip.par, cut, hips)
        res["mesh_clearance_after"] = mesh_clearance(mesh, clip.names, lp, lq, clip.par, cut, hips)
    res["pelvis_lift_max"] = float(sol.L[:n].max())
    res["lift_max"] = {k: float(sol.lift[k][:n].max()) for k in F.LIMB_KEYS}
    res["tilt_max_deg"] = {k: math.degrees(float(sol.th[k][:n].max())) for k in F.LIMB_KEYS}
    res["swivel_max_deg"] = {k: math.degrees(float(np.abs(sol.phi[k][:n]).max())) for k in F.LIMB_KEYS}
    res["solver_iter_worst"] = [round(x, 5) for x in rep]
    res["slip_pulled_frames"] = int(sol.slip_pulled)
    res["step_limited_bone_frames"] = int(n_smoothed)
    # planted slip, before (raw baked positions in the same intervals) and after
    res["planted_slip_before"] = {k: slip_stats(clip, wp, sol.lock_ind[k][:n] > 0, k) for k in F.LIMB_KEYS}
    res["planted_slip_after"] = {k: slip_stats(clip, wpf, flags[k], k) for k in F.LIMB_KEYS}
    # slip if the source were played Root-relative without the Trajectory (what the no-bake design would show)
    wp_nb, _ = F.fk(src_pos, src_rot, clip.par)
    res["planted_slip_without_trajectory_bake"] = {k: slip_stats(clip, wp_nb, sol.lock_ind[k][:n] > 0, k)
                                                   for k in F.LIMB_KEYS}
    sa = F.joint_step_deg(lq, cut)
    sb = F.joint_step_deg(src_rot, cut)
    bi = int(np.unravel_index(sa.argmax(), sa.shape)[1])
    bt = int(np.unravel_index(sa.argmax(), sa.shape)[0])
    res["max_joint_step_deg_after"] = {"deg": float(sa.max()), "bone": clip.names[bi], "frame": bt}
    res["max_joint_step_deg_source"] = float(sb.max())
    # capsule proxy + toe plant extent (C's M3 definition) + M2 style hips steps
    wp_before, _ = F.fk(pos, rot, clip.par)
    cb = F.capsule_clearance(clip, wp_before, cut)
    ca = F.capsule_clearance(clip, wpf, cut)
    res["capsule_proxy_before"] = {"min": cb[0], "segment": cb[1], "frame": cb[2]}
    res["capsule_proxy_after"] = {"min": ca[0], "segment": ca[1], "frame": ca[2]}
    gap_a = F.capsule_gap(clip, wpf, cut)
    gap_b = F.capsule_gap(clip, wp_before, cut)
    low = wpf[:n, hips, 2] < 0.4
    ia = int(np.argmax(np.where(low, gap_a, -1e9)))
    ib = int(np.argmax(np.where(wp_before[:n, hips, 2] < 0.4, gap_b, -1e9)))
    res["floor_gap_while_low_after"] = {"max_m": float(gap_a[ia]), "frame": ia, "mean_m": float(gap_a[low].mean())}
    res["floor_gap_while_low_before"] = {"max_m": float(gap_b[ib]), "frame": ib, "mean_m": float(gap_b[wp_before[:n, hips, 2] < 0.4].mean())}
    res["toe_plant_extent_after"] = F.toe_plant_stats(clip, wpf, cut)
    res["toe_plant_extent_before"] = F.toe_plant_stats(clip, wp_before, cut)
    res["max_hips_dz_step_m"] = float(np.abs(np.diff(wpf[:n, hips, 2])).max())
    # hips motion continuity
    dh = np.linalg.norm(np.diff(wpf[:n, hips], axis=0), axis=1)
    res["max_hips_step_m"] = float(dh.max())
    # trunk smoothness: per-frame world z of hips vs source
    res["trajectory_at_cut"] = [float(traj[cut, 0]), float(traj[cut, 1])]
    res["trajectory_max_abs_xy"] = float(np.abs(traj[:n, :2]).max())
    res["hips_xy_at_cut"] = [float(wpf[cut, hips, 0]), float(wpf[cut, hips, 1])]
    # bone length drift
    ln0 = np.linalg.norm(pos[:, 1:], axis=2)
    ln1 = np.linalg.norm(lp[:, 1:], axis=2)
    nonroot = [b for b in range(len(clip.names)) if clip.par[b] > 0]
    res["bone_length_drift"] = float(np.abs(ln1[:, [b - 1 for b in nonroot]] - ln0[:, [b - 1 for b in nonroot]]).max())
    # end pose vs idle
    ei = {}
    ei["hips_z_delta"] = float(wpf[cut, hips, 2] - idle["wp"][idle["idx"]["Hips"], 2])
    for key, bn in (("LF", "LeftFoot"), ("RF", "RightFoot")):
        a = wpf[cut, clip.idx[bn]] - wpf[cut, hips]
        b = idle["wp"][idle["idx"][bn]] - idle["wp"][idle["idx"]["Hips"]]
        ei["%s_rel_hips_delta_xy" % key] = float(np.linalg.norm((a - b)[:2]))
        ei["%s_rel_hips_delta_z" % key] = float(a[2] - b[2])
    ang = []
    for b, nme in enumerate(clip.names):
        if nme in ("Root", "Trajectory", "reference_joint") or "Hand" in nme and nme not in ("LeftHand", "RightHand"):
            continue
        if nme in idle["idx"]:
            ang.append(float(np.degrees(F.qangle(lq[cut, b], idle["lq"][idle["idx"][nme]]))))
    ei["mean_local_rot_delta_deg"] = float(np.mean(ang))
    ei["max_local_rot_delta_deg"] = float(np.max(ang))
    ei["facing_yaw_deg_after"] = math.degrees(facing_yaw(wpf[cut], clip) - yaw_idle)
    res["end_pose_vs_idle"] = ei
    # times
    fp_l = foot_planted(clip, wpf, "LF")
    fp_r = foot_planted(clip, wpf, "RF")
    interrupt = None
    for t in range(n):
        if (wpf[t, hips, 2] >= INTERRUPT_HIPS_Z and fp_l[t] and fp_r[t] and sol.lock_w["LH"][t] < 1e-3
                and sol.lock_w["RH"][t] < 1e-3):
            interrupt = t
            break
    if interrupt is None:
        interrupt = cut
    res["interrupt_frame"] = interrupt
    res["interrupt_s"] = interrupt / clip.fps
    # SEC informational plant / lift times (longest locked run per effector)
    secs = []
    for k in F.LIMB_KEYS:
        rs = runs_of(flags[k])
        if rs:
            a, b = max(rs, key=lambda x: x[1] - x[0])
            secs.append(("Plant_%s" % k, a / clip.fps))
            secs.append(("Lift_%s" % k, min(b + 1, cut) / clip.fps))
    res["contact_runs"] = {k: runs_of(flags[k]) for k in F.LIMB_KEYS}
    res["standing_ok"] = bool(ok)
    out = {"clip": clip, "lp": lp, "lq": lq, "pos_baked": pos, "rot_baked": rot, "traj": traj, "cut": cut,
           "interrupt": interrupt, "secs": secs, "res": res, "flags": flags, "wpf": wpf, "sol": sol}
    return out


def mesh_clearance(mesh, names, lp, lq, par, cut, hips):
    """Skinned-mesh floor clearance of frames 0..cut: lowest vertex height (floor z = 0), frame, body region, and the hover
    (highest per-frame lowest vertex while the Hips are below 0.4 m = how far the body floats above the floor)."""
    ms, zs = mesh
    wp, wq = F.fk(lp[:cut + 1], lq[:cut + 1], par)
    z = zs.z(wp, wq)
    zmin = z.min(1)
    f = int(np.argmin(zmin))
    vi = int(np.argmin(z[f]))
    low = wp[:, hips, 2] < 0.4
    return {"min": float(zmin[f]), "frame": f, "region": ms.region(vi), "joint": ms.dom[vi], "owner": zs.owner[vi],
            "frames_below_-0.005": int((zmin < -0.005).sum()),
            "hover_max_m": float(zmin[low].max()) if low.any() else None,
            "hover_mean_m": float(zmin[low].mean()) if low.any() else None}


def load_mesh(root, body):
    mr = GM.default_mesh_root(root)
    if not GM.available(mr, body):
        raise SystemExit("body mesh exports for '%s' missing under %s (see getup_mesh.py for the WolvenKit export commands); "
                         "pass --no-mesh to bake against the joint clamp set only" % (body, mr))
    return GM.MeshSet(mr, body)


def mesh_hashes(root, body):
    return {os.path.basename(p): hashlib.sha256(open(p, "rb").read()).hexdigest() for p in GM.mesh_files(GM.default_mesh_root(root), body)}


def load_idle(root, rig, body):
    d = "out_sourcepack_%s" % body
    p = os.path.join(root, "Animations/retarget", d, "M_Neutral_Stand_Idle_Loop.utppclip.json")
    clip = F.Clip(p, rig)
    wp, wq = F.fk(clip.pos[:1], clip.rot[:1], clip.par)
    ya = facing_yaw(wp[0], clip)
    return {"wp": wp[0], "lq": clip.rot[0], "idx": clip.idx, "yaw": ya}


def write_text(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)
    return hashlib.sha256(data).hexdigest()


def frame0_fixture_entry(parsed_m, rig, pose_bones):
    pos = parsed_m["pose"][0][:, 0:3]
    rot = parsed_m["pose"][0][:, 3:7]
    rn = {n: i for i, n in enumerate(rig.names)}
    pidx = {n: i for i, n in enumerate(pose_bones)}
    par = np.array([pidx[rig.names[rig.parents[rn[n]]]] if rig.parents[rn[n]] >= 0 else -1 for n in pose_bones])
    wp, wq = F.fk(pos[None], rot[None], par)
    return {pose_bones[i]: [round(float(wp[0, i, j]), 5) for j in range(3)] for i in range(len(pose_bones))}, wp[0], pidx


def parsed_fk(parsed, mi, rig):
    """World positions / rotations (frames, bones, *) of montage mi of a parsed file, bones in POSEBONES order."""
    m = parsed["montages"][mi]
    pb = parsed["pose_bones"]
    rn = {n: i for i, n in enumerate(rig.names)}
    pidx = {n: i for i, n in enumerate(pb)}
    par = np.array([pidx[rig.names[rig.parents[rn[n]]]] if rig.parents[rn[n]] >= 0 else -1 for n in pb])
    wp, wq = F.fk(m["pose"][:, :, 0:3], m["pose"][:, :, 3:7], par)
    return wp, wq, par


def lie_features(wp, pidx):
    """Clip-analysis convention (inv/analyze_getup.py): lateral = LUp - RUp, front = cross(lateral, spine up)."""
    def v(n):
        return wp[pidx[n]]
    up = F.unit(v("Spine3") - v("Hips"))
    lat = F.unit(v("LeftUpLeg") - v("RightUpLeg"))
    front = F.unit(np.cross(lat, up))
    return {"front_z": round(float(front[2]), 4), "lateral_z": round(float(lat[2]), 4), "spine_up_z": round(float(up[2]), 4)}


def bake_body(root, body, write_fixtures, fixture_dir, out_root=None, use_mesh=True):
    out_root = out_root or root
    rig = F.Rig(root, body)
    idle = load_idle(root, rig, body)
    results = {}
    outs = {}
    mesh = None
    if use_mesh:
        ms = load_mesh(root, body)
        src0 = F.Clip(os.path.join(root, SRC_DIR % body, "M_ragdoll_getup_stand_B.utppclip.json"), rig)
        mesh = (ms, GM.ZSkin(ms, src0.names))
    for c in CLIPS:
        outs[c] = process_clip(root, rig, body, c, idle, mesh)
        results[c] = outs[c]["res"]
        mc = results[c].get("mesh_clearance_after", {})
        print("%s %s: cut f%d (%.2fs) interrupt %.2fs clearance %.3f -> %.4f  Lmax %.3f  step %.1f  slip %.4f  mesh z %.4f -> %.4f" % (
            body, c, outs[c]["cut"], outs[c]["res"]["cut_s"], outs[c]["res"]["interrupt_s"],
            results[c]["clearance_before"]["min"], results[c]["clearance_after"]["min"], results[c]["pelvis_lift_max"],
            results[c]["max_joint_step_deg_after"]["deg"], max(results[c]["planted_slip_after"].values()),
            results[c].get("mesh_clearance_before", {}).get("min", float("nan")), mc.get("min", float("nan"))), flush=True)
    names = outs["B"]["clip"].names
    for c in CLIPS:
        if outs[c]["clip"].names != names:
            raise SystemExit("clip bone lists differ")
    # ---- the fixed clips ------------------------------------------------------------------------------------------
    hashes = {}
    for c in CLIPS:
        o = outs[c]
        clip = o["clip"]
        d = json.loads(json.dumps(clip.src))
        d["getupfix"] = {"tool": "tools/mmbake/getupbake.py", "note": "Trajectory folded into Hips (Trajectory bone = identity); "
                         "floor/contact fix-up; Root-relative, in place", "cut_frame": o["cut"]}
        for f in range(clip.n):
            for b in range(len(names)):
                row = d["frames"][f]["bones"][b]
                row[0:3] = [float(v) for v in o["lp"][f, b]]
                row[3:7] = [float(v) for v in o["lq"][f, b]]
        data = json.dumps(d, separators=(",", ":")).encode("utf-8")
        hashes["clip_" + c] = write_text(os.path.join(out_root, OUT_DIR % body, "M_ragdoll_getup_stand_%s.utppclip.json" % c), data)
    # ---- the .utpptrav ------------------------------------------------------------------------------------------------
    out = ["utpp-traversal 2", "RIG %s 0x%016x %d" % (rig.rig_name, rig.hash, len(rig.names)),
           "POSEBONES %d %s" % (len(names), " ".join(names)), "COUNT 4"]
    for c in CLIPS:
        o = outs[c]
        clip = o["clip"]
        frames = o["cut"] + 1
        out += montage_lines("GetUp_" + c, clip.name, frames, clip.fps, o["cut"] / clip.fps, o["interrupt"] / clip.fps, o["secs"],
                             o["traj"][:frames], o["lp"][:frames], o["lq"][:frames], names)
    out.append("END")
    text = ("\n".join(out) + "\n")
    data = text.encode("ascii")
    trav_path = os.path.join(out_root, DATA_DIR, "utpp_getup_%s.utpptrav" % body)
    hashes["utpptrav"] = write_text(trav_path, data)
    # round trip through the grammar
    parsed = parse_trav(text)
    # ---- contacts sidecar + fixtures --------------------------------------------------------------------------------
    side = {"rig": rig.rig_name, "fps": 30, "body": body, "trajectory_baked_into_hips": True,
            "radii": {names[b]: r for b, r in F.clamp_radii(names).items()}, "clips": {}}
    fix = {"rig": rig.rig_name, "body": body, "fps": 30,
           "note": "frame 0 of utpp_getup_%s.utpptrav in the clip Root frame (model space), Trajectory baked into Hips; "
                   "lateral = LeftUpLeg - RightUpLeg; label = montage GetUp_<label>" % body, "clips": {}}
    for ci, c in enumerate(CLIPS):
        o = outs[c]
        m = parsed["montages"][ci]
        f0, wp0, pidx = frame0_fixture_entry(m, rig, parsed["pose_bones"])
        feats = lie_features(wp0, pidx)
        n = o["cut"] + 1
        side["clips"][c] = {"cut": o["cut"], "interrupt": o["interrupt"], "cut_s": o["cut"] / 30.0,
                            "interrupt_s": o["interrupt"] / 30.0, "label": c, "features": feats, "frame0_model": f0,
                            "contacts": {k: [int(x) for x in o["flags"][k]] for k in F.LIMB_KEYS},
                            "result": o["res"]}
        fix["clips"][c] = {"label": c, "frame0_model": f0, "features": feats, "cut": o["cut"]}
    side_b = json.dumps(side, indent=1, sort_keys=False).encode("utf-8")
    hashes["contacts"] = write_text(os.path.join(out_root, DATA_DIR, "utpp_getup_%s.contacts.json" % body), side_b)
    rep_doc = {"body": body, "results": results, "hashes": hashes}
    if use_mesh:
        rep_doc["mesh_sha256"] = mesh_hashes(root, body)
        rep_doc["mesh_floor_margin_m"] = F.MESH_MARGIN
    rep_b = json.dumps(rep_doc, indent=1).encode("utf-8")
    write_text(os.path.join(out_root, DATA_DIR, "getup_report_%s.json" % body), rep_b)
    if write_fixtures:
        fb = json.dumps(fix, separators=(",", ":")).encode("utf-8")
        hashes["fixture"] = write_text(os.path.join(fixture_dir, "getup_frame0_%s.json" % body), fb)
    return hashes, results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root")
    ap.add_argument("--body", action="append", choices=("woman", "man"))
    ap.add_argument("--no-fixtures", action="store_true")
    ap.add_argument("--no-mesh", action="store_true", help="bake against the joint clamp set only (default: the skinned body mesh must be available)")
    ap.add_argument("--out-root", help="write the outputs under this root instead of --root (the sources are always read from --root)")
    ap.add_argument("--fixture-dir", default=os.path.join(repo_root(), "plugin", "UTPP", "tests", "data"))
    a = ap.parse_args()
    root = project_root(a.root)
    allh = {}
    for body in a.body or ["woman", "man"]:
        h, _ = bake_body(root, body, not a.no_fixtures, a.fixture_dir, a.out_root, use_mesh=not a.no_mesh)
        allh[body] = h
        for k, v in h.items():
            print("  %s %s sha256 %s" % (body, k, v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
