"""travbake.py - source pack traversal montage bake.

For every traversal montage listed in Data/traversal/traversal_table.json (PSD_Traversal: Hurdle / Mantle / Vault /
Climb_Start / Catch_*) writes one text file per rig (FORMAT_TRAVERSAL.txt): montage blend settings, sections, the
MotionWarping windows (target, frames, flags, warp point provider), the root track (x y z yaw, z from the *_rtz
root_z when present), the Distance_From_Ledge curve and the CP contact bone + its offset from the root at every
window end frame. source-pack-derived output.

usage: python travbake.py [--root <project folder>] [--rig woman|man ...] [--out-dir Data/traversal]
                          [--format 1|2] [--variant base|knee] [--overwrite] [--rtz-dir DIR] [--clips-dir DIR]
  --format 1 (default): utpp_traversal_<rig>.utpptrav, FORMAT_TRAVERSAL.txt version 1 (unchanged).
  --format 2: utpp_traversal_<rig>_motion.utpptrav (base) / utpp_traversal_<rig>_motion_knee.utpptrav (knee), version 2 = version 1 + POSEBONES line + per montage the local pose of
              every montage frame (P lines, trav: v2 plays montages whose sequence is not a clip of the v2
              database from these poses). An existing output file is never overwritten without --overwrite.
  --variant knee: clips from out_sourcepack_<rig>_knee / out_sourcepack_<rig>_rtz_knee (the retarget of the staged
              utpp_motion_<rig>_knee.utppmm); base (default) = out_sourcepack_<rig> / _rtz.
"""
import argparse
import hashlib
import json
import math
import os
import re
import struct
import sys

FORMAT_VERSION = 1
RIGS = {
    "woman": ("Extracted/game/json/vanilla/player_woman_skeleton.rig.json", "Animations/retarget/out_sourcepack_woman",
              "Animations/retarget/out_sourcepack_woman_rtz"),
    "man": ("Extracted/game/json/vanilla-man/player_man_skeleton.rig.json", "Animations/retarget/out_sourcepack_man",
            "Animations/retarget/out_sourcepack_man_rtz"),
}
VARIANT_SUFFIX = {"base": "", "knee": "_knee"}  # appended to the clip / rtz dirs of RIGS
HANDS = ("LeftHand", "RightHand")
BLEND_OPT = {"LINEAR": "Linear", "CUBIC": "Cubic", "HERMITE_CUBIC": "HermiteCubic", "SINUSOIDAL": "Sinusoidal",
             "QUADRATIC_IN_OUT": "QuadraticInOut", "CUBIC_IN_OUT": "CubicInOut", "QUARTIC_IN_OUT": "QuarticInOut",
             "QUINTIC_IN_OUT": "QuinticInOut", "CIRCULAR_IN": "CircularIn", "CIRCULAR_OUT": "CircularOut",
             "CIRCULAR_IN_OUT": "CircularInOut", "EXP_IN": "ExpIn", "EXP_OUT": "ExpOut", "EXP_IN_OUT": "ExpInOut"}


def load_json(p):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def f6(x):
    s = "%.6f" % x
    return "0.000000" if s == "-0.000000" else s


def tok(s):
    return s if s and not re.search(r"\s", s) else "-"


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def qyaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rig_info(rig_path):
    rc = load_json(rig_path)["Data"]["RootChunk"]
    names = [b["$value"] if isinstance(b, dict) else b for b in rc["boneNames"]]
    parents = [int(p) for p in rc["boneParentIndexes"]]
    buf = bytearray()
    for n, p, t in zip(names, parents, rc["boneTransforms"]):
        tr = t["Translation"]
        q = [int(math.floor(float(struct.unpack("<f", struct.pack("<f", tr[a]))[0]) * 1000.0 + 0.5)) for a in "XYZ"]
        buf += n.encode("utf-8") + b"\x00" + struct.pack("<iiii", p, *q)
    h = 0xcbf29ce484222325
    for c in bytes(buf):
        h = ((h ^ c) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return names, parents, h


def fk(frame_bones, clip_names, rig_names, rig_parents, want):
    """Model-space positions of the bones in `want` for one frame (clip bones by name, local to rig parents)."""
    idx = {n: i for i, n in enumerate(clip_names)}
    rig_idx = {n: i for i, n in enumerate(rig_names)}
    cache = {}

    def glob(name):
        if name in cache:
            return cache[name]
        b = frame_bones[idx[name]]
        p, q = (b[0], b[1], b[2]), (b[3], b[4], b[5], b[6])
        par = rig_parents[rig_idx[name]]
        if par < 0:
            r = (p, q)
        else:
            pp, pq = glob(rig_names[par])
            rp = qrot(pq, p)
            r = ((pp[0] + rp[0], pp[1] + rp[1], pp[2] + rp[2]), qmul(pq, q))
        cache[name] = r
        return r

    return {n: glob(n)[0] for n in want}


def traversal_montages(table_path):
    txt = open(table_path, "r", encoding="utf-8").read()
    return sorted(set(re.findall(r'"(AM_[A-Za-z0-9_]*Traversal[A-Za-z0-9_]*)"', txt)))


def action_of(name):
    for key, act in (("_Catch_", "Catch"), ("_Climb_Start", "ClimbStart"), ("_Hurdle_", "Hurdle"),
                     ("_Vault_", "Vault"), ("_Mantle_", "Mantle")):
        if key in name:
            return act
    return "Other"


def pose_line(bones):
    """P line (format 2): local pose of one frame, 7 floats per clip bone (x y z qx qy qz qw), quaternion normalised
    in float64 as mmbake_motion.ClipData (POSE section = the same values as float32)."""
    out = ["P"]
    for b in bones:
        q = (float(b[3]), float(b[4]), float(b[5]), float(b[6]))
        n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if not (abs(n - 1.0) <= 1e-3):
            raise SystemExit("quaternion norm %.6f" % n)
        out += [f6(float(b[0])), f6(float(b[1])), f6(float(b[2]))] + [f6(c / n) for c in q]
    return " ".join(out)


def bake_montage(root, mname, rig_names, rig_parents, out_dir_clips, rtz_dir, poses=False):
    """poses=True (format 2): returns the clip bone names as the third value and P lines before E."""
    meta = load_json(os.path.join(root, "Animations/sourcepack/als_export/meta", mname + ".json"))
    seq = meta["referenced_sequences"][0]
    cp = os.path.join(root, out_dir_clips, seq + ".utppclip.json")
    rp = os.path.join(root, rtz_dir, seq + ".utppclip.json")
    rtz = os.path.isfile(rp)
    clip = load_json(rp if rtz else cp)
    fps = float(clip["fps"])
    n = int(clip["num_frames"])
    names = clip["bone_names"]
    ti = names.index("Trajectory")
    root_z = clip.get("root_z") if rtz else None
    if root_z is not None and len(root_z) != n:
        raise SystemExit("%s: root_z length %d != %d" % (seq, len(root_z), n))
    segs = meta["slot_anim_tracks"][0]["anim_track"]["anim_segments"]
    if len(meta["slot_anim_tracks"]) != 1 or len(segs) != 1 or abs(segs[0]["start_pos"]) > 1e-6 or             abs(segs[0]["anim_play_rate"] - 1.0) > 1e-6:
        raise SystemExit("%s: only one slot with one segment at start_pos 0, rate 1 is supported" % mname)
    f0 = int(math.floor(segs[0]["anim_start_time"] * fps + 0.5))
    f1 = min(n - 1, int(math.floor(segs[0]["anim_end_time"] * fps + 0.5)))
    roots, hands = [], []
    for f, fr in enumerate(clip["frames"]):
        if f < f0 or f > f1:
            continue
        b = fr["bones"][ti]
        z = float(root_z[f]) if root_z is not None else float(b[2])
        roots.append((float(b[0]), float(b[1]), z, qyaw((b[3], b[4], b[5], b[6]))))
        hands.append(fk(fr["bones"], names, rig_names, rig_parents, HANDS))
    dfl = clip.get("curves", {}).get("distance_from_ledge")
    dfl = dfl[f0:f1 + 1] if dfl is not None and len(dfl) == n else None
    n = f1 - f0 + 1
    lines = []
    bi, bo = meta["blend_in"], meta["blend_out"]
    lines.append("M %s %s %s %s %s %s %s %s %d %s %s %s %d %d" % (
        mname, seq, f6(meta["length_s"]), f6(meta.get("rate_scale", 1.0)), f6(bi["blend_time"]),
        BLEND_OPT.get(bi["blend_option"], bi["blend_option"]), f6(bo["blend_time"]),
        BLEND_OPT.get(bo["blend_option"], bo["blend_option"]), 1 if meta.get("enable_auto_blend_out", True) else 0,
        f6(meta.get("blend_out_trigger_time", -1.0)), action_of(mname), f6(fps), n, 1 if rtz else 0))
    secs = meta.get("composite_sections") or []
    if secs:
        for s in secs:
            lines.append("SEC %s %s %s" % (tok(s.get("section_name")), f6(s.get("start_time", 0.0)),
                                           tok(s.get("next_section_name") or "-")))
    else:
        lines.append("SEC Default 0.000000 -")
    wins = [x for x in meta["notifies"] if x.get("class") == "AnimNotifyState_MotionWarping"]
    wins.sort(key=lambda x: x["trigger_time"])
    for w in wins:
        m = w["props"]["root_motion_modifier"]
        sf = min(n - 1, int(math.floor((segs[0]["anim_start_time"] + w["trigger_time"]) * fps + 0.5)) - f0)
        ef = min(n - 1, int(math.floor((segs[0]["anim_start_time"] + w["end_time"]) * fps + 0.5)) - f0)
        tgt = m.get("warp_target_name") or "-"
        rx, ry, rz, ryaw = roots[ef]
        if tgt == "BackFloor":
            bone, off = "Root", (0.0, 0.0, 0.0)
        else:
            # The pose is in place relative to the full root (Trajectory x y yaw + root_z, RTZ.txt), so the FK hand
            # (Root = identity) is already hand - root in root space. Planted hand: smallest world displacement
            # (root + RotZ(yaw) * FK) over the frames around the window end.
            def world(f, h):
                p = hands[f][h]
                cs, sn = math.cos(roots[f][3]), math.sin(roots[f][3])
                return (roots[f][0] + cs * p[0] - sn * p[1], roots[f][1] + sn * p[0] + cs * p[1], roots[f][2] + p[2])
            a, c = max(0, ef - 1), min(n - 1, ef + 1)
            best = None
            for h in HANDS:
                d = math.dist(world(a, h), world(c, h))
                if best is None or d < best[0] - 1e-9:
                    best = (d, h)
            bone = best[1]
            off = tuple(hands[ef][bone])
        lines.append("W %s %s %s %d %d %s %d %d %d %d %s %s %s %s %s %s %s %s %s" % (
            tok(tgt), f6(w["trigger_time"]), f6(w["end_time"]), sf, ef, tok(m.get("_class")),
            1 if m.get("warp_translation") else 0, 1 if m.get("warp_rotation") else 0,
            1 if m.get("ignore_z_axis") else 0, 1 if m.get("warp_to_feet_location") else 0,
            tok(m.get("rotation_type")), f6(m.get("warp_rotation_time_multiplier", 1.0)),
            tok(m.get("warp_point_anim_provider")), tok(m.get("warp_point_anim_bone_name") or "-"), bone,
            f6(off[0]), f6(off[1]), f6(off[2]), f6(m.get("max_speed_clamp_ratio", 0.0))))
    for r in roots:
        lines.append("R %s %s %s %s" % (f6(r[0]), f6(r[1]), f6(r[2]), f6(r[3])))
    if dfl is not None:
        lines.append("C Distance_From_Ledge " + " ".join(f6(float(v)) for v in dfl))
    else:
        lines.append("C Distance_From_Ledge -")
    if poses:
        for f, fr in enumerate(clip["frames"]):
            if f0 <= f <= f1:
                if len(fr["bones"]) != len(names):
                    raise SystemExit("%s frame %d: %d bones != %d" % (seq, f, len(fr["bones"]), len(names)))
                lines.append(pose_line(fr["bones"]))
    lines.append("E")
    if poses:
        return lines, len(wins), list(names)
    return lines, len(wins)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                      "..", "..", "..")))
    ap.add_argument("--rig", action="append", choices=sorted(RIGS))
    ap.add_argument("--out-dir", default="Data/traversal")
    ap.add_argument("--format", type=int, choices=(1, 2), default=FORMAT_VERSION)
    ap.add_argument("--variant", choices=sorted(VARIANT_SUFFIX), default="base")
    ap.add_argument("--overwrite", action="store_true", help="format 2: replace an existing output file")
    ap.add_argument("--rtz-dir", help="override the *_rtz clip dir (absolute or relative to --root), e.g. the twistfixed "
                                      "out_sourcepack_<rig>_rtz_knee_twistfix; applies to every --rig given")
    ap.add_argument("--clips-dir", help="override the plain clip dir (fallback for sequences without an rtz clip)")
    a = ap.parse_args()
    root = a.root
    mons = traversal_montages(os.path.join(root, "Data/traversal/traversal_table.json"))
    if not mons:
        raise SystemExit("no traversal montages in the traversal table")
    poses = a.format == 2
    for rig in a.rig or ["woman", "man"]:
        rig_json, cdir, rdir = RIGS[rig]
        cdir, rdir = cdir + VARIANT_SUFFIX[a.variant], rdir + VARIANT_SUFFIX[a.variant]
        cdir, rdir = a.clips_dir or cdir, a.rtz_dir or rdir
        rnames, rpar, rhash = rig_info(os.path.join(root, rig_json))
        # format 2 names follow the v2 databases: _motion = base clips (utpp_motion_<rig>.utppmm), _motion_knee = knee clips
        op = os.path.join(root, a.out_dir, "utpp_traversal_%s%s.utpptrav" % (
            rig, ("_motion" + VARIANT_SUFFIX[a.variant]) if poses else ""))
        if poses and os.path.exists(op) and not a.overwrite:
            raise SystemExit("%s exists (format 2 never overwrites; --overwrite to replace)" % op)
        body = []
        nw = 0
        pose_bones = None
        for m in mons:
            r = bake_montage(root, m, rnames, rpar, cdir, rdir, poses)
            body += r[0]
            nw += r[1]
            if poses:
                if pose_bones is None:
                    pose_bones = r[2]
                elif r[2] != pose_bones:
                    raise SystemExit("%s: clip bone list differs from the first montage's" % m)
        out = ["utpp-traversal %d" % a.format,
               "RIG %s 0x%016x %d" % (os.path.basename(rig_json).split(".")[0], rhash, len(rnames))]
        if poses:
            if any(b not in rnames for b in pose_bones):
                raise SystemExit("pose bone not in the rig")
            out.append("POSEBONES %d %s" % (len(pose_bones), " ".join(pose_bones)))
        out.append("COUNT %d" % len(mons))
        out += body
        out.append("END")
        data = ("\n".join(out) + "\n").encode("ascii")
        with open(op, "wb") as f:
            f.write(data)
        print("%s: %d montages, %d warp windows, %d bytes, sha256 %s" % (
            op, len(mons), nw, len(data), hashlib.sha256(data).hexdigest()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
