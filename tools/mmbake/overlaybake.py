"""overlaybake.py - bake the source pack overlay layer data (.utppov v1, see FORMAT_OVERLAY.txt) for one CP rig.

Input : retargeted utppclip v1 files (Animations/retarget/out_sourcepack_<rig>), export meta (additive base refs,
        Animations/sourcepack/als_export/meta), CP rig json (rig identity hash as .utppmm v2), retarget bonemap (masks).
Output: Data/overlay/utpp_overlays_<rig>.utppov + .summary.json. Deterministic (no timestamps in the binary).

"""
import argparse
import glob
import hashlib
import json
import os
import struct
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mmbake_base as v1  # noqa: E402
import mmbake_motion as v2  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
NUM_BONES = 73
MAGIC = b"UTPPOV\x00\x00"
VERSION = 1
HEADER_SIZE = 64

RIGS = {
    "woman": dict(rig="Extracted/game/json/vanilla/player_woman_skeleton.rig.json",
                  clips="Animations/retarget/out_sourcepack_woman",
                  bonemap="Source/tools/retarget/bonemap_source_to_game_woman.json", style="Feminine"),
    "man": dict(rig="Extracted/game/json/vanilla-man/player_man_skeleton.rig.json",
                clips="Animations/retarget/out_sourcepack_man",
                bonemap="Source/tools/retarget/bonemap_source_to_game_man.json", style="Masculine"),
}
META_DIR = "Animations/sourcepack/als_export/meta"

# overlay table: (name, kind, flags). kind 0 pose overlay, 1 style (OB), 2 base poses (B/BP), 3 transitions.
# flags bit0 bCanAim (ABP: Rifle, Pistol1H, Pistol2H, Bow, Torch, Binoculars).
POSE_OVERLAYS = ["Default", "Pistol1H", "Pistol2H", "Rifle", "Injured"]
CAN_AIM = {"Pistol1H", "Pistol2H", "Rifle"}
BASE_POSES = ["Pose_Base_Stand", "Pose_Base_Crouch", "Pose_Neutral_Stand_Idle", "Pose_Neutral_Stand_Move",
              "Pose_Neutral_Crouch_Idle", "Pose_Neutral_Crouch_Move"]
TRANSITIONS = ["Pose_Transition", "Stand_Transition"]
# style delta dP = D_MS(BP -> OB) per (stance, idle/move); report 3.11
STYLE_DP = [("Stand_Idle", "Pose_Base_Stand"), ("Stand_Move", "Pose_Base_Stand"), ("Crouch", "Pose_Base_Crouch")]

GROUP = {"base": 0, "style": 1, "overlay": 2, "sweep": 3, "transition": 4}
ADD_TYPE = {"AAT_NONE": 0, "AAT_ROTATION_OFFSET_MESH_SPACE": 1, "AAT_LOCAL_SPACE_BASE": 2}
BASE_KIND = {None: 0, "anim_frame": 1, "local_anim_frame": 2}

# ALS LayeredBoneBlend regions: BranchFilters (UE bone, BlendDepth) of ABP_LayerBlending
# (Extracted/sourcepack/sourcepack/t3d, SK_UEFN_Mannequin; checked against the compiled PerBoneBlendWeights). UE semantics:
# depth 0 = whole subtree 1.0, depth k > 0 = bone at depth d below the filter bone gets min(1, (d+1)/k),
# depth -1 = subtree excluded. Multiple filters of one region combine with max.
UE_REGIONS = {
    "pelvis": [("pelvis", 0), ("thigh_l", -1), ("thigh_r", -1)],
    "spine": [("spine_01", 5)],
    "head": [("neck_01", 1)],
    "arm_l": [("clavicle_l", 1)],
    "arm_r": [("clavicle_r", 1)],
    "hand_l": [(b + "_l", 0) for b in ("index_01", "middle_01", "ring_01", "pinky_01", "thumb_01", "ik_hand")],
    "hand_r": [(b + "_r", 0) for b in ("index_01", "middle_01", "ring_01", "pinky_01", "thumb_01", "ik_hand")],
}
# UE bones covered by a BlendDepth ramp (spine_01 + 5: spine_01..spine_05 = 0.2..1.0). On the CP rig the ramp
# runs over the CP bones mapped from these UE bones (Spine..Spine3 = 4 bones -> 0.25/0.5/0.75/1.0), so the top
# spine bone keeps the full overlay like spine_05 in UE.
UE_RAMP = {"spine_01": ("spine_01", "spine_02", "spine_03", "spine_04", "spine_05")}
# UE IK target bones without a CP source mapping -> CP bone carrying the same role (weapon/hand attach bone).
UE_TO_CP_EXTRA = {"ik_hand_l": "WeaponLeft", "ik_hand_r": "WeaponRight"}
MASKS = ["pelvis", "spine", "head", "arm_l", "arm_r", "hand_l", "hand_r"]

# Deliberate curve overrides per rig, applied only in the pack step of bake() (Clip / delta / name_bytes stay
# untouched). rig -> pose -> curve -> value, written to every frame of that pose.
# WHY: in the relaxed one-handed pistol idle the female rig must keep the unarmed lower body (stance), so the
# layering_legs and layering_pelvis curves of Pose_Pistol1H_Stand_Relax_Idle are 0.0 instead of the source clip's
# 1.0 (2 frames x 2 curves = 4 float32 values). This used to
# be a separate one-off patch of the baked file; the table replaces it, so a normal bake reproduces the shipped
# file. "--no-overrides" gives the plain bake of the source clips.
CURVE_OVERRIDES = {
    "woman": {"Pose_Pistol1H_Stand_Relax_Idle": {"layering_legs": 0.0, "layering_pelvis": 0.0}},
    "man": {},
}


def rel(p):
    return os.path.join(ROOT, p.replace("/", os.sep))


def load_json(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def qinv(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qcanon(q):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return np.where(q[..., 3:4] < 0, -q, q)


def select_clips(style, clip_names):
    """Ordered list of (overlay, kind, flags, [clip names])."""
    have = set(clip_names)
    out = []

    def need(n):
        if n not in have:
            raise SystemExit("missing retargeted clip " + n)
        return n
    out.append(("BasePoses", 2, 0, [need(n) for n in BASE_POSES]))
    st = [need("Pose_%s_%s" % (style, s)) for s, _ in STYLE_DP] + [need("Poses_" + style)]
    out.append((style, 1, 0, st))
    for o in POSE_OVERLAYS:
        if o == "Default":
            out.append((o, 0, 0, []))   # ABP_OverlayPose_Default: no pose, layering = pure locomotion
            continue
        names = sorted(n for n in clip_names if n.startswith("Pose_%s_" % o))
        names += sorted(n for n in clip_names if n == "Poses_" + o or n == "Poses_%s_NoCurves" % o)
        names += sorted(n for n in clip_names if n.startswith("AO_%s_" % o) and n.endswith("_Sweep"))
        if not names:
            raise SystemExit("overlay %s: no clips" % o)
        out.append((o, 0, 1 if o in CAN_AIM else 0, names))
    out.append(("Transitions", 3, 0, [need(n) for n in TRANSITIONS]))
    return out


def build_masks(names, parents, bonemap):
    """UE_REGIONS applied to the CP rig: each UE filter bone -> its CP bone (bonemap cp <- src, UE names are
    case-insensitive; UE_TO_CP_EXTRA for unmapped IK bones); a BlendDepth ramp is re-spread over the CP bones mapped
    from the UE ramp bones (CP depth = their count)."""
    nb = len(names)
    cp_of = {}
    for e in bonemap["bones"]:
        if e.get("src") and e["cp"] in names[:nb]:
            cp_of.setdefault(e["src"].lower(), []).append(names.index(e["cp"]))

    def cp_bone(ue):
        c = cp_of.get(ue.lower(), [])
        if not c and ue in UE_TO_CP_EXTRA:
            c = [names.index(UE_TO_CP_EXTRA[ue])]
        if len(c) != 1:
            raise SystemExit("mask: UE bone %s maps to %d CP bones" % (ue, len(c)))
        return c[0]

    def depth_below(b, root):
        d = 0
        while b >= 0:
            if b == root:
                return d
            b, d = parents[b], d + 1
        return -1

    out = []
    for k in MASKS:
        w = np.zeros(nb)
        excl = np.zeros(nb, bool)
        for ue, depth in UE_REGIONS[k]:
            r = cp_bone(ue)
            if depth > 1:
                chain = [cp_bone(u) for u in UE_RAMP[ue] if u.lower() in cp_of]
                if chain[0] != r or any(parents[b] != a for a, b in zip(chain, chain[1:])):
                    raise SystemExit("mask %s: CP ramp bones %s are not a parent chain from %s"
                                     % (k, [names[b] for b in chain], names[r]))
                depth = len(chain)
            for b in range(nb):
                d = depth_below(b, r)
                if d < 0:
                    continue
                if depth < 0:
                    excl[b] = True
                else:
                    w[b] = max(w[b], 1.0 if depth == 0 else min(1.0, (d + 1.0) / depth))
        w[excl] = 0.0
        out.append(w)
    return out


class Clip:
    def __init__(self, path, names, parents):
        c = load_json(path)
        if c.get("format") != "utppclip" or c.get("version") != 1:
            raise SystemExit(path + ": not utppclip v1")
        if c["bone_names"][:NUM_BONES] != names:
            raise SystemExit(path + ": bone names differ from rig")
        self.name = c["source_clip"]
        self.rig = c["rig"]
        A = np.array([f["bones"][:NUM_BONES] for f in c["frames"]], np.float64)  # (F,B,10)
        self.P, self.Q, self.S = A[..., 0:3], A[..., 3:7], A[..., 7:10]
        self.Q = self.Q / np.linalg.norm(self.Q, axis=-1, keepdims=True)
        self.GP, self.GQ = v1.fk_model(self.P, self.Q, parents)
        self.curves = {k.lower(): np.asarray(v, np.float64) for k, v in c.get("curves", {}).items()}
        self.n = A.shape[0]
        self.sha = hashlib.sha256(open(path, "rb").read()).hexdigest()


def delta(X, Xf, A, Af):
    """Per-bone delta of frames Xf of clip X against frame Af of clip A: (n, B, 14) = dq_ms, dq_ls, dt, ds."""
    out = np.zeros((len(Xf), NUM_BONES, 14))
    for i, f in enumerate(Xf):
        out[i, :, 0:4] = qcanon(v1.qmul(X.GQ[f], qinv(A.GQ[Af])))
        out[i, :, 4:8] = qcanon(v1.qmul(X.Q[f], qinv(A.Q[Af])))
        out[i, :, 8:11] = X.P[f] - A.P[Af]
        out[i, :, 11:14] = X.S[f] / A.S[Af] - 1.0
    return out


def name_bytes(s, n):
    b = s.encode("utf-8")
    if len(b) >= n:
        raise SystemExit("name too long: " + s)
    return b + b"\x00" * (n - len(b))


def bake(rig_key, out_path, clips_dir=None, rig_path=None, style=None, overrides=True):
    cfg = RIGS[rig_key]
    ovr = CURVE_OVERRIDES.get(rig_key, {}) if overrides else {}
    applied = []
    rig_path = rig_path or rel(cfg["rig"])
    clips_dir = clips_dir or rel(cfg["clips"])
    style = style or cfg["style"]
    names, parents = v1.load_rig(rig_path, NUM_BONES)
    rest = v2.rig_rest_local(rig_path, NUM_BONES)
    rig_hash, _ = v2.rig_identity_hash(names, parents, rest)
    files = {os.path.basename(f)[:-len(".utppclip.json")]: f
             for f in glob.glob(os.path.join(clips_dir, "*.utppclip.json"))}
    plan = select_clips(style, sorted(files))
    rig_name = os.path.basename(rig_path).split(".")[0]

    clips = {}
    for _, _, _, cl in plan:
        for n in cl:
            if n not in clips:
                clips[n] = Clip(files[n], names, parents)
                if clips[n].rig != rig_name:
                    raise SystemExit("%s: rig %s != %s" % (n, clips[n].rig, rig_name))
    curve_names = sorted({k for c in clips.values() for k in c.curves})
    if len(curve_names) > 64:
        raise SystemExit("more than 64 curves")

    poses, frames, ovly = [], [], []
    pose_index = {}
    for oname, okind, oflags, cl in plan:
        first = len(poses)
        for n in cl:
            c = clips[n]
            meta = load_json(os.path.join(rel(META_DIR), n + ".json"))["additive"]
            at = ADD_TYPE[meta["additive_anim_type"]]
            base = meta.get("base") if at else None
            if n.startswith("AO_"):
                grp = GROUP["sweep"]
            elif okind == 2:
                grp = GROUP["base"]
            elif okind == 1:
                grp = GROUP["style"]
            elif okind == 3:
                grp = GROUP["transition"]
            else:
                grp = GROUP["overlay"]
            mask = 0
            cv = np.zeros((c.n, len(curve_names)))
            for ci, k in enumerate(curve_names):
                if k in c.curves:
                    mask |= 1 << ci
                    v = c.curves[k]
                    if len(v) != c.n:
                        raise SystemExit("%s: curve %s length %d != %d" % (n, k, len(v), c.n))
                    cv[:, ci] = v
            for k, val in sorted(ovr.get(n, {}).items()):
                if k not in c.curves:
                    raise SystemExit("%s: override curve %s is not in the source clip" % (n, k))
                ci = curve_names.index(k)
                applied.append(dict(pose=n, curve=k, value=val, frames=c.n,
                                    source_values=sorted({float(x) for x in c.curves[k]})))
                cv[:, ci] = val
            pose_index[n] = len(poses)
            poses.append(dict(name=n, group=grp, first=len(frames), n=c.n, add=at,
                              base_seq=(base or {}).get("seq"), base_frame=int((base or {}).get("frame", 0)),
                              base_kind=BASE_KIND[(base or {}).get("kind")], mask=mask))
            for f in range(c.n):
                frames.append((c, f, cv[f]))
        ovly.append(dict(name=oname, kind=okind, flags=oflags, first=first, n=len(poses) - first))

    missing = sorted(n for n in ovr if n not in pose_index)
    if missing:
        raise SystemExit("curve overrides name poses that are not baked: %s" % missing)

    for p in poses:
        if p["base_seq"] is not None:
            if p["base_seq"] not in pose_index:
                raise SystemExit("%s: additive base %s not baked" % (p["name"], p["base_seq"]))
            p["base_pose"] = pose_index[p["base_seq"]]
        else:
            p["base_pose"] = -1

    # deltas: (1) additive assets against their asset base, (2) style dP = D_MS(BP -> OB)
    deltas, dframes = [], []
    for p in poses:
        if p["add"]:
            X, A = clips[p["name"]], clips[p["base_seq"]]
            if p["base_frame"] >= A.n:
                raise SystemExit("%s: base frame out of range" % p["name"])
            d = delta(X, range(X.n), A, p["base_frame"])
            deltas.append(dict(name=p["name"], kind=0, target=pose_index[p["name"]], tfirst=0, n=X.n,
                               base=p["base_pose"], bframe=p["base_frame"], space=p["add"], first=len(dframes)))
            dframes.extend(d)
    for stn, bp in STYLE_DP:
        on = "Pose_%s_%s" % (style, stn)
        d = delta(clips[on], [0], clips[bp], 0)
        deltas.append(dict(name="dP_%s_%s" % (style, stn), kind=1, target=pose_index[on], tfirst=0, n=1,
                           base=pose_index[bp], bframe=0, space=1, first=len(dframes)))
        dframes.extend(d)

    masks = build_masks(names, parents, load_json(rel(cfg["bonemap"])))

    # ---- sections
    sec = []
    sec.append((b"BONE", NUM_BONES, b"".join(name_bytes(n, 32) + struct.pack("<i", p) for n, p in zip(names, parents))))
    sec.append((b"CURV", len(curve_names), b"".join(name_bytes(n, 48) for n in curve_names)))
    pb = bytearray()
    for p in poses:
        pb += name_bytes(p["name"], 64) + struct.pack("<IIIIiIIIQ", p["group"], p["first"], p["n"], p["add"],
                                                      p["base_pose"], p["base_frame"], p["base_kind"], 0, p["mask"])
    sec.append((b"POSE", len(poses), bytes(pb)))
    fb = bytearray()
    for c, f, cv in frames:
        a = np.concatenate([c.P[f], c.Q[f], c.S[f], c.GQ[f], c.GP[f]], axis=-1)  # (B,17)
        fb += a.astype("<f4").tobytes() + np.asarray(cv).astype("<f4").tobytes()
    sec.append((b"FRAM", len(frames), bytes(fb)))
    db = bytearray()
    for d in deltas:
        db += name_bytes(d["name"], 48) + struct.pack("<IiIIiIII", d["kind"], d["target"], d["tfirst"], d["n"],
                                                      d["base"], d["bframe"], d["space"], d["first"])
    sec.append((b"DELT", len(deltas), bytes(db)))
    sec.append((b"DFRM", len(dframes), b"".join(np.asarray(x).astype("<f4").tobytes() for x in dframes)))
    ob = bytearray()
    for o in ovly:
        ob += name_bytes(o["name"], 32) + struct.pack("<IIII", o["kind"], o["flags"], o["first"], o["n"])
    sec.append((b"OVLY", len(ovly), bytes(ob)))
    sec.append((b"MASK", len(masks), b"".join(name_bytes(k, 16) + m.astype("<f4").tobytes() for k, m in zip(MASKS, masks))))

    table_size = 16 * len(sec)
    off = HEADER_SIZE + table_size
    table = bytearray()
    body = bytearray()
    for tag, cnt, data in sec:
        table += tag + struct.pack("<III", off + len(body), len(data), cnt)
        body += data
        body += b"\x00" * ((-len(body)) % 16)
    payload = bytes(table) + bytes(body)
    file_size = HEADER_SIZE + len(payload)
    hdr = bytearray(HEADER_SIZE)
    hdr[0:8] = MAGIC
    struct.pack_into("<IIQIIII", hdr, 8, VERSION, HEADER_SIZE, rig_hash, NUM_BONES, len(sec), file_size,
                     zlib.crc32(payload) & 0xFFFFFFFF)
    struct.pack_into("<I", hdr, 40, zlib.crc32(bytes(hdr)) & 0xFFFFFFFF)
    blob = bytes(hdr) + payload
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(blob)

    summary = dict(format="utppov", version=VERSION, file=os.path.basename(out_path), size_bytes=len(blob),
                   sha256=hashlib.sha256(blob).hexdigest(), rig=rig_name, rig_hash="0x%016x" % rig_hash,
                   rig_json=os.path.relpath(rig_path, ROOT).replace(os.sep, "/"), style=style,
                   clips_dir=os.path.relpath(clips_dir, ROOT).replace(os.sep, "/"),
                   num_poses=len(poses), num_frames=len(frames), num_curves=len(curve_names), curves=curve_names,
                   num_deltas=len(deltas), num_delta_frames=len(dframes),
                   overlays=[dict(name=o["name"], kind=o["kind"], can_aim=bool(o["flags"] & 1),
                                  poses=[p["name"] for p in poses[o["first"]:o["first"] + o["n"]]]) for o in ovly],
                   deltas=[dict(name=d["name"], target=poses[d["target"]]["name"], base=poses[d["base"]]["name"],
                                base_frame=d["bframe"], space={1: "mesh_rotation_offset", 2: "local"}[d["space"]],
                                frames=d["n"]) for d in deltas],
                   masks={k: {names[b]: float(m[b]) for b in range(NUM_BONES) if m[b] != 0.0} for k, m in zip(MASKS, masks)},
                   curve_overrides_applied=applied,
                   source_sha256={n: clips[n].sha for n in sorted(clips)})
    with open(os.path.splitext(out_path)[0] + ".summary.json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(summary, f, indent=1)
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", choices=sorted(RIGS), action="append")
    ap.add_argument("--out-dir", default=rel("Data/overlay"))
    ap.add_argument("--no-overrides", action="store_true",
                    help="plain bake of the source clips, without the CURVE_OVERRIDES table")
    a = ap.parse_args()
    for r in a.rig or ["woman", "man"]:
        s = bake(r, os.path.join(a.out_dir, "utpp_overlays_%s.utppov" % r), overrides=not a.no_overrides)
        print("%s: %d poses, %d frames, %d curves, %d deltas, %d bytes, sha256 %s, rig_hash %s, overrides %d"
              % (r, s["num_poses"], s["num_frames"], s["num_curves"], s["num_deltas"], s["size_bytes"],
                 s["sha256"], s["rig_hash"], len(s["curve_overrides_applied"])))


if __name__ == "__main__":
    main()
