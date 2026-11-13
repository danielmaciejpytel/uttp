#!/usr/bin/env python3
"""UTPP Motion Matching database baker (Phase 3.3).

Reads retargeted clips (Animations/retarget/out/*.utppclip.json, format utppclip v1) and writes a
binary .utppmm database (spec: FORMAT.txt next to this script) plus a JSON summary.

Feature layout, per-group normalization (mean / average-std / weight), trajectory index clamping and the
small/large AABB acceleration structure follow Daniel Holden's "Motion-Matching" reference implementation
(orangeduck, MIT License, Copyright (c) 2021 Daniel Holden; local checkout Tools/repos/Motion-Matching:
database.h normalize_feature / compute_*_feature / database_build_bounds, resources/generate_database.py
velocity finite differences). The code here is an independent re-implementation for the CP2077 rig.

Deterministic: same inputs + config -> byte-identical output (no timestamps / dict-order dependence in the
binary or summary). Requires numpy.
"""
from __future__ import annotations

import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import re
import struct
import sys
import zlib

import numpy as np

FORMAT_VERSION = 1
MAGIC = b"UTPPMMDB"
HEADER_SIZE = 128
SECTION_ENTRY_SIZE = 32
ALIGN = 16

# Header flags
HDR_HAS_SCALE = 1 << 0

# Per-frame flags (FFLG)
FF_SEARCHABLE = 1 << 0
FF_EXCLUDED_NOTIFY = 1 << 1
FF_HORIZON_PAST_END = 1 << 2
FF_LOOP_DUPLICATE = 1 << 3
FF_BLOCK_TRANSITION = 1 << 4
FF_BRANCH_IN = 1 << 5
FF_CLIP_START = 1 << 6
FF_CLIP_END = 1 << 7

# Clip flags (CLIP)
CF_LOOP = 1 << 0
CF_ENDS_POSE_MATCH = 1 << 1
CF_NAME_LOOP = 1 << 2

# Feature group kinds (GRUP)
GK_BONE_POSITION = 1
GK_BONE_VELOCITY = 2
GK_TRAJ_POSITION = 3
GK_TRAJ_DIRECTION = 4

FNV64_OFFSET = 0xCBF29CE484222325
FNV64_PRIME = 0x100000001B3


def project_root() -> str:
    r = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    return os.path.dirname(r) if os.path.basename(r).lower() == "build" else r   # worktree Build/wt-*/tools/mmbake


class Log:
    def __init__(self, path: str | None):
        self.f = open(path, "a", encoding="utf-8") if path else None
        self.warnings = 0

    def __call__(self, msg: str = "") -> None:
        print(msg)
        if self.f:
            self.f.write(msg + "\n")
            self.f.flush()

    def warn(self, msg: str) -> None:
        self.warnings += 1
        self("WARNING: " + msg)


# ---------------------------------------------------------------- math helpers (float64)

def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz], -1)


def qrot(q, v):
    u = q[..., :3]
    w = q[..., 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def fk_model(P, Q, parents):
    """Model-space (Root-space) FK for all frames: P (N,B,3), Q (N,B,4) local -> global."""
    gp = np.zeros_like(P)
    gq = np.zeros_like(Q)
    for b, p in enumerate(parents):
        if p < 0:
            gp[:, b] = P[:, b]
            gq[:, b] = Q[:, b]
        else:
            gq[:, b] = qmul(gq[:, p], Q[:, b])
            gp[:, b] = gp[:, p] + qrot(gq[:, p], P[:, b])
    return gp, gq


def rot2(yaw, xy):
    """Rotate planar vectors xy (...,2) by yaw (...) CCW about +Z."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([c * xy[..., 0] - s * xy[..., 1], s * xy[..., 0] + c * xy[..., 1]], -1)


class Planar:
    """2D rigid transform (x, y, yaw) for the Trajectory bone. Yaw is kept unwrapped (continuous)."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    def compose(self, o: "Planar") -> "Planar":
        p = rot2(np.float64(self.yaw), np.array([o.x, o.y]))
        return Planar(self.x + p[0], self.y + p[1], self.yaw + o.yaw)

    def inverse(self) -> "Planar":
        p = rot2(np.float64(-self.yaw), np.array([-self.x, -self.y]))
        return Planar(p[0], p[1], -self.yaw)

    def power(self, q: int) -> "Planar":
        r = Planar(0.0, 0.0, 0.0)
        base = self if q >= 0 else self.inverse()
        for _ in range(abs(q)):
            r = base.compose(r)
        return r


def fnv1a64(data: bytes) -> int:
    h = FNV64_OFFSET
    for b in data:
        h ^= b
        h = (h * FNV64_PRIME) & 0xFFFFFFFFFFFFFFFF
    return h


def rig_hash(names, parents) -> int:
    """FNV-1a 64 over, for each bone in order: UTF-8 name, 0x00, int32 LE parent index."""
    buf = bytearray()
    for n, p in zip(names, parents):
        buf += n.encode("utf-8") + b"\x00" + struct.pack("<i", int(p))
    return fnv1a64(bytes(buf))


# ---------------------------------------------------------------- inputs

def find_key(o, key):
    if isinstance(o, dict):
        for k, v in o.items():
            if k == key:
                return v
            r = find_key(v, key)
            if r is not None:
                return r
    elif isinstance(o, list):
        for v in o:
            r = find_key(v, key)
            if r is not None:
                return r
    return None


def load_rig(path, nbones):
    rig = json.load(open(path, encoding="utf-8"))
    parents = [int(p) for p in find_key(rig, "boneParentIndexes")][:nbones]
    names = [n["$value"] if isinstance(n, dict) else str(n) for n in find_key(rig, "boneNames")][:nbones]
    if len(parents) != nbones or len(names) != nbones:
        raise SystemExit(f"rig {path}: fewer than {nbones} bones")
    for b, p in enumerate(parents):
        if not (p < b) or (b == 0) != (p < 0):
            raise SystemExit(f"rig {path}: bone {b} parent {p} not topologically ordered")
    return names, parents


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- per-clip processing

def split_dirs(spec: str):
    """--clips / --meta accept one directory or several separated by ';' (e.g. out_a;out_b)."""
    return [d for d in (s.strip() for s in spec.split(";")) if d]


def process_clip(path, meta_dir, cfg, names, parents, horizons, log):
    c = json.load(open(path, encoding="utf-8"))
    name = c["source_clip"]
    if c.get("format") != "utppclip" or c.get("version") != 1:
        raise SystemExit(f"{path}: not utppclip v1")
    if int(c["fps"]) != cfg["fps"]:
        raise SystemExit(f"{path}: fps {c['fps']} != config fps {cfg['fps']}")
    if list(c["bone_names"]) != names:
        raise SystemExit(f"{path}: bone_names differ from rig bones 0..{len(names) - 1}")
    B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
    N = B.shape[0]
    if N != int(c["num_frames"]) or B.shape[1:] != (len(names), 10):
        raise SystemExit(f"{path}: frame array shape {B.shape} inconsistent")
    if not np.all(np.isfinite(B)):
        raise SystemExit(f"{path}: non-finite values")
    if N < 4:
        raise SystemExit(f"{path}: need >= 4 frames for finite differences")
    P, Q, S = B[..., 0:3], B[..., 3:7].copy(), B[..., 7:10]
    qn = np.linalg.norm(Q, axis=-1, keepdims=True)
    if np.abs(qn - 1.0).max() > 1e-3:
        raise SystemExit(f"{path}: quaternion norm deviates {np.abs(qn - 1).max():.2e}")
    Q /= qn

    bi = {n: i for i, n in enumerate(names)}
    ROOT, TRAJ = bi[cfg["bones"]["root"]], bi[cfg["bones"]["trajectory"]]
    # Root must be identity: the body pose (Hips subtree) is then directly in the character frame.
    if np.abs(P[:, ROOT]).max() > 1e-6 or np.abs(Q[:, ROOT, :3]).max() > 1e-6:
        raise SystemExit(f"{path}: Root bone is not identity")
    if parents[TRAJ] != ROOT:
        raise SystemExit("Trajectory is not a child of Root")
    if np.abs(P[:, TRAJ, 2]).max() > 1e-5 or np.abs(Q[:, TRAJ, :2]).max() > 1e-5:
        raise SystemExit(f"{path}: Trajectory is not planar (z / pitch / roll present)")

    gp, _ = fk_model(P, Q, parents)  # character-frame (Root-space) positions; Trajectory not in the body chain
    tx, ty = P[:, TRAJ, 0], P[:, TRAJ, 1]
    yaw = np.unwrap(2.0 * np.arctan2(Q[:, TRAJ, 2], Q[:, TRAJ, 3]))

    # ---- loop detection: name pattern AND first/last body pose match (frame N-1 duplicates frame 0)
    name_loop = re.search(cfg["loop"]["name_regex"], name) is not None
    pose_delta = float(np.abs(gp[0, 2:] - gp[-1, 2:]).max())
    ends_match = pose_delta <= cfg["loop"]["max_pose_delta_m"]
    loop = name_loop and ends_match
    if name_loop and not ends_match:
        log.warn(f"{name}: named as loop but first/last pose differ by {pose_delta:.4f} m -> treated as NON-loop")
    T = [Planar(tx[i], ty[i], yaw[i]) for i in range(N)]
    period = N - 1
    D = T[N - 1].compose(T[0].inverse()) if loop else None

    def T_virtual(j):
        if 0 <= j < N:
            return T[j]
        if loop:
            q, r = divmod(j, period)
            return D.power(q).compose(T[r])
        return T[min(max(j, 0), N - 1)]  # orangeduck-style clamp

    def pose_index(j):
        return j % period if loop else min(max(j, 0), N - 1)

    # ---- world positions of feature bones for virtual frames -1..N (loop) / 0..N-1 (non-loop)
    feat_bones = [bi[cfg["bones"]["left_foot"]], bi[cfg["bones"]["right_foot"]], bi[cfg["bones"]["hips"]]]
    fps = float(cfg["fps"])
    js = list(range(-1, N + 1)) if loop else list(range(N))
    Tv = [T_virtual(j) for j in js]
    tvx = np.array([t.x for t in Tv])
    tvy = np.array([t.y for t in Tv])
    tvyaw = np.array([t.yaw for t in Tv])
    W = np.zeros((len(js), len(feat_bones), 3))
    for k, j in enumerate(js):
        m = gp[pose_index(j), feat_bones]
        W[k, :, :2] = rot2(np.full(len(feat_bones), tvyaw[k]), m[:, :2]) + np.array([tvx[k], tvy[k]])
        W[k, :, 2] = m[:, 2]
    if loop:
        Vw = (W[2:] - W[:-2]) * (0.5 * fps)  # central difference, wraps through the loop seam
    else:
        Vw = np.empty_like(W)
        Vw[1:-1] = 0.5 * (W[2:] - W[1:-1]) * fps + 0.5 * (W[1:-1] - W[:-2]) * fps
        Vw[0] = Vw[1] - (Vw[3] - Vw[2])        # orangeduck generate_database.py end extrapolation
        Vw[-1] = Vw[-2] + (Vw[-2] - Vw[-3])
    Vc = np.empty_like(Vw)  # world velocity expressed in the character frame of each frame
    for b in range(len(feat_bones)):
        Vc[:, b, :2] = rot2(-yaw, Vw[:, b, :2])
        Vc[:, b, 2] = Vw[:, b, 2]

    # ---- features (raw, un-normalized)
    lf, rf = feat_bones[0], feat_bones[1]
    nh = len(horizons)
    traj_pos = np.zeros((N, 2 * nh))
    traj_dir = np.zeros((N, 2 * nh))
    for i in range(N):
        for k, h in enumerate(horizons):
            tf = T_virtual(i + h)
            rel = rot2(np.float64(-yaw[i]), np.array([tf.x - tx[i], tf.y - ty[i]]))
            d = tf.yaw - yaw[i]
            traj_pos[i, 2 * k:2 * k + 2] = rel
            traj_dir[i, 2 * k:2 * k + 2] = (-math.sin(d), math.cos(d))  # +Y forward rotated by d
    raw = {
        "left_foot_position": gp[:, lf],
        "right_foot_position": gp[:, rf],
        "left_foot_velocity": Vc[:, 0],
        "right_foot_velocity": Vc[:, 1],
        "hip_velocity": Vc[:, 2],
        "trajectory_position": traj_pos,
        "trajectory_direction": traj_dir,
    }

    # ---- frame flags
    flags = np.zeros(N, dtype=np.uint32)
    flags[0] |= FF_CLIP_START
    flags[-1] |= FF_CLIP_END
    t = np.arange(N) / fps
    eps = cfg["notify_time_epsilon_s"]
    notify_hits = {"exclude": 0, "block": 0, "branch": 0}

    def mark(nt, bit, key):
        s = float(nt["trigger_time"])
        d = float(nt.get("duration", 0.0))
        if d > 0:
            sel = (t >= s - eps) & (t <= s + d + eps)
        else:
            sel = np.zeros(N, bool)
            sel[min(max(int(round(s * fps)), 0), N - 1)] = True
        flags[sel] |= bit
        notify_hits[key] += 1

    for nt in c.get("notifies", []):
        nn = nt.get("notify_name", "")
        if nn in cfg["exclude_notifies"]:
            mark(nt, FF_EXCLUDED_NOTIFY, "exclude")
        elif nn in cfg["block_transition_notifies"]:
            mark(nt, FF_BLOCK_TRANSITION, "block")
        elif nn in cfg["branch_in_notifies"]:
            mark(nt, FF_BRANCH_IN, "branch")
    if loop:
        flags[N - 1] |= FF_LOOP_DUPLICATE  # identical to frame 0; kept for playback only
    else:
        flags[np.arange(N) + max(horizons) > N - 1] |= FF_HORIZON_PAST_END
    blocking = FF_EXCLUDED_NOTIFY | FF_HORIZON_PAST_END | FF_LOOP_DUPLICATE
    if cfg["block_transition_unsearchable"]:
        blocking |= FF_BLOCK_TRANSITION
    flags[(flags & blocking) == 0] |= FF_SEARCHABLE

    # ---- root motion deltas (forward: frame i -> next played frame), in the Trajectory frame of frame i
    root = np.zeros((N, 4))
    for i in range(N):
        if i < N - 1:
            a, b = T[i], T[i + 1]
        elif loop:
            a, b = T[i], T_virtual(N)  # frame N-1 == frame 0 -> continue into frame 1
        else:
            a, b = T[N - 2], T[N - 1]  # extrapolate the last step
        rel = rot2(np.float64(-a.yaw), np.array([b.x - a.x, b.y - a.y]))
        root[i, :3] = (rel[0], rel[1], b.yaw - a.yaw)

    curves = c.get("curves", {})
    contacts = np.zeros((N, 2))
    for k, cn in enumerate(("contact_l", "contact_r")):
        if cn in curves and len(curves[cn]) == N:
            contacts[:, k] = curves[cn]
        else:
            log.warn(f"{name}: curve {cn} missing -> 0")

    # meta_dir may list several directories separated by ';' (first directory holding <clip>.json wins)
    meta_path = next((p for p in (os.path.join(d, name + ".json") for d in split_dirs(meta_dir)) if os.path.exists(p)), "")
    tags = sorted(json.load(open(meta_path, encoding="utf-8"))["databases"]) if meta_path else []
    if not tags:
        log.warn(f"{name}: no database tags (meta missing)")

    return {
        "name": name, "path": path, "N": N, "P": P, "Q": Q, "S": S, "raw": raw, "flags": flags,
        "root": root, "contacts": contacts, "tags": tags, "loop": loop, "name_loop": name_loop,
        "ends_match": ends_match, "pose_delta": pose_delta, "period": period if loop else 0,
        "notify_hits": notify_hits,
    }


# ---------------------------------------------------------------- binary writer

class Strings:
    def __init__(self):
        self.blob = bytearray()
        self.index = {}

    def add(self, s: str) -> int:
        if s not in self.index:
            self.index[s] = len(self.blob)
            self.blob += s.encode("utf-8") + b"\x00"
        return self.index[s]


def fourcc(s: str) -> int:
    return struct.unpack("<I", s.encode("ascii"))[0]


def write_db(path, header_fields, sections):
    """sections: list of (id str, count, elem_size, bytes)."""
    nsec = len(sections)
    table_off = HEADER_SIZE
    off = table_off + nsec * SECTION_ENTRY_SIZE
    entries = []
    body = bytearray()
    for sid, count, esz, data in sections:
        if len(data) != count * esz:
            raise SystemExit(f"section {sid}: size {len(data)} != {count}*{esz}")
        off = (off + ALIGN - 1) // ALIGN * ALIGN
        entries.append((fourcc(sid), count, esz, 0, off, len(data)))
        body_pos = off - (table_off + nsec * SECTION_ENTRY_SIZE)
        body += b"\x00" * (body_pos - len(body))
        body += data
        off += len(data)
    file_size = (off + ALIGN - 1) // ALIGN * ALIGN
    body += b"\x00" * (file_size - table_off - nsec * SECTION_ENTRY_SIZE - len(body))
    table = b"".join(struct.pack("<IIIIQQ", *e) for e in entries)
    payload = table + bytes(body)
    payload_crc = zlib.crc32(payload) & 0xFFFFFFFF
    h = header_fields
    hdr = bytearray(HEADER_SIZE)
    struct.pack_into("<8sII", hdr, 0, MAGIC, FORMAT_VERSION, HEADER_SIZE)
    struct.pack_into("<QQI", hdr, 16, file_size, h["rig_hash"], h["flags"])
    struct.pack_into("<IIIIIIIIIIIfIII", hdr, 36,
                     h["num_frames"], h["num_bones"], h["num_features"], h["num_clips"], h["num_ranges"],
                     h["num_groups"], h["num_tags"], h["bound_sm_size"], h["bound_lr_size"],
                     h["num_bound_sm"], h["num_bound_lr"], h["fps"], nsec, table_off, payload_crc)
    # header_crc32 at offset 96 computed with the field zeroed
    hcrc = zlib.crc32(bytes(hdr)) & 0xFFFFFFFF
    struct.pack_into("<I", hdr, 96, hcrc)
    data = bytes(hdr) + payload
    assert len(data) == file_size
    with open(path, "wb") as f:
        f.write(data)
    return data


# ---------------------------------------------------------------- main bake

def bake(args, log):
    root = project_root()
    cfg = json.load(open(args.config, encoding="utf-8"))
    nb = int(cfg["num_pose_bones"])
    names, parents = load_rig(args.rig, nb)
    fps = int(cfg["fps"])
    horizons = [int(round(s * fps)) for s in cfg["trajectory_horizons_s"]]
    if len(horizons) > 4 or len(horizons) < 1:
        raise SystemExit("1..4 trajectory horizons supported")
    rh = rig_hash(names, parents)
    log(f"config: {os.path.relpath(args.config, root)} horizons(frames)={horizons} rig_hash=0x{rh:016x}")

    files = sorted(f for d in split_dirs(args.clips) for f in glob.glob(os.path.join(d, "*.utppclip.json")))
    dup = sorted({b for b in (os.path.basename(f) for f in files) if [os.path.basename(g) for g in files].count(b) > 1})
    if dup:
        raise SystemExit(f"duplicate clip files across --clips directories: {dup[:5]}")
    if getattr(args, "clip_names_from", None):  # optional: restrict to the clips listed in a mmbake .summary.json
        keep = {c["name"] for c in json.load(open(args.clip_names_from, encoding="utf-8"))["clips"]}
        files = [f for f in files if os.path.basename(f)[:-len(".utppclip.json")] in keep]
        if len(files) != len(keep):
            raise SystemExit(f"--clip-names-from: {len(keep)} clips listed, {len(files)} found")
    if not files:
        raise SystemExit("no clips found")
    clips = [process_clip(f, args.meta, cfg, names, parents, horizons, log) for f in files]
    clips.sort(key=lambda c: c["name"])

    # ---- feature groups (orangeduck order)
    bi = {n: i for i, n in enumerate(names)}
    W = cfg["weights"]
    ov = cfg.get("group_weight_overrides", {})
    group_defs = [
        ("left_foot_position", GK_BONE_POSITION, 3, bi[cfg["bones"]["left_foot"]], W["foot_position"]),
        ("right_foot_position", GK_BONE_POSITION, 3, bi[cfg["bones"]["right_foot"]], W["foot_position"]),
        ("left_foot_velocity", GK_BONE_VELOCITY, 3, bi[cfg["bones"]["left_foot"]], W["foot_velocity"]),
        ("right_foot_velocity", GK_BONE_VELOCITY, 3, bi[cfg["bones"]["right_foot"]], W["foot_velocity"]),
        ("hip_velocity", GK_BONE_VELOCITY, 3, bi[cfg["bones"]["hips"]], W["hip_velocity"]),
        ("trajectory_position", GK_TRAJ_POSITION, 2 * len(horizons), -1, W["trajectory_position"]),
        ("trajectory_direction", GK_TRAJ_DIRECTION, 2 * len(horizons), -1, W["trajectory_direction"]),
    ]
    for k in ov:
        if k not in [g[0] for g in group_defs]:
            raise SystemExit(f"unknown group override {k}")
    F = sum(g[2] for g in group_defs)

    raw = np.concatenate([np.concatenate([c["raw"][g[0]] for g in group_defs], axis=1) for c in clips], axis=0)
    flags = np.concatenate([c["flags"] for c in clips])
    NF = raw.shape[0]
    searchable = (flags & FF_SEARCHABLE) != 0
    if not searchable.any():
        raise SystemExit("no searchable frames")
    norm_rows = searchable if cfg["normalization_frames"] == "searchable" else np.ones(NF, bool)

    offsets = np.zeros(F)
    scales = np.zeros(F)
    groups = []
    o = 0
    for gname, kind, size, bone, w in group_defs:
        w = float(ov.get(gname, w))
        if not (w > 0):
            raise SystemExit(f"group {gname}: weight must be > 0")
        x = raw[norm_rows, o:o + size]
        mean = x.mean(axis=0)
        std = float(np.mean(np.sqrt(((x - mean) ** 2).mean(axis=0))))  # orangeduck: average std over dims
        if not (std > 1e-8):
            raise SystemExit(f"group {gname}: zero variance")
        offsets[o:o + size] = mean
        scales[o:o + size] = std / w
        groups.append((gname, kind, o, size, bone, w, std))
        o += size
    feats = ((raw - offsets) / scales).astype(np.float32)
    offsets32 = offsets.astype(np.float32)
    scales32 = scales.astype(np.float32)

    # ---- clip table, search ranges
    tag_names = sorted({t for c in clips for t in c["tags"]})
    if len(tag_names) > 64:
        raise SystemExit("more than 64 tags")
    ranges = []
    first = 0
    for c in clips:
        c["first"] = first
        s = (c["flags"] & FF_SEARCHABLE) != 0
        i = 0
        while i < c["N"]:
            if s[i]:
                j = i
                while j < c["N"] and s[j]:
                    j += 1
                ranges.append((first + i, first + j))
                i = j
            else:
                i += 1
        first += c["N"]
    assert first == NF

    # ---- AABB (orangeduck database_build_bounds) over searchable frames only; empty boxes stay 0 (never visited)
    SM, LR = int(cfg["aabb"]["small"]), int(cfg["aabb"]["large"])
    if SM <= 0 or LR <= 0 or LR % SM != 0:
        raise SystemExit("aabb sizes must be positive and large % small == 0")
    nsm, nlr = (NF + SM - 1) // SM, (NF + LR - 1) // LR

    def boxes(size, n):
        mn = np.zeros((n, F), np.float32)
        mx = np.zeros((n, F), np.float32)
        for b in range(n):
            sel = searchable[b * size:(b + 1) * size]
            rows = feats[b * size:(b + 1) * size][sel]
            if len(rows):
                mn[b], mx[b] = rows.min(axis=0), rows.max(axis=0)
        return mn, mx

    bsmn, bsmx = boxes(SM, nsm)
    blrn, blrx = boxes(LR, nlr)

    # ---- pose / scale / root / contacts
    P = np.concatenate([c["P"] for c in clips])
    Qa = np.concatenate([c["Q"] for c in clips])
    S = np.concatenate([c["S"] for c in clips])
    pose = np.concatenate([P, Qa], axis=-1).astype(np.float32)  # (NF, nb, 7)
    scale_dev = float(np.abs(S - 1.0).max())
    has_scale = scale_dev > cfg["scale_tolerance"]
    rootd = np.concatenate([c["root"] for c in clips]).astype(np.float32)
    cont = np.concatenate([c["contacts"] for c in clips]).astype(np.float32)

    # ---- serialize
    st = Strings()
    bone_bytes = b"".join(struct.pack("<IiII", st.add(n), p, len(n.encode("utf-8")), 0) for n, p in zip(names, parents))
    tag_bytes = b"".join(struct.pack("<I", st.add(t)) for t in tag_names)
    clip_bytes = bytearray()
    for c in clips:
        cf = (CF_LOOP if c["loop"] else 0) | (CF_ENDS_POSE_MATCH if c["ends_match"] else 0) | \
             (CF_NAME_LOOP if c["name_loop"] else 0)
        mask = 0
        for t in c["tags"]:
            mask |= 1 << tag_names.index(t)
        nsearch = int(((c["flags"] & FF_SEARCHABLE) != 0).sum())
        clip_bytes += struct.pack("<IIIIIIQ", st.add(c["name"]), c["first"], c["N"], cf, c["period"], nsearch, mask)
    grp_bytes = bytearray()
    for gname, kind, o, size, bone, w, std in groups:
        hz = horizons + [0] * (4 - len(horizons)) if kind in (GK_TRAJ_POSITION, GK_TRAJ_DIRECTION) else [0, 0, 0, 0]
        nhz = len(horizons) if kind in (GK_TRAJ_POSITION, GK_TRAJ_DIRECTION) else 0
        grp_bytes += struct.pack("<IIIIiffI4i", st.add(gname), kind, o, size, bone, w, std, nhz, *hz)
    rng_bytes = b"".join(struct.pack("<II", a, b) for a, b in ranges)
    while len(st.blob) % 4:
        st.blob += b"\x00"

    sections = [
        ("STRS", len(st.blob), 1, bytes(st.blob)),
        ("BONE", nb, 16, bone_bytes),
        ("TAGS", len(tag_names), 4, tag_bytes),
        ("CLIP", len(clips), 32, bytes(clip_bytes)),
        ("GRUP", len(groups), 48, bytes(grp_bytes)),
        ("FOFF", F, 4, offsets32.tobytes()),
        ("FSCL", F, 4, scales32.tobytes()),
        ("FEAT", NF, 4 * F, feats.astype("<f4").tobytes()),
        ("FFLG", NF, 4, flags.astype("<u4").tobytes()),
        ("RNGS", len(ranges), 8, rng_bytes),
        ("BSMN", nsm, 4 * F, bsmn.astype("<f4").tobytes()),
        ("BSMX", nsm, 4 * F, bsmx.astype("<f4").tobytes()),
        ("BLRN", nlr, 4 * F, blrn.astype("<f4").tobytes()),
        ("BLRX", nlr, 4 * F, blrx.astype("<f4").tobytes()),
        ("POSE", NF, nb * 7 * 4, pose.astype("<f4").tobytes()),
        ("ROOT", NF, 16, rootd.astype("<f4").tobytes()),
        ("CONT", NF, 8, cont.astype("<f4").tobytes()),
    ]
    if has_scale:
        sections.append(("SCAL", NF, nb * 3 * 4, S.astype("<f4").tobytes()))
    hdr = dict(rig_hash=rh, flags=HDR_HAS_SCALE if has_scale else 0, num_frames=NF, num_bones=nb, num_features=F,
               num_clips=len(clips), num_ranges=len(ranges), num_groups=len(groups), num_tags=len(tag_names),
               bound_sm_size=SM, bound_lr_size=LR, num_bound_sm=nsm, num_bound_lr=nlr, fps=float(fps))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    data = write_db(args.out, hdr, sections)
    out_sha = hashlib.sha256(data).hexdigest()

    # ---- report
    n_excl = int(((flags & FF_EXCLUDED_NOTIFY) != 0).sum())
    n_hor = int(((flags & FF_HORIZON_PAST_END) != 0).sum())
    n_dup = int(((flags & FF_LOOP_DUPLICATE) != 0).sum())
    n_blk = int(((flags & FF_BLOCK_TRANSITION) != 0).sum())
    n_search = int(searchable.sum())
    log(f"clips={len(clips)} frames={NF} bones={nb} features={F} groups={len(groups)} tags={len(tag_names)} ranges={len(ranges)}")
    log(f"searchable={n_search} unsearchable={NF - n_search} (notify-excluded={n_excl}, horizon-past-end={n_hor}, "
        f"loop-duplicate={n_dup}; overlaps counted in each) block_transition_flagged={n_blk}")
    loops = [c["name"] for c in clips if c["loop"]]
    pose_only = [c["name"] for c in clips if c["ends_match"] and not c["name_loop"]]
    log(f"loops={len(loops)}: {', '.join(loops)}")
    log(f"ends-pose-match but not named loop (treated as non-loop): {len(pose_only)}: {', '.join(pose_only)}")
    for gname, kind, o, size, bone, w, std in groups:
        log(f"group {gname:22s} kind={kind} dims={o}..{o + size - 1} bone={bone:3d} weight={w:.3f} std={std:.5f}")
    log(f"scale max |s-1| = {scale_dev:.2e} -> SCAL section {'stored' if has_scale else 'omitted'}")
    log(f"AABB small={SM} ({nsm} boxes) large={LR} ({nlr} boxes)")
    log(f"output {args.out} size={len(data)} sha256={out_sha}")

    if args.summary:
        inputs_sha = hashlib.sha256()
        for c in clips:
            inputs_sha.update(c["name"].encode() + b"\x00" + sha256_file(c["path"]).encode())
        summary = {
            "format": "utppmm", "version": FORMAT_VERSION, "file": os.path.basename(args.out), "size_bytes": len(data),
            "sha256": out_sha, "rig": "player_woman_skeleton", "rig_hash": f"0x{rh:016x}",
            "config": os.path.relpath(args.config, root).replace("\\", "/"),
            "config_sha256": sha256_file(args.config), "inputs_sha256": inputs_sha.hexdigest(),
            "fps": fps, "horizons_frames": horizons, "num_frames": NF, "num_bones": nb, "num_features": F,
            "has_scale": has_scale, "num_clips": len(clips), "num_ranges": len(ranges),
            "frames_searchable": n_search, "frames_unsearchable": NF - n_search,
            "frames_notify_excluded": n_excl, "frames_horizon_past_end": n_hor, "frames_loop_duplicate": n_dup,
            "frames_block_transition_flagged": n_blk,
            "aabb": {"small": SM, "large": LR, "num_small": nsm, "num_large": nlr},
            "tags": tag_names,
            "groups": [{"name": g[0], "kind": g[1], "first": g[2], "size": g[3], "bone": g[4], "weight": g[5],
                        "std": round(g[6], 7)} for g in groups],
            "clips": [{"name": c["name"], "first_frame": c["first"], "num_frames": c["N"], "loop": c["loop"],
                       "ends_pose_delta_m": round(c["pose_delta"], 6), "tags": c["tags"],
                       "searchable": int(((c["flags"] & FF_SEARCHABLE) != 0).sum()),
                       "notify_excluded": int(((c["flags"] & FF_EXCLUDED_NOTIFY) != 0).sum())} for c in clips],
        }
        with open(args.summary, "w", encoding="utf-8", newline="\n") as f:
            json.dump(summary, f, indent=1)
            f.write("\n")
    return data


def main():
    r = project_root()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--clips", default=os.path.join(r, "Animations", "retarget", "out"))
    ap.add_argument("--meta", default=os.path.join(r, "Animations", "sourcepack", "SourcePack_export", "meta"))
    ap.add_argument("--rig", default=os.path.join(r, "Extracted", "game", "json", "vanilla", "player_woman_skeleton.rig.json"))
    ap.add_argument("--config", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "default.json"))
    ap.add_argument("--out", default=None, help="output .utppmm (default v1: Data/utpp_base_dense_subset.utppmm, "
                                                "v2: Data/utpp_motion_test.utppmm)")
    ap.add_argument("--summary", default=None, help="JSON summary path (default: <out>.summary.json)")
    ap.add_argument("--clip-names-from", default=None, help="only bake clips listed in this .summary.json (default: all)")
    ap.add_argument("--no-summary", action="store_true")
    ap.add_argument("--log", default=None)
    ap.add_argument("--format", choices=("v1", "v2"), default="v1", help="v1 (default, unchanged) or v2 (FORMAT.txt part 2)")
    g2 = ap.add_argument_group("v2 only (data paths default to the baker data file)")
    g2.add_argument("--v2-config", default=os.path.join(r, "Research", "inventory", "mmbake-sourcepack.json"))
    g2.add_argument("--sourcepack-dir", default=None, help="source pack extraction (default: data file sources.sourcepack_dir)")
    g2.add_argument("--clip-map", default=None, help="sourcepack-clip-map.json (default: data file sources.clip_map)")
    g2.add_argument("--bonemap", default=None, help="UE -> game bone map (default: data file sources.bonemap)")
    g2.add_argument("--chooser-out", default=None, help="chooser table data file (default: <out>.chooser.json)")
    g2.add_argument("--no-chooser", action="store_true")
    g2.add_argument("--raw-dump", default=None, help="write raw features / rows (.npz) for the v2 tests")
    g2.add_argument("--motion-src", default=None, help="motion source of the --clips (data file motion_sources; default "
                                                       "motion_sources.default)")
    g2.add_argument("--ue-tracks", default=None, help="UE source tracks of dump_ue_tracks.py for the MotionExtractor "
                                                      "curve validation (default: data file sources.ue_tracks)")
    g2.add_argument("--rtz-clips", default=None, help="';'-separated dirs of RTZ re-retargeted clips (root_z track, "
                                                      "retarget/RTZ.txt) replacing the same-named --clips files")
    g2.add_argument("--sourcepack-only", action="store_true", help="bake Neutral clips only (no clip-map female clips)")
    g2.add_argument("--slide-spec", default=None, help="source pack 1.2 slide spec (make_slide_spec.py): extra schemas / databases / "
                                                       "normalization sets, authored notify windows and the chooser 'slide' section")
    g2.add_argument("--extra-assets", default=None, help="extra_assets.py spec: authored retargeted clips appended as entries "
                                                         "of existing databases (optional; without it the bake is unchanged)")
    a = ap.parse_args()
    if a.out is None:
        a.out = os.path.join(r, "Data", "utpp_motion_test.utppmm" if a.format == "v2" else "utpp_base_dense_subset.utppmm")
    if a.summary is None and not a.no_summary:
        a.summary = os.path.splitext(a.out)[0] + ".summary.json"
    if a.no_summary:
        a.summary = None
    log = Log(a.log)
    log(f"==== mmbake {datetime.datetime.now().isoformat(timespec='seconds')} python {sys.version.split()[0]} numpy {np.__version__}")
    if a.format == "v2":
        import mmbake_motion
        mmbake_motion.bake_v2(a, log)
    else:
        bake(a, log)
    log(f"warnings={log.warnings}")
    log("BAKE_OK")


if __name__ == "__main__":
    main()
