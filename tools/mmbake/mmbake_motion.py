#!/usr/bin/env python3
"""UTPP Motion Matching database baker, format v2.

Called by mmbake_base.py --format v2 (v1 stays the default and is byte-for-byte unchanged). Binary layout:
FORMAT.txt, part "VERSION 2". The module contains no Neutral database / clip names, thresholds or regexes: they are
read at bake time from the extraction (Extracted\\sourcepack\\sourcepack\\posesearch.json, anims.json, anim_curves.json,
choosers.json), the clip map (Research\\inventory\\sourcepack-clip-map.json) and the baker data file
(Research\\inventory\\mmbake-sourcepack.json). Everything written is source-pack-derived data.

UE 5.8 PoseSearch behaviour reproduced (PS = Engine\\Plugins\\Animation\\PoseSearch\\
Source\\Runtime\\Private):
  * schemas -> leaf channels as the UE Finalize() of Trajectory / Group / Pose channels
    (PS\\PoseSearchFeatureChannel_Trajectory.cpp:50-150, _Pose.cpp:36-120, _Group.cpp:8-23);
  * feature values as FAssetIndexer (PS\\PoseSearchAssetIndexer.cpp:376-610): positions relative to the root at the
    origin time; velocity = 1/60 s backward difference (FiniteDelta, PoseSearchDefines.h:43) with the origin moving
    along (character space) or fixed (world space); velocity direction = velocity clamped to length 1 in UE units
    (PS\\PoseSearchFeatureChannel_Velocity.cpp:199-220); heading = bone axis, stripped then normalized
    (PS\\PoseSearchFeatureVectorHelper.cpp:25-103);
  * root motion outside the clip: loops wrap, non-loops extrapolate the first / last 1/30 s of root motion
    (PS\\PoseSearchAssetSampler.cpp:15-52, 600-660) - replaces the v1 HORIZON_PAST_END cut; poses are clamped;
  * indexed poses (FindValidRanges PS\\PoseSearchDerivedData.cpp:409-505, InitSearchIndexAssets 531-640,
    FSearchIndexAsset ceil/floor, Public\\PoseSearch\\PoseSearchIndex.h:117-140, TRange semantics);
  * pose metadata (PS\\PoseSearchAssetIndexer.cpp:180-215): BlockTransition, cost addend = BaseCostBias or the
    ModifyCost value (replaces), + LoopingCostBias for looping assets; continuing addend = ContinuingPoseCostBias or
    OverrideContinuingPoseCostBias (PS\\PoseSearchDatabase.cpp:1754-1862); notify states active in a 1/120 s window
    centred on the sample time (PS\\PoseSearchAssetSampler.cpp:670-718);
  * normalization (PS\\PoseSearchDerivedData.cpp:231-405, 654-712, 2092-2160): mean absolute deviation (mean norm of
    the centred vectors) per group of channels that CanBeNormalizedWith (PS\\PoseSearchFeatureChannel.cpp:93-127),
    over every indexed pose of every database of (normalization set + main database) indexed with the main schema
    (NormalizeWithCommonSchema); deviation <= 0.1 UE units -> 1 UE unit, i.e. <= 0.001 -> 0.01 for metres / metres per
    second and <= 0.1 -> 1 for unitless directions; weights w / sum(w), square root, divided by the deviation;
  * search (reference for the C++ runtime): cost = sum((q - f)^2) over normalized features + row addend, BlockTransition
    rows never start a jump, AABB 16/64 with the minimum addend per box, equal to brute force.
AnimModifiers recreated: MotionExtractorModifier TranslationSpeed (FootSpeed_L/R, MoveData_Speed),
AM_OrientationWarpingAlpha / AM_RateWarpingAlpha, AM_BakePhaseCurveFromFootstepNotifies (sources quoted in the data
file). Requires numpy.
"""
from __future__ import annotations

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mmbake_base as v1  # noqa: E402  (shared helpers: FK, rig, strings, FNV)

FORMAT_VERSION = 2
MAGIC = b"UTPPMMDB"
HEADER_SIZE = 128
SECTION_ENTRY_SIZE = 32
ALIGN = 16
FPS = 30
HALF = 2 * FPS                     # half-frame grid (60 Hz): every source pack sample offset is a multiple of 1/60 s

HDR_HAS_SCALE = 1 << 0

# FFLG (per global frame)
FF_SELECTABLE = 1 << 0             # non-block indexed row in at least one database
FF_EXCLUDED_NOTIFY = 1 << 1        # inside an ExcludeFromDatabase notify state
FF_LOOP_DUPLICATE = 1 << 3
FF_BLOCK_TRANSITION = 1 << 4       # BlockTransition notify state active
FF_BRANCH_IN = 1 << 5
FF_CLIP_START = 1 << 6
FF_CLIP_END = 1 << 7
FF_EXCLUDED_TAIL = 1 << 8          # removed by ExcludeFromDatabaseParameters (last 0.3 s of non-loops)
FF_INDEXED = 1 << 9                # indexed (row) in at least one database

CF_LOOP = 1 << 0
CF_ENDS_POSE_MATCH = 1 << 1
CF_NAME_LOOP = 1 << 2
CF_RTZ_UNKNOWN = 1 << 3            # vertical root expected but no root_z track: RTZ rows 0 and NOT valid
CF_ADDED = 1 << 4
CF_RTZ_PRESENT = 1 << 5            # RTZ rows filled from the clip root_z track (RTZ.txt)
CF_RTZ_NONE = 1 << 6               # no vertical root: RTZ rows are 0 by definition (valid)

# CLP2 rtz_source
RTZ_SRC_NONE, RTZ_SRC_UNKNOWN, RTZ_SRC_ROOT_Z = 0, 1, 2

# rig identity (header rig_hash of v2): topology + rest local translations quantized to 1 mm (0.1 mm would put
# rig values printed with 9 digits, e.g. -0.0209500007, within 1e-9 m of a rounding boundary)
RIG_ID_QUANT_PER_M = 1000.0
RIG_ID_MIN_MARGIN = 1e-3           # quanta (1 um): closer to a rounding boundary -> warning

# channel kinds (SCHC)
CK_POSITION, CK_VELOCITY, CK_VELOCITY_DIR, CK_HEADING = 1, 2, 3, 4
CK_NAMES = {CK_POSITION: "Pos", CK_VELOCITY: "Vel", CK_VELOCITY_DIR: "VelDir", CK_HEADING: "Head"}
COMP_XYZ, COMP_XY, COMP_Z = 7, 3, 4
VS_NONE, VS_CHARACTER, VS_WORLD = 0, 1, 2
IQ_CONTINUING, IQ_CHARACTER = 0, 1
CHF_TRAJECTORY = 1 << 0
CHF_NORMALIZE_OUTPUT = 1 << 1
CHF_CLAMP_UNIT_UE = 1 << 2
CHF_Z_FROM_RTZ = 1 << 3
CHF_UNITLESS = 1 << 4

PREPROC = {"NONE": 0, "NORMALIZE": 1, "NORMALIZE_WITH_COMMON_SCHEMA": 2}

# DROW flags
RF_BLOCK = 1 << 0
RF_BRANCH_IN = 1 << 1
RF_LOOPING = 1 << 2
RF_RANGE_START = 1 << 3
RF_RANGE_END = 1 << 4

# DENT flags
EF_ENABLED = 1 << 0
EF_DISABLE_RESELECTION = 1 << 1
EF_LOOPING = 1 << 2
EF_ADDED = 1 << 3
EF_NOTIFY_MOTION58 = 1 << 4

# DBAS flags
DF_RTZ_PENDING = 1 << 0
DF_IN_NORMALIZATION_SET = 1 << 1

# EVNT flags
EVF_SIDE_LEFT = 1
EVF_SIDE_RIGHT = 2
EVF_DERIVED = 1 << 2
EVF_STATE = 1 << 3

CU_UNITS = {"alpha": 0, "m/s": 1, "m": 2, "phase": 3}

UE_MIN_DEV = 0.1                   # PoseSearchDerivedData.cpp:396-397 (UE units)
UE_CM = 100.0                      # 1 m = 100 UE units
DEV_LEN = (UE_MIN_DEV / UE_CM, 1.0 / UE_CM)      # metres: threshold 0.001, replacement 0.01
DEV_UNITLESS = (UE_MIN_DEV, 1.0)
NOTIFY_WINDOW = 1.0 / 120.0        # PoseSearchAssetSampler.h:79 ExtractionInterval
KEY_TOL = 1e-4                     # FRichCurve::UpdateOrAddKey tolerance (KINDA_SMALL_NUMBER)
F32 = np.float32


def short_name(path):
    s = str(path or "")
    s = s.rsplit("/", 1)[-1]
    return s.split(".", 1)[-1] if "." in s else s


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


# ============================================================================================ planar transforms
# arrays (..., 3) = (x, y, yaw); a o b = apply b first then a (Planar.compose of v1)

def p_compose(a, b):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    c, s = np.cos(a[..., 2]), np.sin(a[..., 2])
    return np.stack([a[..., 0] + c * b[..., 0] - s * b[..., 1], a[..., 1] + s * b[..., 0] + c * b[..., 1],
                     a[..., 2] + b[..., 2]], -1)


def p_inverse(a):
    a = np.asarray(a, float)
    c, s = np.cos(-a[..., 2]), np.sin(-a[..., 2])
    return np.stack([-(c * a[..., 0] - s * a[..., 1]), -(s * a[..., 0] + c * a[..., 1]), -a[..., 2]], -1)


def p_power(d, n):
    r = np.zeros(3)
    base = d if n >= 0 else p_inverse(d)
    for _ in range(abs(int(n))):
        r = p_compose(base, r)
    return r


def rot2(yaw, xy):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.stack([c * xy[..., 0] - s * xy[..., 1], s * xy[..., 0] + c * xy[..., 1]], -1)


def qz(yaw):
    return np.stack([np.zeros_like(yaw), np.zeros_like(yaw), np.sin(0.5 * yaw), np.cos(0.5 * yaw)], -1)


# ============================================================================================ TRange emulation

class Bound:
    __slots__ = ("v", "incl", "open")

    def __init__(self, v=0.0, incl=True, is_open=False):
        self.v, self.incl, self.open = F32(v), incl, is_open


def rng(lo, hi, lo_incl=True, hi_incl=True):
    return (Bound(lo, lo_incl), Bound(hi, hi_incl))


def r_empty(r):
    lo, hi = r
    if lo.open or hi.open:
        return False
    if lo.v > hi.v:
        return True
    return lo.v == hi.v and (not lo.incl or not hi.incl)


def _upper_overlap(a_hi, b_lo):
    if a_hi.open or b_lo.open:
        return True
    if a_hi.incl and b_lo.incl:
        return b_lo.v <= a_hi.v
    return b_lo.v < a_hi.v


def r_overlaps(x, y):
    if r_empty(x) or r_empty(y):
        return False
    return _upper_overlap(x[1], y[0]) and _upper_overlap(y[1], x[0])


def _max_lower(a, b):
    if a.open:
        return b
    if b.open:
        return a
    if a.v != b.v:
        return a if a.v > b.v else b
    return a if not a.incl else b


def _min_upper(a, b):
    if a.open:
        return b
    if b.open:
        return a
    if a.v != b.v:
        return a if a.v < b.v else b
    return a if not a.incl else b


def r_intersection(x, y):
    if r_empty(x) or r_empty(y):
        return None
    r = (_max_lower(x[0], y[0]), _min_upper(x[1], y[1]))
    return None if r_empty(r) else r


def _flip(b):
    return Bound(b.v, not b.incl, b.open)


def r_difference(x, y):
    if not r_overlaps(x, y):
        return [x]
    out = []
    lower = (x[0], _flip(y[0]))
    upper = (_flip(y[1]), x[1])
    if not r_empty(lower):
        out.append(lower)
    if not r_empty(upper):
        out.append(upper)
    return out


def ue_sample_bounds(lo, hi, rate=FPS):
    """FSearchIndexAsset: FirstSampleIdx = CeilToInt(Min * rate), LastSampleIdx = FloorToInt(Max * rate) (float)."""
    a = int(math.ceil(float(F32(lo) * F32(rate))))
    b = int(math.floor(float(F32(hi) * F32(rate))))
    return a, b


# ============================================================================================ schema compilation

class Channel:
    def __init__(self, **kw):
        self.kind = kw["kind"]
        self.cls = kw["cls"]                       # UE class used by CanBeNormalizedWith
        self.comp = kw.get("comp", COMP_XYZ)
        self.bone = kw.get("bone", "root")         # UE bone name, 'root' = root (trajectory)
        self.origin = kw.get("origin", "root")
        self.t_sample = float(kw.get("t_sample", 0.0))
        self.t_origin = float(kw.get("t_origin", 0.0))
        self.vel_space = kw.get("vel_space", VS_NONE)
        self.input_query = kw.get("input_query", IQ_CONTINUING)
        self.weight = float(kw["weight"])
        self.group = kw.get("group") or None
        self.axis = kw.get("axis")                 # heading axis letter of the UE bone ('X'/'Y'/'Z')
        self.flags = kw.get("flags", 0)
        self.outer = kw.get("outer", "")
        self.card = {COMP_XYZ: 3, COMP_XY: 2, COMP_Z: 1}[self.comp]
        self.offset = 0
        self.local_axis = (0.0, 1.0, 0.0)          # filled from rest poses for bone headings
        self.game_bone = -1
        self.game_origin = -1

    @property
    def unitless(self):
        return self.kind in (CK_VELOCITY_DIR, CK_HEADING)

    def label(self):
        strip = {COMP_XYZ: "", COMP_XY: "_xy", COMP_Z: "_z"}[self.comp]
        s = "%s%s%s" % (self.outer, CK_NAMES[self.kind], strip)
        if self.kind == CK_HEADING and self.axis:
            s += "_" + self.axis
        s += "_" + ("Trj" if self.bone == "root" else self.bone)
        if self.origin != "root":
            s += "_" + self.origin
        s += " %.2f" % self.t_sample
        if abs(self.t_origin) > 1e-6:
            s += "-%.2f" % self.t_origin
        return s

    def can_normalize_with(self, o):
        if self is o:
            return True
        if self.card != o.card or self.cls != o.cls:
            return False
        if self.group is not None and self.group == o.group:
            return True
        return self.label() == o.label()


def _none(name):
    return name is None or name == "None" or name == ""


def _bone(ref):
    n = (ref or {}).get("bone_name") if isinstance(ref, dict) else ref
    return "root" if _none(n) else n


def compile_schema(schema):
    """posesearch.json schema -> list of leaf Channel in UE data order (PS Finalize)."""
    out = []

    def strip_of(s):
        return {"NONE": COMP_XYZ, "STRIP_Z": COMP_XY, "STRIP_XY": COMP_Z}[s or "NONE"]

    def iq(s):
        return IQ_CHARACTER if s == "USE_CHARACTER_POSE" else IQ_CONTINUING

    def leaf(ch, outer):
        cls = ch["_class"]
        if cls == "PoseSearchFeatureChannel_Trajectory":
            w = float(ch["weight"])
            for smp in ch.get("samples") or []:
                fl = int(smp["flags"])
                kw = dict(t_sample=float(smp["offset"]), weight=float(smp["weight"]) * w, input_query=IQ_CHARACTER,
                          group=None if _none(smp.get("normalization_group")) else smp["normalization_group"],
                          flags=CHF_TRAJECTORY, outer=outer + "Traj_")
                if fl & (2 | 32):
                    out.append(Channel(kind=CK_POSITION, cls="Position", comp=COMP_XY if fl & 32 else COMP_XYZ, **kw))
                if fl & (1 | 16):
                    out.append(Channel(kind=CK_VELOCITY, cls="Velocity", comp=COMP_XY if fl & 16 else COMP_XYZ,
                                       vel_space=VS_WORLD, **kw))
                if fl & (4 | 64):
                    k2 = dict(kw, flags=kw["flags"] | CHF_CLAMP_UNIT_UE)
                    out.append(Channel(kind=CK_VELOCITY_DIR, cls="Velocity", comp=COMP_XY if fl & 64 else COMP_XYZ,
                                       vel_space=VS_WORLD, **k2))
                if fl & (8 | 128):
                    k2 = dict(kw, flags=kw["flags"] | CHF_NORMALIZE_OUTPUT)
                    out.append(Channel(kind=CK_HEADING, cls="Heading", comp=COMP_XY if fl & 128 else COMP_XYZ,
                                       axis="X", **k2))
            return
        if cls == "PoseSearchFeatureChannel_Group":
            for sc in ch.get("sub_channels") or []:
                leaf(sc, outer + "Grp_")
            return
        if cls == "PoseSearchFeatureChannel_Pose":
            w = float(ch["weight"])
            cs = VS_CHARACTER if ch.get("use_character_space_velocities", True) else VS_WORLD
            for sb in ch.get("sampled_bones") or []:
                fl = int(sb["flags"])
                kw = dict(bone=_bone(sb.get("reference")), weight=float(sb["weight"]) * w,
                          group=None if _none(sb.get("normalization_group")) else sb["normalization_group"],
                          input_query=iq(ch.get("input_query_pose")), outer=outer + "Pose_")
                if fl & 2:
                    out.append(Channel(kind=CK_POSITION, cls="Position", **kw))
                if fl & 4:
                    out.append(Channel(kind=CK_HEADING, cls="Heading", axis="X", flags=CHF_NORMALIZE_OUTPUT, **kw))
                    out.append(Channel(kind=CK_HEADING, cls="Heading", axis="Y", flags=CHF_NORMALIZE_OUTPUT, **kw))
                if fl & 1:
                    out.append(Channel(kind=CK_VELOCITY, cls="Velocity", vel_space=cs, **kw))
                if fl & 8:
                    raise SystemExit("schema %s: Pose channel Phase sub-channel not supported" % short_name(schema["_path"]))
            return
        common = dict(bone=_bone(ch.get("bone")), origin=_bone(ch.get("origin_bone")),
                      t_sample=float(ch.get("sample_time_offset", 0.0)), t_origin=float(ch.get("origin_time_offset", 0.0)),
                      weight=float(ch["weight"]), comp=strip_of(ch.get("component_stripping")),
                      group=None if _none(ch.get("normalization_group")) else ch["normalization_group"],
                      input_query=iq(ch.get("input_query_pose")), outer=outer)
        if ch.get("sampling_attribute_id", -1) not in (-1, None):
            raise SystemExit("sampling attributes not supported")
        if ch.get("permutation_time_type", "USE_SAMPLE_TIME") != "USE_SAMPLE_TIME":
            raise SystemExit("permutation time types other than USE_SAMPLE_TIME not supported")
        if cls == "PoseSearchFeatureChannel_Position":
            if ch.get("normalize_displacement"):
                raise SystemExit("normalize_displacement not supported")
            out.append(Channel(kind=CK_POSITION, cls="Position", **common))
        elif cls == "PoseSearchFeatureChannel_Velocity":
            vs = VS_CHARACTER if ch.get("use_character_space_velocities", True) else VS_WORLD
            if ch.get("normalize"):
                out.append(Channel(kind=CK_VELOCITY_DIR, cls="Velocity", vel_space=vs, flags=CHF_CLAMP_UNIT_UE, **common))
            else:
                out.append(Channel(kind=CK_VELOCITY, cls="Velocity", vel_space=vs, **common))
        elif cls == "PoseSearchFeatureChannel_Heading":
            out.append(Channel(kind=CK_HEADING, cls="Heading", axis=ch.get("heading_axis", "X"),
                               flags=CHF_NORMALIZE_OUTPUT, **common))
        else:
            raise SystemExit("schema %s: channel class %s not supported by the v2 baker" % (short_name(schema["_path"]), cls))

    for ch in schema["channels"]:
        leaf(ch, "")
    off = 0
    for c in out:
        c.offset = off
        off += c.card
        if c.unitless:
            c.flags |= CHF_UNITLESS
        # value depends on the root vertical (database: RTZ root_z; query: capsule vertical history / prediction):
        # the root Z enters as rz(sample) - rz(origin) (positions, character-space velocities with different times) or
        # rz(t) - rz(t - 1/60) (world-space velocities); VelDir XY too, because the clamp uses the 3D length
        moves = abs(c.t_sample - c.t_origin) > 1e-6 or (c.kind in (CK_VELOCITY, CK_VELOCITY_DIR) and c.vel_space == VS_WORLD)
        if c.kind != CK_HEADING and moves and (c.comp != COMP_XY or c.kind == CK_VELOCITY_DIR):
            c.flags |= CHF_Z_FROM_RTZ
    if int(schema.get("number_of_permutations", 1)) != 1 or int(schema.get("sample_rate", FPS)) != FPS:
        raise SystemExit("schema %s: permutations / sample rate %s not supported" % (short_name(schema["_path"]),
                                                                                    schema.get("sample_rate")))
    for c in out:
        for t in (c.t_sample, c.t_origin):
            if abs(t * HALF - round(t * HALF)) > 1e-3:
                raise SystemExit("schema %s: time offset %.4f is not a multiple of 1/60 s" % (short_name(schema["_path"]), t))
    return out, off


def channel_groups(channels):
    """FMeanDeviationCalculator::Add: first group whose first entry CanBeNormalizedWith the channel."""
    groups = []
    for c in channels:
        for g in groups:
            if g[0].can_normalize_with(c):
                g.append(c)
                break
        else:
            groups.append([c])
    return groups


# ============================================================================================ rest-pose axes

def quat_rotate(q, v):
    return v1.qrot(np.asarray(q, float), np.asarray(v, float))


def rig_rest_global(rig_path, nbones):
    rc = load_json(rig_path)["Data"]["RootChunk"]
    parents = [int(p) for p in rc["boneParentIndexes"]][:nbones]
    P = np.zeros((1, nbones, 3))
    Q = np.zeros((1, nbones, 4))
    for b in range(nbones):
        t = rc["boneTransforms"][b]
        P[0, b] = (t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"])
        r = t["Rotation"]
        Q[0, b] = (r["i"], r["j"], r["k"], r["r"])
    Q /= np.linalg.norm(Q, axis=-1, keepdims=True)
    gp, gq = v1.fk_model(P, Q, parents)
    return gp[0], gq[0]


def rig_rest_local(rig_path, nbones):
    """RIGR records: rest local transform of bones 0..nbones-1 as stored in the rig (boneTransforms Translation X/Y/Z,
    Rotation i/j/k/r), float32, not normalized."""
    rc = load_json(rig_path)["Data"]["RootChunk"]
    out = np.zeros((nbones, 7), np.float32)
    for b in range(nbones):
        t = rc["boneTransforms"][b]
        r = t["Rotation"]
        out[b] = (t["Translation"]["X"], t["Translation"]["Y"], t["Translation"]["Z"], r["i"], r["j"], r["k"], r["r"])
    return out


def rig_quantize(t32):
    """Rest translation (float32, metres) -> int32 on the 1 mm grid: floor(double(t) * 1000 + 0.5); also the
    distance (in quanta) of every value from a rounding boundary."""
    x = np.asarray(t32, np.float32).astype(np.float64) * RIG_ID_QUANT_PER_M
    q = np.floor(x + 0.5)
    margin = 0.5 - np.abs(x - q)
    return q.astype(np.int64), margin


def rig_identity_hash(names, parents, rest):
    """FNV-1a 64 over, for each bone: UTF-8 name, 0x00, int32 parent, int32 qx, int32 qy, int32 qz (rig_quantize)."""
    q, margin = rig_quantize(np.asarray(rest)[:, :3])
    if np.abs(q).max() >= 2 ** 31:
        raise SystemExit("rig rest translation out of the int32 1 mm range")
    buf = bytearray()
    for n, p, t in zip(names, parents, q):
        buf += n.encode("utf-8") + b"\x00" + struct.pack("<iiii", int(p), int(t[0]), int(t[1]), int(t[2]))
    return v1.fnv1a64(bytes(buf)), float(margin.min())


def ue_rest_world(skeletons_json, skeleton_name):
    s = load_json(skeletons_json)
    for sk in s["skeletons"]:
        if short_name(sk["path"]) == skeleton_name:
            return {b["name"].lower(): np.array(b["ref_world"], float) for b in sk["bones"]}
    raise SystemExit("skeleton %s not in %s" % (skeleton_name, skeletons_json))


def ue_to_cp_matrix(ue_rest, cp_gp, ue_to_game, game_index):
    """Axis map UE component space -> CP character frame, found from the rest poses (left foot / toe / up)."""
    def v(name):
        return ue_rest[name][:3]
    fl, bl = v("foot_l"), v("ball_l")
    cp_lf = cp_gp[game_index[ue_to_game["foot_l"]]]
    cp_lt = cp_gp[game_index[ue_to_game["ball_l"]]]
    ue_side = np.argmax(np.abs(fl[:2]))
    ue_fwd = 1 - ue_side
    cp_side = int(np.argmax(np.abs(cp_lf[:2])))
    cp_fwd = 1 - cp_side
    M = np.zeros((3, 3))
    M[cp_side, ue_side] = np.sign(cp_lf[cp_side]) * np.sign(fl[ue_side])
    M[cp_fwd, ue_fwd] = np.sign(cp_lt[cp_fwd] - cp_lf[cp_fwd]) * np.sign(bl[ue_fwd] - fl[ue_fwd])
    M[2, 2] = 1.0
    return M


# ============================================================================================ clip sampling

class ClipData:
    """One retargeted clip with a 60 Hz grid of root transforms and character-frame bone transforms."""

    def __init__(self, path, cfg1, names, parents):
        c = load_json(path)
        if c.get("format") != "utppclip" or c.get("version") != 1:
            raise SystemExit(f"{path}: not utppclip v1")
        if int(c["fps"]) != FPS:
            raise SystemExit(f"{path}: fps {c['fps']}")
        if list(c["bone_names"]) != names:
            raise SystemExit(f"{path}: bone_names differ from the rig")
        B = np.array([fr["bones"] for fr in c["frames"]], dtype=np.float64)
        N = B.shape[0]
        if N != int(c["num_frames"]) or N < 4 or not np.all(np.isfinite(B)):
            raise SystemExit(f"{path}: bad frame array")
        self.name = c["source_clip"]
        self.path = path
        self.N = N
        self.P = B[..., 0:3]
        Q = B[..., 3:7].copy()
        qn = np.linalg.norm(Q, axis=-1, keepdims=True)
        if np.abs(qn - 1.0).max() > 1e-3:
            raise SystemExit(f"{path}: quaternion norm")
        self.Q = Q / qn
        self.S = B[..., 7:10]
        bi = {n: i for i, n in enumerate(names)}
        self.ROOT, self.TRAJ = bi[cfg1["bones"]["root"]], bi[cfg1["bones"]["trajectory"]]
        if np.abs(self.P[:, self.ROOT]).max() > 1e-6 or np.abs(self.Q[:, self.ROOT, :3]).max() > 1e-6:
            raise SystemExit(f"{path}: Root bone is not identity")
        if np.abs(self.P[:, self.TRAJ, 2]).max() > 1e-5 or np.abs(self.Q[:, self.TRAJ, :2]).max() > 1e-5:
            raise SystemExit(f"{path}: Trajectory is not planar")
        self.parents = parents
        self.notifies_clip = c.get("notifies", [])
        self.curves_clip = c.get("curves", {})
        rt = c.get("retarget", {})
        self.hip_ratio = float(rt.get("hip_ratio", 1.0))
        self.retarget = rt
        self.rig = c.get("rig")
        # root vertical track (Source/tools/retarget/RTZ.txt): root_z[f] metres, CP +Z up, scale = hip_ratio
        self.root_z = None
        if "root_z" in c:
            rz = np.asarray(c["root_z"], dtype=np.float64)
            d = rt.get("root_z") or {}
            if rz.shape != (N,) or not np.all(np.isfinite(rz)):
                raise SystemExit(f"{path}: root_z is not {N} finite values")
            if d.get("track") != "root_z" or int(d.get("version", 0)) != 1 or d.get("units") != "m" \
                    or abs(float(d.get("scale", float("nan"))) - self.hip_ratio) > 1e-6:
                raise SystemExit(f"{path}: retarget.root_z descriptor missing or not RTZ.txt version 1 (scale = hip_ratio)")
            self.root_z = rz
        yaw = np.unwrap(2.0 * np.arctan2(self.Q[:, self.TRAJ, 2], self.Q[:, self.TRAJ, 3]))
        self.T = np.stack([self.P[:, self.TRAJ, 0], self.P[:, self.TRAJ, 1], yaw], -1)
        gp, _ = v1.fk_model(self.P[:1], self.Q[:1], parents)
        gl, _ = v1.fk_model(self.P[-1:], self.Q[-1:], parents)
        self.pose_delta = float(np.abs(gp[0, 2:] - gl[0, 2:]).max())
        self.ends_match = self.pose_delta <= cfg1["loop"]["max_pose_delta_m"]
        self.name_loop = re.search(cfg1["loop"]["name_regex"], self.name) is not None
        self.loop = False          # set by the baker (UE bLoop for source pack clips, v1 rule otherwise)
        self.length = F32((N - 1) / FPS)
        self.contacts = np.zeros((N, 2))
        for k, cn in enumerate(("contact_l", "contact_r")):
            cv = self.curves_clip.get(cn)
            if cv is not None and len(cv) == N:
                self.contacts[:, k] = cv
        self.grid = None

    # ---- 60 Hz grid ----------------------------------------------------------------------------------------

    def build_grid(self, h_rel_lo, h_rel_hi):
        N, H = self.N, 2 * (self.N - 1)
        # local pose on half frames 0..H (loops: frame N-1 == frame 0, period H)
        Ph = np.empty((H + 1,) + self.P.shape[1:])
        Qh = np.empty((H + 1,) + self.Q.shape[1:])
        Ph[0::2], Qh[0::2] = self.P, self.Q
        a, b = self.Q[:-1], self.Q[1:]
        sgn = np.where(np.sum(a * b, -1, keepdims=True) < 0, -1.0, 1.0)
        qm = a + sgn * b
        Qh[1::2] = qm / np.linalg.norm(qm, axis=-1, keepdims=True)
        Ph[1::2] = 0.5 * (self.P[:-1] + self.P[1:])
        gp, gq = v1.fk_model(Ph, Qh, self.parents)
        self.h_lo = h_rel_lo
        hs = np.arange(h_rel_lo, H + h_rel_hi + 1)
        if self.loop:
            ph = np.mod(hs, H)
        else:
            ph = np.clip(hs, 0, H)
        self.grid = dict(gp=gp, gq=gq, ph=ph, root=self.root_at(hs), rz=self.rz_at(hs))

    def rz_at(self, hs):
        """Root vertical (RTZ root_z, metres) at half-frame indices, same rules as root_at: loops repeat the per-period
        rise (root_z[N-1] - root_z[0]) like UE ExtractRootMotion; non-loops extrapolate the first / last 1/30 s of the
        vertical root motion linearly (ExtrapolateRootMotion translation: whole samples + Blend(Identity, d, frac)).
        Clips without root_z: 0."""
        hs = np.asarray(hs)
        if self.root_z is None:
            return np.zeros(len(hs))
        z = self.root_z
        H = 2 * (self.N - 1)
        zh = np.empty(H + 1)
        zh[0::2] = z
        zh[1::2] = 0.5 * (z[:-1] + z[1:])
        if self.loop:
            q, r = np.divmod(hs, H)
            return zh[r] + q * (z[-1] - z[0])
        out = np.empty(len(hs))
        inside = (hs >= 0) & (hs <= H)
        out[inside] = zh[hs[inside]]
        above = hs > H
        out[above] = z[-1] + 0.5 * (hs[above] - H) * (z[-1] - z[-2])
        below = hs < 0
        out[below] = z[0] + 0.5 * hs[below] * (z[1] - z[0])
        return out

    def _interp_T(self, h):
        k = h // 2
        out = self.T[np.minimum(k, self.N - 1)].copy()
        odd = (h % 2) == 1
        if odd.any():
            out[odd] = 0.5 * (self.T[k[odd]] + self.T[k[odd] + 1])
        return out

    def root_at(self, hs):
        """Root (Trajectory) transform at half-frame indices: wrap for loops, UE extrapolation for non-loops."""
        hs = np.asarray(hs)
        H = 2 * (self.N - 1)
        out = np.empty((len(hs), 3))
        if self.loop:
            q, r = np.divmod(hs, H)
            base = self._interp_T(r)
            D = p_compose(self.T[-1], p_inverse(self.T[0]))
            for qq in np.unique(q):
                sel = q == qq
                out[sel] = p_compose(p_power(D, qq), base[sel]) if qq != 0 else base[sel]
            return out
        inside = (hs >= 0) & (hs <= H)
        out[inside] = self._interp_T(hs[inside])
        above = hs > H
        if above.any():
            delta = p_compose(p_inverse(self.T[-2]), self.T[-1])
            for h in np.unique(hs[above]):
                e = int(h - H)
                n, frac = e // 2, (e % 2) * 0.5
                rem = delta * frac
                out[hs == h] = p_compose(self.T[-1], p_compose(p_power(delta, n), rem))
        below = hs < 0
        if below.any():
            d0inv = p_compose(p_inverse(self.T[1]), self.T[0])
            for h in np.unique(hs[below]):
                e = int(-h)
                n, frac = e // 2, (e % 2) * 0.5
                rem = d0inv * frac
                out[hs == h] = p_compose(self.T[0], p_compose(p_power(d0inv, n), rem))
        return out

    # ---- feature primitives (grid index g = h - h_lo) -------------------------------------------------------

    def anim_pos(self, g, bone):
        G = self.grid
        root = G["root"][g]
        if bone < 0:
            return np.stack([root[:, 0], root[:, 1], G["rz"][g]], -1)
        ch = G["gp"][G["ph"][g], bone]
        xy = rot2(root[:, 2], ch[:, :2]) + root[:, :2]
        return np.concatenate([xy, (ch[:, 2] + G["rz"][g])[:, None]], -1)

    def rel_pos(self, gs, go, bone, origin):
        G = self.grid
        a = self.anim_pos(gs, bone)
        o = self.anim_pos(go, origin)
        d = a - o
        xy = rot2(-G["root"][go][:, 2], d[:, :2])
        return np.concatenate([xy, d[:, 2:3]], -1)

    def heading(self, gs, go, bone, local_axis):
        G = self.grid
        dyaw = G["root"][gs][:, 2] - G["root"][go][:, 2]
        if bone < 0:
            ax = np.broadcast_to(np.asarray(local_axis, float), (len(gs), 3))
        else:
            ax = quat_rotate(G["gq"][G["ph"][gs], bone], np.broadcast_to(np.asarray(local_axis, float), (len(gs), 3)))
        xy = rot2(dyaw, ax[:, :2])
        return np.concatenate([xy, ax[:, 2:3]], -1)


def encode(vec, comp, normalize):
    if comp == COMP_XYZ:
        v = vec
    elif comp == COMP_XY:
        v = vec[:, :2]
    else:
        v = vec[:, 2:3]
    if normalize:
        n2 = np.sum(v * v, -1, keepdims=True)
        v = np.where(n2 > 1e-8, v / np.sqrt(np.maximum(n2, 1e-300)), v)
    return v


def schema_features(clip, channels, dims):
    """Raw features (N, dims) for every frame of the clip (UE FAssetIndexer semantics, UTPP units)."""
    N = clip.N
    h0 = 2 * np.arange(N)
    X = np.zeros((N, dims))
    for c in channels:
        hs = int(round(c.t_sample * HALF))
        ho = int(round(c.t_origin * HALF))
        gs = h0 + hs - clip.h_lo
        go = h0 + ho - clip.h_lo
        if c.kind == CK_POSITION:
            v = clip.rel_pos(gs, go, c.game_bone, c.game_origin)
        elif c.kind in (CK_VELOCITY, CK_VELOCITY_DIR):
            present = clip.rel_pos(gs, go, c.game_bone, c.game_origin)
            past = clip.rel_pos(gs - 1, go - 1 if c.vel_space == VS_CHARACTER else go, c.game_bone, c.game_origin)
            v = (present - past) * HALF
            if c.kind == CK_VELOCITY_DIR:          # GetClampedToMaxSize(1) on the UE (cm/s) vector, before stripping
                u = v * UE_CM
                n = np.linalg.norm(u, axis=-1, keepdims=True)
                v = np.where(n > 1.0, u / np.maximum(n, 1e-300), u)
        else:
            v = clip.heading(gs, go, c.game_bone, c.local_axis)
        X[:, c.offset:c.offset + c.card] = encode(v, c.comp, c.kind == CK_HEADING)
    return X


# ============================================================================================ curves / AnimModifiers

class KeyList:
    """FRichCurve keys with UpdateOrAddKey (tolerance 1e-4); interp 'linear' or 'constant'."""

    def __init__(self):
        self.k = []

    def set(self, t, v, interp="linear"):
        t = float(t)
        for e in self.k:
            if abs(e[0] - t) < KEY_TOL:
                e[1], e[2] = float(v), interp
                return
        self.k.append([t, float(v), interp])
        self.k.sort(key=lambda e: e[0])

    def times(self):
        return np.array([e[0] for e in self.k])

    def evaluate(self, t):
        t = np.asarray(t, float)
        if not self.k:
            return np.zeros_like(t)
        kt = np.array([e[0] for e in self.k])
        kv = np.array([e[1] for e in self.k])
        out = np.interp(t, kt, kv)
        for i, e in enumerate(self.k[:-1]):
            if e[2] == "constant":
                sel = (t >= kt[i]) & (t < kt[i + 1])
                out[sel] = kv[i]
        return out


def sourcepack_curve_eval(cv, t, phase=False):
    kt = np.asarray(cv["t"], float)
    kv = np.asarray(cv["v"], float)
    out = np.interp(t, kt, kv)
    if phase:
        for i in range(len(kt) - 1):
            if kv[i] == -1.0 and kv[i + 1] == 1.0 and kt[i + 1] - kt[i] < 0.0115:
                sel = (t >= kt[i]) & (t < kt[i + 1])
                out[sel] = -1.0
    return out


def mx_speed_curve(A, axis, N):
    """MotionExtractorModifier TranslationSpeed: keys at k/30 (k >= 1) = |delta_axis| * 30; frame 0 = key 1."""
    sel = [i for i, a in enumerate("XYZ") if a in axis]
    d = A[1:, sel] - A[:-1, sel]
    v = np.linalg.norm(d, axis=-1) * FPS
    return np.concatenate([v[:1], v])


GRID_TOL_S = 1e-3                  # key time tolerance of the 1/30 s MotionExtractor grid
SPEED_RANGE_FLOOR_CM = 1.0         # speed curves whose range is below 1 cm/s are compared against 1 cm/s
LAG_MAX = 10                       # cross-correlation search window (frames) of the curve / POSE time-offset check


def keys_on_grid(cv, n):
    """True when the UE keys are the MotionExtractorModifier grid of the CURRENT clip: k/30 for k = 1..n-1
    (MotionExtractorModifier.cpp: first sample skipped; one extra key at the play length from float rounding of
    the last sample time is accepted). Keys off this grid were baked on another version of the clip (cropped or
    resampled after the modifier ran, e.g. M_Neutral_Run_Prism_* key spacing 1/48.6 s): their time base does not
    match POSE."""
    t = np.asarray(cv["t"], float)
    exp = np.arange(1, n) / FPS
    if len(t) == len(exp) + 1 and len(t) >= 2 and abs(t[-1] - t[-2]) < GRID_TOL_S:
        t = t[:-1]
    return len(t) == len(exp) and (not len(t) or float(np.abs(t - exp).max()) < GRID_TOL_S)


def speed_rel_err(rec, ref, floor):
    """MAE / range of the reference speed curve; range = max - min(0, min) (speeds start at 0), at least floor."""
    rng = max(float(np.max(ref) - min(0.0, float(np.min(ref)))), floor)
    return float(np.abs(np.asarray(rec) - np.asarray(ref)).mean()) / rng, rng


def best_lag(a, b, max_lag=LAG_MAX):
    """Shift (frames) maximizing the normalized cross-correlation of a(t + lag) with b(t); 0 for flat curves."""
    a = np.asarray(a, float) - np.mean(a)
    b = np.asarray(b, float) - np.mean(b)
    n = len(a)
    if n < 3 or float(np.abs(a).max()) < 1e-9 or float(np.abs(b).max()) < 1e-9:
        return 0
    best, arg = -np.inf, 0
    for lag in range(-min(max_lag, n - 2), min(max_lag, n - 2) + 1):
        x = a[max(0, lag):n + min(0, lag)]
        y = b[max(0, -lag):n + min(0, -lag)]
        c = float(np.dot(x, y)) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-12)
        if c > best + 1e-12:
            best, arg = c, lag
    return arg


def phase_keys_match_notifies(cv, left, right, tol=2e-3):
    """True when every footstep notify has its Phase key (1 at left, 0 at right times): the curve was baked from
    the CURRENT notifies. False = baked from other notify times (stale), not comparable with a recompute."""
    kt = np.asarray(cv["t"], float)
    kv = np.asarray(cv["v"], float)
    lk, rk = kt[kv == 1.0], kt[kv == 0.0]
    if not len(lk) or not len(rk):
        return False
    return all(float(np.min(np.abs(lk - t))) < tol for t in left) and all(float(np.min(np.abs(rk - t))) < tol for t in right)


def warping_alpha_keys(T, length, p, cm_per_unit):
    """AM_OrientationWarpingAlpha OnApply (pseudo code), root = Trajectory (planar)."""
    keys = KeyList()
    dt = 1.0 / float(p.get("samples_per_second", FPS))
    L = float(length)
    sample_time, idx = 0.0, 0
    last_pos = np.zeros(2)
    last_yaw = 0.0
    ang = (0.0, 0.0)
    last_ang = (0.0, 0.0)
    going = last_going = False
    inv = bool(p.get("invert", False))
    thr_r, thr_t = float(p["rotation_threshold_deg"]), float(p["translation_threshold_deg"])
    bi, bo = float(p["blend_in_s"]), float(p["blend_out_s"])

    def wrap(a):
        return (a + 180.0) % 360.0 - 180.0

    N = len(T)
    while sample_time + 1e-5 < L:
        sample_time = min(max(idx * dt, 0.0), L)
        idx += 1
        k = min(int(round(sample_time * FPS)), N - 1)
        pos = T[k, :2]
        yaw = math.degrees(T[k, 2])
        if idx > 1:
            d = (pos - last_pos) * cm_per_unit
            if float(d @ d) >= 1e-4:               # GetSafeNormal(1e-4) not zero -> MakeRotFromX
                ang = (0.0, math.degrees(math.atan2(d[1], d[0])))
            if idx > 2:
                rot_err = abs(wrap(last_yaw - yaw))
                tr_err = max(abs(wrap(last_ang[0] - ang[0])), abs(wrap(last_ang[1] - ang[1])))
                going = rot_err < thr_r and tr_err < thr_t
                if idx == 4:
                    keys.set(sample_time, float(going ^ inv))
                if going != last_going and idx > 3:
                    keys.set(sample_time - (0.0 if going else bi), 1.0 if (last_going ^ inv) else 0.0)
                    keys.set(sample_time + (bo if going else 0.0), 1.0 if (going ^ inv) else 0.0)
        last_going = going
        last_pos = pos
        last_yaw = yaw
        last_ang = ang
    return keys


def phase_keys(left, right, length, looping, start0, p):
    """AM_BakePhaseCurveFromFootstepNotifies OnApply (pseudo code)."""
    keys = KeyList()
    lv, rv, pre_v, pre_dt = float(p["left_value"]), float(p["right_value"]), float(p["pre_left_value"]), float(p["pre_left_dt_s"])
    L = float(length)

    def add(t, is_left):
        keys.set(t, lv if is_left else rv, "linear")
        if is_left:
            keys.set(t - pre_dt, pre_v, "constant")

    if not left or not right:
        return None
    for t in left:
        add(t, True)
    for t in right:
        add(t, False)
    left_first = left[0] < right[0]
    left_last = left[-1] > right[-1]
    if looping:
        a = right if left_first else left
        add(a[-1] - L, not left_first)
        b = right if left_last else left
        add(b[0] + L, not left_last)
    else:
        if start0:
            add(0.0, not left_first)
        else:
            f1 = (left if left_first else right)[0]
            f2 = (right if left_first else left)[0]
            add(f1 - (f2 - f1), not left_first)
        l1 = (left if left_last else right)[-1]
        l2 = (right if left_last else left)[-1]
        add(l1 + (l1 - l2), not left_last)
    return keys


def phase_cycle(v):
    return np.mod((1.0 - np.asarray(v, float)) * 0.5, 1.0)


def circ_err(a, b):
    d = np.abs(phase_cycle(a) - phase_cycle(b))
    return np.minimum(d, 1.0 - d)


def touchdowns(contact, thr=0.5):
    """Times (s) of rising edges contact >= thr (linear interpolation of the crossing)."""
    out = []
    for i in range(1, len(contact)):
        a, b = contact[i - 1], contact[i]
        if a < thr <= b:
            f = (thr - a) / (b - a) if b != a else 0.0
            out.append((i - 1 + f) / FPS)
    return out


# ============================================================================================ normalization

def norm_dev_for_group(blocks, unitless):
    X = np.concatenate(blocks, axis=0)
    mean = X.mean(axis=0)
    mad = float(np.linalg.norm(X - mean, axis=1).mean())
    thr, repl = DEV_UNITLESS if unitless else DEV_LEN
    return (mad if mad > thr else repl), mad


def compute_norm(channels, dims, raw_blocks):
    """raw_blocks: list of (M_i, dims) raw feature matrices (all indexed poses of every database of the set)."""
    X = np.concatenate(raw_blocks, axis=0)
    dev = np.ones(dims)
    mad_raw = np.zeros(dims)
    for grp in channel_groups(channels):
        blocks = [X[:, c.offset:c.offset + c.card] for c in grp]
        d, m = norm_dev_for_group(blocks, grp[0].unitless)
        for c in grp:
            dev[c.offset:c.offset + c.card] = d
            mad_raw[c.offset:c.offset + c.card] = m
    w = np.zeros(dims)
    for c in channels:
        w[c.offset:c.offset + c.card] = c.weight
    wsum = float(w.sum())
    wn = w / wsum if wsum > 1e-8 else w
    scale = np.sqrt(wn) / dev
    return dict(mean=X.mean(axis=0), dev=dev, mad=mad_raw, weight=w, wnorm=wn, scale=scale, rows=len(X), wsum=wsum)


# ============================================================================================ search (reference)

def box_lower_bound(q, mn, mx):
    d = np.maximum(0.0, np.maximum(mn - q, q - mx))
    return np.sum(d * d, axis=-1, dtype=np.float32)


class DbSearch:
    """Per-database search over the baked (normalized, float32) features: brute force and AABB, identical results."""

    def __init__(self, feats, addend, block, sm, lr):
        self.F = feats
        self.add = addend
        self.block = block
        self.n = len(feats)
        self.sm, self.lr = sm, lr
        self._boxes()

    def _boxes(self):
        n, D = self.F.shape
        self.boxes = {}
        for size in (self.sm, self.lr):
            nb = (n + size - 1) // size
            mn = np.zeros((nb, D), np.float32)
            mx = np.zeros((nb, D), np.float32)
            ma = np.full(nb, np.float32(np.finfo(np.float32).max))
            for b in range(nb):
                sel = ~self.block[b * size:(b + 1) * size]
                rows = self.F[b * size:(b + 1) * size][sel]
                if len(rows):
                    mn[b], mx[b] = rows.min(0), rows.max(0)
                    ma[b] = self.add[b * size:(b + 1) * size][sel].min()
            self.boxes[size] = (mn, mx, ma)

    def costs(self, q, rows):
        d = self.F[rows] - q
        return np.sum(d * d, axis=1, dtype=np.float32) + self.add[rows]

    def brute(self, q, excluded=None):
        ok = ~self.block
        if excluded is not None:
            ok = ok & ~excluded
        rows = np.nonzero(ok)[0]
        if not len(rows):
            return -1, np.float32(np.inf)
        c = self.costs(q, rows)
        i = int(np.argmin(c))
        return int(rows[i]), c[i]

    def aabb(self, q, best_cost=np.float32(np.inf), excluded=None):
        best_i = -1
        best = np.float32(best_cost)
        lmn, lmx, lma = self.boxes[self.lr]
        smn, smx, sma = self.boxes[self.sm]
        llb = box_lower_bound(q, lmn, lmx) + lma
        slb = box_lower_bound(q, smn, smx) + sma
        per = self.lr // self.sm
        for L in range(len(lmn)):
            if not llb[L] < best:
                continue
            for s in range(L * per, min((L + 1) * per, len(smn))):
                if not slb[s] < best:
                    continue
                rows = np.arange(s * self.sm, min((s + 1) * self.sm, self.n))
                ok = ~self.block[rows]
                if excluded is not None:
                    ok &= ~excluded[rows]
                rows = rows[ok]
                if not len(rows):
                    continue
                c = self.costs(q, rows)
                j = int(np.argmin(c))
                if c[j] < best:
                    best, best_i = c[j], int(rows[j])
        return best_i, best


# ============================================================================================ binary writer / reader

def fourcc(s):
    return struct.unpack("<I", s.encode("ascii"))[0]


def write_v2(path, hdr, sections):
    nsec = len(sections)
    if nsec > 64:
        raise SystemExit("too many sections")
    off = HEADER_SIZE + nsec * SECTION_ENTRY_SIZE
    entries, chunks, pos = [], [], off
    for sid, count, esz, data in sections:
        if len(data) != count * esz:
            raise SystemExit(f"section {sid}: size {len(data)} != {count}*{esz}")
        aligned = (pos + ALIGN - 1) // ALIGN * ALIGN
        chunks.append(b"\x00" * (aligned - pos))
        entries.append((fourcc(sid), count, esz, 0, aligned, len(data)))
        chunks.append(data)
        pos = aligned + len(data)
    file_size = (pos + ALIGN - 1) // ALIGN * ALIGN
    chunks.append(b"\x00" * (file_size - pos))
    table = b"".join(struct.pack("<IIIIQQ", *e) for e in entries)
    payload = table + b"".join(chunks)
    pcrc = zlib.crc32(payload) & 0xFFFFFFFF
    h = bytearray(HEADER_SIZE)
    struct.pack_into("<8sII", h, 0, MAGIC, FORMAT_VERSION, HEADER_SIZE)
    struct.pack_into("<QQI", h, 16, file_size, hdr["rig_hash"], hdr["flags"])
    struct.pack_into("<11I", h, 36, hdr["num_frames"], hdr["num_bones"], hdr["num_clips"], hdr["num_schemas"],
                     hdr["num_channels"], hdr["num_norms"], hdr["num_databases"], hdr["num_entries"], hdr["num_rows"],
                     hdr["num_curves"], hdr["num_events"])
    struct.pack_into("<fIII", h, 80, float(hdr["fps"]), nsec, HEADER_SIZE, pcrc)
    struct.pack_into("<IIIQQ", h, 100, hdr["bound_sm_size"], hdr["bound_lr_size"], hdr["max_dims"],
                     hdr["dfea_floats"], hdr["dbox_floats"])
    struct.pack_into("<I", h, 96, zlib.crc32(bytes(h)) & 0xFFFFFFFF)
    data = bytes(h) + payload
    assert len(data) == file_size
    with open(path, "wb") as f:
        f.write(data)
    return data


def read_v2(path_or_bytes):
    """Minimal reader (numpy views) used by the search reference / summary. The tests use their own reader."""
    b = open(path_or_bytes, "rb").read() if isinstance(path_or_bytes, str) else path_or_bytes
    magic, ver, hsz = struct.unpack_from("<8sII", b, 0)
    if magic != MAGIC or ver != FORMAT_VERSION or hsz != HEADER_SIZE:
        raise ValueError("not a utppmm v2 file")
    keys = ("num_frames", "num_bones", "num_clips", "num_schemas", "num_channels", "num_norms", "num_databases",
            "num_entries", "num_rows", "num_curves", "num_events")
    h = dict(zip(keys, struct.unpack_from("<11I", b, 36)))
    h["fps"], h["nsec"], h["table"], h["pcrc"] = struct.unpack_from("<fIII", b, 80)
    h["sm"], h["lr"], h["max_dims"], h["dfea_floats"], h["dbox_floats"] = struct.unpack_from("<IIIQQ", b, 100)
    secs = {}
    for i in range(h["nsec"]):
        sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, h["table"] + SECTION_ENTRY_SIZE * i)
        secs[struct.pack("<I", sid).decode()] = (cnt, esz, off, size)
    return b, h, secs


# ============================================================================================ chooser data file

def export_chooser(cfg2, root_dir, db_index, out_path, db_file_name, log, slide=None):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import motion_chooser_ref as gcr
    ch = cfg2["chooser"]
    cpath = os.path.join(root_dir, ch["choosers_json"])
    ref = gcr.ChooserRef(cpath)
    root = ref.find_root(ch["root"])
    tables = ref.reachable(root)
    tidx = {id(t): i for i, t in enumerate(tables)}
    binds = ref.bindings(root)
    keys = sorted(binds)
    inputs = []
    for k in keys:
        b = binds[k]
        e = {"key": k, "kind": b["kind"], "property_type": b.get("prop_type"), "display": sorted(b.get("display") or [])}
        c0 = b["columns"][0]
        if b["kind"] == "enum" and c0.enum is not None:
            e["enum"] = {"name": c0.enum.name, "values": [{"value": int(v), "name": i, "display": d}
                                                          for v, i, d in c0.enum.entries]}
        inputs.append(e)
    overrides = {(o["table"], int(o["row"])): o for o in ch.get("row_overrides", [])}
    applied = []

    def res(r):
        if r is None:
            return None
        if r.kind == "asset":
            nm = short_name(r.target)
            return {"kind": "database", "name": nm, "db": db_index.get(nm, -1)}
        if r.kind in ("evaluate", "nested"):
            t = ref.resolve(r)
            return {"kind": "table", "table": tidx[id(t)] if t is not None else -1}
        return {"kind": "invalid"}

    out_tables = []
    for t in tables:
        cols = []
        for c in t.columns:
            if not c.has_filters:
                continue
            cd = {"input": keys.index(c.key) if c.key in keys else -1, "key": c.key, "kind": c.kind,
                  "disabled": bool(c.disabled)}
            if c.kind == "float":
                cd.update(wrap=bool(c.wrap), min_value=c.min_value, max_value=c.max_value)
            cd["cells"] = [list(map(lambda x: float(x) if isinstance(x, (float, np.floating)) else x, r))
                           if isinstance(r, tuple) else r for r in c.rows]
            cols.append(cd)
        rows = []
        for i, r in enumerate(t.results):
            ov = overrides.get((t.label, i))
            disabled = bool(t.disabled[i]) or bool(ov and ov["action"] == "disable")
            if ov:
                applied.append({"table": t.label, "row": i, "action": ov["action"], "reason": ov.get("reason")})
            rows.append({"disabled": disabled, "result": res(r)})
        out_tables.append({"label": t.label, "columns": cols, "rows": rows,
                           "fallback": res(t.fallback) if t.fallback is not None else None})
    missing = sorted({r["result"]["name"] for t in out_tables for r in t["rows"]
                      if r["result"] and r["result"]["kind"] == "database" and r["result"]["db"] < 0})
    data = {
        "format": "utpp-chooser-table", "version": 1,
        "licence": "Licensed animation data",
        "generator": "Source/tools/mmbake/mmbake_motion.py (tables parsed by motion_chooser_ref.py)",
        "database_file": db_file_name,
        "semantics": {"mode": "AllResults: union of the databases of every matching row; FirstResult not used",
                      "ranges": "closed [min, max]; no_min / no_max open a side; bounds float32",
                      "bool_cells": "MatchAny | MatchTrue | MatchFalse",
                      "enum_cells": "[comparison, value] with comparison MatchEqual | MatchNotEqual | MatchAny",
                      "results": "database: db = DBAS index in database_file (-1 = not baked); table: index into tables",
                      "fallback": "evaluated only when no row of that table succeeded",
                      "root": "tables[0]"},
        "source": {"choosers_json": ch["choosers_json"], "sha256": v1.sha256_file(cpath), "root": ch["root"]},
        "inputs": inputs, "tables": out_tables, "row_overrides_applied": applied,
        "databases_not_in_file": missing,
    }
    if slide is not None:           # optional slide model: all new fields are additive (FORMAT.txt, chooser slide section)
        import slide_spec as sspec
        sspec.apply_chooser(data, slide, db_index)
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
        f.write("\n")
    log(f"chooser table: {len(out_tables)} tables, {len(inputs)} inputs, overrides applied {len(applied)}, "
        f"databases referenced but not baked {len(missing)} -> {out_path}")
    return data


# ============================================================================================ bake

def _sourcepack_modifiers(rec):
    out = {}
    for u in (rec.get("props") or {}).get("asset_user_data") or []:
        for m in u.get("animation_modifier_instances") or []:
            out[m["_class"]] = m
    return out


def _notify_state_active(notifies, t, length):
    """Notify states overlapping the 1/120 s window centred on min(t, length)."""
    tt = min(float(t), float(length))
    a, b = tt - 0.5 * NOTIFY_WINDOW, tt + 0.5 * NOTIFY_WINDOW
    return [n for n in notifies if n["time"] <= b and n["time"] + n["duration"] > a]


def bake_v2(args, log):
    root = v1.project_root()
    cfg1 = load_json(args.config)
    cfg2_path = args.v2_config
    cfg2 = load_json(cfg2_path)
    nb = int(cfg1["num_pose_bones"])
    names, parents = v1.load_rig(args.rig, nb)
    rh_topo = v1.rig_hash(names, parents)           # v1 rig hash (names + parents): equal for player_man / woman
    rig_name = os.path.basename(args.rig).split(".")[0]
    rig_rest = rig_rest_local(args.rig, nb)
    rh, rig_margin = rig_identity_hash(names, parents, rig_rest)
    log(f"rig {rig_name}: topology hash 0x{rh_topo:016x}, identity hash (header rig_hash) 0x{rh:016x}, "
        f"min distance of a quantized rest translation from a rounding boundary {rig_margin:.4f} quanta")
    if rig_margin < RIG_ID_MIN_MARGIN:
        log.warn(f"rig {rig_name}: a rest translation lies within {RIG_ID_MIN_MARGIN} quanta of a 1 mm rounding boundary")
    gdir = args.sourcepack_dir or os.path.join(root, cfg2["sources"]["sourcepack_dir"])
    ps = load_json(os.path.join(gdir, "posesearch.json"))
    anims = {r["name"]: r for r in load_json(os.path.join(gdir, "anims.json"))["anims"]}
    acurves = load_json(os.path.join(gdir, "anim_curves.json"))["curves"]
    cmap = load_json(args.clip_map or os.path.join(root, cfg2["sources"]["clip_map"]))
    bonemap = load_json(args.bonemap or os.path.join(root, cfg2["sources"]["bonemap"]))
    nc = cfg2["notify_classes"]
    log(f"v2 config: {os.path.relpath(cfg2_path, root)} sourcepack_dir={os.path.relpath(gdir, root)}")

    # ---- bones UE -> game
    bi = {n: i for i, n in enumerate(names)}
    ue_to_game = {e["src"].lower(): e["cp"] for e in bonemap["bones"]}

    def game_bone(ue_name):
        if ue_name == "root":
            return -1
        g = ue_to_game.get(ue_name.lower())
        if g is None or g not in bi:
            raise SystemExit(f"UE bone {ue_name} has no game bone in the bonemap")
        return bi[g]

    # ---- optional source pack slide extension (--slide-spec): extra schemas / databases / normalization sets merged
    #      into the parsed posesearch.json; the extra databases are appended AFTER the sorted base list, so the ids
    #      of every base database are those of a bake without the slide spec
    slide, slide_dbs, slide_schemas, slide_windows = None, [], [], {}
    if getattr(args, "slide_spec", None):
        import slide_spec as sspec
        slide = load_json(args.slide_spec)
        slide_dbs, slide_schemas, slide_windows = sspec.merge_into_posesearch(ps, slide, os.path.join(root, slide["dump_dir"]))
        log(f"slide spec {os.path.relpath(args.slide_spec, root)}: databases {slide_dbs}, schemas {slide_schemas}")

    # ---- databases (family from the data file, list from the clip map)
    fam = cfg2["databases"]["family"]
    db_names = sorted(d["db"] for d in cmap[cfg2["databases"]["clip_map_list"]]) + slide_dbs
    ps_db = {short_name(d["_path"]): d for d in ps["databases"]}
    ps_schema = {short_name(s["_path"]): s for s in ps["schemas"]}
    ps_nset = {short_name(n["_path"]): [short_name(x) for x in n["databases"]] for n in ps["normalization_sets"]}
    for d in db_names:
        if d not in ps_db:
            raise SystemExit(f"database {d} of the clip map not in posesearch.json")
    if len(db_names) > 64:
        raise SystemExit("more than 64 databases")
    db_index = {d: i for i, d in enumerate(db_names)}
    schema_names = sorted({short_name(ps_db[d]["schema"]) for d in db_names if d not in slide_dbs}) + slide_schemas
    schemas = {}
    for s in schema_names:
        chans, dims = compile_schema(ps_schema[s])
        for c in chans:
            c.game_bone = game_bone(c.bone)
            c.game_origin = game_bone(c.origin)
        schemas[s] = (chans, dims)

    # heading local axes from the rest poses (UE pelvis axes -> CP bone frame)
    ue_rest = ue_rest_world(os.path.join(gdir, "skeletons.json"), cfg2["sources"]["ue_skeleton"])
    cp_gp, cp_gq = rig_rest_global(args.rig, nb)
    M = ue_to_cp_matrix(ue_rest, cp_gp, ue_to_game, bi)
    axis_info = {}
    for s, (chans, _) in schemas.items():
        for c in chans:
            if c.kind != CK_HEADING:
                continue
            if c.game_bone < 0:
                c.local_axis = (0.0, 1.0, 0.0)     # any horizontal axis: costs and MAD are invariant (yaw only)
                continue
            ue_q = ue_rest[c.bone.lower()][3:7]
            e = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[c.axis]
            a_cp = M @ quat_rotate(ue_q, np.array(e, float))
            q = cp_gq[c.game_bone]
            l = quat_rotate(np.array([-q[0], -q[1], -q[2], q[3]]), a_cp)
            l = l / np.linalg.norm(l)
            c.local_axis = tuple(float(x) for x in l)
            axis_info[f"{c.bone}.{c.axis}"] = [round(x, 6) for x in l]
    log(f"schemas: " + ", ".join(f"{s}={schemas[s][1]}d/{len(schemas[s][0])}ch" for s in schema_names)
        + f"; heading local axes (game bone frame): {axis_info}")

    # ---- clips
    def dir_clips(spec):
        return sorted(f for d in v1.split_dirs(spec) for f in glob.glob(os.path.join(d, "*.utppclip.json")))

    def stem(f):
        return os.path.basename(f)[:-len(".utppclip.json")]

    def no_dups(fs, what):
        seen = {}
        for f in fs:
            seen[stem(f)] = seen.get(stem(f), 0) + 1
        dup = sorted(k for k, v in seen.items() if v > 1)
        if dup:
            raise SystemExit(f"duplicate clip files across {what} directories: {dup[:5]}")

    cm_clip = {c["name"]: c for c in cmap["clips"]}
    cm_fem = {c["name"]: c for c in cmap.get("female_clips", [])}
    files = dir_clips(args.clips)
    no_dups(files, "--clips")
    if getattr(args, "sourcepack_only", False):
        files = [f for f in files if stem(f) not in cm_fem]
    if getattr(args, "clip_names_from", None):
        keep = {c["name"] for c in load_json(args.clip_names_from)["clips"]}
        files = [f for f in files if stem(f) in keep]
    by_name = {stem(f): f for f in files}
    # ---- optional authored clips (--extra-assets): clip files added by name, entries appended to EXISTING databases below
    extra, extra_files = None, {}
    if getattr(args, "extra_assets", None):
        import extra_assets as xa
        extra = xa.load_spec(args.extra_assets)
        extra_files = xa.resolve_clips(extra, root, set(by_name) | set(cm_clip) | set(cm_fem))
        by_name.update(extra_files)
        log(f"extra assets {os.path.relpath(args.extra_assets, root)}: clips {sorted(extra_files)}")
    # --rtz-clips: RTZ re-retarget of some clips (root vertical in root_z, RTZ.txt) replaces the --clips file
    rtz_files = {}
    rtz_unmatched = []
    if getattr(args, "rtz_clips", None):
        rf = dir_clips(args.rtz_clips)
        no_dups(rf, "--rtz-clips")
        for f in rf:
            if stem(f) in by_name:
                rtz_files[stem(f)] = f
            else:
                rtz_unmatched.append(stem(f))
    log(f"clip files: --clips {len(dir_clips(args.clips))}, "
        f"--rtz-clips replacing a clip file {len(rtz_files)} (RTZ files without a clip file: {len(rtz_unmatched)}), "
        f"sourcepack_only={bool(getattr(args, 'sourcepack_only', False))}, rig {rig_name}")
    # clip-map policies depend on where the retargeted motion comes from (data file motion_sources): with the source pack 5.8
    # export (Animations/retarget/out) the 8 P_MOTION clips are excluded, P_NOTIFY_TIMES uses the source pack 5.8 notify
    # times and P_MOVEDATA recomputes MoveData_Speed; with the source pack export (out_sourcepack_woman / _man) motion,
    # notifies and curves come from the same asset, so no policy override applies.
    ms_all = cfg2["motion_sources"]
    motion_src = getattr(args, "motion_src", None) or ms_all["default"]
    if motion_src not in ms_all or motion_src == "default":
        raise SystemExit(f"--motion-src {motion_src}: not in the data file motion_sources {sorted(ms_all)}")
    msrc = ms_all[motion_src]
    excl_pol = set(msrc.get("exclude_policies", []))
    g58_pol = set(msrc.get("notify_src_gasp58_policies", []))
    recompute_pol = msrc.get("recompute_curves_policies", {})
    log(f"motion source: {motion_src} (exclude policies {sorted(excl_pol)}, Neutral 5.8 notify policies {sorted(g58_pol)}, "
        f"forced curve recompute {recompute_pol})")

    # entries per database: source pack entries from posesearch.json, female entries from the clip map families
    entries = []          # dict(db, clip, enabled, disable_reselection, looping, range, added)
    skipped = {"policy": [], "missing_clip": [], "disabled": 0}
    for d in db_names:
        for e in ps_db[d]["entries"]:
            cn = short_name(e["asset"])
            if not e.get("enabled", True):
                skipped["disabled"] += 1
                continue
            if e.get("asset_class") not in (None, "AnimSequence"):
                raise SystemExit(f"{d}: entry {cn} is a {e.get('asset_class')} (not supported)")
            if e.get("mirror_option", "UnmirroredOnly") != "UnmirroredOnly":
                raise SystemExit(f"{d}: mirrored entry {cn} not supported")
            pol = (cm_clip.get(cn) or {}).get("policy_id")
            if pol in excl_pol:
                skipped["policy"].append(f"{d}:{cn}")
                continue
            if cn not in by_name:
                skipped["missing_clip"].append(f"{d}:{cn}")
                continue
            entries.append(dict(db=d, clip=cn, disable_reselection=bool(e.get("disable_reselection", False)),
                                looping=None if ("looping" in e and e["looping"] is None) else bool(e.get("looping", False)), range=tuple(float(x) for x in e["sampling_range"]),
                                added=False))
    sourcepack_dr = {}
    for e in entries:
        sourcepack_dr.setdefault(e["db"], []).append(e["disable_reselection"])
    for fc in cmap.get("female_clips", []):
        if fc.get("status") != "assigned" or fc["name"] not in by_name:
            continue
        for d in fc.get("families", []):
            if d not in db_index:
                continue
            votes = sourcepack_dr.get(d, [True])
            entries.append(dict(db=d, clip=fc["name"], disable_reselection=sum(votes) * 2 >= len(votes),
                                looping=None, range=(0.0, 0.0), added=True))
    if extra is not None:
        entries += xa.make_entries(extra, extra_files, db_index, sourcepack_dr)
    used = sorted({e["clip"] for e in entries})
    unused_files = sorted(set(by_name) - set(used))
    log(f"entries: {len(entries)} ({sum(1 for e in entries if e['added'])} non-Neutral) over {len(used)} clips; "
        f"skipped by policy {len(skipped['policy'])}, clip file missing {len(skipped['missing_clip'])}, "
        f"disabled {skipped['disabled']}; clip files without any entry (not baked): {len(unused_files)}")
    for s in skipped["missing_clip"][:20]:
        log.warn(f"entry clip not found in --clips: {s}")

    # ---- load clips, loop flags, grids
    h_lo = min(min(int(round(c.t_sample * HALF)), int(round(c.t_origin * HALF))) for s in schemas.values() for c in s[0]) - 1
    h_hi = max(max(int(round(c.t_sample * HALF)), int(round(c.t_origin * HALF))) for s in schemas.values() for c in s[0])
    clips = []
    for cn in used:
        if cn in rtz_files:
            cl = ClipData(rtz_files[cn], cfg1, names, parents)
            if cl.root_z is None:
                raise SystemExit(f"{rtz_files[cn]}: --rtz-clips file has no root_z track")
            n_base = int(load_json(by_name[cn])["num_frames"])
            if n_base != cl.N:
                raise SystemExit(f"{cn}: RTZ clip has {cl.N} frames, the --clips file {n_base}")
        else:
            cl = ClipData(by_name[cn], cfg1, names, parents)
        if cl.rig is not None and cl.rig != rig_name:
            raise SystemExit(f"{cl.path}: retargeted for rig {cl.rig}, the bake rig is {rig_name}")
        rec = anims.get(cn)
        cl.sourcepack = rec is not None and cn in cm_clip and not cn in cm_fem
        cl.rec = rec if cl.sourcepack else None
        cl.policy = (cm_clip.get(cn) or {}).get("policy_id")
        if cl.sourcepack:
            if bool(msrc.get("clips_keep_all_curves")) != ("curve_policy" in cl.retarget):
                log.warn(f"{cn}: clip file does not look like motion source {motion_src} "
                         f"(retarget.curve_policy {'missing' if 'curve_policy' not in cl.retarget else 'present'})")
            # frame convention: UE num_frames counts intervals, the clip holds num_frames + 1 keys (frame f of POSE =
            # UE frame f, t = f / 30); anything else would shift every curve and notify against POSE
            if int(rec.get("num_frames", cl.N - 1)) + 1 != cl.N:
                raise SystemExit(f"{cn}: clip has {cl.N} frames, UE asset {rec.get('num_frames')} + 1 keys")
            cl.loop = bool(rec["props"].get("loop", False))
            cl.length = F32(rec["length_s"])
            if abs(float(cl.length) - (cl.N - 1) / FPS) > 1e-3:
                log.warn(f"{cn}: UE length {float(cl.length):.4f} s != clip {(cl.N - 1) / FPS:.4f} s")
            if cl.loop and not cl.ends_match:
                log.warn(f"{cn}: UE bLoop but first/last pose differ by {cl.pose_delta:.4f} m (wrapped as UE does)")
        else:
            cl.loop = cl.name_loop and cl.ends_match
        cl.build_grid(h_lo, h_hi)
        clips.append(cl)
    clips.sort(key=lambda c: c.name)
    cidx = {c.name: i for i, c in enumerate(clips)}
    for e in entries:
        if e["looping"] is None:
            e["looping"] = clips[cidx[e["clip"]]].loop
        elif e["looping"] != clips[cidx[e["clip"]]].loop:
            raise SystemExit(f"{e['clip']}: entry looping flag differs from the asset flag")
    first = 0
    for c in clips:
        c.first = first
        first += c.N
    NF = first

    # ---- notifies per clip (UE classes), source per clip-map policy
    def notifies_for(c):
        if not c.sourcepack:
            return []
        ue = [dict(time=float(n["time"]), duration=float(n.get("duration", 0.0)), cls=n.get("class") or "",
                   track=n.get("track") or "", props=n.get("props") or {}) for n in c.rec.get("notifies") or []]
        if c.policy in g58_pol:
            out = []
            for n in c.notifies_clip:
                cls = n.get("class") or ""
                cand = [u for u in ue if u["cls"] == cls]
                props = min(cand, key=lambda u: abs(u["time"] - n["trigger_time"]))["props"] if cand else {}
                out.append(dict(time=float(n["trigger_time"]), duration=float(n.get("duration", 0.0)), cls=cls,
                                track="", props=props))
            # footstep tracks are not in the source pack 5.8 meta: keep the source pack ones for Phase / events
            out += [u for u in ue if u["track"] and u["cls"] not in sum((nc[k] for k in ("exclude", "block", "branch_in",
                                                                                         "modify_cost", "override_continuing")), [])]
            return out
        return ue

    for c in clips:
        c.notifies = notifies_for(c)
        c.ps = {k: [n for n in c.notifies if n["cls"] in nc[k]] for k in ("exclude", "block", "branch_in", "modify_cost",
                                                                         "override_continuing", "sampling_range")}
        for kind, wins in (slide_windows.get(c.name) or {}).items():       # authored windows of the slide spec
            for lo, hi in wins:
                c.ps[kind].append(dict(time=float(lo), duration=float(min(hi, (c.N - 1) / FPS) - lo), cls=nc[kind][0],
                                       track="", props={}))
        if c.ps["sampling_range"]:
            raise SystemExit(f"{c.name}: PoseSearchSamplingRange notify states not supported")

    # ---- raw features per schema for every frame
    raw = {}
    for s, (chans, dims) in schemas.items():
        raw[s] = np.concatenate([schema_features(c, chans, dims) for c in clips], axis=0)
        if not np.all(np.isfinite(raw[s])):
            raise SystemExit(f"schema {s}: non-finite features")
    log(f"raw features: frames={NF} " + ", ".join(f"{s}:{raw[s].shape[1]}" for s in schema_names))

    # ---- RTZ (root vertical, Source/tools/retarget/RTZ.txt): clips with a root_z track (re-retarget,
    #      --rtz-clips) -> RTZ_PRESENT; else 'unknown' (RTZ 0, not valid) for clips whose UE root track has a vertical
    #      component or that sit in a database whose schema reads the 3D trajectory (jumps / lands); else RTZ_NONE
    #      (no vertical root, 0 is the true value). Raw features already use root_z (ClipData.rz_at).
    tolz = float(cfg2["jump_vertical"]["root_z_tolerance_cm"])
    traj3d = {s: any(c.flags & CHF_TRAJECTORY and c.kind in (CK_POSITION, CK_VELOCITY) and c.comp != COMP_XY
                     for c in schemas[s][0]) for s in schema_names}
    rtz_end_mismatch = []
    for c in clips:
        needs = False
        if c.sourcepack:
            zs = float(c.rec["root_track_start"]["translation"]["z"])
            ze = float(c.rec["root_track_end"]["translation"]["z"])
            in3d = any(traj3d[short_name(ps_db[e["db"]]["schema"])] for e in entries if e["clip"] == c.name)
            needs = abs(zs) > tolz or abs(ze) > tolz or in3d
            if c.root_z is not None:
                # root_z = hip_ratio * source root Z (RTZ.txt) -> must meet the UE root track ends (cm), 1 cm tolerance
                err = max(abs(c.root_z[0] - c.hip_ratio * zs / UE_CM), abs(c.root_z[-1] - c.hip_ratio * ze / UE_CM))
                if err > 0.01:
                    rtz_end_mismatch.append([c.name, round(err, 4)])
        c.rtz_src = RTZ_SRC_ROOT_Z if c.root_z is not None else (RTZ_SRC_UNKNOWN if needs else RTZ_SRC_NONE)
        c.rtz_unknown = c.rtz_src == RTZ_SRC_UNKNOWN
    n_rtz = sum(1 for c in clips if c.rtz_src == RTZ_SRC_ROOT_Z)
    log(f"RTZ: root_z {n_rtz} clips, unknown {sum(1 for c in clips if c.rtz_unknown)}, none "
        f"{sum(1 for c in clips if c.rtz_src == RTZ_SRC_NONE)}; root_z ends vs UE root track (> 1 cm): {len(rtz_end_mismatch)}")
    unk = [c.name for c in clips if c.rtz_unknown]
    if unk:
        log.warn(f"{len(unk)} clips: vertical root expected but no root_z track (RTZ rows 0, CF_RTZ_UNKNOWN, databases "
                 f"with a 3D trajectory schema get DF_RTZ_PENDING): {', '.join(unk[:8])}{' ...' if len(unk) > 8 else ''}")
    for nm, err in rtz_end_mismatch:
        log.warn(f"{nm}: root_z ends differ from the UE root track ends by {err} m")

    # ---- index entries (UE valid ranges) -> rows
    fflags = np.zeros(NF, np.uint32)
    for c in clips:
        fflags[c.first] |= FF_CLIP_START
        fflags[c.first + c.N - 1] |= FF_CLIP_END
        if c.loop:
            fflags[c.first + c.N - 1] |= FF_LOOP_DUPLICATE
        tt = np.arange(c.N) / FPS
        for n in c.ps["exclude"]:
            sel = (tt >= n["time"] - 1e-6) & (tt <= n["time"] + n["duration"] + 1e-6)
            fflags[c.first:c.first + c.N][sel] |= FF_EXCLUDED_NOTIFY
    db_rows = {d: [] for d in db_names}         # (frame, addend, cont_addend, entry, flags)
    db_entries = {d: [] for d in db_names}
    entries.sort(key=lambda e: (db_index[e["db"]], e["added"], cidx[e["clip"]]))
    for ei, e in enumerate(entries):
        d = e["db"]
        dbr = ps_db[d]
        c = clips[cidx[e["clip"]]]
        L = c.length
        smin, smax = e["range"]
        if smin == 0.0 and smax == 0.0:
            rmin, rmax = F32(0.0), L
        else:
            rmin, rmax = F32(smin), F32(min(float(L), float(F32(smax))))
            if rmin > rmax:
                rmin = rmax
        ranges = [rng(rmin, rmax)]
        if not e["looping"]:
            ex = dbr["exclude_from_database_parameters"]
            xr = (Bound(F32(ex["min"]), True), Bound(F32(L) + F32(ex["max"]), False))   # TRange(A, B) = [A, B)
            r0 = r_intersection(ranges[0], xr)
            ranges = [r0] if r0 is not None else []
        for n in c.ps["exclude"]:
            er = rng(F32(n["time"]), F32(n["time"]) + F32(n["duration"]))
            nr = []
            for r in reversed(ranges):
                nr += r_difference(r, er)
            ranges = nr
        ranges.sort(key=lambda r: float(r[0].v))
        base = float(dbr["base_cost_bias"])
        loopb = float(dbr["looping_cost_bias"]) if e["looping"] else 0.0
        contb = float(dbr["continuing_pose_cost_bias"])
        rows = []
        for r in ranges:
            a, b = ue_sample_bounds(r[0].v, r[1].v)
            for si in range(a, b + 1):
                if si < 0 or si > c.N - 1:
                    raise SystemExit(f"{c.name}: sample {si} outside the clip ({c.N} frames)")
                st = min(float(F32(si) / F32(FPS)), float(L))
                fl = 0
                if _notify_state_active(c.ps["block"], st, L):
                    fl |= RF_BLOCK
                if _notify_state_active(c.ps["branch_in"], st, L):
                    fl |= RF_BRANCH_IN
                if e["looping"]:
                    fl |= RF_LOOPING
                add = base
                for n in _notify_state_active(c.ps["modify_cost"], st, L):
                    add = float(n["props"].get(nc["cost_addend_prop"], add))
                cont = contb
                ov = _notify_state_active(c.ps["override_continuing"], st, L)
                if ov:
                    cont = float(ov[0]["props"].get(nc["cost_addend_prop"], contb))
                rows.append([c.first + si, add + loopb, cont, 0, fl | (RF_RANGE_START if si == a else 0)
                             | (RF_RANGE_END if si == b else 0)])
        e["rows"] = rows
        db_entries[d].append(ei)
        if not e["looping"]:
            ex = dbr["exclude_from_database_parameters"]
            _, last = ue_sample_bounds(0.0, F32(L) + F32(ex["max"]))
            fflags[c.first + max(last + 1, 0):c.first + c.N] |= FF_EXCLUDED_TAIL
    # final entry order: by database, then as sorted above
    for ei, e in enumerate(entries):
        for r in e["rows"]:
            r[3] = ei
            fflags[r[0]] |= FF_INDEXED
            if r[4] & RF_BLOCK:
                fflags[r[0]] |= FF_BLOCK_TRANSITION
            else:
                fflags[r[0]] |= FF_SELECTABLE
            if r[4] & RF_BRANCH_IN:
                fflags[r[0]] |= FF_BRANCH_IN
        db_rows[e["db"]] += e["rows"]

    # ---- normalization per (schema, set + main database)
    norms = []
    norm_key = {}
    db_norm = {}
    for d in db_names:
        s = short_name(ps_db[d]["schema"])
        sset = short_name(ps_db[d]["normalization_set"]) if ps_db[d]["normalization_set"] else None
        members = [d] + [x for x in (ps_nset.get(sset, []) if sset else []) if x != d]
        if ps_schema[s]["data_preprocessor"] != "NORMALIZE_WITH_COMMON_SCHEMA":
            raise SystemExit(f"schema {s}: data preprocessor {ps_schema[s]['data_preprocessor']} not supported "
                             f"(only NormalizeWithCommonSchema is used by the baked databases)")
        key = (s, tuple(sorted(members)))
        if key not in norm_key:
            missing = [m for m in members if m not in db_index]
            if missing:
                raise SystemExit(f"normalization of {d}: set members not baked: {missing}")
            blocks = [raw[s][np.array([r[0] for r in db_rows[m]], dtype=np.int64)] for m in members if db_rows[m]]
            nrm = compute_norm(schemas[s][0], schemas[s][1], blocks)
            nrm.update(schema=s, members=sorted(members), set=sset)
            norm_key[key] = len(norms)
            norms.append(nrm)
        db_norm[d] = norm_key[key]
    log(f"normalizations: {len(norms)} (schema, set+main) pairs")

    # ---- per-database normalized features, boxes
    SM, LR = int(cfg1["aabb"]["small"]), int(cfg1["aabb"]["large"])
    v1_horizon = int(round(max(cfg1["trajectory_horizons_s"]) * FPS))      # v1 HORIZON_PAST_END horizon (30 frames)
    tag_bits = cfg2["databases"]["tag_bits"]
    fam_code = cfg2["databases"]["family_codes"][fam]
    dfea, dbox, drow, dbas_meta = [], [], [], []
    feat_off = box_off = row_off = ent_off = 0
    max_dims = 0
    for d in db_names:
        s = short_name(ps_db[d]["schema"])
        chans, dims = schemas[s]
        nrm = norms[db_norm[d]]
        rows = db_rows[d]
        fr = np.array([r[0] for r in rows], dtype=np.int64)
        F = ((raw[s][fr] - nrm["mean"]) * nrm["scale"]).astype(np.float32) if len(fr) else np.zeros((0, dims), np.float32)
        add = np.array([r[1] for r in rows], np.float32)
        block = np.array([bool(r[4] & RF_BLOCK) for r in rows])
        srch = DbSearch(F, add, block, SM, LR)
        boxes = []
        for size in (SM, LR):
            mn, mx, ma = srch.boxes[size]
            boxes.append(np.concatenate([mn, mx, ma[:, None]], axis=1).astype(np.float32))
        sel = ~block
        flags = 0
        if traj3d[s] and any(clips[cidx[entries[ei]["clip"]]].rtz_unknown for ei in db_entries[d]):
            flags |= DF_RTZ_PENDING
        sset = short_name(ps_db[d]["normalization_set"]) if ps_db[d]["normalization_set"] else None
        if sset and d in ps_nset.get(sset, []):
            flags |= DF_IN_NORMALIZATION_SET
        tb = 0
        for tg in ps_db[d].get("tags") or []:
            if tg not in tag_bits:
                raise SystemExit(f"{d}: tag {tg} has no bit in the data file")
            tb |= tag_bits[tg]
        ne = len(db_entries[d])
        # trajectory extrapolation (replaces the v1 HORIZON_PAST_END cut): rows of non-loop clips whose schema samples
        # reach past the clip ends (root extrapolated), and selectable rows that the v1 rule (frame + v1 horizon >
        # N - 1) would have cut
        t_hi = max(int(round(max(c.t_sample, c.t_origin) * HALF)) for c in chans)
        t_lo = min(int(round(min(c.t_sample, c.t_origin) * HALF)) - (1 if c.kind in (CK_VELOCITY, CK_VELOCITY_DIR) else 0)
                   for c in chans)
        n_ext = n_v1cut = 0
        for r in rows:
            c = clips[cidx[entries[r[3]]["clip"]]]
            if c.loop:
                continue
            si = r[0] - c.first
            if 2 * si + t_hi > 2 * (c.N - 1) or 2 * si + t_lo < 0:
                n_ext += 1
            if not (r[4] & RF_BLOCK) and si + v1_horizon > c.N - 1:
                n_v1cut += 1
        meta = dict(name=d, schema=schema_names.index(s), norm=db_norm[d], family=fam_code, tags=tb, flags=flags,
                    base=float(ps_db[d]["base_cost_bias"]), cont=float(ps_db[d]["continuing_pose_cost_bias"]),
                    loop=float(ps_db[d]["looping_cost_bias"]),
                    ex_min=float(ps_db[d]["exclude_from_database_parameters"]["min"]),
                    ex_max=float(ps_db[d]["exclude_from_database_parameters"]["max"]),
                    first_entry=ent_off, num_entries=ne, first_row=row_off, num_rows=len(rows), dims=dims,
                    feat_off=feat_off, box_off=box_off, nsm=len(boxes[0]), nlr=len(boxes[1]),
                    min_add=float(add[sel].min()) if sel.any() else float(np.finfo(np.float32).max),
                    nsel=int(sel.sum()), nblock=int(block.sum()),
                    nent_added=sum(1 for ei in db_entries[d] if entries[ei]["added"]),
                    rows_extrapolated=n_ext, selectable_v1_horizon_cut=n_v1cut)
        dbas_meta.append(meta)
        dfea.append(F.reshape(-1))
        dbox.append(boxes[0].reshape(-1))
        dbox.append(boxes[1].reshape(-1))
        drow += rows
        feat_off += F.size
        box_off += boxes[0].size + boxes[1].size
        row_off += len(rows)
        ent_off += ne
        max_dims = max(max_dims, dims)
    dfea = np.concatenate(dfea).astype("<f4") if dfea else np.zeros(0, "<f4")
    dbox = np.concatenate(dbox).astype("<f4") if dbox else np.zeros(0, "<f4")

    # ---- curves
    curve_names = [c["name"] for c in cfg2["curves"]]
    curve_units = [CU_UNITS[c["unit"]] for c in cfg2["curves"]]
    cindex = {n: i for i, n in enumerate(curve_names)}
    if len(curve_names) > 32:
        raise SystemExit("more than 32 curves")
    mods = cfg2["modifiers"]
    NC = len(curve_names)
    CURV = np.zeros((NF, NC), np.float64)
    CMASK = np.zeros(NF, np.uint32)
    curve_report = {"source": {}, "phase_variant": {}, "female_modifiers": {}}
    # family majority of modifier instances (source pack clips per database)
    fam_mod = {}
    for d in db_names:
        cnt, tot = {}, 0
        for ei in db_entries[d]:
            c = clips[cidx[entries[ei]["clip"]]]
            if not c.sourcepack:
                continue
            tot += 1
            for m in _sourcepack_modifiers(c.rec):
                cnt[m] = cnt.get(m, 0) + 1
        fam_mod[d] = {m: n / max(tot, 1) for m, n in cnt.items()}
    # phase variant per source pack clip (inferred), majority per family
    fam_start0 = {}
    phase_p = next((p for p in mods.values() if p["kind"] == "phase"), None)

    def steps_sourcepack(c):
        lt = sorted(n["time"] for n in c.notifies if n["track"] == phase_p["left_track"])
        rt = sorted(n["time"] for n in c.notifies if n["track"] == phase_p["right_track"])
        return lt, rt

    def infer_phase(c):
        cv = acurves.get(c.rec["path"], {}).get(phase_p["curve"])
        lt, rt = steps_sourcepack(c)
        if cv is None or not lt or not rt:
            return None
        gk = np.asarray(cv["t"], float)
        best = None
        for var in ((False, False), (False, True), (True, False)):
            k = phase_keys(lt, rt, c.length, var[0], var[1], phase_p)
            kt = k.times()
            if len(kt) == len(gk):
                err = float(np.abs(kt - gk).max())
                if best is None or err < best[0]:
                    best = (err, var)
        return best[1] if best and best[0] < 2e-3 else None

    for c in clips:
        if c.sourcepack and phase_p:
            var = infer_phase(c)
            c.phase_var = var
            if var is not None:
                curve_report["phase_variant"][c.name] = "looping" if var[0] else ("start_at_0" if var[1] else "plain")
    for d in db_names:
        v = [clips[cidx[entries[ei]["clip"]]].phase_var for ei in db_entries[d]
             if clips[cidx[entries[ei]["clip"]]].sourcepack and clips[cidx[entries[ei]["clip"]]].phase_var is not None]
        fam_start0[d] = (sum(1 for x in v if x[1]) * 2 > len(v)) if v else False

    default_inst = {}                      # modifier instance properties of the first source pack clip carrying them
    for nm in sorted(anims):
        for mcls, inst in _sourcepack_modifiers(anims[nm]).items():
            default_inst.setdefault(mcls, inst)
    fmin = float(cfg2["female_modifiers"]["min_fraction"])
    cthr = float(cfg2["female_modifiers"]["footsteps_from_contacts"]["threshold"])
    # UE (source skeleton) tracks for the MotionExtractor validation (Source/tools/mmbake/dump_ue_tracks.py): decide
    # whether a UE FootSpeed / MoveData_Speed curve describes the CURRENT source pack motion independently of the retarget
    # geometry (the CP ball bone is not the UE ball bone scaled by hip_ratio).
    ut_path = getattr(args, "ue_tracks", None)
    if ut_path is None and cfg2["sources"].get("ue_tracks"):
        ut_path = os.path.join(root, cfg2["sources"]["ue_tracks"])
    ue_tracks = {}
    if ut_path and os.path.exists(ut_path):
        ue_tracks = load_json(ut_path)["clips"]
        log(f"UE source tracks: {os.path.relpath(ut_path, root)} ({len(ue_tracks)} clips)")
    elif ut_path:
        log.warn(f"UE source tracks {ut_path} not found: MotionExtractor curves are validated on the retargeted motion")
    validation = {"me": [], "warping": [], "phase": [], "lag": []}
    tol_speed, tol_bin, tol_phase = 0.02, 0.98, 0.05      # acceptance tolerances
    recompute_mask = {}
    for c in clips:
        N = c.N
        tfr = np.arange(N) / FPS
        A_world = {}

        def world(ue_bone):
            # UE component space includes the root bone (root motion): planar Trajectory and, for RTZ clips, root_z
            # (the RTZ pose has the root vertical removed from Hips; without it FootSpeed would lose the jump motion)
            if ue_bone not in A_world:
                gb = game_bone(ue_bone)
                zr = c.root_z if c.root_z is not None else np.zeros(N)
                if gb < 0:
                    A_world[ue_bone] = np.stack([c.T[:, 0], c.T[:, 1], zr], -1)
                else:
                    gp = c.grid["gp"][0:2 * (N - 1) + 1:2, gb]
                    A_world[ue_bone] = np.concatenate([rot2(c.T[:, 2], gp[:, :2]) + c.T[:, :2], (gp[:, 2] + zr)[:, None]], -1)
            return A_world[ue_bone]

        def ue_values(cn):
            """UE curve at the frame times: the per-frame UE sample of the clip file (GetFloatValueAtTime at
            frame / fps, exported with the clip) when present, else the anim_curves.json keys."""
            v = c.curves_clip.get(cn.lower())
            if v is not None and len(v) == N:
                return np.asarray(v, float)
            return sourcepack_curve_eval(gcv[cn], tfr, phase=(cn == (phase_p or {}).get("curve")))

        cm_per_unit = UE_CM / c.hip_ratio if c.sourcepack else UE_CM
        fdb = [e["db"] for e in entries if e["clip"] == c.name]
        sourcepack_mods = _sourcepack_modifiers(c.rec) if c.sourcepack else {}
        if c.sourcepack:
            mod_set = set(sourcepack_mods)
        else:
            mod_set = {m for m, frac in fam_mod.get(fdb[0], {}).items() if frac >= fmin} if fdb else set()
            curve_report["female_modifiers"][c.name] = sorted(mod_set)
        gcv = acurves.get(c.rec["path"], {}) if c.sourcepack else {}
        src = {}
        rmask = 0
        forced = set(recompute_pol.get(c.policy, [])) if c.sourcepack else set()
        for mcls, p in mods.items():
            if mcls not in mod_set or p.get("feasible", True) is False:
                continue
            cn = p["curve"]
            if cn not in cindex:
                continue
            k = cindex[cn]
            if p["kind"] == "motion_extractor":
                inst = sourcepack_mods.get(mcls) or default_inst.get(mcls)
                if inst is None:
                    continue
                if inst.get("motion_type") != "TRANSLATION_SPEED" or inst.get("space") != "COMPONENT_SPACE":
                    continue
                rec_v = mx_speed_curve(world(inst["bone_name"]), inst["axis"], N)
                use, why, r_ue = rec_v, "no_ue_curve", None
                if cn in gcv and c.sourcepack:
                    g_ue = ue_values(cn)
                    g = g_ue / cm_per_unit
                    on_grid = keys_on_grid(gcv[cn], N)
                    rel_game, _ = speed_rel_err(rec_v, g, SPEED_RANGE_FLOOR_CM / cm_per_unit)
                    rel_ue = None
                    trk = ue_tracks.get(c.name, {}).get("bones", {})
                    bt = trk.get("__object__" if inst["bone_name"] == "root" else inst["bone_name"])
                    if bt is not None and len(bt) == N:
                        r_ue = mx_speed_curve(np.asarray(bt, float)[:, :3] * UE_CM, inst["axis"], N)
                        rel_ue, _ = speed_rel_err(r_ue, g_ue, SPEED_RANGE_FLOOR_CM)
                    rel = rel_ue if rel_ue is not None else rel_game
                    if not on_grid:
                        why = "ue_keys_off_grid"
                    elif cn in forced:
                        why = "policy_forced"
                    elif rel > tol_speed:
                        why = "ue_curve_not_this_motion" if rel_ue is not None else "recompute_differs"
                    else:
                        use, why = g, "sourcepack"
                    validation["me"].append(dict(clip=c.name, curve=cn, on_grid=on_grid, rel_ue=rel_ue, rel_game=rel_game,
                                                 lag_game=best_lag(rec_v, g), src=why, n=N))
                CURV[c.first:c.first + N, k] = use
                src[cn] = why
                # time offset against POSE: a UE curve is compared with the recompute on the UE motion (frame f of
                # POSE = UE frame f, N == UE keys), a recomputed curve with the retargeted motion (POSE itself)
                ref = r_ue / cm_per_unit if (why == "sourcepack" and r_ue is not None) else rec_v
                validation["lag"].append(dict(clip=c.name, curve=cn, src=why, lag=best_lag(use, ref),
                                              lag_pose=best_lag(use, rec_v)))
            elif p["kind"] == "warping_alpha":
                keys = warping_alpha_keys(c.T, c.length, p, cm_per_unit)
                rec_v = keys.evaluate(tfr)
                use, why = rec_v, "no_ue_curve"
                if cn in gcv and c.sourcepack:
                    g = ue_values(cn)
                    agree = float(np.mean((rec_v >= 0.5) == (g >= 0.5)))
                    validation["warping"].append(dict(clip=c.name, curve=cn, agree=agree, n=N))
                    if agree >= tol_bin:
                        use, why = g, "sourcepack"
                    else:
                        why = "recompute_differs"
                CURV[c.first:c.first + N, k] = use
                src[cn] = why
            elif p["kind"] == "phase":
                if c.sourcepack:
                    lt, rt = steps_sourcepack(c)
                    var = c.phase_var or (False, fam_start0.get(fdb[0], False) if fdb else False)
                else:
                    lt = touchdowns(c.contacts[:, 0], cthr)
                    rt = touchdowns(c.contacts[:, 1], cthr)
                    var = (False, fam_start0.get(fdb[0], False) if fdb else False)
                keys = phase_keys(lt, rt, c.length, var[0], var[1], p)
                if keys is None:
                    continue
                rec_v = keys.evaluate(tfr)
                use, why = rec_v, "no_ue_curve"
                if cn in gcv and c.sourcepack:
                    g = ue_values(cn)
                    e = circ_err(rec_v, g)
                    consistent = phase_keys_match_notifies(gcv[cn], lt, rt)
                    validation["phase"].append(dict(clip=c.name, err=float(e.mean()), max=float(e.max()), n=N,
                                                    consistent=consistent, variant_inferred=c.phase_var is not None))
                    if float(e.mean()) <= tol_phase:
                        use, why = g, "sourcepack"
                    else:
                        why = "recompute_differs" if consistent else "ue_keys_from_other_notify_times"
                CURV[c.first:c.first + N, k] = use
                src[cn] = why
            if src.get(cn) != "sourcepack":
                rmask |= 1 << k
            CMASK[c.first:c.first + N] |= np.uint32(1 << k)
        # authored source pack curves (no modifier)
        for cn, k in cindex.items():
            if cn in src or not c.sourcepack or cn not in gcv:
                continue
            vals = ue_values(cn)
            unit = curve_units[k]
            if unit in (CU_UNITS["m/s"], CU_UNITS["m"]):
                vals = vals / cm_per_unit
            CURV[c.first:c.first + N, k] = vals
            CMASK[c.first:c.first + N] |= np.uint32(1 << k)
            src[cn] = "sourcepack_authored"
        c.curve_src = src
        recompute_mask[c.name] = rmask
        curve_report["source"][c.name] = src

    # ---- events
    ev_rules = [(re.compile(r["regex"]), r["tag"], r["side"]) for r in cfg2["events"]["class_rules"]]
    events = []
    for ci, c in enumerate(clips):
        evs = []
        if c.sourcepack:
            for n in c.notifies:
                for rx, tag, side in ev_rules:
                    m = rx.match(n["cls"])
                    if m:
                        sd = side.format(**m.groupdict()) if side else ""
                        evs.append((n["time"], n["duration"], tag.format(**m.groupdict()), n["cls"], sd, False))
                        break
        else:
            dd = cfg2["events"]["derived_footstep"]
            for k, sd in ((0, "L"), (1, "R")):
                for tt in touchdowns(c.contacts[:, k], cthr):
                    evs.append((tt, 0.0, dd["tag"], dd["class"], sd, True))
        evs.sort(key=lambda x: (x[0], x[2], x[4]))
        c.first_event = len(events)
        c.num_events = len(evs)
        events += [(ci,) + e for e in evs]

    # ---- RTZ rows: root_z and vz = central difference x fps (one-sided at the first / last frame), RTZ.txt
    RTZ = np.zeros((NF, 2), np.float64)
    for c in clips:
        if c.root_z is not None:
            z = c.root_z
            vz = np.empty(c.N)
            vz[1:-1] = (z[2:] - z[:-2]) * FPS / 2.0
            vz[0] = (z[1] - z[0]) * FPS
            vz[-1] = (z[-1] - z[-2]) * FPS
            RTZ[c.first:c.first + c.N, 0] = z
            RTZ[c.first:c.first + c.N, 1] = vz

    # ---- serialize
    st = v1.Strings()
    st.add("")
    bone_b = b"".join(struct.pack("<IiII", st.add(n), p, len(n.encode("utf-8")), 0) for n, p in zip(names, parents))
    clip_b, clp2_b = bytearray(), bytearray()
    for c in clips:
        mask = 0
        for e in entries:
            if e["clip"] == c.name:
                mask |= 1 << db_index[e["db"]]
        fl = (CF_LOOP if c.loop else 0) | (CF_ENDS_POSE_MATCH if c.ends_match else 0) | (CF_NAME_LOOP if c.name_loop else 0) \
            | (CF_RTZ_UNKNOWN if c.rtz_src == RTZ_SRC_UNKNOWN else 0) | (0 if c.sourcepack else CF_ADDED) \
            | (CF_RTZ_PRESENT if c.rtz_src == RTZ_SRC_ROOT_Z else 0) | (CF_RTZ_NONE if c.rtz_src == RTZ_SRC_NONE else 0)
        nsel = int(((fflags[c.first:c.first + c.N] & FF_SELECTABLE) != 0).sum())
        clip_b += struct.pack("<IIIIIIQ", st.add(c.name), c.first, c.N, fl, c.N - 1 if c.loop else 0, nsel, mask)
        cmask = int(CMASK[c.first]) if c.N else 0
        clp2_b += struct.pack("<IIIIffII", c.first_event, c.num_events, c.rtz_src, cmask,
                              float(c.hip_ratio), float(c.length), 0 if c.sourcepack else 1, recompute_mask.get(c.name, 0))
    schm_b, schc_b = bytearray(), bytearray()
    ch_first = 0
    for s in schema_names:
        chans, dims = schemas[s]
        wsum = float(sum(c.weight * c.card for c in chans))
        schm_b += struct.pack("<IIIIfIfI", st.add(s), dims, ch_first, len(chans), float(FPS),
                              PREPROC[ps_schema[s]["data_preprocessor"]], wsum, 0)
        for c in chans:
            schc_b += struct.pack("<IIiiffIIffIIIIfffII", c.kind, c.comp, c.game_bone, c.game_origin, c.t_sample,
                                  c.t_origin, c.vel_space, c.input_query, c.weight, c.weight / wsum if wsum else 0.0,
                                  c.offset, c.card, st.add(c.group) if c.group else 0xFFFFFFFF, st.add(c.label()),
                                  c.local_axis[0], c.local_axis[1], c.local_axis[2], c.flags, 0)
        ch_first += len(chans)
    norm_b, nrmv = bytearray(), []
    vpos = 0
    for n in norms:
        dims = schemas[n["schema"]][1]
        m = 0
        for x in n["members"]:
            m |= 1 << db_index[x]
        norm_b += struct.pack("<IIIIQII", schema_names.index(n["schema"]), dims, vpos, len(n["members"]), m, n["rows"], 0)
        nrmv += [n["mean"], n["scale"], n["dev"]]
        vpos += 3 * dims
    nrmv = np.concatenate(nrmv).astype("<f4")
    dbas_b = bytearray()
    for m in dbas_meta:
        dbas_b += struct.pack("<IIIIIIfffffIIIIIQQIIfI", st.add(m["name"]), m["schema"], m["norm"], m["family"], m["tags"],
                              m["flags"], m["base"], m["cont"], m["loop"], m["ex_min"], m["ex_max"], m["first_entry"],
                              m["num_entries"], m["first_row"], m["num_rows"], m["dims"], m["feat_off"], m["box_off"],
                              m["nsm"], m["nlr"], m["min_add"], m["nsel"])
    dent_b = bytearray()
    row_cursor = 0
    for e in entries:
        fl = EF_ENABLED | (EF_DISABLE_RESELECTION if e["disable_reselection"] else 0) | (EF_LOOPING if e["looping"] else 0) \
            | (EF_ADDED if e["added"] else 0) | (EF_NOTIFY_MOTION58 if clips[cidx[e["clip"]]].policy in g58_pol else 0)
        dent_b += struct.pack("<IIIffIII", db_index[e["db"]], cidx[e["clip"]], fl, e["range"][0], e["range"][1],
                              row_cursor, len(e["rows"]), 0)
        row_cursor += len(e["rows"])
    drow_b = b"".join(struct.pack("<IffII", r[0], r[1], r[2], r[3], r[4]) for r in drow)
    crvn_b = b"".join(struct.pack("<II", st.add(n), u) for n, u in zip(curve_names, curve_units))
    curv = np.zeros((NF, NC + 1), np.float32)
    curv.view(np.uint32)[:, 0] = CMASK
    curv[:, 1:] = CURV.astype(np.float32)
    evnt_b = bytearray()
    for (ci, tt, du, tag, cls, sd, derived) in events:
        fl = (EVF_SIDE_LEFT if sd == "L" else EVF_SIDE_RIGHT if sd == "R" else 0) | (EVF_DERIVED if derived else 0) \
            | (EVF_STATE if du > 0 else 0)
        evnt_b += struct.pack("<IffIII", ci, tt, du, st.add(tag), st.add(cls), fl)
    P = np.concatenate([c.P for c in clips])
    Qa = np.concatenate([c.Q for c in clips])
    S = np.concatenate([c.S for c in clips])
    pose = np.concatenate([P, Qa], axis=-1).astype("<f4")
    scale_dev = float(np.abs(S - 1.0).max())
    has_scale = scale_dev > cfg1["scale_tolerance"]
    rootd = []                             # ROOT as v1: step to the next played frame in the frame of this frame
    for c in clips:
        r = np.zeros((c.N, 4))
        Ta, Tn = c.T.copy(), np.empty_like(c.T)
        Tn[:-1] = c.T[1:]
        if c.loop:
            Tn[-1] = c.root_at(np.array([2 * c.N]))[0]     # frame N-1 == frame 0 -> continue into frame 1
        else:
            Ta[-1], Tn[-1] = c.T[-2], c.T[-1]              # copy of the previous step
        r[:, 0:2] = rot2(-Ta[:, 2], Tn[:, :2] - Ta[:, :2])
        r[:, 2] = Tn[:, 2] - Ta[:, 2]
        rootd.append(r)
    rootd = np.concatenate(rootd).astype("<f4")
    cont = np.concatenate([c.contacts for c in clips]).astype("<f4")
    while len(st.blob) % 4:
        st.blob += b"\x00"
    sections = [
        ("STRS", len(st.blob), 1, bytes(st.blob)),
        ("BONE", nb, 16, bone_b),
        ("RIGR", nb, 28, rig_rest.astype("<f4").tobytes()),
        ("CLIP", len(clips), 32, bytes(clip_b)),
        ("CLP2", len(clips), 32, bytes(clp2_b)),
        ("FFLG", NF, 4, fflags.astype("<u4").tobytes()),
        ("SCHM", len(schema_names), 32, bytes(schm_b)),
        ("SCHC", ch_first, 76, bytes(schc_b)),
        ("NORM", len(norms), 32, bytes(norm_b)),
        ("NRMV", len(nrmv), 4, nrmv.tobytes()),
        ("DBAS", len(dbas_meta), 96, bytes(dbas_b)),
        ("DENT", len(entries), 32, bytes(dent_b)),
        ("DROW", len(drow), 20, drow_b),
        ("DFEA", len(dfea), 4, dfea.tobytes()),
        ("DBOX", len(dbox), 4, dbox.tobytes()),
        ("CRVN", NC, 8, crvn_b),
        ("CURV", NF, 4 * (NC + 1), curv.astype("<f4").tobytes()),
        ("EVNT", len(events), 24, bytes(evnt_b)),
        ("RTZ_", NF, 8, RTZ.astype("<f4").tobytes()),
        ("POSE", NF, nb * 28, pose.tobytes()),
        ("ROOT", NF, 16, rootd.tobytes()),
        ("CONT", NF, 8, cont.tobytes()),
    ]
    if has_scale:
        sections.append(("SCAL", NF, nb * 12, S.astype("<f4").tobytes()))
    hdr = dict(rig_hash=rh, flags=HDR_HAS_SCALE if has_scale else 0, num_frames=NF, num_bones=nb, num_clips=len(clips),
               num_schemas=len(schema_names), num_channels=ch_first, num_norms=len(norms), num_databases=len(db_names),
               num_entries=len(entries), num_rows=len(drow), num_curves=NC, num_events=len(events), fps=FPS,
               bound_sm_size=SM, bound_lr_size=LR, max_dims=max_dims, num_events_clipped=0,
               dfea_floats=len(dfea), dbox_floats=len(dbox))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    data = write_v2(args.out, hdr, sections)
    out_sha = sha256_bytes(data)

    # ---- chooser table next to the database
    chooser = None
    if not getattr(args, "no_chooser", False):
        cpath = args.chooser_out or os.path.splitext(args.out)[0] + ".chooser.json"
        chooser = export_chooser(cfg2, root, db_index, cpath, os.path.basename(args.out), log, slide)

    # ---- report
    sec_sizes = {s[0]: s[1] * s[2] for s in sections}
    n_sel = int(((fflags & FF_SELECTABLE) != 0).sum())
    n_idx = int(((fflags & FF_INDEXED) != 0).sum())
    log(f"clips={len(clips)} frames={NF} databases={len(db_names)} entries={len(entries)} rows={len(drow)} "
        f"schemas={len(schema_names)} channels={ch_first} norms={len(norms)} curves={NC} events={len(events)}")
    log(f"frames indexed={n_idx} selectable={n_sel} block={int(((fflags & FF_BLOCK_TRANSITION) != 0).sum())} "
        f"excluded_notify={int(((fflags & FF_EXCLUDED_NOTIFY) != 0).sum())} excluded_tail={int(((fflags & FF_EXCLUDED_TAIL) != 0).sum())}")
    for m in dbas_meta:
        log(f"db {m['name']:40s} schema={schema_names[m['schema']]:12s} norm={m['norm']:2d} entries={m['num_entries']:3d} "
            f"(non-Neutral {m['nent_added']:2d}) rows={m['num_rows']:5d} selectable={m['nsel']:5d} block={m['nblock']:4d} "
            f"tags={m['tags']} flags={m['flags']} min_addend={m['min_add']:.3f} extrapolated={m['rows_extrapolated']} "
            f"selectable_in_v1_cut={m['selectable_v1_horizon_cut']}")
    v1cut = np.zeros(NF, bool)
    for c in clips:
        if not c.loop:
            v1cut[c.first + max(c.N - 1 - v1_horizon + 1, 0):c.first + c.N] = True
    f_sel = (fflags & FF_SELECTABLE) != 0
    f_idx = (fflags & FF_INDEXED) != 0
    f_xn = (fflags & FF_EXCLUDED_NOTIFY) != 0
    f_tail = (fflags & FF_EXCLUDED_TAIL) != 0
    # exclusive split of the frames the v1 rule cuts (last v1_horizon frames of every non-loop clip), first match wins
    extrap = dict(v1_horizon_frames=v1_horizon, frames_in_v1_cut=int(v1cut.sum()),
                  selectable=int((v1cut & f_sel).sum()),
                  indexed_block_transition=int((v1cut & f_idx & ~f_sel).sum()),
                  excluded_notify=int((v1cut & ~f_idx & f_xn).sum()),
                  excluded_tail=int((v1cut & ~f_idx & ~f_xn & f_tail).sum()),
                  not_indexed_other=int((v1cut & ~f_idx & ~f_xn & ~f_tail).sum()))
    log(f"extrapolation vs the v1 HORIZON_PAST_END cut: {json.dumps(extrap)}")
    for i, n in enumerate(norms):
        log(f"norm {i}: schema={n['schema']} members={len(n['members'])} rows={n['rows']} wsum={n['wsum']:.3f}")
    log("section bytes: " + ", ".join(f"{k}={v}" for k, v in sec_sizes.items()))
    log(f"output {args.out} size={len(data)} sha256={out_sha}")

    val_summary = {}

    def count(v, key):
        out = {}
        for x in v:
            out[x[key]] = out.get(x[key], 0) + 1
        return dict(sorted(out.items()))

    for kind in ("FootSpeed", "MoveData_Speed"):
        v = [x for x in validation["me"] if x["curve"].startswith(kind)]
        if not v:
            continue
        og = [x for x in v if x["on_grid"]]
        rue = np.array([x["rel_ue"] for x in og if x["rel_ue"] is not None])
        rg = np.array([x["rel_game"] for x in og])
        val_summary[kind] = dict(
            curves=len(v), keys_on_grid=len(og), keys_off_grid=len(v) - len(og), source=count(v, "src"),
            ue_motion=dict(curves=len(rue), within_2pct=int((rue <= tol_speed).sum()),
                           rel_median=float(np.median(rue)) if len(rue) else None,
                           rel_p95=float(np.percentile(rue, 95)) if len(rue) else None),
            retargeted_motion=dict(curves=len(rg), within_2pct=int((rg <= tol_speed).sum()),
                                   rel_median=float(np.median(rg)) if len(rg) else None,
                                   rel_p95=float(np.percentile(rg, 95)) if len(rg) else None),
            lag_nonzero_on_grid=sum(1 for x in og if x["lag_game"] != 0),
            worst_ue_motion=[[x["clip"], x["curve"], round(x["rel_ue"], 4)] for x in
                             sorted((x for x in og if x["rel_ue"] is not None), key=lambda x: -x["rel_ue"])[:10]],
            off_grid=sorted({x["clip"] for x in v if not x["on_grid"]}))
    for cn in sorted({x["curve"] for x in validation["warping"]}):
        v = [x for x in validation["warping"] if x["curve"] == cn]
        ag = np.array([x["agree"] for x in v])
        frames = np.array([x["n"] for x in v])
        val_summary[cn] = dict(clips=len(v), frame_agreement=float((ag * frames).sum() / frames.sum()),
                               clips_below_98=int((ag < tol_bin).sum()), min_clip=float(ag.min()),
                               worst=[[x["clip"], round(x["agree"], 4)] for x in sorted(v, key=lambda x: x["agree"])[:10]])
    if validation["phase"]:
        v = validation["phase"]
        cons = [x for x in v if x["consistent"]]
        e = np.array([x["err"] for x in cons])
        fr = np.array([x["n"] for x in cons])
        val_summary["Phase"] = dict(
            clips=len(v), keys_match_notifies=len(cons), keys_from_other_notify_times=len(v) - len(cons),
            consistent_mean_circular_error=float((e * fr).sum() / fr.sum()) if len(e) else None,
            consistent_max_clip_mean=float(e.max()) if len(e) else None,
            consistent_clips_over_0_05=int((e > tol_phase).sum()),
            all_clips_over_0_05=int(sum(1 for x in v if x["err"] > tol_phase)),
            stale=sorted(x["clip"] for x in v if not x["consistent"]))
    if validation["lag"]:
        v = validation["lag"]
        val_summary["curve_time_offset_frames"] = dict(
            curves=len(v), zero=sum(1 for x in v if x["lag"] == 0),
            nonzero=[[x["clip"], x["curve"], x["src"], x["lag"]] for x in v if x["lag"] != 0][:40])
    for k, v in val_summary.items():
        log(f"curve check {k}: {json.dumps(v, default=str)}")

    summary = {
        "format": "utppmm", "version": FORMAT_VERSION, "file": os.path.basename(args.out), "size_bytes": len(data),
        "sha256": out_sha, "rig": rig_name, "rig_hash": f"0x{rh:016x}",
        "rig_identity": {"header_rig_hash": f"0x{rh:016x}", "definition": "FORMAT.txt v2 'Rig identity'",
                         "topology_hash_v1": f"0x{rh_topo:016x}", "rest_quantum_m": 1.0 / RIG_ID_QUANT_PER_M,
                         "min_boundary_margin_quanta": round(rig_margin, 6),
                         "rig_json": os.path.relpath(args.rig, root).replace("\\", "/"),
                         "rig_json_sha256": v1.sha256_file(args.rig)},
        "bake_args": {k: (os.path.relpath(v, root).replace("\\", "/") if isinstance(v, str) and os.path.isabs(v) else v)
                      for k, v in (("clips", ";".join(os.path.relpath(d, root).replace("\\", "/") for d in v1.split_dirs(args.clips))),
                                   ("rtz_clips", ";".join(os.path.relpath(d, root).replace("\\", "/")
                                                          for d in v1.split_dirs(getattr(args, "rtz_clips", None) or ""))),
                                   ("sourcepack_only", bool(getattr(args, "sourcepack_only", False))),
                                   ("rig", args.rig), ("bonemap", args.bonemap or cfg2["sources"]["bonemap"]),
                                   ("motion_src", motion_src))},
        "rtz": {"clips_root_z": n_rtz, "clips_unknown": sorted(c.name for c in clips if c.rtz_unknown),
                "clips_none": sum(1 for c in clips if c.rtz_src == RTZ_SRC_NONE),
                "rtz_files_without_clip_file": rtz_unmatched,
                "root_z_ends_vs_ue_root_track_over_1cm": rtz_end_mismatch},
        "extrapolation": extrap,
        "config": os.path.relpath(args.config, root).replace("\\", "/"), "config_sha256": v1.sha256_file(args.config),
        "v2_config": os.path.relpath(cfg2_path, root).replace("\\", "/"), "v2_config_sha256": v1.sha256_file(cfg2_path),
        "motion_src": motion_src,
        **({"extra_assets": {"file": os.path.relpath(args.extra_assets, root).replace("\\", "/"),
                             "sha256": v1.sha256_file(args.extra_assets),
                             "clips": {n: v1.sha256_file(extra_files[n]) for n in sorted(extra_files)},
                             "entries": [{"db": e["db"], "clip": e["clip"], "disable_reselection": e["disable_reselection"]}
                                         for e in entries if e["added"] and e["clip"] in extra_files]}}
           if extra is not None else {}),
        **({"slide_spec": {"file": os.path.relpath(args.slide_spec, root).replace("\\", "/"),
                           "sha256": v1.sha256_file(args.slide_spec), "databases": slide_dbs}} if slide is not None else {}),
        "ue_tracks": os.path.relpath(ut_path, root).replace("\\", "/") if ue_tracks else None,
        "inputs_sha256": hashlib.sha256(b"".join(c.name.encode() + b"\x00" + v1.sha256_file(c.path).encode()
                                                 for c in clips)).hexdigest(),
        "provenance": {"GASP-ALS": sum(1 for c in clips if c.sourcepack),
                       **{k: v for k, v in sorted(
                           {s: sum(1 for c in clips if not c.sourcepack and (cm_fem.get(c.name) or {}).get("source", "unknown") == s)
                            for s in {(cm_fem.get(c.name) or {}).get("source", "unknown") for c in clips if not c.sourcepack}}.items())}},
        "fps": FPS, "num_frames": NF, "num_clips": len(clips), "num_databases": len(db_names), "num_entries": len(entries),
        "num_rows": len(drow), "num_events": len(events), "has_scale": has_scale,
        "frames_indexed": n_idx, "frames_selectable": n_sel,
        "frames_block_transition": int(((fflags & FF_BLOCK_TRANSITION) != 0).sum()),
        "frames_excluded_notify": int(((fflags & FF_EXCLUDED_NOTIFY) != 0).sum()),
        "frames_excluded_tail": int(((fflags & FF_EXCLUDED_TAIL) != 0).sum()),
        "section_bytes": sec_sizes,
        "ram_estimate_bytes": len(data),
        "schemas": [{"name": s, "dims": schemas[s][1], "channels": [
            {"label": c.label(), "kind": c.kind, "comp": c.comp, "bone": c.bone, "game_bone": c.game_bone,
             "origin": c.origin, "t": c.t_sample, "weight": c.weight, "group": c.group, "offset": c.offset,
             "card": c.card, "axis": list(c.local_axis), "flags": c.flags} for c in schemas[s][0]]} for s in schema_names],
        "norms": [{"schema": n["schema"], "set": n["set"], "members": n["members"], "rows": n["rows"],
                   "dev": [round(float(x), 7) for x in n["dev"]], "mad": [round(float(x), 7) for x in n["mad"]]} for n in norms],
        "databases": [{k: (v if not isinstance(v, float) else round(v, 6)) for k, v in m.items()} for m in dbas_meta],
        "skipped_entries": skipped, "clips_not_baked": unused_files,
        "curve_validation": val_summary, "curve_sources": curve_report,
        "chooser_file": os.path.basename(args.chooser_out or os.path.splitext(args.out)[0] + ".chooser.json") if chooser else None,
        "clips": [{"name": c.name, "first_frame": c.first, "num_frames": c.N, "loop": c.loop, "sourcepack": c.sourcepack,
                   "policy": c.policy, "rtz_unknown": c.rtz_unknown,
                   "rtz_source": ("none", "unknown", "root_z")[c.rtz_src],
                   "selectable": int(((fflags[c.first:c.first + c.N] & FF_SELECTABLE) != 0).sum()),
                   "databases": sorted({e["db"] for e in entries if e["clip"] == c.name})} for c in clips],
    }
    if args.summary:
        with open(args.summary, "w", encoding="utf-8", newline="\n") as f:
            json.dump(summary, f, indent=1)
            f.write("\n")
    if getattr(args, "raw_dump", None):
        dump = {f"raw_{s}": raw[s] for s in schema_names}
        dump["schema_names"] = np.array(schema_names)
        dump["db_names"] = np.array(db_names)
        dump["row_frame"] = np.array([r[0] for r in drow], np.int64)
        dump["row_addend"] = np.array([r[1] for r in drow], np.float64)
        dump["row_cont"] = np.array([r[2] for r in drow], np.float64)
        dump["row_entry"] = np.array([r[3] for r in drow], np.int64)
        dump["row_flags"] = np.array([r[4] for r in drow], np.int64)
        dump["db_first_row"] = np.array([m["first_row"] for m in dbas_meta], np.int64)
        dump["db_num_rows"] = np.array([m["num_rows"] for m in dbas_meta], np.int64)
        dump["db_schema"] = np.array([schema_names[m["schema"]] for m in dbas_meta])
        dump["db_norm"] = np.array([m["norm"] for m in dbas_meta], np.int64)
        dump["fflags"] = fflags
        dump["curv"] = CURV
        dump["cmask"] = CMASK
        dump["clip_names"] = np.array([c.name for c in clips])
        dump["clip_first"] = np.array([c.first for c in clips], np.int64)
        dump["clip_n"] = np.array([c.N for c in clips], np.int64)
        np.savez_compressed(args.raw_dump, **dump)
        log(f"raw dump: {args.raw_dump}")
    return data, summary
