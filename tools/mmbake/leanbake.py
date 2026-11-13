"""leanbake.py - bake the source pack lean / head-turn pose pack "utpp.lean/1" for one CP rig.

Inputs  (all read-only)
  Research/logs/sourcepack-lean-data-2026-10.json     params + runtime_format schema 'utpp.lean/1' + disable_clips
  Research/inventory/sourcepack-blendspaces/*.json    BlendSpace definitions (axes, samples, interp speed)
  Animations/retarget/out_sourcepack_set2_<rig>       Relaxed retargeted clips (utppclip v1) - lean + head poses, turn clips
  Animations/retarget/out_sourcepack_set3_<rig>       Neutral retargeted clips - M_Neutral_Run_Lean_Pose_* (CMC BS1D)
  Animations/sourcepack/set{2,3}_export/meta          additive reference pose per sample (meta.additive.base)
Output  Data/utpp_lean_<rig>.bin  +  Data/utpp_lean_<rig>.json (human readable header, schema 'utpp.lean/1')

The retargeted additive poses are FULL source poses; this baker converts every blend space sample to the additive
delta against the reference pose named in the export meta (AAT_ROTATION_OFFSET_MESH_SPACE), with exactly the
representation of overlaybake.py / Additive.hpp (ov::Xf per bone):
    dq_ms = canon( G_X * inv(G_A) )   mesh-space rotation offset, quaternion x y z w, w >= 0
    dt    = t_X - t_A                 local translation delta (metres)
    ds    = s_X / s_A - 1             local scale delta
Apply (Additive.hpp ApplyAdditive, meshSpace): G <- nlerp(1, dq, a) * G, t += a*dt, s *= 1 + a*ds.

======================================================================================================================
BINARY FORMAT  utpp_lean_<rig>.bin  (little-endian, IEEE float32, 16-byte aligned sections; reader: LeanPack.hpp)
======================================================================================================================
Header (64 bytes)
   0  8  magic "UTPPLEAN"
   8  4  u32 version = 1
  12  4  u32 header_size = 64
  16  8  u64 rig_hash   (rig IDENTITY hash exactly as .utppmm v2 / .utppov; woman 0xf62810a88a8bfda9,
                          man 0xafe2cfedcf97cdf7)
  24  4  u32 num_bones = 73   (CP rig bones 0..72, same order as the v2 bundles)
  28  4  u32 num_sections
  32  4  u32 file_size
  36  4  u32 payload_crc32 = CRC-32 (zlib) of bytes [64, file_size)
  40  4  u32 header_crc32  = CRC-32 of [0,64) with this field zeroed
  44 20  zero
Section table at 64: num_sections x 16 B: tag[4], u32 offset (absolute), u32 size, u32 count. Order:
  BONE PARM LAYR SAMP TRIS DELT CLIP CVAL.   Readers locate by tag and fail closed (size == count * record size).
BONE  count=73, 36 B: name[32] zero padded, i32 parent.
PARM  count=60 f32 (record = 1 float) - see PARM_FIELDS below (cm, cm/s, cm/s^2 as UE; the runtime converts).
LAYR  count=L, 240 B per layer:
        0 id[16]  16 blendspace[48]  64 base_clip[64]  128 axis0[16]  144 axis1[16]
      160 u32 dims (1|2)  164 u32 style (0 Relaxed, 1 Neutral)  168 u32 base_frame  172 u32 role
          (0 body lean, 1 head lean, 2 CMC fallback lean)
      176 u32 first_sample  180 u32 num_samples (<= 16)  184 u32 first_tri  188 u32 num_tris
      192 f32 min0  196 f32 max0  200 f32 min1  204 f32 max1   (axis ranges; axis1 = 0,0 for dims 1)
      208 f32 weight_interp_per_s   212..239 zero
      Layer order: run, walk, head, cmc_run.
SAMP  count=S, 80 B: f32 x, f32 y, u32 clip_frame, u32 0, clip[64] (utppclip name the delta was built from).
      Samples of a layer are contiguous (first_sample..+num_samples) in BlendSpace sample order (sample index =
      position inside the layer). DELT has one block per SAMP record in the same order.
TRIS  count=T, 72 B = 18 x i32 (layer-local sample indices; -1 none):
      2D: UE FBlendSpaceTriangle after FDelaunayTriangleGenerator (UE 5.8 BlendSpaceHelpers.cpp, ported 1:1, run in
          the baker): s0 s1 s2 (CCW), then for each edge e (v[e] -> v[e+1]): neighbour_tri, perim_tri[0],
          perim_tri[1], perim_vert[0], perim_vert[1] (FBlendSpaceTriangleEdgeInfo; all layer-local triangle
          indices). Vertices/normals are NOT stored: the reader derives them from the sample coordinates exactly
          like UE (normalised to the axis range, edge normal = (dy,-dx) normalised).
      1D: segments sorted by x: s0 s1 s1, rest -1.
DELT  count = S, record = num_bones x 10 f32 = dq(x y z w), dt(x y z), ds(x y z)   (see above)
CLIP  count=C, 80 B: name[64], u32 first (CVAL index), u32 count, u32 kind (0 = Relaxed source clip carrying the
      curve, 1 = proposed Neutral equivalent, curve resampled by absolute time), f32 fps (30).
CVAL  f32 per curve frame: Disable_AdditiveLeans value 0..1 (1 = lean muted), 30 fps, frame i at t = i/30 s,
      the runtime clamps to the last value past the end.

PARM_FIELDS (index: value)  [all UE units]
  0 lat_speed_lo 200   1 lat_speed_hi 320   2 lat_accel_lo 500   3 lat_accel_hi 800   4 lat_clamp_lo -1  5 lat_clamp_hi 1
  6 xfade_speed_lo 200 7 xfade_speed_hi 320 8 xfade_interp_per_s 10  9 bool_blend_s 0.3  10 state_blend_s 0.3
  11 disable_curve_interp_per_s 5
  12 cmc_speed_lo 165  13 cmc_speed_hi 375  14 cmc_scale_lo 0.5  15 cmc_scale_hi 1.0  16 cmc_alpha_blend_s 0.2
  17 cmc_weight_interp_per_s 1.5  18 head_weight_interp 1.75  19 body_weight_interp 1.2
  20..31  direction map, enum order F=0 B=1 LR=2 LL=3 RL=4 RR=5: (x_scale, y_scale) per direction
  32..59 zero (reserved)
Gates (not data): lean only while MovementState == Moving and MovementMode != Sliding; head layer dropped while
EnableAO; final lean weight = 1 - Disable_AdditiveLeans (interp 5/s). See LeanPack.hpp for the evaluation API.

Usage: python -I leanbake.py [--rig woman|man ...] [--root <project root>] [--out-dir <dir>]
"""
import argparse
import hashlib
import json
import math
import os
import struct
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mmbake_base as v1  # noqa: E402
import mmbake_motion as v2  # noqa: E402
import overlaybake as ob  # noqa: E402

NUM_BONES = 73
MAGIC = b"UTPPLEAN"
VERSION = 1
HEADER_SIZE = 64
MAX_SAMPLES = 16
LAYR_SIZE, SAMP_SIZE, TRIS_SIZE, CLIP_SIZE, BONE_SIZE = 240, 80, 72, 80, 36
PARM_COUNT = 60
CURVE_FPS = 30.0
DISABLE_CURVE = "disable_additiveleans"

RIGS = {
    "woman": dict(rig="Extracted/game/json/vanilla/player_woman_skeleton.rig.json",
                  relaxed="Animations/retarget/out_sourcepack_set2_woman",
                  neutral="Animations/retarget/out_sourcepack_set3_woman"),
    "man": dict(rig="Extracted/game/json/vanilla-man/player_man_skeleton.rig.json",
                relaxed="Animations/retarget/out_sourcepack_set2_man",
                neutral="Animations/retarget/out_sourcepack_set3_man"),
}
META = {"Relaxed": "Animations/sourcepack/set2_export/meta", "Neutral": "Animations/sourcepack/set3_export/meta"}
STYLE_ID = {"Relaxed": 0, "Neutral": 1}
ROLE_BODY, ROLE_HEAD, ROLE_CMC = 0, 1, 2
# layer id, blend space asset, style, role
LAYERS = [
    ("run", "BS_Relaxed_Run_Leans", "Relaxed", ROLE_BODY),
    ("walk", "BS_Relaxed_Walk_Leans", "Relaxed", ROLE_BODY),
    ("head", "BS_Relaxed_Lean_Head", "Relaxed", ROLE_HEAD),
    ("cmc_run", "BS1D_Additive_Lean_Run", "Neutral", ROLE_CMC),
]
DIRECTION_ORDER = ["F", "B", "LR", "LL", "RL", "RR"]  # E_MovementDirection values 0..5
# NOTE (2026-11-12): this file no longer exists anywhere in the project (searched Research, Build, Data by name); the lean bake
# and test_leanbake.py need it regenerated or a new path before they can run.
LEAN_DATA = "Research/logs/sourcepack-lean-data-2026-10.json"
BS_DIR = "Research/inventory/sourcepack-blendspaces"


def find_root(start=HERE):
    d = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(d, "Animations")) and os.path.isdir(os.path.join(d, "Data")):
            return d
        p = os.path.dirname(d)
        if p == d:
            raise SystemExit("project root (folder with Animations and Data) not found above " + start)
        d = p


def load_json(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------------------------------------------------------------
# UE 5.8 FDelaunayTriangleGenerator, ported 1:1 (BlendSpaceHelpers.cpp / .h) for PreferredTriangulationDirection None.
# ---------------------------------------------------------------------------------------------------------------------
class _V:
    def __init__(self, pos, sample):
        self.pos = pos
        self.sample = sample


class _Tri:
    def __init__(self, a, b, c):
        self.v = [a, b, c]
        self.center = ((a.pos[0] + b.pos[0] + c.pos[0]) / 3.0, (a.pos[1] + b.pos[1] + c.pos[1]) / 3.0)
        d1 = (b.pos[0] - a.pos[0], b.pos[1] - a.pos[1])
        d2 = (c.pos[0] - a.pos[0], c.pos[1] - a.pos[1])
        if d1[0] * d2[1] - d1[1] * d2[0] < 0.0:  # MakeCCW: swap 1 and 2
            self.v[1], self.v[2] = self.v[2], self.v[1]
        self.edges = [(self.v[0], self.v[1]), (self.v[1], self.v[2]), (self.v[2], self.v[0])]

    def same(self, o):
        return all(x is y for x, y in zip(self.v, o.v))

    def has_same_half_edge(self, o):
        return any(a[0] is b[0] and a[1] is b[1] for a in o.edges for b in self.edges)

    def shares_edge(self, o):
        return any(a[0] is b[1] and a[1] is b[0] for a in o.edges for b in self.edges)

    def contains(self, vert):
        return any(vert.pos == x.pos for x in self.v)

    def non_sharing_point(self, o):
        for x in o.v:
            if not self.contains(x):
                return x
        return None


class UEDelaunay:
    def __init__(self, min_xy, max_xy):
        mid = ((min_xy[0] + max_xy[0]) * 0.5, (min_xy[1] + max_xy[1]) * 0.5)
        rng = (max(max_xy[0] - min_xy[0], 1e-8), max(max_xy[1] - min_xy[1], 1e-8))
        self.grid_min = (mid[0] - rng[0] * 0.5, mid[1] - rng[1] * 0.5)
        self.recip = (1.0 / rng[0], 1.0 / rng[1])
        self.points = []
        self.tris = []

    def norm(self, p):
        return ((p[0] - self.grid_min[0]) * self.recip[0], (p[1] - self.grid_min[1]) * self.recip[1])

    def add_point(self, pos, sample):
        if any(p.pos == pos for p in self.points):
            raise SystemExit("duplicate points in blendspace")
        self.points.append(_V(pos, sample))

    @staticmethod
    def collinear(a, b, c):
        d1 = (b.pos[0] - a.pos[0], b.pos[1] - a.pos[1])
        d2 = (c.pos[0] - a.pos[0], c.pos[1] - a.pos[1])
        return d1[0] * d2[1] - d1[1] * d2[0] == 0.0

    def eligible(self, a, b, c):
        return not self.collinear(a, b, c)

    def circumcircle(self, t, test):  # -1 outside, 0 on, 1 inside
        p = [self.norm(x.pos) for x in t.v]
        tp = self.norm(test.pos)
        m00 = p[0][0] - tp[0]
        m01 = p[0][1] - tp[1]
        m02 = p[0][0] * p[0][0] - tp[0] * tp[0] + p[0][1] * p[0][1] - tp[1] * tp[1]
        m10 = p[1][0] - tp[0]
        m11 = p[1][1] - tp[1]
        m12 = p[1][0] * p[1][0] - tp[0] * tp[0] + p[1][1] * p[1][1] - tp[1] * tp[1]
        m20 = p[2][0] - tp[0]
        m21 = p[2][1] - tp[1]
        m22 = p[2][0] * p[2][0] - tp[0] * tp[0] + p[2][1] * p[2][1] - tp[1] * tp[1]
        det = m00 * m11 * m22 + m01 * m12 * m20 + m02 * m10 * m21 - (m02 * m11 * m20 + m01 * m10 * m22 + m00 * m12 * m21)
        if det < 0.0:
            return -1
        if abs(det) <= 1e-8:
            return 0
        return 1

    def add_triangle(self, t, check_half_edge=True):
        for o in self.tris:
            if t.same(o):
                return
            if check_half_edge and t.has_same_half_edge(o):
                return
        self.tris.append(t)

    def flip(self, i, j):
        a, b = self.tris[i], self.tris[j]
        test = a.non_sharing_point(b)
        if self.circumcircle(a, test) != 1:
            return False
        made = []
        for one in range(2):
            for two in range(one + 1, 3):
                if self.eligible(a.v[one], a.v[two], test):
                    nt = _Tri(a.v[one], a.v[two], test)
                    three = 3 - (two + one)
                    if self.circumcircle(nt, a.v[three]) == -1:
                        if len(made) >= 2:
                            raise SystemExit("incorrect number of triangles created")
                        made.append(nt)
        if len(made) == 2:
            self.add_triangle(made[0], False)
            self.add_triangle(made[1], False)
        return len(made) == 2

    def generate(self, total):
        pl = self.points
        if total == 3:
            if self.eligible(pl[0], pl[1], pl[2]):
                self.add_triangle(_Tri(pl[0], pl[1], pl[2]))
        elif not self.tris:
            test = pl[total - 1]
            for i in range(total - 2):
                if self.eligible(pl[i], pl[i + 1], test):
                    self.add_triangle(_Tri(pl[i], pl[i + 1], test))
        else:
            test = pl[total - 1]
            i = 0
            while i < len(self.tris):  # the list grows while iterating, as in UE
                t = self.tris[i]
                for a, b in ((0, 1), (0, 2), (1, 2)):
                    if self.eligible(t.v[a], t.v[b], test):
                        self.add_triangle(_Tri(t.v[a], t.v[b], test))
                i += 1
            i = 0
            while i < len(self.tris):
                restart = False
                a = self.tris[i]
                for j in range(i + 1, len(self.tris)):
                    b = self.tris[j]
                    if a.shares_edge(b) and self.flip(i, j):
                        del self.tris[j]
                        del self.tris[i]
                        restart = True
                        break
                i = 0 if restart else i + 1

    def triangulate(self):
        if len(self.points) < 3:
            raise SystemExit("2D blend space needs >= 3 samples")
        self.points.sort(key=lambda v: (v.pos[1], v.pos[0]))
        for i in range(2, len(self.points)):
            self.generate(i + 1)
        if not self.tris:
            raise SystemExit("collinear 2D blend space not supported")

    def find_tri_with_edge(self, s0, s1):
        for ti, t in enumerate(self.tris):
            for k in range(3):
                if t.v[k].sample == s0 and t.v[(k + 1) % 3].sample == s1:
                    return ti
        return -1

    def calculate(self):
        """FDelaunayTriangleGenerator::CalculateTriangles -> per triangle dict(samples, edges[3]=dict(nbr, pt, pv))."""
        out = []
        for t in self.tris:
            e = []
            for k in range(3):
                kn = (k + 1) % 3
                e.append(dict(nbr=self.find_tri_with_edge(t.v[kn].sample, t.v[k].sample), pt=[-1, -1], pv=[-1, -1]))
            out.append(dict(samples=[v.sample for v in t.v], edges=e, verts=[self.norm(v.pos) for v in t.v]))
        for ti, tr in enumerate(out):
            for k in range(3):
                ei = tr["edges"][k]
                if ei["nbr"] != -1:
                    continue
                kn = (k + 1) % 3
                s, sn = tr["samples"][k], tr["samples"][kn]
                for oi, ot in enumerate(out):
                    if oi == ti:
                        continue
                    for vi in range(3):
                        vp = (vi + 2) % 3
                        if ot["samples"][vi] == s and ot["edges"][vp]["nbr"] == -1:
                            ei["pt"][0], ei["pv"][0] = oi, vp
                        if ot["samples"][vi] == sn and ot["edges"][vi]["nbr"] == -1:
                            ei["pt"][1], ei["pv"][1] = oi, vi
        return out


def segments_1d(samples):
    """UBlendSpace1D::ResampleData1D: segments between x-sorted samples."""
    order = sorted(range(len(samples)), key=lambda i: samples[i]["x"])
    if len(order) < 2:
        raise SystemExit("1D blend space needs >= 2 samples")
    return [(order[i], order[i + 1]) for i in range(len(order) - 1)]


# ---------------------------------------------------------------------------------------------------------------------
def name_field(s, n):
    return ob.name_bytes(s, n)


def clip_file(dirp, name):
    p = os.path.join(dirp, name + ".utppclip.json")
    if not os.path.isfile(p):
        raise SystemExit("missing retargeted clip " + p)
    return p


def read_params(root):
    """Params from the lean data JSON runtime_format.schema.params (the single source for the formula constants)."""
    d = load_json(os.path.join(root, LEAN_DATA.replace("/", os.sep)))
    if d.get("schema") != "utpp.sourcepack.lean.data/1":
        raise SystemExit("lean data schema mismatch")
    rf = d["runtime_format"]
    if rf["name"] != "utpp.lean/1":
        raise SystemExit("runtime_format name mismatch")
    p = rf["schema"]["params"]
    lat = p["lat_accel_amount"]
    if lat["frame"] != "velocity_heading":
        raise SystemExit("lat frame changed")
    dm = p["direction_map"]
    f = [0.0] * PARM_COUNT
    f[0:2] = [float(x) for x in lat["denominator_speed_cm_s"]]
    f[2:4] = [float(x) for x in lat["denominator_accel_cm_s2"]]
    f[4:6] = [float(x) for x in lat["clamp"]]
    f[6:8] = [float(x) for x in p["speed_crossfade"]["speed_cm_s"]]
    f[8] = float(p["speed_crossfade"]["interp_per_s"])
    f[9] = float(p["bool_blend_s"])
    f[10] = float(p["state_blend_s"])
    f[11] = float(p["disable_curve"]["interp_per_s"])
    cmc = p["cmc_style"]
    if cmc["x"] != "rel_accel_y * MapRangeClamped(speed, 165..375 -> 0.5..1)":
        raise SystemExit("cmc_style formula changed")
    f[12:14] = [165.0, 375.0]
    f[14:16] = [0.5, 1.0]
    f[16] = float(cmc["alpha_is_moving_blend_s"])
    f[17] = float(cmc["weight_interp_per_s"])
    for i, k in enumerate(DIRECTION_ORDER):
        f[20 + 2 * i], f[21 + 2 * i] = float(dm[k][0]), float(dm[k][1])
    return f, d


def load_blendspace(root, name):
    b = load_json(os.path.join(root, BS_DIR.replace("/", os.sep), name + ".json"))
    if b.get("schema") != "utpp.gaspals.blendspace/1" or b["name"] != name:
        raise SystemExit(name + ": bad blend space json")
    if b["preferred_triangulation_direction"] != "None":
        raise SystemExit(name + ": preferred triangulation direction != None not supported (AdjustEdgeDirections)")
    if not b["rotation_offset_mesh_space_samples"]:
        raise SystemExit(name + ": not a mesh-space rotation offset blend space")
    if len(b["samples"]) > MAX_SAMPLES:
        raise SystemExit(name + ": too many samples")
    for s in b["samples"]:
        if s["single_frame_for_blending"] or s["frame_index_to_sample"] != 0 or s["rate_scale"] != 1.0 or s["mirror"]:
            raise SystemExit(name + ": sample flags not supported: " + s["animation"])
    return b


def build_layer(root, cfg, rig_cfg, names, parents, clip_cache):
    lid, bsname, style, role = cfg
    bs = load_blendspace(root, bsname)
    dirp = os.path.join(root, rig_cfg["relaxed" if style == "Relaxed" else "neutral"].replace("/", os.sep))
    metadir = os.path.join(root, META[style].replace("/", os.sep))
    dims = len(bs["axes"])
    if dims not in (1, 2):
        raise SystemExit(bsname + ": dims")
    ax = bs["axes"]
    samples, deltas = [], []
    base_clip = None
    for s in bs["samples"]:
        cname = s["animation"]
        meta = load_json(os.path.join(metadir, cname + ".json"))["additive"]
        if meta["additive_anim_type"] != "AAT_ROTATION_OFFSET_MESH_SPACE":
            raise SystemExit(cname + ": not AAT_ROTATION_OFFSET_MESH_SPACE")
        ref = meta["base"]["seq"]
        rframe = int(meta["base"]["frame"])
        if base_clip is None:
            base_clip = (ref, rframe)
        elif base_clip != (ref, rframe):
            raise SystemExit("%s: layer reference pose differs (%s)" % (cname, (ref, rframe)))
        for n in (cname, ref):
            if n not in clip_cache:
                clip_cache[n] = ob.Clip(clip_file(dirp, n), names, parents)
        X, A = clip_cache[cname], clip_cache[ref]
        fr = int(s["frame_index_to_sample"])
        if fr >= X.n or rframe >= A.n:
            raise SystemExit(cname + ": frame out of range")
        d = ob.delta(X, [fr], A, rframe)[0]  # (B,14)
        deltas.append(np.concatenate([d[:, 0:4], d[:, 8:11], d[:, 11:14]], axis=1))
        samples.append(dict(x=float(s["x"]), y=float(s.get("y", 0.0)) if dims == 2 else 0.0, clip=cname, frame=fr))
    layer = dict(id=lid, blendspace=bsname, style=style, role=role, dims=dims, samples=samples, deltas=deltas,
                 base_clip=base_clip[0], base_frame=base_clip[1], interp=float(bs["target_weight_interp_speed"]),
                 axes=[dict(name=a["name"], min=float(a["min"]), max=float(a["max"])) for a in ax])
    if dims == 2:
        dg = UEDelaunay((ax[0]["min"], ax[1]["min"]), (ax[0]["max"], ax[1]["max"]))
        for i, s in enumerate(samples):
            dg.add_point((s["x"], s["y"]), i)
        dg.triangulate()
        layer["tris"] = dg.calculate()
    else:
        layer["tris"] = [dict(samples=[a, b, b], edges=None) for a, b in segments_1d(samples)]
    return layer


def resample_curve(cur, n_out, scale=1.0):
    """Linear resample: output frame i reads the source at i * scale (30 fps both); clamp to the last value."""
    out = []
    for i in range(n_out):
        f = min(float(i) * scale, float(len(cur) - 1))
        i0 = int(f)
        i1 = min(len(cur) - 1, i0 + 1)
        t = f - i0
        out.append(cur[i0] * (1.0 - t) + cur[i1] * t)
    return out


def traj_yaw_f90(clip):
    """First frame where the Trajectory bone (index 1) has turned 90 % of its total yaw (turn clips: ends the turn)."""
    if clip["bone_names"][1] != "Trajectory":
        raise SystemExit("bone 1 is not Trajectory")
    ys = []
    for f in clip["frames"]:
        x, y, z, w = f["bones"][1][3:7]
        ys.append(math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))
    tot = ys[-1] - ys[0]
    if abs(tot) < 1.0:
        raise SystemExit("no trajectory yaw in turn clip")
    return next(i for i, v in enumerate(ys) if abs(v - ys[0]) >= 0.9 * abs(tot)), tot


def neutral_equivalent(relaxed_clip):
    """Relaxed Turn_090 clip name -> Neutral clip name pattern (None when there is no equivalent)."""
    p = relaxed_clip.split("_")
    # M_Relaxed_Run_Turn_L_090_Lfoot -> M_Neutral_Run_Turn_L_090_Lfoot
    # M_Relaxed_Walk_Turn_090_L_Lfoot -> M_Neutral_Walk_Turn_L_090_Lfoot  (word order differs for Walk)
    if len(p) == 7 and p[2] in ("Run", "Walk") and p[3] == "Turn":
        if p[4] in ("L", "R") and p[5] == "090":
            return "_".join(["M", "Neutral", p[2], "Turn", p[4], "090", p[6]])
        if p[4] == "090" and p[5] in ("L", "R"):
            return "_".join(["M", "Neutral", p[2], "Turn", p[5], "090", p[6]])
    return None


def build_disable(root, rig_cfg, lean_data):
    rel_dir = os.path.join(root, rig_cfg["relaxed"].replace("/", os.sep))
    neu_dir = os.path.join(root, rig_cfg["neutral"].replace("/", os.sep))
    clips, cval = [], []
    mapping, unmapped = [], []
    src_curves = {}
    for dc in lean_data["disable_clips"]:
        name = dc["clip"]
        c = load_json(clip_file(rel_dir, name))
        cur = [min(1.0, max(0.0, float(v))) for v in c["curves"][DISABLE_CURVE]]
        cur = [0.0 if v < 1e-6 else v for v in cur]
        if len(cur) != c["num_frames"] or int(c["fps"]) != 30:
            raise SystemExit(name + ": curve length / fps")
        src_curves[name] = cur
        clips.append(dict(name=name, kind=0, first=len(cval), count=len(cur)))
        cval.extend(cur)
    for name in [d["clip"] for d in lean_data["disable_clips"]]:
        neu = neutral_equivalent(name)
        if neu is None:
            unmapped.append(name)
            continue
        nc = load_json(clip_file(neu_dir, neu))
        n_out = int(nc["num_frames"])
        f90_n, tot_n = traj_yaw_f90(nc)
        f90_r, tot_r = traj_yaw_f90(load_json(clip_file(rel_dir, name)))
        if abs(tot_n - tot_r) > 5.0 or (tot_n < 0) != (tot_r < 0):
            raise SystemExit("%s / %s: trajectory turn differs (%.1f vs %.1f deg)" % (neu, name, tot_n, tot_r))
        scale = float(f90_r) / float(f90_n)
        cur = resample_curve(src_curves[name], n_out, scale)
        clips.append(dict(name=neu, kind=1, first=len(cval), count=n_out))
        cval.extend(cur)
        sc = src_curves[name]
        end_f = max(i for i, v in enumerate(sc) if v > 1e-4)
        mapping.append(dict(neutral=neu, source_relaxed=name, neutral_frames=n_out, relaxed_frames=len(sc),
                            relaxed_last_nonzero_frame=end_f, relaxed_trajectory_yaw_f90=f90_r,
                            neutral_trajectory_yaw_f90=f90_n, trajectory_yaw_deg=[round(tot_r, 1), round(tot_n, 1)],
                            time_scale_neutral_to_relaxed=round(scale, 4),
                            mode="curve(i_neutral * f90_relaxed / f90_neutral), linear, 30 fps"))
    return clips, cval, mapping, unmapped


def bake(rig_key, root, out_dir):
    cfg = RIGS[rig_key]
    rig_path = os.path.join(root, cfg["rig"].replace("/", os.sep))
    names, parents = v1.load_rig(rig_path, NUM_BONES)
    rest = v2.rig_rest_local(rig_path, NUM_BONES)
    rig_hash, _ = v2.rig_identity_hash(names, parents, rest)
    parm, lean_data = read_params(root)
    cache = {}
    layers = [build_layer(root, l, cfg, names, parents, cache) for l in LAYERS]
    by_role = {}
    for l in layers:
        by_role.setdefault(l["role"], set()).add(l["interp"])
    if any(len(v) != 1 for v in by_role.values()):
        raise SystemExit("layers of one role differ in weight interp")
    parm[18] = by_role[ROLE_HEAD].copy().pop()
    parm[19] = by_role[ROLE_BODY].copy().pop()   # walk and run share 1.2
    if by_role[ROLE_CMC] != {parm[17]}:
        raise SystemExit("cmc weight interp differs from the lean data JSON")
    clips, cval, mapping, unmapped = build_disable(root, cfg, lean_data)

    # ---- sections
    sec = []
    sec.append((b"BONE", NUM_BONES, b"".join(name_field(n, 32) + struct.pack("<i", p) for n, p in zip(names, parents))))
    sec.append((b"PARM", PARM_COUNT, struct.pack("<%df" % PARM_COUNT, *parm)))
    lb, sb, tb, db = bytearray(), bytearray(), bytearray(), bytearray()
    s_total = t_total = 0
    for l in layers:
        ns, nt = len(l["samples"]), len(l["tris"])
        ax0 = l["axes"][0]
        ax1 = l["axes"][1] if l["dims"] == 2 else dict(name="", min=0.0, max=0.0)
        lb += (name_field(l["id"], 16) + name_field(l["blendspace"], 48) + name_field(l["base_clip"], 64)
               + name_field(ax0["name"], 16) + name_field(ax1["name"], 16)
               + struct.pack("<8I", l["dims"], STYLE_ID[l["style"]], l["base_frame"], l["role"], s_total, ns,
                             t_total, nt)
               + struct.pack("<5f", ax0["min"], ax0["max"], ax1["min"], ax1["max"], l["interp"])
               + b"\x00" * (LAYR_SIZE - 212))
        for s, d in zip(l["samples"], l["deltas"]):
            sb += struct.pack("<ffII", s["x"], s["y"], s["frame"], 0) + name_field(s["clip"], 64)
            db += np.asarray(d, np.float64).astype("<f4").tobytes()
        for t in l["tris"]:
            rec = [-1] * 18
            rec[0:3] = t["samples"]
            if t["edges"]:
                for k, e in enumerate(t["edges"]):
                    rec[3 + 5 * k:8 + 5 * k] = [e["nbr"], e["pt"][0], e["pt"][1], e["pv"][0], e["pv"][1]]
            tb += struct.pack("<18i", *rec)
        s_total += ns
        t_total += nt
    sec.append((b"LAYR", len(layers), bytes(lb)))
    sec.append((b"SAMP", s_total, bytes(sb)))
    sec.append((b"TRIS", t_total, bytes(tb)))
    sec.append((b"DELT", s_total, bytes(db)))
    cb = b"".join(name_field(c["name"], 64) + struct.pack("<IIIf", c["first"], c["count"], c["kind"], CURVE_FPS)
                  for c in clips)
    sec.append((b"CLIP", len(clips), cb))
    sec.append((b"CVAL", len(cval), struct.pack("<%df" % len(cval), *cval)))

    table = bytearray()
    body = bytearray()
    off = HEADER_SIZE + 16 * len(sec)
    off += (-off) % 16
    pad0 = off - (HEADER_SIZE + 16 * len(sec))
    sec_info = []
    for tag, cnt, data in sec:
        sec_info.append(dict(tag=tag.decode(), offset=off + len(body), size=len(data), count=cnt))
        table += tag + struct.pack("<III", off + len(body), len(data), cnt)
        body += data
        body += b"\x00" * ((-len(body)) % 16)
    payload = bytes(table) + b"\x00" * pad0 + bytes(body)
    file_size = HEADER_SIZE + len(payload)
    hdr = bytearray(HEADER_SIZE)
    hdr[0:8] = MAGIC
    struct.pack_into("<IIQIIII", hdr, 8, VERSION, HEADER_SIZE, rig_hash, NUM_BONES, len(sec), file_size,
                     zlib.crc32(payload) & 0xFFFFFFFF)
    struct.pack_into("<I", hdr, 40, zlib.crc32(bytes(hdr)) & 0xFFFFFFFF)
    blob = bytes(hdr) + payload
    os.makedirs(out_dir, exist_ok=True)
    bin_path = os.path.join(out_dir, "utpp_lean_%s.bin" % rig_key)
    with open(bin_path, "wb") as f:
        f.write(blob)

    header = dict(
        schema="utpp.lean/1", rig=os.path.basename(rig_path).split(".")[0], rig_key=rig_key,
        rig_hash="0x%016x" % rig_hash, num_bones=NUM_BONES,
        binary=dict(file=os.path.basename(bin_path), size_bytes=len(blob), sha256=hashlib.sha256(blob).hexdigest(),
                    payload_crc32="0x%08x" % (zlib.crc32(payload) & 0xFFFFFFFF), version=VERSION, sections=sec_info,
                    format="see the file header of Source/tools/mmbake/leanbake.py and plugin/UTPP/src/mm/LeanPack.hpp"),
        additive="mesh_space_rotation_offset",
        delta_layout="per sample num_bones x 10 f32: dq(x y z w, w>=0, canonical), dt(x y z) metres, ds(x y z)",
        layers=[dict(id=l["id"], blendspace=l["blendspace"], style=l["style"], role=l["role"],
                     axes=l["axes"], samples=[dict(x=s["x"], y=s["y"], clip=s["clip"], frame=s["frame"])
                                              for s in l["samples"]],
                     triangles=[t["samples"] for t in l["tris"]],
                     weight_interp_per_s=l["interp"], additive="mesh_space_rotation_offset",
                     base_clip=l["base_clip"], base_frame=l["base_frame"]) for l in layers],
        reference_pose=dict(clip="per layer: layers[].base_clip", frame=0,
                            note="additive base; samples are AAT_ROTATION_OFFSET_MESH_SPACE relative to this frame"),
        params=dict(
            lat_accel_amount=dict(frame="velocity_heading", denominator_speed_cm_s=parm[0:2],
                                  denominator_accel_cm_s2=parm[2:4], clamp=parm[4:6]),
            direction_map={k: [parm[20 + 2 * i], parm[21 + 2 * i]] for i, k in enumerate(DIRECTION_ORDER)},
            direction_enum_order=DIRECTION_ORDER,
            speed_crossfade=dict(from_layer="walk", to_layer="run", speed_cm_s=parm[6:8], interp_per_s=parm[8]),
            head_layer_disabled_when="aim_offset_active", bool_blend_s=parm[9], state_blend_s=parm[10],
            only_when=dict(movement_state="Moving", movement_mode_not=["Sliding"]),
            disable_curve=dict(name="Disable_AdditiveLeans", interp_per_s=parm[11], lean_weight="1 - curve"),
            cmc_style=dict(x="rel_accel_y * MapRangeClamped(speed, 165..375 -> 0.5..1)", speed_cm_s=parm[12:14],
                           scale=parm[14:16], alpha_is_moving_blend_s=parm[16], weight_interp_per_s=parm[17]),
            weight_interp_per_s=dict(head=parm[18], body=parm[19], cmc=parm[17]),
            ease_in_out=True, ease_in_out_note="bTargetWeightInterpolationEaseInOut is an engine default, not serialized",
            units="cm, cm/s, cm/s^2 as in UE; the runtime converts to m (x0.01) like LeanParams"),
        disable_curves=dict(
            curve="Disable_AdditiveLeans", fps=CURVE_FPS, values="clamped to 0..1 (retarget noise ~1e-8 removed)",
            clips=[dict(name=c["name"], kind="relaxed_source" if c["kind"] == 0 else "neutral_proposed",
                        frames=c["count"]) for c in clips],
            neutral_mute_map=mapping, relaxed_clips_without_neutral_equivalent=unmapped,
            neutral_mapping_rule=("Relaxed {Run,Walk} Turn 090 (L|R) (Lfoot|Rfoot) -> Neutral {Run,Walk}_Turn_(L|R)_090_(Lfoot|Rfoot); "
                                  "Spin_F_360 has no Neutral clip (Neutral has Turn_180, which the Relaxed set does NOT mute)")))
    json_path = os.path.join(out_dir, "utpp_lean_%s.json" % rig_key)
    with open(json_path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(header, f, indent=1)
    return dict(bin=bin_path, json=json_path, size=len(blob), sha256=header["binary"]["sha256"],
                rig_hash="0x%016x" % rig_hash, layers=[(l["id"], len(l["samples"]), len(l["tris"])) for l in layers],
                clips=len(clips), cvals=len(cval), mapped=len(mapping), unmapped=len(unmapped))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", choices=sorted(RIGS), action="append")
    ap.add_argument("--root", default=None)
    ap.add_argument("--out-dir", default=None)
    a = ap.parse_args()
    root = a.root or find_root()
    out = a.out_dir or os.path.join(root, "Data")
    for r in a.rig or ["woman", "man"]:
        s = bake(r, root, out)
        print("%s: %s" % (r, json.dumps(s)))


if __name__ == "__main__":
    main()
