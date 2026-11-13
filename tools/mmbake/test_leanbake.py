"""Independent reader / check of the lean pose pack 'utpp.lean/1' (leanbake.py) against the retargeted utppclip files,
the BlendSpace definitions and the lean data JSON, plus mutation proofs for the baker.

Run: python -I test_leanbake.py   (needs the retargeted source pack clips of Animations/retarget; the produced packs in
Data/ are compared byte for byte with a fresh bake when present).

check_pack() re-implements the pack reader and the delta maths WITHOUT the baker's helpers (own quaternion FK, own
circumcircle test) and returns a list of error strings; the unmutated bake must give [], every deliberately broken
baker (Mutations) must give a non-empty list.
"""
import hashlib
import json
import math
import os
import struct
import sys
import tempfile
import unittest
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import leanbake as lb  # noqa: E402
import mmbake_motion as v2  # noqa: E402

ROOT = lb.find_root()
NB = 73
TOL_Q = 2e-5     # quaternion component tolerance (float32 storage)
TOL_T = 1e-6


def P(*a):
    return os.path.join(ROOT, *[x.replace("/", os.sep) for x in a])


def jload(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


_clip_cache = {}


def clip(dirkey, rig, name):
    k = (dirkey, rig, name)
    if k not in _clip_cache:
        _clip_cache[k] = jload(os.path.join(P(lb.RIGS[rig][dirkey]), name + ".utppclip.json"))
    return _clip_cache[k]


# ---- own quaternion maths (x y z w) -------------------------------------------------------------------------------
def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def mesh_rotations(local_q, parents):
    g = np.zeros_like(local_q)
    for i in range(len(parents)):
        g[i] = local_q[i] if parents[i] < 0 else qmul(g[parents[i]], local_q[i])
        g[i] /= np.linalg.norm(g[i])
    return g


def canon(q):
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    return np.where(q[..., 3:4] < 0, -q, q)


def frame_arrays(c, frame):
    b = np.array(c["frames"][frame]["bones"][:NB], np.float64)
    return b[:, 0:3], b[:, 3:7], b[:, 7:10]


# ---- independent pack reader ----------------------------------------------------------------------------------------
def cstr(b):
    return b.split(b"\x00", 1)[0].decode("utf-8")


def read_pack(blob):
    d = {}
    if blob[0:8] != b"UTPPLEAN":
        raise ValueError("magic")
    ver, hsz, rig_hash, nb, nsec, fsz, pcrc, hcrc = struct.unpack_from("<IIQIIIII", blob, 8)
    d.update(version=ver, header_size=hsz, rig_hash=rig_hash, num_bones=nb, num_sections=nsec, file_size=fsz)
    h = bytearray(blob[:64])
    h[40:44] = b"\x00\x00\x00\x00"
    d["crc_ok"] = (zlib.crc32(blob[64:]) & 0xFFFFFFFF) == pcrc and (zlib.crc32(bytes(h)) & 0xFFFFFFFF) == hcrc
    d["payload_crc"] = pcrc
    secs = {}
    for i in range(nsec):
        tag, off, size, cnt = struct.unpack_from("<4sIII", blob, 64 + 16 * i)
        secs[tag.decode()] = (off, size, cnt)
    d["secs"] = secs
    rec = dict(BONE=36, PARM=4, LAYR=240, SAMP=80, TRIS=72, DELT=NB * 40, CLIP=80, CVAL=4)
    d["sec_ok"] = all(secs[t][1] == secs[t][2] * r and secs[t][0] % 16 == 0 and secs[t][0] + secs[t][1] <= len(blob)
                      for t, r in rec.items())
    off, _, cnt = secs["BONE"]
    d["bones"] = [(cstr(blob[off + 36 * i:off + 36 * i + 32]), struct.unpack_from("<i", blob, off + 36 * i + 32)[0])
                  for i in range(cnt)]
    off, _, cnt = secs["PARM"]
    d["parm"] = list(struct.unpack_from("<%df" % cnt, blob, off))
    off, _, cnt = secs["LAYR"]
    layers = []
    for i in range(cnt):
        o = off + 240 * i
        t = struct.unpack_from("<8I", blob, o + 160)
        f = struct.unpack_from("<5f", blob, o + 192)
        layers.append(dict(id=cstr(blob[o:o + 16]), blendspace=cstr(blob[o + 16:o + 64]), base_clip=cstr(blob[o + 64:o + 128]),
                           axes=[cstr(blob[o + 128:o + 144]), cstr(blob[o + 144:o + 160])], dims=t[0], style=t[1],
                           base_frame=t[2], role=t[3], first_sample=t[4], ns=t[5], first_tri=t[6], nt=t[7],
                           min=[f[0], f[2]], max=[f[1], f[3]], interp=f[4]))
    d["layers"] = layers
    off, _, cnt = secs["SAMP"]
    d["samples"] = [dict(x=struct.unpack_from("<ff", blob, off + 80 * i)[0], y=struct.unpack_from("<ff", blob, off + 80 * i)[1],
                         frame=struct.unpack_from("<I", blob, off + 80 * i + 8)[0], clip=cstr(blob[off + 80 * i + 16:off + 80 * i + 80]))
                    for i in range(cnt)]
    off, _, cnt = secs["TRIS"]
    d["tris"] = [list(struct.unpack_from("<18i", blob, off + 72 * i)) for i in range(cnt)]
    off, size, cnt = secs["DELT"]
    d["delta"] = np.frombuffer(blob, "<f4", count=size // 4, offset=off).reshape(cnt, NB, 10).astype(np.float64)
    off, _, cnt = secs["CLIP"]
    d["clips"] = [dict(name=cstr(blob[off + 80 * i:off + 80 * i + 64]), first=struct.unpack_from("<I", blob, off + 80 * i + 64)[0],
                       count=struct.unpack_from("<I", blob, off + 80 * i + 68)[0], kind=struct.unpack_from("<I", blob, off + 80 * i + 72)[0],
                       fps=struct.unpack_from("<f", blob, off + 80 * i + 76)[0]) for i in range(cnt)]
    off, size, cnt = secs["CVAL"]
    d["cval"] = np.frombuffer(blob, "<f4", count=cnt, offset=off).astype(np.float64)
    return d


def in_circle(a, b, c, p):
    """True when p is strictly inside the circumcircle of CCW triangle abc (normalised coordinates)."""
    m = np.array([[a[0] - p[0], a[1] - p[1], (a[0] - p[0]) ** 2 + (a[1] - p[1]) ** 2],
                  [b[0] - p[0], b[1] - p[1], (b[0] - p[0]) ** 2 + (b[1] - p[1]) ** 2],
                  [c[0] - p[0], c[1] - p[1], (c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2]])
    return np.linalg.det(m) > 1e-9


def traj_f90(c):
    ys = []
    for f in c["frames"]:
        x, y, z, w = f["bones"][1][3:7]
        ys.append(math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    tot = ys[-1] - ys[0]
    return next(i for i, v in enumerate(ys) if abs(v - ys[0]) >= 0.9 * abs(tot))


def check_pack(rig, blob):
    """Independent verification of one baked pack. Returns a list of error strings."""
    E = []

    def err(msg):
        E.append(msg)

    try:
        d = read_pack(blob)
    except Exception as e:  # noqa: BLE001
        return ["unreadable: %r" % (e,)]
    cfg = lb.RIGS[rig]
    rig_path = P(cfg["rig"])
    names, parents = v2.load_rig(rig_path, NB) if hasattr(v2, "load_rig") else lb.v1.load_rig(rig_path, NB)
    rest = v2.rig_rest_local(rig_path, NB)
    want_hash, _ = v2.rig_identity_hash(names, parents, rest)
    if d["version"] != 1 or d["header_size"] != 64 or d["num_bones"] != NB or d["num_sections"] != 8:
        err("header fields")
    if d["file_size"] != len(blob) or not d["crc_ok"] or not d["sec_ok"]:
        err("size / crc / section table")
    if d["rig_hash"] != want_hash:
        err("rig hash %016x != %016x" % (d["rig_hash"], want_hash))
    if [b[0] for b in d["bones"]] != names or [b[1] for b in d["bones"]] != list(parents):
        err("BONE section differs from the rig json")

    # params vs the lean data JSON (independent of read_params)
    ld = jload(P(lb.LEAN_DATA))
    pp = ld["runtime_format"]["schema"]["params"]
    f = d["parm"]
    exp = {0: pp["lat_accel_amount"]["denominator_speed_cm_s"][0], 1: pp["lat_accel_amount"]["denominator_speed_cm_s"][1],
           2: pp["lat_accel_amount"]["denominator_accel_cm_s2"][0], 3: pp["lat_accel_amount"]["denominator_accel_cm_s2"][1],
           4: -1, 5: 1, 6: 200, 7: 320, 8: 10, 9: 0.3, 10: 0.3, 11: 5, 12: 165, 13: 375, 14: 0.5, 15: 1.0, 16: 0.2,
           17: 1.5, 18: 1.75, 19: 1.2}
    for i, v in exp.items():
        if abs(f[i] - v) > 1e-6:
            err("PARM[%d] = %g, want %g" % (i, f[i], v))
    dm = pp["direction_map"]
    for i, k in enumerate(lb.DIRECTION_ORDER):
        if f[20 + 2 * i] != dm[k][0] or f[21 + 2 * i] != dm[k][1]:
            err("direction map %s" % k)
    if any(v != 0 for v in f[32:]):
        err("PARM reserved")

    # layers vs blend space definitions + meta references
    ids = [l[0] for l in lb.LAYERS]
    if [l["id"] for l in d["layers"]] != ids:
        err("layer ids %s" % [l["id"] for l in d["layers"]])
        return E
    for lay, (lid, bsname, style, role) in zip(d["layers"], lb.LAYERS):
        bs = jload(P(lb.BS_DIR, bsname + ".json"))
        if lay["blendspace"] != bsname or lay["dims"] != len(bs["axes"]) or lay["role"] != role or lay["style"] != lb.STYLE_ID[style]:
            err("%s: layer header" % lid)
        if abs(lay["interp"] - bs["target_weight_interp_speed"]) > 1e-6:
            err("%s: interp" % lid)
        for a, ax in enumerate(bs["axes"]):
            if lay["min"][a] != ax["min"] or lay["max"][a] != ax["max"]:
                err("%s: axis %d range" % (lid, a))
        smp = d["samples"][lay["first_sample"]:lay["first_sample"] + lay["ns"]]
        if len(smp) != len(bs["samples"]):
            err("%s: sample count" % lid)
            continue
        metadir = P(lb.META[style])
        refs = set()
        for s, bsamp in zip(smp, bs["samples"]):
            if s["clip"] != bsamp["animation"] or s["x"] != bsamp["x"] or (lay["dims"] == 2 and s["y"] != bsamp["y"]) or s["frame"] != bsamp["frame_index_to_sample"]:
                err("%s: sample %s" % (lid, bsamp["animation"]))
            m = jload(os.path.join(metadir, s["clip"] + ".json"))["additive"]
            refs.add((m["base"]["seq"], m["base"]["frame"]))
        if refs != {(lay["base_clip"], lay["base_frame"])}:
            err("%s: reference pose %s vs layer %s" % (lid, refs, (lay["base_clip"], lay["base_frame"])))

        # deltas recomputed independently from the full retargeted poses
        dirkey = "relaxed" if style == "Relaxed" else "neutral"
        A = clip(dirkey, rig, lay["base_clip"])
        tA, qA, sA = frame_arrays(A, lay["base_frame"])
        gA = mesh_rotations(qA, parents)
        for k, s in enumerate(smp):
            X = clip(dirkey, rig, s["clip"])
            tX, qX, sX = frame_arrays(X, s["frame"])
            gX = mesh_rotations(qX, parents)
            dq = canon(qmul(gX, qconj(gA)))
            stored = d["delta"][lay["first_sample"] + k]
            sq = stored[:, 0:4]
            if np.abs(np.where((sq * dq).sum(-1, keepdims=True) < 0, -sq, sq) - dq).max() > TOL_Q:
                err("%s: sample %d (%s) dq differs from G_X * inv(G_A)" % (lid, k, s["clip"]))
            if np.abs(stored[:, 4:7] - (tX - tA)).max() > TOL_T:
                err("%s: sample %d dt" % (lid, k))
            if np.abs(stored[:, 7:10] - (sX / sA - 1.0)).max() > TOL_T:
                err("%s: sample %d ds" % (lid, k))
            # the delta applied to the reference reproduces the target (mesh rotation)
            rec = canon(qmul(sq, gA))
            tgt = canon(gX)
            if (np.abs(np.where((rec * tgt).sum(-1, keepdims=True) < 0, -rec, rec) - tgt)).max() > 5 * TOL_Q:
                err("%s: sample %d dq * G_A != G_X" % (lid, k))
            if s["clip"] == lay["base_clip"] and s["frame"] == lay["base_frame"] and np.abs(sq - np.array([0, 0, 0, 1.0])).max() > TOL_Q:
                err("%s: reference sample is not identity" % lid)

        # triangulation: CCW, empty circumcircles, tiles the hull, edge info consistent
        tris = d["tris"][lay["first_tri"]:lay["first_tri"] + lay["nt"]]
        if lay["dims"] == 1:
            xs = [s["x"] for s in smp]
            order = sorted(range(len(xs)), key=lambda i: xs[i])
            if [t[:3] for t in tris] != [[order[i], order[i + 1], order[i + 1]] for i in range(len(order) - 1)]:
                err("%s: 1D segments" % lid)
            continue
        nrm = [((s["x"] - lay["min"][0]) / (lay["max"][0] - lay["min"][0]), (s["y"] - lay["min"][1]) / (lay["max"][1] - lay["min"][1])) for s in smp]
        area = 0.0
        for ti, t in enumerate(tris):
            a, b, c = [nrm[i] for i in t[:3]]
            ar = 0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            if ar <= 1e-12:
                err("%s: triangle %d not CCW" % (lid, ti))
            area += ar
            for pi, p in enumerate(nrm):
                if pi not in t[:3] and in_circle(a, b, c, p):
                    err("%s: triangle %d circumcircle contains sample %d (not Delaunay)" % (lid, ti, pi))
            for e in range(3):
                nb_ = t[3 + 5 * e]
                s0, s1 = t[e], t[(e + 1) % 3]
                rev = [i for i, u in enumerate(tris) if any(u[w] == s1 and u[(w + 1) % 3] == s0 for w in range(3))]
                if (rev[0] if rev else -1) != nb_:
                    err("%s: triangle %d edge %d neighbour %d != %s" % (lid, ti, e, nb_, rev))
        pts = np.array(nrm)
        hull = convex_area(pts)
        if abs(area - hull) > 1e-9:
            err("%s: triangles tile area %.6f != hull %.6f" % (lid, area, hull))

    # disable curves
    clips = d["clips"]
    srcs = [c for c in clips if c["kind"] == 0]
    neu = [c for c in clips if c["kind"] == 1]
    dc = ld["disable_clips"]
    if [c["name"] for c in srcs] != [x["clip"] for x in dc] or len(neu) != 8 or len(clips) != 24:
        err("curve clip table (%d src, %d neutral)" % (len(srcs), len(neu)))
        return E
    for c in srcs:
        raw = clip("relaxed", rig, c["name"])["curves"][lb.DISABLE_CURVE]
        want = np.clip(np.array(raw, np.float64), 0, 1)
        want[want < 1e-6] = 0.0
        got = d["cval"][c["first"]:c["first"] + c["count"]]
        if c["fps"] != 30.0 or len(got) != len(want) or np.abs(got - want).max() > 1e-6:
            err("curve %s differs from the clip curve" % c["name"])
    for c in neu:
        n = c["name"]
        if not os.path.isfile(os.path.join(P(cfg["neutral"]), n + ".utppclip.json")):
            err("neutral target clip missing: " + n)
            continue
        nc = clip("neutral", rig, n)
        if nc["num_frames"] != c["count"]:
            err("neutral curve length " + n)
        got = d["cval"][c["first"]:c["first"] + c["count"]]
        # source: Relaxed clip with the same turn; time scale = f90 ratio of the Trajectory yaw
        p = n.split("_")
        cand = "M_Relaxed_%s_Turn_%s_090_%s" % (p[2], p[4], p[6]) if p[2] == "Run" else "M_Relaxed_Walk_Turn_090_%s_%s" % (p[4], p[6])
        srcc = next((x for x in srcs if x["name"] == cand), None)
        if srcc is None:
            err("no Relaxed source for " + n)
            continue
        sv = d["cval"][srcc["first"]:srcc["first"] + srcc["count"]]
        scale = traj_f90(clip("relaxed", rig, cand)) / traj_f90(nc)
        f_ = np.minimum(np.arange(c["count"]) * scale, len(sv) - 1)
        i0 = f_.astype(int)
        i1 = np.minimum(len(sv) - 1, i0 + 1)
        want = sv[i0] * (1 - (f_ - i0)) + sv[i1] * (f_ - i0)
        if np.abs(got - want).max() > 1e-6 or abs(got[0] - 1.0) > 1e-6 or got[-1] != 0.0:
            err("neutral curve %s differs from the f90-scaled source (scale %.3f)" % (n, scale))
        if not 0.8 < scale < 1.5:
            err("implausible time scale %.3f for %s" % (scale, n))
    return E


def convex_area(pts):
    pts = sorted(map(tuple, pts))

    def half(ps):
        h = []
        for p in ps:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h
    hull = half(pts)[:-1] + half(pts[::-1])[:-1]
    return 0.5 * abs(sum(hull[i][0] * hull[(i + 1) % len(hull)][1] - hull[(i + 1) % len(hull)][0] * hull[i][1] for i in range(len(hull))))


def bake_to(rig, tmp):
    s = lb.bake(rig, ROOT, tmp)
    with open(s["bin"], "rb") as f:
        blob = f.read()
    with open(s["json"], encoding="utf-8") as f:
        js = f.read()
    return blob, js, s


class LeanBakeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="leanbake_test_")
        cls.packs = {}
        for rig in ("woman", "man"):
            cls.packs[rig] = bake_to(rig, os.path.join(cls.tmp, rig))

    def test_pack_checks_clean(self):
        for rig in ("woman", "man"):
            self.assertEqual(check_pack(rig, self.packs[rig][0]), [], rig)

    def test_deterministic_and_matches_produced_data(self):
        for rig in ("woman", "man"):
            blob2, js2, _ = bake_to(rig, os.path.join(self.tmp, rig + "_again"))
            self.assertEqual(blob2, self.packs[rig][0], rig + " bake is not deterministic")
            self.assertEqual(js2, self.packs[rig][1])
            for ext, idx in (("bin", 0), ("json", 1)):
                path = P("Data", "utpp_lean_%s.%s" % (rig, ext))
                if os.path.isfile(path):
                    cur = open(path, "rb").read()
                    want = self.packs[rig][idx] if idx == 0 else self.packs[rig][1].encode("utf-8")
                    # the JSON header embeds nothing machine specific except relative names, so it must match too
                    self.assertEqual(cur, want, "Data/utpp_lean_%s.%s differs from a fresh bake" % (rig, ext))

    def test_structure_numbers(self):
        d = read_pack(self.packs["woman"][0])
        self.assertEqual([(l["id"], l["ns"], l["nt"]) for l in d["layers"]], [("run", 5, 4), ("walk", 5, 4), ("head", 13, 12), ("cmc_run", 3, 2)])
        self.assertEqual(len(d["clips"]), 24)
        self.assertNotEqual(read_pack(self.packs["man"][0])["rig_hash"], d["rig_hash"])
        h = json.loads(self.packs["woman"][1])
        self.assertEqual(h["schema"], "utpp.lean/1")
        self.assertEqual(len(h["disable_curves"]["neutral_mute_map"]), 8)
        self.assertEqual(len(h["disable_curves"]["relaxed_clips_without_neutral_equivalent"]), 8)
        self.assertEqual(h["binary"]["sha256"], hashlib.sha256(self.packs["woman"][0]).hexdigest())

    def test_cross_layer_is_a_fan_around_base(self):
        d = read_pack(self.packs["woman"][0])
        for lay in d["layers"][:2]:
            tris = d["tris"][lay["first_tri"]:lay["first_tri"] + lay["nt"]]
            self.assertEqual(len(tris), 4)
            self.assertTrue(all(0 in t[:3] for t in tris), "every triangle of the 5-sample cross contains the Base sample")


class Mutations(unittest.TestCase):
    """Each deliberately broken baker must be caught by check_pack."""

    def _caught(self, rig="woman"):
        with tempfile.TemporaryDirectory(prefix="leanbake_mut_") as tmp:
            blob, _, _ = bake_to(rig, tmp)
        return check_pack(rig, blob)

    def _patched(self, obj, name, repl):
        old = getattr(obj, name)
        setattr(obj, name, repl)
        try:
            return self._caught()
        finally:
            setattr(obj, name, old)

    def test_delta_order_swapped(self):
        real = lb.ob.delta

        def bad(X, Xf, A, Af):
            out = real(X, Xf, A, Af).copy()
            out[:, :, 0:4] = lb.ob.qcanon(lb.v1.qmul(lb.ob.qinv(A.GQ[Af]), X.GQ[Xf[0]]))
            return out
        self.assertTrue(self._patched(lb.ob, "delta", bad))

    def test_no_delaunay_flips(self):
        self.assertTrue(self._patched(lb.UEDelaunay, "circumcircle", lambda self, t, test: -1))

    def test_no_neighbour_links(self):
        self.assertTrue(self._patched(lb.UEDelaunay, "find_tri_with_edge", lambda self, a, b: -1))

    def test_curve_time_scale_ignored(self):
        real = lb.resample_curve
        self.assertTrue(self._patched(lb, "resample_curve", lambda cur, n, scale=1.0: real(cur, n, 1.0)))

    def test_param_constant_changed(self):
        real = lb.read_params

        def bad(root):
            f, d = real(root)
            f[1] = 300.0
            return f, d
        self.assertTrue(self._patched(lb, "read_params", bad))

    def test_neutral_mapping_dropped(self):
        self.assertTrue(self._patched(lb, "neutral_equivalent", lambda name: None))

    def test_curve_noise_not_clamped(self):
        real = lb.load_json

        def bad(p):
            j = real(p)
            if isinstance(j, dict) and "curves" in j and lb.DISABLE_CURVE in j["curves"]:
                j["curves"][lb.DISABLE_CURVE] = [v * 0.5 for v in j["curves"][lb.DISABLE_CURVE]]
            return j
        self.assertTrue(self._patched(lb, "load_json", bad))


if __name__ == "__main__":
    unittest.main(verbosity=2)
