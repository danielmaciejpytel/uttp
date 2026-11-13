"""Shared helpers of the traversal / jump generation pipeline (trav_*.py): scenes of source pack pieces with MotionBricks gaps.

World dumps: mannequin_dump.py npz (W [T,B,4,4] cm, Z up, character faces -Y). A scene is built from pieces (frame ranges of
dumps, rigidly placed: heading -> -Y, pelvis xy at a given point, z shift) and gaps (4N-8 placeholder frames, N = 6..16).
Gap i: c = last frame of the piece before it, g = c + 4N - 7 = first frame of the piece after it (clip_inbetween.py layout).
numpy / scipy only (kimodo env python, run with -I). Import only: skeleton_bridge.py, foot_pin.py, compose_transition.py.
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
TP = os.path.join(HERE, 'data', 'body_tpose_generator.npz')
WORK = os.path.join(ROOT, 'Build', 'work', 'trav-mb-20261106')
DUMPS = os.path.join(WORK, 'dump')
BLENDER = os.environ.get('UTPP_BLENDER') or os.path.join(os.environ.get('ProgramFiles(x86)', ''), 'Steam', 'steamapps', 'common', 'Blender', 'blender.exe')
SOURCEPACK_FBX = os.path.join(ROOT, 'Animations', 'sourcepack', 'als_export', 'fbx')
SOURCEPACK_META = os.path.join(ROOT, 'Animations', 'retarget', 'meta_sourcepack')
FWD = np.array([0.0, -1.0, 0.0])

from skeleton_bridge import load_bridge  # noqa: E402
from foot_pin import Skel, contacts, segments  # noqa: E402


class Dump:
    def __init__(self, name):
        d = np.load(os.path.join(DUMPS, name + '.dump.npz'))
        self.name = name
        self.names = [str(x) for x in d['names']]
        self.parents = d['parents']
        self.rest = d['rest']
        self.W = d['W']
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.pel = self.idx['pelvis']

    def heading(self, f):
        v = (self.W[f, self.pel, :3, :3] @ self.rest[self.pel, :3, :3].T @ FWD)[:2]
        return v / np.linalg.norm(v)

    def pel_pos(self, f):
        return self.W[f, self.pel, :3, 3]


def rigid(W, pivot_xy, ang, new_xy, dz):
    """Rotate world matrices about the vertical through pivot_xy by ang, move pivot_xy to new_xy, add dz."""
    c, s = math.cos(ang), math.sin(ang)
    M = np.eye(4)
    M[:2, :2] = [[c, -s], [s, c]]
    M[:2, 3] = np.asarray(new_xy) - M[:2, :2] @ np.asarray(pivot_xy)
    M[2, 3] = dz
    return M[None, None] @ W


def build_scene(items, want=(0.0, -1.0)):
    """items: dicts {kind:'piece', src, a, b, ref, xy (cm), dz (cm), tag, [yaw_deg]} / {kind:'gap', N}.
    Returns dict(W, names, parents, rest, spans [(tag, s0, s1, src, a, b)], gaps [{c, N, g}])."""
    Ws, spans, gaps = [], [], []
    base = None
    n = 0
    for it in items:
        if it['kind'] == 'gap':
            N = int(it['N'])
            c = n - 1
            Ws.append(np.repeat(Ws[-1][-1:], 4 * N - 8, 0))
            n += 4 * N - 8
            gaps.append(dict(c=c, N=N, g=c + 4 * N - 7))
            continue
        if it['kind'] == 'raw':
            Ws.append(it['W'])
            spans.append((it.get('tag', ''), n, n + len(it['W']) - 1, it.get('src', 'blend'), 0, len(it['W']) - 1))
            n += len(it['W'])
            continue
        D = Dump(it['src'])
        base = base or D
        a, b, ref = int(it['a']), int(it['b']), int(it['ref'])
        hd = D.heading(ref)
        ang = math.atan2(want[1], want[0]) - math.atan2(hd[1], hd[0]) + math.radians(it.get('yaw_deg', 0.0))
        W = rigid(D.W[a:b + 1], D.W[ref, D.pel, :2, 3], ang, it['xy'], it.get('dz', 0.0))
        Ws.append(W)
        spans.append((it.get('tag', ''), n, n + len(W) - 1, it['src'], a, b))
        n += len(W)
    for gp in gaps:
        assert gp['g'] < n, 'gap beyond the scene'
    return dict(W=np.concatenate(Ws), names=base.names, parents=base.parents, rest=base.rest, spans=spans, gaps=gaps)


def save_scene(sc, path):
    np.savez(path, W=sc['W'], names=np.array(sc['names']), parents=sc['parents'], rest=sc['rest'], fps=30)
    json.dump(dict(spans=sc['spans'], gaps=sc['gaps']), open(path[:-4] + '.json', 'w'), indent=1)


def to_soma(dump_path, out_path):
    br, d = load_bridge(TP, dump_path)
    G, hips, pos = br.fwd(d['W'])
    np.savez(out_path, G=G, hips=hips, pos=pos, names=np.array(br.snames), k=br.k, fps=30)
    return br


def fk_all(sk, G, hips):
    return np.array([sk.fk(G[t], hips[t]) for t in range(len(G))])


def pin_span(span_npz, scene_soma, out_npz, labels=True):
    """foot_pin.py (generator contact labels), then pin_low: a second pass on every run of >= 4 frames with the lowest foot joint
    within 3 cm of its floor level, whatever its speed (the speed-gated rule leaves a foot that skates at 1-3 m/s unpinned)."""
    import foot_pin
    a = [span_npz, TP, out_npz, '--ground', scene_soma]
    if labels:
        a.append('--labels')
    foot_pin.main(a)
    pin_low(out_npz, scene_soma, out_npz)


def pin_low(span_npz, scene_soma, out_npz, thr=0.03, minlen=4):
    import foot_pin as FP
    sk = FP.Skel(TP)
    s = np.load(span_npz)
    G, hips = s['G'].astype(np.float64).copy(), s['hips'].astype(np.float64)
    P = fk_all(sk, G, hips)
    sc = np.load(scene_soma)
    ground = fk_all(sk, sc['G'].astype(np.float64), sc['hips'].astype(np.float64))[:, :, 1].min(0)
    n_pinned = 0
    for side in ('Left', 'Right'):
        a, t = sk.idx[side + 'Foot'], sk.idx[side + 'ToeBase']
        ha, ht = P[:, a, 1] - ground[a], P[:, t, 1] - ground[t]
        piv = np.where(ha <= ht, a, t)
        mask = np.minimum(ha, ht) < thr
        for s0, e0 in segments(mask):
            if e0 - s0 + 1 < minlen:
                mask[s0:e0 + 1] = False
        off = FP.pin_offsets(P, piv, mask)
        for t_ in range(1, len(G) - 1):
            if np.linalg.norm(off[t_]) > 1e-5:
                FP.leg_ik(sk, G[t_], hips[t_], side, P[t_, a] + off[t_])
                n_pinned += 1
    np.savez(out_npz, G=G, hips=hips, **{k: s[k] for k in s.files if k not in ('G', 'hips')})
    return n_pinned


def foot_contact_masks(P, sk, ground, h=0.05, v=1.0):
    """{side: (mask [T], pivot joint [T])} with the foot_pin rule (lowest of ankle / toe near the floor and slow)."""
    out = {}
    for side in ('Left', 'Right'):
        out[side] = contacts(P, sk, side, ground, h, v)
    return out


def runs(mask):
    return [(int(s), int(e)) for s, e in segments(mask)]


def smooth01(m):
    x = np.asarray(m, dtype=float)
    k = np.array([0.25, 0.5, 0.25])
    y = np.convolve(np.r_[x[0], x, x[-1]], k, mode='valid')
    return np.clip(y, 0.0, 1.0)


def meta_curve(src, name):
    p = os.path.join(SOURCEPACK_META, src + '.json')
    if not os.path.isfile(p):
        return None
    c = json.load(open(p, encoding='utf-8')).get('curves', {}).get(name)
    return c


# ---------------------------------------------------------------- local-space blend of two placed windows
def blend_windows(Wa, Wb, parents, s):
    """Blend two world-matrix windows [n,B,4,4] (same placement) in local space, parents first. s: scalar or [n] weights per frame.
    Rotation: ra * exp(s * log(ra^-1 rb)) with the relative rotation vector unwrapped over time (a bone whose relative rotation crosses 180 deg
    keeps one continuous path instead of flipping to the other side between frames); translation: lerp."""
    from scipy.spatial.transform import Rotation as Rot
    n, nb = Wa.shape[:2]
    sv = np.broadcast_to(np.asarray(s, dtype=float), (n,)).copy()
    out = np.zeros_like(Wa)
    for b in range(nb):
        p = int(parents[b])
        La = Wa[:, b] if p < 0 else np.linalg.inv(Wa[:, p]) @ Wa[:, b]
        Lb = Wb[:, b] if p < 0 else np.linalg.inv(Wb[:, p]) @ Wb[:, b]
        L = np.zeros_like(La)
        L[:, 3, 3] = 1.0
        L[:, :3, 3] = (1 - sv[:, None]) * La[:, :3, 3] + sv[:, None] * Lb[:, :3, 3]
        ra = Rot.from_matrix(np.array([_ortho(m) for m in La[:, :3, :3]]))
        rb = Rot.from_matrix(np.array([_ortho(m) for m in Lb[:, :3, :3]]))
        rv = (ra.inv() * rb).as_rotvec()  # [n,3], angle in [0, pi]
        for t in range(1, n):  # continuity over time: take the long path when it is closer to the previous frame's vector
            ang = np.linalg.norm(rv[t])
            if ang > 1e-9:
                alt = rv[t] - rv[t] / ang * 2.0 * np.pi
                if np.linalg.norm(alt - rv[t - 1]) < np.linalg.norm(rv[t] - rv[t - 1]):
                    rv[t] = alt
        L[:, :3, :3] = (ra * Rot.from_rotvec(sv[:, None] * rv)).as_matrix()
        out[:, b] = L if p < 0 else out[:, p] @ L
    return out


def _ortho(m):
    u, _, vt = np.linalg.svd(m)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def place_window(D, a, b, ref, xy, dz=0.0, want=(0.0, -1.0)):
    hd = D.heading(ref)
    ang = math.atan2(want[1], want[0]) - math.atan2(hd[1], hd[0])
    return rigid(D.W[a:b + 1], D.W[ref, D.pel, :2, 3], ang, xy, dz)
