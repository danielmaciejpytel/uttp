"""Arm-through-torso screen (round F, review M4): the redo_pen_check criterion with POSE-CONSISTENT labels, skinned and as a fast rigid proxy.

  python -I redo_pen.py reference [--rig man] [--out FILE]                      skinned count (pose labels) on the reference skins of BB.REDO\\skin_ref (read-only)
  python -I redo_pen.py calibrate [--rig man] [--out FILE]                      proxy vs skinned count on every demo skin of mb4(_man)\\demo3 (read-only)
  python -I redo_pen.py batch <clips dir> [--kind v3|v4] [--out FILE] [--rig man]  proxy prevalence over a clip folder
Criterion (redo_pen_check.py): per frame the number of ARM vertices (ForeArm / Hand / fingers) closer than PEN_M = 1 cm to a TORSO vertex (Hips / Spine /
Spine1 / Spine2 / Spine3 / Neck); upper-arm vertices are skipped (armpit contact is normal).
Label fix (round F): redo_pen_check.labels labels the skin's 'rest' vertices by the nearest REST-POSE (T-pose) joint, but 'rest' of a skin cache is its posed
FRAME 0 (rest == pc2[0], verified), so the arms of a posed mesh were hardly ever labelled ARM (base_ready: 8 ARM vertices of 22 554; fists combo_attack_01
redo: 0) and its counts / pen_ref.json are not a valid measure.  Here a vertex gets the label of the nearest BONE SEGMENT (parent -> child, labelled by the
parent; leaf joints as points) of the dump's frame 0, i.e. the same pose as the vertices.
Proxy: the frame-0 vertices of a reference skin of the rig (BB.REDO\\skin_ref\\base_ready, read-only) move RIGIDLY with their label joint
(W[t, j] @ inv(W0[j])); same count with a cKDTree.  Threshold = the maximum of the skinned count (pose labels) over the reference skins + PEN_SLACK.
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import base_body as BB  # noqa: E402
import melee_common as MC  # noqa: E402

PEN_M = 0.01
TORSO = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck']
SKIP = ('Root', 'Trajectory', 'reference_joint')
REF_DIR = os.path.join(BB.REDO, 'skin_ref')
REF_KEYS = ('base_ready', 'base_idle', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F', 'AO_Pistol2H_Stand_Sweep', 'overlay_poses')
PEN_SLACK = 20     # calibrate (2026-11-09): the skinned count has a constant 1-31 vertex shell contact in many clips (graph deformation), the references 0; at 20 vertices proxy vs skinned agree on penetration (woman precision 0.74 recall 0.57 incl. the vanilla FPP clips, man 0.86 / 1.0; at 50: 0.97 / 0.97 and 0.77 / 0.95)
_P = None


def _anat(n):
    return not (n in SKIP or 'Weapon' in n or 'InHand' in n or n.endswith('HandEnd') or 'Eye' in n)


def segments(names, par):
    """(a, b, label) joint index triples: every anatomical bone parent -> child labelled by the parent, every anatomical joint also as a point."""
    out = []
    for j, n in enumerate(names):
        if not _anat(n):
            continue
        out.append((j, j, j))
        p = par[j]
        if p >= 0 and _anat(names[p]):
            out.append((p, j, p))
    return out


def label(V, W0, names, par):
    """V [N,3] vertices in the pose W0 [J,4,4] -> joint index per vertex (nearest bone segment of the same pose)."""
    segs = segments(names, par)
    P = W0[:, :3, 3]
    best, lab = np.full(len(V), np.inf), np.zeros(len(V), int)
    for a, b, l in segs:
        A, Bp = P[a], P[b]
        ab = Bp - A
        L2 = float(ab @ ab)
        t = np.clip(((V - A) @ ab) / L2, 0, 1) if L2 > 1e-12 else np.zeros(len(V))
        d = np.linalg.norm(V - (A + t[:, None] * ab), axis=1)
        m = d < best
        best[m], lab[m] = d[m], l
    return lab


def groups(lab, names):
    tor = np.isin(np.array(names)[lab], TORSO)
    arm = np.array([('ForeArm' in names[j] or 'Hand' in names[j]) for j in lab])
    return np.where(tor)[0], np.where(arm)[0]


def count_frames(vt, tor, arm):
    d, _ = cKDTree(vt[tor]).query(vt[arm], distance_upper_bound=PEN_M)
    return int((d < PEN_M).sum())


def skinned(skin_base, dump_W):
    """skinned count with pose labels: skin_base (.npz / .pc2 of demo_skin.skin), dump_W = the joint transforms the skin was made from."""
    z = np.load(skin_base + '.npz')
    v0 = z['rest'].astype(float)
    pc = AG._read_pc2(skin_base + '.pc2', len(v0))
    names, par = BB.rig()[0], [int(p) for p in BB.rig()[1]]
    lab = label(v0, np.asarray(dump_W[0], float), names, par)
    tor, arm = groups(lab, names)
    return np.array([count_frames(pc[t], tor, arm) for t in range(min(len(pc), len(dump_W)))], int)


class Proxy:
    def __init__(self, ref='base_ready'):
        z = np.load(os.path.join(REF_DIR, ref + '.skin.npz'))
        self.v0 = z['rest'].astype(float)
        W0 = np.load(os.path.join(REF_DIR, ref + '.dump.npz'))['W'][0].astype(float)
        self.names = [str(n) for n in BB.rig()[0]]
        self.par = [int(p) for p in BB.rig()[1]]
        self.lab = label(self.v0, W0, self.names, self.par)
        self.tor, self.arm = groups(self.lab, self.names)
        self.Minv = np.linalg.inv(W0)
        self.vh = np.concatenate([self.v0, np.ones((len(self.v0), 1))], 1)
        rp = os.environ.get('UTPP_PEN_REF') or os.path.join(MC.WORK, 'f_work', 'pen_ref_pose_%s.json' % BB.RIG_NAME)
        self.ref_max = json.load(open(rp))['summary']['max'] if os.path.exists(rp) else 0
        self.thr = self.ref_max + PEN_SLACK

    def verts(self, Wt):
        M = np.asarray(Wt, float) @ self.Minv
        return np.einsum('nij,nj->ni', M[self.lab], self.vh)[:, :3]

    def counts(self, W, frames=None):
        return np.array([count_frames(self.verts(W[t]), self.tor, self.arm) for t in (range(len(W)) if frames is None else frames)], int)


def proxy():
    global _P
    if _P is None:
        _P = Proxy()
    return _P


def pen_frames(W, frames=None):
    """-> (proxy counts [n], penetrating [n] bool = count > threshold of the rig)."""
    P = proxy()
    c = P.counts(W, frames)
    return c, c > P.thr


# ------------------------------------------------------------------ torso SURFACE model for the solver (round F step 3; the gate measures the AXIS only)
SURF_BINS = 14
_TM = None


def torso_frame(Wt, ix):
    """origin Hips, unit axis Hips -> Neck (length L), lateral (LeftUpLeg -> RightUpLeg, orthogonalised), forward = axis x lateral oriented to the toes (rig)."""
    ha, hn = Wt[ix['Hips'], :3, 3], Wt[ix['Neck'], :3, 3]
    ax = hn - ha
    L = float(np.linalg.norm(ax))
    u = ax / L
    lat = Wt[ix['RightUpLeg'], :3, 3] - Wt[ix['LeftUpLeg'], :3, 3]
    lat = lat - (lat @ u) * u
    lat /= np.linalg.norm(lat)
    fwd = np.cross(u, lat)
    return ha, u, L, lat, fwd


def torso_model():
    """per-rig cross-section profile of the TORSO vertices (pose labels) of the base_ready reference skin: per height bin (fraction of Hips -> Neck) the
    lateral half-width a and the front / back depths (p98), plus the forearm radius (p75 of ForeArm vertex distances to the bone)."""
    global _TM
    if _TM is not None:
        return _TM
    P = proxy()
    W0 = np.load(os.path.join(REF_DIR, 'base_ready.dump.npz'))['W'][0].astype(float)
    ix = {n: i for i, n in enumerate(P.names)}
    ha, u, L, lat, fwd = torso_frame(W0, ix)
    toe = W0[ix['LeftToeBase'], :3, 3] - W0[ix['LeftFoot'], :3, 3]
    sgn = 1.0 if (toe - (toe @ u) * u) @ fwd > 0 else -1.0
    V = P.v0[P.tor] - ha
    h, x, y = V @ u / L, V @ lat, sgn * (V @ fwd)
    edges = np.linspace(np.percentile(h, 0.5), np.percentile(h, 99.5), SURF_BINS + 1)
    prof = []
    for k in range(SURF_BINS):
        m = (h >= edges[k]) & (h <= edges[k + 1])
        prof.append((float(np.percentile(np.abs(x[m]), 98)), float(np.percentile(np.maximum(y[m], 0), 98)), float(np.percentile(np.maximum(-y[m], 0), 98))))
    fa = [i for i in P.arm if 'ForeArm' in P.names[P.lab[i]]]
    j, k = ix['LeftForeArm'], ix['LeftHand']
    rad = []
    for side in ('Left', 'Right'):
        A, Bq = W0[ix[side + 'ForeArm'], :3, 3], W0[ix[side + 'Hand'], :3, 3]
        idx = [i for i in fa if P.names[P.lab[i]] == side + 'ForeArm']
        Vv = P.v0[idx]
        ab = Bq - A
        t = np.clip((Vv - A) @ ab / (ab @ ab), 0, 1)
        rad.append(np.linalg.norm(Vv - (A + t[:, None] * ab), axis=1))
    _TM = dict(edges=edges, prof=np.array(prof), sgn=sgn, limb_r=float(np.percentile(np.concatenate(rad), 75)), margin=0.0)
    return _TM


def surface_clear(q, frame, tm=None):
    """signed clearance (m) of points q [n,3] from the torso surface + forearm radius (< 0 = inside) and the outward unit directions (perpendicular to the
    torso axis).  frame = torso_frame(...).  Points outside the torso height range get +inf."""
    tm = tm or torso_model()
    ha, u, L, lat, fwd = frame
    rel = q - ha
    h = rel @ u / L
    x, y = rel @ lat, tm['sgn'] * (rel @ fwd)
    e = tm['edges']
    k = np.clip(np.searchsorted(e, h) - 1, 0, len(tm['prof']) - 1)
    a, bf, bb = tm['prof'][k, 0], tm['prof'][k, 1], tm['prof'][k, 2]
    b = np.where(y >= 0, bf, bb)
    r = np.hypot(x, y)
    th = np.arctan2(y, x)
    re = 1.0 / np.sqrt((np.cos(th) / np.maximum(a, 1e-3)) ** 2 + (np.sin(th) / np.maximum(b, 1e-3)) ** 2)
    cl = r - re - tm['limb_r'] - tm['margin']
    cl = np.where((h < e[0]) | (h > e[-1]), np.inf, cl)
    perp = rel - (rel @ u)[:, None] * u[None]
    n = np.linalg.norm(perp, axis=1, keepdims=True)
    return cl, perp / np.maximum(n, 1e-9)


def forearm_points(Wt, ix, side, n=6):
    """sample points of the forearm (elbow -> wrist) and the hand (wrist -> 0.35 x forearm beyond the wrist, the fist / palm)."""
    A, Bq = Wt[ix[side + 'ForeArm'], :3, 3], Wt[ix[side + 'Hand'], :3, 3]
    w = np.linspace(0.0, 1.35, n + 3)
    return A[None] + w[:, None] * (Bq - A)[None]


def surface_report(W):
    """per frame the minimum surface clearance (cm) of the forearm / hand points, both sides."""
    ix = {n: i for i, n in enumerate(BB.rig()[0])}
    out = np.zeros(len(W))
    for t in range(len(W)):
        fr = torso_frame(W[t], ix)
        out[t] = min(float(surface_clear(forearm_points(W[t], ix, s), fr)[0].min()) for s in ('Left', 'Right')) * 100
    return out


def reference():
    rows = {}
    for k in REF_KEYS:
        sk = os.path.join(REF_DIR, k + '.skin')
        if not os.path.exists(sk + '.pc2'):
            continue
        W = np.load(os.path.join(REF_DIR, k + '.dump.npz'))['W']
        c = skinned(sk, W)
        rows[k] = dict(frames=len(c), max=int(c.max()), p95=int(np.percentile(c, 95)), proxy_max=int(proxy().counts(W).max()))
        print(k, rows[k], flush=True)
    return dict(summary=dict(rig=BB.RIG_NAME, max=max(r['max'] for r in rows.values()), proxy_max=max(r['proxy_max'] for r in rows.values())), rows=rows)


def calibrate():
    """proxy vs skinned count (pose labels) on the demo skins of the rig (mb4 / mb4_man demo3: orig / redo / v4 / v4a), read-only."""
    P = proxy()
    root = os.path.join(MC.WORK, 'mb4' if BB.RIG_NAME == 'woman' else 'mb4_man', 'demo3')
    rows, allp, allr = [], [], []
    for d in sorted(glob.glob(os.path.join(root, '*__*'))):
        s, c = os.path.basename(d).split('__')
        for v in ('orig', 'redo', 'v4', 'v4a'):
            sk = os.path.join(d, v + '.skin')
            if not os.path.exists(sk + '.pc2'):
                continue
            W = MC.load_dump(MC.clip_path(s, c))['W'] if v == 'orig' else np.load(os.path.join(d, v + '.dump.npz'))['W']
            real = skinned(sk, W)
            px = P.counts(W)[:len(real)]
            allp.append(px)
            allr.append(real)
            rr, rp = real > P.ref_max, px > P.thr
            rows.append(dict(clip='%s/%s' % (s, c), variant=v, frames=len(real), real_max=int(real.max()), proxy_max=int(px.max()), real_frames=int(rr.sum()),
                             proxy_frames=int(rp.sum()), both=int((rr & rp).sum()), real_only=int((rr & ~rp).sum()), proxy_only=int((~rr & rp).sum())))
            print(rows[-1], flush=True)
    p, r = np.concatenate(allp), np.concatenate(allr)
    rr, rp = r > P.ref_max, p > P.thr
    sm = dict(rig=BB.RIG_NAME, frames=len(p), ref_max=P.ref_max, thr=P.thr, corr=round(float(np.corrcoef(p, r)[0, 1]), 3) if p.std() > 0 and r.std() > 0 else None,
              real_pen_frames=int(rr.sum()), proxy_pen_frames=int(rp.sum()), both=int((rr & rp).sum()), real_only=int((rr & ~rp).sum()), proxy_only=int((~rr & rp).sum()),
              recall=round(float((rr & rp).sum() / max(rr.sum(), 1)), 3), precision=round(float((rr & rp).sum() / max(rp.sum(), 1)), 3))
    tab = {}
    for th in (0, 5, 10, 20, 30, 50, 100):      # agreement when both sides use the same vertex threshold (touching shell vs penetration)
        a_, b_ = r > th, p > th
        tab[str(th)] = dict(real=int(a_.sum()), proxy=int(b_.sum()), both=int((a_ & b_).sum()), recall=round(float((a_ & b_).sum() / max(a_.sum(), 1)), 3),
                            precision=round(float((a_ & b_).sum() / max(b_.sum(), 1)), 3))
    sm['by_threshold'] = tab
    print('CALIBRATE', json.dumps(sm))
    for x, pr_, re_ in zip(rows, allp, allr):
        x['proxy'], x['real'] = [int(v) for v in pr_], [int(v) for v in re_]
    return dict(summary=sm, rows=rows)


def batch(d, kind='v3'):
    rows = {}
    for p in sorted(glob.glob(os.path.join(d, '*.%s.npz' % kind))):
        s, c = os.path.basename(p)[:-len('.%s.npz' % kind)].split('__')
        if MC.RIG_OF_SET.get(s) != BB.RIG_NAME:
            continue
        cnt, pen = pen_frames(np.load(p)['W'])
        rows['%s/%s' % (s, c)] = dict(frames=len(cnt), pen_frames=int(pen.sum()), max=int(cnt.max()), pen_list=[int(x) for x in np.where(pen)[0]])
    sm = dict(clips=len(rows), clips_pen=sum(1 for r in rows.values() if r['pen_frames']), pen_frames=sum(r['pen_frames'] for r in rows.values()),
              frames=sum(r['frames'] for r in rows.values()), thr=proxy().thr)
    return dict(summary=sm, rows=rows)


if __name__ == '__main__':
    a = sys.argv[1:]
    if a[0] == 'reference':
        res = reference()
        print('REFERENCE', json.dumps(res['summary']))
    elif a[0] == 'calibrate':
        res = calibrate()
    else:
        res = batch(os.path.abspath(a[1]), a[a.index('--kind') + 1] if '--kind' in a else 'v3')
        print(json.dumps(res['summary']))
    if '--out' in a:
        MC.jdump(res, os.path.abspath(a[a.index('--out') + 1]))
