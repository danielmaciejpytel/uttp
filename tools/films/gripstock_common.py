"""Rifle grip + stock 2026-11-10 - shared offline code (run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I).

Drives gripstock_eval_{before,after}.exe (gripstock_build_eval.cmd: the PRODUCTION MotionPose.hpp of the tree before / after the fix) with
scripted rifle timelines (weapons_common.Timeline), Female V on the installed data (Build\\work\\weapon-films-20261109\\data, E7E5
overlay) or Male V (Data\\v2), and measures per frame on FK + the skinned body:
  grip_err_cm   LeftHand -> the game's grip point of the weapon (grip_game.json, WeaponRight frame);
  reach         |LeftHand - LeftArm| / (upper + forearm length) (straight arm = 1);
  elbow_flex    left elbow flexion (deg);
  stock_gap_cm  rear of the stock (real mesh, vertices within 4 cm of the rearmost point) -> nearest body skin vertex that is not
                owned by a hand / forearm joint (nearest-joint ownership, weapons_fix_stock.py rule); < 0 never (distance);
  stock_in      body skin vertices (not hand / forearm) inside the weapon mesh (generalized winding number > 0.5);
  hand_in       left-hand skin vertices (owned by LeftHand / LeftInHand* / LeftHand* fingers) inside the weapon mesh.
Writes only into Build\\work\\rifle-grip-stock-20261110.
"""
import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import weapons_common as WC  # noqa: E402
import weapons_fix_mesh as MESH  # noqa: E402

NR, MC = WC.NR, WC.MC
WORK = os.path.join(WC.ROOT, 'Build', 'work', 'rifle-grip-stock-20261110')
TMP = os.path.join(WORK, '_tmp')
BIN = os.path.join(WORK, 'bin')
DATA = {'woman': (os.path.join(WC.DATA, 'utpp_motion_woman.utppmm'), os.path.join(WC.DATA, 'utpp_overlays_woman.utppov'),
                  os.path.join(WC.DATA, 'utpp_style_weights_woman.txt')),
        'man': (os.path.join(WC.ROOT, 'Data', 'motion', 'utpp_motion_man.utppmm'), os.path.join(WC.ROOT, 'Data', 'motion', 'utpp_overlays_man.utppov'),
                os.path.join(WC.ROOT, 'Data', 'motion', 'utpp_style_weights_man.txt'))}
RECOIL = os.path.join(WC.ROOT, 'Data', 'recoil')
PROFILE = {'ajax': 12, 'copperhead': 13, 'sidewinder': 14, 'masamune': 15, 'shingen': 16, 'pulsar': 17, 'saratoga': 18, 'senkoh': 19, 'defender': 23, 'dian': 24, 'warden': 25, 'borg4a': 26, 'hercules': 27}
MESHDIR = {'ajax': (os.path.join(WC.ROOT, 'Extracted', 'game', 'weapon-mesh-ajax-20261109'), ('base1_01.glb', 'mag_std_01.glb')),
           'saratoga': (os.path.join(WC.ROOT, 'Extracted', 'game', 'weapon-mesh-saratoga-20261110'),
                         ('saratoga__base1_01.glb', 'base1_01_stock_01.glb', 'saratoga__mag_std_01.glb'))}
# 2026-11-10 long guns (gripstock_extract_mesh.ps1 -Name longguns): base body + standard magazine (+ the default base stock part if it exists)
_LG = os.path.join(WC.ROOT, 'Extracted', 'game', 'weapon-mesh-longguns-20261110')
for _fam, _key in (('copperhead', 'nokota_copperhead'), ('sidewinder', 'nokota_sidewinder'), ('masamune', 'arasaka_masamune'),
                   ('shingen', 'arasaka_shingen'), ('pulsar', 'darra_pulsar'), ('senkoh', 'senkoh_lx'), ('defender', 'constitutional_defender'),
                   ('dian', 'kangtao_dian')):
    MESHDIR[_fam] = (_LG, ('%s__base1_01.glb' % _key, '%s__base1_01_stock_01.glb' % _key, '%s__mag_std_01.glb' % _key))
MESHDIR['copperhead'] = (_LG, MESHDIR['copperhead'][1] + ('nokota_copperhead__rcv1_01_stock_01.glb',))  # stock is a receiver part
RUN = 'M_Neutral_Run_Loop_F'
POCKET_OWNERS = ('RightShoulder', 'Spine3', 'Spine2', 'Neck')   # shoulder pocket / chest skin (not the upper arm)
SPRINT = 'M_Neutral_Sprint_Loop_F'
_PL = {}


def player(body):
    if body not in _PL:
        _PL[body] = WC.player() if body == 'woman' else NR.Player('man')
    return _PL[body]


def game_grip(family, clip='idlepose_shoulder'):
    d = json.load(open(os.path.join(WORK, 'grip_game.json')))
    return d['woman:%s:%s' % (family, clip)]


def weapon_mesh(family):
    """real game mesh of the weapon (WolvenKit glb; base body + standard magazine, no decals / scopes), weapon frame, game axes."""
    import glob
    root, parts = MESHDIR[family]
    files = sorted(glob.glob(os.path.join(root, '**', '*.glb'), recursive=True))
    if parts:
        files = [f for f in files if any(f.endswith(p) for p in parts)]
    else:
        files = [f for f in files if 'decal' not in f and ('base' in os.path.basename(f) or 'mag_std' in os.path.basename(f))]
    Vs, Fs, off = [], [], 0
    for f in files:
        js, binb = MESH.load(f)
        for m in js['meshes']:
            for p in m['primitives']:
                v = MESH.accessor(js, binb, p['attributes']['POSITION'])
                Vs.append(np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1))
                Fs.append(MESH.accessor(js, binb, p['indices']).reshape(-1, 3).astype(np.int64) + off)
                off += len(v)
    return np.concatenate(Vs), np.concatenate(Fs), [os.path.relpath(f, WC.ROOT) for f in files]


def timeline(scn, recoil_file, fem=0.025):
    """scripted rifle timelines (60 Hz steps, 30 fps output). -> (Timeline, shot times s, phase marks [(t0, label)])."""
    tl = WC.Timeline('Rifle', '-', (0, fem), (0.0, 25.0, 1.45))
    marks, shots = [], []

    def seg(sec, aim, moving, clip=None, label=''):
        if clip is not None and clip != tl.clip:
            tl.switch_clip(clip, 0.0, 0.3)
        marks.append((tl.now_step() * WC.DT, label))
        tl.run(sec, aim=aim, moving=moving)

    if scn == 'main':
        seg(1.0, 'relaxed', 0, WC.IDLE, 'relaxed')
        seg(1.0, 'ready', 0, None, 'ready')
        t = tl.now_step() * WC.DT
        seg(2.4, 'aim', 0, None, 'aim + shots')
        shots += [t + 0.6, t + 1.0, t + 1.4]
        seg(0.8, 'ready', 0, None, 'ready')
        seg(1.5, 'ready', 1, WC.WALK, 'walk ready')
        seg(1.2, 'aim', 1, None, 'walk aim')
        seg(1.2, 'relaxed', 1, RUN, 'run')
        seg(1.5, 'relaxed', 2, SPRINT, 'sprint')
        seg(0.6, 'relaxed', 1, RUN, 'run')
        seg(1.2, 'ready', 0, WC.IDLE, 'stop ready')
    elif scn == 'aim':
        seg(0.6, 'ready', 0, WC.IDLE, 'ready')
        t = tl.now_step() * WC.DT
        seg(3.0, 'aim', 0, None, 'aim + shots')
        shots += [t + 0.8, t + 1.3, t + 1.8]
        seg(0.6, 'ready', 0, None, 'ready')
    elif scn == 'film':  # film 1x: + reload stand-in = hand-back (20th step column; the game's reload animation is not offline)
        hb = []
        seg(0.8, 'relaxed', 0, WC.IDLE, 'relaxed')
        seg(0.8, 'ready', 0, None, 'ready')
        t = tl.now_step() * WC.DT
        seg(2.4, 'aim', 0, None, 'aim + shots')
        shots += [t + 0.6, t + 1.0, t + 1.4]
        seg(0.4, 'ready', 0, None, 'ready')
        k0 = tl.now_step()
        seg(1.6, 'ready', 0, None, 'reload')
        hb.append((k0, tl.now_step()))
        seg(0.6, 'ready', 0, None, 'ready')
        seg(1.4, 'ready', 1, WC.WALK, 'walk ready')
        seg(1.2, 'aim', 1, None, 'walk aim')
        seg(1.2, 'relaxed', 1, RUN, 'run')
        seg(1.4, 'relaxed', 2, SPRINT, 'sprint')
        seg(0.5, 'relaxed', 1, RUN, 'run')
        seg(1.0, 'ready', 0, WC.IDLE, 'stop ready')
        for i, r in enumerate(tl.rows):
            r.append(1 if any(a <= i < b for a, b in hb) else 0)
    else:
        raise ValueError(scn)
    for r in tl.rows:
        r[11] = recoil_file
    return tl, shots, marks


def run(mode, name, tl, shots, body='woman', profile=0, extra=()):
    """gripstock_eval_<mode>.exe on the timeline -> dict(lt, lr [T,73], diag, W [T,152,4,4])."""
    os.makedirs(TMP, exist_ok=True)
    for t in shots:
        tl.rows[int(round(t / WC.DT))][10] = 1
    sp = os.path.join(TMP, name + '.steps.txt')
    tl.write(sp)
    pre = os.path.join(TMP, name)
    db, ov, st = DATA[body]
    cmd = [os.path.join(BIN, 'gripstock_eval_%s.exe' % mode), 'run', db, ov, st, RECOIL, sp, pre, '--profile', str(profile)]
    if body == 'man':
        cmd.append('--man')
    cmd += list(extra)
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode or 'RAN' not in p.stdout:
        raise RuntimeError('gripstock_eval failed %d: %s %s' % (p.returncode, p.stdout[-800:], p.stderr[-800:]))
    lines = open(pre + '.bones.txt').read().split('\n')
    n = int(lines[0])
    names = [lines[1 + i].split()[0] for i in range(n)]
    raw = np.fromfile(pre + '.pose.bin', dtype='<f4').reshape(-1, n, 7).astype(float)
    diag = np.genfromtxt(pre + '.diag.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
    P = player(body)
    assert names == P.names[:73]
    T = len(raw)
    lt = np.repeat(P.ref['rest_lt'][None].astype(float), T, 0)
    lr = np.repeat(P.ref['rest_lr'][None].astype(float), T, 0)
    lt[:, :73], lr[:, :73] = raw[:, :, :3], raw[:, :, 3:]
    W = MC.fk_world(P.par, lt, NR.S.qcontinuous(lr))
    return dict(lt=raw[:, :, :3], lr=raw[:, :, 3:], diag=diag, W=W, log=p.stdout)


def skin(W, body, base):
    """skinned body positions [T,nv,3] (skin_clip.py world mode, rest of the player reference)."""
    P = player(body)
    import hashlib
    h = hashlib.sha1(np.ascontiguousarray(np.round(W[:, :73], 6)).tobytes()).hexdigest()
    hf = base + '.whash'
    fresh = os.path.exists(base + '.pc2') and os.path.exists(hf) and open(hf).read().strip() == h
    if not fresh:  # the skin cache of identical joint matrices is reused
        NR.skin_W(W[:, :73], P.ref['rest'], body, base)
        open(hf, 'w').write(h)
    sk = np.load(base + '.npz')
    nv = len(sk['rest'])
    return np.fromfile(base + '.pc2', dtype='<f4', offset=32).reshape(-1, nv, 3).astype(float)


def winding(V, F, pts, chunk=32):
    """generalized winding number of points w.r.t. the triangle soup V[F] (> 0.5 = inside; open game meshes; Jacobson et al. 2013)."""
    A, B, C = (np.ascontiguousarray(V[F[:, k]], dtype=np.float32) for k in range(3))
    out = np.zeros(len(pts))
    P = np.asarray(pts, np.float32)
    for s in range(0, len(P), chunk):
        p = P[s:s + chunk, None, :]
        a, b, c = A[None] - p, B[None] - p, C[None] - p
        la = np.sqrt(np.einsum('pfi,pfi->pf', a, a))
        lb = np.sqrt(np.einsum('pfi,pfi->pf', b, b))
        lc = np.sqrt(np.einsum('pfi,pfi->pf', c, c))
        bxc = np.empty_like(b)
        bxc[..., 0] = b[..., 1] * c[..., 2] - b[..., 2] * c[..., 1]
        bxc[..., 1] = b[..., 2] * c[..., 0] - b[..., 0] * c[..., 2]
        bxc[..., 2] = b[..., 0] * c[..., 1] - b[..., 1] * c[..., 0]
        det = np.einsum('pfi,pfi->pf', a, bxc)
        den = la * lb * lc + np.einsum('pfi,pfi->pf', a, b) * lc + np.einsum('pfi,pfi->pf', b, c) * la + np.einsum('pfi,pfi->pf', c, a) * lb
        out[s:s + chunk] = np.sum(np.arctan2(det, den), 1, dtype=np.float64) / (2 * np.pi)
    return out


class LocalWinding:
    """winding number restricted to the connected pieces of the mesh soup whose bounding box comes within 4 cm of the query points
    (a closed piece adds ~0 to a point outside it; the game meshes are soups of ~1000 small shells). Checked against the full winding."""

    def __init__(self, V, F):
        import trimesh
        m = trimesh.Trimesh(vertices=V, faces=F, process=False)
        lab = trimesh.graph.connected_component_labels(m.face_adjacency, node_count=len(F))
        self.V, self.F, self.lab = V, F, lab
        n = lab.max() + 1
        tri = V[F]
        self.lo = np.full((n, 3), np.inf)
        self.hi = np.full((n, 3), -np.inf)
        np.minimum.at(self.lo, lab, tri.min(1))
        np.maximum.at(self.hi, lab, tri.max(1))

    def __call__(self, P, margin=0.04):
        if not len(P):
            return np.zeros(0)
        lo, hi = P.min(0) - margin, P.max(0) + margin
        keep = np.all((self.hi >= lo) & (self.lo <= hi), 1)
        f = self.F[keep[self.lab]]
        return winding(self.V, f, P) if len(f) else np.zeros(len(P))


class Inside:
    """inside test of points near a game mesh (open triangle soup): nearest surface point (trimesh.proximity.closest_point) and the sign of
    (p - closest) . face normal (< -0.5 mm = inside). Points farther than 3.5 cm from the surface are outside (the guns are <= 6.5 cm thick).
    Same decision as the generalized winding number on near-surface points, O(n log m)."""

    def __init__(self, V, F):
        import trimesh
        self.m = trimesh.Trimesh(vertices=V, faces=F, process=False)
        self.n = self.m.face_normals

    def depth(self, P):
        """-> depth [n] (m, > 0 inside, 0 outside)."""
        import trimesh
        out = np.zeros(len(P))
        if not len(P):
            return out
        c, d, tri = trimesh.proximity.closest_point(self.m, P)
        s = np.einsum('ij,ij->i', P - c, self.n[tri])
        ins = (s < -0.0005) & (d < 0.035)
        out[ins] = d[ins]
        return out


def hand_swing(W, ix):
    """signed twist (deg) of the left hand about the forearm axis (+X local): the deformer / DLL limit measure (weapons_fix_calib.py)."""
    from scipy.spatial.transform import Rotation as Rot
    Rl = np.swapaxes(W[:, ix['LeftForeArm'], :3, :3], -1, -2) @ W[:, ix['LeftHand'], :3, :3]
    q = Rot.from_matrix(Rl).as_quat()
    q = np.where(q[..., 3:4] < 0, -q, q)
    return np.degrees(2 * np.arctan2(q[..., 0], q[..., 3]))


def elbow_flex(W, ix):
    a, b, c = W[:, ix['LeftArm'], :3, 3], W[:, ix['LeftForeArm'], :3, 3], W[:, ix['LeftHand'], :3, 3]
    u, v = b - a, c - b
    cs = np.sum(u * v, 1) / (np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1))
    return np.degrees(np.arccos(np.clip(cs, -1, 1)))


def measure(W, B, body, family, every=1, mesh=None, inside_every=4):
    """per-frame metrics (see module doc). B = skinned body [T,nv,3] or None (no skin metrics)."""
    P = player(body)
    ix = P.ix
    g = game_grip(family)
    gt = np.array(g['left_t'])
    M = W[:, ix['WeaponRight']]
    tgt = M[:, :3, 3] + np.einsum('tij,j->ti', M[:, :3, :3], gt)
    lh = W[:, ix['LeftHand'], :3, 3]
    a, b = W[:, ix['LeftArm'], :3, 3], W[:, ix['LeftForeArm'], :3, 3]
    full = np.linalg.norm(b - a, axis=1) + np.linalg.norm(lh - b, axis=1)
    out = dict(grip_err_cm=np.linalg.norm(lh - tgt, axis=1) * 100, reach=np.linalg.norm(lh - a, axis=1) / full,
               reach_target=np.linalg.norm(tgt - a, axis=1) / full, elbow_flex=elbow_flex(W, ix), swing=hand_swing(W, ix),
               lh_in_weapon_cm=np.einsum('tji,tj->ti', M[:, :3, :3], lh - M[:, :3, 3]) * 100,
               weapon_in_rshoulder_cm=np.einsum('tji,tj->ti', W[:, ix['RightShoulder'], :3, :3], M[:, :3, 3] - W[:, ix['RightShoulder'], :3, 3]) * 100,
               weapon_in_spine3_cm=np.einsum('tji,tj->ti', W[:, ix['Spine3'], :3, :3], M[:, :3, 3] - W[:, ix['Spine3'], :3, 3]) * 100)
    if B is None or mesh is None:
        return out
    V, F = mesh[0], mesh[1]
    mtree = cKDTree(V)
    lw = LocalWinding(V, F)
    rear = V[:, 1].min()
    stock = V[V[:, 1] < rear + 0.04]
    joints = [j for j in range(73) if P.names[j] not in ('Root', 'Trajectory', 'reference_joint', 'WeaponLeft', 'WeaponRight', 'LeftEye',
                                                         'RightEye')]
    jn = [P.names[j] for j in joints]
    j_hand = np.array([('Hand' in n) or ('ForeArm' in n) for n in jn])
    j_lhand = np.array([n.startswith('LeftHand') or n.startswith('LeftInHand') for n in jn])
    j_pocket = np.array([n in POCKET_OWNERS for n in jn])
    T = min(len(W), len(B))
    gap = np.full(T, np.nan)
    gap_bone = [''] * T
    gap_pocket = np.full(T, np.nan)
    stock_in = np.full(T, -1)
    hand_in = np.full(T, -1)
    hand_depth = np.full(T, np.nan)
    for f in range(0, T, every):
        jp = W[f, joints, :3, 3]
        _, own = cKDTree(jp).query(B[f])
        handish = j_hand[own]
        lhand = j_lhand[own]
        Mf = M[f]
        S = stock @ Mf[:3, :3].T + Mf[:3, 3]
        body_pts = B[f][~handish]
        d, k = cKDTree(body_pts).query(S)
        gap[f] = d.min() * 100
        gap_bone[f] = jn[own[~handish][k[int(np.argmin(d))]]]
        pocket = j_pocket[own]
        if pocket.any():
            gap_pocket[f] = cKDTree(B[f][pocket]).query(S)[0].min() * 100
        if f % inside_every == 0:
            Bw = (B[f] - Mf[:3, 3]) @ Mf[:3, :3]  # body in the weapon frame
            lo, hi = V.min(0) - 0.01, V.max(0) + 0.01
            box = np.all((Bw > lo) & (Bw < hi), 1)
            if box.any():  # an inside point of a <= 6.5 cm thick gun lies within 3.5 cm of its surface: test only those
                near = np.zeros(len(Bw), bool)
                near[np.nonzero(box)[0]] = mtree.query(Bw[box])[0] < 0.035
                box &= near
            sel = box & ~handish
            stock_in[f] = int(np.sum(lw(Bw[sel]) > 0.5)) if sel.any() else 0
            selh = box & lhand
            if selh.any():
                ins = lw(Bw[selh]) > 0.5
                hand_in[f] = int(ins.sum())
                hand_depth[f] = float(mtree.query(Bw[selh][ins])[0].max() * 100) if ins.any() else 0.0
            else:
                hand_in[f], hand_depth[f] = 0, 0.0
    out.update(stock_gap_cm=gap, stock_gap_pocket_cm=gap_pocket, stock_gap_bone=gap_bone, stock_in=stock_in, hand_in=hand_in, hand_depth_cm=hand_depth)
    return out


def summ(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not len(x):
        return None
    return [round(float(x.min()), 2), round(float(np.median(x)), 2), round(float(x.max()), 2)]


def phase_of(marks, T):
    lab = [''] * T
    for i, (t0, l) in enumerate(marks):
        f0 = int(round(t0 * WC.FPS))
        f1 = int(round(marks[i + 1][0] * WC.FPS)) if i + 1 < len(marks) else T
        for f in range(f0, min(f1, T)):
            lab[f] = l
    return lab
