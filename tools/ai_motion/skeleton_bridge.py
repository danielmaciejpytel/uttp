"""source pack (UEFN mannequin) <-> SOMA77 pose bridge for Kimodo / MotionBricks. numpy + scipy only.

SOMA side ("soma npz"): G [T,77,3,3] global rotation deltas from the SOMA T-pose (identity = the T-pose of the
tool's skeleton; Y up, +Z forward, left +X, metres), hips [T,3] Hips position, names [77], plus pos [T,77,3] (FK).
source pack side ("dump npz", from mannequin_dump.py): W [T,B,4,4] bone world matrices in cm, Blender world (Z up, -Y forward),
rest [B,4,4] bind matrices.

Rotation transfer per mapped bone: G_j(t) = C * (R_u(t) R_u(bind)^T) * C^T * A_j, where A_j turns the SOMA T-pose
segment frame into the source pack bind segment frame (primary bone direction + anatomical secondary axis, so the roll is
defined). Root: the pelvis maps to Hips with the leg-length ratio k (horizontal and vertical above the ankle height).

  python skeleton_bridge.py fwd <dump.npz> <tpose.npz> <out_soma.npz>
  python skeleton_bridge.py inv <soma.npz> <template_dump.npz> <tpose.npz> <out_world.npz> [--frames a:b] [--leg-ik]
  (inv: frame i of soma.npz uses template frame a+i for the bones SOMA does not drive: fingers, twist, ik, props)
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp

CM = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # Blender (Z up, -Y fwd) -> SOMA (Y up, +Z fwd)

SIDES = (('Left', '_l'), ('Right', '_r'))
MAP = [  # soma joint, uefn bone, soma primary pair, uefn primary pair, secondary
    ('Hips', 'pelvis', ('Hips', 'Neck1'), ('pelvis', 'neck_01'), 'LR'),
    ('Spine1', 'spine_01', ('Spine1', 'Neck1'), ('spine_01', 'neck_01'), 'LR'),
    ('Spine2', 'spine_03', ('Spine2', 'Neck1'), ('spine_03', 'neck_01'), 'LR'),
    ('Chest', 'spine_05', ('Chest', 'Neck1'), ('spine_05', 'neck_01'), 'LR'),
    ('Neck1', 'neck_01', ('Neck1', 'Head'), ('neck_01', 'head'), 'LR'),
    ('Neck2', 'neck_02', ('Neck2', 'Head'), ('neck_02', 'head'), 'LR'),
    ('Head', 'head', ('Neck1', 'Head'), ('neck_01', 'head'), 'LR'),
]
for S, s in SIDES:
    MAP += [
        (S + 'Shoulder', 'clavicle' + s, (S + 'Shoulder', S + 'Arm'), ('clavicle' + s, 'upperarm' + s), 'FWD'),
        (S + 'Arm', 'upperarm' + s, (S + 'Arm', S + 'ForeArm'), ('upperarm' + s, 'lowerarm' + s), 'FWD'),
        (S + 'ForeArm', 'lowerarm' + s, (S + 'ForeArm', S + 'Hand'), ('lowerarm' + s, 'hand' + s), 'FWD'),
        (S + 'Hand', 'hand' + s, (S + 'Hand', S + 'HandMiddle2'), ('hand' + s, 'middle_01' + s),
         ('PALM', (S + 'HandIndex2', S + 'HandPinky2'), ('index_01' + s, 'pinky_01' + s))),
        (S + 'Leg', 'thigh' + s, (S + 'Leg', S + 'Shin'), ('thigh' + s, 'calf' + s), 'FWD'),
        (S + 'Shin', 'calf' + s, (S + 'Shin', S + 'Foot'), ('calf' + s, 'foot' + s), 'FWD'),
        (S + 'Foot', 'foot' + s, (S + 'Foot', S + 'ToeBase'), ('foot' + s, 'ball' + s), 'LR'),
        (S + 'ToeBase', 'ball' + s, (S + 'Foot', S + 'ToeBase'), ('foot' + s, 'ball' + s), 'LR'),
    ]
# uefn bones SOMA does not drive but that should bend with the spine: world delta slerped between two mapped bones
SPINE_FILL = {'spine_02': ('spine_01', 'spine_03', 0.5), 'spine_04': ('spine_03', 'spine_05', 0.5)}
DERIVED = {'ik_foot_l': 'foot_l', 'ik_foot_r': 'foot_r', 'ik_hand_l': 'hand_l', 'ik_hand_r': 'hand_r',
           'ik_hand_gun': 'hand_r', 'weapon_r': 'hand_r', 'weapon_l': 'hand_l'}
# skeleton profile: the source rig's mapping, axes (cm: rig axes -> SOMA axes, both right-handed, rig is Z up), units
# per metre and the root / leg bone names. Other rigs (game_skeleton_bridge.py) pass their own.
PROFILE_UEFN = dict(map=MAP, spine_fill=SPINE_FILL, derived=DERIVED, cm=CM, unit=100.0, pelvis='pelvis',
                    legs={S: tuple(x + s for x in ('thigh', 'calf', 'foot')) for S, s in SIDES})


def ortho(m):
    u, _, vt = np.linalg.svd(m)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def frame(p, s):
    e1 = p / np.linalg.norm(p)
    e3 = np.cross(e1, s)
    e3 /= np.linalg.norm(e3)
    return np.stack([e1, np.cross(e3, e1), e3], axis=1)


class Bridge:
    def __init__(self, tpose_path, rest, unames, prof=None):
        prof = prof or PROFILE_UEFN
        self.map, self.spine_fill, self.derived = prof['map'], prof['spine_fill'], prof['derived']
        self.cm, self.unit, self.pelvis, self.legs = prof['cm'], prof['unit'], prof['pelvis'], prof['legs']
        CM = self.cm
        t = np.load(tpose_path)
        self.snames = [str(x) for x in t['names']]
        self.sidx = {n: i for i, n in enumerate(self.snames)}
        self.spar = np.asarray(t['parents'])
        self.spos = np.asarray(t['pos'], dtype=np.float64)  # T-pose, metres, Y up
        self.unames = [str(x) for x in unames]
        self.uidx = {n: i for i, n in enumerate(self.unames)}
        self.rest = rest
        ub = (CM @ rest[:, :3, 3].T).T / self.unit  # rig bind positions in SOMA axes, metres
        sp = lambda n: self.spos[self.sidx[n]]
        up = lambda n: ub[self.uidx[n]]
        lr_s = sp('LeftLeg') - sp('RightLeg')
        lr_u = up(self.legs['Left'][0]) - up(self.legs['Right'][0])
        fwd = np.array([0.0, 0, 1])
        self.A = {}
        for j, u, ps, pu, sec in self.map:
            if sec == 'LR':
                ss, su = lr_s, lr_u
            elif sec == 'FWD':
                ss = su = fwd
            else:
                ss = sp(sec[1][0]) - sp(sec[1][1])
                su = up(sec[2][0]) - up(sec[2][1])
            Ft = frame(sp(ps[1]) - sp(ps[0]), ss)
            Fb = frame(up(pu[1]) - up(pu[0]), su)
            self.A[j] = Fb @ Ft.T
        # root mapping: leg-length ratio, ankle heights
        self.ank_s = 0.5 * (sp('LeftFoot')[1] + sp('RightFoot')[1])
        self.ank_u = 0.5 * (rest[self.uidx[self.legs['Left'][2]], 2, 3]
                            + rest[self.uidx[self.legs['Right'][2]], 2, 3]) / self.unit
        self.k = (sp('Hips')[1] - self.ank_s) / (rest[self.uidx[self.pelvis], 2, 3] / self.unit - self.ank_u)

    # ---- positions
    def pelvis_to_hips(self, p_cm):
        v = self.cm @ p_cm / self.unit
        return np.array([v[0] * self.k, self.ank_s + self.k * (v[1] - self.ank_u), v[2] * self.k])

    def hips_to_pelvis(self, h):
        v = np.array([h[0] / self.k, self.ank_u + (h[1] - self.ank_s) / self.k, h[2] / self.k])
        return self.cm.T @ v * self.unit

    def soma_fk(self, G, hips):
        P = np.zeros((len(self.snames), 3))
        for j in range(len(self.snames)):
            pj = self.spar[j]
            P[j] = hips if pj < 0 else P[pj] + G[pj] @ (self.spos[j] - self.spos[pj])
        return P

    # ---- source pack -> SOMA
    def fwd(self, W):
        T = W.shape[0]
        G = np.tile(np.eye(3), (T, len(self.snames), 1, 1))
        hips = np.zeros((T, 3))
        CM = self.cm
        Rb = {u: ortho(self.rest[self.uidx[u], :3, :3]) for _, u, *_ in self.map}
        for t in range(T):
            for j, u, *_ in self.map:
                D = CM @ (ortho(W[t, self.uidx[u], :3, :3]) @ Rb[u].T) @ CM.T
                G[t, self.sidx[j]] = D @ self.A[j]
            for j in range(len(self.snames)):  # unmapped joints follow their parent rigidly (T-pose relative)
                if self.snames[j] not in self.A and self.spar[j] >= 0:
                    G[t, j] = G[t, self.spar[j]]
            hips[t] = self.pelvis_to_hips(W[t, self.uidx[self.pelvis], :3, 3])
        pos = np.array([self.soma_fk(G[t], hips[t]) for t in range(T)])
        return G, hips, pos

    # ---- SOMA -> source pack world matrices
    def inv(self, G, hips, tmpl_W, tmpl_frames, leg_ik=False):
        T = G.shape[0]
        nb = len(self.unames)
        par = self.uparents
        rest = self.rest
        Lrest = [np.linalg.inv(rest[par[b]]) @ rest[b] if par[b] >= 0 else rest[b] for b in range(nb)]
        out = np.zeros((T, nb, 4, 4))
        CM = self.cm
        drive = {u: j for j, u, *_ in self.map}
        for t in range(T):
            wo = tmpl_W[tmpl_frames[t]]
            Rw = {}
            for u, j in drive.items():
                D = CM.T @ (G[t, self.sidx[j]] @ self.A[j].T) @ CM
                Rw[u] = ortho(D @ ortho(rest[self.uidx[u], :3, :3]))
            for u, (a, b, w) in self.spine_fill.items():
                Da = Rot.from_matrix(Rw[a] @ ortho(rest[self.uidx[a], :3, :3]).T)
                Db = Rot.from_matrix(Rw[b] @ ortho(rest[self.uidx[b], :3, :3]).T)
                Dm = Slerp([0, 1], Rot.concatenate([Da, Db]))(w).as_matrix()
                Rw[u] = ortho(Dm @ ortho(rest[self.uidx[u], :3, :3]))
            Wt = np.zeros((nb, 4, 4))
            pel_new = self.hips_to_pelvis(hips[t])
            pel_old = wo[self.uidx[self.pelvis], :3, 3]
            for b in range(nb):  # bones are stored parents first
                n = self.unames[b]
                if n == self.pelvis:
                    m = np.eye(4); m[:3, :3] = Rw[n]; m[:3, 3] = pel_new
                elif n in Rw:
                    m = np.eye(4); m[:3, :3] = Rw[n]
                    m[:3, 3] = (Wt[par[b]] @ Lrest[b])[:3, 3]
                elif n in self.derived:
                    a = self.uidx[self.derived[n]]
                    m = Wt[a] @ (np.linalg.inv(wo[a]) @ wo[b])
                elif par[b] >= 0:
                    m = Wt[par[b]] @ (np.linalg.inv(wo[par[b]]) @ wo[b])  # template local (fingers, twist, props)
                else:  # other roots (attach): follow the pelvis on the ground plane
                    m = wo[b].copy()
                    m[0, 3] += pel_new[0] - pel_old[0]
                    m[1, 3] += pel_new[1] - pel_old[1]
                Wt[b] = m
            if leg_ik:
                self._leg_ik(Wt, G[t], hips[t])
            out[t] = Wt
        return out

    def _leg_ik(self, Wt, G, hips):
        """Two-bone IK so the source pack ankle lands on the SOMA ankle, mapped with the same root transform."""
        P = self.soma_fk(G, hips)
        for S, _ in SIDES:
            th, ca, fo = (self.uidx[x] for x in self.legs[S])
            goal = self.hips_to_pelvis(P[self.sidx[S + 'Foot']] - P[self.sidx['Hips']] + hips) - \
                self.hips_to_pelvis(hips) + Wt[self.uidx[self.pelvis], :3, 3]
            # keep relative leg geometry: goal = pelvis + (soma ankle - soma hips) / k in source pack axes
            hip = Wt[th, :3, 3]; knee0 = Wt[ca, :3, 3]; ank0 = Wt[fo, :3, 3]
            la, lb = np.linalg.norm(knee0 - hip), np.linalg.norm(ank0 - knee0)
            d = goal - hip; dist = min(np.linalg.norm(d), la + lb - 1e-5)
            n = d / np.linalg.norm(d)
            pole = knee0 - hip; pp = pole - n * pole.dot(n); pp /= np.linalg.norm(pp)
            x = (la * la - lb * lb + dist * dist) / (2 * dist); h = np.sqrt(max(0.0, la * la - x * x))
            knee = hip + n * x + pp * h
            rf = Wt[fo, :3, :3].copy()
            r1 = rot_between(knee0 - hip, knee - hip)
            self._rotate_subtree(Wt, th, r1, hip)
            r2 = rot_between(Wt[fo, :3, 3] - Wt[ca, :3, 3], goal - Wt[ca, :3, 3])
            self._rotate_subtree(Wt, ca, r2, Wt[ca, :3, 3].copy())
            r3 = rf @ Wt[fo, :3, :3].T  # restore the foot's world orientation
            self._rotate_subtree(Wt, fo, r3, Wt[fo, :3, 3].copy())

    def _rotate_subtree(self, Wt, b, r, pivot):
        for c in self.subtree(b):
            Wt[c, :3, :3] = r @ Wt[c, :3, :3]
            Wt[c, :3, 3] = pivot + r @ (Wt[c, :3, 3] - pivot)

    def subtree(self, b):
        out = [b]
        for c in range(len(self.unames)):
            p = self.uparents[c]
            while p >= 0:
                if p == b:
                    out.append(c); break
                p = self.uparents[p]
        return out


def rot_between(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = a.dot(b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    return Rot.from_rotvec(v / np.linalg.norm(v) * np.arctan2(np.linalg.norm(v), c)).as_matrix()


def load_bridge(tpose, dump, prof=None):
    d = np.load(dump)
    br = Bridge(tpose, d['rest'], d['names'], prof)
    br.uparents = np.asarray(d['parents'])
    return br, d


def main(a):
    if a[0] == 'fwd':
        br, d = load_bridge(a[2], a[1])
        G, hips, pos = br.fwd(d['W'])
        np.savez(a[3], G=G, hips=hips, pos=pos, names=np.array(br.snames), k=br.k, fps=30)
        print('FWD', a[3], 'frames', len(G), 'k=%.4f' % br.k)
    elif a[0] == 'inv':
        br, d = load_bridge(a[3], a[2])
        s = np.load(a[1])
        T = len(s['G'])
        f0 = 0
        if '--frames' in a:
            f0 = int(a[a.index('--frames') + 1].split(':')[0])
        frames = [min(f0 + i, len(d['W']) - 1) for i in range(T)]
        W = br.inv(s['G'], s['hips'], d['W'], frames, leg_ik='--leg-ik' in a)
        np.savez(a[4], W=W, names=d['names'], parents=d['parents'], fps=30)
        print('INV', a[4], 'frames', T)


if __name__ == '__main__':
    main(sys.argv[1:])
