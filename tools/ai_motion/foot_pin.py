"""Foot pinning for a generated span (soma npz from clip_inbetween.py / constraint_inbetween.py): removes foot sliding while
the foot is on the ground with a two-bone leg IK on the SOMA joints, leaving the span's first and last frames (the
context and target poses) untouched.

python -I foot_pin.py <span.npz> <tpose.npz> <out.npz> [--ground <full_soma.npz>] [--h 0.04] [--v 1.2] [--report] [--labels]
  contact: the foot's lower joint (ankle or toe) within --h m of the ground level and moving slower than --v m/s
           horizontally (ground level per joint = its minimum height in --ground, else in the span)
  pinning: inside a contact the horizontal motion of the current contact joint is cancelled frame by frame (a roll from
           heel to toe stays natural); the accumulated offset moves the ankle target, the knee keeps its bending plane
           and the foot keeps its world orientation. Contacts touching the span ends are integrated from that end
           (offset 0 there); between contacts the offset is interpolated linearly, so frames 0 and T-1 stay exact.
"""
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

FPS = 30.0
SIDES = ('Left', 'Right')


class Skel:
    def __init__(self, tpose):
        t = np.load(tpose)
        self.names = [str(x) for x in t['names']]
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.par = np.asarray(t['parents'])
        self.pos = np.asarray(t['pos'], dtype=np.float64)
        self.kids = {j: [k for k in range(len(self.names)) if self.par[k] == j] for j in range(len(self.names))}

    def fk(self, G, hips):
        P = np.zeros((len(self.names), 3))
        for j in range(len(self.names)):
            p = self.par[j]
            P[j] = hips if p < 0 else P[p] + G[p] @ (self.pos[j] - self.pos[p])
        return P

    def subtree(self, j):
        out, st = [], [j]
        while st:
            k = st.pop()
            out.append(k)
            st += self.kids[k]
        return out


def rot_between(a, b):
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    return Rot.from_rotvec(v / s * np.arctan2(s, a.dot(b))).as_matrix()


def leg_ik(sk, G, hips, side, goal):
    """Rotate thigh and shin so the ankle reaches goal; knee stays in its current bending plane; the foot subtree keeps
    its world rotations."""
    th, sh, ft = (sk.idx[side + n] for n in ('Leg', 'Shin', 'Foot'))
    foot_keep = {j: G[j].copy() for j in sk.subtree(ft)}
    P = sk.fk(G, hips)
    hip, knee0, ank0 = P[th], P[sh], P[ft]
    la, lb = np.linalg.norm(knee0 - hip), np.linalg.norm(ank0 - knee0)
    d = goal - hip
    dist = np.clip(np.linalg.norm(d), abs(la - lb) + 1e-4, la + lb - 1e-4)
    n = d / np.linalg.norm(d)
    pole = knee0 - hip
    pp = pole - n * pole.dot(n)
    if np.linalg.norm(pp) < 1e-6:
        return
    pp /= np.linalg.norm(pp)
    x = (la * la - lb * lb + dist * dist) / (2 * dist)
    knee = hip + n * x + pp * np.sqrt(max(0.0, la * la - x * x))
    r1 = rot_between(knee0 - hip, knee - hip)
    for j in sk.subtree(th):
        G[j] = r1 @ G[j]
    P = sk.fk(G, hips)
    r2 = rot_between(P[ft] - P[sh], hip + n * dist - P[sh])
    for j in sk.subtree(sh):
        G[j] = r2 @ G[j]
    for j, g in foot_keep.items():
        G[j] = g


def contacts(P, sk, side, ground, h_thr, v_thr):
    a, t = sk.idx[side + 'Foot'], sk.idx[side + 'ToeBase']
    ha, ht = P[:, a, 1] - ground[a], P[:, t, 1] - ground[t]
    piv = np.where(ha <= ht, a, t)  # the lower joint carries the weight
    v = np.zeros(len(P))
    for f in range(1, len(P)):
        j = piv[f]
        v[f] = np.linalg.norm((P[f, j] - P[f - 1, j])[[0, 2]]) * FPS
    v[0] = v[1] if len(P) > 1 else 0
    return (np.minimum(ha, ht) < h_thr) & (v < v_thr), piv


def segments(mask):
    out, f = [], 0
    while f < len(mask):
        if mask[f]:
            e = f
            while e + 1 < len(mask) and mask[e + 1]:
                e += 1
            out.append((f, e))
            f = e + 1
        else:
            f += 1
    return out


def pin_offsets(P, piv, mask):
    """Horizontal ankle offset per frame (T,3), zero at both span ends."""
    T = len(P)
    off = np.full((T, 3), np.nan)
    for s, e in segments(mask):
        if s == 0 or e < T - 1:  # forward from the contact start (or from the span start)
            off[s] = 0.0
            for f in range(s + 1, e + 1):
                j = piv[f]
                dv = P[f, j] - P[f - 1, j]
                off[f] = off[f - 1] - np.array([dv[0], 0.0, dv[2]])
            if s == 0 and e == T - 1:  # contact over the whole span: remove only the END residual, linearly
                w = np.linspace(0, 1, T)[:, None]  # (scaling the whole offset by 1 - w left mid-span drift)
                off = off - w * off[-1]
        else:  # contact reaching the last frame: integrate backwards from it
            off[e] = 0.0
            for f in range(e - 1, s - 1, -1):
                j = piv[f + 1]
                dv = P[f + 1, j] - P[f, j]
                off[f] = off[f + 1] + np.array([dv[0], 0.0, dv[2]])
    if np.isnan(off[0, 0]):
        off[0] = 0.0
    if np.isnan(off[-1, 0]):
        off[-1] = 0.0
    known = ~np.isnan(off[:, 0])
    fr = np.arange(T)
    for c in range(3):
        off[:, c] = np.interp(fr, fr[known], off[known, c])
    return off


def main(a):
    span, tpose, out = a[0], a[1], a[2]
    opt = lambda k, d: type(d)(a[a.index(k) + 1]) if k in a else d
    h_thr, v_thr = opt('--h', 0.04), opt('--v', 1.2)
    sk = Skel(tpose)
    s = np.load(span)
    G, hips = s['G'].astype(np.float64).copy(), s['hips'].astype(np.float64)
    P = np.array([sk.fk(G[t], hips[t]) for t in range(len(G))])
    if '--ground' in a:
        f = np.load(a[a.index('--ground') + 1])
        Pg = np.array([sk.fk(f['G'][t], f['hips'][t]) for t in range(len(f['G']))])
        ground = Pg[:, :, 1].min(0)
    else:
        ground = P[:, :, 1].min(0)
    rep = []
    for side in SIDES:
        mask, piv = contacts(P, sk, side, ground, h_thr, v_thr)
        if '--labels' in a:  # the generator's own foot-contact probabilities (soma span npz: foot_contacts [T,4] = L heel / L toe / R heel / R toe)
            fc = s['foot_contacts']
            cols = (0, 1) if side == 'Left' else (2, 3)
            mask = fc[:, cols].max(1) > 0.5
            for f in range(1, len(mask) - 1):  # close one-frame gaps
                if not mask[f] and mask[f - 1] and mask[f + 1]:
                    mask[f] = True
        off = pin_offsets(P, piv, mask)
        ft = sk.idx[side + 'Foot']
        for t in range(1, len(G) - 1):
            if np.linalg.norm(off[t]) > 1e-5:
                leg_ik(sk, G[t], hips[t], side, P[t, ft] + off[t])
        rep.append('%s contacts %s max offset %.1f cm' % (side, segments(mask), np.linalg.norm(off, axis=1).max() * 100))
    P2 = np.array([sk.fk(G[t], hips[t]) for t in range(len(G))])
    ends = max(np.abs(P2[0] - P[0]).max(), np.abs(P2[-1] - P[-1]).max()) * 100
    keep = {k: s[k] for k in s.files if k not in ('G', 'hips')}
    np.savez(out, G=G, hips=hips, **keep)
    print('FOOT_PIN %s | %s | end frames changed %.3f cm' % (out, ' | '.join(rep), ends))


if __name__ == '__main__':
    main(sys.argv[1:])
