"""Weapon films 2026-11-09 - left side "Game Animations": the game's own Female V player clips, no UTPP correction.

Sources (WolvenKit glb exports, read-only, 152-joint player_woman rig, game axes after NR.locals_to_game):
  Nue  : Extracted\\game\\glb\\weapon-grip-vanilla\\pwa_w_handgun__tsunami_nue.anims.glb
  Ajax : Extracted\\game\\glb\\weapon-recoil-20261021\\base\\animations\\weapon\\firearms\\rifle_assault\\militech_ajax\\
         pwa_w_rifle_assault__militech_ajax.anims.glb
Poses: idlepose_shoulder (hip / ready), idlepose_ironsight (ADS).  Recoil: the clips' additive tracks applied to EVERY bone as a
bind-relative local delta (WolvenKit wrote the additive keys on top of the bind locals, the same reading as
tools/analysis/export_nue_native_recoil_curves.py uses for the profiles): d_r = q_bind^-1 q_key, d_t = t_key - t_bind;
pose = (t_base + d_t, q_base d_r).  A new shot restarts the clip (the graph's SkAnim restarts on the shot event).
Automatic fire (Ajax): the graph's auto-fire state machine (Start -> toAutoFireLoop -> Loop, player_ranged.animgraph.json) =
add_recoil_start_ironsight -> add_recoil_loop_ironsight (looped) -> add_recoil_end_ironsight.
"""
import os

import numpy as np

import weapons_common as WC

NR, MC = WC.NR, WC.MC
GLB = os.path.join(WC.ROOT, 'Extracted', 'game', 'glb')
SETS = {'nue': os.path.join(GLB, 'weapon-grip-vanilla', 'pwa_w_handgun__tsunami_nue.anims.glb'),
        'ajax': os.path.join(GLB, 'weapon-recoil-20261021', 'base', 'animations', 'weapon', 'firearms', 'rifle_assault', 'militech_ajax',
                             'pwa_w_rifle_assault__militech_ajax.anims.glb')}
_C = {}


def qmul(a, b):
    x1, y1, z1, w1 = np.moveaxis(a, -1, 0)
    x2, y2, z2, w2 = np.moveaxis(b, -1, 0)
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], -1)


def qinv(a):
    return a * np.array([-1, -1, -1, 1.0])


class VSet:
    def __init__(self, key):
        self.key = key
        self.path = SETS[key]
        self.sk = NR.skeleton(self.path)
        P = WC.player()
        assert list(self.sk.names) == P.names, 'vanilla set joint order differs from the player rig'
        bt, br = NR.locals_to_game(self.sk.rest_t[None].astype(float), self.sk.rest_r[None].astype(float))
        self.bind_t, self.bind_r = bt[0], br[0]

    def clip(self, name):
        k = (self.key, name)
        if k not in _C:
            lt, lr, ex, dur = NR.sample_clip(self.sk, name)
            lt, lr = NR.locals_to_game(lt, lr)
            _C[k] = (lt, NR.S.qcontinuous(lr), dur)
        return _C[k]

    def pose(self, name):
        lt, lr, _ = self.clip(name)
        return lt[0], lr[0]

    def delta(self, name):
        """bind-relative local delta of an additive clip: (d_t [T,152,3], d_r [T,152,4])."""
        lt, lr, dur = self.clip(name)
        d_r = qmul(qinv(self.bind_r)[None], lr)
        d_r *= np.sign(d_r[..., 3:4] + 1e-12)
        return lt - self.bind_t[None], d_r, dur


def sstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3 - 2 * u)


def slerp(a, b, w):
    """per-joint quaternion slerp a [N,4] -> b [N,4] by scalar w."""
    d = np.sum(a * b, -1, keepdims=True)
    b = np.where(d < 0, -b, b)
    d = np.abs(d)
    om = np.arccos(np.clip(d, -1, 1))
    so = np.sin(om)
    small = so < 1e-6
    k0 = np.where(small, 1 - w, np.sin((1 - w) * om) / np.where(small, 1, so))
    k1 = np.where(small, w, np.sin(w * om) / np.where(small, 1, so))
    q = k0 * a + k1 * b
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def timeline(vs, T, aim_on, aim_off, ads_s, shots, burst=None):
    """locals [T,152] at 30 fps.  aim_on / aim_off: s of the ADS press / release; ads_s: blend time shoulder <-> ironsight;
    shots: single-shot times (s); burst: (t0, t1) automatic fire (start -> loop -> end), None = none."""
    st, sr = vs.pose('idlepose_shoulder')
    it, ir = vs.pose('idlepose_ironsight')
    lt = np.empty((T, len(st), 3))
    lr = np.empty((T, len(st), 4))
    single = vs.delta('add_recoil_single_shot_ironsight')
    for f in range(T):
        t = f / WC.FPS
        w = sstep((t - aim_on) / ads_s) * (1 - sstep((t - aim_off) / ads_s))
        lt[f] = (1 - w) * st + w * it
        lr[f] = slerp(sr, ir, w)
        d = None
        past = [s for s in shots if s <= t + 1e-9]
        if past:
            e = t - past[-1]
            dt_, dr_, dur = single
            if e < dur:
                k = min(int(round(e * WC.FPS)), len(dt_) - 1)
                d = (dt_[k], dr_[k])
        if burst and burst[0] <= t + 1e-9:
            s_t, s_r, s_d = vs.delta('add_recoil_start_ironsight')
            l_t, l_r, l_d = vs.delta('add_recoil_loop_ironsight')
            e_t, e_r, e_d = vs.delta('add_recoil_end_ironsight')
            e = t - burst[0]
            if t < burst[1]:
                if e < s_d:
                    k = min(int(round(e * WC.FPS)), len(s_t) - 1)
                    d = (s_t[k], s_r[k])
                else:
                    k = int(round((e - s_d) * WC.FPS)) % (len(l_t) - 1)
                    d = (l_t[k], l_r[k])
            elif t - burst[1] < e_d:
                k = min(int(round((t - burst[1]) * WC.FPS)), len(e_t) - 1)
                d = (e_t[k], e_r[k])
        if d is not None:
            lt[f] = lt[f] + d[0]
            lr[f] = qmul(lr[f], d[1])
    return lt, NR.S.qcontinuous(lr)


def weapon_delta(W, base_W, ix):
    """WeaponRight motion in its own base frame: base^-1 * cur -> (translation m, rotation rad) per frame."""
    B = base_W[:, ix['WeaponRight']]
    C = W[:, ix['WeaponRight']]
    D = np.linalg.inv(B) @ C
    tr = np.linalg.norm(D[:, :3, 3], axis=1)
    ang = np.arccos(np.clip((np.trace(D[:, :3, :3], axis1=1, axis2=2) - 1) / 2, -1, 1))
    return tr, ang
