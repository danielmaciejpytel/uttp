"""Foot-slide measurement and offline foot pinning for utppclip (game rig, 30 fps) clips - MM locomotion families.

World = Trajectory (planar root motion) o model-space pose (Hips is a child of Root, not of Trajectory).
Contact = clip curve contact_l / contact_r >= 0.5. Slide of a contact frame pair = horizontal world displacement (cm) of the
ground contact point (the lower of Heel/ToeBase; when both are within FLAT_H of the ground the smaller of the two displacements,
so a flat-foot pivot is judged by its pivot point).
"""
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))   # the tools folder
from utpp_paths import project_root  # noqa: E402
ROOT = project_root()
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'mmbake'))
import mmbake_base as v1  # noqa: E402

CFG1 = json.load(open(os.path.join(ROOT, 'Source/tools/mmbake/config/default.json')))
RIGS = {
    'woman': os.path.join(ROOT, 'Extracted/game/json/vanilla/player_woman_skeleton.rig.json'),
    'man': os.path.join(ROOT, 'Extracted/game/json/vanilla-man/player_man_skeleton.rig.json'),
}
_RIG = {}


def rig(g):
    if g not in _RIG:
        _RIG[g] = v1.load_rig(RIGS[g], int(CFG1['num_pose_bones']))
    return _RIG[g]


def bone_ids(g):
    names, parents = rig(g)
    return {n: i for i, n in enumerate(names)}


FPS = 30.0
FLAT_H = 0.03


class Clip:
    def __init__(self, path, g):
        self.path = path
        self.g = g
        self.d = json.load(open(path))
        self.names, self.parents = rig(g)
        self.bi = bone_ids(g)
        B = np.array([f['bones'] for f in self.d['frames']], dtype=np.float64)
        self.B = B
        self.N = B.shape[0]
        self.P = B[..., 0:3].copy()
        self.Q = B[..., 3:7].copy()
        self.c = {k: np.asarray(self.d['curves'][k], dtype=float) for k in ('contact_l', 'contact_r') if k in self.d.get('curves', {})}

    def world(self):
        """world positions/rotations (N,B,3),(N,B,4): model FK then Trajectory transform."""
        gp, gq = v1.fk_model(self.P, self.Q, self.parents)
        t = self.bi['Trajectory']
        tp, tq = gp[:, t], gq[:, t]
        wp = tp[:, None, :] + v1.qrot(np.repeat(tq[:, None, :], gp.shape[1], 1), gp)
        return gp, gq, wp

    def foot_points(self, side):
        gp, gq, wp = self.world()
        h = wp[:, self.bi[side + 'Heel']]
        t = wp[:, self.bi[side + 'ToeBase']]
        a = wp[:, self.bi[side + 'Foot']]
        return a, h, t


def contact_point_slide(a, h, t, flat_h=FLAT_H):
    """(N-1,) horizontal displacement (cm) between frame k and k+1 of the ground contact point."""
    dh = np.linalg.norm(np.diff(h[:, :2], axis=0), axis=1) * 100
    dt = np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1) * 100
    hz = 0.5 * (h[1:, 2] + h[:-1, 2])
    tz = 0.5 * (t[1:, 2] + t[:-1, 2])
    lower = np.where(hz < tz, dh, dt)
    both = (np.minimum(hz, tz) < flat_h) & (np.abs(hz - tz) < flat_h)
    return np.where(both, np.minimum(dh, dt), lower)


def segments(mask):
    """list of (start, end_inclusive) runs of True."""
    out, s = [], None
    for i, m in enumerate(mask):
        if m and s is None:
            s = i
        if (not m) and s is not None:
            out.append((s, i - 1))
            s = None
    if s is not None:
        out.append((s, len(mask) - 1))
    return out


PLANT_H = 0.015


def ground_levels(clip):
    """per-clip ground level of Heel/ToeBase joints = their minimum height over both feet (clip-specific rig offset)."""
    hs, ts = [], []
    for side in ('Left', 'Right'):
        a, h, t = clip.foot_points(side)
        hs.append(h[:, 2].min())
        ts.append(t[:, 2].min())
    return min(hs), min(ts)


def planted(clip, side, thr=0.5, plant_h=PLANT_H, gh=None, gt=None):
    """bool (N,): contact curve >= thr AND the heel or the toe within plant_h of its ground level (foot really on the floor)."""
    if gh is None:
        gh, gt = ground_levels(clip)
    a, h, t = clip.foot_points(side)
    cm = clip.c['contact_l' if side == 'Left' else 'contact_r'] >= thr
    return cm & ((h[:, 2] < gh + plant_h) | (t[:, 2] < gt + plant_h))


def measure(clip, thr=0.5):
    """dict side -> dict(frames, mean, p95, max (cm/frame over contact frame pairs), path (sum cm over all contact pairs),
    worst_seg (max segment path cm), segs)"""
    res = {}
    for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
        a, h, t = clip.foot_points(side)
        s = contact_point_slide(a, h, t)
        cm = planted(clip, side, thr)
        pair = cm[:-1] & cm[1:]
        sl = s[pair]
        segs = []
        for (b, e) in segments(pair):
            segs.append((b, e + 1, float(s[b:e + 1].sum())))
        res[side] = dict(frames=int(pair.sum()), mean=float(sl.mean()) if len(sl) else 0.0,
                         p95=float(np.percentile(sl, 95)) if len(sl) else 0.0, max=float(sl.max()) if len(sl) else 0.0,
                         path=float(sl.sum()), worst_seg=max([x[2] for x in segs], default=0.0), segs=segs)
    return res


def source_footspeed(clip, thr=0.5):
    """source pack source foot speed (UE cm/s, curves footspeed_l/r) while in contact -> cm/frame mean / max / path."""
    out = {}
    for side, fk, ck in (('Left', 'footspeed_l', 'contact_l'), ('Right', 'footspeed_r', 'contact_r')):
        fs = np.asarray(clip.d['curves'].get(fk, []), dtype=float) / FPS
        cm = clip.c[ck] >= thr
        v = fs[cm] if len(fs) == clip.N else np.zeros(0)
        out[side] = dict(mean=float(v.mean()) if len(v) else 0.0, max=float(v.max()) if len(v) else 0.0, path=float(v.sum()))
    return out


def toe_contact_speed_trimmed(clip, thr=0.5, trim=1, min_seg=3):
    """Mirror of the replay's FLTAB statistic (FeedReplayMotion.cpp FlFinalize) on the clip alone: horizontal world speed (cm/s) of the
    ToeBase in contact-curve segments (>= thr) with `trim` frames dropped at both ends (replay: 3 frames at 60 Hz = 1.5 at 30 Hz, min 10 frames at 60 Hz = 5 at 30 Hz -> trim 1, min_seg 3). Returns (n_pairs, mean cm/s, list)."""
    out = []
    for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
        a, h, t = clip.foot_points(side)
        sp = np.linalg.norm(np.diff(t[:, :2], axis=0), axis=1) * 100 * FPS
        for (b, e) in segments(clip.c[key] >= thr):
            if e - b + 1 < 2 * trim + min_seg:
                continue
            out += list(sp[b + trim:e - trim])   # pairs (k, k+1) for k in b+trim .. e-trim-1
    return len(out), (float(np.mean(out)) if out else 0.0), out
