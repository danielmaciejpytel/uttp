"""getup_fixup.py - offline floor / contact fix-up of the source pack ragdoll get-up clips (UTPP ragdoll get-up, spec 7 "A").

Library used by getupbake.py and test_getupbake.py. source pack derived output: keep outside git.

Pipeline for one clip (all in the retarget rig, metres, +Z up, +Y forward, quaternions xyzw, local transforms):
  1. bake_trajectory: the source clips are authored relative to Root + Trajectory (planted hands and feet are stationary
     only after the Trajectory translation is added). The DLL plays the pose Root-relative from one world anchor and never
     reads the Trajectory bone, so the Trajectory is folded into the Hips local transform (Trajectory bone -> identity).
  2. contacts: planted hand / foot intervals (height + speed), lock weights (triangular blur, +-3 frames).
  3. solve: violation driven correction with smooth (max-filter + triangular blur, never lowers a peak) parameters:
       L        pelvis lift (Hips z)
       lift[k]  extra z of the wrist / ankle target
       th[k]    hand / foot tilt about the wrist / ankle (raises fingers / toes)
       phi[k]   swivel of the elbow / knee about the shoulder-wrist axis
     planted effectors blend to their locked xy position; two-bone IK keeps the original pole direction.
     Only local rotations of Arm/ForeArm/Hand (Leg) bones and the Hips z are changed: bone lengths stay exact.
  4. cut / interrupt selection.
"""
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import travbake as T  # noqa: E402

FPS = 30.0
# Shared contract (spec 3.4): clamp set radii in metres. Fingers use FINGER_R. Everything else is not a clamp joint.
CLAMP_R = {"Hips": 0.10, "Spine3": 0.10, "Head": 0.09, "LeftArm": 0.05, "RightArm": 0.05,
           "LeftForeArm": 0.04, "RightForeArm": 0.04, "LeftHand": 0.03, "RightHand": 0.03,
           "LeftLeg": 0.05, "RightLeg": 0.05, "LeftFoot": 0.05, "RightFoot": 0.05,
           "LeftToeBase": 0.02, "RightToeBase": 0.02}
FINGER_R = 0.01
HIP_R = 0.08          # solver-only extra radius of the LeftUpLeg / RightUpLeg joints (not part of the shared clamp set)
FINGER_RE = re.compile(r"^(Left|Right)Hand(Thumb|Index|Middle|Ring|Pinky)[123]$")
# NOT clamp joints: Root, Trajectory, reference_joint, Weapon*, *HandEnd, *InHand*, *Eye, *Heel, Neck*, Spine/Spine1/Spine2, shoulders.
LIMBS = {  # key: (shoulder/hip, elbow/knee, wrist/ankle, direction bone for the tilt axis, side)
    "LH": ("LeftArm", "LeftForeArm", "LeftHand", "LeftHandMiddle3"),
    "RH": ("RightArm", "RightForeArm", "RightHand", "RightHandMiddle3"),
    "LF": ("LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase"),
    "RF": ("RightUpLeg", "RightLeg", "RightFoot", "RightToeBase"),
}
LIMB_KEYS = ("LH", "RH", "LF", "RF")
TILT_MAX = math.radians(60.0)
SWIVEL_MAX = math.radians(40.0)
TOL = 0.001           # a clamp joint is "violating" when z < r - TOL/2
MESH_MARGIN = 0.002   # skinned-mesh floor clearance wanted by the solver (m): the lowest vertex ends up >= margin - TOL/2
SHRUG_MAX = math.radians(25.0)
FOOT_YAW_MAX = math.radians(35.0)
RATE_L, RATE_LIFT = 0.008, 0.012          # per frame (m)
RATE_TH, RATE_PHI, RATE_SH = math.radians(5.0), math.radians(2.5), math.radians(2.5)  # per frame (rad)
KERNEL = np.array([1, 2, 3, 4, 3, 2, 1], dtype=np.float64) / 16.0


# --------------------------------------------------------------------------------------------------------------------
# quaternion / vector helpers (arrays with a trailing dim of 4 or 3)
def qmul(a, b):
    ax, ay, az, aw = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bx, by, bz, bw = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


def qconj(q):
    return q * np.array([-1.0, -1.0, -1.0, 1.0])


def qrot(q, v):
    qv = q[..., :3]
    t = 2.0 * np.cross(qv, v)
    return v + q[..., 3:4] * t + np.cross(qv, t)


def qnormalize(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def q_axis_angle(axis, ang):
    s = math.sin(ang * 0.5)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang * 0.5)])


def unit(v, fallback=(0.0, 0.0, 1.0)):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.array(fallback, dtype=np.float64)
    return np.asarray(v, dtype=np.float64) / n


def rot_between(u, v):
    """Minimal rotation quaternion taking unit vector u to unit vector v."""
    u, v = unit(u), unit(v)
    d = float(np.dot(u, v))
    if d > 1.0 - 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    if d < -1.0 + 1e-9:
        a = unit(np.cross(u, (1.0, 0.0, 0.0) if abs(u[0]) < 0.9 else (0.0, 1.0, 0.0)))
        return np.array([a[0], a[1], a[2], 0.0])
    c = np.cross(u, v)
    return qnormalize(np.array([c[0], c[1], c[2], 1.0 + d]))


def rodrigues(v, axis, ang):
    axis = unit(axis)
    return v * math.cos(ang) + np.cross(axis, v) * math.sin(ang) + axis * np.dot(axis, v) * (1 - math.cos(ang))


def qangle(a, b):
    """Angle in radians between two unit quaternions (shortest arc)."""
    d = np.abs(np.sum(a * b, axis=-1))
    return 2.0 * np.arccos(np.clip(d, 0.0, 1.0))


# --------------------------------------------------------------------------------------------------------------------
class Rig:
    def __init__(self, root, body):
        rj = {"woman": "Extracted/game/json/vanilla/player_woman_skeleton.rig.json",
              "man": "Extracted/game/json/vanilla-man/player_man_skeleton.rig.json"}[body]
        self.body = body
        self.json_path = os.path.join(root, rj)
        self.names, self.parents, self.hash = T.rig_info(self.json_path)
        self.rig_name = os.path.basename(rj).split(".")[0]


class Clip:
    """Source clip arrays. pos (N,B,3), rot (N,B,4) local; par (B,) parent index inside the clip bone list."""

    def __init__(self, path, rig):
        d = json.load(open(path, "r", encoding="utf-8"))
        self.path = path
        self.src = d
        self.names = list(d["bone_names"])
        self.fps = float(d["fps"])
        self.n = int(d["num_frames"])
        b = np.array([f["bones"] for f in d["frames"]], dtype=np.float64)
        self.pos = b[:, :, 0:3].copy()
        self.rot = qnormalize(b[:, :, 3:7].copy())
        self.scale = b[:, :, 7:10].copy() if b.shape[2] >= 10 else None
        idx = {n: i for i, n in enumerate(self.names)}
        self.idx = idx
        rig_idx = {n: i for i, n in enumerate(rig.names)}
        self.par = np.array([idx[rig.names[rig.parents[rig_idx[n]]]] if rig.parents[rig_idx[n]] >= 0 else -1
                             for n in self.names])
        for i, p in enumerate(self.par):
            if p >= i:
                raise SystemExit("clip bone order: parent after child (%s)" % self.names[i])
        self.notifies = [(x.get("class", ""), float(x["trigger_time"])) for x in d.get("notifies", [])]
        self.name = d["source_clip"]


def fk(pos, rot, par):
    """World positions / rotations for (N,B,*) local arrays."""
    wp = np.zeros_like(pos)
    wq = np.zeros_like(rot)
    for b in range(pos.shape[1]):
        p = par[b]
        if p < 0:
            wp[:, b] = pos[:, b]
            wq[:, b] = rot[:, b]
        else:
            wp[:, b] = wp[:, p] + qrot(wq[:, p], pos[:, b])
            wq[:, b] = qmul(wq[:, p], rot[:, b])
    return wp, wq


def bake_trajectory(clip):
    """Fold the Trajectory bone into the Hips local transform. Returns (pos, rot, traj_xy_yaw) with Trajectory identity."""
    pos, rot = clip.pos.copy(), clip.rot.copy()
    ti, hi = clip.idx["Trajectory"], clip.idx["Hips"]
    if clip.par[hi] != clip.idx["Root"] or clip.par[ti] != clip.idx["Root"]:
        raise SystemExit("Hips / Trajectory are expected to be Root children")
    tp, tq = clip.pos[:, ti], clip.rot[:, ti]
    # Root local is identity in these clips; Root-space = world.
    pos[:, hi] = tp + qrot(tq, clip.pos[:, hi])
    rot[:, hi] = qnormalize(qmul(tq, clip.rot[:, hi]))
    pos[:, ti] = 0.0
    rot[:, ti] = np.array([0.0, 0.0, 0.0, 1.0])
    traj = np.stack([tp[:, 0], tp[:, 1], tp[:, 2], np.array([T.qyaw(q) for q in tq])], -1)
    return pos, rot, traj


def clamp_radii(names):
    """{bone index: radius} of the clamp set + fingers."""
    out = {}
    for i, n in enumerate(names):
        if n in CLAMP_R:
            out[i] = CLAMP_R[n]
        elif FINGER_RE.match(n):
            out[i] = FINGER_R
    return out


# --------------------------------------------------------------------------------------------------------------------
def cone(x, rate):
    """Rate-limited upper envelope: max_s (x[s] - rate * |t - s|) (two passes)."""
    y = np.array(x, dtype=np.float64)
    for t in range(1, len(y)):
        y[t] = max(y[t], y[t - 1] - rate)
    for t in range(len(y) - 2, -1, -1):
        y[t] = max(y[t], y[t + 1] - rate)
    return y


def env_smooth(x, rate=None):
    """Max-filter over +-3 frames, optional rate limit (cone), then triangular blur over +-3: the result is >= x everywhere
    (a peak is never lowered), and with `rate` it can not change faster than about `rate` per frame beyond the peaks."""
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    p = np.pad(x, (6, 6), mode="edge")
    mx = np.max(np.stack([p[k:k + n + 6] for k in range(7)]), axis=0)  # frames -3 .. n+2
    if rate is not None:
        mx = cone(mx, rate)
    return sum(KERNEL[m] * mx[m:m + n] for m in range(7))


def env_smooth_ref(x):
    """Straightforward reference of env_smooth (used by the tests): edge-extended input, box max, triangular blur."""
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    xe = lambda i: x[min(max(i, 0), n - 1)]  # noqa: E731
    mx = lambda f: max(xe(f + k) for k in range(-3, 4))  # noqa: E731
    return np.array([sum(KERNEL[k + 3] * mx(i + k) for k in range(-3, 4)) for i in range(n)])


def blur(x):
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    cl = lambda i: min(max(i, 0), n - 1)  # noqa: E731
    return np.array([sum(KERNEL[k + 3] * x[cl(i + k)] for k in range(-3, 4)) for i in range(n)])


# --------------------------------------------------------------------------------------------------------------------
def detect_contacts(clip, wp, key, z_max, v_max=0.30, min_len=4, gap=2, max_dev=0.05):
    """Planted intervals of one effector from baked world positions (height + xy speed). Returns list of (a, b) inclusive."""
    e = clip.idx[LIMBS[key][2]]
    p = wp[:, e]
    n = len(p)
    sp = np.zeros(n)
    for t in range(n):
        a, b = max(0, t - 1), min(n - 1, t + 1)
        sp[t] = math.hypot(*(p[b, :2] - p[a, :2])) / ((b - a) / clip.fps)
    raw = (p[:, 2] <= z_max) & (sp <= v_max)
    # close small gaps
    t = 0
    while t < n:
        if not raw[t]:
            u = t
            while u < n and not raw[u]:
                u += 1
            if t > 0 and u < n and u - t <= gap:
                raw[t:u] = True
            t = u
        else:
            t += 1
    runs = []
    t = 0
    while t < n:
        if raw[t]:
            u = t
            while u < n and raw[u]:
                u += 1
            runs.append([t, u - 1])
            t = u
        else:
            t += 1
    out = []
    for a, b in runs:
        # trim until the xy deviation from the mean is small
        while b - a + 1 >= min_len:
            m = p[a:b + 1, :2].mean(0)
            dev = np.linalg.norm(p[a:b + 1, :2] - m, axis=1)
            if dev.max() <= max_dev:
                break
            if dev[0] >= dev[-1]:
                a += 1
            else:
                b -= 1
        if b - a + 1 >= min_len:
            out.append((a, b))
    return out


class Solver:
    def __init__(self, clip, pos, rot, locks, radii, verbose=False, mesh=None, mesh_upto=None, mesh_margin=MESH_MARGIN):
        """pos/rot: baked local arrays (N,B,*); locks: {key: [(a,b),...]}.
        mesh: optional getup_mesh.ZSkin over the clip bones. The lowest skinned vertex of frames 0..mesh_upto is kept at least
        mesh_margin above the floor. A vertex deficit is a floor violation of the solver bone that owns the vertex
        (getup_mesh.solver_bone: fingers -> wrist target lift, toes -> ankle lift, arm / forearm / knee -> shrug / swivel, rest ->
        pelvis lift), so it is resolved by the same levers as a clamp-joint violation."""
        self.c = clip
        self.mesh = mesh
        self.mesh_upto = (clip.n - 1) if mesh_upto is None else min(mesh_upto, clip.n - 1)
        self.mesh_margin = mesh_margin
        self.pos0, self.rot0 = pos, rot
        self.n = clip.n
        self.wp0, self.wq0 = fk(pos, rot, clip.par)
        self.radii = radii
        self.verbose = verbose
        idx = clip.idx
        self.ci = {k: tuple(idx[x] for x in LIMBS[k]) for k in LIMB_KEYS}
        self.rE = {k: radii[self.ci[k][2]] for k in LIMB_KEYS}
        self.rM = {k: radii[self.ci[k][1]] for k in LIMB_KEYS}
        self.hips = idx["Hips"]
        # clavicle (Shoulder) bone of the arms: shrug raises the Arm joint before the pelvis is lifted
        self.sh_idx = {"LH": idx["LeftShoulder"], "RH": idx["RightShoulder"]}
        self.trunk = [idx[x] for x in ("Hips", "Spine3", "Head", "LeftUpLeg", "RightUpLeg")]
        self.arm_joint = {idx["LeftArm"]: "LH", idx["RightArm"]: "RH"}
        # descendants of each wrist / ankle in the clamp set
        self.desc = {}
        for k in LIMB_KEYS:
            e = self.ci[k][2]
            ds = []
            for b in radii:
                p = b
                while p >= 0 and p != e:
                    p = clip.par[p]
                if p == e and b != e:
                    ds.append(b)
            self.desc[k] = ds
        self.lock_ind = {}
        self.lock_w = {}
        self.lock_xy = {}
        self.toe_rel = {}
        for k in LIMB_KEYS:
            ind = np.zeros(self.n)
            lxy = np.zeros((self.n, 2))
            e = self.ci[k][2]
            ivs = locks.get(k, [])
            for a, b in ivs:
                ind[a:b + 1] = 1.0
                lxy[a:b + 1] = self.wp0[a:b + 1, e, :2].mean(0)
            self.lock_ind[k] = ind
            self.lock_w[k] = blur(ind)
            # lock position of the blend frames outside an interval = mean of the nearest interval (in time)
            for t in range(self.n):
                if ind[t] > 0 or not ivs:
                    continue
                best = min(ivs, key=lambda ab: (min(abs(t - ab[0]), abs(t - ab[1])), ab[0]))
                lxy[t] = self.wp0[best[0]:best[1] + 1, e, :2].mean(0)
            self.lock_xy[k] = lxy
            if k[1] == "F" and ivs:
                # planted foot: horizontal ankle->toe direction of the nearest interval (keeps the toe from sweeping on the floor)
                toe = idx[LIMBS[k][3]]
                rel = self.wp0[:, toe, :2] - self.wp0[:, e, :2]
                tr = np.zeros((self.n, 2))
                for t in range(self.n):
                    best = min(ivs, key=lambda ab: (0 if ab[0] <= t <= ab[1] else min(abs(t - ab[0]), abs(t - ab[1])), ab[0]))
                    tr[t] = rel[best[0]:best[1] + 1].mean(0)
                self.toe_rel[k] = tr
        n = self.n
        self.L = np.zeros(n)
        self.lift = {k: np.zeros(n) for k in LIMB_KEYS}
        self.lift_extra = {k: np.zeros(n) for k in LIMB_KEYS}
        self.need_abs = {}
        self.th = {k: np.zeros(n) for k in LIMB_KEYS}
        self.phi = {k: np.zeros(n) for k in LIMB_KEYS}
        self.sh = {k: np.zeros(n) for k in ("LH", "RH")}
        self.slip_pulled = 0
        self.use_limit = False
        self.n_limited = 0
        self._build_axes()
        self._init_tilt()

    def enable_mesh(self, mesh, upto=None, margin=MESH_MARGIN):
        """Start keeping the skinned mesh above the floor (see __init__). Call after the joint-only solve converged: the mesh pass
        then only adds what the clamp-set solution leaves, instead of overshooting from the raw source."""
        self.mesh = mesh
        self.mesh_upto = (self.n - 1) if upto is None else min(upto, self.n - 1)
        self.mesh_margin = margin

    # ---- initial tilt: cheapest tilt/lift trade per frame ------------------------------------------------------------
    def _tilt_axis(self, k, t):
        return self.axis[k][t]

    def _build_axes(self):
        """Tilt axis per limb and frame: horizontal axis perpendicular to the forearm (hands) / foot (feet) direction, so it is
        stable while the hand points straight down. Degenerate frames reuse the previous axis, then a +-3 frame blur."""
        self.axis = {}
        for k in LIMB_KEYS:
            s, m, e, _ = self.ci[k]
            didx = self.c.idx[LIMBS[k][3]]
            raw = np.zeros((self.n, 3))
            prev = None
            for t in range(self.n):
                src = (self.wp0[t, e] - self.wp0[t, m]) if k[1] == "H" else (self.wp0[t, didx] - self.wp0[t, e])
                hv = np.array([src[0], src[1], 0.0])
                if np.linalg.norm(hv) >= 0.03:
                    prev = unit(np.cross(unit(hv), (0.0, 0.0, 1.0)))
                if prev is None:
                    prev = np.array([1.0, 0.0, 0.0])
                raw[t] = prev
            sm = np.stack([blur(raw[:, i]) for i in range(3)], -1)
            self.axis[k] = np.array([unit(v, tuple(r)) for v, r in zip(sm, raw)])

    def _init_tilt(self):
        grid = [math.radians(a) for a in range(0, 61, 2)]
        for k in LIMB_KEYS:
            e = self.ci[k][2]
            ds = self.desc[k] + [e]
            for t in range(self.n):
                ez = self.wp0[t, e, 2]
                if ez > 0.35:
                    continue
                ax = self._tilt_axis(k, t)
                best = None
                for th in grid:
                    q = q_axis_angle(ax, th)
                    need = 0.0
                    for b in ds:
                        v = self.wp0[t, b] - self.wp0[t, e]
                        z = ez + qrot(q, v)[2]
                        need = max(need, self.radii[b] - z)
                    cost = need * 100.0 + math.degrees(th) * 0.12
                    if best is None or cost < best[0] - 1e-12:
                        best = (cost, th)
                self.th[k][t] = best[1]
            self.th[k] = np.minimum(env_smooth(self.th[k], RATE_TH), TILT_MAX)
            # exact wrist height needed so that wrist + hand-chain joints clear their radii with the (now fixed) tilt
            need = np.full(self.n, -1.0)
            for t in range(self.n):
                if self.wp0[t, e, 2] > 0.6:
                    continue
                q = q_axis_angle(self.axis[k][t], self.th[k][t])
                nd = self.radii[e]
                for b in self.desc[k]:
                    nd = max(nd, self.radii[b] - qrot(q, self.wp0[t, b] - self.wp0[t, e])[2])
                need[t] = nd
            self.need_abs[k] = need

    # ---- IK ------------------------------------------------------------------------------------------------------------
    @staticmethod
    def ik_mid(S, Et, M0, l1, l2, E0, phi):
        """Elbow/knee position for shoulder S, wanted wrist Et; pole preserved from the original (M0, E0 relative to S).
        Returns (M, E_achieved)."""
        axis0 = unit(E0 - S)
        v = M0 - S
        n0 = v - axis0 * np.dot(v, axis0)
        a_full = Et - S
        d = float(np.linalg.norm(a_full))
        a = unit(a_full, tuple(axis0))
        dmax = (l1 + l2) * 0.9995
        dmin = abs(l1 - l2) * 1.0005 + 1e-4
        dc = min(max(d, dmin), dmax)
        n = n0 - a * np.dot(n0, a)
        n = unit(n, tuple(unit(np.cross(a, (1.0, 0.0, 0.0)))))
        if phi != 0.0:
            n = rodrigues(n, a, phi)
        x = (l1 * l1 - l2 * l2 + dc * dc) / (2 * dc)
        y = math.sqrt(max(l1 * l1 - x * x, 0.0))
        return S + a * x + n * y, S + a * dc

    # ---- one frame ----------------------------------------------------------------------------------------------------
    def chain(self, t, k):
        """Geometry of limb k at frame t after the pelvis lift and the clavicle shrug (before IK):
        (S0, M0, E0, Rsh, l1, l2, Et)."""
        s, m, e, _ = self.ci[k]
        L = self.L[t]
        up = np.array([0.0, 0.0, L])
        S0 = self.wp0[t, s] + up
        M0 = self.wp0[t, m] + up
        E0 = self.wp0[t, e] + up
        Rsh = None
        if k in self.sh_idx and self.sh[k][t] > 1e-9:
            si = self.sh_idx[k]
            Sh0 = self.wp0[t, si] + up
            ax = unit(np.cross(self.wp0[t, s] - self.wp0[t, si], (0.0, 0.0, 1.0)), (1.0, 0.0, 0.0))
            Rsh = q_axis_angle(ax, self.sh[k][t])
            S0 = Sh0 + qrot(Rsh, S0 - Sh0)
            M0 = Sh0 + qrot(Rsh, M0 - Sh0)
            E0 = Sh0 + qrot(Rsh, E0 - Sh0)
        l1 = float(np.linalg.norm(self.wp0[t, m] - self.wp0[t, s]))
        l2 = float(np.linalg.norm(self.wp0[t, e] - self.wp0[t, m]))
        w = self.lock_w[k][t]
        # free target: the original wrist (moved with the body); planted: blend to the locked xy, z stays on the floor
        E_free = self.wp0[t, e] + up
        Et = np.array([(1 - w) * E_free[0] + w * self.lock_xy[k][t, 0], (1 - w) * E_free[1] + w * self.lock_xy[k][t, 1],
                       self.wp0[t, e, 2] + (1 - w) * L + self.lift[k][t]])
        Et[2] = max(Et[2], self.rE[k])
        return S0, M0, E0, Rsh, l1, l2, Et

    def apply_frame(self, t, lq, lp):
        """Write the corrected local rotations of the limb bones for frame t into lq (N,B,4); Hips z into lp."""
        c = self.c
        lp[t, self.hips, 2] = self.pos0[t, self.hips, 2] + self.L[t]
        for k in LIMB_KEYS:
            s, m, e, _ = self.ci[k]
            S0, M0, E0, Rsh, l1, l2, Et = self.chain(t, k)
            Mn, En = self.ik_mid(S0, Et, M0, l1, l2, E0, self.phi[k][t])
            if np.linalg.norm(En - Et) > 0.003:
                self.slip_pulled += 1
            ps = c.par[s]
            wq_ps = self.wq0[t, ps]
            base_s = self.wq0[t, s]
            base_m = self.wq0[t, m]
            if Rsh is not None:
                si = self.sh_idx[k]
                lq[t, si] = qnormalize(qmul(qconj(self.wq0[t, c.par[si]]), qmul(Rsh, self.wq0[t, si])))
                wq_ps = qmul(Rsh, wq_ps)
                base_s = qmul(Rsh, base_s)
                base_m = qmul(Rsh, base_m)
            R1 = rot_between(M0 - S0, Mn - S0)
            wq_s = qmul(R1, base_s)
            lq[t, s] = qnormalize(qmul(qconj(wq_ps), wq_s))
            E1 = Mn + qrot(R1, E0 - M0)
            R2 = rot_between(E1 - Mn, En - Mn)
            wq_m = qmul(R2, qmul(R1, base_m))
            lq[t, m] = qnormalize(qmul(qconj(wq_s), wq_m))
            ax = self._tilt_axis(k, t)
            q_tilt = q_axis_angle(ax, self.th[k][t])
            wq_e = qmul(q_tilt, self.wq0[t, e])
            w = self.lock_w[k][t]
            if k in self.toe_rel and w > 1e-6:
                # yaw the planted foot about the ankle so that the toe keeps the interval's direction (no toe sweep)
                toe = c.idx[LIMBS[k][3]]
                hv = qrot(q_tilt, self.wp0[t, toe] - self.wp0[t, e])[:2]
                tg = self.toe_rel[k][t]
                if np.linalg.norm(hv) > 0.03 and np.linalg.norm(tg) > 0.03:
                    ang = math.atan2(hv[0] * tg[1] - hv[1] * tg[0], hv[0] * tg[0] + hv[1] * tg[1])
                    ang = max(-FOOT_YAW_MAX, min(FOOT_YAW_MAX, ang)) * w
                    wq_e = qmul(q_axis_angle((0.0, 0.0, 1.0), ang), wq_e)
            lq[t, e] = qnormalize(qmul(qconj(wq_m), wq_e))

    def build(self):
        lq = self.rot0.copy()
        lp = self.pos0.copy()
        for t in range(self.n):
            self.apply_frame(t, lq, lp)
        self.n_limited = 0
        if self.use_limit:
            lq, self.n_limited = limit_steps(lq, 25.0)
        return lp, lq

    def violations(self, lp, lq):
        wp, wq = fk(lp, lq, self.c.par)
        viol = {b: r - wp[:, b, 2] for b, r in self.radii.items()}
        if self.mesh is not None:
            u = self.mesh_upto + 1
            gm = self.mesh.group_min(self.mesh.z(wp[:u], wq[:u]))
            for g, m in gm.items():
                b = self.c.idx[g]
                a = np.full(self.n, -1e9)
                a[:u] = self.mesh_margin - m
                viol[b] = np.maximum(viol[b], a)
        return wp, viol

    def _update_lift(self):
        for k in LIMB_KEYS:
            e = self.ci[k][2]
            raw = np.maximum(self.need_abs[k] - self.wp0[:, e, 2] - (1.0 - self.lock_w[k]) * self.L, 0.0)
            raw[self.need_abs[k] < 0] = 0.0
            self.lift[k] = env_smooth(raw + self.lift_extra[k], RATE_LIFT)

    # ---- violation driven solve ---------------------------------------------------------------------------------------
    def mid_choose(self, t, k):
        """Swivel sign (+1/-1) that raises the elbow/knee at frame t, or 0 if neither helps."""
        S0, M0, E0, Rsh, l1, l2, Et = self.chain(t, k)
        cur = self.phi[k][t]
        d = math.radians(3.0)
        z0 = self.ik_mid(S0, Et, M0, l1, l2, E0, cur)[0][2]
        zp = self.ik_mid(S0, Et, M0, l1, l2, E0, cur + d)[0][2]
        zm = self.ik_mid(S0, Et, M0, l1, l2, E0, cur - d)[0][2]
        if max(zp, zm) <= z0 + 1e-4:
            return 0
        return 1 if zp >= zm else -1

    def _params(self):
        """(container, key, shrink step, signed) of every relaxable parameter array."""
        out = [(self.__dict__, "L", 0.003)]
        for k in LIMB_KEYS:
            out.append((self.phi, k, math.radians(1.0)))
        for k in ("LH", "RH"):
            out.append((self.sh, k, math.radians(1.0)))
        return out

    def relax(self, rounds=40):
        """Shrink L / swivel / shrug wherever the floor constraints stay satisfied (the violation loop only ever grows them, so
        they overshoot and spread through the envelopes). Frames (+-4) whose shrunken pose violates keep their old values."""
        for rnd in range(rounds):
            saved = [(c, k, np.array(c[k])) for c, k, _ in self._params()]
            for c, k, step in self._params():
                a = c[k]
                c[k] = np.sign(a) * np.maximum(np.abs(a) - step, 0.0)
            self._update_lift()
            lp, lq = self.build()
            wp, viol = self.violations(lp, lq)
            bad = np.zeros(self.n, dtype=bool)
            for v in viol.values():
                bad |= v > TOL * 0.5
            bad = np.convolve(bad.astype(float), np.ones(9), mode="same") > 0
            changed = False
            for c, k, old in saved:
                new = np.array(c[k])
                new[bad] = old[bad]
                if np.abs(new - old).max() > 0:
                    changed = True
                c[k] = new
            if not changed:
                break
        # the frame-wise shrink / restore leaves steps: re-apply the (never lowering) rate-limited envelopes
        self.L = env_smooth(self.L, RATE_L)
        for k in LIMB_KEYS:
            ph = self.phi[k]
            self.phi[k] = (np.minimum(env_smooth(np.maximum(ph, 0.0), RATE_PHI), SWIVEL_MAX)
                           - np.minimum(env_smooth(np.maximum(-ph, 0.0), RATE_PHI), SWIVEL_MAX))
        for k in ("LH", "RH"):
            self.sh[k] = np.minimum(env_smooth(self.sh[k], RATE_SH), SHRUG_MAX)
        self._update_lift()

    def solve(self, iters=160):
        n = self.n
        rep = []
        for it in range(iters):
            self._update_lift()
            lp, lq = self.build()
            wp, viol = self.violations(lp, lq)
            worst = max(float(v.max()) for v in viol.values())
            rep.append(worst)
            if self.verbose:
                wk = max(viol, key=lambda kk: float(viol[kk].max()))
                print("  iter %d worst violation %.4f (%s f%d)  Lmax %.3f" % (
                    it, worst, self.c.names[wk], int(np.argmax(viol[wk])), self.L.max()))
            if worst <= TOL * 0.5:
                if not self.use_limit:
                    self.use_limit = True      # from now on the 25 deg/frame limiter is part of the pose that is checked
                    continue
                break
            inc = 1.0 + 0.1 * it
            dL = np.zeros(n)
            dlift = {k: np.zeros(n) for k in LIMB_KEYS}
            dphi = {k: np.zeros(n) for k in LIMB_KEYS}
            dsh = {k: np.zeros(n) for k in ("LH", "RH")}
            tol = TOL * 0.5
            anyv = np.zeros(n, dtype=bool)
            for v in viol.values():
                anyv |= v > tol
            for t in np.nonzero(anyv)[0]:
                act_L = 0.0
                for k in LIMB_KEYS:
                    s, m, e, _ = self.ci[k]
                    ve = max(float(viol[b][t]) for b in [e] + self.desc[k])
                    if ve > tol:
                        dlift[k][t] = ve
                        continue
                    if k in self.sh_idx and float(viol[s][t]) > tol:
                        if self.sh[k][t] < SHRUG_MAX - 1e-6 and it < 24:
                            dsh[k][t] = math.radians(2.0 * inc)
                        else:
                            act_L = max(act_L, float(viol[s][t]) * 0.5 * inc)
                        continue
                    vm = float(viol[m][t])
                    if vm > tol:
                        sg = self.mid_choose(t, k)
                        if it < 24 and sg != 0 and abs(self.phi[k][t]) < SWIVEL_MAX - 1e-6:
                            dphi[k][t] = sg * math.radians(1.0 * inc)
                        else:
                            act_L = max(act_L, vm * 0.5 * inc)
                for b in self.trunk:
                    if float(viol[b][t]) > tol:
                        act_L = max(act_L, float(viol[b][t]) * 0.5 * inc)
                dL[t] = act_L
            self.L = env_smooth(self.L + dL, RATE_L)
            for k in LIMB_KEYS:
                self.lift_extra[k] = self.lift_extra[k] + dlift[k]
                ph = self.phi[k] + dphi[k]
                self.phi[k] = (np.minimum(env_smooth(np.maximum(ph, 0.0), RATE_PHI), SWIVEL_MAX)
                               - np.minimum(env_smooth(np.maximum(-ph, 0.0), RATE_PHI), SWIVEL_MAX))
            for k in ("LH", "RH"):
                self.sh[k] = np.minimum(env_smooth(self.sh[k] + dsh[k], RATE_SH), SHRUG_MAX)
        self._update_lift()
        lp, lq = self.build()
        wp, viol = self.violations(lp, lq)
        rep.append(max(float(v.max()) for v in viol.values()))
        return lp, lq, rep


# --------------------------------------------------------------------------------------------------------------------
def metrics_clearance(clip, lp, lq, radii, upto=None):
    wp, _ = fk(lp, lq, clip.par)
    n = clip.n if upto is None else upto + 1
    best = (1e9, None, None)
    for b, r in radii.items():
        cl = wp[:n, b, 2] - r
        t = int(np.argmin(cl))
        if cl[t] < best[0]:
            best = (float(cl[t]), clip.names[b], t)
    return best


def joint_step_deg(lq, upto=None):
    n = lq.shape[0] if upto is None else upto + 1
    a = lq[:n - 1]
    b = lq[1:n]
    ang = np.degrees(qangle(a, b))  # (n-1, B)
    return ang


def limit_steps(lq, max_deg=25.0, passes=12, protect=None):
    """Smooth the local rotation tracks of bones whose frame-to-frame rotation exceeds max_deg (source pops included).
    Returns (lq, number of smoothed bone-frames). `protect`: set of bone indices that must not be touched."""
    lq = lq.copy()
    total = 0
    n = lq.shape[0]
    for _ in range(passes):
        a = np.degrees(qangle(lq[:-1], lq[1:]))  # (n-1, B)
        bad = np.argwhere(a > max_deg)
        if len(bad) == 0:
            break
        for t, b in bad:
            if protect and b in protect:
                continue
            total += 1
            lo, hi = max(0, t - 2), min(n - 1, t + 3)
            ref = lq[t, b]
            for f in range(lo, hi + 1):
                acc = np.zeros(4)
                ws = 0.0
                for k in range(-3, 4):
                    g = min(max(f + k, 0), n - 1)
                    q = lq[g, b]
                    if np.dot(q, ref) < 0:
                        q = -q
                    w = KERNEL[k + 3]
                    acc += w * q
                    ws += w
                lq[f, b] = acc / np.linalg.norm(acc)
    return lq, total


# Capsule proxy of the body (NOT the skinned mesh: no body glb export exists in the tree): segment (a, b, radius a, radius b).
CAPSULES = [("Hips", "Spine3", 0.10, 0.10), ("Spine3", "Head", 0.10, 0.09),
            ("LeftArm", "LeftForeArm", 0.05, 0.04), ("LeftForeArm", "LeftHand", 0.04, 0.03),
            ("RightArm", "RightForeArm", 0.05, 0.04), ("RightForeArm", "RightHand", 0.04, 0.03),
            ("LeftHand", "LeftHandMiddle3", 0.03, 0.01), ("RightHand", "RightHandMiddle3", 0.03, 0.01),
            ("LeftUpLeg", "LeftLeg", 0.08, 0.05), ("LeftLeg", "LeftFoot", 0.05, 0.05), ("LeftFoot", "LeftToeBase", 0.04, 0.02),
            ("RightUpLeg", "RightLeg", 0.08, 0.05), ("RightLeg", "RightFoot", 0.05, 0.05), ("RightFoot", "RightToeBase", 0.04, 0.02)]


def capsule_clearance(clip, wp, upto):
    """Lowest (z - radius) over sample points of the capsule proxy for frames 0..upto. Returns (value, segment, frame)."""
    best = (1e9, None, None)
    for a, b, ra, rb in CAPSULES:
        ia, ib = clip.idx[a], clip.idx[b]
        for u in np.linspace(0.0, 1.0, 9):
            z = wp[:upto + 1, ia, 2] * (1 - u) + wp[:upto + 1, ib, 2] * u - (ra * (1 - u) + rb * u)
            t = int(np.argmin(z))
            if z[t] < best[0]:
                best = (float(z[t]), "%s-%s" % (a, b), t)
    return best


def capsule_gap(clip, wp, upto):
    """Per frame 0..upto: lowest (z - radius) of the capsule proxy = the gap between the body and the floor (negative = inside)."""
    g = np.full(upto + 1, 1e9)
    for a, b, ra, rb in CAPSULES:
        ia, ib = clip.idx[a], clip.idx[b]
        for u in np.linspace(0.0, 1.0, 9):
            g = np.minimum(g, wp[:upto + 1, ia, 2] * (1 - u) + wp[:upto + 1, ib, 2] * u - (ra * (1 - u) + rb * u))
    return g


def toe_plant_stats(clip, wp, upto, z_max=0.06, v_max=0.25, min_len=3):
    """Runs of toe z < z_max and xy speed < v_max for >= min_len frames; returns {toe: worst xy extent (m) over the runs}."""
    out = {}
    for toe in ("LeftToeBase", "RightToeBase"):
        p = wp[:upto + 1, clip.idx[toe]]
        n = len(p)
        flag = np.zeros(n, dtype=bool)
        for t in range(n):
            a, b = max(0, t - 1), min(n - 1, t + 1)
            v = math.hypot(*(p[b, :2] - p[a, :2])) / ((b - a) / clip.fps)
            flag[t] = p[t, 2] < z_max and v < v_max
        worst, t = 0.0, 0
        while t < n:
            if flag[t]:
                u = t
                while u < n and flag[u]:
                    u += 1
                if u - t >= min_len:
                    seg = p[t:u, :2]
                    worst = max(worst, float(np.max(np.linalg.norm(seg[:, None] - seg[None], axis=2))))
                t = u
            else:
                t += 1
        out[toe] = worst
    return out
