"""Approach mode of the door clearance gate (check_door_clearance.py --approach): V's body from the MOUNT, not from the clip start.

The clip gate places V at the clip start with the door timeline of the clip clock. In the game V stands somewhere beside the car, the workspot slides the
entity in a straight line onto the clip start (vanilla), UTPP's native root hold (VehicleTransition.hpp: cumulative / jump path) keeps the VISIBLE body at the
pre-mount spot for a while, and the door opens from the mount line. This module models that chain and gates the capsules of the body against the door:

  start spot      the 30 measured mount spots of the car-door2 game test (approach_fixtures.json, build_approach_fixtures.py) of the class + seat - the walks of the
                  harness presets (door r 0.5 / f 2.6 yaw 90, parallel r 2.6, behind r 1.9 / f 4.0) end at those spots, the fixture 'case' says which preset - plus a
                  grid of 0.1 m x 5 deg over the swept sector of the door and a band of 0.3 m / 15 deg around it (stand spots outside the closed door), facing the car
  vanilla slide   straight line start spot -> clip frame-0 root over T (class median of the measured slides: 0.30 s, Turbo-R 0.80 s), progress
                  sin(pi/2 t/T), z and yaw interpolated (shortest way); the clip pose is frame 0 during the slide (INFERRED from the workspot code, no pose
                  measurement)
  clock           alignment 'T': clip time = t - (T + 0.02 s): MEASURED - the entity vtpp series after the slide follows the clip root path with 1-3 cm rms at that
                  offset for 24 of 30 rides (36 cm rms at the mount-line offset 0.025 s of the clip gate). Alignment 'door': the clip clock starts with the door
                  (the rear-gap report's video mapping; the clip plays while the entity still slides) - contradicted by that fit, but run as well; worst case taken
  native hold     python port of VehicleTransition.hpp UpdateWorld / RootFactor / YawFactor (constants READ from the header, door_approach.K) fed with entity
                  samples at --fps: a step > kJumpDistance between two samples within kWorldWatchSeconds = jump path (offset decays over 0.30 s from the last jump
                  sample); otherwise the cumulative path engages when the entity has moved > kJumpDistance from the mount (visible root stays at the previous
                  sample until 0.45 s, planar offset released by smoothstep over 0.60 s, clamp 1.2 m, yaw align at 30 deg)
  hold cap        (--hold-cap on) the NATIVE hold cap (VehicleTransition.hpp HOLD CAP: SetHoldCap / PlanarFactor / ApplyJumpCap; ported 1:1, constants read from the header).
                  Input = what utpp_vehicle_door.reds UTPPDoorCapOf sends: the PRE-MOUNT spot (the entity at the mount line) against the door arc of
                  door_timing.json (read at run time): cap = max(t_edge - 0.05, 0) when t_edge <= 0.75 s, else no cap. The native then decides alone:
                  cumulative path - a gate (the hold carried the visible root >= kHoldCapGateDistM within kHoldCapGateWithinSeconds of the mount) latches the cap at the
                  first sample with gate and cap (ts <= kRootTrackSeconds); the release starts at max(latch ts, min(kRootTrackSeconds, cap - kHoldCapTailSeconds)) and lasts
                  clamp(cap - start, kHoldCapMinReleaseSeconds, kRootTailSeconds); planar factor = min(natural, release curve); the peak is tracked to kRootTrackSeconds
                  as without a cap, after it the yaw part is scaled by trackG (monotonic, as native). Jump path - at a jump sample (every one re-evaluates, the gates only
                  until the first application) the cap acts when the post-clamp offset >= kHoldCapJumpGateDistM and cap - ts <= kRootSeconds - kHoldCapJumpGainSeconds,
                  tail max(cap - ts, kHoldCapMinReleaseSeconds). No shortcut for small caps (the native clamps, it never skips). The cap is assumed to be in place at BeginMount
                  (arrival <= 0, UNVERIFIED against the game) and the standing gate of the cumulative path to pass.
  glide limit     (VehicleTransition.hpp GLIDE LIMIT, env UTPP_APPROACH_GLIDE=0 = the hold before it) a car mount WITHOUT a door cap: planar clamp kGlideMaxDistance, the jump path keeps
                  re-basing while the entity outruns kGlideFollowSpeedMps (the vanilla slide), the planar release lasts clamp(1.5 d / kGlideSpeedMps, base, kGlideEndSeconds - start).
                  A mount WITH a cap (the door script sent one) keeps the old hold. SPEED GATE (visible_speed): peak planar speed of the visible root at the native feed instants from
                  0.06 s on <= SPEED_MAX_MPS for every start without a cap; capped starts are reported and exempt (the door arrives on its own clock).
  door            vehicle door clip frame by time since the mount line: starts at the open delay of the class (door_timing.json measured range, swept
                  [lo - 0.03, nominal, hi + 0.03], worst case), opens over open_dur_s; per-phase gates of the clip gate (closed / opening 1 cm, open: max(2 cm,
                  seated baseline of the capsule))
  classes         per time step and capsule: the visible body (with the hold offset) over its gate while the entity-only body is not = 'hold' (HARD fail: the native
                  hold carries the body through the door); the entity-only body over its gate during the slide (t < T) with the door edge below 10 deg = 'slide'
                  (WARNING: vanilla behaviour UTPP cannot path-change; printed with its numbers, never waived); the entity-only body over its gate otherwise (the
                  clip start / the clip in the door's path) = 'entity' (HARD fail).

Everything in the vehicle frame (x right, y forward, z up, m, degrees CCW) except the capsule test, which runs in the door-data rig space (glTF: vehicle = M @ glTF).
"""
import json
import math
import os
import re
import sys

import numpy as np

import gen_door_timing as G

HERE = os.path.dirname(os.path.abspath(__file__))
HPP = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'src', 'mm', 'VehicleTransition.hpp'))
FIXTURES = os.path.join(HERE, 'approach_fixtures.json')
M = np.array(G.GLTF_TO_VEHICLE)
HZ = 120.0
DT = 1.0 / HZ
REL_TOL_CM = 0.05  # a candidate capsule is WORSE than the baseline when its excess is more than this above the baseline's excess of the same capsule
CLIP_GAP_S = 0.02  # clip time 0 = mount + T + this (measured, see the module docstring)
SLIDE_DOOR_DEG = 10.0
HARD = ('hold', 'entity', 'rear_hold', 'rear_entity')  # failure classes that fail the gate ('slide' is a warning)
WINDOW_S = 1.4
NEAR_M = 1.3  # no capsule test while the entity root and the visible root are farther than this from the sector the door has swept so far (reach of the body)
DEFAULT_FPS = 60.0
FPS_SET = (40.0, 50.0, 60.0, 90.0)  # native UpdateWorld sample rates swept (the jump / cumulative path of the same slide flips between ~45 and ~60 fps in the 30 rides)
SLIDE_T = {'sport1': 0.80}  # everything else 0.30 (approach_fixtures.json slide_T_by_class_s: 0.295..0.305, Turbo-R 0.82)

_NAMES = ('kRootSeconds', 'kRootMaxDistance', 'kRootMaxYawDeg', 'kWorldWatchSeconds', 'kJumpDistance', 'kJumpYawDeg', 'kRootTrackSeconds', 'kAlignYawDeg',
          'kAlignSeconds', 'kRootTailSeconds',
          # HOLD CAP
          'kHoldCapTailSeconds', 'kHoldCapMinReleaseSeconds', 'kHoldCapGateDistM', 'kHoldCapGateWithinSeconds', 'kHoldCapJumpGainSeconds', 'kHoldCapJumpGateDistM',
          # GLIDE LIMIT
          'kGlideSpeedMps', 'kGlideMaxDistance', 'kGlideFollowSeconds', 'kGlideFollowSpeedMps', 'kGlideEndSeconds')
_DEFAULTS = dict(kRootSeconds=0.30, kRootMaxDistance=1.2, kRootMaxYawDeg=120.0, kWorldWatchSeconds=0.15, kJumpDistance=0.15, kJumpYawDeg=15.0,
                 kRootTrackSeconds=0.45, kAlignYawDeg=30.0, kAlignSeconds=0.35, kRootTailSeconds=0.60,
                 kHoldCapTailSeconds=0.25, kHoldCapMinReleaseSeconds=0.15, kHoldCapGateDistM=0.25, kHoldCapGateWithinSeconds=0.13, kHoldCapJumpGainSeconds=0.10,
                 kHoldCapJumpGateDistM=0.60,
                 kGlideSpeedMps=2.0, kGlideMaxDistance=3.0, kGlideFollowSeconds=0.35, kGlideFollowSpeedMps=2.0, kGlideEndSeconds=1.50)  # = VehicleTransition.hpp (check_door_approach_defaults.py)


def read_consts(path=HPP):
    """The hold constants of VehicleTransition.hpp (read, not retyped). Returns (dict, source)."""
    try:
        txt = open(path, encoding='utf-8').read()
    except OSError:
        sys.stderr.write('door_approach: WARNING %s not found - using the built-in constants (check_door_approach_defaults.py compares them with the header)\n' % path)
        return dict(_DEFAULTS), 'defaults (header not found)'
    out = {}
    for n in _NAMES:
        m = re.search(r'constexpr float %s\s*=\s*([-0-9.eE+]+)f' % n, txt)
        if not m:
            raise RuntimeError('%s: constant %s not found - the model would silently use a stale default' % (path, n))
        out[n] = float(m.group(1))
    return out, path


K, K_SRC = read_consts()
GLIDE = os.environ.get('UTPP_APPROACH_GLIDE', '1') != '0'  # 0 = the hold before the glide limit (VehicleTransition.hpp GLIDE LIMIT), for before / after runs
SPEED_FROM_S = 0.06  # the speed gate starts here: before the first re-base / the cumulative engage the visible root IS the entity (<= 2 feed intervals, <= 0.2 m)
SPEED_MAX_MPS = 2.75  # visible planar root speed limit of a mount without a door cap (C++ test kGlideVisibleMax), unless it is the clip's own walk (<= SPEED_WALK x the entity's speed)
SPEED_WALK = 1.1
REDS = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'scripts', 'utpp_vehicle_door.reds'))
_REDS_DEFAULTS = dict(UTPPDoorCapBodyM=0.20, UTPPDoorCapMarginS=0.05, UTPPDoorCapHoldKeptS=1.5)


def read_reds_consts(path=REDS):
    """The three constants of UTPPDoorCapOf (body radius + margin, cap margin, assumed hold duration), read from the reds sender. Returns (dict, source)."""
    try:
        txt = open(path, encoding='utf-8').read()
    except OSError:
        return dict(_REDS_DEFAULTS), 'defaults (reds not found)'
    out = {}
    for n in _REDS_DEFAULTS:
        m = re.search(r'func %s\(\)\s*->\s*Float\s*\{\s*return\s*([-0-9.eE+]+)\s*;' % n, txt)
        if not m:
            raise RuntimeError('%s: constant %s not found' % (path, n))
        out[n] = float(m.group(1))
    return out, path


RC, RC_SRC = read_reds_consts()
BODY_M, CAP_MARGIN_S, HOLD_KEPT_S = RC['UTPPDoorCapBodyM'], RC['UTPPDoorCapMarginS'], RC['UTPPDoorCapHoldKeptS']


def ss(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def sss(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * x * (x * (6.0 * x - 15.0) + 10.0)


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


# ---------------------------------------------------------------- door arc geometry (vehicle frame)
def dir_of(deg):
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])


def seg_dist(p, a, b):
    ab = b - a
    t = min(max(float((p - a) @ ab) / float(ab @ ab), 0.0), 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def sector_dist(p, arc, alpha_deg):
    """Planar distance (m, 0 inside) of point p to the region the door sweeps from the closed direction to closed + alpha (signed deg)."""
    h = np.array([arc['hingeX'], arc['hingeY']])
    L = arc['length']
    c0 = arc['closedDirDeg']
    c1 = c0 + alpha_deg
    d = p - h
    r = float(np.linalg.norm(d))
    ang = math.degrees(math.atan2(d[1], d[0]))
    lo, hi = (c0, c1) if alpha_deg >= 0 else (c1, c0)
    rel = (ang - lo) % 360.0
    if rel <= (hi - lo):
        return max(0.0, r - L)
    return min(seg_dist(p, h, h + L * dir_of(c0)), seg_dist(p, h, h + L * dir_of(c1)))


def edge_deg(arc, t):
    """Door edge direction (deg) t seconds after the mount line (the UTPPCarDoorArc contract)."""
    return arc['closedDirDeg'] + arc['openDeg'] * ss((t - arc['openDelayS']) / arc['openDurS'])


def t_edge(arc, p, radius=BODY_M):
    """UTPPDoorCapOf's t_edge: time (s after the mount line) the door edge reaches the near side of the disk of `radius` around p, or None (reds why 2 / 3: p beyond
    the door radius, the disk is never swept). A disk that already overlaps the closed door's swing start is reached when the door starts to move.
    Mirrors utpp_vehicle_door.reds comparison for comparison (r > length + body; th + delta < 0 or th - delta >= openDeg)."""
    h = np.array([arc['hingeX'], arc['hingeY']])
    d = np.asarray(p, float)[:2] - h
    r = float(np.linalg.norm(d))
    if r > arc['length'] + radius:
        return None
    ang = math.degrees(math.atan2(d[1], d[0]))
    sgn = 1.0 if arc['openDeg'] > 0 else -1.0
    th = sgn * wrap(ang - arc['closedDirDeg'])  # signed angle from the closed direction, positive toward the open door
    delta = math.degrees(math.asin(min(1.0, radius / max(r, radius))))
    op = abs(arc['openDeg'])
    if th + delta < 0.0 or th - delta >= op:
        return None
    return arc['openDelayS'] + arc['openDurS'] * G.smoothstep_inv(min(max(max(th - delta, 0.0) / op, 0.0), 1.0))


def reds_cap(arc, p):
    """(cap seconds or None, t_edge or None): the cap utpp_vehicle_door.reds UTPPDoorCapOf sends for the pre-mount spot p (vehicle frame).
    cap = max(t_edge - 0.05, 0) when t_edge <= UTPPDoorCapHoldKeptS (1.5 s; the uncapped hold is assumed to still hold the root then), else none."""
    if arc is None:
        return None, None
    te = t_edge(arc, p)
    if te is None or te > HOLD_KEPT_S:
        return None, te
    return max(te - CAP_MARGIN_S, 0.0), te


# ---------------------------------------------------------------- entity path
class Path:
    """Entity root over time since the mount line (vehicle frame): vanilla slide, then the clip root path.
    t0 = clip time zero (mount + T + gap for alignment 'T', the door start for alignment 'door')."""

    def __init__(self, model, start_xyz, start_heading, T, t0, align):
        self.m = model
        self.T = T
        self.t0 = t0
        self.align = align
        self.S = M @ model.root_g[0]  # clip frame-0 root, vehicle frame
        fv = M @ model.fwd0_g
        self.psiS = math.degrees(math.atan2(fv[1], fv[0]))  # body heading at the clip start
        self.P0 = np.array(start_xyz, float)
        self.psi0 = float(start_heading)
        self.dpsi = wrap(self.psiS - self.psi0)
        self.root_v = model.root_g @ M.T
        loc = model.root_R[0].T @ model.fwd0_g  # forward axis in the root frame
        psi = []
        for i in range(len(model.root_g)):
            v = M @ (model.root_R[i] @ loc)
            psi.append(math.degrees(math.atan2(v[1], v[0])))
        psi = np.degrees(np.unwrap(np.radians(psi)))
        self.root_psi = psi - psi[0] + self.psiS

    def clip_root(self, tc):
        x = min(max(tc * 30.0, 0.0), len(self.root_v) - 1.0)
        i0 = int(x)
        i1 = min(i0 + 1, len(self.root_v) - 1)
        u = x - i0
        return self.root_v[i0] * (1 - u) + self.root_v[i1] * u, self.root_psi[i0] * (1 - u) + self.root_psi[i1] * u

    def slide(self, t):
        u = math.sin(math.pi / 2.0 * min(max(t / self.T, 0.0), 1.0))
        return self.P0 + u * (self.S - self.P0), self.psi0 + u * self.dpsi

    def entity(self, t):
        """(root xyz, heading deg) of the entity at t."""
        r, psi = self.clip_root(max(t - self.t0, 0.0))
        if t >= self.T:
            return r, psi
        e, ep = self.slide(t)
        if self.align == 'T':
            return e, ep  # the clip is on frame 0 during the slide
        dd = rz(ep - self.psiS) @ (r[:2] - self.S[:2])
        return np.array([e[0] + dd[0], e[1] + dd[1], e[2] + r[2] - self.S[2]]), ep + (psi - self.psiS)


# ---------------------------------------------------------------- native hold (port of VehicleTransition.hpp)
class Hold:
    """Visible-root offset of the native standing-entry hold, fed with entity samples at `fps` (UpdateWorld) and read at render time (RootFactor / PlanarFactor /
    YawFactor). 1:1 port incl. the HOLD CAP: `cap` = the seconds the door script sends (SetHoldCap: reds_cap), None = no cap (capHave false). The cap is assumed
    present from BeginMount on (arrival <= 0)."""

    def __init__(self, path, fps=DEFAULT_FPS, cap=None, align_car=True, glide=None):
        self.mode = 0
        self.fps = fps
        self.engage_at = -1.0
        self.peak_dist = 0.0
        self.anchor = None
        self.t_last_jump = None
        self.align_t = None
        self.cap = cap
        self.cap_state = 0  # g_capState: 0 no cap, 1 received but not needed (gate / gain), 2 applied on the cumulative path, 3 applied on the jump path
        self.cap_events = []  # (path, sample ts, release start, tail) of every application
        self.cap_gate_t = None
        kk = K
        cap_have = cap is not None
        glide = (GLIDE if glide is None else glide) and align_car and not cap_have  # GLIDE LIMIT: a car mount without a door cap (the native decides per sample, a later cap switches it off)
        max_dist = kk['kGlideMaxDistance'] if glide else kk['kRootMaxDistance']
        planar_s = 0.0
        yaw_ticks = None  # the jump path yaw decay clock after a slide-follow re-base (None = root_ticks)
        cap_on, cap_start, cap_tail = False, 0.0, 0.0
        cap_gate = cap_latched = cap_jump_applied = False
        cap_frozen = None  # cumulative cap: the world planar offset frozen at the release start (VehicleTransition capFrozen / capFx / capFy)
        jump_mag = 0.0
        track_g = 1.0
        e0, p0 = path.entity(0.0)
        begin, begin_psi = e0[:2].copy(), p0
        last, last_psi = e0[:2].copy(), p0
        root, root_ticks = (0.0, 0.0, 0.0), None
        vis, vis_psi, peak = None, 0.0, (np.zeros(2), 0.0)
        pub = []  # (t, root x char, root y char, root yaw, mode, rootTicks, capOn, capStart, capTail)
        dt = 1.0 / fps
        k = 1
        # review r2 finding 20: the glided planar tail of the cumulative path (planar_s up to kGlideEndSeconds - kRootTrackSeconds) keeps being re-published
        while k * dt <= kk['kRootTrackSeconds'] + max(kk['kRootTailSeconds'], planar_s if self.mode == 2 else 0.0) + 1e-9:
            ts = k * dt
            e, psi = path.entity(ts)
            pos = e[:2]
            step_speed = float(np.linalg.norm(pos - last)) / dt
            follow = glide and self.mode == 1 and not cap_jump_applied and ts <= kk['kGlideFollowSeconds'] and step_speed > kk['kGlideFollowSpeedMps']
            jump_sample = ts <= kk['kWorldWatchSeconds'] and self.mode != 2 and (float(np.linalg.norm(pos - last)) > kk['kJumpDistance'] or abs(wrap(psi - last_psi)) > kk['kJumpYawDeg'])
            follow_only = follow and not jump_sample  # the planar part only: the yaw keeps its own decay clock (Control::yawTicks)
            if jump_sample or follow_only:
                f = self._native_factor(self.mode, root, yaw_ticks if yaw_ticks is not None else root_ticks, ts)
                fp = self._planar(self._planar_natural(self.mode, root, root_ticks, ts, planar_s), cap_on, cap_start, cap_tail, ts)  # PlanarFactor of the old control; the yaw part keeps RootFactor
                vis_p = last + rz(last_psi) @ np.array(root[:2]) * fp
                vis_y = last_psi + root[2] * f
                ox, oy = rz(-psi) @ (vis_p - pos)
                oyaw = wrap(vis_y - psi)
                d = math.hypot(ox, oy)
                if d > max_dist:
                    ox, oy = ox * max_dist / d, oy * max_dist / d
                if follow_only:
                    if yaw_ticks is None:
                        yaw_ticks = root_ticks
                    root = (ox, oy, root[2])
                else:
                    root = (ox, oy, max(-kk['kRootMaxYawDeg'], min(kk['kRootMaxYawDeg'], oyaw)))
                    yaw_ticks = None
                root_ticks = ts
                self.mode = 1
                planar_s = self._glide_seconds(math.hypot(ox, oy), kk['kRootSeconds'], kk['kGlideEndSeconds'] - ts) if glide else 0.0
                self.t_last_jump = ts
                if self.anchor is None:
                    self.anchor = last.copy()
                # a new jump sample restarts the natural decay: the cap is evaluated again (capOn = 0, ApplyJumpCap)
                cap_on = False
                jump_mag = math.hypot(ox, oy)
                if cap_have and not follow_only:
                    rem = cap - ts
                    ok = True
                    if not cap_jump_applied:  # the gates decide once per mount: until the first application every jump sample asks again
                        if jump_mag < kk['kHoldCapJumpGateDistM'] or rem > kk['kRootSeconds'] - kk['kHoldCapJumpGainSeconds']:
                            self.cap_state = 1
                            ok = False
                    if ok:
                        cap_jump_applied = True
                        cap_on, cap_start = True, ts
                        cap_tail = kk['kHoldCapMinReleaseSeconds'] if rem < kk['kHoldCapMinReleaseSeconds'] else rem
                        self.cap_state = 3
                        self.cap_events.append(('jump', ts, ts, cap_tail))
                pub.append((ts,) + root + (1, ts, cap_on, cap_start, cap_tail, planar_s, yaw_ticks))
            if self.mode != 1:
                track = self.mode == 2
                if not track and ts <= kk['kRootTrackSeconds']:
                    if float(np.linalg.norm(pos - begin)) > kk['kJumpDistance'] or abs(wrap(psi - begin_psi)) > kk['kJumpYawDeg']:
                        vis, vis_psi = last.copy(), last_psi
                        self.mode = 2
                        track = True
                        track_g = 1.0
                        self.engage_at = ts
                        self.anchor = vis.copy()
                if track:
                    w, oyaw = vis - pos, wrap(vis_psi - psi)
                    if align_car and self.align_t is None and ts <= kk['kRootTrackSeconds'] and abs(oyaw) > kk['kAlignYawDeg']:
                        self.align_t = ts
                    if ts <= kk['kRootTrackSeconds']:
                        self.peak_dist = max(self.peak_dist, math.hypot(w[0], w[1]))  # the diagnostics and the gate read the live hold
                        if ts <= kk['kHoldCapGateWithinSeconds'] and math.hypot(w[0], w[1]) >= kk['kHoldCapGateDistM']:
                            cap_gate = True
                            if self.cap_gate_t is None:
                                self.cap_gate_t = ts
                        if cap_have and cap_gate and not cap_latched:
                            cap_latched = True
                            if cap >= kk['kRootTrackSeconds'] + kk['kRootTailSeconds']:
                                self.cap_state = 1  # the natural release ends before the cap: nothing to do
                            else:
                                hold = cap - kk['kHoldCapTailSeconds']
                                start = max(ts, min(kk['kRootTrackSeconds'], hold))
                                tail = cap - start
                                tail = kk['kHoldCapMinReleaseSeconds'] if tail < kk['kHoldCapMinReleaseSeconds'] else (kk['kRootTailSeconds'] if tail > kk['kRootTailSeconds'] else tail)
                                cap_on, cap_start, cap_tail = True, start, tail
                                self.cap_state = 2
                                self.cap_events.append(('cum', ts, start, tail))
                        # the release has started (this sample or an earlier one): from its first sample the planar offset is frozen in the world
                        if cap_on:
                            if cap_frozen is None and ts >= cap_start - 1e-6:
                                cap_frozen = w.copy()
                            if cap_frozen is not None:
                                w = cap_frozen.copy()
                        peak = (w.copy(), oyaw)
                    else:
                        # after the window: the planar offset stays at the peak (decays by time only), the yaw offset follows the clip's root-yaw progress (monotonic)
                        gp = 1.0
                        if abs(peak[1]) > 1.0:
                            gp = min(max(oyaw / peak[1], 0.0), 1.0)
                        track_g = min(track_g, gp)
                        w, oyaw = peak[0], peak[1] * track_g
                    ox, oy = rz(-psi) @ w
                    d = math.hypot(ox, oy)
                    if d > max_dist:
                        ox, oy = ox * max_dist / d, oy * max_dist / d
                    if ts <= kk['kRootTrackSeconds'] and glide:  # the planar tail of the natural release is fixed by the offset held at the end of the tracking window
                        planar_s = self._glide_seconds(math.hypot(ox, oy), kk['kRootTailSeconds'], kk['kGlideEndSeconds'] - kk['kRootTrackSeconds'])
                    root = (ox, oy, max(-kk['kRootMaxYawDeg'], min(kk['kRootMaxYawDeg'], oyaw)))
                    root_ticks = ts
                    pub.append((ts,) + root + (2, ts, cap_on, cap_start, cap_tail, planar_s, None))
            last, last_psi = pos.copy(), psi
            k += 1
        self.pub = pub
        self.pub_t = np.array([p[0] for p in pub]) if pub else np.zeros(0)

    @staticmethod
    def _glide_seconds(d, base, budget):
        """VehicleTransition.hpp detail::GlideSeconds: release duration of a held planar offset d (peak closing speed 1.5 d / D = kGlideSpeedMps), never below base, never past the budget."""
        x = 1.5 * d / K['kGlideSpeedMps']
        cap = budget if budget > base else base
        x = cap if x > cap else x
        return base if x < base else x

    @staticmethod
    def _planar_natural(mode, root, ticks, t, planar_s):
        """PlanarNatural: the natural factor of the planar part over the per-mount duration (GLIDE LIMIT); RootFactor without one."""
        if planar_s <= 0.0 or ticks is None or (root[0] == 0 and root[1] == 0 and root[2] == 0):
            return Hold._native_factor(mode, root, ticks, t)
        if mode == 2:
            return 1.0 if t <= K['kRootTrackSeconds'] else 1.0 - ss((t - K['kRootTrackSeconds']) / planar_s)
        return 1.0 - ss((t - ticks) / planar_s)

    @staticmethod
    def _native_factor(mode, root, ticks, t):
        """RootFactor: the natural factor."""
        if ticks is None or (root[0] == 0 and root[1] == 0 and root[2] == 0):
            return 0.0
        if mode == 2:
            return 1.0 if t <= K['kRootTrackSeconds'] else 1.0 - ss((t - K['kRootTrackSeconds']) / K['kRootTailSeconds'])
        return 1.0 - ss((t - ticks) / K['kRootSeconds'])

    @staticmethod
    def _planar(f, cap_on, cap_start, cap_tail, t):
        """PlanarFactor: min(natural factor, release curve of the cap: 1 until its start, then 1 - smoothstep over its tail)."""
        if not cap_on or not cap_tail > 0.0:
            return f
        u = t - cap_start
        curve = 1.0 if u <= 0.0 else 1.0 - ss(u / cap_tail)
        return curve if curve < f else f

    def cap_sig(self):
        """What the cap did (state, applications), rounded: part of the dedupe key of the sample-rate sweep."""
        return (self.cap_state,) + tuple((c[0], round(c[2], 3), round(c[3], 3)) for c in self.cap_events)

    def offset(self, t, ent_psi):
        """(dx, dy world m, dyaw deg) of the visible root against the entity at t."""
        if not len(self.pub_t):
            return 0.0, 0.0, 0.0
        i = int(np.searchsorted(self.pub_t, t + 1e-9, side='right')) - 1
        if i < 0:
            return 0.0, 0.0, 0.0
        _, ox, oy, oyaw, mode, ticks, cap_on, cap_start, cap_tail, planar_s, yticks = self.pub[i]
        yf = self._native_factor(mode, (1, 1, 1), ticks if yticks is None else yticks, t)
        pf = self._planar(self._planar_natural(mode, (1, 1, 1), ticks, t, planar_s), cap_on, cap_start, cap_tail, t)
        if mode == 2 and self.align_t is not None:
            yf *= 1.0 - sss((t - self.align_t) / K['kAlignSeconds'])
        w = rz(ent_psi) @ np.array([ox, oy]) * pf
        return float(w[0]), float(w[1]), oyaw * yf


def visible_speed(path, hold, fps, t_from=SPEED_FROM_S, t_end=1.6):
    """Planar world speed (m/s) of the VISIBLE root (entity + native hold offset) between consecutive native feed instants k / fps while the hold runs (visible root
    farther than 1 cm from the entity). The published offset is constant between two feeds, so a finer sampling would add the entity's own slide to every in-between
    frame and measure the raw slide, not the hold (VehicleTransition test GlideStats). 'peak' is taken from t_from on, 'early' before it (the pre-engage frames:
    the visible root IS the entity until the first re-base / the cumulative engage, <= 0.2 m), 'closing' is the fastest shrink of |visible - entity| over >= 0.1 s
    (the release speed), 'end' the last time the offset is > 1 cm, 'peak_off' its largest value."""
    dt = 1.0 / fps
    ts, vis, off, ents = [], [], [], []
    k = 0
    while k * dt <= t_end + 1e-9:
        t = k * dt
        e, psi = path.entity(t)
        dx, dy, _ = hold.offset(t, psi)
        ts.append(t)
        ents.append(np.array([e[0], e[1]]))
        vis.append(np.array([e[0] + dx, e[1] + dy]))
        off.append(math.hypot(dx, dy))
        k += 1
    end = max([t for t, o in zip(ts, off) if o > 0.01], default=0.0)
    out = dict(peak=0.0, at=0.0, ent=0.0, early=0.0, closing=0.0, end=round(end, 3), peak_off=round(max(off), 3) if off else 0.0)
    for i in range(1, len(ts)):
        if ts[i - 1] > end:
            break
        v = float(np.linalg.norm(vis[i] - vis[i - 1])) / dt
        if ts[i] < t_from - 1e-9:
            out['early'] = max(out['early'], v)
        elif v > out['peak']:
            out['peak'], out['at'] = v, ts[i]
            out['ent'] = float(np.linalg.norm(ents[i] - ents[i - 1])) / dt
        for j in range(i + 1, len(ts)):
            if ts[j] - ts[i - 1] >= 0.1 - 1e-9:
                out['closing'] = max(out['closing'], (off[i - 1] - off[j]) / (ts[j] - ts[i - 1]))
                break
    return {k_: round(float(v), 3) for k_, v in out.items()}


def make_hold(path, fps, arc=None, cap_on=False):
    """The uncapped hold, the cap the door script sends for the pre-mount spot (reds_cap) and the hold with that cap (the native decides alone whether it acts).
    Returns dict(hold = the hold to gate (capped when cap_on), uncapped, capped, trig = the native APPLIED the cap (state 2 / 3), tEnd = the cap seconds sent
    (None = none), t_edge, anchor, mode, cap_state)."""
    h0 = Hold(path, fps)
    out = dict(hold=h0, uncapped=h0, capped=h0, trig=False, tEnd=None, t_edge=None, anchor=h0.anchor, mode=h0.mode, cap_state=0, fps=fps)
    if arc is None:
        return out
    capS, te = reds_cap(arc, path.entity(0.0)[0][:2])  # the pre-mount spot: V where the entity stands at the mount line
    out['t_edge'] = te
    out['tEnd'] = capS
    if capS is None:
        return out
    h1 = Hold(path, fps, cap=capS)
    out['capped'] = h1
    out['trig'] = h1.cap_state in (2, 3)
    out['cap_state'] = h1.cap_state
    if cap_on:
        out['hold'] = h1
    return out


# ---------------------------------------------------------------- starts
def load_fixtures(path=FIXTURES):
    return json.load(open(path, encoding='utf-8'))


def facing_car(seat):
    return 0.0 if seat == 'driver' else 180.0


def fixture_starts(fx, cls, seat):
    out = []
    for r in fx['rides']:
        if r['cls'] == cls and r['seat'] == seat:
            h = r['mount_heading_deg']
            out.append(dict(name='ride%d_%s' % (r['ride'], r['case']), ride=r['ride'], kind='fixture', xyz=r['mount_spot'], heading=facing_car(seat) if h is None else h,
                            heading_src='POS' if h is not None else 'facing-car default', review=r['review'], logged=dict(mode=r['vt_mode'], engage=r['rootEngageAt'],
                                                                                                                        peak=r['rootPeakDist_log'])))
    return out


def grid_starts(arc, seat, z, hinge_clear=0.25, step_r=0.1, step_deg=5.0, band_m=0.3, band_deg=15.0):
    """Stand spots over the swept sector of the door and a band around it, outside the closed door, facing the car."""
    if arc is None:
        return []
    h = np.array([arc['hingeX'], arc['hingeY']])
    sgn = 1.0 if arc['openDeg'] > 0 else -1.0
    out = []
    for r in np.arange(0.3, arc['length'] + band_m + 1e-9, step_r):
        for a in np.arange(-band_deg, abs(arc['openDeg']) + band_deg + 1e-9, step_deg):
            p = h + r * dir_of(arc['closedDirDeg'] + sgn * a)
            if abs(p[0]) < abs(arc['hingeX']) + hinge_clear or abs(p[0]) > 3.5:
                continue
            if (p[0] > 0) != (arc['hingeX'] > 0):
                continue
            out.append(dict(name='grid_r%.1f_a%+.0f' % (r, a), kind='grid', xyz=[round(float(p[0]), 3), round(float(p[1]), 3), z], heading=facing_car(seat), heading_src='facing-car'))
    return out


def arrival_starts(spot_xy, spot_yaw_deg, z, radii=(0.06, 0.12, 0.15), n_dirs=8):
    """r13 ARRIVAL scenario: V ends the native approach walk within the arrival gate of the entry clip's start spot (ApproachCore.hpp arriveGateM 0.10 + the facing carry; the
    logged residuals of the r6a game test were 0.06-0.15 m) and the mount runs from there. The approach_fixtures.json rides are the harness presets of the game test that PRE-DATES the
    walk (the vanilla slide from 0.8-2.8 m = the fallback after an abort / timeout / no_motion); this ring is the primary case of the shipped build. heading = -yaw (the harness convention)."""
    out = []
    for r in radii:
        for k in range(n_dirs if r > radii[0] else n_dirs // 2):
            a = 2.0 * math.pi * k / (n_dirs if r > radii[0] else n_dirs // 2)
            out.append(dict(name='arrival_r%.2f_d%03d' % (r, round(math.degrees(a))), kind='arrival', xyz=[round(float(spot_xy[0] + r * math.cos(a)), 3), round(float(spot_xy[1] + r * math.sin(a)), 3), z],
                            heading=-float(spot_yaw_deg), heading_src='clip start yaw'))
    return out


# ---------------------------------------------------------------- evaluation
class Approach:
    """One clip (check_door_clearance.Model) against the door of its seat from the mount."""

    def __init__(self, model, tj, fx=None, tol_cm=1.0, open_tol_cm=2.0, opening_tol_cm=None, fps=DEFAULT_FPS):
        self.m = model
        self.tj = tj
        self.fx = fx or load_fixtures()
        self.cls, self.seat, self.side = model.cls, model.seat, model.side
        self.VV = model.V @ M.T
        self.arc = G.arc_of(tj, model.cls, model.side)
        self.T = SLIDE_T.get(model.cls, 0.30)
        self.fps = fps
        self.od = float(model.tm['open_dur_s'])
        c = tj['classes'][model.cls]
        lo, hi = c.get('measured_open_start_range_s', [model.tm['open_start_s']] * 2)
        self.delays = sorted(set([round(max(0.0, lo - 0.03), 4), round(float(model.tm['open_start_s']), 4), round(hi + 0.03, 4)]))
        nc = len(model.names)
        self.gate = {'closed': np.full(nc, float(tol_cm)), 'opening': np.full(nc, float(tol_cm if opening_tol_cm is None else opening_tol_cm)),
                     'open': model.gate_cm('open', tol_cm, open_tol_cm)}
        if model.rear is not None:
            self.gate['rear'] = model.gate_cm('rear', tol_cm, open_tol_cm)
        self.nT = len(model.t)

    def door_state(self, t, delay):
        s = t - delay
        if s < 0:
            return 'closed', 0.0
        if s < self.od:
            return ('opening' if s < self.m.s90_frac * self.od else 'open'), self.m.su * s / self.od
        return 'open', float(self.m.su)

    def door_deg(self, t, delay):
        if self.arc is not None:
            return abs(self.arc['openDeg']) * ss((t - delay) / self.arc['openDurS'])
        return 70.0 * ss((t - delay) / self.od)

    def clip_points(self, tc):
        x = min(max(tc * 30.0, 0.0), self.nT - 1.0)
        i0 = int(x)
        i1 = min(i0 + 1, self.nT - 1)
        u = x - i0
        return self.VV[i0] * (1 - u) + self.VV[i1] * u

    def place(self, path, t):
        """Entity-aligned body points (vehicle frame) at t: the clip pose, rigidly moved by the slide while the slide runs."""
        p = self.clip_points(max(t - path.t0, 0.0))
        if t >= path.T:
            return p
        es, eps = path.slide(t)
        q = p.copy()
        d = (rz(eps - path.psiS) @ (p[:, :2] - path.S[:2]).T).T
        q[:, 0], q[:, 1] = es[0] + d[:, 0], es[1] + d[:, 1]
        q[:, 2] = es[2] + p[:, 2] - path.S[2]
        return q

    def run(self, path, mh, delay, window=WINDOW_S, keep_series=False):
        """One configuration: entity path + hold (make_hold result) + open delay. Returns dict(classes: {'hold'|'slide'|'entity': worst record}, hold info)."""
        m = self.m
        hold = mh['hold']
        speed = dict(visible_speed(path, hold, mh['fps']), capSent=hold.cap is not None, glide=GLIDE)  # GLIDE LIMIT gate: a mount with a door cap is exempt (the door arrives first)
        wc = {}  # (kind, capsule) -> worst record (the baseline-relative verdict compares capsule by capsule)
        series = []

        def note(kind, c, ex, pen, t, ph, bone, ang):
            key = (kind, m.names[c])
            w = wc.get(key)
            if w is None or ex > w['_ex']:
                wc[key] = dict(_ex=float(ex), excess_cm=round(float(ex), 2), pen_cm=round(float(pen * 100), 2), t=round(t, 4), phase=ph, capsule=m.names[c], door_bone=bone,
                               door_deg=round(ang, 1))

        for i in range(int(window * HZ) + 1):
            t = i * DT
            e, ep = path.entity(t)
            q = self.place(path, t)
            ph, f = self.door_state(t, delay)
            dx, dy, dyaw = hold.offset(t, ep)
            qv = q
            if abs(dx) + abs(dy) + abs(dyaw) > 1e-6:
                qv = q.copy()
                d = (rz(dyaw) @ (q[:, :2] - e[:2]).T).T
                qv[:, 0], qv[:, 1] = e[0] + d[:, 0] + dx, e[1] + d[:, 1] + dy
            ang = self.door_deg(t, delay)
            far = False
            if self.arc is not None:
                alpha = self.arc['openDeg'] * ss((t - delay) / self.arc['openDurS'])
                far = min(sector_dist(e[:2], self.arc, alpha), sector_dist(e[:2] + np.array([dx, dy]), self.arc, alpha)) > NEAR_M
            if m.rear is not None:  # the static rear door / glass / B-pillar, every time step
                rE, whoE = m.rear_pen_caps_V(q @ M)
                rV, whoV = (rE, whoE) if qv is q else m.rear_pen_caps_V(qv @ M)
                gR = self.gate['rear']
                for c in np.flatnonzero((rE * 100 - gR > 0) | (rV * 100 - gR > 0)):
                    if rE[c] * 100 - gR[c] > 0:
                        kind, ex, pen, who = 'rear_entity', rE[c] * 100 - gR[c], rE[c], whoE[c]
                    else:
                        kind, ex, pen, who = 'rear_hold', rV[c] * 100 - gR[c], rV[c], whoV[c]
                    note(kind, c, ex, pen, t, 'rear', who, ang)
            if far:
                if keep_series:
                    series.append((round(t, 4), ph, -99.0, -99.0, round(dx, 3), round(dy, 3)))
                continue
            penE, bones = m.pen_caps_V(q @ M, f)
            penV, bonesV = (penE, bones) if qv is q else m.pen_caps_V(qv @ M, f)
            g = self.gate[ph]
            exE = penE * 100 - g
            exV = penV * 100 - g
            if keep_series:
                series.append((round(t, 4), ph, round(float(np.max(exV)), 2), round(float(np.max(exE)), 2), round(dx, 3), round(dy, 3)))
            for c in np.flatnonzero((exE > 0) | (exV > 0)):
                if exE[c] > 0:
                    kind = 'slide' if (t < path.T and ang < SLIDE_DOOR_DEG) else 'entity'
                    ex, pen, bone = exE[c], penE[c], bones[c]
                else:
                    kind, ex, pen, bone = 'hold', exV[c], penV[c], bonesV[c]
                note(kind, c, ex, pen, t, ph, bone, ang)
        by_cap = {}
        for (kind, cn), w in wc.items():
            by_cap.setdefault(kind, {})[cn] = {k: v for k, v in w.items() if k != '_ex'}
        worst = {kind: max(d.values(), key=lambda w: w['excess_cm']) for kind, d in by_cap.items()}
        cp = mh['capped']
        out = dict(classes=worst, classes_cap=by_cap, trig=mh['trig'], tEnd=mh['tEnd'], t_edge=mh['t_edge'], cap_state=cp.cap_state, cap_events=cp.cap_events, anchor=None if mh['anchor'] is None else [round(float(v), 3) for v in mh['anchor']],
                   mode={0: 'none', 1: 'jump', 2: 'hold'}[mh['mode']], engage=round(mh['uncapped'].engage_at, 3), peak=round(mh['uncapped'].peak_dist, 3), speed=speed)
        if keep_series:
            out['series'] = series
        return out

    def sweep(self, start, cap_on, aligns=('T', 'door'), delays=None, fps_set=FPS_SET, **kw):
        """Worst case over the alignments, the open delays and the native sample rates. A start with a logged native path (fixture: vt_mode) runs only the sample
        rates that reproduce that path (the rate jitters between ~45 and ~60 fps in the 30 rides, no single rate reproduces all labels); grid starts run every
        distinct hold of the rate set."""
        want = None
        if start.get('logged') and start.get('use_label'):  # calibration of the SHIPPED clips only: a different clip has a different slide, its path is not the logged one
            lm = str(start['logged']['mode'])
            # 'hold' -> cumulative path, 'jump' / 'jump(est)' -> jump path. A logged 'none' (no cumulative engage logged) says nothing about the jump path (the C++ replay at
            # 50 Hz puts rides 14 / 16 / 23 / 26 / 27 on the jump path): it is NOT forced onto either path, the full sample-rate set runs
            want = 2 if lm == 'hold' else (None if lm == 'none' else 1)
        res = []
        for a in aligns:
            t0 = lambda d: (self.T + CLIP_GAP_S) if a == 'T' else d
            for d in (delays or self.delays):
                path = Path(self.m, start['xyz'], start['heading'], self.T, t0(d), a)
                holds = [(fps, make_hold(path, fps, self.arc, cap_on)) for fps in fps_set]
                if want is not None and any(h['mode'] == want for _, h in holds):
                    holds = [(f, h) for f, h in holds if h['mode'] == want]
                seen = {}
                for fps, mh in holds:
                    key = (mh['mode'], round(mh['uncapped'].engage_at, 2), round(mh['uncapped'].peak_dist, 2), mh['uncapped'].t_last_jump and round(mh['uncapped'].t_last_jump, 3),
                           None if mh['tEnd'] is None or not cap_on else round(mh['tEnd'], 3), mh['hold'].cap_sig())
                    if key in seen:
                        seen[key]['fps'].append(fps)
                        continue
                    r = self.run(path, mh, d, **kw)
                    r['align'], r['delay'], r['fps'] = a, d, [fps]
                    seen[key] = r
                    res.append(r)
        worst, worst_cap = {}, {}
        for r in res:
            for k, d in r['classes_cap'].items():
                for cn, w in d.items():
                    if cn not in worst_cap.setdefault(k, {}) or w['excess_cm'] > worst_cap[k][cn]['excess_cm']:
                        worst_cap[k][cn] = dict(w, align=r['align'], delay=r['delay'], fps=r['fps'], hold_mode=r['mode'])
        for k, d in worst_cap.items():
            worst[k] = max(d.values(), key=lambda w: w['excess_cm'])
        # the speed gate reads the MEASURED clock alignment 'T' (the 'door' alignment adds the clip's walk to the slide: a stress variant) when it ran
        sp = max((dict(r['speed'], align=r['align'], delay=r['delay'], fps=r['fps']) for r in (res if not any(r['align'] == 'T' for r in res) else [r for r in res if r['align'] == 'T'])),
                 key=lambda s: s['peak'], default=None)
        return dict(worst=worst, worst_cap=worst_cap, trig=any(r['trig'] for r in res), modes=sorted(set(r['mode'] for r in res)), runs=res, speed=sp)

    @staticmethod
    def verdict(sweep):
        hard = [k for k in HARD if k in sweep['worst']]
        return ('FAIL' if hard else 'PASS'), hard, ('slide' in sweep['worst'])


def relative_worse(cand, base, tol_cm=REL_TOL_CM):
    """Baseline-relative verdict, capsule by capsule. cand / base = {class: {capsule: record with excess_cm}} (sweep()['worst_cap']). Returns
    {class: {capsule: (excess_cm, baseline excess_cm or None)}} for the HARD classes where a capsule's excess is more than tol_cm above the excess the baseline has for
    the SAME capsule (a capsule absent from the baseline counts as -inf: every excess of it is worse). Empty dict = not worse than the baseline."""
    out = {}
    for k, d in cand.items():
        if k not in HARD:
            continue
        for cn, w in d.items():
            b = base.get(k, {}).get(cn)
            be = None if b is None else b['excess_cm']
            if be is None or w['excess_cm'] > be + tol_cm:
                out.setdefault(k, {})[cn] = (w['excess_cm'], be)
    return out
