"""Build a player vehicle-entry clip from an NPC clip (optionally mirrored) and write a WolvenKit-importable GLB.

Pipeline (all numpy, deterministic; Blender is used only for previews):
  1. sample the source clip (WolvenKit export with root motion, Root bone = root motion) at 30 fps;
  2. optional mirror about the character's sagittal plane at the start (glTF X -> -X): world-space mirror of every
     bone from its left/right counterpart with the rest-pose axis correction (M W_mb M)(M R_mb M)^T R_b;
  3. retarget to the player rig: local rotations copied by bone name (the engine applies clips by name to both rigs),
     local translations = player rest + (source local - source rest); Root copied (root motion);
  4. player-only helper bones (IK targets, Torso_* controls, weapon/arms spaces) re-attached to the best anchor bone
     found in the vanilla player clip (relative transform with minimal variance, time-aligned);
  5. root-motion end correction to the vanilla player clip end (position + yaw), smoothstep weight over a window;
  6. optional end-pose blend of the common bones towards the vanilla clip's last frame (continuity into the seated idle);
  7. float tracks and events: taken from the vanilla player clip (player rig track set), FootPlant events re-timed
     from the measured foot contacts of the new clip.

Usage (python -I):
  build_entry_clip.py --src <npc.anims.glb> --src-clip enter --target <pwa.anims.glb> --target-clip enter
                      --out <out.anims.glb> --name enter [--mirror] [--corr-start 0.9] [--end-blend 0.35]
                      [--report <json>]
"""
import argparse
import copy
import hashlib
import json
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402
import skel as S  # noqa: E402
import ik as IK  # noqa: E402

FPS = 30.0
M = np.diag([-1.0, 1.0, 1.0])  # glTF X is the character's lateral axis (CP x)
UP = np.array([0.0, 1.0, 0.0])


def frames_of(sk, clip):
    ch = G.channels(sk.gj, sk.binb, clip)
    dur = max(float(tr[0][-1]) for d in ch.values() for tr in d.values())
    n = int(round(dur * FPS)) + 1
    return dur, np.arange(n) / FPS


def mirror_pose(sk, lt, lr):
    Wt, Wr = sk.world(lt, lr)
    rt, rr = sk.rest_world()
    T, N = lt.shape[:2]
    Wt2 = np.zeros_like(Wt)
    Wr2 = np.zeros_like(Wr)
    for b, name in enumerate(sk.names):
        mn = S.mirror_name(name)
        mb = sk.index.get(mn, b)
        corr = (M @ rr[mb] @ M).T @ rr[b]
        Wr2[:, b] = np.einsum('ij,tjk,kl->til', M, Wr[:, mb], M) @ corr
        Wt2[:, b] = Wt[:, mb] @ M.T
    return sk.local_from_world(Wt2, Wr2)


def yaw_of(q):
    """Rotation about glTF Y (up) in degrees for (x,y,z,w) quaternions (twist part)."""
    return np.degrees(2 * np.arctan2(q[..., 1], q[..., 3]))


def qyaw(deg):
    h = np.radians(deg) / 2
    return np.stack([np.zeros_like(h), np.sin(h), np.zeros_like(h), np.cos(h)], axis=-1)


ROOF_GRID_M = 0.04
GLTF_TO_VEHICLE = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])


def roof_cap(args, W1, nlt, nlr, tgt, hi, ri):
    """GEOMETRIC head cap (--roof-json): the ceiling of the car (body-data of extract_door_data.py --body: roof shell, roof lining above the seat, glass) in the VEHICLE frame at the
    head's (x, y), minus the head top (--roof-top above the Head joint = the head sphere + hair of check_door_clearance's roof phase) minus --roof-margin, as a world height of the clip
    (T,) (9.0 = no ceiling there). Never lower than the seated end pose's head minus --roof-floor-below-seated.
    Vehicle frame = seat * inverse(last-frame root) as in check_door_clearance.Model; the ceiling is the lowest roof point within 0.16 m of the head (conservative at rails / pillars).
    --roof-relax-seated: the roof model is not exact (the vanilla seated head is up to 13 cm inside the Turbo-R lining), so the head sphere of the cap is shrunk by the radial
    penetration of the SEATED end pose itself - the clip may not penetrate the ceiling more than the seated pose does (the same baseline as the gate of the roof phase)."""
    j = json.load(open(args.roof_json, encoding='utf-8'))
    dd = json.load(open(str(Path(args.roof_json).parent.parent / 'door-data' / (j['class'] + '.json')), encoding='utf-8'))
    sR = np.array(dd['seat'][args.roof_seat]['R']).reshape(3, 3)
    st = np.array(dd['seat'][args.roof_seat]['t'])
    seat_z = min(v[2] for v in j['seat'].values())
    P = np.concatenate([np.asarray(j['parts'][k]['points'], float).reshape(-1, 3) for k in ('shell', 'glass', 'lining')])
    P = P[P[:, 2] >= seat_z + 0.45]
    Re, pe = S.q2m(nlr[-1, ri]), nlt[-1, ri]
    A = sR @ Re.T
    up_m = float(args.roof_top) - 0.15  # centre of the head sphere above the Head joint (0.09)

    def cap_of(hv, R):  # (n, 3) Head joint positions in the vehicle frame, sphere radius R -> vehicle-frame z cap of the Head joint (9 = no ceiling)
        out = np.full(len(hv), 9.0)
        for i in range(len(hv)):
            c = hv[i] + np.array([0.0, 0.0, up_m])  # sphere centre (vehicle frame; the skull centre is above the Head joint along world up)
            dxy = np.linalg.norm(P[:, :2] - c[:2], axis=1)
            k = (dxy < R) & (P[:, 2] > c[2] - 0.02)
            if k.any():
                zc = float((P[k, 2] - np.sqrt(np.maximum(R * R - dxy[k] ** 2, 0.0))).min())  # highest centre height that keeps the sphere under every ceiling point
                out[i] = zc - up_m
        return out

    # the seated end pose placed on the final root: floor of the cap and (--roof-relax-seated) the radial penetration of the seated head itself
    ep, ec = args.end_pose.rsplit(':', 1)
    e_last = ec.endswith('@last')
    ec = ec[:-5] if e_last else ec
    esk = S.Skeleton.from_glb(ep)
    ecl = esk.clip(ec)
    _, etimes = frames_of(esk, ecl)
    elt, elr = esk.sample_local(ecl, etimes[-1:] if e_last else etimes[:1])
    elt[0, ri], elr[0, ri] = nlt[-1, ri], nlr[-1, ri]
    We, _ = tgt.world(elt, elr)
    floor_y = float(We[0, hi, 1]) - float(args.roof_floor_below_seated)  # entering needs the head BELOW the seated height under the door header / roof rail (vanilla does too); check (n) bounds the duck anatomically
    relax = 0.0
    if getattr(args, 'roof_relax_seated', False) or getattr(args, 'roof_gate_mode', False):
        hs = (np.einsum('ij,tj->ti', A, We[:, hi] - pe) + st) @ GLTF_TO_VEHICLE.T
        cs_ = hs[0] + np.array([0.0, 0.0, up_m])
        relax = max(0.0, 0.15 - float(np.linalg.norm(P - cs_, axis=1).min()))  # radial penetration of the seated head sphere (same model as the roof phase)
    if getattr(args, 'roof_gate_mode', False):
        # target = the gate of the roof phase itself (check_door_clearance: penetration <= max(1 cm, seated head penetration + 0.5 cm)) minus roof_margin of slack
        R = max(0.01, 0.15 - max(0.01, relax + 0.005) + float(args.roof_margin))
    else:
        R = max(0.02, 0.15 + float(args.roof_margin) - relax)  # skull + hair + margin, minus what the seated pose itself penetrates (roof model inexact at the seat)
    def world_cap(Hw, step=0.01):
        """EXACT cap of the finished pose: for every Head joint position (n, 3) (world) the highest height NOT above it at which the head sphere (radius R, centre up_m above the joint)
        no longer meets any roof / pillar / glass point of the cloud P (the same nearest-point model as the roof phase of check_door_clearance.py; the one-sided cap_of() above ignores
        points beside or below the sphere centre, which a lowered head moves into). Never below floor_y; 9 where the head is not near the roof at all."""
        hv_ = (np.einsum('ij,tj->ti', A, Hw - pe) + st) @ GLTF_TO_VEHICLE.T
        out = np.full(len(Hw), 9.0)
        for i in range(len(Hw)):
            c0 = hv_[i] + np.array([0.0, 0.0, up_m])
            near = P[np.sum((P - c0) ** 2, axis=1) < (R + 0.5) ** 2]
            if not len(near):
                continue
            dzmax = max(0.0, float(Hw[i, 1]) - floor_y)
            hit = None
            for dz in np.arange(0.0, dzmax + 1e-9, step):
                c = c0 - np.array([0.0, 0.0, dz])
                if float(np.sqrt(np.min(np.sum((near - c) ** 2, axis=1)))) >= R:
                    hit = dz
                    break
            out[i] = float(Hw[i, 1]) - (dzmax if hit is None else hit)
        return out

    hv = (np.einsum('ij,tj->ti', A, W1[:, hi] - pe) + st) @ GLTF_TO_VEHICLE.T
    cap_v = cap_of(hv, R)
    kf = int(args.roof_lookahead)  # the head of the finished clip is a few cm from this intermediate pose (later smoothing / IK): min-filter the cap +-kf frames
    if kf > 0:
        pad = np.concatenate([np.full(kf, 9.0), cap_v, np.full(kf, 9.0)])
        cap_v = np.min([pad[k:k + len(cap_v)] for k in range(2 * kf + 1)], axis=0)
    ceil = np.where(cap_v < 8.0, cap_v + float(args.roof_top), np.nan)
    cap = np.where(cap_v < 8.0, cap_v - st[1] + pe[1], 9.0)  # world height of the clip
    if getattr(args, 'roof_cap_file', None):  # closed loop (roof_cap_iter.py): the cap measured at the head positions of the finished previous pass
        prev = np.asarray(json.load(open(args.roof_cap_file, encoding='utf-8'))['cap_y'], float)
        assert len(prev) == len(cap), 'roof cap file length differs'
        cap = np.minimum(cap, prev)
    return np.where(cap < 8.0, np.maximum(cap, floor_y), 9.0), float(floor_y), float(np.nanmin(ceil)) if np.isfinite(ceil).any() else None, relax, world_cap


def smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def anchor_offsets(tgt, Wt, Wr, common):
    """For each player-only bone: best anchor among common bones + per-frame offset (rot, trans in anchor frame)."""
    res = {}
    cidx = [tgt.index[b] for b in common]
    for b, name in enumerate(tgt.names):
        if name in common:
            continue
        best = None
        for a in cidx:
            Rinv = np.transpose(Wr[:, a], (0, 2, 1))
            ot = np.einsum('tij,tj->ti', Rinv, Wt[:, b] - Wt[:, a])
            orr = Rinv @ Wr[:, b]
            var = float(np.mean(np.linalg.norm(ot - ot.mean(0), axis=-1)) + 0.3 * np.mean(np.linalg.norm((orr - orr.mean(0)).reshape(len(orr), -1), axis=-1)))
            if best is None or var < best[0]:
                best = (var, a, ot, orr)
        res[b] = best
    return res


def foot_contacts(sk, lt, lr, foot, toe=None, h=0.06, v=0.25):
    """Frames where the foot is low and slow (in root-motion space: world = Root * locals)."""
    Wt, _ = sk.world(lt, lr)
    p = Wt[:, sk.index[foot]]
    vel = np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=-1) * FPS]
    ground = p[:, 1] - p[:, 1].min()
    return (ground < h) & (vel < v), p


def plant_frames(sk, lt, lr, foot, slow=1.0, fast=1.5):
    """Foot plant = first frame whose foot speed drops below `slow` m/s after exceeding `fast` m/s (re-armed by
    the next fast phase). Calibrated on the vanilla pwa_sport2_driver enter (gives 24/46/50 vs authored 20/42/50)."""
    Wt, _ = sk.world(lt, lr)
    p = Wt[:, sk.index[foot]]
    vel = np.r_[0, np.linalg.norm(np.diff(p, axis=0), axis=-1) * FPS]
    out, armed = [], False
    for i, v in enumerate(vel):
        if v > fast:
            armed = True
        elif armed and v < slow:
            out.append(i)
            armed = False
    return out


def contact_starts(mask):
    return [int(i) for i in range(1, len(mask)) if mask[i] and not mask[i - 1]] + ([0] if mask[0] else [])


def catmull_rom(times_src, vals, times_out):
    """Uniform Catmull-Rom interpolation of (Ts, ...) samples at times_src (uniform spacing) onto times_out.
    Used for the time warp instead of piecewise-linear sampling (which leaves a velocity step at every source key)."""
    dt = float(times_src[1] - times_src[0])
    n = len(times_src)
    u = np.clip(np.asarray(times_out) / dt, 0, n - 1)
    i1 = np.minimum(np.floor(u).astype(int), n - 2)
    s = (u - i1).reshape((-1,) + (1,) * (vals.ndim - 1))
    i0, i2, i3 = np.maximum(i1 - 1, 0), i1 + 1, np.minimum(i1 + 2, n - 1)
    p0, p1, p2, p3 = vals[i0], vals[i1], vals[i2], vals[i3]
    return 0.5 * ((2 * p1) + (-p0 + p2) * s + (2 * p0 - 5 * p1 + 4 * p2 - p3) * s ** 2 + (-p0 + 3 * p1 - 3 * p2 + p3) * s ** 3)


def gauss1d(x, sigma, mode='edge'):
    """Gaussian smoothing along axis 0 (any trailing shape); mode 'edge' (replicate) or 'zero' (rest outside)."""
    if sigma <= 0:
        return x.copy()
    r = max(1, int(3 * sigma))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = [(r, r)] + [(0, 0)] * (x.ndim - 1)
    xp = np.pad(x, pad, mode='edge' if mode == 'edge' else 'constant')
    out = np.zeros_like(x, dtype=np.float64)
    for o, kk in enumerate(k):
        out += kk * xp[o:o + len(x)]
    return out


def speed_rule_violations(v, ratio=0.2, floor=1.0):
    """Root speed rule: |v_i - v_(i-1)| <= ratio * max(v_i, v_(i-1), floor) for consecutive segment speeds v (cm/frame), with
    the entity at rest before the first and after the last segment. Returns the excess per transition (> 0 = violation)."""
    vv = np.r_[0.0, v, 0.0]
    dv = np.abs(np.diff(vv))
    lim = ratio * np.maximum(np.maximum(vv[1:], vv[:-1]), floor)
    return dv - lim


def smooth_root_profile(p, yaw, ratio=0.2, floor=1.0, sigma=2.5, max_iter=500, ref=None):
    """Endpoint-preserving smoothing of a root trajectory (positions p (T,3) m, yaw (T,) deg), entity at rest before and after:
      path: the geometry is Gaussian-smoothed (end points pinned) and re-timed along its arc length with a speed profile =
            the Gaussian-smoothed speed of `ref` (the hips, so the entity moves with the body; default the root itself),
            scaled (bisection) and limited forward / backward to the speed rule (speed_rule_violations) so that it covers
            the path length;
      yaw:  the yaw rate is Gaussian-smoothed (rest outside) and its residual to the total turn redistributed in proportion
            to the smoothed rate.
    The path is pulled towards the straight chord when it is longer than the rule allows (alpha < 1).
    Returns positions, yaw and dict(alpha, path_cm)."""
    T = len(p)
    a = np.linspace(0, 1, T)[:, None]
    ps0 = gauss1d(p, sigma, 'edge')
    ps0 = ps0 + (1 - a) * (p[0] - ps0[0]) + a * (p[-1] - ps0[-1])
    chord = p[0] + a * (p[-1] - p[0])

    def path_len(al):
        q = chord + al * (ps0 - chord)
        return q, float(np.linalg.norm(np.diff(q, axis=0), axis=-1).sum()) * 100

    # the longest path the speed rule allows in T frames (rest at both ends, growth limit at the tightest margin below):
    # a longer (meandering) root path is pulled towards the straight start -> seat chord until it fits
    env = np.full(T - 1, 1e9)
    for _ in range(2):
        prev = 0.0
        for i in range(len(env)):
            env[i] = min(env[i], prev + ratio * 0.5 * max(prev, floor))
            prev = env[i]
        env = env[::-1]
    cap = 0.9 * float(env.sum())
    ps, L = path_len(1.0)
    alpha = 1.0
    if L > cap:
        lo, hi = 0.0, 1.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if path_len(mid)[1] > cap:
                hi = mid
            else:
                lo = mid
        alpha = lo
        ps, L = path_len(alpha)
    seg = np.linalg.norm(np.diff(ps, axis=0), axis=-1)
    s_path = np.r_[0.0, np.cumsum(seg)]
    v0 = gauss1d(np.linalg.norm(np.diff(p if ref is None else ref, axis=0), axis=-1) * 100, sigma, 'zero')
    v0 = np.maximum(v0, 1e-6)
    v0 *= L / v0.sum()
    it = 0
    # the re-timed positions are interpolated on the path polyline (chord <= arc): tighten the margin until the rule holds
    # on the actual per-frame displacements
    for lim in (0.9, 0.8, 0.7, 0.6, 0.5):
        def clip_fb(x):
            # one forward and one backward pass of the growth limit up(v) = v + ratio * lim * max(v, floor) (rest at both
            # ends) give a sequence that meets it in both directions
            x = x.copy()
            prev = 0.0
            for i in range(len(x)):
                x[i] = min(x[i], prev + ratio * lim * max(prev, floor))
                prev = x[i]
            prev = 0.0
            for i in range(len(x) - 1, -1, -1):
                x[i] = min(x[i], prev + ratio * lim * max(prev, floor))
                prev = x[i]
            return x
        # the clipped sum grows monotonically with the scale c of the target profile: bisect c so the speeds cover the path
        lo, hi = 0.0, 1.0
        while clip_fb(hi * v0).sum() < L and hi < 1e9:
            hi *= 2
        for it in range(max_iter):
            mid = 0.5 * (lo + hi)
            if clip_fb(mid * v0).sum() < L:
                lo = mid
            else:
                hi = mid
            if hi - lo < 1e-9 * hi:
                break
        v = clip_fb(hi * v0)
        v *= L / v.sum()  # residual of the bisection (relative 1e-9)
        s_new = np.r_[0.0, np.cumsum(v)] / 100
        s_new[-1] = s_path[-1]
        p2 = np.stack([np.interp(s_new, s_path, ps[:, k]) for k in range(3)], axis=-1)
        p2[0], p2[-1] = p[0], p[-1]
        if speed_rule_violations(np.linalg.norm(np.diff(p2, axis=0), axis=-1) * 100, ratio, floor).max() <= 0:
            break
    Y = yaw[-1] - yaw[0]
    w = gauss1d(np.diff(yaw), sigma, 'zero')
    aw = np.abs(w)
    w = w + (Y - w.sum()) * (aw / aw.sum() if aw.sum() > 1e-9 else np.full(len(w), 1.0 / len(w)))
    y2 = np.r_[yaw[0], yaw[0] + np.cumsum(w)]
    return p2, y2, dict(alpha=alpha, path_cm=L, bisect_steps=it)


def minjerk(x):
    x = np.clip(x, 0, 1)
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def plant_segments(f, min_plant=3):
    """Plant intervals [a, b] (inclusive frames) where the thresholded swing weight is 0; plants shorter than min_plant
    frames (except at the clip ends) are treated as swing."""
    fe = np.clip((f - 0.05) / 0.95, 0, 1)
    planted = fe <= 0
    T = len(f)
    segs = []
    i = 0
    while i < T:
        if planted[i]:
            j = i
            while j + 1 < T and planted[j + 1]:
                j += 1
            segs.append([i, j])
            i = j + 1
        else:
            i += 1
    segs = [s for s in segs if s[1] - s[0] + 1 >= min_plant or s[0] == 0 or s[1] == T - 1]
    if not segs or segs[0][0] != 0:
        segs.insert(0, [0, 0])
    if segs[-1][1] != T - 1:
        segs.append([T - 1, T - 1])
    return segs


def keyframe_goals(follow_pos, f, start_pos, end_target, land_by=None, replant_frames=8, replant_min_m=0.03, min_swing=8,
                   hip_pos=None, reach=None, replant_raised_by=None):
    """Foot goals from plant keyframes: each plant holds one position exactly (first = start_pos, last = end_target, middle =
    the corrected foot at the plant's middle frame); a swing goes from P_a to P_b on a min-jerk weight m and adds the
    corrected (body-following) source swing's deviation from its own chord, windowed by 4 m (1 - m):
    goal = P_a + m (P_b - P_a) + 4 m (1 - m) [(F(t) - F(a)) - m (F(b) - F(a))], so lift-off and landing have zero velocity
    and the swing keeps its timing and its arc. The last swing lands no later than land_by (frame). hip_pos / reach: the
    upper-leg joint (T,3) and the reach limit (m) used to end a middle plant before its spot is out of reach.
    Returns goals and the plant list."""
    T = len(follow_pos)
    segs = plant_segments(f)
    if land_by is not None and len(segs) >= 2 and segs[-1][0] > land_by:
        segs[-1][0] = max(land_by, segs[-2][1] + 2)
    replant = None
    if end_target is not None and land_by is not None and len(segs) >= 2:
        # the last plant starts on the source spot; when that is away from the end pose foot and the plant starts early,
        # the foot stays there and steps onto the end pose spot (landing at land_by) instead of being dragged by the IK
        a, b = segs[-1]
        spot = follow_pos[min(a + 2, b)]
        dist = float(np.linalg.norm(spot - end_target))
        # a step up onto a raised spot (the peg) waits until the hips have settled: the supporting leg would otherwise
        # fold fast under the descending hips (> 15 deg/frame) and the IK could not keep it planted
        land = land_by
        if replant_raised_by is not None and end_target[1] - spot[1] > 0.2:
            land = max(land_by, replant_raised_by)
        s0 = land - replant_frames
        if dist > replant_min_m and s0 >= a + 2:
            segs[-1] = [a, s0]
            segs.append([land, T - 1])
            replant = dict(dist_m=dist, swing=[s0, land])
    pos = []
    for k, (a, b) in enumerate(segs):
        if k == 0:
            pos.append(start_pos)
        elif k == len(segs) - 1 and end_target is not None:
            pos.append(end_target)
        elif replant is not None and k == len(segs) - 2:
            pos.append(follow_pos[min(a + 2, b)])
        else:
            pos.append(follow_pos[(a + b) // 2])
    # a swing shorter than min_swing frames between plants more than 3 cm apart starts earlier (the plant before it ends
    # earlier): no hop of several cm per frame onto the next plant
    for k in range(len(segs) - 1):
        a, b = segs[k]
        s1 = segs[k + 1][0]
        if s1 - b < min_swing and np.linalg.norm(pos[k + 1] - pos[k]) > 0.03:
            segs[k][1] = max(a, s1 - min_swing)
    if hip_pos is not None:
        # a middle plant ends before its spot gets out of the leg's reach (the hips move on): lift the foot instead of
        # letting the IK drag it
        for k in range(1, len(segs) - 1):
            a, b = segs[k]
            far = np.flatnonzero(np.linalg.norm(hip_pos[a:b + 1] - pos[k], axis=-1) > reach)
            if far.size:
                segs[k][1] = a + int(far[0]) - 1
        # the last plant starts once its spot stays within reach (the landing waits for the hips); the end pose may itself
        # hold the leg nearly straight, so its own extension (+5 mm) counts as reachable
        a, b = segs[-1]
        # (the hips here are from before the end blend: keep a 3 % margin so the leg is not at full stretch on landing)
        reach_end = max(0.97 * reach, float(np.linalg.norm(hip_pos[-1] - pos[-1])) + 0.005)
        far = np.flatnonzero(np.linalg.norm(hip_pos[a:] - pos[-1], axis=-1) > reach_end)
        if far.size and len(segs) > 1:
            segs[-1][0] = min(T - 2, max(a, a + int(far[-1]) + 1))
    # middle plants shorter than 3 frames are dropped (the swing goes on to the next plant)
    keep = [k for k, (a, b) in enumerate(segs) if k in (0, len(segs) - 1) or b - a + 1 >= 3]
    segs = [segs[k] for k in keep]
    pos = [pos[k] for k in keep]
    F = follow_pos
    g = np.zeros_like(F)
    for k, (a, b) in enumerate(segs):
        g[a:b + 1] = pos[k]
        if k + 1 < len(segs):
            s0, s1 = b, segs[k + 1][0]
            if s1 - s0 < 1:
                continue
            m = minjerk(np.linspace(0, 1, s1 - s0 + 1))[:, None]
            Fw = F[s0:s1 + 1]
            # chord P_a -> P_b on the min-jerk timing plus the source swing's deviation from its own chord (the lift over
            # the seat), windowed by 4 m (1 - m) so the foot leaves and lands with zero velocity
            dev = (Fw - F[s0]) - m * (F[s1] - F[s0])
            g[s0:s1 + 1] = pos[k] + m * (pos[k + 1] - pos[k]) + 4 * m * (1 - m) * dev
    return g, dict(plants=segs, replant=replant)


# limb IK helpers: the game's player leg IK (and the arm IK while the hands are in world space) pulls the limbs onto these;
# in the vanilla clips they sit on the bone they describe (feet / hands <= 0.2 cm, poles at a constant offset from the knee /
# elbow). Candidate anchors per helper (the one with the smallest offset variance over the vanilla clip is used).
LIMB_IK_HELPERS = {}
for _s in ('Left', 'Right'):
    _l = _s.lower()
    for _h in ('leg_%s_ik' % _l, 'Torso_%sLeg_Foot_IK_JNT' % _s, 'Torso_World_%sLeg_Foot_IK_JNT' % _s):
        LIMB_IK_HELPERS[_h] = (_s + 'Foot',)
    for _h in ('Torso_%sLeg_PoleVector_JNT' % _s, 'Torso_World_%sLeg_PoleVector_JNT' % _s):
        LIMB_IK_HELPERS[_h] = (_s + 'Leg', _s + 'UpLeg')
    for _h in ('arm_%s_ik' % _l, 'Torso_%sArm_Hand_IK_JNT' % _s, 'Torso_World_%sArm_Hand_IK_JNT' % _s):
        LIMB_IK_HELPERS[_h] = (_s + 'Hand',)
    for _h in ('Torso_%sArm_PoleVector_JNT' % _s, 'Torso_World_%sArm_PoleVector_JNT' % _s):
        LIMB_IK_HELPERS[_h] = (_s + 'ForeArm', _s + 'Arm')


def limb_anchor_offsets(tgt, vWt, vWr):
    """Per limb IK helper: (anchor index, per-frame offset translation (T,3) in the anchor frame, rotation (T,3,3)) from the
    vanilla clip (time-aligned), anchor = the candidate with the smallest offset variance."""
    res = {}
    for h, cands in LIMB_IK_HELPERS.items():
        if h not in tgt.index:
            continue
        b = tgt.index[h]
        best = None
        for an in cands:
            a = tgt.index[an]
            Rinv = np.transpose(vWr[:, a], (0, 2, 1))
            ot = np.einsum('tij,tj->ti', Rinv, vWt[:, b] - vWt[:, a])
            orr = Rinv @ vWr[:, b]
            var = float(np.mean(np.linalg.norm(ot - ot.mean(0), axis=-1)) + 0.3 * np.mean(np.linalg.norm((orr - orr.mean(0)).reshape(len(orr), -1), axis=-1)))
            if best is None or var < best[0]:
                best = (var, a, ot, orr)
        res[b] = best[1:]
    return res


LEG_BONES = ('LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase', 'LeftHeel', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase',
             'RightHeel')


class _StartCaptured(Exception):
    """--hips-path-keep-start: the inner no-layer build stops once its start pose is known."""


def build(args):
    src = S.Skeleton.from_glb(args.src)
    tgt = S.Skeleton.from_glb(args.target)
    s_clip = src.clip(args.src_clip)
    t_clip = tgt.clip(args.target_clip)
    s_dur, s_times = frames_of(src, s_clip)
    t_dur, t_times = frames_of(tgt, t_clip)
    src_times = s_times
    native_times = s_times
    if args.target_duration:  # bike entry: time-warp the source onto the vanilla clip's duration (PSM timing unchanged)
        n_out = int(round(t_dur * FPS)) + 1
        src_times = np.linspace(0.0, float(s_times[-1]), n_out)
        s_times = np.arange(n_out) / FPS
        s_dur = float(s_times[-1])
        if args.start_hold > 0:
            # door clearance: V waits H s at the start pose (the door of the car swings out right where she steps in), then plays the
            # source faster (total duration unchanged): the source time follows a velocity profile that ramps 0 -> v0 over the first
            # fraction lam of the remaining time and then stays constant (no velocity step, no acceleration spike beyond the ramp)
            lam = float(args.hold_ease)
            tau = np.clip((s_times - args.start_hold) / max(1e-6, s_dur - args.start_hold), 0.0, 1.0)
            v0 = 1.0 / (1.0 - lam / 2.0)
            u = np.where(tau < lam, v0 * tau ** 2 / (2.0 * lam), v0 * (tau - lam / 2.0))
            src_times = float(native_times[-1]) * np.clip(u, 0.0, 1.0)
    if args.warp_interp == 'cubic' and len(src_times) != len(native_times):
        # sample the source at its own 30 fps frames, then Catmull-Rom onto the warped times: a piecewise-linear resample
        # leaves a velocity step at every source key (joint acceleration spikes after the warp)
        nlt0, nlr0 = src.sample_local(s_clip, native_times)
        nlr0 = S.qcontinuous(nlr0)
        lt = catmull_rom(native_times, nlt0, src_times)
        lr = catmull_rom(native_times, nlr0, src_times)
        lr /= np.linalg.norm(lr, axis=-1, keepdims=True)
    else:
        lt, lr = src.sample_local(s_clip, src_times)
    lr = S.qcontinuous(lr)
    if args.mirror:
        lt, lr = mirror_pose(src, lt, lr)
    T = len(s_times)
    common = [b for b in src.names if b in tgt.index]
    # 3. retarget
    nlt = np.repeat(tgt.rest_t[None], T, 0).copy()
    nlr = np.repeat(tgt.rest_r[None], T, 0).copy()
    for b in common:
        i, j = src.index[b], tgt.index[b]
        nlr[:, j] = lr[:, i]
        if b == 'Root':
            nlt[:, j] = lt[:, i]
        else:
            nlt[:, j] = tgt.rest_t[j] + (lt[:, i] - src.rest_t[i])
    def limit_rot_steps(local=False, frozen=None, lim=None, keep=0.0):  # per-bone angular speed limit (deg / frame): binomial smoothing of the offending bones only
        # frozen: {bone: (T,) bool} frames a local pass must not change (planted leg frames)
        lim = args.smooth_rot_limit if lim is None else lim
        smoothed = {}
        for b in common:
            if b in ('Root', 'reference_joint', 'Trajectory'):
                continue
            j = tgt.index[b]
            q = S.qcontinuous(nlr[:, j].copy())
            q_in = q.copy()
            n0 = 0
            pad = args.limit_pad
            while n0 < (200 if local else 80):
                d = 2 * np.degrees(np.arccos(np.clip(np.abs(np.sum(q[1:] * q[:-1], axis=-1)), 0, 1)))
                if d.max() <= lim:
                    break
                if local:
                    # only the frames next to an offending step (2 frames around it) are smoothed, with a feathered weight
                    # (a hard mask edge leaves a velocity step between smoothed and untouched frames: a one-frame
                    # acceleration spike after many passes); the first and last two frames stay as the exact eases left
                    # them (endpoint steps)
                    bad = np.flatnonzero(d > lim)
                    core = np.zeros(len(q))
                    for i_ in bad:
                        core[max(0, i_ - 1):min(len(q), i_ + 3)] = 1.0
                    wgt = np.minimum(1.0, 2.0 * gauss1d(core, 1.5, 'zero'))
                    wgt[:2] = 0.0
                    wgt[-2:] = 0.0
                    qs = q.copy()
                    qs[1:-1] = q[:-2] + 2 * q[1:-1] + q[2:]
                    qs[1:-1] /= np.linalg.norm(qs[1:-1], axis=-1, keepdims=True)
                    if frozen and b in frozen:
                        wgt = np.where(frozen[b], 0.0, wgt)
                    if not (wgt > 0).any():
                        break
                    q = S.qcontinuous(G.slerp(q, qs, wgt[:, None]))
                elif pad > 0:
                    # replicated frames outside the clip (rest before / after): the end frames are smoothed too, so the
                    # first / last segment does not keep its velocity; the exact end poses are restored by the end eases
                    qp = np.concatenate([np.repeat(q[:1], pad, 0), q, np.repeat(q[-1:], pad, 0)])
                    qs = qp.copy()
                    qs[1:-1] = qp[:-2] + 2 * qp[1:-1] + qp[2:]
                    qs[1:-1] /= np.linalg.norm(qs[1:-1], axis=-1, keepdims=True)
                    q = qs[pad:-pad]
                else:
                    qs = q.copy()
                    qs[1:-1] = q[:-2] + 2 * q[1:-1] + q[2:]
                    qs[1:-1] /= np.linalg.norm(qs[1:-1], axis=-1, keepdims=True)
                    q = qs
                n0 += 1
            if n0:
                smoothed[b] = n0
                if keep > 0 and not local:  # the first `keep` seconds keep the input of this pass (start pose), then a min-jerk blend into the smoothed bone
                    kw = minjerk((s_times - keep) / 0.3)[:, None]
                    q = S.qcontinuous(G.slerp(q_in, q, kw))
            nlr[:, j] = q
        print('smoothed bones (passes):', len(smoothed), sorted(smoothed.items(), key=lambda kv: -kv[1])[:6])
    if args.smooth_rot_limit > 0:
        limit_rot_steps(lim=args.smooth_rot_limit_global if args.smooth_rot_limit_global else None)  # the first, GLOBAL pass may use its own (looser) limit: it also reshapes the start pose
        if args.smooth_rot_limit2 > 0:  # second global pass (tighter limit) that leaves the first --smooth-keep-first seconds as the first pass left them
            limit_rot_steps(lim=args.smooth_rot_limit2, keep=args.smooth_keep_first)
    # vanilla target clip, time-aligned to the new frame count
    v_times = np.linspace(0, t_dur, T)
    vlt, vlr = tgt.sample_local(t_clip, v_times)
    vlr = S.qcontinuous(vlr)
    vWt, vWr = tgt.world(vlt, vlr)
    ri = tgt.index['Root']
    # 5. root end correction (before helpers so they follow the corrected root)
    end_pos = vlt[-1, ri].copy()
    if (args.start_out and args.out_axis) or args.start_back:
        # door clearance: move the START spot (where the engine places V before the clip) away from the car side by this many metres
        # (--start-out, seat-frame x) and/or farther back behind the door's rear edge (--start-back, seat-frame +z = behind the seat; the
        # door clearance model: Source\tools\anim\vehicle-entry\check_door_clearance.py).
        # The engine places the entity at seat * inverse(root delta), so a longer root delta (end_pos + delta) moves the start spot by
        # -R_end^T delta in the seat frame; out_axis is the car-side-outward direction in the seat frame (-1 driver / +1 passenger, x).
        sv = np.array([float(args.out_axis or 0.0) * args.start_out, 0.0, float(args.start_back)])
        end_pos = end_pos - G.qrot(vlr[-1, ri], sv)
        out_world = -G.qrot(vlr[-1, ri], np.array([float(args.out_axis or 0.0) * args.start_out, 0.0, 0.0]))  # the start-out part of the shift (for --out-ramp)
    else:
        out_world = np.zeros(3)
    # the head cap below compares depth with the vanilla clip's envelope, which is measured from the vanilla start spot: with --start-out the
    # root end is shifted by start_shift against the vanilla one, so the vanilla-equivalent depth of a point is -(p - start_shift).z
    start_shift = end_pos - vlt[-1, ri]
    end_yaw = yaw_of(vlr[-1, ri])
    cur_pos = nlt[-1, ri].copy()
    cur_yaw = yaw_of(nlr[-1, ri])
    before = dict(pos_err_m=float(np.linalg.norm(cur_pos - end_pos)), yaw_err_deg=float(cur_yaw - end_yaw),
                  pos_err_xyz=(cur_pos - end_pos).round(4).tolist())
    d_pos = end_pos - cur_pos
    d_yaw = end_yaw - cur_yaw
    t = s_times / s_times[-1]
    w = smoothstep((t - args.corr_start) / max(1e-6, args.corr_end - args.corr_start))
    # --back-ramp A B: the START-SHIFT part of the correction (start_shift = the --start-back / --start-out vector) has its own smoothstep over A..B seconds of clip
    # time instead of the common 25..80 % window: the forward move that brings V from the start-back spot to the vanilla path completes BEFORE the source's
    # sideways entry (clip 0.5..1.0 s) carries her past the B-pillar. The base correction (vanilla end - source end) keeps w. Default off = d_pos * w as before.
    # --out-ramp A B: the START-OUT part (seat-frame x) has its OWN window A..B seconds, independent of the start-back window (default: it follows the start-back window):
    # the lateral offset from the car side keeps the hands / arms off the rear door skin only while V is still behind the front door's rear edge (clip ~0..0.4 s) and
    # is gone before the native hold releases (the open door must not meet the head / arms of the lagging visible body at t ~ 0.8 s).
    w_shift = None
    w_out = None
    if args.out_ramp and np.linalg.norm(out_world) > 1e-9:
        oa, ob = args.out_ramp
        w_out = smoothstep((s_times - oa) / max(1e-6, ob - oa))
    if (args.back_ramp or w_out is not None) and np.linalg.norm(start_shift) > 1e-9:
        if args.back_ramp:
            ba, bb = args.back_ramp
            w_shift = smoothstep((s_times - ba) / max(1e-6, bb - ba))
        else:
            w_shift = w
        d_pos = d_pos - start_shift  # d_pos = base correction; the shift is added below with w_shift
    shift_t = None
    if w_shift is not None:
        shift_t = w_shift[:, None] * start_shift[None] if w_out is None else w_shift[:, None] * (start_shift - out_world)[None] + w_out[:, None] * out_world[None]
    W0, _ = tgt.world(nlt, nlr)  # effector positions before any correction
    eff = ['LeftFoot', 'RightFoot', 'LeftHand', 'RightHand']
    orig = {k: W0[:, tgt.index[k]].copy() for k in eff}
    follow = {k: IK.follow_weight(orig[k]) for k in eff}
    # yaw correction about the vertical through the root position at each frame (rotate the root in place), then shift
    nlr[:, ri] = G.qmul(qyaw(w * d_yaw), nlr[:, ri])
    nlt[:, ri] = nlt[:, ri] + w[:, None] * d_pos[None]
    if w_shift is not None:
        nlt[:, ri] = nlt[:, ri] + shift_t
    yaw_rep = None
    if args.root_yaw_total is not None:
        # 5a. total root yaw reduction (bike entry): the engine places the clip start at seat * inverse(root delta), so a
        # large total root yaw makes the entity slide in facing away from the bike. Scale the root's turn towards the end
        # (seat) yaw: y'(t) = yE - k (yE - y(t)); each root keeps its position (the start spot beside the bike and the seat
        # are unchanged in the seat frame), the body turns with the root, and the clip is re-based so frame 0 is identity.
        y = np.degrees(np.unwrap(np.radians(yaw_of(nlr[:, ri]))))
        tot = float(y[-1] - y[0])
        k = min(1.0, abs(args.root_yaw_total) / abs(tot)) if abs(tot) > 1e-6 else 1.0
        c = (1 - k) * (y - y[-1])
        nlr[:, ri] = G.qmul(qyaw(-c), nlr[:, ri])
        y0 = float(y[0] - c[0])
        p0 = nlt[0, ri].copy()
        qb = np.repeat(qyaw(np.array([-y0]))[0][None], T, 0)
        nlt[:, ri] = G.qrot(qb, nlt[:, ri] - p0[None])
        nlr[:, ri] = G.qmul(qb, nlr[:, ri])
        yaw_rep = dict(total_before_deg=tot, total_after_deg=float(yaw_of(nlr[-1, ri]) - yaw_of(nlr[0, ri])), scale=k)
    hips_path_rep = None
    hp_shift = None  # (T, 3) clip-world translation of the hips path layer (--hips-path-hands)
    if args.hips_path:
        # --hips-path EPS DELTA SIGMA: keep the Hips from sliding to the rear while the pelvis sits down. Along the car-forward axis s (seat frame -z) the Hips path s(t) is held
        # inside the band [c(t) - DELTA, c(t)] with c(t) = min(running maximum of s, s_seat + EPS): a lunge past the seat is pulled back to s_seat + EPS, a later dip is carried
        # forward to c - DELTA. The difference is applied to the root as a smooth (SIGMA frames) forward translation (zero at the first and the last frame), so the roof cap, the
        # limb locks, the end blend and the head duck below all solve on the new path.
        eps_h, dlt_h, sig_h = (float(v) for v in args.hips_path)
        hi_ = tgt.index['Hips']
        fwd = G.qrot(vlr[-1, ri], np.array([0.0, 0.0, -1.0]))
        Wp, _ = tgt.world(nlt, nlr)
        if args.end_pose:
            ep_, ec_ = args.end_pose.rsplit(':', 1)
            e_last_ = ec_.endswith('@last')
            ec_ = ec_[:-5] if e_last_ else ec_
            esk_ = S.Skeleton.from_glb(ep_)
            ecl_ = esk_.clip(ec_)
            _, etimes_ = frames_of(esk_, ecl_)
            e_lt_, e_lr_ = esk_.sample_local(ecl_, etimes_[-1:] if e_last_ else etimes_[:1])
        else:
            e_lt_, e_lr_ = vlt[-1:], vlr[-1:]
        e_lt_, e_lr_ = e_lt_[:1].copy(), e_lr_[:1].copy()
        e_lt_[0, ri] = nlt[-1, ri]
        e_lr_[0, ri] = nlr[-1, ri]
        We_, _ = tgt.world(e_lt_, e_lr_)
        s_end = float(We_[0, hi_] @ fwd)
        s_h = Wp[:, hi_] @ fwd
        c_h = np.minimum(np.maximum.accumulate(s_h), s_end + eps_h)
        if args.hips_path_hold:
            # --hips-path-hold F0 F1 RATE: frames before F0 keep the source path (no cap); frames F0..F1-1 hold the level of frame F0-1 (no further forward travel); from F1 the cap
            # comes down by at most RATE m per frame until it meets the cap above (the rearward travel then happens at a bounded speed in the frames chosen, not while sitting down)
            h0_, h1_, hr_ = int(args.hips_path_hold[0]), int(args.hips_path_hold[1]), float(args.hips_path_hold[2])
            c2_ = np.maximum.accumulate(s_h).copy()
            lev_ = float(s_h[h0_ - 1])
            for i_ in range(h0_, T):
                if i_ >= h1_:
                    lev_ = lev_ - hr_
                c2_[i_] = min(c2_[i_], max(c_h[i_], lev_))
            c_h = c2_
        dpath = np.clip(s_h, c_h - dlt_h, c_h) - s_h
        dpath = gauss1d(dpath, sig_h)
        ii_ = np.arange(T)
        fa_, fb_ = args.hips_path_fade if args.hips_path_fade else (0.0, 6.0)  # --hips-path-fade A B: the layer fades in over frames A..B (default 0..6)
        dpath = dpath * smoothstep((ii_ - fa_) / max(1e-6, fb_ - fa_)) * smoothstep((T - 1 - ii_) / 6.0)
        nlt[:, ri] = nlt[:, ri] + dpath[:, None] * fwd[None]
        hp_shift = dpath[:, None] * fwd[None]
        hips_path_rep = dict(eps_m=eps_h, delta_m=dlt_h, sigma_frames=sig_h, s_end=s_end, max_forward_m=float(dpath.max()), max_back_m=float(dpath.min()),
                             frames_changed=int((np.abs(dpath) > 0.005).sum()))
    W1, _ = tgt.world(nlt, nlr)
    # 5b. head-height cap (duck): never raise the head above the vanilla player clip's envelope
    duck_rep = dict(head_peak_before=None, hips_drop_max_m=0.0)
    delta = np.zeros(T)
    if args.head_cap in ('vanilla', 'vanilla-depth'):
        hi = tgt.index['Head']
        if args.head_cap == 'vanilla':
            cap = IK.maxfilter(vWt[:, hi, 1], int(args.cap_window * FPS)) + args.cap_margin
        else:
            # spatial envelope: the start frame looks into the car (-Z glTF = towards the seat), so head depth = -z.
            # The vanilla player clip never has its head higher than h once it is at least this deep -> use the same
            # bound (the game's own collision-safe trajectory through the door opening).
            vd, vh = -vWt[:, hi, 2], vWt[:, hi, 1]
            if args.cap_clip:
                cpath, cclip = args.cap_clip.rsplit(':', 1)
                csk = S.Skeleton.from_glb(cpath)
                cc = csk.clip(cclip)
                cdur, ctimes = frames_of(csk, cc)
                clt, clr = csk.sample_local(cc, ctimes)
                cW, _ = csk.world(clt, S.qcontinuous(clr))
                vd, vh = -cW[:, csk.index['Head'], 2], cW[:, csk.index['Head'], 1]
            nd = -W1[:, hi, 2] + float(start_shift[2])
            cap = np.array([vh[vd >= d - args.cap_depth_tol].max() if np.any(vd >= d - args.cap_depth_tol) else vh[-1]
                            for d in nd]) + args.cap_margin
            # only near / inside the door opening: no cap while the head is clearly outside the car side
            s = smoothstep((nd - args.cap_start_depth) / max(1e-6, args.cap_full_depth - args.cap_start_depth))
            cap = (1 - s) * 3.0 + s * cap
        roof_rep = None
        if args.roof_json:
            rc, rfloor, rmin, rrelax, _ = roof_cap(args, W1, nlt, nlr, tgt, hi, ri)
            cap = np.minimum(cap, rc)
            roof_rep = dict(floor_y=rfloor, relax_seated_m=rrelax, lowest_ceiling_at_head_m=rmin, frames_capped=int((rc < 8.0).sum()), cap_y=[round(float(x), 3) for x in rc], head_y_before=[round(float(x), 3) for x in W1[:, hi, 1]])
        delta, hp = IK.hips_drop(tgt, nlt, nlr, cap)
        duck_rep = dict(roof=roof_rep, head_peak_before=hp, hips_drop_max_m=float(delta.max()), frames_lowered=int((delta > 0.005).sum()))
    # end pose (frame 0 of another clip of the same rig, e.g. the TPP standing-on-bike pose) or the vanilla last frame
    end_lt, end_lr = vlt[-1:], vlr[-1:]
    if args.end_pose:
        ep, ec = args.end_pose.rsplit(':', 1)
        e_last = ec.endswith('@last')  # 'clip@last': the clip's LAST frame (car entry: the vanilla enter's seated end pose)
        ec = ec[:-5] if e_last else ec
        esk = S.Skeleton.from_glb(ep)
        ecl = esk.clip(ec)
        _, etimes = frames_of(esk, ecl)
        elt, elr = esk.sample_local(ecl, etimes[-1:] if e_last else etimes[:1])
        assert list(esk.names) == list(tgt.names), 'end-pose rig differs from the target rig'
        end_lt, end_lr = elt, elr
    end_full_lt, end_full_lr = end_lt[:1].copy(), end_lr[:1].copy()
    end_full_lt[0, ri] = nlt[-1, ri]  # the end pose placed on this clip's final root (the seat)
    end_full_lr[0, ri] = nlr[-1, ri]
    Wend, _ = tgt.world(end_full_lt, end_full_lr)
    # 5c. limb lock: planted feet / holding hands keep their pre-correction world position; swinging limbs follow
    # the corrected (and lowered) body
    goals = {}
    plant_segs = {}
    for k in eff:
        shift = W1[:, tgt.index[k]] - orig[k]
        if 'Hand' in k:  # hands hold the door/roof only loosely: they always take part of the body correction
            follow[k] = np.maximum(follow[k], args.hand_follow_min)
        if args.plant_hold and 'Foot' in k:
            land = int(round(args.land_by * FPS)) if args.land_by else None
            up_, lo_ = IK.LIMBS[k]
            leg_len = float(np.linalg.norm(tgt.rest_t[tgt.index[lo_]]) + np.linalg.norm(tgt.rest_t[tgt.index[k]]))
            wf_ = W1[:, tgt.index[k]]
            if hp_shift is not None and args.hips_path_feet:
                # --hips-path-feet: the foot path the plant / swing goals are taken from carries K x the hips path translation (default 1 = all of it, as before; K = 0: the source world path)
                kf = dict((s.split(':')[0], float(s.split(':')[1])) if ':' in s else (k, float(s)) for s in args.hips_path_feet.replace('+', ',').split(','))
                if k in kf:
                    wf_ = wf_ + (kf[k] - 1.0) * hp_shift
            g, plant_segs[k] = keyframe_goals(wf_, follow[k], W1[0, tgt.index[k]],
                                              Wend[0, tgt.index[k]] if args.end_pose else None, land,
                                              hip_pos=W1[:, tgt.index[up_]], reach=(args.ik_max_extend or 1.0) * leg_len,
                                              replant_raised_by=int(round(args.replant_raised_by * FPS)) if args.replant_raised_by else None)
        else:
            if 'Hand' in k and args.hand_follow_shift > 0 and w_shift is not None:
                sh_s = shift_t  # the part of the body shift that is the start-back / start-out ramp
                g = orig[k] + follow[k][:, None] * (shift - sh_s) + args.hand_follow_shift * sh_s
            else:
                g = orig[k] + follow[k][:, None] * shift
        if 'Hand' in k and hp_shift is not None and args.hips_path_hands:
            # --hips-path-hands: the hand goal takes K x the hips path translation instead of follow x it (K = 1: the hand moves with the body, the arm keeps its reach;
            # K = 0: the hand keeps its source world path)
            kh = dict((s.split(':')[0], float(s.split(':')[1])) if ':' in s else (k, float(s)) for s in args.hips_path_hands.replace('+', ',').split(','))
            if k in kh:
                g = g + (kh[k] - follow[k])[:, None] * hp_shift
        g[:, 1] -= follow[k] * delta
        if 'Hand' in k and args.hand_out > 0 and (args.hand_out_side == 'both' or k.startswith(args.hand_out_side)):
            # --hand-out M A B: the hand goal is M metres farther from the car side (the --out-axis direction of the start shift) until A s, then back to its place by B s (smoothstep):
            # a hand resting on the car body behind the front door's rear edge would be inside the rear door skin
            if not args.out_axis:
                raise SystemExit('--hand-out needs --out-axis')
            hand_out_dir = G.qrot(vlr[-1, ri], np.array([float(args.out_axis), 0.0, 0.0]))
            ho_env = (1.0 - smoothstep((s_times - args.hand_out_window[0]) / max(1e-6, args.hand_out_window[1] - args.hand_out_window[0])))                 * smoothstep((s_times - args.hand_out_rise[0]) / max(1e-6, args.hand_out_rise[1] - args.hand_out_rise[0]))  # rises from 0 at the clip start: frame 0 (the pose held during the engine slide) is unchanged
            g = g + ho_env[:, None] * (args.hand_out * hand_out_dir)[None]
        goals[k] = g
    limbs = eff if args.hand_ik else eff[:2]
    duck_rep['limb_lock_max_err_m'] = IK.lock_limbs(tgt, nlt, nlr, goals, limbs, max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma)

    def planted_slide(own_plants=False):
        # per-frame foot displacement (m) over the frames the source foot is planted (follow < 0.05), or with own_plants
        # over the plant intervals of the keyframe goals (a deliberate re-plant step is a swing there)
        Wl, _ = tgt.world(nlt, nlr)
        out = {}
        for k in eff[:2]:
            if own_plants:
                msk = np.zeros(T - 1, bool)
                for a_, b_ in plant_segs[k]['plants']:
                    msk[a_:b_] = True
            else:
                msk = follow[k][1:] < 0.05
            out[k] = float(np.max(np.where(msk, np.linalg.norm(np.diff(Wl[:, tgt.index[k]], axis=0), axis=-1), 0)))
        return out
    duck_rep['foot_slide_planted_max_m'] = planted_slide()
    # 6. end blend of common bones towards the end pose (legs optionally over a longer window)
    if args.end_blend > 0:
        def ramp(sec):
            nb = sec * FPS
            return smoothstep((np.arange(T) - (T - 1 - nb)) / nb)
        wb_body = ramp(args.end_blend)
        wb_legs = ramp(args.end_blend_legs) if args.end_blend_legs else wb_body
        for b in common:
            if b == 'Root':
                continue
            j = tgt.index[b]
            wb = wb_legs if b in LEG_BONES else wb_body
            q0 = nlr[:, j]
            q1 = np.repeat(end_lr[:1, j], T, 0)
            nlr[:, j] = G.slerp(q0, q1, wb[:, None])
            nlt[:, j] = nlt[:, j] * (1 - wb[:, None]) + end_lt[:1, j] * wb[:, None]
        if args.plant_hold:  # the blend of local rotations moves the planted foot: lock it again (goal = end pose at the end)
            duck_rep['limb_lock_after_blend_err_m'] = IK.lock_limbs(tgt, nlt, nlr, goals, eff[:2], max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma)
    # 4. helpers
    Wt, Wr = tgt.world(nlt, nlr)
    offs = anchor_offsets(tgt, vWt, vWr, set(common))
    for b, (var, a, ot, orr) in offs.items():
        Wr[:, b] = Wr[:, a] @ orr
        Wt[:, b] = Wt[:, a] + np.einsum('tij,tj->ti', Wr[:, a], ot)
    flt, flr = tgt.local_from_world(Wt, Wr)
    helper = [tgt.index[n] for n in tgt.names if n not in common]
    nlt[:, helper] = flt[:, helper]
    nlr[:, helper] = flr[:, helper]
    nlr = S.qcontinuous(nlr)
    start_lt, start_lr = nlt[:1].copy(), nlr[:1].copy()
    if getattr(args, '_start_capture', None) is not None:  # inner no-layer pass of --hips-path-keep-start: hand the start pose over, skip the rest
        args._start_capture.extend([start_lt, start_lr])
        raise _StartCaptured()
    if args.hips_path and args.hips_path_keep_start:
        # --hips-path-keep-start (r13j): the start pose = the one of the same build WITHOUT the hips path layer. The layer is 0 at frame 0, but the knee bend direction of the
        # foot IK is smoothed over time (--ik-bend-sigma), so the layer's Hips motion of the next frames swivels the frame-0 knee (sport1 w:driver: RightLeg 0.43 cm). The
        # engine holds frame 0 during the vanilla slide (Turbo-R 0.8 s) while the door opens: the arrival ring sees exactly that pose. The ease-in below starts from it.
        if not args.ease_in > 0:
            raise SystemExit('--hips-path-keep-start needs --ease-in')
        a0_ = copy.copy(args)
        a0_.hips_path, a0_.hips_path_keep_start, a0_._start_capture = None, False, []
        try:
            build(a0_)
        except _StartCaptured:
            pass
        k_lt_, k_lr_ = a0_._start_capture
        Wk_, _ = tgt.world(k_lt_, k_lr_)
        Ws_, _ = tgt.world(start_lt, start_lr)
        dk_ = np.linalg.norm(Wk_[0] - Ws_[0], axis=-1)
        if hips_path_rep is not None:
            hips_path_rep['keep_start'] = dict(max_joint_shift_cm=round(float(dk_.max()) * 100, 3), joint=tgt.names[int(np.argmax(dk_))])
        start_lt, start_lr = k_lt_, k_lr_
    if args.smooth_rot_limit > 0 and not args.plant_hold:  # again after the limb IK / end blend / helpers
        limit_rot_steps()
    root_rep = None
    if args.root_smooth:
        # 8. root replacement: the time-warped NPC root is stair-stepped (its extraction steps are compensated by the Hips
        # track, the body itself moves smoothly). Replace it with an endpoint-preserving smoothed root (velocity and yaw
        # rate, speed rule of speed_rule_violations) and re-express the root's children so the world body is unchanged.
        Wt8, Wr8 = tgt.world(nlt, nlr)
        p_old = nlt[:, ri].copy()
        y_old = np.degrees(np.unwrap(np.radians(yaw_of(nlr[:, ri]))))
        v_old = np.linalg.norm(np.diff(p_old, axis=0), axis=-1) * 100
        hips_w = Wt8[:, tgt.index['Hips']]
        p_new, y_new, iters = smooth_root_profile(p_old, y_old, args.root_speed_ratio, args.root_speed_floor_cm, ref=hips_w)
        nlt[:, ri] = p_new
        nlr[:, ri] = qyaw(y_new)
        Rn = S.q2m(nlr[:, ri])
        RnT = np.transpose(Rn, (0, 2, 1))
        for j in range(len(tgt.names)):
            if tgt.parent[j] != ri or tgt.names[j] in ('Trajectory', 'reference_joint'):
                continue  # Trajectory stays identity (as in every vanilla clip); reference_joint is set below
            nlt[:, j] = np.einsum('tij,tj->ti', RnT, Wt8[:, j] - p_new)
            nlr[:, j] = S.m2q(RnT @ Wr8[:, j])
        nlr = S.qcontinuous(nlr)
        v_new = np.linalg.norm(np.diff(p_new, axis=0), axis=-1) * 100
        root_rep = dict(profile=iters, speed_rule_excess_before=float(speed_rule_violations(v_old, args.root_speed_ratio, args.root_speed_floor_cm).max()),
                        speed_rule_excess_after=float(speed_rule_violations(v_new, args.root_speed_ratio, args.root_speed_floor_cm).max()),
                        speed_cm_per_frame_max=float(v_new.max()))
    mono_rep = None
    if args.root_monotone is not None:
        # 8b. monotone root along the car-forward axis (r13e): the replaced root can still travel backward along the car while the body does not (the NPC root's late
        # settle into the seat: the entity slides to the rear although the Hips move forward). The car-forward component s(t) of the root (seat frame -z) is replaced by
        # its isotonic fit in the direction start -> seat (pool-adjacent-violators), clipped to [start, seat], Gaussian-smoothed (SIGMA frames, a smoothed monotone
        # sequence stays monotone) with exact endpoints; the root's children are re-expressed so the world body is unchanged (as in step 8). Only the entity path changes.
        import limb_path as LP
        Wt8, Wr8 = tgt.world(nlt, nlr)
        # car-forward in clip space: vehicle rig = sR Re^T (clip - root end) + seat (check_door_clearance.Model), vehicle forward = rig -z, so clip forward = Re sR^T (0, 0, -1).
        # The seat rotation sR of door-data is NOT identity on every class (utility: 5.7 deg pitch); without it the axis picks up the 1 m climb of the root.
        sR8 = np.eye(3)
        if args.root_monotone_door_json:
            dj8 = json.load(open(args.root_monotone_door_json, encoding='utf-8'))
            sR8 = np.array(dj8['seat'][args.root_monotone_side]['R'], float).reshape(3, 3)
        Re8 = S.q2m(nlr[-1:, ri])[0]
        fwd8 = Re8 @ sR8.T @ np.array([0.0, 0.0, -1.0])
        p_old = nlt[:, ri].copy()
        s_old = p_old @ fwd8
        s0_, s1_ = float(s_old[0]), float(s_old[-1])
        up_ = s1_ >= s0_
        s_iso = LP.pav(s_old) if up_ else -LP.pav(-s_old)
        s_iso = np.clip(s_iso, min(s0_, s1_), max(s0_, s1_))
        s_iso = gauss1d(s_iso, float(args.root_monotone), 'edge')
        a_ = np.linspace(0.0, 1.0, T)
        s_iso = s_iso + (1 - a_) * (s0_ - s_iso[0]) + a_ * (s1_ - s_iso[-1])
        ds_ = s_iso - s_old
        p_new = p_old + ds_[:, None] * fwd8[None]
        nlt[:, ri] = p_new
        Rn = S.q2m(nlr[:, ri])
        RnT = np.transpose(Rn, (0, 2, 1))
        for j in range(len(tgt.names)):
            if tgt.parent[j] != ri or tgt.names[j] in ('Trajectory', 'reference_joint'):
                continue
            nlt[:, j] = np.einsum('tij,tj->ti', RnT, Wt8[:, j] - p_new)
            nlr[:, j] = S.m2q(RnT @ Wr8[:, j])
        nlr = S.qcontinuous(nlr)
        # ranges where the entity path changed by > 5 mm and the world Hips travel (car-forward) over each range: a rearward Hips travel there would mean the
        # monotone root hides body motion
        hs_ = Wt8[:, tgt.index['Hips']] @ fwd8
        chg = np.abs(ds_) > 0.005
        ranges = []
        i_ = 0
        while i_ < T:
            if chg[i_]:
                k_ = i_
                while k_ + 1 < T and chg[k_ + 1]:
                    k_ += 1
                seg_ = hs_[i_:k_ + 1]
                ranges.append(dict(frames=[int(i_), int(k_)], root_change_max_m=round(float(np.abs(ds_[i_:k_ + 1]).max()), 4),
                                   hips_fwd_travel_m=round(float(seg_[-1] - seg_[0]), 4), hips_back_excursion_m=round(float(np.max(np.maximum.accumulate(seg_) - seg_)), 4)))
                i_ = k_ + 1
            else:
                i_ += 1
        v_m = np.linalg.norm(np.diff(p_new, axis=0), axis=-1) * 100
        mono_rep = dict(sigma_frames=float(args.root_monotone), direction='forward' if up_ else 'rearward', start_to_seat_m=round(s1_ - s0_, 4),
                        root_back_excursion_before_m=round(float(np.max(np.maximum.accumulate(s_old) - s_old)), 4),
                        root_back_excursion_after_m=round(float(np.max(np.maximum.accumulate(s_iso) - s_iso)), 4), changed_ranges=ranges,
                        speed_rule_excess_after=float(speed_rule_violations(v_m, args.root_speed_ratio, args.root_speed_floor_cm).max()),
                        speed_cm_per_frame_max=float(v_m.max()))
    if args.plant_hold:
        # final foot lock after the root replacement (the eases below restore the exact start and end poses; the step
        # limiter runs last, locally)
        duck_rep['limb_lock_final_err_m'] = IK.lock_limbs(tgt, nlt, nlr, goals, eff[:2], max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma)
        nlr = S.qcontinuous(nlr)
        duck_rep['plant_segments'] = {k: v for k, v in plant_segs.items()}
    nonroot = [j for j in range(len(tgt.names)) if j != ri]
    # helpers and the weapon attachment bones differ most from the TPP end pose: longer end ease
    helper_set = set(helper) | {tgt.index[n] for n in ('WeaponLeft', 'WeaponRight') if n in tgt.index}
    if args.ease_in > 0:
        # 9. ease-in from the exact start pose (frame 0 before the step limiter), quintic: zero velocity and acceleration
        e = minjerk(s_times / args.ease_in)[:, None]
        nlr[:, nonroot] = G.slerp(np.repeat(start_lr[:, nonroot], T, 0), nlr[:, nonroot], e[:, :, None])
        nlt[:, nonroot] = start_lt[:, nonroot] * (1 - e[:, :, None]) + nlt[:, nonroot] * e[:, :, None]
    if args.end_ease > 0:
        # 10. quintic ease-out into the exact end pose (every bone; helpers and weapon bones get the longer
        # --end-ease-helpers window): last frame = end pose, zero velocity at the end
        for grp, sec in (([j for j in nonroot if j not in helper_set], args.end_ease),
                         ([j for j in nonroot if j in helper_set], max(args.end_ease, args.end_ease_helpers))):
            e = minjerk((s_times - (s_times[-1] - sec)) / sec)[:, None]
            nlr[:, grp] = G.slerp(nlr[:, grp], np.repeat(end_full_lr[:, grp], T, 0), e[:, :, None])
            nlt[:, grp] = nlt[:, grp] * (1 - e[:, :, None]) + end_full_lt[:, grp] * e[:, :, None]
    if args.plant_hold:
        if args.smooth_rot_limit > 0:  # smooth only the frames next to a step over the limit, then lock the feet again
            limit_rot_steps(local=True)
            # re-lock the planted frames only (weight 1 on the plants, fading out within ~5 frames into a swing), so the
            # limited swings stay as they are
            errs = {}
            for k in eff[:2]:
                msk = np.zeros(T)
                for a_, b_ in plant_segs[k]['plants']:
                    msk[a_:b_ + 1] = 1.0
                wpl = np.maximum(msk, np.minimum(1.0, 2.0 * gauss1d(msk, 2.0, 'edge')))
                if args.ease_in > 0:  # the eased first / last frames stay exact
                    wpl = wpl * minjerk(s_times / args.ease_in)
                if args.end_ease > 0:
                    wpl = wpl * (1 - minjerk((s_times - (s_times[-1] - args.end_ease)) / args.end_ease))
                a_, b_ = IK.LIMBS[k]
                errs[k] = IK.two_bone(tgt, nlt, nlr, tgt.index[a_], tgt.index[b_], tgt.index[k], goals[k], weight=np.maximum(wpl, 1e-9),
                                      max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma)
            duck_rep['limb_lock_post_limit_err_m'] = errs
            nlr = S.qcontinuous(nlr)
            # the re-lock can bring back a step at a landing: limit again with the planted leg frames frozen
            frozen = {}
            for k, side in (('LeftFoot', 'Left'), ('RightFoot', 'Right')):
                msk = np.zeros(T, bool)
                for a_, b_ in plant_segs[k]['plants']:
                    msk[a_:b_ + 1] = True
                for bn in LEG_BONES:
                    if bn.startswith(side):
                        frozen[bn] = msk
            frozen['Hips'] = frozen['LeftUpLeg'] | frozen['RightUpLeg']  # the hips carry both planted legs
            limit_rot_steps(local=True, frozen=frozen)
        duck_rep['foot_slide_planted_final_max_m'] = planted_slide(own_plants=True)
        duck_rep['foot_slide_source_planted_final_max_m'] = planted_slide()
    if args.ref_joint_fixed and 'reference_joint' in tgt.index:
        # reference_joint marks a fixed point of the vehicle (constant in world in the vanilla clips): hold the end pose's
        # reference_joint (relative to the final root = the seat) in world for the whole clip
        rj = tgt.index['reference_joint']
        Rr = S.q2m(nlr[:, ri])
        Re = S.q2m(nlr[-1, ri])
        w_t = nlt[-1, ri] + Re @ end_full_lt[0, rj]
        w_r = Re @ S.q2m(end_full_lr[0, rj])
        RrT = np.transpose(Rr, (0, 2, 1))
        nlt[:, rj] = np.einsum('tij,tj->ti', RrT, w_t[None] - nlt[:, ri])
        nlr[:, rj] = S.qcontinuous(S.m2q(RrT @ w_r[None]))
    if args.roof_json and args.roof_final:
        a2 = copy.copy(args)
        a2.roof_margin, a2.roof_lookahead, a2.roof_cap_file = args.roof_final_margin, 0, None
        a2.roof_gate_mode, a2.roof_floor_below_seated = True, args.roof_final_floor
        hi2 = tgt.index['Head']
        Wf, _ = tgt.world(nlt, nlr)
        _, _, _, _, capf = roof_cap(a2, Wf, nlt, nlr, tgt, hi2, ri)
        over_f = np.maximum(0.0, Wf[:, hi2, 1] - capf(Wf[:, hi2]))
        env_f = 1.0 - minjerk((s_times - (s_times[-1] - args.roof_final_fade)) / args.roof_final_fade)
        dlt = np.minimum(np.maximum(IK.smooth(IK.maxfilter(over_f * env_f, 2), 1.5), over_f * env_f) * env_f, args.roof_final_max)
        nlt[:, tgt.index['Hips'], 1] -= dlt
        errs_f = {}
        for k in eff[:2]:  # feet stay where they are in world
            a_, b_ = IK.LIMBS[k]
            errs_f[k] = IK.two_bone(tgt, nlt, nlr, tgt.index[a_], tgt.index[b_], tgt.index[k], Wf[:, tgt.index[k]], weight=np.where(dlt > 1e-4, 1.0, 1e-9),
                                    max_extend=args.ik_max_extend, bend_sigma=args.ik_bend_sigma)
        nlr = S.qcontinuous(nlr)
        duck_rep['roof_final'] = dict(over_before_max_m=float(over_f.max()), drop_max_m=float(dlt.max()), frames=int((dlt > 1e-4).sum()), foot_err_m=errs_f)
    if args.head_duck:
        # neck / upper-spine duck layer on the FINISHED pose (head_duck.py): nothing after this moves the head except the limb IK helpers
        import head_duck as HD
        HD.apply(args, tgt, nlt, nlr, ri, s_times, duck_rep)
    if args.limb_path:
        # round 7: hips path / limb escape layer on the FINISHED pose (limb_path.py); default off
        import limb_path as LP
        LP.apply(args, tgt, nlt, nlr, ri, s_times, duck_rep)
    limb_rep = None
    if args.limb_ik_final:
        # 11. limb IK helpers on the FINAL limbs: every foot / hand edit above (IK, eases, limiter, re-lock) happened after the
        # helpers were placed (step 4); the game's leg / arm IK pulls the limbs onto the helpers, so they must describe the
        # final pose. Vanilla per-frame offsets to the anchor bone, then local transforms of the helpers and their children
        # (whose world transforms stay) are recomputed; other bones unchanged.
        Wt, Wr = tgt.world(nlt, nlr)
        loffs = limb_anchor_offsets(tgt, vWt, vWr)
        # the offsets follow the clip's own eases (no helper motion at the first / last frame beyond its anchor's)
        e_in = minjerk(s_times / args.ease_in) if args.ease_in > 0 else np.ones(T)
        e_out = minjerk((s_times - (s_times[-1] - args.end_ease)) / args.end_ease) if args.end_ease > 0 else np.zeros(T)
        for b, (a, ot, orr) in list(loffs.items()):
            oq = S.qcontinuous(S.m2q(orr))
            ot = ot[:1] + e_in[:, None] * (ot - ot[:1])
            oq = G.slerp(np.repeat(oq[:1], T, 0), oq, e_in[:, None])
            ot = ot + e_out[:, None] * (ot[-1:] - ot)
            oq = G.slerp(oq, np.repeat(oq[-1:], T, 0), e_out[:, None])
            loffs[b] = (a, ot, S.q2m(oq))
        for b, (a, ot, orr) in loffs.items():
            Wr[:, b] = Wr[:, a] @ orr
            Wt[:, b] = Wt[:, a] + np.einsum('tij,tj->ti', Wr[:, a], ot)
        # pole vectors: the IK bends the knee / elbow towards them, so their position is placed in the limb plane frame
        # (origin knee / elbow, u = root -> end, v = bend direction, w = u x v) with the vanilla per-frame coordinates (a bone
        # anchored offset would follow the bone's twist, which the two-bone IK above does not preserve)
        e_in = minjerk(s_times / args.ease_in) if args.ease_in > 0 else np.ones(T)
        e_out = minjerk((s_times - (s_times[-1] - args.end_ease)) / args.end_ease) if args.end_ease > 0 else np.zeros(T)

        def limb_frame(Wp, r, m, e):
            A_, K_, E_ = Wp[:, r], Wp[:, m], Wp[:, e]
            u = (E_ - A_) / np.linalg.norm(E_ - A_, axis=-1, keepdims=True)
            v = (K_ - A_) - np.sum((K_ - A_) * u, -1, keepdims=True) * u
            for i in range(len(v)):
                if np.linalg.norm(v[i]) < 0.01 and i:
                    v[i] = v[i - 1] - np.dot(v[i - 1], u[i]) * u[i]
            v /= np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)
            return K_, u, v, np.cross(u, v)
        pole_rep = {}
        for s_ in ('Left', 'Right'):
            for limb, chain in (('Leg', (s_ + 'UpLeg', s_ + 'Leg', s_ + 'Foot')), ('Arm', (s_ + 'Arm', s_ + 'ForeArm', s_ + 'Hand'))):
                ch = [tgt.index[n] for n in chain]
                vK, vu, vv, vw = limb_frame(vWt, *ch)
                nK, nu, nv, nw = limb_frame(Wt, *ch)
                for pn in ('Torso_%s%s_PoleVector_JNT' % (s_, limb), 'Torso_World_%s%s_PoleVector_JNT' % (s_, limb)):
                    if pn not in tgt.index:
                        continue
                    pb = tgt.index[pn]
                    d = vWt[:, pb] - vK
                    c = np.stack([np.sum(d * vu, -1), np.sum(d * vv, -1), np.sum(d * vw, -1)], -1)
                    # in the bend plane by construction: the vanilla bend direction is undefined while its limb is nearly
                    # straight, so its off-plane part goes into the in-plane distance (distance from the axis kept)
                    c = np.stack([c[:, 0], np.hypot(c[:, 1], c[:, 2]), np.zeros(len(c))], -1)
                    c = c[:1] + e_in[:, None] * (c - c[:1])
                    c = c + e_out[:, None] * (c[-1:] - c)
                    Wt[:, pb] = nK + c[:, :1] * nu + c[:, 1:2] * nv + c[:, 2:] * nw
                    pole_rep[pn] = 'limb plane %s' % chain[1]
        flt, flr = tgt.local_from_world(Wt, Wr)
        upd = set(loffs) | {j for j, pj in enumerate(tgt.parent) if pj in loffs}
        upd = sorted(upd)
        nlt[:, upd] = flt[:, upd]
        nlr[:, upd] = flr[:, upd]
        nlr = S.qcontinuous(nlr)
        limb_rep = {tgt.names[b]: tgt.names[a] for b, (a, ot, orr) in loffs.items()}
        limb_rep.update(pole_rep)
    after = dict(pos_err_m=float(np.linalg.norm(nlt[-1, ri] - end_pos)), yaw_err_deg=float(yaw_of(nlr[-1, ri]) - end_yaw))
    # 7. events
    t_anim = tgt.gj['animations'][t_clip]
    extras = copy.deepcopy(t_anim.get('extras', {}))
    extras['fallbackFrameIndices'] = []
    ls, rs = plant_frames(tgt, nlt, nlr, 'LeftFoot'), plant_frames(tgt, nlt, nlr, 'RightFoot')
    evs = []
    for e in extras.get('animEvents') or []:
        if e['type'] == 'FootPlant':
            continue
        e = dict(e)
        e['startFrame'] = int(round(e['startFrame'] * (T - 1) / max(1, round(t_dur * FPS))))
        evs.append(e)
    tmpl = next((e for e in (extras.get('animEvents') or []) if e['type'] == 'FootPlant'), None)
    if tmpl:
        for side, starts in (('Left', ls), ('Right', rs)):
            for f in starts:
                if f == 0:
                    continue
                e = dict(tmpl)
                e['eventName'] = 'leftFootOnGround' if side == 'Left' else 'rightFootOnGround'
                e['side'] = side
                e['startFrame'] = f
                evs.append(e)
    extras['animEvents'] = sorted(evs, key=lambda e: e['startFrame'])
    if abs(t_dur - s_dur) > 1e-3:
        sc = s_dur / t_dur
        for key in ('trackKeys', 'constTrackKeys'):
            for k in extras.get(key) or []:
                k['time'] = float(k['time']) * sc
    write_out(tgt, args.out, args.name, s_times, nlt, nlr, extras)
    # diagnostics
    Wt_n, Wr_n = tgt.world(nlt, nlr)
    rep = dict(src=str(args.src), src_clip=args.src_clip, mirror=args.mirror, target=str(args.target), target_clip=args.target_clip,
               frames=T, duration_s=float(s_times[-1]), bones_written=len(tgt.names), common_bones=len(common),
               helper_bones=len(helper), root_end_before=before, root_end_after=after, corr_start=args.corr_start,
               end_blend_s=args.end_blend, duck=duck_rep,
               head_peak_m=float(Wt_n[:, tgt.index['Head'], 1].max()),
               head_peak_vanilla_m=float(vWt[:, tgt.index['Head'], 1].max()),
               hips_peak_m=float(Wt_n[:, tgt.index['Hips'], 1].max()),
               hips_peak_vanilla_m=float(vWt[:, tgt.index['Hips'], 1].max()), foot_contact_starts=dict(left=ls, right=rs),
               events=[(e['startFrame'], e['type'], e['eventName']) for e in extras['animEvents']])
    # continuity: max per-frame change of root translation (m/frame) and of any common bone world rotation (deg/frame)
    rv = np.linalg.norm(np.diff(nlt[:, ri], axis=0), axis=-1) * FPS
    rep['root_speed_max_mps'] = float(rv.max())
    rep['root_accel_max_mps2'] = float(np.abs(np.diff(rv)).max() * FPS)
    ang = []
    for b in common:
        j = tgt.index[b]
        R = np.einsum('tji,tjk->tik', Wr_n[:-1, j], Wr_n[1:, j])
        ang.append(np.degrees(np.arccos(np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1))).max())
    rep['max_bone_world_rot_step_deg'] = float(max(ang))
    rep['max_bone_world_rot_step_bone'] = common[int(np.argmax(ang))]
    # end pose vs vanilla last frame (relative to root): world positions of key joints
    keyj = ['Hips', 'Spine3', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot', 'LeftArm', 'RightArm']
    rep['end_pose_joint_err_m'] = {k: float(np.linalg.norm(Wt_n[-1, tgt.index[k]] - vWt[-1, tgt.index[k]])) for k in keyj}
    rep['start_pose_joint_err_m'] = {k: float(np.linalg.norm(Wt_n[0, tgt.index[k]] - vWt[0, tgt.index[k]])) for k in keyj}
    if args.mirror:
        # left/right swap check on the source rig: mirrored LeftHand == M * original RightHand
        olt, olr = src.sample_local(s_clip, s_times)
        oWt, _ = src.world(olt, olr)
        mlt, mlr = mirror_pose(src, olt, S.qcontinuous(olr))
        mWt, _ = src.world(mlt, mlr)
        errs = {}
        for a in ('LeftHand', 'RightFoot', 'LeftShoulder', 'Head'):
            bn = S.mirror_name(a)
            errs[a] = float(np.abs(mWt[:, src.index[a]] - oWt[:, src.index[bn]] @ M.T).max())
        rep['mirror_swap_max_err_m'] = errs
        rep['left_hand_x_mean_src'] = float(oWt[:, src.index['LeftHand'], 0].mean())
        rep['left_hand_x_mean_mirrored'] = float(mWt[:, src.index['LeftHand'], 0].mean())
    rep['limb_ik_helper_anchors'] = limb_rep
    rep['root_yaw_reduction'] = yaw_rep
    rep['hips_path'] = hips_path_rep
    rep['root_replacement'] = root_rep
    rep['root_monotone'] = mono_rep

    def start_in_seat(lt_, lr_):  # where the engine places the clip start relative to the seat (= the root at the end)
        ye = float(yaw_of(lr_[-1, ri]))
        d = G.qrot(qyaw(np.array([-ye])), (lt_[0, ri] - lt_[-1, ri])[None])[0]
        return dict(pos_m=np.round(d, 4).tolist(), yaw_deg=round(float(yaw_of(lr_[0, ri])) - ye, 2))
    rep['start_in_seat_frame'] = dict(new=start_in_seat(nlt, nlr), vanilla=start_in_seat(vlt, vlr))
    rep['out'] = str(args.out)
    rep['out_sha256'] = hashlib.sha256(Path(args.out).read_bytes()).hexdigest()
    return rep


def write_out(tgt, out, name, times, lt, lr, extras):
    gj = copy.deepcopy(tgt.gj)
    binb = bytearray()
    old = tgt.binb
    # keep the skin inverse bind matrices accessor only
    ibm = gj['skins'][0].get('inverseBindMatrices')
    accessors, views = [], []

    def add(arr, typ, minmax=False):
        nonlocal binb
        arr = np.ascontiguousarray(arr, dtype=np.float32)
        while len(binb) % 4:
            binb += b'\0'
        off = len(binb)
        binb += arr.tobytes()
        views.append(dict(buffer=0, byteOffset=off, byteLength=arr.nbytes))
        acc = dict(bufferView=len(views) - 1, componentType=5126, count=int(arr.shape[0]), type=typ)
        if minmax:
            acc['min'] = [float(arr.min())]
            acc['max'] = [float(arr.max())]
        accessors.append(acc)
        return len(accessors) - 1

    if ibm is not None:
        ibm_arr = G.accessor(tgt.gj, old, ibm)
        gj['skins'][0]['inverseBindMatrices'] = add(ibm_arr, 'MAT4')
    tin = add(times.astype(np.float32), 'SCALAR', minmax=True)
    samplers, chans = [], []
    for j, bn in enumerate(tgt.names):
        node = tgt.node_of[bn]
        for path, data, typ in (('translation', lt[:, j], 'VEC3'), ('rotation', lr[:, j], 'VEC4'),
                                ('scale', np.ones((len(times), 3)), 'VEC3')):
            samplers.append(dict(input=tin, output=add(data, typ), interpolation='LINEAR'))
            chans.append(dict(sampler=len(samplers) - 1, target=dict(node=node, path=path)))
    gj['animations'] = [dict(name=name, samplers=samplers, channels=chans, extras=extras)]
    gj['accessors'] = accessors
    gj['bufferViews'] = views
    gj['buffers'] = [dict(byteLength=len(binb))]
    for k in ('meshes',):
        if k in gj:
            raise SystemExit('template has meshes; not supported')
    G.write_glb(str(out), gj, binb)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--src', required=True)
    p.add_argument('--src-clip', default='enter')
    p.add_argument('--target', required=True)
    p.add_argument('--target-clip', default='enter')
    p.add_argument('--out', required=True)
    p.add_argument('--name', default='enter')
    p.add_argument('--mirror', action='store_true')
    p.add_argument('--corr-start', type=float, default=0.2)
    p.add_argument('--corr-end', type=float, default=0.42)
    p.add_argument('--head-cap', choices=['none', 'vanilla', 'vanilla-depth'], default='vanilla-depth')
    p.add_argument('--cap-depth-tol', type=float, default=0.05)
    p.add_argument('--cap-start-depth', type=float, default=0.6,
                   help='m from the start towards the seat; estimate of where the door frame begins to matter')
    p.add_argument('--cap-full-depth', type=float, default=0.8)
    p.add_argument('--hand-follow-min', type=float, default=0.5)
    p.add_argument('--cap-clip', help='glb:clip whose head envelope bounds the head (default: the target clip)')
    p.add_argument('--cap-window', type=float, default=0.2, help='s, max-filter of the vanilla head height')
    p.add_argument('--cap-margin', type=float, default=0.0)
    p.add_argument('--roof-json', help='body-data/<class>.json (extract_door_data.py --body): GEOMETRIC head cap from the car roof (ceiling above the head in the vehicle frame), in addition to the vanilla envelope')
    p.add_argument('--roof-seat', choices=['left', 'right'], default='left', help='seat of the clip (door-data seat transform): left = driver, right = front passenger')
    p.add_argument('--roof-top', type=float, default=0.24, help='m: head sphere top above the Head joint (0.09 centre + 0.10 skull + 0.05 hair)')
    p.add_argument('--roof-lookahead', type=int, default=2, help='frames: the geometric cap is min-filtered over +- this many frames')
    p.add_argument('--roof-margin', type=float, default=0.01, help='m: clearance kept under the ceiling')
    p.add_argument('--roof-floor-below-seated', type=float, default=0.25, help='m: the geometric cap never asks for a head lower than the seated end pose head minus this')
    p.add_argument('--roof-relax-seated', action='store_true', help='raise the geometric cap by the amount the SEATED end pose itself exceeds it (the roof model is inexact: a clip must not penetrate more than the seated pose)')
    p.add_argument('--roof-cap-file', help='json {cap_y: [T]} of roof_cap_iter.py: cap measured at the head positions of the finished previous pass (min with the geometric cap)')
    p.add_argument('--roof-final', action='store_true', help='final roof correction on the FINISHED pose (after the end blend, the eases, the root smoothing and the foot locks): the Hips are lowered per frame by the amount the '
                   'head sphere still exceeds the ceiling at its real position (pure vertical move: the head x / z do not change, so the ceiling at the head is exact), faded out over --roof-final-fade s before the last '
                   'frame (the end pose stays exact); the feet are held in world by two-bone IK')
    p.add_argument('--roof-final-margin', type=float, default=0.003, help='m: slack of the final correction under the gate of the roof phase (penetration <= max(1 cm, seated penetration + 0.5 cm) - this)')
    p.add_argument('--roof-final-max', type=float, default=0.12, help='m: the final correction lowers the Hips by at most this (a deeper dip is not an entry any more; the residual is reported by the roof phase)')
    p.add_argument('--roof-final-floor', type=float, default=0.40, help='m: the final correction never asks for a head lower than the seated head minus this (check (n) bounds the duck anatomically)')
    p.add_argument('--roof-final-fade', type=float, default=0.1, help='s: the final correction fades out over this long before the last frame')
    p.add_argument('--hand-ik', action='store_true', help='keep hands at their world positions while ducking')
    p.add_argument('--end-blend', type=float, default=0.0)
    p.add_argument('--start-out', type=float, default=0.0, help='m: start the entry this far farther from the car side (the start spot is not pinned to the vanilla start)')
    p.add_argument('--out-axis', type=float, help='car-side outward direction (x of the seat frame): -1 driver (left door), +1 passenger (right door)')
    p.add_argument('--start-back', type=float, default=0.0, help='m: start the entry this far farther back (seat-frame +z) than the vanilla start spot')
    p.add_argument('--back-ramp', type=float, nargs=2, metavar=('A', 'B'), help='s: own smoothstep window of the start-back / start-out part of the root correction (default: the common --corr-start / --corr-end window)')
    p.add_argument('--out-ramp', type=float, nargs=2, metavar=('A', 'B'), help='s: own smoothstep window of the start-out part (seat-frame x) of the start shift (default: the start-back window; needs --start-out)')
    p.add_argument('--hand-out', type=float, default=0.0, help='m: hand goals this far farther from the car side (needs --out-axis), see --hand-out-window')
    p.add_argument('--hand-out-window', type=float, nargs=2, default=[0.4, 0.8], metavar=('A', 'B'), help='s: full --hand-out until A, smoothstep back to 0 at B')
    p.add_argument('--hand-out-rise', type=float, nargs=2, default=[0.0, 0.2], metavar=('A', 'B'), help='s: --hand-out is 0 until A and full from B (smoothstep): frame 0 stays the start pose')
    p.add_argument('--hand-out-side', default='both', choices=('both', 'Left', 'Right'), help='which hand (default both)')
    p.add_argument('--hand-follow-shift', type=float, default=0.0, help='with --back-ramp: follow weight of the HANDS for the start-back ramp (1 = the hand goals move with the body shift)')
    p.add_argument('--start-hold', type=float, default=0.0, help='s: hold the source start pose this long, then play the source faster (needs --target-duration)')
    p.add_argument('--hold-ease', type=float, default=0.25, help='fraction of the post-hold time over which the source speed ramps up from 0')
    p.add_argument('--smooth-rot-limit', type=float, default=0.0, help='deg/frame: smooth the bones whose local rotation step exceeds it')
    p.add_argument('--target-duration', action='store_true', help='resample the source onto the target clip duration at 30 fps')
    p.add_argument('--end-pose', help='glb:clip - the end blend goes to frame 0 of this clip (same rig as the target); glb:clip@last = its last frame')
    p.add_argument('--warp-interp', choices=['linear', 'cubic'], default='linear',
                   help='time-warp resampling of the source: linear (keys sampled directly) or Catmull-Rom over its 30 fps frames')
    p.add_argument('--smooth-rot-limit-global', type=float, default=0.0, help='deg/frame: limit of the FIRST (global, whole clip) smoothing pass only; default = --smooth-rot-limit. A tight global limit also reshapes the start pose; the later local passes keep --smooth-rot-limit')
    p.add_argument('--smooth-rot-limit2', type=float, default=0.0, help='deg/frame: limit of an optional SECOND global smoothing pass after the first (see --smooth-keep-first)')
    p.add_argument('--smooth-keep-first', type=float, default=0.0, help='s: the second global pass leaves this many seconds from the clip start as the first pass left them (blend over 0.3 s)')
    p.add_argument('--limit-pad', type=int, default=0, help='replicated frames at both ends in each --smooth-rot-limit pass')
    p.add_argument('--hips-path', type=float, nargs=3, metavar=('EPS', 'DELTA', 'SIGMA'), help='m, m, frames: hold the Hips path along the car-forward axis inside [c - DELTA, c], c = min(running max, seat + EPS) (no lunge past the seat, no rearward slide while sitting down); default off')
    p.add_argument('--hips-path-hands', default=None, metavar='SPEC', help="with --hips-path: share of the hips path translation the hand goals take, 'K' (both hands) or 'LeftHand:K,RightHand:K' ('+' also separates); default off = the follow weight (planted hands keep their world position)")
    p.add_argument('--hips-path-hold', type=float, nargs=3, default=None, metavar=('F0', 'F1', 'RATE'), help='with --hips-path: no cap before frame F0, the Hips held at the frame F0-1 level until F1, then the cap comes down by at most RATE m per frame to the --hips-path cap; default off')
    p.add_argument('--hips-path-fade', type=float, nargs=2, default=None, metavar=('A', 'B'), help='with --hips-path: the layer fades in over clip frames A..B (smoothstep); default 0 6')
    p.add_argument('--hips-path-feet', default=None, metavar='SPEC', help="with --hips-path and --plant-hold: share of the hips path translation in the foot path the plant / swing goals are taken from, 'K' or 'LeftFoot:K,RightFoot:K'; default off = 1 (the feet move with the body)")
    p.add_argument('--hips-path-keep-start', action='store_true', help='with --hips-path and --ease-in: the start pose (frame 0, held during the vanilla slide) is the one of the same build '
                   'WITHOUT the hips path layer (the IK knee bend smoothing otherwise carries the layer into frame 0); default off')
    p.add_argument('--root-yaw-total', type=float, help='deg: scale the root turn so the total root yaw is at most this')
    p.add_argument('--plant-hold', action='store_true',
                   help='foot IK goals from plant keyframes (held exactly, the last one = the end pose foot) and min-jerk swings')
    p.add_argument('--ik-bend-sigma', type=float, help='frames: temporal smoothing of the knee bend direction in the foot IK')
    p.add_argument('--ik-max-extend', type=float, help='two-bone IK reach cap as a fraction of the limb length (keeps a bend)')
    p.add_argument('--replant-raised-by', type=float, help='s: a re-plant step onto a spot > 20 cm higher (the peg) lands at this time')
    p.add_argument('--land-by', type=float, help='s: the last foot swing lands no later than this (with --plant-hold)')
    p.add_argument('--end-blend-legs', type=float, default=0.0, help='s: end blend window of the leg bones (default --end-blend)')
    p.add_argument('--root-smooth', action='store_true', help='replace the root by a smoothed one (world body unchanged)')
    p.add_argument('--root-monotone', type=float, metavar='SIGMA', help='frames: after the root replacement, the car-forward component of the root travels monotonically from the start to the seat (isotonic fit, Gaussian SIGMA, exact endpoints; world body unchanged, only the entity path); default off')
    p.add_argument('--root-monotone-door-json', help='door-data/<class>.json: seat rotation of the vehicle frame for --root-monotone (default identity)')
    p.add_argument('--root-monotone-side', choices=['left', 'right'], default='left', help='seat of the clip in door-data: left = driver, right = front passenger')
    p.add_argument('--root-speed-ratio', type=float, default=0.2)
    p.add_argument('--root-speed-floor-cm', type=float, default=1.0)
    p.add_argument('--ease-in', type=float, default=0.0, help='s: quintic ease-in from the exact start pose')
    p.add_argument('--end-ease', type=float, default=0.0, help='s: quintic ease-out into the exact end pose (all bones)')
    p.add_argument('--end-ease-helpers', type=float, default=0.0, help='s: the same for the helper bones (longer window)')
    p.add_argument('--limb-ik-final', action='store_true',
                   help='place the limb IK helpers (feet / hands / poles) on the final pose with the vanilla anchor offsets')
    p.add_argument('--ref-joint-fixed', action='store_true', help='reference_joint fixed in world at the end pose (seat) value')
    import head_duck as HD
    HD.add_args(p)
    import limb_path as LP
    LP.add_args(p)
    p.add_argument('--report')
    a = p.parse_args()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    rep = build(a)
    s = json.dumps(rep, indent=1, default=float)
    print(s)
    if a.report:
        Path(a.report).write_text(s + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
