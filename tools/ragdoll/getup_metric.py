"""Acceptance metric for the UTPP ragdoll get-up (bodytrace CSV + game log).

Usage:
    python -I tools/ragdoll/getup_metric.py --csv <bodytrace.csv> --log <utpp-*.log> [--log <digest.txt>] --out result.json

Inputs
  --csv   body trace written by the DLL (red4ext/logs/utpp-bodytrace-*.csv; the stage-1 columns, after MM and aim, are used).
          A trimmed copy (header lines kept) works as well as the whole file.
  --log   game log (lines '[YYYY-MM-DD HH:MM:SS.mmm] ... [utpp_ragdoll] release|end seq=N') and/or the harness digest
          ('  HH:MM:SS.mmm MOTION  db=...'). May be given several times; every file is scanned for all event kinds.

Episode: the last 'release' + 'end' pair of the script log lines (--seq N selects another). Window: release - 0.2 s to end + 1.5 s.
Metrics (thresholds are the acceptance gate of the get-up spec, section 6.1):
  M1 floor        every joint of the clamp set + fingers stays above the ground by its radius - 0.01 m after the release.
  M2 hips step    per frame |dHips| <= 0.06 m and |dHipsZ| <= 0.05 m (model frame). A pair longer than one 60 fps frame (a hitch or a
                  dropped trace frame is not a pop) is judged by its speed: step * (1/60) / dt; shorter pairs by the raw step. The raw
                  maxima are reported too (max_step / max_step_z), the gate uses the normalised ones (max_step_norm / max_step_z_norm).
  M3 foot slip    planted toes (z < 0.06 m, speed < 0.25 m/s for >= 3 frames) move at most 0.03 m in XY.
  M4 false air    no MOTION pick of a Jumps / Lands database (or a Jump / Land clip) from the native hand-off line
                  ('[utpp_ragdoll] getup handoff', minus one 0.1 s tick; the end line when there is none) to 1.5 s after 'end'.
  M5 standing     hips height at the 'end' line >= 0.9 x the standing hips height.
  M6 release      continuity at the release (world frame, needs the 'getup begin' line): from the last frame before the release to
                  +0.2 s every frame uses the entity transform AFTER the teleport (the script teleports in the Begin call, so there is no
                  free landing frame as in M2): world hips step <= 0.06 m and no Head / LeftEye / RightEye world orientation turn
                  of more than 30 degrees between two frames. A frame that still shows the game pose or a pose expressed in the old
                  entity frame fails here although M2 (best landing split) passes. n/a (passes, reason given) without the begin line.
Exit code: 0 = all metrics pass (or, with --expect-fail, every listed metric fails), 1 = verdict differs, 2 = bad input.

Model frame = entity frame (+Z up, +Y forward); the CSV carries no entity transform. Ground:
  - the release line says 'getup=utpp' and the log has the '[utpp_ragdoll] getup begin ... pre=(x,y,z) preFwd=(..) feet=(..) fwd=(..)'
    line: M1, M2, M3, M5 are evaluated in the WORLD frame. The entity transform before the teleport (pre) holds up to the landing
    frame, the one after (feet) from there on; the landing frame is the split near the release with the smallest world hips step,
    and that step is reported (a real hips snap shows up as a large step at the landing). Ground = feet z;
  - the same without the begin line: model frame, ground = 0, the teleport is not compensated (M2 / M3 may then trip on it);
  - otherwise the lowest (z - radius) over the 0.3 s before the release, shifted per frame by the entity z change taken from the
    '[night] POS p=x,y,z' log lines (sample and hold; --no-entity-z switches the shift off).
  --ground <metres> overrides both. The model frame jumps by the entity move at a script teleport: use --m2-ignore to skip
  frames around it (seconds relative to the release, e.g. 0.0:0.1) and read M2 'worst' with that in mind.
"""
import argparse
import datetime as dt
import json
import math
import re
import sys

# ------------------------------------------------------------------------------------------------------------------ constants
FPS_WINDOW_PRE_S = 0.2      # window start before the release
WINDOW_POST_S = 1.5         # window end after the 'end' line
GROUND_PRE_S = 0.3          # ground estimate: frames in this span before the release
M1_TOL = 0.01
M2_MAX_STEP = 0.06
M2_MAX_STEP_Z = 0.05
M2_FRAME_S = 1.0 / 60.0     # M2 judges every pair as a 60 fps frame: a longer pair (a hitch, a dropped trace frame) is scaled by M2_FRAME_S / dt
M3_Z_MAX = 0.06
M3_SPEED_MAX = 0.25
M3_MIN_FRAMES = 3
M3_MAX_SLIP = 0.03
M4_WINDOW_S = 1.5
M5_FRACTION = 0.9
M6_WINDOW_S = 0.2           # release continuity: frames from the release to this long after it
M6_MAX_STEP = 0.06
M6_MAX_TURN_DEG = 30.0
M6_ORIENT_BONES = ("Head", "LeftEye", "RightEye")
STAND_HIPS_DEFAULT = 1.0
FINGER_RADIUS = 0.01
# clamp set and radii (m), the same table as the spec 3.4
RADII = {
    "Hips": 0.10, "Spine3": 0.10, "Head": 0.09,
    "LeftArm": 0.05, "RightArm": 0.05, "LeftForeArm": 0.04, "RightForeArm": 0.04,
    "LeftHand": 0.03, "RightHand": 0.03, "LeftLeg": 0.05, "RightLeg": 0.05,
    "LeftFoot": 0.05, "RightFoot": 0.05, "LeftToeBase": 0.02, "RightToeBase": 0.02,
}
FINGER_RE = re.compile(r"^(Left|Right)Hand(Thumb|Index|Middle|Ring|Pinky)[123]$")
TOES = ("LeftToeBase", "RightToeBase")
METRICS = ("M1", "M2", "M3", "M4", "M5", "M6")


def radius_of(name):
    if name in RADII:
        return RADII[name]
    if FINGER_RE.match(name):
        return FINGER_RADIUS
    return None


# ------------------------------------------------------------------------------------------------------------------ quaternions
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx))


def fk_model(local, parents, wanted):
    """Model-space positions of the bones in `wanted` (names -> index in `index_of`) from the local transforms.

    local: gameIndex -> (name, pos, quat); parents: gameIndex -> parent index (-1 = none). The chain of every wanted bone is
    walked to the root with memoisation. A bone whose parent is absent from the frame (and is not -1) is unresolved and omitted
    from the result: a missing ancestor never silently turns a local transform into a model transform.
    Returns (positions: name -> (x, y, z), unresolved: [names]).
    """
    out, _, missing = fk_model_q(local, parents, wanted)
    return out, missing


def fk_model_q(local, parents, wanted):
    """fk_model with the model-space orientation too: returns (positions, quaternions: name -> (x, y, z, w), unresolved)."""
    memo = {}

    def resolve(i):
        if i in memo:
            return memo[i]
        rec = local.get(i)
        if rec is None:
            memo[i] = None
            return None
        p = parents.get(i, -1)
        if p < 0:
            memo[i] = (rec[1], rec[2])
            return memo[i]
        up = resolve(p)
        if up is None:
            memo[i] = None
            return None
        pp, pq = up
        r = qrot(pq, rec[1])
        memo[i] = ((pp[0] + r[0], pp[1] + r[1], pp[2] + r[2]), qmul(pq, rec[2]))
        return memo[i]

    out = {}
    quats = {}
    missing = []
    for i in wanted:
        res = resolve(i)
        if res is None:
            if i in local:
                missing.append(local[i][0])
            continue
        out[local[i][0]] = res[0]
        quats[local[i][0]] = res[1]
    return out, quats, missing


# ------------------------------------------------------------------------------------------------------------------ bodytrace
class Header:
    def __init__(self):
        self.hz = 1e7
        self.arm = None
        self.utc = None
        self.parents = {}


def parse_header(path):
    h = Header()
    with open(path, "r", newline="") as f:
        for line in f:
            if line.startswith("ticks"):
                break
            if not line.startswith("#"):
                break
            m = re.match(r"# qpc_hz=(\d+) arm_ticks=(\d+) utc=(\S+)", line)
            if m:
                h.hz = float(m.group(1))
                h.arm = int(m.group(2))
                h.utc = dt.datetime.strptime(m.group(3), "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=dt.timezone.utc)
            m = re.match(r"# rig_bone index=(\d+) hash=\S+ parent=(-?\d+)", line)
            if m:
                h.parents[int(m.group(1))] = int(m.group(2))
    if h.arm is None or h.utc is None:
        raise ValueError("bodytrace header: '# qpc_hz=... arm_ticks=... utc=...' line missing")
    if not h.parents:
        raise ValueError("bodytrace header: no '# rig_bone' lines")
    return h


def arm_local_seconds_of_day(h, utc_offset_hours):
    if utc_offset_hours is None:
        local = h.utc.astimezone()
    else:
        local = h.utc.astimezone(dt.timezone(dt.timedelta(hours=utc_offset_hours)))
    return local.hour * 3600 + local.minute * 60 + local.second + local.microsecond / 1e6, local.date()


def read_frames(path, h, t_lo, t_hi, stage):
    """Frames with arm-relative time in [t_lo, t_hi]: list of (t, local: gameIndex -> (name, pos, quat)).

    One frame = the rows of one (ticks, desc) group; at equal ticks the group with the most rows wins."""
    base = 8 if stage == 0 else 20
    best = {}
    cur_key = None
    cur = {}
    arm, hz = h.arm, h.hz

    def flush():
        if cur_key is None or not cur:
            return
        tk = cur_key[0]
        prev = best.get(tk)
        if prev is None or len(cur) > len(prev):
            best[tk] = dict(cur)

    with open(path, "r", newline="") as f:
        for line in f:
            if line.startswith("#") or line.startswith("ticks"):
                continue
            comma = line.find(",")
            tk = int(line[:comma])
            t = (tk - arm) / hz
            if t < t_lo:
                continue
            if t > t_hi:
                break
            c = line.rstrip("\r\n").split(",")
            key = (tk, c[1])
            if key != cur_key:
                flush()
                cur_key = key
                cur = {}
            v = [float(x) for x in c[base:base + 12]]
            cur[int(c[7])] = (c[6], (v[0], v[1], v[2]), (v[4], v[5], v[6], v[7]))
        flush()
    return [((tk - arm) / hz, best[tk]) for tk in sorted(best)]


# ------------------------------------------------------------------------------------------------------------------ log
FULL_TS = re.compile(r"^\[(\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2}):(\d{2}(?:\.\d+)?)\]")
DIGEST_TS = re.compile(r"^\s+(\d{2}):(\d{2}):(\d{2}(?:\.\d+)?)\s")
RE_START = re.compile(r"\[utpp_ragdoll\] start seq=(\d+)")
RE_RELEASE = re.compile(r"\[utpp_ragdoll\] release seq=(\d+) why=(.*?) t=([\d.]+)(.*)$")
RE_END = re.compile(r"\[utpp_ragdoll\] end seq=(\d+) why=(.*?) dt=([\d.]+)")
RE_ABORT = re.compile(r"\[utpp_ragdoll\] abort seq=(\d+)")
RE_MMV2 = re.compile(r"\bMotion\b[^\n]*?\bdb=(\d+):(\S+)\s+clip=(\d+):(\S+)")
RE_POS = re.compile(r"\[night\] POS p=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)")
RE_GETUP = re.compile(r"getup=(\w+)(?: clip=(\d+))?(?: rms=([\d.]+))?(?: why=(\S+))?")
_NUM = r"(-?[\d.]+)"
RE_GU_BEGIN = re.compile(r"\[utpp_ragdoll\] getup begin seq=(\d+) .*?pre=\(%s,%s,%s\) preFwd=\(%s,%s,%s\) feet=\(%s,%s,%s\) fwd=\(%s,%s,%s\)"
                         % ((_NUM,) * 12))
RE_GU_HANDOFF = re.compile(r"\[utpp_ragdoll\] getup handoff seq=(\d+)")


def line_seconds_of_day(line):
    m = FULL_TS.match(line)
    if m:
        return int(m.group(4)) * 3600 + int(m.group(5)) * 60 + float(m.group(6))
    m = DIGEST_TS.match(line)
    if m:
        return int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
    return None


def parse_log(paths):
    """Events with seconds-of-day times (local clock): dict(episodes, mm, pos)."""
    episodes = {}
    mm = []
    pos = []
    seen_mm = set()
    seen_pos = set()
    for path in paths:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                s = line_seconds_of_day(line)
                if s is None:
                    continue
                m = RE_START.search(line)
                if m:
                    episodes.setdefault(int(m.group(1)), {})["start"] = s
                    continue
                m = RE_RELEASE.search(line)
                if m:
                    e = episodes.setdefault(int(m.group(1)), {})
                    e["release"] = s
                    e["release_why"] = m.group(2)
                    g = RE_GETUP.search(m.group(4))
                    if g:
                        e["getup"] = g.group(1)
                        e["getup_clip"] = int(g.group(2)) if g.group(2) else None
                        e["getup_rms"] = float(g.group(3)) if g.group(3) else None
                        e["getup_why"] = g.group(4)
                    continue
                m = RE_END.search(line)
                if m:
                    e = episodes.setdefault(int(m.group(1)), {})
                    e["end"] = s
                    e["end_why"] = m.group(2)
                    continue
                m = RE_ABORT.search(line)
                if m:
                    episodes.setdefault(int(m.group(1)), {})["abort"] = s
                    continue
                m = RE_GU_BEGIN.search(line)
                if m:
                    v = [float(x) for x in m.groups()[1:]]
                    e = episodes.setdefault(int(m.group(1)), {})
                    e["gu_begin_s"] = s
                    e["gu_begin"] = {"pre": v[0:3], "pre_fwd": v[3:5], "feet": v[6:9], "fwd": v[9:11]}
                    continue
                m = RE_GU_HANDOFF.search(line)
                if m:
                    episodes.setdefault(int(m.group(1)), {}).setdefault("handoff", s)
                    continue
                m = RE_MMV2.search(line)
                if m:
                    key = (round(s, 3), m.group(1), m.group(3))
                    if key not in seen_mm:
                        seen_mm.add(key)
                        mm.append({"s": s, "db": m.group(2), "clip": m.group(4)})
                    continue
                m = RE_POS.search(line)
                if m:
                    key = round(s, 3)
                    if key not in seen_pos:
                        seen_pos.add(key)
                        pos.append((s, float(m.group(1)), float(m.group(2)), float(m.group(3))))
    mm.sort(key=lambda x: x["s"])
    pos.sort()
    return {"episodes": episodes, "mm": mm, "pos": pos}


def pick_episode(events, seq):
    eps = events["episodes"]
    if seq is not None:
        if seq not in eps:
            raise ValueError("episode seq=%d not found in the log" % seq)
        return seq, eps[seq]
    complete = [k for k in sorted(eps) if "release" in eps[k] and "end" in eps[k]]
    if complete:
        return complete[-1], eps[complete[-1]]
    released = [k for k in sorted(eps) if "release" in eps[k]]
    if released:
        return released[-1], eps[released[-1]]
    raise ValueError("no '[utpp_ragdoll] release seq=' line in the log")


# ------------------------------------------------------------------------------------------------------------------ analysis
def unit_fwd(fx, fy):
    n = math.hypot(fx, fy)
    return (0.0, 1.0) if n < 1e-6 else (fx / n, fy / n)


def to_world(p, ent):
    """Model point -> world. ent = (x, y, z, fx, fy): entity position and unit forward; model +Y is forward, right = (fy, -fx)."""
    return (ent[0] + p[0] * ent[4] + p[1] * ent[3], ent[1] - p[0] * ent[3] + p[1] * ent[4], ent[2] + p[2])


def yaw_of(ent):
    """Entity yaw (rad) of the unit forward (ent[3], ent[4]): model +Y -> world (fx, fy) = (-sin yaw, cos yaw)."""
    return math.atan2(-ent[3], ent[4])


def world_quat(q, ent):
    """Model-space orientation -> world orientation of an entity transform (rotation about +Z by its yaw)."""
    h = 0.5 * yaw_of(ent)
    return qmul((0.0, 0.0, math.sin(h), math.cos(h)), q)


def turn_deg(a, b):
    """Angle (degrees) of the rotation between two unit quaternions."""
    d = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def find_landing(frames, t_lo, t_hi, e0, e1):
    """The script teleports the entity once. The trace is in the entity frame, so the transform changes between two frames.
    Returns (hips_step, frame_index) of the split (frames before it use e0, the others e1) with the smallest world hips step."""
    best = None
    for k in range(1, len(frames)):
        f0, f1 = frames[k - 1], frames[k]
        if not (t_lo <= f1["t"] <= t_hi) or "Hips" not in f0["p"] or "Hips" not in f1["p"]:
            continue
        a = to_world(f0["p"]["Hips"], e0)
        b = to_world(f1["p"]["Hips"], e1)
        d = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        if best is None or d < best[0]:
            best = (d, k)
    return best


def hold(samples, t):
    """Latest sample with time <= t (samples sorted by time), else the first one; None for an empty list."""
    if not samples:
        return None
    best = samples[0]
    for s in samples:
        if s[0] <= t:
            best = s
        else:
            break
    return best


def analyse(frames_local, header, ev, ep, args):
    """frames_local: [(t, local)] covering at least the pre-release span and the window. Times are arm-relative seconds."""
    day0 = ev["day0"]  # seconds of day at arm (local)

    def rel(s):
        d = s - day0
        if d < -43200:
            d += 86400
        if d > 43200:
            d -= 86400
        return d

    t_rel = rel(ep["release"])
    t_end = rel(ep["end"]) if "end" in ep else None
    t_start = rel(ep["start"]) if "start" in ep else None
    win_lo = t_rel - FPS_WINDOW_PRE_S
    win_hi = (t_end + WINDOW_POST_S) if t_end is not None else t_rel + 6.0
    parents = header.parents

    # model-space positions of the needed bones per frame
    name_idx = {}
    for _, local in frames_local[:50]:
        for i, rec in local.items():
            name_idx[rec[0]] = i
    wanted_names = [n for n in name_idx if radius_of(n) is not None or n in M6_ORIENT_BONES]
    wanted_idx = [name_idx[n] for n in wanted_names]
    frames = []
    incomplete = 0
    for t, local in frames_local:
        idx = [i for i in wanted_idx if i in local]
        pos, quats, missing = fk_model_q(local, parents, idx)
        if missing:
            incomplete += 1
        frames.append({"t": t, "p": pos, "q": quats, "missing": missing})
    if not frames:
        raise ValueError("no body trace frames in the window")
    pre = [f for f in frames if t_rel - GROUND_PRE_S <= f["t"] < t_rel]
    inwin = [f for f in frames if win_lo <= f["t"] <= win_hi]
    post = [f for f in inwin if f["t"] >= t_rel]

    # entity z track from '[night] POS' lines (arm-relative)
    pos_track = [(rel(s), x, y, z) for (s, x, y, z) in ev["pos"]]
    z_ref = None
    if pos_track:
        before = [p for p in pos_track if p[0] <= t_rel]
        z_ref = (before[-1] if before else pos_track[0])[3]

    getup_utpp = ep.get("getup") == "utpp"
    begin = ep.get("gu_begin") if getup_utpp else None
    wmode = begin is not None   # world frame: the entity transform before / after the script teleport comes from the 'getup begin' line
    ground_info = {"mode": None, "base": 0.0, "pre_frames": len(pre)}
    if args.ground is not None:
        base = args.ground
        ground_info["mode"] = "fixed"
        use_entity_z = False
    elif wmode:
        base = begin["feet"][2]
        ground_info["mode"] = "getup-world"
        use_entity_z = False
    elif getup_utpp:
        base = 0.0
        ground_info["mode"] = "getup-grounded (no 'getup begin' line: model frame, teleport not compensated)"
        use_entity_z = False
    else:
        vals = []
        for f in pre:
            for n, p in f["p"].items():
                r = radius_of(n)
                if r is not None:
                    vals.append(p[2] - r)
        if vals:
            base = min(vals)
            ground_info["mode"] = "pre-release"
        else:
            base = 0.0
            ground_info["mode"] = "zero (no pre-release frames)"
        use_entity_z = (not args.no_entity_z) and z_ref is not None
    ground_info["base"] = base
    ground_info["entity_z_shift"] = bool(use_entity_z)

    def ground_at(t):
        g = base
        if use_entity_z:
            s = hold(pos_track, t)
            if s is not None:
                g += z_ref - s[3]
        return g

    result = {"window": {"release_s": t_rel, "end_s": t_end, "lo_s": win_lo, "hi_s": win_hi,
                         "frames": len(inwin), "frames_after_release": len(post), "incomplete_frames": incomplete},
              "ground": ground_info, "metrics": {}}

    # world positions of the needed bones: the model frame itself, or (get-up with a 'getup begin' line) the world frame of the
    # entity before the teleport for the frames up to the landing frame and of the teleported entity after it
    landing = None
    if wmode:
        e0 = tuple(begin["pre"]) + unit_fwd(*begin["pre_fwd"])
        e1 = tuple(begin["feet"]) + unit_fwd(*begin["fwd"])
        landing = find_landing(frames, t_rel - 0.1, t_rel + 0.5, e0, e1)
        if landing is None:
            k_land = next((i for i, f in enumerate(frames) if f["t"] >= t_rel), len(frames))
        else:
            k_land = landing[1]
        for i, f in enumerate(frames):
            ent = e0 if i < k_land else e1
            f["w"] = {n: to_world(p, ent) for n, p in f["p"].items()}
        ground_info["landing_t_rel_release"] = frames[k_land]["t"] - t_rel if k_land < len(frames) else None
        ground_info["landing_hips_step"] = landing[0] if landing else None
        ground_info["entity_before"] = {"pos": begin["pre"], "fwd": e0[3:]}
        ground_info["entity_after"] = {"pos": begin["feet"], "fwd": e1[3:]}
    else:
        for f in frames:
            f["w"] = f["p"]

    ignore = []
    for spec in (args.m2_ignore or []):
        a, b = spec.split(":")
        ignore.append((t_rel + float(a), t_rel + float(b)))

    # ---- M1 floor
    worst = None
    for f in post:
        g = ground_at(f["t"])
        for n, p in f["w"].items():
            r = radius_of(n)
            if r is None:
                continue
            margin = p[2] - g - r
            if worst is None or margin < worst[0]:
                worst = (margin, n, f["t"], p[2] - g)
    if worst is None:
        result["metrics"]["M1"] = {"pass": False, "reason": "no frames after the release"}
    else:
        result["metrics"]["M1"] = {
            "pass": worst[0] >= -M1_TOL, "value": worst[0], "threshold": -M1_TOL, "bone": worst[1],
            "t_rel_release": worst[2] - t_rel, "height_above_ground": worst[3],
            "note": "value = lowest (z - ground - radius); bone height is the joint z above the ground"}

    # ---- M2 hips continuity
    hip = [(f["t"], f["w"]["Hips"]) for f in inwin if "Hips" in f["w"]]
    m2 = {"max_step": 0.0, "max_step_z": 0.0, "t_rel_release": None, "t_rel_release_z": None, "pairs": max(len(hip) - 1, 0),
          "dt_s": None, "speed_ms": None, "max_dt_s": 0.0,  # dt_s / speed_ms: the pair of the raw max_step; max_dt_s: the longest pair
          "max_step_norm": 0.0, "max_step_z_norm": 0.0, "t_rel_release_norm": None, "t_rel_release_z_norm": None,  # the gate (60 fps normalised)
          "dt_norm_s": None}
    for (t0, a), (t1, b) in zip(hip, hip[1:]):
        if any(lo <= t1 <= hi for lo, hi in ignore):
            continue
        d = math.sqrt(sum((b[k] - a[k]) ** 2 for k in range(3)))
        dz = abs(b[2] - a[2])
        m2["max_dt_s"] = max(m2["max_dt_s"], t1 - t0)
        scale = min(1.0, M2_FRAME_S / max(t1 - t0, 1e-4))
        if d * scale > m2["max_step_norm"]:
            m2["max_step_norm"], m2["t_rel_release_norm"], m2["dt_norm_s"] = d * scale, t1 - t_rel, t1 - t0
        if dz * scale > m2["max_step_z_norm"]:
            m2["max_step_z_norm"], m2["t_rel_release_z_norm"] = dz * scale, t1 - t_rel
        if d > m2["max_step"]:
            m2["max_step"], m2["t_rel_release"] = d, t1 - t_rel
            m2["dt_s"], m2["speed_ms"] = t1 - t0, d / max(t1 - t0, 1e-4)
        if dz > m2["max_step_z"]:
            m2["max_step_z"], m2["t_rel_release_z"] = dz, t1 - t_rel
    m2["pass"] = bool(hip) and m2["max_step_norm"] <= M2_MAX_STEP and m2["max_step_z_norm"] <= M2_MAX_STEP_Z
    m2["threshold"] = {"step": M2_MAX_STEP, "step_z": M2_MAX_STEP_Z}
    m2["frame"] = "world (entity transform before / after the teleport)" if wmode else "model (entity frame)"
    if wmode:
        m2["landing"] = {"t_rel_release": ground_info.get("landing_t_rel_release"), "hips_step": ground_info.get("landing_hips_step")}
    result["metrics"]["M2"] = m2

    # ---- M3 planted toes
    xy_shift = (0.0, 0.0)
    moving_entity = False
    if pos_track:
        xs = [p[1] for p in pos_track if win_lo <= p[0] <= win_hi]
        ys = [p[2] for p in pos_track if win_lo <= p[0] <= win_hi]
        moving_entity = (not wmode) and bool(xs) and (max(xs) - min(xs) > 0.05 or max(ys) - min(ys) > 0.05)
    intervals = []
    t_hand = rel(ep["handoff"]) if "handoff" in ep else None
    for toe in TOES:
        series = []
        for f in inwin:
            if toe not in f["w"]:
                continue
            p = f["w"][toe]
            w = (p[0], p[1])
            if moving_entity:
                s = hold(pos_track, f["t"])
                w = (p[0] + s[1], p[1] + s[2])
            series.append((f["t"], w, p[2] - ground_at(f["t"])))
        run = []
        for k in range(len(series)):
            t, w, z = series[k]
            speed = None
            if k > 0:
                t0, w0, z0 = series[k - 1]
                dtt = max(t - t0, 1e-4)
                speed = math.sqrt((w[0] - w0[0]) ** 2 + (w[1] - w0[1]) ** 2 + (z - z0) ** 2) / dtt
            planted = z < M3_Z_MAX and speed is not None and speed < M3_SPEED_MAX
            if planted:
                run.append(series[k])
            if (not planted or k == len(series) - 1) and run:
                if len(run) >= M3_MIN_FRAMES:
                    w0 = run[0][1]
                    slip = max(math.hypot(r[1][0] - w0[0], r[1][1] - w0[1]) for r in run)
                    iv = {"bone": toe, "t_rel_release": run[0][0] - t_rel, "frames": len(run), "slip": slip}
                    # informational: where the planted interval starts relative to the native hand-off / the end line (the stance change into MM)
                    if t_hand is not None:
                        iv["t_rel_handoff"] = run[0][0] - t_hand
                    if t_end is not None:
                        iv["t_rel_end"] = run[0][0] - t_end
                    intervals.append(iv)
                run = []
    worst_slip = max([i["slip"] for i in intervals], default=0.0)
    result["metrics"]["M3"] = {"pass": worst_slip <= M3_MAX_SLIP, "value": worst_slip, "threshold": M3_MAX_SLIP,
                               "intervals": len(intervals), "worst": max(intervals, key=lambda i: i["slip"]) if intervals else None,
                               "entity_xy_added": moving_entity}

    # ---- M4 false air
    if t_end is None:
        result["metrics"]["M4"] = {"pass": False, "reason": "no '[utpp_ragdoll] end' line for this episode"}
    else:
        # Motion matching resumes at the native hand-off, which is logged before the 'end' line (blend-out + one tick): the window
        # starts a tick before the hand-off line when there is one, else at the end line. Picks between the release and that
        # start are listed for information only (the get-up pose covers them).
        w_lo = t_end
        if "handoff" in ep:
            w_lo = min(t_end, rel(ep["handoff"]) - 0.1)
        bad = []
        early = []
        for m in ev["mm"]:
            tt = rel(m["s"])
            air = re.search(r"Jump|Land", m["db"]) or re.search(r"Jump|Land", m["clip"])
            if not air:
                continue
            if w_lo <= tt <= t_end + M4_WINDOW_S:
                bad.append({"t_after_end": tt - t_end, "db": m["db"], "clip": m["clip"]})
            elif t_rel <= tt < w_lo:
                early.append({"t_after_release": tt - t_rel, "db": m["db"], "clip": m["clip"]})
        result["metrics"]["M4"] = {"pass": not bad, "picks": bad, "window_s": M4_WINDOW_S, "window_start_after_end": w_lo - t_end,
                                   "picks_before_window": early}

    # ---- M5 standing at hand-off
    stand = args.stand_hips
    stand_src = "argument"
    if stand is None:
        ref = []
        if t_start is not None:
            ref = [f["p"]["Hips"][2] for f in frames if "Hips" in f["p"] and t_start - 1.2 <= f["t"] <= t_start - 0.1]
        if ref:
            ref.sort()
            stand = ref[len(ref) // 2]
            stand_src = "median hips z 1.2..0.1 s before the hit (%d frames)" % len(ref)
        else:
            stand = STAND_HIPS_DEFAULT
            stand_src = "default %.2f m (no pre-hit frames)" % STAND_HIPS_DEFAULT
    if t_end is None:
        result["metrics"]["M5"] = {"pass": False, "reason": "no '[utpp_ragdoll] end' line for this episode"}
    else:
        cand = [f for f in frames if "Hips" in f["w"] and f["t"] <= t_end]
        if not cand:
            result["metrics"]["M5"] = {"pass": False, "reason": "no hips frame at the end line"}
        else:
            f = cand[-1]
            h = f["w"]["Hips"][2] - ground_at(f["t"])
            result["metrics"]["M5"] = {"pass": h >= M5_FRACTION * stand, "value": h, "threshold": M5_FRACTION * stand,
                                       "standing_hips": stand, "standing_source": stand_src,
                                       "frame_age_s": t_end - f["t"]}
    # ---- M6 release continuity
    if not wmode:
        result["metrics"]["M6"] = {"pass": True, "na": "no 'getup begin' line (or not a native get-up): the entity frames before / after the "
                                   "teleport are unknown"}
    else:
        # the script teleports in the Begin call, which logs before the release line: the frames after it are in the new entity frame
        t_cut = min(t_rel, rel(ep["gu_begin_s"])) if "gu_begin_s" in ep else t_rel
        w6 = [f for f in frames if "Hips" in f["p"]]
        prev_k = max((i for i, f in enumerate(w6) if f["t"] <= t_cut), default=None)
        m6 = {"max_step": 0.0, "t_rel_release": None, "max_turn_deg": 0.0, "turn_bone": None, "t_rel_release_turn": None, "pairs": 0,
              "threshold": {"step": M6_MAX_STEP, "turn_deg": M6_MAX_TURN_DEG}, "window_s": M6_WINDOW_S,
              "frame": "world, entity after the teleport for every frame after the release"}
        seen_bones = set()
        if prev_k is None:
            m6["pass"] = False
            m6["reason"] = "no trace frame at or before the release"
        else:
            def ent_of(f):
                return e0 if f["t"] <= t_cut else e1
            for a, b in zip(w6[prev_k:], w6[prev_k + 1:]):
                if b["t"] > t_cut + M6_WINDOW_S:
                    break
                m6["pairs"] += 1
                pa = to_world(a["p"]["Hips"], ent_of(a))
                pb = to_world(b["p"]["Hips"], ent_of(b))
                d = math.sqrt(sum((pb[k] - pa[k]) ** 2 for k in range(3)))
                if d > m6["max_step"]:
                    m6["max_step"], m6["t_rel_release"] = d, b["t"] - t_rel
                for bone in M6_ORIENT_BONES:
                    qa, qb = a["q"].get(bone), b["q"].get(bone)
                    if qa is None or qb is None:
                        continue
                    seen_bones.add(bone)
                    ang = turn_deg(world_quat(qa, ent_of(a)), world_quat(qb, ent_of(b)))
                    if ang > m6["max_turn_deg"]:
                        m6["max_turn_deg"], m6["turn_bone"], m6["t_rel_release_turn"] = ang, bone, b["t"] - t_rel
            m6["orientation_bones"] = sorted(seen_bones)
            m6["orientation_bones_missing"] = [b for b in M6_ORIENT_BONES if b not in seen_bones]
            if m6["pairs"] == 0:
                m6["pass"] = False
                m6["reason"] = "no trace frame within %.1f s after the release" % M6_WINDOW_S
            else:
                m6["pass"] = m6["max_step"] <= M6_MAX_STEP and m6["max_turn_deg"] <= M6_MAX_TURN_DEG
                if not seen_bones:
                    m6["orientation"] = "n/a: no Head / LeftEye / RightEye rows in the window (only the hips step was checked)"
        result["metrics"]["M6"] = m6
    result["overall_pass"] = all(result["metrics"][m]["pass"] for m in METRICS)
    return result


def summary_lines(res, ep, seq):
    m = res["metrics"]
    out = []
    w = res["window"]
    end_txt = "%.3f" % w["end_s"] if w["end_s"] is not None else "n/a"
    out.append("episode seq=%s release=%s end=%s (arm-relative s), window %.3f..%.3f, %d frames (%d after release, %d incomplete)"
               % (seq, "%.3f" % w["release_s"], end_txt, w["lo_s"], w["hi_s"], w["frames"], w["frames_after_release"],
                  w["incomplete_frames"]))
    out.append("release: why=%s getup=%s%s" % (ep.get("release_why"), ep.get("getup", "(none logged)"),
                                              (" clip=%s rms=%s" % (ep.get("getup_clip"), ep.get("getup_rms")))
                                              if ep.get("getup") == "utpp" else
                                              (" why=%s" % ep.get("getup_why") if ep.get("getup_why") else "")))
    g = res["ground"]
    out.append("ground: %s base=%.3f entity-z-shift=%s" % (g["mode"], g["base"], g["entity_z_shift"]))
    if g.get("landing_hips_step") is not None:
        out.append("teleport landing: %.3f s after the release, world hips step %.3f m (entity moved %s -> %s)"
                   % (g["landing_t_rel_release"], g["landing_hips_step"], g["entity_before"]["pos"], g["entity_after"]["pos"]))

    def verdict(k):
        return "PASS" if m[k]["pass"] else "FAIL"
    x = m["M1"]
    out.append("M1 floor        %s  %s" % (verdict("M1"), ("lowest margin %.3f m (%s, %.2f s after release, joint %.3f m above ground)"
               % (x["value"], x["bone"], x["t_rel_release"], x["height_above_ground"])) if "value" in x else x.get("reason")))
    x = m["M2"]
    out.append("M2 hips step    %s  max |dHips| %.3f m/60fps-frame (at %s s), max |dHipsZ| %.3f m (at %s s) over %d pairs; raw %.3f / %.3f m"
               % (verdict("M2"), x["max_step_norm"], "%.3f" % x["t_rel_release_norm"] if x["t_rel_release_norm"] is not None else "-",
                  x["max_step_z_norm"], "%.3f" % x["t_rel_release_z_norm"] if x["t_rel_release_z_norm"] is not None else "-", x["pairs"],
                  x["max_step"], x["max_step_z"]))
    if x.get("dt_s") is not None:
        out.append("                      (largest step: pair of %.1f ms = %.2f m/s; longest pair in the window %.1f ms)" % (
            x["dt_s"] * 1000.0, x["speed_ms"], x["max_dt_s"] * 1000.0))
    x = m["M3"]
    out.append("M3 foot slip    %s  worst %.3f m over %d planted interval(s)" % (verdict("M3"), x["value"], x["intervals"]))
    if x.get("worst") and "t_rel_handoff" in x["worst"]:
        out.append("                      (worst interval %s starts %+.3f s from the hand-off%s)" % (
            x["worst"]["bone"], x["worst"]["t_rel_handoff"], (", %+.3f s from the end line" % x["worst"]["t_rel_end"]) if "t_rel_end" in x["worst"] else ""))
    x = m["M4"]
    out.append("M4 false air    %s  %s" % (verdict("M4"), x.get("reason") or (
        ("no Jumps / Lands pick from %.2f s after the end line" % x["window_start_after_end"]) if not x["picks"] else
        "; ".join("%s clip=%s %+.3f s" % (p["db"], p["clip"], p["t_after_end"]) for p in x["picks"]))))
    if x.get("picks_before_window"):
        out.append("                      (not counted, before the window: %s)" % "; ".join(
            "%s %+.3f s after release" % (p["db"], p["t_after_release"]) for p in x["picks_before_window"]))
    x = m["M5"]
    out.append("M5 standing     %s  %s" % (verdict("M5"), ("hips %.3f m vs %.3f m needed (standing %.3f m: %s)"
               % (x["value"], x["threshold"], x["standing_hips"], x["standing_source"])) if "value" in x else x.get("reason")))
    x = m["M6"]
    if "na" in x:
        out.append("M6 release      PASS  n/a: %s" % x["na"])
    elif "reason" in x:
        out.append("M6 release      FAIL  %s" % x["reason"])
    else:
        out.append("M6 release      %s  max world |dHips| %.3f m (at %s s), max Head / Eye turn %.1f deg (%s at %s s) over %d pairs%s"
                   % (verdict("M6"), x["max_step"], "%.3f" % x["t_rel_release"] if x["t_rel_release"] is not None else "-", x["max_turn_deg"],
                      x["turn_bone"] or "-", "%.3f" % x["t_rel_release_turn"] if x["t_rel_release_turn"] is not None else "-", x["pairs"],
                      (" [%s]" % x["orientation"]) if "orientation" in x else
                      (" [no rows for %s]" % ",".join(x["orientation_bones_missing"]) if x.get("orientation_bones_missing") else "")))
    out.append("overall: %s" % ("PASS" if res["overall_pass"] else "FAIL (%s)" % ", ".join(k for k in METRICS if not m[k]["pass"])))
    return out


def run(args):
    header = parse_header(args.csv)
    events = parse_log(args.log)
    day0, _ = arm_local_seconds_of_day(header, args.utc_offset_hours)
    events["day0"] = day0
    seq, ep = pick_episode(events, args.seq)
    if "release" not in ep:
        raise ValueError("episode seq=%s has no release line" % seq)

    def rel(s):
        d = s - day0
        if d < -43200:
            d += 86400
        if d > 43200:
            d -= 86400
        return d

    t_rel = rel(ep["release"])
    lo = t_rel - 1.0
    if "start" in ep:
        lo = min(lo, rel(ep["start"]) - 1.3)
    hi = (rel(ep["end"]) if "end" in ep else t_rel + 6.0) + WINDOW_POST_S + 0.1
    frames = read_frames(args.csv, header, lo, hi, args.stage)
    res = analyse(frames, header, events, ep, args)
    res["episode"] = {"seq": seq, **{k: v for k, v in ep.items()}}
    res["inputs"] = {"csv": args.csv, "logs": list(args.log), "stage": args.stage, "trace_window_s": [lo, hi]}
    return res, ep, seq


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", required=True, help="bodytrace CSV")
    ap.add_argument("--log", action="append", required=True, help="game log or harness digest (repeatable)")
    ap.add_argument("--out", help="write the result as JSON")
    ap.add_argument("--seq", type=int, help="episode sequence number (default: the last complete release/end pair)")
    ap.add_argument("--stage", type=int, default=1, choices=(0, 1), help="CSV stage: 0 before MM, 1 final (default)")
    ap.add_argument("--ground", type=float, help="model-frame ground height in metres (overrides the automatic choice)")
    ap.add_argument("--no-entity-z", action="store_true", help="do not shift the ground by the entity z of the POS log lines")
    ap.add_argument("--stand-hips", type=float, help="standing hips height (default: measured before the hit, else 1.0)")
    ap.add_argument("--utc-offset-hours", type=float, help="local clock offset of the log (default: this machine's zone at the CSV time)")
    ap.add_argument("--m2-ignore", action="append", metavar="LO:HI",
                    help="skip M2 frame pairs ending within LO..HI seconds after the release (e.g. the script teleport frame)")
    ap.add_argument("--expect-fail", nargs="?", const="M1,M2,M4", metavar="LIST",
                    help="exit 0 only when every listed metric fails (default list M1,M2,M4: the r6a fixture)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    try:
        res, ep, seq = run(args)
    except (OSError, ValueError) as e:
        print("getup_metric: %s" % e, file=sys.stderr)
        return 2
    lines = summary_lines(res, ep, seq)
    if not args.quiet:
        print("\n".join(lines))
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=1, sort_keys=True)
            f.write("\n")
    if args.expect_fail is not None:
        want = {x.strip() for x in args.expect_fail.split(",") if x.strip()}
        got = {k for k in METRICS if not res["metrics"][k]["pass"]}
        ok = want <= got
        if not args.quiet:
            print("expect-fail %s: failing=%s -> %s" % (",".join(sorted(want)), ",".join(sorted(got)) or "none", "as expected" if ok else "MISMATCH"))
        return 0 if ok else 1
    return 0 if res["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
