"""Offline tests of tools/ragdoll/getup_metric.py: synthetic traces (a pass and one failure per metric), the forward kinematics,
and, when the research logs exist, the r6a car-hit fixture (must fail M1, M2, M4) and the r13b release fixture (must fail M6).

    python -I tools/ragdoll/test_getup_metric.py
"""
import json
import math
import os
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import getup_metric as gm  # noqa: E402

PASSED = 0
FAILED = 0
SKIPPED = 0


def check(name, cond, extra=""):
    global PASSED, FAILED
    if cond:
        PASSED += 1
        print("PASS: " + name)
    else:
        FAILED += 1
        print("FAIL: %s %s" % (name, extra))


def skip(name, why):
    global SKIPPED
    SKIPPED += 1
    print("SKIP: %s (%s)" % (name, why))


# ------------------------------------------------------------------------------------------------------ synthetic trace
BONES = {  # name -> game index; every bone is a direct child of Root (index 0), identity rotations: local = model
    "Root": 0, "Hips": 2, "Spine3": 13, "Head": 22, "LeftArm": 17, "RightArm": 18, "LeftForeArm": 20, "RightForeArm": 21,
    "LeftHand": 23, "RightHand": 24, "LeftLeg": 8, "RightLeg": 9, "LeftFoot": 11, "RightFoot": 12,
    "LeftToeBase": 43, "RightToeBase": 44, "LeftHandIndex1": 46, "LeftEye": 31, "RightEye": 32,
}
HZ = 10000000
ARM = 1000000000
T_START, T_RELEASE, T_END = 10.0, 12.0, 15.0  # seconds after the arm; the log clock is 13:00:00 at the arm (UTC+1)
STAND = {  # standing model positions
    "Hips": (0, 0, 1.0), "Spine3": (0, 0, 1.3), "Head": (0, 0, 1.6), "LeftArm": (-0.2, 0, 1.4), "RightArm": (0.2, 0, 1.4),
    "LeftForeArm": (-0.3, 0, 1.2), "RightForeArm": (0.3, 0, 1.2), "LeftHand": (-0.3, 0, 1.0), "RightHand": (0.3, 0, 1.0),
    "LeftLeg": (-0.1, 0, 0.5), "RightLeg": (0.1, 0, 0.5), "LeftFoot": (-0.1, 0, 0.1), "RightFoot": (0.1, 0, 0.1),
    "LeftToeBase": (-0.1, 0.1, 0.03), "RightToeBase": (0.1, 0.1, 0.03), "LeftHandIndex1": (-0.3, 0.05, 1.0),
}


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def lying(name):
    x, y, z = STAND[name]
    r = gm.radius_of(name)
    return (x, y, r + 0.005)  # every joint rests on the floor (z = radius)


def good_pose(t):
    """Stand until the hit, lie from 10.3 s, rise from the release to 14.8 s (toes stay planted)."""
    if t < T_START + 0.3:
        w = 1.0
    elif t < T_RELEASE:
        w = 0.0
    else:
        w = smoothstep((t - T_RELEASE) / (14.8 - T_RELEASE))
    pose = {}
    for n in STAND:
        a, b = lying(n), STAND[n]
        pose[n] = tuple(a[k] + (b[k] - a[k]) * w for k in range(3))
    for toe in gm.TOES:
        pose[toe] = STAND[toe]
    return pose


def write_csv(path, pose_fn, t_lo=8.0, t_hi=17.5, fps=60, quat_fn=None, gaps=()):
    lines = ["# qpc_hz=%d arm_ticks=%d utc=2026-11-04T12:00:00.000Z max_frames=0 continuous=1" % (HZ, ARM)]
    for n, i in BONES.items():
        lines.append("# rig_bone index=%d hash=0000000000000000 parent=%d" % (i, -1 if i == 0 else 0))
    lines.append("ticks,desc,thread,rig,generation,rigCount,name,gameIndex," + ",".join("s0_%d" % k for k in range(12)) + ","
                 + ",".join("s1_%d" % k for k in range(12)))
    k = 0
    while True:
        t = t_lo + k / fps
        if t > t_hi:
            break
        if k in gaps:  # a dropped trace frame / a hitch: no rows for this frame
            k += 1
            continue
        pose = pose_fn(t)
        tk = ARM + int(round(t * HZ))
        for n, i in BONES.items():
            p = pose.get(n, (0.0, 0.0, 0.0)) if n != "Root" else (0.0, 0.0, 0.0)
            q = quat_fn(t, n) if quat_fn and n != "Root" else (0.0, 0.0, 0.0, 1.0)
            row = [p[0], p[1], p[2], 0.0, q[0], q[1], q[2], q[3], 1.0, 1.0, 1.0, 1.0]
            vals = ",".join("%.6f" % v for v in row)
            lines.append("%d,77,1,2,3,645,%s,%d,%s,%s" % (tk, n, i, vals, vals))
        k += 1
    Path(path).write_text("\n".join(lines) + "\n", encoding="ascii")


def stamp(t):
    s = 13 * 3600 + t
    return "%02d:%02d:%06.3f" % (int(s // 3600), int(s % 3600 // 60), s % 60)


def write_log(path, getup="getup=utpp clip=1 rms=0.12", picks=(), end=True, pos=(), extra=()):
    L = []
    L.append("[2026-11-04 %s] [info    ] [1] [UTPP] UTPP CET [utpp_ragdoll] start seq=1 src=carhit push=separation" % stamp(T_START))
    L.append("[2026-11-04 %s] [info    ] [1] [UTPP] UTPP CET [utpp_ragdoll] release seq=1 why=settled t=2.00 faceUp=1 effect=1 "
             "effectRemaining=1.80 pushed=0 %s" % (stamp(T_RELEASE), getup))
    if end:
        L.append("[2026-11-04 %s] [info    ] [1] [UTPP] UTPP CET [utpp_ragdoll] end seq=1 why=getup done dt=5.00 effectRemoved=1 "
                 "effectLeft=0 pushed=0" % stamp(T_END))
    for t, db, clip in picks:
        L.append("[2026-11-04 %s] [info    ] [2] [UTPP] UTPP MOTION t=1.000 db=6:%s clip=234:%s ct=0.000 cost=0.5" % (stamp(t), db, clip))
    for t, x, y, z in pos:
        L.append("[2026-11-04 %s] [info    ] [1] [UTPP] UTPP CET [night] POS p=%.2f,%.2f,%.2f cf=0,1" % (stamp(t), x, y, z))
    for t, text in extra:
        L.append("[2026-11-04 %s] [info    ] [1] [UTPP] UTPP CET %s" % (stamp(t), text))
    Path(path).write_text("\n".join(L) + "\n", encoding="ascii")


def run_metric(tmp, pose_fn, extra=(), extra_lines=(), quat_fn=None, drop_bones=(), gaps=(), **logkw):
    csv = os.path.join(tmp, "t.csv")
    log = os.path.join(tmp, "t.log")
    out = os.path.join(tmp, "o.json")
    write_csv(csv, pose_fn, quat_fn=quat_fn, gaps=gaps)
    if drop_bones:
        lines = [l for l in Path(csv).read_text().splitlines() if not any(",%s," % b in l for b in drop_bones)]
        Path(csv).write_text("\n".join(lines) + "\n")
    write_log(log, extra=extra_lines, **logkw)
    rc = gm.main(["--csv", csv, "--log", log, "--out", out, "--utc-offset-hours", "1", "--quiet"] + list(extra))
    return rc, json.load(open(out)) if os.path.exists(out) else None


def failing(res):
    return sorted(k for k in gm.METRICS if not res["metrics"][k]["pass"])


# ------------------------------------------------------------------------------------------------------ tests
def test_fk():
    h2 = math.sqrt(0.5)
    local = {0: ("Root", (0, 0, 0), (0, 0, 0, 1)),
             1: ("A", (0, 0, 1), (0, 0, h2, h2)),        # 90 deg about z
             2: ("B", (1, 0, 0), (0, 0, 0, 1)),
             3: ("Orphan", (5, 5, 5), (0, 0, 0, 1))}
    parents = {0: -1, 1: 0, 2: 1, 3: 9}                    # 9 is not in the frame
    pos, missing = gm.fk_model(local, parents, [0, 1, 2, 3])
    check("fk: child of a rotated parent", all(abs(a - b) < 1e-9 for a, b in zip(pos["B"], (0, 1, 1))), str(pos.get("B")))
    check("fk: parent listed after its child (index order is not assumed)",
          all(abs(a - b) < 1e-9 for a, b in zip(gm.fk_model({5: ("C", (1, 0, 0), (0, 0, 0, 1)), 2: ("P", (0, 2, 0), (0, 0, 0, 1))},
                                                            {5: 2, 2: -1}, [5])[0]["C"], (1, 2, 0))))
    check("fk: missing ancestor is reported, not treated as a model position", "Orphan" not in pos and missing == ["Orphan"], str(missing))


def test_pass():
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose)
        check("synthetic get-up passes all metrics (exit 0)", rc == 0 and res and res["overall_pass"], str(failing(res) if res else rc))
        check("synthetic pass: without a 'getup begin' line the grounded model frame is used", res["ground"]["mode"].startswith("getup-grounded"))
        check("synthetic pass: standing height measured before the hit",
              abs(res["metrics"]["M5"]["standing_hips"] - 1.0) < 1e-6 and "before the hit" in res["metrics"]["M5"]["standing_source"],
              str(res["metrics"]["M5"]))
        check("synthetic pass: planted toes were found and did not slip", res["metrics"]["M3"]["intervals"] >= 2
              and res["metrics"]["M3"]["value"] < 1e-6, str(res["metrics"]["M3"]))
        check("synthetic pass: window frames after the release counted", res["window"]["frames_after_release"] > 150
              and res["window"]["incomplete_frames"] == 0)


def test_m1():
    def pose(t):
        p = good_pose(t)
        if T_RELEASE + 0.5 <= t <= T_RELEASE + 0.6:
            p["Head"] = (p["Head"][0], p["Head"][1], -0.2)  # head through the floor
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose)
        check("M1: a head 0.2 m under the floor fails only M1", failing(res) == ["M1"] and rc == 1, str(failing(res)))
        check("M1: names the bone", res["metrics"]["M1"]["bone"] == "Head" and res["metrics"]["M1"]["value"] < -0.2)

    def pose_finger(t):
        p = good_pose(t)
        if T_RELEASE + 0.5 <= t <= T_RELEASE + 0.6:
            p["LeftHandIndex1"] = (p["LeftHandIndex1"][0], p["LeftHandIndex1"][1], -0.05)
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose_finger)
        check("M1: fingers are part of the check (radius 0.01)", failing(res) == ["M1"] and res["metrics"]["M1"]["bone"] == "LeftHandIndex1",
              str(failing(res)))


def test_m2():
    def pose(t):
        p = good_pose(t)
        if t >= T_RELEASE + 0.02:  # the whole body jumps 0.08 m up in one frame and stays
            p = {n: (v[0], v[1], v[2] + 0.08) if n != "LeftToeBase" and n != "RightToeBase" else v for n, v in p.items()}
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose)
        m = res["metrics"]["M2"]
        check("M2: a 0.08 m hips jump in one frame fails M2", not m["pass"] and m["max_step_z"] > 0.07, str(m))
        check("M2: reported just after the release", m["t_rel_release_z"] is not None and 0.0 < m["t_rel_release_z"] < 0.1)
        check("M2: informational pair dt / speed of the largest step (not a gate)", m["dt_s"] is not None and 0.0 < m["dt_s"] < 0.1
              and abs(m["speed_ms"] - m["max_step"] / m["dt_s"]) < 1e-6 and m["max_dt_s"] >= m["dt_s"], str(m))
        rc2, res2 = run_metric(tmp, pose, extra=["--m2-ignore", "0.0:0.1"])
        check("M2: --m2-ignore skips the pair around a script teleport", res2["metrics"]["M2"]["pass"], str(res2["metrics"]["M2"]))


def test_m2_long_frame():
    # the whole body rises at 1.4 m/s from the release on (a get-up rise); the 60 fps trace loses 2 frames (a 50 ms pair: raw step 0.07 m)
    def rising(t):
        p = good_pose(t)
        if t >= T_RELEASE + 0.5:
            dz = 1.4 * min(t - T_RELEASE - 0.5, 0.8)
            p = {n: (v[0], v[1], v[2] + dz) if n not in gm.TOES else v for n, v in p.items()}
        return p
    k0 = int(round((T_RELEASE + 0.7 - 8.0) * 60))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, rising, gaps={k0, k0 + 1})
        m = res["metrics"]["M2"]
        check("M2: a long frame (50 ms) at a steady 1.4 m/s rise is not a pop: raw step over the limit, normalised step under it, M2 passes",
              m["pass"] and m["max_step_z"] > 0.06 and 0.0 < m["max_step_z_norm"] < 0.04 and m["max_dt_s"] > 0.045, str(m))
        check("M2: a long frame reports raw and normalised values and the pair dt", m["dt_s"] > 0.045 and m["dt_norm_s"] is not None
              and m["max_step_norm"] <= m["max_step"] + 1e-12, str(m))
    # a real pop on a normal-length frame still fails (normalised == raw below 1/60 s)
    def popping(t):
        p = rising(t)
        if t >= T_RELEASE + 1.2:
            p = {n: (v[0], v[1], v[2] + 0.08) if n not in gm.TOES else v for n, v in p.items()}
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, popping)
        m = res["metrics"]["M2"]
        check("M2: an 8 cm pop on a 16.7 ms frame still fails (normalised step = raw step)", not m["pass"] and m["max_step_z_norm"] > 0.07
              and abs(m["max_step_z_norm"] - m["max_step_z"]) < 0.01, str(m))


def test_m3():
    def pose(t):
        p = good_pose(t)
        if t >= T_RELEASE + 0.3:  # the left toe creeps 0.2 m/s while it stays below 0.06 m
            x, y, z = p["LeftToeBase"]
            p["LeftToeBase"] = (x + 0.2 * (t - T_RELEASE - 0.3), y, z)
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose)
        m = res["metrics"]["M3"]
        check("M3: a creeping planted toe fails M3 only", failing(res) == ["M3"] and m["worst"]["bone"] == "LeftToeBase"
              and m["value"] > 0.1, str(m))
        check("M3: the worst interval carries its offset from the end line (informational)", "t_rel_end" in m["worst"]
              and m["worst"]["t_rel_end"] < 0.0, str(m["worst"]))


def test_m4():
    picks = [(T_END + 0.2, "PSD_Dense_Jumps", "M_Neutral_Jump_F_Start_Walk_Rfoot")]
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, picks=picks)
        m = res["metrics"]["M4"]
        check("M4: a Jumps pick 0.2 s after the end fails M4 only", failing(res) == ["M4"] and len(m["picks"]) == 1
              and abs(m["picks"][0]["t_after_end"] - 0.2) < 1e-3, str(m))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, picks=[(T_END + 1.7, "PSD_Dense_Jumps", "M_Neutral_Jump_F_Start_Walk_Rfoot"),
                                                    (T_END - 0.5, "PSD_Dense_Jumps", "x"),
                                                    (T_END + 0.3, "PSD_Dense_Stand_Idles", "Stand_Idle_Loop")])
        check("M4: picks outside the 1.5 s window and non-air picks are ignored", res["metrics"]["M4"]["pass"], str(res["metrics"]["M4"]))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, picks=[(T_END + 0.4, "PSD_Dense_Stand_Idle_Lands_Light", "M_Neutral_Jump_F_Land")])
        check("M4: a Lands database pick fails", failing(res) == ["M4"])
    with tempfile.TemporaryDirectory() as tmp:
        csv = os.path.join(tmp, "t.csv")
        log = os.path.join(tmp, "t.log")
        write_csv(csv, good_pose)
        write_log(log, end=False)
        rc = gm.main(["--csv", csv, "--log", log, "--utc-offset-hours", "1", "--quiet"])
        check("M4 / M5: a missing end line fails instead of passing silently", rc == 1)


def test_m5():
    def pose(t):
        p = good_pose(t)
        if t > T_RELEASE:  # never rises above 0.5 m
            w = smoothstep((t - T_RELEASE) / 2.8)
            p = {n: (v[0], v[1], v[2] * (0.5 + 0.0 * w) if gm.radius_of(n) and n not in gm.TOES else v[2]) if n == "Hips" else v
                 for n, v in p.items()}
        return p
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose)
        check("M5: hips at 0.5 m at the end fails M5", not res["metrics"]["M5"]["pass"] and res["metrics"]["M5"]["value"] < 0.6,
              str(res["metrics"]["M5"]))


def test_auto_ground():
    """Vanilla path (no getup=utpp): ground from the lying pose, shifted by the entity z of the POS lines."""
    def pose(t):
        p = good_pose(t)
        return {n: (v[0], v[1], v[2] - 0.2) for n, v in p.items()}   # the whole trace sits 0.2 m low (entity above the floor)
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, pose, getup="getup=vanilla why=off")
        g = res["ground"]
        check("auto ground: pre-release lowest (z - radius) with the 0.2 m offset", g["mode"] == "pre-release" and abs(g["base"] + 0.195) < 0.01,
              str(g))
        check("auto ground: vanilla pose then passes the floor metric", res["metrics"]["M1"]["pass"], str(res["metrics"]["M1"]))
        pos = [(T_RELEASE - 0.5, 10.0, 10.0, 50.17), (T_RELEASE + 1.0, 10.0, 10.0, 50.00)]
        rc2, res2 = run_metric(tmp, pose, getup="getup=vanilla why=off", pos=pos)
        check("auto ground: an entity drop of 0.17 m raises the ground in the model frame",
              res2["ground"]["entity_z_shift"] and not res2["metrics"]["M1"]["pass"], str(res2["metrics"]["M1"]))
        rc3, res3 = run_metric(tmp, pose, getup="getup=vanilla why=off", pos=pos, extra=["--no-entity-z"])
        check("auto ground: --no-entity-z keeps the pre-release ground", res3["metrics"]["M1"]["pass"])
        rc4, res4 = run_metric(tmp, good_pose, extra=["--ground", "0"], getup="getup=vanilla why=off")
        check("--ground overrides the automatic choice", res4["ground"]["mode"] == "fixed" and res4["ground"]["base"] == 0.0)


# ------------------------------------------------------------------------------------------------ teleport (world frame)
E0 = (100.0, 200.0, 50.0, 0.0, 1.0)                                  # entity before the script teleport (x, y, z, fwd x, fwd y)
_A = math.radians(40.0)
E1 = (100.3, 199.8, 49.83, -math.sin(_A), math.cos(_A))            # after: 0.3 / -0.2 m, 0.17 m lower, turned 40 degrees
T_LAND = T_RELEASE + 0.001  # the script teleports in the Begin call: the first frame after the release is already in the new entity frame


def steady_quat(t_land=None, flip=None):
    """Model orientations of bones that keep one world orientation across the teleport (model = inverse entity yaw); flip(t, name) -> an
    extra rotation quaternion (the bone turns in the world) or None."""
    tl = T_LAND if t_land is None else t_land

    def fn(t, name):
        ent = E0 if t < tl else E1
        h = -0.5 * gm.yaw_of(ent)
        q = (0.0, 0.0, math.sin(h), math.cos(h))
        f = flip(t, name) if flip else None
        return gm.qmul(q, f) if f else q
    return fn


def world_pose_fn(base_fn, snap=0.0, t_land=None):
    """Model-frame pose of a body that stays put in the world (base_fn is in the frame of the teleported entity E1)."""
    tl = T_LAND if t_land is None else t_land

    def fn(t):
        base = base_fn(t)
        out = {}
        for n, p in base.items():
            q = (p[0], p[1], p[2] + (snap if t >= tl and n not in gm.TOES else 0.0))
            w = gm.to_world(q, E1)
            if t < tl:
                dx, dy = w[0] - E0[0], w[1] - E0[1]
                out[n] = (dx * E0[4] - dy * E0[3], dx * E0[3] + dy * E0[4], w[2] - E0[2])
            else:
                out[n] = q
        return out
    return fn


def begin_line():
    return (T_RELEASE + 0.001, "[utpp_ragdoll] getup begin seq=1 clip=1 pre=(%.4f,%.4f,%.4f) preFwd=(%.4f,%.4f,0.0000) feet=(%.4f,%.4f,%.4f) "
            "fwd=(%.4f,%.4f,0.0000) ground=ray restriction=0 status=phase=1" % (E0[0], E0[1], E0[2], E0[3], E0[4], E1[0], E1[1], E1[2], E1[3], E1[4]))


def test_teleport_world():
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose), extra_lines=[begin_line()], quat_fn=steady_quat())
        g = res["ground"]
        check("teleport: a body that stays put in the world passes all metrics although the entity moved 0.36 m / 0.17 m / 40 deg",
              rc == 0 and res["overall_pass"], str(failing(res)))
        check("teleport: world mode, landing frame found right after the release, tiny world hips step",
              g["mode"] == "getup-world" and 0.0 <= g["landing_t_rel_release"] <= 0.04 and g["landing_hips_step"] < 0.01, str(g))
        check("teleport: ground is the teleport target z", abs(g["base"] - E1[2]) < 1e-4)
        m6 = res["metrics"]["M6"]
        check("M6: a body that stays put in the world across the teleport passes the release continuity (hips and Head / Eye orientation)",
              m6["pass"] and m6["max_step"] < 0.01 and m6["max_turn_deg"] < 1.0 and m6["orientation_bones"] == ["Head", "LeftEye", "RightEye"],
              str(m6))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose))
        check("teleport: without the begin line the model-frame fallback trips on the teleport (limitation is visible)",
              not res["overall_pass"] and res["ground"]["mode"].startswith("getup-grounded"), str(failing(res)))
        check("M6: without the begin line it is n/a (passes with the reason), the other metrics decide",
              res["metrics"]["M6"]["pass"] and "na" in res["metrics"]["M6"])
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose, snap=0.1), extra_lines=[begin_line()], quat_fn=steady_quat())
        m2 = res["metrics"]["M2"]
        check("teleport: a 0.1 m hips snap at the landing still fails M2, reported at the landing (and M6: it is the first frame)",
              failing(res) == ["M2", "M6"]
              and 0.09 < m2["max_step_z"] < 0.11 and abs(m2["t_rel_release_z"] - res["ground"]["landing_t_rel_release"]) < 0.02, str(m2))
        check("teleport: the landing step is reported", abs(m2["landing"]["hips_step"] - 0.1) < 0.01, str(m2["landing"]))
    with tempfile.TemporaryDirectory() as tmp:
        def sinking(t):
            p = world_pose_fn(good_pose)(t)
            if T_RELEASE + 0.5 <= t <= T_RELEASE + 0.6:
                p["Head"] = (p["Head"][0], p["Head"][1], p["Head"][2] - 0.4)
            return p
        rc, res = run_metric(tmp, sinking, extra_lines=[begin_line()], quat_fn=steady_quat())
        check("teleport: a limb under the world floor fails M1", failing(res) == ["M1"] and res["metrics"]["M1"]["bone"] == "Head",
              str(failing(res)))


def test_m6_release():
    """The r13b defect: one frame between the release and the first get-up pose shows the game pose / a pose in the old entity frame."""
    # (1) the landing is one frame late: that frame is expressed in the entity frame BEFORE the teleport although the entity already moved.
    # M2 cannot see it (find_landing picks the split with the smallest hips step), M6 can.
    late = T_RELEASE + 0.03
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose, t_land=late), extra_lines=[begin_line()], quat_fn=steady_quat(late))
        m6 = res["metrics"]["M6"]
        check("M6: a one-frame-late re-expression into the new entity frame fails M6 while M2 passes (the r13b blind spot)",
              failing(res) == ["M6"] and res["metrics"]["M2"]["pass"] and 0.3 < m6["max_step"] < 0.45 and m6["t_rel_release"] < 0.05, str(m6))
    # (2) a Head / Eye orientation flip of one frame right after the release (the game graph pose) with continuous hips
    h = math.radians(160.0) / 2.0

    def flip(t, name):
        if T_RELEASE < t < T_RELEASE + 0.03 and name in ("LeftEye", "RightEye", "Head"):
            return (math.sin(h), 0.0, 0.0, math.cos(h))
        return None
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose), extra_lines=[begin_line()], quat_fn=steady_quat(flip=flip))
        m6 = res["metrics"]["M6"]
        check("M6: a 160 degree Head / Eye flip in the first frame after the release fails M6 with the hips continuous",
              failing(res) == ["M6"] and m6["max_step"] < 0.01 and 150.0 < m6["max_turn_deg"] < 170.0 and m6["turn_bone"] in ("Head", "LeftEye", "RightEye"),
              str(m6))
    # (3) a flip outside the window (0.2 s after the release) is not an M6 matter
    def late_flip(t, name):
        return flip(t - 0.5, name) if T_RELEASE + 0.5 < t < T_RELEASE + 0.53 else None
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose), extra_lines=[begin_line()], quat_fn=steady_quat(flip=late_flip))
        check("M6: only the first 0.2 s after the release are checked", res["metrics"]["M6"]["pass"], str(res["metrics"]["M6"]))
    # (4) no Head / Eye rows: the hips decide, the missing orientation check is stated, not silently counted as a pass
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, world_pose_fn(good_pose), extra_lines=[begin_line()], quat_fn=steady_quat(),
                             drop_bones=("Head", "LeftEye", "RightEye"))
        m6 = res["metrics"]["M6"]
        check("M6: without Head / Eye rows the orientation part is reported n/a", m6["pass"] and "n/a" in m6.get("orientation", "")
              and m6["orientation_bones_missing"] == ["Head", "LeftEye", "RightEye"], str(m6))
        check("M6: the n/a is visible in the summary line", any("n/a" in l for l in gm.summary_lines(res, {"release_why": "x", "getup": "utpp"}, 1)
                                                               if l.startswith("M6")))


def test_m4_handoff_window():
    ho = (T_END - 0.4, "[utpp_ragdoll] getup handoff seq=1 effectRemoved=1 state=3")
    air = "PSD_Dense_Jumps", "M_Neutral_Jump_F_Start_Walk_Rfoot"
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, extra_lines=[ho], picks=[(T_END - 0.3,) + air])
        m = res["metrics"]["M4"]
        check("M4: a Jumps pick between the hand-off and the end fails", failing(res) == ["M4"] and len(m["picks"]) == 1
              and abs(m["picks"][0]["t_after_end"] + 0.3) < 1e-3, str(m))
        check("M4: window starts one tick before the hand-off line", abs(m["window_start_after_end"] + 0.5) < 1e-3, str(m))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, extra_lines=[ho], picks=[(T_END - 0.9,) + air])
        m = res["metrics"]["M4"]
        check("M4: a pick during the get-up (before the window) is listed, not counted", m["pass"] and len(m["picks_before_window"]) == 1,
              str(m))
    with tempfile.TemporaryDirectory() as tmp:
        rc, res = run_metric(tmp, good_pose, picks=[(T_END - 0.3,) + air])
        check("M4: without a hand-off line the window still starts at the end line", res["metrics"]["M4"]["pass"])


def test_missing_ancestor_frames():
    with tempfile.TemporaryDirectory() as tmp:
        csv = os.path.join(tmp, "t.csv")
        write_csv(csv, good_pose)
        # drop the Root rows: every bone's parent (0) is absent in each frame -> unresolved, frames counted as incomplete
        lines = [l for l in Path(csv).read_text().splitlines() if ",Root," not in l]
        Path(csv).write_text("\n".join(lines) + "\n")
        log = os.path.join(tmp, "t.log")
        write_log(log)
        out = os.path.join(tmp, "o.json")
        rc = gm.main(["--csv", csv, "--log", log, "--out", out, "--utc-offset-hours", "1", "--quiet"])
        res = json.load(open(out)) if os.path.exists(out) else None
        check("frames whose ancestors are missing are counted and fail the floor check instead of passing",
              rc == 1 and res and res["window"]["incomplete_frames"] > 100 and not res["metrics"]["M1"]["pass"], str(rc))


def test_bad_input():
    with tempfile.TemporaryDirectory() as tmp:
        log = os.path.join(tmp, "t.log")
        csv = os.path.join(tmp, "t.csv")
        write_csv(csv, good_pose)
        Path(log).write_text("[2026-11-04 13:00:00.000] [info    ] [1] [UTPP] nothing here\n")
        check("a log without a release line exits with 2", gm.main(["--csv", csv, "--log", log, "--quiet"]) == 2)
        check("a missing CSV exits with 2", gm.main(["--csv", os.path.join(tmp, "none.csv"), "--log", log, "--quiet"]) == 2)


def find_research():
    for p in HERE.parents:
        r = p / "Research" / "logs" / "gametest" / "r6a-cycle-20261104c-carhit"
        if r.is_dir():
            return r
    return None


def test_r6a_fixture():
    r = find_research()
    if r is None:
        skip("r6a fixture", "Research/logs/gametest/r6a-cycle-20261104c-carhit not found")
        return
    digest = r / "carhit_queue_mm" / "carhit_still_r1.txt"
    ev = gm.parse_log([str(digest)])
    seq, ep = gm.pick_episode(ev, None) if any("release" in e for e in ev["episodes"].values()) else (1, ev["episodes"].get(1, {}))
    check("r6a digest: end line and MOTION picks parsed", "end" in ep and len(ev["mm"]) >= 3, str(ep))
    t_end = ep["end"]
    picks = [m for m in ev["mm"] if t_end <= m["s"] <= t_end + gm.M4_WINDOW_S and (("Jump" in m["db"]) or ("Land" in m["db"]) or ("Jump" in m["clip"]))]
    check("r6a digest: Jumps pick right after the end, then Lands (M4 fails)", len(picks) >= 2 and "Jumps" in picks[0]["db"]
          and 0.0 <= picks[0]["s"] - t_end < 0.1 and any("Lands" in p["db"] for p in picks), str(picks[:3]))
    csv = r / "bodytrace" / "utpp-bodytrace-2026-11-04-13-25-01.window.csv"
    full_log = None
    for p in HERE.parents:
        c = p / "Research" / "logs" / "game" / "utpp-2026-11-04-13-24-24.log"
        if c.exists():
            full_log = c
            break
    if not csv.exists() or full_log is None:
        skip("r6a full metric", "trimmed bodytrace CSV or game log missing")
        return
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "o.json")
        rc = gm.main(["--csv", str(csv), "--log", str(full_log), "--log", str(digest), "--out", out, "--utc-offset-hours", "1", "--quiet"])
        res = json.load(open(out))
        check("r6a fixture FAILS the acceptance gate (exit 1)", rc == 1 and not res["overall_pass"])
        m = res["metrics"]
        check("r6a fixture fails M1 (limb under the floor)", not m["M1"]["pass"] and m["M1"]["value"] < -0.5, str(m["M1"]))
        check("r6a fixture fails M2 (hips snap of about 0.37 m in one frame)", not m["M2"]["pass"] and 0.3 < m["M2"]["max_step_z"] < 0.45
              and m["M2"]["t_rel_release_z"] < 0.1, str(m["M2"]))
        check("r6a fixture fails M4 (Jump / Land picks after the end)", not m["M4"]["pass"] and len(m["M4"]["picks"]) >= 2)
        rc = gm.main(["--csv", str(csv), "--log", str(full_log), "--log", str(digest), "--utc-offset-hours", "1", "--quiet", "--expect-fail"])
        check("r6a fixture with --expect-fail exits 0", rc == 0)
        rc = gm.main(["--csv", str(csv), "--log", str(full_log), "--log", str(digest), "--utc-offset-hours", "1", "--quiet",
                      "--expect-fail", "M3"])
        check("--expect-fail with a metric that passes exits 1", rc == 1)


def test_r13b_fixture():
    """In-game r13b (release 265.311): one frame between the release and the first get-up pose. M6 must fail, the others pass."""
    r = None
    for p in HERE.parents:
        c = p / "Research" / "logs" / "gametest" / "r13b-carhit2" / "carhit_queue_mm-getup"
        if c.is_dir():
            r = c
            break
    if r is None:
        skip("r13b fixture", "Research/logs/gametest/r13b-carhit2/carhit_queue_mm-getup not found")
        return
    csv = r / "utpp-bodytrace-2026-11-04-18-28-24.window.csv"
    log = r / "carhit_queue_mm-getup-window.log"
    if not csv.exists() or not log.exists():
        skip("r13b fixture", "window CSV or log missing")
        return
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "o.json")
        rc = gm.main(["--csv", str(csv), "--log", str(log), "--out", out, "--utc-offset-hours", "1", "--quiet"])
        res = json.load(open(out))
        m = res["metrics"]
        check("r13b fixture FAILS the acceptance gate through M6 only (M2 and the others pass: the one-frame mismatch is invisible to them)",
              rc == 1 and failing(res) == ["M6"] and m["M2"]["pass"], str(failing(res)))
        check("r13b fixture M6: world hips step ~0.49 m and an eye flip of ~170 degrees in the first frame after the release",
              0.4 < m["M6"]["max_step"] < 0.6 and m["M6"]["max_turn_deg"] > 120.0 and m["M6"]["t_rel_release"] < 0.08, str(m["M6"]))
        rc = gm.main(["--csv", str(csv), "--log", str(log), "--utc-offset-hours", "1", "--quiet", "--expect-fail", "M6"])
        check("r13b fixture with --expect-fail M6 exits 0", rc == 0)


def main():
    test_fk()
    test_pass()
    test_m1()
    test_m2()
    test_m2_long_frame()
    test_m3()
    test_m4()
    test_m5()
    test_teleport_world()
    test_m6_release()
    test_m4_handoff_window()
    test_auto_ground()
    test_missing_ancestor_frames()
    test_bad_input()
    test_r6a_fixture()
    test_r13b_fixture()
    print("getup_metric tests: %d passed, %d failed, %d skipped" % (PASSED, FAILED, SKIPPED))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
