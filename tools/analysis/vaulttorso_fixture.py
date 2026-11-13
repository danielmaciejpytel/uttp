"""Builds the small test fixture of tests/VaultTorsoTest.cpp from a recording of 2026-11-10 13:00 (hood slide with / without a
weapon): the first 29 local bone records (Root .. WeaponRight: spine chain, shoulders, neck, head, arms, hands, weapon slot) of every frame of a few vault / climb windows of the continuous body trace
(utpp-bodytrace-*.csv, stage s1 = the pose the game rig shows), plus the release timeline (travRelease on / off) from the DLL log.

Usage: python -I vaulttorso_fixture.py <bodytrace.csv> <utpp.log> <out.txt>
The recording (log + bodytrace + pose csv): utpp-2026-11-10-13-00-01.log, utpp-bodytrace-2026-11-10-13-00-50.csv;
the fixture goes to Build\\test-data\\vaulttorso\\ (read by utpp_vehexit_vaulttorso_test; the test names its path in tests/vehexit.cmake).
"""
import math
import re
import sys

BODY_BONES = 29  # Root .. Spine3 (0..13), LeftShoulder, RightShoulder, Neck, LeftArm, RightArm, Neck1, LeftForeArm, RightForeArm, Head, LeftHand, RightHand, LeftHandEnd, WeaponLeft, RightHandEnd, WeaponRight
PARENTS = [-1, 0, 0, 0, 2, 2, 2, 4, 5, 6, 7, 8, 9, 10, 13, 13, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24, 24]
NAMES = ["Root", "Trajectory", "Hips", "reference_joint", "Spine", "LeftUpLeg", "RightUpLeg", "Spine1", "LeftLeg", "RightLeg", "Spine2",
         "LeftFoot", "RightFoot", "Spine3", "LeftShoulder", "RightShoulder", "Neck", "LeftArm", "RightArm", "Neck1", "LeftForeArm", "RightForeArm",
         "Head", "LeftHand", "RightHand", "LeftHandEnd", "WeaponLeft", "RightHandEnd", "WeaponRight"]
ARM_TICKS = None
WINDOW_BEFORE, WINDOW_AFTER = 0.25, 0.60  # seconds around the release (on - before .. off + after)

# (label, release-on local time of the log, weapon in hand, detail, v2 owns it (no release), kind)
# kind: "veh" = vault over a vehicle (released to the game), "mantle" = v2-owned mantle / released climb
CASES = [
    ("rifle_vault_a", "13:03:31.940", 1, 9, 0), ("rifle_vault_b", "13:03:54.873", 1, 9, 0), ("rifle_vault_c", "13:03:59.513", 1, 9, 0),
    ("rifle_vault_d", "13:05:12.593", 1, 9, 0), ("rifle_vault_e", "13:05:17.218", 1, 9, 0), ("rifle_vault_f", "13:05:37.888", 1, 9, 0),
    ("rifle_vault_run", "13:03:24.380", 1, 9, 0), ("rifle_vault_stand", "13:03:29.824", 1, 9, 0),
    ("unarmed_vault_a", "13:03:41.583", 0, 9, 0), ("unarmed_vault_b", "13:03:43.636", 0, 9, 0), ("unarmed_vault_c", "13:04:14.387", 0, 9, 0),
    ("unarmed_vault_run", "13:04:40.695", 0, 9, 0),
    ("rifle_climb_released", "13:05:43.964", 1, 8, 0),
]
# v2-owned mantles: no travRelease at all (window start, local time of the log)
MANTLES = [("rifle_mantle_motion", "13:03:21.300", 1, 8), ("unarmed_mantle_v2", "13:03:39.450", 0, 8)]


def secs(hms):
    h, m, s = hms.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def main():
    global ARM_TICKS
    trace, logp, out = sys.argv[1:4]
    with open(trace, "r", encoding="utf8") as f:
        head = f.readline()
    m = re.search(r"arm_ticks=(\d+) utc=(\d+)-(\d+)-(\d+)T(\d+):(\d+):([\d.]+)Z", head)
    ARM_TICKS = int(m.group(1))
    arm_utc = int(m.group(5)) * 3600 + int(m.group(6)) * 60 + float(m.group(7))
    arm_local = arm_utc + 3600  # CET (UTC+1) of the recording
    # release timeline from the log: 'travRelease on' / 'off' lines
    events = []
    with open(logp, "r", encoding="utf8", errors="replace") as f:
        for line in f:
            if "MMTRAV travRelease" in line:
                t = secs(line[1:24].split(" ")[1])
                events.append((t, "on" in line.split("travRelease")[1].split()[0], re.search(r"detail=(-?\d+)", line).group(1)))
    on_off = []
    cur = None
    for t, on, d in events:
        if on:
            cur = t
        elif cur is not None:
            on_off.append((cur, t))
            cur = None

    def off_for(t_on):
        for a, b in on_off:
            if abs(a - t_on) < 0.01:
                return b
        raise SystemExit("no release pair for %f" % t_on)

    wins = []
    for label, hms, weapon, detail, _ in CASES:
        on = secs(hms)
        wins.append((label, on, off_for(on), weapon, detail, 1))
    for label, hms, weapon, detail in MANTLES:
        a = secs(hms)
        wins.append((label, a + WINDOW_BEFORE, a + 1.1, weapon, detail, 0))  # release flag 0 for the whole window
    wins.sort(key=lambda w: w[1])
    lo = min(w[1] for w in wins) - WINDOW_BEFORE - 0.1
    hi = max(w[2] for w in wins) + WINDOW_AFTER + 0.1
    frames = {w[0]: [] for w in wins}
    cur_t = None
    rec = {}

    def flush():
        if cur_t is None or len(rec) < BODY_BONES:
            return
        tl = arm_local + (cur_t - ARM_TICKS) / 1e7
        for w in wins:
            if w[1] - WINDOW_BEFORE <= tl <= w[2] + WINDOW_AFTER:
                frames[w[0]].append((tl - w[1], [rec[i] for i in range(BODY_BONES)]))

    with open(trace, "r", encoding="utf8") as f:
        for line in f:
            if line[0] == "#" or line.startswith("ticks"):
                continue
            t = int(line[: line.find(",")])
            tl = arm_local + (t - ARM_TICKS) / 1e7
            if tl < lo:
                continue
            if tl > hi:
                break
            cc = line.split(",")
            if t != cur_t:
                flush()
                cur_t = t
                rec = {}
            b = int(cc[7])
            if b < BODY_BONES:
                v = cc[20:32]  # stage s1 (after_mm_and_aim): pos4, quat4, scale4
                rec[b] = (float(v[0]), float(v[1]), float(v[2]), float(v[4]), float(v[5]), float(v[6]), float(v[7]))
        flush()
    with open(out, "w", encoding="utf8") as o:
        o.write("# utpp vaulttorso fixture v1: local bone records (pos xyz, quat xyzw) of bones 0..13 per frame, from the recording\n")
        o.write("# 2026-11-10 13:00 (utpp-bodytrace-2026-11-10-13-00-50.csv stage s1, DLL log utpp-2026-11-10-13-00-01.log); tools/analysis/vaulttorso_fixture.py\n")
        o.write("rig %d\n" % BODY_BONES)
        for i in range(BODY_BONES):
            o.write("bone %d %s %d\n" % (i, NAMES[i], PARENTS[i]))
        for label, on, off, weapon, detail, release in wins:
            fr = frames[label]
            o.write("vault %s weapon=%d detail=%d release=%d on=%.4f off=%.4f frames=%d\n" % (label, weapon, detail, release, 0.0, off - on, len(fr)))
            for t, bones in fr:
                o.write("f %.5f" % t)
                for b in bones:
                    o.write(" " + " ".join("%.7g" % x for x in b))
                o.write("\n")
            o.write("end\n")
            print(label, len(fr), "frames", "on", 0.0, "off %.3f" % (off - on))


if __name__ == "__main__":
    main()
