#!/usr/bin/env python
"""Shooter mode, evidence A9: build the H2 gate sidecar of a recorded game session (READ-ONLY on the inputs).

Usage:
    python -I h2_sidecar_from_log.py --feed <utpp-feed-*.txt> --log <utpp-*.log> --out <sidecar.txt> [--name <label>]

Inputs are the 31-column feed dump (Research/logs/game/utpp-feed-*.txt, MM feed dump) and the red4ext/CET log of the same session. The
feed has no column for the native body turn, Locomotion or the rotation mode, so those are reconstructed from the log on the FEED TIME axis:

    feed t = wall time of the log line - wall time of the "MM feed dump on" line of that feed (spec off_invariants P4; the dump starts
    its clock there). The alignment is verified: the CET camTrace "body=" yaw is compared with the feed yaw at the shifted time (median
    of |dyaw| over the samples where the body turns) for lags -0.6 .. +0.6 s; the result is printed and written to the sidecar header.

Sidecar lines (all times are feed seconds, "inf" = to the end of the feed; '#' lines are comments/statistics):
    session/feed/log/dumpon/frames/align      provenance
    mask <t> <hex> <weaponturn 0|1>           UTPPLocomotionSetBlockMask / "Locomotion block mask -> 0x.... (weapon turn on|off)" (CET)
    freeMoveEnable <t> <0|1>                      UTPPLocomotionSetEnabled
    off <t0> <t1>                             native body turn OFF (UTPPSetMMBodyTurn(0) .. (1); the DLL g_bodyTurnOn starts true)
    freeMove <t0> <t1>                            camTrace freeMove=1 (Locomotion active seen by the camera), 10 Hz resolution
    why <label> <t0> <t1>                     camTrace bt=<0|1>(<label>) windows: why the native is on/off (paused, aim_align, aim_aligned,
                                              aim_align_waiting, locomotion_inactive, native_inactive, applied, deadband, ...)
    strafe <t0> <t1>                          MOTION trace rot=strafe (rotation mode Strafe in game: ADS, hip-fire pose)
    act <n> <t0> <t1>                         MOTION trace act=<n> (weapon action; 8 = shoot) windows, n != 0
    melee <t0> <t1>                           MMGATE melee=0x.. / meleeState / meleeHold non-zero windows (log based)
    lease <t0> <t1>                           the CET hip-fire lease (UTPPSetMotionHipLease(1) .. (0), logged by the DLL on every change) - an H2
                                              exclusion. Logs written before that native existed have no such lines: the windows are then DERIVED
                                              from the weapon action 8 windows + the 0.75 s lease tail (the lease is Shoot=8 with a weapon drawn),
                                              which misses only weapons whose resolved aim pose is Default (header line "leasesource derived").
Edges: off/strafe/act edges are exact to the log line (about one frame); freeMove / why windows to the camTrace period (0.1 s).
"""
import argparse
import bisect
import datetime
import math
import os
import re
import sys

WALL_RE = re.compile(r"^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]")
CAM_RE = re.compile(r"camTrace t=(\S+) body=(\S+) .*?mode=(\w+) freeMove=(\d) .*?bt=([01])\(([^)]*)\)")
MMV2_RE = re.compile(r"UTPP MOTION t=\S+ .*? rot=(\w+) ")
ACT_RE = re.compile(r" act=(\d+) ")
MASK_RE = re.compile(r"Locomotion block mask -> (0x[0-9A-Fa-f]+) \(weapon turn (on|off)\)")
MASK2_RE = re.compile(r"UTPPLocomotionSetBlockMask\((0x[0-9A-Fa-f]+)\)")
GATE_RE = re.compile(r"UTPP MMGATE .*?melee=(0x[0-9A-Fa-f]+) meleeState=(\d+) meleeHold=(\d+)")
BT_RE = re.compile(r"UTPPSetMMBodyTurn\(([01])\)")
LEASE_RE = re.compile(r"UTPPSetMotionHipLease\(([01])\)")
LEASE_TAIL_S = 0.75
ENABLE_RE = re.compile(r"UTPPLocomotionSetEnabled\(([01])\)")
INF = float("inf")


def wall_of(line):
    m = WALL_RE.match(line)
    if not m:
        return None
    return datetime.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f")


def wrap(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def label_of(s):
    return re.sub(r"\W+", "_", s.strip()).strip("_") or "none"


def merge_samples(times, gap, tail):
    """Sorted sample times -> windows [first, last + tail] with gaps <= gap merged."""
    out = []
    for t in times:
        if out and t - out[-1][1] <= gap:
            out[-1][1] = t
        else:
            out.append([t, t])
    return [(a, b + tail) for a, b in out]


def fmt(x):
    return "inf" if x == INF else "%.3f" % x


def load_feed(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if line[0] == "#" or not line.strip():
                continue
            p = line.split()
            if len(p) < 31:
                continue
            rows.append(p)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--feed", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="")
    a = ap.parse_args()

    feed = load_feed(a.feed)
    if not feed:
        print("FAIL: no 31-column rows in", a.feed)
        return 2
    ft = [float(p[0]) for p in feed]
    fyaw = [float(p[7]) for p in feed]
    feed_end = ft[-1]
    feed_base = os.path.basename(a.feed)

    # ---- pass over the log
    dumpon = None
    cand = []  # all dump-on lines (wall, file)
    bt_ev, en_ev, mask_ev, cam, mmv2, gate, lease_ev = [], [], [], [], [], [], []
    with open(a.log, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if "[UTPP]" not in line:
                continue
            if "MM feed dump on:" in line:
                w = wall_of(line)
                cand.append((w, line.strip().split("\\")[-1].split("/")[-1]))
                continue
            w = wall_of(line)
            if w is None:
                continue
            m = BT_RE.search(line)
            if m:
                bt_ev.append((w, m.group(1) == "1"))
                continue
            m = LEASE_RE.search(line)
            if m:
                lease_ev.append((w, m.group(1) == "1"))
                continue
            if "camTrace" in line:
                m = CAM_RE.search(line)
                if m:
                    cam.append((w, float(m.group(2)), m.group(3), m.group(4) == "1", m.group(5) == "1", label_of(m.group(6))))
                continue
            if "UTPP MOTION t=" in line:
                m = MMV2_RE.search(line)
                if m:
                    am = ACT_RE.search(line)
                    mmv2.append((w, m.group(1), int(am.group(1)) if am else None))
                continue
            if "MMGATE" in line:
                m = GATE_RE.search(line)
                if m:
                    gate.append((w, int(m.group(1), 16) != 0 or int(m.group(2)) != 0 or int(m.group(3)) != 0))
                continue
            m = MASK_RE.search(line)
            if m:
                mask_ev.append((w, m.group(1), m.group(2) == "on"))
                continue
            m = ENABLE_RE.search(line)
            if m:
                en_ev.append((w, m.group(1) == "1"))
    for w, name in cand:
        if name == feed_base:
            dumpon = w
    if dumpon is None:
        print("FAIL: no 'MM feed dump on' line for", feed_base, "in", a.log, "(found: %s)" % [c[1] for c in cand])
        return 2

    def tf(w):
        return (w - dumpon).total_seconds()

    # ---- alignment check: camTrace body yaw vs feed yaw
    def yaw_at(t):
        i = bisect.bisect_left(ft, t)
        if i <= 0 or i >= len(ft):
            return None
        t0, t1 = ft[i - 1], ft[i]
        if t1 - t0 > 0.2:
            return None
        k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return fyaw[i - 1] + wrap(fyaw[i] - fyaw[i - 1]) * k

    samples = []
    for (w, body, mode, freeMove, btv, why) in cam:
        t = tf(w)
        y0, y1 = yaw_at(t - 0.15), yaw_at(t + 0.15)
        if y0 is None or y1 is None:
            continue
        if abs(wrap(y1 - y0)) >= 2.0:  # the body turns here: informative for the lag
            samples.append((t, body))

    def med_err(lag):
        errs = []
        for t, body in samples:
            y = yaw_at(t + lag)
            if y is not None:
                errs.append(abs(wrap(body - y)))
        if not errs:
            return None
        errs.sort()
        return errs[len(errs) // 2]

    e0 = med_err(0.0)
    best = (None, 1e9)
    for k in range(-60, 61):
        lag = k * 0.01
        e = med_err(lag)
        if e is not None and e < best[1]:
            best = (lag, e)
    align = "samples(body turning)=%d camTrace=%d median|dyaw| at lag 0: %s deg; best lag %s s -> %s deg" % (
        len(samples), len(cam), "n/a" if e0 is None else "%.2f" % e0, "n/a" if best[0] is None else "%.2f" % best[0],
        "n/a" if best[0] is None else "%.2f" % best[1])
    print("ALIGN", align)

    # ---- windows
    off = []
    state_on, start = True, None
    for w, on in bt_ev:
        t = tf(w)
        if not on and state_on:
            start = t
        if on and not state_on and start is not None:
            off.append((max(start, 0.0), t))
        state_on = on
    if not state_on and start is not None:
        off.append((max(start, 0.0), INF))
    off = [(s, e) for (s, e) in off if e > 0.0]

    freeMoveWindows = merge_samples([tf(w) for (w, b, mo, m, bb, wy) in cam if m], 0.25, 0.1)
    # label = native state + the CET reason text: off_paused, off_aim_align, off_locomotion_inactive, on_applied, on_native_inactive, ...
    whyw = {}
    for lab in sorted(set(("on_" if c[4] else "off_") + c[5] for c in cam)):
        whyw[lab] = merge_samples([tf(c[0]) for c in cam if (("on_" if c[4] else "off_") + c[5]) == lab], 0.25, 0.1)

    def state_windows(events, value_of, wanted):
        """events (wall, v) in order; windows (feed t) in which value_of(v) == wanted; a window ends at the next event."""
        out, cur = [], None
        for w, v in events:
            t = tf(w)
            on = value_of(v) == wanted
            if on and cur is None:
                cur = t
            elif not on and cur is not None:
                out.append((cur, t))
                cur = None
        if cur is not None:
            out.append((cur, min(feed_end + 0.5, INF)))
        return [(max(s, 0.0), e) for (s, e) in out if e > 0.0]

    strafe = state_windows([(w, rot) for (w, rot, act) in mmv2], lambda v: v, "strafe")
    acts = {}
    for n in sorted(set(x[2] for x in mmv2 if x[2])):
        acts[n] = state_windows([(w, act) for (w, rot, act) in mmv2 if act is not None], lambda v: v, n)
    melee = state_windows(gate, lambda v: v, True)
    if lease_ev:
        lease = state_windows(lease_ev, lambda v: v, True)
        lease_source = "log"
    else:
        lease = merge_samples([], 0, 0)
        for s0, e0 in acts.get(8, []):
            lease.append((s0, e0 + LEASE_TAIL_S))
        lease_source = "derived"

    masks = []
    seen = set()
    for w, hx, wt in mask_ev:
        masks.append((tf(w), hx, 1 if wt else 0))
        seen.add(round(tf(w), 3))
    for w, on in en_ev:
        pass

    # ---- reference statistics (python twin of the C++ replay counts)
    def in_win(t, wins, pre, post):
        for s, e in wins:
            if s - pre <= t <= e + post:
                return True
        return False

    stats = {}
    n_off = n_mv = n_mv_out = 0
    cnt = {}
    for p in feed:
        t = float(p[0])
        vx, vy = float(p[2]), float(p[3])
        sp = math.hypot(vx, vy)
        ground = p[27] == "1"
        aim = p[25] == "6"
        psm = int(p[10])
        meleef = (psm & 6144) != 0  # PsmMeleeBits: kPsmMeleeAttack (11) | kPsmMeleePose (12)
        if not in_win(t, off, 0.0, 0.0):
            continue
        n_off += 1
        if not ground or sp < 0.5 or aim:
            continue
        n_mv += 1
        win = in_win(t, strafe, 0.02, 0.15)
        act8 = in_win(t, acts.get(8, []), 0.02, 0.15)
        yaw = float(p[7])
        cam_y = float(p[8])
        mvx, mvy = float(p[5]), float(p[6])
        stick = math.hypot(mvx, mvy)
        if p[17] == "1" and p[18] != "-":
            iy = float(p[18])
        else:
            iy = cam_y - math.degrees(math.atan2(mvx, mvy)) if stick > 1e-6 else yaw
        ia = abs(wrap(iy - yaw)) if stick >= 0.15 else 0.0
        va = abs(wrap(math.degrees(math.atan2(-vx, vy)) - yaw))
        outside = not win and not act8
        n_mv_out += outside
        for key, cond in (("sp>=1.5", sp >= 1.5), ("in>=105", ia >= 105 and sp >= 1.5), ("vel>=105", va >= 105 and sp >= 1.5),
                          ("in>=45", ia >= 45 and sp >= 1.5), ("vel>=90", va >= 90), ("vel>=130", va >= 130)):
            if cond:
                cnt[(key, "all")] = cnt.get((key, "all"), 0) + 1
                if outside:
                    cnt[(key, "outside")] = cnt.get((key, "outside"), 0) + 1
                    if not meleef:
                        cnt[(key, "outside_nomelee")] = cnt.get((key, "outside_nomelee"), 0) + 1
    stat_lines = [
        "frames %d feed t %.3f..%.3f" % (len(feed), ft[0], ft[-1]),
        "native-off frames %d (off windows %d, %.2f s)" % (n_off, len(off), sum(min(e, feed_end) - s for s, e in off)),
        "native-off, ground, sp>=0.5, not ADS: %d; outside strafe/act8 windows (-0.02/+0.15 s): %d" % (n_mv, n_mv_out),
    ]
    for key in ("sp>=1.5", "in>=45", "in>=105", "vel>=105", "vel>=90", "vel>=130"):
        stat_lines.append("  %-9s all %5d | outside windows %5d | outside windows, no melee %5d" % (
            key, cnt.get((key, "all"), 0), cnt.get((key, "outside"), 0), cnt.get((key, "outside_nomelee"), 0)))

    # ---- write
    name = a.name or os.path.splitext(feed_base)[0]
    with open(a.out, "w", encoding="utf-8", newline="\n") as o:
        o.write("# h2 sidecar v1 (tools/replay/h2_sidecar_from_log.py; feed seconds = log wall time - 'MM feed dump on' wall time)\n")
        o.write("session %s\n" % name)
        o.write("feed %s\n" % feed_base)
        o.write("log %s\n" % os.path.basename(a.log))
        o.write("dumpon %s\n" % dumpon.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3])
        o.write("frames %d %s\n" % (len(feed), fmt(feed_end)))
        o.write("align %s\n" % align)
        o.write("leasesource %s\n" % lease_source)
        for ln in stat_lines:
            o.write("# stats: %s\n" % ln)
        for (t, hx, wt) in masks:
            o.write("mask %s %s %d\n" % (fmt(t), hx, wt))
        for w, on in en_ev:
            o.write("freeMoveEnable %s %d\n" % (fmt(tf(w)), 1 if on else 0))
        for s, e in off:
            o.write("off %s %s\n" % (fmt(s), fmt(e)))
        for s, e in freeMoveWindows:
            o.write("freeMove %s %s\n" % (fmt(s), fmt(e)))
        for lab in sorted(whyw):
            for s, e in whyw[lab]:
                o.write("why %s %s %s\n" % (lab, fmt(s), fmt(e)))
        for s, e in strafe:
            o.write("strafe %s %s\n" % (fmt(s), fmt(e)))
        for n in sorted(acts):
            for s, e in acts[n]:
                o.write("act %d %s %s\n" % (n, fmt(s), fmt(e)))
        for s, e in melee:
            o.write("melee %s %s\n" % (fmt(s), fmt(e)))
        for s, e in lease:
            o.write("lease %s %s\n" % (fmt(s), fmt(e)))
    print("SIDECAR", a.out)
    for ln in stat_lines:
        print("  " + ln)
    print("  windows: off %d freeMove %d strafe %d act %s melee %d lease %d (%s) why %s" % (
        len(off), len(freeMoveWindows), len(strafe), {n: len(v) for n, v in acts.items()}, len(melee), len(lease), lease_source,
        {k: len(v) for k, v in whyw.items()}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
