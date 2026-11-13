"""Offline check of the forearm roll pass (plugin/UTPP/src/mm/ForearmTwist.hpp) on a recorded pose trace.

Streams a utpp-pose-*.csv, takes the MM stage (mm_0..2 local position, mm_3..6 local quaternion x y z w) of
LeftForeArm/RightForeArm (20/21) and LeftHand/RightHand (23/24) per frame, applies the same swing-twist move as the
runtime (forearm' = swing, hand' = twist * hand, axis = normalized hand local translation) and reports per
(overlay, action, aiming) group: forearm twist before/after, hand-local roll before/after (roll of the hand local
rotation about the same axis) and the maximum hand model error in the upper-arm frame (position m, rotation deg).
Also prints the stage s0 (game) forearm twist for the same rows as the reference convention.

usage: python -I forearm_twist_offline.py <utpp-pose-*.csv> [--every N] [--at TICKS]
"""
import math
import statistics
import sys

SIDES = {"L": (20, 23), "R": (21, 24)}
BONES = {20, 21, 23, 24}


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qnorm(q):
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return tuple(v / n for v in q)


def qrot(q, v):
    p = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return p[:3]


def twist_deg(q, a):
    d = q[0] * a[0] + q[1] * a[1] + q[2] * a[2]
    w = q[3]
    if w < 0:
        w, d = -w, -d
    return math.degrees(2 * math.atan2(d, w))


def angle_deg(a, b):
    r = qmul(qconj(qnorm(a)), qnorm(b))
    v = math.sqrt(r[0] ** 2 + r[1] ** 2 + r[2] ** 2)
    return math.degrees(2 * math.atan2(v, abs(r[3])))


def move(fa_q, hand_q, hand_t):
    """Returns (forearm', hand') exactly like ov::MoveForearmTwistToHand (None when skipped)."""
    ln = math.sqrt(sum(v * v for v in hand_t))
    if ln <= 1e-5:
        return None
    a = tuple(v / ln for v in hand_t)
    q = qnorm(fa_q)
    d = q[0] * a[0] + q[1] * a[1] + q[2] * a[2]
    tw = (a[0] * d, a[1] * d, a[2] * d, q[3])
    tn = math.sqrt(sum(v * v for v in tw))
    if tn <= 1e-6:
        return None
    tw = tuple(v / tn for v in tw)
    if tw[3] < 0:
        tw = tuple(-v for v in tw)
    return qnorm(qmul(q, qconj(tw))), qnorm(qmul(tw, hand_q)), a


def summ(v):
    if not v:
        return "-"
    s = sorted(abs(x) for x in v)
    return f"med {statistics.median(v):7.1f} p95|.| {s[int(0.95 * (len(s) - 1))]:6.1f} max|.| {s[-1]:6.1f}"


def main():
    path = sys.argv[1]
    every = 1
    at = None
    if "--every" in sys.argv:
        every = int(sys.argv[sys.argv.index("--every") + 1])
    if "--at" in sys.argv:
        at = int(sys.argv[sys.argv.index("--at") + 1])
    groups = {}
    tot = {"frames": 0, "skipped": 0, "pos": 0.0, "rot": 0.0}
    printed_at = False
    with open(path, newline="") as fh:
        fh.readline()
        hdr = fh.readline().strip().split(",")
        ix = {k: i for i, k in enumerate(hdr)}
        ib = ix["bone"]
        im, is0 = ix["mm_0"], ix["s0_0"]
        frame_key = None
        cur = {}
        meta = None
        nframe = 0

        def flush():
            nonlocal printed_at
            if meta is None or not BONES.issubset(cur):
                return
            key = meta[:3]
            g = groups.setdefault(key, {s: {"fb": [], "fa": [], "hb": [], "ha": [], "s0": []} for s in SIDES} | {"n": 0})
            g["n"] += 1
            for s, (fb, hb) in SIDES.items():
                fpos, fq, fs0 = cur[fb]
                hpos, hq, _ = cur[hb]
                if any(math.isnan(v) for v in fq + hq + hpos):
                    continue
                r = move(fq, hq, hpos)
                if r is None:
                    tot["skipped"] += 1
                    continue
                fq2, hq2, a = r
                gs = g[s]
                gs["fb"].append(twist_deg(qnorm(fq), a))
                gs["fa"].append(twist_deg(fq2, a))
                gs["hb"].append(twist_deg(qnorm(hq), a))
                gs["ha"].append(twist_deg(hq2, a))
                if fs0 is not None and not any(math.isnan(v) for v in fs0):
                    gs["s0"].append(twist_deg(qnorm(fs0), a))
                # hand model in the upper-arm frame: rot = fa * hand, pos = fa.t + fa(hand.t)
                m0r, m1r = qmul(qnorm(fq), qnorm(hq)), qmul(fq2, hq2)
                p0 = [fpos[i] + v for i, v in enumerate(qrot(qnorm(fq), hpos))]
                p1 = [fpos[i] + v for i, v in enumerate(qrot(fq2, hpos))]
                tot["pos"] = max(tot["pos"], math.dist(p0, p1))
                tot["rot"] = max(tot["rot"], angle_deg(m0r, m1r))
                if at is not None and not printed_at and int(frame_key[0]) >= at:
                    print(f"frame {frame_key[0]} ov/act/aim {key} {s}: forearm twist {twist_deg(qnorm(fq), a):.1f} -> "
                          f"{twist_deg(fq2, a):.4f}, hand roll {twist_deg(qnorm(hq), a):.1f} -> {twist_deg(hq2, a):.1f}, "
                          f"s0 forearm twist {twist_deg(qnorm(fs0), a) if fs0 else float('nan'):.1f}")
                    if s == "R":
                        printed_at = True
            tot["frames"] += 1

        for line in fh:
            if not line or line[0] == "#":
                continue
            head = line.split(",", ib + 2)
            if len(head) <= ib + 1:
                continue
            key = (head[0], head[3])
            if key != frame_key:
                if nframe % every == 0:
                    flush()
                nframe += 1
                frame_key, cur, meta = key, {}, None
            b = int(head[ib])
            if b not in BONES:
                continue
            row = line.rstrip("\r\n").split(",")
            meta = (row[ix["overlay"]], row[ix["action"]], row[ix["aiming"]])
            mm = [float(row[im + k]) for k in range(7)]
            s0 = [float(row[is0 + 4 + k]) for k in range(4)]
            cur[b] = ((mm[0], mm[1], mm[2]), (mm[3], mm[4], mm[5], mm[6]), tuple(s0))
        if nframe % every == 0:
            flush()

    print(f"frames {tot['frames']} (every {every}), skipped sides {tot['skipped']}, "
          f"max hand model error in the upper-arm frame: pos {tot['pos']:.2e} m, rot {tot['rot']:.2e} deg")
    for k in sorted(groups):
        g = groups[k]
        if g["n"] < 30:
            continue
        for s in SIDES:
            gs = g[s]
            print(f"ov={k[0]} act={k[1]} aim={k[2]} {s} n={g['n']:6d} | s0 fa {summ(gs['s0'])} | mm fa before {summ(gs['fb'])}"
                  f" after {summ(gs['fa'])} | hand roll before {summ(gs['hb'])} after {summ(gs['ha'])}")


if __name__ == "__main__":
    main()
