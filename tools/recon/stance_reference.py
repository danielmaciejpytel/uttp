"""Stance reference from a CP2077 locomotion .anims.glb (read-only): hip width, ankle / knee lateral separation and shin
distance over a walk cycle, for comparison with the UTPP motion matching database (utpp_mm_steering_test UTPP_TEST_STANCE).

Usage: python stance_reference.py <file.anims.glb> <anim name> [<anim name> ...]
Lateral axis = horizontal hip axis (LeftUpLeg -> RightUpLeg) of each frame; 'passing' frames = ankles within 15 cm along
the walking direction (perpendicular horizontal axis). Up axis = +Y (WolvenKit glb export).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from glb_anim_pose import accessor, load  # noqa: E402

LEGS = ["LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


def seg_dist(p1, q1, p2, q2):
    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def dot(a, b):
        return sum(a[i] * b[i] for i in range(3))
    d1, d2, r = sub(q1, p1), sub(q2, p2), sub(p1, p2)
    a, e, f = dot(d1, d1), dot(d2, d2), dot(d2, r)
    c, b = dot(d1, r), dot(d1, d2)
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0), 1) if den > 1e-12 else 0
    t = (b * s + f) / e if e > 1e-12 else 0
    if t < 0:
        t, s = 0, min(max(-c / a, 0), 1)
    elif t > 1:
        t, s = 1, min(max((b - c) / a, 0), 1)
    w = [p1[i] + d1[i] * s - p2[i] - d2[i] * t for i in range(3)]
    return math.sqrt(dot(w, w))


def main():
    js, binb = load(sys.argv[1])
    nodes = js['nodes']
    parent = {}
    for i, n in enumerate(nodes):
        for c in n.get('children', []):
            parent[c] = i
    index = {n.get('name'): i for i, n in enumerate(nodes)}
    for name in sys.argv[2:]:
        anim = next(a for a in js['animations'] if a.get('name') == name)
        nfr = len(accessor(js, binb, anim['samplers'][0]['input']))
        chans = {}
        for ch in anim['channels']:
            if ch['target']['path'] in ('translation', 'rotation'):
                chans[(ch['target']['node'], ch['target']['path'])] = accessor(js, binb, anim['samplers'][ch['sampler']]['output'])
        stats = dict(n=0, pas=0, hip=0.0, ank=[], knee=[], kneeAll=[], ankAll=[], shin=1e9, cross=0)
        for f in range(nfr):
            cache = {}

            def world(i):
                if i in cache:
                    return cache[i]
                n = nodes[i]
                t = chans.get((i, 'translation'))
                r = chans.get((i, 'rotation'))
                lt = list(t[min(f, len(t) - 1)]) if t else list(n.get('translation', [0, 0, 0]))
                lr = tuple(r[min(f, len(r) - 1)]) if r else tuple(n.get('rotation', [0, 0, 0, 1]))
                if i in parent:
                    pp, pq = world(parent[i])
                    p = [pp[k] + v for k, v in enumerate(qrot(pq, lt))]
                    q = qmul(pq, lr)
                else:
                    p, q = lt, lr
                cache[i] = (p, q)
                return cache[i]
            P = {b: world(index[b])[0] for b in LEGS}
            lx, lz = P["RightUpLeg"][0] - P["LeftUpLeg"][0], P["RightUpLeg"][2] - P["LeftUpLeg"][2]
            ll = math.hypot(lx, lz)
            lx, lz = lx / ll, lz / ll
            fx, fz = -lz, lx  # horizontal perpendicular (walking direction up to sign)
            mx = 0.5 * (P["RightUpLeg"][0] + P["LeftUpLeg"][0])
            mz = 0.5 * (P["RightUpLeg"][2] + P["LeftUpLeg"][2])

            def lat(p):
                return (p[0] - mx) * lx + (p[2] - mz) * lz

            def fwd(p):
                return (p[0] - mx) * fx + (p[2] - mz) * fz
            stats['n'] += 1
            stats['hip'] += lat(P["RightUpLeg"]) - lat(P["LeftUpLeg"])
            a_sep = lat(P["RightFoot"]) - lat(P["LeftFoot"])
            k_sep = lat(P["RightLeg"]) - lat(P["LeftLeg"])
            stats['ankAll'].append(a_sep)
            stats['kneeAll'].append(k_sep)
            stats['shin'] = min(stats['shin'], seg_dist(P["LeftLeg"], P["LeftFoot"], P["RightLeg"], P["RightFoot"]))
            if abs(fwd(P["RightFoot"]) - fwd(P["LeftFoot"])) < 0.15:
                stats['pas'] += 1
                stats['ank'].append(a_sep)
                stats['knee'].append(k_sep)
                if a_sep < 0:
                    stats['cross'] += 1
        n = stats['n']
        avg = lambda v: sum(v) / len(v) if v else float('nan')  # noqa: E731
        print(f"{name}: frames={n} hipWidth={100 * stats['hip'] / n:.1f}cm ankleSep mean={100 * avg(stats['ankAll']):.1f}cm "
              f"kneeSep mean={100 * avg(stats['kneeAll']):.1f}cm | passing ({stats['pas']}): ankleSep mean={100 * avg(stats['ank']):.1f} "
              f"min={100 * min(stats['ank'] or [float('nan')]):.1f}cm kneeSep mean={100 * avg(stats['knee']):.1f} "
              f"min={100 * min(stats['knee'] or [float('nan')]):.1f}cm cross={stats['cross']} shin min={100 * stats['shin']:.1f}cm "
              f"knee/ankle={avg(stats['kneeAll']) / avg(stats['ankAll']):.2f}")


if __name__ == '__main__':
    main()
