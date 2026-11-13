"""Offline port of the walk PLANNER of plugin\\UTPP\\src\\input\\ApproachCore.hpp (Approach::Plan, SegmentOk, PolyDist, BoxPoly, SectorPoly; Params defaults read here as constants) to
report the path V walks from a pre-mount position to a clip start spot - in particular the detour waypoints around the vehicle box (margin 0.35 m) and the closed door's sweep sector.

  python -I approach_plan.py --spots <sources-final.txt> --start -2.2,0.0 [--start -1.8,-0.7 ...] [--json out.json]     (spots measured from the sources' clips, entry_approach.json frame)
  python -I approach_plan.py --table <entry_approach.json> --start ...                                                  (spots of the table)

Only the geometry is ported (float32 rounding aside); the controller is not. The scene is the one the reds feeds (utpp_vehicle_approach.reds -> N_ApproachSetScene): box = entry_approach.json
'box' with margin 0.35, sector = SectorPoly(hinge, length, closedDeg, openDeg, extraDeg 8). Begin() order: runUp point 0.6 m behind the spot along its heading when the walk is
> 0.9 m and the bearing differs from the spot yaw by > 35 deg; a plan longer than maxPathM 3.0 is not started (TooFar), no plan = NoPath.
"""
import argparse
import json
import math
import os

KMAXPATH = 8
RUNUP_M, RUNUP_MIN_DIST_M, RUNUP_ANGLE_DEG, EXEMPT_M, MAX_PATH_M = 0.6, 0.9, 35.0, 0.7, 3.0


def rad(d):
    return d * math.pi / 180.0


def yaw_of(d):
    return math.degrees(math.atan2(-d[0], d[1]))


def fwd(yaw):
    return (-math.sin(rad(yaw)), math.cos(rad(yaw)))


def wrap(a):
    a = math.fmod(a, 360.0)
    return a - 360.0 if a > 180.0 else (a + 360.0 if a <= -180.0 else a)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def mul(a, k):
    return (a[0] * k, a[1] * k)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def seg_dist(p, a, b):
    ab = sub(b, a)
    l2 = ab[0] ** 2 + ab[1] ** 2
    t = min(max(((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2, 0.0), 1.0) if l2 > 1e-12 else 0.0
    return dist(p, add(a, mul(ab, t)))


class Poly:
    def __init__(self, v, margin):
        self.v, self.margin = list(v), margin

    def centroid(self):
        n = len(self.v)
        return (sum(p[0] for p in self.v) / n, sum(p[1] for p in self.v) / n)


def poly_dist(g, p):
    """Distance to the polygon, 0 inside (the C++: inside when all cross products have one sign)."""
    pos = neg = False
    best = 1e9
    n = len(g.v)
    for i in range(n):
        a, b = g.v[i], g.v[(i + 1) % n]
        c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        pos = pos or c > 1e-9
        neg = neg or c < -1e-9
        best = min(best, seg_dist(p, a, b))
    return best if (pos and neg) else 0.0


def box_poly(cx, cy, hx, hy, margin):
    return Poly([(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)], margin)


def sector_poly(hinge, length, closed_deg, open_deg, extra_deg):
    r = (length + 0.20) * 1.02
    sgn = -1.0 if open_deg < 0.0 else 1.0
    a0 = closed_deg - sgn * extra_deg
    span = abs(open_deg) + 2.0 * extra_deg
    v = [hinge]
    for i in range(7):
        a = rad(a0 + sgn * span * i / 6.0)
        v.append(add(hinge, mul((math.cos(a), math.sin(a)), r)))
    return Poly(v, 0.15)


def segment_ok(polys, a, b, target):
    ln = dist(a, b)
    n = max(2, int(math.ceil(ln / 0.05)))
    for g in polys:
        best = poly_dist(g, a)
        for i in range(1, n + 1):
            p = add(a, mul(sub(b, a), i / n))
            d = poly_dist(g, p)
            exempt = dist(p, target) <= EXEMPT_M
            if not exempt and d < min(g.margin, best) - 0.03:
                return False
            best = max(best, d)
    return True


def plan(box, sector, start, spot, spot_yaw):
    polys = [box] + ([sector] if sector else [])
    pts = [start]
    bearing = yaw_of(sub(spot, start))
    if dist(start, spot) > RUNUP_MIN_DIST_M and abs(wrap(bearing - spot_yaw)) > RUNUP_ANGLE_DEG:
        pre = sub(spot, mul(fwd(spot_yaw), RUNUP_M))
        if poly_dist(box, pre) > 0.05:
            pts.append(pre)
    pts.append(spot)
    cand = []
    for g in polys:
        c = g.centroid()
        off = max(g.margin, 0.35) + 0.15
        for v in g.v:
            d = sub(v, c)
            l = math.hypot(*d)
            if l < 1e-4:
                continue
            w = add(v, mul(d, off / l))
            if poly_dist(box, w) >= box.margin - 0.03:
                cand.append(w)
    out = [pts[0]]
    for s in range(1, len(pts)):
        a, b = pts[s - 1], pts[s]
        if segment_ok(polys, a, b, spot):
            out.append(b)
            continue
        best, bw = 1e9, None
        for c in cand:
            if segment_ok(polys, a, c, spot) and segment_ok(polys, c, b, spot):
                l = dist(a, c) + dist(c, b)
                if l < best:
                    best, bw = l, [c]
        if bw is None:
            for i, ci in enumerate(cand):
                for j, cj in enumerate(cand):
                    if i == j:
                        continue
                    l = dist(a, ci) + dist(ci, cj) + dist(cj, b)
                    if l < best and segment_ok(polys, a, ci, spot) and segment_ok(polys, ci, cj, spot) and segment_ok(polys, cj, b, spot):
                        best, bw = l, [ci, cj]
        if bw is None or len(out) + len(bw) + 1 > KMAXPATH:
            return None
        out += bw + [b]
    return out


def scene_of(cls_entry, seat):
    bx = cls_entry['box']
    box = box_poly(bx['cx'], bx['cy'], bx['hx'], bx['hy'], 0.35)
    side = 'left' if seat == 'driver' else 'right'
    ar = (cls_entry.get('arcs') or {}).get(side)
    sector = sector_poly((ar['hingeX'], ar['hingeY']), ar['length'], ar['closedDirDeg'], ar['openDeg'], 8.0) if ar else None
    return box, sector


def run(entry_json, starts):
    tj = json.load(open(entry_json, encoding='utf-8'))
    res = {}
    for cls, ce in sorted(tj['classes'].items()):
        for seat, se in ce['seats'].items():
            box, sector = scene_of(ce, seat)
            for body, sp in se['spots'].items():
                for st in starts:
                    p = plan(box, sector, st, (sp['x'], sp['y']), sp['yaw_deg'])
                    ln = sum(dist(p[i - 1], p[i]) for i in range(1, len(p))) if p else None
                    res['%s %s %s from %s' % (cls, seat, body, st)] = dict(spot=(sp['x'], sp['y']), path=[(round(a, 2), round(b, 2)) for a, b in p] if p else None,
                                                                          length=round(ln, 2) if ln is not None else None, started=bool(p and ln <= MAX_PATH_M))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--table', required=True, help='entry_approach.json')
    ap.add_argument('--start', action='append', required=True, help='x,y pre-mount position (vehicle frame)')
    ap.add_argument('--json')
    a = ap.parse_args()
    starts = [tuple(float(v) for v in s.split(',')) for s in a.start]
    res = run(a.table, starts)
    for k, v in res.items():
        print('%-52s spot %-16s %s len %s %s' % (k, v['spot'], v['path'], v['length'], 'started' if v['started'] else 'NOT STARTED (vanilla slide)'))
    if a.json:
        json.dump(res, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
