"""Speeds / slide episodes measured in the recorded in-game feed dumps (utpp-feed-*.txt), read only.

  python -I loco_feed_stats.py [feed dump ...]      (default: every utpp-feed-*.txt of the game log folder)
Purpose: parameters of the synthetic locomotion feeds of the comparison films (loco_feeds.py): walk / run / sprint / crouch capsule
speeds, the capsule acceleration and braking, and the slide (PSM LocomotionDetailed 5) episodes: duration, entry / exit speed.
Feed dump columns (MMDriver feed dump): t dt vx vy vz mvx mvy yawDeg camDeg loco psm ... (first 17 = the .feed format).
loco: 1 stand, 2 crouch, 3 sprint; PSM detail = ((psm >> 16) & 0xFF) - 1 (5 = slide).  Velocities in m/s (game units).
Prints a small table; writes nothing.
"""
import glob
import math
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))


def load(path):
    rows = []
    for ln in open(path, encoding='utf-8', errors='replace'):
        if not ln.strip() or ln[0] == '#':
            continue
        p = ln.split()
        if len(p) < 17:
            continue
        try:
            rows.append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), float(p[5]), float(p[6]), int(p[9]), int(p[10]) & 0xFFFFFFFF, float(p[7])))
        except ValueError:
            continue
    a = np.array(rows, float) if rows else np.zeros((0, 9))
    return a


def main(paths):
    acc = {1: [], 2: [], 3: []}
    acc_full = {1: [], 2: [], 3: []}
    slides = []
    accel = []
    brake, yawr = [], []
    for p in paths:
        a = load(p)
        if len(a) < 10:
            continue
        sp = np.hypot(a[:, 2], a[:, 3])
        stick = np.hypot(a[:, 4], a[:, 5])
        loco = a[:, 6].astype(int)
        det = ((a[:, 7].astype(np.int64) >> 16) & 0xFF) - 1
        for k in acc:
            m = (loco == k) & (sp > 0.3) & (det != 5)
            acc[k] += list(sp[m])
            acc_full[k] += list(sp[m & (stick > 0.95)])
        # acceleration from rest to moving (stand / sprint), dv/dt over consecutive rows with dt > 0
        dv = np.diff(sp) / np.maximum(a[1:, 1], 1e-3)
        accel += list(dv[(sp[:-1] < 3.0) & (stick[1:] > 0.9) & (a[1:, 1] > 0.005) & (dv > 0)])
        brake.extend(list(-dv[(sp[:-1] > 0.5) & (stick[1:] < 0.05) & (a[1:, 1] > 0.005) & (dv < 0) & (det[1:] != 5)]))
        yaw = np.unwrap(np.radians(a[:, 8]))
        yr = np.degrees(np.abs(np.diff(yaw))) / np.maximum(a[1:, 1], 1e-3)
        yawr.extend(list(yr[(sp[1:] > 1.0) & (a[1:, 1] > 0.005) & (yr > 30)]))
        i = 0
        while i < len(a):
            if det[i] == 5:
                j = i
                while j + 1 < len(a) and det[j + 1] == 5:
                    j += 1
                slides.append((os.path.basename(p), a[i, 0], a[j, 0] - a[i, 0] + a[j, 1], sp[max(i - 1, 0)], sp[i], sp[j], loco[max(i - 1, 0)],
                               loco[min(j + 1, len(a) - 1)]))
                i = j + 1
            else:
                i += 1
    for k, nm in ((1, 'stand'), (2, 'crouch'), (3, 'sprint')):
        v = np.array(acc[k])
        f = np.array(acc_full[k])
        if len(v):
            print('%-7s moving n=%6d speed p50 %.2f p90 %.2f p99 %.2f | full stick n=%6d p50 %.2f p90 %.2f' %
                  (nm, len(v), np.percentile(v, 50), np.percentile(v, 90), np.percentile(v, 99), len(f),
                   np.percentile(f, 50) if len(f) else math.nan, np.percentile(f, 90) if len(f) else math.nan))
    if accel:
        print('accel (speed rising, full stick) p50 %.1f p90 %.1f m/s2' % (np.percentile(accel, 50), np.percentile(accel, 90)))
    if brake:
        print('braking (stick released) p50 %.1f p90 %.1f m/s2' % (np.percentile(brake, 50), np.percentile(brake, 90)))
    if yawr:
        print('actor yaw rate while turning (>30 deg/s, moving) p50 %.0f p90 %.0f p99 %.0f deg/s' % tuple(np.percentile(yawr, [50, 90, 99])))
    print('slides (PSM detail 5): %d' % len(slides))
    for s in slides[:40]:
        print('  %s t=%.2f dur %.2f s speed before %.2f entry %.2f exit %.2f loco before %d after %d' % s)


if __name__ == '__main__':
    args = sys.argv[1:] or sorted(glob.glob(os.path.join(ROOT, 'Research', 'logs', 'game', 'utpp-feed-*.txt')))
    main(args)
