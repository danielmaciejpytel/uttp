"""Freeze the 30 measured mount spots of the car-door2 game test (2026-11-02) into approach_fixtures.json for check_door_clearance.py --approach.

  python -I build_approach_fixtures.py [--diag <car-door2-...-diag dir>] [--logs <car-door2 gametest dir>] [--out approach_fixtures.json]

Inputs (read only): <diag>\\slide\\slide_table.json + slide_series.json (per ride: mount spot, hold mode, logged hold statistics, entity vtpp series),
<logs>\\<vehicle>\\*-lines.txt ('[night] POS p= cf= v=' sample before the mount line -> V's heading in the vehicle frame).
Output per ride: class / seat / case / mount spot (vehicle frame: x right, y forward, z up, m) / heading (deg, atan2(fy, fx) of V's forward in the vehicle
frame, null when the ride has no POS sample before the mount) / logged native hold statistics / the slide duration T fitted on the entity series
(progress = sin(pi/2 (t - t0) / T) along the straight line mount spot -> clip start, the vanilla slide) and the per-class median of T.
"""
import argparse
import glob
import json
import math
import os
import re
import statistics

import numpy as np
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

DIAG = os.path.join(project_root(), r'Research\logs\gametest\car-door2-20261102-diag')
LOGS = os.path.join(project_root(), r'Research\logs\gametest\car-door2-20261102')
POS = re.compile(r'\[night\] POS p=([-\d.]+),([-\d.]+),([-\d.]+) cf=([-\d.]+),([-\d.]+) v=([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+)')
TS = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3})\]')
MOUNT = re.compile(r'\[utpp_vehicle_anim\] ride (\d+): workspot animVariables')


def stamp(line):
    m = TS.match(line)
    if not m:
        return None
    import datetime
    return datetime.datetime.strptime(m.group(1), '%Y-%m-%d %H:%M:%S.%f').timestamp()


def headings(logs):
    """{reds ride number: (heading deg in the vehicle frame, mount spot xy from the POS sample)} from the POS sample just before the mount line."""
    out = {}
    for f in glob.glob(os.path.join(logs, '*', '*-lines.txt')):
        lines = open(f, encoding='utf-8', errors='replace').read().splitlines()
        for i, l in enumerate(lines):
            m = MOUNT.search(l)
            if not m:
                continue
            t0 = stamp(l)
            best = None
            for ll in lines[max(0, i - 60):i + 5]:
                p = POS.search(ll)
                t = stamp(ll)
                if p and t is not None and -0.35 <= t - t0 <= 0.05:
                    best = p
            if best is None:
                continue
            px, py, pz, cfx, cfy, vx, vy, vz, a, b = [float(best.group(k)) for k in range(1, 11)]
            right, fwd = (a, b), (-b, a)
            d = (px - vx, py - vy)
            xy = (d[0] * right[0] + d[1] * right[1], d[0] * fwd[0] + d[1] * fwd[1])
            f_v = (cfx * right[0] + cfy * right[1], cfx * fwd[0] + cfy * fwd[1])
            out[int(m.group(1))] = (math.degrees(math.atan2(f_v[1], f_v[0])), xy)
    return out


def fit_slide(ser, start_xy):
    """(T, t0) of progress = sin(pi/2 (t - t0) / T) (clamped 0..1) fitted on the raw entity samples of a ride up to the end of the slide (the entity reaches the
    clip start `start_xy`; the samples after that are the clip's own root motion)."""
    t = np.array(ser['samples_t'])
    p = np.array(ser['samples'])
    S = np.array(start_xy, float)
    D = float(np.linalg.norm(S - p[0, :2]))
    if D < 0.3:
        return None
    u = (p[:, :2] - p[0, :2]) @ (S - p[0, :2]) / D / D
    k = np.flatnonzero(u >= 0.97)
    n = (int(k[0]) + 2) if len(k) else len(t)
    t, u = t[:n], np.clip(u[:n], 0.0, 1.0)
    best = None
    for T in np.arange(0.15, 1.0, 0.005):
        for t0 in np.arange(-0.03, 0.08, 0.005):
            q = np.sin(np.pi / 2 * np.clip((t - t0) / T, 0, 1))
            e = float(np.sum((q - u) ** 2))
            if best is None or e < best[0]:
                best = (e, float(T), float(t0))
    return {'T': round(best[1], 3), 't0': round(best[2], 3), 'rms': round(math.sqrt(best[0] / len(t)), 3), 'dist_m': round(D, 3), 'n_samples': int(len(t))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--diag', default=DIAG)
    ap.add_argument('--logs', default=LOGS)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'approach_fixtures.json'))
    a = ap.parse_args()
    st = json.load(open(os.path.join(a.diag, 'slide', 'slide_table.json'), encoding='utf-8'))
    series = json.load(open(os.path.join(a.diag, 'slide', 'slide_series.json'), encoding='utf-8'))
    hd = headings(a.logs)
    rides, per_cls = [], {}
    for r in st['rides']:
        fit = fit_slide(series[str(r['ride'])], r['clip_start'][:2])
        h = hd.get(r['ride'])
        if h is not None:
            err = math.hypot(h[1][0] - r['mount_spot'][0], h[1][1] - r['mount_spot'][1])
            if err > 0.06:  # the POS sample is not the mount spot (walking): the heading is of another moment
                h = None
        rides.append({'ride': r['ride'], 'vehicle': r['vehicle'], 'cls': r['cls'], 'seat': r['seat'], 'case': r['case'], 'review': r['fail'],
                      'clip': r['clip'], 'mount_spot': r['mount_spot'], 'mount_heading_deg': None if h is None else round(h[0], 1),
                      'clip_start_logged': r['clip_start'], 'seat_pos_logged': r['seat_pos'], 'vt_mode': r['vt_mode'], 'rootEngageAt': r['rootEngageAt'],
                      'rootPeakDist_log': r['rootPeakDist_log'], 'rootOnlyWrites': r['rootOnlyWrites'], 'anchor_logged': r['anchor'], 'door': r['door'],
                      'door_E_s': r['door_E_s'], 'slide_m': r['slide_m'], 'slide_fit': fit})
        if fit and fit['dist_m'] >= 0.5:
            per_cls.setdefault(r['cls'], []).append(fit['T'])
    cls_T = {c: round(statistics.median(v), 3) for c, v in per_cls.items()}
    json.dump({'_note': 'frozen from the car-door2 game test 2026-11-02 (build_approach_fixtures.py); vehicle frame x right, y forward, z up, m; female V (pwa) clips',
               'slide_T_by_class_s': cls_T, 'slide_T_samples': per_cls, 'rides': rides}, open(a.out, 'w', encoding='utf-8'), indent=1)
    print('wrote %s: %d rides, headings for %d, slide T by class %s' % (a.out, len(rides), sum(1 for r in rides if r['mount_heading_deg'] is not None), cls_T))


if __name__ == '__main__':
    main()
