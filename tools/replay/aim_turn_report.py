#!/usr/bin/env python3
"""Turns in place while aiming, from recorded game sessions: how far the camera turned, whether the motion driver picked a
turn-in-place clip, how long it took and how far the body and the capsule were left behind.

Read only. Inputs are the game-test session folders (Research/logs/night/<run>/) of a run with the ranged family phase
`aim_turns` (scenario `ranged_aim_turns`, family copies rf_<family>_aim_turns): the feed dump (logs/utpp-feed-*.txt), the
plugin log (logs/utpp-<stamp>.log: MOTION trace lines and the harness POS lines) and the step windows (gametest/*_r<n>.txt).
Feed and log clocks are aligned as in speed_calibration.py (harness POS lines matched to the feed capsule path).

    python -I aim_turn_report.py <run folder> [<run folder> ...] [--json out.json]

Per turn step (turn_<nominal>_<r|l>; the nominal angle in the name is only the target of the pulse length):
  pan_deg        camera yaw change over the step (feed camDeg, unwrapped; + = to the right as the feed counts it)
  actor_deg      player entity yaw change over the step (feed yawDeg)
  lag_end_deg    |camera - entity yaw| at the end of the step (what is left behind after the 2.5 s hold)
  lag_max_deg    the largest |camera - entity yaw| inside the step
  tip            a turn-in-place clip (name matching TIP_RE) was picked inside the step; tip_clip, tip_delay_s (from the step start)
  capsule_cm     largest horizontal capsule distance from where the step began (a turn in place should not move V)
  aim_fraction   share of the step's feed rows with the aim state (PSM UpperBody 6)
Summary per angle class of the measured |pan_deg| (CLASSES) and over all turns of at least 45 deg: count, TIP share, lag and capsule
movement (median / max). The planted-foot slide of each turn comes from world_slide.py, whose per-step windows are these steps.
"""
import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import speed_calibration as scal  # noqa: E402

TURN_STEP = re.compile(r'^turn_(\d+)_([rl])$')
SCENARIO = re.compile(r'^rf_(\w+)_aim_turns(?:#\d+)?$')
TIP_RE = re.compile(r'Stand_Turn|Turn_?In_?Place|_TIP_', re.I)
CLASSES = ((0.0, 37.5, '30'), (37.5, 52.5, '45'), (52.5, 75.0, '60'), (75.0, 135.0, '90'), (135.0, 1e9, '180+'))
GATE_MIN_DEG = 45.0
CAM_COL, YAW_COL = 8, 7


def read_angles(path):
    """camDeg and yawDeg columns of a feed dump, the rows speed_calibration.read_feed keeps (same filter)."""
    cam, yaw = [], []
    with open(path, encoding='utf-8', errors='replace') as f:
        for ln in f:
            if not ln.strip() or ln[0] == '#':
                continue
            p = ln.split()
            if len(p) < scal.FEED_COLS:
                continue
            cam.append(scal._num(p[CAM_COL]))
            yaw.append(scal._num(p[YAW_COL]))
    return np.unwrap(np.radians(np.asarray(cam, float))), np.unwrap(np.radians(np.asarray(yaw, float)))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def angle_class(pan):
    a = abs(pan)
    return next(lab for lo, hi, lab in CLASSES if lo <= a < hi)


def turns_of_run(run, cam, yaw):
    out = []
    f = run.feed
    for scen, step, s0, s1, _verdict in run.steps:
        m = TURN_STEP.match(step)
        sm = SCENARIO.match(scen)
        if not m or not sm:
            continue
        i0, i1 = run.row_at(s0), run.row_at(s1)
        if i1 - i0 < 5:
            continue
        c, y = np.degrees(cam[i0:i1 + 1]), np.degrees(yaw[i0:i1 + 1])
        lag = np.abs(wrap180(c - y))
        dx, dy = f['x'][i0:i1 + 1] - f['x'][i0], f['y'][i0:i1 + 1] - f['y'][i0]
        picks = [mo for mo in run.log['motion'] if s0 <= mo['wall'] - 0.0 <= s1 and TIP_RE.search(mo['name'] or '')]
        upper = f['upper'][i0:i1 + 1]
        out.append({
            'run': run.name, 'family': sm.group(1), 'step': step, 'nominal_deg': int(m.group(1)), 'dir': m.group(2),
            'pan_deg': round(float(c[-1] - c[0]), 1), 'actor_deg': round(float(y[-1] - y[0]), 1),
            'lag_end_deg': round(float(lag[-1]), 1), 'lag_max_deg': round(float(np.nanmax(lag)), 1),
            'tip': bool(picks), 'tip_clip': picks[0]['name'] if picks else None,
            'tip_delay_s': round(picks[0]['wall'] - s0, 2) if picks else None,
            'capsule_cm': round(float(np.nanmax(np.hypot(dx, dy))) * 100.0, 1),
            'aim_fraction': round(float(np.mean(upper == scal.UPPER_AIM)), 2),
            'aligned': bool(run.aligned)})
    for t in out:
        t['class'] = angle_class(t['pan_deg'])
    return out


def summarize(turns):
    def block(ts):
        if not ts:
            return {'n': 0}
        lag = [t['lag_end_deg'] for t in ts]
        cap = [t['capsule_cm'] for t in ts]
        return {'n': len(ts), 'tip_share': round(sum(t['tip'] for t in ts) / len(ts), 3),
                'lag_end_p50': round(float(np.median(lag)), 1), 'lag_max': round(float(max(t['lag_max_deg'] for t in ts)), 1),
                'capsule_cm_p50': round(float(np.median(cap)), 1), 'capsule_cm_max': round(float(max(cap)), 1)}
    aimed = [t for t in turns if t['aim_fraction'] >= 0.8]
    out = {'classes': {lab: block([t for t in aimed if t['class'] == lab]) for _, _, lab in CLASSES},
           'at_least_45': block([t for t in aimed if abs(t['pan_deg']) >= GATE_MIN_DEG]),
           'not_aimed': len(turns) - len(aimed), 'unaligned_runs': sorted({t['run'] for t in turns if not t['aligned']})}
    return out


def report(folders):
    turns, runs = [], []
    for folder in folders:
        run = scal.Run(folder)
        feeds = sorted(glob.glob(os.path.join(folder, 'logs', 'utpp-feed-*.txt')))
        if run.feed is None or not feeds:
            runs.append({'run': run.name, 'note': 'no feed dump'})
            continue
        cam, yaw = read_angles(feeds[0])
        mine = turns_of_run(run, cam, yaw)
        runs.append({'run': run.name, 'turns': len(mine), 'aligned': run.aligned, 'align_mad_s': run.align_mad})
        turns += mine
    return {'runs': runs, 'turns': turns, 'summary': summarize(turns)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+', help='session folders (Research/logs/night/<run>)')
    ap.add_argument('--json', help='write the result here')
    a = ap.parse_args(argv)
    res = report(a.runs)
    for t in res['turns']:
        print(f"{t['run']} {t['family']:14} {t['step']:11} pan {t['pan_deg']:7.1f} actor {t['actor_deg']:7.1f} "
              f"lag end {t['lag_end_deg']:5.1f} max {t['lag_max_deg']:5.1f} tip {'yes' if t['tip'] else 'no ':3} "
              f"{t['tip_clip'] or '-'} capsule {t['capsule_cm']:.1f} cm aim {t['aim_fraction']:.2f}")
    s = res['summary']
    for lab, b in list(s['classes'].items()) + [('>=45', s['at_least_45'])]:
        if b['n']:
            print(f"class {lab:5}: n {b['n']:3} TIP {b['tip_share'] * 100:5.1f} % lag end p50 {b['lag_end_p50']:5.1f} "
                  f"max {b['lag_max']:5.1f} capsule p50 {b['capsule_cm_p50']:.1f} max {b['capsule_cm_max']:.1f} cm")
    if s['unaligned_runs']:
        print('WARNING clocks not aligned (no POS match): ' + ', '.join(s['unaligned_runs']))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(res, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
