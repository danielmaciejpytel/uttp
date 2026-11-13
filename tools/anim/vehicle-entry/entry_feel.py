"""Driver-side entry FEEL gates (r13): where the approach ends, how far V travels along the car before she steps in, whether she slides to the rear, where the legs
cross the aperture edge, and the head / hair clearance under the roof during the transition. Offline, vehicle frame (x right, y forward, z up), 30 fps clip,
entity root = the clip root mapped by seat * inverse(root delta) (the same map as check_door_clearance.Model / measure_entry_paths).

  python -I entry_feel.py --sources <sources-final-*.txt> [--seats driver] [--vanilla] [--logs <utpp-*.log ...>] [--json out.json]
  python -I entry_feel.py --glb <clip.anims.glb> ...  [--selftest]

Why these gates exist (review of the installed r6a pack, driver side; the older gates measured the HIPS crossing the car side against the door arc and the shell and
all passed): the visible defects were (1) the approach ends 0.3-1.0 m behind the door opening and V then walks 0.5-0.9 m forward ALONG the car side before she steps in,
(2) a rearward slide while she sits, (3) a 4-14 cm jump between the last approach sample and the first clip sample, (4) head / hair at the roof rail.

Gates (a row passes when every line is ok):
 E1 START        start-spot y not more than START_FWD_TOL_M (0.15 m) behind the most forward spot the door swing allows at the same lateral distance (outside the swept wedge
                 of the opening door + 0.35 m = body radius 0.20 as in UTPPDoorCapOf / door_approach.py + 0.15 m arrival residual of the native approach walk). The door opens from the mount event, so V cannot stand in front of
                 the opening inside the swept wedge: the literal 'between the A- and B-pillar' is not reachable, 'as far forward as the swing allows' is.
 E2 ALONG-CAR   root forward travel from the start until the root reaches the car side line: <= ALONG_CAR_MAX_M (0.35 m).
 E3 REARWARD     root back excursion (travel back from the running forward-most y) <= REAR_MAX_M (0.05 m) and sink slide of the Hips <= REAR_MAX_M. Absolute. The Hips excursion
                 is printed but not gated: the game's own clips settle the pelvis back 12 cm.
 E4 LEGS         hips, thighs, KNEES and spine crossing the side line: y at the crossing minus the aperture rear edge at that height (shell hole, not the arc) minus the limb
                 radius >= EDGE_MARGIN_M (0 = no overlap with the B-pillar line); knees are new (the older rule tracked hips, thighs and spine). Feet, toes and hands are printed only.
 E5 ROOF         head sphere (r 0.10 + 0.05 hair, centre 0.09 above the Head joint) vs the roof cloud on every frame before the pinned seated tail (last 18 frames):
                 clearance (-penetration) >= HAIR_MARGIN_CM (2.0 cm) beyond the hair allowance. The seated frame keeps its own gate (seated + 0.5 cm, check_door_clearance 'roof').
 E6 MOUNT JUMP   the approach table spot equals the clip's frame-0 root (<= 1 cm) and, with --logs, every logged approach end has residual <= JUMP_MAX_M (0.03 m): that residual
                 is the jump from the last approach sample to the first clip sample (native arrival gate, ApproachCore.hpp arriveGateM).
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import check_door_clearance as CD  # noqa: E402
import measure_entry_paths as MEP  # noqa: E402
import start_gate as SG  # noqa: E402

START_FWD_TOL_M = 0.15
DOOR_DISK_MARGIN_M = 0.35  # 0.15 body radius + 0.05 + the logged arrival residual of the native approach walk (r6a game test: 0.06-0.15 m); 0.20 alone put the left thigh into the opening door in the arrival ring
WEDGE_MARGIN_DEG = 8.0
ALONG_CAR_MAX_M = 0.35
REAR_MAX_M = 0.05
EDGE_MARGIN_M = 0.0
HAIR_MARGIN_CM = 2.0
TAIL_FRAMES = 18
JUMP_MAX_M = 0.03
SPOT_TOL_M = 0.01
GATED_JOINTS = ('Hips', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'Spine', 'Spine2')  # E4 gates the pillar-sensitive joints; feet, toes and hands are reported only (they rest on the sill / frame in the game's own clips)
LEG_JOINTS = {'Hips': 0.11, 'LeftUpLeg': 0.085, 'RightUpLeg': 0.085, 'LeftLeg': 0.055, 'RightLeg': 0.055, 'LeftFoot': 0.05, 'RightFoot': 0.05,
              'LeftToeBase': 0.04, 'RightToeBase': 0.04, 'Spine': 0.14, 'Spine2': 0.15, 'LeftHand': 0.04, 'RightHand': 0.04}
APPROACH_TABLE = os.path.join(HERE, 'entry_approach.json')
END_RE = re.compile(r'utpp_vehicle_approach\] end reason=(\S+) mount=(\d).*? residual=([0-9.]+)')
SKIP0_RE = re.compile(r'utpp_vehicle_approach\] skip code=0 .*?from=\(([-0-9.]+),([-0-9.]+),.*?\) to=\(([-0-9.]+),([-0-9.]+),')  # 'already at the spot': the mount ran at once, the engine slide covers this distance


def feasible_start_y(cls, side, x, tj, edge_y, seat_y):
    """Most forward start y (vehicle frame) at lateral position x that stays outside the closing door's swept wedge + DOOR_DISK_MARGIN_M, scanned down from the door centre.
    Classes without a valid arc (lifting door) return the door centre (no swing limit)."""
    a = tj['arcs'].get(cls, {}).get(side)
    if not a or not a.get('valid', True):
        return None
    hx, hy, L = a['hingeX'], a['hingeY'], a['length']
    d0 = math.radians(a['closedDirDeg'])
    d1 = math.radians(a['closedDirDeg'] + a['openDeg'])
    lo, hi = min(d0, d1) - math.radians(WEDGE_MARGIN_DEG), max(d0, d1) + math.radians(WEDGE_MARGIN_DEG)
    R = L + DOOR_DISK_MARGIN_M
    y_top = 0.5 * (edge_y + hy)
    y = y_top
    while y > edge_y - 3.0:
        dx, dy = x - hx, y - hy
        ang = math.atan2(dy, dx)
        if not (math.hypot(dx, dy) < R and lo <= ang <= hi):
            return float(y)
        y -= 0.01
    return float(y)


def tracks(path, cls, seat):
    old = MEP.TRACK
    MEP.TRACK = [('Root', 'Root')] + [(k, k) for k in LEG_JOINTS] + [('Head', 'Head')]
    try:
        t, J = MEP.joint_paths(path, cls, seat)
    finally:
        MEP.TRACK = old
    return t, J


def _cross(P, sg, x_out):
    """(frame index (float), point) at which the track first goes inboard of the side line (|x| < x_out), None when it never does or starts inboard."""
    d = sg * P[:, 0] - x_out
    k = np.flatnonzero(d <= 0)
    if not len(k):
        return None
    i = int(k[0])
    if i == 0:
        return 0.0, P[0]
    u = d[i - 1] / (d[i - 1] - d[i]) if d[i - 1] != d[i] else 0.0
    return i - 1 + u, P[i - 1] * (1 - u) + P[i] * u


def evaluate(path, cls, seat, tj=None, mutate=None, spot=None):
    """mutate: dict(shift_y=m (all tracks, rearward negative), bump=(m, f0, f1), head_shift=(dx, dy, dz) m (head sphere, transition frames only))."""
    tj = tj or CD.load_timing()
    mut = mutate or {}
    side = 'left' if seat == 'driver' else 'right'
    sg = -1.0 if side == 'left' else 1.0
    t, J = tracks(path, cls, seat)
    if mut.get('shift_y'):
        for k in J:
            J[k] = J[k] + np.array([0.0, float(mut['shift_y']), 0.0])
    if mut.get('bump'):  # smooth mid-clip rearward bump of every track (sin^2 over the clip fraction f0..f1), the mutation that E3 must reject
        amp, f0, f1 = mut['bump']
        tt = np.arange(len(t)) / max(1.0, len(t) - 1.0)
        w = np.where((tt >= f0) & (tt <= f1), np.sin(np.pi * (tt - f0) / max(1e-6, f1 - f0)) ** 2, 0.0)
        for k in J:
            J[k] = J[k] + np.stack([np.zeros_like(w), -float(amp) * w, np.zeros_like(w)], axis=1)
    m = CD.Model(path, cls, seat)
    bd = m.bodyd
    x_out, y_edge_arc, _ = MEP.class_geom(cls, side, tj)
    zs, ys, _ = SG.aperture_rear(bd, side, SG.arc_edge_y(m))
    R = J['Root']
    out = {'row': os.path.basename(path), 'cls': cls, 'seat': seat, 'frames': len(t)}
    # E1
    x0, y0 = float(R[0, 0]), float(R[0, 1])
    yf = feasible_start_y(cls, side, x0, tj, y_edge_arc, float(bd['json']['seat'][side][1]))
    seat_t = np.array(bd['json']['seat'][side])
    out['start'] = [round(x0, 3), round(y0, 3)]
    out['start_feasible_y'] = None if yf is None else round(yf, 3)
    out['start_gap_m'] = None if yf is None else round(yf - y0, 3)
    out['E1'] = True if yf is None else (yf - y0) <= START_FWD_TOL_M + 1e-9
    # E2 along-car travel to the side line (root)
    c = _cross(R, sg, x_out)
    along = None if c is None else float(c[1][1] - y0)
    out['along_car_m'] = None if along is None else round(along, 3)
    out['E2'] = along is None or along <= ALONG_CAR_MAX_M + 1e-9
    # E3 rearward
    y_root, y_hips, z_hips = R[:, 1], J['Hips'][:, 1], J['Hips'][:, 2]
    rr = {'root': SG.back_excursion(y_root), 'Hips': SG.back_excursion(y_hips), 'sink': SG.sink_slide(y_hips, z_hips)}
    out['rear_m'] = {k: round(v, 3) for k, v in rr.items()}
    out['E3'] = rr['root'] <= REAR_MAX_M + 1e-9 and rr['sink'] <= REAR_MAX_M + 1e-9  # Hips excursion is reported only: the game's own clips lean the pelvis back 12 cm while she settles
    # E4 limbs crossing the side line vs the aperture rear edge
    legs, ok4 = {}, True
    for nm, rad in LEG_JOINTS.items():
        P = J[nm]
        cc = _cross(P, sg, x_out)
        if cc is None:
            continue
        p = cc[1]
        edge = float(np.interp(p[2], zs, ys))
        mg = float(p[1] - edge - rad)
        legs[nm] = round(mg, 3)
        if nm in GATED_JOINTS and mg < EDGE_MARGIN_M - 1e-9:
            ok4 = False
    out['legs_margin_m'] = legs
    out['E4'] = ok4
    # E5 roof, transition frames
    rc = bd['roof']
    n = len(t) - TAIL_FRAMES
    pens = []
    for i in range(n):
        v = (m.head_c[i] @ CD.GLTF_TO_VEH.T).astype(np.float32)
        if mut.get('head_shift'):  # vehicle-frame vector (x right, y forward, z up), m
            v = v + np.array(mut['head_shift'], np.float32)
        d = float(np.linalg.norm(rc - v, axis=1).min())
        pens.append((CD.ROOF_HEAD_R_M + CD.ROOF_HAIR_M - d) * 100.0)
    pens = np.array(pens)
    out['roof_transition_max_pen_cm'] = round(float(pens.max()), 2)
    out['roof_transition_frame'] = int(pens.argmax())
    out['E5'] = float(pens.max()) <= -HAIR_MARGIN_CM + 1e-9
    # E6 table spot vs clip start
    try:
        ap = json.load(open(APPROACH_TABLE, encoding='utf-8'))['classes']
        g = CD.parse_name(path)[0]
        sp = ap[cls]['seats'][seat]['spots']['female' if g == 'w' else 'male']
        dxy = math.hypot(sp['x'] - x0, sp['y'] - y0)
        out['table_spot'] = [sp['x'], sp['y']]
        out['spot_vs_clip_m'] = round(dxy, 3)
        out['E6_spot'] = dxy <= SPOT_TOL_M + 1e-9
    except Exception as e:  # noqa: BLE001
        out['E6_spot'] = None
        out['spot_err'] = str(e)
    out['ok'] = bool(out['E1'] and out['E2'] and out['E3'] and out['E4'] and out['E5'] and out['E6_spot'] is not False)
    return out


def logs_residuals(paths):
    res = []
    for p in paths:
        try:
            for line in open(p, encoding='utf-8', errors='replace'):
                m = END_RE.search(line)
                if m and m.group(2) == '1':
                    res.append((m.group(1), float(m.group(3))))
                    continue
                m = SKIP0_RE.search(line)  # r13c: the 'already there' mounts count as well (they used to slide up to 0.35 m)
                if m:
                    res.append(('already_there', math.hypot(float(m.group(1)) - float(m.group(3)), float(m.group(2)) - float(m.group(4)))))
        except OSError:
            pass
    return res


def fmt(o):
    f = lambda v: '-' if v is None else v  # noqa: E731
    leg = ' '.join('%s %.2f' % (k[:9], v) for k, v in sorted(o['legs_margin_m'].items(), key=lambda kv: kv[1])[:3])
    return ('%-34s E1 %s start (%+.2f,%+.2f) feasible y %s gap %s | E2 %s along %s | E3 %s root %.3f hips %.3f sink %.3f | E4 %s worst %s | E5 %s roof max %+.1f cm@f%d | E6 %s spot-clip %s | %s' % (
        o['row'].replace('.anims.glb', ''), 'ok' if o['E1'] else 'FAIL', o['start'][0], o['start'][1], f(o['start_feasible_y']), f(o['start_gap_m']),
        'ok' if o['E2'] else 'FAIL', f(o['along_car_m']), 'ok' if o['E3'] else 'FAIL', o['rear_m']['root'], o['rear_m']['Hips'], o['rear_m']['sink'],
        'ok' if o['E4'] else 'FAIL', leg, 'ok' if o['E5'] else 'FAIL', o['roof_transition_max_pen_cm'], o['roof_transition_frame'],
        {True: 'ok', False: 'FAIL', None: 'n/a'}[o['E6_spot']], f(o.get('spot_vs_clip_m')), 'PASS' if o['ok'] else 'FAIL'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources')
    ap.add_argument('--glb', nargs='*', default=[])
    ap.add_argument('--seats', default='driver')
    ap.add_argument('--vanilla', action='store_true')
    ap.add_argument('--logs', nargs='*', default=[])
    ap.add_argument('--json')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    tj = CD.load_timing()
    files = list(a.glb)
    if a.sources:
        files += MEP.parse_sources(a.sources)
    seats = a.seats.split(',')
    if a.vanilla:
        gd = os.path.join(os.path.dirname(CD.DATA), 'glb')
        for f in list(files):
            g, cls, seat = CD.parse_name(f)
            files.append(os.path.join(gd, 'p%sa_%s_%s.anims.glb' % (g, cls, seat)))
    allr, bad = [], 0
    for f in files:
        g, cls, seat = CD.parse_name(f)
        if seat not in seats:
            continue
        van = not os.path.basename(f).startswith('utpp_car_')
        o = evaluate(f, cls, seat, tj)
        o['vanilla'] = van
        allr.append(o)
        print(('VANILLA ' if van else '') + fmt(o))
        if not van and not o['ok']:
            bad += 1
        if a.selftest and not van:
            for nm, mu in (('rear-0.30m', {'shift_y': -0.30}), ('bump0.15m', {'bump': (0.15, 0.25, 0.75)})):
                mo = evaluate(f, cls, seat, tj, mutate=mu)
                print('    mutation %-10s -> E1 %s E2 %s E3 %s E4 %s E5 %s' % (nm, mo['E1'], mo['E2'], mo['E3'], mo['E4'], mo['E5']))
            rej = None  # the head sphere moves toward the roof in the first direction / size that reaches it (the ladder of check_door_clearance selftest_static: 0.12 m, then 0.30 / 0.45 m)
            for sz in (0.12, 0.30, 0.45):
                for h in ((0, 0, sz), (sz, 0, 0), (-sz, 0, 0), (0, sz, 0), (0, -sz, 0)):
                    if not evaluate(f, cls, seat, tj, mutate={'head_shift': h})['E5']:
                        rej = h
                        break
                if rej:
                    break
            print('    mutation head up / +-x / +-y 0.12 .. 0.45 m -> E5 rejected by %s' % (str(rej) if rej else 'NONE (vacuous for this row)'))
    if a.logs:
        r = logs_residuals(a.logs)
        if r:
            worst = max(v for _, v in r)
            over = [v for _, v in r if v > JUMP_MAX_M]
            print('E6 logs: %d approach ends with a mount (incl. already-there skips), residual max %.3f m median %.3f m, over %.2f m: %d  ->  %s' % (
                len(r), worst, float(np.median([v for _, v in r])), JUMP_MAX_M, len(over), 'ok' if not over else 'FAIL'))
            if over:
                bad += 1
    if a.json:
        json.dump(allr, open(a.json, 'w'), indent=1)
    print('ENTRY FEEL: %d clip rows over their gates' % bad)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
