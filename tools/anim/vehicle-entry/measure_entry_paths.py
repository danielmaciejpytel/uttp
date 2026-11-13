"""Vehicle-frame joint paths of car entry clips: where does V cross the car-side line, and does the pelvis slide backwards while she sits down.

  python -I measure_entry_paths.py --sources <sources.txt | run dir listing> [--vanilla] [--out table.txt] [--label BEFORE]
  python -I measure_entry_paths.py --glb <clip.anims.glb> ...

Frame: vehicle space (x right, y forward, z up) = GLTF_TO_VEH @ (seat_bone_R @ inverse(last-frame Root R) @ (clip world - last-frame Root t) + seat_bone_t): the
engine places the entity at seat * inverse(root delta), so the clip root of the LAST frame sits in the seat slot (check_door_clearance.Model uses the same map).
Car-side line |x| = the OUTER x of the closed front door skin (door-data); door rear edge y = hinge + length * dir from door_timing.json 'arcs' (the closed rear edge).
Per tracked joint: first frame at which the joint path goes inboard of the side line, its y there, and y - rear edge y (negative = BEHIND the rear edge of the door,
i.e. through the B-pillar / quarter panel); the rearmost y of the joint while it is inboard of the line (before the seat is reached: the first frame the pelvis is within
15 cm of its final x). Seated phase: from the first frame the pelvis is within 15 cm of its final x (lateral), the pelvis y drawdown (max running-max minus value) and the
largest backward speed.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import skel as S  # noqa: E402
import check_door_clearance as CD  # noqa: E402

DOOR_PRESET = (-1.85, 0.5)  # harness door-preset walk end (car space, driver side; mirrored in x for the passenger): the native entry-glide reference spot
SLIDE_OK_M = 0.57          # clip start within this of the preset keeps the engine slide <= ~3 m/s (T 0.30 s)
TRACK = [('Root', 'Root'), ('Hips', 'Hips'), ('Neck', 'Neck1'), ('Head', 'Head'), ('shoL', 'LeftArm'), ('shoR', 'RightArm'), ('handL', 'LeftHand'), ('handR', 'RightHand')]
FPS = 30.0
SEAT_H_M = 0.12


def class_geom(cls, side, tj):
    """(outer |x| of the closed door skin, door rear-edge y from the arc)."""
    dj = CD.load_door_data(CD.DATA, cls)
    p = dj['door'][side]['parts'][0]
    P = np.asarray(p['points'])
    R = S.q2m(np.asarray(p['frames_q'][0]))
    W = (R @ P.T).T + np.asarray(p['frames_t'][0])
    V = W @ CD.GLTF_TO_VEH.T
    x_out = float(np.abs(V[:, 0]).max())
    a = tj['arcs'][cls][side]
    th = math.radians(a['closedDirDeg'])
    return x_out, a['hingeY'] + a['length'] * math.sin(th), a['hingeX'] + a['length'] * math.cos(th)


def joint_paths(path, cls, seat, clip_name='enter'):
    sk, t, Wt, Wr = CD.load_clip(path, clip_name)
    dj = CD.load_door_data(CD.DATA, cls)
    side = 'left' if seat == 'driver' else 'right'
    st, sR = np.array(dj['seat'][side]['t']), np.array(dj['seat'][side]['R']).reshape(3, 3)
    ri = sk.index['Root']
    Re, pe = Wr[-1, ri], Wt[-1, ri]
    A = sR @ Re.T
    out = {}
    for nm, b in TRACK:
        if b not in sk.index:
            continue
        P = np.einsum('ij,tj->ti', A, Wt[:, sk.index[b]] - pe) + st
        out[nm] = P @ CD.GLTF_TO_VEH.T
    return t, out


def analyse(path, cls, seat, tj):
    side = 'left' if seat == 'driver' else 'right'
    x_out, y_edge, x_edge = class_geom(cls, side, tj)
    sg = -1.0 if side == 'left' else 1.0  # side of the car (outer x sign)
    t, J = joint_paths(path, cls, seat)
    res = {'x_out': x_out, 'y_edge': y_edge, 'joints': {}}
    H = J['Hips']
    xf = H[-1, 0]
    near = np.flatnonzero(np.abs(H[:, 0] - xf) <= 0.15)
    i_seat = int(near[0]) if len(near) else len(t) - 1
    R0 = J['Root'][0]
    spot = np.array([DOOR_PRESET[0] * (1.0 if side == 'left' else -1.0), DOOR_PRESET[1]])
    res['root_start'] = [float(R0[0]), float(R0[1])]
    res['start_to_preset_m'] = float(np.linalg.norm(R0[:2] - spot))
    for nm, P in J.items():
        if nm == 'Root':
            continue
        d = sg * P[:, 0] - x_out  # > 0: outboard of the side line
        k = np.flatnonzero(d <= 0)
        e = {'cross_frame': None, 'cross_y': None, 'margin': None, 'rear_inboard_y': None, 'rear_inboard_margin': None}
        if len(k) and k[0] > 0:
            i = int(k[0])
            u = d[i - 1] / (d[i - 1] - d[i]) if d[i - 1] != d[i] else 0.0
            y = float(P[i - 1, 1] * (1 - u) + P[i, 1] * u)
            e.update(cross_frame=float(i - 1 + u), cross_y=y, margin=y - y_edge)
        elif len(k) and k[0] == 0:
            e.update(cross_frame=0.0, cross_y=float(P[0, 1]), margin=float(P[0, 1] - y_edge))  # starts inside
        # rearmost y while inboard (until the seat is reached)
        kk = list(k)
        if kk:
            yy = P[kk, 1]
            e['rear_inboard_y'] = float(yy.min())
            e['rear_inboard_margin'] = float(yy.min() - y_edge)
        res['joints'][nm] = e
    y = H[:, 1]
    low = H[:, 2] <= H[-1, 2] + SEAT_H_M  # pelvis near seated height (door7-turbo/search.py: pelvis z < 0.30 on the Turbo-R, here final z + 12 cm)
    bd, vb = 0.0, 0.0
    for i in range(len(y) - 12):
        if low[i] and low[i + 12]:
            bd = max(bd, float(y[i] - y[i + 12]))
    for i in range(len(y) - 1):
        if low[i] and low[i + 1]:
            vb = max(vb, float(-(y[i + 1] - y[i]) * FPS))
    res['seat_frame'] = i_seat
    res['pelvis_back_drawdown_m'] = bd
    res['pelvis_back_speed_mps'] = vb
    res['frames'] = len(t)
    res['pelvis_start'] = [float(H[0, 0]), float(H[0, 1])]
    res['pelvis_final'] = [float(H[-1, 0]), float(H[-1, 1])]
    return res


def fmt_row(name, r, van=None):
    """Header line + one line per joint: crossing frame / y / margin vs the door rear edge, rearmost inboard y margin."""
    rb = [(r['joints'][n]['rear_inboard_margin'], n) for n in r['joints'] if r['joints'][n]['rear_inboard_margin'] is not None]
    worst = min(rb) if rb else (float('nan'), '-')
    slide = '' if van is None else ' start_vs_vanilla_dy %+.3f dx %+.3f' % (r['pelvis_start'][1] - van['pelvis_start'][1], r['pelvis_start'][0] - van['pelvis_start'][0])
    L = ['%-26s x_out %.3f edge_y %+.3f | worst rearmost-inboard margin %+.3f (%s) | pelvis seated back %.1f cm (0.4 s window) max back speed %.2f m/s | pelvis start (%+.2f, %+.2f) final (%+.2f, %+.2f) | CLIP START (root) %+.2f,%+.2f = %.2f m from the door preset (target <= 0.57)%s' % (
        name, r['x_out'], r['y_edge'], worst[0], worst[1], r['pelvis_back_drawdown_m'] * 100, r['pelvis_back_speed_mps'], r['pelvis_start'][0], r['pelvis_start'][1],
        r['pelvis_final'][0], r['pelvis_final'][1], r['root_start'][0], r['root_start'][1], r['start_to_preset_m'], slide)]
    for nm, e in r['joints'].items():
        if nm == 'Root':
            continue
        if e['cross_y'] is None:
            L.append('    %-6s never inboard of the side line' % nm)
        else:
            L.append('    %-6s crosses |x|=x_out at frame %5.1f y %+.3f  margin %+.3f   rearmost inboard y %+.3f margin %+.3f' % (
                nm, e['cross_frame'], e['cross_y'], e['margin'], e['rear_inboard_y'], e['rear_inboard_margin']))
    return '\n'.join(L)


HEADER = ('margin = y at the side-line crossing (or the rearmost y while inboard of the line) minus the door rear-edge y of door_timing.json arcs; NEGATIVE = BEHIND the door rear edge\n'
          '(through the B-pillar / quarter panel). Vehicle frame: x right, y forward, z up. Seated pelvis back = largest y drop over any 0.4 s window while the pelvis is within 12 cm of its final height.')


def parse_sources(p):
    files = []
    for line in open(p, encoding='ascii'):
        line = line.rstrip('\n')
        if not line.strip():
            continue
        n, run = line.split('  ', 1)
        files.append(os.path.join(run.strip(), 'roundtrip', n + '.anims.glb'))
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sources')
    ap.add_argument('--glb', nargs='*', default=[])
    ap.add_argument('--vanilla', action='store_true', help='also the vanilla player FPP clip of every set (Extracted\\game\\vehicle-entry-car\\glb\\p?a_<class>_<seat>)')
    ap.add_argument('--out')
    ap.add_argument('--label', default='')
    ap.add_argument('--json')
    a = ap.parse_args()
    tj = CD.load_timing()
    files = list(a.glb)
    if a.sources:
        files += parse_sources(a.sources)
    if a.vanilla:
        gd = os.path.join(os.path.dirname(CD.DATA), 'glb')
        for f in list(files):
            g, cls, seat = CD.parse_name(f)
            files.append(os.path.join(gd, 'p%sa_%s_%s.anims.glb' % (g, cls, seat)))
    lines = ['## ' + a.label if a.label else '##', HEADER]
    allr = {}
    gd = os.path.join(os.path.dirname(CD.DATA), 'glb')
    for f in files:
        g, cls, seat = CD.parse_name(f)
        van = not os.path.basename(f).startswith('utpp_car_')
        r = analyse(f, cls, seat, tj)
        nm = ('VANILLA ' if van else '') + 'p%sa %s %s' % (g, cls, seat)
        vr = None
        if not van:
            vp = os.path.join(gd, 'p%sa_%s_%s.anims.glb' % (g, cls, seat))
            if os.path.exists(vp):
                vr = analyse(vp, cls, seat, tj)
        lines.append(fmt_row(nm, r, vr))
        allr[nm] = r
    txt = '\n'.join(lines)
    print(txt)
    if a.out:
        with open(a.out, 'a', encoding='utf-8') as fh:
            fh.write(txt + '\n\n')
    if a.json:
        json.dump(allr, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
