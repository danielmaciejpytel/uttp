"""Entry-approach table: where V has to stand before a car mount is issued (the clip start spot), per vehicle class / seat / body.

  python -I gen_entry_approach.py                                  regenerate plugin\\UTPP\\scripts\\utpp_vehicle_approach_table.reds from entry_approach.json
  python -I gen_entry_approach.py --check                          exit 1 when the .reds is not what entry_approach.json generates, or the table breaks its invariants
  python -I gen_entry_approach.py --measure --sources <sources.txt> [--boxes] [--out <json>] [--print]
                                                                   (re)measure the spots from the staged entry clips (sources.txt: '<clip name>  <run dir>' per line, the
                                                                   file the pack step writes next to the clips) and, with --boxes, the footprint boxes from the extracted
                                                                   body meshes; writes entry_approach.json (default) or --out. Needs numpy and the extracted door data.
  python -I gen_entry_approach.py --verify-sources <sources.txt>   exit 1 when a clip's sha256 differs from the one stored in entry_approach.json (stale table)

Why: the engine slides the entity to the entry clip's start spot in a fixed time (about 0.30 s, Turbo-R 0.82 s) whatever the distance, which at 1-3 m
skates V through the car. utpp_vehicle_approach.reds walks V to the spot first and mounts when she is there.

Frame (all numbers): vehicle frame = vehicle entity origin, x right, y forward, z up, metres. yaw_deg: UTPP convention (counter-clockwise, 0 = vehicle forward
= +y, facing direction = (-sin yaw, cos yaw)); the clip start heading, i.e. the way V faces when the clip starts.

Spot measurement (same map as measure_entry_paths.py / check_door_clearance.Model): the engine places the entity at seat * inverse(root delta), so the clip root
of the LAST frame sits in the seat slot of the representative car (door-data 'seat' t / R):
    P_veh = GLTF_TO_VEH @ (seat_R @ inv(R_root_last) @ (root_t_first - root_t_last) + seat_t)
    facing = GLTF_TO_VEH @ seat_R @ inv(R_root_last) @ R_root_first @ inv(seat_R) @ inv(GLTF_TO_VEH) @ (0, 1, 0)      (the seated root faces vehicle forward)
The spots are of ONE representative vehicle per class (door-data 'vehicle'); other models of the class have a seat slot that differs by some centimetres,
the engine slide covers that residual (it stays far below the 0.5-3 m the walk removes). 'seat_frame' keeps the start in the seat frame as well.

Footprint box: x / y extent of the ext01 body mesh of the representative vehicle (glb-body, vehicle axes = (gx, -gz)), used by the planner as the obstacle the
walk keeps 0.35 m away from. Door arcs (closed door sweep) are copied from door_timing.json 'arcs' (valid swing doors only; the Turbo-R lifting door has none;
NO_APPROACH_SECTOR lists the sides whose arc is left out on purpose: all swing-door classes, the door opens only after the mount and the sector only forced a detour round the closed door).

Classes without an entry, seats without a row (rear seats) and bodies without a clip have no approach: the vanilla mount path runs.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import re
import struct
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
JSON_PATH = os.path.join(HERE, 'entry_approach.json')
TIMING = os.path.join(HERE, 'door_timing.json')
REDS = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'plugin', 'UTPP', 'scripts', 'utpp_vehicle_approach_table.reds'))
EXTRACTED = os.path.join(project_root(), r'Extracted\game\vehicle-entry-car')
VAR_CLASS = {'utility': 'utility4'}  # make_car_workspot.py VAR_CLASS: the seat variable of the utility class
SEAT_SIDE = {'driver': 'front_left', 'passenger_r': 'front_right'}
DOOR_SIDE = {'driver': 'left', 'passenger_r': 'right'}
BODY_NAME = {'w': 'female', 'm': 'male'}
BODY_CLIP = {'female': 'pwa', 'male': 'pma'}
BOX_MARGIN_M = 0.35       # planner clearance (ApproachCore Scene)
MIN_SPOT_CLEARANCE_M = 0.25  # the spot lies at least this far outside the footprint box (invariant; the box is the loose mesh bounding box, mirrors included)
# Sides whose approach scene carries no closed-door sweep sector (the planner then keeps only the footprint box clear). The approach walk ends with the mount
# (the mount is deferred until V stands on the spot) and the door opens only after it (VehicleComponent.OnMountingEvent queues the open; logged: approach end ->
# mount -> door open +0.04..0.11 s, never before), so during the walk the door is closed and lies inside the footprint box: the sector protects nothing. It only
# lengthened the path: it sits between a V standing at the door and a clip start spot behind the door's rear edge, so the walk went round the open-door tip
# (sport2 driver from the door: 2.7 m, a 270 deg turn out, along the car and back, instead of 1.65 m; standard3 driver from beside the hinge 3.1 m instead of 2.3 m).
# Applies to every class with a swing door (vehall2: sport2, vehall3: standard2, standard25, standard3, utility; sport1 has no arc anyway). Where the start spot lies
# inside the sector's radius (utility) the open door still sweeps over V standing on the spot; that is the door hold cap's job (utpp_vehicle_door.reds), the sector never
# covered the last 0.7 m before the spot (planner exemptM).
# The door arcs stay in door_timing.json (door cap / close timing); only the approach copy omits them.
NO_APPROACH_SECTOR = {(cls, side) for cls in ('sport2', 'standard2', 'standard25', 'standard3', 'utility') for side in ('left', 'right')}


def seat_var(cls, seat):
    return '%s_vehicle_seat_%s' % (VAR_CLASS.get(cls, cls), SEAT_SIDE[seat])


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest().upper()


def yaw_of(dx, dy):
    """UTPP yaw (deg) of a planar direction: 0 = +y, counter-clockwise."""
    return math.degrees(math.atan2(-dx, dy))


def wrap_deg(a):
    a = math.fmod(a, 360.0)
    if a > 180.0:
        a -= 360.0
    elif a <= -180.0:
        a += 360.0
    return a


# --------------------------------------------------------------------------------------------------------------- measurement
def measure_clip(path, cls, seat):
    """(x, y, yaw_deg, seat-frame start (gltf xyz), root yaw check) of one entry clip in the vehicle frame of the representative car."""
    import numpy as np
    sys.path.insert(0, HERE)
    import check_door_clearance as CD
    sk, _t, Wt, Wr = CD.load_clip(path, 'enter')
    dj = CD.load_door_data(CD.DATA, cls)
    side = DOOR_SIDE[seat]
    st = np.array(dj['seat'][side]['t'], float)
    sR = np.array(dj['seat'][side]['R'], float).reshape(3, 3)
    M = CD.GLTF_TO_VEH
    ri = sk.index['Root']
    R0, Re = Wr[0, ri], Wr[-1, ri]
    A = sR @ Re.T
    p_seat = A @ (Wt[0, ri] - Wt[-1, ri])            # start offset in the seat frame's axes (rig axes)
    veh = M @ (p_seat + st)
    face = M @ sR @ Re.T @ R0 @ sR.T @ M.T @ np.array([0.0, 1.0, 0.0])
    end_face = M @ sR @ sR.T @ M.T @ np.array([0.0, 1.0, 0.0])  # identity by construction: the seated root faces forward
    assert abs(end_face[1] - 1.0) < 1e-9
    return {
        'x': float(veh[0]), 'y': float(veh[1]), 'yaw_deg': float(yaw_of(face[0], face[1])),
        'seat_frame': {'x': float(p_seat[0]), 'y': float(p_seat[1]), 'z': float(p_seat[2]),
                       'note': 'start offset from the seated root, seat rotation applied, rig axes (x, up, z); the vehicle frame spot adds the seat slot t'},
        'seat_t_vehicle': [float(v) for v in (M @ st)],
    }


def parse_sources(path):
    out = {}
    for line in open(path, encoding='ascii'):
        line = line.rstrip('\n')
        if not line.strip():
            continue
        name, run = line.split('  ', 1)
        out[name.strip()] = os.path.join(run.strip(), 'roundtrip', name.strip() + '.anims.glb')
    return out


def glb_position_bounds(path):
    """(min, max) of every POSITION accessor of a glb (no node transforms in these exports)."""
    b = open(path, 'rb').read()
    n = struct.unpack_from('<I', b, 12)[0]
    j = json.loads(b[20:20 + n])
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for m in j['meshes']:
        for p in m['primitives']:
            a = j['accessors'][p['attributes']['POSITION']]
            lo = [min(x, y) for x, y in zip(lo, a['min'])]
            hi = [max(x, y) for x, y in zip(hi, a['max'])]
    return lo, hi


def measure_box(cls):
    pat = os.path.join(EXTRACTED, 'glb-body', 'v_%s_*__ext01_body_01.glb' % VAR_CLASS.get(cls, cls))
    files = sorted(glob.glob(pat))
    if not files:
        raise SystemExit('no body mesh for %s (%s)' % (cls, pat))
    lo, hi = glb_position_bounds(files[0])
    x0, x1 = lo[0], hi[0]
    y0, y1 = -hi[2], -lo[2]  # vehicle y = -gz
    return {'cx': round((x0 + x1) / 2, 3), 'cy': round((y0 + y1) / 2, 3), 'hx': round((x1 - x0) / 2, 3), 'hy': round((y1 - y0) / 2, 3),
            'source': os.path.basename(files[0]), 'source_sha256': sha256(files[0])}


def cmd_measure(a):
    tj = json.load(open(TIMING, encoding='utf-8'))
    old = json.load(open(JSON_PATH, encoding='utf-8')) if os.path.exists(JSON_PATH) else {'classes': {}}
    sources = parse_sources(a.sources)
    sys.path.insert(0, HERE)
    import check_door_clearance as CD
    data = {'_note': 'Measured by gen_entry_approach.py --measure; vehicle frame x right, y forward, m; yaw UTPP convention (0 = forward, CCW). Do not edit by hand.',
            'sources_file': a.sources.replace('\\', '/'), 'margin_m': BOX_MARGIN_M, 'classes': {}}
    for name, path in sorted(sources.items()):
        g, cls, seat = CD.parse_name(path)
        if not os.path.exists(path):
            raise SystemExit('missing clip: ' + path)
        c = data['classes'].setdefault(cls, {'seats': {}})
        m = measure_clip(path, cls, seat)
        m.update({'clip': name, 'clip_sha256': sha256(path)})
        c['seats'].setdefault(seat, {'seat_var': seat_var(cls, seat), 'spots': {}})['spots'][BODY_NAME[g]] = {
            k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.items() if k not in ('seat_frame', 'seat_t_vehicle')}
        c['seats'][seat]['spots'][BODY_NAME[g]]['seat_frame'] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in m['seat_frame'].items()}
        c['seats'][seat]['seat_t_vehicle'] = [round(v, 3) for v in m['seat_t_vehicle']]
    for cls, c in data['classes'].items():
        dj = json.load(open(os.path.join(CD.DATA, cls + '.json'), encoding='utf-8'))
        c['vehicle'] = dj.get('vehicle', '')
        if a.boxes or 'box' not in old.get('classes', {}).get(cls, {}):
            c['box'] = measure_box(cls)
        else:
            c['box'] = old['classes'][cls]['box']
        arcs = tj['arcs'].get(cls, {})
        c['arcs'] = {}
        for side in ('left', 'right'):
            ar = arcs.get(side)
            if ar and ar.get('valid') and (cls, side) not in NO_APPROACH_SECTOR:
                c['arcs'][side] = {k: round(float(ar[k]), 3) for k in ('hingeX', 'hingeY', 'length', 'closedDirDeg', 'openDeg')}
    text = json.dumps(data, indent=1, sort_keys=True) + '\n'
    if a.print:
        print(text)
    out = a.out or JSON_PATH
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print('wrote', out)
    return 0


# --------------------------------------------------------------------------------------------------------------- emit / check
def f3(v):
    return '%.3f' % v


def iter_rows(data):
    for cls in sorted(data['classes']):
        c = data['classes'][cls]
        for seat in ('driver', 'passenger_r'):
            s = c['seats'].get(seat)
            if s:
                yield cls, c, seat, s


def emit(data):
    L = []
    L.append('// GENERATED by Source\\tools\\anim\\vehicle-entry\\gen_entry_approach.py from entry_approach.json - do not edit')
    L.append('// Entry approach table: the clip start spot V is walked to before a car mount (utpp_vehicle_approach.reds), and the scene the walk keeps clear of.')
    L.append('// Vehicle frame: x right, y forward, metres (origin = vehicle entity origin). yawDeg: UTPP convention (counter-clockwise, 0 = vehicle forward).')
    L.append('// Spots of one representative vehicle per class (entry_approach.json); seat variables as in utpp_vehicle_door_timing.reds. Seats / bodies without a row: no approach.')
    L.append('//   seat variable                              body     x       y      yaw    clip')
    for cls, c, seat, s in iter_rows(data):
        for body in ('female', 'male'):
            sp = s['spots'].get(body)
            if sp:
                L.append('//   %-42s %-7s %+7.3f %+7.3f %+7.1f  %s' % (s['seat_var'], body, sp['x'], sp['y'], sp['yaw_deg'], sp['clip']))
    L.append('')
    L.append('public struct UTPPApproachSpot { public let valid: Bool; public let x: Float; public let y: Float; public let yawDeg: Float; }')
    L.append('')
    L.append('public func UTPPApproachSpotOf(seatVar: CName, female: Bool) -> UTPPApproachSpot {')
    L.append('  let a: UTPPApproachSpot;')
    for cls, c, seat, s in iter_rows(data):
        for body in ('female', 'male'):
            sp = s['spots'].get(body)
            if not sp:
                continue
            L.append('  if Equals(seatVar, n"%s") && %sfemale {' % (s['seat_var'], '' if body == 'female' else '!'))
            L.append('    a.valid = true; a.x = %s; a.y = %s; a.yawDeg = %s;' % (f3(sp['x']), f3(sp['y']), '%.2f' % sp['yaw_deg']))
            L.append('    return a;')
            L.append('  }')
    L.append('  a.valid = false;')
    L.append('  return a;')
    L.append('}')
    L.append('')
    L.append('// Footprint box (obstacle) and the closed front-door sweep of the seat\'s class; arcValid = false for the Turbo-R lifting door and where NO_APPROACH_SECTOR says so (every class since vehall3).')
    L.append('public struct UTPPApproachScene { public let valid: Bool; public let boxCx: Float; public let boxCy: Float; public let boxHx: Float; public let boxHy: Float; '
             'public let arcValid: Bool; public let hingeX: Float; public let hingeY: Float; public let arcLength: Float; public let arcClosedDeg: Float; public let arcOpenDeg: Float; }')
    L.append('')
    L.append('public func UTPPApproachSceneOf(seatVar: CName) -> UTPPApproachScene {')
    L.append('  let a: UTPPApproachScene;')
    for cls, c, seat, s in iter_rows(data):
        b = c['box']
        L.append('  if Equals(seatVar, n"%s") {' % s['seat_var'])
        L.append('    a.valid = true; a.boxCx = %s; a.boxCy = %s; a.boxHx = %s; a.boxHy = %s;' % (f3(b['cx']), f3(b['cy']), f3(b['hx']), f3(b['hy'])))
        ar = c['arcs'].get(DOOR_SIDE[seat])
        if ar:
            L.append('    a.arcValid = true; a.hingeX = %s; a.hingeY = %s; a.arcLength = %s; a.arcClosedDeg = %s; a.arcOpenDeg = %s;' % (
                f3(ar['hingeX']), f3(ar['hingeY']), f3(ar['length']), '%.2f' % ar['closedDirDeg'], '%.2f' % ar['openDeg']))
        else:
            L.append('    a.arcValid = false;')
        L.append('    return a;')
        L.append('  }')
    L.append('  a.valid = false;')
    L.append('  return a;')
    L.append('}')
    return '\n'.join(L) + '\n'


def invariants(data):
    """Problems of the stored table (empty = fine). No extraction needed."""
    bad = []
    if not data.get('classes'):
        bad.append('no classes')
    for cls, c, seat, s in iter_rows(data):
        b = c.get('box')
        if not b or not all(isinstance(b.get(k), (int, float)) and math.isfinite(b[k]) for k in ('cx', 'cy', 'hx', 'hy')) or b['hx'] <= 0.5 or b['hy'] <= 1.0:
            bad.append('%s: bad box %s' % (cls, b))
            continue
        side_sign = -1.0 if seat == 'driver' else 1.0
        if s['seat_var'] != seat_var(cls, seat):
            bad.append('%s %s: seat variable %s' % (cls, seat, s['seat_var']))
        for body, sp in s['spots'].items():
            tag = '%s %s %s' % (cls, seat, body)
            if body not in BODY_CLIP or sp.get('clip', '').split('_')[2:3] != [BODY_CLIP[body]]:
                bad.append('%s: clip %s does not match the body' % (tag, sp.get('clip')))
            for k in ('x', 'y', 'yaw_deg'):
                if not isinstance(sp.get(k), (int, float)) or not math.isfinite(sp[k]):
                    bad.append('%s: %s not finite' % (tag, k))
            if bad and bad[-1].startswith(tag):
                continue
            if sp['x'] * side_sign <= 0:
                bad.append('%s: spot on the wrong side of the car (x %.2f)' % (tag, sp['x']))
            # distance of the spot to the box
            dx = max(abs(sp['x'] - b['cx']) - b['hx'], 0.0)
            dy = max(abs(sp['y'] - b['cy']) - b['hy'], 0.0)
            if math.hypot(dx, dy) < MIN_SPOT_CLEARANCE_M:
                bad.append('%s: spot only %.2f m from the footprint box (< %.2f)' % (tag, math.hypot(dx, dy), MIN_SPOT_CLEARANCE_M))
            if not re.fullmatch(r'[0-9A-F]{64}', sp.get('clip_sha256', '')):
                bad.append('%s: missing clip sha256' % tag)
        if set(s['spots']) != {'female', 'male'}:
            bad.append('%s %s: bodies %s (both expected)' % (cls, seat, sorted(s['spots'])))
    return bad


def cmd_emit(check):
    data = json.load(open(JSON_PATH, encoding='utf-8'))
    bad = invariants(data)
    for m in bad:
        print('INVARIANT:', m)
    text = emit(data)
    if check:
        cur = open(REDS, encoding='utf-8', newline='').read() if os.path.exists(REDS) else None
        if cur != text:
            print('STALE: %s differs from what entry_approach.json generates' % REDS)
            return 1
        if bad:
            return 1
        print('gen_entry_approach --check OK (%d rows)' % sum(len(s['spots']) for _c, _d, _s, s in iter_rows(data)))
        return 0
    if bad:
        return 1
    with open(REDS, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print('wrote', REDS)
    return 0


def cmd_verify(a):
    data = json.load(open(JSON_PATH, encoding='utf-8'))
    src = parse_sources(a.verify_sources)
    stale = 0
    for cls, _c, seat, s in iter_rows(data):
        for body, sp in s['spots'].items():
            p = src.get(sp['clip'])
            if not p or not os.path.exists(p):
                print('MISSING', sp['clip'])
                stale += 1
            elif sha256(p) != sp['clip_sha256']:
                print('STALE', sp['clip'], p)
                stale += 1
    print('verify-sources: %d stale/missing' % stale)
    return 1 if stale else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--measure', action='store_true')
    ap.add_argument('--sources')
    ap.add_argument('--boxes', action='store_true')
    ap.add_argument('--out')
    ap.add_argument('--print', action='store_true')
    ap.add_argument('--verify-sources', dest='verify_sources')
    a = ap.parse_args()
    if a.measure:
        if not a.sources:
            ap.error('--measure needs --sources')
        return cmd_measure(a)
    if a.verify_sources:
        return cmd_verify(a)
    return cmd_emit(a.check)


if __name__ == '__main__':
    sys.exit(main())
