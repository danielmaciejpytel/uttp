"""Convert an exported box layout from the Unreal axis convention to the game world convention.

Input: a list of parts (see the README), each with the placement of a cube mesh:
    loc    [x, y, z]            centimetres, Unreal world (left-handed: X forward, Y right, Z up)
    rot    [roll, pitch, yaw]   degrees, Unreal rotator
    scale  [sx, sy, sz]
    lmin / lmax [x, y, z]       centimetres, local mesh bounds (unscaled)
Output: oriented boxes in metres in the game world (right-handed: X right, Y forward, Z up).

Convention (documented once, used everywhere in this folder):
    position mapping   M(x, y, z) = (y, x, z)       Unreal forward (X) becomes game forward (Y),
                                                     Unreal right (Y) becomes game right (X)
    M is a reflection (det = -1), so rotations are conjugated:  R_game = M * R_unreal * M
    consequences       yaw changes sign; the two horizontal local axes swap, so the half extents are
                       (hy, hx, hz) and the local centre offset is (cy, cx, cz)
    units              1 m = 100 cm
Run `python -I ue_to_red.py --self-test` to check one hand-computed part.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

CUBE_MESHES = ('Cube', 'SM_Cube')


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def ue_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Matrix whose columns are the local X, Y, Z axes expressed in Unreal world coordinates."""
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    # Unreal's rotation matrix, rows are the transformed local axes.
    rows = [
        [cp * cy, cp * sy, sp],
        [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp],
        [-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp],
    ]
    return [[rows[j][i] for j in range(3)] for i in range(3)]  # transpose: axes as columns


_SWAP = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]


def to_game_rotation(r_ue):
    return _mat_mul(_mat_mul(_SWAP, r_ue), _SWAP)


def matrix_to_quaternion(m):
    """Right-handed rotation matrix (columns = axes) to (i, j, k, r)."""
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        r = 0.25 * s
        i = (m[2][1] - m[1][2]) / s
        j = (m[0][2] - m[2][0]) / s
        k = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        r = (m[2][1] - m[1][2]) / s
        i = 0.25 * s
        j = (m[0][1] + m[1][0]) / s
        k = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        r = (m[0][2] - m[2][0]) / s
        i = (m[0][1] + m[1][0]) / s
        j = 0.25 * s
        k = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        r = (m[1][0] - m[0][1]) / s
        i = (m[0][2] + m[2][0]) / s
        j = (m[1][2] + m[2][1]) / s
        k = 0.25 * s
    n = math.sqrt(i * i + j * j + k * k + r * r)
    if r < 0:
        n = -n
    return [i / n, j / n, k / n, r / n]


def quaternion_to_matrix(q):
    i, j, k, r = q
    return [
        [1 - 2 * (j * j + k * k), 2 * (i * j - k * r), 2 * (i * k + j * r)],
        [2 * (i * j + k * r), 1 - 2 * (i * i + k * k), 2 * (j * k - i * r)],
        [2 * (i * k - j * r), 2 * (j * k + i * r), 1 - 2 * (i * i + j * j)],
    ]


def convert_part(part):
    """One Unreal cube part to an oriented box in game metres. Returns a dict."""
    roll, pitch, yaw = part['rot']
    r_ue = ue_rotation_matrix(roll, pitch, yaw)
    sx, sy, sz = part['scale']
    lmin, lmax = part['lmin'], part['lmax']
    c_local = [(lmin[a] + lmax[a]) / 2 * s for a, s in enumerate((sx, sy, sz))]  # centre offset, local, cm
    half_ue = [(lmax[a] - lmin[a]) / 2 * abs(s) for a, s in enumerate((sx, sy, sz))]
    c_world = [part['loc'][a] + sum(r_ue[a][b] * c_local[b] for b in range(3)) for a in range(3)]
    center = [c_world[1] / 100.0, c_world[0] / 100.0, c_world[2] / 100.0]       # M, cm -> m
    half = [half_ue[1] / 100.0, half_ue[0] / 100.0, half_ue[2] / 100.0]          # local X/Y swap
    quat = matrix_to_quaternion(to_game_rotation(r_ue))
    return {
        'name': '%s.%s' % (part.get('actor', 'part'), part.get('comp', '')),
        'center': [round(v, 6) for v in center],
        'half': [round(v, 6) for v in half],
        'quat': [round(v, 9) for v in quat],
        'source_collision': part.get('coll'),
        'visible': bool(part.get('vis', True)),
    }


def convert_parts(parts):
    out = []
    for p in parts:
        mesh = p['mesh'].rsplit('/', 1)[-1].split('.')[0]
        if mesh not in CUBE_MESHES or p.get('coll') == 'NoCollision':
            continue
        out.append(convert_part(p))
    return out


def self_test():
    # Part: 2 m x 4 m x 1 m block (Unreal X forward = 4 m long, Y = 2 m), centred cube mesh,
    # at Unreal (1000, 500, 50) cm, yaw 90 deg. Hand computation:
    #   local X axis -> Unreal +Y, so the 4 m side lies along Unreal Y; game: along X.
    #   centre (1000, 500, 50) cm -> game (5.0, 10.0, 0.5) m.
    part = {'actor': 't', 'comp': 'c', 'mesh': '/Engine/BasicShapes/Cube.Cube', 'loc': [1000, 500, 50],
            'rot': [0, 0, 90], 'scale': [4, 2, 1], 'lmin': [-50, -50, -50], 'lmax': [50, 50, 50], 'coll': 'BlockAll'}
    b = convert_part(part)
    assert [round(v, 4) for v in b['center']] == [5.0, 10.0, 0.5], b
    # Unreal half extents (2, 1, 0.5) m with yaw 90: long axis is the box local X, which points along game +X
    # (Unreal +Y). Game local axes are swapped, so the stored half extents are (1, 2, .5) and the box local
    # Y axis (length 2) carries the long side after the rotation.
    assert [round(v, 4) for v in b['half']] == [1.0, 2.0, 0.5], b
    m = quaternion_to_matrix(b['quat'])
    long_axis = [m[a][1] for a in range(3)]   # game local Y in game world
    assert [round(v, 4) for v in long_axis] == [1.0, 0.0, 0.0], long_axis
    # Yaw sign: Unreal yaw +90 turns forward (X) to right (Y); in game coordinates forward Y turns to right X,
    # which is a rotation of -90 deg about Z in a right-handed frame: (0,1,0) -> (1,0,0).
    q = matrix_to_quaternion(to_game_rotation(ue_rotation_matrix(0, 0, 90)))
    rm = quaternion_to_matrix(q)
    fwd = [rm[a][1] for a in range(3)]
    assert [round(v, 4) for v in fwd] == [1.0, 0.0, 0.0], fwd
    # Pitch: Unreal pitch +30 raises the local X axis: (cos30, 0, sin30) -> game (0, cos30, sin30), carried by
    # the game local Y axis (Unreal local X after the axis swap).
    qp = matrix_to_quaternion(to_game_rotation(ue_rotation_matrix(0, 30, 0)))
    mp = quaternion_to_matrix(qp)
    axis = [mp[a][1] for a in range(3)]
    assert all(abs(u - v) < 1e-6 for u, v in zip(axis, [0.0, math.cos(math.radians(30)), math.sin(math.radians(30))])), axis
    # Roll and mixed angles: the converted matrix must be a proper rotation (determinant +1).
    rr = to_game_rotation(ue_rotation_matrix(40, 25, -70))
    det = (rr[0][0] * (rr[1][1] * rr[2][2] - rr[1][2] * rr[2][1]) - rr[0][1] * (rr[1][0] * rr[2][2] - rr[1][2] * rr[2][0])
           + rr[0][2] * (rr[1][0] * rr[2][1] - rr[1][1] * rr[2][0]))
    assert abs(det - 1.0) < 1e-9, det
    print('self-test ok: center %s half %s quat %s' % (b['center'], b['half'], b['quat']))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--input', help='exported parts JSON (list of parts)')
    ap.add_argument('--output', help='default: playground_boxes.json next to the input')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.input:
        ap.error('--input is required')
    if not args.output:
        args.output = os.path.join(os.path.dirname(os.path.abspath(args.input)), 'playground_boxes.json')
    with open(args.input, encoding='utf-8') as f:
        parts = json.load(f)
    boxes = convert_parts(parts)
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump({'units': 'metres', 'frame': 'game world, right-handed, X right, Y forward, Z up',
                   'source_parts': len(parts), 'boxes': boxes}, f, indent=1)
    print('wrote %d boxes of %d parts -> %s' % (len(boxes), len(parts), args.output))
    return 0


if __name__ == '__main__':
    sys.exit(main())
