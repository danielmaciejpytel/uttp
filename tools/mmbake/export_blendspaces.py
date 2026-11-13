"""Export source pack blend spaces + UE reference weights for utpp_mm_montage_test (report 4.2 #33).

Reads Extracted/sourcepack/sourcepack/anims.json (BlendSpace / BlendSpace1D entries) and writes Data/anim/blendspaces.txt:

  utpp-blendspaces 1
  BS <name> <dims> <min0> <max0> <min1> <max1> <speedPerSec> <easeInOut 0|1> <nsamples>
  S <x> <y> <anim>
  REF <x> <y> <n> <sampleIdx> <weight> ...
  END

Reference weights follow UE 5.8 (Engine/Source/Runtime/Engine/Private/Animation):
  - BlendSpace.cpp 2138-2145 GetNormalizedBlendInput: clamp to [Min, Max] (no wrap here), normalize.
  - BlendSpace.cpp 2904-2980 GetSamples1D: piecewise-linear segment search, end clamp to weight 1.
  - BlendSpace.cpp 2982-3100 GetSamples2D: containing triangle -> FMath::GetBaryCentric2D.
  - BlendSpaceHelpers.cpp 57-105 Triangulate (Delaunay of raw sample values) + 141-195 AdjustEdgeDirections:
    for axis-aligned right triangles the hypotenuse is flipped so that its slope satisfies
    Tangential: up-slope wanted iff EdgeMid.X * EdgeMid.Y < 0 (raw, un-normalized sample space).
This reference is computed independently of the C++ code: brute-force Delaunay (every sample triple whose circumcircle
contains no other sample strictly inside), then among the candidate triangles containing the point the one whose
hypotenuse slope matches the Tangential rule.
source-pack-derived output.
"""
import argparse
import itertools
import json
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def find_root():
    for cand in (ROOT, os.path.abspath(os.path.join(ROOT, '..', '..'))):
        if os.path.isdir(os.path.join(cand, 'Extracted', 'sourcepack', 'sourcepack')):
            return cand
    return os.path.abspath(os.path.join(ROOT, '..'))


def circum(a, b, c):
    ax, ay = a; bx, by = b; cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    return ux, uy, (ax - ux) ** 2 + (ay - uy) ** 2


def bary(p, a, b, c):
    # FMath::GetBaryCentric2D
    v0 = (b[0] - a[0], b[1] - a[1]); v1 = (c[0] - a[0], c[1] - a[1]); v2 = (p[0] - a[0], p[1] - a[1])
    d00 = v0[0] * v0[0] + v0[1] * v0[1]; d01 = v0[0] * v1[0] + v0[1] * v1[1]; d11 = v1[0] * v1[0] + v1[1] * v1[1]
    d20 = v2[0] * v0[0] + v2[1] * v0[1]; d21 = v2[0] * v1[0] + v2[1] * v1[1]
    den = d00 * d11 - d01 * d01
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    return 1.0 - v - w, v, w


def hyp_ok(pts, tri):
    # axis-aligned right triangle: the edge that is neither horizontal nor vertical is the hypotenuse
    for i in range(3):
        a = pts[tri[i]]; b = pts[tri[(i + 1) % 3]]
        dx = b[0] - a[0]; dy = b[1] - a[1]
        if abs(dx) > 1e-9 and abs(dy) > 1e-9:
            up = dx * dy > 0
            mx = (a[0] + b[0]) * 0.5; my = (a[1] + b[1]) * 0.5
            return up == (mx * my < 0)
    return True


def ref2d(pts, p):
    cands = []
    for tri in itertools.combinations(range(len(pts)), 3):
        cc = circum(*(pts[i] for i in tri))
        if cc is None:
            continue
        ux, uy, r2 = cc
        if any((pts[k][0] - ux) ** 2 + (pts[k][1] - uy) ** 2 < r2 * (1 - 1e-9) for k in range(len(pts)) if k not in tri):
            continue
        w = bary(p, *(pts[i] for i in tri))
        if min(w) >= -1e-9:
            cands.append((tri, w))
    good = [c for c in cands if hyp_ok(pts, c[0])]
    tri, w = (good or cands)[0]
    return {tri[i]: w[i] for i in range(3) if w[i] > 1e-5}


def ref1d(xs, p):
    order = sorted(range(len(xs)), key=lambda i: xs[i])
    if p <= xs[order[0]]:
        return {order[0]: 1.0}
    if p >= xs[order[-1]]:
        return {order[-1]: 1.0}
    for a, b in zip(order, order[1:]):
        if xs[a] <= p <= xs[b]:
            f = (p - xs[a]) / (xs[b] - xs[a])
            return {k: v for k, v in ((a, 1 - f), (b, f)) if v > 1e-5}
    raise RuntimeError('no segment')


def main():
    root = find_root()
    ap = argparse.ArgumentParser()
    ap.add_argument('--anims', default=os.path.join(root, 'Extracted', 'sourcepack', 'sourcepack', 'anims.json'))
    ap.add_argument('--out', default=os.path.join(root, 'Data', 'anim', 'blendspaces.txt'))
    a = ap.parse_args()
    d = json.load(open(a.anims, encoding='utf-8'))
    lines = ['utpp-blendspaces 1']
    # 20 deterministic query points per blend space (incl. outside-range clamp cases)
    q2 = [(0, 0), (10, 20), (-30, 60), (60, -30), (89, 89), (-89, -45), (22.5, 45), (-22.5, 45), (22.5, -45),
          (-67.5, -45), (67.5, 45), (45, 0), (-45, 30), (5, -85), (-80, 5), (33, 77), (-12, -64), (120, 0),
          (-100, -120), (71, -13)]
    q1 = [-1.5, -1, -0.75, -0.5, -0.33, -0.1, 0, 0.05, 0.2, 0.25, 0.4, 0.5, 0.61, 0.75, 0.9, 0.99, 1, 1.2, -0.01, 0.333]
    for e in d['anims']:
        if e.get('class') not in ('BlendSpace', 'BlendSpace1D'):
            continue
        bs = e['blendspace']; props = e.get('props') or {}
        axes = bs['axes']; dims = 1 if e['class'] == 'BlendSpace1D' else 2
        mins = [axes[i]['min'] for i in range(dims)] + [0.0] * (2 - dims)
        maxs = [axes[i]['max'] for i in range(dims)] + [1.0] * (2 - dims)
        speed = float(bs['other'].get('TargetWeightInterpolationSpeedPerSec', 0))
        ease = 1 if props.get('target_weight_interpolation_ease_in_out', True) else 0
        if props.get('interpolate_using_grid'):
            raise SystemExit('interpolate_using_grid not supported by this reference: ' + e['name'])
        smp = bs['samples']
        lines.append('BS %s %d %.6f %.6f %.6f %.6f %.6f %d %d' % (e['name'], dims, mins[0], maxs[0], mins[1], maxs[1],
                                                                 speed, ease, len(smp)))
        for s in smp:
            lines.append('S %.6f %.6f %s' % (s['x'], s['y'], s['anim'].rsplit('.', 1)[-1]))
        if dims == 2:
            pts = [(s['x'], s['y']) for s in smp]
            for qx, qy in q2:
                cx = min(max(qx, mins[0]), maxs[0]); cy = min(max(qy, mins[1]), maxs[1])
                w = ref2d(pts, (cx, cy))
                lines.append('REF %.6f %.6f %d %s' % (qx, qy, len(w), ' '.join('%d %.7f' % kv for kv in sorted(w.items()))))
        else:
            xs = [s['x'] for s in smp]
            for qx in q1:
                cx = min(max(qx, mins[0]), maxs[0])
                w = ref1d(xs, cx)
                lines.append('REF %.6f 0 %d %s' % (qx, len(w), ' '.join('%d %.7f' % kv for kv in sorted(w.items()))))
        lines.append('END')
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote', a.out, len(lines), 'lines')
    export_montages(root, d, os.path.join(os.path.dirname(a.out), 'montages.txt'))


# ---- montages (report 4.2 #32) -------------------------------------------------------------------------------------
# Data/anim/montages.txt:
#   utpp-montages 1
#   M <name> <length> <rateScale> <blendInTime> <blendInOpt> <blendOutTime> <blendOutOpt> <autoBlendOut> <blendOutTrigger> <slot>
#   SEC <name> <startTime> <nextSection|->
#   SEG <anim> <startPos> <animStart> <animEnd> <playRate> <loopingCount>
#   N <state|event> <name> <class> <start> <duration> <warpTarget|->
#   RFPS <fps> <numFrames>            root track of the referenced sequence (retarget woman, bone Trajectory, metres)
#   R <tx> <ty> <tz> <qx> <qy> <qz> <qw>
#   END
# Sources: anims.json montage.sections / slot_tracks (T3D), Animations/sourcepack/als_export/meta/<montage>.json (blend in/out,
# auto blend out, notifies from the Python API), Animations/retarget/out_sourcepack_woman/<sequence>.utppclip.json.
MONTAGES = ['AM_M_Neutral_Traversal_Hurdle_1_0_run_F_Lfoot', 'AM_M_Neutral_Traversal_Mantle_1_0_stand_F_Lfoot']
OPT = {'HERMITE_CUBIC': 'HermiteCubic', 'LINEAR': 'Linear', 'CUBIC': 'Cubic', 'Linear': 'Linear', 'Cubic': 'Cubic',
       'HermiteCubic': 'HermiteCubic'}


def export_montages(root, anims, out):
    import ast
    by_name = {e['name']: e for e in anims['anims'] if e.get('class') == 'AnimMontage'}
    lines = ['utpp-montages 1']
    for name in MONTAGES:
        e = by_name[name]
        meta = json.load(open(os.path.join(root, 'Animations', 'sourcepack', 'als_export', 'meta', name + '.json'), encoding='utf-8'))
        bi, bo = meta['blend_in'], meta['blend_out']
        slot = meta['slot_names'][0]
        lines.append('M %s %.6f %.6f %.6f %s %.6f %s %d %.6f %s' % (
            name, meta['length_s'], meta['rate_scale'], bi['blend_time'], OPT[bi['blend_option']], bo['blend_time'],
            OPT[bo['blend_option']], 1 if meta['enable_auto_blend_out'] else 0, meta['blend_out_trigger_time'], slot))
        for s in e['montage']['sections']:
            lines.append('SEC %s %.6f %s' % (s['name'], s['start_time'], s.get('next_section') or '-'))
        segs = meta['slot_anim_tracks'][0]['anim_track']['anim_segments']
        for g in segs:
            lines.append('SEG %s %.6f %.6f %.6f %.6f %d' % (g['anim_reference'].rsplit('.', 1)[-1], g['start_pos'],
                                                          g['anim_start_time'], g['anim_end_time'], g['anim_play_rate'],
                                                          g['looping_count']))
        nots = meta['notifies']
        nots = ast.literal_eval(nots) if isinstance(nots, str) else nots
        for n in sorted(nots, key=lambda x: x['trigger_time']):
            rmm = (n.get('props') or {}).get('root_motion_modifier') or {}
            wt = rmm.get('warp_target_name') or '-'
            lines.append('N %s %s %s %.6f %.6f %s' % (n.get('kind', 'event'), n['notify_name'], n['class'],
                                                      n['trigger_time'], n.get('duration') or 0.0, wt))
        seq = segs[0]['anim_reference'].rsplit('.', 1)[-1]
        clip = json.load(open(os.path.join(root, 'Animations', 'retarget', 'out_sourcepack_woman', seq + '.utppclip.json'),
                              encoding='utf-8'))
        ti = clip['bone_names'].index('Trajectory')
        lines.append('RFPS %.6f %d' % (clip['fps'], clip['num_frames']))
        for fr in clip['frames']:
            b = fr['bones'][ti]
            lines.append('R %.6f %.6f %.6f %.7f %.7f %.7f %.7f' % tuple(b[:7]))
        lines.append('END')
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote', out, len(lines), 'lines')


if __name__ == '__main__':
    main()
