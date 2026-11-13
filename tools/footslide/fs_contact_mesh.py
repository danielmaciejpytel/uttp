"""Mesh check of foot contact frames: skins a clip with the game body mesh (Source/tools/ai_motion/skin_clip.py --world --deform rigid,
barefoot V body) and measures, per frame and foot, the lowest foot-mesh vertex above the floor (floor = lowest foot vertex of the clip over
both feet). Independent of the joint-height rule used by fs_contacts / fs_audit / fs_contact_fix (heel AND toe joint > 5 cm).

python -I fs_contact_mesh.py <woman|man> <workdir> <frames.json> <out.json> [--clipdir DIR]
  frames.json: {"clipdir": "...", "clips": {"<clip name>": {"contact_l": [frames...], "contact_r": [frames...]}, ...}}, or the
               fs_contact_fix.py verify --frames-json output (per side {removed, residual, before}: residual + removed frames are measured).
  --clipdir:   read the clips from DIR instead of frames.json clipdir (e.g. the patched dir: same bones, new contact curves).
  out.json:    per clip / side / frame: sole height above the floor (cm) + the joint-rule heights, for the listed frames, and the standing
               level (median sole height on the clip's contact frames that are grounded by the joint rule).
Foot vertex set per side: vertices (on the frame where that heel is lowest) below 14 cm and within 16 cm (horizontal) of that foot's Heel-ToeBase segment, nearer to it
than to the other foot (barefoot body mesh; toes included).
"""
import json
import os
import struct
import subprocess
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L  # noqa: E402

SKIN = os.path.join(L.ROOT, 'Source', 'tools', 'ai_motion', 'skin_clip.py')


def read_pc2(p):
    with open(p, 'rb') as f:
        hdr = f.read(32)
        assert hdr[:11] == b'POINTCACHE2'
        _ver, npts, _start, _rate, ns = struct.unpack('<iiffi', hdr[12:32])
        a = np.frombuffer(f.read(), dtype='<f4', count=ns * npts * 3).reshape(ns, npts, 3)
    return a


def seg_dist2d(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def foot_sets(V, c):
    """per side: vertex indices of the foot, picked on the frame where that foot's heel is lowest (a pivot clip may start with a foot
    lifted above the 14 cm cut)."""
    gp, gq, wp = c.world()
    out = {}
    for side, other in (('Left', 'Right'), ('Right', 'Left')):
        k = int(np.argmin(wp[:, c.bi[side + 'Heel'], 2]))
        v = V[k]
        d = seg_dist2d(v[:, :2], wp[k, c.bi[side + 'Heel']][:2], wp[k, c.bi[side + 'ToeBase']][:2])
        do = seg_dist2d(v[:, :2], wp[k, c.bi[other + 'Heel']][:2], wp[k, c.bi[other + 'ToeBase']][:2])
        out[side] = np.nonzero((v[:, 2] < 0.14) & (d < 0.16) & (d < do))[0]
    return out


def side_frames(v):
    """frames.json side entry: a frame list, or the fs_contact_fix.py verify --frames-json dict {removed, residual, before}."""
    if isinstance(v, dict):
        return [(int(k), 'residual') for k in v.get('residual', [])] + [(int(k), 'removed') for k in v.get('removed', [])]
    return [(int(k), 'listed') for k in v]


def main(g, work, frames_json, out_json, clipdir=None):
    spec = json.load(open(frames_json, encoding='utf-8'))
    os.makedirs(work, exist_ok=True)
    res = {}
    summary = []
    for name, sides in spec['clips'].items():
        if not any(side_frames(v) for v in sides.values()):
            continue
        clip_path = os.path.join(clipdir or spec['clipdir'], name + '.utppclip.json')
        base = os.path.join(work, ('m_' if g == 'man' else 'w_') + name)
        if not os.path.exists(base + '.pc2'):
            r = subprocess.run([sys.executable, '-I', SKIN, clip_path, base, '--deform', 'rigid', '--world', '--body', g],
                               capture_output=True, text=True)
            if r.returncode != 0:
                res[name] = dict(error=(r.stdout + r.stderr)[-400:])
                continue
        V = read_pc2(base + '.pc2')
        c = L.Clip(clip_path, g)
        if V.shape[0] != c.N:
            res[name] = dict(error='frames %d != %d' % (V.shape[0], c.N))
            continue
        fs = foot_sets(V, c)
        sole = {s: V[:, fs[s], 2].min(axis=1) for s in fs}
        floor = min(sole['Left'].min(), sole['Right'].min())
        gh, gt = L.ground_levels(c)
        r = dict(nverts={s: int(len(fs[s])) for s in fs}, floor_cm=float(floor * 100), sides={}, standing_cm={})
        for side, key in (('Left', 'contact_l'), ('Right', 'contact_r')):
            a, h, t = c.foot_points(side)
            hr, tr = h[:, 2] - gh, t[:, 2] - gt
            # standing level: median sole height on contact frames (curves of the skinned clip file) that are grounded by the joint rule
            cv = np.asarray(c.c.get(key, np.zeros(c.N)), dtype=float)
            grounded = (cv >= 0.5) & ~((hr > 0.05) & (tr > 0.05))
            stand = float(np.median(sole[side][grounded] - floor) * 100) if grounded.any() else float('nan')
            r['standing_cm'][key] = stand
            rows = []
            for k, kind in side_frames(sides.get(key, [])):
                rows.append(dict(f=k, kind=kind, sole_cm=float((sole[side][k] - floor) * 100), heel_cm=float(hr[k] * 100),
                                 toe_cm=float(tr[k] * 100)))
            r['sides'][key] = rows
            r.setdefault('sole_all_cm', {})[key] = [round(float((x - floor) * 100), 2) for x in sole[side]]
            for kind in ('residual', 'removed', 'listed'):
                s = [x['sole_cm'] for x in rows if x['kind'] == kind]
                if s:
                    summary.append((name, key, kind, len(s), min(s), max(s), stand))
        res[name] = r
    with open(out_json, 'w') as fh:
        json.dump(res, fh, indent=1)
    print('%-46s %-9s %-8s %3s %7s %7s %8s %9s' % ('clip', 'side', 'kind', 'n', 'sole_min', 'sole_max', 'standing', 'max-stand'))
    for (n, key, kind, cnt, lo, hi, st) in summary:
        print('%-46s %-9s %-8s %3d %7.1f %7.1f %8.1f %9.1f' % (n.replace('M_Neutral_', ''), key, kind, cnt, lo, hi, st, hi - st))
    for kind in ('residual', 'removed'):
        rows = [x for x in summary if x[2] == kind]
        if rows:
            print('%s: %d clip sides, %d frames, sole above the clip floor %.1f..%.1f cm, above the standing level max %.1f cm' % (
                kind, len(rows), sum(x[3] for x in rows), min(x[4] for x in rows), max(x[5] for x in rows),
                max(x[5] - x[6] for x in rows)))


if __name__ == '__main__':
    av = sys.argv[1:]
    cd = None
    if '--clipdir' in av:
        i = av.index('--clipdir')
        cd = av[i + 1]
        del av[i:i + 2]
    main(*av[:4], clipdir=cd)
