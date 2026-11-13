"""Stage 6: ONE contact sheet - Female V skinned (graph deformation, skin_clip.py via demo_skin) at the worst visible frame of the
prototype clips, vanilla shown pose vs prototype guard, two views each.  Software rendering (PIL painter's algorithm, flat
shading, orthographic): a look at the shapes, not a game render.

  python -I survey_sheet.py --rig woman [--n 12] --out <png>
Reads Build\\work\\anim-survey-20261109\\skin\\<rig>\\{v,g}__<clip>.skin.{npz,pc2} and proto_<rig>.json.
"""
import io
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

from PIL import Image, ImageDraw  # noqa: E402

RIG = sys.argv[sys.argv.index('--rig') + 1] if '--rig' in sys.argv else 'woman'
N = int(sys.argv[sys.argv.index('--n') + 1]) if '--n' in sys.argv else 12
OUT = sys.argv[sys.argv.index('--out') + 1]
SK = os.path.join(SC.WORK, 'skin', RIG)


def load(tag, k):
    z = np.load(os.path.join(SK, tag + '.skin.npz'))
    nv = len(z['rest'])
    pc = np.fromfile(os.path.join(SK, tag + '.skin.pc2'), dtype='<f4', offset=32).reshape(-1, nv, 3)
    return pc[k].astype(float), z['faces'], z['rest']


def tile(P, F, up, azim, title, color, size=(230, 300)):
    """painter's-algorithm flat-shaded render (PIL polygons, far to near), orthographic, elevation 8 deg."""
    c = P.mean(0)
    P = P - c
    order = [i for i in range(3) if i != up] + [up]
    Q = P[:, order]                                   # x, y horizontal, z up
    a, e = np.radians(azim), np.radians(8.0)
    right = np.array([np.cos(a), np.sin(a), 0.0])
    fwd = np.array([-np.sin(a) * np.cos(e), np.cos(a) * np.cos(e), np.sin(e)])
    upv = np.cross(right, fwd)
    sx, sy, dz = Q @ right, Q @ upv, Q @ fwd
    tri = Q[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    light = -fwd * 0.8 + upv * 0.5 + right * 0.3
    sh = np.clip(np.abs(n @ (light / np.linalg.norm(light))), 0.18, 1.0)
    W_, H_ = size
    r = max(np.abs(sx).max(), np.abs(sy).max()) * 1.08
    px = (sx / r * 0.5 + 0.5) * (W_ - 1)
    py = (0.5 - sy / r * 0.5) * (H_ - 1)
    img = Image.new('RGB', size, 'white')
    d = ImageDraw.Draw(img)
    depth = dz[F].mean(1)
    for t in np.argsort(-depth):
        f = F[t]
        cc = tuple(int(255 * min(1.0, x * sh[t])) for x in color)
        d.polygon([(px[f[0]], py[f[0]]), (px[f[1]], py[f[1]]), (px[f[2]], py[f[2]])], fill=cc)
    d.text((3, H_ - 12), title, fill='black')
    return img


def main():
    pr = json.load(open(os.path.join(SC.WORK, 'proto_%s.json' % RIG), encoding='utf-8'))
    keys = sorted(pr, key=lambda k: -pr[k]['fail_pct_vanilla'] - 10 * pr[k]['skin_excess_vanilla'])[:N]
    rows = []
    for k in keys:
        v = pr[k]
        tag = k.replace('/', '__')
        fr = v['skin_frames']
        kk = 1 if len(fr) > 1 else 0                   # the worst frame sits in the middle of its +-1 triple
        P0, F, rest = load('v__' + tag, kk)
        P2, _, _ = load('g__' + tag, kk)
        up = int(np.argmax(np.ptp(rest, axis=0)))
        lab = '%s  f%d  %s' % (k[:48], fr[kk], v['verdict'])
        ims = [tile(P0, F, up, 150, 'vanilla 3/4 front', (0.85, 0.55, 0.5)), tile(P2, F, up, 150, 'guard 3/4 front', (0.5, 0.7, 0.85)),
               tile(P0, F, up, 90, 'vanilla side', (0.85, 0.55, 0.5)), tile(P2, F, up, 90, 'guard side', (0.5, 0.7, 0.85))]
        h = max(i.height for i in ims)
        w = sum(i.width for i in ims)
        row = Image.new('RGB', (w, h + 16), 'white')
        x = 0
        for i in ims:
            row.paste(i, (x, 16))
            x += i.width
        d = ImageDraw.Draw(row)
        d.text((4, 2), lab + '   fail %.0f%%->%.0f%%  skin %.2f->%.2f  hand %.1fcm  pop %.0f' % (
            v['fail_pct_vanilla'], v['fail_pct_guard'], v['skin_excess_vanilla'], v['skin_excess_guard'], v['hand_residual_cm_max'], v['pop_deg']),
            fill='black')
        rows.append(row)
        print('row', k)
    W = max(r.width for r in rows)
    sheet = Image.new('RGB', (W * 2, (sum(r.height for r in rows) + 1) // 2 + max(r.height for r in rows) + 30), 'white')
    d = ImageDraw.Draw(sheet)
    d.text((6, 6), 'UTPP anim deformation survey 2026-11-09 - Female V, worst visible frames: vanilla FPP clip as shown in TPP (red) vs '
                   'offline guard prototype (blue). Software render of the skinned offline mesh, not a game capture.', fill='black')
    y = [30, 30]
    for i, r in enumerate(rows):
        c = 0 if y[0] <= y[1] else 1
        sheet.paste(r, (c * W, y[c]))
        y[c] += r.height
    sheet = sheet.crop((0, 0, W * 2, max(y)))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    sheet.save(OUT)
    print('WROTE', OUT, sheet.size)


if __name__ == '__main__':
    main()
