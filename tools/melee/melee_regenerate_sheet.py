"""Contact sheets of the v4 demo clips: V3 (redo) vs V4 side by side, 3/4 and side view, at chosen frames (to LOOK at the result).

  python -I melee_regenerate_sheet.py <set> <clip> [--frames 3,9,14,...] [--n 6] [--out PNG] [--rig man]
Without --frames: sheet_segments = frames spread over the generated (mb) segments; sheet_joins = key -1 / key / key +1 of every key bordering a generated segment; sheet_departure = frames where V4 departs most from V3 (Spine3, hand, hips, acceleration, gate margin).
Columns: 3q v3 | 3q v4 | side v3 | side v4 (from <mb4>\\demo3\\<set>__<clip>\\{3q,side}_{redo,v4}\\f_####.png, k = 2 * frame).
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_regenerate as M  # noqa: E402


def sheet(d, frames, out, title):
    tiles = []
    for f in frames:
        row = []
        for view in ('3q', 'side'):
            for var in ('redo', 'v4'):
                im = Image.open(os.path.join(d, '%s_%s' % (view, var), 'f_%04d.png' % (2 * f))).convert('RGB')
                im = im.resize((im.width // 2, im.height // 2))
                dr = ImageDraw.Draw(im)
                dr.text((4, 4), '%s f%d %s' % ('V3' if var == 'redo' else 'V4', f, view), fill=(0, 0, 0))
                row.append(im)
        tiles.append(row)
    w, h = tiles[0][0].size
    S = Image.new('RGB', (w * 4, h * len(tiles)), (255, 255, 255))
    for r, row in enumerate(tiles):
        for c, im in enumerate(row):
            S.paste(im, (c * w, r * h))
    S.save(out)
    print('SHEET', out, frames, title)


def main(a):
    s, c = a[0], a[1]
    d = os.path.join(M.MB4, 'demo3', '%s__%s' % (s, c))
    info = json.load(open(os.path.join(M.MB4, 'clips', '%s__%s.v4.info.json' % (s, c))))
    T = info['frames']
    n = int(a[a.index('--n') + 1]) if '--n' in a else 6
    if '--frames' in a:
        fr = [int(x) for x in a[a.index('--frames') + 1].split(',')]
        sheet(d, fr, a[a.index('--out') + 1] if '--out' in a else os.path.join(d, 'sheet_custom.png'), 'custom')
        return
    mid = []
    for sg in info['segments']:
        if sg['status'] == 'mb':
            for q in (0.25, 0.5, 0.75):
                mid.append(int(round(sg['c'] + q * (sg['g'] - sg['c']))))
    if not mid:
        mid = [int(x) for x in np.linspace(0, T - 1, n)]
    pick = sorted(set(mid))
    if len(pick) > n:
        pick = [pick[int(i)] for i in np.linspace(0, len(pick) - 1, n)]
    sheet(d, pick, os.path.join(d, 'sheet_segments.png'), 'mb segment frames')
    import anatomy_gate_base as GB
    B = M.RS.Body()
    base = os.path.join(M.MB4, 'clips', '%s__%s' % (s, c))
    W3, W4 = np.load(base + '.v3.npz')['W'], np.load(base + '.v4.npz')['W']
    ix = B.ix
    # joins: key - 1, key, key + 1 of the interior keys that border a generated segment
    jk = sorted(set(k for sg in info['segments'] if sg['status'] == 'mb' for k in (sg['c'], sg['g']) if 0 < k < T - 1))
    fr = sorted(set(min(max(f, 0), T - 1) for k in jk for f in (k - 1, k, k + 1)))
    if fr:
        sheet(d, fr[:8], os.path.join(d, 'sheet_joins.png'), 'key -1 / key / key +1 of keys %s' % jk)
    # departure: frames where V4 differs most from V3 (hips, chest, hand, accel, gate margin)
    g3, g4 = GB.frame_gate(W3, B.rg), GB.frame_gate(W4, B.rg)
    chest = M.rot_angle_deg(W4[:, ix['Spine3'], :3, :3], W3[:, ix['Spine3'], :3, :3])
    hand = np.max([np.linalg.norm(W4[:, ix[S + 'Hand'], :3, 3] - W3[:, ix[S + 'Hand'], :3, 3], axis=1) for S in ('Left', 'Right')], axis=0)
    hips = np.linalg.norm(W4[:, ix['Hips'], :3, 3] - W3[:, ix['Hips'], :3, 3], axis=1)
    P4, P3 = W4[:, :, :3, 3], W3[:, :, :3, 3]
    ac = lambda P: np.concatenate([[0], np.linalg.norm((P[2:] - 2 * P[1:-1] + P[:-2]), axis=2).max(1), [0]]) * 100
    pick = [int(chest.argmax()), int(hand.argmax()), int(hips.argmax()), int((ac(P4) - ac(P3)).argmax()), int((g4['margin'] - g3['margin']).argmin())]
    pick = sorted(set(pick))
    sheet(d, pick, os.path.join(d, 'sheet_departure.png'), 'max Spine3 dev %d (%.1f deg), max hand dev %d (%.1f cm), max hips dev %d (%.1f cm), max accel increase %d, worst margin change %d' % (
        chest.argmax(), chest.max(), hand.argmax(), hand.max() * 100, hips.argmax(), hips.max() * 100, (ac(P4) - ac(P3)).argmax(), (g4['margin'] - g3['margin']).argmin()))


if __name__ == '__main__':
    main(sys.argv[1:])
