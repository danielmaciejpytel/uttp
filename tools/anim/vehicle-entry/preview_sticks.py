"""Stick-figure contact sheet of entry clips (no meshes needed): rows = clip x view (top = X/Z plane, side = Z/Y plane), columns = clip-time
fractions. Root motion is kept (the approach to the seat is visible); one fixed window per clip so clips of one set are comparable. The
seat (Root at the last frame) is the red cross, the ground the grey line (side) and the start spot the blue dot.

python -I preview_sticks.py --out sheet.png --frames 0,0.2,0.35,0.5,0.65,0.8,1.0 --clip LABEL=path.glb:clip [--clip ...]
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402

BODY = ['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Head']
CHAINS = [BODY, ['Spine3', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand'], ['Spine3', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand'],
          ['Hips', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase'], ['Hips', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase']]
COL = {'Left': (40, 90, 220), 'Right': (220, 60, 40)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', default='0,0.2,0.35,0.5,0.65,0.8,1.0')
    ap.add_argument('--clip', action='append', required=True)
    ap.add_argument('--cell', type=int, default=170)
    a = ap.parse_args()
    fr = [float(x) for x in a.frames.split(',')]
    cell = a.cell
    clips = []
    for spec in a.clip:
        label, rest = spec.split('=', 1)
        path, cn = rest.rsplit(':', 1)
        sk = S.Skeleton.from_glb(path)
        c = sk.clip(cn)
        dur, t = B.frames_of(sk, c)
        lt, lr = sk.sample_local(c, t)
        Wt, _ = sk.world(lt, S.qcontinuous(lr))
        clips.append((label, sk, t, Wt, lt))
    # common window over all clips: x/z extent
    allp = np.concatenate([w[:, [sk.index['Hips'], sk.index['Head'], sk.index['LeftFoot'], sk.index['RightFoot'], sk.index['LeftHand'], sk.index['RightHand']]].reshape(-1, 3)
                           for _, sk, _, w, _ in clips])
    x0, x1 = allp[:, 0].min() - 0.3, allp[:, 0].max() + 0.3
    z0, z1 = allp[:, 2].min() - 0.3, allp[:, 2].max() + 0.3
    y1 = allp[:, 1].max() + 0.2
    img = Image.new('RGB', (cell * len(fr) + 90, cell * 2 * len(clips)), (245, 245, 245))
    d = ImageDraw.Draw(img)
    for ci, (label, sk, t, Wt, lt) in enumerate(clips):
        ri = sk.index['Root']
        seat = lt[-1, ri]
        start = lt[0, ri]
        for vi, view in enumerate(('top', 'side')):
            oy = (ci * 2 + vi) * cell
            d.text((4, oy + 4), label + ('\n' + view), fill=(0, 0, 0))
            for fi, f in enumerate(fr):
                i = int(round(f * (len(t) - 1)))
                ox = 90 + fi * cell
                d.rectangle([ox, oy, ox + cell - 1, oy + cell - 1], outline=(200, 200, 200))
                if view == 'top':
                    sx = (cell - 10) / max(x1 - x0, z1 - z0)

                    def P(p):
                        return (ox + 5 + (p[0] - x0) * sx, oy + 5 + (p[2] - z0) * sx)
                    d.ellipse([P(start)[0] - 3, P(start)[1] - 3, P(start)[0] + 3, P(start)[1] + 3], fill=(60, 60, 255))
                    cx, cy = P(seat)
                    d.line([cx - 5, cy, cx + 5, cy], fill=(255, 0, 0)); d.line([cx, cy - 5, cx, cy + 5], fill=(255, 0, 0))
                else:
                    sx = (cell - 10) / max(z1 - z0, y1 + 0.05)

                    def P(p):
                        return (ox + 5 + (p[2] - z0) * sx, oy + cell - 5 - (p[1] + 0.05) * sx)
                    gy = P(np.array([0, 0.0, 0]))[1]
                    d.line([ox, gy, ox + cell, gy], fill=(150, 150, 150))
                    cx, cy = P(np.array([0, seat[1], seat[2]]))
                    d.line([cx - 5, cy, cx + 5, cy], fill=(255, 0, 0)); d.line([cx, cy - 5, cx, cy + 5], fill=(255, 0, 0))
                for ch in CHAINS:
                    side = 'Left' if any(n.startswith('Left') for n in ch) else ('Right' if any(n.startswith('Right') for n in ch) else None)
                    col = COL.get(side, (20, 20, 20))
                    pts = [P(Wt[i, sk.index[n]]) for n in ch if n in sk.index]
                    d.line(pts, fill=col, width=2)
                hd = P(Wt[i, sk.index['Head']])
                d.ellipse([hd[0] - 4, hd[1] - 4, hd[0] + 4, hd[1] + 4], outline=(20, 20, 20))
    img.save(a.out)
    print('wrote', a.out, img.size)


if __name__ == '__main__':
    main()
