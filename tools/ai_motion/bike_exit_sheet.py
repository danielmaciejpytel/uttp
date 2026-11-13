"""Front-view stick-figure contact sheet of a MOTORCYCLE get-off clip, before vs after (world positions with Root motion, offline, PIL only).

python -I bike_exit_sheet.py --out sheet.png --clip LABEL=path.glb:clip [--clip ...] [--frames 6,14,22,...] [--view front|side]
Rows = clips, columns = frames; x (front) or y (side) horizontal, z up, one fixed window for all panels. Ground = the lowest toe of the first clip's last frame, ticks every 0.25 m (a planted foot stays
on its tick, a gliding one moves along it). Left leg red, right leg blue, spine black, arms green; the pinned ankle of the left foot is marked with a ring; the hips trail is the grey line.
"""
import argparse
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bike_steps_gen as BSG  # noqa: E402

CH = {'spine': (['Hips', 'Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Head'], (0, 0, 0)),
      'L': (['Hips', 'LeftUpLeg', 'LeftLeg', 'LeftFoot', 'LeftToeBase'], (215, 50, 40)),
      'R': (['Hips', 'RightUpLeg', 'RightLeg', 'RightFoot', 'RightToeBase'], (40, 90, 215)),
      'LA': (['Spine3', 'LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand'], (40, 150, 70)),
      'RA': (['Spine3', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand'], (40, 150, 70))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--clip', action='append', required=True)
    ap.add_argument('--frames', default='6,14,22,30,36,40,42,44,46,48,52,58')
    ap.add_argument('--view', default='front', choices=('front', 'side'))
    ap.add_argument('--scale', type=float, default=150.0, help='px per metre')
    a = ap.parse_args()
    frames = [int(x) for x in a.frames.split(',')]
    data = []
    for spec in a.clip:
        lab, src = spec.split('=', 1)
        W, names = BSG.load(src)
        data.append((lab, W, {n: i for i, n in enumerate(names)}))
    ax = 0 if a.view == 'front' else 1
    allp = np.concatenate([W[:, :, :3, 3].reshape(-1, 3) for _, W, _ in data], 0)
    x0, x1 = float(allp[:, ax].min()) - 0.15, float(allp[:, ax].max()) + 0.15
    z0, z1 = float(allp[:, 2].min()) - 0.1, float(allp[:, 2].max()) + 0.1
    pw, ph = int((x1 - x0) * a.scale), int((z1 - z0) * a.scale)
    lab_w = 110
    img = Image.new('RGB', (lab_w + pw * len(frames), ph * len(data)), (245, 245, 245))
    d = ImageDraw.Draw(img)
    W0, ix0 = data[0][1], data[0][2]
    ground = float(min(W0[-1, ix0['LeftToeBase'], 2, 3], W0[-1, ix0['RightToeBase'], 2, 3]))

    def px(p, col, row):
        return (lab_w + col * pw + (p[ax] - x0) * a.scale, row * ph + (z1 - p[2]) * a.scale)

    for r, (lab, W, ix) in enumerate(data):
        d.text((4, r * ph + 4), lab, fill=(0, 0, 0))
        hips = W[:, ix['Hips'], :3, 3]
        for c, f_req in enumerate(frames):
            f = min(f_req, len(W) - 1)
            d.rectangle([lab_w + c * pw, r * ph, lab_w + (c + 1) * pw - 1, (r + 1) * ph - 1], outline=(190, 190, 190))
            d.text((lab_w + c * pw + 3, r * ph + 3), 'f%d' % f_req, fill=(0, 0, 0))
            gy = r * ph + (z1 - ground) * a.scale
            d.line([lab_w + c * pw, gy, lab_w + (c + 1) * pw, gy], fill=(120, 120, 120))
            t = np.ceil(x0 / 0.25) * 0.25
            while t < x1:
                X = lab_w + c * pw + (t - x0) * a.scale
                d.line([X, gy, X, gy + 5], fill=(120, 120, 120))
                t += 0.25
            pts = [px(hips[k], c, r) for k in range(0, f + 1)]
            d.line(pts, fill=(170, 170, 170))
            for key, (chain, col) in CH.items():
                P = [px(W[f, ix[n], :3, 3], c, r) for n in chain if n in ix]
                d.line(P, fill=col, width=2)
            for s_, col in (('Left', (215, 50, 40)), ('Right', (40, 90, 215))):
                fx, fy = px(W[f, ix[s_ + 'Foot'], :3, 3], c, r)
                d.ellipse([fx - 3, fy - 3, fx + 3, fy + 3], outline=col)
    img.save(a.out)
    print('wrote', a.out, img.size)


if __name__ == '__main__':
    main()
