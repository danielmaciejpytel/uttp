"""Contact sheet of car EXIT clips (game rig dumps of game_skeleton_bridge.py dump-glb): top view of the foot paths + legs at chosen frames, ankle distance and planted-foot speed over time.
Rows = clips, columns = frames. Left foot blue, right foot red, hips grey; the ground contact (ankle within 3 cm of the floor) is drawn as a filled dot, a swing as a hollow one.

python -I exit_steps_sheet.py <out.png> <floor_ref.dump.npz> LABEL=dump.npz [LABEL=dump.npz ...] [--frames 22,30,36,40,44,50,56,60]
"""
import sys

import numpy as np
from PIL import Image, ImageDraw

SC = 150.0   # px per metre (top view)
PW, PH = 330, 300


def load(p):
    d = np.load(p, allow_pickle=True)
    return d['W'], {str(n): i for i, n in enumerate(d['names'])}


def main(a):
    out, ref = a[0], a[1]
    frames = [22, 30, 36, 40, 44, 50, 56, 60]
    if '--frames' in a:
        frames = [int(x) for x in a[a.index('--frames') + 1].split(',')]
        a = [x for i, x in enumerate(a) if x != '--frames' and a[i - 1] != '--frames']
    clips = [x.split('=', 1) for x in a[2:]]
    Wr, ixr = load(ref)
    floor = {s: Wr[40:, ixr[s + 'Foot'], 2, 3].min() for s in ('Left', 'Right')}
    n = len(clips)
    cols = len(frames)
    img = Image.new('RGB', (cols * PW, n * (PH + 150)), (250, 250, 250))
    dr = ImageDraw.Draw(img)
    for r, (label, p) in enumerate(clips):
        W, ix = load(p)
        y0 = r * (PH + 150)
        hips = W[:, ix['Hips'], :3, 3]
        for c, f in enumerate(frames):
            cx = hips[f, 0]
            cy = hips[f, 1]
            ox, oy = c * PW + PW / 2, y0 + PH / 2
            P = lambda pt: (ox + (-(pt[0] - cx)) * SC, oy + (-(pt[1] - cy)) * SC)   # travel (-x) to the right, +y (forward of the car) up
            dr.rectangle([c * PW, y0, (c + 1) * PW - 1, y0 + PH - 1], outline=(200, 200, 200))
            for s, col in (('Left', (40, 90, 220)), ('Right', (220, 60, 40))):
                tr = [P(W[t, ix[s + 'Foot'], :3, 3]) for t in range(len(W))]
                dr.line(tr, fill=tuple(int(0.45 * v + 0.55 * 255) for v in col), width=1)
            dr.line([P(hips[t]) for t in range(len(W))], fill=(190, 190, 190), width=1)
            for s, col in (('Left', (40, 90, 220)), ('Right', (220, 60, 40))):
                ch = [P(W[f, ix[s + b], :3, 3]) for b in ('UpLeg', 'Leg', 'Foot', 'ToeBase')]
                dr.line([P(hips[f])] + ch, fill=col, width=3)
                a_ = W[f, ix[s + 'Foot'], 2, 3] - floor[s]
                x, y = ch[2]
                if a_ < 0.03:
                    dr.ellipse([x - 5, y - 5, x + 5, y + 5], fill=col)
                else:
                    dr.ellipse([x - 5, y - 5, x + 5, y + 5], outline=col, width=2)
            la, ra = W[f, ix['LeftFoot'], :2, 3], W[f, ix['RightFoot'], :2, 3]
            dr.text((c * PW + 6, y0 + 4), '%s  f%d  feet %.0f cm' % (label, f, np.linalg.norm(la - ra) * 100), fill=(0, 0, 0))
        # time chart: ankle distance (black) and planted speed of both feet (blue / red), cm
        cy0 = y0 + PH + 5
        cw = cols * PW - 20
        dr.rectangle([10, cy0, 10 + cw, cy0 + 140], outline=(180, 180, 180))
        T = len(W)
        d = np.linalg.norm(W[:, ix['LeftFoot'], :2, 3] - W[:, ix['RightFoot'], :2, 3], axis=1) * 100
        for lvl in (20, 40, 60, 80, 100):
            y = cy0 + 140 - lvl * 1.3
            dr.line([10, y, 10 + cw, y], fill=(225, 225, 225))
            dr.text((12, y - 10), '%d cm' % lvl, fill=(120, 120, 120))
        dr.line([(10 + t * cw / (T - 1), cy0 + 140 - min(d[t], 105) * 1.3) for t in range(T)], fill=(0, 0, 0), width=2)
        for s, col in (('Left', (40, 90, 220)), ('Right', (220, 60, 40))):
            A = W[:, ix[s + 'Foot'], :3, 3]
            sp = np.r_[0, np.linalg.norm(np.diff(A[:, :2], axis=0), axis=1)] * 100
            con = (A[:, 2] - floor[s]) < 0.03
            pts = [(10 + t * cw / (T - 1), cy0 + 140 - min(sp[t], 20) * 5.0) for t in range(T)]
            for t in range(1, T):
                dr.line([pts[t - 1], pts[t]], fill=col if (con[t] and con[t - 1]) else tuple(int(0.4 * v + 0.6 * 255) for v in col), width=2 if (con[t] and con[t - 1]) else 1)
        dr.text((cw - 330, cy0 + 4), 'black: ankle distance; blue / red: ankle speed (cm/frame x5), bold = planted', fill=(60, 60, 60))
        dr.text((12, cy0 + 126), '0   frame -> %d' % (T - 1), fill=(120, 120, 120))
    img.save(out)
    print('SHEET', out, img.size)


if __name__ == '__main__':
    main(sys.argv[1:])
