"""Contact sheet of the car ENTRY glide pin (2026-11-06): one row per clip, left = top view of both ankle paths (grey = shipped clip, blue / red = left / right foot after the pin; filled dot = frame
inside a pinned span c..g, the planted-foot glide of the shipped clip is marked with a black ring), right = ankle speed over time (cm / frame, thin = shipped, thick = after, red ticks = glide frames
of the shipped clip, shaded = spliced span c..g).

python -I entry_steps_sheet.py <out.png> <row dir 1> [<row dir 2> ...]    (row dir = entry_steps_batch.py output of one row: cur.dump.npz, after_pin.dump.npz, plants.json, steps_pin.steps.npz)
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import entry_steps_gen as E  # noqa: E402

PW, PH, GW = 380, 300, 520


def ank(d, names, s):
    ix = {n: i for i, n in enumerate(names)}
    return d['W'][:, ix[s + 'Foot'], :3, 3]


def main(a):
    out, dirs = a[0], a[1:]
    sheet = Image.new('RGB', (PW + GW, PH * len(dirs)), 'white')
    dr = ImageDraw.Draw(sheet)
    for r, rd in enumerate(dirs):
        d0 = np.load(os.path.join(rd, 'cur.dump.npz'), allow_pickle=True)
        d1 = np.load(os.path.join(rd, 'after_pin.dump.npz'), allow_pickle=True)
        st = np.load(os.path.join(rd, 'steps_pin.steps.npz'), allow_pickle=True)
        pl = json.load(open(os.path.join(rd, 'plants.json')))
        names = [str(x) for x in d0['names']]
        c, g = int(st['c']), int(st['g'])
        y0 = r * PH
        allp = np.concatenate([ank(d0, names, s)[:, :2] for s in E.SIDES])
        lo, hi = allp.min(0) - 0.15, allp.max(0) + 0.15
        sc = min((PW - 20) / (hi[0] - lo[0]), (PH - 40) / (hi[1] - lo[1]))
        px = lambda p: (10 + (p[0] - lo[0]) * sc, y0 + PH - 20 - (p[1] - lo[1]) * sc)
        dr.text((8, y0 + 4), os.path.basename(rd)[9:] + '  span %d..%d' % (c, g), fill='black')
        g0 = E.plant_summary(d0['W'], names, pl)
        for s, col in zip(E.SIDES, ((40, 90, 230), (220, 50, 50))):
            A0, A1 = ank(d0, names, s), ank(d1, names, s)
            for f in range(len(A0) - 1):
                dr.line([px(A0[f]), px(A0[f + 1])], fill=(185, 185, 185), width=1)
                dr.line([px(A1[f]), px(A1[f + 1])], fill=col, width=2)
            for f in range(len(A1)):
                x, y = px(A1[f])
                if c <= f <= g:
                    dr.ellipse([x - 2, y - 2, x + 2, y + 2], fill=col)
                if g0[s]['sp'][f] > 1.0:
                    x0, y0_ = px(A0[f])
                    dr.ellipse([x0 - 5, y0_ - 5, x0 + 5, y0_ + 5], outline='black')
        # speed plot
        T = len(d0['W'])
        ox, oy, w, h = PW + 20, y0 + 20, GW - 40, PH - 50
        for s, col in zip(E.SIDES, ((40, 90, 230), (220, 50, 50))):
            v0 = np.r_[0, np.linalg.norm(np.diff(ank(d0, names, s), axis=0), axis=1)] * 100
            v1 = np.r_[0, np.linalg.norm(np.diff(ank(d1, names, s), axis=0), axis=1)] * 100
            vm = 22.0
            dr.rectangle([ox + c / (T - 1) * w, oy, ox + g / (T - 1) * w, oy + h], fill=(240, 240, 235))
            for f in range(T - 1):
                dr.line([(ox + f / (T - 1) * w, oy + h - min(v0[f], vm) / vm * h), (ox + (f + 1) / (T - 1) * w, oy + h - min(v0[f + 1], vm) / vm * h)], fill=(170, 170, 170), width=1)
                dr.line([(ox + f / (T - 1) * w, oy + h - min(v1[f], vm) / vm * h), (ox + (f + 1) / (T - 1) * w, oy + h - min(v1[f + 1], vm) / vm * h)], fill=col, width=2)
            for f in np.flatnonzero(g0[s]['sp'] > 1.0):
                dr.line([(ox + f / (T - 1) * w, oy + h), (ox + f / (T - 1) * w, oy + h + 6)], fill='red', width=2)
        dr.rectangle([ox, oy, ox + w, oy + h], outline='black')
        gm = max(g0[s]['max'] for s in E.SIDES)
        g1 = E.plant_summary(d1['W'], names, pl)
        dr.text((ox, oy + h + 10), 'glide max (cm/frame) before %.2f -> after %.2f | mean %.2f -> %.2f | grey = shipped, colour = after, shaded = span, red ticks = glide frames' % (
            gm, max(g1[s]['max'] for s in E.SIDES), np.mean([g0[s]['mean'] for s in E.SIDES]), np.mean([g1[s]['mean'] for s in E.SIDES])), fill='black')
        dr.line([(0, y0 + PH - 1), (PW + GW, y0 + PH - 1)], fill='black')
    sheet.save(out)
    print('SHEET', out, sheet.size)


if __name__ == '__main__':
    main(sys.argv[1:])
