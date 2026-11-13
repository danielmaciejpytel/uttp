"""Independent visual check of a .utppclip.json: FK in Python with the game rig, stick figures (front + side).

Usage: python plot_clip.py <rig.json> <clip.utppclip.json> <out.svg> [frames]
"""
import json
import sys

import numpy as np  # noqa: E402


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rig = json.load(open(sys.argv[1], encoding='utf-8'))['Data']['RootChunk']
clip = json.load(open(sys.argv[2], encoding='utf-8'))
parents = rig['boneParentIndexes']
n = len(clip['bone_names'])
frames = [int(f) for f in sys.argv[4].split(',')] if len(sys.argv) > 4 else \
    np.linspace(0, clip['num_frames'] - 1, 6).astype(int).tolist()

W, H, S = 150, 260, 120.0  # cell size and pixels per metre
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W * len(frames)}" height="{2 * H + 20}" style="background:#fff">',
       f'<text x="4" y="14" font-size="11">{clip["source_clip"]} (game rig FK, model space)</text>']
for col, fi in enumerate(frames):
    bones = clip['frames'][fi]['bones']
    R, T = [None] * n, [None] * n
    for i in range(n):
        b = bones[i]
        r = qmat(*b[3:7]) * np.array(b[7:10])
        t = np.array(b[0:3])
        p = parents[i]
        if p < 0:
            R[i], T[i] = r, t
        else:
            R[i], T[i] = R[p] @ r, R[p] @ t + T[p]
    for row, (a, title) in enumerate(((0, 'front X-Z'), (1, 'side Y-Z'))):
        ox, oy = col * W + W / 2, 20 + row * H + H - 15
        svg.append(f'<text x="{col * W + 4}" y="{20 + row * H + 12}" font-size="10">{title} f{fi}</text>')
        svg.append(f'<line x1="{col * W}" y1="{oy}" x2="{col * W + W}" y2="{oy}" stroke="#bbb"/>')
        for i in range(n):
            p = parents[i]
            if p >= 2:
                x1, y1 = ox + T[p][a] * S, oy - T[p][2] * S
                x2, y2 = ox + T[i][a] * S, oy - T[i][2] * S
                color = '#c00' if 'Left' in clip['bone_names'][i] else ('#00c' if 'Right' in clip['bone_names'][i] else '#000')
                svg.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{color}" stroke-width="1.5"/>')
svg.append('</svg>')
open(sys.argv[3], 'w').write(chr(10).join(svg))
print('saved', sys.argv[3])
