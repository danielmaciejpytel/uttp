"""Software (numpy + PIL) renderer for render_nue_reload_man.py caches: grid PNG, rows = clips (female/male pairs),
columns = frames, side and front view, vertex + triangle-centroid splatting with a z-buffer and Lambert shading.
Used because headless Blender 5.2 workbench crashes (tbbmalloc access violation) on this machine.
Offline math preview of LBS only (no engine skinning/helper joints).

python -I render_nue_reload_man_png.py <cache base> <out png prefix> [--px-per-m 300]
"""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

COLORS = [(217, 115, 64), (89, 140, 217)]  # female, male


def normals(v, f):
    n = np.zeros_like(v)
    tri = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], tri)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return n / np.maximum(ln, 1e-12), tri


def render_cell(v, f, view, ppm, w, h, color, zmin_z=-0.05):
    # Blender-style coordinates (X, Y, Z-up). side: right = +Y, depth = X (camera on -X). front: right = -X, depth = -Y.
    n, tri = normals(v, f)
    cen = v[f].mean(axis=1)
    cn = tri / np.maximum(np.linalg.norm(tri, axis=1, keepdims=True), 1e-12)
    pts = np.concatenate([v, cen])
    nrm = np.concatenate([n, cn])
    if view == 'side':
        hx, depth, look = pts[:, 1], pts[:, 0], np.array([1.0, 0, 0])
    else:
        hx, depth, look = -pts[:, 0], -pts[:, 1], np.array([0, -1.0, 0])
    shade = 0.30 + 0.70 * np.abs(nrm @ -look)
    px = (hx * ppm + w / 2).astype(int)
    py = (h - 12 - (pts[:, 2] - zmin_z) * ppm).astype(int)
    img = np.full((h, w, 3), 235, np.uint8)
    zbuf = np.full((h, w), np.inf)
    col = np.zeros((h, w, 3))
    order = np.argsort(-depth)  # far first, near overwrites
    for dx in (0, 1):
        for dy in (0, 1):
            x = px + dx
            y = py + dy
            ok = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            idx = order[ok[order]]
            col[y[idx], x[idx]] = np.array(color)[None, :] * shade[idx, None]
            zbuf[y[idx], x[idx]] = 0
    mask = zbuf < np.inf
    img[mask] = np.clip(col[mask], 0, 255).astype(np.uint8)
    return img


def main():
    base, out = sys.argv[1], sys.argv[2]
    ppm = float(sys.argv[sys.argv.index('--px-per-m') + 1]) if '--px-per-m' in sys.argv else 300.0
    layout = json.loads(Path(base + '.json').read_text())
    data = np.load(base + '.npz')
    rows = layout['rows']
    cols = len(layout['frames'])
    cw, ch = int(1.5 * ppm), int(1.95 * ppm)
    for view in ('side', 'front'):
        sheet = Image.new('RGB', (cols * cw, len(rows) * ch + 0), (235, 235, 235))
        for r, row in enumerate(rows):
            f = data['faces_%d' % r]
            for k in range(cols):
                v = data['verts_%d_%d' % (r, k)].astype(np.float64)
                cell = Image.fromarray(render_cell(v, f, view, ppm, cw, ch, COLORS[r % 2]))
                d = ImageDraw.Draw(cell)
                d.text((6, 4), '%s t=%.3fs' % (row['label'], row['times'][k]), fill=(20, 20, 20))
                d.line([(0, ch - 12), (cw, ch - 12)], fill=(120, 120, 120))
                sheet.paste(cell, (k * cw, r * ch))
        path = '%s_%s.png' % (out, view)
        sheet.save(path)
        print('WROTE', path, sheet.size)


if __name__ == '__main__':
    main()
