"""Rifle / 2H hand fix 2026-11-09 - Ajax stock length from the game mesh (read-only).
  python -I weapons_fix_mesh.py <mesh.glb>  -> stdout: nodes / skin joints, vertex bounding box (glTF axes and game axes Z-up),
  extent of the rear part along the barrel axis, and the vertex extent behind the pistol-grip region.
glTF export of WolvenKit: Y-up, -Z forward = game +Y forward (game (x, y, z) = (x, -z, y)).
"""
import json
import struct
import sys

import numpy as np

CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def load(path):
    b = open(path, 'rb').read()
    assert b[:4] == b'glTF'
    off = 12
    js, binb = None, None
    while off < len(b):
        ln, typ = struct.unpack_from('<II', b, off)
        chunk = b[off + 8: off + 8 + ln]
        if typ == 0x4E4F534A:
            js = json.loads(chunk)
        elif typ == 0x004E4942:
            binb = chunk
        off += 8 + ln
    return js, binb


def accessor(js, binb, i):
    a = js['accessors'][i]
    bv = js['bufferViews'][a['bufferView']]
    n = NC[a['type']]
    dt = np.dtype(CT[a['componentType']])
    start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = bv.get('byteStride', 0) or n * dt.itemsize
    raw = np.frombuffer(binb, dtype=np.uint8, count=stride * (a['count'] - 1) + n * dt.itemsize, offset=start)
    out = np.lib.stride_tricks.as_strided(raw, shape=(a['count'], n * dt.itemsize), strides=(stride, 1)).copy()
    return out.view(dt).reshape(a['count'], n).astype(float)


def main(path):
    js, binb = load(path)
    print('NODES', [(i, n.get('name'), n.get('translation'), n.get('rotation')) for i, n in enumerate(js['nodes'])][:40])
    for s in js.get('skins', []):
        print('SKIN joints', [js['nodes'][j].get('name') for j in s['joints']])
        if 'inverseBindMatrices' in s:
            ibm = accessor(js, binb, s['inverseBindMatrices']).reshape(-1, 4, 4)
            for j, m in zip(s['joints'], ibm):
                M = np.linalg.inv(m.T)
                print('  bind %-24s pos(gltf) %s' % (js['nodes'][j].get('name'), np.round(M[:3, 3], 4)))
    allv = []
    for m in js['meshes']:
        for p in m['primitives']:
            v = accessor(js, binb, p['attributes']['POSITION'])
            allv.append(v)
            print('MESH %-40s nv %6d bbox gltf min %s max %s' % (m.get('name'), len(v), np.round(v.min(0), 4), np.round(v.max(0), 4)))
    V = np.concatenate(allv)
    G = np.stack([V[:, 0], -V[:, 2], V[:, 1]], 1)        # game axes: x right, y forward, z up
    print('ALL game-axes bbox min %s max %s' % (np.round(G.min(0), 4), np.round(G.max(0), 4)))
    # rear part (behind the origin along -Y): extent and the height band of the stock
    rear = G[G[:, 1] < -0.02]
    print('REAR y min %.4f (stock butt), verts %d' % (rear[:, 1].min(), len(rear)))
    for y0 in (-0.05, -0.1, -0.15, -0.2, -0.25, -0.3, -0.35, -0.4):
        s = G[(G[:, 1] < y0 + 0.025) & (G[:, 1] >= y0 - 0.025)]
        if len(s):
            print('  slice y %.3f: z %.3f..%.3f  x %.3f..%.3f  n %d' % (y0, s[:, 2].min(), s[:, 2].max(), s[:, 0].min(), s[:, 0].max(), len(s)))


if __name__ == '__main__':
    main(sys.argv[1])
