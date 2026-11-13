"""Minimal GLB animation reader/writer used by the vehicle-entry pipeline (numpy only).

read_glb(path) -> (gltf_json, bin_bytes)
Clip helpers sample the WolvenKit-exported skeleton animations (LINEAR translation/scale, LINEAR rotation as slerp).
"""
import json
import struct

import numpy as np

COMP = {5126: np.float32, 5125: np.uint32, 5123: np.uint16, 5121: np.uint8}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def read_glb(path):
    data = open(path, 'rb').read()
    magic, ver, length = struct.unpack_from('<III', data, 0)
    assert magic == 0x46546C67, 'not a GLB'
    off = 12
    gj, binb = None, b''
    while off < length:
        clen, ctype = struct.unpack_from('<II', data, off)
        chunk = data[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            gj = json.loads(chunk.decode('utf-8'))
        elif ctype == 0x004E4942:
            binb = chunk
        off += 8 + clen
    return gj, binb


def write_glb(path, gj, binb):
    js = json.dumps(gj, separators=(',', ':')).encode('utf-8')
    js += b' ' * ((4 - len(js) % 4) % 4)
    binb = bytes(binb) + b'\0' * ((4 - len(binb) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(binb)
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, total))
        f.write(struct.pack('<II', len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack('<II', len(binb), 0x004E4942))
        f.write(binb)


def accessor(gj, binb, idx):
    a = gj['accessors'][idx]
    bv = gj['bufferViews'][a['bufferView']]
    n = NCOMP[a['type']]
    dt = COMP[a['componentType']]
    start = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = bv.get('byteStride')
    item = np.dtype(dt).itemsize * n
    if stride and stride != item:
        raw = np.frombuffer(binb, dtype=np.uint8, count=stride * a['count'], offset=start).reshape(a['count'], stride)
        arr = raw[:, :item].copy().view(dt).reshape(a['count'], n)
    else:
        arr = np.frombuffer(binb, dtype=dt, count=a['count'] * n, offset=start).reshape(a['count'], n).copy()
    return arr if n > 1 else arr[:, 0]


def node_names(gj):
    return [n.get('name', '') for n in gj['nodes']]


def clips(gj):
    return {a.get('name', str(i)): i for i, a in enumerate(gj.get('animations', []))}


def channels(gj, binb, anim_index):
    """{node_name: {'translation': (t, v), 'rotation': (t, v), 'scale': (t, v)}}"""
    a = gj['animations'][anim_index]
    names = node_names(gj)
    out = {}
    for ch in a['channels']:
        s = a['samplers'][ch['sampler']]
        t = accessor(gj, binb, s['input'])
        v = accessor(gj, binb, s['output'])
        node = names[ch['target']['node']]
        out.setdefault(node, {})[ch['target']['path']] = (t, v, s.get('interpolation', 'LINEAR'))
    return out


def qnorm(q):
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def slerp(q0, q1, u):
    q0 = qnorm(q0)
    q1 = qnorm(q1)
    d = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(d < 0, -q1, q1)
    d = np.abs(d)
    lin = d > 0.9995
    th = np.arccos(np.clip(d, -1, 1))
    s = np.sin(th)
    s = np.where(s < 1e-9, 1.0, s)
    a = np.sin((1 - u) * th) / s
    b = np.sin(u * th) / s
    r = np.where(lin, (1 - u) * q0 + u * q1, a * q0 + b * q1)
    return qnorm(r)


def sample(track, times, kind):
    t, v, interp = track
    times = np.asarray(times, dtype=np.float64)
    if len(t) == 1:
        return np.repeat(v[:1], len(times), axis=0).astype(np.float64)
    idx = np.clip(np.searchsorted(t, times, side='right') - 1, 0, len(t) - 2)
    t0 = t[idx]
    t1 = t[idx + 1]
    u = np.clip((times - t0) / np.maximum(t1 - t0, 1e-9), 0, 1)[:, None]
    if interp == 'STEP':
        u = np.zeros_like(u)
    if kind == 'rotation':
        return slerp(v[idx].astype(np.float64), v[idx + 1].astype(np.float64), u)
    return v[idx] * (1 - u) + v[idx + 1] * u


# quaternion math, glTF order (x, y, z, w)
def qmul(a, b):
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz], axis=-1)


def qrot(q, v):
    qv = np.concatenate([v, np.zeros(v.shape[:-1] + (1,))], axis=-1)
    qc = q * np.array([-1, -1, -1, 1.0])
    return qmul(qmul(q, qv), qc)[..., :3]


def qyaw_deg_about(q, axis):
    """Signed rotation angle (deg) of q about the given unit axis (twist part)."""
    v = q[..., :3]
    proj = np.sum(v * axis, axis=-1)
    return np.degrees(2 * np.arctan2(proj, q[..., 3]))


def skin_joints(gj):
    names = node_names(gj)
    sk = gj['skins'][0]
    return [names[j] for j in sk['joints']], sk


def parents(gj):
    p = {}
    for i, n in enumerate(gj['nodes']):
        for c in n.get('children', []):
            p[c] = i
    return p
