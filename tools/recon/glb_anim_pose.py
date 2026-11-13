"""Minimal glTF/GLB animation reader (stdlib only): list animations and sample local bone transforms.

Usage:
  python glb_anim_pose.py <file.glb> list
  python glb_anim_pose.py <file.glb> pose <animation name> [frame] > pose.json
pose prints {bone: {"t": [x,y,z], "r": [x,y,z,w]}} for the given keyframe index (default 0), falling back to the
node rest transform for unanimated channels. Coordinates as stored in the glb (WolvenKit: Y up); read-only.
"""
import json
import struct
import sys

CT = {5126: ('f', 4), 5123: ('H', 2), 5125: ('I', 4), 5121: ('B', 1)}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def load(path):
    data = open(path, 'rb').read()
    assert data[:4] == b'glTF'
    jl, = struct.unpack_from('<I', data, 12)
    js = json.loads(data[20:20 + jl])
    bo = 20 + jl
    bl, = struct.unpack_from('<I', data, bo)
    return js, data[bo + 8:bo + 8 + bl]


def accessor(js, binb, i):
    a = js['accessors'][i]
    bv = js['bufferViews'][a['bufferView']]
    fmt, sz = CT[a['componentType']]
    n = NC[a['type']]
    off = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = bv.get('byteStride', sz * n)
    out = []
    for k in range(a['count']):
        out.append(struct.unpack_from('<%d%s' % (n, fmt), binb, off + k * stride))
    return out


def main():
    js, binb = load(sys.argv[1])
    anims = js.get('animations', [])
    if sys.argv[2] == 'list':
        for a in anims:
            t = accessor(js, binb, a['samplers'][0]['input'])
            print(f"{a.get('name')}\t{t[-1][0]:.2f}s\t{len(t)}")
        return
    name = sys.argv[3]
    frame = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    a = next(x for x in anims if x.get('name') == name)
    nodes = js['nodes']
    pose = {}
    for n in nodes:
        pose[n.get('name')] = {'t': list(n.get('translation', [0, 0, 0])), 'r': list(n.get('rotation', [0, 0, 0, 1]))}
    for ch in a['channels']:
        path = ch['target']['path']
        if path not in ('translation', 'rotation'):
            continue
        nm = nodes[ch['target']['node']].get('name')
        vals = accessor(js, binb, a['samplers'][ch['sampler']]['output'])
        v = vals[min(frame, len(vals) - 1)]
        pose[nm]['t' if path == 'translation' else 'r'] = list(v)
    json.dump(pose, sys.stdout, indent=0)


if __name__ == '__main__':
    main()
