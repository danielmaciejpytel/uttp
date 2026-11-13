import sys, zlib, struct
# Read Float flats of tweakdb.bin (layout probed: type table @0x24, 20-byte entries hash/valueCount/keyCount/offset;
# block @offset: u32 nvalues, float[n], u32 nkeys, (u64 id, u32 idx)[nkeys]).
data = open(sys.argv[1], 'rb').read()
def fnv(s):
    h = 0xcbf29ce484222325
    for c in s.encode():
        h ^= c; h = (h * 0x100000001b3) & 0xffffffffffffffff
    return h
def block(tname):
    i = data.find(struct.pack('<Q', fnv(tname)), 0, 0x400)
    nv, nk, off = struct.unpack_from('<3I', data, i + 8)
    p = off
    n = struct.unpack_from('<I', data, p)[0]; p += 4
    assert n == nv, (n, nv)
    return nv, nk, off, p
nv, nk, off, p = block('Float')
vals = struct.unpack_from('<%df' % nv, data, p); p += 4 * nv
k = struct.unpack_from('<I', data, p)[0]; p += 4
assert k == nk, (k, nk)
idmap = {}
for j in range(nk):
    tid, idx = struct.unpack_from('<QI', data, p + 12 * j)
    idmap[tid & 0xffffffffff] = idx
states = ['climb', 'vault']
params = ['highThreshold', 'midThreshold', 'horizontalDurationHigh', 'verticalDurationHigh', 'horizontalDurationMid',
          'verticalDurationMid', 'horizontalDurationLow', 'verticalDurationLow', 'forwardStep', 'capsuleRadius',
          'capsuleHeight', 'cameraPitchThreshold', 'inputAngleThreshold', 'obstacleEnterAngleThreshold',
          'minDetectionRange', 'maxSpeedNormalizer', 'detectionRange', 'maxExtent', 'minSpeed', 'enterAngleThreshold',
          'stateDuration', 'duration']
for s in states:
    for par in params:
        n = 'playerStateMachineLocomotion.%s.%s' % (s, par)
        tid = (zlib.crc32(n.encode()) & 0xffffffff) | (len(n) << 32)
        if tid in idmap:
            print('%s = %.6g' % (n, vals[idmap[tid]]))
        else:
            print('%s = <absent: script default>' % n)
