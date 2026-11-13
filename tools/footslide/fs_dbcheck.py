"""python -I fs_dbcheck.py <woman|man> <db.utppmm> <clipdir> <clip name> [...]: reads the POSE section of a baked .utppmm v2 (30 Hz clip frames, game-local
transforms incl. Trajectory) for the named clips, compares it with the clip json, and re-measures the contact-point slide on the BAKED poses with the same
code as fs_lib.measure (contact curves from the clip json). Shows what the bake adds on top of the clip (quantisation only) and that a pin reached the DB."""
import json
import os
import struct
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fs_lib as L  # noqa: E402

HEADER_FMT = "<8sIIQQI" + "I" * 11 + "f" + "IIIIIII" + "QQ"


class Db:
    def __init__(self, path):
        self.b = open(path, 'rb').read()
        b = self.b
        h = struct.unpack_from(HEADER_FMT, b, 0)
        assert h[0] == b'UTPPMMDB' and h[1] == 2
        self.nb, self.nf, self.nc, self.nsec = h[7], h[6], h[8], h[18]
        self.secs = {}
        for i in range(self.nsec):
            sid, cnt, esz, _, off, size = struct.unpack_from("<IIIIQQ", b, 128 + 32 * i)
            self.secs[struct.pack("<I", sid).decode('ascii')] = (cnt, esz, off, size)
        so = self.secs['STRS'][2]
        self.clips = {}
        co = self.secs['CLIP'][2]
        for c in range(self.nc):
            name_off, first, n = struct.unpack_from("<III", b, co + 32 * c)
            e = b.index(b"\0", so + name_off)
            self.clips[b[so + name_off:e].decode('utf-8')] = (first, n)

    def pose(self, name):
        first, n = self.clips[name]
        off = self.secs['POSE'][2] + first * self.nb * 28
        a = np.frombuffer(self.b, dtype='<f4', count=n * self.nb * 7, offset=off).reshape(n, self.nb, 7).astype(np.float64)
        return a


def clip_from_pose(ref, pose):
    c = L.Clip.__new__(L.Clip)
    c.__dict__.update(ref.__dict__)
    c.N = pose.shape[0]
    c.P = pose[..., 0:3].copy()
    c.Q = pose[..., 3:7].copy()
    return c


if __name__ == '__main__':
    g, dbp, d = sys.argv[1:4]
    db = Db(dbp)
    for n in sys.argv[4:]:
        ref = L.Clip(os.path.join(d, n + '.utppclip.json'), g)
        pose = db.pose(n)
        c2 = clip_from_pose(ref, pose)
        dp = float(np.abs(pose[..., 0:3] - ref.P).max())
        dq = float(np.abs(np.abs(np.sum(pose[..., 3:7] * ref.Q, -1)) - 1).max())
        ma, mb = L.measure(ref), L.measure(c2)
        print('%-38s frames %d  POSE vs json: max |dP| %.2e m, max (1-|q.q|) %.1e | slide worst plant %.2f -> %.2f cm, mean %.3f -> %.3f cm/frame'
              % (n.replace('M_Neutral_', ''), ref.N, dp, dq, max(ma['Left']['worst_seg'], ma['Right']['worst_seg']),
                 max(mb['Left']['worst_seg'], mb['Right']['worst_seg']), (ma['Left']['mean'] + ma['Right']['mean']) / 2, (mb['Left']['mean'] + mb['Right']['mean']) / 2))
