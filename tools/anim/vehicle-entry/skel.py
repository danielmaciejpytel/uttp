"""Skeleton + clip sampling on top of glbanim (model space = glTF armature space, Y up, character faces -Z... as exported).

Quaternions are (x, y, z, w). Rotation matrices are used for mirroring.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402


def q2m(q):
    q = np.asarray(q, dtype=np.float64)
    x, y, z, w = np.moveaxis(q, -1, 0)
    m = np.empty(q.shape[:-1] + (3, 3))
    m[..., 0, 0] = 1 - 2 * (y * y + z * z)
    m[..., 0, 1] = 2 * (x * y - z * w)
    m[..., 0, 2] = 2 * (x * z + y * w)
    m[..., 1, 0] = 2 * (x * y + z * w)
    m[..., 1, 1] = 1 - 2 * (x * x + z * z)
    m[..., 1, 2] = 2 * (y * z - x * w)
    m[..., 2, 0] = 2 * (x * z - y * w)
    m[..., 2, 1] = 2 * (y * z + x * w)
    m[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return m


def m2q(m):
    m = np.asarray(m, dtype=np.float64)
    shp = m.shape[:-2]
    m = m.reshape(-1, 3, 3)
    q = np.empty((m.shape[0], 4))
    for i, r in enumerate(m):
        t = r[0, 0] + r[1, 1] + r[2, 2]
        if t > 0:
            s = np.sqrt(t + 1.0) * 2
            q[i] = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            q[i] = [0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif r[1, 1] > r[2, 2]:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            q[i] = [(r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            q[i] = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return q.reshape(shp + (4,))


def qcontinuous(q):
    """Make a quaternion time series sign-continuous (axis 0 = time)."""
    q = q.copy()
    for i in range(1, q.shape[0]):
        flip = np.sum(q[i] * q[i - 1], axis=-1) < 0
        q[i][flip] *= -1
    return q


class Skeleton:
    def __init__(self, gj, binb, path=None):
        self.gj, self.binb, self.path = gj, binb, path
        names = G.node_names(gj)
        joints, sk = G.skin_joints(gj)
        node_of = {names[j]: j for j in sk['joints']}
        self.names = joints
        self.index = {n: i for i, n in enumerate(joints)}
        par = G.parents(gj)
        self.parent = []
        for n in joints:
            p = par.get(node_of[n])
            pn = names[p] if p is not None else None
            self.parent.append(self.index.get(pn, -1))
        self.rest_t = np.array([gj['nodes'][node_of[n]].get('translation', [0, 0, 0]) for n in joints], dtype=np.float64)
        self.rest_r = np.array([gj['nodes'][node_of[n]].get('rotation', [0, 0, 0, 1]) for n in joints], dtype=np.float64)
        self.rest_s = np.array([gj['nodes'][node_of[n]].get('scale', [1, 1, 1]) for n in joints], dtype=np.float64)
        self.node_of = node_of
        # topological order check (parents before children)
        for i, p in enumerate(self.parent):
            assert p < i, 'skin joints not in topological order: %s' % joints[i]

    @classmethod
    def from_glb(cls, path):
        gj, binb = G.read_glb(str(path))
        return cls(gj, binb, path)

    def clip(self, name):
        return G.clips(self.gj)[name]

    def clip_tracks(self, name):
        return G.channels(self.gj, self.binb, self.clip(name))

    def sample_local(self, anim_index, times, rm_time_fix=True):
        ch = G.channels(self.gj, self.binb, anim_index)
        # WolvenKit 9.0.1 exports the root-motion (Root) track with key times = u16/65535, i.e. NORMALISED time
        # (0..1), while the bone tracks use seconds. The RED key time is normalised to the clip duration (the vanilla
        # spline ends with keys at 65534/65535 = end of clip). Rescale the Root track to seconds.
        ex = self.gj['animations'][anim_index].get('extras', {})
        if rm_time_fix and 'Root' in ch and ex.get('rootMotionType') not in (None, 'None', 'Unknown') \
                and not ex.get('utppRootTimesInSeconds'):
            dur = max(float(tr[0][-1]) for b, d in ch.items() if b != 'Root' for tr in d.values())
            rt = ch['Root']
            for k in list(rt.keys()):
                t, v, it = rt[k]
                if float(t[-1]) <= 1.0001 and dur > 1.0001:
                    rt[k] = (t * dur, v, it)
        n = len(self.names)
        T = len(times)
        lt = np.repeat(self.rest_t[None], T, 0).copy()
        lr = np.repeat(self.rest_r[None], T, 0).copy()
        for b, d in ch.items():
            if b not in self.index:
                continue
            i = self.index[b]
            if 'translation' in d:
                lt[:, i] = G.sample(d['translation'], times, 'translation')
            if 'rotation' in d:
                lr[:, i] = G.sample(d['rotation'], times, 'rotation')
        return lt, lr

    def world(self, lt, lr):
        """lt (T,N,3), lr (T,N,4) -> world translations (T,N,3), rotation matrices (T,N,3,3). Scale ignored (all 1)."""
        T, N = lt.shape[:2]
        Wt = np.zeros((T, N, 3))
        Wr = np.zeros((T, N, 3, 3))
        R = q2m(lr)
        for i in range(N):
            p = self.parent[i]
            if p < 0:
                Wr[:, i] = R[:, i]
                Wt[:, i] = lt[:, i]
            else:
                Wr[:, i] = Wr[:, p] @ R[:, i]
                Wt[:, i] = Wt[:, p] + np.einsum('tij,tj->ti', Wr[:, p], lt[:, i])
        return Wt, Wr

    def rest_world(self):
        Wt, Wr = self.world(self.rest_t[None], self.rest_r[None])
        return Wt[0], Wr[0]

    def local_from_world(self, Wt, Wr):
        T, N = Wt.shape[:2]
        lt = np.zeros((T, N, 3))
        lr = np.zeros((T, N, 4))
        for i in range(N):
            p = self.parent[i]
            if p < 0:
                lt[:, i] = Wt[:, i]
                lr[:, i] = m2q(Wr[:, i])
            else:
                inv = np.transpose(Wr[:, p], (0, 2, 1))
                lt[:, i] = np.einsum('tij,tj->ti', inv, Wt[:, i] - Wt[:, p])
                lr[:, i] = m2q(inv @ Wr[:, i])
        return lt, qcontinuous(lr)


def mirror_name(n):
    for a, b in (('Left', 'Right'), ('left', 'right'), ('_l_', '_r_'), ('_L_', '_R_')):
        if a in n:
            return n.replace(a, b)
        if b in n:
            return n.replace(b, a)
    if n.endswith('_l'):
        return n[:-2] + '_r'
    if n.endswith('_r'):
        return n[:-2] + '_l'
    return n
