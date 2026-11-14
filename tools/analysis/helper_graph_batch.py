"""Frame-batched evaluator of the player body deformer graph (skin helper bones), for offline reports.

BatchEvaluator evaluates the same graph with the same arithmetic as evaluate_body_deformer_graph.Evaluator (the scalar offline
reference), but for N frames at once: every bone matrix is an [N, 4, 4] array and every graph node is one vectorised step. A scalar
evaluation costs about 20 ms per pose; the batched form handles a 5-minute recording (about 18000 frames) in seconds.

  ev = BatchEvaluator('man')
  models, bad = ev.run(core_locals, want=['l_SHL_0_JNT', 'Spine3'])     # core_locals: {bone name: [N,4,4] local matrices}

Inputs are LOCAL matrices (translation in metres, rotation * scale) of every physical core bone of the rig (ev.core_names); helper
bones start from their reference locals, exactly like the scalar evaluator. The result holds the MODEL-space matrices of the requested
bones. `bad` is a boolean [N] mask of frames whose evaluation the scalar evaluator would have refused (non-finite input, singular
matrix, antipodal swing, non-rigid or gimbal rotation-channel input, degenerate unit vector in a twist or a scale channel): their
values are meaningless and callers must exclude or count them (never drop them silently). A degenerate aim constraint skips only its
own write for that frame, as in the scalar evaluator.

Run as a script for a self-check against the scalar evaluator on perturbed poses:  python -I helper_graph_batch.py --check [--body man]
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import evaluate_body_deformer_graph as E  # noqa: E402

SKIPPED_TYPES = frozenset(['Root', 'Output', 'ReferencePoseTerminator', 'SharedMetaPose', 'PoseLsToMs', 'PoseMsToLs', 'StackTransformsShrinker',
                           'StackTracksExtender', 'StackTracksShrinker', 'ConditionalSegmentBegin', 'ConditionalSegmentEnd',
                           'SkipPerformanceModeBegin', 'SkipPerformanceModeEnd'])
EYE = np.eye(4)


# --------------------------------------------------------------------------------------------------------------------- vector maths
def _norm(v):
    return np.sqrt(np.sum(v * v, axis=-1))


def _unit(v, bad):
    """Unit vectors of v [...,3]; frames with a (near) zero vector are flagged in bad and get a harmless value."""
    n = _norm(v)
    small = n < 1e-10
    bad |= small.reshape(bad.shape[0], -1).any(axis=1) if small.ndim > 1 else small
    return v / np.where(small, 1.0, n)[..., None]


def _rot_from_quat(q):
    """[...,4] xyzw -> [...,3,3] (normalised first, like replay_body_skin.trs)."""
    q = q / _norm(q)[..., None]
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    r = np.empty(q.shape[:-1] + (3, 3))
    r[..., 0, 0] = 1 - 2 * (y * y + z * z); r[..., 0, 1] = 2 * (x * y - z * w); r[..., 0, 2] = 2 * (x * z + y * w)
    r[..., 1, 0] = 2 * (x * y + z * w); r[..., 1, 1] = 1 - 2 * (x * x + z * z); r[..., 1, 2] = 2 * (y * z - x * w)
    r[..., 2, 0] = 2 * (x * z - y * w); r[..., 2, 1] = 2 * (y * z + x * w); r[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return r


def _quat_from_rot(r):
    """[...,3,3] -> [...,4] xyzw: dominant eigenvector of the 4x4 symmetric matrix of evaluate_body_deformer_graph.quat."""
    a, b, c = r[..., 0, 0], r[..., 0, 1], r[..., 0, 2]
    d, e, f = r[..., 1, 0], r[..., 1, 1], r[..., 1, 2]
    g, h, i = r[..., 2, 0], r[..., 2, 1], r[..., 2, 2]
    k = np.empty(r.shape[:-2] + (4, 4))
    k[..., 0, 0] = a - e - i; k[..., 0, 1] = b + d; k[..., 0, 2] = c + g; k[..., 0, 3] = h - f
    k[..., 1, 0] = b + d; k[..., 1, 1] = e - a - i; k[..., 1, 2] = f + h; k[..., 1, 3] = c - g
    k[..., 2, 0] = c + g; k[..., 2, 1] = f + h; k[..., 2, 2] = i - a - e; k[..., 2, 3] = d - b
    k[..., 3, 0] = h - f; k[..., 3, 1] = c - g; k[..., 3, 2] = d - b; k[..., 3, 3] = a + e + i
    k /= 3.0
    _, v = np.linalg.eigh(k)
    return v[..., :, -1]


def _slerp(a, b, w):
    dot = np.sum(a * b, axis=-1)
    b = np.where((dot < 0)[..., None], -b, b)
    c = np.clip(np.sum(a * b, axis=-1), -1, 1)
    t = np.arccos(c)
    st = np.sin(t)
    near = c > .9995
    lin = (1 - w) * a + w * b
    lin = lin / _norm(lin)[..., None]
    wide = (np.sin((1 - w) * t) / np.where(near, 1.0, st))[..., None] * a + (np.sin(w * t) / np.where(near, 1.0, st))[..., None] * b
    return np.where(near[..., None], lin, wide)


def _safe_inv(m, bad):
    """Batched inverse of [N,4,4]; singular or non-finite frames are flagged and get the identity."""
    det = np.linalg.det(m)
    sing = ~np.isfinite(det) | (np.abs(det) < 1e-18)
    if sing.any():
        bad |= sing
        m = np.where(sing[:, None, None], EYE, m)
    return np.linalg.inv(m)


def _full(x, n):
    return x if x.ndim == 3 and x.shape[0] == n else np.broadcast_to(x, (n, 4, 4))


def _mm(a, b, n):
    if a.ndim == 2 and b.ndim == 2:
        return a @ b                                                    # constant (frame independent) product
    return np.matmul(_full(a, n), _full(b, n))


def _replace_rotation_channels(local, channels, bad):
    """Batched deformer_rotation_channels.replace_rotation_channels (channels: {'RotX'|'RotY'|'RotZ': degrees [N]})."""
    out = local.copy()
    scale = _norm(np.swapaxes(out[:, :3, :3], 1, 2))              # column norms [N,3]
    sing = scale.min(axis=1) < 1e-9
    r = out[:, :3, :3] / np.where(scale < 1e-9, 1.0, scale)[:, None, :]
    rtr = np.matmul(np.swapaxes(r, 1, 2), r) - np.eye(3)
    nonrigid = (np.abs(rtr).max(axis=(1, 2)) > 1e-5) | (np.linalg.det(r) < .99999)
    pitch = np.arcsin(np.clip(r[:, 2, 1], -1, 1))
    gimbal = np.abs(np.cos(pitch)) < 1e-7
    bad |= sing | nonrigid | gimbal | ~np.isfinite(out).all(axis=(1, 2))
    yaw = np.arctan2(-r[:, 0, 1], r[:, 1, 1])
    roll = np.arctan2(-r[:, 2, 0], r[:, 2, 2])
    values = {'RotX': pitch, 'RotY': roll, 'RotZ': yaw}
    for channel, value in channels.items():
        values[channel] = np.deg2rad(value)
    p, b, y = values['RotX'], values['RotY'], values['RotZ']
    cp, sp, cb, sb, cy, sy = np.cos(p), np.sin(p), np.cos(b), np.sin(b), np.cos(y), np.sin(y)
    n = len(p)
    zero, one = np.zeros(n), np.ones(n)
    rx = np.stack([np.stack([one, zero, zero], -1), np.stack([zero, cp, -sp], -1), np.stack([zero, sp, cp], -1)], 1)
    ry = np.stack([np.stack([cb, zero, sb], -1), np.stack([zero, one, zero], -1), np.stack([-sb, zero, cb], -1)], 1)
    rz = np.stack([np.stack([cy, -sy, zero], -1), np.stack([sy, cy, zero], -1), np.stack([zero, zero, one], -1)], 1)
    out[:, :3, :3] = np.matmul(np.matmul(rz, rx), ry) * scale[:, None, :]
    return out


def _swing(a, b, bad):
    """Batched swing(): rotation taking a to b; [N,3,3]."""
    a = _unit(a, bad)
    b = _unit(b, bad)
    v = np.cross(a, b)
    c = np.sum(a * b, axis=-1)
    bad |= c < -.999999
    k = np.zeros(v.shape[:-1] + (3, 3))
    k[..., 0, 1], k[..., 0, 2] = -v[..., 2], v[..., 1]
    k[..., 1, 0], k[..., 1, 2] = v[..., 2], -v[..., 0]
    k[..., 2, 0], k[..., 2, 1] = -v[..., 1], v[..., 0]
    return np.eye(3) + k + np.matmul(k, k) / (1 + np.where(c < -.999999, 0.0, c))[..., None, None]


def _twist_angle(A, B, axis, bad):
    side = np.array([0., 0., 1.]) if abs(axis[2]) < .5 else np.array([1., 0., 0.])
    af = np.einsum('nij,j->ni', A, axis)
    aside = np.einsum('nij,j->ni', A, side)
    sw = _swing(np.einsum('nij,j->ni', B, axis), af, bad)
    bs = np.einsum('nij,njk,k->ni', sw, B, side)
    return np.arctan2(np.sum(bs * np.cross(af, aside), axis=-1), np.sum(aside * bs, axis=-1))


def _basis(a, b, bad):
    """Batched basis(): columns (a, b orthogonalised, a x b) of [N,3] vectors."""
    a = _unit(a, bad)
    b = _unit(b - a * np.sum(a * b, axis=-1, keepdims=True), bad)
    return np.stack([a, b, np.cross(a, b)], axis=-1)


# --------------------------------------------------------------------------------------------------------------------- evaluator
class BatchEvaluator:
    """Compiles the graph of one body once; run() evaluates N frames at a time."""

    def __init__(self, body, evaluator=None):
        ev = evaluator or E.Evaluator(allow_rotation_bounce=True, body=body)
        self.ev, self.body = ev, body
        self.names = list(ev.names)
        self.core_names = set(ev.coreNames)
        self.rig_core = [n for n in self.names if n in self.core_names]           # core bones the rig has (all must be supplied)
        self.helper_names = [n for n in self.names if n not in self.core_names]
        self._ops = []
        self._compile()

    # ------------------------------------------------------------------ compile
    def _compile(self):
        ev = self.ev
        names, parents = self.names, ev.parents
        parent_names = {n: (names[parents[i]] if parents[i] >= 0 else None) for i, n in enumerate(names)}
        known = set(names)
        ops = self._ops

        def nm(x):
            return x['name']['$value']

        for h in sorted(ev.nodes, reverse=True):
            if h < 1:
                break
            node = ev.nodes[h]
            t = node['$type'].removeprefix('animAnimNode_')
            if 'SourceChannel' in t or t in SKIPPED_TYPES:
                continue
            if t == 'StackTransformsExtender':
                if set(node['snapMethods']) != {'NoSnapping'} or any(node['snapToReferenceValues']) or any(node['offsetToReferenceValues']):
                    raise ValueError('Unsupported extender snapping')
                if any(x['name']['$value'] != 'None' for x in node['offsetSpaceBones']):
                    raise ValueError('Unsupported extender offset space')
                if any(np.max(np.abs(E.tm(x) - np.eye(4))) > 1e-7 for x in node['offsets']):
                    raise ValueError('Unsupported extender offsets')
                pending = list(node['transformInfos'])
                while pending:
                    done = []
                    for e in pending:
                        n, p = nm(e), e['parentName']['$value']
                        parent_names[n] = p
                        if p in known or p == 'None':
                            ops.append(('init', n, p if p in known else None, E.tm(e['referenceTransformLs'])))
                            known.add(n)
                            done.append(e)
                    if not done:
                        raise ValueError('Unresolved logical parents')
                    pending = [e for e in pending if e not in done]
                continue
            if node.get('weight', 1) != 1:
                raise ValueError('Unsupported fractional constraint weight')
            if t == 'TwistConstraint':
                front = E.axis(node['frontAxis'])
                outs = []
                for e in node['outputs']:
                    n = nm(e['twistedTransform'])
                    outs.append((n, parent_names[n], E.axis(e['twistAxis']), e['positiveScale'], e['negativeScale']))
                ops.append(('twist', nm(node['transformA']), nm(node['transformB']), front, outs))
            elif t == 'SetBoneTransform':
                entries = []
                for e in node['entries']:
                    if e['snapToReference'] or e['offsetToReference']:
                        raise ValueError('Reference snapping unsupported')
                    method = e['setMethod']
                    if method not in ('WholeTransform', 'TranslationOnly', 'RotationOnly', 'NoSnapping'):
                        raise ValueError(method)
                    entries.append((nm(e['transformToChange']), nm(e['sourceBone']), nm(e['offsetSpaceBone']), method, E.tm(e['offset'])))
                ops.append(('set', entries))
            elif t == 'PointConstraint':
                n = nm(node['transformIndex'])
                ops.append(('point', n, [(nm(x['transform']), float(w)) for w, x in zip(node['preprocessedWeights'], node['inputWeightedTransforms'])]))
            elif t == 'OrientConstraint':
                n = nm(node['transformIndex'])
                entries, weights = node['inputWeightedTransforms'], node['preprocessedWeights']
                ops.append(('orient', n, nm(entries[0]['transform']), [(nm(x['transform']), float(w)) for w, x in zip(weights, entries[1:])]))
            elif t.startswith('AimConstraint_'):
                n = nm(node['transformIndex'])
                fixed_bad = False
                try:
                    constant = E.basis(E.vec(node['forwardAxisLS']), E.vec(node['upAxisLS'])).T
                except ValueError:
                    constant, fixed_bad = None, True
                rotation_up = t.endswith('ObjectRotationUp')
                ops.append(('aim', n, nm(node['targetTransform']), nm(node['upTransform']),
                            E.vec(node['upTransformVector']) if rotation_up else np.zeros(3), rotation_up, constant, fixed_bad))
            elif t == 'SimpleSpline':
                n = nm(node['constrainedTransform'])
                if node['progressMode'] == 'FloatTrack':
                    ops.append(('spline', n, nm(node['startTransform']), nm(node['middleTransform']), nm(node['endTransform']), None, nm(node['progressTrack'])))
                elif node['progressMode'] == 'Static':
                    ops.append(('spline', n, nm(node['startTransform']), nm(node['middleTransform']), nm(node['endTransform']), float(node['defaultProgress']), None))
                else:
                    raise ValueError('Unsupported spline progress mode')
            elif t == 'SimpleBounce':
                if nm(node['outputDriverTrack']) != 'None':
                    raise ValueError('Unsupported bounce driver track')
                outs = []
                for e in node['transformOutputs']:
                    chans = []
                    for c in e['channelEntries']:
                        channel = c['transformChannel']
                        if channel[:3] not in ('Pos', 'Sca', 'Rot'):
                            raise ValueError('Rotation bounce unsupported ' + channel)
                        chans.append((channel, 'XYZ'.index(channel[-1]), float(c['multiplier'])))
                    outs.append((nm(e['targetTransform']), nm(e['parentTransform']), chans))
                ops.append(('bounce', nm(node['startTransform']), nm(node['endTransform']), float(node['multiplier']), float(node['negativeMultiplier']),
                            float(node['delay']), float(node['smoothStep']), float(node['offset']),
                            [(nm(e['targetTrack']), float(e['multiplier'])) for e in node['trackOutputs']], outs))
            elif t == 'TranslationLimit':
                n = nm(node['constrainedTransform'])
                pn = nm(node['parentTransform'])
                lims = []
                for a in range(3):
                    c = node['limitOn' + 'XYZ'[a] + 'Axis']
                    lims.append((c['useMin'], c['min'], c['useMax'], c['max']))
                ops.append(('limit', n, parent_names[n] if pn == 'None' else pn, lims))
            else:
                raise ValueError('Unsupported ' + t)

    # ------------------------------------------------------------------ run
    def run(self, core_locals, want=None):
        """core_locals {core bone: [N,4,4] local matrices}; returns ({bone: [N,4,4] model matrix}, bad [N] bool).
        want = bone names to return (default: every rig bone and every logical node the graph created)."""
        n = None
        for v in core_locals.values():
            n = v.shape[0]
            break
        if n is None:
            raise ValueError('no core pose given')
        missing = [b for b in self.rig_core if b not in core_locals]
        if missing:
            raise ValueError('Missing physical core input: ' + ', '.join(sorted(missing)))
        bad = np.zeros(n, dtype=bool)
        locals_ = {}
        for b in self.rig_core:
            m = core_locals[b]
            if m.shape != (n, 4, 4):
                raise ValueError('core pose %s has shape %s, expected %s' % (b, m.shape, (n, 4, 4)))
            fin = np.isfinite(m).all(axis=(1, 2))
            if not fin.all():
                bad |= ~fin
                m = np.where(fin[:, None, None], m, self.ev.refLocal[b])
            locals_[b] = m
        M = {}
        for i, name in enumerate(self.names):
            p = self.ev.parents[i]
            local = locals_.get(name)
            if local is None:
                local = self.ev.refLocal[name]
            M[name] = local if p < 0 else _mm(M[self.names[p]], local, n)
        tracks = {}
        for op in self._ops:
            getattr(self, '_op_' + op[0])(op, M, tracks, bad, n)
        if want is None:
            want = list(M)
        return {w: np.array(_full(M[w], n)) for w in want}, bad

    # ------------------------------------------------------------------ ops (each mirrors the matching branch of Evaluator.run)
    def _op_init(self, op, M, tracks, bad, n):
        _, name, parent, local = op
        M[name] = local if parent is None else _mm(M[parent], local, n)

    def _op_twist(self, op, M, tracks, bad, n):
        _, an, bn, front, outs = op
        A = _full(M[an], n)[:, :3, :3]
        B = _full(M[bn], n)[:, :3, :3]
        ang = _twist_angle(A, B, front, bad)
        for name, parent, taxis, pos_scale, neg_scale in outs:
            theta = ang * np.where(ang < 0, neg_scale, pos_scale)
            q = np.concatenate([taxis[None, :] * np.sin(theta / 2)[:, None], np.cos(theta / 2)[:, None]], axis=1)
            new = np.array(_full(M[name], n))
            new[:, :3, :3] = np.matmul(_full(M[parent], n)[:, :3, :3], _rot_from_quat(q))
            M[name] = new

    def _op_set(self, op, M, tracks, bad, n):
        for name, source, offset_space, method, off in op[1]:
            new = np.array(_full(M[name], n))
            src = _full(M[source], n)
            if method == 'WholeTransform':
                new = np.array(src)
            elif method == 'TranslationOnly':
                new[:, :3, 3] = src[:, :3, 3]
            elif method == 'RotationOnly':
                new[:, :3, :3] = src[:, :3, :3]
            M[name] = new
            if offset_space != 'None':
                P = _full(M[offset_space], n)
                L = np.matmul(_safe_inv(np.array(P), bad), new)
                L[:, :3, 3] += off[:3, 3]
                L[:, :3, :3] = np.matmul(off[:3, :3], L[:, :3, :3])
                M[name] = np.matmul(P, L)

    def _op_point(self, op, M, tracks, bad, n):
        _, name, inputs = op
        t = sum(w * _full(M[s], n)[:, :3, 3] for s, w in inputs)
        new = np.array(_full(M[name], n))
        new[:, :3, 3] = t
        M[name] = new

    def _op_orient(self, op, M, tracks, bad, n):
        _, name, first, rest = op
        q = _quat_from_rot(_full(M[first], n)[:, :3, :3])
        for s, w in rest:
            q = _slerp(q, _quat_from_rot(_full(M[s], n)[:, :3, :3]), w)
        new = np.array(_full(M[name], n))
        new[:, :3, :3] = _rot_from_quat(q)
        M[name] = new

    def _op_aim(self, op, M, tracks, bad, n):
        _, name, target, up, upvec, rotation_up, constant, fixed_bad = op
        if fixed_bad:
            return
        cur = _full(M[name], n)
        P = cur[:, :3, 3]
        d = _full(M[target], n)[:, :3, 3] - P
        upm = _full(M[up], n)
        u = np.einsum('nij,j->ni', upm[:, :3, :3], upvec) if rotation_up else upm[:, :3, 3] - P
        local_bad = np.zeros(n, dtype=bool)
        R = np.matmul(_basis(d, u, local_bad), constant)
        new = np.array(cur)
        new[:, :3, :3] = np.where(local_bad[:, None, None], cur[:, :3, :3], R)
        M[name] = new

    def _op_spline(self, op, M, tracks, bad, n):
        _, name, s, m, e, static, track = op
        if static is None:
            if track not in tracks:
                raise ValueError('Unproduced spline track ' + track)
            u = np.clip(tracks[track], 0, 1)[:, None]
        else:
            u = static
        t = (1 - u) ** 2 * _full(M[s], n)[:, :3, 3] + 2 * u * (1 - u) * _full(M[m], n)[:, :3, 3] + u * u * _full(M[e], n)[:, :3, 3]
        new = np.array(_full(M[name], n))
        new[:, :3, 3] = t
        M[name] = new

    def _op_bounce(self, op, M, tracks, bad, n):
        _, start, end, mult, neg_mult, delay, smooth, offset, track_outs, outs = op
        val = np.matmul(_safe_inv(np.array(_full(M[start], n)), bad), _full(M[end], n))[:, 0, 3]
        v = np.maximum(0, np.abs(val) - delay)
        mult_v = np.where(val < 0, neg_mult, mult)
        shaped = np.where(v < smooth, v * v / smooth, v) if smooth else v
        out = shaped * mult_v + offset
        for track, m in track_outs:
            value = out * m
            bad |= ~np.isfinite(value)
            tracks[track] = value
        for name, parent, chans in outs:
            P = np.array(_full(M[parent], n))
            L = np.matmul(_safe_inv(np.array(P), bad), _full(M[name], n))
            rotations = {}
            for channel, a, m in chans:
                value = m * out
                if channel.startswith('Pos'):
                    L[:, a, 3] = value
                elif channel.startswith('Scale'):
                    L[:, :3, a] = _unit(L[:, :3, a], bad) * value[:, None]
                else:
                    rotations[channel] = value
            if rotations:
                L = _replace_rotation_channels(L, rotations, bad)
            M[name] = np.matmul(P, L)

    def _op_limit(self, op, M, tracks, bad, n):
        _, name, parent, lims = op
        P = np.array(_full(M[parent], n))
        L = np.matmul(_safe_inv(np.array(P), bad), _full(M[name], n))
        for a, (use_min, mn, use_max, mx) in enumerate(lims):
            if use_min:
                L[:, a, 3] = np.maximum(L[:, a, 3], mn)
            if use_max:
                L[:, a, 3] = np.minimum(L[:, a, 3], mx)
        M[name] = np.matmul(P, L)


# --------------------------------------------------------------------------------------------------------------------- helpers for callers
def quat_matrices(values):
    """Local TRS rows [..., 12] (translation xyz, unused, quaternion xyzw, scale xyz, unused; the recording layout) -> [...,4,4]
    float64, the quaternion renormalised (same convention as replay_body_skin.trs)."""
    v = np.asarray(values, dtype=np.float64)
    m = np.zeros(v.shape[:-1] + (4, 4))
    m[..., :3, :3] = _rot_from_quat(v[..., 4:8]) * v[..., None, 8:11]
    m[..., :3, 3] = v[..., 0:3]
    m[..., 3, 3] = 1.0
    return m


def rotation_error_deg(a, b):
    """Rotation angle in degrees between the rotation parts of [...,4,4] matrices a and b (columns normalised first, scale ignored)."""
    ra = a[..., :3, :3] / np.maximum(_norm(np.swapaxes(a[..., :3, :3], -1, -2)), 1e-12)[..., None, :]
    rb = b[..., :3, :3] / np.maximum(_norm(np.swapaxes(b[..., :3, :3], -1, -2)), 1e-12)[..., None, :]
    r = np.matmul(ra, np.swapaxes(rb, -1, -2))
    cos = (r[..., 0, 0] + r[..., 1, 1] + r[..., 2, 2] - 1) * 0.5
    sin = 0.5 * np.sqrt((r[..., 2, 1] - r[..., 1, 2]) ** 2 + (r[..., 0, 2] - r[..., 2, 0]) ** 2 + (r[..., 1, 0] - r[..., 0, 1]) ** 2)
    return np.degrees(np.arctan2(sin, cos))


# --------------------------------------------------------------------------------------------------------------------- self-check
def perturbed_core_locals(ev, n, seed, max_deg=30.0, max_cm=2.0):
    """n random poses: every core local = reference local composed with a random rotation (<= max_deg) and shift (<= max_cm)."""
    rng = np.random.default_rng(seed)
    out = {}
    for b in sorted(ev.coreNames):
        if b not in ev.refLocal:
            continue
        ang = np.radians(rng.uniform(-max_deg, max_deg, (n, 3)))
        r = _rot_from_quat(np.concatenate([ang / 2, np.ones((n, 1))], axis=1))          # small rotation vector; normalised inside
        m = np.broadcast_to(ev.refLocal[b], (n, 4, 4)).copy()
        m[:, :3, :3] = np.matmul(m[:, :3, :3], r)
        m[:, :3, 3] += rng.uniform(-max_cm, max_cm, (n, 3)) / 100.0
        out[b] = m
    return out


def compare_with_scalar(body, n=40, seed=1):
    """Max position (cm) and rotation (deg) difference between the batched and the scalar evaluator over every rig bone."""
    ev = E.Evaluator(allow_rotation_bounce=True, body=body)
    batch = BatchEvaluator(body, ev)
    locs = perturbed_core_locals(ev, n, seed)
    models, bad = batch.run(locs)
    worst_p, worst_r, used, scalar_failed = 0.0, 0.0, 0, 0
    for k in range(n):
        try:
            ref = ev.run({b: m[k] for b, m in locs.items() if b in ev.coreNames})
        except Exception:                                            # noqa: BLE001 - the scalar evaluator refuses this pose
            scalar_failed += 1
            if not bad[k]:
                raise AssertionError('frame %d: scalar evaluator refused the pose but the batch did not flag it' % k)
            continue
        if bad[k]:
            raise AssertionError('frame %d flagged bad by the batch evaluator only' % k)
        used += 1
        for name, mat in ref.items():
            if name not in models:
                continue
            got = models[name][k]
            worst_p = max(worst_p, float(np.linalg.norm(got[:3, 3] - mat[:3, 3]) * 100.0))
            worst_r = max(worst_r, float(rotation_error_deg(got, mat)))
    return dict(body=body, frames=n, compared=used, scalar_refused=scalar_failed, max_pos_cm=worst_p, max_rot_deg=worst_r)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--check', action='store_true', help='compare with the scalar evaluator on perturbed poses')
    ap.add_argument('--body', choices=['man', 'woman'], action='append')
    ap.add_argument('--frames', type=int, default=40)
    a = ap.parse_args(argv)
    if not a.check:
        ap.error('nothing to do (use --check)')
    for body in a.body or ['man', 'woman']:
        print(compare_with_scalar(body, a.frames))
    return 0


if __name__ == '__main__':
    sys.exit(main())
