"""Exports a runtime helper bundle (program.ir + binding.dat) from a player deformer graph.

The bundle is read by src/mm/BodyDeformerProgramFile.hpp and src/mm/BodyHelperBindingFile.hpp and run by
src/mm/BodyDeformerOffline.hpp (flat interpreter). The flattening mirrors evaluate_body_deformer_graph.Evaluator (the
offline reference, validated against engine recordings) node for node, in the same execution order; only operators the
interpreter supports are exported. Output nodes are the owned helper bones; everything that does not feed them is pruned.

Usage: export_player_helper_bundle.py --body man --owned owned.txt --out-dir DIR [--manifest manifest.json]
  owned.txt: one helper bone name per line (non-core bones of the deformer rig). The set must be closed under rig
  descendants (a frozen child would keep a stale model transform); the exporter refuses an open set unless the
  missing children are listed in --compensate (hashes of unowned direct children in the merged player rig).
Inputs (graph/rig JSON written by WolvenKit from the installed game, core clip name list, the owned list and the merged rig) and the tool itself
are hashed into the manifest. The output is deterministic: the same inputs give byte-identical program.ir / binding.dat.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import evaluate_body_deformer_graph as E  # noqa: E402

KIND_REFERENCE, KIND_CORE, KIND_LOGICAL = 0, 1, 2
OP_INIT, OP_SET, OP_POINT, OP_ORIENT, OP_AIMOBJ, OP_AIMROT, OP_SPLINE, OP_TWIST, OP_BOUNCE, OP_LIMIT, OP_TRACKBOUNCE, OP_SPLINETRACK = range(12)
METHODS = {'WholeTransform': 0, 'TranslationOnly': 1, 'RotationOnly': 2, 'NoSnapping': 3}
# 0..2 position, 3..5 scale, 6..8 rotation in degrees (Rz * Rx * Ry convention of deformer_rotation_channels.py); the interpreter's channel kinds
CHANNELS = {'PosX': 0, 'PosY': 1, 'PosZ': 2, 'ScaleX': 3, 'ScaleY': 4, 'ScaleZ': 5, 'RotX': 6, 'RotY': 7, 'RotZ': 8}


def fnv64(name):
    h = 0xcbf29ce484222325
    for c in name.encode():
        h ^= c
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def tqs(x):
    t = x['Translation']
    q = x['Rotation']
    s = x['Scale']
    return ((t['X'], t['Y'], t['Z']), (q['i'], q['j'], q['k'], q['r']), (s['X'], s['Y'], s['Z']))


def fmt(x):
    return repr(float(x))


class Op:
    def __init__(self, kind, handle, target=-1, a=-1, b=-1, c=-1):
        self.kind, self.handle, self.target, self.a, self.b, self.c = kind, handle, target, a, b, c
        self.p = [0.0] * 8
        self.v = [(0.0, 0.0, 0.0)] * 3
        self.x = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (1.0, 1.0, 1.0))
        self.inputs = []
        self.channels = []
        self.unsupported = []
        self.channel_unsupported = []  # per channel: the unknown channel name or '' (parallel to channels)

    def reads(self):
        out = set()
        for n in (self.a, self.b, self.c):
            if n >= 0:
                out.add(n)
        out.update(n for n, _ in self.inputs)
        for tgt, parent, _, _ in self.channels:
            out.add(tgt)
            out.add(parent)
        if self.kind not in (OP_INIT, OP_BOUNCE, OP_TRACKBOUNCE) and self.target >= 0:
            out.add(self.target)  # in-place update: the previous value is an input
        return out

    def writes(self):
        if self.kind == OP_BOUNCE:
            return {tgt for tgt, _, _, _ in self.channels}
        if self.kind == OP_TRACKBOUNCE:
            return set()
        return {self.target}


class Program:
    def __init__(self):
        self.names = []     # unique internal names
        self.parent = []
        self.kind = []
        self.ref = []
        self.ops = []
        self.track_ops_target = {}

    def add(self, name, parent, kind, ref):
        self.names.append(name)
        self.parent.append(parent)
        self.kind.append(kind)
        self.ref.append(ref)
        return len(self.names) - 1


def flatten(ev):
    """Full program for the body (all ops) with name->node versioning for extender re-initialisations."""
    prog = Program()
    rig_ref = ev.rig['boneTransforms']
    cur = {}
    for i, n in enumerate(ev.names):
        parent = ev.parents[i]
        kind = KIND_CORE if n in ev.coreNames else KIND_REFERENCE
        cur[n] = prog.add(n, parent, kind, tqs(rig_ref[i]))
    parent_names = {n: (ev.names[ev.parents[i]] if ev.parents[i] >= 0 else None) for i, n in enumerate(ev.names)}
    tracks = {}
    skipped = []

    def nm(x):
        return x['name']['$value']

    def node(n):
        if n not in cur:
            raise ValueError('Unknown transform ' + n)
        return cur[n]

    for h in sorted(ev.nodes, reverse=True):
        if h < 1:
            break
        node_ = ev.nodes[h]
        t = node_['$type'].removeprefix('animAnimNode_')
        if 'SourceChannel' in t:
            continue
        if t in ['Root', 'Output', 'ReferencePoseTerminator', 'SharedMetaPose', 'PoseLsToMs', 'PoseMsToLs', 'StackTransformsShrinker',
                 'StackTracksExtender', 'StackTracksShrinker', 'ConditionalSegmentBegin', 'ConditionalSegmentEnd',
                 'SkipPerformanceModeBegin', 'SkipPerformanceModeEnd']:
            continue
        if t == 'StackTransformsExtender':
            pending = list(node_['transformInfos'])
            while pending:
                done = []
                for e in pending:
                    n, p = nm(e), e['parentName']['$value']
                    parent_names[n] = p
                    if p in cur:
                        if n in cur:  # re-initialisation of an existing name: the old node keeps an internal name
                            old = cur[n]
                            prog.names[old] = '%s~old%d' % (n, old)
                        new = prog.add(n, -1, KIND_LOGICAL, ((0, 0, 0), (0, 0, 0, 1), (1, 1, 1)))
                        op = Op(OP_INIT, h, target=new, a=cur[p])
                        op.x = tqs(e['referenceTransformLs'])
                        prog.ops.append(op)
                        cur[n] = new
                        done.append(e)
                    elif p == 'None':
                        raise ValueError('Extender transform without parent: ' + n)
                if not done:
                    raise ValueError('Unresolved logical parents')
                pending = [e for e in pending if e not in done]
            continue
        if node_.get('weight', 1) != 1:
            raise ValueError('Unsupported fractional constraint weight')
        if t == 'TwistConstraint':
            front = E.axis(node_['frontAxis'])
            side = np.array([0., 0., 1.]) if abs(front[2]) < .5 else np.array([1., 0., 0.])
            for e in node_['outputs']:
                n = nm(e['twistedTransform'])
                op = Op(OP_TWIST, h, target=node(n), a=node(nm(node_['transformA'])), b=node(nm(node_['transformB'])),
                        c=node(parent_names[n]))
                op.p[0], op.p[1] = e['positiveScale'], e['negativeScale']
                op.v = [tuple(front), tuple(side), tuple(E.axis(e['twistAxis']))]
                prog.ops.append(op)
        elif t == 'SetBoneTransform':
            for e in node_['entries']:
                if e['snapToReference'] or e['offsetToReference']:
                    raise ValueError('Reference snapping unsupported')
                n, s, p = nm(e['transformToChange']), nm(e['sourceBone']), nm(e['offsetSpaceBone'])
                method = METHODS[e['setMethod']]
                op = Op(OP_SET, h, target=node(n), a=node(s) if method != 3 else -1, b=node(p) if p != 'None' else -1)
                op.p[0] = float(method)
                op.x = tqs(e['offset'])
                prog.ops.append(op)
        elif t in ('PointConstraint', 'OrientConstraint'):
            n = nm(node_['transformIndex'])
            entries, weights = node_['inputWeightedTransforms'], node_['preprocessedWeights']
            op = Op(OP_POINT if t == 'PointConstraint' else OP_ORIENT, h, target=node(n))
            if t == 'PointConstraint':
                op.inputs = [(node(nm(x['transform'])), float(w)) for w, x in zip(weights, entries)]
            else:
                op.inputs = [(node(nm(entries[0]['transform'])), 0.0)] + \
                            [(node(nm(x['transform'])), float(w)) for w, x in zip(weights, entries[1:])]
            prog.ops.append(op)
        elif t.startswith('AimConstraint_'):
            n = nm(node_['transformIndex'])
            rotation_up = t.endswith('ObjectRotationUp')
            op = Op(OP_AIMROT if rotation_up else OP_AIMOBJ, h, target=node(n), a=node(nm(node_['targetTransform'])),
                    b=node(nm(node_['upTransform'])))
            up_vec = E.vec(node_['upTransformVector']) if rotation_up else np.zeros(3)
            op.v = [tuple(E.vec(node_['forwardAxisLS'])), tuple(E.vec(node_['upAxisLS'])), tuple(up_vec)]
            prog.ops.append(op)
        elif t == 'SimpleSpline':
            n = nm(node_['constrainedTransform'])
            if node_['progressMode'] == 'FloatTrack':
                track = nm(node_['progressTrack'])
                if track not in tracks:
                    raise ValueError('Unproduced spline track ' + track)
                op = Op(OP_SPLINETRACK, h, target=node(n), a=node(nm(node_['startTransform'])), b=node(nm(node_['middleTransform'])),
                        c=node(nm(node_['endTransform'])))
                op.p[1] = float(tracks[track])
            elif node_['progressMode'] == 'Static':
                op = Op(OP_SPLINE, h, target=node(n), a=node(nm(node_['startTransform'])), b=node(nm(node_['middleTransform'])),
                        c=node(nm(node_['endTransform'])))
                op.p[0] = float(node_['defaultProgress'])
            else:
                raise ValueError('Unsupported spline progress mode')
            prog.ops.append(op)
        elif t == 'SimpleBounce':
            if nm(node_['outputDriverTrack']) != 'None':
                raise ValueError('Unsupported bounce driver track')
            start, end = node(nm(node_['startTransform'])), node(nm(node_['endTransform']))
            params = [node_['multiplier'], node_['negativeMultiplier'], node_['delay'], node_['smoothStep'], node_['offset']]
            for e in node_['trackOutputs']:
                track = nm(e['targetTrack'])
                if track not in tracks:
                    if len(tracks) >= 32:
                        raise ValueError('Too many tracks')
                    tracks[track] = len(tracks)
                op = Op(OP_TRACKBOUNCE, h, target=tracks[track], a=start, b=end)
                op.p[:5] = [float(x) for x in params]
                op.p[5] = float(e['multiplier'])
                prog.ops.append(op)
            channels = []
            unsupported = []
            channel_unsupported = []
            for e in node_['transformOutputs']:
                n = nm(e['targetTransform'])
                parent = node(nm(e['parentTransform']))
                for c in e['channelEntries']:
                    if c['transformChannel'] not in CHANNELS:
                        # a channel the interpreter does not know: keep the op so that a slice that needs it is refused
                        unsupported.append(n + ':' + c['transformChannel'])
                        channel_unsupported.append(n + ':' + c['transformChannel'])
                        channels.append((node(n), parent, 99, 0.0))  # invalid kind: the interpreter refuses it
                    else:
                        channel_unsupported.append('')
                        channels.append((node(n), parent, CHANNELS[c['transformChannel']], float(c['multiplier'])))
            if channels:
                if len(channels) > 32:
                    raise ValueError('Too many channels')
                op = Op(OP_BOUNCE, h, a=start, b=end)
                op.p[:5] = [float(x) for x in params]
                op.channels = channels
                op.unsupported = unsupported
                op.channel_unsupported = channel_unsupported
                prog.ops.append(op)
        elif t == 'TranslationLimit':
            n = nm(node_['constrainedTransform'])
            pn = nm(node_['parentTransform'])
            op = Op(OP_LIMIT, h, target=node(n), a=node(parent_names[n] if pn == 'None' else pn))
            mn, mx, lo, hi = 0, 0, [0., 0., 0.], [0., 0., 0.]
            for a in range(3):
                c = node_['limitOn' + 'XYZ'[a] + 'Axis']
                if c['useMin']:
                    mn |= 1 << a
                    lo[a] = c['min']
                if c['useMax']:
                    mx |= 1 << a
                    hi[a] = c['max']
            op.p[0], op.p[1] = float(mn), float(mx)
            op.v = [tuple(lo), tuple(hi), (0., 0., 0.)]
            prog.ops.append(op)
        else:
            raise ValueError('Unsupported ' + t)
    return prog, cur


def slice_program(prog, outputs, anchor):
    """Backward slice: keep the ops that write (transitively) into the outputs; return keep-node-set, kept ops."""
    need = set(outputs) | {anchor}
    keep_ops = []
    need_tracks = set()
    for op in reversed(prog.ops):
        if op.kind == OP_TRACKBOUNCE:
            if op.target in need_tracks:
                keep_ops.append(op)
                need.update(op.reads())
            continue
        if op.writes() & need:
            if op.kind == OP_BOUNCE:
                # Only the outputs of this bounce that something downstream needs are kept: a dropped target is never read later
                # (a later reader would already be in `need`), so its value stays unobserved and the result is identical.
                kept = [(c, u) for c, u in zip(op.channels, op.channel_unsupported) if c[0] in need]
                if len(kept) != len(op.channels):
                    op = copy.copy(op)
                    op.channels = [c for c, _ in kept]
                    op.channel_unsupported = [u for _, u in kept]
                    op.unsupported = [u for u in op.channel_unsupported if u]
            keep_ops.append(op)
            need.update(op.reads())
            if op.kind == OP_SPLINETRACK:
                need_tracks.add(int(op.p[1]))
    keep_ops.reverse()
    blocked = [u for op in keep_ops for u in op.unsupported]
    if blocked:
        raise ValueError('Slice needs unsupported operators: ' + ', '.join(blocked))
    # reference/core nodes need their parent chain (first-pass model composition)
    changed = True
    while changed:
        changed = False
        for n in list(need):
            if prog.kind[n] != KIND_LOGICAL and prog.parent[n] >= 0 and prog.parent[n] not in need:
                need.add(prog.parent[n])
                changed = True
    return need, keep_ops


def write_bundle(prog, cur, ev, owned, children, out_dir, identity, anchor_name='Spine3'):
    owned_nodes = [cur[n] for n in owned]
    for n in owned:
        if prog.kind[cur[n]] == KIND_CORE:
            raise ValueError('Owned helper is a core bone: ' + n)
    anchor = cur[anchor_name]
    need, ops = slice_program(prog, owned_nodes, anchor)
    # renumber: reference/core nodes keep rig order, logical nodes follow in creation order
    order = [i for i in range(len(prog.names)) if i in need and prog.kind[i] != KIND_LOGICAL] + \
            [i for i in range(len(prog.names)) if i in need and prog.kind[i] == KIND_LOGICAL]
    new_index = {old: k for k, old in enumerate(order)}
    mapn = lambda n: new_index[n] if n >= 0 else -1  # noqa: E731
    lines = []
    for old in order:
        t, q, s = prog.ref[old]
        parent = mapn(prog.parent[old]) if prog.parent[old] >= 0 else -1
        lines.append('NODE %d %s %d %d %s' % (new_index[old], prog.names[old], parent, prog.kind[old],
                                              ' '.join(fmt(x) for x in (*t, *q, *s))))
    names = [prog.names[o] for o in order]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate internal names')
    op_lines = []
    for op in ops:
        vals = [op.kind, op.handle, mapn(op.target) if op.kind not in (OP_TRACKBOUNCE,) else op.target, mapn(op.a), mapn(op.b), mapn(op.c)]
        s = ' '.join(str(x) for x in vals)
        s += ' ' + ' '.join(fmt(x) for x in op.p)
        s += ' ' + ' '.join(fmt(x) for v in op.v for x in v)
        s += ' ' + ' '.join(fmt(x) for x in (*op.x[0], *op.x[1], *op.x[2]))
        s += ' %d' % len(op.inputs) + ''.join(' %d %s' % (mapn(n), fmt(w)) for n, w in op.inputs)
        s += ' %d' % len(op.channels) + ''.join(' %d %d %d %s' % (mapn(tg), mapn(pr), k, fmt(m)) for tg, pr, k, m in op.channels)
        op_lines.append('OP ' + s)
    # track registers must be dense from 0 in the interpreter; remap
    used = sorted({op.target for op in ops if op.kind == OP_TRACKBOUNCE})
    if used != list(range(len(used))):
        remap = {old: k for k, old in enumerate(used)}
        fixed = []
        for op, line in zip(ops, op_lines):
            if op.kind == OP_TRACKBOUNCE:
                parts = line.split(' ')
                parts[3] = str(remap[op.target])
                line = ' '.join(parts)
            elif op.kind == OP_SPLINETRACK:
                parts = line.split(' ')
                # p[1] sits at token index 3 (OP kind handle target a b c) -> p0 at 7, p1 at 8
                parts[8] = fmt(float(remap[int(op.p[1])]))
                line = ' '.join(parts)
            fixed.append(line)
        op_lines = fixed
    program_text = 'UTPP_DEFORMER_IR 1 %d %d\nSHA256 %s\n' % (len(order), len(ops), identity) + '\n'.join(lines) + '\n' + '\n'.join(op_lines) + '\n'
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    program_bytes = program_text.encode('ascii')
    (out / 'program.ir').write_bytes(program_bytes)
    prog_sha = sha256_bytes(program_bytes)
    core_names = sorted(ev.coreNames, key=lambda n: fnv64(n))
    owned_sorted = sorted(owned, key=lambda n: fnv64(n))
    b = ['UTPP_HELPER_BINDING 1', 'SHA256 ' + identity, 'PROGRAM_SHA256 ' + prog_sha,
         'COUNTS %d %d %d %d 0' % (len(order), len(core_names), len(owned_sorted), len(children)),
         'ANCHOR %016X %d' % (fnv64(anchor_name), new_index[anchor])]
    b += ['CORE %016X' % fnv64(n) for n in core_names]
    b += ['OWNED %016X %d' % (fnv64(n), new_index[cur[n]]) for n in owned_sorted]
    b += ['CHILD %016X' % h for h in sorted(children)]
    b += ['END']
    binding_bytes = ('\n'.join(b) + '\n').encode('ascii')
    (out / 'binding.dat').write_bytes(binding_bytes)
    return {'nodes': len(order), 'ops': len(ops), 'owned': len(owned_sorted), 'children': len(children),
            'program_sha256': prog_sha, 'binding_sha256': sha256_bytes(binding_bytes),
            'program_bytes': len(program_bytes), 'binding_bytes': len(binding_bytes), 'node_names': names,
            'owned_index': {n: new_index[cur[n]] for n in owned_sorted}}


def merged_children(ev, owned, merged_names_hash, merged_parents, merged_hashes_known):
    """Direct non-owned children (hash) of owned bones in the merged player rig."""
    owned_hash = {fnv64(n): n for n in owned}
    out = []
    for i, p in enumerate(merged_parents):
        if p >= 0 and merged_names_hash[p] in owned_hash and merged_names_hash[i] not in owned_hash:
            out.append(merged_names_hash[i])
    return sorted(set(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--body', default='man')
    ap.add_argument('--owned', required=True)
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--merged-rig', help='json {hashes:[..hex..], parents:[..]} of the player merged rig (children detection)')
    ap.add_argument('--manifest')
    a = ap.parse_args()
    ev = E.Evaluator(body=a.body)
    owned = [x.strip() for x in Path(a.owned).read_text().splitlines() if x.strip() and not x.startswith('#')]
    for n in owned:
        if n not in ev.names or n in ev.coreNames:
            raise SystemExit('Not a non-core rig bone: ' + n)
    prog, cur = flatten(ev)
    # closure check in the deformer rig
    owned_set = set(owned)
    rig_parent = {n: (ev.names[ev.parents[i]] if ev.parents[i] >= 0 else None) for i, n in enumerate(ev.names)}
    for n in ev.names:
        if n not in owned_set and rig_parent[n] in owned_set and n not in ev.coreNames:
            raise SystemExit('Owned set not closed under descendants: %s is a child of %s' % (n, rig_parent[n]))
    children = []
    if a.merged_rig:
        m = json.loads(Path(a.merged_rig).read_text())
        hashes = [int(x, 16) for x in m['hashes']]
        children = merged_children(ev, owned, hashes, m['parents'], None)
    inputs = [ev.graphPath, ev.rigPath, ev.coreClipPath]
    idh = hashlib.sha256()
    for p in inputs:
        idh.update(Path(p).read_bytes())
    idh.update('\n'.join(sorted(owned)).encode())
    identity = idh.hexdigest()
    info = write_bundle(prog, cur, ev, owned, children, a.out_dir, identity)
    extra = [('owned list', a.owned)] + ([('merged rig (hashes + parents)', a.merged_rig)] if a.merged_rig else [])
    manifest = {'tool': 'export_player_helper_bundle.py', 'tool_sha256': sha256_bytes(Path(__file__).read_bytes()), 'body': a.body, 'identity': identity,
                'inputs': [{'path': str(Path(p).resolve()), 'sha256': sha256_bytes(Path(p).read_bytes())} for p in inputs] +
                          [{'path': str(Path(p).resolve()), 'sha256': sha256_bytes(Path(p).read_bytes()), 'role': role} for role, p in extra],
                'merged_rig_children': [('%016X' % h) for h in children]}
    manifest.update({k: v for k, v in info.items() if k not in ('node_names',)})
    if a.manifest:
        Path(a.manifest).write_text(json.dumps(manifest, indent=1))
    print(json.dumps({k: v for k, v in manifest.items() if k not in ('inputs', 'owned_index', 'merged_rig_children')}, indent=1))


if __name__ == '__main__':
    main()
