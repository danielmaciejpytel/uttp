"""Replay owned LOCAL TRS capture on a verified retail mesh. Never fills missing poses.
Requires numpy; output is local analysis data, not a distributable asset.
"""
import argparse
import csv
import hashlib
import io
import json
import re
from pathlib import Path
import numpy as np


def cname(name):
    value = 0xcbf29ce484222325
    for byte in name.encode('utf-8'):
        value = ((value ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return value


def trs(values):
    """Decode raw game QsTransform: position[4], quaternion xyzw[4], scale[4]."""
    values = np.asarray(values, dtype=float)
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite captured LOCAL TRS')
    x, y, z, w = values[4:8]
    norm = np.linalg.norm(values[4:8])
    if abs(norm - 1) > .01:
        raise ValueError(f'Invalid quaternion norm {norm}')
    x, y, z, w = values[4:8] / norm
    matrix = np.eye(4)
    matrix[:3, :3] = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]) @ np.diag(values[8:11])
    matrix[:3, 3] = values[:3]
    return matrix


def read_capture(path, stage=None):
    lines = path.read_text(encoding='utf-8-sig').splitlines()
    post_helper = any(line.startswith('#') and
                      re.search(r'\bstage=post_helper_apply_after_commit\b', line)
                      for line in lines)
    if (stage == 'post-helper') != post_helper:
        raise ValueError('Post-helper capture requires explicit --stage post-helper and its after-commit stage marker')
    topology = {}
    for line in lines:
        match = re.fullmatch(r'# rig_bone index=(\d+) hash=([0-9A-Fa-f]+) parent=(-?\d+)', line)
        if match:
            index, hashed, parent = match.groups()
            index = int(index)
            if index in topology:
                raise ValueError('Duplicate topology index')
            topology[index] = (int(hashed, 16), int(parent))
    if not topology or sorted(topology) != list(range(len(topology))):
        raise ValueError('Missing/noncontiguous owned topology')
    if len({v[0] for v in topology.values()}) != len(topology):
        raise ValueError('Ambiguous owned name hashes')
    for hashed, parent in topology.values():
        if parent < -1 or parent >= len(topology):
            raise ValueError('Topology parent outside owned bounds')
    frames = {}
    receipts = {}
    reader = csv.DictReader(io.StringIO('\n'.join(x for x in lines if not x.startswith('#'))))
    if post_helper:
        required = {f'local_{i}' for i in range(12)} | {
            'receiptGeneration', 'receiptCycle', 'receiptStartedTicks', 'receiptLastTicks'}
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError('Incomplete post-helper record schema')
    for row in reader:
        key = tuple(int(row[k]) for k in ('ticks', 'desc', 'thread', 'rig', 'generation'))
        if post_helper:
            receipt = tuple(int(row[k]) for k in ('receiptGeneration', 'receiptCycle',
                                                 'receiptStartedTicks', 'receiptLastTicks'))
            if not (receipt[0] > 0 and receipt[1] > 0 and 0 < receipt[2] <= receipt[3] <= key[0]):
                raise ValueError('Invalid post-helper mapping receipt')
            if receipts.setdefault(key, receipt) != receipt:
                raise ValueError('Mixed mapping receipts within post-helper frame')
        index = int(row['gameIndex'])
        if int(row['rigCount']) != len(topology):
            raise ValueError('Frame rig count differs from owned topology')
        if index < 0:
            continue  # Explicit missing slot; closure check below still fails if required.
        if index not in topology or cname(row['name']) != topology[index][0]:
            raise ValueError('Captured name/index differs from owned topology')
        frame = frames.setdefault(key, {})
        if index in frame:
            raise ValueError('Duplicate captured bone')
        frame[index] = row
    return topology, frames


def replay(skin_path, metadata_path, capture_path, out, selected, stage):
    if stage not in (0, 1, '0', '1', 'post-helper'):
        raise ValueError('Unknown capture stage')
    stage = stage if stage == 'post-helper' else int(stage)
    meta = json.loads(metadata_path.read_text())
    skin = np.load(skin_path, allow_pickle=False)
    names = meta['jointNames']
    topology, frames = read_capture(capture_path, stage)
    by_hash = {value[0]: index for index, value in topology.items()}
    absent = [name for name in names if cname(name) not in by_hash]
    if absent:
        raise ValueError(f'Mesh joints missing from owned rig: {absent}')
    indices = [by_hash[cname(name)] for name in names]
    weighted_skin_indices = set()
    for primitive in meta['primitives']:
        prefix = primitive['key']
        joints = skin[prefix+'_joints'].astype(int)
        weights = skin[prefix+'_weights'].astype(float)
        if not np.isfinite(weights).all() or (weights < 0).any():
            raise ValueError('Invalid mesh weights')
        active = weights > 0
        if ((joints[active] < 0) | (joints[active] >= len(names))).any():
            raise ValueError('Nonzero influence joint outside skin bounds')
        weighted_skin_indices.update(int(i) for i in joints[active])
    closure = set()
    for index in (indices[i] for i in sorted(weighted_skin_indices)):
        seen = set()
        while index >= 0:
            if index in seen:
                raise ValueError('Owned parent cycle')
            seen.add(index)
            closure.add(index)
            index = topology[index][1]
    C = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]], float)
    Ci = C.T
    bind = skin['jointBindMatrices']
    ibm = skin['inverseBindMatrices']
    if bind.shape != (len(names),4,4) or ibm.shape != bind.shape:
        raise ValueError('Bind matrix dimensions differ from metadata')
    bind_error = float(np.max(np.abs(bind @ ibm - np.eye(4))))
    if bind_error > 1e-4:
        raise ValueError(f'Bind validation failed: {bind_error}')
    out.mkdir(parents=True, exist_ok=True)
    results = []
    ordered = sorted(frames.items())
    for frame_number in selected:
        if not 0 <= frame_number < len(ordered):
            raise ValueError(f'Frame ordinal outside capture: {frame_number}')
        key, rows = ordered[frame_number]
        missing = sorted(closure - rows.keys())
        if missing:
            raise ValueError(f'Frame {frame_number}: missing ancestor/joint LOCAL TRS indices {missing}; no rest fallback')
        globals_cp = {}
        def fk(index):
            if index not in globals_cp:
                row = rows[index]
                prefix = 'local' if stage == 'post-helper' else f's{stage}'
                local = trs([float(row[f'{prefix}_{i}']) for i in range(12)])
                parent = topology[index][1]
                globals_cp[index] = (fk(parent) if parent >= 0 else np.eye(4)) @ local
            return globals_cp[index]
        # Zero matrices are explicit placeholders for unused joints, never rest poses.
        # Every nonzero influence below must belong to the computed set.
        computed = np.array([index in closure for index in indices], dtype=bool)
        global_gltf = np.zeros((len(indices),4,4), dtype=float)
        for i in np.flatnonzero(computed):
            global_gltf[i] = C @ fk(indices[i]) @ Ci
        arrays = {'jointGlobalMatrices':global_gltf, 'jointComputedMask':computed}
        obj_lines, vertex_offset = [], 1
        primitive_reports = []
        for primitive in meta['primitives']:
            prefix = primitive['key']
            positions = skin[prefix+'_positions'].astype(float)
            joints = skin[prefix+'_joints'].astype(int)
            weights = skin[prefix+'_weights'].astype(float)
            if (joints < 0).any() or (joints >= len(names)).any() or not np.isfinite(weights).all() or (weights < 0).any():
                raise ValueError('Invalid mesh influences')
            if np.max(np.abs(weights.sum(axis=1)-1)) > 1e-4:
                raise ValueError('Invalid mesh weight sums')
            if not computed[joints[weights > 0]].all():
                raise ValueError('Nonzero influence has no computed joint palette')
            mesh_global = np.array(primitive['meshNodeTransform'])
            palette = np.linalg.inv(mesh_global) @ global_gltf @ ibm
            homogeneous = np.column_stack((positions, np.ones(len(positions))))
            vertices = np.zeros_like(positions)
            for influence in range(joints.shape[1]):
                vertices += weights[:,influence,None] * np.einsum('nij,nj->ni', palette[joints[:,influence]], homogeneous)[:,:3]
            if not np.isfinite(vertices).all():
                raise ValueError('Nonfinite skinned vertices')
            triangles = skin[prefix+'_indices'].reshape(-1,3)
            arrays[prefix+'_positions'] = vertices.astype(np.float32)
            arrays[prefix+'_indices'] = triangles
            obj_lines.append(f'o {prefix}')
            obj_lines.extend('v %.9g %.9g %.9g' % tuple(v) for v in vertices)
            obj_lines.extend('f %d %d %d' % tuple(face+vertex_offset) for face in triangles)
            vertex_offset += len(vertices)
            primitive_reports.append({'key':prefix,'vertices':len(vertices),'boundsMin':vertices.min(axis=0).tolist(),'boundsMax':vertices.max(axis=0).tolist()})
        label = 'post-helper' if stage == 'post-helper' else f's{stage}'
        stem = f'frame-{frame_number:04d}-{label}'
        np.savez_compressed(out/(stem+'.npz'), **arrays)
        (out/(stem+'.obj')).write_text('\n'.join(obj_lines)+'\n')
        frame_report = {'frameOrdinal':frame_number,'ticksDescThreadRigGeneration':key,'capturedBones':len(rows),'closureBones':len(closure),'missingClosure':0,'primitives':primitive_reports}
        if stage == 'post-helper':
            first = next(iter(rows.values()))
            frame_report['mappingReceipt'] = {k:int(first[k]) for k in (
                'receiptGeneration','receiptCycle','receiptStartedTicks','receiptLastTicks')}
            frame_report['captureStage'] = 'post_helper_apply_after_commit'
        results.append(frame_report)
    report = {'scope':'Geometric replay only; no visual quality PASS or actual variant identification',
              'capture':str(capture_path),'captureSha256':hashlib.sha256(capture_path.read_bytes()).hexdigest(),
              'skinSha256':hashlib.sha256(skin_path.read_bytes()).hexdigest(),
              'totalFrames':len(ordered),'ownedRigBones':len(topology),'meshJoints':len(names),
              'closureBones':len(closure),'weightedSkinJoints':len(weighted_skin_indices),
              'computedSkinJointNames':[names[i] for i in np.flatnonzero(computed)],
              'uncomputedUnusedSkinJointNames':[names[i] for i in np.flatnonzero(~computed)],
              'unusedPalettePolicy':'Explicit zero matrix, permitted only for zero weight influences; no rest fallback',
              'bindIdentityMaxElementError':bind_error,'stage':stage,'frames':results}
    (out/'replay-report.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skin', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--frames', default='0', help='Comma separated zero based frame ordinals')
    parser.add_argument('--stage', choices=('0','1','post-helper'), default='0',
                        help='0/1 are legacy before/after MM; post-helper requires the separate after-commit capture')
    args = parser.parse_args()
    print(json.dumps(replay(args.skin,args.metadata,args.capture,args.out,[int(x) for x in args.frames.split(',')],args.stage), indent=2))

if __name__ == '__main__':
    main()
