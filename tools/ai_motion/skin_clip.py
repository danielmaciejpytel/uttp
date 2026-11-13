"""Linear-blend skin the female V body meshes with a retargeted .utppclip.json and write a Blender mesh cache.

python -I skin_clip.py <clip.utppclip.json> <out_base> [--deform graph|rigid] [--world] [--body woman|man] [--strict]
--body man: male V (pma meshes + player_man_fpp_deformations rig/graph); default woman = unchanged female behaviour.
--strict: a graph evaluation error aborts the run (no silent rigid fallback).
--world: vertices and joints in game world (Trajectory root motion applied) instead of the in-place armature space.
Writes <out_base>.npz (faces int32, rest verts of frame 0, joints: world positions of a few joints per frame, game
coordinates = Blender Z-up metres, armature space of the retarget preview) and <out_base>.pc2 (POINTCACHE2, one sample
per clip frame) for a MESH_CACHE modifier.
Meshes: the offline exports listed in Source/tools/anim/vehicle-entry/skin_cache.py (body, arms_hq l/r, head).
Skeleton: Extracted/game/json/vanilla-extra/player_woman_base_deformations.rig.json (core bones + body helper joints with their
real parents: deltoids, scapula, chest, latissimus, triceps, calf, thigh ...). Core locals come from the clip
([t xyz, q xyzw, s xyz]; Trajectory left out, see clip_cores); helpers start from their reference locals.
--deform graph (default): helpers driven by the game's player_woman_base_deformations.animgraph (offline evaluator
  Source/tools/analysis/evaluate_body_deformer_graph.py; falls back to rigid per frame if the evaluator raises).
--deform rigid: helpers follow their parent rigidly.
glTF joint frames = C W C^-1 (C: game Z-up -> glTF Y-up, checked against the meshes' inverse bind matrices). Mesh joints
missing from the deformation rig (face) follow Head (head mesh) or the nearest rig joint.
"""
import json
import os
import struct
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'anim', 'vehicle-entry'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'analysis'))
import skel as S  # noqa: E402
import skin_cache as SC  # noqa: E402

DEFORM_RIG = os.path.join(ROOT, 'Extracted', 'game', 'json', 'vanilla-extra', 'player_woman_base_deformations.rig.json')
# --body man: the male player deformation rig (player_man_fpp_deformations, current game, Extracted/game/body-v/json) and pma meshes
DEFORM_RIG_BY_BODY = {'woman': DEFORM_RIG,
                      'man': os.path.join(ROOT, 'Extracted', 'game', 'body-v', 'json', 'player_man_fpp_deformations.rig.json')}
C = np.eye(4)
C[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
CI = np.linalg.inv(C)
CHECK = ['Hips', 'Head', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot']


def mat(t, q, s=(1, 1, 1)):
    m = np.eye(4)
    m[:3, :3] = S.q2m(np.asarray(q, dtype=np.float64)) * np.asarray(s, dtype=np.float64)[None, :]
    m[:3, 3] = t
    return m


def load_rig(path):
    rc = json.load(open(path, encoding='utf-8'))['Data']['RootChunk']
    names = [b['$value'] for b in rc['boneNames']]
    par = list(rc['boneParentIndexes'])
    loc = []
    for t in rc['boneTransforms']:
        p, q, s = t['Translation'], t['Rotation'], t['Scale']
        loc.append(mat([p['X'], p['Y'], p['Z']], [q['i'], q['j'], q['k'], q['r']], [s['X'], s['Y'], s['Z']]))
    return names, par, loc


def clip_cores(clip, rig_names):
    """Per frame: {bone: local 4x4} for the clip bones the deformation rig has. Trajectory (root motion) is left out:
    the retarget preview carries it on the armature object and the cache is in armature space."""
    bn = clip['bone_names']
    out = []
    for fr in clip['frames']:
        loc = {n: mat(b[0:3], b[3:7], b[7:10]) for n, b in zip(bn, fr['bones'])}
        out.append({n: m for n, m in loc.items() if n in rig_names})
    return out


def fk_rigid(names, par, ref, core):
    W = {}
    for i, n in enumerate(names):
        L = core.get(n, ref[i])
        W[n] = L if par[i] < 0 else W[names[par[i]]] @ L
    return W


def write_pc2(path, frames):
    T, V = frames.shape[:2]
    with open(path, 'wb') as f:
        f.write(b'POINTCACHE2\0' + struct.pack('<iiffi', 1, V, 0.0, 1.0, T))
        f.write(frames.astype('<f4').tobytes())


def bindings(p, rig_idx, head_part):
    out = SC.joint_binding(p, rig_idx)
    if head_part and 'Head' in p['node_of']:
        h = p['node_of']['Head']
        for k, jn in enumerate(p['jn']):
            if jn not in rig_idx:
                out[k] = ('Head', np.linalg.inv(p['nodew'][h]) @ p['nodew'][p['node_of'][jn]] @ p['ibm'][k])
    return out


def main(a):
    mode = a[a.index('--deform') + 1] if '--deform' in a else 'graph'
    body = a[a.index('--body') + 1] if '--body' in a else 'woman'  # default = historical female behaviour
    strict = '--strict' in a  # measurement runs: abort instead of silently falling back to rigid helpers when the graph raises
    names, par, ref = load_rig(DEFORM_RIG_BY_BODY[body])
    idx = {n: i for i, n in enumerate(names)}
    clip = json.load(open(a[0], encoding='utf-8'))
    cores = clip_cores(clip, set(names))
    world = '--world' in a
    ti = clip['bone_names'].index('Trajectory')
    trajs = [mat(fr['bones'][ti][0:3], fr['bones'][ti][3:7]) for fr in clip['frames']]
    ev = None
    if mode == 'graph':
        from evaluate_body_deformer_graph import Evaluator
        ev = Evaluator(allow_rotation_bounce=True, body=body)
    parts = SC.load_meshes(body)
    faces, off, binds = [], 0, []
    for p in parts:
        faces.append(p['faces'] + off)
        off += len(p['pos'])
        b = bindings(p, idx, p['name'].startswith('h0_'))
        binds.append(([n for n, _ in b], np.stack([m for _, m in b])))
    T = len(cores)
    verts = np.zeros((T, off, 3), dtype=np.float32)
    joints = np.zeros((T, len(CHECK), 3))
    fallback = []
    for t, core in enumerate(cores):
        W = fk_rigid(names, par, ref, core)
        if ev is not None:
            try:
                M = ev.run({n: m for n, m in core.items() if n in ev.coreNames})
                W.update({n: m for n, m in M.items() if n in W})
            except Exception as e:  # noqa: BLE001 - report and keep the rigid pose for this frame
                if strict:
                    raise
                fallback.append((t, str(e)[:80]))
        vs = []
        for p, (jn, pre) in zip(parts, binds):
            Wl = np.stack([C @ W[n] @ CI for n in jn])
            skin = Wl @ pre
            ph = np.c_[p['pos'], np.ones(len(p['pos']))]
            v = np.zeros((len(ph), 3))
            for c in range(p['js'].shape[1]):
                v += p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', skin[p['js'][:, c]], ph)[:, :3]
            vs.append(v)
        v = np.concatenate(vs)
        verts[t] = np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1)  # glTF -> game / Blender Z-up
        joints[t] = [W[n][:3, 3] for n in CHECK]
        if world:  # bake the root motion (Trajectory under an identity Root) into the cache
            Tm = trajs[t]
            verts[t] = verts[t] @ Tm[:3, :3].T + Tm[:3, 3]
            joints[t] = joints[t] @ Tm[:3, :3].T + Tm[:3, 3]
    np.savez(a[1] + '.npz', faces=np.concatenate(faces).astype(np.int32), rest=verts[0], joints=joints,
             joint_names=np.array(CHECK), clip=os.path.basename(a[0]), deform=mode)
    write_pc2(a[1] + '.pc2', verts)
    print('SKIN_CLIP %s frames %d verts %d faces %d deform %s graph-fallback frames %d%s'
          % (a[1], T, off, sum(len(f) for f in faces), mode, len(fallback),
             (' first: %s' % (fallback[0],)) if fallback else ''))


if __name__ == '__main__':
    main(sys.argv[1:])
