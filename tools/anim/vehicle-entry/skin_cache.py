"""Linear-blend skin the female player body meshes (offline exports under Build/experiments/BodyDeformationOffline/female) with
poses from vehicle-entry clips and write a cache for the Blender preview renderer.

Output: <out>.npz (verts_<row>_<col> float32 (V,3) in Blender Z-up coordinates, faces) + <out>.json (layout).
Root horizontal motion is removed per pose (vertical and yaw kept) so the poses can be laid out in a grid.

Usage (python -I):
  skin_cache.py --out <base> --frames 0,0.2,0.35,0.5,0.75,1.0 --clip LABEL=path.glb:clipname [--clip ...]
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402
import skel as S  # noqa: E402
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from utpp_paths import project_root  # noqa: E402

ROOT = Path(project_root())
MESHES = [ROOT / 'Build/experiments/BodyDeformationOffline/female/retail-export/t0_000_pwa_base__full.glb',
          ROOT / 'Build/experiments/BodyDeformationOffline/female/arms-export/base/characters/common/player_base_bodies/player_female_average/arms_hq/a0_000_pwa_base_hq__l.glb',
          ROOT / 'Build/experiments/BodyDeformationOffline/female/arms-export/base/characters/common/player_base_bodies/player_female_average/arms_hq/a0_000_pwa_base_hq__r.glb',
          ROOT / 'Build/experiments/BodyDeformationOffline/female/head-export-20261017/base/characters/head/pwa/h0_000_pwa_c__basehead/h0_000_pwa_c__basehead.glb']


# male V (pma) meshes: the same four parts, extracted read-only from basegame_4_appearance.archive into Extracted/game/body-v/glb
BODY_GLB = ROOT / 'Extracted/game/body-v/glb'
MESHES_MAN = [BODY_GLB / 't0_000_pma_base__full.glb', BODY_GLB / 'a0_000_pma_base_hq__l.glb',
              BODY_GLB / 'a0_000_pma_base_hq__r.glb', BODY_GLB / 'h0_000_pma_c__basehead.glb']


def load_meshes(body='woman'):
    """body: 'woman' (default, historical female meshes, unchanged) or 'man' (pma meshes)."""
    parts = []
    for p in (MESHES_MAN if body == 'man' else MESHES):
        gj, binb = G.read_glb(str(p))
        names = G.node_names(gj)
        for ni, node in enumerate(gj['nodes']):
            if 'mesh' not in node or 'skin' not in node:
                continue
            sk = gj['skins'][node['skin']]
            jn = [names[j] for j in sk['joints']]
            ibm = G.accessor(gj, binb, sk['inverseBindMatrices']).reshape(-1, 4, 4).transpose(0, 2, 1)  # column-major
            for prim in gj['meshes'][node['mesh']]['primitives']:
                at = prim['attributes']
                if 'JOINTS_0' not in at:
                    continue
                pos = G.accessor(gj, binb, at['POSITION']).astype(np.float64)
                js = G.accessor(gj, binb, at['JOINTS_0']).astype(np.int64)
                ws = G.accessor(gj, binb, at['WEIGHTS_0']).astype(np.float64)
                if 'JOINTS_1' in at:
                    js = np.concatenate([js, G.accessor(gj, binb, at['JOINTS_1']).astype(np.int64)], 1)
                    ws = np.concatenate([ws, G.accessor(gj, binb, at['WEIGHTS_1']).astype(np.float64)], 1)
                idx = G.accessor(gj, binb, prim['indices']).astype(np.int64).reshape(-1, 3)
                parts.append(dict(name=p.stem + ':' + gj['meshes'][node['mesh']].get('name', ''), pos=pos, js=js, ws=ws,
                                  faces=idx, jn=jn, ibm=ibm, nodew=node_world(gj), node_of={n: i for i, n in enumerate(names)},
                                  parents=G.parents(gj)))
    return parts


def node_world(gj):
    par = G.parents(gj)
    W = {}

    def local(n):
        m = np.eye(4)
        if 'matrix' in n:
            return np.array(n['matrix']).reshape(4, 4).T
        r = S.q2m(np.array(n.get('rotation', [0, 0, 0, 1])))
        m[:3, :3] = r * np.array(n.get('scale', [1, 1, 1]))[None, :]
        m[:3, 3] = n.get('translation', [0, 0, 0])
        return m

    def get(i):
        if i not in W:
            W[i] = (get(par[i]) if i in par else np.eye(4)) @ local(gj['nodes'][i])
        return W[i]
    return [get(i) for i in range(len(gj['nodes']))]


def joint_binding(p, anim_index):
    """Per mesh joint: (animated ancestor name, pre-matrix) so that skin = W_anim(anc) @ pre.
    Helper joints missing from the animation rig (deltoids, face) follow their nearest animated ancestor rigidly."""
    names = list(p['node_of'].keys())
    animated = [i for i, n in enumerate(names) if n in anim_index and n not in ('Root', 'Trajectory', 'reference_joint')]
    apos = np.array([p['nodew'][i][:3, 3] for i in animated])
    out = []
    for j, jn in enumerate(p['jn']):
        ni = p['node_of'][jn]
        if jn in anim_index:
            a = ni
        else:
            # helper joint (twist, deltoid, face): follow the animated joint with the nearest bind position
            a = animated[int(np.argmin(np.linalg.norm(apos - p['nodew'][ni][:3, 3], axis=1)))]
        anc = names[a]
        pre = np.linalg.inv(p['nodew'][a]) @ p['nodew'][ni] @ p['ibm'][j]
        if anc == jn:
            pre = p['ibm'][j]
        out.append((anc, pre))
    return out


def pose_world(path, clip, u):
    sk = S.Skeleton.from_glb(path)
    if clip == 'REST':
        lt = np.repeat(sk.rest_t[None], len(u), 0)
        lr = np.repeat(sk.rest_r[None], len(u), 0)
        Wt, Wr = sk.world(lt, lr)
        return sk, Wt, Wr, np.zeros(len(u))
    c = sk.clip(clip)
    ch = G.channels(sk.gj, sk.binb, c)
    dur = max(float(tr[0][-1]) for d in ch.values() for tr in d.values())
    times = np.array([round(x * dur * 30) / 30 for x in u])
    lt, lr = sk.sample_local(c, times)
    ri = sk.index['Root']
    lt[:, ri, 0] = 0.0
    lt[:, ri, 2] = 0.0
    Wt, Wr = sk.world(lt, S.qcontinuous(lr))
    return sk, Wt, Wr, times


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--frames', default='0,0.2,0.35,0.5,0.75,1.0')
    ap.add_argument('--clip', action='append', required=True)
    a = ap.parse_args()
    u = [float(x) for x in a.frames.split(',')]
    parts = load_meshes()
    arrays = {}
    faces_all = []
    off = 0
    for p in parts:
        faces_all.append(p['faces'] + off)
        off += len(p['pos'])
    arrays['faces'] = np.concatenate(faces_all).astype(np.int32)
    layout = dict(frames=u, rows=[], parts=[p['name'] for p in parts])
    for r, spec in enumerate(a.clip):
        label, rest = spec.split('=', 1)
        path, clip = rest.rsplit(':', 1)
        sk, Wt, Wr, times = pose_world(path, clip, u)
        layout['rows'].append(dict(label=label, path=path, clip=clip, times=times.tolist(),
                                   head_y=[float(Wt[k, sk.index['Head'], 1]) for k in range(len(u))]))
        for k in range(len(u)):
            Mw = np.zeros((len(sk.names), 4, 4))
            Mw[:, :3, :3] = Wr[k]
            Mw[:, :3, 3] = Wt[k]
            Mw[:, 3, 3] = 1
            verts = []
            for p in parts:
                bind = joint_binding(p, sk.index)
                jmap = np.array([sk.index[a] for a, _ in bind])
                skin = Mw[jmap] @ np.stack([m for _, m in bind])  # (J,4,4)
                ph = np.c_[p['pos'], np.ones(len(p['pos']))]
                v = np.zeros((len(ph), 3))
                for c in range(p['js'].shape[1]):
                    m = skin[p['js'][:, c]]
                    v += p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', m, ph)[:, :3]
                verts.append(v)
            v = np.concatenate(verts)
            arrays['verts_%d_%d' % (r, k)] = np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1).astype(np.float32)
    np.savez_compressed(a.out + '.npz', **arrays)
    Path(a.out + '.json').write_text(json.dumps(layout, indent=1), encoding='utf-8')
    print(json.dumps(layout))


if __name__ == '__main__':
    main()
