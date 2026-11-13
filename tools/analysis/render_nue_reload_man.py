"""Skin cache (LBS of the real player body/arms/head meshes) for female vs male Nue reload frames + hand-to-torso clearance.

Female rows use the female installed set (player_woman_skeleton + pwa meshes), male rows the retargeted male set
(player_man_skeleton + pma meshes).  Writes <out>.npz/.json for vehicle-entry/render_preview_blender.py and a clearance
report (min distance of the left-hand joints / right hand to the torso mesh skin, per clip, male vs female).
Offline, math preview only (no engine skinning/helpers): not a runtime proof.

python -I render_nue_reload_man.py --female F.anims.glb --male M.anims.glb --out BASE --clips A,B --frames 5,20,51,80 --nframes 81 [--report R.json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
VE = ROOT / 'Source' / 'tools' / 'anim' / 'vehicle-entry'
sys.path.insert(0, str(VE))
import skin_cache as SC  # noqa: E402
import skel as S  # noqa: E402

HANDS_L = ['LeftHand', 'LeftHandEnd', 'LeftInHandThumb', 'LeftInHandIndex', 'LeftInHandMiddle', 'LeftInHandRing',
           'LeftInHandPinky', 'LeftHandIndex3', 'LeftHandMiddle3', 'LeftHandThumb2', 'LeftForeArm']
HANDS_R = ['RightHand', 'RightHandEnd', 'RightInHandIndex', 'RightHandIndex3', 'RightHandMiddle3', 'RightForeArm']


def skin(parts, sk, Wt, Wr, k):
    Mw = np.zeros((len(sk.names), 4, 4))
    Mw[:, :3, :3] = Wr[k]
    Mw[:, :3, 3] = Wt[k]
    Mw[:, 3, 3] = 1
    out = []
    for p in parts:
        bind = SC.joint_binding(p, sk.index)
        jmap = np.array([sk.index[a] for a, _ in bind])
        sm = Mw[jmap] @ np.stack([m for _, m in bind])
        ph = np.c_[p['pos'], np.ones(len(p['pos']))]
        v = np.zeros((len(ph), 3))
        for c in range(p['js'].shape[1]):
            m = sm[p['js'][:, c]]
            v += p['ws'][:, c:c + 1] * np.einsum('vij,vj->vi', m, ph)[:, :3]
        out.append((p['name'], v))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--female', required=True)
    ap.add_argument('--male', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--clips', required=True)
    ap.add_argument('--frames', required=True, help='frame indices, e.g. 5,20,51,80')
    ap.add_argument('--report')
    ap.add_argument('--no-clearance', action='store_true')
    a = ap.parse_args()
    frames = [int(x) for x in a.frames.split(',')]
    clips = a.clips.split(',')
    meshes = {'woman': SC.load_meshes('woman'), 'man': SC.load_meshes('man')}
    arrays = {}
    layout = dict(frames=[], rows=[], parts=[])
    row = 0
    report = {}
    faces = None
    for clip in clips:
        for label, path, body in (('F ' + clip, a.female, 'woman'), ('M ' + clip, a.male, 'man')):
            sk = S.Skeleton.from_glb(path)
            ci = sk.clip(clip)
            import glbanim as G
            ch = G.channels(sk.gj, sk.binb, ci)
            n = len(ch['LeftHand']['rotation'][0])
            times = ch['LeftHand']['rotation'][0][frames]
            lt, lr = sk.sample_local(ci, times)
            ri = sk.index['Root']
            lt[:, ri, 0] = 0.0
            lt[:, ri, 2] = 0.0
            Wt, Wr = sk.world(lt, S.qcontinuous(lr))
            parts = meshes[body]
            fl, off = [], 0
            for p in parts:
                fl.append(p['faces'] + off)
                off += len(p['pos'])
            arrays['faces_%d' % row] = np.concatenate(fl).astype(np.int32)  # male/female meshes differ: faces per row
            layout['parts'] = [p['name'] for p in parts]
            layout['rows'].append(dict(label=label, path=path, clip=clip, times=[float(t) for t in times]))
            for k in range(len(frames)):
                sv = skin(parts, sk, Wt, Wr, k)
                v = np.concatenate([x[1] for x in sv])
                arrays['verts_%d_%d' % (row, k)] = np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1).astype(np.float32)
            row += 1
    layout['frames'] = [f / 80.0 for f in frames]
    np.savez_compressed(a.out + '.npz', **arrays)
    Path(a.out + '.json').write_text(json.dumps(layout, indent=1), encoding='utf-8')
    print('cache', a.out, 'rows', row)
    if a.no_clearance:
        return
    # clearance over ALL frames of each clip: torso mesh (t0_*) vs hand joints
    for clip in clips:
        res = {}
        for body, path in (('woman', a.female), ('man', a.male)):
            sk = S.Skeleton.from_glb(path)
            import glbanim as G
            ci = sk.clip(clip)
            ch = G.channels(sk.gj, sk.binb, ci)
            t = ch['LeftHand']['rotation'][0]
            lt, lr = sk.sample_local(ci, t)
            ri = sk.index['Root']
            lt[:, ri, 0] = 0.0
            lt[:, ri, 2] = 0.0
            Wt, Wr = sk.world(lt, S.qcontinuous(lr))
            parts = [p for p in meshes[body] if p['name'].startswith('t0_')]
            dl, dr = [], []
            for k in range(len(t)):
                tv = skin(parts, sk, Wt, Wr, k)[0][1]
                for names, dst in ((HANDS_L, dl), (HANDS_R, dr)):
                    pts = np.array([Wt[k, sk.index[n]] for n in names if n in sk.index])
                    d = np.linalg.norm(tv[None, :, :] - pts[:, None, :], axis=-1).min(axis=1)  # per joint nearest torso vertex
                    dst.append(d.min())
            res[body] = dict(leftHandMinDistToTorsoVertexMeters=float(np.min(dl)), leftHandP05=float(np.percentile(dl, 5)),
                             rightHandMinDistMeters=float(np.min(dr)), frames=len(t))
        report[clip] = res
        print(clip, json.dumps(res))
    if a.report:
        Path(a.report).write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
