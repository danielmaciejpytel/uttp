"""Compare an authored clip GLB with the GLB re-exported from the imported .anims (WolvenKit round trip) and read the
imported asset's JSON facts.

Usage (python -I): verify_roundtrip.py <authored.glb> <roundtrip.glb> <imported.anims.json> --clip enter [--report out.json]
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402
import skel as S  # noqa: E402


def world(path, clip):
    sk = S.Skeleton.from_glb(path)
    c = sk.clip(clip)
    ch = G.channels(sk.gj, sk.binb, c)
    dur = max(float(tr[0][-1]) for d in ch.values() for tr in d.values())
    t = np.arange(int(round(dur * 30)) + 1) / 30
    lt, lr = sk.sample_local(c, t)
    Wt, Wr = sk.world(lt, S.qcontinuous(lr))
    return sk, dur, t, Wt, Wr, sk.gj['animations'][c].get('extras', {}), set(ch.keys())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('authored')
    ap.add_argument('roundtrip')
    ap.add_argument('json')
    ap.add_argument('--clip', default='enter')
    ap.add_argument('--report')
    a = ap.parse_args()
    s1, d1, t1, W1, R1, e1, ch1 = world(a.authored, a.clip)
    s2, d2, t2, W2, R2, e2, ch2 = world(a.roundtrip, a.clip)
    rep = dict(duration_authored=d1, duration_roundtrip=d2, frames_authored=len(t1), frames_roundtrip=len(t2),
               bones_rig=len(s2.names), bones_animated_roundtrip=len(ch2 & set(s2.names)),
               missing_bones=sorted(set(s2.names) - ch2))
    n = min(len(t1), len(t2))
    perr = np.linalg.norm(W1[:n] - W2[:n][:, [s2.index[b] for b in s1.names]], axis=-1)
    rep['max_joint_pos_err_m'] = float(perr.max())
    rep['max_joint_pos_err_bone'] = s1.names[int(np.argmax(perr.max(0)))]
    rep['mean_joint_pos_err_m'] = float(perr.mean())
    ri = s1.index['Root']
    rep['root_end_authored'] = W1[n - 1, ri].round(4).tolist()
    rep['root_end_roundtrip'] = W2[n - 1, s2.index['Root']].round(4).tolist()
    rep['root_end_err_m'] = float(np.linalg.norm(W1[n - 1, ri] - W2[n - 1, s2.index['Root']]))
    ev1 = sorted((e['startFrame'], e['type'], e['eventName']) for e in e1.get('animEvents') or [])
    ev2 = sorted((e['startFrame'], e['type'], e['eventName']) for e in e2.get('animEvents') or [])
    rep['events_equal'] = ev1 == ev2
    rep['events_roundtrip'] = ev2
    rep['root_motion_type_roundtrip'] = e2.get('rootMotionType')
    doc = json.loads(Path(a.json).read_text(encoding='utf-8'))
    rc = doc['Data']['RootChunk']
    rep['rig'] = rc.get('rig', {}).get('DepotPath', {}).get('$value')
    clips = []
    for an in rc['animations']:
        d = an['Data']['animation']['Data']
        clips.append(dict(name=d['name']['$value'], duration=d.get('duration'), type=d.get('animationType'),
                          motionExtraction=(d.get('motionExtraction') or {}).get('Data', {}).get('$type'),
                          buffer=(d.get('animBuffer') or {}).get('Data', {}).get('$type'),
                          numFrames=(d.get('animBuffer') or {}).get('Data', {}).get('numFrames'),
                          numJoints=(d.get('animBuffer') or {}).get('Data', {}).get('numJoints'),
                          numTracks=(d.get('animBuffer') or {}).get('Data', {}).get('numTracks')))
    rep['asset_clips'] = clips
    rep['authored_sha256'] = hashlib.sha256(Path(a.authored).read_bytes()).hexdigest()
    s = json.dumps(rep, indent=1, default=float)
    print(s)
    if a.report:
        Path(a.report).write_text(s + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
