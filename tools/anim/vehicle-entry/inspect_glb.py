"""Print a summary of an exported .anims.glb: nodes, skin, clips, channels, extras, root motion end."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
import glbanim as G  # noqa: E402


def main():
    path = sys.argv[1]
    want = sys.argv[2:] or None
    gj, binb = G.read_glb(path)
    names = G.node_names(gj)
    joints, sk = G.skin_joints(gj)
    par = G.parents(gj)
    print('nodes', len(names), 'skin joints', len(joints), 'scenes roots', gj['scenes'][0]['nodes'])
    print('first nodes', [(i, names[i], names[par[i]] if i in par else None) for i in range(min(6, len(names)))])
    print('skin extras', json.dumps(sk.get('extras'))[:300])
    print('root node xform', {k: gj['nodes'][gj['scenes'][0]['nodes'][0]].get(k) for k in ('rotation', 'translation', 'scale')})
    for name, i in G.clips(gj).items():
        if want and name not in want:
            continue
        a = gj['animations'][i]
        ex = a.get('extras', {})
        ch = G.channels(gj, binb, i)
        tmax = max(float(tr[0][-1]) for d in ch.values() for tr in d.values())
        exs = {k: v for k, v in ex.items() if k not in ('animEvents', 'AnimEvents', 'trackKeys', 'TrackKeys', 'constTrackKeys', 'ConstTrackKeys')}
        print('== clip', name, 'tmax', round(tmax, 4), 'nodes animated', len(ch))
        print('   extras', json.dumps(exs)[:600])
        ev = ex.get('animEvents') or ex.get('AnimEvents')
        if ev:
            print('   events', json.dumps(ev)[:900])
        for rn in ('Root', 'root', 'Hips', 'Trajectory'):
            if rn in ch:
                for k, tr in ch[rn].items():
                    print('   ', rn, k, 'keys', len(tr[0]), 'first', np.round(tr[1][0], 4), 'last', np.round(tr[1][-1], 4))


if __name__ == '__main__':
    main()
