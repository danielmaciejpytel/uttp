"""All-edge skinned strain: the within-region statistic of redo_skin_check (p98 of |log2(edge / bind edge)| over edges whose BOTH
vertices belong to the region) is blind to a rigid shift of a whole region (an FPP clavicle translated by 20-40 cm moves the shoulder
region rigidly: the tear sits on the edges BETWEEN regions).  Here: p98 over every edge whose FIRST vertex lies in the region
(cross-region edges included).  Reference band = max over the source pack reference skins that redo_skin_check calibrate already wrote
(<REDO>\\skin_ref\\*.skin.pc2, read only) + margin 0.10.

  python -I skin_all.py --rig woman calibrate        -> Build\\work\\anim-survey-20261109\\skin_ref_all_<rig>.json
  python -I skin_all.py --rig woman rescore          -> adds all-edge numbers to skin_<rig>.json (the confirm skins in skin\\<rig>\\)
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_metrics as SM  # noqa: E402
import survey_common as SC  # noqa: E402

RIG = SM.RIG
MARGIN = 0.10
REF_P = os.path.join(SC.WORK, 'skin_ref_all_%s.json' % RIG)
_MESH = {}


def mesh(skin_npz, W0):
    if skin_npz in _MESH:
        return _MESH[skin_npz]
    z = np.load(skin_npz)
    rest = z['rest'].astype(float)
    faces = z['faces']
    e = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), 1), axis=0)
    l0 = np.linalg.norm(rest[e[:, 0]] - rest[e[:, 1]], axis=1)
    k = l0 > 0.004
    e, l0 = e[k], l0[k]
    rg = SM.BB.rig_for_gate()
    J = np.stack([W0[rg.ix[n], :3, 3] for n in SM.AG.SKIN_REGIONS])
    reg = np.argmin(np.linalg.norm(rest[:, None, :] - J[None], axis=2), axis=1)
    er = np.minimum(reg[e[:, 0]], reg[e[:, 1]])          # an edge counts for both of its regions (taken as the lower index once)
    er2 = np.maximum(reg[e[:, 0]], reg[e[:, 1]])
    _MESH[skin_npz] = (e, l0, er, er2, len(rest))
    return _MESH[skin_npz]


def stats_all(skin_base):
    names = SM.BB.rig()[0]
    W0 = SM.BB.inplace(np.load(os.path.join(SM.BB.REDO, 'base_pose.npz'))['W'], names)[0]
    e, l0, er, er2, nv = mesh(skin_base + '.npz', W0)
    pc = SM.AG._read_pc2(skin_base + '.pc2', nv)
    out = np.zeros((len(pc), len(SM.AG.SKIN_REGIONS)))
    for t in range(len(pc)):
        s = np.abs(np.log2(np.maximum(np.linalg.norm(pc[t][e[:, 0]] - pc[t][e[:, 1]], axis=1), 1e-6) / l0))
        for r in range(len(SM.AG.SKIN_REGIONS)):
            m = (er == r) | (er2 == r)
            out[t, r] = np.percentile(s[m], 98) if m.sum() > 30 else 0.0
    return out


def calibrate():
    per = {}
    for p in sorted(glob.glob(os.path.join(SM.BB.REDO, 'skin_ref', '*.skin.pc2'))):
        k = os.path.basename(p)[:-len('.skin.pc2')]
        per[k] = stats_all(p[:-len('.pc2')]).max(0)
    mx = np.max(list(per.values()), 0)
    d = dict(margin=MARGIN, regions=dict(zip(SM.AG.SKIN_REGIONS, [round(float(x), 3) for x in mx])),
             per_reference={k: dict(zip(SM.AG.SKIN_REGIONS, [round(float(x), 3) for x in v])) for k, v in per.items()},
             source='stats over %s\\skin_ref (read only)' % SM.BB.REDO)
    SC.jdump(d, REF_P)
    print('WROTE', REF_P, d['regions'])


def excess_all(st):
    ref = json.load(open(REF_P, encoding='utf-8'))
    allow = np.array([ref['regions'][r] for r in SM.AG.SKIN_REGIONS]) + ref['margin']
    ex = st - allow[None]
    ex[:, [r.endswith('Hand') for r in SM.AG.SKIN_REGIONS]] = -1.0
    return ex


def rescore():
    sp = os.path.join(SC.WORK, 'skin_%s.json' % RIG)
    sk = json.load(open(sp, encoding='utf-8'))
    for k, v in sk.items():
        base = os.path.join(SC.WORK, 'skin', RIG, 'c__' + k.replace('/', '__') + '.skin')
        if not os.path.exists(base + '.pc2'):
            continue
        ex = excess_all(stats_all(base))
        r = int(np.unravel_index(ex.argmax(), ex.shape)[1])
        v.update(all_excess_max=round(float(ex.max()), 3), all_region=SM.AG.SKIN_REGIONS[r],
                 all_regions_over=[SM.AG.SKIN_REGIONS[i] for i in range(ex.shape[1]) if ex[:, i].max() > 0])
        v['confirmed_within'] = v.get('confirmed_within', v.get('confirmed'))
        v['confirmed'] = bool(v['confirmed_within'] or ex.max() > 0)
    SC.jdump(sk, sp)
    print('rescored', len(sk), 'confirmed', sum(1 for v in sk.values() if v['confirmed']), 'within-only', sum(1 for v in sk.values() if v['confirmed_within']))


if __name__ == '__main__':
    if 'calibrate' in sys.argv:
        calibrate()
    if 'rescore' in sys.argv:
        rescore()
