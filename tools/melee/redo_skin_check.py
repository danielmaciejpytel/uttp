"""Skinned-mesh check of the redo against the BASE body of V (not against the vanilla melee clip).

  python -I redo_skin_check.py calibrate [--rig man]   skins the base pose + source pack reference clips on the V mesh of the rig (woman default), writes <REDO>\\skin_ref.json
  python -I redo_skin_check.py check <clip_demo_dir> [name=redo] [--rig man]
Statistic (as anatomy_gate.skin_check): p98 over the edges of a region of |log2(edge length / bind edge length)|, region = nearest joint of the bind
vertex (shoulders, arms, forearms, hands, thighs, shins, neck, chest).  Allowed per region = max over the reference set (base ready pose, base idle pose,
source pack clips with raised arms / overhead reach / run / sprint / idle) + SKIN_MARGIN, so a legitimate raised arm is never flagged and a collapsed shoulder is.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import anatomy_gate as AG  # noqa: E402
import base_body as BB  # noqa: E402
import demo_skin  # noqa: E402
import melee_common as MC  # noqa: E402

SKIN_MARGIN = 0.10
REF_CLIPS = ['M_Neutral_Stand_Idle_Loop', 'M_Neutral_Run_Loop_F', 'M_Neutral_Sprint_Loop_F', 'AO_Pistol2H_Stand_Sweep', 'AO_Rifle_Stand_Sweep',
             'M_Neutral_Traversal_Catch_Hurdle_high_stand', 'M_Neutral_Traversal_Catch_Cliff_high_stand', 'M_Neutral_Crouch_Idle_Loop']
OUT = os.path.join(BB.REDO, 'skin_ref')


class Mesh:
    def __init__(self, skin_npz, rg, W0):
        z = np.load(skin_npz)
        self.rest = z['rest'].astype(float)
        faces = z['faces']
        e = np.unique(np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), 1), axis=0)
        l0 = np.linalg.norm(self.rest[e[:, 0]] - self.rest[e[:, 1]], axis=1)
        k = l0 > 0.004
        self.e, self.l0 = e[k], l0[k]
        J = np.stack([W0[rg.ix[n], :3, 3] for n in AG.SKIN_REGIONS])
        reg = np.argmin(np.linalg.norm(self.rest[:, None, :] - J[None], axis=2), axis=1)
        self.er = reg[self.e[:, 0]]
        self.same = self.er == reg[self.e[:, 1]]

    def stat(self, p):
        l = np.linalg.norm(p[self.e[:, 0]] - p[self.e[:, 1]], axis=1)
        s = np.abs(np.log2(np.maximum(l, 1e-6) / self.l0))
        return np.array([np.percentile(s[self.same & (self.er == r)], 98) if (self.same & (self.er == r)).sum() > 30 else 0.0
                         for r in range(len(AG.SKIN_REGIONS))])


def stats(skin_base, rg, W0):
    m = Mesh(skin_base + '.npz', rg, W0)
    pc = AG._read_pc2(skin_base + '.pc2', len(m.rest))
    return np.stack([m.stat(pc[t]) for t in range(len(pc))])


def calibrate():
    os.makedirs(OUT, exist_ok=True)
    rg = BB.rig_for_gate()
    names, par, ref, rest = BB.rig()
    z = np.load(os.path.join(BB.REDO, 'base_pose.npz'))
    srcs = {'base_ready': z['W'], 'base_idle': z['Wi']}
    for c in REF_CLIPS:
        try:
            srcs[c] = BB.sourcepack_W(c)
        except Exception:
            continue
    # all shipped overlay poses (V as she looks in game) as ONE 'clip': one frame per pose
    srcs['overlay_poses'] = np.concatenate([BB.fkL(BB.overlay_L(pn))[None] for pn in BB.overlay_pose_names()])
    allv, strict, per = [], [], {}
    W0 = BB.inplace(z['W'], names)[0]
    for k, W in srcs.items():
        if len(W) == 1:
            W = np.repeat(W, 3, axis=0)
        p = os.path.join(OUT, k + '.dump.npz')
        np.savez(p, W=W, rest=rest, names=np.array(names), parents=np.array(par), fps=30)
        demo_skin.skin(p, os.path.join(OUT, k + '.skin'), BB.RIG_NAME, k)
        st = stats(os.path.join(OUT, k + '.skin'), rg, W0)
        per[k] = dict(zip(AG.SKIN_REGIONS, [round(float(x), 3) for x in st.max(0)]))
        allv.append(st.max(0))
        if not k.startswith('AO_'):
            strict.append(st.max(0))
    mx = np.max(allv, 0)
    mx_s = np.max(strict, 0)
    ref_d = dict(margin=SKIN_MARGIN, regions=dict(zip(AG.SKIN_REGIONS, [round(float(x), 3) for x in mx])),
                 regions_strict=dict(zip(AG.SKIN_REGIONS, [round(float(x), 3) for x in mx_s])), per_reference=per)
    MC.jdump(ref_d, os.path.join(BB.REDO, 'skin_ref.json'))
    print(json.dumps(ref_d['regions']))
    return ref_d


def check(d, name='redo'):
    rg = BB.rig_for_gate()
    ref = json.load(open(os.path.join(BB.REDO, 'skin_ref.json')))
    W0 = np.load(os.path.join(BB.REDO, 'base_pose.npz'))['W']
    W0 = BB.inplace(W0, BB.rig()[0])[0]
    st = stats(os.path.join(d, name + '.skin'), rg, W0)
    allow = np.array([ref['regions'][r] for r in AG.SKIN_REGIONS]) + ref['margin']
    hand = np.array([r.endswith('Hand') for r in AG.SKIN_REGIONS])      # fingers curl (fists / grips of the game): not a shoulder / arm deformation
    st = st.copy()
    ex = st - allow[None]
    ex[:, hand] = -1.0
    worst = []
    for r in np.argsort(-ex.max(0))[:4]:
        t = int(ex[:, r].argmax())
        worst.append((AG.SKIN_REGIONS[r], t, round(float(ex[t, r]), 3), round(float(st[t, r]), 3)))
    allow_s = np.array([ref['regions_strict'][r] for r in AG.SKIN_REGIONS]) + ref['margin']
    exs = st - allow_s[None]
    exs[:, hand] = -1.0
    ws = []
    for r in np.argsort(-exs.max(0))[:3]:
        t = int(exs[:, r].argmax())
        ws.append((AG.SKIN_REGIONS[r], t, round(float(exs[t, r]), 3)))
    return dict(passed=bool(ex.max() <= 0), excess_max=round(float(ex.max()), 3), worst=worst,
                strict_excess_max=round(float(exs.max()), 3), strict_worst=ws, strict_frames_over=int((exs.max(1) > 0).sum()),
                frames_over=int((ex.max(1) > 0).sum()), max_stat=round(float(st.max()), 3))


if __name__ == '__main__':
    if sys.argv[1] == 'calibrate':
        calibrate()
    else:
        print(check(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else 'redo'))
