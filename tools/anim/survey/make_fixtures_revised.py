"""Test fixtures for a C++ (DLL) port of the anti-deformation guard v2 (STATEFUL: per-frame state, ramps) - offline, 2026-11-09.

  python -I make_fixtures_revised.py --rig woman|man
-> Build\\work\\anim-guard-v2-20261109\\fixtures\\<rig>\\NN__<setkey>__<clip>.json + index.json

Selection (from eval_<rig>_p*.json): per state with a v2 GO clip the first GO clip, plus the 2 NO-GO clips with the largest vanilla
skin excess (negative tests: the port must reproduce the same contact / pop behaviour, not a better one by accident).
Window: from frame a (fresh guard state = state entry) to the worst skinned frame + 2, at most 90 frames (a = 0 when it fits,
else worst - 60). The guard is re-run on the window itself, so a port that starts with a fresh state at window frame 0 must match.
Per fixture, LOCAL poses of the 73-joint rig (game axes, metres, quaternions x y z w):
  joints              name, parent, rest t / q;  guarded = joint names the mode may write
  frames[i].vanilla   input pose;  .steady = guard at w = 1;  .trans = TRANSITION run (w ramps 0 -> 1 from window frame 0)
  frames[i].diag      w, contact, gamma_need, per arm beta / beta_reach / phi / hand_cm / lim_after / lim_vanilla / wrist_clamp
  post[j]             TRANSITION run post-roll (last vanilla frame held, w -> 0): local pose + w
Tolerances for a port: hand position <= 0.5 cm vs the fixture, guarded joint rotations <= 2 deg (swivel search resolution 7.5 deg
then 2.5 deg fine pass; a finer C++ search may differ by a few deg in swivel - compare limits / hands first, then rotations).
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ARGS = list(sys.argv)
import guard_eval as GE  # noqa: E402   (imports guard_proto_v2 -> survey_metrics: RIG from --rig)
import guard_proto_revised as G2  # noqa: E402
import guard_proto as GP  # noqa: E402
import guard_limits as GL  # noqa: E402
import survey_metrics as SM  # noqa: E402
import survey_common as SC  # noqa: E402

RIG = G2.RIG
OUTF = os.path.join(G2.OUT, 'fixtures', RIG)
DIAG = ('beta', 'beta_reach', 'phi', 'hand_cm', 'lim_after', 'lim_vanilla', 'wrist_clamp', 'kept_cm', 'delta_deg')


def rl(x, n=6):
    return np.round(np.asarray(x, float), n).tolist()


def pose(L, t):
    return dict(t=rl(t), q=rl(R.from_matrix(L).as_quat()))


def main():
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    res = {}
    for f in sorted(glob.glob(os.path.join(G2.OUT, 'eval_%s_p*.json' % RIG))):
        res.update(json.load(open(f, encoding='utf-8')))
    ok = {k: r for k, r in res.items() if 'error' not in r and 'sheet_frame' in r}
    picks, seen = [], set()
    for k, r in sorted(ok.items(), key=lambda kr: (kr[1]['state_key'], kr[1]['frames'])):
        if GE.verdict(r, 'v2')[0] == 'GO' and r['state_key'] not in seen:
            picks.append((k, 'GO'))
            seen.add(r['state_key'])
    nogo = sorted([k for k, r in ok.items() if GE.verdict(r, 'v2')[0] == 'NO-GO'], key=lambda k: -ok[k]['skin_v'])[:2]
    picks += [(k, 'NO-GO') for k in nogo]
    ctx = SM.Ctx()
    names, par, _, _ = SM.BB.rig()
    rg = ctx.rg
    os.makedirs(OUTF, exist_ok=True)
    index = []
    for n_, (k, vd) in enumerate(picks, 1):
        m, r = met[k], ok[k]
        W0 = GP.load_shown(ctx, k, m)
        T = len(W0)
        wf = int(r['sheet_frame'])
        a = 0 if wf + 2 <= 90 else wf - 60
        b = min(T, a + 90, max(wf + 2, a + 2))
        Wn = W0[a:b]
        legs = m['mode'] == 'full' and bool(m['legs'])
        W2, dg = G2.run_clip(Wn, ctx, m['mode'], legs, ramp=False)
        W3, dg3 = G2.run_clip(Wn, ctx, m['mode'], legs, ramp=True)
        Lv, tv = GL.locals_of(Wn, rg.par)
        Lg, tg = GL.locals_of(W2, rg.par)
        Lt, tt = GL.locals_of(W3, rg.par)
        g = G2.GuardV2(ctx, m['mode'], legs)
        frames = []
        for i in range(len(Wn)):
            d = dg[i]
            dd = dict(w=round(float(dg3[i].get('w', 0.0)), 4), contact=bool(d.get('contact')), gamma_need=round(float(d.get('gamma_need', 0.0)), 3))
            for s in ('Left', 'Right'):
                dd[s] = {x: round(float(d.get(s + 'Arm.' + x, 0.0) or 0.0), 3) for x in DIAG}
            frames.append(dict(i=a + i, vanilla=pose(Lv[i], tv[i]), steady=pose(Lg[i], tg[i]), trans=pose(Lt[i], tt[i]), diag=dd))
        post = [dict(w=round(float(dg3[j].get('w', 0.0)), 4), **pose(Lt[j], tt[j])) for j in range(len(Wn), len(W3))]
        fx = dict(rig=RIG, key=k, cls=m['cls'], state=m['state'], mode=m['mode'], legs=legs, verdict_v2=vd, window=[a, b], worst_frame=wf,
                  joints=[dict(name=names[j], parent=int(par[j]), rest=pose(rg.Lrest[j], rg.Lrest_t[j])) for j in range(len(names))],
                  guarded=[names[j] for j in g.joints], frames=frames, post=post)
        name = '%02d__%s.json' % (n_, k.replace('/', '__'))
        with open(os.path.join(OUTF, name), 'w', encoding='utf-8') as fh:
            json.dump(fx, fh, separators=(',', ':'))
        index.append(dict(file=name, key=k, state=r['state_key'], verdict_v2=vd, window=[a, b], frames=b - a))
        print('fixture', name, vd, a, b, flush=True)
    SC.jdump(dict(rig=RIG, P=G2.P, limits={j: GP.LIMITS[j] for j in GP.LIMITS}, fixtures=index), os.path.join(OUTF, 'index.json'))
    print('WROTE', OUTF, len(index))


if __name__ == '__main__':
    main()
