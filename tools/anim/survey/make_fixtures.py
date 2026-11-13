"""Stage 7: test fixtures for the C++ (DLL) anatomy guard, from the offline survey (numpy, read only on the dumps).

  python -I make_fixtures.py --rig woman [--n 10] [--per-state 2]
-> Build\\work\\anim-survey-20261109\\fixtures\\<rig>\\NN__<setkey>__<clip>.json  and  fixtures\\<rig>\\index.json

Selection: visible clips (survey class A / B / E) without a degenerate export and without glitch frames, ranked by survey severity
(survey_metrics), one clip per set and at most --per-state clips per triage state (variety: car seat, reload, bike, carry, ...), top --n.

Per fixture: the worst frame of the clip (survey_metrics worst_frame) and its +-1 neighbours of the pose SHOWN in TPP (class B: the
hybrid of survey_metrics.Ctx.shown = MM base pose on the MM-owned joints, game pose on the game-owned ones), as LOCAL poses of the
73-joint rig, game axes, metres, quaternions x y z w:
  joints[j]           name, parent index (-1 = root), rest local t / q
  frames[i].vanilla   local t / q per joint (the input the guard sees)
  frames[i].clamp     after the soft clamp + translation reset (guard_proto.guard, no IK)
  frames[i].guard     final guard output (hinge IK restores the hands (+ feet for A / E full-body clips with clamped legs), wrist clamp)
  frames[i].decomp    per guarded joint [twist, swing_y, swing_z] deg of the vanilla pose: 'wrapped' (+-180) and 'unwrapped' (np.unwrap
                      from clip frame 0 = what the prototype clamps; the DLL equivalent is a per-joint previous-frame state)
  frames[i].excess_vanilla / excess_guard   survey gate excess of the failing features (deg / cm / %)
  frames[i].hands_cm  world hand positions vanilla vs guard (cm, root space of the dump) and the residual (2 cm contact rule)
The guard runs over the whole clip (continuity state, hinge side latched at frame 0, pole blending) and is sliced afterwards, so a C++
port that starts at the fixture frame without that state can differ on twist branches; compare clamp output by the property
'every guarded joint inside its limits, identity inside [lo + K, hi - K]' and the guard output within tolerance.
index.json carries the limits table (guard_limits_<rig>.json joints), the constants and the selection.
"""
import json
import os
import re
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ARGS = list(sys.argv)                    # base_body strips --rig from sys.argv at import
import guard_proto as GP  # noqa: E402   (imports survey_metrics first: RIG from --rig)
import guard_limits as GL  # noqa: E402
import survey_metrics as SM  # noqa: E402
import survey_common as SC  # noqa: E402

RIG = SM.RIG
N = int(ARGS[ARGS.index('--n') + 1]) if '--n' in ARGS else 10
PER_STATE = int(ARGS[ARGS.index('--per-state') + 1]) if '--per-state' in ARGS else 2
STATES = re.compile(ARGS[ARGS.index('--states') + 1]) if '--states' in ARGS else None   # regex on the triage state (e.g. guard-applied states)
SUB = ARGS[ARGS.index('--sub') + 1] if '--sub' in ARGS else ''                            # fixtures\<rig>\<sub>\
OUT = os.path.join(SC.WORK, 'fixtures', RIG, SUB)


def quats(L):
    return R.from_matrix(L.reshape(-1, 3, 3)).as_quat().reshape(L.shape[:-2] + (4,))


def rl(x, n=6):
    return [round(float(v), n) for v in np.ravel(x)]


def select(met):
    rows = sorted(((m['severity'], k) for k, m in met.items() if m['cls'] in ('A', 'B', 'E') and m.get('top_frames')
                   and not m.get('degenerate_export') and not m.get('glitch_frames') and (STATES is None or STATES.search(m['state']))),
                  reverse=True)
    sets, states, keys = set(), {}, []
    for _, k in rows:
        m = met[k]
        if m['setkey'] in sets or states.get(m['state'], 0) >= PER_STATE:
            continue
        sets.add(m['setkey'])
        states[m['state']] = states.get(m['state'], 0) + 1
        keys.append(k)
        if len(keys) >= N:
            break
    return keys


def pose_block(L, t, g):
    return dict(t=[rl(x) for x in t[g]], q=[rl(x) for x in quats(L[g])])


def main():
    met = json.load(open(os.path.join(SC.WORK, 'metrics_%s.json' % RIG), encoding='utf-8'))
    ctx = SM.Ctx()
    rg, ix = ctx.rg, ctx.ix
    os.makedirs(OUT, exist_ok=True)
    keys = select(met)
    joints = [dict(name=n, parent=int(p), rest_t=rl(rg.Lrest_t[j]), rest_q=rl(quats(rg.Lrest[j][None])[0]))
              for j, (n, p) in enumerate(zip(ctx.names, rg.par))]
    index = dict(rig=RIG, joints_order=ctx.names, units='metres, degrees; quaternions x y z w; game axes (Z up), local = parent space',
                 representation='Rel = Lrest^T L; q = swing * twist; twist about local +X; swing rotvec (0, swing_y, swing_z)',
                 constants=dict(K_DEG=GP.K_DEG, trans_knee='trans_dev_cm / 2', HAND_GO_CM=GP.HAND_GO_CM, POP_GO_DEG=GP.POP_GO_DEG,
                                ik='analytic hinge (elbow / knee about local Z, flex = -swing_z <= 0), hinge side latched at frame 0, '
                                   'pole blended with the previous frame w = min(1, |pole| / 0.05 m)^2, hand world rotation restored, '
                                   'then wrist soft clamp'),
                 selection='visible A/B/E, no degenerate export, no glitch frames, by severity, 1 clip per set, <= %d per state, top %d%s'
                           % (PER_STATE, N, ', states ~ /%s/' % STATES.pattern if STATES else ''),
                 limits=GP.LIMITS, fixtures=[])
    for i, k in enumerate(keys):
        m = met[k]
        W0 = GP.load_shown(ctx, k, m)
        feet = m['cls'] in ('A', 'E') and m['legs'] and m['mode'] == 'full'
        Wc, W2, ncl = GP.guard(W0, ctx, feet=feet)
        T = len(W0)
        f = int(m['worst_frame'])
        fr = [g for g in (f - 1, f, f + 1) if 0 <= g < T]
        L0, t0 = GL.locals_of(W0, rg.par)
        Lc, tc = GL.locals_of(Wc, rg.par)
        L2, t2 = GL.locals_of(W2, rg.par)
        _, e0 = ctx.measure(W0, legs=m['legs'])
        _, e2 = ctx.measure(W2, legs=m['legs'])
        dec = {}
        for n in GP.LIMITS:
            j = ix[n]
            tw, sy, sz, _ = GL.decompose(np.swapaxes(rg.Lrest[j], -1, -2)[None] @ L0[:, j])
            dec[n] = ((tw, sy, sz), [np.degrees(np.unwrap(np.radians(x))) for x in (tw, sy, sz)])
        frames = []
        for g in fr:
            hands = {}
            for S in ('Left', 'Right'):
                a, b = W0[g, ix[S + 'Hand'], :3, 3] * 100, W2[g, ix[S + 'Hand'], :3, 3] * 100
                hands[S] = dict(vanilla=rl(a, 2), guard=rl(b, 2), residual=round(float(np.linalg.norm(a - b)), 2))
            frames.append(dict(frame=g, vanilla=pose_block(L0, t0, g), clamp=pose_block(Lc, tc, g), guard=pose_block(L2, t2, g),
                               decomp={n: dict(wrapped=rl([x[g] for x in w], 3), unwrapped=rl([x[g] for x in u], 3)) for n, (w, u) in dec.items()},
                               excess_vanilla={kk: round(float(v[g]), 3) for kk, v in e0.items() if v[g] > 0},
                               excess_guard={kk: round(float(v[g]), 3) for kk, v in e2.items() if v[g] > 0},
                               hands_cm=hands))
        name = '%02d__%s__%s.json' % (i + 1, m['setkey'], m['clip'])
        fx = dict(rig=RIG, key=k, set=m['set'], setkey=m['setkey'], clip=m['clip'], cls=m['cls'], state=m['state'], mode=m['mode'],
                  legs=m['legs'], feet_ik=feet, clip_frames=T, fps=SC.FPS, worst_frame=f, severity=m['severity'], fail_pct=m['fail_pct'],
                  worst_feature=m['worst_feature'], worst_excess=m['worst_excess'], clamped=ncl, joints=joints, frames=frames)
        with open(os.path.join(OUT, name), 'w', encoding='utf-8') as fh:
            json.dump(fx, fh, separators=(',', ':'))
            fh.write('\n')
        res = max(h['residual'] for fr_ in frames for h in fr_['hands_cm'].values())
        index['fixtures'].append(dict(file=name, key=k, cls=m['cls'], state=m['state'], mode=m['mode'], worst_frame=f,
                                      severity=m['severity'], worst_feature=m['worst_feature'], hand_residual_cm=res))
        print('FIXTURE %s %s %s sev %.2f worst %s hand %.2f cm' % (name, m['cls'], m['state'], m['severity'], m['worst_feature'], res))
    SC.jdump(index, os.path.join(OUT, 'index.json'))
    print('WROTE', OUT, len(keys), 'fixtures')


if __name__ == '__main__':
    main()
