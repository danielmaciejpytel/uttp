"""Does the hinge IK move the upper-arm / forearm / hand locals outside the guard's own limits (no re-clamp after IK)?
Reads proto__<tag>.npz (W0 vanilla, W2 guard at the skin frames) of the prototype; prints the max excess beyond the limits."""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
rig = sys.argv[sys.argv.index('--rig') + 1]
import survey_common as SC  # noqa: E402
import guard_limits as GL  # noqa: E402
import base_body as BB  # noqa: E402

rg = BB.rig_for_gate()
W_ = os.path.join(SC.ROOT, 'Build', 'work', 'anim-survey-20261109')
lim = json.load(open(os.path.join(W_, 'guard_limits_%s.json' % rig), encoding='utf-8'))['joints']
pr = json.load(open(os.path.join(W_, 'proto_%s.json' % rig), encoding='utf-8'))
for k, v in pr.items():
    z = np.load(os.path.join(W_, 'skin', rig, 'proto__' + k.replace('/', '__') + '.npz'))
    out = []
    for tag in ('W0', 'W2'):
        L, _ = GL.locals_of(z[tag].astype(float), rg.par)
        worst = (0.0, '')
        for n in ('LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftUpLeg', 'RightUpLeg'):
            j = rg.ix[n]
            tw, sy, sz, _ = GL.decompose(np.swapaxes(rg.Lrest[j], -1, -2)[None] @ L[:, j])
            for x, a in ((tw, 'twist'), (sy, 'swing_y'), (sz, 'swing_z')):
                lo, hi = lim[n][a]
                e = float(np.max(np.maximum(lo - x, x - hi)))
                if e > worst[0]:
                    worst = (e, '%s.%s' % (n, a))
        out.append('%s %5.1f %-18s' % (tag, worst[0], worst[1]))
    print('%-5s %-62s %s' % (v['verdict'], k[:62], ' | '.join(out)))
