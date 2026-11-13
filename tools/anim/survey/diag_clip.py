"""Diagnose one dumped clip: per-frame rot_step peak (which bone), local translation deviations, constant (untracked) joints.
  python -I diag_clip.py --rig woman <setkey>/<clip>"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_metrics as SM  # noqa: E402
import guard_limits as GL  # noqa: E402
import survey_common as SC  # noqa: E402
import melee_common as MC  # noqa: E402

key = [a for a in sys.argv[1:] if '/' in a][0]
sk, clip = key.split('/', 1)
d = MC.load_dump(os.path.join(SC.DUMPS, SM.RIG, sk, clip + '.npz'))
ctx = SM.Ctx()
W = d['W'].astype(float)
L, lt = GL.locals_of(W, ctx.rg.par)
T = len(W)
names = ctx.names
M = np.swapaxes(L[:-1], -1, -2) @ L[1:]
ang = np.degrees(np.arccos(np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1)))     # [T-1, 73]
t, j = np.unravel_index(ang.argmax(), ang.shape)
print('frames', T, 'max one-frame rot', round(float(ang.max()), 1), 'between', t, t + 1, 'bone', names[j])
big = [(int(a), names[b], round(float(ang[a, b]), 1)) for a, b in zip(*np.nonzero(ang > 30))][:12]
print('steps > 30 deg:', big)
dev = np.linalg.norm(lt - ctx.rg.Lrest_t[None], axis=2) * 100      # [T,73] cm
top = np.argsort(-dev.max(0))[:10]
print('largest local translation deviation (cm, max over frames):', [(names[i], round(float(dev[:, i].max()), 1)) for i in top])
const = [names[i] for i in range(73) if np.ptp(d['lr'][:, i], axis=0).max() < 1e-5 and np.ptp(d['lt'][:, i], axis=0).max() < 1e-6]
print('constant joints (%d):' % len(const), const[:40])
f, ex = ctx.measure(W)
for k in ('shoulder_width_s3', 'clav_gap_s3', 'spine_twist_Spine2', 'seg_len_dev_pct', 'clav_shift_L_cm', 'clav_shift_R_cm'):
    v = np.asarray(f[k])
    print('%-20s f0 %.1f  mid %.1f  min %.1f max %.1f allowed %s' % (k, v[0], v[T // 2], v.min(), v.max(), ctx.allow.get(k)))
print(json.dumps(d['meta'])[:300])
