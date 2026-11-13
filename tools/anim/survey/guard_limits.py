"""Per-joint LOCAL limits of the generic anatomy guard, in the guard's own representation (the rig's joint frames, rest-relative).

  python -I guard_limits.py --rig woman|man      -> Build\\work\\anim-survey-20261109\\guard_limits_<rig>.json
For every guarded joint j and every frame of the source pack reference corpus (base_body.RANGE_PATTERNS on the V rig of the body: idle,
walk, run, sprint, crouch, jump, traversal, turns, aim sweeps) plus every shipped overlay pose:
    Rel = Lrest_j^T * L_j                      (rest-relative local rotation, what the DLL sees per frame after the rest rebase)
    twist  tau = signed angle about local +X   (2 atan2(qx, qw), q with w >= 0)
    swing  s = Rel * twist^-1 -> rotation vector (0, sy, sz)   (q = swing * twist, swing axis in the local YZ plane)
    trans  |t_j - t_rest_j| (cm)               (local translation of the joint in its parent frame vs the rest local)
Limits = [p0.5 - tol, p99.5 + tol] per axis (tol 5 deg; legs: human margin 15 deg, knee flex up to 150 deg about the hinge),
translation tolerance = p99.5 + 0.5 cm.  The same table is the clamp of the offline prototype (guard_proto.py) and the DLL table.
Also checks that +X is the bone axis of every guarded joint (child rest translation direction).
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import survey_common as SC  # noqa: E402

RIG = sys.argv[sys.argv.index('--rig') + 1] if '--rig' in sys.argv else 'woman'
os.environ['UTPP_RIG'] = RIG
SC.add_tool_paths()
import anatomy_gate as AG  # noqa: E402
import base_body as BB  # noqa: E402

GUARD = ['Spine', 'Spine1', 'Spine2', 'Spine3', 'Neck', 'Neck1', 'Head', 'LeftShoulder', 'RightShoulder', 'LeftArm', 'RightArm',
         'LeftForeArm', 'RightForeArm', 'LeftHand', 'RightHand', 'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot']
LEGS = {'LeftUpLeg', 'RightUpLeg', 'LeftLeg', 'RightLeg', 'LeftFoot', 'RightFoot'}
TOL, TOL_LEG, TOL_T = 5.0, 15.0, 0.5


def decompose(Rel):
    """Rel [...,3,3] -> (twist deg, sy deg, sz deg)."""
    sh = Rel.shape[:-2]
    q = R.from_matrix(Rel.reshape(-1, 3, 3)).as_quat()
    q = np.where(q[:, 3:4] < 0, -q, q)
    tau = 2.0 * np.arctan2(q[:, 0], q[:, 3])
    tq = np.stack([np.sin(tau / 2), np.zeros_like(tau), np.zeros_like(tau), np.cos(tau / 2)], 1)
    s = (R.from_quat(q) * R.from_quat(tq).inv()).as_rotvec()
    tw = (np.degrees(tau) + 180.0) % 360.0 - 180.0
    return tw.reshape(sh), np.degrees(s[:, 1]).reshape(sh), np.degrees(s[:, 2]).reshape(sh), np.degrees(s[:, 0]).reshape(sh)


def compose(tw, sy, sz):
    """inverse of decompose -> Rel [...,3,3]."""
    sh = np.shape(tw)
    tw, sy, sz = (np.radians(np.ravel(x)) for x in (tw, sy, sz))
    t = R.from_rotvec(np.stack([tw, np.zeros_like(tw), np.zeros_like(tw)], 1))
    s = R.from_rotvec(np.stack([np.zeros_like(sy), sy, sz], 1))
    return (s * t).as_matrix().reshape(sh + (3, 3))


def locals_of(W, par):
    """W [T,N,4,4] -> local rotations [T,N,3,3], local translations [T,N,3]."""
    L = AG._local(W, par)
    P = W[:, :, :3, 3]
    lt = P.copy()
    for j, p in enumerate(par):
        if p >= 0:
            lt[:, j] = np.einsum('tji,tj->ti', W[:, p, :3, :3], P[:, j] - P[:, p])
    return L, lt


def main():
    names, par, ref, rest = BB.rig()
    rg = BB.rig_for_gate()
    ix = rg.ix
    files = []
    for pat in BB.RANGE_PATTERNS:
        files += glob.glob(os.path.join(BB.CLIP_DIR, pat + '.utppclip.json'))
    Ws = []
    for f in sorted(set(files)):
        W = BB.sourcepack_W(os.path.basename(f)[:-len('.utppclip.json')])
        if len(W) >= 5:
            Ws.append(W)
    for pn in BB.overlay_pose_names():
        try:
            Ws.append(BB.fkL(BB.overlay_L(pn))[None])
        except Exception:
            pass
    W = np.concatenate(Ws)
    L, lt = locals_of(W, rg.par)
    out = dict(rig=RIG, frames=int(len(W)), corpus_clips=len(Ws), tol_deg=TOL, tol_leg_deg=TOL_LEG, tol_trans_cm=TOL_T,
               representation='Rel = Lrest^T L (local, rest-relative); q = swing * twist; twist about local +X; swing rotvec (0, sy, sz); deg',
               joints={})
    for n in GUARD:
        j = ix[n]
        Rel = np.swapaxes(rg.Lrest[j], -1, -2)[None] @ L[:, j]
        tw, sy, sz, sx = decompose(Rel)
        dt = np.linalg.norm(lt[:, j] - rg.Lrest_t[j], axis=1) * 100
        kids = [k for k, p in enumerate(rg.par) if p == j and names[k] in names[:73]]
        axis = None
        if kids:
            c = max(kids, key=lambda k: np.linalg.norm(rg.Lrest_t[k]))
            v = rg.Lrest_t[c] / max(np.linalg.norm(rg.Lrest_t[c]), 1e-9)
            axis = dict(child=names[c], dir=[round(float(x), 3) for x in v])
        tol = TOL_LEG if n in LEGS else TOL
        lim = {}
        for k, v in (('twist', tw), ('swing_y', sy), ('swing_z', sz)):
            lo, hi = float(np.percentile(v, 0.5)), float(np.percentile(v, 99.5))
            lim[k] = [round(lo - tol, 1), round(hi + tol, 1)]
            lim[k + '_p'] = [round(lo, 1), round(hi, 1)]
        if n in ('LeftLeg', 'RightLeg'):          # knee hinge: flexion = -sz up to 150 deg (human), not the corpus maximum
            lim['swing_z'][0] = min(lim['swing_z'][0], -150.0)
        if n in ('LeftHand', 'RightHand'):       # game convention: forearm = pure hinge, the pronation roll sits in the HAND (ForearmTwist.hpp)
            lim['twist'] = [-130.0, 130.0]           # -> no corpus twist band on the hand, only the human pronation bound (anatomy_gate pronation 130)
        lim['trans_dev_cm'] = round(float(np.percentile(dt, 99.5)) + TOL_T, 2)
        lim['trans_dev_cm_p'] = round(float(np.percentile(dt, 99.5)), 2)
        lim['swing_x_residual_max'] = round(float(np.abs(sx).max()), 4)
        lim['bone_axis'] = axis
        lim['rest_local_t_cm'] = [round(float(x) * 100, 2) for x in rg.Lrest_t[j]]
        out['joints'][n] = lim
        print('%-13s tw %-14s sy %-14s sz %-14s trans<=%.2fcm axis %s' % (n, lim['twist'], lim['swing_y'], lim['swing_z'], lim['trans_dev_cm'],
                                                                     axis and (axis['child'], axis['dir'])))
    # shoulder width / clavicle gap in the Spine3 frame over the same corpus (survey_metrics uses them as the structural band)
    if '--rig' not in sys.argv:            # base_body strips the --rig switch from sys.argv at import; survey_metrics reads it again
        sys.argv += ['--rig', RIG]
    import survey_metrics as SM
    ctx = SM.Ctx.__new__(SM.Ctx)
    ctx.ix, ctx.rest = ix, np.asarray(rest, float)
    sw, gp = SM.Ctx.s3_width_gap(ctx, W)
    out['shoulder_width_s3'] = [round(float(np.percentile(sw, 0.5)) - 1.5, 2), round(float(np.percentile(sw, 99.5)) + 1.5, 2)]
    out['clav_gap_s3'] = [round(float(np.percentile(gp, 0.5)) - 1.5, 2), 1e9]
    print('shoulder_width_s3', out['shoulder_width_s3'], 'clav_gap_s3', out['clav_gap_s3'])
    SC.jdump(out, os.path.join(SC.WORK, 'guard_limits_%s.json' % RIG))
    print('WROTE guard_limits_%s.json from %d frames' % (RIG, len(W)))


if __name__ == '__main__':
    main()
