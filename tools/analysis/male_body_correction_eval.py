"""Offline evaluation of the male upper-body correction (DLL: src/mm/MaleBodyCorrection.hpp) with the T4 deformation measurement.

python -I male_body_correction_eval.py measure --body man|woman --out <json> [--correction <corr.json>] [--no-forearm-twist] [--clips-suffix _knee]
python -I male_body_correction_eval.py dump --body man --out <dir> --cases label:clip:frame:family,... --correction <corr.json> [--tag before|after]

Same pipeline as measure_body_deformation.py (clip core pose -> deformation rig FK -> game deformer graph -> LBS, REF-relative edge metrics);
only the core pose is modified before skinning:
  1. runtime pass MoveForearmTwistToHands (python port of src/mm/ForearmTwist.hpp; the runtime runs it for both bodies, the raw clips do not have it)
  2. optional male correction: per family a set of WORLD-PRESERVING rotations  bone.r <- bone.r * D  and for every child c: c.r <- D^-1 c.r, c.t <- D^-1 c.t
     (same math as MaleBodyCorrection.hpp: hands, weapons, head and feet model transforms are unchanged).
Corrections JSON: {"families": {name: {"spine3": [rx,ry,rz deg], "clavL": [...], "clavR": [...], "rollL": deg, "rollR": deg, "knee..": ...}},
                   "clip_family": {clip: family}, "aim_gate": {family: [relax, ready, aim]}, "strength": 1.0}
Also reports sev_p999 (as measure_body_deformation), S = sum over region edges of max(0,|ln r|-ln 1.3)^2 and the count of extreme edges (r>1.5 or <0.5).
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Source', 'tools', 'analysis'))
import measure_body_deformation as MB  # noqa: E402

SK = MB.SK


def rotmat(rv_deg):
    rv = np.asarray(rv_deg, float)
    a = np.radians(np.linalg.norm(rv))
    if a < 1e-12:
        return np.eye(3)
    x, y, z = rv / np.linalg.norm(rv)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def children(rig, name):
    i = rig.idx[name]
    return [rig.names[k] for k, p in enumerate(rig.par) if p == i]


def rot_keep_children(rig, core, bone, D):
    """bone.r <- bone.r D; children c.r <- D^T c.r, c.t <- D^T c.t: the model transform of everything below the bone is unchanged."""
    core = dict(core)
    L = core.get(bone, rig.ref[rig.idx[bone]]).copy()
    L[:3, :3] = L[:3, :3] @ D
    core[bone] = L
    for c in children(rig, bone):
        Lc = core.get(c, rig.ref[rig.idx[c]]).copy()
        Lc[:3, :3] = D.T @ Lc[:3, :3]
        Lc[:3, 3] = D.T @ Lc[:3, 3]
        core[c] = Lc
    return core


def forearm_twist(rig, core):
    """Python port of MoveForearmTwistToHand: swing-twist of the forearm about the forearm->hand axis, twist moved into the hand."""
    core = dict(core)
    for fa, hand in (('LeftForeArm', 'LeftHand'), ('RightForeArm', 'RightHand')):
        if fa not in core or hand not in core:
            continue
        ht = core[hand][:3, 3]
        ln = np.linalg.norm(ht)
        if ln < 1e-5:
            continue
        a = ht / ln
        R = core[fa][:3, :3]
        q = rot_to_quat(R)
        d = float(q[:3] @ a)
        tw = np.array([a[0] * d, a[1] * d, a[2] * d, q[3]])
        tn = np.linalg.norm(tw)
        if tn < 1e-6:
            continue
        tw /= tn
        if tw[3] < 0:
            tw = -tw
        Rt = quat_to_rot(tw)
        L = core[fa].copy()
        L[:3, :3] = R @ Rt.T
        core[fa] = L
        for c in children(rig, fa):
            if c not in core:
                continue
            Lc = core[c].copy()
            Lc[:3, :3] = Rt @ Lc[:3, :3]
            if c != hand:
                Lc[:3, 3] = Rt @ Lc[:3, 3]
            core[c] = Lc
    return core


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def scale_rv(v, s):
    return np.asarray(v, float) * s


def apply_family(rig, core, fam, strength=1.0):
    """fam: spine3 / neck / clavL / clavR (rv deg), rollL / rollR (deg about the upper-arm axis X), shinL / shinR, thighL / thighR (roll about X)."""
    if not fam or strength <= 0:
        return core
    c = core
    if 'spine3' in fam:
        c = rot_keep_children(rig, c, 'Spine3', rotmat(scale_rv(fam['spine3'], strength)))
    if 'neck' in fam:
        c = rot_keep_children(rig, c, 'Neck', rotmat(scale_rv(fam['neck'], strength)))
    for side, cl, arm in (('L', 'LeftShoulder', 'LeftArm'), ('R', 'RightShoulder', 'RightArm')):
        if 'clav' + side in fam:
            c = rot_keep_children(rig, c, cl, rotmat(scale_rv(fam['clav' + side], strength)))
        if fam.get('roll' + side, 0):
            c = rot_keep_children(rig, c, arm, rotmat([fam['roll' + side] * strength, 0, 0]))
    for side, th, sh in (('L', 'LeftUpLeg', 'LeftLeg'), ('R', 'RightUpLeg', 'RightLeg')):
        if fam.get('thigh' + side, 0):
            c = rot_keep_children(rig, c, th, rotmat([fam['thigh' + side] * strength, 0, 0]))
        if fam.get('shin' + side, 0):
            c = rot_keep_children(rig, c, sh, rotmat([fam['shin' + side] * strength, 0, 0]))
    return c


def aim_level(clip):
    """aim state of the pose key the clip belongs to (0 relaxed, 1 ready, 2 aiming; locomotion / sweeps = 2)."""
    if clip.startswith('AO_'):
        return 2
    if 'Relax' in clip:
        return 0
    if 'Ready' in clip:
        return 1
    return 2


def strength_for(corr, clip):
    fam = corr.get('clip_family', {}).get(clip)
    g = corr.get('aim_gate', {}).get(fam, [1, 1, 1])
    return corr.get('strength', 1.0) * g[aim_level(clip)]


def extra_metrics(rig, r):
    """per region: S (smooth cost), extreme edge count (ratio > 1.5 or < 0.5), sev; 'rest' = every vertex outside the four regions."""
    out = {}
    anyreg = np.zeros(len(rig.pos), bool)
    for reg in ('shoulder', 'elbow', 'neck', 'knee'):
        anyreg |= rig.region[reg]
    for reg in ('shoulder', 'elbow', 'neck', 'knee', 'rest'):
        m = rig.region[reg] if reg != 'rest' else ~anyreg
        em = m[rig.edges[:, 0]] & m[rig.edges[:, 1]]
        rr = np.clip(r[em], 1e-3, 1e3)
        lr = np.abs(np.log(rr))
        out[reg] = dict(S=float((np.maximum(0, lr - np.log(1.3)) ** 2).sum()), extreme=int(((rr > 1.5) | (rr < 0.5)).sum()),
                        sev_p999=float(max(np.percentile(rr, 99.9), 1.0 / max(np.percentile(rr, 0.1), 1e-6))))
    return out


def run_measure(body, out, corr, forearm=True, suffix='', offset=0):
    MB.RETARGET[body] = MB.RETARGET[body] + suffix
    rig = MB.Rig(body)
    refv, refW = rig.skin(rig.ref_core())
    _, _, refstate = rig.metrics(refv, refW, refW)
    cf = (corr or {}).get('clip_family', {})
    fams = (corr or {}).get('families', {})
    res = dict(body=body, correction=bool(corr), forearm_twist=forearm, poses=[])
    for label, clip, sel in MB.POSES:
        path = os.path.join(MB.RETARGET[body], clip + '.utppclip.json')
        cores = SK.clip_cores(json.load(open(path, encoding='utf-8')), set(rig.names))
        fam = fams.get(cf.get(clip, ''), None) if corr else None
        per = []
        for f in sorted({(f0 + offset) % len(cores) for f0 in MB.frames_for(sel, len(cores))} if sel != 'all' else MB.frames_for(sel, len(cores))):
            core = cores[f]
            if forearm:
                core = forearm_twist(rig, core)
            if fam:
                core = apply_family(rig, core, fam, strength_for(corr, clip))
            v, W = rig.skin(core)
            m, r, _ = rig.metrics(v, W, refW, refstate)
            for k, e in extra_metrics(rig, r).items():
                m.setdefault(k, {}).update(e)
            per.append(dict(frame=f, m=m))
        worst = {}
        for reg in list(MB.REGIONS) + ['rest']:
            b = max(per, key=lambda x: x['m'][reg]['sev_p999'])
            worst[reg] = dict(frame=b['frame'], **b['m'][reg])
        mean = {reg: dict(sev=float(np.mean([x['m'][reg]['sev_p999'] for x in per])), S=float(np.mean([x['m'][reg]['S'] for x in per])),
                          extreme=float(np.mean([x['m'][reg]['extreme'] for x in per]))) for reg in list(MB.REGIONS) + ['rest']}
        res['poses'].append(dict(label=label, clip=clip, frames=len(per), worst=worst, mean=mean, family=cf.get(clip)))
        print(body, label, ' '.join('%s=%.2f@%d' % (k, v['sev_p999'], v['frame']) for k, v in worst.items()), flush=True)
    json.dump(res, open(out, 'w'), indent=1)


def run_dump(body, outdir, corr, cases, tag):
    rig = MB.Rig(body)
    refv, refW = rig.skin(rig.ref_core())
    _, _, refstate = rig.metrics(refv, refW, refW)
    os.makedirs(outdir, exist_ok=True)
    for spec in cases.split(','):
        label, clip, fr, fam = spec.split(':')
        core = SK.clip_cores(json.load(open(os.path.join(MB.RETARGET[body], clip + '.utppclip.json'), encoding='utf-8')), set(rig.names))[int(fr)]
        core = forearm_twist(rig, core)
        if fam != 'none':
            core = apply_family(rig, core, corr['families'][fam], strength_for(dict(corr, clip_family={clip: fam}), clip))
        v, W = rig.skin(core)
        m, r, _ = rig.metrics(v, W, refW, refstate)
        st = rig.vertex_stretch(r)
        np.savez(os.path.join(outdir, '%s_%s.npz' % (tag, label)), verts=np.stack([v[:, 0], -v[:, 2], v[:, 1]], 1).astype(np.float32),
                 faces=rig.faces.astype(np.int32), stretch=st.astype(np.float32),
                 bind=np.stack([rig.pos[:, 0], -rig.pos[:, 2], rig.pos[:, 1]], 1).astype(np.float32),
                 **{'mask_' + k: x for k, x in rig.region.items()},
                 joints=np.array([W[n][:3, 3] for n in SK.CHECK]), joint_names=np.array(SK.CHECK), domname=rig.domname.astype(str),
                 part_of=rig.part_of, edges=rig.edges.astype(np.int32), edge_ratio=r.astype(np.float32), metrics=json.dumps(m))
        print('dumped', tag, label, clip, fr, fam, {k: round(m[k]['sev_p999'], 2) for k in MB.REGIONS})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['measure', 'dump'])
    ap.add_argument('--body', default='man', choices=['man', 'woman'])
    ap.add_argument('--out', required=True)
    ap.add_argument('--correction')
    ap.add_argument('--no-forearm-twist', action='store_true')
    ap.add_argument('--clips-suffix', default='')
    ap.add_argument('--frame-offset', type=int, default=0, help='shift of the sampled loop / sweep frames (held-out check: frames the fit never saw)')
    ap.add_argument('--cases')
    ap.add_argument('--tag', default='man')
    a = ap.parse_args()
    corr = json.load(open(a.correction, encoding='utf-8')) if a.correction else None
    if a.mode == 'measure':
        run_measure(a.body, a.out, corr, not a.no_forearm_twist, a.clips_suffix, a.frame_offset)
    else:
        run_dump(a.body, a.out, corr, a.cases, a.tag)


if __name__ == '__main__':
    main()
