"""Retarget the authored female Nue reload set (utpp_nue_tpp_reload.anims, player_woman_skeleton) onto
player_man_skeleton for male V.  Offline only; source-pack-derived data.

Facts measured on the two game rigs (152 joints, identical names + hierarchy): arms, hands, shoulders and WeaponRight
have identical local bind translations; only the hips/spine chain (and feet) differ, and the arm bones carry a 3.44 deg
twist bind difference (LeftArm/RightArm/ForeArm world bind rotation, hands equal).  Therefore:

  * every true bone: world rotation = female world rotation * (femaleBindWorld^-1 * maleBindWorld)
    (same skin matrix as the female clip), local translation = male bind + female delta from bind;
  * helper joints (IK targets, arm spaces, camera aims): if the female clip leaves them at their bind pose the male keeps
    its own bind pose, otherwise they keep the female MODEL-SPACE transform (hand/gun relations preserved exactly);
  * the Hips translation is shifted so that Spine3 lands on the female Spine3 position, so shoulders, arms, hands and
    WeaponRight keep the female model-space positions and every hand-to-weapon-to-magazine relation is unchanged
    (the magazine set sits in weapon space on the shared weapon_root.ent);
  * timeline, events, float tracks and sampling are copied from the female clip unchanged (81 / 101 frames, same phases);
  * the two dense 960 Hz gun-space controllers are re-derived at the female dense times against the male parent chain.

No game writes.  Usage:
  python -I retarget_nue_reload_man.py --female-glb F.glb --male-glb M.glb --out OUT.glb --report REPORT.json
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'Source' / 'tools' / 'anim' / 'vehicle-entry'))
import glbanim as G  # noqa: E402
from skel import Skeleton, q2m, m2q, qcontinuous  # noqa: E402

DENSE = ('arm_left_ik_offset', 'arm_right_ik_offset')
CLIPS = ('utpp_nue_reload_aim', 'utpp_nue_reload_ready', 'utpp_nue_reload_2h_aim', 'utpp_nue_reload_2h_ready',
         'utpp_nue_empty_reload_aim', 'utpp_nue_empty_reload_ready', 'utpp_nue_empty_reload_2h_aim',
         'utpp_nue_empty_reload_2h_ready')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def modes(F, M, lt, lr):
    """Per joint: 'bone' (skin-matrix retarget), 'rest' (helper at bind in the female clip), 'world' (helper moved)."""
    n = len(F.names)
    first_helper = F.index['Torso_COG_Control_JNT']
    dev = np.zeros(n)
    for j in range(n):
        dt = np.abs(lt[:, j] - F.rest_t[j]).max()
        dq = np.abs(np.abs(np.sum(lr[:, j] * F.rest_r[j], axis=-1)) - 1).max()
        dev[j] = max(dt, 10 * dq)
    out = []
    for j in range(n):
        if j < first_helper:
            out.append('bone')
        else:
            out.append('world' if dev[j] > 1e-6 else 'rest')
    return out, dev


def retarget(F, M, lt, lr, mode, Dm, Fw=None):
    """lt/lr (T,N,..) female local. Returns male local (T,N,3),(T,N,4), world pos (T,N,3), world rot (T,N,3,3)."""
    T, N = lt.shape[:2]
    Wtf, Wrf = Fw if Fw is not None else F.world(lt, lr)
    Wt = np.zeros((T, N, 3))
    Wr = np.zeros((T, N, 3, 3))
    lt_m = np.zeros((T, N, 3))
    lq_m = np.zeros((T, N, 4))
    Mr = q2m(M.rest_r)
    for j in range(N):
        p = F.parent[j]
        pt = Wt[:, p] if p >= 0 else np.zeros((T, 3))
        pr = Wr[:, p] if p >= 0 else np.broadcast_to(np.eye(3), (T, 3, 3))
        if mode[j] == 'bone':
            Wr[:, j] = Wrf[:, j] @ Dm[j]
            lt_m[:, j] = M.rest_t[j] + (lt[:, j] - F.rest_t[j])
            Wt[:, j] = pt + np.einsum('tij,tj->ti', pr, lt_m[:, j])
        elif mode[j] == 'rest':
            Wr[:, j] = pr @ Mr[j]
            lt_m[:, j] = M.rest_t[j]
            Wt[:, j] = pt + np.einsum('tij,tj->ti', pr, lt_m[:, j])
        else:
            Wr[:, j] = Wrf[:, j]
            Wt[:, j] = Wtf[:, j]
            lt_m[:, j] = np.einsum('tji,tj->ti', pr, Wt[:, j] - pt)  # pr^T (x - pt)
        lq_m[:, j] = m2q(np.transpose(pr, (0, 2, 1)) @ Wr[:, j])
    return lt_m, qcontinuous(lq_m), Wt, Wr


def rel(Wt, Wr, a, b):
    """Transform of b in the frame of a: (pos (T,3), rot (T,3,3))."""
    inv = np.transpose(Wr[:, a], (0, 2, 1))
    return np.einsum('tij,tj->ti', inv, Wt[:, b] - Wt[:, a]), inv @ Wr[:, b]


def ang(A, B):
    r = np.einsum('...ji,...jk->...ik', A, B)
    tr = r[..., 0, 0] + r[..., 1, 1] + r[..., 2, 2]
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--female-glb', required=True)
    ap.add_argument('--male-glb', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--report', required=True)
    a = ap.parse_args()
    F = Skeleton.from_glb(a.female_glb)
    M = Skeleton.from_glb(a.male_glb)
    assert F.names == M.names and F.parent == M.parent, 'rig topology differs'
    Fbt, Fbr = F.rest_world()
    Mbt, Mbr = M.rest_world()
    Dm = np.einsum('nji,njk->nik', Fbr, Mbr)  # Fbr^T @ Mbr
    s3 = F.index['Spine3']
    hips = F.index['Hips']
    ix = F.index
    gj = copy.deepcopy(M.gj)
    binb = bytearray(M.binb)
    new_anims = []
    report = dict(femaleGlbSha256=sha(a.female_glb), maleGlbSha256=sha(a.male_glb), clips={})

    def add_accessor(values, kind):
        arr = np.asarray(values, dtype='<f4')
        binb.extend(b'\0' * ((-len(binb)) % 4))
        off = len(binb)
        binb.extend(arr.tobytes())
        view = len(gj['bufferViews'])
        gj['bufferViews'].append(dict(buffer=0, byteOffset=off, byteLength=arr.nbytes))
        e = dict(bufferView=view, componentType=5126, count=len(arr), type=kind)
        if kind == 'SCALAR':
            e.update(min=[float(arr.min())], max=[float(arr.max())])
        gj['accessors'].append(e)
        return len(gj['accessors']) - 1

    for name in CLIPS:
        ci = F.clip(name)
        ch = F.clip_tracks(name)
        t_sp = ch['LeftHand']['rotation'][0]
        t_dn = ch[DENSE[0]]['rotation'][0]
        assert len(t_sp) in (81, 101) and len(t_dn) in (2561, 3201, 1601), (len(t_sp), len(t_dn))
        lt, lr = F.sample_local(ci, t_sp)
        mode, dev = modes(F, M, lt, lr)
        lt_m, lq_m, Wt, Wr = retarget(F, M, lt, lr, mode, Dm)
        Wtf, Wrf = F.world(lt, lr)
        # shift Hips so that Spine3 keeps the female model-space position
        shift = Wtf[:, s3] - Wt[:, s3]
        lt_adj = lt.copy()
        # hips parent is Root with identity transform: world shift == local shift
        assert F.parent[hips] == ix['Root'] and np.abs(Wtf[:, ix['Root']]).max() < 1e-9
        lt_adj[:, hips] = lt[:, hips] + shift
        lt_m, lq_m, Wt, Wr = retarget(F, M, lt_adj, lr, mode, Dm, Fw=(Wtf, Wrf))
        # --- dense gun-space controllers: against the male parent chain at the female dense times
        lt_d, lr_d = F.sample_local(ci, t_dn)
        Wtfd, Wrfd = F.world(lt_d, lr_d)
        # male sparse locals resampled at the dense times (engine: LINEAR / slerp)
        ltm_d = np.zeros_like(lt_d)
        lqm_d = np.zeros_like(lr_d)
        for j in range(len(F.names)):
            ltm_d[:, j] = G.sample((t_sp, lt_m[:, j], 'LINEAR'), t_dn, 'translation')
            lqm_d[:, j] = G.sample((t_sp, lq_m[:, j], 'LINEAR'), t_dn, 'rotation')
        Wtmd, Wrmd = M.world(ltm_d, lqm_d)
        dense_loc = {}
        for dn in DENSE:
            j = ix[dn]
            p = F.parent[j]
            pr = Wrmd[:, p]
            tl = np.einsum('tji,tj->ti', pr, Wtfd[:, j] - Wtmd[:, p])
            ql = qcontinuous(m2q(np.transpose(pr, (0, 2, 1)) @ Wrfd[:, j]))
            dense_loc[dn] = (tl, ql)
        # --- validation: male FK from the stored (float32) locals vs the female model space
        lq32 = lq_m.astype(np.float32).astype(np.float64)
        lt32 = lt_m.astype(np.float32).astype(np.float64)
        Wtm, Wrm = M.world(lt32, lq32)
        arm = ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm',
               'RightHand', 'WeaponRight', 'Spine3', 'Neck', 'Head'] + [n for n in F.names if 'Hand' in n and 'In' in n or 'Thumb' in n or 'Index' in n or 'Middle' in n or 'Ring' in n or 'Pinky' in n]
        pos_err = {n: float(np.linalg.norm(Wtm[:, ix[n]] - Wtf[:, ix[n]], axis=-1).max()) for n in arm}
        rot_err = {n: float(ang(Wrm[:, ix[n]], Wrf[:, ix[n]]).max()) for n in arm}
        helpers_world = [F.names[j] for j in range(len(F.names)) if mode[j] == 'world']
        hp = {n: float(np.linalg.norm(Wtm[:, ix[n]] - Wtf[:, ix[n]], axis=-1).max()) for n in helpers_world}
        rel_f = rel(Wtf, Wrf, ix['WeaponRight'], ix['LeftHand'])
        rel_m = rel(Wtm, Wrm, ix['WeaponRight'], ix['LeftHand'])
        lh_in_weapon_pos = float(np.linalg.norm(rel_f[0] - rel_m[0], axis=-1).max())
        lh_in_weapon_rot = float(ang(rel_f[1], rel_m[1]).max())
        relr_f = rel(Wtf, Wrf, ix['WeaponRight'], ix['RightHand'])
        relr_m = rel(Wtm, Wrm, ix['WeaponRight'], ix['RightHand'])
        rh_pos = float(np.linalg.norm(relr_f[0] - relr_m[0], axis=-1).max())
        # fingertip-ish: all finger joints in the left hand frame
        report['clips'][name] = dict(
            frames=len(t_sp), denseKeys=len(t_dn), duration=float(t_sp[-1]),
            modes={m: [F.names[j] for j in range(len(F.names)) if mode[j] == m] for m in ('rest', 'world')},
            hipsShiftMeters=[float(x) for x in shift[0]], hipsShiftConstant=bool(np.abs(shift - shift[0]).max() < 1e-9),
            maxPositionErrorVsFemaleMeters=max(pos_err.values()), worstPosJoint=max(pos_err, key=pos_err.get),
            maxRotationErrorVsFemaleDeg=max(rot_err.values()), worstRotJoint=max(rot_err, key=rot_err.get),
            leftHandInWeaponRightPosDiffMeters=lh_in_weapon_pos, leftHandInWeaponRightRotDiffDeg=lh_in_weapon_rot,
            rightHandInWeaponRightPosDiffMeters=rh_pos, helperWorldMaxErrorMeters=max(hp.values()) if hp else 0.0)
        # --- build the glTF animation (channels for all 152 joints, female key times; female extras verbatim)
        src_anim = F.gj['animations'][ci]
        anim = dict(name=name, extras=copy.deepcopy(src_anim['extras']), channels=[], samplers=[])
        in_sp = add_accessor(t_sp, 'SCALAR')
        in_dn = add_accessor(t_dn, 'SCALAR')
        node_of = M.node_of
        for j, jn in enumerate(F.names):
            dense = jn in dense_loc
            times_acc = in_dn if dense else in_sp
            tr = dense_loc[jn][0] if dense else lt_m[:, j]
            q = dense_loc[jn][1] if dense else lq_m[:, j]
            scale = np.ones((len(t_dn) if dense else len(t_sp), 3))
            for kind, values, typ in (('translation', tr, 'VEC3'), ('rotation', q, 'VEC4'), ('scale', scale, 'VEC3')):
                idx = add_accessor(values, typ)
                s = len(anim['samplers'])
                anim['samplers'].append(dict(input=times_acc, output=idx, interpolation='LINEAR'))
                anim['channels'].append(dict(sampler=s, target=dict(node=node_of[jn], path=kind)))
        new_anims.append(anim)
    gj['animations'] = new_anims
    gj['buffers'][0]['byteLength'] = len(binb)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    G.write_glb(str(out), gj, bytes(binb))
    report['output'] = str(out)
    report['outputSha256'] = sha(out)
    Path(a.report).write_text(json.dumps(report, indent=2) + '\n')
    for n, c in report['clips'].items():
        print(n, 'frames', c['frames'], 'maxPos %.6f (%s) maxRot %.4f (%s) LHinW %.6f/%.4f RHinW %.6f helpers %.6f hips shift %s const=%s' % (
            c['maxPositionErrorVsFemaleMeters'], c['worstPosJoint'], c['maxRotationErrorVsFemaleDeg'], c['worstRotJoint'],
            c['leftHandInWeaponRightPosDiffMeters'], c['leftHandInWeaponRightRotDiffDeg'], c['rightHandInWeaponRightPosDiffMeters'],
            c['helperWorldMaxErrorMeters'], np.round(c['hipsShiftMeters'], 4), c['hipsShiftConstant']))


if __name__ == '__main__':
    main()
