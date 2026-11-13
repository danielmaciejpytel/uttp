"""Verify the male Nue reload .anims against the female installed set and against the retargeted source GLB.

Inputs are GLB exports made by WolvenKit (export_glb.ps1): the female installed set, the intended male GLB (retarget
output) and the male set re-exported from the packed .anims.  Offline only.
Checks: clip names + order independent equality, frame counts, key times, events + float tracks identical to the female
clip (same phases), max local rotation/translation error male-roundtrip vs intended, male FK vs female model space for the
arm complex and the left-hand-in-WeaponRight relation (magazine contact), dense gun-space controllers.
Usage: python -I verify_nue_reload_man_roundtrip.py --female F.glb --intended I.glb --roundtrip R.glb --report OUT.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'Source' / 'tools' / 'anim' / 'vehicle-entry'))
import glbanim as G  # noqa: E402
from skel import Skeleton  # noqa: E402

DENSE = ('arm_left_ik_offset', 'arm_right_ik_offset')


def ang(A, B):
    r = np.einsum('...ji,...jk->...ik', A, B)
    tr = r[..., 0, 0] + r[..., 1, 1] + r[..., 2, 2]
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--female', required=True)
    ap.add_argument('--intended', required=True)
    ap.add_argument('--roundtrip', required=True)
    ap.add_argument('--report', required=True)
    a = ap.parse_args()
    F = Skeleton.from_glb(a.female)
    I = Skeleton.from_glb(a.intended)
    R = Skeleton.from_glb(a.roundtrip)
    fc, ic, rc = G.clips(F.gj), G.clips(I.gj), G.clips(R.gj)
    rep = dict(ok=True, clips={})
    assert set(fc) == set(rc) == set(ic), (sorted(fc), sorted(rc))
    assert R.names == F.names and R.parent == F.parent
    ix = F.index
    for name in sorted(fc):
        fa, ia, ra = F.gj['animations'][fc[name]], I.gj['animations'][ic[name]], R.gj['animations'][rc[name]]
        fch, ich, rch = F.clip_tracks(name), I.clip_tracks(name), R.clip_tracks(name)
        tf = fch['LeftHand']['rotation'][0]
        tr = rch['LeftHand']['rotation'][0]
        c = dict(framesFemale=len(tf), framesMale=len(tr), durationFemale=float(tf[-1]), durationMale=float(tr[-1]))
        c['sameFrameCount'] = len(tf) == len(tr)
        c['maxKeyTimeDiffSeconds'] = float(np.abs(tf - tr).max()) if len(tf) == len(tr) else None
        fe, re = fa['extras'], ra['extras']
        c['eventsIdentical'] = json.dumps(fe.get('animEvents'), sort_keys=True) == json.dumps(re.get('animEvents'), sort_keys=True)
        c['trackKeysIdentical'] = json.dumps(fe.get('trackKeys'), sort_keys=True) == json.dumps(re.get('trackKeys'), sort_keys=True)
        c['constTrackKeysIdentical'] = json.dumps(fe.get('constTrackKeys'), sort_keys=True) == json.dumps(re.get('constTrackKeys'), sort_keys=True)
        c['eventFrames'] = sorted((e.get('eventName'), e['startFrame']) for e in re.get('animEvents', []))
        c['animationType'] = re.get('animationType')
        # roundtrip vs intended at female key times (sparse) and dense times
        lt_i, lr_i = I.sample_local(ic[name], tf)
        lt_r, lr_r = R.sample_local(rc[name], tf)
        dq = np.abs(np.abs(np.sum(lr_i * lr_r, axis=-1)) - 1).max()
        c['roundtripMaxLocalTranslationErrMeters'] = float(np.abs(lt_i - lt_r).max())
        c['roundtripMaxLocalRotationErrDeg'] = float(np.degrees(2 * np.arccos(np.clip(1 - dq, -1, 1))))
        td = fch[DENSE[0]]['rotation'][0]
        lt_id, lr_id = I.sample_local(ic[name], td)
        lt_rd, lr_rd = R.sample_local(rc[name], td)
        c['denseKeysFemale'] = len(td)
        c['denseKeysMale'] = len(rch[DENSE[0]]['rotation'][0])
        c['denseRoundtripMaxTranslationErrMeters'] = float(np.abs(lt_id - lt_rd)[:, [ix[d] for d in DENSE]].max())
        # male FK (roundtrip) vs female model space
        lt_f, lr_f = F.sample_local(fc[name], tf)
        Wtf, Wrf = F.world(lt_f, lr_f)
        Wtm, Wrm = R.world(lt_r, lr_r)
        arm = ['LeftShoulder', 'LeftArm', 'LeftForeArm', 'LeftHand', 'RightShoulder', 'RightArm', 'RightForeArm', 'RightHand', 'WeaponRight']
        arm += [n for n in F.names if n.startswith(('LeftHand', 'RightHand', 'LeftInHand', 'RightInHand')) and n not in arm]
        c['armComplexMaxPosDiffMeters'] = float(max(np.linalg.norm(Wtm[:, ix[n]] - Wtf[:, ix[n]], axis=-1).max() for n in arm))
        c['handsMaxRotDiffDeg'] = float(max(ang(Wrm[:, ix[n]], Wrf[:, ix[n]]).max() for n in ('LeftHand', 'RightHand', 'WeaponRight')))

        def relp(Wt, Wr, p, q):
            inv = np.transpose(Wr[:, p], (0, 2, 1))
            return np.einsum('tij,tj->ti', inv, Wt[:, q] - Wt[:, p]), inv @ Wr[:, q]
        rf, rm = relp(Wtf, Wrf, ix['WeaponRight'], ix['LeftHand']), relp(Wtm, Wrm, ix['WeaponRight'], ix['LeftHand'])
        c['magazineContactLeftHandInWeaponPosDiffMeters'] = float(np.linalg.norm(rf[0] - rm[0], axis=-1).max())
        c['magazineContactLeftHandInWeaponRotDiffDeg'] = float(ang(rf[1], rm[1]).max())
        ok = (c['sameFrameCount'] and c['eventsIdentical'] and c['trackKeysIdentical'] and c['constTrackKeysIdentical']
              and c['denseKeysFemale'] == c['denseKeysMale'] and c["maxKeyTimeDiffSeconds"] < 1e-4
              and c['roundtripMaxLocalRotationErrDeg'] < 0.05 and c['roundtripMaxLocalTranslationErrMeters'] < 5e-4
              and c['magazineContactLeftHandInWeaponPosDiffMeters'] < 1e-3 and c['magazineContactLeftHandInWeaponRotDiffDeg'] < 0.5)
        c['pass'] = bool(ok)
        rep['ok'] &= bool(ok)
        rep['clips'][name] = c
        print(name, 'PASS' if ok else 'FAIL', {k: (round(v, 6) if isinstance(v, float) else v) for k, v in c.items() if k not in ('eventFrames',)})
    Path(a.report).write_text(json.dumps(rep, indent=2) + '\n')
    print('ALL PASS' if rep['ok'] else 'FAILURES')
    return 0 if rep['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
