"""Mesh clearance + helper gate for a motorcycle mount / dismount clip of the player (bike13, 2026-11-09): run on the ROUND-TRIPPED GLB
(export_glb.ps1 -RootMotion of the imported .anims), i.e. on what the game will play.

  python -I bike_mount_gate.py <glb:clip> --set p?a_sportbike[2|3] --side l|r [--kind enter|exit] [--margin 4] [--report out.json]
         [--inst <installed glb:clip>]  (the installed clip: its swing-foot landing frame bounds the air window, its final pose the seated contact)

Gates (exit code 0 = PASS):
  swing leg (the leg that goes over the saddle: Right for enter_l / exit_r, Left for enter_r / exit_l), while in the AIR (enter: from lift-off to
  2 frames before the landing; exit: from the lift-off from the peg to 2 frames before the ground): shin / foot clearance from the bike mesh
  >= --margin (cm, radius model 6 cm shin / 5 cm foot + 9 cm shoe tip) and NO surface crossing of a leg sample between consecutive frames;
  stance leg: no surface crossing while standing (enter: until the stance foot starts for the peg; exit: after it leaves the peg);
  seated phase (thighs on the saddle, foot on the peg): reported only (the vanilla TPP idle itself has the toe 1 cm inside the engine cover);
  helpers: arm_*_ik, arm_*_ik_offset, Torso_(World_)*Arm_Hand_IK_JNT within 1.5 cm of the hand (the vanilla pma_sportbike2 FPP clip itself has 1.0) and leg_*_ik, Torso_(World_)*Leg_Foot_IK_JNT
  within 0.5 cm of the foot on EVERY frame (the bike12 rule: a stale hand target glues the hand to the hip in game).
"""
import argparse
import json
import sys
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'anim', 'vehicle-entry'))
import skel as S  # noqa: E402
import build_entry_clip as B  # noqa: E402
import bike_model as BM  # noqa: E402

CR = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])


def load(spec):
    glb, clip = spec.rsplit(':', 1)
    sk = S.Skeleton.from_glb(glb)
    c = sk.clip(clip)
    dur, t = B.frames_of(sk, c)
    lt, lr = sk.sample_local(c, t)
    Wt, Wr = sk.world(lt, S.qcontinuous(lr))
    ri = sk.index['Root']
    R0, t0 = Wr[-1, ri], Wt[-1, ri]
    P = np.einsum('ij,tnj->tni', CR.T @ R0.T, Wt - t0)
    return sk, t, Wt, P


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('clip')
    ap.add_argument('--set', required=True)
    ap.add_argument('--side', required=True, choices=('l', 'r'))
    ap.add_argument('--kind', default='enter', choices=('enter', 'exit'))
    ap.add_argument('--margin', type=float, default=4.0, help='cm')
    ap.add_argument('--inst', help='installed clip glb:clip (landing frame / seated contact reference)')
    ap.add_argument('--report')
    a = ap.parse_args()
    sk, t, Wt, P = load(a.clip)
    ix = sk.index
    T = len(t)
    model = BM.BikeModel.for_set(a.set)
    if a.kind == 'enter':
        swing = 'Right' if a.side == 'l' else 'Left'
    else:
        swing = 'Left' if a.side == 'l' else 'Right'
    stance = 'Left' if swing == 'Right' else 'Right'
    fo_s, fo_t = ix[swing + 'Foot'], ix[stance + 'Foot']
    fr = np.arange(T)
    # phases from the clip itself: the swing foot's own motion (speed) and its final / initial spot
    vs = np.r_[0, np.linalg.norm(np.diff(P[:, fo_s], axis=0), axis=1)] * 30
    if a.kind == 'enter':
        d_end = np.linalg.norm(P[:, fo_s] - P[-1, fo_s], axis=1) * 100
        f_land = next((f for f in range(T) if (d_end[f:] < 2.0).all()), T - 5)
        f_lift = next((f for f in range(1, T) if P[f, fo_s, 2] > P[0, fo_s, 2] + 0.03), 1) - 1
        air = (fr > f_lift) & (fr < f_land - 2)
        zs = P[:, fo_t, 2]
        zpl = zs[:max(f_land, 1)].min() if f_land > 0 else zs.min()
        f_stance_end = next((f for f in range(f_lift, T) if zs[f] > zpl + 0.02), T - 1)
        stand = fr <= f_stance_end
    else:
        d0 = np.linalg.norm(P[:, fo_s] - P[0, fo_s], axis=1) * 100
        f_lift = next((f for f in range(T) if d0[f] > 2.0), 1) - 1
        f_land = next((f for f in range(f_lift + 3, T) if vs[f] < 0.3 and P[f, fo_s, 2] < P[f_lift, fo_s, 2] - 0.15), T - 3)
        air = (fr > f_lift) & (fr < f_land - 2)
        stand = fr >= f_land
    res = dict(clip=a.clip, set=a.set, side=a.side, kind=a.kind, frames=T, swing=swing, f_lift=int(f_lift), f_land=int(f_land), margin_cm=a.margin)
    rows_s = BM.clearance_report(model, P, ix, swing, thigh=False)
    cr_s = BM.sweep_crossings(model, P, ix, swing, thigh=False)
    rows_t = BM.clearance_report(model, P, ix, stance, thigh=False)
    cr_t = BM.sweep_crossings(model, P, ix, stance, thigh=False)
    air_rows = [r for r in rows_s if air[r['frame']]]
    w = min(air_rows, key=lambda r: r['clear_cm']) if air_rows else None
    res['swing_air'] = dict(min_clear_cm=w['clear_cm'] if w else None, frame=w['frame'] if w else None, part=w['part'] if w else None, seg=w['seg'] if w else None,
                            crossings=int(sum(c for f, c in enumerate(cr_s) if air[f])), frames=[int(f) for f in fr[air]])
    res['stance_standing'] = dict(crossings=int(sum(c for f, c in enumerate(cr_t) if stand[f])), min_clear_cm=min((r['clear_cm'] for r in rows_t if stand[r['frame']]), default=None))
    res['seated_min_clear_cm'] = dict(swing=min(r['clear_cm'] for r in rows_s if r['frame'] >= f_land), stance=min(r['clear_cm'] for r in rows_t if r['frame'] >= f_land)) if a.kind == 'enter' else None
    res['per_frame'] = dict(swing=[(r['frame'], r['clear_cm'], r['part'], int(c)) for r, c in zip(rows_s, cr_s)], stance=[(r['frame'], r['clear_cm'], r['part'], int(c)) for r, c in zip(rows_t, cr_t)])
    hel = {}
    for sd in ('Left', 'Right'):
        lo = sd.lower()
        for src, names, lim in ((sd + 'Hand', ['arm_%s_ik' % lo, 'arm_%s_ik_offset' % lo, 'Torso_%sArm_Hand_IK_JNT' % sd, 'Torso_World_%sArm_Hand_IK_JNT' % sd], 1.5),
                                (sd + 'Foot', ['leg_%s_ik' % lo, 'Torso_%sLeg_Foot_IK_JNT' % sd, 'Torso_World_%sLeg_Foot_IK_JNT' % sd], 0.5)):
            for h in names:
                if h in ix:
                    d = np.linalg.norm(Wt[:, ix[h]] - Wt[:, ix[src]], axis=1) * 100
                    hel[h] = dict(max_cm=round(float(d.max()), 2), frame=int(np.argmax(d)), limit_cm=lim, ok=bool(d.max() <= lim))
    res['helpers'] = hel
    ok_air = (w is None or w['clear_cm'] >= a.margin) and res['swing_air']['crossings'] == 0
    ok_stance = res['stance_standing']['crossings'] == 0
    ok_hel = all(v['ok'] for v in hel.values())
    res['ok'] = bool(ok_air and ok_stance and ok_hel)
    res['checks'] = dict(swing_air_ok=bool(ok_air), stance_standing_ok=bool(ok_stance), helpers_ok=bool(ok_hel))
    if a.report:
        json.dump(res, open(a.report, 'w', encoding='utf-8'), indent=1)
    print('MOUNTGATE %s %s %s %s | swing air clear %s cm @f%s (%s) crossings %d [f%d..f%d] | stance standing crossings %d | seated min %s | helpers worst %.2f cm | %s' % (
        a.set, a.side, a.kind, 'PASS' if res['ok'] else 'FAIL', res['swing_air']['min_clear_cm'], res['swing_air']['frame'], res['swing_air']['part'], res['swing_air']['crossings'], f_lift, f_land,
        res['stance_standing']['crossings'], res['seated_min_clear_cm'], max(v['max_cm'] for v in hel.values()), os.path.basename(a.clip.rsplit(':', 1)[0])))
    return 0 if res['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
