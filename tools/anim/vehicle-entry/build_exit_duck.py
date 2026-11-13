"""Head duck for a VANILLA car EXIT clip (neck / upper-spine bend that keeps the head under the roof while V leans out of the seat).

  python -I build_exit_duck.py --glb <vanilla p?a_<class>_<seat>.anims.glb> --clip exit --out <exit.anims.glb> --name exit --duck-roof-json <body-data\\<class>.json>
         --duck-roof-seat left --duck-chain Neck:0.6,Neck1:0.4 [--duck-... options of head_duck.py] [--report out.json] [--identity]

The entry layer (head_duck.py, applied by build_entry_clip.py --head-duck) runs here with --duck-anchor first: the SEATED pose is the FIRST frame of the exit (the vehicle frame
and the penetration gate max(1 cm, seated + 0.5 cm) - slack are taken from it), the bend is 0 at the first frame and in the last --duck-fade s, and the LAST frame is never changed
(the game hands the exit end pose to the motion matcher; VX_Exit_Idle_N10_MB continues from it). Only the chain bones (default Neck, Neck1) are written differently; every other bone track,
the Root (root motion), the events and the float tracks are the vanilla ones. --identity writes the resampled vanilla clip without the duck (control for the round trip).
The output GLB goes through the same WolvenKit import / motion-extraction patch as the entry clips (build_exit.ps1); the exit gates are in check_car_exit.py.
"""
import argparse
import copy
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402
import skel as S  # noqa: E402
import head_duck as HD  # noqa: E402
import build_entry_clip as B  # noqa: E402


def quat_angle_deg(a, b):
    d = np.abs(np.sum(a * b, axis=-1)) / np.maximum(np.linalg.norm(a, axis=-1) * np.linalg.norm(b, axis=-1), 1e-12)
    return np.degrees(2.0 * np.arccos(np.clip(d, -1.0, 1.0)))


def limb_frame(Wp, r, m, e):
    """limb plane frame (origin knee / elbow, u = root -> end, v = bend direction, w = u x v) per frame (same as build_entry_clip.py --limb-ik-final)."""
    A_, K_, E_ = Wp[:, r], Wp[:, m], Wp[:, e]
    u = (E_ - A_) / np.linalg.norm(E_ - A_, axis=-1, keepdims=True)
    v = (K_ - A_) - np.sum((K_ - A_) * u, -1, keepdims=True) * u
    for i in range(len(v)):
        if np.linalg.norm(v[i]) < 0.01 and i:
            v[i] = v[i - 1] - np.dot(v[i - 1], u[i]) * u[i]
    v /= np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)
    return K_, u, v, np.cross(u, v)


def place_poles(tgt, lt, lr, vWt, vWr, active, min_move_cm=0.0):
    """The knee / elbow pole vector helpers are world-fixed helpers the game's limb IK bends toward; after the duck (Hips sink, the knee bend direction moves up to ~13 deg) they are re-placed in
    the limb plane frame with their vanilla frame coordinates (build_entry_clip.py --limb-ik-final, same maths), on the frames where the layer is on; the other frames keep the vanilla values.
    min_move_cm > 0 (2026-11-05, rows without a Hips sink): a limb's poles are re-placed only on the active frames where one of its three joints moved more than this against the vanilla pose
    (an untouched, nearly straight leg has no well defined bend plane: the re-placement moved the pole of the unchanged utility / standard2 left leg by 32-66 cm); 0 = the old behaviour (every active frame)."""
    Wt, Wr = tgt.world(lt, lr)
    done = []
    masks = {}
    for s_ in ('Left', 'Right'):
        for limb, chain in (('Leg', (s_ + 'UpLeg', s_ + 'Leg', s_ + 'Foot')), ('Arm', (s_ + 'Arm', s_ + 'ForeArm', s_ + 'Hand'))):
            ch = [tgt.index[n] for n in chain]
            vK, vu, vv, vw = limb_frame(vWt, *ch)
            nK, nu, nv, nw = limb_frame(Wt, *ch)
            if min_move_cm > 0:
                moved = np.max([np.linalg.norm(Wt[:, j] - vWt[:, j], axis=-1) for j in ch], axis=0) * 100.0
                lmask = active & (moved > min_move_cm)
            else:
                lmask = active
            for pn in ('Torso_%s%s_PoleVector_JNT' % (s_, limb), 'Torso_World_%s%s_PoleVector_JNT' % (s_, limb)):
                if pn not in tgt.index:
                    continue
                pb = tgt.index[pn]
                d = vWt[:, pb] - vK
                c = np.stack([np.sum(d * vu, -1), np.sum(d * vv, -1), np.sum(d * vw, -1)], -1)
                c = np.stack([c[:, 0], np.hypot(c[:, 1], c[:, 2]), np.zeros(len(c))], -1)
                Wt[:, pb] = nK + c[:, :1] * nu + c[:, 1:2] * nv + c[:, 2:] * nw
                done.append(pn)
                masks[pn] = lmask
    flt, flr = tgt.local_from_world(Wt, Wr)
    for pn in done:
        pb = tgt.index[pn]
        lt[masks[pn], pb] = flt[masks[pn], pb]
        lr[masks[pn], pb] = flr[masks[pn], pb]
    lr[:] = S.qcontinuous(lr)
    return done


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--glb', required=True, help='vanilla set GLB (all clips)')
    p.add_argument('--clip', default='exit')
    p.add_argument('--out', required=True)
    p.add_argument('--name', default='exit')
    p.add_argument('--report')
    p.add_argument('--identity', action='store_true', help='no duck: the resampled vanilla clip (round trip control)')
    p.add_argument('--steps', help='steps npz (Source/tools/ai_motion/exit_steps_gen.py assemble): generated stepping for frames c..g, spliced after the duck (exit_steps.py); Root / events / tracks untouched')
    p.add_argument('--steps-bones', default='lower', choices=('lower', 'all'), help='lower: Hips / spine to Spine2 / legs from the generator, the upper body stays (Spine3 keeps its world orientation); all: every rig bone')
    p.add_argument('--steps-ramp', type=int, default=3, help='frames: blend of the generated pose into the existing pose at both ends of c..g (weight 0 at c and g)')
    p.add_argument('--ik-max-extend', type=float, default=0.999)
    p.add_argument('--ik-bend-sigma', type=float, default=2.0)
    p.add_argument('--duck-pole-min-move-cm', type=float, default=0.0, help='re-place a limb knee / elbow pole helper only on the active frames where that limb moved more than this against vanilla (0 = every active frame, the sport2 w row)')
    HD.add_args(p)
    p.set_defaults(duck_anchor='first')
    a = p.parse_args()
    tgt = S.Skeleton.from_glb(a.glb)
    c = tgt.clip(a.clip)
    dur, times = B.frames_of(tgt, c)
    lt0, lr0 = tgt.sample_local(c, times)
    lr0 = S.qcontinuous(lr0)
    ri = tgt.index['Root']
    lt, lr = lt0.copy(), lr0.copy()
    rep = dict(glb=str(a.glb), clip=a.clip, frames=len(times), duration_s=float(dur), identity=bool(a.identity))
    if not a.identity:
        assert a.duck_roof_json, '--duck-roof-json required'
        duck_rep = {}
        HD.apply(a, tgt, lt, lr, ri, times, duck_rep)
        rep['head_duck'] = duck_rep['head_duck']
        # frames where the layer is off (bend 0 and no Hips sink) are restored to the vanilla values bit for bit: the foot IK smooths the knee bend direction over time (--ik-bend-sigma)
        # and leaks a few 1e-3 deg into neighbouring frames (r13j lesson); the first and the last frame in particular must stay the vanilla ones
        sinkv = HD.sink_profile(a.duck_sink_profile, len(times)) + a.duck_sink_cm_per_deg / 100.0 * np.array(rep['head_duck']['bend_deg'])
        active = (np.array(rep['head_duck']['bend_deg']) > 1e-6) | (np.abs(sinkv) > 1e-9)
        leak = 0.0
        for t in np.flatnonzero(~active):
            leak = max(leak, float(max(quat_angle_deg(lr[t], lr0[t]).max(), np.linalg.norm(lt[t] - lt0[t], axis=-1).max() * 1000.0)))
            lt[t], lr[t] = lt0[t], lr0[t]
        vWt, vWr = tgt.world(lt0, lr0)
        rep['poles_replaced'] = place_poles(tgt, lt, lr, vWt, vWr, active, a.duck_pole_min_move_cm)
        rep['restored_inactive_frames'] = int((~active).sum())
        rep['restored_max_leak'] = leak   # deg (rotation) / mm (translation)
    if a.steps:
        import exit_steps as ES
        st = np.load(a.steps, allow_pickle=True)
        rep['steps'] = ES.splice(tgt, lt, lr, st, a.steps_ramp, a.steps_bones)
        vWt2, vWr2 = tgt.world(lt0, lr0)
        rep['steps']['helpers'] = ES.reanchor_helpers(tgt, lt, lr, vWt2, vWr2, int(st['c']), int(st['g']), (int(st['c_gen']), int(st['g_gen'])) if 'c_gen' in st.files else None)
    # frames / bones changed (rotation > 0.001 deg, translation > 0.01 mm), the first and the last frame
    dang = np.stack([quat_angle_deg(lr[:, j], lr0[:, j]) for j in range(lr.shape[1])], 1)
    dtr = np.linalg.norm(lt - lt0, axis=-1)
    chg = (dang > 1e-3) | (dtr > 1e-5)
    rep['changed_frames'] = [int(t) for t in np.flatnonzero(chg.any(1))]
    rep['changed_bones'] = sorted({tgt.names[j] for j in np.flatnonzero(chg.any(0))})
    rep['first_frame_max_diff'] = dict(rot_deg=float(dang[0].max()), trans_m=float(dtr[0].max()))
    rep['last_frame_max_diff'] = dict(rot_deg=float(dang[-1].max()), trans_m=float(dtr[-1].max()))
    rep['root_max_diff'] = dict(rot_deg=float(dang[:, ri].max()), trans_m=float(dtr[:, ri].max()))
    extras = copy.deepcopy(tgt.gj['animations'][c].get('extras', {}))
    B.write_out(tgt, a.out, a.name, times, lt, lr, extras)
    if a.report:
        json.dump(rep, open(a.report, 'w'), indent=1)
    if a.steps:
        print('steps c=%d g=%d ramp %d bones %d helpers %d' % (rep['steps']['c'], rep['steps']['g'], rep['steps']['ramp'], len(rep['steps']['bones']), len(rep['steps']['helpers'])))
    if not a.identity:
        hd = rep['head_duck']
        print('target %.2f cm gate %.2f seated %.2f; max bend %.1f deg; unreachable %s' % (hd['target_cm'], hd['gate_cm'], hd['seated_pen_cm'], hd['max_bend_deg'], hd['unreachable_frames']))
        print('bend   ', hd['bend_deg'])
        print('pen cm ', hd['pen_after_cm'])
        print('neck   ', hd['neck_flex_after'])
        print('lean   ', hd['trunk_lean_after'])
        print('repair ', hd['repair'])
    print('changed frames', rep['changed_frames'], 'bones', rep['changed_bones'])
    print('first frame diff', rep['first_frame_max_diff'], 'last frame diff', rep['last_frame_max_diff'], 'root diff', rep['root_max_diff'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
