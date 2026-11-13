"""Groundslam clips of the fists packs (npc1 layout), run AFTER npc_build.py on the same output folder (2026-11-09, fists final round).

  python -I npc_groundslam.py [--rig woman|man] [--out DIR]        (DIR = the npc_build.py --out folder; default npc_build.OUT)
Reads  DIR\\clips\\<set>__melee_idlepose.npc1.npz (the guard npc_build.py built) and the frozen mb7 v4 clips
       Build\\work\\melee-export-20261109\\input-mb7-frozen\\clips\\<set>__<clip>.v4.npz (W [T,73,4,4] world, game axes; MotionBricks + v3b
       re-solve of the game's own first-person groundslam between the game clip's key poses, inputs from the game only; provenance.txt /
       sha256.txt there).
Writes DIR\\clips\\<set>__<clip>.npc1.npz / .info.json for the 4 groundslam clips (they REPLACE the 'guardhold' versions npc_build.py wrote;
       its build_report.json keeps the guardhold rows) and DIR\\groundslam_build.json.
Method (ported unchanged from Build\\work\\melee-fists-final-20261109\\scripts\\groundslam.py, which built the final fists packs): the LOCAL
rotations of Spine1 and every body joint below it (the upper body the game shows under the melee upper-body mask) from mb7 on the npc1 guard
(Root, Hips, Spine, legs and the 79 helper joints; body joint translations = the guard's = the player rig);
  air_start: frame 0 = the guard, pose blend (slerp of the locals) into mb7 over 10 frames, the last 4 frames blended onto air_idle frame 0;
  air_idle: mb7 as is (loop);
  air_exit: starts on the air_idle loop pose (offset decaying over 4 frames), impact frame 8, blend back to the guard over frames 11-24;
  groundslam_attack: frame 0 = the guard (slerp over 12 frames), impact frame 21, blend back to the guard over frames 24-37;
then npc_build.eye_push (wrist >= EYE_MIN from the eyes) and npc_build.gates on every frame.  Licence note for the packs: these 4 clips are
generated clips (MotionBricks, NVIDIA Open Model Agreement; inputs = the game clip only).
"""
import hashlib
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import npc_build as NB  # noqa: E402  (imports base_body: --rig is consumed there)
import npc_retarget as NR  # noqa: E402
import melee_common as MC  # noqa: E402

MB7 = os.path.join(MC.ROOT, 'Build', 'work', 'melee-export-20261109', 'input-mb7-frozen', 'clips')
CLIPS = ['melee_groundslam_attack_air_start', 'melee_groundslam_attack_air_idle', 'melee_groundslam_attack', 'melee_groundslam_attack_air_exit']
SET = NB.SET
IX, PAR = NB.IX, np.asarray(NB.PAR)
N = len(PAR)
NAMES = NB.P.names
# impact = the vanilla 'w_cyb_strongarms_hit_back' sound event frame (groundslam_attack 21, air_exit 8); the end blend to the guard runs over
# [impact + 3, impact + NB.GUARD_BY] (the npc1 attack rule: back at the guard GUARD_BY frames after the impact), the guard is held after it.
IMPACT = {'melee_groundslam_attack': 21, 'melee_groundslam_attack_air_exit': 8}
START_IN = {'melee_groundslam_attack_air_start': 10, 'melee_groundslam_attack': 12}   # frame-0 offset to the guard decays over these frames
CHAIN_OUT = 4      # air_start: last frames blended onto air_idle frame 0 (the clip the game loops next)
EXIT_IN = 4        # air_exit: frame-0 offset to the air_idle loop pose decays over these frames (before the impact, frame 8)


def descendants(root):
    out = {root}
    for j in range(N):
        k = j
        while k >= 0:
            if k in out:
                out.add(j)
                break
            k = PAR[k]
    return sorted(out)


MASK73 = [j for j in descendants(IX['Spine1']) if j < 73]


def sha256(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest().upper()


def local73(W):
    L = W.copy()
    for j in range(73):
        if PAR[j] >= 0:
            L[:, j] = np.linalg.inv(W[:, PAR[j]]) @ W[:, j]
    return L


def mb7_composite(clip, g_lt, g_lr):
    """-> lt, lr [T,152] (game axes), report: mb7 local translation vs the guard's (cm) on the mask joints."""
    p = os.path.join(MB7, '%s__%s.v4.npz' % (SET, clip))
    z = np.load(p)
    names = [str(x) for x in z['names']]
    assert names == list(NAMES[:73]), 'mb7 joint order'
    Lm = local73(z['W'].astype(float))
    T = len(Lm)
    lt, lr = np.repeat(g_lt[None], T, 0), np.repeat(g_lr[None], T, 0)
    lr[:, MASK73] = R.from_matrix(Lm[:, MASK73, :3, :3].reshape(-1, 3, 3)).as_quat().reshape(T, len(MASK73), 4)
    dt = np.linalg.norm(Lm[:, MASK73][:, :, :3, 3] - g_lt[None, MASK73], axis=-1).max() * 100
    return lt, NB.qcont(lr), dict(src=p, src_sha256=sha256(p), mb7_local_t_vs_rig_cm=round(float(dt), 3), frames=T)


def pose_d(Wa, Wb):
    """max body-joint position (cm) / rotation (deg) difference of two poses (73 body joints except Root / Trajectory / reference_joint)."""
    b = NB.BODY_IX
    p = np.linalg.norm(Wa[b, :3, 3] - Wb[b, :3, 3], axis=-1).max() * 100
    ang = MC.rot_delta_deg(Wa[b, :3, :3], Wb[b, :3, :3]).max()
    return round(float(p), 2), round(float(ang), 2)


def seams(Ws, g_w):
    s = {}
    a, i, k, x = (Ws[c] for c in CLIPS)
    s['air_start f0 vs guard'] = pose_d(a[0], g_w)
    s['air_start last vs air_idle f0'] = pose_d(a[-1], i[0])
    s['air_idle last vs air_idle f0 (loop)'] = pose_d(i[-1], i[0])
    s['air_idle last vs groundslam_attack f0'] = pose_d(i[-1], k[0])
    s['air_idle last vs air_exit f0'] = pose_d(i[-1], x[0])
    s['air_idle f0 vs groundslam_attack f0'] = pose_d(i[0], k[0])
    s['groundslam_attack last vs guard'] = pose_d(k[-1], g_w)
    s['air_exit last vs guard'] = pose_d(x[-1], g_w)
    s['air_idle f0 vs guard'] = pose_d(i[0], g_w)
    return s


def sstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def gates(W, loop=False):
    gg = NB.gates(W, np.zeros((len(W), 3)), loop=loop)
    return {k: v for k, v in gg.items() if not k.startswith('_')}, gg


def cross(W):
    """check_cross rule: crossed = left hand right of the right hand, or a hand > 5 cm past the chest midline (shoulder line, ground plane)."""
    P = W[:, :, :3, 3]
    la, ra = P[:, IX['LeftArm']], P[:, IX['RightArm']]
    mid = (la + ra) / 2
    x = ra - la
    x[:, 2] = 0
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    lat = lambda j: np.einsum('ti,ti->t', P[:, IX[j]] - mid, x) * 100
    lh, rh = lat('LeftHand'), lat('RightHand')
    crossed = (lh > rh) | (lh > 5.0) | (rh < -5.0)
    return dict(crossed_frames=int(crossed.sum()), min_gap_cm=round(float((rh - lh).min()), 1))


def build(out_dir):
    cdir = os.path.join(out_dir, 'clips')
    gp = os.path.join(cdir, '%s__melee_idlepose.npc1.npz' % SET)
    g = np.load(gp)
    g_lt, g_lr = g['lt'].astype(float)[0], g['lr'].astype(float)[0]
    g_w = NB.fk(g_lt[None], g_lr[None])[0]
    comp = {c: mb7_composite(c, g_lt, g_lr) for c in CLIPS}
    rep, Ws, fails = {}, {}, []
    for c in CLIPS:
        lt, lr, info = comp[c]
        lt, lr = lt.copy(), lr.copy()
        T = len(lr)
        i = np.arange(T, dtype=float)
        steps = []
        if c in START_IN:                     # frame 0 = the guard, pose blend (slerp of the locals) into mb7 over START_IN frames; an offset
            w = 1 - sstep(i / START_IN[c])    # decay (npc_build.offset_start) lifts the fist past the eyes on frames 2-4 (hand_eye gate)
            lr = NB.qcont(NB.qslerp(lr, np.repeat(g_lr[None], T, 0), np.repeat(w[:, None], N, 1)))
            steps.append('frame 0 = the guard, pose blend into mb7 over %d frames' % START_IN[c])
        if c == 'melee_groundslam_attack_air_start':
            nxt = comp['melee_groundslam_attack_air_idle'][1][0]
            w = sstep((i - (T - 1 - CHAIN_OUT)) / CHAIN_OUT)
            lr = NB.qcont(NB.qslerp(lr, np.repeat(nxt[None], T, 0), np.repeat(w[:, None], N, 1)))
            steps.append('last %d frames blended onto air_idle frame 0' % CHAIN_OUT)
        if c == 'melee_groundslam_attack_air_exit':   # starts where air_idle loops (Male mb7: 2.1 cm / 3.3 deg apart)
            prv = comp['melee_groundslam_attack_air_idle'][1][-1]
            w = 1 - sstep(i / EXIT_IN)
            lr = NB.qcont(NB.offset_start(lr, prv, w))
            steps.append('frame-0 offset to the air_idle loop pose decaying over %d frames' % EXIT_IN)
        if c in IMPACT:
            f0, f1 = IMPACT[c] + 3, IMPACT[c] + NB.GUARD_BY
            w = sstep((i - f0) / float(f1 - f0))
            lr = NB.qcont(NB.blend_to(lr, g_lr, w))
            steps.append('blend to the guard over frames %d-%d (impact %d), guard held after' % (f0, f1, IMPACT[c]))
        push = NB.eye_push(lt, lr)              # npc_build anatomy step: wrist >= EYE_MIN from the eyes (arm hinge IK, fist world rotation kept)
        lr = NB.qcont(lr)
        steps.append('eye push (npc_build.eye_push) max %.1f cm' % push)
        W = NB.fk(lt, lr)
        Ws[c] = W
        gg, graw = gates(W, loop=c.endswith('air_idle'))
        cr = cross(W)
        meta = dict(set=SET, clip=c, rig=NB.RIG, animation_type='Normal', additive=False, root_motion_type='None', frames=T,
                    duration=(T - 1) / 30.0, events=[], source='groundslam.py (mb7 v4 upper body on the npc1 guard) npc1')
        dv, _ = NB.vanilla(c)
        assert len(dv['W']) == T, (c, len(dv['W']), T)
        d = NR.make_dump(NB.P, lt, lr, meta)
        base = os.path.join(cdir, '%s__%s.npc1' % (SET, c))
        NR.save_dump(base + '.npz', d)
        inf = dict(kind='mb7conv', note='groundslam: MotionBricks/v3b re-solve (mb7 v4) of the game clip, upper body (Spine1 and below) on the npc1 guard',
                   source=info['src'], source_sha256=info['src_sha256'], mb7_local_t_vs_rig_cm=info['mb7_local_t_vs_rig_cm'], steps=steps,
                   impact_vanilla=IMPACT.get(c), guard='melee_idlepose of %s (sha256 %s)' % (cdir, sha256(gp)), gates=gg, cross=cr,
                   tool='npc_groundslam.py', licence='generated clip: MotionBricks (NVIDIA Open Model Agreement); inputs = the game clip only',
                   failing_frames=[dict(frame=int(t), worst=graw['_worst'][t]) for t in np.where(~graw['_ok_frames'])[0]][:40])
        MC.jdump(inf, base + '.info.json')
        rep[c] = dict(PASS=gg['PASS'], gates=gg, cross=cr, steps=steps)
        if not gg['PASS']:
            fails.append(c)
        print('%-36s %s base %d/%d margin %.3f spike %.2f head %.1f ff %.1f wrist %.1f slide %s sole %.2f cross %s %s' % (
            c, 'PASS' if gg['PASS'] else 'FAIL', gg['base_gate_pass'], gg['frames'], gg['margin_min'], gg['spike_max_cm'], gg['head_clear_min_cm'],
            gg['fist_fist_min_cm'], gg['wrist_max_m_s'], gg['slide_max_cm'], gg['sole_min_cm'], cr,
            ('loop %.3f cm %.3f deg' % (gg['loop_gap_cm'], gg['loop_gap_deg'])) if 'loop_gap_cm' in gg else ''), flush=True)
    rep['seams'] = seams(Ws, g_w)
    rep['seams']['air_exit last vs groundslam_attack f0'] = pose_d(Ws[CLIPS[3]][-1], Ws[CLIPS[2]][0])
    for k, v in rep['seams'].items():
        print('  seam %-42s %s' % (k, v))
    MC.jdump(rep, os.path.join(out_dir, 'groundslam_build.json'))
    print('GROUNDSLAM %s %d clips, FAIL %s -> %s' % (SET, len(CLIPS), fails or 'none', cdir), flush=True)
    return 0 if not fails else 1


if __name__ == '__main__':
    a = sys.argv[1:]
    sys.exit(build(a[a.index('--out') + 1] if '--out' in a else NB.OUT))
