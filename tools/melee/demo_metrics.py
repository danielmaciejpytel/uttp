"""Before / after metrics of the melee demo clips (original game clip vs corrected MotionBricks clip), from the 73-joint dumps.

  python -I demo_metrics.py [--out demo\\metrics.json]
Per clip: max speed of the hands and of any joint (m/s, world), max one-frame joint rotation step (deg, local, with joint+frame),
foot slide (max horizontal drift of each foot from frame 0, cm, and max foot speed cm/frame), seam jumps (largest one-frame local
rotation step in +-1 frame around every key time_frame vs the clip median step), neck / head (Spine3-Head distance range, Neck1-Head
bone length range, head z and hips z drop vs frame 0, cm).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

CLIPS = [('pwa_katana', 'combo_attack_04_hit'), ('pwa_fists', 'combo_attack_01'), ('pwa_fists', 'block_deflect_right'),
         ('pwa_two_handed_blunt', 'heavy_attack_01'), ('pwa_two_handed_blunt', 'melee_equip'), ('pwa_katana', 'combo_attack_01')]
# male twins of the demo clips (Male V, pma_* sets; redo_demo.py --rig man)
CLIPS_MAN = [('pma_katana', 'combo_attack_04_hit'), ('pma_fists', 'combo_attack_01'), ('pma_fists', 'block_deflect_right'),
             ('pma_two_handed_blunt', 'heavy_attack_01'), ('pma_two_handed_blunt', 'melee_equip'), ('pma_katana', 'combo_attack_01')]


# clips whose correction was refined for the demo (extra key spreading the hit snap, MB seed picked by the smallest one-frame step)
DEMO_MB = {'combo_attack_04_hit': os.path.join(MC.WORK, 'demo', 'seed11', 'combo_attack_04_hit'),
           'heavy_attack_01': os.path.join(MC.WORK, 'demo', 'fixed', 'pwa_two_handed_blunt', 'heavy_attack_01')}  # k4/k5 window 1: hips return over 5 frames instead of 3


def mb_prefix(s, c):
    return DEMO_MB[c] if c in DEMO_MB else os.path.join(MC.WORK, 'corrected', s, c)


def load(path):
    z = np.load(path, allow_pickle=True)
    return z['W'], [str(n) for n in z['names']], z['parents']


def local_steps(W, par):
    """[T-1, J] angle (deg) of the local rotation change of every joint between consecutive frames."""
    T, J = W.shape[:2]
    L = W[:, :, :3, :3].copy()
    for j in range(J):
        if par[j] >= 0:
            L[:, j] = np.swapaxes(W[:, par[j], :3, :3], -1, -2) @ W[:, j, :3, :3]
    M = np.swapaxes(L[:-1], -1, -2) @ L[1:]
    tr = np.trace(M, axis1=-2, axis2=-1)
    return np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))


def metrics(W, names, par, seams, spans=()):
    ix = {n: i for i, n in enumerate(names)}
    P = W[:, :, :3, 3]
    out = {}
    v = np.linalg.norm(np.diff(P, axis=0), axis=-1) * 30.0   # m/s
    hands = [ix['LeftHand'], ix['RightHand']]
    out['max_hand_speed_m_s'] = round(float(v[:, hands].max()), 2)
    j = int(v.max(axis=0).argmax())
    out['max_joint_speed_m_s'] = [round(float(v.max()), 2), names[j], int(v[:, j].argmax())]
    st = local_steps(W, par)
    st[:, ix['Hips']] = 0   # hips local = world (root heading/translation), not a joint rotation
    jj = int(st.max(axis=0).argmax())
    out['max_rot_step_deg'] = [round(float(st.max()), 1), names[jj], int(st[:, jj].argmax())]
    out['median_rot_step_deg'] = round(float(np.median(st.max(axis=1))), 2)
    for side in ('Left', 'Right'):
        f = P[:, ix[side + 'Foot']]
        out[side.lower() + '_foot_drift_cm'] = round(float(np.linalg.norm(f[:, :2] - f[0, :2], axis=1).max() * 100), 2)
        out[side.lower() + '_foot_max_step_cm'] = round(float(np.linalg.norm(np.diff(f, axis=0), axis=1).max() * 100), 2)
    sj = []
    for s in seams:
        lo, hi = max(s - 1, 0), min(s + 1, len(st) - 1)
        seg = st[lo:hi + 1]
        sj.append([int(s), round(float(seg.max()), 1), names[int(seg.max(axis=0).argmax())]])
    out['seam_rot_steps'] = sj
    out['seam_max_step_deg'] = max([x[1] for x in sj], default=0.0)
    out['mb_span_max_step_deg'] = [round(float(st[a:b].max()), 1) for a, b in spans if b > a]
    head, s3, n1 = P[:, ix['Head']], P[:, ix['Spine3']], P[:, ix['Neck1']]
    d = np.linalg.norm(head - s3, axis=1) * 100
    nl = np.linalg.norm(head - n1, axis=1) * 100
    out['spine3_head_dist_cm'] = [round(float(d.min()), 1), round(float(d.max()), 1)]
    out['neck1_head_len_cm'] = [round(float(nl.min()), 2), round(float(nl.max()), 2)]
    out['head_z_drop_cm'] = round(float((P[0, ix['Head'], 2] - P[:, ix['Head'], 2]).max() * 100), 1)
    out['hips_z_drop_cm'] = round(float((P[0, ix['Hips'], 2] - P[:, ix['Hips'], 2]).max() * 100), 1)
    return out


def main(a):
    res = {}
    for s, c in CLIPS:
        Wo, names, par = load(MC.clip_path(s, c))
        Wm, _, _ = load(mb_prefix(s, c) + '.mb.dump.npz')
        kj = json.load(open(mb_prefix(s, c) + '.keys.json', encoding='utf-8'))
        keys = {k['id']: k['time_frame'] for k in kj['keys']}
        seams = sorted({keys[x] for sg in kj['segments'] if sg['mode'] == 'mb' for x in (sg['from'], sg['to'])})
        spans = [(keys[sg['from']], keys[sg['to']]) for sg in kj['segments'] if sg['mode'] == 'mb']
        res['%s/%s' % (s, c)] = dict(frames=[len(Wo), len(Wm)], mb_seam_frames=seams, mb_spans=spans, orig=metrics(Wo, names, par, seams, spans),
                                      mb=metrics(Wm, names, par, seams, spans))
    out = a[a.index('--out') + 1] if '--out' in a else os.path.join(MC.WORK, 'demo', 'metrics.json')
    MC.jdump(res, out)
    for k, v in res.items():
        print(k, v['frames'], 'seams', v['mb_seam_frames'])
        for who in ('orig', 'mb'):
            m = v[who]
            print('  %-4s hand %.1f m/s | joint %s | rot step %s med %.1f | foot drift L %.1f R %.1f cm step %.1f/%.1f | seam %.1f deg | s3-head %s neck1-head %s | head dz %.1f hips dz %.1f | span step %s' % (
                who, m['max_hand_speed_m_s'], m['max_joint_speed_m_s'], m['max_rot_step_deg'], m['median_rot_step_deg'],
                m['left_foot_drift_cm'], m['right_foot_drift_cm'], m['left_foot_max_step_cm'], m['right_foot_max_step_cm'],
                m['seam_max_step_deg'], m['spine3_head_dist_cm'], m['neck1_head_len_cm'], m['head_z_drop_cm'], m['hips_z_drop_cm'], m['mb_span_max_step_deg']))


if __name__ == '__main__':
    main(sys.argv[1:])
