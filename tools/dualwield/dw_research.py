"""Dual-wield research (2026-11-09): NPC rig compatibility (copy-locals test), left / right ownership of the player dual-wield clips,
NPC donor survey (hands, guns, root travel, shot frames).

  python -I dw_research.py rig     -> Build\\work\\dualwield-npc-20261109\\research\\rig_compat.json
  python -I dw_research.py own     -> ...\\research\\player_lr_ownership.json
  python -I dw_research.py survey  -> ...\\research\\npc_survey.json, player_survey.json
Copy-locals test: the NPC clip is retargeted with npc_retarget.retarget (NPC LOCAL rotations by name on V's 152-joint rig, V's bone lengths,
Root = identity); for every NPC body bone whose parent is also on both rigs the world bone direction (parent -> child) and the world joint
rotation on V are compared with the NPC's own skeleton, every frame.  0 deg = the joint frames are identical (exact transfer).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dw_common as DC  # noqa: E402

MC, NR = DC.MC, DC.NR
RES = os.path.join(DC.OUT, 'research')
BODY = set(MC.SPINE + MC.HEADNECK + MC.L_ARM + MC.R_ARM + MC.L_LEG + MC.R_LEG + MC.FINGERS_L + MC.FINGERS_R + ['WeaponRight', 'WeaponLeft'])


def rig_test(set_name, clip, P):
    nsk = NR.skeleton(os.path.join(DC.NPC_GLB, set_name + '.anims.glb'))
    nlt, nlr, ex, dur = NR.sample_clip(nsk, clip)
    ri = nsk.index['Root']
    a_lt, a_lr = nlt.copy(), nlr.copy()
    a_lt[:, ri] = 0
    a_lr[:, ri] = [0, 0, 0, 1.0]
    nWt, nWr = nsk.world(a_lt, a_lr)
    lt, lr, missing = NR.retarget(P, nsk, nlt, nlr)
    pWt, pWr = P.sk.world(lt, lr)
    par_mismatch, rows = [], {}
    for n in nsk.names:
        if n == 'Root' or n not in P.ix:
            continue
        k, j = nsk.index[n], P.ix[n]
        pn_n = nsk.names[nsk.parent[k]] if nsk.parent[k] >= 0 else None
        pn_p = P.names[P.par[j]] if P.par[j] >= 0 else None
        if pn_n != pn_p:
            par_mismatch.append((n, pn_n, pn_p))
        rot = MC.rot_delta_deg(nWr[:, k], pWr[:, j])
        row = dict(rot_max=float(rot.max()))
        if pn_n and pn_n == pn_p and pn_n != 'Root':
            kp, jp = nsk.index[pn_n], P.ix[pn_n]
            dn, dp = nWt[:, k] - nWt[:, kp], pWt[:, j] - pWt[:, jp]
            if np.linalg.norm(dn, axis=1).min() > 1e-4:
                row['dir_max'] = float(MC.angle_between(dn, dp).max())
                row['len_npc_cm'] = round(float(np.linalg.norm(dn, axis=1).mean() * 100), 2)
                row['len_v_cm'] = round(float(np.linalg.norm(dp, axis=1).mean() * 100), 2)
        rows[n] = row
    # reference joint frames: rest local rotations of the shared joints
    frame_diff = {}
    for n in nsk.names:
        if n in P.ix and n != 'Root':
            qa, qb = nsk.rest_r[nsk.index[n]], P.sk.rest_r[P.ix[n]]
            frame_diff[n] = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(qa, qb)))))))
    body = {n: r for n, r in rows.items() if n in BODY}
    dmax = max((r.get('dir_max', 0) for r in body.values()), default=0)
    rmax = max(r['rot_max'] for r in body.values())
    worst = sorted(((r.get('dir_max', 0), n) for n, r in body.items()), reverse=True)[:8]
    arm = {}
    for s in DC.SIDES:
        a = ('%sArm' % s, '%sForeArm' % s, '%sHand' % s)
        arm[s] = dict(npc_cm=round(sum(body[x].get('len_npc_cm', 0) for x in a[1:]), 2), v_cm=round(sum(body[x].get('len_v_cm', 0) for x in a[1:]), 2))
    return dict(set=set_name, clip=clip, frames=len(nlt), npc_joints=len(nsk.names), shared=len(rows), missing_on_npc=len(missing),
                parent_mismatch=par_mismatch, body_dir_max_deg=round(dmax, 4), body_rot_max_deg=round(rmax, 4),
                worst_dir=[(n, round(d, 3)) for d, n in worst], ref_frame_diff_max_deg=round(max(frame_diff.values()), 4),
                ref_frame_diff_worst=sorted(((round(v, 3), n) for n, v in frame_diff.items()), reverse=True)[:6],
                upper_plus_forearm_len=arm, has_weapon_bones=[w for w in ('WeaponRight', 'WeaponLeft') if w in nsk.index],
                has_fingers=any('HandIndex' in n for n in nsk.names))


def rig(a):
    P = DC.player('woman')
    tests = [(DC.NPC_DUAL['man_average'], 'aim_ovr_idle'), (DC.NPC_DUAL['man_average'], 'ovr_reload'), (DC.NPC_DUAL['man_average'], 'shoot_precise'),
             (DC.NPC_DUAL['man_big'], 'aim_ovr_idle'), (DC.NPC_ONEHAND['woman_right'], None), (DC.NPC_ONEHAND['woman_left'], None),
             (DC.NPC_ONEHAND['woman_both'], None)]
    out = []
    for s, c in tests:
        if c is None:
            c = sorted(DC.set_clips(s))[0]
        r = rig_test(s, c, P)
        out.append(r)
        print('%-48s %-22s dir_max %7.3f rot_max %7.3f frame_diff %7.3f  par_mismatch %d  worst %s' % (
            s, c, r['body_dir_max_deg'], r['body_rot_max_deg'], r['ref_frame_diff_max_deg'], len(r['parent_mismatch']), r['worst_dir'][:3]))
    DC.jdump(out, os.path.join(RES, 'rig_compat.json'))


def groups_of(P):
    g = {k: [P.ix[n] for n in v] for k, v in MC.GROUPS.items()}
    g['weapons'] = [P.ix['WeaponRight'], P.ix['WeaponLeft']]
    g['helpers_L'] = [P.ix[n] for n in DC.HELPERS['Left'] if n in P.ix]
    g['helpers_R'] = [P.ix[n] for n in DC.HELPERS['Right'] if n in P.ix]
    g['arms_spaces'] = [i for i, n in enumerate(P.names) if n.startswith('arms_') or n.startswith('Arm')]
    return g


def own(a):
    """per joint group: mean local-rotation difference right clip vs left clip (same frame), and each vs the rig reference."""
    P = DC.player('woman')
    G = groups_of(P)
    pairs = [('idlepose_shoulder', 'idlepose_shoulder_l'), ('idlepose_ironsight', 'idlepose_ironsight_l'), ('equip', 'equip_l'),
             ('reload', 'reload_l'), ('unequip', 'unequip_l'), ('melee', 'melee_l'), ('first_equip', 'first_equip_l'),
             ('add_recoil_single_shot_shoulder', 'add_recoil_single_shot_shoulder_l'), ('add_recoil_start', 'add_recoil_start_l'),
             ('add_recoil_loop', 'add_recoil_loop_l'), ('add_recoil_end', 'add_recoil_end_l'),
             ('add_recoil_single_shot_ironsight', 'add_recoil_single_shot_ironsight_l')]
    out = {}
    for r_, l_ in pairs:
        A, B = DC.player_clip(r_), DC.player_clip(l_)
        T = min(len(A['lr']), len(B['lr']))
        Ra = MC.mat4(A['lt'][:T], A['lr'][:T])[..., :3, :3]
        Rb = MC.mat4(B['lt'][:T], B['lr'][:T])[..., :3, :3]
        d = MC.rot_delta_deg(Ra, Rb)                       # [T,152]
        dt = np.linalg.norm(A['lt'][:T] - B['lt'][:T], axis=-1) * 100
        row = {k: dict(rot_mean=round(float(d[:, v].mean()), 2), rot_max=round(float(d[:, v].max()), 2), tr_max_cm=round(float(dt[:, v].max()), 2))
               for k, v in G.items() if v}
        # which hand moves in each clip (hand world travel)
        for nm, C in ((r_, A), (l_, B)):
            row['travel_' + nm] = {s: round(float(np.linalg.norm(np.diff(C['W'][:, P.ix[s + 'Hand'], :3, 3], axis=0), axis=1).sum() * 100), 1)
                                   for s in DC.SIDES}
            row['sets_' + nm] = C['set']
        row['frames'] = (len(A['lr']), len(B['lr']))
        out['%s|%s' % (r_, l_)] = row
        print(r_, l_, row['frames'], {k: (v['rot_mean'], v['rot_max']) for k, v in row.items() if isinstance(v, dict) and 'rot_mean' in v},
              row['travel_' + r_], row['travel_' + l_])
    DC.jdump(out, os.path.join(RES, 'player_lr_ownership.json'))


def gun_axes(W, ix, f):
    """which local axis (+-X/Y/Z) of WeaponRight / WeaponLeft points most along the body forward (barrel), at frame f."""
    H, _ = DC.body_frame(W, ix)
    res = {}
    for w in ('WeaponRight', 'WeaponLeft'):
        if w not in ix:
            continue
        Rw = W[f, ix[w], :3, :3]
        Rb = H[f].T @ Rw                     # columns = weapon axes in the body frame (x right, y fwd, z up)
        best = max(((abs(Rb[1, c]), c, np.sign(Rb[1, c])) for c in range(3)))
        res[w] = dict(axis=('+' if best[2] > 0 else '-') + 'XYZ'[best[1]], fwd_dot=round(float(best[0]), 3),
                      body_cols=np.round(Rb, 3).tolist())
    return res


def survey_clip(W, ix, extras=None):
    T = len(W)
    hr = {s: DC.rel_body(W, ix, DC.pos(W, ix, s + 'Hand')) for s in DC.SIDES}
    sep = np.linalg.norm(DC.pos(W, ix, 'LeftHand') - DC.pos(W, ix, 'RightHand'), axis=1)
    out = dict(frames=T, hands_sep_cm=[round(float(sep.min() * 100), 1), round(float(sep.max() * 100), 1)],
               hand_f0_cm={s: np.round(hr[s][0] * 100, 1).tolist() for s in DC.SIDES},
               hand_mid_cm={s: np.round(hr[s][T // 2] * 100, 1).tolist() for s in DC.SIDES},
               hand_end_cm={s: np.round(hr[s][-1] * 100, 1).tolist() for s in DC.SIDES},
               hand_travel_cm={s: round(float(np.linalg.norm(np.diff(hr[s], axis=0), axis=1).sum() * 100), 1) for s in DC.SIDES},
               guns_f0=gun_axes(W, ix, 0), guns_mid=gun_axes(W, ix, T // 2))
    # shot candidates: per hand, frames where the wrist (hand relative to Spine3) jumps back/up fast (recoil kick)
    for s in DC.SIDES:
        v = np.diff(hr[s], axis=0) * 30
        kick = (-v[:, 1] + v[:, 2])                      # backward + upward speed m/s
        pk = [int(i + 1) for i in range(1, len(kick) - 1) if kick[i] > 0.6 and kick[i] >= kick[i - 1] and kick[i] >= kick[i + 1]]
        out['kick_frames_' + s[0]] = pk
    return out


def survey(a):
    P = DC.player('woman')
    res = {}
    for key, s in list(DC.NPC_DUAL.items()) + [('woman_right', DC.NPC_ONEHAND['woman_right']), ('woman_left', DC.NPC_ONEHAND['woman_left'])]:
        res[s] = {}
        for c in sorted(DC.set_clips(s)):
            try:
                at, dur, evs = DC.anim_info(s, c)
                if at != 'Normal':
                    res[s][c] = dict(type=at, dur=dur)
                    continue
                raw = DC.npc_raw(s, c)
                r = survey_clip(raw['W'], raw['ix'])
                rt = raw['root_t']
                r.update(type=at, dur=round(dur, 3), root_travel_cm=round(float(np.linalg.norm(rt[-1, :2] - rt[0, :2]) * 100), 1),
                         phases=[(e[1], e[2], e[3]) for e in evs if 'Phase' in e[0]],
                         events=sorted({e[1] for e in evs if 'Phase' not in e[0] and 'Foot' not in e[0] and 'Foley' not in e[0]}))
                res[s][c] = r
                print('%-44s %-26s %3d sep %-12s L%-20s R%-20s kickL %s kickR %s guns %s' % (
                    s[:44], c, r['frames'], r['hands_sep_cm'], r['hand_f0_cm']['Left'], r['hand_f0_cm']['Right'], r['kick_frames_L'][:8],
                    r['kick_frames_R'][:8], {k: v['axis'] for k, v in r['guns_f0'].items()}))
            except Exception as e:  # report and continue
                res[s][c] = dict(error=repr(e))
                print(s, c, 'ERROR', repr(e))
    DC.jdump(res, os.path.join(RES, 'npc_survey.json'))
    pres = {}
    for c in ('idlepose_shoulder', 'idlepose_shoulder_l', 'idlepose_ironsight', 'idlepose_ironsight_l', 'equip', 'equip_l', 'reload', 'reload_l',
              'idlepose_frenzy', 'friendly_idlepose'):
        C = DC.player_clip(c)
        r = survey_clip(C['W'], P.ix)
        r['set'] = C['set']
        pres[c] = r
        print('PLAYER %-22s %3d sep %-12s L%-20s R%-20s guns %s' % (c, r['frames'], r['hands_sep_cm'], r['hand_f0_cm']['Left'], r['hand_f0_cm']['Right'],
                                                               {k: v['axis'] for k, v in r['guns_f0'].items()}))
    DC.jdump(pres, os.path.join(RES, 'player_survey.json'))


if __name__ == '__main__':
    {'rig': rig, 'own': own, 'survey': survey}[sys.argv[1]](sys.argv[1:])
