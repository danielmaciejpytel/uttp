"""Shared set-up of the NPC-based ONE-HANDED BLUNT and BATON third-person clips of V (2026-11-09).  Import this module BEFORE
npc_blade_build / npc_blade_film (it registers the two weapons in npc_blade_common and routes the vanilla baton dumps).

  python -I npc_blunt_common.py axis   [--rig woman|man]   weapon axis in the grip (vanilla + NPC), WeaponRight translation, NPC donor
                                                           penetration of the weapon proxy for several lengths -> survey\\axis_<rig>.json
  python -I npc_blunt_common.py survey --weapon blunt|baton [--rig ...]   npc_blade_common.survey (tip paths, strike frames, directions)
  python -I npc_blunt_common.py grip   --weapon blunt|baton [--rig ...]   npc_blade_common.grip (weapon bone vs hand, vanilla vs NPC)

Weapons (player set <- NPC donor set; the game plays p?a_one_handed_blunt for crowbar, pipes, tire iron, wrench, tomahawk, cane, machete,
kukri, wakizashi, chef / butcher knife and the toy weapon; p?a_baton for the telescopic baton):
  blunt  pwa_one_handed_blunt / pma_one_handed_blunt <- wa_ / ma_gang_onehanded_action_combat (the NPC set the game registers for
         one-handed clubs, axes and machetes; exact rigs)
  baton  pwa_baton / pma_baton                       <- the same NPC set (no separate NPC baton set)
Weapon proxy (capsule along +Z of WeaponRight, (start, tip, radius) m; ASSUMPTION, no weapon meshes in the project; +Z = the knuckle line
Pinky1 -> Index1 of the grip, checked by 'axis'): blunt 0.05..0.75 m r 0.025 (longest plausible one-handed club / machete / cane head
beyond the fist; on the NPC donors 0.45 -> 0.75 m adds 9 penetrating frames of 3203, survey\axis_woman.json), baton 0.03..0.50 m r 0.018 (extended telescopic baton).
Weapon proxy vs body (npc_blade_common.blade_pen, replaced on import by blade_pen_release): the left hand also counts as on the
shaft on the 2 frames next to a two-hand grip while its IK weight is above 0 (fingers leaving / reaching the shaft), for a graze of at
most 0.5 cm inside the proxy; deeper is counted.
Vanilla baton clips: Build\\work\\melee-npc-blunt-20261109\\vanilla-dump\\npz (npc_blunt_dump.py); every other set unchanged
(melee_common.clip_path default).  OUTW = Build\\work\\melee-npc-blunt-20261109 (npc_blade_common.OUTW is overridden in this process).
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import melee_common as MC  # noqa: E402

OUTW = os.path.join(MC.ROOT, 'Build', 'work', 'melee-npc-blunt-20261109')
VDUMP = os.path.join(OUTW, 'vanilla-dump', 'npz')
BATON_SETS = ('pwa_baton', 'pma_baton')
_clip_path0 = MC.clip_path


def _clip_path(set_name, clip, base=None):
    if base is None and set_name in BATON_SETS:
        return os.path.join(VDUMP, set_name, clip + '.npz')
    return _clip_path0(set_name, clip, base)


MC.clip_path = _clip_path
for _s in BATON_SETS:
    MC.RIG_OF_SET.setdefault(_s, 'man' if _s.startswith('pma_') else 'woman')

import npc_blade_common as BC  # noqa: E402  (imports npc_build -> base_body, which consumes --rig)

BC.OUTW = OUTW
BC.WEAPONS['blunt'] = dict(player={'woman': 'pwa_one_handed_blunt', 'man': 'pma_one_handed_blunt'}, action='gang_onehanded_action_combat',
                           loco=('gang_onehanded_locomotion_combat',), two_hand=False)
BC.WEAPONS['baton'] = dict(player={'woman': 'pwa_baton', 'man': 'pma_baton'}, action='gang_onehanded_action_combat',
                           loco=('gang_onehanded_locomotion_combat',), two_hand=False)
BC.BLADE['blunt'] = (0.05, 0.75, 0.025)
BC.BLADE['baton'] = (0.03, 0.50, 0.018)
_vindex0 = BC.vanilla_index


def _vanilla_index(set_name):
    if set_name in BATON_SETS:
        return json.load(open(os.path.join(VDUMP, set_name, '_index.json')))
    return _vindex0(set_name)


BC.vanilla_index = _vanilla_index


REL_PAD = 2                               # frames next to a two-hand grip (IK weight > 0.5) on which the releasing / reaching left hand may touch the shaft
GRAZE_CM = 0.5                            # ... by at most this depth inside the proxy (contact); deeper = fingers through the weapon: counted
_blade_pen0 = BC.blade_pen


def blade_pen_release(W, weapon, two_hand=None, detail=False):
    """npc_blade_common.blade_pen; the left hand also counts as on the shaft on the REL_PAD frames before / after a two-hand grip
    (IK weight > 0.5) while its IK weight is still above 0 - but only for a graze: on those frames a left-hand vertex deeper than
    GRAZE_CM inside the proxy still counts (the original result of that frame is kept).  b2: LeftHandMiddle2/3 0.1-0.3 cm inside the
    2.5 cm proxy on the release frame of block_end / block_deflect_* (contact, passes); m1: Male combo / final attacks with the left
    fingers 1.5-2.3 cm inside at IK weight 0.36-0.40 (through the weapon, fails).  Every other body part and frame unchanged."""
    if two_hand is None:
        return _blade_pen0(W, weapon, two_hand, detail)
    w = np.asarray(two_hand, float) * np.ones(len(W))
    on = w > 0.5
    near = on.copy()
    for o in range(1, REL_PAD + 1):
        near[o:] |= on[:-o]
        near[:-o] |= on[o:]
    pad = near & (w > 0) & ~on
    if not pad.any():
        return _blade_pen0(W, weapon, w, detail)
    a = _blade_pen0(W, weapon, np.where(pad, 1.0, w), detail=True)
    b = _blade_pen0(W, weapon, w, detail=True)
    deep = pad & (b[1] < -GRAZE_CM)
    out = tuple(np.where(deep if x.ndim == 1 else deep[:, None], y, x) for x, y in zip(a, b))
    return out if detail else out[:2]


if not getattr(BC.blade_pen, '_release', False):
    blade_pen_release._release = True
    BC.blade_pen = blade_pen_release
NB, NR, P = BC.NB, BC.NR, BC.P
IX = P.ix
RIG = BC.RIG


def knuckle_in_wr(W, ix=None):
    """unit knuckle line RightHandPinky1 -> RightHandIndex1 and the palm centre in the WeaponRight frame [T,3] (x, y, z)."""
    ix = ix or IX
    M = np.linalg.inv(W[:, ix['WeaponRight']])
    a = np.einsum('tij,tj->ti', M[:, :3, :3], W[:, ix['RightHandIndex1'], :3, 3] - W[:, ix['RightHandPinky1'], :3, 3])
    a /= np.linalg.norm(a, axis=1, keepdims=True)
    pc = (W[:, ix['RightHand'], :3, 3] + W[:, ix['RightHandMiddle1'], :3, 3]) / 2
    p = np.einsum('tij,tj->ti', M[:, :3, :3], pc - W[:, ix['WeaponRight'], :3, 3])
    return a, p


def axis(a):
    import glbanim as GA
    rep = dict(rig=RIG, vanilla={}, npc={}, proxy_pen={})
    for w in ('blunt', 'baton'):
        st = BC.player_set(w)
        for clip in ('melee_idlepose', 'combo_attack_01', 'heavy_attack_01'):
            d, _ = BC.vanilla(st, clip)
            W = MC.world_all(d)
            k, p = knuckle_in_wr(W)
            G = np.linalg.inv(W[:, IX['RightHand']]) @ W[:, IX['WeaponRight']]
            rep['vanilla']['%s/%s' % (st, clip)] = dict(knuckle_in_WR_f0=np.round(k[0], 3).tolist(), knuckle_z_min=round(float(k[:, 2].min()), 3),
                                                       palm_in_WR_cm_f0=np.round(p[0] * 100, 1).tolist(), WR_t_in_hand_cm_f0=np.round(G[0, :3, 3] * 100, 2).tolist())
    sn = BC.npc_set('blunt')
    raw = BC.raw_npc(sn, 'idle_combat')
    k, p = knuckle_in_wr(raw['W'], raw['ix'])
    G = np.linalg.inv(raw['W'][:, raw['ix']['RightHand']]) @ raw['W'][:, raw['ix']['WeaponRight']]
    rep['npc'][sn + '/idle_combat'] = dict(knuckle_in_WR_f0=np.round(k[0], 3).tolist(), palm_in_WR_cm_f0=np.round(p[0] * 100, 1).tolist(),
                                           WR_t_in_hand_cm_f0=np.round(G[0, :3, 3] * 100, 2).tolist())
    # NPC donors retargeted to V (BSeq): weapon proxy vs V's body proxy, several lengths (from WeaponRight along +Z)
    px = BC.proxy()
    names = np.array(px.names)
    excl = np.isin(names[px.lab], BC.HAND_EXCL['Right'])
    sk = NR.skeleton(os.path.join(NR.NPC_DIR, sn + '.anims.glb'))
    tips = (0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75)
    cnt = {t: 0 for t in tips}
    per = {}
    nfr = 0
    for clip in GA.clips(sk.gj):
        if not any(s in clip for s in ('combo', 'heavy', 'charge', 'jump_attack', 'parry', 'block', 'deflect', 'idle_combat', 'ovr_idle', 'equip')):
            continue
        sq = BC.BSeq([(sn, clip)])
        W = sq.W
        hit = {t: 0 for t in tips}
        for t in range(len(W)):
            V = px.verts(W[t, :73])[~excl]
            M = W[t, BC.WR]
            for tp in tips:
                a0 = M[:3, 3] + 0.05 * M[:3, 2]
                b0 = M[:3, 3] + tp * M[:3, 2]
                ab = b0 - a0
                u = np.clip(((V - a0) @ ab) / float(ab @ ab), 0, 1)
                d = np.linalg.norm(V - (a0 + u[:, None] * ab), axis=1)
                if (d < 0.025).any():
                    hit[tp] += 1
        nfr += len(W)
        for tp in tips:
            cnt[tp] += hit[tp]
        per[clip] = dict(frames=len(W), **{'pen_%02d' % int(tp * 100): hit[tp] for tp in tips})
    rep['proxy_pen'] = dict(radius_m=0.025, start_m=0.05, frames_total=nfr, frames_with_pen={'%.2f' % t: cnt[t] for t in tips}, per_clip=per)
    MC.jdump(rep, os.path.join(OUTW, 'survey', 'axis_%s.json' % RIG))
    print(json.dumps({k: v for k, v in rep.items() if k != 'proxy_pen'}, indent=1))
    print('PROXY PEN frames (V body vs capsule 0.05..L r 0.025) of %d NPC donor frames:' % nfr, rep['proxy_pen']['frames_with_pen'])
    worst = sorted(per.items(), key=lambda kv: -kv[1]['pen_70'])[:12]
    for c_, v in worst:
        print('  %-36s %s' % (c_, v))


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'axis':
        axis(sys.argv[1:])
    else:
        {'survey': BC.survey, 'grip': BC.grip}[cmd](sys.argv[1:])
