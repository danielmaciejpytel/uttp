"""Shared code of the dual-wield TPP pilot built from the game's NPC animations (2026-11-09).

Run with Tools\\ai-motion\\envs\\kimodo\\Scripts\\python.exe -I.  Imports the melee NPC tools (Source\\tools\\melee: npc_retarget, npc_build,
melee_common, anatomy_gate_base ...) WITHOUT editing them; the only runtime adaptation is npc_retarget.NPC_DIR, pointed at the dual-wield
extraction (Extracted\\game\\npc-dualwield\\glb, extract_npc_dualwield.ps1).
Reads: Extracted\\game\\npc-dualwield\\glb (NPC sets with root motion + the Female V player dual-wield sets), the player reference rig of the
melee tools (Build\\work\\melee-anim-20261107\\glb\\pwa_fists.anims.glb, 152 joints).  Writes nothing by itself.
Axes: game (z up, the character faces +Y, +X = her right), metres, 30 fps.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
MELEE = os.path.join(ROOT, 'Source', 'tools', 'melee')
sys.path.insert(0, MELEE)
import melee_common as MC  # noqa: E402
import npc_retarget as NR  # noqa: E402

NPC_GLB = os.path.join(ROOT, 'Extracted', 'game', 'npc-dualwield', 'glb')
NPC_JSON = os.path.join(ROOT, 'Extracted', 'game', 'npc-dualwield', 'json')
NR.NPC_DIR = NPC_GLB                    # runtime redirect of the donor folder (module global read by NR.Donor); the file is not changed
OUT = os.path.join(ROOT, 'Build', 'work', 'dualwield-npc-20261109')
MEDIA_MP4 = os.path.join(ROOT, 'Media', 'mp4', '2026-11-09_dualwield-npc')
MEDIA_PNG = os.path.join(ROOT, 'Media', 'png', '2026-11-09_dualwield-npc')

# Female V player dual-wield sets of the pilot (vanilla; priority 64 < 100 < 200 = the first one that has the clip wins)
PLAYER_SETS = ['pwa_w_handgun_dual_wield_right__militech_lexington', 'pwa_w_handgun_dual_wield_left__militech_lexington',
               'pwa_w_handgun_dual_wield_base', 'pwa_w_default_weapon_dual_wield']
NPC_DUAL = {'man_average': 'ma_gang_handgundual_action_combat', 'man_big': 'mb_gang_handgundual_action_combat'}
NPC_ONEHAND = {'woman_right': 'wa_gang_handgunright_wounded_left_arm_actions', 'woman_left': 'wa_gang_handgunleft_wounded_right_arm_actions',
               'man_right': 'ma_gang_handgunright_wounded_left_arm_actions', 'man_left': 'ma_gang_handgunleft_wounded_right_arm_actions',
               'woman_both': 'wa_gang_handgunboth_action_combat'}
SIDES = ('Left', 'Right')
# hand IK helper chain of the player rig (arms_root_space (Spine3) -> ... -> arms_gun_space_offset -> arm_<side>_ik -> _offset -> _weapon)
HELPERS = {s: ['arm_%s_ik' % s.lower(), 'arm_%s_ik_offset' % s.lower(), 'arm_%s_ik_weapon' % s.lower(),
               'Torso_%sArm_Hand_IK_JNT' % s, 'Torso_World_%sArm_Hand_IK_JNT' % s] for s in SIDES}
_P = {}
_SETCLIPS = {}


def player(rig='woman'):
    if rig not in _P:
        _P[rig] = NR.Player(rig)
    return _P[rig]


def set_clips(set_name):
    if set_name not in _SETCLIPS:
        sk = NR.skeleton(os.path.join(NPC_GLB, set_name + '.anims.glb'))
        _SETCLIPS[set_name] = set(NR.GA.clips(sk.gj))
    return _SETCLIPS[set_name]


def find_player_set(clip):
    for s in PLAYER_SETS:
        if clip in set_clips(s):
            return s
    raise KeyError(clip)


def anim_info(set_name, clip):
    """(animationType, duration, events [(type, name, start, duration)]) from the serialized .anims json."""
    d = json.load(open(os.path.join(NPC_JSON, set_name + '.anims.json'), encoding='utf-8'))
    rc = d['Data']['RootChunk']
    for a in rc['animations']:
        ent = a['Data']
        an = ent['animation']['Data']
        if an['name']['$value'] != clip:
            continue
        evs = []
        ev = ent.get('events')
        if isinstance(ev, dict):
            for e in (ev.get('Data') or {}).get('events') or []:
                e = e.get('Data', e)
                nm = e.get('eventName')
                nm = nm.get('$value') if isinstance(nm, dict) else nm
                evs.append((e.get('$type', ''), nm, int(e.get('startFrame', 0)), int(e.get('durationInFrames', 0) or 0)))
        return an['animationType'], float(an['duration']), evs
    raise KeyError((set_name, clip))


def player_clip(clip, set_name=None, rig='woman'):
    """vanilla player clip on the 152-joint rig: game-axes locals lt [T,152,3], lr [T,152,4], world W [T,152,4,4] (Root as exported)."""
    P = player(rig)
    set_name = set_name or find_player_set(clip)
    sk = NR.skeleton(os.path.join(NPC_GLB, set_name + '.anims.glb'))
    assert list(sk.names) == P.names, 'player dual-wield glb joint order differs from the reference rig'
    lt, lr, ex, dur = NR.sample_clip(sk, clip)
    lt, lr = NR.locals_to_game(lt, lr)
    return dict(lt=lt, lr=lr, W=MC.fk_world(P.par, lt, lr), extras=ex, dur=dur, set=set_name)


def npc_raw(set_name, clip):
    """NPC clip on the NPC's OWN skeleton (game axes): world W [T,N,4,4] with Root = identity, locals, name index, root path (game)."""
    nsk = NR.skeleton(os.path.join(NPC_GLB, set_name + '.anims.glb'))
    nlt, nlr, ex, dur = NR.sample_clip(nsk, clip)
    lt, lr = NR.locals_to_game(nlt, nlr)
    ri = nsk.index['Root']
    rt, rq = lt[:, ri].copy(), lr[:, ri].copy()
    lt[:, ri] = 0
    lr[:, ri] = [0, 0, 0, 1.0]
    return dict(W=MC.fk_world(np.array(nsk.parent), lt, lr), lt=lt, lr=lr, ix=dict(nsk.index), names=list(nsk.names), parent=list(nsk.parent),
                root_t=rt, root_q=rq, extras=ex, dur=dur, sk=nsk)


def pos(W, ix, n):
    return W[:, ix[n], :3, 3]


def body_frame(W, ix):
    """hips heading frame [T,3,3] (columns right, forward, up) and Spine3 position."""
    return MC.heading_frame(pos(W, ix, 'LeftUpLeg'), pos(W, ix, 'RightUpLeg')), pos(W, ix, 'Spine3')


def rel_body(W, ix, p):
    """world points [T,3] -> hips heading frame relative to Spine3 (x right, y forward, z up)."""
    H, s3 = body_frame(W, ix)
    return np.einsum('tji,tj->ti', H, p - s3)


def jdump(obj, path):
    MC.jdump(obj, path)
