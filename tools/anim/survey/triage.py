"""Stage 1 of the survey: classify every registered player animset (317) by its visibility in UTPP third person, from code evidence.

  python -I triage.py      -> Build\\work\\anim-survey-20261109\\triage_sets.csv, measure_<rig>.txt (depot<TAB>clip regex), report.txt block
Classes: A = full body shown in TPP (PsmBlock release states), B = upper body shown over MM (mask / release / weapon-action states not
covered by UTPP), C = replaced or hidden by UTPP (MM, ALS overlays, the .xl overrides), D = never visible in TPP (camera / procedural /
additive / no clips), E = unknown (+ the in-game check that settles it).  Visibility is DERIVED FROM CODE (file:line), not seen in game.
FPP-authored: player FPP rig sets (p?a_, player\\female|male) = FPP unless tpp_ / player_tpp_head; confirmed per clip in the dumps by the
Torso_fppCamera_* tracks (survey_dump.py, column fpp_cam in survey_clips.csv).
"""
import csv
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import survey_common as SC  # noqa: E402
from survey_dump import setkey  # noqa: E402

GI = 'Source\\plugin\\UTPP\\src\\mm\\GameIntegration.hpp'
V2 = 'Source\\plugin\\UTPP\\scripts\\utpp_motion.reds'
DRV = 'Source\\plugin\\UTPP\\src\\MotionDriver.cpp'
EV = {
    'release': GI + ':973 kPsmReleaseMask (vehicle|scene|swim|workspot|takedown|carry|vitals) + :978 kPsmReleaseDetails (10..13 ladder, 29 knockdown, 31 felled)',
    'hold': GI + ':964-971 hold last pose (jump/fall/slide/dodge/AimWalk/stagger/menu are HELD by MM, game pose not shown); :979-987 air/land ids',
    'trav': GI + ':1006-1024 TravReleases: climb 8 / vault 9 released to the game when v2 traversal does not own the episode',
    'carry': GI + ':106 kPsmCarry (BodyCarrying / Carrying) in the release mask; ' + V2 + ':634-637 carry -> release',
    'takedown': GI + ':105 kPsmTakedown (grapple, leap, takedown) in the release mask',
    'workspot': GI + ':104 kPsmWorkspot in the release mask',
    'scene': GI + ':102 kPsmScene / :104 kPsmWorkspot in the release mask',
    'vehicle': GI + ':101 kPsmVehicle in the release mask (car / bike seats, combat from the seat)',
    'swim': GI + ':103 kPsmSwimming; death: :108 kPsmVitals',
    'gamepose': V2 + ':412-428 melee + arm cyberware rows -> UTPPMotionPoseGame; :668-676 release; ' + DRV + ':2179 g_release (game owns the upper body)',
    'melee_mask': GI + ':114 kPsmMeleeAttack (incl. RangedWeapon QuickMelee) -> kGateMelee; :893 kMaskStateMelee; :936-962 DecideUpperMask (upper body from the game)',
    'grenade': V2 + ':640-649 CombatGadget 3 -> Pistol1H aim overlay (hidden), 4 / 6 -> release (game upper body: the throw)',
    'launcher': V2 + ':651-656 LeftHandCyberware 5/6/8/9 -> Pistol1H aim overlay (hidden)',
    'reload': DRV + ':5292 reloadHandback + :5305-5313 BuildWeaponActionMask (clavicles, arms, hands, weapon from the game; spine/head MM); '
              + GI + ':838-855 IsWeaponActionBone; female Nue: utpp_nue_reload.archive.xl (authored TPP reload, Nue only)',
    'overlay': V2 + ':398-411 firearm rows -> Rifle / Pistol overlays (ALS, MM owns the body); cet\\utpp_motion\\init.lua:749 aimGameUpper default false',
    'meleexl': 'Source\\plugin\\UTPP\\archive\\utpp_melee_tpp.archive.xl (12 sets, third-person clips, MeleeFullBody.hpp)',
    'vehxl': 'Source\\plugin\\UTPP\\archive\\utpp_vehicle_tpp.archive.xl (tpp bike sets; clips missing there - enter_l/r, idle, slide_exit, combat_* - still come from the FPP set)',
    'footxl': 'Source\\plugin\\UTPP\\archive\\utpp_foot_tpp.archive.xl (TPP idle / security cam)',
    'loco': GI + ':130-136 LocoState Stand/Crouch/Sprint owned by MM (GASP-ALS); FPP locomotion legs only visible inside release states (carry)',
    'default': V2 + ':657-666 empty hands / no weapon -> default overlay: MM owns the whole body (no gate reason for consumables)',
    'additive': 'additive clips only (camera / procedural layers): no visible pose of their own',
    'noclips': 'set has no clips (look-at / ep1 stub)',
}

# (path regex, class, state, evidence key, visible-clip regex (None = all non-additive), in-game check for E, note)
RULES = [
    (r'look_at|head_bobbing', 'D', 'camera', 'additive', None, '', 'camera / look-at layer'),
    (r'_procedural(_weapon)?\.anims$', 'D', 'procedural', 'additive', None, '', 'procedural additive layer'),
    (r'\\tpp_p[wm]a_sportbike', 'C', 'vehicle bike (TPP set)', 'vehxl', None, '', 'third-person authored; UTPP registers it for TPP riding'),
    (r'\\tpp_p[wm]a_security_cam', 'C', 'security cam (TPP set)', 'footxl', None, '', 'third-person flag set'),
    (r'\\sportbike\d?\\character\\p[wm]a_sportbike', 'A', 'vehicle bike (FPP set)', 'vehxl', r'^(combat|sit_to_combat|combat_to_sit|idle|slide_exit|enter_l|enter_r)', '',
     'riding / mount / dismount replaced by the TPP set + UTPP exit; FPP leftovers visible'),
    (r'vehicle\\player_default_vehicle', 'A', 'vehicle combat', 'vehicle', None, '', 'combat from the seat (driver / turret)'),
    (r'vehicle\\(sport|standard|utility|generic)\\', 'A', 'vehicle car seat', 'vehicle', r'^(?!enter|exit|cool_exit|combat_exit|enter_fast)', '',
     'car entry / exit handled by UTPP (C); seat clips visible as in the vanilla vehicle TPP camera'),
    (r'weapon\\(default_melee_weapon|pma_default_melee_weapon)\.anims$|\\p[wm]a_(fists|katana|knife|one_handed_blunt|two_handed_blunt)\.anims$', 'C', 'melee',
     'meleexl', None, '', 'UTPP third-person melee clips (melee export)'),
    (r'one_handed_blunt_dildo', 'B', 'melee equip', 'melee_mask', None, '', 'melee_first_equip only'),
    (r'katana_finisher|mantisblade_finisher', 'A', 'takedown / finisher', 'takedown', None, '', ''),
    (r'weapon\\(baton|two_handed_hammer)\\', 'B', 'melee (not covered by the melee xl)', 'melee_mask', None, '', 'vanilla FPP melee upper body'),
    (r'cyberware\\(mantisblade|monowire)\\|strongarms_melee_fists', 'B', 'arm cyberware weapon', 'gamepose', None, '', 'vanilla FPP upper body (release)'),
    (r'cyberware\\strongarms\\p[wm]a_strongarms\.anims$|doors\\p[wm]a_strongarms_no_cyberware', 'E', 'force door / move object', 'workspot', None,
     'force a door / push a dumpster in TPP; UTPP status psm bits: workspot (0x8) or takedown -> A (full body), none -> hidden (MM)', ''),
    (r'cyberware\\launcher', 'C', 'projectile launcher', 'launcher', None, '', 'aim overlay while charging / firing'),
    (r'cyberware\\(default_cw_player|pma_default_cw)\.anims$', 'E', 'generic cyberware', 'default', None,
     'activate an arm cyberware without its own set in TPP; if the arms move (not the MM idle), the game upper body is shown', ''),
    (r'personal_link', 'A', 'workspot (jack-in) / takedown', 'workspot', None, '', ''),
    (r'items\\interactive\\|items\\environment\\', 'E', 'interaction', 'workspot', None,
     'run the interaction in TPP; UTPP status psm bit workspot (0x8) -> A, otherwise hidden by MM', ''),
    (r'consumable|data_splinter', 'C', 'consumable (hidden per code)', 'default', None,
     'use an inhaler / injector in TPP: if the arm reaches the mouth / neck, the game upper body is shown -> B', 'verify in game'),
    (r'items\\player\\grenade', 'B', 'grenade', 'grenade', r'throw|quick_throw', '', 'charge / idle on the aim overlay (C)'),
    (r'body_carry|\\q306\\', 'A', 'body carry', 'carry', None, '', ''),
    (r'takedown|cerberus', 'A', 'takedown', 'takedown', None, '', ''),
    (r'sniper_nest|interactive_scene\\', 'A', 'scene / workspot', 'scene', None, '', ''),
    (r'player\\(female|male)\\body\\player_exploration', 'A', 'exploration (mixed)', 'release',
     r'^(ladder|knockdown|vehicle_knockdown|death|fpp_swim|fpp_idle_swim|vault|climb)', '',
     'ladder / knockdown / death / swim = A; vault / climb = E (TravReleases); jump / fall / land / dodge / slide / thrusters = C (held); bike exits = C'),
    (r'player_locomotion_box', 'A', 'carry box', 'carry', None, '', 'PSM.Carrying'),
    (r'player_locomotion_(johnny_guitar|sandstorm|sick)', 'E', 'quest locomotion', 'loco', None,
     'quest-specific walk; visible only if the quest sets a release reason (scene / workspot / carry) - check the UTPP status psm bits in that quest', ''),
    (r'player\\(female|male)\\body\\player_locomotion\.anims$', 'C', 'locomotion', 'loco', r'^fpp_(idle_stand|walk_slow_0|walk_fast_0|crouch_idle|crouch_walk_0|jog_0)$', '',
     'MM owns locomotion; the FPP legs show only under carry (A) - a few clips measured'),
    (r'weapon\\(default_weapon|pma_default_weapon)\.anims$', 'B', 'firearm action (reload / quick melee)', 'reload', r'reload|^melee$', '',
     'idle / aim / shoot / equip on the ALS overlays (C)'),
    (r'weapon\\firearms\\', 'B', 'firearm action (reload / quick melee)', 'reload', r'reload|^melee$', '', 'idle / aim / shoot / equip on the ALS overlays (C)'),
]
CLIP_CLASS_EXPLORATION = [(r'^(ladder|knockdown|vehicle_knockdown|death|fpp_swim|fpp_idle_swim)', 'A'), (r'^(vault|climb)', 'E'),
                          (r'^exit_driver_sportbike', 'C'), (r'.', 'C')]


def classify(depot):
    for rx, cls, state, ev, sub, check, note in RULES:
        if re.search(rx, depot, re.I):
            return cls, state, ev, sub, check, note
    return 'E', 'unclassified', 'default', None, 'no rule: inspect the set in game', ''


def main():
    doc = json.load(open(SC.SETS_JSON, encoding='utf-8'))
    rows = {}
    for ent, lst in doc.items():
        for r in lst:
            k = r['set'].lower()
            e = rows.setdefault(k, dict(ents=set(), pr=set(), vars=set()))
            e['ents'].add(ent)
            e['pr'].add(int(r['priority']))
            e['vars'].update(r.get('vars') or [])
    out, meas = [], {'woman': [], 'man': []}
    keys = {}
    for depot in sorted(rows):
        e = rows[depot]
        js = SC.load_set_json(depot)
        rig, cl = SC.clips_of(js) if js else ('', [])
        nadd = sum(1 for c in cl if c['type'] in ('Additive', 'AdditiveWithoutFirstFrame'))
        cls, state, ev, sub, check, note = classify(depot)
        if not cl or nadd == len(cl):
            if cls not in ('C',):
                cls, ev, note = 'D', ('noclips' if not cl else 'additive'), (note + '; ' if note else '') + ('no clips' if not cl else 'additive only')
        body = 'man' if 'player_man_skeleton' in rig else 'woman'
        tpp = 'tpp_' in os.path.basename(depot) or 'player_tpp_head' in e['vars']
        fpp = 'TPP (name / player_tpp_head var)' if tpp else ('FPP rig (player FPP skeleton; per-clip check in dumps)' if 'player_' in rig else 'unknown')
        nvis = len([c for c in cl if c['type'] not in ('Additive', 'AdditiveWithoutFirstFrame') and (sub is None or re.search(sub, c['name']))])
        sk = setkey(depot)
        assert sk not in keys, (sk, depot, keys.get(sk))
        keys[sk] = depot
        out.append(dict(body=body, set=depot, setkey=sk, entities='+'.join(sorted(x.replace('player_', '') for x in e['ents'])),
                        priority=','.join(map(str, sorted(e['pr']))), vars=' '.join(sorted(e['vars'])), rig=os.path.basename(rig), clips=len(cl),
                        additive=nadd, cls=cls, state=state, visible_clips=nvis, clip_filter=sub or '', fpp_authored=fpp,
                        evidence=EV[ev], in_game_check=check, note=note))
        if cls in ('A', 'B', 'E') and not tpp and nvis > 0:
            meas[body].append((depot, sub or '.'))
        elif cls == 'C' and state.startswith(('consumable', 'locomotion')) and nvis > 0:      # cheap extra: hidden per code, measured anyway
            meas[body].append((depot, sub or '.'))
    p = os.path.join(SC.WORK, 'triage_sets.csv')
    with open(p, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader()
        w.writerows(out)
    for b, lst in meas.items():
        with open(os.path.join(SC.WORK, 'measure_%s.txt' % b), 'w', encoding='utf-8') as f:
            for depot, rx in lst:
                f.write('%s\t%s\n' % (depot, rx))
    # summary
    lines = ['sets: %d (woman rig %d, man rig %d)' % (len(out), sum(r['body'] == 'woman' for r in out), sum(r['body'] == 'man' for r in out))]
    for cls in 'ABCDE':
        rs = [r for r in out if r['cls'] == cls]
        lines.append('class %s: %3d sets (woman %d / man %d), clips %d, visible clips %d' % (
            cls, len(rs), sum(r['body'] == 'woman' for r in rs), sum(r['body'] == 'man' for r in rs), sum(r['clips'] for r in rs),
            sum(r['visible_clips'] for r in rs)))
    st = {}
    for r in out:
        st.setdefault((r['cls'], r['state']), []).append(r)
    lines.append('by class / state (sets, visible clips):')
    for (cls, state), rs in sorted(st.items()):
        lines.append('  %s  %-40s %3d sets  %4d visible clips' % (cls, state, len(rs), sum(r['visible_clips'] for r in rs)))
    lines.append('TPP-authored (excluded from measurement): %d sets' % sum('TPP' in r['fpp_authored'] for r in out))
    lines.append('to measure: woman %d sets (%s), man %d sets' % (len(meas['woman']), 'measure_woman.txt', len(meas['man'])))
    lines.append('E checks:')
    for r in out:
        if r['cls'] == 'E' and r['body'] == 'woman':
            lines.append('  %s : %s' % (r['setkey'], r['in_game_check']))
    print('\n'.join(lines))
    SC.report('STAGE 1 TRIAGE (triage_sets.csv; visibility derived from code, not seen in game)', '\n'.join(lines))


if __name__ == '__main__':
    main()
