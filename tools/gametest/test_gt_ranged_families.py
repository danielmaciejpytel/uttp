"""Offline tests for the ranged weapon family set (gt_ranged_families.py) and the runner parts it touches (phase windows of the pose
meter and the contact sheets, thresholds of the family copies). No input is sent, no process is started.

  python Source\\tools\\gametest\\test_gt_ranged_families.py -v
"""
from __future__ import annotations

import ctypes
import os
import re
import sys
import tempfile
import types
import unittest
from datetime import datetime
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_melee_weapons as gw  # noqa: E402
import gt_night as gn  # noqa: E402
import gt_post  # noqa: E402
import gt_posemeter as gp  # noqa: E402
import gt_ranged_families as rf  # noqa: E402
import gt_settings  # noqa: E402
import run as gt_run  # noqa: E402
from gt_input import file_size, read_since  # noqa: E402
from gt_scenario import BUTTONS, load_scenarios  # noqa: E402

PREFIX = '[2026-11-14 10:00:{s:06.3f}] [info    ] [ 1] [UTPP] UTPP '
ORIG_HASH, GIVEN_HASH = 1111, 2222


def stamp(sec: float, text: str) -> str:
    return PREFIX.format(s=sec) + text


class _G:
    def wait(self, s):
        pass

    def check(self):
        pass


class _Pad:
    def release(self):
        pass


class FakeNight:
    """The parts of the night runner the macro uses; `script(event, cmd)` gives the log lines of an event (None = lost)."""

    def __init__(self, tmp: str, script, args=None):
        self.utpp_log = os.path.join(tmp, 'session.log')
        open(self.utpp_log, 'w').close()
        self.scenarios = load_scenarios(gw.SCENARIO_FILE)
        self.scenario_deadline = 99.0
        self.args = args or types.SimpleNamespace(ranged_families='', ranged_records=[])
        self.script = script
        self.logs: list = []
        self.calls: list = []
        self.sent: list = []
        self.clock = 0.0
        self.settings = gt_settings.Ledger(None, log=lambda m: None)

    def log(self, msg):
        self.logs.append(msg)

    def append(self, lines):
        with open(self.utpp_log, 'a', encoding='utf-8') as f:
            for ln in lines or []:
                self.clock += 0.05
                f.write(stamp(self.clock, ln) + '\n')

    def play(self, pad, guard, name, plan, sc=None):
        self.calls.append(('play', name))
        now = datetime.now()
        plan['runs'].append({'scenario': name, 'run': 1, 'aborted': None,
                             'steps': [{'name': 'step', 'start': now, 'end': now, 'metrics': True}]})
        self.append(self.script('play:' + name, ''))
        return {'steps': []}

    def play_walking(self, pad, guard, plan, name, key_crouch=False, clear=True):
        self.calls.append(('walk', name, clear))
        plan['runs'].append({'scenario': name, 'run': 1, 'steps': []})
        off = file_size(self.utpp_log)
        self.append(self.script('play:' + name, ''))
        return {'res': {}, 'cfails': [], 'off': off, 'start': datetime.now(), 'discarded': None}

    def console_verified(self, pad, guard, cmd, echo, label, timeout=3.0):
        tag = re.search(r"\[night\] (RF_\w+) ok=", cmd).group(1)
        self.calls.append(('cmd', tag))
        self.sent.append((tag, cmd))
        off = file_size(self.utpp_log)
        lines = self.script(tag, cmd)
        if lines is None:
            return None
        self.append(lines)
        return read_since(self.utpp_log, off)

    def cmds(self):
        return [c[1] for c in self.calls if c[0] == 'cmd']

    def plays(self):
        return [c[1] for c in self.calls if c[0] in ('play', 'walk')]


def good_script(tag='handgun', cap=12, how='given', pick='ok', lost_once=(), orig=True, mag_readable=True, shots_fired=True,
                reload_grows=True, never_drawn=False, draw_after=1, hand_ok=True, give_lost_after_run=False, pick_rec=None,
                seen=None):
    """The game as the macro sees it: a hand state, a magazine, an inventory. `lost_once` = tags whose first send is lost."""
    fam = rf.FAMILIES[tag]
    rec = pick_rec or fam['records'][0]
    st = {'hand': 'orig' if orig else None, 'mag': cap, 'cap': cap, 'reload_polls': None, 'sent': {}, 'reads': 0, 'drawn_polls': 0}

    def hand_line(cmd):
        h = {'orig': ORIG_HASH, 'weapon': GIVEN_HASH, None: 0}[st['hand']]
        if st['hand'] == 'weapon':
            st['drawn_polls'] += 1
            if never_drawn or st['drawn_polls'] <= draw_after - 1:
                h, st['hand_shown'] = ORIG_HASH, 'orig'
        match = 1 if (h == GIVEN_HASH and f"TweakDBID.new('{rec}')" in cmd) else 0
        mag = f'{st["mag"]}/{st["cap"]}' if mag_readable and st['hand'] == 'weapon' else '-/-'
        if st['reload_polls'] is not None and st['hand'] == 'weapon':
            st['reload_polls'] -= 1
            if st['reload_polls'] <= 0 and reload_grows:
                st['mag'] = st['cap']
        return f'CET [night] RF_HAND ok=true {tag} hash={h} match={match} mag={mag} total=250'

    def script(event, cmd):
        if event in lost_once and not st['sent'].get(event):
            st['sent'][event] = 1
            return None
        if event == 'RF_PICK':
            if pick == 'missing':
                return [f'CET [night] RF_PICK ok=false {tag} no usable record: ' + ','.join(r.replace('Items.', '') + '=missing'
                                                                                          for r in fam['records']) + ',']
            if pick == 'wrongtype':
                return [f'CET [night] RF_PICK ok=false {tag} no usable record: ' + ','.join(r.replace('Items.', '') + '=Wea_Melee'
                                                                                          for r in fam['records']) + ',']
            shown = seen if seen is not None else ','.join(r.replace('Items.', '') + '=' + fam['types'][0] for r in fam['records']) + ','
            return [f'CET [night] RF_PICK ok=true {tag} rec={rec} type={fam["types"][0]} n={0 if how == "given" else 1} ammo=40 '
                    f'seen={shown}']
        if event == 'RF_GIVE':
            st['hand'] = 'weapon'
            st['drawn_polls'] = 0
            if give_lost_after_run and not st['sent'].get('gave'):
                st['sent']['gave'] = 1                        # the give ran, its echo never reached the log
                return None
            if st['sent'].get('gave'):                        # sent again: the inventory already holds the weapon and the ammunition
                return [f'CET [night] RF_GIVE ok=true {tag} rec={rec} had n=1->1 ammo=300->300']
            return [f'CET [night] RF_GIVE ok=true {tag} rec={rec} {how} n={0 if how == "given" else 1}->1 ammo=40->300']
        if event == 'RF_HAND':
            return [hand_line(cmd)] if hand_ok else ['CET [night] RF_HAND ok=false boom']
        if event == f'play:rf_{tag}_hip_fire':
            st['mag'] -= 3 if shots_fired else 0
            return []
        if event == f'play:rf_{tag}_aim_fire':
            st['mag'] -= 3 if shots_fired else 0
            return []
        if event == f'play:rf_{tag}_reload':
            st['reload_polls'] = 2
            return []
        if event == 'RF_HOLSTER':
            st['hand'] = None
            return ['CET [night] RF_HOLSTER ok=true eqsystem']
        if event == 'RF_AMMO_OFF':
            return ['CET [night] RF_AMMO_OFF ok=true removed 262->40']
        if event == 'RF_REMOVE':
            return ['CET [night] RF_REMOVE ok=true removed 1->0 r=true']
        if event == 'RF_REDRAW':
            st['hand'] = 'orig'
            return ['CET [night] RF_REDRAW ok=true found=1']
        return []
    script.state = st
    return script


def combined(*scripts):
    """One script for several families: each family script answers its own events; the hand state is shared via the last one."""
    def script(event, cmd):
        for s in scripts:
            r = s(event, cmd)
            if r != []:
                return r
        return []
    return script


class Registration(unittest.TestCase):
    def test_set_resolves_and_validates(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        self.assertEqual(gn.resolve_sets('ranged_families'), ['ranged_families'])
        self.assertEqual(gn.validate_sets(['ranged_families'], scen), [])
        self.assertNotIn('ranged_families', gn.SETS['all'])
        self.assertEqual(gn.MACROS['ranged_families'], rf.PARTS)
        for part in rf.PARTS:
            self.assertIn(part, scen)
        self.assertEqual(set(rf.MOVING_PARTS.values()), set(rf.PARTS))

    def test_pad_parts_are_pad_only_and_the_melee_sets_are_untouched(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        for part in rf.PARTS:
            for st in scen[part].steps:
                self.assertTrue(all(b.button in BUTTONS for b in st.buttons), part)
        self.assertEqual(gn.MACROS['melee_katana'], gw.PARTS)
        self.assertEqual(gn.resolve_sets('melee_weapons'), ['melee_weapons'])

    def test_families_table(self):
        self.assertEqual(list(rf.FAMILIES), ['handgun', 'revolver', 'smg', 'assault_rifle', 'shotgun', 'double_shotgun',
                                             'precision_rifle', 'sniper_rifle', 'lmg'])
        self.assertNotIn('hmg', rf.FAMILIES)                       # picked up from the world, not given and reloaded like the others
        seen = {}
        for tag, f in rf.FAMILIES.items():
            self.assertTrue(f['records'] and all(re.fullmatch(r'Items\.Preset_\w+_Default', r) for r in f['records']), tag)
            self.assertTrue(f['types'] and all(t.startswith('Wea_') for t in f['types']), tag)
            self.assertIn(f['ammo'], ('Ammo.HandgunAmmo', 'Ammo.RifleAmmo', 'Ammo.ShotgunAmmo', 'Ammo.SniperRifleAmmo'))
            for r in f['records']:
                self.assertNotIn(r, seen, f'{r} listed for {tag} and {seen.get(r)}')
                seen[r] = tag
        self.assertIn('Wea_Rifle', rf.FAMILIES['assault_rifle']['types'])      # the assault rifles of game 2.5 have the Wea_Rifle type
        self.assertNotIn('Items.Preset_Kyubi_Default', seen)
        for banned in ('RocketLauncher', 'Tech_Sniper', 'HMG', 'Nue'):         # not given and reloaded like the family weapons
            self.assertFalse([r for r in seen if banned in r], banned)

    def test_evidence_tiers(self):
        self.assertEqual(rf.record_evidence('Items.Preset_Defender_Default'), 'live')
        self.assertEqual(rf.record_evidence('Items.Preset_Lexington_Default'), 'tweak')
        un = rf.unverified_records()
        self.assertIn('Items.Preset_Lexington_Default', un)
        self.assertNotIn('Items.Preset_Defender_Default', un)
        self.assertTrue(set(un) <= {r for f in rf.FAMILIES.values() for r in f['records']})

    def test_time_limit_grows_with_the_families(self):
        self.assertGreater(rf.time_limit_s(9), rf.time_limit_s(1))
        self.assertGreater(rf.time_limit_s(9), 180.0)               # a full set cannot run in the default scenario timeout


class Options(unittest.TestCase):
    def test_families(self):
        self.assertEqual(rf.parse_families(''), list(rf.FAMILIES))
        self.assertEqual(rf.parse_families('smg, handgun'), ['handgun', 'smg'])         # table order
        with self.assertRaises(ValueError):
            rf.parse_families('handgun,railgun')

    def test_records(self):
        self.assertEqual(rf.parse_records(['handgun=Items.A_B,Items.C']), {'handgun': ('Items.A_B', 'Items.C')})
        for bad in ('handgun', 'railgun=Items.A', 'handgun=', 'handgun=NoDot'):
            with self.assertRaises(ValueError, msg=bad):
                rf.parse_records([bad])

    def test_options_of_and_overrides(self):
        args = types.SimpleNamespace(ranged_families='handgun,lmg', ranged_records=['lmg=Items.Preset_MA70_Default'])
        fams, recs = rf.options_of(args)
        self.assertEqual(fams, ['handgun', 'lmg'])
        self.assertEqual(recs['lmg'], ('Items.Preset_MA70_Default',))
        self.assertEqual(recs['handgun'], rf.FAMILIES['handgun']['records'])
        self.assertEqual(rf.options_of(types.SimpleNamespace())[0], list(rf.FAMILIES))
        self.assertEqual(rf.option_errors(types.SimpleNamespace(ranged_families='x', ranged_records=[]))[0][:20], 'unknown ranged famil')

    def test_night_command_line(self):
        import night
        p = night.build_parser()
        a = p.parse_args(['--set', 'ranged_families', '--ranged-families', 'handgun,smg', '--ranged-records', 'smg=Items.Preset_Dian_Default',
                          '--ranged-records', 'handgun=Items.Preset_Nue_Default'])
        self.assertEqual(night.validate_args(night.finalize_args(a)), [])
        self.assertEqual(rf.options_of(a)[1]['handgun'], ('Items.Preset_Nue_Default',))
        bad = p.parse_args(['--ranged-families', 'handgun,laser'])
        self.assertTrue(any('laser' in e for e in night.validate_args(night.finalize_args(bad))))
        self.assertEqual(rf.options_of(p.parse_args([]))[0], list(rf.FAMILIES))

    def test_dry_run_lines_follow_the_options(self):
        args = types.SimpleNamespace(ranged_families='sniper_rifle', ranged_records=['sniper_rifle=Items.Preset_Ashura_Default'])
        fams, recs = rf.options_of(args)
        text = '\n'.join('\n'.join(rf.dry_run_lines(t, recs[t], load_scenarios(gw.SCENARIO_FILE))) for t in fams)
        self.assertIn('rf_sniper_rifle_hip_fire', text)
        self.assertIn('Items.Preset_Ashura_Default [tweak]', text)
        self.assertNotIn('Items.Preset_Grad_Default', text)
        self.assertNotIn('rf_handgun_', text)
        self.assertIn('static pad part ranged_walk_aimed', text)


class Commands(unittest.TestCase):
    def all_commands(self):
        for tag, f in rf.FAMILIES.items():
            yield rf.pick_cmd(f, f['records'])
            yield rf.give_cmd(f, f['records'][0])
            yield rf.hand_cmd(f['records'][0])
            yield rf.ammo_off_cmd(f, f['records'][0], 40)
            yield rf.remove_cmd(f['records'][0], 0)
        yield rf.hand_cmd(None)
        yield rf.holster_cmd()
        yield rf.redraw_cmd(1234)

    def test_echo_tags_and_no_paths(self):
        for cmd in self.all_commands():
            self.assertRegex(cmd, r"Game\.UTPPLogLine\('\[night\] RF_\w+ ok='")
            self.assertNotRegex(cmd, r'[A-Za-z]:\\')
            self.assertNotIn('\n', cmd)
            self.assertNotIn('MW_', cmd)
            self.assertLess(len(cmd), 2500)

    def test_pick_reads_only_and_gates_on_the_item_type(self):
        f = rf.FAMILIES['assault_rifle']
        cmd = rf.pick_cmd(f, f['records'])
        self.assertNotIn('GiveItem', cmd)
        self.assertIn("['Wea_Rifle']=true", cmd)
        self.assertIn("['Wea_AssaultRifle']=true", cmd)
        for r in f['records']:
            self.assertIn(r, cmd)
        self.assertIn('no usable record', cmd)

    def test_give_counts_inside_and_restores_nothing_it_did_not_add(self):
        f = rf.FAMILIES['shotgun']
        cmd = rf.give_cmd(f, 'Items.Preset_Crusher_Default')
        self.assertIn('if n0==0 then ts:GiveItem', cmd)                       # a repeated send gives nothing twice
        self.assertIn('DrawItemRequest', cmd)
        self.assertIn("'Ammo.ShotgunAmmo'", cmd)                              # fallback when the record has no ammunition type
        self.assertIn('Ammo():GetID()', cmd)
        self.assertIn('if a0<300 then', cmd)
        self.assertNotIn('Equip', cmd.replace('EquipmentSystem', ''))         # no loadout slot is touched
        off = rf.ammo_off_cmd(f, 'Items.Preset_Crusher_Default', 17)
        self.assertIn('local keep=17 ', off)
        self.assertIn('ts:RemoveItem(p,am,n0-keep)', off)
        self.assertIn('local keep=3 ', rf.remove_cmd('Items.X', 3).replace('local keep=3  ', 'local keep=3 '))

    def test_lua_syntax(self):
        dll = os.path.join(os.path.dirname(os.path.dirname(HERE)), '..', 'Tools', 'lua-5.1.5', 'lua51.dll')
        try:
            from utpp_paths import project_root
            dll = os.path.join(str(project_root()), 'Tools', 'lua-5.1.5', 'lua51.dll')
        except Exception:     # noqa: BLE001
            pass
        if not os.path.isfile(dll):
            self.skipTest('lua51.dll not available')
        lua = ctypes.CDLL(dll)
        lua.luaL_newstate.restype = ctypes.c_void_p
        lua.luaL_loadstring.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lua.lua_tolstring.restype = ctypes.c_char_p
        lua.lua_tolstring.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
        state = lua.luaL_newstate()
        for cmd in self.all_commands():
            rc = lua.luaL_loadstring(state, cmd.encode())
            self.assertEqual(rc, 0, lua.lua_tolstring(state, -1, None))

    def test_parsers(self):
        ok, info = gw.echo_of('x\nCET [night] RF_GIVE ok=true handgun rec=Items.A given n=0->1 ammo=40->300\n', 'RF_GIVE')
        self.assertTrue(ok)
        self.assertEqual(rf.parse_give(info), {'rec': 'Items.A', 'how': 'given', 'n_before': 0, 'n_after': 1, 'ammo_before': 40,
                                               'ammo_after': 300})
        self.assertEqual(rf.parse_pick('handgun rec=Items.A type=Wea_Handgun n=2 ammo=40 seen=A=Wea_Handgun,B=missing,C=Wea_Rifle,'),
                         {'rec': 'Items.A', 'type': 'Wea_Handgun', 'n': 2, 'ammo': 40,
                          'seen': [('A', 'Wea_Handgun'), ('B', 'missing'), ('C', 'Wea_Rifle')]})
        self.assertIsNone(rf.parse_pick('no usable record: A=missing,'))
        self.assertEqual(rf.candidate_notes([('A', 'Wea_Handgun'), ('B', 'missing'), ('C', 'Wea_Rifle'), ('D', 'Wea_Handgun')],
                                            ('Wea_Handgun',), 'Items.A'),
                         ['B: record not found', 'C: item type Wea_Rifle is not a type of the family (Wea_Handgun)'])
        self.assertEqual(rf.parse_hand('handgun hash=2222 match=1 mag=9/12 total=250'),
                         {'hash': 2222, 'match': True, 'mag': 9, 'cap': 12, 'total': 250})
        self.assertEqual(rf.parse_hand('hash=0 match=0 mag=-/- total=-'),
                         {'hash': 0, 'match': False, 'mag': None, 'cap': None, 'total': None})
        self.assertIsNone(rf.parse_hand('boom'))

    def test_skip_text_names_the_cause(self):
        f = rf.FAMILIES['revolver']
        t = rf.skip_text(f, f['records'], 'revolver no usable record: Preset_X_Default=missing,Preset_Y_Default=missing,')
        self.assertTrue(t.startswith('skip: record not found'))
        for r in f['records']:
            self.assertIn(r, t)
        t = rf.skip_text(f, f['records'], 'revolver no usable record: Preset_X_Default=Wea_Handgun,')
        self.assertIn('no candidate record has an item type of the family', t)
        self.assertIn('Wea_Revolver', t)


class PureLogic(unittest.TestCase):
    def test_plan_shots(self):
        self.assertEqual(rf.plan_shots(None), (3, 3))
        self.assertEqual(rf.plan_shots(12), (3, 3))
        self.assertEqual(rf.plan_shots(6), (3, 2))               # one round stays: the game would reload an empty revolver itself
        self.assertEqual(rf.plan_shots(7), (3, 3))
        self.assertEqual(rf.plan_shots(2), (1, 0))               # double-barrel shotgun
        self.assertEqual(rf.plan_shots(1), (1, 0))
        self.assertEqual(rf.plan_shots(4), (2, 1))
        for cap in (1, 2, 3, 5, 8, 100):
            hip, aim = rf.plan_shots(cap)
            self.assertLessEqual(hip + aim, max(1, cap))

    def test_fire_result(self):
        self.assertEqual(rf.fire_result(12, 9)['shots'], 3)
        self.assertIsNone(rf.fire_result(12, 9)['problem'])
        self.assertIn('no shot', rf.fire_result(12, 12)['problem'])
        self.assertIsNone(rf.fire_result(None, 9)['problem'])    # not readable: not judged
        self.assertIsNone(rf.fire_result(None, 9)['shots'])
        # the total decides when it is known: 3 shots emptied a revolver and the game reloaded it (magazine 3 -> 6)
        r = rf.fire_result(3, 6, 469, 466, 3)
        self.assertEqual(r['shots'], 3)
        self.assertIsNone(r['problem'])
        self.assertIn('reloaded by itself', r['note'])
        self.assertIn('no shot', rf.fire_result(2, 2, 196, 196, 1)['problem'])
        self.assertEqual(rf.fire_result(12, 9, 250, 250, 3)['shots'], 3)       # total not counted down: the magazine decides
        self.assertIsNone(rf.fire_result(None, None, 250, 250, 3)['problem'])    # nothing readable changed: not judged
        self.assertIsNone(rf.fire_result(1, 1, 196, 196, 0)['problem'])    # nothing planned: nothing judged

    def test_reload_result(self):
        self.assertEqual(rf.reload_result(6, 12, 12, [], 2.0)['detected'], True)
        self.assertEqual(rf.reload_result(6, 12, 12, [], 2.0)['via'], 'magazine')
        self.assertEqual(rf.reload_result(6, 6, 12, [1.9], 2.0)['via'], 'log')
        self.assertFalse(rf.reload_result(6, 6, 12, [], 8.0)['detected'])
        self.assertIsNone(rf.reload_result(None, None, None, [], 8.0)['detected'])
        self.assertTrue(rf.reload_result(0, 12, 12, [], 2.0)['full'])

    def test_scenarios_build_pad_only_and_stay_within_the_budget(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        for tag in rf.FAMILIES:
            for ph in ('aim_idle', 'hip_fire', 'aim_fire', 'reload', 'crouch_aim', 'stand'):
                sc = rf.scenario_for(tag, ph)
                self.assertEqual(sc.name, f'rf_{tag}_{ph}')
                for st in sc.steps:
                    self.assertTrue(all(b.button in ('X', 'R3') for b in st.buttons))
                    self.assertFalse(st.metrics, f'{sc.name}.{st.name}: fire / aim steps carry no pass / fail metrics')
            self.assertLess(rf.pad_seconds(tag, scen), 30.0, tag)       # + commands, reload wait and the path probes: ~40 s
        self.assertEqual(rf.phase_label('smg', 'hip_fire'), 'rf_smg_hip_fire')
        self.assertEqual(rf.phase_plan('smg')[0], 'equip')
        self.assertEqual(rf.phase_plan('smg')[-1], 'remove')

    def test_fire_taps_follow_the_shot_count_and_gap(self):
        raw = rf.fire_scenario_dict('hip_fire', 1.0, 2)
        keys = raw['steps'][0]['rt']['keys']
        self.assertEqual([k for k in keys if k[1] == 1], [[0.3, 1], [1.3, 1]])
        raw = rf.fire_scenario_dict('aim_fire', 0.5, 1)
        self.assertEqual([s['name'] for s in raw['steps']], ['aim', 'shots', 'lower'])
        self.assertGreaterEqual(raw['steps'][1]['lt'][0][1], 1)           # aim held while shooting
        with self.assertRaises(ValueError):
            rf.fire_scenario_dict('reload', 1.0, 1)

    def test_crouch_aim_has_two_crouch_taps_and_no_shot(self):
        sc = rf.scenario_for('handgun', 'crouch_aim')
        taps = [b for st in sc.steps for b in st.buttons]
        self.assertEqual([b.button for b in taps], ['R3', 'R3'])
        self.assertTrue(all('rt' not in st.tracks or st.tracks['rt'].keys[-1][1] == 0 for st in sc.steps))


def run_macro(script, args=None, patch_reload=True, raises=None, fams=None):
    with tempfile.TemporaryDirectory() as d:
        n = FakeNight(d, script, args)
        if fams:
            n.args.ranged_families = fams
        plan = {'runs': []}
        if raises:
            orig = n.play

            def play(pad, g, name, plan, sc=None):
                if name == raises:
                    raise RuntimeError('boom')
                return orig(pad, g, name, plan, sc)
            n.play = play
        ctx = mock.patch.object(rf, 'RELOAD_MAX_S', 0.05) if patch_reload else mock.patch.object(rf, 'RELOAD_MAX_S', rf.RELOAD_MAX_S)
        ctx2 = mock.patch.object(rf, 'UNREADABLE_WAIT_S', 0.05) if patch_reload else mock.patch.object(rf, 'UNREADABLE_WAIT_S', rf.UNREADABLE_WAIT_S)
        with ctx, ctx2:
            try:
                res = rf.run_ranged_macro(n, _Pad(), _G(), plan, 'ranged_families', {'name': 'ranged_families', 'run': 1})
            except RuntimeError as e:
                return n, e, plan
        return n, res, plan


class Macro(unittest.TestCase):
    PHASES = ['draw', 'aim_idle', 'hip_fire', 'aim_fire', 'reload', 'walk', 'walk_aimed', 'sprint', 'crouch_aim', 'holster']

    def test_handgun_passes_in_order_with_a_label_per_phase(self):
        n, res, plan = run_macro(good_script('handgun'), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'PASS', r['fails'])
        self.assertEqual(res['verdict'], 'PASS')
        self.assertEqual(r['record'], rf.FAMILIES['handgun']['records'][0])
        self.assertEqual(r['evidence'], 'tweak')
        self.assertEqual([w['label'] for w in res['phase_windows']], [f'rf_handgun_{p}' for p in self.PHASES])
        for w in res['phase_windows']:
            self.assertLessEqual(w['start'], w['end'])
        self.assertEqual(n.plays(), ['rf_setup_walk', 'rf_handgun_aim_idle', 'rf_handgun_hip_fire', 'rf_handgun_aim_fire',
                                     'rf_handgun_reload', 'rf_handgun_walk', 'rf_handgun_walk_aimed', 'rf_handgun_sprint',
                                     'rf_handgun_crouch_aim'])
        cmds = n.cmds()
        self.assertEqual(cmds[0], 'RF_HAND')                          # the weapon in the hand before the set
        self.assertEqual(cmds[1:3], ['RF_PICK', 'RF_GIVE'])
        tail = cmds[-6:]
        self.assertEqual([c for c in tail if c != 'RF_HAND'], ['RF_HOLSTER', 'RF_AMMO_OFF', 'RF_REMOVE', 'RF_REDRAW'])
        self.assertEqual((r['removed'], r['ammo_removed']), ('removed', 'removed'))
        self.assertEqual(res['original_weapon']['result'], 'draw requested')
        self.assertEqual(r['shots_planned'], [3, 3])
        self.assertEqual(r['capacity'], 12)

    def test_walking_phases_are_copies_that_name_their_part(self):
        n, res, plan = run_macro(good_script('handgun'), fams='handgun')
        parts = {r['scenario']: r.get('part') for r in plan['runs']}
        self.assertEqual(parts['rf_handgun_walk'], 'ranged_walk')
        self.assertEqual(parts['rf_handgun_walk_aimed'], 'ranged_walk_aimed')
        self.assertEqual(parts['rf_handgun_sprint'], 'ranged_sprint')
        self.assertEqual(parts['rf_setup_walk'], 'ranged_walk')
        self.assertNotIn('part', [r for r in plan['runs'] if r['scenario'] == 'rf_handgun_hip_fire'][0])
        # open ground is probed before the walk and the sprint only
        self.assertEqual([c[2] for c in n.calls if c[0] == 'walk'], [True, True, False, True])
        self.assertEqual(n.scenarios['rf_handgun_walk'].steps[0].name, 'walk_armed')

    def test_ledger_lists_the_weapon_and_the_ammunition_as_restored(self):
        n, res, plan = run_macro(good_script('handgun'), fams='handgun')
        self.assertEqual([(e['key'], e['owner'], e['state']) for e in n.settings.record()],
                         [('inventory.handgun', 'ranged_families', 'restored'), ('ammo.handgun', 'ranged_families', 'restored')])

    def test_record_not_found_is_a_skip_with_a_line_and_changes_nothing(self):
        n, res, plan = run_macro(good_script('smg', pick='missing'), fams='smg')
        r = res['families']['smg']
        self.assertEqual((r['verdict'], res['verdict']), ('SKIP', 'SKIP'))
        self.assertIn('record not found', r['skip_reason'])
        self.assertTrue(any('record not found' in ln for ln in n.logs), n.logs)
        self.assertTrue(any('record not found' in w for w in res['warnings']))      # never a silent pass
        self.assertNotIn('RF_GIVE', n.cmds())
        self.assertNotIn('RF_REDRAW', n.cmds())                                      # the hand was never touched
        self.assertEqual(n.settings.record(), [])
        self.assertEqual(res['phase_windows'], [])

    def test_wrong_item_type_is_a_skip_with_its_own_text(self):
        n, res, plan = run_macro(good_script('smg', pick='wrongtype'), fams='smg')
        r = res['families']['smg']
        self.assertEqual(r['verdict'], 'SKIP')
        self.assertIn('no candidate record has an item type', r['skip_reason'])

    def test_override_record_reaches_the_command(self):
        args = types.SimpleNamespace(ranged_families='handgun', ranged_records=['handgun=Items.Preset_Nue_Default'])
        seen = []
        base = good_script('handgun')

        def script(event, cmd):
            if event == 'RF_PICK':
                seen.append(cmd)
            return base(event, cmd)
        run_macro(script, args=args)
        self.assertIn('Items.Preset_Nue_Default', seen[0])
        self.assertNotIn('Items.Preset_Lexington_Default', seen[0])

    def test_a_lost_command_is_sent_again(self):
        n, res, plan = run_macro(good_script('handgun', lost_once=('RF_PICK', 'RF_GIVE')), fams='handgun')
        self.assertEqual(res['families']['handgun']['verdict'], 'PASS', res['families']['handgun']['fails'])
        self.assertEqual(n.cmds().count('RF_PICK'), 2)
        self.assertEqual(n.cmds().count('RF_GIVE'), 2)
        self.assertTrue(any('echo LOST, sent again' in ln for ln in n.logs))

    def test_a_command_lost_twice_fails_the_family_and_still_cleans_up(self):
        script = good_script('handgun')

        def lost(event, cmd):
            return None if event == 'RF_GIVE' else script(event, cmd)
        n, res, plan = run_macro(lost, fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertTrue(any('give command failed' in f for f in r['fails']))
        self.assertIn('RF_REMOVE', n.cmds())                         # the cleanup was armed before the give was sent
        self.assertIn('local keep=40 ', [c for t, c in n.sent if t == 'RF_AMMO_OFF'][0])      # baseline of the pick, not of the give echo

    def test_weapon_that_never_reaches_the_hand_fails_after_one_redraw_and_cleans_up(self):
        n, res, plan = run_macro(good_script('handgun', never_drawn=True), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertTrue(any('not in the hand' in f for f in r['fails']), r['fails'])
        cmds = n.cmds()
        gives = [i for i, c in enumerate(cmds) if c == 'RF_GIVE']
        self.assertEqual(len(gives), 2)                              # the one cheap retry
        self.assertEqual(cmds[gives[0] + 1:gives[1]].count('RF_HAND'), rf.DRAW_POLLS)
        self.assertIn('RF_REMOVE', n.cmds())
        self.assertNotIn('rf_handgun_hip_fire', n.plays())           # no phase without the weapon in the hand

    def test_no_shot_fails_the_fire_phases_and_the_reload_is_not_judged_full(self):
        n, res, plan = run_macro(good_script('handgun', shots_fired=False), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'FAIL')
        self.assertEqual(sum('no shot was fired' in f for f in r['fails']), 2)
        self.assertTrue(any('magazine was full' in w for w in r['warnings']), r['warnings'])

    def test_reload_that_never_refills_fails(self):
        n, res, plan = run_macro(good_script('handgun', reload_grows=False), fams='handgun')
        r = res['families']['handgun']
        self.assertTrue(any('did not refill' in f for f in r['fails']), r['fails'])
        self.assertEqual(r['phases'][[p['phase'] for p in r['phases']].index('reload')]['result']['detected'], False)

    def test_a_give_that_ran_but_whose_echo_was_lost_keeps_the_baseline_of_the_pick(self):
        n, res, plan = run_macro(good_script('handgun', give_lost_after_run=True), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'PASS', r['fails'])
        self.assertEqual(n.cmds().count('RF_GIVE'), 2)
        self.assertEqual(r['removed'], 'removed')                                   # not "the player owned it"
        remove = [c for t, c in n.sent if t == 'RF_REMOVE'][0]
        ammo_off = [c for t, c in n.sent if t == 'RF_AMMO_OFF'][0]
        self.assertIn('local keep=0 ', remove)
        self.assertIn('local keep=40 ', ammo_off)                                   # the count the pick read before the first give
        equip = r['phases'][0]
        self.assertEqual(equip['baseline'], {'n': 0, 'ammo': 40})
        self.assertEqual(equip['give_echo']['how'], 'had')

    def test_an_unknown_first_candidate_is_reported_although_a_later_one_wins(self):
        recs = rf.FAMILIES['handgun']['records']
        seen = f'{recs[0].replace("Items.", "")}=missing,{recs[1].replace("Items.", "")}=Wea_Handgun,{recs[2].replace("Items.", "")}=Wea_Rifle,'
        n, res, plan = run_macro(good_script('handgun', pick_rec=recs[1], seen=seen), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['record'], recs[1])
        self.assertTrue(any('record not found' in ln and 'Lexington_Default' in ln for ln in n.logs), n.logs)
        self.assertTrue(any('Lexington_Default: record not found' in w for w in r['warnings']))
        self.assertTrue(any('Kenshin_Default: item type Wea_Rifle' in w for w in r['warnings']))

    def test_draw_holster_and_the_reload_wait_are_plan_runs_for_the_helper_reports(self):
        n, res, plan = run_macro(good_script('handgun'), fams='handgun')
        runs = {r['scenario']: r for r in plan['runs']}
        for ph in ('draw', 'holster'):
            r = runs[f'rf_handgun_{ph}']
            self.assertEqual([s['name'] for s in r['steps']], [ph])
            self.assertFalse(r['steps'][0]['metrics'])
            self.assertLessEqual(r['steps'][0]['start'], r['steps'][0]['end'])
            self.assertIsNone(r['aborted'])
        reload_step = runs['rf_handgun_reload']['steps'][0]
        reload_win = next(w for w in res['phase_windows'] if w['phase'] == 'reload')
        self.assertEqual(reload_step['end'].strftime('%Y-%m-%d %H:%M:%S.%f')[:23], reload_win['end'])   # covers the wait, not the tap
        # the plan runs of the harness analysis accept these runs
        scen = load_scenarios(gw.SCENARIO_FILE)
        rows = gt_run.analyze({'runs': [runs['rf_handgun_draw'], runs['rf_handgun_holster']]}, [], gt_log.DEFAULT_THRESHOLDS, scen)
        self.assertEqual([r['verdict'] for r in rows], ['SKIP', 'SKIP'])

    def test_unreadable_magazine_does_not_wait_for_a_log_line_that_may_never_come(self):
        with mock.patch.object(rf, 'UNREADABLE_WAIT_S', 0.02), mock.patch.object(rf, 'RELOAD_MAX_S', 30.0):
            n, res, plan = run_macro(good_script('handgun', mag_readable=False), patch_reload=False, fams='handgun')
        reload_rec = next(p for p in res['families']['handgun']['phases'] if p['phase'] == 'reload')
        self.assertEqual(reload_rec['wait_limit_s'], 0.02)
        self.assertIsNone(reload_rec['result']['detected'])

    def test_unreadable_magazine_is_a_warning_not_a_failure(self):
        n, res, plan = run_macro(good_script('handgun', mag_readable=False), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['verdict'], 'PASS', r['fails'])
        self.assertTrue(any('not verified' in w for w in r['warnings']))
        self.assertTrue(any('capacity could not be read' in w for w in r['warnings']))

    def test_double_barrel_capacity_limits_the_shots(self):
        n, res, plan = run_macro(good_script('double_shotgun', cap=2), fams='double_shotgun')
        r = res['families']['double_shotgun']
        self.assertEqual(r['shots_planned'], [1, 0])     # one round stays in the magazine

    def test_weapon_the_player_owned_stays_and_only_the_ammunition_is_restored(self):
        n, res, plan = run_macro(good_script('handgun', how='had'), fams='handgun')
        r = res['families']['handgun']
        self.assertEqual(r['removed'], 'kept (the player owned it)')
        self.assertNotIn('RF_REMOVE', n.cmds())
        self.assertIn('RF_AMMO_OFF', n.cmds())

    def test_an_error_in_a_phase_still_holsters_removes_and_restores(self):
        n, res, plan = run_macro(good_script('handgun'), raises='rf_handgun_aim_fire', fams='handgun')
        self.assertIsInstance(res, RuntimeError)
        cmds = n.cmds()
        self.assertEqual([c for c in cmds[cmds.index('RF_HOLSTER'):] if c != 'RF_HAND'], ['RF_HOLSTER', 'RF_AMMO_OFF', 'RF_REMOVE', 'RF_REDRAW'])
        self.assertEqual([(e['key'], e['state']) for e in n.settings.record()],
                         [('inventory.handgun', 'restored'), ('ammo.handgun', 'restored')])

    def test_two_families_share_one_restore_at_the_end(self):
        scripts = {t: good_script(t) for t in ('handgun', 'smg')}
        current = {'t': 'handgun'}

        def script(event, cmd):
            m = re.match(r'play:rf_(handgun|smg)_', event)
            if m:
                current['t'] = m.group(1)
            else:
                for tag in scripts:
                    if f"' {tag} '" in cmd or any(r in cmd for r in rf.FAMILIES[tag]['records']):
                        current['t'] = tag
                        break
            return scripts[current['t']](event, cmd)
        n, res, plan = run_macro(script, fams='handgun,smg')
        self.assertEqual(list(res['families']), ['handgun', 'smg'])
        self.assertEqual((res['families']['handgun']['verdict'], res['families']['smg']['verdict']), ('PASS', 'PASS'))
        self.assertEqual(n.cmds().count('RF_REDRAW'), 1)                       # at the very end, not between the families
        self.assertEqual(n.cmds()[-1], 'RF_REDRAW')
        self.assertEqual([w['label'].split('_')[1] for w in res['phase_windows']], ['handgun'] * 10 + ['smg'] * 10)
        self.assertEqual(n.plays().count('rf_setup_walk'), 1)

    def test_set_verdict_fails_when_one_family_fails(self):
        n, res, plan = run_macro(good_script('handgun', shots_fired=False), fams='handgun')
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertTrue(any(f.startswith('handgun: ') for f in res['fails']))

    def test_hand_empty_at_the_start_is_not_drawn_at_the_end(self):
        n, res, plan = run_macro(good_script('handgun', orig=False), fams='handgun')
        self.assertEqual(res['families']['handgun']['verdict'], 'PASS')
        self.assertNotIn('RF_REDRAW', n.cmds())

    def test_unreadable_original_hand_is_reported(self):
        base = good_script('handgun', orig=True)
        calls = {'n': 0}

        def script(event, cmd):
            if event == 'RF_HAND' and "TweakDBID.new('Items." not in cmd:
                calls['n'] += 1
                return ['CET [night] RF_HAND ok=false nothing']
            return base(event, cmd)
        n, res, plan = run_macro(script, fams='handgun')
        self.assertTrue(any('before the set could not be read' in w for w in res['warnings']))


class RunnerParts(unittest.TestCase):
    WINDOWS = [{'family': 'handgun', 'phase': 'hip_fire', 'label': 'rf_handgun_hip_fire', 'start': '2026-11-14 10:00:05.000',
                'end': '2026-11-14 10:00:08.000'},
               {'family': 'handgun', 'phase': 'reload', 'label': 'rf_handgun_reload', 'start': '2026-11-14 10:00:08.000',
                'end': '2026-11-14 10:00:11.500'}]

    def records(self):
        return [{'name': 'ranged_families', 'run': 1, 'start': datetime(2026, 11, 14, 10, 0, 0), 'end': datetime(2026, 11, 14, 10, 1, 0),
                 'phase_windows': list(self.WINDOWS)},
                {'name': 'melee_katana', 'run': 1, 'start': datetime(2026, 11, 14, 10, 2, 0), 'end': datetime(2026, 11, 14, 10, 3, 0),
                 'phase_windows': [{'weapon': 'katana', 'phase': 'holster', 'start': '2026-11-14 10:02:01.000',
                                    'end': '2026-11-14 10:02:02.000'}]}]       # the melee windows carry no label: nothing is added

    def test_pose_meter_rows_per_labelled_phase(self):
        wins = gp.build_windows(self.records(), [])
        labels = [w[0] for w in wins]
        self.assertIn('rf_handgun_hip_fire#1', labels)
        self.assertIn('rf_handgun_reload#1', labels)
        self.assertEqual(sum(1 for lb in labels if 'katana' in lb and 'holster' in lb), 0)
        self.assertIn('ranged_families#1~macro', labels)
        hip = next(w for w in wins if w[0] == 'rf_handgun_hip_fire#1')
        self.assertEqual((hip[1], hip[2]), (datetime(2026, 11, 14, 10, 0, 5), datetime(2026, 11, 14, 10, 0, 8)))
        self.assertTrue(gp.is_scenario_label('rf_handgun_hip_fire#1'))              # a row of its own, report only

    def test_pose_meter_keeps_its_rows_without_phase_windows(self):
        recs = [{k: v for k, v in r.items() if k != 'phase_windows'} for r in self.records()]
        labels = [w[0] for w in gp.build_windows(recs, [])]
        self.assertEqual(labels, ['ranged_families#1~macro', 'melee_katana#1~macro'])

    def test_phase_windows_and_sheet_jobs(self):
        phases = gn.phase_windows_of(self.records())
        self.assertEqual([p['name'] for p in phases], ['rf_handgun_hip_fire_r1', 'rf_handgun_reload_r1'])
        vs = datetime(2026, 11, 14, 9, 59, 50)
        wins = gn.phase_sheet_windows(vs, 100.0, phases)
        self.assertEqual(wins[0]['start'], round(15.0 - 0.3, 2))
        self.assertEqual(wins[0]['end'], round(18.0 + 0.3, 2))
        self.assertGreaterEqual(wins[0]['every'], 0.1)
        self.assertLess((wins[0]['end'] - wins[0]['start']) / wins[0]['every'], 31)
        self.assertEqual(gn.phase_windows_of([{'name': 'x', 'run': 1}]), [])
        self.assertEqual(gn.phase_sheet_windows(vs, 10.0, [{'name': 'late', 'start': datetime(2026, 11, 14, 10, 5, 0),
                                                            'end': datetime(2026, 11, 14, 10, 5, 3)}]), [])   # outside the recording
        scs = [{'name': 'ranged_families_r1', 'start': datetime(2026, 11, 14, 10, 0, 0), 'end': datetime(2026, 11, 14, 10, 1, 0)}]
        jobs = gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 10:01:30.000', 0.0, 'ff', vs, 100.0, scs, 3.0, 'sd', phases=phases)
        names = [j['sheet'] for j in jobs]
        self.assertIn('rf_handgun_reload_r1', names)
        self.assertIn('ranged_families_r1_start', names)
        job = next(j for j in jobs if j['sheet'] == 'rf_handgun_hip_fire_r1')
        self.assertEqual(job['args'][job['args'].index('--cols') + 1], '6')
        only = gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 10:01:30.000', 0.0, 'ff', vs, 100.0, scs, 3.0, 'sd',
                                   only='rf_handgun_reload_r1', phases=phases)
        self.assertEqual([j['sheet'] for j in only], ['rf_handgun_reload_r1'])
        plain = gt_post.plan_sheets('v.mp4', 'u.log', '2026-11-14 10:01:30.000', 0.0, 'ff', vs, 100.0, scs, 3.0, 'sd')
        self.assertFalse([j for j in plain if j['sheet'].startswith('rf_')])     # runs without labelled phases are unchanged

    def test_family_copy_is_scored_with_the_thresholds_of_its_part(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        t0, t1 = datetime(2026, 11, 14, 10, 0, 0), datetime(2026, 11, 14, 10, 0, 3)
        step = {'name': 'aim_walk', 'start': t0, 'end': t1, 'metrics': True}
        thr = gt_run.load_thresholds(os.path.join(HERE, 'thresholds.json'))
        with_part = {'runs': [{'scenario': 'rf_handgun_walk_aimed', 'part': 'ranged_walk_aimed', 'run': 1, 'aborted': None,
                               'steps': [dict(step)]}]}
        without = {'runs': [{'scenario': 'rf_handgun_walk_aimed', 'run': 1, 'aborted': None, 'steps': [dict(step)]}]}
        rows = gt_run.analyze(with_part, [], thr, scen)
        self.assertEqual((rows[0]['metrics']['stick_min'], rows[0]['metrics']['stick_max']), (0.0, 0.5))
        rows = gt_run.analyze(without, [], thr, scen)
        self.assertEqual(rows[0]['metrics']['stick_max'], None)             # an unknown name has no stick data

    def test_the_part_steps_keep_the_names_of_the_existing_parts(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        self.assertEqual([s.name for s in scen['ranged_walk'].steps], ['walk_armed', 'stop'])
        self.assertEqual([s.name for s in scen['ranged_walk_aimed'].steps], ['aim_walk', 'stop'])
        self.assertEqual(scen['ranged_sprint'].steps[0].name, 'sprint')
        self.assertEqual(scen['ranged_sprint'].steps[0].buttons[0].button, 'LS')       # the sprint tap of slide_sprint
        for part in rf.PARTS:
            self.assertFalse(scen[part].steps[-1].metrics)                             # the stop step is not judged

    def test_the_tool_test_registry_covers_this_file(self):
        testing = os.path.join(os.path.dirname(HERE), 'testing')
        sys.path.insert(0, testing)
        try:
            import run_tool_tests as rt
        finally:
            sys.path.remove(testing)
        self.assertIn('gametest/test_gt_ranged_families.py', rt.REGISTRY)
        self.assertIn('gametest/test_gt_ranged_families.py', rt.discover())


if __name__ == '__main__':
    unittest.main()
