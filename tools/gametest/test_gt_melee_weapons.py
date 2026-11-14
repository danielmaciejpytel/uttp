"""Offline tests for the melee weapon macros (gt_melee_weapons.py). No input is sent, no process is started.

  python Source\\tools\\gametest\\test_gt_melee_weapons.py -v
"""
from __future__ import annotations

import contextlib
import ctypes
import io
import json
import os
import re
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_settings  # noqa: E402
import gt_melee_weapons as gw  # noqa: E402
import gt_night as gn  # noqa: E402
from gt_input import file_size, read_since  # noqa: E402
from gt_scenario import BUTTONS, load_scenarios  # noqa: E402

PREFIX = '[2026-11-14 10:00:{s:06.3f}] [info    ] [ 1] [UTPP] UTPP '

# lines in the shape the DLL writes (timestamps replaced)
MASK_ON = ('MMMASK on reasons=0x1 (was 0x0) gate=0x400 covered=0x400 loco=1 upperBody=0 melee=1 exit=0.00 w=1.00 mask=1.00 '
           'full=0.00 fullWant=0 fullWhy={why}')
MASK_OFF = ('MMMASK off reasons=0x0 (was 0x1) gate=0x0 covered=0x0 loco=1 upperBody=0 melee=0 exit=0.00 w=1.00 mask=1.00 '
            'full=0.00 fullWant=0 fullWhy=state')
FULL = ('MMMELEEFULL {onoff} why={why} full=0.00 speed={speed} loco=1 rawLoco=1 detail=-1 vz=0.00 reasons=0x1 gate=0x400 '
        'stick=0 ground=1 fastT=0.00 fastOff=0 airT=0.00 state={state} states=0x{states:X} guard=0')
AIM = ('MELEEAIM src=cam tiltW=1.00 tgtPitch=0.0 tgtYaw=2.3 rawPitch=-11.5 rawYaw=7.3 pitch=0.0 yaw=3.2 flags=0x0 maxUp=20 '
       'maxDown=35 hz=10 calls=33 applied=32 busy=0 reapplied=0 fails=0 | BASE body=0 clip={clip} f=156 rms=1.1 gate=1.00 w=1.00 '
       'raw=-35/-22/-2 lim=-35/-22/-2 flags=0x0 matched=31 none=2 applied={applied} limited=1 fails=0 noBody=0 lowerKept=0/33 '
       'hipsZ=0.95/0.99')


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
    """The parts of the night runner the macro uses; `script(event, cmd)` gives the log lines of an event."""

    def __init__(self, tmp: str, script):
        self.utpp_log = os.path.join(tmp, 'session.log')
        open(self.utpp_log, 'w').close()
        self.scenarios = load_scenarios(gw.SCENARIO_FILE)
        self.scenario_deadline = 99.0
        self.script = script
        self.logs: list = []
        self.calls: list = []
        self.clock = 0.0

    def log(self, msg):
        self.logs.append(msg)

    def append(self, lines):
        with open(self.utpp_log, 'a', encoding='utf-8') as f:
            for ln in lines:
                self.clock += 0.05
                f.write(stamp(self.clock, ln) + '\n')

    def play(self, pad, guard, name, plan, sc=None):
        self.calls.append(('play', name))
        self.append(self.script('play:' + name, ''))
        return {'steps': []}

    def play_walking(self, pad, guard, plan, name, key_crouch=False, clear=True):
        self.calls.append(('walk', name, clear))
        off = file_size(self.utpp_log)
        self.append(self.script('play:' + name, ''))
        return {'res': {}, 'cfails': [], 'off': off, 'discarded': None}

    def console_verified(self, pad, guard, cmd, echo, label, timeout=3.0):
        tag = re.search(r"\[night\] (MW_\w+) ok=", cmd).group(1)
        self.calls.append(('cmd', tag))
        off = file_size(self.utpp_log)
        lines = self.script(tag, cmd)
        if lines is None:
            return None                               # lost
        self.append(lines)
        return read_since(self.utpp_log, off)


def good_script(tag='katana', fn=None, how='given', states=None, remove='removed 1->0 r=true', holster_clears=True,
                full_on_standing=True, block=True):
    w = gw.WEAPONS[tag]
    fn = fn or w['anim_var']
    states = states if states is not None else gw.expected_states(gw.load_script_tables(), w['anim_var'])

    held = {'now': True}

    def script(event, cmd):
        if event == 'MW_COUNT':
            return [f'CET [night] MW_COUNT ok=true {tag} rec={w["records"][0]} fn={w["anim_var"]} n={0 if how == "given" else 1}']
        if event == 'MW_GIVE':
            held['now'] = True
            return [f'CET [night] MW_GIVE ok=true {tag} rec={w["records"][0]} {how} n={0 if how == "given" else 1}->1']
        if event == 'MW_INHAND':
            return [f'CET [night] MW_INHAND ok=true {fn if held["now"] else "none"} | meleefull flag=1 states=0x{states:X}']
        if event == 'play:mw_light_combo':
            out = [MASK_ON.format(why='flag' if not full_on_standing else 'melee')]
            if full_on_standing:
                out.append(FULL.format(onoff='on', why='on', speed='0.00', state=11, states=states))
            out.append(AIM.format(clip='combo_attack_01', applied=17))
            return out
        if event == 'play:mw_strong_attack':
            return [FULL.format(onoff='off', why='state', speed='0.00', state=2, states=states)] + (
                [FULL.format(onoff='on', why='on', speed='0.00', state=13, states=states)] if full_on_standing else [])
        if event == 'play:melee_block':
            return [FULL.format(onoff='on', why='on', speed='0.00', state=8, states=states)] if full_on_standing else []
        if event == 'play:melee_punch_walk':
            return [FULL.format(onoff='off', why='speed', speed='0.80', state=11, states=states)]
        if event == 'MW_HOLSTER':
            held['now'] = False
            return ['CET [night] MW_HOLSTER ok=true eqsystem'] + ([MASK_OFF] if holster_clears else [])
        if event == 'MW_REMOVE':
            return [f'CET [night] MW_REMOVE ok=true {remove}']
        return []
    return script


class ScriptTables(unittest.TestCase):
    def test_real_script_tables(self):
        t = gw.load_script_tables()
        self.assertIsNotNone(t, 'plugin/UTPP/scripts/utpp_melee.reds not readable from the tools folder')
        self.assertTrue({'w_melee_fists', 'w_melee_knife', 'w_melee_katana', 'w_melee_one_hand_blunt',
                         'w_melee_baton', 'w_two_hand_blunt'} <= t['covered'])
        self.assertEqual(t['states']['w_melee_knife'], 3668192)
        self.assertEqual(t['states']['w_melee_katana'], 3669472)
        self.assertEqual(t['states']['w_two_hand_blunt'], 3669472)
        self.assertEqual(gw.expected_states(t, 'w_melee_hammer'), t['default_states'])
        self.assertFalse(gw.is_covered(t, 'w_melee_hammer'))
        self.assertFalse(gw.is_covered(t, 'mantis_blade'))
        sa = gw.WEAPONS['strong_arms']
        self.assertEqual(sa['anim_var'], 'w_strong_arms')           # its own variable in the hand, not the fists one
        self.assertFalse(gw.is_covered(t, sa['anim_var']))
        self.assertFalse(gw.weapon_covered(sa, t))                  # the script excludes arm cyberware
        self.assertEqual(gw.weapon_states(sa, t), t['default_states'])
        self.assertEqual(gw.weapon_covered(gw.WEAPONS['katana'], t), True)
        for tag in gw.PACK_WEAPONS:
            self.assertTrue(gw.is_covered(t, gw.WEAPONS[tag]['anim_var']), tag)

    def test_parse_synthetic_tables(self):
        text = '''
public func UTPPMeleeFullBodyCoveredName(n: CName) -> Bool {
  return Equals(n, n"w_melee_a") || Equals(n, n"w_melee_b");
}
public func UTPPMeleeStatesDefault() -> Int32 {
  return 111;
}
public func UTPPMeleeFullBodyStates(n: CName) -> Int32 {
  // UTPP_STATES_TABLE begin
  if Equals(n, n"w_melee_a") { return 222; }
  // UTPP_STATES_TABLE end
  return UTPPMeleeStatesDefault();
}
public func UTPPMeleeGuardCoveredName(n: CName) -> Bool {
  return Equals(n, n"w_melee_a");
}
'''
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'x.reds')
            with open(p, 'w') as f:
                f.write(text)
            t = gw.load_script_tables(p)
        self.assertEqual((t['covered'], t['default_states'], t['states'], t['guard']),
                         ({'w_melee_a', 'w_melee_b'}, 111, {'w_melee_a': 222}, {'w_melee_a'}))
        self.assertIsNone(gw.load_script_tables(os.path.join(tempfile.gettempdir(), 'no_such_file.reds')))


class Registration(unittest.TestCase):
    def test_sets_resolve_and_validate(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        for name in ('melee_knife', 'melee_katana', 'melee_blunt', 'melee_baton', 'melee_bat', 'melee_hammer', 'melee_mantis',
                     'melee_strong_arms', 'melee_weapons'):
            names = gn.resolve_sets(name)
            self.assertEqual(names, [name])
            self.assertEqual(gn.validate_sets(names, scen), [], name)
        self.assertEqual(gw.WEAPON_SETS['melee_weapons'], ['knife', 'katana', 'blunt', 'baton', 'bat'])
        self.assertNotIn('melee_weapons', gn.SETS['all'])
        self.assertTrue(all(not n.startswith('melee_k') for n in gn.SETS['all']))
        self.assertIn('melee_fists', gn.SETS['all'])                # the fists macro is untouched

    def test_pad_parts_exist_and_use_pad_only(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        for part in gw.PARTS:
            self.assertIn(part, scen)
            for st in scen[part].steps:
                self.assertTrue(all(b.button in BUTTONS for b in st.buttons), part)

    def test_scripted_parts_build_and_carry_the_crouch_button(self):
        for name in gw.SCRIPTED_PARTS:
            sc = gw.scripted_scenario(name)
            self.assertTrue(any(b.button == 'R3' for st in sc.steps for b in st.buttons), name)
            self.assertNotIn(name, load_scenarios(gw.SCENARIO_FILE))
        self.assertEqual(sum(1 for st in gw.scripted_scenario('mw_crouch_attack').steps for b in st.buttons), 2)

    def test_time_limit(self):
        self.assertGreater(gw.time_limit_s('melee_weapons'), 4 * 100)
        self.assertLess(gw.time_limit_s('melee_knife'), gw.time_limit_s('melee_weapons'))

    def test_phase_plan(self):
        self.assertEqual(gw.phase_plan('katana'), ['equip', 'draw', 'light_combo', 'guard_idle', 'strong_attack', 'block',
                                                   'walk_strikes', 'sprint_attack', 'crouch_attack', 'holster', 'remove'])
        self.assertNotIn('block', gw.phase_plan('knife'))
        self.assertNotIn('LT', json.dumps(gw.dry_run_lines('knife')[:2]))


class Commands(unittest.TestCase):
    def all_commands(self):
        for tag, w in gw.WEAPONS.items():
            if w['kind'] == 'cyberware':
                yield gw.cyberware_equip_cmd(w)
                yield gw.cyberware_draw_cmd()
                yield gw.cyberware_restore_cmd(w, 12345, True)
            else:
                yield gw.count_cmd(w)
                yield gw.give_cmd(w, w['records'][0])
                yield gw.remove_cmd(w['records'][0], 0)
        yield gw.inhand_cmd()
        yield gw.holster_cmd()

    def test_commands_echo_and_carry_no_paths(self):
        for cmd in self.all_commands():
            self.assertRegex(cmd, r"Game\.UTPPLogLine\('\[night\] MW_\w+ ok='")
            self.assertNotRegex(cmd, r'[A-Za-z]:\\')
            self.assertNotIn('\n', cmd)

    def test_give_gates_on_the_anim_variable_and_names_the_records(self):
        cmd = gw.count_cmd(gw.WEAPONS['baton'])
        self.assertIn("want='w_melee_baton'", cmd)
        for rec in gw.WEAPONS['baton']['records']:
            self.assertIn(rec, cmd)
        self.assertNotIn('GiveItem', cmd)                                    # the count only reads
        give = gw.give_cmd(gw.WEAPONS['baton'], 'Items.Preset_Baton_Alpha')
        self.assertIn('DrawItemRequest', give)
        self.assertIn('if n0==0 then ts:GiveItem', give)                     # counted inside: a repeated send gives nothing twice
        self.assertNotIn('Equip', give.replace('EquipmentSystem', ''))       # no loadout slot is touched
        self.assertIn('MW_GIVE ok=', give)
        for c in (cmd, give, gw.remove_cmd('Items.X', 0)):
            self.assertIn("error('GetItemList failed", c)                    # a failing list read is an error, not 'empty'

    def test_remove_keeps_what_the_player_owned(self):
        self.assertIn('local keep=2 ', gw.remove_cmd('Items.X', 2))
        self.assertIn('ts:RemoveItem(p,l[i],1)', gw.remove_cmd('Items.X', 0))

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

    def test_echo_and_parsers(self):
        text = 'x\nCET [night] MW_GIVE ok=true katana rec=Items.A given n=0->1\n'
        ok, info = gw.echo_of(text, 'MW_GIVE')
        self.assertTrue(ok)
        self.assertEqual(gw.parse_give(info), {'rec': 'Items.A', 'how': 'given', 'n_before': 0, 'n_after': 1})
        self.assertEqual(gw.parse_count('rec=Items.A fn=w_melee_katana n=2'), {'rec': 'Items.A', 'fn': 'w_melee_katana', 'n': 2})
        self.assertIsNone(gw.parse_count('no record'))
        self.assertEqual(gw.echo_of('nothing', 'MW_GIVE'), (None, ''))
        self.assertEqual(gw.echo_of('CET [night] MW_GIVE ok=false x', 'MW_GIVE')[0], False)
        self.assertEqual(gw.inhand_name('w_melee_katana | meleefull flag=1'), 'w_melee_katana')
        self.assertEqual(gw.inhand_name('none | meleefull'), 'none')
        self.assertEqual([gw.parse_remove(x) for x in ('removed 1->0 r=true', 'nothing to remove 0 keep 0', 'NOT removed 1->1 r=-/-')],
                         ['removed', 'nothing', 'failed'])
        cw = gw.parse_give('rec=Items.MantisBlades given prev=991')
        self.assertEqual((cw['how'], cw['prev_hash']), ('given', 991))


class Evidence(unittest.TestCase):
    def test_parse_real_shaped_lines(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        text = '\n'.join(stamp(1 + i * 0.1, ln) for i, ln in enumerate([
            MASK_ON.format(why='melee'), FULL.format(onoff='on', why='on', speed='0.00', state=11, states=s),
            AIM.format(clip='combo_attack_01', applied=17), AIM.format(clip='combo_attack_02', applied=40),
            FULL.format(onoff='off', why='state', speed='0.00', state=2, states=s), MASK_OFF,
            'CET [utpp_body] t=3.00 utpp_melee_tpp=1']))
        ev = gw.parse_evidence(text)
        self.assertTrue(ev['mask_on'])
        self.assertEqual((ev['full_on'], ev['full_off'], ev['last_full_on'], ev['last_why']), (1, 1, False, 'state'))
        self.assertEqual(ev['psm_states'], [2, 11])
        self.assertEqual(ev['dll_states'], [s])
        self.assertEqual(ev['melee_tpp_last'], 1)
        self.assertEqual(ev['aim']['n'], 2)
        self.assertEqual(ev['aim']['base_clips'], ['combo_attack_01', 'combo_attack_02'])
        self.assertEqual(ev['aim']['base_applied_delta'], 23)
        self.assertEqual(ev['aim']['src'], {'cam': 2})
        self.assertEqual(ev['aim']['tgt_yaw'], [2.3, 2.3])
        json.dumps(ev)

    def test_parses_a_line_of_a_recorded_session(self):
        line = ('[2026-11-14 00:43:33.968] [info    ] [ 53196] [UTPP] UTPP MELEEAIM src=cam tiltW=1.00 tgtPitch=0.0 tgtYaw=8.2 '
                'rawPitch=-11.7 rawYaw=13.2 pitch=0.0 yaw=7.6 flags=0x0 maxUp=20 maxDown=35 hz=10 calls=17 applied=16 busy=0 '
                'reapplied=0 fails=0 | BASE body=0 clip=combo_attack_01 f=147 rms=2.5 gate=1.00 w=1.00 raw=-26/-30/-1 '
                'lim=-26/-30/-1 flags=0x0 matched=15 none=2 applied=17 limited=1 fails=0 noBody=0 lowerKept=0/17 hipsZ=0.98/0.99')
        ev = gw.parse_evidence(line)
        self.assertEqual((ev['aim']['n'], ev['aim']['base_clips'], ev['aim']['tgt_yaw']), (1, ['combo_attack_01'], [8.2, 8.2]))

    def evaluate(self, tag, lines, phase='light_combo', covered=None, carried_full=False, carried_mask=False):
        w = gw.WEAPONS[tag]
        tables = gw.load_script_tables()
        text = '\n'.join(stamp(1 + i * 0.1, ln) for i, ln in enumerate(lines))
        ev = gw.parse_evidence(text)
        ms = gn.melee_state([], text, carried_mask)
        ph = next(p for p in gw.PHASES if p['name'] == phase)
        cov = gw.is_covered(tables, w['anim_var']) if covered is None else covered
        return gw.evaluate_phase(w, ph, ev, ms, cov, gw.expected_states(tables, w['anim_var']), carried_full, carried_mask)

    def test_covered_weapon_with_full_body_passes(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        r = self.evaluate('katana', [MASK_ON.format(why='melee'), FULL.format(onoff='on', why='on', speed='0', state=11, states=s)])
        self.assertEqual((r['fails'], r['own_clip']), ([], []))

    def test_first_person_clip_sign_when_the_full_body_never_switches_on(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        r = self.evaluate('katana', [MASK_ON.format(why='flag'),
                                     FULL.format(onoff='off', why='flag', speed='0', state=11, states=s)])
        self.assertTrue(any('never switched on' in f for f in r['fails']))
        self.assertIn('why=flag', r['own_clip'][0])

    def test_state_refused_although_it_is_in_the_table(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        r = self.evaluate('katana', [MASK_ON.format(why='state'),
                                     FULL.format(onoff='off', why='state', speed='0', state=6, states=s)])
        self.assertTrue(any('refused' in x for x in r['own_clip']))

    def test_dll_mask_differs_from_the_table(self):
        r = self.evaluate('knife', [MASK_ON.format(why='melee'), FULL.format(onoff='on', why='on', speed='0', state=11, states=3667968)])
        self.assertTrue(any('differs from the script table' in x for x in r['own_clip']))

    def test_control_weapon_must_not_switch_the_full_body_on(self):
        r = self.evaluate('hammer', [MASK_ON.format(why='flag'), FULL.format(onoff='on', why='on', speed='0', state=11, states=3667968)])
        self.assertTrue(any('does not cover' in f for f in r['fails']))
        r = self.evaluate('hammer', [MASK_ON.format(why='flag'), FULL.format(onoff='off', why='flag', speed='0', state=11, states=3667968)])
        self.assertEqual((r['fails'], r['own_clip']), ([], []))

    def test_walk_phase_expects_the_full_body_off(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        ok = self.evaluate('katana', [MASK_ON.format(why='melee'), FULL.format(onoff='off', why='speed', speed='0.9', state=11, states=s)],
                           phase='walk_strikes')
        self.assertEqual((ok['fails'], ok['warns']), ([], []))
        stuck = self.evaluate('katana', [MASK_ON.format(why='melee'), FULL.format(onoff='on', why='on', speed='0', state=11, states=s)],
                              phase='walk_strikes')
        self.assertTrue(any('stayed on' in x for x in stuck['warns']))

    def test_weight_drop_fails_standing_and_warns_in_sprint(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        text_lines = [MASK_ON.format(why='melee').replace('w=1.00', 'w=0.20')]
        text_lines += ['MMGATE melee=0x400 w=0.20 mask=1.00'] * 8       # samples after the mask blend
        text_lines.append(FULL.format(onoff='on', why='on', speed='0', state=11, states=s))
        r = self.evaluate('katana', text_lines)
        self.assertTrue(any('MM weight' in f for f in r['fails']), r)
        r = self.evaluate('katana', text_lines, phase='sprint_attack')
        self.assertFalse(any('MM weight' in f for f in r['fails']))
        self.assertTrue(any('MM weight' in x for x in r['warns']))

    def test_carried_state_counts(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        r = self.evaluate('katana', [FULL.format(onoff='off', why='state', speed='0', state=2, states=s)], carried_full=True,
                          carried_mask=True)
        self.assertEqual(r['fails'], [])

    def test_base_correction_note_for_non_fists(self):
        s = gw.expected_states(gw.load_script_tables(), 'w_melee_katana')
        r = self.evaluate('katana', [MASK_ON.format(why='melee'), FULL.format(onoff='on', why='on', speed='0', state=11, states=s),
                                     AIM.format(clip='combo_attack_01', applied=10), AIM.format(clip='combo_attack_01', applied=30)])
        self.assertTrue(any('chest base correction' in x for x in r['warns']))


class Macro(unittest.TestCase):
    def run_macro(self, script, set_name='melee_katana', tmp=None, raises=None):
        with tempfile.TemporaryDirectory() as d:
            n = FakeNight(d, script)
            n.settings = gt_settings.Ledger(None, log=lambda m: None)      # the run record's list of what a macro changed
            plan = {'runs': []}
            if raises:
                orig = n.play

                def play(pad, g, name, plan, sc=None):
                    if name == raises:
                        raise RuntimeError('boom')
                    return orig(pad, g, name, plan, sc)
                n.play = play
            try:
                res = gw.run_weapon_macro(n, _Pad(), _G(), plan, set_name)
            except RuntimeError as e:
                return n, e
            return n, res

    def test_given_weapon_is_listed_in_the_ledger_and_marked_restored(self):
        n, res = self.run_macro(good_script('katana'))
        self.assertEqual([(e['key'], e['owner'], e['state']) for e in n.settings.record()],
                         [('inventory.katana', 'melee_weapons', 'restored')])

    def test_katana_pass_in_order(self):
        n, res = self.run_macro(good_script('katana'))
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertEqual(n.calls, [
            ('cmd', 'MW_COUNT'), ('cmd', 'MW_GIVE'), ('cmd', 'MW_INHAND'),
            ('play', 'mw_light_combo'), ('play', 'mw_guard_idle'), ('play', 'mw_strong_attack'), ('play', 'melee_block'),
            ('walk', 'melee_punch_walk', True),
            ('cmd', 'MW_INHAND'), ('walk', 'mw_sprint_attack', True),
            ('cmd', 'MW_INHAND'), ('play', 'mw_crouch_attack'),
            ('cmd', 'MW_HOLSTER'), ('cmd', 'MW_INHAND'), ('cmd', 'MW_REMOVE')])
        r = res['weapons']['katana']
        self.assertEqual([p['phase'] for p in r['phases']], gw.phase_plan('katana'))
        self.assertEqual([w['phase'] for w in res['phase_windows']], gw.phase_plan('katana'))
        for w in res['phase_windows']:
            self.assertRegex(w['start'], r'^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3}$')
            self.assertLessEqual(w['start'], w['end'])
        self.assertEqual((r['removed'], r['given']['how'], r['own_clip_phases']), ('removed', 'given', []))
        self.assertTrue(r['phases'][2]['full_on'] >= 1 and r['phases'][2]['mask_on'])
        self.assertEqual(r['phases'][2]['aim']['base_clips'], ['combo_attack_01'])
        json.dumps(res)

    def test_knife_has_no_block_phase_and_no_lt(self):
        n, res = self.run_macro(good_script('knife'), 'melee_knife')
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertNotIn(('play', 'melee_block'), n.calls)
        self.assertNotIn('block', [p['phase'] for p in res['weapons']['knife']['phases']])

    def test_pack_set_runs_the_four_weapons_in_order(self):
        scripts = {t: good_script(t) for t in gw.PACK_WEAPONS}
        current = {'tag': None}

        def script(event, cmd):
            if event == 'MW_COUNT':
                current['tag'] = re.search(r"MW_COUNT ok='..tostring\(ok\)..' (\w+) ", cmd).group(1)
            return scripts[current['tag']](event, cmd)
        n, res = self.run_macro(script, 'melee_weapons')
        self.assertEqual(list(res['weapons']), ['knife', 'katana', 'blunt', 'baton', 'bat'])
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertEqual([c for c in n.calls if c == ('cmd', 'MW_GIVE')], [('cmd', 'MW_GIVE')] * 5)
        self.assertEqual(sum(1 for c in n.calls if c == ('cmd', 'MW_REMOVE')), 5)

    def test_wrong_anim_variable_is_skipped_and_cleaned_up(self):
        base = good_script('katana')

        def script(event, cmd):
            if event == 'MW_INHAND':
                return ['CET [night] MW_INHAND ok=true w_melee_one_hand_blunt | meleefull flag=0']
            return base(event, cmd)
        n, res = self.run_macro(script)
        r = res['weapons']['katana']
        self.assertEqual((r['verdict'], res['verdict']), ('SKIP', 'SKIP'))
        self.assertIn('wrong anim variable', r['skip_reason'])
        self.assertEqual(r['fails'], [])
        self.assertNotIn(('play', 'mw_light_combo'), n.calls)
        self.assertIn(('cmd', 'MW_HOLSTER'), n.calls)
        self.assertIn(('cmd', 'MW_REMOVE'), n.calls)

    def test_no_candidate_record_is_a_skip_without_cleanup(self):
        def script(event, cmd):
            if event == 'MW_COUNT':
                return ['CET [night] MW_COUNT ok=false baton CET: no record with friendlyName w_melee_baton seen Items.A=x,']
            return []
        n, res = self.run_macro(script, 'melee_baton')
        self.assertEqual(res['verdict'], 'SKIP')
        self.assertEqual([c for c in n.calls if c[0] == 'cmd'], [('cmd', 'MW_COUNT')])       # nothing given, nothing to clean up

    def test_lost_count_echo_fails_before_anything_is_given(self):
        def script(event, cmd):
            return None if event == 'MW_COUNT' else []
        n, res = self.run_macro(script)
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertEqual(n.calls, [('cmd', 'MW_COUNT')])

    def test_lost_give_echo_is_still_cleaned_up(self):
        base = good_script('katana')

        def script(event, cmd):
            if event == 'MW_GIVE':
                base(event, cmd)
                return None                                          # the command may have run: the echo is lost
            return base(event, cmd)
        n, res = self.run_macro(script)
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertIn(('cmd', 'MW_REMOVE'), n.calls)                 # armed from the count, not from the give echo
        self.assertEqual(res['weapons']['katana']['removed'], 'removed')

    def test_scenario_deadline_survives_a_normal_run_and_is_cleared_on_an_error(self):
        deadlines = []
        scripts = {t: good_script(t) for t in gw.PACK_WEAPONS}
        current = {'tag': None}
        holder = {}

        def script(event, cmd):
            if event == 'MW_COUNT':
                current['tag'] = re.search(r"MW_COUNT ok='..tostring\(ok\)..' (\w+) ", cmd).group(1)
                deadlines.append((current['tag'], holder['n'].scenario_deadline))
            return scripts[current['tag']](event, cmd)
        with tempfile.TemporaryDirectory() as d:
            n = FakeNight(d, script)
            holder['n'] = n
            gw.run_weapon_macro(n, _Pad(), _G(), {'runs': []}, 'melee_weapons')
        self.assertEqual(deadlines, [(t, 99.0) for t in gw.PACK_WEAPONS])      # still armed while weapons 2..4 run
        self.assertEqual(n.scenario_deadline, 99.0)
        n, err = self.run_macro(good_script('katana'), raises='mw_strong_attack')
        self.assertIsNone(n.scenario_deadline)

    def test_results_of_finished_weapons_survive_an_error(self):
        scripts = {t: good_script(t) for t in gw.PACK_WEAPONS}
        current = {'tag': None}

        def script(event, cmd):
            if event == 'MW_COUNT':
                current['tag'] = re.search(r"MW_COUNT ok='..tostring\(ok\)..' (\w+) ", cmd).group(1)
            return scripts[current['tag']](event, cmd)
        out = {'name': 'melee_weapons'}
        with tempfile.TemporaryDirectory() as d:
            n = FakeNight(d, script)
            orig = n.play

            def play(pad, g, name, plan, sc=None):
                if current['tag'] == 'blunt' and name == 'mw_strong_attack':
                    raise RuntimeError('boom')
                return orig(pad, g, name, plan, sc)
            n.play = play
            with self.assertRaises(RuntimeError):
                gw.run_weapon_macro(n, _Pad(), _G(), {'runs': []}, 'melee_weapons', out)
        self.assertEqual(out['name'], 'melee_weapons')
        self.assertEqual(list(out['weapons']), ['knife', 'katana', 'blunt'])
        self.assertEqual(out['weapons']['knife']['verdict'], 'PASS')
        windows = {(w['weapon'], w['phase']) for w in out['phase_windows']}
        self.assertIn(('knife', 'remove'), windows)
        self.assertIn(('katana', 'remove'), windows)
        self.assertIn(('blunt', 'light_combo'), windows)               # the interrupted weapon keeps its phases
        self.assertIn(('blunt', 'remove'), windows)                    # and was cleaned up
        self.assertTrue(any('RuntimeError' in f for f in out['fails']))
        self.assertNotIn('verdict', out)

    def test_owned_item_is_kept(self):
        n, res = self.run_macro(good_script('katana', how='had'))
        r = res['weapons']['katana']
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertNotIn(('cmd', 'MW_REMOVE'), n.calls)
        self.assertIn('kept', r['removed'])

    def test_removal_waits_until_the_hand_is_empty(self):
        base = good_script('katana')
        seen = {'holstered': False, 'reads': 0}

        def script(event, cmd):
            if event == 'MW_HOLSTER':
                seen['holstered'] = True
            if event == 'MW_INHAND' and seen['holstered']:
                seen['reads'] += 1
                if seen['reads'] < 3:                               # still in the hand for two reads
                    return ['CET [night] MW_INHAND ok=true w_melee_katana | meleefull flag=1']
            return base(event, cmd)
        n, res = self.run_macro(script)
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertEqual(n.calls[-5:], [('cmd', 'MW_HOLSTER'), ('cmd', 'MW_INHAND'), ('cmd', 'MW_INHAND'),
                                        ('cmd', 'MW_INHAND'), ('cmd', 'MW_REMOVE')])

    def test_failed_removal_fails_the_run(self):
        n, res = self.run_macro(good_script('katana', remove='NOT removed 1->1 r=false/false'))
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertTrue(any('remove' in f for f in res['fails']))

    def fast_clock(self):
        """gw.time with a clock that advances 0.5 s per read (the guard's wait does nothing in tests)."""
        ticks = {'t': 0.0}

        def monotonic():
            ticks['t'] += 0.5
            return ticks['t']
        orig = gw.time
        gw.time = types.SimpleNamespace(monotonic=monotonic)
        self.addCleanup(setattr, gw, 'time', orig)

    def test_holster_falls_back_to_the_y_double_tap(self):
        self.fast_clock()
        base = good_script('katana')

        def script(event, cmd):
            if event == 'play:melee_holster':
                return [MASK_OFF]
            if event == 'MW_HOLSTER':
                return ['CET [night] MW_HOLSTER ok=true eqsystem']
            return base(event, cmd)
        n, res = self.run_macro(script)
        self.assertIn(('play', 'melee_holster'), n.calls)
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertTrue(res['weapons']['katana']['phases'][-2]['y_double_tap'])

    def test_mask_that_never_clears_fails(self):
        self.fast_clock()
        base = good_script('katana')

        def script(event, cmd):
            if event == 'MW_HOLSTER':
                return ['CET [night] MW_HOLSTER ok=true eqsystem']
            return base(event, cmd)
        n, res = self.run_macro(script)
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertTrue(any('mask still active' in f for f in res['fails']))

    def test_covered_weapon_without_full_body_reports_the_own_clip_sign(self):
        n, res = self.run_macro(good_script('katana', full_on_standing=False))
        self.assertEqual(res['verdict'], 'FAIL')
        phases = [p['phase'] for p in res['own_clip_phases']]
        self.assertIn('light_combo', phases)
        self.assertTrue(all(p['weapon'] == 'katana' for p in res['own_clip_phases']))

    def test_exception_still_holsters_and_removes(self):
        n, err = self.run_macro(good_script('katana'), raises='mw_strong_attack')
        self.assertIsInstance(err, RuntimeError)
        self.assertEqual(n.calls[-3:], [('cmd', 'MW_HOLSTER'), ('cmd', 'MW_INHAND'), ('cmd', 'MW_REMOVE')])
        self.assertIsNone(n.scenario_deadline)

    def test_cyberware_needs_the_env_switch(self):
        os.environ.pop(gw.ALLOW_CYBERWARE_ENV, None)
        n, res = self.run_macro(good_script('katana'), 'melee_mantis')
        self.assertEqual(res['verdict'], 'SKIP')
        self.assertEqual(n.calls, [])

    def strong_arms_script(self, full_on=False, prev=0):
        held = {'now': False}
        states = gw.load_script_tables()['default_states']

        def script(event, cmd):
            if event == 'MW_CW_EQUIP':
                return [f'CET [night] MW_CW_EQUIP ok=true strong_arms rec=Items.StrongArms given prev={prev}']
            if event == 'MW_CW_DRAW':
                held['now'] = True
                return ['CET [night] MW_CW_DRAW ok=true eqsystem']
            if event == 'MW_INHAND':
                return [f'CET [night] MW_INHAND ok=true {"w_strong_arms" if held["now"] else "none"} | meleefull flag=0']
            if event in ('play:mw_light_combo', 'play:mw_strong_attack', 'play:melee_block'):
                out = [MASK_ON.format(why='flag')] if event == 'play:mw_light_combo' else []
                out.append(FULL.format(onoff='on' if full_on else 'off', why='on' if full_on else 'flag', speed='0.00', state=11,
                                       states=states))
                return out
            if event == 'MW_HOLSTER':
                held['now'] = False
                return ['CET [night] MW_HOLSTER ok=true eqsystem', MASK_OFF]
            if event == 'MW_CW_RESTORE':
                return [f'CET [night] MW_CW_RESTORE ok=true strong_arms restored={1 if prev else 0} removed=1']
            return []
        return script

    def test_strong_arms_runs_the_standing_phases_with_the_full_body_off(self):
        os.environ[gw.ALLOW_CYBERWARE_ENV] = '1'
        self.addCleanup(os.environ.pop, gw.ALLOW_CYBERWARE_ENV, None)
        n, res = self.run_macro(self.strong_arms_script(prev=1234), 'melee_strong_arms')
        r = res['weapons']['strong_arms']
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertFalse(r['covered'])                              # the fists variable is covered, the cyberware is not
        self.assertEqual(r['expected_states'], gw.load_script_tables()['default_states'])
        self.assertEqual([p['phase'] for p in r['phases']], gw.phase_plan('strong_arms'))
        self.assertEqual([c for c in n.calls if c[0] in ('play', 'walk')],
                         [('play', 'mw_light_combo'), ('play', 'mw_strong_attack'), ('play', 'melee_block')])
        self.assertIn(('cmd', 'MW_CW_RESTORE'), n.calls)
        self.assertNotIn(('cmd', 'MW_REMOVE'), n.calls)
        self.assertEqual(r['removed'], 'removed')
        self.assertEqual(res['own_clip_phases'], [])

    def test_strong_arms_with_the_full_body_on_fails(self):
        os.environ[gw.ALLOW_CYBERWARE_ENV] = '1'
        self.addCleanup(os.environ.pop, gw.ALLOW_CYBERWARE_ENV, None)
        n, res = self.run_macro(self.strong_arms_script(full_on=True), 'melee_strong_arms')
        self.assertEqual(res['verdict'], 'FAIL')
        self.assertTrue(any('does not cover' in f for f in res['fails']), res['fails'])
        self.assertIn(('cmd', 'MW_CW_RESTORE'), n.calls)            # the loadout is restored on a failure too

    def test_strong_arms_warns_when_the_pose_recording_is_off(self):
        os.environ[gw.ALLOW_CYBERWARE_ENV] = '1'
        self.addCleanup(os.environ.pop, gw.ALLOW_CYBERWARE_ENV, None)
        orig = FakeNight.__init__

        def init(self_, tmp, script):
            orig(self_, tmp, script)
            self_.args = types.SimpleNamespace(diag='off')
        FakeNight.__init__ = init
        self.addCleanup(setattr, FakeNight, '__init__', orig)
        n, res = self.run_macro(self.strong_arms_script(), 'melee_strong_arms')
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertTrue(any('pose recording' in x for x in res['warnings']), res['warnings'])

    def test_strong_arms_needs_the_env_switch(self):
        os.environ.pop(gw.ALLOW_CYBERWARE_ENV, None)
        n, res = self.run_macro(self.strong_arms_script(), 'melee_strong_arms')
        self.assertEqual(res['verdict'], 'SKIP')
        self.assertEqual(n.calls, [])

    def test_control_weapon_keeps_the_full_body_off(self):
        n, res = self.run_macro(good_script('hammer', full_on_standing=False), 'melee_hammer')
        self.assertEqual(res['verdict'], 'PASS', res['fails'])
        self.assertFalse(res['weapons']['hammer']['covered'])
        self.assertEqual(res['own_clip_phases'], [])


class DryRun(unittest.TestCase):
    def test_prints_commands_for_every_weapon(self):
        scen = load_scenarios(gw.SCENARIO_FILE)
        for tag in gw.WEAPONS:
            text = '\n'.join(gw.dry_run_lines(tag, scen))
            self.assertIn(gw.WEAPONS[tag]['anim_var'], text)
            self.assertIn('MW_HOLSTER', text)
            self.assertIn('MW_INHAND', text)
            self.assertIn('phases  : equip -> draw', text)
        self.assertIn(gw.ALLOW_CYBERWARE_ENV, '\n'.join(gw.dry_run_lines('mantis', scen)))
        text = '\n'.join(gw.dry_run_lines('strong_arms', scen))
        self.assertIn(gw.ALLOW_CYBERWARE_ENV, text)
        self.assertIn('Items.StrongArms', text)
        self.assertIn('pose    : needs the pose recording', text)
        self.assertIn('walk_strikes  : not run for this weapon', text)
        self.assertIn('script covers it: False', text)
        self.assertIn('phases  : equip -> draw -> light_combo -> strong_attack -> block -> holster -> remove', text)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertEqual(gw.main(['--dry-run', '--weapons', 'knife,katana,blunt,baton']), 0)
        self.assertIn('== melee_baton', out.getvalue())


if __name__ == '__main__':
    unittest.main()
