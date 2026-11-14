"""Offline tests of gt_settings.py (the ledger of settings a test run changes and restores).

  python Source\\tools\\gametest\\test_gt_settings.py -v

FakeIo plays the mod's settings API: Get / Set of a store; setting the diagnostic logs option to "off" returns the five switches it
holds to their registered defaults (false), like the real option does, which is why the ledger writes the previous values back.
"""
from __future__ import annotations

import json
import os
import re
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_settings as gs  # noqa: E402

LIT = r"""('(?:[^'\\]|\\.)*'|true|false|-?[\d.]+)"""


def unlit(s):
    if s in ('true', 'false'):
        return s == 'true'
    if s.startswith("'"):
        return s[1:-1].replace("\\'", "'").replace('\\\\', '\\')
    f = float(s)
    return int(f) if f.is_integer() and '.' not in s else f


def typed(v):
    if v is None:
        return 'nil:nil'
    if isinstance(v, bool):
        return 'boolean:' + str(v).lower()
    if isinstance(v, (int, float)):
        return f'number:{v}'
    return f'string:{v}'


class FakeIo:
    def __init__(self, store, pending=None, lose=()):
        self.store = dict(store)
        self.pending = pending                   # path of the pending file: its existence is sampled at every Set
        self.sent = []                           # (kind, payload)
        self.pending_at_set = []
        self.lose = set(lose)                    # echo tags / kinds whose echo is lost: 'get', 'set', 'set#2'
        self.sets = 0
        self.no_save_seen = []

    def run(self, cmd, tag, label):
        if tag == 'SETGET':
            keys = [unlit(x) for x in re.findall(LIT, re.search(r'ipairs\(\{(.*?)\}\)', cmd).group(1))]
            self.sent.append(('get', keys))
            if 'get' in self.lose:
                return None
            body = ';'.join(f'{k}={typed(self.store.get(k))}' for k in keys)
            return f'[night] SETGET ok=true {body}'
        if tag == 'SETSET':
            body = re.search(r'ipairs\(\{(.*?)\}\) do', cmd).group(1)
            pairs = [(unlit(a), unlit(b)) for a, b in re.findall(r'\{' + LIT + ',' + LIT + r'\}', body)]
            self.sets += 1
            self.sent.append(('set', pairs))
            self.no_save_seen.append('noSave=true' in cmd)
            self.pending_at_set.append(bool(self.pending and os.path.isfile(self.pending)))
            if 'set' in self.lose or f'set#{self.sets}' in self.lose:
                return None
            for k, v in pairs:
                self.store[k] = v
                if k == gs.DIAG_KEY and v == 'off':
                    for d in gs.DIAG_DEPS:
                        self.store[d] = False
                if k in gs.DIAG_DEPS and self.store.get(gs.DIAG_KEY) not in (None, 'off') and v is not True:
                    self.store[k] = True          # held on while the logs option is on
            return '[night] SETSET ok=true all'
        self.sent.append(('cmd', tag))
        return f'[night] {tag} ok=true'


START = {gs.DIAG_KEY: 'off', 'core.log': False, 'mm.trace': True, 'mm.feedDump': False, 'camera.trace': False,
         'perspective.log': True, 'perspective.aiming': 'fpp'}


class ParseTests(unittest.TestCase):
    def test_literals(self):
        self.assertEqual(gs.lua_literal(True), 'true')
        self.assertEqual(gs.lua_literal(2.5), '2.5')
        self.assertEqual(gs.lua_literal("it's"), "'it\\'s'")

    def test_parse_get_types_and_missing(self):
        txt = '[x] [night] SETGET ok=true a=string:off;b=boolean:true;c=number:2.5;d=nil:nil;e=number:3'
        self.assertEqual(gs.parse_get(txt, ['a', 'b', 'c', 'd', 'e']), {'a': 'off', 'b': True, 'c': 2.5, 'e': 3})
        self.assertIsNone(gs.parse_get('[night] SETGET ok=false boom', ['a']))
        self.assertIsNone(gs.parse_get(None, ['a']))

    def test_parse_set(self):
        self.assertEqual(gs.parse_set('[night] SETSET ok=true all'), (True, 'all'))
        self.assertEqual(gs.parse_set('[night] SETSET ok=false core.log:unknown'), (False, 'core.log:unknown'))
        self.assertEqual(gs.parse_set('nothing'), (None, ''))

    def test_commands_carry_the_save_flag_and_pairs(self):
        c = gs.set_cmd([('core.diagnosticLogs', 'until'), ('core.log', True)], no_save=False)
        self.assertIn('{noSave=false}', c)
        self.assertIn("{'core.diagnosticLogs','until'},{'core.log',true}", c)
        self.assertIn('{noSave=true}', gs.set_cmd([('a', 1)]))
        self.assertIn("'core.log','mm.trace'", gs.get_cmd(['core.log', 'mm.trace']))

    def test_unowned_settings_in_raw_commands(self):
        cmds = ["local ok,e=pcall(function() return GetMod('utpp_core').Set('core.diagnosticLogs','timed') end)",
                'GetMod("utpp_core").Set("mm.debug", true)', 'Game.UTPPSetMMEnabled(true)']
        self.assertEqual(gs.scan_unowned(cmds), ['core.diagnosticLogs', 'mm.debug'])


class LedgerTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='gs_')
        self.pending = os.path.join(self.dir, 'pending_restore.json')
        self.msgs = []

    def ledger(self):
        return gs.Ledger(self.pending, log=self.msgs.append)

    def test_diag_is_set_then_restored_with_the_held_switches_as_they_were(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        ent = led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        self.assertEqual(ent['state'], 'applied')
        self.assertEqual(io.store[gs.DIAG_KEY], 'until')
        self.assertTrue(io.pending_at_set[0])                      # written ahead of the change
        self.assertEqual(io.sent[0], ('get', [gs.DIAG_KEY, *gs.DIAG_DEPS]))
        self.assertTrue(io.no_save_seen[0])
        res = led.restore_all(io, 'before ExitGame')
        self.assertEqual(res['restored'], 1)
        for k, v in START.items():
            self.assertEqual(io.store[k], v, k)                     # mm.trace and perspective.log are back to true
        self.assertFalse(os.path.exists(self.pending))
        self.assertEqual(led.record()[0]['state'], 'restored')
        self.assertEqual(led.open_entries(), [])

    def test_an_unreadable_previous_value_changes_nothing(self):
        io = FakeIo(START, self.pending, lose=['get'])
        led = self.ledger()
        ent = led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        self.assertEqual(ent['state'], 'not_applied')
        self.assertIn('could not be read', ent['note'])
        self.assertFalse([s for s in io.sent if s[0] == 'set'])
        self.assertFalse(os.path.exists(self.pending))
        self.assertEqual(io.store[gs.DIAG_KEY], 'off')
        io2 = FakeIo({k: v for k, v in START.items() if k != gs.DIAG_KEY}, self.pending)   # the key is not set at all
        self.assertEqual(self.ledger().apply(io2, gs.DIAG_KEY, 'until', '--diag')['state'], 'not_applied')

    def test_lost_set_echo_still_restores(self):
        io = FakeIo(START, self.pending, lose=['set#1'])
        led = self.ledger()
        ent = led.apply(io, gs.DIAG_KEY, 'timed', '--diag', deps=gs.DIAG_DEPS)
        self.assertEqual(ent['state'], 'applied')                   # the command may have run
        self.assertIn('no echo', ent['note'])
        self.assertEqual(led.restore_all(io, 'x')['restored'], 1)

    def test_restore_failure_is_retried_once_then_reported_and_kept(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        io.lose = {'set#2', 'set#3'}
        res = led.restore_all(io, 'before ExitGame')
        self.assertEqual((res['restored'], res['failed']), (0, [gs.DIAG_KEY]))
        self.assertEqual(led.record()[0]['state'], 'restore_failed')
        self.assertTrue(os.path.isfile(self.pending))
        io.lose = set()
        self.assertEqual(led.restore_all(io, 'again')['restored'], 1)
        self.assertFalse(os.path.exists(self.pending))

    def test_game_gone_keeps_the_pending_file_and_says_so_loudly(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        self.assertEqual(led.game_gone('the game ended'), [gs.DIAG_KEY])
        self.assertTrue(any(gs.DIAG_LEFT_TEXT in m for m in self.msgs), self.msgs)
        self.assertEqual(led.record()[0]['state'], 'left')
        with open(self.pending, encoding='utf-8') as f:
            data = json.load(f)
        self.assertEqual(data['entries'][0]['key'], gs.DIAG_KEY)
        self.assertEqual(data['entries'][0]['previous'][0], [gs.DIAG_KEY, 'off'])

    def test_next_launch_restores_first_with_the_persistent_save(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.game_gone('gone')
        # a new runner process, a new game session: the option is still "until" (it was saved)
        io2 = FakeIo({**START, gs.DIAG_KEY: 'until', 'core.log': True, 'mm.feedDump': True}, self.pending)
        led2 = self.ledger()
        rec = led2.restore_pending(io2)
        self.assertEqual(rec['result'], 'restored')
        self.assertEqual(io2.no_save_seen, [False])                  # persistent: the saved "until" is overwritten
        for k, v in START.items():
            self.assertEqual(io2.store[k], v, k)
        self.assertFalse(os.path.exists(self.pending))
        self.assertIsNone(self.ledger().restore_pending(io2))         # nothing pending any more

    def test_next_launch_leaves_a_value_that_is_already_right(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'timed', '--diag', deps=gs.DIAG_DEPS)
        led.game_gone('gone')
        io2 = FakeIo(START, self.pending)                            # noSave: the file never got the change
        rec = self.ledger().restore_pending(io2)
        self.assertEqual(rec['result'], 'already as before')
        self.assertFalse([s for s in io2.sent if s[0] == 'set'])
        self.assertFalse(os.path.exists(self.pending))

    def test_next_launch_never_overwrites_a_value_somebody_set_meanwhile(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.game_gone('gone')
        io2 = FakeIo({**START, gs.DIAG_KEY: 'timed'}, self.pending)          # the player chose "timed" after the crash
        rec = self.ledger().restore_pending(io2)
        self.assertIn('left alone', rec['result'])
        self.assertEqual(rec['entries'][0]['verdict'], 'changed since: left alone')
        self.assertFalse([s for s in io2.sent if s[0] == 'set'])
        self.assertEqual(io2.store[gs.DIAG_KEY], 'timed')
        self.assertFalse(os.path.exists(self.pending))

    def test_next_launch_writes_nothing_when_the_current_values_cannot_be_read(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.game_gone('gone')
        io2 = FakeIo({**START, gs.DIAG_KEY: 'until'}, self.pending, lose=['get'])
        led2 = self.ledger()
        rec = led2.restore_pending(io2)
        self.assertIn('could not be read', rec['result'])
        self.assertFalse([s for s in io2.sent if s[0] == 'set'])               # a setting that cannot be read is not changed
        self.assertTrue(os.path.isfile(self.pending))
        led2.register('perspective.aiming', 'fpp', 'tpp', 'aim_probe')
        with open(self.pending, encoding='utf-8') as f:
            self.assertEqual([e['key'] for e in json.load(f)['entries']], [gs.DIAG_KEY, 'perspective.aiming'])

    def test_an_entry_whose_previous_value_was_a_fallback_never_reaches_the_pending_file(self):
        led = self.ledger()
        led.register('perspective.aiming', 'fpp', 'tpp', 'aim_probe', volatile=True, restore=('CMD', 'AIMPROBE aiming=fpp'))
        self.assertFalse(os.path.exists(self.pending))
        led.game_gone('gone')
        self.assertFalse(os.path.exists(self.pending))

    def test_failed_pending_restore_survives_the_next_write(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.game_gone('gone')
        io2 = FakeIo({**START, gs.DIAG_KEY: 'until'}, self.pending, lose=['set'])
        led2 = self.ledger()
        self.assertIn('FAILED', led2.restore_pending(io2)['result'])
        self.assertTrue(os.path.isfile(self.pending))
        led2.register('perspective.aiming', 'fpp', 'tpp', 'aim_probe')   # a new write must not drop the old entry
        with open(self.pending, encoding='utf-8') as f:
            keys = [e['key'] for e in json.load(f)['entries']]
        self.assertEqual(keys, [gs.DIAG_KEY, 'perspective.aiming'])

    def test_unreadable_pending_file_is_reported(self):
        with open(self.pending, 'w') as f:
            f.write('{broken')
        rec = self.ledger().restore_pending(FakeIo(START))
        self.assertIn('error', rec)

    def test_macro_entries_registered_restored_and_volatile(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        aim = led.register('perspective.aiming', 'fpp', 'tpp', 'aim_probe')
        fists = led.register('equipment.fists', 'away', 'raised', 'melee_fists', volatile=True, restore=('FISTSCMD', 'FISTS_OFF'))
        with open(self.pending, encoding='utf-8') as f:
            self.assertEqual([e['key'] for e in json.load(f)['entries']], ['perspective.aiming'])   # fists do not survive a restart
        led.mark_restored(aim)
        self.assertFalse(os.path.exists(self.pending))
        res = led.restore_all(io, 'before ExitGame')          # the fists entry is still open: its own command is sent
        self.assertEqual(res['restored'], 1)
        self.assertEqual(io.sent[-1], ('cmd', 'FISTS_OFF'))
        self.assertEqual(fists['state'], 'restored')

    def test_listed_only_entries_are_never_sent_to_the_settings_api(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.register('inventory.knife', 'as before', 'given', 'melee_weapons', volatile=True)
        res = led.restore_all(io, 'before ExitGame')
        self.assertEqual(res.get('unhandled'), ['inventory.knife'])
        self.assertEqual(io.sent, [])
        self.assertEqual(led.record()[0]['state'], 'not_restored')
        self.assertEqual(led.open_entries(), [])

    def test_left_text_is_worded_from_the_requested_value(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        on = led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        self.assertEqual(gs.left_text(on), gs.DIAG_LEFT_TEXT)
        io2 = FakeIo({**START, gs.DIAG_KEY: 'until'})
        off = self.ledger().apply(io2, gs.DIAG_KEY, 'off', '--diag', deps=gs.DIAG_DEPS)
        self.assertIn('switched off', gs.left_text(off))
        self.assertIn("'until'", gs.left_text(off))
        self.assertNotIn('left ON', gs.left_text(off))

    def test_several_entries_restore_newest_first_in_one_command(self):
        io = FakeIo(START, self.pending)
        led = self.ledger()
        led.register('perspective.aiming', 'fpp', 'tpp', 'aim_probe')
        io.store['perspective.aiming'] = 'tpp'
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.restore_all(io, 'x')
        sets = [s for s in io.sent if s[0] == 'set']
        self.assertEqual(sets[-1][1][0][0], gs.DIAG_KEY)              # the newest entry first, diag before its held switches
        self.assertEqual(io.store['perspective.aiming'], 'fpp')
        self.assertEqual(io.store['mm.trace'], True)
        self.assertEqual(len(sets), 2)                                # one apply + ONE restore for both entries

    def test_no_pending_path_writes_nothing(self):
        led = gs.Ledger(None, log=self.msgs.append)
        io = FakeIo(START)
        led.apply(io, gs.DIAG_KEY, 'until', '--diag', deps=gs.DIAG_DEPS)
        led.restore_all(io, 'x')
        self.assertEqual(os.listdir(self.dir), [])


LUA_DLL = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'Tools', 'lua-5.1.5', 'lua51.dll'))

LUA_MOCK = """
LOG = {}
Game = { UTPPLogLine = function(s) LOG[#LOG+1] = s end }
STORE = { ['core.diagnosticLogs'] = 'off', ['core.log'] = false, ['mm.trace'] = true, ['perspective.aiming'] = 'fpp' }
SAVED = {}
local core = {}
function core.Get(key) return STORE[key] end
function core.Set(key, value, opts)
  if STORE[key] == nil then return nil, 'unknown setting ' .. tostring(key) end
  STORE[key] = value
  SAVED[key] = not (opts and opts.noSave)
  return value
end
function GetMod(name) if name == 'utpp_core' then return core end end
"""


class LuaTests(unittest.TestCase):
    """The generated CET lines run in the real Lua 5.1 interpreter against a mock of the settings API."""

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(LUA_DLL):
            raise unittest.SkipTest('Lua 5.1 DLL not available')
        import ctypes
        cls.lua = lua = ctypes.CDLL(LUA_DLL)
        lua.luaL_newstate.restype = ctypes.c_void_p
        for name, args in (('luaL_openlibs', [ctypes.c_void_p]), ('luaL_loadstring', [ctypes.c_void_p, ctypes.c_char_p]),
                           ('lua_pcall', [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]),
                           ('lua_tolstring', [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]), ('lua_settop', [ctypes.c_void_p, ctypes.c_int]),
                           ('lua_close', [ctypes.c_void_p])):
            getattr(lua, name).argtypes = args
        lua.luaL_loadstring.restype = ctypes.c_int
        lua.lua_pcall.restype = ctypes.c_int
        lua.lua_tolstring.restype = ctypes.c_char_p

    def setUp(self):
        self.state = self.lua.luaL_newstate()
        self.lua.luaL_openlibs(self.state)
        self.run_lua(LUA_MOCK)

    def tearDown(self):
        self.lua.lua_close(self.state)

    def run_lua(self, code):
        rc = self.lua.luaL_loadstring(self.state, code.encode('utf-8')) or self.lua.lua_pcall(self.state, 0, 1, 0)
        out = self.lua.lua_tolstring(self.state, -1, None)
        text = out.decode('utf-8') if out else ''
        self.lua.lua_settop(self.state, 0)
        self.assertEqual(rc, 0, text)
        return text

    def log(self):
        return self.run_lua("return table.concat(LOG, '\\n')")

    def test_get_reads_typed_values_and_unset_keys(self):
        self.run_lua(gs.get_cmd(['core.diagnosticLogs', 'core.log', 'mm.trace', 'not.set']))
        txt = self.log()
        self.assertEqual(gs.parse_get(txt, ['core.diagnosticLogs', 'core.log', 'mm.trace', 'not.set']),
                         {'core.diagnosticLogs': 'off', 'core.log': False, 'mm.trace': True})

    def test_set_writes_values_with_the_save_flag_and_reports_all(self):
        self.run_lua(gs.set_cmd([('core.diagnosticLogs', 'until'), ('core.log', True)], no_save=True))
        self.assertEqual(gs.parse_set(self.log()), (True, 'all'))
        self.assertEqual(self.run_lua("return STORE['core.diagnosticLogs'] .. tostring(STORE['core.log']) .. tostring(SAVED['core.log'])"),
                         'untiltruefalse')
        self.run_lua(gs.set_cmd([('core.log', False)], no_save=False))
        self.assertEqual(self.run_lua("return tostring(SAVED['core.log'])"), 'true')

    def test_set_of_false_is_not_an_error_and_an_unknown_key_is(self):
        self.run_lua(gs.set_cmd([('core.log', False)]))
        self.assertEqual(gs.parse_set(self.log()), (True, 'all'))                  # false is a value, not a failure
        self.run_lua('LOG = {}')
        self.run_lua(gs.set_cmd([('core.log', True), ('nope', 1)]))
        ok, info = gs.parse_set(self.log())
        self.assertFalse(ok)
        self.assertIn('nope:unknown setting nope', info)
        self.assertEqual(self.run_lua("return tostring(STORE['core.log'])"), 'true')   # the rest was still written


class SessionSettingsTests(unittest.TestCase):
    """--settings author | shipped: the values of the session, what differs from the shipped defaults, and the way back."""

    PLAYER = {'mm.bodyYawOffset': 8, 'mm.pelvisYawTrim': 5, 'camera.side': 1, 'mm.shinobiBonusReload': False,
              'perspective.scene': True, 'perspective.blend': 0.8}

    def test_plans(self):
        self.assertEqual(dict(gs.session_plan('author')),
                         {'mm.bodyYawOffset': 4, 'mm.pelvisYawTrim': 2, 'camera.side': 1, 'mm.shinobiBonusReload': True,
                          'perspective.scene': False, 'perspective.blend': 0.6})
        shipped = dict(gs.session_plan('shipped'))
        self.assertEqual(shipped, {'mm.bodyYawOffset': 8, 'mm.pelvisYawTrim': 5, 'mm.shinobiBonusReload': False,
                                   'perspective.scene': True, 'perspective.blend': 0.8})      # camera.side: no known default

    def test_difference_from_shipped(self):
        diff = {d['key']: d for d in gs.differs_from_shipped(dict(gs.session_plan('author')))}
        self.assertEqual(set(diff), {'mm.bodyYawOffset', 'mm.pelvisYawTrim', 'camera.side', 'mm.shinobiBonusReload',
                                     'perspective.scene', 'perspective.blend'})
        self.assertEqual((diff['mm.bodyYawOffset']['value'], diff['mm.bodyYawOffset']['shipped']), (4, 8))
        self.assertIsNone(diff['camera.side']['shipped'])
        self.assertEqual(gs.differs_from_shipped(dict(gs.session_plan('shipped'))), [])
        self.assertEqual(gs.differs_from_shipped({'perspective.blend': 0.8}), [])
        line = gs.settings_line('author', dict(gs.session_plan('author')))
        self.assertIn('settings author: differ from the shipped defaults: mm.bodyYawOffset 4 (shipped 8)', line)
        self.assertIn('camera.side 1 (shipped unknown)', line)
        self.assertEqual(gs.settings_line('shipped', dict(gs.session_plan('shipped'))), 'settings shipped: all as shipped')
        self.assertIn('NOT applied: camera.side', gs.settings_line('author', {}, ['camera.side']))

    def make_night(self, mode, store):
        import night
        args = night.build_parser().parse_args(['--out-root', 'unused', '--log-dir', tempfile.mkdtemp(), '--settings', mode])
        n = night.Night(args, runner=lambda c, **k: None)
        n.settings.pending_path = None
        n.cycle = {'name': 't', 'steps': []}
        n.log = lambda msg: None
        io = FakeIo(store)
        n.settings_io = lambda pad, guard: io
        return n, io

    def test_the_night_applies_the_values_records_them_and_puts_everything_back(self):
        for mode in ('author', 'shipped'):
            n, io = self.make_night(mode, self.PLAYER)
            info = {}
            n.apply_session_settings(None, info)
            plan = dict(gs.session_plan(mode))
            for k, v in plan.items():
                self.assertEqual(io.store[k], v, (mode, k))
            rec = info['session_settings']
            self.assertEqual(rec['mode'], mode)
            self.assertEqual(rec['values'], plan)
            self.assertEqual(n.cycle['session_settings'], rec)
            self.assertEqual(bool(rec['differs_from_shipped']), mode == 'author')
            self.assertTrue(all(e['owner'] == '--settings' and e['state'] == 'applied' for e in n.settings.entries))
            res = n.settings.restore_all(io, 'test')
            self.assertEqual(res['failed'], [])
            self.assertEqual(io.store, self.PLAYER, mode)                     # the player's values are back

    def test_a_cet_command_that_sets_the_same_keys_is_warned_about(self):
        n, io = self.make_night('author', self.PLAYER)
        n.args.cet = "for k,v in pairs({['mm.bodyYawOffset']=4}) do c.Set(k,v) end"
        info = {}
        n.apply_session_settings(None, info)
        self.assertEqual(info['session_settings_conflict'], ['mm.bodyYawOffset'])
        n2, io2 = self.make_night('author', self.PLAYER)
        info2 = {}
        n2.apply_session_settings(None, info2)
        self.assertNotIn('session_settings_conflict', info2)

    def test_a_key_the_game_does_not_know_is_listed_not_applied(self):
        store = {k: v for k, v in self.PLAYER.items() if k != 'camera.side'}
        n, io = self.make_night('author', store)
        info = {}
        n.apply_session_settings(None, info)
        rec = info['session_settings']
        self.assertEqual(rec['not_applied'], ['camera.side'])
        self.assertNotIn('camera.side', rec['values'])
        self.assertIn('NOT applied: camera.side', rec['line'])
        self.assertNotIn('camera.side', io.store)

    def test_the_summary_line_reaches_the_body_summary(self):
        import gt_session
        d = {'run': 'r', 'result': 'ok', 'pose_body': 'woman', 'scenarios': {}, 'settings_line': 'settings author: x'}
        lines = gt_session.side_by_side_lines({'female': d})
        self.assertIn('female: settings author: x', lines)


if __name__ == '__main__':
    unittest.main()
