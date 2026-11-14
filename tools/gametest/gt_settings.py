"""Settings of the mod that a test run changes, and the guarantee that they are put back.

Every change goes through a Ledger: the previous value is READ first (a setting that cannot be read is not changed), the change
is written ahead into a pending file, and the restore is sent before every ExitGame, in a macro's finally, and - when the game is
already gone - by the NEXT launch of the runner, which reads the pending file first thing.

Commands are CET Lua lines for the utpp_core settings API (GetMod('utpp_core').Get / .Set), each echoed into the UTPP log:
  [night] SETGET ok=true <key>=<type>:<value>;<key>=<type>:<value>
  [night] SETSET ok=true <what>
The Io object passed to the Ledger sends one command and returns the log text since the echo appeared (None = lost):
  io.run(cmd, tag, label) -> str | None

Diagnostic logs: core.diagnosticLogs = off | timed | until. Switching it OFF returns the five switches it holds
(core.log, mm.trace, mm.feedDump, camera.trace, perspective.log) to their registered defaults, not to what they were, so the Ledger
reads all six first and writes the five back after the logs option.
"""
from __future__ import annotations

import json
import os
import re
from datetime import datetime

DIAG_KEY = 'core.diagnosticLogs'
DIAG_DEPS = ('core.log', 'mm.trace', 'mm.feedDump', 'camera.trace', 'perspective.log')
DIAG_VALUES = {'off': 'off', 'timed': 'timed', 'until': 'until'}
DIAG_LEFT_TEXT = 'diagnostic logs left ON - restore needed'

# The mod settings a test session runs with. 'author' = the values the author plays with (the harness applies them itself,
# through the ledger, so they are put back before the game is quit); 'shipped' = the registered defaults of the same keys.
SETTINGS_AUTHOR = 'author'
SETTINGS_SHIPPED = 'shipped'
SETTINGS_MODES = (SETTINGS_AUTHOR, SETTINGS_SHIPPED)
AUTHOR_VALUES = (('mm.bodyYawOffset', 4), ('mm.pelvisYawTrim', 2), ('camera.side', 1), ('mm.shinobiBonusReload', True),
                 ('perspective.scene', False), ('perspective.blend', 0.6))
# registered defaults (the `default =` of each Register call); None = not known to the harness (nothing is applied for it
# in 'shipped' mode and a difference cannot be named)
SHIPPED_DEFAULTS = {'mm.bodyYawOffset': 8, 'mm.pelvisYawTrim': 5, 'camera.side': None, 'mm.shinobiBonusReload': False,
                    'perspective.scene': True, 'perspective.blend': 0.8}


def session_plan(mode: str) -> list[tuple[str, object]]:
    """[(key, value)] the harness sets after arming for a --settings mode."""
    if mode == SETTINGS_SHIPPED:
        return [(k, SHIPPED_DEFAULTS[k]) for k, _ in AUTHOR_VALUES if SHIPPED_DEFAULTS.get(k) is not None]
    return list(AUTHOR_VALUES)


def _same(a, b) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    try:
        return abs(float(a) - float(b)) < 1e-9
    except (TypeError, ValueError):
        return a == b


def differs_from_shipped(values: dict) -> list[dict]:
    """The session values (key -> value in force) that are not the shipped default: [{'key', 'value', 'shipped'}]; 'shipped' is None
    for a key whose default the harness does not know (listed, because it cannot be called equal)."""
    out = []
    for key, value in values.items():
        shipped = SHIPPED_DEFAULTS.get(key)
        if shipped is None or not _same(value, shipped):
            out.append({'key': key, 'value': value, 'shipped': shipped})
    return out


def settings_line(mode: str, values: dict, not_applied=()) -> str:
    """One line for the run record: which settings of this session differ from the shipped defaults."""
    diff = differs_from_shipped(values)
    text = ', '.join(f'{d["key"]} {d["value"]!r} (shipped {"unknown" if d["shipped"] is None else repr(d["shipped"])})' for d in diff)
    line = f'settings {mode}: ' + (f'differ from the shipped defaults: {text}' if diff else 'all as shipped')
    if not_applied:
        line += f'; NOT applied: {", ".join(not_applied)}'
    return line


_SETGET_RE = re.compile(r'\[night\] SETGET ok=(true|false)\s*(.*)')
_SETSET_RE = re.compile(r'\[night\] SETSET ok=(true|false)\s*(.*)')
_SET_CALL_RE = re.compile(r'''Set\(\s*['"]([\w.]+)['"]''')


def lua_literal(v) -> str:
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (int, float)):
        return repr(v)
    return "'" + str(v).replace('\\', '\\\\').replace("'", "\\'") + "'"


def get_cmd(keys) -> str:
    ks = ','.join(lua_literal(k) for k in keys)
    return ("local ok,e=pcall(function() local c=GetMod('utpp_core') local t={} for _,k in ipairs({" + ks + "}) do "
            "local v=c.Get(k) t[#t+1]=k..'='..type(v)..':'..tostring(v) end return table.concat(t,';') end) "
            "Game.UTPPLogLine('[night] SETGET ok='..tostring(ok)..' '..tostring(e))")


def set_cmd(pairs, no_save: bool = True) -> str:
    ps = ','.join('{' + lua_literal(k) + ',' + lua_literal(v) + '}' for k, v in pairs)
    return ("local ok,e=pcall(function() local c=GetMod('utpp_core') local bad={} for _,kv in ipairs({" + ps + "}) do "
            "local r,err=c.Set(kv[1],kv[2],{noSave=" + ('true' if no_save else 'false') + "}) "
            "if r==nil then bad[#bad+1]=kv[1]..':'..tostring(err) end end "
            "return #bad==0 and 'all' or table.concat(bad,';') end) "
            "Game.UTPPLogLine('[night] SETSET ok='..tostring(ok and e=='all')..' '..tostring(e))")


def _typed(text: str):
    """'string:off' -> 'off'; 'boolean:true' -> True; 'number:2.5' -> 2.5; 'nil:nil' -> None (not set)."""
    kind, _, val = text.partition(':')
    if kind == 'boolean':
        return val == 'true'
    if kind == 'number':
        try:
            f = float(val)
            return int(f) if f.is_integer() and '.' not in val else f
        except ValueError:
            return None
    if kind == 'string':
        return val
    return None


def parse_get(text: str | None, keys) -> dict | None:
    """{key: value} of the last SETGET line; a key the line does not carry or that is not set (nil) is missing. None = no echo / ok=false."""
    if not text:
        return None
    line = None
    for ln in text.splitlines():
        m = _SETGET_RE.search(ln)
        if m:
            line = m
    if line is None or line.group(1) != 'true':
        return None
    out = {}
    for item in line.group(2).split(';'):
        k, _, rest = item.partition('=')
        k = k.strip()
        if k in keys and rest and not rest.startswith('nil:'):
            out[k] = _typed(rest.strip())
    return out


def parse_set(text: str | None) -> tuple[bool | None, str]:
    if not text:
        return None, ''
    line = None
    for ln in text.splitlines():
        m = _SETSET_RE.search(ln)
        if m:
            line = m
    if line is None:
        return None, ''
    return line.group(1) == 'true', line.group(2).strip()[:160]


def left_text(entry: dict) -> str:
    """What the run record says about an entry that could not be put back, worded from the value that was requested."""
    if entry['key'] != DIAG_KEY:
        return f'{entry["key"]} left changed - restore needed'
    if entry.get('requested') in ('timed', 'until'):
        return DIAG_LEFT_TEXT
    prev = entry['previous'][0][1] if entry.get('previous') else '?'
    return f'diagnostic logs left switched off (previous value {prev!r}) - restore needed'


def scan_unowned(cmds) -> list[str]:
    """Keys that raw CET commands (--cet / --cet-file) change through the settings API: the run does not own them."""
    keys = []
    for c in cmds or []:
        for m in _SET_CALL_RE.finditer(c):
            if m.group(1) not in keys:
                keys.append(m.group(1))
    return keys


class Ledger:
    """Entries: {id, key, owner, previous (list of [key, value]), requested, state, volatile, restore (cmd, tag) | None, note}.
    state: applied | restored | restore_failed | left. `previous` lists every key to write back, in order."""

    def __init__(self, pending_path: str | None, log=print, clock=datetime.now):
        self.pending_path = pending_path
        self.log = log
        self.clock = clock
        self.entries: list[dict] = []
        self.startup: dict | None = None            # what the restore of an earlier run's pending file did
        self.carried: list[dict] = []               # entries of an earlier run's pending file that could not be restored yet
        self.cycle_name = ''

    # ---- pending file (write ahead)
    def _write_pending(self) -> None:
        if not self.pending_path:
            return
        todo = [e for e in self.entries if e['state'] in ('applied', 'restore_failed', 'left') and not e['volatile']]
        try:
            if not todo and not self.carried:
                if os.path.isfile(self.pending_path):
                    os.remove(self.pending_path)
                return
            os.makedirs(os.path.dirname(self.pending_path) or '.', exist_ok=True)
            data = {'written': self.clock().strftime('%Y-%m-%d %H:%M:%S'), 'cycle': self.cycle_name,
                    'entries': list(self.carried) + [{'key': e['key'], 'owner': e['owner'], 'previous': e['previous'],
                                                      'requested': e['requested'], 'state': e['state']} for e in todo]}
            tmp = self.pending_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=1)
            os.replace(tmp, self.pending_path)
        except OSError as e:
            self.log(f'[settings] pending file not written: {e!r}')

    # ---- read / apply
    def read(self, io, keys, label: str = 'settings_get') -> dict | None:
        keys = list(keys)
        return parse_get(io.run(get_cmd(keys), 'SETGET', label), keys)

    def apply(self, io, key: str, value, owner: str, deps=(), no_save: bool = True) -> dict:
        """Reads key (+ deps), registers the restore and writes the new value (deps untouched here: the setting itself drives
        them). Returns the entry; entry['state'] 'applied' on success, 'not_applied' (nothing was changed) when the previous
        value could not be read or the write failed."""
        keys = [key] + list(deps)
        cur = self.read(io, keys, f'{owner}_get')
        entry = {'id': len(self.entries) + 1, 'key': key, 'owner': owner, 'requested': value, 'state': 'not_applied',
                 'volatile': False, 'restore': None, 'previous': [], 'note': ''}
        if cur is None or key not in cur:
            entry['note'] = f'previous value of {key} could not be read (no echo or not set) - not changed'
            self.log(f'[settings] {entry["note"]}')
            self.entries.append(entry)
            return entry
        entry['previous'] = [[key, cur[key]]] + [[k, cur[k]] for k in deps if k in cur]
        entry['state'] = 'applied'
        self.entries.append(entry)
        self._write_pending()                       # before the write: a crash from here on is restored by the next launch
        ok, info = parse_set(io.run(set_cmd([(key, value)], no_save), 'SETSET', f'{owner}_set'))
        if ok is not True:
            entry['note'] = f'set {key}={value!r}: ' + ('the command failed (' + info + ')' if ok is False else 'no echo')
            self.log(f'[settings] {entry["note"]}')
            if ok is False:
                entry['state'] = 'not_applied'
                entry['note'] += ' - nothing to restore'
                self._write_pending()
            # no echo: the command may still have run, the entry stays 'applied' and is restored
        else:
            self.log(f'[settings] {owner}: {key} {cur[key]!r} -> {value!r} (previous value remembered, restored before ExitGame)')
        return entry

    def register(self, key: str, previous, requested, owner: str, volatile: bool = False, restore=None, extra_previous=()) -> dict:
        """A change made by the caller's own command: remember how to undo it (the caller marks it restored)."""
        entry = {'id': len(self.entries) + 1, 'key': key, 'owner': owner, 'requested': requested, 'state': 'applied',
                 'volatile': volatile, 'restore': restore, 'previous': [[key, previous]] + [list(x) for x in extra_previous],
                 'note': ''}
        self.entries.append(entry)
        self._write_pending()
        return entry

    def mark_restored(self, entry: dict) -> None:
        entry['state'] = 'restored'
        self._write_pending()

    # ---- restore
    def open_entries(self) -> list[dict]:
        return [e for e in self.entries if e['state'] in ('applied', 'restore_failed')]

    def restore_all(self, io, why: str, no_save: bool = True) -> dict:
        """Sends the restore of every open entry (newest first). Returns {'restored': n, 'failed': [keys], 'why': why}."""
        todo = list(reversed(self.open_entries()))
        res = {'why': why, 'restored': 0, 'failed': []}
        if not todo:
            return res
        generic = [e for e in todo if not e['restore'] and not e['volatile']]
        for e in todo:                                  # listed only: no command can undo it, the macro's own cleanup did not finish
            if e['volatile'] and not e['restore']:
                e['state'] = 'not_restored'
                e['note'] = 'the macro cleanup did not finish and there is no restore command'
                res.setdefault('unhandled', []).append(e['key'])
        for attempt in (1, 2):
            if not generic:
                break
            pairs = [(k, v) for e in generic for k, v in e['previous']]
            ok, info = parse_set(io.run(set_cmd(pairs, no_save), 'SETSET', 'settings_restore'))
            if ok is True:
                for e in generic:
                    e['state'] = 'restored'
                res['restored'] += len(generic)
                generic = []
            else:
                self.log(f'[settings] restore attempt {attempt}/2 ' + ('failed: ' + info if ok is False else 'got no echo'))
        for e in generic:
            e['state'] = 'restore_failed'
            res['failed'].append(e['key'])
        for e in [e for e in todo if e['restore'] and e['state'] != 'not_restored']:
            cmd, tag = e['restore']
            text = io.run(cmd, tag, f'{e["owner"]}_restore')
            if text and f'[night] {tag} ok=true' in text:
                e['state'] = 'restored'
                res['restored'] += 1
            else:
                e['state'] = 'restore_failed'
                res['failed'].append(e['key'])
        self._write_pending()
        self.log(f'[settings] restore ({why}): {res["restored"]} restored' + (f', FAILED {res["failed"]}' if res['failed'] else ''))
        return res

    def game_gone(self, why: str) -> list[str]:
        """The game is not there to take the restore: the open entries stay in the pending file, the next launch restores them."""
        left = []
        for e in self.open_entries():
            e['state'] = 'left'
            e['note'] = f'left changed: {why}'
            left.append(e['key'])
            if e['volatile']:
                self.log(f'[settings] {e["owner"]}: {e["key"]} left changed ({why}) - it does not survive a game restart')
            else:
                self.log(f'[settings] {left_text(e)} ({why}); the next launch of the runner restores it first')
        if left:
            self._write_pending()
        return left

    def left(self) -> list[dict]:
        return [e for e in self.entries if e['state'] == 'left']

    def restore_pending(self, io) -> dict | None:
        """First thing at the next launch: put back what an earlier run left changed. Uses the persistent form of Set (the
        earlier value may have been saved). Returns the record (also self.startup), None = no pending file."""
        if not self.pending_path or not os.path.isfile(self.pending_path):
            return None
        try:
            with open(self.pending_path, encoding='utf-8') as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            self.startup = {'error': repr(e), 'file': self.pending_path}
            self.log(f'[settings] pending restore file unreadable: {e!r}')
            return self.startup
        rec = {'from_cycle': data.get('cycle'), 'written': data.get('written'), 'entries': []}
        entries = data.get('entries', [])
        keys = []
        for e in entries:
            for k, _ in e.get('previous', []):
                if k not in keys:
                    keys.append(k)
        cur = self.read(io, keys, 'pending_get') if keys else {}
        rec['read'] = cur is not None
        if cur is None:
            # a setting that cannot be read is not changed: nothing is written, the file stays for the next launch
            rec['result'] = 'the current values could not be read - nothing written, the file stays'
            self.carried = list(entries)
            self.startup = rec
            self.log(f'[settings] pending restore of cycle {rec["from_cycle"]}: {rec["result"]}')
            return rec
        pairs, leaked = [], []
        for e in entries:
            prev = e.get('previous') or []
            now = cur.get(e.get('key'))
            if prev and now == e.get('requested') and now != prev[0][1]:
                pairs += [(k, v) for k, v in prev]              # the run's change is still there: it leaked
                leaked.append(e)
                verdict = 'leaked'
            elif prev and now == prev[0][1]:
                verdict = 'already as before'
            else:
                verdict = 'changed since: left alone'            # somebody set something else meanwhile: never overwritten
            rec['entries'].append({'key': e.get('key'), 'owner': e.get('owner'), 'previous': prev, 'requested': e.get('requested'),
                                   'current': now, 'verdict': verdict})
        if not leaked:
            rec['result'] = ('already as before' if all(x['verdict'] == 'already as before' for x in rec['entries'])
                             else 'nothing to restore (' + ', '.join(f'{x["key"]}: {x["verdict"]}' for x in rec['entries']) + ')')
            os.remove(self.pending_path)
        else:
            ok, info = parse_set(io.run(set_cmd(pairs, no_save=False), 'SETSET', 'pending_restore'))
            rec['result'] = 'restored' if ok else f'FAILED ({info or "no echo"}) - the file stays'
            if ok:
                os.remove(self.pending_path)
            else:
                self.carried = list(leaked)                      # a later write of the file keeps them
        self.startup = rec
        self.log(f'[settings] pending restore of cycle {rec["from_cycle"]}: {rec["result"]}')
        return rec

    # ---- record
    def record(self) -> list[dict]:
        return [{k: e[k] for k in ('id', 'key', 'owner', 'requested', 'previous', 'state', 'volatile', 'note')}
                for e in self.entries]
