"""Melee weapon scenarios for the night runner (night.py): one parameterised macro per weapon family.

  python gt_melee_weapons.py --dry-run [--weapons knife,katana,blunt,baton]

Sequence per weapon (script commands and virtual pad only, so it also runs with --background):
  equip -> draw -> light combo -> guard / idle pause -> strong attack -> block hold (weapons with a block) -> strikes
  while walking -> sprint attack -> crouch attack -> holster -> remove.
The weapon is given to the player and drawn by script commands (no loadout slot is touched), its anim variable is read back
from the item in the hand, and at the end it is holstered and taken out of the inventory again. Items the player already
owned are used as they are and never removed.

Evidence of every phase comes from the UTPP log window of that phase: `MMMASK` / `MMGATE` melee mask lines, `MMMELEEFULL`
full-body lines (reason, PSM melee state, state mask the DLL applies), `MELEEAIM` strike direction lines with the chest base
correction state, and `utpp_melee_tpp` lines. The state mask and the covered anim variables the script sends are read from
plugin/UTPP/scripts/utpp_melee.reds, so the expectations follow the registered tables. A phase in which a covered weapon
strikes while the full body never switched on is reported as "own clip sign": the game most likely played its own
first-person clip (flag, state bit or registration entry wrong). The log does not name the clip that played for these
weapons; the sign is a hint to look at the recording, not a proof.

Every phase records its wall-clock window (same local clock as the UTPP log) in `phase_windows`, so the pose recording and
the video can be cut per phase.
"""
from __future__ import annotations

import os
import re
import sys
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gt_input import file_size, read_since  # noqa: E402

REDS_REL = os.path.join('..', '..', 'plugin', 'UTPP', 'scripts', 'utpp_melee.reds')
SCENARIO_FILE = os.path.join(HERE, 'scenarios.json')

# env switch for the arm cyberware test: it changes the cyberware loadout (restored at the end)
ALLOW_CYBERWARE_ENV = 'UTPP_NIGHT_ALLOW_CYBERWARE'

# pad parts (scenarios.json): melee_block, melee_punch_walk and melee_holster are shared with the fists macro
PARTS = ['mw_light_combo', 'mw_guard_idle', 'mw_strong_attack', 'melee_block', 'melee_punch_walk', 'mw_sprint_attack',
         'melee_holster']

# pad parts with the crouch button (R3 tap) are built here: scenarios.json keeps the crouch button to the crouch scenario
_R3_TAP = {'t': 0.3, 'button': 'R3', 'hold': 0.08}
_IDLE = {'ls': [[0, 0, 0]], 'rs': [[0, 0, 0]], 'lt': [[0, 0]], 'rt': [[0, 0]], 'metrics': False}
SCRIPTED_PARTS = {
    # crouch (one toggle), 3 RT taps while crouched, stand up (one toggle)
    'mw_crouch_attack': {'steps': [
        dict(_IDLE, name='crouch', duration=1.5, buttons=[_R3_TAP]),
        {'name': 'crouch_strikes', 'duration': 3.0, 'ls': [[0, 0, 0]], 'metrics': False,
         'rt': {'keys': [[0, 0], [0.3, 1], [0.45, 0], [1.0, 1], [1.15, 0], [1.7, 1], [1.85, 0]], 'interp': 'step'}},
        {'name': 'stand', 'duration': 1.5, 'ls': [[0, 0, 0]], 'rt': [[0, 0]], 'buttons': [_R3_TAP], 'metrics': False}]},
    # one crouch toggle: stands V up again when the crouch phase ended crouched
    'mw_crouch_toggle': {'steps': [dict(_IDLE, name='toggle', duration=1.5, buttons=[_R3_TAP])]},
}


def scripted_scenario(name: str):
    from gt_scenario import parse_scenario
    return parse_scenario(name, SCRIPTED_PARTS[name])

# candidate records are tried in order; the in-hand anim variable (item record friendlyName) is the gate. 'covered' = the
# script's full-body list is expected to contain the variable (None = read the script table). 'block' = the weapon has a
# block set; the knife does not (its LT is the throw aim and a thrown knife would leave the hand).
WEAPONS = {
    'knife': {'label': 'knife', 'anim_var': 'w_melee_knife', 'block': False, 'kind': 'weapon',
              'records': ('Items.Preset_Knife_Default', 'Items.Preset_Tanto_Default')},
    'katana': {'label': 'katana', 'anim_var': 'w_melee_katana', 'block': True, 'kind': 'weapon',
               'records': ('Items.Preset_Katana_Default',)},
    'blunt': {'label': 'one-handed blunt', 'anim_var': 'w_melee_one_hand_blunt', 'block': True, 'kind': 'weapon',
              'records': ('Items.Preset_Crowbar_Default', 'Items.Preset_Tire_Iron_Default', 'Items.Preset_Machete_Default')},
    'baton': {'label': 'baton', 'anim_var': 'w_melee_baton', 'block': True, 'kind': 'weapon',
              'records': ('Items.Preset_Baton_Alpha', 'Items.Preset_Baton_Beta', 'Items.Preset_Baton_Gamma',
                          'Items.Base_Baton')},
    'bat': {'label': 'two-handed bat', 'anim_var': 'w_two_hand_blunt', 'block': True, 'kind': 'weapon',
            'records': ('Items.Preset_Baseball_Bat_Default',)},
    # no set of its own in the pack: full body is expected to stay off (control)
    'hammer': {'label': 'two-handed hammer', 'anim_var': 'w_melee_hammer', 'block': True, 'kind': 'weapon',
               'records': ('Items.Preset_Hammer_Default', 'Items.Preset_Hammer_Sasquatch')},
    # arm cyberware: needs the cyberware loadout change, see cyberware_equip_cmd
    'mantis': {'label': 'mantis blades', 'anim_var': 'mantis_blade', 'block': True, 'kind': 'cyberware',
               'records': ('Items.MantisBlades',)},
    # strong arms: arm cyberware with its own anim variable in the hand (w_strong_arms, read in game 2.5); the pack registers
    # third-person fist sets for it behind the StrongArms variable. The script never switches the full body (nor the full pose
    # layer) on for arm cyberware, so 'covered' is False and the state mask is the default one.
    # 'phases' = the standing phases only; 'pose' = the run needs the pose recording (diagnostic logs on) to measure the body.
    'strong_arms': {'label': 'strong arms (fists cyberware)', 'anim_var': 'w_strong_arms', 'block': True, 'kind': 'cyberware',
                    'records': ('Items.StrongArms',), 'covered': False, 'states_default': True,
                    'phases': ('light_combo', 'strong_attack', 'block'), 'pose': True},
}
for _tag, _w in WEAPONS.items():
    _w['tag'] = _tag
    _w['set'] = 'melee_' + _tag

PACK_WEAPONS = ['knife', 'katana', 'blunt', 'baton', 'bat']
WEAPON_SETS = {w['set']: [t] for t, w in WEAPONS.items()}
WEAPON_SETS['melee_weapons'] = list(PACK_WEAPONS)
# registered as scenario sets (gt_night.SETS); 'melee_weapons' = the pack weapons in one launch
SET_MEMBERS = {name: [name] for name in WEAPON_SETS}
SECONDS_PER_WEAPON = 150.0

# PSM.MeleeWeapon states (bit = 1 << state)
STATE_NAMES = {0: 'NotReady', 1: 'Equipping', 2: 'Idle', 3: 'Safe', 4: 'PublicSafe', 5: 'Parried', 6: 'Hold', 7: 'ChargedHold',
               8: 'Block', 9: 'Targeting', 10: 'Deflect', 11: 'ComboAttack', 12: 'FinalAttack', 13: 'StrongAttack',
               14: 'SafeAttack', 15: 'BlockAttack', 16: 'SprintAttack', 17: 'CrouchAttack', 18: 'JumpAttack',
               19: 'ThrowAttack', 20: 'DeflectAttack', 21: 'EquipAttack'}
BLOCK_STATE_BIT = 1 << 8


def time_limit_s(set_name: str) -> float:
    return 60.0 + SECONDS_PER_WEAPON * len(WEAPON_SETS.get(set_name, []))


# ---------------------------------------------------------------------------------------------- script tables

def default_reds_path() -> str:
    return os.path.normpath(os.path.join(HERE, REDS_REL))


def load_script_tables(path: str | None = None) -> dict | None:
    """Covered anim variables, state masks and the default mask the melee script sends; None = file not readable."""
    path = path or default_reds_path()
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return None

    def body(name: str) -> str:
        m = re.search(r'func ' + name + r'\([^)]*\)[^{]*\{(.*?)\n\}', text, re.S)
        return m.group(1) if m else ''
    covered = set(re.findall(r'n"(w_[A-Za-z0-9_]+|mantis_blade)"', body('UTPPMeleeFullBodyCoveredName')))
    guard = set(re.findall(r'n"([A-Za-z0-9_]+)"', body('UTPPMeleeGuardCoveredName')))
    dm = re.search(r'return (\d+);', body('UTPPMeleeStatesDefault'))
    section = re.search(r'UTPP_STATES_TABLE begin(.*?)UTPP_STATES_TABLE end', text, re.S)
    states = {k: int(v) for k, v in re.findall(r'n"([A-Za-z0-9_]+)"\) \{ return (\d+); \}', section.group(1))} if section else {}
    if not covered or not dm:
        return None
    return {'covered': covered, 'guard': guard, 'default_states': int(dm.group(1)), 'states': states}


def expected_states(tables: dict | None, anim_var: str) -> int | None:
    if not tables:
        return None
    return tables['states'].get(anim_var, tables['default_states'])


def is_covered(tables: dict | None, anim_var: str) -> bool | None:
    return None if not tables else anim_var in tables['covered']


# ---------------------------------------------------------------------------------------------- script commands
# Every command ends with a '[night] <TAG> ok=<true|false> ...' echo line (Game.UTPPLogLine).

# strict: a failing or odd GetItemList is an error (a silent empty list would count the item as absent)
_LUA_IDS = ("local function lst() local ok2,a,b=pcall(function() return ts:GetItemList(p) end) "
            "if not ok2 then error('GetItemList failed: '..tostring(a)) end local l=type(b)=='table' and b or a "
            "if type(l)~='table' then error('GetItemList returned '..type(l)) end return l end "
            "local function ids(tdb) local out={} for _,it in ipairs(lst()) do "
            "if ItemID.GetTDBID(it:GetID())==tdb then out[#out+1]=it:GetID() end end return out end ")


def _lua_list(items) -> str:
    return '{' + ','.join("'" + i + "'" for i in items) + '}'


def count_cmd(w: dict) -> str:
    """Read-only: pick the first candidate record whose friendlyName equals the expected anim variable and count the items
    of that record in the inventory. Echo `[night] MW_COUNT ok=.. <tag> rec=<record> fn=<friendlyName> n=<count>`; an error
    'no record with friendlyName ..' means no candidate has the variable."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local want='" + w['anim_var'] + "' local rec,fn,seen=nil,nil,'' "
            "for _,c in ipairs(" + _lua_list(w['records']) + ") do local r=TweakDBInterface.GetItemRecord(TweakDBID.new(c)) "
            "if r then local f=tostring(r:FriendlyName()) seen=seen..c..'='..f..',' if f==want and not rec then rec,fn=c,f end end end "
            "if not rec then error('no record with friendlyName '..want..' seen '..seen) end "
            "return 'rec='..rec..' fn='..fn..' n='..#ids(TweakDBID.new(rec)) end) "
            "Game.UTPPLogLine('[night] MW_COUNT ok='..tostring(ok)..' " + w['tag'] + " '..tostring(e))")


def give_cmd(w: dict, rec: str) -> str:
    """Give the record when the inventory has none (a copy the player owns is drawn as it is) and queue the draw request for
    the item. The inventory is counted inside the command, so a repeated send gives nothing twice.
    Echo `[night] MW_GIVE ok=.. <tag> rec=<record> given|had n=<before>-><after>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local tdb=TweakDBID.new('" + rec + "') local have=ids(tdb) local n0=#have local how='had' "
            "if n0==0 then ts:GiveItem(p,ItemID.FromTDBID(tdb),1) how='given' have=ids(tdb) end "
            "if #have==0 then error('item not in the inventory after the give') end "
            "local r=DrawItemRequest.new() r.owner=p r.itemID=have[1] "
            "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) "
            "return 'rec=" + rec + " '..how..' n='..n0..'->'..#have end) "
            "Game.UTPPLogLine('[night] MW_GIVE ok='..tostring(ok)..' " + w['tag'] + " '..tostring(e))")


def inhand_cmd() -> str:
    """Anim variable (friendlyName) of the item in the weapon slot and the melee status. Echo `[night] MW_INHAND ok=.. <name|none|?> | <status>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() "
            "local w=Game.GetTransactionSystem():GetItemInSlot(p,TweakDBID.new('AttachmentSlots.WeaponRight')) "
            "if not w then return 'none' end local rec=TweakDBInterface.GetItemRecord(w:GetItemID().id) if not rec then return '?' end "
            "return tostring(rec:FriendlyName()) end) local st='' pcall(function() st=tostring(Game.UTPPMeleeStatus()) end) "
            "Game.UTPPLogLine('[night] MW_INHAND ok='..tostring(ok)..' '..tostring(e)..' | '..st)")


def holster_cmd() -> str:
    """UnequipWeapon request. Echo `[night] MW_HOLSTER ok=..`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local r=EquipmentSystemWeaponManipulationRequest.new() "
            "r.requestType=Enum.new('EquipmentManipulationAction','UnequipWeapon') r.owner=p "
            "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'eqsystem' end) "
            "Game.UTPPLogLine('[night] MW_HOLSTER ok='..tostring(ok)..' '..tostring(e))")


def remove_cmd(rec: str, keep: int = 0) -> str:
    """Remove the given copies of a record: items are taken out until only `keep` (the count before the give) are left.
    Echo `[night] MW_REMOVE ok=.. removed <n0>-><n1> | nothing to remove .. | NOT removed <n0>-><n1> ..`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local tdb=TweakDBID.new('" + rec + "') local keep=" + str(int(keep)) + " local n0=#ids(tdb) "
            "if n0<=keep then return 'nothing to remove '..n0..' keep '..keep end local r1='-' local l=ids(tdb) "
            "for i=#l,keep+1,-1 do r1=tostring(ts:RemoveItem(p,l[i],1)) end local n1=#ids(tdb) local r2='-' "
            "if n1>keep then r2=tostring(pcall(function() return ts:RemoveItemByTDBID(p,tdb,n1-keep) end)) n1=#ids(tdb) end "
            "if n1<=keep then return 'removed '..n0..'->'..n1..' r='..r1 end "
            "return 'NOT removed '..n0..'->'..n1..' r='..r1..'/'..r2 end) "
            "Game.UTPPLogLine('[night] MW_REMOVE ok='..tostring(ok)..' '..tostring(e))")


def cyberware_equip_cmd(w: dict) -> str:
    """Arm cyberware test, step 1: give the item (when missing), remember the arms item that is installed (TweakDBID hash, 0 =
    none) and queue an EquipRequest for the arms slot. Echo `[night] MW_CW_EQUIP ok=.. <tag> rec=.. given|had prev=<hash>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local es=Game.GetScriptableSystemsContainer():Get('EquipmentSystem') local area=Enum.new('gamedataEquipmentArea','ArmsCW') "
            "local prev=es:GetItemInEquipSlot(p,area,0) local ph=0 if prev and ItemID.IsValid(prev) then ph=ItemID.GetTDBID(prev).hash end "
            "local rec='" + w['records'][0] + "' local tdb=TweakDBID.new(rec) local have=ids(tdb) local how='had' "
            "if #have==0 then ts:GiveItem(p,ItemID.FromTDBID(tdb),1) how='given' have=ids(tdb) end "
            "if #have==0 then error('item not in the inventory after the give') end "
            "local r=EquipRequest.new() r.owner=p r.itemID=have[1] r.slotIndex=0 es:QueueRequest(r) "
            "return 'rec='..rec..' '..how..' prev='..tostring(ph) end) "
            "Game.UTPPLogLine('[night] MW_CW_EQUIP ok='..tostring(ok)..' " + w['tag'] + " '..tostring(e))")


def cyberware_draw_cmd() -> str:
    """Arm cyberware test, step 2: draw the installed arms weapon. Echo `[night] MW_CW_DRAW ok=..`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local r=EquipmentSystemWeaponManipulationRequest.new() "
            "r.requestType=Enum.new('EquipmentManipulationAction','RequestActiveMeleeware') r.owner=p "
            "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) return 'eqsystem' end) "
            "Game.UTPPLogLine('[night] MW_CW_DRAW ok='..tostring(ok)..' '..tostring(e))")


def cyberware_restore_cmd(w: dict, prev_hash: int, given: bool) -> str:
    """Arm cyberware test, last step: put the previous arms item back (or empty the slot when there was none) and take the
    given item out of the inventory. Echo `[night] MW_CW_RESTORE ok=.. <tag> restored=<n> removed=<n>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local es=Game.GetScriptableSystemsContainer():Get('EquipmentSystem') local prevh=" + str(int(prev_hash))
            + " local restored=0 if prevh~=0 then local a,b=ts:GetItemList(p) local l=type(b)=='table' and b or a "
            "for _,it in ipairs(l) do if ItemID.GetTDBID(it:GetID()).hash==prevh and restored==0 then local r=EquipRequest.new() "
            "r.owner=p r.itemID=it:GetID() r.slotIndex=0 es:QueueRequest(r) restored=1 end end else "
            "local u=UnequipRequest.new() u.owner=p u.areaType=Enum.new('gamedataEquipmentArea','ArmsCW') u.slotIndex=0 es:QueueRequest(u) end "
            "local removed=0 if " + ('true' if given else 'false') + " then local tdb=TweakDBID.new('" + w['records'][0] + "') "
            "for _,id in ipairs(ids(tdb)) do ts:RemoveItem(p,id,1) removed=removed+1 end end "
            "return 'restored='..restored..' removed='..removed end) "
            "Game.UTPPLogLine('[night] MW_CW_RESTORE ok='..tostring(ok)..' " + w['tag'] + " '..tostring(e))")


def echo_of(text: str | None, tag: str) -> tuple[bool | None, str]:
    """(ok, info) of the last '[night] <tag> ok=<true|false> <info>' line; (None, '') = no echo (command lost)."""
    found = ''
    for line in (text or '').splitlines():
        if f'[night] {tag} ok=' in line:
            found = line
    if not found:
        return None, ''
    info = found.split(f'[night] {tag} ok=', 1)[1]
    return info.startswith('true'), info.split(' ', 1)[1].strip() if ' ' in info else ''


_COUNT_RE = re.compile(r'rec=(\S+) fn=(\S+) n=(\d+)')
_GIVE_RE = re.compile(r'rec=(\S+) (given|had) n=(\d+)->(\d+)')
_GIVE_RE_CW = re.compile(r'rec=(\S+) (given|had) prev=(\d+)')


def parse_count(info: str) -> dict | None:
    m = _COUNT_RE.search(info or '')
    return {'rec': m.group(1), 'fn': m.group(2), 'n': int(m.group(3))} if m else None


def parse_give(info: str) -> dict | None:
    m = _GIVE_RE.search(info or '')
    if m:
        return {'rec': m.group(1), 'how': m.group(2), 'n_before': int(m.group(3)), 'n_after': int(m.group(4))}
    m = _GIVE_RE_CW.search(info or '')
    if m:
        return {'rec': m.group(1), 'how': m.group(2), 'n_before': 0 if m.group(2) == 'given' else 1, 'n_after': 1,
                'prev_hash': int(m.group(3))}
    return None


def inhand_name(info: str) -> str:
    """Anim variable from an MW_INHAND echo info ('w_melee_katana | meleefull ...' -> 'w_melee_katana')."""
    return (info or '').split('|', 1)[0].strip().split(' ')[0] if info else ''


def parse_remove(info: str) -> str:
    """'removed' | 'nothing' | 'failed' from an MW_REMOVE echo info."""
    if (info or '').startswith('removed'):
        return 'removed'
    if (info or '').startswith('nothing to remove'):
        return 'nothing'
    return 'failed'


# ---------------------------------------------------------------------------------------------- log evidence

_TS_RE = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3})\]')
_MASK_RE = re.compile(r'UTPP MMMASK (on|off) reasons=0x([0-9A-Fa-f]+) \(was 0x([0-9A-Fa-f]+)\).*?\bfullWhy=(\w+)')
_FULL_RE = re.compile(r'UTPP MMMELEEFULL (on|off) why=(\w+) full=(-?[\d.]+) speed=(-?[\d.]+) loco=(-?\d+) rawLoco=(-?\d+) '
                      r'detail=(-?\d+).*?\bstate=(-?\d+) states=0x([0-9A-Fa-f]+)')
_AIM_RE = re.compile(r'UTPP MELEEAIM src=(\w+) tiltW=(-?[\d.]+) tgtPitch=(-?[\d.]+) tgtYaw=(-?[\d.]+) rawPitch=(-?[\d.]+) '
                     r'rawYaw=(-?[\d.]+) pitch=(-?[\d.]+) yaw=(-?[\d.]+) flags=0x([0-9A-Fa-f]+)')
_BASE_RE = re.compile(r'\| BASE body=(-?\d+) clip=(\S+) f=(-?\d+).*? raw=(-?\d+)/(-?\d+)/(-?\d+).*? applied=(\d+) ')
_TPP_RE = re.compile(r'utpp_melee_tpp=(\d)')


def _span(values):
    return [min(values), max(values)] if values else None


def parse_evidence(text: str) -> dict:
    """Mask / full-body / aim / wrapper-variable evidence of a UTPP log window. Pure."""
    mask, full, aim, tpp = [], [], [], []
    base_applied = []
    for line in (text or '').splitlines():
        tm = _TS_RE.match(line)
        ts = tm.group(1)[11:] if tm else None
        m = _MASK_RE.search(line)
        if m:
            mask.append({'ts': ts, 'on': m.group(1) == 'on', 'reasons': int(m.group(2), 16), 'full_why': m.group(4)})
            continue
        m = _FULL_RE.search(line)
        if m:
            full.append({'ts': ts, 'on': m.group(1) == 'on', 'why': m.group(2), 'speed': float(m.group(4)),
                         'loco': int(m.group(5)), 'detail': int(m.group(7)), 'state': int(m.group(8)),
                         'states': int(m.group(9), 16)})
            continue
        m = _AIM_RE.search(line)
        if m:
            row = {'ts': ts, 'src': m.group(1), 'tgt_yaw': float(m.group(4)), 'tgt_pitch': float(m.group(3)),
                   'yaw': float(m.group(8)), 'pitch': float(m.group(7)), 'tilt_w': float(m.group(2))}
            b = _BASE_RE.search(line)
            if b:
                row.update(base_clip=b.group(2), base_raw=[int(b.group(4)), int(b.group(5)), int(b.group(6))],
                           base_applied=int(b.group(7)))
                base_applied.append(int(b.group(7)))
            aim.append(row)
            continue
        m = _TPP_RE.search(line)
        if m and 'utpp_melee_tpp' in line:
            tpp.append(int(m.group(1)))
    src: dict = {}
    for a in aim:
        src[a['src']] = src.get(a['src'], 0) + 1
    clips: list = []
    for a in aim:
        c = a.get('base_clip')
        if c and c != '-' and c not in clips:
            clips.append(c)
    return {
        'mask_events': mask, 'full_events': full,
        'mask_on': any(e['on'] and e['reasons'] & 1 for e in mask),
        'full_on': sum(1 for e in full if e['on']), 'full_off': sum(1 for e in full if not e['on']),
        'last_full_on': (full[-1]['on'] if full else None),
        'last_why': (full[-1]['why'] if full else (mask[-1]['full_why'] if mask else None)),
        'mask_why': sorted({e['full_why'] for e in mask}),
        'psm_states': sorted({e['state'] for e in full}),
        'dll_states': sorted({e['states'] for e in full}),
        'off_reasons': sorted({e['why'] for e in full if not e['on']}),
        'melee_tpp_last': (tpp[-1] if tpp else None),
        'aim': {'n': len(aim), 'src': src, 'tgt_yaw': _span([a['tgt_yaw'] for a in aim]),
                'yaw': _span([a['yaw'] for a in aim]), 'pitch': _span([a['pitch'] for a in aim]),
                'base_clips': clips,
                'base_raw_yaw': _span([a['base_raw'][0] for a in aim if 'base_raw' in a]),
                'base_applied_delta': (base_applied[-1] - base_applied[0]) if len(base_applied) > 1 else 0},
    }


def state_names(states) -> list[str]:
    return [f'{s}:{STATE_NAMES.get(s, "?")}' for s in states]


# ---------------------------------------------------------------------------------------------- phase table
# expect_full: 'on' = the full body must switch on at least once in the phase (standing strikes), 'off' = it is expected
# to stay off (stick held, sprint, crouch), None = nothing expected. weight_warn: an MM weight below the melee threshold
# while masked is a warning instead of a failure (stances outside the standing fists check).
PHASES = (
    {'name': 'light_combo', 'part': 'mw_light_combo', 'kind': 'pad', 'strike': True, 'expect_full': 'on'},
    {'name': 'guard_idle', 'part': 'mw_guard_idle', 'kind': 'pad', 'strike': False, 'expect_full': None},
    {'name': 'strong_attack', 'part': 'mw_strong_attack', 'kind': 'pad', 'strike': True, 'expect_full': 'on'},
    {'name': 'block', 'part': 'melee_block', 'kind': 'pad', 'strike': False, 'expect_full': 'block', 'needs_block': True},
    {'name': 'walk_strikes', 'part': 'melee_punch_walk', 'kind': 'walk', 'strike': True, 'expect_full': 'off'},
    {'name': 'sprint_attack', 'part': 'mw_sprint_attack', 'kind': 'walk', 'strike': True, 'expect_full': 'off',
     'weight_warn': True},
    {'name': 'crouch_attack', 'part': 'mw_crouch_attack', 'kind': 'pad', 'strike': True, 'expect_full': 'off',
     'weight_warn': True, 'crouch': True, 'scripted': True},
)


def weapon_phases(w: dict) -> list[dict]:
    """The strike phases a weapon runs, in order: no block phase without a block set; a 'phases' list keeps only those names."""
    only = w.get('phases')
    return [p for p in PHASES if not (p.get('needs_block') and not w['block']) and (only is None or p['name'] in only)]


def weapon_covered(w: dict, tables: dict | None) -> bool | None:
    """Coverage of a weapon: its own 'covered' entry when it has one, else the script's full-body list."""
    return w['covered'] if 'covered' in w else is_covered(tables, w['anim_var'])


def weapon_states(w: dict, tables: dict | None) -> int | None:
    """State mask the script sends for a weapon: the default mask for 'states_default' entries, else the table entry."""
    if w.get('states_default'):
        return tables['default_states'] if tables else None
    return expected_states(tables, w['anim_var'])


def phase_plan(tag: str) -> list[str]:
    """Names of all phases in order for a weapon (equip .. remove)."""
    w = WEAPONS[tag]
    mid = [p['name'] for p in weapon_phases(w)]
    return ['equip', 'draw'] + mid + ['holster', 'remove']


def evaluate_phase(w: dict, phase: dict, ev: dict, ms: dict, covered: bool | None, exp_states: int | None,
                   carried_full: bool, carried_mask: bool) -> dict:
    """Judge one strike phase from its evidence. Returns {'fails', 'warns', 'own_clip'}; `own_clip` = signs that the game
    played its own clip. `ms` = gt_night.melee_state of the window, carried_* = state at the window start."""
    fails, warns, signs = [], [], []
    name, expect = phase['name'], phase['expect_full']
    melee_seen = ev['mask_on'] or carried_mask or ms.get('on')
    full_seen = ev['full_on'] > 0 or carried_full
    if phase['strike'] and not melee_seen:
        soft = phase.get('weight_warn') or covered is False
        (warns if soft else fails).append(f'{name}: no melee mask line (the strikes did not start or the mask was not applied)')
    if ms.get('bad'):
        ts, wv = min(ms['bad'], key=lambda b: b[1])
        msg = f'{name}: MM weight {wv:.2f} < 0.9 while the melee mask was active ({ts}; {len(ms["bad"])} samples)'
        (warns if phase.get('weight_warn') else fails).append(msg)
    if covered:
        for dll in ev['dll_states']:
            if exp_states is not None and dll != exp_states:
                signs.append(f'DLL state mask 0x{dll:X} differs from the script table 0x{exp_states:X}')
        if ev['melee_tpp_last'] == 0:
            signs.append('utpp_melee_tpp=0 in the window (the third-person clips are not selected)')
        if expect == 'on' and melee_seen and not full_seen:
            signs.append(f'melee strikes without the full body (why={ev["last_why"]}, PSM states {state_names(ev["psm_states"])})')
            if exp_states is not None:
                refused = sorted({e['state'] for e in ev['full_events']
                                  if not e['on'] and e['why'] == 'state' and e['state'] >= 5 and exp_states >> e['state'] & 1})
                if refused:
                    signs.append(f'state {state_names(refused)} is in the script table but the DLL refused it')
        if expect == 'on' and not full_seen:
            fails.append(f'{name}: full body never switched on (MMMELEEFULL) although the weapon is covered')
        if expect == 'block' and exp_states is not None and exp_states & BLOCK_STATE_BIT and not full_seen:
            fails.append(f'{name}: the block state is in the table but the full body never switched on')
        if expect == 'off' and ev['full_on'] and not ev['full_off']:
            warns.append(f'{name}: full body stayed on while the phase moves V (expected off: speed / stance)')
    elif covered is False:
        if ev['full_on']:
            fails.append(f'{name}: full body switched on for a weapon the script does not cover')
        if expect == 'on' and ev['last_why'] not in (None, 'flag'):
            warns.append(f'{name}: last full-body reason {ev["last_why"]} (control weapon expects flag)')
    else:
        warns.append(f'{name}: script tables not readable, coverage not judged')
    if w['tag'] != 'fists' and ev['aim']['base_applied_delta'] > 0:
        warns.append(f'{name}: chest base correction applied {ev["aim"]["base_applied_delta"]} times (its table is built for '
                     'fists; clips named in the BASE field are fists clips)')
    return {'fails': fails, 'warns': warns, 'own_clip': signs}


# ---------------------------------------------------------------------------------------------- macro

def _gn():
    import gt_night
    return gt_night


def _stamp(dt: datetime) -> str:
    """Local date and time to the millisecond, the clock of the UTPP log."""
    return dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


class _Run:
    """State of one weapon run (log window bookkeeping, results)."""

    def __init__(self, night, pad, guard, plan, w: dict, tables: dict | None):
        self.night, self.pad, self.guard, self.plan, self.w, self.tables = night, pad, guard, plan, w, tables
        self.covered = weapon_covered(w, tables)
        self.exp_states = weapon_states(w, tables)
        self.mask_active = False
        self.full_active = False
        self.phases: list = []
        self.windows: list = []
        self.fails: list = []
        self.warns: list = []
        self.own_clip: list = []
        self.given: dict | None = None
        self.removed = None
        # the lists are shared, so the result stays readable when an exception ends the run early
        self.view = {'weapon': w['tag'], 'label': w['label'], 'verdict': None, 'skip_reason': None, 'anim_var': w['anim_var'],
                     'covered': self.covered, 'expected_states': self.exp_states, 'given': None, 'removed': None,
                     'fails': self.fails, 'warnings': self.warns, 'own_clip_phases': self.own_clip, 'phases': self.phases,
                     'phase_windows': self.windows}

    def say(self, msg: str) -> None:
        self.night.log(f'[weapons] {self.w["tag"]}: {msg}')

    def cmd(self, tag: str, cmd: str, label: str):
        text = self.night.console_verified(self.pad, self.guard, cmd, lambda t: f'[night] {tag} ok=' in t,
                                           f'{self.w["tag"]}/{label}')
        ok, info = echo_of(text, tag)
        self.say(f'{label}: ' + (f'ok={ok} {info[:140]}' if ok is not None else 'echo LOST'))
        return ok, info

    def record(self, name: str, off: int, t0: datetime, extra: dict | None = None, judge: dict | None = None) -> dict:
        """Close a phase: read its log window, record evidence and the verdict of a strike phase."""
        self.guard.wait(0.5)                               # log flush
        t1 = datetime.now()
        text = read_since(self.night.utpp_log, off)
        gn = _gn()
        carried_mask, carried_full = self.mask_active, self.full_active
        ms = gn.melee_state([l for l in text.splitlines() if '[night] POS' in l], text, carried_mask)
        ev = parse_evidence(text)
        self.mask_active = ms['active_now']
        if ev['last_full_on'] is not None:
            self.full_active = bool(ev['last_full_on'])
        rec = {'phase': name, 'start': _stamp(t0), 'end': _stamp(t1), 'mask_on': ms['on'], 'mask_off': ms['off'],
               'mask_active_end': ms['active_now'], 'min_w': ms['min_w'], 'samples': ms['samples'],
               'full_on': ev['full_on'], 'full_off': ev['full_off'], 'full_active_end': self.full_active,
               'off_reasons': ev['off_reasons'], 'psm_states': state_names(ev['psm_states']),
               'dll_states': [f'0x{s:X}' for s in ev['dll_states']], 'mask_why': ev['mask_why'],
               'melee_tpp_last': ev['melee_tpp_last'], 'aim': ev['aim']}
        if extra:
            rec.update(extra)
        if judge:
            r = evaluate_phase(self.w, judge, ev, ms, self.covered, self.exp_states, carried_full, carried_mask)
            rec.update(fails=r['fails'], warns=r['warns'], own_clip=r['own_clip'])
            self.fails += r['fails']
            self.warns += r['warns']
            if r['own_clip']:
                self.own_clip.append({'phase': name, 'signs': r['own_clip']})
        self.phases.append(rec)
        self.windows.append({'weapon': self.w['tag'], 'phase': name, 'start': _stamp(t0), 'end': _stamp(t1)})
        self.say(f'{name}: {_stamp(t0)}..{_stamp(t1)} mask_on={ms["on"]} full on/off={ev["full_on"]}/{ev["full_off"]} '
                 f'states={rec["psm_states"]} why={ev["last_why"]} own_clip={len(rec.get("own_clip", []))}')
        return rec


def _in_hand(run: _Run, label: str = 'inhand'):
    """(name | None, info): the anim variable of the item in the hand, read by script."""
    ok, info = run.cmd('MW_INHAND', inhand_cmd(), label)
    if ok is not True:
        return None, info
    return inhand_name(info), info


def _equip(run: _Run) -> str | None:
    """Give + draw and read the hand back. Returns None when the weapon is in the hand, else the reason it is not testable
    ('skip: ...' = not a mod failure, 'fail: ...' = harness / command failure)."""
    w = run.w
    t0, off = datetime.now(), file_size(run.night.utpp_log)
    if w['kind'] == 'cyberware':
        ok, info = run.cmd('MW_CW_EQUIP', cyberware_equip_cmd(w), 'equip')
        if ok is not True:
            return 'fail: arms cyberware equip ' + ('failed ' + info if ok is False else 'echo lost')
        run.given = parse_give(info)
        if run.given is None:
            return 'fail: arms cyberware equip echo not understood, the loadout cannot be restored'
        run.record('equip', off, t0, {'echo': info})
        t0, off = datetime.now(), file_size(run.night.utpp_log)
        ok, info = run.cmd('MW_CW_DRAW', cyberware_draw_cmd(), 'draw')
        if ok is not True:
            return 'fail: arms cyberware draw ' + ('failed ' + info if ok is False else 'echo lost')
    else:
        ok, info = run.cmd('MW_COUNT', count_cmd(w), 'count')
        if ok is False and 'no record with friendlyName' in info:
            return f'skip: no candidate record has the anim variable {w["anim_var"]} ({info[:200]})'
        counted = parse_count(info) if ok else None
        if not counted:
            return 'fail: the inventory count failed, nothing was given' + (f' ({info[:160]})' if info else ' (no echo)')
        # the cleanup is armed from the count: it takes out only what is added on top of the copies the player owned
        run.given = {'rec': counted['rec'], 'how': 'had' if counted['n'] > 0 else 'given', 'n_before': counted['n']}
        ok, info = run.cmd('MW_GIVE', give_cmd(w, counted['rec']), 'equip')
        run.record('equip', off, t0, {'echo': info, 'record': counted['rec'], 'how': run.given['how']})
        if not (ok and parse_give(info)):
            return 'fail: the give command failed' + (f' ({info[:160]})' if info else ' (no echo)')
        t0, off = datetime.now(), file_size(run.night.utpp_log)
    name = None
    info = ''
    for attempt in range(3):
        name, info = _in_hand(run, 'inhand')
        if name not in (None, 'none', '?'):
            break
        run.guard.wait(1.0)
    run.record('draw', off, t0, {'in_hand': name, 'echo': info[:300]})
    if name in (None, 'none', '?'):
        return 'fail: nothing in the weapon slot after the draw request'
    if name != w['anim_var']:
        return f'skip: the hand holds {name}, expected {w["anim_var"]} (wrong anim variable, not tested)'
    return None


def _ensure_in_hand(run: _Run, where: str) -> bool:
    name, _ = _in_hand(run, f'inhand-{where}')
    if name == run.w['anim_var']:
        return True
    run.warns.append(f'{where}: the weapon was not in the hand (hand: {name}); drawn again by script')
    if run.w['kind'] == 'cyberware':
        run.cmd('MW_CW_DRAW', cyberware_draw_cmd(), f'redraw-{where}')
    else:
        run.cmd('MW_GIVE', give_cmd(run.w, run.given['rec']), f'redraw-{where}')
    run.guard.wait(1.0)
    name, _ = _in_hand(run, f'inhand-{where}-2')
    return name == run.w['anim_var']


def _crouch_state(text: str) -> str | None:
    """Last stance ('crouched' / 'standing') of the MMTRACE lines in a window (None = no trace lines)."""
    import gt_log
    gn = _gn()
    stances = [s for s in (gn.stance_of(l) for l in gt_log.parse_text(text)) if s]
    return stances[-1] if stances else None


def _strike_phase(run: _Run, phase: dict) -> None:
    night, pad, guard, plan = run.night, run.pad, run.guard, run.plan
    gn = _gn()
    if phase['name'] in ('sprint_attack', 'crouch_attack') and not _ensure_in_hand(run, phase['name']):
        run.fails.append(f'{phase["name"]}: the weapon could not be put back in the hand; phase skipped')
        return
    t0 = datetime.now()
    if phase['kind'] == 'walk':
        walked = night.play_walking(pad, guard, plan, phase['part'],
                                    clear=gn.commands_movement(night.scenarios[phase['part']]))
        off = walked['off']
    else:
        off = file_size(night.utpp_log)
        night.play(pad, guard, phase['part'], plan, scripted_scenario(phase['part']) if phase.get('scripted') else None)
    rec = run.record(phase['name'], off, t0, judge=phase)
    if phase.get('crouch'):
        text = read_since(night.utpp_log, off)
        stance = _crouch_state(text)
        rec['stance_end'] = stance
        if stance == 'crouched':
            run.warns.append('crouch_attack: V is still crouched after the phase; one more R3 tap')
            night.play(pad, guard, 'mw_crouch_toggle', plan, scripted_scenario('mw_crouch_toggle'))
        elif stance is None:
            run.warns.append('crouch_attack: no MMTRACE stance lines, the crouch was not verified')


def _holster(run: _Run, final: bool = False) -> None:
    night, pad, guard, plan = run.night, run.pad, run.guard, run.plan
    gn = _gn()
    t0, off = datetime.now(), file_size(night.utpp_log)
    ok, info = run.cmd('MW_HOLSTER', holster_cmd(), 'holster')
    cleared = False
    t_end = time.monotonic() + gn.MELEE_OFF_WINDOW_S
    while True:
        guard.wait(0.5)
        text = read_since(night.utpp_log, off)
        ms = gn.melee_state([], text, run.mask_active)
        cleared = not ms['active_now']
        if cleared or time.monotonic() >= t_end:
            break
    used_y = False
    if not cleared and not final:
        run.say('melee still active after the unequip request -> Y double tap (melee_holster)')
        used_y = True
        night.play(pad, guard, 'melee_holster', plan)
        guard.wait(0.5)
        ms = gn.melee_state([], read_since(night.utpp_log, off), run.mask_active)
        cleared = not ms['active_now']
    run.record('holster', off, t0, {'echo': info[:160], 'mask_cleared': cleared, 'y_double_tap': used_y})
    if ok is not True:
        run.warns.append('holster: unequip request ' + ('failed' if ok is False else 'echo lost'))
    if not cleared:
        run.fails.append(f'holster: melee mask still active {gn.MELEE_OFF_WINDOW_S:.0f} s after the unequip request'
                         + ('' if final else ' and the Y double tap'))


def _wait_hand_empty(run: _Run) -> bool:
    """Poll the weapon slot (up to 3 reads, 0.8 s apart) until the unequip request has emptied it; an item that is still in
    the hand cannot be taken out of the inventory. A lost / failed read does not stall the cleanup."""
    for _ in range(3):
        name, _info = _in_hand(run, 'inhand-after-holster')
        if name in (None, 'none'):
            return True
        run.guard.wait(0.8)
    run.warns.append('remove: the weapon was still in the hand 2.4 s after the unequip request')
    return False


def _remove(run: _Run) -> None:
    g, w = run.given, run.w
    if not g:
        run.removed = 'not given'
        return
    _wait_hand_empty(run)
    t0, off = datetime.now(), file_size(run.night.utpp_log)
    if w['kind'] == 'cyberware':
        ok, info = run.cmd('MW_CW_RESTORE', cyberware_restore_cmd(w, g.get('prev_hash', 0), g['how'] == 'given'), 'restore')
        state = 'removed' if ok else 'failed'
    elif g['n_before'] > 0:
        run.removed = 'kept (the player owned it)'
        run.say('the player owned the item: kept, nothing removed')
        run.record('remove', off, t0, {'result': run.removed})
        return
    else:
        ok, info = run.cmd('MW_REMOVE', remove_cmd(g['rec'], g['n_before']), 'remove')
        state = parse_remove(info) if ok else 'failed'
    run.removed = state
    run.record('remove', off, t0, {'result': state, 'echo': info[:160]})
    if state == 'failed':
        run.fails.append(f'remove: the given item {g["rec"]} may still be in the inventory ({info[:120] or "no echo"})')
    elif state == 'nothing':
        run.warns.append('remove: nothing to remove although the item was given (it was removed earlier?)')


def run_one(night, pad, guard, plan, tag: str, tables: dict | None, out: dict | None = None) -> dict:
    """One weapon, all phases. Always holsters and removes in a finally. `out['weapons'][tag]` is the live result."""
    w = WEAPONS[tag]
    run = _Run(night, pad, guard, plan, w, tables)
    if out is not None:
        out['weapons'][tag] = run.view
    reason = None
    skip = False
    failed = False
    try:
        if w['kind'] == 'cyberware' and os.environ.get(ALLOW_CYBERWARE_ENV) != '1':
            reason, skip = f'skip: arm cyberware changes the cyberware loadout; set {ALLOW_CYBERWARE_ENV}=1 to run it', True
        else:
            if w.get('pose') and getattr(getattr(night, 'args', None), 'diag', 'keep') == 'off':
                run.warns.append('pose recording wanted for this weapon but the diagnostic logs are off (--diag off): '
                                 'the body shape cannot be measured')
            reason = _equip(run)
            skip = bool(reason and reason.startswith('skip'))
        if reason is None:
            for phase in weapon_phases(w):
                _strike_phase(run, phase)
            _holster(run)
    except BaseException as e:      # noqa: BLE001 - the cleanup below must run for every error, then it is re-raised
        failed = True
        run.fails.append(f'{type(e).__name__}: {str(e)[:160]}')
        raise
    finally:
        if run.given:
            if failed:
                night.scenario_deadline = None     # an expired scenario deadline would stop the cleanup
            try:
                if not any(p['phase'] == 'holster' for p in run.phases):
                    _holster(run, final=True)
                _remove(run)
            except Exception as e:  # noqa: BLE001
                run.fails.append(f'cleanup: {type(e).__name__}: {str(e)[:120]}')
            ledger = getattr(night, 'settings', None)          # listed in the run record: what this weapon changed and its state
            if ledger is not None:
                what = 'cyberware.loadout' if w['kind'] == 'cyberware' else f'inventory.{tag}'
                ent = ledger.register(what, 'as before', 'given', 'melee_weapons', volatile=True)
                if run.removed in ('removed', 'nothing'):
                    ledger.mark_restored(ent)
        run.view.update(given=run.given, removed=run.removed)
    if reason and skip:
        run.warns.append(reason)
    elif reason:
        run.fails.append(reason[6:] if reason.startswith('fail: ') else reason)
    run.view.update(verdict='SKIP' if skip else 'FAIL' if run.fails else 'PASS', skip_reason=reason if skip else None)
    return run.view


def run_weapon_macro(night, pad, guard, plan, set_name: str, out: dict | None = None) -> dict:
    """The macro of a weapon set (`melee_knife` .. `melee_weapons`). `out` (the scenario record) is filled while it runs, so
    the windows and evidence of finished weapons survive a timeout; returns `out`."""
    tables = load_script_tables()
    out = {} if out is None else out
    for key, empty in (('weapons', {}), ('fails', []), ('warnings', []), ('phase_windows', []), ('own_clip_phases', [])):
        out.setdefault(key, empty)
    if tables is None:
        out['warnings'].append('script tables not readable: coverage and state masks are not judged')
    for tag in WEAPON_SETS[set_name]:
        r = None
        try:
            r = run_one(night, pad, guard, plan, tag, tables, out)
        finally:
            view = out['weapons'][tag]
            out['fails'] += [f'{tag}: {f}' for f in view['fails']]
            out['warnings'] += [f'{tag}: {x}' for x in view['warnings']]
            out['phase_windows'] += view['phase_windows']
            out['own_clip_phases'] += [dict(p, weapon=tag) for p in view['own_clip_phases']]
        night.log(f'[weapons] {tag}: {r["verdict"]}' + (f': {"; ".join(r["fails"])}' if r['fails'] else '')
                  + (f' ({r["skip_reason"]})' if r['skip_reason'] else ''))
        secure = getattr(night, 'secure_session_files', None)      # a long run: finished recording segments go before the ring drops them
        if secure is not None:
            secure('between weapons')
    verdicts = [r['verdict'] for r in out['weapons'].values()]
    out['verdict'] = 'FAIL' if 'FAIL' in verdicts else 'SKIP' if all(v == 'SKIP' for v in verdicts) else 'PASS'
    return out


# ---------------------------------------------------------------------------------------------- dry run

def dry_run_lines(tag: str, scenarios: dict | None = None) -> list[str]:
    w = WEAPONS[tag]
    tables = load_script_tables()
    cov = weapon_covered(w, tables)
    exp = weapon_states(w, tables)
    lines = [f'== {w["set"]}: {w["label"]} ({tag}), anim variable {w["anim_var"]}, script covers it: {cov}, '
             f'state mask {"0x%X" % exp if exp is not None else "?"} ({exp})',
             f'   records (first with friendlyName == {w["anim_var"]} is used): {", ".join(w["records"])}']
    if w['kind'] == 'cyberware':
        lines += [f'   needs {ALLOW_CYBERWARE_ENV}=1 (changes and restores the cyberware loadout)',
                  f'   equip   : {cyberware_equip_cmd(w)}', f'   draw    : {cyberware_draw_cmd()}']
    if w.get('pose'):
        lines.append('   pose    : needs the pose recording (diagnostic logs on: night.py --diag timed | until) to measure the body')
    else:
        lines += [f'   count   : {count_cmd(w)}',
                  f'   equip   : {give_cmd(w, w["records"][0])}  (record from the count echo; nothing is given when the player owns one)']
    lines.append(f'   in hand : {inhand_cmd()}')
    runs = [p['name'] for p in weapon_phases(w)]
    for p in PHASES:
        if p.get('needs_block') and not w['block']:
            lines.append(f'   {p["name"]:14}: skipped (no block set)')
            continue
        if p['name'] not in runs:
            lines.append(f'   {p["name"]:14}: not run for this weapon')
            continue
        dur = ''
        sc = scenarios.get(p['part']) if scenarios else None
        if sc is None and p.get('scripted'):
            sc = scripted_scenario(p['part'])
        if sc is not None:
            dur = f', {sc.duration:.1f} s of pad input'
        expect = 'off (the script does not cover this weapon)' if cov is False else p['expect_full']
        lines.append(f'   {p["name"]:14}: pad part {p["part"]} ({p["kind"]}){dur}; expects full body {expect}')
    lines.append(f'   holster : {holster_cmd()}  (fallback pad part melee_holster)')
    if w['kind'] == 'cyberware':
        lines.append(f'   restore : {cyberware_restore_cmd(w, 0, True)}  (prev hash comes from the equip echo)')
    else:
        lines.append(f'   remove  : {remove_cmd(w["records"][0], 0)}  (record from the equip echo; keep = items owned before)')
    lines.append('   phases  : ' + ' -> '.join(phase_plan(tag)))
    return lines


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dry-run', action='store_true', help='print the commands and pad parts of each weapon')
    ap.add_argument('--weapons', default=','.join(WEAPONS), help='comma list of weapon tags')
    a = ap.parse_args(argv)
    tags = [t.strip() for t in a.weapons.split(',') if t.strip()]
    bad = [t for t in tags if t not in WEAPONS]
    if bad:
        ap.error(f'unknown weapon(s) {bad}; known: {", ".join(WEAPONS)}')
    from gt_scenario import load_scenarios
    scenarios = load_scenarios(SCENARIO_FILE)
    missing = [p for p in PARTS if p not in scenarios]
    for t in tags:
        print('\n'.join(dry_run_lines(t, scenarios)))
    if missing:
        print(f'MISSING pad parts in scenarios.json: {missing}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
