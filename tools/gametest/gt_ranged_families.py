"""Ranged weapon family scenarios for the night runner (night.py): one macro, one weapon per family.

  python gt_ranged_families.py --dry-run [--families handgun,smg] [--records family=Items.X]

Set name: `ranged_families` (not part of 'all'). Options of night.py: `--ranged-families a,b,c` (default: every family) and
`--ranged-records family=Items.X[,Items.Y]` (repeatable; replaces the candidate list of that family).

Sequence per family (script commands and the virtual pad only, so it also runs with --background):
  pick record -> give weapon + ammunition, draw -> confirm the weapon in the hand (log) -> aim idle -> hip fire -> aimed fire ->
  gun bash -> reload (the magazine is no longer full) -> reload while walking -> reload, then run (before the reload ends) ->
  walk -> walk aimed -> sprint -> crouch + aim -> holster -> take out what was given.
Every phase has its own label `rf_<family>_<phase>` (plan run, pose meter window, contact sheet), so the pose reports and the
sheets are per family and state. The phases measure what the runner already measures (clip changes, foot slip, frame pacing,
pose capture with the helper reports); no limit is added here. Where no shape limit exists the values are only reported.

Gun bash (phase `bash`, flag `bash` of a family, default on): one quick-melee tap with the weapon in the hand, a 1.5 s window, no
pass / fail limit. The button is the quick-melee binding of the Alternative input scheme (LB, r6 input mapping
QuickMelee_Button_Alternative; the harness runs under that scheme, crouch is R3); it is taken from the configuration and not yet
confirmed in play. Whether the game played the move is read from the log: an `MMGATE` line (written with the MM trace on, when the
melee gate changes) whose `melee=` value has the melee-attack bit 0x800, which the plugin sets while PSM.RightWeapon is QuickMelee.
No such line = `not verified` (a warning, never a failure and never "no bash": the line is only written when the gate changes).

Reload while moving (needs the reload diagnostics, whose `RELOAD episode ... dur=` line is read; without it the timing is reported as
not verified): each of the two phases fires a few shots first (the magazine must be short), then plays one pad scenario with the reload
button inside it. The reload button is also the mount button next to a vehicle: V seated before or after the press makes the phase
INVALID (not scored, listed in `invalid_phases`), V leaves the vehicle again and walks away from it.
  reload_walk      walk (stick 0.55), press reload while walking; passes when the magazine was refilled and V moved at least 1 m
                   during the reload.
  reload_then_run  stand, press reload, push the stick fully forward about 0.4 s before the reload is due to end (the duration of
                   the standing reload phase; 1.2 s after the press when it is unknown), keep running until 1 s after the reload
                   episode ended. Passes when the magazine was refilled and the movement began before the episode ended; a reload
                   that had already ended before V moved makes the phase INVALID. The press time, the push time, the episode end
                   and the measured start of the movement are recorded in the phase record.
The game ignores the trigger and the reload button while it is busy with the weapon (the bolt cycle after a sniper shot lasts about
2.5 s, a reload goes on after the magazine count has grown, a quick melee). The weapon state is in the position lines (`psmW=`); the
shots before a moving reload and every reload press wait (up to 4 s) until that state is no longer one of WEAPON_BUSY. Shots that
left the magazine full and the ammunition count unchanged are sent once more (a pull can also only raise a lowered weapon). A
magazine that is full again although rounds were fired belongs to a weapon one pull empties (the double-barrel shotgun): the game
reloads it by itself, so the two phases pull the trigger where they would press the reload button (`reload_by: shot` in the phase
record). The standing reload waits for the end of the reload as well, so its duration is known to the moving phases.
Option `ranged_phases` (night.py: --ranged-phases a,b) runs only the listed phases (equip, draw and the cleanup always run; `reload`
is added when `reload_then_run` is listed, it supplies the duration).

Records: the first candidate the game knows and whose item type belongs to the family is used. A family without such a record is
skipped with a `record not found` line (never a silent pass). Candidates are base game `_Default` presets of the 2.5 weapon
tables, power weapons first (a tech weapon charges and a smart weapon locks on, which changes what a trigger tap does; this
order is an assumption). RECORD_EVIDENCE names the evidence of each record:
  live   given and drawn in a game session of this project
  tweak  present in the 2.5 weapon tables, never given by the harness (unverified in play)
Not in the list: the heavy machine gun (Base_HMG is picked up from the world: tags TakeAndEquip / DiscardOnEmpty /
HideInBackpackUI, drawn with RequestHeavyWeapon, not a weapon that is given and reloaded like the others), the rocket launcher
and the tech sniper rifle of the sniper item type, grenade launchers.

Cleanup (always, also after an error): the weapon is holstered, the given weapon is taken out of the inventory (a copy the
player owned stays), the ammunition is brought back to the count it had, and the weapon that was in the hand at the start is
drawn again. The harness never saves the game.
"""
from __future__ import annotations

import dataclasses
import os
import re
import sys
import time
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gt_log  # noqa: E402
import gt_melee_weapons as gw  # noqa: E402
from gt_input import file_size, read_since  # noqa: E402

SET_NAME = 'ranged_families'
SET_MEMBERS = {SET_NAME: [SET_NAME]}
# static pad parts of scenarios.json: the family copies of them (rf_<family>_<phase>) keep their thresholds
PARTS = ['ranged_walk', 'ranged_walk_aimed', 'ranged_sprint']
MOVING_PARTS = {'walk': 'ranged_walk', 'walk_aimed': 'ranged_walk_aimed', 'sprint': 'ranged_sprint'}
PHASES = ('draw', 'aim_idle', 'hip_fire', 'aim_fire', 'bash', 'reload', 'reload_walk', 'reload_then_run', 'walk', 'walk_aimed',
          'sprint', 'crouch_aim', 'holster')
BASH_BUTTON = 'LB'                    # QuickMelee_Button_Alternative (the harness runs under the Alternative scheme: crouch = R3)
BASH_TAP_S = 0.08                     # a longer LB hold is the scanner (VisionHold times out at 0.1 s), not the quick melee
BASH_WINDOW_S = 1.5                   # length of the phase
BASH_MELEE_BIT = 0x800                # MMGATE melee= bit of a melee attack; the plugin sets it while PSM.RightWeapon is QuickMelee
AMMO_GIVEN = 300                      # ammunition of the weapon's type is raised to this count
SECONDS_PER_FAMILY = 100.0            # time limit per family (the run itself takes about 50-60 s)
RELOAD_MAX_S = 8.0                    # longest wait for the magazine to refill
RELOAD_POLL_S = 0.6
# reload while moving (phases reload_walk, reload_then_run)
RELOAD_MOTION_PHASES = ('reload_walk', 'reload_then_run')
RELOAD_WALK_STICK = 0.55              # reload_walk: left stick while walking and pressing reload
RELOAD_WALK_MIN_M = 1.0               # reload_walk: V must have moved at least this far during the reload
RELOAD_WALK_LEAD_S = 1.0              # reload_walk: walking before the reload press
RELOAD_WALK_TAIL_S = 1.0              # reload_walk: time after the expected end of the reload
RELOAD_PRESS_AT_S = 0.2               # reload_then_run: standing time before the reload press
RELOAD_RUN_LEAD_S = 0.4               # reload_then_run: the stick is pushed this long before the reload is due to end
RELOAD_RUN_FALLBACK_START_S = 1.2     # reload_then_run: movement starts this long after the press when the duration is unknown
RELOAD_RUN_TAIL_S = 1.0               # reload_then_run: running goes on this long after the reload episode ended
RELOAD_UNKNOWN_S = 2.5                # length assumed for the scenario when the reload duration is unknown
RELOAD_MOVING_MIN_M = 0.3             # displacement from where V stood at the push that counts as "moving"
RELOAD_PREP_MAX_SHOTS = 2             # shots fired before a moving reload so that the magazine is short
UNREADABLE_WAIT_S = 3.0               # wait after the reload tap when the magazine cannot be read (only a reload log line could end it)
# weapon states of the position lines (`psmW=`) in which the game ignores the trigger and the reload button
WEAPON_BUSY = {2: 'reloading', 3: 'in a quick melee', 4: 'out of ammunition', 8: 'cycling after a shot'}
WEAPON_READY_WAIT_S = 4.0             # longest wait for the weapon to leave such a state before shots or a reload press
WEAPON_READY_POLL_S = 0.25
WEAPON_STATE_TAIL = 20000             # bytes at the end of the log searched for the newest position line
PROBE_PHASES = ('walk', 'sprint')       # walking phases that look at the ground ahead first (the others reuse the ground just covered)
DRAW_POLLS = 4                        # hand reads (0.6 s apart) before the one re-draw
LOG_PREFIX = '[ranged]'

# item types (gamedataItemType) per family; the assault rifle family holds the Wea_Rifle records of game 2.5
FAMILIES = {
    'handgun': {'label': 'handgun', 'types': ('Wea_Handgun',), 'gap': 0.5,
                'records': ('Items.Preset_Lexington_Default', 'Items.Preset_Liberty_Default', 'Items.Preset_Kenshin_Default'),
                'ammo': 'Ammo.HandgunAmmo'},
    'revolver': {'label': 'revolver', 'types': ('Wea_Revolver',), 'gap': 1.0,
                 'records': ('Items.Preset_Overture_Default', 'Items.Preset_Nova_Default', 'Items.Preset_Quasar_Default'),
                 'ammo': 'Ammo.HandgunAmmo'},
    'smg': {'label': 'submachine gun', 'types': ('Wea_SubmachineGun',), 'gap': 0.5,
            'records': ('Items.Preset_Borg4a_Default', 'Items.Preset_Saratoga_Default', 'Items.Preset_Dian_Default'),
            'ammo': 'Ammo.HandgunAmmo'},
    'assault_rifle': {'label': 'assault rifle', 'types': ('Wea_Rifle', 'Wea_AssaultRifle'), 'gap': 0.5,
                      'records': ('Items.Preset_Ajax_Default', 'Items.Preset_Copperhead_Default', 'Items.Preset_Hercules_Default'),
                      'ammo': 'Ammo.RifleAmmo'},
    'shotgun': {'label': 'shotgun', 'types': ('Wea_Shotgun',), 'gap': 1.0,
                'records': ('Items.Preset_Crusher_Default', 'Items.Preset_Carnage_Default', 'Items.Preset_Tactician_Default'),
                'ammo': 'Ammo.ShotgunAmmo'},
    'double_shotgun': {'label': 'double-barrel shotgun', 'types': ('Wea_ShotgunDual',), 'gap': 1.0,
                       'records': ('Items.Preset_Testera_Default', 'Items.Preset_Igla_Default', 'Items.Preset_Palica_Default'),
                       'ammo': 'Ammo.ShotgunAmmo'},
    'precision_rifle': {'label': 'precision rifle', 'types': ('Wea_PrecisionRifle',), 'gap': 1.0,
                        'records': ('Items.Preset_Sor22_Default', 'Items.Preset_Kolac_Default', 'Items.Preset_Achilles_Default'),
                        'ammo': 'Ammo.SniperRifleAmmo'},
    'sniper_rifle': {'label': 'sniper rifle', 'types': ('Wea_SniperRifle',), 'gap': 1.3,
                     'records': ('Items.Preset_Grad_Default', 'Items.Preset_Osprey_Default', 'Items.Preset_Ashura_Default'),
                     'ammo': 'Ammo.SniperRifleAmmo'},
    'lmg': {'label': 'light machine gun', 'types': ('Wea_LightMachineGun',), 'gap': 0.5,
            'records': ('Items.Preset_Defender_Default', 'Items.Preset_MA70_Default'), 'ammo': 'Ammo.RifleAmmo'},
}
for _tag, _f in FAMILIES.items():
    _f['tag'] = _tag
    _f.setdefault('bash', True)        # False switches the gun bash phase of the family off
DEFAULT_RECORDS = {t: f['records'] for t, f in FAMILIES.items()}

# 'live' = given and drawn in an earlier game session; everything else of the list is only known from the 2.5 weapon tables
RECORD_EVIDENCE = {r: 'live' for r in ('Items.Preset_Borg4a_Default', 'Items.Preset_Dian_Default', 'Items.Preset_Kenshin_Default',
                                       'Items.Preset_Hercules_Default', 'Items.Preset_Defender_Default')}


def record_evidence(rec: str) -> str:
    return RECORD_EVIDENCE.get(rec, 'tweak')


def unverified_records() -> list[str]:
    """Records of the default lists that were never given in a game session of this project."""
    return [r for recs in DEFAULT_RECORDS.values() for r in recs if record_evidence(r) != 'live']


# ---------------------------------------------------------------------------------------------- options

def parse_families(text) -> list[str]:
    """'a,b' -> family tags in the table order (None / '' = every family). ValueError for an unknown tag."""
    names = [t.strip() for t in str(text or '').split(',') if t.strip()]
    bad = [n for n in names if n not in FAMILIES]
    if bad:
        raise ValueError(f'unknown ranged family {bad}; known: {", ".join(FAMILIES)}')
    return [t for t in FAMILIES if not names or t in names]


def parse_records(items) -> dict[str, tuple]:
    """['family=Items.A,Items.B', ...] -> {family: (records)}. ValueError for a bad item."""
    out: dict = {}
    for item in items or []:
        fam, sep, recs = str(item).partition('=')
        fam = fam.strip()
        if not sep or fam not in FAMILIES:
            raise ValueError(f'--ranged-records {item!r}: expected <family>=<record>[,<record>] with a family of: {", ".join(FAMILIES)}')
        names = tuple(r.strip() for r in recs.split(',') if r.strip())
        if not names or any(not re.fullmatch(r'[A-Za-z0-9_]+\.[A-Za-z0-9_.]+', r) for r in names):
            raise ValueError(f'--ranged-records {item!r}: record names look like Items.Preset_Name_Default')
        out[fam] = names
    return out


def options_of(args) -> tuple[list[str], dict]:
    """(selected families, records per family) from the night.py arguments (every option optional)."""
    fams = parse_families(getattr(args, 'ranged_families', None))
    over = parse_records(getattr(args, 'ranged_records', None))
    return fams, {t: over.get(t, DEFAULT_RECORDS[t]) for t in fams}


def parse_phases(text) -> list[str] | None:
    """'a,b' -> phase names in the table order; None / '' = every phase. ValueError for an unknown name."""
    names = [t.strip() for t in str(text or '').split(',') if t.strip()]
    bad = [n for n in names if n not in PHASES]
    if bad:
        raise ValueError(f'unknown ranged phase {bad}; known: {", ".join(PHASES)}')
    return [p for p in PHASES if p in names] if names else None


def phase_filter(args) -> set[str] | None:
    """The phases to run according to the `ranged_phases` option (None = all). The standing reload supplies the duration of
    `reload_then_run`, so it is part of a filter that names that phase."""
    only = parse_phases(getattr(args, 'ranged_phases', None))
    if only is None:
        return None
    wanted = set(only)
    if 'reload_then_run' in wanted:
        wanted.add('reload')
    return wanted


def option_errors(args) -> list[str]:
    try:
        options_of(args)
        parse_phases(getattr(args, 'ranged_phases', None))
    except ValueError as e:
        return [str(e)]
    return []


def time_limit_s(n_families: int) -> float:
    return 60.0 + SECONDS_PER_FAMILY * max(1, n_families)


# ---------------------------------------------------------------------------------------------- script commands
# Every command ends with a '[night] RF_<TAG> ok=<true|false> ...' echo line (Game.UTPPLogLine); each of them can be sent again
# without a second effect, so a lost command is simply re-sent. The echo text stays short: a log line holds 479 characters.

_LUA_IDS = gw._LUA_IDS
_AMMO = ("local function amid() local a=nil pcall(function() a=TweakDBInterface.GetWeaponItemRecord(tdb):Ammo():GetID() end) "
         "if a and a.hash~=0 then return a end return TweakDBID.new('{fallback}') end ")


def _lua_list(items) -> str:
    return '{' + ','.join("'" + i + "'" for i in items) + '}'


def _echo(tag: str, fam: str = '') -> str:
    return "Game.UTPPLogLine('[night] " + tag + " ok='..tostring(ok)..' " + (fam + ' ' if fam else '') + "'..tostring(e))"


def pick_cmd(fam: dict, records) -> str:
    """Read-only: the first candidate the game knows whose item type is one of the family's. Echo
    `[night] RF_PICK ok=.. <family> rec=<record> type=<type> n=<copies in the inventory> ammo=<ammunition of its type in the
    inventory> seen=<short name>=missing|<type>, ..` (every candidate that was checked); an error `no usable record: <seen>` when
    none fits. n and ammo are the baseline the cleanup brings back; they are read here, before anything is given."""
    want = '{' + ','.join(f"['{t}']=true" for t in fam['types']) + '}'
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local want=" + want + " local rec,ty,seen=nil,nil,'' "
            "local function sn(c) return (string.gsub(c,'^Items%.','')) end "
            "for _,c in ipairs(" + _lua_list(records) + ") do local r=TweakDBInterface.GetItemRecord(TweakDBID.new(c)) "
            "if not r then seen=seen..sn(c)..'=missing,' else local t=nil "
            "pcall(function() t=string.match(tostring(r:ItemType():Type()),'Wea_%w+') end) seen=seen..sn(c)..'='..tostring(t)..',' "
            "if t and want[t] and not rec then rec,ty=c,t end end end "
            "if not rec then error('no usable record: '..seen) end local tdb=TweakDBID.new(rec) " + _AMMO.format(fallback=fam['ammo'])
            + "local a0=ts:GetItemQuantity(p,ItemID.FromTDBID(amid())) "
            "return 'rec='..rec..' type='..ty..' n='..#ids(tdb)..' ammo='..a0..' seen='..seen end) "
            + _echo('RF_PICK', fam['tag']))


def give_cmd(fam: dict, rec: str, ammo: int = AMMO_GIVEN) -> str:
    """Give the record when the inventory has none (a copy the player owns is drawn as it is), raise the ammunition of the
    weapon's type to `ammo` and queue the draw request. Counted inside the command, so a repeated send gives nothing twice.
    A scope on the given copy is taken off (a preset can come with one, and aiming through a scope is shown in first person,
    which would hide the body in the aimed phases); a weapon the player owned is left as it is.
    Echo `[night] RF_GIVE ok=.. <family> rec=<record> given|had n=<before>-><after> ammo=<before>-><after> scope=<off|none|kept>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() " + _LUA_IDS
            + "local tdb=TweakDBID.new('" + rec + "') " + _AMMO.format(fallback=fam['ammo'])
            + "local have=ids(tdb) local n0=#have local how='had' "
            "if n0==0 then ts:GiveItem(p,ItemID.FromTDBID(tdb),1) how='given' have=ids(tdb) end "
            "if #have==0 then error('item not in the inventory after the give') end "
            "local sc='kept' if how=='given' then sc='none' local okS,part=pcall(function() "
            "return ts:RemovePart(p,have[1],TweakDBID.new('AttachmentSlots.Scope')) end) "
            "if okS and part and ItemID.IsValid(part) then sc='off' end end "
            "local am=ItemID.FromTDBID(amid()) local a0=ts:GetItemQuantity(p,am) "
            "if a0<" + str(int(ammo)) + " then ts:GiveItem(p,am," + str(int(ammo)) + "-a0) end "
            "local a1=ts:GetItemQuantity(p,am) "
            "local r=DrawItemRequest.new() r.owner=p r.itemID=have[1] "
            "Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) "
            "return 'rec=" + rec + " '..how..' n='..n0..'->'..#have..' ammo='..a0..'->'..a1..' scope='..sc end) "
            + _echo('RF_GIVE', fam['tag']))


def hand_cmd(rec: str | None = None) -> str:
    """Read-only: the item in the right-hand weapon slot. Echo `[night] RF_HAND ok=.. hash=<TweakDBID hash, 0 = empty>
    match=<1 when it is `rec`> mag=<rounds>/<capacity> total=<rounds incl. the inventory>` (mag / total `-` = not readable)."""
    cmp_ = ("h==TweakDBID.new('" + rec + "').hash") if rec else 'false'
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() "
            "local w=Game.GetTransactionSystem():GetItemInSlot(p,TweakDBID.new('AttachmentSlots.WeaponRight')) "
            "if not w then return 'hash=0 match=0 mag=-/- total=-' end "
            "local h=ItemID.GetTDBID(w:GetItemID()).hash local m,c,t='-','-','-' "
            "pcall(function() m=WeaponObject.GetMagazineAmmoCount(w) c=WeaponObject.GetMagazineCapacity(w) t=w:GetTotalAmmoCount() end) "
            "return 'hash='..h..' match='..(" + cmp_ + " and 1 or 0)..' mag='..tostring(m)..'/'..tostring(c)..' total='..tostring(t) end) "
            "Game.UTPPLogLine('[night] RF_HAND ok='..tostring(ok)..' '..tostring(e))")


def holster_cmd() -> str:
    """UnequipWeapon request. Echo `[night] RF_HOLSTER ok=..`."""
    return gw.holster_cmd().replace('MW_HOLSTER', 'RF_HOLSTER')


def remove_cmd(rec: str, keep: int = 0) -> str:
    """Take the given copies out (until `keep`, the count before the give, are left). Echo `[night] RF_REMOVE ok=.. removed ..`."""
    return gw.remove_cmd(rec, keep).replace('MW_REMOVE', 'RF_REMOVE')


def ammo_off_cmd(fam: dict, rec: str, keep: int) -> str:
    """Bring the ammunition of the record's type back to `keep` (the count before the give; nothing is added when fewer are left).
    Echo `[night] RF_AMMO_OFF ok=.. removed <n0>-><n1> | nothing to remove .. | NOT removed ..`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() local ts=Game.GetTransactionSystem() "
            "local tdb=TweakDBID.new('" + rec + "') " + _AMMO.format(fallback=fam['ammo'])
            + "local am=ItemID.FromTDBID(amid()) local keep=" + str(int(keep)) + " local n0=ts:GetItemQuantity(p,am) "
            "if n0<=keep then return 'nothing to remove '..n0..' keep '..keep end ts:RemoveItem(p,am,n0-keep) "
            "local n1=ts:GetItemQuantity(p,am) if n1<=keep then return 'removed '..n0..'->'..n1 end "
            "return 'NOT removed '..n0..'->'..n1 end) " + _echo('RF_AMMO_OFF'))


def redraw_cmd(item_hash: int) -> str:
    """Draw the inventory item with this TweakDBID hash (the weapon that was in the hand before the set). Echo
    `[night] RF_REDRAW ok=.. found=<0|1>`."""
    return ("local ok,e=pcall(function() local p=Game.GetPlayer() " + _LUA_IDS + "local want=" + str(int(item_hash)) + " local f=0 "
            "for _,it in ipairs(lst()) do if f==0 and ItemID.GetTDBID(it:GetID()).hash==want then f=1 local r=DrawItemRequest.new() "
            "r.owner=p r.itemID=it:GetID() Game.GetScriptableSystemsContainer():Get('EquipmentSystem'):QueueRequest(r) end end "
            "return 'found='..f end) Game.UTPPLogLine('[night] RF_REDRAW ok='..tostring(ok)..' '..tostring(e))")


_PICK_RE = re.compile(r'rec=(\S+) type=(\S+) n=(\d+) ammo=(\d+) seen=(\S*)')
_GIVE_RE = re.compile(r'rec=(\S+) (given|had) n=(\d+)->(\d+) ammo=(\d+)->(\d+)')
_HAND_RE = re.compile(r'hash=(\d+) match=(\d) mag=(\S+)/(\S+) total=(\S+)')


def _int_or_none(text: str):
    return int(text) if re.fullmatch(r'\d+', text or '') else None


def parse_pick(info: str) -> dict | None:
    m = _PICK_RE.search(info or '')
    if not m:
        return None
    seen = [tuple(x.split('=', 1)) for x in m.group(5).split(',') if '=' in x]
    return {'rec': m.group(1), 'type': m.group(2), 'n': int(m.group(3)), 'ammo': int(m.group(4)), 'seen': seen}


def candidate_notes(seen, types, chosen: str) -> list[str]:
    """One line per checked candidate that is not the chosen one and cannot be used: an unknown record or an item type outside the
    family (a typo in an override must not vanish behind a later candidate that works)."""
    short = chosen.replace('Items.', '', 1)
    out = []
    for name, status in seen:
        if name == short:
            continue
        if status == 'missing':
            out.append(f'{name}: record not found')
        elif status not in types:
            out.append(f'{name}: item type {status} is not a type of the family ({", ".join(types)})')
    return out


def parse_give(info: str) -> dict | None:
    m = _GIVE_RE.search(info or '')
    if not m:
        return None
    return {'rec': m.group(1), 'how': m.group(2), 'n_before': int(m.group(3)), 'n_after': int(m.group(4)),
            'ammo_before': int(m.group(5)), 'ammo_after': int(m.group(6))}


def parse_hand(info: str) -> dict | None:
    """RF_HAND echo -> {'hash', 'match' (the record in the hand), 'mag', 'cap', 'total'} (numbers or None)."""
    m = _HAND_RE.search(info or '')
    if not m:
        return None
    return {'hash': int(m.group(1)), 'match': m.group(2) == '1', 'mag': _int_or_none(m.group(3)), 'cap': _int_or_none(m.group(4)),
            'total': _int_or_none(m.group(5))}


def skip_text(fam: dict, records, echo_info: str = '') -> str:
    """The skip line of a family without a usable record."""
    seen = re.search(r'no usable record: (.*)', echo_info or '')
    types_seen = bool(seen) and any(f'={t}' in seen.group(1) for t in re.findall(r'Wea_\w+', seen.group(1)))
    if types_seen:
        return (f'skip: no candidate record has an item type of the family ({", ".join(fam["types"])}); tried '
                f'{", ".join(records)} ({seen.group(1).strip(",")[:200]})')
    return f'skip: record not found (tried {", ".join(records)})'


# ---------------------------------------------------------------------------------------------- pure phase logic

def plan_shots(cap: int | None, total: int = 6) -> tuple[int, int]:
    """(hip shots, aimed shots) for a magazine of `cap` rounds: at most `total` in all, split hip first, and one round stays in
    the magazine (the game reloads an empty magazine by itself, which would move the reload out of its phase); a double-barrel
    shotgun fires one hip shot, a magazine of one round is fired once. Unknown capacity = 3 + 3."""
    if not cap or cap <= 0:
        return 3, 3
    n = max(1, min(total, cap - 1 if cap > 1 else 1))
    hip = min(3, (n + 1) // 2)
    return hip, min(3, n - hip)


def fire_result(mag_before: int | None, mag_after: int | None, total_before: int | None = None,
                total_after: int | None = None, planned: int | None = None) -> dict:
    """Rounds fired in a fire phase. A drop of the ammunition total (magazine + inventory) counts the shots: the total only
    drops by shots, while the magazine also rises when the game reloads an empty magazine by itself. When the total did not drop
    (or is unknown) the magazine difference decides. `problem` when the readings are known, shots were planned and nothing was
    fired."""
    if planned == 0:
        return {'shots': 0, 'problem': None, 'note': 'no shot planned (one round stays in the magazine)'}
    if total_before is not None and total_after is not None and total_before > total_after:
        shots = total_before - total_after
        self_reload = mag_before is not None and mag_after is not None and mag_after > mag_before - shots
        return {'shots': shots, 'problem': None, 'note': 'the game reloaded by itself during the phase' if self_reload else ''}
    if mag_before is None or mag_after is None:
        return {'shots': None, 'problem': None, 'note': 'magazine not readable: shots not counted'}
    shots = mag_before - mag_after
    return {'shots': shots, 'problem': 'the magazine did not change: no shot was fired' if shots <= 0 else None, 'note': ''}


def reload_result(mag_start: int | None, mag_end: int | None, cap: int | None, episodes: list, seconds: float) -> dict:
    """Judge the reload from the magazine readings and the reload episodes of the log (the log lines exist only with the reload
    diagnostics on). detected: True / False / None (no way to tell)."""
    grew = mag_start is not None and mag_end is not None and mag_end > mag_start
    full = mag_end is not None and cap is not None and mag_end >= cap
    if episodes or grew:
        via = 'magazine' if grew else 'log'
        return {'detected': True, 'via': via, 'seconds': round(seconds, 1), 'mag_start': mag_start, 'mag_end': mag_end,
                'cap': cap, 'full': full, 'episodes': episodes}
    if mag_start is None or mag_end is None:
        return {'detected': None, 'via': None, 'seconds': round(seconds, 1), 'mag_start': mag_start, 'mag_end': mag_end, 'cap': cap,
                'full': full, 'episodes': episodes}
    return {'detected': False, 'via': None, 'seconds': round(seconds, 1), 'mag_start': mag_start, 'mag_end': mag_end, 'cap': cap,
            'full': full, 'episodes': episodes}


_EPISODE_RE = re.compile(r'UTPPMotion RELOAD episode .*?\bdur=(\d+(?:\.\d+)?)')
_LOG_TS = '%Y-%m-%d %H:%M:%S.%f'


def _line_time(line: str) -> datetime | None:
    m = _gn().LOGTS_RE.match(line)
    if not m:
        return None
    try:
        return datetime.strptime(m.group(1)[:23], _LOG_TS)
    except ValueError:
        return None


def reload_episode_stamps(text: str) -> list[tuple[datetime | None, float]]:
    """[(time the line was written = end of the reload episode, duration in s)] of the `RELOAD episode ... dur=` lines of a log text."""
    out = []
    for line in (text or '').splitlines():
        m = _EPISODE_RE.search(line)
        if m:
            out.append((_line_time(line), float(m.group(1))))
    return out


def pos_records(text: str) -> list[dict]:
    """[{'t': datetime, 'xy': (x, y), 'mounted': True | False | None}] of the `[night] POS` lines of a log text, in file order
    (`m=` is the mounted flag of the position line)."""
    gn = _gn()
    out = []
    flag = {'1': True, '0': False}
    for line in (text or '').splitlines():
        if '[night] POS p=' not in line:
            continue
        t, pm = _line_time(line), gn.POS_RE.search(line)
        if t is not None and pm:
            out.append({'t': t, 'xy': (float(pm.group(1)), float(pm.group(2))), 'mounted': flag.get(pm.group(9))})
    return out


_WEAPON_STATE_RE = re.compile(r'\[night\] POS p=[^\n]*?\bpsmW=(\d+)')


def weapon_state(text: str) -> int | None:
    """Weapon state of the newest `[night] POS` line of a log text (its `psmW=` field); None when no position line carries it."""
    found = None
    for m in _WEAPON_STATE_RE.finditer(text or ''):
        found = m
    return int(found.group(1)) if found else None


def position_at(samples: list[dict], t: datetime):
    """Position (x, y) at time `t`, interpolated between the two samples around it (the nearest sample outside their span);
    None without samples."""
    if not samples:
        return None
    if t <= samples[0]['t']:
        return samples[0]['xy']
    for a, b in zip(samples, samples[1:]):
        if a['t'] <= t <= b['t']:
            span = (b['t'] - a['t']).total_seconds()
            k = (t - a['t']).total_seconds() / span if span > 0 else 0.0
            return (a['xy'][0] + (b['xy'][0] - a['xy'][0]) * k, a['xy'][1] + (b['xy'][1] - a['xy'][1]) * k)
    return samples[-1]['xy']


def moved_between(samples: list[dict], t_a: datetime, t_b: datetime) -> float | None:
    """Horizontal distance (m) V covered between two times, or None when the position lines do not reach into that time span."""
    if len(samples) < 2 or samples[-1]['t'] < t_a or samples[0]['t'] > t_b:
        return None
    pa, pb = position_at(samples, t_a), position_at(samples, t_b)
    return round(((pb[0] - pa[0]) ** 2 + (pb[1] - pa[1]) ** 2) ** 0.5, 2)


def moving_since(samples: list[dict], t: datetime, min_m: float = RELOAD_MOVING_MIN_M) -> datetime | None:
    """Time of the first position line after `t` that lies at least `min_m` from where V stood at `t`; None = not seen."""
    p0 = position_at(samples, t)
    if p0 is None:
        return None
    for s in samples:
        if s['t'] >= t and ((s['xy'][0] - p0[0]) ** 2 + (s['xy'][1] - p0[1]) ** 2) ** 0.5 >= min_m:
            return s['t']
    return None


def reload_prep_shots(cap: int | None) -> int:
    """Shots fired before a moving reload: a few, never the last round of the magazine (an empty magazine is refilled by the game)."""
    hip = plan_shots(cap)[0]
    return max(1, min(RELOAD_PREP_MAX_SHOTS, hip))


def reload_run_timing(reload_s: float | None) -> dict:
    """Seconds from the start of the reload_then_run scenario: the reload press, the push of the stick, the expected end of the
    reload and the length of the scenario. `reload_s` = duration of the standing reload of this weapon (None = unknown: the stick is
    pushed RELOAD_RUN_FALLBACK_START_S after the press and the end is assumed at RELOAD_UNKNOWN_S)."""
    press = RELOAD_PRESS_AT_S
    if reload_s and reload_s > 0:
        push = press + max(0.0, reload_s - RELOAD_RUN_LEAD_S)
        end = press + reload_s
    else:
        push = press + RELOAD_RUN_FALLBACK_START_S
        end = press + max(RELOAD_UNKNOWN_S, RELOAD_RUN_FALLBACK_START_S)
    return {'press': round(press, 3), 'push': round(push, 3), 'end': round(end, 3),
            'duration': round(max(end, push) + RELOAD_RUN_TAIL_S + 0.2, 2)}


def reload_walk_timing(reload_s: float | None) -> dict:
    """Seconds from the start of the reload_walk scenario: the reload press, the expected end of the reload, the scenario length."""
    est = reload_s if reload_s and reload_s > 0 else RELOAD_UNKNOWN_S
    return {'press': RELOAD_WALK_LEAD_S, 'end': round(RELOAD_WALK_LEAD_S + est, 3),
            'duration': round(RELOAD_WALK_LEAD_S + est + RELOAD_WALK_TAIL_S, 2)}


def _reload_trigger(press: float, by_shot: bool) -> dict:
    """What starts the reload at `press`: the reload button, or (`by_shot`: a weapon whose magazine one pull empties, the game
    then reloads it by itself) one pull of the trigger."""
    if by_shot:
        return {'rt': {'keys': [[0, 0], [press, 1], [round(press + 0.1, 3), 0]], 'interp': 'step'}}
    return {'buttons': [{'t': press, 'button': 'X', 'hold': 0.12}]}


def reload_walk_dict(reload_s: float | None = None, by_shot: bool = False) -> dict:
    t = reload_walk_timing(reload_s)
    return {'steps': [dict(_IDLE, name='reload_walk', duration=t['duration'], interp='step',
                           ls=[[0, 0, 0], [0.05, 0, RELOAD_WALK_STICK]], **_reload_trigger(t['press'], by_shot))]}


def reload_then_run_dict(reload_s: float | None = None, by_shot: bool = False) -> dict:
    t = reload_run_timing(reload_s)
    return {'steps': [dict(_IDLE, name='reload_run', duration=t['duration'], interp='step',
                           ls=[[0, 0, 0], [t['push'], 0, 0], [round(t['push'] + 0.05, 3), 0, 1.0]],
                           **_reload_trigger(t['press'], by_shot))]}


def judge_reload_walk(res: dict, moved_m: float | None) -> dict:
    """reload_walk: -> {'verdict': 'PASS' | 'FAIL', 'fails': [..], 'warnings': [..]}. The magazine must have been refilled and V must
    have covered RELOAD_WALK_MIN_M during the reload (not measurable = a warning)."""
    fails, warns = [], []
    if res['detected'] is False:
        fails.append(f'the magazine did not refill ({res["mag_start"]} -> {res["mag_end"]} of {res["cap"]}) and no reload episode is '
                     'in the log')
    elif res['detected'] is None:
        warns.append('the magazine is not readable and no reload episode is in the log: the reload is not verified')
    elif res['cap'] is not None and not res['full']:
        warns.append(f'the magazine was at {res["mag_end"]} of {res["cap"]} when the wait ended')
    if moved_m is None:
        warns.append('the distance covered during the reload is not verified (no position lines around the reload)')
    elif moved_m < RELOAD_WALK_MIN_M:
        fails.append(f'V moved only {moved_m:.2f} m during the reload (at least {RELOAD_WALK_MIN_M:g} m expected)')
    return {'verdict': 'FAIL' if fails else 'PASS', 'fails': fails, 'warnings': warns}


def judge_reload_run(res: dict, push_at: datetime | None, episode_end: datetime | None, end_source: str) -> dict:
    """reload_then_run: -> {'verdict': 'PASS' | 'FAIL' | 'INVALID', 'fails', 'warnings', 'invalid_reason'}. The magazine must have been
    refilled and the movement must have begun before the reload episode ended; a reload that ended before V moved is INVALID (the
    phase did not reach the case it is meant to measure). `end_source` says where `episode_end` comes from ('log' | 'estimated' | '')."""
    fails, warns, invalid = [], [], None
    if res['detected'] is False:
        fails.append(f'the magazine did not refill ({res["mag_start"]} -> {res["mag_end"]} of {res["cap"]}) and no reload episode is '
                     'in the log')
    elif res['detected'] is None:
        warns.append('the magazine is not readable and no reload episode is in the log: the reload is not verified')
    elif res['cap'] is not None and not res['full']:
        warns.append(f'the magazine was at {res["mag_end"]} of {res["cap"]} when the wait ended')
    if not fails:
        if push_at is None or episode_end is None:
            warns.append('the end of the reload episode is not known: the order of reload end and movement is not verified')
        else:
            gap = (episode_end - push_at).total_seconds()
            if gap <= 0:
                invalid = (f'the reload had already ended before V moved (the stick was pushed {-gap:.2f} s after the end of the '
                           f'reload episode, {end_source})')
            elif end_source == 'estimated':
                warns.append('the end of the reload episode is estimated (no episode line in the log), not read')
    return {'verdict': 'FAIL' if fails else 'INVALID' if invalid else 'PASS', 'fails': fails, 'warnings': warns,
            'invalid_reason': invalid}


def phases_of(tag: str) -> tuple:
    """The phases of a family in order (PHASES without `bash` when the family has the flag switched off)."""
    on = FAMILIES[tag].get('bash', True)
    return tuple(p for p in PHASES if p != 'bash' or on)


def bash_result(text: str) -> dict:
    """Did the game play the quick melee? `text` = the UTPP log written since the tap. verified: True when an MMGATE line carries the
    melee-attack bit (BASH_MELEE_BIT) in `melee=` (the line exists only with the MM trace on and when the melee gate changes), else
    False = not verified (the move may still have happened). `line` = the first such line, trimmed."""
    lines = []
    for l in gt_log.parse_text(text or ''):
        if l.kind != 'GATE':
            continue
        try:
            bits = int(l.fields.get('melee', '0x0'), 16)
        except ValueError:
            continue
        if bits & BASH_MELEE_BIT:
            lines.append(l)
    return {'verified': bool(lines), 'gate_lines': len(lines), 'line': lines[0].raw.split('[UTPP]', 1)[-1].strip()[:160] if lines else None}


def _taps(n: int, gap: float, first: float = 0.3, hold: float = 0.1) -> list:
    keys: list = [[0, 0]]
    for i in range(n):
        t = round(first + i * gap, 3)
        keys += [[t, 1], [round(t + hold, 3), 0]]
    return keys


_IDLE = gw._IDLE


def fire_scenario_dict(phase: str, family_gap: float, shots: int) -> dict:
    """Pad steps of a fire phase. hip_fire: no aim, `shots` RT taps; aim_fire: LT up, taps with LT held, LT released. The steps
    carry no pass / fail metrics (they are measured by the pose meter windows and the log)."""
    gap = max(0.4, float(family_gap))
    if phase == 'hip_fire':
        return {'steps': [dict(_IDLE, name='fire', duration=round(0.3 + max(1, shots) * gap + 0.3, 2),
                               rt={'keys': _taps(shots, gap), 'interp': 'step'})]}
    if phase == 'aim_fire':
        return {'steps': [
            {'name': 'aim', 'duration': 0.7, 'ls': [[0, 0, 0]], 'rs': [[0, 0, 0]], 'lt': [[0, 0], [0.1, 1]], 'rt': [[0, 0]],
             'metrics': False},
            {'name': 'shots', 'duration': round(0.2 + max(1, shots) * gap + 0.3, 2), 'lt': [[0, 1]], 'metrics': False,
             'rt': {'keys': _taps(shots, gap, first=0.2), 'interp': 'step'}},
            {'name': 'lower', 'duration': 0.4, 'lt': [[0, 1], [0.1, 0]], 'rt': [[0, 0]], 'metrics': False}]}
    raise ValueError(f'not a fire phase: {phase}')


def static_scenario_dict(phase: str) -> dict:
    """Pad steps of the phases that need no family data."""
    if phase == 'aim_idle':
        return {'steps': [
            dict(_IDLE, name='aim', duration=2.0, lt={'keys': [[0, 0], [0.1, 1]], 'interp': 'smooth'}),
            {'name': 'lower', 'duration': 0.4, 'lt': [[0, 1], [0.1, 0]], 'metrics': False}]}
    if phase == 'reload':
        # X tap = Reload_Button (X is also the interaction button: the set walks away from the car first)
        return {'steps': [dict(_IDLE, name='reload', duration=0.6, buttons=[{'t': 0.2, 'button': 'X', 'hold': 0.12}])]}
    if phase == 'bash':
        # one LB tap (quick melee, Alternative scheme) with the weapon in the hand, hip stance, then the window runs out
        return {'steps': [dict(_IDLE, name='bash', duration=BASH_WINDOW_S,
                               buttons=[{'t': 0.3, 'button': BASH_BUTTON, 'hold': BASH_TAP_S}])]}
    if phase == 'reload_shot':
        # one trigger pull inside the reload phase: a magazine that one pull empties is refilled by the game right away, so the
        # reload of such a weapon can only be seen after a shot
        return {'steps': [dict(_IDLE, name='shot', duration=0.9, rt={'keys': _taps(1, 0.5, first=0.2), 'interp': 'step'})]}
    if phase == 'crouch_aim':
        return {'steps': [
            dict(_IDLE, name='crouch', duration=1.0, buttons=[gw._R3_TAP]),
            {'name': 'aim', 'duration': 1.6, 'ls': [[0, 0, 0]], 'lt': [[0, 0], [0.1, 1]], 'metrics': False},
            {'name': 'lower', 'duration': 0.3, 'lt': [[0, 1], [0.1, 0]], 'metrics': False},
            dict(_IDLE, name='stand', duration=1.0, buttons=[gw._R3_TAP])]}
    if phase == 'stand':
        return {'steps': [dict(_IDLE, name='toggle', duration=1.2, buttons=[gw._R3_TAP])]}
    raise ValueError(f'no static scenario for {phase}')


def phase_label(tag: str, phase: str) -> str:
    return f'rf_{tag}_{phase}'


def phase_plan(tag: str) -> list[str]:
    """Names of all phases in order for a family (equip .. remove)."""
    return ['equip'] + list(phases_of(tag)) + ['remove']


def scenario_for(tag: str, phase: str, shots: tuple[int, int] = (3, 3), reload_s: float | None = None, by_shot: bool = False):
    """The Scenario of a pad phase (named rf_<family>_<phase>); the moving phases are copies of the static parts of
    scenarios.json (loaded by the runner), so this builds only the phases without one. `reload_s` = reload duration of the weapon for
    the two moving-reload phases; `reload_prep` = the shots that make the magazine short before them (`shots[0]` of them);
    `by_shot` = the two moving-reload phases pull the trigger where they would press the reload button."""
    from gt_scenario import parse_scenario
    name = phase_label(tag, phase)
    if phase in ('hip_fire', 'aim_fire'):
        raw = fire_scenario_dict(phase, FAMILIES[tag]['gap'], shots[0] if phase == 'hip_fire' else shots[1])
    elif phase == 'reload_prep':
        raw = fire_scenario_dict('hip_fire', FAMILIES[tag]['gap'], shots[0])
    elif phase == 'reload_walk':
        raw = reload_walk_dict(reload_s, by_shot)
    elif phase == 'reload_then_run':
        raw = reload_then_run_dict(reload_s, by_shot)
    else:
        raw = static_scenario_dict(phase)
    return parse_scenario(name, dict(raw, description=f'ranged family {tag}: {phase}'))


def reload_motion_pad_seconds(tag: str) -> float:
    """Pad input time of the two moving-reload phases of a family, including the shots before each (reload duration unknown)."""
    prep = scenario_for(tag, 'reload_prep', (reload_prep_shots(None), 0)).duration
    return sum(prep + scenario_for(tag, ph).duration for ph in RELOAD_MOTION_PHASES if ph in phases_of(tag))


def pad_seconds(tag: str, scenarios: dict | None = None, shots: tuple[int, int] = (3, 3)) -> float:
    """Pad input time of one family (the phases that play a scenario)."""
    total = 0.0
    for ph in phases_of(tag):
        if ph in MOVING_PARTS:
            sc = (scenarios or {}).get(MOVING_PARTS[ph])
            total += sc.duration if sc is not None else 0.0
        elif ph in ('draw', 'holster'):
            continue
        elif ph in RELOAD_MOTION_PHASES:
            continue                                   # counted below with the shots before them
        else:
            total += scenario_for(tag, ph, shots).duration
    return total + reload_motion_pad_seconds(tag)


# ---------------------------------------------------------------------------------------------- macro

def _gn():
    import gt_night
    return gt_night


class _Run:
    """State of one family run (log windows, results)."""

    def __init__(self, night, pad, guard, plan, fam: dict, records, run_no: int = 1, out: dict | None = None):
        self.night, self.pad, self.guard, self.plan, self.fam, self.records = night, pad, guard, plan, fam, tuple(records)
        self.out = out if out is not None else {}
        self.tag = fam['tag']
        self.run_no = run_no
        self.phases: list = []
        self.windows: list = []
        self.fails: list = []
        self.warns: list = []
        self.invalid_phases: list = []         # phases that were not scored: [{'phase', 'reason'}]
        self.reload_dur: float | None = None   # duration of the standing reload (the `RELOAD episode` line), None = unknown
        self.seated = False                    # V sits in a vehicle and could not be taken out again
        self.given: dict | None = None
        self.record: str | None = None
        self.removed = None
        self.ammo_removed = None
        self.cap: int | None = None
        self.mag: int | None = None
        self.total: int | None = None
        self.shots = (3, 3)
        self.view = {'family': self.tag, 'label': fam['label'], 'verdict': None, 'skip_reason': None, 'record': None,
                     'evidence': None, 'types': list(fam['types']), 'candidates': list(self.records), 'given': None,
                     'removed': None, 'ammo_removed': None, 'fails': self.fails, 'warnings': self.warns, 'phases': self.phases,
                     'phase_windows': self.windows, 'invalid_phases': self.invalid_phases}

    def say(self, msg: str) -> None:
        self.night.log(f'{LOG_PREFIX} {self.tag}: {msg}')

    def cmd(self, tag: str, cmd: str, label: str):
        """Send an echo-verified command; a lost command is sent once more (every command is safe to repeat)."""
        ok, info = None, ''
        for attempt in (1, 2):
            text = self.night.console_verified(self.pad, self.guard, cmd, lambda t, tg=tag: f'[night] {tg} ok=' in t,
                                               f'{self.tag}/{label}')
            ok, info = gw.echo_of(text, tag)
            if ok is not None:
                break
            self.say(f'{label}: echo LOST' + (', sent again' if attempt == 1 else ''))
        self.say(f'{label}: ' + (f'ok={ok} {info[:140]}' if ok is not None else 'echo LOST twice'))
        return ok, info

    def hand(self, label: str = 'hand') -> dict | None:
        ok, info = self.cmd('RF_HAND', hand_cmd(self.record), label)
        h = parse_hand(info) if ok else None
        if h:
            self.mag = h['mag']
            self.total = h['total']
            self.cap = h['cap'] or self.cap
        return h

    def window(self, phase: str, t0: datetime, t1: datetime, extra: dict | None = None, plan_run: bool = False) -> dict:
        """Record a labelled phase. plan_run: the phase plays no pad scenario, so it gets a plan run of its own (one step over the
        window, no metrics): the helper reports and the harness tables read phases from the plan runs."""
        label = phase_label(self.tag, phase)
        if plan_run:
            n = 1 + sum(1 for r in self.plan['runs'] if r.get('scenario') == label)
            self.plan['runs'].append({'scenario': label, 'run': n, 'aborted': None,
                                      'steps': [{'name': phase, 'start': t0, 'end': t1, 'metrics': False}]})
        rec = {'phase': phase, 'label': label, 'start': gw._stamp(t0), 'end': gw._stamp(t1)}
        if extra:
            rec.update(extra)
        self.phases.append(rec)
        self.windows.append({'family': self.tag, 'phase': phase, 'label': label, 'start': rec['start'], 'end': rec['end']})
        self.say(f'{phase}: {rec["start"]}..{rec["end"]}' + (f' {extra}' if extra else ''))
        return rec


def _register(run: _Run, phase: str, shots=(3, 3), reload_s: float | None = None, by_shot: bool = False):
    """The Scenario of a phase, registered in the runner's table under its own name (the walking helpers look scenarios up there)."""
    n = run.night
    name = phase_label(run.tag, phase)
    if phase in MOVING_PARTS:
        base = n.scenarios[MOVING_PARTS[phase]]
        sc = dataclasses.replace(base, name=name, description=f'ranged family {run.tag}: {phase} ({MOVING_PARTS[phase]})')
    else:
        sc = scenario_for(run.tag, phase, shots, reload_s, by_shot)
    n.scenarios[name] = sc
    return sc


def _wait_weapon_ready(run: _Run, why: str, limit: float | None = None) -> int | None:
    """Hold the next shot or reload press until the game takes it: the newest position line must show a weapon state that is not in
    WEAPON_BUSY (the bolt cycle of a sniper rifle lasts about 2.5 s; a reload goes on after the magazine count has grown). Returns
    the state; None = the position lines carry no weapon state, so there is nothing to wait for."""
    log = run.night.utpp_log

    def newest() -> int | None:
        return weapon_state(read_since(log, max(0, file_size(log) - WEAPON_STATE_TAIL)))

    state = newest()
    if state not in WEAPON_BUSY:
        return state
    limit = WEAPON_READY_WAIT_S if limit is None else limit
    first, t0 = state, time.monotonic()
    while state in WEAPON_BUSY and time.monotonic() - t0 < limit:
        run.guard.wait(WEAPON_READY_POLL_S)
        state = newest()
    if state in WEAPON_BUSY:
        run.warns.append(f'{why}: the weapon was still {WEAPON_BUSY[state]} after {limit:g} s; the button was sent anyway')
    else:
        run.say(f'{why}: waited {time.monotonic() - t0:.1f} s for the weapon ({WEAPON_BUSY[first]})')
    return state


def _extend_run_end(plan: dict, name: str, end: datetime) -> None:
    """The last plan run of this name ends its last step at `end` (the reload plays on after the pad input ended)."""
    for r in reversed(plan.get('runs', [])):
        if r.get('scenario') == name:
            steps = r.get('steps') or []
            if steps and steps[-1].get('end') and steps[-1]['end'] < end:
                steps[-1]['end'] = end
            return


def _tag_part(plan: dict, name: str, part: str) -> None:
    """The plan runs of a family copy name the static part they were built from, so the analysis uses its thresholds."""
    for r in list(plan.get('runs', [])) + list(plan.get('discarded_runs', [])):
        if r.get('scenario') == name:
            r['part'] = part


def _pad_phase(run: _Run, phase: str, extra: dict | None = None) -> dict:
    sc = _register(run, phase, run.shots)
    t0 = datetime.now()
    run.night.play(run.pad, run.guard, sc.name, run.plan, sc)
    return run.window(phase, t0, datetime.now(), extra)


def _walk_phase(run: _Run, phase: str) -> dict:
    gn = _gn()
    sc = _register(run, phase)
    # open ground is probed before the walk and the sprint (the aimed walk follows the walk on the ground it just covered)
    walked = run.night.play_walking(run.pad, run.guard, run.plan, sc.name,
                                    clear=gn.commands_movement(sc) and phase in PROBE_PHASES)
    _tag_part(run.plan, sc.name, MOVING_PARTS[phase])
    return run.window(phase, walked.get('start') or datetime.now(), datetime.now())


def _fire_phase(run: _Run, phase: str) -> dict:
    mag_before, total_before = run.mag, run.total
    planned = run.shots[0] if phase == 'hip_fire' else run.shots[1]
    rec = _pad_phase(run, phase, {'shots_planned': planned})
    run.guard.wait(0.3)
    h = run.hand(f'mag-{phase}')
    res = fire_result(mag_before, run.mag if h else None, total_before, run.total if h else None, planned)
    rec.update(mag_before=mag_before, mag_after=run.mag if h else None, total_before=total_before,
               total_after=run.total if h else None, shots_seen=res['shots'])
    if res['problem']:
        run.fails.append(f'{phase}: {res["problem"]} (magazine {mag_before} -> {run.mag}, total {total_before} -> {run.total})')
    elif res['note']:
        run.warns.append(f'{phase}: {res["note"]}')
    return rec


def _bash_phase(run: _Run) -> dict:
    """One quick-melee tap with the weapon in the hand. No pass / fail limit: the shape values of the window are only reported. Whether
    the game played the move is read from the log written since the tap; without the marker the phase is `not verified` (a warning)."""
    off = file_size(run.night.utpp_log)
    rec = _pad_phase(run, 'bash', {'button': BASH_BUTTON, 'window_s': BASH_WINDOW_S})
    # the magazine is read again (a bash can cost ammunition and the reload starts from it); the echo of that command also gives the
    # log a moment to write the gate line before it is read
    h = run.hand('mag-bash')
    rec.update(mag_after=run.mag if h else None, total_after=run.total if h else None)
    res = bash_result(read_since(run.night.utpp_log, off))
    rec['quick_melee'] = 'seen' if res['verified'] else 'not verified'
    rec['marker'] = res['line']
    if res['verified']:
        run.say(f'bash: quick melee seen in the log ({res["gate_lines"]} MMGATE line(s) with the melee-attack bit)')
    else:
        run.say('bash: no quick melee line in the log: not verified')
        run.warns.append(f'bash: not verified - no MMGATE line with the melee-attack bit {BASH_MELEE_BIT:#x} after the {BASH_BUTTON} tap '
                         '(the game may not play a gun bash on the third-person body, the binding may differ, or the MM trace / '
                         'melee gate is off)')
    return rec


def _equip(run: _Run) -> str | None:
    """Pick, give and draw. Returns None when the weapon is in the hand, else the reason it is not testable ('skip: ...' = no
    record, 'fail: ...' = command failure)."""
    fam = run.fam
    t0, off = datetime.now(), file_size(run.night.utpp_log)
    ok, info = run.cmd('RF_PICK', pick_cmd(fam, run.records), 'pick')
    picked = parse_pick(info) if ok else None
    if ok is False and 'no usable record' in info:
        return skip_text(fam, run.records, info)
    if not picked:
        return 'fail: the record check got no answer' + (f' ({info[:160]})' if info else ' (no echo)')
    run.record = picked['rec']
    run.view.update(record=picked['rec'], evidence=record_evidence(picked['rec']), item_type=picked['type'])
    # the cleanup is armed from the pick (copies owned before the give) before the give is sent: a lost or odd echo must not leave
    # the weapon in the inventory; the ammunition count before the give is known only from the give echo
    for note in candidate_notes(picked['seen'], fam['types'], picked['rec']):
        run.say(f'candidate {note}')
        run.warns.append(f'pick: candidate {note}')
    run.given = {'rec': picked['rec'], 'how': 'had' if picked['n'] > 0 else 'given', 'n_before': picked['n'],
                 'ammo_before': picked['ammo']}
    run.out['hand_changed'] = True
    ok, info = run.cmd('RF_GIVE', give_cmd(fam, picked['rec']), 'give')
    given = parse_give(info) if ok else None
    if not given:
        run.phases.append({'phase': 'equip', 'echo': info[:200], 'record': picked['rec']})
        return 'fail: the give command failed' + (f' ({info[:160]})' if info else ' (no echo)')
    # a command that was sent again reports the state after its first run (had n=1->1): only the pick counts as the baseline
    run.phases.append({'phase': 'equip', 'start': gw._stamp(t0), 'end': gw._stamp(datetime.now()), 'echo': info[:200],
                       'record': picked['rec'], 'how': run.given['how'], 'baseline': {'n': picked['n'], 'ammo': picked['ammo']},
                       'give_echo': {'how': given['how'], 'n': [given['n_before'], given['n_after']],
                                     'ammo': [given['ammo_before'], given['ammo_after']]}})
    t0 = datetime.now()
    h = None
    for attempt in range(2):                                           # one re-draw at most
        for _ in range(DRAW_POLLS):
            h = run.hand('hand-draw')
            if h and h['match']:
                break
            run.guard.wait(0.6)
        if h and h['match']:
            break
        if attempt == 0:
            run.warns.append('draw: the weapon was not in the hand after the first request; drawn again by script')
            run.cmd('RF_GIVE', give_cmd(fam, picked['rec']), 'redraw')
    drawn = bool(h and h['match'])
    run.window('draw', t0, datetime.now(), {'in_hand': drawn, 'mag': h['mag'] if h else None, 'cap': h['cap'] if h else None,
                                            'hash': h['hash'] if h else None}, plan_run=True)
    if not drawn:
        return 'fail: the weapon is not in the hand after the draw request' + (f' (hand hash {h["hash"]})' if h else ' (no echo)')
    run.shots = plan_shots(run.cap)
    if run.cap is None:
        run.warns.append('draw: the magazine capacity could not be read; 3 + 3 shots planned')
    return None


def _ensure_in_hand(run: _Run, where: str) -> bool:
    h = run.hand(f'hand-{where}')
    if h and h['match']:
        return True
    run.warns.append(f'{where}: the weapon was not in the hand (hand: {h["hash"] if h else "no answer"}); drawn again by script')
    run.cmd('RF_GIVE', give_cmd(run.fam, run.record), f'redraw-{where}')
    run.guard.wait(1.0)
    h = run.hand(f'hand-{where}-2')
    return bool(h and h['match'])


def _wait_refill(run: _Run, mag_start: int | None, off: int, shot_in_phase: bool = False) -> tuple[dict, float]:
    """Poll the magazine (and the reload episodes of the log since `off`) until the reload is seen to be done or the limit is up.
    -> (reload_result, limit in s)."""
    gn = _gn()
    readable = run.cap is not None and mag_start is not None
    limit = RELOAD_MAX_S if readable else UNREADABLE_WAIT_S        # nothing to wait for when only a log line could say it ended
    t_end = time.monotonic() + limit
    t_begin = time.monotonic()
    res = None
    while True:
        h = run.hand('mag-reload')
        episodes = gn.reload_episodes(read_since(run.night.utpp_log, off))
        res = reload_result(mag_start, run.mag if h else None, run.cap, episodes, time.monotonic() - t_begin)
        if shot_in_phase and not res['detected'] and res['full']:
            # a shot was fired in this phase and the magazine is full again: the game reloaded by itself
            res.update(detected=True, via='self reload after a shot')
        res['shot_in_phase'] = shot_in_phase
        if res['detected'] and (res['full'] or episodes):
            break
        if time.monotonic() >= t_end:
            break
        run.guard.wait(RELOAD_POLL_S)
    return res, limit


def _reload_phase(run: _Run) -> dict:
    mag_start, total_start = run.mag, run.total
    t0, off = datetime.now(), file_size(run.night.utpp_log)
    shot_in_phase = False
    if run.cap is not None and mag_start is not None and mag_start >= run.cap:
        # nothing to reload: the game refilled the magazine by itself after the last shot. One shot inside this phase.
        shot = _register(run, 'reload_shot')
        run.night.play(run.pad, run.guard, shot.name, run.plan, shot)
        h = run.hand('mag-reload-shot')
        shot_in_phase = bool(h and total_start is not None and run.total is not None and run.total < total_start)
        if shot_in_phase and run.mag is not None and run.mag < run.cap:
            mag_start = run.mag            # the magazine is short now: the reload below fills it
    if not shot_in_phase:
        _wait_weapon_ready(run, 'reload: before the press')
    sc = _register(run, 'reload')
    run.night.play(run.pad, run.guard, sc.name, run.plan, sc)
    res, limit = _wait_refill(run, mag_start, off, shot_in_phase)
    if res['detected'] and not res['episodes']:
        # the magazine count grows before the reload has ended: the log line with its duration (the moving phases are timed with
        # it) is written, and the game takes the next button, only once the reload is over
        _wait_weapon_ready(run, 'reload: end of the reload')
        res['episodes'] = _gn().reload_episodes(read_since(run.night.utpp_log, off))
    if res['episodes']:
        run.reload_dur = res['episodes'][-1]
    t1 = datetime.now()
    rec = run.window('reload', t0, t1, {'result': res, 'wait_limit_s': limit})
    _extend_run_end(run.plan, phase_label(run.tag, 'reload'), t1)
    if res['detected'] is False:
        if mag_start is not None and run.cap is not None and mag_start >= run.cap:
            run.warns.append('reload: the magazine was full before the reload (no shot was counted), the reload was not exercised')
        else:
            run.fails.append(f'reload: the magazine did not refill within {limit:.0f} s ({mag_start} -> {res["mag_end"]}'
                             f' of {run.cap}) and no reload episode is in the log')
    elif res['detected'] and run.cap is not None and not res['full']:
        run.warns.append(f'reload: the magazine was at {res["mag_end"]} of {run.cap} when the wait ended ({limit:.0f} s)')
    elif res['detected'] is None:
        run.warns.append('reload: the magazine is not readable and no reload episode is in the log: the reload is not verified')
    return rec


_RERAISE =('ScenarioTimeout', 'StopNight', 'CycleError', 'EarlyStop', 'GuardError')


EXIT_RECHECKS = 4          # position reads after a vehicle exit before V counts as still seated
EXIT_RECHECK_S = 0.5       # pause between them (the position lines come about twice a second)


def _mount_state(run: _Run) -> tuple[bool | None, dict | None]:
    """(V in a vehicle: True | False | None = unknown, the position record) from a position read."""
    poll = getattr(run.night, 'poll_pos', None)
    pos = poll(run.guard, run.pad) if poll is not None else None
    return (pos.get('mounted') if pos else None), pos


def _leave_vehicle(run: _Run, pos: dict | None, why: str) -> bool:
    """V sits in a vehicle (the reload button is also the mount button): leave it with the runner's own procedure, then walk away
    from it, so that the next reload press cannot mount again. True = V is on foot."""
    night = run.night
    ensure = getattr(night, 'ensure_unmounted', None)
    if ensure is None:
        run.seated = True
        run.warns.append(f'{why}: the runner has no vehicle exit; V is still in the vehicle')
        return False
    try:
        if pos and pos.get('mounted') is True:
            ensure(run.pad, run.guard, run.plan, why, pos=pos)
        else:
            ensure(run.pad, run.guard, run.plan, why)
    except Exception as e:  # noqa: BLE001
        if type(e).__name__ in _RERAISE:
            raise
        run.warns.append(f'{why}: leaving the vehicle failed: {type(e).__name__}: {str(e)[:100]}')
    state, _ = _mount_state(run)
    for _ in range(EXIT_RECHECKS):               # the newest position line can be older than the exit that just ended
        if state is not True:
            break
        run.guard.wait(EXIT_RECHECK_S)
        state, _ = _mount_state(run)
    run.seated = state is True
    if run.seated:
        run.warns.append(f'{why}: V is still in the vehicle after the exit procedure')
        return False
    away = getattr(night, 'leave_car', None)
    if away is not None:
        try:
            away(run.pad, run.guard, run.plan)
        except Exception as e:  # noqa: BLE001
            if type(e).__name__ in _RERAISE:
                raise
            run.warns.append(f'{why}: walking away from the vehicle failed: {type(e).__name__}: {str(e)[:100]}')
    return True


def _not_scored(run: _Run, phase: str, reason: str, t0: datetime, extra: dict | None = None, t1: datetime | None = None) -> dict:
    """A moving-reload phase that could not be scored: recorded as a phase window with the reason, never a failure. `t1` = end of the
    window (default now); a phase that ends with a vehicle exit passes the time taken before it, so the window does not cover the exit."""
    rec = run.window(phase, t0, t1 or datetime.now(), {'verdict': 'INVALID', 'invalid_reason': reason})
    if extra:
        rec.update(extra)
    run.invalid_phases.append({'phase': phase, 'reason': reason})
    run.warns.append(f'{phase}: INVALID (not scored): {reason}')
    return rec


def _reload_motion_phase(run: _Run, phase: str) -> dict | None:
    """reload_walk / reload_then_run (see the module text): a few shots, then one pad scenario with the reload button in it, then the
    magazine. The mounted flag is read before the press and after it (position lines of the window, the vehicle camera line, a read
    after the scenario); a V that got into a vehicle makes the phase INVALID."""
    gn = _gn()
    night = run.night
    t_phase = datetime.now()
    if run.seated:
        return _not_scored(run, phase, 'V is still in a vehicle from the earlier phase', t_phase)
    if not _ensure_in_hand(run, phase):
        run.fails.append(f'{phase}: the weapon could not be put back in the hand; phase skipped')
        return None
    # shots first: the magazine must be short, else there is nothing to reload
    _wait_weapon_ready(run, f'{phase}: before the shots')
    total_start = run.total
    prep = _register(run, 'reload_prep', (reload_prep_shots(run.cap), 0))
    h, full, fired = None, False, False
    for attempt in (1, 2):
        night.play(run.pad, run.guard, prep.name, run.plan, prep)
        run.guard.wait(0.3)
        h = run.hand(f'mag-{phase}-prep')
        full = run.cap is not None and run.mag is not None and run.mag >= run.cap
        fired = bool(h) and total_start is not None and run.total is not None and run.total < total_start
        if not full or fired or attempt == 2:
            break
        # no round left the weapon: the pulls came while the game was still busy with it, or only raised a lowered weapon
        run.say(f'{phase}: the shots before the reload did not fire ({run.mag} of {run.cap}); sent once more')
        _wait_weapon_ready(run, f'{phase}: before the second shots')
    if full and not fired:
        return _not_scored(run, phase, f'the magazine is still full after the shots ({run.mag} of {run.cap}): nothing to reload',
                           datetime.now())
    # full again although rounds were fired: one pull empties this magazine and the game reloads it by itself, so the reload of the
    # phase is started by a pull of the trigger instead of the reload button
    by_shot = full and fired
    mag_start = run.mag if h else None
    seated, pos0 = _mount_state(run)
    if seated is True:
        t_end = datetime.now()
        left = _leave_vehicle(run, pos0, f'{phase}: before the press')
        return _not_scored(run, phase, 'V was in a vehicle before the reload press' + ('' if left else ' and could not leave it'),
                           t_end, t1=t_end)
    reload_s = run.reload_dur
    sc = _register(run, phase, (0, 0), reload_s, by_shot)
    clear = getattr(night, 'clear_path', None)
    if phase == 'reload_walk' and clear is not None:
        clear(run.pad, run.guard, run.plan, sc.name)                 # open ground ahead (one probe, no repeat of the phase)
    _wait_weapon_ready(run, f'{phase}: before the press')            # a press during the shot cycle or a reload is ignored
    total_before = run.total
    off, t0 = file_size(night.utpp_log), datetime.now()
    played = night.play(run.pad, run.guard, sc.name, run.plan, sc) or {}
    steps = played.get('steps') or []
    start = steps[0].get('start') if steps and isinstance(steps[0], dict) and steps[0].get('start') else t0
    timing = reload_walk_timing(reload_s) if phase == 'reload_walk' else reload_run_timing(reload_s)
    press_at = start + timedelta(seconds=timing['press'])
    # what the scenario commanded, kept for every outcome
    when = {'press_at': gw._stamp(press_at), 'reload_by': 'shot' if by_shot else 'button'}
    if phase == 'reload_then_run':
        when['push_at'] = gw._stamp(start + timedelta(seconds=timing['push']))
    text = read_since(night.utpp_log, off)
    samples = pos_records(text)
    after, pos1 = _mount_state(run)
    mounted = any(x['mounted'] for x in samples) or gn.MOUNT_RIDE in text or after is True
    if mounted:
        t_end = datetime.now()
        left = _leave_vehicle(run, pos1, f'{phase}: after the press')
        return _not_scored(run, phase, 'the reload button mounted V into a vehicle' + ('' if left else ' (V could not leave it)'),
                           t0, dict(when, mounted=True), t_end)
    if phase == 'reload_walk':
        blocked = getattr(night, 'attempt_blocked', None)
        rep = blocked(sc.name, {'res': played, 'off': off}) if blocked is not None and played else None
        if rep:
            return _not_scored(run, phase, f'V was stuck while the stick was pushed (stall {rep["stall_s"]} s)', t0,
                               dict(when, blocked=rep['stall_s']))
    res, limit = _wait_refill(run, mag_start, off, by_shot)
    t1 = datetime.now()
    text = read_since(night.utpp_log, off)
    samples = pos_records(text)
    if any(x['mounted'] for x in samples) or gn.MOUNT_RIDE in text:
        # the mount showed only after the scenario (the enter animation writes its line late): the magazine of a seated V is not judged
        left = _leave_vehicle(run, None, f'{phase}: after the press')
        return _not_scored(run, phase, 'the reload button mounted V into a vehicle (seen after the scenario)'
                           + ('' if left else ' (V could not leave it)'), t0, dict(when, mounted=True), t1)
    if by_shot and not res['episodes'] and not (total_before is not None and run.total is not None and run.total < total_before):
        # a full magazine alone does not show a reload here: without a fired round the game had nothing to refill
        return _not_scored(run, phase, f'the pull of the trigger fired no round (ammunition {total_before} -> {run.total}): the game '
                           'had nothing to reload', t0, when, t1)
    stamps = [(t, d) for t, d in reload_episode_stamps(text) if t is not None]
    ep_end, ep_dur = (stamps[-1] if stamps else (None, None))
    detail: dict = {'result': res, 'wait_limit_s': limit, 'mag_start': mag_start, 'reload_s_used': reload_s,
                    'reload_s_source': 'standing reload episode' if reload_s else 'unknown (fallback timing)',
                    'press_at': when['press_at'], 'reload_by': when['reload_by'],
                    'episode_end': gw._stamp(ep_end) if ep_end else None}
    if phase == 'reload_walk':
        a = ep_end - timedelta(seconds=ep_dur) if ep_end else press_at
        b = ep_end if ep_end else start + timedelta(seconds=timing['end'])
        moved = moved_between(samples, a, b)
        detail.update(moved_m=moved, moved_from=gw._stamp(a), moved_to=gw._stamp(b))
        verdict = judge_reload_walk(res, moved)
    else:
        push_at = start + timedelta(seconds=timing['push'])
        if ep_end:
            end, src = ep_end, 'log'
        elif reload_s:
            end, src = press_at + timedelta(seconds=reload_s), 'estimated'
        else:
            end, src = None, ''
        seen = moving_since(samples, push_at)
        detail.update(push_at=gw._stamp(push_at), episode_end_source=src or None,
                      push_before_end_s=round((end - push_at).total_seconds(), 2) if end else None,
                      moving_at=gw._stamp(seen) if seen else None,
                      moved_after_push_m=moved_between(samples, push_at, start + timedelta(seconds=timing['duration'])))
        after_end = round((start + timedelta(seconds=timing['duration']) - end).total_seconds(), 2) if end else None
        detail['run_after_end_s'] = after_end                   # how long the stick stayed pushed after the reload episode ended
        verdict = judge_reload_run(res, push_at, end, src)
        if after_end is not None and after_end < RELOAD_RUN_TAIL_S:
            verdict['warnings'].append(f'the scenario ended {after_end:.2f} s after the reload episode (wanted {RELOAD_RUN_TAIL_S:g} s of running)')
        if samples and seen is None and not verdict['invalid_reason']:
            verdict['warnings'].append(f'V was not seen moving after the stick was pushed (no position line {RELOAD_MOVING_MIN_M:g} m '
                                       'away from the push position)')
    detail.update(verdict=verdict['verdict'], warnings=verdict['warnings'])
    rec = run.window(phase, t0, t1, {'verdict': verdict['verdict']})
    rec.update(detail)
    _extend_run_end(run.plan, phase_label(run.tag, phase), t1)
    run.warns += [f'{phase}: {w}' for w in verdict['warnings']]
    if verdict['verdict'] == 'FAIL':
        run.fails += [f'{phase}: {f}' for f in verdict['fails']]
    elif verdict['verdict'] == 'INVALID':
        reason = verdict['invalid_reason']
        rec['invalid_reason'] = reason
        run.invalid_phases.append({'phase': phase, 'reason': reason})
        run.warns.append(f'{phase}: INVALID (not scored): {reason}')
    return rec


STANCE_SETTLE_READS = 4               # stance reads, 0.5 s apart, after a crouch toggle (the stand-up takes a moment)


def _settled_stance(run: _Run, off: int) -> str | None:
    """Stance once it has settled: the stand-up is still under way when a pad phase ends, so the last trace line read at once
    can still say crouched. Reads the log from `off` up to STANCE_SETTLE_READS times and stops at the first standing reading."""
    stance = None
    for _ in range(STANCE_SETTLE_READS):
        run.guard.wait(0.5)
        stance = gw._crouch_state(read_since(run.night.utpp_log, off))
        if stance != 'crouched':
            break
    return stance


def _stand_up(run: _Run, why: str) -> str | None:
    """One R3 tap when V is crouched (stance settled first); returns the stance afterwards. Never taps a standing V."""
    off = max(0, file_size(run.night.utpp_log) - 60000)
    stance = _settled_stance(run, off)
    if stance != 'crouched':
        return stance
    run.warns.append(f'{why}: V is crouched; one R3 tap to stand up')
    off = file_size(run.night.utpp_log)
    sc = _register(run, 'stand')
    run.night.play(run.pad, run.guard, sc.name, run.plan, sc)
    stance = _settled_stance(run, off)
    if stance == 'crouched':
        run.fails.append(f'{why}: V is still crouched after the R3 tap')
    return stance


def _crouch_aim_phase(run: _Run) -> dict:
    off = file_size(run.night.utpp_log)
    rec = _pad_phase(run, 'crouch_aim')
    stance = _settled_stance(run, off)
    rec['stance_end'] = stance
    if stance == 'crouched':
        rec['stance_end'] = _stand_up(run, 'crouch_aim')
    elif stance is None:
        run.warns.append('crouch_aim: no MMTRACE stance lines, the crouch was not verified')
    return rec


def _wait_hand_empty(run: _Run) -> bool:
    for _ in range(3):
        h = run.hand('hand-after-holster')
        if h and h['hash'] == 0:
            return True
        run.guard.wait(0.8)
    run.warns.append('remove: the weapon was still in the hand 2.4 s after the unequip request')
    return False


def _holster(run: _Run) -> None:
    t0 = datetime.now()
    ok, info = run.cmd('RF_HOLSTER', holster_cmd(), 'holster')
    empty = _wait_hand_empty(run)
    run.window('holster', t0, datetime.now(), {'echo': info[:120], 'hand_empty': empty}, plan_run=True)
    if ok is not True:
        run.warns.append('holster: unequip request ' + ('failed' if ok is False else 'echo lost'))
    if not empty:
        run.fails.append('holster: the weapon is still in the hand after the unequip request')


def _remove(run: _Run) -> None:
    g = run.given
    if not g:
        run.removed = 'not given'
        return
    t0, off = datetime.now(), file_size(run.night.utpp_log)
    info = ''
    if g.get('ammo_before') is None:
        run.ammo_removed = 'not given'
        run.warns.append('remove: the ammunition count before the give is unknown (no give echo): the ammunition was not touched')
    else:
        ok, info = run.cmd('RF_AMMO_OFF', ammo_off_cmd(run.fam, g['rec'], g['ammo_before']), 'ammo-off')
        run.ammo_removed = gw.parse_remove(info) if ok else 'failed'
    if g['n_before'] > 0:
        run.removed = 'kept (the player owned it)'
        run.say('the player owned the item: kept, nothing removed')
    else:
        ok2, info2 = run.cmd('RF_REMOVE', remove_cmd(g['rec'], g['n_before']), 'remove')
        run.removed = gw.parse_remove(info2) if ok2 else 'failed'
        info = info2
    run.phases.append({'phase': 'remove', 'start': gw._stamp(t0), 'end': gw._stamp(datetime.now()), 'result': run.removed,
                       'ammo_result': run.ammo_removed, 'echo': info[:120]})
    if run.removed == 'failed':
        run.fails.append(f'remove: the given item {g["rec"]} may still be in the inventory ({info[:120] or "no echo"})')
    elif run.removed == 'nothing':
        run.warns.append('remove: nothing to remove although the item was given')
    if run.ammo_removed == 'failed':
        run.warns.append('remove: the ammunition could not be brought back to its earlier count')


def run_one(night, pad, guard, plan, tag: str, records, out: dict | None = None, run_no: int = 1) -> dict:
    """One family, all phases. Always holsters and removes in a finally. `out['families'][tag]` is the live result."""
    fam = FAMILIES[tag]
    run = _Run(night, pad, guard, plan, fam, records, run_no, out)
    if out is not None:
        out['families'][tag] = run.view
    reason, skip, failed = None, False, False
    try:
        reason = _equip(run)
        skip = bool(reason and reason.startswith('skip'))
        if reason is None:
            only = phase_filter(getattr(night, 'args', None))            # --ranged-phases: None = every phase

            def want(ph: str) -> bool:
                return only is None or ph in only
            _stand_up(run, 'start')          # a crouch left by an earlier set or family would run every phase crouched
            if want('aim_idle'):
                _pad_phase(run, 'aim_idle')
            if want('hip_fire'):
                _fire_phase(run, 'hip_fire')
            if want('aim_fire'):
                _fire_phase(run, 'aim_fire')
            if 'bash' in phases_of(tag) and want('bash'):
                _bash_phase(run)
            if want('reload'):
                _reload_phase(run)
            for ph in RELOAD_MOTION_PHASES:
                if want(ph):
                    _reload_motion_phase(run, ph)
            for ph in ('walk', 'walk_aimed', 'sprint'):
                if not want(ph):
                    continue
                if not _ensure_in_hand(run, ph):
                    run.fails.append(f'{ph}: the weapon could not be put back in the hand; phase skipped')
                    continue
                _walk_phase(run, ph)
            if want('crouch_aim'):
                if _ensure_in_hand(run, 'crouch_aim'):
                    _crouch_aim_phase(run)
                else:
                    run.fails.append('crouch_aim: the weapon could not be put back in the hand; phase skipped')
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
                    run.cmd('RF_HOLSTER', holster_cmd(), 'holster-final')
                    _wait_hand_empty(run)
                _remove(run)
            except Exception as e:  # noqa: BLE001
                run.fails.append(f'cleanup: {type(e).__name__}: {str(e)[:120]}')
            ledger = getattr(night, 'settings', None)
            if ledger is not None:
                for what, state in ((f'inventory.{tag}', run.removed in ('removed', 'nothing', 'kept (the player owned it)')),
                                    (f'ammo.{tag}', run.ammo_removed in ('removed', 'nothing'))):
                    ent = ledger.register(what, 'as before', 'given', SET_NAME, volatile=True)
                    if state:
                        ledger.mark_restored(ent)
        run.view.update(given=run.given, removed=run.removed, ammo_removed=run.ammo_removed, shots_planned=list(run.shots),
                        capacity=run.cap)
    if reason and skip:
        run.warns.append(reason)
    elif reason:
        run.fails.append(reason[6:] if reason.startswith('fail: ') else reason)
    run.view.update(verdict='SKIP' if skip else 'FAIL' if run.fails else 'PASS', skip_reason=reason if skip else None)
    return run.view


def _orig_hand(night, pad, guard) -> dict | None:
    """What is in the hand before the set (hash, 0 = nothing); the final step draws it again."""
    cmd = hand_cmd(None)
    for _ in range(2):
        text = night.console_verified(pad, guard, cmd, lambda t: '[night] RF_HAND ok=' in t, 'ranged/orig-hand')
        ok, info = gw.echo_of(text, 'RF_HAND')
        h = parse_hand(info) if ok else None
        if h:
            return h
    return None


def _restore_hand(night, pad, guard, out: dict, orig: dict | None) -> None:
    """Draw the weapon that was in the hand at the start again (nothing when the hand was empty)."""
    if not orig or not orig['hash'] or not out.get('hand_changed'):
        return
    gn_cmd = redraw_cmd(orig['hash'])
    text = night.console_verified(pad, guard, gn_cmd, lambda t: '[night] RF_REDRAW ok=' in t, 'ranged/redraw-original')
    ok, info = gw.echo_of(text, 'RF_REDRAW')
    state = 'draw requested' if ok and 'found=1' in info else f'not restored ({info[:80] or "no echo"})'
    out['original_weapon'] = {'hash': orig['hash'], 'result': state}
    night.log(f'{LOG_PREFIX} original weapon: {state}')
    if not (ok and 'found=1' in info):
        out['warnings'].append(f'the weapon that was in the hand at the start could not be drawn again ({state})')


def _setup_walk(night, pad, guard, plan, out: dict) -> None:
    """Before the first reload: V walks away from where the test save starts (next to a car: X would mount it)."""
    gn = _gn()
    sc = dataclasses.replace(night.scenarios['ranged_walk'], name='rf_setup_walk',
                             description='ranged families: walk away from the start place (unarmed)')
    night.scenarios[sc.name] = sc
    try:
        night.play_walking(pad, guard, plan, sc.name, clear=gn.commands_movement(sc))
        _tag_part(plan, sc.name, 'ranged_walk')
    except Exception as e:  # noqa: BLE001 - not worth the run
        if type(e).__name__ in ('ScenarioTimeout', 'StopNight', 'CycleError', 'EarlyStop', 'NeedsKeyboard'):
            raise
        out['warnings'].append(f'setup walk failed: {type(e).__name__}: {str(e)[:100]}')


def run_ranged_macro(night, pad, guard, plan, set_name: str, out: dict | None = None) -> dict:
    """The macro of the set. `out` (the scenario record) is filled while it runs, so the windows and evidence of finished families
    survive a timeout; returns `out`."""
    fams, records = options_of(getattr(night, 'args', None))
    out = {} if out is None else out
    for key, empty in (('families', {}), ('fails', []), ('warnings', []), ('phase_windows', [])):
        out.setdefault(key, empty)
    out['unverified_records'] = unverified_records()
    orig = _orig_hand(night, pad, guard)
    if orig is None:
        out['warnings'].append('the weapon in the hand before the set could not be read: it is not drawn again at the end')
    try:
        if fams:
            _setup_walk(night, pad, guard, plan, out)
        for tag in fams:
            r = None
            near = getattr(night, 'keep_near_home', None)     # nothing in the hand here: the place to put V back
            if near is not None:
                near(pad, guard, plan, f'before {tag}')
            try:
                r = run_one(night, pad, guard, plan, tag, records[tag], out, run_no=int(out.get('run', 1) or 1))
            finally:
                view = out['families'][tag]
                out['fails'] += [f'{tag}: {f}' for f in view['fails']]
                out['warnings'] += [f'{tag}: {x}' for x in view['warnings']]
                out['phase_windows'] += view['phase_windows']
            night.log(f'{LOG_PREFIX} {tag}: {r["verdict"]}' + (f': {"; ".join(r["fails"])}' if r['fails'] else '')
                      + (f' ({r["skip_reason"]})' if r['skip_reason'] else ''))
            secure = getattr(night, 'secure_session_files', None)
            if secure is not None:
                secure('between families')
    finally:
        try:
            _restore_hand(night, pad, guard, out, orig)
        except Exception as e:  # noqa: BLE001 - the set result must survive
            out['warnings'].append(f'the original weapon was not restored: {type(e).__name__}: {str(e)[:100]}')
    verdicts = [r['verdict'] for r in out['families'].values()]
    out['verdict'] = 'FAIL' if 'FAIL' in verdicts else 'SKIP' if not verdicts or all(v == 'SKIP' for v in verdicts) else 'PASS'
    return out


# ---------------------------------------------------------------------------------------------- second try of a part

FIRST_TRY_SUFFIX = '_first_try'
# what a part view keeps of the try that came first when a second try replaces it
FIRST_TRY_KEYS = ('verdict', 'skip_reason', 'fails', 'warnings', 'phases', 'phase_windows', 'invalid_phases', 'given', 'removed',
                  'ammo_removed', 'record', 'evidence', 'shots_planned', 'capacity')


def mark_first_try(windows) -> list:
    """Flag phase windows as those of a first try. The label gets FIRST_TRY_SUFFIX too: the sheets and the pose meter rows are
    named after the label and the run, so a second try with the same labels would otherwise take their names."""
    windows = list(windows or [])
    for w in windows:
        w['first_try'] = True
        label = w.get('label')
        if label and not label.endswith(FIRST_TRY_SUFFIX):
            w['label'] = label + FIRST_TRY_SUFFIX
    return windows


def keep_first_try_windows(first, new) -> list:
    """The phase windows of a record whose second try rebuilt them: the windows of the first try (marked) followed by the new ones."""
    first = mark_first_try(first)
    return first + [w for w in (new or []) if not any(w is f for f in first)]


def merge_second_try(rec: dict, tag: str, view: dict, key: str = 'families') -> dict:
    """Put the view of a second try of part `tag` into the macro record `rec` (`key` = 'families' or 'weapons') and keep what the first
    try recorded: its view moves to view['first_try'] (verdict, failures, phases and evidence), its phase windows stay in
    rec['phase_windows'] marked as first try. Returns the view."""
    views = rec.setdefault(key, {})
    old = views.get(tag)
    windows = list(rec.get('phase_windows') or [])
    new_windows = view.get('phase_windows') or []
    if old is not None and old is not view:
        view['first_try'] = {k: old[k] for k in FIRST_TRY_KEYS if k in old}
        mine = [w for w in windows if w.get('family', w.get('weapon')) == tag and not any(w is n for n in new_windows)]
        mine += [w for w in old.get('phase_windows') or [] if not any(w is m for m in mine)]
        mark_first_try(mine)
        windows += [w for w in mine if not any(w is x for x in windows)]
    view['second_try_of_invalid'] = True
    views[tag] = view
    rec['phase_windows'] = windows + [w for w in new_windows if not any(w is x for x in windows)]
    return view


# ---------------------------------------------------------------------------------------------- dry run

def dry_run_lines(tag: str, records=None, scenarios: dict | None = None) -> list[str]:
    fam = FAMILIES[tag]
    recs = tuple(records or fam['records'])
    ev = ', '.join(f'{r} [{record_evidence(r)}]' for r in recs)
    lines = [f'== {SET_NAME}: {fam["label"]} ({tag}), item type {" / ".join(fam["types"])}',
             f'   records (first the game knows with the right type is used): {ev}',
             f'   pick    : {pick_cmd(fam, recs)}',
             f'   give    : {give_cmd(fam, recs[0])}  (record from the pick echo)',
             f'   hand    : {hand_cmd(recs[0])}']
    shots = plan_shots(None)
    for ph in phases_of(tag):
        if ph in ('draw', 'holster'):
            continue
        if ph in MOVING_PARTS:
            sc = (scenarios or {}).get(MOVING_PARTS[ph])
            lines.append(f'   {ph:10}: static pad part {MOVING_PARTS[ph]}' + (f', {sc.duration:.1f} s of pad input' if sc else '')
                         + f' (family copy {phase_label(tag, ph)})')
        else:
            sc = scenario_for(tag, ph, shots)
            lines.append(f'   {ph:10}: pad scenario {sc.name}, {sc.duration:.1f} s of pad input'
                         + (f'; {BASH_BUTTON} tap = quick melee, seen in the log as MMGATE melee={BASH_MELEE_BIT:#x}, no limit' if ph == 'bash' else ''))
    lines.append(f'   holster : {holster_cmd()}')
    lines.append(f'   remove  : {remove_cmd(recs[0], 0)}  ;  {ammo_off_cmd(fam, recs[0], 0)}')
    lines.append(f'   pad time ~{pad_seconds(tag, scenarios):.0f} s; phases: ' + ' -> '.join(phase_plan(tag)))
    return lines


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dry-run', action='store_true', help='print the commands and pad scenarios of each family')
    ap.add_argument('--families', default=','.join(FAMILIES), help='comma list of family tags')
    ap.add_argument('--records', action='append', default=[], help='family=Items.X[,Items.Y] (repeatable)')
    a = ap.parse_args(argv)
    try:
        fams = parse_families(a.families)
        recs = parse_records(a.records)
    except ValueError as e:
        ap.error(str(e))
    from gt_scenario import load_scenarios
    scenarios = load_scenarios(gw.SCENARIO_FILE)
    missing = [p for p in PARTS if p not in scenarios]
    for t in fams:
        print('\n'.join(dry_run_lines(t, recs.get(t), scenarios)))
    if missing:
        print(f'MISSING pad parts in scenarios.json: {missing}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
