-- Dual-wield D0: utpp_probe DualProbe / DualProbeEvery (offline, mocks only). UTPP_PROBE_PATH is set by run_lua_tests.py.
local events, logs = {}, {}
registerForEvent = function(n, f) events[n] = f end
local ints = { UpperBody = 7, RightWeapon = 8, LeftWeapon = 5, ShootState = 2 }
local bools = { IsDualWielding = true }
local defs = { PlayerStateMachine = { UpperBody = 'UpperBody', RightWeapon = 'RightWeapon', LeftWeapon = 'LeftWeapon',
                                      IsDualWielding = 'IsDualWielding', ShootState = 'ShootState' } }
GetAllBlackboardDefs = function() return defs end
local items = { ['AttachmentSlots.WeaponRight'] = { id = 'Items.Preset_Nue_Default', ent = '111' },
                ['AttachmentSlots.WeaponLeft'] = { id = 'Items.Preset_Yukimura_Default', ent = '222' } }
local function itemObj(d)
  return { GetItemID = function() return d.id end, GetEntityID = function() return d.ent end }
end
local player = { GetEntityID = function() return 'player' end }
Game = {
  UTPPLogLine = function(s) logs[#logs + 1] = s end,
  GetPlayer = function() return player end,
  GetBlackboardSystem = function()
    return { GetLocalInstanced = function()
      return { GetInt = function(_, k) return ints[k] end, GetBool = function(_, k) return bools[k] end }
    end }
  end,
  GetTransactionSystem = function()
    return { GetItemInSlot = function(_, _, slot) local d = items[slot]; return d and itemObj(d) or nil end }
  end,
}
TweakDBID = { new = function(s) return s end }
TDBID = { ToStringDEBUG = function(x) return x end }
ItemID = { GetTDBID = function(x) return x end }
WeaponObject = { GetWeaponType = function(id) return id == 'Items.Preset_Nue_Default' and 'Wea_Handgun' or 'Wea_Handgun' end }
EntityID = { ToDebugStringDecimal = function(x) return x end }
local enums = {
  gamePSMUpperBodyStates = { [6] = 'Aim', [7] = 'DualWield' },
  gamePSMShootStates = { [0] = 'RightWeapon', [1] = 'LeftWeapon', [2] = 'BothWeapons', [3] = 'ValentinosFrenzyInit' },
  gamePSMRangedWeaponStates = { [5] = 'Ready', [8] = 'Shoot' },
}
EnumValueToString = function(e, v)
  local t = enums[e]
  if t and t[v] then return t[v] end
  error('no name')
end
local realPrint = print
print = function() end

local P = dofile(UTPP_PROBE_PATH)
P.LOG_FILE = 'NUL' -- fileLog must not create probe_dump.log in the working directory
local n = 0
local function check(x, what) n = n + 1; assert(x, what or ('check ' .. n)) end

local line = P.DualProbe()
check(type(line) == 'string' and line:find('[utpp_dual] probe ', 1, true) == 1, 'prefix: ' .. tostring(line))
check(line:find('upper=7(DualWield)', 1, true), 'upper')
check(line:find('IsDualWielding=true', 1, true), 'flag')
check(line:find('shootState=2(BothWeapons)', 1, true), 'shoot')
check(line:find('shootField=ShootState', 1, true), 'shootField')
check(line:find('rw=8(Shoot)', 1, true) and line:find('lw=5(Ready)', 1, true), 'rw/lw')
check(line:find('R=Items.Preset_Nue_Default|Wea_Handgun|111', 1, true), 'right item')
check(line:find('L=Items.Preset_Yukimura_Default|Wea_Handgun|222', 1, true), 'left item')
check(line:find('toggle=n/a', 1, true) and line:find('apiDual=?', 1, true), 'toggle / api')
check(line:find('shootEnum=0:RightWeapon,1:LeftWeapon,2:BothWeapons,3:ValentinosFrenzyInit', 1, true), 'enum table')
check(logs[#logs] == line, 'logged to the UTPP log without a second prefix')

-- CET form used by the other UTPP mods: Game.EnumValueToString wins over the bare global
Game.EnumValueToString = function(e, v) return 'G' .. EnumValueToString(e, v) end
local lg = P.DualProbe()
check(lg:find('upper=7(GDualWield)', 1, true), 'Game.EnumValueToString form')
Game.EnumValueToString = nil

-- empty left slot, failing enum names, missing ShootState field: "-" / "?" and no error
items['AttachmentSlots.WeaponLeft'] = nil
defs.PlayerStateMachine.ShootState = nil
ints.UpperBody = 1
bools.IsDualWielding = false
local l2 = P.DualProbe()
check(l2:find('L=-', 1, true) and l2:find('shootState=?(?)', 1, true) and l2:find('shootField=?', 1, true), 'degraded read')
check(l2:find('upper=1(?)', 1, true) and l2:find('IsDualWielding=false', 1, true), 'single weapon')

-- no player: one short line, nil
Game.GetPlayer = function() return nil end
local before = #logs
check(P.DualProbe() == nil and #logs == before + 1 and logs[#logs]:find('probe no player', 1, true), 'no player')
Game.GetPlayer = function() return player end

-- repeat from onUpdate
check(P.DualProbeEvery(0.5) == 0.5)
before = #logs
events.onUpdate(0.2); events.onUpdate(0.2)
check(#logs == before, 'no probe before the period')
events.onUpdate(0.2)
check(#logs == before + 1 and logs[#logs]:find('[utpp_dual] probe ', 1, true) == 1, 'one probe per period')
check(P.DualProbeEvery(nil) == 0)
before = #logs
for _ = 1, 10 do events.onUpdate(0.5) end
check(#logs == before, 'repeat off')
print = realPrint
io.write('PASS: utpp_probe dual-wield probe (' .. n .. ' checks)\n')
