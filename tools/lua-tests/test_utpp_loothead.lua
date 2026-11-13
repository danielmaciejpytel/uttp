-- UTPP 5N.5 offline check of cet/utpp_loothead/init.lua (settings wiring, kill switch, fallback). Needs UTPP_LOOTHEAD_PATH.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. detail) or ""))
end

local savedPrint, savedRegister, savedGetMod, savedGame = print, registerForEvent, GetMod, Game

local function env(opts)
  local calls, handlers, lines = {}, {}, {}
  local function rec(name, ret)
    return function(...) calls[#calls + 1] = { name = name, args = { ... } }; return ret end
  end
  Game = {
    UTPPLogLine = function() end,
    UTPPLootRangeSetEnabled = rec("UTPPLootRangeSetEnabled"),
    UTPPLootRangeSetScale = function(v) calls[#calls + 1] = { name = "UTPPLootRangeSetScale", args = { v } }; return v >= 0.25 and v <= 2.5 end,
    UTPPHeadGuardSetEnabled = rec("UTPPHeadGuardSetEnabled"),
    UTPPLootRangeKill = rec("UTPPLootRangeKill"),
    UTPPHeadGuardKill = rec("UTPPHeadGuardKill"),
    UTPPLootRangeStatus = rec("UTPPLootRangeStatus", "LR status"),
    UTPPHeadGuardStatus = rec("UTPPHeadGuardStatus", "HG status"),
    UTPPInteractRangeSetEnabled = rec("UTPPInteractRangeSetEnabled"),
    UTPPInteractRangeSetScale = function(v) calls[#calls + 1] = { name = "UTPPInteractRangeSetScale", args = { v } }; return v >= 0.25 and v <= 2.5 end,
    UTPPInteractRangeKill = rec("UTPPInteractRangeKill"),
    UTPPInteractRangeStatus = rec("UTPPInteractRangeStatus", "IR status"),
  }
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  print = function(...) local t = {} for i = 1, select("#", ...) do t[#t + 1] = tostring(select(i, ...)) end lines[#lines + 1] = table.concat(t, " ") end
  local core = nil
  if opts.core then
    core = { defs = {}, actions = {} }
    function core.RegisterSection() end
    function core.Register(def)
      core.defs[def.key] = def
      local v = def.default
      if def.apply then def.apply(v) end
    end
  end
  GetMod = nil
  local ok, mod = pcall(dofile, UTPP_LOOTHEAD_PATH)
  GetMod = function(name) if name == "utpp_core" then return core end return nil end
  if ok and handlers.onInit then handlers.onInit() end
  print, registerForEvent, GetMod, Game = savedPrint, savedRegister, savedGetMod, savedGame
  return ok, mod, calls, lines, core
end

local function last(calls, name)
  for i = #calls, 1, -1 do if calls[i].name == name then return calls[i] end end
  return nil
end
local function count(calls, name)
  local n = 0
  for _, c in ipairs(calls) do if c.name == name then n = n + 1 end end
  return n
end

-- 1) utpp_core present: defaults on -> both enabled, scale 1.0 pushed, kill action registered.
local ok, mod, calls, lines, core = env({ core = true })
check("loads", ok and type(mod) == "table", ok and nil or tostring(mod))
check("registers lootRange.impl / lootRange.scale / headGuard.impl", core and core.defs["lootRange.impl"] and core.defs["lootRange.scale"]
  and core.defs["headGuard.impl"] and core.defs["lootRange.impl"].type == "bool"
  and core.defs["lootRange.impl"].default == true and core.defs["headGuard.impl"].default == true)
check("LootRange enabled via native", last(calls, "UTPPLootRangeSetEnabled") and last(calls, "UTPPLootRangeSetEnabled").args[1] == true)
check("HeadGuard enabled via native (native stays inert without a site)", last(calls, "UTPPHeadGuardSetEnabled")
  and last(calls, "UTPPHeadGuardSetEnabled").args[1] == true)
check("scale 1.0 pushed", last(calls, "UTPPLootRangeSetScale") and last(calls, "UTPPLootRangeSetScale").args[1] == 1.0)
check("no key action registered (bindings removed)", core and next(core.actions) == nil)
check("onInit line printed", (function() for _, l in ipairs(lines) do if l:find("settings via utpp_core ok", 1, true) then return true end end return false end)(),
  table.concat(lines, " | "))

-- Interaction reach: registered with a neutral label, default on, pushed to the natives once.
local ird = core and core.defs["interactRange.impl"]
check("registers interactRange.impl (bool, default on) / interactRange.scale", ird and ird.type == "bool" and ird.default == true
  and core.defs["interactRange.scale"] and core.defs["interactRange.scale"].default == 1.0
  and core.defs["interactRange.scale"].min == 0.5 and core.defs["interactRange.scale"].max == 2.0)
check("interactRange description is neutral and states FPP is unaffected", ird and type(ird.desc) == "string"
  and ird.desc:find("first person is unaffected", 1, true) ~= nil and ird.label == "Use UTPP interaction reach")
check("InteractRange enabled via native", last(calls, "UTPPInteractRangeSetEnabled") and last(calls, "UTPPInteractRangeSetEnabled").args[1] == true)
check("interact scale 1.0 pushed", last(calls, "UTPPInteractRangeSetScale") and last(calls, "UTPPInteractRangeSetScale").args[1] == 1.0)

-- Apply is idempotent; Kill calls all kill natives and drops the wanted state.
local n0 = count(calls, "UTPPLootRangeSetEnabled")
local ni0 = count(calls, "UTPPInteractRangeSetEnabled")
mod.Apply()
check("Apply without change = no native call", count(calls, "UTPPLootRangeSetEnabled") == n0 and count(calls, "UTPPInteractRangeSetEnabled") == ni0)
Game = { UTPPLogLine = function() end, UTPPLootRangeKill = function() calls[#calls + 1] = { name = "K1" } end,
         UTPPHeadGuardKill = function() calls[#calls + 1] = { name = "K2" } end,
         UTPPInteractRangeKill = function() calls[#calls + 1] = { name = "K3" } end }
print = function() end
local killed = mod.Kill()
print, Game = savedPrint, savedGame
check("Kill calls all three kill natives", killed == true and count(calls, "K1") == 1 and count(calls, "K2") == 1 and count(calls, "K3") == 1)
check("Kill clears wanted state", mod._state.lootOn == false and mod._state.headOn == false and mod._state.irOn == false)
check("SetScale range checked", mod.SetScale(0.3) == false and mod.SetScale(5) == false)
check("SetInteractScale range checked", mod.SetInteractScale(0.3) == false and mod.SetInteractScale(5) == false)
Game = { UTPPLogLine = function() end, UTPPInteractRangeSetVehicleMode = function(m) calls[#calls + 1] = { name = "VM", args = { m } }; return true end }
local vmOk = mod.SetVehicleMode(2)
local vmBad = mod.SetVehicleMode(7)
local vmAB = mod.SetVehicleMode(3)
local vmNeg = mod.SetVehicleMode(-1)
Game = savedGame
check("SetVehicleMode passes 0..3 to the native, rejects others", vmOk == true and vmBad == false and vmAB == true
  and vmNeg == false and last(calls, "VM").args[1] == 3 and count(calls, "VM") == 2)

-- 2) no utpp_core: fallback turns the loot range and the interaction reach on, head guard stays off.
local _, _, calls3 = env({ core = false })
check("fallback: LootRange on", last(calls3, "UTPPLootRangeSetEnabled") and last(calls3, "UTPPLootRangeSetEnabled").args[1] == true)
check("fallback: InteractRange on", last(calls3, "UTPPInteractRangeSetEnabled") and last(calls3, "UTPPInteractRangeSetEnabled").args[1] == true)
check("fallback: HeadGuard not enabled", last(calls3, "UTPPHeadGuardSetEnabled") == nil or last(calls3, "UTPPHeadGuardSetEnabled").args[1] == false)

print(string.format("test_utpp_loothead pass=%d fail=%d", results.pass, results.fail))
UTPP_LOOTHEAD_RESULT = results
