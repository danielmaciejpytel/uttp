-- UTPP utpp_loothead - settings and kill switches for the native LootRange, HeadGuard and InteractRange
-- (UTPP.dll, scripts/utpp_loothead.reds). Standalone implementation.
--
-- Options (UTPP tab via utpp_core, saved in utpp_settings.txt):
--   lootRange.impl  "Use UTPP loot range"   on | off (default on; stored legacy "auto"/"on" = on)
--   lootRange.scale window size relative to the vanilla loot drop window (0.5 .. 2.0)
--   headGuard.impl  "Use UTPP head guard"   on | off (default on; site found by signature at load; FAILED -> inert, see Status)
--   interactRange.impl  "Use UTPP interaction reach"  on | off (default on): third person only - vehicle seats offer their
--                   prompt when V stands at the door, devices / doors / NPC talk use a screen-centre window
--   interactRange.scale  window size for devices / doors / talk (0.5 .. 2.0)
-- Console:
--   print(GetMod("utpp_loothead").Status())
--   print(GetMod("utpp_loothead").Probe(4.0))  -- interaction descriptors of the look-at object and of entities near V
--   GetMod("utpp_loothead").Kill()            -- all off for the session (vanilla restored immediately)
--   GetMod("utpp_loothead").SetLootRange(true|false) / SetScale(1.2) / SetHeadGuard(true|false)
--   GetMod("utpp_loothead").SetInteractRange(true|false) / SetInteractScale(1.2) / SetVehicleMode(0|1|2|3)

local M = { VERSION = "0.2.0" }
local S = { lootOn = false, scale = 1.0, headOn = false, irOn = false, irScale = 1.0, applied = {}, nativeOk = nil, log = {} }

-- no file system paths in the log: a Lua error text names the script file with its full path (user name included); a path in
-- the CET mods folder is cut to "mods\...", the game folder prefix is dropped, a user profile path becomes "~" (same rules as
-- utpp_core M.ScrubPaths)
local function scrubPaths(s)
  s = tostring(s)
  if not string.find(s, "%a:[\\/]") then return s end
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-([Mm]ods[\\/])", "%1")
  s = string.gsub(s, "%a:[\\/][^\r\n\"']-Cyberpunk 2077[\\/]", "")
  s = string.gsub(s, "%a:[\\/][Uu]sers[\\/][^\\/%s\"']+", "~")
  return (string.gsub(s, "%a:[\\/][^%s\"']*", "<path>"))
end

local function sink(msg)
  S.log[#S.log + 1] = msg
  if #S.log > 50 then table.remove(S.log, 1) end
  print(msg)
  pcall(function() if Game and Game.UTPPLogLine then Game.UTPPLogLine(scrubPaths(msg)) end end)
end
M._sink = sink

local function call(name, ...)
  if not Game then return false, "no Game" end
  local fn = Game[name]
  if type(fn) ~= "function" then return false, "missing " .. name end
  return pcall(fn, ...)
end
M._call = call

-- Pushes the wanted state to the natives (only on change).
function M.Apply()
  local a = S.applied
  if a.lootOn ~= S.lootOn then
    local ok, err = call("UTPPLootRangeSetEnabled", S.lootOn)
    if ok then a.lootOn = S.lootOn else S.nativeOk = false; sink("[utpp_loothead] UTPPLootRangeSetEnabled FAILED " .. tostring(err)) end
  end
  if a.scale ~= S.scale then
    local ok, res = call("UTPPLootRangeSetScale", S.scale)
    if ok and res ~= false then a.scale = S.scale else sink("[utpp_loothead] UTPPLootRangeSetScale(" .. tostring(S.scale) .. ") rejected") end
  end
  if a.headOn ~= S.headOn then
    local ok, err = call("UTPPHeadGuardSetEnabled", S.headOn)
    if ok then a.headOn = S.headOn else S.nativeOk = false; sink("[utpp_loothead] UTPPHeadGuardSetEnabled FAILED " .. tostring(err)) end
  end
  if a.irOn ~= S.irOn then
    local ok, err = call("UTPPInteractRangeSetEnabled", S.irOn)
    if ok then a.irOn = S.irOn else S.nativeOk = false; sink("[utpp_loothead] UTPPInteractRangeSetEnabled FAILED " .. tostring(err)) end
  end
  if a.irScale ~= S.irScale then
    local ok, res = call("UTPPInteractRangeSetScale", S.irScale)
    if ok and res ~= false then a.irScale = S.irScale else sink("[utpp_loothead] UTPPInteractRangeSetScale(" .. tostring(S.irScale) .. ") rejected") end
  end
end

function M.SetLootRange(on) S.lootOn = on and true or false; M.Apply() end
function M.SetHeadGuard(on) S.headOn = on and true or false; M.Apply() end
function M.SetInteractRange(on) S.irOn = on and true or false; M.Apply() end
function M.SetScale(v)
  v = tonumber(v)
  if not v or v < 0.5 or v > 2.0 then return false end
  S.scale = v; M.Apply(); return true
end
function M.SetInteractScale(v)
  v = tonumber(v)
  if not v or v < 0.5 or v > 2.0 then return false end
  S.irScale = v; M.Apply(); return true
end
-- Console only: 0 vanilla look-at for vehicle seats, 1 seat sphere around V (default), 2 screen-centre window,
-- 3 seat sphere without the classification list change (A/B check).
function M.SetVehicleMode(m)
  m = tonumber(m)
  if m ~= 0 and m ~= 1 and m ~= 2 and m ~= 3 then return false end
  local ok, res = call("UTPPInteractRangeSetVehicleMode", m)
  return ok and res ~= false
end

function M.Kill()
  local ok1 = call("UTPPLootRangeKill")
  local ok2 = call("UTPPHeadGuardKill")
  local ok3 = call("UTPPInteractRangeKill")
  S.lootOn, S.headOn, S.irOn = false, false, false
  sink("[utpp_loothead] Kill(): LootRange " .. (ok1 and "killed" or "kill FAILED") .. ", HeadGuard " .. (ok2 and "killed" or "kill FAILED")
    .. ", InteractRange " .. (ok3 and "killed" or "kill FAILED"))
  return ok1 and ok2 and ok3
end

function M.Status()
  local _, lr = call("UTPPLootRangeStatus")
  local _, hg = call("UTPPHeadGuardStatus")
  local _, ir = call("UTPPInteractRangeStatus")
  return string.format("utpp_loothead %s want lootRange=%s scale=%.2f headGuard=%s interactRange=%s interactScale=%.2f\n%s\n%s\n%s",
    M.VERSION, tostring(S.lootOn), S.scale, tostring(S.headOn), tostring(S.irOn), S.irScale, tostring(lr), tostring(hg), tostring(ir))
end

-- Read-only console probe (redscript UTPPInteractProbe in utpp_loothead.reds).
function M.Probe(range)
  local ok, res = call("UTPPInteractProbe", tonumber(range) or 4.0)
  return ok and tostring(res) or ("UTPPInteractProbe FAILED " .. tostring(res))
end

local function register(core)
  core.RegisterSection("loot", "Loot")
  core.Register({ key = "lootRange.impl", section = "loot", type = "bool", default = true, label = "Use UTPP loot range",
    desc = "In third person the loot prompt of containers and bodies follows the camera (screen-centre window, as for "
      .. "dropped items). Reach around V is unchanged.",
    apply = function(v) M.SetLootRange(v) end })
  core.Register({ key = "lootRange.scale", section = "loot", type = "float", default = 1.0, min = 0.5, max = 2.0, step = 0.05,
    format = "%.2f", label = "Loot window size", desc = "Size of the screen-centre window relative to the game's dropped-item window.",
    apply = function(v) M.SetScale(v) end })
  core.RegisterSection("headguard", "Head near camera")
  core.Register({ key = "headGuard.impl", section = "headguard", type = "bool", default = true, label = "Use UTPP head guard",
    desc = "Keeps the head visible when the third-person camera is close. Inert when the game's patch site does not "
      .. "match (see Status in the console).", apply = function(v) M.SetHeadGuard(v) end })
  core.RegisterSection("interact", "Interactions")
  core.Register({ key = "interactRange.impl", section = "interact", type = "bool", default = true,
    label = "Use UTPP interaction reach",
    desc = "In third person, vehicle doors and seats offer their prompt when V stands next to them, and doors, terminals "
      .. "and conversations react to the screen centre instead of an exact look-at line. Reach around V is unchanged; "
      .. "first person is unaffected.",
    apply = function(v) M.SetInteractRange(v) end })
  core.Register({ key = "interactRange.scale", section = "interact", type = "float", default = 1.0, min = 0.5, max = 2.0,
    step = 0.05, format = "%.2f", label = "Interaction window size",
    desc = "Size of the screen-centre window for doors, terminals and conversations, relative to the game's dropped-item window.",
    apply = function(v) M.SetInteractScale(v) end })
end

registerForEvent("onInit", function()
  local core = GetMod and GetMod("utpp_core") or nil
  if type(core) == "table" and type(core.Register) == "function" then
    local ok, err = pcall(register, core)
    sink("[utpp_loothead] onInit " .. M.VERSION .. ": settings via utpp_core " .. (ok and "ok" or ("FAILED " .. tostring(err))))
    if ok then return end
  end
  -- Fallback without utpp_core: loot range and interaction reach on; head guard stays off.
  S.lootOn = true
  S.irOn = true
  M.Apply()
  sink("[utpp_loothead] onInit " .. M.VERSION .. ": utpp_core not found, lootRange=" .. tostring(S.lootOn)
    .. " interactRange=" .. tostring(S.irOn))
end)

M._state = S
return M
