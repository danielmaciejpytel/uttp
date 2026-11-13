-- UTPP 0.3.7 offline check: CET load-time wiring of utpp_bodyturn and utpp_vehiclecam.
-- In game (0.3.1-0.3.6) neither mod registered its events: the load-time guard required type(GetMod) == "function"
-- when init.lua ran, which was false in CET (scripting.log showed only "Mod ... loaded!", never "wrapped" / "FAILED").
-- This test loads each init.lua in a CET-like environment where registerForEvent exists but GetMod is defined only
-- AFTER the file has been loaded, then runs onInit / onUpdate and checks that:
--   * onInit and onUpdate are registered at load;
--   * an "onInit" status line is printed once and also reaches Game.UTPPLogLine;
--   * a negative control (the old guard) registers nothing.
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. detail) or ""))
end

local savedPrint, savedRegister, savedGetMod, savedGame = print, registerForEvent, GetMod, Game

local function loadInCetEnv(path, getModAfterLoad)
  local handlers, lines = {}, {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  GetMod = nil
  local logged = {}
  -- 0.3.8: status lines must also reach the native UTPP log (Game.UTPPLogLine)
  Game = setmetatable({ UTPPLogLine = function(msg) logged[#logged + 1] = msg end },
                      { __index = function() return function() return nil end end })
  lines.logged = logged
  print = function(...)
    local parts = {}
    for i = 1, select("#", ...) do parts[#parts + 1] = tostring(select(i, ...)) end
    lines[#lines + 1] = table.concat(parts, " ")
  end
  local ok, mod = pcall(dofile, path)
  GetMod = getModAfterLoad -- defined only after the file was loaded (CET-like)
  if ok and handlers.onInit then pcall(handlers.onInit) end
  for _ = 1, 310 do
    if ok and handlers.onUpdate then pcall(handlers.onUpdate, 1 / 60) end
  end
  print, registerForEvent, GetMod, Game = savedPrint, savedRegister, savedGetMod, savedGame
  return ok, mod, handlers, lines
end

local function count(lines, needle)
  local n = 0
  for _, l in ipairs(lines) do
    if string.find(l, needle, 1, true) then n = n + 1 end
  end
  return n
end

for _, m in ipairs({ { name = "utpp_bodyturn", path = UTPP_BODYTURN_PATH }, { name = "utpp_vehiclecam", path = UTPP_VEHICLECAM_PATH } }) do
  local tag = "[" .. m.name .. "]"
  -- GetMod defined after load returns nil
  local ok, mod, handlers, lines = loadInCetEnv(m.path, function() return nil end)
  check(m.name .. ": loads", ok and type(mod) == "table", ok and nil or tostring(mod))
  check(m.name .. ": onInit / onUpdate registered although GetMod is defined only after load",
        type(handlers.onInit) == "function" and type(handlers.onUpdate) == "function")
  check(m.name .. ": onInit status line printed once", count(lines, tag .. " onInit:") == 1, table.concat(lines, " | "))
  check(m.name .. ": status lines also sent to Game.UTPPLogLine (0.3.8)",
        count(lines.logged, tag .. " onInit:") == 1, table.concat(lines.logged, " | "))
end

-- 0.5.0: utpp_core in the same CET-like environment (GetMod only after load, Native Settings missing, no player).
if UTPP_CORE_PATH then
  local inputs = {}
  local savedInput = registerInput
  registerInput = function(id, label, fn) inputs[#inputs + 1] = id end
  local ok, mod, handlers, lines = loadInCetEnv(UTPP_CORE_PATH, function() return nil end)
  registerInput = savedInput
  check("utpp_core: loads", ok and type(mod) == "table" and mod.VERSION == "0.5.5", ok and nil or tostring(mod))
  check("utpp_core: onInit / onUpdate / onShutdown registered at load", type(handlers.onInit) == "function"
        and type(handlers.onUpdate) == "function" and type(handlers.onShutdown) == "function")
  -- 2026-10-30: the own key bindings were removed; utpp_core registers no CET input binding any more
  check("utpp_core: no CET input bindings registered while loading", #inputs == 0, table.concat(inputs, ","))
  check("utpp_core: one onInit status line also in Game.UTPPLogLine",
        count(lines, "[utpp_core] onInit 0.5.5: settings") == 1 and count(lines.logged, "[utpp_core] onInit 0.5.5") == 1,
        table.concat(lines, " | "))
  check("utpp_core: Native Settings missing reported once", count(lines, "Native Settings not found") == 1)
  check("utpp_core: no update errors without a player", count(lines, "update error") == 0)
  check("utpp_core: head look and loot assist on by default", count(lines, "headLook=true lootAssist=true") == 1,
        table.concat(lines, " | "))
end

-- Negative control: the pre-0.3.7 guard pattern registers nothing in the same environment.
do
  local handlers = {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  GetMod = nil
  local chunk = assert(loadstring([[
    if type(registerForEvent) == "function" and type(GetMod) == "function" then
      registerForEvent("onInit", function() end)
    end
  ]]))
  chunk()
  registerForEvent, GetMod = savedRegister, savedGetMod
  check("negative control: old guard registers no events when GetMod is defined after load", handlers.onInit == nil)
end

print(string.format("utpp CET load test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp CET load test failed")
