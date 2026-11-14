-- Offline check of the CET load-time wiring of utpp_body (same pattern as test_utpp_cet_load.lua): registerForEvent
-- exists while init.lua loads, GetMod only afterwards, no player in the world. Checks: events registered at load, one
-- onInit status line (also in Game.UTPPLogLine), settings registered in utpp_core when present,
-- the data table is found (or its absence reported), no update errors without a player, API usable before a player exists.
-- Globals: UTPP_BODY_PATH, UTPP_BODY_DATA_PATH, UTPP_CORE_PATH (optional).
local results = { pass = 0, fail = 0 }
local function check(name, ok, detail)
  if ok then results.pass = results.pass + 1 else results.fail = results.fail + 1 end
  print(string.format("[%s] %s%s", ok and "PASS" or "FAIL", name, detail and (" -- " .. tostring(detail)) or ""))
end

local savedPrint, savedRegister, savedGetMod, savedGame, savedData = print, registerForEvent, GetMod, Game, UTPP_BODY_DATA_PATH

local function count(lines, needle)
  local n = 0
  for _, l in ipairs(lines) do if string.find(l, needle, 1, true) then n = n + 1 end end
  return n
end

local function loadCore()
  if not UTPP_CORE_PATH then return nil end
  local r = registerForEvent
  registerForEvent = nil
  local core = dofile(UTPP_CORE_PATH)
  registerForEvent = r
  core.Save = function() return true end
  return core
end

-- getModAfterLoad(name) -> table|nil ; dataPath = UTPP_BODY_DATA_PATH for this run
local function loadInCetEnv(getModAfterLoad, dataPath, updates)
  local handlers, lines, logged = {}, {}, {}
  registerForEvent = function(ev, fn) handlers[ev] = fn end
  GetMod = nil
  UTPP_BODY_DATA_PATH = dataPath
  Game = setmetatable({ UTPPLogLine = function(msg) logged[#logged + 1] = msg end },
                      { __index = function() return function() return nil end end })
  print = function(...)
    local parts = {}
    for i = 1, select("#", ...) do parts[#parts + 1] = tostring(select(i, ...)) end
    lines[#lines + 1] = table.concat(parts, " ")
  end
  local ok, mod = pcall(dofile, UTPP_BODY_PATH)
  GetMod = getModAfterLoad
  local apiOk = true
  if ok and handlers.onInit then pcall(handlers.onInit) end
  if ok then apiOk = pcall(function() mod.SetTppBody(true, "load test"); mod.Status(); mod.GetState()
    mod.guard:SetWindow("unmount"); mod.HeadGuardStatus(); lines.probe = mod.HeadProbe(); assert(type(lines.probe) == "string") end) end
  for _ = 1, (updates or 120) do
    if ok and handlers.onUpdate then pcall(handlers.onUpdate, 1 / 60) end
  end
  if ok and handlers.onShutdown then pcall(handlers.onShutdown) end
  print, registerForEvent, GetMod, Game, UTPP_BODY_DATA_PATH = savedPrint, savedRegister, savedGetMod, savedGame, savedData
  lines.logged = logged
  return ok, mod, handlers, lines, apiOk
end

-- 1) utpp_core present
local core = loadCore()
local ok, mod, handlers, lines, apiOk = loadInCetEnv(function(n) if n == "utpp_core" then return core end end, savedData)
check("utpp_body: loads", ok and type(mod) == "table" and mod.VERSION == "0.1.11", ok and nil or tostring(mod))
check("utpp_body: onInit / onUpdate / onShutdown registered at load", type(handlers.onInit) == "function"
  and type(handlers.onUpdate) == "function" and type(handlers.onShutdown) == "function")
check("utpp_body: one onInit status line, also in Game.UTPPLogLine", count(lines, "[utpp_body] onInit 0.1.11") == 1
  and count(lines.logged, "[utpp_body] onInit 0.1.11") == 1, table.concat(lines, " | "))
check("utpp_body: head guard on by default (headGuard=true in the onInit line)", count(lines, "headGuard=true") == 1,
  table.concat(lines, " | "))
check("utpp_body: head guard window without a player -> opened, no slot writes, no errors",
  count(lines, "head guard open (unmount)") == 1 and mod.guard.stats.repairs == 0 and count(lines, "head guard error") == 0,
  table.concat(lines, " | "))
check("utpp_body: data table loaded", count(lines, "data gen_bodyparts.py") == 1 and mod.data ~= nil and mod.data.parts > 200)
if core then
  check("utpp_body: settings registered in utpp_core", count(lines, "settings via utpp_core") == 1
    and core.settings.defs["body.impl"] ~= nil and core.settings.sections.body ~= nil)
  check("utpp_body: enabled by default", count(lines, "enabled=true,") == 1, table.concat(lines, " | "))
end
check("utpp_body: API usable before a player exists", apiOk and mod.IsTppBody() == true)
check("utpp_body: HeadProbe reports the redscript bridge (0.1.4; unreachable in this mock -> FAILED, no error)",
  type(lines.probe) == "string" and string.find(lines.probe, "reds=FAILED bridge not reachable", 1, true) ~= nil, tostring(lines.probe))
check("utpp_body: no update errors without a player", count(lines, "update error") == 0, table.concat(lines, " | "))
check("utpp_body: frozen while there is no player (no writes)", mod.GetState().frozen == "noplayer"
  and mod.GetState().stats.activations == 0)
check("utpp_body 0.1.7: no vehicle window errors without a player", count(lines, "vehicle window error") == 0
  and count(lines, "head guard window") == 0, table.concat(lines, " | "))
check("utpp_body 0.1.8: utpp_vehiclecam not loaded -> hook still pending, no hook errors", mod.vcamHook.state == "pending"
  and mod.vcamHook.tries >= 2 and count(lines, "vehicle camera hook") == 0, mod.vcamHook.state .. " " .. mod.vcamHook.tries)
if core then
  check("utpp_body 0.1.7: setting body.shadowProxy registered (default off)", core.settings.defs["body.shadowProxy"] ~= nil
    and mod.Config().shadowProxy == false)
end
-- 0.1.7 adapter: api.vehicle() reads PSM Vehicle (blackboard) + Game.GetMountedVehicle with a CET-like Game mock
do
  local sG, sDefs = Game, GetAllBlackboardDefs
  local defs = { PlayerStateMachine = { Vehicle = "Vehicle" } }
  GetAllBlackboardDefs = function() return defs end
  local player = { GetEntityID = function() return 7 end }
  local mounted = {}
  Game = { GetPlayer = function() return player end,
    GetBlackboardSystem = function() return { GetLocalInstanced = function(_, id, d)
      return { GetInt = function(_, key) return (id == 7 and key == "Vehicle") and 4 or -1 end } end } end,
    GetMountedVehicle = function(p) return p == player and mounted.v or nil end }
  local api = mod.GameApi()
  local okA, a1 = pcall(api.vehicle)
  mounted.v = { name = "bike" }
  local okB, a2 = pcall(api.vehicle)
  Game, GetAllBlackboardDefs = sG, sDefs
  check("utpp_body 0.1.7 adapter: api.vehicle -> PSM state + mounted", okA and okB and a1.state == 4 and a1.mounted == false
    and a2.state == 4 and a2.mounted == true, tostring(a1) .. " " .. tostring(a2))
end

-- 2) utpp_core present, body.impl stored off -> disabled, requests refused
core = loadCore()
if core then core.settings.stored["body.impl"] = "false" end
ok, mod, handlers, lines = loadInCetEnv(function(n)
  if n == "utpp_core" then return core end
end, savedData)
check("utpp_body: stored off -> disabled", count(lines, "enabled=false,") == 1 and mod.IsTppBody() == false,
  table.concat(lines, " | "))
check("utpp_body: refused request logged once", count(lines, "SetTppBody(true) ignored") == 1)

-- 3) no utpp_core, data file not found -> reported, no crash
ok, mod, handlers, lines = loadInCetEnv(function() return nil end, "Z:\\missing\\body_parts.lua", 30)
check("utpp_body: without utpp_core -> defaults", count(lines, "defaults (utpp_core missing)") == 1, table.concat(lines, " | "))
check("utpp_body: missing data table reported in the onInit line", count(lines, "data MISSING") == 1 and mod.data == nil
  or (mod.data ~= nil and count(lines, "data gen_bodyparts.py") == 1), table.concat(lines, " | "))
check("utpp_body: missing data -> no update errors", count(lines, "update error") == 0)

-- 4) 0.1.5 late injection (log utpp-0.5.9-user-play 11:46:44.173: "attempt to index upvalue 'G' (a nil value)" for both
-- bridge routes). The chunk runs in a CET-like environment WITHOUT _G and WITHOUT Game / NewObject / GetPlayer /
-- GetSingleton; CET assigns them only AFTER the chunk ran. Variant "nogetfenv": getfenv not whitelisted; variant
-- "foreignfenv": getfenv gives a table without Game. QueueRep / RedsSlot / CreateNative are called WITHOUT env.
local function lateInjection(variant)
  local realG = _G
  local hidden = { _G = true, getfenv = true, Game = true, NewObject = true, GetPlayer = true, GetSingleton = true }
  local handlers = {}
  local env = setmetatable({ registerForEvent = function(ev, fn) handlers[ev] = fn end,
    print = function() end, UTPP_BODY_DATA_PATH = savedData },
    { __index = function(_, k) if hidden[k] then return nil end; return realG[k] end })
  if variant == "foreignfenv" then rawset(env, "getfenv", function() return {} end) end
  local chunk = assert(loadfile(UTPP_BODY_PATH))
  setfenv(chunk, env)
  local okL, mod = pcall(chunk)
  -- negative control = the 0.1.4 default built in the same environment at the same moment (load): getfenv source or _G,
  -- plus Game / NewObject / GetPlayer captured at load
  local old04 = setfenv(function()
    local src = nil
    if type(getfenv) == "function" then local ok, e = pcall(function() return getfenv(1) end); if ok then src = e end end
    if src == nil and type(_G) == "table" then src = _G end
    return src, { Game = Game, NewObject = NewObject, GetPlayer = GetPlayer }
  end, env)
  local oldSrc, oldCaptured = old04()
  local okBefore, errBefore = false, "not loaded"
  if okL then okBefore, errBefore = pcall(mod.QueueRep, true) end
  -- CET injects the globals now (after the chunk ran)
  local calls, created = {}, {}
  rawset(env, "Game", { UTPPQueueTppRepresentation = function(on) calls[#calls + 1] = on; return true end,
    UTPPTppHeadSlotState = function() return "tpp" end })
  rawset(env, "NewObject", function(n) created[#created + 1] = n; return { name = n } end)
  rawset(env, "GetPlayer", function() return nil end)
  local r = { okL = okL, mod = mod, okBefore = okBefore, errBefore = errBefore, calls = calls, created = created }
  if okL then
    r.okQ, r.how = pcall(mod.QueueRep, true)
    r.okQ2, r.how2 = pcall(mod.QueueRep, false)
    r.okS, r.slot, r.slotHow = pcall(mod.RedsSlot)
    r.okC, r.obj, r.objHow = pcall(mod.CreateNative, mod.EVENT_NAMES.activate)
    r.okOld, r.errOld = pcall(mod.QueueRep, true, mod.MakeEnv(oldSrc, oldCaptured))
    if handlers.onInit then pcall(handlers.onInit) end
    r.okP, r.probe = pcall(mod.HeadProbe)
  end
  return r
end
for _, variant in ipairs({ "nogetfenv", "foreignfenv" }) do
  local r = lateInjection(variant)
  local tag = "utpp_body late injection (" .. variant .. "): "
  check(tag .. "chunk loads without _G / Game", r.okL and type(r.mod) == "table" and r.mod.VERSION == "0.1.11", tostring(r.mod))
  check(tag .. "before injection QueueRep fails with a route list, not a crash", not r.okBefore
    and string.find(tostring(r.errBefore), "no route for the TPP representation", 1, true) ~= nil, tostring(r.errBefore))
  check(tag .. "QueueRep(true) without env -> Game.UTPPQueueTppRepresentation(true)", r.okQ
    and r.how == "Game.UTPPQueueTppRepresentation(true)" and r.calls[1] == true, tostring(r.how))
  check(tag .. "QueueRep(false) without env passes false", r.okQ2
    and r.how2 == "Game.UTPPQueueTppRepresentation(false)" and r.calls[2] == false, tostring(r.how2))
  check(tag .. "RedsSlot() without env reads the bridge", r.okS and r.slot == "tpp" and r.slotHow == "Game.UTPPTppHeadSlotState",
    tostring(r.slot))
  check(tag .. "CreateNative without env uses the injected NewObject", r.okC and r.objHow == "NewObject(gameActivateTPPRepresentationEvent)"
    and r.created[1] == "gameActivateTPPRepresentationEvent", tostring(r.objHow))
  check(tag .. "HeadProbe reports the bridge", r.okP and string.find(tostring(r.probe), "reds=tpp via Game.UTPPTppHeadSlotState", 1, true) ~= nil,
    tostring(r.probe))
  check(tag .. "negative control: the 0.1.4 capture-at-load default fails exactly like the log", not r.okOld
    and string.find(tostring(r.errOld), "Game.UTPPQueueTppRepresentation error attempt to index upvalue 'G' (a nil value)", 1, true) ~= nil
    and string.find(tostring(r.errOld), "UTPPBodyBridge error attempt to index upvalue 'G' (a nil value)", 1, true) ~= nil,
    tostring(r.errOld))
end

-- 5) 0.1.8: utpp_vehiclecam reachable only through GetMod after load -> its optional callback field onVehiclePerspective
-- is set at run time (first onUpdate); calls without a player do not raise
do
  local vc = { VERSION = "test" }
  local core5 = loadCore()
  local ok5, mod5, _, lines5 = loadInCetEnv(function(n)
    if n == "utpp_core" then return core5 elseif n == "utpp_vehiclecam" then return vc end
  end, savedData, 30)
  check("utpp_body 0.1.8: utpp_vehiclecam found at run time -> onVehiclePerspective set", ok5 and mod5.vcamHook.state == "attached"
    and vc.onVehiclePerspective == mod5.OnVehiclePerspective
    and count(lines5, "vehicle camera hook: utpp_vehiclecam.onVehiclePerspective set") == 1, table.concat(lines5, " | "))
  local okC1, r1 = pcall(vc.onVehiclePerspective, 0, 4)
  local okC2, r2 = pcall(vc.onVehiclePerspective, 4, 0)
  check("utpp_body 0.1.8: hook calls without a player do not raise", okC1 and okC2 and type(r1) == "boolean"
    and type(r2) == "boolean", tostring(r1) .. " " .. tostring(r2))
  check("utpp_body 0.1.8: no hook / update errors", count(lines5, "vehicle camera hook error") == 0
    and count(lines5, "update error") == 0, table.concat(lines5, " | "))
end

print(string.format("utpp_body CET load test: %d passed, %d failed", results.pass, results.fail))
assert(results.fail == 0, "utpp_body CET load test failed")
